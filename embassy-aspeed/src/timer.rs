//! Hardware timer driver: 8 × 32-bit countdown timers.
//!
//! The embassy SysTick time driver (see `time_driver.rs`) drives `embassy_time`
//! timers.  These 8 hardware timers are for application use cases that need a
//! hardware interrupt at a precise period without consuming the SysTick.
//!
//! # Timer numbering
//!
//! Timers are numbered 1–8.  Timer 1 (IRQ 16) through Timer 8 (IRQ 23).
//!
//! # Clock source
//!
//! Each timer can use either PCLK (APB1 clock) or a fixed 1 MHz source.
//! Using 1 MHz simplifies period calculations: RELOAD = period_us − 1.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::timer::CountdownTimer;
//!
//! let mut t = CountdownTimer::new(1); // timer 1
//! t.start(1_000_000); // 1 second period (1 MHz clock → 1_000_000 µs)
//! t.wait().await;
//! ```

use core::future::Future;
use core::pin::Pin;
use core::ptr;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

// ── Register addresses ────────────────────────────────────────────────────────

const TIMER_BASE: usize = 0x7E78_2000;

// Per-timer STATUS register offsets (word index = byte_offset / 4).
// Timer N status is at: base + (N-1) * 0x10 if N <= 3, or base + ((N-1)*0x10 + 0x10) if N >= 4
// Actually timers 1-3 at 0x00/0x10/0x20, timers 4-8 skip the 0x30 control gap:
const TIMER_STATUS_OFF: [usize; 8] = [
    0x00 / 4, // T1
    0x10 / 4, // T2
    0x20 / 4, // T3
    0x40 / 4, // T4
    0x50 / 4, // T5
    0x60 / 4, // T6
    0x70 / 4, // T7
    0x80 / 4, // T8
];

const TMC_CTRL: *mut u32 = (TIMER_BASE + 0x30) as *mut u32;
const TMC_INT_STATUS: *mut u32 = (TIMER_BASE + 0x34) as *mut u32;
const TMC_CTRL_CLR: *mut u32 = (TIMER_BASE + 0x3C) as *mut u32;

// ── Global wakers (one per timer) ─────────────────────────────────────────────

static TIMER_WAKERS: [AtomicWaker; 8] = [
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
];

// ── CountdownTimer ────────────────────────────────────────────────────────────

/// Hardware countdown timer (1-based, 1–8).
pub struct CountdownTimer {
    /// Timer number (1–8). Stored as 0-based index internally.
    idx: u8,
}

impl CountdownTimer {
    /// Create a timer for `n` (1–8).
    ///
    /// # Panics
    ///
    /// Panics if `n` is not 1–8.
    pub fn new(n: u8) -> Self {
        assert!(n >= 1 && n <= 8, "timer number must be 1–8");
        Self { idx: n - 1 }
    }

    /// Start the timer with a `period_us` period (1 MHz clock).
    ///
    /// The timer overflows (fires the interrupt) every `period_us` microseconds.
    pub fn start(&mut self, period_us: u32) {
        let base = TIMER_BASE as *mut u32;
        let status_off = TIMER_STATUS_OFF[self.idx as usize];
        let reload_off = status_off + (0x04 / 4);

        // SAFETY: Timer MMIO writes.
        unsafe {
            ptr::write_volatile(base.add(reload_off), period_us);
            // Enable: set EN bit and CLK_SEL=1 (1 MHz) and OVF_INTR=1.
            // Bit layout: [EN, CLK_SEL, OVF_INTR, WDT_EN] at bits idx*4 + [0,1,2,3].
            let ctrl_bits = 0b0111u32 << (self.idx * 4); // EN|CLK_SEL|OVF_INTR
                                                         // Read-modify-write: TMC_CTRL controls all 8 timers in one register.
                                                         // Overwriting the whole register would stop every other running timer.
            let prev = ptr::read_volatile(TMC_CTRL);
            ptr::write_volatile(TMC_CTRL, prev | ctrl_bits);
        }
    }

    /// Stop the timer.
    pub fn stop(&mut self) {
        let ctrl_bits = 0b1111u32 << (self.idx * 4); // clear all 4 bits
                                                     // SAFETY: Timer MMIO write.
        unsafe { ptr::write_volatile(TMC_CTRL_CLR, ctrl_bits) };
    }

    /// Async wait for the next timer overflow interrupt.
    pub fn wait(&mut self) -> TimerWait<'_> {
        TimerWait { timer: self }
    }

    /// Called from ISR for timer `n` (1-based).
    pub(crate) fn on_interrupt(n: u8) {
        let bit = 1u32 << (n - 1);
        // Clear interrupt status.
        // SAFETY: Timer ISR.
        unsafe { ptr::write_volatile(TMC_INT_STATUS, bit) };
        TIMER_WAKERS[(n - 1) as usize].wake();
    }
}

// ── TimerWait future ──────────────────────────────────────────────────────────

/// Future returned by [`CountdownTimer::wait`].
pub struct TimerWait<'a> {
    timer: &'a mut CountdownTimer,
}

impl<'a> Future for TimerWait<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let idx = self.timer.idx as usize;
        let bit = 1u32 << idx;

        // Check if already fired.
        // SAFETY: reading interrupt status register.
        let status = unsafe { ptr::read_volatile(TMC_INT_STATUS) };
        if status & bit != 0 {
            unsafe { ptr::write_volatile(TMC_INT_STATUS, bit) };
            return Poll::Ready(());
        }

        TIMER_WAKERS[idx].register(cx.waker());

        // Re-check after registration.
        let status = unsafe { ptr::read_volatile(TMC_INT_STATUS) };
        if status & bit != 0 {
            unsafe { ptr::write_volatile(TMC_INT_STATUS, bit) };
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// ── Interrupt handlers ────────────────────────────────────────────────────────

macro_rules! timer_irq {
    ($name:ident, $n:expr) => {
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn $name() {
            CountdownTimer::on_interrupt($n);
        }
    };
}

timer_irq!(TIMER1, 1);
timer_irq!(TIMER2, 2);
timer_irq!(TIMER3, 3);
timer_irq!(TIMER4, 4);
timer_irq!(TIMER5, 5);
timer_irq!(TIMER6, 6);
timer_irq!(TIMER7, 7);
timer_irq!(TIMER8, 8);
