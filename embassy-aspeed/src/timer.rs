//! Hardware countdown timer driver (AST1060) — PAC-based.
//!
//! Source of truth: `aspeed-data/data/registers/timer_v1.yaml`.
//!
//! 8 × 32-bit countdown timers at `0x7E78_2000`.
//! Each timer counts down from `RELOAD` to zero, fires an interrupt, and
//! reloads automatically.
//!
//! # Clock source
//!
//! Each timer can use PCLK (`CLK_SEL=0`) or a fixed 1 MHz source (`CLK_SEL=1`).
//! Using 1 MHz simplifies period calculation: `RELOAD = period_µs − 1`.
//!
//! # Control register model
//!
//! `TIMER.CTRL` (TMC30) is a **set** register: write 1 to set bits.
//! `TIMER.CTRL_CLR` (TMC3C) is the **clear** register: write 1 to clear bits.
//! This allows safe read-modify-write per-timer without disturbing other timers.
//!
//! # Async model
//!
//! `CountdownTimer::wait()` returns a `TimerWait` future.  The per-timer ISR
//! clears `INT_STATUS` and wakes the `AtomicWaker`; the future resolves on the
//! next poll.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::timer::CountdownTimer;
//!
//! let mut t = CountdownTimer::new(1);      // timer 1
//! t.start(1_000_000);                      // 1 s at 1 MHz
//! t.wait().await;                          // await one period
//! ```

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::pac;

// ── Global wakers ──────────────────────────────────────────────────────────────

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

/// Hardware countdown timer (1-indexed, 1–8).
pub struct CountdownTimer {
    /// Timer index 0-based internally (timer number - 1).
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

    /// Start the timer at a period of `period_us` microseconds (1 MHz clock).
    ///
    /// Uses the 1 MHz fixed clock source.  Each overflow fires the interrupt.
    pub fn start(&mut self, period_us: u32) {
        // Write reload value for this timer.
        set_reload(self.idx, period_us);
        // Set EN=1, CLK_SEL=1 (1 MHz), OVF_INTR=1 via the set register.
        set_ctrl_bits(self.idx, true, true, true);
    }

    /// Stop the timer.
    pub fn stop(&mut self) {
        // Clear EN, CLK_SEL, OVF_INTR, WDT_EN by writing to CTRL_CLR.
        clear_ctrl_bits(self.idx);
        let _ = pac::TIMER; // suppress unused warning
    }

    /// Async wait for the next timer overflow interrupt.
    pub fn wait(&mut self) -> TimerWait<'_> {
        TimerWait { timer: self }
    }

    /// Called from ISR for timer `n` (1-based).
    pub(crate) fn on_interrupt(n: u8) {
        // Clear the interrupt status bit for this timer (RW1C).
        let idx = n - 1;
        clear_int_status(idx);
        TIMER_WAKERS[idx as usize].wake();
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

        if read_int_status(self.timer.idx) {
            clear_int_status(self.timer.idx);
            return Poll::Ready(());
        }

        TIMER_WAKERS[idx].register(cx.waker());

        // Re-check after registration (close the check→register race).
        if read_int_status(self.timer.idx) {
            clear_int_status(self.timer.idx);
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// ── Register helpers (match on 0-based index) ─────────────────────────────────
//
// The PAC exposes per-timer named registers (T1_RELOAD, T2_RELOAD, …) rather
// than an array, so indexed access requires a match.  Each arm calls the
// appropriate typed register accessor from `timer_v1::TIMER`.

fn set_reload(idx: u8, val: u32) {
    let t = pac::TIMER;
    match idx {
        0 => t.T1_RELOAD().write(|w| w.set_VALUE(val)),
        1 => t.T2_RELOAD().write(|w| w.set_VALUE(val)),
        2 => t.T3_RELOAD().write(|w| w.set_VALUE(val)),
        3 => t.T4_RELOAD().write(|w| w.set_VALUE(val)),
        4 => t.T5_RELOAD().write(|w| w.set_VALUE(val)),
        5 => t.T6_RELOAD().write(|w| w.set_VALUE(val)),
        6 => t.T7_RELOAD().write(|w| w.set_VALUE(val)),
        7 => t.T8_RELOAD().write(|w| w.set_VALUE(val)),
        _ => {}
    }
}

/// Write to CTRL (set register): enable EN, optionally CLK_SEL, OVF_INTR.
fn set_ctrl_bits(idx: u8, en: bool, clk_sel: bool, ovf_intr: bool) {
    let t = pac::TIMER;
    match idx {
        0 => t.CTRL().write(|w| {
            w.set_T1_EN(en);
            w.set_T1_CLK_SEL(clk_sel);
            w.set_T1_OVF_INTR(ovf_intr);
        }),
        1 => t.CTRL().write(|w| {
            w.set_T2_EN(en);
            w.set_T2_CLK_SEL(clk_sel);
            w.set_T2_OVF_INTR(ovf_intr);
        }),
        2 => t.CTRL().write(|w| {
            w.set_T3_EN(en);
            w.set_T3_CLK_SEL(clk_sel);
            w.set_T3_OVF_INTR(ovf_intr);
        }),
        3 => t.CTRL().write(|w| {
            w.set_T4_EN(en);
            w.set_T4_CLK_SEL(clk_sel);
            w.set_T4_OVF_INTR(ovf_intr);
        }),
        4 => t.CTRL().write(|w| {
            w.set_T5_EN(en);
            w.set_T5_CLK_SEL(clk_sel);
            w.set_T5_OVF_INTR(ovf_intr);
        }),
        5 => t.CTRL().write(|w| {
            w.set_T6_EN(en);
            w.set_T6_CLK_SEL(clk_sel);
            w.set_T6_OVF_INTR(ovf_intr);
        }),
        6 => t.CTRL().write(|w| {
            w.set_T7_EN(en);
            w.set_T7_CLK_SEL(clk_sel);
            w.set_T7_OVF_INTR(ovf_intr);
        }),
        7 => t.CTRL().write(|w| {
            w.set_T8_EN(en);
            w.set_T8_CLK_SEL(clk_sel);
            w.set_T8_OVF_INTR(ovf_intr);
        }),
        _ => {}
    }
}

/// Write to CTRL_CLR: clear all 4 control bits for this timer.
fn clear_ctrl_bits(idx: u8) {
    let t = pac::TIMER;
    match idx {
        0 => t.CTRL_CLR().write(|w| {
            w.set_T1_EN(true);
            w.set_T1_CLK_SEL(true);
            w.set_T1_OVF_INTR(true);
            w.set_T1_WDT_EN(true);
        }),
        1 => t.CTRL_CLR().write(|w| {
            w.set_T2_EN(true);
            w.set_T2_CLK_SEL(true);
            w.set_T2_OVF_INTR(true);
            w.set_T2_WDT_EN(true);
        }),
        2 => t.CTRL_CLR().write(|w| {
            w.set_T3_EN(true);
            w.set_T3_CLK_SEL(true);
            w.set_T3_OVF_INTR(true);
            w.set_T3_WDT_EN(true);
        }),
        3 => t.CTRL_CLR().write(|w| {
            w.set_T4_EN(true);
            w.set_T4_CLK_SEL(true);
            w.set_T4_OVF_INTR(true);
            w.set_T4_WDT_EN(true);
        }),
        4 => t.CTRL_CLR().write(|w| {
            w.set_T5_EN(true);
            w.set_T5_CLK_SEL(true);
            w.set_T5_OVF_INTR(true);
            w.set_T5_WDT_EN(true);
        }),
        5 => t.CTRL_CLR().write(|w| {
            w.set_T6_EN(true);
            w.set_T6_CLK_SEL(true);
            w.set_T6_OVF_INTR(true);
            w.set_T6_WDT_EN(true);
        }),
        6 => t.CTRL_CLR().write(|w| {
            w.set_T7_EN(true);
            w.set_T7_CLK_SEL(true);
            w.set_T7_OVF_INTR(true);
            w.set_T7_WDT_EN(true);
        }),
        7 => t.CTRL_CLR().write(|w| {
            w.set_T8_EN(true);
            w.set_T8_CLK_SEL(true);
            w.set_T8_OVF_INTR(true);
            w.set_T8_WDT_EN(true);
        }),
        _ => {}
    }
}

/// Read the interrupt status bit for this timer index (0-based).
fn read_int_status(idx: u8) -> bool {
    let s = pac::TIMER.INT_STATUS().read();
    match idx {
        0 => s.T1_INT(),
        1 => s.T2_INT(),
        2 => s.T3_INT(),
        3 => s.T4_INT(),
        4 => s.T5_INT(),
        5 => s.T6_INT(),
        6 => s.T7_INT(),
        7 => s.T8_INT(),
        _ => false,
    }
}

/// Write 1 to INT_STATUS to clear the interrupt for this timer (RW1C).
fn clear_int_status(idx: u8) {
    let t = pac::TIMER;
    match idx {
        0 => t.INT_STATUS().write(|w| w.set_T1_INT(true)),
        1 => t.INT_STATUS().write(|w| w.set_T2_INT(true)),
        2 => t.INT_STATUS().write(|w| w.set_T3_INT(true)),
        3 => t.INT_STATUS().write(|w| w.set_T4_INT(true)),
        4 => t.INT_STATUS().write(|w| w.set_T5_INT(true)),
        5 => t.INT_STATUS().write(|w| w.set_T6_INT(true)),
        6 => t.INT_STATUS().write(|w| w.set_T7_INT(true)),
        7 => t.INT_STATUS().write(|w| w.set_T8_INT(true)),
        _ => {}
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
