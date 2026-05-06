//! Embassy time driver for the AST2700 BootMCU 64-bit timer.
//!
//! The BootMCU has a custom 64-bit free-running timer at `0x14C36000` that
//! counts at **1 MHz** (1 µs per tick), matching `tick-hz-1_000_000`.
//!
//! # Interrupt
//!
//! The timer fires the **machine timer interrupt** (MTIP, IRQ 7 from hlic).
//! In `riscv-rt` 0.12, this is handled by a function exported as `MachineTimer`.
//!
//! # Design
//!
//! The 64-bit counter runs freely.  `schedule_wake` writes ALARM_L/H and
//! enables the interrupt via CTRL[EN].  On each alarm the ISR drains the
//! embassy queue and programs the next alarm.
//!
//! # 64-bit counter read
//!
//! On RV32 there is no single-instruction 64-bit read.  We use the H/L/H
//! pattern: read COUNT_H, then COUNT_L, then COUNT_H again; if the two H
//! readings differ (very rare — requires overflow at exactly the right cycle)
//! we use the second H value with L=0 as an approximation.
//!
//! # Sources
//!
//! Zephyr `drivers/timer/ast2700_bootmcu_timer.c`
//! ROADMAP.md Task 41

use core::cell::RefCell;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

// ── Timer register base (BootMCU view) ────────────────────────────────────────

const TIMER_BASE: usize = 0x14C3_6000;

const COUNT_L:   *const u32 = (TIMER_BASE + 0x00) as *const u32;
const COUNT_H:   *const u32 = (TIMER_BASE + 0x04) as *const u32;
const ALARM_L:   *mut u32   = (TIMER_BASE + 0x08) as *mut u32;
const ALARM_H:   *mut u32   = (TIMER_BASE + 0x0C) as *mut u32;
const CTRL:      *mut u32   = (TIMER_BASE + 0x10) as *mut u32;
const CTRL_CLR:  *mut u32   = (TIMER_BASE + 0x14) as *mut u32;

const EN:        u32 = 1 << 0;
const RESET_EN:  u32 = 1 << 3;
const COUNT_CLR: u32 = 1 << 4;

// ── Driver ────────────────────────────────────────────────────────────────────

struct BootMcuTimerDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl BootMcuTimerDriver {
    /// Read the free-running 64-bit counter using the H/L/H pattern to avoid
    /// a race on the 32-bit boundary.
    #[inline]
    fn read_count(&self) -> u64 {
        // H/L/H pattern: if the high word changed between the two reads, a
        // 32-bit rollover occurred during the window.  Retry until we get a
        // consistent pair rather than returning a corrupted value.
        loop {
            let h1 = unsafe { COUNT_H.read_volatile() };
            let l  = unsafe { COUNT_L.read_volatile() };
            let h2 = unsafe { COUNT_H.read_volatile() };
            if h1 == h2 {
                return ((h1 as u64) << 32) | (l as u64);
            }
            // Rollover occurred between reads; the L value is stale.  Retry
            // for a consistent H/L pair.  This branch is taken at most once
            // per 2^32 µs ≈ every ~71 minutes.
        }
    }

    /// Arm the alarm at `at` ticks and enable the interrupt.
    /// Writing ALARM_L/H also clears INTR_STS.
    #[inline]
    fn set_alarm(&self, at: u64) {
        unsafe {
            // Disable interrupt while reprogramming.
            CTRL_CLR.write_volatile(EN);
            // Writing alarm clears INTR_STS.
            ALARM_L.write_volatile(at as u32);
            ALARM_H.write_volatile((at >> 32) as u32);
            // Re-enable interrupt.
            CTRL.write_volatile(EN);
        }
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            // Disable the alarm interrupt.
            unsafe { CTRL_CLR.write_volatile(EN); }

            let now = self.read_count();
            let mut queue = self.queue.borrow(cs).borrow_mut();

            // Wake all expired tasks and find the next scheduled wake.
            let mut next = queue.next_expiration(now);
            while next <= now {
                next = queue.next_expiration(now);
            }

            if next != u64::MAX {
                // Drop the borrow before calling set_alarm to avoid re-entrancy.
                drop(queue);
                self.set_alarm(next);
            }
        });
    }
}

impl Driver for BootMcuTimerDriver {
    fn now(&self) -> u64 {
        self.read_count()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            if queue.schedule_wake(at, waker) {
                let now = self.read_count();
                let next = queue.next_expiration(now);
                drop(queue);
                self.set_alarm(next);
            }
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: BootMcuTimerDriver = BootMcuTimerDriver {
    queue: Mutex::new(RefCell::new(Queue::new())),
});

// ── Machine timer ISR ─────────────────────────────────────────────────────────

/// riscv-rt machine timer interrupt handler.
///
/// The function name `MachineTimer` is the weak symbol defined in riscv-rt's
/// link.x.  Overriding it hooks into the machine timer interrupt (MTIP).
#[no_mangle]
extern "C" fn MachineTimer() {
    DRIVER.on_interrupt();
}

// ── Initialisation ────────────────────────────────────────────────────────────

/// Initialise the BootMCU timer driver.
///
/// Called once from `embassy_aspeed::init()`.
pub fn init() {
    unsafe {
        // Disable interrupt.
        CTRL_CLR.write_volatile(EN);
        // Set alarm to maximum to suppress spurious firings.
        ALARM_L.write_volatile(0xFFFF_FFFF);
        ALARM_H.write_volatile(0xFFFF_FFFF);
        // Reset the counter and keep RESET_EN so INTR_STS clears correctly.
        CTRL.write_volatile(COUNT_CLR | RESET_EN);
        // Counter now runs freely; interrupt fires only when armed by schedule_wake.
    }
}
