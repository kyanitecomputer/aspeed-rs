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
//! `init()` enables EN — the 64-bit counter runs freely from 0 for the
//! entire lifetime of the firmware.  `schedule_wake` writes ALARM_L/H
//! (which also clears INTR_STS).  On each alarm the ISR drains the
//! embassy queue and programs the next alarm.  RESET_EN is explicitly
//! cleared so the counter never wraps to 0 on alarm match.
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
//! aspeed-data/data/registers/bootmcu_timer_v1.yaml (source of truth)

use core::cell::RefCell;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::pac;

// PAC accessor — source of truth: bootmcu_timer_v1.yaml
// pac::TIMER = TIMER at 0x14C36000 (ast2700_bootmcu chip YAML)
#[inline(always)]
fn timer() -> pac::bootmcu_timer_v1::TIMER {
    pac::TIMER
}

// ── Driver ────────────────────────────────────────────────────────────────────

struct BootMcuTimerDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl BootMcuTimerDriver {
    /// Read the free-running 64-bit counter using the H/L/H pattern.
    ///
    /// On RV32 there is no single-instruction 64-bit read.  If COUNT_H
    /// changes between the two H reads, a 32-bit rollover occurred and we
    /// retry.  This branch fires at most once per 2^32 µs ≈ every 71 min.
    /// Source of truth: bootmcu_timer_v1.yaml COUNT_L/COUNT_H (Read access).
    #[inline]
    fn read_count(&self) -> u64 {
        loop {
            let h1 = timer().COUNT_H().read();
            let l = timer().COUNT_L().read();
            let h2 = timer().COUNT_H().read();
            if h1 == h2 {
                return ((h1 as u64) << 32) | (l as u64);
            }
        }
    }

    /// Arm the alarm at `at` µs ticks.
    ///
    /// Counter runs freely (EN=1 set during init).  Writing ALARM also
    /// clears CTRL.INTR_STS (bootmcu_timer_v1.yaml ALARM_L/H description).
    /// Write high word first to avoid a transient 64-bit match.
    #[inline]
    fn set_alarm(&self, at: u64) {
        timer().MATCH_H().write_value((at >> 32) as u32);
        timer().MATCH_L().write_value(at as u32);
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            let now = self.read_count();
            let mut queue = self.queue.borrow(cs).borrow_mut();

            let mut next = queue.next_expiration(now);
            while next <= now {
                next = queue.next_expiration(now);
            }

            if next != u64::MAX {
                drop(queue);
                self.set_alarm(next);
            } else {
                // No pending wakes — park alarm at MAX to silence future
                // spurious interrupts.  Writing ALARM clears INTR_STS.
                timer().MATCH_H().write_value(0xFFFF_FFFF);
                timer().MATCH_L().write_value(0xFFFF_FFFF);
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
    // All register accesses via PAC (bootmcu_timer_v1.yaml).
    // Disable timer — stops counter and interrupt.
    timer().CTRL_CLR().write(|w| w.set_INTR_EN(true));

    // Park alarm at MAX to suppress spurious firings.
    // Writing MATCH_L/H also clears INTR_STS (bootmcu_timer_v1.yaml).
    timer().MATCH_H().write_value(0xFFFF_FFFF);
    timer().MATCH_L().write_value(0xFFFF_FFFF);

    // Reset counter to 0 (COUNT_CLR is self-clearing per YAML).
    timer().CTRL().write(|w| w.set_COUNT_CLR(true));

    // Enable timer — counter starts from 0, runs freely at 1 MHz.
    timer().CTRL().write(|w| w.set_INTR_EN(true));

    // Enable machine timer interrupt (mie[7] = MTIE).
    // ibex wires this timer to the RISC-V machine timer IRQ.
    unsafe { riscv::register::mie::set_mtimer() };
}
