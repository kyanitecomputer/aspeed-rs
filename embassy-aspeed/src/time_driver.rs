//! Embassy time driver using the ARM Cortex-M SysTick timer.
//!
//! Tick rate: 1 MHz (1 µs per tick), matching `tick-hz-1_000_000` in
//! `embassy-time`.
//!
//! # Design
//!
//! SysTick counts down from `RELOAD` to 0 at the core clock (HCLK/PCLK),
//! producing an interrupt every 1 µs.  Each interrupt increments a 64-bit
//! `TICKS` counter protected by a `critical-section` mutex.
//!
//! `schedule_wake(at, waker)` enqueues the waker in an `embassy-time-queue-utils`
//! `Queue`.  On each tick interrupt the queue is drained, waking any futures
//! whose deadline has passed.
//!
//! # Default clock sources
//!
//! | Chip | Core clock at boot | SYSTICK_RELOAD |
//! |------|--------------------|----------------|
//! | AST2600 SSP | 200 MHz (set by CA7) | 199 |
//! | AST1060 | 25 MHz (HPLL bypassed, SCU200[24]=1) | 24 |
//!
//! For AST1060, the PLL is bypassed at reset (HCLK = CLKIN = 25 MHz).
//! Firmware can initialise the PLL and call [`reinit`] to update the
//! SysTick period.
//!
//! # Initialisation
//!
//! Called once from [`crate::init`]:
//!
//! ```rust,ignore
//! embassy_aspeed::init(Config::default()); // internally calls time_driver::init()
//! ```

use core::cell::{Cell, RefCell};
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Embassy tick frequency (1 MHz = 1 µs per tick).
const TICK_HZ: u64 = 1_000_000;

/// Default core clock for AST2600 SSP (set by CA7 before SSP release).
#[cfg(feature = "ast2600-ssp")]
const DEFAULT_HCLK_HZ: u64 = 200_000_000;

/// Default core clock for AST1060 at boot (HPLL bypassed → PCLK = CLKIN = 25 MHz).
/// After PLL setup, call `reinit(new_hclk_hz)` to update the SysTick period.
#[cfg(feature = "ast1060")]
const DEFAULT_HCLK_HZ: u64 = 25_000_000;

/// SysTick reload value computed from the default HCLK.
const SYSTICK_RELOAD: u32 = (DEFAULT_HCLK_HZ / TICK_HZ - 1) as u32;

// ── Driver state ──────────────────────────────────────────────────────────────

struct SysTickDriver {
    /// Current time in µs ticks, protected by critical section.
    ticks: Mutex<Cell<u64>>,
    /// Timer waker queue.
    queue: Mutex<RefCell<Queue>>,
}

impl SysTickDriver {
    fn on_tick(&'static self) {
        critical_section::with(|cs| {
            let cell = self.ticks.borrow(cs);
            let t = cell.get().wrapping_add(1);
            cell.set(t);
            // Drain expired wakers.
            self.queue.borrow(cs).borrow_mut().next_expiration(t);
        });
    }
}

impl Driver for SysTickDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| self.ticks.borrow(cs).get())
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow(cs).borrow_mut();
            if q.schedule_wake(at, waker) {
                // If the waker was newly enqueued, check if it already expired.
                let t = self.ticks.borrow(cs).get();
                q.next_expiration(t);
            }
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: SysTickDriver = SysTickDriver {
    ticks: Mutex::new(Cell::new(0)),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

// ── Initialisation ────────────────────────────────────────────────────────────

/// Initialise the SysTick time driver using the default chip clock.
///
/// - AST2600 SSP: HCLK = 200 MHz (set by CA7).
/// - AST1060: PCLK = 25 MHz (HPLL bypassed at reset).
///
/// Called once from [`crate::init`].
pub fn init() {
    systick_configure(SYSTICK_RELOAD);
}

/// Reinitialise SysTick after a clock frequency change.
///
/// Call this after enabling the PLL on AST1060 so the SysTick period
/// is corrected to maintain 1 µs ticks.
///
/// # Example (AST1060 after PLL enable)
///
/// ```rust,ignore
/// // After configuring SCU200 for 1000 MHz HPLL and SCU310 for ÷2:
/// embassy_aspeed::time_driver::reinit(500_000_000);
/// ```
pub fn reinit(hclk_hz: u32) {
    let reload = (hclk_hz as u64 / TICK_HZ - 1) as u32;
    systick_configure(reload);
}

fn systick_configure(reload: u32) {
    // SAFETY: sole owner of core peripherals at init time.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    cp.SYST.set_reload(reload);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
}

// ── SysTick exception handler ─────────────────────────────────────────────────

/// Cortex-M SysTick exception handler.
#[cortex_m_rt::exception]
fn SysTick() {
    DRIVER.on_tick();
}
