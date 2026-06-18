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
//! # Minimum SysTick reload
//!
//! At low HCLK the SysTick reload value for 1 µs ticks can be so small that
//! the CPU cannot service the interrupt (Cortex-M4 exception entry/exit costs
//! ~24 cycles alone).  [`init`] reads the actual HCLK from SCU registers and
//! clamps the reload to [`MIN_SYSTICK_RELOAD`] (99 → 100 cycle period).
//! When clamped, `embassy-time` runs slower than real-time; firmware should
//! enable the PLL and call [`reinit`] to restore 1:1 µs ticks.
//!
//! # Default clock sources
//!
//! | Chip | Core clock at boot | SYSTICK_RELOAD |
//! |------|--------------------|----------------|
//! | AST2600 SSP | 200 MHz (set by CA7) | 199 |
//! | AST1060 | 25 MHz (PLL bypassed) → clamped to 99 |  |
//! | AST1060 | 200 MHz (after PLL init) | 199 |
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

/// Minimum SysTick reload value.  Cortex-M4 exception entry (12 cycles) +
/// handler body (~15 cycles) + exit (12 cycles) ≈ 39 cycles minimum.
/// A reload of 99 (100 cycle period) gives ~60% CPU headroom for main.
const MIN_SYSTICK_RELOAD: u32 = 99;

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

/// Initialise the SysTick time driver by reading the actual core clock from
/// SCU registers.
///
/// - AST2600 SSP: HCLK = 200 MHz (set by CA7 before SSP release).
/// - AST1060: reads HPLL + PCLK divider from SCU200/SCU310.  At reset
///   (HPLL bypassed), PCLK = 25 MHz → reload would be 24, which starves
///   the CPU.  The reload is clamped to [`MIN_SYSTICK_RELOAD`] (99) so
///   the CPU can still execute main-thread code.
///
/// Called once from [`crate::init`].
pub fn init() {
    let reload = compute_reload();
    systick_configure(reload);
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
    let raw = (hclk_hz as u64 / TICK_HZ - 1) as u32;
    let reload = if raw < MIN_SYSTICK_RELOAD {
        MIN_SYSTICK_RELOAD
    } else {
        raw
    };
    systick_configure(reload);
}

/// Compute the SysTick reload value from the actual hardware clock.
#[cfg(feature = "ast2600-ssp")]
fn compute_reload() -> u32 {
    // CA7 configures HCLK = 200 MHz before releasing the SSP.
    // No SCU read needed — the value is fixed.
    let raw = (200_000_000u64 / TICK_HZ - 1) as u32;
    if raw < MIN_SYSTICK_RELOAD {
        MIN_SYSTICK_RELOAD
    } else {
        raw
    }
}

#[cfg(feature = "ast1060")]
fn compute_reload() -> u32 {
    use crate::clock::ast1060_clk;
    use aspeed_mmio::MmioBlock;

    const SCU_BASE: usize = 0x7E6E_2000;

    // Read actual HPLL and PCLK divider from SCU registers.
    let scu = unsafe { MmioBlock::new(SCU_BASE) };
    let hpll_reg = scu.read32(ast1060_clk::HPLL_PARAM);
    let clk_sel4 = scu.read32(ast1060_clk::CLK_SEL4);
    let hpll_hz = ast1060_clk::hpll_from_reg(hpll_reg);
    let pclk_hz = if hpll_hz == 0 {
        // PLL powered down — fall back to crystal input.
        ast1060_clk::CLKIN_HZ
    } else {
        ast1060_clk::pclk_from_hpll_and_reg(hpll_hz, clk_sel4)
    };

    let raw = (pclk_hz as u64 / TICK_HZ).saturating_sub(1) as u32;
    if raw < MIN_SYSTICK_RELOAD {
        MIN_SYSTICK_RELOAD
    } else {
        raw
    }
}

fn systick_configure(reload: u32) {
    // SAFETY: sole owner of core peripherals at init time.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.SYST
        .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
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
