//! Watchdog timer driver (AST1060) — PAC-based.
//!
//! Source of truth: `aspeed-data/data/registers/wdt_v1.yaml`.
//!
//! The AST1060 has 4 WDT instances:
//!
//! | Instance | Base address | PAC constant |
//! |----------|-------------|-------------|
//! | 1 | `0x7E78_5000` | `pac::WDT1` |
//! | 2 | `0x7E78_5080` | `pac::WDT2` |
//! | 3 | `0x7E78_5100` | `pac::WDT3` |
//! | 4 | `0x7E78_5180` | `pac::WDT4` |
//!
//! Register layout (from `wdt_v1.yaml`):
//!
//! | Register | Description |
//! |----------|-------------|
//! | `STATUS` | Current counter value (µs, read-only) |
//! | `RELOAD` | Counter reload value (µs) |
//! | `RESTART` | Write `0x4755` to reload and restart |
//! | `CTRL` | Enable, reset mode, interrupt |
//! | `SW_RESET_CTRL` | Write `0xAEED_F123` to trigger software SOC reset |
//! | `SW_RESET_MASK1` | Subsystem reset enable bitmask |
//!
//! # Software reboot
//!
//! 1. Write subsystem mask to `SW_RESET_MASK1`.
//! 2. Write trigger key `0xAEED_F123` to `SW_RESET_CTRL`.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::wdt::Watchdog;
//!
//! let mut wdt = Watchdog::new(1, 5_000_000); // WDT1, 5 s timeout
//! wdt.start();
//! loop { wdt.feed(); /* … */ }
//! ```

use crate::pac;

// ── Constants from wdt_v1.yaml ────────────────────────────────────────────────

/// Feed key for `RESTART` register (wdt_v1.yaml `WDT_RESTART.KEY`).
const RESTART_KEY: u32 = 0x4755;

/// Trigger key for `SW_RESET_CTRL` (wdt_v1.yaml `WDT_SW_RESET_CTRL.TRIGGER`).
const SW_RESET_KEY: u32 = 0xAEEDF123;

/// Standard subsystem reset mask (wdt_v1.yaml `WDT_SW_RESET_MASK1.MASK`
/// value used for system reset.
const STD_RESET_MASK: u32 = 0x03FF_FFF1;

// ── Software reboot ───────────────────────────────────────────────────────────

/// Immediately reboot the SoC using WDT1 software reset.  Does not return.
///
/// Resets the standard subsystem set (ARM, SDRAM, AHB bridges, peripherals).
pub fn sys_reboot() -> ! {
    let wdt = pac::WDT1;
    wdt.SW_RESET_MASK1().write(|w| w.set_MASK(STD_RESET_MASK));
    wdt.SW_RESET_CTRL().write(|w| w.set_TRIGGER(SW_RESET_KEY));
    loop {
        core::hint::spin_loop();
    }
}

// ── Watchdog ──────────────────────────────────────────────────────────────────

/// Watchdog timer driver.
///
/// The counter counts at 1 MHz (1 µs per tick).  If `feed()` is not called
/// before the counter reaches zero the SoC resets.
pub struct Watchdog {
    regs: pac::wdt_v1::WDT,
    timeout_us: u32,
}

impl Watchdog {
    /// Create a watchdog bound to WDT instance `inst` (1–4).
    ///
    /// `timeout_us`: microseconds before reset (max ~71 minutes).
    ///
    /// # Panics
    ///
    /// Panics if `inst` is not 1–4.
    pub fn new(inst: u8, timeout_us: u32) -> Self {
        let regs = match inst {
            1 => pac::WDT1,
            2 => pac::WDT2,
            3 => pac::WDT3,
            _ => panic!("WDT instance must be 1–3"),
        };
        Self { regs, timeout_us }
    }

    /// Arm the watchdog: load timeout and start countdown.
    pub fn start(&mut self) {
        self.regs.RELOAD().write(|w| w.set_VALUE(self.timeout_us));
        self.regs.RESTART().write(|w| w.set_KEY(RESTART_KEY as u16));
        // CTRL: WDT_EN=1, RST_SYS=1 (SOC reset on timeout), RST_MODE=00.
        self.regs.CTRL().write(|w| {
            w.set_WDT_EN(true);
            w.set_RST_SYS(true);
            w.set_RST_MODE(0);
        });
    }

    /// Feed the watchdog (reload counter from `RELOAD`).
    ///
    /// Must be called within `timeout_us` µs of the last call.
    #[inline(always)]
    pub fn feed(&mut self) {
        self.regs.RESTART().write(|w| w.set_KEY(RESTART_KEY as u16));
    }

    /// Stop (disable) the watchdog.
    pub fn stop(&mut self) {
        self.regs.CTRL().write(|w| w.set_WDT_EN(false));
    }

    /// Read the current counter value in microseconds.
    pub fn status_us(&self) -> u32 {
        self.regs.STATUS().read().VALUE()
    }
}
