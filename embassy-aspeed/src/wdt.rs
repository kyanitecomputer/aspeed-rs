//! Watchdog timer driver and software reboot.
//!
//! # Registers (WDT0 base 0x7E78_5000)
//!
//! | Offset | Name             | Description |
//! |--------|------------------|-------------|
//! | 0x00   | STATUS           | Current counter value (µs, read-only) |
//! | 0x04   | RELOAD           | Counter reload value (µs) |
//! | 0x08   | RESTART          | Write 0x4755 to feed the watchdog |
//! | 0x0C   | CTRL             | Enable, reset mode, interrupt |
//! | 0x24   | SW_RESET_CTRL    | Write 0xAEED_F123 to trigger software reset |
//! | 0x28   | SW_RESET_MASK1   | Which subsystems to reset |
//!
//! # Software reboot
//!
//! Matches Zephyr's `sys_arch_reboot()` exactly:
//! 1. Write `0x3FFF_FF1` to `SW_RESET_MASK1` (enable standard subsystem resets).
//! 2. Write `0xAEED_F123` to `SW_RESET_CTRL` to fire.
//!
//! # Watchdog usage
//!
//! ```rust,ignore
//! use embassy_aspeed::wdt::Watchdog;
//!
//! let mut wdt = Watchdog::new(5_000_000); // 5-second timeout
//! wdt.start();
//! loop {
//!     wdt.feed();
//!     // do work
//! }
//! ```

use core::ptr;

// ── WDT0 register addresses ───────────────────────────────────────────────────

const WDT0_BASE: usize = 0x7E78_5000;

const WDT_STATUS: *const u32 = (WDT0_BASE + 0x00) as *const u32;
const WDT_RELOAD: *mut u32 = (WDT0_BASE + 0x04) as *mut u32;
const WDT_RESTART: *mut u32 = (WDT0_BASE + 0x08) as *mut u32;
const WDT_CTRL: *mut u32 = (WDT0_BASE + 0x0C) as *mut u32;
const WDT_SW_RESET_CTRL: *mut u32 = (WDT0_BASE + 0x24) as *mut u32;
const WDT_SW_RESET_MASK1: *mut u32 = (WDT0_BASE + 0x28) as *mut u32;

/// Magic key to feed the watchdog (write to WDT_RESTART).
const WDT_FEED_KEY: u32 = 0x4755;

/// Magic key to trigger software mode reset (write to WDT_SW_RESET_CTRL).
const WDT_SW_RESET_KEY: u32 = 0xAEEDF123;

/// Standard subsystem reset mask used by Zephyr's `sys_arch_reboot()`.
const WDT_STD_RESET_MASK: u32 = 0x03FFF_FF1;

/// WDT control: enable + SOC reset mode (bits [6:5] = 0b00) + RST_SYS (bit 1).
const WDT_CTRL_ENABLE: u32 = (1 << 0) | (1 << 1); // EN + RST_SYS

// ── Software reboot ───────────────────────────────────────────────────────────

/// Immediately reboot the SoC using the WDT software reset mechanism.
///
/// Equivalent to Zephyr's `sys_arch_reboot()`.  Does not return.
///
/// Resets: ARM CPU, SDRAM controller, AHB bridges, coprocessor, SOC
/// controllers (WDT, RTC, Timer, UART, SRAM), USB, Ethernet MACs, video,
/// hash/crypto, and more — identical to the standard Zephyr reset mask.
pub fn sys_reboot() -> ! {
    // SAFETY: intentional hardware reset; no return path.
    unsafe {
        ptr::write_volatile(WDT_SW_RESET_MASK1, WDT_STD_RESET_MASK);
        ptr::write_volatile(WDT_SW_RESET_CTRL, WDT_SW_RESET_KEY);
    }
    // Hardware asserts reset within a few cycles.  Loop in case of delay.
    loop {
        core::hint::spin_loop();
    }
}

// ── Watchdog struct ───────────────────────────────────────────────────────────

/// Watchdog timer driver wrapping WDT0.
///
/// The watchdog counts down at 1 MHz (1 µs per tick).  If `feed()` is not
/// called before the counter reaches zero, the SoC resets.
pub struct Watchdog {
    /// Timeout in microseconds.
    timeout_us: u32,
}

impl Watchdog {
    /// Create a new watchdog with `timeout_us` microseconds before reset.
    ///
    /// Call [`start`](Self::start) to arm the watchdog.
    pub fn new(timeout_us: u32) -> Self {
        Self { timeout_us }
    }

    /// Arm the watchdog: load the timeout and enable countdown.
    pub fn start(&mut self) {
        // SAFETY: WDT MMIO writes.
        unsafe {
            ptr::write_volatile(WDT_RELOAD, self.timeout_us);
            ptr::write_volatile(WDT_RESTART, WDT_FEED_KEY);
            ptr::write_volatile(WDT_CTRL, WDT_CTRL_ENABLE);
        }
    }

    /// Feed the watchdog: reload the counter from `RELOAD`.
    ///
    /// Must be called within `timeout_us` microseconds of the last feed.
    pub fn feed(&mut self) {
        // SAFETY: WDT MMIO write.
        unsafe { ptr::write_volatile(WDT_RESTART, WDT_FEED_KEY) };
    }

    /// Stop (disable) the watchdog.
    pub fn stop(&mut self) {
        // SAFETY: WDT MMIO write.
        unsafe { ptr::write_volatile(WDT_CTRL, 0) };
    }

    /// Read the current counter value in microseconds.
    pub fn status_us(&self) -> u32 {
        // SAFETY: WDT MMIO read.
        unsafe { ptr::read_volatile(WDT_STATUS) }
    }
}
