//! Embassy async HAL for ASPEED ARM Cortex-M coprocessors and standalone SoCs.
//!
//! # Quick start
//!
//! ```rust,ignore
//! #![no_std]
//! #![no_main]
//!
//! use embassy_aspeed as hal;
//! use embassy_executor::Spawner;
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     hal::init(hal::Config::default());
//!     // use peripheral drivers from hal::uart, hal::gpio, …
//! }
//! ```
//!
//! # Feature flags
//!
//! Enable exactly one chip feature:
//! - `ast2600-ssp`     — ASPEED AST2600 SSP (ARM Cortex-M3, 200 MHz HCLK)
//! - `ast1060`         — ASPEED AST1060 standalone SoC (ARM Cortex-M4F)
//! - `ast2700-bootmcu` — ASPEED AST2700 BootMCU (RISC-V ibex RV32IMC)
//!
//! Optional:
//! - `defmt` (default) — structured logging via RTT

#![no_std]

// The chip feature guard is skipped during host-side unit tests (no chip
// feature is needed for pure-logic tests in clock.rs, addr.rs, etc.).
#[cfg(not(any(
    feature = "ast2600-ssp",
    feature = "ast1060",
    feature = "ast2700-bootmcu",
    test
)))]
compile_error!(
    "embassy-aspeed: select a chip target feature — \
     enable exactly one of: ast2600-ssp, ast1060, ast2700-bootmcu"
);

// Re-export the PAC so downstream crates can access registers directly.
pub use aspeed_pac as pac;

// ── HAL driver modules ────────────────────────────────────────────────────────
// Modules that require ARM cortex-m are gated on chip feature flags so that
// host-side unit tests (cargo test --target x86_64) can compile without them.
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod boot;
#[cfg(feature = "ast2700-bootmcu")]
pub mod boot_rv;
pub mod clock;
#[cfg(feature = "ast2700-bootmcu")]
pub mod ipc1;
// uart, gpio, timer, wdt are chip-specific (use hardware registers directly).
// Gated to prevent compilation for host-side unit tests (no chip feature active).
pub mod addr;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod gpio;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod i2c;
pub mod ipc;
#[cfg(feature = "ast1060")]
pub mod spi;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod time_driver;
#[cfg(feature = "ast2700-bootmcu")]
pub mod time_driver_rv;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod timer;
#[cfg(any(
    feature = "ast2600-ssp",
    feature = "ast1060",
    feature = "ast2700-bootmcu"
))]
pub mod uart;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod wdt;

// ── Public top-level API ──────────────────────────────────────────────────────

/// HAL initialisation configuration.
pub struct Config {
    _private: (),
}

impl Default for Config {
    fn default() -> Self {
        Self { _private: () }
    }
}

/// Initialise the HAL.
///
/// Must be called once at the start of `main()` before using any peripheral
/// driver. Performs:
///
/// 1. Unlock the SCU protection key (allows HAL drivers to write clock/reset regs).
/// 2. Initialise the embassy SysTick time driver (1 µs tick).
///
/// On AST2600 SSP: cache and VTOR are handled in `#[cortex_m_rt::pre_init]`
/// (see `boot.rs`). HCLK = 200 MHz is set by the CA7 before the SSP boots.
///
/// On AST1060: FPU enable is handled in `#[cortex_m_rt::pre_init]` (see
/// `boot.rs`). HCLK = 500 MHz (H-PLL 1000 MHz ÷ PCLK divider 2) by default.
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub fn init(_config: Config) {
    boot::platform_init();
    time_driver::init();
}

/// Immediately reboot the SoC via the WDT software-reset mechanism.
///
/// Equivalent to Zephyr's `sys_arch_reboot()`. Does not return.
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub fn reboot() -> ! {
    wdt::sys_reboot()
}

/// Initialise the HAL for the AST2700 BootMCU (RV32IMC).
///
/// Call once at the start of `main()`. Performs minimal platform init;
/// the BootMCU ROM has already configured DRAM and clocks before this runs.
#[cfg(feature = "ast2700-bootmcu")]
pub fn init(_config: Config) {
    boot_rv::platform_init_rv();
    time_driver_rv::init();
}
