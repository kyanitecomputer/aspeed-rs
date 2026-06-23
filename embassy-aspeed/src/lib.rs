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
//! - `log-uart` — `log`-crate global logger over UART12 (AST2700 BootMCU)
//! - `defmt` (being retired) — structured logging support
//! - `defmt-uart` (being retired) — AST2700 BootMCU defmt frames over UART12

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

// Re-export safe MMIO primitives for HAL drivers and downstream users.
pub use aspeed_mmio as mmio;

// ── HAL driver modules ────────────────────────────────────────────────────────
// Modules that require ARM cortex-m are gated on chip feature flags so that
// host-side unit tests (cargo test --target x86_64) can compile without them.
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod boot;
#[cfg(feature = "ast2700-bootmcu")]
pub mod boot_rv;
#[cfg(feature = "ast2700-bootmcu")]
pub mod bootmode;
#[cfg(feature = "ast2700-bootmcu")]
pub mod ca35;
pub mod clock;
#[cfg(feature = "ast2700-bootmcu")]
pub mod cptra;
#[cfg(all(feature = "ast2700-bootmcu", feature = "defmt-uart"))]
pub mod defmt_uart;
#[cfg(all(feature = "ast2700-bootmcu", feature = "log-uart"))]
pub mod log_uart;
#[cfg(feature = "ast2700-bootmcu")]
pub mod display;
#[cfg(feature = "ast2700-bootmcu")]
pub mod emmc;
// FLSH container parser is pure logic (no MMIO): ungated so host unit tests can
// exercise it without a chip feature.
pub mod flsh;
#[cfg(feature = "ast2700-bootmcu")]
pub mod extrst;
#[cfg(feature = "ast2700-bootmcu")]
pub mod ipc1;
// LZ4 payload decompression: pure logic (no MMIO), gated on the lz4 feature so
// host tests can run it via `--features lz4` without a chip feature.
#[cfg(feature = "lz4")]
pub mod lz4;
#[cfg(feature = "ast2700-bootmcu")]
pub mod manifest;
#[cfg(feature = "ast2700-bootmcu")]
pub mod otp;
#[cfg(feature = "ast2700-bootmcu")]
pub mod prebuilt;
#[cfg(feature = "ast2700-bootmcu")]
pub mod scu;
#[cfg(feature = "ast2700-bootmcu")]
pub mod sdrammc;
#[cfg(feature = "ast2700-bootmcu")]
pub mod sli;
#[cfg(feature = "ast2700-bootmcu")]
pub mod ssp_tsp;
// uart, gpio, timer, wdt are chip-specific (use hardware registers directly).
// Gated to prevent compilation for host-side unit tests (no chip feature active).
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod adc;
pub mod addr;
#[cfg(feature = "ast1060")]
pub mod ecdsa;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod gpio;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod hace;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod i2c;
#[cfg(feature = "ast1060")]
pub mod i2cfilter;
#[cfg(feature = "ast1060")]
pub mod i3c;
pub mod ipc;
#[cfg(feature = "ast2600-ssp")]
pub mod pwm;
#[cfg(feature = "ast1060")]
pub mod reset;
#[cfg(feature = "ast1060")]
pub mod rsa;
#[cfg(feature = "ast1060")]
pub mod sgpio;
#[cfg(any(feature = "ast1060", feature = "ast2700-bootmcu"))]
pub mod spi;
#[cfg(feature = "ast1060")]
pub mod spi_monitor;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod time_driver;
#[cfg(feature = "ast2700-bootmcu")]
pub mod time_driver_rv;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod timer;
#[cfg(feature = "ast1060")]
pub mod trng;
#[cfg(any(
    feature = "ast2600-ssp",
    feature = "ast1060",
    feature = "ast2700-bootmcu"
))]
pub mod uart;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub mod wdt;
#[cfg(feature = "ast2700-bootmcu")]
pub mod wdt_ast2700;

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
/// Equivalent to sys_arch_reboot(). Does not return.
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
