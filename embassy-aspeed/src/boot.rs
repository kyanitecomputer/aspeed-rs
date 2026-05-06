//! Boot module: early platform initialisation for ASPEED ARM Cortex-M targets.
//!
//! Supports:
//! - **AST2600 SSP** (`ast2600-ssp` feature) — Cortex-M3 co-processor
//! - **AST1060** (`ast1060` feature) — Cortex-M4F standalone SoC
//!
//! # AST2600 SSP boot sequence
//!
//! The ASPEED CA7 loader reads an 8-word (32-byte) `sb_header` at the start of
//! the firmware image to determine its size (`img_size`).  The linker script
//! (`link/ast2600-ssp.x`) generates this header entirely using `LONG()`
//! expressions — no Rust symbols need to be placed in `.sboot`.
//!
//! The first two words of the header are repurposed as CM3 reset vectors:
//!
//! | Offset | `sb_header` field | CM3 meaning |
//! |--------|-------------------|-------------|
//! | 0x000  | `key_location`    | Initial SP (= `_stack_start`) |
//! | 0x004  | `enc_img_addr`    | Reset vector (= `Reset` handler) |
//! | 0x008  | `img_size`        | Total image size (for CA7 loader) |
//! | 0x00C–0x01C | …            | Zeros (no key, sign, patch, checksum) |
//! | 0x020–0x3FF | padding      | Zeros — pads section to 1 KB |
//!
//! ## VTOR adjustment (AST2600 only)
//!
//! The `.sboot` section occupies `0x000–0x3FF` (1 KB).  The actual vector
//! table is at `ORIGIN(FLASH)` = `0x400`.  Because the CM3 defaults to
//! `VTOR = 0x0` after reset, interrupts would incorrectly dispatch into the
//! sboot padding bytes.  [`pre_init_ast2600`] sets `SCB.VTOR = 0x400` before
//! any interrupt can fire.
//!
//! ## Cache (AST2600 only)
//!
//! The CA7 configures the ASPEED cache (SCUA40 cacheable area, SCUA48 enable)
//! before starting the SSP.  [`pre_init_ast2600`] only enables the instruction
//! cache as an early safety measure; full cache configuration is the CA7's
//! responsibility.
//!
//! ## Non-cached BSS (AST2600 only)
//!
//! The `RAM_NC` region (`0x1000000–0x1FFFFFF`) is not covered by
//! `cortex-m-rt`'s standard BSS zeroing.  [`pre_init_ast2600`] zeros it
//! explicitly.
//!
//! # AST1060 boot sequence
//!
//! The AST1060 boots like a standard Cortex-M4F: the FMC maps SPI flash at
//! `0x0000_0000`, the CPU fetches Initial SP from `[0x0]` and Reset vector
//! from `[0x4]`, and begins executing XIP from flash.
//!
//! - No SBOOT header is needed (the AST1060's own Secure Boot MCU handles
//!   firmware measurement independently via OTP/SEC registers).
//! - VTOR stays at `0x0000_0000` (standard CM4F reset default) — no
//!   adjustment required.
//! - The CM4F FPU must be enabled explicitly; [`pre_init_ast1060`] sets
//!   `CPACR` CP10/CP11 to full access before any FP instructions run.
//! - The cache is enabled by default (SCUA58 reset value = 0x1); no explicit
//!   enable is needed.
//! - There is no `RAM_NC` region; standard BSS zeroing by `cortex-m-rt`
//!   covers all RAM.

use core::ptr;

// ── Shared register addresses ─────────────────────────────────────────────────

/// ARMv7-M System Control Block — Vector Table Offset Register.
#[cfg(feature = "ast2600-ssp")]
const SCB_VTOR: *mut u32 = 0xE000_ED08 as *mut u32;

/// ARMv7E-M Coprocessor Access Control Register.
/// CP10 and CP11 bits [21:20] and [23:22] must be set to 0b11 (full access)
/// to allow FPU instructions without triggering a UsageFault.
#[cfg(feature = "ast1060")]
const CPACR: *mut u32 = 0xE000_ED88 as *mut u32;

/// ASPEED SCU Protection Key register (shared address on AST2600 and AST1060).
/// Write `SCU_UNLOCK_KEY` to unlock all SCU registers.
const SCU_KEY: *mut u32 = 0x7E6E_2000 as *mut u32;

/// Magic value that unlocks the ASPEED SCU on AST2600 and AST1060.
const SCU_UNLOCK_KEY: u32 = 0x1688_A8A8;

// ── AST2600 SSP — register addresses ─────────────────────────────────────────

/// AST2600 SSP Cache Function Control register (CM3 view).
/// Bit 0 = `CACHE_EN` (enables both I-cache and D-cache).
#[cfg(feature = "ast2600-ssp")]
const SCU_CACHE_FUNC: *mut u32 = 0x7E6E_2A48 as *mut u32;

// ── AST2600 SSP — linker symbols ──────────────────────────────────────────────

#[cfg(feature = "ast2600-ssp")]
extern "C" {
    /// Start of the non-cached RAM region (`ORIGIN(RAM_NC)` from the linker).
    static __RAM_NC_start: u32;
    /// End of the non-cached RAM region.
    static __RAM_NC_end: u32;
}

// ── AST2600 SSP — pre_init ────────────────────────────────────────────────────

/// Called by `cortex-m-rt` before `.data` and `.bss` are initialised (AST2600 SSP).
///
/// # Safety
///
/// Uses raw pointer writes to hardware registers.  Must run before any code
/// that relies on correct interrupt dispatch or non-cached memory being zero.
#[cfg(feature = "ast2600-ssp")]
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    // 1. Redirect interrupt dispatch to the real vector table.
    //
    //    The .sboot section occupies 0x000–0x3FF.  After VTOR adjustment,
    //    all Cortex-M exceptions use the table at 0x400 (ORIGIN(FLASH)).
    //
    //    0x400 = 1024 which satisfies the VTOR alignment requirement for
    //    256 exceptions (240 IRQs + 16 ARM system exceptions):
    //      min alignment = 256 × 4 = 1024 = 0x400.
    ptr::write_volatile(SCB_VTOR, 0x400);

    // 2. Enable the ASPEED custom cache.
    //
    //    The CA7 configures cacheable area (SCUA40) and memory-map limits
    //    (SCUA04/08/0C) before releasing the SSP.  We only set the enable bit.
    //    CACHE_EN = bit 0 of CACHE_FUNC (SCUA48).
    ptr::write_volatile(SCU_CACHE_FUNC, 0b01);

    // 3. Zero the non-cached RAM region.
    //
    //    cortex-m-rt zeros .bss (in RAM) but not RAM_NC.
    let nc_start = &__RAM_NC_start as *const u32 as usize;
    let nc_end = &__RAM_NC_end as *const u32 as usize;
    ptr::write_bytes(nc_start as *mut u8, 0, nc_end - nc_start);
}

// ── AST1060 — pre_init ────────────────────────────────────────────────────────

/// Called by `cortex-m-rt` before `.data` and `.bss` are initialised (AST1060).
///
/// # Safety
///
/// Uses raw pointer writes to hardware registers.
#[cfg(feature = "ast1060")]
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    // Enable the Cortex-M4F FPU.
    //
    // The CM4F FPU coprocessors (CP10, CP11) are disabled after reset.
    // Any FP instruction before this point causes a UsageFault.
    // Set both to full access (0b11).
    //
    // CPACR bits: [21:20] = CP10 access, [23:22] = CP11 access.
    let cpacr = ptr::read_volatile(CPACR);
    ptr::write_volatile(CPACR, cpacr | (0b11 << 20) | (0b11 << 22));

    // No VTOR adjustment: the AST1060 has no SBOOT header. The vector table
    // sits at 0x0000_0000 (ORIGIN(FLASH)), which is the CM4F reset default.
    //
    // No cache enable: AST1060 SCUA58 reset value is 0x1 (cache already on).
    //
    // No RAM_NC zeroing: the AST1060 has no non-cached RAM region.
}

// ── Post-runtime initialisation (shared) ─────────────────────────────────────

/// Called from [`crate::init()`] after the Rust runtime is ready.
///
/// Unlocks the ASPEED SCU protection key so that clock and reset registers
/// can be written by the HAL drivers.
///
/// For the AST2600 SSP, clock frequencies are not changed here — the CA7
/// configures the PLL (HCLK = 200 MHz) before releasing the SSP from reset.
///
/// For the AST1060, the H-PLL runs at 1000 MHz by default (HCLK = 500 MHz
/// via PCLK divider); HAL clock drivers may adjust this.
pub fn platform_init() {
    // Unlock the SCU protection key.
    //
    // The key is not consumed by writes — it persists until explicitly locked.
    // A read of SCU000 returns 1 if unlocked, 0 if locked.
    //
    // Safety: raw write to a well-documented hardware register.
    unsafe { ptr::write_volatile(SCU_KEY, SCU_UNLOCK_KEY) };
}
