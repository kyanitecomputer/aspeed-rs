//! Early-boot hooks and platform initialisation for the AST2700 BootMCU
//! (lowRISC ibex, RV32IMC).
//!
//! Unlike the Cortex-M targets, the BootMCU ROM performs all early
//! initialisation (DRAM controller, stack setup, .data/.bss copy) before
//! transferring control to the Rust entry point.  This module therefore
//! provides a minimal `pre_init` hook (no-op) and an empty `platform_init_rv`
//! for any future SCU1 or peripheral early configuration.
//!
//! # Boot flow
//!
//! ```text
//! ROM → loads ELF from SPI FMC (0x20000000) → DRAM (0x80000000)
//!     → jumps to riscv-rt __start
//!     → __pre_init (this module, no-op)
//!     → zeroes .bss
//!     → user #[riscv_rt::entry] main
//!     → embassy_aspeed::init()   (calls platform_init_rv + time driver)
//! ```

use riscv_rt::pre_init;

/// Called by `riscv-rt` before `.bss` is zeroed and `.data` is copied.
///
/// For the AST2700 BootMCU the ROM has already configured DRAM and the
/// system is ready to run.  No early hardware setup is required here.
///
/// # Safety
///
/// Runs before global variables are initialised. Must not access `.bss` or
/// `.data` memory.
#[pre_init]
unsafe fn pre_init() {
    // Nothing to do: ROM has handled DRAM, stack pointer, and clocks.
}

/// Perform platform-level initialisation after the runtime is ready.
///
/// Called from `embassy_aspeed::init()`. Currently a no-op; add SCU1
/// register writes or peripheral early configuration here as needed.
pub fn platform_init_rv() {}
