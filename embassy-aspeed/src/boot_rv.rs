//! Early-boot hooks and platform initialisation for the AST2700 BootMCU
//! (lowRISC ibex, RV32IMC).
//!
//! # Root cause 1: riscv-rt _start hangs on AST2700 A1 silicon
//!
//! riscv-rt's `_start` opens with `lui ra / jr` — a 2-instruction window
//! before `csrwi mie,0`.  If the ROM left any interrupt pending, it fires
//! through ROM's mtvec and never returns.
//!
//! Fixed by an unnamed startup stub in `.section .init` that executes
//! `csrwi mie,0` as the absolute first instruction.
//!
//! # Root cause 2: ibex forces vectored interrupt mode
//!
//! ibex hardwires `mtvec.MODE=1` (vectored).  Writes of Direct mode are
//! ignored.  In vectored mode, interrupts jump to `mtvec.BASE + 4*cause`,
//! NOT to `mtvec.BASE` (the direct-mode handler).  Without a proper vector
//! table, machine timer interrupt (cause 7) lands 28 bytes into the trap
//! handler — mid-instruction → crash.
//!
//! `select RISCV_VECTORED_MODE` + `GEN_IRQ_VECTOR_TABLE`
//! for both A0 and A1 BootMCU in `zephyr/soc/aspeed/ast27xx/Kconfig`.
//!
//! Fixed by providing a 256-byte–aligned vector table where every entry
//! jumps to `default_start_trap` (riscv-rt's full save/dispatch/restore
//! handler).  `_setup_interrupts` is overridden to set mtvec in Vectored
//! mode pointing to this table.
//!
//! # Boot flow
//!
//! ```text
//! ROM → copies FMC from SPI flash to SRAM (0x14B80A00) → jumps there
//!   → _bootmcu_init (.section .init)
//!       csrwi mie,0 / csrwi mip,0 / GP / SP / j _start_rust
//!   → _start_rust (riscv-rt)
//!       → __pre_init: set mtvec → _vector_table (Vectored)
//!       → .data copy (no-op, LMA=VMA)
//!       → zero .bss
//!       → _setup_interrupts: set mtvec → _vector_table (Vectored)
//!       → main (Embassy executor)
//!   → embassy_aspeed::init() → platform_init_rv() + time_driver_rv::init()
//! ```

use core::arch::global_asm;
use riscv_rt::pre_init;

// ── Safe startup stub ────────────────────────────────────────────────────────
//
// Section .init is KEEP'd and placed BEFORE .init.rust in riscv-rt's link.x.
// No .global _start → no duplicate-symbol conflict with riscv-rt.

global_asm!(
    ".section .init,\"ax\"",
    "_bootmcu_init:",
    "   csrwi mie, 0",
    "   csrwi mip, 0",
    "   .option push",
    "   .option norelax",
    "   la    gp, __global_pointer$",
    "   .option pop",
    "   la    sp, _stack_start",
    "   j     _start_rust",
);

// ── Interrupt vector table for ibex vectored mode ────────────────────────────
//
// ibex hardwires mtvec.MODE=1 (vectored).  On interrupt with cause N, the
// CPU jumps to mtvec.BASE + 4*N.  Each 4-byte slot holds a single `j`
// instruction to the full trap handler.
//
// Alignment: ibex uses mtvec[31:8] as the base, so the table must be
// 256-byte aligned.  Entries 0–11 cover all standard RISC-V causes:
//   [0]  = exceptions (all synchronous traps)
//   [1]  = supervisor software interrupt
//   [3]  = machine software interrupt
//   [5]  = supervisor timer interrupt
//   [7]  = machine timer interrupt  ← Embassy timer driver
//   [9]  = supervisor external interrupt
//   [11] = machine external interrupt
//
// All entries dispatch through default_start_trap (riscv-rt) which
// saves registers, reads mcause, dispatches to the correct Rust handler
// (e.g. MachineTimer), restores registers, and mrets.

global_asm!(
    ".section .trap, \"ax\"",
    ".balign 256",
    ".global _vector_table",
    "_vector_table:",
    ".option push",
    ".option norvc",
    "j default_start_trap", // [0]  exceptions
    "j default_start_trap", // [1]  supervisor software
    "j default_start_trap", // [2]  reserved
    "j default_start_trap", // [3]  machine software
    "j default_start_trap", // [4]  reserved
    "j default_start_trap", // [5]  supervisor timer
    "j default_start_trap", // [6]  reserved
    "j default_start_trap", // [7]  machine timer
    "j default_start_trap", // [8]  reserved
    "j default_start_trap", // [9]  supervisor external
    "j default_start_trap", // [10] reserved
    "j default_start_trap", // [11] machine external
    ".option pop",
);

// ── __pre_init ────────────────────────────────────────────────────────────────

/// Called by riscv-rt before .data copy / .bss zero.
///
/// Sets mtvec to the vector table in Vectored mode so any exception
/// during startup is caught by our handler instead of ROM's.
#[pre_init]
unsafe fn pre_init() {
    extern "C" {
        fn _vector_table();
    }
    riscv::register::mtvec::write(
        _vector_table as *const () as usize,
        riscv::register::mtvec::TrapMode::Vectored,
    );
}

// ── Override riscv-rt's _setup_interrupts ─────────────────────────────────────
//
// riscv-rt's default_setup_interrupts writes mtvec in Direct mode, which
// ibex ignores (forces Vectored).  We override to set Vectored mode
// explicitly, pointing at our vector table.

#[no_mangle]
unsafe extern "Rust" fn _setup_interrupts() {
    extern "C" {
        fn _vector_table();
    }
    riscv::register::mtvec::write(
        _vector_table as *const () as usize,
        riscv::register::mtvec::TrapMode::Vectored,
    );
}

// ── Platform init ─────────────────────────────────────────────────────────────

/// Perform platform-level initialisation after the runtime is ready.
///
/// Called from `embassy_aspeed::init()`.  Enables global machine interrupts
/// (`mstatus.MIE`) so that timer ISRs can fire.
pub fn platform_init_rv() {
    unsafe { riscv::register::mstatus::set_mie() };
}
