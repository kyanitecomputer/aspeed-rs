/* AST1060 (ARM Cortex-M4F) memory layout.
 *
 * Used as `memory.x` by cortex-m-rt (INCLUDE'd at the top of link.x).
 * Build.rs copies this file to OUT_DIR/memory.x and adds OUT_DIR to the
 * linker search path.
 *
 * Sources:
 *   ast1060v19.pdf  v1.9  Section 7.1 — Memory map
 */

/* ── Memory regions ──────────────────────────────────────────────────────────
 *
 * The AST1060 is a standalone Cortex-M4F SoC with:
 *   - 768 KB on-chip SRAM at 0x0000_0000
 *   - SPI flash via FMC CE0, memory-mapped at 0x0000_0000 (boot window)
 *
 * Boot model (XIP):
 *   The FMC maps the boot SPI flash at 0x0000_0000. The CM4F fetches its
 *   initial SP and Reset vector from there. Code executes XIP from flash.
 *   SRAM is used for .data, .bss, and stack.
 *
 * Address split (matches Hubris ast1060 memory.toml reference):
 *   FLASH (rx):  0x0000_0000 – 0x0001_FFFF (128 KB for code + rodata)
 *   RAM   (rwx): 0x0002_0000 – 0x000B_FFFF (640 KB for data/bss/stack)
 *   Total SRAM used = 768 KB (128 + 640).
 *
 *   The CM4F I-Code bus reads instructions from FLASH (FMC → SPI flash).
 *   The CM4F D-Code/AHB bus accesses RAM (on-chip SRAM controller).
 *   These are independent physical buses — no address conflict at runtime.
 *
 * Secure boot header:
 *   The AST1060 Secure Boot MCU reads a 32-byte header at 0x400 (immediately
 *   after the 240-exception vector table: 256 × 4 = 0x400 bytes). The header
 *   is written by the secure boot toolchain, not by this linker script.
 *
 * VTOR:
 *   Cortex-M4F boots with VTOR = 0x0000_0000. The vector table is placed by
 *   cortex-m-rt at ORIGIN(FLASH) = 0x0. No adjustment is needed (unlike the
 *   AST2600 SSP which requires VTOR = 0x400 due to the SBOOT header).
 *
 * FPU:
 *   boot.rs enables the FPU (CPACR CP10/CP11) in pre_init before any
 *   floating-point instructions can execute.
 *
 * Secure Boot:
 *   The AST1060 has its own OTP/Secure Boot Controller (SEC). No ASPEED
 *   sb_header is needed in the linker script — the Secure Boot MCU (SBMCU)
 *   handles firmware measurement independently of the vector table layout.
 */
MEMORY {
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
  RAM   (rwx) : ORIGIN = 0x00020000, LENGTH = 640K
}
