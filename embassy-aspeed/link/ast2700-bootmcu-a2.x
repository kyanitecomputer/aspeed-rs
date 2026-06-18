/*
 * AST2700 BootMCU (A2) memory layout for riscv-rt.
 *
 * A2 differs from A1 in the FMC load base. On A1 the ROM reserved the first
 * 0xA00 bytes of GSRAM for the ASTH header and placed the FMC text at
 * 0x14B80A00. A2 boots from a Caliptra "FLSH" container (no ASTH header), so
 * the ROM loads the MCU-runtime FMC at the base of GSRAM, 0x14B80000, and
 * jumps there.
 *
 * Evidence (vendor sources, this tree):
 *   zephyr/boards/aspeed/ast2700_evb/ast2700_evb_ast2700_bootmcu.dts (A2):
 *     sram: memory@14b80000 { reg = <0x14b80000 0x2d400>; };
 *   vs the A1 board overlay:
 *     sram: memory@14b80a00 { reg = <0x14b80a00 0x2ca00>; };
 *   Both end at 0x14BAD400; A2 reclaims the 0xA00 ASTH header region.
 *
 * SRAM (GSRAM) usable: 0x14B80000 .. 0x14BAD400 (0x2D400, ~181 KB) for
 * .text + .rodata + .data + .bss + heap + stack.
 *
 * Everything else matches A1: SDRAM (0x80000000) needs DRAM training, the SPI
 * XIP window is at 0x20000000.
 */

MEMORY {
    SRAM (rwx) : ORIGIN = 0x14B80000, LENGTH = 0x2D400
}

REGION_ALIAS("REGION_TEXT",   SRAM);
REGION_ALIAS("REGION_RODATA", SRAM);
REGION_ALIAS("REGION_DATA",   SRAM);
REGION_ALIAS("REGION_BSS",    SRAM);
REGION_ALIAS("REGION_HEAP",   SRAM);
REGION_ALIAS("REGION_STACK",  SRAM);
