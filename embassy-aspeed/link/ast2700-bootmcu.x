/*
 * AST2700 BootMCU memory layout for riscv-rt.
 *
 * Execution model: firmware is loaded from SPI flash (FMC, 0x20000000)
 * into DRAM (0x80000000) by the BootMCU ROM before jumping to the entry
 * point.  CONFIG_XIP is NOT set — all code and data reside in DRAM.
 *
 * This file is consumed as `memory.x` by riscv-rt's link.x.
 * riscv-rt REGION_ALIAS macros route all sections to DRAM.
 *
 * DMA address translation (firmware reference):
 *   virt >= 0x80000000  →  phys = (virt & ~0x80000000) | 0x4_0000_0000
 *   virt <  0x80000000  →  phys = virt  (1:1)
 *
 * Source: Zephyr dts/riscv/aspeed/ast27xx.dtsi
 *         (dram: memory@80000000 { reg = <0x80000000 0x400000>; })
 */

MEMORY {
    /* 4 MB DRAM — primary execution region (loaded by ROM) */
    RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 4M
}

/* Route all riscv-rt sections to the single RAM region. */
REGION_ALIAS("REGION_TEXT",   RAM);
REGION_ALIAS("REGION_RODATA", RAM);
REGION_ALIAS("REGION_DATA",   RAM);
REGION_ALIAS("REGION_BSS",    RAM);
REGION_ALIAS("REGION_HEAP",   RAM);
REGION_ALIAS("REGION_STACK",  RAM);
