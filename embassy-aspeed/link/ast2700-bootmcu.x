/*
 * AST2700 BootMCU (A1) memory layout for riscv-rt.
 *
 * Execution model:
 *   ROM copies FMC binary from SPI flash into GSRAM (SRAM) at 0x14B80A00,
 *   then jumps to the entry point.  Everything — code, read-only data,
 *   mutable data, BSS, stack — lives in SRAM.  SDRAM (0x80000000) is
 *   NOT available until the BootMCU performs DRAM training.
 *
 * Evidence:
 *   ast2700_secure_boot_v0203.md §2.1:
 *     "BootMCU ROM loads the SoC FMC from boot media to SRAM."
 *   u-boot/configs/ibex-ast2700_defconfig:
 *     CONFIG_SPL_TEXT_BASE=0x14b80a00   (.text in SRAM)
 *     CONFIG_SPL_XIP=y                  (run in-place from SRAM, no relocation)
 *   u-boot/arch/riscv/cpu/ast2700/u-boot-spl.lds:
 *     All of .text, .rodata, .data in .spl_mem (SRAM)
 *   zephyr/boards/aspeed/ast2700_evb/ast2700_evb_ast2700_a1_bootmcu.overlay:
 *     sram: memory@14b80a00 { reg = <0x14b80a00 0x2ee00>; };
 *   ast2700v111.md §6.4 BootMCU Address Space:
 *     0x14B80000–0x14BFFFFF  SRAM Memory Buffer (GSRAM), 256 KB
 *     0x80000000–0xBFFFFFFF  SDRAM (requires DRAM training)
 *
 * SRAM layout (GSRAM, 256 KB total at 0x14B80000):
 *   0x14B80000–0x14B809FF  (2560 bytes)  Reserved — ROM places ASTH header here
 *   0x14B80A00–0x14BAF7FF  (~187.5 KB)   FMC binary: .text + .rodata + .data +
 *                                         .bss + heap + stack
 *
 * Other memory available before DRAM training:
 *   0x10000000  ECC SRAM, 128 KB (usable for DMA buffers, heap expansion)
 *   0x20000000  SPI flash XIP window, 512 MB (read-only, prebuilt binaries)
 *
 * Stack: riscv-rt places SP at end of REGION_STACK = 0x14BAF800.
 * This matches the Zephyr A1 overlay SRAM end (0x14B80A00 + 0x2EE00).
 */

MEMORY {
    /*
     * GSRAM: all code and data lives here.
     *
     * Origin = 0x14B80A00: first 0xA00 bytes are the ASTH header placed by ROM.
     * Length = 0x2EE00: Zephyr A1 overlay limit, ends at 0x14BAF800.
     *   This is within the 192 KB SRAM region verified by Zephyr DTS.
     *   (256 KB datasheet size minus reserved/inaccessible upper region.)
     */
    SRAM (rwx) : ORIGIN = 0x14B80A00, LENGTH = 0x2EE00
}

/*
 * riscv-rt REGION_ALIAS — everything in SRAM.
 *
 * With all regions mapped to SRAM, riscv-rt's .data copy is a no-op
 * (LMA = VMA), .bss zeroing is fast, and all runtime accesses are at
 * full SRAM speed.  No SDRAM dependency.
 */
REGION_ALIAS("REGION_TEXT",   SRAM);
REGION_ALIAS("REGION_RODATA", SRAM);
REGION_ALIAS("REGION_DATA",   SRAM);
REGION_ALIAS("REGION_BSS",    SRAM);
REGION_ALIAS("REGION_HEAP",   SRAM);
REGION_ALIAS("REGION_STACK",  SRAM);
