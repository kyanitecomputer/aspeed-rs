/* AST2600 SSP (ARM Cortex-M3) memory layout.
 *
 * Used as `memory.x` by cortex-m-rt (INCLUDE'd at the top of link.x).
 * Build.rs copies this file to OUT_DIR/memory.x and adds OUT_DIR to the
 * linker search path.
 *
 * Sources:
 *   ast2600_ssp.md   v0.1f  — SSP memory map (SCUA04 base, XIP model)
 *   ast2600_evb_ast2600_ssp.dts — Zephyr board DTS (memory regions)
 *   ARM Cortex-M3 TRM — VTOR alignment requirement
 */

/* ── Memory regions ──────────────────────────────────────────────────────────
 *
 * The SSP runs from DRAM (Execute-In-Place).  The CA7 configures the SSP via
 * SCU registers, copies the firmware image to DRAM, and releases the CM3 from
 * reset.  All addresses below are CM3 virtual addresses (DRAM base = 0x0).
 *
 * SBOOT (1 KB) at 0x0:
 *   Contains the ASPEED 8-word (32-byte) secure-boot header, then padding to
 *   1 KB.  The header's first two 32-bit words overlap the CM3 reset vectors:
 *     [0x000] key_location  ← set to _stack_start by the linker (CM3 initial SP)
 *     [0x004] enc_img_addr  ← set to Reset         by the linker (CM3 Reset vector)
 *     [0x008] img_size      ← set to _flash_used   by the linker (for CA7 loader)
 *     [0x00C–0x01C]         ← zeros                 (sign/revision/patch/checksum)
 *     [0x020–0x3FF]         ← zeros                 (padding to 1 KB)
 *   The CA7 loader reads img_size from this header to know how much to copy.
 *
 * VTOR alignment note:
 *   The Cortex-M3 VTOR register requires alignment ≥ (num_exceptions × 4).
 *   For 240 external IRQs + 16 system exceptions = 256 exceptions:
 *     min alignment = 256 × 4 = 1024 = 0x400.
 *   Therefore the vector table MUST start at a 0x400-aligned address.
 *   We set ORIGIN(FLASH) = 0x400 so cortex-m-rt places .vector_table there.
 *   firmware MUST set SCB.VTOR = 0x400 early in pre_init (see boot.rs).
 *
 * FLASH (127 KB) starting at 0x400:
 *   cortex-m-rt places .vector_table at ORIGIN(FLASH) = 0x400.
 *
 * RAM (16,256 KB) starting at 0x20000:
 *   .data, .bss, heap, stack.
 *   _stack_start = 0x20000 + 16256K = 0x20000 + 0xFF0000 = 0x1010000.
 *
 * RAM_NC (16,384 KB) starting at 0x1000000:
 *   Non-cached window (cache area bit 1 must be cleared by CA7 via SCUA40).
 *   DMA descriptors, UART DMA ring buffers, IPC shared-memory live here.
 */
MEMORY {
  SBOOT  (rx) : ORIGIN = 0x00000000, LENGTH = 1K
  FLASH  (rx) : ORIGIN = 0x00000400, LENGTH = 128K - 1K
  RAM   (rwx) : ORIGIN = 0x00020000, LENGTH = (16384 - 128) * 1024
  RAM_NC(rwx) : ORIGIN = 0x01000000, LENGTH = 16384 * 1024
}

/* ── Secure boot header + padding ────────────────────────────────────────────
 *
 * The ASPEED sb_header (8 × u32 = 32 bytes) is generated entirely by the
 * linker.  No Rust symbols in .sboot are needed.
 *
 * key_location and enc_img_addr double as CM3 reset vectors and are set to
 * the correct runtime values (_stack_start, Reset) by LONG() expressions.
 *
 * Note on LONG(Reset):
 *   The Reset symbol is a Thumb function; GNU ld automatically includes bit 0
 *   in the symbol value, so no +1 is needed.
 *
 * The section is padded to 1 KB so that cortex-m-rt's .vector_table lands at
 * ORIGIN(FLASH) = 0x400, satisfying the VTOR alignment requirement above.
 */
SECTIONS {
  .sboot ORIGIN(SBOOT) : {
    LONG(_stack_start & 0xFFFFFFF8); /* [0x000] key_location  = initial SP  */
    LONG(Reset);                      /* [0x004] enc_img_addr  = Reset vector */
    LONG(_flash_used);                /* [0x008] img_size for CA7 loader      */
    LONG(0);                          /* [0x00C] sign_location = 0 (unsigned) */
    LONG(0); LONG(0);                 /* [0x010] header_rev[0..1]             */
    LONG(0);                          /* [0x018] patch_location               */
    LONG(0);                          /* [0x01C] checksum                     */
    . = 0x400;                        /* pad to 1 KB (fills 0x020–0x3FF)      */
  } > SBOOT
}

/* _flash_used = total size of the firmware image (sboot header + vector table
 * + code + rodata + LMA copy of .data).  The CA7 loader uses this to know
 * how many bytes to copy from flash to DRAM.
 * Forward references to .data are resolved at final link time by GNU ld.      */
PROVIDE(_flash_used = LOADADDR(.data) + SIZEOF(.data) - ORIGIN(SBOOT));

/* ── Non-cached region symbols ───────────────────────────────────────────────
 *
 * boot.rs zeros [__RAM_NC_start, __RAM_NC_end) in #[cortex_m_rt::pre_init]
 * before the Rust runtime initialises .bss.
 */
__RAM_NC_start = ORIGIN(RAM_NC);
__RAM_NC_end   = ORIGIN(RAM_NC) + LENGTH(RAM_NC);
