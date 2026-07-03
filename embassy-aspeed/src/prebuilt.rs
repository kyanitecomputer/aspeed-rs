//! ASTH prebuilt binary table reader.
//!
//! The ROM places a 2560-byte ASTH header immediately before the FMC binary
//! in SRAM.  The header's body contains a list of prebuilt entries, each
//! recording the type, size, and SHA-384 digest of a binary stored on SPI
//! flash following the FMC image.
//!
//! This module reads the prebuilt table from SRAM and computes the SPI flash
//! XIP address of each prebuilt binary.  The binaries themselves are accessed
//! via the SPI FMC memory-mapped window at 0x20000000.
//!
//! # ASTH layout on flash (AST2700-A1)
//!
//! ```text
//! 0x00000000   Caliptra FW (CMAN manifest, ~106 KB)
//! 0x00020000   ASTH header (2560 bytes)
//!   +0x000     Preamble (1792 bytes): magic, version, signatures
//!   +0x700     Body (768 bytes):
//!                svn       4B
//!                fmc_size  4B   ← size of FMC binary that follows header
//!                sha384   48B   SHA-384 of FMC binary
//!                entries[]:    type(4B) + size(4B) + sha384(48B) = 56B each
//!                end-mark:     type=0, size=0, sha384=zeros
//! 0x00020A00   FMC binary (fmc_size bytes)
//! 0x00020A00+fmc_size  prebuilt[0]
//! ...
//! ```
//!
//! # Prebuilt types
//!
//! | Type | Content |
//! |------|---------|
//! | 0x01 | DDR4 PMU training IMEM |
//! | 0x02 | DDR4 PMU training DMEM |
//! | 0x03 | DDR4 2D PMU training IMEM |
//! | 0x04 | DDR4 2D PMU training DMEM |
//! | 0x05 | DDR5 PMU training IMEM |
//! | 0x06 | DDR5 PMU training DMEM |
//! | 0x07 | DisplayPort firmware |
//! | 0x08 | UEFI Option ROM |
//!
//! # Memory map
//!
//! SPI FMC XIP window: 0x20000000 – 0x3FFFFFFF (512 MB, read-only).
//! ASTH text offset for A1 silicon: 0x20000 from flash start.
//! FMC binary follows ASTH header at: SPI_BASE + 0x20000 + 2560.
//!
//! # Source
//!
//! AST2700 secure boot guide §2.1 (ASTH header format).
//! Derived independently from header structure; no vendor code copied.

/// SPI FMC XIP base address (BootMCU view).
const SPI_BASE: usize = 0x2000_0000;

/// ASTH flash offset for A1 silicon (text_ofst).
const ASTH_FLASH_OFFSET: usize = 0x0002_0000;

/// ASTH header total size in bytes.
const ASTH_SIZE: usize = 2560; // 0xA00

/// ASTH preamble size (magic + version + signatures).
const ASTH_PREAMBLE_SIZE: usize = 1792; // 0x700

/// Offset of the ASTH body within the header.
const ASTH_BODY_OFFSET: usize = ASTH_PREAMBLE_SIZE;

/// Prebuilt entry size (type + size + sha384 = 4 + 4 + 48 = 56 bytes).
const PREBUILT_ENTRY_SIZE: usize = 56;

/// Maximum number of prebuilt entries in the table.
const MAX_ENTRIES: usize = 12;

/// Known prebuilt entry types.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum PrebuiltType {
    Ddr4TrainImem = 0x01,
    Ddr4TrainDmem = 0x02,
    Ddr4Train2dImem = 0x03,
    Ddr4Train2dDmem = 0x04,
    Ddr5TrainImem = 0x05,
    Ddr5TrainDmem = 0x06,
    DpFw = 0x07,
    UefiOprom = 0x08,
}

/// A single parsed prebuilt entry.
#[derive(Clone, Copy, Debug)]
pub struct PrebuiltEntry {
    pub kind: u32,
    pub size: u32,
}

/// Where the prebuilt binaries are catalogued.
enum Source {
    /// A1: ASTH header on SPI flash lists the prebuilts after the FMC.
    Asth {
        /// FMC binary size (bytes) — used to compute the first prebuilt offset.
        fmc_size: u32,
        /// Up to MAX_ENTRIES prebuilt entries (kind=0 = end-of-table).
        entries: [PrebuiltEntry; MAX_ENTRIES],
        /// Number of valid entries (kind != 0).
        count: usize,
    },
    /// A2: the prebuilts are SoC images inside the FLSH container. The DRAM
    /// training / DP firmware map to FLSH ids ID_SOC_IMAGES_BASE + (kind - 1),
    /// matching cairn's tools/a2/gen-a2-image.sh --soc-image order.
    Flsh(crate::manifest::Manifest),
}

/// Parsed prebuilt table (ASTH on A1, FLSH container on A2).
pub struct PrebuiltTable {
    source: Source,
}

impl PrebuiltTable {
    /// Parse the prebuilt catalogue for the detected silicon: the FLSH container
    /// on A2, otherwise the ASTH header. Falls back to ASTH if A2 FLSH parsing
    /// fails.
    pub fn parse() -> Self {
        let (_dev, hw) = crate::scu::silicon_rev();
        if hw == crate::scu::HwRev::A2 {
            if let Ok(manifest) = crate::manifest::Manifest::parse_for(hw) {
                return Self {
                    source: Source::Flsh(manifest),
                };
            }
        }
        Self::parse_from_spi()
    }

    /// FLSH SoC-image identifier backing prebuilt `kind` (A2).
    fn flsh_id_for(kind: u32) -> u32 {
        crate::flsh::ID_SOC_IMAGES_BASE + kind.saturating_sub(1)
    }

    /// Parse the ASTH header from the SPI flash XIP window (A1).
    ///
    /// Reads the header from `SPI_BASE + ASTH_FLASH_OFFSET` (read-only SPI XIP).
    /// Does not access SRAM — reads directly from the memory-mapped flash window.
    ///
    /// # Safety
    ///
    /// Safe to call after ROM boot; SPI FMC XIP window is always mapped read-only.
    pub fn parse_from_spi() -> Self {
        let header_base = unsafe { aspeed_mmio::MmioBlock::new(SPI_BASE + ASTH_FLASH_OFFSET) };
        let body_base =
            unsafe { aspeed_mmio::MmioBlock::new(SPI_BASE + ASTH_FLASH_OFFSET + ASTH_BODY_OFFSET) };

        // Validate magic: b"ASTH" = 0x48545341 little-endian.
        let magic = header_base.read32(0);
        debug_assert_eq!(magic, 0x4854_5341, "ASTH magic mismatch");

        // Body layout (all u32 little-endian):
        //   [0] = svn
        //   [1] = fmc_size
        //   [2..13] = sha384 digest (12 × u32)
        //   [14..] = prebuilt entries, each 56/4 = 14 u32 words
        //           entry[n].type    = body[14 + n*14 + 0]
        //           entry[n].size    = body[14 + n*14 + 1]
        //           entry[n].sha384  = body[14 + n*14 + 2..14]
        let fmc_size = body_base.read32(4);

        let entry_start = 14usize; // words into body where entries begin
        let mut entries = [PrebuiltEntry { kind: 0, size: 0 }; MAX_ENTRIES];
        let mut count = 0;

        for i in 0..MAX_ENTRIES {
            let base_word = entry_start + i * (PREBUILT_ENTRY_SIZE / 4);
            let kind = body_base.read32(base_word * 4);
            let size = body_base.read32((base_word + 1) * 4);
            if kind == 0 {
                break; // end-of-table marker
            }
            entries[i] = PrebuiltEntry { kind, size };
            count += 1;
        }

        Self {
            source: Source::Asth {
                fmc_size,
                entries,
                count,
            },
        }
    }

    /// Find a prebuilt entry by type.
    pub fn find(&self, kind: u32) -> Option<PrebuiltEntry> {
        match &self.source {
            Source::Asth { entries, count, .. } => {
                entries[..*count].iter().find(|e| e.kind == kind).copied()
            }
            Source::Flsh(manifest) => manifest
                .find(Self::flsh_id_for(kind))
                .map(|img| PrebuiltEntry {
                    kind,
                    size: img.size,
                }),
        }
    }

    /// Return a read-only byte slice into the SPI flash XIP window for
    /// the prebuilt entry with the given type.
    ///
    /// Returns `None` if the type is not in the table.
    ///
    /// # Safety
    ///
    /// The returned slice points into the read-only SPI XIP window
    /// (0x20000000+). Callers must not write to it.
    pub fn spi_slice(&self, kind: u32) -> Option<&'static [u8]> {
        match &self.source {
            // A2: the training/DP firmware is an image in the FLSH container.
            Source::Flsh(manifest) => manifest.image_slice(Self::flsh_id_for(kind)).ok(),

            // A1: walk the ASTH prebuilt list on flash.
            Source::Asth {
                fmc_size,
                entries,
                count,
            } => {
                let entry = entries[..*count].iter().find(|e| e.kind == kind)?;

                // Compute flash offset:
                //   ASTH at ASTH_FLASH_OFFSET
                //   FMC  at ASTH_FLASH_OFFSET + ASTH_SIZE
                //   prebuilt[0] at align16(FMC end)
                //   prebuilt[n] at align16(prebuilt[n-1] end)
                //
                // gen-spi-image.py aligns BEFORE each entry (including the first).
                let mut offset = ASTH_FLASH_OFFSET + ASTH_SIZE + *fmc_size as usize;
                for e in &entries[..*count] {
                    // Align to 16-byte boundary BEFORE each entry.
                    offset = (offset + 15) & !15;
                    if e.kind == kind {
                        let ptr = (SPI_BASE + offset) as *const u8;
                        return Some(unsafe {
                            core::slice::from_raw_parts(ptr, entry.size as usize)
                        });
                    }
                    offset += e.size as usize;
                }
                None
            }
        }
    }
}
