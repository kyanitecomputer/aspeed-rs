//! AST2700 firmware image loader.
//!
//! Supports two flash layouts:
//!
//! ## Layout A — CMAN bundle (ASPEED secure boot, production)
//!
//! ```text
//! 0x00100000  cptra_manifest_hdr: magic=0x48534C46, img_count
//! +0x08       cptra_checksum_info (8 bytes, skipped — ROM verified)
//! +0x10       cptra_image_info[N]: identifier(4) + offset(4) + size(4)
//! +offset     Image data (ATF, OP-TEE, U-Boot, SSP, TSP, ...)
//! ```
//!
//! ## Layout B — Raw payload at fixed flash offset (development / Tamago)
//!
//! For development (no Caliptra secure boot), load a raw binary directly
//! from a known flash offset. Use `RawLoader` instead of `Manifest`.
//!
//! ## Sources
//!
//! `board_ast2700.c`. Values re-expressed as Rust — no C code copied.

use aspeed_mmio::MmioBlock;

/// SPI FMC XIP window base (BootMCU view).
const SPI_BASE: usize = 0x2000_0000;

/// A1 silicon: the FLSH bundle starts at 0x100000 from flash start.
/// (ROM loads Caliptra FW at offset 0, ASTH at 0x20000, CMAN/FLSH at 0x100000)
const MANIFEST_FLASH_OFFSET_A1: usize = 0x0010_0000;

/// A2 silicon: the entire flash IS one top-level FLSH container starting at
/// byte 0 (Caliptra FW, SoC manifest, MCU runtime and SoC images are all
/// entries within it), so the container base is flash offset 0.
const MANIFEST_FLASH_OFFSET_A2: usize = 0x0000_0000;

/// Max image entries in the manifest (must match [`crate::flsh::MAX_IMAGES`]).
const MAX_IMG_COUNT: usize = crate::flsh::MAX_IMAGES;

// ── Image identifiers (cptra_manifest_hdr image identifier values) ────────────

pub const HDR_ID_SOC_MANIFEST: u32 = 0x0002;
pub const HDR_ID_FMC: u32 = 0x0003;
pub const HDR_ID_DDR4_IMEM: u32 = 0x1000;
pub const HDR_ID_DDR4_DMEM: u32 = 0x1001;
pub const HDR_ID_ATF: u32 = 0x1008;
pub const HDR_ID_OPTEE: u32 = 0x1009;
pub const HDR_ID_UBOOT: u32 = 0x100A;
pub const HDR_ID_SSP: u32 = 0x100B;
pub const HDR_ID_TSP: u32 = 0x100C;

// ── Image load addresses ──────────────────

/// ATF (TF-A Secure Monitor) load address in DRAM.
pub const ATF_LOAD_ADDR: usize = 0xB000_0000;
/// OP-TEE load address.
pub const OPTEE_LOAD_ADDR: usize = 0xB008_0000;
/// U-Boot load address.
pub const UBOOT_LOAD_ADDR: usize = 0x8000_0000;
/// SSP (Secure Service Processor) firmware load address.
pub const SSP_LOAD_ADDR: usize = 0xAC00_0000;
/// TSP (Trusted Service Processor) firmware load address.
pub const TSP_LOAD_ADDR: usize = 0xAE00_0000;

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ManifestError {
    /// CMAN header magic mismatch.
    BadMagic,
    /// Image count exceeds maximum.
    TooManyImages,
    /// Requested image identifier not found.
    ImageNotFound,
    /// Image offset/size would overflow flash window.
    InvalidImageInfo,
}

// ── Parsed image descriptor ───────────────────────────────────────────────────

#[derive(Clone, Copy, Debug)]
pub struct ImageInfo {
    pub identifier: u32,
    /// Byte offset from CMAN bundle start.
    pub offset: u32,
    pub size: u32,
}

// ── Manifest ──────────────────────────────────────────────────────────────────

/// Parsed CMAN bundle header — holds image descriptors from SPI XIP memory.
pub struct Manifest {
    bundle_base: usize, // absolute byte address in SPI XIP window
    images: [ImageInfo; MAX_IMG_COUNT],
    count: usize,
}

impl Manifest {
    /// Parse the FLSH bundle for A1 silicon (bundle at SPI offset 0x100000).
    ///
    /// Prefer [`Manifest::parse_for`] on systems that may be A2.
    pub fn parse() -> Result<Self, ManifestError> {
        Self::parse_at(SPI_BASE + MANIFEST_FLASH_OFFSET_A1)
    }

    /// Parse the FLSH bundle for the given silicon revision, selecting the
    /// container base automatically (A2 = flash byte 0; A0/A1 = 0x100000).
    pub fn parse_for(hw: crate::scu::HwRev) -> Result<Self, ManifestError> {
        let offset = match hw {
            crate::scu::HwRev::A2 => MANIFEST_FLASH_OFFSET_A2,
            _ => MANIFEST_FLASH_OFFSET_A1,
        };
        Self::parse_at(SPI_BASE + offset)
    }

    /// Parse the FLSH container located at the given absolute XIP base.
    ///
    /// Checksum verification is skipped — the ROM already verified the bundle.
    /// Reads are XIP-safe 32-bit words via the shared [`crate::flsh`] parser,
    /// keeping the container format in one place (byte-verified against the
    /// `cairn` writer and the official ASPEED A2 image).
    pub fn parse_at(bundle_base: usize) -> Result<Self, ManifestError> {
        let container = crate::flsh::Container::parse(|off| rd32(bundle_base + off))
            .map_err(|e| match e {
                crate::flsh::FlshError::TooManyImages => ManifestError::TooManyImages,
                crate::flsh::FlshError::ImageNotFound => ManifestError::ImageNotFound,
                // BadMagic / BadVersion both mean the bundle header is invalid.
                _ => ManifestError::BadMagic,
            })?;

        let mut images = [ImageInfo {
            identifier: 0,
            offset: 0,
            size: 0,
        }; MAX_IMG_COUNT];
        let parsed = container.images();
        for (slot, im) in images.iter_mut().zip(parsed.iter()) {
            *slot = ImageInfo {
                identifier: im.identifier,
                offset: im.offset,
                size: im.size,
            };
        }

        Ok(Manifest {
            bundle_base,
            images,
            count: parsed.len(),
        })
    }

    /// Find an image descriptor by identifier.
    pub fn find(&self, identifier: u32) -> Option<ImageInfo> {
        self.images[..self.count]
            .iter()
            .find(|img| img.identifier == identifier)
            .copied()
    }

    /// Absolute XIP address of an image's first byte (container base + offset).
    /// Lets the caller read a small in-image header (e.g. the CA35 boot header)
    /// with 32-bit XIP reads before copying the payload.
    pub fn image_addr(&self, identifier: u32) -> Result<usize, ManifestError> {
        let img = self.find(identifier).ok_or(ManifestError::ImageNotFound)?;
        Ok(self.bundle_base + img.offset as usize)
    }

    /// Return a byte slice into the SPI XIP window for the given image identifier.
    pub fn image_slice(&self, identifier: u32) -> Result<&'static [u8], ManifestError> {
        let img = self.find(identifier).ok_or(ManifestError::ImageNotFound)?;
        if img.size == 0 {
            return Err(ManifestError::InvalidImageInfo);
        }
        let ptr = (self.bundle_base + img.offset as usize) as *const u8;
        Ok(unsafe { core::slice::from_raw_parts(ptr, img.size as usize) })
    }

    /// Copy an image from SPI flash XIP window to a destination address in DRAM.
    ///
    /// Uses 32-bit word copies (same as memcpy32).
    ///
    /// # Safety
    ///
    /// `dst..dst+image_size` must be a valid writable DRAM range and must not
    /// overlap with any live Rust references.
    pub unsafe fn load_image(&self, identifier: u32, dst: usize) -> Result<u32, ManifestError> {
        let img = self.find(identifier).ok_or(ManifestError::ImageNotFound)?;
        if img.size == 0 {
            return Err(ManifestError::InvalidImageInfo);
        }

        let src = self.bundle_base + img.offset as usize;
        unsafe { copy32(dst, src, img.size as usize) };
        Ok(img.size)
    }

    /// Load all known boot images to their destination DRAM addresses.
    ///
    /// Loads: ATF, OP-TEE, U-Boot, SSP, TSP (skips any not present).
    /// Returns a bitmask of which images were loaded (`LOADED_*` constants).
    pub fn load_boot_images(&self) -> LoadedImages {
        let mut loaded = LoadedImages::default();

        if let Ok(sz) = unsafe { self.load_image(HDR_ID_ATF, ATF_LOAD_ADDR) } {
            loaded.atf = true;
            loaded.atf_size = sz;
        }
        if let Ok(sz) = unsafe { self.load_image(HDR_ID_OPTEE, OPTEE_LOAD_ADDR) } {
            loaded.optee = true;
            loaded.optee_size = sz;
        }
        if let Ok(sz) = unsafe { self.load_image(HDR_ID_UBOOT, UBOOT_LOAD_ADDR) } {
            loaded.uboot = true;
            loaded.uboot_size = sz;
        }
        if let Ok(sz) = unsafe { self.load_image(HDR_ID_SSP, SSP_LOAD_ADDR) } {
            loaded.ssp = true;
            loaded.ssp_size = sz;
        }
        if let Ok(sz) = unsafe { self.load_image(HDR_ID_TSP, TSP_LOAD_ADDR) } {
            loaded.tsp = true;
            loaded.tsp_size = sz;
        }

        loaded
    }
}

/// Which boot firmware images were successfully loaded.
#[derive(Default, Debug)]
pub struct LoadedImages {
    pub atf: bool,
    pub atf_size: u32,
    pub optee: bool,
    pub optee_size: u32,
    pub uboot: bool,
    pub uboot_size: u32,
    pub ssp: bool,
    pub ssp_size: u32,
    pub tsp: bool,
    pub tsp_size: u32,
}

// ── Helpers ───────────────────────────────────────────────────────────────────

#[inline(always)]
fn rd32(addr: usize) -> u32 {
    let regs = unsafe { MmioBlock::new(addr) };
    regs.read32(0)
}

/// Copy `len` bytes from `src` to `dst` using 32-bit word stores.
/// `len` is rounded up to the nearest 4-byte boundary.
///
/// A RISC-V fence is issued after the copy to ensure all writes are visible
/// to other bus masters (e.g. CA35) before returning.
///
/// # Safety
///
/// `src..src+len` must be a valid readable range. `dst..dst+len` must be a
/// valid writable range and must not overlap with any live Rust references.
pub unsafe fn copy32(dst: usize, src: usize, len: usize) {
    let words = (len + 3) / 4;
    let src_regs = unsafe { MmioBlock::new(src) };
    let mut dst_regs = unsafe { MmioBlock::new(dst) };
    for i in 0..words {
        let off = i * 4;
        let v = src_regs.read32(off);
        dst_regs.write32(off, v);
    }
    unsafe { core::arch::asm!("fence iorw, iorw") };
}

// ── Raw payload loader (development / Tamago path) ───────────────────────────

/// Flash offset where the raw A35 payload binary is placed.
/// Chosen to be well past the ASTH prebuilts (~512 KB) and CMAN region.
/// gen-spi-image.py uses `--a35-payload` to place the binary here.
pub const RAW_A35_PAYLOAD_FLASH_OFFSET: usize = 0x0080_0000; // 8 MB mark

/// Flash offset of the 16-byte A35 boot header written by imgtools spi-image.
/// Layout (little-endian u32 words): [magic, entry_off, payload_len, check].
/// `entry_off` is the byte offset of the A35 entry point (_rt0) within the raw
/// payload, i.e. e_entry - link text base. Lets the BootMCU jump to the entry
/// without a hardcoded offset that breaks when the payload size changes.
pub const RAW_A35_HEADER_FLASH_OFFSET: usize = 0x007F_0000;

/// Magic for [`RAW_A35_HEADER_FLASH_OFFSET`] (word 0).
pub const RAW_A35_HEADER_MAGIC: u32 = 0xA35E_B007;

/// Load a raw binary from a fixed SPI flash offset to a DRAM destination.
///
/// No headers, no checksums — the binary is copied verbatim.
/// Used for development payloads (bare-metal Rust, Tamago) where Caliptra
/// secure boot is not active.
///
/// `flash_offset`: byte offset from start of flash (0 = SPI_BASE).
/// `dst`: destination address in DRAM.
/// `len`: number of bytes to copy.
///
/// # Safety
///
/// `dst..dst+len` must be a valid writable DRAM range and must not overlap
/// with any live Rust references.
pub unsafe fn load_raw(flash_offset: usize, dst: usize, len: usize) {
    let src = SPI_BASE + flash_offset;
    unsafe { copy32(dst, src, len) };
}
