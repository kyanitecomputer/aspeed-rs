//! AST2700-A2 "FLSH" flash container parser.
//!
//! A2 replaces the A1 ASTH secure-boot header with a top-level "FLSH" flash
//! container that the MCU ROM / BootMCU FMC parses to locate the Caliptra
//! firmware, the SoC (auth) manifest, the MCU runtime and any additional SoC
//! images; each image is then authorized through Caliptra using the digests
//! carried in the SoC manifest.
//!
//! This is the read side of the writer in `cairn` (tools/imgtools/flsh.go), a
//! byte-for-byte port of the caliptra-mcu-sw flash-image builder at the pinned
//! revision `2b7837402328ab611968d40243075082469df7ae`, verified against the
//! official ASPEED A2 image. Keep both in sync.
//!
//! Layout (all little-endian except the ASCII magic):
//!
//! ```text
//!   Header       magic[4]="FLSH" | version u16=1 | image_count u16
//!   Checksums    header_crc32 u32 | payload_crc32 u32
//!   ImageInfo[n] identifier u32 | image_offset u32 | size u32   (12 bytes each)
//!   Images       raw data, each padded to a 4-byte boundary, in table order
//! ```
//!
//!   - `image_offset` is absolute from byte 0 of the Header.
//!   - `size` is the padded (4-byte-aligned) length.
//!   - `header_crc32`  = CRC-32/IEEE over the 8-byte Header.
//!   - `payload_crc32` = CRC-32/IEEE over the ImageInfo table + all images.
//!
//! The parser reads only 32-bit words at 4-byte-aligned offsets so it is safe
//! over a SPI XIP window (which may not support arbitrary byte-width reads).

/// FLSH magic ("FLSH" ASCII read as a little-endian u32: 'F','L','S','H').
pub const MAGIC: u32 = 0x4853_4C46;
/// Supported header version.
pub const VERSION: u16 = 1;

/// Header size: magic(4) + version(2) + image_count(2).
pub const HEADER_SIZE: usize = 8;
/// Checksum block size: header_crc32(4) + payload_crc32(4).
pub const CKSUM_SIZE: usize = 8;
/// Image-info entry size: identifier(4) + image_offset(4) + size(4).
pub const INFO_SIZE: usize = 12;

// ── Image identifiers (caliptra-mcu-sw builder) ─────────────────────────────
pub const ID_CALIPTRA: u32 = 0x0000_0001;
pub const ID_SOC_MANIFEST: u32 = 0x0000_0002;
pub const ID_MCU_RT: u32 = 0x0000_0003;
/// Additional SoC images are numbered from here, incrementing.
pub const ID_SOC_IMAGES_BASE: u32 = 0x0000_1000;

/// Maximum number of image entries parsed from a container.
pub const MAX_IMAGES: usize = 32;

/// Byte offset of the ImageInfo table within the container.
pub const INFO_TABLE_OFFSET: usize = HEADER_SIZE + CKSUM_SIZE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlshError {
    /// Header magic mismatch.
    BadMagic,
    /// Header version not supported.
    BadVersion,
    /// Image count exceeds [`MAX_IMAGES`].
    TooManyImages,
    /// Requested identifier not present.
    ImageNotFound,
}

/// A parsed image descriptor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImageInfo {
    pub identifier: u32,
    /// Absolute byte offset from the container base.
    pub offset: u32,
    /// Padded (4-byte-aligned) size in bytes.
    pub size: u32,
}

/// A parsed FLSH container header + image table (no image data copied).
#[derive(Clone, Copy, Debug)]
pub struct Container {
    pub version: u16,
    pub header_crc32: u32,
    pub payload_crc32: u32,
    images: [ImageInfo; MAX_IMAGES],
    count: usize,
}

impl Container {
    /// Parse a container using a 32-bit word reader.
    ///
    /// `read(off)` must return the little-endian u32 stored at byte offset
    /// `off` (a multiple of 4) from the container base.
    pub fn parse<F: Fn(usize) -> u32>(read: F) -> Result<Self, FlshError> {
        if read(0) != MAGIC {
            return Err(FlshError::BadMagic);
        }
        let w1 = read(4);
        let version = (w1 & 0xFFFF) as u16;
        let count = (w1 >> 16) as usize;
        if version != VERSION {
            return Err(FlshError::BadVersion);
        }
        if count > MAX_IMAGES {
            return Err(FlshError::TooManyImages);
        }

        let header_crc32 = read(8);
        let payload_crc32 = read(12);

        let mut images = [ImageInfo {
            identifier: 0,
            offset: 0,
            size: 0,
        }; MAX_IMAGES];
        for (i, slot) in images.iter_mut().take(count).enumerate() {
            let base = INFO_TABLE_OFFSET + i * INFO_SIZE;
            *slot = ImageInfo {
                identifier: read(base),
                offset: read(base + 4),
                size: read(base + 8),
            };
        }

        Ok(Container {
            version,
            header_crc32,
            payload_crc32,
            images,
            count,
        })
    }

    /// Parsed image descriptors, in table order.
    pub fn images(&self) -> &[ImageInfo] {
        &self.images[..self.count]
    }

    /// Find an image descriptor by identifier.
    pub fn find(&self, identifier: u32) -> Result<ImageInfo, FlshError> {
        self.images()
            .iter()
            .find(|img| img.identifier == identifier)
            .copied()
            .ok_or(FlshError::ImageNotFound)
    }
}

/// CRC-32/IEEE (reflected, poly 0xEDB88320), matching Go's `hash/crc32`
/// `ChecksumIEEE` and the caliptra-mcu-sw builder. Bitwise (no table) to keep
/// the footprint small; used to validate container integrity.
pub fn crc32_ieee(bytes: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in bytes {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fixed-capacity, alloc-free byte buffer (the crate is `no_std`).
    struct Buf {
        data: [u8; 512],
        len: usize,
    }
    impl Buf {
        fn new() -> Self {
            Buf {
                data: [0; 512],
                len: 0,
            }
        }
        fn push(&mut self, bytes: &[u8]) {
            self.data[self.len..self.len + bytes.len()].copy_from_slice(bytes);
            self.len += bytes.len();
        }
        fn as_slice(&self) -> &[u8] {
            &self.data[..self.len]
        }
    }

    fn pad4(n: usize) -> usize {
        (n + 3) & !3
    }

    /// Build a FLSH container identical to the `cairn` Go writer, so this
    /// mirrors tools/imgtools/flsh_test.go and the caliptra-mcu-sw builder test.
    fn build(images: &[(u32, &[u8])]) -> Buf {
        let info_base = INFO_TABLE_OFFSET + INFO_SIZE * images.len();
        let mut info = Buf::new();
        let mut off = info_base as u32;
        for (id, d) in images {
            let plen = pad4(d.len()) as u32;
            info.push(&id.to_le_bytes());
            info.push(&off.to_le_bytes());
            info.push(&plen.to_le_bytes());
            off += plen;
        }

        let mut payload = Buf::new();
        payload.push(info.as_slice());
        for (_, d) in images {
            payload.push(d);
            payload.push(&[0u8; 4][..pad4(d.len()) - d.len()]);
        }

        let mut hdr = [0u8; HEADER_SIZE];
        hdr[0..4].copy_from_slice(b"FLSH");
        hdr[4..6].copy_from_slice(&VERSION.to_le_bytes());
        hdr[6..8].copy_from_slice(&(images.len() as u16).to_le_bytes());

        let mut out = Buf::new();
        out.push(&hdr);
        out.push(&crc32_ieee(&hdr).to_le_bytes());
        out.push(&crc32_ieee(payload.as_slice()).to_le_bytes());
        out.push(payload.as_slice());
        out
    }

    /// Read a little-endian u32 at byte offset `off` from a buffer.
    fn reader(buf: &[u8]) -> impl Fn(usize) -> u32 + '_ {
        move |off| u32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
    }

    #[test]
    fn parses_five_images_like_go_writer() {
        let caliptra = b"Caliptra Firmware Data - ABCDEFGH".as_slice();
        let manifest = b"Soc Manifest Data - 123456789".as_slice();
        let mcu = b"MCU Runtime Data - QWERTYUI".as_slice();
        let soc1 = b"Soc Image 1 Data - ZXCVBNMLKJ".as_slice();
        let soc2 = b"Soc Image 2 Data - POIUYTREWQ".as_slice();

        let imgs: [(u32, &[u8]); 5] = [
            (ID_CALIPTRA, caliptra),
            (ID_SOC_MANIFEST, manifest),
            (ID_MCU_RT, mcu),
            (ID_SOC_IMAGES_BASE, soc1),
            (ID_SOC_IMAGES_BASE + 1, soc2),
        ];
        let buf = build(&imgs);
        let buf = buf.as_slice();

        // Magic stored as ASCII "FLSH".
        assert_eq!(&buf[0..4], b"FLSH");
        // header_crc over [0..8], payload_crc over [16..].
        assert_eq!(
            u32::from_le_bytes(buf[8..12].try_into().unwrap()),
            crc32_ieee(&buf[0..8])
        );
        assert_eq!(
            u32::from_le_bytes(buf[12..16].try_into().unwrap()),
            crc32_ieee(&buf[16..])
        );

        let c = Container::parse(reader(&buf)).unwrap();
        assert_eq!(c.version, VERSION);
        assert_eq!(c.images().len(), 5);
        assert_eq!(c.header_crc32, crc32_ieee(&buf[0..8]));
        assert_eq!(c.payload_crc32, crc32_ieee(&buf[16..]));

        for (i, (id, body)) in imgs.iter().enumerate() {
            let info = c.find(*id).unwrap();
            assert_eq!(info.identifier, *id);
            assert_eq!(info.size as usize, (body.len() + 3) & !3);
            let start = info.offset as usize;
            assert_eq!(&buf[start..start + body.len()], *body);
            // Table order is preserved.
            assert_eq!(c.images()[i], info);
        }
    }

    #[test]
    fn crc32_matches_known_vector() {
        // CRC-32/IEEE of "123456789" is the standard check value 0xCBF43926.
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn rejects_bad_magic() {
        let read = |off: usize| if off == 0 { 0xDEAD_BEEF } else { 0 };
        assert!(matches!(Container::parse(read), Err(FlshError::BadMagic)));
    }
}
