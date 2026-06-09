//! ASPEED Hash and Crypto Engine (HACE) driver.
//!
//! Supports AST1060 and AST2600 SSP.  Base address: `0x7E6D_0000`.
//!
//! # Implemented operations
//!
//! - SHA-1, SHA-224, SHA-256 (32-byte digest)
//! - SHA-384, SHA-512, SHA-512/224, SHA-512/256 (48/64-byte digest)
//! - HMAC-SHA-256 (key up to 64 bytes, 32-byte digest)
//!
//! Crypto engine (AES/DES/RC4) is not implemented in this initial version.
//!
//! # Memory requirements
//!
//! All input buffers and the digest output buffer must be in memory accessible
//! by the HACE DMA engine.  On AST1060/AST2600 Cortex-M, SRAM at 0x0–0xBFFFF
//! and internal SRAM are both accessible.  No cache flushing is required on
//! Cortex-M (no data cache).
//!
//! HMAC requires a 64-byte key buffer aligned to 64 bytes.  Callers must
//! supply a suitably aligned buffer; see [`Hace::hmac_sha256`].
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::hace::Hace;
//!
//! let hace = Hace::new();
//!
//! // SHA-256
//! let mut digest = [0u8; 32];
//! hace.sha256(b"hello", &mut digest).unwrap();
//!
//! // HMAC-SHA-256 (key buffer must be 64-byte aligned)
//! #[repr(align(64))]
//! struct KeyBuf([u8; 64]);
//! let mut key_buf = KeyBuf([0u8; 64]);
//! key_buf.0[..4].copy_from_slice(b"key!");
//! hace.hmac_sha256(b"key!", b"message", &mut key_buf.0, &mut digest).unwrap();
//! ```
//!
//! # References
//!
//! - `drivers/crypto/aspeed/aspeed-hace.h` (vendor Linux 6.18.20)
//! - `data/registers/hace_v1.yaml`
//! - `ast1060v19.pdf` Chapter 18

use core::ptr;

// ── Base address ──────────────────────────────────────────────────────────────

const HACE_BASE: usize = 0x7E6D_0000;

// ── Register offsets (bytes) ──────────────────────────────────────────────────

// Crypto engine (not used in this driver version):
const _CRYPTO_SRC: usize = 0x00;
const _CRYPTO_DST: usize = 0x04;
const _CRYPTO_CTX: usize = 0x08;
const _CRYPTO_DATA_LEN: usize = 0x0C;
const _CRYPTO_CMD: usize = 0x10;
const _GCM_AAD_LEN: usize = 0x14;
const _GCM_TAG_BASE: usize = 0x18;

// Status (shared):
const HACE_STS: usize = 0x1C;

// Hash engine:
const HASH_SRC: usize = 0x20;
const HASH_DIGEST: usize = 0x24;
const HASH_KEY: usize = 0x28;
const HASH_DATA_LEN: usize = 0x2C;
const HASH_CMD: usize = 0x30;

// ── HACE_STS bit definitions ──────────────────────────────────────────────────

const STS_HASH_BUSY: u32 = 1 << 0;
const STS_HASH_ISR: u32 = 1 << 9;
const STS_CRYPTO_ISR: u32 = 1 << 12;

// ── HASH_CMD bit definitions (from aspeed-hace.h) ─────────────────────────────

// Algorithm family [7:4] — selects which SHA family / DES variant.
const HASH_CMD_SHA1: u32 = 0x2 << 4;
const HASH_CMD_SHA224: u32 = 0x4 << 4;
const HASH_CMD_SHA256: u32 = 0x5 << 4;
const HASH_CMD_SHA512_SER: u32 = 0x6 << 4; // "serial" SHA-512 family

// SHA-512 variant [11:10] — only meaningful when HASH_CMD_SHA512_SER is set.
const HASH_CMD_SHA512: u32 = 0x0 << 10; // plain SHA-512
const HASH_CMD_SHA384: u32 = 0x1 << 10;
const HASH_CMD_SHA512_256: u32 = 0x2 << 10;
const HASH_CMD_SHA512_224: u32 = 0x3 << 10;

// HMAC mode [8:7].
const HASH_CMD_NORMAL: u32 = 0x0 << 7;
const HASH_CMD_HMAC: u32 = 0x1 << 7;   // HMAC computation (after key setup)
const HASH_CMD_HMAC_KEY: u32 = 0x3 << 7; // HMAC key pre-processing step

// Interrupt enable.
const HASH_CMD_INT_EN: u32 = 1 << 9;

// DMA mode: scatter-gather source [18].
const HASH_CMD_SG_SRC: u32 = 1 << 18;

// Mbus request sync [20].
const HASH_CMD_MBUS_SYNC: u32 = 1 << 20;

// ── Public types ─────────────────────────────────────────────────────────────

/// SHA algorithm selection.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum HashAlgo {
    Sha1,
    Sha224,
    Sha256,
    Sha384,
    Sha512,
    Sha512_224,
    Sha512_256,
}

impl HashAlgo {
    fn cmd_bits(self) -> u32 {
        match self {
            HashAlgo::Sha1 => HASH_CMD_SHA1,
            HashAlgo::Sha224 => HASH_CMD_SHA224,
            HashAlgo::Sha256 => HASH_CMD_SHA256,
            HashAlgo::Sha384 => HASH_CMD_SHA512_SER | HASH_CMD_SHA384,
            HashAlgo::Sha512 => HASH_CMD_SHA512_SER | HASH_CMD_SHA512,
            HashAlgo::Sha512_224 => HASH_CMD_SHA512_SER | HASH_CMD_SHA512_224,
            HashAlgo::Sha512_256 => HASH_CMD_SHA512_SER | HASH_CMD_SHA512_256,
        }
    }

    /// Digest length in bytes.
    pub fn digest_len(self) -> usize {
        match self {
            HashAlgo::Sha1 => 20,
            HashAlgo::Sha224 | HashAlgo::Sha512_224 => 28,
            HashAlgo::Sha256 | HashAlgo::Sha512_256 => 32,
            HashAlgo::Sha384 => 48,
            HashAlgo::Sha512 => 64,
        }
    }
}

/// Error type for HACE operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum HaceError {
    /// Engine did not complete within the polling timeout.
    Timeout,
    /// Digest output buffer is too small for the selected algorithm.
    BufferTooSmall,
    /// HMAC key buffer is not 64-byte aligned.
    KeyBufMisaligned,
    /// Input data too large (>= 256 MB).
    DataTooLarge,
}

// ── Hace ─────────────────────────────────────────────────────────────────────

/// ASPEED Hash and Crypto Engine handle.
///
/// Stateless wrapper — all state is held in hardware registers.
/// Safe to create multiple times; all operations are blocking.
pub struct Hace;

impl Hace {
    /// Obtain a handle to the HACE.
    ///
    /// Does not perform any hardware initialisation.
    pub fn new() -> Self {
        Self
    }

    /// Compute SHA-256 of `data` into `digest` (must be ≥ 32 bytes).
    pub fn sha256(&self, data: &[u8], digest: &mut [u8]) -> Result<(), HaceError> {
        self.hash(HashAlgo::Sha256, data, digest)
    }

    /// Compute SHA-384 of `data` into `digest` (must be ≥ 48 bytes).
    pub fn sha384(&self, data: &[u8], digest: &mut [u8]) -> Result<(), HaceError> {
        self.hash(HashAlgo::Sha384, data, digest)
    }

    /// Compute SHA-512 of `data` into `digest` (must be ≥ 64 bytes).
    pub fn sha512(&self, data: &[u8], digest: &mut [u8]) -> Result<(), HaceError> {
        self.hash(HashAlgo::Sha512, data, digest)
    }

    /// Compute a hash of `data` using `algo`, storing the result in `digest`.
    ///
    /// `digest` must be at least `algo.digest_len()` bytes long.
    ///
    /// # Errors
    ///
    /// - [`HaceError::BufferTooSmall`] if `digest.len() < algo.digest_len()`.
    /// - [`HaceError::DataTooLarge`] if `data.len() >= 256 MB`.
    /// - [`HaceError::Timeout`] if the engine does not complete in time.
    pub fn hash(&self, algo: HashAlgo, data: &[u8], digest: &mut [u8]) -> Result<(), HaceError> {
        if digest.len() < algo.digest_len() {
            return Err(HaceError::BufferTooSmall);
        }
        if data.len() >= 256 * 1024 * 1024 {
            return Err(HaceError::DataTooLarge);
        }

        let cmd = algo.cmd_bits() | HASH_CMD_NORMAL | HASH_CMD_MBUS_SYNC;
        self.run_hash(data.as_ptr() as usize, data.len(), digest.as_mut_ptr() as usize, 0, cmd)
    }

    /// Compute HMAC-SHA-256.
    ///
    /// - `key`: HMAC key (up to 64 bytes; longer keys are not supported here).
    /// - `data`: message data.
    /// - `key_buf`: caller-supplied 64-byte buffer **aligned to 64 bytes**.
    ///   The key is copied into this buffer before processing.  It will be
    ///   overwritten.
    /// - `digest`: output buffer, must be ≥ 32 bytes.
    ///
    /// # Errors
    ///
    /// - [`HaceError::KeyBufMisaligned`] if `key_buf.as_ptr() % 64 != 0`.
    /// - [`HaceError::BufferTooSmall`] if `digest.len() < 32`.
    /// - [`HaceError::DataTooLarge`] if `data.len() >= 256 MB`.
    /// - [`HaceError::Timeout`] if the engine does not complete in time.
    pub fn hmac_sha256(
        &self,
        key: &[u8],
        data: &[u8],
        key_buf: &mut [u8; 64],
        digest: &mut [u8],
    ) -> Result<(), HaceError> {
        if digest.len() < 32 {
            return Err(HaceError::BufferTooSmall);
        }
        if data.len() >= 256 * 1024 * 1024 {
            return Err(HaceError::DataTooLarge);
        }
        if (key_buf.as_ptr() as usize) % 64 != 0 {
            return Err(HaceError::KeyBufMisaligned);
        }

        // Zero-fill key buffer and copy key.
        key_buf.fill(0);
        let copy_len = key.len().min(64);
        key_buf[..copy_len].copy_from_slice(&key[..copy_len]);

        let key_addr = key_buf.as_ptr() as usize;
        let src_addr = data.as_ptr() as usize;
        let dig_addr = digest.as_mut_ptr() as usize;

        // Step 1: pre-process key (HMAC_KEY mode, data = key_buf itself, len = 64).
        let key_cmd = HASH_CMD_SHA256 | HASH_CMD_HMAC_KEY | HASH_CMD_MBUS_SYNC;
        self.run_hash(key_addr, 64, dig_addr, key_addr, key_cmd)?;

        // Step 2: compute HMAC (HMAC mode, data = actual message).
        let hmac_cmd = HASH_CMD_SHA256 | HASH_CMD_HMAC | HASH_CMD_MBUS_SYNC;
        self.run_hash(src_addr, data.len(), dig_addr, key_addr, hmac_cmd)
    }

    // ── Internal ─────────────────────────────────────────────────────────────

    fn run_hash(
        &self,
        src: usize,
        len: usize,
        digest: usize,
        key: usize,
        cmd: u32,
    ) -> Result<(), HaceError> {
        // Wait for any previous hash operation to complete.
        self.wait_hash_idle()?;

        unsafe {
            // Clear any stale interrupt flags first.
            ptr::write_volatile(reg(HACE_STS), STS_HASH_ISR | STS_CRYPTO_ISR);

            ptr::write_volatile(reg(HASH_SRC), src as u32);
            ptr::write_volatile(reg(HASH_DIGEST), digest as u32);
            ptr::write_volatile(reg(HASH_KEY), key as u32);
            ptr::write_volatile(reg(HASH_DATA_LEN), len as u32);

            // Writing HASH_CMD fires the operation.
            ptr::write_volatile(reg(HASH_CMD), cmd);
        }

        // Poll until done.
        self.wait_hash_done()
    }

    fn wait_hash_idle(&self) -> Result<(), HaceError> {
        const MAX: u32 = 100_000_000;
        let mut n = 0u32;
        loop {
            let s = unsafe { ptr::read_volatile(reg(HACE_STS)) };
            if s & STS_HASH_BUSY == 0 {
                return Ok(());
            }
            n += 1;
            if n > MAX {
                return Err(HaceError::Timeout);
            }
            core::hint::spin_loop();
        }
    }

    fn wait_hash_done(&self) -> Result<(), HaceError> {
        const MAX: u32 = 100_000_000;
        let mut n = 0u32;
        loop {
            let s = unsafe { ptr::read_volatile(reg(HACE_STS)) };
            if s & STS_HASH_BUSY == 0 {
                // Clear interrupt flag.
                unsafe { ptr::write_volatile(reg(HACE_STS), STS_HASH_ISR) };
                return Ok(());
            }
            n += 1;
            if n > MAX {
                return Err(HaceError::Timeout);
            }
            core::hint::spin_loop();
        }
    }
}

impl Default for Hace {
    fn default() -> Self {
        Self::new()
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn reg(byte_offset: usize) -> *mut u32 {
    (HACE_BASE + byte_offset) as *mut u32
}
