//! Hash and Crypto Engine (HACE) driver — PAC-based, async ISR-driven.
//!
//! Source of truth: `aspeed-data/data/registers/hace_v1.yaml`.
//!
//! Base: `pac::HACE` at `0x7E6D_0000`.  NVIC IRQ: **4** (`HACE`).
//!
//! # Implemented operations
//!
//! - SHA-1, SHA-224, SHA-256, SHA-384, SHA-512, SHA-512/224, SHA-512/256
//! - HMAC-SHA-256 (key ≤ 64 bytes, 32-byte output)
//!
//! # Async model
//!
//! 1. Check engine idle (blocking spin — should be immediate).
//! 2. Enable `HASH_CMD.HASH_INT_EN` so the NVIC fires on completion.
//! 3. Write `HASH_CMD` (fires the DMA).
//! 4. Await `HashWaitFuture` which registers `HACE_WAKER` and yields.
//! 5. ISR (`HACE`, IRQ 4) clears `HACE_STATUS.HASH_INT` and wakes the task.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::hace::{Hace, HashAlgo};
//!
//! let hace = Hace::new();
//! let mut digest = [0u8; 32];
//! hace.sha256(b"hello world", &mut digest).await.unwrap();
//! ```

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::pac;

// ── Global waker ──────────────────────────────────────────────────────────────

static HACE_WAKER: AtomicWaker = AtomicWaker::new();

// ── HASH_CMD field encodings (from hace_v1.yaml HACE_HASH_CMD fieldset) ───────

// HASH_ALGO field values (bits[6:4]).
#[allow(dead_code)]
const ALGO_MD5: u8 = 0x0; // reserved for future MD5 support
const ALGO_SHA1: u8 = 0x2;
const ALGO_SHA224: u8 = 0x4;
const ALGO_SHA256: u8 = 0x5;
const ALGO_SHA512SER: u8 = 0x6; // SHA-512 series; variant in SHA512_SEL[12:10]

// SHA512_SEL values (bits[12:10] — only meaningful when ALGO=SHA512SER).
const SHA512_SEL_SHA512: u8 = 0;
const SHA512_SEL_SHA384: u8 = 1;
const SHA512_SEL_SHA512_256: u8 = 2;
const SHA512_SEL_SHA512_224: u8 = 3;

// HMAC_CMD field values (bits[8:7]).
const HMAC_NORMAL: u8 = 0; // plain hash
const HMAC_HMAC: u8 = 1; // HMAC computation
const HMAC_KEY: u8 = 3; // HMAC key pre-processing

// Build HASH_CMD from named fields (from hace_v1.yaml HACE_HASH_CMD):
//   CASCADE_MODE [1:0] = 0
//   BYTE_SWAP    [3:2] = 2 (SHA big-endian swap)
//   HASH_ALGO    [6:4]
//   HMAC_CMD     [8:7]
//   HASH_INT_EN  [9]   = 1 (interrupt on done)
//   SHA512_SEL   [12:10]
//   MBUS_SYNC    [20]  = 1
fn hash_cmd(algo: u8, sha512_sel: u8, hmac: u8) -> u32 {
    let mut cmd = pac::hace_v1::HACE_HASH_CMD(0);
    cmd.set_BYTE_SWAP(2); // SHA big-endian
    cmd.set_HASH_ALGO(algo);
    cmd.set_HMAC_CMD(hmac);
    cmd.set_HASH_INT_EN(true); // fire IRQ on completion
    cmd.set_SHA512_SEL(sha512_sel);
    cmd.set_MBUS_SYNC(true); // M-Bus request sync
    cmd.0
}

// ── HashAlgo ──────────────────────────────────────────────────────────────────

/// Hash algorithm selection.
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
    fn cmd_word(self) -> u32 {
        match self {
            HashAlgo::Sha1 => hash_cmd(ALGO_SHA1, 0, HMAC_NORMAL),
            HashAlgo::Sha224 => hash_cmd(ALGO_SHA224, 0, HMAC_NORMAL),
            HashAlgo::Sha256 => hash_cmd(ALGO_SHA256, 0, HMAC_NORMAL),
            HashAlgo::Sha384 => hash_cmd(ALGO_SHA512SER, SHA512_SEL_SHA384, HMAC_NORMAL),
            HashAlgo::Sha512 => hash_cmd(ALGO_SHA512SER, SHA512_SEL_SHA512, HMAC_NORMAL),
            HashAlgo::Sha512_224 => hash_cmd(ALGO_SHA512SER, SHA512_SEL_SHA512_224, HMAC_NORMAL),
            HashAlgo::Sha512_256 => hash_cmd(ALGO_SHA512SER, SHA512_SEL_SHA512_256, HMAC_NORMAL),
        }
    }

    /// Digest output length in bytes.
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

// ── HaceError ────────────────────────────────────────────────────────────────

/// HACE operation error.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum HaceError {
    /// `digest` buffer shorter than `algo.digest_len()`.
    BufferTooSmall,
    /// HMAC key buffer not aligned to 64 bytes.
    KeyBufMisaligned,
    /// Input data ≥ 256 MB (hardware limit).
    DataTooLarge,
}

// ── Hace ─────────────────────────────────────────────────────────────────────

/// ASPEED Hash and Crypto Engine driver.
///
/// Stateless; all hardware state lives in `pac::HACE` registers.
/// Not safe to use concurrently from multiple tasks without a `Mutex`.
pub struct Hace;

impl Hace {
    /// Obtain a HACE handle.  No hardware initialisation performed.
    pub const fn new() -> Self {
        Self
    }

    /// SHA-256 of `data` → `digest` (must be ≥ 32 bytes).
    pub async fn sha256(&self, data: &[u8], digest: &mut [u8]) -> Result<(), HaceError> {
        self.hash(HashAlgo::Sha256, data, digest).await
    }

    /// SHA-384 of `data` → `digest` (must be ≥ 48 bytes).
    pub async fn sha384(&self, data: &[u8], digest: &mut [u8]) -> Result<(), HaceError> {
        self.hash(HashAlgo::Sha384, data, digest).await
    }

    /// SHA-512 of `data` → `digest` (must be ≥ 64 bytes).
    pub async fn sha512(&self, data: &[u8], digest: &mut [u8]) -> Result<(), HaceError> {
        self.hash(HashAlgo::Sha512, data, digest).await
    }

    /// Hash `data` using `algo` → `digest` (must be ≥ `algo.digest_len()` bytes).
    pub async fn hash(
        &self,
        algo: HashAlgo,
        data: &[u8],
        digest: &mut [u8],
    ) -> Result<(), HaceError> {
        if digest.len() < algo.digest_len() {
            return Err(HaceError::BufferTooSmall);
        }
        if data.len() >= 256 * 1024 * 1024 {
            return Err(HaceError::DataTooLarge);
        }
        let cmd = algo.cmd_word();
        self.run_hash(
            data.as_ptr() as u32,
            data.len() as u32,
            digest.as_mut_ptr() as u32,
            0,
            cmd,
        )
        .await
    }

    /// HMAC-SHA-256.
    ///
    /// - `key_buf`: caller-supplied 64-byte buffer **aligned to 64 bytes**.
    ///   Will be overwritten.
    /// - `digest`: must be ≥ 32 bytes.
    pub async fn hmac_sha256(
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
        if key_buf.as_ptr() as usize % 64 != 0 {
            return Err(HaceError::KeyBufMisaligned);
        }

        key_buf.fill(0);
        let copy_len = key.len().min(64);
        key_buf[..copy_len].copy_from_slice(&key[..copy_len]);

        let key_addr = key_buf.as_ptr() as u32;
        let dig_addr = digest.as_mut_ptr() as u32;

        // Step 1: pre-process key.
        let key_cmd = hash_cmd(ALGO_SHA256, 0, HMAC_KEY);
        self.run_hash(key_addr, 64, dig_addr, key_addr, key_cmd)
            .await?;

        // Step 2: HMAC with message.
        let hmac_cmd = hash_cmd(ALGO_SHA256, 0, HMAC_HMAC);
        self.run_hash(
            data.as_ptr() as u32,
            data.len() as u32,
            dig_addr,
            key_addr,
            hmac_cmd,
        )
        .await
    }

    // ── Internal ─────────────────────────────────────────────────────────────

    async fn run_hash(
        &self,
        src: u32,
        len: u32,
        digest: u32,
        key: u32,
        cmd: u32,
    ) -> Result<(), HaceError> {
        // Wait for any previous operation to complete (blocking spin — typically instant).
        while pac::HACE.HACE_STATUS().read().HASH_BUSY() {
            core::hint::spin_loop();
        }

        let hace = pac::HACE;

        // Clear stale interrupt flags (hace_v1.yaml HACE_STATUS: HASH_INT=bit9, CRYPTO_INT=bit12).
        hace.HACE_STATUS().write(|w| {
            w.set_HASH_INT(true);
            w.set_CRYPTO_INT(true);
        });

        // Load DMA source, digest base, key base, data length.
        hace.HASH_SRC().write(|w| w.set_ADDR(src >> 1)); // addr field is bits[30:0], shift=0
        hace.HASH_DIGEST_BASE().write(|w| w.set_ADDR(digest >> 1));
        hace.HASH_KEY_BASE().write(|w| w.set_ADDR(key >> 1));
        hace.HASH_DATA_LEN().write(|w| w.set_LEN(len));

        // Writing HASH_CMD starts the DMA + hash engine.
        hace.HASH_CMD().write(|w| {
            w.0 = cmd;
        });

        HashWaitFuture.await;
        Ok(())
    }
}

impl Default for Hace {
    fn default() -> Self {
        Self::new()
    }
}

// ── HashWaitFuture ────────────────────────────────────────────────────────────

/// Future that resolves when `HACE_STATUS.HASH_BUSY` clears.
///
/// The `HACE` ISR (IRQ 4) clears the interrupt flag and calls `HACE_WAKER.wake()`.
struct HashWaitFuture;

impl Future for HashWaitFuture {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if !pac::HACE.HACE_STATUS().read().HASH_BUSY() {
            pac::HACE.HACE_STATUS().write(|w| w.set_HASH_INT(true)); // W1C
            return Poll::Ready(());
        }
        HACE_WAKER.register(cx.waker());
        // Re-check after registration (close check→register race).
        if !pac::HACE.HACE_STATUS().read().HASH_BUSY() {
            pac::HACE.HACE_STATUS().write(|w| w.set_HASH_INT(true));
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// ── HACE interrupt handler — IRQ 4 ───────────────────────────────────────────

#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn HACE() {
    // Clear hash-done and crypto-done interrupt flags (W1C per hace_v1.yaml).
    pac::HACE.HACE_STATUS().write(|w| {
        w.set_HASH_INT(true);
        w.set_CRYPTO_INT(true);
    });
    HACE_WAKER.wake();
}
