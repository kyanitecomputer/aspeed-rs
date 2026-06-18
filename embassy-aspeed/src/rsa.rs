//! RSA engine driver for AST1060 — async (timer-yield polling).
//!
//! Uses the Secure Boot Controller (SECURE) hardware RSA engine.
//! Base: `0x7E6F_2000`.  SRAM base: `0x7900_0000`.
//!
//! # Supported key sizes
//!
//! 1024, 2048, 3072, and 4096-bit moduli.  Exponent up to 4096 bits.
//!
//! # Operations
//!
//! - **`verify`**: compute `sig^e mod m` and compare tail bytes to `digest`
//!   (PKCS#1 v1.5 public-key operation).
//! - **`sign`**: compute `msg^d mod m` where `msg` is a PKCS#1 v1.5 padded
//!   digest.
//!
//! # Async model
//!
//! The RSA engine takes 10–100 ms.  This driver fires the trigger then yields
//! to the executor via `embassy_time::Timer` in a polling loop, checking
//! `SECURE.ENGINE_STATUS.RSA_READY`.  This avoids blocking the executor for
//! the duration of the operation.
//!
//! No dedicated NVIC IRQ is routed for the RSA/ECC engine on AST1060; the
//! ENGINE_IRQ registers exist but are unused here.
//!
//! # SRAM layout
//!
//! | Offset | Size | Content |
//! |--------|------|---------|
//! | +0x000 | 0x400 | Exponent (e or d), byte-reversed |
//! | +0x400 | 0x400 | Modulus (m), byte-reversed |
//! | +0x800 | 0x800 | Input data (padded msg or sig), byte-reversed |
//! | +0x1400 | 0x400 | **Result** (output), read MSB-first from +0x17FF |
//!
//! All SRAM regions are zeroed before and after each operation.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::rsa::{Rsa, DigestAlgo};
//!
//! let result = Rsa::verify(
//!     &signature, &digest, DigestAlgo::Sha256,
//!     &modulus, &exponent, 2048, 17,
//! ).await.unwrap();
//! assert!(result);
//! ```

use aspeed_mmio::{poll_until_async, MmioBlock};
use embassy_time::Duration;

use crate::pac;

// ── SRAM addresses ────────────────────────────────────────────────────────────

const RSA_SRAM_BASE: usize = 0x7900_0000;
const SRAM_EXPONENT: usize = 0x0000; // offset for e / d
const SRAM_MODULUS: usize = 0x0400; // offset for m
const SRAM_INPUT: usize = 0x0800; // offset for input (padded msg / sig)
const SRAM_RESULT: usize = 0x1400; // offset for output result
const SRAM_CLEAR_SIZE: usize = 0x1800; // total bytes zeroed per op

// ── PKCS#1 v1.5 DigestInfo prefixes ──────────────────────────────────────────

/// Digest algorithm for PKCS#1 v1.5 padding.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DigestAlgo {
    Sha256,
    Sha384,
    Sha512,
}

impl DigestAlgo {
    fn prefix(self) -> &'static [u8] {
        match self {
            DigestAlgo::Sha256 => &[
                0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02,
                0x01, 0x05, 0x00, 0x04, 0x20,
            ],
            DigestAlgo::Sha384 => &[
                0x30, 0x41, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02,
                0x02, 0x05, 0x00, 0x04, 0x30,
            ],
            DigestAlgo::Sha512 => &[
                0x30, 0x51, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02,
                0x03, 0x05, 0x00, 0x04, 0x40,
            ],
        }
    }

    fn hash_len(self) -> usize {
        match self {
            DigestAlgo::Sha256 => 32,
            DigestAlgo::Sha384 => 48,
            DigestAlgo::Sha512 => 64,
        }
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

/// RSA engine error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RsaError {
    /// Key or message component too large for the engine.
    DataTooLarge,
    /// Engine did not become ready within the polling limit.
    Timeout,
    /// Modulus byte count is invalid for the hardware.
    InvalidKeySize,
}

// ── Rsa ───────────────────────────────────────────────────────────────────────

/// AST1060 RSA engine driver.
///
/// All methods are `async` — they yield to the executor while the engine runs.
pub struct Rsa;

impl Rsa {
    /// RSA public-key verify: compute `sig^e mod m`, check against `digest`.
    ///
    /// - `sig`: signature bytes (big-endian, same length as modulus).
    /// - `digest`: expected message digest.
    /// - `algo`: digest algorithm (for PKCS#1 v1.5 prefix check).
    /// - `modulus`, `exponent`: public key components (big-endian).
    /// - `mod_bits`, `exp_bits`: bit widths of modulus and exponent.
    ///
    /// Returns `Ok(true)` if signature is valid, `Ok(false)` if the tail
    /// of the result does not match the digest.
    pub async fn verify(
        sig: &[u8],
        digest: &[u8],
        algo: DigestAlgo,
        modulus: &[u8],
        exponent: &[u8],
        mod_bits: u32,
        exp_bits: u32,
    ) -> Result<bool, RsaError> {
        let m_bytes = ((mod_bits + 7) / 8) as usize;
        let mut result = [0u8; 512];
        Self::run_engine(
            sig,
            &mut result[..m_bytes],
            modulus,
            exponent,
            mod_bits,
            exp_bits,
        )
        .await?;

        // Check PKCS#1 v1.5 tail: result should end with DER prefix + digest.
        let prefix = algo.prefix();
        let hash_len = algo.hash_len();
        let expected_len = prefix.len() + hash_len;
        if m_bytes < expected_len {
            return Ok(false);
        }
        let tail = &result[m_bytes - expected_len..m_bytes];
        let matches = &tail[..prefix.len()] == prefix
            && &tail[prefix.len()..] == &digest[..hash_len.min(digest.len())];
        Ok(matches)
    }

    /// RSA private-key sign: compute PKCS#1 v1.5 padded `digest^d mod m`.
    ///
    /// Output is written into `out` (must be ≥ `mod_bits / 8` bytes).
    pub async fn sign(
        digest: &[u8],
        algo: DigestAlgo,
        modulus: &[u8],
        private_key: &[u8],
        mod_bits: u32,
        key_bits: u32,
        out: &mut [u8],
    ) -> Result<(), RsaError> {
        let m_bytes = ((mod_bits + 7) / 8) as usize;
        if out.len() < m_bytes {
            return Err(RsaError::DataTooLarge);
        }

        // Build PKCS#1 v1.5 padded input.
        let prefix = algo.prefix();
        let hash_len = algo.hash_len();
        let mut padded = [0u8; 512];
        let padded_slice = &mut padded[..m_bytes];

        // EM = 0x00 || 0x01 || PS (0xFF bytes) || 0x00 || T
        let t_len = prefix.len() + hash_len;
        let ps_len = m_bytes.saturating_sub(3 + t_len);
        padded_slice[0] = 0x00;
        padded_slice[1] = 0x01;
        for i in 2..2 + ps_len {
            padded_slice[i] = 0xFF;
        }
        padded_slice[2 + ps_len] = 0x00;
        padded_slice[3 + ps_len..3 + ps_len + prefix.len()].copy_from_slice(prefix);
        padded_slice[3 + ps_len + prefix.len()..3 + ps_len + t_len]
            .copy_from_slice(&digest[..hash_len.min(digest.len())]);

        Self::run_engine(
            padded_slice,
            &mut out[..m_bytes],
            modulus,
            private_key,
            mod_bits,
            key_bits,
        )
        .await
    }

    // ── Internal engine execution ─────────────────────────────────────────────

    async fn run_engine(
        input: &[u8],
        output: &mut [u8],
        modulus: &[u8],
        exp_or_key: &[u8],
        mod_bits: u32,
        ed_bits: u32,
    ) -> Result<(), RsaError> {
        let m_bytes = ((mod_bits + 7) / 8) as usize;
        if m_bytes > 0x400 || exp_or_key.len() > 0x400 || input.len() > 0x800 {
            return Err(RsaError::DataTooLarge);
        }

        // Zero the shared SRAM region.
        sram_zero(0, SRAM_CLEAR_SIZE);

        // Write components byte-reversed (big-endian input → little-endian SRAM).
        sram_write_reversed(SRAM_EXPONENT, exp_or_key);
        sram_write_reversed(SRAM_MODULUS, modulus);
        sram_write_reversed(SRAM_INPUT, input);

        // Configure key lengths and trigger engine.
        let sec = pac::SECURE;
        sec.RSA_CTRL().write(|w| {
            w.set_RSA_MOD_BITS(mod_bits as u16);
            w.set_RSA_EXP_BITS(ed_bits as u16);
        });
        sec.ENGINE_TRIG().write(|w| w.set_RSA_TRIG(true));
        sec.ENGINE_TRIG().write(|w| w.set_RSA_TRIG(false));

        // Yield-poll until ENGINE_STATUS.RSA_READY = 1.
        let ready = poll_until_async(
            || sec.ENGINE_STATUS().read().RSA_READY(),
            |ready| *ready,
            Duration::from_micros(10),
            Duration::from_millis(100),
        )
        .await;
        if ready.is_err() {
            sram_zero(0, SRAM_CLEAR_SIZE);
            return Err(RsaError::Timeout);
        }

        // Read result MSB-first from SRAM+0x17FF downward.
        let result_end = RSA_SRAM_BASE + SRAM_RESULT + 0x400; // exclusive end
        let out_len = output.len().min(0x400);
        for (i, slot) in output[..out_len].iter_mut().enumerate() {
            let addr = result_end - 1 - i;
            *slot = unsafe { MmioBlock::new(addr) }.read8(0);
        }

        // Wipe SRAM.
        sram_zero(0, SRAM_CLEAR_SIZE);
        Ok(())
    }
}

// ── SRAM helpers ──────────────────────────────────────────────────────────────

/// Zero `len` bytes of RSA SRAM starting at `offset`.
fn sram_zero(offset: usize, len: usize) {
    let mut sram = unsafe { MmioBlock::new(RSA_SRAM_BASE + offset) };
    for i in 0..len {
        sram.write8(i, 0);
    }
}

/// Write `data` into RSA SRAM at `offset` in byte-reversed order
/// (converts big-endian input to little-endian SRAM layout).
fn sram_write_reversed(offset: usize, data: &[u8]) {
    let mut sram = unsafe { MmioBlock::new(RSA_SRAM_BASE + offset) };
    let n = data.len();
    for (i, &b) in data.iter().rev().enumerate() {
        sram.write8(i, b);
    }
    let _ = n;
}
