//! Hardware True Random Number Generator (TRNG) driver — AST1060.
//!
//! Base address: `0x7E6E_2530`.  No interrupt — polled only.
//! Register size: 16 bytes (CTRL at +0x00, DATA at +0x04).
//!
//! # Enable sequence
//!
//! 1. Read `CTRL`.
//! 2. Clear `RNG_DISABLE` (bit 0) and write `RNG_MODE = 0x18` (bits[5:1]).
//! 3. Write `CTRL` back.
//! 4. Poll `CTRL.RNG_READY` (bit 31); when set, read `DATA`.
//!    Reading `DATA` clears `RNG_READY`.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::trng::Trng;
//!
//! let mut rng = Trng::new();
//! rng.enable();
//!
//! let word = rng.read_word().await.unwrap();
//! let mut buf = [0u8; 32];
//! rng.fill_bytes(&mut buf).await.unwrap();
//! ```

use embassy_time::Timer;

use crate::pac;

// ── Constants ─────────────────────────────────────────────────────────────────

/// LFSR mode. Bits[5:1] of CTRL.
const RNG_MODE: u8 = 0x18;

/// Microseconds between ready-bit polls.
const POLL_US: u64 = 100;

/// Maximum poll attempts per 32-bit word before returning an error.
const MAX_POLLS: u32 = 10;

// ── Error type ────────────────────────────────────────────────────────────────

/// TRNG error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TrngError {
    /// RNG_READY did not set within the poll timeout.
    NotReady,
}

// ── Trng ─────────────────────────────────────────────────────────────────────

/// AST1060 Hardware TRNG driver.
pub struct Trng {
    regs: pac::trng_v1::TRNG,
}

impl Trng {
    /// Obtain a TRNG handle.
    ///
    /// Call [`enable`] before reading entropy.
    pub fn new() -> Self {
        Self { regs: pac::TRNG }
    }

    /// Enable the TRNG with the default LFSR mode (0x18).
    ///
    /// Idempotent — safe to call multiple times.
    pub fn enable(&self) {
        self.regs.CTRL().modify(|w| {
            w.set_RNG_DISABLE(false);
            w.set_RNG_MODE(RNG_MODE);
        });
    }

    /// Disable the TRNG (power savings).
    pub fn disable(&self) {
        self.regs.CTRL().modify(|w| w.set_RNG_DISABLE(true));
    }

    /// Read one 32-bit random word.
    ///
    /// Yields to the executor every `POLL_US` µs while waiting for
    /// `RNG_READY`.  Returns [`TrngError::NotReady`] if `MAX_POLLS`
    /// attempts are exhausted without a fresh word.
    pub async fn read_word(&self) -> Result<u32, TrngError> {
        for _ in 0..MAX_POLLS {
            if self.regs.CTRL().read().RNG_READY() {
                return Ok(self.regs.DATA().read().DATA());
            }
            Timer::after_micros(POLL_US).await;
        }
        Err(TrngError::NotReady)
    }

    /// Fill `buf` with random bytes.
    ///
    /// Reads 32-bit words until `buf` is full.  Partial final words are
    /// handled correctly.
    ///
    /// # Errors
    ///
    /// Returns `TrngError::NotReady` if the hardware fails to produce a word
    /// within `MAX_POLLS × POLL_US` µs for any single word.
    pub async fn fill_bytes(&self, buf: &mut [u8]) -> Result<(), TrngError> {
        let mut offset = 0;
        while offset < buf.len() {
            let word = self.read_word().await?;
            let bytes = word.to_le_bytes();
            let remaining = buf.len() - offset;
            let chunk = remaining.min(4);
            buf[offset..offset + chunk].copy_from_slice(&bytes[..chunk]);
            offset += chunk;
        }
        Ok(())
    }

    /// Blocking read of one 32-bit word (for use before Embassy executor starts).
    ///
    /// Spins until `RNG_READY` is set; returns `TrngError::NotReady` if the
    /// counter exhausts.
    pub fn read_word_blocking(&self) -> Result<u32, TrngError> {
        for _ in 0..MAX_POLLS {
            if self.regs.CTRL().read().RNG_READY() {
                return Ok(self.regs.DATA().read().DATA());
            }
            for _ in 0..10_000u32 {
                core::hint::spin_loop();
            }
        }
        Err(TrngError::NotReady)
    }
}

impl Default for Trng {
    fn default() -> Self {
        Self::new()
    }
}
