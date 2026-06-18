//! ECDSA-384 verify driver for AST1060 — async (timer-yield polling).
//!
//! Uses the Secure Boot Controller (SECURE) ECC engine.
//! Base: `0x7E6F_2000`.  SRAM base: `0x7900_0000`.
//!
//! Supports secp384r1 (P-384) signature verification only.
//! Curve parameters (Gx, Gy, P, N) are read from OTP registers in the SECURE
//! peripheral at offsets `0xA00`–`0xAEC`.
//!
//! # Async model
//!
//! The ECDSA engine takes ~5–10 ms.  This driver fires the trigger then yields
//! via `embassy_time::Timer` while polling `SECURE014` raw bits 20 (done) and
//! 21 (pass).  No dedicated NVIC IRQ is routed.
//!
//! # SRAM layout (from ECDSA_SRAM_BASE = `0x7900_0000`)
//!
//! | SRAM offset | Size | Content |
//! |-------------|------|---------|
//! | `+0x2000` | 48 B | Generator point Gx (from OTP) |
//! | `+0x2040` | 48 B | Gy (from OTP) |
//! | `+0x2080` | 48 B | Public key Qx |
//! | `+0x20C0` | 48 B | Public key Qy |
//! | `+0x2100` | 48 B | Curve prime P (from OTP) |
//! | `+0x2140` | 48 B | Curve coefficient A (all zeros for secp384r1) |
//! | `+0x2180` | 48 B | Curve order N (from OTP) |
//! | `+0x21C0` | 48 B | Signature R |
//! | `+0x2200` | 48 B | Signature S |
//! | `+0x2240` | 48 B | Message digest M (SHA-384) |
//! | `+0x23C0` | 4 B  | Command word (write 1 to verify) |
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::ecdsa::Ecdsa;
//!
//! let pass = Ecdsa::verify(&qx, &qy, &sig_r, &sig_s, &digest).await.unwrap();
//! assert!(pass);
//! ```

use aspeed_mmio::{poll_until_async, MmioBlock};
use embassy_time::{Duration, Timer};

use crate::pac;

// ── SECURE peripheral register offsets (raw byte offsets from base) ───────────

// Magic word used for ECC engine sequencing (vendor driver pattern).
const SECURE_MAGIC_OFF: usize = 0x7c;

// OTP curve-parameter offsets (read from SEC hardware, not programmed by SW).
const OTP_GX_OFF: usize = 0x0A00;
const OTP_GY_OFF: usize = 0x0A40;
const OTP_P_OFF: usize = 0x0A80;
const OTP_N_OFF: usize = 0x0AC0;

// SRAM destinations (offsets from `ECDSA_SRAM_BASE`).
const SRAM_GX: usize = 0x2000;
const SRAM_GY: usize = 0x2040;
const SRAM_QX: usize = 0x2080;
const SRAM_QY: usize = 0x20C0;
const SRAM_P: usize = 0x2100;
const SRAM_A: usize = 0x2140; // zeros (secp384r1: A = –3 mod P, handled internally)
const SRAM_N: usize = 0x2180;
const SRAM_R: usize = 0x21C0;
const SRAM_S: usize = 0x2200;
const SRAM_M: usize = 0x2240;
const SRAM_CMD: usize = 0x23C0; // write 1 here to issue verify instruction

const ECDSA_SRAM_BASE: usize = 0x7900_0000;
const SCALAR_BYTES: usize = 48; // P-384 field element size

// ENGINE_STATUS raw bit positions (bits 20 and 21 are undocumented in SVD).
const STATUS_ECDSA_DONE_BIT: u32 = 1 << 20;
const STATUS_ECDSA_PASS_BIT: u32 = 1 << 21;

// ── Error type ────────────────────────────────────────────────────────────────

/// ECDSA engine error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EcdsaError {
    /// Engine did not complete within the polling limit.
    Timeout,
    /// Input scalar length incorrect (must be exactly 48 bytes for secp384r1).
    InvalidLength,
}

// ── Ecdsa ─────────────────────────────────────────────────────────────────────

/// AST1060 ECDSA-384 verify engine.
///
/// All methods are `async`.
pub struct Ecdsa;

impl Ecdsa {
    /// Verify an ECDSA-384 signature.
    ///
    /// - `qx`, `qy`: public key X and Y components (48 bytes each, big-endian).
    /// - `sig_r`, `sig_s`: signature components (48 bytes each, big-endian).
    /// - `digest`: SHA-384 message digest (48 bytes).
    ///
    /// Returns `Ok(true)` if signature is valid, `Ok(false)` if invalid.
    pub async fn verify(
        qx: &[u8; 48],
        qy: &[u8; 48],
        sig_r: &[u8; 48],
        sig_s: &[u8; 48],
        digest: &[u8; 48],
    ) -> Result<bool, EcdsaError> {
        let sec_base = pac::SECURE.as_ptr() as usize;

        // Step 1: reset ECC engine.
        pac::SECURE.ECC_CTRL().write(|w| {
            w.set_ECC_EN(false);
            w.set_ECDSA384_EN(false);
        });

        // Step 2: enable ECC engine, wait 5 µs.
        pac::SECURE.ECC_CTRL().write(|w| {
            w.set_ECC_EN(true);
            w.set_ECDSA384_EN(false);
        });
        Timer::after_micros(5).await;

        // Step 3: copy OTP curve parameters → SRAM.
        sec_to_sram(
            sec_base + OTP_GX_OFF,
            ECDSA_SRAM_BASE + SRAM_GX,
            SCALAR_BYTES,
        );
        sec_to_sram(
            sec_base + OTP_GY_OFF,
            ECDSA_SRAM_BASE + SRAM_GY,
            SCALAR_BYTES,
        );
        sec_to_sram(sec_base + OTP_P_OFF, ECDSA_SRAM_BASE + SRAM_P, SCALAR_BYTES);
        sec_to_sram(sec_base + OTP_N_OFF, ECDSA_SRAM_BASE + SRAM_N, SCALAR_BYTES);
        // A coefficient = 0 (secp384r1 handled internally by hardware).
        sram_zero(ECDSA_SRAM_BASE + SRAM_A, SCALAR_BYTES);

        // Step 4: write magic word (vendor sequencing requirement).
        unsafe { MmioBlock::new(sec_base) }.write32(SECURE_MAGIC_OFF, 0x0300_f00b);

        // Step 5: load public key and signature into SRAM.
        sram_write_scalar(ECDSA_SRAM_BASE + SRAM_QX, qx);
        sram_write_scalar(ECDSA_SRAM_BASE + SRAM_QY, qy);
        sram_write_scalar(ECDSA_SRAM_BASE + SRAM_R, sig_r);
        sram_write_scalar(ECDSA_SRAM_BASE + SRAM_S, sig_s);
        sram_write_scalar(ECDSA_SRAM_BASE + SRAM_M, digest);

        // Step 6: clear magic word.
        unsafe { MmioBlock::new(sec_base) }.write32(SECURE_MAGIC_OFF, 0);

        // Step 7: write verify command.
        unsafe { MmioBlock::new(ECDSA_SRAM_BASE) }.write32(SRAM_CMD, 1);

        // Step 8: trigger ECC engine (set bit 1, wait 5 µs, clear).
        pac::SECURE.ENGINE_TRIG().write(|w| w.set_ECC_TRIG(true));
        Timer::after_micros(5).await;
        pac::SECURE.ENGINE_TRIG().write(|w| w.set_ECC_TRIG(false));

        // Step 9: yield-poll until ECDSA done (bit 20) or timeout.
        let raw = poll_until_async(
            || unsafe { MmioBlock::new(pac::SECURE.as_ptr() as usize) }.read32(0x014),
            |raw| raw & STATUS_ECDSA_DONE_BIT != 0,
            Duration::from_micros(5),
            Duration::from_millis(50),
        )
        .await
        .map_err(|_| EcdsaError::Timeout)?;

        let pass = raw & STATUS_ECDSA_PASS_BIT != 0;

        Ok(pass)
    }
}

// ── SRAM / SEC helpers ────────────────────────────────────────────────────────

/// Copy `len` bytes from a SECURE peripheral OTP region → SRAM (word-by-word).
fn sec_to_sram(sec_src: usize, sram_dst: usize, len: usize) {
    let words = len / 4;
    let sec = unsafe { MmioBlock::new(sec_src) };
    let mut sram = unsafe { MmioBlock::new(sram_dst) };
    for i in 0..words {
        let off = i * 4;
        let val = sec.read32(off);
        sram.write32(off, val);
    }
}

/// Zero `len` bytes in SRAM.
fn sram_zero(addr: usize, len: usize) {
    let words = len / 4;
    let mut sram = unsafe { MmioBlock::new(addr) };
    for i in 0..words {
        sram.write32(i * 4, 0);
    }
}

/// Write a 48-byte big-endian scalar into SRAM as a sequence of LE u32 words.
///
/// Each 4-byte chunk is read big-endian from `scalar` and written to SRAM
/// as a little-endian 32-bit word (matching the hardware's expectation).
fn sram_write_scalar(sram_addr: usize, scalar: &[u8; SCALAR_BYTES]) {
    let words = SCALAR_BYTES / 4;
    let mut sram = unsafe { MmioBlock::new(sram_addr) };
    for i in 0..words {
        let b = &scalar[i * 4..(i + 1) * 4];
        let word = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
        sram.write32(i * 4, word);
    }
}
