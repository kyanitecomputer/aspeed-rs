//! ADC driver for AST1060 (1 bank, 8 channels) and AST2600 SSP (2 banks × 8 channels).
#![allow(dead_code)]
//!
//! # Hardware
//!
//! | Chip | Banks | Channels | Base addresses |
//! |------|-------|----------|----------------|
//! | AST1060 | 1 | 0–7 | 0x7E6E_9000 |
//! | AST2600 SSP | 2 | bank 0: 0–7, bank 1: 0–7 | 0x7E6E_9000, 0x7E6E_9100 |
//!
//! 10-bit resolution.  Reference voltage selectable: internal 2.5 V (default),
//! 1.2 V, or external.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::adc::{Adc, AdcRef};
//!
//! // AST1060: single bank, channels 0–7
//! let mut adc = Adc::new(0, AdcRef::Internal2500mV).unwrap();
//! let raw = adc.read_channel(3); // 10-bit value, 0–1023
//!
//! // AST2600: bank 0 (CH0–7) or bank 1 (CH8–15 physical)
//! let mut adc0 = Adc::new(0, AdcRef::Internal2500mV).unwrap();
//! let mut adc1 = Adc::new(1, AdcRef::Internal2500mV).unwrap();
//! ```
//!
//! # Initialisation
//!
//! `Adc::new` powers up the engine, enables all requested channels, and waits
//! for `INIT_RDY` (hardware calibration complete).  The init timeout is
//! 500 ms; [`AdcError::Timeout`] is returned if the engine does not become
//! ready in time.  All registers are accessed using `MmioBlock`; no DMA.
//!
//! # Clock
//!
//! The ADC sample clock divisor is not configured here; the hardware reset
//! default gives a sample rate of approximately 65 kHz from PCLK.  Use
//! [`Adc::set_clock_div`] to adjust for a specific PCLK frequency if needed.

use aspeed_mmio::{poll_until, MmioBlock};

// ── Base addresses ────────────────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
const ADC_BANK0_BASE: usize = 0x7E6E_9000;
#[cfg(feature = "ast1060")]
const N_ADC_BANKS: usize = 1;

#[cfg(feature = "ast2600-ssp")]
const ADC_BANK0_BASE: usize = 0x7E6E_9000;
#[cfg(feature = "ast2600-ssp")]
const N_ADC_BANKS: usize = 2;

const ADC_BANK_STRIDE: usize = 0x100;
const N_CH_PER_BANK: usize = 8;

// ── Register offsets (byte) ───────────────────────────────────────────────────

const ENGINE_CTRL: usize = 0x00;
const INT_CTRL: usize = 0x04;
// 0x08: VGA_DETECT_CTRL (not used here)
const CLK_CTRL: usize = 0x0C;
// Channel data: 0x10 + ch * 2 (10-bit result in bits [9:0])
const CH_DATA_BASE: usize = 0x10;
const COMP_TRIM: usize = 0xC4;

// ── ENGINE_CTRL bit fields ────────────────────────────────────────────────────

const ENGINE_EN: u32 = 1 << 0;
const OP_MODE_NORMAL: u32 = 7 << 1; // continuous scan
const OP_MODE_MASK: u32 = 7 << 1;
const CTRL_COMPENSATION: u32 = 1 << 4;
const AUTO_COMPENSATION: u32 = 1 << 5;
const REF_VOLTAGE_SHIFT: u32 = 6;
const REF_VOLTAGE_MASK: u32 = 3 << 6;
const CTRL_INIT_RDY: u32 = 1 << 8;
const CH7_BAT_MODE: u32 = 1 << 12;
const BAT_SENSING_EN: u32 = 1 << 13;
const CH_EN_SHIFT: u32 = 16; // bits [31:16], bit 16 = CH0

// ── Reference voltage encoding ────────────────────────────────────────────────

/// ADC reference voltage selection.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AdcRef {
    /// Internal 2.5 V reference (default, always available).
    Internal2500mV,
    /// Internal 1.2 V reference (lower noise at low signal levels).
    Internal1200mV,
    /// External reference via ADCVREFP pin (high range).
    ExternalHigh,
    /// External reference via ADCVREFEXT pin (low range).
    ExternalLow,
}

impl AdcRef {
    fn encoding(self) -> u32 {
        match self {
            AdcRef::Internal2500mV => 0,
            AdcRef::Internal1200mV => 1,
            AdcRef::ExternalHigh => 2,
            AdcRef::ExternalLow => 3,
        }
    }
}

/// Error type for ADC operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AdcError {
    /// Bank index out of range.
    InvalidBank,
    /// Channel index out of range (must be 0–7 per bank).
    InvalidChannel,
    /// Engine did not become ready within the timeout window.
    Timeout,
}

// ── Adc ──────────────────────────────────────────────────────────────────────

/// ASPEED ADC bank driver.
///
/// One instance per bank.  AST1060 has one bank; AST2600 SSP has two.
pub struct Adc {
    base: usize,
}

impl Adc {
    /// Power up ADC bank `bank` with the given reference voltage.
    ///
    /// Enables all 8 channels in continuous-scan mode.
    /// Blocks until the engine reports `INIT_RDY` or returns [`AdcError::Timeout`].
    ///
    /// # Errors
    ///
    /// - [`AdcError::InvalidBank`] if `bank >= N_ADC_BANKS`.
    /// - [`AdcError::Timeout`] if the ADC engine takes too long to initialise.
    pub fn new(bank: u8, vref: AdcRef) -> Result<Self, AdcError> {
        if bank as usize >= N_ADC_BANKS {
            return Err(AdcError::InvalidBank);
        }

        let base = ADC_BANK0_BASE + bank as usize * ADC_BANK_STRIDE;
        let adc = Self { base };

        // Program reference voltage, enable engine, start normal scan.
        // Enable all 8 channels (bits [23:16] of ENGINE_CTRL).
        let ctrl = ENGINE_EN
            | OP_MODE_NORMAL
            | AUTO_COMPENSATION
            | (vref.encoding() << REF_VOLTAGE_SHIFT)
            | (0xFF << CH_EN_SHIFT);
        let mut regs = adc.regs();
        regs.write32(ENGINE_CTRL, ctrl);

        // Wait for hardware calibration to complete (INIT_RDY = bit 8).
        // Poll up to ~500 ms.  Cortex-M at 25 MHz: ~500 cycles per µs → 250 M cycles.
        const MAX_POLLS: u32 = 250_000_000;
        poll_until(
            || adc.regs().read32(ENGINE_CTRL),
            |v| v & CTRL_INIT_RDY != 0,
            MAX_POLLS,
        )
        .map_err(|_| AdcError::Timeout)?;

        Ok(adc)
    }

    /// Read the raw 10-bit result for channel `ch` (0–7).
    ///
    /// Returns the latest value captured by the hardware during continuous
    /// scanning.  The engine must be powered up (i.e., `Adc::new` must have
    /// succeeded).
    ///
    /// # Errors
    ///
    /// - [`AdcError::InvalidChannel`] if `ch >= 8`.
    pub fn read_channel(&self, ch: u8) -> Result<u16, AdcError> {
        if ch as usize >= N_CH_PER_BANK {
            return Err(AdcError::InvalidChannel);
        }

        let off = CH_DATA_BASE + ch as usize * 2;
        // Channel registers are 16-bit; result is in bits [9:0].
        let raw = self.regs().read16(off);
        Ok(raw & 0x3FF)
    }

    /// Set the ADC sample clock divisor.
    ///
    /// `sample_rate_hz = pclk_hz / (2 * (div + 1))`
    ///
    /// Helper: `div = pclk_hz / (2 * target_rate_hz) - 1`.
    /// The hardware maximum is 16 bits (65535 → very slow scan).
    ///
    /// Call after `Adc::new` if the default ~65 kHz rate is not suitable.
    pub fn set_clock_div(&self, div: u16) {
        self.regs().write32(CLK_CTRL, div as u32);
    }

    /// Enable or disable individual channels via a bitmask (bit N = channel N).
    ///
    /// Channel 0 → bit 0, channel 7 → bit 7.
    /// Channels not enabled are skipped in the continuous scan.
    ///
    /// This overwrites the channel-enable field.  Pass `0xFF` to re-enable all.
    pub fn set_channel_mask(&self, mask: u8) {
        let ctrl = self.regs().read32(ENGINE_CTRL);
        let new = (ctrl & !(0xFF << CH_EN_SHIFT)) | ((mask as u32) << CH_EN_SHIFT);
        self.regs().write32(ENGINE_CTRL, new);
    }

    /// Power down the ADC bank.
    pub fn shutdown(&self) {
        // OP_MODE = 0 (power-down) + ENGINE_EN = 0.
        self.regs().write32(ENGINE_CTRL, 0);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    fn regs(&self) -> MmioBlock {
        unsafe { MmioBlock::new(self.base) }
    }
}
