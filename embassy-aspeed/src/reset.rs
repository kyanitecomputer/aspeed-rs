//! Per-peripheral reset and clock-gate control — AST1060.
//!
//! Wraps the SCU `RESET_CTRL1/2` and `CLKSTOP1/2` register pairs.  All
//! operations are instantaneous register writes; no async is needed.
//!
//! # Reset registers (RW1S / RW1C pairs)
//!
//! | Register | SCU offset | Action |
//! |----------|-----------|--------|
//! | `RESET_CTRL1_SET` | 0x040 | Write 1 to **assert** reset |
//! | `RESET_CTRL1_CLR` | 0x044 | Write 1 to **deassert** reset |
//! | `RESET_CTRL2_SET` | 0x050 | Write 1 to **assert** reset |
//! | `RESET_CTRL2_CLR` | 0x054 | Write 1 to **deassert** reset |
//!
//! # Clock-stop registers (RW1S / RW1C pairs)
//!
//! | Register | SCU offset | Action |
//! |----------|-----------|--------|
//! | `CLKSTOP1_SET` | 0x080 | Write 1 to **stop** clock |
//! | `CLKSTOP1_CLR` | 0x084 | Write 1 to **start** clock |
//! | `CLKSTOP2_SET` | 0x090 | Write 1 to **stop** clock |
//! | `CLKSTOP2_CLR` | 0x094 | Write 1 to **start** clock |
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::reset::{reset_pulse, clock_enable, Peripheral};
//!
//! // Bring I2C/SMBus out of reset with its clock running.
//! clock_enable(Peripheral::I2cSmbus);
//! reset_pulse(Peripheral::I2cSmbus);
//!
//! // Put the HACE engine in reset, then bring it back.
//! reset_assert(Peripheral::Hace);
//! reset_deassert(Peripheral::Hace);
//! ```

use crate::pac;

// ── Peripheral enum ───────────────────────────────────────────────────────────

/// AST1060 peripheral identifiers used with reset and clock-gate functions.
///
/// Each variant maps to one or more bits in the SCU RESET_CTRL / CLKSTOP
/// registers.  Consult the register YAML comments for exact bit positions.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Peripheral {
    // ── RESET_CTRL1 / CLKSTOP1 ───────────────────────────────────────────────
    /// SRAM controller.
    Sram,
    /// Hash and Crypto Engine (HACE). Clock: CLKSTOP1 bit 13.
    Hace,

    // ── RESET_CTRL2 ──────────────────────────────────────────────────────────
    /// I2C/SMBus controller (all 14 channels share one reset bit).
    I2cSmbus,
    /// I3C DMA controller.
    I3cDma,
    /// I3C channel 0.
    I3c0,
    /// I3C channel 1.
    I3c1,
    /// I3C channel 2.
    I3c2,
    /// I3C channel 3.
    I3c3,
    /// ADC controller.
    Adc,
    /// JTAG master 1.
    Jtag1,
    /// UART1.
    Uart1,
    /// UART2.
    Uart2,
    /// UART3.
    Uart3,
    /// UART4.
    Uart4,

    // ── CLKSTOP2 only ────────────────────────────────────────────────────────
    /// RSA/ECC engine clock (CLKSTOP2 bit 6).
    RsaEcc,
}

// ── Internal bit descriptors ──────────────────────────────────────────────────

/// Which SCU reset register bank controls this peripheral, and which bit.
/// `None` means no reset bit (clock-only or combined).
struct ResetBit {
    bank: u8, // 1 = RESET_CTRL1, 2 = RESET_CTRL2
    bit: u32,
}

/// Which SCU clock-stop register bank controls this peripheral, and which bit.
/// `None` means no dedicated clock-stop bit (e.g. always-on).
struct ClockBit {
    bank: u8, // 1 = CLKSTOP1, 2 = CLKSTOP2
    bit: u32,
}

fn reset_info(p: Peripheral) -> Option<ResetBit> {
    match p {
        Peripheral::Sram => Some(ResetBit { bank: 1, bit: 0 }),
        Peripheral::I2cSmbus => Some(ResetBit { bank: 2, bit: 2 }),
        Peripheral::I3cDma => Some(ResetBit { bank: 2, bit: 7 }),
        Peripheral::I3c0 => Some(ResetBit { bank: 2, bit: 8 }),
        Peripheral::I3c1 => Some(ResetBit { bank: 2, bit: 9 }),
        Peripheral::I3c2 => Some(ResetBit { bank: 2, bit: 10 }),
        Peripheral::I3c3 => Some(ResetBit { bank: 2, bit: 11 }),
        Peripheral::Adc => Some(ResetBit { bank: 2, bit: 23 }),
        Peripheral::Jtag1 => Some(ResetBit { bank: 2, bit: 26 }),
        Peripheral::Uart1 => Some(ResetBit { bank: 2, bit: 28 }),
        Peripheral::Uart2 => Some(ResetBit { bank: 2, bit: 29 }),
        Peripheral::Uart3 => Some(ResetBit { bank: 2, bit: 30 }),
        Peripheral::Uart4 => Some(ResetBit { bank: 2, bit: 31 }),
        // HACE has no dedicated reset bit in the PAC fieldsets (always running).
        Peripheral::Hace | Peripheral::RsaEcc => None,
    }
}

fn clock_info(p: Peripheral) -> Option<ClockBit> {
    match p {
        Peripheral::Sram => Some(ClockBit { bank: 1, bit: 0 }),
        Peripheral::Hace => Some(ClockBit { bank: 1, bit: 13 }),
        Peripheral::RsaEcc => Some(ClockBit { bank: 2, bit: 6 }),
        Peripheral::I3c0 => Some(ClockBit { bank: 2, bit: 8 }),
        Peripheral::I3c1 => Some(ClockBit { bank: 2, bit: 9 }),
        Peripheral::I3c2 => Some(ClockBit { bank: 2, bit: 10 }),
        Peripheral::I3c3 => Some(ClockBit { bank: 2, bit: 11 }),
        // I2C, ADC, JTAG, UARTs share the PCLK — no individual clock-stop bits.
        _ => None,
    }
}

// ── Public API ─────────────────────────────────────────────────────────────────

/// Assert reset for `periph`.
///
/// Has no effect if the peripheral has no reset bit in the SCU.
pub fn reset_assert(periph: Peripheral) {
    if let Some(rb) = reset_info(periph) {
        let mask = 1u32 << rb.bit;
        let scu = pac::SCU;
        match rb.bank {
            1 => scu.RESET_CTRL1_SET().write(|w| {
                // Write a raw 32-bit value — the PAC only exposes bit 0 in the
                // fieldset, so use DATA field directly.
                w.0 = mask;
            }),
            2 => scu.RESET_CTRL2_SET().write(|w| {
                w.0 = mask;
            }),
            _ => {}
        }
    }
}

/// Deassert reset for `periph`.
pub fn reset_deassert(periph: Peripheral) {
    if let Some(rb) = reset_info(periph) {
        let mask = 1u32 << rb.bit;
        let scu = pac::SCU;
        match rb.bank {
            1 => scu.RESET_CTRL1_CLR().write(|w| {
                w.0 = mask;
            }),
            2 => scu.RESET_CTRL2_CLR().write(|w| {
                w.0 = mask;
            }),
            _ => {}
        }
    }
}

/// Assert reset, then immediately deassert (pulse reset).
///
/// Minimal hardware pulse — callers that need a delay should call
/// `reset_assert` + delay + `reset_deassert` manually.
pub fn reset_pulse(periph: Peripheral) {
    reset_assert(periph);
    // A single instruction delay is enough for the hardware to latch the reset.
    core::hint::spin_loop();
    reset_deassert(periph);
}

/// Enable the clock for `periph` (un-stop it in CLKSTOP register).
///
/// Has no effect if the peripheral has no dedicated clock-stop bit.
pub fn clock_enable(periph: Peripheral) {
    if let Some(cb) = clock_info(periph) {
        let mask = 1u32 << cb.bit;
        let scu = pac::SCU;
        match cb.bank {
            1 => scu.CLKSTOP1_CLR().write(|w| {
                w.0 = mask;
            }),
            2 => scu.CLKSTOP2_CLR().write(|w| {
                w.0 = mask;
            }),
            _ => {}
        }
    }
}

/// Stop the clock for `periph`.
pub fn clock_disable(periph: Peripheral) {
    if let Some(cb) = clock_info(periph) {
        let mask = 1u32 << cb.bit;
        let scu = pac::SCU;
        match cb.bank {
            1 => scu.CLKSTOP1_SET().write(|w| {
                w.0 = mask;
            }),
            2 => scu.CLKSTOP2_SET().write(|w| {
                w.0 = mask;
            }),
            _ => {}
        }
    }
}

/// Enable clock + deassert reset for `periph` (typical init sequence).
pub fn periph_enable(periph: Peripheral) {
    clock_enable(periph);
    reset_deassert(periph);
}
