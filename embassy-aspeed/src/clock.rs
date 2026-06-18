//! Clock control driver: SCU clock gating and rate queries.
//!
//! Supports:
//! - **AST2600 SSP** (`ast2600-ssp` feature) — Cortex-M3 co-processor
//! - **AST1060** (`ast1060` feature) — Cortex-M4F standalone SoC
//!
//! Both chips share the same SCU base address (`0x7E6E_2000`) and the same
//! protection key, but have very different clock gate maps and PLL layouts.
//!
//! ## Clock gate protocol (both chips)
//!
//! Write 1 to the **SET** register (`SCU080`/`SCU090`) to **stop** a clock.
//! Write 1 to the **CLR** register (`SCU084`/`SCU094`) to **start** a clock.
//!
//! ## AST2600 SSP
//!
//! Two gate banks:
//! - Group 0 (`SCU080/084`): 28 clocks (bits 0–27)
//! - Group 1 (`SCU090/094`): 31 clocks (bits 0–30, enum value = 32 + bit)
//!
//! Fixed rates (CA7 sets PLLs before releasing the SSP):
//! - HCLK = 200 MHz, UART1–13 = 24 MHz / 13 ≈ 1,846,153 Hz
//!
//! ## AST1060
//!
//! Two gate banks with very few user-controllable bits:
//! - Group 0 (`SCU080/084`): only bits 13 (HACE) and 0 (SRAM) are functional
//! - Group 1 (`SCU090/094`): bits 11:8 (I3C0–3), 6 (RSA/ECC), 2 (REFCLK)
//!
//! Computed rates (read from SCU registers):
//! - HPLL: `25 MHz × (M+1) / (N+1) / (P+1)` — default 1000 MHz
//! - PCLK: `HPLL / (2 × (SCU310[11:8] + 1))` — default 500 MHz
//! - UART5: `24 MHz / 13` or `192 MHz / 13` depending on `SCU310[4]`
//!
//! # Example
//!
//! ```rust,ignore
//! use embassy_aspeed::clock::{ClockGate, clock_enable};
//! clock_enable(ClockGate::UART11CLK);  // AST2600
//! clock_enable(ClockGate::HaceYclk);  // AST1060
//! ```

#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
use aspeed_mmio::MmioBlock;

// ── Shared register addresses (SCU base 0x7E6E_2000 on both chips) ────────────

#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
const SCU_BASE: usize = 0x7E6E_2000;

#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
const CLK_STOP0_SET: usize = 0x080;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
const CLK_STOP0_CLR: usize = 0x084;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
const CLK_STOP1_SET: usize = 0x090;
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
const CLK_STOP1_CLR: usize = 0x094;

#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
#[inline(always)]
fn scu() -> MmioBlock {
    unsafe { MmioBlock::new(SCU_BASE) }
}

// ── AST2600 SSP ───────────────────────────────────────────────────────────────

#[cfg(feature = "ast2600-ssp")]
mod ast2600_clk {
    pub const APLL_PARAM: usize = 0x210;
    pub const CLK_SEL0: usize = 0x300;
    pub const CLK_SEL4: usize = 0x310;

    /// Oscillator input frequency (25 MHz).
    pub const CLKIN_HZ: u32 = 25_000_000;
    /// Fixed HCLK (AHB bus clock), set by CA7 before SSP start.
    pub const HCLK_HZ: u32 = 200_000_000;
    /// Fixed HPLL output used for APB1.
    pub const HPLL_HZ: u32 = 1_200_000_000;
    /// UART1–13 clock source (24 MHz / 13).
    pub const UART_CLK_HZ: u32 = 24_000_000 / 13;
}

/// Identifies a gateable peripheral clock (AST2600 SSP).
///
/// Values 0–31 → Group 0 (SCU080 bit = value).
/// Values 32–63 → Group 1 (SCU090 bit = value − 32).
/// Values ≥ 64 → always-on; `clock_enable`/`clock_disable` are no-ops.
#[cfg(feature = "ast2600-ssp")]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ClockGate {
    // Group 0 (SCU080 bits 0–27)
    MCLK = 0,
    ECLK = 1,
    GCLK = 2,
    VCLK = 3,
    BCLK = 4,
    DCLK = 5,
    REF0CLK = 6,
    USBPORT2CLK = 7,
    // 8 reserved
    USBUHCICLK = 9,
    D1CLK = 10,
    // 11–12 reserved
    YCLK = 13,
    USBPORT1CLK = 14,
    UART5CLK = 15,
    // 16–19 reserved
    MAC1CLK = 20,
    MAC2CLK = 21,
    // 22–23 reserved
    RSACLK = 24,
    RVASCLK = 25,
    // 26 reserved
    EMMCCLK = 27,

    // Group 1 (SCU090 bits 0–30; enum value = 32 + bit)
    LCLK = 32,
    ESPICLK = 33,
    REF1CLK = 34,
    // 35 reserved
    SDCLK = 36,
    LHCCLK = 37,
    // 38–39 reserved
    I3C0CLK = 40,
    I3C1CLK = 41,
    I3C2CLK = 42,
    I3C3CLK = 43,
    I3C4CLK = 44,
    I3C5CLK = 45,
    // 46–47 reserved
    UART1CLK = 48,
    UART2CLK = 49,
    UART3CLK = 50,
    UART4CLK = 51,
    MAC3CLK = 52,
    MAC4CLK = 53,
    UART6CLK = 54,
    UART7CLK = 55,
    UART8CLK = 56,
    UART9CLK = 57,
    UART10CLK = 58,
    /// UART11 clock — SSP console UART.
    UART11CLK = 59,
    UART12CLK = 60,
    UART13CLK = 61,
    FSICLK = 62,
}

/// Identifies a clock whose rate can be queried (AST2600 SSP).
#[cfg(feature = "ast2600-ssp")]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ClockId {
    /// AHB bus clock (fixed at 200 MHz).
    HCLK,
    /// APB1 clock derived from HPLL with a programmable divider.
    APB1,
    /// APB2 clock derived from HCLK with a programmable divider.
    APB2,
    /// UART1–13 clock source (24 MHz / 13 = 1,846,153 Hz).
    UART,
}

/// Return the frequency in Hz for a given clock (AST2600 SSP).
#[cfg(feature = "ast2600-ssp")]
pub fn get_rate(id: ClockId) -> u32 {
    use ast2600_clk::*;
    match id {
        ClockId::HCLK => HCLK_HZ,
        ClockId::UART => UART_CLK_HZ,
        ClockId::APB1 => {
            // APB1 = HPLL / ((APB1_DIV + 1) * 4); APB1_DIV = CLK_SEL0[25:23].
            let reg = scu().read32(CLK_SEL0);
            let div = ((reg >> 23) & 0x7) as u32;
            HPLL_HZ / ((div + 1) * 4)
        }
        ClockId::APB2 => {
            // APB2 = HCLK / ((APB2_DIV + 1) * 2); APB2_DIV = CLK_SEL4[11:9].
            let reg = scu().read32(CLK_SEL4);
            let div = ((reg >> 9) & 0x7) as u32;
            HCLK_HZ / ((div + 1) * 2)
        }
    }
}

/// Read the current APLL frequency in Hz (AST2600 SSP).
#[cfg(feature = "ast2600-ssp")]
pub fn apll_hz() -> u32 {
    use ast2600_clk::*;
    let reg = scu().read32(APLL_PARAM);
    if reg & (1 << 24) != 0 {
        CLKIN_HZ // bypass
    } else {
        let m = (reg & 0x1FFF) as u32;
        let n = ((reg >> 13) & 0x3F) as u32;
        let p = ((reg >> 19) & 0xF) as u32;
        // Multiply first in u64 to avoid truncation: CLKIN/(N+1)*(M+1) drops
        // remainder bits when CLKIN is not evenly divisible by (N+1).
        (CLKIN_HZ as u64 * (m + 1) as u64 / (n + 1) as u64 / (p + 1) as u64) as u32
    }
}

// Re-export chip-specific UART clock constant at the module level so other
// HAL modules (uart.rs) can reference `crate::clock::UART_CLK_HZ` uniformly.
// The fallback constant allows host-side unit tests to compile without a chip feature.
#[cfg(feature = "ast1060")]
pub use ast1060_clk::UART5_CLK_24M_HZ as UART_CLK_HZ;
#[cfg(feature = "ast2600-ssp")]
pub use ast2600_clk::UART_CLK_HZ;
#[cfg(not(any(feature = "ast2600-ssp", feature = "ast1060")))]
pub const UART_CLK_HZ: u32 = 24_000_000 / 13; // placeholder for host test builds

// ── AST1060 ───────────────────────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
pub(crate) mod ast1060_clk {
    /// H-PLL parameter register (SCU200).
    /// FreqOut = 25 MHz × (M+1) / (N+1) / (P+1).
    /// Default: M=0x77(119), N=2, P=0 → 1000 MHz.
    pub const HPLL_PARAM: usize = 0x200;

    /// Clock selection register 4 (SCU310).
    /// [11:8] = PCLK divider, [4] = UART5 clock source.
    pub const CLK_SEL4: usize = 0x310;

    /// Crystal oscillator input (25 MHz).
    pub const CLKIN_HZ: u32 = 25_000_000;

    /// UART5 clock when SCU310[4]=0: 24 MHz / 13.
    pub const UART5_CLK_24M_HZ: u32 = 24_000_000 / 13; // 1,846,153 Hz

    /// UART5 clock when SCU310[4]=1: 192 MHz / 13.
    pub const UART5_CLK_192M_HZ: u32 = 192_000_000 / 13; // 14,769,230 Hz

    /// Compute HPLL frequency from SCU200 register value.
    ///
    /// `reg` is the raw SCU200 value.
    pub fn hpll_from_reg(reg: u32) -> u32 {
        if reg & (1 << 24) != 0 {
            // bypass mode: HPLL output = CLKIN
            CLKIN_HZ
        } else if reg & (1 << 23) != 0 {
            // power-down: clock stopped (return 0)
            0
        } else {
            let m = (reg & 0x1FFF) as u32;
            let n = ((reg >> 13) & 0x3F) as u32;
            let p = ((reg >> 19) & 0xF) as u32;
            // FreqOut = 25 MHz × (M+1) / (N+1) / (P+1).
            // Use u64 intermediate to avoid truncation when CLKIN % (N+1) != 0.
            (CLKIN_HZ as u64 * (m + 1) as u64 / (n + 1) as u64 / (p + 1) as u64) as u32
        }
    }

    /// Compute PCLK from HPLL frequency and raw SCU310 value.
    ///
    /// PCLK = HPLL / (2 × (div + 1)) where div = SCU310[11:8].
    pub fn pclk_from_hpll_and_reg(hpll_hz: u32, clk_sel4: u32) -> u32 {
        let div = ((clk_sel4 >> 8) & 0xF) as u32;
        hpll_hz / (2 * (div + 1))
    }
}

/// Identifies a gateable peripheral clock (AST1060).
///
/// Values 0–31 → Group 0 (SCU080 bit = value).
/// Values 32–63 → Group 1 (SCU090 bit = value − 32).
///
/// **Warning:** Do not gate `SramMclk` — stopping the SRAM clock halts
/// the CPU.
#[cfg(feature = "ast1060")]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ClockGate {
    /// SRAM controller clock (SCU080[0]). Stopped by default = 0 (running).
    /// Do **not** disable this clock.
    SramMclk = 0,
    /// HACE (Hash & Crypto Engine) clock (SCU080[13]). Stopped by default.
    HaceYclk = 13,

    /// I3C0 controller clock (SCU090[8]). Stopped by default.
    /// Enum value = 32 + bit (32 + 8 = 40).
    I3c0Clk = 40,
    /// I3C1 controller clock (SCU090[9]).
    I3c1Clk = 41,
    /// I3C2 controller clock (SCU090[10]).
    I3c2Clk = 42,
    /// I3C3 controller clock (SCU090[11]).
    I3c3Clk = 43,
    /// RSA/ECC engine clock (SCU090[6]). Stopped by default.
    /// Enum value = 32 + 6 = 38.
    RsaEccClk = 38,
}

/// Identifies a clock whose rate can be queried (AST1060).
#[cfg(feature = "ast1060")]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ClockId {
    /// H-PLL output (default 1000 MHz; read from SCU200).
    Hpll,
    /// APB peripheral clock = HPLL / (2 × (SCU310[11:8] + 1)).
    /// Default: HPLL/2 = 500 MHz.
    Pclk,
    /// UART5 clock source. Frequency depends on SCU310[4]:
    /// 0 = 24 MHz / 13 ≈ 1,846,153 Hz; 1 = 192 MHz / 13 ≈ 14,769,231 Hz.
    Uart5,
    /// I3C clock = HPLL / ((SCU310[30:28] + 1) × 2) — default HPLL/10 = 100 MHz.
    I3c,
}

/// Return the frequency in Hz for a given clock (AST1060).
#[cfg(feature = "ast1060")]
pub fn get_rate(id: ClockId) -> u32 {
    use ast1060_clk::*;
    match id {
        ClockId::Hpll => {
            let reg = scu().read32(HPLL_PARAM);
            hpll_from_reg(reg)
        }
        ClockId::Pclk => {
            let hpll = get_rate(ClockId::Hpll);
            let sel = scu().read32(CLK_SEL4);
            pclk_from_hpll_and_reg(hpll, sel)
        }
        ClockId::Uart5 => {
            let sel = scu().read32(CLK_SEL4);
            if sel & (1 << 4) == 0 {
                UART5_CLK_24M_HZ
            } else {
                UART5_CLK_192M_HZ
            }
        }
        ClockId::I3c => {
            // I3C = HPLL / (2 × (SCU310[30:28] + 1)); default [30:28]=4 → ÷10.
            let hpll = get_rate(ClockId::Hpll);
            let sel = scu().read32(CLK_SEL4);
            let div = ((sel >> 28) & 0x7) as u32;
            hpll / (2 * (div + 1))
        }
    }
}

// ── Shared clock gate API (both chips) ────────────────────────────────────────

/// Enable (un-gate) a peripheral clock.
///
/// Writes 1 to the appropriate CLR register to start the clock.
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub fn clock_enable(gate: ClockGate) {
    let (clr, bit) = gate_reg_bit(gate);
    if let Some(clr) = clr {
        scu().write32(clr, 1 << bit);
    }
}

/// Disable (gate) a peripheral clock.
///
/// Writes 1 to the appropriate SET register to stop the clock.
///
/// **AST1060 users:** Do not gate `ClockGate::SramMclk`.
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
pub fn clock_disable(gate: ClockGate) {
    let (_, bit) = gate_reg_bit(gate);
    let set = if (gate as u8) < 32 {
        CLK_STOP0_SET
    } else if (gate as u8) < 64 {
        CLK_STOP1_SET
    } else {
        return;
    };
    scu().write32(set, 1 << bit);
}

/// Returns `(clr_reg_offset, bit_index)` for a given gate.
#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
fn gate_reg_bit(gate: ClockGate) -> (Option<usize>, u32) {
    let v = gate as u8 as u32;
    if v < 32 {
        (Some(CLK_STOP0_CLR), v)
    } else if v < 64 {
        (Some(CLK_STOP1_CLR), v - 32)
    } else {
        (None, 0)
    }
}

// ── Host-side unit tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    // AST2600 SSP tests
    #[cfg(feature = "ast2600-ssp")]
    mod ast2600 {
        use super::super::*;

        #[test]
        fn hclk_rate() {
            assert_eq!(get_rate(ClockId::HCLK), 200_000_000);
        }

        #[test]
        fn uart_rate() {
            assert_eq!(get_rate(ClockId::UART), 1_846_153);
        }

        #[test]
        fn gate_enum_bits() {
            assert_eq!(ClockGate::UART5CLK as u8, 15);
            assert_eq!(ClockGate::MAC1CLK as u8, 20);
            assert_eq!(ClockGate::UART11CLK as u8, 59);
            let (_, bit) = gate_reg_bit(ClockGate::UART11CLK);
            assert_eq!(bit, 27); // 59 - 32 = 27
        }
    }

    // AST1060 tests — use pure helper functions, no hardware reads.
    #[cfg(feature = "ast1060")]
    mod ast1060 {
        use super::super::{ast1060_clk, gate_reg_bit, ClockGate};

        #[test]
        fn hpll_default_params() {
            // Default SCU200: M=0x77(119), N=2, P=0 → 25 × 120/3/1 = 1000 MHz.
            let default_reg: u32 = (0 << 23) | (0 << 24) | (0 << 19) | (2 << 13) | 0x77;
            assert_eq!(ast1060_clk::hpll_from_reg(default_reg), 1_000_000_000);
        }

        #[test]
        fn pclk_default_div() {
            // Default SCU310[11:8]=0 → PCLK = 1000 MHz / (2×1) = 500 MHz.
            assert_eq!(
                ast1060_clk::pclk_from_hpll_and_reg(1_000_000_000, 0),
                500_000_000
            );
        }

        #[test]
        fn uart5_rate_24mhz_source() {
            assert_eq!(ast1060_clk::UART5_CLK_24M_HZ, 1_846_153);
        }

        #[test]
        fn i3c0_gate_bit() {
            // I3c0Clk = 40 → Group 1, bit = 40 - 32 = 8
            let (_, bit) = gate_reg_bit(ClockGate::I3c0Clk);
            assert_eq!(bit, 8);
        }

        #[test]
        fn hace_gate_bit() {
            // HaceYclk = 13 → Group 0, bit = 13
            let (_, bit) = gate_reg_bit(ClockGate::HaceYclk);
            assert_eq!(bit, 13);
        }
    }
}
