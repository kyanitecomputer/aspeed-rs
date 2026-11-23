//! AST2400 firmware SPI controller (FMC): CE0 read mode and read timing.

use aspeed_pac as pac;

pub use crate::fmc::{Ce0Read, Error};

/// H-PLL output for the strap's default setting (SCU70\[9:8\]), in MHz,
/// from a 24/48 MHz and from a 25 MHz CLKIN.
const HPLL_DEFAULT_MHZ: [[u64; 4]; 2] = [[384, 360, 336, 408], [400, 375, 350, 425]];

/// H-PLL output in Hz: the strap default unless SCU24 selects its own
/// parameters, CLKIN when bypassed. The PLL reference is 25 MHz with a
/// 25 MHz CLKIN and 24 MHz otherwise (a 48 MHz CLKIN is halved).
fn hpll_hz(clkin_25mhz: bool, strap_default: u8, pll: pac::scu_ast2400_v1::SCU_HPLL) -> u64 {
    let reference: u64 = if clkin_25mhz { 25_000_000 } else { 24_000_000 };
    if pll.BYPASS() {
        reference
    } else if !pll.PARAM_SEL() {
        HPLL_DEFAULT_MHZ[usize::from(clkin_25mhz)][usize::from(strap_default & 3)] * 1_000_000
    } else {
        reference * (2 - u64::from(pll.OD())) * (u64::from(pll.NUM()) + 2)
            / (u64::from(pll.DENUM()) + 1)
    }
}

/// CPU/AHB ratio for strap SCU70\[11:10\].
const fn ahb_divisor(code: u8) -> u64 {
    match code & 3 {
        0 => 1,
        1 => 2,
        2 => 4,
        _ => 3,
    }
}

/// AHB clock (HCLK): the H-PLL, which clocks the CPU, divided by the
/// CPU/AHB ratio strap.
pub fn hclk_hz() -> u32 {
    let strap = pac::SCU.HWSTRAP().read();
    let hpll = hpll_hz(
        strap.CLKIN_25MHZ(),
        strap.HPLL_DEFAULT(),
        pac::SCU.HPLL().read(),
    );
    (hpll / ahb_divisor(strap.CPU_AHB_RATIO())) as u32
}

/// Switch CE0 to the dual output fast read at the fastest SPI clock up to
/// `max_hz` that calibrates; see [`crate::fmc::fast_read`].
pub fn fast_read(max_hz: u32, area: u32, len: u32) -> Result<Ce0Read, Error> {
    crate::fmc::fast_read(hclk_hz(), max_hz, area, len)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pac::scu_ast2400_v1::SCU_HPLL;

    #[test]
    fn the_strap_picks_the_default_hpll() {
        // SCU24 reset value: parameters from the strap.
        let pll = SCU_HPLL(0x291);
        assert_eq!(hpll_hz(false, 0, pll), 384_000_000);
        assert_eq!(hpll_hz(false, 3, pll), 408_000_000);
        assert_eq!(hpll_hz(true, 0, pll), 400_000_000);
    }

    #[test]
    fn programmed_hpll_follows_the_formula() {
        // NUM 30, DENUM 1, OD 0: 24 MHz * 2 * 32 / 2 = 768 MHz.
        let pll = SCU_HPLL(1 << 18 | 30 << 5 | 1);
        assert_eq!(hpll_hz(false, 0, pll), 768_000_000);
        // OD 1 halves it.
        let pll = SCU_HPLL(1 << 18 | 30 << 5 | 1 << 4 | 1);
        assert_eq!(hpll_hz(false, 0, pll), 384_000_000);
        let bypass = SCU_HPLL(1 << 17);
        assert_eq!(hpll_hz(true, 0, bypass), 25_000_000);
    }

    #[test]
    fn the_ratio_strap_divides_the_cpu_clock() {
        assert_eq!([0, 1, 2, 3].map(ahb_divisor), [1, 2, 4, 3]);
    }
}
