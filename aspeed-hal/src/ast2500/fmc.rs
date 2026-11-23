//! AST2500 firmware SPI controller (FMC): CE0 read mode and read timing.

use aspeed_pac as pac;

pub use crate::fmc::{Ce0Read, Error};

/// AHB clock (HCLK) from the H-PLL and the AXI/AHB ratio strap.
pub fn hclk_hz() -> u32 {
    let strap = pac::SCU.HWSTRAP().read();
    let clkin: u64 = if strap.CLKIN_25MHZ() {
        25_000_000
    } else {
        24_000_000
    };
    let pll = pac::SCU.HPLL().read();
    let hpll = if pll.BYPASS() {
        clkin
    } else {
        clkin * (u64::from(pll.M()) + 1) / ((u64::from(pll.N()) + 1) * (u64::from(pll.P()) + 1))
    };
    let ratio = u64::from(strap.AXI_AHB_RATIO()).max(1);
    (hpll / (2 * (ratio + 1))) as u32
}

/// Switch CE0 to the dual output fast read at the fastest SPI clock up to
/// `max_hz` that calibrates; see [`crate::fmc::fast_read`].
pub fn fast_read(max_hz: u32, area: u32, len: u32) -> Result<Ce0Read, Error> {
    crate::fmc::fast_read(hclk_hz(), max_hz, area, len)
}
