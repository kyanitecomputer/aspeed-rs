//! AST2500 MAC clocks and pins, set up before an OS drives the MACs.
//!
//! The D2-PLL runs at 250 MHz and feeds the RGMII 125 MHz (/2) and RMII
//! 50 MHz (/5) clock dividers; the graphics CRT clock moves off it, onto
//! the USB2.0 PHY's 40 MHz. The MAC bus clock follows the interface straps
//! (SCU70\[7:6\]). Each MAC's clock delays, RGMII transmit clock duty, RMII
//! reference clock output and MDIO pins depend on the board. The MACs stay
//! in reset with their clocks stopped, for the OS driver to start.

use aspeed_pac as pac;

/// SCU13C parameter bits \[36:28\] for integer D2-PLL settings.
const D2PLL_PARAM: u16 = 0x2C;
/// SCU1C SIP and SIC for numerators above 99.
const D2PLL_SIP: u8 = 0x11;
const D2PLL_SIC: u8 = 0x18;
/// SCU4C divider codes off 250 MHz: 0 and 1 divide by 2, n > 1 by n + 1.
const RGMII_DIV_125MHZ: u8 = 0;
const RMII_DIV_50MHZ: u8 = 4;
/// SCU08 MAC bus clock: H-PLL/6 with an RGMII MAC, H-PLL/10 otherwise.
const MAC_AHB_DIV_RGMII: u8 = 2;
const MAC_AHB_DIV_RMII: u8 = 4;

/// One MAC's board timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mac {
    /// RGMII transmit clock output delay, 0 to 63 steps.
    pub tx_delay: u8,
    /// RGMII receive clock or RMII reference clock input delay, 0 to 63
    /// steps.
    pub rx_delay: u8,
    /// RGMII transmit clock duty calibration: bit 6 enables it, bit 5
    /// inverts the source clock, bits \[4:0\] select the stage.
    pub tx_duty: u8,
    /// Drive the RMII 50 MHz reference clock out of the RCLKO pin, for a
    /// PHY or NC-SI device that does not provide it.
    pub rmii_clock_out: bool,
    /// Route the MAC's MDC/MDIO to its pins.
    pub mdio: bool,
}

/// Both MACs' board timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacConfig {
    pub mac1: Mac,
    pub mac2: Mac,
}

/// D2-PLL M, N and P for 250 MHz: CLKIN * (M+1) / (N+1) / (P+1).
const fn d2pll_250mhz(clkin_25mhz: bool) -> (u8, u8, u8) {
    if clkin_25mhz {
        (119, 1, 5)
    } else {
        (124, 1, 5)
    }
}

/// Hold both MACs in reset and set up their clocks, delays and pins as
/// `cfg` says. The SCU must be unlocked.
pub fn init(cfg: &MacConfig) {
    let scu = pac::SCU;
    scu.SYSRST_CTRL().modify(|w| {
        w.set_MAC1(true);
        w.set_MAC2(true);
    });

    scu.D2PLL_EXT().write(|w| {
        w.set_PARAM(D2PLL_PARAM);
        w.set_RESET(true);
        w.set_POWER_DOWN(true);
    });
    scu.MISC_CTRL().modify(|w| {
        w.set_D2PLL_DIS(false);
        w.set_GCRT_CLK_SEL_HI(true);
    });
    let strap = scu.HWSTRAP().read();
    let (m, n, p) = d2pll_250mhz(strap.CLKIN_25MHZ());
    scu.D2PLL().write(|w| {
        w.set_M(m);
        w.set_N(n);
        w.set_P(p);
        w.set_SIC(D2PLL_SIC);
        w.set_SIP(D2PLL_SIP);
    });
    scu.D2PLL_EXT().write(|w| w.set_PARAM(D2PLL_PARAM));

    scu.MISC2_CTRL().modify(|w| {
        w.set_RGMII_CLK_DIV_SRC(false);
        w.set_RGMII_CLK_DIV(RGMII_DIV_125MHZ);
        w.set_RMII_CLK_DIV_SRC(false);
        w.set_RMII_CLK_DIV(RMII_DIV_50MHZ);
    });
    let rgmii = strap.MAC1_RGMII() || strap.MAC2_RGMII();
    scu.CLK_SEL().modify(|w| {
        w.set_MAC_AHB_DIV(if rgmii {
            MAC_AHB_DIV_RGMII
        } else {
            MAC_AHB_DIV_RMII
        })
    });

    scu.MAC_CLK_DELAY().write(|w| {
        w.set_MAC1_TX_DELAY(cfg.mac1.tx_delay);
        w.set_MAC2_TX_DELAY(cfg.mac2.tx_delay);
        w.set_MAC1_RX_DELAY(cfg.mac1.rx_delay);
        w.set_MAC2_RX_DELAY(cfg.mac2.rx_delay);
        w.set_RMII1_RCLK_OUT_EN(cfg.mac1.rmii_clock_out);
        w.set_RMII2_RCLK_OUT_EN(cfg.mac2.rmii_clock_out);
        w.set_RGMII_CLK_INTERNAL(true);
    });
    scu.CLK_DUTY().modify(|w| {
        w.set_RGMII1_TXCK(cfg.mac1.tx_duty);
        w.set_RGMII2_TXCK(cfg.mac2.tx_duty);
    });

    scu.PIN_CTRL5().modify(|w| {
        w.set_RGMII1_TX_PD_DIS(false);
        w.set_RGMII1_RX_PD_DIS(false);
        w.set_RGMII2_TX_PD_DIS(false);
        w.set_RGMII2_RX_PD_DIS(false);
        if cfg.mac2.mdio {
            w.set_MDIO2_EN(true);
        }
    });
    if cfg.mac1.mdio {
        scu.PIN_CTRL3().modify(|w| {
            w.set_MDC1_EN(true);
            w.set_MDIO1_EN(true);
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_d2pll_runs_at_250mhz_from_either_crystal() {
        for (clkin, crystal_25mhz) in [(24_000_000u64, false), (25_000_000, true)] {
            let (m, n, p) = d2pll_250mhz(crystal_25mhz);
            let hz = clkin * (u64::from(m) + 1) / (u64::from(n) + 1) / (u64::from(p) + 1);
            assert_eq!(hz, 250_000_000);
            assert!(m > 99, "SIP and SIC are for numerators above 99");
        }
    }
}
