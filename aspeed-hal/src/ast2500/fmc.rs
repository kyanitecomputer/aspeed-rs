//! AST2500 firmware SPI controller (FMC): CE0 read mode and read timing.
//!
//! After reset CE0 reads with the normal read command (03h) at HCLK/16.
//! [`fast_read`] switches it to the dual output fast read (3Bh) at the
//! fastest SPI clock up to a limit whose read delay calibrates. Each trial
//! runs on the FMC DMA engine in checksum calibration mode, which carries its
//! own clock and delay: CE0 itself keeps its safe setting until a winner is
//! found, so code executing in place from CE0 runs on throughout.

use aspeed_pac as pac;
use pac::fmc_ast2500_v1 as regs;

/// CE0 control (FMC10) for the dual output fast read (3Bh) with one dummy
/// byte, before the clock selection.
const CE0_DUAL_FAST_READ: u32 = 0b010 << 28 | 0x3B << 16 | 1 << 6 | 0b01;
/// FMC80: DMA enable, checksum only, calibration mode.
const DMA_CALIBRATE: u32 = 0b1101;
/// Polls of the DMA status before a checksum counts as failed.
const DMA_POLLS: u32 = 1_000_000;
/// Delay steps per divisor: 0 to 5 HCLK, each without and with 4 ns.
const DELAY_STEPS: u8 = 12;
/// Divisors with a read delay field in FMC94; slower clocks need none.
const TIMED_DIVISORS: u32 = 5;
/// Checksums per trial setting.
const TRIAL_REPEATS: usize = 2;

/// FMC10\[11:8\] clock selection for HCLK/`div`, `div` from 1 to 16.
const fn divisor_code(div: u32) -> u32 {
    const CODES: [u32; 16] = [
        0xF, 0x7, 0xE, 0x6, 0xD, 0x5, 0xC, 0x4, 0xB, 0x3, 0xA, 0x2, 0x9, 0x1, 0x8, 0x0,
    ];
    CODES[div as usize - 1]
}

/// FMC94 delay field for step `step`: HCLK cycles in bits \[2:0\], the
/// 4 ns data input delay in bit 3.
const fn delay_code(step: u8) -> u32 {
    (step / 2) as u32 | if step & 1 != 0 { 8 } else { 0 }
}

/// Smallest HCLK divisor whose SPI clock does not exceed `max_hz`.
const fn first_divisor(hclk: u32, max_hz: u32) -> u32 {
    let div = hclk.div_ceil(if max_hz == 0 { 1 } else { max_hz });
    if div == 0 {
        1
    } else if div > 16 {
        16
    } else {
        div
    }
}

/// The delay step to use from pass/fail results in increasing delay: the
/// step before the third consecutive pass, which has a passing step on
/// either side.
fn pick_delay(passes: impl IntoIterator<Item = bool>) -> Option<u8> {
    let mut run = 0;
    for (step, pass) in (0..).zip(passes) {
        run = if pass { run + 1 } else { 0 };
        if run == 3 {
            return Some(step - 1);
        }
    }
    None
}

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

/// CE0's read setting after [`fast_read`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ce0Read {
    /// SPI clock is HCLK divided by this.
    pub divisor: u32,
    pub spi_hz: u32,
    /// FMC94 read delay field for the divisor, if it has one.
    pub delay: Option<u32>,
}

/// Why [`fast_read`] left CE0 in normal read mode at HCLK/16.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The calibration area reads mostly as all zeros or all ones.
    UniformArea,
    /// The DMA engine never finished the reference checksum.
    Dma,
    /// No divisor up to HCLK/16 read the area correctly.
    NoTiming,
}

/// Switch CE0 to the dual output fast read at the fastest SPI clock up to
/// `max_hz` that reads the `len` bytes at `area` (an address in the CE0
/// window, word aligned) correctly. The reference is their checksum in
/// normal read mode at HCLK/16, the reset setting, so a flash without the
/// dual output read fails every trial rather than calibrating against
/// garbage. Divisors up to 5 calibrate a read delay with one passing step
/// either side; slower ones need a single matching checksum.
///
/// On failure CE0 is back in normal read mode at HCLK/16.
pub fn fast_read(max_hz: u32, area: u32, len: u32) -> Result<Ce0Read, Error> {
    let fmc = pac::FMC;
    let ce0 = fmc.CE_N_CTRL(0);
    ce0.write_value(regs::FMC_CE_N_CTRL(0));
    if !varied(area, len) {
        return Err(Error::UniformArea);
    }
    let golden = checksum(area, len, divisor_code(16), 0).ok_or(Error::Dma)?;

    ce0.write_value(regs::FMC_CE_N_CTRL(CE0_DUAL_FAST_READ));
    let hclk = hclk_hz();
    for divisor in first_divisor(hclk, max_hz)..=16 {
        let code = divisor_code(divisor);
        let matches =
            |delay| (0..TRIAL_REPEATS).all(|_| checksum(area, len, code, delay) == Some(golden));
        let delay = if divisor <= TIMED_DIVISORS {
            match pick_delay((0..DELAY_STEPS).map(|step| matches(delay_code(step)))) {
                Some(step) => Some(delay_code(step)),
                None => continue,
            }
        } else if matches(0) {
            None
        } else {
            continue;
        };
        if let Some(delay) = delay {
            let shift = 4 * (divisor - 1);
            let timing = fmc.READ_TIMING().read().0 & !(0xF << shift) | delay << shift;
            fmc.READ_TIMING().write_value(regs::FMC_READ_TIMING(timing));
        }
        ce0.write_value(regs::FMC_CE_N_CTRL(CE0_DUAL_FAST_READ | code << 8));
        return Ok(Ce0Read {
            divisor,
            spi_hz: hclk / divisor,
            delay,
        });
    }
    ce0.write_value(regs::FMC_CE_N_CTRL(0));
    Err(Error::NoTiming)
}

/// Whether at least 64 words of the area are neither all zeros nor all
/// ones, so that a wrong read changes its checksum.
fn varied(area: u32, len: u32) -> bool {
    (area..area + len)
        .step_by(4)
        .filter(|&addr| {
            // SAFETY: the caller's area lies in the readable CE0 window.
            let word = unsafe { core::ptr::read_volatile(addr as *const u32) };
            word != 0 && word != u32::MAX
        })
        .nth(63)
        .is_some()
}

/// Checksum the area through the DMA engine in calibration mode, with SPI
/// clock selection `clock` and read delay field `delay`.
fn checksum(area: u32, len: u32, clock: u32, delay: u32) -> Option<u32> {
    let fmc = pac::FMC;
    fmc.DMA_FLASH_ADDR().write_value(regs::FMC_VALUE32(area));
    fmc.DMA_LEN().write_value(regs::FMC_VALUE32(len));
    fmc.DMA_CTRL()
        .write_value(regs::FMC_DMA_CTRL(DMA_CALIBRATE | clock << 4 | delay << 8));
    let done = (0..DMA_POLLS).any(|_| fmc.IRQ_CTRL().read().DMA_STATUS());
    let sum = fmc.DMA_CHECKSUM().read().0;
    fmc.DMA_CTRL().write_value(regs::FMC_DMA_CTRL(0));
    done.then_some(sum)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisor_codes_follow_the_clock_selection_table() {
        assert_eq!(divisor_code(1), 0xF);
        assert_eq!(divisor_code(2), 0x7);
        assert_eq!(divisor_code(4), 0x6);
        assert_eq!(divisor_code(5), 0xD);
        assert_eq!(divisor_code(16), 0x0);
    }

    #[test]
    fn delay_steps_alternate_the_4ns_input_delay() {
        let codes: [u32; 4] = core::array::from_fn(|i| delay_code(i as u8));
        assert_eq!(codes, [0x0, 0x8, 0x1, 0x9]);
        assert_eq!(delay_code(11), 0xD);
    }

    #[test]
    fn the_divisor_keeps_the_clock_at_or_below_the_limit() {
        assert_eq!(first_divisor(198_000_000, 100_000_000), 2);
        assert_eq!(first_divisor(198_000_000, 50_000_000), 4);
        assert_eq!(first_divisor(198_000_000, 200_000_000), 1);
        assert_eq!(first_divisor(198_000_000, 1_000_000), 16);
        assert_eq!(first_divisor(198_000_000, 0), 16);
    }

    #[test]
    fn the_delay_has_a_passing_step_either_side() {
        let at = |passes: &[bool]| pick_delay(passes.iter().copied());
        assert_eq!(at(&[false, true, true, true, true]), Some(2));
        assert_eq!(at(&[true, true, false, true, true, true]), Some(4));
        assert_eq!(at(&[true, true, false, true, true]), None);
        assert_eq!(at(&[true; 12]), Some(1));
    }
}
