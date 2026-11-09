//! AST2500 SDRAM controller: DDR4 initialisation and training.
//!
//! The sequence: M-PLL, controller reset through WDT3, controller and PHY
//! setup, a PHY-side and a DRAM-side Vref sweep each qualified by the
//! controller's CBR test engine, the read margin check, the size probe,
//! cache enable, a final engine test and optional ECC initialisation. The
//! PHY register window at `0x1E6E_0200` has no published layout; its values
//! are fixed settings.

pub mod train;

use aspeed_mmio::{poll_until, MmioBlock};
use aspeed_pac as pac;
use pac::sdmc_ast2500_v1 as regs;

use crate::ast2500::timer::Delay;
use train::{Capacity, ScanError, Window};

/// MCR00 unlock key; any other value locks.
const UNLOCK_KEY: u32 = 0xFC60_0309;
/// DDR PHY register window.
const PHY_BASE: usize = 0x1E6E_0200;
/// PHY: DRAM-side Vref (bits 13:8) and the training mode.
const PHY_DRAM_VREF: usize = 0xC0;
/// PHY: PHY-side Vref for both byte lanes.
const PHY_PHY_VREF: usize = 0xCC;
/// PHY: initialisation status; bits 19 and 17 flag a failed calibration.
const PHY_STATUS: usize = 0x100;
const PHY_STATUS_ERROR: u32 = 0x000A_0000;
/// PHY: gate training pass window, both byte lanes (MCR3A0).
const PHY_GATE_WINDOW: usize = 0x1A0;
/// PHY: read data eye pass window, both byte lanes (MCR3D0).
const PHY_EYE_WINDOW: usize = 0x1D0;
/// DRAM base on the AST2500.
pub const DRAM_BASE: u32 = 0x8000_0000;
/// PHY initialisation attempts before the whole training restarts.
const PHY_INIT_ATTEMPTS: usize = 16;
/// PHY initialisation timeout.
const PHY_INIT_TIMEOUT_US: u32 = 300_000;

/// DDR4 data rate. Below 1600 MT/s the bus runs the DDR4-1600 timings on a
/// slower M-PLL, which buys noise margin on boards or parts that need it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    Mt1440,
    Mt1488,
    Mt1536,
    Mt1600,
}

impl Speed {
    /// SCU20 M-PLL parameter for a 24 MHz or 25 MHz CLKIN.
    const fn mpll(self, clkin_25mhz: bool) -> u32 {
        match (self, clkin_25mhz) {
            (Speed::Mt1600, false) => 0x9300_2400,
            (Speed::Mt1600, true) | (Speed::Mt1536, false) => 0x9300_23E0,
            (Speed::Mt1536, true) | (Speed::Mt1488, false) => 0x9300_23C0,
            (Speed::Mt1488, true) | (Speed::Mt1440, false) => 0x9300_23A0,
            (Speed::Mt1440, true) => 0x9300_2380,
        }
    }
}

/// DDR4 nominal on-die termination (MR1 RTT_NOM).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ddr4Odt {
    /// RZQ/4.
    Ohm60,
    /// RZQ/5, for parts with less noise margin.
    Ohm48,
}

impl Ddr4Odt {
    /// MCR30 MR1/MR3 with the DLL enabled.
    const fn mr13(self) -> u32 {
        match self {
            Ddr4Odt::Ohm60 => 0x0000_0101,
            Ddr4Odt::Ohm48 => 0x0000_0501,
        }
    }
}

/// Board choices for [`init`].
#[derive(Debug, Clone, Copy)]
pub struct DramConfig {
    pub speed: Speed,
    pub ddr4_odt: Ddr4Odt,
    /// Bytes from [`DRAM_BASE`] to protect with ECC; `None` leaves it off.
    /// Its 1/8 code storage must fit below the VGA carve-out.
    pub ecc_bytes: Option<u32>,
}

/// DDR4-1600 controller timings and fixed PHY settings.
const AC_TIMING: [u32; 3] = [0x6360_4E37, 0xE97A_FA99, 0x0001_9000];
const MR46: u32 = 0x0800_0000;
const MR5: u32 = 0x0000_0400;
const MR02: u32 = 0x0000_0410;
const PHY_214: u32 = 0x0000_0024;
const PHY_2E0_2F8: [u32; 7] = [
    0x0300_2900,
    0x0E00_00A0,
    0x000E_001C,
    0x35B8_C106,
    0x0808_0607,
    0x9B00_0900,
    0x0E40_0A00,
];
/// tRFC per capacity, 128 MiB in the low byte.
const TRFC: u32 = 0x9971_4545;
/// MCR120 compatible-mode M-PLL value once training is done.
const COMPAT_MPLL: u32 = 0x0000_71C1;
/// Maximum grant lengths, MCR40..MCR4C.
const MAX_GRANT: [u32; 4] = [0x8844_8844, 0x2442_2288, 0x2222_2222, 0x2222_2222];

/// Fatal configuration problems; training failures are retried.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The strap selects DDR3.
    Ddr3,
    /// The ECC range is not a whole number of MiB, or does not fit.
    EccRange,
}

/// Why one training attempt failed; [`init`] restarts from the controller
/// reset after each.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrainError {
    /// The PHY never reported a clean initialisation.
    PhyInit,
    /// PHY-side Vref sweep.
    PhyVref(ScanError<()>),
    /// DRAM-side Vref sweep.
    DramVref(ScanError<()>),
    /// Gate (MCR3A0) or read-eye (MCR3D0) window too small.
    Margin { gate: u32, eye: u32 },
    /// The engine test after the cache was enabled failed.
    Test,
}

/// Training results worth logging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Training {
    /// Failed attempts before this one.
    pub retries: u32,
    pub phy_vref: Window,
    pub dram_vref: Window,
    /// MCR3A0 gate training pass window.
    pub gate: u32,
    /// MCR3D0 read data eye pass window.
    pub eye: u32,
    /// Microseconds spent, M-PLL to done.
    pub elapsed_us: u32,
}

/// Initialised DRAM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Dram {
    pub capacity: Capacity,
    /// VGA carve-out at the top of DRAM (SCU70 strap).
    pub vga_bytes: u32,
    /// Bytes at [`DRAM_BASE`] the payload may use: the ECC range, or the
    /// capacity below the VGA carve-out.
    pub usable_bytes: u32,
    /// `None` when SCU40 showed an earlier boot's initialisation.
    pub training: Option<Training>,
}

fn phy() -> MmioBlock {
    // SAFETY: fixed AST2500 DDR PHY window, used only by this module.
    unsafe { MmioBlock::new(PHY_BASE) }
}

fn vga_bytes() -> u32 {
    (8 << 20) << pac::SCU.HWSTRAP().read().VGA_MEM_SIZE()
}

/// Initialise DDR4 from SCU70's straps, retrying failed training from the
/// controller reset until it passes. `on_retry` sees each failed attempt.
/// Expects an unlocked SCU and WDT3 stopped; leaves the controller locked
/// and timer 3 stopped.
///
/// Skips training when SCU40 reports DRAM already initialised: a WDT reset
/// with the normal mask leaves the controller running.
pub fn init(cfg: &DramConfig, mut on_retry: impl FnMut(TrainError)) -> Result<Dram, Error> {
    let strap = pac::SCU.HWSTRAP().read();
    if !strap.DDR4() {
        return Err(Error::Ddr3);
    }
    let vga = vga_bytes();

    pac::SCU
        .VGA_HANDSHAKE0()
        .modify(|w| w.set_DRAM_INIT_BY_SOC(true));
    pac::SDMC.PROTECT().write(|w| w.set_KEY(UNLOCK_KEY));
    let training = if pac::SCU.VGA_HANDSHAKE0().read().DRAM_INIT_READY() {
        None
    } else {
        let delay = Delay::new();
        let start = delay.now();
        set_mpll(cfg.speed, strap.CLKIN_25MHZ(), &delay);
        let mut retries = 0;
        let mut t = loop {
            match train(cfg, &delay) {
                Ok(t) => break t,
                Err(e) => {
                    retries += 1;
                    on_retry(e)
                }
            }
        };
        t.retries = retries;
        t.elapsed_us = delay.elapsed_us(start);
        pac::SCU.VGA_HANDSHAKE0().modify(|w| {
            w.set_DRAM_INIT_READY(true);
            w.set_IKVM_WIDE_SCREEN(true);
        });
        pac::SCU.MISC_CTRL().modify(|w| w.set_VGA_CRT_DIS(false));
        Some(t)
    };

    let capacity = Capacity::from_bits(pac::SDMC.CONFIG().read().DRAM_SIZE().to_bits());
    let usable_bytes = match cfg.ecc_bytes {
        Some(bytes) if train::ecc_fits(bytes, capacity.bytes() - vga) => {
            enable_ecc(bytes);
            bytes
        }
        Some(_) => return Err(Error::EccRange),
        None => capacity.bytes() - vga,
    };

    pac::SDMC.PROTECT().write(|w| w.set_KEY(0));
    Ok(Dram {
        capacity,
        vga_bytes: vga,
        usable_bytes,
        training,
    })
}

/// Blank the VGA output, program the M-PLL and let it settle.
fn set_mpll(speed: Speed, clkin_25mhz: bool, delay: &Delay) {
    pac::SCU.MISC_CTRL().modify(|w| w.set_VGA_CRT_DIS(true));
    if clkin_25mhz {
        pac::SCU
            .UART_24M_REF()
            .write_value(pac::scu_ast2500_v1::SCU_UART_24M_REF(0x0001_1320));
    }
    pac::SCU
        .MPLL()
        .write_value(pac::scu_ast2500_v1::SCU_PLL(speed.mpll(clkin_25mhz)));
    delay.delay_us(3_000);
}

/// Reset the controller through WDT3 (reset mask: SDRAM only), unlock it and
/// hold off requests for 10 ms.
fn reset_controller(delay: &Delay) {
    let wdt = pac::WDT3;
    wdt.RESET_MASK()
        .write_value(pac::wdt_ast2500_v1::WDT_RESET_MASK(0x0000_0004));
    wdt.RELOAD()
        .write_value(pac::wdt_ast2500_v1::WDT_COUNTER(1));
    wdt.RESTART().write(|w| w.set_KEY(0x4755));
    wdt.CTRL().write(|w| {
        w.set_WDT_EN(true);
        w.set_RST_SYS(true);
        w.set_CLK_1MHZ(true);
    });
    while wdt.CTRL().read().RST_SYS() {
        core::hint::spin_loop();
    }
    wdt.RESET_MASK()
        .write_value(pac::wdt_ast2500_v1::WDT_RESET_MASK(0x023F_FFF3));
    wdt.RELOAD()
        .write_value(pac::wdt_ast2500_v1::WDT_COUNTER(1_000_000));
    wdt.RESTART().write(|w| w.set_KEY(0x4755));
    wdt.CLEAR_TIMEOUT_STATUS()
        .write_value(pac::wdt_ast2500_v1::WDT_CLEAR_TIMEOUT_STATUS(0x77));
    wdt.CTRL().write(|_| {});

    let sdmc = pac::SDMC;
    loop {
        sdmc.PROTECT().write(|w| w.set_KEY(UNLOCK_KEY));
        if sdmc.PROTECT().read().KEY() == 1 {
            break;
        }
    }
    sdmc.POWER_CTRL().write(|w| w.set_REQ_IN_DIS(true));
    delay.delay_us(10_000);
}

/// Request and PHY setup common to every attempt, before the DDR4 specific
/// programming.
fn setup_common(phy: &mut MmioBlock) {
    let sdmc = pac::SDMC;
    sdmc.POWER_CTRL()
        .write_value(regs::SDMC_POWER_CTRL(0x0002_0080));
    sdmc.GFX_PROTECT()
        .write_value(regs::SDMC_REQ_MASK(0x2003_000F));
    sdmc.REQ_LIMIT_MASK()
        .write_value(regs::SDMC_REQ_MASK(0xFFFF_EBFF));
    for (n, v) in MAX_GRANT.iter().enumerate() {
        sdmc.MAX_GRANT(n).write_value(regs::SDMC_MAX_GRANT(*v));
    }
    sdmc.INT_CTRL().write(|w| w.set_CLEAR(true));
    for offset in [0x08, 0x18, 0x20, 0x28, 0x30, 0xA8, 0xB0] {
        phy.write32(offset, 0);
    }
    phy.write32(0x40, 0x8600_0000);
    phy.write32(0x44, 0x0000_8600);
    phy.write32(0x48, 0x8000_0000);
    phy.write32(0x4C, 0x8080_8080);
}

/// One attempt, from the controller reset to the post-cache engine test.
fn train(cfg: &DramConfig, delay: &Delay) -> Result<Training, TrainError> {
    let sdmc = pac::SDMC;
    let mut phy = phy();
    reset_controller(delay);
    setup_common(&mut phy);

    sdmc.CONFIG().write_value(regs::SDMC_CONFIG(0x0000_0313));
    sdmc.AC_TIMING1()
        .write_value(regs::SDMC_AC_TIMING1(AC_TIMING[0]));
    sdmc.AC_TIMING2()
        .write_value(regs::SDMC_AC_TIMING2(AC_TIMING[1]));
    sdmc.AC_TIMING3()
        .write_value(regs::SDMC_AC_TIMING3(AC_TIMING[2]));
    sdmc.MR46().write_value(regs::SDMC_MR46(MR46));
    sdmc.MR5().write_value(regs::SDMC_MR5(MR5));
    sdmc.MR02().write_value(regs::SDMC_MR02(MR02 | 0x100));
    sdmc.MR13()
        .write_value(regs::SDMC_MR13(cfg.ddr4_odt.mr13()));
    phy.write32(0x00, 0x4249_2AAE);
    phy.write32(0x04, 0x0900_2800);
    phy.write32(0x0C, 0x55E0_0B0B);
    phy.write32(0x10, 0x2000_0000);
    phy.write32(0x14, PHY_214);
    for (i, v) in PHY_2E0_2F8.iter().enumerate() {
        phy.write32(0xE0 + 4 * i, *v);
    }
    phy.write32(0x90, 0x0010_0008);
    phy.write32(0xC4, 0x3C18_3C3C);
    phy.write32(0xC8, 0x0063_1E0E);
    sdmc.POWER_CTRL()
        .write_value(regs::SDMC_POWER_CTRL(0x0001_A991));

    let phy_vref = train::scan(
        || {
            set_compat_mpll(1);
            self::phy().write32(PHY_DRAM_VREF, 0x0000_1C06);
            0x40..=0x7F
        },
        |vref| {
            let mut phy = self::phy();
            fire_phy(false);
            phy.write32(PHY_PHY_VREF, u32::from(vref) * 0x0101);
            fire_phy(true);
            phy_init(delay)?;
            let pass = cbr_test();
            Ok(pass.then(|| train::eye_metric(phy.read32(PHY_EYE_WINDOW))))
        },
    )
    .map_err(|e| scan_error(e, TrainError::PhyVref))?;
    phy.write32(PHY_PHY_VREF, u32::from(phy_vref.best) * 0x0101);

    let dram_vref = train::scan(
        || {
            set_compat_mpll(2);
            0x01..=0x3F
        },
        |vref| {
            fire_phy(false);
            self::phy().write32(PHY_DRAM_VREF, 0x06 | u32::from(vref) << 8);
            fire_phy(true);
            phy_init(delay)?;
            Ok(cbr_test().then_some(0))
        },
    )
    .map_err(|e| scan_error(e, TrainError::DramVref))?;
    fire_phy(false);
    phy.write32(PHY_DRAM_VREF, 0x06 | u32::from(dram_vref.center()) << 8);

    set_compat_mpll(3);
    fire_phy(true);
    phy_init(delay).map_err(|()| TrainError::PhyInit)?;
    let (gate, eye) = check_margins()?;

    sdmc.REFRESH().write_value(regs::SDMC_REFRESH(0x42AA_5C81));
    sdmc.POWER_CTRL()
        .write_value(regs::SDMC_POWER_CTRL(0x0001_AF93));
    set_compat_mpll(COMPAT_MPLL);

    size_and_cache(TRFC);
    engine_test()?;

    Ok(Training {
        retries: 0,
        phy_vref,
        dram_vref,
        gate,
        eye,
        elapsed_us: 0,
    })
}

fn scan_error(e: ScanError<()>, wrap: fn(ScanError<()>) -> TrainError) -> TrainError {
    match e {
        ScanError::Probe(()) => TrainError::PhyInit,
        e => wrap(e),
    }
}

/// Gate and read-eye windows after the final PHY initialisation.
fn check_margins() -> Result<(u32, u32), TrainError> {
    let phy = phy();
    let gate = phy.read32(PHY_GATE_WINDOW);
    let eye = phy.read32(PHY_EYE_WINDOW);
    if train::margins_ok(gate, eye) {
        Ok((gate, eye))
    } else {
        Err(TrainError::Margin { gate, eye })
    }
}

/// MCR120 selects the PHY training step while training, and holds the
/// compatible-mode M-PLL value afterwards.
fn set_compat_mpll(value: u32) {
    pac::SDMC
        .COMPAT_MPLL()
        .write_value(regs::SDMC_COMPAT_MPLL(value));
}

/// MCR60: 5 fires PHY initialisation (reset released), 0 holds the PHY in
/// reset.
fn fire_phy(on: bool) {
    pac::SDMC.PHY_CTRL1().write(|w| {
        w.set_INIT(on);
        w.set_RESET_DIS(on);
    });
}

/// Wait for a fired PHY initialisation; refire on timeout or on a failed
/// calibration, then leave MCR60 at 6 (auto update, reset released).
fn phy_init(delay: &Delay) -> Result<(), ()> {
    let phy = phy();
    for _ in 0..PHY_INIT_ATTEMPTS {
        let start = delay.now();
        let mut done = false;
        while delay.elapsed_us(start) < PHY_INIT_TIMEOUT_US {
            if !pac::SDMC.PHY_CTRL1().read().INIT() {
                done = true;
                break;
            }
        }
        if done && phy.read32(PHY_STATUS) & PHY_STATUS_ERROR == 0 {
            pac::SDMC.PHY_CTRL1().write(|w| {
                w.set_AUTO_UPDATE(true);
                w.set_RESET_DIS(true);
            });
            return Ok(());
        }
        fire_phy(false);
        delay.delay_us(10);
        fire_phy(true);
    }
    Err(())
}

/// Run one engine command; `true` when it finished without failure.
fn engine(cmd: u32, max_polls: u32) -> bool {
    let test = pac::SDMC.TEST_CTRL();
    test.write_value(regs::SDMC_TEST_CTRL(0));
    test.write_value(regs::SDMC_TEST_CTRL(cmd));
    poll_until(|| test.read(), |v| v.DONE() || v.FAIL(), max_polls).is_ok() && !test.read().FAIL()
}

/// CBR test: one write/read pass and four burst data patterns over 64 KiB.
fn cbr_test() -> bool {
    let sdmc = pac::SDMC;
    sdmc.REFRESH().write_value(regs::SDMC_REFRESH(0x0000_5C01));
    sdmc.TEST_ADDR()
        .write_value(regs::SDMC_TEST_ADDR(0x0000_FFFF));
    sdmc.TEST_INIT_VAL()
        .write_value(regs::SDMC_VALUE32(0xFF00_FF00));
    let pass = engine(0x85, 0x5_0000) && (0..4u32).all(|gen| engine(0xC1 | gen << 3, 0x2_0000));
    sdmc.TEST_CTRL().write_value(regs::SDMC_TEST_CTRL(0));
    sdmc.REFRESH().write_value(regs::SDMC_REFRESH(0));
    pass
}

/// Probe the size by aliasing, set capacity and tRFC (`trfc` packs one byte
/// per capacity), then enable the controller cache.
fn size_and_cache(trfc: u32) {
    let sdmc = pac::SDMC;
    // SAFETY: DRAM is trained; the probe addresses lie inside the 1 GiB
    // window the controller decodes.
    let mut dram = unsafe { MmioBlock::new(DRAM_BASE as usize) };
    const PROBES: [(usize, u32); 4] = [
        (0x2010_0000, 0x4142_4344),
        (0x1010_0000, 0x3536_3738),
        (0x0810_0000, 0x292A_2B2C),
        (0x0010_0000, 0x1D1E_1F10),
    ];
    for (offset, pattern) in PROBES {
        dram.write32(offset, pattern);
    }
    let alive = |i: usize| dram.read32(PROBES[i].0) == PROBES[i].1;
    let capacity = Capacity::from_probe(alive(2), alive(1), alive(0));

    sdmc.CONFIG()
        .modify(|w| w.set_DRAM_SIZE(regs::DramSize::from_bits(capacity as u8)));
    sdmc.AC_TIMING2()
        .modify(|w| w.set_TRFC(capacity.trfc(trfc)));

    let config = sdmc.CONFIG().read();
    let mut init = config;
    init.set_CACHE_INIT_EN(true);
    sdmc.CONFIG().write_value(init);
    while !sdmc.CONFIG().read().CACHE_INIT_DONE() {
        core::hint::spin_loop();
    }
    let mut enabled = config;
    enabled.set_CACHE_EN(true);
    sdmc.CONFIG().write_value(enabled);

    sdmc.MISC().write(|w| w.set_REQ_THRESHOLD(8));
    sdmc.REQ_LIMIT_MASK()
        .write_value(regs::SDMC_REQ_MASK(0xFFFF_FF00));
}

/// One burst pass over 64 KiB with the cache on.
fn engine_test() -> Result<(), TrainError> {
    let sdmc = pac::SDMC;
    sdmc.TEST_ADDR()
        .write_value(regs::SDMC_TEST_ADDR(0x0000_FFFF));
    sdmc.TEST_INIT_VAL()
        .write_value(regs::SDMC_VALUE32(0xFF00_FF00));
    let pass = engine(0xC1, 0x2_0000);
    sdmc.TEST_CTRL().write_value(regs::SDMC_TEST_CTRL(0));
    pass.then_some(()).ok_or(TrainError::Test)
}

/// Enable ECC over `[DRAM_BASE, DRAM_BASE + bytes)`, initialise its codes
/// with the engine, clear the error counters and start auto-scrubbing.
fn enable_ecc(bytes: u32) {
    let sdmc = pac::SDMC;
    sdmc.CONFIG().modify(|w| {
        w.set_ECC_EN(true);
        w.set_CACHE_RANGE_EN(true);
    });
    sdmc.ECC_RANGE()
        .write_value(regs::SDMC_ECC_RANGE(train::ecc_range(bytes)));
    sdmc.TEST_INIT_VAL().write_value(regs::SDMC_VALUE32(0));
    sdmc.TEST_ADDR().write_value(regs::SDMC_TEST_ADDR(0));
    sdmc.TEST_CTRL()
        .write_value(regs::SDMC_TEST_CTRL(0x0000_0221));
    while !sdmc.TEST_CTRL().read().DONE() {
        core::hint::spin_loop();
    }
    sdmc.TEST_CTRL().write_value(regs::SDMC_TEST_CTRL(0));
    sdmc.INT_CTRL().write(|w| w.set_CLEAR(true));
    sdmc.INT_CTRL().write(|_| {});
    sdmc.TEST_CTRL().write(|w| w.set_ECC_SCRUB_EN(true));
}
