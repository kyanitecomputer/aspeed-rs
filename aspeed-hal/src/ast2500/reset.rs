//! AST2500 power-on reset procedure.
//!
//! A cold boot comes up with the fast-reset strap flag (SCU70\[27\]) set, and
//! not every block is fully reset then. Before the DRAM controller is
//! touched, the LPC controller state is cleared and WDT1 takes one SoC reset
//! with the full reset mask; the second pass clears the flag and goes on.

use aspeed_mmio::MmioBlock;
use aspeed_pac as pac;

/// AHB controller: protection key and the bus-lock registers to clear.
const AHBC_BASE: usize = 0x1E60_0000;
const AHBC_UNLOCK_KEY: u32 = 0xAEED_1A03;
/// FMC9C: writing this key clears the software strap flag.
const FMC_BASE: usize = 0x1E62_0000;
const FMC_SW_STRAP_CLEAR: u32 = 0xAEED_FC20;
/// SCU00 protection key.
const SCU_UNLOCK_KEY: u32 = 0x1688_A8A8;
/// WDT restart magic.
const WDT_RESTART_KEY: u16 = 0x4755;
/// WDT1 reset mask: every block, for the first reset.
const RESET_MASK_FULL: u32 = 0x03FF_FFFF;
/// WDT1 reset mask restored afterwards: all but the SDRAM controller, the
/// AHB bridges, MCTP, X-DMA and the SPI controller, so a watchdog reset
/// keeps DRAM running.
const RESET_MASK_NORMAL: u32 = 0x023F_FFF3;

/// LPC, KCS/BT and UART state to reset before the first SoC reset, as
/// offsets from `0x1E78_0000`. Reads clear status.
const LPC_APB_BASE: usize = 0x1E78_0000;
enum Op {
    W(usize, u32),
    R(usize),
}
#[rustfmt::skip]
const LPC_CLEAR: &[Op] = &[
    Op::W(0x7008, 0x7), Op::W(0x700C, 0x3), Op::W(0x7020, 0), Op::W(0x7034, 0),
    Op::W(0x7004, 0), Op::W(0x7010, 0), Op::W(0x701C, 0),
    Op::R(0x7014), Op::R(0x7018), Op::R(0x7008),
    Op::R(0x301C), Op::R(0xD01C), Op::R(0xE01C), Op::R(0xF01C),
    Op::W(0x8020, 0), Op::W(0x8034, 0), Op::W(0x800C, 0),
    Op::W(0x9008, 0), Op::W(0x9010, 0x40),
    Op::R(0x9024), Op::R(0x9028), Op::R(0x902C), Op::R(0x9114), Op::R(0x9124),
    Op::W(0x903C, 0), Op::W(0x9040, 0), Op::W(0x9044, 0), Op::W(0x911C, 0),
    Op::W(0x912C, 0x200), Op::W(0x9104, 0xCC00), Op::W(0x9108, 0),
    Op::W(0x910C, 0x1F0), Op::W(0x9170, 0), Op::W(0x9174, 0),
    Op::W(0x90A0, 0xFF00), Op::W(0x90A4, 0), Op::W(0x9080, 0x400),
    Op::W(0x9084, 0x0001_000F), Op::W(0x9088, 0x3000_FFF8),
    Op::W(0x908C, 0xFFF8_F007), Op::W(0x9098, 0xA30), Op::W(0x909C, 0),
    Op::W(0x9100, 0), Op::W(0x9130, 0x80), Op::W(0x9138, 0x0001_0198),
    Op::W(0x9140, 0xA000), Op::W(0x9158, 0x80), Op::W(0x9180, 0xB6DB_1BFF),
    Op::W(0x9184, 0), Op::W(0x9188, 0), Op::W(0x918C, 0),
    Op::W(0x9190, 0x0502_0100), Op::W(0x9194, 0x0700_0706), Op::W(0x9198, 0),
    Op::W(0x919C, 0x30), Op::W(0x91A0, 0x8100), Op::W(0x91A4, 0x2000),
    Op::W(0x91A8, 0x3FF), Op::W(0x91AC, 0),
    Op::W(0x9240, 0xFF), Op::W(0x9244, 0xFF), Op::W(0x9248, 0x80),
    Op::W(0x9250, 0), Op::W(0x9254, 0),
];

/// Outcome of [`initial_reset`] when it returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitialReset {
    /// The fast-reset flag was clear: no reset was needed.
    NotNeeded,
    /// This is the pass after the first reset; the flag is now cleared.
    Completed,
}

/// Clear the AHB bus lock, then take the first SoC reset if the fast-reset
/// flag is set. Diverges into the reset on the first pass of a cold boot.
///
/// Leaves the SCU unlocked.
pub fn initial_reset() -> InitialReset {
    // SAFETY: fixed AST2500 AHB controller and FMC addresses.
    let mut ahbc = unsafe { MmioBlock::new(AHBC_BASE) };
    ahbc.write32(0x00, AHBC_UNLOCK_KEY);
    ahbc.write32(0x84, 0x0001_0000);
    ahbc.write32(0x88, 0);

    pac::SCU.PROTECT().write(|w| w.set_KEY(SCU_UNLOCK_KEY));
    if !pac::SCU.HWSTRAP().read().ICE_FAST_RESET() {
        return InitialReset::NotNeeded;
    }

    let wdt = pac::WDT1;
    if wdt.TIMEOUT_STATUS().read().0 != 0 {
        wdt.CLEAR_TIMEOUT_STATUS()
            .write_value(pac::wdt_ast2500_v1::WDT_CLEAR_TIMEOUT_STATUS(0x77));
        pac::SCU.HWSTRAP_CLR().write(|w| w.set_ICE_FAST_RESET(true));
        let status = pac::SCU.SYSRST_STATUS().read().0;
        pac::SCU
            .SYSRST_STATUS()
            .write_value(pac::scu_ast2500_v1::SCU_SYSRST_STATUS(status & 1));
        wdt.RESET_MASK()
            .write_value(pac::wdt_ast2500_v1::WDT_RESET_MASK(RESET_MASK_NORMAL));
        return InitialReset::Completed;
    }

    // SAFETY: fixed AST2500 LPC/KCS/UART register window.
    let mut lpc = unsafe { MmioBlock::new(LPC_APB_BASE) };
    for op in LPC_CLEAR {
        match *op {
            Op::W(offset, value) => lpc.write32(offset, value),
            Op::R(offset) => {
                lpc.read32(offset);
            }
        }
    }
    // SAFETY: fixed AST2500 FMC address.
    unsafe { MmioBlock::new(FMC_BASE) }.write32(0x9C, FMC_SW_STRAP_CLEAR);

    wdt.RELOAD()
        .write_value(pac::wdt_ast2500_v1::WDT_COUNTER(1));
    wdt.RESTART().write(|w| w.set_KEY(WDT_RESTART_KEY));
    wdt.RESET_MASK()
        .write_value(pac::wdt_ast2500_v1::WDT_RESET_MASK(RESET_MASK_FULL));
    wdt.CTRL().write(|w| {
        w.set_WDT_EN(true);
        w.set_RST_SYS(true);
        w.set_CLK_1MHZ(true);
    });
    loop {
        core::hint::spin_loop();
    }
}
