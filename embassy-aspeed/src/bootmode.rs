//! AST2700 BootMCU boot-mode strap decoding.

use crate::pac;
use aspeed_mmio::MmioBlock;

const SCU1_OTPCFG_03_02: usize = 0x14c0_2884;
const OTPCFG2_DIS_RECOVERY_MODE: u32 = 1 << 3;
const HWSTRAP1_EN_RECOVERY_BOOT: u32 = 1 << 4;
const HWSTRAP1_BOOT_EMMC_UFS: u32 = 1 << 11;
const HWSTRAP1_BOOT_UFS: u32 = 1 << 23;
const HWSTRAP1_RECOVERY_INTERFACE_MASK: u32 = 0x3 << 26;
const HWSTRAP1_RECOVERY_USB: u32 = 0x1 << 26;
const HWSTRAP1_RECOVERY_I2C: u32 = 0x2 << 26;
const HWSTRAP1_RECOVERY_I3C: u32 = 0x3 << 26;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootMode {
    NorFlash,
    Emmc,
    Ufs,
    UsbDfu,
    I2cRecovery,
    I3cRecovery,
    UartRecovery,
}

impl BootMode {
    pub fn as_str(self) -> &'static str {
        match self {
            BootMode::NorFlash => "NOR",
            BootMode::Emmc => "eMMC",
            BootMode::Ufs => "UFS",
            BootMode::UsbDfu => "USB-DFU",
            BootMode::I2cRecovery => "I2C recovery",
            BootMode::I3cRecovery => "I3C recovery",
            BootMode::UartRecovery => "UART recovery",
        }
    }
}

#[inline]
fn rd32(addr: usize) -> u32 {
    let regs = unsafe { MmioBlock::new(addr) };
    regs.read32(0)
}

pub fn detect() -> BootMode {
    let dis = rd32(SCU1_OTPCFG_03_02);
    let strap = pac::SCU1.HWSTRAP1().read().0;

    if dis & OTPCFG2_DIS_RECOVERY_MODE == 0 && strap & HWSTRAP1_EN_RECOVERY_BOOT != 0 {
        return match strap & HWSTRAP1_RECOVERY_INTERFACE_MASK {
            HWSTRAP1_RECOVERY_USB => BootMode::UsbDfu,
            HWSTRAP1_RECOVERY_I2C => BootMode::I2cRecovery,
            HWSTRAP1_RECOVERY_I3C => BootMode::I3cRecovery,
            _ => BootMode::UartRecovery,
        };
    }

    if strap & HWSTRAP1_BOOT_EMMC_UFS != 0 {
        if strap & HWSTRAP1_BOOT_UFS != 0 {
            BootMode::Ufs
        } else {
            BootMode::Emmc
        }
    } else {
        BootMode::NorFlash
    }
}
