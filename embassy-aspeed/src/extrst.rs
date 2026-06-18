//! AST2700 external reset mask initialization.
//!
//! Uses `MmioBlock` for safe volatile register access (derive-mmio pattern).

use aspeed_mmio::MmioBlock;

const SCU0_BASE: usize = 0x12C0_2000;
const SCU1_BASE: usize = 0x14C0_2000;

const RESET_LOG1: usize = 0x050;
const HW_STRAP1: usize = 0x010;
const MODRST2_CTRL: usize = 0x220;
const EXTRST_MASK: usize = 0x2F0;

/// Program vendor external reset masks after a power-on reset.
pub fn init() {
    let scu0 = unsafe { MmioBlock::new(SCU0_BASE) };
    let mut scu1 = unsafe { MmioBlock::new(SCU1_BASE) };

    if scu0.read32(RESET_LOG1) & 1 == 0 {
        return;
    }

    let mut scu0 = unsafe { MmioBlock::new(SCU0_BASE) };
    scu0.write32(EXTRST_MASK, 0x8207_FF71);
    scu0.write32(EXTRST_MASK + 4, 0x0000_03F6);
    scu1.write32(EXTRST_MASK, 0x0000_93EC);
    scu1.write32(EXTRST_MASK + 4, 0x4030_3801);
    scu1.write32(EXTRST_MASK + 8, 0x0032_0000);

    if scu1.read32(HW_STRAP1) & (1 << 3) != 0 {
        scu1.write32(EXTRST_MASK + 4, 0x4030_3801 | (1 << 1));
    }

    scu1.write32(MODRST2_CTRL, 1 << 15);
}
