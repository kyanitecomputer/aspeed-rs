//! AST2700 BootMCU watchdog reset-mask initialization.
//!
//! Uses `MmioBlock` for safe volatile register access (derive-mmio pattern).

use aspeed_mmio::MmioBlock;

const WDT_BASE: usize = 0x14C3_7000;
const WDT_STRIDE: usize = 0x80;
const WDT_COUNT: usize = 8;

const RST_MASK1: usize = 0x1C;
const SW_RST_MASK1: usize = 0x34;

const A1_MASKS: [u32; 5] = [
    0x8207_FF79,
    0x0000_03F6,
    0x0000_93EC,
    0x4030_3803,
    0x0032_0000,
];
const A0_MASKS: [u32; 5] = [
    0x0003_0421,
    0x0000_0036,
    0x0000_93EC,
    0x0130_3803,
    0x0000_0000,
];

fn delay_5us() {
    for _ in 0..2_000 {
        core::hint::spin_loop();
    }
}

fn write_masks(wdt: &mut MmioBlock, masks: [u32; 5]) {
    for (idx, mask) in masks.iter().copied().enumerate() {
        wdt.write32(RST_MASK1 + idx * 4, mask);
        delay_5us();
    }
    for (idx, mask) in masks.iter().copied().enumerate() {
        wdt.write32(SW_RST_MASK1 + idx * 4, mask);
        delay_5us();
    }
}

/// Program WDT reset masks for all AST2700 watchdog instances.
pub fn init() {
    let (_, hw) = crate::scu::silicon_rev();
    let masks = match hw {
        crate::scu::HwRev::A0 => A0_MASKS,
        // A2 defaults to the A1 masks until the A2 register deltas are verified
        // against the datasheet / BootMCU FMC; A2 is a minor A1 stepping.
        crate::scu::HwRev::A1 | crate::scu::HwRev::A2 | crate::scu::HwRev::Unknown(_) => A1_MASKS,
    };

    for idx in 0..WDT_COUNT {
        let mut wdt = unsafe { MmioBlock::new(WDT_BASE + idx * WDT_STRIDE) };
        write_masks(&mut wdt, masks);
    }
}
