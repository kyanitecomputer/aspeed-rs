//! AST2700 SSP/TSP Cortex-M4 release helpers.

use aspeed_mmio::{poll_until, MmioBlock};

const SCU0: usize = 0x12C0_2000;
const ASPEED_DRAM_BASE: u64 = 0x8000_0000;
const SYS_DRAM_BASE: u64 = 0x4_0000_0000;
const MAX_ID_ADDR: u32 = 512 * 1024 * 1024;
const SSP_TCM_SIZE: u32 = 8 * 1024;
const SSP_DRAM_WINDOW_SIZE: u32 = 0x0588_0000;
const TSP_DRAM_WINDOW_SIZE: u32 = 0x0200_0000;

const CTRL_DEBUG_RESET: u32 = (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);
const CTRL_ENABLE_RESET: u32 = (1 << 1) | (1 << 0);

#[inline(always)]
fn wr32(addr: usize, val: u32) {
    let mut regs = unsafe { MmioBlock::new(SCU0) };
    regs.write32(addr - SCU0, val)
}

#[inline(always)]
fn rd32(addr: usize) -> u32 {
    let regs = unsafe { MmioBlock::new(SCU0) };
    regs.read32(addr - SCU0)
}

#[inline(always)]
fn setbits32(addr: usize, val: u32) {
    wr32(addr, rd32(addr) | val);
}

fn dram_remap(load_addr: usize) -> u32 {
    (((load_addr as u64 - ASPEED_DRAM_BASE) | SYS_DRAM_BASE) >> 4) as u32
}

fn poll_sram_ready(ctrl_addr: usize) {
    let _ = poll_until(|| rd32(ctrl_addr), |v| v & 0xE0 == 0, 100_000);
}

/// Configure SSP remaps while SSP is held in reset.
pub fn init_ssp(load_addr: usize, visible_size: u32, cacheable: bool) {
    let _ = cacheable;
    let _ = visible_size;
    wr32(SCU0 + 0x200, 1 << 30);
    wr32(SCU0 + 0x204, 1 << 30);
    wr32(SCU0 + 0x120, CTRL_DEBUG_RESET);
    poll_sram_ready(SCU0 + 0x120);

    wr32(SCU0 + 0x150, 0);
    wr32(SCU0 + 0x154, SSP_DRAM_WINDOW_SIZE);
    wr32(SCU0 + 0x148, SSP_DRAM_WINDOW_SIZE);
    wr32(
        SCU0 + 0x14C,
        MAX_ID_ADDR - SSP_DRAM_WINDOW_SIZE - SSP_TCM_SIZE,
    );
    wr32(SCU0 + 0x140, MAX_ID_ADDR - SSP_TCM_SIZE);
    wr32(SCU0 + 0x144, SSP_TCM_SIZE);

    wr32(SCU0 + 0x124, (SYS_DRAM_BASE >> 4) as u32);
    wr32(SCU0 + 0x128, dram_remap(load_addr));

    wr32(SCU0 + 0x12C, 0xFFFF_FFFF);
    wr32(SCU0 + 0x130, 0xFFFF_FFFF);
    wr32(SCU0 + 0x138, 0x3);
}

/// Trigger SSP enable.
pub fn enable_ssp() {
    setbits32(SCU0 + 0x120, CTRL_ENABLE_RESET);
}

/// Configure TSP remap while TSP is held in reset.
pub fn init_tsp(load_addr: usize, visible_size: u32, cacheable: bool) {
    let _ = cacheable;
    let _ = visible_size;
    wr32(SCU0 + 0x220, 1 << 9);
    wr32(SCU0 + 0x224, 1 << 9);
    wr32(SCU0 + 0x160, CTRL_DEBUG_RESET);
    poll_sram_ready(SCU0 + 0x160);

    wr32(SCU0 + 0x194, TSP_DRAM_WINDOW_SIZE);
    wr32(SCU0 + 0x168, dram_remap(load_addr));

    wr32(SCU0 + 0x16C, 0xFFFF_FFFF);
    wr32(SCU0 + 0x170, 0xFFFF_FFFF);
    wr32(SCU0 + 0x178, 0x3);
}

/// Trigger TSP enable.
pub fn enable_tsp() {
    setbits32(SCU0 + 0x160, CTRL_ENABLE_RESET);
}
