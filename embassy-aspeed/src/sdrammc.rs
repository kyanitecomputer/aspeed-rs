//! AST2700 SDRAM Memory Controller (DRAMC) + DWC DDR PHY driver.
//!
//! Clean-room Rust reimplementation.  Register accesses use the generated PAC
//! (aspeed-data/data/registers/sdrammc_ast2700_v1.yaml).  PHY APB accesses
//! are raw MMIO (16-bit APB, stride 2 bytes per register).
//!
//! Timing values and PHY PUB register values are derived from AST2700 vendor
//! expressed as Rust const tables — no C code is used.
//!
//! # Sequence (AST2700 datasheet §26.4.1 + DWC PHY docs)
//!
//! 1.  MPLL re-lock (read-modify-write RESET bit only; M/N/P set by ROM)
//! 2.  Enable PHY clock gate (SCU0 CLK_STOP_CLR)
//! 3.  Unlock SDRAMMC (PROTECT = 0x1688A8A8)
//! 4.  Configure MAIN_CONF + AC timing + DFI + REFCTL + ZQCTL
//! 5.  PHY power-on: three writes to MAIN_CTRL (cold reset sequence)
//! 6.  Step C: ~80 DWC PHY PUB register writes (config before training)
//! 7.  D: Load PHY training IMEM from ASTH prebuilt on SPI flash
//! 8.  E: Mailbox init (2 PHY APB writes)
//! 9.  F: Load PHY training DMEM (message block with training parameters)
//! 10. G: Poll PHY mailbox (0xd0054) until training complete (msg 0x07)
//! 11. J: Enter mission mode — clear PHY training bits, trigger
//!         MAIN_CTRL.INIT_TRIGGER, poll INTR_STS.DDRPHY_INIT_DONE
//! 12. Exit self-refresh: MAIN_CTRL.SELF_REF_TRIGGER, poll SELF_REF_DONE
//! 13. Configure MRS (DDR4 mode registers via DRAMC MR_CTRL)
//! 14. Enable auto-refresh (REFRESH_CTRL)
//! 15. ZQ short calibration
//! 16. Lock (PROTECT = 0xDEADDEAD)

use crate::pac;

use crate::prebuilt::{PrebuiltTable, PrebuiltType};
use crate::scu;
use aspeed_mmio::MmioBlock;

// ── Constants ─────────────────────────────────────────────────────────────────

const UNLOCK_KEY: u32 = 0x1688_A8A8;
const SOFT_LOCK_KEY: u32 = 0;

/// DWC DDR PHY base address.
/// PHY APB: byte_addr = PHY_BASE + 2 * phy_apb_word_addr.
const PHY_BASE: usize = 0x1300_0000;

/// PHY IMEM start: PHY APB word addr 0x50000 → byte offset 0xA0000.
const PHY_IMEM_BASE: usize = PHY_BASE + 0x000A_0000;
/// PHY DMEM start: PHY APB word addr 0x58000 → byte offset 0xB0000.
const PHY_DMEM_BASE: usize = PHY_BASE + 0x000B_0000;

/// SCU0 clock stop clear register (write-1-to-clear).
/// Bit 11 = DDR PHY clock gate.
const SCU0_CLK_STOP_CLR: usize = 0x12C0_2244;
const SCU0_CLK_PHY_BIT: u32 = 1 << 11;

/// SCU0 MPLL parameter and extended registers.
const SCU0_MPLL_PARAM: usize = 0x12C0_2310;
const SCU0_MPLL_EXT: usize = 0x12C0_2314;
const MPLL_RESET: u32 = 1 << 25;
const MPLL_BYPASS: u32 = 1 << 24;
const MPLL_LOCK: u32 = 1 << 31;

/// MAIN_CTRL bit values (DRAMC + 0x14).
const MCTL_PHY_RESET: u32 = 1 << 17;
const MCTL_PHY_POWER_ON: u32 = 1 << 16;
const MCTL_PHY_INIT_START: u32 = 1 << 0;

/// WDT0 registers for DRAMC soft-reset (IO-die WDT, 0x14C37000).
const WDT0_SW_RST_KICK: usize = 0x14C3_7030;
const WDT0_SW_RST_SEL0: usize = 0x14C3_7034;
const WDT0_SW_RST_KICK_KEY: u32 = 0xAEED_F123;
const WDT0_SW_RST_DRAMC_BIT: u32 = 1 << 1;

const SCU0_VGA0_SCRATCH: usize = 0x12C0_2900;
const SCU0_VGA1_SCRATCH: usize = 0x12C0_2910;
const DRAMC_INIT_DONE: u32 = 1 << 6;

/// Polling timeout — generous for training (>20 seconds at 200 MHz).
const POLL_TIMEOUT: u32 = 1_000_000_000;
const TRAINING_ATTEMPTS: usize = 5;

/// SCU1 MCU0 control register.
/// Bits [22:16] = MAP1: maps MCU0 0xC0000000..0xFFFFFFFF to a physical DRAM region.
const SCU1_MCU0_CTRL: usize = 0x14C0_2110;
const SCU1_MCU0_MAP1_MASK: u32 = 0x007F_0000; // GENMASK(22, 16)
const SCU1_MCU0_MAP1_SHIFT: u32 = 16;

/// MCU0 upper window base (via MAP1).
const DRAM_TEST_ADDR: usize = 0xC000_0000;
/// MCU0 lower window base (via MAP0, fixed to DRAM 0).
const DRAM_START_ADDR: usize = 0x8000_0000;

/// Per-size: (MAP1 value, DDR4 tRFC/2, DDR5 tRFC/2).
/// Indices: 0=256MB, 1=512MB, 2=1GB, 3=2GB, 4=4GB, 5=8GB.
const DRAM_SIZE_TABLE: [(u32, u32, u32); 6] = [
    (0x40, 128, 104), // 256MB  (DDR4: 256/2, DDR5: 208/2)
    (0x40, 208, 104), // 512MB  (DDR4: 416/2, DDR5: 208/2)
    (0x40, 280, 104), // 1GB    (DDR4: 560/2, DDR5: 208/2)
    (0x44, 440, 236), // 2GB    (DDR4: 880/2, DDR5: 472/2)
    (0x48, 440, 328), // 4GB    (DDR4: 880/2, DDR5: 656/2)
    (0x50, 440, 440), // 8GB    (DDR4: 880/2, DDR5: 880/2)
];
/// First size index to test (minimum assumed is 1GB, test starts from 2GB).
const SDRAM_SIZE_2GB_IDX: usize = 3;
const SDRAM_SIZE_COUNT: usize = 6;

// ── DRAMC register helpers ─────────────────────────────────────────────────────

#[inline(always)]
fn dramc() -> pac::sdrammc_ast2700_v1::SDRAMMC {
    pac::SDRAMMC
}

#[inline]
fn unlock() {
    dramc().PROTECT().write_value(UNLOCK_KEY);
}

#[inline]
fn lock() {
    // do not hard-lock DRAMC with 0xDEADDEAD
    // before CA35 handoff.  Hard-lock is irreversible until reset and differs
    // from the working BootMCU stacks.  A non-unlock value soft-locks registers.
    dramc().PROTECT().write_value(SOFT_LOCK_KEY);
}

// ── Raw MMIO helpers (MmioBlock-backed, absolute-address interface) ────────────
//
// sdrammc.rs uses many absolute MMIO addresses for DRAMC, PHY, SCU, and WDT
// registers.  These helpers provide the same flat-address API as raw volatile
// calls while routing access through MmioBlock for volatile correctness.

#[inline(always)]
fn rd_raw(addr: usize) -> u32 {
    let block = unsafe { MmioBlock::new(addr) };
    block.read32(0)
}

#[inline(always)]
fn wr_raw(addr: usize, val: u32) {
    let mut block = unsafe { MmioBlock::new(addr) };
    block.write32(0, val);
}

/// Raw read of DRAMC MAIN_CTRL register via MmioBlock.
#[inline]
fn mctl_read() -> u32 {
    let block = unsafe { MmioBlock::new(0x12C0_0000usize) };
    block.read32(0x14)
}

/// Raw write of DRAMC MAIN_CTRL register via MmioBlock.
#[inline]
fn mctl_write(val: u32) {
    let mut block = unsafe { MmioBlock::new(0x12C0_0000usize) };
    block.write32(0x14, val);
}

// ── PHY APB helpers ───────────────────────────────────────────────────────────

/// Write one 16-bit value to DWC DDR PHY APB register.
/// byte_addr = PHY_BASE + 2 * apb_word_addr.
#[inline]
fn phy_write(apb_addr: u32, val: u16) {
    let byte_addr = PHY_BASE + 2 * apb_addr as usize;
    let mut block = unsafe { MmioBlock::new(byte_addr) };
    block.write16(0, val);
}

/// Read one 16-bit value from DWC DDR PHY APB register.
#[inline]
fn phy_read(apb_addr: u32) -> u16 {
    let byte_addr = PHY_BASE + 2 * apb_addr as usize;
    let block = unsafe { MmioBlock::new(byte_addr) };
    block.read16(0)
}

#[derive(Clone, Copy)]
enum PhyWidth {
    W16,
    W32,
}

#[derive(Clone, Copy)]
struct PhyWrite {
    addr: u32,
    val: u32,
    width: PhyWidth,
}

#[inline]
fn phy_write32(apb_addr: u32, val: u32) {
    let byte_addr = PHY_BASE + 2 * apb_addr as usize;
    let mut block = unsafe { MmioBlock::new(byte_addr) };
    block.write32(0, val);
}

fn apply_phy_writes(writes: &[PhyWrite]) {
    for write in writes {
        match write.width {
            PhyWidth::W16 => phy_write(write.addr, write.val as u16),
            PhyWidth::W32 => phy_write32(write.addr, write.val),
        }
    }
}

/// Write PHY firmware blob (IMEM or DMEM) to PHY memory via MmioBlock.
///
/// Uses 32-bit writes — the PHY SRAM bus bridge requires word-aligned
/// 32-bit transactions.  Each 4-byte chunk from the blob maps to two
/// consecutive 16-bit PHY SRAM words.
fn load_phy_blob(dst_byte_base: usize, src: &[u8]) {
    let mut offset = 0usize;
    for chunk in src.chunks(4) {
        let mut word = 0u32;
        for (j, &b) in chunk.iter().enumerate() {
            word |= (b as u32) << (j * 8);
        }
        let mut block = unsafe { MmioBlock::new(dst_byte_base + offset) };
        block.write32(0, word);
        offset += 4;
    }
}

/// Busy-wait approximately `n` microseconds at ~200 MHz.
/// Conservative: 1000 nop iterations ≈ 5 µs at 200 MHz.
fn delay_us(us: u32) {
    for _ in 0..(us * 200) {
        unsafe { core::arch::asm!("nop") };
    }
}

// ── MPLL re-lock ─────────────────────────────────────────────────────────────

/// Re-lock the MPLL without changing M/N/P (set by ROM).
///
/// Read-modify-write: cycle RESET+BYPASS bits, wait for LOCK.
fn mpll_relock() {
    let param = rd_raw(SCU0_MPLL_PARAM);
    wr_raw(SCU0_MPLL_PARAM, param | MPLL_BYPASS | MPLL_RESET);
    wr_raw(SCU0_MPLL_PARAM, param | MPLL_BYPASS);
    wr_raw(SCU0_MPLL_PARAM, param);
    while rd_raw(SCU0_MPLL_EXT) & MPLL_LOCK == 0 {
        core::hint::spin_loop();
    }
}

// ── AC timing and DRAMC configuration ────────────────────────────────────────
//
// DDR4-3200 and DDR5-3200 speed grade register values
// tables. Fields are stored as half-cycle counts (field_val = timing_cycles/2)
// in the DRAMC hardware.
//
// Register layout (from ASPEED DRAMC register map, sdrammc_ast2700_v1.yaml):
//   ACTIME1 [31:24]=tCCD_L [27:24]=tRRD_L [23:20]=tRRD [15:8]=tMRD
//   ACTIME2 [31:24]=tFAW   [23:16]=tRP    [15:8]=tRAS   [7:0]=tRCD
//   ACTIME3 [31:24]=(WL+BL/2+tWTR_S)/2 [23:16]=RTW/2 [15:8]=(WL+BL/2+tWR)/2 [7:0]=tRTP/2
//   ACTIME4 [31:24]=(WL+BL/2+tWTR_A)/2 [23:16]=(WL+BL/2+tWTR_L)/2
//   ACTIME5 [31:20]=tREFSBRD/2 [19:10]=tRFCsb/2 [9:0]=tRFC/2
//   ACTIME6 [31:24]=tCSHSR/2 [23:16]=tPD/2 [15:8]=tXP/2 [7:0]=tCKSRE/2
//   ACTIME7 [31:16]=tZQ/2 [15:0]=tDLLK/2

/// DDR4-3200 DRAMC register values.
/// t_cl=22, t_cwl=16, t_bl=8; derived fields computed per DRAMC spec.
struct Ddr4Regs;
impl Ddr4Regs {
    const ACTIME1: u32 = (8 << 24) | (5 << 16) | (4 << 8) | 12;

    const ACTIME2: u32 = (24 << 24) | (11 << 16) | (26 << 8) | 11;

    const ACTIME3: u32 = (12 << 24) | (6 << 16) | (22 << 8) | 6;

    const ACTIME4: u32 = (10 << 8) | 16;

    // tREFSBRD=0, tRFCsb=0, tRFC=880→440
    const ACTIME5: u32 = 440;

    // tCSHSR=0, tPD=8→4, tXP=10→5, tCKSRE=16→8
    const ACTIME6: u32 = (4 << 16) | (5 << 8) | 8;

    // tZQ=80→40, tDLLK=1023→511
    // Vendor macro: ACTIME7(zqcs, dllk) = ((zqcs >> 1) << 10) | (dllk >> 1)
    const ACTIME7: u32 = (40 << 10) | 511;

    // DFI timing: t_phy_wrlat=7 (CWL-5-4=7), t_phy_wrdata=2,
    //   t_phy_rddata_en=13 (CL-5-4=13), t_phy_odtlat=7
    // [5:0]=wrlat [8:6]=wrdata [15:10]=rddata_en [19:16]=odtlat
    const DFI_TIMING: u32 = (7 << 16) | (13 << 10) | (2 << 6) | 7;

    // MAIN_CONF raw: 0x20 | (size<<2) | DDR4_type=0
    // bit 5 always set; size field comes entirely from the dram_size argument.
    // Size is NOT pre-baked here — it is OR'd in at init() and corrected by
    // size_detect() before lock().
    const MAIN_CONF_BASE: u32 = 0x20;

    // REFRESH_CTRL: 0x40B48200
    // (includes tREFI encoding specific to 1600 MHz DDR4)
    const REFCTL: u32 = 0x40B4_8200;

    // ZQ control: 0x42AA1800
    const ZQCTL: u32 = 0x42AA_1800;

    // DDR4 mode registers from vendor sdramc_configure_mrs().
    const MR0: u32 = 0x2150;
    const MR1: u32 = 0x0201;
    const MR2: u32 = 0x0228;
    const MR3: u32 = 0x0000;
    const MR4: u32 = 0x0000;
    const MR5: u32 = 0x0420;
    const MR6: u32 = 0x1000;
}

/// DDR5-3200 DRAMC register values.
struct Ddr5Regs;
impl Ddr5Regs {
    const ACTIME1: u32 = (8 << 24) | (4 << 16) | (4 << 8) | 11;
    // tFAW=40→20, tRP=26→13, tRAS=52→26, tRCD=26→13
    const ACTIME2: u32 = (20 << 24) | (13 << 16) | (26 << 8) | 13;
    // WL+BL/2+tWTR_S=24+8+4=36→18, RTW=(CL-CWL+BL/2+2)=(26-24+8+2)=12→6
    // WL+BL/2+tWR=24+8+48=80→40, tRTP=12→6
    const ACTIME3: u32 = (18 << 24) | (6 << 16) | (40 << 8) | 6;
    // WL+BL/2+tWTR_A=24+8+36=68→34, WL+BL/2+tWTR_L=24+8+16=48→24
    const ACTIME4: u32 = (34 << 8) | 24;
    // tREFSBRD=48→24, tRFCsb=208→104, tRFC=880→440
    const ACTIME5: u32 = (24 << 20) | (104 << 10) | 440;
    // tCSHSR=30→15, tPD=13→6, tXP=13→6, tCKSRE=9→4
    const ACTIME6: u32 = (15 << 24) | (6 << 16) | (6 << 8) | 4;
    const ACTIME7: u32 = (24 << 10) | 512;
    // DFI: t_phy_wrlat=8(CWL-13-3=8), t_phy_wrdata=6, t_phy_rddata_en=10(CL-13-3=10)
    const DFI_TIMING: u32 = (10 << 10) | (6 << 6) | 8;
    const MAIN_CONF_BASE: u32 = 0x20 | 1; // DDR5 type=1 in bit 0; no size pre-baked
    const REFCTL: u32 = 0x40B4_8200;
    const ZQCTL: u32 = 0x42AA_1800;
}

// ── DWC DDR PHY Step C: pre-training PUB register configuration ───────────────
//
// These are the DWC DDR PHY APB register writes required before loading the
// IMEM training firmware.  Without them, the PHY training firmware cannot
// correctly calibrate the PHY.
//
// Format: (apb_word_addr, value) pairs.
// byte_addr = 0x13000000 + 2 * apb_word_addr.
//
// Step C (phyinit_C_initPhyConfig).  Values re-expressed as Rust const table.

/// PHY configuration table for DDR4-3200 generated from vendor Step C.
const PHY_CONFIG_DDR4: &[PhyWrite] = &[
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09029,
        val: 0x000000c4,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1005f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1015f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1105f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1115f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05055,
        val: 0x0000015a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09055,
        val: 0x0000011e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2008c,
        val: 0x00000372,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200c5,
        val: 0x00000019,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200c7,
        val: 0x00000061,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200ca,
        val: 0x0000400f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200cc,
        val: 0x000000d2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2002e,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20051,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20024,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2003a,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004d,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1014d,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104d,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1114d,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10041,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10049,
        val: 0x00000fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10141,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10149,
        val: 0x00000fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1014b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11041,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11049,
        val: 0x00000fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11141,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11149,
        val: 0x00000fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1114b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20018,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20075,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20050,
        val: 0x00000082,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20008,
        val: 0x00000320,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20088,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200b2,
        val: 0x000000f8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10043,
        val: 0x00002500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10143,
        val: 0x00002500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11043,
        val: 0x00002500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11143,
        val: 0x00002500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004c,
        val: 0x0000001c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104c,
        val: 0x0000001c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00042,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00042,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20019,
        val: 0x00000005,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f0,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f1,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f2,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f3,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f4,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f5,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f6,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f7,
        val: 0x0000f000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000b,
        val: 0x00000064,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000c,
        val: 0x000000c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000d,
        val: 0x000002bc,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000e,
        val: 0x0000002c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004a,
        val: 0x00000500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104a,
        val: 0x00000500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20025,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90307,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2002d,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20040,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200ea,
        val: 0x00001080,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xc0086,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2002b,
        val: 0x00009820,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1002b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1102b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0002b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0102b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0202b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0302b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0402b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0502b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0602b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0702b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0802b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0902b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10040,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10030,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10050,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10060,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10140,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10130,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10150,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10160,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10240,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10230,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10250,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10260,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10340,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10330,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10350,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10360,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10440,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10430,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10450,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10460,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10540,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10530,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10550,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10560,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10640,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10630,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10650,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10660,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10740,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10730,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10750,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10760,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10840,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10830,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10850,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10860,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11040,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11030,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11050,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11060,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11140,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11130,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11150,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11160,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11240,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11230,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11250,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11260,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11340,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11330,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11350,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11360,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11440,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11430,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11450,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11460,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11540,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11530,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11550,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11560,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11640,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11630,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11650,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11660,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11740,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11730,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11750,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11760,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11840,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11830,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11850,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11860,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200fa,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00028,
        val: 0x0000000c,
        width: PhyWidth::W16,
    },
];

/// PHY configuration table for DDR5-3200 generated from vendor Step C.
const PHY_CONFIG_DDR5: &[PhyWrite] = &[
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09029,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90301,
        val: 0x00000059,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90302,
        val: 0x00000058,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1005f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1015f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1105f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1115f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05055,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09055,
        val: 0x000001be,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2008c,
        val: 0x00000300,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200c5,
        val: 0x00000019,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200c7,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200ca,
        val: 0x0000402f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200cc,
        val: 0x0000017f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2002e,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20051,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20024,
        val: 0x00000088,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a1,
        val: 0x00000701,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a2,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200fe,
        val: 0x000000f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200fc,
        val: 0x000000f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200fd,
        val: 0x000000f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2003a,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004d,
        val: 0x00000082,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1014d,
        val: 0x00000082,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104d,
        val: 0x00000082,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1114d,
        val: 0x00000082,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09043,
        val: 0x0000cfff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10041,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10049,
        val: 0x0000079e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10141,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10149,
        val: 0x0000079e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1014b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11041,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11049,
        val: 0x0000079e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11141,
        val: 0x0000030c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11149,
        val: 0x0000079e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1114b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20018,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20075,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20050,
        val: 0x00000082,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20008,
        val: 0x00000320,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20088,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200b2,
        val: 0x000000f8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10043,
        val: 0x00002900,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10143,
        val: 0x00002900,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11043,
        val: 0x00002900,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11143,
        val: 0x00002900,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004c,
        val: 0x0000001c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104c,
        val: 0x0000001c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20019,
        val: 0x00000005,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f0,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f1,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f2,
        val: 0x00004444,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f3,
        val: 0x00008888,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f4,
        val: 0x00005555,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f5,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f6,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200f7,
        val: 0x0000f000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000b,
        val: 0x00000064,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000c,
        val: 0x000000c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000d,
        val: 0x000002bc,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2000e,
        val: 0x0000002c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1004a,
        val: 0x00000500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1104a,
        val: 0x00000500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20025,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2019a,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x400f5,
        val: 0x00001200,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x400f6,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20120,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20121,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20124,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20122,
        val: 0x0000080c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20123,
        val: 0x0000080c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20125,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2012e,
        val: 0x00000321,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20140,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20141,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20144,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20142,
        val: 0x0000080c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20143,
        val: 0x0000080c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20145,
        val: 0x0000080e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2014e,
        val: 0x00000321,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90307,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20040,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09066,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200ea,
        val: 0x00001080,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xc0086,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2002b,
        val: 0x00009820,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1002b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x1102b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0002b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0102b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0202b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0302b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0402b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0502b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0602b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0702b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0802b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x0902b,
        val: 0x00008020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x01066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x02066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x03066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x05066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09066,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10040,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10030,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10050,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10060,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10140,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10130,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10150,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10160,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10240,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10230,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10250,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10260,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10340,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10330,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10350,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10360,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10440,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10430,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10450,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10460,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10540,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10530,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10550,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10560,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10640,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10630,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10650,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10660,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10740,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10730,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10750,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10760,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10840,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10830,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10850,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x10860,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11040,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11030,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11050,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11060,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11140,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11130,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11150,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11160,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11240,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11230,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11250,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11260,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11340,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11330,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11350,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11360,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11440,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11430,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11450,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11460,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11540,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11530,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11550,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11560,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11640,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11630,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11650,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11660,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11740,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11730,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11750,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11760,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11840,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11830,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11850,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x11860,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200fa,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x100aa,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x110aa,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x00028,
        val: 0x0000000c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x04028,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x06028,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x07028,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x08028,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x09028,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
];

/// DDR4 1D DMEM message block writes generated from vendor Step F.
const DMEM_MSG_DDR4_1D: &[PhyWrite] = &[
    PhyWrite {
        addr: 0x58000,
        val: 0x00000100,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58002,
        val: 0x0c800000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58004,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58006,
        val: 0x10000240,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58008,
        val: 0x00000001,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800a,
        val: 0x031f0000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800c,
        val: 0x000000c8,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58010,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58012,
        val: 0x00000002,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58014,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58016,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58018,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58020,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58022,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58024,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58026,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58028,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802e,
        val: 0x21500000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58030,
        val: 0x02280101,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58032,
        val: 0x00000400,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58034,
        val: 0x104f0500,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58036,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58038,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58040,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58042,
        val: 0x0f0f0000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58044,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58046,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58048,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804a,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804c,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804e,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58050,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58052,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58054,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58056,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58058,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805a,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805c,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805e,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58060,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58062,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58064,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58066,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58068,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806a,
        val: 0x00000f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58070,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58072,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58074,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58076,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58078,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58080,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58082,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58084,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58086,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58088,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58090,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58092,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58094,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58096,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58098,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580aa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ac,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ae,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ba,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580bc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580be,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ca,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580cc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ce,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580de,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fe,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58100,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58102,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58104,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58106,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58108,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58110,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58112,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58114,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58116,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58118,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58120,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58122,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58124,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58126,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58128,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58130,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58132,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58134,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58136,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58138,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58140,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58142,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58144,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58146,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58148,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58150,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58152,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58154,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58156,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58158,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58160,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58162,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58164,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58166,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58168,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58170,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58172,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58174,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58176,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58178,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58180,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58182,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58184,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58186,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58188,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58190,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58192,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58194,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58196,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58198,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581aa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ac,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ae,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ba,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581bc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581be,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ca,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581cc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ce,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581de,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fe,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0xd0000,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
];

/// DDR4 2D DMEM message block writes generated from vendor Step F.
const DMEM_MSG_DDR4_2D: &[PhyWrite] = &[
    PhyWrite {
        addr: 0x58002,
        val: 0x0c800000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58004,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58006,
        val: 0x10000240,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58008,
        val: 0x00000001,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800a,
        val: 0x00610000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800c,
        val: 0x000000c8,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800e,
        val: 0x00008020,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58010,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58012,
        val: 0x00000002,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58014,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58016,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58018,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58020,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58022,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58024,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58026,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58028,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802e,
        val: 0x21500000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58030,
        val: 0x02280101,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58032,
        val: 0x00000400,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58034,
        val: 0x104f0500,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58036,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58038,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58040,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58042,
        val: 0x0f0f0000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58044,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58046,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58048,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804a,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804c,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804e,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58050,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58052,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58054,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58056,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58058,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805a,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805c,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805e,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58060,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58062,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58064,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58066,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58068,
        val: 0x0f0f0f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806a,
        val: 0x00000f0f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58070,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58072,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58074,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58076,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58078,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58080,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58082,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58084,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58086,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58088,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58090,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58092,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58094,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58096,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58098,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580aa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ac,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ae,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ba,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580bc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580be,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ca,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580cc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ce,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580de,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fe,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58100,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58102,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58104,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58106,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58108,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58110,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58112,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58114,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58116,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58118,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58120,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58122,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58124,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58126,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58128,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58130,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58132,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58134,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58136,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58138,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58140,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58142,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58144,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58146,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58148,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58150,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58152,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58154,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58156,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58158,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58160,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58162,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58164,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58166,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58168,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58170,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58172,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58174,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58176,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58178,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58180,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58182,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58184,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58186,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58188,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58190,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58192,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58194,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58196,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58198,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581aa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ac,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ae,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ba,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581bc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581be,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ca,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581cc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ce,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581de,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fe,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0xd0000,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
];

/// DDR5 1D DMEM message block writes generated from vendor Step F.
const DMEM_MSG_DDR5_1D: &[PhyWrite] = &[
    PhyWrite {
        addr: 0x58000,
        val: 0x00000100,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58002,
        val: 0x0c800000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58004,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58006,
        val: 0x00000040,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58008,
        val: 0x00c8827f,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800a,
        val: 0x01020000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800c,
        val: 0x10000001,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5800e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58010,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58012,
        val: 0x00000110,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58014,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58016,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58018,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5801e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58020,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58022,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58024,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58026,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58028,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5802e,
        val: 0x84080000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58030,
        val: 0x00200000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58032,
        val: 0x2d000800,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58034,
        val: 0x0000d62d,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58036,
        val: 0x04240003,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58038,
        val: 0x2c000499,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803a,
        val: 0x00002c2c,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5803e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58040,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58042,
        val: 0x00000408,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58044,
        val: 0x08000020,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58046,
        val: 0xd62d2d00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58048,
        val: 0x00030000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804a,
        val: 0x04110000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804c,
        val: 0x2c2c2c00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5804e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58050,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58052,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58054,
        val: 0x04080000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58056,
        val: 0x00200000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58058,
        val: 0x2d000800,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805a,
        val: 0x0000d62d,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805c,
        val: 0x00000003,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5805e,
        val: 0x2c000411,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58060,
        val: 0x00002c2c,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58062,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58064,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58066,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58068,
        val: 0x00000408,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806a,
        val: 0x08000020,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806c,
        val: 0xd62d2d00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5806e,
        val: 0x00030000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58070,
        val: 0x04110000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58072,
        val: 0x2c2c2c00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58074,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58076,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58078,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5807e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58080,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58082,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58084,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58086,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58088,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5808e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58090,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58092,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58094,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58096,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58098,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5809e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a4,
        val: 0x04080000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a6,
        val: 0x00200000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580a8,
        val: 0x2d000800,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580aa,
        val: 0x0000d62d,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ac,
        val: 0x00000003,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ae,
        val: 0x2c000411,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b0,
        val: 0x00002c2c,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580b8,
        val: 0x00000408,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ba,
        val: 0x08000020,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580bc,
        val: 0xd62d2d00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580be,
        val: 0x00030000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c0,
        val: 0x04110000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c2,
        val: 0x2c2c2c00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ca,
        val: 0x04080000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580cc,
        val: 0x00200000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ce,
        val: 0x2d000800,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d0,
        val: 0x0000d62d,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d2,
        val: 0x00000003,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d4,
        val: 0x2c000411,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d6,
        val: 0x00002c2c,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580de,
        val: 0x00000408,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e0,
        val: 0x08000020,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e2,
        val: 0xd62d2d00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e4,
        val: 0x00030000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e6,
        val: 0x04110000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580e8,
        val: 0x2c2c2c00,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x580fe,
        val: 0x00a00060,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58100,
        val: 0x00000001,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58102,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58104,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58106,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58108,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5810e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58110,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58112,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58114,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58116,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58118,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5811e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58120,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58122,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58124,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58126,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58128,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5812e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58130,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58132,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58134,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58136,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58138,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5813e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58140,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58142,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58144,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58146,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58148,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5814e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58150,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58152,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58154,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58156,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58158,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5815e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58160,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58162,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58164,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58166,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58168,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5816e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58170,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58172,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58174,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58176,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58178,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5817e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58180,
        val: 0x00000001,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58182,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58184,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58186,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58188,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5818e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58190,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58192,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58194,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58196,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58198,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5819e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581a8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581aa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ac,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ae,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581b8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ba,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581bc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581be,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ca,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581cc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ce,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581de,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581e8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x581fe,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58200,
        val: 0x00000001,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58202,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58204,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58206,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58208,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5820a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5820c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5820e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58210,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58212,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58214,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58216,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58218,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5821a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5821c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5821e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58220,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58222,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58224,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58226,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58228,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5822a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5822c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5822e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58230,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58232,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58234,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58236,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58238,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5823a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5823c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5823e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58240,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58242,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58244,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58246,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58248,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5824a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5824c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5824e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58250,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58252,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58254,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58256,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58258,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5825a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5825c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5825e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58260,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58262,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58264,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58266,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58268,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5826a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5826c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5826e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58270,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58272,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58274,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58276,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58278,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5827a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5827c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5827e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58280,
        val: 0x00000001,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58282,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58284,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58286,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58288,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5828a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5828c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5828e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58290,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58292,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58294,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58296,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58298,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5829a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5829c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5829e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582a4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582a6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582a8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582aa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ac,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ae,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582b0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582b8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ba,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582bc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582be,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582c0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582c2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ca,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582cc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ce,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582d0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582d2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582d4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582d6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582de,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582e0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582e2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582e4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582e6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582e8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x582fe,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58300,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58302,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58304,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58306,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58308,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5830a,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5830c,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5830e,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58310,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58312,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58314,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58316,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58318,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5831a,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5831c,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5831e,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58320,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58322,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58324,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58326,
        val: 0x17171717,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58328,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5832a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5832c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5832e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58330,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58332,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58334,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58336,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58338,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5833a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5833c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5833e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58340,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58342,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58344,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58346,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58348,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5834a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5834c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5834e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58350,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58352,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58354,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58356,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58358,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5835a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5835c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5835e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58360,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58362,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58364,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58366,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58368,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5836a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5836c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5836e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58370,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58372,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58374,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58376,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58378,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5837a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5837c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5837e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58380,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58382,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58384,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58386,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58388,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5838a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5838c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5838e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58390,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58392,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58394,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58396,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x58398,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5839a,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5839c,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x5839e,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583a0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583a2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583a4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583a6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583a8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583aa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ac,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ae,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583b0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583b2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583b4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583b6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583b8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ba,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583bc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583be,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583c0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583c2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583c4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583c6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583c8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ca,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583cc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ce,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583d0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583d2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583d4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583d6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583d8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583da,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583dc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583de,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583e0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583e2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583e4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583e6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583e8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ea,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ec,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583ee,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583f0,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583f2,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583f4,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583f6,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583f8,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583fa,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583fc,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0x583fe,
        val: 0x00000000,
        width: PhyWidth::W32,
    },
    PhyWrite {
        addr: 0xd0000,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
];

// ── Step I: PIE (PHY Init Engine) sequencer code ─────────────────────────────
//
// The PIE runs the DFI initialization handshake that responds to DRAMC's
// PHY_INIT_START.  Without PIE loaded, DDRPHY_INIT_DONE never fires.
//
// Includes MicroContMuxSel toggling, sequencer register writes, start
// vectors, disable flags, calibration arm, and PMU clock disable.

const PIE_DDR4: &[PhyWrite] = &[
    PhyWrite {
        addr: 0xd0000,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90000,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90001,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90002,
        val: 0x0000010e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90003,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90004,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90005,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90029,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002a,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002b,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002c,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002d,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002e,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90030,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90031,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90032,
        val: 0x0000000b,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90033,
        val: 0x00000480,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90034,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90035,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90036,
        val: 0x00000448,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90037,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90038,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90039,
        val: 0x00000478,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003a,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003b,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003c,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003d,
        val: 0x00000139,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003e,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003f,
        val: 0x000007c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90040,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90041,
        val: 0x00000107,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90042,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90043,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90044,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90045,
        val: 0x000000e0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90046,
        val: 0x00000139,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90047,
        val: 0x00000147,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90048,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90049,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004a,
        val: 0x0000014f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004b,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004c,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004d,
        val: 0x00000007,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004e,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004f,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90050,
        val: 0x00000047,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90051,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90052,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90053,
        val: 0x0000004f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90054,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90055,
        val: 0x00000179,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90056,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90057,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90058,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90059,
        val: 0x00000011,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005a,
        val: 0x00000530,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005b,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005c,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005d,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005e,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005f,
        val: 0x0000014f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90060,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90061,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90062,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90063,
        val: 0x0000045a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90064,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90065,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90066,
        val: 0x00000530,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90067,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90068,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90069,
        val: 0x0000065a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006a,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006b,
        val: 0x00000041,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006c,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006d,
        val: 0x00000179,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006e,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006f,
        val: 0x00000618,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90070,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90071,
        val: 0x000040c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90072,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90073,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90074,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90075,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90076,
        val: 0x00000048,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90077,
        val: 0x00004040,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90078,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90079,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007a,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007b,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007c,
        val: 0x00000048,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007d,
        val: 0x00000040,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007e,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007f,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90080,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90081,
        val: 0x00000658,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90082,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90083,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90084,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90085,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90086,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90087,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90088,
        val: 0x00000078,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90089,
        val: 0x00000549,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008a,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008b,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008c,
        val: 0x00000d49,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008d,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008e,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008f,
        val: 0x0000094c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90090,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90091,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90092,
        val: 0x0000094c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90093,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90094,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90095,
        val: 0x00000442,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90096,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90097,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90098,
        val: 0x00000042,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90099,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009a,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009b,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009c,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009d,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009e,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009f,
        val: 0x000000e0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a0,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a1,
        val: 0x0000000a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a2,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a3,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a4,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a5,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a6,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a7,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a8,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a9,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900aa,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ab,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ac,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ad,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ae,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900af,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b0,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b1,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b2,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b3,
        val: 0x0000000c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b4,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b5,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b6,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b7,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b8,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b9,
        val: 0x00000007,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ba,
        val: 0x000007c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bb,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bc,
        val: 0x0000003a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bd,
        val: 0x000001e2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900be,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bf,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c0,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c1,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c2,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c3,
        val: 0x00008140,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c4,
        val: 0x0000010c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c5,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c6,
        val: 0x00008138,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c7,
        val: 0x0000010c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c8,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c9,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ca,
        val: 0x0000010e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cb,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cc,
        val: 0x00000448,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cd,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ce,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cf,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d0,
        val: 0x00000101,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d1,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d2,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d3,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d4,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d5,
        val: 0x00000448,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d6,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d7,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d8,
        val: 0x000007c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d9,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900da,
        val: 0x00000007,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900db,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900dc,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900dd,
        val: 0x00000047,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900de,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900df,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e0,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e1,
        val: 0x00000618,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e2,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e3,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e4,
        val: 0x000000e0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e5,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e6,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e7,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e8,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e9,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ea,
        val: 0x00008140,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900eb,
        val: 0x0000010c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ec,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ed,
        val: 0x00000478,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ee,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ef,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f0,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f1,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f2,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f3,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f4,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f5,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f6,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f7,
        val: 0x00000101,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90006,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90007,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90008,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90009,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000a,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xd00e7,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90017,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90026,
        val: 0x00000038,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000c,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000d,
        val: 0x00000173,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000e,
        val: 0x00000060,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000f,
        val: 0x00006110,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90010,
        val: 0x00002152,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90011,
        val: 0x0000dfbd,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90012,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90013,
        val: 0x00006152,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2006d,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200e8,
        val: 0x00000fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20089,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20088,
        val: 0x00000019,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xc0080,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xd0000,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
];

/// PIE table for DDR5-3200 generated from vendor Step I.
const PIE_DDR5: &[PhyWrite] = &[
    PhyWrite {
        addr: 0xd0000,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90000,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90001,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90002,
        val: 0x0000010e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90003,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90004,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90005,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41000,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41001,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41002,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41003,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41004,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41005,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41006,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41007,
        val: 0x00000a01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41008,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41009,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4100a,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4100b,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4100c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4100d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4100e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4100f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41010,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41011,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41012,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41013,
        val: 0x000001c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41014,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41015,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41016,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41017,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41018,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41019,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4101a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4101b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4101c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4101d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4101e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4101f,
        val: 0x00000101,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41020,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41021,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41022,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41023,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41024,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41025,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41026,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41027,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41028,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41029,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4102a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4102b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4102c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4102d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4102e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4102f,
        val: 0x0000f901,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41030,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41031,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41032,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41033,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41034,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41035,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41036,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41037,
        val: 0x00005901,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41038,
        val: 0x000005a5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41039,
        val: 0x00004000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4103a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4103b,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4103c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4103d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4103e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4103f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41040,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41041,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41042,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41043,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41044,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41045,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41046,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41047,
        val: 0x00000a01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41048,
        val: 0x000000ef,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41049,
        val: 0x0000ef00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4104a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4104b,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4104c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4104d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4104e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4104f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41050,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41051,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41052,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41053,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41054,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41055,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41056,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41057,
        val: 0x0000ff01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41058,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41059,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4105a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4105b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4105c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4105d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4105e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4105f,
        val: 0x0000ff01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41060,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41061,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41062,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41063,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41064,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41065,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41066,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41067,
        val: 0x00000a01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41068,
        val: 0x000085d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41069,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4106a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4106b,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4106c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4106d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4106e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4106f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41070,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41071,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41072,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41073,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41074,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41075,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41076,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41077,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41078,
        val: 0x000085f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41079,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4107a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4107b,
        val: 0x00000800,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4107c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4107d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4107e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4107f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41080,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41081,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41082,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41083,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41084,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41085,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41086,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41087,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41088,
        val: 0x000045d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41089,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4108a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4108b,
        val: 0x00000401,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4108c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4108d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4108e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4108f,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41090,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41091,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41092,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41093,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41094,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41095,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41096,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41097,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41098,
        val: 0x000045f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41099,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4109a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4109b,
        val: 0x00000801,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4109c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4109d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4109e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4109f,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a8,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410a9,
        val: 0x00000062,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410aa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ab,
        val: 0x00000402,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ac,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ad,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ae,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410af,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b8,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410b9,
        val: 0x00000062,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ba,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410bb,
        val: 0x00000802,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410bc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410bd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410be,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410bf,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c8,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410c9,
        val: 0x00000061,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ca,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410cb,
        val: 0x00000403,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410cc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410cd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ce,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410cf,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d8,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410d9,
        val: 0x00000061,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410da,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410db,
        val: 0x00000803,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410dc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410dd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410de,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410df,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e7,
        val: 0x00001d01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e8,
        val: 0x00000213,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410e9,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ea,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410eb,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ec,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ed,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ee,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ef,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f7,
        val: 0x0000ef00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410f9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410fa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410fb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410fc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410fd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410fe,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x410ff,
        val: 0x00005900,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41100,
        val: 0x00000217,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41101,
        val: 0x00001700,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41102,
        val: 0x000003c2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41103,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41104,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41105,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41106,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41107,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41108,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41109,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4110a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4110b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4110c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4110d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4110e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4110f,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41110,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41111,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41112,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41113,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41114,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41115,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41116,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41117,
        val: 0x00000a21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41118,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41119,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4111a,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4111b,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4111c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4111d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4111e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4111f,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41120,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41121,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41122,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41123,
        val: 0x000001e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41124,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41125,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41126,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41127,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41128,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41129,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4112a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4112b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4112c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4112d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4112e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4112f,
        val: 0x00000121,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41130,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41131,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41132,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41133,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41134,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41135,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41136,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41137,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41138,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41139,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4113a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4113b,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4113c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4113d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4113e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4113f,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41140,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41141,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41142,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41143,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41144,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41145,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41146,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41147,
        val: 0x0000f921,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41148,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41149,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4114a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4114b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4114c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4114d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4114e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4114f,
        val: 0x00005921,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41150,
        val: 0x000005a5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41151,
        val: 0x0000a500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41152,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41153,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41154,
        val: 0x0000c040,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41155,
        val: 0x00004003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41156,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41157,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41158,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41159,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4115a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4115b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4115c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4115d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4115e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4115f,
        val: 0x00000a21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41160,
        val: 0x000000ef,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41161,
        val: 0x0000ef00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41162,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41163,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41164,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41165,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41166,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41167,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41168,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41169,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4116a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4116b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4116c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4116d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4116e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4116f,
        val: 0x0000ff21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41170,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41171,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41172,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41173,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41174,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41175,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41176,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41177,
        val: 0x0000ff21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41178,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41179,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4117a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4117b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4117c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4117d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4117e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4117f,
        val: 0x00000a21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41180,
        val: 0x000085d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41181,
        val: 0x0000d563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41182,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41183,
        val: 0x00000420,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41184,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41185,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41186,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41187,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41188,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41189,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4118a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4118b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4118c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4118d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4118e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4118f,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41190,
        val: 0x000085f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41191,
        val: 0x0000f563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41192,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41193,
        val: 0x00000820,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41194,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41195,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41196,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41197,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41198,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41199,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4119a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4119b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4119c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4119d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4119e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4119f,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a0,
        val: 0x000045d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a1,
        val: 0x0000d563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a3,
        val: 0x00000421,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a7,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411a9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411aa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ab,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ac,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ad,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ae,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411af,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b0,
        val: 0x000045f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b1,
        val: 0x0000f563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b3,
        val: 0x00000821,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b7,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411b9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ba,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411bb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411bc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411bd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411be,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411bf,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c0,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c1,
        val: 0x0000d562,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c3,
        val: 0x00000422,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c7,
        val: 0x00000022,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411c9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ca,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411cb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411cc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411cd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ce,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411cf,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d0,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d1,
        val: 0x0000f562,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d3,
        val: 0x00000822,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d7,
        val: 0x00000022,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411d9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411da,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411db,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411dc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411dd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411de,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411df,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e0,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e1,
        val: 0x0000d561,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e3,
        val: 0x00000423,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e7,
        val: 0x00000023,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411e9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ea,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411eb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ec,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ed,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ee,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ef,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f0,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f1,
        val: 0x0000f561,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f3,
        val: 0x00000823,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f7,
        val: 0x00000023,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411f9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411fa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411fb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411fc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411fd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411fe,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x411ff,
        val: 0x00001d01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41200,
        val: 0x00000213,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41201,
        val: 0x00001300,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41202,
        val: 0x000003c2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41203,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41204,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41205,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41206,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41207,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41208,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41209,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4120a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4120b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4120c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4120d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4120e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4120f,
        val: 0x0000ef20,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41210,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41211,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41212,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41213,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41214,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41215,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41216,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41217,
        val: 0x00005920,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41218,
        val: 0x00000217,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41219,
        val: 0x00001700,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4121a,
        val: 0x000003c2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4121b,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4121c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4121d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4121e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4121f,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41220,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41221,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41222,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41223,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41224,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41225,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41226,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x41227,
        val: 0x00000420,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42000,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42001,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42002,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42003,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42004,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42005,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42006,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42007,
        val: 0x00000a01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42008,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42009,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4200a,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4200b,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4200c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4200d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4200e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4200f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42010,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42011,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42012,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42013,
        val: 0x000001c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42014,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42015,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42016,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42017,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42018,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42019,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4201a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4201b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4201c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4201d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4201e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4201f,
        val: 0x00000101,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42020,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42021,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42022,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42023,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42024,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42025,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42026,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42027,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42028,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42029,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4202a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4202b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4202c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4202d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4202e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4202f,
        val: 0x0000f901,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42030,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42031,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42032,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42033,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42034,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42035,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42036,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42037,
        val: 0x00005901,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42038,
        val: 0x000005a5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42039,
        val: 0x00004000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4203a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4203b,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4203c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4203d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4203e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4203f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42040,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42041,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42042,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42043,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42044,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42045,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42046,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42047,
        val: 0x00000a01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42048,
        val: 0x000000ef,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42049,
        val: 0x0000ef00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4204a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4204b,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4204c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4204d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4204e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4204f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42050,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42051,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42052,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42053,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42054,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42055,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42056,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42057,
        val: 0x0000ff01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42058,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42059,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4205a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4205b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4205c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4205d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4205e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4205f,
        val: 0x0000ff01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42060,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42061,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42062,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42063,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42064,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42065,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42066,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42067,
        val: 0x00000a01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42068,
        val: 0x000085d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42069,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4206a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4206b,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4206c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4206d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4206e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4206f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42070,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42071,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42072,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42073,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42074,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42075,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42076,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42077,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42078,
        val: 0x000085f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42079,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4207a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4207b,
        val: 0x00000800,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4207c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4207d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4207e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4207f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42080,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42081,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42082,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42083,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42084,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42085,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42086,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42087,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42088,
        val: 0x000045d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42089,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4208a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4208b,
        val: 0x00000401,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4208c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4208d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4208e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4208f,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42090,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42091,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42092,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42093,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42094,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42095,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42096,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42097,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42098,
        val: 0x000045f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42099,
        val: 0x00000063,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4209a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4209b,
        val: 0x00000801,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4209c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4209d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4209e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4209f,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a8,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420a9,
        val: 0x00000062,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420aa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ab,
        val: 0x00000402,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ac,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ad,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ae,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420af,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b8,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420b9,
        val: 0x00000062,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ba,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420bb,
        val: 0x00000802,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420bc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420bd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420be,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420bf,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c8,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420c9,
        val: 0x00000061,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ca,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420cb,
        val: 0x00000403,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420cc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420cd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ce,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420cf,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d7,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d8,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420d9,
        val: 0x00000061,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420da,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420db,
        val: 0x00000803,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420dc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420dd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420de,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420df,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e7,
        val: 0x00001d01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e8,
        val: 0x00000213,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420e9,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ea,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420eb,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ec,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ed,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ee,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ef,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f0,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f1,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f2,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f3,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f7,
        val: 0x0000ef00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420f9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420fa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420fb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420fc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420fd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420fe,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x420ff,
        val: 0x00005900,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42100,
        val: 0x00000217,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42101,
        val: 0x00001700,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42102,
        val: 0x000003c2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42103,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42104,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42105,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42106,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42107,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42108,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42109,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4210a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4210b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4210c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4210d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4210e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4210f,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42110,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42111,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42112,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42113,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42114,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42115,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42116,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42117,
        val: 0x00000a21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42118,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42119,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4211a,
        val: 0x0000003f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4211b,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4211c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4211d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4211e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4211f,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42120,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42121,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42122,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42123,
        val: 0x000001e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42124,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42125,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42126,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42127,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42128,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42129,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4212a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4212b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4212c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4212d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4212e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4212f,
        val: 0x00000121,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42130,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42131,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42132,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42133,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42134,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42135,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42136,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42137,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42138,
        val: 0x00003fff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42139,
        val: 0x0000ff00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4213a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4213b,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4213c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4213d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4213e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4213f,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42140,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42141,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42142,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42143,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42144,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42145,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42146,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42147,
        val: 0x0000f921,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42148,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42149,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4214a,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4214b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4214c,
        val: 0x0000ffff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4214d,
        val: 0x0000ff03,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4214e,
        val: 0x000003ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4214f,
        val: 0x00005921,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42150,
        val: 0x000005a5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42151,
        val: 0x0000a500,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42152,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42153,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42154,
        val: 0x0000c040,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42155,
        val: 0x00004003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42156,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42157,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42158,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42159,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4215a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4215b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4215c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4215d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4215e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4215f,
        val: 0x00000a21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42160,
        val: 0x000000ef,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42161,
        val: 0x0000ef00,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42162,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42163,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42164,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42165,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42166,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42167,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42168,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42169,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4216a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4216b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4216c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4216d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4216e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4216f,
        val: 0x0000ff21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42170,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42171,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42172,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42173,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42174,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42175,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42176,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42177,
        val: 0x0000ff21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42178,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42179,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4217a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4217b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4217c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4217d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4217e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4217f,
        val: 0x00000a21,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42180,
        val: 0x000085d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42181,
        val: 0x0000d563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42182,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42183,
        val: 0x00000420,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42184,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42185,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42186,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42187,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42188,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42189,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4218a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4218b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4218c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4218d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4218e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4218f,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42190,
        val: 0x000085f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42191,
        val: 0x0000f563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42192,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42193,
        val: 0x00000820,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42194,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42195,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42196,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42197,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42198,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42199,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4219a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4219b,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4219c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4219d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4219e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4219f,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a0,
        val: 0x000045d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a1,
        val: 0x0000d563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a3,
        val: 0x00000421,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a7,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421a9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421aa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ab,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ac,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ad,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ae,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421af,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b0,
        val: 0x000045f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b1,
        val: 0x0000f563,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b3,
        val: 0x00000821,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b7,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421b9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ba,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421bb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421bc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421bd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421be,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421bf,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c0,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c1,
        val: 0x0000d562,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c3,
        val: 0x00000422,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c7,
        val: 0x00000022,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421c9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ca,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421cb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421cc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421cd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ce,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421cf,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d0,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d1,
        val: 0x0000f562,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d3,
        val: 0x00000822,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d7,
        val: 0x00000022,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421d9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421da,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421db,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421dc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421dd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421de,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421df,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e0,
        val: 0x0000c5d5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e1,
        val: 0x0000d561,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e3,
        val: 0x00000423,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e7,
        val: 0x00000023,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421e9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ea,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421eb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ec,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ed,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ee,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ef,
        val: 0x00001001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f0,
        val: 0x0000c5f5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f1,
        val: 0x0000f561,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f2,
        val: 0x000003c5,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f3,
        val: 0x00000823,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f4,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f5,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f6,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f7,
        val: 0x00000023,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f8,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421f9,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421fa,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421fb,
        val: 0x000002c1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421fc,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421fd,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421fe,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x421ff,
        val: 0x00001d01,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42200,
        val: 0x00000213,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42201,
        val: 0x00001300,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42202,
        val: 0x000003c2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42203,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42204,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42205,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42206,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42207,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42208,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42209,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4220a,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4220b,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4220c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4220d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4220e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4220f,
        val: 0x0000ef20,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42210,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42211,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42212,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42213,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42214,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42215,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42216,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42217,
        val: 0x00005920,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42218,
        val: 0x00000217,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42219,
        val: 0x00001700,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4221a,
        val: 0x000003c2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4221b,
        val: 0x00000021,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4221c,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4221d,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4221e,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x4221f,
        val: 0x00000020,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42220,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42221,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42222,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42223,
        val: 0x000002e1,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42224,
        val: 0x0000c000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42225,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42226,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x42227,
        val: 0x00000420,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90029,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002a,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002b,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002c,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002d,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002e,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9002f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90030,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90031,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90032,
        val: 0x0000000b,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90033,
        val: 0x00000480,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90034,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90035,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90036,
        val: 0x00000448,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90037,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90038,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90039,
        val: 0x00000478,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003a,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003c,
        val: 0x000000e8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003d,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003e,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9003f,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90040,
        val: 0x00000139,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90041,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90042,
        val: 0x000007c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90043,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90044,
        val: 0x00000107,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90045,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90046,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90047,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90048,
        val: 0x000000e0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90049,
        val: 0x00000139,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004a,
        val: 0x00000147,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004b,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004c,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004d,
        val: 0x0000014f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004e,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9004f,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90050,
        val: 0x00000007,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90051,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90052,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90053,
        val: 0x00000047,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90054,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90055,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90056,
        val: 0x0000004f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90057,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90058,
        val: 0x00000179,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90059,
        val: 0x00000100,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005a,
        val: 0x0000015c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005b,
        val: 0x00000139,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005c,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005d,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005e,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9005f,
        val: 0x00000011,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90060,
        val: 0x00000530,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90061,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90062,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90063,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90064,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90065,
        val: 0x0000014f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90066,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90067,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90068,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90069,
        val: 0x0000045a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006a,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006c,
        val: 0x00000530,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006d,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006e,
        val: 0x0000c100,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9006f,
        val: 0x0000015c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90070,
        val: 0x00000139,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90071,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90072,
        val: 0x0000065a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90073,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90074,
        val: 0x00000041,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90075,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90076,
        val: 0x00000179,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90077,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90078,
        val: 0x00000618,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90079,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007a,
        val: 0x000040c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007b,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007c,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007d,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007e,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9007f,
        val: 0x00000048,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90080,
        val: 0x00004040,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90081,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90082,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90083,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90084,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90085,
        val: 0x00000048,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90086,
        val: 0x00000040,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90087,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90088,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90089,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008a,
        val: 0x00000658,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008b,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008c,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008d,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008e,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9008f,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90090,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90091,
        val: 0x00000078,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90092,
        val: 0x00000549,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90093,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90094,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90095,
        val: 0x00000d49,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90096,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90097,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90098,
        val: 0x0000094c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90099,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009a,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009b,
        val: 0x0000094c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009c,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009d,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009e,
        val: 0x00000442,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9009f,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a0,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a1,
        val: 0x00000042,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a2,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a3,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a4,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a5,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a6,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a7,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a8,
        val: 0x000000e0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900a9,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900aa,
        val: 0x0000000a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ab,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ac,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ad,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ae,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900af,
        val: 0x00000149,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b0,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b1,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b2,
        val: 0x00000159,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b3,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b4,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b5,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b6,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b7,
        val: 0x000003c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b8,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900b9,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ba,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bb,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bc,
        val: 0x0000000c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bd,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900be,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900bf,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c0,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c1,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c2,
        val: 0x00000007,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c3,
        val: 0x000007c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c4,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c5,
        val: 0x0000003a,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c6,
        val: 0x000001e2,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c7,
        val: 0x00000009,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c8,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900c9,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ca,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cb,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cc,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cd,
        val: 0x0000016e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ce,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900cf,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d0,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d1,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d2,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d3,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d4,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d5,
        val: 0x00000978,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d6,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d7,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d8,
        val: 0x00000a78,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900d9,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900da,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900db,
        val: 0x00000980,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900dc,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900dd,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900de,
        val: 0x00000a80,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900df,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e0,
        val: 0x00000032,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e1,
        val: 0x00000952,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e2,
        val: 0x00000069,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e3,
        val: 0x00000032,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e4,
        val: 0x00000a52,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e5,
        val: 0x00000069,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e6,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e7,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e8,
        val: 0x00000068,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900e9,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ea,
        val: 0x00000370,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900eb,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ec,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ed,
        val: 0x00001400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ee,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ef,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f0,
        val: 0x000008e8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f1,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f2,
        val: 0x000002cd,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f3,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f4,
        val: 0x00000068,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f5,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f6,
        val: 0x000008e8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f7,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f8,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900f9,
        val: 0x000003c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900fa,
        val: 0x000001e9,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900fb,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900fc,
        val: 0x00000370,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900fd,
        val: 0x00000169,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900fe,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x900ff,
        val: 0x000000e8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90100,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90101,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90102,
        val: 0x00008140,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90103,
        val: 0x0000010c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90104,
        val: 0x00000010,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90105,
        val: 0x00008138,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90106,
        val: 0x0000010c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90107,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90108,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90109,
        val: 0x0000010e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9010a,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9010b,
        val: 0x00000448,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9010c,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9010d,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9010e,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9010f,
        val: 0x00000101,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90110,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90111,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90112,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90113,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90114,
        val: 0x00000448,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90115,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90116,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90117,
        val: 0x000007c0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90118,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90119,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9011a,
        val: 0x000000e8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9011b,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9011c,
        val: 0x00000007,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9011d,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9011e,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9011f,
        val: 0x00000047,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90120,
        val: 0x00000630,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90121,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90122,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90123,
        val: 0x00000618,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90124,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90125,
        val: 0x00000018,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90126,
        val: 0x000000e0,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90127,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90128,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90129,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9012a,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9012b,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9012c,
        val: 0x00008140,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9012d,
        val: 0x0000010c,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9012e,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9012f,
        val: 0x00000478,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90130,
        val: 0x00000109,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90131,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90132,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90133,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90134,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90135,
        val: 0x00000004,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90136,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90137,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90138,
        val: 0x000007c8,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90139,
        val: 0x00000101,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90006,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90007,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90008,
        val: 0x00000008,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90009,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000a,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000b,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xd00e7,
        val: 0x00000400,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20240,
        val: 0x00004300,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20242,
        val: 0x00008944,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20241,
        val: 0x00004300,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20243,
        val: 0x00008944,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90017,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9001f,
        val: 0x00000036,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90026,
        val: 0x0000004d,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000c,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000d,
        val: 0x00000173,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000e,
        val: 0x00000060,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x9000f,
        val: 0x00006110,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90010,
        val: 0x00002152,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90011,
        val: 0x0000dfbd,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90012,
        val: 0x00008060,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90013,
        val: 0x00006152,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20010,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20011,
        val: 0x00000003,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20281,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2003b,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20041,
        val: 0x0000131f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20131,
        val: 0x0000000e,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20151,
        val: 0x0000000f,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x90306,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2012a,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x2014a,
        val: 0x00000002,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20126,
        val: 0x000007ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20146,
        val: 0x000007ff,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20127,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20147,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20089,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x20088,
        val: 0x00000019,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0x200a6,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xc0080,
        val: 0x00000000,
        width: PhyWidth::W16,
    },
    PhyWrite {
        addr: 0xd0000,
        val: 0x00000001,
        width: PhyWidth::W16,
    },
];

// ── Post-training helpers ─────────────────────────────────────────────────────

/// Enable auto-refresh by clearing the TREFI_DISABLE bit (bit 15) in REFRESH_CTRL.
///
/// REFRESH_CTRL (offset 0x64) is written at init step 9 with 0x40B48200, which
/// has bit 15 set, suspending auto-refresh during training.  This call must be
/// made after MRS writes to activate refresh for normal DRAM operation.
///
fn enable_refresh() {
    let addr = 0x12C0_0000 + 0x64;
    let v = rd_raw(addr);
    wr_raw(addr, v & !0x8000);
}

fn qos_init() {
    const DRAMC_BASE: usize = 0x12C0_0000;
    const PORT_STRIDE: usize = 0x80;
    const PORT_CFG: usize = 0x00;
    const PORT_READ_QOS: usize = 0x08;
    const PORT_WRITE_QOS: usize = 0x10;
    const PORT_CFG_QOS: u32 = (1 << 2) | (1 << 3) | (8 << 4);

    let wr32 = |addr: usize, val: u32| wr_raw(addr, val);

    let port = |n: usize| DRAMC_BASE + 0x200 + n * PORT_STRIDE;

    // sdramc_qos_init().
    wr32(port(4) + PORT_WRITE_QOS, 10 << (3 * 4));
    wr32(port(4) + PORT_READ_QOS, 9 << (3 * 4));
    wr32(port(4) + PORT_CFG, PORT_CFG_QOS);

    wr32(
        port(2) + PORT_READ_QOS,
        (9 << (0 * 4)) | (9 << (1 * 4)) | (9 << (2 * 4)),
    );
    wr32(port(2) + PORT_CFG, PORT_CFG_QOS);

    wr32(port(3) + PORT_READ_QOS, 9 << (3 * 4));
    wr32(port(3) + PORT_CFG, PORT_CFG_QOS);

    wr32(port(1) + PORT_READ_QOS, (9 << (2 * 4)) | (9 << (3 * 4)));
    wr32(port(1) + PORT_CFG, PORT_CFG_QOS);
}

fn set_dramc_init_done_flag() {
    wr_raw(SCU0_VGA0_SCRATCH, rd_raw(SCU0_VGA0_SCRATCH) | DRAMC_INIT_DONE);
    wr_raw(SCU0_VGA1_SCRATCH, rd_raw(SCU0_VGA1_SCRATCH) | DRAMC_INIT_DONE);
}

/// Detect physical DRAM size via MAP1 aliasing sweep.
///
/// Sweeps MAP1 (MCU0 upper 1 GB window) from the 2GB mapping upward.
/// Writes a test pattern to 0xC0000000 (MAP1 window) and checks if it
/// aliases to 0x80000000 (DRAM base).  The first aliasing MAP1 setting
/// indicates the physical DRAM is smaller than that mapping → actual size
/// is the previous index.
///
/// After detection:
/// - Updates DRAMC mcfg bits[4:2] with the correct size index.
/// - Updates DRAMC actime5 bits[9:0] with the correct tRFC/2 value.
/// - MAP1 is left at the first wrap-triggering MAP1 value (harmless; BootMCU
///   only needs the MAP0 window at 0x80000000..0xBFFFFFFF for payload load).
///
/// Must be called while DRAMC is unlocked.
///
fn size_detect(is_ddr4: bool) {
    let mut pattern: u32 = 0xdead_beef;
    let mut sz = SDRAM_SIZE_2GB_IDX;

    while sz < SDRAM_SIZE_COUNT {
        let (map1, _, _) = DRAM_SIZE_TABLE[sz];
        let ctrl = rd_raw(SCU1_MCU0_CTRL);
        wr_raw(SCU1_MCU0_CTRL, (ctrl & !SCU1_MCU0_MAP1_MASK) | (map1 << SCU1_MCU0_MAP1_SHIFT));
        let _ = rd_raw(SCU1_MCU0_CTRL); // read-back fence
        wr_raw(DRAM_TEST_ADDR, pattern);
        // Delay to prevent store-buffer RAW hazard (≥10µs).
        delay_us(10);

        let readback = rd_raw(DRAM_START_ADDR);
        if readback == pattern {
            // Aliased → physical DRAM is smaller than `sz` mapping.
            break;
        }
        pattern >>= 4;
        sz += 1;
    }

    // sz is the first aliasing index; actual size index is one below.
    let sz = sz.saturating_sub(1);
    let (_, rfc_ddr4, rfc_ddr5) = DRAM_SIZE_TABLE[sz];
    let rfc_half = if is_ddr4 { rfc_ddr4 } else { rfc_ddr5 };

    // Update mcfg bits[4:2] with detected size index.
    let mcfg_addr = 0x12C0_0000 + 0x10;
    let mcfg = rd_raw(mcfg_addr);
    wr_raw(mcfg_addr, (mcfg & !(0x7 << 2)) | ((sz as u32) << 2));

    // Update actime5 bits[9:0] with tRFC/2 for detected size.
    let ac5 = dramc().AC_TIMING5().read();
    dramc().AC_TIMING5().write_value((ac5 & !0x3ff) | rfc_half);
}

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DramError {
    PhyInitTimeout,
    SelfRefTimeout,
    BistFail,
    NoPhyFirmware,
}

// ── BIST ──────────────────────────────────────────────────────────────────────

/// Run BIST over the first 4 MB of DRAM after training.

fn run_bist_zephyr(addr: u32, size: u32, cfg: u32) -> Result<(), DramError> {
    const BIST_CFG: usize = 0x12C0_0000 + 0xC0;
    wr_raw(BIST_CFG, 0);
    wr_raw(BIST_CFG, cfg);
    dramc().BIST_ADDR().write_value(addr >> 4);
    dramc().BIST_SIZE().write_value(size);
    dramc().BIST_PATTERN().write_value(0x89ab_cdef);
    wr_raw(BIST_CFG, cfg | 1);

    for _ in 0..POLL_TIMEOUT {
        if dramc().INTR_STS().read().BIST_DONE() {
            dramc().INTR_CLR().write(|w| w.set_BIST_DONE(true));
            let r = dramc().BIST_RESULT().read();
            return if r.FAIL() {
                Err(DramError::BistFail)
            } else {
                Ok(())
            };
        }
        core::hint::spin_loop();
    }
    Err(DramError::BistFail)
}

fn patch_dmem_message_block(is_ddr4: bool, train_2d: bool) {
    match (is_ddr4, train_2d) {
        (true, false) => apply_phy_writes(DMEM_MSG_DDR4_1D),
        (true, true) => apply_phy_writes(DMEM_MSG_DDR4_2D),
        (false, false) => apply_phy_writes(DMEM_MSG_DDR5_1D),
        (false, true) => {}
    }
}

fn phy_read_msg_half(addr_half: u32) -> u32 {
    let word_addr = (addr_half >> 1) << 1;
    let byte_addr = PHY_BASE + 2 * word_addr as usize;
    let word = rd_raw(byte_addr);
    if addr_half & 1 == 0 { word & 0xffff } else { word >> 16 }
}

fn run_training_pass(
    imem: &[u8],
    dmem: &[u8],
    is_ddr4: bool,
    train_2d: bool,
) -> Result<(), DramError> {
    phy_write(0xd0000, 0);
    if !train_2d {
        phy_write(0x20060, 2);
    }
    load_phy_blob(PHY_IMEM_BASE, imem);
    phy_write(0xd0000, 1);

    phy_write(0xd0031, 1);
    phy_write(0xc0033, 1);

    phy_write(0xd0000, 0);
    load_phy_blob(PHY_DMEM_BASE, dmem);
    patch_dmem_message_block(is_ddr4, train_2d);

    phy_write(0xd0099, 0x9);
    phy_write(0xd0099, 0x1);
    phy_write(0xd0099, 0x0);

    'outer: {
        for _ in 0..POLL_TIMEOUT {
            let shadow = phy_read(0xd0004);
            if shadow & 1 == 0 {
                let msg = phy_read(0xd0032) & 0xFF;
                phy_write(0xd0031, 0);
                for _ in 0..1_000_000u32 {
                    if phy_read(0xd0004) & 1 != 0 {
                        break;
                    }
                    core::hint::spin_loop();
                }
                phy_write(0xd0031, 1);
                if msg == 0x07 {
                    break 'outer;
                }
                if msg == 0xFF {
                    return Err(DramError::PhyInitTimeout);
                }
            }
            core::hint::spin_loop();
        }
        return Err(DramError::PhyInitTimeout);
    }

    phy_write(0xd0099, 0x1);
    phy_write(0x20089, 0x0);

    phy_write(0xd0000, 0);
    let result = if is_ddr4 {
        phy_read_msg_half(0x5800a)
    } else {
        phy_read_msg_half(0x58007)
    };
    phy_write(0xd0000, 1);
    if (is_ddr4 && result & 0xff != 0) || (!is_ddr4 && result & 0xff00 != 0) {
        return Err(DramError::PhyInitTimeout);
    }
    Ok(())
}

// ── Main init ─────────────────────────────────────────────────────────────────

/// Initialise DRAM.  On success SDRAM at 0x80000000 is usable.
///
/// `dram_size`: must match installed hardware.
///   AST2750-A1 DCSCM = 2 GB DDR4 → `DramSize::GB2`.
pub fn init() -> Result<(), DramError> {
    #[cfg(feature = "defmt")]
    defmt::info!("DRAM controller init start");

    // Step 1: detect DDR type from hardware strap.
    use crate::pac::sdrammc_ast2700_v1::DdrType;
    let ddr_type = scu::ddr_type_strap();
    let is_ddr4 = matches!(ddr_type, DdrType::DDR4);
    #[cfg(feature = "defmt")]
    defmt::info!("DRAM strap type: {:?}", ddr_type);

    // Step 2: parse the prebuilt table (ASTH on A1, FLSH container on A2).
    let table = PrebuiltTable::parse();
    let (imem_type, dmem_type, imem_2d_type, dmem_2d_type) = if is_ddr4 {
        (
            PrebuiltType::Ddr4TrainImem as u32,
            PrebuiltType::Ddr4TrainDmem as u32,
            PrebuiltType::Ddr4Train2dImem as u32,
            PrebuiltType::Ddr4Train2dDmem as u32,
        )
    } else {
        (
            PrebuiltType::Ddr5TrainImem as u32,
            PrebuiltType::Ddr5TrainDmem as u32,
            PrebuiltType::Ddr5TrainImem as u32,
            PrebuiltType::Ddr5TrainDmem as u32,
        )
    };
    let imem = table.spi_slice(imem_type).ok_or(DramError::NoPhyFirmware)?;
    let dmem = table.spi_slice(dmem_type).ok_or(DramError::NoPhyFirmware)?;
    let imem_2d = table
        .spi_slice(imem_2d_type)
        .ok_or(DramError::NoPhyFirmware)?;
    let dmem_2d = table
        .spi_slice(dmem_2d_type)
        .ok_or(DramError::NoPhyFirmware)?;

    // Step 3: MPLL re-lock (M/N/P already set by ROM; just cycle reset).
    mpll_relock();

    // Step 4: enable PHY clock via SCU0 CLK_STOP_CLR.
    wr_raw(SCU0_CLK_STOP_CLR, SCU0_CLK_PHY_BIT);

    // Vendor retry loop from reset through BIST.
    let mut last_err = DramError::BistFail;
    let mut init_ok = false;
    let mut saved = [0u32; 5];
    for retry in 0..TRAINING_ATTEMPTS {
        #[cfg(not(feature = "defmt"))]
        let _ = retry;
        #[cfg(feature = "defmt")]
        defmt::info!("DRAM training attempt {}", retry + 1);

        // Step 4b: WDT-based DRAMC soft-reset.
        for i in 0..5 {
            saved[i] = rd_raw(WDT0_SW_RST_SEL0 + i * 4);
        }
        wr_raw(WDT0_SW_RST_SEL0, WDT0_SW_RST_DRAMC_BIT);
        wr_raw(WDT0_SW_RST_SEL0 + 4, 0);
        wr_raw(WDT0_SW_RST_SEL0 + 8, 0);
        wr_raw(WDT0_SW_RST_SEL0 + 12, 0);
        wr_raw(WDT0_SW_RST_SEL0 + 16, 0);
        wr_raw(WDT0_SW_RST_KICK, WDT0_SW_RST_KICK_KEY);
        delay_us(1000);

        // Step 5: unlock SDRAMMC.
        unlock();

        // Step 6: mask all SDRAMMC interrupts during init.
        wr_raw(0x12C0_0000 + 0x0C, 0xFFFF_FFFF);

        // Step 7: configure MAIN_CONF.
        // Bit 5 always set (ASPEED-specific).
        // CRITICAL: use maximum size (5 = 8GB) during training, NOT the actual
        // installed size. The DRAMC address decoder must cover all possible
        // address bits during PHY training. size_detect() corrects mcfg
        // after training + BIST to the actual installed size.
        const DRAM_SIZE_MAX_TRAINING: u32 = 5; // 8GB — exercises all address bits
        let main_conf_val = if is_ddr4 {
            Ddr4Regs::MAIN_CONF_BASE | (DRAM_SIZE_MAX_TRAINING << 2)
        } else {
            Ddr5Regs::MAIN_CONF_BASE | (DRAM_SIZE_MAX_TRAINING << 2)
        };
        wr_raw(0x12C0_0000 + 0x10, main_conf_val);

        // Step 8: AC timing registers.
        let (ac1, ac2, ac3, ac4, ac5, ac6, ac7, dfi) = if is_ddr4 {
            (
                Ddr4Regs::ACTIME1,
                Ddr4Regs::ACTIME2,
                Ddr4Regs::ACTIME3,
                Ddr4Regs::ACTIME4,
                Ddr4Regs::ACTIME5,
                Ddr4Regs::ACTIME6,
                Ddr4Regs::ACTIME7,
                Ddr4Regs::DFI_TIMING,
            )
        } else {
            (
                Ddr5Regs::ACTIME1,
                Ddr5Regs::ACTIME2,
                Ddr5Regs::ACTIME3,
                Ddr5Regs::ACTIME4,
                Ddr5Regs::ACTIME5,
                Ddr5Regs::ACTIME6,
                Ddr5Regs::ACTIME7,
                Ddr5Regs::DFI_TIMING,
            )
        };
        dramc().AC_TIMING1().write_value(ac1);
        dramc().AC_TIMING2().write_value(ac2);
        dramc().AC_TIMING3().write_value(ac3);
        dramc().AC_TIMING4().write_value(ac4);
        dramc().AC_TIMING5().write_value(ac5);
        dramc().AC_TIMING6().write_value(ac6);
        dramc().AC_TIMING7().write_value(ac7);
        dramc().DFI_TIMING().write_value(dfi);
        dramc().DFI_CONF().write_value(0); // dcfg = 0
        dramc().DFI_MSG().write_value(0);

        // Step 9: refresh and ZQ configuration.
        let (refctl, zqctl) = if is_ddr4 {
            (Ddr4Regs::REFCTL, Ddr4Regs::ZQCTL)
        } else {
            (Ddr5Regs::REFCTL, Ddr5Regs::ZQCTL)
        };
        // REFRESH_CTRL at offset 0x64 (verified against sdramc_regs struct and PAC).
        // ZQCTL at offset 0x70 (annotated in struct).
        // NOTE: REFCTL is written here with bit 15 (TREFI_DISABLE) set in 0x40B48200.
        // Bit 15 is cleared by enable_refresh() after MRS to actually start auto-refresh.
        wr_raw(0x12C0_0000 + 0x64, refctl);
        if !is_ddr4 {
            wr_raw(0x12C0_0000 + 0x68, 0);
        }
        wr_raw(0x12C0_0000 + 0x70, zqctl);
        wr_raw(0x12C0_0000 + 0x88, 0);

        // Step 10: PHY power-on cold reset sequence (3 writes, 2µs between each).
        mctl_write(MCTL_PHY_RESET);
        delay_us(2);
        // Assert power-good while reset still active
        mctl_write(MCTL_PHY_RESET | MCTL_PHY_POWER_ON);
        delay_us(2);
        // De-assert reset — PHY APB now accessible
        mctl_write(MCTL_PHY_POWER_ON);
        delay_us(2);

        // Step 11 (C): PHY PUB pre-training configuration.
        apply_phy_writes(if is_ddr4 {
            PHY_CONFIG_DDR4
        } else {
            PHY_CONFIG_DDR5
        });

        // Steps D/E/F/G/H: Run vendor training passes.
        if let Err(e) = run_training_pass(imem, dmem, is_ddr4, false) {
            last_err = e;
            #[cfg(feature = "defmt")]
            defmt::warn!("DRAM 1D training failed: {:?}", e);
            continue; // retry
        }
        if is_ddr4 {
            if let Err(e) = run_training_pass(imem_2d, dmem_2d, is_ddr4, true) {
                last_err = e;
                #[cfg(feature = "defmt")]
                defmt::warn!("DRAM 2D training failed: {:?}", e);
                continue; // retry
            }
        }

        // Step I: Load PIE (PHY Init Engine) sequencer code.
        // Without PIE, DDRPHY_INIT_DONE never fires in Step J.
        apply_phy_writes(if is_ddr4 { PIE_DDR4 } else { PIE_DDR5 });

        // Step 16 (J): Enter mission mode.

        // Mask ALL DRAMC interrupts.
        wr_raw(0x12C0_0000 + 0x0C, 0xFFFF_FFFF);

        // Clear SDRAMMC DFI_CONF (dcfg = 0).
        dramc().DFI_CONF().write_value(0);

        // Trigger DRAMC PHY initialization (mission mode handshake).
        let ctrl = mctl_read();
        mctl_write(ctrl | MCTL_PHY_INIT_START);

        // Poll DRAMC INTR_STS.DDRPHY_INIT_DONE (raw status, ignoring mask).
        for _ in 0..POLL_TIMEOUT {
            if dramc().INTR_STS().read().DDRPHY_INIT_DONE() {
                break;
            }
            core::hint::spin_loop();
        }
        if !dramc().INTR_STS().read().DDRPHY_INIT_DONE() {
            last_err = DramError::PhyInitTimeout;
            #[cfg(feature = "defmt")]
            defmt::warn!("DRAM PHY init timed out");
            continue; // retry
        }
        // Clear interrupts.
        wr_raw(0x12C0_0000 + 0x08, 0x0000_FFFF);
        // Wait for all status bits to clear.
        for _ in 0..POLL_TIMEOUT {
            if dramc().INTR_STS().read().0 == 0 {
                break;
            }
            core::hint::spin_loop();
        }

        // Step 17: Exit self-refresh.
        let ctrl = mctl_read();
        wr_raw(0x12C0_0000 + 0x14, ctrl | (1 << 1)); // SELF_REF_TRIGGER
        for _ in 0..POLL_TIMEOUT {
            if dramc().INTR_STS().read().SELF_REF_DONE() {
                break;
            }
            core::hint::spin_loop();
        }
        if !dramc().INTR_STS().read().SELF_REF_DONE() {
            last_err = DramError::SelfRefTimeout;
            #[cfg(feature = "defmt")]
            defmt::warn!("DRAM self-refresh exit timed out");
            continue; // retry
        }
        dramc().INTR_CLR().write(|w| w.set_SELF_REF_DONE(true));

        // Step 18: Configure DDR4 mode registers.
        if is_ddr4 {
            configure_mrs_ddr4();
        }

        // Step 18a: Enable auto-refresh (clears TREFI_DISABLE bit 15 in REFRESH_CTRL).
        // Must happen after MRS writes, before lock.
        enable_refresh();

        // Step 18b: BIST — vendor runs CRC read/write-switch over first 64KB.
        // bistcfg = PMODE_CRC(3<<4) | BMODE_RW_SWITCH(3<<2) | ENABLE(1<<1) = 0x3E
        if run_bist_zephyr(0, 0x10000, 0x3E).is_err() {
            last_err = DramError::BistFail;
            #[cfg(feature = "defmt")]
            defmt::warn!("DRAM BIST failed");
            continue; // retry from WDT reset
        }

        // BIST passed — break out of retry loop.
        init_ok = true;
        break;
    } // end retry loop

    if !init_ok {
        #[cfg(feature = "defmt")]
        defmt::error!("DRAM controller init failed: {:?}", last_err);
        return Err(last_err);
    }

    // Step 18c: sdramc_postset() — restore WDT software reset masks.
    for i in 0..5 {
        wr_raw(WDT0_SW_RST_SEL0 + i * 4, saved[i]);
    }

    // Step 18d: sdramc_size_detect() — program physical DRAM size and tRFC.
    size_detect(is_ddr4);

    // Step 18e: sdramc_ecc_enable(). ECC is disabled for current DCSCM bringup,
    // matching no-op path when the DT property is absent.

    // Step 19: QoS setup before handoff.
    qos_init();

    // Keep interrupt mask at 0xFFFFFFFF.

    // Step 20: Lock SDRAMMC.
    lock();

    // sdramc_set_flag(DRAMC_INIT_DONE).
    set_dramc_init_done_flag();

    #[cfg(feature = "defmt")]
    defmt::info!("DRAM controller init complete");

    Ok(())
}

fn mr_send(mr_num: u32) {
    const MRWR: usize = 0x12C0_0000 + 0x04C;
    const MRCTL: usize = 0x12C0_0000 + 0x048;
    const INTR_STS: usize = 0x12C0_0000 + 0x004;
    const INTR_CLR: usize = 0x12C0_0000 + 0x008;
    const MR_DONE: u32 = 1 << 1;
    const CMD_WR: u32 = 1 << 1;
    const CMD_START: u32 = 1 << 0;

    let ctrl = (mr_num << 8) | CMD_WR;
    wr_raw(MRWR, 0);
    wr_raw(MRCTL, ctrl | CMD_START);
    while rd_raw(INTR_STS) & MR_DONE == 0 {
        core::hint::spin_loop();
    }
    wr_raw(INTR_CLR, MR_DONE);
}

/// Issue DDR4 mode register writes via DRAMC MR_CTRL mechanism.
///
/// Matches vendor sdramc_configure_mrs(): write all MR register pairs,
/// then send individual MR commands in JEDEC order MR3→6→5→4→2→1→0.
fn configure_mrs_ddr4() {
    dramc()
        .MR0_1()
        .write_value((Ddr4Regs::MR1 << 16) | Ddr4Regs::MR0);
    dramc()
        .MR2_3()
        .write_value((Ddr4Regs::MR3 << 16) | Ddr4Regs::MR2);
    dramc()
        .MR4_5()
        .write_value((Ddr4Regs::MR5 << 16) | Ddr4Regs::MR4);
    dramc().MR6_7().write_value(Ddr4Regs::MR6);

    mr_send(3);
    mr_send(6);
    mr_send(5);
    mr_send(4);
    mr_send(2);
    mr_send(1);
    mr_send(0);
}
