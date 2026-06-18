//! AST2700 CA35 (Cortex-A35 quad-core) release sequence.
//!
//! After boot firmware (ATF, OP-TEE, U-Boot) has been loaded into DRAM,
//! this module programs the reset vector base address registers and releases
//! the CA35 cores from reset.
//!
//! Uses `MmioBlock` for safe volatile register access (derive-mmio pattern)
//! and PAC accessors where generated register definitions exist.
//!
//! ## RVBAR address encoding
//!
//! The CA35 RVBAR register stores the reset vector as a physical address
//! right-shifted by 4 bits. The CA35 sees DRAM through an AXI remapper at
//! physical base `0x400000000` (SYS_DRAM_BASE). The BootMCU-side DRAM base
//! is `0x80000000` (ASPEED_DRAM_BASE).
//!
//! Conversion:
//! ```text
//! ep_arm = (load_addr - ASPEED_DRAM_BASE) | SYS_DRAM_BASE
//! rvbar  = ep_arm >> 4
//! ```
//!
//! ## SMP secondary cores
//!
//! Secondary CA35 cores spin on `SCU0_CPU_SMP_EP1..3`.  These must be
//! cleared before primary core release so secondaries don't jump to stale
//! addresses, then populated by ATF via PSCI.
//!
//! ## Sources
//!
//! `board_prepare_for_boot()`.  Values re-expressed as Rust — no C code copied.

use aspeed_mmio::MmioBlock;
use crate::pac;

// ── SCU0 register offsets ──────────────────────────────────────────────────────

const SCU0_BASE: usize = 0x12C0_2000;

const CA35_REL: usize = 0x10C;
const CA35_RVBAR0: usize = 0x110;
const CA35_RVBAR1: usize = 0x114;
const CA35_RVBAR2: usize = 0x118;
const CA35_RVBAR3: usize = 0x11C;

const CPU_SMP_EP0: usize = 0x780;
const CPU_SMP_EP1: usize = 0x788;
const CPU_SMP_EP2: usize = 0x790;
const CPU_SMP_EP3: usize = 0x798;

const MODRST1_CLR: usize = 0x204;
const MODRST2_CLR: usize = 0x224;
const CLKGATE_CLR: usize = 0x244;

const RST_EMMC: u32 = 1 << 17;
const RST_DP: u32 = 1 << 28;
const RST_DP_MCU: u32 = 1 << 29;
const RST2_VLINK: u32 = 1 << 12;
const CLKGATE1_DAC: u32 = 1 << 17;
const CLKGATE1_DP: u32 = 1 << 18;
const CLKGATE1_EMMC: u32 = 1 << 27;

// ── DRAM MPU ──────────────────────────────────────────────────────────────────

const DRAMC_BASE: usize = 0x12C0_0000;
const DRAMC_MPU_REGION0_OFF: usize = 0x600;
const DRAMC_MPU_PROTECT_LOCK_SET_OFF: usize = 0x090;

const DRAMC_MPU_EN: u32 = 1 << 0;
const DRAMC_MPU_CA35_SECURE_READWRITE: u32 = 0xF0;
const DRAMC_MPU_MASTER_SLIM: u32 = 1 << 19;
/// GFX (SOC display controller / CRT scanout) master in the MPU MASTER_1 word
/// (vendor MPU bitmap: {MPU_ID_GFX, ofst 0x14, BIT(7)}). The CRT scanout DMA
/// must be permitted to READ the framebuffer or the display shows black.
const DRAMC_MPU_MASTER_GFX: u32 = 1 << 7;
const DRAMC_MPU_STRIDE: usize = 0x40;

// ── Address translation ────────────────────────────────────────────────────────

const ASPEED_DRAM_BASE: u64 = 0x8000_0000;
const SYS_DRAM_BASE: u64 = 0x4_0000_0000;

fn rvbar_from_load_addr(load_addr: usize) -> u32 {
    let ep_arm = (load_addr as u64 - ASPEED_DRAM_BASE) | SYS_DRAM_BASE;
    (ep_arm >> 4) as u32
}

// ── RISC-V fence ──────────────────────────────────────────────────────────────

#[inline(always)]
fn fence_ow() {
    unsafe {
        core::arch::asm!("fence ow, ow", options(nostack, preserves_flags));
    }
}

// ── SCU0 block accessor ──────────────────────────────────────────────────────

fn scu0() -> MmioBlock {
    unsafe { MmioBlock::new(SCU0_BASE) }
}

fn dramc() -> MmioBlock {
    unsafe { MmioBlock::new(DRAMC_BASE) }
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Set the reset vector for all 4 CA35 cores to `atf_load_addr`.
///
/// Must be called after ATF is copied to DRAM, before `release()`.
pub fn set_rvbar(atf_load_addr: usize) {
    let rvbar = rvbar_from_load_addr(atf_load_addr);
    #[cfg(feature = "defmt")]
    defmt::info!("CA35 RVBAR set to 0x{:08x}", rvbar);
    let mut scu = scu0();
    fence_ow();
    scu.write32(CA35_RVBAR0, rvbar);
    fence_ow();
    scu.write32(CA35_RVBAR1, rvbar);
    fence_ow();
    scu.write32(CA35_RVBAR2, rvbar);
    fence_ow();
    scu.write32(CA35_RVBAR3, rvbar);
}

/// Set the CA35 BL33/non-secure entry point mailbox (SMP_EP0).
///
/// Called when U-Boot is loaded directly (no ATF). The CA35 primary core
/// jumps here after reset vector handling.
pub fn set_primary_ep(load_addr: usize) {
    let ep_arm = (load_addr as u64 - ASPEED_DRAM_BASE) | SYS_DRAM_BASE;
    let mut scu = scu0();
    fence_ow();
    scu.write64(CPU_SMP_EP0, ep_arm);
}

/// Set U-Boot SMP entry point (EP0) for vendor TF-A handoff.
pub fn set_uboot_ep(uboot_load_addr: usize) {
    set_primary_ep(uboot_load_addr);
}

/// Configure DRAM MPU region 0 to grant secure CA35 full read/write access
/// to the entire DRAM, while leaving the BootMCU SLI path usable for diagnostics.
///
/// Must be called before `release()`.  Without this the MPU blocks
/// CA35 instruction fetches from DRAM.
///
/// ## MPU address encoding
///
/// Registers use **DRAM-relative offsets** (NOT CA35 absolute addresses).
/// DRAM offset 0 = first byte of physical DRAM.  Register bits [31:8]
/// store address bits [35:12] (4 KB granularity); writing `offset >> 4`
/// places bits correctly.
///
/// CA35 leaves reset in secure EL3.  Program CA35 as
/// `S_READWRITE` (`ctrl = 0xF0`) before setting the region enable bit.  Using
/// `ctrl = 0x01` can leave secure CA35 fetches blocked before the reset vector
/// executes.
pub fn enable_dram_access(dram_size_bytes: u64) {
    let mut mc = dramc();
    let r = DRAMC_MPU_REGION0_OFF;

    mc.write32(r + 0x08, 0);
    mc.write32(r + 0x0C, ((dram_size_bytes - 1) >> 4) as u32);

    mc.write32(r + 0x00, 0x30);
    mc.write32(r + 0x10, 0xFFFF_FFFF);
    mc.write32(r + 0x14, 0xFFFF_FFFF & !DRAMC_MPU_MASTER_SLIM & !DRAMC_MPU_MASTER_GFX);
    mc.write32(r + 0x18, 0xFFFF_FFFF);
    mc.write32(r + 0x1C, 0xFFFF_FFFF & !DRAMC_MPU_MASTER_SLIM & !DRAMC_MPU_MASTER_GFX);
    mc.write32(r + 0x20, 0);
    mc.write32(r + 0x24, 0);
    mc.write32(r + 0x28, 0);
    mc.write32(r + 0x2C, 0);

    mc.write32(r + 0x00, DRAMC_MPU_CA35_SECURE_READWRITE | DRAMC_MPU_EN);
    mc.write32(DRAMC_MPU_PROTECT_LOCK_SET_OFF, 1);
}

/// Configure MPU region 0 like the vendor SPL ATF window on 1GB DCSCM.
pub fn enable_vendor_atf_access() {
    let mut mc = dramc();
    let r = DRAMC_MPU_REGION0_OFF;

    mc.write32(r + 0x08, 0x0300_0000);
    mc.write32(r + 0x0C, 0x0310_7FFF);
    mc.write32(r + 0x00, 0x30);
    mc.write32(r + 0x10, 0x00FF_00FF);
    mc.write32(r + 0x14, 0x0F07_FFFF);
    mc.write32(r + 0x18, 0x00FF_00FF);
    mc.write32(r + 0x1C, 0x0F07_FFFF);
    mc.write32(r + 0x20, 0);
    mc.write32(r + 0x24, 0);
    mc.write32(r + 0x28, 0);
    mc.write32(r + 0x2C, 0);
    mc.write32(r + 0x00, DRAMC_MPU_CA35_SECURE_READWRITE | DRAMC_MPU_EN);
    mc.write32(DRAMC_MPU_PROTECT_LOCK_SET_OFF, 1);
}

fn mpu_init_region(
    mc: &mut MmioBlock,
    idx: usize,
    start: u32,
    end: u32,
    ctrl: u32,
    wm0: u32,
    wm1: u32,
    rm0: u32,
    rm1: u32,
) {
    let r = DRAMC_MPU_REGION0_OFF + idx * DRAMC_MPU_STRIDE;
    mc.write32(r + 0x08, start);
    mc.write32(r + 0x0C, end);
    mc.write32(r + 0x00, ctrl & !DRAMC_MPU_EN);
    mc.write32(r + 0x10, wm0);
    mc.write32(r + 0x14, wm1);
    mc.write32(r + 0x18, rm0);
    mc.write32(r + 0x1C, rm1);
    mc.write32(r + 0x20, 0);
    mc.write32(r + 0x24, 0);
    mc.write32(r + 0x28, 0);
    mc.write32(r + 0x2C, 0);
    mc.write32(r + 0x00, ctrl | DRAMC_MPU_EN);
    mc.write32(DRAMC_MPU_PROTECT_LOCK_SET_OFF, 1 << idx);
}

/// Configure all 6 MPU regions matching vendor Ibex DTS for AST2700 DCSCM.
///
/// Region layout (DRAM-relative addresses, shifted >>4 in registers):
/// - 0: ca35_s  — ATF+OP-TEE (CA35 S_RW, SLIM RW)
/// - 1: ssp     — SSP code/data (SSP_INST RO, SSP_DATA RW, SLIM RW)
/// - 2: tsp     — TSP code/data (TSP_INST RO, TSP_DATA RW, SLIM RW)
/// - 3: share_1 — IPC SSP<->OP-TEE (SSP_DATA RO, CA35 S_RW)
/// - 4: share_2 — IPC SSP<->Linux (SSP_DATA RO, CA35 NS_RW)
/// - 5: vb      — Video BIOS (E2M RO, E2M1 RO)
pub fn enable_vendor_mpu_regions() {
    #[cfg(feature = "defmt")]
    defmt::info!("CA35 MPU region setup start");

    let mut mc = dramc();

    mpu_init_region(&mut mc, 0, 0x0300_0000, 0x0310_7FFF, 0xF0,
        0xFFFF_FFFF, 0xFFF7_FFFF, 0xFFFF_FFFF, 0xFFF7_FFFF);
    mpu_init_region(&mut mc, 1, 0x02C0_0000, 0x02CF_FFFF, 0x30,
        0xFFFF_FFFF, 0xFFF7_BFFF, 0xFFFF_FFFF, 0xFFF7_9FFF);
    mpu_init_region(&mut mc, 2, 0x02E0_0000, 0x02FF_FFFF, 0x30,
        0xFFFF_FFFF, 0xFFF7_FFDF, 0xFFEF_FFFF, 0xFFF7_FFDF);
    mpu_init_region(&mut mc, 3, 0x0310_8000, 0x0314_7FFF, 0xF0,
        0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_BFFF);
    mpu_init_region(&mut mc, 4, 0x0314_8000, 0x0318_7FFF, 0x00,
        0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_BFFF);
    mpu_init_region(&mut mc, 5, 0x031B_B000, 0x031B_CFFF, 0x30,
        0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFEF, 0xFFFF_FFBF);

    #[cfg(feature = "defmt")]
    defmt::info!("CA35 MPU region setup complete");
}

const PLDA1_BASE: usize = 0x12C1_5000;
const PLDA2_BASE: usize = 0x12C1_5800;
const PLDA3_BASE: usize = 0x14C1_C000;
const PLDA_MSI_CAP: usize = 0x10;
const PLDA_PRESET0: usize = 0xB0;
const PLDA_PRESET1: usize = 0xB4;

pub fn init_pci_e2m() {
    #[cfg(feature = "defmt")]
    defmt::info!("PCI E2M init start");

    let scu0 = pac::SCU0;
    let rst2 = scu0.RST_CTRL2().read();
    if !rst2.E2M0() && !rst2.E2M1() {
        #[cfg(feature = "defmt")]
        defmt::warn!("PCI E2M init skipped: E2M reset already clear");
        return;
    }

    let mut plda2 = unsafe { MmioBlock::new(PLDA2_BASE) };
    scu0.PCI1_MISC70().write_value(0x0101_0101);
    plda2.write32(PLDA_PRESET0, 0x1260_0000);
    plda2.write32(PLDA_PRESET1, 0x0001_2600);
    plda2.modify32(PLDA_MSI_CAP, |v| (v & !0xFF) | 0x01);
    scu0.CLKGATE1_CLR().write(|w| w.set_E2M1(true));
    for _ in 0..2_000_000 {
        core::hint::spin_loop();
    }
    scu0.RST_CLR2().write(|w| w.set_E2M1(true));

    let mut plda1 = unsafe { MmioBlock::new(PLDA1_BASE) };
    scu0.PCI0_MISC70().write_value(0x0101_0101);
    plda1.write32(PLDA_PRESET0, 0x1260_0000);
    plda1.write32(PLDA_PRESET1, 0x0001_2600);
    plda1.modify32(PLDA_MSI_CAP, |v| (v & !0xFF) | 0x01);
    scu0.CLKGATE1_CLR().write(|w| w.set_E2M0(true));
    for _ in 0..2_000_000 {
        core::hint::spin_loop();
    }
    scu0.RST_CLR2().write(|w| w.set_E2M0(true));

    let mut plda3 = unsafe { MmioBlock::new(PLDA3_BASE) };
    plda3.modify32(PLDA_MSI_CAP, |v| (v & !0xFF) | 0x01);

    #[cfg(feature = "defmt")]
    defmt::info!("PCI E2M init complete");
}

const UFS_BASE: usize = 0x12C0_8000;
const UFS_PATH_AXI: usize = 0xE4;

pub fn init_ufs_axi_path() {
    #[cfg(feature = "defmt")]
    defmt::info!("UFS AXI path init start");

    let scu0 = pac::SCU0;
    let ufs = pac::UFS;
    let mut ufs_mmio = unsafe { MmioBlock::new(UFS_BASE) };

    scu0.CLKGATE1_CLR().write(|w| w.set_UFS(true));
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
    scu0.RST_CTRL1().write(|w| w.set_UFS(true));
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
    scu0.RST_CLR1().write(|w| w.set_UFS(true));
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
    ufs.HCLKDIV().write_value(pac::ufs_ast2700_v1::HCLKDIV(0x05FF_F00));
    ufs.AHIT().write_value(pac::ufs_ast2700_v1::AHIT(0x000D_0707));
    ufs.HCE().write_value(pac::ufs_ast2700_v1::HCE(0));
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
    ufs.HCE().write(|w| {
        w.set_ENABLE(true);
    });
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
    ufs_mmio.write32(UFS_PATH_AXI, 0);

    #[cfg(feature = "defmt")]
    defmt::info!("UFS AXI path init complete");
}

/// Release additional fabric/display/storage gates that vendor SPL leaves active.
pub fn init_vendor_runtime_fabric() {
    #[cfg(feature = "defmt")]
    defmt::info!("runtime fabric init start");

    let mut scu = scu0();

    scu.write32(CLKGATE_CLR, CLKGATE1_DAC | CLKGATE1_DP | CLKGATE1_EMMC);
    for _ in 0..2_000_000 {
        core::hint::spin_loop();
    }
    scu.write32(MODRST1_CLR, RST_EMMC | RST_DP | RST_DP_MCU);
    scu.write32(MODRST2_CLR, RST2_VLINK);

    #[cfg(feature = "defmt")]
    defmt::info!("runtime fabric init complete");
}

pub fn release_vlink_reset() {
    let mut scu = scu0();
    scu.write32(MODRST2_CLR, RST2_VLINK);
    #[cfg(feature = "defmt")]
    defmt::info!("VLINK reset released");
}

/// Release CA35 cores from reset.
///
/// Preconditions:
/// - ATF/U-Boot loaded into DRAM
/// - RVBAR programmed via `set_rvbar()` or SMP EP via `set_uboot_ep()`
/// - DRAM MPU configured via `enable_dram_access()`
/// - SLI and DRAM fully initialised
///
/// Sequence:
/// 1. Switch UFS path to AXI (required before CA35 can access DRAM)
/// 2. Clear secondary SMP entry points (EP1..3 = 0)
/// 3. Write 1 to SCU0_CA35_REL
pub fn release() {
    #[cfg(feature = "defmt")]
    defmt::info!("CA35 release start");

    let mut scu = scu0();
    let mut ufs_mmio = unsafe { MmioBlock::new(UFS_BASE) };
    ufs_mmio.write32(UFS_PATH_AXI, 1);
    scu.write64(CPU_SMP_EP1, 0);
    scu.write64(CPU_SMP_EP2, 0);
    scu.write64(CPU_SMP_EP3, 0);
    scu.write32(CA35_REL, 1);

    #[cfg(feature = "defmt")]
    defmt::info!("CA35 release complete");
}

/// Full CA35 bringup: configure MPU, set RVBAR, release.
///
/// `dram_size_bytes`: total DRAM size (e.g. `2 * 1024 * 1024 * 1024` for 2 GB).
pub fn start(atf_load_addr: usize, dram_size_bytes: u64) {
    set_rvbar(atf_load_addr);
    enable_dram_access(dram_size_bytes);
    release();
}

/// Full vendor TF-A bringup: configure BL31 reset vector and BL33 entry.
pub fn start_atf_bl33(atf_load_addr: usize, bl33_load_addr: usize, dram_size_bytes: u64) {
    set_rvbar(atf_load_addr);
    set_primary_ep(bl33_load_addr);
    enable_dram_access(dram_size_bytes);
    release();
}
