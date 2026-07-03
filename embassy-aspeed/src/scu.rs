//! AST2700 System Control Unit helpers for the BootMCU.
//!
//! Source of truth: aspeed-data/data/registers/scu0_ast2700_v1.yaml
//!                  aspeed-data/data/registers/scu1_ast2700_v1.yaml
//!
//! # SCU topology
//!
//! | Unit | Base | Die | Covers |
//! |------|------|-----|--------|
//! | SCU0 | 0x12C02000 | CPU-die | Silicon rev, CA35/SSP/TSP, HPLL/MPLL, DDR PHY reset |
//! | SCU1 | 0x14C02000 | IO-die | Silicon rev, hardware straps, Caliptra ctrl, IO-die PLLs |
//!
//! The BootMCU accesses both SCUs over the SLI (system link interface).
//!
//! # Silicon revision
//!
//! Read SCU0.SIL_REV or SCU1.SIL_REV:
//!   [31:24] = BMC generation (0x06 = AST2700 G7)
//!   [23:16] = HW revision    (0x00 = A0,  0x01 = A1,  0x02 = A2)
//!   [15:8]  = EFUSE device   (0x00 = AST2750, 0x01 = AST2700, 0x02 = AST2720)

use crate::pac;
use crate::pac::sdrammc_ast2700_v1::DdrType;

// ── Silicon revision ──────────────────────────────────────────────────────────

/// AST2700 hardware revision.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HwRev {
    /// A0 silicon (first production stepping).
    A0,
    /// A1 silicon (current production stepping, AST2750).
    A1,
    /// A2 silicon (Caliptra FLSH boot container + ATMN SoC auth-manifest).
    A2,
    /// Unknown revision.
    Unknown(u8),
}

/// AST2700 device variant.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DeviceId {
    /// AST2750 (full AST2700 with security extensions).
    Ast2750,
    /// AST2700 (base variant).
    Ast2700,
    /// AST2720 (reduced feature variant).
    Ast2720,
    /// Unknown device ID.
    Unknown(u8),
}

/// Read the silicon revision from SCU0.
///
/// Returns `(device_id, hw_rev)`.
pub fn silicon_rev() -> (DeviceId, HwRev) {
    let rev = pac::SCU0.SIL_REV().read();
    let dev = match rev.DEVICE_ID() {
        0x00 => DeviceId::Ast2750,
        0x01 => DeviceId::Ast2700,
        0x02 => DeviceId::Ast2720,
        n => DeviceId::Unknown(n),
    };
    let hw = match rev.HW_REV() {
        0x00 => HwRev::A0,
        0x01 => HwRev::A1,
        0x02 => HwRev::A2,
        n => HwRev::Unknown(n),
    };
    (dev, hw)
}

// ── DDR type detection ────────────────────────────────────────────────────────

/// Read the DDR type from the hardware strap.
///
/// Strap pin: GPIOG0 (AST2700 datasheet §29.2).
/// 0 = DDR5, 1 = DDR4 (PAC renders 1-bit enum field as bool).
///
/// Returns a `DdrType` for use with SDRAMMC.MAIN_CONF.DDR_TYPE.
pub fn ddr_type_strap() -> DdrType {
    // HWSTRAP1.DDR_TYPE: bool — false=DDR5, true=DDR4
    // (scu1_ast2700_v1.yaml DdrTypeStrap: DDR5=0, DDR4=1)
    if pac::SCU1.HWSTRAP1().read().DDR_TYPE() {
        DdrType::DDR4
    } else {
        DdrType::DDR5
    }
}

// ── HPLL / MPLL status ───────────────────────────────────────────────────────

/// Return true if the CPU-die HPLL has acquired lock.
///
pub fn hpll_locked() -> bool {
    pac::SCU0.HPLL_EXT().read().LOCK()
}

/// Return true if the MPLL (memory PLL) has acquired lock.
///
pub fn mpll_locked() -> bool {
    pac::SCU0.MPLL_EXT().read().LOCK()
}

// ── DDR PHY reset control ─────────────────────────────────────────────────────

/// Assert the DDR PHY reset via SCU0.RST_CTRL1.DDRPHY.
///
/// Write-to-set semantics: RST_CTRL1 asserts reset, RST_CLR1 de-asserts.
pub fn ddrphy_reset_assert() {
    pac::SCU0.RST_CTRL1().write(|w| w.set_DDRPHY(true));
}

/// De-assert the DDR PHY reset via SCU0.RST_CLR1.DDRPHY.
pub fn ddrphy_reset_deassert() {
    pac::SCU0.RST_CLR1().write(|w| w.set_DDRPHY(true));
}

// ── SCU register access policy ───────────────────────────────────────────────

const SCU0_BASE: usize = 0x12C0_2000;
const SCU1_BASE: usize = 0x14C0_2000;

const SYS_POLICY_RESET_LOCK: u32 = (0b111 << 13) | (0b111 << 5);
const SYS_POLICY_CLK0_LOCK: u32 = 0b111 << 21;
const SYS_POLICY_CLK1_LOCK: u32 = (0b111 << 21) | (0b111 << 29);
const SYS_POLICY_CLK0_SEL1_LOCK: u32 = 0x0000_FFFF;
const SYS_POLICY_CLK0_SEL2_LOCK: u32 = 0x0000_1FFF;
const SYS_POLICY_CLK0_SEL3_LOCK: u32 = 0x0000_0003;
const SYS_POLICY_CLK1_SEL1_LOCK: u32 = (0b11 << 13) | (1 << 18) | (1 << 21) | (1 << 25) | (1 << 29);
const SYS_POLICY_CLK1_SEL2_LOCK: u32 =
    (1 << 0) | (1 << 3) | (1 << 8) | (1 << 12) | (0b11_1111 << 15) | (1 << 23);

/// Processor group used by AST2700 per-register SCU access control.
#[derive(Clone, Copy)]
pub enum PolicyGroup {
    /// BootMCU plus secure PSP.
    SecurePsp,
    /// BootMCU plus SSP.
    Ssp,
    /// BootMCU plus non-secure PSP.
    Psp,
    /// BootMCU plus TSP.
    Tsp,
    /// BootMCU plus PSP and SSP.
    PspSsp,
    /// BootMCU plus SSP and TSP.
    SspTsp,
    /// BootMCU only.
    BootMcu,
}

impl PolicyGroup {
    const fn mask(self) -> u32 {
        match self {
            Self::SecurePsp => 0b001,
            Self::Ssp => 0b010,
            Self::Psp => 0b011,
            Self::Tsp => 0b100,
            Self::PspSsp => 0b101,
            Self::SspTsp => 0b110,
            Self::BootMcu => 0b111,
        }
    }
}

/// SCU register access-control list for one reset or clock controller.
pub struct PolicyList<'a> {
    /// Register bit IDs that BootMCU plus secure PSP may access.
    pub secure_psp: &'a [u32],
    /// Register bit IDs that BootMCU plus SSP may access.
    pub ssp: &'a [u32],
    /// Register bit IDs that BootMCU plus non-secure PSP may access.
    pub psp: &'a [u32],
    /// Register bit IDs that BootMCU plus TSP may access.
    pub tsp: &'a [u32],
    /// Register bit IDs that BootMCU plus PSP and SSP may access.
    pub psp_ssp: &'a [u32],
    /// Register bit IDs that BootMCU plus SSP and TSP may access.
    pub ssp_tsp: &'a [u32],
    /// Register bit IDs that only BootMCU may access.
    pub bootmcu: &'a [u32],
}

impl PolicyList<'_> {
    const EMPTY: Self = Self {
        secure_psp: &[],
        ssp: &[],
        psp: &[],
        tsp: &[],
        psp_ssp: &[],
        ssp_tsp: &[],
        bootmcu: &[],
    };
}

#[derive(Clone, Copy)]
struct PolicyRegs {
    bank: [[u32; 3]; 2],
}

impl PolicyRegs {
    const fn new() -> Self {
        Self { bank: [[0; 3]; 2] }
    }

    fn add_group(&mut self, group: PolicyGroup, ids: &[u32], max_id: u32) {
        let mask = group.mask();
        for &id in ids {
            if id <= max_id {
                let bank = (id / 32) as usize;
                let bit = 1u32 << (id % 32);
                if mask & 0b001 != 0 {
                    self.bank[bank][0] |= bit;
                }
                if mask & 0b010 != 0 {
                    self.bank[bank][1] |= bit;
                }
                if mask & 0b100 != 0 {
                    self.bank[bank][2] |= bit;
                }
            }
        }
    }

    fn from_list(list: &PolicyList<'_>, max_id: u32) -> Self {
        let mut regs = Self::new();
        regs.add_group(PolicyGroup::SecurePsp, list.secure_psp, max_id);
        regs.add_group(PolicyGroup::Ssp, list.ssp, max_id);
        regs.add_group(PolicyGroup::Psp, list.psp, max_id);
        regs.add_group(PolicyGroup::Tsp, list.tsp, max_id);
        regs.add_group(PolicyGroup::PspSsp, list.psp_ssp, max_id);
        regs.add_group(PolicyGroup::SspTsp, list.ssp_tsp, max_id);
        regs.add_group(PolicyGroup::BootMcu, list.bootmcu, max_id);
        regs
    }
}

#[inline(always)]
fn policy_wr32(addr: usize, val: u32) {
    let mut regs = unsafe { aspeed_mmio::MmioBlock::new(addr) };
    regs.write32(0, val)
}

fn apply_two_bank_policy(base: usize, regs: PolicyRegs) {
    policy_wr32(base + 0x14, regs.bank[0][0]);
    policy_wr32(base + 0x18, regs.bank[0][1]);
    policy_wr32(base + 0x1C, regs.bank[0][2]);
    policy_wr32(base + 0x34, regs.bank[1][0]);
    policy_wr32(base + 0x38, regs.bank[1][1]);
    policy_wr32(base + 0x3C, regs.bank[1][2]);
}

fn apply_one_bank_policy(base: usize, regs: PolicyRegs) {
    policy_wr32(base + 0x14, regs.bank[0][0]);
    policy_wr32(base + 0x18, regs.bank[0][1]);
    policy_wr32(base + 0x1C, regs.bank[0][2]);
}

/// Program AST2700 SCU reset/clock per-register access policy.
pub fn apply_register_policy(
    soc0_reset: &PolicyList<'_>,
    soc1_reset: &PolicyList<'_>,
    soc0_clock: &PolicyList<'_>,
    soc1_clock: &PolicyList<'_>,
) {
    let soc0_reset_base = SCU0_BASE + 0x200;
    let soc1_reset_base = SCU1_BASE + 0x200;
    let soc0_clock_base = SCU0_BASE + 0x240;
    let soc1_clock_base = SCU1_BASE + 0x240;

    apply_two_bank_policy(soc0_reset_base, PolicyRegs::from_list(soc0_reset, 44));
    apply_two_bank_policy(soc1_reset_base, PolicyRegs::from_list(soc1_reset, 57));
    apply_one_bank_policy(soc0_clock_base, PolicyRegs::from_list(soc0_clock, 28));
    apply_two_bank_policy(soc1_clock_base, PolicyRegs::from_list(soc1_clock, 51));

    policy_wr32(soc0_reset_base + 0xC10, SYS_POLICY_RESET_LOCK);
    policy_wr32(soc1_reset_base + 0xC10, SYS_POLICY_RESET_LOCK);
    policy_wr32(soc0_clock_base + 0xBD0, SYS_POLICY_CLK0_LOCK);
    policy_wr32(soc1_clock_base + 0xBD0, SYS_POLICY_CLK1_LOCK);

    policy_wr32(soc0_clock_base + 0x50, SYS_POLICY_CLK0_SEL1_LOCK);
    policy_wr32(soc0_clock_base + 0x54, SYS_POLICY_CLK0_SEL2_LOCK);
    policy_wr32(soc0_clock_base + 0x58, SYS_POLICY_CLK0_SEL3_LOCK);
    policy_wr32(soc1_clock_base + 0x60, SYS_POLICY_CLK1_SEL1_LOCK);
    policy_wr32(soc1_clock_base + 0x70, SYS_POLICY_CLK1_SEL2_LOCK);
}

/// Apply the same empty policy lists used by the current AST2700 Ibex DTS.
pub fn apply_ibex_default_register_policy() {
    apply_register_policy(
        &PolicyList::EMPTY,
        &PolicyList::EMPTY,
        &PolicyList::EMPTY,
        &PolicyList::EMPTY,
    );
}
