//! AST2700 SLI (Serial Link Interface) — CPU die ↔ IO die interconnect.
//!
//! Three independent SLI channels per die:
//!
//! | Channel | Offset | Protocol           | Lanes |
//! |---------|--------|--------------------|-------|
//! | SLIM    | +0x000 | MBUS (memory bus)  | 4     |
//! | SLIH    | +0x200 | AHB                | 2     |
//! | SLIV    | +0x400 | Video              | 2     |
//!
//! ## Init sequence
//!
//! Two phases must run in order:
//!
//! 1. `init_f()` — "first" init, runs on BootMCU (IO-die context).
//!    Calibrates AHB and MBUS downstream links.
//!
//! 2. `init_r()` — "remote" init, runs after `init_f` signals completion.
//!    Waits for IO-die readiness, restores AHB timeouts, calibrates Video SLI.
//!
//! On AST2750-A1 (`SLI_TARGET_PHYCLK` = 1 GHz) both phases are required.
//!
//! ## Sources
//!
//! Values re-expressed as Rust — no C code copied.

// ── Base addresses ─────────────────────────────────────────────────────────────

/// CPU-die SLI register base (SLI0).
const SLI0_BASE: usize = 0x12C1_7000;
/// IO-die SLI register base (SLI1).
const SLI1_BASE: usize = 0x14C1_E000;

/// SLIM (MBUS) sub-block offset.
const SLIM_OFF: usize = 0x000;
/// SLIH (AHB) sub-block offset.
const SLIH_OFF: usize = 0x200;
/// SLIV (Video) sub-block offset.
const SLIV_OFF: usize = 0x400;

// AHB controller timeout registers — disabled during calibration.
const AHBC0_BASE: usize = 0x1200_0000;
const AHBC1_BASE: usize = 0x140B_0000;
const AHBC_TIMEOUT_OFFSETS: &[usize] = &[
    0x034, 0x074, 0x0b4, 0x0f4, // 4 entries for AHBC0
];
const AHBC1_TIMEOUT_OFFSETS: &[usize] = &[0x034, 0x074, 0x0b4, 0x0f4, 0x134, 0x174, 0x1b4, 0x1f4];
const AHBC_MAX_TIMEOUT: u32 = 0x1ff;

/// IO INTC reset interrupt status register.
const IO_INTC_INTR_STATUS: usize = 0x14C1_8014;

// ── SCU scratch registers ──────────────────────────────────────────────────────
//
// SCU1 = IO die, base 0x14C02000
// SCU0 = CPU die, base 0x12C02000
//
//   scu1->scratch[28] = 0x14C021F0  MBUS DS calibration window (lanes 0+1)
//   scu1->scratch[29] = 0x14C021F4  MBUS DS calibration window (lanes 2+3)
//   scu1->scratch[30] = 0x14C021F8  AHB + Video US windows
//   scu1->scratch[31] = 0x14C021FC  flags: SLI0_READY, SLI_SKIP_CALI
//
//   scu0->cpu_scratch[28] = 0x12C027F0  MBUS US calibration window (lanes 0+1)
//   scu0->cpu_scratch[29] = 0x12C027F4  MBUS US calibration window (lanes 2+3)
//   scu0->cpu_scratch[30] = 0x12C027F8  Video DS window
//   scu0->cpu_scratch[31] = 0x12C027FC  SLI1_READY flag

const SCU1_BASE: usize = 0x14C0_2000;
const SCU0_BASE: usize = 0x12C0_2000;

const SCU1_SCRATCH28: usize = SCU1_BASE + 0x1F0;
const SCU1_SCRATCH30: usize = SCU1_BASE + 0x1F8;
const SCU1_SCRATCH31: usize = SCU1_BASE + 0x1FC;

const SCU0_CPU_SCRATCH28: usize = SCU0_BASE + 0x7F0;
const SCU0_CPU_SCRATCH30: usize = SCU0_BASE + 0x7F8;
const SCU0_CPU_SCRATCH31: usize = SCU0_BASE + 0x7FC;

/// Bit 0 of scu1->scratch[31]: IO die has finished SLI0 calibration.
const SCU1_SCRATCH31_SLI0_READY: u32 = 1 << 0;
/// Bit 1 of scu1->scratch[31]: skip calibration (already done).
const SCU1_SCRATCH31_SLI_SKIP_CALI: u32 = 1 << 1;
/// Bit 0 of scu0->cpu_scratch[31]: CPU die has acknowledged SLI1.
const SCU0_SCRATCH31_SLI1_READY: u32 = 1 << 0;

// ── Register offsets (within each SLIM/SLIH/SLIV block) ───────────────────────

const SLI_CTRL_I: usize = 0x00;
const SLI_CTRL_II: usize = 0x04;
const SLI_CTRL_III: usize = 0x08;
const SLI_CTRL_IV: usize = 0x0C;
const SLI_INTR_STATUS: usize = 0x14;
const SLIM_MARB_FUNC_I: usize = 0x60;

// ── SLI_CTRL_I bits ────────────────────────────────────────────────────────────

const SLI_AUTO_CLR_OFF_DAT: u32 = 1 << 23;
const SLI_AUTO_CLR_OFF_CLK: u32 = 1 << 22;
const SLI_NO_RST_TXCLK_CHG: u32 = 1 << 17;
const SLIV_RAW_MODE: u32 = 1 << 15;
const SLI_TX_MODE: u32 = 1 << 14;
const SLI_RX_PHY_LAH_SEL_NEG: u32 = 1 << 12;
const SLI_AUTO_SEND_TRN_OFF: u32 = 1 << 8;
const SLI_CLEAR_BUS: u32 = 1 << 6;
#[allow(dead_code)]
const SLI_TRANS_EN: u32 = 1 << 5;
const SLI_CLEAR_RX: u32 = 1 << 2;
#[allow(dead_code)]
const SLI_CLEAR_TX: u32 = 1 << 1;
const SLI_RESET_TRIGGER: u32 = 1 << 0;

// ── SLI_CTRL_II bits ───────────────────────────────────────────────────────────

/// SLIV TX enter-suspend wait count — set all-ones = maximum.
const SLIV_TX_ENT_SUSPEND: u32 = 0b11 << 14;

// ── SLI_CTRL_III / IV — clock selects and pad delays ──────────────────────────

/// Engine clock select — bits [31:28].
const SLI_CLK_SEL_SHIFT: u32 = 28;
const SLI_CLK_SEL_MASK: u32 = 0xF << 28;
/// PHY TX clock select — bits [27:24].
const SLI_PHYCLK_SEL_SHIFT: u32 = 24;
const SLI_PHYCLK_SEL_MASK: u32 = 0xF << 24;

// Engine clock values.
const SLI_CLK_500M: u32 = 0x6;

// PHY clock values.
const SLI_PHYCLK_1G: u32 = 0x5;
// Target PHY clock for AST2750-A1 DCSCM: 1 GHz.
const SLI_TARGET_PHYCLK: u32 = SLI_PHYCLK_1G;

// SLIH pad delay masks (CTRL_III bits [23:0]).
const SLIH_PAD_DLY_TX1_MASK: u32 = 0x3F << 18;
const SLIH_PAD_DLY_TX0_MASK: u32 = 0x3F << 12;
const SLIH_PAD_DLY_RX1_MASK: u32 = 0x3F << 6;
const SLIH_PAD_DLY_RX0_MASK: u32 = 0x3F << 0;

// SLIV pad delay masks (same bit layout).
const SLIV_PAD_DLY_RX1_MASK: u32 = 0x3F << 6;
const SLIV_PAD_DLY_RX0_MASK: u32 = 0x3F;
const SLIV_PAD_DLY_TX1_MASK: u32 = 0x3F << 18;
const SLIV_PAD_DLY_TX0_MASK: u32 = 0x3F << 12;

// SLIM lane mask (6 bits per lane, 4 lanes in CTRL_III/IV).
const SLIM_ALL_LANES_MASK: u32 = 0x00FF_FFFF; // bits [23:0]

// ── SLI_INTR_STATUS bits ───────────────────────────────────────────────────────

const SLI_INTR_TX_SUSPEND: u32 = 1 << 4;
const SLI_INTR_RX_SUSPEND: u32 = 1 << 1;
const SLI_INTR_RX_ERR: u32 = 1 << 13;
const SLI_INTR_RX_NACK: u32 = 1 << 12;
const SLI_INTR_RX_DISCONN: u32 = 1 << 6;
const SLI_INTR_RX_ERRORS: u32 = SLI_INTR_RX_ERR | SLI_INTR_RX_NACK | SLI_INTR_RX_DISCONN;

// ── SLIM_MARB_FUNC_I bits ─────────────────────────────────────────────────────

const SLIM_SLI_MARB_CLR: u32 = 1 << 4;
const SLIM_SLI_MARB_RR: u32 = 1 << 0;

// ── Calibration parameters ────────────────────────────────────────────────────

const CAL_DELAY_US: u32 = 200;
const SET_DELAY_US: u32 = 8;

const SLIH_COARSE_D_BEGIN: i32 = 6;
const SLIH_COARSE_D_END: i32 = 28;
const SLIM_COARSE_D_BEGIN: i32 = 0;
const SLIM_COARSE_D_END: i32 = 28;
const SLIM_FINE_MARGIN: i32 = 5;
const SLIV_COARSE_D_BEGIN: i32 = 0;
const SLIV_COARSE_D_END: i32 = 28;

const SLI_MAX_POLL_CNT_SUSPEND: u32 = 10;
const SLI_MAX_POLL_CNT_CLEAR: u32 = 10;
const SLIM_RETRY_COUNT: u32 = 50;

/// Default MBUS pad delay when coarse midpoint resolves to 0 (1G/800M PHY).
const SLIM_DEFAULT_DELAY: i32 = 5;

// ── MMIO helpers (using MmioBlock for volatile safety) ───────────────────────
//
// SLI calibration computes absolute addresses from multiple bases + offsets.
// These helpers wrap MmioBlock at offset 0 to provide volatile access through
// the safe MMIO abstraction while preserving the flat-address calling convention.

use aspeed_mmio::MmioBlock;

#[inline(always)]
fn rd(addr: usize) -> u32 {
    let block = unsafe { MmioBlock::new(addr) };
    block.read32(0)
}

#[inline(always)]
fn wr(addr: usize, val: u32) {
    let mut block = unsafe { MmioBlock::new(addr) };
    block.write32(0, val);
}

#[inline(always)]
fn set_bits(addr: usize, bits: u32) {
    let mut block = unsafe { MmioBlock::new(addr) };
    block.set_bits32(0, bits);
}

#[inline(always)]
fn clr_bits(addr: usize, bits: u32) {
    let mut block = unsafe { MmioBlock::new(addr) };
    block.clr_bits32(0, bits);
}

#[inline(always)]
fn clrset_bits(addr: usize, clr: u32, set: u32) {
    let mut block = unsafe { MmioBlock::new(addr) };
    block.modify32(0, |v| (v & !clr) | set);
}

#[inline]
fn delay_us(us: u32) {
    for _ in 0..(us * 200) {
        unsafe { core::arch::asm!("nop") };
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SliError {
    /// Remote (CPU-die) did not signal ready within timeout.
    RemoteTimeout,
}

// ── Low-level SLI operations ──────────────────────────────────────────────────

/// Clear all SLI interrupt status bits (W1C, write 0xFFFFF).
#[inline]
fn clear_intr(base: usize) {
    wr(base + SLI_INTR_STATUS, 0xF_FFFF);
}

/// Non-blocking: check if both RX and TX are suspended.
/// Returns 1=suspended, 0=not yet, -1=RX error.
fn is_suspend(base: usize) -> i32 {
    let v = rd(base + SLI_INTR_STATUS);
    if v & SLI_INTR_RX_ERRORS != 0 {
        return -1;
    }
    if (v & (SLI_INTR_RX_SUSPEND | SLI_INTR_TX_SUSPEND))
        == (SLI_INTR_RX_SUSPEND | SLI_INTR_TX_SUSPEND)
    {
        return 1;
    }
    0
}

/// Poll until both RX and TX are suspended, or timeout/error.
fn wait_suspend(base: usize) -> i32 {
    clear_intr(base);
    for _ in 0..SLI_MAX_POLL_CNT_SUSPEND {
        let v = rd(base + SLI_INTR_STATUS);
        if v & SLI_INTR_RX_ERRORS != 0 {
            return -2;
        }
        if (v & (SLI_INTR_RX_SUSPEND | SLI_INTR_TX_SUSPEND))
            == (SLI_INTR_RX_SUSPEND | SLI_INTR_TX_SUSPEND)
        {
            return 0;
        }
        delay_us(1);
    }
    -1
}

/// Poll CTRL_I until `target` bits self-clear.
fn wait_clear_done(base: usize, target: u32) -> i32 {
    for _ in 0..SLI_MAX_POLL_CNT_CLEAR {
        if rd(base + SLI_CTRL_I) & target == 0 {
            return 0;
        }
        delay_us(1);
    }
    -1
}

/// Assert `clr` bits in CTRL_I and wait for them to self-clear.
/// SLI_CLEAR_BUS is excluded from the wait (it's level-triggered).
fn sli_clear(base: usize, clr: u32) -> i32 {
    set_bits(base + SLI_CTRL_I, clr);
    wait_clear_done(base, clr & !SLI_CLEAR_BUS)
}

// ── Pad delay accessors ───────────────────────────────────────────────────────

/// Set SLIH RX pad delays for both lanes.
fn slih_set_rx_delay(base: usize, d0: i32, d1: i32) {
    let set = ((d1 as u32) << 6) | (d0 as u32);
    clrset_bits(
        base + SLI_CTRL_III,
        SLIH_PAD_DLY_RX1_MASK | SLIH_PAD_DLY_RX0_MASK,
        set,
    );
    let _ = rd(base + SLI_CTRL_III); // flush
    delay_us(SET_DELAY_US);
}

/// Set all 4 SLIM pad delays at once (used for coarse sweep).
/// `is_rx=true` → CTRL_III, `is_rx=false` → CTRL_IV.
fn slim_set_delay_all(base: usize, d: i32, is_rx: bool) {
    let d = d as u32;
    let val = (d << 18) | (d << 12) | (d << 6) | d;
    let reg = if is_rx {
        base + SLI_CTRL_III
    } else {
        base + SLI_CTRL_IV
    };
    clrset_bits(reg, SLIM_ALL_LANES_MASK, val);
    let _ = rd(reg);
    delay_us(SET_DELAY_US);
}

/// Set a single SLIM pad delay lane (for fine-tuning).
fn slim_set_delay_single(base: usize, index: usize, d: i32, is_rx: bool) {
    let offset = index * 6;
    let mask = 0x3Fu32 << offset;
    let val = (d as u32) << offset;
    let reg = if is_rx {
        base + SLI_CTRL_III
    } else {
        base + SLI_CTRL_IV
    };
    clrset_bits(reg, mask, val);
    let _ = rd(reg);
    delay_us(SET_DELAY_US);
}

/// Set SLIV RX pad delays.
/// `is_k_rx=true`  → RX positions (bits [11:0])
/// `is_k_rx=false` → TX positions (bits [23:12], shifted by 12)
fn sliv_set_rx_delay(base: usize, d0: i32, d1: i32, is_k_rx: bool) {
    let (d0, d1) = (d0 as u32, d1 as u32);
    if is_k_rx {
        let set = (d1 << 6) | d0;
        clrset_bits(
            base + SLI_CTRL_III,
            SLIV_PAD_DLY_RX1_MASK | SLIV_PAD_DLY_RX0_MASK,
            set,
        );
    } else {
        let set = (d1 << 18) | (d0 << 12);
        clrset_bits(
            base + SLI_CTRL_III,
            SLIV_PAD_DLY_TX1_MASK | SLIV_PAD_DLY_TX0_MASK,
            set,
        );
    }
    let _ = rd(base + SLI_CTRL_III);
    delay_us(SET_DELAY_US);
}

// ── SCU scratch persistence ───────────────────────────────────────────────────

fn log_ahb_window(first: i32, last: i32) {
    let v = ((last as u32 & 0xFF) << 8) | (first as u32 & 0xFF);
    clrset_bits(SCU1_SCRATCH30, 0xFFFF, v);
}

fn get_ahb_window() -> (i32, i32) {
    let v = rd(SCU1_SCRATCH30);
    ((v & 0xFF) as i32, ((v >> 8) & 0xFF) as i32)
}

/// Encode {first, last} for SLIM lane `index` into the SCU scratch words.
/// DS: scu1->scratch[28/29]; US: scu0->cpu_scratch[28/29].
fn log_slim_window(scu_base: usize, index: usize, first: i32, last: i32) {
    let addr = scu_base + if index > 1 { 4 } else { 0 };
    let bit_off = if index & 1 != 0 { 16u32 } else { 0u32 };
    let val = ((last as u32 & 0xFF) << (bit_off + 8)) | ((first as u32 & 0xFF) << bit_off);
    clrset_bits(addr, 0xFFFF << bit_off, val);
}

fn get_slim_window(scu_base: usize, index: usize) -> (i32, i32) {
    let addr = scu_base + if index > 1 { 4 } else { 0 };
    let bit_off = if index & 1 != 0 { 16u32 } else { 0u32 };
    let v = rd(addr);
    let first = ((v >> bit_off) & 0xFF) as i32;
    let last = ((v >> (bit_off + 8)) & 0xFF) as i32;
    (first, last)
}

fn log_sliv_window(scu_addr: usize, first: i32, last: i32) {
    let v = ((last as u32 & 0xFF) << 24) | ((first as u32 & 0xFF) << 16);
    clrset_bits(scu_addr, 0xFFFF_0000, v);
}

fn get_sliv_window(scu_addr: usize) -> (i32, i32) {
    let v = rd(scu_addr);
    (((v >> 16) & 0xFF) as i32, ((v >> 24) & 0xFF) as i32)
}

// ── Check: has SLI already been calibrated? ───────────────────────────────────

fn is_calibrated() -> bool {
    // If IO-die SLIH engine clock has been sped up (SLI_CLK_SEL != 0),
    // calibration has already run.
    let v = rd(SLI1_BASE + SLIH_OFF + SLI_CTRL_III);
    (v & SLI_CLK_SEL_MASK) >> SLI_CLK_SEL_SHIFT != 0
}

// ── MAC hotfix (errata workaround for SLIM MARB) ──────────────────────────────

fn mac_hotfix() {
    let val = rd(SLI1_BASE + SLIM_OFF + 0xB8) & 0xE00;
    if val == 0 {
        return;
    }
    wr(SLI1_BASE + SLIM_OFF + 0x68, val);
    set_bits(SLI1_BASE + SLIM_OFF + SLIM_MARB_FUNC_I, 1 << 5);
}

// ── AHB pad delay calibration ─────────────────────────────────────────────────

fn calibrate_ahb() {
    let io_slih = SLI1_BASE + SLIH_OFF;
    let mut d_first = -1i32;
    let mut d_last = -1i32;
    let mut win_size = 0i32;

    set_bits(io_slih + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);

    // AST2750-A1: SLI_FLAG_RX_LAH_NEG_IO_SLIH is NOT set (A1 + 1GHz).
    clr_bits(io_slih + SLI_CTRL_I, SLI_RX_PHY_LAH_SEL_NEG);

    for dc in SLIH_COARSE_D_BEGIN..SLIH_COARSE_D_END {
        slih_set_rx_delay(io_slih, dc, dc);
        sli_clear(io_slih, SLI_CLEAR_RX | SLI_CLEAR_BUS);
        clear_intr(io_slih);
        delay_us(CAL_DELAY_US);

        if is_suspend(io_slih) > 0 {
            if d_first == -1 {
                d_first = dc;
            }
            d_last = dc;
        } else if d_last != -1 {
            if d_last - d_first > win_size {
                win_size = d_last - d_first;
                log_ahb_window(d_first, d_last);
            }
            d_first = -1;
            d_last = -1;
        }
    }

    if d_last != -1 && d_last - d_first > win_size {
        log_ahb_window(d_first, d_last);
    } else {
        let w = get_ahb_window();
        d_first = w.0;
        d_last = w.1;
    }

    let dc = (d_first + d_last) >> 1;
    slih_set_rx_delay(io_slih, dc, dc);
    sli_clear(io_slih, SLI_CLEAR_RX | SLI_CLEAR_BUS);
    clr_bits(io_slih + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);
    wait_suspend(io_slih);
}

// ── MBUS fine-tune single lane ─────────────────────────────────────────────────

/// Calibrate a single SLIM pad lane over `[begin, end)`.
/// Returns the selected midpoint delay.
fn calibrate_slim_lane(
    tx: usize,
    rx: usize,
    kx: usize,
    scu_base: usize,
    index: usize,
    begin: i32,
    end: i32,
    is_k_rx: bool,
) -> i32 {
    let mut d_first = -1i32;
    let mut d_last = -1i32;

    'outer: for _ in 0..SLIM_RETRY_COUNT {
        d_first = -1;
        d_last = -1;

        for d in begin..end {
            slim_set_delay_single(kx, index, d, is_k_rx);
            sli_clear(tx, SLI_RESET_TRIGGER);
            sli_clear(rx, SLI_RESET_TRIGGER);
            clear_intr(rx);
            delay_us(CAL_DELAY_US);

            if is_suspend(rx) > 0 {
                if d_first == -1 {
                    d_first = d;
                }
                d_last = d;
            } else if d_last != -1 {
                break;
            }
        }

        if d_last - d_first >= 3 {
            break 'outer;
        }
    }

    let d = if d_first == -1 {
        (begin + end) >> 1
    } else {
        (d_first + d_last) >> 1
    };
    log_slim_window(scu_base, index, d_first, d_last);
    d
}

// ── MBUS pad delay calibration ─────────────────────────────────────────────────

/// Calibrate SLIM (MBUS) pad delays.
/// `is_ds=true` → downstream (CPU→IO); `is_k_rx=true` → sweep IO-die RX pads.
fn calibrate_slim(is_ds: bool, is_k_rx: bool) {
    let (tx, rx, scu_base) = if is_ds {
        (SLI0_BASE + SLIM_OFF, SLI1_BASE + SLIM_OFF, SCU1_SCRATCH28)
    } else {
        (
            SLI1_BASE + SLIM_OFF,
            SLI0_BASE + SLIM_OFF,
            SCU0_CPU_SCRATCH28,
        )
    };
    let kx = if is_k_rx { rx } else { tx };

    set_bits(rx + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);

    // AST2750-A1 at 1GHz: SLI_FLAG_RX_LAH_NEG_IO_SLIM is NOT set.
    clr_bits(kx + SLI_CTRL_I, SLI_RX_PHY_LAH_SEL_NEG);

    // Coarse sweep: all 4 lanes simultaneously.
    let mut d_first = -1i32;
    let mut d_last = -1i32;
    let mut win_size = 0i32;

    'retry: for _ in 0..SLIM_RETRY_COUNT {
        d_first = -1;
        d_last = -1;

        for dc in SLIM_COARSE_D_BEGIN..SLIM_COARSE_D_END {
            slim_set_delay_all(kx, dc, is_k_rx);
            sli_clear(tx, SLI_RESET_TRIGGER);
            sli_clear(rx, SLI_RESET_TRIGGER);
            clear_intr(rx);
            delay_us(CAL_DELAY_US);

            if is_suspend(rx) > 0 {
                if d_first == -1 {
                    d_first = dc;
                }
                d_last = dc;
            } else if d_last != -1 {
                if d_last - d_first > win_size {
                    win_size = d_last - d_first;
                    log_slim_window(scu_base, 0, d_first, d_last);
                }
                d_first = -1;
                d_last = -1;
            }
        }

        if d_last != -1 && d_last - d_first > win_size {
            win_size = d_last - d_first;
            log_slim_window(scu_base, 0, d_first, d_last);
        } else {
            let w = get_slim_window(scu_base, 0);
            d_first = w.0;
            d_last = w.1;
        }

        if d_last - d_first >= 3 {
            break 'retry;
        }
    }

    let mut dc = (d_first + d_last) >> 1;
    if dc == 0 {
        dc = SLIM_DEFAULT_DELAY;
    }

    slim_set_delay_all(kx, dc, is_k_rx);

    if win_size > 0 {
        // Fine-tune each lane individually.
        let begin = (dc - SLIM_FINE_MARGIN).max(0);
        let end = (dc + SLIM_FINE_MARGIN).min(31);

        for idx in 0..4usize {
            let d = calibrate_slim_lane(tx, rx, kx, scu_base, idx, begin, end, is_k_rx);
            slim_set_delay_single(kx, idx, d, is_k_rx);
        }
    }

    sli_clear(tx, SLI_RESET_TRIGGER);
    sli_clear(rx, SLI_RESET_TRIGGER);
    clr_bits(rx + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);
    wait_suspend(rx);

    // Enable MARB round-robin mode.
    set_bits(rx + SLIM_MARB_FUNC_I, SLIM_SLI_MARB_RR);
}

// ── Video SLI pad delay calibration ───────────────────────────────────────────

/// Calibrate SLIV (Video) pad delays.
/// `is_ds=true` → downstream (CPU→IO).
fn calibrate_sliv(is_ds: bool, is_k_rx: bool) {
    let (tx, rx, scu_addr) = if is_ds {
        (
            SLI0_BASE + SLIV_OFF,
            SLI1_BASE + SLIV_OFF,
            SCU0_CPU_SCRATCH30,
        )
    } else {
        (SLI1_BASE + SLIV_OFF, SLI0_BASE + SLIV_OFF, SCU1_SCRATCH30)
    };
    let kx = if is_k_rx { rx } else { tx };

    set_bits(rx + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);
    set_bits(tx + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);

    // AST2750-A1: SLI_FLAG_RX_LAH_NEG_IO_SLIV not set.
    clr_bits(rx + SLI_CTRL_I, SLI_RX_PHY_LAH_SEL_NEG);

    // Set roles: RX = raw receiver, TX = raw transmitter.
    clrset_bits(rx + SLI_CTRL_I, SLI_TX_MODE, SLIV_RAW_MODE);
    set_bits(tx + SLI_CTRL_I, SLIV_RAW_MODE | SLI_TX_MODE);
    // Max TX enter-suspend wait.
    set_bits(tx + SLI_CTRL_II, SLIV_TX_ENT_SUSPEND);

    let mut d_first = -1i32;
    let mut d_last = -1i32;
    let mut win_size = 0i32;

    for d in SLIV_COARSE_D_BEGIN..SLIV_COARSE_D_END {
        sliv_set_rx_delay(kx, d, d, is_k_rx);
        sli_clear(rx, SLI_CLEAR_BUS | SLI_RESET_TRIGGER);
        sli_clear(tx, SLI_CLEAR_BUS | SLI_RESET_TRIGGER);
        clear_intr(rx);
        delay_us(CAL_DELAY_US);

        if is_suspend(rx) > 0 {
            if d_first == -1 {
                d_first = d;
            }
            d_last = d;
        } else if d_last != -1 {
            if d_last - d_first > win_size {
                win_size = d_last - d_first;
                log_sliv_window(scu_addr, d_first, d_last);
            }
            d_first = -1;
            d_last = -1;
        }
    }

    if d_last != -1 && d_last - d_first > win_size {
        log_sliv_window(scu_addr, d_first, d_last);
    } else {
        let w = get_sliv_window(scu_addr);
        d_first = w.0;
        d_last = w.1;
    }

    let mut d = (d_first + d_last) >> 1;
    if d == 0 {
        d = 12;
    } // hardcoded fallback

    sliv_set_rx_delay(kx, d, d, is_k_rx);

    sli_clear(rx, SLI_CLEAR_BUS | SLI_RESET_TRIGGER);
    sli_clear(tx, SLI_CLEAR_BUS | SLI_RESET_TRIGGER);
    delay_us(CAL_DELAY_US);
    clr_bits(rx + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);
    clr_bits(tx + SLI_CTRL_I, SLI_AUTO_SEND_TRN_OFF);
    wait_suspend(rx);
}

// ── Public API ────────────────────────────────────────────────────────────────

/// "First" SLI init — runs on BootMCU (IO-die context).
///
/// Calibrates AHB (downstream) and MBUS (downstream) links.
/// Signals completion to the CPU-die via SCU1 scratch[31].
///
/// Call before any IO-die peripheral access that crosses the SLI.
pub fn init_f() {
    #[cfg(feature = "defmt")]
    defmt::info!("SLI first-stage init start");

    // Already calibrated (e.g. warm reset)?
    if is_calibrated() {
        #[cfg(feature = "defmt")]
        defmt::warn!("SLI first-stage init skipped: already calibrated");
        return;
    }

    // Skip-calibration flag set by a prior successful run?
    if rd(SCU1_SCRATCH31) & SCU1_SCRATCH31_SLI_SKIP_CALI != 0 {
        #[cfg(feature = "defmt")]
        defmt::warn!("SLI first-stage init skipped: skip flag set");
        return;
    }

    // Disable AHBC timeouts during calibration (prevents AHB bus errors
    // from incomplete transactions while delays are being swept).
    for &off in AHBC1_TIMEOUT_OFFSETS {
        wr(AHBC1_BASE + off, 0);
    }
    for &off in AHBC_TIMEOUT_OFFSETS {
        wr(AHBC0_BASE + off, 0);
    }

    // Speed up engine clock on both dies (500 MHz).
    clrset_bits(
        SLI1_BASE + SLIH_OFF + SLI_CTRL_III,
        SLI_CLK_SEL_MASK,
        SLI_CLK_500M << SLI_CLK_SEL_SHIFT,
    );
    clrset_bits(
        SLI0_BASE + SLIH_OFF + SLI_CTRL_III,
        SLI_CLK_SEL_MASK,
        SLI_CLK_500M << SLI_CLK_SEL_SHIFT,
    );

    // A1 silicon: disable auto-clear on clock/data changes, no reset on TX clk change.
    for base in [SLI1_BASE + SLIH_OFF, SLI0_BASE + SLIH_OFF] {
        set_bits(
            base + SLI_CTRL_I,
            SLI_AUTO_CLR_OFF_DAT | SLI_AUTO_CLR_OFF_CLK | SLI_NO_RST_TXCLK_CHG,
        );
    }

    // Speed up CPU-die PHY TX clock and clear its TX pad delays.
    clrset_bits(
        SLI0_BASE + SLIH_OFF + SLI_CTRL_III,
        SLI_PHYCLK_SEL_MASK | SLIH_PAD_DLY_TX1_MASK | SLIH_PAD_DLY_TX0_MASK,
        SLI_TARGET_PHYCLK << SLI_PHYCLK_SEL_SHIFT,
    );

    // Calibrate AHB downstream (IO-die RX).
    calibrate_ahb();

    // Calibrate MBUS downstream: sweep IO-die RX pads (CONFIG_SLI_K_ON_CPU not set).
    calibrate_slim(true, true);

    // Signal to CPU-die: IO-die SLI0 calibration complete.
    set_bits(SCU1_SCRATCH31, SCU1_SCRATCH31_SLI0_READY);

    // Reset CPU-die SLIH (clear + wait suspend).
    sli_clear(SLI0_BASE + SLIH_OFF, SLI_CLEAR_BUS);
    wait_suspend(SLI0_BASE + SLIH_OFF);

    #[cfg(feature = "defmt")]
    defmt::info!("SLI first-stage init complete");
}

/// "Remote" SLI init — runs on BootMCU after `init_f` signals SLI0_READY.
///
/// Restores AHB timeouts, acknowledges calibration, calibrates Video SLI.
/// Returns `Err(RemoteTimeout)` if the IO die never set SLI0_READY.
///
/// Note: on BootMCU both dies are accessible, so "remote" here means the
/// CPU-die perspective. `init_r` finalises the link after `init_f` has
/// calibrated the IO-die side.
pub fn init_r() -> Result<(), SliError> {
    #[cfg(feature = "defmt")]
    defmt::info!("SLI remote init start");

    // Skip-cal already set: hotfix + return.
    if rd(SCU1_SCRATCH31) & SCU1_SCRATCH31_SLI_SKIP_CALI != 0 {
        mac_hotfix();
        #[cfg(feature = "defmt")]
        defmt::warn!("SLI remote init skipped: skip flag set");
        return Ok(());
    }

    // Wait up to ~10 s for IO die to finish AHB+MBUS calibration.
    let mut ready = false;
    for _ in 0..100u32 {
        if rd(SCU1_SCRATCH31) & SCU1_SCRATCH31_SLI0_READY != 0 {
            ready = true;
            break;
        }
        // 100 ms wait: ~20M nops at 200 MHz
        for _ in 0..20_000_000u32 {
            unsafe { core::arch::asm!("nop") };
        }
    }
    if !ready {
        #[cfg(feature = "defmt")]
        defmt::error!("SLI remote init timeout waiting for SLI0 ready");
        return Err(SliError::RemoteTimeout);
    }

    // Clear IO-die SLIH RX+bus, wait suspend.
    sli_clear(SLI1_BASE + SLIH_OFF, SLI_CLEAR_RX | SLI_CLEAR_BUS);
    wait_suspend(SLI1_BASE + SLIH_OFF);
    delay_us(CAL_DELAY_US);

    // Restore AHBC1 timeouts.
    for &off in AHBC1_TIMEOUT_OFFSETS {
        wr(AHBC1_BASE + off, AHBC_MAX_TIMEOUT);
    }
    delay_us(CAL_DELAY_US);

    // Signal CPU-die acknowledgement.
    set_bits(SCU0_CPU_SCRATCH31, SCU0_SCRATCH31_SLI1_READY);
    delay_us(CAL_DELAY_US);

    // Restore AHBC0 timeouts.
    for &off in AHBC_TIMEOUT_OFFSETS {
        wr(AHBC0_BASE + off, AHBC_MAX_TIMEOUT);
    }

    // Mark calibration complete so it's not repeated on warm reset.
    set_bits(SCU1_SCRATCH31, SCU1_SCRATCH31_SLI_SKIP_CALI);

    // Reset SLIM MARB before first SLIM use.
    set_bits(SLI1_BASE + SLIM_OFF + SLIM_MARB_FUNC_I, SLIM_SLI_MARB_CLR);

    // Clear IO INTC reset interrupt (W1C: read current status, write it back).
    let v = rd(IO_INTC_INTR_STATUS);
    wr(IO_INTC_INTR_STATUS, v);

    // Calibrate Video SLI upstream (IO→CPU, is_ds=false, is_k_rx=true).
    calibrate_sliv(false, true);

    // Calibrate Video SLI downstream (CPU→IO, is_ds=true, is_k_rx=true).
    // CONFIG_SLI_K_ON_CPU not set, so sweep IO-die RX pads.
    calibrate_sliv(true, true);

    #[cfg(feature = "defmt")]
    defmt::info!("SLI remote init complete");

    Ok(())
}
