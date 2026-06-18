//! SPI Passthrough Filter / QSPI Monitor (SPIPF) driver — AST1060.
//!
//! 4 instances at `0x7E79_1000`–`0x7E79_4000` (stride `0x1000`).
//! NVIC interrupts: `SPIPF1`=87, `SPIPF2`=88, `SPIPF3`=89, `SPIPF4`=90.
//!
//! # Function
//!
//! Each SPIPF instance monitors the SPI bus between a host SPI master (e.g. BMC)
//! and a flash device.  It can:
//!
//! - Pass-through single-bit or multi-bit (Dual/Quad) SPI traffic.
//! - Filter commands: only opcodes present in the 32-entry command table are
//!   allowed through.
//! - Filter write addresses: a 512-entry × 1-bit-per-16-KB table controls which
//!   flash regions a write command may access.
//! - Capture blocked transactions in a FIFO or DMA buffer.
//! - Fire an interrupt on command-block, write-block, or read-block events.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::spi_monitor::{SpiMonitor, CmdEntry};
//!
//! // Configure SPIPF1 (instance 1) to pass all single/multi-bit traffic
//! // and enable command filtering.
//! let mut mon = SpiMonitor::new(1);
//! mon.set_passthrough(true, true);
//! mon.set_filter_enable(true);
//!
//! // Allow Page Program (0x02): write cmd, 3-byte address, 1-bit data.
//! mon.allow_command(4, CmdEntry {
//!     opcode: 0x02, addr_bytes: 3, is_write: true, data_width: 1, ..Default::default()
//! });
//!
//! // Block writes to the first 64 KB (4 × 16 KB regions) of flash.
//! mon.set_write_protect(0, 0x0000_000F); // bits 0-3 → regions 0-3
//! mon.commit_write_protect();
//! ```

use crate::pac;
use aspeed_mmio::MmioBlock;

const SPIPF_BASE: usize = 0x7E79_1000;
const SPIPF_STRIDE: usize = 0x1000;
const IRQ_CTRL: usize = 0x04;
const CMD_TABLE0: usize = 0x80;
const ADDR_TABLE0: usize = 0x100;

// ── SpiMonitor ────────────────────────────────────────────────────────────────

/// SPI Passthrough Filter instance (1-indexed, 1–4).
pub struct SpiMonitor {
    regs: pac::spipf_v1::SPIPF,
    base: usize,
    #[allow(dead_code)]
    inst: u8,
}

impl SpiMonitor {
    /// Create a driver for SPIPF instance `inst` (1–4).
    ///
    /// Does not modify any registers.
    ///
    /// # Panics
    ///
    /// Panics if `inst` is not 1–4.
    pub fn new(inst: u8) -> Self {
        assert!((1..=4).contains(&inst), "SPIPF instance must be 1–4");
        let regs = match inst {
            1 => pac::SPIPF1,
            2 => pac::SPIPF2,
            3 => pac::SPIPF3,
            4 => pac::SPIPF4,
            _ => unreachable!(),
        };
        Self {
            regs,
            base: SPIPF_BASE + (inst as usize - 1) * SPIPF_STRIDE,
            inst,
        }
    }

    fn mmio(&self) -> MmioBlock {
        unsafe { MmioBlock::new(self.base) }
    }

    // ── Passthrough and filter enable ─────────────────────────────────────────

    /// Enable or disable single-bit and multi-bit (Dual/Quad) passthrough.
    ///
    /// Setting both to `false` blocks all SPI traffic.
    pub fn set_passthrough(&self, single_bit: bool, multi_bit: bool) {
        self.regs.ENGINE_CTRL().modify(|w| {
            w.set_SINGLE_BIT_PT(single_bit);
            w.set_MULTI_BIT_PT(multi_bit);
        });
    }

    /// Enable or disable command + address filtering.
    ///
    /// When disabled, all commands pass through regardless of the command table.
    pub fn set_filter_enable(&self, enable: bool) {
        self.regs.ENGINE_CTRL().modify(|w| w.set_FILTER_EN(enable));
    }

    /// Issue a software reset.  Clears FIFO and state machines.
    pub fn reset(&self) {
        self.regs.ENGINE_CTRL().modify(|w| w.set_SW_RESET(true));
        // Self-clearing — no need to write back 0.
    }

    // ── Interrupt control ─────────────────────────────────────────────────────

    /// Enable/disable interrupts for command-block, write-block, read-block events.
    pub fn set_irq_enable(&self, cmd_blk: bool, write_blk: bool, read_blk: bool) {
        self.regs.IRQ_CTRL().modify(|w| {
            w.set_IRQ_EN_CMD_BLK(cmd_blk);
            w.set_IRQ_EN_WRITE_BLK(write_blk);
            w.set_IRQ_EN_READ_BLK(read_blk);
        });
    }

    /// Enable or disable push-pull output mode for monitored SPI signals.
    pub fn set_push_pull(&self, enable: bool) {
        const PUSH_PULL_BIT: u32 = 1 << 31;
        let mut regs = self.mmio();
        regs.modify32(IRQ_CTRL, |value| {
            if enable {
                value | PUSH_PULL_BIT
            } else {
                value & !PUSH_PULL_BIT
            }
        });
    }

    /// Clear all interrupt status bits (write-1-to-clear).
    pub fn clear_irq_status(&self) {
        self.regs.IRQ_CTRL().modify(|w| {
            w.set_IRQ_ST_CMD_BLK(true);
            w.set_IRQ_ST_WRITE_BLK(true);
            w.set_IRQ_ST_READ_BLK(true);
        });
    }

    /// Read current interrupt status.  Returns `(cmd_blocked, write_blocked, read_blocked)`.
    pub fn irq_status(&self) -> (bool, bool, bool) {
        let r = self.regs.IRQ_CTRL().read();
        (
            r.IRQ_ST_CMD_BLK(),
            r.IRQ_ST_WRITE_BLK(),
            r.IRQ_ST_READ_BLK(),
        )
    }

    // ── Command table ─────────────────────────────────────────────────────────

    /// Install a command table entry at `slot` (0–31).
    ///
    /// Sets `VALID=1` so the entry is active.
    ///
    /// # Panics
    ///
    /// Panics if `slot` > 31.
    pub fn allow_command(&self, slot: u8, entry: CmdEntry) {
        assert!(slot <= 31, "command table slot must be 0–31");
        let mut w = pac::spipf_v1::SPIPF_CMD_ENTRY(0);
        w.set_COMMAND(entry.opcode);
        w.set_ADDR_MASK(entry.addr_bytes);
        w.set_ADDR_WIDTH(entry.addr_width);
        w.set_DATA_WIDTH(entry.data_width);
        w.set_READ_CMD(entry.is_read);
        w.set_WRITE_CMD(entry.is_write);
        w.set_MEM_CMD(entry.is_mem);
        w.set_ERASE_SIZE(entry.erase_size);
        w.set_DUMMY_CYCLES(entry.dummy_cycles);
        w.set_VALID(true);
        self.mmio().write32(CMD_TABLE0 + slot as usize * 4, w.0);
    }

    /// Disable command table entry at `slot` (clear VALID bit).
    pub fn block_command(&self, slot: u8) {
        assert!(slot <= 31, "command table slot must be 0–31");
        self.mmio().write32(CMD_TABLE0 + slot as usize * 4, 0);
    }

    /// Clear all 32 command table entries (block every command).
    pub fn clear_command_table(&self) {
        for slot in 0..32u8 {
            self.block_command(slot);
        }
    }

    // ── Address (write-protect) table ─────────────────────────────────────────

    /// Select the Write-Disable address table for subsequent `set_write_protect` calls.
    ///
    /// Must be called before writing `ADDR_TABLE` entries.
    pub fn select_write_disable_table(&self) {
        // Write magic byte 0x57 ('W') to ENGINE_CTRL[31:24].
        self.regs
            .ENGINE_CTRL()
            .modify(|w| w.set_ADDR_TBL_SEL_WRITE(0x57));
    }

    /// Select the Read-Enable address table.
    pub fn select_read_enable_table(&self) {
        self.regs
            .ENGINE_CTRL()
            .modify(|w| w.set_ADDR_TBL_SEL_WRITE(0x52));
    }

    /// Write one 32-bit entry of the address filter table.
    ///
    /// Each bit N in `regions` controls the 16 KB flash region at offset
    /// `(entry * 32 + N) * 16 KB`.
    ///
    /// In Write-Disable table: bit=1 blocks writes to that region.
    /// In Read-Enable table:   bit=1 allows reads from that region.
    ///
    /// `entry` covers the first 16 entries (0–15), addressing up to 8 MB.
    ///
    /// # Panics
    ///
    /// Panics if `entry` > 15.
    pub fn set_address_table(&self, entry: u8, regions: u32) {
        assert!(entry <= 15, "address table entry must be 0–15");
        self.mmio()
            .write32(ADDR_TABLE0 + entry as usize * 4, regions);
    }

    /// Convenience: protect the first `size_bytes` of flash from host writes.
    ///
    /// Selects the Write-Disable table, then sets bits for all 16 KB regions
    /// within `[0, size_bytes)`.
    pub fn protect_flash_region(&self, size_bytes: u32) {
        self.select_write_disable_table();
        let regions_16kb = (size_bytes + 0x3FFF) / 0x4000; // ceiling
        let full_entries = (regions_16kb / 32) as u8;
        let tail_bits = regions_16kb % 32;

        for e in 0..full_entries.min(16) {
            self.set_address_table(e, 0xFFFF_FFFF);
        }
        if tail_bits > 0 && full_entries < 16 {
            self.set_address_table(full_entries, (1u32 << tail_bits) - 1);
        }
    }

    /// ISR entry point.  Called by SPIPF1–4 interrupt vectors.
    pub(crate) fn on_interrupt(_inst: u8) {
        // Future: wake a blocked-transaction watcher task.
        // For now, just clear all status bits.
    }
}

// ── CmdEntry ──────────────────────────────────────────────────────────────────

/// Command table entry for [`SpiMonitor::allow_command`].
#[derive(Copy, Clone, Debug, Default)]
pub struct CmdEntry {
    /// SPI command opcode.
    pub opcode: u8,
    /// Address byte count (0=no address, 1–4 bytes).
    pub addr_bytes: u8,
    /// Address width (0=no addr, 1=1-bit, 2=dual, 3=quad).
    pub addr_width: u8,
    /// Data width (0=no data, 1=1-bit, 2=dual, 3=quad).
    pub data_width: u8,
    /// Number of dummy cycles between address and data phases.
    pub dummy_cycles: u8,
    /// Maximum erase size (0=not erase, 1=≤4KB, 2=≤8KB, … 7=≤256KB).
    pub erase_size: u8,
    /// Command is a read (from flash to host).
    pub is_read: bool,
    /// Command is a write (from host to flash).
    pub is_write: bool,
    /// Command accesses flash memory array.
    pub is_mem: bool,
}

// ── Interrupt handlers ────────────────────────────────────────────────────────

macro_rules! spipf_irq {
    ($name:ident, $inst:expr) => {
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn $name() {
            SpiMonitor::on_interrupt($inst);
        }
    };
}

spipf_irq!(SPIPF1, 1);
spipf_irq!(SPIPF2, 2);
spipf_irq!(SPIPF3, 3);
spipf_irq!(SPIPF4, 4);
