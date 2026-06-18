//! SMBus/I2C PFR Filter driver — AST1060.
//!
//! Controls the 4-instance SMBus filter at `0x7E7B_2000`.
//! NVIC interrupt: **IRQ 127** (`SMBUS_FILTER`).
//!
//! # Function
//!
//! Each filter instance monitors one I2C bus and either passes or blocks
//! transactions based on a 256-bit whitelist bitmap stored in SRAM.
//!
//! The bitmap has one bit per (address, direction) pair:
//! - Bit `(addr << 1) | 0` = 7-bit address, write direction.
//! - Bit `(addr << 1) | 1` = 7-bit address, read direction.
//!
//! Setting all bits allows all transactions; clearing all blocks everything.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::i2cfilter::{I2cFilter, FilterTable};
//!
//! // Static whitelist table (must be in DMA-accessible, non-cached SRAM).
//! static mut FILTER_TABLE: FilterTable = FilterTable::PASS_ALL;
//!
//! let filter = I2cFilter::new(0); // instance 0
//! filter.init(100_000_000, 100); // 100 MHz PCLK, 100 kHz I2C
//!
//! // Allow address 0x50 read and write, block everything else.
//! // SAFETY: single-threaded init before enabling filter.
//! unsafe {
//!     FILTER_TABLE.block_all();
//!     FILTER_TABLE.allow_addr(0x50, true, true);
//!     filter.set_whitelist(core::ptr::addr_of!(FILTER_TABLE) as u32);
//! }
//! filter.enable(true);
//! ```

use crate::pac;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Number of SMBUS filter instances.
pub const FILTER_COUNT: usize = 4;

// ── I2cFilter ─────────────────────────────────────────────────────────────────

/// SMBus/I2C PFR Filter instance (0-indexed, 0–3).
pub struct I2cFilter {
    inst: u8,
    regs: pac::i2cfilter_v1::SMBUS_FILTER,
    global: pac::i2cfilter_v1::SMBUS_FILTER_GLOBAL,
}

impl I2cFilter {
    /// Create a driver for filter instance `inst` (0–3).
    ///
    /// Does not modify any registers.
    ///
    /// # Panics
    ///
    /// Panics if `inst` ≥ 4.
    pub fn new(inst: u8) -> Self {
        assert!(
            (inst as usize) < FILTER_COUNT,
            "filter instance must be 0–3"
        );
        let regs = match inst {
            0 => pac::SMBUS_FILTER0,
            1 => pac::SMBUS_FILTER1,
            2 => pac::SMBUS_FILTER2,
            3 => pac::SMBUS_FILTER3,
            _ => unreachable!(),
        };
        Self {
            inst,
            regs,
            global: pac::SMBUS_FILTER_GLOBAL,
        }
    }

    /// Initialise the filter: configure timeout timing, clear and enable
    /// the local interrupt, and register in the global interrupt enable.
    ///
    /// - `pclk_hz`: APB peripheral clock frequency in Hz (e.g. 100_000_000).
    /// - `i2c_khz`: I2C bus clock in kHz (e.g. 100).
    pub fn init(&self, pclk_hz: u32, i2c_khz: u32) {
        // Disable filter before configuration.
        self.regs.EN().write(|w| w.set_EN(false));
        self.regs.CFG().write(|w| w.set_EN(false));

        // Timeout count = PCLK / (i2c_kHz * 3 * 1000).
        let count = pclk_hz / (i2c_khz * 3 * 1000);
        let timing = (count << 16) | (count & 0xFFFF);
        self.regs.TIMING().write(|w| {
            w.set_TIMEOUT_LO(count as u16);
            w.set_TIMEOUT_HI(count as u16);
        });
        let _ = timing; // used via individual fields above

        // Clear then enable local interrupt.
        self.regs.INT_STS().write(|w| w.set_EN(true));
        self.regs.INT_EN().write(|w| w.set_EN(true));

        // Enable this instance in the global interrupt mask.
        self.global.GLOBAL_INT_EN().modify(|w| {
            let prev = w.INST_MASK();
            w.set_INST_MASK(prev | (1 << self.inst));
        });
    }

    /// Point the filter hardware at the whitelist table in SRAM.
    ///
    /// `phys_addr` must be the **physical** (DMA-visible) address of a
    /// [`FilterTable`] struct, aligned to 16 bytes.
    ///
    /// Call before [`enable`].
    pub fn set_whitelist(&self, phys_addr: u32) {
        self.regs.BUF().write(|w| w.set_ADDR(phys_addr));
    }

    /// Enable or disable the filter and optional whitelist enforcement.
    ///
    /// - `filter_en = true, whitelist_en = false`: enable filter, pass-all mode
    ///   (requires a pass-all bitmap in the table).
    /// - `filter_en = true, whitelist_en = true`: enforce per-address bitmap.
    /// - `filter_en = false`: pass all traffic (filter bypassed).
    pub fn enable(&self, filter_en: bool) {
        self.regs.EN().write(|w| w.set_EN(filter_en));
        self.regs.CFG().write(|w| w.set_EN(filter_en));
    }

    /// Disable this filter instance and clear its global interrupt bit.
    pub fn disable(&self) {
        self.regs.EN().write(|w| w.set_EN(false));
        self.regs.CFG().write(|w| w.set_EN(false));
        self.regs.INT_EN().write(|w| w.set_EN(false));
        self.global.GLOBAL_INT_EN().modify(|w| {
            let prev = w.INST_MASK();
            w.set_INST_MASK(prev & !(1 << self.inst));
        });
    }

    /// Set an address re-map slot.
    ///
    /// `slot`: 0–15.  `addr`: 7-bit I2C address to map to this bitmap entry.
    pub fn set_remap_slot(&self, slot: u8, addr: u8) {
        assert!(slot <= 15, "remap slot must be 0–15");
        let reg_idx = slot / 4;
        let byte_shift = (slot % 4) * 8;
        let map_reg = match reg_idx {
            0 => self.regs.MAP0(),
            1 => self.regs.MAP1(),
            2 => self.regs.MAP2(),
            3 => self.regs.MAP3(),
            _ => unreachable!(),
        };
        map_reg.modify(|w| {
            // Each MAP register holds 4 address bytes.  We write the slot's byte.
            // The PAC exposes SLOT0_ADDR–SLOT3_ADDR per register.
            match slot % 4 {
                0 => w.set_SLOT0_ADDR(addr),
                1 => w.set_SLOT1_ADDR(addr),
                2 => w.set_SLOT2_ADDR(addr),
                3 => w.set_SLOT3_ADDR(addr),
                _ => unreachable!(),
            }
        });
        let _ = byte_shift; // already handled via individual field setters
    }

    /// Read and clear the local interrupt status.  Returns `true` if the
    /// interrupt was pending.
    pub fn take_irq(&self) -> bool {
        let pending = self.regs.INT_STS().read().EN();
        if pending {
            self.regs.INT_STS().write(|w| w.set_EN(true)); // W1C
        }
        pending
    }

    /// ISR entry point.  Called from the `SMBUS_FILTER` vector.
    pub(crate) fn on_interrupt() {
        // Read global status to find which instance(s) fired.
        let global = pac::SMBUS_FILTER_GLOBAL;
        let sts = global.GLOBAL_INT_STS().read().INST_MASK();
        // Clear all pending status bits.
        global.GLOBAL_INT_STS().write(|w| w.set_INST_MASK(sts));
        // Future: wake per-instance watchers.
    }
}

// ── FilterTable ───────────────────────────────────────────────────────────────

/// Whitelist bitmap table for one filter instance.
///
/// Must be placed in DMA-accessible, non-cached SRAM and aligned to 16 bytes.
///
/// Filter table layout:
/// - Entry 0: default bitmap (used when no slot matches).
/// - Entries 1–16: per-slot bitmaps, indexed by `MAP0`–`MAP3` re-map slots.
///
/// Each 32-byte bitmap has 256 bits:
/// - Bit `(7-bit_addr << 1) | R/nW`: 1 = allow, 0 = block.
#[repr(C, align(16))]
pub struct FilterTable {
    /// 17 bitmap entries × 8 × u32 = 17 × 32 bytes = 544 bytes.
    pub entries: [[u32; 8]; 17],
}

impl FilterTable {
    /// A table that passes all addresses in all directions.
    pub const PASS_ALL: Self = Self {
        entries: [[0xFFFF_FFFF; 8]; 17],
    };

    /// A table that blocks all addresses.
    pub const BLOCK_ALL: Self = Self {
        entries: [[0u32; 8]; 17],
    };

    /// Zero all entries (block everything).
    pub fn block_all(&mut self) {
        for entry in self.entries.iter_mut() {
            entry.fill(0);
        }
    }

    /// Set all entries to all-ones (pass everything).
    pub fn pass_all(&mut self) {
        for entry in self.entries.iter_mut() {
            entry.fill(0xFFFF_FFFF);
        }
    }

    /// Allow or block a 7-bit address in the default bitmap (entry 0).
    ///
    /// - `allow_write`: allow transactions with R/nW = 0 (write).
    /// - `allow_read`:  allow transactions with R/nW = 1 (read).
    pub fn allow_addr(&mut self, addr: u8, allow_write: bool, allow_read: bool) {
        let addr = addr as usize & 0x7F;
        if allow_write {
            let bit = addr * 2;
            self.entries[0][bit / 32] |= 1 << (bit % 32);
        }
        if allow_read {
            let bit = addr * 2 + 1;
            self.entries[0][bit / 32] |= 1 << (bit % 32);
        }
    }

    /// Block a 7-bit address in the default bitmap (entry 0).
    pub fn block_addr(&mut self, addr: u8) {
        let addr = addr as usize & 0x7F;
        for dir in 0..2usize {
            let bit = addr * 2 + dir;
            self.entries[0][bit / 32] &= !(1 << (bit % 32));
        }
    }
}

// ── Interrupt handler — IRQ 127 ───────────────────────────────────────────────

#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn SMBUS_FILTER() {
    I2cFilter::on_interrupt();
}
