//! SPI flash driver: read/write/erase for FMC and SPI1/SPI2 (AST1060).
#![allow(dead_code)]
//!
//! # Controllers
//!
//! | Instance | Ctrl base | Flash window | CEs |
//! |----------|-----------|-------------|-----|
//! | FMC | `0x7E62_0000` | `0x0000_0000` (XIP) | CE0 (boot), CE1 |
//! | SPI1 | `0x7E63_0000` | `0x9000_0000` | CE0, CE1 |
//! | SPI2 | `0x7E64_0000` | `0xB000_0000` | CE0, CE1 |
//!
//! # Transfer modes
//!
//! - **Memory-mapped read** (`read_memory_mapped`): direct pointer read from the
//!   XIP window; fastest path, no register access needed.
//! - **Command-mode write/erase** (`write_page`, `erase_sector`): temporarily
//!   sets CE to Normal-Write mode (`CMD_MODE=0b10`), issues write-enable (WREN)
//!   then the write or erase command, polls WIP until done, restores CE mode.
//!
//! # Write protection
//!
//! The controller's command filter and write-address filter provide hardware
//! write protection. This driver does not modify those registers; ensure
//! filters are configured before calling write/erase functions.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::spi::{SpiBus, Controller};
//!
//! // Read from FMC CE0 (boot flash, XIP window at 0x0).
//! let bus = SpiBus::new(Controller::Fmc, 0); // CE0
//! let mut buf = [0u8; 256];
//! bus.read_memory_mapped(0x1000, &mut buf);  // read 256 bytes at flash offset 0x1000
//!
//! // Erase a 4 KB sector and write a page.
//! bus.erase_sector(0x1000).await.unwrap();
//! bus.write_page(0x1000, &data).await.unwrap();
//! ```

use core::ptr;

// ── Controller bases and flash windows ───────────────────────────────────────

/// SPI flash controller instance.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Controller {
    Fmc,
    Spi1,
    Spi2,
}

impl Controller {
    fn ctrl_base(self) -> usize {
        match self {
            Controller::Fmc => 0x7E62_0000,
            Controller::Spi1 => 0x7E63_0000,
            Controller::Spi2 => 0x7E64_0000,
        }
    }

    fn flash_window(self) -> usize {
        match self {
            Controller::Fmc => 0x0000_0000,
            Controller::Spi1 => 0x9000_0000,
            Controller::Spi2 => 0xB000_0000,
        }
    }
}

// ── Register offsets (word index from controller base) ────────────────────────

const CE_TYPE: usize = 0x000 / 4; // CE type / write-enable
const CE_CTRL: usize = 0x004 / 4; // CE control (tCSH, address mode)
const IRQ_CTRL: usize = 0x008 / 4; // Interrupt / status
const CE0_CTRL: usize = 0x010 / 4; // CE0 control (I/O mode, clock, cmd)
const CE1_CTRL: usize = 0x014 / 4; // CE1 control
const CE0_RANGE: usize = 0x030 / 4; // CE0 address decode range
const CE1_RANGE: usize = 0x034 / 4; // CE1 address decode range
const DMA_CTRL: usize = 0x080 / 4; // DMA control
const DMA_FLASH: usize = 0x084 / 4; // DMA flash address
const DMA_RAM: usize = 0x088 / 4; // DMA DRAM address
const DMA_LEN: usize = 0x08C / 4; // DMA length

// ── CE control register bit fields ───────────────────────────────────────────

/// CE control: CMD_MODE bits [1:0] in CE0/CE1 control register.
const CMD_MODE_MASK: u32 = 0b11;
const CMD_MODE_AUTO_READ: u32 = 0b00;
const CMD_MODE_NORMAL_READ: u32 = 0b01;
const CMD_MODE_NORMAL_WRITE: u32 = 0b10;
const CMD_MODE_USER: u32 = 0b11;

/// CE_STOP bit [2]: deactivate CE# immediately (end User-Mode transfer).
const CE_STOP: u32 = 1 << 2;

/// CE SPI command byte [23:16].
const SPI_CMD_SHIFT: u32 = 16;

// ── Common SPI flash commands ─────────────────────────────────────────────────

const CMD_WREN: u8 = 0x06; // Write Enable
const CMD_RDSR: u8 = 0x05; // Read Status Register
const CMD_PP: u8 = 0x02; // Page Program (3-byte addr)
const CMD_PP4B: u8 = 0x12; // Page Program (4-byte addr)
const CMD_SE: u8 = 0x20; // Sector Erase 4 KB (3-byte addr)
const CMD_SE4B: u8 = 0x21; // Sector Erase 4 KB (4-byte addr)
const CMD_BE: u8 = 0xD8; // Block Erase 64 KB
const CMD_JEDEC: u8 = 0x9F; // Read JEDEC ID

/// WIP (Write In Progress) bit in status register.
const SR_WIP: u8 = 1 << 0;

// ── SpiBus ────────────────────────────────────────────────────────────────────

/// SPI flash bus driver for one chip-enable on one controller.
pub struct SpiBus {
    ctrl: usize, // controller base address
    ce_idx: u8,  // 0 or 1
    win: usize,  // flash memory-mapped window base
}

impl SpiBus {
    /// Create a bus instance for the given controller and chip-enable (0 or 1).
    ///
    /// # Panics
    ///
    /// Panics if `ce` is not 0 or 1.
    pub fn new(controller: Controller, ce: u8) -> Self {
        assert!(ce <= 1, "CE must be 0 or 1");
        Self {
            ctrl: controller.ctrl_base(),
            ce_idx: ce,
            win: controller.flash_window(),
        }
    }

    // ── Register accessors ────────────────────────────────────────────────────

    fn rr(&self, off: usize) -> u32 {
        unsafe { ptr::read_volatile((self.ctrl as *const u32).add(off)) }
    }

    fn rw(&self, off: usize, val: u32) {
        unsafe { ptr::write_volatile((self.ctrl as *mut u32).add(off), val) }
    }

    fn ce_ctrl_off(&self) -> usize {
        if self.ce_idx == 0 {
            CE0_CTRL
        } else {
            CE1_CTRL
        }
    }

    fn ce_range_off(&self) -> usize {
        if self.ce_idx == 0 {
            CE0_RANGE
        } else {
            CE1_RANGE
        }
    }

    fn set_cmd_mode(&self, mode: u32) {
        let off = self.ce_ctrl_off();
        let v = self.rr(off);
        self.rw(off, (v & !CMD_MODE_MASK) | (mode & CMD_MODE_MASK));
    }

    fn restore_cmd_mode(&self) {
        self.set_cmd_mode(CMD_MODE_AUTO_READ);
    }

    // ── User-mode single-byte transfer ────────────────────────────────────────

    /// Issue a single command byte in user mode and read `rx_len` bytes.
    /// Used for WREN, RDSR (WIP polling), JEDEC ID.
    fn user_transfer(&self, cmd: u8, tx: &[u8], rx: &mut [u8]) {
        let off = self.ce_ctrl_off();

        // Enter user mode.
        let saved = self.rr(off);
        self.rw(off, (saved & !CMD_MODE_MASK) | CMD_MODE_USER);

        // The flash window in user mode is a data port.
        let win_ptr = self.win as *mut u8;

        // SAFETY: user-mode MMIO write — each write triggers one SPI clock.
        unsafe {
            ptr::write_volatile(win_ptr, cmd);
            for &b in tx {
                ptr::write_volatile(win_ptr, b);
            }
            for slot in rx.iter_mut() {
                // Write dummy to clock in RX data.
                ptr::write_volatile(win_ptr, 0xFF);
                *slot = ptr::read_volatile(win_ptr);
            }
        }

        // Deactivate CE# (end transfer).
        self.rw(off, self.rr(off) | CE_STOP);

        // Restore previous mode.
        self.rw(off, (saved & !CMD_MODE_MASK) | CMD_MODE_AUTO_READ);
    }

    fn write_enable(&self) {
        self.user_transfer(CMD_WREN, &[], &mut []);
    }

    fn read_status(&self) -> u8 {
        let mut sr = [0u8];
        self.user_transfer(CMD_RDSR, &[], &mut sr);
        sr[0]
    }

    fn wait_not_busy(&self) {
        while self.read_status() & SR_WIP != 0 {
            core::hint::spin_loop();
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Read from the memory-mapped XIP window.
    ///
    /// This is a direct pointer read — no register access. Always valid when
    /// the controller is in auto-read mode (the default after reset).
    ///
    /// `offset` is the byte offset into the flash from the CE0 base of this
    /// controller's flash window.
    pub fn read_memory_mapped(&self, offset: u32, buf: &mut [u8]) {
        let base = (self.win + offset as usize) as *const u8;
        // SAFETY: XIP read from memory-mapped flash window.
        for (i, slot) in buf.iter_mut().enumerate() {
            *slot = unsafe { ptr::read_volatile(base.add(i)) };
        }
    }

    /// Read the JEDEC ID (3 bytes: manufacturer, memory type, capacity).
    pub fn read_jedec_id(&self) -> [u8; 3] {
        let mut id = [0u8; 3];
        self.user_transfer(CMD_JEDEC, &[], &mut id);
        id
    }

    /// Erase a 4 KB sector at `offset` (must be 4 KB aligned).
    ///
    /// Blocks until WIP clears (~100 ms for most SPI flash).
    pub async fn erase_sector(&self, offset: u32) -> Result<(), SpiError> {
        if offset & 0xFFF != 0 {
            return Err(SpiError::Unaligned);
        }
        self.wait_not_busy();
        self.write_enable();

        // Issue sector erase via user mode (3-byte address).
        let addr = [(offset >> 16) as u8, (offset >> 8) as u8, offset as u8];
        self.user_transfer(CMD_SE, &addr, &mut []);
        self.wait_not_busy();
        Ok(())
    }

    /// Erase a 64 KB block at `offset` (must be 64 KB aligned).
    pub async fn erase_block(&self, offset: u32) -> Result<(), SpiError> {
        if offset & 0xFFFF != 0 {
            return Err(SpiError::Unaligned);
        }
        self.wait_not_busy();
        self.write_enable();

        let addr = [(offset >> 16) as u8, (offset >> 8) as u8, offset as u8];
        self.user_transfer(CMD_BE, &addr, &mut []);
        self.wait_not_busy();
        Ok(())
    }

    /// Write up to 256 bytes to a page.
    ///
    /// `offset` and `data.len()` must both fit within one 256-byte page
    /// (i.e., `(offset & 0xFF) + data.len() <= 256`).
    ///
    /// The target sector must have been erased before writing.
    ///
    /// Blocks until WIP clears (~1 ms for most SPI flash).
    pub async fn write_page(&self, offset: u32, data: &[u8]) -> Result<(), SpiError> {
        if data.is_empty() || data.len() > 256 {
            return Err(SpiError::InvalidLength);
        }
        if ((offset & 0xFF) as usize) + data.len() > 256 {
            return Err(SpiError::PageCrossBoundary);
        }
        self.wait_not_busy();
        self.write_enable();

        let off = self.ce_ctrl_off();
        let saved = self.rr(off);

        // Set Normal-Write mode with Page Program command (0x02).
        self.rw(
            off,
            (saved & !CMD_MODE_MASK & !(0xFF << SPI_CMD_SHIFT))
                | CMD_MODE_NORMAL_WRITE
                | ((CMD_PP as u32) << SPI_CMD_SHIFT),
        );

        // Write the 3-byte address then data bytes to the flash window.
        // In Normal-Write mode, the controller issues CMD+ADDR+DATA automatically.
        let win = (self.win + offset as usize) as *mut u8;
        // SAFETY: MMIO write to flash window in Normal-Write mode.
        for (i, &b) in data.iter().enumerate() {
            unsafe { ptr::write_volatile(win.add(i), b) };
        }

        // Restore auto-read mode.
        self.rw(off, (saved & !CMD_MODE_MASK) | CMD_MODE_AUTO_READ);

        self.wait_not_busy();
        Ok(())
    }

    /// DMA-based read from flash to SRAM (uses controller DMA engine).
    ///
    /// `flash_offset`: byte offset in flash (4-byte aligned).
    /// `sram_addr`: physical SRAM destination address (4-byte aligned).
    /// `len`: transfer length in bytes (4-byte aligned).
    pub fn dma_read(&self, flash_offset: u32, sram_addr: u32, len: u32) -> Result<(), SpiError> {
        if len == 0 {
            // A zero-length DMA programs DMA_LEN = 0xFFFF_FFFF (saturating_sub(1)),
            // which the hardware interprets as a ~4 GB transfer.
            return Err(SpiError::InvalidLength);
        }
        if flash_offset & 3 != 0 || sram_addr & 3 != 0 || len & 3 != 0 {
            return Err(SpiError::Unaligned);
        }
        // Configure DMA.
        self.rw(DMA_FLASH, (self.win as u32) + flash_offset);
        self.rw(DMA_RAM, sram_addr);
        self.rw(DMA_LEN, len.saturating_sub(1)); // len-1 for 0-based count
                                                 // Enable DMA read (dir=0, mem mode).
        self.rw(DMA_CTRL, 0b01); // enable=1, dir=0 (read)

        // Poll DMA status.
        while self.rr(IRQ_CTRL) & (1 << 11) == 0 {
            core::hint::spin_loop();
        }
        // Disable DMA.
        self.rw(DMA_CTRL, 0);
        Ok(())
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

/// SPI flash operation error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SpiError {
    /// Address or length not aligned to required boundary.
    Unaligned,
    /// Data length invalid (e.g. > 256 bytes for page write).
    InvalidLength,
    /// Write would cross a 256-byte page boundary.
    PageCrossBoundary,
    /// Write attempted to a protected address range.
    WriteProtected,
}
