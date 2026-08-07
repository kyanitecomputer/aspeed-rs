//! SPI flash driver — PAC-based, async timer-yield.
//!
//! Source of truth: `aspeed-data/data/registers/fmc_v1.yaml`.
//!
//! # Controllers
//!
//! | Instance | PAC | Ctrl base | Flash window | CEs |
//! |----------|-----|-----------|-------------|-----|
//! | FMC | `pac::FMC` / `fmc_v1::FMC` | `0x7E62_0000` | `0x0000_0000` (XIP) | CE0, CE1 |
//! | SPI1 | `pac::SPI1` / `fmc_v1::SPI` | `0x7E63_0000` | `0x9000_0000` | CE0, CE1 |
//! | SPI2 | `pac::SPI2` / `fmc_v1::SPI` | `0x7E64_0000` | `0xB000_0000` | CE0, CE1 |
//!
//! # PAC register mapping
//!
//! All MMIO controller registers are accessed via PAC types.  The flash
//! **memory window** (XIP reads, user-mode data port) uses `MmioBlock` with
//! byte-width accessors — it is address-space memory, not a named peripheral
//! register block.
//!
//! Both `fmc_v1::FMC` and `fmc_v1::SPI` expose `CE0_CTRL`/`CE1_CTRL` with the
//! shared `SPI_CE_N_CTRL` fieldset.  The key fields used here:
//!
//! | PAC field | Meaning |
//! |-----------|---------|
//! | `CMD_MODE` [1:0] | 0=Auto-Read, 2=Normal-Write, 3=User-Mode |
//! | `CE_STOP` [2] | Deactivate CE# (end User-Mode transfer) |
//! | `SPI_CMD` [23:16] | Command byte for Normal-Read/Write CMD phase |
//!
//! # SPI1/SPI2 DMA note
//!
//! SPI1 and SPI2 DMA requires AHB arbitration (fmc_v1.yaml `SPI_DMA_CTRL`
//! comment: "write 0xAEED_0000 to request, wait for bit[30]=1").  The current
//! `dma_read` is therefore FMC-only.  SPI1/SPI2 DMA returns `SpiError::NotSupported`.
//!
//! # Async model
//!
//! - `erase_sector` / `erase_block` / `write_page`: issue SPI command, then
//!   yield every `WIP_POLL_US` µs while polling `SR.WIP`.
//! - `dma_read`: start DMA (FMC only), yield every `DMA_POLL_US` µs polling
//!   `IRQ_CTRL.DMA_STATUS` (fmc_v1.yaml `SPI_IRQ_CTRL.DMA_STATUS` = bit 11).

use aspeed_mmio::MmioBlock;

#[cfg(feature = "ast1060")]
use embassy_time::Timer;

#[cfg(feature = "ast1060")]
use crate::pac;

#[cfg(feature = "ast2700-bootmcu")]
const AST2700_FMC_BASE: usize = 0x1400_0000;
#[cfg(feature = "ast2700-bootmcu")]
const AST2700_SPI_WINDOW: usize = 0x2000_0000;

/// Enable 4-byte address mode for the AST2700 BootMCU FMC CE0 boot flash.
///
/// Sends the standard SPI NOR `Enter 4-Byte Address Mode` command (0xB7) via
/// FMC user mode, then configures the FMC auto-read path for 4-byte addresses.
#[cfg(feature = "ast2700-bootmcu")]
pub fn ast2700_fmc_enable_ce0_4byte_addr() {
    const FMC_CE_CTRL: usize = 0x004;
    const FMC_CE0_CTRL: usize = 0x010;

    let mut fmc = unsafe { MmioBlock::new(AST2700_FMC_BASE) };
    let mut win = unsafe { MmioBlock::new(AST2700_SPI_WINDOW) };

    let saved = fmc.read32(FMC_CE0_CTRL);
    fmc.write32(FMC_CE0_CTRL, (saved & !0x07) | 0x07); // CE_STOP
    fmc.write32(FMC_CE0_CTRL, (saved & !0x07) | 0x03); // User mode
    win.write8(0, 0xB7); // Enter 4-byte mode command
    fmc.write32(FMC_CE0_CTRL, (saved & !0x07) | 0x07); // CE_STOP
    fmc.write32(FMC_CE0_CTRL, saved); // Restore

    fmc.modify32(FMC_CE_CTRL, |v| v | (1 << 0) | (1 << 4));
}

/// Read from AST2700 BootMCU FMC CE0 into DRAM using FMC DMA (sync).
///
/// Returns `false` if the transfer is unaligned, exceeds 32 MiB, or times out.
#[cfg(feature = "ast2700-bootmcu")]
pub fn ast2700_fmc_dma_read_sync(flash_offset: usize, dst: usize, len: usize) -> bool {
    const FMC_IRQ_CTRL: usize = 0x008;
    const FMC_DMA_CTRL: usize = 0x080;
    const FMC_DMA_FLASH_ADDR: usize = 0x084;
    const FMC_DMA_RAM_ADDR: usize = 0x088;
    const FMC_DMA_LEN: usize = 0x08c;
    const DMA_STATUS: u32 = 1 << 11;
    const DMA_TIMEOUT_LOOPS: u32 = 50_000_000;

    if len == 0 || len > 32 * 1024 * 1024 || flash_offset & 3 != 0 || dst & 3 != 0 || len & 3 != 0 {
        return false;
    }

    let mut fmc = unsafe { MmioBlock::new(AST2700_FMC_BASE) };

    fmc.write32(FMC_IRQ_CTRL, DMA_STATUS);
    // The FMC DMA engine takes BYTE addresses: a flash-relative byte offset for
    // the source and a byte physical DRAM address for the destination. A `>> 2`
    // word-scaling (and folding in the XIP window base) is wrong — so scaled,
    // the DMA transfers nothing. Byte addressing was confirmed on the identical
    // AST2700 FMC IP from the CA35 side; see FMC_SPI_HANDOFF.md.
    fmc.write32(FMC_DMA_FLASH_ADDR, flash_offset as u32);
    fmc.write32(FMC_DMA_RAM_ADDR, dst as u32);
    fmc.write32(FMC_DMA_LEN, (len as u32).saturating_sub(1));
    fmc.write32(FMC_DMA_CTRL, 1);

    let ok = aspeed_mmio::poll_until(
        || fmc.read32(FMC_IRQ_CTRL),
        |v| v & DMA_STATUS != 0,
        DMA_TIMEOUT_LOOPS,
    ).is_ok();

    if ok {
        fmc.write32(FMC_DMA_CTRL, 0);
        fmc.write32(FMC_IRQ_CTRL, DMA_STATUS);
    } else {
        fmc.write32(FMC_DMA_CTRL, 0);
    }
    ok
}

// ── Controller enum ───────────────────────────────────────────────────────────

/// SPI flash controller instance.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg(feature = "ast1060")]
pub enum Controller {
    Fmc,
    Spi1,
    Spi2,
}

#[cfg(feature = "ast1060")]
impl Controller {
    fn flash_window(self) -> usize {
        match self {
            Controller::Fmc => 0x0000_0000,
            Controller::Spi1 => 0x9000_0000,
            Controller::Spi2 => 0xB000_0000,
        }
    }
}

// ── Internal PAC dispatch ─────────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
enum Ctrl {
    Fmc(pac::fmc_v1::FMC),
    Spi(pac::fmc_v1::SPI),
}

#[cfg(feature = "ast1060")]
impl Ctrl {
    fn from(c: Controller) -> Self {
        match c {
            Controller::Fmc => Ctrl::Fmc(pac::FMC),
            Controller::Spi1 => Ctrl::Spi(pac::SPI1),
            Controller::Spi2 => Ctrl::Spi(pac::SPI2),
        }
    }

    fn with_ce_ctrl<F>(&self, ce_idx: u8, f: F)
    where
        F: FnOnce(&dyn CeCtrl),
    {
        match self {
            Ctrl::Fmc(r) => {
                let proxy = FmcCeProxy { r: *r, ce_idx };
                f(&proxy);
            }
            Ctrl::Spi(r) => {
                let proxy = SpiCeProxy { r: *r, ce_idx };
                f(&proxy);
            }
        }
    }

    fn dma_done(&self) -> bool {
        match self {
            Ctrl::Fmc(r) => r.IRQ_CTRL().read().DMA_STATUS(),
            Ctrl::Spi(r) => r.IRQ_CTRL().read().DMA_STATUS(),
        }
    }

    fn clear_dma_done(&self) {
        match self {
            Ctrl::Fmc(r) => r.IRQ_CTRL().write(|w| w.set_DMA_STATUS(true)),
            Ctrl::Spi(r) => r.IRQ_CTRL().write(|w| w.set_DMA_STATUS(true)),
        }
    }

    fn fmc_dma_start(&self, flash_addr: u32, ram_addr: u32, len_minus1: u32) {
        if let Ctrl::Fmc(r) = self {
            r.DMA_FLASH_ADDR()
                .write(|w| w.set_FLASH_ADDR(flash_addr >> 2));
            r.DMA_RAM_ADDR().write(|w| w.set_DRAM_ADDR(ram_addr >> 2));
            r.DMA_LEN().write(|w| w.set_DMA_LEN(len_minus1));
            r.DMA_CTRL().write(|w| {
                w.set_DMA_ENABLE(true);
                w.set_DMA_DIR(false);
            });
        }
    }

    fn fmc_dma_stop(&self) {
        if let Ctrl::Fmc(r) = self {
            r.DMA_CTRL().write(|w| w.set_DMA_ENABLE(false));
        }
    }

    fn is_fmc(&self) -> bool {
        matches!(self, Ctrl::Fmc(_))
    }
}

// ── CE control proxy trait ────────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
trait CeCtrl {
    fn read_ctrl(&self) -> u32;
    fn write_ctrl(&self, val: u32);
}

#[cfg(feature = "ast1060")]
struct FmcCeProxy {
    r: pac::fmc_v1::FMC,
    ce_idx: u8,
}
#[cfg(feature = "ast1060")]
struct SpiCeProxy {
    r: pac::fmc_v1::SPI,
    ce_idx: u8,
}

#[cfg(feature = "ast1060")]
impl CeCtrl for FmcCeProxy {
    fn read_ctrl(&self) -> u32 {
        if self.ce_idx == 0 { self.r.CE0_CTRL().read().0 } else { self.r.CE1_CTRL().read().0 }
    }
    fn write_ctrl(&self, val: u32) {
        if self.ce_idx == 0 {
            self.r.CE0_CTRL().write(|w| { w.0 = val; });
        } else {
            self.r.CE1_CTRL().write(|w| { w.0 = val; });
        }
    }
}

#[cfg(feature = "ast1060")]
impl CeCtrl for SpiCeProxy {
    fn read_ctrl(&self) -> u32 {
        if self.ce_idx == 0 { self.r.CE0_CTRL().read().0 } else { self.r.CE1_CTRL().read().0 }
    }
    fn write_ctrl(&self, val: u32) {
        if self.ce_idx == 0 {
            self.r.CE0_CTRL().write(|w| { w.0 = val; });
        } else {
            self.r.CE1_CTRL().write(|w| { w.0 = val; });
        }
    }
}

// ── SPI_CE_N_CTRL bit positions ────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
const CMD_AUTO_READ: u32 = 0;
#[cfg(feature = "ast1060")]
const CMD_NORMAL_WRITE: u32 = 2;
#[cfg(feature = "ast1060")]
const CMD_USER: u32 = 3;
#[cfg(feature = "ast1060")]
const CMD_MODE_MASK: u32 = 0b11;
#[cfg(feature = "ast1060")]
const CE_STOP_BIT: u32 = 1 << 2;
#[cfg(feature = "ast1060")]
const SPI_CMD_SHIFT: u32 = 16;
#[cfg(feature = "ast1060")]
const SPI_CMD_MASK: u32 = 0xFF << SPI_CMD_SHIFT;

// ── SPI flash commands ────────────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
const CMD_WREN: u8 = 0x06;
#[cfg(feature = "ast1060")]
const CMD_RDSR: u8 = 0x05;
#[cfg(feature = "ast1060")]
const CMD_PP: u8 = 0x02;
#[cfg(feature = "ast1060")]
const CMD_SE: u8 = 0x20;
#[cfg(feature = "ast1060")]
const CMD_BE: u8 = 0xD8;
#[cfg(feature = "ast1060")]
const CMD_JEDEC: u8 = 0x9F;
#[cfg(feature = "ast1060")]
const CMD_RSTEN: u8 = 0x66;
#[cfg(feature = "ast1060")]
const CMD_RST: u8 = 0x99;
#[cfg(feature = "ast1060")]
const SR_WIP: u8 = 1 << 0;

#[cfg(feature = "ast1060")]
const WIP_POLL_US: u64 = 500;
#[cfg(feature = "ast1060")]
const DMA_POLL_US: u64 = 100;

// ── SpiBus ────────────────────────────────────────────────────────────────────

/// SPI flash bus driver for one chip-enable on one controller.
#[cfg(feature = "ast1060")]
pub struct SpiBus {
    ctrl: Ctrl,
    ce_idx: u8,
    /// Flash memory-mapped window base address.
    /// `MmioBlock` byte-width accessors are used for XIP window access
    /// (User-Mode SPI clocking and Auto-Read reads).
    win: usize,
}

#[cfg(feature = "ast1060")]
impl SpiBus {
    /// Create a bus instance for the given controller and chip-enable (0 or 1).
    pub fn new(controller: Controller, ce: u8) -> Self {
        assert!(ce <= 1, "CE must be 0 or 1");
        Self {
            win: controller.flash_window(),
            ctrl: Ctrl::from(controller),
            ce_idx: ce,
        }
    }

    // ── User-mode SPI transfer ────────────────────────────────────────────────
    //
    // In User-Mode the flash window acts as a data port: each volatile byte
    // write clocks one SPI byte out (MOSI), each byte read clocks one in
    // (MISO).  MmioBlock.write8/read8 provide safe volatile access.

    fn user_transfer(&self, cmd: u8, tx: &[u8], rx: &mut [u8]) {
        self.ctrl.with_ce_ctrl(self.ce_idx, |ce| {
            let saved = ce.read_ctrl();
            ce.write_ctrl((saved & !CMD_MODE_MASK) | CMD_USER);

            let mut win = unsafe { MmioBlock::new(self.win) };
            win.write8(0, cmd);
            for &b in tx {
                win.write8(0, b);
            }
            for slot in rx.iter_mut() {
                win.write8(0, 0xFF); // clock dummy
                *slot = win.read8(0);
            }

            let mid = ce.read_ctrl();
            ce.write_ctrl(mid | CE_STOP_BIT);
            ce.write_ctrl((saved & !CMD_MODE_MASK) | CMD_AUTO_READ);
        });
    }

    fn write_enable(&self) {
        self.user_transfer(CMD_WREN, &[], &mut []);
    }

    fn read_status(&self) -> u8 {
        let mut sr = [0u8];
        self.user_transfer(CMD_RDSR, &[], &mut sr);
        sr[0]
    }

    async fn wait_not_busy_async(&self) {
        loop {
            if self.read_status() & SR_WIP == 0 {
                return;
            }
            Timer::after_micros(WIP_POLL_US).await;
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Synchronous XIP read via byte-level MmioBlock access.
    pub fn read_memory_mapped(&self, offset: u32, buf: &mut [u8]) {
        let win = unsafe { MmioBlock::new(self.win) };
        for (i, slot) in buf.iter_mut().enumerate() {
            *slot = win.read8(offset as usize + i);
        }
    }

    /// Read the JEDEC ID (3 bytes: manufacturer, memory type, capacity).
    pub fn read_jedec_id(&self) -> [u8; 3] {
        let mut id = [0u8; 3];
        self.user_transfer(CMD_JEDEC, &[], &mut id);
        id
    }

    /// Reset the attached SPI NOR using the RSTEN/RST command sequence.
    pub async fn reset_by_command(&self) {
        self.user_transfer(CMD_RSTEN, &[], &mut []);
        self.user_transfer(CMD_RST, &[], &mut []);
        Timer::after_micros(WIP_POLL_US).await;
    }

    /// Erase a 4 KB sector at `offset` (must be 4 KB aligned).
    pub async fn erase_sector(&self, offset: u32) -> Result<(), SpiError> {
        if offset & 0xFFF != 0 {
            return Err(SpiError::Unaligned);
        }
        self.wait_not_busy_async().await;
        self.write_enable();
        self.user_transfer(CMD_SE, &addr_bytes_3(offset), &mut []);
        self.wait_not_busy_async().await;
        Ok(())
    }

    /// Erase a 64 KB block at `offset` (must be 64 KB aligned).
    pub async fn erase_block(&self, offset: u32) -> Result<(), SpiError> {
        if offset & 0xFFFF != 0 {
            return Err(SpiError::Unaligned);
        }
        self.wait_not_busy_async().await;
        self.write_enable();
        self.user_transfer(CMD_BE, &addr_bytes_3(offset), &mut []);
        self.wait_not_busy_async().await;
        Ok(())
    }

    /// Write up to 256 bytes to a page.
    pub async fn write_page(&self, offset: u32, data: &[u8]) -> Result<(), SpiError> {
        if data.is_empty() || data.len() > 256 {
            return Err(SpiError::InvalidLength);
        }
        if ((offset & 0xFF) as usize) + data.len() > 256 {
            return Err(SpiError::PageCrossBoundary);
        }

        self.wait_not_busy_async().await;
        self.write_enable();

        self.ctrl.with_ce_ctrl(self.ce_idx, |ce| {
            let saved = ce.read_ctrl();
            ce.write_ctrl(
                (saved & !CMD_MODE_MASK & !SPI_CMD_MASK)
                    | CMD_NORMAL_WRITE
                    | ((CMD_PP as u32) << SPI_CMD_SHIFT),
            );

            let mut win = unsafe { MmioBlock::new(self.win + offset as usize) };
            for (i, &b) in data.iter().enumerate() {
                win.write8(i, b);
            }

            ce.write_ctrl((saved & !CMD_MODE_MASK) | CMD_AUTO_READ);
        });

        self.wait_not_busy_async().await;
        Ok(())
    }

    /// DMA-based read from flash to SRAM.  **FMC only.**
    pub async fn dma_read(
        &self,
        flash_offset: u32,
        sram_addr: u32,
        len: u32,
    ) -> Result<(), SpiError> {
        if !self.ctrl.is_fmc() {
            return Err(SpiError::NotSupported);
        }
        if len == 0 {
            return Err(SpiError::InvalidLength);
        }
        if flash_offset & 3 != 0 || sram_addr & 3 != 0 || len & 3 != 0 {
            return Err(SpiError::Unaligned);
        }

        self.ctrl.clear_dma_done();

        let flash_phys = (self.win as u32) + flash_offset;
        self.ctrl
            .fmc_dma_start(flash_phys, sram_addr, len.saturating_sub(1));

        loop {
            if self.ctrl.dma_done() {
                self.ctrl.fmc_dma_stop();
                self.ctrl.clear_dma_done();
                return Ok(());
            }
            Timer::after_micros(DMA_POLL_US).await;
        }
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
fn addr_bytes_3(offset: u32) -> [u8; 3] {
    [(offset >> 16) as u8, (offset >> 8) as u8, offset as u8]
}

// ── Error type ────────────────────────────────────────────────────────────────

/// SPI flash operation error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg(feature = "ast1060")]
pub enum SpiError {
    Unaligned,
    InvalidLength,
    PageCrossBoundary,
    WriteProtected,
    NotSupported,
}
