//! AST2700 eMMC boot backend bring-up hooks.
//!
//! # Sync and async
//!
//! Both sync (blocking) and async (yielding) APIs are provided:
//!
//! - `Emmc::init()` / `Emmc::init_async()` — initialise and identify the eMMC device
//! - `Emmc::copy()` / `Emmc::copy_async()` — copy bytes from eMMC to DRAM
//!
//! Async variants use `poll_until_async` to yield to the Embassy executor
//! while waiting for SDHCI command/data completion.

use crate::pac;
use aspeed_mmio::MmioBlock;

const SCU0_BASE: usize = 0x12c0_2000;
const EMMC_PIN_RELEASE: usize = 0x12c0_b00c;
const GPIO18_PINMUX: usize = 0x12c0_2400;
const EMMC_BASE: usize = 0x1209_0000;
const MMC_CLK_DRIVING_REG: usize = SCU0_BASE + 0x480;
const MMC_CMD_DRIVING_REG: usize = SCU0_BASE + 0x484;
const MMC_DAT0_DRIVING_REG: usize = SCU0_BASE + 0x488;
const MMC_DAT1_DRIVING_REG: usize = SCU0_BASE + 0x48c;
const MMC_DAT2_DRIVING_REG: usize = SCU0_BASE + 0x490;
const MMC_DAT3_DRIVING_REG: usize = SCU0_BASE + 0x494;
const SCU0_CLKGATE1_EMMC: u32 = 1 << 27;
const SCU0_RST1_EMMC: u32 = 1 << 17;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    NoController,
    Timeout,
    CommandFailed,
}

// ── SDHCI register offsets ────────────────────────────────────────────────────

const SDHCI_BUFFER: usize = 0x20;
const SDHCI_PRESENT_STATE: usize = 0x24;
const SDHCI_HOST_CTRL: usize = 0x28;
const SDHCI_POWER_CTRL: usize = 0x29;
const SDHCI_BLOCK_SIZE: usize = 0x04;
const SDHCI_BLOCK_COUNT: usize = 0x06;
const SDHCI_ARGUMENT: usize = 0x08;
const SDHCI_TRANSFER_MODE: usize = 0x0c;
const SDHCI_COMMAND: usize = 0x0e;
const SDHCI_RESPONSE: usize = 0x10;
const SDHCI_CLOCK_CONTROL: usize = 0x2c;
const SDHCI_TIMEOUT_CONTROL: usize = 0x2e;
const SDHCI_SOFTWARE_RESET: usize = 0x2f;
const SDHCI_INT_STATUS: usize = 0x30;
const SDHCI_INT_ENABLE: usize = 0x34;
const SDHCI_SIGNAL_ENABLE: usize = 0x38;
const PRESENT_CMD_INHIBIT: u32 = 1 << 0;
const PRESENT_DAT_INHIBIT: u32 = 1 << 1;
const INT_CMD_COMPLETE: u32 = 1 << 0;
const INT_XFER_COMPLETE: u32 = 1 << 1;
const INT_BUF_READ_READY: u32 = 1 << 5;
const INT_ERROR: u32 = 0xffff_0000;
const CMD_RESP_136: u16 = 0x01;
const CMD_RESP_48: u16 = 0x02;
const CMD_RESP_48_BUSY: u16 = 0x03;
const CMD_CRC: u16 = 0x08;
const CMD_INDEX: u16 = 0x10;
const CMD_DATA: u16 = 0x20;
const XFER_BLOCK_COUNT_EN: u16 = 1 << 1;
const XFER_READ: u16 = 1 << 4;
const MMC_BLOCK_LEN: usize = 512;
const POLL_LOOPS: u32 = 1_000_000;

#[inline]
fn emmc() -> MmioBlock {
    unsafe { MmioBlock::new(EMMC_BASE) }
}

pub struct Emmc {
    rca: u16,
}

impl Emmc {
    // ── Sync API ──────────────────────────────────────────────────────────

    pub fn init() -> Result<Self, Error> {
        Self::setup_clocks_and_pins();
        Self::check_controller_present()?;
        let mut dev = Self { rca: 0 };
        dev.reset_host()?;
        dev.power_on();
        dev.identify()?;
        Ok(dev)
    }

    /// Copy bytes from eMMC flash to a DRAM destination.
    ///
    /// # Safety
    ///
    /// `dst` must be a valid, writable pointer to at least `len` bytes. No
    /// bounds checking is performed; passing an invalid or unaligned address
    /// will produce undefined behaviour.
    pub unsafe fn copy(&self, dst: usize, src: u32, len: usize) -> Result<(), Error> {
        let mut out = dst as *mut u8;
        let mut offset = src as usize;
        let mut remaining = len;
        let mut scratch = [0u8; MMC_BLOCK_LEN];
        while remaining != 0 {
            let lba = offset / MMC_BLOCK_LEN;
            let block_offset = offset % MMC_BLOCK_LEN;
            self.read_block(lba as u32, &mut scratch)?;
            let n = (MMC_BLOCK_LEN - block_offset).min(remaining);
            unsafe {
                core::ptr::copy_nonoverlapping(scratch[block_offset..].as_ptr(), out, n);
                out = out.add(n);
            }
            offset += n;
            remaining -= n;
        }
        Ok(())
    }

    // ── Async API ─────────────────────────────────────────────────────────

    pub async fn init_async() -> Result<Self, Error> {
        Self::setup_clocks_and_pins();
        Self::check_controller_present()?;
        let mut dev = Self { rca: 0 };
        dev.reset_host_async().await?;
        dev.power_on();
        dev.identify_async().await?;
        Ok(dev)
    }

    /// Copy bytes from eMMC flash to a DRAM destination (async variant).
    ///
    /// # Safety
    ///
    /// `dst` must be a valid, writable pointer to at least `len` bytes. No
    /// bounds checking is performed.
    pub async unsafe fn copy_async(&self, dst: usize, src: u32, len: usize) -> Result<(), Error> {
        let mut out = dst as *mut u8;
        let mut offset = src as usize;
        let mut remaining = len;
        let mut scratch = [0u8; MMC_BLOCK_LEN];
        while remaining != 0 {
            let lba = offset / MMC_BLOCK_LEN;
            let block_offset = offset % MMC_BLOCK_LEN;
            self.read_block_async(lba as u32, &mut scratch).await?;
            let n = (MMC_BLOCK_LEN - block_offset).min(remaining);
            unsafe {
                core::ptr::copy_nonoverlapping(scratch[block_offset..].as_ptr(), out, n);
                out = out.add(n);
            }
            offset += n;
            remaining -= n;
        }
        Ok(())
    }

    // ── Shared init helpers ───────────────────────────────────────────────

    fn setup_clocks_and_pins() {
        pac::SCU0
            .CLKGATE1_CLR()
            .write_value(pac::scu0_ast2700_v1::CLKGATE1(SCU0_CLKGATE1_EMMC));
        pac::SCU0
            .RST_CLR1()
            .write_value(pac::scu0_ast2700_v1::RST_CTRL1(SCU0_RST1_EMMC));

        let mut scu0 = unsafe { MmioBlock::new(SCU0_BASE) };
        scu0.write32(MMC_CLK_DRIVING_REG - SCU0_BASE, 2);
        scu0.write32(MMC_CMD_DRIVING_REG - SCU0_BASE, 1);
        scu0.write32(MMC_DAT0_DRIVING_REG - SCU0_BASE, 1);
        scu0.write32(MMC_DAT1_DRIVING_REG - SCU0_BASE, 1);
        scu0.write32(MMC_DAT2_DRIVING_REG - SCU0_BASE, 1);
        scu0.write32(MMC_DAT3_DRIVING_REG - SCU0_BASE, 1);

        let mut pin = unsafe { MmioBlock::new(EMMC_PIN_RELEASE) };
        pin.write32(0, 0);

        let mut gpio = unsafe { MmioBlock::new(GPIO18_PINMUX) };
        gpio.write32(0, 0xff);
    }

    fn check_controller_present() -> Result<(), Error> {
        let v = emmc().read16(0xfe);
        if v == 0 || v == 0xffff {
            Err(Error::NoController)
        } else {
            Ok(())
        }
    }

    // ── Sync internals ────────────────────────────────────────────────────

    fn reset_host(&mut self) -> Result<(), Error> {
        let mut regs = emmc();
        regs.write8(SDHCI_SOFTWARE_RESET, 0x07);
        self.wait8_clear(SDHCI_SOFTWARE_RESET, 0x07)?;
        let mut regs = emmc();
        regs.write32(SDHCI_INT_STATUS, 0xffff_ffff);
        regs.write32(SDHCI_INT_ENABLE, 0xffff_ffff);
        regs.write32(SDHCI_SIGNAL_ENABLE, 0);
        regs.write8(SDHCI_TIMEOUT_CONTROL, 0x0e);
        Ok(())
    }

    fn power_on(&self) {
        let mut regs = emmc();
        regs.write8(SDHCI_POWER_CTRL, 0x0f);
        regs.write16(SDHCI_CLOCK_CONTROL, 0x0007);
        regs.write8(SDHCI_HOST_CTRL, 0x02);
    }

    fn identify(&mut self) -> Result<(), Error> {
        self.cmd(0, 0, 0)?;
        let mut ready = false;
        for _ in 0..1000 {
            let ocr = self.cmd(1, 0x40ff_8080, CMD_RESP_48)?;
            if ocr & (1 << 31) != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            return Err(Error::Timeout);
        }
        self.cmd(2, 0, CMD_RESP_136 | CMD_CRC)?;
        let rca = self.cmd(3, 0x0001_0000, CMD_RESP_48 | CMD_CRC | CMD_INDEX)?;
        self.rca = if rca >> 16 != 0 { (rca >> 16) as u16 } else { 1 };
        self.cmd(7, (self.rca as u32) << 16, CMD_RESP_48_BUSY | CMD_CRC | CMD_INDEX)?;
        self.cmd(16, MMC_BLOCK_LEN as u32, CMD_RESP_48 | CMD_CRC | CMD_INDEX)?;
        Ok(())
    }

    fn read_block(&self, lba: u32, out: &mut [u8; MMC_BLOCK_LEN]) -> Result<(), Error> {
        self.wait_present_clear(PRESENT_CMD_INHIBIT | PRESENT_DAT_INHIBIT)?;
        let mut regs = emmc();
        regs.write16(SDHCI_BLOCK_SIZE, MMC_BLOCK_LEN as u16);
        regs.write16(SDHCI_BLOCK_COUNT, 1);
        regs.write16(SDHCI_TRANSFER_MODE, XFER_BLOCK_COUNT_EN | XFER_READ);
        regs.write32(SDHCI_ARGUMENT, lba);
        regs.write32(SDHCI_INT_STATUS, 0xffff_ffff);
        regs.write16(SDHCI_COMMAND, ((17u16) << 8) | CMD_RESP_48 | CMD_CRC | CMD_INDEX | CMD_DATA);
        self.wait_int(INT_CMD_COMPLETE)?;
        self.wait_int(INT_BUF_READ_READY)?;
        let regs = emmc();
        for chunk in out.chunks_mut(4) {
            let word = regs.read32(SDHCI_BUFFER).to_le_bytes();
            chunk.copy_from_slice(&word[..chunk.len()]);
        }
        self.wait_int(INT_XFER_COMPLETE)
    }

    fn cmd(&self, idx: u16, arg: u32, flags: u16) -> Result<u32, Error> {
        self.wait_present_clear(PRESENT_CMD_INHIBIT)?;
        let mut regs = emmc();
        regs.write32(SDHCI_ARGUMENT, arg);
        regs.write32(SDHCI_INT_STATUS, 0xffff_ffff);
        regs.write16(SDHCI_TRANSFER_MODE, 0);
        regs.write16(SDHCI_COMMAND, (idx << 8) | flags);
        self.wait_int(INT_CMD_COMPLETE)?;
        Ok(emmc().read32(SDHCI_RESPONSE))
    }

    fn wait_present_clear(&self, mask: u32) -> Result<(), Error> {
        aspeed_mmio::poll_until(
            || emmc().read32(SDHCI_PRESENT_STATE),
            |v| v & mask == 0,
            POLL_LOOPS,
        ).map(|_| ()).map_err(|_| Error::Timeout)
    }

    fn wait_int(&self, mask: u32) -> Result<(), Error> {
        for _ in 0..POLL_LOOPS {
            let status = emmc().read32(SDHCI_INT_STATUS);
            if status & INT_ERROR != 0 {
                let mut e = emmc();
                e.write32(SDHCI_INT_STATUS, status);
                return Err(Error::CommandFailed);
            }
            if status & mask != 0 {
                let mut e = emmc();
                e.write32(SDHCI_INT_STATUS, mask);
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(Error::Timeout)
    }

    fn wait8_clear(&self, off: usize, mask: u8) -> Result<(), Error> {
        aspeed_mmio::poll_until(
            || emmc().read8(off),
            |v| v & mask == 0,
            POLL_LOOPS,
        ).map(|_| ()).map_err(|_| Error::Timeout)
    }

    // ── Async internals ───────────────────────────────────────────────────

    async fn reset_host_async(&mut self) -> Result<(), Error> {
        let mut e = emmc();
        e.write8(SDHCI_SOFTWARE_RESET, 0x07);
        self.wait8_clear_async(SDHCI_SOFTWARE_RESET, 0x07).await?;
        let mut e = emmc();
        e.write32(SDHCI_INT_STATUS, 0xffff_ffff);
        e.write32(SDHCI_INT_ENABLE, 0xffff_ffff);
        e.write32(SDHCI_SIGNAL_ENABLE, 0);
        e.write8(SDHCI_TIMEOUT_CONTROL, 0x0e);
        Ok(())
    }

    async fn identify_async(&mut self) -> Result<(), Error> {
        self.cmd_async(0, 0, 0).await?;
        let mut ready = false;
        for _ in 0..1000 {
            let ocr = self.cmd_async(1, 0x40ff_8080, CMD_RESP_48).await?;
            if ocr & (1 << 31) != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            return Err(Error::Timeout);
        }
        self.cmd_async(2, 0, CMD_RESP_136 | CMD_CRC).await?;
        let rca = self.cmd_async(3, 0x0001_0000, CMD_RESP_48 | CMD_CRC | CMD_INDEX).await?;
        self.rca = if rca >> 16 != 0 { (rca >> 16) as u16 } else { 1 };
        self.cmd_async(7, (self.rca as u32) << 16, CMD_RESP_48_BUSY | CMD_CRC | CMD_INDEX).await?;
        self.cmd_async(16, MMC_BLOCK_LEN as u32, CMD_RESP_48 | CMD_CRC | CMD_INDEX).await?;
        Ok(())
    }

    async fn read_block_async(
        &self,
        lba: u32,
        out: &mut [u8; MMC_BLOCK_LEN],
    ) -> Result<(), Error> {
        self.wait_present_clear_async(PRESENT_CMD_INHIBIT | PRESENT_DAT_INHIBIT).await?;
        let mut regs = emmc();
        regs.write16(SDHCI_BLOCK_SIZE, MMC_BLOCK_LEN as u16);
        regs.write16(SDHCI_BLOCK_COUNT, 1);
        regs.write16(SDHCI_TRANSFER_MODE, XFER_BLOCK_COUNT_EN | XFER_READ);
        regs.write32(SDHCI_ARGUMENT, lba);
        regs.write32(SDHCI_INT_STATUS, 0xffff_ffff);
        regs.write16(SDHCI_COMMAND, ((17u16) << 8) | CMD_RESP_48 | CMD_CRC | CMD_INDEX | CMD_DATA);
        self.wait_int_async(INT_CMD_COMPLETE).await?;
        self.wait_int_async(INT_BUF_READ_READY).await?;
        let regs = emmc();
        for chunk in out.chunks_mut(4) {
            let word = regs.read32(SDHCI_BUFFER).to_le_bytes();
            chunk.copy_from_slice(&word[..chunk.len()]);
        }
        self.wait_int_async(INT_XFER_COMPLETE).await
    }

    async fn cmd_async(&self, idx: u16, arg: u32, flags: u16) -> Result<u32, Error> {
        self.wait_present_clear_async(PRESENT_CMD_INHIBIT).await?;
        let mut regs = emmc();
        regs.write32(SDHCI_ARGUMENT, arg);
        regs.write32(SDHCI_INT_STATUS, 0xffff_ffff);
        regs.write16(SDHCI_TRANSFER_MODE, 0);
        regs.write16(SDHCI_COMMAND, (idx << 8) | flags);
        self.wait_int_async(INT_CMD_COMPLETE).await?;
        Ok(emmc().read32(SDHCI_RESPONSE))
    }

    async fn wait_present_clear_async(&self, mask: u32) -> Result<(), Error> {
        aspeed_mmio::poll_until_async(
            || emmc().read32(SDHCI_PRESENT_STATE),
            |v| v & mask == 0,
            embassy_time::Duration::from_micros(50),
            embassy_time::Duration::from_millis(500),
        ).await.map(|_| ()).map_err(|_| Error::Timeout)
    }

    async fn wait_int_async(&self, mask: u32) -> Result<(), Error> {
        use embassy_time::{Duration, Instant, Timer};
        const TIMEOUT: Duration = Duration::from_millis(500);
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let status = emmc().read32(SDHCI_INT_STATUS);
            if status & INT_ERROR != 0 {
                let mut e = emmc();
                e.write32(SDHCI_INT_STATUS, status);
                return Err(Error::CommandFailed);
            }
            if status & mask != 0 {
                let mut e = emmc();
                e.write32(SDHCI_INT_STATUS, mask);
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(Error::Timeout);
            }
            Timer::after(Duration::from_micros(50)).await;
        }
    }

    async fn wait8_clear_async(&self, off: usize, mask: u8) -> Result<(), Error> {
        aspeed_mmio::poll_until_async(
            || emmc().read8(off),
            |v| v & mask == 0,
            embassy_time::Duration::from_micros(50),
            embassy_time::Duration::from_millis(100),
        ).await.map(|_| ()).map_err(|_| Error::Timeout)
    }
}
