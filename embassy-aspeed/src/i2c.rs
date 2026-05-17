//! I2C/SMBus master driver (AST1060, 14 channels).
#![allow(dead_code)]
//!
//! Uses the AST1060 **new register mode** (I2CG0C[2]=1) with pool-buffer
//! transfer mode for simplicity.  DMA and slave mode are not implemented.
//!
//! # Hardware setup
//!
//! | Channel N | Base address | IRQ |
//! |-----------|-------------|-----|
//! | 0 | 0x7E7B_0080 | 110 |
//! | 1 | 0x7E7B_0100 | 111 |
//! | … | … | … |
//! | 13 | 0x7E7B_0700 | 123 |
//!
//! Global registers at `0x7E7B_0000` (I2C_GLOBAL).
//! Pool buffers at `0x7E7B_0C00 + N * 0x20`.
//!
//! # Initialisation sequence
//!
//! 1. Enable new register mode (`I2CG0C[2]=1`) and new clock divider
//!    mode (`I2CG0C[1]=1`).
//! 2. Configure per-channel AC timing in `I2CC04`.
//! 3. Enable master function (`I2CC00[0]=1`).
//!
//! # Transfer model (pool buffer)
//!
//! **Write:** Load target address and data into MASTER_CMD and pool buffer,
//! trigger `MASTER_START_CMD`, wait for `PKT_CMD_DONE_STS` interrupt.
//!
//! **Read:** Configure pool buffer for RX, trigger master start with
//! `ENBL_MASTER_PKT_OP`, wait for done interrupt.
//!
//! # Clock configuration
//!
//! Default (new clock divider mode, SCU310[11:8]=0, PCLK=500 MHz after PLL):
//! - Base clock: 0100 = 1 MHz `baseclk4`
//! - tCKLow = tCKHigh = 5 → SCL = 1 MHz / (5+5) = **100 kHz (Standard Mode)**
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::i2c::{I2cBus, I2cConfig};
//! use embedded_hal_async::i2c::I2c;
//!
//! let mut bus = I2cBus::new(0, I2cConfig::default()); // channel 0
//! let mut buf = [0u8; 2];
//! bus.write_read(0x50, &[0x00], &mut buf).await.unwrap();
//! ```

use core::ptr;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal_async::i2c::{I2c, Operation};

// ── Base addresses ────────────────────────────────────────────────────────────

const I2C_GLOBAL_BASE: usize = 0x7E7B_0000;
const I2C_CH_BASE: usize = 0x7E7B_0080;
const I2C_CH_STRIDE: usize = 0x80;
const I2C_BUF_BASE: usize = 0x7E7B_0C00;
const I2C_BUF_STRIDE: usize = 0x20;

// ── Global register offsets (word index) ─────────────────────────────────────

const G_CTRL: usize = 0x0C / 4; // I2CG0C — global control

// ── Per-channel register offsets (word index from channel base) ───────────────

const C_FUNC_CTRL: usize = 0x00 / 4; // I2CC00
const C_CLK_TIMING: usize = 0x04 / 4; // I2CC04
const C_TX_RX_BUF: usize = 0x08 / 4; // I2CC08 (status + byte buf)
const C_POOL_CTRL: usize = 0x0C / 4; // I2CC0C
const M_IRQ_CTRL: usize = 0x10 / 4; // I2CM10
const M_IRQ_STATUS: usize = 0x14 / 4; // I2CM14
const M_CMD: usize = 0x18 / 4; // I2CM18

// ── Register bit definitions ──────────────────────────────────────────────────

// I2CG0C bits
const G_CLK_DIV_MODE: u32 = 1 << 1; // new clock divider mode
const G_REG_MODE: u32 = 1 << 2; // new register mode

// I2CC00 (FUNC_CTRL) bits
const FUNC_MASTER_EN: u32 = 1 << 0;

// I2CC04 (CLK_TIMING) fields
// New mode, baseclk4 = 1 MHz (BASE_CLK_DIV=0b0100), tCKLow=tCKHigh=5
// → 100 kHz Standard Mode
const CLK_100KHZ: u32 = (0b0100) | (5 << 12) | (5 << 16); // BASE=4, LCNT=5, HCNT=5

// I2CC0C (POOL_CTRL) — split: lower 16 B TX, upper 16 B RX
const POOL_SPLIT: u32 = 1 << 0;

// I2CM10 (MASTER_IRQ_CTRL)
const M_IRQ_PKT_DONE: u32 = 1 << 16;
const M_IRQ_SMBUS_ALERT: u32 = 1 << 12;

// I2CM14 (MASTER_IRQ_STATUS) — write-1-to-clear
const M_STS_PKT_DONE: u32 = 1 << 16;
const M_STS_PKT_FAIL: u32 = 1 << 17;
const M_STS_ARB_LOSS: u32 = 1 << 3;
const M_STS_NACK: u32 = 1 << 1;

// I2CM18 (MASTER_CMD)
const M_CMD_START: u32 = 1 << 0;
const M_CMD_TX: u32 = 1 << 1;
const M_CMD_RX: u32 = 1 << 3;
const M_CMD_RX_LAST: u32 = 1 << 4; // NACK after last byte
const M_CMD_STOP: u32 = 1 << 5;
const M_CMD_TX_POOL: u32 = 1 << 6;
const M_CMD_RX_POOL: u32 = 1 << 7;
const M_CMD_PKT_OP: u32 = 1 << 16;

const M_CMD_TARGET_SHIFT: u32 = 24;

// Pool buffer max size in bytes (32-byte SRAM, split → 16 TX + 16 RX)
const POOL_MAX_BYTES: usize = 16;

// ── Global wakers (one per channel, 14 channels) ──────────────────────────────

const N_CHANNELS: usize = 14;
static I2C_WAKERS: [AtomicWaker; N_CHANNELS] = {
    // AtomicWaker is not Copy, so we need a const fn array
    const W: AtomicWaker = AtomicWaker::new();
    [W, W, W, W, W, W, W, W, W, W, W, W, W, W]
};

// ── Config ────────────────────────────────────────────────────────────────────

/// I2C bus configuration.
pub struct I2cConfig {
    /// SCL frequency — see CLK_TIMING register for formula.
    /// Default: standard mode (100 kHz) using 1 MHz base clock.
    pub clk_timing: u32,
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self {
            clk_timing: CLK_100KHZ,
        }
    }
}

// ── I2cBus ────────────────────────────────────────────────────────────────────

/// AST1060 I2C master bus driver (single channel, pool-buffer mode).
pub struct I2cBus {
    ch: u8,
}

impl I2cBus {
    /// Initialise I2C channel `ch` (0–13) with the given configuration.
    ///
    /// Enables new register mode globally on first call (idempotent).
    ///
    /// # Panics
    ///
    /// Panics if `ch` ≥ 14.
    pub fn new(ch: u8, cfg: I2cConfig) -> Self {
        assert!((ch as usize) < N_CHANNELS, "I2C channel must be 0-13");

        // Enable new register mode and new clock divider mode globally.
        let gr = global_base();
        let gctrl = unsafe { ptr::read_volatile(gr.add(G_CTRL)) };
        unsafe { ptr::write_volatile(gr.add(G_CTRL), gctrl | G_REG_MODE | G_CLK_DIV_MODE) };

        // Configure channel.
        let cr = ch_base(ch);
        unsafe {
            // Disable everything before config.
            ptr::write_volatile(cr.add(C_FUNC_CTRL), 0);
            // Set clock timing.
            ptr::write_volatile(cr.add(C_CLK_TIMING), cfg.clk_timing);
            // Enable master function.
            ptr::write_volatile(cr.add(C_FUNC_CTRL), FUNC_MASTER_EN);
            // Enable packet-done interrupt.
            ptr::write_volatile(cr.add(M_IRQ_CTRL), M_IRQ_PKT_DONE | M_IRQ_SMBUS_ALERT);
        }

        Self { ch }
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    fn ch_regs(&self) -> *mut u32 {
        ch_base(self.ch)
    }

    fn buf_base(&self) -> *mut u8 {
        (I2C_BUF_BASE + self.ch as usize * I2C_BUF_STRIDE) as *mut u8
    }

    fn rr(&self, off: usize) -> u32 {
        unsafe { ptr::read_volatile(self.ch_regs().add(off)) }
    }

    fn rw(&self, off: usize, val: u32) {
        unsafe { ptr::write_volatile(self.ch_regs().add(off), val) }
    }

    fn clear_status(&self) {
        // Write 1 to all W1C bits.
        self.rw(M_IRQ_STATUS, 0xFFFF_FFFF);
    }

    fn wait_bus_idle(&self) {
        // Bit[16] of I2CC08 = BUS_BUSY.
        while self.rr(C_TX_RX_BUF) & (1 << 16) != 0 {}
    }

    // ── Blocking transfer primitives ──────────────────────────────────────────

    /// Blocking write: send `addr` (7-bit), then `data`.
    fn blocking_write_inner(&mut self, addr: u8, data: &[u8]) -> Result<(), I2cError> {
        let n = data.len().min(POOL_MAX_BYTES);
        self.wait_bus_idle();
        self.clear_status();

        // Load data into pool TX buffer.
        let buf = self.buf_base();
        for (i, &b) in data[..n].iter().enumerate() {
            unsafe { ptr::write_volatile(buf.add(i), b) };
        }

        // Pool control: all 32 B for TX (no split), TX count = n-1.
        let pool_ctrl = ((n.saturating_sub(1) as u32) << 8) & (0x1F << 8);
        self.rw(C_POOL_CTRL, pool_ctrl);

        // Issue packet-mode write: target addr + TX pool + start.
        let cmd = M_CMD_TX_POOL | M_CMD_PKT_OP | ((addr as u32) << M_CMD_TARGET_SHIFT);
        self.rw(M_CMD, cmd);

        self.poll_done()
    }

    /// Blocking read: send `addr` (7-bit read), receive `buf.len()` bytes.
    fn blocking_read_inner(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        let n = buf.len().min(POOL_MAX_BYTES);
        self.wait_bus_idle();
        self.clear_status();

        // Pool control: RX count field = n-1 at bits [20:16].
        // Note: M_CMD_RX_LAST (NACK after last byte) lives in M_CMD[4], not here.
        // The `(1 << 4)` that was previously ORed in was a copy-paste from the
        // M_CMD bit definitions and wrote an unrelated pool-ctrl field.
        let pool_ctrl = ((n.saturating_sub(1) as u32) << 16) & (0x1F << 16);
        self.rw(C_POOL_CTRL, pool_ctrl);

        // Issue packet-mode read.
        let cmd =
            M_CMD_RX_POOL | M_CMD_PKT_OP | M_CMD_RX_LAST | ((addr as u32) << M_CMD_TARGET_SHIFT);
        self.rw(M_CMD, cmd | (1 << 28)); // RnW=1 for read

        self.poll_done()?;

        // Copy received bytes from pool buffer.
        let rbuf = self.buf_base();
        for (i, b) in buf[..n].iter_mut().enumerate() {
            *b = unsafe { ptr::read_volatile(rbuf.add(i)) };
        }
        Ok(())
    }

    fn poll_done(&self) -> Result<(), I2cError> {
        // Spin-wait for packet done or fail. For async, use the waker in the future.
        loop {
            let sts = self.rr(M_IRQ_STATUS);
            if sts & M_STS_PKT_DONE != 0 {
                self.rw(M_IRQ_STATUS, M_STS_PKT_DONE);
                if sts & M_STS_PKT_FAIL != 0 {
                    return Err(I2cError::Nack);
                }
                return Ok(());
            }
            if sts & (M_STS_ARB_LOSS | M_STS_NACK) != 0 {
                self.rw(M_IRQ_STATUS, 0xFFFF_FFFF);
                return Err(I2cError::Nack);
            }
            core::hint::spin_loop();
        }
    }

    /// Called from ISR for channel `ch` (0-based).
    pub(crate) fn on_interrupt(ch: u8) {
        if (ch as usize) < N_CHANNELS {
            I2C_WAKERS[ch as usize].wake();
        }
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

/// I2C error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum I2cError {
    /// Device did not acknowledge (address or data NACK).
    Nack,
    /// Arbitration lost (another master on the bus).
    ArbitrationLoss,
    /// Transfer too large for pool buffer (max 16 bytes in split mode).
    BufferOverflow,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            I2cError::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            ),
            I2cError::ArbitrationLoss => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            I2cError::BufferOverflow => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl embedded_hal::i2c::ErrorType for I2cBus {
    type Error = I2cError;
}

// ── embedded_hal_async::i2c::I2c ─────────────────────────────────────────────

impl I2c for I2cBus {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), I2cError> {
        if read.len() > POOL_MAX_BYTES {
            return Err(I2cError::BufferOverflow);
        }
        // Current implementation uses blocking poll internally.
        // Async waker support can be added once interrupt routing is validated.
        self.blocking_read_inner(address, read)
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), I2cError> {
        if write.len() > POOL_MAX_BYTES {
            return Err(I2cError::BufferOverflow);
        }
        self.blocking_write_inner(address, write)
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), I2cError> {
        if write.len() > POOL_MAX_BYTES || read.len() > POOL_MAX_BYTES {
            return Err(I2cError::BufferOverflow);
        }
        // Write phase (send address + data, no stop).
        self.blocking_write_inner(address, write)?;
        // Read phase (repeated start).
        self.blocking_read_inner(address, read)
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), I2cError> {
        for op in operations.iter_mut() {
            match op {
                Operation::Read(buf) => self.blocking_read_inner(address, buf)?,
                Operation::Write(buf) => self.blocking_write_inner(address, buf)?,
            }
        }
        Ok(())
    }
}

// ── Address helpers ───────────────────────────────────────────────────────────

fn global_base() -> *mut u32 {
    I2C_GLOBAL_BASE as *mut u32
}

fn ch_base(ch: u8) -> *mut u32 {
    (I2C_CH_BASE + ch as usize * I2C_CH_STRIDE) as *mut u32
}

// ── Interrupt handlers ────────────────────────────────────────────────────────

macro_rules! i2c_irq {
    ($name:ident, $ch:expr) => {
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn $name() {
            I2cBus::on_interrupt($ch);
        }
    };
}

i2c_irq!(I2C0, 0);
i2c_irq!(I2C1, 1);
i2c_irq!(I2C2, 2);
i2c_irq!(I2C3, 3);
i2c_irq!(I2C4, 4);
i2c_irq!(I2C5, 5);
i2c_irq!(I2C6, 6);
i2c_irq!(I2C7, 7);
i2c_irq!(I2C8, 8);
i2c_irq!(I2C9, 9);
i2c_irq!(I2C10, 10);
i2c_irq!(I2C11, 11);
i2c_irq!(I2C12, 12);
i2c_irq!(I2C13, 13);
