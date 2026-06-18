//! I2C/SMBus master driver — PAC-based, async ISR-driven.
//!
//! Sources of truth:
//! - `aspeed-data/data/registers/i2c_v1.yaml`  (per-channel registers)
//! - `aspeed-data/data/registers/i2cglobal_v1.yaml` (global control)
//! - `aspeed-data/data/registers/i2cbuff_v1.yaml` (pool-buffer SRAM)
//!
//! # Hardware instances (AST1060)
//!
//! | Channel | PAC | Base | IRQ |
//! |---------|-----|------|-----|
//! | 0 | `pac::I2C0` | `0x7E7B_0080` | 110 |
//! | … | … | … | … |
//! | 13 | `pac::I2C13` | `0x7E7B_0700` | 123 |
//!
//! Global: `pac::I2C_GLOBAL` at `0x7E7B_0000`.
//! Pool SRAM: `0x7E7B_0C00 + N * 0x20` (32 bytes per channel, raw SRAM).
//!
//! # Mode
//!
//! New register mode (`I2CGLOBAL.CTRL.NEW_REG_MODE=1`) with pool-buffer transfer
//! (max 16 bytes per phase in split mode: lower 16 B TX, upper 16 B RX).
//!
//! # Async model
//!
//! - `start_write` / `start_read`: configure pool buffer and issue `MASTER_CMD`,
//!   return immediately.
//! - `wait_done()` → `WaitDoneFuture`: registers `AtomicWaker`, yields until
//!   `MASTER_IRQ_STATUS.PKT_CMD_DONE_STS` fires.
//! - Per-channel ISR (I2C0–I2C13): calls `on_interrupt(ch)` → wakes the waker.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::i2c::{I2cBus, I2cConfig};
//! use embedded_hal_async::i2c::I2c;
//!
//! let mut bus = I2cBus::new(0, I2cConfig::default());
//! let mut buf = [0u8; 2];
//! bus.write_read(0x50, &[0x00], &mut buf).await.unwrap();
//! ```

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use aspeed_mmio::MmioBlock;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal_async::i2c::{I2c, Operation};

use crate::pac;

// ── Pool-buffer SRAM constants ────────────────────────────────────────────────
// i2cbuff_v1.yaml: per-channel pool SRAM at base+0xC00, stride 0x20 (32 bytes).
// Lower 16 bytes = TX, upper 16 bytes = RX when POOL_CTRL.BUF_ORGANIZATION=1.

#[cfg(feature = "ast1060")]
const I2C_BUF_BASE: usize = 0x7E7B_0C00;
#[cfg(feature = "ast2600-ssp")]
const I2C_BUF_BASE: usize = 0x7E78_AC00;
const I2C_BUF_STRIDE: usize = 0x20;

/// Maximum bytes per pool-buffer phase in split mode (16 TX + 16 RX).
const POOL_MAX: usize = 16;

// ── Channel count ──────────────────────────────────────────────────────────────

#[cfg(feature = "ast1060")]
const N_CH: usize = 14;
#[cfg(feature = "ast2600-ssp")]
const N_CH: usize = 16;

// ── Global wakers ──────────────────────────────────────────────────────────────

static I2C_WAKERS: [AtomicWaker; N_CH] = {
    const W: AtomicWaker = AtomicWaker::new();
    #[cfg(feature = "ast1060")]
    {
        [W, W, W, W, W, W, W, W, W, W, W, W, W, W]
    }
    #[cfg(feature = "ast2600-ssp")]
    {
        [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W]
    }
};

// ── Config ────────────────────────────────────────────────────────────────────

/// I2C bus configuration.
pub struct I2cConfig {
    /// `CLK_TIMING` register value for the desired SCL frequency.
    ///
    /// Default: 100 kHz Standard Mode for AST1060 at 500 MHz PCLK.
    /// Formula (new clock divider mode, from i2c_v1.yaml `I2C_CLK_TIMING`):
    ///   `BASE_CLK_DIV=4` (1 MHz base), `TCKLOW=5`, `TCKIGH=5`
    ///   → SCL = 1 MHz / (5+5) = 100 kHz
    pub clk_timing: u32,
}

impl Default for I2cConfig {
    fn default() -> Self {
        // BASE_CLK_DIV=4, TCKLOW=5, TCKIGH=5 (i2c_v1.yaml I2C_CLK_TIMING fieldset).
        let mut t = pac::i2c_v1::I2C_CLK_TIMING(0);
        t.set_BASE_CLK_DIV(4);
        t.set_TCKLOW(5);
        t.set_TCKIGH(5);
        Self { clk_timing: t.0 }
    }
}

// ── I2cBus ────────────────────────────────────────────────────────────────────

/// I2C master bus driver (single channel, pool-buffer mode).
pub struct I2cBus {
    ch: u8,
}

impl I2cBus {
    /// Initialise I2C channel `ch` (0–13 on AST1060, 0–15 on AST2600 SSP).
    ///
    /// Enables new register mode globally and configures the channel.
    ///
    /// # Panics
    ///
    /// Panics if `ch` ≥ channel count for this chip.
    pub fn new(ch: u8, cfg: I2cConfig) -> Self {
        assert!((ch as usize) < N_CH, "I2C channel out of range");

        // Enable new register mode + new clock divider mode.
        // i2cglobal_v1.yaml GLOBAL_CTRL: REG_MODE=bit2, CLK_DIVIDER_MODE=bit1.
        pac::I2C_GLOBAL.GLOBAL_CTRL().modify(|w| {
            w.set_REG_MODE(true);
            w.set_CLK_DIVIDER_MODE(true);
        });

        with_ch(ch, |regs| {
            // Disable channel before configuring (clearing ENBL_MASTER_FN resets state).
            regs.FUNC_CTRL().write(|w| w.set_ENBL_MASTER_FN(false));

            // Set SCL timing.
            regs.CLK_TIMING().write(|w| {
                w.0 = cfg.clk_timing;
            });

            // Enable master function.
            regs.FUNC_CTRL().write(|w| w.set_ENBL_MASTER_FN(true));

            // Enable PKT_CMD_DONE interrupt (required for async WaitDoneFuture).
            // i2c_v1.yaml: MASTER_IRQ_CTRL.ENBL_PKT_CMD_DONE_INT = bit16.
            regs.MASTER_IRQ_CTRL().write(|w| {
                w.set_ENBL_PKT_CMD_DONE_INT(true);
                w.set_ENBL_SMBUS_ALERT_INT(true);
            });
        });

        Self { ch }
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    fn buf_regs(&self) -> MmioBlock {
        unsafe { MmioBlock::new(I2C_BUF_BASE + self.ch as usize * I2C_BUF_STRIDE) }
    }

    /// Spin while BUS_BUSY (i2c_v1.yaml TX_RX_BUF.BUS_BUSY = bit16).
    fn wait_bus_idle(&self) {
        with_ch(self.ch, |r| {
            while r.TX_RX_BUF().read().BUS_BUSY() {
                core::hint::spin_loop();
            }
        });
    }

    fn clear_status(&self) {
        // W1C: write all-ones to clear sticky status bits.
        with_ch(self.ch, |r| {
            r.MASTER_IRQ_STATUS().write(|w| {
                w.0 = 0xFFFF_FFFF;
            });
        });
    }

    // ── Non-blocking transfer initiation ──────────────────────────────────────

    fn start_write(&mut self, addr: u8, data: &[u8]) {
        let n = data.len().min(POOL_MAX);
        self.wait_bus_idle();
        self.clear_status();

        // Write TX bytes into pool SRAM (lower 16 bytes).
        let mut buf = self.buf_regs();
        for (i, &b) in data[..n].iter().enumerate() {
            buf.write8(i, b);
        }

        with_ch(self.ch, |r| {
            // POOL_CTRL: TX only (BUF_ORGANIZATION=0), TX_DATA_BYTE_COUNT = n-1.
            // i2c_v1.yaml: POOL_CTRL.TX_DATA_BYTE_COUNT [12:8] = N+1 bytes encoded as N.
            r.POOL_CTRL().write(|w| {
                w.set_BUF_ORGANIZATION(false); // all 32 bytes for TX
                w.set_TX_DATA_BYTE_COUNT((n.saturating_sub(1)) as u8);
            });

            // Issue packet-mode write (i2c_v1.yaml MASTER_CMD fieldset).
            r.MASTER_CMD().write(|w| {
                w.set_ENBL_MASTER_TX_POOL(true);
                w.set_ENBL_MASTER_PKT_OP(true);
                w.set_TARGET_ADDR(addr & 0x7F);
            });
        });
    }

    fn start_read(&mut self, addr: u8, n: usize) {
        self.wait_bus_idle();
        self.clear_status();

        with_ch(self.ch, |r| {
            // POOL_CTRL: RX_POOL_BUF_SIZE at bits[20:16] = n-1.
            r.POOL_CTRL().write(|w| {
                w.set_BUF_ORGANIZATION(false);
                w.set_RX_POOL_BUF_SIZE((n.saturating_sub(1)) as u8);
            });

            // Issue packet-mode read.
            // Direction is determined by ENBL_MASTER_RX_POOL; TARGET_ADDR is 7-bit.
            // i2c_v1.yaml: no separate RnW bit in packet mode — hardware derives it.
            r.MASTER_CMD().write(|w| {
                w.set_ENBL_MASTER_RX_POOL(true);
                w.set_ENBL_MASTER_PKT_OP(true);
                w.set_TARGET_ADDR(addr & 0x7F);
            });
        });
    }

    fn read_pool_result(&self, buf: &mut [u8]) {
        let rbuf = self.buf_regs();
        for (i, slot) in buf.iter_mut().enumerate() {
            *slot = rbuf.read8(i);
        }
    }

    /// Called from the per-channel ISR.
    pub(crate) fn on_interrupt(ch: u8) {
        if (ch as usize) < N_CH {
            I2C_WAKERS[ch as usize].wake();
        }
    }
}

// ── WaitDoneFuture ────────────────────────────────────────────────────────────

/// Resolves when `MASTER_IRQ_STATUS.PKT_CMD_DONE_STS` fires for channel `ch`.
///
/// Registers `I2C_WAKERS[ch]` so the per-channel ISR can wake the task.
struct WaitDoneFuture2 {
    ch: u8,
}

impl Future for WaitDoneFuture2 {
    type Output = Result<(), I2cError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let result = with_ch_result(self.ch, |r| {
            let sts = r.MASTER_IRQ_STATUS().read();
            if sts.PKT_CMD_DONE_STS() {
                r.MASTER_IRQ_STATUS()
                    .write(|w| w.set_PKT_CMD_DONE_STS(true));
                return Some(if sts.PKT_CMD_FAIL_STS() {
                    Err(I2cError::Nack)
                } else {
                    Ok(())
                });
            }
            if sts.ARB_LOSS_STS() {
                r.MASTER_IRQ_STATUS().write(|w| {
                    w.0 = 0xFFFF_FFFF;
                });
                return Some(Err(I2cError::ArbitrationLoss));
            }
            None
        });

        if let Some(r) = result {
            return Poll::Ready(r);
        }

        I2C_WAKERS[self.ch as usize].register(cx.waker());

        // Re-check after registration.
        let result2 = with_ch_result(self.ch, |r| {
            let sts = r.MASTER_IRQ_STATUS().read();
            if sts.PKT_CMD_DONE_STS() {
                r.MASTER_IRQ_STATUS()
                    .write(|w| w.set_PKT_CMD_DONE_STS(true));
                return Some(if sts.PKT_CMD_FAIL_STS() {
                    Err(I2cError::Nack)
                } else {
                    Ok(())
                });
            }
            if sts.ARB_LOSS_STS() {
                r.MASTER_IRQ_STATUS().write(|w| {
                    w.0 = 0xFFFF_FFFF;
                });
                return Some(Err(I2cError::ArbitrationLoss));
            }
            None
        });

        match result2 {
            Some(r) => Poll::Ready(r),
            None => Poll::Pending,
        }
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

/// I2C transfer error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum I2cError {
    /// Device did not acknowledge (address or data NACK).
    Nack,
    /// Arbitration lost.
    ArbitrationLoss,
    /// Transfer exceeds pool buffer (max 16 bytes per phase).
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
        if read.len() > POOL_MAX {
            return Err(I2cError::BufferOverflow);
        }
        self.start_read(address, read.len());
        WaitDoneFuture2 { ch: self.ch }.await?;
        self.read_pool_result(read);
        Ok(())
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), I2cError> {
        if write.len() > POOL_MAX {
            return Err(I2cError::BufferOverflow);
        }
        self.start_write(address, write);
        WaitDoneFuture2 { ch: self.ch }.await
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), I2cError> {
        if write.len() > POOL_MAX || read.len() > POOL_MAX {
            return Err(I2cError::BufferOverflow);
        }
        self.start_write(address, write);
        WaitDoneFuture2 { ch: self.ch }.await?;
        self.start_read(address, read.len());
        WaitDoneFuture2 { ch: self.ch }.await?;
        self.read_pool_result(read);
        Ok(())
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), I2cError> {
        for op in operations.iter_mut() {
            match op {
                Operation::Read(buf) => {
                    if buf.len() > POOL_MAX {
                        return Err(I2cError::BufferOverflow);
                    }
                    self.start_read(address, buf.len());
                    WaitDoneFuture2 { ch: self.ch }.await?;
                    self.read_pool_result(buf);
                }
                Operation::Write(buf) => {
                    if buf.len() > POOL_MAX {
                        return Err(I2cError::BufferOverflow);
                    }
                    self.start_write(address, buf);
                    WaitDoneFuture2 { ch: self.ch }.await?;
                }
            }
        }
        Ok(())
    }
}

// ── Channel dispatch helpers ──────────────────────────────────────────────────
// Maps channel index to the correct `pac::I2CX` constant.
// The PAC exposes 14 separate typed instances rather than an array.

fn with_ch<F>(ch: u8, f: F)
where
    F: FnOnce(pac::i2c_v1::I2C),
{
    match ch {
        0 => f(pac::I2C0),
        1 => f(pac::I2C1),
        2 => f(pac::I2C2),
        3 => f(pac::I2C3),
        4 => f(pac::I2C4),
        5 => f(pac::I2C5),
        6 => f(pac::I2C6),
        7 => f(pac::I2C7),
        8 => f(pac::I2C8),
        9 => f(pac::I2C9),
        10 => f(pac::I2C10),
        11 => f(pac::I2C11),
        12 => f(pac::I2C12),
        13 => f(pac::I2C13),
        _ => {}
    }
}

fn with_ch_result<F, T>(ch: u8, f: F) -> T
where
    F: FnOnce(pac::i2c_v1::I2C) -> T,
    T: Default,
{
    match ch {
        0 => f(pac::I2C0),
        1 => f(pac::I2C1),
        2 => f(pac::I2C2),
        3 => f(pac::I2C3),
        4 => f(pac::I2C4),
        5 => f(pac::I2C5),
        6 => f(pac::I2C6),
        7 => f(pac::I2C7),
        8 => f(pac::I2C8),
        9 => f(pac::I2C9),
        10 => f(pac::I2C10),
        11 => f(pac::I2C11),
        12 => f(pac::I2C12),
        13 => f(pac::I2C13),
        _ => T::default(),
    }
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
#[cfg(feature = "ast2600-ssp")]
i2c_irq!(I2C14, 14);
#[cfg(feature = "ast2600-ssp")]
i2c_irq!(I2C15, 15);
