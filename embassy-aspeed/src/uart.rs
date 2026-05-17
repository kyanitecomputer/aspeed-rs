//! UART driver: async 16550-compatible UART.
//!
//! | Chip | UART | Base | IRQ | Clock |
//! |------|------|------|-----|-------|
//! | AST2600 SSP | UART11 | `0x7E79_0500` | 62 | 24 MHz/13 (gate UART11CLK) |
//! | AST1060 | UART5 | `0x7E78_4000` | 8 | 24 MHz/13 (always-on) |
//! | AST2700 BootMCU | UART12 | `0x14C3_3B00` | — | 24 MHz/13 (polling only) |
//!
//! # Baud-rate divisor
//!
//! ```text
//! divisor = UART_CLK_HZ / (16 × baudrate)
//!         = 1_846_153  / (16 × 115_200) ≈ 1
//! ```
//!
//! # DLAB aliasing
//!
//! Offsets 0x00 and 0x04 are aliased by `LCR[DLAB]`:
//! - `DLAB=0`: RBR (read) / THR (write) at 0x00, IER at 0x04.
//! - `DLAB=1`: DLL at 0x00, DLH at 0x04.
//!
//! # Usage (AST2600 SSP)
//!
//! ```rust,ignore
//! use embassy_aspeed::{clock::{ClockGate, clock_enable}, uart::{Uart, Config}};
//! clock_enable(ClockGate::UART11CLK);
//! let mut uart = Uart::new_uart11(Config::default());
//! uart.blocking_write(b"Hello!\r\n");
//! ```
//!
//! # Usage (AST1060)
//!
//! ```rust,ignore
//! use embassy_aspeed::uart::{Uart, Config};
//! // UART5 clock is always-on on AST1060 (no gate needed).
//! let mut uart = Uart::new_uart5(Config::default());
//! uart.blocking_write(b"Hello from AST1060!\r\n");
//! ```

use core::future::Future;
use core::pin::Pin;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use embedded_io::{ErrorType, Write as BlockingWrite};
use embedded_io_async::Write as AsyncWrite;

// ── UART base addresses ───────────────────────────────────────────────────────

/// UART11 base (AST2600 SSP console).
#[cfg(feature = "ast2600-ssp")]
const UART11_BASE: usize = 0x7E79_0500;
/// UART5 base (AST1060 debug console).
#[cfg(feature = "ast1060")]
const UART5_BASE: usize = 0x7E78_4000;
/// UART12 base (AST2700 BootMCU console, reg-shift=2, polling only).
#[cfg(feature = "ast2700-bootmcu")]
const UART12_BASE: usize = 0x14C3_3B00;

// Register word offsets (byte offset / 4)
const REG_RBR_THR: usize = 0x00 / 4;
const REG_IER: usize = 0x04 / 4;
const REG_FCR_IIR: usize = 0x08 / 4;
const REG_LCR: usize = 0x0C / 4;
const REG_MCR: usize = 0x10 / 4;
const REG_LSR: usize = 0x14 / 4;

/// LSR bits
const LSR_DR: u32 = 1 << 0; // Data Ready (RX)
const LSR_OE: u32 = 1 << 1; // Overrun Error
const LSR_THRE: u32 = 1 << 5; // TX Holding Register Empty
const LSR_TEMT: u32 = 1 << 6; // TX Empty

/// IER bits
const IER_ERBFI: u32 = 1 << 0; // Enable RX interrupt
const IER_ETBEI: u32 = 1 << 1; // Enable TX interrupt

/// FCR initialisation: enable FIFO, reset TX+RX FIFOs, RX trigger=1 byte.
const FCR_INIT: u32 = 0x07;

/// LCR: 8-bit, 1 stop, no parity.
const LCR_8N1: u32 = 0x03;

/// LCR DLAB bit.
const LCR_DLAB: u32 = 1 << 7;

/// MCR: OUT2=1 (enables UART interrupts on AST2600).
const MCR_OUT2: u32 = 1 << 3;

// ── Global interrupt state ────────────────────────────────────────────────────

static TX_WAKER: AtomicWaker = AtomicWaker::new();
static RX_WAKER: AtomicWaker = AtomicWaker::new();
/// Set by ISR when THRE fires; cleared by async TX before next write.
static TX_READY: AtomicBool = AtomicBool::new(true);
/// Set by ISR when RDA fires; cleared by async RX after reading data.
static RX_READY: AtomicBool = AtomicBool::new(false);

// ── Config ────────────────────────────────────────────────────────────────────

/// UART configuration.
pub struct Config {
    /// Baud rate in bits per second. Default: 115200.
    pub baudrate: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self { baudrate: 115_200 }
    }
}

// ── Uart struct ───────────────────────────────────────────────────────────────

/// 16550-compatible UART driver (one instance per physical UART).
///
/// Construct with [`Uart::new_uart11`].
pub struct Uart {
    base: *mut u32,
}

// SAFETY: Single-core Cortex-M3; no cross-task sharing for this pointer.
unsafe impl Send for Uart {}

impl Uart {
    /// Initialise UART11 at the baud rate in `cfg` (AST2600 SSP).
    ///
    /// Call [`crate::clock::clock_enable`] with `ClockGate::UART11CLK` first.
    #[cfg(feature = "ast2600-ssp")]
    pub fn new_uart11(cfg: Config) -> Self {
        let uart = Self {
            base: UART11_BASE as *mut u32,
        };
        uart.hw_init(cfg);
        uart
    }

    /// Initialise UART5 at the baud rate in `cfg` (AST1060).
    ///
    /// UART5 clock is always-on on AST1060; no clock gate call required.
    #[cfg(feature = "ast1060")]
    pub fn new_uart5(cfg: Config) -> Self {
        let uart = Self {
            base: UART5_BASE as *mut u32,
        };
        uart.hw_init(cfg);
        uart
    }

    /// Initialise UART12 at the baud rate in `cfg` (AST2700 BootMCU).
    ///
    /// UART12 has no dedicated IRQ on the BootMCU; only blocking I/O is
    /// available.  UART clock = 24 MHz / 13 ≈ 1,846,153 Hz.
    #[cfg(feature = "ast2700-bootmcu")]
    pub fn new_uart12(cfg: Config) -> Self {
        let uart = Self {
            base: UART12_BASE as *mut u32,
        };
        uart.hw_init(cfg);
        uart
    }

    // ── Private register accessors ────────────────────────────────────────────

    #[inline(always)]
    fn rr(&self, word_off: usize) -> u32 {
        // SAFETY: `word_off` always refers to a valid UART register.
        unsafe { ptr::read_volatile(self.base.add(word_off)) }
    }

    #[inline(always)]
    fn rw(&self, word_off: usize, val: u32) {
        // SAFETY: `word_off` always refers to a valid UART register.
        unsafe { ptr::write_volatile(self.base.add(word_off), val) }
    }

    // ── Initialisation ────────────────────────────────────────────────────────

    fn hw_init(&self, cfg: Config) {
        // 1. Disable all interrupts during config.
        self.rw(REG_IER, 0);

        // 2. Set baud rate via DLAB=1.
        // Use u64 to prevent overflow if baudrate is pathologically large.
        let divisor =
            (crate::clock::UART_CLK_HZ as u64 / (16u64 * cfg.baudrate as u64).max(1)) as u32;
        self.rw(REG_LCR, LCR_8N1 | LCR_DLAB);
        self.rw(REG_RBR_THR, divisor & 0xFF); // DLL
        self.rw(REG_IER, (divisor >> 8) & 0xFF); // DLH (IER offset with DLAB=1)
        self.rw(REG_LCR, LCR_8N1); // clear DLAB

        // 3. Configure FIFOs.
        self.rw(REG_FCR_IIR, FCR_INIT);

        // 4. Enable MCU interrupt line and UART interrupt sources.
        self.rw(REG_MCR, MCR_OUT2);
        self.rw(REG_IER, IER_ERBFI | IER_ETBEI);
    }

    // ── Blocking I/O ─────────────────────────────────────────────────────────

    /// Transmit all bytes in `buf`, blocking until TX shift register drains.
    pub fn blocking_write(&mut self, buf: &[u8]) {
        for &b in buf {
            while self.rr(REG_LSR) & LSR_THRE == 0 {}
            self.rw(REG_RBR_THR, b as u32);
        }
        while self.rr(REG_LSR) & LSR_TEMT == 0 {}
    }

    /// Block until a byte is available in the RX FIFO and return it.
    pub fn blocking_read_byte(&mut self) -> u8 {
        while self.rr(REG_LSR) & LSR_DR == 0 {}
        (self.rr(REG_RBR_THR) & 0xFF) as u8
    }

    // ── Interrupt handling (called from ISR — ARM chips only) ────────────────

    /// Called from the UART ISR.  Not used on AST2700 BootMCU (UART12 polls).
    #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
    pub(crate) fn on_interrupt() {
        // Choose the correct UART base per chip.
        #[cfg(feature = "ast2600-ssp")]
        let base = UART11_BASE as *const u32;
        #[cfg(feature = "ast1060")]
        let base = UART5_BASE as *const u32;
        // SAFETY: volatile read of LSR in ISR context; single-core.
        let lsr = unsafe { ptr::read_volatile(base.add(REG_LSR)) };
        if lsr & LSR_THRE != 0 {
            TX_READY.store(true, Ordering::Release);
            TX_WAKER.wake();
        }
        if lsr & LSR_DR != 0 {
            RX_READY.store(true, Ordering::Release);
            RX_WAKER.wake();
        }
        // Reading LSR clears OE; no extra action needed.
        let _ = lsr & LSR_OE;
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

/// UART error (overrun / framing / parity reported as generic I/O error).
#[derive(Debug, Copy, Clone)]
pub struct UartError;

impl embedded_io::Error for UartError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl ErrorType for Uart {
    type Error = UartError;
}

// ── embedded_io::Write (blocking) ────────────────────────────────────────────

impl BlockingWrite for Uart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, UartError> {
        self.blocking_write(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), UartError> {
        while self.rr(REG_LSR) & LSR_TEMT == 0 {}
        Ok(())
    }
}

// ── embedded_io_async::Write ──────────────────────────────────────────────────

impl AsyncWrite for Uart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, UartError> {
        if buf.is_empty() {
            return Ok(0);
        }
        // embedded_io_async::Write requires n > 0 for non-empty buf.
        // Loop until at least one byte is accepted by the FIFO.
        loop {
            TxReady.await;
            TX_READY.store(false, Ordering::Release);

            let mut written = 0;
            for &b in buf {
                if self.rr(REG_LSR) & LSR_THRE == 0 {
                    break;
                }
                self.rw(REG_RBR_THR, b as u32);
                written += 1;
            }
            if written > 0 {
                return Ok(written);
            }
            // THRE was clear despite TxReady firing (spurious wake). Re-arm
            // TX_READY so TxReady.await immediately re-checks the flag, then
            // loop back to wait for a genuine THRE interrupt.
            TX_READY.store(true, Ordering::Release);
        }
    }

    async fn flush(&mut self) -> Result<(), UartError> {
        // Blocking drain — acceptable for console UART.
        while self.rr(REG_LSR) & LSR_TEMT == 0 {}
        Ok(())
    }
}

// ── TxReady future ────────────────────────────────────────────────────────────

struct TxReady;

impl Future for TxReady {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if TX_READY.load(Ordering::Acquire) {
            Poll::Ready(())
        } else {
            TX_WAKER.register(cx.waker());
            if TX_READY.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }
}

// ── Interrupt handlers ────────────────────────────────────────────────────────

/// UART11 interrupt handler (AST2600 SSP, IRQ 62).
#[cfg(feature = "ast2600-ssp")]
#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn UART11() {
    Uart::on_interrupt();
}

/// UART5 interrupt handler (AST1060, IRQ 8).
#[cfg(feature = "ast1060")]
#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn UART5() {
    Uart::on_interrupt();
}
