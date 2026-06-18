//! UART driver — PAC-based (16550-compatible).
//!
//! Source of truth: `aspeed-data/data/registers/uart_v1.yaml`.
//!
//! | Chip | UART | Base address | PAC | IRQ | Clock |
//! |------|------|-------------|-----|-----|-------|
//! | AST1060 | UART5 | `0x7E78_4000` | `pac::UART5` | 8 | 24 MHz / 13 |
//! | AST2600 SSP | UART11 | `0x7E79_0500` | (ast2600 pac) | 62 | 24 MHz / 13 |
//! | AST2700 BootMCU | UART12 | `0x14C3_3B00` | — | — | polling only |
//!
//! # DLAB aliasing
//!
//! Offsets 0x00 and 0x04 are aliased by `LCR.DLAB`:
//! - `DLAB=0`: `RBR_THR` (0x00) and `IER` (0x04) — modelled by PAC.
//! - `DLAB=1`: `DLL` (0x00) and `DLH` (0x04) — accessed via raw pointer during
//!   baud-rate initialisation only.  The PAC `uart_v1.yaml` documents this
//!   aliasing explicitly.
//!
//! # Async model
//!
//! - **TX** (`embedded_io_async::Write`): ISR-driven via `TX_WAKER`/`TX_READY`.
//!   `IER.ETBEI` enables the THRE interrupt; the ISR sets `TX_READY` and wakes
//!   the `TxReady` future.
//! - **RX** (`embedded_io_async::Read`): ISR-driven via `RX_WAKER`/`RX_READY`.
//!   `IER.ERBFI` enables the RDA interrupt; the ISR sets `RX_READY` and wakes
//!   the `RxReady` future.
//! - **AST2700 BootMCU UART12**: no NVIC interrupt line; both TX and RX use
//!   blocking spin (documented in the constructor).

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll};

#[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
use aspeed_mmio::MmioBlock;
use embassy_sync::waitqueue::AtomicWaker;

use embedded_io::{ErrorType, Write as BlockingWrite};
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

use crate::pac;

// ── UART12 PAC accessor (AST2700 BootMCU) ────────────────────────────────────
// Uses the generated pac::UART12 constant (from uart_v1.yaml via the PAC
// generator).  Source of truth: aspeed-data/data/registers/uart_v1.yaml.
#[cfg(feature = "ast2700-bootmcu")]
#[inline(always)]
fn uart12() -> pac::uart_v1::UART {
    pac::UART12
}

// ── Global interrupt state (async ISR infrastructure, not yet wired) ─────────

#[allow(dead_code)]
static TX_WAKER: AtomicWaker = AtomicWaker::new();
#[allow(dead_code)]
static RX_WAKER: AtomicWaker = AtomicWaker::new();
/// Set by ISR when THRE fires; cleared before the next async write attempt.
#[allow(dead_code)]
static TX_READY: AtomicBool = AtomicBool::new(true);
/// Set by ISR when RDA fires; cleared after reading available data.
#[allow(dead_code)]
static RX_READY: AtomicBool = AtomicBool::new(false);

// ── Config ────────────────────────────────────────────────────────────────────

/// UART configuration.
pub struct Config {
    /// Baud rate in bits per second.  Default: 115_200.
    pub baudrate: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self { baudrate: 115_200 }
    }
}

// ── Uart ──────────────────────────────────────────────────────────────────────

/// 16550-compatible UART driver.
pub struct Uart {
    /// Base address for DLAB-aliased baud-rate registers and UART12.
    #[allow(dead_code)]
    base: usize,
    #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
    _use_pac: bool, // placeholder: PAC is always used where available
}

impl Uart {
    /// Initialise UART5 (AST1060).  No clock gate needed (always-on).
    #[cfg(feature = "ast1060")]
    pub fn new_uart5(cfg: Config) -> Self {
        let base = pac::UART5.as_ptr() as usize;
        let uart = Self {
            base,
            _use_pac: true,
        };
        uart.hw_init(cfg);
        uart
    }

    /// Initialise UART11 (AST2600 SSP).
    ///
    /// Call `crate::clock::clock_enable(ClockGate::UART11CLK)` first.
    #[cfg(feature = "ast2600-ssp")]
    pub fn new_uart11(cfg: Config) -> Self {
        // AST2600 uses the ast2600 pac feature, which has its own UART11 constant.
        // Use raw base address — same hw_init path applies.
        const UART11_BASE: usize = 0x7E79_0500;
        let base = UART11_BASE;
        let uart = Self {
            base,
            _use_pac: true,
        };
        uart.hw_init(cfg);
        uart
    }

    /// Obtain a UART12 handle for the AST2700 BootMCU (polling only).
    ///
    /// **Does not reinitialise UART12.**  The ROM configures UART12 at
    /// 115200 8N1 before handing off; reinitialising kills further output.
    /// All accesses go through `pac::UART12` (uart_v1.yaml, THR/LSR only).
    #[cfg(feature = "ast2700-bootmcu")]
    pub fn new_uart12(_cfg: Config) -> Self {
        // Use the PAC base address as the raw pointer for DLAB-path compatibility.
        // On BootMCU we never call hw_init, so this pointer is never dereferenced
        // for DLAB-aliased registers.
        Self {
            base: pac::UART12.as_ptr() as usize,
        }
    }

    // ── Initialisation ────────────────────────────────────────────────────────

    #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
    fn hw_init(&self, cfg: Config) {
        // Use PAC for all registers except the DLAB-aliased DLL/DLH.
        // uart_v1.yaml documents: "DLAB=1 access handled in HAL code via raw pointer."

        let uart = self.pac_regs();

        // 1. Disable all interrupts while configuring (IER = 0).
        uart.IER().write(|w| {
            w.set_ERBFI(false);
            w.set_ETBEI(false);
            w.set_ELSI(false);
            w.set_EDSSI(false);
        });

        // 2. Set baud rate via DLAB=1 (raw pointer access for DLL/DLH).
        let divisor =
            (crate::clock::UART_CLK_HZ as u64 / (16u64 * cfg.baudrate as u64).max(1)) as u32;

        // DLAB=1 aliases the LCR-gated DLL/DLH registers at the same physical
        // offsets as RBR_THR and IER. The PAC models the DLAB=0 view, so use
        // MmioBlock only for this transient baud-rate setup.
        uart.LCR().modify(|w| w.set_DLAB(true));
        let mut regs = unsafe { MmioBlock::new(self.base) };
        regs.write32(0x00, divisor & 0xFF); // DLL
        regs.write32(0x04, (divisor >> 8) & 0xFF); // DLH
        uart.LCR().modify(|w| w.set_DLAB(false));

        // 3. Set 8N1 line format (WLS=11, STB=0, PEN=0).
        uart.LCR().write(|w| {
            w.set_WLS(0b11); // 8 data bits
            w.set_STB(false); // 1 stop bit
            w.set_PEN(false); // no parity
            w.set_DLAB(false);
        });

        // 4. Enable and reset FIFOs via FCR (write-only at offset 0x08).
        // FCR_IIR is write→FCR / read→IIR per uart_v1.yaml.
        uart.FCR_IIR().write(|w| {
            w.set_FIFOE_OR_IP(true); // FIFO enable
            w.set_RFIFOR_OR_IID1(true); // RX FIFO reset
            w.set_XFIFOR_OR_IID2(true); // TX FIFO reset
        });

        // 5. MCR: OUT2=1 to enable interrupts on ASPEED UARTs.
        uart.MCR().write(|w| w.set_OUT2(true));

        // 6. Enable RX and TX interrupts.
        uart.IER().write(|w| {
            w.set_ERBFI(true); // RDA interrupt (RX data available)
            w.set_ETBEI(true); // THRE interrupt (TX holding register empty)
        });
    }

    // ── PAC register accessor ─────────────────────────────────────────────────

    #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
    fn pac_regs(&self) -> pac::uart_v1::UART {
        // SAFETY: `base` was set to a valid UART MMIO address in the constructor.
        unsafe { pac::uart_v1::UART::from_ptr(self.base as *mut ()) }
    }

    /// Read LSR via pac::UART12 (AST2700 BootMCU).
    #[cfg(feature = "ast2700-bootmcu")]
    fn lsr_bootmcu(&self) -> pac::uart_v1::LSR {
        uart12().LSR().read()
    }

    // ── Blocking I/O ──────────────────────────────────────────────────────────

    /// Transmit a single byte, polling until THRE.
    #[inline(always)]
    pub fn write_byte(&mut self, byte: u8) {
        #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
        {
            let uart = self.pac_regs();
            while !uart.LSR().read().THRE() {}
            uart.RBR_THR().write(|w| w.set_DATA(byte));
        }
        #[cfg(feature = "ast2700-bootmcu")]
        {
            // Poll LSR.THRE (uart_v1.yaml) then write THR.
            while !self.lsr_bootmcu().THRE() {}
            uart12().RBR_THR().write(|w| w.set_DATA(byte));
        }
    }

    /// Transmit a byte slice, blocking until all bytes are sent and TX drains.
    pub fn blocking_write(&mut self, buf: &[u8]) {
        for &b in buf {
            self.write_byte(b);
        }
        #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
        {
            let uart = self.pac_regs();
            while !uart.LSR().read().TEMT() {} // wait for TX shift register empty
        }
        #[cfg(feature = "ast2700-bootmcu")]
        while !self.lsr_bootmcu().TEMT() {} // LSR.TEMT: TX shift register empty
    }

    /// Block until a byte is available in RX FIFO and return it.
    pub fn blocking_read_byte(&mut self) -> u8 {
        #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
        {
            let uart = self.pac_regs();
            while !uart.LSR().read().DR() {}
            uart.RBR_THR().read().DATA()
        }
        #[cfg(feature = "ast2700-bootmcu")]
        {
            while !self.lsr_bootmcu().DR() {}
            uart12().RBR_THR().read().DATA()
        }
    }

    // ── Interrupt handler (ARM chips only) ────────────────────────────────────

    #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
    pub(crate) fn on_interrupt(base: usize) {
        let lsr_word = unsafe { MmioBlock::new(base) }.read32(0x14);
        if lsr_word & (1 << 5) != 0 {
            // THRE: TX holding register empty.
            TX_READY.store(true, Ordering::Release);
            TX_WAKER.wake();
        }
        if lsr_word & 1 != 0 {
            // DR: data ready in RX FIFO.
            RX_READY.store(true, Ordering::Release);
            RX_WAKER.wake();
        }
        // Reading LSR clears OE (overrun error) — no extra action needed.
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

/// UART I/O error.
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
        #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
        while !self.pac_regs().LSR().read().TEMT() {}
        #[cfg(feature = "ast2700-bootmcu")]
        while !self.lsr_bootmcu().TEMT() {}
        Ok(())
    }
}

// ── embedded_io_async::Write ──────────────────────────────────────────────────

impl AsyncWrite for Uart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, UartError> {
        if buf.is_empty() {
            return Ok(0);
        }
        #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
        loop {
            TxReady.await;
            TX_READY.store(false, Ordering::Release);

            let uart = self.pac_regs();
            let mut written = 0;
            for &b in buf {
                if !uart.LSR().read().THRE() {
                    break;
                }
                uart.RBR_THR().write(|w| w.set_DATA(b));
                written += 1;
            }
            if written > 0 {
                return Ok(written);
            }
            // Spurious wakeup — re-arm and retry.
            TX_READY.store(true, Ordering::Release);
        }
        #[cfg(feature = "ast2700-bootmcu")]
        {
            // UART12 has no IRQ — blocking transmit.
            self.blocking_write(buf);
            Ok(buf.len())
        }
    }

    async fn flush(&mut self) -> Result<(), UartError> {
        // Blocking drain is acceptable for console UART.
        BlockingWrite::flush(self)
    }
}

// ── embedded_io_async::Read ───────────────────────────────────────────────────

impl AsyncRead for Uart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, UartError> {
        if buf.is_empty() {
            return Ok(0);
        }
        #[cfg(any(feature = "ast2600-ssp", feature = "ast1060"))]
        loop {
            RxReady.await;
            RX_READY.store(false, Ordering::Release);

            let uart = self.pac_regs();
            let mut count = 0;
            while count < buf.len() {
                if !uart.LSR().read().DR() {
                    break;
                }
                buf[count] = uart.RBR_THR().read().DATA();
                count += 1;
            }
            if count > 0 {
                return Ok(count);
            }
            // Spurious wakeup.
            RX_READY.store(true, Ordering::Release);
        }
        #[cfg(feature = "ast2700-bootmcu")]
        {
            // UART12: spin until at least one byte arrives, then drain.
            while !self.lsr_bootmcu().DR() {}
            let mut count = 0;
            while count < buf.len() {
                if !self.lsr_bootmcu().DR() {
                    break;
                }
                buf[count] = uart12().RBR_THR().read().DATA();
                count += 1;
            }
            Ok(count)
        }
    }
}

// ── Futures (async interrupt-driven UART — not yet wired to ISR) ─────────────

#[allow(dead_code)]
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

#[allow(dead_code)]
struct RxReady;

impl Future for RxReady {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if RX_READY.load(Ordering::Acquire) {
            Poll::Ready(())
        } else {
            RX_WAKER.register(cx.waker());
            if RX_READY.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }
}

// ── Interrupt handlers ────────────────────────────────────────────────────────

#[cfg(feature = "ast2600-ssp")]
#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn UART11() {
    const UART11_BASE: usize = 0x7E79_0500;
    Uart::on_interrupt(UART11_BASE);
}

#[cfg(feature = "ast1060")]
#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn UART5() {
    Uart::on_interrupt(pac::UART5.as_ptr() as usize);
}
