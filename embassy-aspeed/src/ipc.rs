//! IPC mailbox driver: 15-channel async doorbell between SSP (CM3) and CA7.
//!
//! # Hardware layout (AST2600)
//!
//! IPC base: `0x7E6C_0000` (CM3 view).
//!
//! | Offset | Register | Description |
//! |--------|----------|-------------|
//! | 0x18   | TRIG     | CM3→CA7: write `BIT(n)` to ring doorbell on channel n |
//! | 0x28   | STATUS   | Bit n set = channel n pending (CA7→CM3 or waiting for CA7 ack) |
//! | 0x2C   | CLEAR    | Write `BIT(n)` to acknowledge CM3←CA7 channel n |
//!
//! Each of the 15 channels (0–14) has a dedicated IRQ: `IPC0`–`IPC14` (182–196).
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::ipc::{send, send_wait_async, recv, Channel};
//!
//! send(Channel::new(0)).unwrap();          // non-blocking send
//! send_wait_async(Channel::new(0)).await;  // send and yield until CA7 acks
//! let ch = recv().await;                  // wait for any CA7 doorbell
//! ```

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use core::cell::Cell;
use critical_section::Mutex;

use embassy_sync::waitqueue::AtomicWaker;
use aspeed_mmio::MmioBlock;

// ── IPC register layout ───────────────────────────────────────────────────────

const IPC_BASE: usize = 0x7E6C_0000;
const IPC_TRIG_OFF: usize = 0x18;
const IPC_STATUS_OFF: usize = 0x28;
const IPC_CLEAR_OFF: usize = 0x2C;

#[inline]
fn ipc() -> MmioBlock {
    unsafe { MmioBlock::new(IPC_BASE) }
}

pub const NUM_CHANNELS: usize = 15;

// ── Global waker storage ──────────────────────────────────────────────────────

static CHANNEL_WAKERS: [AtomicWaker; NUM_CHANNELS] = {
    [
        AtomicWaker::new(), AtomicWaker::new(), AtomicWaker::new(),
        AtomicWaker::new(), AtomicWaker::new(), AtomicWaker::new(),
        AtomicWaker::new(), AtomicWaker::new(), AtomicWaker::new(),
        AtomicWaker::new(), AtomicWaker::new(), AtomicWaker::new(),
        AtomicWaker::new(), AtomicWaker::new(), AtomicWaker::new(),
    ]
};

static PENDING: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

// ── Channel newtype ───────────────────────────────────────────────────────────

/// An IPC channel number in the range `0..NUM_CHANNELS`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Channel(u8);

impl Channel {
    /// Construct a channel.
    ///
    /// # Panics
    ///
    /// Panics if `n >= NUM_CHANNELS` (15).
    pub const fn new(n: u8) -> Self {
        assert!(
            (n as usize) < NUM_CHANNELS,
            "IPC channel out of range (0..15)"
        );
        Self(n)
    }

    pub const fn number(self) -> u8 {
        self.0
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Debug, Copy, Clone)]
pub enum IpcError {
    /// The channel is already pending (CA7 has not yet acknowledged).
    Busy,
}

// ── Send ──────────────────────────────────────────────────────────────────────

/// Ring the doorbell on `channel` to notify the CA7.
///
/// Returns `Err(IpcError::Busy)` if the channel is already pending.
pub fn send(channel: Channel) -> Result<(), IpcError> {
    let mask = 1u32 << channel.0;
    if ipc().read32(IPC_STATUS_OFF) & mask != 0 {
        return Err(IpcError::Busy);
    }
    let mut ipc = ipc();
    ipc.write32(IPC_TRIG_OFF, mask);
    Ok(())
}

/// Ring the doorbell on `channel` and busy-wait until the CA7 acknowledges.
///
/// Spins on STATUS until the channel bit clears, sends TRIG, then spins again
/// until the CA7 clears the ack bit.
pub fn send_wait(channel: Channel) {
    let mask = 1u32 << channel.0;
    let ipc = ipc();
    while ipc.read32(IPC_STATUS_OFF) & mask != 0 {
        core::hint::spin_loop();
    }
    let mut ipc_w = ipc;
    ipc_w.write32(IPC_TRIG_OFF, mask);
    while ipc_w.read32(IPC_STATUS_OFF) & mask != 0 {
        core::hint::spin_loop();
    }
}

/// Ring the doorbell on `channel` and yield until the CA7 acknowledges.
///
/// Yields to the Embassy executor between polls instead of busy-waiting,
/// allowing other tasks to run while waiting for CA7 acknowledgement.
#[cfg(any(feature = "ast2700-bootmcu", feature = "ast2600-ssp", feature = "ast1060"))]
pub async fn send_wait_async(channel: Channel) {
    use embassy_time::Duration;
    const POLL_INTERVAL: Duration = Duration::from_micros(10);
    const TIMEOUT: Duration = Duration::from_millis(500);

    let mask = 1u32 << channel.0;

    let _ = aspeed_mmio::poll_until_async(
        || ipc().read32(IPC_STATUS_OFF),
        |s| s & mask == 0,
        POLL_INTERVAL,
        TIMEOUT,
    ).await;

    let mut ipc_w = ipc();
    ipc_w.write32(IPC_TRIG_OFF, mask);

    let _ = aspeed_mmio::poll_until_async(
        || ipc().read32(IPC_STATUS_OFF),
        |s| s & mask == 0,
        POLL_INTERVAL,
        TIMEOUT,
    ).await;
}

// ── Receive ───────────────────────────────────────────────────────────────────

/// Wait for any CA7→CM3 IPC doorbell and return the triggered channel.
pub fn recv() -> RecvAny {
    RecvAny
}

/// Wait for a specific CA7→CM3 doorbell on `channel`.
pub fn recv_channel(channel: Channel) -> RecvChannel {
    RecvChannel(channel)
}

/// Future returned by [`recv`].
pub struct RecvAny;

impl Future for RecvAny {
    type Output = Channel;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Channel> {
        let pending = critical_section::with(|cs| PENDING.borrow(cs).get());
        if pending != 0 {
            let n = pending.trailing_zeros() as u8;
            critical_section::with(|cs| {
                let cell = PENDING.borrow(cs);
                cell.set(cell.get() & !(1u32 << n));
            });
            return Poll::Ready(Channel(n));
        }
        for w in &CHANNEL_WAKERS {
            w.register(cx.waker());
        }
        let pending = critical_section::with(|cs| PENDING.borrow(cs).get());
        if pending != 0 {
            let n = pending.trailing_zeros() as u8;
            critical_section::with(|cs| {
                let cell = PENDING.borrow(cs);
                cell.set(cell.get() & !(1u32 << n));
            });
            Poll::Ready(Channel(n))
        } else {
            Poll::Pending
        }
    }
}

/// Future returned by [`recv_channel`].
pub struct RecvChannel(Channel);

impl Future for RecvChannel {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let mask = 1u32 << self.0 .0;
        let pending = critical_section::with(|cs| PENDING.borrow(cs).get());
        if pending & mask != 0 {
            critical_section::with(|cs| {
                let cell = PENDING.borrow(cs);
                cell.set(cell.get() & !mask);
            });
            return Poll::Ready(());
        }
        CHANNEL_WAKERS[self.0 .0 as usize].register(cx.waker());
        let pending = critical_section::with(|cs| PENDING.borrow(cs).get());
        if pending & mask != 0 {
            critical_section::with(|cs| {
                let cell = PENDING.borrow(cs);
                cell.set(cell.get() & !mask);
            });
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// ── ISR helper ────────────────────────────────────────────────────────────────

/// Called from each IPC interrupt handler with the channel index that fired.
pub(crate) fn on_interrupt(ch: u8) {
    let mask = 1u32 << ch;
    let status = ipc().read32(IPC_STATUS_OFF);
    // Only acknowledge and wake if the status bit is actually set.
    // Spurious interrupt entries (status already cleared) must not produce
    // ghost messages in PENDING, which would cause RecvAny/RecvChannel to
    // deliver a receive event with no actual data.
    if status & mask == 0 {
        return;
    }
    let mut ipc_w = ipc();
    ipc_w.write32(IPC_CLEAR_OFF, mask);
    critical_section::with(|cs| {
        let cell = PENDING.borrow(cs);
        cell.set(cell.get() | mask);
    });
    CHANNEL_WAKERS[ch as usize].wake();
}

// ── Interrupt handlers ────────────────────────────────────────────────────────

macro_rules! ipc_irq {
    ($name:ident, $n:expr) => {
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn $name() {
            on_interrupt($n);
        }
    };
}

ipc_irq!(IPC0, 0);
ipc_irq!(IPC1, 1);
ipc_irq!(IPC2, 2);
ipc_irq!(IPC3, 3);
ipc_irq!(IPC4, 4);
ipc_irq!(IPC5, 5);
ipc_irq!(IPC6, 6);
ipc_irq!(IPC7, 7);
ipc_irq!(IPC8, 8);
ipc_irq!(IPC9, 9);
ipc_irq!(IPC10, 10);
ipc_irq!(IPC11, 11);
ipc_irq!(IPC12, 12);
ipc_irq!(IPC13, 13);
ipc_irq!(IPC14, 14);

// ── Host tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_bit_mask() {
        assert_eq!(1u32 << Channel::new(0).number(), 1);
        assert_eq!(1u32 << Channel::new(14).number(), 1 << 14);
    }

    #[test]
    #[should_panic]
    fn channel_out_of_range() {
        Channel::new(15);
    }
}
