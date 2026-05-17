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
//! use embassy_aspeed::ipc::{send, recv, Channel};
//!
//! send(Channel::new(0)).unwrap();          // send doorbell to CA7 on channel 0
//! let ch = recv().await;                  // wait for any CA7 doorbell
//! ```

use core::future::Future;
use core::pin::Pin;
use core::ptr;
use core::task::{Context, Poll};

use core::cell::Cell;
use critical_section::Mutex;

use embassy_sync::waitqueue::AtomicWaker;

// ── IPC register addresses ────────────────────────────────────────────────────

const IPC_BASE: usize = 0x7E6C_0000;
const IPC_TRIG: *mut u32 = (IPC_BASE + 0x18) as *mut u32;
const IPC_STATUS: *const u32 = (IPC_BASE + 0x28) as *const u32;
const IPC_CLEAR: *mut u32 = (IPC_BASE + 0x2C) as *mut u32;

pub const NUM_CHANNELS: usize = 15;

// ── Global waker storage ──────────────────────────────────────────────────────

/// One waker per IPC channel (IPC0–IPC14).
static CHANNEL_WAKERS: [AtomicWaker; NUM_CHANNELS] = {
    // const-init array of AtomicWaker
    [
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
        AtomicWaker::new(),
    ]
};

/// Bitmask of channels that have fired (set by ISR, read by futures).
/// Each bit N = channel N received a CA7 doorbell.
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

    /// Return the channel number.
    pub const fn number(self) -> u8 {
        self.0
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

/// Error returned by [`send`].
#[derive(Debug, Copy, Clone)]
pub enum IpcError {
    /// The channel is already pending (CA7 has not yet acknowledged the
    /// previous send, or another pending CA7→CM3 bell uses this bit).
    Busy,
}

// ── Send ──────────────────────────────────────────────────────────────────────

/// Ring the doorbell on `channel` to notify the CA7.
///
/// Returns `Err(IpcError::Busy)` if the channel is already pending.
pub fn send(channel: Channel) -> Result<(), IpcError> {
    let mask = 1u32 << channel.0;
    // SAFETY: volatile read/write of IPC MMIO.
    let status = unsafe { ptr::read_volatile(IPC_STATUS) };
    if status & mask != 0 {
        return Err(IpcError::Busy);
    }
    unsafe { ptr::write_volatile(IPC_TRIG, mask) };
    Ok(())
}

/// Ring the doorbell on `channel` and busy-wait until the CA7 acknowledges.
///
/// If the channel is already pending (e.g. a previous send was not yet
/// acknowledged, or a CA7→CM3 bell on the same channel is outstanding),
/// waits for it to clear before asserting the new trigger.
pub fn send_wait(channel: Channel) {
    let mask = 1u32 << channel.0;
    // SAFETY: IPC MMIO access.
    unsafe {
        // Wait for any in-flight state on this channel to clear first.
        // Writing TRIG while STATUS[n] is already set has undefined hardware
        // behaviour; send() already guards against this with Busy, so we
        // mirror that here.
        while ptr::read_volatile(IPC_STATUS) & mask != 0 {
            core::hint::spin_loop();
        }
        ptr::write_volatile(IPC_TRIG, mask);
        // Now wait for the CA7 to acknowledge (STATUS[n] goes back to 0).
        while ptr::read_volatile(IPC_STATUS) & mask != 0 {
            core::hint::spin_loop();
        }
    }
}

// ── Receive ───────────────────────────────────────────────────────────────────

/// Wait for any CA7→CM3 IPC doorbell and return the triggered channel.
///
/// If multiple channels are pending at once, the lowest-numbered one is
/// returned first.
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
            // Clear only this channel's bit.
            critical_section::with(|cs| {
                let cell = PENDING.borrow(cs);
                cell.set(cell.get() & !(1u32 << n));
            });
            return Poll::Ready(Channel(n));
        }
        // Register wakers on all channels.
        for w in &CHANNEL_WAKERS {
            w.register(cx.waker());
        }
        // Re-check after registering.
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
    // Read and clear the status bit for this channel.
    // SAFETY: volatile read/write in ISR; single-core.
    let mask = 1u32 << ch;
    unsafe {
        let status = ptr::read_volatile(IPC_STATUS);
        ptr::write_volatile(IPC_CLEAR, status & mask);
    }
    // Mark the channel as pending and wake any waiting futures.
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
