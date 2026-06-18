//! IPC1 data-channel bus driver for the AST2700 BootMCU.
//!
//! IPC1 provides four sub-channels between the BootMCU and the other
//! processors on the AST2700:
//!
//! | Sub-channel | Peer |
//! |------------|------|
//! | 0 | Secure CA35 |
//! | 1 | Non-secure CA35 |
//! | 2 | SSP |
//! | 3 | TSP |
//!
//! Each sub-channel supports 4 message IDs (0-3).  Each message ID carries
//! a 32-byte payload.  Messages are triggered by writing to the TRIG register
//! and acknowledged by writing 1 to the STATUS register.
//!
//! # Sync and async
//!
//! The BootMCU IPC1 has no IRQ line; all operations are polling.
//! Both sync (blocking) and async (yielding) APIs are provided:
//!
//! - `send()` / `send_async()` — write payload and trigger remote
//! - `try_recv()` — non-blocking check (same for both modes)
//! - `recv()` / `recv_async()` — wait for a message
//!
//! # Memory layout
//!
//! ```text
//! IPC1 base: 0x14C3_9000
//!
//! Sub-channel n at base + n * 0x200:
//!   RX half (from remote):  sub-channel_base + 0x000
//!   TX half (to remote):    sub-channel_base + 0x100
//!
//! Per half:
//!   +0x00  TRIG    trigger (write BIT(id) to send)
//!   +0x04  ENABLE  per-ID receive enable mask
//!   +0x08  STATUS  pending IDs (write-1-clear)
//!   +0x10  DATA0   32-byte payload for ID 0  (8 x u32)
//!   +0x30  DATA1   32-byte payload for ID 1
//!   +0x50  DATA2   32-byte payload for ID 2
//!   +0x70  DATA3   32-byte payload for ID 3
//! ```

use aspeed_mmio::MmioBlock;

// ── Register layout ───────────────────────────────────────────────────────────

const IPC1_BASE: usize = 0x14C3_9000;
const SUBCHAN_SIZE: usize = 0x200;
const TX_OFFSET: usize = 0x100;
const RX_OFFSET: usize = 0x000;

const IPCR_TRIG: usize = 0x00;
const IPCR_ENABLE: usize = 0x04;
const IPCR_STATUS: usize = 0x08;
const IPCR_DATA0: usize = 0x10;
const IPCR_DATA1: usize = 0x30;
const IPCR_DATA2: usize = 0x50;
const IPCR_DATA3: usize = 0x70;

const DATA_OFFSETS: [usize; 4] = [IPCR_DATA0, IPCR_DATA1, IPCR_DATA2, IPCR_DATA3];

/// 32-byte message payload.
pub type Payload = [u8; 32];

// ── Helpers ───────────────────────────────────────────────────────────────────

#[inline]
fn subchan_rx(channel: u8) -> MmioBlock {
    unsafe { MmioBlock::new(IPC1_BASE + (channel as usize) * SUBCHAN_SIZE + RX_OFFSET) }
}

#[inline]
fn subchan_tx(channel: u8) -> MmioBlock {
    unsafe { MmioBlock::new(IPC1_BASE + (channel as usize) * SUBCHAN_SIZE + TX_OFFSET) }
}

fn write_payload(tx: &mut MmioBlock, id: u8, payload: &Payload) {
    let data_off = DATA_OFFSETS[id as usize];
    for (i, chunk) in payload.chunks_exact(4).enumerate() {
        let word = u32::from_le_bytes(chunk.try_into().unwrap());
        tx.write32(data_off + i * 4, word);
    }
}

fn read_payload(rx: &MmioBlock, id: u8) -> Payload {
    let data_off = DATA_OFFSETS[id as usize];
    let mut payload = [0u8; 32];
    for (i, chunk) in payload.chunks_exact_mut(4).enumerate() {
        let word = rx.read32(data_off + i * 4);
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    payload
}

// ── Public API ────────────────────────────────────────────────────────────────

/// IPC1 driver handle.
///
/// Uses `MmioBlock` for safe volatile register access (derive-mmio pattern).
/// Holds `&mut self` for write operations to enforce exclusive access through
/// the borrow checker.
pub struct Ipc1 {
    _private: (),
}

impl Ipc1 {
    /// Create an IPC1 driver handle and enable all RX IDs on every sub-channel.
    pub fn new() -> Self {
        for ch in 0..4u8 {
            let mut rx = subchan_rx(ch);
            rx.write32(IPCR_ENABLE, 0xF);
            rx.write32(IPCR_STATUS, 0xF);
        }
        Self { _private: () }
    }

    // ── Sync API ──────────────────────────────────────────────────────────

    /// Blocking send: write `payload` to `id` on `channel` and trigger the remote.
    ///
    /// Busy-waits until a previous message with the same ID has been acknowledged.
    ///
    /// - `channel`: 0=secure-CA35, 1=non-secure-CA35, 2=SSP, 3=TSP
    /// - `id`: message ID 0-3
    pub fn send(&mut self, channel: u8, id: u8, payload: &Payload) {
        let mut tx = subchan_tx(channel);
        while tx.read32(IPCR_STATUS) & (1 << id) != 0 {
            core::hint::spin_loop();
        }
        write_payload(&mut tx, id, payload);
        tx.write32(IPCR_TRIG, 1 << id);
    }

    /// Non-blocking receive check.
    ///
    /// Returns `Some((id, payload))` if any message is pending on `channel`,
    /// or `None` if the RX FIFO is empty.  Clears the STATUS bit on receipt.
    pub fn try_recv(&mut self, channel: u8) -> Option<(u8, Payload)> {
        let rx = subchan_rx(channel);
        let status = rx.read32(IPCR_STATUS);
        if status == 0 {
            return None;
        }
        let id = status.trailing_zeros() as u8;
        let payload = read_payload(&rx, id);
        let mut rx = subchan_rx(channel);
        rx.write32(IPCR_STATUS, 1 << id);
        Some((id, payload))
    }

    /// Blocking receive: busy-wait until a message arrives on `channel`.
    pub fn recv(&mut self, channel: u8) -> (u8, Payload) {
        loop {
            if let Some(msg) = self.try_recv(channel) {
                return msg;
            }
            core::hint::spin_loop();
        }
    }

    // ── Async API ─────────────────────────────────────────────────────────

    /// Async send: write `payload` to `id` on `channel` and trigger the remote.
    ///
    /// Yields to the executor while waiting for a previous message to be
    /// acknowledged, instead of busy-waiting.
    #[cfg(feature = "ast2700-bootmcu")]
    pub async fn send_async(&mut self, channel: u8, id: u8, payload: &Payload) {
        let mut tx = subchan_tx(channel);
        let _ = aspeed_mmio::poll_until_async(
            || tx.read32(IPCR_STATUS),
            |s| s & (1 << id) == 0,
            embassy_time::Duration::from_micros(100),
            embassy_time::Duration::from_secs(5),
        )
        .await;
        write_payload(&mut tx, id, payload);
        tx.write32(IPCR_TRIG, 1 << id);
    }

    /// Async receive: yield to the executor while waiting for a message.
    ///
    /// Polls the RX status register at 100us intervals, yielding between
    /// polls to allow other Embassy tasks to run.
    #[cfg(feature = "ast2700-bootmcu")]
    pub async fn recv_async(&mut self, channel: u8) -> (u8, Payload) {
        loop {
            if let Some(msg) = self.try_recv(channel) {
                return msg;
            }
            embassy_time::Timer::after(embassy_time::Duration::from_micros(100)).await;
        }
    }
}
