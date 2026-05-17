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
//! Each sub-channel supports 4 message IDs (0–3).  Each message ID carries
//! a 32-byte payload.  Messages are triggered by writing to the TRIG register
//! and acknowledged by writing 1 to the STATUS register.
//!
//! # Polling-only
//!
//! The BootMCU IPC1 has no IRQ line; all operations are polling.  Zephyr's
//! `ipm_bootmcu.c` driver uses a thread with `k_msleep(1)`.  Our driver
//! provides blocking `send` and `recv` that busy-wait on STATUS.
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
//!   +0x10  DATA0   32-byte payload for ID 0  (8 × u32)
//!   +0x30  DATA1   32-byte payload for ID 1
//!   +0x50  DATA2   32-byte payload for ID 2
//!   +0x70  DATA3   32-byte payload for ID 3
//! ```
//!
//! Source: Zephyr `drivers/ipm/ipm_bootmcu.c`

use core::ptr;

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

/// Return the base address for a sub-channel.
#[inline]
fn subchan_base(channel: u8) -> usize {
    IPC1_BASE + (channel as usize) * SUBCHAN_SIZE
}

#[inline]
fn read_reg(half_base: usize, off: usize) -> u32 {
    unsafe { ptr::read_volatile((half_base + off) as *const u32) }
}

#[inline]
fn write_reg(half_base: usize, off: usize, val: u32) {
    unsafe { ptr::write_volatile((half_base + off) as *mut u32, val) }
}

// ── Public API ────────────────────────────────────────────────────────────────

/// IPC1 driver handle (zero-size — all state is in MMIO registers).
pub struct Ipc1;

impl Ipc1 {
    /// Create an IPC1 driver handle and enable all RX IDs on every sub-channel.
    pub fn new() -> Self {
        for ch in 0..4u8 {
            let rx_base = subchan_base(ch) + RX_OFFSET;
            write_reg(rx_base, IPCR_ENABLE, 0xF); // enable IDs 0–3
            write_reg(rx_base, IPCR_STATUS, 0xF); // clear any stale status
        }
        Self
    }

    /// Blocking send: write `payload` to `id` on `channel` and trigger the remote.
    ///
    /// Busy-waits until a previous message with the same ID has been acknowledged.
    ///
    /// - `channel`: 0=secure-CA35, 1=non-secure-CA35, 2=SSP, 3=TSP
    /// - `id`: message ID 0–3
    pub fn send(&self, channel: u8, id: u8, payload: &Payload) {
        let tx_base = subchan_base(channel) + TX_OFFSET;
        // Wait until previous message with this ID has been consumed.
        while read_reg(tx_base, IPCR_STATUS) & (1 << id) != 0 {
            core::hint::spin_loop();
        }
        // Write payload as u32 words.
        let data_off = DATA_OFFSETS[id as usize];
        for (i, chunk) in payload.chunks_exact(4).enumerate() {
            let word = u32::from_le_bytes(chunk.try_into().unwrap());
            write_reg(tx_base, data_off + i * 4, word);
        }
        // Trigger the remote.
        write_reg(tx_base, IPCR_TRIG, 1 << id);
    }

    /// Non-blocking receive check.
    ///
    /// Returns `Some((id, payload))` if any message is pending on `channel`,
    /// or `None` if the RX FIFO is empty.  Clears the STATUS bit on receipt.
    pub fn try_recv(&self, channel: u8) -> Option<(u8, Payload)> {
        let rx_base = subchan_base(channel) + RX_OFFSET;
        let status = read_reg(rx_base, IPCR_STATUS);
        if status == 0 {
            return None;
        }
        let id = status.trailing_zeros() as u8; // lowest pending ID
        let data_off = DATA_OFFSETS[id as usize];
        let mut payload = [0u8; 32];
        for (i, chunk) in payload.chunks_exact_mut(4).enumerate() {
            let word = read_reg(rx_base, data_off + i * 4);
            chunk.copy_from_slice(&word.to_le_bytes());
        }
        // Acknowledge (write-1-clear).
        write_reg(rx_base, IPCR_STATUS, 1 << id);
        Some((id, payload))
    }

    /// Blocking receive: busy-wait until a message arrives on `channel`.
    pub fn recv(&self, channel: u8) -> (u8, Payload) {
        loop {
            if let Some(msg) = self.try_recv(channel) {
                return msg;
            }
            core::hint::spin_loop();
        }
    }
}
