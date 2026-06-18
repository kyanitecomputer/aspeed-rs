//! I3C master driver (AST1060) — clean-room PAC-based implementation.
//!
//! Sources of truth:
//! - `aspeed-data/data/registers/i3c_v1.yaml`        (DWC I3C per-channel)
//! - `aspeed-data/data/registers/i3cglobal_v1.yaml`  (ASPEED global wrapper)
//!
//! # Controller instances
//!
//! | Channel | PAC | Base | Global PHY | IRQ |
//! |---------|-----|------|-----------|-----|
//! | 0 | `pac::I3C0` | `0x7E7A_2000` | `I3C_GLOBAL.CH1_*` | 102 |
//! | 1 | `pac::I3C1` | `0x7E7A_3000` | `I3C_GLOBAL.CH2_*` | 103 |
//! | 2 | `pac::I3C2` | `0x7E7A_4000` | `I3C_GLOBAL.CH3_*` | 104 |
//! | 3 | `pac::I3C3` | `0x7E7A_5000` | `I3C_GLOBAL.CH4_*` | 105 |
//!
//! # DWC I3C command-queue model
//!
//! All operations pass through a hardware command queue and response queue:
//!
//! 1. Push command descriptors to `CMD_QUEUE_PORT` (write-only FIFO).
//! 2. Push TX data words to `TX_RX_DATA_PORT` (write = TX FIFO).
//! 3. Hardware executes and pushes a response to `RESP_QUEUE_PORT`.
//! 4. Pop response — check `ERR_STATUS`.
//! 5. Pop RX data from `TX_RX_DATA_PORT` (read = RX FIFO).
//!
//! Commands are 32-bit words with `CMD_ATTR[2:0]` selecting the format
//! (see `i3c_v1.yaml` `I3C_CMD_PORT` fieldset and the encoding constants below).
//!
//! # Async model
//!
//! The `RESP_READY` interrupt (INTR bit 4, signal enabled via `INTR_SIGNAL_EN`)
//! fires when at least one response entry is in the response queue.  Each
//! transfer awaits a `RespReadyFuture` which registers a per-channel
//! `AtomicWaker`; the ISR wakes it.
//!
//! # Device Address Table (DAT)
//!
//! The hardware has 8 DAT slots (`DAT_DEV1`–`DAT_DEV8`).  Call
//! `add_device(slot, addr)` to register a target before issuing transfers.
//! `run_entdaa()` fills DAT slots automatically during address assignment.
//!
//! # Timing (default configuration: 200 MHz I3C core clock)
//!
//! | Mode | Registers | Default counts | Resulting frequency |
//! |------|-----------|----------------|---------------------|
//! | SDR0 push-pull | `SCL_I3C_PP_TIMING` | HCNT=8, LCNT=8 | 12.5 MHz |
//! | Open-drain | `SCL_I3C_OD_TIMING` | HCNT=16, LCNT=40 | ≈3.6 MHz |
//! | I2C FM | `SCL_I2C_FM_TIMING` | HCNT=240, LCNT=300 | 370 kHz |
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::i3c::{I3cMaster, I3cConfig};
//!
//! let mut bus = I3cMaster::new(0, I3cConfig::default());
//!
//! // Assign dynamic addresses to all devices that respond to ENTDAA.
//! let mut devices = [DaaResult::default(); 8];
//! let count = bus.run_entdaa(&mut devices).await.unwrap();
//!
//! // Write 4 bytes to the device at DAT slot 0.
//! bus.write(0, &[0x01, 0x02, 0x03, 0x04]).await.unwrap();
//! ```

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::pac;

// ── Global wakers (one per channel) ──────────────────────────────────────────

static I3C_WAKERS: [AtomicWaker; 4] = [
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
];

// ── Command word field encodings ─────────────────────────────────────────────
//
// All command words start with CMD_ATTR in bits[2:0].

/// Command attribute values (bits[2:0]).
const CMD_ATTR_TC: u32 = 0; // Transfer Command
const CMD_ATTR_TARG: u32 = 1; // Transfer Argument (data length)
const CMD_ATTR_AAC: u32 = 3; // Address Assignment Command (ENTDAA)

/// Transfer Command (TC) bit positions.
const TC_TID_SHIFT: u32 = 3; // bits[6:3]  — 4-bit transaction ID
const TC_CMD_SHIFT: u32 = 7; // bits[14:7] — 8-bit CCC/command code
const TC_CP_BIT: u32 = 1 << 15; // CCC Present
const TC_DEV_IDX_SHIFT: u32 = 16; // bits[22:16] — 7-bit DAT table index
const TC_ROC_BIT: u32 = 1 << 28; // Response On Completion
const TC_RNW_BIT: u32 = 1 << 27; // 1=Read, 0=Write
const TC_TOC_BIT: u32 = 1 << 29; // Terminate On Completion (STOP)

/// Transfer Argument (TARG) bit positions.
const TARG_DL_SHIFT: u32 = 3; // bits[15:3] — 13-bit data length

/// Address Assignment Command (AAC) bit positions.
const AAC_TID_SHIFT: u32 = 3; // bits[6:3]
const AAC_DEV_COUNT_SHIFT: u32 = 16; // bits[22:16] — max devices

/// Response queue error status values (i3c_v1.yaml I3C_RESP_PORT.ERR_STATUS).
const RESP_ERR_OK: u32 = 0;

fn tc_word(tid: u8, dev_idx: u8, rnw: bool, roc: bool, toc: bool) -> u32 {
    CMD_ATTR_TC
        | ((tid as u32 & 0xF) << TC_TID_SHIFT)
        | ((dev_idx as u32 & 0x7F) << TC_DEV_IDX_SHIFT)
        | (if rnw { TC_RNW_BIT } else { 0 })
        | (if roc { TC_ROC_BIT } else { 0 })
        | (if toc { TC_TOC_BIT } else { 0 })
}

fn tc_ccc_bcast(tid: u8, ccc: u8, roc: bool, toc: bool) -> u32 {
    CMD_ATTR_TC
        | TC_CP_BIT
        | ((tid as u32 & 0xF) << TC_TID_SHIFT)
        | ((ccc as u32) << TC_CMD_SHIFT)
        | (if roc { TC_ROC_BIT } else { 0 })
        | (if toc { TC_TOC_BIT } else { 0 })
}

fn targ_word(data_len: u16) -> u32 {
    CMD_ATTR_TARG | ((data_len as u32 & 0x1FFF) << TARG_DL_SHIFT)
}

fn aac_word(tid: u8, dev_count: u8, roc: bool, toc: bool) -> u32 {
    CMD_ATTR_AAC
        | ((tid as u32 & 0xF) << AAC_TID_SHIFT)
        | ((dev_count as u32 & 0x7F) << AAC_DEV_COUNT_SHIFT)
        | (if roc { TC_ROC_BIT } else { 0 })
        | (if toc { TC_TOC_BIT } else { 0 })
}

/// Compute the odd-parity bit for a 7-bit I3C dynamic address.
///
/// Per MIPI I3C specification: bit 7 of the DAT `DEV_DYNAMIC_ADDR` field is
/// the odd-parity bit over the 7-bit address (XOR of all bits, then inverted).
fn odd_parity(addr: u8) -> u8 {
    let v = addr & 0x7F;
    let xor = (v ^ (v >> 1) ^ (v >> 2) ^ (v >> 3) ^ (v >> 4) ^ (v >> 5) ^ (v >> 6)) & 1;
    ((!xor) & 1) << 7 // 1 = odd parity satisfied
}

fn dat_dynamic_addr_field(addr: u8) -> u8 {
    (addr & 0x7F) | odd_parity(addr)
}

// ── Config ────────────────────────────────────────────────────────────────────

/// I3C master configuration.
///
/// Default values target a 200 MHz I3C core clock (AST1060 default):
/// SCU CLK_SEL4[31:28] = 0b0100 → HPLL(1000 MHz) / 5 = 200 MHz.
pub struct I3cConfig {
    /// SCL open-drain HCNT (high period, core clock cycles).
    pub od_hcnt: u8,
    /// SCL open-drain LCNT (low period, core clock cycles).
    pub od_lcnt: u8,
    /// SCL push-pull HCNT (SDR0 = 12.5 MHz at 200 MHz core).
    pub pp_hcnt: u8,
    /// SCL push-pull LCNT.
    pub pp_lcnt: u8,
    /// SCL I2C FM HCNT (for legacy I2C devices).
    pub fm_hcnt: u16,
    /// SCL I2C FM LCNT.
    pub fm_lcnt: u16,
    /// SDA TX hold time (1–7 core clock cycles, i3c_v1.yaml SDA_HOLD).
    pub sda_tx_hold: u8,
    /// Enable 2 KΩ on-chip SDA pull-up (channels 1–4 only per global wrapper).
    pub pullup_2k: bool,
    /// Enable 750 Ω on-chip SDA pull-up.
    pub pullup_750: bool,
    /// Set `I2C_SLAVE_PRESENT` — must be `true` if any legacy I2C devices share the bus.
    pub i2c_slaves_present: bool,
}

impl Default for I3cConfig {
    fn default() -> Self {
        Self {
            // 200 MHz core clock → SDR0 (12.5 MHz PP): HCNT=LCNT=8 → 16 cycles = 80 ns
            pp_hcnt: 8,
            pp_lcnt: 8,
            // OD: ~3.6 MHz → HCNT=16 (80 ns), LCNT=40 (200 ns) → 280 ns period
            od_hcnt: 16,
            od_lcnt: 40,
            // I2C FM 400 kHz: period=2500 ns → HCNT=240 (1200 ns), LCNT=300 (1500 ns)
            fm_hcnt: 240,
            fm_lcnt: 300,
            sda_tx_hold: 2,
            pullup_2k: true,
            pullup_750: false,
            i2c_slaves_present: false,
        }
    }
}

// ── Error types ───────────────────────────────────────────────────────────────

/// I3C transfer error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum I3cError {
    /// Hardware returned a non-zero ERR_STATUS in the response queue.
    ///
    /// Values from i3c_v1.yaml RESP_PORT.ERR_STATUS:
    /// 1=CRC, 2=Parity, 3=Frame, 4=Broadcast NACK, 5=Addr NACK (ENTDAA),
    /// 6=Overflow/Underflow, 8=Aborted, 9=I2C NACK.
    TransferError(u8),
    /// DAT slot index out of range (must be 0–7).
    DatSlotOutOfRange,
    /// TX data exceeds the 4-byte FIFO word boundary (must be padded to 4 bytes).
    DataTooLarge,
    /// No response received from the hardware (timeout placeholder).
    NoResponse,
}

/// Result of one ENTDAA address assignment round.
#[derive(Debug, Copy, Clone, Default)]
pub struct DaaResult {
    /// Provisional ID [47:0] from the winning device.
    pub pid: u64,
    /// Bus Characteristic Register value.
    pub bcr: u8,
    /// Device Characteristic Register value.
    pub dcr: u8,
    /// Dynamic address assigned to this device.
    pub dynamic_addr: u8,
}

// ── I3cMaster ─────────────────────────────────────────────────────────────────

/// DWC I3C master controller driver.
pub struct I3cMaster {
    ch: u8,
    /// Next transaction ID (wraps at 15).
    tid: u8,
}

impl I3cMaster {
    /// Initialise I3C channel `ch` (0–3) as master.
    ///
    /// # Panics
    ///
    /// Panics if `ch` > 3.
    pub fn new(ch: u8, cfg: I3cConfig) -> Self {
        assert!(ch <= 3, "I3C channel must be 0–3");

        with_ch(ch, |r| {
            // 1. Ensure controller is disabled before configuring.
            r.DEVICE_CTRL().modify(|w| w.set_I3C_EN(false));

            // 2. Software-reset all queues and FIFOs (i3c_v1.yaml RESET_CTRL).
            r.RESET_CTRL().write(|w| {
                w.set_CORE_RST(true);
                w.set_CMD_QUEUE_RST(true);
                w.set_RESP_QUEUE_RST(true);
                w.set_TX_FIFO_RST(true);
                w.set_RX_FIFO_RST(true);
                w.set_IBI_QUEUE_RST(true);
            });
            // Resets are self-clearing; brief spin for hardware to apply.
            for _ in 0..100u32 {
                core::hint::spin_loop();
            }

            // 3. Configure SCL timing.
            r.SCL_I3C_OD_TIMING().write(|w| {
                w.set_HCNT(cfg.od_hcnt);
                w.set_LCNT(cfg.od_lcnt);
            });
            r.SCL_I3C_PP_TIMING().write(|w| {
                w.set_HCNT(cfg.pp_hcnt);
                w.set_LCNT(cfg.pp_lcnt);
            });
            r.SCL_I2C_FM_TIMING().write(|w| {
                w.set_I2C_FM_HCNT(cfg.fm_hcnt);
                w.set_I2C_FM_LCNT(cfg.fm_lcnt);
            });

            // 4. SDA hold + PP↔OD switching delays.
            r.SDA_HOLD_SW_DLY().write(|w| {
                w.set_SDA_TX_HOLD(cfg.sda_tx_hold);
                w.set_SDA_OD_PP_SWITCH_DLY(2);
                w.set_SDA_PP_OD_SWITCH_DLY(2);
            });

            // 5. Bus free timing: 200 MHz × 500 ns ≈ 100 cycles master free.
            r.BUS_FREE_TIMING().write(|w| {
                w.set_I3C_MST_FREE(100);
                w.set_I3C_IBI_FREE(50);
            });

            // 6. Queue thresholds: respond when ≥1 response entry ready.
            r.QUEUE_THLD_CTRL().write(|w| {
                w.set_RESP_BUF_THLD(1);
                w.set_CMD_EMPTY_BUF_THLD(1);
                w.set_IBI_STATUS_THLD(1);
            });

            // 7. Data buffer thresholds.
            r.DATA_BUF_THLD_CTRL().write(|w| {
                w.set_TX_EMPTY_BUF_THLD(1);
                w.set_RX_BUF_THLD(1);
                w.set_TX_START_THLD(1);
                w.set_RX_START_THLD(1);
            });

            // 8. Enable RESP_READY interrupt signal (INTR bit 4 per INTR fieldset).
            r.INTR_STATUS_EN().write(|w| {
                w.set_RESP_READY(true);
                w.set_TRANSFER_ERR(true);
                w.set_TRANSFER_ABORT(true);
            });
            r.INTR_SIGNAL_EN().write(|w| {
                w.set_RESP_READY(true);
                w.set_TRANSFER_ERR(true);
            });

            // 9. Device control: master mode, I2C slave present if configured.
            r.DEVICE_CTRL().write(|w| {
                w.set_I2C_SLAVE_PRESENT(cfg.i2c_slaves_present);
                w.set_HOT_JOIN_CTRL(true); // NACK hot-join (we control DAA)
                w.set_I3C_EN(true);
            });

            // 10. Set operation mode to master.
            r.DEVICE_CTRL_EXT().write(|w| {
                w.set_DEV_OPERATION_MODE(0); // 0 = Master
            });
        });

        // 11. Configure global wrapper PHY (SDA pull-ups, de-glitch).
        // i3cglobal_v1.yaml: CH1=I3C0, CH2=I3C1, CH3=I3C2, CH4=I3C3.
        // Channels 1-4 have pull-up control bits.
        let g = pac::I3C_GLOBAL;
        match ch {
            0 => g.CH1_PHY().modify(|w| {
                w.set_SDA_PULLUP_EN_2K(cfg.pullup_2k);
                w.set_SDA_PULLUP_EN_750(cfg.pullup_750);
            }),
            1 => g.CH2_PHY().modify(|w| {
                w.set_SDA_PULLUP_EN_2K(cfg.pullup_2k);
                w.set_SDA_PULLUP_EN_750(cfg.pullup_750);
            }),
            2 => g.CH3_PHY().modify(|w| {
                w.set_SDA_PULLUP_EN_2K(cfg.pullup_2k);
                w.set_SDA_PULLUP_EN_750(cfg.pullup_750);
            }),
            3 => g.CH4_PHY().modify(|w| {
                w.set_SDA_PULLUP_EN_2K(cfg.pullup_2k);
                w.set_SDA_PULLUP_EN_750(cfg.pullup_750);
            }),
            _ => {}
        }

        Self { ch, tid: 0 }
    }

    // ── DAT table management ──────────────────────────────────────────────────

    /// Register a device in DAT slot `slot` (0–7) with a 7-bit dynamic address.
    ///
    /// Optionally set `legacy_i2c = true` for legacy I2C-only devices.
    ///
    /// # Errors
    ///
    /// Returns `I3cError::DatSlotOutOfRange` if `slot` > 7.
    pub fn add_device(&mut self, slot: u8, dyn_addr: u8, legacy_i2c: bool) -> Result<(), I3cError> {
        if slot > 7 {
            return Err(I3cError::DatSlotOutOfRange);
        }
        with_ch(self.ch, |r| {
            write_dat(r, slot, |w| {
                w.set_DEV_DYNAMIC_ADDR(dat_dynamic_addr_field(dyn_addr));
                w.set_LEGACY_I2C_DEVICE(legacy_i2c);
                w.set_SIR_REJECT(true); // reject SIR by default (can enable later)
                w.set_MR_REJECT(true);
            });
        });
        Ok(())
    }

    /// Clear a DAT slot.
    pub fn remove_device(&mut self, slot: u8) {
        if slot > 7 {
            return;
        }
        with_ch(self.ch, |r| {
            write_dat(r, slot, |w| {
                w.0 = 0;
            });
        });
    }

    // ── ENTDAA ────────────────────────────────────────────────────────────────

    /// Run the ENTDAA procedure.
    ///
    /// Assigns dynamic addresses to all I3C devices on the bus that respond
    /// to the broadcast address assignment command.  DAT slots 0 through
    /// `results.len()-1` are populated.
    ///
    /// Returns the number of devices assigned.
    ///
    /// # Errors
    ///
    /// Returns `I3cError::TransferError` if the hardware reports an error
    /// other than `ADDR_NACK` (which signals no more devices).
    pub async fn run_entdaa(&mut self, results: &mut [DaaResult]) -> Result<u8, I3cError> {
        let max = results.len().min(8) as u8;
        if max == 0 {
            return Ok(0);
        }

        let tid = self.next_tid();

        // Pre-populate DAT slots with the addresses we will assign.
        // Simple policy: assign addr = 0x08 + slot (avoids reserved addresses).
        for slot in 0..max {
            self.add_device(slot, 0x08 + slot, false)?;
        }

        // Push Address Assignment Command (AAC).
        with_ch(self.ch, |r| {
            r.CMD_QUEUE_PORT().write(|w| {
                w.set_DATA(aac_word(tid, max, true, true) >> 3); // DATA = bits[31:3]
                w.set_CMD_ATTR(CMD_ATTR_AAC as u8);
            });
        });

        // Await response.
        let resp = RespReadyFuture { ch: self.ch }.await;
        let err = ((resp >> 28) & 0xF) as u8;
        let dev_count = (resp & 0xFFFF) as u8;

        // ERR_STATUS=5 = "Addr NACK (ENTDAA)" means no more devices — treat as OK.
        if err != 0 && err != 5 {
            return Err(I3cError::TransferError(err));
        }

        // Read DCT entries for each assigned device.
        with_ch(self.ch, |r| {
            for slot in 0..dev_count.min(max) as usize {
                let (pid_lo, pid_hi, bcr_dcr, dyn_addr) = read_dct(r, slot);
                results[slot] = DaaResult {
                    pid: (pid_lo as u64) | ((pid_hi as u64) << 32),
                    bcr: ((bcr_dcr >> 8) & 0xFF) as u8,
                    dcr: (bcr_dcr & 0xFF) as u8,
                    dynamic_addr: (dyn_addr & 0x7F) as u8,
                };
            }
        });

        Ok(dev_count.min(max))
    }

    // ── SDR write ─────────────────────────────────────────────────────────────

    /// Write `data` to the device at DAT `slot`.
    ///
    /// `data` length may be 1–8191 bytes.  For MCTP use 64–256 bytes per call.
    ///
    /// # Errors
    ///
    /// `I3cError::DatSlotOutOfRange` if slot > 7.
    /// `I3cError::TransferError(code)` on hardware error.
    pub async fn write(&mut self, slot: u8, data: &[u8]) -> Result<(), I3cError> {
        if slot > 7 {
            return Err(I3cError::DatSlotOutOfRange);
        }
        let len = data.len() as u16;
        let tid = self.next_tid();

        with_ch(self.ch, |r| {
            // TARG: specifies data length.
            r.CMD_QUEUE_PORT().write(|w| {
                w.set_CMD_ATTR(CMD_ATTR_TARG as u8);
                w.set_DATA(targ_word(len) >> 3);
            });
            // TC: write to device at `slot`, SDR0, with STOP.
            r.CMD_QUEUE_PORT().write(|w| {
                let word = tc_word(tid, slot, false, true, true);
                w.set_CMD_ATTR(CMD_ATTR_TC as u8);
                w.set_DATA(word >> 3);
            });

            // Push TX data words (4 bytes each, padded with zeros).
            let mut i = 0;
            while i < data.len() {
                let w0 = *data.get(i).unwrap_or(&0) as u32;
                let w1 = *data.get(i + 1).unwrap_or(&0) as u32;
                let w2 = *data.get(i + 2).unwrap_or(&0) as u32;
                let w3 = *data.get(i + 3).unwrap_or(&0) as u32;
                let word = w0 | (w1 << 8) | (w2 << 16) | (w3 << 24);
                r.TX_RX_DATA_PORT().write(|w| w.set_DATA(word));
                i += 4;
            }
        });

        let resp = RespReadyFuture { ch: self.ch }.await;
        let err = ((resp >> 28) & 0xF) as u8;
        if err != RESP_ERR_OK as u8 {
            Err(I3cError::TransferError(err))
        } else {
            Ok(())
        }
    }

    // ── SDR read ──────────────────────────────────────────────────────────────

    /// Read up to `buf.len()` bytes from the device at DAT `slot`.
    ///
    /// Returns the number of bytes actually received.
    pub async fn read(&mut self, slot: u8, buf: &mut [u8]) -> Result<usize, I3cError> {
        if slot > 7 {
            return Err(I3cError::DatSlotOutOfRange);
        }
        let len = buf.len() as u16;
        let tid = self.next_tid();

        with_ch(self.ch, |r| {
            // TARG: expected byte count.
            r.CMD_QUEUE_PORT().write(|w| {
                w.set_CMD_ATTR(CMD_ATTR_TARG as u8);
                w.set_DATA(targ_word(len) >> 3);
            });
            // TC: read from device at `slot`, SDR0, with STOP.
            r.CMD_QUEUE_PORT().write(|w| {
                let word = tc_word(tid, slot, true, true, true);
                w.set_CMD_ATTR(CMD_ATTR_TC as u8);
                w.set_DATA(word >> 3);
            });
        });

        let resp = RespReadyFuture { ch: self.ch }.await;
        let err = ((resp >> 28) & 0xF) as u8;
        if err != RESP_ERR_OK as u8 {
            return Err(I3cError::TransferError(err));
        }

        let received = (resp & 0xFFFF) as usize;
        let to_copy = received.min(buf.len());

        // Pop RX FIFO words.
        with_ch(self.ch, |r| {
            let mut i = 0;
            while i < to_copy {
                let word = r.TX_RX_DATA_PORT().read().DATA();
                for byte_idx in 0..4usize {
                    if i + byte_idx < to_copy {
                        buf[i + byte_idx] = ((word >> (byte_idx * 8)) & 0xFF) as u8;
                    }
                }
                i += 4;
            }
        });

        Ok(to_copy)
    }

    // ── Broadcast CCC write ───────────────────────────────────────────────────

    /// Send a broadcast CCC (Common Command Code) with optional data.
    ///
    /// Sends: START + 0x7E (broadcast address) + CCC byte + [data bytes] + STOP.
    ///
    /// Common CCCs: `0x06`=DISEC, `0x07`=ENEC, `0x02`=RSTDAA.
    pub async fn write_bcast_ccc(&mut self, ccc: u8, data: &[u8]) -> Result<(), I3cError> {
        let len = data.len() as u16;
        let tid = self.next_tid();

        with_ch(self.ch, |r| {
            if !data.is_empty() {
                r.CMD_QUEUE_PORT().write(|w| {
                    w.set_CMD_ATTR(CMD_ATTR_TARG as u8);
                    w.set_DATA(targ_word(len) >> 3);
                });
            }
            r.CMD_QUEUE_PORT().write(|w| {
                let word = tc_ccc_bcast(tid, ccc, true, true);
                w.set_CMD_ATTR(CMD_ATTR_TC as u8);
                w.set_DATA(word >> 3);
            });
            let mut i = 0;
            while i < data.len() {
                let w0 = *data.get(i).unwrap_or(&0) as u32;
                let w1 = *data.get(i + 1).unwrap_or(&0) as u32;
                let w2 = *data.get(i + 2).unwrap_or(&0) as u32;
                let w3 = *data.get(i + 3).unwrap_or(&0) as u32;
                r.TX_RX_DATA_PORT()
                    .write(|w| w.set_DATA(w0 | (w1 << 8) | (w2 << 16) | (w3 << 24)));
                i += 4;
            }
        });

        let resp = RespReadyFuture { ch: self.ch }.await;
        let err = ((resp >> 28) & 0xF) as u8;
        if err != RESP_ERR_OK as u8 {
            Err(I3cError::TransferError(err))
        } else {
            Ok(())
        }
    }

    // ── Internal ─────────────────────────────────────────────────────────────

    fn next_tid(&mut self) -> u8 {
        let t = self.tid;
        self.tid = (self.tid + 1) & 0xF;
        t
    }

    /// Called from the per-channel ISR.
    pub(crate) fn on_interrupt(ch: u8) {
        if (ch as usize) < 4 {
            // Clear interrupt status (RESP_READY is level-based, auto-clears;
            // error bits are sticky RW1C — clear them here).
            with_ch(ch, |r| {
                let sts = r.INTR_STATUS().read();
                if sts.TRANSFER_ERR() || sts.TRANSFER_ABORT() {
                    r.INTR_STATUS().write(|w| {
                        w.set_TRANSFER_ERR(true);
                        w.set_TRANSFER_ABORT(true);
                    });
                }
            });
            I3C_WAKERS[ch as usize].wake();
        }
    }
}

// ── RespReadyFuture ───────────────────────────────────────────────────────────

/// Future that resolves with the raw 32-bit response queue entry when
/// `INTR_STATUS.RESP_READY` is set.
struct RespReadyFuture {
    ch: u8,
}

impl Future for RespReadyFuture {
    type Output = u32;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<u32> {
        // Check if a response is already available.
        let ready = with_ch_read(self.ch, |r| {
            r.QUEUE_STATUS_LEVEL().read().RESP_BUF_BLR() > 0
        });
        if ready {
            let resp = with_ch_read(self.ch, |r| r.RESP_QUEUE_PORT().read().0);
            return Poll::Ready(resp);
        }

        I3C_WAKERS[self.ch as usize].register(cx.waker());

        // Re-check after waker registration to close the race.
        let ready2 = with_ch_read(self.ch, |r| {
            r.QUEUE_STATUS_LEVEL().read().RESP_BUF_BLR() > 0
        });
        if ready2 {
            let resp = with_ch_read(self.ch, |r| r.RESP_QUEUE_PORT().read().0);
            Poll::Ready(resp)
        } else {
            Poll::Pending
        }
    }
}

// ── Channel dispatch helpers ──────────────────────────────────────────────────

fn with_ch<F>(ch: u8, f: F)
where
    F: FnOnce(pac::i3c_v1::I3C),
{
    match ch {
        0 => f(pac::I3C0),
        1 => f(pac::I3C1),
        2 => f(pac::I3C2),
        3 => f(pac::I3C3),
        _ => {}
    }
}

fn with_ch_read<F, T: Default>(ch: u8, f: F) -> T
where
    F: FnOnce(pac::i3c_v1::I3C) -> T,
{
    match ch {
        0 => f(pac::I3C0),
        1 => f(pac::I3C1),
        2 => f(pac::I3C2),
        3 => f(pac::I3C3),
        _ => T::default(),
    }
}

// ── DAT table helpers ────────────────────────────────────────────────────────

fn write_dat<F>(r: pac::i3c_v1::I3C, slot: u8, f: F)
where
    F: FnOnce(&mut pac::i3c_v1::I3C_DAT_ENTRY),
{
    match slot {
        0 => r.DAT_DEV1().modify(f),
        1 => r.DAT_DEV2().modify(f),
        2 => r.DAT_DEV3().modify(f),
        3 => r.DAT_DEV4().modify(f),
        4 => r.DAT_DEV5().modify(f),
        5 => r.DAT_DEV6().modify(f),
        6 => r.DAT_DEV7().modify(f),
        7 => r.DAT_DEV8().modify(f),
        _ => {}
    }
}

/// Read DCT entry for `slot` (0-based). Returns (PID_LO, PID_HI, BCR_DCR, DYN_ADDR).
///
/// `PID_HI` is a 16-bit field (i3c_v1.yaml I3C_DCT_PID_HI.PID_HI);
/// `DEV_DYNAMIC_ADDR` is an 8-bit field (I3C_DCT_DYN_ADDR.DEV_DYNAMIC_ADDR).
/// Both are cast to u32 to match the return tuple type.
fn read_dct(r: pac::i3c_v1::I3C, slot: usize) -> (u32, u32, u32, u32) {
    macro_rules! dct {
        ($l1:ident, $l2:ident, $l3:ident, $l4:ident) => {
            (
                r.$l1().read().PID_LO(),
                r.$l2().read().PID_HI() as u32,
                r.$l3().read().0,
                r.$l4().read().DEV_DYNAMIC_ADDR() as u32,
            )
        };
    }
    match slot {
        0 => dct!(DCT_DEV1_LOC1, DCT_DEV1_LOC2, DCT_DEV1_LOC3, DCT_DEV1_LOC4),
        1 => dct!(DCT_DEV2_LOC1, DCT_DEV2_LOC2, DCT_DEV2_LOC3, DCT_DEV2_LOC4),
        2 => dct!(DCT_DEV3_LOC1, DCT_DEV3_LOC2, DCT_DEV3_LOC3, DCT_DEV3_LOC4),
        3 => dct!(DCT_DEV4_LOC1, DCT_DEV4_LOC2, DCT_DEV4_LOC3, DCT_DEV4_LOC4),
        4 => dct!(DCT_DEV5_LOC1, DCT_DEV5_LOC2, DCT_DEV5_LOC3, DCT_DEV5_LOC4),
        5 => dct!(DCT_DEV6_LOC1, DCT_DEV6_LOC2, DCT_DEV6_LOC3, DCT_DEV6_LOC4),
        6 => dct!(DCT_DEV7_LOC1, DCT_DEV7_LOC2, DCT_DEV7_LOC3, DCT_DEV7_LOC4),
        7 => dct!(DCT_DEV8_LOC1, DCT_DEV8_LOC2, DCT_DEV8_LOC3, DCT_DEV8_LOC4),
        _ => (0, 0, 0, 0),
    }
}

// ── Interrupt handlers — IRQs 102–105 ────────────────────────────────────────

macro_rules! i3c_irq {
    ($name:ident, $ch:expr) => {
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn $name() {
            I3cMaster::on_interrupt($ch);
        }
    };
}

i3c_irq!(I3C0, 0);
i3c_irq!(I3C1, 1);
i3c_irq!(I3C2, 2);
i3c_irq!(I3C3, 3);
