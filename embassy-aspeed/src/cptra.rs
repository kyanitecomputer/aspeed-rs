//! Caliptra security subsystem driver for the AST2700 BootMCU.
//!
//! Caliptra is a hardware Root of Trust IP integrated into the AST2700 IO-die.
//! The BootMCU communicates with Caliptra via a register-based mailbox protocol.
//!
//! # Hardware blocks
//!
//! | Block | Base | Description |
//! |-------|------|-------------|
//! | Mailbox | `0x14C60000` | Command/data transport |
//! | SHA | `0x14C61000` | SHA384/SHA512 accelerator |
//! | IFC | `0x14C70000` | Boot status, flow status, TRNG, errors |
//!
//! # Mailbox protocol
//!
//! 1. Acquire lock → check FSM is `RDY_FOR_CMD`
//! 2. Compute subtractive checksum over command + payload
//! 3. Write CMD, DLEN, DATAIN (checksum word first, then payload words), EXEC
//! 4. Poll STS until status ≠ `CMD_BUSY`
//! 5. If `DATA_READY`: read DLEN, drain output from DATAOUT **before** unlock
//! 6. Release lock
//!
//! # Sync and async
//!
//! All mailbox transactions have sync and async variants:
//! - `transaction_*` / `transaction_*_async` — internal core
//! - `fw_info` / `fw_info_async` — high-level commands
//!
//! Async variants use `poll_until_async` to yield to the Embassy executor
//! instead of busy-waiting on the mailbox STATUS register.

use crate::pac;
use aspeed_mmio::MmioBlock;

// ── Base addresses ────────────────────────────────────────────────────────────

const MBOX_BASE: usize = 0x14C6_0000;
const IFC_BASE: usize = 0x14C7_0000;

// ── Mailbox register offsets ──────────────────────────────────────────────────

const MBOX_LOCK: usize = 0x00;
#[allow(dead_code)]
const MBOX_USER: usize = 0x04;
const MBOX_CMD: usize = 0x08;
const MBOX_DLEN: usize = 0x0C;
const MBOX_DATAIN: usize = 0x10;
const MBOX_DATAOUT: usize = 0x14;
const MBOX_EXEC: usize = 0x18;
const MBOX_STS: usize = 0x1C;
#[allow(dead_code)]
const MBOX_UNLOCK: usize = 0x20;

const STS_PS_MASK: u32 = 0x0F;
const STS_FSM_PS_SHIFT: u32 = 6;
const STS_FSM_PS_MASK: u32 = 0x07 << STS_FSM_PS_SHIFT;
const STS_SOC_LOCK: u32 = 1 << 9;

// ── IFC register offsets ──────────────────────────────────────────────────────

const IFC_HW_ERROR_FATAL: usize = 0x000;
const IFC_HW_ERROR_NONFATAL: usize = 0x004;
const IFC_FW_ERROR_FATAL: usize = 0x008;
const IFC_FW_ERROR_NONFATAL: usize = 0x00C;
const IFC_BOOT_STS: usize = 0x038;
const IFC_FLOW_STS: usize = 0x03C;
const IFC_RST_REASON: usize = 0x040;
const IFC_TRNG_DATA: usize = 0x078;
const IFC_TRNG_STS: usize = 0x0AC;

const FLOW_STS_RDY_FOR_FUSES: u32 = 1 << 30;
const FLOW_STS_RDY_FOR_RT: u32 = 1 << 29;
const FLOW_STS_RDY_FOR_FW: u32 = 1 << 28;
const TRNG_STS_DATA_REQ: u32 = 1 << 0;
const TRNG_STS_DATA_WR_DONE: u32 = 1 << 1;

const MAX_TRNG_WORDS: usize = 12;
const MBOX_POLL_LOOPS: u32 = 10_000_000;

// ── MmioBlock helpers ─────────────────────────────────────────────────────────

#[inline]
fn mbox() -> MmioBlock {
    unsafe { MmioBlock::new(MBOX_BASE) }
}

#[inline]
fn ifc() -> MmioBlock {
    unsafe { MmioBlock::new(IFC_BASE) }
}

// ── Mailbox commands ──────────────────────────────────────────────────────────

pub mod cmd {
    pub const FW_INFO: u32 = 0x494E_464F;
    pub const CAPABILITIES: u32 = 0x4341_5053;
    pub const FIPS_VERSION: u32 = 0x4650_5652;
    pub const SELF_TEST_START: u32 = 0x4650_4C54;
    pub const SELF_TEST_GET_RESULTS: u32 = 0x4650_4C67;
    pub const SHUTDOWN: u32 = 0x4650_5344;
    pub const STASH_MEASUREMENT: u32 = 0x4D45_4153;
    pub const QUOTE_PCRS: u32 = 0x5043_5251;
    pub const GET_IDEV_CERT: u32 = 0x4944_4543;
    pub const POPULATE_IDEV_CERT: u32 = 0x4944_4550;
    pub const GET_LDEV_CERT: u32 = 0x4C44_4556;
    pub const GET_FMC_ALIAS_CERT: u32 = 0x4345_5246;
    pub const GET_RT_ALIAS_CERT: u32 = 0x4345_5252;
    pub const INVOKE_DPE_COMMAND: u32 = 0x4450_4543;
    pub const DISABLE_ATTESTATION: u32 = 0x4453_424C;
    pub const SET_AUTH_MANIFEST: u32 = 0x4154_4D4E;
    pub const AUTHORIZE_AND_STASH: u32 = 0x4154_5348;
    pub const EXTEND_PCR: u32 = 0x5043_5245;
    pub const FW_LOAD: u32 = 0x4657_4C44;
}

pub const MAX_AUTH_MANIFEST_SIZE: usize = 34 * 1024;
pub const MAX_IDEVID_ECC384_CERT_SIZE: usize = 1024;
pub const MAX_IDEVID_ECC384_TBS_SIZE: usize = 916;

// ── AUTHORIZE_AND_STASH ─────────────────────────────────────────────────────
// Layout per caliptra-sw api/src/mailbox.rs `AuthorizeAndStashReq` at the
// pinned AST2700-A2 revision (879608b): after the checksum header the request
// carries fw_id[4] + measurement[48] + context[48] + svn + flags + source +
// image_size (116 payload bytes). The response is auth_req_result (u32).

/// SHA-384 image digest length used by AUTHORIZE_AND_STASH.
pub const IMAGE_DIGEST_SIZE: usize = 48;

/// `auth_req_result`: image authorized (fw_id + digest matched, or the metadata
/// entry has `ignore_auth_check`).
pub const IMAGE_AUTHORIZED: u32 = 0xDEAD_C0DE;
/// `auth_req_result`: fw_id not found in the image-metadata collection.
pub const IMAGE_NOT_AUTHORIZED: u32 = 0x2152_3F21;

/// `flags` bit: skip stashing the measurement into DPE (authorization only).
pub const AUTH_FLAG_SKIP_STASH: u32 = 0x1;

/// Where Caliptra sources the image bytes to hash for AUTHORIZE_AND_STASH.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u32)]
pub enum ImageHashSource {
    /// The caller supplies the precomputed digest in `measurement`.
    InRequest = 1,
    /// Caliptra hashes `image_size` bytes at the image's load address.
    LoadAddress = 2,
    /// Caliptra hashes `image_size` bytes at the image's staging address.
    StagingAddress = 3,
}

/// Verdict returned by AUTHORIZE_AND_STASH.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AuthResult {
    /// Image authorized.
    Authorized,
    /// fw_id not present in the manifest metadata / digest mismatch.
    NotAuthorized,
    /// Unrecognized result code.
    Unknown(u32),
}

impl AuthResult {
    fn from_raw(v: u32) -> Self {
        match v {
            IMAGE_AUTHORIZED => Self::Authorized,
            IMAGE_NOT_AUTHORIZED => Self::NotAuthorized,
            other => Self::Unknown(other),
        }
    }

    /// True only for [`AuthResult::Authorized`].
    pub fn is_authorized(self) -> bool {
        matches!(self, Self::Authorized)
    }
}

/// Little-endian request fields for AUTHORIZE_AND_STASH (excluding the checksum
/// header and the 48-byte digest, which the transaction supplies separately).
struct AuthStashParts {
    fw_id: [u8; 4],
    context: [u8; IMAGE_DIGEST_SIZE],
    svn: [u8; 4],
    flags: [u8; 4],
    source: [u8; 4],
    image_size: [u8; 4],
}

// ── Status types ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MboxStatus {
    CmdBusy = 0,
    DataReady = 1,
    CmdComplete = 2,
    CmdFailure = 3,
}

impl MboxStatus {
    fn from_raw(v: u32) -> Self {
        match v & STS_PS_MASK {
            0 => Self::CmdBusy,
            1 => Self::DataReady,
            2 => Self::CmdComplete,
            _ => Self::CmdFailure,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MboxFsm {
    Idle = 0,
    RdyForCmd = 1,
    RdyForDlen = 2,
    RdyForData = 3,
    ExecUc = 4,
    ExecSoc = 5,
    Error = 6,
}

impl MboxFsm {
    fn from_raw(v: u32) -> Self {
        match (v & STS_FSM_PS_MASK) >> STS_FSM_PS_SHIFT {
            0 => Self::Idle,
            1 => Self::RdyForCmd,
            2 => Self::RdyForDlen,
            3 => Self::RdyForData,
            4 => Self::ExecUc,
            5 => Self::ExecSoc,
            _ => Self::Error,
        }
    }
}

// ── IFC status types ──────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy)]
pub struct BootStatus(pub u32);

#[derive(Debug, Clone, Copy)]
pub struct FlowStatus(pub u32);

impl FlowStatus {
    pub fn rdy_for_fuses(&self) -> bool { self.0 & FLOW_STS_RDY_FOR_FUSES != 0 }
    pub fn rdy_for_rt(&self) -> bool    { self.0 & FLOW_STS_RDY_FOR_RT != 0 }
    pub fn rdy_for_fw(&self) -> bool    { self.0 & FLOW_STS_RDY_FOR_FW != 0 }
}

#[derive(Debug, Clone, Copy)]
pub struct ResetReason(pub u32);

impl ResetReason {
    pub fn fw_update_reset(&self) -> bool { self.0 & (1 << 0) != 0 }
    pub fn warm_reset(&self) -> bool      { self.0 & (1 << 1) != 0 }
}

// ── Response types ────────────────────────────────────────────────────────────

#[derive(Debug)]
#[repr(C)]
pub struct FwInfoResponse {
    pub checksum: u32,
    pub fips_status: u32,
    pub pl0_pauser: u32,
    pub runtime_svn: u32,
    pub min_runtime_svn: u32,
    pub fmc_manifest_svn: u32,
    pub attestation_disabled: u32,
    pub rom_revision: [u8; 20],
    pub fmc_revision: [u8; 20],
    pub runtime_revision: [u8; 20],
    pub rom_sha256_digest: [u32; 8],
    pub fmc_sha384_digest: [u32; 12],
    pub runtime_sha384_digest: [u32; 12],
    pub owner_pub_key_hash: [u32; 12],
}

#[derive(Debug)]
#[repr(C)]
pub struct CapabilitiesResponse {
    pub checksum: u32,
    pub fips_status: u32,
    pub capabilities: [u8; 16],
}

#[derive(Debug)]
#[repr(C)]
pub struct FipsVersionResponse {
    pub checksum: u32,
    pub fips_status: u32,
    pub mode: u32,
    pub fips_rev: [u32; 3],
    pub name: [u8; 12],
}

#[derive(Debug)]
#[repr(C)]
pub struct SelfTestResponse {
    pub checksum: u32,
    pub fips_status: u32,
}

#[derive(Debug)]
#[repr(C)]
pub struct ShutdownResponse {
    pub checksum: u32,
    pub fips_status: u32,
}

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CptraError {
    MboxBusy,
    NotReady,
    CmdFailed,
    Timeout,
    HwFatal(u32),
    InvalidLength,
}

// ── Subtractive checksum ──────────────────────────────────────────────────────

pub fn checksum(csum: u32, data: &[u8]) -> u32 {
    let mut v = csum;
    for &b in data {
        v = v.wrapping_sub(b as u32);
    }
    v
}

// ── Caliptra driver ───────────────────────────────────────────────────────────

pub struct Caliptra;

impl Caliptra {
    // ── IFC queries ────────────────────────────────────────────────────────

    pub fn boot_status() -> BootStatus {
        BootStatus(ifc().read32(IFC_BOOT_STS))
    }

    pub fn flow_status() -> FlowStatus {
        FlowStatus(ifc().read32(IFC_FLOW_STS))
    }

    pub fn reset_reason() -> ResetReason {
        ResetReason(ifc().read32(IFC_RST_REASON))
    }

    pub fn errors() -> (u32, u32, u32, u32) {
        let ifc = ifc();
        (
            ifc.read32(IFC_HW_ERROR_FATAL),
            ifc.read32(IFC_HW_ERROR_NONFATAL),
            ifc.read32(IFC_FW_ERROR_FATAL),
            ifc.read32(IFC_FW_ERROR_NONFATAL),
        )
    }

    /// Poll `FLOW_STS.RDY_FOR_FW` until set or `timeout_loops` exhausted.
    pub fn wait_rdy_for_fw(timeout_loops: u32) -> Result<(), CptraError> {
        aspeed_mmio::poll_until(
            || Self::flow_status(),
            |s| s.rdy_for_fw(),
            timeout_loops,
        ).map(|_| ()).map_err(|_| CptraError::Timeout)
    }

    pub fn is_rdy_for_rt() -> bool {
        if Self::flow_status().rdy_for_rt() {
            return true;
        }
        pac::SCU1.CPTRA_CTRL().read().RDY_FOR_RT()
    }

    /// Poll until Caliptra runtime is ready, feeding TRNG entropy each iteration.
    ///
    /// Spins and provides entropy on every TRNG request. Errors on hw_fatal or
    /// timeout.
    pub fn wait_rdy_for_rt_with_trng(
        entropy_fn: impl Fn() -> [u32; MAX_TRNG_WORDS],
        max_loops: usize,
    ) -> Result<(), CptraError> {
        for _ in 0..max_loops {
            let hw_fatal = ifc().read32(IFC_HW_ERROR_FATAL);
            if hw_fatal != 0 {
                return Err(CptraError::HwFatal(hw_fatal));
            }
            if Self::is_rdy_for_rt() {
                return Ok(());
            }
            let trng_sts = ifc().read32(IFC_TRNG_STS);
            if trng_sts & TRNG_STS_DATA_REQ != 0 {
                let entropy = entropy_fn();
                let mut ifc_w = ifc();
                for i in 0..MAX_TRNG_WORDS {
                    ifc_w.write32(IFC_TRNG_DATA + i * 4, entropy[i]);
                }
                ifc_w.write32(IFC_TRNG_STS, TRNG_STS_DATA_WR_DONE);
            }
            core::hint::spin_loop();
        }
        Err(CptraError::Timeout)
    }

    /// Async version: yield to executor between polls while feeding TRNG.
    pub async fn wait_rdy_for_rt_with_trng_async(
        entropy_fn: impl Fn() -> [u32; MAX_TRNG_WORDS],
        timeout: embassy_time::Duration,
    ) -> Result<(), CptraError> {
        use embassy_time::{Instant, Timer};
        let deadline = Instant::now() + timeout;
        loop {
            let hw_fatal = ifc().read32(IFC_HW_ERROR_FATAL);
            if hw_fatal != 0 {
                return Err(CptraError::HwFatal(hw_fatal));
            }
            if Self::is_rdy_for_rt() {
                return Ok(());
            }
            let trng_sts = ifc().read32(IFC_TRNG_STS);
            if trng_sts & TRNG_STS_DATA_REQ != 0 {
                let entropy = entropy_fn();
                let mut ifc_w = ifc();
                for i in 0..MAX_TRNG_WORDS {
                    ifc_w.write32(IFC_TRNG_DATA + i * 4, entropy[i]);
                }
                ifc_w.write32(IFC_TRNG_STS, TRNG_STS_DATA_WR_DONE);
            }
            if Instant::now() >= deadline {
                return Err(CptraError::Timeout);
            }
            Timer::after(embassy_time::Duration::from_micros(100)).await;
        }
    }

    /// Feed TRNG entropy if requested.  Returns `true` if entropy was provided.
    pub fn feed_trng(entropy: &[u32; MAX_TRNG_WORDS]) -> bool {
        let sts = ifc().read32(IFC_TRNG_STS);
        if sts & TRNG_STS_DATA_REQ == 0 {
            return false;
        }
        let mut ifc_w = ifc();
        for i in 0..MAX_TRNG_WORDS {
            ifc_w.write32(IFC_TRNG_DATA + i * 4, entropy[i]);
        }
        ifc_w.write32(IFC_TRNG_STS, TRNG_STS_DATA_WR_DONE);
        true
    }

    // ── Mailbox low-level ──────────────────────────────────────────────────

    pub fn mbox_status_raw() -> u32 {
        mbox().read32(MBOX_STS)
    }

    pub fn mbox_fsm() -> MboxFsm {
        MboxFsm::from_raw(Self::mbox_status_raw())
    }

    pub fn mbox_lock() -> Result<(), CptraError> {
        let sts = Self::mbox_status_raw();
        if sts & STS_SOC_LOCK != 0 {
            return Ok(());
        }
        let lock = mbox().read32(MBOX_LOCK);
        if lock & 1 != 0 {
            return Err(CptraError::MboxBusy);
        }
        Ok(())
    }

    pub fn mbox_unlock() {
        let sts = Self::mbox_status_raw();
        if sts & STS_SOC_LOCK == 0 {
            return;
        }
        if MboxStatus::from_raw(sts) == MboxStatus::CmdBusy {
            return;
        }
        let mut mb = mbox();
        mb.write32(MBOX_EXEC, 0);
    }

    pub fn mbox_dump() -> [u32; 9] {
        let mb = mbox();
        [
            mb.read32(0x00), mb.read32(0x04), mb.read32(0x08),
            mb.read32(0x0C), mb.read32(0x10), mb.read32(0x14),
            mb.read32(0x18), mb.read32(0x1C), mb.read32(0x20),
        ]
    }

    // ── Core mailbox transaction (sync) ───────────────────────────────────

    fn transaction(cmd: u32, input: &[u8], out_buf: &mut [u32]) -> Result<usize, CptraError> {
        Self::transaction_parts(cmd, &[input], out_buf)
    }

    fn transaction_parts(
        cmd: u32,
        input_parts: &[&[u8]],
        out_buf: &mut [u32],
    ) -> Result<usize, CptraError> {
        Self::mbox_lock()?;

        let sts = Self::mbox_status_raw();
        if MboxFsm::from_raw(sts) != MboxFsm::RdyForCmd {
            Self::mbox_unlock();
            return Err(CptraError::NotReady);
        }

        let (csum, input_len) = compute_checksum(cmd, input_parts);

        let mut mb = mbox();
        mb.write32(MBOX_CMD, cmd);
        mb.write32(MBOX_DLEN, 4 + input_len as u32);
        mb.write32(MBOX_DATAIN, csum);
        write_input_parts_mmio(&mut mb, input_parts);
        mb.write32(MBOX_EXEC, 1);

        let mb_sts = match aspeed_mmio::poll_until(
            || MboxStatus::from_raw(Self::mbox_status_raw()),
            |s| *s != MboxStatus::CmdBusy,
            MBOX_POLL_LOOPS,
        ) {
            Ok(s) => s,
            Err(_) => { Self::mbox_unlock(); return Err(CptraError::Timeout); }
        };

        let words_read = Self::drain_output(mb_sts, out_buf)?;
        Self::mbox_unlock();

        if mb_sts == MboxStatus::CmdFailure {
            return Err(CptraError::CmdFailed);
        }
        Ok(words_read)
    }

    fn transaction_bytes_parts(
        cmd: u32,
        input_parts: &[&[u8]],
        out_buf: &mut [u8],
    ) -> Result<usize, CptraError> {
        Self::mbox_lock()?;

        let sts = Self::mbox_status_raw();
        if MboxFsm::from_raw(sts) != MboxFsm::RdyForCmd {
            Self::mbox_unlock();
            return Err(CptraError::NotReady);
        }

        let (csum, input_len) = compute_checksum(cmd, input_parts);

        let mut mb = mbox();
        mb.write32(MBOX_CMD, cmd);
        mb.write32(MBOX_DLEN, 4 + input_len as u32);
        mb.write32(MBOX_DATAIN, csum);
        write_input_parts_mmio(&mut mb, input_parts);
        mb.write32(MBOX_EXEC, 1);

        let mb_sts = match aspeed_mmio::poll_until(
            || MboxStatus::from_raw(Self::mbox_status_raw()),
            |s| *s != MboxStatus::CmdBusy,
            MBOX_POLL_LOOPS,
        ) {
            Ok(s) => s,
            Err(_) => { Self::mbox_unlock(); return Err(CptraError::Timeout); }
        };

        let bytes_read = Self::drain_output_bytes(mb_sts, out_buf)?;
        Self::mbox_unlock();

        if mb_sts == MboxStatus::CmdFailure {
            return Err(CptraError::CmdFailed);
        }
        Ok(bytes_read)
    }

    // ── Core mailbox transaction (async) ──────────────────────────────────

    async fn transaction_async(
        cmd: u32,
        input: &[u8],
        out_buf: &mut [u32],
    ) -> Result<usize, CptraError> {
        Self::transaction_parts_async(cmd, &[input], out_buf).await
    }

    async fn transaction_parts_async(
        cmd: u32,
        input_parts: &[&[u8]],
        out_buf: &mut [u32],
    ) -> Result<usize, CptraError> {
        Self::mbox_lock()?;

        let sts = Self::mbox_status_raw();
        if MboxFsm::from_raw(sts) != MboxFsm::RdyForCmd {
            Self::mbox_unlock();
            return Err(CptraError::NotReady);
        }

        let (csum, input_len) = compute_checksum(cmd, input_parts);

        let mut mb = mbox();
        mb.write32(MBOX_CMD, cmd);
        mb.write32(MBOX_DLEN, 4 + input_len as u32);
        mb.write32(MBOX_DATAIN, csum);
        write_input_parts_mmio(&mut mb, input_parts);
        mb.write32(MBOX_EXEC, 1);

        let mb_sts = match aspeed_mmio::poll_until_async(
            || MboxStatus::from_raw(Self::mbox_status_raw()),
            |s| *s != MboxStatus::CmdBusy,
            embassy_time::Duration::from_micros(10),
            embassy_time::Duration::from_secs(5),
        ).await {
            Ok(s) => s,
            Err(_) => { Self::mbox_unlock(); return Err(CptraError::Timeout); }
        };

        let words_read = Self::drain_output(mb_sts, out_buf)?;
        Self::mbox_unlock();

        if mb_sts == MboxStatus::CmdFailure {
            return Err(CptraError::CmdFailed);
        }
        Ok(words_read)
    }

    async fn transaction_bytes_parts_async(
        cmd: u32,
        input_parts: &[&[u8]],
        out_buf: &mut [u8],
    ) -> Result<usize, CptraError> {
        Self::mbox_lock()?;

        let sts = Self::mbox_status_raw();
        if MboxFsm::from_raw(sts) != MboxFsm::RdyForCmd {
            Self::mbox_unlock();
            return Err(CptraError::NotReady);
        }

        let (csum, input_len) = compute_checksum(cmd, input_parts);

        let mut mb = mbox();
        mb.write32(MBOX_CMD, cmd);
        mb.write32(MBOX_DLEN, 4 + input_len as u32);
        mb.write32(MBOX_DATAIN, csum);
        write_input_parts_mmio(&mut mb, input_parts);
        mb.write32(MBOX_EXEC, 1);

        let mb_sts = match aspeed_mmio::poll_until_async(
            || MboxStatus::from_raw(Self::mbox_status_raw()),
            |s| *s != MboxStatus::CmdBusy,
            embassy_time::Duration::from_micros(10),
            embassy_time::Duration::from_secs(5),
        ).await {
            Ok(s) => s,
            Err(_) => { Self::mbox_unlock(); return Err(CptraError::Timeout); }
        };

        let bytes_read = Self::drain_output_bytes(mb_sts, out_buf)?;
        Self::mbox_unlock();

        if mb_sts == MboxStatus::CmdFailure {
            return Err(CptraError::CmdFailed);
        }
        Ok(bytes_read)
    }

    // ── Response draining helpers ──────────────────────────────────────────

    fn drain_output(mb_sts: MboxStatus, out_buf: &mut [u32]) -> Result<usize, CptraError> {
        if mb_sts == MboxStatus::DataReady {
            let out_dlen = mbox().read32(MBOX_DLEN) as usize;
            let word_count = out_dlen / 4;
            if word_count > out_buf.len() {
                // Drain excess words to leave the FIFO in a clean state;
                // the caller's buffer is too small for the response.
                for _ in 0..word_count {
                    let _ = mbox().read32(MBOX_DATAOUT);
                }
                return Err(CptraError::InvalidLength);
            }
            for word in out_buf.iter_mut().take(word_count) {
                *word = mbox().read32(MBOX_DATAOUT);
            }
            Ok(word_count)
        } else {
            Ok(0)
        }
    }

    fn drain_output_bytes(mb_sts: MboxStatus, out_buf: &mut [u8]) -> Result<usize, CptraError> {
        if mb_sts == MboxStatus::DataReady {
            let out_dlen = mbox().read32(MBOX_DLEN) as usize;
            if out_dlen > out_buf.len() {
                Self::mbox_unlock();
                return Err(CptraError::InvalidLength);
            }
            for chunk in out_buf[..out_dlen].chunks_mut(4) {
                let word = mbox().read32(MBOX_DATAOUT).to_le_bytes();
                let n = chunk.len();
                chunk.copy_from_slice(&word[..n]);
            }
            Ok(out_dlen)
        } else {
            Ok(0)
        }
    }

    // ── High-level commands (sync) ─────────────────────────────────────────

    pub fn fw_info() -> Result<FwInfoResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<FwInfoResponse>() / 4];
        Self::transaction(cmd::FW_INFO, &[], &mut buf)?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const FwInfoResponse) })
    }

    pub fn capabilities() -> Result<CapabilitiesResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<CapabilitiesResponse>() / 4];
        Self::transaction(cmd::CAPABILITIES, &[], &mut buf)?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const CapabilitiesResponse) })
    }

    pub fn fips_version() -> Result<FipsVersionResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<FipsVersionResponse>() / 4];
        Self::transaction(cmd::FIPS_VERSION, &[], &mut buf)?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const FipsVersionResponse) })
    }

    pub fn self_test_start() -> Result<SelfTestResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<SelfTestResponse>() / 4];
        Self::transaction(cmd::SELF_TEST_START, &[], &mut buf)?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const SelfTestResponse) })
    }

    pub fn self_test_get_results() -> Result<SelfTestResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<SelfTestResponse>() / 4];
        Self::transaction(cmd::SELF_TEST_GET_RESULTS, &[], &mut buf)?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const SelfTestResponse) })
    }

    pub fn shutdown() -> Result<ShutdownResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<ShutdownResponse>() / 4];
        Self::transaction(cmd::SHUTDOWN, &[], &mut buf)?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const ShutdownResponse) })
    }

    pub fn set_auth_manifest(manifest: &[u8]) -> Result<(), CptraError> {
        if manifest.len() > MAX_AUTH_MANIFEST_SIZE {
            return Err(CptraError::InvalidLength);
        }
        let mut out = [0u32; 2];
        let len = (manifest.len() as u32).to_le_bytes();
        Self::transaction_parts(cmd::SET_AUTH_MANIFEST, &[&len, manifest], &mut out)?;
        Ok(())
    }

    /// Authorize (and optionally stash) an image against the SoC auth-manifest
    /// previously installed with [`set_auth_manifest`].
    ///
    /// `fw_id` is matched against the manifest image-metadata collection. When
    /// `source` is [`ImageHashSource::InRequest`] the caller supplies the
    /// SHA-384 `digest`; for `LoadAddress`/`StagingAddress` Caliptra hashes
    /// `image_size` bytes at the image address itself and `digest` is ignored.
    /// `skip_stash` suppresses recording the measurement in DPE.
    pub fn authorize_and_stash(
        fw_id: u32,
        digest: &[u8; IMAGE_DIGEST_SIZE],
        svn: u32,
        skip_stash: bool,
        source: ImageHashSource,
        image_size: u32,
    ) -> Result<AuthResult, CptraError> {
        let mut out = [0u32; 3];
        let parts = Self::auth_and_stash_parts(fw_id, svn, skip_stash, source, image_size);
        Self::transaction_parts(
            cmd::AUTHORIZE_AND_STASH,
            &[
                &parts.fw_id,
                digest,
                &parts.context,
                &parts.svn,
                &parts.flags,
                &parts.source,
                &parts.image_size,
            ],
            &mut out,
        )?;
        // Response: [chksum, fips_status, auth_req_result].
        Ok(AuthResult::from_raw(out[2]))
    }

    /// Build the fixed-size little-endian request fields for AUTHORIZE_AND_STASH.
    fn auth_and_stash_parts(
        fw_id: u32,
        svn: u32,
        skip_stash: bool,
        source: ImageHashSource,
        image_size: u32,
    ) -> AuthStashParts {
        AuthStashParts {
            fw_id: fw_id.to_le_bytes(),
            context: [0u8; IMAGE_DIGEST_SIZE],
            svn: svn.to_le_bytes(),
            flags: (if skip_stash { AUTH_FLAG_SKIP_STASH } else { 0 }).to_le_bytes(),
            source: (source as u32).to_le_bytes(),
            image_size: image_size.to_le_bytes(),
        }
    }

    pub fn get_idev_ecc384_cert(
        tbs: &[u8],
        signature_r: &[u8; 48],
        signature_s: &[u8; 48],
        cert_out: &mut [u8],
    ) -> Result<usize, CptraError> {
        if tbs.len() > MAX_IDEVID_ECC384_TBS_SIZE {
            return Err(CptraError::InvalidLength);
        }
        let mut response = [0u8; 12 + MAX_IDEVID_ECC384_CERT_SIZE];
        let tbs_len = (tbs.len() as u32).to_le_bytes();
        let len = Self::transaction_bytes_parts(
            cmd::GET_IDEV_CERT,
            &[&tbs_len, signature_r, signature_s, tbs],
            &mut response,
        )?;
        if len < 12 {
            return Err(CptraError::InvalidLength);
        }
        let cert_size =
            u32::from_le_bytes([response[8], response[9], response[10], response[11]]) as usize;
        if cert_size > cert_out.len() || 12 + cert_size > len {
            return Err(CptraError::InvalidLength);
        }
        cert_out[..cert_size].copy_from_slice(&response[12..12 + cert_size]);
        Ok(cert_size)
    }

    pub fn populate_idev_ecc384_cert(cert: &[u8]) -> Result<(), CptraError> {
        if cert.len() > MAX_IDEVID_ECC384_CERT_SIZE {
            return Err(CptraError::InvalidLength);
        }
        let mut out = [0u32; 2];
        let cert_len = (cert.len() as u32).to_le_bytes();
        Self::transaction_parts(cmd::POPULATE_IDEV_CERT, &[&cert_len, cert], &mut out)?;
        Ok(())
    }

    // ── High-level commands (async) ────────────────────────────────────────

    pub async fn fw_info_async() -> Result<FwInfoResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<FwInfoResponse>() / 4];
        Self::transaction_async(cmd::FW_INFO, &[], &mut buf).await?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const FwInfoResponse) })
    }

    pub async fn capabilities_async() -> Result<CapabilitiesResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<CapabilitiesResponse>() / 4];
        Self::transaction_async(cmd::CAPABILITIES, &[], &mut buf).await?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const CapabilitiesResponse) })
    }

    pub async fn fips_version_async() -> Result<FipsVersionResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<FipsVersionResponse>() / 4];
        Self::transaction_async(cmd::FIPS_VERSION, &[], &mut buf).await?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const FipsVersionResponse) })
    }

    pub async fn self_test_start_async() -> Result<SelfTestResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<SelfTestResponse>() / 4];
        Self::transaction_async(cmd::SELF_TEST_START, &[], &mut buf).await?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const SelfTestResponse) })
    }

    pub async fn self_test_get_results_async() -> Result<SelfTestResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<SelfTestResponse>() / 4];
        Self::transaction_async(cmd::SELF_TEST_GET_RESULTS, &[], &mut buf).await?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const SelfTestResponse) })
    }

    pub async fn shutdown_async() -> Result<ShutdownResponse, CptraError> {
        let mut buf = [0u32; core::mem::size_of::<ShutdownResponse>() / 4];
        Self::transaction_async(cmd::SHUTDOWN, &[], &mut buf).await?;
        Ok(unsafe { core::ptr::read(buf.as_ptr() as *const ShutdownResponse) })
    }

    pub async fn set_auth_manifest_async(manifest: &[u8]) -> Result<(), CptraError> {
        if manifest.len() > MAX_AUTH_MANIFEST_SIZE {
            return Err(CptraError::InvalidLength);
        }
        let mut out = [0u32; 2];
        let len = (manifest.len() as u32).to_le_bytes();
        Self::transaction_parts_async(cmd::SET_AUTH_MANIFEST, &[&len, manifest], &mut out).await?;
        Ok(())
    }

    /// Async form of [`Caliptra::authorize_and_stash`].
    pub async fn authorize_and_stash_async(
        fw_id: u32,
        digest: &[u8; IMAGE_DIGEST_SIZE],
        svn: u32,
        skip_stash: bool,
        source: ImageHashSource,
        image_size: u32,
    ) -> Result<AuthResult, CptraError> {
        let mut out = [0u32; 3];
        let parts = Self::auth_and_stash_parts(fw_id, svn, skip_stash, source, image_size);
        Self::transaction_parts_async(
            cmd::AUTHORIZE_AND_STASH,
            &[
                &parts.fw_id,
                digest,
                &parts.context,
                &parts.svn,
                &parts.flags,
                &parts.source,
                &parts.image_size,
            ],
            &mut out,
        )
        .await?;
        Ok(AuthResult::from_raw(out[2]))
    }

    pub async fn get_idev_ecc384_cert_async(
        tbs: &[u8],
        signature_r: &[u8; 48],
        signature_s: &[u8; 48],
        cert_out: &mut [u8],
    ) -> Result<usize, CptraError> {
        if tbs.len() > MAX_IDEVID_ECC384_TBS_SIZE {
            return Err(CptraError::InvalidLength);
        }
        let mut response = [0u8; 12 + MAX_IDEVID_ECC384_CERT_SIZE];
        let tbs_len = (tbs.len() as u32).to_le_bytes();
        let len = Self::transaction_bytes_parts_async(
            cmd::GET_IDEV_CERT,
            &[&tbs_len, signature_r, signature_s, tbs],
            &mut response,
        ).await?;
        if len < 12 {
            return Err(CptraError::InvalidLength);
        }
        let cert_size =
            u32::from_le_bytes([response[8], response[9], response[10], response[11]]) as usize;
        if cert_size > cert_out.len() || 12 + cert_size > len {
            return Err(CptraError::InvalidLength);
        }
        cert_out[..cert_size].copy_from_slice(&response[12..12 + cert_size]);
        Ok(cert_size)
    }

    pub async fn populate_idev_ecc384_cert_async(cert: &[u8]) -> Result<(), CptraError> {
        if cert.len() > MAX_IDEVID_ECC384_CERT_SIZE {
            return Err(CptraError::InvalidLength);
        }
        let mut out = [0u32; 2];
        let cert_len = (cert.len() as u32).to_le_bytes();
        Self::transaction_parts_async(
            cmd::POPULATE_IDEV_CERT,
            &[&cert_len, cert],
            &mut out,
        ).await?;
        Ok(())
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

fn compute_checksum(cmd: u32, parts: &[&[u8]]) -> (u32, usize) {
    let mut csum = checksum(0, &cmd.to_le_bytes());
    let mut total_len = 0usize;
    for part in parts {
        csum = checksum(csum, part);
        total_len += part.len();
    }
    (csum, total_len)
}

fn write_input_parts_mmio(mb: &mut MmioBlock, parts: &[&[u8]]) {
    let mut word = [0u8; 4];
    let mut used = 0usize;
    for part in parts {
        for &byte in *part {
            word[used] = byte;
            used += 1;
            if used == 4 {
                mb.write32(MBOX_DATAIN, u32::from_le_bytes(word));
                word = [0u8; 4];
                used = 0;
            }
        }
    }
    if used != 0 {
        mb.write32(MBOX_DATAIN, u32::from_le_bytes(word));
    }
}
