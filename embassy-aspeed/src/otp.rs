//! AST2700 OTP controller driver.
//!
//! OTP addresses are 16-bit word indices, matching ASPEED Zephyr and U-Boot.
//!
//! # Sync and async
//!
//! Both sync (blocking) and async (yielding) APIs are provided:
//!
//! - `read_word()` / `read_word_async()` — read a single OTP word
//! - `read_words()` / `read_words_async()` — read a slice of OTP words
//! - `read_bytes()` / `read_bytes_async()` — read OTP data as bytes
//! - `program_word()` / `program_word_async()` — program a single word
//! - `program_words()` / `program_words_async()` — program multiple words
//!
//! Async variants use `poll_until_async` to yield to the Embassy executor
//! instead of busy-waiting on the OTP STATUS register.

use crate::pac;

use pac::otp_ast2700_v1;

const PASSWORD: u32 = 0x349f_e38a;
const CMD_READ: u32 = 0x23b1_e361;
const CMD_PROG: u32 = 0x23b1_e364;
const CMD_PROG_MULTI: u32 = 0x23b1_e365;
const OTPSTRAP14_ADDR: u32 = 0x420 + 0x0e;
const TIMEOUT_LOOPS: u32 = 10_000;

pub const ROM_START: u32 = 0x0000;
pub const ROM_END: u32 = 0x03e0;
pub const RBP_START: u32 = ROM_END;
pub const RBP_END: u32 = 0x0400;
pub const CONF_START: u32 = RBP_END;
pub const CONF_END: u32 = 0x0420;
pub const STRAP_START: u32 = CONF_END;
pub const STRAP_END: u32 = 0x0430;
pub const STRAP_EXT_START: u32 = STRAP_END;
pub const STRAP_EXT_END: u32 = 0x0440;
pub const USER_START: u32 = STRAP_EXT_END;
pub const USER_END: u32 = 0x1000;
pub const SECURE_START: u32 = USER_END;
pub const SECURE_END: u32 = 0x1c00;
pub const CALIPTRA_START: u32 = SECURE_END;
pub const CALIPTRA_END: u32 = 0x1f80;
pub const SW_PUF_START: u32 = CALIPTRA_END;
pub const SW_PUF_END: u32 = 0x1fa0;
pub const HW_PUF_START: u32 = SW_PUF_END;
pub const HW_PUF_END: u32 = 0x2000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Timeout,
    CommandFailed(u32),
    InvalidLength,
}

#[derive(Clone, Copy)]
pub struct Otp {
    ecc_enabled: bool,
}

impl Otp {
    pub fn new() -> Result<Self, Error> {
        pac::OTP.KEY().write_value(otp_ast2700_v1::KEY(PASSWORD));
        let mut otp = Self { ecc_enabled: false };
        otp.ecc_enabled = otp.read_ecc_strap()?;
        Ok(otp)
    }

    pub const fn with_ecc_enabled(ecc_enabled: bool) -> Self {
        Self { ecc_enabled }
    }

    pub fn ecc_enabled(&self) -> bool {
        self.ecc_enabled
    }

    pub fn set_ecc_enabled(&mut self, enabled: bool) {
        self.ecc_enabled = enabled;
    }

    // ── Sync API ──────────────────────────────────────────────────────────

    pub fn read_word(&self, offset: u32) -> Result<u16, Error> {
        self.write_ecc_mode();
        pac::OTP.ADDR().write_value(otp_ast2700_v1::ADDR(offset));
        pac::OTP.CMD().write_value(otp_ast2700_v1::CMD(CMD_READ));
        self.wait_complete()?;
        Ok(pac::OTP.RDATA().read().WORD())
    }

    pub fn read_words(&self, offset: u32, out: &mut [u16]) -> Result<(), Error> {
        for (i, word) in out.iter_mut().enumerate() {
            *word = self.read_word(offset + i as u32)?;
        }
        Ok(())
    }

    pub fn read_bytes(&self, offset: u32, out: &mut [u8]) -> Result<(), Error> {
        let mut addr = offset;
        for chunk in out.chunks_mut(2) {
            let word = self.read_word(addr)?.to_le_bytes();
            chunk[0] = word[0];
            if chunk.len() == 2 {
                chunk[1] = word[1];
            }
            addr += 1;
        }
        Ok(())
    }

    pub fn program_word(&self, offset: u32, data: u16) -> Result<(), Error> {
        self.write_ecc_mode();
        pac::OTP.ADDR().write_value(otp_ast2700_v1::ADDR(offset));
        pac::OTP.WDATA0().write_value(data as u32);
        pac::OTP.CMD().write_value(otp_ast2700_v1::CMD(CMD_PROG));
        self.wait_complete()
    }

    pub fn program_words(&self, offset: u32, data: &[u16]) -> Result<(), Error> {
        if data.len() == 1 {
            return self.program_word(offset, data[0]);
        }
        if data.len() % 2 != 0 || data.len() > 8 {
            return Err(Error::InvalidLength);
        }
        self.write_ecc_mode();
        pac::OTP.ADDR().write_value(otp_ast2700_v1::ADDR(offset));
        let regs = [
            pac::OTP.WDATA0(),
            pac::OTP.WDATA1(),
            pac::OTP.WDATA2(),
            pac::OTP.WDATA3(),
        ];
        for (i, pair) in data.chunks_exact(2).enumerate() {
            regs[i].write_value(pair[0] as u32 | ((pair[1] as u32) << 16));
        }
        pac::OTP.CMD().write_value(otp_ast2700_v1::CMD(CMD_PROG_MULTI));
        self.wait_complete()
    }

    // ── Async API ─────────────────────────────────────────────────────────

    #[cfg(feature = "ast2700-bootmcu")]
    pub async fn read_word_async(&self, offset: u32) -> Result<u16, Error> {
        self.write_ecc_mode();
        pac::OTP.ADDR().write_value(otp_ast2700_v1::ADDR(offset));
        pac::OTP.CMD().write_value(otp_ast2700_v1::CMD(CMD_READ));
        self.wait_complete_async().await?;
        Ok(pac::OTP.RDATA().read().WORD())
    }

    #[cfg(feature = "ast2700-bootmcu")]
    pub async fn read_words_async(&self, offset: u32, out: &mut [u16]) -> Result<(), Error> {
        for (i, word) in out.iter_mut().enumerate() {
            *word = self.read_word_async(offset + i as u32).await?;
        }
        Ok(())
    }

    #[cfg(feature = "ast2700-bootmcu")]
    pub async fn read_bytes_async(&self, offset: u32, out: &mut [u8]) -> Result<(), Error> {
        let mut addr = offset;
        for chunk in out.chunks_mut(2) {
            let word = self.read_word_async(addr).await?.to_le_bytes();
            chunk[0] = word[0];
            if chunk.len() == 2 {
                chunk[1] = word[1];
            }
            addr += 1;
        }
        Ok(())
    }

    #[cfg(feature = "ast2700-bootmcu")]
    pub async fn program_word_async(&self, offset: u32, data: u16) -> Result<(), Error> {
        self.write_ecc_mode();
        pac::OTP.ADDR().write_value(otp_ast2700_v1::ADDR(offset));
        pac::OTP.WDATA0().write_value(data as u32);
        pac::OTP.CMD().write_value(otp_ast2700_v1::CMD(CMD_PROG));
        self.wait_complete_async().await
    }

    #[cfg(feature = "ast2700-bootmcu")]
    pub async fn program_words_async(&self, offset: u32, data: &[u16]) -> Result<(), Error> {
        if data.len() == 1 {
            return self.program_word_async(offset, data[0]).await;
        }
        if data.len() % 2 != 0 || data.len() > 8 {
            return Err(Error::InvalidLength);
        }
        self.write_ecc_mode();
        pac::OTP.ADDR().write_value(otp_ast2700_v1::ADDR(offset));
        let regs = [
            pac::OTP.WDATA0(),
            pac::OTP.WDATA1(),
            pac::OTP.WDATA2(),
            pac::OTP.WDATA3(),
        ];
        for (i, pair) in data.chunks_exact(2).enumerate() {
            regs[i].write_value(pair[0] as u32 | ((pair[1] as u32) << 16));
        }
        pac::OTP.CMD().write_value(otp_ast2700_v1::CMD(CMD_PROG_MULTI));
        self.wait_complete_async().await
    }

    // ── Internals ─────────────────────────────────────────────────────────

    fn read_ecc_strap(&self) -> Result<bool, Error> {
        pac::OTP
            .ECC_EN()
            .write_value(otp_ast2700_v1::ECC_EN(0));
        pac::OTP.ADDR().write_value(otp_ast2700_v1::ADDR(OTPSTRAP14_ADDR));
        pac::OTP.CMD().write_value(otp_ast2700_v1::CMD(CMD_READ));
        self.wait_complete()?;
        Ok(pac::OTP.RDATA().read().WORD() & 1 != 0)
    }

    fn write_ecc_mode(&self) {
        pac::OTP
            .ECC_EN()
            .write_value(otp_ast2700_v1::ECC_EN(self.ecc_enabled as u32));
    }

    fn wait_complete(&self) -> Result<(), Error> {
        aspeed_mmio::poll_until(
            || pac::OTP.STATUS().read(),
            |status| !status.BUSY(),
            TIMEOUT_LOOPS,
        )
        .map_err(|_| Error::Timeout)?;

        let status = pac::OTP.STATUS().read();
        let cmd_sts = status.CMD_STS();
        if cmd_sts == 0 {
            Ok(())
        } else {
            Err(Error::CommandFailed(cmd_sts as u32))
        }
    }

    #[cfg(feature = "ast2700-bootmcu")]
    async fn wait_complete_async(&self) -> Result<(), Error> {
        aspeed_mmio::poll_until_async(
            || pac::OTP.STATUS().read(),
            |status| !status.BUSY(),
            embassy_time::Duration::from_micros(10),
            embassy_time::Duration::from_millis(100),
        )
        .await
        .map_err(|_| Error::Timeout)?;

        let status = pac::OTP.STATUS().read();
        let cmd_sts = status.CMD_STS();
        if cmd_sts == 0 {
            Ok(())
        } else {
            Err(Error::CommandFailed(cmd_sts as u32))
        }
    }
}
