//! `log`-crate global logger over the ROM-configured UART12 (AST2700 BootMCU).
//!
//! Renders each record as a `LEVEL target: message` line, guarded by a critical
//! section so concurrent tasks/IRQs cannot interleave bytes. This is the uniform
//! logging path for the runtime; it replaces the retired defmt UART logger and
//! the ad-hoc `uart.blocking_write(b"...")` calls.
//!
//! Install once at boot with [`init`], then use the `log::{info,warn,error,
//! debug,trace}` macros anywhere.

use core::fmt::Write;

use log::{Level, LevelFilter, Metadata, Record};

use crate::pac;

struct UartLogger;

static LOGGER: UartLogger = UartLogger;

/// Install the UART global logger and set the maximum level.
///
/// Uses the `*_racy` (non-atomic) installers because the RISC-V ibex target
/// has no atomic CAS (`target_has_atomic = "ptr"` is unset), so the atomic
/// `set_logger`/`set_max_level` are not compiled. This is sound here: the
/// runtime is single-hart and `init` runs once at boot before any logging or
/// concurrency. Idempotent for bring-up convenience — a second call cannot
/// re-install the logger but still updates the level filter.
pub fn init(level: LevelFilter) {
    unsafe {
        let _ = log::set_logger_racy(&LOGGER);
        log::set_max_level_racy(level);
    }
}

impl log::Log for UartLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        let restore = unsafe { critical_section::acquire() };
        let mut uart = Uart;
        let _ = writeln!(
            uart,
            "{} {}: {}",
            level_tag(record.level()),
            record.target(),
            record.args()
        );
        unsafe { critical_section::release(restore) };
    }

    fn flush(&self) {
        while !pac::UART12.LSR().read().TEMT() {}
    }
}

/// A minimal `core::fmt::Write` sink that blocking-writes bytes to UART12,
/// translating each `\n` into `\r\n` so serial terminals don't staircase.
struct Uart;

impl Uart {
    fn put(&self, byte: u8) {
        while !pac::UART12.LSR().read().THRE() {}
        pac::UART12.RBR_THR().write(|w| w.set_DATA(byte));
    }
}

impl Write for Uart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &byte in s.as_bytes() {
            if byte == b'\n' {
                self.put(b'\r');
            }
            self.put(byte);
        }
        Ok(())
    }
}

fn level_tag(level: Level) -> &'static str {
    match level {
        Level::Error => "ERROR",
        Level::Warn => "WARN",
        Level::Info => "INFO",
        Level::Debug => "DEBUG",
        Level::Trace => "TRACE",
    }
}
