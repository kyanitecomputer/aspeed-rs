use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::pac;

#[defmt::global_logger]
struct Logger;

static ENCODER: UartEncoder = UartEncoder::new();

struct UartEncoder {
    taken: AtomicBool,
    cs_restore: UnsafeCell<critical_section::RestoreState>,
    encoder: UnsafeCell<defmt::Encoder>,
}

impl UartEncoder {
    const fn new() -> Self {
        Self {
            taken: AtomicBool::new(false),
            cs_restore: UnsafeCell::new(critical_section::RestoreState::invalid()),
            encoder: UnsafeCell::new(defmt::Encoder::new()),
        }
    }

    fn acquire(&self) {
        let restore = unsafe { critical_section::acquire() };
        if self.taken.load(Ordering::Relaxed) {
            panic!("defmt UART logger taken reentrantly");
        }
        self.taken.store(true, Ordering::Relaxed);

        unsafe {
            self.cs_restore.get().write(restore);
            (*self.encoder.get()).start_frame(write_all);
        }
    }

    unsafe fn write(&self, bytes: &[u8]) {
        (*self.encoder.get()).write(bytes, write_all);
    }

    unsafe fn flush(&self) {
        while !pac::UART12.LSR().read().TEMT() {}
    }

    unsafe fn release(&self) {
        if !self.taken.load(Ordering::Relaxed) {
            panic!("defmt UART logger released out of context");
        }

        (*self.encoder.get()).end_frame(write_all);
        let restore = self.cs_restore.get().read();
        self.taken.store(false, Ordering::Relaxed);
        critical_section::release(restore);
    }
}

unsafe impl Sync for UartEncoder {}

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        ENCODER.acquire();
    }

    unsafe fn flush() {
        ENCODER.flush();
    }

    unsafe fn release() {
        ENCODER.release();
    }

    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes);
    }
}

fn write_all(bytes: &[u8]) {
    for &byte in bytes {
        while !pac::UART12.LSR().read().THRE() {}
        pac::UART12.RBR_THR().write(|w| w.set_DATA(byte));
    }
}
