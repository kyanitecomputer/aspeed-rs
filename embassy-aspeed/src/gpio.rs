//! GPIO driver: 208-pin (AST2600) / 149-pin (AST1060) input/output/interrupt.
//!
//! # Pin numbering
//!
//! Pins are numbered continuously (0-based):
//! - Port A pin 0 = pin 0, …, Port A pin 7 = pin 7
//! - Port B pin 0 = pin 8, …
//!
//! | Chip | Ports | Max pins |
//! |------|-------|----------|
//! | AST2600 SSP | A–Z | 208 |
//! | AST1060 | A–U | 149 (ports T and U are input-only) |
//!
//! # Interrupt sensitivity
//!
//! ```text
//! index_data = int_enable | (int_type << 1)
//! int_type: 0=FALLING_EDGE, 1=RISING_EDGE, 2=LEVEL_LOW, 3=LEVEL_HIGH, 4=BOTH_EDGES
//! ```
//!
//! # Command source (AST2600 SSP only)
//!
//! On the AST2600, pins controlled by the SSP co-processor must have their
//! command source set to SSP master ID 6 (`ASPEED_GPIO_SEL_SSP`).
//! On the AST1060, the CM4F is the native ARM master (ID 0); no claim needed.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::gpio::{pin, Output, Level};
//! use embedded_hal::digital::OutputPin;
//!
//! let mut led = Output::new(pin('A', 0), Level::Low);
//! led.set_high().ok();
//! ```

use core::future::Future;
use core::pin::Pin as PinFut;
use core::task::{Context, Poll};

use aspeed_mmio::MmioBlock;
use embassy_sync::waitqueue::AtomicWaker;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use embedded_hal_async::digital::Wait;

// ── GPIO base and constants ───────────────────────────────────────────────────

const GPIO_BASE: usize = 0x7E78_0000;

/// SSP master index in the CMD_SRC_SEL register (AST2600 SSP only).
/// On AST1060 the CM4F is the ARM master (ID 0); no override needed.
#[cfg(feature = "ast2600-ssp")]
const SSP_CMD_SRC_SEL: u32 = 6;

/// GPIO index register command types.
const IDX_TYPE_DIR: u32 = 1;
const IDX_TYPE_INTR: u32 = 2;
const IDX_CMD_WRITE: u32 = 0;

// ── Index register helper ─────────────────────────────────────────────────────

/// Offsets within the GPIO block (byte offsets / 4).
const IDX_REG_WORD: usize = 0x2AC / 4;
#[cfg(feature = "ast2600-ssp")]
const CMD_SRC_SEL_WORD: usize = 0x2D0 / 4;

#[inline(always)]
fn gpio_rr(word_off: usize) -> u32 {
    let regs = unsafe { MmioBlock::new(GPIO_BASE) };
    regs.read32(word_off * 4)
}

#[inline(always)]
fn gpio_rw(word_off: usize, val: u32) {
    let mut regs = unsafe { MmioBlock::new(GPIO_BASE) };
    regs.write32(word_off * 4, val)
}

fn index_write(pin_num: u8, idx_type: u32, idx_data: u32) {
    let word = (pin_num as u32) | (IDX_CMD_WRITE << 12) | (idx_type << 16) | (idx_data << 20);
    gpio_rw(IDX_REG_WORD, word);
}

// ── Group layout table ────────────────────────────────────────────────────────

/// Per-group register offsets (in 32-bit words from GPIO base).
struct GroupRegs {
    data: usize,
    #[allow(dead_code)]
    dir: usize,
    int_status: usize,
    /// CMD_SRC registers — used on AST2600 SSP to claim SSP ownership.
    /// Present but unused on AST1060 (CM4F is native ARM master).
    #[allow(dead_code)]
    cmd_src0: usize,
    #[allow(dead_code)]
    cmd_src1: usize,
    data_read: usize,
}

/// Register offsets for each GPIO group.
static GROUPS: [GroupRegs; 7] = [
    // Group 0: A/B/C/D
    GroupRegs {
        data: 0x000 / 4,
        dir: 0x004 / 4,
        int_status: 0x018 / 4,
        cmd_src0: 0x060 / 4,
        cmd_src1: 0x064 / 4,
        data_read: 0x0C0 / 4,
    },
    // Group 1: E/F/G/H
    GroupRegs {
        data: 0x020 / 4,
        dir: 0x024 / 4,
        int_status: 0x038 / 4,
        cmd_src0: 0x068 / 4,
        cmd_src1: 0x06C / 4,
        data_read: 0x0C4 / 4,
    },
    // Group 2: I/J/K/L
    GroupRegs {
        data: 0x070 / 4,
        dir: 0x074 / 4,
        int_status: 0x0A8 / 4,
        cmd_src0: 0x090 / 4,
        cmd_src1: 0x094 / 4,
        data_read: 0x0C8 / 4,
    },
    // Group 3: M/N/O/P
    GroupRegs {
        data: 0x078 / 4,
        dir: 0x07C / 4,
        int_status: 0x0F8 / 4,
        cmd_src0: 0x0E0 / 4,
        cmd_src1: 0x0E4 / 4,
        data_read: 0x0CC / 4,
    },
    // Group 4: Q/R/S/T
    GroupRegs {
        data: 0x080 / 4,
        dir: 0x084 / 4,
        int_status: 0x128 / 4,
        cmd_src0: 0x110 / 4,
        cmd_src1: 0x114 / 4,
        data_read: 0x0D0 / 4,
    },
    // Group 5: U/V/W/X
    GroupRegs {
        data: 0x088 / 4,
        dir: 0x08C / 4,
        int_status: 0x158 / 4,
        cmd_src0: 0x140 / 4,
        cmd_src1: 0x144 / 4,
        data_read: 0x0D4 / 4,
    },
    // Group 6: Y/Z (16 pins only)
    GroupRegs {
        data: 0x1E0 / 4,
        dir: 0x1E4 / 4,
        int_status: 0x188 / 4,
        cmd_src0: 0x170 / 4,
        cmd_src1: 0x174 / 4,
        data_read: 0x0D8 / 4,
    },
];

// ── Wakers (one per pin, up to 208 pins) ─────────────────────────────────────

// 208 wakers is a lot of static storage. Use a smaller pool.
// Only 32 wakers = 32 pins can wait for edges simultaneously.
const MAX_WAITING: usize = 32;
static GPIO_WAKERS: [AtomicWaker; MAX_WAITING] = [
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
    AtomicWaker::new(),
    AtomicWaker::new(),
];
/// Pin-to-waker slot mapping, protected by critical section.
static WAKER_PINS: critical_section::Mutex<core::cell::Cell<[u8; MAX_WAITING]>> =
    critical_section::Mutex::new(core::cell::Cell::new([0xFF; MAX_WAITING]));

/// Bitmask of waker slots with a pending fired interrupt (bit i = slot i fired).
///
/// Set by the GPIO ISR **after** clearing the hw `int_status` register.
/// Cleared by `EdgeFuture::poll` when it returns `Ready`, or by
/// `EdgeFuture::drop` on cancellation.
///
/// This software flag prevents the race where the ISR clears `int_status`
/// before the future's next `poll` can observe it, which would cause the
/// edge event to be silently dropped.
static WAKER_SLOT_FIRED: critical_section::Mutex<core::cell::Cell<u32>> =
    critical_section::Mutex::new(core::cell::Cell::new(0));

// Drop unused variable warning
const _: () = {
    let _ = &AtomicWaker::new;
};

// ── GpioPin ───────────────────────────────────────────────────────────────────

/// A GPIO pin identified by its absolute pin number (0–207).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GpioPin {
    /// Absolute pin number (0 = port A bit 0, 207 = port Z bit 7).
    number: u8,
    /// Group index (0–6).
    group: u8,
    /// Bit within the 32-bit group register (0–31).
    bit: u8,
}

impl GpioPin {
    /// Return the absolute pin number.
    pub const fn number(&self) -> u8 {
        self.number
    }

    fn gr(&self) -> &'static GroupRegs {
        &GROUPS[self.group as usize]
    }
}

/// Construct a `GpioPin` from a port letter and bit index.
///
/// - `port`: `'A'`–`'Z'` on AST2600; `'A'`–`'U'` on AST1060 (case-insensitive).
/// - `bit`: `0`–`7`.
///
/// # Panics
///
/// Panics if `port` is out of range or `bit` is not 0–7.
pub const fn pin(port: char, bit: u8) -> GpioPin {
    let p = match port {
        'A' | 'a' => 0u8,
        'B' | 'b' => 1,
        'C' | 'c' => 2,
        'D' | 'd' => 3,
        'E' | 'e' => 4,
        'F' | 'f' => 5,
        'G' | 'g' => 6,
        'H' | 'h' => 7,
        'I' | 'i' => 8,
        'J' | 'j' => 9,
        'K' | 'k' => 10,
        'L' | 'l' => 11,
        'M' | 'm' => 12,
        'N' | 'n' => 13,
        'O' | 'o' => 14,
        'P' | 'p' => 15,
        'Q' | 'q' => 16,
        'R' | 'r' => 17,
        'S' | 's' => 18,
        'T' | 't' => 19,
        'U' | 'u' => 20,
        // V–Z only available on AST2600 SSP (208 pins).
        #[cfg(feature = "ast2600-ssp")]
        'V' | 'v' => 21,
        #[cfg(feature = "ast2600-ssp")]
        'W' | 'w' => 22,
        #[cfg(feature = "ast2600-ssp")]
        'X' | 'x' => 23,
        #[cfg(feature = "ast2600-ssp")]
        'Y' | 'y' => 24,
        #[cfg(feature = "ast2600-ssp")]
        'Z' | 'z' => 25,
        _ => panic!("invalid GPIO port for this chip"),
    };
    assert!(bit < 8, "GPIO bit must be 0-7");
    let number = p * 8 + bit;
    let group = p / 4;
    let bit_in_group = (p % 4) * 8 + bit;
    GpioPin {
        number,
        group,
        bit: bit_in_group,
    }
}

// ── Claim master ownership ────────────────────────────────────────────────────

/// On AST2600 SSP: configure CMD_SRC to route the pin through the SSP master.
/// On AST1060: no-op — the CM4F is the native ARM master (default CMD_SRC).
fn claim_master(p: &GpioPin) {
    #[cfg(feature = "ast2600-ssp")]
    {
        // Configure CMD_SRC_SEL slot 2 (mst3) to hold SSP master ID = 6.
        let sel = gpio_rr(CMD_SRC_SEL_WORD);
        if ((sel >> 10) & 0x1F) != SSP_CMD_SRC_SEL {
            gpio_rw(
                CMD_SRC_SEL_WORD,
                (sel & !(0x1F << 10)) | (SSP_CMD_SRC_SEL << 10),
            );
        }
        let gr = p.gr();
        let mask = 1u32 << p.bit;
        // Set CMD_SRC = SSP: {CMD_SRC1=1, CMD_SRC0=0}.
        gpio_rw(gr.cmd_src0, gpio_rr(gr.cmd_src0) & !mask);
        gpio_rw(gr.cmd_src1, gpio_rr(gr.cmd_src1) | mask);
    }
    #[cfg(feature = "ast1060")]
    {
        // AST1060: CM4F is the ARM master (ID 0, default). No change needed.
        let _ = p;
    }
}

// ── Logic level ───────────────────────────────────────────────────────────────

/// Logic level for GPIO initial state.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Level {
    Low,
    High,
}

// ── GPIO error type ───────────────────────────────────────────────────────────

/// GPIO error (infallible on this platform).
#[derive(Debug, Copy, Clone)]
pub struct GpioError;

impl embedded_hal::digital::Error for GpioError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

// ── Output ────────────────────────────────────────────────────────────────────

/// GPIO output pin.
pub struct Output {
    pin: GpioPin,
}

impl Output {
    /// Configure `pin` as an output with the given initial level.
    pub fn new(p: GpioPin, initial: Level) -> Self {
        claim_master(&p);

        let gr = p.gr();
        let mask = 1u32 << p.bit;

        // Set direction to output.
        index_write(p.number, IDX_TYPE_DIR, 1);

        // Set initial level.
        let data = gpio_rr(gr.data);
        let new_data = match initial {
            Level::High => data | mask,
            Level::Low => data & !mask,
        };
        gpio_rw(gr.data, new_data);

        Self { pin: p }
    }

    /// Set the output high.
    pub fn set_high(&mut self) {
        let gr = self.pin.gr();
        let mask = 1u32 << self.pin.bit;
        gpio_rw(gr.data, gpio_rr(gr.data) | mask);
    }

    /// Set the output low.
    pub fn set_low(&mut self) {
        let gr = self.pin.gr();
        let mask = 1u32 << self.pin.bit;
        gpio_rw(gr.data, gpio_rr(gr.data) & !mask);
    }

    /// Toggle the output.
    pub fn toggle(&mut self) {
        let gr = self.pin.gr();
        let mask = 1u32 << self.pin.bit;
        // Read from data_read (latched input) not data (write register) for
        // accurate state tracking.
        let current = gpio_rr(gr.data_read) & mask;
        if current != 0 {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

impl ErrorType for Output {
    type Error = GpioError;
}

impl OutputPin for Output {
    fn set_low(&mut self) -> Result<(), GpioError> {
        self.set_low();
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), GpioError> {
        self.set_high();
        Ok(())
    }
}

impl StatefulOutputPin for Output {
    fn is_set_high(&mut self) -> Result<bool, GpioError> {
        let gr = self.pin.gr();
        let mask = 1u32 << self.pin.bit;
        Ok(gpio_rr(gr.data_read) & mask != 0)
    }
    fn is_set_low(&mut self) -> Result<bool, GpioError> {
        Ok(!self.is_set_high()?)
    }
}

// ── Input ─────────────────────────────────────────────────────────────────────

/// GPIO input pin.
pub struct Input {
    pin: GpioPin,
}

impl Input {
    /// Configure `pin` as an input.
    pub fn new(p: GpioPin) -> Self {
        claim_master(&p);
        // Set direction to input (0).
        index_write(p.number, IDX_TYPE_DIR, 0);
        Self { pin: p }
    }

    /// Read the current input state.
    pub fn get_level(&self) -> Level {
        let gr = self.pin.gr();
        let mask = 1u32 << self.pin.bit;
        if gpio_rr(gr.data_read) & mask != 0 {
            Level::High
        } else {
            Level::Low
        }
    }
}

impl ErrorType for Input {
    type Error = GpioError;
}

impl InputPin for Input {
    fn is_high(&mut self) -> Result<bool, GpioError> {
        Ok(self.get_level() == Level::High)
    }
    fn is_low(&mut self) -> Result<bool, GpioError> {
        Ok(self.get_level() == Level::Low)
    }
}

impl Wait for Input {
    async fn wait_for_high(&mut self) -> Result<(), GpioError> {
        self.wait_edge(3 /* LEVEL_HIGH */).await;
        Ok(())
    }
    async fn wait_for_low(&mut self) -> Result<(), GpioError> {
        self.wait_edge(2 /* LEVEL_LOW */).await;
        Ok(())
    }
    async fn wait_for_rising_edge(&mut self) -> Result<(), GpioError> {
        self.wait_edge(1 /* RISING_EDGE */).await;
        Ok(())
    }
    async fn wait_for_falling_edge(&mut self) -> Result<(), GpioError> {
        self.wait_edge(0 /* FALLING_EDGE */).await;
        Ok(())
    }
    async fn wait_for_any_edge(&mut self) -> Result<(), GpioError> {
        self.wait_edge(4 /* DUAL_EDGE */).await;
        Ok(())
    }
}

impl Input {
    async fn wait_edge(&mut self, int_type: u32) {
        let data = 1u32 | (int_type << 1); // bit0=enable, bits[4:1]=type
        index_write(self.pin.number, IDX_TYPE_INTR, data);
        // `int_armed: true` tells Drop to disable the interrupt.  This covers
        // both normal completion and cancellation (future dropped before Ready).
        EdgeFuture {
            pin: self.pin,
            slot: None,
            int_armed: true,
        }
        .await;
    }
}

// ── EdgeFuture ────────────────────────────────────────────────────────────────

struct EdgeFuture {
    pin: GpioPin,
    /// Waker slot index; `None` until allocated on the first `poll()`.
    slot: Option<usize>,
    /// `true` once the pin interrupt has been armed via `index_write`.
    /// Used by `Drop` to know whether to disarm the interrupt.
    int_armed: bool,
}

impl Future for EdgeFuture {
    type Output = ();

    fn poll(self: PinFut<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        // EdgeFuture is Unpin (all fields are Unpin), so get_mut() is safe.
        let this = self.get_mut();

        // Allocate a waker slot once on the first poll.
        if this.slot.is_none() {
            this.slot = alloc_waker_slot(this.pin.number);
        }

        if let Some(slot) = this.slot {
            // Check the software pending flag written by the ISR.
            // We use this flag instead of the hw int_status register because
            // the ISR clears int_status *before* waking us; a second hw read
            // would always see 0 and the event would be lost.
            let fired = critical_section::with(|cs| {
                let cell = WAKER_SLOT_FIRED.borrow(cs);
                let bits = cell.get();
                if bits & (1u32 << slot) != 0 {
                    cell.set(bits & !(1u32 << slot));
                    true
                } else {
                    false
                }
            });
            if fired {
                return Poll::Ready(());
            }

            GPIO_WAKERS[slot].register(cx.waker());

            // Re-check after registering: ISR may have fired in the window
            // between the check above and the register() call.
            let fired = critical_section::with(|cs| {
                let cell = WAKER_SLOT_FIRED.borrow(cs);
                let bits = cell.get();
                if bits & (1u32 << slot) != 0 {
                    cell.set(bits & !(1u32 << slot));
                    true
                } else {
                    false
                }
            });
            if fired {
                return Poll::Ready(());
            }
        } else {
            // All 32 waker slots are occupied.  Fall back to a direct hw
            // status read to avoid a permanent deadlock; this degrades to a
            // busy-check on each executor tick rather than an interrupt-driven
            // wake.
            let gr = this.pin.gr();
            let mask = 1u32 << this.pin.bit;
            let status = gpio_rr(gr.int_status);
            if status & mask != 0 {
                gpio_rw(gr.int_status, mask);
                return Poll::Ready(());
            }
        }

        Poll::Pending
    }
}

impl Drop for EdgeFuture {
    fn drop(&mut self) {
        // Disarm the pin interrupt if we enabled it.  This covers cancellation
        // (future dropped before Ready) as well as the normal completion path.
        if self.int_armed {
            index_write(self.pin.number, IDX_TYPE_INTR, 0);
        }
        // Release the waker slot and clear any pending fired bit so a
        // subsequent waiter on the same slot starts clean.
        if let Some(slot) = self.slot {
            critical_section::with(|cs| {
                let pins_cell = WAKER_PINS.borrow(cs);
                let mut arr = pins_cell.get();
                if arr[slot] == self.pin.number {
                    arr[slot] = 0xFF;
                    pins_cell.set(arr);
                }
                let fired = WAKER_SLOT_FIRED.borrow(cs);
                fired.set(fired.get() & !(1u32 << slot));
            });
        }
    }
}

fn alloc_waker_slot(pin_num: u8) -> Option<usize> {
    critical_section::with(|cs| {
        let cell = WAKER_PINS.borrow(cs);
        let mut arr = cell.get();
        // Only claim a genuinely free slot (0xFF).  Do not reuse a slot that
        // already holds the same pin number: two concurrent waiters on the
        // same pin would alias the same slot and corrupt each other's state.
        for (i, &p) in arr.iter().enumerate() {
            if p == 0xFF {
                arr[i] = pin_num;
                cell.set(arr);
                return Some(i);
            }
        }
        None
    })
}

// ── GPIO interrupt handler ────────────────────────────────────────────────────

/// Called from the GPIO peripheral interrupt handler.
pub(crate) fn on_interrupt() {
    // Scan all groups for pending interrupt status.
    for (g, group) in GROUPS.iter().enumerate() {
        let status = gpio_rr(group.int_status);
        if status == 0 {
            continue;
        }
        // Clear the hw status (RW1C) BEFORE updating the software pending
        // flag.  This ordering ensures that if the same edge fires again
        // immediately after we clear, the new hw bit is not lost: the next
        // ISR entry will see it and set the fired flag again.
        gpio_rw(group.int_status, status);

        // For each bit that fired, record it in the software pending bitmask
        // and wake the registered future.
        let mut bits = status;
        while bits != 0 {
            let bit = bits.trailing_zeros() as u8;
            bits &= bits - 1;
            let pin_num = g as u8 * 32 + bit;

            critical_section::with(|cs| {
                let pins_cell = WAKER_PINS.borrow(cs);
                let arr = pins_cell.get();
                let fired = WAKER_SLOT_FIRED.borrow(cs);
                for (i, &p) in arr.iter().enumerate() {
                    if p == pin_num {
                        // Set the software fired bit so poll() sees the event
                        // even though hw int_status has already been cleared.
                        fired.set(fired.get() | (1u32 << i));
                        GPIO_WAKERS[i].wake();
                    }
                }
            });
        }
    }
}

/// GPIO peripheral interrupt handler.
#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn GPIO() {
    on_interrupt();
}
