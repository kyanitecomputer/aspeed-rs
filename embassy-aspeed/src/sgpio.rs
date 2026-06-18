//! Serial GPIO Master (SGPIOM) driver — AST1060.
//!
//! The SGPIOM shifts serial data to/from external 74LV595/74LV165 style shift
//! registers via 4 dedicated pins: SGPMCLK, SGPMLD, SGPMO, SGPMI.
//!
//! **Base address:** `0x7E78_0500` (GPIO base + 0x500).
//! **NVIC IRQ:** 51 (`SGPIO_MASTER`).
//!
//! # Pin numbering
//!
//! Up to 128 output + 128 input bits across 4 × 32-bit groups:
//!
//! | Group | Output register | Input register | Pin range |
//! |-------|----------------|---------------|-----------|
//! | ABCD | `ABCD_DATA` | `ABCD_DATA` (read) | 0–31 |
//! | EFGH | `EFGH_DATA` | `EFGH_DATA` (read) | 32–63 |
//! | IJKL | `IJKL_DATA` | `IJKL_DATA` (read) | 64–95 |
//! | MNOP | `MNOP_DATA` | `MNOP_DATA` (read) | 96–127 |
//!
//! Each group register holds 4 bytes: byte 0 = port A/E/I/M, byte 1 = B/F/J/N, …
//!
//! The shift clock runs continuously while `SGPIO_EN = 1`.  Writing to DATA
//! registers queues the next output; reading them returns the last shifted-in
//! input values.
//!
//! # Interrupt model
//!
//! Each input pin can trigger an interrupt on rising edge, falling edge, dual
//! edge, level-high, or level-low.  The SGPIO_MASTER IRQ (51) fires when any
//! enabled pin fires its configured event.  `SgpioInput::wait_for_change`
//! registers an `AtomicWaker` and yields until the ISR fires.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::sgpio::{Sgpio, SgpioGroup, IrqSensitivity};
//!
//! // Init: 16 bytes (128 pins), PCLK/64 clock.
//! let mut sgpio = Sgpio::new(16, 31); // pin_count=16, clk_div=31
//! sgpio.enable();
//!
//! // Write output group ABCD (first 32 bits).
//! sgpio.set_output(SgpioGroup::Abcd, 0x0000_00FF); // set port A all high
//!
//! // Read current input group ABCD.
//! let inputs = sgpio.get_input(SgpioGroup::Abcd);
//!
//! // Wait for any change on ABCD inputs.
//! sgpio.enable_irq(SgpioGroup::Abcd, 0xFFFF_FFFF, IrqSensitivity::DualEdge);
//! sgpio.wait_change(SgpioGroup::Abcd).await;
//! ```

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::pac;

// ── Global wakers (one per group) ─────────────────────────────────────────────

static SGPIO_WAKERS: [AtomicWaker; 4] = [
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
];

// ── Public types ──────────────────────────────────────────────────────────────

/// SGPIO pin group (32 bits = 4 serial GPIO ports × 8 pins).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SgpioGroup {
    /// Ports A–D (pins 0–31).
    Abcd = 0,
    /// Ports E–H (pins 32–63).
    Efgh = 1,
    /// Ports I–L (pins 64–95).
    Ijkl = 2,
    /// Ports M–P (pins 96–127).
    Mnop = 3,
}

/// Interrupt sensitivity for SGPIO input pins.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IrqSensitivity {
    FallingEdge,
    RisingEdge,
    DualEdge,
    LevelLow,
    LevelHigh,
}

// ── Sgpio ─────────────────────────────────────────────────────────────────────

/// ASPEED Serial GPIO Master driver.
pub struct Sgpio {
    regs: pac::sgpio_v1::SGPIO,
}

impl Sgpio {
    /// Create a driver instance.
    ///
    /// - `pin_count`: number of serial GPIO **bytes** to shift per cycle (1–16).
    ///   1 = 8 pins (group ABCD only), 16 = 128 pins (all four groups).
    /// - `clk_div`: SGPMCLK divider.
    ///   `SGPMCLK = PCLK / (2 × (clk_div + 1))`.
    ///   E.g. clk_div=31 with PCLK=500 MHz → SGPMCLK ≈ 7.8 MHz.
    ///
    /// Does **not** enable the SGPIO master; call [`enable`] separately.
    ///
    /// # Panics
    ///
    /// Panics if `pin_count` is 0 or > 16.
    pub fn new(pin_count: u8, clk_div: u16) -> Self {
        assert!((1..=16).contains(&pin_count), "pin_count must be 1–16");
        let regs = pac::SGPIO;
        regs.SGPIO_CTRL().modify(|w| {
            w.set_SGPIO_EN(false); // disable before configuring
            w.set_PIN_COUNT(pin_count);
            w.set_CLK_DIV(clk_div);
        });
        Self { regs }
    }

    /// Enable the SGPIO master (start the shift clock).
    pub fn enable(&self) {
        self.regs.SGPIO_CTRL().modify(|w| w.set_SGPIO_EN(true));
    }

    /// Disable the SGPIO master (stop the shift clock).
    pub fn disable(&self) {
        self.regs.SGPIO_CTRL().modify(|w| w.set_SGPIO_EN(false));
    }

    // ── Output ────────────────────────────────────────────────────────────────

    /// Write the 32-bit output word for a pin group.
    ///
    /// Bit layout within the word:
    /// - bits[7:0]   = port A/E/I/M (pin 0/32/64/96)
    /// - bits[15:8]  = port B/F/J/N
    /// - bits[23:16] = port C/G/K/O
    /// - bits[31:24] = port D/H/L/P
    pub fn set_output(&self, group: SgpioGroup, val: u32) {
        match group {
            SgpioGroup::Abcd => self.regs.ABCD_DATA().modify(|w| w.set_PINS(val)),
            SgpioGroup::Efgh => self.regs.EFGH_DATA().modify(|w| w.set_PINS(val)),
            SgpioGroup::Ijkl => self.regs.IJKL_DATA().modify(|w| w.set_PINS(val)),
            SgpioGroup::Mnop => self.regs.MNOP_DATA().modify(|w| w.set_PINS(val)),
        }
    }

    /// Set or clear a single output pin by index (0–127).
    pub fn set_output_pin(&self, pin: u8, high: bool) {
        let group = group_of(pin);
        let bit = 1u32 << (pin % 32);
        let cur = self.get_output(group);
        self.set_output(group, if high { cur | bit } else { cur & !bit });
    }

    /// Read back the last value written to an output group.
    pub fn get_output(&self, group: SgpioGroup) -> u32 {
        match group {
            SgpioGroup::Abcd => self.regs.ABCD_DATA_READ().read().PINS(),
            SgpioGroup::Efgh => self.regs.EFGH_DATA_READ().read().PINS(),
            SgpioGroup::Ijkl => self.regs.IJKL_DATA_READ().read().PINS(),
            SgpioGroup::Mnop => self.regs.MNOP_DATA_READ().read().PINS(),
        }
    }

    // ── Input ─────────────────────────────────────────────────────────────────

    /// Read the 32-bit input word for a group (last shifted-in value).
    pub fn get_input(&self, group: SgpioGroup) -> u32 {
        // Input data is in the same DATA register (lower bits) after a shift cycle.
        // On AST1060 the shifted-in data overwrites ABCD/EFGH/IJKL/MNOP_DATA[7:0]
        // for each byte position; DATA_READ mirrors the previously driven output.
        // The hardware places received data in the DATA register itself.
        match group {
            SgpioGroup::Abcd => self.regs.ABCD_DATA().read().PINS(),
            SgpioGroup::Efgh => self.regs.EFGH_DATA().read().PINS(),
            SgpioGroup::Ijkl => self.regs.IJKL_DATA().read().PINS(),
            SgpioGroup::Mnop => self.regs.MNOP_DATA().read().PINS(),
        }
    }

    /// Read a single input pin by index (0–127).
    pub fn get_input_pin(&self, pin: u8) -> bool {
        (self.get_input(group_of(pin)) >> (pin % 32)) & 1 != 0
    }

    // ── Interrupt configuration ───────────────────────────────────────────────

    /// Configure and enable interrupts for `mask` pins in `group`.
    ///
    /// `mask`: bit N=1 enables the interrupt for pin N within the group.
    pub fn enable_irq(&self, group: SgpioGroup, mask: u32, sens: IrqSensitivity) {
        let (s0, s1, s2) = sens_bits(sens);
        match group {
            SgpioGroup::Abcd => {
                self.regs
                    .ABCD_IRQ_SENS0()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s0)));
                self.regs
                    .ABCD_IRQ_SENS1()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s1)));
                self.regs
                    .ABCD_IRQ_SENS2()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s2)));
                self.regs
                    .ABCD_IRQ_EN()
                    .modify(|w| w.set_PINS(w.PINS() | mask));
            }
            SgpioGroup::Efgh => {
                self.regs
                    .EFGH_IRQ_SENS0()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s0)));
                self.regs
                    .EFGH_IRQ_SENS1()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s1)));
                self.regs
                    .EFGH_IRQ_SENS2()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s2)));
                self.regs
                    .EFGH_IRQ_EN()
                    .modify(|w| w.set_PINS(w.PINS() | mask));
            }
            SgpioGroup::Ijkl => {
                self.regs
                    .IJKL_IRQ_SENS0()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s0)));
                self.regs
                    .IJKL_IRQ_SENS1()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s1)));
                self.regs
                    .IJKL_IRQ_SENS2()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s2)));
                self.regs
                    .IJKL_IRQ_EN()
                    .modify(|w| w.set_PINS(w.PINS() | mask));
            }
            SgpioGroup::Mnop => {
                self.regs
                    .MNOP_IRQ_SENS0()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s0)));
                self.regs
                    .MNOP_IRQ_SENS1()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s1)));
                self.regs
                    .MNOP_IRQ_SENS2()
                    .modify(|w| w.set_PINS(sens_apply(w.PINS(), mask, s2)));
                self.regs
                    .MNOP_IRQ_EN()
                    .modify(|w| w.set_PINS(w.PINS() | mask));
            }
        }
    }

    /// Disable interrupts for `mask` pins in `group`.
    pub fn disable_irq(&self, group: SgpioGroup, mask: u32) {
        match group {
            SgpioGroup::Abcd => self
                .regs
                .ABCD_IRQ_EN()
                .modify(|w| w.set_PINS(w.PINS() & !mask)),
            SgpioGroup::Efgh => self
                .regs
                .EFGH_IRQ_EN()
                .modify(|w| w.set_PINS(w.PINS() & !mask)),
            SgpioGroup::Ijkl => self
                .regs
                .IJKL_IRQ_EN()
                .modify(|w| w.set_PINS(w.PINS() & !mask)),
            SgpioGroup::Mnop => self
                .regs
                .MNOP_IRQ_EN()
                .modify(|w| w.set_PINS(w.PINS() & !mask)),
        }
    }

    /// Read and clear the interrupt status for `group`.
    ///
    /// Returns the bitmask of pins that fired (1 = fired).
    pub fn take_irq_status(&self, group: SgpioGroup) -> u32 {
        let sts = match group {
            SgpioGroup::Abcd => self.regs.ABCD_IRQ_STATUS().read().PINS(),
            SgpioGroup::Efgh => self.regs.EFGH_IRQ_STATUS().read().PINS(),
            SgpioGroup::Ijkl => self.regs.IJKL_IRQ_STATUS().read().PINS(),
            SgpioGroup::Mnop => self.regs.MNOP_IRQ_STATUS().read().PINS(),
        };
        // W1C: write the status bits back to clear them.
        match group {
            SgpioGroup::Abcd => self.regs.ABCD_IRQ_STATUS().modify(|w| w.set_PINS(sts)),
            SgpioGroup::Efgh => self.regs.EFGH_IRQ_STATUS().modify(|w| w.set_PINS(sts)),
            SgpioGroup::Ijkl => self.regs.IJKL_IRQ_STATUS().modify(|w| w.set_PINS(sts)),
            SgpioGroup::Mnop => self.regs.MNOP_IRQ_STATUS().modify(|w| w.set_PINS(sts)),
        }
        sts
    }

    // ── Async wait ────────────────────────────────────────────────────────────

    /// Async wait for any interrupt on `group`.
    ///
    /// Caller must have configured interrupts via [`enable_irq`] first.
    /// Returns the interrupt-status bitmask (which pins fired).
    pub fn wait_change(&self, group: SgpioGroup) -> SgpioWait<'_> {
        SgpioWait { sgpio: self, group }
    }

    /// Called from the SGPIO_MASTER ISR.
    pub(crate) fn on_interrupt() {
        let regs = pac::SGPIO;
        // Wake groups that have pending interrupts.
        if regs.ABCD_IRQ_STATUS().read().PINS() != 0 {
            SGPIO_WAKERS[0].wake();
        }
        if regs.EFGH_IRQ_STATUS().read().PINS() != 0 {
            SGPIO_WAKERS[1].wake();
        }
        if regs.IJKL_IRQ_STATUS().read().PINS() != 0 {
            SGPIO_WAKERS[2].wake();
        }
        if regs.MNOP_IRQ_STATUS().read().PINS() != 0 {
            SGPIO_WAKERS[3].wake();
        }
    }
}

// ── SgpioWait future ──────────────────────────────────────────────────────────

/// Future returned by [`Sgpio::wait_change`].
pub struct SgpioWait<'a> {
    sgpio: &'a Sgpio,
    group: SgpioGroup,
}

impl<'a> Future for SgpioWait<'a> {
    type Output = u32;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<u32> {
        let idx = self.group as usize;

        // Check if already pending.
        let sts = match self.group {
            SgpioGroup::Abcd => self.sgpio.regs.ABCD_IRQ_STATUS().read().PINS(),
            SgpioGroup::Efgh => self.sgpio.regs.EFGH_IRQ_STATUS().read().PINS(),
            SgpioGroup::Ijkl => self.sgpio.regs.IJKL_IRQ_STATUS().read().PINS(),
            SgpioGroup::Mnop => self.sgpio.regs.MNOP_IRQ_STATUS().read().PINS(),
        };
        if sts != 0 {
            // Clear and return.
            match self.group {
                SgpioGroup::Abcd => self
                    .sgpio
                    .regs
                    .ABCD_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
                SgpioGroup::Efgh => self
                    .sgpio
                    .regs
                    .EFGH_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
                SgpioGroup::Ijkl => self
                    .sgpio
                    .regs
                    .IJKL_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
                SgpioGroup::Mnop => self
                    .sgpio
                    .regs
                    .MNOP_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
            }
            return Poll::Ready(sts);
        }

        SGPIO_WAKERS[idx].register(cx.waker());

        // Re-check after registration to close the race.
        let sts = match self.group {
            SgpioGroup::Abcd => self.sgpio.regs.ABCD_IRQ_STATUS().read().PINS(),
            SgpioGroup::Efgh => self.sgpio.regs.EFGH_IRQ_STATUS().read().PINS(),
            SgpioGroup::Ijkl => self.sgpio.regs.IJKL_IRQ_STATUS().read().PINS(),
            SgpioGroup::Mnop => self.sgpio.regs.MNOP_IRQ_STATUS().read().PINS(),
        };
        if sts != 0 {
            match self.group {
                SgpioGroup::Abcd => self
                    .sgpio
                    .regs
                    .ABCD_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
                SgpioGroup::Efgh => self
                    .sgpio
                    .regs
                    .EFGH_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
                SgpioGroup::Ijkl => self
                    .sgpio
                    .regs
                    .IJKL_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
                SgpioGroup::Mnop => self
                    .sgpio
                    .regs
                    .MNOP_IRQ_STATUS()
                    .modify(|w| w.set_PINS(sts)),
            }
            Poll::Ready(sts)
        } else {
            Poll::Pending
        }
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn group_of(pin: u8) -> SgpioGroup {
    match pin / 32 {
        0 => SgpioGroup::Abcd,
        1 => SgpioGroup::Efgh,
        2 => SgpioGroup::Ijkl,
        _ => SgpioGroup::Mnop,
    }
}

/// Decode IrqSensitivity into (TYPE0, TYPE1, TYPE2) bits.
/// See SGPIO yaml: TYPE1=0,TYPE2=0,TYPE0=0→Falling; TYPE1=0,TYPE2=0,TYPE0=1→Rising;
///                TYPE1=0,TYPE2=1→DualEdge; TYPE1=1,TYPE0=0→LevelLow; TYPE1=1,TYPE0=1→LevelHigh.
fn sens_bits(s: IrqSensitivity) -> (bool, bool, bool) {
    // returns (TYPE0_bit, TYPE1_bit, TYPE2_bit)
    match s {
        IrqSensitivity::FallingEdge => (false, false, false),
        IrqSensitivity::RisingEdge => (true, false, false),
        IrqSensitivity::DualEdge => (false, false, true),
        IrqSensitivity::LevelLow => (false, true, false),
        IrqSensitivity::LevelHigh => (true, true, false),
    }
}

/// Apply a sensitivity bit to all masked pins in a 32-bit register value.
fn sens_apply(cur: u32, mask: u32, bit: bool) -> u32 {
    if bit {
        cur | mask
    } else {
        cur & !mask
    }
}

// ── Interrupt handler — IRQ 51 ────────────────────────────────────────────────

#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn SGPIO_MASTER() {
    Sgpio::on_interrupt();
}
