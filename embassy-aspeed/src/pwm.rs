//! PWM and Fan Tachometer driver for AST2600 SSP (G6 IP, 16 channels each).
//!
//! This is the **G6 redesigned** PWM/TACH IP found in the AST2600.  It is
//! **not** compatible with the older `aspeed-pwm-tacho` IP in AST2400/AST2500.
//!
//! Base address: `0x7E61_0000` (AST2600 CM3 view).
//!
//! # PWM
//!
//! Up to 16 independent PWM outputs (CH0–15), each configurable with:
//! - Clock prescaler: `period_Q = input_clk / ((div_l + 1) << div_h)`
//! - Duty cycle period: `period = (duty_period + 1) * period_Q`
//! - Active time: `(falling - rising) / (duty_period + 1)` × 100 %
//!
//! # Fan Tachometer
//!
//! Up to 16 independent tachometer inputs (TACH0–15), each configurable with:
//! - Input clock divisor: `4^clk_div_t`
//! - Edge detection: falling-to-falling, rising-to-rising, or both
//! - Fan RPM formula: `rpm = (clk_hz * 60) / (value * ppr * 4^clk_div_t)`
//!   where `ppr` is pulses-per-revolution (typically 2 for PC fans).
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_aspeed::pwm::{PwmChannel, TachChannel, TachEdge};
//!
//! // Set fan PWM to ~50% at ~25 kHz (HCLK = 200 MHz)
//! let mut pwm = PwmChannel::new(0);
//! pwm.set_frequency_hz(25_000, 200_000_000);
//! pwm.set_duty_percent(50);
//! pwm.enable();
//!
//! // Read fan tachometer (2 pulses per revolution)
//! let mut tach = TachChannel::new(0, TachEdge::FallingToFalling);
//! tach.enable();
//! let rpm = tach.read_rpm(200_000_000, 2);
//! ```

use core::ptr;

// ── Base address ──────────────────────────────────────────────────────────────

const PWM_TACH_BASE: usize = 0x7E61_0000;
const N_CHANNELS: usize = 16;
const CH_STRIDE: usize = 0x10;

// ── Per-channel register offsets (bytes from channel base) ───────────────────

const PWM_CTRL_OFF: usize = 0x00;
const PWM_DUTY_OFF: usize = 0x04;
const TACH_CTRL_OFF: usize = 0x08;
const TACH_STS_OFF: usize = 0x0C;

// ── PWM_CTRL bit fields ───────────────────────────────────────────────────────

const PWM_CLK_DIV_L_MASK: u32 = 0xFF;
const PWM_CLK_DIV_H_SHIFT: u32 = 8;
const PWM_CLK_DIV_H_MASK: u32 = 0xF << 8;
const PWM_PIN_EN: u32 = 1 << 12;
const PWM_OPEN_DRAIN: u32 = 1 << 13;
const PWM_INVERSE: u32 = 1 << 14;
const PWM_LEVEL_OUT: u32 = 1 << 15;
const PWM_CLK_EN: u32 = 1 << 16;
const PWM_DUTY_SYNC_DIS: u32 = 1 << 17;
const PWM_DUTY_LOAD_WDT_EN: u32 = 1 << 18;
const PWM_LOAD_SEL_RISING_WDT: u32 = 1 << 19;

// ── PWM_DUTY bit fields ───────────────────────────────────────────────────────

const PWM_RISING_MASK: u32 = 0xFF;
const PWM_FALLING_SHIFT: u32 = 8;
const PWM_WDT_SHIFT: u32 = 16;
const PWM_PERIOD_SHIFT: u32 = 24;
const PWM_PERIOD_MAX: u32 = 0xFF;

// ── TACH_CTRL bit fields ──────────────────────────────────────────────────────

const TACH_THRESHOLD_MASK: u32 = 0xF_FFFF;
const TACH_CLK_DIV_T_SHIFT: u32 = 20;
const TACH_CLK_DIV_T_MASK: u32 = 0xF << 20;
const TACH_IO_EDGE_SHIFT: u32 = 24;
const TACH_IO_EDGE_MASK: u32 = 3 << 24;
const TACH_DEBOUNCE_SHIFT: u32 = 26;
const TACH_ENABLE: u32 = 1 << 28;
const TACH_LOOPBACK: u32 = 1 << 29;
const TACH_IER: u32 = 1 << 31;

// ── TACH_STS bit fields ───────────────────────────────────────────────────────

const TACH_VALUE_MASK: u32 = 0xF_FFFF;
const TACH_FULL_MEAS: u32 = 1 << 20;
const TACH_VALUE_UPDATE: u32 = 1 << 21;
const TACH_ISR: u32 = 1 << 31; // RW1C

// ── Error type ────────────────────────────────────────────────────────────────

/// Error type for PWM/TACH operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PwmError {
    /// Channel index out of range (must be 0–15).
    InvalidChannel,
}

// ── Edge selection ────────────────────────────────────────────────────────────

/// Tachometer edge detection mode.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TachEdge {
    /// Measure falling-to-falling period (default).
    FallingToFalling,
    /// Measure rising-to-rising period.
    RisingToRising,
    /// Measure both edges (half-cycle, double the RPM count).
    BothEdges,
}

impl TachEdge {
    fn encoding(self) -> u32 {
        match self {
            TachEdge::FallingToFalling => 0,
            TachEdge::RisingToRising => 1,
            TachEdge::BothEdges => 2,
        }
    }
}

// ── PwmChannel ────────────────────────────────────────────────────────────────

/// Single PWM output channel.
pub struct PwmChannel {
    base: usize,
}

impl PwmChannel {
    /// Create a handle for PWM channel `ch` (0–15).
    ///
    /// Does not configure or enable the output.
    /// Returns [`PwmError::InvalidChannel`] if `ch >= 16`.
    pub fn new(ch: u8) -> Result<Self, PwmError> {
        if ch as usize >= N_CHANNELS {
            return Err(PwmError::InvalidChannel);
        }
        Ok(Self {
            base: PWM_TACH_BASE + ch as usize * CH_STRIDE,
        })
    }

    /// Configure the PWM output for a target frequency and 50% duty cycle.
    ///
    /// Uses `duty_period = 255` (maximum) for fine-grained duty control.
    /// Sets `div_h = 0` (no shift), only adjusts `div_l`.
    ///
    /// `target_hz` — desired PWM output frequency in Hz.
    /// `input_clk_hz` — input clock frequency in Hz (typically HCLK = 200 MHz on AST2600 SSP).
    ///
    /// After this call, use [`set_duty_percent`] to change the duty cycle.
    /// Call [`enable`] to start the output.
    pub fn set_frequency_hz(&self, target_hz: u32, input_clk_hz: u32) {
        // period = (duty_period + 1) * (div_l + 1) / input_clk
        // → (div_l + 1) = input_clk / (target_hz * (duty_period + 1))
        let duty_period = PWM_PERIOD_MAX;
        let divisor = input_clk_hz / (target_hz * (duty_period + 1));
        let div_l = divisor.saturating_sub(1).min(0xFF);

        let ctrl = unsafe { ptr::read_volatile(self.ctrl()) };
        let new_ctrl = (ctrl & !PWM_CLK_DIV_L_MASK & !PWM_CLK_DIV_H_MASK)
            | (div_l & 0xFF);
        unsafe { ptr::write_volatile(self.ctrl(), new_ctrl) };

        // Set duty_period; rising = 0, falling = period+1 → 100% (set via set_duty_percent).
        let duty_reg = (duty_period << PWM_PERIOD_SHIFT) | (0 << 0); // rising=0, falling=0→100%
        unsafe { ptr::write_volatile(self.duty(), duty_reg) };
    }

    /// Set duty cycle as a percentage (0–100).
    ///
    /// Assumes `duty_period = 255` (set by [`set_frequency_hz`] or via [`set_raw`]).
    /// Duty = `(falling - rising) / 256 * 100 %`.  `rising = 0` always.
    ///
    /// 0% → CLK_EN cleared (output inactive).
    /// 100% → falling = rising = 0 (always active when CLK_EN set).
    pub fn set_duty_percent(&self, percent: u8) {
        let duty = unsafe { ptr::read_volatile(self.duty()) };
        let period = (duty >> PWM_PERIOD_SHIFT) as u8;

        let falling = if percent == 0 {
            0u8
        } else if percent >= 100 {
            0u8 // falling == rising == 0 → 100%
        } else {
            ((period as u32 + 1) * percent as u32 / 100) as u8
        };

        let new_duty = (duty & 0xFF00_00FF) | ((falling as u32) << PWM_FALLING_SHIFT);
        unsafe { ptr::write_volatile(self.duty(), new_duty) };

        // Update CLK_EN: clear for 0%, set otherwise.
        let ctrl = unsafe { ptr::read_volatile(self.ctrl()) };
        let new_ctrl = if percent == 0 {
            ctrl & !PWM_CLK_EN
        } else {
            ctrl | PWM_CLK_EN
        };
        unsafe { ptr::write_volatile(self.ctrl(), new_ctrl) };
    }

    /// Set raw PWM control and duty registers directly.
    ///
    /// For advanced use when the helper functions are insufficient.
    pub fn set_raw(&self, ctrl: u32, duty: u32) {
        unsafe {
            ptr::write_volatile(self.ctrl(), ctrl);
            ptr::write_volatile(self.duty(), duty);
        }
    }

    /// Enable the PWM output pin (set PIN_EN and CLK_EN).
    pub fn enable(&self) {
        let ctrl = unsafe { ptr::read_volatile(self.ctrl()) };
        unsafe { ptr::write_volatile(self.ctrl(), ctrl | PWM_PIN_EN | PWM_CLK_EN) };
    }

    /// Disable the PWM output pin (clear PIN_EN; duty counter keeps running).
    pub fn disable(&self) {
        let ctrl = unsafe { ptr::read_volatile(self.ctrl()) };
        unsafe { ptr::write_volatile(self.ctrl(), ctrl & !PWM_PIN_EN) };
    }

    /// Set output polarity.
    ///
    /// `true` = active low (inverted).  `false` = active high (normal).
    pub fn set_inverted(&self, inverted: bool) {
        let ctrl = unsafe { ptr::read_volatile(self.ctrl()) };
        let new = if inverted {
            ctrl | PWM_INVERSE
        } else {
            ctrl & !PWM_INVERSE
        };
        unsafe { ptr::write_volatile(self.ctrl(), new) };
    }

    fn ctrl(&self) -> *mut u32 {
        (self.base + PWM_CTRL_OFF) as *mut u32
    }

    fn duty(&self) -> *mut u32 {
        (self.base + PWM_DUTY_OFF) as *mut u32
    }
}

// ── TachChannel ──────────────────────────────────────────────────────────────

/// Single fan tachometer input channel.
pub struct TachChannel {
    base: usize,
    edge: TachEdge,
}

impl TachChannel {
    /// Create a handle for tachometer channel `ch` (0–15).
    ///
    /// `edge` — edge detection mode.
    /// Returns [`PwmError::InvalidChannel`] if `ch >= 16`.
    pub fn new(ch: u8, edge: TachEdge) -> Result<Self, PwmError> {
        if ch as usize >= N_CHANNELS {
            return Err(PwmError::InvalidChannel);
        }
        Ok(Self {
            base: PWM_TACH_BASE + ch as usize * CH_STRIDE,
            edge,
        })
    }

    /// Enable the tachometer channel.
    ///
    /// Sets a default stopped-fan threshold of `0x7FFFF` (maximum).
    /// Call [`set_threshold`] to configure fan-stopped detection.
    pub fn enable(&self) {
        let ctrl = (0x7_FFFF & TACH_THRESHOLD_MASK)
            | ((self.edge.encoding()) << TACH_IO_EDGE_SHIFT)
            | TACH_ENABLE;
        unsafe { ptr::write_volatile(self.tach_ctrl(), ctrl) };
    }

    /// Disable the tachometer channel.
    pub fn disable(&self) {
        let ctrl = unsafe { ptr::read_volatile(self.tach_ctrl()) };
        unsafe { ptr::write_volatile(self.tach_ctrl(), ctrl & !TACH_ENABLE) };
    }

    /// Set the fan-stopped threshold.
    ///
    /// The hardware fires an interrupt (if `IER` set) when the counter between
    /// edges exceeds this value.  Use [`threshold_from_rpm`] to compute the
    /// value for a minimum RPM.
    pub fn set_threshold(&self, threshold: u32) {
        let ctrl = unsafe { ptr::read_volatile(self.tach_ctrl()) };
        let new = (ctrl & !TACH_THRESHOLD_MASK) | (threshold & TACH_THRESHOLD_MASK);
        unsafe { ptr::write_volatile(self.tach_ctrl(), new) };
    }

    /// Compute the threshold value for a minimum RPM (for fan-stopped detection).
    ///
    /// `min_rpm` — minimum expected RPM before declaring fan stopped.
    /// `input_clk_hz` — input clock (typically HCLK = 200 MHz on AST2600 SSP).
    /// `ppr` — pulses per revolution of the fan (typically 2).
    pub fn threshold_from_rpm(min_rpm: u32, input_clk_hz: u32, ppr: u32) -> u32 {
        // threshold = clk_hz * 60 / (min_rpm * ppr * 4^0)
        // Using clk_div_t = 0 (divisor = 1)
        let val = input_clk_hz / (min_rpm / 60 * ppr);
        val.min(TACH_THRESHOLD_MASK)
    }

    /// Read the latest captured tachometer count.
    ///
    /// Returns the raw counter value (clock cycles between edges, divided by
    /// `4^clk_div_t`).  Convert to RPM with [`TachChannel::count_to_rpm`].
    ///
    /// Returns `None` if no complete measurement is available yet
    /// (before the first edge pair after [`enable`]).
    pub fn read_count(&self) -> Option<u32> {
        let sts = unsafe { ptr::read_volatile(self.tach_sts()) };
        if sts & TACH_FULL_MEAS == 0 {
            return None;
        }
        Some(sts & TACH_VALUE_MASK)
    }

    /// Convert a raw tachometer count to RPM.
    ///
    /// `count` — value from [`read_count`].
    /// `input_clk_hz` — input clock frequency in Hz.
    /// `ppr` — pulses per revolution (typically 2 for PC fans).
    /// `clk_div_t` — clock divisor exponent (0 = ÷1, 1 = ÷4, etc.).
    ///
    /// Returns 0 if `count == 0` (avoid division by zero).
    pub fn count_to_rpm(count: u32, input_clk_hz: u32, ppr: u32, clk_div_t: u32) -> u32 {
        if count == 0 {
            return 0;
        }
        let divisor = 4u32.pow(clk_div_t);
        (input_clk_hz / divisor) * 60 / (count * ppr)
    }

    /// Read fan speed in RPM.
    ///
    /// Convenience wrapper around [`read_count`] + [`count_to_rpm`].
    /// Uses `clk_div_t = 0` (no prescaling).
    ///
    /// Returns `None` if no measurement is available yet.
    pub fn read_rpm(&self, input_clk_hz: u32, ppr: u32) -> Option<u32> {
        let count = self.read_count()?;
        Some(Self::count_to_rpm(count, input_clk_hz, ppr, 0))
    }

    /// Clear the tachometer interrupt status flag (RW1C).
    pub fn clear_interrupt(&self) {
        let sts = unsafe { ptr::read_volatile(self.tach_sts()) };
        unsafe { ptr::write_volatile(self.tach_sts(), sts | TACH_ISR) };
    }

    fn tach_ctrl(&self) -> *mut u32 {
        (self.base + TACH_CTRL_OFF) as *mut u32
    }

    fn tach_sts(&self) -> *mut u32 {
        (self.base + TACH_STS_OFF) as *mut u32
    }
}
