//! Microsecond delays from timer 3.

use aspeed_pac as pac;

/// Timer 3 at index 2 of the per-timer register arrays.
const T3: usize = 2;

/// Timer 3 free-running down from `u32::MAX` on the 1 MHz clock.
///
/// Owns timer 3 and stops it on drop, so a payload finds it quiet.
pub struct Delay(());

impl Delay {
    /// Start timer 3.
    pub fn new() -> Self {
        pac::TIMER.RELOAD(T3).write(|w| w.set_VALUE(u32::MAX));
        pac::TIMER.CTRL().modify(|w| {
            w.set_T3_EN(true);
            w.set_T3_CLK_SEL(true);
        });
        Delay(())
    }

    /// Current counter value; it counts down by one per microsecond.
    pub fn now(&self) -> u32 {
        pac::TIMER.STATUS(T3).read().VALUE()
    }

    /// Microseconds elapsed since `start`, a value of [`Delay::now`].
    pub fn elapsed_us(&self, start: u32) -> u32 {
        start.wrapping_sub(self.now())
    }

    /// Busy-wait for at least `us` microseconds.
    pub fn delay_us(&self, us: u32) {
        let start = self.now();
        while self.elapsed_us(start) <= us {
            core::hint::spin_loop();
        }
    }
}

impl Default for Delay {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Delay {
    fn drop(&mut self) {
        pac::TIMER.CTRL().modify(|w| {
            w.set_T3_EN(false);
            w.set_T3_CLK_SEL(false);
        });
    }
}
