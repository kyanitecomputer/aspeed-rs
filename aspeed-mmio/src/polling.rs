/// Error returned when a polling operation exceeds its iteration/time limit.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimeoutError;

/// Synchronous polling: repeatedly call `read_fn` until `predicate` returns
/// `true` or `max_iterations` is exhausted.
///
/// Between iterations the CPU executes a `spin_loop` hint to reduce power
/// and avoid bus contention.
///
/// # Example
///
/// ```rust,ignore
/// use aspeed_mmio::{poll_until, TimeoutError};
///
/// let status = poll_until(
///     || block.read32(STATUS_OFFSET),
///     |s| s & DONE_BIT != 0,
///     10_000,
/// )?;
/// ```
#[inline]
pub fn poll_until<T, R, P>(read_fn: R, predicate: P, max_iterations: u32) -> Result<T, TimeoutError>
where
    R: Fn() -> T,
    P: Fn(&T) -> bool,
{
    for _ in 0..max_iterations {
        let val = read_fn();
        if predicate(&val) {
            return Ok(val);
        }
        core::hint::spin_loop();
    }
    Err(TimeoutError)
}

/// Asynchronous polling: repeatedly call `read_fn` until `predicate` returns
/// `true`, yielding to the executor between attempts.
///
/// Uses Embassy's `Timer::after` to sleep between polls, avoiding busy-wait
/// and allowing other async tasks to run.
///
/// # Example
///
/// ```rust,ignore
/// use aspeed_mmio::poll_until_async;
/// use embassy_time::Duration;
///
/// let status = poll_until_async(
///     || block.read32(STATUS_OFFSET),
///     |s| s & DONE_BIT != 0,
///     Duration::from_micros(100),  // poll interval
///     Duration::from_millis(1000), // total timeout
/// ).await?;
/// ```
#[cfg(feature = "async")]
pub async fn poll_until_async<T, R, P>(
    read_fn: R,
    predicate: P,
    interval: embassy_time::Duration,
    timeout: embassy_time::Duration,
) -> Result<T, TimeoutError>
where
    R: Fn() -> T,
    P: Fn(&T) -> bool,
{
    use embassy_time::{Instant, Timer};

    let deadline = Instant::now() + timeout;
    loop {
        let val = read_fn();
        if predicate(&val) {
            return Ok(val);
        }
        if Instant::now() >= deadline {
            return Err(TimeoutError);
        }
        Timer::after(interval).await;
    }
}
