//! Rate and timing utilities for HORUS background threads.
//!
//! # Rate — background thread rate limiting
//!
//! [`HorusRate`] is the equivalent of `ros::Rate` for standalone threads that
//! want to run at a fixed frequency without a scheduler.
//!
//! ```rust,ignore
//! use horus_core::core::timer::HorusRate;
//!
//! std::thread::spawn(|| {
//!     let mut rate = HorusRate::new(100.0); // 100 Hz
//!     loop {
//!         do_work();
//!         rate.sleep(); // sleeps the remaining fraction of 10 ms
//!     }
//! });
//! ```
//!
//! For sub-frequency work inside a scheduled node `tick()`, use a counter:
//!
//! ```rust,ignore
//! fn tick(&mut self) {
//!     self.tick_count += 1;
//!     if self.tick_count % 100 == 0 {  // every 100 ticks
//!         self.publish_diagnostics();
//!     }
//! }
//! ```

use std::time::{Duration, Instant};

// ─── HorusRate ────────────────────────────────────────────────────────────────

/// A rate limiter for background threads.
///
/// Call `sleep()` at the end of each loop iteration to maintain a target
/// frequency.  Drift-compensated: if work takes longer than the period,
/// sleep is skipped and the next cycle catches up.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::core::timer::HorusRate;
///
/// let mut rate = HorusRate::new(50.0); // 50 Hz
/// loop {
///     do_work(); // ≤ 20 ms of work
///     rate.sleep();
///     hlog!(debug, "actual: {:.1} Hz", rate.actual_hz());
/// }
/// ```
#[derive(Debug)]
pub struct HorusRate {
    period: Duration,
    last_cycle_start: Instant,
    /// Smoothed actual period (for `actual_hz()`).
    smoothed_period: Option<Duration>,
}

impl HorusRate {
    /// Create a rate limiter targeting `hz` Hz.
    ///
    /// # Panics
    ///
    /// Panics if `hz` is zero or negative.
    pub fn new(hz: f64) -> Self {
        assert!(hz > 0.0, "HorusRate: frequency must be positive (got {})", hz);
        Self {
            period: Duration::from_secs_f64(1.0 / hz),
            last_cycle_start: Instant::now(),
            smoothed_period: None,
        }
    }

    /// Sleep for the remainder of the current period.
    ///
    /// If the work took longer than a full period, no sleep occurs and the
    /// next cycle start advances by one period (no drift accumulation).
    pub fn sleep(&mut self) {
        let elapsed = self.last_cycle_start.elapsed();

        // Update smoothed actual period.
        self.smoothed_period = Some(match self.smoothed_period {
            None => elapsed,
            Some(prev) => Duration::from_secs_f64(
                0.9 * prev.as_secs_f64() + 0.1 * elapsed.as_secs_f64(),
            ),
        });

        if let Some(remaining) = self.period.checked_sub(elapsed) {
            std::thread::sleep(remaining);
        }

        // Advance by one period (drift compensation).
        self.last_cycle_start += self.period;

        // If we fell far behind, reset to now.
        if self.last_cycle_start.elapsed() > self.period {
            self.last_cycle_start = Instant::now();
        }
    }

    /// Actual achieved frequency in Hz (exponentially smoothed).
    ///
    /// Returns the target frequency before the first `sleep()` call.
    #[inline]
    pub fn actual_hz(&self) -> f64 {
        let period = self
            .smoothed_period
            .unwrap_or(self.period)
            .as_secs_f64();
        if period > 0.0 { 1.0 / period } else { f64::INFINITY }
    }

    /// The target frequency in Hz.
    #[inline]
    pub fn target_hz(&self) -> f64 {
        1.0 / self.period.as_secs_f64()
    }

    /// The target period.
    #[inline]
    pub fn period(&self) -> Duration {
        self.period
    }

    /// Reset the cycle start to now.
    ///
    /// Useful after a long pause to avoid immediately sleeping zero.
    #[inline]
    pub fn reset(&mut self) {
        self.last_cycle_start = Instant::now();
    }

    /// Whether the current cycle has already exceeded the target period.
    #[inline]
    pub fn is_late(&self) -> bool {
        self.last_cycle_start.elapsed() >= self.period
    }
}

// ─── Stopwatch ────────────────────────────────────────────────────────────────

/// A simple stopwatch for measuring elapsed time.
///
/// ```rust,ignore
/// let mut sw = Stopwatch::start();
/// do_work();
/// hlog!(debug, "work took {:.2} ms", sw.elapsed_ms());
/// ```
#[derive(Debug, Clone)]
pub struct Stopwatch {
    start: Instant,
}

impl Stopwatch {
    /// Create a stopwatch that starts immediately.
    #[inline]
    pub fn start() -> Self {
        Self { start: Instant::now() }
    }

    /// Elapsed time since `start()`.
    #[inline]
    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    /// Elapsed time in microseconds.
    #[inline]
    pub fn elapsed_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }

    /// Elapsed time in milliseconds (f64).
    #[inline]
    pub fn elapsed_ms(&self) -> f64 {
        self.start.elapsed().as_secs_f64() * 1000.0
    }

    /// Reset the stopwatch.
    #[inline]
    pub fn reset(&mut self) {
        self.start = Instant::now();
    }

    /// Return elapsed time and reset.
    #[inline]
    pub fn lap(&mut self) -> Duration {
        let elapsed = self.start.elapsed();
        self.start = Instant::now();
        elapsed
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_rate_target_hz() {
        let r = HorusRate::new(100.0);
        assert!((r.target_hz() - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_stopwatch_elapsed() {
        let sw = Stopwatch::start();
        std::thread::sleep(Duration::from_millis(10));
        assert!(sw.elapsed() >= Duration::from_millis(10));
        assert!(sw.elapsed_ms() >= 10.0);
    }

    #[test]
    fn test_stopwatch_lap() {
        let mut sw = Stopwatch::start();
        std::thread::sleep(Duration::from_millis(10));
        let lap = sw.lap();
        assert!(lap >= Duration::from_millis(10));
        assert!(sw.elapsed() < Duration::from_millis(5));
    }
}
