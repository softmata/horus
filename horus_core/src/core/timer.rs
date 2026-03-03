//! Timer and Rate utilities for HORUS nodes.
//!
//! These types provide time-based control inside node `tick()` methods and
//! standalone background threads, closing the gap with ROS2's
//! `rclcpp::TimerBase` and `ros::Rate`.
//!
//! # Timer — periodic events inside a node
//!
//! [`HorusTimer`] fires at a fixed wall-clock interval regardless of the
//! scheduler's tick rate.  Typical usage:
//!
//! ```rust,ignore
//! use horus_core::core::timer::HorusTimer;
//!
//! pub struct MyNode {
//!     publish_timer: HorusTimer,  // fire every 100 ms
//!     status_timer: HorusTimer,   // fire every 1 s
//! }
//!
//! impl Node for MyNode {
//!     fn init(&mut self) -> HorusResult<()> {
//!         self.publish_timer = HorusTimer::every_ms(100);
//!         self.status_timer  = HorusTimer::every_secs(1.0);
//!         Ok(())
//!     }
//!
//!     fn tick(&mut self) {
//!         if self.publish_timer.is_ready() {
//!             // Runs at ~10 Hz regardless of scheduler tick rate
//!         }
//!         if self.status_timer.is_ready() {
//!             hlog!(info, "Still alive");
//!         }
//!     }
//! }
//! ```
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

use std::time::{Duration, Instant};

// ─── HorusTimer ───────────────────────────────────────────────────────────────

/// A wall-clock timer that fires at a fixed interval.
///
/// Designed to be stored as a field in a HORUS [`Node`](crate::core::Node) and
/// checked each `tick()`.  The timer is completely independent of the scheduler
/// tick rate — it fires based on elapsed real time.
///
/// # Example
///
/// ```rust,ignore
/// let mut timer = HorusTimer::every_hz(10.0);  // fire 10 times per second
/// loop {
///     if timer.is_ready() {
///         do_something_at_10hz();
///     }
/// }
/// ```
#[derive(Debug, Clone)]
pub struct HorusTimer {
    interval: Duration,
    last_fire: Instant,
}

impl HorusTimer {
    /// Create a timer that fires every `interval`.
    pub fn every(interval: Duration) -> Self {
        Self {
            interval,
            // Set last_fire in the past so the timer fires immediately on first check.
            last_fire: Instant::now() - interval,
        }
    }

    /// Create a timer that fires at `hz` Hz (fires immediately on first check).
    ///
    /// # Panics
    ///
    /// Panics if `hz` is zero or negative.
    pub fn every_hz(hz: f64) -> Self {
        assert!(hz > 0.0, "HorusTimer: rate must be positive (got {})", hz);
        Self::every(Duration::from_secs_f64(1.0 / hz))
    }

    /// Create a timer that fires every `ms` milliseconds.
    pub fn every_ms(ms: u64) -> Self {
        Self::every(Duration::from_millis(ms))
    }

    /// Create a timer that fires every `secs` seconds.
    pub fn every_secs(secs: f64) -> Self {
        assert!(secs > 0.0, "HorusTimer: period must be positive");
        Self::every(Duration::from_secs_f64(secs))
    }

    /// Returns `true` if the interval has elapsed since the last fire.
    ///
    /// When this returns `true`, the internal clock resets — the *next* call
    /// will only return `true` after another full `interval` has passed.
    #[inline]
    pub fn is_ready(&mut self) -> bool {
        let elapsed = self.last_fire.elapsed();
        if elapsed >= self.interval {
            // Advance by one interval to avoid drift accumulation.
            self.last_fire += self.interval;
            // Guard against large gaps (e.g. after a long pause): reset fully.
            if self.last_fire.elapsed() >= self.interval {
                self.last_fire = Instant::now();
            }
            true
        } else {
            false
        }
    }

    /// Reset the timer without firing.
    ///
    /// The next `is_ready()` call will return `true` after a full interval.
    #[inline]
    pub fn reset(&mut self) {
        self.last_fire = Instant::now();
    }

    /// Time remaining until the next fire.
    ///
    /// Returns `Duration::ZERO` if the timer has already elapsed.
    #[inline]
    pub fn remaining(&self) -> Duration {
        self.interval.saturating_sub(self.last_fire.elapsed())
    }

    /// Time elapsed since the last fire (or since construction).
    #[inline]
    pub fn elapsed(&self) -> Duration {
        self.last_fire.elapsed()
    }

    /// The configured interval between fires.
    #[inline]
    pub fn interval(&self) -> Duration {
        self.interval
    }

    /// Equivalent rate in Hz.
    #[inline]
    pub fn hz(&self) -> f64 {
        1.0 / self.interval.as_secs_f64()
    }
}

impl Default for HorusTimer {
    /// Creates a 1 Hz timer (fires every second).
    fn default() -> Self {
        Self::every_hz(1.0)
    }
}

// ─── HorusRate ────────────────────────────────────────────────────────────────

/// A rate limiter for background threads.
///
/// The equivalent of `ros::Rate` — call `sleep()` at the end of a loop
/// iteration to maintain a target frequency.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::core::timer::HorusRate;
///
/// let mut rate = HorusRate::new(50.0); // 50 Hz
/// loop {
///     let start = Instant::now();
///     do_work(); // ≤ 20 ms of work
///     rate.sleep(); // sleeps the remainder of the 20 ms window
///
///     // Optional: log actual rate
///     hlog!(debug, "actual rate: {:.1} Hz", rate.actual_hz());
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
    /// If the work took longer than a full period, no sleep occurs (the loop
    /// catches up immediately).  The next cycle start is advanced by one period
    /// to avoid drift accumulation.
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
        if period > 0.0 {
            1.0 / period
        } else {
            f64::INFINITY
        }
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
    ///
    /// If `true`, a call to `sleep()` will return immediately.
    #[inline]
    pub fn is_late(&self) -> bool {
        self.last_cycle_start.elapsed() >= self.period
    }
}

// ─── Stopwatch ────────────────────────────────────────────────────────────────

/// A simple stopwatch for measuring elapsed time in nodes.
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
        Self {
            start: Instant::now(),
        }
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
    fn test_timer_fires_immediately() {
        let mut t = HorusTimer::every_ms(100);
        // Should fire on first check (last_fire is in the past)
        assert!(t.is_ready());
    }

    #[test]
    fn test_timer_does_not_fire_twice_immediately() {
        let mut t = HorusTimer::every_ms(100);
        assert!(t.is_ready()); // first fire
        assert!(!t.is_ready()); // should not fire again immediately
    }

    #[test]
    fn test_timer_fires_after_interval() {
        let mut t = HorusTimer::every_ms(50);
        assert!(t.is_ready()); // initial fire
        std::thread::sleep(Duration::from_millis(60));
        assert!(t.is_ready()); // should fire after 50 ms
    }

    #[test]
    fn test_timer_reset() {
        let mut t = HorusTimer::every_ms(50);
        assert!(t.is_ready()); // initial fire
        t.reset();
        assert!(!t.is_ready()); // reset — should not fire
    }

    #[test]
    fn test_timer_hz() {
        let t = HorusTimer::every_hz(10.0);
        assert!((t.hz() - 10.0).abs() < 0.01);
    }

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
        // After lap, elapsed resets
        assert!(sw.elapsed() < Duration::from_millis(5));
    }
}
