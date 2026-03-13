//! Ergonomic duration and frequency helpers for RT configuration.
//!
//! These extensions replace verbose `Duration::from_micros(200)` with `200_u64.us()`,
//! and provide a `Frequency` type that auto-derives budget and deadline from rate.
//!
//! # Examples
//!
//! ```rust
//! use horus_core::core::duration_ext::{DurationExt, Frequency};
//!
//! // Duration helpers
//! let budget = 200_u64.us();   // Duration::from_micros(200)
//! let deadline = 1_u64.ms();   // Duration::from_millis(1)
//!
//! // Frequency with auto-derived timing
//! let freq = 100_u64.hz();     // 100 Hz
//! assert_eq!(freq.period(), 10_u64.ms());
//! ```

use std::fmt;
use std::time::Duration;

/// A frequency value (in Hz) for configuring node tick rates.
///
/// Created via the `.hz()` extension method. Carries frequency semantics
/// and can auto-derive timing parameters (period, budget, deadline).
///
/// # Examples
///
/// ```rust
/// use horus_core::core::duration_ext::DurationExt;
///
/// let freq = 100_u64.hz();
/// assert_eq!(freq.value(), 100.0);
///
/// // Period = 1/frequency
/// let period = freq.period();
/// assert_eq!(period.as_millis(), 10);
///
/// // Budget = 80% of period (safe default)
/// let budget = freq.budget_default();
/// assert_eq!(budget.as_millis(), 8);
///
/// // Deadline = 95% of period
/// let deadline = freq.deadline_default();
/// assert_eq!(deadline.as_micros(), 9500);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frequency(f64);

impl fmt::Display for Frequency {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} Hz", self.0)
    }
}

impl Frequency {
    /// The frequency in Hz.
    #[inline]
    pub fn value(&self) -> f64 {
        self.0
    }

    /// Period (1 / frequency).
    #[inline]
    pub fn period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.0)
    }

    /// Default tick budget: 80% of period.
    ///
    /// Leaves 20% headroom for scheduling overhead, cache misses,
    /// and interrupt handling.
    #[inline]
    pub fn budget_default(&self) -> Duration {
        Duration::from_secs_f64(0.8 / self.0)
    }

    /// Default deadline: 95% of period.
    ///
    /// Tighter than budget — the absolute latest a tick can finish
    /// before the next one is due.
    #[inline]
    pub fn deadline_default(&self) -> Duration {
        Duration::from_secs_f64(0.95 / self.0)
    }
}

/// Extension trait for ergonomic duration and frequency construction.
///
/// Implemented for `u64` and `f64`.
///
/// # Examples
///
/// ```rust
/// use horus_core::core::duration_ext::DurationExt;
/// use std::time::Duration;
///
/// // Integer usage
/// assert_eq!(200_u64.us(), Duration::from_micros(200));
/// assert_eq!(5_u64.ms(), Duration::from_millis(5));
/// assert_eq!(100_u64.hz().period(), Duration::from_millis(10));
///
/// // Float usage
/// assert_eq!(1.5.ms(), Duration::from_micros(1500));
/// assert_eq!(33.33.hz().value(), 33.33);
/// ```
pub trait DurationExt {
    /// Create a `Duration` in nanoseconds.
    fn ns(self) -> Duration;
    /// Create a `Duration` in microseconds.
    fn us(self) -> Duration;
    /// Create a `Duration` in milliseconds.
    fn ms(self) -> Duration;
    /// Create a `Duration` in seconds.
    fn secs(self) -> Duration;
    /// Create a `Frequency` in Hz.
    fn hz(self) -> Frequency;
}

impl DurationExt for u64 {
    #[inline]
    fn ns(self) -> Duration {
        Duration::from_nanos(self)
    }

    #[inline]
    fn us(self) -> Duration {
        Duration::from_micros(self)
    }

    #[inline]
    fn ms(self) -> Duration {
        Duration::from_millis(self)
    }

    #[inline]
    fn secs(self) -> Duration {
        Duration::from_secs(self)
    }

    #[inline]
    fn hz(self) -> Frequency {
        assert!(self > 0, "frequency must be positive (got 0)");
        Frequency(self as f64)
    }
}

impl DurationExt for f64 {
    #[inline]
    fn ns(self) -> Duration {
        Duration::from_secs_f64(self / 1_000_000_000.0)
    }

    #[inline]
    fn us(self) -> Duration {
        Duration::from_secs_f64(self / 1_000_000.0)
    }

    #[inline]
    fn ms(self) -> Duration {
        Duration::from_secs_f64(self / 1_000.0)
    }

    #[inline]
    fn secs(self) -> Duration {
        Duration::from_secs_f64(self)
    }

    #[inline]
    fn hz(self) -> Frequency {
        assert!(
            self.is_finite() && self > 0.0,
            "frequency must be finite and positive (got {self})"
        );
        Frequency(self)
    }
}

// Also implement for i32/i64 for convenience (common literal types)
impl DurationExt for i32 {
    #[inline]
    fn ns(self) -> Duration {
        assert!(self >= 0, "duration must be non-negative (got {self})");
        Duration::from_nanos(self as u64)
    }

    #[inline]
    fn us(self) -> Duration {
        assert!(self >= 0, "duration must be non-negative (got {self})");
        Duration::from_micros(self as u64)
    }

    #[inline]
    fn ms(self) -> Duration {
        assert!(self >= 0, "duration must be non-negative (got {self})");
        Duration::from_millis(self as u64)
    }

    #[inline]
    fn secs(self) -> Duration {
        assert!(self >= 0, "duration must be non-negative (got {self})");
        Duration::from_secs(self as u64)
    }

    #[inline]
    fn hz(self) -> Frequency {
        assert!(self > 0, "frequency must be positive (got {self})");
        Frequency(self as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Duration helpers ──

    #[test]
    fn us_from_u64() {
        assert_eq!(200_u64.us(), Duration::from_micros(200));
        assert_eq!(0_u64.us(), Duration::ZERO);
        assert_eq!(1_000_000_u64.us(), Duration::from_secs(1));
    }

    #[test]
    fn ms_from_u64() {
        assert_eq!(5_u64.ms(), Duration::from_millis(5));
        assert_eq!(0_u64.ms(), Duration::ZERO);
        assert_eq!(1000_u64.ms(), Duration::from_secs(1));
    }

    #[test]
    fn us_from_f64() {
        let d = 1.5.us();
        assert_eq!(d, Duration::from_nanos(1500));
    }

    #[test]
    fn ms_from_f64() {
        let d = 1.5.ms();
        assert_eq!(d, Duration::from_micros(1500));
    }

    #[test]
    fn us_from_i32() {
        assert_eq!(100_i32.us(), Duration::from_micros(100));
    }

    #[test]
    fn ms_from_i32() {
        assert_eq!(10_i32.ms(), Duration::from_millis(10));
    }

    #[test]
    #[should_panic(expected = "non-negative")]
    fn negative_i32_us_panics() {
        (-1_i32).us();
    }

    #[test]
    #[should_panic(expected = "non-negative")]
    fn negative_i32_ms_panics() {
        (-1_i32).ms();
    }

    // ── Frequency ──

    #[test]
    fn hz_from_u64() {
        let f = 100_u64.hz();
        assert_eq!(f.value(), 100.0);
    }

    #[test]
    fn hz_from_f64() {
        let f = 33.33.hz();
        assert!((f.value() - 33.33).abs() < 1e-10);
    }

    #[test]
    fn hz_from_i32() {
        let f = 500_i32.hz();
        assert_eq!(f.value(), 500.0);
    }

    #[test]
    fn period_calculation() {
        assert_eq!(100_u64.hz().period(), Duration::from_millis(10));
        assert_eq!(1000_u64.hz().period(), Duration::from_millis(1));
        assert_eq!(1_u64.hz().period(), Duration::from_secs(1));
    }

    #[test]
    fn budget_default_is_80_percent() {
        let freq = 100_u64.hz(); // period = 10ms
        let budget = freq.budget_default();
        assert_eq!(budget, Duration::from_millis(8)); // 80% of 10ms
    }

    #[test]
    fn deadline_default_is_95_percent() {
        let freq = 100_u64.hz(); // period = 10ms
        let deadline = freq.deadline_default();
        assert_eq!(deadline, Duration::from_micros(9500)); // 95% of 10ms
    }

    #[test]
    fn high_frequency() {
        let freq = 10_000_u64.hz(); // 10kHz, period = 100us
        assert_eq!(freq.period(), Duration::from_micros(100));
        assert_eq!(freq.budget_default(), Duration::from_micros(80));
        assert_eq!(freq.deadline_default(), Duration::from_nanos(95_000));
    }

    #[test]
    fn low_frequency() {
        let freq = 1_u64.hz(); // 1Hz, period = 1s
        assert_eq!(freq.period(), Duration::from_secs(1));
        assert_eq!(freq.budget_default(), Duration::from_millis(800));
        assert_eq!(freq.deadline_default(), Duration::from_millis(950));
    }

    #[test]
    fn fractional_frequency() {
        let freq = 0.5.hz(); // 0.5Hz, period = 2s
        assert_eq!(freq.period(), Duration::from_secs(2));
    }

    #[test]
    #[should_panic(expected = "positive")]
    fn zero_hz_u64_panics() {
        0_u64.hz();
    }

    #[test]
    #[should_panic(expected = "positive")]
    fn zero_hz_f64_panics() {
        0_u64.hz();
    }

    #[test]
    #[should_panic(expected = "finite")]
    fn inf_hz_panics() {
        f64::INFINITY.hz();
    }

    #[test]
    #[should_panic(expected = "finite")]
    fn nan_hz_panics() {
        f64::NAN.hz();
    }

    #[test]
    #[should_panic(expected = "positive")]
    fn negative_hz_i32_panics() {
        (-10_i32).hz();
    }
}
