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

    // ========================================================================
    // Clock and timing edge cases
    // ========================================================================

    #[test]
    fn test_very_small_frequency_period() {
        // 0.001 Hz = 1000 second period
        let freq = 0.001_f64.hz();
        let period = freq.period();
        assert_eq!(period, Duration::from_secs(1000));
    }

    #[test]
    fn test_very_large_frequency_period() {
        // 1MHz = 1us period
        let freq = 1_000_000_u64.hz();
        let period = freq.period();
        assert_eq!(period, Duration::from_micros(1));
    }

    #[test]
    fn test_frequency_period_roundtrip() {
        // freq → period → 1/period should approximate original freq
        let original = 100.0_f64;
        let freq = original.hz();
        let period = freq.period();
        let recovered = 1.0 / period.as_secs_f64();
        assert!((recovered - original).abs() < 0.001, "roundtrip error: {} vs {}", recovered, original);
    }

    #[test]
    fn test_us_zero_is_zero_duration() {
        let d = 0_u64.us();
        assert_eq!(d, Duration::ZERO);
    }

    #[test]
    fn test_ms_zero_is_zero_duration() {
        let d = 0_u64.ms();
        assert_eq!(d, Duration::ZERO);
    }

    #[test]
    fn test_us_max_does_not_overflow() {
        // u64::MAX microseconds should not overflow Duration
        let d = u64::MAX.us();
        assert!(d > Duration::from_secs(1));
    }

    #[test]
    fn test_ms_large_value() {
        let d = 86_400_000_u64.ms(); // 24 hours in ms
        assert_eq!(d, Duration::from_secs(86400));
    }

    #[test]
    fn test_frequency_1hz_period_is_1s() {
        let freq = 1_u64.hz();
        assert_eq!(freq.period(), Duration::from_secs(1));
    }

    #[test]
    fn test_frequency_value_roundtrip() {
        let freq = 500_u64.hz();
        let hz_value = freq.value();
        assert!((hz_value - 500.0).abs() < 0.001);
    }

    // ========================================================================
    // Boundary value tests
    // ========================================================================

    // ── Zero frequency panics (all types) ──

    #[test]
    #[should_panic(expected = "positive")]
    fn zero_hz_f64_actually_f64_panics() {
        // The existing zero_hz_f64_panics test mistakenly uses 0_u64.
        // This tests the actual f64 code path.
        0.0_f64.hz();
    }

    #[test]
    #[should_panic(expected = "positive")]
    fn zero_hz_i32_panics() {
        0_i32.hz();
    }

    // ── Negative frequency panics (f64) ──

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn negative_hz_f64_panics() {
        (-5.0_f64).hz();
    }

    // ── Special float panics ──

    #[test]
    #[should_panic(expected = "finite")]
    fn neg_inf_hz_panics() {
        f64::NEG_INFINITY.hz();
    }

    // ── Negative i32 duration panics (ns and secs) ──

    #[test]
    #[should_panic(expected = "non-negative")]
    fn negative_i32_ns_panics() {
        (-1_i32).ns();
    }

    #[test]
    #[should_panic(expected = "non-negative")]
    fn negative_i32_secs_panics() {
        (-1_i32).secs();
    }

    // ── Zero i32 durations are valid ──

    #[test]
    fn zero_i32_ns_is_zero() {
        assert_eq!(0_i32.ns(), Duration::ZERO);
    }

    #[test]
    fn zero_i32_us_is_zero() {
        assert_eq!(0_i32.us(), Duration::ZERO);
    }

    #[test]
    fn zero_i32_ms_is_zero() {
        assert_eq!(0_i32.ms(), Duration::ZERO);
    }

    #[test]
    fn zero_i32_secs_is_zero() {
        assert_eq!(0_i32.secs(), Duration::ZERO);
    }

    // ── u64 ns and secs basic coverage ──

    #[test]
    fn ns_from_u64() {
        assert_eq!(1_u64.ns(), Duration::from_nanos(1));
        assert_eq!(0_u64.ns(), Duration::ZERO);
        assert_eq!(1_000_000_000_u64.ns(), Duration::from_secs(1));
    }

    #[test]
    fn secs_from_u64() {
        assert_eq!(1_u64.secs(), Duration::from_secs(1));
        assert_eq!(0_u64.secs(), Duration::ZERO);
        assert_eq!(3600_u64.secs(), Duration::from_secs(3600));
    }

    // ── u64::MAX boundary values ──

    #[test]
    fn u64_max_ns_does_not_overflow() {
        let d = u64::MAX.ns();
        assert!(d > Duration::from_secs(1));
    }

    #[test]
    fn u64_max_ms_does_not_overflow() {
        let d = u64::MAX.ms();
        assert!(d > Duration::from_secs(1));
    }

    #[test]
    fn u64_max_secs_does_not_overflow() {
        let d = u64::MAX.secs();
        assert!(d > Duration::from_secs(1));
    }

    // ── i32::MAX boundary values ──

    #[test]
    fn i32_max_us() {
        let d = i32::MAX.us();
        assert_eq!(d, Duration::from_micros(i32::MAX as u64));
    }

    #[test]
    fn i32_max_ms() {
        let d = i32::MAX.ms();
        assert_eq!(d, Duration::from_millis(i32::MAX as u64));
    }

    #[test]
    fn i32_max_ns() {
        let d = i32::MAX.ns();
        assert_eq!(d, Duration::from_nanos(i32::MAX as u64));
    }

    #[test]
    fn i32_max_secs() {
        let d = i32::MAX.secs();
        assert_eq!(d, Duration::from_secs(i32::MAX as u64));
    }

    #[test]
    fn i32_max_hz() {
        let f = i32::MAX.hz();
        assert_eq!(f.value(), i32::MAX as f64);
    }

    // ── f64 duration helpers (ns, secs) ──

    #[test]
    fn ns_from_f64() {
        let d = 1500.0_f64.ns();
        // 1500 ns = 1.5us = 0.0000015s
        assert_eq!(d, Duration::from_secs_f64(1500.0 / 1_000_000_000.0));
    }

    #[test]
    fn secs_from_f64() {
        let d = 1.5_f64.secs();
        assert_eq!(d, Duration::from_secs_f64(1.5));
    }

    #[test]
    fn f64_zero_us_is_zero() {
        assert_eq!(0.0_f64.us(), Duration::ZERO);
    }

    #[test]
    fn f64_zero_ms_is_zero() {
        assert_eq!(0.0_f64.ms(), Duration::ZERO);
    }

    #[test]
    fn f64_zero_ns_is_zero() {
        assert_eq!(0.0_f64.ns(), Duration::ZERO);
    }

    #[test]
    fn f64_zero_secs_is_zero() {
        assert_eq!(0.0_f64.secs(), Duration::ZERO);
    }

    // ── Very tiny f64 frequency ──

    #[test]
    fn f64_epsilon_hz() {
        let freq = f64::EPSILON.hz();
        assert_eq!(freq.value(), f64::EPSILON);
        // Period should be finite and very large
        let period = freq.period();
        assert!(period.as_secs() > 0);
    }

    #[test]
    fn f64_very_small_hz_budget_and_deadline() {
        // 0.01 Hz = 100 second period
        let freq = 0.01_f64.hz();
        let budget = freq.budget_default();
        let deadline = freq.deadline_default();
        assert_eq!(budget, Duration::from_secs(80));     // 80% of 100s
        assert_eq!(deadline.as_millis(), 95_000);         // 95% of 100s
    }

    // ── Very large frequency budget/deadline ──

    #[test]
    fn very_large_frequency_budget_and_deadline() {
        // 1 MHz = 1us period
        let freq = 1_000_000_u64.hz();
        let budget = freq.budget_default();
        let deadline = freq.deadline_default();
        // budget = 0.8us = 800ns
        assert_eq!(budget, Duration::from_nanos(800));
        // deadline = 0.95us = 950ns
        assert_eq!(deadline, Duration::from_nanos(950));
    }

    // ── Frequency Display trait ──

    #[test]
    fn frequency_display_integer() {
        let freq = 100_u64.hz();
        let s = format!("{freq}");
        assert_eq!(s, "100 Hz");
    }

    #[test]
    fn frequency_display_fractional() {
        let freq = 33.33_f64.hz();
        let s = format!("{freq}");
        assert_eq!(s, "33.33 Hz");
    }

    #[test]
    fn frequency_display_small() {
        let freq = 0.5_f64.hz();
        let s = format!("{freq}");
        assert_eq!(s, "0.5 Hz");
    }

    // ── Frequency Debug trait ──

    #[test]
    fn frequency_debug() {
        let freq = 100_u64.hz();
        let s = format!("{freq:?}");
        assert!(s.contains("Frequency"));
        assert!(s.contains("100"));
    }

    // ── Frequency Clone and Copy ──

    #[test]
    fn frequency_clone() {
        let freq = 50_u64.hz();
        let cloned = freq.clone();
        assert_eq!(freq, cloned);
    }

    #[test]
    fn frequency_copy() {
        let freq = 50_u64.hz();
        let copied = freq; // Copy
        let _ = freq; // original still usable if Copy
        assert_eq!(copied.value(), 50.0);
    }

    // ── Frequency PartialEq ──

    #[test]
    fn frequency_eq_same_value() {
        let a = 100_u64.hz();
        let b = 100.0_f64.hz();
        assert_eq!(a, b);
    }

    #[test]
    fn frequency_ne_different_value() {
        let a = 100_u64.hz();
        let b = 200_u64.hz();
        assert_ne!(a, b);
    }

    // ── Period roundtrip accuracy at various scales ──

    #[test]
    fn period_roundtrip_low_frequency() {
        let original = 0.1_f64;
        let freq = original.hz();
        let recovered = 1.0 / freq.period().as_secs_f64();
        assert!((recovered - original).abs() < 1e-9,
            "low freq roundtrip: {} vs {}", recovered, original);
    }

    #[test]
    fn period_roundtrip_high_frequency() {
        let original = 10_000.0_f64;
        let freq = original.hz();
        let recovered = 1.0 / freq.period().as_secs_f64();
        assert!((recovered - original).abs() < 0.01,
            "high freq roundtrip: {} vs {}", recovered, original);
    }

    // ── i32 secs and ns convenience ──

    #[test]
    fn ns_from_i32() {
        assert_eq!(100_i32.ns(), Duration::from_nanos(100));
    }

    #[test]
    fn secs_from_i32() {
        assert_eq!(10_i32.secs(), Duration::from_secs(10));
    }

    // ── Boundary: i32::MIN panics ──

    #[test]
    #[should_panic(expected = "non-negative")]
    fn i32_min_us_panics() {
        i32::MIN.us();
    }

    #[test]
    #[should_panic(expected = "non-negative")]
    fn i32_min_ms_panics() {
        i32::MIN.ms();
    }

    #[test]
    #[should_panic(expected = "non-negative")]
    fn i32_min_ns_panics() {
        i32::MIN.ns();
    }

    #[test]
    #[should_panic(expected = "non-negative")]
    fn i32_min_secs_panics() {
        i32::MIN.secs();
    }

    #[test]
    #[should_panic(expected = "positive")]
    fn i32_min_hz_panics() {
        i32::MIN.hz();
    }

    // ── Boundary: f64::MIN_POSITIVE (smallest positive f64) ──

    #[test]
    fn f64_min_positive_hz() {
        let freq = f64::MIN_POSITIVE.hz();
        assert_eq!(freq.value(), f64::MIN_POSITIVE);
    }

    // ── Boundary: largest finite f64 ──

    #[test]
    fn f64_max_hz() {
        let freq = f64::MAX.hz();
        assert_eq!(freq.value(), f64::MAX);
        // Period should be essentially zero
        let period = freq.period();
        assert_eq!(period, Duration::ZERO);
    }
}
