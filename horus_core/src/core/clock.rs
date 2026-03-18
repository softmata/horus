//! Clock abstraction for the HORUS scheduler.
//!
//! Provides a unified time source that the scheduler and all timing operations
//! use. Users never interact with these types directly — they call `horus::now()`,
//! `horus::dt()`, etc., which read from the ambient tick context.
//!
//! # Clock Backends
//!
//! | Backend | Mode | Behavior |
//! |---------|------|----------|
//! | `WallClock` | Normal (default) | Passthrough to `Instant::now()`. Zero overhead. |
//! | `SimClock` | `.deterministic(true)` | Virtual time, advances by exact dt per tick. |
//! | `ReplayClock` | `.replay_from(path)` | Steps through recorded timestamps. |

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

// ─── ClockInstant ────────────────────────────────────────────────────────────

/// Unified time instant that works across all clock backends.
///
/// Wraps a monotonic nanosecond counter. In normal mode this maps from
/// `Instant`; in deterministic mode it's pure virtual time.
///
/// Supports duration arithmetic via `elapsed_since()` and `Sub`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[doc(hidden)]
pub struct ClockInstant {
    /// Monotonic nanoseconds from an arbitrary epoch.
    nanos: u64,
}

impl ClockInstant {
    /// Create from raw nanoseconds.
    #[doc(hidden)]
    #[inline]
    pub fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    /// Raw nanosecond value.
    #[doc(hidden)]
    #[inline]
    pub fn as_nanos(&self) -> u64 {
        self.nanos
    }

    /// Duration elapsed since an earlier instant.
    ///
    /// Returns `Duration::ZERO` if `earlier` is after `self` (saturating).
    #[inline]
    pub fn elapsed_since(&self, earlier: ClockInstant) -> Duration {
        Duration::from_nanos(self.nanos.saturating_sub(earlier.nanos))
    }
}

impl std::ops::Sub for ClockInstant {
    type Output = Duration;

    #[inline]
    fn sub(self, rhs: Self) -> Duration {
        self.elapsed_since(rhs)
    }
}

impl Default for ClockInstant {
    fn default() -> Self {
        Self { nanos: 0 }
    }
}

// ─── Clock Trait ─────────────────────────────────────────────────────────────

/// Clock source for the scheduler and all timing operations.
///
/// The scheduler selects the implementation based on mode:
/// - Normal: `WallClock` (passthrough to `Instant::now()`)
/// - Deterministic: `SimClock` (fixed-step virtual time)
/// - Replay: `ReplayClock` (reads recorded timestamps)
///
/// Users never interact with this trait. They call `horus::now()` etc.
#[doc(hidden)]
pub trait Clock: Send + Sync {
    /// Current time instant.
    fn now(&self) -> ClockInstant;

    /// Advance the clock by one tick's dt.
    ///
    /// - `WallClock`: no-op (time advances on its own).
    /// - `SimClock`: increments virtual time by exactly `dt`.
    /// - `ReplayClock`: steps to the next recorded timestamp.
    fn advance(&self, dt: Duration);

    /// Reset to initial state. Used by replay `seek(0)`.
    fn reset(&self);

    /// Total time elapsed since clock construction / scheduler start.
    fn elapsed(&self) -> Duration;
}

// ─── WallClock ───────────────────────────────────────────────────────────────

/// Real wall clock. Default in normal mode. Zero overhead.
///
/// Delegates to `std::time::Instant` — behavior is identical to calling
/// `Instant::now()` directly. `advance()` and `reset()` are no-ops because
/// wall time advances on its own.
pub(crate) struct WallClock {
    /// Captured at construction. All measurements are relative to this.
    epoch: Instant,
}

impl WallClock {
    pub(crate) fn new() -> Self {
        Self {
            epoch: Instant::now(),
        }
    }
}

impl Clock for WallClock {
    #[inline]
    fn now(&self) -> ClockInstant {
        ClockInstant::from_nanos(self.epoch.elapsed().as_nanos() as u64)
    }

    #[inline]
    fn advance(&self, _dt: Duration) {
        // No-op — wall time advances on its own.
    }

    #[inline]
    fn reset(&self) {
        // No-op — can't reset wall time.
    }

    #[inline]
    fn elapsed(&self) -> Duration {
        self.epoch.elapsed()
    }
}

// SAFETY: WallClock contains only Instant which is Send + Sync.
unsafe impl Send for WallClock {}
unsafe impl Sync for WallClock {}

// ─── SimClock ────────────────────────────────────────────────────────────────

/// Virtual clock for deterministic mode.
///
/// Time only advances when the scheduler calls `advance(dt)`. Two runs
/// with the same tick sequence produce identical clock values regardless
/// of CPU speed or system load.
///
/// Uses `AtomicU64` for lock-free reads from executor threads.
#[doc(hidden)]
pub struct SimClock {
    /// Current virtual time in nanoseconds.
    nanos: AtomicU64,
}

impl SimClock {
    #[doc(hidden)]
    pub fn new() -> Self {
        Self {
            nanos: AtomicU64::new(0),
        }
    }
}

impl Clock for SimClock {
    #[inline]
    fn now(&self) -> ClockInstant {
        ClockInstant::from_nanos(self.nanos.load(Ordering::Acquire))
    }

    #[inline]
    fn advance(&self, dt: Duration) {
        self.nanos
            .fetch_add(dt.as_nanos() as u64, Ordering::Release);
    }

    #[inline]
    fn reset(&self) {
        self.nanos.store(0, Ordering::Release);
    }

    #[inline]
    fn elapsed(&self) -> Duration {
        Duration::from_nanos(self.nanos.load(Ordering::Acquire))
    }
}

// ─── ReplayClock ─────────────────────────────────────────────────────────────

/// Replay clock. Steps through recorded timestamps from a session.
///
/// Each `advance()` steps to the next recorded timestamp. `now()` returns
/// the current position's timestamp. Used during `Scheduler::replay_from()`.
#[doc(hidden)]
pub struct ReplayClock {
    /// Recorded timestamps in nanoseconds, sorted.
    timestamps: Vec<u64>,
    /// Current position in the timestamps vector.
    index: AtomicU64,
}

impl ReplayClock {
    /// Create from a list of nanosecond timestamps.
    #[doc(hidden)]
    pub fn new(timestamps: Vec<u64>) -> Self {
        Self {
            timestamps,
            index: AtomicU64::new(0),
        }
    }

    /// Number of recorded timestamps.
    pub fn len(&self) -> usize {
        self.timestamps.len()
    }

    /// Whether the clock has no timestamps.
    pub fn is_empty(&self) -> bool {
        self.timestamps.is_empty()
    }

    /// Current index position.
    pub fn current_index(&self) -> u64 {
        self.index.load(Ordering::Acquire)
    }
}

impl Clock for ReplayClock {
    #[inline]
    fn now(&self) -> ClockInstant {
        let idx = self.index.load(Ordering::Acquire) as usize;
        let nanos = self
            .timestamps
            .get(idx)
            .copied()
            .or_else(|| self.timestamps.last().copied())
            .unwrap_or(0);
        ClockInstant::from_nanos(nanos)
    }

    #[inline]
    fn advance(&self, _dt: Duration) {
        let current = self.index.load(Ordering::Acquire);
        let max = self.timestamps.len().saturating_sub(1) as u64;
        if current < max {
            self.index.store(current + 1, Ordering::Release);
        }
    }

    fn reset(&self) {
        self.index.store(0, Ordering::Release);
    }

    fn elapsed(&self) -> Duration {
        Duration::from_nanos(self.now().nanos)
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;

    // ── ClockInstant ──

    #[test]
    fn clock_instant_default_is_zero() {
        let ci = ClockInstant::default();
        assert_eq!(ci.as_nanos(), 0);
    }

    #[test]
    fn clock_instant_from_nanos() {
        let ci = ClockInstant::from_nanos(1_000_000);
        assert_eq!(ci.as_nanos(), 1_000_000);
    }

    #[test]
    fn clock_instant_elapsed_since_normal() {
        let earlier = ClockInstant::from_nanos(1_000_000);
        let later = ClockInstant::from_nanos(3_000_000);
        assert_eq!(later.elapsed_since(earlier), Duration::from_micros(2000));
    }

    #[test]
    fn clock_instant_elapsed_since_zero() {
        let ci = ClockInstant::from_nanos(5_000);
        assert_eq!(ci.elapsed_since(ci), Duration::ZERO);
    }

    #[test]
    fn clock_instant_elapsed_since_reversed_saturates() {
        let earlier = ClockInstant::from_nanos(5_000_000);
        let later = ClockInstant::from_nanos(1_000_000);
        // reversed: later.elapsed_since(earlier) should saturate to 0
        assert_eq!(later.elapsed_since(earlier), Duration::ZERO);
    }

    #[test]
    fn clock_instant_sub_operator() {
        let a = ClockInstant::from_nanos(10_000_000);
        let b = ClockInstant::from_nanos(3_000_000);
        assert_eq!(a - b, Duration::from_nanos(7_000_000));
    }

    #[test]
    fn clock_instant_sub_reversed_saturates() {
        let a = ClockInstant::from_nanos(1_000);
        let b = ClockInstant::from_nanos(5_000);
        assert_eq!(a - b, Duration::ZERO);
    }

    #[test]
    fn clock_instant_ordering() {
        let a = ClockInstant::from_nanos(100);
        let b = ClockInstant::from_nanos(200);
        assert!(a < b);
        assert!(b > a);
        assert_eq!(a, ClockInstant::from_nanos(100));
    }

    // ── WallClock ──

    #[test]
    fn wall_clock_now_is_positive() {
        let clock = WallClock::new();
        std::thread::sleep(Duration::from_millis(1));
        let now = clock.now();
        assert!(now.as_nanos() > 0);
    }

    #[test]
    fn wall_clock_monotonic() {
        let clock = WallClock::new();
        let a = clock.now();
        std::thread::sleep(Duration::from_millis(1));
        let b = clock.now();
        assert!(b > a);
    }

    #[test]
    fn wall_clock_elapsed_matches_now() {
        let clock = WallClock::new();
        std::thread::sleep(Duration::from_millis(5));
        let elapsed = clock.elapsed();
        let now_nanos = clock.now().as_nanos();
        // They should be very close (within 100μs)
        let diff = if now_nanos > elapsed.as_nanos() as u64 {
            now_nanos - elapsed.as_nanos() as u64
        } else {
            elapsed.as_nanos() as u64 - now_nanos
        };
        assert!(diff < 100_000, "now and elapsed differ by {}ns", diff);
    }

    #[test]
    fn wall_clock_advance_is_noop() {
        let clock = WallClock::new();
        let before = clock.now();
        clock.advance(Duration::from_secs(1000));
        let after = clock.now();
        // advance should not jump forward by 1000s
        let diff = after.elapsed_since(before);
        assert!(diff < Duration::from_millis(10));
    }

    // ── SimClock ──

    #[test]
    fn sim_clock_starts_at_zero() {
        let clock = SimClock::new();
        assert_eq!(clock.now().as_nanos(), 0);
        assert_eq!(clock.elapsed(), Duration::ZERO);
    }

    #[test]
    fn sim_clock_advance_exact() {
        let clock = SimClock::new();
        clock.advance(1_u64.ms());
        assert_eq!(clock.now().as_nanos(), 1_000_000);
        assert_eq!(clock.elapsed(), 1_u64.ms());
    }

    #[test]
    fn sim_clock_advance_accumulates() {
        let clock = SimClock::new();
        clock.advance(100_u64.us());
        clock.advance(100_u64.us());
        clock.advance(100_u64.us());
        assert_eq!(clock.now().as_nanos(), 300_000);
        assert_eq!(clock.elapsed(), 300_u64.us());
    }

    #[test]
    fn sim_clock_advance_large_dt() {
        let clock = SimClock::new();
        clock.advance(Duration::from_secs(3600)); // 1 hour
        assert_eq!(clock.elapsed(), Duration::from_secs(3600));
    }

    #[test]
    fn sim_clock_reset() {
        let clock = SimClock::new();
        clock.advance(5_u64.ms());
        assert_eq!(clock.elapsed(), 5_u64.ms());
        clock.reset();
        assert_eq!(clock.now().as_nanos(), 0);
        assert_eq!(clock.elapsed(), Duration::ZERO);
    }

    #[test]
    fn sim_clock_deterministic() {
        // Two clocks with same advance sequence must produce identical values
        let a = SimClock::new();
        let b = SimClock::new();

        let advances = [100_u64.us(), 200_u64.us(), 50_u64.us(), 1_u64.ms()];

        for dt in &advances {
            a.advance(*dt);
            b.advance(*dt);
        }

        assert_eq!(a.now(), b.now());
        assert_eq!(a.elapsed(), b.elapsed());
    }

    #[test]
    fn sim_clock_thread_safe() {
        use std::sync::Arc;

        let clock = Arc::new(SimClock::new());
        let clock2 = clock.clone();

        let writer = std::thread::spawn(move || {
            for _ in 0..1000 {
                clock2.advance(1_u64.us());
            }
        });

        // Reader: just keep reading, should never panic
        let mut readings = Vec::new();
        for _ in 0..100 {
            readings.push(clock.now().as_nanos());
        }

        writer.join().unwrap();

        // Final value should be exactly 1000μs = 1_000_000ns
        assert_eq!(clock.now().as_nanos(), 1_000_000);

        // Readings should be monotonically non-decreasing
        for window in readings.windows(2) {
            assert!(window[1] >= window[0], "Non-monotonic reading");
        }
    }

    // ── Clock trait usage ──

    #[test]
    fn clock_trait_wall_clock() {
        let clock: Box<dyn Clock> = Box::new(WallClock::new());
        std::thread::sleep(Duration::from_millis(1));
        assert!(clock.now().as_nanos() > 0);
    }

    #[test]
    fn clock_trait_sim_clock() {
        let clock: Box<dyn Clock> = Box::new(SimClock::new());
        assert_eq!(clock.now().as_nanos(), 0);
        clock.advance(500_u64.us());
        assert_eq!(clock.now().as_nanos(), 500_000);
    }

    #[test]
    fn clock_trait_timing_measurement() {
        let clock: Box<dyn Clock> = Box::new(SimClock::new());
        let start = clock.now();
        clock.advance(250_u64.us());
        let elapsed = clock.now().elapsed_since(start);
        assert_eq!(elapsed, 250_u64.us());
    }

    // ── ReplayClock ──

    #[test]
    fn replay_clock_empty() {
        let clock = ReplayClock::new(vec![]);
        assert_eq!(clock.now().as_nanos(), 0);
        assert!(clock.is_empty());
        clock.advance(1_u64.ms()); // should not panic
        assert_eq!(clock.now().as_nanos(), 0);
    }

    #[test]
    fn replay_clock_single_timestamp() {
        let clock = ReplayClock::new(vec![5_000_000]);
        assert_eq!(clock.now().as_nanos(), 5_000_000);
        clock.advance(1_u64.ms());
        assert_eq!(clock.now().as_nanos(), 5_000_000); // stays at last
    }

    #[test]
    fn replay_clock_steps_through_timestamps() {
        let timestamps = vec![1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000];
        let clock = ReplayClock::new(timestamps);

        assert_eq!(clock.now().as_nanos(), 1_000_000);
        clock.advance(Duration::ZERO);
        assert_eq!(clock.now().as_nanos(), 2_000_000);
        clock.advance(Duration::ZERO);
        assert_eq!(clock.now().as_nanos(), 3_000_000);
        clock.advance(Duration::ZERO);
        assert_eq!(clock.now().as_nanos(), 4_000_000);
        clock.advance(Duration::ZERO);
        assert_eq!(clock.now().as_nanos(), 5_000_000);
        // Beyond end: stays at last
        clock.advance(Duration::ZERO);
        assert_eq!(clock.now().as_nanos(), 5_000_000);
    }

    #[test]
    fn replay_clock_reset() {
        let timestamps = vec![1_000_000, 2_000_000, 3_000_000];
        let clock = ReplayClock::new(timestamps);

        clock.advance(Duration::ZERO);
        clock.advance(Duration::ZERO);
        assert_eq!(clock.now().as_nanos(), 3_000_000);

        clock.reset();
        assert_eq!(clock.now().as_nanos(), 1_000_000);
    }

    #[test]
    fn replay_clock_elapsed() {
        let timestamps = vec![0, 10_000_000, 20_000_000];
        let clock = ReplayClock::new(timestamps);

        assert_eq!(clock.elapsed(), Duration::ZERO);
        clock.advance(Duration::ZERO);
        assert_eq!(clock.elapsed(), 10_u64.ms());
        clock.advance(Duration::ZERO);
        assert_eq!(clock.elapsed(), 20_u64.ms());
    }

    #[test]
    fn replay_clock_via_trait() {
        let clock: Box<dyn Clock> = Box::new(ReplayClock::new(vec![100_000, 200_000, 300_000]));
        assert_eq!(clock.now().as_nanos(), 100_000);
        clock.advance(Duration::ZERO);
        assert_eq!(clock.now().as_nanos(), 200_000);
    }
}
