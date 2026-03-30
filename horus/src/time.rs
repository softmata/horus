//! Framework time API for HORUS nodes.
//!
//! These functions are THE standard way to get time, timestep, and random
//! numbers in horus — same pattern as `hlog!()` for logging. The scheduler
//! sets the ambient context before each `tick()` call; these functions read
//! from it.
//!
//! | Function | Normal mode | Deterministic mode | Outside tick() |
//! |----------|-------------|-------------------|----------------|
//! | `now()` | Wall clock | Virtual SimClock | Wall clock (fallback) |
//! | `dt()` | Real elapsed | Fixed 1/rate | `Duration::ZERO` |
//! | `elapsed()` | Wall elapsed | Accumulated virtual | `Duration::ZERO` |
//! | `tick()` | Tick number | Tick number | 0 |
//! | `rng()` | System entropy | Tick-seeded | System entropy |
//! | `budget_remaining()` | Time left in budget | Time left (SimClock) | `Duration::MAX` |
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! struct Controller { position: f64, velocity: f64 }
//!
//! impl Node for Controller {
//!     fn tick(&mut self) {
//!         let dt = horus::dt();
//!         self.position += self.velocity * dt.as_secs_f64();
//!         hlog!(debug, "pos={:.3} at t={:?}", self.position, horus::elapsed());
//!     }
//! }
//! ```

use std::time::Duration;

use horus_core::core::tick_context;

/// Opaque timestamp from `horus::now()`. Supports duration arithmetic.
///
/// In normal mode, this has the same precision as `Instant`.
/// In deterministic mode, this has nanosecond precision from virtual clock.
///
/// # Example
///
/// ```rust,ignore
/// let start = horus::now();
/// do_work();
/// let elapsed = horus::since(start);
/// hlog!(debug, "work took {:?}", elapsed);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TimeStamp(pub(crate) u64);

impl TimeStamp {
    /// Duration elapsed since this timestamp (using the framework clock).
    pub fn elapsed(&self) -> Duration {
        let current = now();
        Duration::from_nanos(current.0.saturating_sub(self.0))
    }

    /// Raw nanosecond value (for serialization/debugging).
    pub fn as_nanos(&self) -> u64 {
        self.0
    }

    /// Create from raw nanoseconds (for deserialization).
    pub fn from_nanos(nanos: u64) -> Self {
        Self(nanos)
    }
}

impl std::ops::Sub for TimeStamp {
    type Output = Duration;

    #[inline]
    fn sub(self, rhs: Self) -> Duration {
        Duration::from_nanos(self.0.saturating_sub(rhs.0))
    }
}

impl std::fmt::Display for TimeStamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let secs = self.0 as f64 / 1_000_000_000.0;
        write!(f, "{:.6}s", secs)
    }
}

// ─── Public Functions ────────────────────────────────────────────────────────

/// Current time. THE way to get time in horus.
///
/// - Normal mode: wall clock (equivalent to `Instant::now()`)
/// - Deterministic mode: virtual SimClock time
/// - Replay mode: recorded timestamp
/// - Outside `tick()`: wall clock fallback
///
/// # Example
///
/// ```rust,ignore
/// fn tick(&mut self) {
///     let start = horus::now();
///     self.do_work();
///     hlog!(debug, "work took {:?}", horus::since(start));
/// }
/// ```
#[inline]
pub fn now() -> TimeStamp {
    TimeStamp(tick_context::ctx_now().as_nanos())
}

/// Duration since a previous `horus::now()` timestamp.
///
/// # Example
///
/// ```rust,ignore
/// let start = horus::now();
/// expensive_computation();
/// let elapsed = horus::since(start);
/// ```
#[inline]
pub fn since(earlier: TimeStamp) -> Duration {
    let current = now();
    Duration::from_nanos(current.0.saturating_sub(earlier.0))
}

/// Timestep for this tick. THE way to get dt in horus.
///
/// - Normal mode: actual elapsed since last tick
/// - Deterministic mode: fixed `1/rate` (e.g., 10ms for 100Hz)
/// - Outside `tick()`: `Duration::ZERO`
///
/// # Example
///
/// ```rust,ignore
/// fn tick(&mut self) {
///     let dt = horus::dt();
///     self.position += self.velocity * dt.as_secs_f64();
/// }
/// ```
#[inline]
pub fn dt() -> Duration {
    tick_context::ctx_dt()
}

/// Time elapsed since scheduler start.
///
/// - Normal mode: wall-clock elapsed
/// - Deterministic mode: accumulated virtual dt
/// - Outside `tick()`: `Duration::ZERO`
#[inline]
pub fn elapsed() -> Duration {
    tick_context::ctx_elapsed()
}

/// Current tick number.
///
/// - Returns 0 outside `tick()`
#[inline]
pub fn tick() -> u64 {
    tick_context::ctx_tick()
}

/// Time remaining in this tick's budget.
///
/// - Returns `Duration::MAX` if no budget is configured or outside `tick()`
///
/// # Example — Anytime algorithm
///
/// ```rust,ignore
/// fn tick(&mut self) {
///     loop {
///         self.improve_plan();
///         if horus::budget_remaining() < 50_u64.us() {
///             break; // stop before budget runs out
///         }
///     }
///     self.path_topic.send(self.plan.clone());
/// }
/// ```
#[inline]
pub fn budget_remaining() -> Duration {
    tick_context::ctx_budget_remaining()
}

/// Access the deterministic RNG. THE way to get random numbers in horus.
///
/// - Normal mode: system entropy
/// - Deterministic mode: seeded from tick number + node name (reproducible)
/// - Outside `tick()`: system entropy fallback
///
/// Returns a value via closure to avoid exposing the RNG type directly.
///
/// # Example
///
/// ```rust,ignore
/// fn tick(&mut self) {
///     let noise: f64 = horus::rng(|r| r.gen_range(-0.01..0.01));
///     self.measurement += noise;
/// }
/// ```
#[inline]
pub fn rng<R>(f: impl FnOnce(&mut rand::rngs::SmallRng) -> R) -> R {
    tick_context::ctx_with_rng(f)
}

/// Get the scheduler-managed CUDA stream for the current GPU node's tick.
///
/// Returns `None` for non-GPU nodes (BestEffort, Compute, RT, etc.).
/// Returns `Some(raw_stream_ptr)` for GPU nodes during their tick().
///
/// The returned pointer is a raw `CUstream` handle that can be passed to
/// CUDA kernel launch functions. It is valid only for the duration of tick().
///
/// # Example
///
/// ```rust,ignore
/// fn tick(&mut self) {
///     if let Some(stream) = horus::gpu_stream() {
///         // Launch CUDA kernel on the scheduler-managed stream
///         my_cuda_kernel(data_ptr, size, stream);
///     }
/// }
/// ```
#[inline]
pub fn gpu_stream() -> Option<*const std::ffi::c_void> {
    let ptr = tick_context::ctx_gpu_stream();
    if ptr.is_null() {
        None
    } else {
        Some(ptr)
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use horus_core::core::clock::{Clock, ClockInstant, SimClock};
    use horus_core::core::duration_ext::DurationExt;
    use horus_core::core::tick_context::{clear_tick_context, set_tick_context};

    #[test]
    fn now_outside_tick_returns_wallclock() {
        clear_tick_context();
        let a = now();
        std::thread::sleep(Duration::from_millis(1));
        let b = now();
        assert!(b > a);
    }

    #[test]
    fn now_inside_tick_returns_simclock() {
        let clock = SimClock::new();
        clock.advance(42_u64.ms());
        let start = clock.now();
        set_tick_context("test", 1, &clock, 1_u64.ms(), 42_u64.ms(), start, None);

        let ts = now();
        assert_eq!(ts.as_nanos(), 42_000_000);

        clear_tick_context();
    }

    #[test]
    fn dt_inside_tick() {
        let clock = SimClock::new();
        let start = clock.now();
        set_tick_context("test", 1, &clock, 10_u64.ms(), 10_u64.ms(), start, None);

        assert_eq!(dt(), 10_u64.ms());

        clear_tick_context();
    }

    #[test]
    fn elapsed_inside_tick() {
        let clock = SimClock::new();
        let start = clock.now();
        set_tick_context("test", 50, &clock, 1_u64.ms(), 50_u64.ms(), start, None);

        assert_eq!(elapsed(), 50_u64.ms());

        clear_tick_context();
    }

    #[test]
    fn tick_number_inside_tick() {
        let clock = SimClock::new();
        let start = clock.now();
        set_tick_context("test", 99, &clock, 1_u64.ms(), 99_u64.ms(), start, None);

        assert_eq!(tick(), 99);

        clear_tick_context();
    }

    #[test]
    fn since_measures_duration() {
        let clock = SimClock::new();
        let start_ci = clock.now();
        set_tick_context("test", 1, &clock, 1_u64.ms(), 0_u64.ms(), start_ci, None);

        let start = now();
        clock.advance(500_u64.us());
        let elapsed = since(start);
        assert_eq!(elapsed, 500_u64.us());

        clear_tick_context();
    }

    #[test]
    fn timestamp_sub() {
        let a = TimeStamp(10_000_000);
        let b = TimeStamp(3_000_000);
        assert_eq!(a - b, Duration::from_nanos(7_000_000));
    }

    #[test]
    fn timestamp_sub_reversed_saturates() {
        let a = TimeStamp(1_000);
        let b = TimeStamp(5_000);
        assert_eq!(a - b, Duration::ZERO);
    }

    #[test]
    fn timestamp_elapsed() {
        let clock = SimClock::new();
        clock.advance(10_u64.ms());
        let start_ci = clock.now();
        set_tick_context("test", 1, &clock, 1_u64.ms(), 10_u64.ms(), start_ci, None);

        let ts = now(); // 10ms
        clock.advance(3_u64.ms());
        let elapsed = ts.elapsed(); // should be 3ms
        assert_eq!(elapsed, 3_u64.ms());

        clear_tick_context();
    }

    #[test]
    fn timestamp_display() {
        let ts = TimeStamp(1_500_000_000); // 1.5 seconds
        assert_eq!(format!("{}", ts), "1.500000s");
    }

    #[test]
    fn budget_remaining_inside_tick() {
        let clock = SimClock::new();
        let start = clock.now();
        set_tick_context(
            "test",
            1,
            &clock,
            1_u64.ms(),
            1_u64.ms(),
            start,
            Some(800_u64.us()),
        );

        assert_eq!(budget_remaining(), 800_u64.us());
        clock.advance(300_u64.us());
        assert_eq!(budget_remaining(), 500_u64.us());

        clear_tick_context();
    }

    #[test]
    fn rng_deterministic() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context("ctrl", 42, &clock, 1_u64.ms(), 42_u64.ms(), start, None);
        let v1 = rng(|r| {
            use rand::Rng;
            r.gen::<u64>()
        });
        clear_tick_context();

        set_tick_context("ctrl", 42, &clock, 1_u64.ms(), 42_u64.ms(), start, None);
        let v2 = rng(|r| {
            use rand::Rng;
            r.gen::<u64>()
        });
        clear_tick_context();

        assert_eq!(v1, v2);
    }

    #[test]
    fn rng_outside_tick_uses_entropy() {
        clear_tick_context();
        // Should not panic
        let _ = rng(|r| {
            use rand::Rng;
            r.gen::<f64>()
        });
    }
}
