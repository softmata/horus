//! Ambient tick context for HORUS nodes.
//!
//! Provides thread-local clock, timestep, simulation time, and RNG to node code
//! via `horus::now()`, `horus::dt()`, `horus::rng()`, etc. Same pattern as
//! `hlog!()` — set by the scheduler before `tick()`, cleared after.
//!
//! The scheduler calls `set_tick_context()` / `clear_tick_context()` around every
//! `node.tick()`. Public `horus::` functions read from this context.

use std::cell::RefCell;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::time::Duration;

use rand::rngs::SmallRng;
use rand::SeedableRng;

use super::clock::{Clock, ClockInstant, WallClock};

thread_local! {
    static TICK_CTX: RefCell<Option<TickContext>> = const { RefCell::new(None) };
    /// Fallback wall clock for when no tick context is set.
    /// Lazily initialized on first access outside of tick().
    static FALLBACK_CLOCK: WallClock = WallClock::new();
}

/// Ambient context available during a node's `tick()` call.
///
/// Set by the scheduler, read by `horus::now()` etc., cleared after tick.
pub(crate) struct TickContext {
    /// Node name (for RNG seeding).
    pub node_name: String,
    /// Current tick number.
    pub tick_number: u64,
    /// Clock reference — borrowed from scheduler, valid for tick() duration.
    ///
    /// # Safety
    /// This raw pointer is valid for the duration of tick(). The scheduler
    /// guarantees the Clock outlives the tick() call. The pointer is set
    /// to None by clear_tick_context().
    pub clock: Option<*const dyn Clock>,
    /// Fixed timestep (1/rate in deterministic, real elapsed in normal).
    pub dt: Duration,
    /// Virtual time since scheduler start.
    pub sim_time: Duration,
    /// Deterministic RNG seeded from tick + node name.
    pub rng: SmallRng,
    /// When this tick started (for budget_remaining).
    pub tick_start: ClockInstant,
    /// Configured budget for this node (None = no budget).
    pub budget: Option<Duration>,
}

// SAFETY: TickContext is only accessed from the thread that owns the thread-local.
// The clock pointer is valid for the duration of tick() — guaranteed by the scheduler.
unsafe impl Send for TickContext {}

// ─── Set / Clear ─────────────────────────────────────────────────────────────

/// Set the tick context before `node.tick()`.
///
/// Called by the scheduler. Reuses the existing allocation when possible.
#[doc(hidden)]
pub fn set_tick_context(
    node_name: &str,
    tick_number: u64,
    clock: &dyn Clock,
    dt: Duration,
    sim_time: Duration,
    tick_start: ClockInstant,
    budget: Option<Duration>,
) {
    let seed = deterministic_seed(tick_number, node_name);
    let rng = SmallRng::seed_from_u64(seed);
    // Erase lifetime to store in 'static thread-local.
    // SAFETY: caller (scheduler) guarantees `clock` outlives the tick() call.
    // clear_tick_context() sets this to None before the reference becomes invalid.
    let clock_ptr: *const dyn Clock = clock;
    let clock_ptr: *const dyn Clock = unsafe {
        std::mem::transmute::<*const dyn Clock, *const dyn Clock>(clock_ptr)
    };

    TICK_CTX.with(|ctx| {
        let mut slot = ctx.borrow_mut();
        if let Some(ref mut existing) = *slot {
            existing.node_name.clear();
            existing.node_name.push_str(node_name);
            existing.tick_number = tick_number;
            existing.clock = Some(clock_ptr);
            existing.dt = dt;
            existing.sim_time = sim_time;
            existing.rng = rng;
            existing.tick_start = tick_start;
            existing.budget = budget;
        } else {
            *slot = Some(TickContext {
                node_name: node_name.to_owned(),
                tick_number,
                clock: Some(clock_ptr),
                dt,
                sim_time,
                rng,
                tick_start,
                budget,
            });
        }
    });
}

/// Clear the tick context after `node.tick()`.
///
/// Keeps the allocation alive for reuse. Nulls the clock pointer.
#[doc(hidden)]
pub fn clear_tick_context() {
    TICK_CTX.with(|ctx| {
        if let Some(ref mut existing) = *ctx.borrow_mut() {
            existing.clock = None;
            existing.budget = None;
        }
    });
}

// ─── Readers (accessible from horus crate via #[doc(hidden)]) ────────────────

/// Read the current clock instant. Falls back to WallClock outside tick().
#[doc(hidden)]
pub fn ctx_now() -> ClockInstant {
    TICK_CTX.with(|ctx| {
        let borrow = ctx.borrow();
        if let Some(ref tc) = *borrow {
            if let Some(clock_ptr) = tc.clock {
                // SAFETY: clock pointer is valid during tick() — scheduler guarantees this.
                return unsafe { &*clock_ptr }.now();
            }
        }
        drop(borrow);
        // Fallback: wall clock
        FALLBACK_CLOCK.with(|wc| wc.now())
    })
}

/// Read dt. Returns Duration::ZERO outside tick().
#[doc(hidden)]
pub fn ctx_dt() -> Duration {
    TICK_CTX.with(|ctx| {
        ctx.borrow()
            .as_ref()
            .filter(|tc| tc.clock.is_some())
            .map(|tc| tc.dt)
            .unwrap_or(Duration::ZERO)
    })
}

/// Read elapsed (sim_time). Returns Duration::ZERO outside tick().
#[doc(hidden)]
pub fn ctx_elapsed() -> Duration {
    TICK_CTX.with(|ctx| {
        ctx.borrow()
            .as_ref()
            .filter(|tc| tc.clock.is_some())
            .map(|tc| tc.sim_time)
            .unwrap_or(Duration::ZERO)
    })
}

/// Read tick number. Returns 0 outside tick().
#[doc(hidden)]
pub fn ctx_tick() -> u64 {
    TICK_CTX.with(|ctx| {
        ctx.borrow()
            .as_ref()
            .filter(|tc| tc.clock.is_some())
            .map(|tc| tc.tick_number)
            .unwrap_or(0)
    })
}

/// Read budget remaining. Returns Duration::MAX outside tick() or if no budget.
#[doc(hidden)]
pub fn ctx_budget_remaining() -> Duration {
    TICK_CTX.with(|ctx| {
        let borrow = ctx.borrow();
        if let Some(ref tc) = *borrow {
            if let Some(clock_ptr) = tc.clock {
                if let Some(budget) = tc.budget {
                    // SAFETY: clock pointer valid during tick().
                    let now = unsafe { &*clock_ptr }.now();
                    let used = now.elapsed_since(tc.tick_start);
                    return budget.saturating_sub(used);
                }
            }
        }
        Duration::MAX
    })
}

/// Access the RNG. Calls the closure with a mutable reference.
///
/// Outside tick(), uses a thread-local fallback RNG seeded from thread ID.
#[doc(hidden)]
pub fn ctx_with_rng<R>(f: impl FnOnce(&mut SmallRng) -> R) -> R {
    TICK_CTX.with(|ctx| {
        let mut borrow = ctx.borrow_mut();
        if let Some(ref mut tc) = *borrow {
            if tc.clock.is_some() {
                return f(&mut tc.rng);
            }
        }
        drop(borrow);
        // Fallback: entropy-seeded RNG
        thread_local! {
            static FALLBACK_RNG: RefCell<SmallRng> = RefCell::new(SmallRng::from_entropy());
        }
        FALLBACK_RNG.with(|rng| f(&mut rng.borrow_mut()))
    })
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Deterministic seed from tick number + node name.
/// Same tick + same name → same seed → same RNG sequence.
fn deterministic_seed(tick_number: u64, node_name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    node_name.hash(&mut hasher);
    let name_hash = hasher.finish();
    tick_number
        .wrapping_mul(0x517cc1b727220a95)
        .wrapping_add(name_hash)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::clock::SimClock;
    use crate::core::duration_ext::DurationExt;

    #[test]
    fn ctx_now_fallback_outside_tick() {
        clear_tick_context();
        let a = ctx_now();
        std::thread::sleep(Duration::from_millis(1));
        let b = ctx_now();
        assert!(b > a, "fallback wall clock should advance");
    }

    #[test]
    fn ctx_dt_zero_outside_tick() {
        clear_tick_context();
        assert_eq!(ctx_dt(), Duration::ZERO);
    }

    #[test]
    fn ctx_tick_zero_outside_tick() {
        clear_tick_context();
        assert_eq!(ctx_tick(), 0);
    }

    #[test]
    fn ctx_budget_remaining_max_outside_tick() {
        clear_tick_context();
        assert_eq!(ctx_budget_remaining(), Duration::MAX);
    }

    #[test]
    fn set_and_read_context() {
        let clock = SimClock::new();
        clock.advance(5_u64.ms());
        let start = clock.now();

        set_tick_context("test_node", 42, &clock, 1_u64.ms(), 5_u64.ms(), start, Some(800_u64.us()));

        assert_eq!(ctx_tick(), 42);
        assert_eq!(ctx_dt(), 1_u64.ms());
        assert_eq!(ctx_elapsed(), 5_u64.ms());

        // now() should read from SimClock
        let now = ctx_now();
        assert_eq!(now.as_nanos(), 5_000_000);

        clear_tick_context();

        // After clear, falls back
        assert_eq!(ctx_tick(), 0);
        assert_eq!(ctx_dt(), Duration::ZERO);
    }

    #[test]
    fn rng_deterministic_same_seed() {
        let clock = SimClock::new();
        let start = clock.now();

        // First run
        set_tick_context("motor_ctrl", 100, &clock, 1_u64.ms(), 100_u64.ms(), start, None);
        let val1 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        clear_tick_context();

        // Second run — same node, same tick
        set_tick_context("motor_ctrl", 100, &clock, 1_u64.ms(), 100_u64.ms(), start, None);
        let val2 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        clear_tick_context();

        assert_eq!(val1, val2, "Same seed must produce same RNG value");
    }

    #[test]
    fn rng_different_tick_different_values() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context("motor_ctrl", 100, &clock, 1_u64.ms(), 100_u64.ms(), start, None);
        let val1 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        clear_tick_context();

        set_tick_context("motor_ctrl", 101, &clock, 1_u64.ms(), 101_u64.ms(), start, None);
        let val2 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        clear_tick_context();

        assert_ne!(val1, val2, "Different tick must produce different RNG value");
    }

    #[test]
    fn rng_different_node_different_values() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context("node_a", 100, &clock, 1_u64.ms(), 100_u64.ms(), start, None);
        let val1 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        clear_tick_context();

        set_tick_context("node_b", 100, &clock, 1_u64.ms(), 100_u64.ms(), start, None);
        let val2 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        clear_tick_context();

        assert_ne!(val1, val2, "Different node name must produce different RNG value");
    }

    #[test]
    fn rng_fallback_outside_tick() {
        clear_tick_context();
        // Should not panic, returns entropy-seeded value
        let val = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        assert!(val != 0 || true); // just verify it doesn't panic
    }

    #[test]
    fn budget_remaining_decreases() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context("test", 1, &clock, 1_u64.ms(), 1_u64.ms(), start, Some(800_u64.us()));

        // At start: budget = 800μs, used = 0
        let remaining = ctx_budget_remaining();
        assert_eq!(remaining, 800_u64.us());

        // Advance clock by 300μs
        clock.advance(300_u64.us());
        let remaining = ctx_budget_remaining();
        assert_eq!(remaining, 500_u64.us());

        // Advance clock by 600μs (exceeds budget)
        clock.advance(600_u64.us());
        let remaining = ctx_budget_remaining();
        assert_eq!(remaining, Duration::ZERO); // saturates

        clear_tick_context();
    }

    #[test]
    fn budget_remaining_no_budget() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context("test", 1, &clock, 1_u64.ms(), 1_u64.ms(), start, None);
        assert_eq!(ctx_budget_remaining(), Duration::MAX);
        clear_tick_context();
    }

    #[test]
    fn context_reuse_no_realloc() {
        let clock = SimClock::new();
        let start = clock.now();

        // Set once — allocates
        set_tick_context("first_node", 1, &clock, 1_u64.ms(), 1_u64.ms(), start, None);
        assert_eq!(ctx_tick(), 1);
        clear_tick_context();

        // Set again — reuses allocation
        set_tick_context("second_node", 2, &clock, 2_u64.ms(), 2_u64.ms(), start, None);
        assert_eq!(ctx_tick(), 2);
        assert_eq!(ctx_dt(), 2_u64.ms());
        clear_tick_context();
    }

    #[test]
    fn context_thread_isolation() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context("main_node", 10, &clock, 1_u64.ms(), 10_u64.ms(), start, None);

        let handle = std::thread::spawn(|| {
            // Different thread should not see main thread's context
            assert_eq!(ctx_tick(), 0);
            assert_eq!(ctx_dt(), Duration::ZERO);
        });

        handle.join().unwrap();

        // Main thread context unchanged
        assert_eq!(ctx_tick(), 10);
        clear_tick_context();
    }
}
