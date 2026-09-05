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

// SAFETY: this impl is a promise that a `TickContext` may be MOVED to another
// thread, and the previous comment ("only accessed from the thread that owns the
// thread-local") argued the opposite — it is the reason the impl is not needed,
// not a reason it is sound. What actually makes it sound is narrower, and worth
// writing down because it is also the invariant that keeps `clock` from
// dangling: the only `TickContext` that ever exists lives in `TICK_CTX`, a
// thread-local, and is written solely by `set_tick_context` and cleared solely
// by `clear_tick_context` on the same thread; nothing in this crate hands one
// out, boxes it, or sends it. `clock` is therefore only ever dereferenced on the
// thread whose `set_tick_context` installed it, inside the enter..clear window
// the scheduler brackets each `tick()` with (`primitives::TickContextGuard`
// clears it even when the tick unwinds).
//
// If a future change ever does move a `TickContext` across a thread boundary,
// this impl is what will let it compile, and the `clock` pointer's validity
// argument does NOT survive that move. Delete the impl rather than widen it.
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
    set_tick_context_with_hash(
        node_name,
        hash_node_name(node_name),
        tick_number,
        clock,
        dt,
        sim_time,
        tick_start,
        budget,
    )
}

/// `set_tick_context` for a caller that already knows the node's name hash.
///
/// The RNG seed is `f(tick_number, hash(node_name))`, and this runs once per
/// node per tick — so the by-name form above re-derives a per-node CONSTANT on
/// every period. At 1 kHz that is one SipHash per node per millisecond, ~25 ns
/// each, to obtain a value that cannot have changed.
///
/// It is deliberately a parameter rather than a memo on the context. A memo
/// would have to be keyed on the name and revalidated, and it only ever hits
/// when the SAME node is entered twice in a row — which for a multi-node chain
/// is never, since the executor walks A, B, C, A, B, C and the name differs on
/// every call. Measured, that memo cost 5 ns per tick more than it saved on any
/// chain longer than one node. An executor that resolved the hash once at
/// thread start pays nothing here at all.
///
/// `name_hash` MUST be `hash_node_name(node_name)`; passing anything else
/// silently changes `horus::rng()`'s stream for that node.
#[doc(hidden)]
#[allow(clippy::too_many_arguments)]
pub fn set_tick_context_with_hash(
    node_name: &str,
    name_hash: u64,
    tick_number: u64,
    clock: &dyn Clock,
    dt: Duration,
    sim_time: Duration,
    tick_start: ClockInstant,
    budget: Option<Duration>,
) {
    let seed = seed_from_parts(tick_number, name_hash);
    let rng = SmallRng::seed_from_u64(seed);
    // Erase the trait object's lifetime so the pointer can live in a `'static`
    // thread-local.
    //
    // The `&dyn Clock -> *const dyn Clock` coercion is NOT sufficient on its
    // own, and it is worth being precise about why, because this transmute was
    // once removed as a no-op and the crate stopped compiling. A raw pointer to
    // a trait object still carries the object's lifetime bound: the coercion
    // gives `*const (dyn Clock + '_)`, borrowed from the argument, while
    // `TickContext::clock` is `*const (dyn Clock + 'static)`. Both spell
    // `*const dyn Clock` in source, which is exactly what makes the old pair of
    // identically-annotated bindings read as an identity transmute — lifetime
    // elision was filling in two different lifetimes.
    //
    // SAFETY: the `'static` is a storage requirement, not a claim that the clock
    // lives forever. The pointer is only ever dereferenced on this thread inside
    // the enter..clear window the scheduler brackets each `tick()` with:
    // `clear_tick_context` nulls it before `clock`'s borrow ends, and
    // `primitives::TickContextGuard` runs that on the unwinding path too.
    let clock_ptr = clock as *const dyn Clock;
    let clock_ptr: *const (dyn Clock + 'static) = unsafe { std::mem::transmute(clock_ptr) };

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
            static FALLBACK_RNG: RefCell<SmallRng> = RefCell::new(SmallRng::from_os_rng());
        }
        FALLBACK_RNG.with(|rng| f(&mut rng.borrow_mut()))
    })
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Deterministic seed from tick number + node name.
/// Same tick + same name → same seed → same RNG sequence.
pub(crate) fn hash_node_name(node_name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    node_name.hash(&mut hasher);
    hasher.finish()
}

fn seed_from_parts(tick_number: u64, name_hash: u64) -> u64 {
    tick_number
        .wrapping_mul(0x517cc1b727220a95)
        .wrapping_add(name_hash)
}

/// Unchanged in behaviour — `seed_from_parts(tick, hash_node_name(name))` is
/// the same expression the single function used to evaluate inline. Kept so the
/// property test below can assert the memoized path agrees with it.
#[cfg(test)]
pub(crate) fn deterministic_seed(tick_number: u64, node_name: &str) -> u64 {
    seed_from_parts(tick_number, hash_node_name(node_name))
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::clock::SimClock;
    use crate::core::duration_ext::DurationExt;
    use rand::RngCore;

    /// What resolving the node-name hash once removes from each tick.
    ///
    /// Also records the road not taken. Memoizing the hash on the thread-local
    /// context — comparing the name and reusing the hash when it matches — is
    /// the obvious cheaper-looking fix, and it is worse: it only HITS when the
    /// same node is entered twice in a row, which for a multi-node chain never
    /// happens, because the executor walks A, B, C, A, B, C and the name
    /// differs on every call. On a miss it pays the comparison AND the hash.
    /// The numbers below are why this file takes the hash as a parameter
    /// instead.
    ///
    /// `#[ignore]`d: a stopwatch, not an assertion. Run with:
    ///
    /// ```text
    /// cargo test -p horus_core --release name_hash_cost -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "stopwatch, not an assertion — run manually with --ignored"]
    fn name_hash_cost() {
        use std::hint::black_box;
        use std::time::Instant;

        const N: usize = 2_000_000;
        let name = "perception_stage_3";
        let other = "perception_stage_4";

        // What the old path paid every tick, and what passing the hash removes.
        let t = Instant::now();
        let mut acc = 0u64;
        for _ in 0..N {
            acc ^= hash_node_name(black_box(name));
        }
        black_box(acc);
        let hashing = t.elapsed().as_nanos() as f64 / N as f64;

        // A thread-local memo, had it hit: the guard comparison alone.
        let t = Instant::now();
        let mut hits = 0u64;
        for _ in 0..N {
            hits += (black_box(name) == black_box(name)) as u64;
        }
        black_box(hits);
        let memo_hit = t.elapsed().as_nanos() as f64 / N as f64;

        // The same memo on a miss, which is what a multi-node chain always gets.
        let t = Instant::now();
        let mut acc = 0u64;
        for _ in 0..N {
            if black_box(name) != black_box(other) {
                acc ^= hash_node_name(black_box(other));
            }
        }
        black_box(acc);
        let memo_miss = t.elapsed().as_nanos() as f64 / N as f64;

        println!("\n  node-name hash, per node per tick:");
        println!("    recompute every tick (what this replaces): {hashing:6.2} ns");
        println!("    resolved once at thread start (now)      :   0.00 ns");
        println!("    -- the memo alternative, for the record --");
        println!("    memo hit  (single-node chain only)       : {memo_hit:6.2} ns");
        println!("    memo miss (any chain of 2+ nodes)        : {memo_miss:6.2} ns");
        println!(
            "    => a memo LOSES {:.2} ns/tick on a multi-node chain\n",
            memo_miss - hashing
        );
    }

    /// The precomputed-hash path must produce a bit-identical seed to the
    /// by-name path, for every node and every tick.
    ///
    /// This is the whole correctness burden of letting the RT executor resolve
    /// `hash_node_name` once at thread start and pass it in. `horus::rng()` is
    /// documented as deterministic in (tick, node) — replay, simulation
    /// reproducibility and every seeded-fixture test rest on it. If the two
    /// paths disagreed, nothing would fail loudly: RT nodes would simply draw a
    /// different random stream than the same node drew on the main loop, and
    /// two runs of the same scenario would quietly diverge.
    #[test]
    fn the_precomputed_hash_path_seeds_identically_to_the_by_name_path() {
        let clock = SimClock::new();

        // A first draw from a freshly seeded RNG identifies the seed: SmallRng
        // is a pure function of it.
        fn draw(hashed: bool, name: &str, tick: u64, clock: &SimClock) -> u64 {
            if hashed {
                set_tick_context_with_hash(
                    name,
                    hash_node_name(name),
                    tick,
                    clock,
                    Duration::from_millis(1),
                    Duration::ZERO,
                    ClockInstant::from_nanos(0),
                    None,
                );
            } else {
                set_tick_context(
                    name,
                    tick,
                    clock,
                    Duration::from_millis(1),
                    Duration::ZERO,
                    ClockInstant::from_nanos(0),
                    None,
                );
            }
            let drawn = ctx_with_rng(|r| r.next_u64());
            clear_tick_context();
            drawn
        }

        for name in ["perception", "control", "", "a", "perception_stage_11"] {
            for tick in 0..8u64 {
                let by_name = draw(false, name, tick, &clock);
                let precomputed = draw(true, name, tick, &clock);
                assert_eq!(
                    by_name, precomputed,
                    "precomputed hash changed the RNG stream for {name:?} at tick {tick}"
                );
                // ...and both must equal the seed the documented formula gives.
                assert_eq!(
                    by_name,
                    SmallRng::seed_from_u64(deterministic_seed(tick, name)).next_u64(),
                    "the seed for {name:?} at tick {tick} is no longer f(tick, hash(name))"
                );
            }
        }

        // Distinct nodes on the same tick must still differ — the property the
        // per-node hash exists to provide.
        assert_ne!(
            draw(true, "perception", 3, &clock),
            draw(true, "control", 3, &clock)
        );
    }

    /// Interleaving nodes on one thread must not let one node's seed leak into
    /// another's.
    ///
    /// An executor thread walks its whole chain every period — A, B, C, A, B,
    /// C — reusing one thread-local context, and a mixed-rate chain also
    /// produces runs like A, A, ..., A, B, A, A where a node repeats. Any
    /// per-thread caching of the name or its hash has to survive both shapes.
    #[test]
    fn interleaved_nodes_on_one_thread_keep_their_own_seeds() {
        let clock = SimClock::new();

        let draw = |name: &str, tick: u64| {
            set_tick_context_with_hash(
                name,
                hash_node_name(name),
                tick,
                &clock,
                Duration::from_millis(1),
                Duration::ZERO,
                ClockInstant::from_nanos(0),
                None,
            );
            let d = ctx_with_rng(|r| r.next_u64());
            clear_tick_context();
            d
        };
        let expect = |name: &str, tick: u64| {
            SmallRng::seed_from_u64(deterministic_seed(tick, name)).next_u64()
        };

        // Round-robin, the multi-node chain shape.
        for tick in 0..4u64 {
            for name in ["a_node", "b_node", "c_node"] {
                assert_eq!(draw(name, tick), expect(name, tick), "{name} at {tick}");
            }
        }

        // A node repeating right after a different one ran, the mixed-rate shape.
        for tick in 0..4u64 {
            let _ = draw("slow_planner", tick);
            assert_eq!(draw("fast_servo", tick), expect("fast_servo", tick));
            assert_eq!(draw("fast_servo", tick + 1), expect("fast_servo", tick + 1));
        }
    }
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

        set_tick_context(
            "test_node",
            42,
            &clock,
            1_u64.ms(),
            5_u64.ms(),
            start,
            Some(800_u64.us()),
        );

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
        set_tick_context(
            "motor_ctrl",
            100,
            &clock,
            1_u64.ms(),
            100_u64.ms(),
            start,
            None,
        );
        let val1 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.random::<u64>()
        });
        clear_tick_context();

        // Second run — same node, same tick
        set_tick_context(
            "motor_ctrl",
            100,
            &clock,
            1_u64.ms(),
            100_u64.ms(),
            start,
            None,
        );
        let val2 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.random::<u64>()
        });
        clear_tick_context();

        assert_eq!(val1, val2, "Same seed must produce same RNG value");
    }

    #[test]
    fn rng_different_tick_different_values() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context(
            "motor_ctrl",
            100,
            &clock,
            1_u64.ms(),
            100_u64.ms(),
            start,
            None,
        );
        let val1 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.random::<u64>()
        });
        clear_tick_context();

        set_tick_context(
            "motor_ctrl",
            101,
            &clock,
            1_u64.ms(),
            101_u64.ms(),
            start,
            None,
        );
        let val2 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.random::<u64>()
        });
        clear_tick_context();

        assert_ne!(
            val1, val2,
            "Different tick must produce different RNG value"
        );
    }

    #[test]
    fn rng_different_node_different_values() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context("node_a", 100, &clock, 1_u64.ms(), 100_u64.ms(), start, None);
        let val1 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.random::<u64>()
        });
        clear_tick_context();

        set_tick_context("node_b", 100, &clock, 1_u64.ms(), 100_u64.ms(), start, None);
        let val2 = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.random::<u64>()
        });
        clear_tick_context();

        assert_ne!(
            val1, val2,
            "Different node name must produce different RNG value"
        );
    }

    #[test]
    fn rng_fallback_outside_tick() {
        clear_tick_context();
        // Should not panic, returns entropy-seeded value
        let val = ctx_with_rng(|rng| {
            use rand::Rng;
            rng.random::<u64>()
        });
        let _ = val; // just verify it doesn't panic
    }

    #[test]
    fn budget_remaining_decreases() {
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
        set_tick_context(
            "second_node",
            2,
            &clock,
            2_u64.ms(),
            2_u64.ms(),
            start,
            None,
        );
        assert_eq!(ctx_tick(), 2);
        assert_eq!(ctx_dt(), 2_u64.ms());
        clear_tick_context();
    }

    #[test]
    fn context_thread_isolation() {
        let clock = SimClock::new();
        let start = clock.now();

        set_tick_context(
            "main_node",
            10,
            &clock,
            1_u64.ms(),
            10_u64.ms(),
            start,
            None,
        );

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
