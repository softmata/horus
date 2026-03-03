//! Deterministic Execution System
//!
//! Provides reproducible execution of node graphs using virtual time
//! and seeded RNG. Useful for debugging and regression testing.
//!
//! ## Key Features
//!
//! - **Deterministic Clock**: Virtual time that advances predictably
//! - **Seeded RNG**: All randomness derived from seed

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

/// Configuration for deterministic execution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeterministicConfig {
    /// Seed for deterministic RNG
    pub seed: u64,
    /// Whether to use virtual time (ignore wall clock)
    pub virtual_time: bool,
    /// Fixed tick duration in nanoseconds (for virtual time)
    pub tick_duration_ns: u64,
}

impl Default for DeterministicConfig {
    fn default() -> Self {
        Self {
            seed: 42,
            virtual_time: true,
            tick_duration_ns: 1_000_000, // 1ms per tick
        }
    }
}

/// Deterministic clock that provides reproducible time
#[derive(Debug)]
pub struct DeterministicClock {
    /// Current virtual time in nanoseconds
    virtual_time_ns: AtomicU64,
    /// Tick duration in nanoseconds
    tick_duration_ns: u64,
    /// Current tick number
    tick: AtomicU64,
    /// Whether to use virtual time
    use_virtual_time: bool,
    /// Real start time (for hybrid mode)
    real_start: Instant,
    /// Seed for RNG
    seed: u64,
    /// Current RNG state
    rng_state: AtomicU64,
}

impl DeterministicClock {
    /// Create a new deterministic clock
    pub fn new(config: &DeterministicConfig) -> Self {
        Self {
            virtual_time_ns: AtomicU64::new(0),
            tick_duration_ns: config.tick_duration_ns,
            tick: AtomicU64::new(0),
            use_virtual_time: config.virtual_time,
            real_start: Instant::now(),
            seed: config.seed,
            rng_state: AtomicU64::new(config.seed),
        }
    }

    /// Get current time in nanoseconds
    pub fn now_ns(&self) -> u64 {
        if self.use_virtual_time {
            self.virtual_time_ns.load(Ordering::Acquire)
        } else {
            self.real_start.elapsed().as_nanos() as u64
        }
    }

    /// Get current time as Duration
    pub fn now(&self) -> Duration {
        Duration::from_nanos(self.now_ns())
    }

    /// Get current tick number
    pub fn tick(&self) -> u64 {
        self.tick.load(Ordering::Acquire)
    }

    /// Advance to next tick
    pub fn advance_tick(&self) -> u64 {
        let new_tick = self.tick.fetch_add(1, Ordering::AcqRel) + 1;
        if self.use_virtual_time {
            self.virtual_time_ns
                .fetch_add(self.tick_duration_ns, Ordering::AcqRel);
        }
        new_tick
    }

    /// Get deterministic random number.
    ///
    /// Uses a CAS loop to atomically advance the xorshift64 state.  The old
    /// load+compute+store sequence had a lost-update race: two threads loading
    /// the same state would compute the same next value, both store it, and
    /// return identical numbers.  The CAS loop guarantees each winner sees a
    /// distinct `old_state`, so every returned value is unique within the
    /// xorshift64 cycle (2^64 − 1 values before repeating).
    pub fn random_u64(&self) -> u64 {
        loop {
            let old_state = self.rng_state.load(Ordering::Acquire);
            // xorshift64 — bijection over all non-zero u64 values
            let mut new_state = old_state;
            new_state ^= new_state << 13;
            new_state ^= new_state >> 7;
            new_state ^= new_state << 17;
            // Only the thread that wins the CAS advances the state and returns.
            // Losers reload old_state (which now has the winner's new value) and retry.
            if self
                .rng_state
                .compare_exchange(old_state, new_state, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return new_state;
            }
            // Another thread advanced the state concurrently; spin and retry.
        }
    }

    /// Reset clock to initial state
    pub fn reset(&self) {
        self.tick.store(0, Ordering::Release);
        self.virtual_time_ns.store(0, Ordering::Release);
        self.rng_state.store(self.seed, Ordering::Release);
    }

    /// Get seed
    pub fn seed(&self) -> u64 {
        self.seed
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deterministic_clock() {
        let config = DeterministicConfig::default();
        let clock = DeterministicClock::new(&config);

        assert_eq!(clock.tick(), 0);
        assert_eq!(clock.now_ns(), 0);

        clock.advance_tick();
        assert_eq!(clock.tick(), 1);
        assert_eq!(clock.now_ns(), config.tick_duration_ns);

        clock.advance_tick();
        assert_eq!(clock.tick(), 2);
        assert_eq!(clock.now_ns(), config.tick_duration_ns * 2);
    }

    #[test]
    fn test_deterministic_rng() {
        let config = DeterministicConfig::default();
        let clock1 = DeterministicClock::new(&config);
        let clock2 = DeterministicClock::new(&config);

        // Same seed should produce same sequence
        let seq1: Vec<u64> = (0..10).map(|_| clock1.random_u64()).collect();
        let seq2: Vec<u64> = (0..10).map(|_| clock2.random_u64()).collect();

        assert_eq!(seq1, seq2);
    }

    /// Verify that concurrent calls to random_u64() never return duplicates.
    ///
    /// xorshift64 is a bijection (period 2^64−1) and the CAS loop guarantees
    /// each call wins with a distinct old_state, so no two calls ever return
    /// the same value.  4 threads × 1000 calls = 4000 values; all must be
    /// unique.
    #[test]
    fn test_random_u64_concurrent_uniqueness() {
        use std::collections::HashSet;
        use std::sync::{Arc, Mutex};
        use std::thread;

        let config = DeterministicConfig::default();
        let clock = Arc::new(DeterministicClock::new(&config));
        let collected: Arc<Mutex<HashSet<u64>>> = Arc::new(Mutex::new(HashSet::new()));

        const THREAD_COUNT: usize = 4;
        const CALLS_PER_THREAD: usize = 1000;

        let mut handles = Vec::with_capacity(THREAD_COUNT);
        for _ in 0..THREAD_COUNT {
            let c = clock.clone();
            let col = collected.clone();
            handles.push(thread::spawn(move || {
                let values: Vec<u64> = (0..CALLS_PER_THREAD).map(|_| c.random_u64()).collect();
                let mut set = col.lock().unwrap();
                for v in values {
                    assert!(set.insert(v), "random_u64() returned duplicate value {}", v);
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        let total = collected.lock().unwrap().len();
        assert_eq!(
            total,
            THREAD_COUNT * CALLS_PER_THREAD,
            "expected {} unique values, got {}",
            THREAD_COUNT * CALLS_PER_THREAD,
            total
        );
    }

    /// Verify that advance_tick() keeps virtual_time_ns monotonically
    /// non-decreasing under concurrent access from 8 threads.
    ///
    /// The fix is to use fetch_add (not load+compute+store) for virtual_time_ns
    /// so there is no window where thread A's computed time (tick=100) can
    /// overwrite thread B's already-stored higher time (tick=101).
    ///
    /// Each thread samples now_ns() after every advance_tick() call and asserts
    /// it never decreased relative to the previous sample in that thread.
    #[test]
    fn test_advance_tick_concurrent_monotonicity() {
        use std::sync::Arc;
        use std::thread;

        let config = DeterministicConfig {
            seed: 0,
            virtual_time: true,
            tick_duration_ns: 1_000_000, // 1ms per tick
        };
        let clock = Arc::new(DeterministicClock::new(&config));

        const THREAD_COUNT: usize = 8;
        const TICKS_PER_THREAD: usize = 1000;

        let mut handles = Vec::with_capacity(THREAD_COUNT);
        for _ in 0..THREAD_COUNT {
            let c = clock.clone();
            handles.push(thread::spawn(move || {
                let mut last_ns = 0u64;
                for _ in 0..TICKS_PER_THREAD {
                    c.advance_tick();
                    let now = c.now_ns();
                    assert!(
                        now >= last_ns,
                        "virtual_time_ns went backwards: {} < {}",
                        now,
                        last_ns
                    );
                    last_ns = now;
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        // All 8 * 1000 ticks must have been applied exactly once.
        let expected_ticks = (THREAD_COUNT * TICKS_PER_THREAD) as u64;
        assert_eq!(
            clock.tick(),
            expected_ticks,
            "tick counter must equal total advance_tick() calls"
        );
        assert_eq!(
            clock.now_ns(),
            expected_ticks * config.tick_duration_ns,
            "virtual_time_ns must equal ticks * period — no tick lost"
        );
    }
}
