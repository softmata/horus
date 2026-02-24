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

    /// Get deterministic random number
    pub fn random_u64(&self) -> u64 {
        // Simple xorshift64 for deterministic randomness
        let mut state = self.rng_state.load(Ordering::Acquire);
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        self.rng_state.store(state, Ordering::Release);
        state
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
}
