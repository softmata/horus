//! Loom-based exhaustive concurrency tests for the CircuitBreaker state machine.
//!
//! These tests verify that the CAS-based state transitions in CircuitBreaker
//! are race-free under every possible thread interleaving. Loom replaces the
//! standard atomics with its own instrumented versions and explores the full
//! space of concurrent executions.
//!
//! The circuit breaker is re-implemented here with loom atomics so loom can
//! track every load/store/CAS and detect data races or invariant violations.
//!
//! Run with:
//!   cargo test --test loom_circuit_breaker -- --nocapture
//!
//! # Invariants under test
//!
//! 1. **Closed → Open (record_failure)**:
//!    When N concurrent threads each call record_failure() and the failure
//!    threshold is N, exactly ONE thread must win the CAS and transition to
//!    Open. All other threads must see the CAS fail and not perform the
//!    associated side effect (recording failure time).  The state must end
//!    up as Open, never as a mix of Closed and Open.
//!
//! 2. **HalfOpen → Closed (record_success)**:
//!    When N concurrent threads call record_success() and the success
//!    threshold is N, exactly ONE thread resets the counters. The state
//!    must end up as Closed with failure_count == 0 and success_count == 0.
//!
//! # Why bare store() is wrong
//!
//! Without CAS:
//! - Thread A reads state=Closed, increments count to threshold, stores Open.
//! - Thread B reads state=Closed (before A's store is visible), also increments
//!   count to threshold, also stores Open.
//! - Both threads execute the failure-time side effect — last_failure_time is
//!   updated twice, potentially extending the cooldown window unexpectedly.
//! - The state is still Open (idempotent), but counter resets in subsequent
//!   transitions may fire twice, leaving the machine in an inconsistent state.

use loom::sync::{
    atomic::{AtomicU32, AtomicU8, Ordering},
    Arc,
};

/// Circuit breaker states (mirrors CircuitState in production code).
const STATE_CLOSED: u8 = 0;
const STATE_OPEN: u8 = 1;
const STATE_HALF_OPEN: u8 = 2;

/// Minimal circuit breaker with CAS state transitions, implemented with
/// loom atomics for exhaustive concurrency verification.
///
/// Uses an `AtomicU32` side-effect counter to record how many times the
/// "winner" callback fired. In production this is last_failure_time update
/// or counter reset; here we just count invocations.
struct LoomCircuitBreaker {
    state: AtomicU8,
    failure_count: AtomicU32,
    success_count: AtomicU32,
    /// Counts how many threads won a CAS transition (should always be 1).
    transition_winner_count: AtomicU32,
    failure_threshold: u32,
    success_threshold: u32,
}

impl LoomCircuitBreaker {
    fn new_closed(failure_threshold: u32, success_threshold: u32) -> Arc<Self> {
        Arc::new(Self {
            state: AtomicU8::new(STATE_CLOSED),
            failure_count: AtomicU32::new(0),
            success_count: AtomicU32::new(0),
            transition_winner_count: AtomicU32::new(0),
            failure_threshold,
            success_threshold,
        })
    }

    fn new_half_open(success_threshold: u32) -> Arc<Self> {
        Arc::new(Self {
            state: AtomicU8::new(STATE_HALF_OPEN),
            failure_count: AtomicU32::new(0),
            success_count: AtomicU32::new(0),
            transition_winner_count: AtomicU32::new(0),
            failure_threshold: u32::MAX,
            success_threshold,
        })
    }

    /// CAS-based record_failure for Closed → Open transition.
    fn record_failure_closed(&self) {
        let count = self.failure_count.fetch_add(1, Ordering::AcqRel) + 1;
        if count >= self.failure_threshold {
            if self
                .state
                .compare_exchange(
                    STATE_CLOSED,
                    STATE_OPEN,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                // Exactly one thread should reach here.
                self.transition_winner_count.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// CAS-based record_success for HalfOpen → Closed transition.
    fn record_success_half_open(&self) {
        let count = self.success_count.fetch_add(1, Ordering::AcqRel) + 1;
        if count >= self.success_threshold {
            if self
                .state
                .compare_exchange(
                    STATE_HALF_OPEN,
                    STATE_CLOSED,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                // Exactly one thread should reach here.
                self.transition_winner_count.fetch_add(1, Ordering::Relaxed);
                // Reset counters (only done once by the winner).
                self.failure_count.store(0, Ordering::Release);
                self.success_count.store(0, Ordering::Release);
            }
        }
    }
}

/// Loom test: 2 threads calling record_failure() concurrently with threshold 2.
///
/// Invariant: exactly one CAS wins (transition_winner_count == 1) and the
/// final state is Open.
#[test]
fn test_concurrent_failure_exactly_one_open_transition() {
    loom::model(|| {
        let cb = LoomCircuitBreaker::new_closed(2, 1);

        let t1_cb = cb.clone();
        let t1 = loom::thread::spawn(move || {
            t1_cb.record_failure_closed();
        });

        let t2_cb = cb.clone();
        let t2 = loom::thread::spawn(move || {
            t2_cb.record_failure_closed();
        });

        t1.join().unwrap();
        t2.join().unwrap();

        // Both threads called record_failure — count must have reached threshold
        assert_eq!(cb.failure_count.load(Ordering::Acquire), 2);

        // State must be Open
        assert_eq!(
            cb.state.load(Ordering::Acquire),
            STATE_OPEN,
            "state must be Open after threshold failures"
        );

        // Exactly one thread won the CAS
        assert_eq!(
            cb.transition_winner_count.load(Ordering::Acquire),
            1,
            "exactly one thread must win the Closed→Open CAS"
        );
    });
}

/// Loom test: 2 threads calling record_success() concurrently with threshold 2.
///
/// Invariant: exactly one CAS wins the HalfOpen → Closed transition
/// (transition_winner_count == 1), and after the transition, failure_count
/// and success_count are both 0 (counters reset exactly once).
#[test]
fn test_concurrent_success_exactly_one_closed_transition() {
    loom::model(|| {
        let cb = LoomCircuitBreaker::new_half_open(2);

        let t1_cb = cb.clone();
        let t1 = loom::thread::spawn(move || {
            t1_cb.record_success_half_open();
        });

        let t2_cb = cb.clone();
        let t2 = loom::thread::spawn(move || {
            t2_cb.record_success_half_open();
        });

        t1.join().unwrap();
        t2.join().unwrap();

        // State must be Closed
        assert_eq!(
            cb.state.load(Ordering::Acquire),
            STATE_CLOSED,
            "state must be Closed after success threshold"
        );

        // Exactly one thread won the CAS
        assert_eq!(
            cb.transition_winner_count.load(Ordering::Acquire),
            1,
            "exactly one thread must win the HalfOpen→Closed CAS"
        );

        // Counters were reset exactly once by the winner.
        // After the reset, no new failures/successes can have incremented
        // them (both threads finished), so they must both be 0.
        assert_eq!(
            cb.failure_count.load(Ordering::Acquire),
            0,
            "failure_count must be 0 after counter reset"
        );
        assert_eq!(
            cb.success_count.load(Ordering::Acquire),
            0,
            "success_count must be 0 after counter reset"
        );
    });
}

/// Loom test: one thread fails while the other succeeds in HalfOpen.
///
/// Either the failure re-opens (HalfOpen → Open) or the success closes
/// (HalfOpen → Closed). The final state must be one of those two, and
/// exactly one transition_winner must exist (no double-transition).
#[test]
fn test_concurrent_half_open_fail_and_success() {
    loom::model(|| {
        let cb = LoomCircuitBreaker::new_half_open(1);

        // Thread 1: record a failure (HalfOpen → Open)
        let t1_cb = cb.clone();
        let t1 = loom::thread::spawn(move || {
            // HalfOpen → Open CAS path
            if t1_cb
                .state
                .compare_exchange(
                    STATE_HALF_OPEN,
                    STATE_OPEN,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                t1_cb
                    .transition_winner_count
                    .fetch_add(1, Ordering::Relaxed);
                t1_cb.failure_count.store(0, Ordering::Release);
                t1_cb.success_count.store(0, Ordering::Release);
            }
        });

        // Thread 2: record a success (HalfOpen → Closed)
        let t2_cb = cb.clone();
        let t2 = loom::thread::spawn(move || {
            t2_cb.record_success_half_open();
        });

        t1.join().unwrap();
        t2.join().unwrap();

        let final_state = cb.state.load(Ordering::Acquire);
        // Final state must be either Open or Closed, never HalfOpen
        assert!(
            final_state == STATE_OPEN || final_state == STATE_CLOSED,
            "final state must be Open or Closed, got {}",
            final_state
        );

        // Exactly one of the two CAS operations must have won
        assert_eq!(
            cb.transition_winner_count.load(Ordering::Acquire),
            1,
            "exactly one transition must win"
        );
    });
}
