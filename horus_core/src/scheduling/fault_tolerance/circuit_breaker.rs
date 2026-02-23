use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Circuit breaker states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircuitState {
    /// Normal operation - allowing all requests
    Closed = 0,
    /// Failing too often - rejecting all requests
    Open = 1,
    /// Testing if the service has recovered
    HalfOpen = 2,
}

impl From<u8> for CircuitState {
    fn from(value: u8) -> Self {
        match value {
            0 => CircuitState::Closed,
            1 => CircuitState::Open,
            2 => CircuitState::HalfOpen,
            _ => CircuitState::Closed,
        }
    }
}

/// Lock-free circuit breaker for node fault tolerance
/// Prevents cascading failures by temporarily disabling failing nodes
pub struct CircuitBreaker {
    /// Current state (atomic for lock-free access)
    state: Arc<AtomicU8>,

    /// Failure count in current window
    failure_count: Arc<AtomicU32>,

    /// Success count in half-open state
    success_count: Arc<AtomicU32>,

    /// Configuration
    failure_threshold: u32,
    success_threshold: u32,
    timeout: Duration,

    /// Last failure time
    last_failure_time: Arc<parking_lot::Mutex<Option<Instant>>>,
}

impl Clone for CircuitBreaker {
    fn clone(&self) -> Self {
        // Clone creates a new instance with same config but fresh state
        Self {
            state: Arc::new(AtomicU8::new(self.state.load(Ordering::SeqCst))),
            failure_count: Arc::new(AtomicU32::new(self.failure_count.load(Ordering::SeqCst))),
            success_count: Arc::new(AtomicU32::new(self.success_count.load(Ordering::SeqCst))),
            failure_threshold: self.failure_threshold,
            success_threshold: self.success_threshold,
            timeout: self.timeout,
            last_failure_time: Arc::new(parking_lot::Mutex::new(*self.last_failure_time.lock())),
        }
    }
}

impl std::fmt::Debug for CircuitBreaker {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CircuitBreaker")
            .field("state", &self.get_state())
            .field("failure_count", &self.failure_count.load(Ordering::SeqCst))
            .field("success_count", &self.success_count.load(Ordering::SeqCst))
            .field("failure_threshold", &self.failure_threshold)
            .field("success_threshold", &self.success_threshold)
            .field("timeout", &self.timeout)
            .finish()
    }
}

impl CircuitBreaker {
    /// Create new circuit breaker
    ///
    /// # Arguments
    /// * `failure_threshold` - Number of failures before opening circuit
    /// * `success_threshold` - Number of successes in half-open before closing
    /// * `timeout_ms` - Time to wait in open state before trying half-open
    pub fn new(failure_threshold: u32, success_threshold: u32, timeout_ms: u64) -> Self {
        Self {
            state: Arc::new(AtomicU8::new(CircuitState::Closed as u8)),
            failure_count: Arc::new(AtomicU32::new(0)),
            success_count: Arc::new(AtomicU32::new(0)),
            failure_threshold,
            success_threshold,
            timeout: Duration::from_millis(timeout_ms),
            last_failure_time: Arc::new(parking_lot::Mutex::new(None)),
        }
    }

    /// Check if request should be allowed
    pub fn should_allow(&self) -> bool {
        let current_state = self.get_state();

        match current_state {
            CircuitState::Closed => true,
            CircuitState::Open => {
                // Check if timeout has elapsed
                let guard = self.last_failure_time.lock();
                if let Some(last_failure) = *guard {
                    if last_failure.elapsed() >= self.timeout {
                        // Try half-open
                        self.state
                            .store(CircuitState::HalfOpen as u8, Ordering::SeqCst);
                        self.success_count.store(0, Ordering::SeqCst);
                        return true;
                    }
                }
                false
            }
            CircuitState::HalfOpen => {
                // Allow limited requests to test recovery
                true
            }
        }
    }

    /// Record a successful execution
    pub fn record_success(&self) {
        let current_state = self.get_state();

        match current_state {
            CircuitState::Closed => {
                // Reset failure count on success
                self.failure_count.store(0, Ordering::SeqCst);
            }
            CircuitState::HalfOpen => {
                let count = self.success_count.fetch_add(1, Ordering::SeqCst) + 1;

                if count >= self.success_threshold {
                    // Recovered! Close the circuit
                    self.state
                        .store(CircuitState::Closed as u8, Ordering::SeqCst);
                    self.failure_count.store(0, Ordering::SeqCst);
                    self.success_count.store(0, Ordering::SeqCst);
                }
            }
            CircuitState::Open => {
                // Shouldn't happen, but handle gracefully
            }
        }
    }

    /// Record a failed execution
    pub fn record_failure(&self) {
        let current_state = self.get_state();

        match current_state {
            CircuitState::Closed => {
                let count = self.failure_count.fetch_add(1, Ordering::SeqCst) + 1;

                if count >= self.failure_threshold {
                    // Too many failures, open the circuit
                    self.state.store(CircuitState::Open as u8, Ordering::SeqCst);
                    let mut guard = self.last_failure_time.lock();
                    *guard = Some(Instant::now());
                }
            }
            CircuitState::HalfOpen => {
                // Failed during recovery test, reopen immediately
                self.state.store(CircuitState::Open as u8, Ordering::SeqCst);
                self.failure_count.store(0, Ordering::SeqCst);
                self.success_count.store(0, Ordering::SeqCst);
                let mut guard = self.last_failure_time.lock();
                *guard = Some(Instant::now());
            }
            CircuitState::Open => {
                // Already open, update last failure time
                let mut guard = self.last_failure_time.lock();
                *guard = Some(Instant::now());
            }
        }
    }

    /// Get current state
    pub fn get_state(&self) -> CircuitState {
        CircuitState::from(self.state.load(Ordering::SeqCst))
    }

    /// Reset circuit breaker to closed state
    #[cfg(test)]
    pub fn reset(&self) {
        self.state
            .store(CircuitState::Closed as u8, Ordering::SeqCst);
        self.failure_count.store(0, Ordering::SeqCst);
        self.success_count.store(0, Ordering::SeqCst);
        let mut guard = self.last_failure_time.lock();
        *guard = None;
    }

    /// Get current statistics
    pub fn stats(&self) -> CircuitBreakerStats {
        CircuitBreakerStats {
            state: self.get_state(),
            failure_count: self.failure_count.load(Ordering::SeqCst),
            success_count: self.success_count.load(Ordering::SeqCst),
        }
    }
}

/// Statistics for monitoring
#[derive(Debug, Clone)]
pub struct CircuitBreakerStats {
    pub state: CircuitState,
    pub failure_count: u32,
    #[cfg_attr(not(test), allow(dead_code))]
    pub success_count: u32,
}

impl Default for CircuitBreaker {
    fn default() -> Self {
        // Default: 5 failures to open, 3 successes to close, 5 second timeout
        Self::new(5, 3, 5000)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// CircuitBreaker starts in Closed state and allows all requests.
    /// Robotics: newly created sensor driver node starts in healthy state.
    #[test]
    fn initial_state_is_closed_and_allows() {
        let cb = CircuitBreaker::new(3, 2, 100);
        assert_eq!(cb.get_state(), CircuitState::Closed);
        assert!(cb.should_allow());
        let stats = cb.stats();
        assert_eq!(stats.failure_count, 0);
        assert_eq!(stats.success_count, 0);
    }

    /// Closed → Open: after failure_threshold failures, circuit opens.
    /// should_allow returns false (node disabled).
    ///
    /// Robotics: sensor driver fails 3 consecutive reads → circuit opens,
    /// scheduler stops calling tick() on this node.
    #[test]
    fn closed_to_open_after_threshold_failures() {
        let cb = CircuitBreaker::new(3, 2, 100);

        // First two failures: still Closed
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        assert!(cb.should_allow());
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        assert!(cb.should_allow());

        // Third failure: trips to Open
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Open);
        assert!(!cb.should_allow(), "Open circuit should reject requests");
    }

    /// Open → HalfOpen: after timeout elapses, should_allow transitions to HalfOpen.
    /// Next request is allowed as a probe.
    ///
    /// Robotics: sensor was disconnected, cooldown period passes, scheduler
    /// tries one tick to see if sensor reconnected.
    #[test]
    fn open_to_half_open_after_timeout() {
        let cb = CircuitBreaker::new(2, 1, 50); // 50ms timeout

        // Trip the breaker
        cb.record_failure();
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Open);
        assert!(!cb.should_allow());

        // Wait for timeout
        std::thread::sleep(Duration::from_millis(60));

        // should_allow transitions to HalfOpen and returns true
        assert!(cb.should_allow());
        assert_eq!(cb.get_state(), CircuitState::HalfOpen);
    }

    /// HalfOpen → Closed: after success_threshold successes, circuit closes.
    /// Node returns to normal operation.
    ///
    /// Robotics: sensor reconnected, passes 2 consecutive health checks,
    /// circuit closes and node resumes full-speed ticking.
    #[test]
    fn half_open_to_closed_on_success() {
        let cb = CircuitBreaker::new(2, 3, 50); // need 3 successes to close

        // Trip and wait for timeout
        cb.record_failure();
        cb.record_failure();
        std::thread::sleep(Duration::from_millis(60));
        assert!(cb.should_allow()); // transitions to HalfOpen

        // First two successes: still HalfOpen
        cb.record_success();
        assert_eq!(cb.get_state(), CircuitState::HalfOpen);
        cb.record_success();
        assert_eq!(cb.get_state(), CircuitState::HalfOpen);

        // Third success: closes circuit
        cb.record_success();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        assert!(cb.should_allow());

        // Failure count should be reset
        let stats = cb.stats();
        assert_eq!(stats.failure_count, 0);
        assert_eq!(stats.success_count, 0);
    }

    /// HalfOpen → Open: any failure during HalfOpen reopens the circuit.
    ///
    /// Robotics: sensor probe tick fails → still disconnected, reopen and wait.
    #[test]
    fn half_open_to_open_on_failure() {
        let cb = CircuitBreaker::new(2, 3, 50);

        // Trip and transition to HalfOpen
        cb.record_failure();
        cb.record_failure();
        std::thread::sleep(Duration::from_millis(60));
        assert!(cb.should_allow()); // HalfOpen

        // One success
        cb.record_success();
        assert_eq!(cb.get_state(), CircuitState::HalfOpen);

        // Then a failure: reopens
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Open);
        assert!(!cb.should_allow());
    }

    /// Full state machine cycle: Closed → Open → HalfOpen → Closed.
    /// Simulates sensor disconnect and reconnect.
    ///
    /// Robotics: LIDAR driver fails (cable loose) → opens → cooldown →
    /// half-open probe succeeds (cable plugged back in) → closes.
    #[test]
    fn full_circuit_breaker_cycle() {
        let cb = CircuitBreaker::new(3, 2, 50);

        // Phase 1: Closed, normal operation
        assert_eq!(cb.get_state(), CircuitState::Closed);
        cb.record_success();
        cb.record_success();
        assert_eq!(cb.get_state(), CircuitState::Closed);

        // Phase 2: Failures accumulate → Open
        cb.record_failure();
        cb.record_failure();
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Open);

        // Phase 3: Wait for cooldown → HalfOpen
        std::thread::sleep(Duration::from_millis(60));
        assert!(cb.should_allow());
        assert_eq!(cb.get_state(), CircuitState::HalfOpen);

        // Phase 4: Recovery succeeds → Closed
        cb.record_success();
        cb.record_success();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        assert!(cb.should_allow());

        // Phase 5: Verify fully reset — can accumulate failures again
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        assert_eq!(cb.stats().failure_count, 1);
    }

    /// Multiple open-close cycles: circuit breaker can cycle multiple times.
    ///
    /// Robotics: intermittent sensor connection over hours of operation.
    #[test]
    fn multiple_open_close_cycles() {
        let cb = CircuitBreaker::new(2, 1, 30); // fast timeout for test

        for cycle in 0..5 {
            // Trip
            cb.record_failure();
            cb.record_failure();
            assert_eq!(
                cb.get_state(),
                CircuitState::Open,
                "Cycle {}: should be Open",
                cycle
            );

            // Cooldown
            std::thread::sleep(Duration::from_millis(40));
            assert!(
                cb.should_allow(),
                "Cycle {}: should transition to HalfOpen",
                cycle
            );

            // Recover
            cb.record_success();
            assert_eq!(
                cb.get_state(),
                CircuitState::Closed,
                "Cycle {}: should be Closed after recovery",
                cycle
            );
        }
    }

    /// Success in Closed state resets failure count.
    /// Prevents stale failures from eventually tripping the breaker.
    ///
    /// Robotics: occasional sensor glitch doesn't accumulate if followed by success.
    #[test]
    fn success_in_closed_resets_failure_count() {
        let cb = CircuitBreaker::new(3, 1, 100);

        // Two failures
        cb.record_failure();
        cb.record_failure();
        assert_eq!(cb.stats().failure_count, 2);

        // Success resets failure count
        cb.record_success();
        assert_eq!(cb.stats().failure_count, 0);
        assert_eq!(cb.get_state(), CircuitState::Closed);

        // Need 3 more failures to trip (not just 1 more)
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Open);
    }

    /// Reset returns circuit breaker to initial Closed state.
    #[test]
    fn reset_returns_to_closed() {
        let cb = CircuitBreaker::new(2, 1, 100);

        // Trip the breaker
        cb.record_failure();
        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Open);

        // Reset
        cb.reset();
        assert_eq!(cb.get_state(), CircuitState::Closed);
        assert_eq!(cb.stats().failure_count, 0);
        assert_eq!(cb.stats().success_count, 0);
        assert!(cb.should_allow());
    }

    /// CircuitState::from(u8) maps correctly for all variants.
    #[test]
    fn circuit_state_from_u8() {
        assert_eq!(CircuitState::from(0), CircuitState::Closed);
        assert_eq!(CircuitState::from(1), CircuitState::Open);
        assert_eq!(CircuitState::from(2), CircuitState::HalfOpen);
        // Unknown values default to Closed
        assert_eq!(CircuitState::from(255), CircuitState::Closed);
        assert_eq!(CircuitState::from(3), CircuitState::Closed);
    }

    /// Open state doesn't allow before timeout.
    #[test]
    fn open_rejects_before_timeout() {
        let cb = CircuitBreaker::new(1, 1, 200); // 200ms timeout

        cb.record_failure();
        assert_eq!(cb.get_state(), CircuitState::Open);

        // Immediately: not allowed
        assert!(!cb.should_allow());

        // After short wait (< timeout): still not allowed
        std::thread::sleep(Duration::from_millis(50));
        assert!(!cb.should_allow());
        assert_eq!(cb.get_state(), CircuitState::Open);
    }
}
