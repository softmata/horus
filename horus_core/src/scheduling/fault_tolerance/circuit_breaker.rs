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
    /// Kept for manual recovery/debugging scenarios
    #[allow(dead_code)]
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
    pub success_count: u32,
}

impl Default for CircuitBreaker {
    fn default() -> Self {
        // Default: 5 failures to open, 3 successes to close, 5 second timeout
        Self::new(5, 3, 5000)
    }
}
