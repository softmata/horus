/// Connection state machine and reconnection logic for network transports
///
/// Production-grade connection handling with observable state transitions,
/// exponential backoff, and connection quality metrics.
///
/// # State Machine
///
/// ```text
///                    ┌─────────────────┐
///                    │  Disconnected   │◄──────────────────────────────┐
///                    └────────┬────────┘                               │
///                             │ connect()                              │
///                             ▼                                        │
///                    ┌─────────────────┐                               │
///             ┌──────│   Resolving     │──────┐                        │
///             │      └─────────────────┘      │                        │
///             │ resolved                      │ resolve_failed         │
///             ▼                               │                        │
///     ┌─────────────────┐                     │                        │
///     │  Connecting     │─────────────────────┼────────────────────────┤
///     └────────┬────────┘   connect_failed    │                        │
///              │ connected                    │                        │
///              ▼                              │                        │
///     ┌─────────────────┐                     │                        │
///     │   Connected     │◄────────────────────┤                        │
///     └────────┬────────┘                     │                        │
///              │ connection_lost              │                        │
///              ▼                              │                        │
///     ┌─────────────────┐   retry_succeeded   │                        │
///     │  Reconnecting   │─────────────────────┘                        │
///     └────────┬────────┘                                              │
///              │ max_retries_exceeded                                  │
///              ▼                                                       │
///     ┌─────────────────┐                                              │
///     │    Failed       │──────────────────────────────────────────────┘
///     └─────────────────┘   reset() or manual reconnect
/// ```
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

const INITIAL_BACKOFF: Duration = Duration::from_millis(100);
const MAX_BACKOFF: Duration = Duration::from_secs(30);
const BACKOFF_MULTIPLIER: f64 = 2.0;
const MAX_RETRIES: usize = 10; // 0 means infinite retries

// ============================================================================
// Connection State Machine
// ============================================================================

/// Connection states in the state machine
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionState {
    /// Not connected, initial state
    Disconnected,
    /// Resolving endpoint (DNS, mDNS, STUN, etc.)
    Resolving { endpoint: String },
    /// Establishing connection
    Connecting { endpoint: String, attempt: u32 },
    /// Connection established and healthy
    Connected {
        endpoint: String,
        connected_at: std::time::SystemTime,
    },
    /// Connection lost, attempting to reconnect
    Reconnecting {
        endpoint: String,
        attempt: u32,
        last_error: String,
    },
    /// Connection permanently failed
    Failed { reason: FailureReason },
}

impl ConnectionState {
    /// Get a human-readable name for the state
    pub fn name(&self) -> &'static str {
        match self {
            Self::Disconnected => "Disconnected",
            Self::Resolving { .. } => "Resolving",
            Self::Connecting { .. } => "Connecting",
            Self::Connected { .. } => "Connected",
            Self::Reconnecting { .. } => "Reconnecting",
            Self::Failed { .. } => "Failed",
        }
    }

    /// Check if the connection is usable for sending/receiving
    pub fn is_connected(&self) -> bool {
        matches!(self, Self::Connected { .. })
    }

    /// Check if the connection is in a transitional state
    pub fn is_transitioning(&self) -> bool {
        matches!(
            self,
            Self::Resolving { .. } | Self::Connecting { .. } | Self::Reconnecting { .. }
        )
    }

    /// Check if the connection has permanently failed
    pub fn is_failed(&self) -> bool {
        matches!(self, Self::Failed { .. })
    }
}

impl std::fmt::Display for ConnectionState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Disconnected => write!(f, "Disconnected"),
            Self::Resolving { endpoint } => write!(f, "Resolving {}", endpoint),
            Self::Connecting { endpoint, attempt } => {
                write!(f, "Connecting to {} (attempt #{})", endpoint, attempt)
            }
            Self::Connected { endpoint, .. } => write!(f, "Connected to {}", endpoint),
            Self::Reconnecting {
                endpoint,
                attempt,
                last_error,
            } => {
                write!(
                    f,
                    "Reconnecting to {} (attempt #{}, last error: {})",
                    endpoint, attempt, last_error
                )
            }
            Self::Failed { reason } => write!(f, "Failed: {}", reason),
        }
    }
}

/// Reason for connection failure
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FailureReason {
    /// Could not resolve endpoint (DNS, mDNS, etc.)
    ResolutionFailed { endpoint: String, error: String },
    /// Connection refused by peer
    ConnectionRefused { endpoint: String },
    /// Connection timed out
    Timeout { endpoint: String, timeout: Duration },
    /// Authentication failed
    AuthenticationFailed { endpoint: String, error: String },
    /// Maximum retry attempts exceeded
    MaxRetriesExceeded {
        endpoint: String,
        attempts: u32,
        last_error: String,
    },
    /// Network unreachable
    NetworkUnreachable { error: String },
    /// Peer explicitly closed connection
    PeerClosed { endpoint: String, reason: String },
    /// Protocol error
    ProtocolError { error: String },
    /// Shutdown requested by user
    Shutdown,
    /// Custom error
    Other { error: String },
}

impl std::fmt::Display for FailureReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ResolutionFailed { endpoint, error } => {
                write!(f, "Failed to resolve {}: {}", endpoint, error)
            }
            Self::ConnectionRefused { endpoint } => {
                write!(f, "Connection refused by {}", endpoint)
            }
            Self::Timeout { endpoint, timeout } => {
                write!(
                    f,
                    "Connection to {} timed out after {:?}",
                    endpoint, timeout
                )
            }
            Self::AuthenticationFailed { endpoint, error } => {
                write!(f, "Authentication failed for {}: {}", endpoint, error)
            }
            Self::MaxRetriesExceeded {
                endpoint,
                attempts,
                last_error,
            } => {
                write!(
                    f,
                    "Max retries ({}) exceeded for {}: {}",
                    attempts, endpoint, last_error
                )
            }
            Self::NetworkUnreachable { error } => write!(f, "Network unreachable: {}", error),
            Self::PeerClosed { endpoint, reason } => {
                write!(f, "Peer {} closed connection: {}", endpoint, reason)
            }
            Self::ProtocolError { error } => write!(f, "Protocol error: {}", error),
            Self::Shutdown => write!(f, "Shutdown requested"),
            Self::Other { error } => write!(f, "{}", error),
        }
    }
}

/// State transition event for observability
#[derive(Debug, Clone)]
pub struct StateTransition {
    /// Previous state
    pub from: ConnectionState,
    /// New state
    pub to: ConnectionState,
    /// When the transition occurred
    pub timestamp: Instant,
    /// Human-readable reason for the transition
    pub reason: String,
}

impl std::fmt::Display for StateTransition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} -> {} ({})",
            self.from.name(),
            self.to.name(),
            self.reason
        )
    }
}

/// Callback type for state change notifications
pub type StateChangeCallback = Box<dyn Fn(&StateTransition) + Send + Sync>;

/// Connection quality metrics
#[derive(Debug, Default)]
pub struct ConnectionQuality {
    /// Round-trip time in microseconds (smoothed average)
    pub rtt_us: AtomicU64,
    /// RTT variance in microseconds
    pub rtt_variance_us: AtomicU64,
    /// Packet loss percentage (0-100, scaled by 100 for precision)
    pub packet_loss_percent_x100: AtomicU32,
    /// Packets sent
    pub packets_sent: AtomicU64,
    /// Packets received
    pub packets_received: AtomicU64,
    /// Packets lost (detected via sequence gaps or timeouts)
    pub packets_lost: AtomicU64,
    /// Bytes sent
    pub bytes_sent: AtomicU64,
    /// Bytes received
    pub bytes_received: AtomicU64,
    /// Last successful communication
    last_activity: AtomicU64, // Epoch millis
}

impl Clone for ConnectionQuality {
    fn clone(&self) -> Self {
        Self {
            rtt_us: AtomicU64::new(self.rtt_us.load(Ordering::Relaxed)),
            rtt_variance_us: AtomicU64::new(self.rtt_variance_us.load(Ordering::Relaxed)),
            packet_loss_percent_x100: AtomicU32::new(
                self.packet_loss_percent_x100.load(Ordering::Relaxed),
            ),
            packets_sent: AtomicU64::new(self.packets_sent.load(Ordering::Relaxed)),
            packets_received: AtomicU64::new(self.packets_received.load(Ordering::Relaxed)),
            packets_lost: AtomicU64::new(self.packets_lost.load(Ordering::Relaxed)),
            bytes_sent: AtomicU64::new(self.bytes_sent.load(Ordering::Relaxed)),
            bytes_received: AtomicU64::new(self.bytes_received.load(Ordering::Relaxed)),
            last_activity: AtomicU64::new(self.last_activity.load(Ordering::Relaxed)),
        }
    }
}

impl ConnectionQuality {
    /// Create new connection quality metrics
    pub fn new() -> Self {
        Self::default()
    }

    /// Get RTT as Duration
    pub fn rtt(&self) -> Duration {
        Duration::from_micros(self.rtt_us.load(Ordering::Relaxed))
    }

    /// Get RTT variance as Duration
    pub fn rtt_variance(&self) -> Duration {
        Duration::from_micros(self.rtt_variance_us.load(Ordering::Relaxed))
    }

    /// Get packet loss as percentage (0.0 - 100.0)
    pub fn packet_loss(&self) -> f64 {
        self.packet_loss_percent_x100.load(Ordering::Relaxed) as f64 / 100.0
    }

    /// Update RTT with a new sample (uses exponential smoothing)
    pub fn update_rtt(&self, sample: Duration) {
        let sample_us = sample.as_micros() as u64;
        let current_rtt = self.rtt_us.load(Ordering::Relaxed);

        if current_rtt == 0 {
            // First sample
            self.rtt_us.store(sample_us, Ordering::Relaxed);
        } else {
            // Exponential smoothing: new = 0.875 * old + 0.125 * sample
            let smoothed = (current_rtt * 7 + sample_us) / 8;
            self.rtt_us.store(smoothed, Ordering::Relaxed);

            // Update variance: |sample - smoothed|
            let diff = sample_us.abs_diff(smoothed);
            let current_var = self.rtt_variance_us.load(Ordering::Relaxed);
            let new_var = (current_var * 3 + diff) / 4;
            self.rtt_variance_us.store(new_var, Ordering::Relaxed);
        }
    }

    /// Record a sent packet
    pub fn record_send(&self, bytes: u64) {
        self.packets_sent.fetch_add(1, Ordering::Relaxed);
        self.bytes_sent.fetch_add(bytes, Ordering::Relaxed);
        self.update_activity();
    }

    /// Record a received packet
    pub fn record_receive(&self, bytes: u64) {
        self.packets_received.fetch_add(1, Ordering::Relaxed);
        self.bytes_received.fetch_add(bytes, Ordering::Relaxed);
        self.update_activity();
        self.recalculate_packet_loss();
    }

    /// Record a lost packet
    pub fn record_loss(&self) {
        self.packets_lost.fetch_add(1, Ordering::Relaxed);
        self.recalculate_packet_loss();
    }

    /// Get time since last activity
    pub fn time_since_activity(&self) -> Duration {
        let last = self.last_activity.load(Ordering::Relaxed);
        if last == 0 {
            return Duration::MAX;
        }
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        Duration::from_millis(now.saturating_sub(last))
    }

    fn update_activity(&self) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        self.last_activity.store(now, Ordering::Relaxed);
    }

    fn recalculate_packet_loss(&self) {
        let sent = self.packets_sent.load(Ordering::Relaxed);
        let lost = self.packets_lost.load(Ordering::Relaxed);
        if sent > 0 {
            let loss_percent = (lost as f64 / sent as f64 * 10000.0) as u32;
            self.packet_loss_percent_x100
                .store(loss_percent.min(10000), Ordering::Relaxed);
        }
    }

    /// Get quality score (0-100, higher is better)
    pub fn quality_score(&self) -> u32 {
        let rtt_score = match self.rtt().as_millis() {
            0..=10 => 100,
            11..=50 => 80,
            51..=100 => 60,
            101..=200 => 40,
            201..=500 => 20,
            _ => 0,
        };

        let loss_score = match self.packet_loss() as u32 {
            0 => 100,
            1 => 80,
            2..=5 => 50,
            6..=10 => 20,
            _ => 0,
        };

        // Weighted average: 60% RTT, 40% loss
        (rtt_score * 60 + loss_score * 40) / 100
    }

    /// Get a quality description
    pub fn quality_description(&self) -> &'static str {
        match self.quality_score() {
            90..=100 => "Excellent",
            70..=89 => "Good",
            50..=69 => "Fair",
            30..=49 => "Poor",
            _ => "Critical",
        }
    }
}

/// Connection state machine with observable state transitions
pub struct ConnectionStateMachine {
    /// Current state
    state: RwLock<ConnectionState>,
    /// Connection quality metrics
    quality: Arc<ConnectionQuality>,
    /// Reconnection strategy
    strategy: ReconnectStrategy,
    /// State change callbacks
    callbacks: RwLock<Vec<StateChangeCallback>>,
    /// State transition history (most recent N)
    history: RwLock<Vec<StateTransition>>,
    /// Maximum history size
    max_history: usize,
}

impl ConnectionStateMachine {
    /// Create a new connection state machine
    pub fn new() -> Self {
        Self::with_strategy(ReconnectStrategy::default())
    }

    /// Create with a custom reconnection strategy
    pub fn with_strategy(strategy: ReconnectStrategy) -> Self {
        Self {
            state: RwLock::new(ConnectionState::Disconnected),
            quality: Arc::new(ConnectionQuality::new()),
            strategy,
            callbacks: RwLock::new(Vec::new()),
            history: RwLock::new(Vec::new()),
            max_history: 100,
        }
    }

    /// Get the current state
    pub fn state(&self) -> ConnectionState {
        self.state.read().unwrap().clone()
    }

    /// Get connection quality metrics
    pub fn quality(&self) -> Arc<ConnectionQuality> {
        Arc::clone(&self.quality)
    }

    /// Get the reconnection strategy
    pub fn strategy(&self) -> &ReconnectStrategy {
        &self.strategy
    }

    /// Register a callback for state changes
    pub fn on_state_change(&self, callback: StateChangeCallback) {
        self.callbacks.write().unwrap().push(callback);
    }

    /// Get recent state transitions
    pub fn history(&self) -> Vec<StateTransition> {
        self.history.read().unwrap().clone()
    }

    // State transition methods

    /// Transition to Resolving state
    pub fn start_resolve(&self, endpoint: &str) -> Result<(), String> {
        self.transition(
            ConnectionState::Resolving {
                endpoint: endpoint.to_string(),
            },
            format!("Starting resolution for {}", endpoint),
        )
    }

    /// Transition to Connecting state
    pub fn start_connect(&self, endpoint: &str, attempt: u32) -> Result<(), String> {
        self.transition(
            ConnectionState::Connecting {
                endpoint: endpoint.to_string(),
                attempt,
            },
            format!("Connecting to {} (attempt #{})", endpoint, attempt),
        )
    }

    /// Transition to Connected state
    pub fn mark_connected(&self, endpoint: &str) -> Result<(), String> {
        self.transition(
            ConnectionState::Connected {
                endpoint: endpoint.to_string(),
                connected_at: std::time::SystemTime::now(),
            },
            format!("Connected to {}", endpoint),
        )
    }

    /// Transition to Reconnecting state
    pub fn start_reconnect(&self, endpoint: &str, attempt: u32, error: &str) -> Result<(), String> {
        self.transition(
            ConnectionState::Reconnecting {
                endpoint: endpoint.to_string(),
                attempt,
                last_error: error.to_string(),
            },
            format!(
                "Connection lost, reconnecting (attempt #{}, error: {})",
                attempt, error
            ),
        )
    }

    /// Transition to Failed state
    pub fn mark_failed(&self, reason: FailureReason) -> Result<(), String> {
        let reason_str = reason.to_string();
        self.transition(
            ConnectionState::Failed { reason },
            format!("Connection failed: {}", reason_str),
        )
    }

    /// Reset to Disconnected state
    pub fn reset(&self) -> Result<(), String> {
        self.transition(
            ConnectionState::Disconnected,
            "Connection reset".to_string(),
        )
    }

    fn transition(&self, new_state: ConnectionState, reason: String) -> Result<(), String> {
        let mut state = self.state.write().unwrap();
        let old_state = state.clone();

        // Validate transition
        if !self.is_valid_transition(&old_state, &new_state) {
            return Err(format!(
                "Invalid state transition: {} -> {}",
                old_state.name(),
                new_state.name()
            ));
        }

        // Record transition
        let transition = StateTransition {
            from: old_state,
            to: new_state.clone(),
            timestamp: Instant::now(),
            reason,
        };

        // Update state
        *state = new_state;
        drop(state);

        // Add to history
        {
            let mut history = self.history.write().unwrap();
            history.push(transition.clone());
            if history.len() > self.max_history {
                history.remove(0);
            }
        }

        // Notify callbacks
        let callbacks = self.callbacks.read().unwrap();
        for callback in callbacks.iter() {
            callback(&transition);
        }

        Ok(())
    }

    fn is_valid_transition(&self, from: &ConnectionState, to: &ConnectionState) -> bool {
        use ConnectionState::*;
        matches!(
            (from, to),
            // From Disconnected
            (Disconnected, Resolving { .. })
                | (Disconnected, Connecting { .. })
                |
            // From Resolving
            (Resolving { .. }, Connecting { .. })
                | (Resolving { .. }, Failed { .. })
                | (Resolving { .. }, Disconnected)
                |
            // From Connecting
            (Connecting { .. }, Connected { .. })
                | (Connecting { .. }, Reconnecting { .. })
                | (Connecting { .. }, Failed { .. })
                | (Connecting { .. }, Disconnected)
                |
            // From Connected
            (Connected { .. }, Reconnecting { .. })
                | (Connected { .. }, Disconnected)
                | (Connected { .. }, Failed { .. })
                |
            // From Reconnecting
            (Reconnecting { .. }, Connected { .. })
                | (Reconnecting { .. }, Reconnecting { .. })
                | (Reconnecting { .. }, Failed { .. })
                | (Reconnecting { .. }, Disconnected)
                |
            // From Failed
            (Failed { .. }, Disconnected)
                | (Failed { .. }, Resolving { .. })
                | (Failed { .. }, Connecting { .. })
        )
    }
}

impl Default for ConnectionStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Original Reconnection Strategy (enhanced)
// ============================================================================

/// Reconnection strategy with exponential backoff
#[derive(Debug, Clone)]
pub struct ReconnectStrategy {
    pub initial_backoff: Duration,
    pub max_backoff: Duration,
    pub multiplier: f64,
    pub max_retries: usize, // 0 = infinite
    pub jitter: bool,       // Add random jitter to avoid thundering herd
}

impl Default for ReconnectStrategy {
    fn default() -> Self {
        Self {
            initial_backoff: INITIAL_BACKOFF,
            max_backoff: MAX_BACKOFF,
            multiplier: BACKOFF_MULTIPLIER,
            max_retries: MAX_RETRIES,
            jitter: true,
        }
    }
}

impl ReconnectStrategy {
    /// Create a new reconnection strategy
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a strategy for production use (longer backoffs, infinite retries)
    pub fn production() -> Self {
        Self {
            initial_backoff: Duration::from_millis(500),
            max_backoff: Duration::from_secs(60),
            multiplier: 2.0,
            max_retries: 0, // Never give up
            jitter: true,
        }
    }

    /// Create a strategy for testing (short backoffs, limited retries)
    pub fn testing() -> Self {
        Self {
            initial_backoff: Duration::from_millis(10),
            max_backoff: Duration::from_millis(500),
            multiplier: 1.5,
            max_retries: 3,
            jitter: false,
        }
    }

    /// Calculate backoff delay for the given attempt
    pub fn backoff_delay(&self, attempt: usize) -> Duration {
        if attempt == 0 {
            return Duration::ZERO;
        }

        // Calculate exponential backoff
        let delay_ms =
            self.initial_backoff.as_millis() as f64 * self.multiplier.powi((attempt - 1) as i32);

        let delay = Duration::from_millis(delay_ms as u64);
        let capped_delay = delay.min(self.max_backoff);

        // Add jitter (±20%) to avoid thundering herd
        if self.jitter {
            use std::collections::hash_map::RandomState;
            use std::hash::BuildHasher;

            let hash_value = RandomState::new().hash_one(std::thread::current().id());
            let jitter_factor = 0.8 + (hash_value % 40) as f64 / 100.0; // 0.8 to 1.2

            Duration::from_millis((capped_delay.as_millis() as f64 * jitter_factor) as u64)
        } else {
            capped_delay
        }
    }

    /// Check if we should retry after this many attempts
    pub fn should_retry(&self, attempt: usize) -> bool {
        self.max_retries == 0 || attempt < self.max_retries
    }
}

/// Connection state tracker for monitoring
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionHealth {
    Connected,
    Reconnecting { attempt: usize },
    Failed { attempts: usize },
}

/// Reconnection context that tracks state
pub struct ReconnectContext {
    pub strategy: ReconnectStrategy,
    pub attempt: usize,
    pub health: ConnectionHealth,
}

impl ReconnectContext {
    pub fn new(strategy: ReconnectStrategy) -> Self {
        Self {
            strategy,
            attempt: 0,
            health: ConnectionHealth::Connected,
        }
    }

    /// Start a reconnection attempt
    pub fn begin_reconnect(&mut self) {
        self.attempt += 1;
        self.health = ConnectionHealth::Reconnecting {
            attempt: self.attempt,
        };
    }

    /// Mark connection as successful (resets attempt counter)
    pub fn mark_connected(&mut self) {
        self.attempt = 0;
        self.health = ConnectionHealth::Connected;
    }

    /// Mark connection as failed
    pub fn mark_failed(&mut self) {
        self.health = ConnectionHealth::Failed {
            attempts: self.attempt,
        };
    }

    /// Get the current backoff delay
    pub fn backoff_delay(&self) -> Duration {
        self.strategy.backoff_delay(self.attempt)
    }

    /// Check if we should continue retrying
    pub fn should_retry(&self) -> bool {
        self.strategy.should_retry(self.attempt)
    }

    /// Wait for the backoff period
    pub fn wait_backoff(&self) {
        let delay = self.backoff_delay();
        if !delay.is_zero() {
            std::thread::sleep(delay);
        }
    }
}

// ============================================================================
// Retry Executor with Cancellation Support
// ============================================================================

/// Token for cancelling retry operations
#[derive(Debug, Clone)]
pub struct CancellationToken {
    cancelled: Arc<std::sync::atomic::AtomicBool>,
}

impl CancellationToken {
    /// Create a new cancellation token
    pub fn new() -> Self {
        Self {
            cancelled: Arc::new(std::sync::atomic::AtomicBool::new(false)),
        }
    }

    /// Cancel the operation
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::SeqCst);
    }

    /// Check if cancelled
    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::SeqCst)
    }

    /// Reset the token (for reuse)
    pub fn reset(&self) {
        self.cancelled.store(false, Ordering::SeqCst);
    }
}

impl Default for CancellationToken {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of a retry operation
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RetryResult<T> {
    /// Operation succeeded
    Success(T),
    /// Operation was cancelled
    Cancelled,
    /// All retries exhausted
    Exhausted { attempts: usize, last_error: String },
    /// Operation timed out
    TimedOut {
        elapsed: Duration,
        timeout: Duration,
    },
}

impl<T> RetryResult<T> {
    /// Check if the result is a success
    pub fn is_success(&self) -> bool {
        matches!(self, Self::Success(_))
    }

    /// Get the success value if present
    pub fn success(self) -> Option<T> {
        match self {
            Self::Success(v) => Some(v),
            _ => None,
        }
    }

    /// Convert to Result
    pub fn into_result(self) -> Result<T, String> {
        match self {
            Self::Success(v) => Ok(v),
            Self::Cancelled => Err("Operation cancelled".to_string()),
            Self::Exhausted {
                attempts,
                last_error,
            } => Err(format!(
                "Exhausted after {} attempts: {}",
                attempts, last_error
            )),
            Self::TimedOut { elapsed, timeout } => Err(format!(
                "Timed out after {:?} (timeout: {:?})",
                elapsed, timeout
            )),
        }
    }
}

/// Retry event for logging/monitoring
#[derive(Debug, Clone)]
pub struct RetryEvent {
    /// Current attempt number (1-indexed)
    pub attempt: usize,
    /// Maximum attempts (0 = infinite)
    pub max_attempts: usize,
    /// Error from this attempt
    pub error: String,
    /// How long until next retry
    pub next_retry_in: Duration,
    /// Total elapsed time
    pub elapsed: Duration,
}

impl std::fmt::Display for RetryEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.max_attempts == 0 {
            write!(
                f,
                "Retry attempt {} failed: {}. Retrying in {:?}...",
                self.attempt, self.error, self.next_retry_in
            )
        } else {
            write!(
                f,
                "Retry attempt {}/{} failed: {}. Retrying in {:?}...",
                self.attempt, self.max_attempts, self.error, self.next_retry_in
            )
        }
    }
}

/// Callback type for retry events
pub type RetryCallback = Box<dyn Fn(&RetryEvent) + Send + Sync>;

/// Retry executor with cancellation support and logging
pub struct RetryExecutor {
    strategy: ReconnectStrategy,
    timeout: Option<Duration>,
    cancel_token: Option<CancellationToken>,
    on_retry: Option<RetryCallback>,
    operation_name: String,
}

impl RetryExecutor {
    /// Create a new retry executor with default settings
    pub fn new() -> Self {
        Self {
            strategy: ReconnectStrategy::default(),
            timeout: None,
            cancel_token: None,
            on_retry: None,
            operation_name: "operation".to_string(),
        }
    }

    /// Create with production settings (longer backoffs, infinite retries)
    pub fn production() -> Self {
        Self {
            strategy: ReconnectStrategy::production(),
            timeout: None,
            cancel_token: None,
            on_retry: None,
            operation_name: "operation".to_string(),
        }
    }

    /// Create with test settings (short backoffs, limited retries)
    pub fn testing() -> Self {
        Self {
            strategy: ReconnectStrategy::testing(),
            timeout: None,
            cancel_token: None,
            on_retry: None,
            operation_name: "operation".to_string(),
        }
    }

    /// Set the operation name for logging
    pub fn name(mut self, name: impl Into<String>) -> Self {
        self.operation_name = name.into();
        self
    }

    /// Set custom retry strategy
    pub fn strategy(mut self, strategy: ReconnectStrategy) -> Self {
        self.strategy = strategy;
        self
    }

    /// Set overall timeout for all retry attempts
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    /// Set cancellation token
    pub fn cancel_token(mut self, token: CancellationToken) -> Self {
        self.cancel_token = Some(token);
        self
    }

    /// Set retry callback for logging/monitoring
    pub fn on_retry(mut self, callback: RetryCallback) -> Self {
        self.on_retry = Some(callback);
        self
    }

    /// Set maximum number of retries
    pub fn max_retries(mut self, max: usize) -> Self {
        self.strategy.max_retries = max;
        self
    }

    /// Set initial backoff delay
    pub fn initial_backoff(mut self, delay: Duration) -> Self {
        self.strategy.initial_backoff = delay;
        self
    }

    /// Set maximum backoff delay
    pub fn max_backoff(mut self, delay: Duration) -> Self {
        self.strategy.max_backoff = delay;
        self
    }

    /// Execute an operation with retry logic (blocking)
    ///
    /// # Example
    /// ```ignore
    /// let result = RetryExecutor::new()
    ///     .max_retries(5)
    ///     .execute(|| connect_to_server());
    /// ```
    pub fn execute<T, E, F>(&self, mut operation: F) -> RetryResult<T>
    where
        E: std::fmt::Display,
        F: FnMut() -> Result<T, E>,
    {
        let start = Instant::now();
        let mut attempt = 0usize;
        #[allow(unused_assignments)]
        let mut last_error = String::new();

        loop {
            // Check cancellation
            if let Some(ref token) = self.cancel_token {
                if token.is_cancelled() {
                    return RetryResult::Cancelled;
                }
            }

            // Check timeout
            if let Some(timeout) = self.timeout {
                if start.elapsed() >= timeout {
                    return RetryResult::TimedOut {
                        elapsed: start.elapsed(),
                        timeout,
                    };
                }
            }

            attempt += 1;

            // Try the operation
            match operation() {
                Ok(result) => return RetryResult::Success(result),
                Err(e) => {
                    last_error = e.to_string();

                    // Check if we should retry
                    if !self.strategy.should_retry(attempt) {
                        return RetryResult::Exhausted {
                            attempts: attempt,
                            last_error,
                        };
                    }

                    // Calculate backoff
                    let backoff = self.strategy.backoff_delay(attempt);

                    // Fire retry callback
                    if let Some(ref callback) = self.on_retry {
                        callback(&RetryEvent {
                            attempt,
                            max_attempts: self.strategy.max_retries,
                            error: last_error.clone(),
                            next_retry_in: backoff,
                            elapsed: start.elapsed(),
                        });
                    }

                    // Check timeout with backoff
                    if let Some(timeout) = self.timeout {
                        if start.elapsed() + backoff >= timeout {
                            return RetryResult::TimedOut {
                                elapsed: start.elapsed(),
                                timeout,
                            };
                        }
                    }

                    // Wait with cancellation check
                    self.interruptible_sleep(backoff);
                }
            }
        }
    }

    /// Execute with a state machine for observability
    pub fn execute_with_state_machine<T, E, F>(
        &self,
        endpoint: &str,
        state_machine: &ConnectionStateMachine,
        mut operation: F,
    ) -> RetryResult<T>
    where
        E: std::fmt::Display,
        F: FnMut() -> Result<T, E>,
    {
        let start = Instant::now();
        let mut attempt = 0u32;
        let mut last_error = String::new();

        loop {
            // Check cancellation
            if let Some(ref token) = self.cancel_token {
                if token.is_cancelled() {
                    let _ = state_machine.reset();
                    return RetryResult::Cancelled;
                }
            }

            // Check timeout
            if let Some(timeout) = self.timeout {
                if start.elapsed() >= timeout {
                    let _ = state_machine.mark_failed(FailureReason::Timeout {
                        endpoint: endpoint.to_string(),
                        timeout,
                    });
                    return RetryResult::TimedOut {
                        elapsed: start.elapsed(),
                        timeout,
                    };
                }
            }

            attempt += 1;

            // Update state machine
            if attempt == 1 {
                let _ = state_machine.start_connect(endpoint, attempt);
            } else {
                let _ = state_machine.start_reconnect(endpoint, attempt, &last_error);
            }

            // Try the operation
            match operation() {
                Ok(result) => {
                    let _ = state_machine.mark_connected(endpoint);
                    return RetryResult::Success(result);
                }
                Err(e) => {
                    last_error = e.to_string();

                    // Check if we should retry
                    if !self.strategy.should_retry(attempt as usize) {
                        let _ = state_machine.mark_failed(FailureReason::MaxRetriesExceeded {
                            endpoint: endpoint.to_string(),
                            attempts: attempt,
                            last_error: last_error.clone(),
                        });
                        return RetryResult::Exhausted {
                            attempts: attempt as usize,
                            last_error,
                        };
                    }

                    // Calculate backoff
                    let backoff = self.strategy.backoff_delay(attempt as usize);

                    // Fire retry callback
                    if let Some(ref callback) = self.on_retry {
                        callback(&RetryEvent {
                            attempt: attempt as usize,
                            max_attempts: self.strategy.max_retries,
                            error: last_error.clone(),
                            next_retry_in: backoff,
                            elapsed: start.elapsed(),
                        });
                    }

                    // Wait with cancellation check
                    self.interruptible_sleep(backoff);
                }
            }
        }
    }

    /// Sleep that can be interrupted by cancellation
    fn interruptible_sleep(&self, duration: Duration) {
        if duration.is_zero() {
            return;
        }

        // Sleep in small chunks to check for cancellation
        let chunk_ms = 50;
        let chunks = duration.as_millis() / chunk_ms;
        let remainder = duration.as_millis() % chunk_ms;

        for _ in 0..chunks {
            if let Some(ref token) = self.cancel_token {
                if token.is_cancelled() {
                    return;
                }
            }
            std::thread::sleep(Duration::from_millis(chunk_ms as u64));
        }

        if remainder > 0 {
            std::thread::sleep(Duration::from_millis(remainder as u64));
        }
    }
}

impl Default for RetryExecutor {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience function for simple retry with default settings
pub fn retry<T, E, F>(operation: F) -> RetryResult<T>
where
    E: std::fmt::Display,
    F: FnMut() -> Result<T, E>,
{
    RetryExecutor::new().execute(operation)
}

/// Convenience function for retry with logging
pub fn retry_with_logging<T, E, F>(operation_name: &str, operation: F) -> RetryResult<T>
where
    E: std::fmt::Display,
    F: FnMut() -> Result<T, E>,
{
    RetryExecutor::new()
        .name(operation_name)
        .on_retry(Box::new(|event| {
            eprintln!("[RETRY] {}", event);
        }))
        .execute(operation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    // ========================================================================
    // Connection State Machine Tests
    // ========================================================================

    #[test]
    fn test_connection_state_names() {
        assert_eq!(ConnectionState::Disconnected.name(), "Disconnected");
        assert_eq!(
            ConnectionState::Resolving {
                endpoint: "test".to_string()
            }
            .name(),
            "Resolving"
        );
        assert_eq!(
            ConnectionState::Connecting {
                endpoint: "test".to_string(),
                attempt: 1
            }
            .name(),
            "Connecting"
        );
        assert_eq!(
            ConnectionState::Connected {
                endpoint: "test".to_string(),
                connected_at: std::time::SystemTime::now()
            }
            .name(),
            "Connected"
        );
        assert_eq!(
            ConnectionState::Reconnecting {
                endpoint: "test".to_string(),
                attempt: 1,
                last_error: "timeout".to_string()
            }
            .name(),
            "Reconnecting"
        );
        assert_eq!(
            ConnectionState::Failed {
                reason: FailureReason::Shutdown
            }
            .name(),
            "Failed"
        );
    }

    #[test]
    fn test_connection_state_display() {
        let state = ConnectionState::Connecting {
            endpoint: "192.168.1.1:8080".to_string(),
            attempt: 3,
        };
        let display = format!("{}", state);
        assert!(display.contains("192.168.1.1:8080"));
        assert!(display.contains("attempt #3"));
    }

    #[test]
    fn test_connection_state_predicates() {
        let disconnected = ConnectionState::Disconnected;
        assert!(!disconnected.is_connected());
        assert!(!disconnected.is_transitioning());
        assert!(!disconnected.is_failed());

        let resolving = ConnectionState::Resolving {
            endpoint: "test".to_string(),
        };
        assert!(!resolving.is_connected());
        assert!(resolving.is_transitioning());
        assert!(!resolving.is_failed());

        let connected = ConnectionState::Connected {
            endpoint: "test".to_string(),
            connected_at: std::time::SystemTime::now(),
        };
        assert!(connected.is_connected());
        assert!(!connected.is_transitioning());
        assert!(!connected.is_failed());

        let failed = ConnectionState::Failed {
            reason: FailureReason::Shutdown,
        };
        assert!(!failed.is_connected());
        assert!(!failed.is_transitioning());
        assert!(failed.is_failed());
    }

    #[test]
    fn test_failure_reason_display() {
        let reason = FailureReason::MaxRetriesExceeded {
            endpoint: "192.168.1.1".to_string(),
            attempts: 5,
            last_error: "connection refused".to_string(),
        };
        let display = format!("{}", reason);
        assert!(display.contains("5"));
        assert!(display.contains("192.168.1.1"));
        assert!(display.contains("connection refused"));
    }

    #[test]
    fn test_state_machine_happy_path() {
        let sm = ConnectionStateMachine::new();
        assert!(matches!(sm.state(), ConnectionState::Disconnected));

        // Disconnected -> Resolving
        sm.start_resolve("robot.local").unwrap();
        assert!(matches!(sm.state(), ConnectionState::Resolving { .. }));

        // Resolving -> Connecting
        sm.start_connect("192.168.1.100:8080", 1).unwrap();
        assert!(matches!(sm.state(), ConnectionState::Connecting { .. }));

        // Connecting -> Connected
        sm.mark_connected("192.168.1.100:8080").unwrap();
        assert!(matches!(sm.state(), ConnectionState::Connected { .. }));

        // Verify history
        let history = sm.history();
        assert_eq!(history.len(), 3);
        assert_eq!(history[0].from.name(), "Disconnected");
        assert_eq!(history[0].to.name(), "Resolving");
    }

    #[test]
    fn test_state_machine_reconnection_flow() {
        let sm = ConnectionStateMachine::new();

        // Get to connected state
        sm.start_connect("192.168.1.100:8080", 1).unwrap();
        sm.mark_connected("192.168.1.100:8080").unwrap();
        assert!(sm.state().is_connected());

        // Connection lost -> Reconnecting
        sm.start_reconnect("192.168.1.100:8080", 1, "connection reset")
            .unwrap();
        assert!(matches!(sm.state(), ConnectionState::Reconnecting { .. }));

        // Reconnect succeeds
        sm.mark_connected("192.168.1.100:8080").unwrap();
        assert!(sm.state().is_connected());
    }

    #[test]
    fn test_state_machine_failure_flow() {
        let sm = ConnectionStateMachine::new();

        // Try to connect
        sm.start_connect("192.168.1.100:8080", 1).unwrap();

        // Connection fails permanently
        sm.mark_failed(FailureReason::MaxRetriesExceeded {
            endpoint: "192.168.1.100:8080".to_string(),
            attempts: 5,
            last_error: "connection refused".to_string(),
        })
        .unwrap();
        assert!(sm.state().is_failed());

        // Reset and try again
        sm.reset().unwrap();
        assert!(matches!(sm.state(), ConnectionState::Disconnected));

        // Can reconnect from Failed after reset
        sm.start_connect("192.168.1.100:8080", 1).unwrap();
        assert!(matches!(sm.state(), ConnectionState::Connecting { .. }));
    }

    #[test]
    fn test_state_machine_invalid_transition() {
        let sm = ConnectionStateMachine::new();

        // Can't go directly from Disconnected to Connected
        let result = sm.mark_connected("test");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid state transition"));
    }

    #[test]
    fn test_state_machine_callback() {
        let sm = ConnectionStateMachine::new();
        let callback_count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&callback_count);

        sm.on_state_change(Box::new(move |_transition| {
            count_clone.fetch_add(1, Ordering::SeqCst);
        }));

        sm.start_connect("test", 1).unwrap();
        sm.mark_connected("test").unwrap();

        assert_eq!(callback_count.load(Ordering::SeqCst), 2);
    }

    // ========================================================================
    // Connection Quality Tests
    // ========================================================================

    #[test]
    fn test_connection_quality_rtt() {
        let quality = ConnectionQuality::new();

        // First sample sets RTT directly
        quality.update_rtt(Duration::from_millis(10));
        assert_eq!(quality.rtt().as_millis(), 10);

        // Subsequent samples are smoothed
        quality.update_rtt(Duration::from_millis(20));
        // Smoothed: (10 * 7 + 20) / 8 = 11.25 -> 11ms
        let rtt = quality.rtt().as_millis();
        assert!(rtt >= 10 && rtt <= 15);
    }

    #[test]
    fn test_connection_quality_packet_tracking() {
        let quality = ConnectionQuality::new();

        // Send some packets
        for _ in 0..100 {
            quality.record_send(1000);
        }
        assert_eq!(quality.packets_sent.load(Ordering::Relaxed), 100);
        assert_eq!(quality.bytes_sent.load(Ordering::Relaxed), 100_000);

        // Receive most of them
        for _ in 0..95 {
            quality.record_receive(1000);
        }
        assert_eq!(quality.packets_received.load(Ordering::Relaxed), 95);

        // Record lost packets
        for _ in 0..5 {
            quality.record_loss();
        }

        // Packet loss should be 5%
        let loss = quality.packet_loss();
        assert!((loss - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_connection_quality_score() {
        let quality = ConnectionQuality::new();

        // Low RTT, no loss = excellent
        quality.rtt_us.store(5000, Ordering::Relaxed); // 5ms
        quality.packet_loss_percent_x100.store(0, Ordering::Relaxed);
        assert!(quality.quality_score() >= 90);
        assert_eq!(quality.quality_description(), "Excellent");

        // High RTT, high loss = poor/critical
        quality.rtt_us.store(300_000, Ordering::Relaxed); // 300ms
        quality
            .packet_loss_percent_x100
            .store(1500, Ordering::Relaxed); // 15%
        assert!(quality.quality_score() < 30);
    }

    #[test]
    fn test_connection_quality_activity() {
        let quality = ConnectionQuality::new();

        // No activity yet
        assert_eq!(quality.time_since_activity(), Duration::MAX);

        // Record some activity
        quality.record_send(100);
        let since = quality.time_since_activity();
        assert!(since < Duration::from_secs(1));
    }

    // ========================================================================
    // Original Reconnection Strategy Tests
    // ========================================================================

    #[test]
    fn test_backoff_increases() {
        let strategy = ReconnectStrategy::testing();

        let delay1 = strategy.backoff_delay(1);
        let delay2 = strategy.backoff_delay(2);
        let delay3 = strategy.backoff_delay(3);

        assert!(delay2 > delay1);
        assert!(delay3 > delay2);
    }

    #[test]
    fn test_backoff_caps_at_max() {
        let strategy = ReconnectStrategy {
            initial_backoff: Duration::from_secs(1),
            max_backoff: Duration::from_secs(5),
            multiplier: 2.0,
            max_retries: 0,
            jitter: false,
        };

        let delay = strategy.backoff_delay(100); // Very high attempt
        assert!(delay <= Duration::from_secs(5));
    }

    #[test]
    fn test_max_retries() {
        let strategy = ReconnectStrategy {
            initial_backoff: INITIAL_BACKOFF,
            max_backoff: MAX_BACKOFF,
            multiplier: BACKOFF_MULTIPLIER,
            max_retries: 3,
            jitter: false,
        };

        assert!(strategy.should_retry(0));
        assert!(strategy.should_retry(1));
        assert!(strategy.should_retry(2));
        assert!(!strategy.should_retry(3));
        assert!(!strategy.should_retry(4));
    }

    #[test]
    fn test_infinite_retries() {
        let strategy = ReconnectStrategy::production();

        assert!(strategy.should_retry(0));
        assert!(strategy.should_retry(100));
        assert!(strategy.should_retry(1000));
    }

    #[test]
    fn test_context_state_transitions() {
        let mut ctx = ReconnectContext::new(ReconnectStrategy::testing());

        assert_eq!(ctx.health, ConnectionHealth::Connected);
        assert_eq!(ctx.attempt, 0);

        ctx.begin_reconnect();
        assert_eq!(ctx.health, ConnectionHealth::Reconnecting { attempt: 1 });
        assert_eq!(ctx.attempt, 1);

        ctx.mark_connected();
        assert_eq!(ctx.health, ConnectionHealth::Connected);
        assert_eq!(ctx.attempt, 0);

        ctx.begin_reconnect();
        ctx.mark_failed();
        assert_eq!(ctx.health, ConnectionHealth::Failed { attempts: 1 });
    }

    // ========================================================================
    // Retry Executor Tests
    // ========================================================================

    #[test]
    fn test_retry_executor_success_first_try() {
        let result: RetryResult<i32> = RetryExecutor::testing().execute(|| Ok::<i32, &str>(42));

        assert!(result.is_success());
        assert_eq!(result.success(), Some(42));
    }

    #[test]
    fn test_retry_executor_success_after_failures() {
        let attempt_count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&attempt_count);

        let result: RetryResult<i32> = RetryExecutor::testing().max_retries(5).execute(move || {
            let attempt = count_clone.fetch_add(1, Ordering::SeqCst) + 1;
            if attempt < 3 {
                Err(format!("Attempt {} failed", attempt))
            } else {
                Ok(42)
            }
        });

        assert!(result.is_success());
        assert_eq!(result.success(), Some(42));
        assert_eq!(attempt_count.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn test_retry_executor_exhausted() {
        let attempt_count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&attempt_count);

        let result: RetryResult<i32> = RetryExecutor::testing().max_retries(3).execute(move || {
            count_clone.fetch_add(1, Ordering::SeqCst);
            Err::<i32, _>("always fails")
        });

        assert!(matches!(result, RetryResult::Exhausted { .. }));
        if let RetryResult::Exhausted {
            attempts,
            last_error,
        } = result
        {
            assert_eq!(attempts, 3);
            assert_eq!(last_error, "always fails");
        }
    }

    #[test]
    fn test_retry_executor_timeout() {
        let result: RetryResult<i32> = RetryExecutor::testing()
            .timeout(Duration::from_millis(100))
            .initial_backoff(Duration::from_millis(50))
            .max_retries(0) // Infinite retries
            .execute(|| Err::<i32, _>("always fails"));

        assert!(matches!(result, RetryResult::TimedOut { .. }));
    }

    #[test]
    fn test_retry_executor_cancellation() {
        let cancel_token = CancellationToken::new();
        let token_clone = cancel_token.clone();
        let attempt_count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&attempt_count);

        // Cancel after 2 attempts in a separate thread
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(75));
            token_clone.cancel();
        });

        let result: RetryResult<i32> = RetryExecutor::testing()
            .cancel_token(cancel_token)
            .initial_backoff(Duration::from_millis(50))
            .max_retries(0)
            .execute(move || {
                count_clone.fetch_add(1, Ordering::SeqCst);
                Err::<i32, _>("always fails")
            });

        assert!(matches!(result, RetryResult::Cancelled));
    }

    #[test]
    fn test_retry_executor_callback() {
        let events = Arc::new(RwLock::new(Vec::new()));
        let events_clone = Arc::clone(&events);
        let attempt_count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&attempt_count);

        let _result: RetryResult<i32> = RetryExecutor::testing()
            .max_retries(3)
            .on_retry(Box::new(move |event| {
                events_clone.write().unwrap().push(event.attempt);
            }))
            .execute(move || {
                let attempt = count_clone.fetch_add(1, Ordering::SeqCst) + 1;
                if attempt < 3 {
                    Err(format!("Attempt {} failed", attempt))
                } else {
                    Ok(42)
                }
            });

        let recorded = events.read().unwrap();
        // Callback fires on attempts 1 and 2 (before success on attempt 3)
        assert_eq!(recorded.len(), 2);
        assert_eq!(recorded[0], 1);
        assert_eq!(recorded[1], 2);
    }

    #[test]
    fn test_retry_executor_with_state_machine() {
        let sm = ConnectionStateMachine::new();
        let attempt_count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&attempt_count);

        let result: RetryResult<i32> = RetryExecutor::testing()
            .max_retries(5)
            .execute_with_state_machine("192.168.1.100:8080", &sm, move || {
                let attempt = count_clone.fetch_add(1, Ordering::SeqCst) + 1;
                if attempt < 2 {
                    Err(format!("Attempt {} failed", attempt))
                } else {
                    Ok(42)
                }
            });

        assert!(result.is_success());
        assert!(sm.state().is_connected());

        // Check history includes state transitions
        let history = sm.history();
        assert!(!history.is_empty());
    }

    #[test]
    fn test_retry_executor_state_machine_failure() {
        let sm = ConnectionStateMachine::new();

        let result: RetryResult<i32> = RetryExecutor::testing()
            .max_retries(2)
            .execute_with_state_machine("192.168.1.100:8080", &sm, || {
                Err::<i32, _>("always fails")
            });

        assert!(matches!(result, RetryResult::Exhausted { .. }));
        assert!(sm.state().is_failed());

        if let ConnectionState::Failed { reason } = sm.state() {
            assert!(matches!(reason, FailureReason::MaxRetriesExceeded { .. }));
        }
    }

    #[test]
    fn test_cancellation_token() {
        let token = CancellationToken::new();
        assert!(!token.is_cancelled());

        token.cancel();
        assert!(token.is_cancelled());

        token.reset();
        assert!(!token.is_cancelled());
    }

    #[test]
    fn test_retry_result_into_result() {
        let success: RetryResult<i32> = RetryResult::Success(42);
        assert_eq!(success.into_result(), Ok(42));

        let cancelled: RetryResult<i32> = RetryResult::Cancelled;
        assert!(cancelled.into_result().is_err());

        let exhausted: RetryResult<i32> = RetryResult::Exhausted {
            attempts: 3,
            last_error: "failed".to_string(),
        };
        let err = exhausted.into_result().unwrap_err();
        assert!(err.contains("Exhausted"));
        assert!(err.contains("3"));

        let timed_out: RetryResult<i32> = RetryResult::TimedOut {
            elapsed: Duration::from_secs(1),
            timeout: Duration::from_secs(1),
        };
        assert!(timed_out.into_result().is_err());
    }

    #[test]
    fn test_retry_event_display() {
        let event = RetryEvent {
            attempt: 2,
            max_attempts: 5,
            error: "connection refused".to_string(),
            next_retry_in: Duration::from_millis(200),
            elapsed: Duration::from_millis(300),
        };

        let display = format!("{}", event);
        assert!(display.contains("2/5"));
        assert!(display.contains("connection refused"));
        assert!(display.contains("200ms"));
    }

    #[test]
    fn test_retry_event_display_infinite() {
        let event = RetryEvent {
            attempt: 10,
            max_attempts: 0,
            error: "timeout".to_string(),
            next_retry_in: Duration::from_secs(1),
            elapsed: Duration::from_secs(5),
        };

        let display = format!("{}", event);
        assert!(display.contains("attempt 10")); // No /N since infinite
        assert!(!display.contains("/0"));
    }

    #[test]
    fn test_convenience_retry_functions() {
        // Test simple retry function
        let result = retry(|| Ok::<i32, &str>(42));
        assert!(result.is_success());

        // Test retry with logging (just verify it compiles and works)
        let result = retry_with_logging("test_operation", || Ok::<i32, &str>(42));
        assert!(result.is_success());
    }

    #[test]
    fn test_retry_executor_builder_pattern() {
        let token = CancellationToken::new();

        let executor = RetryExecutor::new()
            .name("test_op")
            .max_retries(5)
            .initial_backoff(Duration::from_millis(100))
            .max_backoff(Duration::from_secs(10))
            .timeout(Duration::from_secs(30))
            .cancel_token(token);

        // Verify we can still execute
        let result: RetryResult<i32> = executor.execute(|| Ok::<i32, &str>(42));
        assert!(result.is_success());
    }
}
