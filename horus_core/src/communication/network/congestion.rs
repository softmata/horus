//! Congestion control for network backends
//!
//! Prevents sender from flooding the network with configurable drop policies,
//! rate limiting, and backpressure mechanisms.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Default send buffer size (number of messages)
const DEFAULT_BUFFER_SIZE: usize = 1000;
/// Default rate limit (messages per second, 0 = unlimited)
const DEFAULT_RATE_LIMIT: u64 = 0;
/// Default buffer size in bytes (10MB)
const DEFAULT_BUFFER_BYTES: usize = 10 * 1024 * 1024;

/// Policy for handling congestion when buffer is full
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DropPolicy {
    /// Return error to caller immediately
    #[default]
    Error,
    /// Block until buffer space is available (with timeout)
    Block,
    /// Drop the oldest message in buffer
    DropOldest,
    /// Drop the newest message (caller's message)
    DropNewest,
    /// Drop based on priority (requires priority field)
    DropLowestPriority,
}

/// Congestion control configuration
#[derive(Debug, Clone)]
pub struct CongestionConfig {
    /// Maximum messages in send buffer
    pub max_messages: usize,
    /// Maximum bytes in send buffer
    pub max_bytes: usize,
    /// Rate limit (messages per second, 0 = unlimited)
    pub rate_limit: u64,
    /// Drop policy when buffer is full
    pub drop_policy: DropPolicy,
    /// Block timeout (only used with DropPolicy::Block)
    pub block_timeout: Duration,
    /// Whether congestion control is enabled
    pub enabled: bool,
}

impl Default for CongestionConfig {
    fn default() -> Self {
        Self {
            max_messages: DEFAULT_BUFFER_SIZE,
            max_bytes: DEFAULT_BUFFER_BYTES,
            rate_limit: DEFAULT_RATE_LIMIT,
            drop_policy: DropPolicy::Error,
            block_timeout: Duration::from_millis(100),
            enabled: true,
        }
    }
}

impl CongestionConfig {
    /// Create config that blocks on congestion
    pub fn blocking() -> Self {
        Self {
            drop_policy: DropPolicy::Block,
            block_timeout: Duration::from_secs(1),
            ..Default::default()
        }
    }

    /// Create config that drops oldest messages
    pub fn drop_oldest() -> Self {
        Self {
            drop_policy: DropPolicy::DropOldest,
            ..Default::default()
        }
    }

    /// Create config with rate limiting
    pub fn rate_limited(messages_per_second: u64) -> Self {
        Self {
            rate_limit: messages_per_second,
            ..Default::default()
        }
    }

    /// Disable congestion control
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Default::default()
        }
    }
}

/// A queued message with metadata
#[derive(Debug)]
struct QueuedMessage {
    /// Serialized payload
    payload: Vec<u8>,
    /// Message priority (higher = more important)
    priority: u8,
}

/// Congestion control result
#[derive(Debug)]
pub enum CongestionResult {
    /// Message was accepted into the send buffer
    Accepted,
    /// Message was dropped due to congestion
    Dropped(DropReason),
    /// Caller should retry after this duration
    RetryAfter(Duration),
}

/// Reason for dropping a message
#[derive(Debug, Clone, Copy)]
pub enum DropReason {
    /// Buffer is full and policy is Error
    BufferFull,
    /// Buffer is full and policy is DropNewest
    DroppedNewest,
    /// Rate limit exceeded
    RateLimited,
    /// Block timeout exceeded
    BlockTimeout,
}

/// Token bucket rate limiter
pub struct TokenBucket {
    /// Tokens available
    tokens: f64,
    /// Maximum tokens (bucket capacity)
    max_tokens: f64,
    /// Tokens added per second
    refill_rate: f64,
    /// Last refill time
    last_refill: Instant,
}

impl TokenBucket {
    pub fn new(rate: u64) -> Self {
        assert!(rate > 0, "TokenBucket rate must be > 0 to avoid division by zero");
        let rate = rate as f64;
        Self {
            tokens: rate, // Start full
            max_tokens: rate,
            refill_rate: rate,
            last_refill: Instant::now(),
        }
    }

    /// Try to consume a token, returns true if successful
    pub fn try_consume(&mut self) -> bool {
        self.refill();

        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }

    /// Get time until next token is available
    pub fn time_until_token(&mut self) -> Duration {
        self.refill();

        if self.tokens >= 1.0 {
            Duration::ZERO
        } else {
            let needed = 1.0 - self.tokens;
            let seconds = needed / self.refill_rate;
            Duration::from_secs_f64(seconds)
        }
    }

    fn refill(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_refill).as_secs_f64();
        self.tokens = (self.tokens + elapsed * self.refill_rate).min(self.max_tokens);
        self.last_refill = now;
    }
}

/// Congestion controller that manages send buffer and rate limiting
pub struct CongestionController {
    config: CongestionConfig,
    /// Send buffer queue
    send_buffer: VecDeque<QueuedMessage>,
    /// Current buffer size in bytes
    current_bytes: usize,
    /// Rate limiter (if enabled)
    rate_limiter: Option<TokenBucket>,
    /// Statistics
    stats: CongestionStats,
}

/// Congestion statistics
#[derive(Debug, Default)]
pub struct CongestionStats {
    /// Total messages accepted
    pub accepted: AtomicU64,
    /// Total messages dropped
    pub dropped: AtomicU64,
    /// Total messages rate limited
    pub rate_limited: AtomicU64,
    /// Current buffer utilization (0-100)
    pub buffer_utilization: AtomicU64,
}

impl CongestionController {
    pub fn new(config: CongestionConfig) -> Self {
        let rate_limiter = if config.rate_limit > 0 {
            Some(TokenBucket::new(config.rate_limit))
        } else {
            None
        };

        Self {
            send_buffer: VecDeque::with_capacity(config.max_messages),
            current_bytes: 0,
            rate_limiter,
            stats: CongestionStats::default(),
            config,
        }
    }

    /// Try to enqueue a message for sending
    pub fn try_send(&mut self, payload: Vec<u8>, priority: u8) -> CongestionResult {
        if !self.config.enabled {
            return CongestionResult::Accepted;
        }

        // Check rate limit first
        if let Some(ref mut limiter) = self.rate_limiter {
            if !limiter.try_consume() {
                self.stats.rate_limited.fetch_add(1, Ordering::Relaxed);
                let wait_time = limiter.time_until_token();
                return CongestionResult::RetryAfter(wait_time);
            }
        }

        let payload_size = payload.len();

        // Check buffer capacity
        let buffer_full = self.send_buffer.len() >= self.config.max_messages
            || self.current_bytes + payload_size > self.config.max_bytes;

        if buffer_full {
            match self.config.drop_policy {
                DropPolicy::Error => {
                    self.stats.dropped.fetch_add(1, Ordering::Relaxed);
                    return CongestionResult::Dropped(DropReason::BufferFull);
                }
                DropPolicy::DropNewest => {
                    self.stats.dropped.fetch_add(1, Ordering::Relaxed);
                    return CongestionResult::Dropped(DropReason::DroppedNewest);
                }
                DropPolicy::DropOldest => {
                    // Remove oldest message
                    if let Some(old) = self.send_buffer.pop_front() {
                        self.current_bytes -= old.payload.len();
                        self.stats.dropped.fetch_add(1, Ordering::Relaxed);
                    }
                }
                DropPolicy::DropLowestPriority => {
                    // Find and remove lowest priority message
                    if let Some(idx) = self.find_lowest_priority() {
                        let old = self.send_buffer.remove(idx).unwrap();
                        self.current_bytes -= old.payload.len();
                        self.stats.dropped.fetch_add(1, Ordering::Relaxed);
                    }
                }
                DropPolicy::Block => {
                    // In blocking mode, we would wait - but this is synchronous
                    // Return retry suggestion
                    return CongestionResult::RetryAfter(Duration::from_millis(1));
                }
            }
        }

        // Add message to buffer
        self.current_bytes += payload_size;
        self.send_buffer
            .push_back(QueuedMessage { payload, priority });

        self.stats.accepted.fetch_add(1, Ordering::Relaxed);
        self.update_utilization();

        CongestionResult::Accepted
    }

    /// Pop the next message to send
    pub fn pop(&mut self) -> Option<Vec<u8>> {
        if let Some(msg) = self.send_buffer.pop_front() {
            self.current_bytes -= msg.payload.len();
            self.update_utilization();
            Some(msg.payload)
        } else {
            None
        }
    }

    /// Peek at the next message without removing it
    pub fn peek(&self) -> Option<&[u8]> {
        self.send_buffer.front().map(|m| m.payload.as_slice())
    }

    /// Get number of messages in buffer
    pub fn len(&self) -> usize {
        self.send_buffer.len()
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.send_buffer.is_empty()
    }

    /// Get current buffer size in bytes
    pub fn bytes(&self) -> usize {
        self.current_bytes
    }

    /// Get buffer utilization percentage
    pub fn utilization(&self) -> f64 {
        let msg_util = self.send_buffer.len() as f64 / self.config.max_messages as f64;
        let byte_util = self.current_bytes as f64 / self.config.max_bytes as f64;
        (msg_util.max(byte_util) * 100.0).min(100.0)
    }

    /// Get statistics
    pub fn stats(&self) -> &CongestionStats {
        &self.stats
    }

    /// Clear all pending messages
    pub fn clear(&mut self) {
        self.send_buffer.clear();
        self.current_bytes = 0;
        self.update_utilization();
    }

    fn find_lowest_priority(&self) -> Option<usize> {
        self.send_buffer
            .iter()
            .enumerate()
            .min_by_key(|(_, m)| m.priority)
            .map(|(i, _)| i)
    }

    fn update_utilization(&self) {
        self.stats
            .buffer_utilization
            .store(self.utilization() as u64, Ordering::Relaxed);
    }
}

/// Thread-safe congestion controller
pub struct SharedCongestionController {
    inner: Arc<Mutex<CongestionController>>,
}

impl SharedCongestionController {
    pub fn new(config: CongestionConfig) -> Self {
        Self {
            inner: Arc::new(Mutex::new(CongestionController::new(config))),
        }
    }

    pub fn try_send(&self, payload: Vec<u8>, priority: u8) -> CongestionResult {
        self.inner.lock().unwrap().try_send(payload, priority)
    }

    pub fn pop(&self) -> Option<Vec<u8>> {
        self.inner.lock().unwrap().pop()
    }

    pub fn len(&self) -> usize {
        self.inner.lock().unwrap().len()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.lock().unwrap().is_empty()
    }

    pub fn utilization(&self) -> f64 {
        self.inner.lock().unwrap().utilization()
    }

    pub fn clear(&self) {
        self.inner.lock().unwrap().clear()
    }
}

impl Clone for SharedCongestionController {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accept_messages() {
        let mut controller = CongestionController::new(CongestionConfig::default());

        let result = controller.try_send(vec![1, 2, 3], 0);
        assert!(matches!(result, CongestionResult::Accepted));
        assert_eq!(controller.len(), 1);
    }

    #[test]
    fn test_buffer_full_error() {
        let config = CongestionConfig {
            max_messages: 2,
            drop_policy: DropPolicy::Error,
            ..Default::default()
        };
        let mut controller = CongestionController::new(config);

        controller.try_send(vec![1], 0);
        controller.try_send(vec![2], 0);

        let result = controller.try_send(vec![3], 0);
        assert!(matches!(
            result,
            CongestionResult::Dropped(DropReason::BufferFull)
        ));
    }

    #[test]
    fn test_drop_oldest() {
        let config = CongestionConfig {
            max_messages: 2,
            drop_policy: DropPolicy::DropOldest,
            ..Default::default()
        };
        let mut controller = CongestionController::new(config);

        controller.try_send(vec![1], 0);
        controller.try_send(vec![2], 0);
        controller.try_send(vec![3], 0); // Should drop vec![1]

        assert_eq!(controller.len(), 2);
        assert_eq!(controller.pop().unwrap(), vec![2]);
        assert_eq!(controller.pop().unwrap(), vec![3]);
    }

    #[test]
    fn test_drop_newest() {
        let config = CongestionConfig {
            max_messages: 2,
            drop_policy: DropPolicy::DropNewest,
            ..Default::default()
        };
        let mut controller = CongestionController::new(config);

        controller.try_send(vec![1], 0);
        controller.try_send(vec![2], 0);

        let result = controller.try_send(vec![3], 0);
        assert!(matches!(
            result,
            CongestionResult::Dropped(DropReason::DroppedNewest)
        ));

        assert_eq!(controller.len(), 2);
        assert_eq!(controller.pop().unwrap(), vec![1]);
    }

    #[test]
    fn test_drop_lowest_priority() {
        let config = CongestionConfig {
            max_messages: 2,
            drop_policy: DropPolicy::DropLowestPriority,
            ..Default::default()
        };
        let mut controller = CongestionController::new(config);

        controller.try_send(vec![1], 10); // High priority
        controller.try_send(vec![2], 1); // Low priority
        controller.try_send(vec![3], 5); // Medium priority - should drop vec![2]

        assert_eq!(controller.len(), 2);
        assert_eq!(controller.pop().unwrap(), vec![1]); // High priority kept
        assert_eq!(controller.pop().unwrap(), vec![3]); // Medium priority kept
    }

    #[test]
    fn test_rate_limiting() {
        let config = CongestionConfig {
            rate_limit: 10, // 10 messages per second
            ..Default::default()
        };
        let mut controller = CongestionController::new(config);

        // Should accept first 10 quickly (bucket starts full)
        for i in 0..10 {
            let result = controller.try_send(vec![i], 0);
            assert!(
                matches!(result, CongestionResult::Accepted),
                "Message {} should be accepted",
                i
            );
        }

        // 11th should be rate limited
        let result = controller.try_send(vec![10], 0);
        assert!(matches!(result, CongestionResult::RetryAfter(_)));
    }

    #[test]
    fn test_byte_limit() {
        let config = CongestionConfig {
            max_messages: 1000,
            max_bytes: 100,
            drop_policy: DropPolicy::Error,
            ..Default::default()
        };
        let mut controller = CongestionController::new(config);

        // Add 50 bytes
        controller.try_send(vec![0; 50], 0);
        // Add another 50 bytes
        controller.try_send(vec![0; 50], 0);

        // This should fail - would exceed 100 bytes
        let result = controller.try_send(vec![0; 50], 0);
        assert!(matches!(
            result,
            CongestionResult::Dropped(DropReason::BufferFull)
        ));
    }

    #[test]
    fn test_utilization() {
        let config = CongestionConfig {
            max_messages: 100,
            max_bytes: 10000,
            ..Default::default()
        };
        let mut controller = CongestionController::new(config);

        // Empty buffer
        assert_eq!(controller.utilization(), 0.0);

        // Add 50 messages
        for i in 0..50 {
            controller.try_send(vec![i], 0);
        }
        assert!((controller.utilization() - 50.0).abs() < 1.0);
    }

    #[test]
    fn test_token_bucket() {
        let mut bucket = TokenBucket::new(10); // 10 tokens/sec

        // Should have full bucket initially
        for _ in 0..10 {
            assert!(bucket.try_consume());
        }

        // Bucket should be empty now
        assert!(!bucket.try_consume());

        // Wait time should be about 100ms for 1 token
        let wait = bucket.time_until_token();
        assert!(wait.as_millis() > 0 && wait.as_millis() <= 150);
    }

    #[test]
    fn test_disabled() {
        let config = CongestionConfig::disabled();
        let mut controller = CongestionController::new(config);

        // Should always accept when disabled
        for i in 0..1000 {
            let result = controller.try_send(vec![i as u8], 0);
            assert!(matches!(result, CongestionResult::Accepted));
        }
    }
}
