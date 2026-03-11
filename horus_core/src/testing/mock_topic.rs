//! MockTopic — a configurable mock for `Topic<T>` that supports failure injection.
//!
//! Use this in tests to simulate send failures, recv timeouts, capacity exhaustion,
//! and other error conditions without needing real shared memory.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::testing::{MockTopic, MockTopicConfig};
//!
//! // Create a mock that fails every 3rd send
//! let config = MockTopicConfig {
//!     send_fail_every_n: Some(3),
//!     ..Default::default()
//! };
//! let topic: MockTopic<u64> = MockTopic::new("test_topic", config);
//! topic.send(1); // ok
//! topic.send(2); // ok
//! topic.send(3); // dropped (failure injected)
//! ```

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

use crate::communication::topic::metrics::TopicMetrics;
use crate::communication::SendBlockingError;

/// Configuration for MockTopic failure injection.
#[derive(Debug, Clone, Default)]
pub struct MockTopicConfig {
    /// Maximum capacity of the internal buffer (0 = unlimited)
    pub capacity: usize,
    /// Fail send every N-th call (None = never fail)
    pub send_fail_every_n: Option<u64>,
    /// Fail recv every N-th call by returning None (None = never fail)
    pub recv_fail_every_n: Option<u64>,
    /// Always fail sends after this many total sends (None = no limit)
    pub send_fail_after: Option<u64>,
    /// Always return None from recv after this many total recvs (None = no limit)
    pub recv_fail_after: Option<u64>,
    /// Simulate send_blocking timeout (if true, send_blocking always returns Timeout)
    pub send_blocking_always_timeout: bool,
}

/// A mock implementation of the Topic API for testing.
///
/// Provides the same public interface as `Topic<T>` but uses an in-memory
/// buffer instead of shared memory, with configurable failure injection.
pub struct MockTopic<T> {
    name: String,
    buffer: Mutex<VecDeque<T>>,
    config: MockTopicConfig,
    send_count: AtomicU64,
    recv_count: AtomicU64,
    send_failures: AtomicU64,
    recv_failures: AtomicU64,
}

impl<T: Clone> MockTopic<T> {
    /// Create a new MockTopic with the given name and configuration.
    pub fn new(name: impl Into<String>, config: MockTopicConfig) -> Self {
        Self {
            name: name.into(),
            buffer: Mutex::new(VecDeque::new()),
            config,
            send_count: AtomicU64::new(0),
            recv_count: AtomicU64::new(0),
            send_failures: AtomicU64::new(0),
            recv_failures: AtomicU64::new(0),
        }
    }

    /// Create a MockTopic with default config (no failures).
    pub fn simple(name: impl Into<String>) -> Self {
        Self::new(name, MockTopicConfig::default())
    }

    /// Get the topic name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Send a message (fire-and-forget).
    ///
    /// Subject to failure injection based on config.
    pub fn send(&self, msg: T) {
        let _ = self.try_send(msg);
    }

    /// Try to send a message. Returns `Err(msg)` if the send fails.
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        let count = self.send_count.fetch_add(1, Ordering::Relaxed) + 1;

        // Check send_fail_after
        if let Some(limit) = self.config.send_fail_after {
            if count > limit {
                self.send_failures.fetch_add(1, Ordering::Relaxed);
                return Err(msg);
            }
        }

        // Check send_fail_every_n
        if let Some(n) = self.config.send_fail_every_n {
            if n > 0 && count.is_multiple_of(n) {
                self.send_failures.fetch_add(1, Ordering::Relaxed);
                return Err(msg);
            }
        }

        let mut buf = self.buffer.lock().unwrap();

        // Check capacity
        if self.config.capacity > 0 && buf.len() >= self.config.capacity {
            self.send_failures.fetch_add(1, Ordering::Relaxed);
            return Err(msg);
        }

        buf.push_back(msg);
        Ok(())
    }

    /// Send a message, blocking until space is available or timeout expires.
    pub fn send_blocking(
        &self,
        msg: T,
        _timeout: std::time::Duration,
    ) -> Result<(), SendBlockingError> {
        if self.config.send_blocking_always_timeout {
            return Err(SendBlockingError::Timeout);
        }
        match self.try_send(msg) {
            Ok(()) => Ok(()),
            Err(_) => Err(SendBlockingError::Timeout),
        }
    }

    /// Receive a message.
    pub fn recv(&self) -> Option<T> {
        self.try_recv()
    }

    /// Try to receive a message.
    pub fn try_recv(&self) -> Option<T> {
        let count = self.recv_count.fetch_add(1, Ordering::Relaxed) + 1;

        // Check recv_fail_after
        if let Some(limit) = self.config.recv_fail_after {
            if count > limit {
                self.recv_failures.fetch_add(1, Ordering::Relaxed);
                return None;
            }
        }

        // Check recv_fail_every_n
        if let Some(n) = self.config.recv_fail_every_n {
            if n > 0 && count.is_multiple_of(n) {
                self.recv_failures.fetch_add(1, Ordering::Relaxed);
                return None;
            }
        }

        let mut buf = self.buffer.lock().unwrap();
        buf.pop_front()
    }

    /// Read the most recent message without consuming it.
    pub fn read_latest(&self) -> Option<T> {
        let buf = self.buffer.lock().unwrap();
        buf.back().cloned()
    }

    /// Check if a message is available.
    pub fn has_message(&self) -> bool {
        let buf = self.buffer.lock().unwrap();
        !buf.is_empty()
    }

    /// Get the number of pending messages.
    pub fn pending_count(&self) -> u64 {
        let buf = self.buffer.lock().unwrap();
        buf.len() as u64
    }

    /// Get topic metrics.
    pub fn metrics(&self) -> TopicMetrics {
        TopicMetrics::new(
            self.send_count.load(Ordering::Relaxed),
            self.recv_count.load(Ordering::Relaxed),
            self.send_failures.load(Ordering::Relaxed),
            self.recv_failures.load(Ordering::Relaxed),
        )
    }

    /// Get the number of messages currently in the buffer.
    pub fn len(&self) -> usize {
        let buf = self.buffer.lock().unwrap();
        buf.len()
    }

    /// Check if the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Clear all messages from the buffer.
    pub fn clear(&self) {
        let mut buf = self.buffer.lock().unwrap();
        buf.clear();
    }

    /// Get total send count (including failures).
    pub fn total_sends(&self) -> u64 {
        self.send_count.load(Ordering::Relaxed)
    }

    /// Get total recv count (including failures).
    pub fn total_recvs(&self) -> u64 {
        self.recv_count.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Basic operations ──

    #[test]
    fn mock_topic_send_recv_roundtrip() {
        let topic: MockTopic<u64> = MockTopic::simple("test");
        topic.send(42);
        assert_eq!(topic.recv(), Some(42));
        assert_eq!(topic.recv(), None);
    }

    #[test]
    fn mock_topic_fifo_order() {
        let topic: MockTopic<u64> = MockTopic::simple("fifo");
        for i in 0..10 {
            topic.send(i);
        }
        for i in 0..10 {
            assert_eq!(topic.recv(), Some(i));
        }
    }

    #[test]
    fn mock_topic_name() {
        let topic: MockTopic<u64> = MockTopic::simple("my_topic");
        assert_eq!(topic.name(), "my_topic");
    }

    #[test]
    fn mock_topic_has_message() {
        let topic: MockTopic<u64> = MockTopic::simple("test");
        assert!(!topic.has_message());
        topic.send(1);
        assert!(topic.has_message());
        topic.recv();
        assert!(!topic.has_message());
    }

    #[test]
    fn mock_topic_pending_count() {
        let topic: MockTopic<u64> = MockTopic::simple("test");
        assert_eq!(topic.pending_count(), 0);
        topic.send(1);
        topic.send(2);
        assert_eq!(topic.pending_count(), 2);
        topic.recv();
        assert_eq!(topic.pending_count(), 1);
    }

    #[test]
    fn mock_topic_read_latest() {
        let topic: MockTopic<u64> = MockTopic::simple("test");
        assert_eq!(topic.read_latest(), None);
        topic.send(1);
        topic.send(2);
        topic.send(3);
        assert_eq!(topic.read_latest(), Some(3));
        // read_latest doesn't consume
        assert_eq!(topic.pending_count(), 3);
    }

    #[test]
    fn mock_topic_metrics() {
        let topic: MockTopic<u64> = MockTopic::simple("test");
        topic.send(1);
        topic.send(2);
        topic.recv();
        let m = topic.metrics();
        assert_eq!(m.messages_sent(), 2);
        assert_eq!(m.messages_received(), 1);
    }

    #[test]
    fn mock_topic_clear() {
        let topic: MockTopic<u64> = MockTopic::simple("test");
        topic.send(1);
        topic.send(2);
        assert_eq!(topic.len(), 2);
        topic.clear();
        assert!(topic.is_empty());
    }

    // ── Capacity limiting ──

    #[test]
    fn mock_topic_capacity_limit() {
        let config = MockTopicConfig {
            capacity: 2,
            ..Default::default()
        };
        let topic: MockTopic<u64> = MockTopic::new("cap", config);
        topic.try_send(1).unwrap();
        topic.try_send(2).unwrap();
        // Third send should fail — at capacity
        let result = topic.try_send(3);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), 3);
    }

    // ── Send failure injection ──

    #[test]
    fn mock_topic_send_fail_every_n() {
        let config = MockTopicConfig {
            send_fail_every_n: Some(3),
            ..Default::default()
        };
        let topic: MockTopic<u64> = MockTopic::new("fail", config);
        topic.try_send(1).unwrap(); // 1st
        topic.try_send(2).unwrap(); // 2nd
        assert!(topic.try_send(3).is_err()); // 3rd — fail
        topic.try_send(4).unwrap(); // 4th
        topic.try_send(5).unwrap(); // 5th
        assert!(topic.try_send(6).is_err()); // 6th — fail

        let m = topic.metrics();
        assert_eq!(m.send_failures(), 2);
    }

    #[test]
    fn mock_topic_send_fail_after() {
        let config = MockTopicConfig {
            send_fail_after: Some(2),
            ..Default::default()
        };
        let topic: MockTopic<u64> = MockTopic::new("fail_after", config);
        topic.try_send(1).unwrap(); // 1st
        topic.try_send(2).unwrap(); // 2nd
        assert!(topic.try_send(3).is_err()); // 3rd — all fail after 2
        assert!(topic.try_send(4).is_err()); // 4th — still failing
    }

    // ── Recv failure injection ──

    #[test]
    fn mock_topic_recv_fail_every_n() {
        let config = MockTopicConfig {
            recv_fail_every_n: Some(2),
            ..Default::default()
        };
        let topic: MockTopic<u64> = MockTopic::new("recv_fail", config);
        topic.send(10);
        topic.send(20);
        topic.send(30);

        assert_eq!(topic.recv(), Some(10)); // 1st recv — ok
        assert_eq!(topic.recv(), None); // 2nd recv — injected failure
        assert_eq!(topic.recv(), Some(20)); // 3rd recv — ok (message still there)
        assert_eq!(topic.recv(), None); // 4th recv — injected failure
    }

    #[test]
    fn mock_topic_recv_fail_after() {
        let config = MockTopicConfig {
            recv_fail_after: Some(1),
            ..Default::default()
        };
        let topic: MockTopic<u64> = MockTopic::new("recv_after", config);
        topic.send(10);
        topic.send(20);

        assert_eq!(topic.recv(), Some(10)); // 1st — ok
        assert_eq!(topic.recv(), None); // 2nd — fail, even though messages exist
    }

    // ── send_blocking failure ──

    #[test]
    fn mock_topic_send_blocking_timeout() {
        let config = MockTopicConfig {
            send_blocking_always_timeout: true,
            ..Default::default()
        };
        let topic: MockTopic<u64> = MockTopic::new("blocking", config);
        let result = topic.send_blocking(42, std::time::Duration::from_secs(1));
        assert_eq!(result, Err(SendBlockingError::Timeout));
    }

    #[test]
    fn mock_topic_send_blocking_success() {
        let topic: MockTopic<u64> = MockTopic::simple("blocking_ok");
        let result = topic.send_blocking(42, std::time::Duration::from_secs(1));
        assert_eq!(result, Ok(()));
        assert_eq!(topic.recv(), Some(42));
    }

    // ── Combined failure modes ──

    #[test]
    fn mock_topic_capacity_and_send_failure() {
        let config = MockTopicConfig {
            capacity: 3,
            send_fail_every_n: Some(2),
            ..Default::default()
        };
        let topic: MockTopic<u64> = MockTopic::new("combined", config);
        topic.try_send(1).unwrap(); // 1st — ok
        assert!(topic.try_send(2).is_err()); // 2nd — fail (every 2)
        topic.try_send(3).unwrap(); // 3rd — ok
        assert!(topic.try_send(4).is_err()); // 4th — fail (every 2)
        topic.try_send(5).unwrap(); // 5th — ok
                                    // Buffer has [1, 3, 5] — at capacity
        assert!(topic.try_send(6).is_err()); // 6th — fail (every 2, AND capacity)
        assert!(topic.try_send(7).is_err()); // 7th — fail (capacity full)
    }

    // ── String type ──

    #[test]
    fn mock_topic_string_type() {
        let topic: MockTopic<String> = MockTopic::simple("strings");
        topic.send("hello".to_string());
        topic.send("world".to_string());
        assert_eq!(topic.recv(), Some("hello".to_string()));
        assert_eq!(topic.recv(), Some("world".to_string()));
    }
}
