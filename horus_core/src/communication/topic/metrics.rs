//! Metrics tracking for topics.

use std::sync::atomic::{AtomicU32, AtomicU64};

/// Migration and operational metrics for a Topic
#[derive(Debug, Default)]
pub(crate) struct MigrationMetrics {
    /// Messages sent through this topic
    pub messages_sent: AtomicU64,
    /// Messages received through this topic
    pub messages_received: AtomicU64,
    /// Number of send failures
    pub send_failures: AtomicU64,
    /// Sends dropped because the retry budget was gone, rather than because
    /// the ring stayed full for the whole retry.
    pub send_retry_overruns: AtomicU64,
    /// Number of receive failures
    pub recv_failures: AtomicU64,
    /// Number of backend migrations performed
    pub migrations: AtomicU32,
}

/// Non-atomic snapshot of topic metrics (for external consumers)
#[derive(Debug, Clone, Default)]
pub struct TopicMetrics {
    messages_sent: u64,
    messages_received: u64,
    send_failures: u64,
    recv_failures: u64,
    send_retry_overruns: u64,
}

impl TopicMetrics {
    /// Create a metrics snapshot.
    pub(crate) fn new(
        messages_sent: u64,
        messages_received: u64,
        send_failures: u64,
        recv_failures: u64,
        send_retry_overruns: u64,
    ) -> Self {
        Self {
            messages_sent,
            messages_received,
            send_failures,
            recv_failures,
            send_retry_overruns,
        }
    }

    pub fn messages_sent(&self) -> u64 {
        self.messages_sent
    }

    pub fn messages_received(&self) -> u64 {
        self.messages_received
    }

    pub fn send_failures(&self) -> u64 {
        self.send_failures
    }

    pub fn recv_failures(&self) -> u64 {
        self.recv_failures
    }

    /// How many of [`send_failures`](Self::send_failures) were dropped because
    /// the publisher ran out of retry budget rather than because the consumer
    /// was behind.
    ///
    /// Kept separate deliberately. Folding the two together would make an
    /// oversubscribed box — where a single `yield_now` can take tens of
    /// milliseconds — indistinguishable from a slow consumer, and they call for
    /// opposite responses.
    pub fn send_retry_overruns(&self) -> u64 {
        self.send_retry_overruns
    }
}
