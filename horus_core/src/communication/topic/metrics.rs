//! Metrics tracking for topics.

use std::sync::atomic::{AtomicU32, AtomicU64};

/// Migration and operational metrics for a Topic
#[derive(Debug, Default)]
pub(crate) struct MigrationMetrics {
    /// Messages sent through this topic, counted on every `send()`.
    pub messages_sent: AtomicU64,
    /// Messages delivered by `recv()` on this topic.
    ///
    /// `try_recv()` is the low-level path and does not count; `recv()` is what
    /// `Topic::recv` uses.
    pub messages_received: AtomicU64,
    /// Number of send failures
    pub send_failures: AtomicU64,
    /// Sends dropped because the retry budget was gone, rather than because
    /// the ring stayed full for the whole retry.
    pub send_retry_overruns: AtomicU64,
    /// Number of backend migrations performed
    pub migrations: AtomicU32,
}

/// Non-atomic snapshot of topic metrics (for external consumers)
///
/// `messages_sent` and `messages_received` used to move only on the `#[cold]`
/// verbose-logging path, which runs while the `horus monitor` TUI has set a
/// topic's verbose flag — so in an ordinary run both read 0 no matter how much
/// traffic the topic carried, while `MockTopic` maintained them faithfully. A
/// test double that reports what the real transport does not is the one defect
/// a double cannot have. Both are counted on the ordinary paths now.
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

    /// Receive failures.
    ///
    /// Only [`MockTopic`](crate::testing::MockTopic) produces a non-zero value
    /// here, from its `recv_fail_after` / `recv_fail_every_n` fault injection.
    /// The real transport never increments it, deliberately: a `recv()` that
    /// returns `None` is an empty ring, not a failure. For real consumer-side
    /// loss — a publisher lapping a subscriber — use `Topic::missed_count()`.
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
