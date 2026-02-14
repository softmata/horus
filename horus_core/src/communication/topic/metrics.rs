//! Metrics tracking for topics.

use std::sync::atomic::{AtomicU32, AtomicU64};

/// Migration and operational metrics for a Topic
#[derive(Debug, Default)]
pub struct MigrationMetrics {
    /// Messages sent through this topic
    pub messages_sent: AtomicU64,
    /// Messages received through this topic
    pub messages_received: AtomicU64,
    /// Number of send failures
    pub send_failures: AtomicU64,
    /// Number of receive failures
    pub recv_failures: AtomicU64,
    /// Number of backend migrations performed
    pub migrations: AtomicU32,
    /// Current backend latency estimate (ns)
    pub estimated_latency_ns: AtomicU32,
    /// Last observed epoch
    pub last_epoch: AtomicU64,
}

/// Non-atomic snapshot of topic metrics (for external consumers)
#[derive(Debug, Clone, Default)]
pub struct TopicMetrics {
    /// Number of messages successfully sent
    pub messages_sent: u64,
    /// Number of messages successfully received
    pub messages_received: u64,
    /// Number of send failures
    pub send_failures: u64,
    /// Number of receive failures
    pub recv_failures: u64,
}
