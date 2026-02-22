//! Node discovery via shared memory topic
//!
//! Provides ROS-like node discovery where nodes announce their presence
//! to a discovery topic at startup and shutdown. Monitor can read this
//! topic to see all active nodes without any file I/O.

use crate::communication::Topic;
use crate::core::node::{LogSummary, TopicMetadata};
use serde::{Deserialize, Serialize};
use std::sync::OnceLock;
use std::time::{SystemTime, UNIX_EPOCH};

/// Discovery topic name
pub const DISCOVERY_TOPIC: &str = "horus.discovery";

/// Node announcement message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeAnnouncement {
    /// Node name
    pub name: String,
    /// Process ID
    pub pid: u32,
    /// Announcement type
    pub event: NodeEvent,
    /// Topics this node publishes to
    pub publishers: Vec<String>,
    /// Topics this node subscribes to
    pub subscribers: Vec<String>,
    /// Timestamp in milliseconds since epoch
    pub timestamp_ms: u64,
}

/// Node lifecycle event
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum NodeEvent {
    /// Node started
    Started,
    /// Node stopped gracefully
    Stopped,
    /// Node crashed/errored
    Crashed,
}

impl LogSummary for NodeAnnouncement {
    fn log_summary(&self) -> String {
        format!(
            "{}:{:?} pubs={:?} subs={:?}",
            self.name, self.event, self.publishers, self.subscribers
        )
    }
}

impl NodeAnnouncement {
    /// Create a started announcement
    pub fn started(
        name: &str,
        publishers: &[TopicMetadata],
        subscribers: &[TopicMetadata],
    ) -> Self {
        Self {
            name: name.to_string(),
            pid: std::process::id(),
            event: NodeEvent::Started,
            publishers: publishers.iter().map(|t| t.topic_name.clone()).collect(),
            subscribers: subscribers.iter().map(|t| t.topic_name.clone()).collect(),
            timestamp_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64,
        }
    }

    /// Create a stopped announcement
    pub fn stopped(name: &str) -> Self {
        Self {
            name: name.to_string(),
            pid: std::process::id(),
            event: NodeEvent::Stopped,
            publishers: Vec::new(),
            subscribers: Vec::new(),
            timestamp_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64,
        }
    }

}

/// Global discovery topic (lazy initialized)
static DISCOVERY: OnceLock<Option<Topic<NodeAnnouncement>>> = OnceLock::new();

/// Get or create the discovery topic
fn get_discovery_topic() -> Option<&'static Topic<NodeAnnouncement>> {
    DISCOVERY
        .get_or_init(|| Topic::new(DISCOVERY_TOPIC).ok())
        .as_ref()
}

/// Announce node started
pub fn announce_started(name: &str, publishers: &[TopicMetadata], subscribers: &[TopicMetadata]) {
    if let Some(topic) = get_discovery_topic() {
        let announcement = NodeAnnouncement::started(name, publishers, subscribers);
        topic.send(announcement);
    }
}

/// Announce node stopped
pub fn announce_stopped(name: &str) {
    if let Some(topic) = get_discovery_topic() {
        let announcement = NodeAnnouncement::stopped(name);
        topic.send(announcement);
    }
}
