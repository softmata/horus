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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::node::TopicMetadata;

    fn make_topic_meta(name: &str) -> TopicMetadata {
        TopicMetadata {
            topic_name: name.to_string(),
            type_name: "TestType".to_string(),
        }
    }

    // ── NodeEvent ──

    #[test]
    fn test_node_event_equality() {
        assert_eq!(NodeEvent::Started, NodeEvent::Started);
        assert_eq!(NodeEvent::Stopped, NodeEvent::Stopped);
        assert_ne!(NodeEvent::Started, NodeEvent::Stopped);
    }

    // ── NodeAnnouncement::started ──

    #[test]
    fn test_started_basic() {
        let pubs = vec![make_topic_meta("cmd_vel")];
        let subs = vec![make_topic_meta("odom")];
        let ann = NodeAnnouncement::started("motor_ctrl", &pubs, &subs);

        assert_eq!(ann.name, "motor_ctrl");
        assert_eq!(ann.event, NodeEvent::Started);
        assert_eq!(ann.pid, std::process::id());
        assert_eq!(ann.publishers, vec!["cmd_vel".to_string()]);
        assert_eq!(ann.subscribers, vec!["odom".to_string()]);
        assert!(ann.timestamp_ms > 0);
    }

    #[test]
    fn test_started_empty_pubs_subs() {
        let ann = NodeAnnouncement::started("bare_node", &[], &[]);
        assert_eq!(ann.name, "bare_node");
        assert!(ann.publishers.is_empty());
        assert!(ann.subscribers.is_empty());
    }

    #[test]
    fn test_started_multiple_pubs_subs() {
        let pubs = vec![
            make_topic_meta("topic_a"),
            make_topic_meta("topic_b"),
            make_topic_meta("topic_c"),
        ];
        let subs = vec![make_topic_meta("input_1"), make_topic_meta("input_2")];
        let ann = NodeAnnouncement::started("multi", &pubs, &subs);
        assert_eq!(ann.publishers.len(), 3);
        assert_eq!(ann.subscribers.len(), 2);
    }

    #[test]
    fn test_started_empty_name() {
        let ann = NodeAnnouncement::started("", &[], &[]);
        assert_eq!(ann.name, "");
    }

    #[test]
    fn test_started_long_name() {
        let long_name = "a".repeat(10_000);
        let ann = NodeAnnouncement::started(&long_name, &[], &[]);
        assert_eq!(ann.name.len(), 10_000);
    }

    #[test]
    fn test_started_unicode_name() {
        let ann = NodeAnnouncement::started("ロボット制御", &[], &[]);
        assert_eq!(ann.name, "ロボット制御");
    }

    #[test]
    fn test_started_timestamp_reasonable() {
        let ann = NodeAnnouncement::started("ts_test", &[], &[]);
        // Timestamp should be after 2024-01-01 (1704067200000 ms)
        assert!(ann.timestamp_ms > 1_704_067_200_000);
        // And before 2030-01-01 (1893456000000 ms)
        assert!(ann.timestamp_ms < 1_893_456_000_000);
    }

    #[test]
    fn test_started_pid_matches_current() {
        let ann = NodeAnnouncement::started("pid_test", &[], &[]);
        assert_eq!(ann.pid, std::process::id());
    }

    // ── NodeAnnouncement::stopped ──

    #[test]
    fn test_stopped_basic() {
        let ann = NodeAnnouncement::stopped("shutting_down");
        assert_eq!(ann.name, "shutting_down");
        assert_eq!(ann.event, NodeEvent::Stopped);
        assert_eq!(ann.pid, std::process::id());
        assert!(ann.publishers.is_empty());
        assert!(ann.subscribers.is_empty());
        assert!(ann.timestamp_ms > 0);
    }

    #[test]
    fn test_stopped_empty_name() {
        let ann = NodeAnnouncement::stopped("");
        assert_eq!(ann.name, "");
        assert_eq!(ann.event, NodeEvent::Stopped);
    }

    // ── Serialization roundtrip ──

    #[test]
    fn test_serde_roundtrip_started() {
        let pubs = vec![make_topic_meta("pub1")];
        let subs = vec![make_topic_meta("sub1")];
        let ann = NodeAnnouncement::started("serde_test", &pubs, &subs);

        let serialized = serde_json::to_string(&ann).unwrap();
        let deserialized: NodeAnnouncement = serde_json::from_str(&serialized).unwrap();

        assert_eq!(deserialized.name, "serde_test");
        assert_eq!(deserialized.event, NodeEvent::Started);
        assert_eq!(deserialized.publishers, vec!["pub1".to_string()]);
        assert_eq!(deserialized.subscribers, vec!["sub1".to_string()]);
        assert_eq!(deserialized.pid, ann.pid);
        assert_eq!(deserialized.timestamp_ms, ann.timestamp_ms);
    }

    #[test]
    fn test_serde_roundtrip_stopped() {
        let ann = NodeAnnouncement::stopped("serde_stop");
        let serialized = serde_json::to_string(&ann).unwrap();
        let deserialized: NodeAnnouncement = serde_json::from_str(&serialized).unwrap();

        assert_eq!(deserialized.name, "serde_stop");
        assert_eq!(deserialized.event, NodeEvent::Stopped);
    }

    // ── LogSummary ──

    #[test]
    fn test_log_summary_format() {
        let pubs = vec![make_topic_meta("cmd_vel")];
        let ann = NodeAnnouncement::started("motor", &pubs, &[]);
        let summary = ann.log_summary();
        assert!(summary.contains("motor"));
        assert!(summary.contains("Started"));
        assert!(summary.contains("cmd_vel"));
    }

    // ── DISCOVERY_TOPIC constant ──

    #[test]
    fn test_discovery_topic_name() {
        assert_eq!(DISCOVERY_TOPIC, "horus.discovery");
    }

    // ── Clone ──

    #[test]
    fn test_announcement_clone() {
        let ann = NodeAnnouncement::started("clone_test", &[], &[]);
        let cloned = ann.clone();
        assert_eq!(ann.name, cloned.name);
        assert_eq!(ann.event, cloned.event);
        assert_eq!(ann.pid, cloned.pid);
        assert_eq!(ann.timestamp_ms, cloned.timestamp_ms);
    }
}
