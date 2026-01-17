//! Topic Discovery and Subscription System
//!
//! Scans HORUS shared memory to discover active topics and allows subscribing to them.
//! Reuses horus_core's shm_topics_dir() for platform-agnostic paths.

use bevy::prelude::*;
use horus_core::communication::Topic;
use horus_core::core::NodeInfo;
use horus_core::memory::shm_topics_dir;
use horus_library::messages::{
    control::JointCommand,
    geometry::Twist,
    sensor::{Imu, LaserScan, Odometry},
};
use std::collections::HashMap;
use std::time::{Duration, Instant, SystemTime};

/// Known message types for HORUS topics
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    Twist,
    Odometry,
    Imu,
    LaserScan,
    JointCommand,
    Unknown,
}

impl MessageType {
    /// Infer message type from topic name conventions
    pub fn from_topic_name(name: &str) -> Self {
        // Check suffix patterns (e.g., "robot.cmd_vel" → Twist)
        if name.ends_with(".cmd_vel") || name.ends_with("_cmd_vel") {
            MessageType::Twist
        } else if name.ends_with(".odom") || name.ends_with("_odom") || name.ends_with(".odometry")
        {
            MessageType::Odometry
        } else if name.ends_with(".imu") || name.ends_with("_imu") {
            MessageType::Imu
        } else if name.ends_with(".scan") || name.ends_with("_scan") || name.ends_with(".laser") {
            MessageType::LaserScan
        } else if name.ends_with(".joint_cmd")
            || name.ends_with(".joint_states")
            || name.ends_with("_joints")
        {
            MessageType::JointCommand
        } else {
            MessageType::Unknown
        }
    }

    /// Get display name for the message type
    pub fn display_name(&self) -> &'static str {
        match self {
            MessageType::Twist => "Twist",
            MessageType::Odometry => "Odometry",
            MessageType::Imu => "IMU",
            MessageType::LaserScan => "LaserScan",
            MessageType::JointCommand => "JointCmd",
            MessageType::Unknown => "Unknown",
        }
    }
}

/// Information about a discovered HORUS topic
#[derive(Debug, Clone)]
pub struct TopicInfo {
    pub name: String,
    pub size_bytes: u64,
    pub last_modified: Option<SystemTime>,
    pub is_active: bool,
    pub message_type: MessageType,
}

/// Resource that tracks discovered topics
#[derive(Resource)]
pub struct TopicScanner {
    /// All discovered topics
    pub topics: HashMap<String, TopicInfo>,
    /// Last scan time
    last_scan: Instant,
    /// Scan interval
    scan_interval: Duration,
}

impl Default for TopicScanner {
    fn default() -> Self {
        Self {
            topics: HashMap::new(),
            last_scan: Instant::now() - Duration::from_secs(10), // Force initial scan
            scan_interval: Duration::from_millis(500),
        }
    }
}

impl TopicScanner {
    /// Check if a scan is needed
    pub fn needs_scan(&self) -> bool {
        self.last_scan.elapsed() > self.scan_interval
    }

    /// Perform a scan of the topics directory
    pub fn scan(&mut self) {
        let topics_path = shm_topics_dir();

        if !topics_path.exists() {
            self.topics.clear();
            self.last_scan = Instant::now();
            return;
        }

        let mut found_topics = HashMap::new();

        if let Ok(entries) = std::fs::read_dir(&topics_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                if !path.is_file() {
                    continue;
                }

                let Some(name) = path.file_name().and_then(|s| s.to_str()) else {
                    continue;
                };

                let metadata = entry.metadata().ok();
                let size_bytes = metadata.as_ref().map(|m| m.len()).unwrap_or(0);
                let last_modified = metadata.as_ref().and_then(|m| m.modified().ok());

                // Consider topic active if modified within last 5 seconds
                let is_active = last_modified
                    .and_then(|t| t.elapsed().ok())
                    .map(|e| e.as_secs() < 5)
                    .unwrap_or(false);

                let message_type = MessageType::from_topic_name(name);

                found_topics.insert(
                    name.to_string(),
                    TopicInfo {
                        name: name.to_string(),
                        size_bytes,
                        last_modified,
                        is_active,
                        message_type,
                    },
                );
            }
        }

        self.topics = found_topics;
        self.last_scan = Instant::now();
    }

    /// Get topics sorted by name
    pub fn sorted_topics(&self) -> Vec<&TopicInfo> {
        let mut topics: Vec<_> = self.topics.values().collect();
        topics.sort_by(|a, b| a.name.cmp(&b.name));
        topics
    }

    /// Get active topics only
    pub fn active_topics(&self) -> Vec<&TopicInfo> {
        self.topics.values().filter(|t| t.is_active).collect()
    }
}

/// System that periodically scans for topics
pub fn topic_scan_system(mut scanner: ResMut<TopicScanner>) {
    if scanner.needs_scan() {
        scanner.scan();
    }
}

/// Plugin for topic discovery
pub struct TopicDiscoveryPlugin;

impl Plugin for TopicDiscoveryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TopicScanner>()
            .init_resource::<TopicSubscriptions>()
            .add_systems(Update, (topic_scan_system, topic_subscription_system));
    }
}

/// Holds the latest received value for each message type
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum LatestMessage {
    Twist(Twist),
    Odometry(Odometry),
    Imu(Imu),
    LaserScan(LaserScan),
    JointCommand(JointCommand),
}

/// A single topic subscription with its Hub
pub struct TopicSubscription {
    pub topic_name: String,
    pub message_type: MessageType,
    pub latest: Option<LatestMessage>,
    hub: SubscriptionHub,
}

/// Type-erased hub holder
enum SubscriptionHub {
    Twist(Topic<Twist>),
    Odometry(Topic<Odometry>),
    Imu(Topic<Imu>),
    LaserScan(Topic<LaserScan>),
    JointCommand(Topic<JointCommand>),
}

impl TopicSubscription {
    /// Create a new subscription for the given topic
    pub fn new(topic_name: &str, message_type: MessageType) -> Option<Self> {
        let hub = match message_type {
            MessageType::Twist => Topic::new(topic_name).ok().map(SubscriptionHub::Twist),
            MessageType::Odometry => Topic::new(topic_name).ok().map(SubscriptionHub::Odometry),
            MessageType::Imu => Topic::new(topic_name).ok().map(SubscriptionHub::Imu),
            MessageType::LaserScan => Topic::new(topic_name).ok().map(SubscriptionHub::LaserScan),
            MessageType::JointCommand => {
                Topic::new(topic_name).ok().map(SubscriptionHub::JointCommand)
            }
            MessageType::Unknown => None,
        }?;

        Some(Self {
            topic_name: topic_name.to_string(),
            message_type,
            latest: None,
            hub,
        })
    }

    /// Try to receive the latest message
    pub fn recv(&mut self) {
        match &mut self.hub {
            SubscriptionHub::Twist(hub) => {
                if let Some(msg) = hub.recv(&mut None::<&mut NodeInfo>) {
                    self.latest = Some(LatestMessage::Twist(msg));
                }
            }
            SubscriptionHub::Odometry(hub) => {
                if let Some(msg) = hub.recv(&mut None::<&mut NodeInfo>) {
                    self.latest = Some(LatestMessage::Odometry(msg));
                }
            }
            SubscriptionHub::Imu(hub) => {
                if let Some(msg) = hub.recv(&mut None::<&mut NodeInfo>) {
                    self.latest = Some(LatestMessage::Imu(msg));
                }
            }
            SubscriptionHub::LaserScan(hub) => {
                if let Some(msg) = hub.recv(&mut None::<&mut NodeInfo>) {
                    self.latest = Some(LatestMessage::LaserScan(msg));
                }
            }
            SubscriptionHub::JointCommand(hub) => {
                if let Some(msg) = hub.recv(&mut None::<&mut NodeInfo>) {
                    self.latest = Some(LatestMessage::JointCommand(msg));
                }
            }
        }
    }
}

/// Resource holding all active topic subscriptions
#[derive(Resource, Default)]
pub struct TopicSubscriptions {
    pub subscriptions: HashMap<String, TopicSubscription>,
}

impl TopicSubscriptions {
    /// Subscribe to a topic
    pub fn subscribe(&mut self, topic_name: &str, message_type: MessageType) -> bool {
        if self.subscriptions.contains_key(topic_name) {
            return true; // Already subscribed
        }

        if let Some(sub) = TopicSubscription::new(topic_name, message_type) {
            self.subscriptions.insert(topic_name.to_string(), sub);
            true
        } else {
            false
        }
    }

    /// Unsubscribe from a topic
    pub fn unsubscribe(&mut self, topic_name: &str) {
        self.subscriptions.remove(topic_name);
    }

    /// Check if subscribed to a topic
    pub fn is_subscribed(&self, topic_name: &str) -> bool {
        self.subscriptions.contains_key(topic_name)
    }

    /// Get subscription for a topic
    pub fn get(&self, topic_name: &str) -> Option<&TopicSubscription> {
        self.subscriptions.get(topic_name)
    }
}

/// System that receives messages for all active subscriptions
pub fn topic_subscription_system(mut subscriptions: ResMut<TopicSubscriptions>) {
    for sub in subscriptions.subscriptions.values_mut() {
        sub.recv();
    }
}
