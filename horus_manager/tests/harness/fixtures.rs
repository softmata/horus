//! Test node configuration and preset fixtures.
#![allow(dead_code)]

use horus_core::core::HealthStatus;

/// Configuration for a simulated test node.
///
/// The test harness writes a presence file that exactly matches what
/// `NodePresence::read_all()` expects (JSON in the SHM nodes directory).
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct TestNodeConfig {
    /// Node name (also used as filename stem).
    pub name: String,
    /// PID to record in the presence file.
    /// Defaults to the current process so `is_alive()` returns true.
    pub pid: u32,
    /// Health status — only affects how the node *should* appear in the
    /// monitor; the presence file itself does not carry health.
    pub health: HealthStatus,
    /// Published topics: `(topic_name, type_name)`.
    pub publishers: Vec<(String, String)>,
    /// Subscribed topics: `(topic_name, type_name)`.
    pub subscribers: Vec<(String, String)>,
    /// Scheduler name (if running under a scheduler).
    pub scheduler: Option<String>,
    /// Target tick rate in Hz.
    pub rate_hz: Option<f64>,
    /// Node priority.
    pub priority: u32,
}

impl TestNodeConfig {
    /// Minimal node with no topics.
    pub fn bare(name: &str) -> Self {
        Self {
            name: name.to_string(),
            pid: std::process::id(),
            health: HealthStatus::Healthy,
            publishers: Vec::new(),
            subscribers: Vec::new(),
            scheduler: None,
            rate_hz: None,
            priority: 0,
        }
    }

    /// Sensor-style node that publishes on one topic.
    pub fn sensor(name: &str, topic: &str, msg_type: &str) -> Self {
        Self {
            name: name.to_string(),
            pid: std::process::id(),
            health: HealthStatus::Healthy,
            publishers: vec![(topic.to_string(), msg_type.to_string())],
            subscribers: Vec::new(),
            scheduler: None,
            rate_hz: Some(30.0),
            priority: 10,
        }
    }

    /// Actuator-style node that subscribes to one topic.
    pub fn actuator(name: &str, topic: &str, msg_type: &str) -> Self {
        Self {
            name: name.to_string(),
            pid: std::process::id(),
            health: HealthStatus::Healthy,
            publishers: Vec::new(),
            subscribers: vec![(topic.to_string(), msg_type.to_string())],
            scheduler: None,
            rate_hz: Some(100.0),
            priority: 20,
        }
    }

    /// Processing node that subscribes to one topic and publishes to another.
    pub fn processor(
        name: &str,
        input_topic: &str,
        input_type: &str,
        output_topic: &str,
        output_type: &str,
    ) -> Self {
        Self {
            name: name.to_string(),
            pid: std::process::id(),
            health: HealthStatus::Healthy,
            publishers: vec![(output_topic.to_string(), output_type.to_string())],
            subscribers: vec![(input_topic.to_string(), input_type.to_string())],
            scheduler: Some("main".to_string()),
            rate_hz: Some(60.0),
            priority: 15,
        }
    }

    // -- Builder methods --

    /// Set the scheduler name.
    pub fn with_scheduler(mut self, scheduler: &str) -> Self {
        self.scheduler = Some(scheduler.to_string());
        self
    }

    /// Set the tick rate.
    pub fn with_rate_hz(mut self, hz: f64) -> Self {
        self.rate_hz = Some(hz);
        self
    }

    /// Set the priority.
    pub fn with_priority(mut self, priority: u32) -> Self {
        self.priority = priority;
        self
    }

    /// Add a publisher topic.
    pub fn with_publisher(mut self, topic: &str, msg_type: &str) -> Self {
        self.publishers
            .push((topic.to_string(), msg_type.to_string()));
        self
    }

    /// Add a subscriber topic.
    pub fn with_subscriber(mut self, topic: &str, msg_type: &str) -> Self {
        self.subscribers
            .push((topic.to_string(), msg_type.to_string()));
        self
    }
}
