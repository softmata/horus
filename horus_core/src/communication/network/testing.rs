//! Testing utilities for ROS2 interoperability
//!
//! This module provides mock implementations for testing ROS2 communication
//! patterns without requiring an actual ROS2 installation.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::network::testing::{MockRos2Node, MockTopic};
//! use horus_ros2_msgs::geometry_msgs::Twist;
//!
//! // Create a mock ROS2 node
//! let mut mock = MockRos2Node::new("test_robot");
//!
//! // Create a mock subscriber
//! let cmd_vel = mock.create_subscriber::<Twist>("cmd_vel");
//!
//! // Inject a test message
//! mock.inject_message("cmd_vel", Twist {
//!     linear: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
//!     angular: Vector3::default(),
//! });
//!
//! // Receive it in the subscriber
//! let msg = cmd_vel.recv().unwrap();
//! assert_eq!(msg.linear.x, 1.0);
//! ```

use std::any::{Any, TypeId};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

/// A mock ROS2 node for testing without ROS2 installation
///
/// This provides a local message passing system that simulates ROS2 pub/sub
/// patterns, allowing you to test your HORUS-ROS2 bridge code without
/// needing ROS2 installed.
#[derive(Debug)]
#[allow(clippy::type_complexity)]
pub struct MockRos2Node {
    /// Node name
    name: String,
    /// Topic registry with message queues
    topics: Arc<RwLock<HashMap<String, MockTopicData>>>,
    /// Record of all published messages by topic
    published_messages: Arc<Mutex<HashMap<String, Vec<Box<dyn Any + Send + Sync>>>>>,
    /// Node creation time
    created_at: Instant,
}

/// Internal topic data
#[derive(Debug)]
#[allow(dead_code)]
struct MockTopicData {
    /// Type ID for type checking
    type_id: TypeId,
    /// Message queue
    messages: Arc<Mutex<Vec<Box<dyn Any + Send + Sync>>>>,
    /// Number of subscribers
    subscriber_count: usize,
    /// Number of publishers
    publisher_count: usize,
}

impl MockRos2Node {
    /// Create a new mock ROS2 node
    ///
    /// # Arguments
    /// * `name` - Node name (e.g., "turtlebot_controller")
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            topics: Arc::new(RwLock::new(HashMap::new())),
            published_messages: Arc::new(Mutex::new(HashMap::new())),
            created_at: Instant::now(),
        }
    }

    /// Get the node name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get uptime since node creation
    pub fn uptime(&self) -> Duration {
        self.created_at.elapsed()
    }

    /// Create a mock publisher for a topic
    ///
    /// # Type Parameters
    /// * `T` - Message type (must be Clone + Send + Sync + 'static)
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "cmd_vel")
    pub fn create_publisher<T>(&mut self, topic: &str) -> MockPublisher<T>
    where
        T: Clone + Send + Sync + 'static,
    {
        let type_id = TypeId::of::<T>();
        let topic_name = topic.to_string();

        {
            let mut topics = self.topics.write().unwrap();
            let entry = topics
                .entry(topic_name.clone())
                .or_insert_with(|| MockTopicData {
                    type_id,
                    messages: Arc::new(Mutex::new(Vec::new())),
                    subscriber_count: 0,
                    publisher_count: 0,
                });
            entry.publisher_count += 1;
        }

        MockPublisher {
            topic: topic_name,
            topics: Arc::clone(&self.topics),
            published: Arc::clone(&self.published_messages),
            _marker: std::marker::PhantomData,
        }
    }

    /// Create a mock subscriber for a topic
    ///
    /// # Type Parameters
    /// * `T` - Message type (must be Clone + Send + Sync + 'static)
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "odom")
    pub fn create_subscriber<T>(&mut self, topic: &str) -> MockSubscriber<T>
    where
        T: Clone + Send + Sync + 'static,
    {
        let type_id = TypeId::of::<T>();
        let topic_name = topic.to_string();

        let messages = {
            let mut topics = self.topics.write().unwrap();
            let entry = topics
                .entry(topic_name.clone())
                .or_insert_with(|| MockTopicData {
                    type_id,
                    messages: Arc::new(Mutex::new(Vec::new())),
                    subscriber_count: 0,
                    publisher_count: 0,
                });
            entry.subscriber_count += 1;
            Arc::clone(&entry.messages)
        };

        MockSubscriber {
            topic: topic_name,
            messages,
            _marker: std::marker::PhantomData,
        }
    }

    /// Inject a message directly into a topic (for testing)
    ///
    /// This simulates receiving a message from an external ROS2 node.
    ///
    /// # Type Parameters
    /// * `T` - Message type
    ///
    /// # Arguments
    /// * `topic` - Topic name
    /// * `message` - Message to inject
    pub fn inject_message<T>(&self, topic: &str, message: T)
    where
        T: Clone + Send + Sync + 'static,
    {
        let topics = self.topics.read().unwrap();
        if let Some(topic_data) = topics.get(topic) {
            let mut messages = topic_data.messages.lock().unwrap();
            messages.push(Box::new(message));
        }
    }

    /// Get all published messages for a topic
    ///
    /// # Type Parameters
    /// * `T` - Message type
    ///
    /// # Arguments
    /// * `topic` - Topic name
    ///
    /// # Returns
    /// Vector of published messages (cloned)
    pub fn get_published<T>(&self, topic: &str) -> Vec<T>
    where
        T: Clone + 'static,
    {
        let published = self.published_messages.lock().unwrap();
        if let Some(messages) = published.get(topic) {
            messages
                .iter()
                .filter_map(|m| m.downcast_ref::<T>().cloned())
                .collect()
        } else {
            Vec::new()
        }
    }

    /// Clear all published messages for a topic
    pub fn clear_published(&self, topic: &str) {
        let mut published = self.published_messages.lock().unwrap();
        if let Some(messages) = published.get_mut(topic) {
            messages.clear();
        }
    }

    /// Get topic statistics
    pub fn get_topic_stats(&self, topic: &str) -> Option<TopicStats> {
        let topics = self.topics.read().unwrap();
        topics.get(topic).map(|data| TopicStats {
            subscriber_count: data.subscriber_count,
            publisher_count: data.publisher_count,
            pending_messages: data.messages.lock().unwrap().len(),
        })
    }

    /// List all registered topics
    pub fn list_topics(&self) -> Vec<String> {
        let topics = self.topics.read().unwrap();
        topics.keys().cloned().collect()
    }
}

/// Statistics for a mock topic
#[derive(Debug, Clone)]
pub struct TopicStats {
    /// Number of subscribers
    pub subscriber_count: usize,
    /// Number of publishers
    pub publisher_count: usize,
    /// Number of pending messages in queue
    pub pending_messages: usize,
}

/// A mock publisher for testing
#[derive(Debug)]
#[allow(clippy::type_complexity)]
pub struct MockPublisher<T> {
    topic: String,
    topics: Arc<RwLock<HashMap<String, MockTopicData>>>,
    published: Arc<Mutex<HashMap<String, Vec<Box<dyn Any + Send + Sync>>>>>,
    _marker: std::marker::PhantomData<T>,
}

impl<T> MockPublisher<T>
where
    T: Clone + Send + Sync + 'static,
{
    /// Publish a message to the topic
    ///
    /// The message is delivered to all subscribers and recorded for later inspection.
    pub fn publish(&self, message: T) {
        // Record published message
        {
            let mut published = self.published.lock().unwrap();
            let messages = published.entry(self.topic.clone()).or_default();
            messages.push(Box::new(message.clone()));
        }

        // Deliver to subscribers
        let topics = self.topics.read().unwrap();
        if let Some(topic_data) = topics.get(&self.topic) {
            let mut messages = topic_data.messages.lock().unwrap();
            messages.push(Box::new(message));
        }
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

/// A mock subscriber for testing
#[derive(Debug)]
pub struct MockSubscriber<T> {
    topic: String,
    messages: Arc<Mutex<Vec<Box<dyn Any + Send + Sync>>>>,
    _marker: std::marker::PhantomData<T>,
}

impl<T> MockSubscriber<T>
where
    T: Clone + 'static,
{
    /// Try to receive a message without blocking
    ///
    /// Returns the oldest message in the queue, if any.
    pub fn try_recv(&self) -> Option<T> {
        let mut messages = self.messages.lock().unwrap();
        if messages.is_empty() {
            return None;
        }
        // Pop from front (oldest first)
        let boxed = messages.remove(0);
        boxed.downcast::<T>().ok().map(|b| *b)
    }

    /// Receive a message, blocking until one is available or timeout
    ///
    /// # Arguments
    /// * `timeout` - Maximum time to wait
    ///
    /// # Returns
    /// Message if received within timeout, None otherwise
    pub fn recv_timeout(&self, timeout: Duration) -> Option<T> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if let Some(msg) = self.try_recv() {
                return Some(msg);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        None
    }

    /// Receive a message with default timeout (1 second)
    pub fn recv(&self) -> Option<T> {
        self.recv_timeout(Duration::from_secs(1))
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Get the number of pending messages
    pub fn pending(&self) -> usize {
        self.messages.lock().unwrap().len()
    }

    /// Clear all pending messages
    pub fn clear(&self) {
        self.messages.lock().unwrap().clear();
    }
}

/// Builder for creating complex mock scenarios
#[derive(Debug)]
pub struct MockScenarioBuilder {
    nodes: HashMap<String, MockRos2Node>,
}

impl Default for MockScenarioBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl MockScenarioBuilder {
    /// Create a new scenario builder
    pub fn new() -> Self {
        Self {
            nodes: HashMap::new(),
        }
    }

    /// Add a mock node to the scenario
    pub fn with_node(mut self, name: &str) -> Self {
        self.nodes.insert(name.to_string(), MockRos2Node::new(name));
        self
    }

    /// Build the scenario, returning all nodes
    pub fn build(self) -> HashMap<String, MockRos2Node> {
        self.nodes
    }

    /// Get a mutable reference to a node (for setup)
    pub fn node_mut(&mut self, name: &str) -> Option<&mut MockRos2Node> {
        self.nodes.get_mut(name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, PartialEq)]
    struct TestMessage {
        value: i32,
        name: String,
    }

    #[test]
    fn test_mock_node_creation() {
        let node = MockRos2Node::new("test_node");
        assert_eq!(node.name(), "test_node");
        assert!(node.uptime() < Duration::from_secs(1));
    }

    #[test]
    fn test_mock_pub_sub() {
        let mut node = MockRos2Node::new("test_node");

        let publisher = node.create_publisher::<TestMessage>("test_topic");
        let subscriber = node.create_subscriber::<TestMessage>("test_topic");

        // Publish a message
        publisher.publish(TestMessage {
            value: 42,
            name: "hello".to_string(),
        });

        // Receive it
        let msg = subscriber.recv().unwrap();
        assert_eq!(msg.value, 42);
        assert_eq!(msg.name, "hello");
    }

    #[test]
    fn test_inject_message() {
        let mut node = MockRos2Node::new("test_node");
        let subscriber = node.create_subscriber::<TestMessage>("test_topic");

        // Inject a message (simulating external ROS2 node)
        node.inject_message(
            "test_topic",
            TestMessage {
                value: 100,
                name: "injected".to_string(),
            },
        );

        let msg = subscriber.recv().unwrap();
        assert_eq!(msg.value, 100);
        assert_eq!(msg.name, "injected");
    }

    #[test]
    fn test_published_record() {
        let mut node = MockRos2Node::new("test_node");
        let publisher = node.create_publisher::<TestMessage>("test_topic");

        publisher.publish(TestMessage {
            value: 1,
            name: "a".to_string(),
        });
        publisher.publish(TestMessage {
            value: 2,
            name: "b".to_string(),
        });
        publisher.publish(TestMessage {
            value: 3,
            name: "c".to_string(),
        });

        let published: Vec<TestMessage> = node.get_published("test_topic");
        assert_eq!(published.len(), 3);
        assert_eq!(published[0].value, 1);
        assert_eq!(published[2].value, 3);
    }

    #[test]
    fn test_topic_stats() {
        let mut node = MockRos2Node::new("test_node");

        let _pub1 = node.create_publisher::<TestMessage>("test_topic");
        let _pub2 = node.create_publisher::<TestMessage>("test_topic");
        let _sub = node.create_subscriber::<TestMessage>("test_topic");

        let stats = node.get_topic_stats("test_topic").unwrap();
        assert_eq!(stats.publisher_count, 2);
        assert_eq!(stats.subscriber_count, 1);
    }

    #[test]
    fn test_list_topics() {
        let mut node = MockRos2Node::new("test_node");

        node.create_publisher::<i32>("topic_a");
        node.create_subscriber::<String>("topic_b");

        let topics = node.list_topics();
        assert_eq!(topics.len(), 2);
        assert!(topics.contains(&"topic_a".to_string()));
        assert!(topics.contains(&"topic_b".to_string()));
    }

    #[test]
    fn test_timeout_recv() {
        let mut node = MockRos2Node::new("test_node");
        let subscriber = node.create_subscriber::<TestMessage>("empty_topic");

        // Should timeout
        let result = subscriber.recv_timeout(Duration::from_millis(50));
        assert!(result.is_none());
    }

    #[test]
    fn test_clear_pending() {
        let mut node = MockRos2Node::new("test_node");
        let subscriber = node.create_subscriber::<TestMessage>("test_topic");

        node.inject_message(
            "test_topic",
            TestMessage {
                value: 1,
                name: "a".to_string(),
            },
        );
        node.inject_message(
            "test_topic",
            TestMessage {
                value: 2,
                name: "b".to_string(),
            },
        );

        assert_eq!(subscriber.pending(), 2);
        subscriber.clear();
        assert_eq!(subscriber.pending(), 0);
    }

    #[test]
    fn test_scenario_builder() {
        let mut builder = MockScenarioBuilder::new()
            .with_node("robot")
            .with_node("controller");

        // Setup robot node
        {
            let robot = builder.node_mut("robot").unwrap();
            let _odom_pub = robot.create_publisher::<f64>("odom");
        }

        // Setup controller node
        {
            let controller = builder.node_mut("controller").unwrap();
            let _odom_sub = controller.create_subscriber::<f64>("odom");
        }

        let nodes = builder.build();
        assert_eq!(nodes.len(), 2);
    }
}
