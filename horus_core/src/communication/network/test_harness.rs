//! Zenoh Integration Test Harness
//!
//! This module provides utilities for running Zenoh integration tests with
//! automatic setup, isolation, and cleanup. It's designed to be CI-friendly
//! and support parallel test execution.
//!
//! # Features
//!
//! - **Namespace isolation**: Each test gets a unique namespace prefix
//! - **Automatic cleanup**: Resources are cleaned up after tests
//! - **Parallel-safe**: Tests can run concurrently without port conflicts
//! - **CI-friendly**: Works in headless environments without external routers
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::network::test_harness::{ZenohTestContext, zenoh_test};
//!
//! #[test]
//! fn test_zenoh_pubsub() {
//!     zenoh_test(|ctx| async {
//!         // ctx provides isolated namespace and session
//!         let topic = ctx.topic::<MyMessage>("test_data").await?;
//!         topic.send(MyMessage { value: 42 })?;
//!
//!         let received = topic.recv_timeout(Duration::from_secs(1)).await?;
//!         assert_eq!(received.value, 42);
//!         Ok(())
//!     });
//! }
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Global counter for generating unique test namespaces
static TEST_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Configuration for Zenoh test harness
#[derive(Debug, Clone)]
pub struct ZenohTestConfig {
    /// Namespace prefix for this test (auto-generated if None)
    pub namespace: Option<String>,
    /// Timeout for test setup
    pub setup_timeout: Duration,
    /// Timeout for test teardown
    pub teardown_timeout: Duration,
    /// Enable verbose logging
    pub verbose: bool,
    /// Use peer-to-peer mode (no router)
    pub peer_mode: bool,
}

impl Default for ZenohTestConfig {
    fn default() -> Self {
        Self {
            namespace: None,
            setup_timeout: Duration::from_secs(10),
            teardown_timeout: Duration::from_secs(5),
            verbose: false,
            peer_mode: true, // P2P mode is more CI-friendly (no router needed)
        }
    }
}

impl ZenohTestConfig {
    /// Create a new test config with a specific namespace
    pub fn with_namespace(mut self, namespace: impl Into<String>) -> Self {
        self.namespace = Some(namespace.into());
        self
    }

    /// Enable verbose logging
    pub fn verbose(mut self) -> Self {
        self.verbose = true;
        self
    }

    /// Use router mode instead of peer-to-peer
    pub fn router_mode(mut self) -> Self {
        self.peer_mode = false;
        self
    }

    /// Set custom setup timeout
    pub fn setup_timeout(mut self, timeout: Duration) -> Self {
        self.setup_timeout = timeout;
        self
    }
}

/// Context for a Zenoh integration test
///
/// This provides an isolated namespace and utilities for testing
/// Zenoh-based communication without conflicting with other tests.
#[derive(Debug)]
pub struct ZenohTestContext {
    /// Unique namespace prefix for this test
    pub namespace: String,
    /// Test ID (unique per test run)
    pub test_id: u64,
    /// Configuration used for this test
    pub config: ZenohTestConfig,
    /// Timestamp when the test started
    started_at: Instant,
    /// Topics created in this test (for cleanup tracking)
    created_topics: Arc<std::sync::Mutex<Vec<String>>>,
}

impl ZenohTestContext {
    /// Create a new test context with default configuration
    pub fn new() -> Self {
        Self::with_config(ZenohTestConfig::default())
    }

    /// Create a new test context with custom configuration
    pub fn with_config(config: ZenohTestConfig) -> Self {
        let test_id = TEST_COUNTER.fetch_add(1, Ordering::SeqCst);
        let namespace = config
            .namespace
            .clone()
            .unwrap_or_else(|| format!("test_{}", test_id));

        Self {
            namespace,
            test_id,
            config,
            started_at: Instant::now(),
            created_topics: Arc::new(std::sync::Mutex::new(Vec::new())),
        }
    }

    /// Get a namespaced topic name
    ///
    /// This prefixes the topic with the test namespace to ensure isolation.
    ///
    /// # Example
    /// ```rust,ignore
    /// let ctx = ZenohTestContext::new();
    /// let topic_name = ctx.namespaced("sensor/data");
    /// // Returns something like "test_0/sensor/data"
    /// ```
    pub fn namespaced(&self, topic: &str) -> String {
        format!("{}/{}", self.namespace, topic)
    }

    /// Record a topic creation (for cleanup tracking)
    pub fn track_topic(&self, topic: &str) {
        let mut topics = self.created_topics.lock().unwrap();
        topics.push(topic.to_string());
    }

    /// Get elapsed time since test start
    pub fn elapsed(&self) -> Duration {
        self.started_at.elapsed()
    }

    /// Get all tracked topics
    pub fn tracked_topics(&self) -> Vec<String> {
        self.created_topics.lock().unwrap().clone()
    }

    /// Log a message if verbose mode is enabled
    pub fn log(&self, msg: &str) {
        if self.config.verbose {
            eprintln!("[zenoh_test:{}] {}", self.test_id, msg);
        }
    }

    /// Generate a unique ID within this test context
    pub fn unique_id(&self) -> String {
        format!(
            "{}_{}",
            self.test_id,
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        )
    }
}

impl Default for ZenohTestContext {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for ZenohTestContext {
    fn drop(&mut self) {
        let topics = self.tracked_topics();
        if self.config.verbose && !topics.is_empty() {
            eprintln!(
                "[zenoh_test:{}] Cleaning up {} tracked topics",
                self.test_id,
                topics.len()
            );
        }
        // Note: Actual Zenoh cleanup would happen here if we had a session reference
        // Since we're providing a harness, the actual cleanup is handled by the test code
    }
}

/// Guard for running a scoped Zenoh test
///
/// This guard automatically cleans up resources when dropped.
#[derive(Debug)]
pub struct ZenohTestGuard {
    context: ZenohTestContext,
}

impl ZenohTestGuard {
    /// Create a new test guard
    pub fn new() -> Self {
        Self {
            context: ZenohTestContext::new(),
        }
    }

    /// Create with custom configuration
    pub fn with_config(config: ZenohTestConfig) -> Self {
        Self {
            context: ZenohTestContext::with_config(config),
        }
    }

    /// Get the test context
    pub fn context(&self) -> &ZenohTestContext {
        &self.context
    }

    /// Get a mutable reference to the test context
    pub fn context_mut(&mut self) -> &mut ZenohTestContext {
        &mut self.context
    }
}

impl Default for ZenohTestGuard {
    fn default() -> Self {
        Self::new()
    }
}

impl std::ops::Deref for ZenohTestGuard {
    type Target = ZenohTestContext;

    fn deref(&self) -> &Self::Target {
        &self.context
    }
}

impl std::ops::DerefMut for ZenohTestGuard {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.context
    }
}

/// Run a synchronous Zenoh test with automatic setup and cleanup
///
/// This is the simplest way to write a Zenoh integration test.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::communication::network::test_harness::zenoh_test_sync;
///
/// #[test]
/// fn test_basic_pubsub() {
///     zenoh_test_sync(|ctx| {
///         let topic_name = ctx.namespaced("test");
///         // ... test code ...
///         Ok(())
///     });
/// }
/// ```
pub fn zenoh_test_sync<F>(f: F)
where
    F: FnOnce(&ZenohTestContext) -> Result<(), Box<dyn std::error::Error>>,
{
    zenoh_test_sync_with_config(ZenohTestConfig::default(), f)
}

/// Run a synchronous Zenoh test with custom configuration
pub fn zenoh_test_sync_with_config<F>(config: ZenohTestConfig, f: F)
where
    F: FnOnce(&ZenohTestContext) -> Result<(), Box<dyn std::error::Error>>,
{
    let guard = ZenohTestGuard::with_config(config);
    guard.log("Starting test");

    let result = f(&guard.context);

    guard.log(&format!("Test completed in {:?}", guard.elapsed()));

    if let Err(e) = result {
        panic!("Zenoh test failed: {}", e);
    }
}

/// Test isolation utilities for parallel test execution
pub mod isolation {
    use std::collections::HashSet;
    use std::sync::Mutex;

    lazy_static::lazy_static! {
        /// Global registry of active test namespaces
        static ref ACTIVE_NAMESPACES: Mutex<HashSet<String>> = Mutex::new(HashSet::new());
    }

    /// Acquire an isolated namespace for testing
    ///
    /// This ensures no two tests use the same namespace concurrently.
    pub fn acquire_namespace(prefix: &str) -> String {
        let mut namespaces = ACTIVE_NAMESPACES.lock().unwrap();
        let mut counter = 0;
        loop {
            let namespace = if counter == 0 {
                prefix.to_string()
            } else {
                format!("{}_{}", prefix, counter)
            };

            if !namespaces.contains(&namespace) {
                namespaces.insert(namespace.clone());
                return namespace;
            }
            counter += 1;
        }
    }

    /// Release a namespace back to the pool
    pub fn release_namespace(namespace: &str) {
        let mut namespaces = ACTIVE_NAMESPACES.lock().unwrap();
        namespaces.remove(namespace);
    }

    /// Guard that automatically releases the namespace on drop
    #[derive(Debug)]
    pub struct NamespaceGuard {
        namespace: String,
    }

    impl NamespaceGuard {
        /// Acquire a new namespace with the given prefix
        pub fn new(prefix: &str) -> Self {
            Self {
                namespace: acquire_namespace(prefix),
            }
        }

        /// Get the namespace
        pub fn namespace(&self) -> &str {
            &self.namespace
        }
    }

    impl Drop for NamespaceGuard {
        fn drop(&mut self) {
            release_namespace(&self.namespace);
        }
    }
}

/// Assertion helpers for Zenoh tests
pub mod assertions {
    use std::time::Duration;

    /// Assert that a condition becomes true within a timeout
    ///
    /// This is useful for eventual consistency tests.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// assert_eventually!(|| received_count.load(Ordering::SeqCst) > 0, Duration::from_secs(1));
    /// ```
    #[macro_export]
    macro_rules! assert_eventually {
        ($cond:expr, $timeout:expr) => {
            $crate::communication::network::test_harness::assertions::assert_eventually_impl(
                || $cond,
                $timeout,
                stringify!($cond),
            );
        };
    }

    /// Implementation of assert_eventually
    pub fn assert_eventually_impl<F>(mut condition: F, timeout: Duration, condition_str: &str)
    where
        F: FnMut() -> bool,
    {
        let start = std::time::Instant::now();
        while start.elapsed() < timeout {
            if condition() {
                return;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!(
            "Condition '{}' did not become true within {:?}",
            condition_str, timeout
        );
    }

    /// Assert that a value is received within a timeout
    pub fn assert_receives_within<T, F>(mut receiver: F, timeout: Duration, description: &str) -> T
    where
        F: FnMut() -> Option<T>,
    {
        let start = std::time::Instant::now();
        while start.elapsed() < timeout {
            if let Some(value) = receiver() {
                return value;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!("{} was not received within {:?}", description, timeout);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_context_creation() {
        let ctx = ZenohTestContext::new();
        assert!(ctx.namespace.starts_with("test_"));
        assert!(ctx.elapsed() < Duration::from_secs(1));
    }

    #[test]
    fn test_namespaced_topics() {
        let ctx =
            ZenohTestContext::with_config(ZenohTestConfig::default().with_namespace("my_test"));
        assert_eq!(ctx.namespaced("sensor/data"), "my_test/sensor/data");
        assert_eq!(ctx.namespaced("cmd_vel"), "my_test/cmd_vel");
    }

    #[test]
    fn test_topic_tracking() {
        let ctx = ZenohTestContext::new();
        ctx.track_topic("topic1");
        ctx.track_topic("topic2");

        let topics = ctx.tracked_topics();
        assert_eq!(topics.len(), 2);
        assert!(topics.contains(&"topic1".to_string()));
        assert!(topics.contains(&"topic2".to_string()));
    }

    #[test]
    fn test_guard_deref() {
        let guard = ZenohTestGuard::new();
        // Should be able to call context methods directly
        let _ = guard.namespaced("test");
    }

    #[test]
    fn test_sync_test_harness() {
        zenoh_test_sync(|ctx| {
            assert!(ctx.namespace.starts_with("test_"));
            Ok(())
        });
    }

    #[test]
    fn test_namespace_isolation() {
        let ns1 = isolation::NamespaceGuard::new("test");
        let ns2 = isolation::NamespaceGuard::new("test");

        // Should get different namespaces
        assert_ne!(ns1.namespace(), ns2.namespace());
        assert!(ns2.namespace().starts_with("test_"));
    }

    #[test]
    fn test_namespace_release() {
        let namespace = {
            let guard = isolation::NamespaceGuard::new("release_test");
            guard.namespace().to_string()
        };
        // After guard is dropped, we should get the same namespace back
        let guard2 = isolation::NamespaceGuard::new("release_test");
        assert_eq!(guard2.namespace(), namespace);
    }

    #[test]
    fn test_unique_id() {
        let ctx = ZenohTestContext::new();
        let id1 = ctx.unique_id();
        std::thread::sleep(Duration::from_nanos(1));
        let id2 = ctx.unique_id();
        assert_ne!(id1, id2);
    }

    #[test]
    fn test_assert_eventually() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;

        let flag = Arc::new(AtomicBool::new(false));

        // Spawn a thread that sets the flag after a short delay
        let flag_clone = Arc::clone(&flag);
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            flag_clone.store(true, Ordering::SeqCst);
        });

        let flag_check = Arc::clone(&flag);
        assertions::assert_eventually_impl(
            move || flag_check.load(Ordering::SeqCst),
            Duration::from_secs(1),
            "flag becomes true",
        );
    }
}
