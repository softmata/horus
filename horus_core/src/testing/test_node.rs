//! TestNode — a configurable mock Node for testing scheduler and executor behavior.
//!
//! Consolidates the many scattered mock node types (CounterNode, PanicNode, SlowNode,
//! PanicInitNode, OrderTrackingNode, StubNode) into a single reusable builder.
//!
//! # Examples
//!
//! ```rust,ignore
//! use horus_core::testing::TestNode;
//! use std::sync::Arc;
//! use std::sync::atomic::AtomicUsize;
//!
//! // Simple counter node
//! let counter = Arc::new(AtomicUsize::new(0));
//! let node = TestNode::counter("sensor", counter.clone());
//!
//! // Node that panics on 3rd tick
//! let node = TestNode::builder("crasher")
//!     .panic_on_tick(3)
//!     .build();
//!
//! // Slow node (sleeps 10ms per tick)
//! let node = TestNode::builder("slow")
//!     .tick_delay(std::time::Duration::from_millis(10))
//!     .build();
//!
//! // Node that fails init
//! let node = TestNode::builder("bad_init")
//!     .fail_init()
//!     .build();
//! ```

use crate::core::Node;
use crate::error::HorusResult;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Behavior for the `init()` lifecycle method.
#[derive(Debug, Clone)]
pub enum InitBehavior {
    /// init() succeeds (default)
    Ok,
    /// init() panics with a message
    Panic(String),
}

/// Behavior for the `tick()` method.
#[derive(Clone)]
pub enum TickBehavior {
    /// tick() does nothing (default)
    Noop,
    /// tick() increments the provided counter
    Count(Arc<AtomicUsize>),
    /// tick() sleeps for the given duration
    Delay(Duration),
    /// tick() panics on the N-th invocation (1-indexed)
    PanicOnTick(u32),
    /// tick() records its name to a shared order tracker
    TrackOrder(Arc<Mutex<Vec<String>>>),
    /// tick() runs a custom closure
    Custom(Arc<dyn Fn(&str, u32) + Send + Sync>),
}

impl std::fmt::Debug for TickBehavior {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Noop => write!(f, "Noop"),
            Self::Count(_) => write!(f, "Count(...)"),
            Self::Delay(d) => write!(f, "Delay({:?})", d),
            Self::PanicOnTick(n) => write!(f, "PanicOnTick({})", n),
            Self::TrackOrder(_) => write!(f, "TrackOrder(...)"),
            Self::Custom(_) => write!(f, "Custom(...)"),
        }
    }
}

/// Behavior for the `shutdown()` lifecycle method.
#[derive(Debug, Clone)]
pub enum ShutdownBehavior {
    /// shutdown() does nothing (default)
    Ok,
    /// shutdown() panics
    Panic(String),
}

/// A configurable mock Node for testing.
///
/// Supports all common test patterns: counting ticks, injecting panics,
/// simulating slow operations, tracking execution order, and custom behaviors.
pub struct TestNode {
    name: String,
    init_behavior: InitBehavior,
    tick_behavior: TickBehavior,
    shutdown_behavior: ShutdownBehavior,
    tick_count: u32,
}

impl TestNode {
    /// Create a builder for a TestNode with the given name.
    pub fn builder(name: impl Into<String>) -> TestNodeBuilder {
        TestNodeBuilder {
            name: name.into(),
            init_behavior: InitBehavior::Ok,
            tick_behavior: TickBehavior::Noop,
            shutdown_behavior: ShutdownBehavior::Ok,
        }
    }

    /// Create a simple stub node that does nothing.
    pub fn stub(name: impl Into<String>) -> Box<dyn Node> {
        Box::new(Self {
            name: name.into(),
            init_behavior: InitBehavior::Ok,
            tick_behavior: TickBehavior::Noop,
            shutdown_behavior: ShutdownBehavior::Ok,
            tick_count: 0,
        })
    }

    /// Create a counter node that increments a shared counter on each tick.
    pub fn counter(name: impl Into<String>, counter: Arc<AtomicUsize>) -> Box<dyn Node> {
        Box::new(Self {
            name: name.into(),
            init_behavior: InitBehavior::Ok,
            tick_behavior: TickBehavior::Count(counter),
            shutdown_behavior: ShutdownBehavior::Ok,
            tick_count: 0,
        })
    }

    /// Create a slow node that sleeps for the given duration on each tick.
    pub fn slow(name: impl Into<String>, delay: Duration) -> Box<dyn Node> {
        Box::new(Self {
            name: name.into(),
            init_behavior: InitBehavior::Ok,
            tick_behavior: TickBehavior::Delay(delay),
            shutdown_behavior: ShutdownBehavior::Ok,
            tick_count: 0,
        })
    }

    /// Create a node that panics on the N-th tick.
    pub fn panic_on_tick(name: impl Into<String>, tick_num: u32) -> Box<dyn Node> {
        Box::new(Self {
            name: name.into(),
            init_behavior: InitBehavior::Ok,
            tick_behavior: TickBehavior::PanicOnTick(tick_num),
            shutdown_behavior: ShutdownBehavior::Ok,
            tick_count: 0,
        })
    }

    /// Create a node that tracks execution order to a shared vec.
    pub fn order_tracker(name: impl Into<String>, order: Arc<Mutex<Vec<String>>>) -> Box<dyn Node> {
        Box::new(Self {
            name: name.into(),
            init_behavior: InitBehavior::Ok,
            tick_behavior: TickBehavior::TrackOrder(order),
            shutdown_behavior: ShutdownBehavior::Ok,
            tick_count: 0,
        })
    }
}

/// Builder for constructing TestNode with custom behaviors.
pub struct TestNodeBuilder {
    name: String,
    init_behavior: InitBehavior,
    tick_behavior: TickBehavior,
    shutdown_behavior: ShutdownBehavior,
}

impl TestNodeBuilder {
    /// Set init to panic.
    pub fn fail_init(mut self) -> Self {
        self.init_behavior = InitBehavior::Panic("test init failure".to_string());
        self
    }

    /// Set init to panic with a custom message.
    pub fn panic_init(mut self, msg: impl Into<String>) -> Self {
        self.init_behavior = InitBehavior::Panic(msg.into());
        self
    }

    /// Set tick to count using a shared counter.
    pub fn count(mut self, counter: Arc<AtomicUsize>) -> Self {
        self.tick_behavior = TickBehavior::Count(counter);
        self
    }

    /// Set tick to sleep for a duration.
    pub fn tick_delay(mut self, delay: Duration) -> Self {
        self.tick_behavior = TickBehavior::Delay(delay);
        self
    }

    /// Set tick to panic on the N-th invocation (1-indexed).
    pub fn panic_on_tick(mut self, tick_num: u32) -> Self {
        self.tick_behavior = TickBehavior::PanicOnTick(tick_num);
        self
    }

    /// Set tick to record name to shared order tracker.
    pub fn track_order(mut self, order: Arc<Mutex<Vec<String>>>) -> Self {
        self.tick_behavior = TickBehavior::TrackOrder(order);
        self
    }

    /// Set tick to run a custom closure.
    pub fn custom_tick(mut self, f: impl Fn(&str, u32) + Send + Sync + 'static) -> Self {
        self.tick_behavior = TickBehavior::Custom(Arc::new(f));
        self
    }

    /// Set shutdown to panic.
    pub fn panic_shutdown(mut self, msg: impl Into<String>) -> Self {
        self.shutdown_behavior = ShutdownBehavior::Panic(msg.into());
        self
    }

    /// Build the TestNode as a boxed dyn Node.
    pub fn build(self) -> Box<dyn Node> {
        Box::new(TestNode {
            name: self.name,
            init_behavior: self.init_behavior,
            tick_behavior: self.tick_behavior,
            shutdown_behavior: self.shutdown_behavior,
            tick_count: 0,
        })
    }
}

impl Node for TestNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn init(&mut self) -> HorusResult<()> {
        match &self.init_behavior {
            InitBehavior::Ok => Ok(()),
            InitBehavior::Panic(msg) => panic!("{}", msg),
        }
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        match &self.tick_behavior {
            TickBehavior::Noop => {}
            TickBehavior::Count(counter) => {
                counter.fetch_add(1, Ordering::SeqCst);
            }
            TickBehavior::Delay(d) => {
                std::thread::sleep(*d);
            }
            TickBehavior::PanicOnTick(n) => {
                if self.tick_count >= *n {
                    panic!("intentional panic at tick {}", self.tick_count);
                }
            }
            TickBehavior::TrackOrder(order) => {
                order.lock().unwrap().push(self.name.clone());
            }
            TickBehavior::Custom(f) => {
                f(&self.name, self.tick_count);
            }
        }
    }

    fn shutdown(&mut self) -> HorusResult<()> {
        match &self.shutdown_behavior {
            ShutdownBehavior::Ok => Ok(()),
            ShutdownBehavior::Panic(msg) => panic!("{}", msg),
        }
    }
}

// Implement Debug for TickBehavior manually since closures don't implement Debug
impl std::fmt::Debug for TestNode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TestNode")
            .field("name", &self.name)
            .field("tick_count", &self.tick_count)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stub_node() {
        let mut node = TestNode::stub("stub");
        assert_eq!(node.name(), "stub");
        node.init().unwrap();
        node.tick();
        node.shutdown().unwrap();
    }

    #[test]
    fn test_counter_node() {
        let counter = Arc::new(AtomicUsize::new(0));
        let mut node = TestNode::counter("counter", counter.clone());
        node.init().unwrap();
        node.tick();
        node.tick();
        node.tick();
        assert_eq!(counter.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn test_slow_node() {
        let mut node = TestNode::slow("slow", Duration::from_millis(1));
        let start = std::time::Instant::now();
        node.tick();
        assert!(start.elapsed() >= Duration::from_millis(1));
    }

    #[test]
    #[should_panic(expected = "intentional panic at tick 2")]
    fn test_panic_on_tick() {
        let mut node = TestNode::panic_on_tick("crasher", 2);
        node.tick(); // tick 1 — ok
        node.tick(); // tick 2 — panic
    }

    #[test]
    fn test_order_tracker() {
        let order = Arc::new(Mutex::new(Vec::new()));
        let mut a = TestNode::order_tracker("a", order.clone());
        let mut b = TestNode::order_tracker("b", order.clone());

        a.tick();
        b.tick();
        a.tick();

        let recorded = order.lock().unwrap();
        assert_eq!(*recorded, vec!["a", "b", "a"]);
    }

    #[test]
    #[should_panic(expected = "test init failure")]
    fn test_fail_init() {
        let mut node = TestNode::builder("bad").fail_init().build();
        let _ = node.init();
    }

    #[test]
    #[should_panic(expected = "custom init panic")]
    fn test_custom_init_panic() {
        let mut node = TestNode::builder("bad")
            .panic_init("custom init panic")
            .build();
        let _ = node.init();
    }

    #[test]
    #[should_panic(expected = "shutdown boom")]
    fn test_panic_shutdown() {
        let mut node = TestNode::builder("bad")
            .panic_shutdown("shutdown boom")
            .build();
        let _ = node.init();
        let _ = node.shutdown();
    }

    #[test]
    fn test_custom_tick() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let calls_clone = calls.clone();
        let mut node = TestNode::builder("custom")
            .custom_tick(move |name, count| {
                calls_clone
                    .lock()
                    .unwrap()
                    .push(format!("{}:{}", name, count));
            })
            .build();

        node.tick();
        node.tick();

        let recorded = calls.lock().unwrap();
        assert_eq!(*recorded, vec!["custom:1", "custom:2"]);
    }

    #[test]
    fn test_builder_chaining() {
        let counter = Arc::new(AtomicUsize::new(0));
        let mut node = TestNode::builder("chain").count(counter.clone()).build();

        node.init().unwrap();
        node.tick();
        node.tick();
        node.shutdown().unwrap();

        assert_eq!(counter.load(Ordering::SeqCst), 2);
    }
}
