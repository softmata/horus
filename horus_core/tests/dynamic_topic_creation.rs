//! Integration tests for dynamic Topic creation during scheduler execution.
//!
//! Validates that Topic::new() can be called from within Node::tick() while
//! the scheduler is running. This is a common production pattern — nodes
//! discover sensors at runtime and create topics dynamically.
//!
//! **Gap addressed**: No existing test exercises Topic::new() inside tick().

mod common;

use common::cleanup_stale_shm;
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

fn unique(prefix: &str) -> String {
    use std::sync::atomic::AtomicU64;
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

// ============================================================================
// Nodes
// ============================================================================

/// Publisher that creates its topic lazily in the first tick(), not init().
struct LazyPublisher {
    name: &'static str,
    topic_name: String,
    topic: Option<Topic<u64>>,
    counter: Arc<AtomicU64>,
    tick_count: u64,
}

impl Node for LazyPublisher {
    fn name(&self) -> &'static str {
        self.name
    }
    fn tick(&mut self) {
        // Create topic on first tick
        if self.topic.is_none() {
            self.topic = Topic::new(&self.topic_name).ok();
        }
        if let Some(ref topic) = self.topic {
            self.tick_count += 1;
            topic.send(self.tick_count);
            self.counter.fetch_add(1, Ordering::SeqCst);
        }
    }
}

/// Publisher that creates its topic conditionally after N ticks.
struct ConditionalPublisher {
    name: &'static str,
    topic_name: String,
    topic: Option<Topic<u64>>,
    counter: Arc<AtomicU64>,
    tick_count: u64,
    create_at_tick: u64,
}

impl Node for ConditionalPublisher {
    fn name(&self) -> &'static str {
        self.name
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        // Create topic only after reaching the threshold tick
        if self.topic.is_none() && self.tick_count >= self.create_at_tick {
            self.topic = Topic::new(&self.topic_name).ok();
        }
        if let Some(ref topic) = self.topic {
            topic.send(self.tick_count);
            self.counter.fetch_add(1, Ordering::SeqCst);
        }
    }
}

/// Subscriber that creates its topic in init() (standard pattern).
struct EagerSubscriber {
    name: &'static str,
    topic_name: String,
    topic: Option<Topic<u64>>,
    received: Arc<AtomicU64>,
    latest: Arc<AtomicU64>,
}

impl Node for EagerSubscriber {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref mut topic) = self.topic {
            while let Some(val) = topic.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
                self.latest.store(val, Ordering::SeqCst);
            }
        }
    }
}

/// Subscriber that creates its topic lazily in first tick().
struct LazySubscriber {
    name: &'static str,
    topic_name: String,
    topic: Option<Topic<u64>>,
    received: Arc<AtomicU64>,
}

impl Node for LazySubscriber {
    fn name(&self) -> &'static str {
        self.name
    }
    fn tick(&mut self) {
        if self.topic.is_none() {
            self.topic = Topic::new(&self.topic_name).ok();
        }
        if let Some(ref mut topic) = self.topic {
            while let Some(_val) = topic.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
            }
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

/// Test: Publisher creates Topic in tick() (not init()), subscriber creates
/// in init(). Data should flow once the publisher's lazy topic is created.
#[test]
fn lazy_publisher_eager_subscriber() {
    cleanup_stale_shm();

    let topic_name = unique("dyn_lazy_pub");
    let pub_count = Arc::new(AtomicU64::new(0));
    let sub_received = Arc::new(AtomicU64::new(0));
    let sub_latest = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(LazyPublisher {
            name: "lazy_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
            tick_count: 0,
        })
        .order(0)
        .build();

    scheduler
        .add(EagerSubscriber {
            name: "eager_sub",
            topic_name: topic_name.clone(),
            topic: None,
            received: sub_received.clone(),
            latest: sub_latest.clone(),
        })
        .order(1)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(result.is_ok(), "Should not error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    let received = sub_received.load(Ordering::SeqCst);

    assert!(
        published > 10,
        "Lazy publisher should have sent >10 messages, got {published}"
    );
    assert!(
        received > 0,
        "Subscriber should receive from lazily-created topic, got {received}"
    );
}

/// Test: Publisher creates topic only after tick 20. Subscriber is eager.
/// Assert: no messages before tick 20, messages flow after.
#[test]
fn conditional_topic_creation_after_n_ticks() {
    cleanup_stale_shm();

    let topic_name = unique("dyn_conditional");
    let pub_count = Arc::new(AtomicU64::new(0));
    let sub_received = Arc::new(AtomicU64::new(0));
    let sub_latest = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(ConditionalPublisher {
            name: "cond_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
            tick_count: 0,
            create_at_tick: 20,
        })
        .order(0)
        .build();

    scheduler
        .add(EagerSubscriber {
            name: "cond_sub",
            topic_name: topic_name.clone(),
            topic: None,
            received: sub_received.clone(),
            latest: sub_latest.clone(),
        })
        .order(1)
        .build();

    let result = scheduler.run_for(1_u64.secs());
    assert!(result.is_ok(), "Should not error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    let received = sub_received.load(Ordering::SeqCst);
    let latest = sub_latest.load(Ordering::SeqCst);

    assert!(
        published > 5,
        "Should publish after tick 20, got {published} publishes"
    );
    assert!(
        received > 0,
        "Should receive after conditional creation, got {received}"
    );
    // Latest value should be > 20 (topic created at tick 20, values are tick_count)
    assert!(
        latest >= 20,
        "Latest value should be >= 20 (created at tick 20), got {latest}"
    );
}

/// Test: Publisher creates topic in init(), subscriber creates in tick().
/// Both should communicate — the subscriber discovers the existing topic
/// when it creates its handle in tick().
#[test]
fn eager_publisher_lazy_subscriber() {
    cleanup_stale_shm();

    let topic_name = unique("dyn_lazy_sub");
    let pub_count = Arc::new(AtomicU64::new(0));
    let sub_received = Arc::new(AtomicU64::new(0));

    /// Standard publisher using init() for topic creation.
    struct EagerPublisher {
        name: &'static str,
        topic_name: String,
        topic: Option<Topic<u64>>,
        counter: Arc<AtomicU64>,
        tick_count: u64,
    }

    impl Node for EagerPublisher {
        fn name(&self) -> &'static str {
            self.name
        }
        fn init(&mut self) -> horus_core::error::Result<()> {
            self.topic = Some(Topic::new(&self.topic_name)?);
            Ok(())
        }
        fn tick(&mut self) {
            self.tick_count += 1;
            if let Some(ref topic) = self.topic {
                topic.send(self.tick_count);
                self.counter.fetch_add(1, Ordering::SeqCst);
            }
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(EagerPublisher {
            name: "eager_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
            tick_count: 0,
        })
        .order(0)
        .build();

    scheduler
        .add(LazySubscriber {
            name: "lazy_sub",
            topic_name: topic_name.clone(),
            topic: None,
            received: sub_received.clone(),
        })
        .order(1)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(result.is_ok(), "Should not error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    let received = sub_received.load(Ordering::SeqCst);

    assert!(published > 10, "Publisher should send >10, got {published}");
    assert!(
        received > 0,
        "Lazy subscriber should receive from existing topic, got {received}"
    );
}
