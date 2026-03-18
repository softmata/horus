//! Integration tests for multi-process scheduler coordination via SHM topics.
//!
//! Validates that two separate processes, each running their own Scheduler with
//! nodes, can communicate via shared memory topics. This is the production
//! pattern of `horus run sensor.rs controller.rs`.
//!
//! **Gap addressed**: Existing cross-process tests (cross_process_ipc.rs) test
//! raw Topic send/recv without schedulers. This test runs a full Scheduler in
//! each process.

mod common;

use common::cleanup_stale_shm;
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Env var flag to detect child process.
const CHILD_ENV: &str = "HORUS_MULTI_SCHED_CHILD";
/// Env var carrying the topic name.
const TOPIC_ENV: &str = "HORUS_MULTI_SCHED_TOPIC";
/// Env var carrying the test function name.
const TEST_ENV: &str = "HORUS_MULTI_SCHED_TEST";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

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

/// Publisher node: sends incrementing counter via Topic.
struct ScheduledPublisher {
    name: &'static str,
    topic_name: String,
    topic: Option<Topic<u64>>,
    counter: Arc<AtomicU64>,
}

impl Node for ScheduledPublisher {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let val = self.counter.fetch_add(1, Ordering::SeqCst) + 1;
        if let Some(ref topic) = self.topic {
            topic.send(val);
        }
    }
}

/// Subscriber node: receives from Topic, counts messages.
struct ScheduledSubscriber {
    name: &'static str,
    topic_name: String,
    topic: Option<Topic<u64>>,
    received: Arc<AtomicU64>,
}

impl Node for ScheduledSubscriber {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref mut topic) = self.topic {
            while let Some(_val) = topic.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
            }
        }
    }
}

// ============================================================================
// Child process entry points
// ============================================================================

/// Child runs a Scheduler with a SubscriberNode, prints received count.
fn child_subscriber_scheduler() {
    let topic_name = std::env::var(TOPIC_ENV).expect("HORUS_MULTI_SCHED_TOPIC not set");

    let received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledSubscriber {
            name: "child_sub",
            topic_name,
            topic: None,
            received: received.clone(),
        })
        .order(0)
        .build();

    // Run for 1.5 seconds (parent runs 2 seconds, starts 500ms before us)
    let _ = scheduler.run_for(1500_u64.ms());

    let count = received.load(Ordering::SeqCst);
    println!("CHILD_RECEIVED:{count}");
}

/// Child runs a Scheduler with BOTH a publisher and subscriber for bidirectional test.
fn child_bidirectional_scheduler() {
    let topic_a = std::env::var("HORUS_MULTI_SCHED_TOPIC_A").expect("TOPIC_A not set");
    let topic_b = std::env::var("HORUS_MULTI_SCHED_TOPIC_B").expect("TOPIC_B not set");

    let pub_count = Arc::new(AtomicU64::new(0));
    let sub_received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // Child subscribes to topic_a (parent publishes)
    scheduler
        .add(ScheduledSubscriber {
            name: "child_sub_a",
            topic_name: topic_a,
            topic: None,
            received: sub_received.clone(),
        })
        .order(0)
        .build();

    // Child publishes to topic_b (parent subscribes)
    scheduler
        .add(ScheduledPublisher {
            name: "child_pub_b",
            topic_name: topic_b,
            topic: None,
            counter: pub_count.clone(),
        })
        .order(1)
        .build();

    let _ = scheduler.run_for(1500_u64.ms());

    let recv = sub_received.load(Ordering::SeqCst);
    let sent = pub_count.load(Ordering::SeqCst);
    println!("CHILD_RECEIVED:{recv}");
    println!("CHILD_SENT:{sent}");
}

// ============================================================================
// Helper
// ============================================================================

fn spawn_child_scheduler(test_name: &str, envs: Vec<(&str, String)>) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TEST_ENV, test_name)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    for (key, val) in envs {
        cmd.env(key, val);
    }

    cmd.spawn()
        .expect("failed to spawn child scheduler process")
}

fn parse_child_received(stdout: &str) -> u64 {
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("CHILD_RECEIVED:") {
            return n.parse().unwrap_or(0);
        }
    }
    0
}

fn parse_child_sent(stdout: &str) -> u64 {
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("CHILD_SENT:") {
            return n.parse().unwrap_or(0);
        }
    }
    0
}

// ============================================================================
// Tests
// ============================================================================

/// Test: Parent scheduler publishes, child scheduler subscribes.
/// Two independent Scheduler instances in separate processes communicating
/// via a shared SHM topic.
#[test]
fn two_schedulers_parent_publishes_child_subscribes() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "two_schedulers_parent_publishes_child_subscribes" {
            child_subscriber_scheduler();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_uni");

    // Spawn child first so it's ready to receive
    let child = spawn_child_scheduler(
        "two_schedulers_parent_publishes_child_subscribes",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    // Wait for child to start and create its Topic
    std::thread::sleep(500_u64.ms());

    // Parent scheduler with publisher
    let pub_count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledPublisher {
            name: "parent_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
        })
        .order(0)
        .build();

    let result = scheduler.run_for(2_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    assert!(published > 50, "Parent should publish >50, got {published}");

    // Wait for child to finish
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    let child_received = parse_child_received(&stdout);

    // Child may not receive ALL messages (cross-process migration takes time)
    // but should receive SOME messages after migration completes.
    assert!(
        child_received > 0,
        "Child scheduler should receive messages via SHM. Published: {published}, child received: {child_received}. Stdout: {stdout}"
    );
}

/// Test: Bidirectional — parent publishes on topic_a, child publishes on topic_b.
/// Both subscribe to the other's topic. Validates two-way cross-process IPC
/// with independent schedulers.
#[test]
fn two_schedulers_bidirectional() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "two_schedulers_bidirectional" {
            child_bidirectional_scheduler();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_a = unique("multi_sched_a");
    let topic_b = unique("multi_sched_b");

    // Spawn child
    let child = spawn_child_scheduler(
        "two_schedulers_bidirectional",
        vec![
            ("HORUS_MULTI_SCHED_TOPIC_A", topic_a.clone()),
            ("HORUS_MULTI_SCHED_TOPIC_B", topic_b.clone()),
        ],
    );

    std::thread::sleep(500_u64.ms());

    // Parent: publishes to topic_a, subscribes to topic_b
    let pub_count = Arc::new(AtomicU64::new(0));
    let parent_received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(ScheduledPublisher {
            name: "parent_pub_a",
            topic_name: topic_a.clone(),
            topic: None,
            counter: pub_count.clone(),
        })
        .order(0)
        .build();

    scheduler
        .add(ScheduledSubscriber {
            name: "parent_sub_b",
            topic_name: topic_b.clone(),
            topic: None,
            received: parent_received.clone(),
        })
        .order(1)
        .build();

    let result = scheduler.run_for(2_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    let parent_recv = parent_received.load(Ordering::SeqCst);

    // Wait for child
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    let child_received = parse_child_received(&stdout);
    let child_sent = parse_child_sent(&stdout);

    assert!(
        published > 50,
        "Parent should publish >50 on topic_a, got {published}"
    );
    assert!(
        child_sent > 50,
        "Child should publish >50 on topic_b, got {child_sent}"
    );

    // Both should receive from the other (bidirectional)
    assert!(
        child_received > 0,
        "Child should receive from parent via topic_a. Parent published: {published}, child received: {child_received}"
    );
    assert!(
        parent_recv > 0,
        "Parent should receive from child via topic_b. Child sent: {child_sent}, parent received: {parent_recv}"
    );
}
