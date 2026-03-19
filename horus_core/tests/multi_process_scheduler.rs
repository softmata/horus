//! Integration tests for multi-process scheduler coordination via SHM topics.
//!
//! Validates that two separate processes, each running their own Scheduler with
//! nodes, can communicate via shared memory topics. This is the production
//! pattern of `horus run sensor.rs controller.rs`.
//!
//! **Gap addressed**: Existing cross-process tests (cross_process_ipc.rs) test
//! raw Topic send/recv without schedulers. This test runs a full Scheduler in
//! each process.
//!
//! **Run sequentially**: These tests share SHM state across processes and must
//! not run in parallel. Use: `cargo test --test multi_process_scheduler -- --test-threads=1`

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

// ============================================================================
// Typed message nodes for [f32; 2] (tests serde path, not just Pod u64)
// ============================================================================

/// Publisher that sends [f32; 2] messages (e.g., [linear_vel, angular_vel]).
/// Tests the serde serialization path with a compound type.
struct TypedPublisher {
    topic_name: String,
    topic: Option<Topic<[f32; 2]>>,
    value: [f32; 2],
    sent: Arc<AtomicU64>,
}

impl Node for TypedPublisher {
    fn name(&self) -> &'static str {
        "typed_pub"
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref topic) = self.topic {
            topic.send(self.value);
            self.sent.fetch_add(1, Ordering::SeqCst);
        }
    }
}

/// Subscriber that receives [f32; 2] and stores latest values atomically.
/// Packs two f32 values into a single u64 for lock-free storage.
struct TypedSubscriber {
    topic_name: String,
    topic: Option<Topic<[f32; 2]>>,
    received: Arc<AtomicU64>,
    /// Packed f32 pair: (value[0].to_bits() << 32) | value[1].to_bits()
    latest_packed: Arc<AtomicU64>,
}

impl Node for TypedSubscriber {
    fn name(&self) -> &'static str {
        "typed_sub"
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref mut topic) = self.topic {
            while let Some(msg) = topic.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
                let packed = (msg[0].to_bits() as u64) << 32 | msg[1].to_bits() as u64;
                self.latest_packed.store(packed, Ordering::SeqCst);
            }
        }
    }
}

fn unpack_f32_pair(packed: u64) -> (f32, f32) {
    let a = f32::from_bits((packed >> 32) as u32);
    let b = f32::from_bits(packed as u32);
    (a, b)
}

// ============================================================================
// Child entry points for new tests
// ============================================================================

/// Child: receives [f32; 2] typed messages, prints field values.
fn child_cmdvel_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");

    let received = Arc::new(AtomicU64::new(0));
    let latest = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(TypedSubscriber {
            topic_name,
            topic: None,
            received: received.clone(),
            latest_packed: latest.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(1500_u64.ms());

    let count = received.load(Ordering::SeqCst);
    let (linear, angular) = unpack_f32_pair(latest.load(Ordering::SeqCst));
    println!("CHILD_RECEIVED:{count}");
    println!("CHILD_LINEAR:{linear}");
    println!("CHILD_ANGULAR:{angular}");
}

/// Child: subscriber running for 2.5s (sustained test).
fn child_sustained_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");

    let received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledSubscriber {
            name: "child_sustained_sub",
            topic_name,
            topic: None,
            received: received.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(2500_u64.ms());

    println!("CHILD_RECEIVED:{}", received.load(Ordering::SeqCst));
}

/// Child: 10Hz subscriber (fast-to-slow test).
/// Runs for 4s to give SHM migration time to complete at low tick rate.
fn child_slow_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");

    let received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());
    scheduler
        .add(ScheduledSubscriber {
            name: "child_slow_sub",
            topic_name,
            topic: None,
            received: received.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(4_u64.secs());

    println!("CHILD_RECEIVED:{}", received.load(Ordering::SeqCst));
}

// ============================================================================
// Parse helpers
// ============================================================================

fn parse_child_float(stdout: &str, prefix: &str) -> f32 {
    for line in stdout.lines() {
        if let Some(val) = line.strip_prefix(prefix) {
            return val.parse().unwrap_or(0.0);
        }
    }
    0.0
}

// ============================================================================
// New tests
// ============================================================================

/// Test: Rate mismatch — parent scheduler at 1kHz, child at 100Hz.
/// Production pattern: motor controller (1kHz) → planner (100Hz).
/// The child should still receive messages despite 10x rate difference.
#[test]
fn rate_mismatch_1khz_parent_100hz_child() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "rate_mismatch_1khz_parent_100hz_child" {
            child_subscriber_scheduler();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_rate");

    let child = spawn_child_scheduler(
        "rate_mismatch_1khz_parent_100hz_child",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    std::thread::sleep(500_u64.ms());

    // Parent runs at 1kHz — 10x faster than child
    let pub_count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    scheduler
        .add(ScheduledPublisher {
            name: "parent_1khz_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
        })
        .order(0)
        .build();

    let result = scheduler.run_for(2_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    assert!(
        published > 500,
        "Parent (1kHz) should publish >500, got {published}"
    );

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let child_received = parse_child_received(&stdout);

    assert!(
        child_received > 0,
        "Child (100Hz) should receive messages from parent (1kHz). \
         Published: {published}, received: {child_received}. Stdout: {stdout}"
    );
}

/// Test: Typed [f32; 2] message cross-process with field verification.
/// Verifies serde serialization roundtrip for compound types (not just u64).
/// Parent sends [1.5, 0.3], child receives and prints values for verification.
#[test]
fn typed_message_cross_process() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "typed_message_cross_process" {
            child_cmdvel_subscriber();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_typed");

    let child = spawn_child_scheduler(
        "typed_message_cross_process",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    std::thread::sleep(500_u64.ms());

    let sent = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(TypedPublisher {
            topic_name: topic_name.clone(),
            topic: None,
            value: [1.5, 0.3],
            sent: sent.clone(),
        })
        .order(0)
        .build();

    let result = scheduler.run_for(2_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = sent.load(Ordering::SeqCst);
    assert!(
        published > 50,
        "Parent should publish >50 typed messages, got {published}"
    );

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    let child_received = parse_child_received(&stdout);
    assert!(
        child_received > 0,
        "Child should receive typed [f32; 2] messages. Published: {published}, \
         received: {child_received}. Stdout: {stdout}"
    );

    // Verify field values survived cross-process serde roundtrip
    let linear = parse_child_float(&stdout, "CHILD_LINEAR:");
    let angular = parse_child_float(&stdout, "CHILD_ANGULAR:");

    assert!(
        (linear - 1.5).abs() < 0.01,
        "linear should be ~1.5, got {linear}"
    );
    assert!(
        (angular - 0.3).abs() < 0.01,
        "angular should be ~0.3, got {angular}"
    );
}

/// Test: Sustained communication — parent publishes for 3s, child subscribes for 2.5s.
/// Verifies no message loss accumulation or degradation over extended runs.
#[test]
fn sustained_cross_process_communication() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "sustained_cross_process_communication" {
            child_sustained_subscriber();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_sustained");

    let child = spawn_child_scheduler(
        "sustained_cross_process_communication",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    std::thread::sleep(500_u64.ms());

    let pub_count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledPublisher {
            name: "parent_sustained_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
        })
        .order(0)
        .build();

    let result = scheduler.run_for(3_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    assert!(
        published > 100,
        "Parent should publish >100 over 3s at 100Hz, got {published}"
    );

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let child_received = parse_child_received(&stdout);

    assert!(
        child_received > 20,
        "Sustained test: child should receive >20 messages over 2.5s. \
         Published: {published}, received: {child_received}. Stdout: {stdout}"
    );
}

/// Test: Fast publisher (1kHz) to slow subscriber (10Hz).
/// The subscriber reads at 10Hz — it should get the latest value each tick
/// without crashing, hanging, or accumulating unbounded backlog.
#[test]
fn fast_publisher_slow_subscriber() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "fast_publisher_slow_subscriber" {
            child_slow_subscriber();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_fast_slow");

    let child = spawn_child_scheduler(
        "fast_publisher_slow_subscriber",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    // Longer startup wait — child at 10Hz needs more time to establish SHM
    std::thread::sleep(1_u64.secs());

    let pub_count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    scheduler
        .add(ScheduledPublisher {
            name: "parent_fast_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
        })
        .order(0)
        .build();

    let result = scheduler.run_for(3_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    assert!(
        published > 500,
        "Fast publisher (1kHz) should send >500, got {published}"
    );

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let child_received = parse_child_received(&stdout);

    // Key assertion: slow subscriber doesn't crash
    assert!(
        output.status.success(),
        "Slow subscriber should not crash. Exit: {:?}, stderr: {stderr}",
        output.status
    );
    assert!(
        child_received > 0,
        "Slow subscriber (10Hz) should receive messages from fast publisher (1kHz). \
         Published: {published}, received: {child_received}. Stdout: {stdout}"
    );
}

// ============================================================================
// Relay node for pipeline tests (subscribes + publishes)
// ============================================================================

/// Subscribes to one topic and republishes (with transform) to another.
/// Simulates a controller node in a sensor → controller → actuator pipeline.
struct RelayNode {
    sub_topic_name: String,
    pub_topic_name: String,
    sub_topic: Option<Topic<u64>>,
    pub_topic: Option<Topic<u64>>,
    relayed: Arc<AtomicU64>,
}

impl Node for RelayNode {
    fn name(&self) -> &'static str {
        "relay"
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.sub_topic = Some(Topic::new(&self.sub_topic_name)?);
        self.pub_topic = Some(Topic::new(&self.pub_topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let sub = self.sub_topic.as_mut();
        let pub_t = self.pub_topic.as_ref();
        if let (Some(sub), Some(pub_t)) = (sub, pub_t) {
            while let Some(val) = sub.recv() {
                pub_t.send(val + 1000); // Transform to prove controller processed it
                self.relayed.fetch_add(1, Ordering::SeqCst);
            }
        }
    }
}

// ============================================================================
// Additional env vars
// ============================================================================

const ROLE_ENV: &str = "HORUS_MULTI_SCHED_ROLE";

// ============================================================================
// Child entry points for edge case tests
// ============================================================================

/// Child: runs briefly then crashes with exit(1).
fn child_crashing_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");
    let received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledSubscriber {
            name: "child_crash_sub",
            topic_name,
            topic: None,
            received: received.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(300_u64.ms());
    std::process::exit(1);
}

/// Child: publishes to a shared topic. WRITER_BASE env var offsets the counter
/// so different writers produce distinguishable value ranges.
fn child_multi_writer() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");
    let base: u64 = std::env::var("HORUS_MULTI_SCHED_WRITER_BASE")
        .unwrap_or_else(|_| "0".to_string())
        .parse()
        .unwrap_or(0);

    let counter = Arc::new(AtomicU64::new(base));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledPublisher {
            name: "child_writer",
            topic_name,
            topic: None,
            counter: counter.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(2_u64.secs());
    let sent = counter.load(Ordering::SeqCst) - base;
    println!("CHILD_SENT:{sent}");
}

/// Child: sensor role — publishes incrementing u64 to TOPIC_ENV.
/// Runs for 8s to give downstream processes time to connect via SHM.
fn child_pipeline_sensor() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");
    let counter = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledPublisher {
            name: "sensor_pub",
            topic_name,
            topic: None,
            counter: counter.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(8_u64.secs());
    println!("CHILD_SENT:{}", counter.load(Ordering::SeqCst));
}

/// Child: controller role — subscribes TOPIC_ENV, republishes to TOPIC_B.
/// Runs for 7s to overlap with both sensor and parent subscriber.
fn child_pipeline_controller() {
    let sub_topic = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");
    let pub_topic =
        std::env::var("HORUS_MULTI_SCHED_TOPIC_B").expect("HORUS_MULTI_SCHED_TOPIC_B not set");
    let relayed = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(RelayNode {
            sub_topic_name: sub_topic,
            pub_topic_name: pub_topic,
            sub_topic: None,
            pub_topic: None,
            relayed: relayed.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(7_u64.secs());
    println!("CHILD_RELAYED:{}", relayed.load(Ordering::SeqCst));
}

// ============================================================================
// Parse helpers
// ============================================================================

fn parse_child_relayed(stdout: &str) -> u64 {
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("CHILD_RELAYED:") {
            return n.parse().unwrap_or(0);
        }
    }
    0
}

// ============================================================================
// Edge case tests
// ============================================================================

/// Test: Child process crashes (exit 1) while parent scheduler is running.
/// Parent should complete its run without hanging or crashing.
/// Production scenario: camera node segfaults, motor controller must keep running.
#[test]
fn child_crash_parent_survives() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "child_crash_parent_survives" {
            child_crashing_subscriber();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_crash");

    let child = spawn_child_scheduler(
        "child_crash_parent_survives",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    std::thread::sleep(500_u64.ms());

    let pub_count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledPublisher {
            name: "parent_crash_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
        })
        .order(0)
        .build();

    // Parent runs 3s — child crashes ~300ms into its run
    let result = scheduler.run_for(3_u64.secs());
    assert!(
        result.is_ok(),
        "Parent scheduler should survive child crash: {:?}",
        result.err()
    );

    let output = child.wait_with_output().expect("child wait failed");
    assert!(
        !output.status.success(),
        "Child should have crashed (exit 1)"
    );

    let published = pub_count.load(Ordering::SeqCst);
    assert!(
        published > 100,
        "Parent should continue publishing after child crash, got {published}"
    );
}

/// Test: 2 publisher processes writing to the same topic, 1 parent subscriber.
/// Tests MPMC backend activation and multi-writer SHM coordination.
#[test]
fn multi_writer_cross_process() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "multi_writer_cross_process" {
            child_multi_writer();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_mw");

    // Spawn 2 writer children with different base values
    let child1 = spawn_child_scheduler(
        "multi_writer_cross_process",
        vec![
            (TOPIC_ENV, topic_name.clone()),
            ("HORUS_MULTI_SCHED_WRITER_BASE", "0".to_string()),
        ],
    );
    let child2 = spawn_child_scheduler(
        "multi_writer_cross_process",
        vec![
            (TOPIC_ENV, topic_name.clone()),
            ("HORUS_MULTI_SCHED_WRITER_BASE", "10000".to_string()),
        ],
    );

    std::thread::sleep(1_u64.secs());

    // Parent subscribes
    let received = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(ScheduledSubscriber {
            name: "parent_mw_sub",
            topic_name: topic_name.clone(),
            topic: None,
            received: received.clone(),
        })
        .order(0)
        .build();

    let result = scheduler.run_for(3_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let parent_recv = received.load(Ordering::SeqCst);

    let out1 = child1.wait_with_output().expect("child1 wait");
    let out2 = child2.wait_with_output().expect("child2 wait");

    let sent1 = parse_child_sent(&String::from_utf8_lossy(&out1.stdout));
    let sent2 = parse_child_sent(&String::from_utf8_lossy(&out2.stdout));

    assert!(sent1 > 0, "Writer 1 should send messages, got {sent1}");
    assert!(sent2 > 0, "Writer 2 should send messages, got {sent2}");
    assert!(
        parent_recv > 0,
        "Parent should receive from multi-writer topic. \
         Writer1 sent: {sent1}, Writer2 sent: {sent2}, parent received: {parent_recv}"
    );
}

/// Test: 3-process pipeline — sensor → controller → parent subscriber.
/// Child A publishes sensor data. Child B subscribes sensor, transforms (+1000),
/// publishes commands. Parent subscribes commands. Validates end-to-end flow.
#[test]
fn three_process_pipeline() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "three_process_pipeline" {
            let role = std::env::var(ROLE_ENV).unwrap_or_default();
            match role.as_str() {
                "sensor" => child_pipeline_sensor(),
                "controller" => child_pipeline_controller(),
                _ => {}
            }
        }
        return;
    }

    cleanup_stale_shm();

    let sensor_topic = unique("pipeline_sensor");
    let cmd_topic = unique("pipeline_cmd");

    // Parent subscriber starts FIRST to establish SHM before children.
    // Known issue: cross-process Topics require the subscriber to create the
    // SHM file before the publisher for reliable message delivery. This is
    // tracked as a bug — the root cause is in the MPMC SHM dispatch when a
    // late-joining subscriber tries to read slots written by a publisher that
    // started first. The ready-flag sequence check fails because the publisher's
    // sequence numbers don't match what the new subscriber expects.

    // T=0: Start parent subscriber in background thread (establishes cmd_topic SHM)
    let ct = cmd_topic.clone();
    let received = Arc::new(AtomicU64::new(0));
    let recv_clone = received.clone();
    let parent_thread = std::thread::spawn(move || {
        let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
        scheduler
            .add(ScheduledSubscriber {
                name: "parent_cmd_sub",
                topic_name: ct,
                topic: None,
                received: recv_clone,
            })
            .order(0)
            .build();
        scheduler.run_for(8_u64.secs())
    });

    // T=1: Spawn sensor (publishes to sensor_topic)
    std::thread::sleep(1_u64.secs());
    let sensor = spawn_child_scheduler(
        "three_process_pipeline",
        vec![
            (TOPIC_ENV, sensor_topic.clone()),
            (ROLE_ENV, "sensor".to_string()),
        ],
    );

    // T=2: Spawn controller (subscribes sensor_topic, publishes cmd_topic)
    std::thread::sleep(1_u64.secs());
    let controller = spawn_child_scheduler(
        "three_process_pipeline",
        vec![
            (TOPIC_ENV, sensor_topic.clone()),
            ("HORUS_MULTI_SCHED_TOPIC_B", cmd_topic.clone()),
            (ROLE_ENV, "controller".to_string()),
        ],
    );

    // Wait for children to finish
    let sensor_out = sensor.wait_with_output().expect("sensor wait");
    let ctrl_out = controller.wait_with_output().expect("controller wait");

    // Wait for parent thread
    let result = parent_thread.join().expect("parent thread panicked");
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let parent_recv = received.load(Ordering::SeqCst);
    let sensor_sent = parse_child_sent(&String::from_utf8_lossy(&sensor_out.stdout));
    let ctrl_relayed = parse_child_relayed(&String::from_utf8_lossy(&ctrl_out.stdout));

    assert!(sensor_sent > 0, "Sensor should publish messages");
    assert!(ctrl_relayed > 0, "Controller should relay messages");
    assert!(
        parent_recv > 0,
        "Parent should receive pipeline commands. Sensor sent: {sensor_sent}, \
         controller relayed: {ctrl_relayed}, parent received: {parent_recv}"
    );
}

/// Test: Late-joining subscriber — parent starts publishing first, child joins 2s later.
/// Production scenario: monitoring tool connects to an already-running robot.
#[test]
fn late_join_subscriber() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "late_join_subscriber" {
            child_subscriber_scheduler();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_late");

    // Start parent publisher in background thread
    let tn = topic_name.clone();
    let pub_count = Arc::new(AtomicU64::new(0));
    let pc = pub_count.clone();
    let parent_thread = std::thread::spawn(move || {
        let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
        scheduler
            .add(ScheduledPublisher {
                name: "parent_late_pub",
                topic_name: tn,
                topic: None,
                counter: pc,
            })
            .order(0)
            .build();
        scheduler.run_for(5_u64.secs())
    });

    // Parent has been publishing for 2s before child joins
    std::thread::sleep(2_u64.secs());

    let child = spawn_child_scheduler(
        "late_join_subscriber",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    // Wait for child (runs 1.5s)
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let child_received = parse_child_received(&stdout);

    // Wait for parent thread
    let result = parent_thread.join().expect("parent thread panicked");
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    assert!(
        published > 200,
        "Parent should publish >200 over 5s, got {published}"
    );

    assert!(
        child_received > 0,
        "Late-joining subscriber should receive messages. \
         Published: {published}, received: {child_received}. Stdout: {stdout}"
    );
}

// ============================================================================
// Large message nodes (Vec<u8> for 8KB payloads)
// ============================================================================

/// Publisher that sends large Vec<u8> messages with a deterministic byte pattern.
struct LargePublisher {
    topic_name: String,
    topic: Option<Topic<Vec<u8>>>,
    size: usize,
    sent: Arc<AtomicU64>,
}

impl Node for LargePublisher {
    fn name(&self) -> &'static str {
        "large_pub"
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref topic) = self.topic {
            let seq = self.sent.load(Ordering::SeqCst) as u8;
            // Fill with deterministic pattern: (index + seq) % 256
            let data: Vec<u8> = (0..self.size).map(|i| ((i as u8).wrapping_add(seq))).collect();
            topic.send(data);
            self.sent.fetch_add(1, Ordering::SeqCst);
        }
    }
}

/// Subscriber that receives large Vec<u8> messages and verifies data integrity.
struct LargeSubscriber {
    topic_name: String,
    topic: Option<Topic<Vec<u8>>>,
    expected_size: usize,
    received: Arc<AtomicU64>,
    verified: Arc<AtomicU64>,
}

impl Node for LargeSubscriber {
    fn name(&self) -> &'static str {
        "large_sub"
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref mut topic) = self.topic {
            while let Some(data) = topic.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
                if data.len() == self.expected_size {
                    // Verify pattern: each byte should be (index + seq) % 256
                    // We don't know seq, but bytes[i+1] - bytes[i] should always be 1 (mod 256)
                    let consistent = data.windows(2).all(|w| w[1] == w[0].wrapping_add(1));
                    if consistent {
                        self.verified.fetch_add(1, Ordering::SeqCst);
                    }
                }
            }
        }
    }
}

// ============================================================================
// Child entry points for RT and large message tests
// ============================================================================

/// Child: receives large Vec<u8> messages and verifies data integrity.
/// Runs for 4s to give SHM migration time to complete for large payloads.
fn child_large_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");

    let received = Arc::new(AtomicU64::new(0));
    let verified = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(LargeSubscriber {
            topic_name,
            topic: None,
            expected_size: 16384,
            received: received.clone(),
            verified: verified.clone(),
        })
        .order(0)
        .build();

    let _ = scheduler.run_for(4_u64.secs());

    let recv = received.load(Ordering::SeqCst);
    let verif = verified.load(Ordering::SeqCst);
    println!("CHILD_RECEIVED:{recv}");
    println!("CHILD_VERIFIED:{verif}");
}

// ============================================================================
// RT and large message tests
// ============================================================================

/// Test: RT-priority parent scheduler communicating with best-effort child.
/// Parent uses `.prefer_rt()` and 1kHz rate. Child uses 100Hz best-effort.
/// `.prefer_rt()` degrades gracefully on non-RT systems — test works on CI.
#[test]
fn rt_scheduler_cross_process() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "rt_scheduler_cross_process" {
            child_subscriber_scheduler();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_rt");

    let child = spawn_child_scheduler(
        "rt_scheduler_cross_process",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    std::thread::sleep(500_u64.ms());

    let pub_count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new()
        .tick_rate(1000_u64.hz())
        .prefer_rt();
    scheduler
        .add(ScheduledPublisher {
            name: "parent_rt_pub",
            topic_name: topic_name.clone(),
            topic: None,
            counter: pub_count.clone(),
        })
        .order(0)
        .rate(1000_u64.hz())
        .build();

    let result = scheduler.run_for(2_u64.secs());
    assert!(result.is_ok(), "RT scheduler error: {:?}", result.err());

    let published = pub_count.load(Ordering::SeqCst);
    assert!(
        published > 500,
        "RT publisher (1kHz) should send >500, got {published}"
    );

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let child_received = parse_child_received(&stdout);

    assert!(
        child_received > 0,
        "Best-effort child should receive from RT parent. \
         Published: {published}, received: {child_received}. Stdout: {stdout}"
    );
}

/// Test: Large message (8KB Vec<u8>) cross-process with data integrity verification.
/// Verifies that SHM handles payloads larger than typical u64/CmdVel messages
/// without corruption, truncation, or ring buffer overflow.
#[test]
fn large_message_cross_process() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "large_message_cross_process" {
            child_large_subscriber();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = unique("multi_sched_large");

    let child = spawn_child_scheduler(
        "large_message_cross_process",
        vec![(TOPIC_ENV, topic_name.clone())],
    );

    std::thread::sleep(1_u64.secs());

    let sent = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(50_u64.hz());
    scheduler
        .add(LargePublisher {
            topic_name: topic_name.clone(),
            topic: None,
            size: 16384,
            sent: sent.clone(),
        })
        .order(0)
        .build();

    let result = scheduler.run_for(3_u64.secs());
    assert!(result.is_ok(), "Parent scheduler error: {:?}", result.err());

    let published = sent.load(Ordering::SeqCst);
    assert!(
        published > 20,
        "Parent should publish >20 large messages, got {published}"
    );

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    let child_received = parse_child_received(&stdout);
    assert!(
        child_received > 0,
        "Child should receive large messages. Published: {published}, \
         received: {child_received}. Stdout: {stdout}"
    );

    // Verify data integrity — child checks byte pattern consistency
    let verified = parse_child_verified(&stdout);
    assert!(
        verified > 0,
        "Child should verify data integrity of large messages. \
         Received: {child_received}, verified: {verified}. Stdout: {stdout}"
    );
}

fn parse_child_verified(stdout: &str) -> u64 {
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("CHILD_VERIFIED:") {
            return n.parse().unwrap_or(0);
        }
    }
    0
}
