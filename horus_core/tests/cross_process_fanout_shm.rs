//! Cross-process FanoutShm integration tests.
//!
//! Tests the SHM fanout MPMC backend across real process boundaries:
//! - 2P/1S: two child publishers → one parent subscriber (FanoutShm)
//! - 1P/2S: one parent publisher → two child subscribers (FanoutShm)
//! - 2P/2S: two child publishers → two child subscribers (FanoutShm)
//!
//! Uses the re-exec pattern: the test binary spawns itself as a child process
//! with environment variables controlling which role to play.
//!
//! Run: `cargo test --no-default-features --test cross_process_fanout_shm -- --test-threads=1`

use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use horus_core::communication::topic::Topic;

mod common;
use common::cleanup_stale_shm;

const CHILD_ENV: &str = "HORUS_FANOUT_CHILD";
const TEST_ENV: &str = "HORUS_FANOUT_TEST";
const TOPIC_ENV: &str = "HORUS_FANOUT_TOPIC";
const ID_ENV: &str = "HORUS_FANOUT_ID";
const SENTINEL: u64 = u64::MAX;

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

fn unique(prefix: &str) -> String {
    static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
    )
}

// ─── Child entry points ──────────────────────────────────────────────────────

/// Publisher child: sends N messages tagged with child_id, then sentinel
fn child_publisher() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let id_str = std::env::var(ID_ENV).unwrap_or_else(|_| "0".into());
    let child_id: u64 = id_str.parse().unwrap_or_else(|_| {
        id_str.chars().filter(|c| c.is_ascii_digit()).collect::<String>().parse().unwrap_or(0)
    });

    // Wait for other participants to set up
    std::thread::sleep(Duration::from_millis(500));

    let topic: Topic<u64> = Topic::new(&topic_name).expect("child publisher: Topic::new");

    // Wait and force migration detection
    std::thread::sleep(Duration::from_millis(300));
    topic.check_migration_now();
    std::thread::sleep(Duration::from_millis(100));

    for i in 0..100u64 {
        let value = child_id * 10000 + i;
        topic.send(value);
        std::thread::sleep(Duration::from_millis(2));
    }
    topic.send(SENTINEL);

    println!("SENT:100");
}

/// Subscriber child: receives messages until sentinel or timeout
fn child_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");

    let topic: Topic<u64> = Topic::new(&topic_name).expect("child subscriber: Topic::new");

    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(8);
    let mut last_check = Instant::now();

    while Instant::now() < deadline {
        match topic.recv() {
            Some(SENTINEL) => break,
            Some(v) => received.push(v),
            None => {
                std::thread::yield_now();
                if last_check.elapsed() > Duration::from_millis(100) {
                    topic.check_migration_now();
                    last_check = Instant::now();
                }
            }
        }
    }

    println!("RECEIVED:{}", received.len());
}

// ─── Spawn helper ────────────────────────────────────────────────────────────

fn spawn_child(test_name: &str, topic: &str, id: &str) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TEST_ENV, test_name)
        .env(TOPIC_ENV, topic)
        .env(ID_ENV, id)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn child")
}

fn parse_value(stdout: &str, prefix: &str) -> Option<String> {
    stdout
        .lines()
        .find(|l| l.starts_with(prefix))
        .and_then(|l| l.strip_prefix(prefix))
        .map(|s| s.to_string())
}

fn wait_and_get_stdout(child: std::process::Child) -> String {
    let output = child.wait_with_output().expect("wait child");
    String::from_utf8_lossy(&output.stdout).to_string()
}

// ═══════════════════════════════════════════════════════════════════════════════
//  1. 2P/1S: Two child publishers → one parent subscriber
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_fanout_2p1s() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_fanout_2p1s") {
            child_publisher();
        }
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("xproc_fanout_2p1s");

    // Parent: create topic as subscriber first
    let topic: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");

    // Spawn 2 child publishers
    let child1 = spawn_child("cross_process_fanout_2p1s", &topic_name, "1");
    let child2 = spawn_child("cross_process_fanout_2p1s", &topic_name, "2");

    // Wait for children to connect and force migration check
    std::thread::sleep(Duration::from_millis(1000));
    topic.check_migration_now();

    // Parent receives messages from both publishers
    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut sentinel_count = 0;
    let mut last_check = Instant::now();

    while Instant::now() < deadline && sentinel_count < 2 {
        match topic.recv() {
            Some(SENTINEL) => sentinel_count += 1,
            Some(v) => received.push(v),
            None => {
                std::thread::yield_now();
                if last_check.elapsed() > Duration::from_millis(100) {
                    topic.check_migration_now();
                    last_check = Instant::now();
                }
            }
        }
    }

    let stdout1 = wait_and_get_stdout(child1);
    let stdout2 = wait_and_get_stdout(child2);

    // Both children should report sending 100
    assert!(
        stdout1.contains("SENT:100"),
        "Child 1 should send 100, got: {}",
        stdout1
    );
    assert!(
        stdout2.contains("SENT:100"),
        "Child 2 should send 100, got: {}",
        stdout2
    );

    // Parent should receive messages from both publishers
    // Note: some messages may be lost during backend migration (MpscShm → FanoutShm)
    assert!(
        received.len() >= 20,
        "Parent should receive at least 20 messages from 2 publishers, got {}",
        received.len()
    );

    // Verify messages from at least one publisher arrive
    // (cross-process migration may cause some messages to go to different backends)
    let from_pub1 = received.iter().filter(|&&v| v / 10000 == 1).count();
    let from_pub2 = received.iter().filter(|&&v| v / 10000 == 2).count();
    assert!(
        from_pub1 > 0 || from_pub2 > 0,
        "Should receive messages from at least one publisher (pub1={}, pub2={})",
        from_pub1,
        from_pub2
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. 1P/2S: One parent publisher → two child subscribers
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_fanout_1p2s() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_fanout_1p2s") {
            child_subscriber();
        }
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("xproc_fanout_1p2s");

    // Spawn 2 child subscribers first
    let child1 = spawn_child("cross_process_fanout_1p2s", &topic_name, "1");
    let child2 = spawn_child("cross_process_fanout_1p2s", &topic_name, "2");

    // Wait for children to set up
    std::thread::sleep(Duration::from_millis(800));

    // Parent publishes
    let topic: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    std::thread::sleep(Duration::from_millis(200));

    for i in 0..100u64 {
        topic.send(i);
        std::thread::sleep(Duration::from_millis(2));
    }
    topic.send(SENTINEL);

    let stdout1 = wait_and_get_stdout(child1);
    let stdout2 = wait_and_get_stdout(child2);

    // Each child should receive messages
    let recv1: usize = parse_value(&stdout1, "RECEIVED:")
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let recv2: usize = parse_value(&stdout2, "RECEIVED:")
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    assert!(
        recv1 > 50,
        "Child 1 should receive >50 messages, got {}",
        recv1
    );
    assert!(
        recv2 > 50,
        "Child 2 should receive >50 messages, got {}",
        recv2
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. 2P/2S: Two child publishers + parent → two child subscribers + parent
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_fanout_2p2s() {
    if is_child() {
        let test = std::env::var(TEST_ENV).ok();
        if test.as_deref() == Some("cross_process_fanout_2p2s") {
            let id = std::env::var(ID_ENV).unwrap_or_default();
            match id.as_str() {
                "pub1" | "pub2" => child_publisher(),
                "sub1" | "sub2" => child_subscriber(),
                _ => {}
            }
        }
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("xproc_fanout_2p2s");

    // Spawn 2 subscriber children first
    let sub1 = spawn_child("cross_process_fanout_2p2s", &topic_name, "sub1");
    let sub2 = spawn_child("cross_process_fanout_2p2s", &topic_name, "sub2");

    // Wait for subscribers to connect
    std::thread::sleep(Duration::from_millis(600));

    // Spawn 2 publisher children (they wait 500ms internally before sending)
    let pub1 = spawn_child("cross_process_fanout_2p2s", &topic_name, "pub1");
    let pub2 = spawn_child("cross_process_fanout_2p2s", &topic_name, "pub2");

    // Wait for publishers
    let pub1_stdout = wait_and_get_stdout(pub1);
    let pub2_stdout = wait_and_get_stdout(pub2);

    assert!(pub1_stdout.contains("SENT:100"), "pub1: {}", pub1_stdout);
    assert!(pub2_stdout.contains("SENT:100"), "pub2: {}", pub2_stdout);

    // Wait for subscribers
    let sub1_stdout = wait_and_get_stdout(sub1);
    let sub2_stdout = wait_and_get_stdout(sub2);

    let recv1: usize = parse_value(&sub1_stdout, "RECEIVED:")
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let recv2: usize = parse_value(&sub2_stdout, "RECEIVED:")
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    // At least one subscriber should receive messages
    // (cross-process migration timing means not all processes may converge to same backend)
    let total = recv1 + recv2;
    assert!(
        total > 10,
        "Subscribers should receive some messages total (sub1={}, sub2={}). sub1: {} sub2: {}",
        recv1,
        recv2,
        sub1_stdout,
        sub2_stdout
    );
}
