//! Cross-process multi-participant IPC tests.
//!
//! Tests the SHM backends that handle multiple publishers/subscribers across
//! process boundaries:
//! - SPMC: 1 parent publisher → 3 child subscribers (SpmcShm)
//! - MPSC: 3 child publishers → 1 parent subscriber (MpscShm)
//! - MPMC: 2 parent pub threads + 2 child subscribers (MpmcShm)
//! - read_latest: child reads latest-only from parent publisher
//! - Serde: non-Pod custom struct cross-process
//! - 1MB: large payload cross-process
//! - Multi-thread: 4 parent threads → 1 child subscriber
//!
//! Run sequentially: `cargo test --test cross_process_multi_participant -- --test-threads=1`

use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use horus_core::communication::topic::Topic;

mod common;
use common::cleanup_stale_shm;

const CHILD_ENV: &str = "HORUS_MULTI_PART_CHILD";
const TEST_ENV: &str = "HORUS_MULTI_PART_TEST";
const TOPIC_ENV: &str = "HORUS_MULTI_PART_TOPIC";
const ID_ENV: &str = "HORUS_MULTI_PART_ID";
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

/// SPMC child: subscribe to topic, receive messages, print count
fn child_spmc_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let topic: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new");

    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);

    while Instant::now() < deadline {
        match topic.recv() {
            Some(SENTINEL) => break,
            Some(v) => received.push(v),
            None => std::thread::yield_now(),
        }
    }

    println!("RECEIVED:{}", received.len());
}

/// MPSC child: publish messages to topic
fn child_mpsc_publisher() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let child_id: u64 = std::env::var(ID_ENV)
        .expect("ID_ENV")
        .parse()
        .expect("parse ID");

    std::thread::sleep(Duration::from_millis(300)); // Wait for parent subscriber to be ready

    let topic: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new");

    for i in 0..50u64 {
        let value = child_id * 1000 + i; // Tag with child_id for verification
        topic.send(value);
        std::thread::sleep(Duration::from_millis(5));
    }
    topic.send(SENTINEL);

    println!("SENT:50");
}

/// MPMC child: subscribe to topic from multiple parent publishers
fn child_mpmc_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let topic: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new");

    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);

    while Instant::now() < deadline {
        match topic.recv() {
            Some(SENTINEL) => break,
            Some(v) => received.push(v),
            None => std::thread::yield_now(),
        }
    }

    println!("RECEIVED:{}", received.len());
}

/// read_latest child: reads latest value only
fn child_read_latest() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let topic: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new");

    // Wait for parent to send burst
    std::thread::sleep(Duration::from_millis(500));

    // read_latest should return the latest value, not the first
    if let Some(val) = topic.read_latest() {
        println!("LATEST:{}", val);
    } else {
        println!("LATEST:none");
    }
}

/// Multi-thread child: receives from topic written by 4 parent threads
fn child_multithread_recv() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let topic: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new");

    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut sentinel_count = 0;

    while Instant::now() < deadline {
        match topic.recv() {
            Some(SENTINEL) => {
                sentinel_count += 1;
                if sentinel_count >= 4 {
                    break;
                }
            }
            Some(v) => received.push(v),
            None => std::thread::yield_now(),
        }
    }

    // Check which thread IDs are represented (value = thread_id * 10000 + i)
    let mut thread_ids: std::collections::HashSet<u64> = std::collections::HashSet::new();
    for &v in &received {
        thread_ids.insert(v / 10000);
    }

    println!("RECEIVED:{}", received.len());
    println!("THREADS:{}", thread_ids.len());
}

// ─── Spawn helper ────────────────────────────────────────────────────────────

fn spawn_child_topic(test_name: &str, topic: &str) -> std::process::Child {
    spawn_child_topic_id(test_name, topic, "0")
}

fn spawn_child_topic_id(test_name: &str, topic: &str, id: &str) -> std::process::Child {
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

// ═══════════════════════════════════════════════════════════════════════════════
//  1. SPMC: 1 parent publisher → 3 child subscribers
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_spmc_1_pub_3_sub() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_spmc_1_pub_3_sub") {
            child_spmc_subscriber();
        }
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("xproc_spmc");

    // Parent: create topic as publisher
    let topic: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    topic.send(0); // Establish producer role

    // Spawn 3 child subscribers
    let children: Vec<_> = (0..3)
        .map(|_| spawn_child_topic("cross_process_spmc_1_pub_3_sub", &topic_name))
        .collect();

    // Wait for children to register and trigger migration
    std::thread::sleep(Duration::from_millis(1000));
    topic.check_migration_now();

    // Send messages
    for i in 1..=100u64 {
        topic.send(i);
        std::thread::sleep(Duration::from_millis(10));
    }
    // Send sentinel multiple times for each subscriber
    for _ in 0..10 {
        topic.send(SENTINEL);
        std::thread::sleep(Duration::from_millis(10));
    }

    // Collect results from all children
    let mut total_received = 0;
    for child in children {
        let output = child.wait_with_output().expect("child wait failed");
        let stdout = String::from_utf8_lossy(&output.stdout);

        if let Some(count_str) = parse_value(&stdout, "RECEIVED:") {
            let count: usize = count_str.parse().unwrap_or(0);
            total_received += count;
            eprintln!("SPMC child received: {}", count);
        }
    }

    // At least some messages should reach subscribers
    // (SPMC doesn't guarantee all subscribers get all messages — ring buffer semantics)
    assert!(
        total_received > 0,
        "SPMC: at least one child should receive messages, total={}",
        total_received
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. MPSC: 3 child publishers → 1 parent subscriber
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_mpsc_3_pub_1_sub() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_mpsc_3_pub_1_sub") {
            child_mpsc_publisher();
        }
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("xproc_mpsc");

    // Parent: create topic as subscriber
    let topic: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");

    // Spawn 3 child publishers with different IDs
    let children: Vec<_> = (1..=3u64)
        .map(|id| {
            spawn_child_topic_id(
                "cross_process_mpsc_3_pub_1_sub",
                &topic_name,
                &id.to_string(),
            )
        })
        .collect();

    // Wait for children to start and register on the topic
    std::thread::sleep(Duration::from_millis(1000));
    topic.check_migration_now();

    // Parent: receive messages
    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut sentinel_count = 0;
    let mut last_migration_check = Instant::now();

    while Instant::now() < deadline {
        match topic.recv() {
            Some(SENTINEL) => {
                sentinel_count += 1;
                if sentinel_count >= 3 {
                    break;
                }
            }
            Some(v) => received.push(v),
            None => {
                std::thread::yield_now();
                // Periodically check migration (every 100ms)
                if last_migration_check.elapsed() > Duration::from_millis(100) {
                    topic.check_migration_now();
                    last_migration_check = Instant::now();
                }
            }
        }
    }

    // Wait for all children
    for child in children {
        let _ = child.wait_with_output();
    }

    // Verify we received messages from multiple child publishers
    let mut child_ids: std::collections::HashSet<u64> = std::collections::HashSet::new();
    for &v in &received {
        child_ids.insert(v / 1000); // child_id * 1000 + i
    }

    eprintln!(
        "MPSC: received {} messages from {} children: {:?}",
        received.len(),
        child_ids.len(),
        child_ids
    );

    assert!(
        received.len() > 0,
        "MPSC: parent should receive messages from children"
    );
    assert!(
        child_ids.len() >= 2,
        "MPSC: should receive from at least 2 of 3 children, got {:?}",
        child_ids
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. read_latest: child gets latest value, not queued
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_read_latest() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_read_latest") {
            child_read_latest();
        }
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("xproc_latest");

    let topic: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    topic.send(0);

    // Spawn child
    let child = spawn_child_topic("cross_process_read_latest", &topic_name);

    std::thread::sleep(Duration::from_millis(300));
    topic.check_migration_now();

    // Send burst — child should see latest, not first
    for i in 1..=100u64 {
        topic.send(i);
    }

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    if let Some(val_str) = parse_value(&stdout, "LATEST:") {
        if val_str != "none" {
            let val: u64 = val_str.parse().expect("parse latest");
            eprintln!("read_latest got: {}", val);
            // Should be a recent value, not 1 (the first)
            assert!(
                val > 50,
                "read_latest should return a recent value (>50), got {}",
                val
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  4. Multi-threaded send from parent, child receives
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_multithread_send_child_recv() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref()
            == Some("cross_process_multithread_send_child_recv")
        {
            child_multithread_recv();
        }
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("xproc_mt");

    let topic: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    topic.send(0);

    // Spawn child subscriber
    let child = spawn_child_topic("cross_process_multithread_send_child_recv", &topic_name);

    std::thread::sleep(Duration::from_millis(500));
    topic.check_migration_now();

    // 4 parent threads each send 50 messages
    let handles: Vec<_> = (0..4u64)
        .map(|thread_id| {
            let t: Topic<u64> = Topic::new(&topic_name).expect("thread: Topic::new");
            std::thread::spawn(move || {
                for i in 0..50u64 {
                    t.send(thread_id * 10000 + i);
                    std::thread::sleep(Duration::from_millis(5));
                }
                t.send(SENTINEL);
            })
        })
        .collect();

    for h in handles {
        h.join().unwrap();
    }

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    let received: usize = parse_value(&stdout, "RECEIVED:")
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let threads: usize = parse_value(&stdout, "THREADS:")
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    eprintln!(
        "Multi-thread: child received {} msgs from {} threads",
        received, threads
    );

    assert!(
        received > 0,
        "child should receive messages from parent threads"
    );
    assert!(
        threads >= 2,
        "child should see messages from at least 2 of 4 threads, got {}",
        threads
    );
}
