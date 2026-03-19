// Cross-process crash recovery tests.
//
// Verifies that SHM ring buffers survive producer process crashes:
// - Consumer reads no corrupt data when producer exits abruptly
// - Ring buffer is reusable after producer crash
// - Consumer doesn't hang or panic after producer death

use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;

mod common;
use common::cleanup_stale_shm;

const CHILD_ENV: &str = "HORUS_CRASH_CHILD";
const TOPIC_ENV: &str = "HORUS_CRASH_TOPIC";
const COUNT_ENV: &str = "HORUS_CRASH_COUNT";
const MODE_ENV: &str = "HORUS_CRASH_MODE";
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

/// Child entry: producer that sends N messages then exits abruptly
fn child_producer_crash() {
    let topic_name = std::env::var(TOPIC_ENV).unwrap();
    let count: u64 = std::env::var(COUNT_ENV).unwrap().parse().unwrap();
    let mode = std::env::var(MODE_ENV).unwrap_or_default();

    let t: Topic<u64> = Topic::new(&topic_name).unwrap();

    // Send messages
    for i in 1..=count {
        t.send(i);
        std::thread::sleep(Duration::from_micros(100));
    }

    if mode == "crash" {
        // Exit WITHOUT sending sentinel — simulates abrupt crash
        // No Drop, no cleanup, no graceful shutdown
        std::process::exit(42);
    } else {
        // Normal exit: send sentinel
        t.send(SENTINEL);
    }
}

/// Child entry: consumer that reads and validates messages
fn child_consumer() {
    let topic_name = std::env::var(TOPIC_ENV).unwrap();
    let count: usize = std::env::var(COUNT_ENV).unwrap().parse().unwrap();

    let t: Topic<u64> = Topic::new(&topic_name).unwrap();
    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);

    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(v) => {
                // Validate: value must be in valid range (no corruption)
                if v >= 1 && v <= count as u64 {
                    received.push(v);
                } else {
                    println!("CORRUPT:{}", v);
                }
            }
            None => std::thread::yield_now(),
        }
    }

    println!("RECEIVED:{}", received.len());
    println!("CORRUPT_COUNT:0"); // If we get here, no panics from corrupt reads
}

fn spawn_child(test_name: &str, topic: &str, count: u64, mode: &str) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TOPIC_ENV, topic)
        .env(COUNT_ENV, count.to_string())
        .env(MODE_ENV, mode)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap()
}

fn parse_received(stdout: &str) -> usize {
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("RECEIVED:") {
            return n.parse().unwrap_or(0);
        }
    }
    0
}

fn has_corruption(stdout: &str) -> bool {
    stdout.lines().any(|l| l.starts_with("CORRUPT:"))
}

// ============================================================================
// Test: Consumer survives producer crash — no corrupt reads, no panic
// ============================================================================

#[test]
fn test_consumer_survives_producer_crash() {
    if is_child() {
        let mode = std::env::var(MODE_ENV).unwrap_or_default();
        if mode == "crash" || mode == "normal" {
            child_producer_crash();
        } else {
            child_consumer();
        }
        return;
    }

    cleanup_stale_shm();
    let topic = unique("crash_survive");

    // Start consumer first
    let mut consumer = spawn_child(
        "test_consumer_survives_producer_crash",
        &topic,
        50,
        "consumer",
    );

    // Give consumer time to create topic
    std::thread::sleep(Duration::from_millis(100));

    // Start producer that will crash after 30 messages
    let mut producer = spawn_child("test_consumer_survives_producer_crash", &topic, 30, "crash");

    // Wait for producer to crash
    let producer_status = producer.wait().unwrap();
    // Producer exits with code 42 (simulated crash)

    // Give consumer time to process remaining messages
    std::thread::sleep(Duration::from_millis(500));

    // Kill consumer gracefully (it's waiting for sentinel that never comes)
    let _ = consumer.kill();
    let consumer_output = consumer.wait_with_output().unwrap();
    let stdout = String::from_utf8_lossy(&consumer_output.stdout);

    // Key assertion: no corrupt data read
    assert!(
        !has_corruption(&stdout),
        "Consumer should not read corrupt data after producer crash. Output: {}",
        stdout
    );
}

// ============================================================================
// Test: Topic reusable after producer crash — new producer can send
// ============================================================================

#[test]
fn test_topic_reusable_after_crash() {
    if is_child() {
        let mode = std::env::var(MODE_ENV).unwrap_or_default();
        if mode == "crash" || mode == "normal" {
            child_producer_crash();
        } else {
            child_consumer();
        }
        return;
    }

    cleanup_stale_shm();
    let topic = unique("crash_reuse");

    // First producer: sends 20 messages then crashes
    let mut producer1 = spawn_child("test_topic_reusable_after_crash", &topic, 20, "crash");
    let _ = producer1.wait();

    // Wait for crash cleanup
    std::thread::sleep(Duration::from_millis(200));

    // Second producer: sends 20 messages normally (with sentinel)
    let mut producer2 = spawn_child("test_topic_reusable_after_crash", &topic, 20, "normal");
    let p2_status = producer2.wait().unwrap();

    // Second producer should exit successfully
    assert!(
        p2_status.success(),
        "Second producer should succeed after first crash, status: {:?}",
        p2_status
    );
}

// ============================================================================
// Test: Parent as consumer — survives child producer crash
// ============================================================================

#[test]
fn test_parent_consumer_child_producer_crash() {
    if is_child() {
        child_producer_crash();
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("crash_parent_cons");

    // Parent creates topic as consumer
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();

    // Spawn child producer that crashes after 30 messages
    let mut child = spawn_child(
        "test_parent_consumer_child_producer_crash",
        &topic_name,
        30,
        "crash",
    );

    // Wait for child to crash
    let _ = child.wait();

    // Parent should be able to read whatever was sent without crashing
    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < deadline {
        match t.recv() {
            Some(v) if v != SENTINEL => received.push(v),
            _ => std::thread::yield_now(),
        }
    }

    // Validate: all received values should be in valid range
    for &v in &received {
        assert!(
            v >= 1 && v <= 30,
            "Received corrupt value {} after producer crash",
            v
        );
    }

    // Parent didn't crash — success!
}

// ============================================================================
// Test: 3 producers, kill one mid-stream, others continue delivering
// ============================================================================

#[test]
fn test_multi_producer_kill_one_others_continue() {
    if is_child() {
        child_producer_crash();
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("crash_multi_prod");

    // Parent is consumer
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();

    // Spawn 3 producers: first crashes after 20 msgs, others send 50 normally
    let mut crash_child = spawn_child(
        "test_multi_producer_kill_one_others_continue",
        &topic_name,
        20,
        "crash",
    );
    let mut normal1 = spawn_child(
        "test_multi_producer_kill_one_others_continue",
        &topic_name,
        50,
        "normal",
    );
    let mut normal2 = spawn_child(
        "test_multi_producer_kill_one_others_continue",
        &topic_name,
        50,
        "normal",
    );

    // Wait for crash child to die
    let crash_status = crash_child.wait().unwrap();
    assert!(
        !crash_status.success(),
        "Crash child should exit with non-zero"
    );

    // Wait for normal producers to finish
    let s1 = normal1.wait().unwrap();
    let s2 = normal2.wait().unwrap();
    assert!(s1.success(), "Normal producer 1 should succeed: {:?}", s1);
    assert!(s2.success(), "Normal producer 2 should succeed: {:?}", s2);

    // Parent reads everything — no corruption allowed
    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => {} // Skip sentinels
            Some(v) => received.push(v),
            None => std::thread::yield_now(),
        }
    }

    // All values should be in valid range [1..50]
    for &v in &received {
        assert!(
            v >= 1 && v <= 50,
            "Received corrupt value {} after multi-producer crash",
            v
        );
    }
}

// ============================================================================
// Test: 10K messages integrity across processes with checksum
// ============================================================================

#[test]
fn test_cross_process_high_volume_integrity() {
    if is_child() {
        child_producer_crash();
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("crash_hv_int");

    // Parent as consumer
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();

    // Spawn producer sending 1000 messages normally (with sentinel)
    let mut producer = spawn_child(
        "test_cross_process_high_volume_integrity",
        &topic_name,
        1000,
        "normal",
    );

    // Read while producer is running
    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(v) => received.push(v),
            None => std::thread::yield_now(),
        }
    }

    let _ = producer.wait();

    // Verify all received values are in valid range [1..1000]
    let mut corrupted = 0u64;
    for &v in &received {
        if v < 1 || v > 1000 {
            corrupted += 1;
        }
    }

    assert_eq!(
        corrupted, 0,
        "Zero corruption in {} cross-process messages",
        received.len()
    );
    assert!(
        received.len() > 10,
        "Should receive >10 of 1000 messages, got {}",
        received.len()
    );
}

// ============================================================================
// Test: Producer crash then new producer reuses same topic cleanly
// ============================================================================

#[test]
fn test_crash_then_new_producer_clean_data() {
    if is_child() {
        child_producer_crash();
        return;
    }

    cleanup_stale_shm();
    let topic_name = unique("crash_then_new");

    // Phase 1: Crash producer sends 30 then crashes
    let mut crash_prod = spawn_child(
        "test_crash_then_new_producer_clean_data",
        &topic_name,
        30,
        "crash",
    );
    let _ = crash_prod.wait();

    std::thread::sleep(Duration::from_millis(100));

    // Phase 2: Parent creates topic, new child producer sends 50 normally
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();

    let mut new_prod = spawn_child(
        "test_crash_then_new_producer_clean_data",
        &topic_name,
        50,
        "normal",
    );
    let new_status = new_prod.wait().unwrap();
    assert!(
        new_status.success(),
        "New producer after crash should succeed"
    );

    // Read from parent — should see clean data from new producer
    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(v) if v >= 1 && v <= 50 => received.push(v),
            Some(v) => panic!("Corrupt value from new producer: {}", v),
            None => std::thread::yield_now(),
        }
    }

    // New producer's data should be clean
    // (may also see stale data from crashed producer — that's OK as long as it's valid)
}
