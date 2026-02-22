//! Cross-process IPC integration tests.
//!
//! These tests verify that HORUS Topics correctly communicate across process
//! boundaries via shared memory. Each test spawns a child process that opens
//! the same SHM-backed Topic as the parent, exercising the real cross-process
//! SHM dispatch path (send_shm_* / recv_shm_* functions in dispatch.rs).
//!
//! The child process is the same test binary re-invoked with an env var flag.
//! Parent role = Producer only (no recv), Child role = Consumer only (no send).
//! This ensures send() goes through fn ptr dispatch (not DirectChannel fast path).

use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use horus_core::communication::topic::Topic;

/// Env var that marks a child process invocation.
const CHILD_ENV: &str = "HORUS_IPC_CHILD";
/// Env var carrying the topic name to the child.
const TOPIC_NAME_ENV: &str = "HORUS_IPC_TOPIC";
/// Env var carrying the message count to the child.
const MSG_COUNT_ENV: &str = "HORUS_IPC_COUNT";
/// Env var carrying which test to run as child.
const TEST_NAME_ENV: &str = "HORUS_IPC_TEST";
/// Sentinel value marking end of stream.
const SENTINEL: u64 = u64::MAX;

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

/// Child entry: open the topic as consumer, receive messages, print results.
fn child_recv_pod() {
    let topic_name = std::env::var(TOPIC_NAME_ENV).expect("HORUS_IPC_TOPIC not set");
    let expected_count: usize = std::env::var(MSG_COUNT_ENV)
        .expect("HORUS_IPC_COUNT not set")
        .parse()
        .expect("invalid count");

    let t: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new failed");

    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);

    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(v) => received.push(v),
            None => std::thread::yield_now(),
        }
    }

    // Validate data integrity in child
    for &v in &received {
        assert!(
            v >= 1 && v <= expected_count as u64,
            "child: corrupted value {} (expected 1..={})",
            v,
            expected_count
        );
    }

    // Print summary for parent to parse
    println!("RECEIVED:{}", received.len());
    for &v in &received {
        println!("V:{}", v);
    }
}

/// Spawn a child process that runs the specified child function.
fn spawn_child(test_name: &str, topic_name: &str, msg_count: usize) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TEST_NAME_ENV, test_name)
        .env(TOPIC_NAME_ENV, topic_name)
        .env(MSG_COUNT_ENV, msg_count.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn child process")
}

/// Parse child stdout for the received count and values.
fn parse_child_output(stdout: &str) -> (usize, Vec<u64>) {
    let mut count = 0;
    let mut values = Vec::new();
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("RECEIVED:") {
            count = n.parse().unwrap_or(0);
        } else if let Some(v) = line.strip_prefix("V:") {
            if let Ok(val) = v.parse::<u64>() {
                values.push(val);
            }
        }
    }
    (count, values)
}

// ============================================================================
// Test 1: Cross-process POD roundtrip (u64 → SpscShm co-located dispatch)
// ============================================================================

#[test]
fn cross_process_shm_pod_roundtrip() {
    if is_child() {
        child_recv_pod();
        return;
    }

    let topic_name = format!("xproc_pod_{}", std::process::id());
    let msg_count = 100usize;

    // Parent: create topic, register as producer only (no recv → role=Producer)
    let t: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    t.send(0); // trigger ensure_producer → SpscIntra (1p+0c, same process)

    // Spawn child (will register as consumer, detect cross-process → migrate to SpscShm)
    let child = spawn_child("cross_process_shm_pod_roundtrip", &topic_name, msg_count);

    // Wait for child to register and trigger cross-process migration to SpscShm.
    // The child's check_migration detects different PIDs → same_process=false → SpscShm.
    std::thread::sleep(Duration::from_millis(1000));

    // Force the parent to re-read the SHM header and detect the child's migration.
    // This installs SHM dispatch fn ptrs and syncs epoch counters.
    t.check_migration_now();

    // Send messages through SHM dispatch (role=Producer → fn ptr dispatch, not DC fast path)
    for i in 1..=msg_count as u64 {
        t.send(i);
    }
    t.send(SENTINEL);

    // Wait for child to finish
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() {
        panic!(
            "Child process failed (exit {:?}):\nstdout: {}\nstderr: {}",
            output.status.code(),
            stdout,
            stderr
        );
    }

    let (received_count, values) = parse_child_output(&stdout);

    assert!(
        received_count > 0,
        "Child received 0 messages.\nstdout: {}\nstderr: {}",
        stdout,
        stderr
    );

    // Verify data integrity
    for &v in &values {
        assert!(
            v >= 1 && v <= msg_count as u64,
            "Cross-process: corrupted value {}",
            v
        );
    }

    // Verify ordering (SPSC preserves order)
    for w in values.windows(2) {
        assert!(
            w[1] > w[0],
            "Cross-process: ordering broken {} → {}",
            w[0],
            w[1]
        );
    }

    eprintln!(
        "Cross-process POD roundtrip: child received {}/{} messages",
        received_count, msg_count
    );
}

// ============================================================================
// Test 2: Cross-process high-throughput stress (no crash, no corruption)
// ============================================================================

#[test]
fn cross_process_shm_stress_no_crash() {
    if is_child() {
        child_recv_pod();
        return;
    }

    let topic_name = format!("xproc_stress_{}", std::process::id());
    let msg_count = 10_000usize;

    let t: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    t.send(0);

    let child = spawn_child("cross_process_shm_stress_no_crash", &topic_name, msg_count);

    std::thread::sleep(Duration::from_millis(1000));
    t.check_migration_now();

    // High-throughput burst
    for i in 1..=msg_count as u64 {
        t.send(i);
    }
    t.send(SENTINEL);

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "Child crashed under stress.\nstdout: {}\nstderr: {}",
        stdout,
        stderr
    );

    let (received_count, _) = parse_child_output(&stdout);

    // Under high throughput, some messages may be lost (ring backpressure).
    // The key assertion is: no crash and no corruption.
    eprintln!(
        "Cross-process stress: child received {}/{} messages ({}%)",
        received_count,
        msg_count,
        received_count * 100 / msg_count
    );
}
