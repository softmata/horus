#![allow(clippy::field_reassign_with_default)]
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
use std::time::Instant;

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;

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
    let deadline = Instant::now() + 10_u64.secs();

    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(v) => received.push(v),
            None => std::thread::yield_now(),
        }
    }

    // Validate data integrity in child.
    // Filter out value 0 — the parent sends send(0) to trigger producer role
    // before sending the real messages 1..=expected_count.
    let received: Vec<u64> = received.into_iter().filter(|&v| v != 0).collect();
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
    std::thread::sleep(1000_u64.ms());

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

    std::thread::sleep(1000_u64.ms());
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

// ============================================================================
// Test 3: Reversed roles — parent=CONSUMER, child=PRODUCER
//
// This is the pattern used by horus-mujoco:
// - Parent (test) creates subscriber Topic<T>, calls recv()
// - Child (binary) creates publisher Topic<T>, calls send()
//
// The existing tests only cover parent=producer, child=consumer.
// ============================================================================

/// Child entry: open the topic as PRODUCER, send messages.
fn child_send_pod() {
    let topic_name = std::env::var(TOPIC_NAME_ENV).expect("HORUS_IPC_TOPIC not set");
    let msg_count: usize = std::env::var(MSG_COUNT_ENV)
        .expect("HORUS_IPC_COUNT not set")
        .parse()
        .expect("invalid count");

    let t: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new failed");

    // Send messages continuously for a few seconds so the parent has time
    // to detect cross-process migration and start receiving.
    let deadline = Instant::now() + 5_u64.secs();
    let mut seq = 1u64;
    while Instant::now() < deadline && seq <= msg_count as u64 {
        t.send(seq);
        seq += 1;
        // Pace at ~1kHz so we don't exhaust messages before parent migrates
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    t.send(SENTINEL);

    println!("SENT:{}", seq - 1);
}

#[test]
fn cross_process_reversed_roles_u64() {
    if is_child() {
        let test_name = std::env::var(TEST_NAME_ENV).unwrap_or_default();
        if test_name == "cross_process_reversed_roles_u64" {
            child_send_pod();
            return;
        }
        // Not our child — fall through to other tests' child handling
        return;
    }

    let topic_name = format!("xproc_rev_{}", std::process::id());
    let msg_count = 200usize;

    // Parent: create topic as CONSUMER first (reversed from test 1)
    let t: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");

    // Spawn child that will be the PRODUCER
    let child = spawn_child("cross_process_reversed_roles_u64", &topic_name, msg_count);

    // Wait for child to start sending, then force migration detection
    std::thread::sleep(500_u64.ms());
    t.check_migration_now();

    // Receive messages
    let mut received = Vec::new();
    let deadline = Instant::now() + 10_u64.secs();
    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(v) if v > 0 => received.push(v),
            _ => std::thread::yield_now(),
        }
    }

    let output = child.wait_with_output().expect("child wait failed");
    assert!(
        output.status.success(),
        "Child failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    eprintln!(
        "Reversed roles (parent=consumer, child=producer): received {}/{} messages",
        received.len(),
        msg_count
    );
    assert!(
        !received.is_empty(),
        "Parent received 0 messages in reversed-role test (parent=consumer, child=producer). \
         This pattern is used by horus-mujoco (binary=producer, test=consumer)."
    );
}

// ============================================================================
// Test 4: Reversed roles with a LARGE POD type (736 bytes, same as Odometry)
// ============================================================================

/// A 736-byte #[repr(C)] Copy type — same size as horus_library::sensor::Odometry.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct LargePod {
    data: [f64; 90], // 720 bytes
    tag: u64,        // 8 bytes
    seq: u64,        // 8 bytes
} // total: 736 bytes

// Manual serde impl to avoid [f64; 90] Deserialize bound issue
impl serde::Serialize for LargePod {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        // Serialize as raw bytes for POD type
        let bytes = unsafe {
            std::slice::from_raw_parts(
                self as *const Self as *const u8,
                std::mem::size_of::<Self>(),
            )
        };
        serde::Serialize::serialize(bytes, s)
    }
}

impl<'de> serde::Deserialize<'de> for LargePod {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let bytes: Vec<u8> = serde::Deserialize::deserialize(d)?;
        if bytes.len() != std::mem::size_of::<Self>() {
            return Err(serde::de::Error::custom("wrong size"));
        }
        Ok(unsafe { std::ptr::read(bytes.as_ptr() as *const Self) })
    }
}

impl Default for LargePod {
    fn default() -> Self {
        Self {
            data: [0.0; 90],
            tag: 0,
            seq: 0,
        }
    }
}

/// Child entry: send LargePod messages as producer.
fn child_send_large_pod() {
    let topic_name = std::env::var(TOPIC_NAME_ENV).expect("HORUS_IPC_TOPIC not set");
    let msg_count: usize = std::env::var(MSG_COUNT_ENV)
        .expect("HORUS_IPC_COUNT not set")
        .parse()
        .expect("invalid count");

    let t: Topic<LargePod> = Topic::new(&topic_name).expect("child: Topic::new failed");

    let deadline = Instant::now() + 5_u64.secs();
    for i in 1..=msg_count as u64 {
        if Instant::now() > deadline {
            break;
        }
        let mut msg = LargePod::default();
        msg.tag = 0xDEADBEEF;
        msg.seq = i;
        t.send(msg);
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    // Send sentinel
    let mut sentinel = LargePod::default();
    sentinel.tag = SENTINEL;
    t.send(sentinel);
    println!("SENT_LARGE:{}", msg_count);
}

#[test]
fn cross_process_reversed_roles_large_pod() {
    if is_child() {
        let test_name = std::env::var(TEST_NAME_ENV).unwrap_or_default();
        if test_name == "cross_process_reversed_roles_large_pod" {
            child_send_large_pod();
            return;
        }
        return;
    }

    let topic_name = format!("xproc_revlg_{}", std::process::id());
    let msg_count = 100usize;

    // Parent: consumer for LargePod (736 bytes — same size as Odometry)
    let t: Topic<LargePod> = Topic::new(&topic_name).expect("parent: Topic::new");

    let child = spawn_child(
        "cross_process_reversed_roles_large_pod",
        &topic_name,
        msg_count,
    );

    std::thread::sleep(500_u64.ms());
    t.check_migration_now();

    let mut received = Vec::new();
    let deadline = Instant::now() + 10_u64.secs();
    while Instant::now() < deadline {
        match t.recv() {
            Some(msg) if msg.tag == SENTINEL => break,
            Some(msg) if msg.tag == 0xDEADBEEF => received.push(msg.seq),
            _ => std::thread::yield_now(),
        }
    }

    let output = child.wait_with_output().expect("child wait failed");
    assert!(
        output.status.success(),
        "Child failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    eprintln!(
        "Reversed roles large POD (736B): received {}/{} messages (size={})",
        received.len(),
        msg_count,
        std::mem::size_of::<LargePod>()
    );
    assert!(
        !received.is_empty(),
        "Parent received 0 large POD messages (736B, same as Odometry)"
    );
}

// ============================================================================
// Test 5-8: ALL POD sizes used by horus — concurrent send/recv
//
// Each test: child sends at 1ms pacing, parent reads concurrently.
// This tests SUSTAINED delivery (not just initial burst).
// ============================================================================

/// Macro to generate a fixed-size POD struct with serde support.
macro_rules! sized_pod {
    ($name:ident, $n:expr) => {
        #[repr(C)]
        #[derive(Clone, Copy, Debug, PartialEq)]
        struct $name {
            seq: u64,
            _pad: [u8; $n - 8],
        }
        impl Default for $name {
            fn default() -> Self {
                Self {
                    seq: 0,
                    _pad: [0; $n - 8],
                }
            }
        }
        impl serde::Serialize for $name {
            fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
                let bytes =
                    unsafe { std::slice::from_raw_parts(self as *const Self as *const u8, $n) };
                serde::Serialize::serialize(bytes, s)
            }
        }
        impl<'de> serde::Deserialize<'de> for $name {
            fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
                let bytes: Vec<u8> = serde::Deserialize::deserialize(d)?;
                if bytes.len() != $n {
                    return Err(serde::de::Error::custom("wrong size"));
                }
                Ok(unsafe { std::ptr::read(bytes.as_ptr() as *const Self) })
            }
        }
    };
}

sized_pod!(Pod40, 40); // Clock size
sized_pod!(Pod304, 304); // Imu size
sized_pod!(Pod912, 912); // JointState size

// TFMessage (6152B) — too large for the macro's [u8; N-8], use nested arrays
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct Pod6152 {
    seq: u64,
    _pad1: [u64; 768], // 768 * 8 = 6144 bytes
}
impl Default for Pod6152 {
    fn default() -> Self {
        Self {
            seq: 0,
            _pad1: [0; 768],
        }
    }
}
impl serde::Serialize for Pod6152 {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let bytes = unsafe { std::slice::from_raw_parts(self as *const Self as *const u8, 6152) };
        serde::Serialize::serialize(bytes, s)
    }
}
impl<'de> serde::Deserialize<'de> for Pod6152 {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let bytes: Vec<u8> = serde::Deserialize::deserialize(d)?;
        if bytes.len() != 6152 {
            return Err(serde::de::Error::custom("wrong size"));
        }
        Ok(unsafe { std::ptr::read(bytes.as_ptr() as *const Self) })
    }
}

/// Generic child: send N POD messages at 5ms pace (slow enough for cross-process migration).
fn child_send_pod_generic<
    T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
>(
    topic_name: &str,
    msg_count: usize,
    make_msg: impl Fn(u64) -> T,
    sentinel: T,
) {
    let t: Topic<T> = Topic::new(topic_name).expect("child: Topic::new");
    for i in 1..=msg_count as u64 {
        t.send(make_msg(i));
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    t.send(sentinel);
}

/// Generic parent: recv and count, verify delivery rate.
/// Starts reading immediately (no sleep) — concurrent with producer.
fn parent_recv_pod_generic<
    T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
>(
    test_name: &str,
    topic_name: &str,
    msg_count: usize,
    min_pct: usize,
    get_seq: impl Fn(&T) -> u64,
    is_sentinel: impl Fn(&T) -> bool,
) {
    let t: Topic<T> = Topic::new(topic_name).expect("parent: Topic::new");
    let child = spawn_child(test_name, topic_name, msg_count);

    let mut received = Vec::new();
    let deadline = Instant::now() + 15_u64.secs();
    while Instant::now() < deadline {
        match t.recv() {
            Some(msg) if is_sentinel(&msg) => break,
            Some(msg) => {
                let seq = get_seq(&msg);
                if seq > 0 {
                    received.push(seq);
                }
            }
            None => std::thread::yield_now(),
        }
    }

    let output = child.wait_with_output().expect("child wait");
    assert!(output.status.success());

    let min_expected = msg_count * min_pct / 100;
    let size = std::mem::size_of::<T>();
    eprintln!(
        "Pod {size}B: {}/{msg_count} received ({}%)",
        received.len(),
        received.len() * 100 / msg_count.max(1)
    );
    assert!(
        received.len() >= min_expected,
        "Pod {size}B: {}<{min_expected} minimum",
        received.len()
    );

    for &seq in &received {
        assert!(seq >= 1 && seq <= msg_count as u64, "corrupted seq {seq}");
    }
}

// --- 40 bytes (Clock size, co-located) ---

#[test]
fn cross_process_pod_40b() {
    if is_child() {
        if std::env::var(TEST_NAME_ENV).unwrap_or_default() == "cross_process_pod_40b" {
            let tn = std::env::var(TOPIC_NAME_ENV).unwrap();
            child_send_pod_generic(
                &tn,
                200,
                |i| {
                    let mut m = Pod40::default();
                    m.seq = i;
                    m
                },
                {
                    let mut s = Pod40::default();
                    s.seq = SENTINEL;
                    s
                },
            );
            return;
        }
        return;
    }
    let tn = format!("xp40_{}", std::process::id());
    parent_recv_pod_generic::<Pod40>(
        "cross_process_pod_40b",
        &tn,
        200,
        80,
        |m| m.seq,
        |m| m.seq == SENTINEL,
    );
}

// --- 304 bytes (Imu size) ---

#[test]
fn cross_process_pod_304b() {
    if is_child() {
        if std::env::var(TEST_NAME_ENV).unwrap_or_default() == "cross_process_pod_304b" {
            let tn = std::env::var(TOPIC_NAME_ENV).unwrap();
            child_send_pod_generic(
                &tn,
                200,
                |i| {
                    let mut m = Pod304::default();
                    m.seq = i;
                    m
                },
                {
                    let mut s = Pod304::default();
                    s.seq = SENTINEL;
                    s
                },
            );
            return;
        }
        return;
    }
    let tn = format!("xp304_{}", std::process::id());
    parent_recv_pod_generic::<Pod304>(
        "cross_process_pod_304b",
        &tn,
        200,
        50,
        |m| m.seq,
        |m| m.seq == SENTINEL,
    );
}

// --- 912 bytes (JointState size) ---

#[test]
fn cross_process_pod_912b() {
    if is_child() {
        if std::env::var(TEST_NAME_ENV).unwrap_or_default() == "cross_process_pod_912b" {
            let tn = std::env::var(TOPIC_NAME_ENV).unwrap();
            child_send_pod_generic(
                &tn,
                200,
                |i| {
                    let mut m = Pod912::default();
                    m.seq = i;
                    m
                },
                {
                    let mut s = Pod912::default();
                    s.seq = SENTINEL;
                    s
                },
            );
            return;
        }
        return;
    }
    let tn = format!("xp912_{}", std::process::id());
    parent_recv_pod_generic::<Pod912>(
        "cross_process_pod_912b",
        &tn,
        200,
        50,
        |m| m.seq,
        |m| m.seq == SENTINEL,
    );
}

// --- 6152 bytes (TFMessage size) ---

#[test]
fn cross_process_pod_6152b() {
    if is_child() {
        if std::env::var(TEST_NAME_ENV).unwrap_or_default() == "cross_process_pod_6152b" {
            let tn = std::env::var(TOPIC_NAME_ENV).unwrap();
            child_send_pod_generic(
                &tn,
                100,
                |i| {
                    let mut m = Pod6152::default();
                    m.seq = i;
                    m
                },
                {
                    let mut s = Pod6152::default();
                    s.seq = SENTINEL;
                    s
                },
            );
            return;
        }
        return;
    }
    let tn = format!("xp6152_{}", std::process::id());
    parent_recv_pod_generic::<Pod6152>(
        "cross_process_pod_6152b",
        &tn,
        100,
        30,
        |m| m.seq,
        |m| m.seq == SENTINEL,
    );
}

// ============================================================================
// Test 9: Sustained 60s cross-process delivery at 100Hz
// ============================================================================

/// Child: send u64 at 100Hz for 60 seconds.
fn child_send_sustained() {
    let topic_name = std::env::var(TOPIC_NAME_ENV).expect("HORUS_IPC_TOPIC not set");
    let t: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new");

    let start = Instant::now();
    let mut seq = 1u64;
    while start.elapsed() < 60_u64.secs() {
        t.send(seq);
        seq += 1;
        std::thread::sleep(std::time::Duration::from_millis(10)); // 100Hz
    }
    t.send(SENTINEL);
    println!("SENT_SUSTAINED:{}", seq - 1);
}

#[test]
fn cross_process_sustained_60s() {
    if is_child() {
        if std::env::var(TEST_NAME_ENV).unwrap_or_default() == "cross_process_sustained_60s" {
            child_send_sustained();
            return;
        }
        return;
    }

    let topic_name = format!("xp_sus60_{}", std::process::id());
    let t: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    let child = spawn_child("cross_process_sustained_60s", &topic_name, 0);

    let mut received = 0u64;
    let mut last_seq = 0u64;
    let mut out_of_order = 0u64;
    let deadline = Instant::now() + 65_u64.secs();

    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(v) if v > 0 => {
                received += 1;
                if v <= last_seq {
                    out_of_order += 1;
                }
                last_seq = v;
            }
            _ => std::thread::yield_now(),
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let sent: u64 = stdout
        .lines()
        .find_map(|l| l.strip_prefix("SENT_SUSTAINED:"))
        .and_then(|n| n.parse().ok())
        .unwrap_or(6000);

    let pct = received * 100 / sent.max(1);
    eprintln!("Sustained 60s: {received}/{sent} received ({pct}%), out_of_order={out_of_order}");

    assert!(output.status.success(), "Child failed");
    assert!(
        received > sent * 95 / 100,
        "Sustained delivery <95%: {received}/{sent} ({pct}%)"
    );
    assert!(
        out_of_order < received / 100,
        "Too many out-of-order: {out_of_order}/{received}"
    );
}

// ============================================================================
// Test 10: Crash recovery — kill producer, restart, consumer reconnects
// ============================================================================

/// Child: send messages then wait to be killed (or exit after all sent).
fn child_send_then_die() {
    let topic_name = std::env::var(TOPIC_NAME_ENV).expect("HORUS_IPC_TOPIC not set");
    let msg_count: usize = std::env::var(MSG_COUNT_ENV)
        .unwrap_or("500".into())
        .parse()
        .unwrap_or(500);
    let t: Topic<u64> = Topic::new(&topic_name).expect("child: Topic::new");

    for i in 1..=msg_count as u64 {
        t.send(i);
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    // Don't send sentinel — simulate crash by just exiting
    println!("SENT_CRASH:{msg_count}");
}

#[test]
fn cross_process_crash_recovery() {
    if is_child() {
        let test_name = std::env::var(TEST_NAME_ENV).unwrap_or_default();
        if test_name == "cross_process_crash_recovery" {
            child_send_then_die();
            return;
        }
        return;
    }

    let topic_name = format!("xp_crash_{}", std::process::id());

    // Phase 1: Start first producer, let it send, then kill it
    let t: Topic<u64> = Topic::new(&topic_name).expect("parent: Topic::new");
    let mut child1 = spawn_child("cross_process_crash_recovery", &topic_name, 200);

    let mut phase1_received = 0u64;
    let deadline1 = Instant::now() + 5_u64.secs();
    while Instant::now() < deadline1 {
        match t.recv() {
            Some(v) if v > 0 && v != SENTINEL => {
                phase1_received += 1;
            }
            _ => std::thread::yield_now(),
        }
        // Kill after receiving some messages
        if phase1_received >= 50 {
            let _ = child1.kill();
            break;
        }
    }
    let _ = child1.wait();

    eprintln!("Phase 1: received {phase1_received} before kill");
    assert!(
        phase1_received >= 10,
        "Should receive some messages before kill"
    );

    // Phase 2: consumer keeps polling (should NOT crash on stale producer)
    let mut stale_reads = 0u64;
    for _ in 0..100 {
        if t.recv().is_some() {
            stale_reads += 1
        }
        std::thread::yield_now();
    }
    eprintln!("Phase 2: {stale_reads} stale reads after kill (expected: some or 0)");

    // Phase 3: Start NEW producer on same topic — consumer should get new data
    let mut child2 = spawn_child("cross_process_crash_recovery", &topic_name, 100);

    let mut phase3_received = 0u64;
    let deadline3 = Instant::now() + 10_u64.secs();
    while Instant::now() < deadline3 {
        match t.recv() {
            Some(v) if v > 0 && v != SENTINEL => {
                phase3_received += 1;
            }
            _ => std::thread::yield_now(),
        }
        if phase3_received >= 50 {
            break;
        }
    }
    let _ = child2.wait();

    eprintln!("Phase 3: received {phase3_received} from new producer");
    assert!(
        phase3_received > 0,
        "Consumer must receive messages from restarted producer (got 0)"
    );
}
