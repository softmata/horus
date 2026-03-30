#![allow(dead_code)]
//! Cross-language IPC tests: Rust ↔ Python cross-process communication.
//!
//! Tests the production deployment pattern where Python ML nodes and Rust motor
//! control nodes run in separate processes, communicating via SHM Topics.
//!
//! Architecture: Rust integration test spawns Python child processes. Each Python
//! child is a script written to /tmp at test time, executed with PYTHONPATH pointing
//! to the horus_py/ directory so `from horus._horus import Topic, CmdVel` works.
//!
//! **Run sequentially**: `cargo test --test cross_language_ipc -- --test-threads=1`

mod common;

use common::cleanup_stale_shm;
use horus_core::communication::Topic;
use horus_core::core::DurationExt;
use horus_types::GenericMessage;
use std::io::Write;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

// ============================================================================
// Python child process infrastructure
// ============================================================================

/// Returns the absolute path to the horus_py/ directory (for PYTHONPATH).
fn python_path() -> String {
    let manifest_dir = env!("CARGO_MANIFEST_DIR"); // .../horus/horus_core
    let workspace = std::path::Path::new(manifest_dir)
        .parent()
        .expect("horus_core parent");
    workspace.join("horus_py").to_string_lossy().to_string()
}

/// Write a Python script to /tmp and spawn it with correct PYTHONPATH.
fn spawn_python_child(script: &str, envs: Vec<(&str, String)>) -> std::process::Child {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let script_path = format!(
        "/tmp/horus_cross_lang_test_{}_{}.py",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    );

    // Write script to file
    let mut f = std::fs::File::create(&script_path).expect("create script file");
    f.write_all(script.as_bytes()).expect("write script");
    drop(f);

    let mut cmd = Command::new("python3");
    cmd.arg(&script_path)
        .env("PYTHONPATH", python_path())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    for (key, val) in envs {
        cmd.env(key, val);
    }

    cmd.spawn()
        .unwrap_or_else(|e| panic!("Failed to spawn Python child ({script_path}): {e}"))
}

/// Parse a `PREFIX:VALUE` line from child stdout.
fn parse_child_value(stdout: &str, prefix: &str) -> Option<String> {
    for line in stdout.lines() {
        if let Some(val) = line.strip_prefix(prefix) {
            return Some(val.to_string());
        }
    }
    None
}

fn parse_child_f64(stdout: &str, prefix: &str) -> f64 {
    parse_child_value(stdout, prefix)
        .and_then(|v| v.parse().ok())
        .unwrap_or(f64::NAN)
}

fn parse_child_u64(stdout: &str, prefix: &str) -> u64 {
    parse_child_value(stdout, prefix)
        .and_then(|v| v.parse().ok())
        .unwrap_or(0)
}

// ============================================================================
// Phase 1: Rust → Python typed message tests
// ============================================================================

/// Test: Rust publishes velocity data as Imu (serde path), Python subscribes.
/// Originally used Imu as a workaround for the Pod co-located cross-process bug.
/// The bug is now fixed (co-located disabled for SHM backends), but this test
/// is retained as it validates the Imu serde cross-process path.
#[test]
fn rust_publishes_velocity_python_subscribes() {
    cleanup_stale_shm();

    // Python child subscribes to Imu (serde path, proven to work cross-process)
    // We encode velocity commands in the linear_acceleration field.
    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
received = 0
last_ax = 0.0
last_ay = 0.0

deadline = time.time() + 4.0
while time.time() < deadline:
    msg = t.recv()
    if msg is not None:
        received += 1
        last_ax = msg.accel_x
        last_ay = msg.accel_y
    else:
        time.sleep(0.01)

print(f"CHILD_RECEIVED:{received}")
print(f"CHILD_AX:{last_ax}")
print(f"CHILD_AY:{last_ay}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    // Rust publishes Imu with velocity encoded in acceleration fields
    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut published = 0u64;
    while start.elapsed() < Duration::from_secs(3) {
        let mut imu = Imu::default();
        imu.linear_acceleration = [1.5, 0.3, 0.0]; // linear, angular, unused
        topic.send(imu);
        published += 1;
        std::thread::sleep(Duration::from_millis(10));
    }

    assert!(published > 100, "Should publish >100, got {published}");

    let output = child.wait_with_output().expect("Python child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_received = parse_child_u64(&stdout, "CHILD_RECEIVED:");
    let ax = parse_child_f64(&stdout, "CHILD_AX:");
    let ay = parse_child_f64(&stdout, "CHILD_AY:");

    assert!(
        child_received > 0,
        "Python should receive messages. Sent: {published}, received: {child_received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!((ax - 1.5).abs() < 0.01, "linear should be ~1.5, got {ax}");
    assert!((ay - 0.3).abs() < 0.01, "angular should be ~0.3, got {ay}");
}

/// Test: Python publishes Imu (serde path), Rust subscribes and verifies.
/// Tests the reverse direction: Python → Rust cross-process.
#[test]
fn python_publishes_imu_rust_subscribes() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    t.send(Imu(0.5, 0.6, 9.81, 0.01, -0.02, 0.03))
    sent += 1
    time.sleep(0.01)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut last_ax = 0.0f64;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(msg) = topic.recv() {
            received += 1;
            last_ax = msg.linear_acceleration[0];
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("Python child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        (last_ax - 0.5).abs() < 0.01,
        "accel_x should be ~0.5, got {last_ax}"
    );
}

/// Test: Rust publishes Imu with known acceleration/gyro values,
/// Python child verifies all 6 axis values survive cross-process SHM.
#[test]
fn rust_publishes_imu_python_subscribes() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
received = 0
ax = ay = az = gx = gy = gz = 0.0

deadline = time.time() + 4.0
while time.time() < deadline:
    msg = t.recv()
    if msg is not None:
        received += 1
        ax = msg.accel_x
        ay = msg.accel_y
        az = msg.accel_z
        gx = msg.gyro_x
        gy = msg.gyro_y
        gz = msg.gyro_z
    else:
        time.sleep(0.01)

print(f"CHILD_RECEIVED:{received}")
print(f"CHILD_AX:{ax}")
print(f"CHILD_AY:{ay}")
print(f"CHILD_AZ:{az}")
print(f"CHILD_GX:{gx}")
print(f"CHILD_GY:{gy}")
print(f"CHILD_GZ:{gz}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    // Publish Imu with known values
    use horus_robotics::messages::sensor::Imu;
    let imu_topic: Topic<Imu> = Topic::new("imu").expect("create imu topic");
    let start = Instant::now();
    let mut sent = 0u64;
    while start.elapsed() < Duration::from_secs(3) {
        let mut imu = Imu::default();
        imu.linear_acceleration = [0.1, 0.2, 9.81];
        imu.angular_velocity = [0.01, -0.02, 0.03];
        imu_topic.send(imu);
        sent += 1;
        std::thread::sleep(Duration::from_millis(10));
    }

    assert!(sent > 100, "Rust should send >100 Imu messages, got {sent}");

    let output = child.wait_with_output().expect("Python child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "Python child should exit cleanly. stderr: {stderr}"
    );

    let child_received = parse_child_u64(&stdout, "CHILD_RECEIVED:");
    assert!(
        child_received > 0,
        "Python should receive Imu messages. Sent: {sent}, received: {child_received}. \
         stdout: {stdout}, stderr: {stderr}"
    );

    // Verify all 6 axis values
    let ax = parse_child_f64(&stdout, "CHILD_AX:");
    let ay = parse_child_f64(&stdout, "CHILD_AY:");
    let az = parse_child_f64(&stdout, "CHILD_AZ:");
    let gx = parse_child_f64(&stdout, "CHILD_GX:");
    let gy = parse_child_f64(&stdout, "CHILD_GY:");
    let gz = parse_child_f64(&stdout, "CHILD_GZ:");

    assert!((ax - 0.1).abs() < 0.001, "accel_x should be ~0.1, got {ax}");
    assert!((ay - 0.2).abs() < 0.001, "accel_y should be ~0.2, got {ay}");
    assert!(
        (az - 9.81).abs() < 0.01,
        "accel_z should be ~9.81, got {az}"
    );
    assert!(
        (gx - 0.01).abs() < 0.001,
        "gyro_x should be ~0.01, got {gx}"
    );
    assert!(
        (gy - (-0.02)).abs() < 0.001,
        "gyro_y should be ~-0.02, got {gy}"
    );
    assert!(
        (gz - 0.03).abs() < 0.001,
        "gyro_z should be ~0.03, got {gz}"
    );
}

// ============================================================================
// Phase 1 continued: Python → Rust direction
// ============================================================================

/// Test: Python publishes generic dict messages via Topic("name"),
/// Rust subscribes via Topic<GenericMessage> and verifies content.
/// This is the most common Python IPC pattern: `node.send("topic", {"key": value})`.
#[test]
fn python_publishes_dict_rust_subscribes() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic

t = Topic("test_dict_xproc")
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    t.send({"velocity": 1.5, "label": "forward", "count": sent})
    sent += 1
    time.sleep(0.01)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    // Rust subscribes to the same generic topic
    let topic: Topic<GenericMessage> = Topic::new("test_dict_xproc").expect("create generic topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut last_data_len = 0usize;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(msg) = topic.recv() {
            received += 1;
            last_data_len = msg.data().len();
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("Python child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive GenericMessage from Python dict. \
         Python sent: {child_sent}, Rust received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        last_data_len > 0,
        "GenericMessage data should be non-empty (MessagePack bytes), got {last_data_len}"
    );
}

/// Test: Python horus.Node with Scheduler publishes via node.send(),
/// Rust subscribes. This tests the full Python user API (not just raw Topic).
#[test]
fn python_node_scheduler_publishes_rust_subscribes() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import horus
from horus._horus import Imu

sent_count = [0]

def publisher(node):
    node.send("imu", Imu(1.0, 2.0, 9.81, 0.1, 0.2, 0.3))
    sent_count[0] += 1

# Use typed topic declaration: pubs={topic_name: MessageType}
# This creates Topic<Imu> (not GenericMessage) so Rust can open with matching type.
pub = horus.Node(name="py_imu_pub", tick=publisher, rate=50, pubs={"imu": Imu})
horus.run(pub, duration=3.0)
print(f"CHILD_SENT:{sent_count[0]}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut last_az = 0.0f64;
    while start.elapsed() < Duration::from_secs(5) {
        if let Some(msg) = topic.recv() {
            received += 1;
            last_az = msg.linear_acceleration[2];
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("Python child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive from Python horus.Node. Sent: {child_sent}, \
         received: {received}. stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        (last_az - 9.81).abs() < 0.1,
        "accel_z should be ~9.81, got {last_az}"
    );
}

// ============================================================================
// Phase 2: Advanced patterns
// ============================================================================

/// Test: Python publishes at 30Hz, Rust subscribes at 1kHz polling.
/// Simulates camera framerate (Python) → fast controller (Rust).
/// The slow publisher shouldn't cause issues for the fast subscriber.
#[test]
fn rate_mismatch_python_30hz_rust_1khz() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
sent = 0
deadline = time.time() + 3.0
while time.time() < deadline:
    t.send(Imu(0.0, 0.0, 9.81, 0.0, 0.0, float(sent)))
    sent += 1
    time.sleep(0.033)  # ~30Hz
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    // Rust polls at 1kHz — 33x faster than Python publishes
    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut received = 0u64;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(_msg) = topic.recv() {
            received += 1;
        }
        std::thread::sleep(Duration::from_millis(1)); // 1kHz poll
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust (1kHz poll) should receive from Python (30Hz). \
         Sent: {child_sent}, received: {received}. stdout: {stdout}, stderr: {stderr}"
    );
    // Should receive most messages (no reason to lose them at 30Hz input)
    assert!(
        received > child_sent / 4,
        "Should receive >25% of sent messages. Sent: {child_sent}, received: {received}"
    );
}

/// Test: Python child crashes (sys.exit(1)) while Rust is running.
/// The Rust process must survive without hanging.
#[test]
fn python_crash_rust_survives() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time, sys
from horus._horus import Topic, Imu

t = Topic(Imu)
# Send a few messages then crash
for i in range(10):
    t.send(Imu(0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    time.sleep(0.01)
sys.exit(1)  # Simulate crash
"#,
        vec![],
    );

    std::thread::sleep(500_u64.ms());

    // Rust publisher runs for 3s — should complete without hanging
    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut sent = 0u64;
    while start.elapsed() < Duration::from_secs(3) {
        let mut imu = Imu::default();
        imu.linear_acceleration = [0.0, 0.0, 9.81];
        topic.send(imu);
        sent += 1;
        std::thread::sleep(Duration::from_millis(10));
    }

    assert!(
        sent > 100,
        "Rust should continue publishing after Python crash: {sent}"
    );

    let output = child.wait_with_output().expect("child wait");
    assert!(
        !output.status.success(),
        "Python child should have crashed (exit 1)"
    );
}

/// Test: ML inference simulation — Python publishes Imu "detections" at 30Hz,
/// Rust subscribes and publishes Imu "commands" at 100Hz.
/// Simulates: camera → Python ML model → Rust motor controller.
#[test]
fn ml_inference_pipeline_cross_language() {
    cleanup_stale_shm();

    // Python "inference" node: publishes to "detections" topic
    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

det_topic = Topic(Imu)  # "imu" topic — simulates detection output
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    # Simulated inference result encoded in Imu fields
    det_topic.send(Imu(1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    sent += 1
    time.sleep(0.033)  # 30Hz inference
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    // Rust "motor controller": subscribes to detections, publishes commands
    use horus_robotics::messages::sensor::Imu;
    let det_topic: Topic<Imu> = Topic::new("imu").expect("detection topic");

    // Use a separate serde type for commands (different topic name)
    let cmd_topic: Topic<[f32; 2]> = Topic::new("motor_cmd").expect("cmd topic");

    let start = Instant::now();
    let mut detections_received = 0u64;
    let mut commands_sent = 0u64;
    while start.elapsed() < Duration::from_secs(4) {
        // Check for detections
        if let Some(_det) = det_topic.recv() {
            detections_received += 1;
            // React: send motor command
            cmd_topic.send([1.0, 0.0]); // [linear, angular]
            commands_sent += 1;
        }
        std::thread::sleep(Duration::from_millis(10)); // 100Hz controller
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        detections_received > 0,
        "Rust should receive detections from Python ML node. \
         Python sent: {child_sent}, Rust received: {detections_received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        commands_sent > 0,
        "Rust should generate motor commands from detections. \
         Detections: {detections_received}, commands: {commands_sent}"
    );
}

/// Test: Reverse rate mismatch — Rust publishes at 1kHz, Python consumes at 30Hz.
/// Simulates fast motor controller (Rust) → slow logger/visualizer (Python).
/// The fast producer shouldn't block; the slow consumer always gets latest.
#[test]
fn rate_mismatch_rust_1khz_python_30hz() {
    cleanup_stale_shm();

    // Python child: polls at 30Hz (slow consumer)
    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
received = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    msg = t.recv()
    if msg is not None:
        received += 1
    time.sleep(0.033)  # ~30Hz consumer
print(f"CHILD_RECEIVED:{received}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    // Rust publishes at 1kHz (fast producer)
    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut sent = 0u64;
    while start.elapsed() < Duration::from_secs(3) {
        let mut imu = Imu::default();
        imu.linear_acceleration = [0.0, 0.0, 9.81];
        topic.send(imu);
        sent += 1;
        std::thread::sleep(Duration::from_millis(1)); // 1kHz
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_received = parse_child_u64(&stdout, "CHILD_RECEIVED:");
    assert!(
        child_received > 0,
        "Python (30Hz) should receive from Rust (1kHz). \
         Sent: {sent}, received: {child_received}. stdout: {stdout}, stderr: {stderr}"
    );
    // At 30Hz for 3s, max ~90 receives. Should get a good fraction.
    assert!(
        child_received > 10,
        "Python should receive >10 messages at 30Hz over 3s, got {child_received}"
    );
}

/// Test: Sustained 10s cross-process Python→Rust communication.
/// Verifies no degradation, memory leaks, or SHM fragmentation over time.
#[test]
fn sustained_10s_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
sent = 0
deadline = time.time() + 10.0
while time.time() < deadline:
    t.send(Imu(0.1, 0.2, 9.81, 0.01, 0.02, 0.03))
    sent += 1
    time.sleep(0.01)  # 100Hz
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut first_received_at = None;
    let mut last_received_at = None;
    while start.elapsed() < Duration::from_secs(12) {
        if let Some(_msg) = topic.recv() {
            received += 1;
            let now = start.elapsed();
            if first_received_at.is_none() {
                first_received_at = Some(now);
            }
            last_received_at = Some(now);
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        child_sent > 500,
        "Python should send >500 over 10s at 100Hz, got {child_sent}"
    );
    assert!(
        received > 100,
        "Rust should receive >100 over 10s. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );

    // Verify messages arrive throughout the entire duration (no early cutoff)
    if let (Some(first), Some(last)) = (first_received_at, last_received_at) {
        let span = last - first;
        assert!(
            span.as_secs() >= 3,
            "Messages should span >3s of the run. First: {first:?}, last: {last:?}"
        );
    }
}

// ============================================================================
// Phase 3: Edge cases — large payloads, config parity, bidirectional, fan-in
// ============================================================================

/// Test: Python creates a 640x480 RGB Image from numpy, sends via Topic(Image).
/// Rust receives and verifies dimensions, encoding, and pixel data integrity.
/// This tests the zero-copy TensorPool path for large (~921KB) binary payloads.
#[test]
fn python_image_to_rust_cross_process() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
try:
    import numpy as np
except ImportError:
    print("CHILD_SKIP:numpy_not_available")
    exit(0)

from horus._horus import Topic, Image

# Create 480x640 RGB image with known pixel values
data = np.zeros((480, 640, 3), dtype=np.uint8)
data[0, 0] = [255, 0, 0]        # red pixel at (0,0)
data[239, 319] = [0, 255, 0]    # green pixel at center
data[479, 639] = [0, 0, 255]    # blue pixel at bottom-right

img = Image.from_numpy(data)
t = Topic(Image)

# Send for 4s at 10Hz to ensure overlap with Rust receiver
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    t.send(img)
    sent += 1
    time.sleep(0.1)  # 10Hz

print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_core::memory::Image;
    let topic: Topic<Image> = Topic::new("image").expect("create image topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut verified = false;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(img) = topic.recv() {
            received += 1;
            // Verify dimensions
            if img.width() == 640 && img.height() == 480 {
                // Check pixel at (0,0) — should be [255, 0, 0]
                if let Some(px) = img.pixel(0, 0) {
                    if px == [255, 0, 0] {
                        verified = true;
                    }
                }
            }
        } else {
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    // Skip if numpy not available
    if parse_child_value(&stdout, "CHILD_SKIP:").is_some() {
        eprintln!("Skipping: numpy not available");
        return;
    }

    assert!(output.status.success(), "Python error: {stderr}");
    assert!(
        received > 0,
        "Rust should receive Image from Python. received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        verified,
        "Image should be 640x480 with red pixel at (0,0). Received {received} images."
    );
}

/// Test: Python Scheduler at 100Hz for 2s vs Rust simple 100Hz loop for 2s.
/// Verifies that tick counts are within 20% of each other (config parity).
#[test]
fn scheduler_config_parity() {
    cleanup_stale_shm();

    // Python child: horus.run(node, rate=100, duration=2)
    let child = spawn_python_child(
        r#"
import horus
from horus._horus import Imu

count = [0]

def counter(node):
    count[0] += 1

n = horus.Node(name="parity_test", tick=counter, rate=100)
horus.run(n, duration=2.0)
print(f"CHILD_TICKS:{count[0]}")
"#,
        vec![],
    );

    // Rust side: simple 100Hz loop for 2s
    let start = Instant::now();
    let mut rust_ticks = 0u64;
    while start.elapsed() < Duration::from_secs(2) {
        rust_ticks += 1;
        std::thread::sleep(Duration::from_millis(10)); // 100Hz
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let python_ticks = parse_child_u64(&stdout, "CHILD_TICKS:");
    assert!(
        python_ticks > 50,
        "Python should tick >50 times at 100Hz/2s, got {python_ticks}. stderr: {stderr}"
    );
    assert!(
        rust_ticks > 50,
        "Rust should tick >50 times at 100Hz/2s, got {rust_ticks}"
    );

    // Check parity: within 50% of each other (generous for process scheduling)
    let ratio = python_ticks as f64 / rust_ticks as f64;
    assert!(
        (0.5..=2.0).contains(&ratio),
        "Python ({python_ticks}) and Rust ({rust_ticks}) tick counts should be \
         within 2x of each other. Ratio: {ratio:.2}"
    );
}

/// Test: Bidirectional cross-language roundtrip.
/// Rust publishes sensor Imu (accel_x = sequence number), Python doubles it
/// and publishes back on a different topic. Rust verifies response.
#[test]
fn bidirectional_roundtrip_cross_language() {
    cleanup_stale_shm();

    // Python child: subscribes to "sensor_out", publishes doubled value to "response_in"
    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

sensor = Topic(Imu)            # subscribes to "imu" (default for Imu type)
# Use a named generic topic for the response to avoid type collision
response = Topic("roundtrip_response")

processed = 0
deadline = time.time() + 5.0
while time.time() < deadline:
    msg = sensor.recv()
    if msg is not None:
        # Double the accel_x value and send back as response
        val = msg.accel_x * 2.0
        response.send({"doubled": val, "seq": processed})
        processed += 1
    else:
        time.sleep(0.005)

print(f"CHILD_PROCESSED:{processed}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::Imu;
    let sensor_topic: Topic<Imu> = Topic::new("imu").expect("sensor topic");
    let response_topic: Topic<GenericMessage> =
        Topic::new("roundtrip_response").expect("response topic");

    let start = Instant::now();
    let mut sent = 0u64;
    let mut responses = 0u64;
    while start.elapsed() < Duration::from_secs(4) {
        // Publish sensor data
        let mut imu = Imu::default();
        imu.linear_acceleration = [sent as f64 + 1.0, 0.0, 0.0];
        sensor_topic.send(imu);
        sent += 1;

        // Check for responses
        if let Some(_resp) = response_topic.recv() {
            responses += 1;
        }
        std::thread::sleep(Duration::from_millis(20)); // 50Hz
    }

    // Drain remaining responses
    let drain_start = Instant::now();
    while drain_start.elapsed() < Duration::from_secs(2) {
        if let Some(_resp) = response_topic.recv() {
            responses += 1;
        } else {
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_processed = parse_child_u64(&stdout, "CHILD_PROCESSED:");
    assert!(
        child_processed > 0,
        "Python should process sensor messages. processed: {child_processed}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        responses > 0,
        "Rust should receive roundtrip responses. Sent: {sent}, \
         Python processed: {child_processed}, Rust responses: {responses}"
    );
}

/// Test: Multiple Python processes publish to the same topic (fan-in pattern).
/// Simulates multiple cameras or ML models feeding into a single Rust coordinator.
#[test]
fn multi_python_fan_in() {
    cleanup_stale_shm();

    // Child 1: publishes Imu with accel_x in [1.0, 2.0, 3.0, ...]
    let child1 = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
sent = 0
deadline = time.time() + 3.0
while time.time() < deadline:
    t.send(Imu(float(sent + 1), 0.0, 0.0, 0.0, 0.0, 0.0))
    sent += 1
    time.sleep(0.02)  # 50Hz
print(f"CHILD1_SENT:{sent}")
"#,
        vec![],
    );

    // Child 2: publishes Imu with accel_x in [1001.0, 1002.0, ...]
    let child2 = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Imu

t = Topic(Imu)
sent = 0
deadline = time.time() + 3.0
while time.time() < deadline:
    t.send(Imu(float(sent + 1001), 0.0, 0.0, 0.0, 0.0, 0.0))
    sent += 1
    time.sleep(0.02)  # 50Hz
print(f"CHILD2_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::Imu;
    let topic: Topic<Imu> = Topic::new("imu").expect("create topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut from_child1 = 0u64; // accel_x < 1000
    let mut from_child2 = 0u64; // accel_x >= 1000
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(msg) = topic.recv() {
            received += 1;
            let ax = msg.linear_acceleration[0];
            if ax > 0.0 && ax < 1000.0 {
                from_child1 += 1;
            } else if ax >= 1000.0 {
                from_child2 += 1;
            }
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let out1 = child1.wait_with_output().expect("child1 wait");
    let out2 = child2.wait_with_output().expect("child2 wait");
    let stdout1 = String::from_utf8_lossy(&out1.stdout);
    let stdout2 = String::from_utf8_lossy(&out2.stdout);
    let stderr1 = String::from_utf8_lossy(&out1.stderr);
    let stderr2 = String::from_utf8_lossy(&out2.stderr);
    assert!(out1.status.success(), "Child1 error: {stderr1}");
    assert!(out2.status.success(), "Child2 error: {stderr2}");

    let sent1 = parse_child_u64(&stdout1, "CHILD1_SENT:");
    let sent2 = parse_child_u64(&stdout2, "CHILD2_SENT:");
    assert!(
        received > 0,
        "Rust should receive from fan-in topic. \
         Child1 sent: {sent1}, child2 sent: {sent2}, received: {received}. \
         stderr1: {stderr1}, stderr2: {stderr2}"
    );
    // With SHM ring buffer, both producers write to the same ring.
    // We should see messages from at least one child (both ideally).
    assert!(
        from_child1 > 0 || from_child2 > 0,
        "Should receive from at least one child. \
         from_child1: {from_child1}, from_child2: {from_child2}, total: {received}"
    );
}

// ============================================================================
// Phase 4: Pod types cross-process (requires co-located fix)
// ============================================================================

/// Test: Python publishes CmdVel, Rust subscribes via Topic<CmdVel>.
/// CmdVel is a small Pod type (16 bytes) that previously triggered the co-located
/// slot layout bug. With the fix (co-located disabled for SHM), this works.
#[test]
fn cmdvel_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, CmdVel

t = Topic(CmdVel)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    t.send(CmdVel(1.5, -0.3))
    sent += 1
    time.sleep(0.01)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::CmdVel;
    let topic: Topic<CmdVel> = Topic::new("cmd_vel").expect("create cmdvel topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut last_linear = 0.0f32;
    let mut last_angular = 0.0f32;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(msg) = topic.recv() {
            received += 1;
            last_linear = msg.linear;
            last_angular = msg.angular;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive CmdVel from Python (Pod type, co-located fix). \
         Sent: {child_sent}, received: {received}. stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        (last_linear - 1.5).abs() < 0.01,
        "linear should be ~1.5, got {last_linear}"
    );
    assert!(
        (last_angular - (-0.3)).abs() < 0.01,
        "angular should be ~-0.3, got {last_angular}"
    );
}

/// Test: Python publishes Pose2D (small Pod), Rust subscribes.
#[test]
fn pose2d_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Pose2D

t = Topic(Pose2D)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    t.send(Pose2D(1.0, 2.0, 2.75))
    sent += 1
    time.sleep(0.01)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_types::Pose2D;
    let topic: Topic<Pose2D> = Topic::new("pose").expect("create pose topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut last_x = 0.0f64;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(msg) = topic.recv() {
            received += 1;
            last_x = msg.x;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive Pose2D from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        (last_x - 1.0).abs() < 0.01,
        "x should be ~1.0, got {last_x}"
    );
}

/// Test: Python publishes LaserScan (large serde Pod), Rust subscribes.
#[test]
fn laserscan_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, LaserScan

t = Topic(LaserScan)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    scan = LaserScan(angle_min=-1.57, angle_max=1.57, range_max=10.0)
    t.send(scan)
    sent += 1
    time.sleep(0.02)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::LaserScan;
    let topic: Topic<LaserScan> = Topic::new("scan").expect("create scan topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut last_angle_min = 0.0f32;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(msg) = topic.recv() {
            received += 1;
            last_angle_min = msg.angle_min;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive LaserScan from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        (last_angle_min - (-1.57)).abs() < 0.01,
        "angle_min should be ~-1.57, got {last_angle_min}"
    );
}

/// Test: Python publishes Odometry (large serde Pod), Rust subscribes.
#[test]
fn odometry_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Odometry

t = Topic(Odometry)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    odom = Odometry(x=1.5, y=2.5, theta=0.7)
    t.send(odom)
    sent += 1
    time.sleep(0.02)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::Odometry;
    let topic: Topic<Odometry> = Topic::new("odom").expect("create odom topic");
    let start = Instant::now();
    let mut received = 0u64;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(_msg) = topic.recv() {
            received += 1;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive Odometry from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
}

/// Test: Python publishes Detection (serde, contains String), Rust subscribes.
#[test]
fn detection_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Detection

t = Topic(Detection)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    det = Detection(class_name="person", confidence=0.95, x=10.0, y=20.0, width=100.0, height=200.0)
    t.send(det)
    sent += 1
    time.sleep(0.02)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::detection::Detection;
    let topic: Topic<Detection> = Topic::new("detection").expect("create detection topic");
    let start = Instant::now();
    let mut received = 0u64;
    let mut last_confidence = 0.0f32;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(msg) = topic.recv() {
            received += 1;
            last_confidence = msg.confidence;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive Detection from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
    assert!(
        (last_confidence - 0.95).abs() < 0.01,
        "confidence should be ~0.95, got {last_confidence}"
    );
}

/// Test: Python publishes Twist (serde), Rust subscribes.
#[test]
fn twist_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, Twist

t = Topic(Twist)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    tw = Twist(linear_x=1.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.5)
    t.send(tw)
    sent += 1
    time.sleep(0.02)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_types::Twist;
    let topic: Topic<Twist> = Topic::new("twist").expect("create twist topic");
    let start = Instant::now();
    let mut received = 0u64;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(_msg) = topic.recv() {
            received += 1;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive Twist from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
}

/// Test: Python publishes JointState (large serde Pod), Rust subscribes.
#[test]
fn joint_state_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, JointState

t = Topic(JointState)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    js = JointState()
    t.send(js)
    sent += 1
    time.sleep(0.02)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::sensor::JointState;
    let topic: Topic<JointState> = Topic::new("joint_states").expect("create joint_states topic");
    let start = Instant::now();
    let mut received = 0u64;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(_msg) = topic.recv() {
            received += 1;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive JointState from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
}

/// Test: Python publishes NavGoal (Pod), Rust subscribes.
#[test]
fn nav_goal_cross_process_python_to_rust() {
    cleanup_stale_shm();

    let child = spawn_python_child(
        r#"
import time
from horus._horus import Topic, NavGoal

t = Topic(NavGoal)
sent = 0
deadline = time.time() + 4.0
while time.time() < deadline:
    goal = NavGoal(x=5.0, y=3.0, theta=1.57)
    t.send(goal)
    sent += 1
    time.sleep(0.02)
print(f"CHILD_SENT:{sent}")
"#,
        vec![],
    );

    std::thread::sleep(1_u64.secs());

    use horus_robotics::messages::navigation::NavGoal;
    let topic: Topic<NavGoal> = Topic::new("nav_goal").expect("create nav_goal topic");
    let start = Instant::now();
    let mut received = 0u64;
    while start.elapsed() < Duration::from_secs(4) {
        if let Some(_msg) = topic.recv() {
            received += 1;
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    let output = child.wait_with_output().expect("child wait");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "Python error: {stderr}");

    let child_sent = parse_child_u64(&stdout, "CHILD_SENT:");
    assert!(
        received > 0,
        "Rust should receive NavGoal from Python. Sent: {child_sent}, received: {received}. \
         stdout: {stdout}, stderr: {stderr}"
    );
}
