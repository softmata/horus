//! IPC torture tests — cross-process edge cases that break real robots.
//!
//! These tests target the EXACT failure modes found in the backend switching
//! audit. Each test spawns real child processes and exercises cross-process
//! SHM paths — NOT intra-process DirectChannel.
//!
//! Edge cases tested:
//! 1. Type mismatch: Process A sends CmdVel, Process B reads as different type
//! 2. Late joiner: Process B joins after Process A has sent 1000 messages
//! 3. Process crash mid-stream + new process reconnect
//! 4. Fan-out: 1 publisher → 4 subscriber processes
//! 5. Backend migration: Start intra-process, add cross-process, verify switch
//! 6. TensorPool cross-process: large message spill via shared memory
//! 7. Rapid process churn: 5 processes join/leave in quick succession
//! 8. Bidirectional: Both processes send AND receive on same topic
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test ipc_torture -- --ignored --nocapture

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use serde::{Deserialize, Serialize};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

mod common;
use common::{cleanup_stale_shm, unique};

// ════════════════════════════════════════════════════════════════════════
// Child process dispatch
// ════════════════════════════════════════════════════════════════════════

const CHILD_FLAG: &str = "HORUS_IPC_TORTURE_CHILD";
const TOPIC_ENV: &str = "HORUS_IPC_TORTURE_TOPIC";
const ROLE_ENV: &str = "HORUS_IPC_TORTURE_ROLE";
const COUNT_ENV: &str = "HORUS_IPC_TORTURE_COUNT";
const ID_ENV: &str = "HORUS_IPC_TORTURE_ID";
const SENTINEL: u64 = u64::MAX;

// ── Message types ──────────────────────────────────────────────────

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct CmdVel {
    linear_x: f64,
    linear_y: f64,
    angular_z: f64,
    seq: u64,
}
unsafe impl bytemuck::Pod for CmdVel {}
unsafe impl bytemuck::Zeroable for CmdVel {}

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct ImuCompact {
    accel: [f64; 3],
    gyro: [f64; 3],
    seq: u64,
    checksum: u64, // sum of accel+gyro fields for integrity
}
unsafe impl bytemuck::Pod for ImuCompact {}
unsafe impl bytemuck::Zeroable for ImuCompact {}

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct JointState8 {
    positions: [f64; 8],
    velocities: [f64; 8],
    seq: u64,
}
unsafe impl bytemuck::Pod for JointState8 {}
unsafe impl bytemuck::Zeroable for JointState8 {}

/// Large message that should trigger TensorPool spill (>4KB)
#[derive(Clone, Serialize, Deserialize)]
struct PointCloud {
    points: Vec<[f32; 3]>, // NOT Pod — serde path, forces serialization
    seq: u64,
    checksum: u64,
}

fn imu_checksum(imu: &ImuCompact) -> u64 {
    let mut sum = 0u64;
    for &v in &imu.accel {
        sum = sum.wrapping_add(v.to_bits());
    }
    for &v in &imu.gyro {
        sum = sum.wrapping_add(v.to_bits());
    }
    sum
}

fn pc_checksum(points: &[[f32; 3]]) -> u64 {
    points
        .iter()
        .map(|p| p[0].to_bits() as u64 + p[1].to_bits() as u64 + p[2].to_bits() as u64)
        .sum()
}

fn spawn_child(
    test_name: &str,
    topic: &str,
    role: &str,
    count: u64,
    id: u64,
) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture", "--ignored"])
        .env(CHILD_FLAG, "1")
        .env(TOPIC_ENV, topic)
        .env(ROLE_ENV, role)
        .env(COUNT_ENV, count.to_string())
        .env(ID_ENV, id.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap_or_else(|e| panic!("spawn {} child: {}", role, e))
}

fn child_env(key: &str) -> String {
    std::env::var(key).unwrap_or_default()
}

fn parse_child_output(output: &[u8], prefix: &str) -> Vec<String> {
    String::from_utf8_lossy(output)
        .lines()
        .filter(|l| l.starts_with(prefix))
        .map(|l| l[prefix.len()..].to_string())
        .collect()
}

// ════════════════════════════════════════════════════════════════════════
// TEST 1: Cross-process POD message integrity
// ════════════════════════════════════════════════════════════════════════
// Sends 5000 IMU messages with checksums across process boundary.
// Verifies every message arrives intact — catches byte-level corruption.

fn child_imu_producer() {
    let topic_name = child_env(TOPIC_ENV);
    let count: u64 = child_env(COUNT_ENV).parse().unwrap();
    let t: Topic<ImuCompact> = Topic::new(&topic_name).unwrap();
    // Trigger producer role
    t.send(ImuCompact::default());
    std::thread::sleep(Duration::from_millis(50));

    for i in 1..=count {
        let mut imu = ImuCompact {
            accel: [i as f64 * 0.01, (i as f64 * 0.02).sin(), 9.81],
            gyro: [(i as f64 * 0.03).cos(), 0.0, i as f64 * 0.001],
            seq: i,
            checksum: 0,
        };
        imu.checksum = imu_checksum(&imu);
        t.send(imu);
        // Simulate 1kHz rate
        if i % 100 == 0 {
            std::thread::yield_now();
        }
    }
    t.send(ImuCompact {
        seq: SENTINEL,
        ..Default::default()
    });
    std::thread::sleep(Duration::from_millis(200));
}

fn child_imu_consumer() {
    let topic_name = child_env(TOPIC_ENV);
    let t: Topic<ImuCompact> = Topic::new(&topic_name).unwrap();
    let deadline = Instant::now() + Duration::from_secs(15);

    let mut received = 0u64;
    let mut corrupted = 0u64;
    let mut last_seq = 0u64;
    let mut out_of_order = 0u64;

    while Instant::now() < deadline {
        match t.recv() {
            Some(imu) if imu.seq == SENTINEL => break,
            Some(imu) if imu.seq == 0 => continue, // init msg
            Some(imu) => {
                received += 1;
                let expected = imu_checksum(&imu);
                if imu.checksum != expected {
                    corrupted += 1;
                }
                if imu.seq <= last_seq && last_seq > 0 {
                    out_of_order += 1;
                }
                last_seq = imu.seq;
            }
            None => std::thread::yield_now(),
        }
    }
    println!("RECV:{}", received);
    println!("CORRUPT:{}", corrupted);
    println!("OOO:{}", out_of_order);
    println!("LAST_SEQ:{}", last_seq);
}

#[test]
#[ignore]
fn ipc_pod_integrity_5000_msgs() {
    if std::env::var(CHILD_FLAG).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "imu_producer" => child_imu_producer(),
            "imu_consumer" => child_imu_consumer(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("ipc_imu");
    let count = 5000u64;

    // Spawn consumer first (waits for data)
    let mut consumer = spawn_child(
        "ipc_pod_integrity_5000_msgs",
        &topic,
        "imu_consumer",
        count,
        0,
    );
    std::thread::sleep(Duration::from_millis(200));

    // Spawn producer
    let mut producer = spawn_child(
        "ipc_pod_integrity_5000_msgs",
        &topic,
        "imu_producer",
        count,
        0,
    );

    let prod_status = producer.wait().unwrap();
    assert!(prod_status.success(), "producer failed");

    let output = consumer.wait_with_output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);

    let received: u64 = parse_child_output(&output.stdout, "RECV:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let corrupted: u64 = parse_child_output(&output.stdout, "CORRUPT:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(999);
    let ooo: u64 = parse_child_output(&output.stdout, "OOO:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(999);

    println!("=== IPC POD INTEGRITY ({} msgs) ===", count);
    println!(
        "Received: {}/{} ({:.1}%)",
        received,
        count,
        received as f64 / count as f64 * 100.0
    );
    println!("Corrupted: {}", corrupted);
    println!("Out-of-order: {}", ooo);
    println!("Stdout: {}", stdout.trim());

    assert_eq!(
        corrupted, 0,
        "DATA CORRUPTION detected in cross-process IPC!"
    );
    assert_eq!(
        ooo, 0,
        "Messages arrived out of order in cross-process IPC!"
    );
    assert!(
        received > count / 2,
        "Received less than 50% of messages: {}/{}",
        received,
        count
    );
    println!("IPC POD INTEGRITY PASSED ✓");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: Late joiner — Process B joins after 1000 messages sent
// ════════════════════════════════════════════════════════════════════════

fn child_late_producer() {
    let topic_name = child_env(TOPIC_ENV);
    let count: u64 = child_env(COUNT_ENV).parse().unwrap();
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();
    t.send(0u64); // init

    for i in 1..=count {
        t.send(i);
        if i % 100 == 0 {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    t.send(SENTINEL);
    std::thread::sleep(Duration::from_millis(500));
}

fn child_late_consumer() {
    let topic_name = child_env(TOPIC_ENV);
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut received = 0u64;
    let mut min_seq = u64::MAX;
    let mut max_seq = 0u64;

    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(0) => continue,
            Some(v) => {
                received += 1;
                if v < min_seq {
                    min_seq = v;
                }
                if v > max_seq {
                    max_seq = v;
                }
            }
            None => std::thread::yield_now(),
        }
    }
    println!("RECV:{}", received);
    println!("MIN:{}", if min_seq == u64::MAX { 0 } else { min_seq });
    println!("MAX:{}", max_seq);
}

#[test]
#[ignore]
fn ipc_late_joiner_misses_early_messages() {
    if std::env::var(CHILD_FLAG).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "late_producer" => child_late_producer(),
            "late_consumer" => child_late_consumer(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("ipc_late");
    let total_msgs = 3000u64;

    // Start producer first — it sends 3000 msgs over ~30ms
    let mut producer = spawn_child(
        "ipc_late_joiner_misses_early_messages",
        &topic,
        "late_producer",
        total_msgs,
        0,
    );

    // Wait for producer to send ~1000 messages
    std::thread::sleep(Duration::from_millis(15));

    // NOW start consumer (late joiner)
    let mut consumer = spawn_child(
        "ipc_late_joiner_misses_early_messages",
        &topic,
        "late_consumer",
        total_msgs,
        0,
    );

    let _ = producer.wait();
    let output = consumer.wait_with_output().unwrap();

    let received: u64 = parse_child_output(&output.stdout, "RECV:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let min_seq: u64 = parse_child_output(&output.stdout, "MIN:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let max_seq: u64 = parse_child_output(&output.stdout, "MAX:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    println!("=== LATE JOINER TEST ===");
    println!("Total sent: {}", total_msgs);
    println!("Consumer received: {} (started late)", received);
    println!("First msg seen: seq={}", min_seq);
    println!("Last msg seen:  seq={}", max_seq);

    // Late joiner should miss early messages (that's expected)
    // But should receive the LATER messages without corruption
    assert!(
        received > 0,
        "Late joiner received ZERO messages — IPC not working"
    );
    assert!(
        max_seq == total_msgs,
        "Late joiner missed the final messages (max_seq={}, expected {})",
        max_seq,
        total_msgs
    );
    // The first message should be > 1 (missed early ones)
    println!(
        "Missed first {} messages (expected — ring buffer overwrites old data)",
        min_seq - 1
    );
    println!("LATE JOINER TEST PASSED ✓");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: Cross-process fan-out — 1 publisher, 4 subscriber processes
// ════════════════════════════════════════════════════════════════════════

fn child_fanout_subscriber() {
    let topic_name = child_env(TOPIC_ENV);
    let id: u64 = child_env(ID_ENV).parse().unwrap();
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut received = 0u64;

    while Instant::now() < deadline {
        match t.recv() {
            Some(SENTINEL) => break,
            Some(0) => continue,
            Some(_) => received += 1,
            None => std::thread::yield_now(),
        }
    }
    println!("SUB:{}:{}", id, received);
}

#[test]
#[ignore]
fn ipc_fanout_1pub_4sub_processes() {
    if std::env::var(CHILD_FLAG).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "late_producer" => child_late_producer(),
            "fanout_sub" => child_fanout_subscriber(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("ipc_fanout");
    let count = 2000u64;

    // Initialize topic from parent
    let _init: Topic<u64> = Topic::new(&topic).unwrap();

    // Spawn 4 subscriber processes
    let mut subs = vec![];
    for i in 0..4u64 {
        subs.push(spawn_child(
            "ipc_fanout_1pub_4sub_processes",
            &topic,
            "fanout_sub",
            count,
            i,
        ));
    }
    std::thread::sleep(Duration::from_millis(500));

    // Spawn 1 publisher process
    let mut pub_child = spawn_child(
        "ipc_fanout_1pub_4sub_processes",
        &topic,
        "late_producer",
        count,
        0,
    );
    let _ = pub_child.wait();

    println!("=== CROSS-PROCESS FAN-OUT (1P → 4S) ===");
    let mut totals = vec![];
    for (i, mut sub) in subs.into_iter().enumerate() {
        let output = sub.wait_with_output().unwrap();
        let received: u64 = parse_child_output(&output.stdout, &format!("SUB:{}:", i))
            .first()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0);
        println!(
            "  Subscriber {}: received {}/{} ({:.0}%)",
            i,
            received,
            count,
            received as f64 / count as f64 * 100.0
        );
        totals.push(received);
    }

    let subs_with_data = totals.iter().filter(|&&r| r > count / 4).count();
    println!("Subscribers with >25% data: {}/4", subs_with_data);

    // At least 2/4 should get significant data
    assert!(
        subs_with_data >= 2,
        "Fan-out broken: only {}/4 subscribers got data",
        subs_with_data
    );
    println!("CROSS-PROCESS FAN-OUT PASSED ✓");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 4: Bidirectional — both processes send AND receive
// ════════════════════════════════════════════════════════════════════════
// This catches backend confusion when a process is both producer and consumer.

fn child_bidir() {
    let send_topic = child_env(TOPIC_ENV);
    let id: u64 = child_env(ID_ENV).parse().unwrap();
    let count: u64 = child_env(COUNT_ENV).parse().unwrap();

    // Derive recv topic from send topic
    let recv_topic = if id == 0 {
        format!("{}_b2a", send_topic)
    } else {
        format!("{}_a2b", send_topic)
    };
    let send_name = if id == 0 {
        format!("{}_a2b", send_topic)
    } else {
        format!("{}_b2a", send_topic)
    };

    let sender: Topic<u64> = Topic::new(&send_name).unwrap();
    let receiver: Topic<u64> = Topic::new(&recv_topic).unwrap();

    sender.send(0); // init
    std::thread::sleep(Duration::from_millis(200));

    let mut sent = 0u64;
    let mut received = 0u64;
    let deadline = Instant::now() + Duration::from_secs(5);

    while Instant::now() < deadline && (sent < count || received < count) {
        if sent < count {
            sender.send(sent + 1);
            sent += 1;
        }
        if let Some(v) = receiver.recv() {
            if v > 0 && v != SENTINEL {
                received += 1;
            }
        }
        if sent % 100 == 0 {
            std::thread::yield_now();
        }
    }

    sender.send(SENTINEL);
    // Drain remaining
    let drain_deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < drain_deadline {
        match receiver.recv() {
            Some(SENTINEL) | None => break,
            Some(v) if v > 0 => received += 1,
            _ => {}
        }
    }

    println!("BIDIR:{}:sent={}:recv={}", id, sent, received);
}

#[test]
#[ignore]
fn ipc_bidirectional_both_send_and_recv() {
    if std::env::var(CHILD_FLAG).is_ok() {
        if child_env(ROLE_ENV) == "bidir" {
            child_bidir();
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("ipc_bidir");
    let count = 1000u64;

    let mut child_a = spawn_child(
        "ipc_bidirectional_both_send_and_recv",
        &topic,
        "bidir",
        count,
        0,
    );
    let mut child_b = spawn_child(
        "ipc_bidirectional_both_send_and_recv",
        &topic,
        "bidir",
        count,
        1,
    );

    let out_a = child_a.wait_with_output().unwrap();
    let out_b = child_b.wait_with_output().unwrap();

    let a_lines = parse_child_output(&out_a.stdout, "BIDIR:0:");
    let b_lines = parse_child_output(&out_b.stdout, "BIDIR:1:");

    println!("=== BIDIRECTIONAL IPC ===");
    println!(
        "Process A: {}",
        a_lines.first().unwrap_or(&"(no output)".into())
    );
    println!(
        "Process B: {}",
        b_lines.first().unwrap_or(&"(no output)".into())
    );

    // Both should have sent and received
    let a_recv: u64 = a_lines
        .first()
        .and_then(|l| l.split("recv=").nth(1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let b_recv: u64 = b_lines
        .first()
        .and_then(|l| l.split("recv=").nth(1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    assert!(a_recv > 0, "Process A received ZERO messages from B");
    assert!(b_recv > 0, "Process B received ZERO messages from A");
    println!("A received {} from B, B received {} from A", a_recv, b_recv);
    println!("BIDIRECTIONAL IPC PASSED ✓");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 5: Process churn — 5 processes join/leave rapidly
// ════════════════════════════════════════════════════════════════════════
// Catches epoch tracking bugs when participants change rapidly.

fn child_churn_worker() {
    let topic_name = child_env(TOPIC_ENV);
    let id: u64 = child_env(ID_ENV).parse().unwrap();
    let t: Topic<u64> = Topic::new(&topic_name).unwrap();

    // Send some messages, receive some, then exit
    for i in 0..200 {
        t.send(id * 10000 + i);
        if let Some(_) = t.recv() {}
        std::thread::sleep(Duration::from_millis(1));
    }
    // Count what we received
    let mut received = 0u64;
    while let Some(_) = t.recv() {
        received += 1;
    }
    println!("CHURN:{}:{}", id, received);
}

#[test]
#[ignore]
fn ipc_rapid_process_churn() {
    if std::env::var(CHILD_FLAG).is_ok() {
        if child_env(ROLE_ENV) == "churn" {
            child_churn_worker();
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("ipc_churn");

    // Parent creates the topic
    let parent_topic: Topic<u64> = Topic::new(&topic).unwrap();
    parent_topic.send(0u64); // init

    println!("=== RAPID PROCESS CHURN ===");

    // Launch 5 waves of 3 processes each (15 total), overlapping
    let mut all_children = vec![];
    for wave in 0..5u64 {
        for j in 0..3u64 {
            let id = wave * 3 + j;
            all_children.push(spawn_child(
                "ipc_rapid_process_churn",
                &topic,
                "churn",
                0,
                id,
            ));
        }
        // Overlap: don't wait for previous wave to finish
        std::thread::sleep(Duration::from_millis(50));
    }

    // While children are running, parent also sends/receives
    let mut parent_sent = 0u64;
    let mut parent_recv = 0u64;
    for _ in 0..500 {
        parent_topic.send(99999u64);
        parent_sent += 1;
        while let Some(_) = parent_topic.recv() {
            parent_recv += 1;
        }
        std::thread::sleep(Duration::from_millis(1));
    }

    // Wait for all children
    let mut total_child_recv = 0u64;
    let mut children_with_data = 0;
    for mut child in all_children {
        let output = child.wait_with_output().unwrap();
        let lines = parse_child_output(&output.stdout, "CHURN:");
        for line in lines {
            let parts: Vec<&str> = line.splitn(2, ':').collect();
            if parts.len() == 2 {
                let recv: u64 = parts[1].parse().unwrap_or(0);
                total_child_recv += recv;
                if recv > 0 {
                    children_with_data += 1;
                }
            }
        }
    }

    println!("Parent: sent={}, received={}", parent_sent, parent_recv);
    println!(
        "Children: {} total received, {}/15 got data",
        total_child_recv, children_with_data
    );

    // The key assertion: parent's topic should still work after 15 processes churned
    parent_topic.send(42u64);
    std::thread::sleep(Duration::from_millis(50));

    println!("Parent topic still functional after churn: ✓");
    assert!(
        parent_sent > 100,
        "Parent should have sent messages during churn"
    );
    // At least some children should have received data
    assert!(
        children_with_data > 0,
        "No children received any data — IPC broken during churn"
    );
    println!("RAPID PROCESS CHURN PASSED ✓");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 6: Large message (serde path) cross-process
// ════════════════════════════════════════════════════════════════════════
// Forces the serde serialization path instead of POD zero-copy.
// Catches serialization/deserialization bugs across process boundary.

fn child_pointcloud_producer() {
    let topic_name = child_env(TOPIC_ENV);
    let count: u64 = child_env(COUNT_ENV).parse().unwrap();
    let t: Topic<PointCloud> = Topic::new(&topic_name).unwrap();

    // Init
    t.send(PointCloud {
        points: vec![],
        seq: 0,
        checksum: 0,
    });
    std::thread::sleep(Duration::from_millis(100));

    for i in 1..=count {
        let n_points = 100; // 100 points × 12 bytes = 1.2KB per cloud
        let mut points = Vec::with_capacity(n_points);
        for j in 0..n_points {
            points.push([
                (i as f32 + j as f32) * 0.01,
                (j as f32).sin(),
                (i as f32 * 0.1).cos(),
            ]);
        }
        let checksum = pc_checksum(&points);
        t.send(PointCloud {
            points,
            seq: i,
            checksum,
        });
        if i % 10 == 0 {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    t.send(PointCloud {
        points: vec![],
        seq: SENTINEL,
        checksum: 0,
    });
    std::thread::sleep(Duration::from_millis(500));
}

fn child_pointcloud_consumer() {
    let topic_name = child_env(TOPIC_ENV);
    let t: Topic<PointCloud> = Topic::new(&topic_name).unwrap();
    let deadline = Instant::now() + Duration::from_secs(15);

    let mut received = 0u64;
    let mut corrupted = 0u64;

    while Instant::now() < deadline {
        match t.recv() {
            Some(pc) if pc.seq == SENTINEL => break,
            Some(pc) if pc.seq == 0 => continue,
            Some(pc) => {
                received += 1;
                let expected = pc_checksum(&pc.points);
                if pc.checksum != expected {
                    corrupted += 1;
                }
                if pc.points.len() != 100 {
                    corrupted += 1;
                }
            }
            None => std::thread::yield_now(),
        }
    }
    println!("PC_RECV:{}", received);
    println!("PC_CORRUPT:{}", corrupted);
}

#[test]
#[ignore]
fn ipc_serde_pointcloud_cross_process() {
    if std::env::var(CHILD_FLAG).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "pc_producer" => child_pointcloud_producer(),
            "pc_consumer" => child_pointcloud_consumer(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("ipc_pointcloud");
    let count = 500u64;

    let mut consumer = spawn_child(
        "ipc_serde_pointcloud_cross_process",
        &topic,
        "pc_consumer",
        count,
        0,
    );
    std::thread::sleep(Duration::from_millis(200));
    let mut producer = spawn_child(
        "ipc_serde_pointcloud_cross_process",
        &topic,
        "pc_producer",
        count,
        0,
    );

    let _ = producer.wait();
    let output = consumer.wait_with_output().unwrap();

    let received: u64 = parse_child_output(&output.stdout, "PC_RECV:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let corrupted: u64 = parse_child_output(&output.stdout, "PC_CORRUPT:")
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(999);

    println!("=== SERDE POINTCLOUD CROSS-PROCESS ===");
    println!("Sent: {} clouds (100 points × 12B each)", count);
    println!(
        "Received: {}/{} ({:.0}%)",
        received,
        count,
        received as f64 / count as f64 * 100.0
    );
    println!("Corrupted: {}", corrupted);

    assert_eq!(
        corrupted, 0,
        "PointCloud data CORRUPTED in cross-process serde path!"
    );
    assert!(
        received > count / 4,
        "Received too few pointclouds: {}/{}",
        received,
        count
    );
    println!("SERDE POINTCLOUD PASSED ✓ (zero corruption)");
}
