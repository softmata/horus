//! Cross-process E2E tests — spawns real peer_process binaries.
//!
//! Verifies data integrity across process boundaries for ALL message types.
//! Each test spawns separate OS processes that share SHM.

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::Duration;

use horus_core::communication::{read_latest_slot_bytes, Topic};
use horus_net::priority::{Encoding, Priority, Reliability};
use horus_net::wire::*;
use horus_robotics::messages::vision::CompressedImage;
use horus_robotics::{CmdVel, Imu, JointState, LaserScan, Odometry};
use horus_sys::shm::shm_topics_dir;
use horus_types::Pose2D;

fn unique_name(base: &str) -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static CTR: AtomicU32 = AtomicU32::new(0);
    format!("xp_{base}_{}", CTR.fetch_add(1, Ordering::Relaxed))
}

fn peer_binary() -> PathBuf {
    let mut path = std::env::current_exe().unwrap();
    path.pop();
    path.pop();
    path.push("deps");
    for entry in std::fs::read_dir(&path).unwrap() {
        let entry = entry.unwrap();
        let name = entry.file_name().to_string_lossy().to_string();
        if name.starts_with("peer_process-") && !name.contains('.') {
            return entry.path();
        }
    }
    panic!("peer_process binary not found in {path:?}");
}

fn shm_path(name: &str) -> PathBuf {
    shm_topics_dir().join(format!("horus_{name}"))
}

/// Run a cross-process write+read test for a given message type.
/// Creates SHM in this process, spawns writer process, reads back.
fn cross_process_type_test<T>(msg_type_name: &str, count: u32, validate: impl Fn(&[u8]) -> bool)
where
    T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
{
    let name = unique_name(msg_type_name);
    let binary = peer_binary();

    // This process creates the SHM (keeps it alive)
    let _topic: Topic<T> = Topic::new(&name).expect("create topic");
    let path = shm_path(&name);

    // Spawn writer process
    let writer = Command::new(&binary)
        .args(["write_raw", &name, &count.to_string(), msg_type_name])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn writer");

    let output = writer.wait_with_output().expect("wait writer");
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains(&format!("WROTE_RAW {count}")),
        "{msg_type_name} writer failed: stdout={stdout}, stderr={}",
        String::from_utf8_lossy(&output.stderr)
    );

    // Read from SHM in this process
    let slot = read_latest_slot_bytes(&path, 0);
    assert!(slot.is_some(), "{msg_type_name}: should read data from SHM");
    let slot = slot.unwrap();
    assert!(
        validate(&slot.payload),
        "{msg_type_name}: payload validation failed (len={})",
        slot.payload.len()
    );
}

/// Run a concurrent cross-process test — reader and writer overlap.
fn cross_process_concurrent_test<T>(msg_type_name: &str, write_count: u32, min_reads: u32)
where
    T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
{
    let name = unique_name(&format!("{msg_type_name}_conc"));
    let binary = peer_binary();

    let _topic: Topic<T> = Topic::new(&name).expect("create topic");

    // Start reader first (3s timeout)
    let mut reader = Command::new(&binary)
        .args(["read_raw", &name, "3"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn reader");

    std::thread::sleep(Duration::from_millis(100));

    // Start writer
    let writer = Command::new(&binary)
        .args(["write_raw", &name, &write_count.to_string(), msg_type_name])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn writer");

    writer.wait_with_output().expect("wait writer");
    let reader_out = reader.wait_with_output().expect("wait reader");
    let stdout = String::from_utf8_lossy(&reader_out.stdout);

    // Parse "READ_RAW N CORRUPT M"
    let parts: Vec<&str> = stdout.trim().split_whitespace().collect();
    let read_count: u32 = parts.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
    let corrupt: u32 = parts.get(3).and_then(|s| s.parse().ok()).unwrap_or(0);

    assert!(
        read_count >= min_reads,
        "{msg_type_name} concurrent: expected >={min_reads} reads, got {read_count}"
    );
    assert_eq!(
        corrupt, 0,
        "{msg_type_name} concurrent: {corrupt} corrupt messages detected"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// CmdVel (16 bytes, POD)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_cmdvel_write_read() {
    cross_process_type_test::<CmdVel>("cmdvel", 50, |payload| {
        payload.len() == std::mem::size_of::<CmdVel>()
    });
}

#[test]
fn xproc_cmdvel_concurrent() {
    cross_process_concurrent_test::<CmdVel>("cmdvel", 200, 10);
}

#[test]
fn xproc_cmdvel_data_integrity() {
    let name = unique_name("cmdvel_int");
    let binary = peer_binary();
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path(&name);

    let writer = Command::new(&binary)
        .args(["write_raw", &name, "100", "cmdvel"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    writer.wait_with_output().unwrap();

    let slot = read_latest_slot_bytes(&path, 0).unwrap();
    let cmd: CmdVel = unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const CmdVel) };
    assert_eq!(cmd.linear, 99.0);
    assert!((cmd.angular - 0.99).abs() < 0.001);
}

// ═══════════════════════════════════════════════════════════════════════════
// Imu (~200 bytes, POD)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_imu_write_read() {
    cross_process_type_test::<Imu>("imu", 50, |payload| {
        payload.len() == std::mem::size_of::<Imu>()
    });
}

#[test]
fn xproc_imu_concurrent() {
    cross_process_concurrent_test::<Imu>("imu", 200, 10);
}

#[test]
fn xproc_imu_data_integrity() {
    let name = unique_name("imu_int");
    let binary = peer_binary();
    let _topic: Topic<Imu> = Topic::new(&name).expect("create topic");
    let path = shm_path(&name);

    let writer = Command::new(&binary)
        .args(["write_raw", &name, "100", "imu"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    writer.wait_with_output().unwrap();

    let slot = read_latest_slot_bytes(&path, 0).unwrap();
    let imu: Imu = unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const Imu) };
    assert!((imu.linear_acceleration[0] - 0.99).abs() < 0.001);
    assert_eq!(imu.timestamp_ns, 99000);
}

// ═══════════════════════════════════════════════════════════════════════════
// JointState (~500 bytes, POD, 32 joints)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_jointstate_write_read() {
    cross_process_type_test::<JointState>("jointstate", 50, |payload| {
        payload.len() == std::mem::size_of::<JointState>()
    });
}

#[test]
fn xproc_jointstate_concurrent() {
    cross_process_concurrent_test::<JointState>("jointstate", 100, 5);
}

#[test]
fn xproc_jointstate_data_integrity() {
    let name = unique_name("js_int");
    let binary = peer_binary();
    let _topic: Topic<JointState> = Topic::new(&name).expect("create topic");
    let path = shm_path(&name);

    let writer = Command::new(&binary)
        .args(["write_raw", &name, "50", "jointstate"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    writer.wait_with_output().unwrap();

    let slot = read_latest_slot_bytes(&path, 0).unwrap();
    let js: JointState =
        unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const JointState) };
    assert_eq!(js.joint_count, 6);
    assert!((js.positions[0] - 4.9).abs() < 0.01); // i=49, 49*0.1=4.9
}

// ═══════════════════════════════════════════════════════════════════════════
// Pose2D (32 bytes, POD)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_pose2d_write_read() {
    cross_process_type_test::<Pose2D>("pose2d", 50, |payload| {
        payload.len() == std::mem::size_of::<Pose2D>()
    });
}

#[test]
fn xproc_pose2d_data_integrity() {
    let name = unique_name("pose_int");
    let binary = peer_binary();
    let _topic: Topic<Pose2D> = Topic::new(&name).expect("create topic");
    let path = shm_path(&name);

    let writer = Command::new(&binary)
        .args(["write_raw", &name, "100", "pose2d"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    writer.wait_with_output().unwrap();

    let slot = read_latest_slot_bytes(&path, 0).unwrap();
    let pose: Pose2D = unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const Pose2D) };
    assert_eq!(pose.x, 99.0);
    assert_eq!(pose.y, 49.5);
}

// ═══════════════════════════════════════════════════════════════════════════
// LaserScan (~1500 bytes, POD, 360 ranges)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_laserscan_write_read() {
    cross_process_type_test::<LaserScan>("laserscan", 30, |payload| {
        payload.len() == std::mem::size_of::<LaserScan>()
    });
}

#[test]
fn xproc_laserscan_data_integrity() {
    let name = unique_name("scan_int");
    let binary = peer_binary();
    let _topic: Topic<LaserScan> = Topic::new(&name).expect("create topic");
    let path = shm_path(&name);

    let writer = Command::new(&binary)
        .args(["write_raw", &name, "50", "laserscan"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    writer.wait_with_output().unwrap();

    let slot = read_latest_slot_bytes(&path, 0).unwrap();
    let scan: LaserScan =
        unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const LaserScan) };
    assert!((scan.ranges[0] - 4.9).abs() < 0.01); // i=49, 49*0.1=4.9
}

// ═══════════════════════════════════════════════════════════════════════════
// Odometry (~600 bytes, POD)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_odometry_write_read() {
    cross_process_type_test::<Odometry>("odometry", 50, |payload| {
        payload.len() == std::mem::size_of::<Odometry>()
    });
}

#[test]
fn xproc_odometry_data_integrity() {
    let name = unique_name("odom_int");
    let binary = peer_binary();
    let _topic: Topic<Odometry> = Topic::new(&name).expect("create topic");
    let path = shm_path(&name);

    let writer = Command::new(&binary)
        .args(["write_raw", &name, "100", "odometry"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    writer.wait_with_output().unwrap();

    let slot = read_latest_slot_bytes(&path, 0).unwrap();
    let odom: Odometry =
        unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const Odometry) };
    assert!((odom.pose.x - 0.99).abs() < 0.001);
}

// ═══════════════════════════════════════════════════════════════════════════
// STRESS: Multiple writers, multiple types simultaneously
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_stress_multi_type_simultaneous() {
    let binary = peer_binary();
    let types = ["cmdvel", "imu", "jointstate", "pose2d"];
    let mut children = Vec::new();

    // Create topics in this process
    let names: Vec<String> = types
        .iter()
        .map(|t| unique_name(&format!("stress_{t}")))
        .collect();
    let _topics: Vec<Box<dyn std::any::Any>> = types
        .iter()
        .zip(&names)
        .map(|(t, n)| -> Box<dyn std::any::Any> {
            match *t {
                "cmdvel" => Box::new(Topic::<CmdVel>::new(n).unwrap()),
                "imu" => Box::new(Topic::<Imu>::new(n).unwrap()),
                "jointstate" => Box::new(Topic::<JointState>::new(n).unwrap()),
                "pose2d" => Box::new(Topic::<Pose2D>::new(n).unwrap()),
                _ => unreachable!(),
            }
        })
        .collect();

    // Spawn 4 writers simultaneously (one per type)
    for (t, n) in types.iter().zip(&names) {
        children.push(
            Command::new(&binary)
                .args(["write_raw", n, "100", t])
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .expect("spawn writer"),
        );
    }

    for (child, t) in children.into_iter().zip(&types) {
        let output = child.wait_with_output().expect("wait writer");
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            stdout.contains("WROTE_RAW 100"),
            "{t} writer failed: {stdout}"
        );
    }

    // Verify all topics have data
    for (t, n) in types.iter().zip(&names) {
        let path = shm_path(n);
        let slot = read_latest_slot_bytes(&path, 0);
        assert!(slot.is_some(), "{t} topic should have data after stress");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CompressedImage (serde, variable-size, Vec<u8> payload — THE camera type)
// This is what users send over the network to see what robots see.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn xproc_compressed_image_publish_subscribe() {
    let name = unique_name("compimg");
    let binary = peer_binary();

    // Publisher sends 20 compressed images (50KB each, serde-serialized)
    // Subscriber receives them via Topic::recv() in a separate process
    // Both processes share the same SHM topic.

    // Start subscriber first (creates topic, waits)
    let mut subscriber = Command::new(&binary)
        .args(["subscribe", &name, "3", "compressed_image"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn subscriber");

    std::thread::sleep(Duration::from_millis(200));

    // Start publisher
    let publisher = Command::new(&binary)
        .args(["publish", &name, "20", "compressed_image"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn publisher");

    let pub_out = publisher.wait_with_output().expect("wait publisher");
    let pub_stdout = String::from_utf8_lossy(&pub_out.stdout);
    assert!(
        pub_stdout.contains("PUBLISHED 20"),
        "publisher failed: {pub_stdout}, stderr: {}",
        String::from_utf8_lossy(&pub_out.stderr)
    );

    let sub_out = subscriber.wait_with_output().expect("wait subscriber");
    let sub_stdout = String::from_utf8_lossy(&sub_out.stdout);

    // Subscriber should have received some images
    let count: u32 = sub_stdout
        .trim()
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    assert!(
        count > 0,
        "subscriber should receive >0 compressed images, got {count}. stderr: {}",
        String::from_utf8_lossy(&sub_out.stderr)
    );
}

#[test]
fn xproc_compressed_image_wire_format() {
    // Verify CompressedImage survives encode→UDP→decode as bincode payload
    use horus_robotics::messages::vision::CompressedImage;
    use std::net::UdpSocket;

    let img = CompressedImage::new("jpeg", vec![0xAA; 10_000]);
    let serialized = bincode::serialize(&img).unwrap();

    let (sa, sb, addr) = {
        let a = UdpSocket::bind("127.0.0.1:0").unwrap();
        let b = UdpSocket::bind("127.0.0.1:0").unwrap();
        b.set_read_timeout(Some(Duration::from_millis(500)))
            .unwrap();
        let addr = b.local_addr().unwrap();
        (a, b, addr)
    };

    let hdr = PacketHeader::new(PacketFlags::empty(), 0xCC, 1);
    let out_msg = OutMessage {
        topic_name: "camera.compressed".into(),
        topic_hash: topic_hash("camera.compressed"),
        payload: serialized.clone(),
        timestamp_ns: 0,
        sequence: 1,
        priority: Priority::Bulk,
        reliability: Reliability::None,
        encoding: Encoding::Bincode,
    };
    let mut buf = [0u8; 65536];
    let len = encode_single(&hdr, &out_msg, &mut buf);
    sa.send_to(&buf[..len], addr).unwrap();

    let mut recv_buf = [0u8; 65536];
    let (n, _) = sb.recv_from(&mut recv_buf).unwrap();
    let (_, msgs) = decode_packet(&recv_buf[..n]).unwrap();
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].encoding, Encoding::Bincode);

    // Deserialize back
    let decoded: CompressedImage = bincode::deserialize(&msgs[0].payload).unwrap();
    assert_eq!(decoded.data.len(), 10_000);
    assert_eq!(&decoded.format_str()[..4], "jpeg");
}

// ═══════════════════════════════════════════════════════════════════════════
// USER API TESTS — Topic::send() ↔ Topic::recv() across processes
// These test EXACTLY what a user writes. No raw SHM, no internal APIs.
// ═══════════════════════════════════════════════════════════════════════════

/// Helper: run a publish→subscribe cross-process test using the REAL user API.
fn user_api_cross_process(msg_type: &str, publish_count: u32, min_recv: u32) {
    let name = unique_name(&format!("api_{msg_type}"));
    let binary = peer_binary();

    // Subscriber starts first (creates topic, waits for data)
    let subscriber = Command::new(&binary)
        .args(["subscribe", &name, "3", msg_type])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn subscriber");

    std::thread::sleep(Duration::from_millis(200));

    // Publisher sends data via Topic::send()
    let publisher = Command::new(&binary)
        .args(["publish", &name, &publish_count.to_string(), msg_type])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn publisher");

    let pub_out = publisher.wait_with_output().expect("wait publisher");
    let pub_stdout = String::from_utf8_lossy(&pub_out.stdout);
    assert!(
        pub_stdout.contains(&format!("PUBLISHED {publish_count}")),
        "{msg_type} publisher failed: stdout={pub_stdout}, stderr={}",
        String::from_utf8_lossy(&pub_out.stderr)
    );

    let sub_out = subscriber.wait_with_output().expect("wait subscriber");
    let sub_stdout = String::from_utf8_lossy(&sub_out.stdout);
    let sub_stderr = String::from_utf8_lossy(&sub_out.stderr);

    let recv_count: u32 = sub_stdout
        .trim()
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    assert!(
        recv_count >= min_recv,
        "{msg_type} user API: subscriber received {recv_count}, expected >={min_recv}. stderr: {sub_stderr}"
    );
}

#[test]
fn user_api_cmdvel_cross_process() {
    user_api_cross_process("cmdvel", 50, 1);
}

#[test]
fn user_api_imu_cross_process() {
    user_api_cross_process("imu", 50, 1);
}

#[test]
fn user_api_pose2d_cross_process() {
    user_api_cross_process("pose2d", 50, 1);
}

#[test]
fn user_api_compressed_image_cross_process() {
    user_api_cross_process("compressed_image", 20, 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// NOTE ON TENSOR TYPES (Image, PointCloud, DepthImage)
// ═══════════════════════════════════════════════════════════════════════════
//
// Image/PointCloud/DepthImage use TensorPool (shared memory pool-backed).
// Their wire format is a *descriptor* (metadata struct), not the pixel data.
// The actual pixel data lives in pool-allocated SHM that is separate from
// the topic's ring buffer SHM.
//
// For NETWORK transport:
// - Raw Image should NOT go over the network (too large, ~6MB for 1080p)
// - CompressedImage (JPEG/WebP, ~50-200KB) goes over the network as a
//   regular serialized message — tested via the msg_size_64kb QA matrix test
// - The dual-publish convention handles this: raw for local, compressed for remote
//
// Therefore: tensor types are NOT tested through cross-process SHM raw write/read
// because that's not how they'd flow in production. CompressedImage IS tested
// as a variable-size serialized payload in the QA matrix.
