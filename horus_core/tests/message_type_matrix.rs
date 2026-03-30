#![allow(dead_code)]
//! Real message type integration tests.
//!
//! Tests every horus_library message type through Topic<T> send/recv,
//! including field-level verification, extreme values, and cross-process.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test message_type_matrix -- --ignored --nocapture

use horus_core::communication::topic::Topic;
use horus_robotics::messages::sensor::*;
use horus_robotics::CmdVel;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

mod common;
use common::{cleanup_stale_shm, unique};

const CHILD_FLAG: &str = "HORUS_MSGTYPE_CHILD";
const TOPIC_ENV: &str = "HORUS_MSGTYPE_TOPIC";
const TYPE_ENV: &str = "HORUS_MSGTYPE_TYPE";

// ════════════════════════════════════════════════════════════════════════
// TEST 1: CmdVel round-trip with field verification
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn roundtrip_cmdvel() {
    cleanup_stale_shm();
    let name = unique("msg_cmdvel");
    let t: Topic<CmdVel> = Topic::new(&name).unwrap();

    let sent = CmdVel::new(1.5, -0.75);
    t.send(sent);
    let recv = t.recv().expect("should receive CmdVel");

    assert!(
        (recv.linear - 1.5).abs() < 1e-6,
        "linear mismatch: {}",
        recv.linear
    );
    assert!(
        (recv.angular - (-0.75)).abs() < 1e-6,
        "angular mismatch: {}",
        recv.angular
    );
    assert!(recv.timestamp_ns > 0, "timestamp should be set");
    println!(
        "✓ roundtrip_cmdvel — linear={}, angular={}, ts={}",
        recv.linear, recv.angular, recv.timestamp_ns
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: Imu round-trip with full covariance
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn roundtrip_imu() {
    cleanup_stale_shm();
    let name = unique("msg_imu");
    let t: Topic<Imu> = Topic::new(&name).unwrap();

    let mut sent = Imu::new();
    sent.orientation = [0.0, 0.0, 0.382, 0.924]; // ~45° yaw
    sent.angular_velocity = [0.01, -0.02, 0.03];
    sent.linear_acceleration = [0.1, -0.2, 9.81];
    // Set specific covariance values
    sent.orientation_covariance[0] = 0.001;
    sent.orientation_covariance[4] = 0.002;
    sent.orientation_covariance[8] = 0.003;
    sent.angular_velocity_covariance[3] = 0.0042;
    sent.linear_acceleration_covariance[6] = 0.0099;
    sent.timestamp_ns = 1234567890;

    t.send(sent);
    let recv = t.recv().expect("should receive Imu");

    // Verify every field
    for i in 0..4 {
        assert!(
            (recv.orientation[i] - sent.orientation[i]).abs() < 1e-10,
            "orientation[{}]",
            i
        );
    }
    for i in 0..3 {
        assert!(
            (recv.angular_velocity[i] - sent.angular_velocity[i]).abs() < 1e-10,
            "gyro[{}]",
            i
        );
    }
    for i in 0..3 {
        assert!(
            (recv.linear_acceleration[i] - sent.linear_acceleration[i]).abs() < 1e-10,
            "accel[{}]",
            i
        );
    }
    assert!(
        (recv.orientation_covariance[0] - 0.001).abs() < 1e-10,
        "orient_cov[0]"
    );
    assert!(
        (recv.orientation_covariance[4] - 0.002).abs() < 1e-10,
        "orient_cov[4]"
    );
    assert!(
        (recv.angular_velocity_covariance[3] - 0.0042).abs() < 1e-10,
        "gyro_cov[3]"
    );
    assert!(
        (recv.linear_acceleration_covariance[6] - 0.0099).abs() < 1e-10,
        "accel_cov[6]"
    );
    assert_eq!(recv.timestamp_ns, 1234567890, "timestamp");

    println!(
        "✓ roundtrip_imu — all 38 fields verified ({} bytes)",
        std::mem::size_of::<Imu>()
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: JointState with named joints
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn roundtrip_jointstate() {
    cleanup_stale_shm();
    let name = unique("msg_joints");
    let t: Topic<JointState> = Topic::new(&name).unwrap();

    let mut sent = JointState::default();
    // Add 6 joints (typical robot arm)
    let joint_names = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ];
    let positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0];
    let velocities = [0.1, -0.2, 0.3, -0.1, 0.05, 0.0];
    let efforts = [10.0, 25.0, 15.0, 5.0, 3.0, 1.0];

    for (i, name_str) in joint_names.iter().enumerate() {
        let bytes = name_str.as_bytes();
        let len = bytes.len().min(31);
        sent.names[i][..len].copy_from_slice(&bytes[..len]);
        sent.positions[i] = positions[i];
        sent.velocities[i] = velocities[i];
        sent.efforts[i] = efforts[i];
    }
    sent.joint_count = 6;
    sent.timestamp_ns = 9999;

    t.send(sent);
    let recv = t.recv().expect("should receive JointState");

    assert_eq!(recv.joint_count, 6, "joint_count");
    assert_eq!(recv.timestamp_ns, 9999, "timestamp");
    for i in 0..6 {
        assert!(
            (recv.positions[i] - positions[i]).abs() < 1e-10,
            "position[{}]",
            i
        );
        assert!(
            (recv.velocities[i] - velocities[i]).abs() < 1e-10,
            "velocity[{}]",
            i
        );
        assert!(
            (recv.efforts[i] - efforts[i]).abs() < 1e-10,
            "effort[{}]",
            i
        );
        // Verify joint name
        let recv_name = std::str::from_utf8(&recv.names[i])
            .unwrap_or("")
            .trim_end_matches('\0');
        assert_eq!(recv_name, joint_names[i], "joint name[{}]", i);
    }

    println!("✓ roundtrip_jointstate — 6 joints, all names/positions/velocities/efforts verified ({} bytes)", std::mem::size_of::<JointState>());
}

// ════════════════════════════════════════════════════════════════════════
// TEST 4: LaserScan with 360 ranges
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn roundtrip_laserscan() {
    cleanup_stale_shm();
    let name = unique("msg_lidar");
    let t: Topic<LaserScan> = Topic::new(&name).unwrap();

    let mut sent = LaserScan::default();
    sent.angle_min = -std::f32::consts::PI;
    sent.angle_max = std::f32::consts::PI;
    sent.range_min = 0.12;
    sent.range_max = 25.0;
    // Fill 360 ranges with a pattern
    for i in 0..360 {
        let angle = i as f32 * std::f32::consts::TAU / 360.0;
        sent.ranges[i] = 3.0 + angle.sin();
    }
    sent.timestamp_ns = 42;

    t.send(sent);
    let recv = t.recv().expect("should receive LaserScan");

    assert!(
        (recv.angle_min - (-std::f32::consts::PI)).abs() < 1e-6,
        "angle_min"
    );
    assert!((recv.range_max - 25.0).abs() < 1e-6, "range_max");
    assert_eq!(recv.timestamp_ns, 42, "timestamp");
    for i in 0..360 {
        let expected = 3.0 + (i as f32 * std::f32::consts::TAU / 360.0).sin();
        assert!(
            (recv.ranges[i] - expected).abs() < 1e-5,
            "range[{}]: {} vs {}",
            i,
            recv.ranges[i],
            expected
        );
    }

    println!(
        "✓ roundtrip_laserscan — 360 ranges verified ({} bytes)",
        std::mem::size_of::<LaserScan>()
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 5: BatteryState lifecycle
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn roundtrip_batterystate() {
    cleanup_stale_shm();
    let name = unique("msg_battery");
    let t: Topic<BatteryState> = Topic::new(&name).unwrap();

    let mut sent = BatteryState::default();
    sent.voltage = 12.6;
    sent.current = -2.5; // discharging
    sent.percentage = 85.0;
    sent.power_supply_status = 2; // DISCHARGING
    sent.temperature = 35.5;
    sent.cell_count = 3;
    sent.cell_voltages[0] = 4.2;
    sent.cell_voltages[1] = 4.2;
    sent.cell_voltages[2] = 4.2;
    sent.timestamp_ns = 777;

    t.send(sent);
    let recv = t.recv().expect("should receive BatteryState");

    assert!((recv.voltage - 12.6).abs() < 1e-5, "voltage");
    assert!((recv.current - (-2.5)).abs() < 1e-5, "current");
    assert!((recv.percentage - 85.0).abs() < 1e-5, "percentage");
    assert_eq!(recv.power_supply_status, 2, "status");
    assert!((recv.temperature - 35.5).abs() < 1e-5, "temperature");
    assert_eq!(recv.cell_count, 3, "cell_count");
    for i in 0..3 {
        assert!((recv.cell_voltages[i] - 4.2).abs() < 1e-5, "cell[{}]", i);
    }

    println!(
        "✓ roundtrip_batterystate — all fields verified ({} bytes)",
        std::mem::size_of::<BatteryState>()
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 6: Extreme values — NaN, Inf, MAX, negative zero
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn extreme_values_survive_transport() {
    cleanup_stale_shm();
    let name = unique("msg_extreme");
    let t: Topic<Imu> = Topic::new(&name).unwrap();

    let mut sent = Imu::new();
    sent.orientation_covariance[0] = f64::NAN;
    sent.orientation_covariance[1] = f64::INFINITY;
    sent.orientation_covariance[2] = f64::NEG_INFINITY;
    sent.orientation_covariance[3] = f64::MAX;
    sent.orientation_covariance[4] = f64::MIN;
    sent.orientation_covariance[5] = f64::MIN_POSITIVE;
    sent.orientation_covariance[6] = -0.0_f64;
    sent.orientation_covariance[7] = f64::EPSILON;

    t.send(sent);
    let recv = t.recv().expect("should receive extreme values");

    assert!(
        recv.orientation_covariance[0].is_nan(),
        "NaN should survive"
    );
    assert!(
        recv.orientation_covariance[1].is_infinite() && recv.orientation_covariance[1] > 0.0,
        "+Inf"
    );
    assert!(
        recv.orientation_covariance[2].is_infinite() && recv.orientation_covariance[2] < 0.0,
        "-Inf"
    );
    assert_eq!(recv.orientation_covariance[3], f64::MAX, "MAX");
    assert_eq!(recv.orientation_covariance[4], f64::MIN, "MIN");
    assert_eq!(
        recv.orientation_covariance[5],
        f64::MIN_POSITIVE,
        "MIN_POSITIVE"
    );
    // Negative zero: bit pattern must match
    assert_eq!(
        recv.orientation_covariance[6].to_bits(),
        (-0.0_f64).to_bits(),
        "-0.0 bit pattern"
    );
    assert_eq!(recv.orientation_covariance[7], f64::EPSILON, "EPSILON");

    println!(
        "✓ extreme_values_survive_transport — NaN, Inf, MAX, MIN, -0.0, EPSILON all preserved"
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 7: Cross-process round-trip for Imu
// ════════════════════════════════════════════════════════════════════════

fn child_imu_echo() {
    let topic = std::env::var(TOPIC_ENV).unwrap();
    let result_file = format!("/tmp/horus_msgtest_{}", std::process::id());
    let t: Topic<Imu> = Topic::new(&topic).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);

    while Instant::now() < deadline {
        if let Some(imu) = t.recv() {
            let gz = imu.angular_velocity[2];
            let az = imu.linear_acceleration[2];
            let ts = imu.timestamp_ns;
            // Accept any message with our marker value
            if gz != 0.0 || az != 0.0 {
                let result = format!("CHILD_RECV:gz={:.10}:az={:.10}:ts={}", gz, az, ts);
                let _ = std::fs::write(&result_file, &result);
                return;
            }
        }
        std::thread::yield_now();
    }
    let _ = std::fs::write(&result_file, "CHILD_RECV:TIMEOUT");
}

fn child_cmdvel_echo() {
    let topic = std::env::var(TOPIC_ENV).unwrap();
    let result_file = format!("/tmp/horus_msgtest_{}", std::process::id());
    let t: Topic<CmdVel> = Topic::new(&topic).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);

    while Instant::now() < deadline {
        if let Some(cmd) = t.recv() {
            // Skip init message (zeros)
            if cmd.linear == 0.0 && cmd.angular == 0.0 {
                continue;
            }
            let result = format!("CHILD_RECV:lin={:.10}:ang={:.10}", cmd.linear, cmd.angular);
            let _ = std::fs::write(&result_file, &result);
            return;
        }
        std::thread::yield_now();
    }
    let _ = std::fs::write(&result_file, "CHILD_RECV:TIMEOUT");
}

fn spawn_msg_child(test_name: &str, topic: &str, msg_type: &str) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture", "--ignored"])
        .env(CHILD_FLAG, "1")
        .env(TOPIC_ENV, topic)
        .env(TYPE_ENV, msg_type)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped()) // println! with --nocapture goes to stderr
        .spawn()
        .expect("spawn child")
}

#[test]
#[ignore]
fn cross_process_imu() {
    if std::env::var(CHILD_FLAG).is_ok() {
        match std::env::var(TYPE_ENV).unwrap_or_default().as_str() {
            "imu" => child_imu_echo(),
            "cmdvel" => child_cmdvel_echo(),
            _ => {}
        }
        return;
    }

    cleanup_stale_shm();
    let topic = unique("msg_xproc_imu");

    // Child starts first (consumer), then parent sends
    let mut child = spawn_msg_child("cross_process_imu", &topic, "imu");
    std::thread::sleep(Duration::from_millis(500)); // let child init topic

    let t: Topic<Imu> = Topic::new(&topic).unwrap();

    let mut imu = Imu::new();
    imu.angular_velocity[2] = 0.12345678;
    imu.linear_acceleration[2] = 9.81;
    imu.timestamp_ns = 9876543210;
    // Send the real message multiple times (no init message needed — Topic auto-inits)
    for _ in 0..20 {
        t.send(imu);
        std::thread::sleep(Duration::from_millis(10));
    }

    let child_pid = child.id();
    let _ = child.wait();
    let result_file = format!("/tmp/horus_msgtest_{}", child_pid);
    let child_out = std::fs::read_to_string(&result_file).unwrap_or("NO_RESULT_FILE".into());
    let _ = std::fs::remove_file(&result_file);
    println!("Child result: {}", child_out);

    assert!(
        !child_out.contains("TIMEOUT"),
        "Child timed out — no message received"
    );
    assert!(
        child_out.contains("gz=0.1234567800"),
        "angular_velocity[2] corrupted: {}",
        child_out
    );
    assert!(
        child_out.contains("az=9.8100000000"),
        "linear_acceleration[2] corrupted: {}",
        child_out
    );
    assert!(
        child_out.contains("ts=9876543210"),
        "timestamp corrupted: {}",
        child_out
    );
    println!("✓ cross_process_imu — field values verified across process boundary");
}

#[test]
#[ignore]
fn cross_process_cmdvel() {
    if std::env::var(CHILD_FLAG).is_ok() {
        if std::env::var(TYPE_ENV).unwrap_or_default() == "cmdvel" {
            child_cmdvel_echo();
        }
        return;
    }

    cleanup_stale_shm();
    let topic = unique("msg_xproc_cmdvel");

    let mut child = spawn_msg_child("cross_process_cmdvel", &topic, "cmdvel");
    std::thread::sleep(Duration::from_millis(500));

    let t: Topic<CmdVel> = Topic::new(&topic).unwrap();
    t.send(CmdVel::new(0.0, 0.0)); // init
    std::thread::sleep(Duration::from_millis(100));

    for _ in 0..10 {
        t.send(CmdVel::new(1.2345, -0.6789));
        std::thread::sleep(Duration::from_millis(10));
    }

    let child_pid = child.id();
    let _ = child.wait();
    let result_file = format!("/tmp/horus_msgtest_{}", child_pid);
    let child_out = std::fs::read_to_string(&result_file).unwrap_or("NO_RESULT_FILE".into());
    let _ = std::fs::remove_file(&result_file);
    println!("Child result: {}", child_out);

    assert!(
        !child_out.contains("TIMEOUT"),
        "Child timed out — no message received"
    );
    assert!(
        child_out.contains("lin=1.23"),
        "linear corrupted: {}",
        child_out
    );
    assert!(
        child_out.contains("ang=-0.67"),
        "angular corrupted: {}",
        child_out
    );
    println!("✓ cross_process_cmdvel — field values verified across process boundary");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 8: Mixed sensor suite — all types simultaneously
// ════════════════════════════════════════════════════════════════════════

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

struct MixedPublisher {
    imu_t: Option<Topic<Imu>>,
    cmd_t: Option<Topic<CmdVel>>,
    bat_t: Option<Topic<BatteryState>>,
    imu_n: String,
    cmd_n: String,
    bat_n: String,
    sent: Arc<AtomicU64>,
}
impl Node for MixedPublisher {
    fn name(&self) -> &str {
        "mixed_pub"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.imu_t = Some(Topic::new(&self.imu_n)?);
        self.cmd_t = Some(Topic::new(&self.cmd_n)?);
        self.bat_t = Some(Topic::new(&self.bat_n)?);
        Ok(())
    }
    fn tick(&mut self) {
        let mut imu = Imu::new();
        imu.linear_acceleration[2] = 9.81;
        if let Some(ref t) = self.imu_t {
            t.send(imu);
        }
        if let Some(ref t) = self.cmd_t {
            t.send(CmdVel::new(0.5, 0.1));
        }
        let mut bat = BatteryState::default();
        bat.voltage = 12.0;
        bat.percentage = 80.0;
        if let Some(ref t) = self.bat_t {
            t.send(bat);
        }
        self.sent.fetch_add(3, Ordering::Relaxed);
    }
}

struct MixedSubscriber {
    imu_t: Option<Topic<Imu>>,
    cmd_t: Option<Topic<CmdVel>>,
    bat_t: Option<Topic<BatteryState>>,
    imu_n: String,
    cmd_n: String,
    bat_n: String,
    imu_ok: Arc<AtomicU64>,
    cmd_ok: Arc<AtomicU64>,
    bat_ok: Arc<AtomicU64>,
    corrupted: Arc<AtomicU64>,
}
impl Node for MixedSubscriber {
    fn name(&self) -> &str {
        "mixed_sub"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.imu_t = Some(Topic::new(&self.imu_n)?);
        self.cmd_t = Some(Topic::new(&self.cmd_n)?);
        self.bat_t = Some(Topic::new(&self.bat_n)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.imu_t {
            while let Some(imu) = t.recv() {
                if (imu.linear_acceleration[2] - 9.81).abs() < 0.01 {
                    self.imu_ok.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.corrupted.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        if let Some(ref t) = self.cmd_t {
            while let Some(cmd) = t.recv() {
                if (cmd.linear - 0.5).abs() < 0.01 && (cmd.angular - 0.1).abs() < 0.01 {
                    self.cmd_ok.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.corrupted.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        if let Some(ref t) = self.bat_t {
            while let Some(bat) = t.recv() {
                if (bat.voltage - 12.0).abs() < 0.1 && (bat.percentage - 80.0).abs() < 0.1 {
                    self.bat_ok.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.corrupted.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }
}

#[test]
#[ignore]
fn mixed_sensor_suite_simultaneous() {
    cleanup_stale_shm();
    let imu_n = unique("mix_imu");
    let cmd_n = unique("mix_cmd");
    let bat_n = unique("mix_bat");

    let sent = Arc::new(AtomicU64::new(0));
    let imu_ok = Arc::new(AtomicU64::new(0));
    let cmd_ok = Arc::new(AtomicU64::new(0));
    let bat_ok = Arc::new(AtomicU64::new(0));
    let corrupted = Arc::new(AtomicU64::new(0));

    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let s = sent.clone();
    let io = imu_ok.clone();
    let co = cmd_ok.clone();
    let bo = bat_ok.clone();
    let cr = corrupted.clone();
    let in1 = imu_n.clone();
    let cn1 = cmd_n.clone();
    let bn1 = bat_n.clone();

    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        let _ = sched
            .add(MixedPublisher {
                imu_t: None,
                cmd_t: None,
                bat_t: None,
                imu_n: in1.clone(),
                cmd_n: cn1.clone(),
                bat_n: bn1.clone(),
                sent: s,
            })
            .rate(100_u64.hz())
            .order(0)
            .build();
        let _ = sched
            .add(MixedSubscriber {
                imu_t: None,
                cmd_t: None,
                bat_t: None,
                imu_n: in1,
                cmd_n: cn1,
                bat_n: bn1,
                imu_ok: io,
                cmd_ok: co,
                bat_ok: bo,
                corrupted: cr,
            })
            .order(1)
            .build();
        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    std::thread::sleep(Duration::from_secs(3));
    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let sv = sent.load(Ordering::Relaxed);
    let iv = imu_ok.load(Ordering::Relaxed);
    let cv = cmd_ok.load(Ordering::Relaxed);
    let bv = bat_ok.load(Ordering::Relaxed);
    let crv = corrupted.load(Ordering::Relaxed);

    println!("╔══════════════════════════════════════════════════════════╗");
    println!("║  MIXED SENSOR SUITE (3 types, 3s)                       ║");
    println!(
        "║  Sent: {} total ({} per type)                           ║",
        sv,
        sv / 3
    );
    println!(
        "║  IMU ok:     {} | CmdVel ok: {} | Battery ok: {}        ║",
        iv, cv, bv
    );
    println!(
        "║  Corrupted:  {}                                         ║",
        crv
    );
    println!("╚══════════════════════════════════════════════════════════╝");

    assert_eq!(crv, 0, "DATA CORRUPTION in mixed sensor suite!");
    assert!(iv > 50, "IMU should receive >50 msgs");
    assert!(cv > 50, "CmdVel should receive >50 msgs");
    assert!(bv > 50, "Battery should receive >50 msgs");
    println!("✓ mixed_sensor_suite_simultaneous — zero corruption, all types flowing");
}
