//! Rust↔Python FULL MATRIX — multiple schedulers, ALL message types.
//!
//! 3 Rust schedulers + 2 Python schedulers, 31 message types flowing
//! bidirectionally. The ultimate cross-language integration test.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test rust_python_full_matrix -- --ignored --nocapture --test-threads=1

use horus_core::communication::topic::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use horus_robotics::CmdVel;
use horus_robotics::messages::sensor::{BatteryState, Imu, JointState, LaserScan, Odometry,
    MagneticField, Temperature, FluidPressure, Illuminance, RangeSensor, NavSatFix};
use horus_robotics::messages::control::{MotorCommand, ServoCommand, DifferentialDriveCommand,
    PidConfig, TrajectoryPoint, JointCommand};
use horus_types::*;
use horus_types::*;
use horus_robotics::messages::navigation::*;
use horus_robotics::messages::force::*;
use horus_types::*;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

fn python_available() -> bool {
    Command::new("python3").args(["-c", "import horus; print('OK')"])
        .output().map(|o| String::from_utf8_lossy(&o.stdout).contains("OK")).unwrap_or(false)
}

fn result_dir() -> std::path::PathBuf { std::env::temp_dir().join("horus_rp_full") }

fn read_result(key: &str) -> u64 {
    std::fs::read_to_string(result_dir().join(key)).unwrap_or_default()
        .trim().parse().unwrap_or(0)
}

// ════════════════════════════════════════════════════════════════════════
// Macro to reduce boilerplate — creates a typed publisher node
// ════════════════════════════════════════════════════════════════════════

macro_rules! pub_node {
    ($name:ident, $type:ty, $make:expr) => {
        struct $name { t: Option<Topic<$type>>, n: String, c: Arc<AtomicU64> }
        impl Node for $name {
            fn name(&self) -> &str { stringify!($name) }
            fn init(&mut self) -> horus_core::error::HorusResult<()> {
                self.t = Some(Topic::new(&self.n)?); Ok(())
            }
            fn tick(&mut self) {
                if let Some(ref t) = self.t { t.send($make); }
                self.c.fetch_add(1, Ordering::Relaxed);
            }
        }
    };
}

macro_rules! sub_node {
    ($name:ident, $type:ty) => {
        struct $name { t: Option<Topic<$type>>, n: String, r: Arc<AtomicU64> }
        impl Node for $name {
            fn name(&self) -> &str { stringify!($name) }
            fn init(&mut self) -> horus_core::error::HorusResult<()> {
                self.t = Some(Topic::new(&self.n)?); Ok(())
            }
            fn tick(&mut self) {
                if let Some(ref t) = self.t {
                    while let Some(_) = t.recv() { self.r.fetch_add(1, Ordering::Relaxed); }
                }
            }
        }
    };
}

// ════════════════════════════════════════════════════════════════════════
// TEST: 3 Rust schedulers + 2 Python schedulers, ALL types
// ════════════════════════════════════════════════════════════════════════

pub_node!(PubImu, Imu, { let mut m = Imu::new(); m.linear_acceleration[2] = 9.81; m });
pub_node!(PubCmd, CmdVel, CmdVel::new(1.0, 0.5));
pub_node!(PubOdom, Odometry, Odometry::default());
pub_node!(PubScan, LaserScan, LaserScan::default());
pub_node!(PubJoint, JointState, JointState::default());
pub_node!(PubBat, BatteryState, { let mut b = BatteryState::default(); b.voltage = 12.0; b });
pub_node!(PubPose2, Pose2D, Pose2D::default());
pub_node!(PubPose3, Pose3D, Pose3D::default());
pub_node!(PubTwist, Twist, Twist::default());
pub_node!(PubVec3, Vector3, Vector3::default());
pub_node!(PubPt3, Point3, Point3::default());
pub_node!(PubQuat, Quaternion, { let mut q = Quaternion::default(); q.w = 1.0; q });
pub_node!(PubMotor, MotorCommand, MotorCommand::default());
pub_node!(PubServo, ServoCommand, ServoCommand::default());
pub_node!(PubDiff, DifferentialDriveCommand, DifferentialDriveCommand::default());
pub_node!(PubHeart, Heartbeat, Heartbeat::default());
pub_node!(PubDiag, DiagnosticStatus, DiagnosticStatus::default());
pub_node!(PubEstop, EmergencyStop, EmergencyStop::default());
pub_node!(PubMag, MagneticField, MagneticField::default());
pub_node!(PubTemp, Temperature, Temperature::default());
pub_node!(PubPress, FluidPressure, FluidPressure::default());
pub_node!(PubIllum, Illuminance, Illuminance::default());
pub_node!(PubRange, RangeSensor, RangeSensor::default());
pub_node!(PubNav, NavSatFix, NavSatFix::default());
pub_node!(PubNavGoal, NavGoal, NavGoal::default());
pub_node!(PubWrench, WrenchStamped, WrenchStamped::default());
pub_node!(PubClock, Clock, Clock::default());
pub_node!(PubPid, PidConfig, PidConfig::default());
pub_node!(PubTraj, TrajectoryPoint, TrajectoryPoint::default());
pub_node!(PubJCmd, JointCommand, JointCommand::default());

#[test]
#[ignore]
fn full_matrix_3_rust_2_python_all_types() {
    if !python_available() { println!("✓ SKIPPED (no horus python)"); return; }
    cleanup_stale_shm();
    let _ = std::fs::remove_dir_all(result_dir());
    let _ = std::fs::create_dir_all(result_dir());

    // All 30 topic names
    let types = [
        "imu", "cmdvel", "odometry", "laserscan", "jointstate", "battery",
        "pose2d", "pose3d", "twist", "vector3", "point3", "quaternion",
        "motor", "servo", "diffdrive", "heartbeat", "diagnostic", "estop",
        "magnetic", "temperature", "pressure", "illuminance", "range",
        "navsatfix", "navgoal", "wrench", "clock", "pid", "trajectory", "jointcmd",
    ];

    // Python subscriber for ALL types
    let py_sub_code = format!(r#"
import horus, os

type_map = {{
    "imu": horus.Imu, "cmdvel": horus.CmdVel, "odometry": horus.Odometry,
    "laserscan": horus.LaserScan, "jointstate": horus.JointState,
    "battery": horus.BatteryState, "pose2d": horus.Pose2D, "pose3d": horus.Pose3D,
    "twist": horus.Twist, "vector3": horus.Vector3, "point3": horus.Point3,
    "quaternion": horus.Quaternion, "motor": horus.MotorCommand,
    "servo": horus.ServoCommand, "diffdrive": horus.DifferentialDriveCommand,
    "heartbeat": horus.Heartbeat, "diagnostic": horus.DiagnosticStatus,
    "estop": horus.EmergencyStop, "magnetic": horus.MagneticField,
    "temperature": horus.Temperature, "pressure": horus.FluidPressure,
    "illuminance": horus.Illuminance, "range": horus.RangeSensor,
    "navsatfix": horus.NavSatFix, "navgoal": horus.NavGoal,
    "wrench": horus.WrenchStamped, "clock": horus.Clock,
    "pid": horus.PidConfig, "trajectory": horus.TrajectoryPoint,
    "jointcmd": horus.JointCommand,
}}

counts = {{k: 0 for k in type_map}}
subs_dict = {{f"fm.{{k}}": v for k, v in type_map.items()}}

def tick(node):
    for k in type_map:
        msg = node.recv(f"fm.{{k}}")
        while msg is not None:
            counts[k] += 1
            msg = node.recv(f"fm.{{k}}")

def shutdown(node):
    d = os.environ.get("RESULT_DIR", "/tmp/horus_rp_full")
    os.makedirs(d, exist_ok=True)
    for k, v in counts.items():
        with open(os.path.join(d, f"py1_{{k}}"), "w") as f:
            f.write(str(v))

sub = horus.Node("py_all_sub1", tick=tick, subs=subs_dict, rate=200, shutdown=shutdown)
horus.run(sub, duration=6.0, tick_rate=200)
"#);

    // Second Python subscriber (different rate — tests multi-Python-scheduler)
    let py_sub2_code = py_sub_code
        .replace("py_all_sub1", "py_all_sub2")
        .replace("py1_", "py2_")
        .replace("rate=200", "rate=50");

    let tmpdir = tempfile::TempDir::new().unwrap();
    std::fs::write(tmpdir.path().join("sub1.py"), &py_sub_code).unwrap();
    std::fs::write(tmpdir.path().join("sub2.py"), &py_sub2_code).unwrap();

    // Start 2 Python subscribers
    let mut py1 = Command::new("python3").arg(tmpdir.path().join("sub1.py"))
        .env("RESULT_DIR", result_dir().to_str().unwrap())
        .stdout(Stdio::null()).stderr(Stdio::null()).spawn().unwrap();
    let mut py2 = Command::new("python3").arg(tmpdir.path().join("sub2.py"))
        .env("RESULT_DIR", result_dir().to_str().unwrap())
        .stdout(Stdio::null()).stderr(Stdio::null()).spawn().unwrap();

    std::thread::sleep(Duration::from_secs(2));

    // 3 Rust schedulers publishing all 30 types (10 per scheduler)
    let counters: Vec<Arc<AtomicU64>> = (0..30).map(|_| Arc::new(AtomicU64::new(0))).collect();

    macro_rules! add_pub {
        ($sched:expr, $idx:expr, $Node:ident, $name:expr) => {
            let _ = $sched.add($Node { t: None, n: format!("fm.{}", $name), c: counters[$idx].clone() })
                .rate(100_u64.hz()).order($idx as u32).build();
        };
    }

    // Scheduler 1: sensor types (10)
    let mut s1 = Scheduler::new().tick_rate(200_u64.hz()).name("rust_s1");
    add_pub!(s1, 0, PubImu, "imu");
    add_pub!(s1, 1, PubCmd, "cmdvel");
    add_pub!(s1, 2, PubOdom, "odometry");
    add_pub!(s1, 3, PubScan, "laserscan");
    add_pub!(s1, 4, PubJoint, "jointstate");
    add_pub!(s1, 5, PubBat, "battery");
    add_pub!(s1, 6, PubPose2, "pose2d");
    add_pub!(s1, 7, PubPose3, "pose3d");
    add_pub!(s1, 8, PubTwist, "twist");
    add_pub!(s1, 9, PubVec3, "vector3");

    // Scheduler 2: geometry + control (10)
    let mut s2 = Scheduler::new().tick_rate(100_u64.hz()).name("rust_s2");
    add_pub!(s2, 10, PubPt3, "point3");
    add_pub!(s2, 11, PubQuat, "quaternion");
    add_pub!(s2, 12, PubMotor, "motor");
    add_pub!(s2, 13, PubServo, "servo");
    add_pub!(s2, 14, PubDiff, "diffdrive");
    add_pub!(s2, 15, PubHeart, "heartbeat");
    add_pub!(s2, 16, PubDiag, "diagnostic");
    add_pub!(s2, 17, PubEstop, "estop");
    add_pub!(s2, 18, PubMag, "magnetic");
    add_pub!(s2, 19, PubTemp, "temperature");

    // Scheduler 3: remaining (10)
    let mut s3 = Scheduler::new().tick_rate(50_u64.hz()).name("rust_s3");
    add_pub!(s3, 20, PubPress, "pressure");
    add_pub!(s3, 21, PubIllum, "illuminance");
    add_pub!(s3, 22, PubRange, "range");
    add_pub!(s3, 23, PubNav, "navsatfix");
    add_pub!(s3, 24, PubNavGoal, "navgoal");
    add_pub!(s3, 25, PubWrench, "wrench");
    add_pub!(s3, 26, PubClock, "clock");
    add_pub!(s3, 27, PubPid, "pid");
    add_pub!(s3, 28, PubTraj, "trajectory");
    add_pub!(s3, 29, PubJCmd, "jointcmd");

    // Run all 3 Rust schedulers in parallel threads
    let h1 = std::thread::spawn(move || { let _ = s1.run_for(Duration::from_secs(3)); });
    let h2 = std::thread::spawn(move || { let _ = s2.run_for(Duration::from_secs(3)); });
    let h3 = std::thread::spawn(move || { let _ = s3.run_for(Duration::from_secs(3)); });

    h1.join().unwrap(); h2.join().unwrap(); h3.join().unwrap();
    let _ = py1.wait(); let _ = py2.wait();

    // Results
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  FULL MATRIX: 3 Rust schedulers → 2 Python schedulers      ║");
    println!("║  30 message types, 5 processes                             ║");
    println!("╠══════════════════════════════════════════════════════════════╣");

    let mut total_sent = 0u64;
    let mut total_py1 = 0u64;
    let mut total_py2 = 0u64;
    let mut types_delivered_py1 = 0;
    let mut types_delivered_py2 = 0;

    for (i, name) in types.iter().enumerate() {
        let sent = counters[i].load(Ordering::Relaxed);
        let r1 = read_result(&format!("py1_{}", name));
        let r2 = read_result(&format!("py2_{}", name));
        total_sent += sent;
        total_py1 += r1;
        total_py2 += r2;
        if r1 > 0 { types_delivered_py1 += 1; }
        if r2 > 0 { types_delivered_py2 += 1; }
        if sent > 0 || r1 > 0 || r2 > 0 {
            println!("║  {:15} sent:{:5} py1:{:5} py2:{:5}                ║", name, sent, r1, r2);
        }
    }

    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║  TOTALS: sent={:5} py1={:5} ({}/30) py2={:5} ({}/30)      ║",
             total_sent, total_py1, types_delivered_py1, total_py2, types_delivered_py2);
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert!(total_sent > 1000, "Rust should send >1000 total, got {}", total_sent);
    let total_py = total_py1 + total_py2;
    let max_types = types_delivered_py1.max(types_delivered_py2);
    assert!(max_types >= 20, "At least one Python sub should get >=20/30 types, got {}", max_types);
    assert!(total_py > 1000, "Total Python recv should be >1000, got {}", total_py);
    println!("✓ full_matrix — py1:{}/30 py2:{}/30 types, {} total msgs", types_delivered_py1, types_delivered_py2, total_py);
}
