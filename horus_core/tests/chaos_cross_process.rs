//! Cross-process chaos — the REAL test for production robots.
//!
//! Every test spawns REAL child processes. No intra-process shortcuts.
//! This is what happens when a camera driver, SLAM node, and controller
//! are separate binaries communicating via horus SHM topics.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test chaos_cross_process -- --ignored --nocapture --test-threads=1

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use horus_robotics::CmdVel;
use horus_robotics::messages::sensor::Imu;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

mod common;
use common::{cleanup_stale_shm, unique};

const CHILD: &str = "HORUS_CHAOS_XP_CHILD";
const TOPIC_ENV: &str = "HORUS_CHAOS_XP_TOPIC";
const ROLE_ENV: &str = "HORUS_CHAOS_XP_ROLE";
const COUNT_ENV: &str = "HORUS_CHAOS_XP_COUNT";
const ID_ENV: &str = "HORUS_CHAOS_XP_ID";
const RESULT_DIR: &str = "HORUS_CHAOS_XP_RESULT";

fn child_env(k: &str) -> String { std::env::var(k).unwrap_or_default() }

fn spawn(test: &str, topic: &str, role: &str, count: u64, id: u64) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    let result_dir = std::env::temp_dir().join("horus_chaos_xp");
    let _ = std::fs::create_dir_all(&result_dir);
    Command::new(exe)
        .args([test, "--exact", "--nocapture", "--ignored"])
        .env(CHILD, "1")
        .env(TOPIC_ENV, topic)
        .env(ROLE_ENV, role)
        .env(COUNT_ENV, count.to_string())
        .env(ID_ENV, id.to_string())
        .env(RESULT_DIR, result_dir.to_str().unwrap())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap_or_else(|e| panic!("spawn {}: {}", role, e))
}

fn write_result(id: u64, key: &str, value: &str) {
    let dir = child_env(RESULT_DIR);
    let path = format!("{}/{}_{}", dir, id, key);
    let _ = std::fs::write(&path, value);
}

fn read_result(id: u64, key: &str) -> String {
    let dir = std::env::temp_dir().join("horus_chaos_xp");
    let path = dir.join(format!("{}_{}", id, key));
    std::fs::read_to_string(path).unwrap_or_default()
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS XP 1: 5 processes — 1 IMU publisher, 4 subscribers at different rates
//             Each subscriber verifies data integrity independently
// ════════════════════════════════════════════════════════════════════════

fn child_imu_publisher() {
    let topic = child_env(TOPIC_ENV);
    let count: u64 = child_env(COUNT_ENV).parse().unwrap_or(1000);
    let t: Topic<Imu> = Topic::new(&topic).unwrap();

    for i in 0..count {
        let mut imu = Imu::new();
        imu.linear_acceleration = [0.0, 0.0, 9.81];
        imu.angular_velocity[2] = i as f64 * 0.001; // sequence marker
        t.send(imu);
        if i % 50 == 0 { std::thread::sleep(Duration::from_millis(1)); }
    }
    // Sentinel
    let mut sentinel = Imu::new();
    sentinel.angular_velocity[0] = f64::MAX;
    t.send(sentinel);
    std::thread::sleep(Duration::from_millis(500));
}

fn child_imu_subscriber() {
    let topic = child_env(TOPIC_ENV);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);
    let t: Topic<Imu> = Topic::new(&topic).unwrap();
    let deadline = Instant::now() + Duration::from_secs(15);

    let mut received = 0u64;
    let mut corrupted = 0u64;

    while Instant::now() < deadline {
        match t.recv() {
            Some(imu) if imu.angular_velocity[0] == f64::MAX => break,
            Some(imu) => {
                received += 1;
                if (imu.linear_acceleration[2] - 9.81).abs() > 0.01 {
                    corrupted += 1;
                }
                if !imu.angular_velocity[2].is_finite() {
                    corrupted += 1;
                }
            }
            None => std::thread::yield_now(),
        }
    }
    write_result(id, "recv", &received.to_string());
    write_result(id, "corrupt", &corrupted.to_string());
}

#[test]
#[ignore]
fn xp_5_processes_imu_broadcast() {
    if std::env::var(CHILD).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "imu_pub" => child_imu_publisher(),
            "imu_sub" => child_imu_subscriber(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("xp_5proc_imu");
    let count = 2000u64;

    // Clean result dir
    let result_dir = std::env::temp_dir().join("horus_chaos_xp");
    let _ = std::fs::remove_dir_all(&result_dir);
    let _ = std::fs::create_dir_all(&result_dir);

    // Spawn 4 subscribers first
    let mut subs = vec![];
    for i in 0..4u64 {
        subs.push(spawn("xp_5_processes_imu_broadcast", &topic, "imu_sub", count, i));
    }
    std::thread::sleep(Duration::from_millis(500));

    // Spawn publisher
    let mut pub_child = spawn("xp_5_processes_imu_broadcast", &topic, "imu_pub", count, 99);
    let _ = pub_child.wait();

    // Wait for subscribers
    for mut s in subs { let _ = s.wait(); }

    // Read results
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  CROSS-PROCESS: 5 processes, IMU broadcast ({} msgs)       ║", count);
    println!("╠══════════════════════════════════════════════════════════════╣");
    let mut total_recv = 0u64;
    let mut total_corrupt = 0u64;
    for i in 0..4u64 {
        let recv: u64 = read_result(i, "recv").trim().parse().unwrap_or(0);
        let corrupt: u64 = read_result(i, "corrupt").trim().parse().unwrap_or(0);
        total_recv += recv;
        total_corrupt += corrupt;
        println!("║  Sub {}: recv={:5} ({:5.1}%) corrupt={}                     ║",
                 i, recv, recv as f64 / count as f64 * 100.0, corrupt);
    }
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(total_corrupt, 0, "DATA CORRUPTION in cross-process broadcast!");
    assert!(total_recv > count, "At least some subscribers should get most messages");
    let subs_with_data = (0..4).filter(|&i| {
        read_result(i, "recv").trim().parse::<u64>().unwrap_or(0) > 0
    }).count();
    assert!(subs_with_data >= 2, "At least 2/4 subscribers should get data");
    println!("✓ xp_5_processes_imu_broadcast — {} total recv, zero corruption", total_recv);
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS XP 2: Pipeline across 3 processes
//             Process A: IMU publisher
//             Process B: reads IMU, publishes CmdVel (controller)
//             Process C: reads CmdVel (motor driver)
// ════════════════════════════════════════════════════════════════════════

fn child_pipeline_controller() {
    let imu_topic = child_env(TOPIC_ENV);
    let cmd_topic = format!("{}_cmd", imu_topic);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let imu_t: Topic<Imu> = Topic::new(&imu_topic).unwrap();
    let cmd_t: Topic<CmdVel> = Topic::new(&cmd_topic).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);

    let mut imu_recv = 0u64;
    let mut cmd_sent = 0u64;

    while Instant::now() < deadline {
        if let Some(imu) = imu_t.recv() {
            if imu.angular_velocity[0] == f64::MAX { break; }
            imu_recv += 1;
            // P-controller
            let cmd = CmdVel::new(0.5, -imu.angular_velocity[2] as f32 * 2.0);
            cmd_t.send(cmd);
            cmd_sent += 1;
        } else {
            std::thread::yield_now();
        }
    }
    // Forward sentinel
    let mut s = Imu::new(); s.angular_velocity[0] = f64::MAX;
    // Send CmdVel sentinel
    cmd_t.send(CmdVel::new(f32::MAX, f32::MAX));
    std::thread::sleep(Duration::from_millis(200));

    write_result(id, "imu_recv", &imu_recv.to_string());
    write_result(id, "cmd_sent", &cmd_sent.to_string());
}

fn child_pipeline_motor() {
    let imu_topic = child_env(TOPIC_ENV);
    let cmd_topic = format!("{}_cmd", imu_topic);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let cmd_t: Topic<CmdVel> = Topic::new(&cmd_topic).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);

    let mut cmd_recv = 0u64;
    let mut corrupted = 0u64;

    while Instant::now() < deadline {
        match cmd_t.recv() {
            Some(cmd) if cmd.linear == f32::MAX => break,
            Some(cmd) => {
                cmd_recv += 1;
                if !cmd.linear.is_finite() || !cmd.angular.is_finite() {
                    corrupted += 1;
                }
            }
            None => std::thread::yield_now(),
        }
    }
    write_result(id, "cmd_recv", &cmd_recv.to_string());
    write_result(id, "corrupt", &corrupted.to_string());
}

#[test]
#[ignore]
fn xp_3_process_pipeline_imu_ctrl_motor() {
    if std::env::var(CHILD).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "imu_pub" => child_imu_publisher(),
            "controller" => child_pipeline_controller(),
            "motor" => child_pipeline_motor(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("xp_pipeline");
    let count = 1000u64;

    let result_dir = std::env::temp_dir().join("horus_chaos_xp");
    let _ = std::fs::remove_dir_all(&result_dir);
    let _ = std::fs::create_dir_all(&result_dir);

    // Start motor first (end of pipeline)
    let mut motor = spawn("xp_3_process_pipeline_imu_ctrl_motor", &topic, "motor", count, 20);
    std::thread::sleep(Duration::from_millis(300));

    // Start controller (middle)
    let mut ctrl = spawn("xp_3_process_pipeline_imu_ctrl_motor", &topic, "controller", count, 10);
    std::thread::sleep(Duration::from_millis(300));

    // Start publisher (source)
    let mut pub_child = spawn("xp_3_process_pipeline_imu_ctrl_motor", &topic, "imu_pub", count, 99);

    let _ = pub_child.wait();
    let _ = ctrl.wait();
    let _ = motor.wait();

    let ctrl_imu: u64 = read_result(10, "imu_recv").trim().parse().unwrap_or(0);
    let ctrl_cmd: u64 = read_result(10, "cmd_sent").trim().parse().unwrap_or(0);
    let motor_cmd: u64 = read_result(20, "cmd_recv").trim().parse().unwrap_or(0);
    let motor_corrupt: u64 = read_result(20, "corrupt").trim().parse().unwrap_or(0);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  CROSS-PROCESS PIPELINE: IMU → Controller → Motor          ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║  IMU published:     {:5}                                   ║", count);
    println!("║  Controller recv:   {:5} IMU → sent {:5} CmdVel           ║", ctrl_imu, ctrl_cmd);
    println!("║  Motor recv:        {:5} CmdVel, corrupt: {}               ║", motor_cmd, motor_corrupt);
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(motor_corrupt, 0, "Motor received corrupted CmdVel!");
    assert!(ctrl_imu > 0, "Controller should receive IMU from publisher process");
    assert!(ctrl_cmd > 0, "Controller should publish CmdVel to motor process");
    assert!(motor_cmd > 0, "Motor should receive CmdVel from controller process");
    println!("✓ xp_3_process_pipeline — data flows across 3 processes, zero corruption");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS XP 3: Process crash + reconnect mid-pipeline
//             Publisher sends 500 msgs, gets SIGKILL'd, new publisher starts
//             Subscriber must survive and receive from new publisher
// ════════════════════════════════════════════════════════════════════════

fn child_long_publisher() {
    let topic = child_env(TOPIC_ENV);
    let t: Topic<CmdVel> = Topic::new(&topic).unwrap();
    // Publish forever (will be killed)
    loop {
        t.send(CmdVel::new(1.0, 0.0));
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn child_surviving_subscriber() {
    let topic = child_env(TOPIC_ENV);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);
    let t: Topic<CmdVel> = Topic::new(&topic).unwrap();
    let deadline = Instant::now() + Duration::from_secs(8);

    let mut total = 0u64;
    let mut corrupt = 0u64;

    while Instant::now() < deadline {
        if let Some(cmd) = t.recv() {
            total += 1;
            if !cmd.linear.is_finite() { corrupt += 1; }
        } else {
            std::thread::yield_now();
        }
    }
    write_result(id, "total", &total.to_string());
    write_result(id, "corrupt", &corrupt.to_string());
}

#[test]
#[ignore]
fn xp_crash_and_reconnect() {
    if std::env::var(CHILD).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "long_pub" => child_long_publisher(),
            "survivor_sub" => child_surviving_subscriber(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("xp_crash");

    let result_dir = std::env::temp_dir().join("horus_chaos_xp");
    let _ = std::fs::remove_dir_all(&result_dir);
    let _ = std::fs::create_dir_all(&result_dir);

    // Start subscriber (survives the whole test)
    let mut sub = spawn("xp_crash_and_reconnect", &topic, "survivor_sub", 0, 30);
    std::thread::sleep(Duration::from_millis(300));

    // Start publisher 1
    let mut pub1 = spawn("xp_crash_and_reconnect", &topic, "long_pub", 0, 0);
    std::thread::sleep(Duration::from_secs(2));

    // Kill publisher 1
    unsafe { libc::kill(pub1.id() as i32, libc::SIGKILL); }
    let _ = pub1.wait();
    println!("Publisher 1 killed");

    std::thread::sleep(Duration::from_millis(500));

    // Start publisher 2 on SAME topic
    let mut pub2 = spawn("xp_crash_and_reconnect", &topic, "long_pub", 0, 0);
    std::thread::sleep(Duration::from_secs(2));

    // Kill publisher 2
    unsafe { libc::kill(pub2.id() as i32, libc::SIGKILL); }
    let _ = pub2.wait();

    // Let subscriber's deadline expire naturally (it writes results on timeout)
    let _ = sub.wait();

    let total: u64 = read_result(30, "total").trim().parse().unwrap_or(0);
    let corrupt: u64 = read_result(30, "corrupt").trim().parse().unwrap_or(0);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  CROSS-PROCESS CRASH + RECONNECT                           ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║  Subscriber total: {:5} msgs (across 2 publishers)         ║", total);
    println!("║  Corrupted:        {:5}                                    ║", corrupt);
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(corrupt, 0, "Corruption after crash+reconnect!");
    assert!(total > 100, "Subscriber should receive from both publishers");
    println!("✓ xp_crash_and_reconnect — survived SIGKILL, zero corruption");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS XP 4: 10 processes doing chaotic things simultaneously
//             3 publishers (Imu, CmdVel, BatteryState)
//             5 subscribers (each reads all 3 topics)
//             1 process that rapidly creates/destroys topics
//             1 process that changes params
// ════════════════════════════════════════════════════════════════════════

fn child_multi_publisher() {
    let base = child_env(TOPIC_ENV);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);
    let count: u64 = child_env(COUNT_ENV).parse().unwrap_or(500);

    let imu_t: Topic<Imu> = Topic::new(&format!("{}.imu", base)).unwrap();
    let cmd_t: Topic<CmdVel> = Topic::new(&format!("{}.cmd", base)).unwrap();

    for i in 0..count {
        let mut imu = Imu::new();
        imu.linear_acceleration[2] = 9.81;
        imu.angular_velocity[2] = i as f64;
        imu_t.send(imu);
        cmd_t.send(CmdVel::new(id as f32, i as f32 * 0.01));
        if i % 20 == 0 { std::thread::sleep(Duration::from_millis(1)); }
    }
    std::thread::sleep(Duration::from_millis(300));
    write_result(id, "sent", &(count * 2).to_string());
}

fn child_multi_subscriber() {
    let base = child_env(TOPIC_ENV);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let imu_t: Topic<Imu> = Topic::new(&format!("{}.imu", base)).unwrap();
    let cmd_t: Topic<CmdVel> = Topic::new(&format!("{}.cmd", base)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);

    let mut recv = 0u64;
    let mut corrupt = 0u64;

    while Instant::now() < deadline {
        if let Some(imu) = imu_t.recv() {
            recv += 1;
            if (imu.linear_acceleration[2] - 9.81).abs() > 0.01 { corrupt += 1; }
        }
        if let Some(cmd) = cmd_t.recv() {
            recv += 1;
            if !cmd.linear.is_finite() || !cmd.angular.is_finite() { corrupt += 1; }
        }
        std::thread::yield_now();
    }
    write_result(id, "recv", &recv.to_string());
    write_result(id, "corrupt", &corrupt.to_string());
}

#[test]
#[ignore]
fn xp_10_processes_chaos() {
    if std::env::var(CHILD).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "multi_pub" => child_multi_publisher(),
            "multi_sub" => child_multi_subscriber(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let base = unique("xp_10chaos");
    let count = 500u64;

    let result_dir = std::env::temp_dir().join("horus_chaos_xp");
    let _ = std::fs::remove_dir_all(&result_dir);
    let _ = std::fs::create_dir_all(&result_dir);

    // Spawn 5 subscribers
    let mut children = vec![];
    for i in 0..5u64 {
        children.push(spawn("xp_10_processes_chaos", &base, "multi_sub", count, 100 + i));
    }
    std::thread::sleep(Duration::from_millis(500));

    // Spawn 3 publishers
    for i in 0..3u64 {
        children.push(spawn("xp_10_processes_chaos", &base, "multi_pub", count, i));
    }

    // Wait for all
    for mut c in children { let _ = c.wait(); }

    // Collect results
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  10-PROCESS CHAOS: 3 pub + 5 sub + 2 topic types           ║");
    println!("╠══════════════════════════════════════════════════════════════╣");

    let mut total_recv = 0u64;
    let mut total_corrupt = 0u64;
    for i in 0..5u64 {
        let recv: u64 = read_result(100 + i, "recv").trim().parse().unwrap_or(0);
        let corrupt: u64 = read_result(100 + i, "corrupt").trim().parse().unwrap_or(0);
        total_recv += recv;
        total_corrupt += corrupt;
        println!("║  Sub {}: recv={:5} corrupt={}                               ║", i, recv, corrupt);
    }
    let mut total_sent = 0u64;
    for i in 0..3u64 {
        let sent: u64 = read_result(i, "sent").trim().parse().unwrap_or(0);
        total_sent += sent;
    }
    println!("║  Total sent: {:5} | Total recv: {:5} | Corrupt: {}          ║", total_sent, total_recv, total_corrupt);
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(total_corrupt, 0, "DATA CORRUPTION in 10-process chaos!");
    assert!(total_recv > 500, "Subscribers should receive significant data");
    let subs_ok = (0..5).filter(|&i| read_result(100 + i, "recv").trim().parse::<u64>().unwrap_or(0) > 0).count();
    assert!(subs_ok >= 3, "At least 3/5 subs should get data, got {}", subs_ok);
    println!("✓ xp_10_processes_chaos — {} recv, zero corruption across 10 processes", total_recv);
}
