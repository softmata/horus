#![allow(dead_code)]
//! Cross-process chaos WITH SCHEDULERS — the actual production pattern.
//!
//! Each child process runs a real Scheduler with real nodes, exactly
//! like `horus run` does. No raw Topic loops — everything goes through
//! the scheduler tick pipeline: init → rate limiting → tick → shutdown.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test chaos_xp_schedulers -- --ignored --nocapture --test-threads=1

use horus_core::communication::topic::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use horus_robotics::messages::sensor::Imu;
use horus_robotics::CmdVel;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::{cleanup_stale_shm, unique};

const CHILD: &str = "HORUS_XPS_CHILD";
const TOPIC_ENV: &str = "HORUS_XPS_TOPIC";
const ROLE_ENV: &str = "HORUS_XPS_ROLE";
const DURATION_ENV: &str = "HORUS_XPS_DUR";
const RATE_ENV: &str = "HORUS_XPS_RATE";
const ID_ENV: &str = "HORUS_XPS_ID";

fn child_env(k: &str) -> String {
    std::env::var(k).unwrap_or_default()
}

fn result_dir() -> std::path::PathBuf {
    std::env::temp_dir().join("horus_xps")
}

fn write_result(id: u64, key: &str, value: &str) {
    let dir = result_dir();
    let _ = std::fs::create_dir_all(&dir);
    let _ = std::fs::write(dir.join(format!("{}_{}", id, key)), value);
}

fn read_result(id: u64, key: &str) -> String {
    std::fs::read_to_string(result_dir().join(format!("{}_{}", id, key))).unwrap_or_default()
}

fn spawn(
    test: &str,
    topic: &str,
    role: &str,
    dur_ms: u64,
    rate: u64,
    id: u64,
) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    Command::new(exe)
        .args([test, "--exact", "--nocapture", "--ignored"])
        .env(CHILD, "1")
        .env(TOPIC_ENV, topic)
        .env(ROLE_ENV, role)
        .env(DURATION_ENV, dur_ms.to_string())
        .env(RATE_ENV, rate.to_string())
        .env(ID_ENV, id.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap()
}

// ════════════════════════════════════════════════════════════════════════
// Child node definitions — used by child processes
// ════════════════════════════════════════════════════════════════════════

struct ImuPubNode {
    topic: Option<Topic<Imu>>,
    topic_name: String,
    seq: u64,
    published: Arc<AtomicU64>,
}
impl Node for ImuPubNode {
    fn name(&self) -> &str {
        "xps_imu_pub"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let mut imu = Imu::new();
        imu.linear_acceleration = [0.0, 0.0, 9.81];
        imu.angular_velocity[2] = self.seq as f64 * 0.001;
        if let Some(ref t) = self.topic {
            t.send(imu);
        }
        self.seq += 1;
        self.published.fetch_add(1, Ordering::Relaxed);
    }
}

struct ControllerNode {
    imu_name: String,
    cmd_name: String,
    imu_topic: Option<Topic<Imu>>,
    cmd_topic: Option<Topic<CmdVel>>,
    imu_recv: Arc<AtomicU64>,
    cmd_sent: Arc<AtomicU64>,
    corrupted: Arc<AtomicU64>,
}
impl Node for ControllerNode {
    fn name(&self) -> &str {
        "xps_controller"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.imu_topic = Some(Topic::new(&self.imu_name)?);
        self.cmd_topic = Some(Topic::new(&self.cmd_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let mut latest_gz = 0.0f64;
        if let Some(ref t) = self.imu_topic {
            while let Some(imu) = t.recv() {
                self.imu_recv.fetch_add(1, Ordering::Relaxed);
                if (imu.linear_acceleration[2] - 9.81).abs() > 0.01 {
                    self.corrupted.fetch_add(1, Ordering::Relaxed);
                }
                latest_gz = imu.angular_velocity[2];
            }
        }
        let cmd = CmdVel::new(0.5, -latest_gz as f32 * 2.0);
        if let Some(ref t) = self.cmd_topic {
            t.send(cmd);
        }
        self.cmd_sent.fetch_add(1, Ordering::Relaxed);
    }
}

struct MotorNode {
    cmd_name: String,
    cmd_topic: Option<Topic<CmdVel>>,
    cmd_recv: Arc<AtomicU64>,
    corrupted: Arc<AtomicU64>,
}
impl Node for MotorNode {
    fn name(&self) -> &str {
        "xps_motor"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.cmd_topic = Some(Topic::new(&self.cmd_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.cmd_topic {
            while let Some(cmd) = t.recv() {
                if !cmd.linear.is_finite() || !cmd.angular.is_finite() {
                    self.corrupted.fetch_add(1, Ordering::Relaxed);
                }
                self.cmd_recv.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

struct ImuSubNode {
    topic: Option<Topic<Imu>>,
    topic_name: String,
    received: Arc<AtomicU64>,
    corrupted: Arc<AtomicU64>,
}
impl Node for ImuSubNode {
    fn name(&self) -> &str {
        "xps_imu_sub"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.topic {
            while let Some(imu) = t.recv() {
                if (imu.linear_acceleration[2] - 9.81).abs() > 0.01 {
                    self.corrupted.fetch_add(1, Ordering::Relaxed);
                }
                self.received.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

// ════════════════════════════════════════════════════════════════════════
// Child entry points — each runs a REAL scheduler
// ════════════════════════════════════════════════════════════════════════

fn child_sched_imu_pub() {
    let topic = child_env(TOPIC_ENV);
    let dur_ms: u64 = child_env(DURATION_ENV).parse().unwrap_or(3000);
    let rate: u64 = child_env(RATE_ENV).parse().unwrap_or(200);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);
    let published = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(rate.hz()).name("imu_pub_sched");
    let _ = sched
        .add(ImuPubNode {
            topic: None,
            topic_name: topic,
            seq: 0,
            published: published.clone(),
        })
        .rate(rate.hz())
        .order(0)
        .build();
    let _ = sched.run_for(Duration::from_millis(dur_ms));

    write_result(
        id,
        "published",
        &published.load(Ordering::Relaxed).to_string(),
    );
}

fn child_sched_controller() {
    let imu_topic = child_env(TOPIC_ENV);
    let cmd_topic = format!("{}_cmd", imu_topic);
    let dur_ms: u64 = child_env(DURATION_ENV).parse().unwrap_or(3000);
    let rate: u64 = child_env(RATE_ENV).parse().unwrap_or(100);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let imu_recv = Arc::new(AtomicU64::new(0));
    let cmd_sent = Arc::new(AtomicU64::new(0));
    let corrupted = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(rate.hz()).name("ctrl_sched");
    let _ = sched
        .add(ControllerNode {
            imu_name: imu_topic,
            cmd_name: cmd_topic,
            imu_topic: None,
            cmd_topic: None,
            imu_recv: imu_recv.clone(),
            cmd_sent: cmd_sent.clone(),
            corrupted: corrupted.clone(),
        })
        .rate(rate.hz())
        .order(0)
        .build();
    let _ = sched.run_for(Duration::from_millis(dur_ms));

    write_result(
        id,
        "imu_recv",
        &imu_recv.load(Ordering::Relaxed).to_string(),
    );
    write_result(
        id,
        "cmd_sent",
        &cmd_sent.load(Ordering::Relaxed).to_string(),
    );
    write_result(
        id,
        "corrupt",
        &corrupted.load(Ordering::Relaxed).to_string(),
    );
}

fn child_sched_motor() {
    let imu_topic = child_env(TOPIC_ENV);
    let cmd_topic = format!("{}_cmd", imu_topic);
    let dur_ms: u64 = child_env(DURATION_ENV).parse().unwrap_or(3000);
    let rate: u64 = child_env(RATE_ENV).parse().unwrap_or(100);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let cmd_recv = Arc::new(AtomicU64::new(0));
    let corrupted = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(rate.hz()).name("motor_sched");
    let _ = sched
        .add(MotorNode {
            cmd_name: cmd_topic,
            cmd_topic: None,
            cmd_recv: cmd_recv.clone(),
            corrupted: corrupted.clone(),
        })
        .rate(rate.hz())
        .order(0)
        .build();
    let _ = sched.run_for(Duration::from_millis(dur_ms));

    write_result(
        id,
        "cmd_recv",
        &cmd_recv.load(Ordering::Relaxed).to_string(),
    );
    write_result(
        id,
        "corrupt",
        &corrupted.load(Ordering::Relaxed).to_string(),
    );
}

fn child_sched_imu_sub() {
    let topic = child_env(TOPIC_ENV);
    let dur_ms: u64 = child_env(DURATION_ENV).parse().unwrap_or(3000);
    let rate: u64 = child_env(RATE_ENV).parse().unwrap_or(100);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let received = Arc::new(AtomicU64::new(0));
    let corrupted = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new()
        .tick_rate(rate.hz())
        .name(&format!("sub_{}_sched", id));
    let _ = sched
        .add(ImuSubNode {
            topic: None,
            topic_name: topic,
            received: received.clone(),
            corrupted: corrupted.clone(),
        })
        .rate(rate.hz())
        .order(0)
        .build();
    let _ = sched.run_for(Duration::from_millis(dur_ms));

    write_result(
        id,
        "received",
        &received.load(Ordering::Relaxed).to_string(),
    );
    write_result(
        id,
        "corrupt",
        &corrupted.load(Ordering::Relaxed).to_string(),
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 1: 3-process pipeline with schedulers
//         Process A: Scheduler + ImuPubNode at 200Hz
//         Process B: Scheduler + ControllerNode at 100Hz (reads IMU, writes CmdVel)
//         Process C: Scheduler + MotorNode at 100Hz (reads CmdVel)
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn xps_3_process_pipeline_with_schedulers() {
    if std::env::var(CHILD).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "sched_imu_pub" => child_sched_imu_pub(),
            "sched_ctrl" => child_sched_controller(),
            "sched_motor" => child_sched_motor(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("xps_pipeline");
    let dur = 3000u64; // 3 seconds

    let _ = std::fs::remove_dir_all(result_dir());
    let _ = std::fs::create_dir_all(result_dir());

    // Start end of pipeline first
    let mut motor = spawn(
        "xps_3_process_pipeline_with_schedulers",
        &topic,
        "sched_motor",
        dur + 1000,
        100,
        20,
    );
    std::thread::sleep(Duration::from_millis(300));
    let mut ctrl = spawn(
        "xps_3_process_pipeline_with_schedulers",
        &topic,
        "sched_ctrl",
        dur + 500,
        100,
        10,
    );
    std::thread::sleep(Duration::from_millis(300));
    let mut pub_child = spawn(
        "xps_3_process_pipeline_with_schedulers",
        &topic,
        "sched_imu_pub",
        dur,
        200,
        1,
    );

    let _ = pub_child.wait();
    let _ = ctrl.wait();
    let _ = motor.wait();

    let published: u64 = read_result(1, "published").trim().parse().unwrap_or(0);
    let ctrl_imu: u64 = read_result(10, "imu_recv").trim().parse().unwrap_or(0);
    let ctrl_cmd: u64 = read_result(10, "cmd_sent").trim().parse().unwrap_or(0);
    let ctrl_corrupt: u64 = read_result(10, "corrupt").trim().parse().unwrap_or(0);
    let motor_cmd: u64 = read_result(20, "cmd_recv").trim().parse().unwrap_or(0);
    let motor_corrupt: u64 = read_result(20, "corrupt").trim().parse().unwrap_or(0);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  3-PROCESS PIPELINE WITH SCHEDULERS (3s)                    ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!(
        "║  Process A (Sched 200Hz): published {:5} Imu               ║",
        published
    );
    println!(
        "║  Process B (Sched 100Hz): recv {:5} Imu → sent {:5} Cmd   ║",
        ctrl_imu, ctrl_cmd
    );
    println!(
        "║  Process C (Sched 100Hz): recv {:5} CmdVel                 ║",
        motor_cmd
    );
    println!(
        "║  Corruption: ctrl={} motor={}                               ║",
        ctrl_corrupt, motor_corrupt
    );
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(
        ctrl_corrupt + motor_corrupt,
        0,
        "DATA CORRUPTION in scheduler pipeline!"
    );
    assert!(published > 100, "Publisher should send >100 Imu");
    assert!(
        ctrl_imu > 0,
        "Controller should receive Imu from publisher process"
    );
    assert!(
        motor_cmd > 0,
        "Motor should receive CmdVel from controller process"
    );
    println!("✓ xps_3_process_pipeline — schedulers across 3 processes, zero corruption");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: 6-process broadcast — 1 pub scheduler + 5 sub schedulers
//         All at different rates (200, 100, 50, 30, 10 Hz)
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn xps_6_process_broadcast_mixed_rates() {
    if std::env::var(CHILD).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "sched_imu_pub" => child_sched_imu_pub(),
            "sched_imu_sub" => child_sched_imu_sub(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("xps_6proc");
    let dur = 3000u64;

    let _ = std::fs::remove_dir_all(result_dir());
    let _ = std::fs::create_dir_all(result_dir());

    // 5 subscribers at different rates
    let rates = [100u64, 50, 30, 20, 10];
    let mut subs = vec![];
    for (i, rate) in rates.iter().enumerate() {
        subs.push(spawn(
            "xps_6_process_broadcast_mixed_rates",
            &topic,
            "sched_imu_sub",
            dur + 1000,
            *rate,
            100 + i as u64,
        ));
    }
    std::thread::sleep(Duration::from_millis(500));

    // Publisher at 200Hz
    let mut pub_child = spawn(
        "xps_6_process_broadcast_mixed_rates",
        &topic,
        "sched_imu_pub",
        dur,
        200,
        1,
    );

    let _ = pub_child.wait();
    for mut s in subs {
        let _ = s.wait();
    }

    let published: u64 = read_result(1, "published").trim().parse().unwrap_or(0);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  6-PROCESS BROADCAST: 1 pub (200Hz) + 5 subs (mixed)       ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!(
        "║  Published: {:5} Imu                                       ║",
        published
    );

    let mut total_recv = 0u64;
    let mut total_corrupt = 0u64;
    for (i, rate) in rates.iter().enumerate() {
        let id = 100 + i as u64;
        let recv: u64 = read_result(id, "received").trim().parse().unwrap_or(0);
        let corrupt: u64 = read_result(id, "corrupt").trim().parse().unwrap_or(0);
        total_recv += recv;
        total_corrupt += corrupt;
        println!(
            "║  Sub {} ({:3}Hz): recv={:5} ({:5.1}%) corrupt={}             ║",
            i,
            rate,
            recv,
            recv as f64 / published.max(1) as f64 * 100.0,
            corrupt
        );
    }
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(total_corrupt, 0, "Corruption in broadcast!");
    assert!(published > 200, "Publisher should send >200");
    let subs_ok = rates
        .iter()
        .enumerate()
        .filter(|(i, _)| {
            read_result(100 + *i as u64, "received")
                .trim()
                .parse::<u64>()
                .unwrap_or(0)
                > 0
        })
        .count();
    assert!(
        subs_ok >= 3,
        "At least 3/5 subs should get data, got {}",
        subs_ok
    );
    println!(
        "✓ xps_6_process_broadcast — {} recv, zero corruption, {}/5 subs active",
        total_recv, subs_ok
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: Bidirectional with schedulers — 2 processes, each pub+sub
//         Process A: Scheduler with ImuPub + CmdVelSub
//         Process B: Scheduler with CmdVelPub + ImuSub
// ════════════════════════════════════════════════════════════════════════

fn child_sched_bidir_a() {
    let imu_topic = child_env(TOPIC_ENV);
    let cmd_topic = format!("{}_cmd", imu_topic);
    let dur_ms: u64 = child_env(DURATION_ENV).parse().unwrap_or(3000);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let pub_count = Arc::new(AtomicU64::new(0));
    let recv_count = Arc::new(AtomicU64::new(0));
    let corrupt = Arc::new(AtomicU64::new(0));

    struct BiPubNode {
        t: Option<Topic<Imu>>,
        n: String,
        c: Arc<AtomicU64>,
    }
    impl Node for BiPubNode {
        fn name(&self) -> &str {
            "bidir_imu_pub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.t = Some(Topic::new(&self.n)?);
            Ok(())
        }
        fn tick(&mut self) {
            let mut imu = Imu::new();
            imu.linear_acceleration[2] = 9.81;
            if let Some(ref t) = self.t {
                t.send(imu);
            }
            self.c.fetch_add(1, Ordering::Relaxed);
        }
    }
    struct BiSubNode {
        t: Option<Topic<CmdVel>>,
        n: String,
        r: Arc<AtomicU64>,
        c: Arc<AtomicU64>,
    }
    impl Node for BiSubNode {
        fn name(&self) -> &str {
            "bidir_cmd_sub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.t = Some(Topic::new(&self.n)?);
            Ok(())
        }
        fn tick(&mut self) {
            if let Some(ref t) = self.t {
                while let Some(cmd) = t.recv() {
                    if !cmd.linear.is_finite() {
                        self.c.fetch_add(1, Ordering::Relaxed);
                    }
                    self.r.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).name("bidir_a");
    let _ = sched
        .add(BiPubNode {
            t: None,
            n: imu_topic,
            c: pub_count.clone(),
        })
        .rate(100_u64.hz())
        .order(0)
        .build();
    let _ = sched
        .add(BiSubNode {
            t: None,
            n: cmd_topic,
            r: recv_count.clone(),
            c: corrupt.clone(),
        })
        .rate(100_u64.hz())
        .order(1)
        .build();
    let _ = sched.run_for(Duration::from_millis(dur_ms));

    write_result(id, "pub", &pub_count.load(Ordering::Relaxed).to_string());
    write_result(id, "recv", &recv_count.load(Ordering::Relaxed).to_string());
    write_result(id, "corrupt", &corrupt.load(Ordering::Relaxed).to_string());
}

fn child_sched_bidir_b() {
    let imu_topic = child_env(TOPIC_ENV);
    let cmd_topic = format!("{}_cmd", imu_topic);
    let dur_ms: u64 = child_env(DURATION_ENV).parse().unwrap_or(3000);
    let id: u64 = child_env(ID_ENV).parse().unwrap_or(0);

    let pub_count = Arc::new(AtomicU64::new(0));
    let recv_count = Arc::new(AtomicU64::new(0));
    let corrupt = Arc::new(AtomicU64::new(0));

    struct BiSubImuNode {
        t: Option<Topic<Imu>>,
        n: String,
        r: Arc<AtomicU64>,
        c: Arc<AtomicU64>,
    }
    impl Node for BiSubImuNode {
        fn name(&self) -> &str {
            "bidir_imu_sub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.t = Some(Topic::new(&self.n)?);
            Ok(())
        }
        fn tick(&mut self) {
            if let Some(ref t) = self.t {
                while let Some(imu) = t.recv() {
                    if (imu.linear_acceleration[2] - 9.81).abs() > 0.01 {
                        self.c.fetch_add(1, Ordering::Relaxed);
                    }
                    self.r.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }
    struct BiPubCmdNode {
        t: Option<Topic<CmdVel>>,
        n: String,
        c: Arc<AtomicU64>,
    }
    impl Node for BiPubCmdNode {
        fn name(&self) -> &str {
            "bidir_cmd_pub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.t = Some(Topic::new(&self.n)?);
            Ok(())
        }
        fn tick(&mut self) {
            if let Some(ref t) = self.t {
                t.send(CmdVel::new(1.0, 0.0));
            }
            self.c.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).name("bidir_b");
    let _ = sched
        .add(BiSubImuNode {
            t: None,
            n: imu_topic,
            r: recv_count.clone(),
            c: corrupt.clone(),
        })
        .rate(100_u64.hz())
        .order(0)
        .build();
    let _ = sched
        .add(BiPubCmdNode {
            t: None,
            n: cmd_topic,
            c: pub_count.clone(),
        })
        .rate(100_u64.hz())
        .order(1)
        .build();
    let _ = sched.run_for(Duration::from_millis(dur_ms));

    write_result(id, "pub", &pub_count.load(Ordering::Relaxed).to_string());
    write_result(id, "recv", &recv_count.load(Ordering::Relaxed).to_string());
    write_result(id, "corrupt", &corrupt.load(Ordering::Relaxed).to_string());
}

#[test]
#[ignore]
fn xps_bidirectional_2_processes_with_schedulers() {
    if std::env::var(CHILD).is_ok() {
        match child_env(ROLE_ENV).as_str() {
            "bidir_a" => child_sched_bidir_a(),
            "bidir_b" => child_sched_bidir_b(),
            _ => {}
        }
        return;
    }
    cleanup_stale_shm();
    let topic = unique("xps_bidir");
    let dur = 3000u64;

    let _ = std::fs::remove_dir_all(result_dir());
    let _ = std::fs::create_dir_all(result_dir());

    let mut proc_a = spawn(
        "xps_bidirectional_2_processes_with_schedulers",
        &topic,
        "bidir_a",
        dur,
        100,
        40,
    );
    let mut proc_b = spawn(
        "xps_bidirectional_2_processes_with_schedulers",
        &topic,
        "bidir_b",
        dur,
        100,
        50,
    );

    let _ = proc_a.wait();
    let _ = proc_b.wait();

    let a_pub: u64 = read_result(40, "pub").trim().parse().unwrap_or(0);
    let a_recv: u64 = read_result(40, "recv").trim().parse().unwrap_or(0);
    let a_corrupt: u64 = read_result(40, "corrupt").trim().parse().unwrap_or(0);
    let b_pub: u64 = read_result(50, "pub").trim().parse().unwrap_or(0);
    let b_recv: u64 = read_result(50, "recv").trim().parse().unwrap_or(0);
    let b_corrupt: u64 = read_result(50, "corrupt").trim().parse().unwrap_or(0);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  BIDIRECTIONAL: 2 processes, each pub+sub with scheduler    ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!(
        "║  Process A: pub {:5} Imu,    recv {:5} CmdVel  corrupt={}  ║",
        a_pub, a_recv, a_corrupt
    );
    println!(
        "║  Process B: pub {:5} CmdVel, recv {:5} Imu     corrupt={}  ║",
        b_pub, b_recv, b_corrupt
    );
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(a_corrupt + b_corrupt, 0, "Bidirectional corruption!");
    assert!(a_pub > 50, "A should publish >50 Imu");
    assert!(b_pub > 50, "B should publish >50 CmdVel");
    assert!(a_recv > 0, "A should receive CmdVel from B");
    assert!(b_recv > 0, "B should receive Imu from A");
    println!("✓ xps_bidirectional — both directions flow with schedulers, zero corruption");
}
