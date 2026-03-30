//! User story tests — the exact code patterns a robotics developer writes.
//!
//! These are NOT backend tests. Each test is a real scenario that a developer
//! deploying HORUS on a production robot would encounter. They use:
//! - Real horus_library message types (CmdVel, Imu, JointState)
//! - Real scheduler configurations (.rate(), .budget(), .order(), .deterministic())
//! - Multiple schedulers sharing topics
//! - Budget violations and degradation
//! - Mixed execution classes (RT, Compute, BestEffort)
//!
//! If any of these fail, a real user will hit the bug.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test user_stories -- --ignored --nocapture

use horus_core::communication::topic::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use horus_robotics::messages::sensor::Imu;
use horus_robotics::CmdVel;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod common;
use common::{cleanup_stale_shm, unique};

// ════════════════════════════════════════════════════════════════════════
// STORY 1: "I have an IMU node at 200Hz publishing to a controller at 100Hz"
// ════════════════════════════════════════════════════════════════════════
// This is THE most common pattern in robotics. If this doesn't work,
// nothing works.

struct ImuDriverNode {
    topic_name: String,
    topic: Option<Topic<Imu>>,
    seq: u64,
    published: Arc<AtomicU64>,
}
impl Node for ImuDriverNode {
    fn name(&self) -> &str {
        "imu_driver"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let mut imu = Imu::new();
        imu.linear_acceleration = [0.0, 0.0, 9.81];
        imu.angular_velocity = [(self.seq as f64 * 0.01).sin() * 0.05, 0.0, 0.0];
        imu.timestamp_ns = self.seq * 5_000_000; // 200Hz = 5ms
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
    imu_received: Arc<AtomicU64>,
    cmd_sent: Arc<AtomicU64>,
}
impl Node for ControllerNode {
    fn name(&self) -> &str {
        "controller"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.imu_topic = Some(Topic::new(&self.imu_name)?);
        self.cmd_topic = Some(Topic::new(&self.cmd_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        // Read latest IMU (drain queue, use last)
        let mut latest = None;
        if let Some(ref t) = self.imu_topic {
            while let Some(imu) = t.recv() {
                latest = Some(imu);
                self.imu_received.fetch_add(1, Ordering::Relaxed);
            }
        }
        // Compute control output
        if let Some(imu) = latest {
            let cmd = CmdVel::new(
                0.5,                                   // forward at 0.5 m/s
                -imu.angular_velocity[0] as f32 * 2.0, // P-controller on yaw
            );
            if let Some(ref t) = self.cmd_topic {
                t.send(cmd);
                self.cmd_sent.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

struct MotorDriverNode {
    cmd_name: String,
    cmd_topic: Option<Topic<CmdVel>>,
    commands_applied: Arc<AtomicU64>,
    last_linear: std::sync::Mutex<f32>,
    last_angular: std::sync::Mutex<f32>,
}
impl Node for MotorDriverNode {
    fn name(&self) -> &str {
        "motor_driver"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.cmd_topic = Some(Topic::new(&self.cmd_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.cmd_topic {
            while let Some(cmd) = t.recv() {
                *self.last_linear.lock().unwrap() = cmd.linear;
                *self.last_angular.lock().unwrap() = cmd.angular;
                self.commands_applied.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

#[test]
#[ignore]
fn story_imu_to_controller_to_motor() {
    cleanup_stale_shm();
    let imu_topic = unique("imu.data");
    let cmd_topic = unique("cmd_vel");

    let imu_published = Arc::new(AtomicU64::new(0));
    let imu_received = Arc::new(AtomicU64::new(0));
    let cmd_sent = Arc::new(AtomicU64::new(0));
    let cmds_applied = Arc::new(AtomicU64::new(0));

    let ip = imu_published.clone();
    let ir = imu_received.clone();
    let cs = cmd_sent.clone();
    let ca = cmds_applied.clone();

    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(200_u64.hz());

        let _ = sched
            .add(ImuDriverNode {
                topic_name: imu_topic.clone(),
                topic: None,
                seq: 0,
                published: ip,
            })
            .rate(200_u64.hz())
            .order(0)
            .build();

        let _ = sched
            .add(ControllerNode {
                imu_name: imu_topic,
                cmd_name: cmd_topic.clone(),
                imu_topic: None,
                cmd_topic: None,
                imu_received: ir,
                cmd_sent: cs,
            })
            .rate(100_u64.hz())
            .order(1)
            .build();

        let _ = sched
            .add(MotorDriverNode {
                cmd_name: cmd_topic,
                cmd_topic: None,
                commands_applied: ca,
                last_linear: std::sync::Mutex::new(0.0),
                last_angular: std::sync::Mutex::new(0.0),
            })
            .rate(100_u64.hz())
            .order(2)
            .build();

        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_micros(4500));
        }
    });

    std::thread::sleep(Duration::from_secs(5));
    running.store(false, Ordering::Relaxed);
    handle.join().unwrap();

    let ip = imu_published.load(Ordering::Relaxed);
    let ir = imu_received.load(Ordering::Relaxed);
    let cs = cmd_sent.load(Ordering::Relaxed);
    let ca = cmds_applied.load(Ordering::Relaxed);

    println!("=== STORY: IMU → Controller → Motor (5s) ===");
    println!("IMU published:      {} ({:.0} Hz)", ip, ip as f64 / 5.0);
    println!(
        "IMU received by ctrl: {} ({:.0}%)",
        ir,
        ir as f64 / ip.max(1) as f64 * 100.0
    );
    println!("Commands sent:      {} ({:.0} Hz)", cs, cs as f64 / 5.0);
    println!(
        "Commands applied:   {} ({:.0}%)",
        ca,
        ca as f64 / cs.max(1) as f64 * 100.0
    );

    assert!(ip > 100, "IMU should publish >100 messages in 5s");
    assert!(ir > 50, "Controller should receive >50 IMU messages");
    assert!(cs > 50, "Controller should send >50 commands");
    assert!(ca > 25, "Motor should apply >25 commands");
    // Pipeline must flow: every stage receives from the previous
    assert!(
        ir > 0 && cs > 0 && ca > 0,
        "Pipeline broken — data not flowing"
    );
    println!("STORY 1 PASSED ✓\n");
}

// ════════════════════════════════════════════════════════════════════════
// STORY 2: "Two schedulers in same process sharing a topic"
// ════════════════════════════════════════════════════════════════════════
// Common pattern: RT scheduler for control, separate scheduler for perception.
// Both read from the same IMU topic.

struct CountingRecvNode {
    topic_name: String,
    node_name: String,
    topic: Option<Topic<Imu>>,
    count: Arc<AtomicU64>,
}
impl Node for CountingRecvNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.topic {
            while t.recv().is_some() {
                self.count.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

#[test]
#[ignore]
fn story_two_schedulers_shared_topic() {
    cleanup_stale_shm();
    let imu_topic = unique("shared.imu");

    let published = Arc::new(AtomicU64::new(0));
    let ctrl_recv = Arc::new(AtomicU64::new(0));
    let percept_recv = Arc::new(AtomicU64::new(0));

    let running = Arc::new(AtomicBool::new(true));

    // Scheduler 1: IMU publisher + control subscriber (500Hz)
    let r1 = running.clone();
    let p1 = published.clone();
    let c1 = ctrl_recv.clone();
    let t1 = imu_topic.clone();
    let sched1_handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new()
            .tick_rate(500_u64.hz())
            .name("control_sched");

        let _ = sched
            .add(ImuDriverNode {
                topic_name: t1.clone(),
                topic: None,
                seq: 0,
                published: p1,
            })
            .rate(500_u64.hz())
            .order(0)
            .build();

        let _ = sched
            .add(CountingRecvNode {
                topic_name: t1,
                node_name: "ctrl_consumer".into(),
                topic: None,
                count: c1,
            })
            .rate(500_u64.hz())
            .order(1)
            .build();

        while r1.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_micros(1500));
        }
    });

    // Scheduler 2: Perception subscriber (30Hz) — reads same IMU topic
    let r2 = running.clone();
    let p2 = percept_recv.clone();
    let t2 = imu_topic.clone();
    let sched2_handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new()
            .tick_rate(30_u64.hz())
            .name("perception_sched");

        let _ = sched
            .add(CountingRecvNode {
                topic_name: t2,
                node_name: "perception_consumer".into(),
                topic: None,
                count: p2,
            })
            .rate(30_u64.hz())
            .order(0)
            .build();

        while r2.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(30));
        }
    });

    std::thread::sleep(Duration::from_secs(5));
    running.store(false, Ordering::Relaxed);
    sched1_handle.join().unwrap();
    sched2_handle.join().unwrap();

    let p = published.load(Ordering::Relaxed);
    let c = ctrl_recv.load(Ordering::Relaxed);
    let pr = percept_recv.load(Ordering::Relaxed);

    println!("=== STORY: Two Schedulers, One Topic (5s) ===");
    println!(
        "IMU published (sched1):        {} ({:.0} Hz)",
        p,
        p as f64 / 5.0
    );
    println!(
        "Control recv (sched1, 500Hz):  {} ({:.0}%)",
        c,
        c as f64 / p.max(1) as f64 * 100.0
    );
    println!(
        "Perception recv (sched2, 30Hz): {} ({:.0}%)",
        pr,
        pr as f64 / p.max(1) as f64 * 100.0
    );

    assert!(p > 100, "IMU should publish >100 msgs");
    assert!(c > 50, "Control scheduler should receive IMU data");
    assert!(
        pr > 0,
        "Perception scheduler should receive IMU data from other scheduler"
    );
    println!("STORY 2 PASSED ✓\n");
}

// ════════════════════════════════════════════════════════════════════════
// STORY 3: "Node execution order is respected"
// ════════════════════════════════════════════════════════════════════════
// Critical: .order(0) MUST run before .order(1) within same tick.

struct OrderTrackerNode {
    node_name: String,
    order_log: Arc<std::sync::Mutex<Vec<String>>>,
}
impl Node for OrderTrackerNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn tick(&mut self) {
        self.order_log.lock().unwrap().push(self.node_name.clone());
    }
}

#[test]
#[ignore]
fn story_execution_order_respected() {
    cleanup_stale_shm();

    let log = Arc::new(std::sync::Mutex::new(Vec::new()));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);

    // Add in REVERSE order to verify ordering isn't just insertion order
    let _ = sched
        .add(OrderTrackerNode {
            node_name: "C_order2".into(),
            order_log: log.clone(),
        })
        .order(2)
        .build();

    let _ = sched
        .add(OrderTrackerNode {
            node_name: "A_order0".into(),
            order_log: log.clone(),
        })
        .order(0)
        .build();

    let _ = sched
        .add(OrderTrackerNode {
            node_name: "B_order1".into(),
            order_log: log.clone(),
        })
        .order(1)
        .build();

    // Run 100 ticks
    for _ in 0..100 {
        let _ = sched.tick_once();
    }

    let entries = log.lock().unwrap();
    println!("=== STORY: Execution Order ===");
    println!("Total entries: {}", entries.len());

    // Check that within each tick, order is A→B→C
    let mut violations = 0;
    for chunk in entries.chunks(3) {
        if chunk.len() == 3 {
            if chunk[0] != "A_order0" || chunk[1] != "B_order1" || chunk[2] != "C_order2" {
                violations += 1;
                if violations <= 3 {
                    println!("  Violation: {:?}", chunk);
                }
            }
        }
    }

    println!(
        "Order violations: {}/{} ticks",
        violations,
        entries.len() / 3
    );
    assert_eq!(
        violations, 0,
        "Execution order violated! .order() not respected"
    );
    println!("STORY 3 PASSED ✓\n");
}

// ════════════════════════════════════════════════════════════════════════
// STORY 4: "Budget violation triggers degradation, not crash"
// ════════════════════════════════════════════════════════════════════════
// User sets .budget(1.ms()) but the node sometimes takes 5ms.
// The scheduler should warn/degrade, NOT crash.

struct SlowNode {
    tick_count: u64,
}
impl Node for SlowNode {
    fn name(&self) -> &str {
        "slow_node"
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        // Every 10th tick, exceed budget by sleeping 5ms
        if self.tick_count % 10 == 0 {
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}

struct HealthyNode {
    tick_count: Arc<AtomicU64>,
}
impl Node for HealthyNode {
    fn name(&self) -> &str {
        "healthy_node"
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
#[ignore]
fn story_budget_violation_degrades_not_crashes() {
    cleanup_stale_shm();

    let healthy_ticks = Arc::new(AtomicU64::new(0));
    let hc = healthy_ticks.clone();

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());

    // Slow node with tight budget
    let _ = sched
        .add(SlowNode { tick_count: 0 })
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .order(0)
        .build();

    // Healthy node should keep running even when slow node violates budget
    let _ = sched
        .add(HealthyNode { tick_count: hc })
        .rate(100_u64.hz())
        .order(1)
        .build();

    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let handle = std::thread::spawn(move || {
        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    std::thread::sleep(Duration::from_secs(3));
    running.store(false, Ordering::Relaxed);
    handle.join().unwrap();

    let ht = healthy_ticks.load(Ordering::Relaxed);
    println!("=== STORY: Budget Violation Degradation ===");
    println!("Healthy node ticks: {} ({:.0} Hz)", ht, ht as f64 / 3.0);
    println!("(Slow node violates 1ms budget every 10th tick with 5ms sleep)");

    // Healthy node MUST keep running (scheduler didn't crash)
    assert!(
        ht > 50,
        "Healthy node should keep running despite slow node's budget violation, got {} ticks",
        ht
    );
    println!("STORY 4 PASSED ✓ (scheduler survived budget violations)\n");
}

// ════════════════════════════════════════════════════════════════════════
// STORY 5: "Deterministic mode produces identical output every run"
// ════════════════════════════════════════════════════════════════════════
// User runs simulation, records output, expects replay to match.

struct DeterministicPubNode {
    topic_name: String,
    topic: Option<Topic<CmdVel>>,
    tick: u64,
}
impl Node for DeterministicPubNode {
    fn name(&self) -> &str {
        "det_pub"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let cmd = CmdVel::new(
            (self.tick as f32 * 0.1).sin(),
            (self.tick as f32 * 0.05).cos(),
        );
        if let Some(ref t) = self.topic {
            t.send(cmd);
        }
        self.tick += 1;
    }
}

struct DeterministicSubNode {
    topic_name: String,
    topic: Option<Topic<CmdVel>>,
    log: Arc<std::sync::Mutex<Vec<(f32, f32)>>>,
}
impl Node for DeterministicSubNode {
    fn name(&self) -> &str {
        "det_sub"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.topic {
            while let Some(cmd) = t.recv() {
                self.log.lock().unwrap().push((cmd.linear, cmd.angular));
            }
        }
    }
}

fn run_deterministic_cmdvel(n_ticks: usize) -> Vec<(f32, f32)> {
    let name = unique("det_cmdvel");
    let log: Arc<std::sync::Mutex<Vec<(f32, f32)>>> = Arc::new(std::sync::Mutex::new(Vec::new()));
    let log_clone = Arc::clone(&log);

    // Scope the scheduler so it's dropped before we read the log
    {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);

        let _ = sched
            .add(DeterministicPubNode {
                topic_name: name.clone(),
                topic: None,
                tick: 0,
            })
            .order(0)
            .build();

        let _ = sched
            .add(DeterministicSubNode {
                topic_name: name,
                topic: None,
                log: log_clone,
            })
            .order(1)
            .build();

        for _ in 0..n_ticks {
            let _ = sched.tick_once();
        }
    }

    Arc::try_unwrap(log).unwrap().into_inner().unwrap()
}

#[test]
#[ignore]
fn story_deterministic_mode_with_real_cmdvel() {
    cleanup_stale_shm();

    let run1 = run_deterministic_cmdvel(500);
    cleanup_stale_shm();
    let run2 = run_deterministic_cmdvel(500);

    println!("=== STORY: Deterministic CmdVel ===");
    println!("Run 1: {} messages", run1.len());
    println!("Run 2: {} messages", run2.len());

    assert_eq!(
        run1.len(),
        run2.len(),
        "Message counts differ: {} vs {}",
        run1.len(),
        run2.len()
    );
    assert_eq!(run1, run2, "CmdVel sequences differ between runs!");
    println!("STORY 5 PASSED ✓ (identical CmdVel across 2 runs)\n");
}

// ════════════════════════════════════════════════════════════════════════
// STORY 6: "Node panics don't kill the scheduler"
// ════════════════════════════════════════════════════════════════════════

struct PanickingNode {
    tick_count: u64,
}
impl Node for PanickingNode {
    fn name(&self) -> &str {
        "panicking_node"
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        if self.tick_count == 5 {
            panic!("intentional panic at tick 5");
        }
    }
}

#[test]
#[ignore]
fn story_node_panic_doesnt_kill_scheduler() {
    cleanup_stale_shm();

    let healthy_ticks = Arc::new(AtomicU64::new(0));
    let hc = healthy_ticks.clone();

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());

    let _ = sched.add(PanickingNode { tick_count: 0 }).order(0).build();

    let _ = sched.add(HealthyNode { tick_count: hc }).order(1).build();

    // Run 50 ticks — panicking node dies at tick 5, healthy continues
    for _ in 0..50 {
        let _ = sched.tick_once();
    }

    let ht = healthy_ticks.load(Ordering::Relaxed);
    println!("=== STORY: Node Panic Isolation ===");
    println!("Healthy node ticks after panic: {}", ht);

    assert!(
        ht >= 45,
        "Healthy node should keep running after panic, got {} ticks",
        ht
    );
    println!("STORY 6 PASSED ✓\n");
}

// ════════════════════════════════════════════════════════════════════════
// STORY 7: "Mixed execution classes in one scheduler"
// ════════════════════════════════════════════════════════════════════════
// RT node at 500Hz, Compute node, BestEffort node — all in one scheduler.

struct RtSensorNode {
    ticks: Arc<AtomicU64>,
}
impl Node for RtSensorNode {
    fn name(&self) -> &str {
        "rt_sensor"
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

struct ComputeVisionNode {
    ticks: Arc<AtomicU64>,
}
impl Node for ComputeVisionNode {
    fn name(&self) -> &str {
        "compute_vision"
    }
    fn tick(&mut self) {
        // Simulate 2ms compute
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(2) {
            std::hint::spin_loop();
        }
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

struct BestEffortLoggerNode {
    ticks: Arc<AtomicU64>,
}
impl Node for BestEffortLoggerNode {
    fn name(&self) -> &str {
        "logger"
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
#[ignore]
fn story_mixed_execution_classes() {
    cleanup_stale_shm();

    let rt_ticks = Arc::new(AtomicU64::new(0));
    let compute_ticks = Arc::new(AtomicU64::new(0));
    let be_ticks = Arc::new(AtomicU64::new(0));

    let rt = rt_ticks.clone();
    let cp = compute_ticks.clone();
    let be = be_ticks.clone();

    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(500_u64.hz());

        // RT sensor at 500Hz (auto-detected from rate)
        let _ = sched
            .add(RtSensorNode { ticks: rt })
            .rate(500_u64.hz())
            .order(0)
            .build();

        // Compute node (explicitly marked)
        let _ = sched
            .add(ComputeVisionNode { ticks: cp })
            .compute()
            .rate(30_u64.hz())
            .order(1)
            .build();

        // BestEffort logger
        let _ = sched
            .add(BestEffortLoggerNode { ticks: be })
            .rate(10_u64.hz())
            .order(2)
            .build();

        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_micros(1500));
        }
    });

    std::thread::sleep(Duration::from_secs(5));
    running.store(false, Ordering::Relaxed);
    handle.join().unwrap();

    let rt = rt_ticks.load(Ordering::Relaxed);
    let cp = compute_ticks.load(Ordering::Relaxed);
    let be = be_ticks.load(Ordering::Relaxed);

    println!("=== STORY: Mixed Execution Classes (5s) ===");
    println!(
        "RT sensor (500Hz):    {} ticks ({:.0} Hz)",
        rt,
        rt as f64 / 5.0
    );
    println!(
        "Compute vision (30Hz): {} ticks ({:.0} Hz)",
        cp,
        cp as f64 / 5.0
    );
    println!(
        "BestEffort log (10Hz): {} ticks ({:.0} Hz)",
        be,
        be as f64 / 5.0
    );

    assert!(rt > 100, "RT node should run >100 ticks");
    assert!(cp > 10, "Compute node should run >10 ticks");
    assert!(be > 5, "BestEffort node should run >5 ticks");
    // RT should run faster than compute
    assert!(rt > cp, "RT node should tick more than compute node");
    println!("STORY 7 PASSED ✓\n");
}
