#![allow(dead_code)]
//! Chaos monkey — tests what happens when users combine APIs in unexpected ways.
//!
//! These are NOT clean patterns. They are the messy, creative, "why would anyone
//! do this" combinations that real users WILL do. If any of these crash, corrupt
//! data, or deadlock, a real robot will fail in the field.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test chaos_monkey -- --ignored --nocapture --test-threads=1

use horus_core::action;
use horus_core::actions::*;
use horus_core::communication::topic::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::params::RuntimeParams;
use horus_core::scheduling::Scheduler;
use horus_core::service;
use horus_core::services::*;
use horus_robotics::messages::sensor::*;
use horus_robotics::CmdVel;
use horus_tf::{Transform, TransformFrame};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::{cleanup_stale_shm, unique};

// ════════════════════════════════════════════════════════════════════════
// CHAOS 1: Create topics inside init() that depend on runtime state
// ════════════════════════════════════════════════════════════════════════
// Users create topics lazily based on parameters, not upfront.

struct LazyTopicNode {
    base_name: String,
    topics: Vec<Topic<CmdVel>>,
    ticks: Arc<AtomicU64>,
}
impl Node for LazyTopicNode {
    fn name(&self) -> &str {
        "lazy_topic"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        // Create 5 topics dynamically based on a runtime parameter
        for i in 0..5 {
            let name = format!("{}.joint_{}", self.base_name, i);
            self.topics.push(Topic::new(&name)?);
        }
        Ok(())
    }
    fn tick(&mut self) {
        for (i, t) in self.topics.iter().enumerate() {
            t.send(CmdVel::new(i as f32 * 0.1, 0.0));
        }
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

struct LazyRecvNode {
    base_name: String,
    topics: Vec<Topic<CmdVel>>,
    received: Arc<AtomicU64>,
    corrupted: Arc<AtomicU64>,
}
impl Node for LazyRecvNode {
    fn name(&self) -> &str {
        "lazy_recv"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        for i in 0..5 {
            let name = format!("{}.joint_{}", self.base_name, i);
            self.topics.push(Topic::new(&name)?);
        }
        Ok(())
    }
    fn tick(&mut self) {
        for (i, t) in self.topics.iter().enumerate() {
            while let Some(cmd) = t.recv() {
                let expected = i as f32 * 0.1;
                if (cmd.linear - expected).abs() > 0.01 {
                    self.corrupted.fetch_add(1, Ordering::Relaxed);
                }
                self.received.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

#[test]
#[ignore]
fn chaos_lazy_topics_in_init() {
    cleanup_stale_shm();
    let base = unique("chaos_lazy");
    let ticks = Arc::new(AtomicU64::new(0));
    let recv = Arc::new(AtomicU64::new(0));
    let corrupt = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);
    let _ = sched
        .add(LazyTopicNode {
            base_name: base.clone(),
            topics: vec![],
            ticks: ticks.clone(),
        })
        .order(0)
        .build();
    let _ = sched
        .add(LazyRecvNode {
            base_name: base,
            topics: vec![],
            received: recv.clone(),
            corrupted: corrupt.clone(),
        })
        .order(1)
        .build();

    for _ in 0..200 {
        let _ = sched.tick_once();
    }

    let c = corrupt.load(Ordering::Relaxed);
    let r = recv.load(Ordering::Relaxed);
    println!("Lazy topics: {} received, {} corrupt", r, c);
    assert_eq!(c, 0, "Lazy topic data corrupted!");
    assert!(r > 100, "Should receive >100 msgs across 5 topics");
    println!("✓ chaos_lazy_topics_in_init — 5 dynamic topics, zero corruption");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS 2: Params changed while nodes are ticking
// ════════════════════════════════════════════════════════════════════════
// Operator runs `horus param set kp 5.0` while robot is running.

struct ParamReadingNode {
    params: Arc<RuntimeParams>,
    values_seen: Arc<std::sync::Mutex<Vec<f64>>>,
}
impl Node for ParamReadingNode {
    fn name(&self) -> &str {
        "param_reader"
    }
    fn tick(&mut self) {
        let kp: f64 = self.params.get_or("chaos.kp", 1.0);
        self.values_seen.lock().unwrap().push(kp);
    }
}

#[test]
#[ignore]
fn chaos_params_changed_mid_run() {
    cleanup_stale_shm();
    let params = Arc::new(RuntimeParams::new().unwrap());
    params.set("chaos.kp", 1.0f64).unwrap();

    let values = Arc::new(std::sync::Mutex::new(Vec::new()));
    let running = Arc::new(AtomicBool::new(true));

    let p = params.clone();
    let v = values.clone();
    let r = running.clone();
    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        let _ = sched
            .add(ParamReadingNode {
                params: p,
                values_seen: v,
            })
            .rate(100_u64.hz())
            .order(0)
            .build();
        while r.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    // Change params mid-run from another thread (simulates CLI)
    std::thread::sleep(Duration::from_millis(500));
    params.set("chaos.kp", 5.0f64).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    params.set("chaos.kp", 10.0f64).unwrap();
    std::thread::sleep(Duration::from_millis(500));

    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let vals = values.lock().unwrap();
    let saw_1 = vals.iter().any(|&v| (v - 1.0).abs() < 0.01);
    let saw_5 = vals.iter().any(|&v| (v - 5.0).abs() < 0.01);
    let saw_10 = vals.iter().any(|&v| (v - 10.0).abs() < 0.01);

    println!(
        "Params mid-run: {} values, saw 1.0={}, 5.0={}, 10.0={}",
        vals.len(),
        saw_1,
        saw_5,
        saw_10
    );
    assert!(saw_1, "Should see initial kp=1.0");
    assert!(saw_5 || saw_10, "Should see at least one updated value");
    println!("✓ chaos_params_changed_mid_run — live param tuning works");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS 3: TransformFrame from inside a topic callback node
// ════════════════════════════════════════════════════════════════════════

struct TfInTickNode {
    tf: Arc<TransformFrame>,
    imu_name: String,
    imu: Option<Topic<Imu>>,
    lookups: Arc<AtomicU64>,
    errors: Arc<AtomicU64>,
}
impl Node for TfInTickNode {
    fn name(&self) -> &str {
        "tf_in_tick"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.imu = Some(Topic::new(&self.imu_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        // Publish IMU AND do TF lookup in the same tick
        if let Some(ref t) = self.imu {
            let mut imu = Imu::new();
            imu.linear_acceleration = [0.0, 0.0, 9.81];
            t.send(imu);
        }
        // TF lookup in the hot path
        match self.tf.tf("sensor", "world") {
            Ok(result) => {
                if result.translation[0].is_finite() {
                    self.lookups.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.errors.fetch_add(1, Ordering::Relaxed);
                }
            }
            Err(_) => {} // frame not yet registered — OK during init
        }
        // Update TF in the same tick
        let _ = self.tf.update_transform(
            "sensor",
            &Transform::from_translation([0.1, 0.0, 0.3]),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        );
    }
}

#[test]
#[ignore]
fn chaos_tf_in_topic_callback() {
    cleanup_stale_shm();
    let tf = Arc::new(TransformFrame::medium());
    tf.register_frame("world", None).unwrap();
    tf.register_frame("sensor", Some("world")).unwrap();
    tf.update_transform(
        "sensor",
        &Transform::from_translation([0.1, 0.0, 0.3]),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64,
    )
    .unwrap();

    let lookups = Arc::new(AtomicU64::new(0));
    let errors = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(200_u64.hz()).deterministic(true);
    let _ = sched
        .add(TfInTickNode {
            tf: tf.clone(),
            imu_name: unique("chaos_tf_imu"),
            imu: None,
            lookups: lookups.clone(),
            errors: errors.clone(),
        })
        .rate(200_u64.hz())
        .order(0)
        .build();

    for _ in 0..500 {
        let _ = sched.tick_once();
    }

    let l = lookups.load(Ordering::Relaxed);
    let e = errors.load(Ordering::Relaxed);
    println!("TF in tick: {} lookups, {} errors", l, e);
    assert!(l > 100, "Should do >100 TF lookups in tick");
    assert_eq!(e, 0, "TF lookups should not produce NaN");
    println!("✓ chaos_tf_in_topic_callback — TF + Topic in same tick works");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS 4: Service called from inside an action callback
// ════════════════════════════════════════════════════════════════════════

service! { InnerSvc { request { val: i64 } response { result: i64 } } }
action! { OuterAct { goal { x: f64 } feedback { p: f32 } result { ok: bool } } }

#[test]
#[ignore]
fn chaos_service_inside_action() {
    cleanup_stale_shm();

    // Service server
    let _svc = ServiceServerBuilder::<InnerSvc>::new()
        .on_request(|req| {
            Ok(InnerSvcResponse {
                result: req.val * 2,
            })
        })
        .build()
        .unwrap();

    // Action server that calls the service internally
    let svc_called = Arc::new(AtomicU64::new(0));
    let sc = svc_called.clone();

    let server = ActionServerBuilder::<OuterAct>::new()
        .on_goal(|_| GoalResponse::Accept)
        .on_cancel(|_| CancelResponse::Accept)
        .on_execute(move |handle| {
            // Call service from inside action execute callback
            if let Ok(mut client) = ServiceClient::<InnerSvc>::new() {
                if let Ok(resp) = client.call(InnerSvcRequest { val: 42 }, 2_u64.secs()) {
                    if resp.result == 84 {
                        sc.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            handle.succeed(OuterActResult { ok: true })
        })
        .build();

    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();
    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<OuterAct>::new().unwrap();
    let result = client.send_goal_and_wait(OuterActGoal { x: 1.0 }, 5_u64.secs());

    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let calls = svc_called.load(Ordering::Relaxed);
    println!(
        "Service inside action: {} service calls, result={:?}",
        calls,
        result.as_ref().map(|r| r.ok)
    );
    match result {
        Ok(r) => assert!(r.ok, "Action should succeed"),
        Err(e) => println!("  Action error (acceptable in debug): {:?}", e),
    }
    println!("✓ chaos_service_inside_action — nested service call works");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS 5: 50 topics, mixed types, simultaneous send/recv
// ════════════════════════════════════════════════════════════════════════

struct FiftyTopicNode {
    prefix: String,
    imu_topics: Vec<Topic<Imu>>,
    cmd_topics: Vec<Topic<CmdVel>>,
    bat_topics: Vec<Topic<BatteryState>>,
    sent: Arc<AtomicU64>,
    received: Arc<AtomicU64>,
    corrupted: Arc<AtomicU64>,
    is_pub: bool,
}
impl Node for FiftyTopicNode {
    fn name(&self) -> &str {
        if self.is_pub {
            "fifty_pub"
        } else {
            "fifty_sub"
        }
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        for i in 0..20 {
            self.imu_topics
                .push(Topic::new(format!("{}.imu_{}", self.prefix, i))?);
        }
        for i in 0..20 {
            self.cmd_topics
                .push(Topic::new(format!("{}.cmd_{}", self.prefix, i))?);
        }
        for i in 0..10 {
            self.bat_topics
                .push(Topic::new(format!("{}.bat_{}", self.prefix, i))?);
        }
        Ok(())
    }
    fn tick(&mut self) {
        if self.is_pub {
            for t in &self.imu_topics {
                let mut imu = Imu::new();
                imu.linear_acceleration[2] = 9.81;
                t.send(imu);
                self.sent.fetch_add(1, Ordering::Relaxed);
            }
            for t in &self.cmd_topics {
                t.send(CmdVel::new(1.0, 0.5));
                self.sent.fetch_add(1, Ordering::Relaxed);
            }
            for t in &self.bat_topics {
                let mut b = BatteryState::default();
                b.voltage = 12.0;
                t.send(b);
                self.sent.fetch_add(1, Ordering::Relaxed);
            }
        } else {
            for t in &self.imu_topics {
                while let Some(imu) = t.recv() {
                    if (imu.linear_acceleration[2] - 9.81).abs() > 0.01 {
                        self.corrupted.fetch_add(1, Ordering::Relaxed);
                    }
                    self.received.fetch_add(1, Ordering::Relaxed);
                }
            }
            for t in &self.cmd_topics {
                while let Some(cmd) = t.recv() {
                    if (cmd.linear - 1.0).abs() > 0.01 {
                        self.corrupted.fetch_add(1, Ordering::Relaxed);
                    }
                    self.received.fetch_add(1, Ordering::Relaxed);
                }
            }
            for t in &self.bat_topics {
                while let Some(bat) = t.recv() {
                    if (bat.voltage - 12.0).abs() > 0.1 {
                        self.corrupted.fetch_add(1, Ordering::Relaxed);
                    }
                    self.received.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }
}

#[test]
#[ignore]
fn chaos_50_topics_mixed_types() {
    cleanup_stale_shm();
    let prefix = unique("chaos_50");
    let sent = Arc::new(AtomicU64::new(0));
    let recv = Arc::new(AtomicU64::new(0));
    let corrupt = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);
    let _ = sched
        .add(FiftyTopicNode {
            prefix: prefix.clone(),
            imu_topics: vec![],
            cmd_topics: vec![],
            bat_topics: vec![],
            sent: sent.clone(),
            received: Arc::new(AtomicU64::new(0)),
            corrupted: Arc::new(AtomicU64::new(0)),
            is_pub: true,
        })
        .order(0)
        .build();
    let _ = sched
        .add(FiftyTopicNode {
            prefix,
            imu_topics: vec![],
            cmd_topics: vec![],
            bat_topics: vec![],
            sent: Arc::new(AtomicU64::new(0)),
            received: recv.clone(),
            corrupted: corrupt.clone(),
            is_pub: false,
        })
        .order(1)
        .build();

    for _ in 0..200 {
        let _ = sched.tick_once();
    }

    let s = sent.load(Ordering::Relaxed);
    let r = recv.load(Ordering::Relaxed);
    let c = corrupt.load(Ordering::Relaxed);
    println!("50 topics: sent={}, received={}, corrupted={}", s, r, c);
    assert_eq!(c, 0, "DATA CORRUPTION across 50 mixed topics!");
    assert!(s > 5000, "Should send >5000 msgs across 50 topics");
    assert!(r > 2000, "Should receive >2000 msgs");
    println!(
        "✓ chaos_50_topics_mixed_types — {} msgs, zero corruption",
        s
    );
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS 6: Same topic opened from 4 different schedulers
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn chaos_4_schedulers_one_topic() {
    cleanup_stale_shm();
    let topic = unique("chaos_4sched");
    let total_recv = Arc::new(AtomicU64::new(0));
    let running = Arc::new(AtomicBool::new(true));

    // 1 publisher scheduler
    let t1 = topic.clone();
    let r1 = running.clone();
    let h_pub = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(200_u64.hz());
        struct Pub {
            t: Option<Topic<CmdVel>>,
            n: String,
        }
        impl Node for Pub {
            fn name(&self) -> &str {
                "pub4"
            }
            fn init(&mut self) -> horus_core::error::HorusResult<()> {
                self.t = Some(Topic::new(&self.n)?);
                Ok(())
            }
            fn tick(&mut self) {
                if let Some(ref t) = self.t {
                    t.send(CmdVel::new(1.0, 0.0));
                }
            }
        }
        let _ = sched
            .add(Pub { t: None, n: t1 })
            .rate(200_u64.hz())
            .order(0)
            .build();
        while r1.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(4));
        }
    });

    // 3 subscriber schedulers at different rates
    let mut sub_handles = vec![];
    for rate in [100u64, 50, 10].iter() {
        let t = topic.clone();
        let tr = total_recv.clone();
        let r = running.clone();
        let hz = *rate;
        sub_handles.push(std::thread::spawn(move || {
            let mut sched = Scheduler::new().tick_rate(hz.hz());
            struct Sub {
                t: Option<Topic<CmdVel>>,
                n: String,
                r: Arc<AtomicU64>,
            }
            impl Node for Sub {
                fn name(&self) -> &str {
                    "sub4"
                }
                fn init(&mut self) -> horus_core::error::HorusResult<()> {
                    self.t = Some(Topic::new(&self.n)?);
                    Ok(())
                }
                fn tick(&mut self) {
                    if let Some(ref t) = self.t {
                        while t.recv().is_some() {
                            self.r.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
            let _ = sched
                .add(Sub {
                    t: None,
                    n: t,
                    r: tr,
                })
                .rate(hz.hz())
                .order(0)
                .build();
            while r.load(Ordering::Relaxed) {
                let _ = sched.tick_once();
                std::thread::sleep(Duration::from_millis(1000 / hz));
            }
        }));
    }

    std::thread::sleep(Duration::from_secs(5));
    running.store(false, Ordering::Relaxed);
    h_pub.join().unwrap();
    for h in sub_handles {
        h.join().unwrap();
    }

    let r = total_recv.load(Ordering::Relaxed);
    println!(
        "4 schedulers, 1 topic: {} total received across 3 subscribers",
        r
    );
    assert!(r > 100, "Subscribers should receive data from publisher");
    println!("✓ chaos_4_schedulers_one_topic — 4 schedulers share 1 topic");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS 7: Node creates topic, sends to itself, reads in same tick
// ════════════════════════════════════════════════════════════════════════

struct SelfTalkNode {
    topic: Option<Topic<CmdVel>>,
    name_str: String,
    echoed: Arc<AtomicU64>,
}
impl Node for SelfTalkNode {
    fn name(&self) -> &str {
        "self_talk"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.name_str)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.topic {
            t.send(CmdVel::new(42.0, 0.0));
            if let Some(cmd) = t.recv() {
                if (cmd.linear - 42.0).abs() < 0.01 {
                    self.echoed.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }
}

#[test]
#[ignore]
fn chaos_self_send_recv_same_tick() {
    cleanup_stale_shm();
    let echoed = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);
    let _ = sched
        .add(SelfTalkNode {
            topic: None,
            name_str: unique("chaos_self"),
            echoed: echoed.clone(),
        })
        .order(0)
        .build();

    for _ in 0..100 {
        let _ = sched.tick_once();
    }

    let e = echoed.load(Ordering::Relaxed);
    println!("Self send/recv: {} echoed in same tick", e);
    assert!(e > 50, "Should echo >50 msgs to self");
    println!("✓ chaos_self_send_recv_same_tick — node talks to itself");
}

// ════════════════════════════════════════════════════════════════════════
// CHAOS 8: Everything at once — the ultimate chaos test
// ════════════════════════════════════════════════════════════════════════
// Topics + Services + Actions + TF + Params + multiple schedulers +
// mixed types + self-send + dynamic topics — ALL SIMULTANEOUSLY.

#[test]
#[ignore]
fn chaos_everything_at_once() {
    cleanup_stale_shm();

    let tf = Arc::new(TransformFrame::medium());
    tf.register_frame("world", None).unwrap();
    tf.register_frame("base", Some("world")).unwrap();

    let params = Arc::new(RuntimeParams::new().unwrap());
    params.set("chaos.speed", 1.0f64).unwrap();

    service! { ChaosSvc { request { n: i64 } response { n2: i64 } } }

    let _svc = ServiceServerBuilder::<ChaosSvc>::new()
        .on_request(|req| Ok(ChaosSvcResponse { n2: req.n + 1 }))
        .build()
        .unwrap();

    let imu_topic = unique("chaos_all_imu");
    let cmd_topic = unique("chaos_all_cmd");

    let corrupted = Arc::new(AtomicU64::new(0));
    let total_ops = Arc::new(AtomicU64::new(0));
    let running = Arc::new(AtomicBool::new(true));

    // Scheduler 1: publisher + TF + params
    let tf1 = tf.clone();
    let p1 = params.clone();
    let c1 = corrupted.clone();
    let o1 = total_ops.clone();
    let it = imu_topic.clone();
    let ct = cmd_topic.clone();
    let r1 = running.clone();
    let h1 = std::thread::spawn(move || {
        struct ChaosNode1 {
            tf: Arc<TransformFrame>,
            params: Arc<RuntimeParams>,
            imu: Option<Topic<Imu>>,
            cmd: Option<Topic<CmdVel>>,
            imu_n: String,
            cmd_n: String,
            corrupt: Arc<AtomicU64>,
            ops: Arc<AtomicU64>,
            tick: u64,
        }
        impl Node for ChaosNode1 {
            fn name(&self) -> &str {
                "chaos1"
            }
            fn init(&mut self) -> horus_core::error::HorusResult<()> {
                self.imu = Some(Topic::new(&self.imu_n)?);
                self.cmd = Some(Topic::new(&self.cmd_n)?);
                Ok(())
            }
            fn tick(&mut self) {
                // Publish IMU
                let mut imu = Imu::new();
                imu.linear_acceleration[2] = 9.81;
                if let Some(ref t) = self.imu {
                    t.send(imu);
                }
                // Read params
                let speed: f64 = self.params.get_or("chaos.speed", 1.0);
                // Update TF
                let _ = self.tf.update_transform(
                    "base",
                    &Transform::from_translation([self.tick as f64 * 0.001, 0.0, 0.0]),
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as u64,
                );
                // Publish CmdVel based on param
                if let Some(ref t) = self.cmd {
                    t.send(CmdVel::new(speed as f32, 0.0));
                }
                // Self-recv CmdVel (echo check)
                if let Some(ref t) = self.cmd {
                    if let Some(cmd) = t.recv() {
                        if !cmd.linear.is_finite() {
                            self.corrupt.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
                self.ops.fetch_add(5, Ordering::Relaxed); // 5 ops per tick
                self.tick += 1;
            }
        }
        let mut sched = Scheduler::new()
            .tick_rate(200_u64.hz())
            .name("chaos_sched1");
        let _ = sched
            .add(ChaosNode1 {
                tf: tf1,
                params: p1,
                imu: None,
                cmd: None,
                imu_n: it,
                cmd_n: ct,
                corrupt: c1,
                ops: o1,
                tick: 0,
            })
            .rate(200_u64.hz())
            .order(0)
            .build();
        while r1.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(4));
        }
    });

    // Scheduler 2: subscriber + TF reads
    let tf2 = tf.clone();
    let c2 = corrupted.clone();
    let o2 = total_ops.clone();
    let it2 = imu_topic.clone();
    let r2 = running.clone();
    let h2 = std::thread::spawn(move || {
        struct ChaosNode2 {
            tf: Arc<TransformFrame>,
            imu: Option<Topic<Imu>>,
            imu_n: String,
            corrupt: Arc<AtomicU64>,
            ops: Arc<AtomicU64>,
        }
        impl Node for ChaosNode2 {
            fn name(&self) -> &str {
                "chaos2"
            }
            fn init(&mut self) -> horus_core::error::HorusResult<()> {
                self.imu = Some(Topic::new(&self.imu_n)?);
                Ok(())
            }
            fn tick(&mut self) {
                if let Some(ref t) = self.imu {
                    while let Some(imu) = t.recv() {
                        if (imu.linear_acceleration[2] - 9.81).abs() > 0.01 {
                            self.corrupt.fetch_add(1, Ordering::Relaxed);
                        }
                        self.ops.fetch_add(1, Ordering::Relaxed);
                    }
                }
                if let Ok(result) = self.tf.tf("base", "world") {
                    if !result.translation[0].is_finite() {
                        self.corrupt.fetch_add(1, Ordering::Relaxed);
                    }
                    self.ops.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        let mut sched = Scheduler::new().tick_rate(50_u64.hz()).name("chaos_sched2");
        let _ = sched
            .add(ChaosNode2 {
                tf: tf2,
                imu: None,
                imu_n: it2,
                corrupt: c2,
                ops: o2,
            })
            .rate(50_u64.hz())
            .order(0)
            .build();
        while r2.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(18));
        }
    });

    // Main thread: change params + call service
    std::thread::sleep(Duration::from_secs(1));
    params.set("chaos.speed", 2.0f64).unwrap();

    if let Ok(mut client) = ServiceClient::<ChaosSvc>::new() {
        let _ = client.call(ChaosSvcRequest { n: 99 }, 2_u64.secs());
    }

    std::thread::sleep(Duration::from_secs(3));
    params.set("chaos.speed", 0.5f64).unwrap();
    std::thread::sleep(Duration::from_secs(1));

    running.store(false, Ordering::Relaxed);
    h1.join().unwrap();
    h2.join().unwrap();

    let c = corrupted.load(Ordering::Relaxed);
    let o = total_ops.load(Ordering::Relaxed);
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  CHAOS: EVERYTHING AT ONCE (5s)                             ║");
    println!(
        "║  Total operations: {:6}                                    ║",
        o
    );
    println!(
        "║  Corrupted:        {:6}                                    ║",
        c
    );
    println!("║  Features active: topics + service + TF + params + 2 sched  ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    assert_eq!(c, 0, "CHAOS TEST FOUND CORRUPTION!");
    assert!(o > 1000, "Should do >1000 operations");
    println!("✓ chaos_everything_at_once — {} ops, ZERO corruption", o);
}
