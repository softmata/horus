//! Real Robot QA Test — 10-node robot using documented horus APIs.
//!
//! This test simulates a realistic robot application with:
//! - Lidar (publishes scan data)
//! - Camera (publishes image data)
//! - IMU (publishes orientation)
//! - SLAM (subscribes scan, publishes map)
//! - Planner (subscribes map, publishes path)
//! - Controller (subscribes path, publishes cmd_vel)
//! - Motor Left (subscribes cmd_vel)
//! - Motor Right (subscribes cmd_vel)
//! - Safety Monitor (subscribes cmd_vel, logs warnings)
//! - Battery (publishes voltage)
//!
//! All nodes use ONLY the documented API patterns from horus-docs:
//! - Topic::new(), topic.send(), topic.recv()
//! - hlog!() for logging
//! - Scheduler::new().tick_rate().add().order().build()
//!
//! Run: cargo test --no-default-features -p horus_core --test real_robot_qa -- --test-threads=1

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ── Sensor Nodes ────────────────────────────────────────────────────────

struct Lidar {
    scan_pub: Option<Topic<f32>>,
    count: Arc<AtomicU64>,
}

impl Node for Lidar {
    fn name(&self) -> &str { "lidar" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.scan_pub = Some(Topic::<f32>::new("scan")?);
        horus_core::hlog!(info, "Lidar initialized");
        Ok(())
    }
    fn tick(&mut self) {
        let val = self.count.fetch_add(1, Ordering::SeqCst) as f32;
        if let Some(ref t) = self.scan_pub {
            t.send(val * 0.1);
        }
    }
}

struct Camera {
    image_pub: Option<Topic<u64>>,
    count: Arc<AtomicU64>,
}

impl Node for Camera {
    fn name(&self) -> &str { "camera" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.image_pub = Some(Topic::<u64>::new("camera.rgb")?);
        horus_core::hlog!(info, "Camera initialized");
        Ok(())
    }
    fn tick(&mut self) {
        let frame = self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref t) = self.image_pub {
            t.send(frame);
        }
    }
}

struct Imu {
    imu_pub: Option<Topic<f32>>,
    count: Arc<AtomicU64>,
}

impl Node for Imu {
    fn name(&self) -> &str { "imu" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.imu_pub = Some(Topic::<f32>::new("imu.data")?);
        horus_core::hlog!(info, "IMU initialized");
        Ok(())
    }
    fn tick(&mut self) {
        let val = self.count.fetch_add(1, Ordering::SeqCst) as f32;
        if let Some(ref t) = self.imu_pub {
            t.send(val * 0.01); // orientation in radians
        }
    }
}

struct Battery {
    voltage_pub: Option<Topic<f32>>,
    voltage: f32,
}

impl Node for Battery {
    fn name(&self) -> &str { "battery" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.voltage_pub = Some(Topic::<f32>::new("battery.voltage")?);
        horus_core::hlog!(info, "Battery monitor initialized");
        Ok(())
    }
    fn tick(&mut self) {
        self.voltage -= 0.0001; // slowly draining
        if let Some(ref t) = self.voltage_pub {
            t.send(self.voltage);
        }
        if self.voltage < 11.0 {
            horus_core::hlog!(warn, "Battery low: {:.2}V", self.voltage);
        }
    }
}

// ── Processing Nodes ────────────────────────────────────────────────────

struct Slam {
    scan_sub: Option<Topic<f32>>,
    map_pub: Option<Topic<f32>>,
    count: Arc<AtomicU64>,
}

impl Node for Slam {
    fn name(&self) -> &str { "slam" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.scan_sub = Some(Topic::<f32>::new("scan")?);
        self.map_pub = Some(Topic::<f32>::new("map")?);
        horus_core::hlog!(info, "SLAM initialized");
        Ok(())
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref sub) = self.scan_sub {
            if let Some(scan) = sub.recv() {
                // Process scan → produce map
                let map_val = scan * 2.0;
                if let Some(ref pub_t) = self.map_pub {
                    pub_t.send(map_val);
                }
            }
        }
    }
}

struct Planner {
    map_sub: Option<Topic<f32>>,
    path_pub: Option<Topic<f32>>,
    count: Arc<AtomicU64>,
}

impl Node for Planner {
    fn name(&self) -> &str { "planner" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.map_sub = Some(Topic::<f32>::new("map")?);
        self.path_pub = Some(Topic::<f32>::new("path")?);
        horus_core::hlog!(info, "Path planner initialized");
        Ok(())
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref sub) = self.map_sub {
            if let Some(map) = sub.recv() {
                let path = map + 1.0;
                if let Some(ref pub_t) = self.path_pub {
                    pub_t.send(path);
                }
            }
        }
    }
}

struct Controller {
    path_sub: Option<Topic<f32>>,
    cmd_pub: Option<Topic<f32>>,
    count: Arc<AtomicU64>,
}

impl Node for Controller {
    fn name(&self) -> &str { "controller" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.path_sub = Some(Topic::<f32>::new("path")?);
        self.cmd_pub = Some(Topic::<f32>::new("cmd_vel")?);
        horus_core::hlog!(info, "Controller initialized");
        Ok(())
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref sub) = self.path_sub {
            if let Some(path) = sub.recv() {
                let cmd = path * 0.5;
                if let Some(ref pub_t) = self.cmd_pub {
                    pub_t.send(cmd);
                }
            }
        }
    }
}

// ── Actuator Nodes ──────────────────────────────────────────────────────

struct Motor {
    node_name: String,
    cmd_sub: Option<Topic<f32>>,
    count: Arc<AtomicU64>,
}

impl Node for Motor {
    fn name(&self) -> &str { &self.node_name }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.cmd_sub = Some(Topic::<f32>::new("cmd_vel")?);
        horus_core::hlog!(info, "{} initialized", self.node_name);
        Ok(())
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref sub) = self.cmd_sub {
            while let Some(_cmd) = sub.recv() {
                // Apply motor command
            }
        }
    }
}

struct SafetyMonitor {
    cmd_sub: Option<Topic<f32>>,
    count: Arc<AtomicU64>,
}

impl Node for SafetyMonitor {
    fn name(&self) -> &str { "safety_monitor" }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.cmd_sub = Some(Topic::<f32>::new("cmd_vel")?);
        horus_core::hlog!(info, "Safety monitor initialized");
        Ok(())
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref sub) = self.cmd_sub {
            if let Some(cmd) = sub.recv() {
                if cmd.abs() > 5.0 {
                    horus_core::hlog!(warn, "High velocity command: {:.2}", cmd);
                }
            }
        }
    }
}

// ── Test ─────────────────────────────────────────────────────────────────

#[test]
fn test_10_node_robot_runs() {
    cleanup_stale_shm();

    let counts: Vec<Arc<AtomicU64>> = (0..10).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut sched = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(10000);

    // Sensors (order 0-3)
    sched.add(Lidar { scan_pub: None, count: counts[0].clone() })
        .order(0).build().unwrap();
    sched.add(Camera { image_pub: None, count: counts[1].clone() })
        .order(1).build().unwrap();
    sched.add(Imu { imu_pub: None, count: counts[2].clone() })
        .order(2).build().unwrap();
    sched.add(Battery { voltage_pub: None, voltage: 12.6 })
        .order(3).build().unwrap();

    // Processing (order 10-12)
    sched.add(Slam { scan_sub: None, map_pub: None, count: counts[4].clone() })
        .order(10).build().unwrap();
    sched.add(Planner { map_sub: None, path_pub: None, count: counts[5].clone() })
        .order(11).build().unwrap();
    sched.add(Controller { path_sub: None, cmd_pub: None, count: counts[6].clone() })
        .order(12).build().unwrap();

    // Actuators (order 20-22)
    sched.add(Motor { node_name: "motor_left".into(), cmd_sub: None, count: counts[7].clone() })
        .order(20).build().unwrap();
    sched.add(Motor { node_name: "motor_right".into(), cmd_sub: None, count: counts[8].clone() })
        .order(21).build().unwrap();
    sched.add(SafetyMonitor { cmd_sub: None, count: counts[9].clone() })
        .order(22).build().unwrap();

    // Run for 3 seconds
    sched.run_for(3_u64.secs()).unwrap();

    // Verify all 10 nodes ticked
    let names = ["lidar", "camera", "imu", "battery", "slam", "planner",
                  "controller", "motor_left", "motor_right", "safety_monitor"];
    for (i, name) in names.iter().enumerate() {
        if i == 3 { continue; } // battery uses its own counter
        let ticks = counts[i].load(Ordering::SeqCst);
        assert!(ticks > 50, "Node '{}' should have ticked >50 times, got {}", name, ticks);
    }

    // Verify TopicNodeRegistry was populated
    let registry = horus_core::communication::topic_node_registry();
    let all = registry.all_topics();
    println!("=== TopicNodeRegistry ===");
    for (topic, assocs) in &all {
        let nodes: Vec<_> = assocs.iter().map(|a| format!("{} ({:?})", a.node_name, a.role)).collect();
        println!("  {} → {}", topic, nodes.join(", "));
    }

    // Check specific associations
    let scan_pubs = registry.publishers_of_topic("scan");
    assert!(scan_pubs.contains(&"lidar".to_string()),
        "scan publishers should include lidar, got: {:?}", scan_pubs);

    let cmd_subs = registry.subscribers_of_topic("cmd_vel");
    assert!(!cmd_subs.is_empty(),
        "cmd_vel should have subscribers: {:?}", cmd_subs);

    // Verify presence files exist
    let presences = horus_core::NodePresence::read_all();
    println!("=== Presence files ===");
    for p in &presences {
        println!("  {} — pubs: {:?}, subs: {:?}, ticks: {}",
            p.name(),
            p.publishers().iter().map(|t| &t.topic_name).collect::<Vec<_>>(),
            p.subscribers().iter().map(|t| &t.topic_name).collect::<Vec<_>>(),
            p.tick_count()
        );
    }

    // Presence files are cleaned up on shutdown — check TopicNodeRegistry instead.
    // (During runtime, CLI reads presence files which are written FROM the registry.
    //  After shutdown, presence files are deleted but the registry persists in-process.)
    let all_topics = registry.all_topics();
    assert!(
        all_topics.len() >= 6,
        "TopicNodeRegistry should have at least 6 topics (scan, map, path, cmd_vel, camera.rgb, imu.data, battery.voltage), got {}",
        all_topics.len()
    );

    // Verify correct pub/sub roles
    let scan_subs = registry.subscribers_of_topic("scan");
    assert!(scan_subs.contains(&"slam".to_string()), "slam should subscribe to scan");

    let map_pubs = registry.publishers_of_topic("map");
    assert!(map_pubs.contains(&"slam".to_string()), "slam should publish map");

    let cmd_pubs = registry.publishers_of_topic("cmd_vel");
    assert!(cmd_pubs.contains(&"controller".to_string()), "controller should publish cmd_vel");

    let cmd_subs_list = registry.subscribers_of_topic("cmd_vel");
    assert!(cmd_subs_list.len() >= 3,
        "cmd_vel should have 3+ subscribers (motor_left, motor_right, safety_monitor), got {:?}", cmd_subs_list);
}
