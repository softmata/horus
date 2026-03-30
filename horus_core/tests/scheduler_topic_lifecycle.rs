// Scheduler + Topic + Node full lifecycle integration test.
//
// Proves the complete wired-together flow that every real user hits:
// Node creates Topics in init() → tick() sends/receives via Topics →
// shutdown() cleans up → scheduler exits cleanly.

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::error::HorusResult;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

fn unique(prefix: &str) -> String {
    static CTR: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        CTR.fetch_add(1, Ordering::Relaxed)
    )
}

// ---------------------------------------------------------------------------
// Sensor node: creates topic in init(), publishes in tick()
// ---------------------------------------------------------------------------
struct SensorNode {
    name: String,
    topic_name: String,
    topic: Option<Topic<u64>>,
    ticks: Arc<AtomicU64>,
    init_called: Arc<AtomicU64>,
    shutdown_called: Arc<AtomicU64>,
}

impl Node for SensorNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        self.init_called.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn tick(&mut self) {
        let t = self.ticks.fetch_add(1, Ordering::SeqCst);
        if let Some(ref topic) = self.topic {
            topic.send(t);
        }
    }

    fn shutdown(&mut self) -> HorusResult<()> {
        self.shutdown_called.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Controller node: receives from topic, publishes to another
// ---------------------------------------------------------------------------
struct ControllerNode {
    name: String,
    sub_topic_name: String,
    pub_topic_name: String,
    sub_topic: Option<Topic<u64>>,
    pub_topic: Option<Topic<u64>>,
    received: Arc<AtomicU64>,
}

impl Node for ControllerNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> HorusResult<()> {
        self.sub_topic = Some(Topic::new(&self.sub_topic_name)?);
        self.pub_topic = Some(Topic::new(&self.pub_topic_name)?);
        Ok(())
    }

    fn tick(&mut self) {
        if let Some(ref sub) = self.sub_topic {
            while let Some(val) = sub.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
                if let Some(ref pub_t) = self.pub_topic {
                    pub_t.send(val * 2);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Actuator node: receives commands
// ---------------------------------------------------------------------------
struct ActuatorNode {
    name: String,
    topic_name: String,
    topic: Option<Topic<u64>>,
    commands: Arc<AtomicU64>,
}

impl Node for ActuatorNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }

    fn tick(&mut self) {
        if let Some(ref topic) = self.topic {
            while let Some(_cmd) = topic.recv() {
                self.commands.fetch_add(1, Ordering::SeqCst);
            }
        }
    }
}

// ============================================================================
// Test: Full 3-node pipeline lifecycle (init → tick → shutdown)
// ============================================================================

#[test]
fn test_full_pipeline_lifecycle() {
    cleanup_stale_shm();

    let sensor_topic = unique("lc_sensor");
    let cmd_topic = unique("lc_cmd");

    let sensor_ticks = Arc::new(AtomicU64::new(0));
    let sensor_init = Arc::new(AtomicU64::new(0));
    let sensor_shutdown = Arc::new(AtomicU64::new(0));
    let ctrl_received = Arc::new(AtomicU64::new(0));
    let act_commands = Arc::new(AtomicU64::new(0));

    let sensor = SensorNode {
        name: format!("lc_sensor_{}", std::process::id()),
        topic_name: sensor_topic.clone(),
        topic: None,
        ticks: sensor_ticks.clone(),
        init_called: sensor_init.clone(),
        shutdown_called: sensor_shutdown.clone(),
    };

    let controller = ControllerNode {
        name: format!("lc_ctrl_{}", std::process::id()),
        sub_topic_name: sensor_topic,
        pub_topic_name: cmd_topic.clone(),
        sub_topic: None,
        pub_topic: None,
        received: ctrl_received.clone(),
    };

    let actuator = ActuatorNode {
        name: format!("lc_act_{}", std::process::id()),
        topic_name: cmd_topic,
        topic: None,
        commands: act_commands.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched.add(sensor).order(0).build().unwrap();
    sched.add(controller).order(1).build().unwrap();
    sched.add(actuator).order(2).build().unwrap();
    sched.run_for(Duration::from_millis(500)).unwrap();

    // Verify lifecycle
    assert_eq!(sensor_init.load(Ordering::SeqCst), 1, "init() called once");
    assert_eq!(
        sensor_shutdown.load(Ordering::SeqCst),
        1,
        "shutdown() called once"
    );

    // Verify data flow
    let st = sensor_ticks.load(Ordering::SeqCst);
    let cr = ctrl_received.load(Ordering::SeqCst);
    let ac = act_commands.load(Ordering::SeqCst);

    assert!(st > 5, "Sensor should tick many times, got {}", st);
    assert!(cr > 0, "Controller should receive sensor data, got {}", cr);
    assert!(ac > 0, "Actuator should receive commands, got {}", ac);
}

// ============================================================================
// Test: Topic created in init(), used in tick(), survives full lifecycle
// ============================================================================

#[test]
fn test_topic_survives_full_lifecycle() {
    cleanup_stale_shm();

    let topic_name = unique("lc_survive");
    let ticks = Arc::new(AtomicU64::new(0));
    let init_ok = Arc::new(AtomicU64::new(0));
    let shutdown_ok = Arc::new(AtomicU64::new(0));

    let node = SensorNode {
        name: format!("lc_survive_{}", std::process::id()),
        topic_name,
        topic: None,
        ticks: ticks.clone(),
        init_called: init_ok.clone(),
        shutdown_called: shutdown_ok.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(50_u64.hz());
    sched.add(node).order(0).build().unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    assert_eq!(init_ok.load(Ordering::SeqCst), 1);
    assert_eq!(shutdown_ok.load(Ordering::SeqCst), 1);
    assert!(
        ticks.load(Ordering::SeqCst) > 1,
        "Should tick multiple times"
    );
}
