// Test comprehensive scheduler configuration
use horus_core::error::Result;
use horus_core::hlog;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::{DurationExt, Node};

/// Test node with an Arc<AtomicU64> counter so the caller can read
/// tick_count after the node has been moved into the scheduler.
struct TestNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl TestNode {
    fn new(name: &str) -> (Self, Arc<AtomicU64>) {
        let tick_count = Arc::new(AtomicU64::new(0));
        let handle = Arc::clone(&tick_count);
        (
            Self {
                name: name.to_string(),
                tick_count,
            },
            handle,
        )
    }
}

impl Node for TestNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        hlog!(info, "{} initialized", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
    }

    fn shutdown(&mut self) -> Result<()> {
        hlog!(
            info,
            "{} shutdown after {} ticks",
            self.name,
            self.tick_count.load(Ordering::Relaxed)
        );
        Ok(())
    }
}

#[test]
fn test_standard_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();

    let (sensor, sensor_ticks) = TestNode::new("sensor1");
    let (controller, ctrl_ticks) = TestNode::new("controller1");

    scheduler.add(sensor).order(0).build().unwrap();
    scheduler.add(controller).order(1).build().unwrap();

    scheduler.run_for(100_u64.ms()).unwrap();

    // Both nodes must have been ticked
    assert!(
        sensor_ticks.load(Ordering::Relaxed) > 0,
        "sensor1 was never ticked"
    );
    assert!(
        ctrl_ticks.load(Ordering::Relaxed) > 0,
        "controller1 was never ticked"
    );

    let nodes = scheduler.node_list();
    assert!(nodes.contains(&"sensor1".to_string()));
    assert!(nodes.contains(&"controller1".to_string()));
}

#[test]
fn test_safety_critical_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());

    let (monitor_node, monitor_ticks) = TestNode::new("safety_monitor");
    let (estop_node, estop_ticks) = TestNode::new("emergency_stop");

    scheduler.add(monitor_node).order(0).build().unwrap();
    scheduler.add(estop_node).order(0).build().unwrap();

    scheduler.run_for(50_u64.ms()).unwrap();

    // At 1kHz for 50ms we expect ~50 ticks; be conservative
    assert!(
        monitor_ticks.load(Ordering::Relaxed) >= 5,
        "safety_monitor tick count too low: {}",
        monitor_ticks.load(Ordering::Relaxed)
    );
    assert!(
        estop_ticks.load(Ordering::Relaxed) >= 5,
        "emergency_stop tick count too low: {}",
        estop_ticks.load(Ordering::Relaxed)
    );
}

#[test]
fn test_high_performance_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().tick_rate(10000_u64.hz());

    let (fast_sensor, fast_sensor_ticks) = TestNode::new("fast_sensor");
    let (fast_control, fast_ctrl_ticks) = TestNode::new("fast_control");

    scheduler.add(fast_sensor).order(0).build().unwrap();
    scheduler.add(fast_control).order(1).build().unwrap();

    scheduler.run_for(50_u64.ms()).unwrap();

    // At 10kHz for 50ms we expect ~500 ticks; CI can be slow, so be lenient
    assert!(
        fast_sensor_ticks.load(Ordering::Relaxed) >= 10,
        "fast_sensor tick count too low: {}",
        fast_sensor_ticks.load(Ordering::Relaxed)
    );
    assert!(
        fast_ctrl_ticks.load(Ordering::Relaxed) >= 10,
        "fast_control tick count too low: {}",
        fast_ctrl_ticks.load(Ordering::Relaxed)
    );
}

#[test]
fn test_space_robot_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();

    let (nav_node, nav_ticks) = TestNode::new("navigation");
    let (solar_node, solar_ticks) = TestNode::new("solar_panel");

    scheduler.add(nav_node).order(0).build().unwrap();
    scheduler.add(solar_node).order(5).build().unwrap();

    scheduler.run_for(100_u64.ms()).unwrap();

    assert!(
        nav_ticks.load(Ordering::Relaxed) > 0,
        "navigation was never ticked"
    );
    assert!(
        solar_ticks.load(Ordering::Relaxed) > 0,
        "solar_panel was never ticked"
    );

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 2);
}

#[test]
fn test_custom_exotic_robot_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(500_u64.hz());

    let (bio_node, bio_ticks) = TestNode::new("bio_sensor");
    let (quantum_node, quantum_ticks) = TestNode::new("quantum_controller");

    scheduler.add(bio_node).order(0).build().unwrap();
    scheduler.add(quantum_node).order(1).build().unwrap();

    scheduler.run_for(100_u64.ms()).unwrap();

    // At 500Hz for 100ms we expect ~50 ticks
    assert!(
        bio_ticks.load(Ordering::Relaxed) >= 5,
        "bio_sensor tick count too low: {}",
        bio_ticks.load(Ordering::Relaxed)
    );
    assert!(
        quantum_ticks.load(Ordering::Relaxed) >= 5,
        "quantum_controller tick count too low: {}",
        quantum_ticks.load(Ordering::Relaxed)
    );
}

#[test]
fn test_execution_classes() {
    cleanup_stale_shm();
    // Test BestEffort nodes (main thread)
    {
        let mut scheduler = Scheduler::new();

        let (seq_node, seq_ticks) = TestNode::new("seq_node");
        scheduler.add(seq_node).order(0).build().unwrap();

        scheduler.run_for(50_u64.ms()).unwrap();

        assert!(
            seq_ticks.load(Ordering::Relaxed) > 0,
            "seq_node was never ticked"
        );
        assert_eq!(scheduler.node_list(), vec!["seq_node".to_string()]);
    }

    // Test mixed execution classes: compute + best-effort
    {
        let mut scheduler = Scheduler::new();

        let (compute_node, compute_ticks) = TestNode::new("compute_node");
        let (main_node, main_ticks) = TestNode::new("main_node");

        scheduler
            .add(compute_node)
            .order(0)
            .compute()
            .build()
            .unwrap();
        scheduler.add(main_node).order(1).build().unwrap();

        scheduler.run_for(50_u64.ms()).unwrap();

        assert!(
            compute_ticks.load(Ordering::Relaxed) > 0,
            "compute_node was never ticked"
        );
        assert!(
            main_ticks.load(Ordering::Relaxed) > 0,
            "main_node was never ticked"
        );
        assert_eq!(scheduler.node_list().len(), 2);
    }
}

#[test]
fn test_swarm_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();

    let (comm_node, comm_ticks) = TestNode::new("swarm_comm");
    let (behavior_node, behavior_ticks) = TestNode::new("swarm_behavior");

    scheduler.add(comm_node).order(0).build().unwrap();
    scheduler.add(behavior_node).order(1).build().unwrap();

    scheduler.run_for(100_u64.ms()).unwrap();

    assert!(
        comm_ticks.load(Ordering::Relaxed) > 0,
        "swarm_comm was never ticked"
    );
    assert!(
        behavior_ticks.load(Ordering::Relaxed) > 0,
        "swarm_behavior was never ticked"
    );
}

#[test]
fn test_soft_robotics_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();

    let (pressure_node, pressure_ticks) = TestNode::new("pressure_sensor");
    let (actuator_node, actuator_ticks) = TestNode::new("soft_actuator");

    scheduler.add(pressure_node).order(0).build().unwrap();
    scheduler.add(actuator_node).order(1).build().unwrap();

    scheduler.run_for(100_u64.ms()).unwrap();

    assert!(
        pressure_ticks.load(Ordering::Relaxed) > 0,
        "pressure_sensor was never ticked"
    );
    assert!(
        actuator_ticks.load(Ordering::Relaxed) > 0,
        "soft_actuator was never ticked"
    );
}

#[test]
fn test_high_performance_optimizer_nodes() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().tick_rate(10000_u64.hz());

    let (perf_sensor, perf_sensor_ticks) = TestNode::new("perf_sensor");
    let (perf_optimizer, perf_opt_ticks) = TestNode::new("perf_optimizer");

    scheduler.add(perf_sensor).order(0).build().unwrap();
    scheduler.add(perf_optimizer).order(1).build().unwrap();

    scheduler.run_for(100_u64.ms()).unwrap();

    // At 10kHz for 100ms we expect ~1000 ticks; be conservative for CI
    assert!(
        perf_sensor_ticks.load(Ordering::Relaxed) >= 10,
        "perf_sensor tick count too low: {}",
        perf_sensor_ticks.load(Ordering::Relaxed)
    );
    assert!(
        perf_opt_ticks.load(Ordering::Relaxed) >= 10,
        "perf_optimizer tick count too low: {}",
        perf_opt_ticks.load(Ordering::Relaxed)
    );
}

// ============================================================================
// Profile Preset Tests — new RT API DX
// ============================================================================

#[test]
fn test_deploy_preset() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(100_u64.hz());

    let (motor_node, motor_ticks) = TestNode::new("motor");
    let (sensor_node, sensor_ticks) = TestNode::new("sensor");

    scheduler
        .add(motor_node)
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();
    scheduler
        .add(sensor_node)
        .order(1)
        .rate(50_u64.hz())
        .build()
        .unwrap();

    scheduler.run_for(100_u64.ms()).unwrap();

    // Both nodes should have ticked
    let motor_count = motor_ticks.load(Ordering::Relaxed);
    let sensor_count = sensor_ticks.load(Ordering::Relaxed);
    assert!(motor_count > 0, "motor was never ticked");
    assert!(sensor_count > 0, "sensor was never ticked");

    // Motor at 100Hz should tick roughly 2x sensor at 50Hz (be lenient)
    // Just verify both ran and motor got at least as many ticks
    assert!(
        motor_count >= sensor_count,
        "motor ({}) should tick at least as often as sensor ({})",
        motor_count,
        sensor_count
    );
}

#[test]
fn test_hard_rt_preset() {
    cleanup_stale_shm();
    // require_rt() verifies system capabilities and enables all RT features.
    // On CI without RT support it panics — that's expected, so we skip the
    // full run in that case. But we no longer wrap this in catch_unwind +
    // discard the result.

    // Probe for RT support without panicking
    let has_rt = std::fs::read_to_string("/sys/kernel/realtime")
        .map(|s| s.trim() == "1")
        .unwrap_or(false);

    if !has_rt {
        // Non-RT system: verify the builder at least constructs, then
        // run without require_rt() so the test still exercises the path.
        let mut scheduler = Scheduler::new()
            .watchdog(500_u64.ms())
            .tick_rate(1000_u64.hz());

        let (rt_node, rt_ticks) = TestNode::new("rt_node");

        scheduler.add(rt_node).order(0).build().unwrap();

        scheduler.run_for(50_u64.ms()).unwrap();

        assert!(
            rt_ticks.load(Ordering::Relaxed) >= 5,
            "rt_node tick count too low: {}",
            rt_ticks.load(Ordering::Relaxed)
        );
        assert_eq!(scheduler.node_list(), vec!["rt_node".to_string()]);
    } else {
        // RT system: use require_rt() for real
        let mut scheduler = Scheduler::new()
            .require_rt()
            .watchdog(500_u64.ms())
            .tick_rate(1000_u64.hz());

        let (rt_node, rt_ticks) = TestNode::new("rt_node");

        scheduler.add(rt_node).order(0).build().unwrap();

        scheduler.run_for(50_u64.ms()).unwrap();

        assert!(
            rt_ticks.load(Ordering::Relaxed) >= 5,
            "rt_node tick count too low on RT system: {}",
            rt_ticks.load(Ordering::Relaxed)
        );
    }
}

#[test]
fn test_auto_derive_tick_rate_from_fastest_node() {
    cleanup_stale_shm();
    // If we set tick_rate(10) but add a node at 500Hz,
    // the scheduler should auto-bump to at least 500Hz
    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());

    let (fast_node, fast_ticks) = TestNode::new("fast_node");
    let (slow_node, slow_ticks) = TestNode::new("slow_node");

    scheduler
        .add(fast_node)
        .order(0)
        .rate(500_u64.hz())
        .build()
        .unwrap();
    scheduler
        .add(slow_node)
        .order(1)
        .rate(1_u64.hz())
        .build()
        .unwrap();

    // After run starts, tick rate should have been bumped
    scheduler.run_for(50_u64.ms()).unwrap();

    let fast_count = fast_ticks.load(Ordering::Relaxed);
    let slow_count = slow_ticks.load(Ordering::Relaxed);

    // The fast node at 500Hz over 50ms should get ~25 ticks.
    // If the scheduler stayed at 10Hz it could only do ~0-1 ticks total.
    // Use a conservative threshold that proves the auto-bump happened.
    assert!(
        fast_count >= 5,
        "fast_node got only {} ticks — tick rate was not auto-bumped from 10Hz",
        fast_count
    );

    // The fast node should have been ticked significantly more than the slow one
    assert!(
        fast_count > slow_count,
        "fast_node ({}) should tick more often than slow_node ({})",
        fast_count,
        slow_count
    );
}

#[test]
fn test_node_rate_via_builder_only() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    let (node_a, a_ticks) = TestNode::new("node_a");
    let (node_b, b_ticks) = TestNode::new("node_b");
    let (node_c, c_ticks) = TestNode::new("node_c");

    // Rate set via builder, not via Node::rate_hz()
    scheduler
        .add(node_a)
        .order(0)
        .rate(50_u64.hz())
        .build()
        .unwrap();
    scheduler
        .add(node_b)
        .order(1)
        .rate(25_u64.hz())
        .build()
        .unwrap();
    scheduler.add(node_c).order(2).build().unwrap(); // Uses global rate

    scheduler.run_for(100_u64.ms()).unwrap();

    let a_count = a_ticks.load(Ordering::Relaxed);
    let b_count = b_ticks.load(Ordering::Relaxed);
    let c_count = c_ticks.load(Ordering::Relaxed);

    // All nodes should have ticked at least once
    assert!(a_count > 0, "node_a was never ticked");
    assert!(b_count > 0, "node_b was never ticked");
    assert!(c_count > 0, "node_c was never ticked");

    // node_a at 50Hz should tick roughly 2x node_b at 25Hz
    assert!(
        a_count >= b_count,
        "node_a ({} at 50Hz) should tick at least as often as node_b ({} at 25Hz)",
        a_count,
        b_count
    );

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 3);
}

#[test]
fn test_preset_chaining() {
    cleanup_stale_shm();
    // Builder methods return Self so they can be chained
    let mut scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(200_u64.hz())
        .verbose(false);

    let (chained_node, chained_ticks) = TestNode::new("chained");

    scheduler.add(chained_node).order(0).build().unwrap();

    scheduler.run_for(50_u64.ms()).unwrap();

    // At 200Hz for 50ms we expect ~10 ticks
    assert!(
        chained_ticks.load(Ordering::Relaxed) >= 2,
        "chained node tick count too low: {}",
        chained_ticks.load(Ordering::Relaxed)
    );
    assert_eq!(scheduler.node_list(), vec!["chained".to_string()]);
}
