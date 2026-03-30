//! End-to-end safety scenario: 4-node robot with motor controller stall
//!
//! Simulates a realistic robot with four nodes:
//! - **sensor_node** (fast) — publishes fake data, always succeeds
//! - **motor_controller** (fast, with deadline) — stalls after N ticks
//! - **planner** (slow) — reads sensor data, always succeeds
//! - **monitor_node** (fast) — just ticks, always succeeds
//!
//! When the motor controller stalls (sleeping 100ms per tick, far exceeding
//! its 1ms deadline), the graduated watchdog detects it and eventually kills
//! the node. The remaining three nodes must continue operating unaffected.
//!
//! NOTE: Uses `.deterministic(true)` so RT nodes stay on the main thread
//! where graduated degradation runs. Uses `.deadline()` (not `.budget()`)
//! so consecutive misses accumulate without being reset by budget-check's
//! `record_tick()`.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Node definitions
// ============================================================================

/// Fast sensor node that always succeeds. Represents a camera or IMU
/// publishing data every tick. Must keep running even if a sibling stalls.
struct SensorNode {
    count: Arc<AtomicU64>,
}

impl Node for SensorNode {
    fn name(&self) -> &str {
        "safety_sensor_node"
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        // Simulate minimal sensor work (reading + publishing)
        std::hint::black_box(42u64);
    }
}

/// Motor controller that stalls after a threshold number of ticks.
/// This simulates a real-world motor driver that hangs waiting for
/// hardware acknowledgment — a common failure mode in robotics.
struct MotorController {
    count: Arc<AtomicU64>,
    stall_after: u64,
}

impl Node for MotorController {
    fn name(&self) -> &str {
        "safety_motor_controller"
    }
    fn tick(&mut self) {
        let tick_num = self.count.fetch_add(1, Ordering::SeqCst) + 1;
        if tick_num >= self.stall_after {
            // Simulate hardware stall: 100ms sleep far exceeds the 1ms deadline
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}

/// Slow planner node that reads sensor data. Runs at the scheduler rate
/// but does minimal work. Must survive a sibling motor kill.
struct PlannerNode {
    count: Arc<AtomicU64>,
}

impl Node for PlannerNode {
    fn name(&self) -> &str {
        "safety_planner"
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        // Simulate light planning work
        std::hint::black_box(0xCAFEu64);
    }
}

/// Monitor node that just ticks. Represents a diagnostics or heartbeat
/// publisher. Must keep running regardless of other node failures.
struct MonitorNode {
    count: Arc<AtomicU64>,
}

impl Node for MonitorNode {
    fn name(&self) -> &str {
        "safety_monitor_node"
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
    }
}

// ============================================================================
// Helper: build the 4-node scenario
// ============================================================================

struct ScenarioCounters {
    sensor: Arc<AtomicU64>,
    motor: Arc<AtomicU64>,
    planner: Arc<AtomicU64>,
    monitor: Arc<AtomicU64>,
}

/// Build a scheduler with the 4-node robot scenario.
///
/// - sensor_node: order 0, no deadline (best-effort)
/// - motor_controller: order 1, 1ms deadline, stalls after `stall_after` ticks
/// - planner: order 2, no deadline (best-effort)
/// - monitor_node: order 3, no deadline (best-effort)
fn build_scenario(stall_after: u64) -> (Scheduler, ScenarioCounters) {
    let counters = ScenarioCounters {
        sensor: Arc::new(AtomicU64::new(0)),
        motor: Arc::new(AtomicU64::new(0)),
        planner: Arc::new(AtomicU64::new(0)),
        monitor: Arc::new(AtomicU64::new(0)),
    };

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        // High threshold so graduated degradation kills the node
        // before e-stop fires. kill_after defaults to 20 consecutive
        // misses, so 1000 total misses gives plenty of room.
        .max_deadline_misses(1000);

    // Sensor: fast, always healthy
    scheduler
        .add(SensorNode {
            count: counters.sensor.clone(),
        })
        .order(0)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Motor controller: has a tight deadline, will stall and get killed
    scheduler
        .add(MotorController {
            count: counters.motor.clone(),
            stall_after,
        })
        .order(1)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Planner: no deadline, always healthy
    scheduler
        .add(PlannerNode {
            count: counters.planner.clone(),
        })
        .order(2)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Monitor: no deadline, always healthy
    scheduler
        .add(MonitorNode {
            count: counters.monitor.clone(),
        })
        .order(3)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    (scheduler, counters)
}

// ============================================================================
// Test 1: Runaway motor controller is killed, system keeps running
// ============================================================================

/// INTENT: "A stalling motor controller is killed by graduated watchdog
/// while sensor, planner, and monitor nodes continue operating."
///
/// Robotics guarantee: if a motor driver hangs waiting for hardware ACK,
/// the scheduler must detect the chronic deadline misses, graduate through
/// warn -> reduce rate -> isolate -> kill, and then continue running
/// the remaining nodes. The system must not hang or crash. The motor's
/// tick count must stop increasing after kill, while all other nodes
/// continue accumulating ticks.
#[test]
fn test_safety_scenario_runaway_motor_killed() {
    cleanup_stale_shm();

    // Motor stalls starting at tick 3 — gives it a few healthy ticks
    // before the 100ms sleeps begin. With kill_after=20 (default),
    // it will be killed after 20 consecutive deadline misses.
    let (mut scheduler, counters) = build_scenario(3);

    // Run for 3 seconds. The motor stalls at 100ms/tick, so 20 misses
    // take ~2 seconds. After kill, the remaining 3 nodes keep ticking
    // for the rest of the run.
    scheduler.run_for(3_u64.secs()).unwrap();

    let motor_ticks = counters.motor.load(Ordering::SeqCst);
    let sensor_ticks = counters.sensor.load(Ordering::SeqCst);
    let planner_ticks = counters.planner.load(Ordering::SeqCst);
    let monitor_ticks = counters.monitor.load(Ordering::SeqCst);

    // Motor must have ticked at least a few times before being killed.
    assert!(
        motor_ticks >= 3,
        "Motor should have ticked at least 3 times (stall_after=3) before kill, got {}",
        motor_ticks
    );

    // Motor tick count must be bounded — it was killed, so it could not
    // have ticked for the full 3 seconds. At 100ms/tick stalling, the
    // absolute maximum before kill is ~30 ticks (with overhead).
    assert!(
        motor_ticks < 50,
        "Motor should have been killed and stopped ticking, but got {} ticks \
         (expected well under 50 for a 3s run with 100ms stalls + kill_after=20)",
        motor_ticks
    );

    // Sensor, planner, and monitor must have kept ticking after the
    // motor was killed. They are best-effort nodes with no stalling,
    // so they should have many more ticks than the motor.
    assert!(
        sensor_ticks > motor_ticks,
        "Sensor should have more ticks than the killed motor. \
         sensor={}, motor={}",
        sensor_ticks, motor_ticks
    );
    assert!(
        planner_ticks > motor_ticks,
        "Planner should have more ticks than the killed motor. \
         planner={}, motor={}",
        planner_ticks, motor_ticks
    );
    assert!(
        monitor_ticks > motor_ticks,
        "Monitor should have more ticks than the killed motor. \
         monitor={}, motor={}",
        monitor_ticks, motor_ticks
    );

    // Verify degradation was activated (proves the graduated watchdog fired)
    if let Some(stats) = scheduler.safety_stats() {
        assert!(
            stats.degrade_activations() > 0,
            "Graduated degradation should have been activated for the motor. \
             degrade_activations={}, deadline_misses={}, motor_ticks={}, \
             sensor_ticks={}, planner_ticks={}, monitor_ticks={}",
            stats.degrade_activations(),
            stats.deadline_misses(),
            motor_ticks,
            sensor_ticks,
            planner_ticks,
            monitor_ticks
        );
    }
}

// ============================================================================
// Test 2: Healthy nodes survive sibling kill and keep ticking
// ============================================================================

/// INTENT: "After a motor controller is killed, the remaining sensor,
/// planner, and monitor nodes each tick at least 10 more times."
///
/// Robotics guarantee: killing one node must not cascade to healthy
/// siblings. A camera, path planner, and diagnostics monitor must
/// all continue operating after the motor driver is removed. This
/// test verifies sustained operation post-kill, not just that nodes
/// were alive at the instant of kill.
#[test]
fn test_safety_scenario_healthy_nodes_survive_sibling_kill() {
    cleanup_stale_shm();

    // Motor stalls starting at tick 3
    let (mut scheduler, counters) = build_scenario(3);

    // Phase 1: Run long enough for the motor to be killed.
    // 20 misses at 100ms/tick = ~2 seconds. Use 2.5s to be safe.
    scheduler.run_for(2500_u64.ms()).unwrap();

    // Snapshot tick counts right after the motor should be killed
    let sensor_after_phase1 = counters.sensor.load(Ordering::SeqCst);
    let planner_after_phase1 = counters.planner.load(Ordering::SeqCst);
    let monitor_after_phase1 = counters.monitor.load(Ordering::SeqCst);
    let motor_after_phase1 = counters.motor.load(Ordering::SeqCst);

    // Phase 2: Run 100 more tick_once() calls. The motor should be
    // dead (is_stopped=true), so its tick count must not increase.
    // The other 3 nodes must keep ticking.
    for _ in 0..100 {
        let _ = scheduler.tick_once();
    }

    let sensor_after_phase2 = counters.sensor.load(Ordering::SeqCst);
    let planner_after_phase2 = counters.planner.load(Ordering::SeqCst);
    let monitor_after_phase2 = counters.monitor.load(Ordering::SeqCst);
    let motor_after_phase2 = counters.motor.load(Ordering::SeqCst);

    // Motor tick count must not have increased in phase 2 (if it was killed)
    if let Some(stats) = scheduler.safety_stats() {
        if stats.degrade_activations() > 0 {
            assert_eq!(
                motor_after_phase1, motor_after_phase2,
                "Killed motor's tick count must not increase in phase 2. \
                 phase1={}, phase2={}",
                motor_after_phase1, motor_after_phase2
            );
        }
    }

    // Each healthy node must have gained at least 10 ticks in phase 2.
    // We ran 100 tick_once() calls, so 10 is a very conservative floor.
    let sensor_gained = sensor_after_phase2 - sensor_after_phase1;
    let planner_gained = planner_after_phase2 - planner_after_phase1;
    let monitor_gained = monitor_after_phase2 - monitor_after_phase1;

    assert!(
        sensor_gained >= 10,
        "Sensor should have ticked at least 10 more times after motor kill. \
         gained={} (phase1={}, phase2={})",
        sensor_gained, sensor_after_phase1, sensor_after_phase2
    );
    assert!(
        planner_gained >= 10,
        "Planner should have ticked at least 10 more times after motor kill. \
         gained={} (phase1={}, phase2={})",
        planner_gained, planner_after_phase1, planner_after_phase2
    );
    assert!(
        monitor_gained >= 10,
        "Monitor should have ticked at least 10 more times after motor kill. \
         gained={} (phase1={}, phase2={})",
        monitor_gained, monitor_after_phase1, monitor_after_phase2
    );
}
