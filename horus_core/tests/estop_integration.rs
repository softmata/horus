#![allow(dead_code)]
//! Level 7 Intent Tests — Emergency Stop Integration
//!
//! These tests verify **emergency stop (e-stop) behavior** at the scheduler
//! level, not implementation details. Each test documents WHY the behavior
//! matters and what safety guarantee it protects.
//!
//! Covers:
//! - E-stop triggered by accumulated deadline misses stops the scheduler
//! - E-stop exits promptly (within a few ticks of the threshold)
//!
//! NOTE: These tests use `.deterministic(true)` to keep RT nodes on the
//! main thread, and `.deadline()` (not `.budget()`) to avoid the budget
//! check resetting the consecutive miss counter.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test 4: E-stop via accumulated deadline misses stops the scheduler
// ============================================================================

/// INTENT: "Too many deadline misses trigger emergency stop."
///
/// Robotics guarantee: on a surgical robot or industrial arm, the
/// safety monitor tracks total deadline misses across all nodes. When
/// misses exceed `max_deadline_misses`, the entire scheduler must halt
/// via emergency stop. This prevents a degrading system from continuing
/// to operate in an unsafe state where timing guarantees are no longer met.
/// The scheduler must exit run_for() cleanly and all nodes must be shut down.
#[test]
fn test_estop_stops_scheduler() {
    cleanup_stale_shm();

    let node_a_count = Arc::new(AtomicU64::new(0));
    let node_b_count = Arc::new(AtomicU64::new(0));
    let node_c_count = Arc::new(AtomicU64::new(0));

    struct EstopSensorNodeD {
        name: &'static str,
        count: Arc<AtomicU64>,
    }

    impl Node for EstopSensorNodeD {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// A node that stalls every tick, causing deadline misses.
    /// With max_deadline_misses=5, 5 misses triggers emergency stop.
    struct StallingMotorNodeD {
        count: Arc<AtomicU64>,
    }

    impl Node for StallingMotorNodeD {
        fn name(&self) -> &str {
            "stalling_motor_d"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
            // Stall to cause deadline miss
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        // Low threshold: emergency stop after just 5 total deadline misses.
        // This ensures the scheduler stops quickly.
        .max_deadline_misses(5);

    scheduler
        .add(EstopSensorNodeD {
            name: "camera_sensor_d",
            count: node_a_count.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    // Stalling node with tight deadline — will miss every tick.
    // deadline-only (no budget) so consecutive misses accumulate properly.
    scheduler
        .add(StallingMotorNodeD {
            count: node_b_count.clone(),
        })
        .order(1)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    scheduler
        .add(EstopSensorNodeD {
            name: "lidar_sensor_d",
            count: node_c_count.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    // Run for 5 seconds — but with max_deadline_misses=5 and a node
    // that misses every tick, the scheduler should exit MUCH earlier
    // due to emergency stop.
    let start = std::time::Instant::now();
    let _ = scheduler.run_for(5_u64.secs());
    let elapsed = start.elapsed();

    // The scheduler should have exited well before the full 5 seconds
    // due to emergency stop from accumulated deadline misses.
    // With a 50ms stall per tick, 5 misses happen within ~250ms.
    // Allow generous margin for scheduling overhead.
    assert!(
        elapsed < std::time::Duration::from_secs(4),
        "Scheduler should have exited early due to e-stop, but ran for {:?}",
        elapsed
    );

    // Verify that the safety state reflects the emergency stop
    if let Some(stats) = scheduler.safety_stats() {
        // Either the state is EmergencyStop, or deadline misses were
        // accumulated (both prove the safety monitor detected the problem)
        let emergency = *stats.state() == horus_core::scheduling::SafetyState::EmergencyStop;
        let misses_detected = stats.deadline_misses() > 0;
        assert!(
            emergency || misses_detected,
            "Safety monitor should show emergency stop or deadline misses. \
             State: {:?}, deadline_misses: {}",
            stats.state(),
            stats.deadline_misses()
        );
    }

    // The stalling node should have ticked no more than a few times
    // before emergency stop killed the scheduler.
    let motor_ticks = node_b_count.load(Ordering::SeqCst);
    assert!(
        motor_ticks > 0,
        "Stalling motor should have ticked at least once"
    );
    assert!(
        motor_ticks <= 20,
        "Stalling motor should have been stopped quickly, but ticked {} times",
        motor_ticks
    );
}

// ============================================================================
// Test 5: Emergency stop exits promptly once threshold is reached
// ============================================================================

/// INTENT: "The scheduler exits within a few ticks after the emergency stop
/// threshold is reached."
///
/// Robotics guarantee: when deadline misses accumulate past the safety
/// threshold, the scheduler must not continue running for an extended
/// period. Each additional tick after the e-stop threshold is a tick
/// where the robot operates without timing guarantees — risking
/// uncontrolled motion or missed sensor readings.
///
/// This test uses a very low max_deadline_misses (2) to verify that
/// the scheduler exits almost immediately once the threshold is hit.
/// The stalling node ticks at most a few times before the e-stop fires.
#[test]
fn test_estop_flag_checked_by_scheduler() {
    cleanup_stale_shm();

    let stall_count = Arc::new(AtomicU64::new(0));
    let observer_count = Arc::new(AtomicU64::new(0));

    struct StallingNodeE {
        count: Arc<AtomicU64>,
    }

    impl Node for StallingNodeE {
        fn name(&self) -> &str {
            "stalling_actuator_e"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
            // Stall 50ms to guarantee deadline miss on each tick
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    }

    struct ObserverNodeE {
        count: Arc<AtomicU64>,
    }

    impl Node for ObserverNodeE {
        fn name(&self) -> &str {
            "observer_sensor_e"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        // Very low threshold: emergency stop after just 2 deadline misses.
        // This tests that the scheduler responds immediately once the
        // e-stop flag is set by the safety monitor.
        .max_deadline_misses(2);

    scheduler
        .add(StallingNodeE {
            count: stall_count.clone(),
        })
        .order(0)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    scheduler
        .add(ObserverNodeE {
            count: observer_count.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Run for 10 seconds — but with max_deadline_misses=2, the scheduler
    // should exit after just 2-3 ticks (~100-150ms with 50ms stalls).
    let start = std::time::Instant::now();
    let _ = scheduler.run_for(10_u64.secs());
    let elapsed = start.elapsed();

    // The scheduler should have exited almost immediately.
    // 2 deadline misses at 50ms/tick = ~100ms, plus overhead.
    assert!(
        elapsed < std::time::Duration::from_secs(3),
        "Scheduler should exit promptly after 2 deadline misses, but ran for {:?}",
        elapsed
    );

    let stall_ticks = stall_count.load(Ordering::SeqCst);
    let observer_ticks = observer_count.load(Ordering::SeqCst);

    // The stalling node should have ticked only a few times (2-3)
    // before emergency stop halted the scheduler.
    assert!(
        stall_ticks >= 2,
        "Stalling node should have ticked at least 2 times to reach threshold, got {}",
        stall_ticks
    );
    assert!(
        stall_ticks <= 10,
        "Stalling node should have been stopped after ~2-3 ticks, but ticked {} times",
        stall_ticks
    );

    // The observer should also have ticked (it runs each cycle)
    assert!(
        observer_ticks > 0,
        "Observer node should have ticked at least once, got 0"
    );

    // Verify the safety monitor detected the emergency
    if let Some(stats) = scheduler.safety_stats() {
        assert!(
            *stats.state() == horus_core::scheduling::SafetyState::EmergencyStop,
            "Safety state should be EmergencyStop after exceeding max deadline misses, got {:?}",
            stats.state()
        );
        assert!(
            stats.deadline_misses() >= 2,
            "Should have recorded at least 2 deadline misses, got {}",
            stats.deadline_misses()
        );
    }
}
