//! Level 7 Intent Tests — BlackBox Flight Recorder
//!
//! These tests verify **behavioral intent** of the BlackBox flight recorder,
//! not implementation details. Each test documents WHY a behavior matters
//! and what user-visible guarantee it protects.
//!
//! Covers:
//! - Events written to the BlackBox are captured and retrievable
//! - Events persist across multiple ticks (no silent data loss)
//! - Anomalies (deadline misses, errors) are surfaced separately
//!
//! Note: The blackbox feature must be enabled for these tests to exercise
//! real recording. With `--no-default-features` the no-op stub is used
//! and assertions are adjusted accordingly (the stub always returns empty).

use horus_core::scheduling::{BlackBox, BlackBoxEvent};

mod common;

// ============================================================================
// Test 1: BlackBox captures events written to it
// ============================================================================

/// INTENT: "BlackBox captures events written to it."
///
/// Robotics guarantee: after a crash or mission failure, operators rely on
/// the flight recorder to contain a faithful record of what happened.
/// If events are silently dropped on write, post-mortem analysis becomes
/// impossible and the root cause of a hardware collision goes undiagnosed.
#[test]
fn test_blackbox_intent_events_recorded() {
    let mut bb = BlackBox::new(1); // 1 MB buffer

    // Record a scheduler start, a successful tick, and a custom event
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "test_scheduler".to_string(),
        node_count: 3,
        config: "default".to_string(),
    });

    bb.record(BlackBoxEvent::NodeTick {
        name: "lidar_node".to_string(),
        duration_us: 450,
        success: true,
    });

    bb.record(BlackBoxEvent::Custom {
        category: "diagnostic".to_string(),
        message: "CAN bus timeout on motor_ctrl".to_string(),
    });

    let events = bb.events();

    // With blackbox feature enabled, we expect real recording.
    // With the no-op stub, events() returns empty — verify graceful behavior.
    if cfg!(feature = "blackbox") {
        assert!(
            events.len() >= 3,
            "BlackBox should have captured at least 3 events, got {}",
            events.len()
        );

        // Verify the events are the ones we wrote (order preserved)
        assert!(
            matches!(events[0].event, BlackBoxEvent::SchedulerStart { .. }),
            "First event should be SchedulerStart"
        );
        assert!(
            matches!(events[1].event, BlackBoxEvent::NodeTick { .. }),
            "Second event should be NodeTick"
        );
        assert!(
            matches!(events[2].event, BlackBoxEvent::Custom { .. }),
            "Third event should be Custom"
        );
    } else {
        // No-op stub: verify that the API is callable and returns gracefully
        assert!(
            events.is_empty(),
            "No-op stub should return empty events vec"
        );
    }
}

// ============================================================================
// Test 2: Events recorded across multiple ticks are all preserved
// ============================================================================

/// INTENT: "Events recorded across multiple ticks are all preserved."
///
/// Robotics guarantee: a mission may run thousands of ticks. The flight
/// recorder must retain events from early ticks as well as late ticks
/// (subject to buffer capacity). If events silently disappear between
/// ticks, the recording has gaps that make timeline reconstruction
/// impossible during incident analysis.
#[test]
fn test_blackbox_intent_persist_across_ticks() {
    let mut bb = BlackBox::new(1); // 1 MB — more than enough for 10 events

    // Simulate 10 separate events representing 10 ticks worth of data
    for tick_num in 0..10u64 {
        bb.record(BlackBoxEvent::NodeTick {
            name: format!("sensor_{}", tick_num % 3),
            duration_us: 100 + tick_num * 10,
            success: true,
        });
    }

    let events = bb.events();

    if cfg!(feature = "blackbox") {
        assert_eq!(
            events.len(),
            10,
            "All 10 events should be preserved, got {}",
            events.len()
        );

        // Verify all events are NodeTick (no corruption or data loss)
        for (i, record) in events.iter().enumerate() {
            assert!(
                matches!(record.event, BlackBoxEvent::NodeTick { .. }),
                "Event {} should be NodeTick, got {:?}",
                i,
                record.event
            );
        }
    } else {
        assert!(
            events.is_empty(),
            "No-op stub should return empty events vec"
        );
    }
}

// ============================================================================
// Test 3: Anomalies (deadline misses, errors) are flagged
// ============================================================================

/// INTENT: "Anomalies (deadline misses, errors) are flagged."
///
/// Robotics guarantee: after a mission, operators need to quickly find
/// what went wrong without sifting through thousands of normal tick
/// events. The anomaly filter must surface deadline misses, budget
/// violations, errors, and emergency stops — the events that indicate
/// something deviated from expected behavior. Missing an anomaly means
/// a latent safety issue goes undetected.
#[test]
fn test_blackbox_intent_anomaly_detection() {
    let mut bb = BlackBox::new(1);

    // Record a mix of normal and anomalous events
    bb.record(BlackBoxEvent::NodeTick {
        name: "camera".to_string(),
        duration_us: 200,
        success: true,
    });

    bb.record(BlackBoxEvent::DeadlineMiss {
        name: "planner".to_string(),
        deadline_us: 1000,
        actual_us: 3500,
    });

    bb.record(BlackBoxEvent::NodeTick {
        name: "camera".to_string(),
        duration_us: 180,
        success: true,
    });

    bb.record(BlackBoxEvent::BudgetViolation {
        name: "slam".to_string(),
        budget_us: 5000,
        actual_us: 12000,
    });

    bb.record(BlackBoxEvent::EmergencyStop {
        reason: "watchdog expired on critical node".to_string(),
    });

    bb.record(BlackBoxEvent::NodeTick {
        name: "camera".to_string(),
        duration_us: 190,
        success: true,
    });

    // Total events: 6 (3 normal ticks + 3 anomalies)
    let all_events = bb.events();
    let anomalies = bb.anomalies();

    if cfg!(feature = "blackbox") {
        assert_eq!(all_events.len(), 6, "Should have recorded 6 total events");

        assert_eq!(
            anomalies.len(),
            3,
            "Should detect exactly 3 anomalies (DeadlineMiss + BudgetViolation + EmergencyStop), got {}",
            anomalies.len()
        );

        // Verify the anomaly types
        let has_deadline_miss = anomalies
            .iter()
            .any(|r| matches!(r.event, BlackBoxEvent::DeadlineMiss { .. }));
        let has_budget_violation = anomalies
            .iter()
            .any(|r| matches!(r.event, BlackBoxEvent::BudgetViolation { .. }));
        let has_emergency_stop = anomalies
            .iter()
            .any(|r| matches!(r.event, BlackBoxEvent::EmergencyStop { .. }));

        assert!(has_deadline_miss, "Anomalies should include DeadlineMiss");
        assert!(
            has_budget_violation,
            "Anomalies should include BudgetViolation"
        );
        assert!(has_emergency_stop, "Anomalies should include EmergencyStop");
    } else {
        // No-op stub: anomalies() also returns empty
        assert!(
            anomalies.is_empty(),
            "No-op stub anomalies() should return empty"
        );
    }
}
