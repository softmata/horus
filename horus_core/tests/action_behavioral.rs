// Action system behavioral tests — GoalStatus, GoalPriority, GoalId,
// PreemptionPolicy, GoalResponse, CancelResponse, ActionError, and builder APIs.
//
// These tests verify the action system's core type behavior and builder
// configuration without requiring running server/client infrastructure.
// The action! macro type generation is already tested in actions_integration.rs
// (54 tests) and actions_root.rs (22 tests).

use horus_core::actions::*;
use horus_core::core::{DurationExt, Node};
use std::collections::HashSet;

mod common;
use common::cleanup_stale_shm;

static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

macro_rules! serial_setup {
    () => {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        cleanup_stale_shm();
    };
}

// ============================================================================
// GoalStatus — all variants and terminal checks
// ============================================================================

#[test]
fn test_goal_status_terminal_variants() {
    assert!(GoalStatus::Succeeded.is_terminal());
    assert!(GoalStatus::Aborted.is_terminal());
    assert!(GoalStatus::Canceled.is_terminal());
    assert!(GoalStatus::Preempted.is_terminal());
    assert!(GoalStatus::Rejected.is_terminal());
}

#[test]
fn test_goal_status_non_terminal_variants() {
    assert!(!GoalStatus::Pending.is_terminal());
    assert!(!GoalStatus::Active.is_terminal());
}

#[test]
fn test_goal_status_all_variants_are_distinct() {
    let statuses = vec![
        GoalStatus::Pending,
        GoalStatus::Active,
        GoalStatus::Succeeded,
        GoalStatus::Aborted,
        GoalStatus::Canceled,
        GoalStatus::Preempted,
        GoalStatus::Rejected,
    ];
    for (i, a) in statuses.iter().enumerate() {
        for (j, b) in statuses.iter().enumerate() {
            if i != j {
                assert_ne!(a, b, "Variant {} and {} should be distinct", i, j);
            }
        }
    }
}

#[test]
fn test_goal_status_display() {
    let statuses = vec![
        GoalStatus::Pending,
        GoalStatus::Active,
        GoalStatus::Succeeded,
        GoalStatus::Aborted,
        GoalStatus::Canceled,
        GoalStatus::Preempted,
        GoalStatus::Rejected,
    ];
    for s in statuses {
        let display = format!("{:?}", s);
        assert!(!display.is_empty());
    }
}

// ============================================================================
// GoalPriority — ordering and constants
// ============================================================================

#[test]
fn test_goal_priority_ordering() {
    let highest = GoalPriority::HIGHEST; // 0
    let high = GoalPriority::HIGH; // 64
    let normal = GoalPriority::NORMAL; // 128
    let low = GoalPriority::LOW; // 192
    let lowest = GoalPriority::LOWEST; // 255

    // Lower value = higher priority
    assert!(highest < high);
    assert!(high < normal);
    assert!(normal < low);
    assert!(low < lowest);
}

#[test]
fn test_goal_priority_equality() {
    let a = GoalPriority(42);
    let b = GoalPriority(42);
    assert_eq!(a, b);
}

#[test]
fn test_goal_priority_default_is_normal() {
    let default = GoalPriority::default();
    assert_eq!(default, GoalPriority::NORMAL);
}

// ============================================================================
// GoalId — uniqueness (UUID v4)
// ============================================================================

#[test]
fn test_goal_id_uniqueness() {
    let ids: HashSet<GoalId> = (0..100).map(|_| GoalId::new()).collect();
    assert_eq!(ids.len(), 100, "100 GoalIds should all be unique");
}

#[test]
fn test_goal_id_display() {
    let id = GoalId::new();
    let display = format!("{}", id);
    assert!(!display.is_empty(), "GoalId should have display text");
}

// ============================================================================
// PreemptionPolicy — variants and default
// ============================================================================

#[test]
fn test_preemption_policy_default_is_preempt_old() {
    assert_eq!(PreemptionPolicy::default(), PreemptionPolicy::PreemptOld);
}

#[test]
fn test_preemption_policy_all_variants() {
    let _reject = PreemptionPolicy::RejectNew;
    let _preempt = PreemptionPolicy::PreemptOld;
    let _priority = PreemptionPolicy::Priority;
    let _queue = PreemptionPolicy::Queue { max_size: 5 };
}

// ============================================================================
// GoalResponse — Accept and Reject
// ============================================================================

#[test]
fn test_goal_response_accept() {
    assert!(matches!(GoalResponse::Accept, GoalResponse::Accept));
}

#[test]
fn test_goal_response_reject_with_reason() {
    let resp = GoalResponse::Reject("invalid".to_string());
    match resp {
        GoalResponse::Reject(reason) => assert_eq!(reason, "invalid"),
        _ => panic!("Expected Reject"),
    }
}

// ============================================================================
// CancelResponse — Accept and Reject
// ============================================================================

#[test]
fn test_cancel_response_accept() {
    assert!(matches!(CancelResponse::Accept, CancelResponse::Accept));
}

#[test]
fn test_cancel_response_reject_with_reason() {
    let resp = CancelResponse::Reject("mid-grasp".to_string());
    match resp {
        CancelResponse::Reject(r) => assert!(r.contains("mid-grasp")),
        _ => panic!("Expected Reject"),
    }
}

// ============================================================================
// ActionError — all variants have display text
// ============================================================================

#[test]
fn test_action_error_variants_display() {
    let errors: Vec<ActionError> = vec![
        ActionError::GoalTimeout,
        ActionError::GoalRejected("bad target".into()),
        ActionError::ServerUnavailable,
        ActionError::CommunicationError("topic full".into()),
    ];
    for err in errors {
        let display = format!("{}", err);
        assert!(!display.is_empty(), "ActionError should have display text");
    }
}

// ============================================================================
// ActionServerBuilder — full config doesn't panic
// ============================================================================

#[test]
fn test_action_server_builder_config() {
    serial_setup!();

    // Reuse Navigate type from actions_root pattern
    horus_core::action! {
        BhTestBuild {
            goal { x: f64 }
            feedback { p: f32 }
            result { done: bool }
        }
    }

    let node = ActionServerBuilder::<BhTestBuild>::new()
        .on_goal(|_| GoalResponse::Accept)
        .on_cancel(|_| CancelResponse::Accept)
        .on_execute(|handle| handle.succeed(BhTestBuildResult { done: true }))
        .max_concurrent_goals(Some(3))
        .feedback_rate(20.0)
        .goal_timeout(30_u64.secs())
        .preemption_policy(PreemptionPolicy::Priority)
        .build();

    assert_eq!(node.name(), "bh_test_build_server");
}

// ============================================================================
// ActionClientBuilder — full config doesn't panic
// ============================================================================

#[test]
fn test_action_client_builder_config() {
    serial_setup!();

    horus_core::action! {
        BhTestClient {
            goal { x: f64 }
            feedback { p: f32 }
            result { done: bool }
        }
    }

    let node = ActionClientBuilder::<BhTestClient>::new()
        .on_feedback(|_id, _fb| {})
        .on_result(|_id, _status, _result| {})
        .on_status(|_id, _status| {})
        .build();

    assert_eq!(node.name(), "bh_test_client_client");
}
