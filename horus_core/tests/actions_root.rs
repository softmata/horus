//! Root tests for the Actions module.
//!
//! Tests ActionServerBuilder, ActionClientNode, SyncActionClient,
//! ServerGoalHandle, goal lifecycle, cancellation, feedback, and types.

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::DurationExt;
use horus_core::Node; // For .name() method
use std::time::Duration;

// Define test actions
action! {
    Navigate {
        goal { x: f64, y: f64 }
        feedback { distance_remaining: f64 }
        result { success: bool }
    }
}

action! {
    Compute {
        goal { input: u32 }
        feedback { progress: f32 }
        result { output: u64 }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Types
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_goal_id_unique() {
    let id1 = GoalId::new();
    let id2 = GoalId::new();
    assert_ne!(id1.as_uuid(), id2.as_uuid(), "goal IDs must be unique");
}

#[test]
fn test_goal_status_active_vs_terminal() {
    assert!(GoalStatus::Pending.is_active());
    assert!(GoalStatus::Active.is_active());
    assert!(!GoalStatus::Succeeded.is_active());
    assert!(GoalStatus::Succeeded.is_terminal());
    assert!(GoalStatus::Aborted.is_terminal());
    assert!(GoalStatus::Canceled.is_terminal());
    assert!(GoalStatus::Preempted.is_terminal());
    assert!(GoalStatus::Rejected.is_terminal());
}

#[test]
fn test_goal_status_success_vs_failure() {
    assert!(GoalStatus::Succeeded.is_success());
    assert!(!GoalStatus::Aborted.is_success());
    assert!(GoalStatus::Aborted.is_failure());
    assert!(GoalStatus::Canceled.is_failure());
    assert!(GoalStatus::Rejected.is_failure());
    assert!(!GoalStatus::Succeeded.is_failure());
}

#[test]
fn test_goal_priority_ordering() {
    assert!(GoalPriority::HIGHEST.is_higher_than(&GoalPriority::NORMAL));
    assert!(GoalPriority::HIGH.is_higher_than(&GoalPriority::LOW));
    assert!(!GoalPriority::LOW.is_higher_than(&GoalPriority::HIGH));
    assert!(!GoalPriority::NORMAL.is_higher_than(&GoalPriority::NORMAL));
}

#[test]
fn test_goal_response_accept_reject() {
    let accept = GoalResponse::Accept;
    assert!(accept.is_accepted());
    assert!(!accept.is_rejected());
    assert!(accept.rejection_reason().is_none());

    let reject = GoalResponse::Reject("too far".into());
    assert!(!reject.is_accepted());
    assert!(reject.is_rejected());
    assert_eq!(reject.rejection_reason(), Some("too far"));
}

#[test]
fn test_cancel_response() {
    assert!(CancelResponse::Accept.is_accepted());
    let reject = CancelResponse::Reject("in progress".into());
    assert!(reject.is_rejected());
    assert_eq!(reject.rejection_reason(), Some("in progress"));
}

#[test]
fn test_action_topic_names() {
    assert_eq!(Navigate::goal_topic(), "navigate.goal");
    assert_eq!(Navigate::cancel_topic(), "navigate.cancel");
    assert_eq!(Navigate::result_topic(), "navigate.result");
    assert_eq!(Navigate::feedback_topic(), "navigate.feedback");
    assert_eq!(Navigate::status_topic(), "navigate.status");
}

#[test]
fn test_goal_request_new() {
    let req = GoalRequest::new(NavigateGoal { x: 1.0, y: 2.0 });
    assert_eq!(req.priority, GoalPriority::NORMAL);
    assert_eq!(req.goal.x, 1.0);
    assert_eq!(req.goal.y, 2.0);
}

#[test]
fn test_goal_request_with_priority() {
    let req = GoalRequest::with_priority(
        NavigateGoal { x: 0.0, y: 0.0 },
        GoalPriority::HIGH,
    );
    assert_eq!(req.priority, GoalPriority::HIGH);
}

#[test]
fn test_action_result_variants() {
    let gid = GoalId::new();
    let s = ActionResult::succeeded(gid, NavigateResult { success: true });
    assert_eq!(s.status, GoalStatus::Succeeded);

    let a = ActionResult::aborted(GoalId::new(), NavigateResult { success: false });
    assert_eq!(a.status, GoalStatus::Aborted);

    let c = ActionResult::canceled(GoalId::new(), NavigateResult { success: false });
    assert_eq!(c.status, GoalStatus::Canceled);
}

#[test]
fn test_action_feedback_new() {
    let gid = GoalId::new();
    let fb = ActionFeedback::new(gid, NavigateFeedback { distance_remaining: 5.0 });
    assert_eq!(fb.feedback.distance_remaining, 5.0);
}

#[test]
fn test_goal_status_update() {
    let gid = GoalId::new();
    let update = GoalStatusUpdate::new(gid, GoalStatus::Active);
    assert_eq!(update.status, GoalStatus::Active);
}

// ═══════════════════════════════════════════════════════════════════════════
// ActionServerConfig
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_server_config_defaults() {
    let cfg = ActionServerConfig::new();
    assert_eq!(cfg.preemption_policy, PreemptionPolicy::PreemptOld);
}

#[test]
fn test_server_config_builder_chain() {
    let cfg = ActionServerConfig::new()
        .max_goals(5)
        .feedback_rate(10.0)
        .timeout(30_u64.secs())
        .preemption(PreemptionPolicy::PreemptOld)
        .history_size(100);

    assert_eq!(cfg.max_concurrent_goals, Some(5));
    assert_eq!(cfg.feedback_rate_hz, 10.0);
    assert_eq!(cfg.goal_timeout, Some(30_u64.secs()));
    assert_eq!(cfg.preemption_policy, PreemptionPolicy::PreemptOld);
    assert_eq!(cfg.result_history_size, 100);
}

#[test]
fn test_server_config_unlimited_goals() {
    let cfg = ActionServerConfig::new().unlimited_goals();
    assert_eq!(cfg.max_concurrent_goals, None);
}

// ═══════════════════════════════════════════════════════════════════════════
// ActionServerBuilder
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_action_server_builder() {
    let node = ActionServerBuilder::<Navigate>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| handle.succeed(NavigateResult { success: true }))
        .max_concurrent_goals(Some(3))
        .feedback_rate(20.0)
        .goal_timeout(10_u64.secs())
        .preemption_policy(PreemptionPolicy::Priority)
        .build();

    assert_eq!(node.name(), "navigate_server");
}

#[test]
fn test_action_server_builder_static() {
    let node = ActionServerNode::<Compute>::builder()
        .on_execute(|handle| handle.succeed(ComputeResult { output: 42 }))
        .build();

    assert_eq!(node.name(), "compute_server");
}

// ═══════════════════════════════════════════════════════════════════════════
// ActionClientBuilder
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_action_client_builder() {
    let node = ActionClientBuilder::<Navigate>::new()
        .on_feedback(|_id, _fb| {})
        .on_result(|_id, _status, _result| {})
        .on_status(|_id, _status| {})
        .build();

    assert_eq!(node.name(), "navigate_client");
}

#[test]
fn test_action_client_builder_static() {
    let node = ActionClientNode::<Compute>::builder().build();
    assert_eq!(node.name(), "compute_client");
}

// ═══════════════════════════════════════════════════════════════════════════
// ActionError
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_action_error_variants() {
    let timeout = ActionError::GoalTimeout;
    let timeout_str = format!("{}", timeout);
    assert!(!timeout_str.is_empty(), "GoalTimeout should have display text");

    let rejected = ActionError::GoalRejected("too far".into());
    let rejected_str = format!("{}", rejected);
    assert!(!rejected_str.is_empty(), "GoalRejected should have display text");
    assert!(rejected_str.contains("too far"), "should contain reason: {}", rejected_str);
}

// ═══════════════════════════════════════════════════════════════════════════
// GoalOutcome
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_goal_outcome_status() {
    let succeeded: GoalOutcome<Navigate> = GoalOutcome::Succeeded(NavigateResult { success: true });
    assert_eq!(succeeded.status(), GoalStatus::Succeeded);

    let aborted: GoalOutcome<Navigate> = GoalOutcome::Aborted(NavigateResult { success: false });
    assert_eq!(aborted.status(), GoalStatus::Aborted);
}

#[test]
fn test_goal_outcome_into_result() {
    let outcome: GoalOutcome<Compute> = GoalOutcome::Succeeded(ComputeResult { output: 99 });
    let result = outcome.into_result();
    assert_eq!(result.output, 99);
}
