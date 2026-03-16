//! Integration tests for the HORUS actions system.
//!
//! Tests the full action lifecycle: macro generation, type system,
//! client/server builders, goal state machine, and standard actions.

use horus_core::actions::*;
use horus_core::action;
use std::time::Duration;

// ============================================================================
// 1. action! macro generates correct types
// ============================================================================

action! {
    MoveForward {
        goal {
            distance_m: f64,
            max_speed: f64,
        }
        feedback {
            distance_remaining: f64,
            current_speed: f64,
        }
        result {
            success: bool,
            actual_distance: f64,
        }
    }
}

#[test]
fn action_macro_generates_goal_type() {
    let goal = MoveForwardGoal {
        distance_m: 5.0,
        max_speed: 1.0,
    };
    assert_eq!(goal.distance_m, 5.0);
    assert_eq!(goal.max_speed, 1.0);
}

#[test]
fn action_macro_goal_new_constructor() {
    let goal = MoveForwardGoal::new(5.0, 2.0);
    assert_eq!(goal.distance_m, 5.0);
    assert_eq!(goal.max_speed, 2.0);
}

#[test]
fn action_macro_generates_feedback_type() {
    let fb = MoveForwardFeedback {
        distance_remaining: 3.0,
        current_speed: 0.8,
    };
    assert_eq!(fb.distance_remaining, 3.0);
    assert_eq!(fb.current_speed, 0.8);
}

#[test]
fn action_macro_generates_result_type() {
    let res = MoveForwardResult {
        success: true,
        actual_distance: 4.95,
    };
    assert!(res.success);
    assert_eq!(res.actual_distance, 4.95);
}

#[test]
fn action_macro_implements_action_trait() {
    assert_eq!(MoveForward::name(), "move_forward");
    assert_eq!(MoveForward::goal_topic(), "move_forward.goal");
    assert_eq!(MoveForward::cancel_topic(), "move_forward.cancel");
    assert_eq!(MoveForward::result_topic(), "move_forward.result");
    assert_eq!(MoveForward::feedback_topic(), "move_forward.feedback");
    assert_eq!(MoveForward::status_topic(), "move_forward.status");
}

#[test]
fn action_types_are_serializable() {
    let goal = MoveForwardGoal::new(3.0, 1.5);
    let json = serde_json::to_string(&goal).unwrap();
    let deserialized: MoveForwardGoal = serde_json::from_str(&json).unwrap();
    assert_eq!(deserialized.distance_m, 3.0);
    assert_eq!(deserialized.max_speed, 1.5);
}

#[test]
fn action_types_implement_clone_debug() {
    let goal = MoveForwardGoal::new(1.0, 2.0);
    let cloned = goal.clone();
    assert_eq!(cloned.distance_m, goal.distance_m);
    let debug_str = format!("{:?}", goal);
    assert!(debug_str.contains("MoveForwardGoal"));
}

// ============================================================================
// 2. Multiple action definitions coexist
// ============================================================================

action! {
    GripObject {
        goal {
            object_id: String,
            force_limit: f64,
        }
        feedback {
            grip_force: f64,
            contact: bool,
        }
        result {
            success: bool,
            final_force: f64,
        }
    }
}

#[test]
fn multiple_actions_coexist() {
    assert_eq!(MoveForward::name(), "move_forward");
    assert_eq!(GripObject::name(), "grip_object");
    assert_ne!(MoveForward::goal_topic(), GripObject::goal_topic());
}

#[test]
fn action_with_string_fields() {
    let goal = GripObjectGoal {
        object_id: "mug_42".to_string(),
        force_limit: 5.0,
    };
    let json = serde_json::to_string(&goal).unwrap();
    assert!(json.contains("mug_42"));
    let de: GripObjectGoal = serde_json::from_str(&json).unwrap();
    assert_eq!(de.object_id, "mug_42");
}

// ============================================================================
// 3. Goal status state machine
// ============================================================================

#[test]
fn goal_status_pending() {
    let s = GoalStatus::Pending;
    // Pending is considered "active" (not yet terminal)
    assert!(s.is_active());
    assert!(!s.is_terminal());
    assert!(!s.is_success());
    assert!(!s.is_failure());
}

#[test]
fn goal_status_active() {
    let s = GoalStatus::Active;
    assert!(s.is_active());
    assert!(!s.is_terminal());
}

#[test]
fn goal_status_succeeded() {
    let s = GoalStatus::Succeeded;
    assert!(s.is_terminal());
    assert!(s.is_success());
    assert!(!s.is_failure());
}

#[test]
fn goal_status_aborted() {
    let s = GoalStatus::Aborted;
    assert!(s.is_terminal());
    assert!(!s.is_success());
    assert!(s.is_failure());
}

#[test]
fn goal_status_canceled() {
    let s = GoalStatus::Canceled;
    assert!(s.is_terminal());
    assert!(s.is_failure());
}

#[test]
fn goal_status_preempted() {
    let s = GoalStatus::Preempted;
    assert!(s.is_terminal());
    assert!(s.is_failure());
}

#[test]
fn goal_status_rejected() {
    let s = GoalStatus::Rejected;
    assert!(s.is_terminal());
    assert!(s.is_failure());
}

#[test]
fn goal_status_display() {
    assert_eq!(format!("{}", GoalStatus::Pending), "PENDING");
    assert_eq!(format!("{}", GoalStatus::Active), "ACTIVE");
    assert_eq!(format!("{}", GoalStatus::Succeeded), "SUCCEEDED");
    assert_eq!(format!("{}", GoalStatus::Aborted), "ABORTED");
    assert_eq!(format!("{}", GoalStatus::Canceled), "CANCELED");
    assert_eq!(format!("{}", GoalStatus::Preempted), "PREEMPTED");
    assert_eq!(format!("{}", GoalStatus::Rejected), "REJECTED");
}

#[test]
fn goal_status_serialization() {
    let s = GoalStatus::Succeeded;
    let json = serde_json::to_string(&s).unwrap();
    let de: GoalStatus = serde_json::from_str(&json).unwrap();
    assert_eq!(de, GoalStatus::Succeeded);
}

// ============================================================================
// 4. Goal response and cancel response
// ============================================================================

#[test]
fn goal_response_accept() {
    let resp = GoalResponse::Accept;
    assert!(resp.is_accepted());
    assert!(!resp.is_rejected());
    assert_eq!(resp.rejection_reason(), None);
}

#[test]
fn goal_response_reject() {
    let resp = GoalResponse::Reject("too far".to_string());
    assert!(!resp.is_accepted());
    assert!(resp.is_rejected());
    assert_eq!(resp.rejection_reason(), Some("too far"));
}

#[test]
fn cancel_response_accept() {
    let resp = CancelResponse::Accept;
    assert!(resp.is_accepted());
    assert!(!resp.is_rejected());
}

#[test]
fn cancel_response_reject() {
    let resp = CancelResponse::Reject("in progress".to_string());
    assert!(!resp.is_accepted());
    assert!(resp.is_rejected());
    assert_eq!(resp.rejection_reason(), Some("in progress"));
}

// ============================================================================
// 5. Goal priority
// ============================================================================

#[test]
fn goal_priority_ordering() {
    assert!(GoalPriority::HIGHEST.is_higher_than(&GoalPriority::HIGH));
    assert!(GoalPriority::HIGH.is_higher_than(&GoalPriority::NORMAL));
    assert!(GoalPriority::NORMAL.is_higher_than(&GoalPriority::LOW));
    assert!(GoalPriority::LOW.is_higher_than(&GoalPriority::LOWEST));
    assert!(!GoalPriority::LOWEST.is_higher_than(&GoalPriority::HIGHEST));
}

#[test]
fn goal_priority_values() {
    assert_eq!(GoalPriority::HIGHEST.0, 0);
    assert_eq!(GoalPriority::HIGH.0, 64);
    assert_eq!(GoalPriority::NORMAL.0, 128);
    assert_eq!(GoalPriority::LOW.0, 192);
    assert_eq!(GoalPriority::LOWEST.0, 255);
}

#[test]
fn goal_priority_custom() {
    let p = GoalPriority(100);
    assert!(p.is_higher_than(&GoalPriority::NORMAL));
    assert!(!p.is_higher_than(&GoalPriority::HIGHEST));
}

// ============================================================================
// 6. Preemption policy
// ============================================================================

#[test]
fn preemption_policy_default_is_preempt_old() {
    let policy = PreemptionPolicy::default();
    assert!(matches!(policy, PreemptionPolicy::PreemptOld));
}

#[test]
fn preemption_policy_queue() {
    let policy = PreemptionPolicy::Queue { max_size: 5 };
    if let PreemptionPolicy::Queue { max_size } = policy {
        assert_eq!(max_size, 5);
    } else {
        panic!("Expected Queue variant");
    }
}

// ============================================================================
// 7. Action error
// ============================================================================

#[test]
fn action_error_goal_rejected() {
    let err = ActionError::GoalRejected("invalid target".to_string());
    let display = format!("{}", err);
    assert!(display.contains("invalid target") || display.contains("rejected"));
}

#[test]
fn action_error_converts_to_horus_error() {
    let err = ActionError::GoalTimeout;
    let horus_err: horus_core::HorusError = err.into();
    let display = format!("{}", horus_err);
    assert!(!display.is_empty());
}

// ============================================================================
// 8. GoalId
// ============================================================================

#[test]
fn goal_id_unique() {
    let id1 = GoalId::new();
    let id2 = GoalId::new();
    assert_ne!(id1, id2);
}

#[test]
fn goal_id_display() {
    let id = GoalId::new();
    let s = format!("{}", id);
    assert!(!s.is_empty());
    // UUID format: 8-4-4-4-12 characters
    assert!(s.len() >= 32);
}

#[test]
fn goal_id_copy() {
    let id = GoalId::new();
    let copied = id;
    assert_eq!(id, copied);
}

#[test]
fn goal_id_hash() {
    use std::collections::HashSet;
    let mut set = HashSet::new();
    let id1 = GoalId::new();
    let id2 = GoalId::new();
    set.insert(id1);
    set.insert(id2);
    assert_eq!(set.len(), 2);
    assert!(set.contains(&id1));
}

#[test]
fn goal_id_serialization() {
    let id = GoalId::new();
    let json = serde_json::to_string(&id).unwrap();
    let deserialized: GoalId = serde_json::from_str(&json).unwrap();
    assert_eq!(id, deserialized);
}

// ============================================================================
// 9. Wire message types
// ============================================================================

#[test]
fn goal_request_roundtrip() {
    let request = GoalRequest {
        goal_id: GoalId::new(),
        goal: MoveForwardGoal::new(5.0, 1.0),
        priority: GoalPriority::NORMAL,
        timestamp: Duration::from_secs(100),
    };
    let json = serde_json::to_string(&request).unwrap();
    let de: GoalRequest<MoveForwardGoal> = serde_json::from_str(&json).unwrap();
    assert_eq!(de.goal.distance_m, 5.0);
    assert_eq!(de.priority, GoalPriority::NORMAL);
}

#[test]
fn action_feedback_roundtrip() {
    let fb = ActionFeedback {
        goal_id: GoalId::new(),
        feedback: MoveForwardFeedback {
            distance_remaining: 2.5,
            current_speed: 0.9,
        },
        timestamp: Duration::from_millis(500),
    };
    let json = serde_json::to_string(&fb).unwrap();
    let de: ActionFeedback<MoveForwardFeedback> = serde_json::from_str(&json).unwrap();
    assert_eq!(de.feedback.distance_remaining, 2.5);
}

#[test]
fn action_result_roundtrip() {
    let result = ActionResult {
        goal_id: GoalId::new(),
        status: GoalStatus::Succeeded,
        result: MoveForwardResult {
            success: true,
            actual_distance: 4.95,
        },
        timestamp: Duration::from_secs(10),
    };
    let json = serde_json::to_string(&result).unwrap();
    let de: ActionResult<MoveForwardResult> = serde_json::from_str(&json).unwrap();
    assert!(de.result.success);
    assert_eq!(de.status, GoalStatus::Succeeded);
}

#[test]
fn cancel_request_roundtrip() {
    let cancel = CancelRequest {
        goal_id: GoalId::new(),
        timestamp: Duration::from_millis(200),
    };
    let json = serde_json::to_string(&cancel).unwrap();
    let de: CancelRequest = serde_json::from_str(&json).unwrap();
    assert_eq!(cancel.goal_id, de.goal_id);
}

// ============================================================================
// 10. ActionClientBuilder
// ============================================================================

#[test]
fn action_client_builder_creates() {
    let _builder = ActionClientBuilder::<MoveForward>::new();
}

// ============================================================================
// 11. standard_action! — smoke test all four patterns
// ============================================================================

// NOTE: standard_action! generates the same types as action! with pre-defined
// fields for common robotics patterns. We test it compiles and produces the
// expected trait implementations.

// NOTE: standard_action! tests are in horus_core/src/actions/macros.rs unit tests
// because the nested macro expansion ($crate::action! inside $crate::standard_action!)
// requires crate-internal resolution that integration tests cannot provide.

// ============================================================================
// 12. GoalId::from_uuid / as_uuid roundtrip
// ============================================================================

#[test]
fn goal_id_from_uuid_roundtrip() {
    let uuid = uuid::Uuid::new_v4();
    let goal_id = GoalId::from_uuid(uuid);
    assert_eq!(*goal_id.as_uuid(), uuid);
}

#[test]
fn goal_id_from_uuid_preserves_value() {
    let uuid = uuid::Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap();
    let goal_id = GoalId::from_uuid(uuid);
    let display = format!("{}", goal_id);
    assert_eq!(display, "550e8400-e29b-41d4-a716-446655440000");
}

#[test]
fn goal_id_as_uuid_returns_reference() {
    let id = GoalId::new();
    let uuid_ref = id.as_uuid();
    // Calling as_uuid twice should return the same value
    assert_eq!(uuid_ref, id.as_uuid());
}

// ============================================================================
// 13. ActionServerConfig builder methods
// ============================================================================

#[test]
fn action_server_config_default_values() {
    let config = ActionServerConfig::new();
    assert_eq!(config.max_concurrent_goals, Some(1));
    assert_eq!(config.feedback_rate_hz, 10.0);
    assert_eq!(config.goal_timeout, None);
    assert_eq!(config.preemption_policy, PreemptionPolicy::PreemptOld);
    assert_eq!(config.result_history_size, 100);
}

#[test]
fn action_server_config_unlimited_goals() {
    let config = ActionServerConfig::new().unlimited_goals();
    assert_eq!(config.max_concurrent_goals, None);
}

#[test]
fn action_server_config_max_goals() {
    let config = ActionServerConfig::new().max_goals(5);
    assert_eq!(config.max_concurrent_goals, Some(5));
}

#[test]
fn action_server_config_feedback_rate() {
    let config = ActionServerConfig::new().feedback_rate(50.0);
    assert_eq!(config.feedback_rate_hz, 50.0);
}

#[test]
fn action_server_config_timeout() {
    let config = ActionServerConfig::new().timeout(Duration::from_secs(30));
    assert_eq!(config.goal_timeout, Some(Duration::from_secs(30)));
}

#[test]
fn action_server_config_preemption() {
    let config = ActionServerConfig::new().preemption(PreemptionPolicy::RejectNew);
    assert_eq!(config.preemption_policy, PreemptionPolicy::RejectNew);
}

#[test]
fn action_server_config_history_size() {
    let config = ActionServerConfig::new().history_size(500);
    assert_eq!(config.result_history_size, 500);
}

#[test]
fn action_server_config_builder_chaining() {
    let config = ActionServerConfig::new()
        .max_goals(3)
        .feedback_rate(20.0)
        .timeout(Duration::from_secs(60))
        .preemption(PreemptionPolicy::Priority)
        .history_size(200);
    assert_eq!(config.max_concurrent_goals, Some(3));
    assert_eq!(config.feedback_rate_hz, 20.0);
    assert_eq!(config.goal_timeout, Some(Duration::from_secs(60)));
    assert_eq!(config.preemption_policy, PreemptionPolicy::Priority);
    assert_eq!(config.result_history_size, 200);
}

// ============================================================================
// 14. GoalOutcome::into_result
// ============================================================================

#[test]
fn goal_outcome_into_result_succeeded() {
    let outcome = GoalOutcome::<MoveForward>::Succeeded(MoveForwardResult {
        success: true,
        actual_distance: 4.95,
    });
    let result = outcome.into_result();
    assert!(result.success);
    assert_eq!(result.actual_distance, 4.95);
}

#[test]
fn goal_outcome_into_result_aborted() {
    let outcome = GoalOutcome::<MoveForward>::Aborted(MoveForwardResult {
        success: false,
        actual_distance: 2.0,
    });
    let result = outcome.into_result();
    assert!(!result.success);
    assert_eq!(result.actual_distance, 2.0);
}

#[test]
fn goal_outcome_into_result_canceled() {
    let outcome = GoalOutcome::<MoveForward>::Canceled(MoveForwardResult {
        success: false,
        actual_distance: 1.5,
    });
    let result = outcome.into_result();
    assert!(!result.success);
    assert_eq!(result.actual_distance, 1.5);
}

#[test]
fn goal_outcome_into_result_preempted() {
    let outcome = GoalOutcome::<MoveForward>::Preempted(MoveForwardResult {
        success: false,
        actual_distance: 3.0,
    });
    let result = outcome.into_result();
    assert!(!result.success);
    assert_eq!(result.actual_distance, 3.0);
}
