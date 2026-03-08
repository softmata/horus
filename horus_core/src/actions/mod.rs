//! HORUS Actions Module
//!
//! Provides a robust action system for long-running tasks in robotics applications.
//! Actions are the HORUS equivalent of ROS2's action system, enabling:
//!
//! - **Long-running tasks** that take more than one tick to complete
//! - **Progress feedback** during execution
//! - **Cancellation** support for graceful interruption
//! - **Preemption** policies for goal management
//!
//! # Key Concepts
//!
//! ## Goal / Feedback / Result Pattern
//!
//! Every action defines three types:
//! - **Goal**: The request sent to start the action
//! - **Feedback**: Progress updates sent during execution
//! - **Result**: The final outcome when the action completes
//!
//! ## Action Server
//!
//! The server receives goals, executes them, and publishes feedback/results:
//!
//! ```rust,ignore
//! use horus_core::actions::{ActionServerBuilder, GoalResponse, CancelResponse};
//!
//! let server = ActionServerBuilder::<NavigateAction>::new()
//!     .on_goal(|goal| {
//!         if goal.target_x.is_finite() {
//!             GoalResponse::Accept
//!         } else {
//!             GoalResponse::Reject("Invalid target".into())
//!         }
//!     })
//!     .on_cancel(|_goal_id| CancelResponse::Accept)
//!     .on_execute(|handle| {
//!         while !handle.is_cancel_requested() {
//!             // Do work...
//!             handle.publish_feedback(feedback);
//!         }
//!         handle.succeed(result)
//!     })
//!     .build();
//! ```
//!
//! ## Action Client
//!
//! The client sends goals and monitors progress:
//!
//! ```rust,ignore
//! use horus_core::actions::ActionClientNode;
//!
//! let client = ActionClientNode::<NavigateAction>::builder()
//!     .on_feedback(|goal_id, feedback| {
//!         println!("Progress: {}%", feedback.progress_percent);
//!     })
//!     .build();
//!
//! let handle = client.send_goal(goal)?;
//! let result = handle.await_result(Duration::from_secs(30));
//! ```
//!
//! ## Defining Actions
//!
//! Use the `action!` macro for declarative action definitions:
//!
//! ```rust,ignore
//! use horus_core::action;
//!
//! action! {
//!     NavigateToGoal {
//!         goal {
//!             target_x: f64,
//!             target_y: f64,
//!             max_speed: f64 = 1.0,
//!         }
//!         feedback {
//!             distance_remaining: f64,
//!             progress_percent: f32,
//!         }
//!         result {
//!             success: bool,
//!             final_x: f64,
//!             final_y: f64,
//!         }
//!     }
//! }
//! ```
//!
//! # Architecture
//!
//! Actions communicate via HORUS Links (topics):
//!
//! ```text
//! Client                          Server
//!   |                                |
//!   |--- goal request ---->          | (action_name/goal)
//!   |                                |
//!   |<---- status update ----        | (action_name/status)
//!   |<---- feedback ----             | (action_name/feedback)
//!   |                                |
//!   |--- cancel request ---->        | (action_name/cancel)
//!   |                                |
//!   |<---- result ----               | (action_name/result)
//!   |                                |
//! ```
//!
//! # Preemption Policies
//!
//! Action servers support multiple preemption strategies:
//!
//! - **RejectNew**: New goals rejected while one is active
//! - **PreemptOld**: New goals cancel the active goal
//! - **Priority**: Higher priority goals preempt lower ones
//! - **Queue**: Goals queued up to a max size
//!
//! # Examples
//!
//! See the `examples/` directory for complete action examples:
//! - `navigate_action.rs` - Navigation action
//! - `manipulate_action.rs` - Manipulation action
//! - `action_sequence.rs` - Chaining actions

pub mod client;
pub mod macros;
pub mod server;
pub mod types;

// Re-export main types at module level
pub use client::{
    ActionClientBuilder, ActionClientMetrics, ActionClientNode, ClientGoalHandle, FeedbackCallback,
    ResultCallback, StatusCallback, SyncActionClient,
};
pub use server::{
    ActionServerBuilder, ActionServerMetrics, ActionServerNode, CancelCallback, ExecuteCallback,
    GoalCallback, GoalOutcome, ServerGoalHandle,
};
pub use types::{
    Action, ActionError, ActionFeedback, ActionResult, ActionServerConfig, CancelRequest,
    CancelResponse, GoalId, GoalPriority, GoalRequest, GoalResponse, GoalStatus, GoalStatusUpdate,
    PreemptionPolicy,
};

// Re-export macros
pub use crate::{action, simple_action, standard_action};

/// Prelude module for convenient imports.
///
/// ```rust,ignore
/// use horus_core::actions::prelude::*;
/// ```
pub mod prelude {
    pub use super::{
        Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
        ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
        GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_module_exports() {
        // Verify that main types are accessible
        let _id = GoalId::new();
        let _status = GoalStatus::Pending;
        let _priority = GoalPriority::NORMAL;
        let _policy = PreemptionPolicy::PreemptOld;
    }

    #[test]
    fn test_goal_status_transitions() {
        assert!(GoalStatus::Pending.is_active());
        assert!(GoalStatus::Active.is_active());
        assert!(!GoalStatus::Succeeded.is_active());
        assert!(GoalStatus::Succeeded.is_terminal());
    }

    #[test]
    fn test_preemption_policies() {
        let default = PreemptionPolicy::default();
        assert_eq!(default, PreemptionPolicy::PreemptOld);

        let queue = PreemptionPolicy::Queue { max_size: 5 };
        assert!(
            matches!(queue, PreemptionPolicy::Queue { max_size } if max_size == 5),
            "Expected Queue policy, got {:?}",
            queue
        );
    }
}
