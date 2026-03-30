//! FFI wrappers for HORUS actions (long-running tasks with progress).
//!
//! ## Design
//!
//! Same JSON-based type erasure as services. Actions have a richer lifecycle:
//!   1. Client sends a Goal
//!   2. Server accepts/rejects → GoalResponse
//!   3. Server publishes Feedback periodically
//!   4. Server publishes final Result
//!   5. Client can Cancel at any time
//!
//! The FFI layer provides opaque client/server types with lifecycle management.

use crate::types_ffi::JsonWireMessage;
use horus_core::communication::Topic;
use std::sync::atomic::{AtomicU64, Ordering};

/// Goal status — matches Rust GoalStatus enum.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FfiGoalStatus {
    Pending = 0,
    Active = 1,
    Succeeded = 2,
    Aborted = 3,
    Canceled = 4,
    Rejected = 5,
}

impl FfiGoalStatus {
    pub fn is_terminal(self) -> bool {
        matches!(
            self,
            Self::Succeeded | Self::Aborted | Self::Canceled | Self::Rejected
        )
    }

    pub fn is_active(self) -> bool {
        matches!(self, Self::Pending | Self::Active)
    }
}

/// Goal response — accept or reject.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FfiGoalResponse {
    Accept = 0,
    Reject = 1,
}

/// Opaque action client — wired to real Pod topics for goal/feedback/result.
pub struct FfiActionClient {
    name: String,
    next_goal_id: AtomicU64,
    goal_topic: Option<Topic<JsonWireMessage>>,
    feedback_topic: Option<Topic<JsonWireMessage>>,
    result_topic: Option<Topic<JsonWireMessage>>,
}

/// Opaque action server — subscribes to goal, publishes feedback/result.
pub struct FfiActionServer {
    name: String,
    goal_topic: Option<Topic<JsonWireMessage>>,
    feedback_topic: Option<Topic<JsonWireMessage>>,
    result_topic: Option<Topic<JsonWireMessage>>,
    accept_handler: Option<Box<dyn Fn(&[u8]) -> FfiGoalResponse + Send + Sync>>,
    execute_handler: Option<Box<dyn Fn(u64, &[u8]) + Send + Sync>>,
}

/// Goal handle for tracking a sent goal.
pub struct FfiGoalHandle {
    goal_id: u64,
    action_name: String,
    status: FfiGoalStatus,
}

// ─── Action Client FFI ───────────────────────────────────────────────────────

/// Create a new action client with Pod-based topic wiring.
pub fn action_client_new(name: &str) -> Box<FfiActionClient> {
    let goal_topic = Topic::<JsonWireMessage>::new(&format!("{}.goal", name)).ok();
    let feedback_topic = Topic::<JsonWireMessage>::new(&format!("{}.feedback", name)).ok();
    let result_topic = Topic::<JsonWireMessage>::new(&format!("{}.result", name)).ok();
    Box::new(FfiActionClient {
        name: name.to_string(),
        next_goal_id: AtomicU64::new(1),
        goal_topic,
        feedback_topic,
        result_topic,
    })
}

/// Get the action name.
pub fn action_client_name(client: &FfiActionClient) -> &str {
    &client.name
}

/// Send a goal (JSON-encoded). Publishes on {action}.goal topic. Returns a goal handle.
pub fn action_client_send_goal(
    client: &FfiActionClient,
    goal_json: &str,
) -> Result<Box<FfiGoalHandle>, String> {
    let _: serde_json::Value =
        serde_json::from_str(goal_json).map_err(|e| format!("Invalid goal JSON: {}", e))?;

    let goal_id = client.next_goal_id.fetch_add(1, Ordering::Relaxed);

    // Publish goal via Pod JsonWireMessage
    if let Some(ref topic) = client.goal_topic {
        let wire = JsonWireMessage::from_json(goal_json, goal_id, 4) // type 4 = goal
            .ok_or_else(|| "Goal JSON too large (max 3968 bytes)".to_string())?;
        topic.send(wire);
    }

    Ok(Box::new(FfiGoalHandle {
        goal_id,
        action_name: client.name.clone(),
        status: FfiGoalStatus::Pending,
    }))
}

/// Cancel a goal.
pub fn action_client_cancel(handle: &mut FfiGoalHandle) {
    if handle.status.is_active() {
        handle.status = FfiGoalStatus::Canceled;
    }
}

/// Get goal status.
pub fn goal_handle_status(handle: &FfiGoalHandle) -> FfiGoalStatus {
    handle.status
}

/// Get goal ID.
pub fn goal_handle_id(handle: &FfiGoalHandle) -> u64 {
    handle.goal_id
}

/// Check if the goal is still active (not terminal).
pub fn goal_handle_is_active(handle: &FfiGoalHandle) -> bool {
    handle.status.is_active()
}

// ─── Action Server FFI ───────────────────────────────────────────────────────

/// Create a new action server with Pod-based topic wiring.
pub fn action_server_new(name: &str) -> Box<FfiActionServer> {
    let goal_topic = Topic::<JsonWireMessage>::new(&format!("{}.goal", name)).ok();
    let feedback_topic = Topic::<JsonWireMessage>::new(&format!("{}.feedback", name)).ok();
    let result_topic = Topic::<JsonWireMessage>::new(&format!("{}.result", name)).ok();
    Box::new(FfiActionServer {
        name: name.to_string(),
        goal_topic,
        feedback_topic,
        result_topic,
        accept_handler: None,
        execute_handler: None,
    })
}

/// Get the action server name.
pub fn action_server_name(server: &FfiActionServer) -> &str {
    &server.name
}

/// Set the goal acceptance handler.
///
/// `handler(goal_bytes, goal_len) -> 0 (accept) or 1 (reject)`
pub fn action_server_set_accept_handler(
    server: &mut FfiActionServer,
    handler: extern "C" fn(*const u8, usize) -> u8,
) {
    server.accept_handler = Some(Box::new(move |goal_bytes: &[u8]| -> FfiGoalResponse {
        let result = handler(goal_bytes.as_ptr(), goal_bytes.len());
        if result == 0 {
            FfiGoalResponse::Accept
        } else {
            FfiGoalResponse::Reject
        }
    }));
}

/// Set the goal execution handler.
///
/// `handler(goal_id, goal_bytes, goal_len)` — called when a goal is accepted.
/// The handler should publish feedback and eventually call complete/abort.
pub fn action_server_set_execute_handler(
    server: &mut FfiActionServer,
    handler: extern "C" fn(u64, *const u8, usize),
) {
    server.execute_handler = Some(Box::new(move |goal_id: u64, goal_bytes: &[u8]| {
        handler(goal_id, goal_bytes.as_ptr(), goal_bytes.len());
    }));
}

/// Check if the server has both handlers set.
pub fn action_server_is_ready(server: &FfiActionServer) -> bool {
    server.accept_handler.is_some() && server.execute_handler.is_some()
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn goal_status_terminal() {
        assert!(!FfiGoalStatus::Pending.is_terminal());
        assert!(!FfiGoalStatus::Active.is_terminal());
        assert!(FfiGoalStatus::Succeeded.is_terminal());
        assert!(FfiGoalStatus::Aborted.is_terminal());
        assert!(FfiGoalStatus::Canceled.is_terminal());
        assert!(FfiGoalStatus::Rejected.is_terminal());
    }

    #[test]
    fn goal_status_active() {
        assert!(FfiGoalStatus::Pending.is_active());
        assert!(FfiGoalStatus::Active.is_active());
        assert!(!FfiGoalStatus::Succeeded.is_active());
    }

    #[test]
    fn client_creation() {
        let client = action_client_new("navigate_to_goal");
        assert_eq!(action_client_name(&client), "navigate_to_goal");
    }

    #[test]
    fn send_goal_returns_handle() {
        let client = action_client_new("nav");
        let handle = action_client_send_goal(&client, r#"{"x": 1.0, "y": 2.0}"#).unwrap();
        assert_eq!(goal_handle_id(&handle), 1);
        assert_eq!(goal_handle_status(&handle), FfiGoalStatus::Pending);
        assert!(goal_handle_is_active(&handle));
    }

    #[test]
    fn send_goal_increments_id() {
        let client = action_client_new("nav");
        let h1 = action_client_send_goal(&client, r#"{}"#).unwrap();
        let h2 = action_client_send_goal(&client, r#"{}"#).unwrap();
        assert_eq!(goal_handle_id(&h1), 1);
        assert_eq!(goal_handle_id(&h2), 2);
    }

    #[test]
    fn send_goal_validates_json() {
        let client = action_client_new("nav");
        let result = action_client_send_goal(&client, "not json");
        assert!(result.is_err());
    }

    #[test]
    fn cancel_goal() {
        let client = action_client_new("nav");
        let mut handle = action_client_send_goal(&client, r#"{}"#).unwrap();
        assert!(goal_handle_is_active(&handle));
        action_client_cancel(&mut handle);
        assert_eq!(goal_handle_status(&handle), FfiGoalStatus::Canceled);
        assert!(!goal_handle_is_active(&handle));
    }

    #[test]
    fn cancel_terminal_goal_noop() {
        let client = action_client_new("nav");
        let mut handle = action_client_send_goal(&client, r#"{}"#).unwrap();
        handle.status = FfiGoalStatus::Succeeded;
        action_client_cancel(&mut handle); // Should be no-op
        assert_eq!(goal_handle_status(&handle), FfiGoalStatus::Succeeded);
    }

    #[test]
    fn server_creation() {
        let server = action_server_new("navigate");
        assert_eq!(action_server_name(&server), "navigate");
        assert!(!action_server_is_ready(&server));
    }

    #[test]
    fn server_accept_handler() {
        let mut server = action_server_new("nav");

        extern "C" fn always_accept(_: *const u8, _: usize) -> u8 {
            0
        }
        action_server_set_accept_handler(&mut server, always_accept);

        assert!(server.accept_handler.is_some());
        let handler = server.accept_handler.as_ref().unwrap();
        assert_eq!(handler(b"test"), FfiGoalResponse::Accept);
    }

    #[test]
    fn server_reject_handler() {
        let mut server = action_server_new("nav");

        extern "C" fn always_reject(_: *const u8, _: usize) -> u8 {
            1
        }
        action_server_set_accept_handler(&mut server, always_reject);

        let handler = server.accept_handler.as_ref().unwrap();
        assert_eq!(handler(b"test"), FfiGoalResponse::Reject);
    }

    #[test]
    fn action_goal_roundtrip_same_process() {
        // Client sends goal via Pod topic, server receives it
        let action_name = format!("action_rt.{}", std::process::id());

        let client = action_client_new(&action_name);
        let server = action_server_new(&action_name);

        // Client sends goal
        let handle = action_client_send_goal(&client, r#"{"target_x":5.0}"#).unwrap();
        assert_eq!(goal_handle_id(&handle), 1);

        // Server receives goal from topic
        if let Some(ref goal_topic) = server.goal_topic {
            let msg = goal_topic.recv();
            assert!(msg.is_some(), "server should receive goal");
            let wire = msg.unwrap();
            assert_eq!(wire.msg_id, 1);
            assert_eq!(wire.msg_type, 4); // goal type
            let json = wire.to_json().unwrap();
            assert!(json.contains("target_x"));
        } else {
            panic!("server goal topic not initialized");
        }
    }

    #[test]
    fn action_feedback_roundtrip() {
        let action_name = format!("action_fb.{}", std::process::id());
        let client = action_client_new(&action_name);
        let server = action_server_new(&action_name);

        // Server publishes feedback
        if let Some(ref fb_topic) = server.feedback_topic {
            let fb = JsonWireMessage::from_json(r#"{"progress":0.5}"#, 1, 3).unwrap();
            fb_topic.send(fb);
        }

        // Client receives feedback
        if let Some(ref fb_topic) = client.feedback_topic {
            let msg = fb_topic.recv();
            assert!(msg.is_some(), "client should receive feedback");
            let json = msg.unwrap().to_json().unwrap();
            assert!(json.contains("progress"));
        }
    }

    #[test]
    fn server_execute_handler() {
        use std::sync::atomic::AtomicU64;
        use std::sync::Arc;

        let mut server = action_server_new("nav");
        static EXECUTED_GOAL_ID: AtomicU64 = AtomicU64::new(0);

        extern "C" fn executor(goal_id: u64, _: *const u8, _: usize) {
            EXECUTED_GOAL_ID.store(goal_id, Ordering::Relaxed);
        }

        action_server_set_execute_handler(&mut server, executor);
        assert!(!action_server_is_ready(&server)); // Only execute handler, no accept

        // Set accept too
        extern "C" fn accept(_: *const u8, _: usize) -> u8 {
            0
        }
        action_server_set_accept_handler(&mut server, accept);
        assert!(action_server_is_ready(&server)); // Both handlers set

        // Test execute
        let handler = server.execute_handler.as_ref().unwrap();
        handler(42, b"goal_data");
        assert_eq!(EXECUTED_GOAL_ID.load(Ordering::Relaxed), 42);
    }
}
