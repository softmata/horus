//! FFI wrappers for HORUS actions (long-running tasks with progress + cancel).
//!
//! ## Design
//!
//! JSON/`JsonWireMessage`-based type erasure (same as services). Actions have a
//! richer lifecycle than services:
//!   1. Client sends a Goal on `{name}.goal`.
//!   2. Server's `process()` receives it, calls the accept handler, and — if
//!      accepted — runs the execute handler on a DEDICATED THREAD (thread-per-
//!      goal), publishing an Active status.
//!   3. The execute handler publishes Feedback and finishes with succeed/abort/
//!      cancel via its goal handle.
//!   4. The server publishes the final Result + terminal status.
//!   5. The client can Cancel at any time on `{name}.cancel`; a running handler
//!      observes it via `is_cancel_requested()`.
//!
//! ## Threading contract (mirrors the Rust `ActionServerNode` fix)
//!
//! `Topic` carries a single-thread contract — it is unsafe under concurrent
//! producers. Therefore ALL action-topic I/O happens on the thread that calls
//! `action_server_process`. Goal-execution threads NEVER touch a topic: they
//! only read a shared `Arc<AtomicBool>` cancel flag and push feedback/completion
//! into an event inbox that `process()` drains. Construct the server and call
//! `process()` on the SAME thread.

use crate::types_ffi::JsonWireMessage;
use horus_core::communication::Topic;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

// Wire `msg_type` tags. Each action sub-channel is its own topic, so these only
// need to be self-consistent between the sender and receiver of one topic.
const MSG_GOAL: u8 = 4;
const MSG_FEEDBACK: u8 = 3;
const MSG_CANCEL: u8 = 5;
// On the result/status topics, `msg_type` carries the terminal FfiGoalStatus.

/// Process-global, cross-process-unique goal-id source. Per-client counters
/// would collide (two clients both emit 1, 2, 3…), making cancel/result
/// correlation ambiguous — so ids mix the PID into the high bits.
static GOAL_ID_COUNTER: AtomicU64 = AtomicU64::new(1);

fn next_goal_id() -> u64 {
    let n = GOAL_ID_COUNTER.fetch_add(1, Ordering::Relaxed) & 0xFFFF_FFFF;
    ((std::process::id() as u64) << 32) | n
}

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

    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Pending,
            1 => Self::Active,
            2 => Self::Succeeded,
            3 => Self::Aborted,
            4 => Self::Canceled,
            _ => Self::Rejected,
        }
    }
}

/// Goal response — accept or reject.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FfiGoalResponse {
    Accept = 0,
    Reject = 1,
}

/// Event produced by a goal-execution thread, drained and published by
/// `action_server_process` on the process() thread (single producer per topic).
enum FfiServerEvent {
    /// Feedback message (already encoded, `msg_id` = goal id).
    Feedback(JsonWireMessage),
    /// Goal finished: (goal_id, terminal status, result message).
    Completed(u64, FfiGoalStatus, JsonWireMessage),
}

type EventInbox = Arc<Mutex<Vec<FfiServerEvent>>>;

// ─── Action Client FFI ───────────────────────────────────────────────────────

/// Opaque action client — wired to real Pod topics for goal/cancel/feedback/result.
#[allow(dead_code)]
pub struct FfiActionClient {
    name: String,
    goal_topic: Option<Topic<JsonWireMessage>>,
    cancel_topic: Option<Topic<JsonWireMessage>>,
    feedback_topic: Option<Topic<JsonWireMessage>>,
    result_topic: Option<Topic<JsonWireMessage>>,
}

/// Goal handle for tracking a sent goal (client side).
#[allow(dead_code)]
pub struct FfiGoalHandle {
    goal_id: u64,
    action_name: String,
    status: FfiGoalStatus,
}

/// Create a new action client with Pod-based topic wiring.
pub fn action_client_new(name: &str) -> Box<FfiActionClient> {
    Box::new(FfiActionClient {
        name: name.to_string(),
        goal_topic: Topic::<JsonWireMessage>::new(format!("{}.goal", name)).ok(),
        cancel_topic: Topic::<JsonWireMessage>::new(format!("{}.cancel", name)).ok(),
        feedback_topic: Topic::<JsonWireMessage>::new(format!("{}.feedback", name)).ok(),
        result_topic: Topic::<JsonWireMessage>::new(format!("{}.result", name)).ok(),
    })
}

/// Get the action name.
pub fn action_client_name(client: &FfiActionClient) -> &str {
    &client.name
}

/// Send a goal (JSON-encoded). Publishes on `{name}.goal`. Returns a goal handle.
pub fn action_client_send_goal(
    client: &FfiActionClient,
    goal_json: &str,
) -> Result<Box<FfiGoalHandle>, String> {
    let _: serde_json::Value =
        serde_json::from_str(goal_json).map_err(|e| format!("Invalid goal JSON: {}", e))?;

    let goal_id = next_goal_id();

    if let Some(ref topic) = client.goal_topic {
        let wire = JsonWireMessage::from_json(goal_json, goal_id, MSG_GOAL)
            .ok_or_else(|| "Goal JSON too large (max 3968 bytes)".to_string())?;
        topic.send(wire);
    }

    Ok(Box::new(FfiGoalHandle {
        goal_id,
        action_name: client.name.clone(),
        status: FfiGoalStatus::Pending,
    }))
}

/// Request cancellation of a goal by id. Publishes on `{name}.cancel`; a running
/// server-side handler observes it via `is_cancel_requested()`.
pub fn action_client_cancel(client: &FfiActionClient, goal_id: u64) {
    if let Some(ref topic) = client.cancel_topic {
        if let Some(wire) = JsonWireMessage::from_json("null", goal_id, MSG_CANCEL) {
            topic.send(wire);
        }
    }
}

/// Poll for the next feedback message. Returns `(goal_id, json)` if one is
/// available, else `None`. Non-blocking.
pub fn action_client_poll_feedback(client: &FfiActionClient) -> Option<(u64, String)> {
    let topic = client.feedback_topic.as_ref()?;
    let wire = topic.recv()?;
    Some((wire.msg_id, wire.to_json()?))
}

/// Poll for a result for `goal_id`. Returns `(status, json)` once the goal has
/// finished, else `None`. Non-blocking; results for other goals seen while
/// polling are discarded (a client is expected to track one goal at a time).
pub fn action_client_poll_result(
    client: &FfiActionClient,
    goal_id: u64,
) -> Option<(FfiGoalStatus, String)> {
    let topic = client.result_topic.as_ref()?;
    while let Some(wire) = topic.recv() {
        if wire.msg_id == goal_id {
            let status = FfiGoalStatus::from_u8(wire.msg_type);
            return Some((status, wire.to_json().unwrap_or_else(|| "null".to_string())));
        }
    }
    None
}

/// Get goal status (client-side handle view).
pub fn goal_handle_status(handle: &FfiGoalHandle) -> FfiGoalStatus {
    handle.status
}

/// Get goal ID.
pub fn goal_handle_id(handle: &FfiGoalHandle) -> u64 {
    handle.goal_id
}

/// Check if the goal is still active (not terminal) per the handle's view.
pub fn goal_handle_is_active(handle: &FfiGoalHandle) -> bool {
    handle.status.is_active()
}

// ─── Server-side Goal Handle ─────────────────────────────────────────────────

/// Handle passed to the execute handler (runs on the goal thread). Mirrors the
/// Rust `ServerGoalHandle`: check cancellation, publish feedback, finish. It
/// never touches a topic — feedback/completion go into the shared event inbox
/// that the process() thread drains.
pub struct FfiActionGoalHandle {
    goal_id: u64,
    cancel_flag: Arc<AtomicBool>,
    events: EventInbox,
    finished: bool,
}

impl FfiActionGoalHandle {
    fn push_completion(&mut self, status: FfiGoalStatus, result_json: &str) {
        // Exactly one completion per goal: first finish wins.
        if self.finished {
            return;
        }
        self.finished = true;
        let wire = JsonWireMessage::from_json(result_json, self.goal_id, status as u8)
            .or_else(|| {
                // Oversize result: still terminate the goal so the client isn't
                // left hanging; deliver a null payload with the same status.
                JsonWireMessage::from_json("null", self.goal_id, status as u8)
            });
        if let Some(wire) = wire {
            self.events
                .lock()
                .unwrap()
                .push(FfiServerEvent::Completed(self.goal_id, status, wire));
        }
    }
}

/// Whether cancellation (or preemption) was requested for this goal.
pub fn action_goal_is_cancel_requested(handle: &FfiActionGoalHandle) -> bool {
    handle.cancel_flag.load(Ordering::Acquire)
}

/// Goal id for this handle.
pub fn action_goal_id(handle: &FfiActionGoalHandle) -> u64 {
    handle.goal_id
}

/// Publish feedback for this goal. Returns false if the JSON is oversize
/// (>3968 bytes) and was dropped. Feedback is delivered by the process() thread.
pub fn action_goal_publish_feedback(handle: &FfiActionGoalHandle, feedback_json: &str) -> bool {
    match JsonWireMessage::from_json(feedback_json, handle.goal_id, MSG_FEEDBACK) {
        Some(wire) => {
            handle
                .events
                .lock()
                .unwrap()
                .push(FfiServerEvent::Feedback(wire));
            true
        }
        None => false,
    }
}

/// Finish the goal as succeeded with a result payload.
pub fn action_goal_succeed(handle: &mut FfiActionGoalHandle, result_json: &str) {
    handle.push_completion(FfiGoalStatus::Succeeded, result_json);
}

/// Finish the goal as aborted.
pub fn action_goal_abort(handle: &mut FfiActionGoalHandle, result_json: &str) {
    handle.push_completion(FfiGoalStatus::Aborted, result_json);
}

/// Finish the goal as canceled (use in response to a cancel request).
pub fn action_goal_canceled(handle: &mut FfiActionGoalHandle, result_json: &str) {
    handle.push_completion(FfiGoalStatus::Canceled, result_json);
}

// ─── Action Server FFI ───────────────────────────────────────────────────────

type ExecuteFn = Arc<dyn Fn(&mut FfiActionGoalHandle, &[u8]) + Send + Sync>;

/// Opaque action server. Receives goals, runs each on its own thread, publishes
/// feedback/result/status, and delivers cancellations.
#[allow(dead_code, clippy::type_complexity)]
pub struct FfiActionServer {
    name: String,
    goal_topic: Option<Topic<JsonWireMessage>>,
    cancel_topic: Option<Topic<JsonWireMessage>>,
    feedback_topic: Option<Topic<JsonWireMessage>>,
    result_topic: Option<Topic<JsonWireMessage>>,
    status_topic: Option<Topic<JsonWireMessage>>,
    accept_handler: Option<Box<dyn Fn(&[u8]) -> FfiGoalResponse + Send + Sync>>,
    execute_handler: Option<ExecuteFn>,
    // Per-active-goal cancel flags (shared with the goal thread's handle).
    cancel_flags: HashMap<u64, Arc<AtomicBool>>,
    // Live goal-execution threads, reaped on completion / shutdown.
    threads: HashMap<u64, JoinHandle<()>>,
    // Feedback/completion pushed by goal threads, drained by `process()`.
    events: EventInbox,
}

/// Create a new action server with Pod-based topic wiring.
pub fn action_server_new(name: &str) -> Box<FfiActionServer> {
    Box::new(FfiActionServer {
        name: name.to_string(),
        goal_topic: Topic::<JsonWireMessage>::new(format!("{}.goal", name)).ok(),
        cancel_topic: Topic::<JsonWireMessage>::new(format!("{}.cancel", name)).ok(),
        feedback_topic: Topic::<JsonWireMessage>::new(format!("{}.feedback", name)).ok(),
        result_topic: Topic::<JsonWireMessage>::new(format!("{}.result", name)).ok(),
        status_topic: Topic::<JsonWireMessage>::new(format!("{}.status", name)).ok(),
        accept_handler: None,
        execute_handler: None,
        cancel_flags: HashMap::new(),
        threads: HashMap::new(),
        events: Arc::new(Mutex::new(Vec::new())),
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
        if handler(goal_bytes.as_ptr(), goal_bytes.len()) == 0 {
            FfiGoalResponse::Accept
        } else {
            FfiGoalResponse::Reject
        }
    }));
}

/// Set the goal execution handler.
///
/// `handler(goal_handle, goal_bytes, goal_len)` — called on a dedicated thread
/// when a goal is accepted. The handler should periodically check
/// `is_cancel_requested()`, publish feedback, and finish with succeed/abort/
/// cancel. If it returns without finishing, the goal is completed as Aborted.
pub fn action_server_set_execute_handler(
    server: &mut FfiActionServer,
    handler: extern "C" fn(*mut FfiActionGoalHandle, *const u8, usize),
) {
    server.execute_handler = Some(Arc::new(
        move |handle: &mut FfiActionGoalHandle, goal_bytes: &[u8]| {
            handler(
                handle as *mut FfiActionGoalHandle,
                goal_bytes.as_ptr(),
                goal_bytes.len(),
            );
        },
    ));
}

/// Check if the server has both handlers set.
pub fn action_server_is_ready(server: &FfiActionServer) -> bool {
    server.accept_handler.is_some() && server.execute_handler.is_some()
}

/// Drive the server once: publish any feedback/results produced by goal threads,
/// accept/reject newly received goals (spawning a thread per accepted goal), and
/// deliver cancellations. Call this repeatedly from a single thread — the same
/// one the server was constructed on.
pub fn action_server_process(server: &mut FfiActionServer) {
    // 1. Drain events from goal threads and publish them (this thread owns all
    //    topic I/O).
    let drained: Vec<FfiServerEvent> = {
        let mut ev = server.events.lock().unwrap();
        std::mem::take(&mut *ev)
    };
    for event in drained {
        match event {
            FfiServerEvent::Feedback(wire) => {
                if let Some(ref t) = server.feedback_topic {
                    t.send(wire);
                }
            }
            FfiServerEvent::Completed(goal_id, status, wire) => {
                if let Some(ref t) = server.result_topic {
                    t.send(wire);
                }
                publish_status(server, goal_id, status);
                if let Some(join) = server.threads.remove(&goal_id) {
                    let _ = join.join();
                }
                server.cancel_flags.remove(&goal_id);
            }
        }
    }

    // 2. Receive and dispatch new goals.
    let goals: Vec<JsonWireMessage> = match server.goal_topic {
        Some(ref t) => std::iter::from_fn(|| t.recv()).collect(),
        None => Vec::new(),
    };
    for wire in goals {
        let goal_id = wire.msg_id;
        let goal_json = wire.to_json().unwrap_or_else(|| "null".to_string());
        let goal_bytes = goal_json.into_bytes();

        let accept = match server.accept_handler {
            Some(ref h) => h(&goal_bytes),
            None => FfiGoalResponse::Accept,
        };
        if accept == FfiGoalResponse::Reject {
            publish_status(server, goal_id, FfiGoalStatus::Rejected);
            continue;
        }

        let execute = match server.execute_handler {
            Some(ref e) => e.clone(),
            None => {
                // No executor: nothing can run the goal; report Aborted.
                publish_status(server, goal_id, FfiGoalStatus::Aborted);
                continue;
            }
        };

        let cancel_flag = Arc::new(AtomicBool::new(false));
        server.cancel_flags.insert(goal_id, cancel_flag.clone());
        publish_status(server, goal_id, FfiGoalStatus::Active);

        let events = server.events.clone();
        let join = std::thread::Builder::new()
            .name(format!("horus-cpp-action-{}", server.name))
            .spawn(move || {
                let mut handle = FfiActionGoalHandle {
                    goal_id,
                    cancel_flag,
                    events,
                    finished: false,
                };
                execute(&mut handle, &goal_bytes);
                // If the handler returned without finishing, default to Aborted.
                handle.push_completion(FfiGoalStatus::Aborted, "null");
            });
        match join {
            Ok(j) => {
                server.threads.insert(goal_id, j);
            }
            Err(_) => {
                server.cancel_flags.remove(&goal_id);
                publish_status(server, goal_id, FfiGoalStatus::Aborted);
            }
        }
    }

    // 3. Deliver cancellations to running goals.
    let cancels: Vec<JsonWireMessage> = match server.cancel_topic {
        Some(ref t) => std::iter::from_fn(|| t.recv()).collect(),
        None => Vec::new(),
    };
    for wire in cancels {
        if let Some(flag) = server.cancel_flags.get(&wire.msg_id) {
            flag.store(true, Ordering::Release);
        }
    }
}

fn publish_status(server: &FfiActionServer, goal_id: u64, status: FfiGoalStatus) {
    if let Some(ref t) = server.status_topic {
        if let Some(wire) = JsonWireMessage::from_json("null", goal_id, status as u8) {
            t.send(wire);
        }
    }
}

impl Drop for FfiActionServer {
    fn drop(&mut self) {
        // Cancel every in-flight goal and join its thread so none outlives the
        // server. A cooperative handler returns promptly; a non-cooperative one
        // (never checks is_cancel_requested) will block here — same contract as
        // the Rust action server and ROS.
        for flag in self.cancel_flags.values() {
            flag.store(true, Ordering::Release);
        }
        for (_id, join) in self.threads.drain() {
            let _ = join.join();
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    fn drive_for<F: FnMut() -> bool>(
        server: &mut FfiActionServer,
        mut done: F,
        timeout: Duration,
    ) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            action_server_process(server);
            if done() {
                return true;
            }
            if Instant::now() > deadline {
                return false;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn goal_status_terminal() {
        assert!(!FfiGoalStatus::Pending.is_terminal());
        assert!(FfiGoalStatus::Succeeded.is_terminal());
        assert!(FfiGoalStatus::Canceled.is_terminal());
    }

    #[test]
    fn ids_are_unique_across_clients() {
        let name = format!("act_ids.{}", std::process::id());
        let c1 = action_client_new(&name);
        let c2 = action_client_new(&name);
        let a = action_client_send_goal(&c1, "{}").unwrap();
        let b = action_client_send_goal(&c2, "{}").unwrap();
        assert_ne!(
            goal_handle_id(&a),
            goal_handle_id(&b),
            "goal ids must be unique across clients"
        );
    }

    #[test]
    fn send_goal_validates_json() {
        let client = action_client_new("act_json_validate");
        assert!(action_client_send_goal(&client, "not json").is_err());
    }

    #[test]
    fn server_processes_goal_end_to_end() {
        static RAN: AtomicBool = AtomicBool::new(false);
        RAN.store(false, Ordering::SeqCst);

        extern "C" fn accept(_: *const u8, _: usize) -> u8 {
            0
        }
        extern "C" fn execute(h: *mut FfiActionGoalHandle, _: *const u8, _: usize) {
            RAN.store(true, Ordering::SeqCst);
            let handle = unsafe { &mut *h };
            action_goal_publish_feedback(handle, r#"{"progress":0.5}"#);
            action_goal_succeed(handle, r#"{"ok":true}"#);
        }

        let name = format!("act_e2e.{}", std::process::id());
        let mut server = action_server_new(&name);
        action_server_set_accept_handler(&mut server, accept);
        action_server_set_execute_handler(&mut server, execute);
        assert!(action_server_is_ready(&server));

        let client = action_client_new(&name);
        std::thread::sleep(Duration::from_millis(50));
        let handle = action_client_send_goal(&client, r#"{"x":1.0}"#).unwrap();
        let goal_id = goal_handle_id(&handle);

        let got = drive_for(
            &mut server,
            || action_client_poll_result(&client, goal_id).is_some() || RAN.load(Ordering::SeqCst),
            Duration::from_secs(3),
        );
        assert!(got, "goal should have executed and produced a result");
        assert!(RAN.load(Ordering::SeqCst), "execute handler must have run");
    }

    // The key real-robot test: a cancel sent over the topic must reach a goal
    // that is CURRENTLY executing on its thread.
    #[test]
    fn cancel_reaches_a_running_goal() {
        static STARTED: AtomicBool = AtomicBool::new(false);
        static SAW_CANCEL: AtomicBool = AtomicBool::new(false);
        static HIT_GUARD: AtomicBool = AtomicBool::new(false);
        STARTED.store(false, Ordering::SeqCst);
        SAW_CANCEL.store(false, Ordering::SeqCst);
        HIT_GUARD.store(false, Ordering::SeqCst);

        extern "C" fn accept(_: *const u8, _: usize) -> u8 {
            0
        }
        extern "C" fn execute(h: *mut FfiActionGoalHandle, _: *const u8, _: usize) {
            let handle = unsafe { &mut *h };
            STARTED.store(true, Ordering::SeqCst);
            for _ in 0..500 {
                if action_goal_is_cancel_requested(handle) {
                    SAW_CANCEL.store(true, Ordering::SeqCst);
                    action_goal_canceled(handle, r#"{"canceled":true}"#);
                    return;
                }
                std::thread::sleep(Duration::from_millis(3));
            }
            HIT_GUARD.store(true, Ordering::SeqCst);
            action_goal_succeed(handle, r#"{"canceled":false}"#);
        }

        let name = format!("act_cancel.{}", std::process::id());
        let mut server = action_server_new(&name);
        action_server_set_accept_handler(&mut server, accept);
        action_server_set_execute_handler(&mut server, execute);

        let client = action_client_new(&name);
        std::thread::sleep(Duration::from_millis(50));
        let handle = action_client_send_goal(&client, "{}").unwrap();
        let goal_id = goal_handle_id(&handle);

        // Drive until the goal is actually running.
        assert!(
            drive_for(
                &mut server,
                || STARTED.load(Ordering::SeqCst),
                Duration::from_secs(3)
            ),
            "goal never started executing"
        );

        // Cancel it over the topic while it runs.
        action_client_cancel(&client, goal_id);

        // Keep driving so process() delivers the cancel; the handler must exit
        // via cancellation, not by running out its guard.
        assert!(
            drive_for(
                &mut server,
                || SAW_CANCEL.load(Ordering::SeqCst) || HIT_GUARD.load(Ordering::SeqCst),
                Duration::from_secs(3)
            ),
            "handler never observed the cancel"
        );
        assert!(
            SAW_CANCEL.load(Ordering::SeqCst),
            "running goal must observe the topic cancel"
        );
        assert!(
            !HIT_GUARD.load(Ordering::SeqCst),
            "goal ran to its guard instead of being canceled"
        );
    }

    #[test]
    fn handler_that_never_finishes_is_aborted() {
        extern "C" fn accept(_: *const u8, _: usize) -> u8 {
            0
        }
        // Returns immediately without calling succeed/abort/cancel.
        extern "C" fn execute(_: *mut FfiActionGoalHandle, _: *const u8, _: usize) {}

        let name = format!("act_default_abort.{}", std::process::id());
        let mut server = action_server_new(&name);
        action_server_set_accept_handler(&mut server, accept);
        action_server_set_execute_handler(&mut server, execute);

        let client = action_client_new(&name);
        std::thread::sleep(Duration::from_millis(50));
        let handle = action_client_send_goal(&client, "{}").unwrap();
        let goal_id = goal_handle_id(&handle);

        let mut result = None;
        drive_for(
            &mut server,
            || {
                result = action_client_poll_result(&client, goal_id);
                result.is_some()
            },
            Duration::from_secs(3),
        );
        assert_eq!(
            result.map(|(s, _)| s),
            Some(FfiGoalStatus::Aborted),
            "a handler that never finishes must complete the goal as Aborted"
        );
    }
}
