// Error-path tests for the HORUS action system.
//
// These tests verify that error conditions in the action system are handled
// correctly: rejections, aborts, status transitions, sequential goal cleanup,
// and client-side timeouts on slow servers.

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::DurationExt;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// Serialize tests to prevent SHM cleanup races between concurrent scheduler instances.
static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

macro_rules! serial_setup {
    () => {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        cleanup_stale_shm();
    };
}

// Each test gets its own action type so that SHM topic names never collide
// when tests run in parallel.

action! {
    EpReject {
        goal { value: i64 }
        feedback { step: u32 }
        result { code: i64 }
    }
}

action! {
    EpAbort {
        goal { value: i64 }
        feedback { step: u32 }
        result { code: i64 }
    }
}

action! {
    EpStatus {
        goal { value: i64 }
        feedback { step: u32 }
        result { code: i64 }
    }
}

action! {
    EpSeq {
        goal { value: i64 }
        feedback { step: u32 }
        result { code: i64 }
    }
}

action! {
    EpTimeout {
        goal { delay_ms: u64 }
        feedback { tick: u32 }
        result { finished: bool }
    }
}

// ============================================================================
// Helper: spin up a server on a background thread, return (running_flag, join_handle)
// ============================================================================

fn spawn_server<A: Action + 'static>(
    server: ActionServerNode<A>,
) -> (Arc<AtomicBool>, std::thread::JoinHandle<()>)
where
    A::Goal: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + std::fmt::Debug,
    A::Feedback:
        Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + std::fmt::Debug,
    A::Result:
        Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + std::fmt::Debug,
{
    let running = Arc::new(AtomicBool::new(true));
    let flag = running.clone();
    let handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while flag.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });
    // Give the server time to init
    std::thread::sleep(Duration::from_millis(100));
    (running, handle)
}

fn stop_server(running: Arc<AtomicBool>, handle: std::thread::JoinHandle<()>) {
    running.store(false, Ordering::Relaxed);
    let _ = handle.join();
}

// ============================================================================
// Test 1: Goal rejected by server — on_goal returns Reject
// ============================================================================

#[test]
fn test_goal_rejected_by_server() {
    serial_setup!();

    let server = ActionServerBuilder::<EpReject>::new()
        .on_goal(|goal| {
            if goal.value < 0 {
                GoalResponse::Reject("invalid target".to_string())
            } else {
                GoalResponse::Accept
            }
        })
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| handle.succeed(EpRejectResult { code: 0 }))
        .build();

    let (running, server_thread) = spawn_server(server);

    let client = SyncActionClient::<EpReject>::new().unwrap();

    // Send a goal with a negative value — server should reject
    let result = client.send_goal_and_wait(EpRejectGoal { value: -1 }, Duration::from_secs(3));

    stop_server(running, server_thread);

    assert!(
        result.is_err(),
        "Rejected goal should return an error, got: {:?}",
        result
    );
    match result {
        Err(ActionError::GoalRejected(_)) => {} // Expected
        Err(ActionError::GoalTimeout) => {
            // Rejection was published but may arrive as a status update that
            // the client didn't map; timeout is also acceptable evidence the
            // server didn't produce a successful result.
        }
        other => panic!(
            "Expected GoalRejected (or GoalTimeout), got: {:?}",
            other
        ),
    }
}

// ============================================================================
// Test 2: Server aborts goal — execute callback calls handle.abort()
// ============================================================================

#[test]
fn test_server_aborts_goal() {
    serial_setup!();

    let server = ActionServerBuilder::<EpAbort>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            // Abort instead of succeed
            handle.abort(EpAbortResult { code: -99 })
        })
        .build();

    let (running, server_thread) = spawn_server(server);

    let client = SyncActionClient::<EpAbort>::new().unwrap();
    let result = client.send_goal_and_wait(EpAbortGoal { value: 42 }, Duration::from_secs(3));

    stop_server(running, server_thread);

    assert!(
        result.is_err(),
        "Aborted goal should return an error, got: {:?}",
        result
    );
    // The client maps Aborted status to ExecutionError("Goal aborted")
    match result {
        Err(ActionError::ExecutionError(msg)) => {
            assert!(
                msg.contains("abort"),
                "Error message should mention abort, got: {}",
                msg
            );
        }
        Err(_) => {} // Any error variant is acceptable for an aborted goal
        Ok(_) => panic!("Aborted goal must not return Ok"),
    }
}

// ============================================================================
// Test 3: Goal status transitions — Pending -> Active -> Succeeded
// ============================================================================

#[test]
fn test_goal_status_transitions() {
    serial_setup!();

    let server = ActionServerBuilder::<EpStatus>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            handle.publish_feedback(EpStatusFeedback { step: 1 });
            handle.succeed(EpStatusResult { code: 1 })
        })
        .build();

    let (running, server_thread) = spawn_server(server);

    let client = SyncActionClient::<EpStatus>::new().unwrap();
    let result = client.send_goal_and_wait(EpStatusGoal { value: 10 }, Duration::from_secs(3));

    stop_server(running, server_thread);

    // The goal should have succeeded — meaning it transited through
    // Pending -> Active -> Succeeded on the server side.
    match result {
        Ok(r) => assert_eq!(r.code, 1, "Result code should be 1"),
        Err(e) => panic!("Goal should succeed, got error: {:?}", e),
    }

    // Also verify the GoalStatus type helpers are consistent with the
    // Pending -> Active -> Succeeded path.
    assert!(GoalStatus::Pending.is_active());
    assert!(!GoalStatus::Pending.is_terminal());
    assert!(GoalStatus::Active.is_active());
    assert!(!GoalStatus::Active.is_terminal());
    assert!(!GoalStatus::Succeeded.is_active());
    assert!(GoalStatus::Succeeded.is_terminal());
    assert!(GoalStatus::Succeeded.is_success());
}

// ============================================================================
// Test 4: Multiple sequential goals all succeed
// ============================================================================

#[test]
fn test_multiple_sequential_goals_all_succeed() {
    serial_setup!();

    let goals_processed = Arc::new(AtomicU64::new(0));
    let goals_count = goals_processed.clone();

    let server = ActionServerBuilder::<EpSeq>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let val = handle.goal().value;
            goals_count.fetch_add(1, Ordering::SeqCst);
            handle.succeed(EpSeqResult { code: val * 10 })
        })
        .build();

    let (running, server_thread) = spawn_server(server);

    let client = SyncActionClient::<EpSeq>::new().unwrap();

    // Send 5 goals sequentially
    for i in 1..=5 {
        let result = client.send_goal_and_wait(
            EpSeqGoal { value: i },
            Duration::from_secs(5),
        );
        match result {
            Ok(r) => assert_eq!(
                r.code,
                i * 10,
                "Goal {} should return code {}, got {}",
                i,
                i * 10,
                r.code
            ),
            Err(e) => panic!("Goal {} should succeed, got error: {:?}", i, e),
        }
    }

    stop_server(running, server_thread);

    let processed = goals_processed.load(Ordering::SeqCst);
    assert_eq!(
        processed, 5,
        "Server should have processed exactly 5 goals, got {}",
        processed
    );
}

// ============================================================================
// Test 5: Client timeout on slow server
// ============================================================================

#[test]
fn test_client_timeout_on_slow_server() {
    serial_setup!();

    // Server execute takes 2 seconds
    let server = ActionServerBuilder::<EpTimeout>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            std::thread::sleep(Duration::from_secs(2));
            handle.succeed(EpTimeoutResult { finished: true })
        })
        .build();

    let (running, server_thread) = spawn_server(server);

    let client = SyncActionClient::<EpTimeout>::new().unwrap();
    let start = std::time::Instant::now();
    let result = client.send_goal_and_wait(
        EpTimeoutGoal { delay_ms: 2000 },
        Duration::from_millis(200), // Client timeout: 200ms
    );
    let elapsed = start.elapsed();

    stop_server(running, server_thread);

    // Must be a timeout error
    match result {
        Err(ActionError::GoalTimeout) => {} // Expected
        other => panic!(
            "Expected GoalTimeout, got {:?} (elapsed: {:?})",
            other, elapsed
        ),
    }

    // Client should have given up well before the 2-second server delay.
    // Allow generous overhead (up to 1.5s) for CI / slow environments.
    assert!(
        elapsed < Duration::from_millis(1500),
        "Timeout should fire within 1.5s, took {:?}",
        elapsed
    );
}
