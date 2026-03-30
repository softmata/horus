#![allow(dead_code)]
// Concurrent goal handling and feedback rate limiting tests for ActionServer.
//
// These tests exercise:
// - Sequential goal processing with correct result propagation
// - Server metric counting across mixed outcomes
// - Feedback publication during execution
// - Cooperative cancellation during execution
// - Goal ID uniqueness across multiple goals
//
// Architecture notes (same as action_runtime.rs):
// - ActionServerNode implements Node trait (init creates topics, tick processes goals)
// - Server's execute callback is SYNCHRONOUS — runs to completion in one tick
// - SyncActionClient polls topics directly (doesn't need a scheduler)
// - Server MUST be ticked for goals to be processed
// - Because execute blocks the server thread, cancel requests sent via topics
//   during execution cannot be processed until the NEXT tick. Cancellation is
//   tested via cooperative patterns (shared AtomicBool checked inside execute).

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

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
// Action types — unique per test to avoid SHM collisions
// ============================================================================

action! {
    SeqDouble {
        goal { value: u64 }
        feedback { step: u32 }
        result { doubled: u64 }
    }
}

action! {
    MetricsMixed {
        goal { mode: u64 }
        feedback { step: u32 }
        result { outcome_code: u64 }
    }
}

action! {
    FbPublish {
        goal { count: u32 }
        feedback { index: u32 }
        result { total_sent: u32 }
    }
}

action! {
    CancelDuring {
        goal { tag: u64 }
        feedback { iteration: u32 }
        result { was_canceled: bool }
    }
}

action! {
    IdUnique {
        goal { seq: u64 }
        feedback { step: u32 }
        result { seq: u64 }
    }
}

// ============================================================================
// Helper: run a server in a background thread
// ============================================================================

fn run_server_background<N: Node + Send + 'static>(
    server: N,
) -> (Arc<AtomicBool>, std::thread::JoinHandle<()>) {
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(5));
        }
    });
    (running, handle)
}

fn stop_server(running: Arc<AtomicBool>, handle: std::thread::JoinHandle<()>) {
    running.store(false, Ordering::Relaxed);
    let _ = handle.join();
}

// ============================================================================
// Test 1: Sequential goals all complete correctly
//
// Send 10 goals sequentially to the same server. Each returns its input * 2.
// Assert all 10 results are correct. This tests the server properly resets
// between goals and does not leak state from one execution into the next.
// ============================================================================

#[test]
fn test_sequential_goals_all_complete_correctly() {
    serial_setup!();

    let server = ActionServerBuilder::<SeqDouble>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            let input = handle.goal().value;
            handle.succeed(SeqDoubleResult { doubled: input * 2 })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<SeqDouble>::new().unwrap();

    for i in 0..10 {
        let result = client.send_goal_and_wait(SeqDoubleGoal { value: i }, Duration::from_secs(3));
        match result {
            Ok(r) => assert_eq!(
                r.doubled,
                i * 2,
                "Goal {} should return {} but got {}",
                i,
                i * 2,
                r.doubled
            ),
            Err(e) => panic!("Goal {} should succeed but got error: {:?}", i, e),
        }
    }

    stop_server(running, server_handle);
}

// ============================================================================
// Test 2: Goal metrics counted accurately
//
// Send 3 goals: one succeeds, one is rejected (via on_goal returning Reject),
// one aborts. After all complete, verify the metric counters via AtomicU64
// tracking in the callbacks (the ActionServerNode is consumed by the
// scheduler and not accessible for metrics() across threads).
//
// Metric expectations:
//   goals dispatched = 3 (all received by on_goal callback)
//   goals accepted   = 2 (succeed + abort)
//   goals rejected   = 1 (mode == 99)
//   goals succeeded  = 1 (mode == 1)
//   goals aborted    = 1 (mode == 2)
// ============================================================================

#[test]
fn test_goal_metrics_counted_accurately() {
    serial_setup!();

    let received = Arc::new(AtomicU64::new(0));
    let accepted = Arc::new(AtomicU64::new(0));
    let rejected = Arc::new(AtomicU64::new(0));
    let succeeded = Arc::new(AtomicU64::new(0));
    let aborted = Arc::new(AtomicU64::new(0));

    let received_c = received.clone();
    let accepted_c = accepted.clone();
    let rejected_c = rejected.clone();
    let succeeded_c = succeeded.clone();
    let aborted_c = aborted.clone();

    let server = ActionServerBuilder::<MetricsMixed>::new()
        .on_goal(move |goal| {
            received_c.fetch_add(1, Ordering::SeqCst);
            if goal.mode == 99 {
                rejected_c.fetch_add(1, Ordering::SeqCst);
                GoalResponse::Reject("mode 99 not allowed".to_string())
            } else {
                accepted_c.fetch_add(1, Ordering::SeqCst);
                GoalResponse::Accept
            }
        })
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| match handle.goal().mode {
            1 => {
                succeeded_c.fetch_add(1, Ordering::SeqCst);
                handle.succeed(MetricsMixedResult { outcome_code: 1 })
            }
            2 => {
                aborted_c.fetch_add(1, Ordering::SeqCst);
                handle.abort(MetricsMixedResult { outcome_code: 2 })
            }
            _ => handle.succeed(MetricsMixedResult { outcome_code: 0 }),
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<MetricsMixed>::new().unwrap();

    // Goal 1: mode=1 -> succeed
    let r1 = client.send_goal_and_wait(MetricsMixedGoal { mode: 1 }, Duration::from_secs(3));
    assert!(r1.is_ok(), "Mode 1 should succeed: {:?}", r1);
    assert_eq!(r1.unwrap().outcome_code, 1);

    // Goal 2: mode=99 -> rejected by on_goal callback
    let r2 = client.send_goal_and_wait(MetricsMixedGoal { mode: 99 }, Duration::from_secs(3));
    assert!(r2.is_err(), "Mode 99 should be rejected: {:?}", r2);

    // Goal 3: mode=2 -> abort
    let r3 = client.send_goal_and_wait(MetricsMixedGoal { mode: 2 }, Duration::from_secs(3));
    assert!(r3.is_err(), "Mode 2 should abort: {:?}", r3);

    stop_server(running, server_handle);

    // Verify metrics
    assert_eq!(
        received.load(Ordering::SeqCst),
        3,
        "Should have received 3 goals"
    );
    assert_eq!(
        accepted.load(Ordering::SeqCst),
        2,
        "Should have accepted 2 goals (succeed + abort)"
    );
    assert_eq!(
        rejected.load(Ordering::SeqCst),
        1,
        "Should have rejected 1 goal"
    );
    assert_eq!(
        succeeded.load(Ordering::SeqCst),
        1,
        "Should have 1 succeeded goal"
    );
    assert_eq!(
        aborted.load(Ordering::SeqCst),
        1,
        "Should have 1 aborted goal"
    );
}

// ============================================================================
// Test 3: Feedback published during execution
//
// Server publishes 5 feedback messages during execute. Client uses
// send_goal_and_wait_with_feedback to receive intermediate feedback via
// a callback. Assert at least some feedback messages are received.
//
// NOTE: Because execute is synchronous (all 5 feedbacks are published in
// one tick before the result), the client's poll loop may not see every
// individual feedback. We verify at least 1 is received.
// ============================================================================

#[test]
fn test_feedback_published_during_execution() {
    serial_setup!();

    let server = ActionServerBuilder::<FbPublish>::new()
        // Use a very high feedback rate so none are dropped by rate limiting
        .feedback_rate(10000.0)
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            let count = handle.goal().count;
            for i in 0..count {
                handle.publish_feedback(FbPublishFeedback { index: i });
                // Small pause between feedbacks to give rate limiter headroom
                std::thread::sleep(Duration::from_millis(1));
            }
            handle.succeed(FbPublishResult { total_sent: count })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    let feedback_count = Arc::new(AtomicU64::new(0));
    let fb_count = feedback_count.clone();

    let client = SyncActionClient::<FbPublish>::new().unwrap();
    let result = client.send_goal_and_wait_with_feedback(
        FbPublishGoal { count: 5 },
        Duration::from_secs(5),
        move |_fb| {
            fb_count.fetch_add(1, Ordering::SeqCst);
        },
    );

    stop_server(running, server_handle);

    assert!(result.is_ok(), "Goal should succeed: {:?}", result);
    assert_eq!(
        result.unwrap().total_sent,
        5,
        "Server should report sending 5 feedbacks"
    );

    let fb_received = feedback_count.load(Ordering::SeqCst);
    assert!(
        fb_received >= 1,
        "Should receive at least 1 feedback, got {}",
        fb_received
    );
}

// ============================================================================
// Test 4: Cancel during execution (cooperative cancel pattern)
//
// The execute callback runs synchronously on the server thread, so cancel
// requests sent via topics cannot be processed until execute returns. This
// test uses a shared AtomicBool (external cancel signal) to simulate the
// cooperative cancellation pattern: the execute callback checks both
// handle.is_cancel_requested() and the external flag on each iteration.
//
// A background thread sets the external cancel flag after 300ms. The execute
// callback detects it, exits its loop early, and returns handle.canceled().
// The client observes GoalCanceled error — confirming the full cancel path
// from execute callback through result publication to client reception.
// ============================================================================

#[test]
fn test_cancel_during_execution() {
    serial_setup!();

    let external_cancel = Arc::new(AtomicBool::new(false));
    let cancel_flag = external_cancel.clone();

    let server = ActionServerBuilder::<CancelDuring>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            // Loop checking for cancel — up to 200 iterations of 10ms = 2s max
            for i in 0..200 {
                if handle.is_cancel_requested() || cancel_flag.load(Ordering::Acquire) {
                    return handle.canceled(CancelDuringResult { was_canceled: true });
                }
                handle.publish_feedback(CancelDuringFeedback { iteration: i });
                std::thread::sleep(Duration::from_millis(10));
            }

            // If we get here, cancel was never requested
            handle.succeed(CancelDuringResult {
                was_canceled: false,
            })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    // Send goal in a background thread (blocking call)
    let goal_thread = std::thread::spawn(move || {
        let client = SyncActionClient::<CancelDuring>::new().unwrap();
        client.send_goal_and_wait(CancelDuringGoal { tag: 42 }, Duration::from_secs(5))
    });

    // Wait for execution to start, then trigger the external cancel flag
    std::thread::sleep(Duration::from_millis(300));
    external_cancel.store(true, Ordering::Release);

    let result = goal_thread.join().unwrap();

    stop_server(running, server_handle);

    // The execute callback should have seen the external cancel flag and
    // returned handle.canceled(), so the client gets GoalCanceled.
    match result {
        Err(ActionError::GoalCanceled) => {} // Expected
        other => panic!("Expected GoalCanceled error, got: {:?}", other),
    }
}

// ============================================================================
// Test 5: Goal ID unique across goals
//
// Send 5 goals. Collect all goal IDs from the server's execute callback.
// Assert all 5 are unique (no reuse). This verifies that GoalId::new()
// generates distinct UUIDs for each request and the server does not confuse
// or reuse IDs across sequential goals.
// ============================================================================

#[test]
fn test_goal_id_unique_across_goals() {
    serial_setup!();

    let goal_ids = Arc::new(std::sync::Mutex::new(Vec::<GoalId>::new()));
    let ids_clone = goal_ids.clone();

    let server = ActionServerBuilder::<IdUnique>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            // Record the goal_id seen by the server
            ids_clone.lock().unwrap().push(handle.goal_id());
            let seq = handle.goal().seq;
            handle.succeed(IdUniqueResult { seq })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<IdUnique>::new().unwrap();

    // Send 5 goals and verify each result matches
    for i in 0..5 {
        let result = client.send_goal_and_wait(IdUniqueGoal { seq: i }, Duration::from_secs(3));
        assert!(result.is_ok(), "Goal {} should succeed: {:?}", i, result);
        assert_eq!(result.unwrap().seq, i, "Goal {} result seq mismatch", i);
    }

    stop_server(running, server_handle);

    // Verify all goal IDs are unique
    let ids = goal_ids.lock().unwrap();
    assert_eq!(ids.len(), 5, "Should have 5 goal IDs, got {}", ids.len());

    let unique_ids: HashSet<_> = ids.iter().collect();
    assert_eq!(
        unique_ids.len(),
        5,
        "All 5 goal IDs should be unique, but only {} are distinct",
        unique_ids.len()
    );

    // Pairwise uniqueness check for detailed failure messages
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            assert_ne!(
                ids[i], ids[j],
                "Goal IDs at index {} and {} should differ: {} vs {}",
                i, j, ids[i], ids[j]
            );
        }
    }
}
