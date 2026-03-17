// Runtime integration tests for ActionServer + SyncActionClient.
//
// These tests PROVE the action system works end-to-end: goals are sent,
// processed by the server, feedback is published, and results are received.
// All previous action tests (94) are type-level — these are the first
// tests that actually run the server processing goals.
//
// Architecture:
// - ActionServerNode implements Node trait (init creates topics, tick processes goals)
// - Server's execute callback is SYNCHRONOUS — runs to completion in one tick
// - SyncActionClient polls topics directly (doesn't need a scheduler)
// - Server MUST be ticked for goals to be processed

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

action! {
    RtNav {
        goal { target: f64 }
        feedback { progress: f32 }
        result { done: bool }
    }
}

action! {
    RtSlow {
        goal { delay_ms: u64 }
        feedback { step: u32 }
        result { completed: bool }
    }
}

// ============================================================================
// Test: Full goal lifecycle — send goal → server accepts → succeeds → client gets result
// ============================================================================

#[test]
fn test_action_full_goal_lifecycle() {
    cleanup_stale_shm();

    // Build server node
    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            // Publish one feedback
            handle.publish_feedback(RtNavFeedback { progress: 1.0 });
            // Succeed
            handle.succeed(RtNavResult { done: true })
        })
        .build();

    // Run server in scheduler on background thread
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        // Run until signaled to stop
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    // Give server time to init
    std::thread::sleep(Duration::from_millis(100));

    // Create client and send goal
    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(
        RtNavGoal { target: 42.0 },
        Duration::from_secs(5),
    );

    // Stop server
    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    // Verify result
    match result {
        Ok(r) => assert!(r.done, "Goal should succeed with done=true"),
        Err(e) => panic!("Goal should succeed but got error: {:?}", e),
    }
}

// ============================================================================
// Test: Server rejects goal — client gets GoalRejected error
// ============================================================================

#[test]
fn test_action_server_rejects_goal() {
    cleanup_stale_shm();

    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Reject("invalid target".to_string()))
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| handle.succeed(RtNavResult { done: false }))
        .build();

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(
        RtNavGoal { target: -999.0 },
        Duration::from_secs(3),
    );

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    assert!(
        result.is_err(),
        "Rejected goal should return error, got: {:?}",
        result
    );
}

// ============================================================================
// Test: Server aborts goal — client gets error
// ============================================================================

#[test]
fn test_action_server_aborts_goal() {
    cleanup_stale_shm();

    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            // Server decides to abort
            handle.abort(RtNavResult { done: false })
        })
        .build();

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(
        RtNavGoal { target: 10.0 },
        Duration::from_secs(3),
    );

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    assert!(
        result.is_err(),
        "Aborted goal should return error, got: {:?}",
        result
    );
}

// ============================================================================
// Test: Client timeout — server too slow, client times out
// ============================================================================

#[test]
fn test_action_client_timeout() {
    cleanup_stale_shm();

    // Server that takes 5 seconds (but client timeout is 500ms)
    let server = ActionServerBuilder::<RtSlow>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            // Simulate slow execution
            std::thread::sleep(Duration::from_secs(5));
            handle.succeed(RtSlowResult { completed: true })
        })
        .build();

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<RtSlow>::new().unwrap();
    let start = std::time::Instant::now();
    let result = client.send_goal_and_wait(
        RtSlowGoal { delay_ms: 5000 },
        Duration::from_millis(500), // Short timeout
    );
    let elapsed = start.elapsed();

    running.store(false, Ordering::Relaxed);
    let _ = server_thread.join();

    // Should timeout
    match result {
        Err(ActionError::GoalTimeout) => {} // Expected
        other => panic!(
            "Expected GoalTimeout, got {:?} (elapsed: {:?})",
            other, elapsed
        ),
    }

    // Should timeout within ~1s (500ms timeout + polling overhead)
    assert!(
        elapsed < Duration::from_secs(2),
        "Timeout should fire within 2s, took {:?}",
        elapsed
    );
}

// ============================================================================
// Test: Feedback callback receives feedback from server
// ============================================================================

#[test]
fn test_action_feedback_received() {
    cleanup_stale_shm();

    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            // Publish multiple feedbacks
            for i in 0..5 {
                handle.publish_feedback(RtNavFeedback {
                    progress: (i + 1) as f32 * 0.2,
                });
            }
            handle.succeed(RtNavResult { done: true })
        })
        .build();

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    std::thread::sleep(Duration::from_millis(100));

    let feedback_count = Arc::new(AtomicU64::new(0));
    let fb_count = feedback_count.clone();

    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait_with_feedback(
        RtNavGoal { target: 100.0 },
        Duration::from_secs(5),
        move |_fb| {
            fb_count.fetch_add(1, Ordering::SeqCst);
        },
    );

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    assert!(result.is_ok(), "Goal should succeed: {:?}", result);

    let fb_received = feedback_count.load(Ordering::SeqCst);
    // At least some feedback should be received (timing-dependent)
    // Server publishes 5 feedbacks but client may not poll fast enough to see all
    assert!(
        fb_received >= 1,
        "Should receive at least 1 feedback, got {}",
        fb_received
    );
}

// ============================================================================
// Test: Multiple sequential goals — server handles them in order
// ============================================================================

#[test]
fn test_action_multiple_sequential_goals() {
    cleanup_stale_shm();

    let goals_processed = Arc::new(AtomicU64::new(0));
    let goals_count = goals_processed.clone();

    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            goals_count.fetch_add(1, Ordering::SeqCst);
            handle.succeed(RtNavResult { done: true })
        })
        .build();

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<RtNav>::new().unwrap();

    // Send 3 goals sequentially
    for i in 0..3 {
        let result = client.send_goal_and_wait(
            RtNavGoal { target: i as f64 },
            Duration::from_secs(3),
        );
        assert!(result.is_ok(), "Goal {} should succeed: {:?}", i, result);
    }

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    let processed = goals_processed.load(Ordering::SeqCst);
    assert_eq!(
        processed, 3,
        "Server should process exactly 3 goals, got {}",
        processed
    );
}
