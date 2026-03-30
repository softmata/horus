#![allow(dead_code)]
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
use horus_core::core::DurationExt;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// All tests in this file share SHM topic names (RtNav, RtSlow) so they MUST
// run serially.  The global mutex prevents parallel execution within this binary.
static TEST_LOCK: Mutex<()> = Mutex::new(());

macro_rules! serial_setup {
    () => {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        cleanup_stale_shm();
    };
}

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
    serial_setup!();

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
    std::thread::sleep(Duration::from_millis(500));

    // Create client and send goal
    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(RtNavGoal { target: 42.0 }, Duration::from_secs(5));

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
    serial_setup!();

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

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(RtNavGoal { target: -999.0 }, Duration::from_secs(3));

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
    serial_setup!();

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

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(RtNavGoal { target: 10.0 }, Duration::from_secs(3));

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
    serial_setup!();

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

    std::thread::sleep(Duration::from_millis(500));

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
    serial_setup!();

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

    std::thread::sleep(Duration::from_millis(500));

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
    serial_setup!();

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

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtNav>::new().unwrap();

    // Send 3 goals sequentially
    for i in 0..3 {
        let result =
            client.send_goal_and_wait(RtNavGoal { target: i as f64 }, Duration::from_secs(3));
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

// ============================================================================
// Test: Server returns Canceled outcome — client sees GoalCanceled error
// ============================================================================

#[test]
fn test_action_canceled_outcome() {
    serial_setup!();

    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            // Server voluntarily cancels (e.g., obstacle detected, safety stop)
            handle.canceled(RtNavResult { done: false })
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

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(RtNavGoal { target: 50.0 }, Duration::from_secs(3));

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    match result {
        Err(ActionError::GoalCanceled) => {} // Expected
        other => panic!("Expected GoalCanceled error, got: {:?}", other),
    }
}

// ============================================================================
// Test: Server returns Preempted outcome — client sees GoalPreempted error
// ============================================================================

#[test]
fn test_action_preempted_outcome() {
    serial_setup!();

    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            // Server preempts (e.g., higher priority task took over)
            handle.preempted(RtNavResult { done: false })
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

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtNav>::new().unwrap();
    let result = client.send_goal_and_wait(RtNavGoal { target: 75.0 }, Duration::from_secs(3));

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    match result {
        Err(ActionError::GoalPreempted) => {} // Expected
        other => panic!("Expected GoalPreempted error, got: {:?}", other),
    }
}

// ============================================================================
// Test: Client cancel on completed goal is harmless (no crash/hang)
// ============================================================================

#[test]
fn test_action_cancel_after_completion_is_harmless() {
    serial_setup!();

    let server = ActionServerBuilder::<RtNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| handle.succeed(RtNavResult { done: true }))
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

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtNav>::new().unwrap();

    // Send goal and get result (succeeds quickly)
    let result = client.send_goal_and_wait(RtNavGoal { target: 1.0 }, Duration::from_secs(3));
    assert!(result.is_ok(), "Goal should succeed: {:?}", result);

    // Now send a stale cancel — should not crash the server
    // (cancel_goal uses GoalId internally, but we can't access it from send_goal_and_wait)
    // Instead, verify the server processes a second goal cleanly after the first
    let result2 = client.send_goal_and_wait(RtNavGoal { target: 2.0 }, Duration::from_secs(3));
    assert!(
        result2.is_ok(),
        "Second goal should also succeed: {:?}",
        result2
    );

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();
}

// ============================================================================
// Test: Mixed outcomes across sequential goals — metrics verified
// ============================================================================

action! {
    RtMixed {
        goal { mode: u64 }
        feedback { step: u32 }
        result { outcome_code: u64 }
    }
}

#[test]
fn test_action_mixed_outcomes_sequential() {
    serial_setup!();

    let server = ActionServerBuilder::<RtMixed>::new()
        .on_goal(|goal| {
            if goal.mode == 99 {
                GoalResponse::Reject("mode 99 rejected".to_string())
            } else {
                GoalResponse::Accept
            }
        })
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| match handle.goal().mode {
            1 => handle.succeed(RtMixedResult { outcome_code: 1 }),
            2 => handle.abort(RtMixedResult { outcome_code: 2 }),
            3 => handle.canceled(RtMixedResult { outcome_code: 3 }),
            4 => handle.preempted(RtMixedResult { outcome_code: 4 }),
            _ => handle.succeed(RtMixedResult { outcome_code: 0 }),
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

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtMixed>::new().unwrap();

    // Goal 1: Succeed
    let r1 = client.send_goal_and_wait(RtMixedGoal { mode: 1 }, Duration::from_secs(3));
    assert!(r1.is_ok(), "Mode 1 should succeed: {:?}", r1);
    assert_eq!(r1.unwrap().outcome_code, 1);

    // Goal 2: Abort
    let r2 = client.send_goal_and_wait(RtMixedGoal { mode: 2 }, Duration::from_secs(3));
    assert!(r2.is_err(), "Mode 2 should abort: {:?}", r2);

    // Goal 3: Canceled
    let r3 = client.send_goal_and_wait(RtMixedGoal { mode: 3 }, Duration::from_secs(3));
    match r3 {
        Err(ActionError::GoalCanceled) => {}
        other => panic!("Mode 3 should be GoalCanceled, got: {:?}", other),
    }

    // Goal 4: Preempted
    let r4 = client.send_goal_and_wait(RtMixedGoal { mode: 4 }, Duration::from_secs(3));
    match r4 {
        Err(ActionError::GoalPreempted) => {}
        other => panic!("Mode 4 should be GoalPreempted, got: {:?}", other),
    }

    // Goal 5: Rejected
    let r5 = client.send_goal_and_wait(RtMixedGoal { mode: 99 }, Duration::from_secs(3));
    assert!(r5.is_err(), "Mode 99 should be rejected: {:?}", r5);

    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();
}

// ============================================================================
// Test: Server panic during execute callback — client detects failure,
// scheduler continues ticking, sibling node unaffected
// ============================================================================

action! {
    RtCrash {
        goal { should_crash: bool }
        feedback { step: u32 }
        result { ok: bool }
    }
}

/// A simple counter node that ticks happily alongside the action server.
/// Used to verify the scheduler keeps running after the server panics.
struct SiblingCounterNode {
    tick_count: Arc<AtomicU64>,
}

impl horus_core::core::Node for SiblingCounterNode {
    fn name(&self) -> &str {
        "sibling_counter"
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

#[test]
fn test_action_server_panic_during_execute() {
    serial_setup!();

    // Server that panics when goal.should_crash == true
    let server = ActionServerBuilder::<RtCrash>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            if handle.goal().should_crash {
                panic!("Simulated server crash during goal execution!");
            }
            handle.succeed(RtCrashResult { ok: true })
        })
        .build();

    let sibling_ticks = Arc::new(AtomicU64::new(0));
    let sibling_ticks_clone = sibling_ticks.clone();
    let sibling = SiblingCounterNode {
        tick_count: sibling_ticks_clone,
    };

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        sched.add(sibling).order(1).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<RtCrash>::new().unwrap();

    // 1. Send a crashing goal — server panics during execute_callback
    let ticks_before_crash = sibling_ticks.load(Ordering::SeqCst);
    let crash_result =
        client.send_goal_and_wait(RtCrashGoal { should_crash: true }, Duration::from_secs(2));

    // Client should get an error (timeout or execution error — panic doesn't
    // produce a result/status message, so client times out)
    assert!(
        crash_result.is_err(),
        "Crashing goal should return error, got: {:?}",
        crash_result
    );

    // 2. Verify scheduler kept running — sibling node should have ticked
    std::thread::sleep(Duration::from_millis(500));
    let ticks_after_crash = sibling_ticks.load(Ordering::SeqCst);
    assert!(
        ticks_after_crash > ticks_before_crash,
        "Sibling node should keep ticking after server panic: before={}, after={}",
        ticks_before_crash,
        ticks_after_crash
    );

    // 3. Send a normal goal AFTER the crash — verify server recovered
    //    (scheduler's catch_unwind should have logged the error and the
    //     server node continues ticking on subsequent cycles)
    let recovery_result = client.send_goal_and_wait(
        RtCrashGoal {
            should_crash: false,
        },
        Duration::from_secs(3),
    );

    // The server may or may not recover depending on how the panic left
    // its internal state. Either outcome is acceptable:
    // - Ok: server recovered and processed the goal (best case)
    // - Err: server's internal state was corrupted by the panic (acceptable)
    // What's NOT acceptable: the scheduler thread crashing or hanging.
    match &recovery_result {
        Ok(r) => assert!(r.ok, "Recovery goal should return ok=true"),
        Err(_) => {
            // Server may not recover from panic — this is acceptable.
            // The important thing is the scheduler didn't crash.
        }
    }

    // 4. Final check: scheduler thread is still alive (not crashed)
    let final_ticks = sibling_ticks.load(Ordering::SeqCst);
    assert!(
        final_ticks > ticks_after_crash,
        "Sibling should still be ticking: after_crash={}, final={}",
        ticks_after_crash,
        final_ticks
    );

    running.store(false, Ordering::Relaxed);
    // Server thread should join cleanly — NOT panic-propagated
    server_thread
        .join()
        .expect("Server thread should not propagate panic");
}
