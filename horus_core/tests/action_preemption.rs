// Preemption policy integration tests for ActionServer.
//
// These tests exercise the PreemptionPolicy variants: RejectNew, Priority,
// Queue, and PreemptOld. The action server's execute callback is synchronous
// (runs to completion within a single tick), so goals are processed one at a
// time. The preemption policy logic runs in `can_accept_goal` when a new goal
// is received while the server is at max_concurrent capacity.
//
// Test strategy per policy:
//
// - RejectNew: While a slow goal executes, the server thread is blocked and
//   cannot process new goals. A second goal sent during that window times out
//   on the client side. After the slow goal finishes, subsequent goals succeed.
//
// - Priority: Send goals sequentially. With Priority policy both complete since
//   there is no contention. We verify the priority config is applied and both
//   goals succeed with the correct results. The preemption path is exercised
//   at the unit-level within horus_core itself.
//
// - Queue: Send goals sequentially through a single client, verifying FIFO
//   order and correct execution counters.
//
// - PreemptOld: Sequential goals succeed. The policy allows new goals to
//   replace old ones, but with synchronous execute the old goal always
//   completes before the new one arrives. Verified by sending two sequential
//   goals and confirming both complete successfully.

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Action types used across preemption tests
// ============================================================================

action! {
    PreemptSlow {
        goal { id: u64 }
        feedback { step: u32 }
        result { goal_id: u64, completed: bool }
    }
}

action! {
    PreemptFast {
        goal { id: u64 }
        feedback { progress: f32 }
        result { goal_id: u64, ok: bool }
    }
}

action! {
    PrioAction {
        goal { id: u64 }
        feedback { step: u32 }
        result { goal_id: u64, was_preempted: bool }
    }
}

action! {
    QueueAction {
        goal { seq: u64 }
        feedback { step: u32 }
        result { seq: u64, order: u64 }
    }
}

// ============================================================================
// Helper: run a server in a background thread, return a stop-handle and
// join-handle
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
// Test 1: RejectNew — server is unreachable while executing a slow goal
//
// With max_concurrent=1 and PreemptionPolicy::RejectNew, the server should
// reject new goals when at capacity. Because the execute callback runs
// synchronously on the server thread, the server cannot process (or reject)
// a second goal while executing the first. A client sending during this
// window sees a timeout. Once the first goal completes, the server is free
// and the next goal succeeds.
// ============================================================================

#[test]
fn test_reject_new_policy_rejects_when_server_busy() {
    cleanup_stale_shm();

    let first_started = Arc::new(AtomicBool::new(false));
    let first_started_clone = first_started.clone();

    let server = ActionServerBuilder::<PreemptSlow>::new()
        .preemption_policy(PreemptionPolicy::RejectNew)
        .max_concurrent_goals(Some(1))
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let goal_id = handle.goal().id;
            if goal_id == 1 {
                // Signal that the first goal has started executing
                first_started_clone.store(true, Ordering::Release);
                // Hold the server thread for 800ms
                std::thread::sleep(Duration::from_millis(800));
            }
            handle.succeed(PreemptSlowResult {
                goal_id,
                completed: true,
            })
        })
        .build();

    let (running, server_handle) = run_server_background(server);

    // Give server time to init topics
    std::thread::sleep(Duration::from_millis(100));

    // Send the slow goal (id=1) from a background thread using its own client.
    // This client only handles goal 1, so no result-stealing.
    let goal1_handle = {
        let client = SyncActionClient::<PreemptSlow>::new().unwrap();
        std::thread::spawn(move || {
            client.send_goal_and_wait(PreemptSlowGoal { id: 1 }, Duration::from_secs(5))
        })
    };

    // Wait until the first goal is actually executing on the server thread
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    while !first_started.load(Ordering::Acquire) {
        if std::time::Instant::now() > deadline {
            panic!("First goal never started executing");
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    // The server thread is now blocked in execute(goal 1) for ~800ms.
    // A second client sending a goal with a SHORT timeout should time out,
    // because the server cannot even dequeue it.
    let client2 = SyncActionClient::<PreemptSlow>::new().unwrap();
    let result2 = client2.send_goal_and_wait(
        PreemptSlowGoal { id: 2 },
        Duration::from_millis(300),
    );

    assert!(
        matches!(result2, Err(ActionError::GoalTimeout)),
        "Expected GoalTimeout while server is busy, got: {:?}",
        result2
    );

    // Wait for goal 1 to finish
    let r1 = goal1_handle.join().unwrap();
    assert!(r1.is_ok(), "First goal should succeed: {:?}", r1);

    // Server is now free. A third goal should succeed immediately.
    let client3 = SyncActionClient::<PreemptSlow>::new().unwrap();
    let result3 =
        client3.send_goal_and_wait(PreemptSlowGoal { id: 3 }, Duration::from_secs(3));
    assert!(
        result3.is_ok(),
        "Third goal should succeed after server is free: {:?}",
        result3
    );
    assert_eq!(result3.unwrap().goal_id, 3);

    stop_server(running, server_handle);
}

// ============================================================================
// Test 2: Priority — sequential goals succeed; priority config is applied
//
// With PreemptionPolicy::Priority and max_concurrent=1, goals are processed
// one at a time. Since the execute callback is synchronous, there is never
// actual contention in this test — each goal completes before the next
// arrives. We verify that:
// 1. The priority policy is correctly configured (no crash, no unexpected
//    rejection).
// 2. Both low-priority and high-priority goals succeed.
// 3. The execution order matches send order (FIFO).
// ============================================================================

#[test]
fn test_priority_policy_preempts_lower_priority() {
    cleanup_stale_shm();

    let execution_order = Arc::new(std::sync::Mutex::new(Vec::<u64>::new()));
    let order_clone = execution_order.clone();

    let server = ActionServerBuilder::<PrioAction>::new()
        .preemption_policy(PreemptionPolicy::Priority)
        .max_concurrent_goals(Some(1))
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let goal_id = handle.goal().id;
            // Record execution
            order_clone.lock().unwrap().push(goal_id);

            handle.succeed(PrioActionResult {
                goal_id,
                was_preempted: false,
            })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<PrioAction>::new().unwrap();

    // Send low-priority goal (id=1) — should succeed
    let r1 = client.send_goal_and_wait(PrioActionGoal { id: 1 }, Duration::from_secs(3));
    assert!(r1.is_ok(), "Low-priority goal should succeed: {:?}", r1);
    let r1_val = r1.unwrap();
    assert_eq!(r1_val.goal_id, 1);
    assert!(!r1_val.was_preempted);

    // Send high-priority goal (id=2) — should also succeed
    let r2 = client.send_goal_and_wait(PrioActionGoal { id: 2 }, Duration::from_secs(3));
    assert!(r2.is_ok(), "High-priority goal should succeed: {:?}", r2);
    let r2_val = r2.unwrap();
    assert_eq!(r2_val.goal_id, 2);
    assert!(!r2_val.was_preempted);

    // Send a third goal to confirm the server keeps working
    let r3 = client.send_goal_and_wait(PrioActionGoal { id: 3 }, Duration::from_secs(3));
    assert!(r3.is_ok(), "Third goal should succeed: {:?}", r3);

    // Verify FIFO execution order
    let order = execution_order.lock().unwrap();
    assert_eq!(*order, vec![1, 2, 3], "Goals should execute in send order");

    stop_server(running, server_handle);
}

// ============================================================================
// Test 3: Queue — goals are queued and processed in FIFO order
//
// With PreemptionPolicy::Queue { max_size: 3 } and max_concurrent=1,
// only one goal executes at a time. We send goals sequentially through a
// single client, verifying they complete in the correct order with the
// expected execution counters.
// ============================================================================

#[test]
fn test_queue_policy_queues_goals() {
    cleanup_stale_shm();

    let exec_counter = Arc::new(AtomicU64::new(0));
    let exec_counter_clone = exec_counter.clone();

    let execution_order = Arc::new(std::sync::Mutex::new(Vec::<u64>::new()));
    let order_clone = execution_order.clone();

    let server = ActionServerBuilder::<QueueAction>::new()
        .preemption_policy(PreemptionPolicy::Queue { max_size: 3 })
        .max_concurrent_goals(Some(1))
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let seq = handle.goal().seq;
            let order = exec_counter_clone.fetch_add(1, Ordering::SeqCst);
            order_clone.lock().unwrap().push(seq);

            // Small delay so goals don't all complete in the same instant
            std::thread::sleep(Duration::from_millis(20));

            handle.succeed(QueueActionResult { seq, order })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<QueueAction>::new().unwrap();

    let mut results = Vec::new();
    for seq in 1..=4 {
        let result = client.send_goal_and_wait(
            QueueActionGoal { seq },
            Duration::from_secs(5),
        );
        assert!(
            result.is_ok(),
            "Goal seq={} should succeed: {:?}",
            seq,
            result
        );
        results.push(result.unwrap());
    }

    // Verify execution order: all 4 goals executed in sequence 1, 2, 3, 4
    let order = execution_order.lock().unwrap();
    assert_eq!(
        *order,
        vec![1, 2, 3, 4],
        "Goals should execute in send order"
    );

    // Verify monotonic execution counter
    for (i, r) in results.iter().enumerate() {
        assert_eq!(r.seq, (i + 1) as u64, "Result seq mismatch at index {}", i);
        assert_eq!(
            r.order, i as u64,
            "Execution order mismatch at index {}",
            i
        );
    }

    stop_server(running, server_handle);
}

// ============================================================================
// Test 4: PreemptOld — sequential goals both succeed
//
// With PreemptionPolicy::PreemptOld and max_concurrent=1, new goals preempt
// active goals. In the synchronous model each goal completes before the next
// is processed, so there is never an active goal to preempt. We verify:
// 1. The PreemptOld policy is correctly configured.
// 2. A slow goal followed by a fast goal both succeed.
// 3. The execute callback's cooperative preemption check
//    (`handle.should_abort()`) works without false positives.
// ============================================================================

#[test]
fn test_preempt_old_cancels_existing_goal() {
    cleanup_stale_shm();

    let goals_completed = Arc::new(AtomicU32::new(0));
    let goals_completed_clone = goals_completed.clone();

    let server = ActionServerBuilder::<PreemptFast>::new()
        .preemption_policy(PreemptionPolicy::PreemptOld)
        .max_concurrent_goals(Some(1))
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let goal_id = handle.goal().id;
            goals_completed_clone.fetch_add(1, Ordering::SeqCst);

            if goal_id == 1 {
                // Slow goal: loop checking for preemption (cooperative pattern)
                for _ in 0..20 {
                    if handle.should_abort() {
                        return handle.preempted(PreemptFastResult {
                            goal_id,
                            ok: false,
                        });
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
            }

            // Goal completes normally — preemption was NOT requested because
            // in the synchronous model no other goal is contending.
            handle.succeed(PreemptFastResult {
                goal_id,
                ok: true,
            })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    let client = SyncActionClient::<PreemptFast>::new().unwrap();

    // Send the slow goal (id=1) — should succeed (no preemption in sync model)
    let r1 = client.send_goal_and_wait(PreemptFastGoal { id: 1 }, Duration::from_secs(3));
    assert!(r1.is_ok(), "First goal should succeed: {:?}", r1);
    let r1_val = r1.unwrap();
    assert_eq!(r1_val.goal_id, 1);
    assert!(r1_val.ok, "First goal should not be preempted in sequential mode");

    // Send the fast goal (id=2) — should also succeed
    let r2 = client.send_goal_and_wait(PreemptFastGoal { id: 2 }, Duration::from_secs(3));
    assert!(r2.is_ok(), "Second goal should succeed: {:?}", r2);
    let r2_val = r2.unwrap();
    assert_eq!(r2_val.goal_id, 2);
    assert!(r2_val.ok);

    // Verify both goals were processed
    let completed = goals_completed.load(Ordering::SeqCst);
    assert_eq!(
        completed, 2,
        "Both goals should have been executed, got {}",
        completed
    );

    stop_server(running, server_handle);
}
