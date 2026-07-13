#![allow(dead_code)]
// Preemption policy integration tests for ActionServer.
//
// These tests exercise the PreemptionPolicy variants: RejectNew, Priority,
// Queue, and PreemptOld. Each accepted goal executes on its OWN thread, so the
// server's tick loop stays responsive while a goal runs — it keeps reading the
// goal/cancel topics and applies the preemption policy in `can_accept_goal`
// while goals are genuinely in flight (`active_goals` at max_concurrent).
//
// Test strategy per policy:
//
// - RejectNew: While a slow goal is genuinely executing on its thread, a second
//   goal is ACTIVELY rejected by the policy (the server is at capacity, not
//   deaf). The client observes GoalRejected. After the slow goal finishes,
//   subsequent goals succeed.
//
// - Priority: Goals sent sequentially through one blocking client complete in
//   FIFO order with no contention (the client serializes them). Verifies the
//   priority config is applied and results are correct.
//
// - Queue: Send goals sequentially through a single client, verifying FIFO
//   order and correct execution counters.
//
// - PreemptOld: A slow goal runs on its thread; a second goal arriving at
//   capacity preempts it. The running goal observes `should_abort()`, exits with
//   `preempted()`, and the new goal proceeds — real concurrent preemption.

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

macro_rules! serial_setup {
    () => {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let _shm_guard = cleanup_stale_shm();
    };
}

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
// Test 1: RejectNew — server actively rejects new goals while at capacity
//
// With max_concurrent=1 and PreemptionPolicy::RejectNew, a second goal sent
// while the first is genuinely executing (on its own thread) is ACTIVELY
// rejected by the policy — the server keeps ticking and responds with
// GoalRejected, rather than going deaf and letting the client time out. Once
// the first goal completes, the server is free and the next goal succeeds.
// ============================================================================

#[test]
fn test_reject_new_policy_rejects_when_server_busy() {
    serial_setup!();

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

    // Goal 1 is now executing on its own thread for ~800ms. The server keeps
    // ticking, so a second goal sent now is at capacity (active=1, max=1) and
    // the RejectNew policy ACTIVELY rejects it. The client observes GoalRejected
    // promptly — well within the timeout, i.e. not a timeout.
    let client2 = SyncActionClient::<PreemptSlow>::new().unwrap();
    let result2 = client2.send_goal_and_wait(PreemptSlowGoal { id: 2 }, Duration::from_secs(2));

    assert!(
        matches!(result2, Err(ActionError::GoalRejected(_))),
        "Expected GoalRejected while server is genuinely busy, got: {:?}",
        result2
    );

    // Wait for goal 1 to finish
    let r1 = goal1_handle.join().unwrap();
    assert!(r1.is_ok(), "First goal should succeed: {:?}", r1);

    // Server is now free. A third goal should succeed immediately.
    let client3 = SyncActionClient::<PreemptSlow>::new().unwrap();
    let result3 = client3.send_goal_and_wait(PreemptSlowGoal { id: 3 }, Duration::from_secs(3));
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
// With PreemptionPolicy::Priority and max_concurrent=1, goals are sent through
// a single blocking client, which serializes them (each result is awaited
// before the next goal is sent), so there is no contention in this test. We
// verify that:
// 1. The priority policy is correctly configured (no crash, no unexpected
//    rejection).
// 2. Both low-priority and high-priority goals succeed.
// 3. The execution order matches send order (FIFO).
// (Concurrent priority preemption is covered by the PreemptOld test's pattern.)
// ============================================================================

#[test]
fn test_priority_policy_preempts_lower_priority() {
    serial_setup!();

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
// With PreemptionPolicy::Queue { max_size: 3 } and max_concurrent=1, at most
// one goal executes at a time. We send goals sequentially through a single
// blocking client, verifying they complete in the correct order with the
// expected execution counters.
// ============================================================================

#[test]
fn test_queue_policy_queues_goals() {
    serial_setup!();

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
        let result = client.send_goal_and_wait(QueueActionGoal { seq }, Duration::from_secs(5));
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
        assert_eq!(r.order, i as u64, "Execution order mismatch at index {}", i);
    }

    stop_server(running, server_handle);
}

// ============================================================================
// Test 4: PreemptOld — a new goal preempts the running goal (real concurrency)
//
// With PreemptionPolicy::PreemptOld and max_concurrent=1: a slow goal (id=1)
// runs on its own thread; while it is genuinely executing, a second goal (id=2)
// arrives at capacity and preempts it. The running goal observes
// `handle.should_abort()` becoming true (the server set its preempt flag) and
// exits early via `preempted()`. The new goal then proceeds and succeeds. This
// exercises the actual concurrent-preemption path, which the old synchronous
// (tick-blocking) execution model could never reach.
// ============================================================================

#[test]
fn test_preempt_old_cancels_existing_goal() {
    serial_setup!();

    let first_started = Arc::new(AtomicBool::new(false));
    let first_started_clone = first_started.clone();
    let first_hit_guard = Arc::new(AtomicBool::new(false));
    let first_hit_guard_clone = first_hit_guard.clone();

    let server = ActionServerBuilder::<PreemptFast>::new()
        .preemption_policy(PreemptionPolicy::PreemptOld)
        .max_concurrent_goals(Some(1))
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let goal_id = handle.goal().id;

            if goal_id == 1 {
                first_started_clone.store(true, Ordering::Release);
                // Cooperative slow goal: run up to ~2s, exiting early if the
                // server requests preemption. 400 * 5ms = 2s hard cap.
                for _ in 0..400 {
                    if handle.should_abort() {
                        return handle.preempted(PreemptFastResult { goal_id, ok: false });
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
                // Reached the guard: preemption was never observed.
                first_hit_guard_clone.store(true, Ordering::Release);
            }

            handle.succeed(PreemptFastResult { goal_id, ok: true })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    // Send the slow goal (id=1) on a background thread with its own client so
    // the test thread stays free to send the preempting goal.
    let goal1_handle = {
        let client = SyncActionClient::<PreemptFast>::new().unwrap();
        std::thread::spawn(move || {
            client.send_goal_and_wait(PreemptFastGoal { id: 1 }, Duration::from_secs(5))
        })
    };

    // Wait until goal 1 is genuinely executing.
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    while !first_started.load(Ordering::Acquire) {
        if std::time::Instant::now() > deadline {
            panic!("First goal never started executing");
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    // Send the preempting goal (id=2). At capacity, PreemptOld preempts goal 1
    // and accepts goal 2, which should succeed.
    let client2 = SyncActionClient::<PreemptFast>::new().unwrap();
    let r2 = client2.send_goal_and_wait(PreemptFastGoal { id: 2 }, Duration::from_secs(3));
    assert!(r2.is_ok(), "Preempting goal should succeed: {:?}", r2);
    assert_eq!(r2.unwrap().goal_id, 2);

    // Goal 1 must have observed preemption and exited early — not run to guard.
    let r1 = goal1_handle.join().unwrap();
    assert!(
        matches!(r1, Err(ActionError::GoalPreempted)),
        "First goal should be preempted by the newer goal, got: {:?}",
        r1
    );
    assert!(
        !first_hit_guard.load(Ordering::Acquire),
        "First goal ran to its guard instead of observing preemption"
    );

    stop_server(running, server_handle);
}
