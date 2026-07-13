#![allow(dead_code)]
// Reproducing tests for the QUEUED-goal paths of the action server.
//
// These paths only became genuinely reachable once goal execution moved off the
// tick thread (thread-per-goal): with real concurrency, a goal can now sit in
// `goal_queue` (accepted-but-not-yet-started under PreemptionPolicy::Queue while
// the server is at max_concurrent) while another goal actually runs.
//
// Two bugs this guards against:
//   1. A queued goal was ALSO rejected. `can_accept_goal` returned `false` for a
//      queued goal, and `handle_goal` treated `false` as "reject" — so the client
//      was told GoalRejected while the goal secretly ran later. A queued goal must
//      instead stay pending and run when a slot frees.
//   2. A queued goal could not be canceled. `handle_cancel` only looked in
//      `active_goals`, so a cancel for a queued goal was "not found" and dropped —
//      the goal ran anyway once dequeued. Canceling a queued goal must remove it
//      from the queue so it never executes.

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

mod common;
use common::cleanup_stale_shm;

static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

macro_rules! serial_setup {
    () => {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let _shm_guard = cleanup_stale_shm();
    };
}

action! {
    QRun {
        goal { id: u64 }
        feedback { step: u32 }
        result { id: u64, ok: bool }
    }
}

action! {
    QCancelable {
        goal { id: u64 }
        feedback { step: u32 }
        result { id: u64, ran: bool }
    }
}

fn run_server_background<N: Node + Send + 'static>(
    server: N,
) -> (Arc<AtomicBool>, std::thread::JoinHandle<()>) {
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(200_u64.hz());
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

fn wait_flag(flag: &AtomicBool, what: &str) {
    let t0 = Instant::now();
    while !flag.load(Ordering::Acquire) {
        assert!(t0.elapsed() < Duration::from_secs(3), "{what} never happened");
        std::thread::sleep(Duration::from_millis(5));
    }
}

// A goal queued behind a running goal must NOT be rejected — it must run when
// the slot frees.
#[test]
fn queued_goal_runs_after_slot_frees() {
    serial_setup!();

    let order = Arc::new(Mutex::new(Vec::<u64>::new()));
    let started1 = Arc::new(AtomicBool::new(false));
    let order_c = order.clone();
    let started1_c = started1.clone();

    let server = ActionServerBuilder::<QRun>::new()
        .preemption_policy(PreemptionPolicy::Queue { max_size: 4 })
        .max_concurrent_goals(Some(1))
        .on_goal(|_g| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let id = handle.goal().id;
            order_c.lock().unwrap().push(id);
            if id == 1 {
                started1_c.store(true, Ordering::Release);
                std::thread::sleep(Duration::from_millis(400)); // hold the single slot
            }
            handle.succeed(QRunResult { id, ok: true })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    // Goal 1 takes the only slot.
    let g1 = {
        let c = SyncActionClient::<QRun>::new().unwrap();
        std::thread::spawn(move || c.send_goal_and_wait(QRunGoal { id: 1 }, Duration::from_secs(5)))
    };
    wait_flag(&started1, "goal 1 start");

    // Goal 2 arrives while goal 1 holds the slot: it must be QUEUED (not rejected)
    // and succeed once goal 1 finishes.
    let c2 = SyncActionClient::<QRun>::new().unwrap();
    let r2 = c2.send_goal_and_wait(QRunGoal { id: 2 }, Duration::from_secs(5));
    assert!(
        r2.is_ok(),
        "queued goal 2 must run and succeed after the slot frees (not be rejected), got: {r2:?}"
    );
    assert_eq!(r2.unwrap().id, 2);

    let r1 = g1.join().unwrap();
    assert!(r1.is_ok(), "goal 1 should succeed: {r1:?}");
    assert_eq!(
        *order.lock().unwrap(),
        vec![1, 2],
        "goals should execute 1 then 2"
    );

    stop_server(running, server_handle);
}

// Canceling a goal that is still QUEUED must remove it so it never executes.
#[test]
fn canceling_a_queued_goal_prevents_it_from_running() {
    serial_setup!();

    let started1 = Arc::new(AtomicBool::new(false));
    let goal2_ran = Arc::new(AtomicBool::new(false));
    let started1_c = started1.clone();
    let goal2_ran_c = goal2_ran.clone();

    let server = ActionServerBuilder::<QCancelable>::new()
        .preemption_policy(PreemptionPolicy::Queue { max_size: 4 })
        .max_concurrent_goals(Some(1))
        .on_goal(|_g| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            let id = handle.goal().id;
            if id == 1 {
                started1_c.store(true, Ordering::Release);
                // Cooperatively hold the slot for ~1s.
                for _ in 0..200 {
                    if handle.should_abort() {
                        return handle.canceled(QCancelableResult { id, ran: true });
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
            } else {
                // Goal 2 must never reach here — it should be canceled while queued.
                goal2_ran_c.store(true, Ordering::Release);
            }
            handle.succeed(QCancelableResult { id, ran: true })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100));

    // Goal 1 takes the only slot.
    let g1 = {
        let c = SyncActionClient::<QCancelable>::new().unwrap();
        std::thread::spawn(move || {
            c.send_goal_and_wait(QCancelableGoal { id: 1 }, Duration::from_secs(6))
        })
    };
    wait_flag(&started1, "goal 1 start");

    // Goal 2 is queued behind goal 1. A short timeout makes the client give up
    // and cancel goal 2 (by its own goal_id) while it is still queued.
    let c2 = SyncActionClient::<QCancelable>::new().unwrap();
    let r2 = c2.send_goal_and_wait(QCancelableGoal { id: 2 }, Duration::from_millis(250));
    assert!(
        r2.is_err(),
        "goal 2 should not succeed — it was canceled while queued, got: {r2:?}"
    );

    // Let goal 1 finish and the queue drain; goal 2 must never have executed.
    let _ = g1.join();
    std::thread::sleep(Duration::from_millis(250));
    assert!(
        !goal2_ran.load(Ordering::Acquire),
        "a canceled queued goal must NOT execute"
    );

    stop_server(running, server_handle);
}
