#![allow(dead_code)]
// Reproducing test: a cancel request sent OVER THE TOPIC must stop a goal that
// is CURRENTLY EXECUTING.
//
// This is the real-robot requirement the other action tests do NOT cover. The
// existing suites (action_concurrent.rs, action_preemption.rs) document — and
// test around — the fact that the execute callback ran synchronously inside the
// server's `tick()`, which blocked the tick loop for the whole goal duration.
// While blocked, the server never read the cancel topic, so a cancel arriving
// mid-execution could not be delivered. Those suites therefore test
// cancellation through an EXTERNAL `AtomicBool` backdoor, never through the
// actual cancel topic.
//
// This test closes that gap. The execute callback loops on ONLY
// `handle.is_cancel_requested()` (the topic-driven flag — no external backdoor)
// with a hard iteration guard. A separate client sends a real cancel request
// over the cancel topic while the goal is running. The goal must observe the
// cancel and exit early; if it instead runs to the guard, the cancel was never
// delivered and the action server is not usable for long-running, cancellable
// goals (e.g. a humanoid "walk to X" you need to be able to abort).
//
// Against the old inline-execution server this FAILS (the callback runs to the
// guard because the blocked tick never delivers the cancel). Against the
// thread-per-goal server it PASSES.

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
    CancelTopic {
        goal { tag: u64 }
        feedback { iteration: u32 }
        result { canceled: bool }
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

#[test]
fn cancel_request_over_topic_stops_a_running_goal() {
    serial_setup!();

    // Whether the execute callback observed the cancel via the TOPIC-driven flag
    // (as opposed to running out its iteration guard).
    let saw_cancel = Arc::new(AtomicBool::new(false));
    let hit_guard = Arc::new(AtomicBool::new(false));
    let started = Arc::new(AtomicBool::new(false));
    // Goal id observed server-side, so the test can target the cancel request.
    let observed_goal_id: Arc<Mutex<Option<GoalId>>> = Arc::new(Mutex::new(None));

    let saw_c = saw_cancel.clone();
    let hit_g = hit_guard.clone();
    let started_c = started.clone();
    let obs_id = observed_goal_id.clone();

    // 1000 * 3ms = 3s hard cap. With a working cancel path the goal exits in
    // well under 200ms; hitting the guard means the cancel was never delivered.
    const GUARD: u32 = 1000;

    let server = ActionServerBuilder::<CancelTopic>::new()
        .on_goal(|_g| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(move |handle| {
            *obs_id.lock().unwrap() = Some(handle.goal_id());
            started_c.store(true, Ordering::Release);
            for i in 0..GUARD {
                // ONLY the topic-driven cancel flag — no external backdoor.
                if handle.is_cancel_requested() {
                    saw_c.store(true, Ordering::Release);
                    return handle.canceled(CancelTopicResult { canceled: true });
                }
                handle.publish_feedback(CancelTopicFeedback { iteration: i });
                std::thread::sleep(Duration::from_millis(3));
            }
            hit_g.store(true, Ordering::Release);
            handle.succeed(CancelTopicResult { canceled: false })
        })
        .build();

    let (running, server_handle) = run_server_background(server);
    std::thread::sleep(Duration::from_millis(100)); // let init() create topics

    // Send the goal (blocking) on a background thread so the test thread stays
    // free to issue the cancel while the goal is still running.
    let goal_thread = std::thread::spawn(move || {
        let client = SyncActionClient::<CancelTopic>::new().unwrap();
        client.send_goal_and_wait(CancelTopicGoal { tag: 7 }, Duration::from_secs(10))
    });

    // Wait until the goal is actually executing.
    let t0 = Instant::now();
    while !started.load(Ordering::Acquire) {
        assert!(
            t0.elapsed() < Duration::from_secs(3),
            "goal never started executing"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    let goal_id = observed_goal_id
        .lock()
        .unwrap()
        .expect("goal id recorded once execution starts");

    // Let the goal get mid-flight, then cancel it OVER THE TOPIC.
    std::thread::sleep(Duration::from_millis(50));
    let canceller = SyncActionClient::<CancelTopic>::new().unwrap();
    canceller.cancel_goal(goal_id);

    let result = goal_thread.join().unwrap();
    stop_server(running, server_handle);

    assert!(
        saw_cancel.load(Ordering::Acquire),
        "execute callback never observed the topic cancel (ran to guard = {}). \
         The action server blocked its tick loop during execution, so the cancel \
         request over the topic was never delivered to the running goal.",
        hit_guard.load(Ordering::Acquire)
    );
    assert!(
        !hit_guard.load(Ordering::Acquire),
        "goal ran to its iteration guard instead of canceling"
    );
    assert!(
        matches!(result, Err(ActionError::GoalCanceled)),
        "client should observe GoalCanceled, got {result:?}"
    );
}
