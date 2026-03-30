//! Cross-process action tests.
//!
//! Proves that SyncActionClient in one process can send goals to an
//! ActionServer in another process, receive results, and correlate GoalIds
//! across the SHM-backed 5-topic pipeline (goal/status/feedback/result/cancel).
//!
//! Run sequentially: `cargo test --test actions_cross_process -- --test-threads=1`

use horus_core::action;
use horus_core::actions::*;
use horus_core::core::DurationExt;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use std::process::{Command, Stdio};

mod common;
use common::cleanup_stale_shm;

// ─── Env vars ────────────────────────────────────────────────────────────────

const CHILD_ENV: &str = "HORUS_ACT_XPROC_CHILD";
const TEST_ENV: &str = "HORUS_ACT_XPROC_TEST";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

// ─── Action definitions ──────────────────────────────────────────────────────

action! {
    /// Cross-process navigation action.
    XProcNav {
        goal { target_x: f64 }
        feedback { progress: f32 }
        result { reached: bool }
    }
}

// ─── Child entry points ──────────────────────────────────────────────────────

fn child_send_goal_and_wait() {
    // Wait for parent's server to be ready
    std::thread::sleep(Duration::from_millis(500));

    let client = SyncActionClient::<XProcNav>::new()
        .expect("child: failed to create SyncActionClient");

    let result = client
        .send_goal_and_wait(XProcNavGoal { target_x: 42.0 }, Duration::from_secs(10));

    match result {
        Ok(r) => {
            println!("REACHED:{}", r.reached);
        }
        Err(e) => {
            println!("ERROR:{:?}", e);
        }
    }
}

// ─── Spawn helper ────────────────────────────────────────────────────────────

fn spawn_child(test_name: &str) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TEST_ENV, test_name)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn child")
}

// ═══════════════════════════════════════════════════════════════════════════════
//  1. Full goal lifecycle across processes
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_action_goal_lifecycle() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref()
            == Some("cross_process_action_goal_lifecycle")
        {
            child_send_goal_and_wait();
        }
        return;
    }

    cleanup_stale_shm();

    // Build action server
    let server = ActionServerBuilder::<XProcNav>::new()
        .on_goal(|_goal| GoalResponse::Accept)
        .on_cancel(|_id| CancelResponse::Accept)
        .on_execute(|handle| {
            handle.publish_feedback(XProcNavFeedback { progress: 0.5 });
            handle.publish_feedback(XProcNavFeedback { progress: 1.0 });
            handle.succeed(XProcNavResult { reached: true })
        })
        .build();

    // Run server in scheduler on background thread
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

    // Give server time to init and create topics
    std::thread::sleep(Duration::from_millis(200));

    // Spawn child client
    let child = spawn_child("cross_process_action_goal_lifecycle");
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    // Stop server
    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    // Check child succeeded
    if !output.status.success() {
        panic!(
            "child should exit successfully.\nstdout: {}\nstderr: {}",
            stdout, stderr
        );
    }

    // Parse result
    if let Some(line) = stdout.lines().find(|l| l.starts_with("REACHED:")) {
        let reached: bool = line
            .strip_prefix("REACHED:")
            .unwrap()
            .parse()
            .expect("parse reached");
        assert!(reached, "goal should be reached across process boundary");
    } else if let Some(line) = stdout.lines().find(|l| l.starts_with("ERROR:")) {
        panic!(
            "child got action error: {}\nstderr: {}",
            line, stderr
        );
    } else {
        panic!(
            "child produced no REACHED: or ERROR: line.\nstdout: {}\nstderr: {}",
            stdout, stderr
        );
    }
}
