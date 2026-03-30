#![allow(dead_code)]
//! Cross-process service tests.
//!
//! Proves that ServiceClient in one process can call ServiceServer in
//! another process via SHM-backed topics. This is the primary production
//! deployment pattern: planner calls path-planning service in another process.
//!
//! Run sequentially: `cargo test --test services_cross_process -- --test-threads=1`

use horus_core::core::DurationExt;
use horus_core::service;
use horus_core::services::{ServiceClient, ServiceServerBuilder};
use std::process::{Command, Stdio};

mod common;
use common::cleanup_stale_shm;

// ─── Env vars ────────────────────────────────────────────────────────────────

const CHILD_ENV: &str = "HORUS_SVC_XPROC_CHILD";
const TEST_ENV: &str = "HORUS_SVC_XPROC_TEST";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

// ─── Service definitions ─────────────────────────────────────────────────────

service! {
    /// Cross-process addition service.
    XProcAdd {
        request { a: i64, b: i64 }
        response { sum: i64 }
    }
}

service! {
    /// Cross-process echo service for string data.
    XProcEcho {
        request { message: String }
        response { echoed: String }
    }
}

// ─── Child entry points ──────────────────────────────────────────────────────

fn child_call_add_service() {
    // Wait for parent's server to be ready
    std::thread::sleep(std::time::Duration::from_millis(200));

    let mut client =
        ServiceClient::<XProcAdd>::new().expect("child: failed to create ServiceClient");

    let resp = client
        .call(XProcAddRequest { a: 10, b: 20 }, 5_u64.secs())
        .expect("child: service call failed");

    println!("RESULT:{}", resp.sum);
}

fn child_call_add_service_multiple() {
    std::thread::sleep(std::time::Duration::from_millis(200));

    let mut client =
        ServiceClient::<XProcAdd>::new().expect("child: failed to create ServiceClient");

    let mut results = Vec::new();
    for i in 0..10i64 {
        let resp = client
            .call(XProcAddRequest { a: i, b: i * 10 }, 5_u64.secs())
            .expect("child: sequential call failed");
        results.push(resp.sum);
    }

    println!("COUNT:{}", results.len());
    for r in &results {
        println!("V:{}", r);
    }
}

fn child_call_echo_service() {
    std::thread::sleep(std::time::Duration::from_millis(200));

    let mut client =
        ServiceClient::<XProcEcho>::new().expect("child: failed to create echo client");

    let resp = client
        .call(
            XProcEchoRequest {
                message: "hello from child".to_string(),
            },
            5_u64.secs(),
        )
        .expect("child: echo call failed");

    println!("ECHO:{}", resp.echoed);
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
//  1. Basic cross-process service round-trip
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_service_add_roundtrip() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_service_add_roundtrip") {
            child_call_add_service();
        }
        return;
    }

    cleanup_stale_shm();

    // Parent: start server
    let _server = ServiceServerBuilder::<XProcAdd>::new()
        .on_request(|req| Ok(XProcAddResponse { sum: req.a + req.b }))
        .poll_interval(5_u64.ms())
        .build()
        .expect("parent: failed to build server");

    // Spawn child client
    let child = spawn_child("cross_process_service_add_roundtrip");
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "child should exit successfully. stderr: {}",
        stderr
    );

    // Parse result
    let result_line = stdout
        .lines()
        .find(|l| l.starts_with("RESULT:"))
        .expect("child should print RESULT:N");
    let sum: i64 = result_line
        .strip_prefix("RESULT:")
        .unwrap()
        .parse()
        .expect("parse sum");

    assert_eq!(sum, 30, "10 + 20 = 30 across process boundary");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. Multiple sequential calls from child
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_service_multiple_calls() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_service_multiple_calls") {
            child_call_add_service_multiple();
        }
        return;
    }

    cleanup_stale_shm();

    let _server = ServiceServerBuilder::<XProcAdd>::new()
        .on_request(|req| Ok(XProcAddResponse { sum: req.a + req.b }))
        .poll_interval(5_u64.ms())
        .build()
        .expect("parent: failed to build server");

    let child = spawn_child("cross_process_service_multiple_calls");
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(
        output.status.success(),
        "child should succeed. stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    // Parse count
    let count_line = stdout.lines().find(|l| l.starts_with("COUNT:"));
    assert!(count_line.is_some(), "child should print COUNT:N");
    let count: usize = count_line
        .unwrap()
        .strip_prefix("COUNT:")
        .unwrap()
        .parse()
        .unwrap();
    assert_eq!(count, 10, "should have 10 results");

    // Parse values
    let values: Vec<i64> = stdout
        .lines()
        .filter_map(|l| l.strip_prefix("V:"))
        .filter_map(|v| v.parse().ok())
        .collect();
    assert_eq!(values.len(), 10);

    // Verify: a=i, b=i*10, sum=i+i*10=i*11
    for (i, &v) in values.iter().enumerate() {
        assert_eq!(
            v,
            (i as i64) * 11,
            "call {} should return {} * 11 = {}, got {}",
            i,
            i,
            i * 11,
            v
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. Cross-process echo service (string data)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_service_echo_string() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_service_echo_string") {
            child_call_echo_service();
        }
        return;
    }

    cleanup_stale_shm();

    let _server = ServiceServerBuilder::<XProcEcho>::new()
        .on_request(|req| {
            Ok(XProcEchoResponse {
                echoed: format!("echo: {}", req.message),
            })
        })
        .poll_interval(5_u64.ms())
        .build()
        .expect("parent: failed to build echo server");

    let child = spawn_child("cross_process_service_echo_string");
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(
        output.status.success(),
        "child should succeed. stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let echo_line = stdout
        .lines()
        .find(|l| l.starts_with("ECHO:"))
        .expect("child should print ECHO:...");
    let echoed = echo_line.strip_prefix("ECHO:").unwrap();

    assert_eq!(
        echoed, "echo: hello from child",
        "string data must round-trip correctly across processes"
    );
}
