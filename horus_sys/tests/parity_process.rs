//! Process Module Behavioral Parity Tests
//!
//! Verifies ProcessHandle, Signal, session/user IDs, PID start time
//! produce identical observable behavior on all platforms.

use horus_sys::process::*;
use std::time::Duration;

// ═══════════════════════════════════════════════════════════════════════════
// ProcessHandle Liveness
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_current_process_is_alive() {
    let handle = ProcessHandle::from_pid(std::process::id());
    assert!(
        handle.is_alive(),
        "current process must be alive on all platforms"
    );
}

#[test]
fn test_dead_pid_is_not_alive() {
    let handle = ProcessHandle::from_pid(99_999_999);
    assert!(!handle.is_alive(), "nonexistent PID must not be alive");
}

#[test]
fn test_pid_zero_is_not_alive() {
    let handle = ProcessHandle::from_pid(0);
    assert!(!handle.is_alive(), "PID 0 must be rejected (process group)");
}

#[test]
fn test_overflow_pid_is_not_alive() {
    let handle = ProcessHandle::from_pid(u32::MAX);
    assert!(!handle.is_alive(), "overflow PID must be rejected");
}

#[test]
fn test_from_pid_stores_pid() {
    let handle = ProcessHandle::from_pid(42);
    assert_eq!(handle.pid(), 42);
}

// ═══════════════════════════════════════════════════════════════════════════
// Signal Sending
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_signal_kill_to_dead_pid_is_ok() {
    let handle = ProcessHandle::from_pid(99_999_999);
    assert!(
        handle.signal(Signal::Kill).is_ok(),
        "signal to dead PID should be Ok (not an error)"
    );
}

#[test]
fn test_signal_terminate_to_dead_pid_is_ok() {
    let handle = ProcessHandle::from_pid(99_999_999);
    handle.signal(Signal::Terminate).unwrap();
}

#[test]
fn test_signal_interrupt_to_dead_pid_is_ok() {
    let handle = ProcessHandle::from_pid(99_999_999);
    handle.signal(Signal::Interrupt).unwrap();
}

#[test]
fn test_signal_overflow_pid_is_ok() {
    let handle = ProcessHandle::from_pid(u32::MAX);
    assert!(
        handle.signal(Signal::Kill).is_ok(),
        "overflow PID signal should be Ok (guard catches it)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Wait Timeout
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_wait_dead_returns_immediately() {
    let handle = ProcessHandle::from_pid(99_999_999);
    let result = handle.wait_timeout(Some(Duration::from_millis(100)));
    assert!(result.is_ok(), "wait on dead PID should not error");
}

// ═══════════════════════════════════════════════════════════════════════════
// Session & User IDs
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_session_id_is_positive() {
    assert!(session_id() > 0, "session_id should be positive");
}

#[test]
fn test_session_id_is_stable() {
    let s1 = session_id();
    let s2 = session_id();
    assert_eq!(s1, s2, "session_id must be stable across calls");
}

#[test]
fn test_user_id_is_stable() {
    let u1 = user_id();
    let u2 = user_id();
    assert_eq!(u1, u2, "user_id must be stable across calls");
}

#[test]
fn test_namespace_id_format() {
    let ns = namespace_id();
    assert!(ns.starts_with("sid"), "namespace should start with 'sid'");
    assert!(ns.contains("_uid"), "namespace should contain '_uid'");
}

#[test]
fn test_namespace_id_contains_actual_ids() {
    let ns = namespace_id();
    let sid = session_id();
    let uid = user_id();
    assert!(
        ns.contains(&format!("sid{}", sid)),
        "namespace should contain session id"
    );
    assert!(
        ns.contains(&format!("uid{}", uid)),
        "namespace should contain user id"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// PID Start Time
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_pid_start_time_current_is_nonzero() {
    let st = pid_start_time(std::process::id());
    assert!(st > 0, "current process start time should be nonzero");
}

#[test]
fn test_pid_start_time_dead_is_zero() {
    let st = pid_start_time(99_999_999);
    assert_eq!(st, 0, "dead process start time should be 0");
}

#[test]
fn test_pid_start_time_stable() {
    let pid = std::process::id();
    let st1 = pid_start_time(pid);
    let st2 = pid_start_time(pid);
    assert_eq!(st1, st2, "start time must be stable for same PID");
}
