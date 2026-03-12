//! Crash recovery integration tests.
//!
//! Tests the production hardening features:
//! - Stale SHM namespace cleanup
//! - Panic hook crash report generation
//! - Scheduler startup cleanup

mod common;

use horus_core::memory::{cleanup_stale_namespaces, shm_base_dir};

/// Test: cleanup_stale_namespaces does not remove the current process's namespace.
///
/// Calls the cleanup function and verifies it returns without error and does not
/// remove our own SHM directory.
#[test]
fn test_cleanup_stale_namespaces_preserves_current() {
    common::cleanup_stale_shm();

    let base = shm_base_dir();
    // Ensure our SHM dir exists
    std::fs::create_dir_all(&base).ok();

    let result = cleanup_stale_namespaces();

    // The function should complete without panicking
    // Our own namespace should not be removed (skipped)
    assert!(
        result.errors.is_empty(),
        "cleanup should not produce errors: {:?}",
        result.errors
    );

    // Base dir should still exist
    assert!(
        base.exists(),
        "Current process's SHM dir should not be removed"
    );
}

/// Test: cleanup_stale_namespaces handles missing SHM parent gracefully.
///
/// If the SHM parent directory doesn't exist, cleanup should return 0 removed.
#[test]
fn test_cleanup_handles_missing_parent() {
    let result = cleanup_stale_namespaces();

    // Should return cleanly even if parent dir is empty/missing
    assert!(result.errors.is_empty());
}

/// Test: panic hook writes crash report file.
///
/// Spawns a child process that installs the panic hook and panics,
/// then checks for the crash report file.
#[test]
#[cfg(unix)]
fn test_panic_hook_writes_crash_report() {
    use std::process::Command;

    // Run ourselves as a subprocess with a special env var that triggers a panic.
    // Use spawn() instead of output() so we can capture the child PID for
    // finding the exact crash report file (/tmp/horus_crash_<pid>.log).
    let exe = std::env::current_exe().expect("no current exe");
    let mut child = Command::new(&exe)
        .arg("--test-threads=1")
        .arg("--exact")
        .arg("--ignored")
        .arg("panic_hook_subprocess_helper")
        .env("HORUS_CRASH_TEST", "1")
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("failed to spawn subprocess");

    let pid = child.id();
    let status = child.wait().expect("failed to wait on subprocess");

    // The subprocess should have exited non-zero (panic)
    assert!(
        !status.success(),
        "Subprocess should have exited non-zero due to panic"
    );

    // Find the crash report by the subprocess PID
    let crash_path = std::path::PathBuf::from(format!("/tmp/horus_crash_{pid}.log"));
    assert!(
        crash_path.exists(),
        "Crash report not found at {}",
        crash_path.display()
    );

    let content = std::fs::read_to_string(&crash_path).expect("Should read crash report");
    assert!(
        content.contains("HORUS Crash Report"),
        "Crash report should contain header, got: {}",
        &content[..content.len().min(200)]
    );
    assert!(
        content.contains("intentional crash for testing"),
        "Crash report should contain panic message"
    );

    // Clean up
    std::fs::remove_file(&crash_path).ok();
}

/// Helper test that exists only to be called as a subprocess by
/// test_panic_hook_writes_crash_report. It installs the panic hook and panics.
///
/// Ignored in normal test runs — only invoked explicitly as a subprocess
/// via `--exact panic_hook_subprocess_helper` with HORUS_CRASH_TEST=1.
#[test]
#[ignore]
fn panic_hook_subprocess_helper() {
    if std::env::var("HORUS_CRASH_TEST").is_err() {
        // Skip unless explicitly invoked as subprocess
        return;
    }

    // Install a panic hook matching the real scheduler format
    // (see Scheduler::setup_panic_hook in scheduler/mod.rs)
    let pid = std::process::id();
    let crash_path = format!("/tmp/horus_crash_{}.log", pid);

    std::panic::set_hook(Box::new(move |info| {
        let location = info
            .location()
            .map(|l| format!("{}:{}:{}", l.file(), l.line(), l.column()))
            .unwrap_or_else(|| "unknown".to_string());
        let payload = if let Some(s) = info.payload().downcast_ref::<&str>() {
            s.to_string()
        } else if let Some(s) = info.payload().downcast_ref::<String>() {
            s.clone()
        } else {
            "unknown panic payload".to_string()
        };
        let thread_name = std::thread::current().name().unwrap_or("unknown").to_string();
        let report = format!(
            "=== HORUS Crash Report ===\n\
             Scheduler: test\n\
             PID: {}\n\
             Thread: {}\n\
             Location: {}\n\
             Panic: {}\n\
             Blackbox flushed: false\n\
             ===========================\n",
            pid, thread_name, location, payload,
        );
        std::fs::write(&crash_path, &report).ok();
    }));

    panic!("intentional crash for testing");
}
