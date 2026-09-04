//! `HORUS_LOG_FILE=1` has to actually start something.
//!
//! `start_log_file_drain` was called by no shipped code path. A tree-wide grep
//! for the symbol returned four hits: the definition, its re-export, and two
//! lines of one integration test that invoked it by hand. So the drain worked,
//! its test passed, and `HORUS_LOG_FILE=1 horus run` still wrote no
//! `.horus/logs/horus.log` — with `HORUS_LOG_DIR`, `_MAX_SIZE` and
//! `_MAX_FILES` dead alongside it. An operator who sets a documented variable
//! to get a post-incident log and finds an empty disk is the exact failure the
//! variable exists to prevent.
//!
//! This test therefore does NOT call the drain. It sets the variable, builds a
//! `Scheduler` the way any program does, and asserts a log file appears. It is
//! alone in its binary because it writes process-global environment and the
//! drain starts once per process.

use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_LOG_BUFFER};
use horus_core::scheduling::Scheduler;
use std::time::Duration;

#[test]
fn setting_horus_log_file_makes_a_scheduler_write_logs_to_disk() {
    let dir = std::env::temp_dir().join(format!("horus-log-drain-wiring-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("temp log dir");

    // SAFETY: this test is alone in its binary, so nothing else is reading or
    // writing the environment concurrently.
    unsafe {
        std::env::set_var("HORUS_LOG_FILE", "1");
        std::env::set_var("HORUS_LOG_DIR", dir.to_str().unwrap());
    }

    // The whole point: building a scheduler is all a program does. Nothing
    // here calls `start_log_file_drain`.
    let _scheduler = Scheduler::new();

    for i in 0..20 {
        GLOBAL_LOG_BUFFER.push(LogEntry {
            timestamp: format!("2026-09-04T00:00:{:02}.000Z", i),
            tick_number: GLOBAL_LOG_BUFFER.write_idx() + 1,
            node_name: "wiring_probe".to_string(),
            log_type: LogType::Info,
            topic: None,
            message: format!("entry {}", i),
            tick_us: 0,
            ipc_ns: 0,
        });
    }

    // The drain polls every 500 ms.
    let log_file = dir.join("horus.log");
    let mut content = String::new();
    for _ in 0..40 {
        std::thread::sleep(Duration::from_millis(250));
        if let Ok(c) = std::fs::read_to_string(&log_file) {
            if c.contains("wiring_probe") {
                content = c;
                break;
            }
        }
    }

    let found = content.contains("wiring_probe");
    let _ = std::fs::remove_dir_all(&dir);
    unsafe {
        std::env::remove_var("HORUS_LOG_FILE");
        std::env::remove_var("HORUS_LOG_DIR");
    }

    assert!(
        found,
        "HORUS_LOG_FILE=1 was set and a Scheduler was built, so {} should \
         contain the log entries. Nothing started the drain.",
        log_file.display()
    );
}
