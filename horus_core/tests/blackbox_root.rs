//! Blackbox Flight Recorder Root Tests
//!
//! Requires `blackbox` feature: cargo test -p horus_core --test blackbox_root

#![cfg(feature = "blackbox")]

use horus_core::scheduling::{BlackBox, BlackBoxEvent};

#[test]
fn test_blackbox_new_is_empty() {
    let bb = BlackBox::new(1);
    assert!(bb.is_empty());
    assert_eq!(bb.len(), 0);
}

#[test]
fn test_blackbox_record_and_retrieve() {
    let mut bb = BlackBox::new(1);
    eprintln!("before record: len={}, empty={}", bb.len(), bb.is_empty());
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "test".into(),
        node_count: 1,
        config: "".into(),
    });
    eprintln!("after record: len={}, empty={}", bb.len(), bb.is_empty());
    assert!(!bb.is_empty(), "len={}", bb.len());
    assert_eq!(bb.len(), 1);
    assert_eq!(bb.events().len(), 1);
}

#[test]
fn test_blackbox_multiple_events() {
    let mut bb = BlackBox::new(1);
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "s".into(),
        node_count: 2,
        config: "".into(),
    });
    bb.record(BlackBoxEvent::NodeAdded {
        name: "n1".into(),
        order: 0,
    });
    bb.record(BlackBoxEvent::SchedulerStop {
        reason: "done".into(),
        total_ticks: 100,
    });
    assert_eq!(bb.len(), 3);
}

#[test]
fn test_blackbox_clear() {
    let mut bb = BlackBox::new(1);
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "s".into(),
        node_count: 0,
        config: "".into(),
    });
    bb.clear();
    assert!(bb.is_empty());
}

#[test]
fn test_blackbox_save_load_roundtrip() {
    let tmp = tempfile::tempdir().unwrap();
    let mut bb = BlackBox::new(1).with_path(tmp.path().to_path_buf());
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "rt".into(),
        node_count: 1,
        config: "".into(),
    });
    bb.record(BlackBoxEvent::NodeAdded {
        name: "node1".into(),
        order: 0,
    });
    bb.record(BlackBoxEvent::SchedulerStop {
        reason: "test".into(),
        total_ticks: 50,
    });
    bb.save().expect("save should succeed");

    let mut bb2 = BlackBox::new(1).with_path(tmp.path().to_path_buf());
    bb2.load().expect("load should succeed");
    assert_eq!(bb2.len(), 3);
}

#[test]
fn test_blackbox_anomalies_filter() {
    let mut bb = BlackBox::new(1);
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "s".into(),
        node_count: 1,
        config: "".into(),
    });
    bb.record(BlackBoxEvent::NodeError {
        name: "n".into(),
        error: "oops".into(),
        severity: horus_core::error::Severity::Permanent,
    });
    bb.record(BlackBoxEvent::DeadlineMiss {
        name: "n".into(),
        deadline_us: 1000,
        actual_us: 5000,
    });
    bb.record(BlackBoxEvent::SchedulerStop {
        reason: "done".into(),
        total_ticks: 10,
    });

    let anomalies = bb.anomalies();
    assert!(
        anomalies.len() >= 2,
        "should have NodeError + DeadlineMiss, got {}",
        anomalies.len()
    );
}

#[test]
fn test_blackbox_wal_flush_no_panic() {
    let tmp = tempfile::tempdir().unwrap();
    let mut bb = BlackBox::new(1)
        .with_path(tmp.path().to_path_buf())
        .with_wal_flush_interval(2);
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "wal".into(),
        node_count: 0,
        config: "".into(),
    });
    bb.flush_wal();
}

#[test]
fn test_blackbox_load_nonexistent_graceful() {
    let tmp = tempfile::tempdir().unwrap();
    let mut bb = BlackBox::new(1).with_path(tmp.path().to_path_buf());
    let _ = bb.load(); // Should not panic
}
