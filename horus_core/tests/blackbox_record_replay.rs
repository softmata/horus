//! Integration tests for BlackBox and Record/Replay system.
//!
//! BlackBox tests: save/load round-trip, legacy array compat, WAL I/O, disabled mode.
//! Record/Replay tests: config paths, NodeRecording save/load, interval skip,
//! replayer advance/seek/reset, RecordingManager session listing.

use horus_core::scheduling::{
    BlackBox, BlackBoxEvent, NodeRecording, NodeReplayer, NodeTickSnapshot, Recording,
    RecordingConfig, RecordingManager,
};
use std::path::PathBuf;

// ============================================================================
// BlackBox tests
// ============================================================================

#[test]
fn test_blackbox_save_load_round_trip() {
    let dir = std::env::temp_dir().join(format!("horus_bb_roundtrip_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);

    let mut bb = BlackBox::new(1).with_path(dir.clone());

    // Record some events
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "test_sched".to_string(),
        node_count: 3,
        config: "default".to_string(),
    });
    bb.record(BlackBoxEvent::NodeTick {
        name: "sensor".to_string(),
        duration_us: 42,
        success: true,
    });
    bb.record(BlackBoxEvent::DeadlineMiss {
        name: "slow_node".to_string(),
        deadline_us: 1000,
        actual_us: 2000,
    });

    let original_events = bb.events();
    assert_eq!(original_events.len(), 3);

    // Save
    bb.save().expect("save should succeed");

    // Load into a new blackbox
    let mut bb2 = BlackBox::new(1).with_path(dir.clone());
    bb2.load().expect("load should succeed");

    let loaded_events = bb2.events();
    assert_eq!(
        loaded_events.len(),
        original_events.len(),
        "Loaded event count should match"
    );

    // Verify first event type
    match &loaded_events[0].event {
        BlackBoxEvent::SchedulerStart {
            name, node_count, ..
        } => {
            assert_eq!(name, "test_sched");
            assert_eq!(*node_count, 3);
        }
        other => unreachable!("First event should be SchedulerStart, got {:?}", other),
    }

    // Cleanup
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_blackbox_legacy_array_compat() {
    let dir = std::env::temp_dir().join(format!("horus_bb_legacy_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    // Write a bare JSON array (legacy format)
    let legacy_json = r#"[
        {
            "timestamp_us": 1000000,
            "tick": 0,
            "event": {
                "Custom": {
                    "category": "test",
                    "message": "legacy event"
                }
            }
        }
    ]"#;
    std::fs::write(dir.join("blackbox.json"), legacy_json).unwrap();

    // Load should succeed with legacy format
    let mut bb = BlackBox::new(1).with_path(dir.clone());
    bb.load().expect("Legacy format should load");

    let events = bb.events();
    assert_eq!(events.len(), 1, "Should load 1 legacy event");
    match &events[0].event {
        BlackBoxEvent::Custom { category, message } => {
            assert_eq!(category, "test");
            assert_eq!(message, "legacy event");
        }
        other => unreachable!("Should be a Custom event, got {:?}", other),
    }

    // loss counter should be 0 for legacy format
    assert_eq!(bb.get_loss_count(), 0);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_blackbox_wal_actual_io() {
    let dir = std::env::temp_dir().join(format!("horus_bb_wal_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);

    let mut bb = BlackBox::new(1)
        .with_wal_flush_interval(1) // Flush every record for test
        .with_path(dir.clone());

    // Record events (WAL should be written)
    for i in 0..5 {
        bb.record(BlackBoxEvent::Custom {
            category: "wal_test".to_string(),
            message: format!("event {}", i),
        });
    }
    bb.flush_wal();

    // Verify WAL file exists and has content
    let wal_path = dir.join("blackbox.wal");
    assert!(wal_path.exists(), "WAL file should exist");

    let wal_content = std::fs::read_to_string(&wal_path).unwrap();
    let lines: Vec<&str> = wal_content.lines().collect();
    assert_eq!(lines.len(), 5, "WAL should have 5 lines");

    // Each line should be valid JSON
    for line in &lines {
        let parsed: serde_json::Value =
            serde_json::from_str(line).expect("Each WAL line should be valid JSON");
        assert!(parsed.get("event").is_some());
    }

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_blackbox_disabled_no_records() {
    let mut bb = BlackBox::new(0); // 0 MB = disabled

    bb.record(BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "should not record".to_string(),
    });
    bb.record(BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "also not recorded".to_string(),
    });

    assert!(bb.is_empty(), "Disabled blackbox should have no records");
    assert_eq!(bb.len(), 0);
}

// ============================================================================
// Record/Replay tests
// ============================================================================

#[test]
fn test_recording_config_paths() {
    let config = RecordingConfig {
        session_name: "test_session".to_string(),
        base_dir: PathBuf::from("/tmp/horus_recordings"),
        interval: 1,
        record_inputs: true,
        record_outputs: true,
        record_timing: true,
    };

    let session_dir = config.session_dir();
    assert_eq!(
        session_dir,
        PathBuf::from("/tmp/horus_recordings/test_session")
    );

    let node_path = config.node_path("motor_ctrl", "node_001");
    assert_eq!(
        node_path,
        PathBuf::from("/tmp/horus_recordings/test_session/motor_ctrl@node_001.horus")
    );

    let sched_path = config.scheduler_path("sched_001");
    assert_eq!(
        sched_path,
        PathBuf::from("/tmp/horus_recordings/test_session/scheduler@sched_001.horus")
    );
}

#[test]
fn test_node_recording_save_load() {
    let dir = std::env::temp_dir().join(format!("horus_recording_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);

    let mut recording = NodeRecording::new("motor_ctrl", "node_001", "test_session");

    // Add snapshots
    recording.add_snapshot(
        NodeTickSnapshot::new(0)
            .with_input("encoder", vec![1, 2, 3])
            .with_output("velocity", vec![4, 5, 6])
            .with_duration(1000),
    );
    recording.add_snapshot(
        NodeTickSnapshot::new(1)
            .with_input("encoder", vec![7, 8, 9])
            .with_state(vec![10, 11, 12])
            .with_duration(2000),
    );

    assert_eq!(recording.snapshot_count(), 2);
    assert_eq!(recording.first_tick, 0);
    assert_eq!(recording.last_tick, 1);

    // Save
    let path = dir.join("test_recording.horus");
    recording.save(&path).expect("save should succeed");

    // Load
    let loaded = NodeRecording::load(&path).expect("load should succeed");

    assert_eq!(loaded.node_name, "motor_ctrl");
    assert_eq!(loaded.node_id, "node_001");
    assert_eq!(loaded.session_name, "test_session");
    assert_eq!(loaded.snapshot_count(), 2);
    assert_eq!(loaded.first_tick, 0);
    assert_eq!(loaded.last_tick, 1);

    // Verify snapshot content
    let snap0 = loaded.snapshot(0).expect("should find tick 0");
    assert_eq!(snap0.inputs.get("encoder").unwrap(), &vec![1, 2, 3]);
    assert_eq!(snap0.outputs.get("velocity").unwrap(), &vec![4, 5, 6]);
    assert_eq!(snap0.duration_ns, 1000);

    let snap1 = loaded.snapshot(1).expect("should find tick 1");
    assert_eq!(snap1.state.as_ref().unwrap(), &vec![10, 11, 12]);
    assert_eq!(snap1.duration_ns, 2000);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_node_recording_interval_skip_simulation() {
    // Simulate the interval-skip behavior that NodeRecorder implements:
    // Only record snapshots at ticks that are multiples of the interval.
    let interval = 3u64;
    let mut recording = NodeRecording::new("test_node", "n001", "interval_test");

    for tick in 0..10u64 {
        // This mirrors NodeRecorder::begin_tick() logic: only record on multiples
        if tick % interval == 0 {
            recording.add_snapshot(NodeTickSnapshot::new(tick).with_duration(1000));
        }
    }

    // With interval=3, ticks 0, 3, 6, 9 should be recorded
    assert_eq!(
        recording.snapshot_count(),
        4,
        "With interval=3, 10 ticks should yield 4 snapshots (0,3,6,9), got {}",
        recording.snapshot_count()
    );

    let ticks: Vec<u64> = recording.snapshots.iter().map(|s| s.tick).collect();
    assert_eq!(ticks, vec![0, 3, 6, 9]);
}

#[test]
fn test_node_replayer_advance_seek_reset() {
    let mut recording = NodeRecording::new("test_node", "n001", "replay_test");

    for tick in 0..5u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick * 10) // ticks: 0, 10, 20, 30, 40
                .with_output("data", vec![tick as u8])
                .with_duration(100),
        );
    }

    let mut replayer = NodeReplayer::from_recording(recording);

    // Initial state
    assert!(!replayer.is_finished());
    assert_eq!(replayer.current_tick(), 0);
    assert_eq!(replayer.total_ticks(), 5);

    // Check current snapshot
    let snap = replayer.current_snapshot().unwrap();
    assert_eq!(snap.tick, 0);

    // Advance
    assert!(replayer.advance());
    assert_eq!(replayer.current_tick(), 10);

    assert!(replayer.advance());
    assert_eq!(replayer.current_tick(), 20);

    // Seek to tick 30
    assert!(replayer.seek(30));
    assert_eq!(replayer.current_tick(), 30);

    // Seek to tick 35 → should land on tick 40 (next available)
    assert!(replayer.seek(35));
    assert_eq!(replayer.current_tick(), 40);

    // Advance past end — advance() returns false when no more ticks
    assert!(!replayer.advance());
    // Replayer stays on the last valid snapshot (index=4), so is_finished() is false
    // (is_finished means current_index >= snapshots.len(), which only happens
    //  if something pushes past the last element)
    assert_eq!(replayer.current_tick(), 40, "Should stay on last tick");

    // Reset
    replayer.reset();
    assert_eq!(replayer.current_tick(), 0);
    assert!(!replayer.is_finished());

    // Verify output data
    let output = replayer.output("data").unwrap();
    assert_eq!(output, &vec![0u8]);
}

#[test]
fn test_recording_manager_list_sessions() {
    let base_dir = std::env::temp_dir().join(format!("horus_manager_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&base_dir);
    std::fs::create_dir_all(&base_dir).unwrap();

    // Create mock session directories
    std::fs::create_dir_all(base_dir.join("session_alpha")).unwrap();
    std::fs::create_dir_all(base_dir.join("session_beta")).unwrap();
    std::fs::create_dir_all(base_dir.join("session_gamma")).unwrap();

    let manager = RecordingManager::with_base_dir(base_dir.clone());
    let sessions = manager.list_sessions().expect("should list sessions");

    assert_eq!(sessions.len(), 3, "Should find 3 sessions");
    assert!(sessions.contains(&"session_alpha".to_string()));
    assert!(sessions.contains(&"session_beta".to_string()));
    assert!(sessions.contains(&"session_gamma".to_string()));

    let _ = std::fs::remove_dir_all(&base_dir);
}
