#![allow(dead_code)]
//! Integration tests for BlackBox and Record/Replay system.
//!
//! BlackBox tests: save/load round-trip, legacy array compat, WAL I/O, disabled mode.
//! Record/Replay tests: config paths, NodeRecording save/load, interval skip,
//! replayer advance/seek/reset, RecordingManager session listing.

mod common;

use common::TestTempDir;
use horus_core::scheduling::{
    diff_recordings, BlackBox, BlackBoxEvent, NodeRecording, NodeReplayer, NodeTickSnapshot,
    Recording, RecordingConfig, RecordingManager,
};
use std::path::PathBuf;

// ============================================================================
// BlackBox tests
// ============================================================================

#[cfg(feature = "blackbox")]
#[test]
fn test_blackbox_save_load_round_trip() {
    let tmp = TestTempDir::new("horus_bb_roundtrip");

    let mut bb = BlackBox::new(1).with_path(tmp.path().to_path_buf());

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
    let mut bb2 = BlackBox::new(1).with_path(tmp.path().to_path_buf());
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
    // tmp is cleaned up automatically on drop
}

#[cfg(feature = "blackbox")]
#[test]
fn test_blackbox_legacy_array_compat() {
    let tmp = TestTempDir::new("horus_bb_legacy");

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
    std::fs::write(tmp.path().join("blackbox.json"), legacy_json).unwrap();

    // Load should succeed with legacy format
    let mut bb = BlackBox::new(1).with_path(tmp.path().to_path_buf());
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
}

#[cfg(feature = "blackbox")]
#[test]
fn test_blackbox_wal_actual_io() {
    let tmp = TestTempDir::new("horus_bb_wal");

    let mut bb = BlackBox::new(1)
        .with_wal_flush_interval(1) // Flush every record for test
        .with_path(tmp.path().to_path_buf());

    // Record events (WAL should be written)
    for i in 0..5 {
        bb.record(BlackBoxEvent::Custom {
            category: "wal_test".to_string(),
            message: format!("event {}", i),
        });
    }
    bb.flush_wal();

    // Verify WAL file exists and has content
    let wal_path = tmp.path().join("blackbox.wal");
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
        max_size_bytes: 0,
        compress: false,
        max_snapshots: 0,
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
    let tmp = TestTempDir::new("horus_recording");

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
    let path = tmp.path().join("test_recording.horus");
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
    let tmp = TestTempDir::new("horus_manager");

    // Create mock session directories
    std::fs::create_dir_all(tmp.path().join("session_alpha")).unwrap();
    std::fs::create_dir_all(tmp.path().join("session_beta")).unwrap();
    std::fs::create_dir_all(tmp.path().join("session_gamma")).unwrap();

    let manager = RecordingManager::with_base_dir(tmp.path().to_path_buf());
    let sessions = manager.list_sessions().expect("should list sessions");

    assert_eq!(sessions.len(), 3, "Should find 3 sessions");
    assert!(sessions.contains(&"session_alpha".to_string()));
    assert!(sessions.contains(&"session_beta".to_string()));
    assert!(sessions.contains(&"session_gamma".to_string()));
}

// ============================================================================
// E2E: Record → Save → Load → Replay → Verify tick-perfect reproduction
// ============================================================================

#[test]
fn test_e2e_record_replay_tick_perfect() {
    let tmp = TestTempDir::new("horus_e2e_replay");
    let path = tmp.path().join("motor@id1.horus");

    // Simulate recording 100 ticks of a motor controller node
    let mut recording = NodeRecording::new("motor_ctrl", "id1", "e2e_test");
    for tick in 0..100u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_input("encoder", tick.to_le_bytes().to_vec())
                .with_output("velocity", (tick * 2).to_le_bytes().to_vec())
                .with_duration(tick * 100),
        );
    }
    recording.finish();
    recording.save(&path).expect("save should succeed");

    // Load and replay — verify every tick matches exactly
    let mut replayer = NodeReplayer::load(&path).expect("load should succeed");
    assert_eq!(replayer.total_ticks(), 100);

    for tick in 0..100u64 {
        let snap = replayer.current_snapshot().expect("should have snapshot");
        assert_eq!(snap.tick, tick, "tick mismatch at {}", tick);
        assert_eq!(
            snap.inputs.get("encoder").unwrap(),
            &tick.to_le_bytes().to_vec(),
            "input mismatch at tick {}",
            tick
        );
        assert_eq!(
            snap.outputs.get("velocity").unwrap(),
            &(tick * 2).to_le_bytes().to_vec(),
            "output mismatch at tick {}",
            tick
        );
        assert_eq!(
            snap.duration_ns,
            tick * 100,
            "duration mismatch at tick {}",
            tick
        );

        if tick < 99 {
            assert!(replayer.advance(), "should advance at tick {}", tick);
        }
    }
}

#[test]
fn test_e2e_diff_detects_divergence() {
    // Record two runs of the same node, with a fault injected in run 2
    let mut run1 = NodeRecording::new("motor_ctrl", "r1", "diff_test");
    let mut run2 = NodeRecording::new("motor_ctrl", "r2", "diff_test");

    for tick in 0..50u64 {
        let normal_output = (tick * 10).to_le_bytes().to_vec();

        run1.add_snapshot(NodeTickSnapshot::new(tick).with_output("cmd", normal_output.clone()));

        // Inject fault at tick 25: run2 produces different output
        let r2_output = if tick == 25 {
            vec![0xFF; 8] // Faulty output
        } else {
            normal_output
        };
        run2.add_snapshot(NodeTickSnapshot::new(tick).with_output("cmd", r2_output));
    }

    let diffs = diff_recordings(&run1, &run2);
    assert_eq!(diffs.len(), 1, "should detect exactly 1 diff at tick 25");
}

#[test]
fn test_e2e_seek_time_travel() {
    let tmp = TestTempDir::new("horus_e2e_seek");
    let path = tmp.path().join("sensor@s1.horus");

    // Record a long run
    let mut recording = NodeRecording::new("lidar", "s1", "seek_test");
    for tick in 0..1000u64 {
        recording.add_snapshot(NodeTickSnapshot::new(tick).with_output("scan", vec![tick as u8]));
    }
    recording.save(&path).expect("save should succeed");

    let mut replayer = NodeReplayer::load(&path).expect("load should succeed");

    // Seek to tick 500
    assert!(replayer.seek(500));
    assert_eq!(replayer.current_tick(), 500);
    assert_eq!(
        replayer
            .current_snapshot()
            .unwrap()
            .outputs
            .get("scan")
            .unwrap(),
        &vec![(500u16 % 256) as u8]
    );

    // Seek backward
    assert!(replayer.seek(100));
    assert_eq!(replayer.current_tick(), 100);

    // Seek to end
    assert!(replayer.seek(999));
    assert_eq!(replayer.current_tick(), 999);
}

#[test]
fn test_e2e_recording_manager_full_lifecycle() {
    let tmp = TestTempDir::new("horus_e2e_manager");
    let manager = RecordingManager::with_base_dir(tmp.path().to_path_buf());

    // Create a session with recordings
    let session_dir = tmp.path().join("crash_2026_03_11");
    std::fs::create_dir_all(&session_dir).unwrap();

    // Save two node recordings in the session
    let mut r1 = NodeRecording::new("motor", "m1", "crash_2026_03_11");
    r1.add_snapshot(NodeTickSnapshot::new(0).with_output("cmd", vec![1]));
    r1.save(&session_dir.join("motor@m1.horus")).unwrap();

    let mut r2 = NodeRecording::new("sensor", "s1", "crash_2026_03_11");
    r2.add_snapshot(NodeTickSnapshot::new(0).with_output("data", vec![2]));
    r2.save(&session_dir.join("sensor@s1.horus")).unwrap();

    // List sessions
    let sessions = manager.list_sessions().unwrap();
    assert_eq!(sessions.len(), 1);
    assert!(sessions.contains(&"crash_2026_03_11".to_string()));

    // List recordings in session
    let recordings = manager.session_recordings("crash_2026_03_11").unwrap();
    assert_eq!(recordings.len(), 2);

    // Total size should be > 0
    let total = manager.total_size().unwrap();
    assert!(total > 0, "total size should be non-zero");

    // Delete session
    manager.delete_session("crash_2026_03_11").unwrap();
    let sessions = manager.list_sessions().unwrap();
    assert!(sessions.is_empty());
}

#[test]
fn test_e2e_versioned_format_backward_compat() {
    let tmp = TestTempDir::new("horus_e2e_compat");

    // Write a "legacy" recording (no magic/version header, just raw bincode)
    let mut rec = NodeRecording::new("legacy_node", "l1", "legacy_session");
    rec.add_snapshot(NodeTickSnapshot::new(0).with_output("out", vec![42]));

    let legacy_path = tmp.path().join("legacy.horus");
    {
        use std::io::Write;
        let file = std::fs::File::create(&legacy_path).unwrap();
        let mut writer = std::io::BufWriter::new(file);
        bincode::serialize_into(&mut writer, &rec).unwrap();
        writer.flush().unwrap();
    }

    // Load should fall back to legacy format
    let loaded = NodeRecording::load(&legacy_path).expect("should load legacy format");
    assert_eq!(loaded.node_name, "legacy_node");
    assert_eq!(loaded.snapshot_count(), 1);
}
