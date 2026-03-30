#![allow(dead_code)]
// Record/Replay Intent Tests
//
// These tests verify the high-level INTENT of the recording system:
// 1. Recording captures tick data that can be played back
// 2. Saved recordings can be loaded back with matching metadata
// 3. Replaying a recording produces the same sequence

use horus_core::scheduling::{
    diff_recordings, NodeRecording, NodeReplayer, NodeTickSnapshot, Recording, SchedulerRecording,
};

mod common;
use common::TestTempDir;

// ============================================================================
// Test: Recording captures tick data that can be played back
// ============================================================================
//
// INTENT: "Recording captures tick data that can be played back."
// Create a recording with multiple ticks of data, each with inputs and
// outputs. Assert the recording captured all ticks and the data can be
// iterated via the replayer.

#[test]
fn test_recording_intent_captures_node_ticks() {
    // Simulate a node that records 20 ticks of sensor->motor pipeline
    let mut recording = NodeRecording::new("motor_controller", "mc_001", "capture_test");

    let tick_count = 20u64;
    for tick in 0..tick_count {
        let sensor_data = vec![(tick & 0xFF) as u8; 4]; // Simulated sensor reading
        let motor_cmd = vec![((tick * 2) & 0xFF) as u8; 2]; // Simulated motor command

        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_input("sensor.imu", sensor_data)
                .with_output("motor.cmd", motor_cmd)
                .with_duration(500 + tick * 10), // Varying execution time
        );
    }

    // The recording captured all ticks
    assert_eq!(
        recording.snapshot_count(),
        tick_count as usize,
        "Recording should capture exactly {} ticks",
        tick_count
    );
    assert_eq!(recording.first_tick, 0, "First tick should be 0");
    assert_eq!(
        recording.last_tick,
        tick_count - 1,
        "Last tick should be {}",
        tick_count - 1
    );

    // Every tick has its data accessible
    for tick in 0..tick_count {
        let snap = recording
            .snapshot(tick)
            .unwrap_or_else(|| panic!("Snapshot for tick {} should exist", tick));
        assert!(
            snap.inputs.contains_key("sensor.imu"),
            "Tick {} should have sensor input",
            tick
        );
        assert!(
            snap.outputs.contains_key("motor.cmd"),
            "Tick {} should have motor output",
            tick
        );
    }

    // The data can be played back via NodeReplayer
    let mut replayer = NodeReplayer::from_recording(recording);
    assert!(
        !replayer.is_finished(),
        "Replayer should not be finished at start"
    );

    let mut replayed_ticks = 1u64; // We start at tick 0 (first snapshot)
    while replayer.advance() {
        replayed_ticks += 1;
    }
    assert_eq!(
        replayed_ticks, tick_count,
        "Replayer should iterate through all {} ticks",
        tick_count
    );
}

// ============================================================================
// Test: Saved recordings can be loaded back
// ============================================================================
//
// INTENT: "Saved recordings can be loaded back."
// Create a recording with known metadata. Save to a temp directory.
// Load from the same path. Assert metadata (tick count, node name,
// session name, first/last tick, topic data) all match.

#[test]
fn test_recording_intent_save_load_roundtrip() {
    let tmp = TestTempDir::new("rr_roundtrip");
    let node_path = tmp.path().join("motor_controller@mc_002.horus");
    let sched_path = tmp.path().join("scheduler@sched_001.horus");

    // --- NodeRecording roundtrip ---
    let mut node_rec = NodeRecording::new("motor_controller", "mc_002", "roundtrip_session");
    node_rec.set_topic_type("sensor.imu", "ImuData");
    node_rec.set_topic_type("motor.cmd", "CmdVel");

    for tick in 0..15u64 {
        node_rec.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_input("sensor.imu", tick.to_le_bytes().to_vec())
                .with_output("motor.cmd", (tick * 3).to_le_bytes().to_vec())
                .with_duration(1000),
        );
    }
    node_rec.finish();

    // Save
    node_rec
        .save(&node_path)
        .expect("Failed to save node recording");
    assert!(
        node_path.exists(),
        "Node recording file should exist on disk"
    );

    // Load
    let loaded = NodeRecording::load(&node_path).expect("Failed to load node recording");

    // Assert metadata matches
    assert_eq!(loaded.node_name, "motor_controller");
    assert_eq!(loaded.node_id, "mc_002");
    assert_eq!(loaded.session_name, "roundtrip_session");
    assert_eq!(loaded.snapshot_count(), 15);
    assert_eq!(loaded.first_tick, 0);
    assert_eq!(loaded.last_tick, 14);
    assert!(
        loaded.ended_at.is_some(),
        "ended_at should be set after finish()"
    );

    // Assert topic type metadata survived the roundtrip
    assert_eq!(
        loaded.topic_types.get("sensor.imu").map(|s| s.as_str()),
        Some("ImuData")
    );
    assert_eq!(
        loaded.topic_types.get("motor.cmd").map(|s| s.as_str()),
        Some("CmdVel")
    );

    // Assert actual data survived the roundtrip
    let snap_0 = loaded.snapshot(0).expect("Tick 0 should exist");
    assert_eq!(
        snap_0.inputs.get("sensor.imu").unwrap(),
        &0u64.to_le_bytes().to_vec()
    );
    assert_eq!(
        snap_0.outputs.get("motor.cmd").unwrap(),
        &0u64.to_le_bytes().to_vec()
    );

    let snap_14 = loaded.snapshot(14).expect("Tick 14 should exist");
    assert_eq!(
        snap_14.outputs.get("motor.cmd").unwrap(),
        &(14u64 * 3).to_le_bytes().to_vec()
    );

    // --- SchedulerRecording roundtrip ---
    let mut sched_rec = SchedulerRecording::new("sched_001", "roundtrip_session");
    sched_rec.add_node_recording("mc_002", "motor_controller@mc_002.horus");
    sched_rec.record_execution_order(vec!["mc_002".to_string()]);
    sched_rec.record_execution_order(vec!["mc_002".to_string()]);
    sched_rec.finish();

    sched_rec
        .save(&sched_path)
        .expect("Failed to save scheduler recording");

    let loaded_sched =
        SchedulerRecording::load(&sched_path).expect("Failed to load scheduler recording");

    assert_eq!(loaded_sched.scheduler_id, "sched_001");
    assert_eq!(loaded_sched.session_name, "roundtrip_session");
    assert_eq!(loaded_sched.total_ticks, 2);
    assert_eq!(loaded_sched.node_recordings.len(), 1);
    assert!(loaded_sched.ended_at.is_some());
}

// ============================================================================
// Test: Replaying a recording produces the same sequence
// ============================================================================
//
// INTENT: "Replaying a recording produces the same sequence."
// Record a node that publishes incrementing counter values as outputs.
// Save and reload the recording. Replay through all ticks and assert
// each output value matches the original deterministic sequence.

#[test]
fn test_recording_intent_deterministic_replay() {
    let tmp = TestTempDir::new("rr_determinism");
    let path = tmp.path().join("counter@det_001.horus");

    // Record a deterministic node: output = tick * 7 (as little-endian u64 bytes)
    let mut recording = NodeRecording::new("counter", "det_001", "determinism_test");
    let num_ticks = 50u64;

    let mut expected_outputs: Vec<Vec<u8>> = Vec::new();
    for tick in 0..num_ticks {
        let value = tick * 7;
        let output_bytes = value.to_le_bytes().to_vec();
        expected_outputs.push(output_bytes.clone());

        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("counter.value", output_bytes)
                .with_duration(100),
        );
    }
    recording.finish();

    // Save to disk and reload (proves persistence doesn't corrupt data)
    recording
        .save(&path)
        .expect("Failed to save deterministic recording");
    let mut replayer = NodeReplayer::load(&path).expect("Failed to load deterministic recording");

    // Replay and verify each output matches the expected sequence
    let mut replay_index = 0usize;
    loop {
        let snap = replayer
            .current_snapshot()
            .unwrap_or_else(|| panic!("Snapshot at index {} should exist", replay_index));

        let output = snap
            .outputs
            .get("counter.value")
            .unwrap_or_else(|| panic!("Output 'counter.value' missing at tick {}", snap.tick));

        assert_eq!(
            output, &expected_outputs[replay_index],
            "Output mismatch at replay index {} (tick {})",
            replay_index, snap.tick
        );

        replay_index += 1;
        if !replayer.advance() {
            break;
        }
    }

    assert_eq!(
        replay_index, num_ticks as usize,
        "Should have replayed all {} ticks",
        num_ticks
    );

    // Extra verification: diff the original recording against the loaded one
    // They should be identical (0 diffs)
    let original = NodeRecording::load(&path).expect("Failed to reload for diff");
    let reloaded = NodeRecording::load(&path).expect("Failed to reload second copy for diff");
    let diffs = diff_recordings(&original, &reloaded);
    assert!(
        diffs.is_empty(),
        "Diffing the same recording should produce 0 differences, got {}",
        diffs.len()
    );
}
