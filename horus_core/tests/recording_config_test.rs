// Regression test: verify RecordingConfigYaml fields are wired through to the recording system
use horus_core::scheduling::{NodeRecorder, RecordingConfig, RecordingConfigYaml};

mod common;

/// Test that record_inputs=false produces snapshots with empty inputs
#[test]
fn test_record_inputs_disabled() {
    let mut config = RecordingConfig::default();
    config.record_inputs = false;

    let mut recorder = NodeRecorder::new("test_node", "test-id-1", config);

    recorder.begin_tick(0);
    recorder.record_input("sensor.data", vec![1, 2, 3]);
    recorder.record_output("motor.cmd", vec![4, 5, 6]);
    recorder.end_tick(1000);

    let recording = recorder.recording();
    let snap = &recording.snapshots[0];
    assert!(
        snap.inputs.is_empty(),
        "inputs should be empty when record_inputs=false"
    );
    assert!(!snap.outputs.is_empty(), "outputs should still be recorded");
}

/// Test that record_outputs=false produces snapshots with empty outputs
#[test]
fn test_record_outputs_disabled() {
    let mut config = RecordingConfig::default();
    config.record_outputs = false;

    let mut recorder = NodeRecorder::new("test_node", "test-id-2", config);

    recorder.begin_tick(0);
    recorder.record_input("sensor.data", vec![1, 2, 3]);
    recorder.record_output("motor.cmd", vec![4, 5, 6]);
    recorder.end_tick(1000);

    let recording = recorder.recording();
    let snap = &recording.snapshots[0];
    assert!(!snap.inputs.is_empty(), "inputs should still be recorded");
    assert!(
        snap.outputs.is_empty(),
        "outputs should be empty when record_outputs=false"
    );
}

/// Test that record_timing=false produces snapshots with zero duration
#[test]
fn test_record_timing_disabled() {
    let mut config = RecordingConfig::default();
    config.record_timing = false;

    let mut recorder = NodeRecorder::new("test_node", "test-id-3", config);

    recorder.begin_tick(0);
    recorder.end_tick(99999); // pass non-zero duration

    let recording = recorder.recording();
    let snap = &recording.snapshots[0];
    assert_eq!(
        snap.duration_ns, 0,
        "duration_ns should be 0 when record_timing=false"
    );
}

/// Test that all three flags enabled (default) produces full recordings
#[test]
fn test_all_recording_enabled() {
    let config = RecordingConfig::default();
    assert!(config.record_inputs);
    assert!(config.record_outputs);
    assert!(config.record_timing);

    let mut recorder = NodeRecorder::new("test_node", "test-id-4", config);

    recorder.begin_tick(0);
    recorder.record_input("sensor.data", vec![1, 2, 3]);
    recorder.record_output("motor.cmd", vec![4, 5, 6]);
    recorder.end_tick(50000);

    let recording = recorder.recording();
    let snap = &recording.snapshots[0];
    assert!(
        !snap.inputs.is_empty(),
        "inputs should be recorded by default"
    );
    assert!(
        !snap.outputs.is_empty(),
        "outputs should be recorded by default"
    );
    assert_eq!(
        snap.duration_ns, 50000,
        "duration should be recorded by default"
    );
}

/// Test From<RecordingConfigYaml> correctly transfers recording flags
#[test]
fn test_from_yaml_transfers_flags() {
    let mut yaml = RecordingConfigYaml::default();
    yaml.record_inputs = false;
    yaml.record_outputs = false;
    yaml.record_timing = false;
    yaml.max_size_mb = 42;

    let config: RecordingConfig = yaml.into();
    assert!(!config.record_inputs);
    assert!(!config.record_outputs);
    assert!(!config.record_timing);
    assert_eq!(config.max_size, 42 * 1024 * 1024);
}
