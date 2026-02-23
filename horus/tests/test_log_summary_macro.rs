//! Integration tests for the `#[derive(LogSummary)]` proc macro.
//!
//! Verifies that the derive generates a `LogSummary` trait implementation
//! that delegates to `Debug` formatting.

use horus::prelude::*;

// ============================================================================
// 1. Simple struct with named fields
// ============================================================================

#[derive(Debug, LogSummary)]
struct SensorReading {
    x: f64,
    y: f64,
    z: f64,
}

#[test]
fn test_log_summary_simple_struct() {
    let reading = SensorReading {
        x: 1.0,
        y: 2.5,
        z: -0.3,
    };
    let summary = reading.log_summary();
    // Should contain Debug output with field names and values
    assert!(summary.contains("SensorReading"));
    assert!(summary.contains("1.0"));
    assert!(summary.contains("2.5"));
    assert!(summary.contains("-0.3"));
}

// ============================================================================
// 2. Struct with many fields — all appear in summary
// ============================================================================

#[derive(Debug, LogSummary)]
struct RobotState {
    position_x: f64,
    position_y: f64,
    heading: f64,
    velocity: f64,
    battery: u8,
    mode: String,
}

#[test]
fn test_log_summary_many_fields() {
    let state = RobotState {
        position_x: 10.5,
        position_y: -3.2,
        heading: 1.57,
        velocity: 0.8,
        battery: 85,
        mode: "autonomous".to_string(),
    };
    let summary = state.log_summary();
    assert!(summary.contains("10.5"));
    assert!(summary.contains("-3.2"));
    assert!(summary.contains("1.57"));
    assert!(summary.contains("0.8"));
    assert!(summary.contains("85"));
    assert!(summary.contains("autonomous"));
}

// ============================================================================
// 3. Enum with variants
// ============================================================================

#[derive(Debug, LogSummary)]
enum MotorState {
    Idle,
    Running,
    Error(String),
}

#[test]
fn test_log_summary_enum_unit_variant() {
    let state = MotorState::Idle;
    let summary = state.log_summary();
    assert!(summary.contains("Idle"));
}

#[test]
fn test_log_summary_enum_running_variant() {
    let state = MotorState::Running;
    let summary = state.log_summary();
    assert!(summary.contains("Running"));
}

#[test]
fn test_log_summary_enum_data_variant() {
    let state = MotorState::Error("overcurrent".to_string());
    let summary = state.log_summary();
    assert!(summary.contains("Error"));
    assert!(summary.contains("overcurrent"));
}

// ============================================================================
// 4. Nested structs
// ============================================================================

#[derive(Debug, LogSummary)]
struct Pose {
    x: f64,
    y: f64,
    theta: f64,
}

#[derive(Debug, LogSummary)]
struct LocalizationResult {
    pose: Pose,
    confidence: f64,
    timestamp_ms: u64,
}

#[test]
fn test_log_summary_nested_struct() {
    let result = LocalizationResult {
        pose: Pose {
            x: 5.0,
            y: 10.0,
            theta: 0.785,
        },
        confidence: 0.95,
        timestamp_ms: 1234567890,
    };
    let summary = result.log_summary();
    // Debug output includes nested struct fields
    assert!(summary.contains("5.0"));
    assert!(summary.contains("10.0"));
    assert!(summary.contains("0.785"));
    assert!(summary.contains("0.95"));
    assert!(summary.contains("1234567890"));
}

// ============================================================================
// 5. Tuple struct
// ============================================================================

#[derive(Debug, LogSummary)]
struct Velocity(f64, f64);

#[test]
fn test_log_summary_tuple_struct() {
    let vel = Velocity(1.5, -0.3);
    let summary = vel.log_summary();
    assert!(summary.contains("Velocity"));
    assert!(summary.contains("1.5"));
    assert!(summary.contains("-0.3"));
}

// ============================================================================
// 6. Unit struct
// ============================================================================

#[derive(Debug, LogSummary)]
struct EmergencyStop;

#[test]
fn test_log_summary_unit_struct() {
    let stop = EmergencyStop;
    let summary = stop.log_summary();
    assert!(summary.contains("EmergencyStop"));
}

// ============================================================================
// 7. Verify return type is String
// ============================================================================

#[test]
fn test_log_summary_returns_string() {
    let reading = SensorReading {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let summary: String = reading.log_summary();
    assert!(!summary.is_empty());
}
