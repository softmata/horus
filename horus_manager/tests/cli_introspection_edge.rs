#![allow(dead_code)]
//! Edge-case tests for CLI introspection commands (msg list, msg info, msg hash).
//!
//! These tests exercise the `horus msg` subcommand family through the compiled
//! binary without requiring a running HORUS system. They create temporary
//! message source directories and point `HORUS_SOURCE_DIR` at them so the
//! message discovery logic finds the synthetic files.

use assert_cmd::cargo::cargo_bin_cmd;
use assert_cmd::Command;
use predicates::prelude::*;
use std::fs;
use tempfile::TempDir;

/// Helper to get the CLI command.
fn horus_cmd() -> Command {
    cargo_bin_cmd!("horus")
}

/// Create a minimal temp messages directory with a given set of `.rs` source
/// files.  Returns the `TempDir` guard (must be kept alive) and the root path
/// to use for `HORUS_SOURCE_DIR`.
fn setup_temp_messages(files: &[(&str, &str)]) -> (TempDir, String) {
    let tmp = TempDir::new().expect("failed to create temp dir");
    let msgs_dir = tmp.path().join("horus_library").join("messages");
    fs::create_dir_all(&msgs_dir).expect("failed to create messages dir");

    for (name, content) in files {
        fs::write(msgs_dir.join(name), content).expect("failed to write source file");
    }

    let root = tmp.path().to_str().unwrap().to_string();
    (tmp, root)
}

/// A rich set of fake message files spanning many "modules" so we can assert
/// the list command returns a large number of types.
fn rich_message_files() -> Vec<(&'static str, &'static str)> {
    vec![
        (
            "control.rs",
            r#"
/// Velocity command (2D differential drive)
pub struct CmdVel {
    pub timestamp_ns: u64,
    pub linear: f32,
    pub angular: f32,
}

/// PID configuration
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

/// Motor command
pub struct MotorCommand {
    pub velocity: f64,
    pub torque: f64,
}

/// Servo command
pub struct ServoCommand {
    pub angle: f64,
    pub speed: f64,
}

/// Joint command
pub struct JointCommand {
    pub positions: f64,
}

/// Differential drive
pub struct DifferentialDriveCommand {
    pub left: f64,
    pub right: f64,
}

/// Trajectory point
pub struct TrajectoryPoint {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}
"#,
        ),
        (
            "geometry.rs",
            r#"
/// 3D vector
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 3D point
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Quaternion
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// 2D pose
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// 3D pose
pub struct Pose3D {
    pub position: f64,
    pub orientation: f64,
}

/// Stamped pose
pub struct PoseStamped {
    pub timestamp_ns: u64,
    pub x: f64,
}

/// 6-DOF twist
pub struct Twist {
    pub linear: f64,
    pub angular: f64,
}

/// Stamped transform
pub struct TransformStamped {
    pub timestamp_ns: u64,
    pub translation: f64,
}

/// Acceleration
pub struct Accel {
    pub linear: f64,
    pub angular: f64,
}

/// Stamped acceleration
pub struct AccelStamped {
    pub timestamp_ns: u64,
    pub accel: f64,
}

/// Pose with covariance
pub struct PoseWithCovariance {
    pub x: f64,
    pub covariance: f64,
}

/// Twist with covariance
pub struct TwistWithCovariance {
    pub twist: f64,
    pub covariance: f64,
}
"#,
        ),
        (
            "sensor.rs",
            r#"
/// Laser scan data
pub struct LaserScan {
    pub angle_min: f32,
    pub angle_max: f32,
    pub range_min: f32,
    pub range_max: f32,
}

/// Inertial measurement unit
pub struct Imu {
    pub accel_x: f64,
    pub accel_y: f64,
    pub accel_z: f64,
    pub gyro_x: f64,
    pub gyro_y: f64,
    pub gyro_z: f64,
}

/// Wheel/visual odometry
pub struct Odometry {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// Battery state
pub struct BatteryState {
    pub voltage: f64,
    pub current: f64,
    pub percentage: f64,
}

/// Temperature
pub struct Temperature {
    pub celsius: f64,
}

/// Fluid pressure
pub struct FluidPressure {
    pub pascal: f64,
}

/// Illuminance
pub struct Illuminance {
    pub lux: f64,
}

/// Magnetic field
pub struct MagneticField {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// GPS fix
pub struct NavSatFix {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

/// Range sensor (sonar/IR)
pub struct RangeSensor {
    pub distance: f64,
    pub min_range: f64,
    pub max_range: f64,
}

/// Joint state
pub struct JointState {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
}
"#,
        ),
        (
            "diagnostics.rs",
            r#"
/// Heartbeat
pub struct Heartbeat {
    pub timestamp_ns: u64,
    pub seq: u64,
}

/// Emergency stop
pub struct EmergencyStop {
    pub active: u8,
}

/// Diagnostic status
pub struct DiagnosticStatus {
    pub level: u8,
}

/// Diagnostic report
pub struct DiagnosticReport {
    pub timestamp_ns: u64,
}

/// Safety status
pub struct SafetyStatus {
    pub safe: u8,
}

/// Node heartbeat
pub struct NodeHeartbeat {
    pub seq: u64,
}

/// Node state
pub struct NodeStateMsg {
    pub state: u8,
}

/// Resource usage
pub struct ResourceUsage {
    pub cpu: f64,
    pub memory: f64,
}
"#,
        ),
        (
            "navigation.rs",
            r#"
/// Navigation goal
pub struct NavGoal {
    pub x: f64,
    pub y: f64,
}

/// Navigation path
pub struct NavPath {
    pub length: f64,
}

/// Occupancy grid
pub struct OccupancyGrid {
    pub width: u32,
    pub height: u32,
}

/// Waypoint
pub struct Waypoint {
    pub x: f64,
    pub y: f64,
}

/// Path plan
pub struct PathPlan {
    pub cost: f64,
}

/// Cost map
pub struct CostMap {
    pub resolution: f64,
}

/// Goal result
pub struct GoalResult {
    pub success: u8,
}

/// Velocity obstacle
pub struct VelocityObstacle {
    pub vx: f64,
    pub vy: f64,
}
"#,
        ),
        (
            "vision.rs",
            r#"
/// Camera info
pub struct CameraInfo {
    pub width: u32,
    pub height: u32,
}

/// Compressed image
pub struct CompressedImage {
    pub format: u8,
}

/// Region of interest
pub struct RegionOfInterest {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

/// Stereo info
pub struct StereoInfo {
    pub baseline: f64,
}
"#,
        ),
        (
            "force.rs",
            r#"
/// Wrench (force + torque) stamped
pub struct WrenchStamped {
    pub force_x: f64,
    pub torque_z: f64,
}

/// Contact info
pub struct ContactInfo {
    pub force: f64,
}

/// Force command
pub struct ForceCommand {
    pub magnitude: f64,
}

/// Haptic feedback
pub struct HapticFeedback {
    pub amplitude: f64,
}

/// Impedance parameters
pub struct ImpedanceParameters {
    pub stiffness: f64,
}
"#,
        ),
        (
            "perception.rs",
            r#"
/// Point field
pub struct PointField {
    pub name: u8,
}

/// Plane detection
pub struct PlaneDetection {
    pub normal_x: f64,
}

/// Plane array
pub struct PlaneArray {
    pub count: u32,
}
"#,
        ),
    ]
}

// ============================================================================
// horus msg list — basic behaviour
// ============================================================================

#[test]
fn test_msg_list_returns_all_message_types() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "list", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg list");

    assert!(
        output.status.success(),
        "horus msg list --json should succeed, stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value =
        serde_json::from_str(&stdout).expect("output should be valid JSON");

    let count = json["count"].as_u64().expect("count field should exist");
    // We defined 50+ types across the synthetic files
    assert!(
        count >= 50,
        "expected at least 50 message types, got {count}"
    );

    // Spot-check a few known types are present
    let items = json["items"].as_array().expect("items should be an array");
    let names: Vec<&str> = items.iter().filter_map(|i| i["name"].as_str()).collect();

    for expected in &[
        "CmdVel",
        "Imu",
        "LaserScan",
        "Twist",
        "Odometry",
        "Heartbeat",
        "NavGoal",
        "CameraInfo",
        "WrenchStamped",
        "PointField",
    ] {
        assert!(
            names.contains(expected),
            "expected message type '{expected}' not found in list. Got: {names:?}"
        );
    }
}

// ============================================================================
// horus msg info — known type
// ============================================================================

#[test]
fn test_msg_show_known_type() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "info", "CmdVel", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg info");

    assert!(
        output.status.success(),
        "horus msg info CmdVel --json should succeed, stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value =
        serde_json::from_str(&stdout).expect("output should be valid JSON");

    assert_eq!(json["name"].as_str(), Some("CmdVel"));
    assert_eq!(json["module"].as_str(), Some("control"));

    let fields = json["fields"].as_array().expect("fields should be array");
    let field_names: Vec<&str> = fields.iter().filter_map(|f| f["name"].as_str()).collect();

    assert!(
        field_names.contains(&"linear"),
        "CmdVel should have a 'linear' field, got: {field_names:?}"
    );
    assert!(
        field_names.contains(&"angular"),
        "CmdVel should have an 'angular' field, got: {field_names:?}"
    );
}

// ============================================================================
// horus msg info — unknown type returns error
// ============================================================================

#[test]
fn test_msg_show_unknown_type_returns_error() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    horus_cmd()
        .args(["msg", "info", "NonexistentType"])
        .env("HORUS_SOURCE_DIR", &root)
        .assert()
        .failure()
        .stderr(predicate::str::contains("NonexistentType"));
}

// ============================================================================
// horus msg info — case-insensitive lookup
// ============================================================================

#[test]
fn test_msg_info_case_insensitive() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    // Lowercase "imu" should match "Imu"
    horus_cmd()
        .args(["msg", "info", "imu", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .assert()
        .success()
        .stdout(predicate::str::contains("\"name\""));
}

// ============================================================================
// horus msg info — module-qualified name
// ============================================================================

#[test]
fn test_msg_info_module_qualified() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    horus_cmd()
        .args(["msg", "info", "sensor::LaserScan", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .assert()
        .success()
        .stdout(predicate::str::contains("\"name\""));
}

// ============================================================================
// horus msg list — filter narrows results
// ============================================================================

#[test]
fn test_msg_list_filter_narrows_results() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "list", "--json", "--filter", "sensor"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg list with filter");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let count = json["count"].as_u64().unwrap();

    // "sensor" module has 11 types; total is 50+
    assert!(
        count > 0 && count < 50,
        "filter 'sensor' should narrow results (got {count})"
    );

    // All returned items should be from the sensor module
    let items = json["items"].as_array().unwrap();
    for item in items {
        let module = item["module"].as_str().unwrap_or("");
        let name = item["name"].as_str().unwrap_or("");
        assert!(
            module.contains("sensor") || name.to_lowercase().contains("sensor"),
            "filtered item '{}::{}' should match 'sensor'",
            module,
            name
        );
    }
}

// ============================================================================
// horus msg list — filter with no match returns empty
// ============================================================================

#[test]
fn test_msg_list_filter_no_match() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "list", "--json", "--filter", "zzz_nonexistent_zzz"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg list");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let count = json["count"].as_u64().unwrap();
    assert_eq!(count, 0, "nonsensical filter should return 0 items");
}

// ============================================================================
// horus msg list — verbose mode does not crash
// ============================================================================

#[test]
fn test_msg_list_verbose_succeeds() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    horus_cmd()
        .args(["msg", "list", "--verbose"])
        .env("HORUS_SOURCE_DIR", &root)
        .assert()
        .success()
        .stdout(predicate::str::contains("Message Type"));
}

// ============================================================================
// horus msg hash — known type returns 16-char hex
// ============================================================================

#[test]
fn test_msg_hash_known_type() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "hash", "Twist"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg hash");

    assert!(output.status.success());

    let hash = String::from_utf8_lossy(&output.stdout).trim().to_string();
    assert_eq!(hash.len(), 16, "hash should be 16 hex chars, got '{hash}'");
    assert!(
        hash.chars().all(|c| c.is_ascii_hexdigit()),
        "hash should be valid hex, got '{hash}'"
    );
}

// ============================================================================
// horus msg hash — unknown type returns error
// ============================================================================

#[test]
fn test_msg_hash_unknown_type_returns_error() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    horus_cmd()
        .args(["msg", "hash", "DoesNotExist"])
        .env("HORUS_SOURCE_DIR", &root)
        .assert()
        .failure()
        .stderr(predicate::str::contains("DoesNotExist"));
}

// ============================================================================
// horus msg hash — json mode includes name + module + hash
// ============================================================================

#[test]
fn test_msg_hash_json_output() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "hash", "Imu", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg hash --json");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();

    assert_eq!(json["name"].as_str(), Some("Imu"));
    assert_eq!(json["module"].as_str(), Some("sensor"));
    assert!(
        json["hash"].as_str().is_some(),
        "JSON output should contain a hash field"
    );
}

// ============================================================================
// horus msg info — empty struct (no fields)
// ============================================================================

#[test]
fn test_msg_info_empty_struct() {
    let files = vec![("empty.rs", "pub struct Marker {}\n")];
    let (_tmp, root) = setup_temp_messages(&files);

    let output = horus_cmd()
        .args(["msg", "info", "Marker", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg info");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();

    assert_eq!(json["name"].as_str(), Some("Marker"));
    let fields = json["fields"].as_array().unwrap();
    assert!(
        fields.is_empty(),
        "Marker should have no fields, got {:?}",
        fields
    );
}

// ============================================================================
// horus msg list — empty messages directory
// ============================================================================

#[test]
fn test_msg_list_empty_dir() {
    let (_tmp, root) = setup_temp_messages(&[]);

    // Should succeed but report 0 types
    horus_cmd()
        .args(["msg", "list"])
        .env("HORUS_SOURCE_DIR", &root)
        .assert()
        .success()
        .stdout(predicate::str::contains("No message types found"));
}

// ============================================================================
// horus msg list — mod.rs is skipped
// ============================================================================

#[test]
fn test_msg_list_skips_mod_rs() {
    let files = vec![
        ("mod.rs", "pub mod sensor;\npub mod control;\n"),
        ("sensor.rs", "pub struct OnlyType { pub x: f64, }\n"),
    ];
    let (_tmp, root) = setup_temp_messages(&files);

    let output = horus_cmd()
        .args(["msg", "list", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg list");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let count = json["count"].as_u64().unwrap();

    // Only sensor.rs contributes; mod.rs is skipped even though it has "pub mod" lines
    assert_eq!(count, 1, "only 1 type from sensor.rs expected, got {count}");

    let items = json["items"].as_array().unwrap();
    assert_eq!(items[0]["name"].as_str(), Some("OnlyType"));
}

// ============================================================================
// horus msg hash — deterministic across runs
// ============================================================================

#[test]
fn test_msg_hash_deterministic() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let run = |root: &str| -> String {
        let output = horus_cmd()
            .args(["msg", "hash", "Twist"])
            .env("HORUS_SOURCE_DIR", root)
            .output()
            .expect("failed to run horus msg hash");
        assert!(output.status.success());
        String::from_utf8_lossy(&output.stdout).trim().to_string()
    };

    let hash1 = run(&root);
    let hash2 = run(&root);
    assert_eq!(
        hash1, hash2,
        "hash should be deterministic across invocations"
    );
}

// ============================================================================
// horus msg list — non-rs files are ignored
// ============================================================================

#[test]
fn test_msg_list_ignores_non_rs_files() {
    let files = vec![("real.rs", "pub struct RealMsg { pub value: f64, }\n")];
    let (_tmp, root) = setup_temp_messages(&files);

    // Add a non-.rs file manually
    let msgs_dir = _tmp.path().join("horus_library").join("messages");
    fs::write(msgs_dir.join("readme.txt"), "not a source file").unwrap();
    fs::write(msgs_dir.join("data.json"), r#"{"key": "value"}"#).unwrap();

    let output = horus_cmd()
        .args(["msg", "list", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg list");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let count = json["count"].as_u64().unwrap();
    assert_eq!(count, 1, "only .rs files should be parsed, got {count}");
}

// ============================================================================
// horus msg info — fields include type information
// ============================================================================

#[test]
fn test_msg_info_field_types_present() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "info", "Imu", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg info");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();

    let fields = json["fields"].as_array().unwrap();
    assert_eq!(
        fields.len(),
        6,
        "Imu should have 6 fields, got {}",
        fields.len()
    );

    // Every field should have name and type
    for field in fields {
        assert!(
            field["name"].as_str().is_some(),
            "field should have a name: {field:?}"
        );
        assert!(
            field["type"].as_str().is_some(),
            "field should have a type: {field:?}"
        );
    }
}

// ============================================================================
// horus msg list — JSON output has expected schema
// ============================================================================

#[test]
fn test_msg_list_json_schema() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let output = horus_cmd()
        .args(["msg", "list", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg list --json");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();

    // Top-level should have "count" and "items"
    assert!(json["count"].is_u64(), "count should be a number");
    assert!(json["items"].is_array(), "items should be an array");

    // Each item should have expected fields
    let items = json["items"].as_array().unwrap();
    assert!(!items.is_empty());

    let first = &items[0];
    assert!(first["name"].is_string(), "item should have 'name'");
    assert!(first["module"].is_string(), "item should have 'module'");
    assert!(
        first["fields"].is_number(),
        "item should have 'fields' count"
    );
    assert!(first["hash"].is_string(), "item should have 'hash'");
}

// ============================================================================
// horus msg info — doc comment is captured
// ============================================================================

#[test]
fn test_msg_info_captures_doc_comment() {
    let files = vec![(
        "documented.rs",
        r#"
/// A well-documented message for testing.
/// It has multiple doc lines.
pub struct DocTest {
    /// The primary value
    pub value: f64,
}
"#,
    )];
    let (_tmp, root) = setup_temp_messages(&files);

    let output = horus_cmd()
        .args(["msg", "info", "DocTest", "--json"])
        .env("HORUS_SOURCE_DIR", &root)
        .output()
        .expect("failed to run horus msg info");

    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout).unwrap();

    let doc = json["doc"].as_str().unwrap_or("");
    assert!(
        doc.contains("well-documented"),
        "doc should contain the doc comment text, got: '{doc}'"
    );
}

// ============================================================================
// horus msg list — compact (non-verbose, non-json) mode succeeds
// ============================================================================

#[test]
fn test_msg_list_compact_mode() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    horus_cmd()
        .args(["msg", "list"])
        .env("HORUS_SOURCE_DIR", &root)
        .assert()
        .success()
        .stdout(predicate::str::contains("Total:"));
}

// ============================================================================
// horus msg hash — different types produce different hashes
// ============================================================================

#[test]
fn test_msg_hash_differs_across_types() {
    let (_tmp, root) = setup_temp_messages(&rich_message_files());

    let get_hash = |name: &str| -> String {
        let output = horus_cmd()
            .args(["msg", "hash", name])
            .env("HORUS_SOURCE_DIR", &root)
            .output()
            .expect("failed to run horus msg hash");
        assert!(output.status.success());
        String::from_utf8_lossy(&output.stdout).trim().to_string()
    };

    let hash_twist = get_hash("Twist");
    let hash_imu = get_hash("Imu");
    let hash_cmdvel = get_hash("CmdVel");

    assert_ne!(
        hash_twist, hash_imu,
        "Twist and Imu should have different hashes"
    );
    assert_ne!(
        hash_twist, hash_cmdvel,
        "Twist and CmdVel should have different hashes"
    );
    assert_ne!(
        hash_imu, hash_cmdvel,
        "Imu and CmdVel should have different hashes"
    );
}
