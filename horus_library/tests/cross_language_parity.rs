//! Cross-language serialization parity tests.
//!
//! Verifies that Rust message types serialize to JSON with stable field names
//! and structure that Python (and future C++) bindings can rely on. If a field
//! is renamed in Rust, these tests break — preventing silent cross-language
//! data corruption.
//!
//! Approach: create messages with known values, serialize to JSON, parse as
//! serde_json::Value, and verify exact field names and values are present.
//! This is the Rust side of parity — Python tests in test_cross_language_parity.py
//! verify the same field names on the Python side.

use horus_library::messages::*;

/// Assert a JSON value has a specific field with a specific numeric value
fn assert_json_field_f64(json: &serde_json::Value, field: &str, expected: f64, msg_type: &str) {
    let val = json
        .get(field)
        .unwrap_or_else(|| panic!("{}: missing field '{}'", msg_type, field));
    let num = val
        .as_f64()
        .unwrap_or_else(|| panic!("{}: field '{}' is not a number: {:?}", msg_type, field, val));
    assert!(
        (num - expected).abs() < 1e-6,
        "{}: field '{}' expected {}, got {}",
        msg_type,
        field,
        expected,
        num
    );
}

fn assert_json_field_u64(json: &serde_json::Value, field: &str, expected: u64, msg_type: &str) {
    let val = json
        .get(field)
        .unwrap_or_else(|| panic!("{}: missing field '{}'", msg_type, field));
    let num = val
        .as_u64()
        .unwrap_or_else(|| panic!("{}: field '{}' is not a u64: {:?}", msg_type, field, val));
    assert_eq!(num, expected, "{}: field '{}' mismatch", msg_type, field);
}

fn assert_json_has_field(json: &serde_json::Value, field: &str, msg_type: &str) {
    assert!(
        json.get(field).is_some(),
        "{}: missing required field '{}'",
        msg_type,
        field
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Parity tests — verify field names match cross-language contract
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn parity_cmd_vel_field_names() {
    let msg = CmdVel::new(1.5, -0.3);
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "linear", "CmdVel");
    assert_json_has_field(&json, "angular", "CmdVel");
    assert_json_has_field(&json, "timestamp_ns", "CmdVel");
    assert_json_field_f64(&json, "linear", 1.5, "CmdVel");
    assert_json_field_f64(&json, "angular", -0.3, "CmdVel");
}

#[test]
fn parity_twist_field_names() {
    let msg = Twist {
        linear: [1.0, 2.0, 3.0],
        angular: [0.1, 0.2, 0.3],
        timestamp_ns: 12345,
    };
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "linear", "Twist");
    assert_json_has_field(&json, "angular", "Twist");
    assert_json_has_field(&json, "timestamp_ns", "Twist");
    assert_json_field_u64(&json, "timestamp_ns", 12345, "Twist");
    // linear and angular are arrays
    let linear = json.get("linear").unwrap().as_array().unwrap();
    assert_eq!(linear.len(), 3, "Twist.linear must have 3 elements");
    assert!((linear[0].as_f64().unwrap() - 1.0).abs() < 1e-6);
}

#[test]
fn parity_pose2d_field_names() {
    let msg = Pose2D {
        x: 10.0,
        y: 20.0,
        theta: 1.57,
        timestamp_ns: 99,
    };
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_field_f64(&json, "x", 10.0, "Pose2D");
    assert_json_field_f64(&json, "y", 20.0, "Pose2D");
    assert_json_field_f64(&json, "theta", 1.57, "Pose2D");
    assert_json_field_u64(&json, "timestamp_ns", 99, "Pose2D");
}

#[test]
fn parity_imu_field_names() {
    let msg = Imu::new();
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    // IMU must have these exact field names for Python parity
    assert_json_has_field(&json, "orientation", "Imu");
    assert_json_has_field(&json, "orientation_covariance", "Imu");
    assert_json_has_field(&json, "angular_velocity", "Imu");
    assert_json_has_field(&json, "angular_velocity_covariance", "Imu");
    assert_json_has_field(&json, "linear_acceleration", "Imu");
    assert_json_has_field(&json, "linear_acceleration_covariance", "Imu");
    assert_json_has_field(&json, "timestamp_ns", "Imu");
    // Verify array sizes
    assert_eq!(
        json["orientation"].as_array().unwrap().len(),
        4,
        "Imu.orientation must be [f64; 4]"
    );
    assert_eq!(
        json["angular_velocity"].as_array().unwrap().len(),
        3,
        "Imu.angular_velocity must be [f64; 3]"
    );
}

#[test]
fn parity_laser_scan_field_names() {
    let msg = LaserScan::new();
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "ranges", "LaserScan");
    assert_json_has_field(&json, "angle_min", "LaserScan");
    assert_json_has_field(&json, "angle_max", "LaserScan");
    assert_json_has_field(&json, "range_min", "LaserScan");
    assert_json_has_field(&json, "range_max", "LaserScan");
    assert_json_has_field(&json, "angle_increment", "LaserScan");
    assert_json_has_field(&json, "timestamp_ns", "LaserScan");
    assert_eq!(
        json["ranges"].as_array().unwrap().len(),
        360,
        "LaserScan.ranges must have 360 elements"
    );
}

#[test]
fn parity_odometry_field_names() {
    let msg = Odometry::new();
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "pose", "Odometry");
    assert_json_has_field(&json, "twist", "Odometry");
    assert_json_has_field(&json, "pose_covariance", "Odometry");
    assert_json_has_field(&json, "twist_covariance", "Odometry");
    assert_json_has_field(&json, "frame_id", "Odometry");
    assert_json_has_field(&json, "child_frame_id", "Odometry");
    assert_json_has_field(&json, "timestamp_ns", "Odometry");
    // Nested pose must have x, y, theta
    let pose = json.get("pose").unwrap();
    assert_json_has_field(pose, "x", "Odometry.pose");
    assert_json_has_field(pose, "y", "Odometry.pose");
    assert_json_has_field(pose, "theta", "Odometry.pose");
}

#[test]
fn parity_joint_state_field_names() {
    let msg = JointState::new();
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "names", "JointState");
    assert_json_has_field(&json, "joint_count", "JointState");
    assert_json_has_field(&json, "positions", "JointState");
    assert_json_has_field(&json, "velocities", "JointState");
    assert_json_has_field(&json, "efforts", "JointState");
    assert_json_has_field(&json, "timestamp_ns", "JointState");
}

#[test]
fn parity_nav_goal_field_names() {
    let msg = NavGoal::new(Pose2D::default(), 0.1, 0.05);
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "target_pose", "NavGoal");
    assert_json_has_field(&json, "tolerance_position", "NavGoal");
    assert_json_has_field(&json, "tolerance_angle", "NavGoal");
    assert_json_has_field(&json, "timeout_seconds", "NavGoal");
    assert_json_has_field(&json, "priority", "NavGoal");
    assert_json_has_field(&json, "goal_id", "NavGoal");
    assert_json_field_f64(&json, "tolerance_position", 0.1, "NavGoal");
    assert_json_field_f64(&json, "tolerance_angle", 0.05, "NavGoal");
}

#[test]
fn parity_detection_field_names() {
    let msg = Detection::new("person", 0.95, 100.0, 200.0, 50.0, 80.0);
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "bbox", "Detection");
    assert_json_has_field(&json, "confidence", "Detection");
    assert_json_has_field(&json, "class_id", "Detection");
    assert_json_has_field(&json, "class_name", "Detection");
    assert_json_has_field(&json, "instance_id", "Detection");
    assert_json_field_f64(&json, "confidence", 0.95, "Detection");
    // bbox must have x, y, width, height
    let bbox = json.get("bbox").unwrap();
    assert_json_has_field(bbox, "x", "Detection.bbox");
    assert_json_has_field(bbox, "y", "Detection.bbox");
    assert_json_has_field(bbox, "width", "Detection.bbox");
    assert_json_has_field(bbox, "height", "Detection.bbox");
}

#[test]
fn parity_heartbeat_field_names() {
    let msg = Heartbeat::new("test_node", 42);
    let json: serde_json::Value = serde_json::to_value(msg).unwrap();
    assert_json_has_field(&json, "node_name", "Heartbeat");
    assert_json_has_field(&json, "node_id", "Heartbeat");
    assert_json_has_field(&json, "sequence", "Heartbeat");
    assert_json_has_field(&json, "alive", "Heartbeat");
    assert_json_has_field(&json, "uptime", "Heartbeat");
    assert_json_has_field(&json, "timestamp_ns", "Heartbeat");
    assert_json_field_u64(&json, "node_id", 42, "Heartbeat");
}

// ═══════════════════════════════════════════════════════════════════════════
// Value roundtrip tests — verify exact values survive JSON roundtrip
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn parity_cmd_vel_values_roundtrip() {
    let original = CmdVel::new(2.5, -1.2);
    let json = serde_json::to_string(&original).unwrap();
    let back: CmdVel = serde_json::from_str(&json).unwrap();
    assert!((back.linear - 2.5).abs() < 1e-6);
    assert!((back.angular - (-1.2)).abs() < 1e-6);
}

#[test]
fn parity_nav_goal_values_roundtrip() {
    let pose = Pose2D {
        x: 5.0,
        y: 10.0,
        theta: 1.0,
        timestamp_ns: 100,
    };
    let original = NavGoal::new(pose, 0.3, 0.1);
    let json = serde_json::to_string(&original).unwrap();
    let back: NavGoal = serde_json::from_str(&json).unwrap();
    assert!((back.target_pose.x - 5.0).abs() < 1e-6);
    assert!((back.target_pose.y - 10.0).abs() < 1e-6);
    assert!((back.tolerance_position - 0.3).abs() < 1e-6);
}

#[test]
fn parity_detection_values_roundtrip() {
    let original = Detection::new("car", 0.87, 50.0, 100.0, 200.0, 150.0);
    let json = serde_json::to_string(&original).unwrap();
    let back: Detection = serde_json::from_str(&json).unwrap();
    assert!((back.confidence - 0.87).abs() < 1e-4);
    assert!((back.bbox.x - 50.0).abs() < 1e-4);
    assert!((back.bbox.width - 200.0).abs() < 1e-4);
}
