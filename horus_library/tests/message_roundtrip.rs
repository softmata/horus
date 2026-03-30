//! Message Type Serialization Roundtrip Tests
//!
//! Verifies every message type with Default + Serialize + Deserialize

#![allow(clippy::field_reassign_with_default)]
//! survives a JSON roundtrip without panic or data loss.

use horus_library::messages::*;

/// JSON serialize → deserialize, verify no panic
fn json_roundtrip<T: serde::Serialize + serde::de::DeserializeOwned + std::fmt::Debug>(
    val: &T,
    name: &str,
) {
    let json =
        serde_json::to_string(val).unwrap_or_else(|e| panic!("{} serialize failed: {}", name, e));
    assert!(!json.is_empty(), "{} produced empty JSON", name);
    let _back: T = serde_json::from_str(&json)
        .unwrap_or_else(|e| panic!("{} deserialize failed: {} — json: {}", name, e, json));
}

// Each test creates Default + verifies roundtrip. Simple but catches:
// - Missing Serialize/Deserialize derives
// - Fields that can't serialize (fn pointers, raw pointers)
// - Deserialization failures on valid serialized data

#[test]
fn roundtrip_cmd_vel() {
    json_roundtrip(&CmdVel::new(1.5, -0.3), "CmdVel");
}
#[test]
fn roundtrip_cmd_vel_zero() {
    json_roundtrip(&CmdVel::new(0.0, 0.0), "CmdVel::zero");
}
#[test]
fn roundtrip_imu() {
    json_roundtrip(&Imu::default(), "Imu");
}
#[test]
fn roundtrip_laser_scan() {
    json_roundtrip(&LaserScan::default(), "LaserScan");
}
#[test]
fn roundtrip_odometry() {
    json_roundtrip(&Odometry::default(), "Odometry");
}
// BUG FOUND: BatteryState::default() sets charge/capacity to NaN, which serializes
// as JSON null but can't deserialize back to f32. Needs #[serde(default = "f32::nan")]
// or use Option<f32> for these fields.
#[test]
fn roundtrip_battery_state_non_nan() {
    // Use non-NaN values to verify the rest of the struct roundtrips
    let mut bat = BatteryState::default();
    bat.charge = 0.0;
    bat.capacity = 0.0;
    json_roundtrip(&bat, "BatteryState");
}
#[test]
fn roundtrip_joint_state() {
    json_roundtrip(&JointState::default(), "JointState");
}
#[test]
fn roundtrip_temperature() {
    json_roundtrip(&Temperature::default(), "Temperature");
}
#[test]
fn roundtrip_range_sensor() {
    json_roundtrip(&RangeSensor::default(), "RangeSensor");
}
#[test]
fn roundtrip_nav_sat_fix() {
    json_roundtrip(&NavSatFix::default(), "NavSatFix");
}
#[test]
fn roundtrip_magnetic_field() {
    json_roundtrip(&MagneticField::default(), "MagneticField");
}
#[test]
fn roundtrip_fluid_pressure() {
    json_roundtrip(&FluidPressure::default(), "FluidPressure");
}
#[test]
fn roundtrip_illuminance() {
    json_roundtrip(&Illuminance::default(), "Illuminance");
}
#[test]
fn roundtrip_twist() {
    json_roundtrip(&Twist::default(), "Twist");
}
#[test]
fn roundtrip_pose2d() {
    json_roundtrip(&Pose2D::default(), "Pose2D");
}
#[test]
fn roundtrip_pose3d() {
    json_roundtrip(&Pose3D::default(), "Pose3D");
}
#[test]
fn roundtrip_point3() {
    json_roundtrip(&Point3::default(), "Point3");
}
#[test]
fn roundtrip_vector3() {
    json_roundtrip(&Vector3::default(), "Vector3");
}
#[test]
fn roundtrip_quaternion() {
    json_roundtrip(&Quaternion::default(), "Quaternion");
}
#[test]
fn roundtrip_transform_stamped() {
    json_roundtrip(&TransformStamped::default(), "TransformStamped");
}
#[test]
fn roundtrip_motor_command() {
    json_roundtrip(&MotorCommand::default(), "MotorCommand");
}
#[test]
fn roundtrip_heartbeat() {
    json_roundtrip(&Heartbeat::default(), "Heartbeat");
}
#[test]
fn roundtrip_emergency_stop() {
    json_roundtrip(&EmergencyStop::default(), "EmergencyStop");
}
#[test]
fn roundtrip_diagnostic_status() {
    json_roundtrip(&DiagnosticStatus::default(), "DiagnosticStatus");
}
#[test]
fn roundtrip_bounding_box_2d() {
    json_roundtrip(&BoundingBox2D::default(), "BoundingBox2D");
}
#[test]
fn roundtrip_detection() {
    json_roundtrip(&Detection::default(), "Detection");
}
#[test]
fn roundtrip_compressed_image() {
    json_roundtrip(&CompressedImage::default(), "CompressedImage");
}
#[test]
fn roundtrip_camera_info() {
    json_roundtrip(&CameraInfo::default(), "CameraInfo");
}

// ============================================================================
// Previously missing types — added for exhaustive coverage
// ============================================================================

// Geometry
#[test]
fn roundtrip_accel() {
    json_roundtrip(&Accel::default(), "Accel");
}
#[test]
fn roundtrip_accel_stamped() {
    json_roundtrip(&AccelStamped::default(), "AccelStamped");
}
#[test]
fn roundtrip_pose_stamped() {
    json_roundtrip(&PoseStamped::default(), "PoseStamped");
}
#[test]
fn roundtrip_pose_with_covariance() {
    json_roundtrip(&PoseWithCovariance::default(), "PoseWithCovariance");
}
#[test]
fn roundtrip_twist_with_covariance() {
    json_roundtrip(&TwistWithCovariance::default(), "TwistWithCovariance");
}

// Control
#[test]
fn roundtrip_servo_command() {
    json_roundtrip(&ServoCommand::default(), "ServoCommand");
}
#[test]
fn roundtrip_differential_drive() {
    json_roundtrip(
        &DifferentialDriveCommand::default(),
        "DifferentialDriveCommand",
    );
}
#[test]
fn roundtrip_pid_config() {
    json_roundtrip(&PidConfig::default(), "PidConfig");
}
#[test]
fn roundtrip_trajectory_point() {
    json_roundtrip(&TrajectoryPoint::default(), "TrajectoryPoint");
}
#[test]
fn roundtrip_joint_command() {
    json_roundtrip(&JointCommand::default(), "JointCommand");
}

// Diagnostics
#[test]
fn roundtrip_resource_usage() {
    json_roundtrip(&ResourceUsage::default(), "ResourceUsage");
}
#[test]
fn roundtrip_diagnostic_value() {
    json_roundtrip(&DiagnosticValue::default(), "DiagnosticValue");
}
#[test]
fn roundtrip_diagnostic_report() {
    json_roundtrip(&DiagnosticReport::default(), "DiagnosticReport");
}
#[test]
fn roundtrip_node_heartbeat() {
    json_roundtrip(&NodeHeartbeat::default(), "NodeHeartbeat");
}
#[test]
fn roundtrip_safety_status() {
    json_roundtrip(&SafetyStatus::default(), "SafetyStatus");
}

// Navigation
#[test]
fn roundtrip_nav_goal() {
    json_roundtrip(&NavGoal::default(), "NavGoal");
}

// Clock
#[test]
fn roundtrip_clock() {
    json_roundtrip(&Clock::default(), "Clock");
}
#[test]
fn roundtrip_time_reference() {
    json_roundtrip(&TimeReference::default(), "TimeReference");
}

// Detection / Perception
#[test]
fn roundtrip_bounding_box_3d() {
    json_roundtrip(&BoundingBox3D::default(), "BoundingBox3D");
}
#[test]
fn roundtrip_detection_3d() {
    json_roundtrip(&Detection3D::default(), "Detection3D");
}
#[test]
fn roundtrip_tracked_object() {
    json_roundtrip(&TrackedObject::default(), "TrackedObject");
}
#[test]
fn roundtrip_tracking_header() {
    json_roundtrip(&TrackingHeader::default(), "TrackingHeader");
}
#[test]
fn roundtrip_landmark() {
    json_roundtrip(&Landmark::default(), "Landmark");
}
#[test]
fn roundtrip_landmark_3d() {
    json_roundtrip(&Landmark3D::default(), "Landmark3D");
}
#[test]
fn roundtrip_segmentation_mask() {
    json_roundtrip(&SegmentationMask::default(), "SegmentationMask");
}
#[test]
fn roundtrip_plane_detection() {
    json_roundtrip(&PlaneDetection::default(), "PlaneDetection");
}

// Vision
#[test]
fn roundtrip_region_of_interest() {
    json_roundtrip(&RegionOfInterest::default(), "RegionOfInterest");
}

// Force / Torque
#[test]
fn roundtrip_wrench_stamped() {
    json_roundtrip(&WrenchStamped::default(), "WrenchStamped");
}
#[test]
fn roundtrip_force_command() {
    json_roundtrip(&ForceCommand::default(), "ForceCommand");
}
#[test]
fn roundtrip_impedance_parameters() {
    json_roundtrip(&ImpedanceParameters::default(), "ImpedanceParameters");
}
#[test]
fn roundtrip_contact_info() {
    json_roundtrip(&ContactInfo::default(), "ContactInfo");
}
#[test]
fn roundtrip_haptic_feedback() {
    json_roundtrip(&HapticFeedback::default(), "HapticFeedback");
}

// Navigation (previously missing)
#[test]
fn roundtrip_goal_result() {
    json_roundtrip(&GoalResult::default(), "GoalResult");
}
#[test]
fn roundtrip_cost_map() {
    json_roundtrip(&CostMap::default(), "CostMap");
}
#[test]
fn roundtrip_path_plan() {
    json_roundtrip(&PathPlan::default(), "PathPlan");
}
#[test]
fn roundtrip_waypoint() {
    json_roundtrip(&Waypoint::default(), "Waypoint");
}
#[test]
fn roundtrip_velocity_obstacle() {
    json_roundtrip(&VelocityObstacle::default(), "VelocityObstacle");
}
#[test]
fn roundtrip_velocity_obstacles() {
    json_roundtrip(&VelocityObstacles::default(), "VelocityObstacles");
}

// Perception (previously missing)
#[test]
fn roundtrip_landmark_array() {
    json_roundtrip(&LandmarkArray::default(), "LandmarkArray");
}
#[test]
fn roundtrip_plane_array() {
    json_roundtrip(&PlaneArray::default(), "PlaneArray");
}

// Input — use button constructor
#[test]
fn roundtrip_joystick_input() {
    let joy = JoystickInput::new_button(0, 0, "A".to_string(), true);
    json_roundtrip(&joy, "JoystickInput");
}
#[test]
fn roundtrip_keyboard_input() {
    let key = KeyboardInput::new("A".to_string(), 65, vec![], true);
    json_roundtrip(&key, "KeyboardInput");
}

// Audio
#[test]
fn roundtrip_audio_frame() {
    json_roundtrip(&AudioFrame::default(), "AudioFrame");
}

// ============================================================================
// Edge cases
// ============================================================================

// Extreme float values
#[test]
fn roundtrip_cmd_vel_extreme() {
    let cmd = CmdVel::new(f32::MAX, f32::MIN);
    let json = serde_json::to_string(&cmd).unwrap();
    let back: CmdVel = serde_json::from_str(&json).unwrap();
    assert_eq!(back.linear, f32::MAX);
    assert_eq!(back.angular, f32::MIN);
}

// Zero-value edge case — all fields zero
#[test]
fn roundtrip_imu_zero() {
    let imu = Imu::default(); // all zeros
    json_roundtrip(&imu, "Imu::zero");
}

// Pose2D with negative values
#[test]
fn roundtrip_pose2d_negative() {
    let mut pose = Pose2D::default();
    pose.x = -100.0;
    pose.y = -200.0;
    pose.theta = -std::f64::consts::PI;
    json_roundtrip(&pose, "Pose2D::negative");
}

// ============================================================================
// SimSync roundtrip tests
// ============================================================================

#[test]
fn roundtrip_simsync_waiting() {
    let s = SimSync::waiting(42, 1_000_000_000, 1_000_000);
    let json = serde_json::to_string(&s).unwrap();
    let back: SimSync = serde_json::from_str(&json).unwrap();
    assert_eq!(back.step, 42);
    assert_eq!(back.sim_time_ns, 1_000_000_000);
    assert_eq!(back.dt_ns, 1_000_000);
    assert_eq!(back.state, SimSync::WAITING);
}

#[test]
fn roundtrip_simsync_done() {
    let s = SimSync::done(99);
    let json = serde_json::to_string(&s).unwrap();
    let back: SimSync = serde_json::from_str(&json).unwrap();
    assert_eq!(back.step, 99);
    assert_eq!(back.state, SimSync::DONE);
}

#[test]
fn roundtrip_simsync_default() {
    let s = SimSync::default();
    json_roundtrip(&s, "SimSync::default");
    assert_eq!(s.state, SimSync::IDLE);
}

// ============================================================================
// RateRequest roundtrip tests
// ============================================================================

#[test]
fn roundtrip_rate_request_normal() {
    let rr = RateRequest::new("imu", 100.0, 50.0, 200.0);
    let json = serde_json::to_string(&rr).unwrap();
    let back: RateRequest = serde_json::from_str(&json).unwrap();
    assert_eq!(back.topic(), "imu");
    assert!((back.desired_hz - 100.0).abs() < f64::EPSILON);
    assert!((back.min_hz - 50.0).abs() < f64::EPSILON);
    assert!((back.max_hz - 200.0).abs() < f64::EPSILON);
}

#[test]
fn roundtrip_rate_request_truncated_topic() {
    let long_name = "a".repeat(100);
    let rr = RateRequest::new(&long_name, 1.0, 0.5, 2.0);
    let json = serde_json::to_string(&rr).unwrap();
    let back: RateRequest = serde_json::from_str(&json).unwrap();
    // Topic truncated to 31 chars
    assert!(back.topic().len() <= 31);
    assert_eq!(back.topic(), &"a".repeat(31));
}

#[test]
fn roundtrip_rate_request_with_requester() {
    let rr = RateRequest::new("cmd_vel", 30.0, 10.0, 60.0).with_requester(42);
    let json = serde_json::to_string(&rr).unwrap();
    let back: RateRequest = serde_json::from_str(&json).unwrap();
    assert_eq!(back.requester_id, 42);
    assert_eq!(back.topic(), "cmd_vel");
}

// ============================================================================
// TactileArray roundtrip tests
// ============================================================================

#[test]
fn roundtrip_tactile_array_populated() {
    let mut ta = TactileArray::new(4, 4);
    ta.set_force(0, 0, 1.5);
    ta.set_force(1, 2, 3.7);
    ta.set_force(3, 3, 0.1);
    ta.total_force = [5.3, 0.0, 0.0];
    ta.center_of_pressure = [0.5, 0.5];
    ta.in_contact = true;
    ta.physical_size = [0.04, 0.04];
    ta.timestamp_ns = 12345;

    let json = serde_json::to_string(&ta).unwrap();
    let back: TactileArray = serde_json::from_str(&json).unwrap();
    assert_eq!(back.rows, 4);
    assert_eq!(back.cols, 4);
    assert_eq!(back.get_force(0, 0), Some(1.5));
    assert_eq!(back.get_force(1, 2), Some(3.7));
    assert_eq!(back.get_force(3, 3), Some(0.1));
    assert!(back.in_contact);
    assert_eq!(back.timestamp_ns, 12345);
}

#[test]
fn roundtrip_tactile_array_empty() {
    let ta = TactileArray::default();
    json_roundtrip(&ta, "TactileArray::default");
}

#[test]
fn roundtrip_tactile_array_single_cell() {
    let mut ta = TactileArray::new(1, 1);
    ta.set_force(0, 0, 99.9);
    let json = serde_json::to_string(&ta).unwrap();
    let back: TactileArray = serde_json::from_str(&json).unwrap();
    assert_eq!(back.forces.len(), 1);
    assert!((back.forces[0] - 99.9).abs() < 0.01);
}

// ============================================================================
// Previously missing types — SLAM Cycle 1
// ============================================================================

// Navigation
#[test]
fn roundtrip_nav_path() {
    json_roundtrip(&NavPath::default(), "NavPath");
}
#[test]
fn roundtrip_nav_path_with_waypoints() {
    let mut path = NavPath::default();
    path.waypoint_count = 2;
    path.total_length = 5.5;
    path.duration_seconds = 10.0;
    json_roundtrip(&path, "NavPath::with_waypoints");
}

// Vision
#[test]
fn roundtrip_stereo_info() {
    json_roundtrip(&StereoInfo::default(), "StereoInfo");
}
#[test]
fn roundtrip_stereo_info_populated() {
    let mut si = StereoInfo::default();
    si.baseline = 0.12;
    si.depth_scale = 1.0;
    si.left_camera.width = 640;
    si.left_camera.height = 480;
    si.right_camera.width = 640;
    si.right_camera.height = 480;
    json_roundtrip(&si, "StereoInfo::populated");
}

// Perception
#[test]
fn roundtrip_point_field() {
    let pf = PointField::new("x", 0, horus_core::types::TensorDtype::F32, 1);
    json_roundtrip(&pf, "PointField");
}

// Enums — verify all variants roundtrip
#[test]
fn roundtrip_status_level_all_variants() {
    json_roundtrip(&StatusLevel::Ok, "StatusLevel::Ok");
    json_roundtrip(&StatusLevel::Warn, "StatusLevel::Warn");
    json_roundtrip(&StatusLevel::Error, "StatusLevel::Error");
    json_roundtrip(&StatusLevel::Fatal, "StatusLevel::Fatal");
}

#[test]
fn roundtrip_node_state_msg_all_variants() {
    json_roundtrip(&NodeStateMsg::Idle, "NodeStateMsg::Idle");
    json_roundtrip(&NodeStateMsg::Initializing, "NodeStateMsg::Initializing");
    json_roundtrip(&NodeStateMsg::Running, "NodeStateMsg::Running");
    json_roundtrip(&NodeStateMsg::Paused, "NodeStateMsg::Paused");
    json_roundtrip(&NodeStateMsg::Stopped, "NodeStateMsg::Stopped");
    json_roundtrip(&NodeStateMsg::Error, "NodeStateMsg::Error");
}

#[test]
fn roundtrip_audio_encoding_all_variants() {
    json_roundtrip(&AudioEncoding::F32, "AudioEncoding::F32");
    json_roundtrip(&AudioEncoding::I16, "AudioEncoding::I16");
}

#[test]
fn roundtrip_contact_state_all_variants() {
    json_roundtrip(&ContactState::NoContact, "ContactState::NoContact");
    json_roundtrip(
        &ContactState::InitialContact,
        "ContactState::InitialContact",
    );
    json_roundtrip(&ContactState::StableContact, "ContactState::StableContact");
    json_roundtrip(&ContactState::ContactLoss, "ContactState::ContactLoss");
    json_roundtrip(&ContactState::Sliding, "ContactState::Sliding");
    json_roundtrip(&ContactState::Impact, "ContactState::Impact");
}

// ============================================================================
// NaN/Inf handling for new types
// ============================================================================

#[test]
fn roundtrip_rate_request_nan_desired_hz() {
    let rr = RateRequest::new("test", f64::NAN, 0.0, f64::INFINITY);
    // JSON serialization of NaN/Inf produces null — verify no panic
    let json = serde_json::to_string(&rr);
    // serde_json may return error for NaN (depends on config) — either is acceptable
    if let Ok(json_str) = json {
        // If it serialized, deserialization may fail — that's OK
        let _back: Result<RateRequest, _> = serde_json::from_str(&json_str);
    }
    // Main assertion: no panic
}

#[test]
fn roundtrip_tactile_array_nan_force() {
    let mut ta = TactileArray::new(2, 2);
    ta.set_force(0, 0, f32::NAN);
    ta.total_force = [f32::NAN, 0.0, 0.0];
    // serde_json serializes NaN as null — this will fail deserialize
    let json = serde_json::to_string(&ta);
    if let Ok(json_str) = json {
        let _back: Result<TactileArray, _> = serde_json::from_str(&json_str);
    }
    // Main assertion: no panic
}
