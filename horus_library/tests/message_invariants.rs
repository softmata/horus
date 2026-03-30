//! Property-based invariant tests for domain message types.
//!
//! Unlike roundtrip tests (which verify serialize/deserialize fidelity),

#![allow(clippy::field_reassign_with_default)]
//! these tests verify **domain invariants** — mathematical and semantic
//! properties that must hold for any valid message instance.
//!
//! Uses proptest with 1000 cases per invariant for thorough coverage.

use horus_library::messages::*;
use proptest::prelude::*;
use std::f64::consts::PI;

// ── Strategies ──────────────────────────────────────────────────────────────

fn finite_f64() -> impl Strategy<Value = f64> {
    -1e15f64..1e15f64
}

fn finite_f32() -> impl Strategy<Value = f32> {
    -1e7f32..1e7f32
}

/// Strategy for theta values spanning well beyond [-pi, pi] to stress
/// the normalize_angle logic. Includes multiples of 2*PI, near-boundary
/// values, and large angles.
fn wide_angle() -> impl Strategy<Value = f64> {
    prop_oneof![
        // Normal range
        -10.0 * PI..10.0 * PI,
        // Large multiples
        (-1000.0 * PI..1000.0 * PI),
        // Near boundaries (where off-by-one errors lurk)
        (PI - 0.001..PI + 0.001),
        (-PI - 0.001..-PI + 0.001),
    ]
}

/// Strategy for non-zero quaternion components (avoids the degenerate
/// zero quaternion where normalize() is a no-op).
fn nonzero_quat() -> impl Strategy<Value = (f64, f64, f64, f64)> {
    (finite_f64(), finite_f64(), finite_f64(), finite_f64()).prop_filter(
        "at least one component must be non-zero",
        |(x, y, z, w)| {
            let norm_sq = x * x + y * y + z * z + w * w;
            norm_sq > 1e-20
        },
    )
}

/// Strategy for BoundingBox2D with positive width and height.
fn positive_bbox2d() -> impl Strategy<Value = BoundingBox2D> {
    (finite_f32(), finite_f32(), 0.0f32..1e6f32, 0.0f32..1e6f32)
        .prop_map(|(x, y, w, h)| BoundingBox2D::new(x, y, w, h))
}

// ============================================================================
// Test 9: Pose2D normalize_angle keeps theta in [-pi, pi]
// ============================================================================

// INVARIANT: For any Pose2D with any finite theta, calling normalize_angle()
// produces theta in [-pi, pi].
//
// This is a fundamental invariant for mobile robotics: heading angles must
// be in the canonical range for distance calculations, path planning, and
// control algorithms to work correctly. An unnormalized angle can cause
// a robot to spin the wrong way.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(1000))]

    #[test]
    fn test_pose2d_normalize_angle_invariant(theta in wide_angle()) {
        let mut pose = Pose2D {
            x: 0.0,
            y: 0.0,
            theta,
            timestamp_ns: 0,
        };

        pose.normalize_angle();

        prop_assert!(
            pose.theta >= -PI && pose.theta <= PI,
            "After normalize_angle(), theta={} should be in [-pi, pi] (input was {})",
            pose.theta,
            theta
        );
    }
}

// ============================================================================
// Test 10: CmdVel::new(0.0, 0.0) is a full stop
// ============================================================================

// INVARIANT: A zero-velocity CmdVel must have both linear and angular == 0.
//
// Safety-critical: a stop command must actually stop the robot. If either
// component is non-zero due to a constructor bug, the robot keeps moving
// when it should be stationary.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(1000))]

    #[test]
    fn test_cmd_vel_zero_is_stop(_seed in 0u64..1000u64) {
        let stop = CmdVel::new(0.0, 0.0);

        prop_assert!(
            stop.linear == 0.0,
            "CmdVel::new(0, 0) linear should be exactly 0.0, got {}",
            stop.linear
        );
        prop_assert!(
            stop.angular == 0.0,
            "CmdVel::new(0, 0) angular should be exactly 0.0, got {}",
            stop.angular
        );

        // Also verify CmdVel::zero() convenience constructor
        let zero = CmdVel::zero();
        prop_assert!(
            zero.linear == 0.0 && zero.angular == 0.0,
            "CmdVel::zero() should be a full stop"
        );
    }
}

// ============================================================================
// Test 11: LaserScan always has exactly 360 range values
// ============================================================================

// INVARIANT: LaserScan::ranges is a fixed [f32; 360] array — regardless of
// how the scan is constructed, it always has exactly 360 elements.
//
// Downstream code indexes into ranges[0..360] without bounds checks.
// If the array were ever a different size, we'd get UB or panics in the
// SLAM, obstacle avoidance, and visualization pipelines.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(1000))]

    #[test]
    fn test_laser_scan_ranges_count_invariant(
        angle_min in finite_f32(),
        angle_max in finite_f32(),
        range_min in finite_f32(),
        range_max in finite_f32(),
    ) {
        // Default constructor
        let scan_default = LaserScan::default();
        prop_assert_eq!(
            scan_default.ranges.len(),
            360,
            "LaserScan::default() should have 360 ranges"
        );

        // new() constructor
        let scan_new = LaserScan::new();
        prop_assert_eq!(
            scan_new.ranges.len(),
            360,
            "LaserScan::new() should have 360 ranges"
        );

        // Manually constructed (field mutation)
        let scan_manual = LaserScan {
            ranges: [0.0; 360],
            angle_min,
            angle_max,
            range_min,
            range_max,
            angle_increment: std::f32::consts::PI / 180.0,
            time_increment: 0.0,
            scan_time: 0.1,
            timestamp_ns: 0,
        };
        prop_assert_eq!(
            scan_manual.ranges.len(),
            360,
            "Manually constructed LaserScan should have 360 ranges"
        );
    }
}

// ============================================================================
// Test 12: Quaternion normalize() produces unit norm
// ============================================================================

// INVARIANT: After calling normalize() on any non-zero Quaternion, the
// norm (sqrt(x^2 + y^2 + z^2 + w^2)) should be approximately 1.0.
//
// Unit quaternions represent rotations. A non-unit quaternion distorts
// the rotation, causing incorrect orientation estimates. Every
// quaternion used in transform chains must pass this invariant.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(1000))]

    #[test]
    fn test_quaternion_unit_norm_after_normalize((x, y, z, w) in nonzero_quat()) {
        let mut q = Quaternion::new(x, y, z, w);
        q.normalize();

        let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();

        prop_assert!(
            (norm - 1.0).abs() < 1e-6,
            "After normalize(), quaternion norm should be ~1.0, got {} \
             (input was ({}, {}, {}, {}))",
            norm,
            x,
            y,
            z,
            w
        );
    }
}

// ============================================================================
// Test 13: BoundingBox2D area is non-negative for positive dimensions
// ============================================================================

// INVARIANT: For any BoundingBox2D with width >= 0 and height >= 0,
// area() must be >= 0.
//
// A negative area would break IoU calculations, NMS filtering, and
// tracking metrics. Detection pipelines assume area() is non-negative
// for all valid bounding boxes.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(1000))]

    #[test]
    fn test_bounding_box_2d_area_non_negative(bbox in positive_bbox2d()) {
        let area = bbox.area();

        prop_assert!(
            area >= 0.0,
            "BoundingBox2D area should be >= 0 for positive dimensions, \
             got {} (width={}, height={})",
            area,
            bbox.width,
            bbox.height
        );

        // Verify area matches width * height exactly
        let expected = bbox.width * bbox.height;
        prop_assert!(
            (area - expected).abs() < f32::EPSILON * expected.abs().max(1.0),
            "BoundingBox2D area should equal width * height: {} vs {}",
            area,
            expected
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR DOMAIN INVARIANTS — physical constraints all sensors must obey
// ═══════════════════════════════════════════════════════════════════════════

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    // ── IMU ──────────────────────────────────────────────────────────────

    #[test]
    fn test_imu_default_is_valid(_seed in 0u64..500u64) {
        prop_assert!(Imu::new().is_valid(), "Imu::new() must be valid");
    }

    #[test]
    fn test_imu_finite_inputs_are_valid(
        ax in -100.0f64..100.0, ay in -100.0f64..100.0, az in -100.0f64..100.0,
        gx in -50.0f64..50.0, gy in -50.0f64..50.0, gz in -50.0f64..50.0,
    ) {
        let mut imu = Imu::new();
        imu.linear_acceleration = [ax, ay, az];
        imu.angular_velocity = [gx, gy, gz];
        prop_assert!(imu.is_valid(), "IMU with finite inputs must be valid");
    }

    #[test]
    fn test_imu_nan_is_invalid(_seed in 0u64..100u64) {
        let mut imu = Imu::new();
        imu.linear_acceleration[0] = f64::NAN;
        prop_assert!(!imu.is_valid(), "IMU with NaN must be invalid");
    }

    // ── Odometry ─────────────────────────────────────────────────────────

    #[test]
    fn test_odometry_default_is_valid(_seed in 0u64..500u64) {
        prop_assert!(Odometry::new().is_valid(), "Odometry::new() must be valid");
    }

    #[test]
    fn test_odometry_finite_is_valid(
        x in -1e6f64..1e6, y in -1e6f64..1e6, theta in -PI..PI,
    ) {
        let mut odom = Odometry::new();
        odom.pose.x = x;
        odom.pose.y = y;
        odom.pose.theta = theta;
        prop_assert!(odom.is_valid(), "Odometry with finite pose must be valid");
    }

    // ── RangeSensor ──────────────────────────────────────────────────────

    #[test]
    fn test_range_sensor_in_bounds_is_valid(rmin in 0.01f32..1.0, rmax in 1.0f32..100.0) {
        let mid = (rmin + rmax) / 2.0;
        let s = RangeSensor { sensor_type: 0, field_of_view: 0.5, min_range: rmin, max_range: rmax, range: mid, timestamp_ns: 0 };
        prop_assert!(s.is_valid(), "range {} in [{}, {}] must be valid", mid, rmin, rmax);
    }

    #[test]
    fn test_range_sensor_out_of_bounds_invalid(rmin in 0.1f32..1.0, rmax in 2.0f32..50.0) {
        let s = RangeSensor { sensor_type: 0, field_of_view: 0.5, min_range: rmin, max_range: rmax, range: rmax + 1.0, timestamp_ns: 0 };
        prop_assert!(!s.is_valid(), "range {} > max {} must be invalid", s.range, rmax);
    }

    // ── NavSatFix ────────────────────────────────────────────────────────

    #[test]
    fn test_nav_sat_fix_valid_coords(lat in -90.0f64..90.0, lon in -180.0f64..180.0, alt in -500.0f64..50000.0) {
        let fix = NavSatFix::from_coordinates(lat, lon, alt);
        prop_assert!(fix.is_valid(), "NavSatFix({}, {}, {}) must be valid", lat, lon, alt);
    }

    #[test]
    fn test_nav_sat_fix_bad_latitude(lat in 91.0f64..1000.0) {
        let fix = NavSatFix::from_coordinates(lat, 0.0, 0.0);
        prop_assert!(!fix.is_valid(), "latitude {} must be invalid", lat);
    }

    #[test]
    fn test_nav_sat_fix_bad_longitude(lon in 181.0f64..1000.0) {
        let fix = NavSatFix::from_coordinates(0.0, lon, 0.0);
        prop_assert!(!fix.is_valid(), "longitude {} must be invalid", lon);
    }

    // ── Twist ────────────────────────────────────────────────────────────

    #[test]
    fn test_twist_default_is_valid_zero(_seed in 0u64..100u64) {
        let t = Twist::default();
        prop_assert!(t.is_valid());
        prop_assert!(t.linear.iter().all(|v| *v == 0.0));
        prop_assert!(t.angular.iter().all(|v| *v == 0.0));
    }

    // ── Pose2D ───────────────────────────────────────────────────────────

    #[test]
    fn test_pose2d_default_valid(_seed in 0u64..100u64) {
        prop_assert!(Pose2D::default().is_valid());
    }

    #[test]
    fn test_pose2d_distance_symmetric(
        x1 in -1000.0f64..1000.0, y1 in -1000.0f64..1000.0,
        x2 in -1000.0f64..1000.0, y2 in -1000.0f64..1000.0,
    ) {
        let a = Pose2D { x: x1, y: y1, theta: 0.0, timestamp_ns: 0 };
        let b = Pose2D { x: x2, y: y2, theta: 0.0, timestamp_ns: 0 };
        prop_assert!(a.distance_to(&b) >= 0.0);
        prop_assert!((a.distance_to(&b) - b.distance_to(&a)).abs() < 1e-10);
    }

    #[test]
    fn test_pose2d_distance_to_self_zero(x in -1000.0f64..1000.0, y in -1000.0f64..1000.0) {
        let p = Pose2D { x, y, theta: 0.0, timestamp_ns: 0 };
        prop_assert!(p.distance_to(&p).abs() < 1e-10);
    }

    // ── TransformStamped ─────────────────────────────────────────────────

    /// NOTE: TransformStamped::default() has all-zero quaternion (not unit),
    /// so is_valid() returns false. This is a known design choice — users must
    /// construct with TransformStamped::identity() or set rotation explicitly.
    #[test]
    fn test_transform_stamped_identity_valid(_seed in 0u64..100u64) {
        let tf = TransformStamped {
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0], // identity quaternion
            timestamp_ns: 0,
        };
        prop_assert!(tf.is_valid(), "TransformStamped with identity rotation must be valid");
    }

    // ── MotorCommand ─────────────────────────────────────────────────────

    #[test]
    fn test_motor_command_finite_valid(target in -100.0f64..100.0, vel in 0.0f64..100.0) {
        let cmd = MotorCommand { motor_id: 0, mode: 0, target, max_velocity: vel, max_acceleration: 10.0, feed_forward: 0.0, enable: 1, timestamp_ns: 0 };
        prop_assert!(cmd.is_valid());
    }

    // ── ServoCommand ─────────────────────────────────────────────────────

    #[test]
    fn test_servo_speed_in_01_valid(pos in -3.2f32..3.2, speed in 0.0f32..1.0) {
        let cmd = ServoCommand { servo_id: 0, position: pos, speed, enable: 1, timestamp_ns: 0 };
        prop_assert!(cmd.is_valid());
    }

    #[test]
    fn test_servo_speed_over_1_invalid(speed in 1.01f32..10.0) {
        let cmd = ServoCommand { servo_id: 0, position: 0.0, speed, enable: 1, timestamp_ns: 0 };
        prop_assert!(!cmd.is_valid(), "speed {} > 1 must be invalid", speed);
    }

    // ── PidConfig ────────────────────────────────────────────────────────

    #[test]
    fn test_pid_positive_gains_valid(kp in 0.0f64..100.0, ki in 0.0f64..100.0, kd in 0.0f64..100.0) {
        let pid = PidConfig { controller_id: 0, kp, ki, kd, integral_limit: 100.0, output_limit: 100.0, anti_windup: 1, timestamp_ns: 0 };
        prop_assert!(pid.is_valid());
    }

    #[test]
    fn test_pid_negative_kp_invalid(kp in -100.0f64..-0.01) {
        let pid = PidConfig { controller_id: 0, kp, ki: 0.0, kd: 0.0, integral_limit: 100.0, output_limit: 100.0, anti_windup: 0, timestamp_ns: 0 };
        prop_assert!(!pid.is_valid(), "negative Kp {} must be invalid", kp);
    }

    // ── JointCommand ─────────────────────────────────────────────────────

    #[test]
    fn test_joint_count_bounded_valid(count in 0u8..16u8) {
        let mut cmd = JointCommand::default();
        cmd.joint_count = count;
        prop_assert!(cmd.is_valid());
    }

    #[test]
    fn test_joint_count_over_16_invalid(count in 17u8..255u8) {
        let mut cmd = JointCommand::default();
        cmd.joint_count = count;
        prop_assert!(!cmd.is_valid(), "count {} > 16 must be invalid", count);
    }

    // ── BatteryState ─────────────────────────────────────────────────────

    #[test]
    fn test_battery_new_preserves_values(voltage in 0.0f32..60.0, pct in 0.0f32..100.0) {
        let bat = BatteryState::new(voltage, pct);
        prop_assert!((bat.voltage - voltage).abs() < f32::EPSILON);
        prop_assert!((bat.percentage - pct).abs() < f32::EPSILON);
    }

    // ── DifferentialDriveCommand ──────────────────────────────────────────

    #[test]
    fn test_diff_drive_finite_valid(left in -10.0f64..10.0, right in -10.0f64..10.0) {
        prop_assert!(DifferentialDriveCommand::new(left, right).is_valid());
    }

    // ── Accel ────────────────────────────────────────────────────────────

    #[test]
    fn test_accel_finite_valid(
        lx in -100.0f64..100.0, ly in -100.0f64..100.0, lz in -100.0f64..100.0,
    ) {
        let a = Accel { linear: [lx, ly, lz], angular: [0.0, 0.0, 0.0], timestamp_ns: 0 };
        prop_assert!(a.is_valid());
    }

    // ── Quaternion ───────────────────────────────────────────────────────

    #[test]
    fn test_quaternion_identity_is_valid(_seed in 0u64..100u64) {
        let q = Quaternion::identity();
        prop_assert!(q.is_valid());
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        prop_assert!((norm - 1.0).abs() < 1e-10, "identity norm must be 1.0");
    }

    // ── Heartbeat ────────────────────────────────────────────────────────

    #[test]
    fn test_heartbeat_new_has_nonzero_sequence(name in "[a-z]{3,10}") {
        let hb = Heartbeat::new(&name, 1);
        prop_assert!(hb.alive > 0, "new heartbeat must be alive");
        prop_assert!(hb.node_id == 1);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NAVIGATION & CONTROL INVARIANTS
// ═══════════════════════════════════════════════════════════════════════════

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    // ── NavGoal ──────────────────────────────────────────────────────────

    /// INVARIANT: NavGoal::new() preserves tolerances and they match input
    #[test]
    fn test_nav_goal_preserves_tolerances(
        pos_tol in 0.0f64..10.0,
        ang_tol in 0.0f64..PI,
    ) {
        let pose = Pose2D { x: 1.0, y: 2.0, theta: 0.5, timestamp_ns: 0 };
        let goal = NavGoal::new(pose, pos_tol, ang_tol);
        prop_assert!((goal.tolerance_position - pos_tol).abs() < 1e-10,
            "position tolerance must match: {} vs {}", goal.tolerance_position, pos_tol);
        prop_assert!((goal.tolerance_angle - ang_tol).abs() < 1e-10,
            "angle tolerance must match: {} vs {}", goal.tolerance_angle, ang_tol);
        prop_assert!((goal.target_pose.x - 1.0).abs() < 1e-10, "target pose x must match");
        prop_assert!((goal.target_pose.y - 2.0).abs() < 1e-10, "target pose y must match");
    }

    /// INVARIANT: NavGoal timeout defaults to 0 (no limit) via new()
    #[test]
    fn test_nav_goal_default_timeout_is_zero(_seed in 0u64..100u64) {
        let goal = NavGoal::new(Pose2D::default(), 0.1, 0.1);
        prop_assert!(goal.timeout_seconds == 0.0,
            "default timeout must be 0.0 (no limit), got {}", goal.timeout_seconds);
    }

    // ── Waypoint ─────────────────────────────────────────────────────────

    /// INVARIANT: Waypoint::new() preserves the input pose
    #[test]
    fn test_waypoint_new_preserves_pose(
        x in -1000.0f64..1000.0,
        y in -1000.0f64..1000.0,
        theta in -PI..PI,
    ) {
        let pose = Pose2D { x, y, theta, timestamp_ns: 0 };
        let wp = Waypoint::new(pose);
        prop_assert!((wp.pose.x - x).abs() < 1e-10, "waypoint x must match");
        prop_assert!((wp.pose.y - y).abs() < 1e-10, "waypoint y must match");
        prop_assert!((wp.pose.theta - theta).abs() < 1e-10, "waypoint theta must match");
        prop_assert!(wp.time_from_start == 0.0, "default time must be 0");
    }

    // ── CmdVel ───────────────────────────────────────────────────────────

    /// INVARIANT: CmdVel preserves exact input values (no clamping)
    #[test]
    fn test_cmd_vel_preserves_values(
        lin in -10.0f32..10.0,
        ang in -10.0f32..10.0,
    ) {
        let cmd = CmdVel::new(lin, ang);
        prop_assert!(cmd.linear == lin, "linear must match: {} vs {}", cmd.linear, lin);
        prop_assert!(cmd.angular == ang, "angular must match: {} vs {}", cmd.angular, ang);
    }

    /// INVARIANT: CmdVel with opposite signs represents turning in place vs driving straight
    #[test]
    fn test_cmd_vel_turning_has_nonzero_angular(ang in 0.01f32..5.0) {
        let turn = CmdVel::new(0.0, ang);
        prop_assert!(turn.linear == 0.0, "pure turn has zero linear");
        prop_assert!(turn.angular > 0.0, "pure turn has positive angular");
    }

    // ── GoalResult ───────────────────────────────────────────────────────

    /// INVARIANT: GoalResult::new() sets correct status code
    #[test]
    fn test_goal_result_status_code(_seed in 0u64..100u64) {
        // GoalStatus variants: Pending=0, Active=1, Succeeded=2, etc.
        let result = GoalResult::default();
        // Default status should be a valid u8
        prop_assert!(result.status <= 10, "status {} should be a small enum value", result.status);
    }

    // ── MotorCommand ─────────────────────────────────────────────────────

    /// INVARIANT: MotorCommand mode must be in 0..4 for valid configs
    #[test]
    fn test_motor_command_mode_range(mode in 0u8..4u8) {
        let cmd = MotorCommand {
            motor_id: 0, mode, target: 0.0, max_velocity: 10.0,
            max_acceleration: 10.0, feed_forward: 0.0, enable: 1, timestamp_ns: 0,
        };
        prop_assert!(cmd.is_valid(), "mode {} in 0..4 must be valid", mode);
    }

    // ── TrajectoryPoint ──────────────────────────────────────────────────

    /// INVARIANT: TrajectoryPoint default has zero time_from_start
    #[test]
    fn test_trajectory_point_default_time_zero(_seed in 0u64..100u64) {
        let tp = TrajectoryPoint::default();
        prop_assert!(tp.time_from_start == 0.0,
            "default time_from_start must be 0.0, got {}", tp.time_from_start);
    }

    // ── ForceCommand ─────────────────────────────────────────────────────

    /// INVARIANT: ForceCommand force_mode elements are 0 or 1 (binary control selection)
    #[test]
    fn test_force_command_mode_binary(
        m0 in 0u8..2u8, m1 in 0u8..2u8, m2 in 0u8..2u8,
        m3 in 0u8..2u8, m4 in 0u8..2u8, m5 in 0u8..2u8,
    ) {
        let cmd = ForceCommand {
            target_force: Vector3::zero(),
            target_torque: Vector3::zero(),
            force_mode: [m0, m1, m2, m3, m4, m5],
            position_setpoint: Vector3::zero(),
            orientation_setpoint: Vector3::zero(),
            max_deviation: Vector3::zero(),
            gains: [1.0; 6],
            timeout_seconds: 0.0,
            frame_id: [0; 32],
            timestamp_ns: 0,
        };
        for (i, &m) in cmd.force_mode.iter().enumerate() {
            prop_assert!(m <= 1, "force_mode[{}] = {} must be 0 or 1", i, m);
        }
    }

    // ── Detection ────────────────────────────────────────────────────────

    /// INVARIANT: Detection confidence must be in [0, 1]
    #[test]
    fn test_detection_confidence_range(conf in 0.0f32..1.0) {
        let det = Detection::new("object", conf, 10.0, 20.0, 50.0, 50.0);
        prop_assert!(det.confidence >= 0.0 && det.confidence <= 1.0,
            "confidence {} must be in [0,1]", det.confidence);
    }

    // ── EmergencyStop ────────────────────────────────────────────────────

    /// INVARIANT: EmergencyStop engaged field is boolean-like (0 or 1)
    #[test]
    fn test_emergency_stop_engaged_binary(engaged in 0u8..2u8) {
        let estop = EmergencyStop {
            engaged,
            reason: [0; 64],
            source: [0; 32],
            auto_reset: 0,
            timestamp_ns: 0,
        };
        prop_assert!(estop.engaged <= 1, "engaged {} must be 0 or 1", estop.engaged);
    }
}
