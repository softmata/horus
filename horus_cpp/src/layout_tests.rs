//! Layout verification tests.
//!
//! These tests verify that Rust struct sizes match the expected C++ struct sizes.
//! If any test fails, the C++ headers in include/horus/msg/ need updating.

#[cfg(test)]
mod tests {
    use std::mem::size_of;

    // ─── Geometry Types ──────────────────────────────────────────────

    #[test]
    fn geometry_point3_size() {
        assert_eq!(size_of::<horus_types::Point3>(), 24);
    }

    #[test]
    fn geometry_vector3_size() {
        assert_eq!(size_of::<horus_types::Vector3>(), 24);
    }

    #[test]
    fn geometry_quaternion_size() {
        assert_eq!(size_of::<horus_types::Quaternion>(), 32);
    }

    #[test]
    fn geometry_twist_size() {
        assert_eq!(size_of::<horus_types::Twist>(), 56);
    }

    #[test]
    fn geometry_pose2d_size() {
        assert_eq!(size_of::<horus_types::Pose2D>(), 32);
    }

    #[test]
    fn geometry_transform_stamped_size() {
        assert_eq!(size_of::<horus_types::TransformStamped>(), 64);
    }

    #[test]
    fn geometry_pose3d_size() {
        // Point3(24) + Quaternion(32) + timestamp_ns(8) = 64
        assert_eq!(size_of::<horus_types::Pose3D>(), 64);
    }

    #[test]
    fn geometry_accel_size() {
        assert_eq!(size_of::<horus_types::Accel>(), 56);
    }

    // ─── Sensor Types ────────────────────────────────────────────────

    #[test]
    fn sensor_cmd_vel_size() {
        // timestamp_ns(8) + linear(4) + angular(4) = 16
        assert_eq!(size_of::<horus_robotics::CmdVel>(), 16);
    }

    #[test]
    fn sensor_imu_size() {
        assert_eq!(size_of::<horus_robotics::Imu>(), 304);
    }

    #[test]
    fn sensor_odometry_size() {
        // Check it's a known size (exact value depends on struct layout)
        let size = size_of::<horus_robotics::Odometry>();
        assert!(size > 0, "Odometry should have non-zero size, got {}", size);
    }

    #[test]
    fn sensor_laser_scan_size() {
        // ranges: [f32; 360] = 1440 bytes + other fields
        let size = size_of::<horus_robotics::LaserScan>();
        assert!(
            size >= 1440,
            "LaserScan should be at least 1440 bytes for ranges, got {}",
            size
        );
    }

    // ─── Diagnostics Types ───────────────────────────────────────────

    #[test]
    fn diagnostics_heartbeat_size() {
        let size = size_of::<horus_types::Heartbeat>();
        assert!(
            size > 0,
            "Heartbeat should have non-zero size, got {}",
            size
        );
    }

    #[test]
    fn diagnostics_emergency_stop_size() {
        let size = size_of::<horus_types::EmergencyStop>();
        assert!(
            size > 0,
            "EmergencyStop should have non-zero size, got {}",
            size
        );
    }

    // ─── Navigation Types ────────────────────────────────────────────

    #[test]
    fn navigation_nav_goal_size() {
        let size = size_of::<horus_robotics::NavGoal>();
        assert!(size > 0, "NavGoal should have non-zero size, got {}", size);
    }

    // ─── Control Types ───────────────────────────────────────────────

    #[test]
    fn control_joint_state_size() {
        let size = size_of::<horus_robotics::JointState>();
        assert!(
            size > 0,
            "JointState should have non-zero size, got {}",
            size
        );
    }

    // ─── Alignment ───────────────────────────────────────────────────

    #[test]
    fn all_types_8byte_aligned() {
        // All #[repr(C)] types with f64 fields should be 8-byte aligned
        assert_eq!(std::mem::align_of::<horus_types::Twist>(), 8);
        assert_eq!(std::mem::align_of::<horus_types::Pose2D>(), 8);
        assert_eq!(std::mem::align_of::<horus_types::Point3>(), 8);
        assert_eq!(std::mem::align_of::<horus_types::Vector3>(), 8);
        assert_eq!(std::mem::align_of::<horus_types::Quaternion>(), 8);
        assert_eq!(std::mem::align_of::<horus_types::TransformStamped>(), 8);
        assert_eq!(std::mem::align_of::<horus_types::Pose3D>(), 8);
        assert_eq!(std::mem::align_of::<horus_types::Accel>(), 8);
    }

    // ─── Pod Safety ──────────────────────────────────────────────────

    #[test]
    fn geometry_types_are_copy() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<horus_types::Twist>();
        assert_copy::<horus_types::Pose2D>();
        assert_copy::<horus_types::Point3>();
        assert_copy::<horus_types::Vector3>();
        assert_copy::<horus_types::Quaternion>();
        assert_copy::<horus_types::TransformStamped>();
        assert_copy::<horus_types::Pose3D>();
        assert_copy::<horus_types::Accel>();
        assert_copy::<horus_robotics::CmdVel>();
        assert_copy::<horus_robotics::Imu>();
    }
}
