//! Shared topic naming convention for the HORUS ecosystem.
//!
//! Both horus-sim3d (simulator) and horus_core (driver system) use this
//! convention to derive topic names from URDF sensor/actuator names.
//! Because both sides use the same function, topic names align automatically
//! — robot code works identically with simulated or real sensors.
//!
//! # Convention
//!
//! ```text
//! Format: {robot_name}.{sensor_name}.{data_type}
//!
//! Examples:
//!   turtlebot.front_lidar.scan
//!   turtlebot.imu_sensor.imu
//!   turtlebot.rgb_camera.image
//!   turtlebot.rgb_camera.depth
//!   turtlebot.rgb_camera.camera_info
//!   turtlebot.odom                    (robot-level, no sensor_name)
//!   turtlebot.joint_state             (robot-level, no sensor_name)
//! ```
//!
//! Empty segments are omitted — no double dots ever appear.
//! Separator is always `.` (not `/`) because HORUS topics are backed by
//! shared memory files under `/dev/shm/horus/topics/`.

/// Construct a HORUS topic name from robot name, sensor name, and data type suffix.
///
/// Joins non-empty segments with `.`. Empty segments are omitted.
///
/// # Examples
///
/// ```
/// use horus_library::topic_convention::sensor_topic;
///
/// assert_eq!(sensor_topic("turtlebot", "front_lidar", "scan"), "turtlebot.front_lidar.scan");
/// assert_eq!(sensor_topic("turtlebot", "", "odom"), "turtlebot.odom");
/// assert_eq!(sensor_topic("", "lidar", "scan"), "lidar.scan");
/// ```
pub fn sensor_topic(robot_name: &str, sensor_name: &str, data_type: &str) -> String {
    let mut segments = Vec::with_capacity(3);
    if !robot_name.is_empty() {
        segments.push(robot_name);
    }
    if !sensor_name.is_empty() {
        segments.push(sensor_name);
    }
    if !data_type.is_empty() {
        segments.push(data_type);
    }
    segments.join(".")
}

/// Construct a topic name with an optional namespace prefix.
///
/// When `namespace` is non-empty, it's prepended as the first segment.
/// Used by sim3d in standalone mode (e.g., `sim.turtlebot.front_lidar.scan`).
/// In driver mode (`horus run --sim`), namespace is empty — topics match
/// the driver side exactly.
pub fn sensor_topic_with_namespace(
    namespace: &str,
    robot_name: &str,
    sensor_name: &str,
    data_type: &str,
) -> String {
    let mut segments = Vec::with_capacity(4);
    if !namespace.is_empty() {
        segments.push(namespace);
    }
    if !robot_name.is_empty() {
        segments.push(robot_name);
    }
    if !sensor_name.is_empty() {
        segments.push(sensor_name);
    }
    if !data_type.is_empty() {
        segments.push(data_type);
    }
    segments.join(".")
}

/// Standard data type suffixes for each sensor type.
///
/// These are the canonical suffixes used by both sim3d and horus drivers.
/// Using any other suffix for these sensor types will break sim/real alignment.
pub mod suffix {
    pub const IMU: &str = "imu";
    pub const SCAN: &str = "scan";
    pub const GPS: &str = "gps";
    pub const WRENCH: &str = "wrench";
    pub const SONAR: &str = "sonar";
    pub const ENCODER: &str = "encoder";
    pub const RADAR: &str = "radar";
    pub const ODOMETRY: &str = "odom";
    pub const JOINT_STATE: &str = "joint_state";
    pub const IMAGE: &str = "image";
    pub const DEPTH: &str = "depth";
    pub const CAMERA_INFO: &str = "camera_info";
    pub const SEGMENTATION: &str = "segmentation";
    pub const THERMAL: &str = "thermal";
    pub const EVENT_CAMERA: &str = "event_camera";
    pub const POINTCLOUD: &str = "pointcloud";
    pub const CMD_VEL: &str = "cmd_vel";
    pub const JOINT_CMD: &str = "joint_cmd";
}

/// Well-known service names for simulator control.
///
/// Any simulator plugin (sim3d, mujoco, isaac) listens on these names.
/// Horus nodes call `Service::<Req, Resp>::new(service::SPAWN)` etc.
pub mod service {
    pub const SPAWN: &str = "sim.spawn";
    pub const DESPAWN: &str = "sim.despawn";
    pub const TELEPORT: &str = "sim.teleport";
    pub const PAUSE: &str = "sim.pause";
    pub const RESUME: &str = "sim.resume";
    pub const RAYCAST: &str = "sim.raycast";
    pub const GET_STATE: &str = "sim.state.get";
    pub const SET_STATE: &str = "sim.state.set";
    pub const SET_PARAM: &str = "sim.param.set";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_topic_name() {
        assert_eq!(
            sensor_topic("turtlebot", "front_lidar", "scan"),
            "turtlebot.front_lidar.scan"
        );
    }

    #[test]
    fn robot_level_topic_no_sensor_name() {
        assert_eq!(sensor_topic("turtlebot", "", "odom"), "turtlebot.odom");
    }

    #[test]
    fn empty_robot_name() {
        assert_eq!(sensor_topic("", "lidar", "scan"), "lidar.scan");
    }

    #[test]
    fn all_empty_produces_empty() {
        assert_eq!(sensor_topic("", "", ""), "");
    }

    #[test]
    fn no_double_dots() {
        let topic = sensor_topic("bot", "", "imu");
        assert!(!topic.contains(".."), "Topic '{}' has double dots", topic);
    }

    #[test]
    fn with_namespace() {
        assert_eq!(
            sensor_topic_with_namespace("sim", "turtlebot", "front_lidar", "scan"),
            "sim.turtlebot.front_lidar.scan"
        );
    }

    #[test]
    fn without_namespace_matches_bare() {
        assert_eq!(
            sensor_topic_with_namespace("", "turtlebot", "front_lidar", "scan"),
            sensor_topic("turtlebot", "front_lidar", "scan")
        );
    }

    // ── Sim-Real alignment tests ──────────────────────────────────────

    /// Verifies that the shared convention produces the same topic name
    /// regardless of which side calls it — this is the sim-real contract.
    #[test]
    fn sim_and_driver_produce_same_name_for_lidar() {
        // sim3d side: SensorMeta::topic_name() delegates to sensor_topic_with_namespace
        let sim_topic = sensor_topic_with_namespace("", "turtlebot", "front_lidar", "scan");
        // driver side: HardwareSet::resolve_topic_name uses sensor_topic
        let driver_topic = sensor_topic("turtlebot", "front_lidar", "scan");
        assert_eq!(sim_topic, driver_topic);
        assert_eq!(sim_topic, "turtlebot.front_lidar.scan");
    }

    #[test]
    fn sim_and_driver_produce_same_name_for_imu() {
        let sim = sensor_topic_with_namespace("", "turtlebot", "imu_sensor", "imu");
        let driver = sensor_topic("turtlebot", "imu_sensor", "imu");
        assert_eq!(sim, driver);
        assert_eq!(sim, "turtlebot.imu_sensor.imu");
    }

    #[test]
    fn sim_and_driver_produce_same_name_for_camera() {
        let sim = sensor_topic_with_namespace("", "bot", "rgb_camera", "image");
        let driver = sensor_topic("bot", "rgb_camera", "image");
        assert_eq!(sim, driver);
    }

    #[test]
    fn sim_and_driver_produce_same_name_for_robot_level() {
        let sim = sensor_topic_with_namespace("", "turtlebot", "", "odom");
        let driver = sensor_topic("turtlebot", "", "odom");
        assert_eq!(sim, driver);
        assert_eq!(sim, "turtlebot.odom");
    }

    #[test]
    fn namespace_breaks_alignment_intentionally() {
        let sim_with_ns = sensor_topic_with_namespace("sim", "bot", "lidar", "scan");
        let driver = sensor_topic("bot", "lidar", "scan");
        assert_ne!(sim_with_ns, driver, "namespace prefix should differ from driver");
        assert_eq!(sim_with_ns, "sim.bot.lidar.scan");
        assert_eq!(driver, "bot.lidar.scan");
    }

    #[test]
    fn all_sensor_suffixes_produce_aligned_names() {
        let suffixes = [
            suffix::IMU, suffix::SCAN, suffix::GPS, suffix::WRENCH,
            suffix::SONAR, suffix::ENCODER, suffix::RADAR, suffix::ODOMETRY,
            suffix::IMAGE, suffix::DEPTH, suffix::CAMERA_INFO,
        ];
        for s in &suffixes {
            let sim = sensor_topic_with_namespace("", "robot", "sensor", s);
            let driver = sensor_topic("robot", "sensor", s);
            assert_eq!(sim, driver, "Alignment broken for suffix '{}'", s);
        }
    }

    #[test]
    fn suffix_constants_are_unique() {
        let suffixes = [
            suffix::IMU, suffix::SCAN, suffix::GPS, suffix::WRENCH,
            suffix::SONAR, suffix::ENCODER, suffix::RADAR, suffix::ODOMETRY,
            suffix::JOINT_STATE, suffix::IMAGE, suffix::DEPTH,
            suffix::CAMERA_INFO, suffix::SEGMENTATION, suffix::THERMAL,
            suffix::EVENT_CAMERA, suffix::POINTCLOUD, suffix::CMD_VEL,
            suffix::JOINT_CMD,
        ];
        let mut seen = std::collections::HashSet::new();
        for s in &suffixes {
            assert!(seen.insert(s), "Duplicate suffix: {}", s);
        }
    }

    // === Boundary / edge case tests (SLAM Cycle 2) ===

    #[test]
    fn only_data_type_produces_bare_suffix() {
        assert_eq!(sensor_topic("", "", "scan"), "scan");
    }

    #[test]
    fn dots_in_names_preserved() {
        // Users shouldn't put dots in names, but verify no double-dots
        let result = sensor_topic("ns.robot", "sensor.name", "type");
        assert!(!result.contains(".."));
        assert_eq!(result, "ns.robot.sensor.name.type");
    }

    #[test]
    fn hyphen_and_underscore_in_names() {
        let result = sensor_topic("my-robot", "front_lidar_2", "scan");
        assert_eq!(result, "my-robot.front_lidar_2.scan");
    }

    #[test]
    fn unicode_names_work() {
        let result = sensor_topic("ロボット", "センサー", "imu");
        assert_eq!(result, "ロボット.センサー.imu");
    }

    #[test]
    fn long_names_work() {
        let long_name = "a".repeat(500);
        let result = sensor_topic(&long_name, "sensor", "scan");
        assert!(result.starts_with(&long_name));
        assert!(result.ends_with("scan"));
    }

    // ── Service name tests ──────────────────────────────────────────

    #[test]
    fn service_names_are_unique() {
        let names = [
            service::SPAWN, service::DESPAWN, service::TELEPORT,
            service::PAUSE, service::RESUME, service::RAYCAST,
            service::GET_STATE, service::SET_STATE, service::SET_PARAM,
        ];
        let mut seen = std::collections::HashSet::new();
        for name in &names {
            assert!(seen.insert(name), "Duplicate service name: {}", name);
        }
    }

    #[test]
    fn service_names_follow_sim_prefix_convention() {
        let names = [
            service::SPAWN, service::DESPAWN, service::TELEPORT,
            service::PAUSE, service::RESUME, service::RAYCAST,
            service::GET_STATE, service::SET_STATE, service::SET_PARAM,
        ];
        for name in &names {
            assert!(name.starts_with("sim."), "Service '{}' must start with 'sim.'", name);
        }
    }

    #[test]
    fn service_names_dont_collide_with_sensor_suffixes() {
        let suffixes = [
            suffix::IMU, suffix::SCAN, suffix::GPS, suffix::WRENCH,
            suffix::SONAR, suffix::ENCODER, suffix::RADAR, suffix::ODOMETRY,
            suffix::JOINT_STATE, suffix::IMAGE, suffix::DEPTH,
            suffix::CAMERA_INFO, suffix::SEGMENTATION, suffix::THERMAL,
            suffix::EVENT_CAMERA, suffix::POINTCLOUD, suffix::CMD_VEL,
            suffix::JOINT_CMD,
        ];
        let services = [
            service::SPAWN, service::DESPAWN, service::TELEPORT,
            service::PAUSE, service::RESUME, service::RAYCAST,
            service::GET_STATE, service::SET_STATE, service::SET_PARAM,
        ];
        for svc in &services {
            for sfx in &suffixes {
                assert_ne!(*svc, *sfx, "Service name '{}' collides with suffix '{}'", svc, sfx);
            }
        }
    }

    #[test]
    fn sensor_topic_and_service_names_use_dot_separator() {
        // All names use dots, never slashes (SHM constraint)
        let all_names: Vec<&str> = [
            suffix::IMU, suffix::SCAN, suffix::CMD_VEL,
            service::SPAWN, service::RAYCAST, service::GET_STATE,
        ].to_vec();
        for name in &all_names {
            assert!(!name.contains('/'), "'{}' contains slash", name);
        }
    }
}
