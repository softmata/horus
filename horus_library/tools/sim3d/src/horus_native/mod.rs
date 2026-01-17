//! Native HORUS Integration for Sim3D
//!
//! Config-driven, auto-wiring HORUS integration.
//! Like Gazebo/Isaac Sim - declare in config, sim3d connects automatically.
//!
//! # Usage
//!
//! In robot URDF/config:
//! ```yaml
//! horus:
//!   cmd_vel: subscribe
//!   odom: publish
//!   scan: publish
//!   imu: publish
//! ```
//!
//! Or in code:
//! ```rust,ignore
//! app.add_plugins(HorusNativePlugin::new("turtlebot"));
//! // That's it - auto-connects all standard topics
//! ```

use bevy::prelude::*;
use horus_core::communication::Topic;
use horus_library::messages::{
    control::JointCommand,
    geometry::Twist,
    sensor::{Imu, LaserScan, Odometry},
};
use std::collections::HashMap;

/// HORUS topic configuration for a robot
#[derive(Clone, Debug, Default)]
pub struct HorusTopicConfig {
    /// Subscribe to cmd_vel (velocity commands)
    pub cmd_vel: bool,
    /// Publish odometry
    pub odom: bool,
    /// Publish laser scan
    pub scan: bool,
    /// Publish IMU data
    pub imu: bool,
    /// Subscribe to joint commands (articulated robots)
    pub joint_cmd: bool,
    /// Publish joint states
    pub joint_states: bool,
}

impl HorusTopicConfig {
    /// Default config for differential drive robots
    pub fn diff_drive() -> Self {
        Self {
            cmd_vel: true,
            odom: true,
            scan: true,
            imu: true,
            joint_cmd: false,
            joint_states: false,
        }
    }

    /// Default config for articulated robots (arms)
    pub fn articulated() -> Self {
        Self {
            cmd_vel: false,
            odom: false,
            scan: false,
            imu: false,
            joint_cmd: true,
            joint_states: true,
        }
    }

    /// Full config - all topics enabled
    pub fn all() -> Self {
        Self {
            cmd_vel: true,
            odom: true,
            scan: true,
            imu: true,
            joint_cmd: true,
            joint_states: true,
        }
    }
}

/// Per-robot HORUS communication hubs
pub struct RobotHubs {
    pub cmd_vel_sub: Option<Topic<Twist>>,
    pub odom_pub: Option<Topic<Odometry>>,
    pub scan_pub: Option<Topic<LaserScan>>,
    pub imu_pub: Option<Topic<Imu>>,
    pub joint_cmd_sub: Option<Topic<JointCommand>>,
    pub joint_state_pub: Option<Topic<JointCommand>>, // Reuse JointCommand for states
}

impl RobotHubs {
    /// Create hubs based on config
    pub fn from_config(robot_name: &str, config: &HorusTopicConfig) -> Self {
        let prefix = robot_name;

        Self {
            cmd_vel_sub: if config.cmd_vel {
                Topic::new(&format!("{}.cmd_vel", prefix)).ok()
            } else {
                None
            },
            odom_pub: if config.odom {
                Topic::new(&format!("{}.odom", prefix)).ok()
            } else {
                None
            },
            scan_pub: if config.scan {
                Topic::new(&format!("{}.scan", prefix)).ok()
            } else {
                None
            },
            imu_pub: if config.imu {
                Topic::new(&format!("{}.imu", prefix)).ok()
            } else {
                None
            },
            joint_cmd_sub: if config.joint_cmd {
                Topic::new(&format!("{}.joint_cmd", prefix)).ok()
            } else {
                None
            },
            joint_state_pub: if config.joint_states {
                Topic::new(&format!("{}.joint_states", prefix)).ok()
            } else {
                None
            },
        }
    }

    /// Log connected topics
    pub fn log_connections(&self, robot_name: &str) {
        let mut topics = Vec::new();
        if self.cmd_vel_sub.is_some() {
            topics.push(format!("{}.cmd_vel [SUB]", robot_name));
        }
        if self.odom_pub.is_some() {
            topics.push(format!("{}.odom [PUB]", robot_name));
        }
        if self.scan_pub.is_some() {
            topics.push(format!("{}.scan [PUB]", robot_name));
        }
        if self.imu_pub.is_some() {
            topics.push(format!("{}.imu [PUB]", robot_name));
        }
        if self.joint_cmd_sub.is_some() {
            topics.push(format!("{}.joint_cmd [SUB]", robot_name));
        }
        if self.joint_state_pub.is_some() {
            topics.push(format!("{}.joint_states [PUB]", robot_name));
        }

        if !topics.is_empty() {
            tracing::info!(
                "HORUS connected for '{}': {}",
                robot_name,
                topics.join(", ")
            );
        }
    }
}

/// Central HORUS communication resource
///
/// Holds all Hub instances, indexed by robot name.
/// Systems access this to send/receive messages.
#[derive(Resource, Default)]
pub struct HorusComm {
    pub robot_hubs: HashMap<String, RobotHubs>,
}

impl HorusComm {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a robot with the given topic config
    pub fn add_robot(&mut self, name: &str, config: &HorusTopicConfig) {
        let hubs = RobotHubs::from_config(name, config);
        hubs.log_connections(name);
        self.robot_hubs.insert(name.to_string(), hubs);
    }

    /// Get hubs for a robot
    pub fn get(&self, robot_name: &str) -> Option<&RobotHubs> {
        self.robot_hubs.get(robot_name)
    }

    /// Get mutable hubs for a robot
    pub fn get_mut(&mut self, robot_name: &str) -> Option<&mut RobotHubs> {
        self.robot_hubs.get_mut(robot_name)
    }
}

/// Component to tag robots with HORUS integration
#[derive(Component, Clone, Debug)]
pub struct RobotCommandHandler {
    pub cmd_vel_topic: String,
}

impl RobotCommandHandler {
    pub fn new(robot_name: impl Into<String>) -> Self {
        Self {
            cmd_vel_topic: format!("{}.cmd_vel", robot_name.into()),
        }
    }
}

/// Plugin for automatic HORUS integration
///
/// Just add this plugin - it auto-wires everything.
pub struct HorusNativePlugin {
    robot_name: String,
    config: HorusTopicConfig,
}

impl HorusNativePlugin {
    /// Create plugin for a robot with default diff-drive config
    pub fn new(robot_name: impl Into<String>) -> Self {
        Self {
            robot_name: robot_name.into(),
            config: HorusTopicConfig::diff_drive(),
        }
    }

    /// Create with custom topic config
    pub fn with_config(robot_name: impl Into<String>, config: HorusTopicConfig) -> Self {
        Self {
            robot_name: robot_name.into(),
            config,
        }
    }

    /// Create for articulated robot
    pub fn articulated(robot_name: impl Into<String>) -> Self {
        Self {
            robot_name: robot_name.into(),
            config: HorusTopicConfig::articulated(),
        }
    }
}

impl Plugin for HorusNativePlugin {
    fn build(&self, app: &mut App) {
        // Initialize or get existing HorusComm
        if !app.world().contains_resource::<HorusComm>() {
            app.init_resource::<HorusComm>();
        }

        // Register robot on startup
        let robot_name = self.robot_name.clone();
        let config = self.config.clone();

        app.add_systems(Startup, move |mut horus_comm: ResMut<HorusComm>| {
            horus_comm.add_robot(&robot_name, &config);
        });
    }
}

// Re-export for convenience
#[allow(unused_imports)]
pub mod prelude {
    pub use super::{
        HorusComm, HorusNativePlugin, HorusTopicConfig, RobotCommandHandler, RobotHubs,
    };
    pub use horus_core::communication::Topic;
    pub use horus_library::messages::{
        control::JointCommand,
        geometry::Twist,
        sensor::{Imu, LaserScan, Odometry},
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topic_config_presets() {
        let diff = HorusTopicConfig::diff_drive();
        assert!(diff.cmd_vel);
        assert!(diff.odom);
        assert!(!diff.joint_cmd);

        let arm = HorusTopicConfig::articulated();
        assert!(!arm.cmd_vel);
        assert!(arm.joint_cmd);
        assert!(arm.joint_states);
    }

    #[test]
    fn test_robot_command_handler() {
        let handler = RobotCommandHandler::new("turtlebot");
        assert_eq!(handler.cmd_vel_topic, "turtlebot.cmd_vel");
    }
}
