//! HORUS Transport Layer - Real integration with horus_core::Hub
//!
//! This module provides actual inter-process communication using horus_core's
//! Hub system for shared memory and network transport.

use bevy::prelude::*;
use horus_core::communication::Hub;
use horus_core::core::LogSummary;
use std::sync::{Arc, Mutex};

use super::messages::*;

// ============================================================================
// LogSummary Implementations for Message Types
// ============================================================================

impl LogSummary for Twist {
    fn log_summary(&self) -> String {
        format!(
            "Twist(lin:[{:.2},{:.2},{:.2}] ang:[{:.2},{:.2},{:.2}])",
            self.linear.x,
            self.linear.y,
            self.linear.z,
            self.angular.x,
            self.angular.y,
            self.angular.z
        )
    }
}

impl LogSummary for TransformStamped {
    fn log_summary(&self) -> String {
        format!(
            "HFrame({}->{} t:{:.3})",
            self.header.frame_id, self.child_frame_id, self.header.stamp
        )
    }
}

impl LogSummary for PointCloud2 {
    fn log_summary(&self) -> String {
        format!(
            "PointCloud2(frame:{} pts:{} t:{:.3})",
            self.header.frame_id,
            self.width * self.height,
            self.header.stamp
        )
    }
}

impl LogSummary for LaserScanMessage {
    fn log_summary(&self) -> String {
        format!(
            "LaserScan(frame:{} rays:{} t:{:.3})",
            self.header.frame_id,
            self.ranges.len(),
            self.header.stamp
        )
    }
}

impl LogSummary for Odometry {
    fn log_summary(&self) -> String {
        format!(
            "Odom(frame:{} pos:[{:.2},{:.2},{:.2}] t:{:.3})",
            self.header.frame_id,
            self.pose.pose.position.x,
            self.pose.pose.position.y,
            self.pose.pose.position.z,
            self.header.stamp
        )
    }
}

impl LogSummary for JointState {
    fn log_summary(&self) -> String {
        format!(
            "JointState(joints:{} t:{:.3})",
            self.name.len(),
            self.header.stamp
        )
    }
}

impl LogSummary for Imu {
    fn log_summary(&self) -> String {
        format!(
            "IMU(frame:{} t:{:.3})",
            self.header.frame_id, self.header.stamp
        )
    }
}

impl LogSummary for ImageMessage {
    fn log_summary(&self) -> String {
        format!(
            "Image(frame:{} {}x{} {} t:{:.3})",
            self.header.frame_id, self.width, self.height, self.encoding, self.header.stamp
        )
    }
}

// ============================================================================
// HORUS Transport - Real Inter-Process Communication
// ============================================================================

/// Configuration for HORUS transport
#[derive(Resource, Clone)]
pub struct HorusTransportConfig {
    /// Enable network transport (if false, only local shared memory)
    pub enable_network: bool,
    /// Network endpoint for remote connections (e.g., "topic@192.168.1.5:9000")
    pub network_endpoint: Option<String>,
    /// Robot name prefix for topics
    pub robot_name: String,
}

impl Default for HorusTransportConfig {
    fn default() -> Self {
        Self {
            enable_network: false,
            network_endpoint: None,
            robot_name: "sim3d_robot".to_string(),
        }
    }
}

/// Real HORUS transport using horus_core::Hub
/// Provides actual shared memory and network communication
#[derive(Resource)]
pub struct HorusTransport {
    /// Hub for Twist/cmd_vel messages (subscriber)
    cmd_vel_hub: Option<Arc<Mutex<Hub<Twist>>>>,
    /// Hub for HFrame (transform) messages (publisher)
    hf_hub: Option<Arc<Mutex<Hub<TransformStamped>>>>,
    /// Hub for PointCloud2 messages (publisher)
    pointcloud_hub: Option<Arc<Mutex<Hub<PointCloud2>>>>,
    /// Hub for LaserScan messages (publisher)
    laserscan_hub: Option<Arc<Mutex<Hub<LaserScanMessage>>>>,
    /// Hub for Odometry messages (publisher)
    odom_hub: Option<Arc<Mutex<Hub<Odometry>>>>,
    /// Hub for JointState messages (publisher)
    joint_state_hub: Option<Arc<Mutex<Hub<JointState>>>>,
    /// Transport enabled flag
    enabled: bool,
    /// Robot name for topic naming
    robot_name: String,
}

impl Default for HorusTransport {
    fn default() -> Self {
        Self::new("sim3d_robot")
    }
}

impl HorusTransport {
    /// Create a new HORUS transport with default local topics
    pub fn new(robot_name: &str) -> Self {
        let mut transport = Self {
            cmd_vel_hub: None,
            hf_hub: None,
            pointcloud_hub: None,
            laserscan_hub: None,
            odom_hub: None,
            joint_state_hub: None,
            enabled: true,
            robot_name: robot_name.to_string(),
        };

        // Initialize hubs (log errors but don't fail)
        transport.init_hubs();
        transport
    }

    /// Create a new HORUS transport with network endpoint
    pub fn with_network(robot_name: &str, endpoint: &str) -> Self {
        let mut transport = Self {
            cmd_vel_hub: None,
            hf_hub: None,
            pointcloud_hub: None,
            laserscan_hub: None,
            odom_hub: None,
            joint_state_hub: None,
            enabled: true,
            robot_name: robot_name.to_string(),
        };

        transport.init_network_hubs(endpoint);
        transport
    }

    fn init_hubs(&mut self) {
        // Create local shared memory hubs
        let cmd_vel_topic = format!("{}.cmd_vel", self.robot_name);
        let hf_topic = format!("{}.hf", self.robot_name);
        let pointcloud_topic = format!("{}.pointcloud", self.robot_name);
        let laserscan_topic = format!("{}.scan", self.robot_name);
        let odom_topic = format!("{}.odom", self.robot_name);
        let joint_state_topic = format!("{}.joint_states", self.robot_name);

        if let Ok(hub) = Hub::<Twist>::new(&cmd_vel_topic) {
            self.cmd_vel_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<TransformStamped>::new(&hf_topic) {
            self.hf_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<PointCloud2>::new(&pointcloud_topic) {
            self.pointcloud_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<LaserScanMessage>::new(&laserscan_topic) {
            self.laserscan_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<Odometry>::new(&odom_topic) {
            self.odom_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<JointState>::new(&joint_state_topic) {
            self.joint_state_hub = Some(Arc::new(Mutex::new(hub)));
        }
    }

    fn init_network_hubs(&mut self, endpoint: &str) {
        // Create network hubs with endpoint
        let cmd_vel_topic = format!("{}.cmd_vel@{}", self.robot_name, endpoint);
        let hf_topic = format!("{}.hf@{}", self.robot_name, endpoint);
        let pointcloud_topic = format!("{}.pointcloud@{}", self.robot_name, endpoint);
        let laserscan_topic = format!("{}.scan@{}", self.robot_name, endpoint);
        let odom_topic = format!("{}.odom@{}", self.robot_name, endpoint);
        let joint_state_topic = format!("{}.joint_states@{}", self.robot_name, endpoint);

        if let Ok(hub) = Hub::<Twist>::new(&cmd_vel_topic) {
            self.cmd_vel_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<TransformStamped>::new(&hf_topic) {
            self.hf_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<PointCloud2>::new(&pointcloud_topic) {
            self.pointcloud_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<LaserScanMessage>::new(&laserscan_topic) {
            self.laserscan_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<Odometry>::new(&odom_topic) {
            self.odom_hub = Some(Arc::new(Mutex::new(hub)));
        }
        if let Ok(hub) = Hub::<JointState>::new(&joint_state_topic) {
            self.joint_state_hub = Some(Arc::new(Mutex::new(hub)));
        }
    }

    /// Enable the transport
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable the transport
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Check if transport is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Get robot name
    pub fn robot_name(&self) -> &str {
        &self.robot_name
    }

    // ========================================================================
    // Publishing Methods
    // ========================================================================

    /// Publish a HFrame (transform) message
    pub fn publish_hf(&self, msg: TransformStamped) -> bool {
        if !self.enabled {
            return false;
        }
        if let Some(ref hub) = self.hf_hub {
            if let Ok(hub_guard) = hub.lock() {
                return hub_guard.send(msg, &mut None).is_ok();
            }
        }
        false
    }

    /// Publish a PointCloud2 message
    pub fn publish_pointcloud(&self, msg: PointCloud2) -> bool {
        if !self.enabled {
            return false;
        }
        if let Some(ref hub) = self.pointcloud_hub {
            if let Ok(hub_guard) = hub.lock() {
                return hub_guard.send(msg, &mut None).is_ok();
            }
        }
        false
    }

    /// Publish a LaserScan message
    pub fn publish_laserscan(&self, msg: LaserScanMessage) -> bool {
        if !self.enabled {
            return false;
        }
        if let Some(ref hub) = self.laserscan_hub {
            if let Ok(hub_guard) = hub.lock() {
                return hub_guard.send(msg, &mut None).is_ok();
            }
        }
        false
    }

    /// Publish an Odometry message
    pub fn publish_odom(&self, msg: Odometry) -> bool {
        if !self.enabled {
            return false;
        }
        if let Some(ref hub) = self.odom_hub {
            if let Ok(hub_guard) = hub.lock() {
                return hub_guard.send(msg, &mut None).is_ok();
            }
        }
        false
    }

    /// Publish a JointState message
    pub fn publish_joint_state(&self, msg: JointState) -> bool {
        if !self.enabled {
            return false;
        }
        if let Some(ref hub) = self.joint_state_hub {
            if let Ok(hub_guard) = hub.lock() {
                return hub_guard.send(msg, &mut None).is_ok();
            }
        }
        false
    }

    // ========================================================================
    // Subscription Methods
    // ========================================================================

    /// Receive a cmd_vel Twist message (non-blocking)
    pub fn recv_cmd_vel(&self) -> Option<Twist> {
        if !self.enabled {
            return None;
        }
        if let Some(ref hub) = self.cmd_vel_hub {
            if let Ok(hub_guard) = hub.lock() {
                return hub_guard.recv(&mut None);
            }
        }
        None
    }
}

// ============================================================================
// Bevy Plugin
// ============================================================================

/// Plugin for HORUS transport integration
#[derive(Default)]
pub struct HorusTransportPlugin {
    config: HorusTransportConfig,
}

impl HorusTransportPlugin {
    /// Create plugin with custom configuration
    pub fn with_config(config: HorusTransportConfig) -> Self {
        Self { config }
    }

    /// Create plugin with robot name
    pub fn with_robot_name(robot_name: impl Into<String>) -> Self {
        Self {
            config: HorusTransportConfig {
                robot_name: robot_name.into(),
                ..Default::default()
            },
        }
    }

    /// Create plugin with network endpoint
    pub fn with_network(robot_name: impl Into<String>, endpoint: impl Into<String>) -> Self {
        Self {
            config: HorusTransportConfig {
                enable_network: true,
                network_endpoint: Some(endpoint.into()),
                robot_name: robot_name.into(),
            },
        }
    }
}

impl Plugin for HorusTransportPlugin {
    fn build(&self, app: &mut App) {
        let transport = if self.config.enable_network {
            if let Some(ref endpoint) = self.config.network_endpoint {
                HorusTransport::with_network(&self.config.robot_name, endpoint)
            } else {
                HorusTransport::new(&self.config.robot_name)
            }
        } else {
            HorusTransport::new(&self.config.robot_name)
        };

        app.insert_resource(self.config.clone())
            .insert_resource(transport)
            .add_systems(Update, horus_transport_recv_system);
    }
}

/// System to receive cmd_vel and apply to robots
fn horus_transport_recv_system(
    transport: Res<HorusTransport>,
    mut query: Query<
        (&Name, &mut crate::physics::rigid_body::Velocity),
        With<crate::robot::robot::Robot>,
    >,
) {
    if !transport.is_enabled() {
        return;
    }

    // Receive cmd_vel and apply to matching robot
    if let Some(twist) = transport.recv_cmd_vel() {
        // Apply to first robot for now (could be extended for multi-robot)
        if let Some((_name, mut velocity)) = query.iter_mut().next() {
            velocity.linear = Vec3::new(twist.linear.x, twist.linear.y, twist.linear.z);
            velocity.angular = Vec3::new(twist.angular.x, twist.angular.y, twist.angular.z);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_twist_log_summary() {
        let twist = Twist {
            linear: Vector3Message {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3Message {
                x: 0.0,
                y: 0.0,
                z: 0.5,
            },
        };
        let summary = twist.log_summary();
        assert!(summary.contains("Twist"));
        assert!(summary.contains("1.00"));
        assert!(summary.contains("0.50"));
    }

    #[test]
    fn test_transport_config_default() {
        let config = HorusTransportConfig::default();
        assert!(!config.enable_network);
        assert!(config.network_endpoint.is_none());
        assert_eq!(config.robot_name, "sim3d_robot");
    }

    #[test]
    fn test_transport_creation() {
        let transport = HorusTransport::new("test_robot");
        assert!(transport.is_enabled());
        assert_eq!(transport.robot_name(), "test_robot");
    }

    #[test]
    fn test_transport_enable_disable() {
        let mut transport = HorusTransport::new("test_robot");
        assert!(transport.is_enabled());

        transport.disable();
        assert!(!transport.is_enabled());

        transport.enable();
        assert!(transport.is_enabled());
    }
}
