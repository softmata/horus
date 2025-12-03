pub mod core_integration;
pub mod horus_transport;
pub mod messages;
pub mod publisher;
pub mod subscriber;

use bevy::prelude::*;
use std::sync::{Arc, Mutex};

pub use messages::*;
pub use publisher::{
    publish_hframe_system, publish_lidar2d_system, publish_lidar3d_system, HorusPublisher,
};
pub use subscriber::{apply_cmd_vel_system, handle_robot_commands_system, HorusSubscriber};

/// Configuration for the HORUS bridge
#[derive(Resource, Clone)]
pub struct HorusBridgeConfig {
    /// Enable/disable the entire bridge
    pub enabled: bool,
    /// Enable HFrame publishing
    pub publish_hframe: bool,
    /// Enable sensor data publishing (lidar, cameras, etc.)
    pub publish_sensors: bool,
    /// Enable command velocity subscription
    pub subscribe_cmd_vel: bool,
    /// Enable odometry publishing
    pub publish_odometry: bool,
    /// Enable joint state publishing
    pub publish_joint_states: bool,
    /// Update rate in Hz
    pub update_rate: f32,
}

impl Default for HorusBridgeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            publish_hframe: true,
            publish_sensors: true,
            subscribe_cmd_vel: true,
            publish_odometry: true,
            publish_joint_states: true,
            update_rate: 30.0,
        }
    }
}

impl HorusBridgeConfig {
    /// Create a new bridge configuration with all features enabled
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a configuration with all features disabled
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Default::default()
        }
    }

    /// Enable all bridge features
    pub fn enable_all(&mut self) {
        self.enabled = true;
        self.publish_hframe = true;
        self.publish_sensors = true;
        self.subscribe_cmd_vel = true;
        self.publish_odometry = true;
        self.publish_joint_states = true;
    }

    /// Disable all bridge features
    pub fn disable_all(&mut self) {
        self.enabled = false;
        self.publish_hframe = false;
        self.publish_sensors = false;
        self.subscribe_cmd_vel = false;
        self.publish_odometry = false;
        self.publish_joint_states = false;
    }

    /// Check if bridge should be active
    pub fn is_active(&self) -> bool {
        self.enabled
    }

    /// Get time interval between updates (in seconds)
    pub fn update_interval(&self) -> f32 {
        1.0 / self.update_rate
    }
}

/// Bridge state tracking
#[derive(Resource, Default)]
pub struct HorusBridgeState {
    /// Last update time
    pub last_update: f64,
    /// Number of messages published
    pub messages_published: usize,
    /// Number of messages received
    pub messages_received: usize,
    /// Bridge active status
    pub is_running: bool,
}

impl HorusBridgeState {
    pub fn new() -> Self {
        Self {
            last_update: 0.0,
            messages_published: 0,
            messages_received: 0,
            is_running: true,
        }
    }

    /// Reset statistics
    pub fn reset(&mut self) {
        self.messages_published = 0;
        self.messages_received = 0;
    }

    /// Record a published message
    pub fn record_publish(&mut self) {
        self.messages_published += 1;
    }

    /// Record a received message
    pub fn record_receive(&mut self) {
        self.messages_received += 1;
    }

    /// Get publishing rate (messages per second)
    pub fn publish_rate(&self, current_time: f64) -> f32 {
        if current_time <= self.last_update {
            return 0.0;
        }
        self.messages_published as f32 / (current_time - self.last_update) as f32
    }
}

/// Bridge coordinator for managing communication
#[derive(Resource)]
pub struct HorusBridge {
    publisher: Arc<Mutex<HorusPublisher>>,
    subscriber: Arc<Mutex<HorusSubscriber>>,
}

impl Default for HorusBridge {
    fn default() -> Self {
        Self::new()
    }
}

impl HorusBridge {
    pub fn new() -> Self {
        Self {
            publisher: Arc::new(Mutex::new(HorusPublisher::new())),
            subscriber: Arc::new(Mutex::new(HorusSubscriber::new())),
        }
    }

    /// Get publisher reference
    pub fn publisher(&self) -> Arc<Mutex<HorusPublisher>> {
        self.publisher.clone()
    }

    /// Get subscriber reference
    pub fn subscriber(&self) -> Arc<Mutex<HorusSubscriber>> {
        self.subscriber.clone()
    }

    /// Enable the bridge
    pub fn enable(&self) {
        if let Ok(mut pub_lock) = self.publisher.lock() {
            pub_lock.enable();
        }
        if let Ok(mut sub_lock) = self.subscriber.lock() {
            sub_lock.enable();
        }
    }

    /// Disable the bridge
    pub fn disable(&self) {
        if let Ok(mut pub_lock) = self.publisher.lock() {
            pub_lock.disable();
        }
        if let Ok(mut sub_lock) = self.subscriber.lock() {
            sub_lock.disable();
        }
    }

    /// Check if bridge is enabled
    pub fn is_enabled(&self) -> bool {
        if let Ok(pub_lock) = self.publisher.lock() {
            return pub_lock.is_enabled();
        }
        false
    }

    /// Clear all pending messages
    pub fn clear(&self) {
        if let Ok(pub_lock) = self.publisher.lock() {
            pub_lock.clear();
        }
        if let Ok(sub_lock) = self.subscriber.lock() {
            sub_lock.clear();
        }
    }
}

/// Bridge utilities for working with HORUS messages
pub struct BridgeUtils;

impl BridgeUtils {
    /// Convert Bevy transform to TransformMessage
    pub fn transform_to_message(transform: &Transform) -> TransformMessage {
        TransformMessage::from_bevy_transform(transform)
    }

    /// Create a TransformStamped message
    pub fn create_transform_stamped(
        frame_id: impl Into<String>,
        child_frame_id: impl Into<String>,
        transform: &Transform,
        time: f64,
    ) -> TransformStamped {
        TransformStamped {
            header: Header::new(frame_id, time),
            child_frame_id: child_frame_id.into(),
            transform: Self::transform_to_message(transform),
        }
    }

    /// Convert Twist message to velocity components
    pub fn twist_to_velocities(twist: &Twist) -> (Vec3, Vec3) {
        let linear = Vec3::new(twist.linear.x, twist.linear.y, twist.linear.z);
        let angular = Vec3::new(twist.angular.x, twist.angular.y, twist.angular.z);
        (linear, angular)
    }

    /// Convert velocity components to Twist message
    pub fn velocities_to_twist(linear: Vec3, angular: Vec3) -> Twist {
        Twist {
            linear: Vector3Message {
                x: linear.x,
                y: linear.y,
                z: linear.z,
            },
            angular: Vector3Message {
                x: angular.x,
                y: angular.y,
                z: angular.z,
            },
        }
    }

    /// Create an odometry message
    pub fn create_odometry(
        frame_id: impl Into<String>,
        child_frame_id: impl Into<String>,
        transform: &Transform,
        linear_vel: Vec3,
        angular_vel: Vec3,
        time: f64,
    ) -> Odometry {
        Odometry {
            header: Header::new(frame_id, time),
            child_frame_id: child_frame_id.into(),
            pose: PoseWithCovariance {
                pose: Pose {
                    position: transform.translation.into(),
                    orientation: transform.rotation.into(),
                },
                covariance: vec![0.0; 36],
            },
            twist: TwistWithCovariance {
                twist: Self::velocities_to_twist(linear_vel, angular_vel),
                covariance: vec![0.0; 36],
            },
        }
    }

    /// Create a joint state message
    pub fn create_joint_state(
        names: Vec<String>,
        positions: Vec<f32>,
        velocities: Vec<f32>,
        efforts: Vec<f32>,
        time: f64,
    ) -> JointState {
        JointState {
            header: Header::new("", time),
            name: names,
            position: positions,
            velocity: velocities,
            effort: efforts,
        }
    }
}

/// Bridge monitoring system to track statistics
fn bridge_monitor_system(
    time: Res<Time>,
    config: Res<HorusBridgeConfig>,
    mut state: ResMut<HorusBridgeState>,
) {
    if !config.is_active() {
        state.is_running = false;
        return;
    }

    state.is_running = true;
    state.last_update = time.elapsed_secs_f64();
}

/// System to periodically clear old messages
fn bridge_cleanup_system(
    config: Res<HorusBridgeConfig>,
    _publisher: Res<HorusPublisher>,
    _subscriber: Res<HorusSubscriber>,
) {
    if !config.is_active() {}

    // Periodic cleanup can be triggered here
    // For now, we keep messages until explicitly cleared
}

/// Plugin to register HORUS bridge systems
#[derive(Default)]
pub struct HorusBridgePlugin {
    config: HorusBridgeConfig,
}

impl HorusBridgePlugin {
    /// Create plugin with default configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Create plugin with custom configuration
    pub fn with_config(config: HorusBridgeConfig) -> Self {
        Self { config }
    }

    /// Create plugin with bridge disabled
    pub fn disabled() -> Self {
        Self {
            config: HorusBridgeConfig::disabled(),
        }
    }
}

impl Plugin for HorusBridgePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.config.clone())
            .init_resource::<HorusPublisher>()
            .init_resource::<HorusSubscriber>()
            .init_resource::<HorusBridgeState>()
            .init_resource::<HorusBridge>();

        // Add publishing systems if enabled
        if self.config.publish_hframe {
            app.add_systems(Update, publish_hframe_system);
        }

        if self.config.publish_sensors {
            app.add_systems(Update, (publish_lidar3d_system, publish_lidar2d_system));
        }

        // Add subscription systems if enabled
        if self.config.subscribe_cmd_vel {
            app.add_systems(Update, (apply_cmd_vel_system, handle_robot_commands_system));
        }

        // Add monitoring and cleanup systems
        app.add_systems(Update, (bridge_monitor_system, bridge_cleanup_system));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bridge_config() {
        let mut config = HorusBridgeConfig::new();
        assert!(config.is_active());
        assert!(config.publish_hframe);

        config.disable_all();
        assert!(!config.is_active());
        assert!(!config.publish_hframe);

        config.enable_all();
        assert!(config.is_active());
    }

    #[test]
    fn test_bridge_state() {
        let mut state = HorusBridgeState::new();
        assert_eq!(state.messages_published, 0);

        state.record_publish();
        assert_eq!(state.messages_published, 1);

        state.reset();
        assert_eq!(state.messages_published, 0);
    }

    #[test]
    fn test_bridge_utils_transform() {
        let transform = Transform::from_translation(Vec3::new(1.0, 2.0, 3.0));
        let msg = BridgeUtils::transform_to_message(&transform);

        assert_eq!(msg.translation.x, 1.0);
        assert_eq!(msg.translation.y, 2.0);
        assert_eq!(msg.translation.z, 3.0);
    }

    #[test]
    fn test_bridge_utils_twist() {
        let linear = Vec3::new(1.0, 0.0, 0.0);
        let angular = Vec3::new(0.0, 0.0, 0.5);

        let twist = BridgeUtils::velocities_to_twist(linear, angular);
        assert_eq!(twist.linear.x, 1.0);
        assert_eq!(twist.angular.z, 0.5);

        let (lin, ang) = BridgeUtils::twist_to_velocities(&twist);
        assert_eq!(lin, linear);
        assert_eq!(ang, angular);
    }

    #[test]
    fn test_bridge_lifecycle() {
        let bridge = HorusBridge::new();
        assert!(bridge.is_enabled());

        bridge.disable();
        assert!(!bridge.is_enabled());

        bridge.enable();
        assert!(bridge.is_enabled());
    }

    #[test]
    fn test_bridge_config_update_interval() {
        let config = HorusBridgeConfig {
            update_rate: 10.0,
            ..Default::default()
        };

        assert_eq!(config.update_interval(), 0.1);
    }
}
