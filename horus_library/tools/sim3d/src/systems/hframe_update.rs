use bevy::prelude::*;
use horus_core::memory::ShmTopic;
use horus_library::hframe::{TFMessage, Transform as HFrameTransform, TransformStamped};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::hframe::HFrameTree;
use crate::physics::rigid_body::RigidBodyComponent;
use crate::physics::world::PhysicsWorld;
use crate::robot::state::{ArticulatedRobot, JointLink, RobotJointStates};

/// Component to mark an entity that should publish its transform to the HFrame tree
#[derive(Component, Clone)]
pub struct HFramePublisher {
    /// Frame name for this entity
    pub frame_name: String,
    /// Parent frame name
    pub parent_frame: String,
    /// Update rate in Hz (0 = every frame)
    pub rate_hz: f32,
    /// Last update time
    pub last_update: f32,
}

impl HFramePublisher {
    pub fn new(frame_name: impl Into<String>, parent_frame: impl Into<String>) -> Self {
        Self {
            frame_name: frame_name.into(),
            parent_frame: parent_frame.into(),
            rate_hz: 0.0,
            last_update: -f32::INFINITY, // Start with negative infinity so first update always succeeds
        }
    }

    pub fn with_rate(mut self, rate_hz: f32) -> Self {
        self.rate_hz = rate_hz;
        self
    }

    pub fn should_update(&self, current_time: f32) -> bool {
        if self.rate_hz <= 0.0 {
            return true;
        }
        current_time - self.last_update >= 1.0 / self.rate_hz
    }

    pub fn update_time(&mut self, current_time: f32) {
        self.last_update = current_time;
    }
}

/// System to update HFrame tree from Bevy transforms
pub fn hframe_update_system(
    time: Res<Time>,
    mut hframe_tree: ResMut<HFrameTree>,
    mut query: Query<(&mut HFramePublisher, &GlobalTransform)>,
) {
    let current_time = time.elapsed_secs();

    for (mut publisher, transform) in query.iter_mut() {
        if !publisher.should_update(current_time) {
            continue;
        }

        publisher.update_time(current_time);

        // Convert Bevy transform to nalgebra Isometry3
        let translation = transform.translation();
        let rotation = transform.to_scale_rotation_translation().1;

        let nalgebra_translation = Translation3::new(translation.x, translation.y, translation.z);
        let nalgebra_rotation = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            rotation.w, rotation.x, rotation.y, rotation.z,
        ));
        let isometry = Isometry3::from_parts(nalgebra_translation, nalgebra_rotation);

        // Update or add frame to HFrame tree
        if hframe_tree.has_frame(&publisher.frame_name) {
            hframe_tree
                .update_frame(&publisher.frame_name, isometry)
                .ok();
        } else {
            hframe_tree
                .add_frame(&publisher.frame_name, &publisher.parent_frame, isometry)
                .ok();
        }
    }
}

/// System to update HFrame tree from physics rigid bodies
pub fn hframe_update_from_physics_system(
    time: Res<Time>,
    physics_world: Res<PhysicsWorld>,
    mut hframe_tree: ResMut<HFrameTree>,
    query: Query<(&RigidBodyComponent, &HFramePublisher)>,
) {
    let current_time = time.elapsed_secs();

    for (rb_component, publisher) in query.iter() {
        if !publisher.should_update(current_time) {
            continue;
        }

        // Get rigid body position from physics world
        if let Some(rb) = physics_world.rigid_body_set.get(rb_component.handle) {
            let position = rb.position();

            // Convert to nalgebra Isometry3
            let isometry = Isometry3::from_parts(
                Translation3::new(
                    position.translation.x,
                    position.translation.y,
                    position.translation.z,
                ),
                UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    position.rotation.w,
                    position.rotation.i,
                    position.rotation.j,
                    position.rotation.k,
                )),
            );

            // Update HFrame tree
            if hframe_tree.has_frame(&publisher.frame_name) {
                hframe_tree
                    .update_frame(&publisher.frame_name, isometry)
                    .ok();
            } else {
                hframe_tree
                    .add_frame(&publisher.frame_name, &publisher.parent_frame, isometry)
                    .ok();
            }
        }
    }
}

/// System to update HFrame tree for articulated robots (joint frames)
pub fn hframe_update_robot_joints_system(
    mut hframe_tree: ResMut<HFrameTree>,
    robot_query: Query<(&ArticulatedRobot, &RobotJointStates)>,
    joint_query: Query<&JointLink>,
) {
    for (_robot, joint_states) in robot_query.iter() {
        for joint_link in joint_query.iter() {
            // Get joint state
            if let Some(joint_state) = joint_states.get_joint(&joint_link.joint_name) {
                // Create transform based on joint position
                // For revolute joints: rotation around an axis
                // For prismatic joints: translation along an axis
                let isometry = match joint_state.joint_type {
                    crate::physics::joints::JointType::Revolute => {
                        // Rotation around Z-axis (assuming that's the joint axis)
                        let rotation =
                            UnitQuaternion::from_euler_angles(0.0, 0.0, joint_state.position);
                        Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), rotation)
                    }
                    crate::physics::joints::JointType::Prismatic => {
                        // Translation along Z-axis (assuming that's the joint axis)
                        Isometry3::from_parts(
                            Translation3::new(0.0, 0.0, joint_state.position),
                            UnitQuaternion::identity(),
                        )
                    }
                    _ => Isometry3::identity(),
                };

                // Update HFrame tree with joint transform
                // Frame name would be the child link name
                let child_frame = format!("{}_frame", joint_link.joint_name);

                if hframe_tree.has_frame(&child_frame) {
                    hframe_tree.update_frame(&child_frame, isometry).ok();
                } else {
                    // Parent frame would be the parent link
                    let parent_frame = format!("link_{}", joint_link.parent_entity.index());
                    hframe_tree
                        .add_frame(&child_frame, &parent_frame, isometry)
                        .ok();
                }
            }
        }
    }
}

/// Resource to configure HFrame update behavior
#[derive(Resource, Clone)]
pub struct HFrameUpdateConfig {
    /// Enable HFrame updates
    pub enabled: bool,
    /// Update from physics rigid bodies
    pub update_from_physics: bool,
    /// Update robot joint frames
    pub update_robot_joints: bool,
    /// Default update rate (Hz), 0 = every frame
    pub default_rate: f32,
}

impl Default for HFrameUpdateConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            update_from_physics: true,
            update_robot_joints: true,
            default_rate: 100.0, // 100 Hz default for HFrame
        }
    }
}

impl HFrameUpdateConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_rate(mut self, rate: f32) -> Self {
        self.default_rate = rate;
        self
    }

    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..default()
        }
    }
}

/// Event fired when HFrame tree is updated
#[derive(Event)]
pub struct HFrameTreeUpdatedEvent {
    pub frame_count: usize,
    pub update_time: f32,
}

/// System to emit HFrame update events
pub fn emit_hframe_updated_event(
    time: Res<Time>,
    hframe_tree: Res<HFrameTree>,
    mut events: EventWriter<HFrameTreeUpdatedEvent>,
) {
    events.send(HFrameTreeUpdatedEvent {
        frame_count: hframe_tree.frame_count(),
        update_time: time.elapsed_secs(),
    });
}

/// Resource to track HFrame update statistics
#[derive(Resource, Default)]
pub struct HFrameUpdateStats {
    pub frame_count: usize,
    pub update_count: u64,
    pub last_update_time: f32,
    pub frames_updated: u64,
}

impl HFrameUpdateStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, frame_count: usize, frames_updated: u64, time: f32) {
        self.frame_count = frame_count;
        self.update_count += 1;
        self.frames_updated += frames_updated;
        self.last_update_time = time;
    }

    pub fn reset(&mut self) {
        self.update_count = 0;
        self.frames_updated = 0;
    }
}

/// System to track HFrame update statistics
pub fn track_hframe_stats_system(
    time: Res<Time>,
    hframe_tree: Res<HFrameTree>,
    mut stats: ResMut<HFrameUpdateStats>,
    query: Query<&HFramePublisher>,
) {
    let frames_updated = query.iter().count() as u64;
    stats.update(
        hframe_tree.frame_count(),
        frames_updated,
        time.elapsed_secs(),
    );
}

/// Resource to hold shared memory publishers for HFrame data
#[derive(Resource)]
pub struct HFrameShmPublisher {
    /// Topic for dynamic transforms (updated frequently)
    tf_topic: Mutex<Option<ShmTopic<TFMessage>>>,
    /// Topic for static transforms (rarely changes)
    tf_static_topic: Mutex<Option<ShmTopic<TFMessage>>>,
    /// Whether publishing is enabled
    pub enabled: bool,
    /// Last publish time for rate limiting
    last_publish: f32,
    /// Publish rate in Hz
    pub publish_rate: f32,
    /// Track which static frames have been published
    published_static_frames: Mutex<std::collections::HashSet<String>>,
}

impl Default for HFrameShmPublisher {
    fn default() -> Self {
        Self {
            tf_topic: Mutex::new(None),
            tf_static_topic: Mutex::new(None),
            enabled: true,
            last_publish: 0.0,
            publish_rate: 50.0, // 50 Hz default
            published_static_frames: Mutex::new(std::collections::HashSet::new()),
        }
    }
}

impl HFrameShmPublisher {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_rate(mut self, rate: f32) -> Self {
        self.publish_rate = rate;
        self
    }

    /// Initialize or get the tf topic
    fn get_or_create_tf_topic(
        &self,
    ) -> Option<std::sync::MutexGuard<'_, Option<ShmTopic<TFMessage>>>> {
        let mut guard = self.tf_topic.lock().ok()?;
        if guard.is_none() {
            match ShmTopic::<TFMessage>::new("tf", 16) {
                Ok(topic) => {
                    info!("Created HFrame shared memory topic 'tf'");
                    *guard = Some(topic);
                }
                Err(e) => {
                    warn!("Failed to create tf topic: {}", e);
                    return None;
                }
            }
        }
        Some(guard)
    }

    /// Initialize or get the tf_static topic
    fn get_or_create_tf_static_topic(
        &self,
    ) -> Option<std::sync::MutexGuard<'_, Option<ShmTopic<TFMessage>>>> {
        let mut guard = self.tf_static_topic.lock().ok()?;
        if guard.is_none() {
            match ShmTopic::<TFMessage>::new("tf_static", 16) {
                Ok(topic) => {
                    info!("Created HFrame shared memory topic 'tf_static'");
                    *guard = Some(topic);
                }
                Err(e) => {
                    warn!("Failed to create tf_static topic: {}", e);
                    return None;
                }
            }
        }
        Some(guard)
    }

    /// Publish transforms to shared memory
    pub fn publish(&self, publishers: &[(&HFramePublisher, &GlobalTransform)]) {
        if !self.enabled || publishers.is_empty() {
            return;
        }

        // Get current timestamp
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);

        // Separate dynamic and static transforms
        let mut dynamic_msg = TFMessage::new();
        let mut static_msg = TFMessage::new();
        let mut published_static = self
            .published_static_frames
            .lock()
            .unwrap_or_else(|e| e.into_inner());

        for (publisher, transform) in publishers {
            // Convert Bevy transform to HFrame Transform
            let translation = transform.translation();
            let rotation = transform.to_scale_rotation_translation().1;

            let hframe_transform = HFrameTransform::new(
                [
                    translation.x as f64,
                    translation.y as f64,
                    translation.z as f64,
                ],
                [
                    rotation.x as f64,
                    rotation.y as f64,
                    rotation.z as f64,
                    rotation.w as f64,
                ],
            );

            let tf_stamped = TransformStamped::new(
                &publisher.parent_frame,
                &publisher.frame_name,
                timestamp,
                hframe_transform,
            );

            // Check if this is a static frame (rate_hz == 0 means static in this context)
            // Or use a naming convention - frames starting with "static_" or containing "_static"
            let is_static = publisher.frame_name.contains("_static")
                || publisher.parent_frame.contains("_static")
                || publisher.rate_hz < 0.1; // Very low rate = effectively static

            if is_static && !published_static.contains(&publisher.frame_name) {
                if static_msg.add(tf_stamped) {
                    published_static.insert(publisher.frame_name.clone());
                }
            } else if !is_static {
                dynamic_msg.add(tf_stamped);
            }
        }

        // Publish dynamic transforms
        if !dynamic_msg.is_empty() {
            if let Some(mut guard) = self.get_or_create_tf_topic() {
                if let Some(ref mut topic) = *guard {
                    if let Err(e) = topic.push(dynamic_msg) {
                        warn!("Failed to publish tf: {:?}", e);
                    }
                }
            }
        }

        // Publish static transforms (only new ones)
        if !static_msg.is_empty() {
            if let Some(mut guard) = self.get_or_create_tf_static_topic() {
                if let Some(ref mut topic) = *guard {
                    if let Err(e) = topic.push(static_msg) {
                        warn!("Failed to publish tf_static: {:?}", e);
                    }
                }
            }
        }
    }

    fn should_publish(&self, current_time: f32) -> bool {
        if self.publish_rate <= 0.0 {
            return true;
        }
        current_time - self.last_publish >= 1.0 / self.publish_rate
    }
}

/// System to publish HFrame data to shared memory
pub fn hframe_shm_publish_system(
    time: Res<Time>,
    mut shm_publisher: ResMut<HFrameShmPublisher>,
    query: Query<(&HFramePublisher, &GlobalTransform)>,
) {
    let current_time = time.elapsed_secs();

    if !shm_publisher.should_publish(current_time) {
        return;
    }

    shm_publisher.last_publish = current_time;

    // Collect all publishers with their transforms
    let publishers: Vec<_> = query.iter().collect();
    shm_publisher.publish(&publishers);
}

/// Plugin to register all HFrame update systems
pub struct HFrameUpdatePlugin;

impl Plugin for HFrameUpdatePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HFrameUpdateConfig>()
            .init_resource::<HFrameUpdateStats>()
            .init_resource::<HFrameShmPublisher>()
            .add_event::<HFrameTreeUpdatedEvent>()
            .add_systems(
                Update,
                (
                    hframe_update_system,
                    hframe_update_from_physics_system,
                    hframe_update_robot_joints_system,
                    emit_hframe_updated_event,
                    track_hframe_stats_system,
                    hframe_shm_publish_system,
                )
                    .chain(),
            );
    }
}

/// Helper function to create a HFrame publisher for an entity
pub fn add_hframe_publisher(
    commands: &mut Commands,
    entity: Entity,
    frame_name: impl Into<String>,
    parent_frame: impl Into<String>,
) {
    commands
        .entity(entity)
        .insert(HFramePublisher::new(frame_name, parent_frame));
}

/// Helper function to create a HFrame publisher with custom rate
pub fn add_hframe_publisher_with_rate(
    commands: &mut Commands,
    entity: Entity,
    frame_name: impl Into<String>,
    parent_frame: impl Into<String>,
    rate_hz: f32,
) {
    commands
        .entity(entity)
        .insert(HFramePublisher::new(frame_name, parent_frame).with_rate(rate_hz));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hframe_publisher() {
        let mut publisher = HFramePublisher::new("test_frame", "world");
        assert_eq!(publisher.frame_name, "test_frame");
        assert_eq!(publisher.parent_frame, "world");
        assert_eq!(publisher.rate_hz, 0.0);

        publisher = publisher.with_rate(50.0);
        assert_eq!(publisher.rate_hz, 50.0);
    }

    #[test]
    fn test_hframe_publisher_rate_limiting() {
        let mut publisher = HFramePublisher::new("test", "world").with_rate(10.0);

        assert!(publisher.should_update(0.0));
        publisher.update_time(0.0);

        assert!(!publisher.should_update(0.05)); // 50ms < 100ms
        assert!(publisher.should_update(0.11)); // 110ms > 100ms
    }

    #[test]
    fn test_hframe_update_config() {
        let config = HFrameUpdateConfig::new().with_rate(200.0);
        assert_eq!(config.default_rate, 200.0);
        assert!(config.enabled);

        let disabled = HFrameUpdateConfig::disabled();
        assert!(!disabled.enabled);
    }

    #[test]
    fn test_hframe_update_stats() {
        let mut stats = HFrameUpdateStats::new();
        assert_eq!(stats.update_count, 0);

        stats.update(10, 5, 1.0);
        assert_eq!(stats.frame_count, 10);
        assert_eq!(stats.update_count, 1);
        assert_eq!(stats.frames_updated, 5);

        stats.reset();
        assert_eq!(stats.update_count, 0);
        assert_eq!(stats.frames_updated, 0);
    }
}
