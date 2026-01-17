//! # sim2d - Simple 2D Robotics Simulator
//!
//! One command, physics + visualization, simple control.
//!
//! Usage:
//!   sim2d                                    # Default robot + world
//!   sim2d --robot my_robot.yaml              # Custom robot
//!   sim2d --world my_world.yaml              # Custom world
//!   sim2d --topic robot.cmd_vel              # Custom control topic
//!
//! Control from another terminal:
//!   cargo run -p simple_driver

// Bevy systems commonly have many arguments and complex query types
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]

// ui module is declared at the lib.rs level
use crate::ui;

use crate::joint::{JointCommand, JointState};
use anyhow::Result;
use bevy::prelude::*;
use clap::Parser;
use horus_core::core::LogSummary;
use horus_core::{communication::Topic, core::NodeInfo};
use horus_library::messages::{CmdVel, Imu, LaserScan, Odometry, Pose2D, Twist};
use rapier2d::prelude::*;
use serde::{Deserialize, Serialize};
use tracing::{info, warn};

/// CLI arguments
#[derive(Parser, Clone)]
#[command(name = "sim2d")]
#[command(about = "Simple 2D robotics simulator with physics")]
pub struct Args {
    /// Robot configuration file (YAML)
    #[arg(long)]
    pub robot: Option<String>,

    /// World configuration file (YAML)
    #[arg(long)]
    pub world: Option<String>,

    /// World image file (PNG, JPG, PGM) - occupancy grid
    #[arg(long)]
    pub world_image: Option<String>,

    /// Resolution in meters per pixel (for world image)
    #[arg(long, default_value = "0.05")]
    pub resolution: f32,

    /// Obstacle threshold (0-255, pixels darker than this are obstacles)
    #[arg(long, default_value = "128")]
    pub threshold: u8,

    /// Topic prefix for robot topics (e.g. robot -> robot.cmd_vel, robot.odom)
    #[arg(long, default_value = "robot")]
    pub topic: String,

    /// Robot name
    #[arg(long, default_value = "robot")]
    pub name: String,

    /// Run in headless mode (no GUI)
    #[arg(long)]
    pub headless: bool,

    /// Articulated robot configuration file (YAML) for multi-link robots
    #[arg(long)]
    pub articulated: Option<String>,

    /// Use a preset articulated robot (arm_2dof, arm_6dof, humanoid)
    #[arg(long)]
    pub preset: Option<String>,

    /// Enable gravity (for side-view humanoid simulation)
    #[arg(long)]
    pub gravity: bool,
}

/// Robot configuration
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct RobotConfig {
    #[serde(default = "default_robot_name")]
    pub name: String,
    #[serde(default = "default_robot_topic_prefix")]
    pub topic_prefix: String, // Topic prefix (e.g., "robot0", "robot1")
    #[serde(default = "default_robot_position")]
    pub position: [f32; 2], // Initial position [x, y]
    pub width: f32,
    pub length: f32,
    pub max_speed: f32,
    pub color: [f32; 3], // RGB
    #[serde(default)]
    pub visual: VisualComponents,
    #[serde(default)]
    pub kinematics: crate::kinematics::KinematicsModel,
    #[serde(default)]
    pub lidar: LidarConfig,
    #[serde(default)]
    pub camera: crate::camera::CameraConfig,
    #[serde(default)]
    pub gps: crate::sensors::GpsConfig,
    #[serde(default)]
    pub ultrasonic: crate::sensors::UltrasonicConfig,
    #[serde(default)]
    pub contact: crate::sensors::ContactConfig,
}

fn default_robot_name() -> String {
    "robot".to_string()
}

fn default_robot_topic_prefix() -> String {
    "robot".to_string()
}

fn default_robot_position() -> [f32; 2] {
    [0.0, 0.0]
}

/// LIDAR sensor configuration
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct LidarConfig {
    #[serde(default = "default_lidar_enabled")]
    pub enabled: bool,
    #[serde(default = "default_lidar_range_max")]
    pub range_max: f32,
    #[serde(default = "default_lidar_range_min")]
    pub range_min: f32,
    #[serde(default = "default_lidar_angle_min")]
    pub angle_min: f32, // radians
    #[serde(default = "default_lidar_angle_max")]
    pub angle_max: f32, // radians
    #[serde(default = "default_lidar_num_rays")]
    pub num_rays: usize,
}

/// Default: LIDAR enabled
pub fn default_lidar_enabled() -> bool {
    true
}

/// Default: 10 meters max range
pub fn default_lidar_range_max() -> f32 {
    10.0
}

/// Default: 10 cm min range
pub fn default_lidar_range_min() -> f32 {
    0.1
}

/// Default: -180 degrees (full 360 scan)
pub fn default_lidar_angle_min() -> f32 {
    -std::f32::consts::PI
}

/// Default: +180 degrees (full 360 scan)
pub fn default_lidar_angle_max() -> f32 {
    std::f32::consts::PI
}

/// Default: 1 degree resolution (360 rays)
pub fn default_lidar_num_rays() -> usize {
    360
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            enabled: default_lidar_enabled(),
            range_max: default_lidar_range_max(),
            range_min: default_lidar_range_min(),
            angle_min: default_lidar_angle_min(),
            angle_max: default_lidar_angle_max(),
            num_rays: default_lidar_num_rays(),
        }
    }
}

/// Optional visual components for robot appearance
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize, Default)]
pub struct VisualComponents {
    /// Turret component (sits on top of hull)
    pub turret: Option<TurretConfig>,
    /// Cannon component (extends from turret)
    pub cannon: Option<CannonConfig>,
    /// Tread components (visual only, for tank-like appearance)
    pub treads: Option<TreadConfig>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct TurretConfig {
    pub width: f32,
    pub length: f32,
    pub offset_x: f32, // Offset from robot center
    pub offset_y: f32,
    #[serde(default = "default_turret_color")]
    pub color: [f32; 3],
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct CannonConfig {
    pub length: f32,
    pub width: f32,
    pub offset_x: f32, // Offset from robot center (typically forward)
    #[serde(default = "default_cannon_color")]
    pub color: [f32; 3],
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct TreadConfig {
    pub width: f32,  // Width of each tread
    pub offset: f32, // Distance from center to each tread
    #[serde(default = "default_tread_color")]
    pub color: [f32; 3],
}

fn default_turret_color() -> [f32; 3] {
    [0.25, 0.35, 0.18] // Darker olive
}

fn default_cannon_color() -> [f32; 3] {
    [0.2, 0.2, 0.2] // Dark gray
}

fn default_tread_color() -> [f32; 3] {
    [0.15, 0.15, 0.15] // Very dark gray
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            name: "robot".to_string(),
            topic_prefix: "robot".to_string(),
            position: [0.0, 0.0],
            width: 0.5,             // 0.5m wide
            length: 0.8,            // 0.8m long
            max_speed: 2.0,         // 2 m/s max
            color: [0.2, 0.8, 0.2], // Green
            visual: VisualComponents::default(),
            kinematics: crate::kinematics::KinematicsModel::default(),
            lidar: LidarConfig::default(),
            camera: crate::camera::CameraConfig::default(),
            gps: crate::sensors::GpsConfig::default(),
            ultrasonic: crate::sensors::UltrasonicConfig::default(),
            contact: crate::sensors::ContactConfig::default(),
        }
    }
}

/// World configuration
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct WorldConfig {
    pub width: f32,
    pub height: f32,
    pub obstacles: Vec<Obstacle>,
    /// Color for boundary walls (RGB 0.0-1.0), defaults to gray
    #[serde(default = "default_wall_color")]
    pub wall_color: [f32; 3],
    /// Default color for obstacles without explicit color (RGB 0.0-1.0), defaults to brown
    #[serde(default = "default_obstacle_color")]
    pub default_obstacle_color: [f32; 3],
}

/// Default wall color (gray)
pub fn default_wall_color() -> [f32; 3] {
    [0.3, 0.3, 0.3] // Gray
}

/// Default obstacle color (brown)
pub fn default_obstacle_color() -> [f32; 3] {
    [0.6, 0.4, 0.2] // Brown
}

/// Obstacle shape type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum ObstacleShape {
    #[default]
    Rectangle,
    Circle,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obstacle {
    pub pos: [f32; 2],
    #[serde(default)]
    pub shape: ObstacleShape,
    /// For rectangles: [width, height], for circles: [radius, radius]
    pub size: [f32; 2],
    #[serde(default)]
    pub color: Option<[f32; 3]>, // RGB color (0.0-1.0)
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            width: 20.0,
            height: 15.0,
            obstacles: vec![
                Obstacle {
                    pos: [5.0, 5.0],
                    shape: ObstacleShape::Rectangle,
                    size: [2.0, 1.0],
                    color: None,
                },
                Obstacle {
                    pos: [-3.0, -2.0],
                    shape: ObstacleShape::Rectangle,
                    size: [1.5, 1.5],
                    color: None,
                },
                Obstacle {
                    pos: [0.0, 7.0],
                    shape: ObstacleShape::Rectangle,
                    size: [3.0, 0.5],
                    color: None,
                },
            ],
            wall_color: default_wall_color(),
            default_obstacle_color: default_obstacle_color(),
        }
    }
}

/// Robot entity in Bevy
#[derive(Component)]
pub struct Robot {
    pub name: String,
    pub config: RobotConfig,
    pub rigid_body_handle: RigidBodyHandle,
}

/// World boundaries and obstacles
#[derive(Component)]
pub struct WorldElement;

/// Obstacle marker
#[derive(Component)]
pub struct ObstacleElement;

/// Grid lines
#[derive(Component)]
pub struct GridLine;

/// Velocity arrow marker
#[derive(Component)]
pub struct VelocityArrow;

/// LIDAR ray marker
#[derive(Component)]
pub struct LidarRay;

/// Trajectory trail marker
#[derive(Component)]
pub struct TrajectoryPoint;

/// Trajectory trail history
#[derive(Resource, Default)]
pub struct TrajectoryHistory {
    points: Vec<(f32, f32)>, // (x, y) positions
}

/// Last LIDAR scan for visualization
#[derive(Resource, Default)]
pub struct LastLidarScan {
    pub ranges: Vec<f32>,
    pub angles: Vec<f32>,
    pub robot_pos: (f32, f32),
    pub robot_angle: f32,
}

/// Camera sensors resource - stores camera instances per robot
#[derive(Resource, Default)]
pub struct CameraSensors {
    pub sensors: std::collections::HashMap<String, crate::camera::CameraSensor>,
    pub last_images: std::collections::HashMap<String, crate::camera::GrayscaleImage>,
}

/// GPS sensor state for all robots
#[derive(Resource, Default)]
pub struct GpsSensors {
    pub sensors: std::collections::HashMap<String, crate::sensors::GpsSensor>,
    pub last_readings: std::collections::HashMap<String, crate::sensors::GpsData>,
}

/// Ultrasonic sensor state for all robots
#[derive(Resource, Default)]
pub struct UltrasonicSensors {
    pub last_readings: std::collections::HashMap<String, Vec<f32>>,
}

/// Contact sensor state for all robots
#[derive(Resource, Default)]
pub struct ContactSensors {
    pub last_readings: std::collections::HashMap<String, crate::sensors::ContactData>,
}

/// Previous velocity for IMU acceleration calculation
#[derive(Resource)]
pub struct PreviousVelocity {
    linear: (f32, f32),
    angular: f32,
    timestamp: std::time::Instant,
}

impl Default for PreviousVelocity {
    fn default() -> Self {
        Self {
            linear: (0.0, 0.0),
            angular: 0.0,
            timestamp: std::time::Instant::now(),
        }
    }
}

/// Collision tracking resource
#[derive(Resource, Default)]
pub struct CollisionState {
    is_colliding: bool,
    collision_count: usize,
    last_collision_time: Option<std::time::Instant>,
}

impl CollisionState {
    /// Create a new CollisionState with default values
    pub fn new() -> Self {
        Self {
            is_colliding: false,
            collision_count: 0,
            last_collision_time: None,
        }
    }
}

/// Collision marker component
#[derive(Component)]
pub struct CollisionIndicator;

/// Obstacle command message for dynamic spawning/removal
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObstacleCommand {
    pub action: ObstacleAction,
    pub obstacle: Obstacle,
}

impl LogSummary for ObstacleCommand {
    fn log_summary(&self) -> String {
        format!(
            "ObstacleCmd({:?}, pos=[{:.1}, {:.1}])",
            self.action, self.obstacle.pos[0], self.obstacle.pos[1]
        )
    }
}

/// Action to perform on an obstacle
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ObstacleAction {
    Add,
    Remove,
}

/// Unique ID for dynamically spawned obstacles
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DynamicObstacleId(u64);

/// Counter for generating unique obstacle IDs
#[derive(Resource, Default)]
pub struct ObstacleIdCounter(u64);

/// Component to track physics rigid body handle for world elements
#[derive(Component)]
pub struct PhysicsHandle {
    rigid_body_handle: RigidBodyHandle,
}

/// Visual component markers - these follow the parent robot
#[derive(Component)]
pub struct RobotTurret {
    parent: Entity,
}

#[derive(Component)]
pub struct RobotCannon {
    parent: Entity,
}

#[derive(Component)]
pub struct RobotTread {
    parent: Entity,
    is_left: bool, // true for left tread, false for right
}

/// Per-robot HORUS communication hubs
pub struct RobotHubs {
    cmd_vel_sub: Topic<CmdVel>,
    odom_pub: Topic<Odometry>,
    lidar_pub: Topic<LaserScan>,
    imu_pub: Topic<Imu>,
}

/// Per-articulated-robot HORUS communication hubs
pub struct ArticulatedRobotHubs {
    pub joint_cmd_sub: Topic<JointCommand>,
    pub joint_state_pub: Topic<JointState>,
}

/// HORUS communication system
#[derive(Resource)]
pub struct HorusComm {
    robot_hubs: std::collections::HashMap<String, RobotHubs>, // Per-robot hubs indexed by robot name
    pub articulated_robot_hubs: std::collections::HashMap<String, ArticulatedRobotHubs>, // Per-articulated robot hubs
    obstacle_cmd_sub: Topic<ObstacleCommand>, // Shared obstacle command topic
    node_info: NodeInfo,
    /// Current topic prefixes per robot (for detecting changes)
    current_topic_prefixes: std::collections::HashMap<String, String>,
}

impl HorusComm {
    /// Recreate hubs for a robot with a new topic prefix
    pub fn update_robot_topics(&mut self, robot_name: &str, new_prefix: &str) -> bool {
        let cmd_vel_topic = format!("{}.cmd_vel", new_prefix);
        let odom_topic = format!("{}.odom", new_prefix);
        let scan_topic = format!("{}.scan", new_prefix);
        let imu_topic = format!("{}.imu", new_prefix);

        match (
            Topic::new(&cmd_vel_topic),
            Topic::new(&odom_topic),
            Topic::new(&scan_topic),
            Topic::new(&imu_topic),
        ) {
            (Ok(cmd_vel_sub), Ok(odom_pub), Ok(lidar_pub), Ok(imu_pub)) => {
                // Replace the old hubs with new ones
                self.robot_hubs.insert(
                    robot_name.to_string(),
                    RobotHubs {
                        cmd_vel_sub,
                        odom_pub,
                        lidar_pub,
                        imu_pub,
                    },
                );
                // Track the new prefix
                self.current_topic_prefixes
                    .insert(robot_name.to_string(), new_prefix.to_string());
                info!(
                    "Updated HORUS topics for robot '{}' to prefix '{}'",
                    robot_name, new_prefix
                );
                true
            }
            _ => {
                warn!("Failed to update HORUS topics for robot '{}'", robot_name);
                false
            }
        }
    }

    /// Get the current topic prefix for a robot
    pub fn get_topic_prefix(&self, robot_name: &str) -> Option<&String> {
        self.current_topic_prefixes.get(robot_name)
    }
}

/// Physics world
#[derive(Resource)]
pub struct PhysicsWorld {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub gravity: Vector<f32>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub physics_hooks: (),
    pub event_handler: (),
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity: vector![0.0, 0.0], // No gravity for top-down view
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: (),
            event_handler: (),
        }
    }
}

/// App configuration
#[derive(Resource)]
pub struct AppConfig {
    pub args: Args,
    pub robots: Vec<RobotConfig>, // Support multiple robots
    pub world_config: WorldConfig,
    pub articulated_robots: Vec<crate::joint::ArticulatedRobotConfig>, // Articulated robots
}

impl AppConfig {
    fn new(args: Args) -> Self {
        // Load robot config(s)
        let robots = if let Some(robot_file) = &args.robot {
            Self::load_robots_config(robot_file).unwrap_or_else(|_| vec![RobotConfig::default()])
        } else {
            // Create single default robot using CLI args
            let robot = RobotConfig {
                name: args.name.clone(),
                topic_prefix: args
                    .topic
                    .strip_suffix(".cmd_vel")
                    .unwrap_or(&args.topic)
                    .to_string(),
                ..Default::default()
            };
            vec![robot]
        };

        // Load world config - prioritize image over config file
        let world_config = if let Some(image_file) = &args.world_image {
            // Load from image
            match Self::load_world_from_image(image_file, args.resolution, args.threshold) {
                Ok(config) => config,
                Err(e) => {
                    warn!("Failed to load world from image: {}", e);
                    warn!("Falling back to default world");
                    WorldConfig::default()
                }
            }
        } else if let Some(world_file) = &args.world {
            // Load from config file
            Self::load_world_config(world_file).unwrap_or_default()
        } else {
            // Use default
            WorldConfig::default()
        };

        // Log robot info
        if robots.len() == 1 {
            info!(
                " Robot: {:.1}m x {:.1}m, max speed: {:.1} m/s",
                robots[0].length, robots[0].width, robots[0].max_speed
            );
            info!(" Control topic: {}.cmd_vel", robots[0].topic_prefix);
        } else {
            info!(" {} robots configured", robots.len());
            for robot in &robots {
                info!(
                    "   - {}: {:.1}m x {:.1}m at {:?}",
                    robot.name, robot.length, robot.width, robot.position
                );
            }
        }

        info!(
            "World: {:.1}m x {:.1}m with {} obstacles",
            world_config.width,
            world_config.height,
            world_config.obstacles.len()
        );

        // Load articulated robot configs
        let articulated_robots = Self::load_articulated_robots(&args);
        if !articulated_robots.is_empty() {
            info!(
                " {} articulated robot(s) configured",
                articulated_robots.len()
            );
            for ar in &articulated_robots {
                info!(
                    "   - {}: {} links, {} joints",
                    ar.name,
                    ar.links.len(),
                    ar.joints.len()
                );
            }
        }

        Self {
            args,
            robots,
            world_config,
            articulated_robots,
        }
    }

    /// Load articulated robots from CLI args
    fn load_articulated_robots(args: &Args) -> Vec<crate::joint::ArticulatedRobotConfig> {
        let mut configs = Vec::new();

        // Load from preset
        if let Some(preset_name) = &args.preset {
            let mut config = match preset_name.as_str() {
                "arm_2dof" | "arm2" => crate::joint::preset_arm_2dof(),
                "arm_6dof" | "arm6" => crate::joint::preset_arm_6dof(),
                "humanoid" | "human" => crate::joint::preset_humanoid_simple(),
                _ => {
                    warn!(
                        "Unknown preset '{}', available: arm_2dof, arm_6dof, humanoid",
                        preset_name
                    );
                    return configs;
                }
            };
            // Apply gravity flag
            if args.gravity {
                config.enable_gravity = true;
            }
            configs.push(config);
        }

        // Load from file
        if let Some(file_path) = &args.articulated {
            match crate::joint::ArticulatedRobotConfig::from_file(file_path) {
                Ok(mut config) => {
                    if args.gravity {
                        config.enable_gravity = true;
                    }
                    configs.push(config);
                }
                Err(e) => {
                    warn!(
                        "Failed to load articulated robot from '{}': {}",
                        file_path, e
                    );
                }
            }
        }

        configs
    }

    pub fn load_robots_config(path: &str) -> Result<Vec<RobotConfig>> {
        let content = std::fs::read_to_string(path)?;

        // Try to parse as multi-robot config first
        #[derive(serde::Deserialize)]
        struct MultiRobotConfig {
            robots: Vec<RobotConfig>,
        }

        // Auto-detect format from file extension
        if path.ends_with(".toml") {
            // Try multi-robot format first
            if let Ok(config) = toml::from_str::<MultiRobotConfig>(&content) {
                Ok(config.robots)
            } else {
                // Fall back to single robot
                Ok(vec![toml::from_str(&content)?])
            }
        } else {
            // YAML format
            if let Ok(config) = serde_yaml::from_str::<MultiRobotConfig>(&content) {
                Ok(config.robots)
            } else {
                // Fall back to single robot
                Ok(vec![serde_yaml::from_str(&content)?])
            }
        }
    }

    pub fn load_robot_config(path: &str) -> Result<RobotConfig> {
        let content = std::fs::read_to_string(path)?;

        // Auto-detect format from file extension
        if path.ends_with(".toml") {
            Ok(toml::from_str(&content)?)
        } else {
            // Default to YAML for .yaml, .yml, or no extension
            Ok(serde_yaml::from_str(&content)?)
        }
    }

    pub fn load_world_config(path: &str) -> Result<WorldConfig> {
        let content = std::fs::read_to_string(path)?;

        // Auto-detect format from file extension
        if path.ends_with(".toml") {
            Ok(toml::from_str(&content)?)
        } else {
            // Default to YAML for .yaml, .yml, or no extension
            Ok(serde_yaml::from_str(&content)?)
        }
    }

    pub fn load_world_from_image(
        image_path: &str,
        resolution: f32,
        threshold: u8,
    ) -> Result<WorldConfig> {
        use image::GenericImageView;

        info!("Loading world from image: {}", image_path);
        info!("   Resolution: {} m/pixel", resolution);
        info!("   Threshold: {} (darker = obstacle)", threshold);

        // Load image
        let img = image::open(image_path)?;
        let (width_px, height_px) = img.dimensions();

        info!("   Image size: {}x{} pixels", width_px, height_px);

        // Convert to grayscale
        let gray_img = img.to_luma8();

        // Calculate world dimensions in meters
        let world_width = width_px as f32 * resolution;
        let world_height = height_px as f32 * resolution;

        info!("   World size: {:.2}m x {:.2}m", world_width, world_height);

        // Convert pixels to obstacles
        // Use a grid-based approach: group adjacent obstacle pixels into rectangles
        let mut obstacles = Vec::new();

        // Simple approach: create small square obstacles for each occupied pixel
        // This can be optimized later to merge adjacent pixels
        for y in 0..height_px {
            for x in 0..width_px {
                let pixel = gray_img.get_pixel(x, y)[0];

                // If pixel is darker than threshold, it's an obstacle
                if pixel < threshold {
                    // Convert pixel coordinates to world coordinates
                    // Image origin (0,0) is top-left, world origin is center
                    let world_x = (x as f32 - width_px as f32 / 2.0) * resolution;
                    let world_y = -(y as f32 - height_px as f32 / 2.0) * resolution; // Flip Y

                    obstacles.push(Obstacle {
                        pos: [world_x, world_y],
                        shape: ObstacleShape::Rectangle,
                        size: [resolution, resolution], // Square obstacle per pixel
                        color: None,
                    });
                }
            }
        }

        info!("   Generated {} obstacle cells", obstacles.len());

        Ok(WorldConfig {
            width: world_width,
            height: world_height,
            obstacles,
            wall_color: default_wall_color(),
            default_obstacle_color: default_obstacle_color(),
        })
    }
}

/// Setup system - initializes everything
/// Helper function to spawn visual components for a robot
fn spawn_robot_visual_components(
    commands: &mut Commands,
    parent_entity: Entity,
    config: &RobotConfig,
    scale: f32,
) {
    info!(" Spawning visual components for robot");

    // Spawn treads if configured
    if let Some(ref tread_config) = config.visual.treads {
        info!("   Spawning treads");

        let tread_color = Color::srgb(
            tread_config.color[0],
            tread_config.color[1],
            tread_config.color[2],
        );

        // Left tread
        commands.spawn((
            Sprite {
                color: tread_color,
                custom_size: Some(Vec2::new(config.length * scale, tread_config.width * scale)),
                ..default()
            },
            Transform::from_translation(Vec3::new(0.0, 0.0, 0.9)), // Slightly below robot
            RobotTread {
                parent: parent_entity,
                is_left: true,
            },
        ));

        // Right tread
        commands.spawn((
            Sprite {
                color: tread_color,
                custom_size: Some(Vec2::new(config.length * scale, tread_config.width * scale)),
                ..default()
            },
            Transform::from_translation(Vec3::new(0.0, 0.0, 0.9)), // Slightly below robot
            RobotTread {
                parent: parent_entity,
                is_left: false,
            },
        ));
    }

    // Spawn turret if configured
    if let Some(ref turret_config) = config.visual.turret {
        info!("   Spawning turret");
        let turret_color = Color::srgb(
            turret_config.color[0],
            turret_config.color[1],
            turret_config.color[2],
        );

        commands.spawn((
            Sprite {
                color: turret_color,
                custom_size: Some(Vec2::new(
                    turret_config.length * scale,
                    turret_config.width * scale,
                )),
                ..default()
            },
            Transform::from_translation(Vec3::new(0.0, 0.0, 1.1)), // Above robot
            RobotTurret {
                parent: parent_entity,
            },
        ));
    }

    // Spawn cannon if configured
    if let Some(ref cannon_config) = config.visual.cannon {
        info!("   Spawning cannon");
        let cannon_color = Color::srgb(
            cannon_config.color[0],
            cannon_config.color[1],
            cannon_config.color[2],
        );

        commands.spawn((
            Sprite {
                color: cannon_color,
                custom_size: Some(Vec2::new(
                    cannon_config.length * scale,
                    cannon_config.width * scale,
                )),
                ..default()
            },
            Transform::from_translation(Vec3::new(0.0, 0.0, 1.2)), // Above turret
            RobotCannon {
                parent: parent_entity,
            },
        ));
    }
}

pub fn setup(
    mut commands: Commands,
    app_config: Res<AppConfig>,
    mut physics_world: ResMut<PhysicsWorld>,
) {
    info!(" Setting up sim2d");

    // Setup camera with better positioning
    commands.spawn((
        Camera2d,
        Transform::from_translation(Vec3::new(0.0, 0.0, 100.0)),
    ));

    // Create HORUS communication - per-robot hubs
    let mut robot_hubs = std::collections::HashMap::new();
    let mut current_topic_prefixes = std::collections::HashMap::new();
    let mut horus_connected = true;

    for robot_config in &app_config.robots {
        let cmd_vel_topic = format!("{}.cmd_vel", robot_config.topic_prefix);
        let odom_topic = format!("{}.odom", robot_config.topic_prefix);
        let scan_topic = format!("{}.scan", robot_config.topic_prefix);
        let imu_topic = format!("{}.imu", robot_config.topic_prefix);

        match (
            Topic::new(&cmd_vel_topic),
            Topic::new(&odom_topic),
            Topic::new(&scan_topic),
            Topic::new(&imu_topic),
        ) {
            (Ok(cmd_vel_sub), Ok(odom_pub), Ok(lidar_pub), Ok(imu_pub)) => {
                robot_hubs.insert(
                    robot_config.name.clone(),
                    RobotHubs {
                        cmd_vel_sub,
                        odom_pub,
                        lidar_pub,
                        imu_pub,
                    },
                );
                // Track the topic prefix for this robot
                current_topic_prefixes
                    .insert(robot_config.name.clone(), robot_config.topic_prefix.clone());
                info!(" Connected HORUS for robot '{}':", robot_config.name);
                info!("    cmd_vel: {}", cmd_vel_topic);
                info!("    odom: {}", odom_topic);
                info!("    scan: {}", scan_topic);
                info!("    imu: {}", imu_topic);
            }
            _ => {
                warn!(" Failed to connect HORUS for robot '{}'", robot_config.name);
                horus_connected = false;
            }
        }
    }

    // Check if we have any robot connections before moving robot_hubs
    let has_robot_hubs = !robot_hubs.is_empty();

    // Create articulated robot hubs
    let mut articulated_robot_hubs = std::collections::HashMap::new();
    for artic_config in &app_config.articulated_robots {
        let joint_cmd_topic = format!("{}.joint_cmd", artic_config.topic_prefix);
        let joint_state_topic = format!("{}.joint_state", artic_config.topic_prefix);

        match (Topic::new(&joint_cmd_topic), Topic::new(&joint_state_topic)) {
            (Ok(joint_cmd_sub), Ok(joint_state_pub)) => {
                articulated_robot_hubs.insert(
                    artic_config.name.clone(),
                    ArticulatedRobotHubs {
                        joint_cmd_sub,
                        joint_state_pub,
                    },
                );
                info!(
                    " Connected HORUS for articulated robot '{}':",
                    artic_config.name
                );
                info!("    joint_cmd: {}", joint_cmd_topic);
                info!("    joint_state: {}", joint_state_topic);
            }
            _ => {
                warn!(
                    " Failed to connect HORUS for articulated robot '{}'",
                    artic_config.name
                );
            }
        }
    }

    // Create shared obstacle command hub
    match Topic::new("sim2d.obstacle_cmd") {
        Ok(obstacle_cmd_sub) => {
            let node_info = NodeInfo::new("sim2d".to_string(), true);
            commands.insert_resource(HorusComm {
                robot_hubs,
                articulated_robot_hubs,
                obstacle_cmd_sub,
                node_info,
                current_topic_prefixes,
            });
            info!(" Connected to obstacle command topic: sim2d.obstacle_cmd");
        }
        _ if !has_robot_hubs => {
            warn!(" Failed to connect to HORUS");
            warn!("   Robots will not respond to external commands or publish sensor data");
        }
        _ => {
            warn!(" Failed to connect to obstacle command topic, but robot topics are OK");
        }
    }

    if !horus_connected && !has_robot_hubs {
        warn!(" No HORUS connections established");
    }

    // Create world boundaries (scale up by 50 for visibility in pixels)
    let scale = 50.0;
    let world_half_width = app_config.world_config.width / 2.0 * scale;
    let world_half_height = app_config.world_config.height / 2.0 * scale;

    let boundaries = [
        // Bottom, Top, Left, Right walls
        (
            vector![0.0, -world_half_height],
            vector![app_config.world_config.width * scale, 0.2 * scale],
        ),
        (
            vector![0.0, world_half_height],
            vector![app_config.world_config.width * scale, 0.2 * scale],
        ),
        (
            vector![-world_half_width, 0.0],
            vector![0.2 * scale, app_config.world_config.height * scale],
        ),
        (
            vector![world_half_width, 0.0],
            vector![0.2 * scale, app_config.world_config.height * scale],
        ),
    ];

    let boundaries_physics = [
        // Bottom, Top, Left, Right walls (original scale for physics)
        (
            vector![0.0, -app_config.world_config.height / 2.0],
            vector![app_config.world_config.width, 0.2],
        ),
        (
            vector![0.0, app_config.world_config.height / 2.0],
            vector![app_config.world_config.width, 0.2],
        ),
        (
            vector![-app_config.world_config.width / 2.0, 0.0],
            vector![0.2, app_config.world_config.height],
        ),
        (
            vector![app_config.world_config.width / 2.0, 0.0],
            vector![0.2, app_config.world_config.height],
        ),
    ];

    for ((pos, size), (pos_scaled, size_scaled)) in boundaries_physics.iter().zip(boundaries.iter())
    {
        // Physics (original scale)
        let rigid_body = RigidBodyBuilder::fixed().translation(*pos).build();
        let collider = ColliderBuilder::cuboid(size.x / 2.0, size.y / 2.0).build();
        let handle = physics_world.rigid_body_set.insert(rigid_body);
        physics_world.collider_set.insert(collider);

        // Visual (scaled for visibility)
        let wc = app_config.world_config.wall_color;
        commands.spawn((
            Sprite {
                color: Color::srgb(wc[0], wc[1], wc[2]),
                custom_size: Some(Vec2::new(size_scaled.x, size_scaled.y)),
                ..default()
            },
            Transform::from_translation(Vec3::new(pos_scaled.x, pos_scaled.y, 0.0)),
            WorldElement,
            PhysicsHandle {
                rigid_body_handle: handle,
            },
        ));
    }

    // Create obstacles
    for obstacle in &app_config.world_config.obstacles {
        let pos_physics = vector![obstacle.pos[0], obstacle.pos[1]];
        let pos_visual = vector![obstacle.pos[0] * scale, obstacle.pos[1] * scale];

        // Determine obstacle color (custom or configurable default)
        let default_oc = app_config.world_config.default_obstacle_color;
        let obstacle_color = obstacle
            .color
            .map(|c| Color::srgb(c[0], c[1], c[2]))
            .unwrap_or_else(|| Color::srgb(default_oc[0], default_oc[1], default_oc[2]));

        // Create physics body and visual representation based on shape
        let (rigid_body, collider, sprite) = match obstacle.shape {
            ObstacleShape::Rectangle => {
                let size_physics = vector![obstacle.size[0], obstacle.size[1]];
                let size_visual = vector![obstacle.size[0] * scale, obstacle.size[1] * scale];

                let rigid_body = RigidBodyBuilder::fixed().translation(pos_physics).build();
                let collider =
                    ColliderBuilder::cuboid(size_physics.x / 2.0, size_physics.y / 2.0).build();
                let sprite = Sprite {
                    color: obstacle_color,
                    custom_size: Some(Vec2::new(size_visual.x, size_visual.y)),
                    ..default()
                };

                (rigid_body, collider, sprite)
            }
            ObstacleShape::Circle => {
                let radius_physics = obstacle.size[0]; // Use first element as radius
                let radius_visual = radius_physics * scale;

                let rigid_body = RigidBodyBuilder::fixed().translation(pos_physics).build();
                let collider = ColliderBuilder::ball(radius_physics).build();

                // Circle obstacles are rendered as square sprites
                // The physics collider is properly circular, ensuring accurate collision detection
                let sprite = Sprite {
                    color: obstacle_color,
                    custom_size: Some(Vec2::new(radius_visual * 2.0, radius_visual * 2.0)),
                    ..default()
                };

                (rigid_body, collider, sprite)
            }
        };

        let handle = physics_world.rigid_body_set.insert(rigid_body);
        physics_world.collider_set.insert(collider);

        // Visual (scaled)
        commands.spawn((
            sprite,
            Transform::from_translation(Vec3::new(pos_visual.x, pos_visual.y, 0.5)),
            WorldElement,
            ObstacleElement, // Mark as obstacle for color updates
            PhysicsHandle {
                rigid_body_handle: handle,
            },
        ));
    }

    // Create robots (support multiple robots)
    for robot_config in &app_config.robots {
        let robot_pos = vector![robot_config.position[0], robot_config.position[1]]; // Use configured position
        let robot_size = vector![robot_config.length * scale, robot_config.width * scale];

        // Physics (use original scale)
        let robot_size_physics = vector![robot_config.length, robot_config.width];
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(robot_pos)
            .linear_damping(2.0) // Natural deceleration
            .angular_damping(1.0)
            .build();
        let collider =
            ColliderBuilder::cuboid(robot_size_physics.x / 2.0, robot_size_physics.y / 2.0).build();

        let robot_handle = physics_world.rigid_body_set.insert(rigid_body);
        physics_world.collider_set.insert(collider);

        // Visual
        let robot_color = Color::srgb(
            robot_config.color[0],
            robot_config.color[1],
            robot_config.color[2],
        );

        let robot_entity = commands
            .spawn((
                Sprite {
                    color: robot_color,
                    custom_size: Some(Vec2::new(robot_size.x, robot_size.y)),
                    ..default()
                },
                Transform::from_translation(Vec3::new(
                    robot_pos.x * scale,
                    robot_pos.y * scale,
                    1.0,
                )),
                Robot {
                    name: robot_config.name.clone(),
                    config: robot_config.clone(),
                    rigid_body_handle: robot_handle,
                },
            ))
            .id();

        // Spawn visual components if configured
        spawn_robot_visual_components(&mut commands, robot_entity, robot_config, scale);
    }

    // Spawn articulated robots
    for articulated_config in &app_config.articulated_robots {
        // Enable gravity if robot config requires it
        if articulated_config.enable_gravity {
            physics_world.gravity = vector![0.0, -9.81];
        }

        crate::joint::spawn_articulated_robot(
            &mut commands,
            &mut physics_world,
            articulated_config,
            scale,
        );
    }

    info!(" sim2d setup complete!");
    info!(
        "   [#] World: {}x{} meters",
        app_config.world_config.width, app_config.world_config.height
    );
    info!("    Robots: {} spawned", app_config.robots.len());
    if !app_config.articulated_robots.is_empty() {
        info!(
            "    Articulated robots: {} spawned",
            app_config.articulated_robots.len()
        );
    }
    info!(
        "   Obstacles: {} created",
        app_config.world_config.obstacles.len()
    );
}

/// HORUS communication system
pub fn horus_system(
    mut horus_comm: Option<ResMut<HorusComm>>,
    robots: Query<&Robot>,
    mut physics_world: ResMut<PhysicsWorld>,
    ui_state: Res<ui::UiState>,
) {
    // Don't process commands if paused
    if ui_state.paused {
        return;
    }

    if let Some(ref mut comm) = horus_comm {
        let HorusComm {
            ref mut robot_hubs,
            ref mut node_info,
            ..
        } = **comm;

        // Process cmd_vel for each robot independently
        for robot in robots.iter() {
            if let Some(hubs) = robot_hubs.get_mut(&robot.name) {
                if let Some(cmd_vel) = hubs.cmd_vel_sub.recv(&mut Some(node_info)) {
                    if let Some(rigid_body) = physics_world
                        .rigid_body_set
                        .get_mut(robot.rigid_body_handle)
                    {
                        // Apply velocity using robot's kinematics model
                        robot.config.kinematics.apply_velocity(
                            &cmd_vel,
                            rigid_body,
                            robot.config.max_speed,
                        );
                    }
                }
            }
        }
    }
}

/// Physics update system
pub fn physics_system(mut physics_world: ResMut<PhysicsWorld>, ui_state: Res<ui::UiState>) {
    // Don't run physics if paused
    if ui_state.paused {
        return;
    }

    let PhysicsWorld {
        ref mut physics_pipeline,
        ref gravity,
        ref mut integration_parameters,
        ref mut island_manager,
        ref mut broad_phase,
        ref mut narrow_phase,
        ref mut rigid_body_set,
        ref mut collider_set,
        ref mut impulse_joint_set,
        ref mut multibody_joint_set,
        ref mut ccd_solver,
        ref physics_hooks,
        ref event_handler,
    } = *physics_world;

    // Apply simulation speed
    let mut params = *integration_parameters;
    params.dt *= ui_state.simulation_speed;

    physics_pipeline.step(
        gravity,
        &params,
        island_manager,
        broad_phase,
        narrow_phase,
        rigid_body_set,
        collider_set,
        impulse_joint_set,
        multibody_joint_set,
        ccd_solver,
        None,
        physics_hooks,
        event_handler,
    );
}

/// Tick start system - marks the beginning of a simulation frame for metrics tracking
pub fn tick_start_system(mut horus_comm: Option<ResMut<HorusComm>>) {
    if let Some(ref mut comm) = horus_comm {
        // Directly increment tick count at frame start so all messages
        // within this frame use the new tick number
        comm.node_info.increment_tick();
        // Start the tick timer for duration tracking
        comm.node_info.start_tick();
    }
}

/// Topic update system - processes pending topic prefix changes without restart
pub fn topic_update_system(
    mut ui_state: ResMut<ui::UiState>,
    mut horus_comm: Option<ResMut<HorusComm>>,
) {
    // Check if there's a pending topic update
    if let Some((robot_name, new_prefix)) = ui_state.pending_topic_update.take() {
        if let Some(ref mut comm) = horus_comm {
            if comm.update_robot_topics(&robot_name, &new_prefix) {
                ui_state.status_message = format!("Topic updated to {} successfully", new_prefix);
                info!(
                    "Dynamic topic update successful: {} -> {}",
                    robot_name, new_prefix
                );
            } else {
                ui_state.status_message = format!("Failed to update topic to {}", new_prefix);
                warn!("Dynamic topic update failed for robot '{}'", robot_name);
            }
        }
    }
}

/// Tick end system - captures recording frames
pub fn tick_end_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut recorder: Option<ResMut<crate::recorder::Recorder>>,
    time: Res<Time>,
    last_scan: Res<LastLidarScan>,
) {
    // Record frame if recording is active
    if let Some(ref mut recorder) = recorder {
        if recorder.is_recording() {
            let mut robot_states = Vec::new();

            for robot in robot_query.iter() {
                if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle)
                {
                    let pos = rigid_body.translation();
                    let linvel = rigid_body.linvel();
                    let angvel = rigid_body.angvel();
                    let heading = rigid_body.rotation().angle();

                    robot_states.push(crate::recorder::RobotFrameState {
                        name: robot.config.name.clone(),
                        position: [pos.x, pos.y],
                        heading,
                        linear_velocity: (linvel.x * linvel.x + linvel.y * linvel.y).sqrt(),
                        angular_velocity: angvel,
                        lidar_scan: if !last_scan.ranges.is_empty() {
                            Some(last_scan.ranges.clone())
                        } else {
                            None
                        },
                    });
                }
            }

            recorder.record_frame(
                time.elapsed_secs_f64(),
                robot_states,
                None, // No screenshots for now
            );
        }
    }
}

/// Visual sync system - updates Bevy transforms from physics
pub fn visual_sync_system(
    mut robot_query: Query<(&Robot, &mut Transform, &mut Sprite)>,
    physics_world: Res<PhysicsWorld>,
    app_config: Res<AppConfig>,
) {
    let scale = 50.0; // Same scale used in setup
    for (robot, mut transform, mut sprite) in robot_query.iter_mut() {
        if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle) {
            // Update robot visual position from physics (scale up for visibility)
            let pos = rigid_body.translation();
            let rot = rigid_body.rotation();

            transform.translation.x = pos.x * scale;
            transform.translation.y = pos.y * scale;
            transform.rotation = Quat::from_rotation_z(rot.angle());

            // Update robot visual size from component config
            sprite.custom_size = Some(Vec2::new(
                robot.config.length * scale,
                robot.config.width * scale,
            ));

            // Update robot color from live AppConfig (allows UI changes to take effect immediately)
            // Find the matching robot config by name
            if let Some(live_config) = app_config.robots.iter().find(|r| r.name == robot.name) {
                sprite.color = Color::srgb(
                    live_config.color[0],
                    live_config.color[1],
                    live_config.color[2],
                );
            } else {
                sprite.color = Color::srgb(
                    robot.config.color[0],
                    robot.config.color[1],
                    robot.config.color[2],
                );
            }
        }
    }
}

/// Telemetry update system - updates robot telemetry data
pub fn telemetry_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut telemetry: ResMut<ui::RobotTelemetry>,
) {
    for robot in robot_query.iter() {
        if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle) {
            let pos = rigid_body.translation();
            let vel = rigid_body.linvel();
            let rot = rigid_body.rotation();
            let angvel = rigid_body.angvel();

            telemetry.position = (pos.x, pos.y);
            telemetry.velocity = (vel.x, vel.y);
            telemetry.heading = rot.angle();
            telemetry.angular_velocity = angvel;
        }
    }
}

/// Performance metrics update system - tracks path, speed, collisions
pub fn metrics_update_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    collision_state: Res<CollisionState>,
    time: Res<Time>,
    ui_state: Res<ui::UiState>,
    mut perf_metrics: ResMut<crate::metrics::PerformanceMetrics>,
) {
    // Don't update metrics if paused
    if ui_state.paused {
        return;
    }

    for robot in robot_query.iter() {
        if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle) {
            let pos = rigid_body.translation();
            let vel = rigid_body.linvel();

            // Update metrics with current robot state
            perf_metrics.update(
                bevy::math::Vec2::new(pos.x, pos.y),
                bevy::math::Vec2::new(vel.x, vel.y),
                time.delta_secs(),
            );
        }
    }

    // Sync collision count from collision detection system
    perf_metrics.collision_count = collision_state.collision_count as u32;
}

/// Odometry publishing system - publishes robot state to HORUS
pub fn odometry_publish_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut horus_comm: Option<ResMut<HorusComm>>,
) {
    if let Some(ref mut comm) = horus_comm {
        let HorusComm {
            robot_hubs,
            node_info,
            ..
        } = &mut **comm;

        for robot in robot_query.iter() {
            // Get this robot's hubs
            if let Some(hubs) = robot_hubs.get_mut(&robot.name) {
                if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle)
                {
                    let pos = rigid_body.translation();
                    let vel = rigid_body.linvel();
                    let rot = rigid_body.rotation();
                    let angvel = rigid_body.angvel();

                    // Get current timestamp
                    let timestamp = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_else(|_| std::time::Duration::from_secs(0))
                        .as_nanos() as u64;

                    // Create odometry message
                    let mut odom = Odometry::new();

                    // Set pose
                    odom.pose = Pose2D::new(pos.x as f64, pos.y as f64, rot.angle() as f64);

                    // Set twist (velocity)
                    odom.twist = Twist {
                        linear: [vel.x as f64, vel.y as f64, 0.0],
                        angular: [0.0, 0.0, angvel as f64],
                        timestamp,
                    };

                    // Set frame IDs
                    let frame_id = b"odom\0";
                    let child_frame_id = b"base_link\0";
                    odom.frame_id[..frame_id.len()].copy_from_slice(frame_id);
                    odom.child_frame_id[..child_frame_id.len()].copy_from_slice(child_frame_id);

                    // Set timestamp
                    odom.timestamp = timestamp;

                    // Publish to this robot's odom topic
                    if let Err(e) = hubs.odom_pub.send(odom, &mut Some(node_info)) {
                        warn!("Failed to publish odometry for {}: {:?}", robot.name, e);
                    }
                }
            }
        }
    }
}

/// IMU simulation system - publishes IMU data
pub fn imu_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut horus_comm: Option<ResMut<HorusComm>>,
    mut prev_vel: ResMut<PreviousVelocity>,
) {
    if let Some(ref mut comm) = horus_comm {
        let HorusComm {
            robot_hubs,
            node_info,
            ..
        } = &mut **comm;

        for robot in robot_query.iter() {
            // Get this robot's hubs
            if let Some(hubs) = robot_hubs.get_mut(&robot.name) {
                if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle)
                {
                    let vel = rigid_body.linvel();
                    let rot = rigid_body.rotation();
                    let angvel = rigid_body.angvel();

                    // Calculate time delta
                    let now = std::time::Instant::now();
                    let dt = now.duration_since(prev_vel.timestamp).as_secs_f32();

                    // Calculate linear acceleration (change in velocity / time)
                    let accel_x = if dt > 0.0 {
                        (vel.x - prev_vel.linear.0) / dt
                    } else {
                        0.0
                    };
                    let accel_y = if dt > 0.0 {
                        (vel.y - prev_vel.linear.1) / dt
                    } else {
                        0.0
                    };

                    // Update previous velocity
                    prev_vel.linear = (vel.x, vel.y);
                    prev_vel.angular = angvel;
                    prev_vel.timestamp = now;

                    // Create IMU message
                    let mut imu = Imu::new();

                    // Set orientation as quaternion (rotation around Z-axis only for 2D)
                    let angle = rot.angle() as f64;
                    let half_angle = angle / 2.0;
                    imu.orientation = [
                        0.0,              // x
                        0.0,              // y
                        half_angle.sin(), // z
                        half_angle.cos(), // w
                    ];

                    // Set angular velocity (only Z-axis for 2D)
                    imu.angular_velocity = [0.0, 0.0, angvel as f64];

                    // Set linear acceleration
                    imu.linear_acceleration = [accel_x as f64, accel_y as f64, 0.0];

                    // Set timestamp
                    imu.timestamp = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_else(|_| std::time::Duration::from_secs(0))
                        .as_nanos() as u64;

                    // Publish to this robot's IMU topic
                    if let Err(e) = hubs.imu_pub.send(imu, &mut Some(node_info)) {
                        warn!("Failed to publish IMU for {}: {:?}", robot.name, e);
                    }
                }
            }
        }
    }
}

/// LIDAR simulation system - performs raycasting and publishes LaserScan
pub fn lidar_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut horus_comm: Option<ResMut<HorusComm>>,
    _app_config: Res<AppConfig>,
    mut last_scan: ResMut<LastLidarScan>,
) {
    if let Some(ref mut comm) = horus_comm {
        let HorusComm {
            robot_hubs,
            node_info,
            ..
        } = &mut **comm;

        for robot in robot_query.iter() {
            // Get this robot's hubs
            if let Some(hubs) = robot_hubs.get_mut(&robot.name) {
                if !robot.config.lidar.enabled {
                    continue;
                }

                if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle)
                {
                    let pos = rigid_body.translation();
                    let rot = rigid_body.rotation();
                    let robot_angle = rot.angle();

                    let lidar_cfg = &robot.config.lidar;

                    // Create laser scan message
                    let mut scan = LaserScan::default();
                    scan.angle_min = lidar_cfg.angle_min;
                    scan.angle_max = lidar_cfg.angle_max;
                    scan.range_min = lidar_cfg.range_min;
                    scan.range_max = lidar_cfg.range_max;
                    scan.angle_increment =
                        (lidar_cfg.angle_max - lidar_cfg.angle_min) / lidar_cfg.num_rays as f32;
                    scan.scan_time = 0.1; // 10 Hz scan rate
                    scan.time_increment = scan.scan_time / lidar_cfg.num_rays as f32;

                    // Perform raycasting for each beam
                    let query_pipeline = QueryPipeline::new();
                    let step = (lidar_cfg.angle_max - lidar_cfg.angle_min)
                        / (lidar_cfg.num_rays as f32 - 1.0);

                    // Store for visualization
                    last_scan.ranges.clear();
                    last_scan.angles.clear();
                    last_scan.robot_pos = (pos.x, pos.y);
                    last_scan.robot_angle = robot_angle;

                    let actual_rays = lidar_cfg.num_rays.min(360);
                    if lidar_cfg.num_rays > 360 {
                        // Log warning once per robot (could use a flag for one-time warning)
                        static WARNED: std::sync::atomic::AtomicBool =
                            std::sync::atomic::AtomicBool::new(false);
                        if !WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                            warn!(
                                "LiDAR ray count {} exceeds maximum of 360, clamping to 360",
                                lidar_cfg.num_rays
                            );
                        }
                    }

                    for i in 0..actual_rays {
                        let angle = lidar_cfg.angle_min + i as f32 * step + robot_angle;
                        let ray_dir = vector![angle.cos(), angle.sin()];
                        let ray = Ray::new(point![pos.x, pos.y], ray_dir);

                        // Cast ray and find nearest intersection
                        let hit = query_pipeline.cast_ray(
                            &physics_world.rigid_body_set,
                            &physics_world.collider_set,
                            &ray,
                            lidar_cfg.range_max,
                            true, // solid objects only
                            QueryFilter::default(),
                        );

                        let range = hit.map(|(_, toi)| toi).unwrap_or(0.0);
                        scan.ranges[i] = range;

                        // Store for visualization
                        last_scan.ranges.push(range);
                        last_scan.angles.push(angle);
                    }

                    // Set timestamp
                    scan.timestamp = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_else(|_| std::time::Duration::from_secs(0))
                        .as_nanos() as u64;

                    // Publish to this robot's LiDAR topic
                    if let Err(e) = hubs.lidar_pub.send(scan, &mut Some(node_info)) {
                        warn!("Failed to publish LiDAR for {}: {:?}", robot.name, e);
                    }
                }
            }
        }
    }
}

/// Camera sensor system - renders camera images from robot perspective
pub fn camera_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    app_config: Res<AppConfig>,
    mut camera_sensors: ResMut<CameraSensors>,
) {
    // Collect robot data first
    let robots_to_render: Vec<_> = robot_query
        .iter()
        .filter(|r| r.config.camera.enabled)
        .filter_map(|robot| {
            physics_world
                .rigid_body_set
                .get(robot.rigid_body_handle)
                .map(|rb| {
                    (
                        robot.config.name.clone(),
                        robot.config.camera.clone(),
                        [rb.translation().x, rb.translation().y],
                        rb.rotation().angle(),
                    )
                })
        })
        .collect();

    // Collect obstacles once
    let obstacles: Vec<Obstacle> = app_config
        .world_config
        .obstacles
        .iter()
        .map(|o| Obstacle {
            pos: o.pos,
            size: o.size,
            shape: o.shape.clone(),
            color: o.color,
        })
        .collect();

    let world_width = app_config.world_config.width;
    let world_height = app_config.world_config.height;

    // Render each camera and collect results
    let mut rendered_images = Vec::new();
    for (name, camera_config, pos, heading) in robots_to_render {
        // Get or create camera sensor
        let sensor = camera_sensors
            .sensors
            .entry(name.clone())
            .or_insert_with(|| crate::camera::CameraSensor::new(camera_config));

        // Render camera image and clone it
        let image = sensor
            .render(pos, heading, &obstacles, world_width, world_height)
            .clone();
        rendered_images.push((name, image));
    }

    // Store all rendered images
    for (name, image) in rendered_images {
        camera_sensors.last_images.insert(name, image);
    }
}

/// GPS sensor system - provides simulated GPS readings
pub fn gps_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    time: Res<Time>,
    mut gps_sensors: ResMut<GpsSensors>,
) {
    let elapsed = time.elapsed().as_secs_f64();

    // Collect robot data first to avoid borrow issues
    let robots_data: Vec<_> = robot_query
        .iter()
        .filter(|r| r.config.gps.enabled)
        .filter_map(|robot| {
            physics_world
                .rigid_body_set
                .get(robot.rigid_body_handle)
                .map(|rb| {
                    let pos = rb.translation();
                    (
                        robot.config.name.clone(),
                        robot.config.gps.clone(),
                        rapier2d::na::vector![pos.x, pos.y],
                    )
                })
        })
        .collect();

    // Process GPS readings
    for (name, gps_config, position) in robots_data {
        let sensor = gps_sensors
            .sensors
            .entry(name.clone())
            .or_insert_with(|| crate::sensors::GpsSensor::new(gps_config));

        if let Some(gps_data) = sensor.update(&position, elapsed) {
            gps_sensors.last_readings.insert(name, gps_data);
        }
    }
}

/// Ultrasonic sensor system - provides range readings using raycasting
pub fn ultrasonic_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut ultrasonic_sensors: ResMut<UltrasonicSensors>,
) {
    let query_pipeline = QueryPipeline::new();

    for robot in robot_query.iter() {
        if !robot.config.ultrasonic.enabled {
            continue;
        }

        if let Some(rb) = physics_world.rigid_body_set.get(robot.rigid_body_handle) {
            let robot_pos = rb.translation();
            let robot_heading = rb.rotation().angle();

            let mut ranges = Vec::new();

            for sensor_config in &robot.config.ultrasonic.sensors {
                // Calculate sensor position in world frame
                let cos_h = robot_heading.cos();
                let sin_h = robot_heading.sin();
                let sensor_x =
                    robot_pos.x + sensor_config.offset[0] * cos_h - sensor_config.offset[1] * sin_h;
                let sensor_y =
                    robot_pos.y + sensor_config.offset[0] * sin_h + sensor_config.offset[1] * cos_h;

                // Calculate ray direction
                let ray_angle = robot_heading + sensor_config.angle;
                let ray_dir = vector![ray_angle.cos(), ray_angle.sin()];

                // Cast ray
                let ray = Ray::new(point![sensor_x, sensor_y], ray_dir);

                let hit = query_pipeline.cast_ray(
                    &physics_world.rigid_body_set,
                    &physics_world.collider_set,
                    &ray,
                    sensor_config.max_range,
                    true,
                    QueryFilter::default(),
                );

                let range = match hit {
                    Some((_, toi)) if toi >= sensor_config.min_range => {
                        sensor_config.noise.apply(toi)
                    }
                    Some((_, toi)) if toi < sensor_config.min_range => {
                        // Below minimum range - report min range
                        sensor_config.min_range
                    }
                    _ => {
                        // No hit - return max range
                        sensor_config.max_range
                    }
                };

                ranges.push(range);
            }

            ultrasonic_sensors
                .last_readings
                .insert(robot.config.name.clone(), ranges);
        }
    }
}

/// Contact sensor system - detects collisions with obstacles
pub fn contact_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut contact_sensors: ResMut<ContactSensors>,
) {
    for robot in robot_query.iter() {
        if !robot.config.contact.enabled {
            continue;
        }

        let num_zones = robot.config.contact.num_zones;
        let mut zones = vec![false; num_zones];
        let mut total_force = 0.0_f32;

        // Check collisions for this robot's collider
        if let Some(collider_handle) = physics_world
            .rigid_body_set
            .get(robot.rigid_body_handle)
            .and_then(|rb| rb.colliders().first().copied())
        {
            // Iterate through contact pairs involving this collider
            for contact_pair in physics_world
                .narrow_phase
                .contact_pairs_with(collider_handle)
            {
                if contact_pair.has_any_active_contact {
                    // Get contact normal to determine which zone
                    for manifold in contact_pair.manifolds.iter() {
                        let normal = manifold.local_n1;
                        let angle = normal.y.atan2(normal.x);

                        // Map angle to zone (0 to num_zones-1)
                        let normalized_angle =
                            (angle + std::f32::consts::PI) / (2.0 * std::f32::consts::PI);
                        let zone_idx = (normalized_angle * num_zones as f32) as usize % num_zones;
                        zones[zone_idx] = true;

                        // Sum up contact forces
                        for point in manifold.points.iter() {
                            total_force += point.data.impulse.abs();
                        }
                    }
                }
            }
        }

        contact_sensors.last_readings.insert(
            robot.config.name.clone(),
            crate::sensors::ContactData { zones, total_force },
        );
    }
}

/// Robot config change detection - respawns visual components when config changes
pub fn robot_visual_reload_system(
    mut commands: Commands,
    robot_query: Query<(Entity, &Robot)>,
    visual_components: Query<Entity, Or<(With<RobotTurret>, With<RobotCannon>, With<RobotTread>)>>,
    app_config: Res<AppConfig>,
) {
    if !app_config.is_changed() {
        return;
    }

    info!("Robot config changed - reloading visual components");

    // Despawn all existing visual components
    for entity in visual_components.iter() {
        commands.entity(entity).despawn();
    }

    // Respawn visual components for each robot with new config
    for (entity, robot) in robot_query.iter() {
        spawn_robot_visual_components(&mut commands, entity, &robot.config, 50.0);
    }
}

/// Visual component sync - updates turret, cannon, treads to follow robot
pub fn visual_component_sync_system(
    robot_query: Query<(&Robot, &Transform)>,
    mut turret_query: Query<(&RobotTurret, &mut Transform), Without<Robot>>,
    mut cannon_query: Query<(&RobotCannon, &mut Transform), (Without<Robot>, Without<RobotTurret>)>,
    mut tread_query: Query<
        (&RobotTread, &mut Transform),
        (Without<Robot>, Without<RobotTurret>, Without<RobotCannon>),
    >,
) {
    let scale = 50.0;

    // Update turrets
    for (turret, mut transform) in turret_query.iter_mut() {
        if let Ok((robot, robot_transform)) = robot_query.get(turret.parent) {
            if let Some(ref turret_config) = robot.config.visual.turret {
                // Apply robot rotation to offset
                let angle = robot_transform.rotation.to_euler(EulerRot::ZYX).0;
                let rotated_offset = Vec2::new(
                    turret_config.offset_x * angle.cos() - turret_config.offset_y * angle.sin(),
                    turret_config.offset_x * angle.sin() + turret_config.offset_y * angle.cos(),
                ) * scale;

                transform.translation.x = robot_transform.translation.x + rotated_offset.x;
                transform.translation.y = robot_transform.translation.y + rotated_offset.y;
                transform.rotation = robot_transform.rotation;
            }
        }
    }

    // Update cannons
    for (cannon, mut transform) in cannon_query.iter_mut() {
        if let Ok((robot, robot_transform)) = robot_query.get(cannon.parent) {
            if let Some(ref cannon_config) = robot.config.visual.cannon {
                // Cannon extends forward from robot center
                let angle = robot_transform.rotation.to_euler(EulerRot::ZYX).0;
                let forward_offset = Vec2::new(
                    cannon_config.offset_x * angle.cos(),
                    cannon_config.offset_x * angle.sin(),
                ) * scale;

                transform.translation.x = robot_transform.translation.x + forward_offset.x;
                transform.translation.y = robot_transform.translation.y + forward_offset.y;
                transform.rotation = robot_transform.rotation;
            }
        }
    }

    // Update treads
    for (tread, mut transform) in tread_query.iter_mut() {
        if let Ok((robot, robot_transform)) = robot_query.get(tread.parent) {
            if let Some(ref tread_config) = robot.config.visual.treads {
                // Treads are offset perpendicular to forward direction
                let angle = robot_transform.rotation.to_euler(EulerRot::ZYX).0;
                let offset_multiplier = if tread.is_left { 1.0 } else { -1.0 };
                let lateral_offset = Vec2::new(
                    -angle.sin() * tread_config.offset * offset_multiplier,
                    angle.cos() * tread_config.offset * offset_multiplier,
                ) * scale;

                transform.translation.x = robot_transform.translation.x + lateral_offset.x;
                transform.translation.y = robot_transform.translation.y + lateral_offset.y;
                transform.rotation = robot_transform.rotation;
            }
        }
    }
}

/// Camera control system - applies zoom and pan
pub fn camera_control_system(
    mut camera_query: Query<(&mut Transform, &mut OrthographicProjection), With<Camera2d>>,
    camera_controller: Res<ui::CameraController>,
) {
    for (mut transform, mut projection) in camera_query.iter_mut() {
        // Apply pan
        transform.translation.x = camera_controller.pan_x;
        transform.translation.y = camera_controller.pan_y;

        // Apply zoom by adjusting orthographic scale
        // Lower scale = more zoomed in, higher scale = more zoomed out
        projection.scale = 1.0 / camera_controller.zoom;
    }
}

/// Collision detection system - detects collisions using the physics engine
pub fn collision_detection_system(
    robot_query: Query<&Robot>,
    physics_world: Res<PhysicsWorld>,
    mut collision_state: ResMut<CollisionState>,
) {
    collision_state.is_colliding = false;

    for robot in robot_query.iter() {
        // Get all colliders attached to the robot's rigid body
        if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle) {
            // Check for contacts by iterating through all contact pairs in narrow phase
            for contact_pair in physics_world.narrow_phase.contact_pairs() {
                // Check if this contact involves any of the robot's colliders
                // We need to check the collider handles, not rigid body handles
                let robot_colliders: Vec<_> = rigid_body.colliders().iter().collect();

                let involves_robot = robot_colliders.contains(&&contact_pair.collider1)
                    || robot_colliders.contains(&&contact_pair.collider2);

                if involves_robot && contact_pair.has_any_active_contact {
                    collision_state.is_colliding = true;
                    collision_state.collision_count += 1;
                    collision_state.last_collision_time = Some(std::time::Instant::now());
                    break;
                }
            }
        }
    }
}

/// Collision visual indicator system - shows visual feedback for collisions
pub fn collision_indicator_system(
    mut commands: Commands,
    robot_query: Query<&Transform, With<Robot>>,
    collision_state: Res<CollisionState>,
    existing_indicators: Query<Entity, With<CollisionIndicator>>,
) {
    // Clear existing indicators
    for entity in existing_indicators.iter() {
        commands.entity(entity).despawn();
    }

    // Show indicator if colliding
    if collision_state.is_colliding {
        for transform in robot_query.iter() {
            // Red circle around robot to indicate collision
            commands.spawn((
                Sprite {
                    color: Color::srgba(1.0, 0.0, 0.0, 0.5), // Semi-transparent red
                    custom_size: Some(Vec2::new(60.0, 60.0)),
                    ..default()
                },
                Transform::from_translation(Vec3::new(
                    transform.translation.x,
                    transform.translation.y,
                    0.9, // Just below robot
                )),
                CollisionIndicator,
            ));
        }
    }
}

/// Mouse camera control system - handles mouse input for camera movement
pub fn mouse_camera_system(
    mut camera_controller: ResMut<ui::CameraController>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    mut mouse_motion: EventReader<bevy::input::mouse::MouseMotion>,
    mut mouse_wheel: EventReader<bevy::input::mouse::MouseWheel>,
) {
    // Handle mouse wheel zoom
    for ev in mouse_wheel.read() {
        let zoom_delta = match ev.unit {
            bevy::input::mouse::MouseScrollUnit::Line => ev.y * 0.1,
            bevy::input::mouse::MouseScrollUnit::Pixel => ev.y * 0.01,
        };

        camera_controller.zoom = (camera_controller.zoom + zoom_delta).clamp(0.1, 5.0);
    }

    // Handle middle mouse button drag for pan
    if mouse_button_input.pressed(MouseButton::Middle) {
        for ev in mouse_motion.read() {
            // Invert the delta for natural panning
            camera_controller.pan_x -= ev.delta.x;
            camera_controller.pan_y += ev.delta.y; // Y is inverted in screen space
        }
    } else {
        // Clear motion events when not panning
        mouse_motion.clear();
    }
}

/// Visual color update system - updates colors based on preferences
pub fn visual_color_system(
    mut obstacle_query: Query<&mut Sprite, (With<ObstacleElement>, Without<GridLine>)>,
    mut wall_query: Query<
        &mut Sprite,
        (
            With<WorldElement>,
            Without<ObstacleElement>,
            Without<GridLine>,
        ),
    >,
    visual_prefs: Res<ui::VisualPreferences>,
) {
    // Update obstacle colors
    for mut sprite in obstacle_query.iter_mut() {
        sprite.color = Color::srgb(
            visual_prefs.obstacle_color[0],
            visual_prefs.obstacle_color[1],
            visual_prefs.obstacle_color[2],
        );
    }

    // Update wall colors
    for mut sprite in wall_query.iter_mut() {
        sprite.color = Color::srgb(
            visual_prefs.wall_color[0],
            visual_prefs.wall_color[1],
            visual_prefs.wall_color[2],
        );
    }
}

/// Grid rendering system - shows/hides and updates grid
pub fn grid_system(
    mut commands: Commands,
    visual_prefs: Res<ui::VisualPreferences>,
    existing_grid: Query<Entity, With<GridLine>>,
    app_config: Res<AppConfig>,
) {
    // Clear existing grid if preferences changed or grid is disabled
    if visual_prefs.is_changed() || !visual_prefs.show_grid {
        for entity in existing_grid.iter() {
            commands.entity(entity).despawn();
        }
    }

    // Re-create grid if enabled
    if visual_prefs.show_grid && visual_prefs.is_changed() {
        let scale = 50.0;
        let world_half_width = app_config.world_config.width / 2.0;
        let world_half_height = app_config.world_config.height / 2.0;
        let spacing = visual_prefs.grid_spacing;

        let grid_color = Color::srgba(
            visual_prefs.grid_color[0],
            visual_prefs.grid_color[1],
            visual_prefs.grid_color[2],
            0.3, // Semi-transparent
        );

        // Vertical lines
        let mut x = -world_half_width;
        while x <= world_half_width {
            commands.spawn((
                Sprite {
                    color: grid_color,
                    custom_size: Some(Vec2::new(
                        0.05 * scale,
                        app_config.world_config.height * scale,
                    )),
                    ..default()
                },
                Transform::from_translation(Vec3::new(x * scale, 0.0, -1.0)),
                GridLine,
            ));
            x += spacing;
        }

        // Horizontal lines
        let mut y = -world_half_height;
        while y <= world_half_height {
            commands.spawn((
                Sprite {
                    color: grid_color,
                    custom_size: Some(Vec2::new(
                        app_config.world_config.width * scale,
                        0.05 * scale,
                    )),
                    ..default()
                },
                Transform::from_translation(Vec3::new(0.0, y * scale, -1.0)),
                GridLine,
            ));
            y += spacing;
        }
    }
}

/// Velocity arrow visualization system
pub fn velocity_arrow_system(
    mut commands: Commands,
    robot_query: Query<(&Robot, &Transform)>,
    physics_world: Res<PhysicsWorld>,
    visual_prefs: Res<ui::VisualPreferences>,
    existing_arrows: Query<Entity, With<VelocityArrow>>,
) {
    // Clear existing arrows
    for entity in existing_arrows.iter() {
        commands.entity(entity).despawn();
    }

    if !visual_prefs.show_velocity_arrows {
        return;
    }

    let scale = 50.0;

    for (robot, robot_transform) in robot_query.iter() {
        if let Some(rigid_body) = physics_world.rigid_body_set.get(robot.rigid_body_handle) {
            let vel = rigid_body.linvel();
            let speed = (vel.x * vel.x + vel.y * vel.y).sqrt();

            if speed < 0.01 {
                continue; // Don't show arrow for stationary robot
            }

            // Arrow length proportional to speed
            let arrow_length = speed * scale * 2.0;
            let vel_angle = vel.y.atan2(vel.x);

            // Draw arrow as a rectangle
            commands.spawn((
                Sprite {
                    color: Color::srgb(1.0, 1.0, 0.0), // Yellow
                    custom_size: Some(Vec2::new(arrow_length, 5.0)),
                    ..default()
                },
                Transform::from_translation(Vec3::new(
                    robot_transform.translation.x + (arrow_length / 2.0) * vel_angle.cos(),
                    robot_transform.translation.y + (arrow_length / 2.0) * vel_angle.sin(),
                    2.0, // Above robot
                ))
                .with_rotation(Quat::from_rotation_z(vel_angle)),
                VelocityArrow,
            ));
        }
    }
}

/// LIDAR rays visualization system
pub fn lidar_rays_system(
    mut commands: Commands,
    visual_prefs: Res<ui::VisualPreferences>,
    last_scan: Res<LastLidarScan>,
    existing_rays: Query<Entity, With<LidarRay>>,
) {
    // Clear existing rays
    for entity in existing_rays.iter() {
        commands.entity(entity).despawn();
    }

    if !visual_prefs.show_lidar_rays || last_scan.ranges.is_empty() {
        return;
    }

    let scale = 50.0;
    let (robot_x, robot_y) = last_scan.robot_pos;

    // Only draw every Nth ray to avoid clutter
    let ray_step = (last_scan.ranges.len() / 60).max(1);

    for i in (0..last_scan.ranges.len()).step_by(ray_step) {
        let range = last_scan.ranges[i];
        if range < 0.01 {
            continue; // Invalid reading
        }

        let angle = last_scan.angles[i];
        let end_x = robot_x + range * angle.cos();
        let end_y = robot_y + range * angle.sin();

        // Calculate midpoint and length
        let mid_x = (robot_x + end_x) / 2.0 * scale;
        let mid_y = (robot_y + end_y) / 2.0 * scale;
        let length = range * scale;

        // Draw ray as thin line
        commands.spawn((
            Sprite {
                color: Color::srgba(0.0, 1.0, 1.0, 0.3), // Cyan, semi-transparent
                custom_size: Some(Vec2::new(length, 1.0)),
                ..default()
            },
            Transform::from_translation(Vec3::new(mid_x, mid_y, 1.5))
                .with_rotation(Quat::from_rotation_z(angle)),
            LidarRay,
        ));
    }
}

/// Trajectory trail system
pub fn trajectory_system(
    mut commands: Commands,
    robot_query: Query<&Transform, With<Robot>>,
    visual_prefs: Res<ui::VisualPreferences>,
    mut trajectory: ResMut<TrajectoryHistory>,
    existing_points: Query<Entity, With<TrajectoryPoint>>,
    time: Res<Time>,
) {
    // Update trajectory history
    if visual_prefs.show_trajectory {
        for transform in robot_query.iter() {
            // Sample every 0.1 seconds
            if (time.elapsed_secs() as usize).is_multiple_of(10) {
                let pos = (transform.translation.x, transform.translation.y);
                trajectory.points.push(pos);

                // Limit trail length
                if trajectory.points.len() > visual_prefs.trajectory_length {
                    trajectory.points.remove(0);
                }
            }
        }
    } else {
        trajectory.points.clear();
    }

    // Clear existing visualization
    for entity in existing_points.iter() {
        commands.entity(entity).despawn();
    }

    if !visual_prefs.show_trajectory || trajectory.points.is_empty() {
        return;
    }

    // Draw trail points
    for (i, &(x, y)) in trajectory.points.iter().enumerate() {
        let alpha = (i as f32 / trajectory.points.len() as f32) * 0.8 + 0.2;

        commands.spawn((
            Sprite {
                color: Color::srgba(0.2, 0.8, 0.2, alpha), // Green with fading
                custom_size: Some(Vec2::new(4.0, 4.0)),
                ..default()
            },
            Transform::from_translation(Vec3::new(x, y, 0.8)),
            TrajectoryPoint,
        ));
    }
}

/// Dynamic obstacle spawning/removal system
pub fn dynamic_obstacle_system(
    mut commands: Commands,
    mut physics_world: ResMut<PhysicsWorld>,
    mut id_counter: ResMut<ObstacleIdCounter>,
    obstacle_entities: Query<(Entity, &DynamicObstacleId, &PhysicsHandle), With<ObstacleElement>>,
    mut horus_comm: Option<ResMut<HorusComm>>,
    visual_prefs: Res<ui::VisualPreferences>,
) {
    // Only process if we have HORUS connection
    let Some(comm) = horus_comm.as_deref_mut() else {
        return;
    };

    // Try to receive obstacle command (non-blocking)
    let HorusComm {
        ref mut obstacle_cmd_sub,
        ref mut node_info,
        ..
    } = *comm;
    if let Some(cmd) = obstacle_cmd_sub.recv(&mut Some(node_info)) {
        match cmd.action {
            ObstacleAction::Add => {
                info!("  Spawning dynamic obstacle at {:?}", cmd.obstacle.pos);

                let scale = 50.0; // Same scale as setup
                let pos_physics = vector![cmd.obstacle.pos[0], cmd.obstacle.pos[1]];
                let pos_visual = vector![cmd.obstacle.pos[0] * scale, cmd.obstacle.pos[1] * scale];

                // Determine obstacle color
                let obstacle_color = cmd
                    .obstacle
                    .color
                    .map(|c| Color::srgb(c[0], c[1], c[2]))
                    .unwrap_or(Color::srgb(
                        visual_prefs.obstacle_color[0],
                        visual_prefs.obstacle_color[1],
                        visual_prefs.obstacle_color[2],
                    ));

                // Create physics body and visual based on shape
                let (rigid_body, collider, sprite) = match cmd.obstacle.shape {
                    ObstacleShape::Rectangle => {
                        let size_physics = vector![cmd.obstacle.size[0], cmd.obstacle.size[1]];
                        let size_visual =
                            vector![cmd.obstacle.size[0] * scale, cmd.obstacle.size[1] * scale];

                        let rigid_body = RigidBodyBuilder::fixed().translation(pos_physics).build();
                        let collider =
                            ColliderBuilder::cuboid(size_physics.x / 2.0, size_physics.y / 2.0)
                                .build();
                        let sprite = Sprite {
                            color: obstacle_color,
                            custom_size: Some(Vec2::new(size_visual.x, size_visual.y)),
                            ..default()
                        };

                        (rigid_body, collider, sprite)
                    }
                    ObstacleShape::Circle => {
                        let radius_physics = cmd.obstacle.size[0];
                        let radius_visual = radius_physics * scale;

                        let rigid_body = RigidBodyBuilder::fixed().translation(pos_physics).build();
                        let collider = ColliderBuilder::ball(radius_physics).build();
                        let sprite = Sprite {
                            color: obstacle_color,
                            custom_size: Some(Vec2::new(radius_visual * 2.0, radius_visual * 2.0)),
                            ..default()
                        };

                        (rigid_body, collider, sprite)
                    }
                };

                // Insert into physics world
                let PhysicsWorld {
                    ref mut rigid_body_set,
                    ref mut collider_set,
                    ..
                } = *physics_world;

                let handle = rigid_body_set.insert(rigid_body);
                collider_set.insert_with_parent(collider, handle, rigid_body_set);

                // Generate unique ID for this obstacle
                let obstacle_id = DynamicObstacleId(id_counter.0);
                id_counter.0 += 1;

                // Spawn visual entity
                commands.spawn((
                    sprite,
                    Transform::from_translation(Vec3::new(pos_visual.x, pos_visual.y, 0.5)),
                    WorldElement,
                    ObstacleElement,
                    obstacle_id,
                    PhysicsHandle {
                        rigid_body_handle: handle,
                    },
                ));

                info!("  Spawned dynamic obstacle with ID: {:?}", obstacle_id);
            }
            ObstacleAction::Remove => {
                // Find and remove obstacle at position
                let target_pos = cmd.obstacle.pos;
                let tolerance = 0.1; // 10cm tolerance

                // Destructure physics_world to avoid borrow checker issues
                let PhysicsWorld {
                    ref mut rigid_body_set,
                    ref mut collider_set,
                    ref mut island_manager,
                    ref mut impulse_joint_set,
                    ref mut multibody_joint_set,
                    ..
                } = *physics_world;

                for (entity, _id, physics_handle) in obstacle_entities.iter() {
                    if let Some(rb) = rigid_body_set.get(physics_handle.rigid_body_handle) {
                        let rb_pos = rb.translation();
                        let distance = ((rb_pos.x - target_pos[0]).powi(2)
                            + (rb_pos.y - target_pos[1]).powi(2))
                        .sqrt();

                        if distance < tolerance {
                            info!("  Removing dynamic obstacle at {:?}", target_pos);

                            // Remove from physics world
                            rigid_body_set.remove(
                                physics_handle.rigid_body_handle,
                                island_manager,
                                collider_set,
                                impulse_joint_set,
                                multibody_joint_set,
                                true,
                            );

                            // Despawn visual entity
                            commands.entity(entity).despawn();
                            break;
                        }
                    }
                }
            }
        }
    }
}

/// Keyboard input system - handles keyboard shortcuts
pub fn keyboard_input_system(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut ui_state: ResMut<ui::UiState>,
) {
    // Space - toggle pause/play
    if keyboard_input.just_pressed(KeyCode::Space) {
        ui_state.paused = !ui_state.paused;
        ui_state.status_message = if ui_state.paused {
            "Simulation paused (Space)".to_string()
        } else {
            "Simulation resumed (Space)".to_string()
        };
    }

    // R - reset simulation
    if keyboard_input.just_pressed(KeyCode::KeyR) {
        ui_state.reset_simulation = true;
        ui_state.status_message = "Resetting simulation (R)...".to_string();
    }
}

/// Reset simulation system - resets robot position and clears trajectory
pub fn reset_system(
    mut ui_state: ResMut<ui::UiState>,
    mut physics_world: ResMut<PhysicsWorld>,
    robot_query: Query<(&Robot, &Transform)>,
    mut trajectory: ResMut<TrajectoryHistory>,
    _app_config: Res<AppConfig>,
) {
    if !ui_state.reset_simulation {
        return;
    }

    // Clear the reset flag
    ui_state.reset_simulation = false;

    // Reset robot position
    for (robot, _) in robot_query.iter() {
        if let Some(rigid_body) = physics_world
            .rigid_body_set
            .get_mut(robot.rigid_body_handle)
        {
            // Get initial position from config
            let initial_pos = robot.config.position;

            // Reset position and rotation
            rigid_body.set_translation(vector![initial_pos[0], initial_pos[1]], true);
            rigid_body.set_rotation(Rotation::new(0.0), true);

            // Reset velocity
            rigid_body.set_linvel(vector![0.0, 0.0], true);
            rigid_body.set_angvel(0.0, true);

            info!(
                "Reset robot to initial position: ({}, {})",
                initial_pos[0], initial_pos[1]
            );
        }
    }

    // Clear trajectory
    trajectory.points.clear();

    ui_state.status_message = "Simulation reset successfully".to_string();
}

/// World reload system - detects config changes and reloads world
pub fn world_reload_system(
    mut commands: Commands,
    app_config: Res<AppConfig>,
    mut physics_world: ResMut<PhysicsWorld>,
    world_entities: Query<
        (Entity, &PhysicsHandle),
        Or<(With<WorldElement>, With<ObstacleElement>)>,
    >,
    visual_prefs: Res<ui::VisualPreferences>,
) {
    // Only reload if world config changed
    if !app_config.is_changed() {
        return;
    }

    info!("Reloading world...");

    // Destructure physics_world to avoid borrow checker issues
    let PhysicsWorld {
        ref mut rigid_body_set,
        ref mut collider_set,
        ref mut island_manager,
        ref mut impulse_joint_set,
        ref mut multibody_joint_set,
        ..
    } = *physics_world;

    // 1. Despawn all existing world entities (visual)
    for (entity, physics_handle) in world_entities.iter() {
        // Remove physics body
        rigid_body_set.remove(
            physics_handle.rigid_body_handle,
            island_manager,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            true,
        );
        // Remove Bevy entity
        commands.entity(entity).despawn();
    }

    // 2. Recreate world with new config
    let scale = 50.0;
    let world_half_width = app_config.world_config.width / 2.0 * scale;
    let world_half_height = app_config.world_config.height / 2.0 * scale;

    let boundaries = [
        // Bottom, Top, Left, Right walls
        (
            vector![0.0, -world_half_height],
            vector![app_config.world_config.width * scale, 0.2 * scale],
        ),
        (
            vector![0.0, world_half_height],
            vector![app_config.world_config.width * scale, 0.2 * scale],
        ),
        (
            vector![-world_half_width, 0.0],
            vector![0.2 * scale, app_config.world_config.height * scale],
        ),
        (
            vector![world_half_width, 0.0],
            vector![0.2 * scale, app_config.world_config.height * scale],
        ),
    ];

    let boundaries_physics = [
        // Bottom, Top, Left, Right walls (original scale for physics)
        (
            vector![0.0, -app_config.world_config.height / 2.0],
            vector![app_config.world_config.width, 0.2],
        ),
        (
            vector![0.0, app_config.world_config.height / 2.0],
            vector![app_config.world_config.width, 0.2],
        ),
        (
            vector![-app_config.world_config.width / 2.0, 0.0],
            vector![0.2, app_config.world_config.height],
        ),
        (
            vector![app_config.world_config.width / 2.0, 0.0],
            vector![0.2, app_config.world_config.height],
        ),
    ];

    for ((pos, size), (pos_scaled, size_scaled)) in boundaries_physics.iter().zip(boundaries.iter())
    {
        // Physics (original scale)
        let rigid_body = RigidBodyBuilder::fixed().translation(*pos).build();
        let collider = ColliderBuilder::cuboid(size.x / 2.0, size.y / 2.0).build();
        let handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(collider, handle, rigid_body_set);

        // Visual (scaled for visibility)
        commands.spawn((
            Sprite {
                color: Color::srgb(
                    visual_prefs.wall_color[0],
                    visual_prefs.wall_color[1],
                    visual_prefs.wall_color[2],
                ),
                custom_size: Some(Vec2::new(size_scaled.x, size_scaled.y)),
                ..default()
            },
            Transform::from_translation(Vec3::new(pos_scaled.x, pos_scaled.y, 0.0)),
            WorldElement,
            PhysicsHandle {
                rigid_body_handle: handle,
            },
        ));
    }

    // Create obstacles
    for obstacle in &app_config.world_config.obstacles {
        let pos_physics = vector![obstacle.pos[0], obstacle.pos[1]];
        let pos_visual = vector![obstacle.pos[0] * scale, obstacle.pos[1] * scale];

        // Determine obstacle color (custom or visual preferences default)
        let obstacle_color = obstacle
            .color
            .map(|c| Color::srgb(c[0], c[1], c[2]))
            .unwrap_or(Color::srgb(
                visual_prefs.obstacle_color[0],
                visual_prefs.obstacle_color[1],
                visual_prefs.obstacle_color[2],
            ));

        // Create physics body and visual representation based on shape
        let (rigid_body, collider, sprite) = match obstacle.shape {
            ObstacleShape::Rectangle => {
                let size_physics = vector![obstacle.size[0], obstacle.size[1]];
                let size_visual = vector![obstacle.size[0] * scale, obstacle.size[1] * scale];

                let rigid_body = RigidBodyBuilder::fixed().translation(pos_physics).build();
                let collider =
                    ColliderBuilder::cuboid(size_physics.x / 2.0, size_physics.y / 2.0).build();
                let sprite = Sprite {
                    color: obstacle_color,
                    custom_size: Some(Vec2::new(size_visual.x, size_visual.y)),
                    ..default()
                };

                (rigid_body, collider, sprite)
            }
            ObstacleShape::Circle => {
                let radius_physics = obstacle.size[0]; // Use first element as radius
                let radius_visual = radius_physics * scale;

                let rigid_body = RigidBodyBuilder::fixed().translation(pos_physics).build();
                let collider = ColliderBuilder::ball(radius_physics).build();
                let sprite = Sprite {
                    color: obstacle_color,
                    custom_size: Some(Vec2::new(radius_visual * 2.0, radius_visual * 2.0)),
                    ..default()
                };

                (rigid_body, collider, sprite)
            }
        };

        let handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(collider, handle, rigid_body_set);

        // Visual (scaled)
        commands.spawn((
            sprite,
            Transform::from_translation(Vec3::new(pos_visual.x, pos_visual.y, 0.5)),
            WorldElement,
            ObstacleElement,
            PhysicsHandle {
                rigid_body_handle: handle,
            },
        ));
    }

    info!(
        " World reloaded: {}x{}m with {} obstacles",
        app_config.world_config.width,
        app_config.world_config.height,
        app_config.world_config.obstacles.len()
    );
}

/// Editor input system - handles mouse input for world editing
pub fn editor_input_system(
    mut commands: Commands,
    mut editor: ResMut<crate::editor::WorldEditor>,
    mut app_config: ResMut<AppConfig>,
    mut physics_world: ResMut<PhysicsWorld>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    camera_controller: Res<ui::CameraController>,
    visual_prefs: Res<ui::VisualPreferences>,
    obstacle_query: Query<(Entity, &PhysicsHandle, &ObstacleElement)>,
    mut obstacle_id_counter: ResMut<ObstacleIdCounter>,
) {
    // Only process if editor is enabled
    if !editor.enabled {
        return;
    }

    // Get window and camera
    let Ok(window) = windows.get_single() else {
        return;
    };
    let Ok((camera, camera_transform)) = camera_query.get_single() else {
        return;
    };

    // Get cursor position in world coordinates
    let Some(cursor_pos) = window.cursor_position() else {
        return;
    };

    // Convert screen to world coordinates
    let Ok(world_pos) = camera.viewport_to_world_2d(camera_transform, cursor_pos) else {
        return;
    };

    // Apply camera controller transforms
    let scale = 50.0; // Visual scale factor
    let world_x = (world_pos.x - camera_controller.pan_x) / (scale * camera_controller.zoom);
    let world_y = (world_pos.y - camera_controller.pan_y) / (scale * camera_controller.zoom);
    let world_pos_adjusted = Vec2::new(world_x, world_y);

    // Update editor with current mouse position
    editor.mouse_world_pos = world_pos_adjusted;

    // Handle mouse input
    if mouse_button_input.just_pressed(MouseButton::Left) {
        match editor.active_tool {
            crate::editor::EditorTool::Select => {
                // For select tool: check if clicking on a selected obstacle to start drag
                let obstacles: Vec<_> = app_config.world_config.obstacles.clone();
                let entities: Vec<_> = obstacle_query.iter().map(|(e, _, _)| e).collect();

                // Check if clicking on an already selected obstacle
                if let Some(idx) = editor.find_obstacle_at(world_pos_adjusted, &obstacles) {
                    if idx < entities.len() {
                        let clicked_entity = entities[idx];
                        if editor.is_selected(clicked_entity) {
                            // Start dragging the selected obstacles
                            editor.start_drag(world_pos_adjusted, &obstacles, &entities);
                        } else {
                            // Select this obstacle instead
                            editor.clear_selection();
                            editor.select_obstacle(clicked_entity);
                            // And start dragging it
                            let obstacles_after: Vec<_> = app_config.world_config.obstacles.clone();
                            let entities_after: Vec<_> =
                                obstacle_query.iter().map(|(e, _, _)| e).collect();
                            editor.start_drag(
                                world_pos_adjusted,
                                &obstacles_after,
                                &entities_after,
                            );
                        }
                    }
                } else {
                    // Clicked on empty space - clear selection
                    editor.clear_selection();
                }
            }
            _ => {
                editor.handle_mouse_press(world_pos_adjusted);
            }
        }
    }

    if mouse_button_input.pressed(MouseButton::Left) {
        match editor.active_tool {
            crate::editor::EditorTool::Select => {
                // Handle dragging: update positions in real-time
                if editor.is_dragging() {
                    if let Some(offset) = editor.get_drag_offset(world_pos_adjusted) {
                        // Update visual positions of selected obstacles during drag
                        for (i, selected_entity) in editor.selected_obstacles.iter().enumerate() {
                            if i < editor.drag_original_positions.len() {
                                let orig_pos = editor.drag_original_positions[i];
                                let new_x = orig_pos[0] + offset.x;
                                let new_y = orig_pos[1] + offset.y;

                                // Update transform for visual feedback
                                for (entity, physics_handle, _) in obstacle_query.iter() {
                                    if entity == *selected_entity {
                                        // Update physics body position for real-time feedback
                                        if let Some(rb) = physics_world
                                            .rigid_body_set
                                            .get_mut(physics_handle.rigid_body_handle)
                                        {
                                            rb.set_translation(
                                                rapier2d::na::Vector2::new(new_x, new_y),
                                                true,
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            _ => {
                editor.handle_mouse_drag(world_pos_adjusted);
            }
        }
    }

    if mouse_button_input.just_released(MouseButton::Left) {
        match editor.active_tool {
            crate::editor::EditorTool::Rectangle | crate::editor::EditorTool::Circle => {
                if let Some(obstacle) = editor.handle_mouse_release() {
                    // Add obstacle to world config
                    app_config.world_config.obstacles.push(obstacle.clone());

                    // Spawn physics and visual for new obstacle
                    obstacle_id_counter.0 += 1;
                    spawn_obstacle(&mut commands, &mut physics_world, &obstacle, &visual_prefs);

                    // Record undo action
                    editor.push_undo(crate::editor::EditorAction::AddObstacle {
                        obstacle,
                        entity: None,
                    });
                }
            }
            crate::editor::EditorTool::Select => {
                // Finish drag operation if dragging
                if editor.is_dragging() {
                    if let Some(offset) = editor.end_drag() {
                        // Only record undo if actually moved
                        if offset.x.abs() > 0.01 || offset.y.abs() > 0.01 {
                            // Update world config obstacles with final positions
                            let entities: Vec<_> =
                                obstacle_query.iter().map(|(e, _, _)| e).collect();

                            for selected_entity in &editor.selected_obstacles.clone() {
                                if let Some(idx) =
                                    entities.iter().position(|e| e == selected_entity)
                                {
                                    if idx < app_config.world_config.obstacles.len() {
                                        let old_pos = app_config.world_config.obstacles[idx].pos;
                                        let new_pos =
                                            [old_pos[0] + offset.x, old_pos[1] + offset.y];

                                        // Record undo action
                                        editor.push_undo(
                                            crate::editor::EditorAction::MoveObstacle {
                                                entity: *selected_entity,
                                                old_pos,
                                                new_pos,
                                            },
                                        );

                                        // Update world config
                                        app_config.world_config.obstacles[idx].pos = new_pos;
                                    }
                                }
                            }
                        }
                    }
                } else {
                    // Simple click without drag - just select
                    let obstacles: Vec<_> = app_config.world_config.obstacles.clone();
                    let entities: Vec<_> = obstacle_query.iter().map(|(e, _, _)| e).collect();
                    editor.try_select_at(world_pos_adjusted, &obstacles, &entities);
                }
            }
            crate::editor::EditorTool::Delete => {
                editor.handle_mouse_release();

                // Find obstacle at click position
                if let Some(idx) =
                    editor.try_delete_at(world_pos_adjusted, &app_config.world_config.obstacles)
                {
                    // Record undo action
                    let removed = app_config.world_config.obstacles.remove(idx);

                    // Find and remove corresponding entity
                    let entity_list: Vec<_> = obstacle_query.iter().collect();
                    if idx < entity_list.len() {
                        let (entity, physics_handle, _) = entity_list[idx];

                        // Destructure to avoid borrow checker issues
                        let PhysicsWorld {
                            ref mut rigid_body_set,
                            ref mut collider_set,
                            ref mut island_manager,
                            ref mut impulse_joint_set,
                            ref mut multibody_joint_set,
                            ..
                        } = *physics_world;

                        // Remove from physics
                        rigid_body_set.remove(
                            physics_handle.rigid_body_handle,
                            island_manager,
                            collider_set,
                            impulse_joint_set,
                            multibody_joint_set,
                            true,
                        );

                        // Despawn entity
                        commands.entity(entity).despawn();

                        editor.push_undo(crate::editor::EditorAction::RemoveObstacle {
                            obstacle: removed,
                            entity,
                        });
                    }
                }
            }
        }
    }
}

/// Spawn a single obstacle with physics and visuals
fn spawn_obstacle(
    commands: &mut Commands,
    physics_world: &mut PhysicsWorld,
    obstacle: &Obstacle,
    visual_prefs: &ui::VisualPreferences,
) {
    let scale = 50.0;
    let color = obstacle.color.unwrap_or(visual_prefs.obstacle_color);

    // Create physics body
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![obstacle.pos[0], obstacle.pos[1]])
        .build();
    let handle = physics_world.rigid_body_set.insert(rigid_body);

    // Create collider based on shape
    let collider = match obstacle.shape {
        ObstacleShape::Rectangle => {
            ColliderBuilder::cuboid(obstacle.size[0] / 2.0, obstacle.size[1] / 2.0).build()
        }
        ObstacleShape::Circle => ColliderBuilder::ball(obstacle.size[0] / 2.0).build(),
    };
    physics_world.collider_set.insert_with_parent(
        collider,
        handle,
        &mut physics_world.rigid_body_set,
    );

    // Create visual
    commands.spawn((
        Sprite {
            color: Color::srgb(color[0], color[1], color[2]),
            custom_size: Some(Vec2::new(
                obstacle.size[0] * scale,
                obstacle.size[1] * scale,
            )),
            ..default()
        },
        Transform::from_xyz(obstacle.pos[0] * scale, obstacle.pos[1] * scale, 1.0),
        ObstacleElement,
        PhysicsHandle {
            rigid_body_handle: handle,
        },
    ));
}

/// Editor preview system - renders obstacle being created
pub fn editor_preview_system(editor: Res<crate::editor::WorldEditor>, mut gizmos: Gizmos) {
    if !editor.enabled {
        return;
    }

    let scale = 50.0;

    // Draw preview of obstacle being created
    if let Some(preview) = editor.get_preview_obstacle() {
        let pos = Vec2::new(preview.pos[0] * scale, preview.pos[1] * scale);
        let color = Color::srgba(0.5, 0.8, 0.5, 0.5); // Semi-transparent green

        match preview.shape {
            ObstacleShape::Rectangle => {
                let half_size =
                    Vec2::new(preview.size[0] * scale / 2.0, preview.size[1] * scale / 2.0);
                gizmos.rect_2d(Isometry2d::from_translation(pos), half_size * 2.0, color);
            }
            ObstacleShape::Circle => {
                let radius = preview.size[0] * scale / 2.0;
                gizmos.circle_2d(Isometry2d::from_translation(pos), radius, color);
            }
        }
    }

    // Draw selection highlights
    // (This would require obstacle positions which we don't have access to here)
}

/// Run the simulation with full CLI argument support
pub fn run_simulation(args: Args) -> Result<()> {
    use bevy::render::batching::gpu_preprocessing::GpuPreprocessingSupport;
    use bevy::render::view::window::screenshot::CapturedScreenshots;
    use bevy::render::RenderApp;
    use std::sync::mpsc::channel;
    use std::sync::{Arc, Mutex};

    let app_config = AppConfig::new(args.clone());
    let headless = args.headless;

    let mut app = bevy::prelude::App::new();

    if headless {
        // Headless mode - minimal plugins, no rendering
        app.add_plugins(bevy::prelude::MinimalPlugins)
            .insert_resource(app_config)
            .insert_resource(PhysicsWorld::default())
            .insert_resource(ui::UiState::default())
            .insert_resource(ui::VisualPreferences::default())
            .insert_resource(ui::RobotTelemetry::default())
            .insert_resource(crate::editor::WorldEditor::new())
            .insert_resource(crate::metrics::PerformanceMetrics::default())
            .insert_resource(LastLidarScan::default())
            .insert_resource(CameraSensors::default())
            .insert_resource(GpsSensors::default())
            .insert_resource(UltrasonicSensors::default())
            .insert_resource(ContactSensors::default())
            .insert_resource(PreviousVelocity::default())
            .insert_resource(ObstacleIdCounter::default())
            .add_systems(bevy::prelude::Startup, setup)
            .add_systems(bevy::prelude::Update, tick_start_system)
            .add_systems(
                bevy::prelude::Update,
                (
                    horus_system,
                    physics_system,
                    telemetry_system,
                    odometry_publish_system,
                    imu_system,
                    lidar_system,
                    camera_system,
                    gps_system,
                    ultrasonic_system,
                    contact_system,
                    dynamic_obstacle_system,
                )
                    .after(tick_start_system),
            )
            // Articulated robot systems
            .add_systems(
                bevy::prelude::Update,
                (
                    crate::joint::articulated_visual_sync_system,
                    crate::joint::joint_state_update_system,
                    crate::joint::joint_command_system,
                    crate::joint::joint_state_publish_system,
                ),
            );
    } else {
        // GUI mode - full visualization
        app.add_plugins(
            bevy::prelude::DefaultPlugins
                .set(bevy::window::WindowPlugin {
                    primary_window: Some(bevy::window::Window {
                        title: "sim2d - 2D Robotics Simulator".to_string(),
                        resolution: (1600.0, 900.0).into(),
                        ..bevy::prelude::default()
                    }),
                    ..bevy::prelude::default()
                })
                .disable::<bevy::log::LogPlugin>()
                .disable::<bevy::render::pipelined_rendering::PipelinedRenderingPlugin>(),
        )
        .add_plugins(bevy_egui::EguiPlugin)
        .insert_resource({
            let (_, rx) = channel();
            CapturedScreenshots(Arc::new(Mutex::new(rx)))
        })
        .insert_resource(app_config)
        .insert_resource(PhysicsWorld::default())
        .insert_resource(ui::UiState::default())
        .insert_resource(ui::VisualPreferences::default())
        .insert_resource(ui::CameraController::default())
        .insert_resource(ui::RobotTelemetry::default())
        .insert_resource(ui::FrameMetrics::default())
        .insert_resource(crate::recorder::Recorder::default())
        .insert_resource(crate::editor::WorldEditor::new())
        .insert_resource(crate::metrics::PerformanceMetrics::default())
        .insert_resource(TrajectoryHistory::default())
        .insert_resource(LastLidarScan::default())
        .insert_resource(CameraSensors::default())
        .insert_resource(GpsSensors::default())
        .insert_resource(UltrasonicSensors::default())
        .insert_resource(ContactSensors::default())
        .insert_resource(PreviousVelocity::default())
        .insert_resource(CollisionState::default())
        .insert_resource(ObstacleIdCounter::default())
        .add_systems(bevy::prelude::Startup, setup)
        .add_systems(bevy::prelude::Update, tick_start_system)
        .add_systems(
            bevy::prelude::Update,
            (
                horus_system,
                physics_system,
                telemetry_system,
                visual_sync_system,
                visual_component_sync_system,
            )
                .after(tick_start_system),
        )
        .add_systems(
            bevy::prelude::Update,
            (
                odometry_publish_system,
                imu_system,
                lidar_system,
                camera_system,
                gps_system,
                ultrasonic_system,
                contact_system,
            )
                .after(tick_start_system),
        )
        .add_systems(
            bevy::prelude::Update,
            (
                trajectory_system,
                velocity_arrow_system,
                collision_detection_system,
                collision_indicator_system,
                dynamic_obstacle_system,
                metrics_update_system,
            ),
        )
        .add_systems(
            bevy::prelude::Update,
            (
                camera_control_system,
                mouse_camera_system,
                keyboard_input_system,
                reset_system,
                robot_visual_reload_system,
                world_reload_system,
            ),
        )
        .add_systems(
            bevy::prelude::Update,
            (visual_color_system, grid_system, lidar_rays_system),
        )
        .add_systems(
            bevy::prelude::Update,
            (editor_input_system, editor_preview_system),
        )
        // Articulated robot systems
        .add_systems(
            bevy::prelude::Update,
            (
                crate::joint::articulated_visual_sync_system,
                crate::joint::joint_marker_sync_system,
                crate::joint::joint_state_update_system,
                crate::joint::joint_command_system,
                crate::joint::joint_state_publish_system,
            ),
        )
        .add_systems(bevy::prelude::Update, ui::ui_system)
        .add_systems(
            bevy::prelude::Update,
            topic_update_system.after(ui::ui_system),
        )
        .add_systems(bevy::prelude::Update, ui::file_dialog_system)
        .add_systems(
            bevy::prelude::Update,
            tick_end_system.after(ui::file_dialog_system),
        );

        // Initialize GpuPreprocessingSupport in RenderApp
        if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app.insert_resource(GpuPreprocessingSupport::None);
        }
    }

    app.run();
    Ok(())
}
