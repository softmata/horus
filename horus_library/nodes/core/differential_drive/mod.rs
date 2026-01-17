use crate::{DifferentialDriveCommand, Odometry, Twist};
use horus_core::error::HorusResult;

// Import algorithms from horus_library/algorithms
use crate::algorithms::differential_drive::DifferentialDrive;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

/// Differential Drive Node - Mobile robot base controller
///
/// Subscribes to Twist velocity commands and converts them to differential drive
/// motor commands. Also publishes odometry based on wheel encoder feedback.
///
/// This node is a thin wrapper around the pure algorithms in horus_library/algorithms.
pub struct DifferentialDriveNode {
    // Publishers
    drive_publisher: Topic<DifferentialDriveCommand>,
    odom_publisher: Topic<Odometry>,

    // Subscribers
    cmd_subscriber: Topic<Twist>,

    // Algorithm instance
    diff_drive: DifferentialDrive,

    // Configuration
    max_linear_vel: f32,  // Max linear velocity (m/s)
    max_angular_vel: f32, // Max angular velocity (rad/s)

    // State
    current_twist: Twist,
    position_x: f64,
    position_y: f64,
    orientation: f64,
    last_update_time: u64,
}

impl DifferentialDriveNode {
    /// Create a new differential drive node with default topics
    pub fn new() -> Result<Self> {
        Self::new_with_topics("cmd_vel", "drive_command", "odom")
    }

    /// Create a new differential drive node with custom topics
    pub fn new_with_topics(cmd_topic: &str, drive_topic: &str, odom_topic: &str) -> Result<Self> {
        // Create differential drive algorithm instance with default parameters
        let diff_drive = DifferentialDrive::new(0.5, 0.1); // 50cm wheel base, 10cm wheel radius

        Ok(Self {
            drive_publisher: Topic::new(drive_topic)?,
            odom_publisher: Topic::new(odom_topic)?,
            cmd_subscriber: Topic::new(cmd_topic)?,

            diff_drive,

            max_linear_vel: 2.0,                   // 2 m/s max
            max_angular_vel: std::f32::consts::PI, // π rad/s max

            current_twist: Twist::default(),
            position_x: 0.0,
            position_y: 0.0,
            orientation: 0.0,
            last_update_time: 0,
        })
    }

    /// Set wheel base (distance between wheels in meters)
    pub fn set_wheel_base(&mut self, wheel_base: f32) {
        let wheel_base = wheel_base.max(0.1);
        self.diff_drive =
            DifferentialDrive::new(wheel_base as f64, self.diff_drive.get_wheel_radius());
    }

    /// Set wheel radius (in meters)
    pub fn set_wheel_radius(&mut self, radius: f32) {
        let radius = radius.max(0.01);
        self.diff_drive = DifferentialDrive::new(self.diff_drive.get_wheel_base(), radius as f64);
    }

    /// Set maximum velocities
    pub fn set_velocity_limits(&mut self, max_linear: f32, max_angular: f32) {
        self.max_linear_vel = max_linear.max(0.1);
        self.max_angular_vel = max_angular.max(0.1);
    }

    /// Reset odometry to origin
    pub fn reset_odometry(&mut self) {
        self.position_x = 0.0;
        self.position_y = 0.0;
        self.orientation = 0.0;
    }

    /// Get current position
    pub fn get_position(&self) -> (f64, f64, f64) {
        (self.position_x, self.position_y, self.orientation)
    }

    fn clamp_twist(&self, mut twist: Twist) -> Twist {
        // Clamp linear velocity
        twist.linear[0] =
            twist.linear[0].clamp(-self.max_linear_vel as f64, self.max_linear_vel as f64);
        twist.linear[1] = 0.0; // Differential drive can't move sideways
        twist.linear[2] = 0.0;

        // Clamp angular velocity
        twist.angular[0] = 0.0;
        twist.angular[1] = 0.0;
        twist.angular[2] =
            twist.angular[2].clamp(-self.max_angular_vel as f64, self.max_angular_vel as f64);

        twist
    }

    fn twist_to_wheel_speeds(&self, twist: &Twist) -> (f32, f32) {
        // Use differential drive algorithm for inverse kinematics
        let linear_vel = twist.linear[0];
        let angular_vel = twist.angular[2];

        let (left_speed, right_speed) = self.diff_drive.inverse_kinematics(linear_vel, angular_vel);

        (left_speed as f32, right_speed as f32)
    }

    fn update_odometry(&mut self, dt: f32) {
        let linear_vel = self.current_twist.linear[0];
        let angular_vel = self.current_twist.angular[2];

        // Use differential drive algorithm for odometry update
        let (new_x, new_y, new_theta) = self.diff_drive.update_odometry(
            (self.position_x, self.position_y, self.orientation),
            linear_vel,
            angular_vel,
            dt as f64,
        );

        self.position_x = new_x;
        self.position_y = new_y;
        self.orientation = new_theta;
    }

    fn publish_drive_command(&self, left_speed: f32, right_speed: f32) {
        let cmd = DifferentialDriveCommand::new(left_speed as f64, right_speed as f64);
        let _ = self.drive_publisher.send(cmd, &mut None);
    }

    fn publish_odometry(&self) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let mut odom = Odometry::new();
        odom.pose.x = self.position_x;
        odom.pose.y = self.position_y;
        odom.pose.theta = self.orientation;
        odom.twist.linear[0] = self.current_twist.linear[0];
        odom.twist.angular[2] = self.current_twist.angular[2];

        // Set frame IDs
        let frame_id = "odom";
        let child_frame_id = "base_link";
        let frame_bytes = frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        odom.frame_id[..len].copy_from_slice(&frame_bytes[..len]);

        let child_frame_bytes = child_frame_id.as_bytes();
        let len = child_frame_bytes.len().min(31);
        odom.child_frame_id[..len].copy_from_slice(&child_frame_bytes[..len]);

        odom.timestamp = current_time;

        let _ = self.odom_publisher.send(odom, &mut None);
    }
}

impl Node for DifferentialDriveNode {
    fn name(&self) -> &'static str {
        "DifferentialDriveNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("DifferentialDriveNode shutting down - stopping drive motors");

        // Set all velocities to zero
        self.current_twist = Twist::default();

        // Publish stop command to motors
        self.publish_drive_command(0.0, 0.0);

        ctx.log_info("Differential drive motors stopped safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Calculate delta time
        let dt = if self.last_update_time > 0 {
            (current_time - self.last_update_time) as f32 / 1000.0
        } else {
            0.01 // 10ms default
        };
        self.last_update_time = current_time;

        // Check for new velocity commands
        if let Some(twist_cmd) = self.cmd_subscriber.recv(&mut None) {
            self.current_twist = self.clamp_twist(twist_cmd);
        }

        // Convert twist to wheel speeds and publish
        let (left_speed, right_speed) = self.twist_to_wheel_speeds(&self.current_twist);
        self.publish_drive_command(left_speed, right_speed);

        // Update and publish odometry
        self.update_odometry(dt);
        self.publish_odometry();
    }
}

// Default impl removed - use Node::new() instead which returns HorusResult
