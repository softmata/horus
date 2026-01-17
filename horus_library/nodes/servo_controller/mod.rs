use crate::{JointCommand, ServoCommand};
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

/// Servo Controller Node - Multi-servo control for robot arms and actuators
///
/// Controls multiple servo motors with position, velocity, and torque commands.
/// Supports both individual servo commands and coordinated joint control.
pub struct ServoControllerNode {
    servo_subscriber: Topic<ServoCommand>,
    joint_subscriber: Topic<JointCommand>,
    status_publisher: Topic<JointCommand>, // Publishes current servo states

    // Configuration
    servo_count: u8,
    position_limits: HashMap<u8, (f64, f64)>, // (min, max) for each servo
    velocity_limits: HashMap<u8, f64>,        // max velocity for each servo
    torque_limits: HashMap<u8, f64>,          // max torque for each servo

    // State
    current_positions: HashMap<u8, f64>,
    current_velocities: HashMap<u8, f64>,
    target_positions: HashMap<u8, f64>,
    last_update_time: u64,

    // Control parameters
    position_tolerance: f64,
    default_velocity: f64,
    interpolation_enabled: bool,
}

impl ServoControllerNode {
    /// Create a new servo controller node with default topics
    pub fn new() -> Result<Self> {
        Self::new_with_topics("servo_command", "joint_command", "joint_states")
    }

    /// Create a new servo controller node with custom topics
    pub fn new_with_topics(
        servo_topic: &str,
        joint_topic: &str,
        status_topic: &str,
    ) -> Result<Self> {
        Ok(Self {
            servo_subscriber: Topic::new(servo_topic)?,
            joint_subscriber: Topic::new(joint_topic)?,
            status_publisher: Topic::new(status_topic)?,

            servo_count: 6, // Default 6-DOF robot arm
            position_limits: HashMap::new(),
            velocity_limits: HashMap::new(),
            torque_limits: HashMap::new(),

            current_positions: HashMap::new(),
            current_velocities: HashMap::new(),
            target_positions: HashMap::new(),
            last_update_time: 0,

            position_tolerance: 0.01, // 1 degree tolerance
            default_velocity: 1.0,    // 1 rad/s default
            interpolation_enabled: true,
        })
    }

    /// Set number of servos to control
    pub fn set_servo_count(&mut self, count: u8) {
        self.servo_count = count;

        // Initialize servo state maps
        for i in 0..count {
            self.current_positions.insert(i, 0.0);
            self.current_velocities.insert(i, 0.0);
            self.target_positions.insert(i, 0.0);

            // Set default limits
            self.position_limits
                .insert(i, (-std::f64::consts::PI, std::f64::consts::PI)); // ±π radians
            self.velocity_limits.insert(i, 2.0); // 2 rad/s
            self.torque_limits.insert(i, 10.0); // 10 Nm
        }
    }

    /// Set position limits for a servo
    pub fn set_position_limits(&mut self, servo_id: u8, min: f64, max: f64) {
        self.position_limits.insert(servo_id, (min, max));
    }

    /// Set velocity limit for a servo
    pub fn set_velocity_limit(&mut self, servo_id: u8, max_velocity: f64) {
        self.velocity_limits.insert(servo_id, max_velocity);
    }

    /// Set torque limit for a servo
    pub fn set_torque_limit(&mut self, servo_id: u8, max_torque: f64) {
        self.torque_limits.insert(servo_id, max_torque);
    }

    /// Enable/disable position interpolation
    pub fn set_interpolation(&mut self, enabled: bool) {
        self.interpolation_enabled = enabled;
    }

    /// Get current position of a servo
    pub fn get_position(&self, servo_id: u8) -> Option<f64> {
        self.current_positions.get(&servo_id).copied()
    }

    /// Get all current positions
    pub fn get_all_positions(&self) -> Vec<f64> {
        (0..self.servo_count)
            .map(|i| self.current_positions.get(&i).copied().unwrap_or(0.0))
            .collect()
    }

    /// Check if servo is within position limits
    fn is_position_valid(&self, servo_id: u8, position: f64) -> bool {
        if let Some((min, max)) = self.position_limits.get(&servo_id) {
            position >= *min && position <= *max
        } else {
            true // No limits set
        }
    }

    /// Clamp position to limits
    fn clamp_position(&self, servo_id: u8, position: f64) -> f64 {
        if let Some((min, max)) = self.position_limits.get(&servo_id) {
            position.clamp(*min, *max)
        } else {
            position
        }
    }

    fn handle_servo_command(&mut self, command: ServoCommand) {
        if command.servo_id < self.servo_count {
            let clamped_position = self.clamp_position(command.servo_id, command.position as f64);

            if self.is_position_valid(command.servo_id, clamped_position) {
                self.target_positions
                    .insert(command.servo_id, clamped_position);
            }
        }
    }

    fn handle_joint_command(&mut self, command: JointCommand) {
        // Handle multi-joint command
        let joint_count = command.positions.len().min(self.servo_count as usize);

        for (i, &position) in command.positions.iter().take(joint_count).enumerate() {
            let servo_id = i as u8;
            let clamped_position = self.clamp_position(servo_id, position);

            if self.is_position_valid(servo_id, clamped_position) {
                self.target_positions.insert(servo_id, clamped_position);
            }
        }
    }

    fn update_servo_positions(&mut self, dt: f64) {
        for servo_id in 0..self.servo_count {
            if let (Some(&current), Some(&target)) = (
                self.current_positions.get(&servo_id),
                self.target_positions.get(&servo_id),
            ) {
                let position_error = target - current;

                if position_error.abs() > self.position_tolerance {
                    let max_velocity = self
                        .velocity_limits
                        .get(&servo_id)
                        .copied()
                        .unwrap_or(self.default_velocity);
                    let max_step = max_velocity * dt;

                    let position_step = if self.interpolation_enabled {
                        if position_error.abs() <= max_step {
                            position_error // Reach target exactly
                        } else {
                            position_error.signum() * max_step
                        }
                    } else {
                        position_error // Move directly to target
                    };

                    let new_position = current + position_step;
                    let velocity = position_step / dt;

                    self.current_positions.insert(servo_id, new_position);
                    self.current_velocities.insert(servo_id, velocity);
                }
            }
        }
    }

    fn publish_joint_states(&self) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Create joint state message with current positions
        let mut positions = [0.0f64; 16];
        let mut velocities = [0.0f64; 16];
        let efforts = [0.0f64; 16];
        let mut joint_names = [[0u8; 32]; 16];
        let modes = [0u8; 16]; // Position control mode

        for i in 0..self.servo_count.min(16) {
            positions[i as usize] = self.current_positions.get(&i).copied().unwrap_or(0.0);
            velocities[i as usize] = self.current_velocities.get(&i).copied().unwrap_or(0.0);

            // Set joint name
            let joint_name = format!("joint_{}", i);
            let name_bytes = joint_name.as_bytes();
            let len = name_bytes.len().min(31);
            joint_names[i as usize][..len].copy_from_slice(&name_bytes[..len]);
        }

        let joint_state = JointCommand {
            joint_names,
            joint_count: self.servo_count,
            positions,
            velocities,
            efforts,
            modes,
            timestamp: current_time,
        };

        let _ = self.status_publisher.send(joint_state, &mut None);
    }

    /// Move all servos to home position (0.0)
    pub fn move_to_home(&mut self) {
        for servo_id in 0..self.servo_count {
            self.target_positions.insert(servo_id, 0.0);
        }
    }

    /// Stop all servo motion
    pub fn stop_all(&mut self) {
        for servo_id in 0..self.servo_count {
            if let Some(&current) = self.current_positions.get(&servo_id) {
                self.target_positions.insert(servo_id, current);
            }
        }
    }
}

impl Node for ServoControllerNode {
    fn name(&self) -> &'static str {
        "ServoControllerNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("ServoControllerNode shutting down - stopping all servos");

        // Stop all servo motion immediately
        self.stop_all();

        // Set all velocities to zero
        for servo_id in 0..self.servo_count {
            self.current_velocities.insert(servo_id, 0.0);
        }

        ctx.log_info("All servos stopped safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Calculate delta time
        let dt = if self.last_update_time > 0 {
            (current_time - self.last_update_time) as f64 / 1000.0
        } else {
            0.01 // 10ms default
        };
        self.last_update_time = current_time;

        // Handle incoming servo commands
        if let Some(servo_cmd) = self.servo_subscriber.recv(&mut None) {
            self.handle_servo_command(servo_cmd);
        }

        // Handle incoming joint commands
        if let Some(joint_cmd) = self.joint_subscriber.recv(&mut None) {
            self.handle_joint_command(joint_cmd);
        }

        // Update servo positions based on targets
        self.update_servo_positions(dt);

        // Publish current joint states
        self.publish_joint_states();
    }
}

// Default impl removed - use ServoControllerNode::new() instead which returns HorusResult
