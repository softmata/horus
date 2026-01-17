use crate::{MotorCommand, PidConfig};
use horus_core::error::HorusResult;

// Import algorithms from horus_library/algorithms
use crate::algorithms::pid::PID;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

/// PID Controller Node - Generic PID control implementation
///
/// Implements a PID controller that can be used for various control applications.
/// Subscribes to setpoint and feedback values, publishes control output.
///
/// This node is a thin wrapper around the pure algorithms in horus_library/algorithms.
pub struct PidControllerNode {
    // Publishers and Subscribers
    output_publisher: Topic<MotorCommand>,
    setpoint_subscriber: Topic<f32>,
    feedback_subscriber: Topic<f32>,
    config_subscriber: Topic<PidConfig>,

    // Algorithm instance
    pid: PID,

    // Node state
    setpoint: f32,
    feedback: f32,
    last_time: u64,
    is_initialized: bool,
    motor_id: u8,
}

impl PidControllerNode {
    /// Create a new PID controller node with default topics
    pub fn new() -> Result<Self> {
        Self::new_with_topics("setpoint", "feedback", "pid_output", "pid_config")
    }

    /// Create a new PID controller node with custom topics
    pub fn new_with_topics(
        setpoint_topic: &str,
        feedback_topic: &str,
        output_topic: &str,
        config_topic: &str,
    ) -> Result<Self> {
        // Create PID instance with default gains
        let mut pid = PID::new(1.0, 0.1, 0.05);

        // Set default limits
        pid.set_output_limits(-100.0, 100.0);
        pid.set_integral_limits(-50.0, 50.0);
        pid.set_deadband(0.01);

        Ok(Self {
            output_publisher: Topic::new(output_topic)?,
            setpoint_subscriber: Topic::new(setpoint_topic)?,
            feedback_subscriber: Topic::new(feedback_topic)?,
            config_subscriber: Topic::new(config_topic)?,

            pid,

            setpoint: 0.0,
            feedback: 0.0,
            last_time: 0,
            is_initialized: false,
            motor_id: 0,
        })
    }

    /// Set PID gains
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.pid.set_gains(kp as f64, ki as f64, kd as f64);
    }

    /// Set output limits
    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.pid.set_output_limits(min as f64, max as f64);
    }

    /// Set integral limits (anti-windup)
    pub fn set_integral_limits(&mut self, min: f32, max: f32) {
        self.pid.set_integral_limits(min as f64, max as f64);
    }

    /// Set deadband (minimum error threshold)
    pub fn set_deadband(&mut self, deadband: f32) {
        self.pid.set_deadband(deadband.abs() as f64);
    }

    /// Set motor ID for output commands
    pub fn set_motor_id(&mut self, motor_id: u8) {
        self.motor_id = motor_id;
    }

    /// Reset PID controller state
    pub fn reset(&mut self) {
        self.pid.reset();
        self.last_time = 0;
    }

    /// Get current PID state
    pub fn get_state(&self) -> (f32, f32, f32, f32) {
        let (last_error, integral) = self.pid.get_state();
        (
            self.setpoint,
            self.feedback,
            last_error as f32,
            integral as f32,
        )
    }

    fn calculate_pid_output(&mut self, dt: f32) -> f32 {
        // Use PID algorithm to compute output
        let output = self
            .pid
            .compute(self.setpoint as f64, self.feedback as f64, dt as f64);
        output as f32
    }

    fn publish_output(&self, output: f32) {
        let _current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let motor_cmd = MotorCommand::velocity(self.motor_id, output as f64);

        let _ = self.output_publisher.send(motor_cmd, &mut None);
    }
}

impl Node for PidControllerNode {
    fn name(&self) -> &'static str {
        "PidControllerNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("PidControllerNode shutting down - sending zero output");

        // Reset PID state and send zero output to motor
        self.reset();
        self.publish_output(0.0);

        ctx.log_info("PID controller stopped with zero output");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Calculate delta time
        let dt = if self.last_time > 0 {
            (current_time - self.last_time) as f32 / 1000.0
        } else {
            0.01 // 10ms default
        };
        self.last_time = current_time;

        // Check for new setpoint
        if let Some(new_setpoint) = self.setpoint_subscriber.recv(&mut None) {
            self.setpoint = new_setpoint;
        }

        // Check for new feedback
        if let Some(new_feedback) = self.feedback_subscriber.recv(&mut None) {
            self.feedback = new_feedback;
        }

        // Check for new PID configuration
        if let Some(config) = self.config_subscriber.recv(&mut None) {
            self.pid.set_gains(config.kp, config.ki, config.kd);
        }

        // Calculate and publish PID output
        if self.is_initialized || (self.setpoint != 0.0 || self.feedback != 0.0) {
            let output = self.calculate_pid_output(dt);
            self.publish_output(output);
            self.is_initialized = true;
        }
    }
}

// Default impl removed - use PidControllerNode::new() instead which returns HorusResult
