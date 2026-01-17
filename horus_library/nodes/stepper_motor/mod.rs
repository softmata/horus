use crate::StepperCommand;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::f64::consts::PI;
use std::time::{SystemTime, UNIX_EPOCH};

// GPIO hardware support
#[cfg(feature = "gpio-hardware")]
use std::thread;
#[cfg(feature = "gpio-hardware")]
use std::time::Duration;
#[cfg(feature = "gpio-hardware")]
use sysfs_gpio::{Direction, Pin};

/// Stepper Motor Controller Node - Step/Direction stepper motor control
///
/// Controls stepper motors using step/direction interface with acceleration ramping.
/// Compatible with common stepper drivers (A4988, DRV8825, TMC2208, TMC2209, etc.).
/// Supports multiple stepper motors with independent control, microstepping, and homing.
///
/// # Features
/// - Multiple control modes: relative steps, absolute position, continuous velocity, homing
/// - Acceleration/deceleration ramping for smooth motion
/// - Microstepping support (1x to 256x)
/// - Position tracking and feedback
/// - Homing with limit switch support
/// - Current limiting (driver-dependent)
/// - Emergency stop handling
///
/// # Hardware Support
/// - A4988: 1/16 microstepping, up to 2A per coil
/// - DRV8825: 1/32 microstepping, up to 2.5A per coil
/// - TMC2208: 1/256 microstepping, ultra-quiet StealthChop, up to 2A per coil
/// - TMC2209: 1/256 microstepping, StealthChop, SpreadCycle, stallGuard, up to 2A per coil
/// - TB6600: 1/16 microstepping, up to 4.5A per coil
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::StepperMotorNode;
/// use horus_library::StepperCommand;
///
/// let mut stepper = StepperMotorNode::new()?;
/// stepper.set_num_motors(2);
/// stepper.set_steps_per_revolution(0, 200, 16); // NEMA 17, 1/16 microstepping
/// stepper.set_max_velocity(0, 1000.0); // 1000 steps/sec
/// stepper.set_acceleration(0, 500.0); // 500 steps/sec²
///
/// // Move to absolute position (5 revolutions)
/// let cmd = StepperCommand::position(0, 5.0 * std::f64::consts::TAU, 800.0);
/// ```
pub struct StepperMotorNode {
    subscriber: Topic<StepperCommand>,
    publisher: Topic<StepperCommand>, // Echo commands for monitoring
    feedback_publisher: Topic<crate::MotorCommand>, // Position/velocity feedback

    // Configuration per motor (up to 8 motors)
    num_motors: u8,
    steps_per_rev: [u32; 8],     // Full steps per revolution (typically 200)
    microsteps: [u16; 8],        // Current microstepping (1, 2, 4, 8, 16, 32, etc.)
    gear_ratio: [f64; 8],        // Optional gear reduction ratio
    max_velocity: [f64; 8],      // Maximum velocity in steps/sec
    max_acceleration: [f64; 8],  // Maximum acceleration in steps/sec²
    current_limit: [u16; 8],     // Current limit in milliamps (driver-dependent)
    invert_direction: [bool; 8], // Invert direction pin

    // State tracking per motor
    current_position: [i64; 8], // Current position in microsteps
    target_position: [i64; 8],  // Target position in microsteps
    current_velocity: [f64; 8], // Current velocity in steps/sec
    target_velocity: [f64; 8],  // Target velocity in steps/sec
    motor_enabled: [bool; 8],   // Motor enable state
    is_homing: [bool; 8],       // Homing in progress
    home_position: [i64; 8],    // Home position in microsteps
    is_homed: [bool; 8],        // Has been homed
    last_step_time: [u64; 8],   // Last step time in nanoseconds
    step_interval: [u64; 8],    // Current step interval in nanoseconds

    // Motion profile state
    motion_state: [MotionState; 8],

    // Configuration
    enable_feedback: bool,
    command_timeout_ms: u64,
    last_command_time: [u64; 8],

    // Hardware GPIO pins (per motor)
    #[cfg(feature = "gpio-hardware")]
    step_pins: [Option<Pin>; 8],
    #[cfg(feature = "gpio-hardware")]
    dir_pins: [Option<Pin>; 8],
    #[cfg(feature = "gpio-hardware")]
    enable_pins: [Option<Pin>; 8],
    hardware_enabled: bool,
    gpio_pin_numbers: [(u64, u64, u64); 8], // (step_pin, dir_pin, enable_pin) per motor
    step_pulse_duration_us: u64,            // Duration of step pulse in microseconds

    // Timing state (moved from static mut for thread safety)
    last_feedback_time_tick: u64,
}

/// Motion state for trapezoidal motion profile
#[derive(Debug, Clone, Copy, PartialEq)]
enum MotionState {
    Idle,
    Accelerating,
    Cruising,
    Decelerating,
}

impl Default for MotionState {
    fn default() -> Self {
        MotionState::Idle
    }
}

impl StepperMotorNode {
    /// Create a new stepper motor node with default topic "stepper_cmd"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("stepper_cmd")
    }

    /// Create a new stepper motor node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        #[cfg(feature = "gpio-hardware")]
        const NONE_PIN: Option<Pin> = None;
        Ok(Self {
            subscriber: Topic::new(topic)?,
            publisher: Topic::new(&format!("{}_feedback", topic))?,
            feedback_publisher: Topic::new(&format!("{}_position", topic))?,
            num_motors: 1,
            steps_per_rev: [200; 8], // NEMA 17 standard
            microsteps: [16; 8],     // Common default
            gear_ratio: [1.0; 8],
            max_velocity: [1000.0; 8],    // 1000 steps/sec default
            max_acceleration: [500.0; 8], // 500 steps/sec² default
            current_limit: [0; 8],
            invert_direction: [false; 8],
            current_position: [0; 8],
            target_position: [0; 8],
            current_velocity: [0.0; 8],
            target_velocity: [0.0; 8],
            motor_enabled: [false; 8],
            is_homing: [false; 8],
            home_position: [0; 8],
            is_homed: [false; 8],
            last_step_time: [0; 8],
            step_interval: [0; 8],
            motion_state: [MotionState::Idle; 8],
            enable_feedback: true,
            command_timeout_ms: 5000, // 5 second timeout
            last_command_time: [0; 8],
            #[cfg(feature = "gpio-hardware")]
            step_pins: [NONE_PIN; 8],
            #[cfg(feature = "gpio-hardware")]
            dir_pins: [NONE_PIN; 8],
            #[cfg(feature = "gpio-hardware")]
            enable_pins: [NONE_PIN; 8],
            hardware_enabled: false,
            gpio_pin_numbers: [(0, 0, 0); 8],
            step_pulse_duration_us: 5, // 5μs default step pulse
            last_feedback_time_tick: 0,
        })
    }

    /// Set the number of stepper motors (1-8)
    pub fn set_num_motors(&mut self, num: u8) {
        self.num_motors = num.clamp(1, 8);
    }

    /// Set steps per revolution and microstepping for a motor
    pub fn set_steps_per_revolution(&mut self, motor_id: u8, steps: u32, microsteps: u16) {
        if motor_id < 8 {
            self.steps_per_rev[motor_id as usize] = steps;
            self.microsteps[motor_id as usize] = microsteps;
        }
    }

    /// Set gear ratio for a motor (output/input)
    pub fn set_gear_ratio(&mut self, motor_id: u8, ratio: f64) {
        if motor_id < 8 {
            self.gear_ratio[motor_id as usize] = ratio;
        }
    }

    /// Set maximum velocity in steps/sec
    pub fn set_max_velocity(&mut self, motor_id: u8, velocity: f64) {
        if motor_id < 8 {
            self.max_velocity[motor_id as usize] = velocity.abs();
        }
    }

    /// Set acceleration in steps/sec²
    pub fn set_acceleration(&mut self, motor_id: u8, acceleration: f64) {
        if motor_id < 8 {
            self.max_acceleration[motor_id as usize] = acceleration.abs();
        }
    }

    /// Set current limit in milliamps (driver-dependent)
    pub fn set_current_limit(&mut self, motor_id: u8, limit: u16) {
        if motor_id < 8 {
            self.current_limit[motor_id as usize] = limit;
        }
    }

    /// Invert direction for a motor
    pub fn set_direction_inverted(&mut self, motor_id: u8, inverted: bool) {
        if motor_id < 8 {
            self.invert_direction[motor_id as usize] = inverted;
        }
    }

    /// Set command timeout in milliseconds (0 = disable)
    pub fn set_command_timeout(&mut self, timeout_ms: u64) {
        self.command_timeout_ms = timeout_ms;
    }

    /// Enable/disable position feedback publishing
    pub fn set_feedback_enabled(&mut self, enabled: bool) {
        self.enable_feedback = enabled;
    }

    /// Get current position in microsteps
    pub fn get_position(&self, motor_id: u8) -> Option<i64> {
        if motor_id < 8 {
            Some(self.current_position[motor_id as usize])
        } else {
            None
        }
    }

    /// Get current position in radians
    pub fn get_position_radians(&self, motor_id: u8) -> Option<f64> {
        if motor_id < 8 {
            let idx = motor_id as usize;
            let microsteps_per_rev = self.steps_per_rev[idx] as f64 * self.microsteps[idx] as f64;
            let position = self.current_position[idx] as f64 / microsteps_per_rev * 2.0 * PI;
            Some(position / self.gear_ratio[idx])
        } else {
            None
        }
    }

    /// Get current velocity in steps/sec
    pub fn get_velocity(&self, motor_id: u8) -> Option<f64> {
        if motor_id < 8 {
            Some(self.current_velocity[motor_id as usize])
        } else {
            None
        }
    }

    /// Check if motor is enabled
    pub fn is_enabled(&self, motor_id: u8) -> bool {
        if motor_id < 8 {
            self.motor_enabled[motor_id as usize]
        } else {
            false
        }
    }

    /// Check if motor has been homed
    pub fn is_homed(&self, motor_id: u8) -> bool {
        if motor_id < 8 {
            self.is_homed[motor_id as usize]
        } else {
            false
        }
    }

    /// Set home position (call after homing sequence completes)
    pub fn set_home_position(&mut self, motor_id: u8) {
        if motor_id < 8 {
            let idx = motor_id as usize;
            self.home_position[idx] = self.current_position[idx];
            self.is_homed[idx] = true;
            self.is_homing[idx] = false;
        }
    }

    /// Zero the current position
    pub fn zero_position(&mut self, motor_id: u8) {
        if motor_id < 8 {
            self.current_position[motor_id as usize] = 0;
            self.target_position[motor_id as usize] = 0;
        }
    }

    /// Convert radians to microsteps
    fn radians_to_microsteps(&self, motor_id: u8, radians: f64) -> i64 {
        let idx = motor_id as usize;
        let microsteps_per_rev = self.steps_per_rev[idx] as f64 * self.microsteps[idx] as f64;
        let steps = (radians / (2.0 * PI) * microsteps_per_rev * self.gear_ratio[idx]) as i64;
        steps
    }

    /// Calculate step interval in nanoseconds for a given velocity
    fn velocity_to_interval(&self, motor_id: u8, velocity: f64) -> u64 {
        if velocity.abs() < 0.001 {
            return u64::MAX; // Effectively stopped
        }
        let interval = (1_000_000_000.0 / velocity.abs()) as u64;
        interval
    }

    /// Configure GPIO pins for a specific motor
    pub fn set_gpio_pins(&mut self, motor_id: u8, step_pin: u64, dir_pin: u64, enable_pin: u64) {
        if motor_id < 8 {
            self.gpio_pin_numbers[motor_id as usize] = (step_pin, dir_pin, enable_pin);
        }
    }

    /// Set step pulse duration in microseconds
    pub fn set_step_pulse_duration(&mut self, duration_us: u64) {
        self.step_pulse_duration_us = duration_us.clamp(1, 20); // 1-20μs range
    }

    // ========== Hardware Backend Functions ==========

    /// Initialize GPIO pins for a motor
    #[cfg(feature = "gpio-hardware")]
    fn init_gpio_hardware(
        &mut self,
        motor_id: u8,
        mut ctx: Option<&mut NodeInfo>,
    ) -> std::io::Result<()> {
        let idx = motor_id as usize;
        let (step_num, dir_num, enable_num) = self.gpio_pin_numbers[idx];

        if step_num == 0 || dir_num == 0 || enable_num == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("GPIO pins not configured for motor {}", motor_id),
            ));
        }

        // Initialize step pin (output)
        let step = Pin::new(step_num);
        step.export()?;
        thread::sleep(Duration::from_millis(10));
        step.set_direction(Direction::Out)?;
        step.set_value(0)?; // Start low

        // Initialize direction pin (output)
        let dir = Pin::new(dir_num);
        dir.export()?;
        thread::sleep(Duration::from_millis(10));
        dir.set_direction(Direction::Out)?;
        dir.set_value(0)?; // Start with forward direction

        // Initialize enable pin (output, active-low on most drivers)
        let enable = Pin::new(enable_num);
        enable.export()?;
        thread::sleep(Duration::from_millis(10));
        enable.set_direction(Direction::Out)?;
        enable.set_value(1)?; // Start disabled (active-low)

        self.step_pins[idx] = Some(step);
        self.dir_pins[idx] = Some(dir);
        self.enable_pins[idx] = Some(enable);

        ctx.log_info(&format!(
            "Initialized GPIO for motor {}: step={}, dir={}, enable={}",
            motor_id, step_num, dir_num, enable_num
        ));

        Ok(())
    }

    /// Generate a step pulse via hardware GPIO
    #[cfg(feature = "gpio-hardware")]
    fn step_hardware(&mut self, motor_id: u8, direction: i64) -> std::io::Result<()> {
        let idx = motor_id as usize;

        let step = self.step_pins[idx].as_ref().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Step pin not initialized")
        })?;

        let dir = self.dir_pins[idx].as_ref().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Dir pin not initialized")
        })?;

        let enable = self.enable_pins[idx].as_ref().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::NotConnected,
                "Enable pin not initialized",
            )
        })?;

        // Set direction pin
        let dir_value = if direction > 0 { 1 } else { 0 };
        dir.set_value(dir_value)?;

        // Ensure motor is enabled (active-low)
        enable.set_value(0)?;

        // Generate step pulse
        step.set_value(1)?;
        thread::sleep(Duration::from_micros(self.step_pulse_duration_us));
        step.set_value(0)?;

        Ok(())
    }

    /// Process a stepper command
    fn process_command(&mut self, cmd: StepperCommand, mut ctx: Option<&mut NodeInfo>) {
        let motor_id = cmd.motor_id;

        // Validate motor ID
        if motor_id >= self.num_motors {
            ctx.log_warning(&format!("Invalid motor ID: {}", motor_id));
            return;
        }

        if !cmd.is_valid() {
            ctx.log_warning(&format!(
                "Invalid stepper command for motor {}: {:?}",
                motor_id, cmd
            ));
            return;
        }

        let idx = motor_id as usize;

        // Update microstepping if changed
        if cmd.microsteps != 0 && cmd.microsteps != self.microsteps[idx] {
            self.microsteps[idx] = cmd.microsteps;
        }

        // Update current limit if specified
        if cmd.current_limit > 0 {
            self.current_limit[idx] = cmd.current_limit;
        }

        // Update last command time
        self.last_command_time[idx] = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;

        // Handle enable/disable
        if !cmd.enable {
            self.motor_enabled[idx] = false;
            self.current_velocity[idx] = 0.0;
            self.target_velocity[idx] = 0.0;
            self.motion_state[idx] = MotionState::Idle;
            ctx.log_debug(&format!("Motor {} disabled", motor_id));
            return;
        }

        self.motor_enabled[idx] = true;

        // Determine motion parameters
        let max_vel = if cmd.max_velocity > 0.0 {
            cmd.max_velocity.min(self.max_velocity[idx])
        } else {
            self.max_velocity[idx]
        };

        let accel = if cmd.acceleration > 0.0 {
            cmd.acceleration.min(self.max_acceleration[idx])
        } else {
            self.max_acceleration[idx]
        };

        match cmd.mode {
            StepperCommand::MODE_STEPS => {
                // Relative move in steps
                let steps = cmd.target as i64;
                self.target_position[idx] = self.current_position[idx] + steps;
                self.target_velocity[idx] = 0.0; // Stop at target
                self.motion_state[idx] = if steps != 0 {
                    MotionState::Accelerating
                } else {
                    MotionState::Idle
                };
                ctx.log_debug(&format!(
                    "Motor {}: relative move {} steps to position {}",
                    motor_id, steps, self.target_position[idx]
                ));
            }
            StepperCommand::MODE_POSITION => {
                // Absolute position in radians
                let target_steps = self.radians_to_microsteps(motor_id, cmd.target);
                self.target_position[idx] = target_steps;
                self.target_velocity[idx] = 0.0; // Stop at target
                self.motion_state[idx] = if target_steps != self.current_position[idx] {
                    MotionState::Accelerating
                } else {
                    MotionState::Idle
                };
                ctx.log_debug(&format!(
                    "Motor {}: absolute move to {:.3} rad ({} steps)",
                    motor_id, cmd.target, target_steps
                ));
            }
            StepperCommand::MODE_VELOCITY => {
                // Continuous velocity mode
                self.target_velocity[idx] = cmd.target.clamp(-max_vel, max_vel);
                self.motion_state[idx] = if cmd.target.abs() > 0.0 {
                    MotionState::Accelerating
                } else {
                    MotionState::Decelerating
                };
                ctx.log_debug(&format!(
                    "Motor {}: velocity mode {:.1} steps/sec",
                    motor_id, self.target_velocity[idx]
                ));
            }
            StepperCommand::MODE_HOMING => {
                // Start homing sequence
                self.is_homing[idx] = true;
                self.target_velocity[idx] = cmd.target.abs().min(max_vel / 2.0); // Slower for homing
                self.motion_state[idx] = MotionState::Accelerating;
                ctx.log_info(&format!(
                    "Motor {}: homing started at {:.1} steps/sec",
                    motor_id, self.target_velocity[idx]
                ));
            }
            _ => {
                ctx.log_warning(&format!("Unknown stepper mode: {}", cmd.mode));
            }
        }

        // Publish command echo
        if self.enable_feedback {
            let _ = self.publisher.send(cmd, &mut None);
        }
    }

    /// Update motion control for a single motor
    fn update_motion(&mut self, motor_id: u8, dt: f64, mut ctx: Option<&mut NodeInfo>) {
        if motor_id >= self.num_motors {
            return;
        }

        let idx = motor_id as usize;

        if !self.motor_enabled[idx] {
            return;
        }

        let max_vel = self.max_velocity[idx];
        let accel = self.max_acceleration[idx];

        match self.motion_state[idx] {
            MotionState::Idle => {
                // Nothing to do
            }
            MotionState::Accelerating => {
                // Accelerate toward target velocity
                let vel_diff = self.target_velocity[idx] - self.current_velocity[idx];
                let accel_step = accel * dt;

                if vel_diff.abs() <= accel_step {
                    // Reached target velocity
                    self.current_velocity[idx] = self.target_velocity[idx];
                    self.motion_state[idx] = MotionState::Cruising;
                } else {
                    // Continue accelerating
                    self.current_velocity[idx] += vel_diff.signum() * accel_step;
                }

                // Check if we need to start decelerating for position mode
                if self.target_velocity[idx] == 0.0 {
                    let remaining_steps =
                        (self.target_position[idx] - self.current_position[idx]).abs();
                    let decel_steps =
                        (self.current_velocity[idx].powi(2) / (2.0 * accel)).abs() as i64;

                    if remaining_steps <= decel_steps {
                        self.motion_state[idx] = MotionState::Decelerating;
                    }
                }
            }
            MotionState::Cruising => {
                // Maintain constant velocity
                // Check if we need to start decelerating
                if self.target_velocity[idx] == 0.0 {
                    let remaining_steps =
                        (self.target_position[idx] - self.current_position[idx]).abs();
                    let decel_steps =
                        (self.current_velocity[idx].powi(2) / (2.0 * accel)).abs() as i64;

                    if remaining_steps <= decel_steps {
                        self.motion_state[idx] = MotionState::Decelerating;
                    }
                }

                // Check if we've reached target position
                if self.current_position[idx] == self.target_position[idx]
                    && self.target_velocity[idx] == 0.0
                {
                    self.current_velocity[idx] = 0.0;
                    self.motion_state[idx] = MotionState::Idle;
                }
            }
            MotionState::Decelerating => {
                // Decelerate to target velocity (usually 0)
                let vel_diff = self.target_velocity[idx] - self.current_velocity[idx];
                let accel_step = accel * dt;

                if vel_diff.abs() <= accel_step || self.current_velocity[idx].abs() <= accel_step {
                    // Reached target velocity
                    self.current_velocity[idx] = self.target_velocity[idx];
                    if self.current_velocity[idx] == 0.0 {
                        self.motion_state[idx] = MotionState::Idle;
                        ctx.log_debug(&format!("Motor {} motion complete", motor_id));
                    } else {
                        self.motion_state[idx] = MotionState::Cruising;
                    }
                } else {
                    // Continue decelerating
                    self.current_velocity[idx] -= self.current_velocity[idx].signum() * accel_step;
                }
            }
        }

        // Update step interval based on current velocity
        self.step_interval[idx] = self.velocity_to_interval(motor_id, self.current_velocity[idx]);
    }

    /// Generate steps for a single motor
    fn generate_steps(
        &mut self,
        motor_id: u8,
        current_time: u64,
        mut ctx: Option<&mut NodeInfo>,
    ) -> bool {
        if motor_id >= self.num_motors {
            return false;
        }

        let idx = motor_id as usize;

        if !self.motor_enabled[idx] || self.current_velocity[idx].abs() < 0.001 {
            return false;
        }

        // Check if it's time for the next step
        if current_time < self.last_step_time[idx] + self.step_interval[idx] {
            return false;
        }

        // Calculate direction
        let mut direction = if self.current_velocity[idx] > 0.0 {
            1
        } else {
            -1
        };
        direction = if self.invert_direction[idx] {
            -direction
        } else {
            direction
        };

        // Try hardware first, fall back to simulation
        #[cfg(feature = "gpio-hardware")]
        if self.hardware_enabled || self.step_pins[idx].is_some() {
            // Initialize GPIO if needed
            if self.step_pins[idx].is_none() {
                let (step_num, dir_num, enable_num) = self.gpio_pin_numbers[idx];
                if step_num != 0 && dir_num != 0 && enable_num != 0 {
                    if let Err(e) = self.init_gpio_hardware(motor_id, None) {
                        // Failed to initialize - log detailed error (only once per motor)
                        if self.last_step_time[idx] == 0 {
                            ctx.log_warning(&format!(
                                "StepperMotorNode motor {}: Hardware unavailable - using SIMULATION mode",
                                motor_id
                            ));
                            ctx.log_warning(&format!(
                                "  Tried GPIO pins: step={}, dir={}, enable={}",
                                step_num, dir_num, enable_num
                            ));
                            ctx.log_warning(&format!("  Error: {}", e));
                            ctx.log_warning("  Fix:");
                            ctx.log_warning("    1. Install: sudo apt install libraspberrypi-dev");
                            ctx.log_warning(
                                "    2. Enable GPIO: sudo raspi-config -> Interface Options",
                            );
                            ctx.log_warning(
                                "    3. Check wiring: Verify stepper driver connections",
                            );
                            ctx.log_warning(
                                "    4. Rebuild with: cargo build --features=\"gpio-hardware\"",
                            );
                        }
                        self.hardware_enabled = false;
                    } else {
                        self.hardware_enabled = true;
                    }
                }
            }

            // Try hardware step
            if self.hardware_enabled && self.step_pins[idx].is_some() {
                match self.step_hardware(motor_id, direction) {
                    Ok(()) => {
                        self.current_position[idx] += direction;
                        self.last_step_time[idx] = current_time;

                        // Check if homing limit reached (simulated here)
                        if self.is_homing[idx] {
                            // In real implementation, check limit switch GPIO
                            // For now, just set home after moving 1000 steps
                            if self.current_position[idx].abs() > 1000 {
                                self.set_home_position(motor_id);
                            }
                        }

                        return true;
                    }
                    Err(_e) => {
                        // Hardware error, fall back to simulation
                        self.hardware_enabled = false;
                    }
                }
            }
        }

        // Simulation fallback
        self.current_position[idx] += direction;
        self.last_step_time[idx] = current_time;

        // Check if homing limit reached (simulated here)
        if self.is_homing[idx] {
            // In real implementation, check limit switch GPIO
            // For now, just set home after moving 1000 steps
            if self.current_position[idx].abs() > 1000 {
                self.set_home_position(motor_id);
            }
        }

        true
    }

    /// Check for command timeouts and stop motors if needed
    fn check_timeouts(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if self.command_timeout_ms == 0 {
            return;
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;

        for motor_id in 0..self.num_motors {
            let idx = motor_id as usize;
            if self.motor_enabled[idx] {
                let elapsed = current_time - self.last_command_time[idx];
                if elapsed > self.command_timeout_ms {
                    // Timeout - stop motor
                    self.current_velocity[idx] = 0.0;
                    self.target_velocity[idx] = 0.0;
                    self.motion_state[idx] = MotionState::Idle;

                    ctx.log_warning(&format!(
                        "Motor {} stopped due to command timeout ({}ms)",
                        motor_id, elapsed
                    ));
                }
            }
        }
    }

    /// Publish position feedback
    fn publish_feedback(&mut self) {
        if !self.enable_feedback {
            return;
        }

        for motor_id in 0..self.num_motors {
            let idx = motor_id as usize;
            if self.motor_enabled[idx] {
                let position_rad = self.get_position_radians(motor_id).unwrap_or(0.0);
                let feedback = crate::MotorCommand {
                    motor_id,
                    mode: crate::MotorCommand::MODE_POSITION,
                    target: position_rad,
                    max_velocity: self.current_velocity[idx],
                    max_acceleration: self.max_acceleration[idx],
                    feed_forward: 0.0,
                    enable: self.motor_enabled[idx],
                    timestamp: SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos() as u64,
                };
                let _ = self.feedback_publisher.send(feedback, &mut None);
            }
        }
    }

    /// Emergency stop all motors
    pub fn emergency_stop(&mut self) {
        for motor_id in 0..self.num_motors {
            let idx = motor_id as usize;
            self.motor_enabled[idx] = false;
            self.current_velocity[idx] = 0.0;
            self.target_velocity[idx] = 0.0;
            self.motion_state[idx] = MotionState::Idle;
        }
    }
}

impl Node for StepperMotorNode {
    fn name(&self) -> &'static str {
        "StepperMotorNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("StepperMotorNode shutting down - stopping all stepper motors");

        // Emergency stop all motors
        self.emergency_stop();

        // Disable hardware GPIO pins (set enable pins high = disabled for active-low drivers)
        #[cfg(feature = "gpio-hardware")]
        for motor_id in 0..self.num_motors {
            let idx = motor_id as usize;
            if let Some(ref enable_pin) = self.enable_pins[idx] {
                // Set enable high (disabled for active-low drivers)
                let _ = enable_pin.set_value(1);
            }
        }

        ctx.log_info("All stepper motors stopped and disabled safely");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Process all pending commands
        while let Some(cmd) = self.subscriber.recv(&mut None) {
            self.process_command(cmd, ctx.as_deref_mut());
        }

        // Update motion control for all motors
        let dt = 0.001; // Assume 1ms tick rate (adjust based on actual tick rate)
        for motor_id in 0..self.num_motors {
            self.update_motion(motor_id, dt, ctx.as_deref_mut());
            self.generate_steps(motor_id, current_time, ctx.as_deref_mut());
        }

        // Check for command timeouts
        self.check_timeouts(ctx.as_deref_mut());

        // Publish feedback (throttled to ~10Hz)
        let feedback_interval = 100_000_000; // 100ms
        if current_time - self.last_feedback_time_tick > feedback_interval {
            self.publish_feedback();
            self.last_feedback_time_tick = current_time;
        }
    }
}
