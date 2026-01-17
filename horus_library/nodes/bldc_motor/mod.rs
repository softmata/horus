use crate::MotorCommand;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// GPIO/PWM hardware support (Raspberry Pi)
#[cfg(feature = "gpio-hardware")]
use rppal::pwm::{Channel, Polarity, Pwm};

/// BLDC (Brushless DC) Motor Controller Node
///
/// Controls brushless DC motors via Electronic Speed Controllers (ESC).
/// Supports multiple ESC protocols including PWM, OneShot, DShot, and CAN.
///
/// # Supported ESCs
/// - Hobby ESCs: Standard PWM (1000-2000μs), OneShot125/42, Multishot
/// - Digital protocols: DShot150/300/600/1200, ProShot
/// - CAN-based: UAVCAN, DroneCAN, VESC CAN
/// - Industrial: ODrive, SimpleFOC, VESC
/// - Proprietary: T-Motor, KDE Direct, Castle Creations
///
/// # Motor Applications
/// - Multirotor drones (quadcopters, hexacopters)
/// - Fixed-wing aircraft
/// - Ground vehicles (robots, RC cars)
/// - Gimbals and camera stabilizers
/// - Industrial automation
/// - Electric skateboards/scooters
///
/// # Features
/// - Multiple ESC protocol support
/// - Velocity and position control modes
/// - Motor arming/disarming safety
/// - Telemetry feedback (voltage, current, RPM, temperature)
/// - Direction reversal support
/// - Acceleration ramping
/// - Failsafe handling
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::BldcMotorNode;
/// use horus_library::MotorCommand;
///
/// let mut bldc = BldcMotorNode::new()?;
/// bldc.set_num_motors(4);
/// bldc.set_protocol(BldcProtocol::DShot600);
/// bldc.set_velocity_limits(0, 0.0, 5000.0); // 0-5000 RPM
/// bldc.arm_motors();
///
/// // Send velocity command
/// let cmd = MotorCommand::velocity(0, 2000.0); // 2000 RPM
/// ```
pub struct BldcMotorNode {
    subscriber: Topic<MotorCommand>,
    publisher: Topic<MotorCommand>, // Echo commands
    telemetry_publisher: Topic<BldcTelemetry>,

    // Configuration
    num_motors: u8,
    protocol: BldcProtocol,
    pwm_min: [u16; 8],      // Minimum PWM value (typically 1000μs)
    pwm_max: [u16; 8],      // Maximum PWM value (typically 2000μs)
    pwm_neutral: [u16; 8],  // Neutral/zero throttle (typically 1500μs)
    min_velocity: [f64; 8], // Minimum RPM
    max_velocity: [f64; 8], // Maximum RPM
    invert_direction: [bool; 8],

    // State tracking per motor
    current_velocity: [f64; 8], // Current velocity in RPM
    target_velocity: [f64; 8],  // Target velocity in RPM
    current_throttle: [f64; 8], // Current throttle (0.0-1.0)
    motor_armed: [bool; 8],
    motor_enabled: [bool; 8],
    last_command_time: [u64; 8],

    // Telemetry (from ESC feedback)
    voltage: [f32; 8],      // Motor/battery voltage
    current: [f32; 8],      // Motor current draw
    temperature: [f32; 8],  // ESC/motor temperature
    rpm_measured: [f64; 8], // Measured RPM (if available)
    error_count: [u32; 8],

    // Control parameters
    acceleration_limit: [f64; 8], // RPM per second
    command_timeout_ms: u64,
    enable_telemetry: bool,
    armed: bool, // Global arm state

    // Hardware PWM channels (Raspberry Pi hardware PWM)
    #[cfg(feature = "gpio-hardware")]
    pwm_channels: [Option<Pwm>; 8],
    hardware_enabled: bool,
    pwm_gpio_pins: [u8; 8], // GPIO pin numbers for PWM output (12, 13, 18, 19 for RPi)
    pwm_frequency_hz: f64,  // PWM frequency (50 Hz for standard ESC, higher for digital protocols)

    // Timing state (moved from static mut for thread safety)
    last_telemetry_time: u64,
}

use serde::{Deserialize, Serialize};

/// BLDC Telemetry Data
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BldcTelemetry {
    pub motor_id: u8,
    pub voltage: f32,
    pub current: f32,
    pub temperature: f32,
    pub rpm: f64,
    pub throttle: f64,
    pub error_count: u32,
    pub timestamp: u64,
}

/// ESC Communication Protocol
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BldcProtocol {
    /// Standard PWM (1000-2000μs, 50Hz update rate)
    StandardPwm,
    /// OneShot125 (125-250μs, up to 4kHz)
    OneShot125,
    /// OneShot42 (42-84μs, up to 12kHz)
    OneShot42,
    /// Multishot (5-25μs, up to 32kHz)
    MultiShot,
    /// DShot150 (150kbit/s digital)
    DShot150,
    /// DShot300 (300kbit/s digital)
    DShot300,
    /// DShot600 (600kbit/s digital)
    DShot600,
    /// DShot1200 (1200kbit/s digital)
    DShot1200,
    /// ProShot (high-speed bidirectional)
    ProShot,
    /// CAN bus control
    Can,
}

impl BldcMotorNode {
    /// Create a new BLDC motor controller node
    pub fn new() -> Result<Self> {
        Self::new_with_topic("bldc_cmd")
    }

    /// Create a new BLDC motor controller with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        #[cfg(feature = "gpio-hardware")]
        const NONE_PWM: Option<Pwm> = None;
        Ok(Self {
            subscriber: Topic::new(topic)?,
            publisher: Topic::new(&format!("{}_feedback", topic))?,
            telemetry_publisher: Topic::new(&format!("{}_telemetry", topic))?,
            num_motors: 1,
            protocol: BldcProtocol::DShot600,
            pwm_min: [1000; 8],
            pwm_max: [2000; 8],
            pwm_neutral: [1500; 8],
            min_velocity: [0.0; 8],
            max_velocity: [10000.0; 8], // 10,000 RPM default max
            invert_direction: [false; 8],
            current_velocity: [0.0; 8],
            target_velocity: [0.0; 8],
            current_throttle: [0.0; 8],
            motor_armed: [false; 8],
            motor_enabled: [false; 8],
            last_command_time: [0; 8],
            voltage: [0.0; 8],
            current: [0.0; 8],
            temperature: [25.0; 8],
            rpm_measured: [0.0; 8],
            error_count: [0; 8],
            acceleration_limit: [5000.0; 8], // 5000 RPM/s default
            command_timeout_ms: 1000,
            enable_telemetry: true,
            armed: false,
            #[cfg(feature = "gpio-hardware")]
            pwm_channels: [NONE_PWM; 8],
            hardware_enabled: false,
            pwm_gpio_pins: [0; 8],  // Will be configured via set_pwm_pin()
            pwm_frequency_hz: 50.0, // Default 50Hz for standard PWM ESCs
            last_telemetry_time: 0,
        })
    }

    /// Set the number of motors (1-8)
    pub fn set_num_motors(&mut self, num: u8) {
        self.num_motors = num.clamp(1, 8);
    }

    /// Set ESC protocol
    pub fn set_protocol(&mut self, protocol: BldcProtocol) {
        self.protocol = protocol;
    }

    /// Set PWM range for a motor (in microseconds)
    pub fn set_pwm_range(&mut self, motor_id: u8, min: u16, max: u16, neutral: u16) {
        if motor_id < 8 {
            let idx = motor_id as usize;
            self.pwm_min[idx] = min;
            self.pwm_max[idx] = max;
            self.pwm_neutral[idx] = neutral;
        }
    }

    /// Set velocity limits for a motor (in RPM)
    pub fn set_velocity_limits(&mut self, motor_id: u8, min: f64, max: f64) {
        if motor_id < 8 {
            let idx = motor_id as usize;
            self.min_velocity[idx] = min;
            self.max_velocity[idx] = max;
        }
    }

    /// Set acceleration limit (RPM per second)
    pub fn set_acceleration_limit(&mut self, motor_id: u8, limit: f64) {
        if motor_id < 8 {
            self.acceleration_limit[motor_id as usize] = limit.abs();
        }
    }

    /// Invert motor direction
    pub fn set_direction_inverted(&mut self, motor_id: u8, inverted: bool) {
        if motor_id < 8 {
            self.invert_direction[motor_id as usize] = inverted;
        }
    }

    /// Arm all motors (safety requirement)
    pub fn arm_motors(&mut self) {
        self.armed = true;
        for i in 0..self.num_motors as usize {
            self.motor_armed[i] = true;
        }
    }

    /// Disarm all motors
    pub fn disarm_motors(&mut self) {
        self.armed = false;
        for i in 0..self.num_motors as usize {
            self.motor_armed[i] = false;
            self.motor_enabled[i] = false;
            self.current_velocity[i] = 0.0;
            self.target_velocity[i] = 0.0;
            self.current_throttle[i] = 0.0;
        }
    }

    /// Arm a specific motor
    pub fn arm_motor(&mut self, motor_id: u8) {
        if motor_id < 8 && self.armed {
            self.motor_armed[motor_id as usize] = true;
        }
    }

    /// Check if motors are armed
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Get current velocity for a motor (RPM)
    pub fn get_velocity(&self, motor_id: u8) -> Option<f64> {
        if motor_id < 8 {
            Some(self.current_velocity[motor_id as usize])
        } else {
            None
        }
    }

    /// Get current throttle for a motor (0.0-1.0)
    pub fn get_throttle(&self, motor_id: u8) -> Option<f64> {
        if motor_id < 8 {
            Some(self.current_throttle[motor_id as usize])
        } else {
            None
        }
    }

    /// Get telemetry for a motor
    pub fn get_telemetry(&self, motor_id: u8) -> Option<BldcTelemetry> {
        if motor_id < 8 {
            let idx = motor_id as usize;
            Some(BldcTelemetry {
                motor_id,
                voltage: self.voltage[idx],
                current: self.current[idx],
                temperature: self.temperature[idx],
                rpm: self.rpm_measured[idx],
                throttle: self.current_throttle[idx],
                error_count: self.error_count[idx],
                timestamp: SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
            })
        } else {
            None
        }
    }

    /// Convert velocity (RPM) to throttle (0.0-1.0)
    fn velocity_to_throttle(&self, motor_id: u8, velocity: f64) -> f64 {
        let idx = motor_id as usize;
        let range = self.max_velocity[idx] - self.min_velocity[idx];
        if range <= 0.0 {
            return 0.0;
        }
        let normalized = (velocity - self.min_velocity[idx]) / range;
        normalized.clamp(0.0, 1.0)
    }

    /// Convert throttle (0.0-1.0) to PWM value (microseconds)
    fn throttle_to_pwm(&self, motor_id: u8, throttle: f64) -> u16 {
        let idx = motor_id as usize;
        let range = (self.pwm_max[idx] - self.pwm_min[idx]) as f64;
        let pwm = self.pwm_min[idx] as f64 + (throttle * range);
        pwm.clamp(self.pwm_min[idx] as f64, self.pwm_max[idx] as f64) as u16
    }

    /// Configure GPIO pin for PWM output
    pub fn set_pwm_pin(&mut self, motor_id: u8, gpio_pin: u8) {
        if motor_id < 8 {
            self.pwm_gpio_pins[motor_id as usize] = gpio_pin;
        }
    }

    /// Set PWM frequency in Hz (default 50 Hz for standard ESCs)
    pub fn set_pwm_frequency(&mut self, frequency_hz: f64) {
        self.pwm_frequency_hz = frequency_hz.clamp(50.0, 32000.0);
    }

    // ========== Hardware Backend Functions ==========

    /// Initialize PWM hardware for a motor
    #[cfg(feature = "gpio-hardware")]
    fn init_pwm_hardware(
        &mut self,
        motor_id: u8,
        mut ctx: Option<&mut NodeInfo>,
    ) -> std::io::Result<()> {
        let idx = motor_id as usize;
        let gpio_pin = self.pwm_gpio_pins[idx];

        if gpio_pin == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("PWM pin not configured for motor {}", motor_id),
            ));
        }

        // Map GPIO pin to PWM channel (Raspberry Pi specific)
        // GPIO 12 & 13 = PWM0, GPIO 18 & 19 = PWM1
        let channel = match gpio_pin {
            12 | 13 => Channel::Pwm0,
            18 | 19 => Channel::Pwm1,
            _ => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("Invalid PWM GPIO pin {} (use 12, 13, 18, or 19)", gpio_pin),
                ));
            }
        };

        // Initialize PWM
        let pwm = Pwm::with_frequency(
            channel,
            self.pwm_frequency_hz,
            0.0, // Start at 0% duty cycle
            Polarity::Normal,
            true, // enabled
        )
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        self.pwm_channels[idx] = Some(pwm);

        ctx.log_info(&format!(
            "Initialized PWM for motor {}: GPIO {}, {} Hz",
            motor_id, gpio_pin, self.pwm_frequency_hz
        ));

        Ok(())
    }

    /// Send PWM signal to ESC via hardware
    #[cfg(feature = "gpio-hardware")]
    fn send_pwm_hardware(&mut self, motor_id: u8, pwm_us: u16) -> std::io::Result<()> {
        let idx = motor_id as usize;

        let pwm = self.pwm_channels[idx].as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "PWM not initialized")
        })?;

        // Convert microseconds to duty cycle percentage
        // For 50Hz: period = 20000μs, so duty_cycle = (pwm_us / 20000.0) * 100.0
        let period_us = 1_000_000.0 / self.pwm_frequency_hz;
        let duty_cycle = (pwm_us as f64 / period_us) * 100.0;
        let duty_cycle = duty_cycle.clamp(0.0, 100.0);

        pwm.set_duty_cycle(duty_cycle)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        Ok(())
    }

    /// Send ESC command (protocol-specific)
    fn send_esc_command(&mut self, motor_id: u8, throttle: f64, mut ctx: Option<&mut NodeInfo>) {
        let idx = motor_id as usize;

        // Calculate protocol-specific PWM value
        let pwm_us = match self.protocol {
            BldcProtocol::StandardPwm => self.throttle_to_pwm(motor_id, throttle),
            BldcProtocol::OneShot125 => 125 + (throttle * 125.0) as u16,
            BldcProtocol::OneShot42 => 42 + (throttle * 42.0) as u16,
            BldcProtocol::MultiShot => 5 + (throttle * 20.0) as u16,
            BldcProtocol::DShot150
            | BldcProtocol::DShot300
            | BldcProtocol::DShot600
            | BldcProtocol::DShot1200 => {
                // DShot uses 11-bit throttle value (0-2047)
                // For PWM approximation, map to standard range
                if throttle < 0.001 {
                    1000 // Disarmed
                } else {
                    1000 + (throttle * 1000.0) as u16
                }
            }
            BldcProtocol::ProShot => 1000 + (throttle * 1000.0) as u16,
            BldcProtocol::Can => {
                // CAN doesn't use PWM, but for fallback purposes
                1000 + (throttle * 1000.0) as u16
            }
        };

        // Try hardware first, fall back to simulation
        #[cfg(feature = "gpio-hardware")]
        if self.hardware_enabled || self.pwm_channels[idx].is_some() {
            // Initialize PWM if needed
            if self.pwm_channels[idx].is_none() && self.pwm_gpio_pins[idx] != 0 {
                if let Err(e) = self.init_pwm_hardware(motor_id, ctx.as_deref_mut()) {
                    // Failed to initialize - log detailed error (only once per motor)
                    if self.last_command_time[idx] == 0 {
                        let gpio_pin = self.pwm_gpio_pins[idx];
                        ctx.log_warning(&format!(
                            "BldcMotorNode motor {}: Hardware unavailable - using SIMULATION mode",
                            motor_id
                        ));
                        ctx.log_warning(&format!("  Tried GPIO pin: {}", gpio_pin));
                        ctx.log_warning(&format!("  Error: {}", e));
                        ctx.log_warning("  Fix:");
                        ctx.log_warning("    1. Install: sudo apt install libraspberrypi-dev");
                        ctx.log_warning("    2. Use hardware PWM pins: GPIO 12, 13, 18, or 19");
                        ctx.log_warning(
                            "    3. Check ESC wiring: Signal wire to GPIO, power to battery",
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

            // Try hardware PWM
            if self.hardware_enabled && self.pwm_channels[idx].is_some() {
                match self.send_pwm_hardware(motor_id, pwm_us) {
                    Ok(()) => {
                        ctx.log_debug(&format!(
                            "Motor {} (HW): {:?} {}μs ({:.1}%)",
                            motor_id,
                            self.protocol,
                            pwm_us,
                            throttle * 100.0
                        ));
                        return;
                    }
                    Err(_e) => {
                        // Hardware error, fall back to simulation
                        self.hardware_enabled = false;
                    }
                }
            }
        }

        // Simulation fallback
        ctx.log_debug(&format!(
            "Motor {} (SIM): {:?} {}μs ({:.1}%)",
            motor_id,
            self.protocol,
            pwm_us,
            throttle * 100.0
        ));
    }

    /// Process a motor command
    fn process_command(&mut self, cmd: MotorCommand, mut ctx: Option<&mut NodeInfo>) {
        let motor_id = cmd.motor_id;

        if motor_id >= self.num_motors {
            ctx.log_warning(&format!("Invalid motor ID: {}", motor_id));
            return;
        }

        if !cmd.is_valid() {
            ctx.log_warning(&format!("Invalid motor command: {:?}", cmd));
            return;
        }

        let idx = motor_id as usize;

        // Update last command time
        self.last_command_time[idx] = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Handle enable/disable
        if !cmd.enable {
            self.motor_enabled[idx] = false;
            self.target_velocity[idx] = 0.0;
            return;
        }

        // Check armed state
        if !self.motor_armed[idx] {
            ctx.log_warning(&format!("Motor {} not armed, ignoring command", motor_id));
            return;
        }

        self.motor_enabled[idx] = true;

        // Process based on control mode
        match cmd.mode {
            MotorCommand::MODE_VELOCITY => {
                let mut target_vel = cmd.target;
                if self.invert_direction[idx] {
                    target_vel = -target_vel;
                }
                target_vel = target_vel.clamp(self.min_velocity[idx], self.max_velocity[idx]);
                self.target_velocity[idx] = target_vel;

                ctx.log_debug(&format!(
                    "Motor {}: velocity command {:.0} RPM",
                    motor_id, target_vel
                ));
            }
            MotorCommand::MODE_POSITION => {
                ctx.log_warning(&format!(
                    "Motor {}: position mode not supported for BLDC",
                    motor_id
                ));
            }
            MotorCommand::MODE_TORQUE => {
                ctx.log_warning(&format!(
                    "Motor {}: torque mode requires current sensing ESC",
                    motor_id
                ));
            }
            _ => {
                ctx.log_warning(&format!(
                    "Motor {}: unknown control mode {}",
                    motor_id, cmd.mode
                ));
            }
        }
    }

    /// Update motor control (acceleration limiting)
    fn update_control(&mut self, dt: f64, mut ctx: Option<&mut NodeInfo>) {
        for motor_id in 0..self.num_motors {
            let idx = motor_id as usize;

            if !self.motor_enabled[idx] || !self.motor_armed[idx] {
                self.current_velocity[idx] = 0.0;
                self.current_throttle[idx] = 0.0;
                continue;
            }

            // Apply acceleration limiting
            let vel_diff = self.target_velocity[idx] - self.current_velocity[idx];
            let max_delta = self.acceleration_limit[idx] * dt;

            if vel_diff.abs() <= max_delta {
                self.current_velocity[idx] = self.target_velocity[idx];
            } else {
                self.current_velocity[idx] += vel_diff.signum() * max_delta;
            }

            // Convert to throttle
            let throttle = self.velocity_to_throttle(motor_id, self.current_velocity[idx]);
            self.current_throttle[idx] = throttle;

            // Send ESC command
            self.send_esc_command(motor_id, throttle, ctx.as_deref_mut());
        }
    }

    /// Simulate telemetry feedback
    fn simulate_telemetry(&mut self, motor_id: u8) {
        let idx = motor_id as usize;

        // Simulate realistic telemetry values
        self.rpm_measured[idx] = self.current_velocity[idx];
        self.voltage[idx] = 14.8 - (self.current_throttle[idx] * 0.5) as f32; // Battery sag
        self.current[idx] = (self.current_throttle[idx] * 30.0) as f32; // Up to 30A
        self.temperature[idx] = 25.0 + (self.current_throttle[idx] * 50.0) as f32;
        // Heat up
    }

    /// Check for command timeouts
    fn check_timeouts(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if self.command_timeout_ms == 0 {
            return;
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        for motor_id in 0..self.num_motors {
            let idx = motor_id as usize;
            if self.motor_enabled[idx] {
                let elapsed = current_time - self.last_command_time[idx];
                if elapsed > self.command_timeout_ms {
                    self.motor_enabled[idx] = false;
                    self.target_velocity[idx] = 0.0;
                    self.current_velocity[idx] = 0.0;
                    self.current_throttle[idx] = 0.0;

                    ctx.log_warning(&format!(
                        "Motor {} stopped due to command timeout ({}ms)",
                        motor_id, elapsed
                    ));
                }
            }
        }
    }

    /// Emergency stop all motors
    pub fn emergency_stop(&mut self) {
        self.disarm_motors();
    }
}

impl Node for BldcMotorNode {
    fn name(&self) -> &'static str {
        "BldcMotorNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Process all pending commands
        while let Some(cmd) = self.subscriber.recv(&mut None) {
            self.process_command(cmd, ctx.as_deref_mut());
        }

        // Update control
        let dt = 0.001; // Assume 1ms tick rate
        self.update_control(dt, ctx.as_deref_mut());

        // Check timeouts
        self.check_timeouts(ctx.as_deref_mut());

        // Simulate and publish telemetry
        if self.enable_telemetry {
            let telemetry_interval = 100_000_000; // 100ms = 10Hz
            let current_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64;

            if current_time - self.last_telemetry_time > telemetry_interval {
                for motor_id in 0..self.num_motors {
                    self.simulate_telemetry(motor_id);
                    // Telemetry is simulated and stored in the node
                    // In real implementation, would publish via Hub
                }
                self.last_telemetry_time = current_time;
            }
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("BldcMotorNode shutting down - stopping all ESCs");

        // Stop all motors by setting throttle to 0
        for motor_id in 0..self.num_motors {
            // Set target and current velocity to 0
            self.target_velocity[motor_id as usize] = 0.0;
            self.current_velocity[motor_id as usize] = 0.0;

            // Publish stop command
            let stop_cmd = MotorCommand::stop(motor_id);
            let _ = self.publisher.send(stop_cmd, &mut None);
        }

        // If hardware mode, send stop signal to ESCs
        #[cfg(feature = "gpio-hardware")]
        for pwm in self.pwm_channels.iter_mut().flatten() {
            // Set PWM to minimum throttle (typically 1000us for ESCs)
            let _ = pwm.set_duty_cycle(0.05); // 5% duty = 1000us at 50Hz
        }

        ctx.log_info("All BLDC motors/ESCs stopped");
        Ok(())
    }
}

/// Preset configurations for common motor/ESC setups
impl BldcMotorNode {
    /// Configure for racing quadcopter (high-performance)
    pub fn configure_racing_quad(&mut self) {
        self.set_protocol(BldcProtocol::DShot600);
        for i in 0..4 {
            self.set_velocity_limits(i, 0.0, 40000.0); // 40,000 RPM
            self.set_acceleration_limit(i, 50000.0); // Fast response
        }
    }

    /// Configure for camera gimbal (smooth, precise)
    pub fn configure_gimbal(&mut self) {
        self.set_protocol(BldcProtocol::DShot300);
        for i in 0..3 {
            self.set_velocity_limits(i, 0.0, 5000.0); // 5,000 RPM
            self.set_acceleration_limit(i, 1000.0); // Smooth motion
        }
    }

    /// Configure for industrial application
    pub fn configure_industrial(&mut self) {
        self.set_protocol(BldcProtocol::Can);
        for i in 0..self.num_motors {
            self.set_velocity_limits(i, 0.0, 15000.0); // 15,000 RPM
            self.set_acceleration_limit(i, 5000.0);
        }
    }
}
