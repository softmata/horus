use crate::MotorCommand;
use horus_core::core::LogSummary;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

// Serial hardware support
#[cfg(feature = "serial-hardware")]
use serialport::SerialPort;
#[cfg(feature = "serial-hardware")]
use std::time::Duration;

/// Roboclaw Motor Controller Node
///
/// Interface for BasicMicro/Ion Motion Control Roboclaw motor controllers.
/// Supports dual-channel DC motor control with encoder feedback, PID control,
/// and current/voltage monitoring.
///
/// # Supported Models
/// - **Roboclaw 2x7A**: 7A continuous, 15A peak per channel
/// - **Roboclaw 2x15A**: 15A continuous, 30A peak per channel
/// - **Roboclaw 2x30A**: 30A continuous, 60A peak per channel
/// - **Roboclaw 2x45A**: 45A continuous, 90A peak per channel
/// - **Roboclaw 2x60A**: 60A continuous, 120A peak per channel
/// - **Roboclaw 2x160A**: 160A continuous, 320A peak per channel (48V max)
/// - **All Solo and ST series variants**
///
/// # Communication Protocol
/// - UART/TTL serial interface (3.3V or 5V logic)
/// - Baud rates: 2400 - 460800 (default: 38400)
/// - Packet mode with CRC-16 checksum
/// - Address range: 0x80-0x87 (up to 8 controllers on one bus)
///
/// # Control Modes
/// - **Duty Cycle**: Direct PWM control (-10000 to +10000)
/// - **Velocity**: Speed control with encoder feedback (QPPS - quad pulses per second)
/// - **Position**: Absolute/relative position control with encoder feedback
/// - **Mixed Mode**: Independent control of both motors simultaneously
///
/// # Features
/// - Dual quadrature encoder inputs (up to 4 million counts)
/// - PID velocity control
/// - Current sensing and limiting
/// - Battery voltage monitoring
/// - Temperature monitoring
/// - E-stop and safety features
/// - Velocity/acceleration ramping
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::RoboclawMotorNode;
///
/// let mut roboclaw = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
/// roboclaw.set_baud_rate(115200);
/// roboclaw.set_velocity_pid(1, 1.0, 0.5, 0.25, 44000); // Motor 1 PID tuning
/// roboclaw.set_max_current(30.0); // 30A current limit
/// ```
///
/// # Hybrid Pattern
///
/// This node supports the hybrid pattern for custom processing of feedback:
///
/// ```rust,ignore
/// let node = RoboclawMotorNode::builder("/dev/ttyUSB0", 0x80)
///     .with_filter(|feedback| {
///         // Only publish feedback when motor is moving
///         if feedback.velocity != 0 {
///             Some(feedback)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct RoboclawMotorNode<P = PassThrough<RoboclawFeedback>>
where
    P: Processor<RoboclawFeedback>,
{
    // Motor command subscribers
    motor1_cmd_sub: Topic<MotorCommand>,
    motor2_cmd_sub: Topic<MotorCommand>,

    // Status publishers
    motor1_feedback_pub: Topic<RoboclawFeedback>,
    motor2_feedback_pub: Topic<RoboclawFeedback>,
    diagnostic_pub: Topic<RoboclawDiagnostics>,

    // Configuration
    device_address: u8, // Roboclaw address (0x80-0x87)
    baud_rate: u32,
    serial_port: String,
    timeout_ms: u32,

    // Hardware serial port
    #[cfg(feature = "serial-hardware")]
    hardware_port: Option<Box<dyn SerialPort>>,
    hardware_enabled: bool,

    // Motor state
    motor_states: [MotorState; 2],

    // PID configuration for each motor
    velocity_pid: [PidParams; 2],
    position_pid: [PidParams; 2],

    // Encoder configuration
    encoder_resolution: [u32; 2], // Pulses per revolution (PPR)
    gear_ratio: [f64; 2],
    wheel_radius: [f64; 2], // meters

    // Limits
    max_current: f64,           // Amperes
    max_velocity: [i32; 2],     // QPPS (quad pulses per second)
    max_acceleration: [u32; 2], // QPPS/second

    // Diagnostics
    battery_voltage: f64,
    main_current: f64,
    motor_currents: [f64; 2],
    temperatures: [f64; 2],
    error_status: u16,
    last_command_time: u64,

    // Statistics
    command_count: u64,
    last_feedback_time: [u64; 2],

    // Timing state (moved from static mut for thread safety)
    feedback_counter: u32,

    // Processor for hybrid pattern
    processor: P,
}

/// Motor state tracker
#[derive(Debug, Clone, Copy)]
struct MotorState {
    encoder_count: i32,
    velocity: i32,   // QPPS
    duty_cycle: i16, // -10000 to +10000
    current: f64,    // Amperes
    enabled: bool,
    direction: i8, // -1, 0, or 1
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            encoder_count: 0,
            velocity: 0,
            duty_cycle: 0,
            current: 0.0,
            enabled: false,
            direction: 0,
        }
    }
}

/// PID parameters
#[derive(Debug, Clone, Copy)]
pub struct PidParams {
    pub p: f64,
    pub i: f64,
    pub d: f64,
    pub qpps: u32, // Max velocity for velocity PID
}

impl Default for PidParams {
    fn default() -> Self {
        Self {
            p: 1.0,
            i: 0.5,
            d: 0.25,
            qpps: 44000, // Default max QPPS
        }
    }
}

/// Roboclaw feedback message
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct RoboclawFeedback {
    pub motor_id: u8, // 1 or 2
    pub encoder_count: i32,
    pub velocity: i32, // QPPS
    pub duty_cycle: i16,
    pub current: f64,         // Amperes
    pub position: f64,        // meters or radians (depends on configuration)
    pub linear_velocity: f64, // m/s or rad/s
    pub timestamp: u64,
}

impl Default for RoboclawFeedback {
    fn default() -> Self {
        Self {
            motor_id: 0,
            encoder_count: 0,
            velocity: 0,
            duty_cycle: 0,
            current: 0.0,
            position: 0.0,
            linear_velocity: 0.0,
            timestamp: 0,
        }
    }
}

/// Roboclaw diagnostic information
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct RoboclawDiagnostics {
    pub battery_voltage: f64, // Volts
    pub main_current: f64,    // Amperes
    pub motor1_current: f64,
    pub motor2_current: f64,
    pub temperature1: f64, // Celsius
    pub temperature2: f64,
    pub error_status: u16,
    pub warning_flags: u8,
    pub timestamp: u64,
}

impl Default for RoboclawDiagnostics {
    fn default() -> Self {
        Self {
            battery_voltage: 0.0,
            main_current: 0.0,
            motor1_current: 0.0,
            motor2_current: 0.0,
            temperature1: 25.0,
            temperature2: 25.0,
            error_status: 0,
            warning_flags: 0,
            timestamp: 0,
        }
    }
}

impl RoboclawMotorNode {
    /// Create a new Roboclaw motor controller node
    pub fn new(serial_port: &str, address: u8) -> Result<Self> {
        if !(0x80..=0x87).contains(&address) {
            return Err(horus_core::error::HorusError::config(format!(
                "Invalid Roboclaw address: 0x{:02X}. Must be 0x80-0x87",
                address
            )));
        }

        Ok(Self {
            motor1_cmd_sub: Topic::new("roboclaw.motor1.cmd")?,
            motor2_cmd_sub: Topic::new("roboclaw.motor2.cmd")?,
            motor1_feedback_pub: Topic::new("roboclaw.motor1.feedback")?,
            motor2_feedback_pub: Topic::new("roboclaw.motor2.feedback")?,
            diagnostic_pub: Topic::new("roboclaw.diagnostics")?,
            device_address: address,
            baud_rate: 38400,
            serial_port: serial_port.to_string(),
            timeout_ms: 100,
            #[cfg(feature = "serial-hardware")]
            hardware_port: None,
            hardware_enabled: false,
            motor_states: [MotorState::default(); 2],
            velocity_pid: [PidParams::default(); 2],
            position_pid: [PidParams::default(); 2],
            encoder_resolution: [1024, 1024], // Common default
            gear_ratio: [1.0, 1.0],
            wheel_radius: [0.05, 0.05], // 5cm radius
            max_current: 30.0,
            max_velocity: [44000, 44000],
            max_acceleration: [10000, 10000],
            battery_voltage: 0.0,
            main_current: 0.0,
            motor_currents: [0.0, 0.0],
            temperatures: [25.0, 25.0],
            error_status: 0,
            last_command_time: 0,
            command_count: 0,
            last_feedback_time: [0, 0],
            feedback_counter: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for custom configuration
    pub fn builder(
        serial_port: &str,
        address: u8,
    ) -> RoboclawMotorNodeBuilder<PassThrough<RoboclawFeedback>> {
        RoboclawMotorNodeBuilder::new(serial_port, address)
    }
}

impl<P> RoboclawMotorNode<P>
where
    P: Processor<RoboclawFeedback>,
{
    /// Set baud rate for serial communication
    pub fn set_baud_rate(&mut self, baud: u32) {
        self.baud_rate = baud;
    }

    /// Set encoder resolution (pulses per revolution)
    pub fn set_encoder_resolution(&mut self, motor_id: u8, ppr: u32) {
        if (1..=2).contains(&motor_id) {
            self.encoder_resolution[(motor_id - 1) as usize] = ppr;
        }
    }

    /// Set gear ratio
    pub fn set_gear_ratio(&mut self, motor_id: u8, ratio: f64) {
        if (1..=2).contains(&motor_id) {
            self.gear_ratio[(motor_id - 1) as usize] = ratio;
        }
    }

    /// Set wheel radius for linear velocity calculation
    pub fn set_wheel_radius(&mut self, motor_id: u8, radius: f64) {
        if (1..=2).contains(&motor_id) {
            self.wheel_radius[(motor_id - 1) as usize] = radius;
        }
    }

    /// Set velocity PID parameters
    pub fn set_velocity_pid(&mut self, motor_id: u8, p: f64, i: f64, d: f64, qpps: u32) {
        if (1..=2).contains(&motor_id) {
            let idx = (motor_id - 1) as usize;
            self.velocity_pid[idx] = PidParams { p, i, d, qpps };
        }
    }

    /// Set position PID parameters
    pub fn set_position_pid(&mut self, motor_id: u8, p: f64, i: f64, d: f64, qpps: u32) {
        if (1..=2).contains(&motor_id) {
            let idx = (motor_id - 1) as usize;
            self.position_pid[idx] = PidParams { p, i, d, qpps };
        }
    }

    /// Set maximum current limit
    pub fn set_max_current(&mut self, amps: f64) {
        self.max_current = amps.max(0.0);
    }

    /// Set maximum velocity for a motor
    pub fn set_max_velocity(&mut self, motor_id: u8, qpps: i32) {
        if (1..=2).contains(&motor_id) {
            self.max_velocity[(motor_id - 1) as usize] = qpps;
        }
    }

    /// Set maximum acceleration
    pub fn set_max_acceleration(&mut self, motor_id: u8, qpps_per_sec: u32) {
        if (1..=2).contains(&motor_id) {
            self.max_acceleration[(motor_id - 1) as usize] = qpps_per_sec;
        }
    }

    /// Get encoder count
    pub fn get_encoder(&self, motor_id: u8) -> Option<i32> {
        if (1..=2).contains(&motor_id) {
            Some(self.motor_states[(motor_id - 1) as usize].encoder_count)
        } else {
            None
        }
    }

    /// Get velocity in QPPS
    pub fn get_velocity(&self, motor_id: u8) -> Option<i32> {
        if (1..=2).contains(&motor_id) {
            Some(self.motor_states[(motor_id - 1) as usize].velocity)
        } else {
            None
        }
    }

    /// Get battery voltage
    pub fn get_battery_voltage(&self) -> f64 {
        self.battery_voltage
    }

    /// Get motor current
    pub fn get_motor_current(&self, motor_id: u8) -> Option<f64> {
        if (1..=2).contains(&motor_id) {
            Some(self.motor_currents[(motor_id - 1) as usize])
        } else {
            None
        }
    }

    /// Reset encoder count
    pub fn reset_encoder(&mut self, motor_id: u8) {
        if (1..=2).contains(&motor_id) {
            self.motor_states[(motor_id - 1) as usize].encoder_count = 0;
        }
    }

    /// Convert encoder counts to linear position
    fn encoder_to_position(&self, motor_id: u8, counts: i32) -> f64 {
        let idx = (motor_id - 1) as usize;
        let revolutions = counts as f64 / (self.encoder_resolution[idx] as f64 * 4.0); // Quad pulses
        let wheel_revolutions = revolutions / self.gear_ratio[idx];
        wheel_revolutions * 2.0 * std::f64::consts::PI * self.wheel_radius[idx]
    }

    /// Convert QPPS to linear velocity
    fn qpps_to_velocity(&self, motor_id: u8, qpps: i32) -> f64 {
        let idx = (motor_id - 1) as usize;
        let rps = qpps as f64 / (self.encoder_resolution[idx] as f64 * 4.0);
        let wheel_rps = rps / self.gear_ratio[idx];
        wheel_rps * 2.0 * std::f64::consts::PI * self.wheel_radius[idx]
    }

    /// Send duty cycle command
    fn send_duty_command(&mut self, motor_id: u8, duty: i16, mut ctx: Option<&mut NodeInfo>) {
        let idx = (motor_id - 1) as usize;
        let clamped_duty = duty.clamp(-10000, 10000);

        // Try hardware first, fall back to simulation
        #[cfg(feature = "serial-hardware")]
        if self.hardware_enabled || self.hardware_port.is_some() {
            if self.hardware_port.is_none() {
                if let Err(e) = self.open_hardware(ctx.as_deref_mut()) {
                    // Provide detailed troubleshooting information (only log once)
                    if self.hardware_enabled
                        || (self.motor_states[0].encoder_count == 0
                            && self.motor_states[1].encoder_count == 0)
                    {
                        ctx.log_warning(
                            "RoboclawMotorNode: Hardware unavailable - using SIMULATION mode",
                        );
                        ctx.log_warning(&format!("  Tried: {}", self.serial_port));
                        ctx.log_warning(&format!("  Error: {}", e));
                        ctx.log_warning("  Fix:");
                        ctx.log_warning("    1. Check serial port: ls /dev/tty*");
                        ctx.log_warning(
                            "    2. Add user to dialout group: sudo usermod -a -G dialout $USER",
                        );
                        ctx.log_warning("    3. Verify Roboclaw power and USB connection");
                        ctx.log_warning("    4. Check device address (default 0x80)");
                        ctx.log_warning(
                            "    5. Rebuild with: cargo build --features=\"serial-hardware\"",
                        );
                    }
                    self.hardware_enabled = false;
                }
            }

            if self.hardware_enabled {
                match self.send_duty_hardware(motor_id, clamped_duty) {
                    Ok(()) => {
                        ctx.log_debug(&format!(
                            "Roboclaw 0x{:02X} M{} (HW): Duty = {} ({:.1}%)",
                            self.device_address,
                            motor_id,
                            clamped_duty,
                            clamped_duty as f64 / 100.0
                        ));

                        // Update state
                        self.motor_states[idx].duty_cycle = clamped_duty;
                        self.motor_states[idx].enabled = clamped_duty.abs() > 0;
                        self.motor_states[idx].direction = clamped_duty.signum() as i8;

                        self.command_count += 1;
                        self.last_command_time = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_nanos() as u64;
                        return;
                    }
                    Err(e) => {
                        ctx.log_warning(&format!(
                            "Roboclaw hardware command failed: {:?}, falling back to simulation",
                            e
                        ));
                        self.hardware_enabled = false;
                    }
                }
            }
        }

        // Simulation fallback
        ctx.log_debug(&format!(
            "Roboclaw 0x{:02X} M{} (SIM): Duty = {} ({:.1}%)",
            self.device_address,
            motor_id,
            clamped_duty,
            clamped_duty as f64 / 100.0
        ));

        // Update state
        self.motor_states[idx].duty_cycle = clamped_duty;
        self.motor_states[idx].enabled = clamped_duty.abs() > 0;
        self.motor_states[idx].direction = clamped_duty.signum() as i8;

        self.command_count += 1;
        self.last_command_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
    }

    /// Send velocity command
    fn send_velocity_command(&mut self, motor_id: u8, qpps: i32, mut ctx: Option<&mut NodeInfo>) {
        let idx = (motor_id - 1) as usize;
        let max_qpps = self.max_velocity[idx];
        let clamped_qpps = qpps.clamp(-max_qpps, max_qpps);

        // Try hardware first, fall back to simulation
        #[cfg(feature = "serial-hardware")]
        if self.hardware_enabled || self.hardware_port.is_some() {
            if self.hardware_port.is_none() {
                if let Err(e) = self.open_hardware(ctx.as_deref_mut()) {
                    // Provide detailed troubleshooting information (only log once)
                    if self.hardware_enabled
                        || (self.motor_states[0].encoder_count == 0
                            && self.motor_states[1].encoder_count == 0)
                    {
                        ctx.log_warning(
                            "RoboclawMotorNode: Hardware unavailable - using SIMULATION mode",
                        );
                        ctx.log_warning(&format!("  Tried: {}", self.serial_port));
                        ctx.log_warning(&format!("  Error: {}", e));
                        ctx.log_warning("  Fix:");
                        ctx.log_warning("    1. Check serial port: ls /dev/tty*");
                        ctx.log_warning(
                            "    2. Add user to dialout group: sudo usermod -a -G dialout $USER",
                        );
                        ctx.log_warning("    3. Verify Roboclaw power and USB connection");
                        ctx.log_warning("    4. Check device address (default 0x80)");
                        ctx.log_warning(
                            "    5. Rebuild with: cargo build --features=\"serial-hardware\"",
                        );
                    }
                    self.hardware_enabled = false;
                }
            }

            if self.hardware_enabled {
                match self.send_velocity_hardware(motor_id, clamped_qpps) {
                    Ok(()) => {
                        ctx.log_debug(&format!(
                            "Roboclaw 0x{:02X} M{} (HW): Velocity = {} QPPS ({:.2} m/s)",
                            self.device_address,
                            motor_id,
                            clamped_qpps,
                            self.qpps_to_velocity(motor_id, clamped_qpps)
                        ));

                        // Update state
                        self.motor_states[idx].velocity = clamped_qpps;
                        self.motor_states[idx].enabled = clamped_qpps.abs() > 0;
                        self.motor_states[idx].direction = clamped_qpps.signum() as i8;

                        self.command_count += 1;
                        self.last_command_time = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_nanos() as u64;
                        return;
                    }
                    Err(e) => {
                        ctx.log_warning(&format!(
                            "Roboclaw hardware command failed: {:?}, falling back to simulation",
                            e
                        ));
                        self.hardware_enabled = false;
                    }
                }
            }
        }

        // Simulation fallback
        ctx.log_debug(&format!(
            "Roboclaw 0x{:02X} M{} (SIM): Velocity = {} QPPS ({:.2} m/s)",
            self.device_address,
            motor_id,
            clamped_qpps,
            self.qpps_to_velocity(motor_id, clamped_qpps)
        ));

        // Update state
        self.motor_states[idx].velocity = clamped_qpps;
        self.motor_states[idx].enabled = clamped_qpps.abs() > 0;
        self.motor_states[idx].direction = clamped_qpps.signum() as i8;

        self.command_count += 1;
        self.last_command_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
    }

    /// Process motor command
    fn process_motor_command(
        &mut self,
        motor_id: u8,
        cmd: MotorCommand,
        mut ctx: Option<&mut NodeInfo>,
    ) {
        match cmd.mode {
            3 => {
                // Voltage/duty cycle mode (MODE 3)
                let duty = (cmd.target * 10000.0) as i16; // Scale -1.0..1.0 to -10000..10000
                self.send_duty_command(motor_id, duty, ctx.as_deref_mut());
            }
            0 => {
                // Velocity mode (MODE 0) - convert m/s to QPPS
                let idx = (motor_id - 1) as usize;
                let rps = cmd.target / (2.0 * std::f64::consts::PI * self.wheel_radius[idx]);
                let motor_rps = rps * self.gear_ratio[idx];
                let qpps = (motor_rps * self.encoder_resolution[idx] as f64 * 4.0) as i32;
                self.send_velocity_command(motor_id, qpps, ctx.as_deref_mut());
            }
            1 => {
                // Position mode (MODE 1)
                ctx.log_debug(&format!(
                    "Roboclaw M{}: Position command = {:.3} (not fully implemented in simulation)",
                    motor_id, cmd.target
                ));
            }
            _ => {
                ctx.log_warning(&format!(
                    "Unknown command mode {} for motor {}",
                    cmd.mode, motor_id
                ));
            }
        }
    }

    /// Simulate encoder updates based on velocity
    fn update_encoders(&mut self, dt: f32) {
        for i in 0..2 {
            if self.motor_states[i].enabled {
                // Simulate encoder increments based on velocity
                let delta = (self.motor_states[i].velocity as f64 * dt as f64) as i32;
                self.motor_states[i].encoder_count =
                    self.motor_states[i].encoder_count.wrapping_add(delta);

                // Simulate current draw (proportional to velocity)
                let velocity_ratio = self.motor_states[i].velocity.abs() as f64 / 44000.0;
                self.motor_states[i].current = velocity_ratio * self.max_current * 0.5;
            } else {
                self.motor_states[i].current = 0.0;
            }
        }

        // Update total current
        self.motor_currents[0] = self.motor_states[0].current;
        self.motor_currents[1] = self.motor_states[1].current;
        self.main_current = self.motor_currents[0] + self.motor_currents[1];
    }

    /// Publish motor feedback (through processor pipeline)
    fn publish_feedback(&mut self, motor_id: u8, mut ctx: Option<&mut NodeInfo>) {
        let idx = (motor_id - 1) as usize;
        let state = &self.motor_states[idx];

        let feedback = RoboclawFeedback {
            motor_id,
            encoder_count: state.encoder_count,
            velocity: state.velocity,
            duty_cycle: state.duty_cycle,
            current: state.current,
            position: self.encoder_to_position(motor_id, state.encoder_count),
            linear_velocity: self.qpps_to_velocity(motor_id, state.velocity),
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };

        // Process through pipeline (filter/transform)
        if let Some(processed) = self.processor.process(feedback) {
            let publisher = if motor_id == 1 {
                &mut self.motor1_feedback_pub
            } else {
                &mut self.motor2_feedback_pub
            };

            if let Err(e) = publisher.send(processed, &mut None) {
                ctx.log_error(&format!(
                    "Failed to publish motor {} feedback: {:?}",
                    motor_id, e
                ));
            }

            self.last_feedback_time[idx] = feedback.timestamp;
        }
    }

    /// Publish diagnostics
    fn publish_diagnostics(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let diag = RoboclawDiagnostics {
            battery_voltage: self.battery_voltage,
            main_current: self.main_current,
            motor1_current: self.motor_currents[0],
            motor2_current: self.motor_currents[1],
            temperature1: self.temperatures[0],
            temperature2: self.temperatures[1],
            error_status: self.error_status,
            warning_flags: 0,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };

        if let Err(e) = self.diagnostic_pub.send(diag, &mut None) {
            ctx.log_error(&format!("Failed to publish diagnostics: {:?}", e));
        }
    }

    // ========== Hardware Backend Functions ==========

    /// Open serial port for Roboclaw communication
    #[cfg(feature = "serial-hardware")]
    fn open_hardware(&mut self, mut ctx: Option<&mut NodeInfo>) -> std::io::Result<()> {
        let port = serialport::new(&self.serial_port, self.baud_rate)
            .timeout(Duration::from_millis(self.timeout_ms as u64))
            .open()?;

        self.hardware_port = Some(port);
        self.hardware_enabled = true;

        ctx.log_info(&format!(
            "Opened Roboclaw port: {} @ {} bps, address 0x{:02X}",
            self.serial_port, self.baud_rate, self.device_address
        ));

        Ok(())
    }

    /// Calculate CRC-16 for Roboclaw protocol
    #[cfg(feature = "serial-hardware")]
    fn crc16_roboclaw(data: &[u8]) -> u16 {
        let mut crc: u16 = 0;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if (crc & 0x8000) != 0 {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    /// Send command packet with CRC-16 checksum
    #[cfg(feature = "serial-hardware")]
    fn send_command_hardware(&mut self, command: u8, data: &[u8]) -> std::io::Result<()> {
        use std::io::Write;

        let port = self.hardware_port.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Serial port not open")
        })?;

        // Build packet: [Address] [Command] [Data...] [CRC-16 MSB] [CRC-16 LSB]
        let mut packet = vec![self.device_address, command];
        packet.extend_from_slice(data);

        let crc = Self::crc16_roboclaw(&packet);
        packet.push((crc >> 8) as u8); // CRC MSB
        packet.push((crc & 0xFF) as u8); // CRC LSB

        port.write_all(&packet)?;
        port.flush()?;

        Ok(())
    }

    /// Send duty cycle command via hardware
    #[cfg(feature = "serial-hardware")]
    fn send_duty_hardware(&mut self, motor_id: u8, duty: i16) -> std::io::Result<()> {
        // Roboclaw duty cycle range is 0-32767
        // Convert from -10000..10000 to direction + magnitude
        let abs_duty = duty.unsigned_abs();
        let magnitude = ((abs_duty as u32 * 32767) / 10000) as u16;

        // Command IDs:
        // M1 Forward: 0, M1 Backward: 1
        // M2 Forward: 4, M2 Backward: 5
        let command = if motor_id == 1 {
            if duty >= 0 {
                0
            } else {
                1
            }
        } else if duty >= 0 {
            4
        } else {
            5
        };

        // Data: 2 bytes (magnitude, big-endian)
        let data = [((magnitude >> 8) & 0xFF) as u8, (magnitude & 0xFF) as u8];

        self.send_command_hardware(command, &data)?;
        Ok(())
    }

    /// Send velocity command via hardware
    #[cfg(feature = "serial-hardware")]
    fn send_velocity_hardware(&mut self, motor_id: u8, qpps: i32) -> std::io::Result<()> {
        // Roboclaw velocity commands:
        // M1 Speed: 35, M2 Speed: 36
        let command = if motor_id == 1 { 35 } else { 36 };

        // Data: 4 bytes (signed QPPS, big-endian)
        let qpps_abs = qpps.unsigned_abs();
        let sign = if qpps >= 0 { 0u8 } else { 0x80 };

        let data = [
            sign | ((qpps_abs >> 24) & 0x7F) as u8,
            ((qpps_abs >> 16) & 0xFF) as u8,
            ((qpps_abs >> 8) & 0xFF) as u8,
            (qpps_abs & 0xFF) as u8,
        ];

        self.send_command_hardware(command, &data)?;
        Ok(())
    }

    /// Simulate battery discharge
    fn simulate_battery(&mut self, dt: f32) {
        // Start with nominal voltage
        if self.battery_voltage == 0.0 {
            self.battery_voltage = 24.0; // 24V nominal
        }

        // Simulate voltage drop under load
        let voltage_drop = self.main_current * 0.1; // 0.1 ohm internal resistance
        self.battery_voltage = (24.0 - voltage_drop).max(20.0);

        // Simulate temperature increase with current
        for i in 0..2 {
            if self.motor_states[i].enabled {
                let heating_rate = self.motor_currents[i] * 0.05;
                self.temperatures[i] = (self.temperatures[i] + heating_rate * dt as f64).min(80.0);
            } else {
                // Cooling when idle
                let cooling_rate = (self.temperatures[i] - 25.0) * 0.1;
                self.temperatures[i] = (self.temperatures[i] - cooling_rate * dt as f64).max(25.0);
            }
        }
    }
}

impl<P> Node for RoboclawMotorNode<P>
where
    P: Processor<RoboclawFeedback>,
{
    fn name(&self) -> &'static str {
        "RoboclawMotorNode"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        let dt = 0.02; // Assume 50Hz tick rate

        // Process motor 1 commands
        if let Some(cmd) = self.motor1_cmd_sub.recv(&mut None) {
            self.process_motor_command(1, cmd, ctx.as_deref_mut());
        }

        // Process motor 2 commands
        if let Some(cmd) = self.motor2_cmd_sub.recv(&mut None) {
            self.process_motor_command(2, cmd, ctx.as_deref_mut());
        }

        // Update simulation
        self.update_encoders(dt);
        self.simulate_battery(dt);

        // Publish feedback at 50Hz (through processor pipeline)
        self.feedback_counter += 1;
        // Publish motor feedback every tick
        self.publish_feedback(1, ctx.as_deref_mut());
        self.publish_feedback(2, ctx.as_deref_mut());

        // Publish diagnostics at 10Hz
        if self.feedback_counter % 5 == 0 {
            self.publish_diagnostics(ctx.as_deref_mut());
        }

        // Periodic status logging at 1Hz
        if self.feedback_counter % 50 == 0 {
            ctx.log_info(&format!(
                "Roboclaw 0x{:02X}: M1={} QPPS ({:.2}A), M2={} QPPS ({:.2}A), Bat={:.1}V",
                self.device_address,
                self.motor_states[0].velocity,
                self.motor_currents[0],
                self.motor_states[1].velocity,
                self.motor_currents[1],
                self.battery_voltage
            ));
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("RoboclawMotorNode shutting down - stopping all motors");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Stop both motors by setting velocity to 0
        self.motor_states[0].velocity = 0;
        self.motor_states[0].duty_cycle = 0;
        self.motor_states[0].enabled = false;
        self.motor_states[1].velocity = 0;
        self.motor_states[1].duty_cycle = 0;
        self.motor_states[1].enabled = false;

        // If hardware mode, send stop command over serial
        #[cfg(feature = "serial-hardware")]
        if let Some(ref mut port) = self.hardware_port {
            use std::io::Write;
            // Roboclaw command 0 with duty 0 = stop motor 1, command 4 = stop motor 2
            let stop_m1 = [self.device_address, 0, 0, 0];
            let stop_m2 = [self.device_address, 4, 0, 0];
            let _ = port.write_all(&stop_m1);
            let _ = port.write_all(&stop_m2);
            let _ = port.flush();
        }

        ctx.log_info("Roboclaw motors stopped safely");
        Ok(())
    }
}

/// Preset configurations for common applications
impl RoboclawMotorNode {
    /// Configure for differential drive robot
    pub fn configure_differential_drive(
        &mut self,
        wheel_radius: f64,
        encoder_ppr: u32,
        gear_ratio: f64,
    ) {
        // Both motors use same configuration
        for motor_id in 1..=2 {
            self.set_encoder_resolution(motor_id, encoder_ppr);
            self.set_gear_ratio(motor_id, gear_ratio);
            self.set_wheel_radius(motor_id, wheel_radius);
            self.set_velocity_pid(motor_id, 1.0, 0.5, 0.25, 44000);
        }
    }

    /// Configure for high-power application (2x60A or higher)
    pub fn configure_high_power(&mut self) {
        self.set_max_current(60.0);
        for motor_id in 1..=2 {
            self.set_max_velocity(motor_id, 100000);
            self.set_max_acceleration(motor_id, 20000);
        }
    }

    /// Configure for precision positioning
    pub fn configure_precision(&mut self) {
        for motor_id in 1..=2 {
            self.set_velocity_pid(motor_id, 2.0, 1.0, 0.5, 20000);
            self.set_position_pid(motor_id, 5.0, 0.1, 1.0, 10000);
            self.set_max_acceleration(motor_id, 5000);
        }
    }
}

/// Implement LogSummary for RoboclawFeedback
impl LogSummary for RoboclawFeedback {
    fn log_summary(&self) -> String {
        format!(
            "M{}: enc={} vel={} duty={} cur={:.2}A pos={:.3} linvel={:.3}",
            self.motor_id,
            self.encoder_count,
            self.velocity,
            self.duty_cycle,
            self.current,
            self.position,
            self.linear_velocity
        )
    }
}

/// Implement LogSummary for RoboclawDiagnostics
impl LogSummary for RoboclawDiagnostics {
    fn log_summary(&self) -> String {
        format!(
            "Bat={:.1}V I={:.2}A M1={:.2}A M2={:.2}A T1={:.1}°C T2={:.1}°C err=0x{:04X}",
            self.battery_voltage,
            self.main_current,
            self.motor1_current,
            self.motor2_current,
            self.temperature1,
            self.temperature2,
            self.error_status
        )
    }
}

/// Builder for RoboclawMotorNode with custom processor
pub struct RoboclawMotorNodeBuilder<P>
where
    P: Processor<RoboclawFeedback>,
{
    serial_port: String,
    device_address: u8,
    baud_rate: u32,
    processor: P,
}

impl RoboclawMotorNodeBuilder<PassThrough<RoboclawFeedback>> {
    /// Create a new builder with default configuration
    pub fn new(serial_port: &str, address: u8) -> Self {
        Self {
            serial_port: serial_port.to_string(),
            device_address: address,
            baud_rate: 38400,
            processor: PassThrough::new(),
        }
    }
}

impl<P> RoboclawMotorNodeBuilder<P>
where
    P: Processor<RoboclawFeedback>,
{
    /// Set baud rate
    pub fn with_baud_rate(mut self, baud_rate: u32) -> Self {
        self.baud_rate = baud_rate;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> RoboclawMotorNodeBuilder<P2>
    where
        P2: Processor<RoboclawFeedback>,
    {
        RoboclawMotorNodeBuilder {
            serial_port: self.serial_port,
            device_address: self.device_address,
            baud_rate: self.baud_rate,
            processor,
        }
    }

    /// Add a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> RoboclawMotorNodeBuilder<ClosureProcessor<RoboclawFeedback, RoboclawFeedback, F>>
    where
        F: FnMut(RoboclawFeedback) -> RoboclawFeedback + Send + 'static,
    {
        RoboclawMotorNodeBuilder {
            serial_port: self.serial_port,
            device_address: self.device_address,
            baud_rate: self.baud_rate,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> RoboclawMotorNodeBuilder<FilterProcessor<RoboclawFeedback, RoboclawFeedback, F>>
    where
        F: FnMut(RoboclawFeedback) -> Option<RoboclawFeedback> + Send + 'static,
    {
        RoboclawMotorNodeBuilder {
            serial_port: self.serial_port,
            device_address: self.device_address,
            baud_rate: self.baud_rate,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (same output type constraint)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> RoboclawMotorNodeBuilder<
        Pipeline<RoboclawFeedback, RoboclawFeedback, RoboclawFeedback, P, P2>,
    >
    where
        P2: Processor<RoboclawFeedback, RoboclawFeedback>,
    {
        RoboclawMotorNodeBuilder {
            serial_port: self.serial_port,
            device_address: self.device_address,
            baud_rate: self.baud_rate,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<RoboclawMotorNode<P>> {
        if !(0x80..=0x87).contains(&self.device_address) {
            return Err(horus_core::error::HorusError::config(format!(
                "Invalid Roboclaw address: 0x{:02X}. Must be 0x80-0x87",
                self.device_address
            )));
        }

        Ok(RoboclawMotorNode {
            motor1_cmd_sub: Topic::new("roboclaw.motor1.cmd")?,
            motor2_cmd_sub: Topic::new("roboclaw.motor2.cmd")?,
            motor1_feedback_pub: Topic::new("roboclaw.motor1.feedback")?,
            motor2_feedback_pub: Topic::new("roboclaw.motor2.feedback")?,
            diagnostic_pub: Topic::new("roboclaw.diagnostics")?,
            device_address: self.device_address,
            baud_rate: self.baud_rate,
            serial_port: self.serial_port,
            timeout_ms: 100,
            #[cfg(feature = "serial-hardware")]
            hardware_port: None,
            hardware_enabled: false,
            motor_states: [MotorState::default(); 2],
            velocity_pid: [PidParams::default(); 2],
            position_pid: [PidParams::default(); 2],
            encoder_resolution: [1024, 1024],
            gear_ratio: [1.0, 1.0],
            wheel_radius: [0.05, 0.05],
            max_current: 30.0,
            max_velocity: [44000, 44000],
            max_acceleration: [10000, 10000],
            battery_voltage: 0.0,
            main_current: 0.0,
            motor_currents: [0.0, 0.0],
            temperatures: [25.0, 25.0],
            error_status: 0,
            last_command_time: 0,
            command_count: 0,
            last_feedback_time: [0, 0],
            feedback_counter: 0,
            processor: self.processor,
        })
    }
}
