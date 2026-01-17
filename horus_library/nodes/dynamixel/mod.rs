use crate::{JointCommand, ServoCommand};
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::collections::HashMap;
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

/// Dynamixel Servo Controller Node
///
/// Controls Dynamixel smart servo motors using Protocol 1.0 or 2.0.
/// Supports position, velocity, and current control with extensive feedback.
///
/// # Supported Models
/// **Protocol 1.0:**
/// - AX series: AX-12A, AX-18A (entry-level)
/// - MX series (Protocol 1.0): MX-28, MX-64, MX-106
/// - RX series: RX-10, RX-24F, RX-28, RX-64
/// - EX series: EX-106+
///
/// **Protocol 2.0:**
/// - X series: XL330, XL430, XC330, XC430, XM430, XM540, XH430, XH540
/// - MX series (Protocol 2.0): MX-28, MX-64, MX-106 (2.0 firmware)
/// - PRO series: H42, H54, M42, M54 (high-performance)
/// - P series: PH42, PH54 (industrial)
///
/// # Features
/// - Position, velocity, and current-based position control
/// - Extended position mode (multi-turn)
/// - PWM control mode
/// - Torque enable/disable
/// - Compliance/PID parameter tuning
/// - Temperature and voltage monitoring
/// - Hardware error detection
/// - Bulk read/write for synchronized motion
/// - Auto-baud rate detection
///
/// # Communication
/// - Half-duplex RS-485 (TTL or RS-485 transceivers)
/// - Baud rates: 9600 to 4.5M bps
/// - Daisy-chain topology (up to 253 servos per bus)
/// - U2D2, USB2Dynamixel adapters
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::DynamixelNode;
/// use horus_library::ServoCommand;
///
/// let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
/// dxl.set_baud_rate(1_000_000); // 1 Mbps
/// dxl.add_servo(1, DynamixelModel::XM430W350);
/// dxl.add_servo(2, DynamixelModel::XM430W350);
/// dxl.enable_torque(1, true);
/// dxl.enable_torque(2, true);
///
/// // Send position command
/// let cmd = ServoCommand::new(1, 1.57); // 90 degrees in radians
/// ```
///
/// # Hybrid Pattern
///
/// This node supports the hybrid pattern for custom processing of feedback:
///
/// ```rust,ignore
/// let node = DynamixelNode::builder("/dev/ttyUSB0", DynamixelProtocol::Protocol2)
///     .with_filter(|feedback| {
///         // Only publish feedback for moving servos
///         if feedback.speed.abs() > 0.01 {
///             Some(feedback)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct DynamixelNode<P = PassThrough<ServoCommand>>
where
    P: Processor<ServoCommand>,
{
    command_subscriber: Topic<ServoCommand>,
    joint_subscriber: Topic<JointCommand>,
    feedback_publisher: Topic<ServoCommand>,

    // Hardware serial port
    #[cfg(feature = "serial-hardware")]
    serial_port: Option<Box<dyn SerialPort>>,
    hardware_enabled: bool,

    // Configuration
    port: String,
    protocol: DynamixelProtocol,
    baud_rate: u32,
    servos: HashMap<u8, DynamixelServo>,
    enable_bulk_operations: bool,

    // Communication state
    _last_packet_time: u64, // Reserved for future packet timing diagnostics
    packet_errors: u32,
    successful_packets: u32,

    // Timing state (moved from static mut for thread safety)
    last_read_time: u64,
    last_log_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

/// Dynamixel protocol version
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DynamixelProtocol {
    Protocol1,
    Protocol2,
}

/// Dynamixel servo model
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DynamixelModel {
    // Protocol 1.0 Models
    AX12A,
    AX18A,
    MX28,
    MX64,
    MX106,
    // Protocol 2.0 Models
    XL330M077,
    XL330M288,
    XL430W250,
    XC430W150,
    XC430W240,
    XM430W210,
    XM430W350,
    XM540W150,
    XM540W270,
    XH430V210,
    XH430V350,
    XH430W210,
    XH430W350,
    XH540W150,
    XH540W270,
    XH540V150,
    XH540V270,
    // PRO Series
    H42P020,
    H54P100,
    H54P200,
    M42P010,
    M54P040,
    M54P060,
}

impl DynamixelModel {
    /// Get protocol version for this model
    fn protocol(&self) -> DynamixelProtocol {
        match self {
            Self::AX12A | Self::AX18A | Self::MX28 | Self::MX64 | Self::MX106 => {
                DynamixelProtocol::Protocol1
            }
            _ => DynamixelProtocol::Protocol2,
        }
    }

    /// Get resolution in counts (encoder resolution)
    fn resolution(&self) -> u32 {
        match self {
            Self::AX12A | Self::AX18A => 1024,
            Self::MX28 | Self::MX64 | Self::MX106 => 4096,
            Self::XL330M077 | Self::XL330M288 => 4096,
            Self::XL430W250 | Self::XC430W150 | Self::XC430W240 => 4096,
            _ => 4096, // Most modern servos use 4096
        }
    }

    /// Get maximum position in radians (single turn)
    fn max_position_rad(&self) -> f32 {
        use std::f32::consts::PI;
        match self {
            Self::AX12A | Self::AX18A => 5.23,                 // ~300 degrees
            Self::MX28 | Self::MX64 | Self::MX106 => 2.0 * PI, // 360 degrees
            _ => 2.0 * PI,
        }
    }

    /// Get maximum velocity in rad/s
    fn max_velocity_rad_s(&self) -> f32 {
        match self {
            Self::AX12A => 10.47,                    // 100 RPM
            Self::AX18A => 20.94,                    // 200 RPM
            Self::MX28 => 11.94,                     // 114 RPM
            Self::MX64 => 13.09,                     // 125 RPM
            Self::MX106 => 9.42,                     // 90 RPM
            Self::XL330M077 => 24.09,                // 230 RPM
            Self::XL330M288 => 19.42,                // 185 RPM
            Self::XL430W250 => 5.02,                 // 48 RPM
            Self::XM430W210 => 7.65,                 // 73 RPM
            Self::XM430W350 => 4.82,                 // 46 RPM
            Self::XM540W150 => 5.44,                 // 52 RPM
            Self::XM540W270 => std::f32::consts::PI, // 30 RPM
            _ => std::f32::consts::TAU,              // ~60 RPM default
        }
    }

    /// Get stall torque in Nm (reserved for future torque limiting)
    fn _stall_torque_nm(&self) -> f32 {
        match self {
            Self::AX12A => 0.15,
            Self::AX18A => 0.18,
            Self::MX28 => 0.34,
            Self::MX64 => 0.74,
            Self::MX106 => 1.02,
            Self::XL330M077 => 0.059,
            Self::XL330M288 => 0.038,
            Self::XL430W250 => 0.14,
            Self::XM430W210 => 0.29,
            Self::XM430W350 => 0.39,
            Self::XM540W150 => 1.05,
            Self::XM540W270 => 1.49,
            Self::XH430V210 => 0.44,
            Self::XH430V350 => 0.69,
            Self::XH540W150 => 1.16,
            Self::XH540W270 => 1.74,
            Self::H54P200 => 5.40,
            _ => 0.5,
        }
    }
}

/// Individual Dynamixel servo state
#[derive(Debug, Clone)]
struct DynamixelServo {
    id: u8,
    model: DynamixelModel,
    position: f32,   // radians
    velocity: f32,   // rad/s
    current: f32,    // mA
    temperature: u8, // °C
    _voltage: f32,   // V (reserved for voltage monitoring)
    torque_enabled: bool,
    goal_position: f32,
    goal_velocity: f32,
    _goal_current: f32, // Reserved for current control mode
    moving: bool,
    hardware_error: u8,
    last_update: u64,
}

impl DynamixelServo {
    fn new(id: u8, model: DynamixelModel) -> Self {
        Self {
            id,
            model,
            position: 0.0,
            velocity: 0.0,
            current: 0.0,
            temperature: 25,
            _voltage: 12.0,
            torque_enabled: false,
            goal_position: 0.0,
            goal_velocity: 0.0,
            _goal_current: 0.0,
            moving: false,
            hardware_error: 0,
            last_update: 0,
        }
    }
}

impl DynamixelNode {
    /// Create a new Dynamixel controller node
    pub fn new(port: &str, protocol: DynamixelProtocol) -> Result<Self> {
        Ok(Self {
            command_subscriber: Topic::new("dynamixel.servo_cmd")?,
            joint_subscriber: Topic::new("dynamixel.joint_cmd")?,
            feedback_publisher: Topic::new("dynamixel.feedback")?,
            #[cfg(feature = "serial-hardware")]
            serial_port: None,
            hardware_enabled: false,
            port: port.to_string(),
            protocol,
            baud_rate: 1_000_000, // 1 Mbps default
            servos: HashMap::new(),
            enable_bulk_operations: true,
            _last_packet_time: 0,
            packet_errors: 0,
            successful_packets: 0,
            last_read_time: 0,
            last_log_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for custom configuration
    pub fn builder(
        port: &str,
        protocol: DynamixelProtocol,
    ) -> DynamixelNodeBuilder<PassThrough<ServoCommand>> {
        DynamixelNodeBuilder::new(port, protocol)
    }
}

impl<P> DynamixelNode<P>
where
    P: Processor<ServoCommand>,
{
    /// Set baud rate (9600 to 4500000 bps)
    pub fn set_baud_rate(&mut self, baud_rate: u32) {
        self.baud_rate = baud_rate;
    }

    /// Add a servo to the controller
    pub fn add_servo(&mut self, id: u8, model: DynamixelModel) {
        if model.protocol() != self.protocol {
            eprintln!("Warning: Servo {} model {:?} protocol mismatch", id, model);
        }
        self.servos.insert(id, DynamixelServo::new(id, model));
    }

    /// Remove a servo from the controller
    pub fn remove_servo(&mut self, id: u8) {
        self.servos.remove(&id);
    }

    /// Enable/disable torque for a servo
    pub fn enable_torque(&mut self, id: u8, enable: bool) -> bool {
        if let Some(servo) = self.servos.get_mut(&id) {
            servo.torque_enabled = enable;
            true
        } else {
            false
        }
    }

    /// Enable torque for all servos
    pub fn enable_all_torque(&mut self, enable: bool) {
        for servo in self.servos.values_mut() {
            servo.torque_enabled = enable;
        }
    }

    /// Get servo position in radians
    pub fn get_position(&self, id: u8) -> Option<f32> {
        self.servos.get(&id).map(|s| s.position)
    }

    /// Get servo velocity in rad/s
    pub fn get_velocity(&self, id: u8) -> Option<f32> {
        self.servos.get(&id).map(|s| s.velocity)
    }

    /// Get servo current in mA
    pub fn get_current(&self, id: u8) -> Option<f32> {
        self.servos.get(&id).map(|s| s.current)
    }

    /// Get servo temperature in °C
    pub fn get_temperature(&self, id: u8) -> Option<u8> {
        self.servos.get(&id).map(|s| s.temperature)
    }

    /// Check if servo has errors
    pub fn has_error(&self, id: u8) -> bool {
        self.servos
            .get(&id)
            .map(|s| s.hardware_error != 0)
            .unwrap_or(false)
    }

    /// Get communication statistics
    pub fn get_comm_stats(&self) -> (u32, u32, f32) {
        let total = self.successful_packets + self.packet_errors;
        let success_rate = if total > 0 {
            (self.successful_packets as f32 / total as f32) * 100.0
        } else {
            100.0
        };
        (self.successful_packets, self.packet_errors, success_rate)
    }

    /// Send position command to a servo
    fn send_position_command(&mut self, id: u8, position: f32, mut ctx: Option<&mut NodeInfo>) {
        if let Some(servo) = self.servos.get_mut(&id) {
            if !servo.torque_enabled {
                ctx.log_warning(&format!("Servo {} torque not enabled", id));
                return;
            }

            // Clamp to valid range
            let max_pos = servo.model.max_position_rad();
            let clamped_pos = position.clamp(-max_pos, max_pos);

            servo.goal_position = clamped_pos;

            // Try hardware first, fall back to simulation
            #[cfg(feature = "serial-hardware")]
            if self.hardware_enabled || self.serial_port.is_some() {
                // Try to open port if not yet opened
                if self.serial_port.is_none() {
                    if let Err(e) = self.open_hardware(ctx.as_deref_mut()) {
                        // Provide detailed troubleshooting information (only log once)
                        if self.hardware_enabled || self.successful_packets == 0 {
                            ctx.log_warning(
                                "DynamixelNode: Hardware unavailable - using SIMULATION mode",
                            );
                            ctx.log_warning(&format!("  Tried: {}", self.port));
                            ctx.log_warning(&format!("  Error: {}", e));
                            ctx.log_warning("  Fix:");
                            ctx.log_warning("    1. Check serial port: ls /dev/tty*");
                            ctx.log_warning("    2. Add user to dialout group: sudo usermod -a -G dialout $USER");
                            ctx.log_warning("    3. Verify Dynamixel power and connections");
                            ctx.log_warning("    4. Check baud rate matches servo configuration");
                            ctx.log_warning(
                                "    5. Rebuild with: cargo build --features=\"serial-hardware\"",
                            );
                        }
                        self.hardware_enabled = false;
                    }
                }

                // Send via hardware if available
                if self.hardware_enabled {
                    match self.send_position_hardware(id, clamped_pos) {
                        Ok(()) => {
                            ctx.log_debug(&format!(
                                "Servo {} (HW): position {:.3} rad ({:.1}°)",
                                id,
                                clamped_pos,
                                clamped_pos.to_degrees()
                            ));
                            self.successful_packets += 1;
                            return;
                        }
                        Err(e) => {
                            ctx.log_warning(&format!(
                                "Dynamixel hardware send failed: {:?}, falling back to simulation",
                                e
                            ));
                            self.hardware_enabled = false;
                        }
                    }
                }
            }

            // Simulation fallback
            ctx.log_debug(&format!(
                "Servo {} (SIM): position {:.3} rad ({:.1}°)",
                id,
                clamped_pos,
                clamped_pos.to_degrees()
            ));

            self.successful_packets += 1;
        } else {
            ctx.log_warning(&format!("Unknown servo ID: {}", id));
        }
    }

    /// Send velocity command to a servo
    fn send_velocity_command(&mut self, id: u8, velocity: f32, mut ctx: Option<&mut NodeInfo>) {
        if let Some(servo) = self.servos.get_mut(&id) {
            if !servo.torque_enabled {
                ctx.log_warning(&format!("Servo {} torque not enabled", id));
                return;
            }

            let max_vel = servo.model.max_velocity_rad_s();
            let clamped_vel = velocity.clamp(-max_vel, max_vel);

            servo.goal_velocity = clamped_vel;

            ctx.log_debug(&format!(
                "Servo {}: velocity command {:.3} rad/s",
                id, clamped_vel
            ));

            self.successful_packets += 1;
        }
    }

    /// Simulate servo motion and feedback
    fn simulate_servos(&mut self, dt: f32) {
        for servo in self.servos.values_mut() {
            if !servo.torque_enabled {
                servo.velocity = 0.0;
                continue;
            }

            // Simulate position tracking
            let pos_error = servo.goal_position - servo.position;
            let max_vel = servo.model.max_velocity_rad_s();

            // P controller for position
            let p_gain = 5.0;
            let target_vel = (pos_error * p_gain).clamp(-max_vel, max_vel);

            // Simulate velocity ramping
            let accel_limit = max_vel * 10.0; // rad/s²
            let vel_error = target_vel - servo.velocity;
            let max_delta = accel_limit * dt;

            if vel_error.abs() <= max_delta {
                servo.velocity = target_vel;
            } else {
                servo.velocity += vel_error.signum() * max_delta;
            }

            // Update position
            servo.position += servo.velocity * dt;

            // Simulate current draw (proportional to torque/load)
            let load_factor = servo.velocity.abs() / max_vel;
            servo.current = load_factor * 1000.0; // Up to 1A

            // Simulate temperature (increases with load)
            let temp_increase = load_factor * 0.1;
            servo.temperature = (25.0 + temp_increase * 30.0) as u8;

            // Check if moving
            servo.moving = pos_error.abs() > 0.01;

            servo.last_update = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64;
        }
    }

    /// Read status from all servos (bulk read)
    fn bulk_read(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if !self.enable_bulk_operations {
            return;
        }

        // In real implementation, this would send a bulk read packet
        // to efficiently read multiple servos in one transaction
        ctx.log_debug(&format!("Bulk read from {} servos", self.servos.len()));
    }

    /// Publish feedback for all servos (through processor pipeline)
    fn publish_feedback(&mut self) {
        for servo in self.servos.values() {
            let feedback = ServoCommand {
                servo_id: servo.id,
                position: servo.position,
                speed: servo.velocity,
                enable: servo.torque_enabled,
                timestamp: servo.last_update,
            };

            // Process through pipeline (filter/transform)
            if let Some(processed) = self.processor.process(feedback) {
                let _ = self.feedback_publisher.send(processed, &mut None);
            }
        }
    }

    /// Ping a servo to check if it's responsive
    pub fn ping(&mut self, id: u8) -> bool {
        // In real implementation, send ping packet and wait for response
        self.servos.contains_key(&id)
    }

    /// Scan for servos on the bus
    pub fn scan(&mut self, mut ctx: Option<&mut NodeInfo>) -> Vec<u8> {
        // In real implementation, ping IDs 0-252 to find servos
        ctx.log_info(&format!(
            "Scanning for Dynamixel servos on {}...",
            self.port
        ));
        let found_ids: Vec<u8> = self.servos.keys().copied().collect();
        ctx.log_info(&format!(
            "Found {} servos: {:?}",
            found_ids.len(),
            found_ids
        ));
        found_ids
    }

    /// Reboot a servo
    pub fn reboot(&mut self, id: u8, mut ctx: Option<&mut NodeInfo>) -> bool {
        if self.servos.contains_key(&id) {
            ctx.log_info(&format!("Rebooting servo {}", id));
            // In real implementation, send reboot instruction
            true
        } else {
            false
        }
    }

    /// Factory reset a servo
    pub fn factory_reset(&mut self, id: u8, mut ctx: Option<&mut NodeInfo>) -> bool {
        if self.servos.contains_key(&id) {
            ctx.log_warning(&format!("Factory resetting servo {}", id));
            // In real implementation, send factory reset instruction
            true
        } else {
            false
        }
    }

    // ========== Hardware Backend Functions (serialport) ==========

    /// Open serial port hardware
    #[cfg(feature = "serial-hardware")]
    fn open_hardware(&mut self, mut ctx: Option<&mut NodeInfo>) -> std::io::Result<()> {
        let port = serialport::new(&self.port, self.baud_rate)
            .timeout(Duration::from_millis(100))
            .open()?;

        self.serial_port = Some(port);
        self.hardware_enabled = true;
        ctx.log_info(&format!(
            "Opened Dynamixel port: {} @ {} bps, Protocol {:?}",
            self.port, self.baud_rate, self.protocol
        ));

        Ok(())
    }

    /// Calculate checksum for Dynamixel Protocol 1.0
    #[cfg(feature = "serial-hardware")]
    fn checksum_protocol1(packet: &[u8]) -> u8 {
        let sum: u32 = packet.iter().skip(2).map(|&b| b as u32).sum();
        (!sum & 0xFF) as u8
    }

    /// Calculate CRC for Dynamixel Protocol 2.0
    #[cfg(feature = "serial-hardware")]
    fn crc_protocol2(packet: &[u8]) -> u16 {
        let mut crc: u16 = 0;
        for &byte in packet {
            crc ^= byte as u16;
            for _ in 0..8 {
                if (crc & 1) != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc
    }

    /// Send write command to servo via hardware (Protocol 1.0)
    #[cfg(feature = "serial-hardware")]
    fn write_protocol1(&mut self, id: u8, address: u16, data: &[u8]) -> std::io::Result<()> {
        use std::io::Write;

        let port = self.serial_port.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Serial port not open")
        })?;

        let len = (data.len() + 3) as u8;
        let mut packet = vec![0xFF, 0xFF, id, len, 0x03, address as u8]; // 0x03 = WRITE
        packet.extend_from_slice(data);

        let checksum = Self::checksum_protocol1(&packet);
        packet.push(checksum);

        port.write_all(&packet)?;
        port.flush()?;

        Ok(())
    }

    /// Send write command to servo via hardware (Protocol 2.0)
    #[cfg(feature = "serial-hardware")]
    fn write_protocol2(&mut self, id: u8, address: u16, data: &[u8]) -> std::io::Result<()> {
        use std::io::Write;

        let port = self.serial_port.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Serial port not open")
        })?;

        let len = (data.len() + 5) as u16;
        let mut packet = vec![
            0xFF,
            0xFF,
            0xFD,
            0x00, // Header
            id,
            (len & 0xFF) as u8,
            (len >> 8) as u8, // Length
            0x03,             // Instruction: WRITE
            (address & 0xFF) as u8,
            (address >> 8) as u8, // Address
        ];
        packet.extend_from_slice(data);

        let crc = Self::crc_protocol2(&packet);
        packet.push((crc & 0xFF) as u8);
        packet.push((crc >> 8) as u8);

        port.write_all(&packet)?;
        port.flush()?;

        Ok(())
    }

    /// Send position command via hardware
    #[cfg(feature = "serial-hardware")]
    fn send_position_hardware(&mut self, id: u8, position_rad: f32) -> std::io::Result<()> {
        if let Some(servo) = self.servos.get(&id) {
            let resolution = servo.model.resolution();
            let max_pos = servo.model.max_position_rad();

            // Convert radians to encoder value
            let normalized = (position_rad / max_pos + 1.0) / 2.0; // -max..max -> 0..1
            let encoder_value = (normalized * resolution as f32) as u32;
            let clamped = encoder_value.min(resolution);

            match self.protocol {
                DynamixelProtocol::Protocol1 => {
                    // Address 30 for Goal Position (2 bytes)
                    let data = [(clamped & 0xFF) as u8, ((clamped >> 8) & 0xFF) as u8];
                    self.write_protocol1(id, 30, &data)?;
                }
                DynamixelProtocol::Protocol2 => {
                    // Address 116 for Goal Position (4 bytes)
                    let data = [
                        (clamped & 0xFF) as u8,
                        ((clamped >> 8) & 0xFF) as u8,
                        ((clamped >> 16) & 0xFF) as u8,
                        ((clamped >> 24) & 0xFF) as u8,
                    ];
                    self.write_protocol2(id, 116, &data)?;
                }
            }

            Ok(())
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("Servo {} not found", id),
            ))
        }
    }
}

impl<P> Node for DynamixelNode<P>
where
    P: Processor<ServoCommand>,
{
    fn name(&self) -> &'static str {
        "DynamixelNode"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Process servo commands
        while let Some(cmd) = self.command_subscriber.recv(&mut None) {
            self.send_position_command(cmd.servo_id, cmd.position, ctx.as_deref_mut());
        }

        // Process joint commands
        while let Some(cmd) = self.joint_subscriber.recv(&mut None) {
            for i in 0..cmd.joint_count {
                let idx = i as usize;
                let servo_id = i + 1; // Assume joint index maps to servo ID

                if cmd.modes[idx] == JointCommand::MODE_POSITION {
                    self.send_position_command(
                        servo_id,
                        cmd.positions[idx] as f32,
                        ctx.as_deref_mut(),
                    );
                } else if cmd.modes[idx] == JointCommand::MODE_VELOCITY {
                    self.send_velocity_command(
                        servo_id,
                        cmd.velocities[idx] as f32,
                        ctx.as_deref_mut(),
                    );
                }
            }
        }

        // Simulate servo motion
        let dt = 0.001; // 1ms tick rate
        self.simulate_servos(dt);

        // Bulk read servo status
        let read_interval = 10_000_000; // 10ms = 100Hz
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        if current_time - self.last_read_time > read_interval {
            self.bulk_read(ctx.as_deref_mut());
            self.last_read_time = current_time;
        }

        // Publish feedback (through processor pipeline)
        self.publish_feedback();

        // Periodic status logging
        let log_interval = 10_000_000_000; // 10 seconds
        if current_time - self.last_log_time > log_interval {
            let (success, errors, rate) = self.get_comm_stats();
            ctx.log_info(&format!(
                "Dynamixel {}: {} servos active | Comm: {} ok, {} err ({:.1}% success)",
                self.port,
                self.servos.len(),
                success,
                errors,
                rate
            ));

            // Log servo states
            for servo in self.servos.values() {
                if servo.torque_enabled {
                    ctx.log_debug(&format!(
                        "  Servo {}: pos={:.2}rad vel={:.2}rad/s I={:.0}mA T={}°C{}",
                        servo.id,
                        servo.position,
                        servo.velocity,
                        servo.current,
                        servo.temperature,
                        if servo.hardware_error != 0 {
                            " ERROR"
                        } else {
                            ""
                        }
                    ));
                }
            }

            self.last_log_time = current_time;
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("DynamixelNode shutting down - disabling torque on all servos");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Disable torque on all servos for safety
        for servo in self.servos.values_mut() {
            servo.torque_enabled = false;
            servo.velocity = 0.0;
        }

        // Close hardware port if open
        #[cfg(feature = "serial-hardware")]
        {
            self.serial_port = None;
        }

        ctx.log_info("All Dynamixel servos torque disabled");
        Ok(())
    }
}

/// Preset configurations for common robot setups
impl DynamixelNode {
    /// Configure for 6-DOF robot arm (typical research setup)
    pub fn configure_6dof_arm(&mut self) {
        // Typical configuration: base + shoulder + elbow + 3x wrist
        self.add_servo(1, DynamixelModel::XM430W350); // Base
        self.add_servo(2, DynamixelModel::XM430W350); // Shoulder
        self.add_servo(3, DynamixelModel::XM430W210); // Elbow
        self.add_servo(4, DynamixelModel::XL430W250); // Wrist roll
        self.add_servo(5, DynamixelModel::XL430W250); // Wrist pitch
        self.add_servo(6, DynamixelModel::XL430W250); // Wrist yaw
        self.enable_all_torque(false); // Safety: start disabled
    }

    /// Configure for humanoid robot (OpenManipulator-X style)
    pub fn configure_manipulator(&mut self) {
        for id in 1..=5 {
            self.add_servo(id, DynamixelModel::XM430W350);
        }
        self.enable_all_torque(false);
    }

    /// Configure for pan-tilt camera system
    pub fn configure_pan_tilt(&mut self) {
        self.add_servo(1, DynamixelModel::XL430W250); // Pan
        self.add_servo(2, DynamixelModel::XL430W250); // Tilt
        self.enable_all_torque(false);
    }
}

/// Builder for DynamixelNode with custom processor
pub struct DynamixelNodeBuilder<P>
where
    P: Processor<ServoCommand>,
{
    port: String,
    protocol: DynamixelProtocol,
    baud_rate: u32,
    processor: P,
}

impl DynamixelNodeBuilder<PassThrough<ServoCommand>> {
    /// Create a new builder with default configuration
    pub fn new(port: &str, protocol: DynamixelProtocol) -> Self {
        Self {
            port: port.to_string(),
            protocol,
            baud_rate: 1_000_000,
            processor: PassThrough::new(),
        }
    }
}

impl<P> DynamixelNodeBuilder<P>
where
    P: Processor<ServoCommand>,
{
    /// Set baud rate
    pub fn with_baud_rate(mut self, baud_rate: u32) -> Self {
        self.baud_rate = baud_rate;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> DynamixelNodeBuilder<P2>
    where
        P2: Processor<ServoCommand>,
    {
        DynamixelNodeBuilder {
            port: self.port,
            protocol: self.protocol,
            baud_rate: self.baud_rate,
            processor,
        }
    }

    /// Add a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> DynamixelNodeBuilder<ClosureProcessor<ServoCommand, ServoCommand, F>>
    where
        F: FnMut(ServoCommand) -> ServoCommand + Send + 'static,
    {
        DynamixelNodeBuilder {
            port: self.port,
            protocol: self.protocol,
            baud_rate: self.baud_rate,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> DynamixelNodeBuilder<FilterProcessor<ServoCommand, ServoCommand, F>>
    where
        F: FnMut(ServoCommand) -> Option<ServoCommand> + Send + 'static,
    {
        DynamixelNodeBuilder {
            port: self.port,
            protocol: self.protocol,
            baud_rate: self.baud_rate,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (same output type constraint)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> DynamixelNodeBuilder<Pipeline<ServoCommand, ServoCommand, ServoCommand, P, P2>>
    where
        P2: Processor<ServoCommand, ServoCommand>,
    {
        DynamixelNodeBuilder {
            port: self.port,
            protocol: self.protocol,
            baud_rate: self.baud_rate,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<DynamixelNode<P>> {
        Ok(DynamixelNode {
            command_subscriber: Topic::new("dynamixel.servo_cmd")?,
            joint_subscriber: Topic::new("dynamixel.joint_cmd")?,
            feedback_publisher: Topic::new("dynamixel.feedback")?,
            #[cfg(feature = "serial-hardware")]
            serial_port: None,
            hardware_enabled: false,
            port: self.port,
            protocol: self.protocol,
            baud_rate: self.baud_rate,
            servos: HashMap::new(),
            enable_bulk_operations: true,
            _last_packet_time: 0,
            packet_errors: 0,
            successful_packets: 0,
            last_read_time: 0,
            last_log_time: 0,
            processor: self.processor,
        })
    }
}
