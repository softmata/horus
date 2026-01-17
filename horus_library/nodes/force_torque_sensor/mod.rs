use crate::WrenchStamped;
use horus_core::core::LogSummary;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(feature = "netft")]
use std::net::UdpSocket;

#[cfg(feature = "robotiq-serial")]
use std::io::{Read, Write};

/// Force/Torque Sensor Node
///
/// Interface for 6-axis force/torque sensors commonly used in robotics for
/// manipulation, compliance control, and safety monitoring.
///
/// # Supported Manufacturers
/// - **ATI Industrial Automation** (Novanta): Nano/Mini/Gamma/Delta/Theta series
/// - **Robotiq FT-300**: Collaborative robot force-torque sensor
/// - **OnRobot HEX-E**: 6-axis force/torque sensor for UR and other cobots
/// - **Weiss Robotics**: KMS series sensors
/// - **Optoforce**: OMD/HEX series optical force sensors
/// - **JR3**: Various industrial F/T sensors
/// - **Generic**: Strain gauge based 6-axis sensors
///
/// # Communication Interfaces
/// - **Ethernet/EtherCAT**: High-speed real-time communication
/// - **CANopen**: Industrial fieldbus protocol
/// - **RS-232/RS-485**: Serial communication
/// - **Analog**: Voltage/current output (legacy)
/// - **USB**: Direct PC connection
///
/// # Measurement Ranges
/// Typical ranges vary by model:
/// - **Nano**: ±12 N force, ±120 Nmm torque (precision)
/// - **Mini**: ±240 N force, ±6 Nm torque (small robots)
/// - **Gamma**: ±660 N force, ±60 Nm torque (collaborative robots)
/// - **Delta**: ±3300 N force, ±330 Nm torque (industrial robots)
/// - **Theta**: ±6600 N force, ±660 Nm torque (heavy duty)
///
/// # Features
/// - 6-axis measurement (Fx, Fy, Fz, Tx, Ty, Tz)
/// - Automatic bias removal (taring/zeroing)
/// - Temperature compensation
/// - Overload protection monitoring
/// - Configurable sampling rates (up to 7kHz+)
/// - Low-pass filtering
/// - Tool transformation (TCP offset compensation)
/// - Calibration matrix application
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = ForceTorqueSensorNode::builder()
///     .with_filter(|wrench| {
///         // Only publish significant forces
///         let mag = (wrench.force.x.powi(2) + wrench.force.y.powi(2) + wrench.force.z.powi(2)).sqrt();
///         if mag > 1.0 { Some(wrench) } else { None }
///     })
///     .build()?;
/// ```
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::ForceTorqueSensorNode;
///
/// let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;
/// ft_sensor.set_sensor_model(SensorModel::ATI_Mini40);
/// ft_sensor.set_sample_rate(1000); // 1kHz
/// ft_sensor.zero_sensor(); // Tare to remove bias
/// ft_sensor.set_tool_mass(0.5, 0.0, 0.0, 0.05); // 0.5kg tool, 5cm offset
/// ```
pub struct ForceTorqueSensorNode<P = PassThrough<WrenchStamped>>
where
    P: Processor<WrenchStamped>,
{
    // Output publisher
    wrench_publisher: Topic<WrenchStamped>,
    calibration_status_pub: Topic<CalibrationStatus>,

    // Configuration
    sensor_model: SensorModel,
    connection_type: ConnectionType,
    device_address: String, // IP address or serial port
    sample_rate: u32,       // Hz
    frame_id: String,

    // Current readings (raw)
    raw_force: [f64; 3],  // Fx, Fy, Fz in sensor frame (Newtons)
    raw_torque: [f64; 3], // Tx, Ty, Tz in sensor frame (Nm)

    // Bias (for zeroing)
    bias_force: [f64; 3],
    bias_torque: [f64; 3],

    // Calibration matrix (6x6 for converting raw ADC to force/torque)
    calibration_matrix: [[f64; 6]; 6],

    // Tool compensation
    tool_mass: f64,           // kg
    tool_com: [f64; 3],       // Center of mass offset (x, y, z) in meters
    gravity_vector: [f64; 3], // Gravity direction in sensor frame

    // Measurement ranges
    force_range: [f64; 3],  // Max force in each axis (N)
    torque_range: [f64; 3], // Max torque in each axis (Nm)

    // Filtering
    use_lowpass_filter: bool,
    filter_cutoff_hz: f64,
    filtered_force: [f64; 3],
    filtered_torque: [f64; 3],
    filter_alpha: f64, // Exponential smoothing factor

    // Overload detection
    overload_threshold: f64, // Fraction of max range (0.0-1.0)
    overload_detected: bool,
    overload_count: u32,

    // Temperature
    temperature: f64, // Celsius
    temperature_compensation_enabled: bool,

    // Statistics
    measurement_count: u64,
    last_measurement_time: u64,
    max_recorded_force: f64,
    max_recorded_torque: f64,

    // Backend
    backend: FtBackend,

    // Hardware fields
    #[cfg(feature = "netft")]
    socket: Option<UdpSocket>,

    #[cfg(feature = "netft")]
    netft_calibration_matrix: [[f32; 6]; 6],

    // Robotiq serial hardware fields
    #[cfg(feature = "robotiq-serial")]
    serial_fd: Option<i32>,
    #[cfg(feature = "robotiq-serial")]
    robotiq_calibration: RobotiqCalibration,

    // Timing state (moved from static mut for thread safety)
    status_counter: u32,

    // Processor for hybrid pattern
    processor: P,
}

/// Robotiq FT-300 calibration data
#[cfg(feature = "robotiq-serial")]
#[derive(Debug, Clone, Copy)]
struct RobotiqCalibration {
    // Scaling factors from raw counts to SI units
    force_scale: [f64; 3],  // N per count
    torque_scale: [f64; 3], // Nm per count
}

/// Force/torque sensor models with predefined specifications
#[allow(non_camel_case_types)] // Product names like ATI_Nano17 are intentional
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SensorModel {
    ATI_Nano17,    // ±12 N, ±120 Nmm
    ATI_Mini40,    // ±240 N, ±6 Nm
    ATI_Mini45,    // ±290 N, ±10 Nm
    ATI_Gamma,     // ±660 N, ±60 Nm
    ATI_Delta,     // ±3300 N, ±330 Nm
    ATI_Theta,     // ±6600 N, ±660 Nm
    Robotiq_FT300, // ±300 N, ±30 Nm
    OnRobot_HexE,  // ±400 N, ±20 Nm
    Weiss_KMS40,   // ±400 N, ±15 Nm
    Optoforce_HEX, // ±200 N, ±4 Nm
    Generic,       // User-defined ranges
}

/// Force/torque backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FtBackend {
    Simulation,
    AtiNetFt,
    RobotiqSerial,
}

/// Connection type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConnectionType {
    Ethernet,
    CANopen,
    Serial,
    USB,
    EtherCAT,
}

/// Calibration status message
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct CalibrationStatus {
    pub is_calibrated: bool,
    pub is_zeroed: bool,
    pub temperature: f64,
    pub overload_detected: bool,
    pub overload_count: u32,
    pub measurement_count: u64,
    pub max_force: f64,
    pub max_torque: f64,
    pub timestamp: u64,
}

impl Default for CalibrationStatus {
    fn default() -> Self {
        Self {
            is_calibrated: true,
            is_zeroed: false,
            temperature: 25.0,
            overload_detected: false,
            overload_count: 0,
            measurement_count: 0,
            max_force: 0.0,
            max_torque: 0.0,
            timestamp: 0,
        }
    }
}

impl LogSummary for CalibrationStatus {
    fn log_summary(&self) -> String {
        format!(
            "Cal={} Zero={} T={:.1}°C Overload={} Fmax={:.1}N Tmax={:.1}Nm",
            self.is_calibrated,
            self.is_zeroed,
            self.temperature,
            self.overload_detected,
            self.max_force,
            self.max_torque
        )
    }
}

impl ForceTorqueSensorNode {
    /// Create a new force/torque sensor node in simulation mode
    pub fn new(device_address: &str) -> Result<Self> {
        Self::new_with_backend(device_address, FtBackend::Simulation)
    }

    /// Create a new force/torque sensor node with specific backend
    pub fn new_with_backend(device_address: &str, backend: FtBackend) -> Result<Self> {
        Ok(Self {
            wrench_publisher: Topic::new("force_torque.wrench")?,
            calibration_status_pub: Topic::new("force_torque.calibration")?,
            sensor_model: SensorModel::Generic,
            connection_type: ConnectionType::Ethernet,
            device_address: device_address.to_string(),
            sample_rate: 1000,
            frame_id: "ft_sensor".to_string(),
            raw_force: [0.0; 3],
            raw_torque: [0.0; 3],
            bias_force: [0.0; 3],
            bias_torque: [0.0; 3],
            calibration_matrix: Self::identity_matrix(),
            tool_mass: 0.0,
            tool_com: [0.0; 3],
            gravity_vector: [0.0, 0.0, -9.81], // Assuming Z-up frame
            force_range: [100.0, 100.0, 100.0], // Default 100N
            torque_range: [10.0, 10.0, 10.0],  // Default 10Nm
            use_lowpass_filter: true,
            filter_cutoff_hz: 100.0,
            filtered_force: [0.0; 3],
            filtered_torque: [0.0; 3],
            filter_alpha: 0.1,
            overload_threshold: 0.95,
            overload_detected: false,
            overload_count: 0,
            temperature: 25.0,
            temperature_compensation_enabled: true,
            measurement_count: 0,
            last_measurement_time: 0,
            max_recorded_force: 0.0,
            max_recorded_torque: 0.0,
            backend,
            #[cfg(feature = "netft")]
            socket: None,
            #[cfg(feature = "netft")]
            netft_calibration_matrix: Self::identity_matrix_f32(),
            #[cfg(feature = "robotiq-serial")]
            serial_fd: None,
            #[cfg(feature = "robotiq-serial")]
            robotiq_calibration: RobotiqCalibration {
                // FT-300 typical scaling (adjust based on calibration file)
                force_scale: [0.001, 0.001, 0.001],  // mN to N
                torque_scale: [0.001, 0.001, 0.001], // mNm to Nm
            },
            status_counter: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> ForceTorqueSensorNodeBuilder<PassThrough<WrenchStamped>> {
        ForceTorqueSensorNodeBuilder::new()
    }

    /// Set force/torque backend
    pub fn set_backend(&mut self, backend: FtBackend) {
        self.backend = backend;
    }

    /// Create 6x6 identity matrix (f64)
    fn identity_matrix() -> [[f64; 6]; 6] {
        let mut matrix = [[0.0; 6]; 6];
        for i in 0..6 {
            matrix[i][i] = 1.0;
        }
        matrix
    }

    /// Create 6x6 identity matrix (f32)
    #[cfg(feature = "netft")]
    fn identity_matrix_f32() -> [[f32; 6]; 6] {
        let mut matrix = [[0.0; 6]; 6];
        for i in 0..6 {
            matrix[i][i] = 1.0;
        }
        matrix
    }

    /// Set sensor model (applies predefined ranges)
    pub fn set_sensor_model(&mut self, model: SensorModel) {
        self.sensor_model = model;

        // Set appropriate ranges based on model
        match model {
            SensorModel::ATI_Nano17 => {
                self.force_range = [12.0, 12.0, 17.0];
                self.torque_range = [0.12, 0.12, 0.12];
            }
            SensorModel::ATI_Mini40 => {
                self.force_range = [240.0, 240.0, 240.0];
                self.torque_range = [6.0, 6.0, 6.0];
            }
            SensorModel::ATI_Mini45 => {
                self.force_range = [290.0, 290.0, 580.0];
                self.torque_range = [10.0, 10.0, 10.0];
            }
            SensorModel::ATI_Gamma => {
                self.force_range = [660.0, 660.0, 1980.0];
                self.torque_range = [60.0, 60.0, 60.0];
            }
            SensorModel::ATI_Delta => {
                self.force_range = [3300.0, 3300.0, 9900.0];
                self.torque_range = [330.0, 330.0, 330.0];
            }
            SensorModel::ATI_Theta => {
                self.force_range = [6600.0, 6600.0, 19800.0];
                self.torque_range = [660.0, 660.0, 660.0];
            }
            SensorModel::Robotiq_FT300 => {
                self.force_range = [300.0, 300.0, 500.0];
                self.torque_range = [30.0, 30.0, 30.0];
            }
            SensorModel::OnRobot_HexE => {
                self.force_range = [400.0, 400.0, 650.0];
                self.torque_range = [20.0, 20.0, 20.0];
            }
            SensorModel::Weiss_KMS40 => {
                self.force_range = [400.0, 400.0, 1000.0];
                self.torque_range = [15.0, 15.0, 15.0];
            }
            SensorModel::Optoforce_HEX => {
                self.force_range = [200.0, 200.0, 500.0];
                self.torque_range = [4.0, 4.0, 4.0];
            }
            SensorModel::Generic => {
                // Keep existing ranges
            }
        }
    }

    /// Set connection type
    pub fn set_connection_type(&mut self, conn_type: ConnectionType) {
        self.connection_type = conn_type;
    }

    /// Set sample rate (Hz)
    pub fn set_sample_rate(&mut self, rate: u32) {
        self.sample_rate = rate.clamp(1, 10000);
        // Update filter based on new sample rate
        self.update_filter_parameters();
    }

    /// Set frame ID
    pub fn set_frame_id(&mut self, frame_id: &str) {
        self.frame_id = frame_id.to_string();
    }

    /// Set custom force/torque ranges
    pub fn set_ranges(&mut self, force_range: [f64; 3], torque_range: [f64; 3]) {
        self.force_range = force_range;
        self.torque_range = torque_range;
    }

    /// Zero the sensor (tare) - removes current bias
    pub fn zero_sensor(&mut self) {
        self.bias_force = self.raw_force;
        self.bias_torque = self.raw_torque;
    }

    /// Reset bias to zero
    pub fn reset_bias(&mut self) {
        self.bias_force = [0.0; 3];
        self.bias_torque = [0.0; 3];
    }

    /// Set tool mass and center of mass for gravity compensation
    pub fn set_tool_mass(&mut self, mass: f64, com_x: f64, com_y: f64, com_z: f64) {
        self.tool_mass = mass;
        self.tool_com = [com_x, com_y, com_z];
    }

    /// Set gravity vector (usually [0, 0, -9.81] for Z-up frame)
    pub fn set_gravity_vector(&mut self, gx: f64, gy: f64, gz: f64) {
        self.gravity_vector = [gx, gy, gz];
    }

    /// Enable/disable low-pass filter
    pub fn set_lowpass_filter(&mut self, enable: bool, cutoff_hz: f64) {
        self.use_lowpass_filter = enable;
        self.filter_cutoff_hz = cutoff_hz;
        self.update_filter_parameters();
    }

    /// Update filter alpha based on sample rate and cutoff frequency
    fn update_filter_parameters(&mut self) {
        if self.use_lowpass_filter {
            let dt = 1.0 / self.sample_rate as f64;
            let rc = 1.0 / (2.0 * std::f64::consts::PI * self.filter_cutoff_hz);
            self.filter_alpha = dt / (rc + dt);
        }
    }

    /// Set overload threshold (0.0-1.0 fraction of max range)
    pub fn set_overload_threshold(&mut self, threshold: f64) {
        self.overload_threshold = threshold.clamp(0.5, 1.0);
    }

    /// Apply calibration matrix
    pub fn set_calibration_matrix(&mut self, matrix: [[f64; 6]; 6]) {
        self.calibration_matrix = matrix;
    }

    /// Get current force reading (compensated)
    pub fn get_force(&self) -> [f64; 3] {
        self.filtered_force
    }

    /// Get current torque reading (compensated)
    pub fn get_torque(&self) -> [f64; 3] {
        self.filtered_torque
    }

    /// Check if sensor is overloaded
    pub fn is_overloaded(&self) -> bool {
        self.overload_detected
    }

    /// Apply gravity compensation
    fn compensate_gravity(&self, force: [f64; 3], torque: [f64; 3]) -> ([f64; 3], [f64; 3]) {
        if self.tool_mass <= 0.0 {
            return (force, torque);
        }

        // Gravity force on tool
        let grav_force = [
            self.tool_mass * self.gravity_vector[0],
            self.tool_mass * self.gravity_vector[1],
            self.tool_mass * self.gravity_vector[2],
        ];

        // Torque due to tool weight (r × F)
        let grav_torque = [
            self.tool_com[1] * grav_force[2] - self.tool_com[2] * grav_force[1],
            self.tool_com[2] * grav_force[0] - self.tool_com[0] * grav_force[2],
            self.tool_com[0] * grav_force[1] - self.tool_com[1] * grav_force[0],
        ];

        // Subtract gravity effects
        let comp_force = [
            force[0] - grav_force[0],
            force[1] - grav_force[1],
            force[2] - grav_force[2],
        ];

        let comp_torque = [
            torque[0] - grav_torque[0],
            torque[1] - grav_torque[1],
            torque[2] - grav_torque[2],
        ];

        (comp_force, comp_torque)
    }

    /// Apply low-pass filter
    fn apply_filter(&mut self, force: [f64; 3], torque: [f64; 3]) -> ([f64; 3], [f64; 3]) {
        if !self.use_lowpass_filter {
            return (force, torque);
        }

        // Exponential moving average
        let alpha = self.filter_alpha;
        for i in 0..3 {
            self.filtered_force[i] = alpha * force[i] + (1.0 - alpha) * self.filtered_force[i];
            self.filtered_torque[i] = alpha * torque[i] + (1.0 - alpha) * self.filtered_torque[i];
        }

        (self.filtered_force, self.filtered_torque)
    }

    /// Check for overload condition
    fn check_overload(&mut self, force: [f64; 3], torque: [f64; 3]) -> bool {
        let mut overload = false;

        // Check force axes
        for i in 0..3 {
            if force[i].abs() > self.force_range[i] * self.overload_threshold {
                overload = true;
            }
        }

        // Check torque axes
        for i in 0..3 {
            if torque[i].abs() > self.torque_range[i] * self.overload_threshold {
                overload = true;
            }
        }

        if overload {
            self.overload_count += 1;
        }

        self.overload_detected = overload;
        overload
    }

    /// Simple pseudo-random number generator (avoiding rand dependency)
    fn pseudo_random(&self) -> f64 {
        // Use measurement count and time as seed
        let seed = self
            .measurement_count
            .wrapping_mul(1103515245)
            .wrapping_add(12345);
        let time_seed = self
            .last_measurement_time
            .wrapping_mul(214013)
            .wrapping_add(2531011);
        let combined = seed.wrapping_add(time_seed);
        ((combined % 1000) as f64 / 1000.0) - 0.5
    }

    /// Initialize hardware sensor
    fn initialize_sensor(&mut self, mut ctx: Option<&mut NodeInfo>) -> bool {
        match self.backend {
            FtBackend::Simulation => {
                // Simulation requires no hardware initialization
                true
            }
            #[cfg(feature = "netft")]
            FtBackend::AtiNetFt => {
                ctx.log_info(&format!(
                    "Initializing ATI NetFT sensor at {}",
                    self.device_address
                ));

                // ATI NetFT uses UDP on port 49152
                let bind_addr = "0.0.0.0:0"; // Bind to any available port

                match UdpSocket::bind(bind_addr) {
                    Ok(socket) => {
                        // Set socket to non-blocking mode
                        if let Err(e) = socket.set_nonblocking(true) {
                            ctx.log_error(&format!("Failed to set socket non-blocking: {:?}", e));
                            ctx.log_warning("Falling back to simulation mode");
                            self.backend = FtBackend::Simulation;
                            return true;
                        }

                        // Set read timeout
                        if let Err(e) =
                            socket.set_read_timeout(Some(std::time::Duration::from_millis(100)))
                        {
                            ctx.log_error(&format!("Failed to set socket timeout: {:?}", e));
                        }

                        // Connect to NetFT sensor
                        let target_addr = format!("{}:49152", self.device_address);
                        if let Err(e) = socket.connect(&target_addr) {
                            ctx.log_error(&format!("Failed to connect to NetFT sensor: {:?}", e));
                            ctx.log_warning("Falling back to simulation mode");
                            self.backend = FtBackend::Simulation;
                            return true;
                        }

                        self.socket = Some(socket);
                        ctx.log_info("ATI NetFT sensor initialized successfully");
                        true
                    }
                    Err(e) => {
                        ctx.log_error(&format!("Failed to create UDP socket: {:?}", e));
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = FtBackend::Simulation;
                        true
                    }
                }
            }
            #[cfg(not(feature = "netft"))]
            FtBackend::AtiNetFt => {
                ctx.log_warning("ATI NetFT backend requested but netft feature not enabled");
                ctx.log_warning("Falling back to simulation mode");
                self.backend = FtBackend::Simulation;
                true
            }
            #[cfg(feature = "robotiq-serial")]
            FtBackend::RobotiqSerial => {
                ctx.log_info(&format!(
                    "Initializing Robotiq FT-300 sensor on {}",
                    self.device_address
                ));

                // Robotiq FT-300 uses RS-485 serial at 19200 baud
                // Protocol: Modbus RTU over serial
                use std::ffi::CString;

                let device_path = if self.device_address.starts_with("/dev/") {
                    self.device_address.clone()
                } else {
                    format!("/dev/{}", self.device_address)
                };

                let c_path = match CString::new(device_path.as_str()) {
                    Ok(p) => p,
                    Err(_) => {
                        ctx.log_error("Invalid device path");
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = FtBackend::Simulation;
                        return true;
                    }
                };

                let fd = unsafe {
                    libc::open(
                        c_path.as_ptr(),
                        libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK,
                    )
                };

                if fd < 0 {
                    ctx.log_error(&format!(
                        "Failed to open serial port {}: {}",
                        device_path,
                        std::io::Error::last_os_error()
                    ));
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = FtBackend::Simulation;
                    return true;
                }

                // Configure serial port: 19200 baud, 8N1
                let mut termios: libc::termios = unsafe { std::mem::zeroed() };
                if unsafe { libc::tcgetattr(fd, &mut termios) } < 0 {
                    ctx.log_error("Failed to get serial port attributes");
                    unsafe { libc::close(fd) };
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = FtBackend::Simulation;
                    return true;
                }

                // Set baud rate to 19200
                unsafe {
                    libc::cfsetispeed(&mut termios, libc::B19200);
                    libc::cfsetospeed(&mut termios, libc::B19200);
                }

                // Configure 8N1: 8 data bits, no parity, 1 stop bit
                termios.c_cflag &= !(libc::PARENB | libc::CSTOPB | libc::CSIZE);
                termios.c_cflag |= libc::CS8 | libc::CLOCAL | libc::CREAD;

                // Raw input mode
                termios.c_lflag &= !(libc::ICANON | libc::ECHO | libc::ECHOE | libc::ISIG);
                termios.c_iflag &=
                    !(libc::IXON | libc::IXOFF | libc::IXANY | libc::INLCR | libc::ICRNL);
                termios.c_oflag &= !libc::OPOST;

                // Read timeout (0.1 second)
                termios.c_cc[libc::VMIN] = 0;
                termios.c_cc[libc::VTIME] = 1;

                if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &termios) } < 0 {
                    ctx.log_error("Failed to set serial port attributes");
                    unsafe { libc::close(fd) };
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = FtBackend::Simulation;
                    return true;
                }

                // Flush any pending data
                unsafe { libc::tcflush(fd, libc::TCIOFLUSH) };

                // Send stream start command (Robotiq protocol)
                // Command: 0x09 0x03 0x00 0x00 0x00 0x06 [CRC16]
                // This requests 6 registers starting at address 0 (force/torque data)
                let modbus_cmd: [u8; 8] = [
                    0x09, // Slave address (default for FT-300)
                    0x03, // Function code: Read holding registers
                    0x00, 0x00, // Starting register address (high, low)
                    0x00, 0x06, // Number of registers to read (6 = Fx,Fy,Fz,Tx,Ty,Tz)
                    0x00, 0x00, // CRC16 placeholder (will be calculated)
                ];

                // Calculate and append CRC16 (Modbus RTU)
                let crc = Self::modbus_crc16(&modbus_cmd[0..6]);
                let mut cmd_with_crc = modbus_cmd;
                cmd_with_crc[6] = (crc & 0xFF) as u8;
                cmd_with_crc[7] = ((crc >> 8) & 0xFF) as u8;

                let written = unsafe {
                    libc::write(
                        fd,
                        cmd_with_crc.as_ptr() as *const libc::c_void,
                        cmd_with_crc.len(),
                    )
                };

                if written < 0 {
                    ctx.log_warning("Failed to send initial command, but continuing...");
                }

                self.serial_fd = Some(fd);
                ctx.log_info("Robotiq FT-300 sensor initialized successfully");
                true
            }
            #[cfg(not(feature = "robotiq-serial"))]
            FtBackend::RobotiqSerial => {
                ctx.log_warning(
                    "Robotiq Serial backend requested but robotiq-serial feature not enabled",
                );
                ctx.log_warning("Falling back to simulation mode");
                self.backend = FtBackend::Simulation;
                true
            }
        }
    }

    /// Calculate Modbus CRC16
    #[cfg(feature = "robotiq-serial")]
    fn modbus_crc16(data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for byte in data {
            crc ^= *byte as u16;
            for _ in 0..8 {
                if (crc & 0x0001) != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc
    }

    /// Read sensor data from hardware
    #[cfg(feature = "netft")]
    fn read_netft_hardware(&mut self) -> bool {
        if let Some(ref socket) = self.socket {
            let mut buffer = [0u8; 36]; // NetFT packet is 36 bytes

            match socket.recv(&mut buffer) {
                Ok(bytes_read) if bytes_read >= 36 => {
                    // ATI NetFT packet format:
                    // Header: 4 bytes (0x1234)
                    // Force/Torque: 24 bytes (6 x i32, big-endian)
                    // Status: 4 bytes
                    // Counter: 4 bytes

                    // Verify header
                    let header = u32::from_be_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
                    if header != 0x1234 {
                        return false;
                    }

                    // Parse force/torque data (6 x i32, big-endian, in counts)
                    let mut ft_counts = [0i32; 6];
                    for i in 0..6 {
                        let offset = 4 + i * 4;
                        ft_counts[i] = i32::from_be_bytes([
                            buffer[offset],
                            buffer[offset + 1],
                            buffer[offset + 2],
                            buffer[offset + 3],
                        ]);
                    }

                    // Apply calibration matrix to convert counts to forces/torques
                    let mut ft_values = [0.0f32; 6];
                    for i in 0..6 {
                        for j in 0..6 {
                            ft_values[i] +=
                                self.netft_calibration_matrix[i][j] * ft_counts[j] as f32;
                        }
                    }

                    // Update raw readings (convert to f64)
                    self.raw_force[0] = ft_values[0] as f64;
                    self.raw_force[1] = ft_values[1] as f64;
                    self.raw_force[2] = ft_values[2] as f64;
                    self.raw_torque[0] = ft_values[3] as f64;
                    self.raw_torque[1] = ft_values[4] as f64;
                    self.raw_torque[2] = ft_values[5] as f64;

                    true
                }
                Ok(_) => {
                    // Incomplete packet
                    false
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No data available (non-blocking)
                    false
                }
                Err(_) => {
                    // Read error
                    false
                }
            }
        } else {
            false
        }
    }

    /// Read sensor data from Robotiq FT-300 via Modbus RTU
    #[cfg(feature = "robotiq-serial")]
    fn read_robotiq_hardware(&mut self) -> bool {
        let fd = match self.serial_fd {
            Some(fd) => fd,
            None => return false,
        };

        // Send read command: read 6 registers starting at 0
        let modbus_cmd: [u8; 6] = [
            0x09, // Slave address
            0x03, // Function code: Read holding registers
            0x00, 0x00, // Starting register address
            0x00, 0x06, // Number of registers (6 = Fx,Fy,Fz,Tx,Ty,Tz)
        ];

        let crc = Self::modbus_crc16(&modbus_cmd);
        let mut cmd_with_crc = [0u8; 8];
        cmd_with_crc[..6].copy_from_slice(&modbus_cmd);
        cmd_with_crc[6] = (crc & 0xFF) as u8;
        cmd_with_crc[7] = ((crc >> 8) & 0xFF) as u8;

        // Send command
        let written = unsafe { libc::write(fd, cmd_with_crc.as_ptr() as *const libc::c_void, 8) };

        if written != 8 {
            return false;
        }

        // Read response: 17 bytes for 6 registers
        // Format: [addr][func][byte_count][data...][crc_lo][crc_hi]
        // = 1 + 1 + 1 + 12 + 2 = 17 bytes
        let mut response = [0u8; 17];
        let mut bytes_read = 0;

        // Read with timeout handling
        for _ in 0..10 {
            let n = unsafe {
                libc::read(
                    fd,
                    response[bytes_read..].as_mut_ptr() as *mut libc::c_void,
                    (17 - bytes_read) as libc::size_t,
                )
            };

            if n > 0 {
                bytes_read += n as usize;
                if bytes_read >= 17 {
                    break;
                }
            } else if n == 0 || (n < 0 && unsafe { *libc::__errno_location() } == libc::EAGAIN) {
                // No data yet, small delay
                std::thread::sleep(std::time::Duration::from_micros(500));
            } else {
                return false;
            }
        }

        if bytes_read < 17 {
            return false;
        }

        // Verify response
        if response[0] != 0x09 || response[1] != 0x03 || response[2] != 12 {
            return false;
        }

        // Verify CRC
        let recv_crc = (response[16] as u16) | ((response[15] as u16) << 8);
        let calc_crc = Self::modbus_crc16(&response[..15]);
        if recv_crc != calc_crc {
            return false;
        }

        // Parse force/torque data (6 x i16, big-endian, in raw counts)
        let mut raw_counts = [0i16; 6];
        for i in 0..6 {
            let offset = 3 + i * 2;
            raw_counts[i] = i16::from_be_bytes([response[offset], response[offset + 1]]);
        }

        // Apply calibration to convert to SI units
        self.raw_force[0] = raw_counts[0] as f64 * self.robotiq_calibration.force_scale[0];
        self.raw_force[1] = raw_counts[1] as f64 * self.robotiq_calibration.force_scale[1];
        self.raw_force[2] = raw_counts[2] as f64 * self.robotiq_calibration.force_scale[2];
        self.raw_torque[0] = raw_counts[3] as f64 * self.robotiq_calibration.torque_scale[0];
        self.raw_torque[1] = raw_counts[4] as f64 * self.robotiq_calibration.torque_scale[1];
        self.raw_torque[2] = raw_counts[5] as f64 * self.robotiq_calibration.torque_scale[2];

        true
    }

    /// Simulate sensor reading (for testing without hardware)
    fn simulate_reading(&mut self) {
        // Simulate small noise
        for i in 0..3 {
            self.raw_force[i] = self.pseudo_random() * 0.1;
            self.raw_torque[i] = self.pseudo_random() * 0.01;
        }

        // Add gravity effects if tool is configured
        if self.tool_mass > 0.0 {
            self.raw_force[0] += self.tool_mass * self.gravity_vector[0];
            self.raw_force[1] += self.tool_mass * self.gravity_vector[1];
            self.raw_force[2] += self.tool_mass * self.gravity_vector[2];
        }
    }

    /// Read sensor data (from hardware or simulation)
    fn read_sensor(&mut self, mut _ctx: Option<&mut NodeInfo>) {
        match self.backend {
            FtBackend::Simulation => {
                self.simulate_reading();
            }
            #[cfg(feature = "netft")]
            FtBackend::AtiNetFt => {
                if !self.read_netft_hardware() {
                    // If hardware read fails, use simulation as fallback
                    self.simulate_reading();
                }
            }
            #[cfg(not(feature = "netft"))]
            FtBackend::AtiNetFt => {
                self.simulate_reading();
            }
            #[cfg(feature = "robotiq-serial")]
            FtBackend::RobotiqSerial => {
                if !self.read_robotiq_hardware() {
                    // If hardware read fails, use simulation as fallback
                    self.simulate_reading();
                }
            }
            #[cfg(not(feature = "robotiq-serial"))]
            FtBackend::RobotiqSerial => {
                self.simulate_reading();
            }
        }
    }

    /// Process and publish measurement
    fn process_measurement(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Remove bias
        let biased_force = [
            self.raw_force[0] - self.bias_force[0],
            self.raw_force[1] - self.bias_force[1],
            self.raw_force[2] - self.bias_force[2],
        ];

        let biased_torque = [
            self.raw_torque[0] - self.bias_torque[0],
            self.raw_torque[1] - self.bias_torque[1],
            self.raw_torque[2] - self.bias_torque[2],
        ];

        // Apply gravity compensation
        let (comp_force, comp_torque) = self.compensate_gravity(biased_force, biased_torque);

        // Apply filtering
        let (final_force, final_torque) = self.apply_filter(comp_force, comp_torque);

        // Check for overload
        let overload = self.check_overload(final_force, final_torque);
        if overload {
            ctx.log_warning(&format!(
                "F/T Sensor OVERLOAD: F=[{:.1}, {:.1}, {:.1}]N T=[{:.1}, {:.1}, {:.1}]Nm",
                final_force[0],
                final_force[1],
                final_force[2],
                final_torque[0],
                final_torque[1],
                final_torque[2]
            ));
        }

        // Update statistics
        let force_mag =
            (final_force[0].powi(2) + final_force[1].powi(2) + final_force[2].powi(2)).sqrt();
        let torque_mag =
            (final_torque[0].powi(2) + final_torque[1].powi(2) + final_torque[2].powi(2)).sqrt();
        self.max_recorded_force = self.max_recorded_force.max(force_mag);
        self.max_recorded_torque = self.max_recorded_torque.max(torque_mag);

        // Create and publish wrench message
        let mut wrench = WrenchStamped::default();
        wrench.force.x = final_force[0];
        wrench.force.y = final_force[1];
        wrench.force.z = final_force[2];
        wrench.torque.x = final_torque[0];
        wrench.torque.y = final_torque[1];
        wrench.torque.z = final_torque[2];

        // Set frame_id
        let frame_bytes = self.frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        wrench.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        wrench.frame_id[len] = 0;

        wrench.timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        let timestamp = wrench.timestamp;

        // Process through pipeline
        if let Some(processed) = self.processor.process(wrench) {
            if let Err(e) = self.wrench_publisher.send(processed, &mut None) {
                ctx.log_error(&format!("Failed to publish wrench: {:?}", e));
            }
        }

        self.measurement_count += 1;
        self.last_measurement_time = timestamp;
    }

    /// Publish calibration status
    fn publish_status(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let status = CalibrationStatus {
            is_calibrated: true,
            is_zeroed: self.bias_force != [0.0; 3] || self.bias_torque != [0.0; 3],
            temperature: self.temperature,
            overload_detected: self.overload_detected,
            overload_count: self.overload_count,
            measurement_count: self.measurement_count,
            max_force: self.max_recorded_force,
            max_torque: self.max_recorded_torque,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };

        if let Err(e) = self.calibration_status_pub.send(status, &mut None) {
            ctx.log_error(&format!("Failed to publish calibration status: {:?}", e));
        }
    }
}

impl<P> Node for ForceTorqueSensorNode<P>
where
    P: Processor<WrenchStamped>,
{
    fn name(&self) -> &'static str {
        "ForceTorqueSensorNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("Force/torque sensor node initialized");

        match self.backend {
            FtBackend::Simulation => {
                ctx.log_info("Force/torque sensor simulation mode enabled");
            }
            FtBackend::AtiNetFt => {
                ctx.log_info(&format!("ATI NetFT sensor: {}", self.device_address));
                if !self.initialize_sensor(Some(ctx)) {
                    ctx.log_error("Failed to initialize ATI NetFT sensor");
                }
            }
            FtBackend::RobotiqSerial => {
                ctx.log_info("Robotiq Serial sensor (not yet implemented)");
            }
        }

        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        // Read from hardware or simulate
        self.read_sensor(ctx.as_deref_mut());

        // Process and publish
        self.process_measurement(ctx.as_deref_mut());

        // Publish status at lower rate
        self.status_counter += 1;
        if self.status_counter % 100 == 0 {
            self.publish_status(ctx.as_deref_mut());
        }

        // Periodic logging at 1Hz
        if self.status_counter % 1000 == 0 {
            ctx.log_info(&format!(
                "F/T Sensor: F=[{:.2}, {:.2}, {:.2}]N T=[{:.2}, {:.2}, {:.2}]Nm (Fmax={:.1}N Tmax={:.1}Nm)",
                self.filtered_force[0],
                self.filtered_force[1],
                self.filtered_force[2],
                self.filtered_torque[0],
                self.filtered_torque[1],
                self.filtered_torque[2],
                self.max_recorded_force,
                self.max_recorded_torque
            ));
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();

        #[cfg(feature = "netft")]
        {
            if self.socket.is_some() {
                ctx.log_info("Closing NetFT sensor connection");
                self.socket = None;
            }
        }

        #[cfg(feature = "robotiq-serial")]
        {
            if let Some(fd) = self.serial_fd.take() {
                ctx.log_info("Closing Robotiq FT-300 serial connection");
                unsafe { libc::close(fd) };
            }
        }

        Ok(())
    }
}

/// Preset configurations
impl ForceTorqueSensorNode {
    /// Configure for ATI Mini40 (common on UR robots)
    pub fn configure_ati_mini40(&mut self) {
        self.set_sensor_model(SensorModel::ATI_Mini40);
        self.set_sample_rate(1000);
        self.set_lowpass_filter(true, 100.0);
    }

    /// Configure for collaborative robot applications
    pub fn configure_cobot(&mut self) {
        self.set_sensor_model(SensorModel::Robotiq_FT300);
        self.set_sample_rate(500);
        self.set_lowpass_filter(true, 50.0);
        self.set_overload_threshold(0.80); // Lower threshold for safety
    }

    /// Configure for precision assembly
    pub fn configure_precision(&mut self) {
        self.set_sensor_model(SensorModel::ATI_Nano17);
        self.set_sample_rate(2000);
        self.set_lowpass_filter(true, 200.0);
    }

    /// Configure for heavy-duty industrial use
    pub fn configure_heavy_duty(&mut self) {
        self.set_sensor_model(SensorModel::ATI_Theta);
        self.set_sample_rate(1000);
        self.set_lowpass_filter(true, 100.0);
        self.set_overload_threshold(0.95);
    }
}

/// Builder for ForceTorqueSensorNode with processor configuration
pub struct ForceTorqueSensorNodeBuilder<P>
where
    P: Processor<WrenchStamped>,
{
    device_address: String,
    backend: FtBackend,
    processor: P,
}

impl ForceTorqueSensorNodeBuilder<PassThrough<WrenchStamped>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            device_address: "127.0.0.1".to_string(),
            backend: FtBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for ForceTorqueSensorNodeBuilder<PassThrough<WrenchStamped>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> ForceTorqueSensorNodeBuilder<P>
where
    P: Processor<WrenchStamped>,
{
    /// Set the device address
    pub fn device_address(mut self, address: &str) -> Self {
        self.device_address = address.to_string();
        self
    }

    /// Set the backend
    pub fn backend(mut self, backend: FtBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> ForceTorqueSensorNodeBuilder<P2>
    where
        P2: Processor<WrenchStamped>,
    {
        ForceTorqueSensorNodeBuilder {
            device_address: self.device_address,
            backend: self.backend,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> ForceTorqueSensorNodeBuilder<ClosureProcessor<WrenchStamped, WrenchStamped, F>>
    where
        F: FnMut(WrenchStamped) -> WrenchStamped + Send + 'static,
    {
        ForceTorqueSensorNodeBuilder {
            device_address: self.device_address,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> ForceTorqueSensorNodeBuilder<FilterProcessor<WrenchStamped, WrenchStamped, F>>
    where
        F: FnMut(WrenchStamped) -> Option<WrenchStamped> + Send + 'static,
    {
        ForceTorqueSensorNodeBuilder {
            device_address: self.device_address,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> ForceTorqueSensorNodeBuilder<Pipeline<WrenchStamped, WrenchStamped, WrenchStamped, P, P2>>
    where
        P2: Processor<WrenchStamped, Output = WrenchStamped>,
    {
        ForceTorqueSensorNodeBuilder {
            device_address: self.device_address,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<ForceTorqueSensorNode<P>> {
        let node = ForceTorqueSensorNode::new_with_backend(&self.device_address, self.backend)?;
        Ok(ForceTorqueSensorNode {
            wrench_publisher: node.wrench_publisher,
            calibration_status_pub: node.calibration_status_pub,
            sensor_model: node.sensor_model,
            connection_type: node.connection_type,
            device_address: node.device_address,
            sample_rate: node.sample_rate,
            frame_id: node.frame_id,
            raw_force: node.raw_force,
            raw_torque: node.raw_torque,
            bias_force: node.bias_force,
            bias_torque: node.bias_torque,
            calibration_matrix: node.calibration_matrix,
            tool_mass: node.tool_mass,
            tool_com: node.tool_com,
            gravity_vector: node.gravity_vector,
            force_range: node.force_range,
            torque_range: node.torque_range,
            use_lowpass_filter: node.use_lowpass_filter,
            filter_cutoff_hz: node.filter_cutoff_hz,
            filtered_force: node.filtered_force,
            filtered_torque: node.filtered_torque,
            filter_alpha: node.filter_alpha,
            overload_threshold: node.overload_threshold,
            overload_detected: node.overload_detected,
            overload_count: node.overload_count,
            temperature: node.temperature,
            temperature_compensation_enabled: node.temperature_compensation_enabled,
            measurement_count: node.measurement_count,
            last_measurement_time: node.last_measurement_time,
            max_recorded_force: node.max_recorded_force,
            max_recorded_torque: node.max_recorded_torque,
            backend: node.backend,
            #[cfg(feature = "netft")]
            socket: node.socket,
            #[cfg(feature = "netft")]
            netft_calibration_matrix: node.netft_calibration_matrix,
            #[cfg(feature = "robotiq-serial")]
            serial_fd: node.serial_fd,
            #[cfg(feature = "robotiq-serial")]
            robotiq_calibration: node.robotiq_calibration,
            status_counter: node.status_counter,
            processor: self.processor,
        })
    }
}
