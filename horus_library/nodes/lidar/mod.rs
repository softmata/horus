use crate::LaserScan;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

// Serial port for hardware LiDAR
#[cfg(feature = "serial-hardware")]
use serialport::SerialPort;

// Hardware LiDAR support status:
// - RPLidar: Disabled due to rplidar_drv crate upstream compilation bug
//   (unaligned packed struct reference in ultra_capsuled_parser.rs)
// - YDLIDAR: Supported via serial protocol (X2, X4, T-mini Pro)
//   Requires 'serial-hardware' feature flag

/// LiDAR backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LidarBackend {
    Simulation,
    RplidarA1,
    RplidarA2,
    RplidarA3,
    YdlidarX2,
    YdlidarX4,
    YdlidarTMiniPro,
}

/// LiDAR Node - Generic LiDAR interface for obstacle detection and mapping
///
/// Captures laser scan data from various LiDAR sensors and publishes LaserScan messages.
/// Supports multiple hardware backends:
/// - RPLidar A1/A2/A3 series
/// - YDLIDAR X2/X4/T-mini Pro series
/// - Simulation mode for testing
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = LidarNode::builder()
///     .with_closure(|mut scan| {
///         // Apply range filtering
///         for r in scan.ranges.iter_mut() {
///             if *r < 0.1 { *r = f32::INFINITY; }
///         }
///         scan
///     })
///     .build()?;
/// ```
pub struct LidarNode<P = PassThrough<LaserScan>>
where
    P: Processor<LaserScan>,
{
    publisher: Hub<LaserScan>,

    // Configuration
    frame_id: String,
    scan_frequency: f32,
    min_range: f32,
    max_range: f32,
    angle_increment: f32,
    backend: LidarBackend,
    serial_port_path: String,
    baud_rate: u32,

    // Hardware serial port (when using YDLIDAR)
    #[cfg(feature = "serial-hardware")]
    serial_port: Option<Box<dyn SerialPort>>,

    // YDLIDAR protocol state
    #[cfg(feature = "serial-hardware")]
    read_buffer: Vec<u8>,

    // State
    is_initialized: bool,
    scan_count: u64,
    last_scan_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl LidarNode {
    /// Create a new LiDAR node with default topic "scan" in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("scan", LidarBackend::Simulation)
    }

    /// Create a new LiDAR node with custom topic in simulation mode
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        Self::new_with_backend(topic, LidarBackend::Simulation)
    }

    /// Create a new LiDAR node with specific backend
    pub fn new_with_backend(topic: &str, backend: LidarBackend) -> Result<Self> {
        // Determine baud rate based on backend
        let baud_rate = match backend {
            LidarBackend::YdlidarX2 => 115200,
            LidarBackend::YdlidarX4 => 128000,
            LidarBackend::YdlidarTMiniPro => 230400,
            _ => 115200,
        };

        Ok(Self {
            publisher: Hub::new(topic)?,
            frame_id: "laser_frame".to_string(),
            scan_frequency: 10.0,
            min_range: 0.1,
            max_range: match backend {
                LidarBackend::RplidarA1 => 12.0,       // A1: 12m max range
                LidarBackend::RplidarA2 => 16.0,       // A2: 16m max range
                LidarBackend::RplidarA3 => 25.0,       // A3: 25m max range
                LidarBackend::YdlidarX2 => 8.0,        // X2: 8m max range
                LidarBackend::YdlidarX4 => 10.0,       // X4: 10m max range
                LidarBackend::YdlidarTMiniPro => 12.0, // T-mini Pro: 12m max range
                _ => 30.0,
            },
            angle_increment: std::f32::consts::PI / 180.0, // 1 degree
            backend,
            serial_port_path: "/dev/ttyUSB0".to_string(),
            baud_rate,
            #[cfg(feature = "serial-hardware")]
            serial_port: None,
            #[cfg(feature = "serial-hardware")]
            read_buffer: Vec::with_capacity(4096),
            is_initialized: false,
            scan_count: 0,
            last_scan_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> LidarNodeBuilder<PassThrough<LaserScan>> {
        LidarNodeBuilder::new()
    }

    /// Set LiDAR backend
    pub fn set_backend(&mut self, backend: LidarBackend) {
        self.backend = backend;
        self.is_initialized = false;
    }

    /// Set serial port for LiDAR
    pub fn set_serial_port(&mut self, port: &str) {
        self.serial_port_path = port.to_string();
        self.is_initialized = false;
    }

    /// Set baud rate for serial communication
    pub fn set_baud_rate(&mut self, baud_rate: u32) {
        self.baud_rate = baud_rate;
        self.is_initialized = false;
    }

    /// Set frame ID for coordinate system
    pub fn set_frame_id(&mut self, frame_id: &str) {
        self.frame_id = frame_id.to_string();
    }

    /// Set scan frequency (Hz)
    pub fn set_scan_frequency(&mut self, frequency: f32) {
        self.scan_frequency = frequency.clamp(0.1, 100.0);
    }

    /// Set range limits (meters)
    pub fn set_range_limits(&mut self, min_range: f32, max_range: f32) {
        self.min_range = min_range.max(0.0);
        self.max_range = max_range.max(self.min_range + 0.1);
    }

    /// Set angular resolution (radians)
    pub fn set_angle_increment(&mut self, increment: f32) {
        self.angle_increment = increment.clamp(0.001, 0.1);
    }

    /// Get actual scan rate (scans per second)
    pub fn get_actual_scan_rate(&self) -> f32 {
        if self.scan_count < 2 {
            return 0.0;
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        let time_diff = current_time - self.last_scan_time;
        if time_diff > 0 {
            1000.0 / time_diff as f32
        } else {
            0.0
        }
    }

    fn initialize_lidar(&mut self) -> bool {
        if self.is_initialized {
            return true;
        }

        match self.backend {
            LidarBackend::Simulation => {
                self.is_initialized = true;
                true
            }
            LidarBackend::RplidarA1 | LidarBackend::RplidarA2 | LidarBackend::RplidarA3 => {
                eprintln!("RPLidar hardware support is temporarily disabled (upstream crate bug)");
                eprintln!("Falling back to simulation mode");
                self.backend = LidarBackend::Simulation;
                self.is_initialized = true;
                true
            }
            LidarBackend::YdlidarX2 | LidarBackend::YdlidarX4 | LidarBackend::YdlidarTMiniPro => {
                #[cfg(feature = "serial-hardware")]
                {
                    match self.initialize_ydlidar() {
                        Ok(()) => {
                            eprintln!("YDLIDAR {:?} initialized on {}", self.backend, self.serial_port_path);
                            self.is_initialized = true;
                            return true;
                        }
                        Err(e) => {
                            eprintln!("Failed to initialize YDLIDAR: {}", e);
                            eprintln!("Falling back to simulation mode");
                        }
                    }
                }

                #[cfg(not(feature = "serial-hardware"))]
                {
                    eprintln!("YDLIDAR support requires 'serial-hardware' feature");
                    eprintln!("Build with: cargo build --features serial-hardware");
                }

                self.backend = LidarBackend::Simulation;
                self.is_initialized = true;
                true
            }
        }
    }

    #[cfg(feature = "serial-hardware")]
    fn initialize_ydlidar(&mut self) -> std::result::Result<(), String> {
        use serialport::SerialPortType;

        // Open serial port
        let port = serialport::new(&self.serial_port_path, self.baud_rate)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| format!("Failed to open serial port {}: {}", self.serial_port_path, e))?;

        self.serial_port = Some(port);
        self.read_buffer.clear();

        // Send start scan command for YDLIDAR
        // YDLIDAR protocol: 0xA5 0x60 (start scan)
        if let Some(ref mut port) = self.serial_port {
            let start_cmd = [0xA5, 0x60];
            port.write_all(&start_cmd)
                .map_err(|e| format!("Failed to send start command: {}", e))?;
        }

        Ok(())
    }

    #[cfg(feature = "serial-hardware")]
    fn stop_ydlidar(&mut self) {
        if let Some(ref mut port) = self.serial_port {
            // Send stop command: 0xA5 0x65
            let stop_cmd = [0xA5, 0x65];
            let _ = port.write_all(&stop_cmd);
        }
        self.serial_port = None;
    }

    fn generate_scan_data(&mut self) -> Option<Vec<f32>> {
        match self.backend {
            LidarBackend::Simulation => {
                // Generate synthetic scan data for testing
                let num_points = (2.0 * std::f32::consts::PI / self.angle_increment) as usize;
                let mut ranges = Vec::with_capacity(num_points);

                for i in 0..num_points {
                    let angle = i as f32 * self.angle_increment;

                    // Create some obstacles at different distances
                    let range = if angle.cos() > 0.8 {
                        2.0 + 0.5 * angle.sin() // Wall-like obstacle
                    } else if (angle - std::f32::consts::PI / 2.0).abs() < 0.5 {
                        1.0 // Closer obstacle
                    } else {
                        self.max_range // No obstacle detected
                    };

                    ranges.push(range.min(self.max_range).max(self.min_range));
                }

                Some(ranges)
            }
            // Note: RPLidar hardware support temporarily disabled (rplidar_drv crate has upstream bug)
            // The backend will have been switched to Simulation in initialize_lidar()
            LidarBackend::RplidarA1 | LidarBackend::RplidarA2 | LidarBackend::RplidarA3 => {
                // Fallback to simulation - this branch shouldn't be reached if initialize_lidar ran
                None
            }
            LidarBackend::YdlidarX2 | LidarBackend::YdlidarX4 | LidarBackend::YdlidarTMiniPro => {
                #[cfg(feature = "serial-hardware")]
                {
                    return self.read_ydlidar_scan();
                }

                #[cfg(not(feature = "serial-hardware"))]
                {
                    // This shouldn't be reached - initialize_lidar should have fallen back to simulation
                    None
                }
            }
        }
    }

    #[cfg(feature = "serial-hardware")]
    fn read_ydlidar_scan(&mut self) -> Option<Vec<f32>> {
        use std::io::Read;

        let port = self.serial_port.as_mut()?;

        // Read available data into buffer
        let mut temp_buf = [0u8; 512];
        match port.read(&mut temp_buf) {
            Ok(n) if n > 0 => {
                self.read_buffer.extend_from_slice(&temp_buf[..n]);
            }
            _ => {}
        }

        // YDLIDAR packet format:
        // Header: 0xAA 0x55
        // CT: 1 byte (scan type)
        // LSN: 1 byte (sample quantity)
        // FSA: 2 bytes (start angle)
        // LSA: 2 bytes (end angle)
        // CS: 2 bytes (checksum)
        // Data: LSN * 2 bytes (distance samples)

        // Find packet header
        let header_pos = self.read_buffer.windows(2).position(|w| w == [0xAA, 0x55])?;

        // Need at least 10 bytes for header + minimal data
        if self.read_buffer.len() < header_pos + 10 {
            return None;
        }

        // Parse packet
        let _ct = self.read_buffer[header_pos + 2];
        let lsn = self.read_buffer[header_pos + 3] as usize;
        let fsa = u16::from_le_bytes([
            self.read_buffer[header_pos + 4],
            self.read_buffer[header_pos + 5],
        ]);
        let lsa = u16::from_le_bytes([
            self.read_buffer[header_pos + 6],
            self.read_buffer[header_pos + 7],
        ]);

        // Check if we have the full packet
        let packet_len = 10 + lsn * 2;
        if self.read_buffer.len() < header_pos + packet_len {
            return None;
        }

        // Parse distance samples
        let mut ranges = vec![self.max_range; 360];
        let start_angle = (fsa >> 1) as f32 / 64.0;
        let end_angle = (lsa >> 1) as f32 / 64.0;

        let angle_step = if lsn > 1 {
            let diff = if end_angle > start_angle {
                end_angle - start_angle
            } else {
                360.0 - start_angle + end_angle
            };
            diff / (lsn - 1) as f32
        } else {
            0.0
        };

        for i in 0..lsn {
            let data_offset = header_pos + 10 + i * 2;
            if data_offset + 1 < self.read_buffer.len() {
                let distance_raw = u16::from_le_bytes([
                    self.read_buffer[data_offset],
                    self.read_buffer[data_offset + 1],
                ]);

                // Convert to meters (YDLIDAR uses mm)
                let distance = distance_raw as f32 / 1000.0;

                // Calculate angle index
                let angle = start_angle + i as f32 * angle_step;
                let angle_idx = (angle % 360.0) as usize;

                if angle_idx < 360 && distance > self.min_range && distance < self.max_range {
                    ranges[angle_idx] = distance;
                }
            }
        }

        // Remove processed data from buffer
        self.read_buffer.drain(..header_pos + packet_len);

        Some(ranges)
    }

    fn publish_scan(&self, ranges: Vec<f32>) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let mut scan = LaserScan::new();
        scan.angle_min = 0.0;
        scan.angle_max = 2.0 * std::f32::consts::PI;
        scan.angle_increment = self.angle_increment;
        scan.time_increment = 1.0 / self.scan_frequency;
        scan.scan_time = 0.1;
        scan.range_min = self.min_range;
        scan.range_max = self.max_range;

        // Copy ranges to fixed array
        for (i, &range) in ranges.iter().take(360).enumerate() {
            scan.ranges[i] = range;
        }

        scan.timestamp = current_time;

        let _ = self.publisher.send(scan, &mut None);
    }
}

impl<P> Node for LidarNode<P>
where
    P: Processor<LaserScan>,
{
    fn name(&self) -> &'static str {
        "LidarNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        ctx.log_info("LidarNode shutting down - stopping LiDAR sensor");

        // Stop LiDAR scanning
        self.is_initialized = false;

        // Stop hardware LiDAR if running
        #[cfg(feature = "serial-hardware")]
        {
            match self.backend {
                LidarBackend::YdlidarX2 | LidarBackend::YdlidarX4 | LidarBackend::YdlidarTMiniPro => {
                    self.stop_ydlidar();
                }
                _ => {}
            }
        }

        ctx.log_info("LiDAR sensor stopped safely");
        Ok(())
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("LidarNode initialized");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        // Initialize LiDAR on first tick
        if !self.is_initialized && !self.initialize_lidar() {
            return; // Skip if initialization failed
        }

        // Generate and publish scan data
        if let Some(ranges) = self.generate_scan_data() {
            self.scan_count += 1;
            self.last_scan_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;

            // Build scan message
            let current_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            let mut scan = LaserScan::new();
            scan.angle_min = 0.0;
            scan.angle_max = 2.0 * std::f32::consts::PI;
            scan.angle_increment = self.angle_increment;
            scan.time_increment = 1.0 / self.scan_frequency;
            scan.scan_time = 0.1;
            scan.range_min = self.min_range;
            scan.range_max = self.max_range;

            for (i, &range) in ranges.iter().take(360).enumerate() {
                scan.ranges[i] = range;
            }

            scan.timestamp = current_time;

            // Process through pipeline and publish
            if let Some(processed) = self.processor.process(scan) {
                let _ = self.publisher.send(processed, &mut None);
            }
        }
    }
}

impl<P> Drop for LidarNode<P>
where
    P: Processor<LaserScan>,
{
    fn drop(&mut self) {
        // Hardware cleanup would go here when LiDAR drivers are enabled
    }
}

/// Builder for LidarNode with fluent API for processor configuration
pub struct LidarNodeBuilder<P>
where
    P: Processor<LaserScan>,
{
    topic: String,
    backend: LidarBackend,
    processor: P,
}

impl LidarNodeBuilder<PassThrough<LaserScan>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "scan".to_string(),
            backend: LidarBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl<P> LidarNodeBuilder<P>
where
    P: Processor<LaserScan>,
{
    /// Set the topic for publishing laser scans
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set the LiDAR backend
    pub fn backend(mut self, backend: LidarBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> LidarNodeBuilder<P2>
    where
        P2: Processor<LaserScan>,
    {
        LidarNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor,
        }
    }

    /// Add a closure processor for transformations
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> LidarNodeBuilder<ClosureProcessor<LaserScan, LaserScan, F>>
    where
        F: FnMut(LaserScan) -> LaserScan + Send + 'static,
    {
        LidarNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> LidarNodeBuilder<FilterProcessor<LaserScan, LaserScan, F>>
    where
        F: FnMut(LaserScan) -> Option<LaserScan> + Send + 'static,
    {
        LidarNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> LidarNodeBuilder<Pipeline<LaserScan, LaserScan, LaserScan, P, P2>>
    where
        P2: Processor<LaserScan, Output = LaserScan>,
        P: Processor<LaserScan, Output = LaserScan>,
    {
        LidarNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the LidarNode
    pub fn build(self) -> Result<LidarNode<P>> {
        let max_range = match self.backend {
            LidarBackend::RplidarA1 => 12.0,
            LidarBackend::RplidarA2 => 16.0,
            LidarBackend::RplidarA3 => 25.0,
            LidarBackend::YdlidarX2 => 8.0,
            LidarBackend::YdlidarX4 => 10.0,
            LidarBackend::YdlidarTMiniPro => 12.0,
            _ => 30.0,
        };

        Ok(LidarNode {
            publisher: Hub::new(&self.topic)?,
            frame_id: "laser_frame".to_string(),
            scan_frequency: 10.0,
            min_range: 0.1,
            max_range,
            angle_increment: std::f32::consts::PI / 180.0,
            backend: self.backend,
            serial_port: "/dev/ttyUSB0".to_string(),
            is_initialized: false,
            scan_count: 0,
            last_scan_time: 0,
            processor: self.processor,
        })
    }
}
