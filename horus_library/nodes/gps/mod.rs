use crate::NavSatFix;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(feature = "nmea-gps")]
use serialport::SerialPort;

#[cfg(feature = "nmea-gps")]
use nmea::Nmea;

#[cfg(feature = "nmea-gps")]
use std::io::{BufRead, BufReader};

/// GPS backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpsBackend {
    Simulation,
    NmeaSerial,
}

/// GPS/GNSS Position Node
///
/// Provides GPS/GNSS position data from satellite navigation receivers.
/// Supports various GPS modules via NMEA serial protocol.
/// Publishes latitude, longitude, altitude, and accuracy information.
///
/// Supported backends:
/// - NMEA Serial (most GPS modules: u-blox, MTK, etc.)
/// - Simulation mode for testing
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = GpsNode::builder()
///     .with_filter(|fix| {
///         // Only publish high-quality fixes
///         if fix.hdop < 2.0 { Some(fix) } else { None }
///     })
///     .build()?;
/// ```
pub struct GpsNode<P = PassThrough<NavSatFix>>
where
    P: Processor<NavSatFix>,
{
    publisher: Hub<NavSatFix>,

    // Configuration
    update_rate_hz: f32,
    min_satellites: u16,
    max_hdop: f32,
    frame_id: String,
    backend: GpsBackend,
    serial_port: String,
    baud_rate: u32,

    // State
    last_fix: NavSatFix,
    fix_count: u64,
    last_update_time: u64,

    // Hardware drivers
    #[cfg(feature = "nmea-gps")]
    nmea_parser: Option<Nmea>,

    #[cfg(feature = "nmea-gps")]
    serial: Option<Box<dyn SerialPort>>,

    // Simulation state
    sim_latitude: f64,
    sim_longitude: f64,
    sim_altitude: f64,

    // Processor for hybrid pattern
    processor: P,
}

impl GpsNode {
    /// Create a new GPS node with default topic "gps.fix" in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("gps.fix", GpsBackend::Simulation)
    }

    /// Create a new GPS node with custom topic in simulation mode
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        Self::new_with_backend(topic, GpsBackend::Simulation)
    }

    /// Create a new GPS node with specific backend
    pub fn new_with_backend(topic: &str, backend: GpsBackend) -> Result<Self> {
        Ok(Self {
            publisher: Hub::new(topic)?,
            update_rate_hz: 1.0, // 1 Hz default (typical for GPS)
            min_satellites: 4,   // Minimum for 3D fix
            max_hdop: 20.0,      // Maximum acceptable HDOP
            frame_id: "gps".to_string(),
            backend,
            serial_port: "/dev/ttyUSB0".to_string(), // Common GPS serial port
            baud_rate: 9600,                         // Standard GPS baud rate
            last_fix: NavSatFix::default(),
            fix_count: 0,
            last_update_time: 0,
            #[cfg(feature = "nmea-gps")]
            nmea_parser: None,
            #[cfg(feature = "nmea-gps")]
            serial: None,
            sim_latitude: 37.7749, // San Francisco (default)
            sim_longitude: -122.4194,
            sim_altitude: 10.0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> GpsNodeBuilder<PassThrough<NavSatFix>> {
        GpsNodeBuilder::new()
    }

    /// Set GPS backend
    pub fn set_backend(&mut self, backend: GpsBackend) {
        self.backend = backend;
    }

    /// Set serial port configuration for NMEA GPS
    pub fn set_serial_config(&mut self, port: &str, baud_rate: u32) {
        self.serial_port = port.to_string();
        self.baud_rate = baud_rate;
    }

    /// Set GPS update rate in Hz (typically 1-10 Hz)
    pub fn set_update_rate(&mut self, rate_hz: f32) {
        self.update_rate_hz = rate_hz.clamp(0.1, 20.0);
    }

    /// Set minimum number of satellites required for valid fix
    pub fn set_min_satellites(&mut self, count: u16) {
        self.min_satellites = count;
    }

    /// Set maximum acceptable HDOP
    pub fn set_max_hdop(&mut self, hdop: f32) {
        self.max_hdop = hdop;
    }

    /// Set coordinate frame ID
    pub fn set_frame_id(&mut self, frame_id: &str) {
        self.frame_id = frame_id.to_string();
    }

    /// Get last GPS fix
    pub fn get_last_fix(&self) -> &NavSatFix {
        &self.last_fix
    }

    /// Get number of fixes received
    pub fn get_fix_count(&self) -> u64 {
        self.fix_count
    }

    /// Check if we have a valid GPS fix
    pub fn has_valid_fix(&self) -> bool {
        self.last_fix.has_fix()
            && self.last_fix.satellites_visible >= self.min_satellites
            && self.last_fix.hdop <= self.max_hdop
    }

    /// Set simulation coordinates
    pub fn set_simulation_position(&mut self, lat: f64, lon: f64, alt: f64) {
        self.sim_latitude = lat;
        self.sim_longitude = lon;
        self.sim_altitude = alt;
    }

    /// Initialize GPS hardware
    fn initialize_gps(&mut self) -> bool {
        match self.backend {
            GpsBackend::Simulation => {
                // Simulation mode requires no hardware initialization
                true
            }
            #[cfg(feature = "nmea-gps")]
            GpsBackend::NmeaSerial => {
                use serialport::available_ports;

                // Check if port exists
                if let Ok(ports) = available_ports() {
                    let port_exists = ports.iter().any(|p| p.port_name == self.serial_port);
                    if !port_exists {
                        eprintln!("Serial port {} not found", self.serial_port);
                        eprintln!("Available ports:");
                        for port in ports {
                            eprintln!("  - {}", port.port_name);
                        }
                        return false;
                    }
                }

                // Open serial port
                match serialport::new(&self.serial_port, self.baud_rate)
                    .timeout(std::time::Duration::from_millis(1000))
                    .open()
                {
                    Ok(port) => {
                        self.serial = Some(port);
                        self.nmea_parser = Some(Nmea::default());
                        true
                    }
                    Err(e) => {
                        eprintln!(
                            "Failed to open GPS serial port {}: {:?}",
                            self.serial_port, e
                        );
                        false
                    }
                }
            }
            #[cfg(not(feature = "nmea-gps"))]
            GpsBackend::NmeaSerial => {
                eprintln!("NMEA GPS backend requires 'nmea-gps' feature to be enabled");
                false
            }
        }
    }

    /// Read GPS data from receiver
    fn read_gps(&mut self, mut ctx: Option<&mut NodeInfo>) -> Option<NavSatFix> {
        // Check if enough time has passed for next update
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        let update_interval_ms = (1000.0 / self.update_rate_hz) as u64;
        if current_time - self.last_update_time < update_interval_ms {
            return None;
        }

        self.last_update_time = current_time;

        match self.backend {
            GpsBackend::Simulation => {
                let mut fix = NavSatFix::from_coordinates(
                    self.sim_latitude,
                    self.sim_longitude,
                    self.sim_altitude,
                );

                // Add realistic GPS characteristics
                fix.satellites_visible = 8;
                fix.hdop = 1.2;
                fix.vdop = 1.8;
                fix.speed = 0.0;
                fix.heading = 0.0;
                fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

                // Set covariance (rough GPS accuracy ~3m)
                fix.position_covariance[0] = 9.0; // lat variance
                fix.position_covariance[4] = 9.0; // lon variance
                fix.position_covariance[8] = 16.0; // alt variance

                ctx.log_debug(&format!(
                    "GPS: {:.6}, {:.6}, alt={:.1}m, sats={}",
                    fix.latitude, fix.longitude, fix.altitude, fix.satellites_visible
                ));

                Some(fix)
            }
            #[cfg(feature = "nmea-gps")]
            GpsBackend::NmeaSerial => {
                if let (Some(ref mut serial), Some(ref mut nmea)) =
                    (&mut self.serial, &mut self.nmea_parser)
                {
                    let mut reader = BufReader::new(serial.try_clone().ok()?);
                    let mut line = String::new();

                    // Try to read a line from serial port
                    match reader.read_line(&mut line) {
                        Ok(0) => None, // EOF
                        Ok(_) => {
                            // Parse NMEA sentence
                            if let Ok(_) = nmea.parse(&line) {
                                // Check if we have a valid fix
                                if let (Some(lat), Some(lon)) = (nmea.latitude, nmea.longitude) {
                                    let mut fix = NavSatFix::from_coordinates(
                                        lat,
                                        lon,
                                        nmea.altitude.unwrap_or(0.0) as f64,
                                    );

                                    fix.satellites_visible =
                                        nmea.num_of_fix_satellites.unwrap_or(0) as u16;
                                    fix.hdop = nmea.hdop.unwrap_or(99.99) as f32;
                                    fix.vdop = nmea.vdop.unwrap_or(99.99) as f32;
                                    fix.speed = nmea.speed_over_ground.unwrap_or(0.0) as f32;
                                    fix.heading = nmea.true_course.unwrap_or(0.0) as f32;

                                    // Estimate covariance from HDOP (rough approximation)
                                    let horizontal_accuracy = fix.hdop * 5.0; // meters
                                    fix.position_covariance[0] =
                                        (horizontal_accuracy * horizontal_accuracy) as f64;
                                    fix.position_covariance[4] =
                                        (horizontal_accuracy * horizontal_accuracy) as f64;
                                    fix.position_covariance[8] =
                                        (horizontal_accuracy * horizontal_accuracy * 2.0) as f64;
                                    fix.position_covariance_type =
                                        NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

                                    ctx.log_debug(&format!(
                                        "GPS NMEA: {:.6}, {:.6}, alt={:.1}m, sats={}, HDOP={:.1}",
                                        fix.latitude,
                                        fix.longitude,
                                        fix.altitude,
                                        fix.satellites_visible,
                                        fix.hdop
                                    ));

                                    Some(fix)
                                } else {
                                    None
                                }
                            } else {
                                None
                            }
                        }
                        Err(_) => None,
                    }
                } else {
                    None
                }
            }
            #[cfg(not(feature = "nmea-gps"))]
            GpsBackend::NmeaSerial => {
                ctx.log_warning("NMEA GPS backend requires 'nmea-gps' feature");
                None
            }
        }
    }

    /// Validate GPS fix quality
    fn validate_fix(&self, fix: &NavSatFix, mut ctx: Option<&mut NodeInfo>) -> bool {
        // Check if coordinates are valid
        if !fix.is_valid() {
            ctx.log_warning("Invalid GPS coordinates");
            return false;
        }

        // Check satellite count
        if fix.satellites_visible < self.min_satellites {
            ctx.log_warning(&format!(
                "Insufficient satellites: {} < {}",
                fix.satellites_visible, self.min_satellites
            ));
            return false;
        }

        // Check HDOP
        if fix.hdop > self.max_hdop {
            ctx.log_warning(&format!(
                "Poor GPS accuracy: HDOP {:.1} > {:.1}",
                fix.hdop, self.max_hdop
            ));
            return false;
        }

        true
    }
}

impl<P> Node for GpsNode<P>
where
    P: Processor<NavSatFix>,
{
    fn name(&self) -> &'static str {
        "GpsNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        ctx.log_info("GpsNode shutting down - closing serial connection");

        // Close serial port
        #[cfg(feature = "nmea-gps")]
        {
            self.serial = None;
            self.nmea_parser = None;
        }

        ctx.log_info("GPS serial connection closed safely");
        Ok(())
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("GPS node initialized");

        match self.backend {
            GpsBackend::Simulation => {
                ctx.log_info("GPS simulation mode enabled");
            }
            GpsBackend::NmeaSerial => {
                ctx.log_info(&format!(
                    "GPS NMEA serial: {} @ {} baud",
                    self.serial_port, self.baud_rate
                ));
            }
        }

        // Initialize hardware
        if !self.initialize_gps() {
            ctx.log_error("Failed to initialize GPS hardware");
        }

        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        // Read GPS data
        if let Some(fix) = self.read_gps(ctx.as_deref_mut()) {
            // Validate fix quality
            if self.validate_fix(&fix, ctx.as_deref_mut()) {
                self.last_fix = fix;
                self.fix_count += 1;

                // Process through pipeline and publish
                if let Some(processed) = self.processor.process(fix) {
                    let _ = self.publisher.send(processed, &mut None);
                }
            }
        }
    }
}

/// Builder for GpsNode with fluent API for processor configuration
pub struct GpsNodeBuilder<P>
where
    P: Processor<NavSatFix>,
{
    topic: String,
    backend: GpsBackend,
    processor: P,
}

impl GpsNodeBuilder<PassThrough<NavSatFix>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "gps.fix".to_string(),
            backend: GpsBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl<P> GpsNodeBuilder<P>
where
    P: Processor<NavSatFix>,
{
    /// Set the topic for publishing GPS fixes
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set the GPS backend
    pub fn backend(mut self, backend: GpsBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> GpsNodeBuilder<P2>
    where
        P2: Processor<NavSatFix>,
    {
        GpsNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor,
        }
    }

    /// Add a closure processor for transformations
    pub fn with_closure<F>(self, f: F) -> GpsNodeBuilder<ClosureProcessor<NavSatFix, NavSatFix, F>>
    where
        F: FnMut(NavSatFix) -> NavSatFix + Send + 'static,
    {
        GpsNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> GpsNodeBuilder<FilterProcessor<NavSatFix, NavSatFix, F>>
    where
        F: FnMut(NavSatFix) -> Option<NavSatFix> + Send + 'static,
    {
        GpsNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> GpsNodeBuilder<Pipeline<NavSatFix, NavSatFix, NavSatFix, P, P2>>
    where
        P2: Processor<NavSatFix, Output = NavSatFix>,
        P: Processor<NavSatFix, Output = NavSatFix>,
    {
        GpsNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the GpsNode
    pub fn build(self) -> Result<GpsNode<P>> {
        Ok(GpsNode {
            publisher: Hub::new(&self.topic)?,
            update_rate_hz: 1.0,
            min_satellites: 4,
            max_hdop: 20.0,
            frame_id: "gps".to_string(),
            backend: self.backend,
            serial_port: "/dev/ttyUSB0".to_string(),
            baud_rate: 9600,
            last_fix: NavSatFix::default(),
            fix_count: 0,
            last_update_time: 0,
            #[cfg(feature = "nmea-gps")]
            nmea_parser: None,
            #[cfg(feature = "nmea-gps")]
            serial: None,
            sim_latitude: 37.7749,
            sim_longitude: -122.4194,
            sim_altitude: 10.0,
            processor: self.processor,
        })
    }
}
