//! GPS Node - GPS/GNSS Position sensor node
//!
//! This node reads position data from GPS sensors and publishes NavSatFix messages.
//! It uses the driver abstraction layer to support multiple hardware backends.

use crate::NavSatFix;
use horus_core::driver::{Driver, Sensor};
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

// Import driver types
use crate::drivers::gps::{GpsDriver, GpsDriverBackend, SimulationGpsDriver};

#[cfg(feature = "nmea-gps")]
use crate::drivers::gps::NmeaGpsDriver;

/// GPS backend type (deprecated - use GpsDriverBackend instead)
///
/// This enum is kept for backward compatibility. New code should use
/// `GpsDriverBackend` from `crate::drivers::gps`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpsBackend {
    Simulation,
    NmeaSerial,
}

impl From<GpsBackend> for GpsDriverBackend {
    fn from(backend: GpsBackend) -> Self {
        match backend {
            GpsBackend::Simulation => GpsDriverBackend::Simulation,
            #[cfg(feature = "nmea-gps")]
            GpsBackend::NmeaSerial => GpsDriverBackend::Nmea,
            #[cfg(not(feature = "nmea-gps"))]
            GpsBackend::NmeaSerial => GpsDriverBackend::Simulation, // Fallback
        }
    }
}

/// GPS/GNSS Position Node
///
/// Provides GPS/GNSS position data from satellite navigation receivers.
/// Supports various GPS modules via NMEA serial protocol.
/// Publishes latitude, longitude, altitude, and accuracy information.
///
/// # Driver System
///
/// This node uses the HORUS driver abstraction layer. Drivers handle all
/// hardware-specific code, while the node handles HORUS integration (topics,
/// scheduling, lifecycle).
///
/// ## Supported Drivers
///
/// - `SimulationGpsDriver` - Always available, generates synthetic positions
/// - `NmeaGpsDriver` - NMEA serial GPS (requires `nmea-gps` feature)
///
/// # Example
///
/// ```rust,ignore
/// use horus_library::nodes::GpsNode;
/// use horus_library::drivers::SimulationGpsDriver;
///
/// // Using the default simulation driver
/// let node = GpsNode::new()?;
///
/// // Using a specific driver
/// let driver = SimulationGpsDriver::new();
/// let node = GpsNode::with_driver("gps.fix", driver)?;
///
/// // Using the builder for custom configuration
/// let node = GpsNode::builder()
///     .topic("custom_gps")
///     .with_backend(GpsBackend::Simulation)
///     .with_filter(|fix| {
///         // Only publish high-quality fixes
///         if fix.hdop < 2.0 { Some(fix) } else { None }
///     })
///     .build()?;
/// ```
pub struct GpsNode<D = GpsDriver, P = PassThrough<NavSatFix>>
where
    D: Sensor<Output = NavSatFix>,
    P: Processor<NavSatFix>,
{
    publisher: Topic<NavSatFix>,

    // Driver (handles hardware abstraction)
    driver: D,

    // Configuration
    min_satellites: u16,
    max_hdop: f32,
    frame_id: String,

    // State
    last_fix: NavSatFix,
    fix_count: u64,
    last_update_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl GpsNode<GpsDriver, PassThrough<NavSatFix>> {
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
        let driver_backend: GpsDriverBackend = backend.into();
        let driver = GpsDriver::new(driver_backend)?;

        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            min_satellites: 4,
            max_hdop: 20.0,
            frame_id: "gps".to_string(),
            last_fix: NavSatFix::default(),
            fix_count: 0,
            last_update_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> GpsNodeBuilder<GpsDriver, PassThrough<NavSatFix>> {
        GpsNodeBuilder::new()
    }
}

impl<D> GpsNode<D, PassThrough<NavSatFix>>
where
    D: Sensor<Output = NavSatFix>,
{
    /// Create a new GPS node with a custom driver
    ///
    /// This allows using any driver that implements `Sensor<Output = NavSatFix>`,
    /// including custom drivers from the marketplace.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_library::nodes::GpsNode;
    /// use horus_library::drivers::SimulationGpsDriver;
    ///
    /// let driver = SimulationGpsDriver::new();
    /// let node = GpsNode::with_driver("gps.fix", driver)?;
    /// ```
    pub fn with_driver(topic: &str, driver: D) -> Result<Self> {
        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            min_satellites: 4,
            max_hdop: 20.0,
            frame_id: "gps".to_string(),
            last_fix: NavSatFix::default(),
            fix_count: 0,
            last_update_time: 0,
            processor: PassThrough::new(),
        })
    }
}

impl<D, P> GpsNode<D, P>
where
    D: Sensor<Output = NavSatFix>,
    P: Processor<NavSatFix>,
{
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

    /// Get the driver's sample rate (if available)
    pub fn get_sample_rate(&self) -> Option<f32> {
        self.driver.sample_rate()
    }

    /// Check if the driver is available
    pub fn is_driver_available(&self) -> bool {
        self.driver.is_available()
    }

    /// Get the driver ID
    pub fn driver_id(&self) -> &str {
        self.driver.id()
    }

    /// Get the driver name
    pub fn driver_name(&self) -> &str {
        self.driver.name()
    }

    /// Validate GPS fix quality
    fn validate_fix(&self, fix: &NavSatFix, ctx: &mut Option<&mut NodeInfo>) -> bool {
        // Check if coordinates are valid
        if !fix.is_valid() {
            if let Some(c) = ctx.as_mut() {
                c.log_warning("Invalid GPS coordinates");
            }
            return false;
        }

        // Check satellite count
        if fix.satellites_visible < self.min_satellites {
            if let Some(c) = ctx.as_mut() {
                c.log_warning(&format!(
                    "Insufficient satellites: {} < {}",
                    fix.satellites_visible, self.min_satellites
                ));
            }
            return false;
        }

        // Check HDOP
        if fix.hdop > self.max_hdop {
            if let Some(c) = ctx.as_mut() {
                c.log_warning(&format!(
                    "Poor GPS accuracy: HDOP {:.1} > {:.1}",
                    fix.hdop, self.max_hdop
                ));
            }
            return false;
        }

        true
    }
}

impl<D, P> Node for GpsNode<D, P>
where
    D: Sensor<Output = NavSatFix>,
    P: Processor<NavSatFix>,
{
    fn name(&self) -> &'static str {
        "GpsNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Initialize the driver
        self.driver.init()?;

        // Initialize processor
        self.processor.on_start();

        ctx.log_info(&format!(
            "GpsNode initialized with driver: {} ({})",
            self.driver.name(),
            self.driver.id()
        ));

        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("GpsNode shutting down - closing GPS connection");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Shutdown driver
        self.driver.shutdown()?;

        ctx.log_info("GPS connection closed safely");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Check if driver has data available
        if !self.driver.has_data() {
            return;
        }

        // Read and publish GPS data (through processor pipeline)
        match self.driver.read() {
            Ok(fix) => {
                // Validate fix quality
                if self.validate_fix(&fix, &mut ctx) {
                    self.last_fix = fix.clone();
                    self.fix_count += 1;
                    self.last_update_time = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_millis() as u64;

                    // Process through pipeline (filter/transform)
                    if let Some(processed) = self.processor.process(fix) {
                        let _ = self.publisher.send(processed, &mut None);
                    }
                }
            }
            Err(e) => {
                // Log error but continue - sensor might recover
                eprintln!("GpsNode: Failed to read data: {}", e);
            }
        }
    }
}

/// Builder for GpsNode with custom processor
pub struct GpsNodeBuilder<D, P>
where
    D: Sensor<Output = NavSatFix>,
    P: Processor<NavSatFix>,
{
    topic: String,
    driver: Option<D>,
    backend: GpsBackend,
    processor: P,
}

impl GpsNodeBuilder<GpsDriver, PassThrough<NavSatFix>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "gps.fix".to_string(),
            driver: None,
            backend: GpsBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for GpsNodeBuilder<GpsDriver, PassThrough<NavSatFix>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D, P> GpsNodeBuilder<D, P>
where
    D: Sensor<Output = NavSatFix>,
    P: Processor<NavSatFix>,
{
    /// Set the topic for publishing GPS fixes
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set the GPS backend (creates appropriate driver on build)
    pub fn backend(mut self, backend: GpsBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Alias for backend
    pub fn with_backend(self, backend: GpsBackend) -> Self {
        self.backend(backend)
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> GpsNodeBuilder<D, P2>
    where
        P2: Processor<NavSatFix>,
    {
        GpsNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor,
        }
    }

    /// Add a closure processor for transformations
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> GpsNodeBuilder<D, ClosureProcessor<NavSatFix, NavSatFix, F>>
    where
        F: FnMut(NavSatFix) -> NavSatFix + Send + 'static,
    {
        GpsNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> GpsNodeBuilder<D, FilterProcessor<NavSatFix, NavSatFix, F>>
    where
        F: FnMut(NavSatFix) -> Option<NavSatFix> + Send + 'static,
    {
        GpsNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> GpsNodeBuilder<D, Pipeline<NavSatFix, NavSatFix, NavSatFix, P, P2>>
    where
        P2: Processor<NavSatFix, Output = NavSatFix>,
        P: Processor<NavSatFix, Output = NavSatFix>,
    {
        GpsNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }
}

impl<P> GpsNodeBuilder<GpsDriver, P>
where
    P: Processor<NavSatFix>,
{
    /// Build the node with GpsDriver (default driver type)
    pub fn build(self) -> Result<GpsNode<GpsDriver, P>> {
        let driver_backend: GpsDriverBackend = self.backend.into();
        let driver = GpsDriver::new(driver_backend)?;

        Ok(GpsNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            min_satellites: 4,
            max_hdop: 20.0,
            frame_id: "gps".to_string(),
            last_fix: NavSatFix::default(),
            fix_count: 0,
            last_update_time: 0,
            processor: self.processor,
        })
    }
}

// Builder for custom drivers
impl<D, P> GpsNodeBuilder<D, P>
where
    D: Sensor<Output = NavSatFix>,
    P: Processor<NavSatFix>,
{
    /// Set a custom driver
    pub fn with_driver<D2>(self, driver: D2) -> GpsNodeBuilder<D2, P>
    where
        D2: Sensor<Output = NavSatFix>,
    {
        GpsNodeBuilder {
            topic: self.topic,
            driver: Some(driver),
            backend: self.backend,
            processor: self.processor,
        }
    }

    /// Build the node with a custom driver (requires driver to be set)
    pub fn build_with_driver(self) -> Result<GpsNode<D, P>>
    where
        D: Default,
    {
        let driver = self.driver.unwrap_or_default();

        Ok(GpsNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            min_satellites: 4,
            max_hdop: 20.0,
            frame_id: "gps".to_string(),
            last_fix: NavSatFix::default(),
            fix_count: 0,
            last_update_time: 0,
            processor: self.processor,
        })
    }
}

// Convenience type aliases for common driver types
/// GpsNode with SimulationGpsDriver
pub type SimulationGpsNode<P = PassThrough<NavSatFix>> = GpsNode<SimulationGpsDriver, P>;

#[cfg(feature = "nmea-gps")]
/// GpsNode with NmeaGpsDriver
pub type NmeaGpsNode<P = PassThrough<NavSatFix>> = GpsNode<NmeaGpsDriver, P>;
