//! LiDAR Node - Generic LiDAR interface for obstacle detection and mapping
//!
//! This node reads laser scan data from LiDAR sensors and publishes LaserScan messages.
//! It uses the driver abstraction layer to support multiple hardware backends.

use crate::LaserScan;
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
use crate::drivers::lidar::{LidarDriver, LidarDriverBackend, SimulationLidarDriver};

#[cfg(feature = "rplidar")]
use crate::drivers::lidar::RplidarDriver;

/// LiDAR backend type (deprecated - use LidarDriverBackend instead)
///
/// This enum is kept for backward compatibility. New code should use
/// `LidarDriverBackend` from `crate::drivers::lidar`.
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

impl From<LidarBackend> for LidarDriverBackend {
    fn from(backend: LidarBackend) -> Self {
        match backend {
            LidarBackend::Simulation => LidarDriverBackend::Simulation,
            #[cfg(feature = "rplidar")]
            LidarBackend::RplidarA1 | LidarBackend::RplidarA2 | LidarBackend::RplidarA3 => {
                LidarDriverBackend::Rplidar
            }
            #[cfg(not(feature = "rplidar"))]
            LidarBackend::RplidarA1 | LidarBackend::RplidarA2 | LidarBackend::RplidarA3 => {
                LidarDriverBackend::Simulation
            }
            // YDLIDAR not yet supported - fall back to simulation
            LidarBackend::YdlidarX2 | LidarBackend::YdlidarX4 | LidarBackend::YdlidarTMiniPro => {
                LidarDriverBackend::Simulation
            }
        }
    }
}

/// LiDAR Node - Generic LiDAR interface for obstacle detection and mapping
///
/// Captures laser scan data from various LiDAR sensors and publishes LaserScan messages.
/// Supports multiple hardware backends through the driver abstraction layer.
///
/// # Driver System
///
/// This node uses the HORUS driver abstraction layer. Drivers handle all
/// hardware-specific code, while the node handles HORUS integration (topics,
/// scheduling, lifecycle).
///
/// ## Supported Drivers
///
/// - `SimulationLidarDriver` - Always available, generates synthetic scans
/// - `RplidarDriver` - RPLidar A2/A3 (requires `rplidar` feature)
///
/// # Example
///
/// ```rust,ignore
/// use horus_library::nodes::LidarNode;
/// use horus_library::drivers::SimulationLidarDriver;
///
/// // Using the default simulation driver
/// let node = LidarNode::new()?;
///
/// // Using a specific driver
/// let driver = SimulationLidarDriver::new();
/// let node = LidarNode::with_driver("scan", driver)?;
///
/// // Using the builder for custom configuration
/// let node = LidarNode::builder()
///     .topic("custom_scan")
///     .with_backend(LidarBackend::Simulation)
///     .with_closure(|mut scan| {
///         // Apply range filtering
///         for r in scan.ranges.iter_mut() {
///             if *r < 0.1 { *r = f32::INFINITY; }
///         }
///         scan
///     })
///     .build()?;
/// ```
pub struct LidarNode<D = LidarDriver, P = PassThrough<LaserScan>>
where
    D: Sensor<Output = LaserScan>,
    P: Processor<LaserScan>,
{
    publisher: Topic<LaserScan>,

    // Driver (handles hardware abstraction)
    driver: D,

    // Configuration
    frame_id: String,
    scan_frequency: f32,
    min_range: f32,
    max_range: f32,
    angle_increment: f32,

    // State
    is_initialized: bool,
    scan_count: u64,
    last_scan_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl LidarNode<LidarDriver, PassThrough<LaserScan>> {
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
        let driver_backend: LidarDriverBackend = backend.into();
        let driver = LidarDriver::new(driver_backend)?;

        let max_range = match backend {
            LidarBackend::RplidarA1 => 12.0,
            LidarBackend::RplidarA2 => 16.0,
            LidarBackend::RplidarA3 => 25.0,
            LidarBackend::YdlidarX2 => 8.0,
            LidarBackend::YdlidarX4 => 10.0,
            LidarBackend::YdlidarTMiniPro => 12.0,
            _ => 30.0,
        };

        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            frame_id: "laser_frame".to_string(),
            scan_frequency: 10.0,
            min_range: 0.1,
            max_range,
            angle_increment: std::f32::consts::PI / 180.0,
            is_initialized: false,
            scan_count: 0,
            last_scan_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> LidarNodeBuilder<LidarDriver, PassThrough<LaserScan>> {
        LidarNodeBuilder::new()
    }
}

impl<D> LidarNode<D, PassThrough<LaserScan>>
where
    D: Sensor<Output = LaserScan>,
{
    /// Create a new LiDAR node with a custom driver
    ///
    /// This allows using any driver that implements `Sensor<Output = LaserScan>`,
    /// including custom drivers from the marketplace.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_library::nodes::LidarNode;
    /// use horus_library::drivers::SimulationLidarDriver;
    ///
    /// let driver = SimulationLidarDriver::new();
    /// let node = LidarNode::with_driver("scan", driver)?;
    /// ```
    pub fn with_driver(topic: &str, driver: D) -> Result<Self> {
        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            frame_id: "laser_frame".to_string(),
            scan_frequency: 10.0,
            min_range: 0.1,
            max_range: 30.0,
            angle_increment: std::f32::consts::PI / 180.0,
            is_initialized: false,
            scan_count: 0,
            last_scan_time: 0,
            processor: PassThrough::new(),
        })
    }
}

impl<D, P> LidarNode<D, P>
where
    D: Sensor<Output = LaserScan>,
    P: Processor<LaserScan>,
{
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
}

impl<D, P> Node for LidarNode<D, P>
where
    D: Sensor<Output = LaserScan>,
    P: Processor<LaserScan>,
{
    fn name(&self) -> &'static str {
        "LidarNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Initialize the driver
        self.driver.init()?;
        self.is_initialized = true;

        // Initialize processor
        self.processor.on_start();

        ctx.log_info(&format!(
            "LidarNode initialized with driver: {} ({})",
            self.driver.name(),
            self.driver.id()
        ));

        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("LidarNode shutting down - stopping LiDAR sensor");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Shutdown driver
        self.driver.shutdown()?;

        self.is_initialized = false;
        ctx.log_info("LiDAR sensor stopped safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Check if driver has data available
        if !self.driver.has_data() {
            return;
        }

        // Read and publish scan data (through processor pipeline)
        match self.driver.read() {
            Ok(scan) => {
                self.scan_count += 1;
                self.last_scan_time = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                // Process through pipeline (filter/transform)
                if let Some(processed) = self.processor.process(scan) {
                    let _ = self.publisher.send(processed, &mut None);
                }
            }
            Err(e) => {
                // Log error but continue - sensor might recover
                eprintln!("LidarNode: Failed to read data: {}", e);
            }
        }
    }
}

/// Builder for LidarNode with custom processor
pub struct LidarNodeBuilder<D, P>
where
    D: Sensor<Output = LaserScan>,
    P: Processor<LaserScan>,
{
    topic: String,
    driver: Option<D>,
    backend: LidarBackend,
    processor: P,
}

impl LidarNodeBuilder<LidarDriver, PassThrough<LaserScan>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "scan".to_string(),
            driver: None,
            backend: LidarBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for LidarNodeBuilder<LidarDriver, PassThrough<LaserScan>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D, P> LidarNodeBuilder<D, P>
where
    D: Sensor<Output = LaserScan>,
    P: Processor<LaserScan>,
{
    /// Set the topic for publishing laser scans
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set the LiDAR backend (creates appropriate driver on build)
    pub fn backend(mut self, backend: LidarBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Alias for backend
    pub fn with_backend(self, backend: LidarBackend) -> Self {
        self.backend(backend)
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> LidarNodeBuilder<D, P2>
    where
        P2: Processor<LaserScan>,
    {
        LidarNodeBuilder {
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
    ) -> LidarNodeBuilder<D, ClosureProcessor<LaserScan, LaserScan, F>>
    where
        F: FnMut(LaserScan) -> LaserScan + Send + 'static,
    {
        LidarNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> LidarNodeBuilder<D, FilterProcessor<LaserScan, LaserScan, F>>
    where
        F: FnMut(LaserScan) -> Option<LaserScan> + Send + 'static,
    {
        LidarNodeBuilder {
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
    ) -> LidarNodeBuilder<D, Pipeline<LaserScan, LaserScan, LaserScan, P, P2>>
    where
        P2: Processor<LaserScan, Output = LaserScan>,
        P: Processor<LaserScan, Output = LaserScan>,
    {
        LidarNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }
}

impl<P> LidarNodeBuilder<LidarDriver, P>
where
    P: Processor<LaserScan>,
{
    /// Build the node with LidarDriver (default driver type)
    pub fn build(self) -> Result<LidarNode<LidarDriver, P>> {
        let driver_backend: LidarDriverBackend = self.backend.into();
        let driver = LidarDriver::new(driver_backend)?;

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
            publisher: Topic::new(&self.topic)?,
            driver,
            frame_id: "laser_frame".to_string(),
            scan_frequency: 10.0,
            min_range: 0.1,
            max_range,
            angle_increment: std::f32::consts::PI / 180.0,
            is_initialized: false,
            scan_count: 0,
            last_scan_time: 0,
            processor: self.processor,
        })
    }
}

// Builder for custom drivers
impl<D, P> LidarNodeBuilder<D, P>
where
    D: Sensor<Output = LaserScan>,
    P: Processor<LaserScan>,
{
    /// Set a custom driver
    pub fn with_driver<D2>(self, driver: D2) -> LidarNodeBuilder<D2, P>
    where
        D2: Sensor<Output = LaserScan>,
    {
        LidarNodeBuilder {
            topic: self.topic,
            driver: Some(driver),
            backend: self.backend,
            processor: self.processor,
        }
    }

    /// Build the node with a custom driver (requires driver to be set)
    pub fn build_with_driver(self) -> Result<LidarNode<D, P>>
    where
        D: Default,
    {
        let driver = self.driver.unwrap_or_default();

        Ok(LidarNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            frame_id: "laser_frame".to_string(),
            scan_frequency: 10.0,
            min_range: 0.1,
            max_range: 30.0,
            angle_increment: std::f32::consts::PI / 180.0,
            is_initialized: false,
            scan_count: 0,
            last_scan_time: 0,
            processor: self.processor,
        })
    }
}

// Convenience type aliases for common driver types
/// LidarNode with SimulationLidarDriver
pub type SimulationLidarNode<P = PassThrough<LaserScan>> = LidarNode<SimulationLidarDriver, P>;

#[cfg(feature = "rplidar")]
/// LidarNode with RplidarDriver
pub type RplidarLidarNode<P = PassThrough<LaserScan>> = LidarNode<RplidarDriver, P>;
