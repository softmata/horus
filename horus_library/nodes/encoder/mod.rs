//! Encoder Node - Wheel/joint position feedback for odometry and control
//!
//! This node reads encoder data from wheels or joints and publishes Odometry messages.
//! It uses the driver abstraction layer to support multiple hardware backends.

use crate::Odometry;
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
use crate::drivers::encoder::{EncoderDriver, EncoderDriverBackend, SimulationEncoderDriver};

#[cfg(feature = "gpio-hardware")]
use crate::drivers::encoder::GpioEncoderDriver;

/// Encoder backend type (deprecated - use EncoderDriverBackend instead)
///
/// This enum is kept for backward compatibility. New code should use
/// `EncoderDriverBackend` from `crate::drivers::encoder`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EncoderBackend {
    Simulation,
    GpioQuadrature,
}

impl From<EncoderBackend> for EncoderDriverBackend {
    fn from(backend: EncoderBackend) -> Self {
        match backend {
            EncoderBackend::Simulation => EncoderDriverBackend::Simulation,
            #[cfg(feature = "gpio-hardware")]
            EncoderBackend::GpioQuadrature => EncoderDriverBackend::Gpio,
            #[cfg(not(feature = "gpio-hardware"))]
            EncoderBackend::GpioQuadrature => EncoderDriverBackend::Simulation, // Fallback
        }
    }
}

/// Encoder Node - Wheel/joint position feedback for odometry and control
///
/// Reads encoder data from wheels or joints and publishes position, velocity,
/// and odometry information for robot navigation and control feedback.
///
/// # Driver System
///
/// This node uses the HORUS driver abstraction layer. Drivers handle all
/// hardware-specific code, while the node handles HORUS integration (topics,
/// scheduling, lifecycle).
///
/// ## Supported Drivers
///
/// - `SimulationEncoderDriver` - Always available, generates synthetic encoder data
/// - `GpioEncoderDriver` - GPIO quadrature encoder (requires `gpio-hardware` feature)
///
/// # Example
///
/// ```rust,ignore
/// use horus_library::nodes::EncoderNode;
/// use horus_library::drivers::SimulationEncoderDriver;
///
/// // Using the default simulation driver
/// let node = EncoderNode::new()?;
///
/// // Using a specific driver
/// let driver = SimulationEncoderDriver::new();
/// let node = EncoderNode::with_driver("odom", driver)?;
///
/// // Using the builder for custom configuration
/// let node = EncoderNode::builder()
///     .topic("custom_odom")
///     .with_backend(EncoderBackend::Simulation)
///     .with_filter(|odom| {
///         // Only publish when moving
///         if odom.twist.linear[0].abs() > 0.01 { Some(odom) } else { None }
///     })
///     .build()?;
/// ```
pub struct EncoderNode<D = EncoderDriver, P = PassThrough<Odometry>>
where
    D: Sensor<Output = Odometry>,
    P: Processor<Odometry>,
{
    publisher: Topic<Odometry>,

    // Driver (handles hardware abstraction)
    driver: D,

    // Configuration
    frame_id: String,
    child_frame_id: String,

    // State
    is_initialized: bool,
    sample_count: u64,
    last_sample_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl EncoderNode<EncoderDriver, PassThrough<Odometry>> {
    /// Create a new encoder node with default topic "odom" in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("odom", EncoderBackend::Simulation)
    }

    /// Create a new encoder node with custom topic in simulation mode
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        Self::new_with_backend(topic, EncoderBackend::Simulation)
    }

    /// Create a new encoder node with specific backend
    pub fn new_with_backend(topic: &str, backend: EncoderBackend) -> Result<Self> {
        let driver_backend: EncoderDriverBackend = backend.into();
        let driver = EncoderDriver::new(driver_backend)?;

        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            frame_id: "odom".to_string(),
            child_frame_id: "base_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> EncoderNodeBuilder<EncoderDriver, PassThrough<Odometry>> {
        EncoderNodeBuilder::new()
    }
}

impl<D> EncoderNode<D, PassThrough<Odometry>>
where
    D: Sensor<Output = Odometry>,
{
    /// Create a new encoder node with a custom driver
    ///
    /// This allows using any driver that implements `Sensor<Output = Odometry>`,
    /// including custom drivers from the marketplace.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_library::nodes::EncoderNode;
    /// use horus_library::drivers::SimulationEncoderDriver;
    ///
    /// let driver = SimulationEncoderDriver::new();
    /// let node = EncoderNode::with_driver("odom", driver)?;
    /// ```
    pub fn with_driver(topic: &str, driver: D) -> Result<Self> {
        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            frame_id: "odom".to_string(),
            child_frame_id: "base_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: PassThrough::new(),
        })
    }
}

impl<D, P> EncoderNode<D, P>
where
    D: Sensor<Output = Odometry>,
    P: Processor<Odometry>,
{
    /// Set coordinate frame IDs
    pub fn set_frame_ids(&mut self, frame_id: &str, child_frame_id: &str) {
        self.frame_id = frame_id.to_string();
        self.child_frame_id = child_frame_id.to_string();
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

    /// Get total samples received
    pub fn get_sample_count(&self) -> u64 {
        self.sample_count
    }
}

impl<D, P> Node for EncoderNode<D, P>
where
    D: Sensor<Output = Odometry>,
    P: Processor<Odometry>,
{
    fn name(&self) -> &'static str {
        "EncoderNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Initialize the driver
        self.driver.init()?;
        self.is_initialized = true;

        // Initialize processor
        self.processor.on_start();

        ctx.log_info(&format!(
            "EncoderNode initialized with driver: {} ({})",
            self.driver.name(),
            self.driver.id()
        ));

        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("EncoderNode shutting down - releasing encoder resources");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Shutdown driver
        self.driver.shutdown()?;

        self.is_initialized = false;
        ctx.log_info("Encoder resources released safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Check if driver has data available
        if !self.driver.has_data() {
            return;
        }

        // Read and publish odometry data (through processor pipeline)
        match self.driver.read() {
            Ok(odom) => {
                self.sample_count += 1;
                self.last_sample_time = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                // Process through pipeline (filter/transform)
                if let Some(processed) = self.processor.process(odom) {
                    let _ = self.publisher.send(processed, &mut None);
                }
            }
            Err(e) => {
                // Log error but continue - sensor might recover
                eprintln!("EncoderNode: Failed to read data: {}", e);
            }
        }
    }
}

/// Builder for EncoderNode with custom processor
pub struct EncoderNodeBuilder<D, P>
where
    D: Sensor<Output = Odometry>,
    P: Processor<Odometry>,
{
    topic: String,
    driver: Option<D>,
    backend: EncoderBackend,
    processor: P,
}

impl EncoderNodeBuilder<EncoderDriver, PassThrough<Odometry>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "odom".to_string(),
            driver: None,
            backend: EncoderBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for EncoderNodeBuilder<EncoderDriver, PassThrough<Odometry>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D, P> EncoderNodeBuilder<D, P>
where
    D: Sensor<Output = Odometry>,
    P: Processor<Odometry>,
{
    /// Set the topic for publishing odometry
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set the encoder backend (creates appropriate driver on build)
    pub fn backend(mut self, backend: EncoderBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Alias for backend
    pub fn with_backend(self, backend: EncoderBackend) -> Self {
        self.backend(backend)
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> EncoderNodeBuilder<D, P2>
    where
        P2: Processor<Odometry>,
    {
        EncoderNodeBuilder {
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
    ) -> EncoderNodeBuilder<D, ClosureProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Odometry + Send + 'static,
    {
        EncoderNodeBuilder {
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
    ) -> EncoderNodeBuilder<D, FilterProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Option<Odometry> + Send + 'static,
    {
        EncoderNodeBuilder {
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
    ) -> EncoderNodeBuilder<D, Pipeline<Odometry, Odometry, Odometry, P, P2>>
    where
        P2: Processor<Odometry, Output = Odometry>,
        P: Processor<Odometry, Output = Odometry>,
    {
        EncoderNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }
}

impl<P> EncoderNodeBuilder<EncoderDriver, P>
where
    P: Processor<Odometry>,
{
    /// Build the node with EncoderDriver (default driver type)
    pub fn build(self) -> Result<EncoderNode<EncoderDriver, P>> {
        let driver_backend: EncoderDriverBackend = self.backend.into();
        let driver = EncoderDriver::new(driver_backend)?;

        Ok(EncoderNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            frame_id: "odom".to_string(),
            child_frame_id: "base_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: self.processor,
        })
    }
}

// Builder for custom drivers
impl<D, P> EncoderNodeBuilder<D, P>
where
    D: Sensor<Output = Odometry>,
    P: Processor<Odometry>,
{
    /// Set a custom driver
    pub fn with_driver<D2>(self, driver: D2) -> EncoderNodeBuilder<D2, P>
    where
        D2: Sensor<Output = Odometry>,
    {
        EncoderNodeBuilder {
            topic: self.topic,
            driver: Some(driver),
            backend: self.backend,
            processor: self.processor,
        }
    }

    /// Build the node with a custom driver (requires driver to be set)
    pub fn build_with_driver(self) -> Result<EncoderNode<D, P>>
    where
        D: Default,
    {
        let driver = self.driver.unwrap_or_default();

        Ok(EncoderNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            frame_id: "odom".to_string(),
            child_frame_id: "base_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: self.processor,
        })
    }
}

// Convenience type aliases for common driver types
/// EncoderNode with SimulationEncoderDriver
pub type SimulationEncoderNode<P = PassThrough<Odometry>> = EncoderNode<SimulationEncoderDriver, P>;

#[cfg(feature = "gpio-hardware")]
/// EncoderNode with GpioEncoderDriver
pub type GpioEncoderNode<P = PassThrough<Odometry>> = EncoderNode<GpioEncoderDriver, P>;
