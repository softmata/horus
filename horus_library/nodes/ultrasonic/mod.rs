//! Ultrasonic Distance Sensor Node
//!
//! This node reads distance measurements from ultrasonic sensors and publishes Range messages.
//! It uses the driver abstraction layer to support multiple hardware backends.

use crate::Range;
use horus_core::driver::{Driver, Sensor};
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

// Import driver types
use crate::drivers::ultrasonic::{
    SimulationUltrasonicDriver, UltrasonicDriver, UltrasonicDriverBackend,
};

#[cfg(feature = "gpio-hardware")]
use crate::drivers::ultrasonic::GpioUltrasonicDriver;

/// Ultrasonic backend type (deprecated - use UltrasonicDriverBackend instead)
///
/// This enum is kept for backward compatibility. New code should use
/// `UltrasonicDriverBackend` from `crate::drivers::ultrasonic`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UltrasonicBackend {
    Simulation,
    Gpio,
}

impl From<UltrasonicBackend> for UltrasonicDriverBackend {
    fn from(backend: UltrasonicBackend) -> Self {
        match backend {
            UltrasonicBackend::Simulation => UltrasonicDriverBackend::Simulation,
            #[cfg(feature = "gpio-hardware")]
            UltrasonicBackend::Gpio => UltrasonicDriverBackend::Gpio,
            #[cfg(not(feature = "gpio-hardware"))]
            UltrasonicBackend::Gpio => UltrasonicDriverBackend::Simulation, // Fallback
        }
    }
}

/// Ultrasonic Distance Sensor Node
///
/// Reads distance measurements from ultrasonic sensors and publishes Range messages
/// for obstacle detection and distance measurement applications.
///
/// # Driver System
///
/// This node uses the HORUS driver abstraction layer. Drivers handle all
/// hardware-specific code, while the node handles HORUS integration (topics,
/// scheduling, lifecycle).
///
/// ## Supported Drivers
///
/// - `SimulationUltrasonicDriver` - Always available, generates synthetic range data
/// - `GpioUltrasonicDriver` - GPIO echo/trigger driver (requires `gpio-hardware` feature)
///
/// # Supported Sensors
/// - HC-SR04: 2cm-400cm range, 15 degrees beam angle, 5V operation
/// - HC-SR04+: 2cm-400cm range, 15 degrees beam angle, 3.3V/5V operation
/// - US-100: 2cm-450cm range, 15 degrees beam angle, UART or echo/trigger mode
/// - JSN-SR04T: 20cm-600cm range, waterproof, 5V operation
///
/// # Example
///
/// ```rust,ignore
/// use horus_library::nodes::UltrasonicNode;
/// use horus_library::drivers::SimulationUltrasonicDriver;
///
/// // Using the default simulation driver
/// let node = UltrasonicNode::new()?;
///
/// // Using a specific driver
/// let driver = SimulationUltrasonicDriver::new();
/// let node = UltrasonicNode::with_driver("range", driver)?;
///
/// // Using the builder for custom configuration
/// let node = UltrasonicNode::builder()
///     .topic("custom_range")
///     .with_backend(UltrasonicBackend::Simulation)
///     .with_filter(|range| {
///         // Only publish valid ranges
///         if range.range > 0.02 && range.range < 4.0 { Some(range) } else { None }
///     })
///     .build()?;
/// ```
pub struct UltrasonicNode<D = UltrasonicDriver, P = PassThrough<Range>>
where
    D: Sensor<Output = Range>,
    P: Processor<Range>,
{
    publisher: Topic<Range>,

    // Driver (handles hardware abstraction)
    driver: D,

    // Processor for hybrid pattern
    processor: P,

    // State
    is_initialized: bool,
    sample_count: u64,
    last_sample_time: u64,
}

impl UltrasonicNode<UltrasonicDriver, PassThrough<Range>> {
    /// Create a new ultrasonic node with default topic "ultrasonic.range" in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("ultrasonic.range", UltrasonicBackend::Simulation)
    }

    /// Create a new ultrasonic node with custom topic in simulation mode
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        Self::new_with_backend(topic, UltrasonicBackend::Simulation)
    }

    /// Create a new ultrasonic node with specific backend
    pub fn new_with_backend(topic: &str, backend: UltrasonicBackend) -> Result<Self> {
        let driver_backend: UltrasonicDriverBackend = backend.into();
        let driver = UltrasonicDriver::new(driver_backend)?;

        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            processor: PassThrough::new(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> UltrasonicNodeBuilder<UltrasonicDriver, PassThrough<Range>> {
        UltrasonicNodeBuilder::new()
    }
}

impl<D> UltrasonicNode<D, PassThrough<Range>>
where
    D: Sensor<Output = Range>,
{
    /// Create a new ultrasonic node with a custom driver
    ///
    /// This allows using any driver that implements `Sensor<Output = Range>`,
    /// including custom drivers from the marketplace.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_library::nodes::UltrasonicNode;
    /// use horus_library::drivers::SimulationUltrasonicDriver;
    ///
    /// let driver = SimulationUltrasonicDriver::new();
    /// let node = UltrasonicNode::with_driver("range", driver)?;
    /// ```
    pub fn with_driver(topic: &str, driver: D) -> Result<Self> {
        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            processor: PassThrough::new(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
        })
    }
}

impl<D, P> UltrasonicNode<D, P>
where
    D: Sensor<Output = Range>,
    P: Processor<Range>,
{
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

impl<D, P> Node for UltrasonicNode<D, P>
where
    D: Sensor<Output = Range>,
    P: Processor<Range>,
{
    fn name(&self) -> &'static str {
        "UltrasonicNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Initialize the driver
        self.driver.init()?;
        self.is_initialized = true;

        // Initialize processor
        self.processor.on_start();

        ctx.log_info(&format!(
            "UltrasonicNode initialized with driver: {} ({})",
            self.driver.name(),
            self.driver.id()
        ));

        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("UltrasonicNode shutting down - releasing sensor resources");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Shutdown driver
        self.driver.shutdown()?;

        self.is_initialized = false;
        ctx.log_info("Ultrasonic sensor resources released safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Check if driver has data available
        if !self.driver.has_data() {
            return;
        }

        // Read and publish range data (through processor pipeline)
        match self.driver.read() {
            Ok(range) => {
                self.sample_count += 1;
                self.last_sample_time = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                // Process through pipeline (filter/transform)
                if let Some(processed) = self.processor.process(range) {
                    let _ = self.publisher.send(processed, &mut None);
                }
            }
            Err(e) => {
                // Log error but continue - sensor might recover
                eprintln!("UltrasonicNode: Failed to read data: {}", e);
            }
        }
    }
}

/// Builder for UltrasonicNode with custom processor
pub struct UltrasonicNodeBuilder<D, P>
where
    D: Sensor<Output = Range>,
    P: Processor<Range>,
{
    topic: String,
    driver: Option<D>,
    backend: UltrasonicBackend,
    processor: P,
}

impl UltrasonicNodeBuilder<UltrasonicDriver, PassThrough<Range>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "ultrasonic.range".to_string(),
            driver: None,
            backend: UltrasonicBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for UltrasonicNodeBuilder<UltrasonicDriver, PassThrough<Range>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D, P> UltrasonicNodeBuilder<D, P>
where
    D: Sensor<Output = Range>,
    P: Processor<Range>,
{
    /// Set the topic for publishing range data
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set the ultrasonic backend (creates appropriate driver on build)
    pub fn backend(mut self, backend: UltrasonicBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Alias for backend
    pub fn with_backend(self, backend: UltrasonicBackend) -> Self {
        self.backend(backend)
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> UltrasonicNodeBuilder<D, P2>
    where
        P2: Processor<Range>,
    {
        UltrasonicNodeBuilder {
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
    ) -> UltrasonicNodeBuilder<D, ClosureProcessor<Range, Range, F>>
    where
        F: FnMut(Range) -> Range + Send + 'static,
    {
        UltrasonicNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> UltrasonicNodeBuilder<D, FilterProcessor<Range, Range, F>>
    where
        F: FnMut(Range) -> Option<Range> + Send + 'static,
    {
        UltrasonicNodeBuilder {
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
    ) -> UltrasonicNodeBuilder<D, Pipeline<Range, Range, Range, P, P2>>
    where
        P2: Processor<Range, Output = Range>,
        P: Processor<Range, Output = Range>,
    {
        UltrasonicNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }
}

impl<P> UltrasonicNodeBuilder<UltrasonicDriver, P>
where
    P: Processor<Range>,
{
    /// Build the node with UltrasonicDriver (default driver type)
    pub fn build(self) -> Result<UltrasonicNode<UltrasonicDriver, P>> {
        let driver_backend: UltrasonicDriverBackend = self.backend.into();
        let driver = UltrasonicDriver::new(driver_backend)?;

        Ok(UltrasonicNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            processor: self.processor,
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
        })
    }
}

// Builder for custom drivers
impl<D, P> UltrasonicNodeBuilder<D, P>
where
    D: Sensor<Output = Range>,
    P: Processor<Range>,
{
    /// Set a custom driver
    pub fn with_driver<D2>(self, driver: D2) -> UltrasonicNodeBuilder<D2, P>
    where
        D2: Sensor<Output = Range>,
    {
        UltrasonicNodeBuilder {
            topic: self.topic,
            driver: Some(driver),
            backend: self.backend,
            processor: self.processor,
        }
    }

    /// Build the node with a custom driver (requires driver to be set)
    pub fn build_with_driver(self) -> Result<UltrasonicNode<D, P>>
    where
        D: Default,
    {
        let driver = self.driver.unwrap_or_default();

        Ok(UltrasonicNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            processor: self.processor,
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
        })
    }
}

// Convenience type aliases for common driver types
/// UltrasonicNode with SimulationUltrasonicDriver
pub type SimulationUltrasonicNode<P = PassThrough<Range>> =
    UltrasonicNode<SimulationUltrasonicDriver, P>;

#[cfg(feature = "gpio-hardware")]
/// UltrasonicNode with GpioUltrasonicDriver
pub type GpioUltrasonicNode<P = PassThrough<Range>> = UltrasonicNode<GpioUltrasonicDriver, P>;
