//! IMU Node - Inertial Measurement Unit sensor node
//!
//! This node reads data from IMU sensors and publishes Imu messages.
//! It uses the driver abstraction layer to support multiple hardware backends.

use crate::Imu;
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
use crate::drivers::imu::{ImuDriver, ImuDriverBackend, SimulationImuDriver};

#[cfg(feature = "mpu6050-imu")]
use crate::drivers::imu::Mpu6050Driver;

#[cfg(feature = "bno055-imu")]
use crate::drivers::imu::Bno055Driver;

#[cfg(feature = "icm20948-imu")]
use crate::drivers::imu::Icm20948Driver;

/// IMU backend type (deprecated - use ImuDriverBackend instead)
///
/// This enum is kept for backward compatibility. New code should use
/// `ImuDriverBackend` from `crate::drivers::imu`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImuBackend {
    Simulation,
    Mpu6050,
    Bno055,
    Icm20948,
}

impl From<ImuBackend> for ImuDriverBackend {
    fn from(backend: ImuBackend) -> Self {
        match backend {
            ImuBackend::Simulation => ImuDriverBackend::Simulation,
            #[cfg(feature = "mpu6050-imu")]
            ImuBackend::Mpu6050 => ImuDriverBackend::Mpu6050,
            #[cfg(not(feature = "mpu6050-imu"))]
            ImuBackend::Mpu6050 => ImuDriverBackend::Simulation, // Fallback
            #[cfg(feature = "bno055-imu")]
            ImuBackend::Bno055 => ImuDriverBackend::Bno055,
            #[cfg(not(feature = "bno055-imu"))]
            ImuBackend::Bno055 => ImuDriverBackend::Simulation, // Fallback
            #[cfg(feature = "icm20948-imu")]
            ImuBackend::Icm20948 => ImuDriverBackend::Icm20948,
            #[cfg(not(feature = "icm20948-imu"))]
            ImuBackend::Icm20948 => ImuDriverBackend::Simulation, // Fallback
        }
    }
}

/// IMU Node - Inertial Measurement Unit for orientation sensing
///
/// Reads accelerometer, gyroscope, and magnetometer data from IMU sensors
/// and publishes Imu messages with orientation and motion information.
///
/// # Driver System
///
/// This node uses the HORUS driver abstraction layer. Drivers handle all
/// hardware-specific code, while the node handles HORUS integration (topics,
/// scheduling, lifecycle).
///
/// ## Supported Drivers
///
/// - `SimulationImuDriver` - Always available, generates synthetic data
/// - `Mpu6050Driver` - MPU6050 6-axis IMU (requires `mpu6050-imu` feature)
/// - `Bno055Driver` - BNO055 9-axis IMU with fusion (requires `bno055-imu` feature)
///
/// # Example
///
/// ```rust,ignore
/// use horus_library::nodes::ImuNode;
/// use horus_library::drivers::SimulationImuDriver;
///
/// // Using the default simulation driver
/// let node = ImuNode::new()?;
///
/// // Using a specific driver
/// let driver = SimulationImuDriver::new();
/// let node = ImuNode::with_driver("imu", driver)?;
///
/// // Using the builder for custom configuration
/// let node = ImuNode::builder()
///     .with_topic("custom_imu")
///     .with_backend(ImuBackend::Simulation)
///     .with_filter(|imu| {
///         // Filter out readings with low acceleration
///         if imu.linear_acceleration[2].abs() > 0.5 {
///             Some(imu)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct ImuNode<D = ImuDriver, P = PassThrough<Imu>>
where
    D: Sensor<Output = Imu>,
    P: Processor<Imu>,
{
    publisher: Topic<Imu>,

    // Driver (handles hardware abstraction)
    driver: D,

    // Configuration
    frame_id: String,

    // State
    is_initialized: bool,
    sample_count: u64,
    last_sample_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl ImuNode<ImuDriver, PassThrough<Imu>> {
    /// Create a new IMU node with default topic "imu" in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("imu", ImuBackend::Simulation)
    }

    /// Create a new IMU node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        Self::new_with_backend(topic, ImuBackend::Simulation)
    }

    /// Create a new IMU node with specific hardware backend
    pub fn new_with_backend(topic: &str, backend: ImuBackend) -> Result<Self> {
        let driver_backend: ImuDriverBackend = backend.into();
        let driver = ImuDriver::new(driver_backend)?;

        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            frame_id: "imu_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for custom configuration
    pub fn builder() -> ImuNodeBuilder<ImuDriver, PassThrough<Imu>> {
        ImuNodeBuilder::new()
    }
}

impl<D> ImuNode<D, PassThrough<Imu>>
where
    D: Sensor<Output = Imu>,
{
    /// Create a new IMU node with a custom driver
    ///
    /// This allows using any driver that implements `Sensor<Output = Imu>`,
    /// including custom drivers from the marketplace.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_library::nodes::ImuNode;
    /// use horus_library::drivers::SimulationImuDriver;
    ///
    /// let driver = SimulationImuDriver::new();
    /// let node = ImuNode::with_driver("imu", driver)?;
    /// ```
    pub fn with_driver(topic: &str, driver: D) -> Result<Self> {
        Ok(Self {
            publisher: Topic::new(topic)?,
            driver,
            frame_id: "imu_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: PassThrough::new(),
        })
    }
}

impl<D, P> ImuNode<D, P>
where
    D: Sensor<Output = Imu>,
    P: Processor<Imu>,
{
    /// Set frame ID for coordinate system
    pub fn set_frame_id(&mut self, frame_id: &str) {
        self.frame_id = frame_id.to_string();
    }

    /// Get the driver's sample rate (if available)
    pub fn get_sample_rate(&self) -> Option<f32> {
        self.driver.sample_rate()
    }

    /// Get actual sample rate based on timestamps
    pub fn get_actual_sample_rate(&self) -> f32 {
        if self.sample_count < 2 {
            return 0.0;
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        let time_diff = current_time - self.last_sample_time;
        if time_diff > 0 {
            1000.0 / time_diff as f32
        } else {
            0.0
        }
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

impl<D, P> Node for ImuNode<D, P>
where
    D: Sensor<Output = Imu>,
    P: Processor<Imu>,
{
    fn name(&self) -> &'static str {
        "ImuNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Initialize the driver
        self.driver.init()?;
        self.is_initialized = true;

        // Initialize processor
        self.processor.on_start();

        ctx.log_info(&format!(
            "ImuNode initialized with driver: {} ({})",
            self.driver.name(),
            self.driver.id()
        ));

        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("ImuNode shutting down - releasing IMU resources");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Shutdown driver
        self.driver.shutdown()?;

        self.is_initialized = false;
        ctx.log_info("IMU resources released safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Check if driver has data available
        if !self.driver.has_data() {
            return;
        }

        // Read and publish IMU data (through processor pipeline)
        match self.driver.read() {
            Ok(imu_data) => {
                self.sample_count += 1;
                self.last_sample_time = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                // Process through pipeline (filter/transform)
                if let Some(processed) = self.processor.process(imu_data) {
                    let _ = self.publisher.send(processed, &mut None);
                }
            }
            Err(e) => {
                // Log error but continue - sensor might recover
                eprintln!("ImuNode: Failed to read data: {}", e);
            }
        }
    }
}

/// Builder for ImuNode with custom processor
pub struct ImuNodeBuilder<D, P>
where
    D: Sensor<Output = Imu>,
    P: Processor<Imu>,
{
    topic: String,
    driver: Option<D>,
    backend: ImuBackend,
    processor: P,
}

impl ImuNodeBuilder<ImuDriver, PassThrough<Imu>> {
    /// Create a new builder with default configuration
    pub fn new() -> Self {
        Self {
            topic: "imu".to_string(),
            driver: None,
            backend: ImuBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for ImuNodeBuilder<ImuDriver, PassThrough<Imu>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D, P> ImuNodeBuilder<D, P>
where
    D: Sensor<Output = Imu>,
    P: Processor<Imu>,
{
    /// Set topic name
    pub fn with_topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set backend (creates appropriate driver on build)
    pub fn with_backend(mut self, backend: ImuBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> ImuNodeBuilder<D, P2>
    where
        P2: Processor<Imu>,
    {
        ImuNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor,
        }
    }

    /// Add a closure-based processor
    pub fn with_closure<F>(self, f: F) -> ImuNodeBuilder<D, ClosureProcessor<Imu, Imu, F>>
    where
        F: FnMut(Imu) -> Imu + Send + 'static,
    {
        ImuNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> ImuNodeBuilder<D, FilterProcessor<Imu, Imu, F>>
    where
        F: FnMut(Imu) -> Option<Imu> + Send + 'static,
    {
        ImuNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor
    pub fn pipe<P2>(self, next: P2) -> ImuNodeBuilder<D, Pipeline<Imu, Imu, Imu, P, P2>>
    where
        P2: Processor<Imu, Imu>,
    {
        ImuNodeBuilder {
            topic: self.topic,
            driver: self.driver,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }
}

impl<P> ImuNodeBuilder<ImuDriver, P>
where
    P: Processor<Imu>,
{
    /// Build the node with ImuDriver (default driver type)
    pub fn build(self) -> Result<ImuNode<ImuDriver, P>> {
        let driver_backend: ImuDriverBackend = self.backend.into();
        let driver = ImuDriver::new(driver_backend)?;

        Ok(ImuNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            frame_id: "imu_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: self.processor,
        })
    }
}

// Builder for custom drivers
impl<D, P> ImuNodeBuilder<D, P>
where
    D: Sensor<Output = Imu>,
    P: Processor<Imu>,
{
    /// Set a custom driver
    pub fn with_driver<D2>(self, driver: D2) -> ImuNodeBuilder<D2, P>
    where
        D2: Sensor<Output = Imu>,
    {
        ImuNodeBuilder {
            topic: self.topic,
            driver: Some(driver),
            backend: self.backend,
            processor: self.processor,
        }
    }

    /// Build the node with a custom driver (requires driver to be set)
    pub fn build_with_driver(self) -> Result<ImuNode<D, P>>
    where
        D: Default,
    {
        let driver = self.driver.unwrap_or_default();

        Ok(ImuNode {
            publisher: Topic::new(&self.topic)?,
            driver,
            frame_id: "imu_link".to_string(),
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            processor: self.processor,
        })
    }
}

// Convenience type aliases for common driver types
/// ImuNode with SimulationImuDriver
pub type SimulationImuNode<P = PassThrough<Imu>> = ImuNode<SimulationImuDriver, P>;

#[cfg(feature = "mpu6050-imu")]
/// ImuNode with Mpu6050Driver
pub type Mpu6050ImuNode<P = PassThrough<Imu>> = ImuNode<Mpu6050Driver, P>;

#[cfg(feature = "bno055-imu")]
/// ImuNode with Bno055Driver
pub type Bno055ImuNode<P = PassThrough<Imu>> = ImuNode<Bno055Driver, P>;

#[cfg(feature = "icm20948-imu")]
/// ImuNode with Icm20948Driver
pub type Icm20948ImuNode<P = PassThrough<Imu>> = ImuNode<Icm20948Driver, P>;
