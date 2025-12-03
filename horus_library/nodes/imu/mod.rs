use crate::Imu;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(any(feature = "mpu6050-imu", feature = "bno055-imu"))]
use linux_embedded_hal::{Delay, I2cdev};

#[cfg(feature = "mpu6050-imu")]
use mpu6050::Mpu6050;

#[cfg(feature = "bno055-imu")]
use bno055::{BNO055OperationMode, Bno055};

/// IMU backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImuBackend {
    Simulation,
    Mpu6050,
    Bno055,
    Icm20948,
}

/// IMU Node - Inertial Measurement Unit for orientation sensing
///
/// Reads accelerometer, gyroscope, and magnetometer data from IMU sensors
/// and publishes Imu messages with orientation and motion information.
///
/// Supports multiple hardware backends:
/// - MPU6050 (6-axis: accel + gyro)
/// - BNO055 (9-axis: accel + gyro + mag with sensor fusion)
/// - ICM20948 (9-axis: accel + gyro + mag)
/// - Simulation mode for testing
///
/// # Hybrid Pattern
///
/// This node supports the hybrid pattern for custom processing:
///
/// ```rust,ignore
/// let node = ImuNode::builder()
///     .with_filter(|imu| {
///         // Filter out noisy readings
///         if imu.linear_acceleration[2].abs() > 0.5 {
///             Some(imu)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct ImuNode<P = PassThrough<Imu>>
where
    P: Processor<Imu>,
{
    publisher: Hub<Imu>,

    // Configuration
    frame_id: String,
    sample_rate: f32,
    backend: ImuBackend,
    i2c_bus: String,
    i2c_address: u8,

    // State
    is_initialized: bool,
    sample_count: u64,
    last_sample_time: u64,

    // Hardware drivers
    #[cfg(feature = "mpu6050-imu")]
    mpu6050: Option<Mpu6050<I2cdev>>,

    #[cfg(feature = "bno055-imu")]
    bno055: Option<Bno055<I2cdev>>,

    // Simulation state for synthetic data
    sim_angle: f32,

    // Processor for hybrid pattern
    processor: P,
}

impl ImuNode {
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
        Ok(Self {
            publisher: Hub::new(topic)?,
            frame_id: "imu_link".to_string(),
            sample_rate: 100.0, // 100 Hz default
            backend,
            i2c_bus: "/dev/i2c-1".to_string(), // Default for Raspberry Pi
            i2c_address: 0x68,                 // Default MPU6050 address
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            #[cfg(feature = "mpu6050-imu")]
            mpu6050: None,
            #[cfg(feature = "bno055-imu")]
            bno055: None,
            sim_angle: 0.0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for custom configuration
    pub fn builder() -> ImuNodeBuilder<PassThrough<Imu>> {
        ImuNodeBuilder::new()
    }
}

impl<P> ImuNode<P>
where
    P: Processor<Imu>,
{
    /// Set hardware backend
    pub fn set_backend(&mut self, backend: ImuBackend) {
        self.backend = backend;
        self.is_initialized = false; // Need to reinitialize
    }

    /// Set I2C bus and address for hardware IMU
    pub fn set_i2c_config(&mut self, bus: &str, address: u8) {
        self.i2c_bus = bus.to_string();
        self.i2c_address = address;
        self.is_initialized = false;
    }

    /// Set frame ID for coordinate system
    pub fn set_frame_id(&mut self, frame_id: &str) {
        self.frame_id = frame_id.to_string();
    }

    /// Set IMU sample rate (Hz)
    pub fn set_sample_rate(&mut self, rate: f32) {
        self.sample_rate = rate.clamp(1.0, 1000.0);
    }

    /// Get actual sample rate (samples per second)
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

    fn initialize_imu(&mut self) -> bool {
        if self.is_initialized {
            return true;
        }

        match self.backend {
            ImuBackend::Simulation => {
                // Simulation mode requires no hardware initialization
                self.is_initialized = true;
                true
            }
            #[cfg(feature = "mpu6050-imu")]
            ImuBackend::Mpu6050 => {
                use std::thread;
                use std::time::Duration;

                match I2cdev::new(&self.i2c_bus) {
                    Ok(i2c) => {
                        match Mpu6050::new(i2c) {
                            Ok(mut mpu) => {
                                // Initialize the MPU6050
                                if mpu.init().is_ok() {
                                    // Small delay for sensor stabilization
                                    thread::sleep(Duration::from_millis(100));
                                    self.mpu6050 = Some(mpu);
                                    self.is_initialized = true;
                                    true
                                } else {
                                    eprintln!("Failed to initialize MPU6050");
                                    false
                                }
                            }
                            Err(e) => {
                                eprintln!("Failed to create MPU6050: {:?}", e);
                                false
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Failed to open I2C bus {}: {:?}", self.i2c_bus, e);
                        false
                    }
                }
            }
            #[cfg(feature = "bno055-imu")]
            ImuBackend::Bno055 => {
                use std::thread;
                use std::time::Duration;

                match I2cdev::new(&self.i2c_bus) {
                    Ok(i2c) => {
                        // Bno055::new returns Bno055 directly (not a Result)
                        let mut bno = Bno055::new(i2c);
                        let mut delay = Delay;
                        // Initialize BNO055 in NDOF mode (full sensor fusion)
                        if bno.init(&mut delay).is_ok()
                            && bno.set_mode(BNO055OperationMode::NDOF, &mut delay).is_ok()
                        {
                            thread::sleep(Duration::from_millis(100));
                            self.bno055 = Some(bno);
                            self.is_initialized = true;
                            true
                        } else {
                            eprintln!("Failed to initialize BNO055");
                            false
                        }
                    }
                    Err(e) => {
                        eprintln!("Failed to open I2C bus {}: {:?}", self.i2c_bus, e);
                        false
                    }
                }
            }
            _ => {
                eprintln!("Unsupported IMU backend: {:?}", self.backend);
                false
            }
        }
    }

    fn read_imu_data(&mut self) -> Option<Imu> {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        match self.backend {
            ImuBackend::Simulation => {
                // Generate synthetic IMU data for testing
                self.sim_angle += 0.01; // Slow rotation

                let mut imu = Imu::new();
                imu.orientation = [
                    0.0,
                    0.0,
                    self.sim_angle.cos() as f64,
                    self.sim_angle.sin() as f64,
                ];
                imu.angular_velocity = [0.01, 0.0, 0.0];
                imu.linear_acceleration = [0.0, 0.0, -9.81];
                imu.timestamp = current_time;
                Some(imu)
            }
            #[cfg(feature = "mpu6050-imu")]
            ImuBackend::Mpu6050 => {
                if let Some(ref mut mpu) = self.mpu6050 {
                    // Read accelerometer and gyroscope data
                    match (mpu.get_acc(), mpu.get_gyro()) {
                        (Ok(acc), Ok(gyro)) => {
                            let mut imu = Imu::new();

                            // MPU6050 provides raw accel/gyro, no orientation
                            // In m/s^2 (MPU returns g-force, convert to m/s^2)
                            imu.linear_acceleration = [
                                acc.x as f64 * 9.81,
                                acc.y as f64 * 9.81,
                                acc.z as f64 * 9.81,
                            ];

                            // In rad/s (MPU returns deg/s, convert to rad/s)
                            imu.angular_velocity = [
                                gyro.x as f64 * 0.017453292519943295,
                                gyro.y as f64 * 0.017453292519943295,
                                gyro.z as f64 * 0.017453292519943295,
                            ];

                            // MPU6050 doesn't provide orientation - would need complementary filter
                            imu.orientation = [0.0, 0.0, 0.0, 1.0]; // Identity quaternion
                            imu.timestamp = current_time;
                            Some(imu)
                        }
                        _ => {
                            eprintln!("Failed to read MPU6050 data");
                            None
                        }
                    }
                } else {
                    None
                }
            }
            #[cfg(feature = "bno055-imu")]
            ImuBackend::Bno055 => {
                if let Some(ref mut bno) = self.bno055 {
                    // BNO055 provides fused orientation as quaternion
                    let mut imu = Imu::new();

                    if let Ok(quat) = bno.quaternion() {
                        // mint::Quaternion has v (Vector3 for x,y,z) and s (scalar for w)
                        imu.orientation = [
                            quat.v.x as f64,
                            quat.v.y as f64,
                            quat.v.z as f64,
                            quat.s as f64,
                        ];
                    }

                    if let Ok(gyro) = bno.gyro_data() {
                        imu.angular_velocity = [gyro.x as f64, gyro.y as f64, gyro.z as f64];
                    }

                    if let Ok(accel) = bno.accel_data() {
                        imu.linear_acceleration = [accel.x as f64, accel.y as f64, accel.z as f64];
                    }

                    imu.timestamp = current_time;
                    Some(imu)
                } else {
                    None
                }
            }
            _ => None,
        }
    }
}

impl<P> Node for ImuNode<P>
where
    P: Processor<Imu>,
{
    fn name(&self) -> &'static str {
        "ImuNode"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("ImuNode shutting down - releasing IMU resources");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Release hardware IMU resources
        #[cfg(feature = "mpu6050-imu")]
        {
            self.mpu6050 = None;
        }

        #[cfg(feature = "bno055-imu")]
        {
            self.bno055 = None;
        }

        self.is_initialized = false;
        ctx.log_info("IMU resources released safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Initialize IMU on first tick
        if !self.is_initialized && !self.initialize_imu() {
            return;
        }

        // Read and publish IMU data (through processor pipeline)
        if let Some(imu_data) = self.read_imu_data() {
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
    }
}

/// Builder for ImuNode with custom processor
pub struct ImuNodeBuilder<P>
where
    P: Processor<Imu>,
{
    topic: String,
    backend: ImuBackend,
    i2c_bus: String,
    i2c_address: u8,
    processor: P,
}

impl ImuNodeBuilder<PassThrough<Imu>> {
    /// Create a new builder with default configuration
    pub fn new() -> Self {
        Self {
            topic: "imu".to_string(),
            backend: ImuBackend::Simulation,
            i2c_bus: "/dev/i2c-1".to_string(),
            i2c_address: 0x68,
            processor: PassThrough::new(),
        }
    }
}

impl<P> ImuNodeBuilder<P>
where
    P: Processor<Imu>,
{
    /// Set topic name
    pub fn with_topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set backend
    pub fn with_backend(mut self, backend: ImuBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set I2C configuration
    pub fn with_i2c(mut self, bus: &str, address: u8) -> Self {
        self.i2c_bus = bus.to_string();
        self.i2c_address = address;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> ImuNodeBuilder<P2>
    where
        P2: Processor<Imu>,
    {
        ImuNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            i2c_bus: self.i2c_bus,
            i2c_address: self.i2c_address,
            processor,
        }
    }

    /// Add a closure-based processor
    pub fn with_closure<F>(self, f: F) -> ImuNodeBuilder<ClosureProcessor<Imu, Imu, F>>
    where
        F: FnMut(Imu) -> Imu + Send + 'static,
    {
        ImuNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            i2c_bus: self.i2c_bus,
            i2c_address: self.i2c_address,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> ImuNodeBuilder<FilterProcessor<Imu, Imu, F>>
    where
        F: FnMut(Imu) -> Option<Imu> + Send + 'static,
    {
        ImuNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            i2c_bus: self.i2c_bus,
            i2c_address: self.i2c_address,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor
    pub fn pipe<P2>(self, next: P2) -> ImuNodeBuilder<Pipeline<Imu, Imu, Imu, P, P2>>
    where
        P2: Processor<Imu, Imu>,
    {
        ImuNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            i2c_bus: self.i2c_bus,
            i2c_address: self.i2c_address,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<ImuNode<P>> {
        Ok(ImuNode {
            publisher: Hub::new(&self.topic)?,
            frame_id: "imu_link".to_string(),
            sample_rate: 100.0,
            backend: self.backend,
            i2c_bus: self.i2c_bus,
            i2c_address: self.i2c_address,
            is_initialized: false,
            sample_count: 0,
            last_sample_time: 0,
            #[cfg(feature = "mpu6050-imu")]
            mpu6050: None,
            #[cfg(feature = "bno055-imu")]
            bno055: None,
            sim_angle: 0.0,
            processor: self.processor,
        })
    }
}
