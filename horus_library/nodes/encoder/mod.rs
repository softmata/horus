use crate::Odometry;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(feature = "gpio-hardware")]
use sysfs_gpio::{Direction, Edge, Pin};

/// Encoder backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EncoderBackend {
    Simulation,
    GpioQuadrature,
}

/// Encoder Node - Wheel/joint position feedback for odometry and control
///
/// Reads encoder data from wheels or joints and publishes position, velocity,
/// and odometry information for robot navigation and control feedback.
///
/// Supported backends:
/// - GPIO Quadrature (interrupt-based quadrature decoding using sysfs_gpio)
/// - Simulation mode for testing
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = EncoderNode::builder()
///     .with_filter(|odom| {
///         // Only publish when moving
///         if odom.twist.linear[0].abs() > 0.01 { Some(odom) } else { None }
///     })
///     .build()?;
/// ```
pub struct EncoderNode<P = PassThrough<Odometry>>
where
    P: Processor<Odometry>,
{
    publisher: Hub<Odometry>,

    // Configuration
    frame_id: String,
    child_frame_id: String,
    encoder_resolution: f64, // pulses per revolution
    wheel_radius: f64,       // wheel radius in meters
    gear_ratio: f64,         // gear ratio
    backend: EncoderBackend,
    gpio_pin_a: u64, // GPIO pin for channel A
    gpio_pin_b: u64, // GPIO pin for channel B

    // State
    last_position: f64, // last encoder position
    last_time: u64,
    velocity: f64,
    total_distance: f64,
    encoder_count: i64, // Raw encoder count

    // Hardware drivers
    #[cfg(feature = "gpio-hardware")]
    pin_a: Option<Pin>,

    #[cfg(feature = "gpio-hardware")]
    pin_b: Option<Pin>,

    #[cfg(feature = "gpio-hardware")]
    last_a: bool,

    #[cfg(feature = "gpio-hardware")]
    last_b: bool,

    // Simulation state
    sim_velocity: f64,
    sim_angular_velocity: f64,

    // Processor for hybrid pattern
    processor: P,
}

impl EncoderNode {
    /// Create a new encoder node with default topic "odom" in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("odom", EncoderBackend::Simulation)
    }

    /// Create a new encoder node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        Self::new_with_backend(topic, EncoderBackend::Simulation)
    }

    /// Create a new encoder node with specific backend
    pub fn new_with_backend(topic: &str, backend: EncoderBackend) -> Result<Self> {
        Ok(Self {
            publisher: Hub::new(topic)?,
            frame_id: "odom".to_string(),
            child_frame_id: "base_link".to_string(),
            encoder_resolution: 1024.0, // 1024 pulses per revolution default
            wheel_radius: 0.1,          // 10cm wheel radius default
            gear_ratio: 1.0,            // Direct drive default
            backend,
            gpio_pin_a: 17, // Default GPIO pins for Raspberry Pi
            gpio_pin_b: 27,
            last_position: 0.0,
            last_time: 0,
            velocity: 0.0,
            total_distance: 0.0,
            encoder_count: 0,
            #[cfg(feature = "gpio-hardware")]
            pin_a: None,
            #[cfg(feature = "gpio-hardware")]
            pin_b: None,
            #[cfg(feature = "gpio-hardware")]
            last_a: false,
            #[cfg(feature = "gpio-hardware")]
            last_b: false,
            sim_velocity: 0.0,
            sim_angular_velocity: 0.0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> EncoderNodeBuilder<PassThrough<Odometry>> {
        EncoderNodeBuilder::new()
    }

    /// Set encoder backend
    pub fn set_backend(&mut self, backend: EncoderBackend) {
        self.backend = backend;
    }

    /// Set GPIO pins for quadrature encoder (channel A and B)
    pub fn set_gpio_pins(&mut self, pin_a: u64, pin_b: u64) {
        self.gpio_pin_a = pin_a;
        self.gpio_pin_b = pin_b;
    }

    /// Set encoder configuration parameters
    pub fn set_encoder_config(&mut self, resolution: f64, wheel_radius: f64, gear_ratio: f64) {
        self.encoder_resolution = resolution;
        self.wheel_radius = wheel_radius;
        self.gear_ratio = gear_ratio;
    }

    /// Set coordinate frame IDs
    pub fn set_frame_ids(&mut self, frame_id: &str, child_frame_id: &str) {
        self.frame_id = frame_id.to_string();
        self.child_frame_id = child_frame_id.to_string();
    }

    /// Get current velocity
    pub fn get_velocity(&self) -> f64 {
        self.velocity
    }

    /// Get total distance traveled
    pub fn get_total_distance(&self) -> f64 {
        self.total_distance
    }

    /// Get raw encoder count
    pub fn get_encoder_count(&self) -> i64 {
        self.encoder_count
    }

    /// Reset encoder position and distance
    pub fn reset(&mut self) {
        self.last_position = 0.0;
        self.total_distance = 0.0;
        self.velocity = 0.0;
        self.encoder_count = 0;
    }

    /// Initialize encoder hardware
    fn initialize_encoder(&mut self, mut ctx: Option<&mut NodeInfo>) -> bool {
        match self.backend {
            EncoderBackend::Simulation => {
                // Simulation mode requires no hardware initialization
                true
            }
            #[cfg(feature = "gpio-hardware")]
            EncoderBackend::GpioQuadrature => {
                ctx.log_info(&format!(
                    "Initializing GPIO quadrature encoder on pins {} and {}",
                    self.gpio_pin_a, self.gpio_pin_b
                ));

                // Initialize pin A
                let pin_a = Pin::new(self.gpio_pin_a);
                if let Err(e) = pin_a.export() {
                    ctx.log_error(&format!(
                        "Failed to export GPIO pin {}: {:?}",
                        self.gpio_pin_a, e
                    ));
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = EncoderBackend::Simulation;
                    return true;
                }

                if let Err(e) = pin_a.set_direction(Direction::In) {
                    ctx.log_error(&format!(
                        "Failed to set GPIO pin {} direction: {:?}",
                        self.gpio_pin_a, e
                    ));
                    let _ = pin_a.unexport();
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = EncoderBackend::Simulation;
                    return true;
                }

                if let Err(e) = pin_a.set_edge(Edge::BothEdges) {
                    ctx.log_error(&format!(
                        "Failed to set GPIO pin {} edge: {:?}",
                        self.gpio_pin_a, e
                    ));
                    let _ = pin_a.unexport();
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = EncoderBackend::Simulation;
                    return true;
                }

                // Initialize pin B
                let pin_b = Pin::new(self.gpio_pin_b);
                if let Err(e) = pin_b.export() {
                    ctx.log_error(&format!(
                        "Failed to export GPIO pin {}: {:?}",
                        self.gpio_pin_b, e
                    ));
                    let _ = pin_a.unexport();
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = EncoderBackend::Simulation;
                    return true;
                }

                if let Err(e) = pin_b.set_direction(Direction::In) {
                    ctx.log_error(&format!(
                        "Failed to set GPIO pin {} direction: {:?}",
                        self.gpio_pin_b, e
                    ));
                    let _ = pin_a.unexport();
                    let _ = pin_b.unexport();
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = EncoderBackend::Simulation;
                    return true;
                }

                // Read initial states
                self.last_a = pin_a.get_value().unwrap_or(0) != 0;
                self.last_b = pin_b.get_value().unwrap_or(0) != 0;

                self.pin_a = Some(pin_a);
                self.pin_b = Some(pin_b);

                ctx.log_info("GPIO quadrature encoder initialized successfully");
                true
            }
            #[cfg(not(feature = "gpio-hardware"))]
            EncoderBackend::GpioQuadrature => {
                ctx.log_warning("GPIO backend requested but gpio-hardware feature not enabled");
                ctx.log_warning("Falling back to simulation mode");
                self.backend = EncoderBackend::Simulation;
                true
            }
        }
    }

    fn read_encoder_position(&mut self) -> f64 {
        match self.backend {
            EncoderBackend::Simulation => {
                // For simulation, generate synthetic encoder data
                let current_time = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as f64
                    / 1000.0;

                // Simulate encoder position based on synthetic velocity
                current_time * self.sim_velocity
            }
            #[cfg(feature = "gpio-hardware")]
            EncoderBackend::GpioQuadrature => {
                // Read current pin states
                if let (Some(ref pin_a), Some(ref pin_b)) = (&self.pin_a, &self.pin_b) {
                    let a = pin_a.get_value().unwrap_or(0) != 0;
                    let b = pin_b.get_value().unwrap_or(0) != 0;

                    // Quadrature decoding logic
                    // Standard Gray code quadrature state machine
                    let state_change = (self.last_a != a) || (self.last_b != b);

                    if state_change {
                        // Determine direction
                        let forward = (self.last_a && !self.last_b && !a && !b)
                            || (!self.last_a && !self.last_b && !a && b)
                            || (!self.last_a && b && a && b)
                            || (a && b && a && !b);

                        let backward = (!self.last_a && !self.last_b && a && !b)
                            || (a && !self.last_b && a && b)
                            || (a && b && !a && b)
                            || (!a && b && !a && !b);

                        if forward {
                            self.encoder_count += 1;
                        } else if backward {
                            self.encoder_count -= 1;
                        }

                        self.last_a = a;
                        self.last_b = b;
                    }

                    // Convert encoder count to position in meters
                    let revolutions = self.encoder_count as f64 / (self.encoder_resolution * 4.0); // 4x for quadrature
                    let wheel_revolutions = revolutions / self.gear_ratio;
                    wheel_revolutions * 2.0 * std::f64::consts::PI * self.wheel_radius
                } else {
                    0.0
                }
            }
            #[cfg(not(feature = "gpio-hardware"))]
            EncoderBackend::GpioQuadrature => 0.0,
        }
    }

    fn calculate_velocity(&mut self, current_position: f64, dt: f64) -> f64 {
        if dt > 0.0 {
            let position_delta = current_position - self.last_position;
            self.velocity = position_delta / dt;
            self.total_distance += position_delta.abs();
        } else {
            self.velocity = 0.0;
        }

        self.last_position = current_position;
        self.velocity
    }

    fn publish_odometry(&self, linear_velocity: f64, angular_velocity: f64) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Create odometry message (simplified - real implementation would calculate pose)
        let mut odom = Odometry::new();

        // Set frame information
        odom.frame_id = self
            .frame_id
            .clone()
            .into_bytes()
            .try_into()
            .unwrap_or([0; 32]);
        odom.child_frame_id = self
            .child_frame_id
            .clone()
            .into_bytes()
            .try_into()
            .unwrap_or([0; 32]);

        // Set velocities
        odom.twist.linear[0] = linear_velocity;
        odom.twist.angular[2] = angular_velocity;

        // Set timestamp
        odom.timestamp = current_time;

        let _ = self.publisher.send(odom, &mut None);
    }

    /// Set simulation velocities (for testing without hardware)
    pub fn set_simulation_velocity(&mut self, linear: f64, angular: f64) {
        self.sim_velocity = linear;
        self.sim_angular_velocity = angular;
    }
}

impl<P> Node for EncoderNode<P>
where
    P: Processor<Odometry>,
{
    fn name(&self) -> &'static str {
        "EncoderNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("Encoder node initialized");

        match self.backend {
            EncoderBackend::Simulation => {
                ctx.log_info("Encoder simulation mode enabled");
            }
            EncoderBackend::GpioQuadrature => {
                ctx.log_info(&format!(
                    "GPIO quadrature encoder: pins {} and {}",
                    self.gpio_pin_a, self.gpio_pin_b
                ));
            }
        }

        // Initialize hardware
        if !self.initialize_encoder(Some(ctx)) {
            ctx.log_error("Failed to initialize encoder hardware");
        }

        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Calculate delta time
        let dt = if self.last_time > 0 {
            (current_time - self.last_time) as f64 / 1000.0
        } else {
            0.01 // 10ms default
        };
        self.last_time = current_time;

        // Read encoder position and calculate velocity
        let current_position = self.read_encoder_position();
        let linear_velocity = self.calculate_velocity(current_position, dt);

        // Build odometry message
        let odom_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let mut odom = Odometry::new();
        odom.frame_id = self
            .frame_id
            .clone()
            .into_bytes()
            .try_into()
            .unwrap_or([0; 32]);
        odom.child_frame_id = self
            .child_frame_id
            .clone()
            .into_bytes()
            .try_into()
            .unwrap_or([0; 32]);
        odom.twist.linear[0] = linear_velocity;
        odom.twist.angular[2] = self.sim_angular_velocity;
        odom.timestamp = odom_time;

        // Process through pipeline and publish
        if let Some(processed) = self.processor.process(odom) {
            let _ = self.publisher.send(processed, &mut None);
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();

        #[cfg(feature = "gpio-hardware")]
        {
            if let Some(ref pin_a) = self.pin_a {
                let _ = pin_a.unexport();
                ctx.log_info(&format!("GPIO pin {} unexported", self.gpio_pin_a));
            }
            if let Some(ref pin_b) = self.pin_b {
                let _ = pin_b.unexport();
                ctx.log_info(&format!("GPIO pin {} unexported", self.gpio_pin_b));
            }
            self.pin_a = None;
            self.pin_b = None;
        }

        Ok(())
    }
}

/// Builder for EncoderNode with fluent API for processor configuration
pub struct EncoderNodeBuilder<P>
where
    P: Processor<Odometry>,
{
    topic: String,
    backend: EncoderBackend,
    processor: P,
}

impl EncoderNodeBuilder<PassThrough<Odometry>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "odom".to_string(),
            backend: EncoderBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl<P> EncoderNodeBuilder<P>
where
    P: Processor<Odometry>,
{
    /// Set the topic for publishing odometry
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set the encoder backend
    pub fn backend(mut self, backend: EncoderBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> EncoderNodeBuilder<P2>
    where
        P2: Processor<Odometry>,
    {
        EncoderNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor,
        }
    }

    /// Add a closure processor for transformations
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> EncoderNodeBuilder<ClosureProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Odometry + Send + 'static,
    {
        EncoderNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> EncoderNodeBuilder<FilterProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Option<Odometry> + Send + 'static,
    {
        EncoderNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> EncoderNodeBuilder<Pipeline<Odometry, Odometry, Odometry, P, P2>>
    where
        P2: Processor<Odometry, Output = Odometry>,
        P: Processor<Odometry, Output = Odometry>,
    {
        EncoderNodeBuilder {
            topic: self.topic,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the EncoderNode
    pub fn build(self) -> Result<EncoderNode<P>> {
        Ok(EncoderNode {
            publisher: Hub::new(&self.topic)?,
            frame_id: "odom".to_string(),
            child_frame_id: "base_link".to_string(),
            encoder_resolution: 1024.0,
            wheel_radius: 0.1,
            gear_ratio: 1.0,
            backend: self.backend,
            gpio_pin_a: 17,
            gpio_pin_b: 27,
            last_position: 0.0,
            last_time: 0,
            velocity: 0.0,
            total_distance: 0.0,
            encoder_count: 0,
            #[cfg(feature = "gpio-hardware")]
            pin_a: None,
            #[cfg(feature = "gpio-hardware")]
            pin_b: None,
            #[cfg(feature = "gpio-hardware")]
            last_a: false,
            #[cfg(feature = "gpio-hardware")]
            last_b: false,
            sim_velocity: 0.0,
            sim_angular_velocity: 0.0,
            processor: self.processor,
        })
    }
}
