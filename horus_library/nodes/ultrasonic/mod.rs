use crate::Range;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

// GPIO hardware support
#[cfg(feature = "gpio-hardware")]
use std::thread;
#[cfg(feature = "gpio-hardware")]
use std::time::Duration;
#[cfg(feature = "gpio-hardware")]
use sysfs_gpio::{Direction, Pin};

/// Ultrasonic Distance Sensor Node
///
/// Reads distance measurements from ultrasonic sensors using echo/trigger interface.
/// Compatible with common ultrasonic distance sensors.
///
/// # Supported Sensors
/// - HC-SR04: 2cm-400cm range, 15° beam angle, 5V operation
/// - HC-SR04+: 2cm-400cm range, 15° beam angle, 3.3V/5V operation
/// - US-100: 2cm-450cm range, 15° beam angle, UART or echo/trigger mode
/// - JSN-SR04T: 20cm-600cm range, waterproof, 5V operation
/// - Maxbotix MB1xxx series: Various ranges, analog/PWM/serial output
/// - DYP-A01: 2cm-450cm range, UART output
///
/// # Hardware Interface
/// - Trigger Pin: Send 10μs HIGH pulse to initiate measurement
/// - Echo Pin: Receives HIGH pulse with duration proportional to distance
/// - Distance = (Echo Duration × Speed of Sound) / 2
/// - Speed of Sound: ~343 m/s at 20°C (adjustable for temperature)
///
/// # Features
/// - Multiple sensor support (up to 16 sensors)
/// - Configurable measurement rate (1-100 Hz)
/// - Median filtering for noise reduction
/// - Temperature compensation
/// - Out-of-range detection
/// - Sensor health monitoring
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = UltrasonicNode::builder()
///     .with_filter(|range| {
///         // Only publish valid ranges
///         if range.range > 0.0 && range.range < 4.0 { Some(range) } else { None }
///     })
///     .build()?;
/// ```
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::UltrasonicNode;
///
/// let mut ultrasonic = UltrasonicNode::new()?;
/// ultrasonic.set_num_sensors(4);
/// ultrasonic.set_measurement_rate(20.0); // 20 Hz
/// ultrasonic.set_temperature(25.0); // 25°C
/// ultrasonic.enable_median_filter(true);
/// ```
pub struct UltrasonicNode<P = PassThrough<Range>>
where
    P: Processor<Range>,
{
    publisher: Hub<Range>,

    // Processor for hybrid pattern
    processor: P,

    // Configuration
    num_sensors: u8,
    sensor_names: [[u8; 32]; 16], // Sensor names for multi-sensor systems
    measurement_rate: f32,        // Hz
    speed_of_sound: f32,          // m/s (depends on temperature)
    temperature_celsius: f32,     // For speed of sound calculation
    min_range: [f32; 16],         // Minimum valid range per sensor (m)
    max_range: [f32; 16],         // Maximum valid range per sensor (m)
    field_of_view: [f32; 16],     // Beam angle in radians
    enable_median_filter: bool,
    median_filter_size: usize,

    // State tracking per sensor
    last_measurement_time: [u64; 16],  // Nanoseconds
    current_range: [f32; 16],          // Current filtered range (m)
    raw_range_history: [[f32; 5]; 16], // For median filtering
    history_index: [usize; 16],
    measurement_count: [u64; 16],
    error_count: [u64; 16],
    trigger_sent: [bool; 16],
    echo_start_time: [u64; 16],
    echo_end_time: [u64; 16],

    // Timing
    trigger_duration_ns: u64, // Trigger pulse duration (10μs default)
    max_echo_time_ns: u64,    // Maximum echo wait time (38ms for 4m range)
    measurement_interval_ns: u64,
    last_trigger_time: u64,

    // Hardware GPIO pins (per sensor)
    #[cfg(feature = "gpio-hardware")]
    trigger_pins: [Option<Pin>; 16],
    #[cfg(feature = "gpio-hardware")]
    echo_pins: [Option<Pin>; 16],
    hardware_enabled: bool,
    gpio_pin_numbers: [(u64, u64); 16], // (trigger_pin, echo_pin) numbers per sensor

    // Timing state (moved from static mut for thread safety)
    last_health_check: u64,
}

impl UltrasonicNode {
    /// Create a new ultrasonic sensor node with default topic "ultrasonic.range"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("ultrasonic.range")
    }

    /// Create a new ultrasonic sensor node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        #[cfg(feature = "gpio-hardware")]
        const NONE_PIN: Option<Pin> = None;
        let mut node = Self {
            publisher: Hub::new(topic)?,
            num_sensors: 1,
            sensor_names: [[0; 32]; 16],
            measurement_rate: 10.0, // 10 Hz default
            speed_of_sound: 343.0,  // m/s at 20°C
            temperature_celsius: 20.0,
            min_range: [0.02; 16],     // 2cm
            max_range: [4.0; 16],      // 4m (HC-SR04 typical)
            field_of_view: [0.26; 16], // ~15 degrees in radians
            enable_median_filter: true,
            median_filter_size: 5,
            last_measurement_time: [0; 16],
            current_range: [0.0; 16],
            raw_range_history: [[0.0; 5]; 16],
            history_index: [0; 16],
            measurement_count: [0; 16],
            error_count: [0; 16],
            trigger_sent: [false; 16],
            echo_start_time: [0; 16],
            echo_end_time: [0; 16],
            trigger_duration_ns: 10_000,          // 10 microseconds
            max_echo_time_ns: 38_000_000,         // 38 milliseconds
            measurement_interval_ns: 100_000_000, // 100ms = 10Hz
            last_trigger_time: 0,
            #[cfg(feature = "gpio-hardware")]
            trigger_pins: [NONE_PIN; 16],
            #[cfg(feature = "gpio-hardware")]
            echo_pins: [NONE_PIN; 16],
            hardware_enabled: false,
            gpio_pin_numbers: [(0, 0); 16],
            last_health_check: 0,
            processor: PassThrough::new(),
        };

        // Set default sensor names
        node.set_sensor_name(0, "ultrasonic_0");
        node.update_measurement_interval();
        node.update_speed_of_sound();

        Ok(node)
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> UltrasonicNodeBuilder<PassThrough<Range>> {
        UltrasonicNodeBuilder::new()
    }

    /// Set the number of ultrasonic sensors (1-16)
    pub fn set_num_sensors(&mut self, num: u8) {
        self.num_sensors = num.clamp(1, 16);
    }

    /// Set sensor name for identification
    pub fn set_sensor_name(&mut self, sensor_id: u8, name: &str) {
        if sensor_id < 16 {
            let idx = sensor_id as usize;
            let name_bytes = name.as_bytes();
            let len = name_bytes.len().min(31);
            self.sensor_names[idx][..len].copy_from_slice(&name_bytes[..len]);
            self.sensor_names[idx][len] = 0; // Null terminator
        }
    }

    /// Set measurement rate in Hz (1-100 Hz)
    pub fn set_measurement_rate(&mut self, rate: f32) {
        self.measurement_rate = rate.clamp(1.0, 100.0);
        self.update_measurement_interval();
    }

    /// Set ambient temperature for speed of sound compensation
    pub fn set_temperature(&mut self, celsius: f32) {
        self.temperature_celsius = celsius;
        self.update_speed_of_sound();
    }

    /// Set range limits for a specific sensor
    pub fn set_range_limits(&mut self, sensor_id: u8, min_range: f32, max_range: f32) {
        if sensor_id < 16 {
            let idx = sensor_id as usize;
            self.min_range[idx] = min_range;
            self.max_range[idx] = max_range;
        }
    }

    /// Set field of view (beam angle) for a sensor in degrees
    pub fn set_field_of_view_degrees(&mut self, sensor_id: u8, degrees: f32) {
        if sensor_id < 16 {
            self.field_of_view[sensor_id as usize] = degrees.to_radians();
        }
    }

    /// Enable/disable median filtering
    pub fn enable_median_filter(&mut self, enable: bool) {
        self.enable_median_filter = enable;
    }

    /// Set median filter size (3, 5, or 7 samples)
    pub fn set_median_filter_size(&mut self, size: usize) {
        self.median_filter_size = match size {
            3 | 5 | 7 => size,
            _ => 5, // Default to 5
        };
    }

    /// Get current range reading for a sensor
    pub fn get_range(&self, sensor_id: u8) -> Option<f32> {
        if sensor_id < self.num_sensors {
            Some(self.current_range[sensor_id as usize])
        } else {
            None
        }
    }

    /// Get measurement statistics for a sensor
    pub fn get_statistics(&self, sensor_id: u8) -> Option<(u64, u64, f32)> {
        if sensor_id < 16 {
            let idx = sensor_id as usize;
            let total = self.measurement_count[idx];
            let errors = self.error_count[idx];
            let error_rate = if total > 0 {
                (errors as f32 / total as f32) * 100.0
            } else {
                0.0
            };
            Some((total, errors, error_rate))
        } else {
            None
        }
    }

    /// Update measurement interval based on rate
    fn update_measurement_interval(&mut self) {
        self.measurement_interval_ns = (1_000_000_000.0 / self.measurement_rate) as u64;
    }

    /// Update speed of sound based on temperature
    /// Formula: v = 331.3 + 0.606 * T (where T is in Celsius)
    fn update_speed_of_sound(&mut self) {
        self.speed_of_sound = 331.3 + 0.606 * self.temperature_celsius;
    }

    /// Calculate distance from echo duration
    /// Distance = (Echo Duration × Speed of Sound) / 2
    fn calculate_distance(&self, echo_duration_ns: u64) -> f32 {
        let echo_duration_s = echo_duration_ns as f32 / 1_000_000_000.0;
        let distance = (echo_duration_s * self.speed_of_sound) / 2.0;
        distance
    }

    /// Apply median filter to range reading
    fn apply_median_filter(&self, sensor_id: u8) -> f32 {
        if !self.enable_median_filter {
            return self.raw_range_history[sensor_id as usize][0];
        }

        let idx = sensor_id as usize;
        let mut sorted = self.raw_range_history[idx].clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Return median value
        sorted[self.median_filter_size / 2]
    }

    /// Configure GPIO pins for a specific sensor
    pub fn set_gpio_pins(&mut self, sensor_id: u8, trigger_pin: u64, echo_pin: u64) {
        if sensor_id < 16 {
            self.gpio_pin_numbers[sensor_id as usize] = (trigger_pin, echo_pin);
        }
    }

    // ========== Hardware Backend Functions ==========

    /// Initialize GPIO pins for a sensor
    #[cfg(feature = "gpio-hardware")]
    fn init_gpio_hardware(
        &mut self,
        sensor_id: u8,
        mut ctx: Option<&mut NodeInfo>,
    ) -> std::io::Result<()> {
        let idx = sensor_id as usize;
        let (trigger_num, echo_num) = self.gpio_pin_numbers[idx];

        if trigger_num == 0 || echo_num == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("GPIO pins not configured for sensor {}", sensor_id),
            ));
        }

        // Initialize trigger pin (output)
        let trigger = Pin::new(trigger_num);
        trigger.export()?;
        thread::sleep(Duration::from_millis(10)); // Give sysfs time to set up
        trigger.set_direction(Direction::Out)?;
        trigger.set_value(0)?; // Start low

        // Initialize echo pin (input)
        let echo = Pin::new(echo_num);
        echo.export()?;
        thread::sleep(Duration::from_millis(10));
        echo.set_direction(Direction::In)?;

        self.trigger_pins[idx] = Some(trigger);
        self.echo_pins[idx] = Some(echo);

        ctx.log_info(&format!(
            "Initialized GPIO for sensor {}: trigger={}, echo={}",
            sensor_id, trigger_num, echo_num
        ));

        Ok(())
    }

    /// Send trigger pulse and measure echo duration via hardware
    #[cfg(feature = "gpio-hardware")]
    fn measure_hardware(&mut self, sensor_id: u8) -> std::io::Result<Option<f32>> {
        let idx = sensor_id as usize;

        let trigger = self.trigger_pins[idx].as_ref().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::NotConnected,
                "Trigger pin not initialized",
            )
        })?;

        let echo = self.echo_pins[idx].as_ref().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Echo pin not initialized")
        })?;

        // Send 10μs trigger pulse
        trigger.set_value(1)?;
        thread::sleep(Duration::from_micros(10));
        trigger.set_value(0)?;

        // Wait for echo to go HIGH (with timeout)
        let start_wait = SystemTime::now();
        let timeout = Duration::from_millis(50); // 50ms timeout

        loop {
            if echo.get_value()? == 1 {
                break;
            }
            if start_wait.elapsed().unwrap_or(Duration::from_secs(1)) > timeout {
                return Ok(None); // Timeout waiting for echo
            }
            thread::sleep(Duration::from_micros(10));
        }

        // Record echo start time
        let echo_start = SystemTime::now();

        // Wait for echo to go LOW (measure pulse width)
        loop {
            if echo.get_value()? == 0 {
                break;
            }
            if echo_start.elapsed().unwrap_or(Duration::from_secs(1)) > Duration::from_millis(40) {
                return Ok(None); // Echo too long (out of range)
            }
            thread::sleep(Duration::from_micros(10));
        }

        // Calculate distance from echo duration
        let echo_duration = echo_start.elapsed().unwrap_or(Duration::ZERO);
        let echo_duration_ns = echo_duration.as_nanos() as u64;
        let distance = self.calculate_distance(echo_duration_ns);

        Ok(Some(distance))
    }

    /// Measure ultrasonic distance (tries hardware first, falls back to simulation)
    fn simulate_measurement(
        &mut self,
        sensor_id: u8,
        current_time: u64,
        mut ctx: Option<&mut NodeInfo>,
    ) -> Option<f32> {
        let idx = sensor_id as usize;

        // Check if it's time for a new measurement
        if current_time < self.last_measurement_time[idx] + self.measurement_interval_ns {
            return None;
        }

        // Try hardware first, fall back to simulation
        #[cfg(feature = "gpio-hardware")]
        if self.hardware_enabled || self.trigger_pins[idx].is_some() {
            // Initialize GPIO if needed
            if self.trigger_pins[idx].is_none() {
                let (trigger_num, echo_num) = self.gpio_pin_numbers[idx];
                if trigger_num != 0 && echo_num != 0 {
                    if let Err(e) = self.init_gpio_hardware(sensor_id, None) {
                        // Failed to initialize - log detailed error (only once)
                        if self.measurement_count[idx] == 0 {
                            ctx.log_warning(&format!(
                                "UltrasonicNode sensor {}: Hardware unavailable - using SIMULATION mode",
                                sensor_id
                            ));
                            ctx.log_warning(&format!(
                                "  Tried GPIO pins: trigger={}, echo={}",
                                trigger_num, echo_num
                            ));
                            ctx.log_warning(&format!("  Error: {}", e));
                            ctx.log_warning("  Fix:");
                            ctx.log_warning("    1. Install: sudo apt install libraspberrypi-dev");
                            ctx.log_warning(
                                "    2. Enable GPIO: sudo raspi-config -> Interface Options",
                            );
                            ctx.log_warning("    3. Check wiring: Verify GPIO pin connections");
                            ctx.log_warning(
                                "    4. Rebuild with: cargo build --features=\"gpio-hardware\"",
                            );
                        }
                        self.hardware_enabled = false;
                    } else {
                        self.hardware_enabled = true;
                    }
                }
            }

            // Try hardware measurement
            if self.hardware_enabled && self.trigger_pins[idx].is_some() {
                match self.measure_hardware(sensor_id) {
                    Ok(Some(distance)) => {
                        self.last_measurement_time[idx] = current_time;
                        return Some(distance);
                    }
                    Ok(None) => {
                        // Valid attempt, but no echo received (out of range)
                        self.last_measurement_time[idx] = current_time;
                        return None;
                    }
                    Err(e) => {
                        // Hardware error, fall back to simulation (log only once)
                        if self.error_count[idx] == 0 {
                            let (trigger_num, echo_num) = self.gpio_pin_numbers[idx];
                            ctx.log_warning(&format!(
                                "UltrasonicNode sensor {}: Measurement failed - using SIMULATION mode",
                                sensor_id
                            ));
                            ctx.log_warning(&format!(
                                "  GPIO: trigger={}, echo={}",
                                trigger_num, echo_num
                            ));
                            ctx.log_warning(&format!("  Error: {}", e));
                            ctx.log_warning("  Common causes: timeout, poor wiring, interference");
                        }
                        self.hardware_enabled = false;
                        self.error_count[idx] += 1;
                    }
                }
            }
        }

        // Simulation fallback
        // Simulate trigger pulse
        if !self.trigger_sent[idx] {
            self.trigger_sent[idx] = true;
            self.echo_start_time[idx] = current_time + self.trigger_duration_ns;
            return None;
        }

        // Simulate echo reception
        if current_time >= self.echo_start_time[idx] {
            // In simulation, generate a realistic distance reading
            let simulated_distance = 0.5 + (sensor_id as f32 * 0.3); // Varies by sensor
            let echo_duration =
                ((simulated_distance * 2.0) / self.speed_of_sound * 1_000_000_000.0) as u64;

            self.echo_end_time[idx] = self.echo_start_time[idx] + echo_duration;
            self.trigger_sent[idx] = false;
            self.last_measurement_time[idx] = current_time;

            let distance = self.calculate_distance(echo_duration);
            Some(distance)
        } else {
            None
        }
    }

    /// Process a measurement for a sensor
    fn process_measurement(
        &mut self,
        sensor_id: u8,
        distance: f32,
        mut ctx: Option<&mut NodeInfo>,
    ) {
        let idx = sensor_id as usize;

        // Validate range
        let is_valid = distance >= self.min_range[idx]
            && distance <= self.max_range[idx]
            && distance.is_finite();

        if !is_valid {
            self.error_count[idx] += 1;
            ctx.log_debug(&format!(
                "Sensor {}: out of range measurement {:.3}m",
                sensor_id, distance
            ));
            return;
        }

        // Update raw history for median filtering
        self.raw_range_history[idx][self.history_index[idx]] = distance;
        self.history_index[idx] = (self.history_index[idx] + 1) % self.median_filter_size;

        // Apply filtering
        let filtered_distance = self.apply_median_filter(sensor_id);
        self.current_range[idx] = filtered_distance;
        self.measurement_count[idx] += 1;

        // Create and publish Range message
        let range_msg = Range {
            sensor_type: Range::ULTRASONIC,
            field_of_view: self.field_of_view[idx],
            min_range: self.min_range[idx],
            max_range: self.max_range[idx],
            range: filtered_distance,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        };

        // Process through pipeline and publish
        if let Some(processed) = self.processor.process(range_msg) {
            if let Err(e) = self.publisher.send(processed, &mut None) {
                ctx.log_error(&format!("Failed to publish range: {:?}", e));
            }
        } else {
            ctx.log_debug(&format!(
                "Sensor {}: {:.3}m (raw: {:.3}m)",
                sensor_id, filtered_distance, distance
            ));
        }
    }

    /// Get sensor health status
    pub fn is_healthy(&self, sensor_id: u8) -> bool {
        if sensor_id >= 16 {
            return false;
        }

        let idx = sensor_id as usize;
        let total = self.measurement_count[idx];
        let errors = self.error_count[idx];

        if total == 0 {
            return true; // Not enough data yet
        }

        let error_rate = (errors as f32 / total as f32) * 100.0;
        error_rate < 20.0 // Less than 20% error rate = healthy
    }
}

impl<P> Node for UltrasonicNode<P>
where
    P: Processor<Range>,
{
    fn name(&self) -> &'static str {
        "UltrasonicNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("UltrasonicNode initialized");
        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        ctx.log_info("UltrasonicNode shutting down - releasing GPIO resources");

        // Release GPIO pins
        #[cfg(feature = "gpio-hardware")]
        {
            for i in 0..16 {
                // Unexport GPIO pins if they were exported
                if let Some(ref trigger_pin) = self.trigger_pins[i] {
                    let _ = trigger_pin.set_value(0); // Set trigger low
                    let _ = trigger_pin.unexport();
                }
                if let Some(ref echo_pin) = self.echo_pins[i] {
                    let _ = echo_pin.unexport();
                }
                self.trigger_pins[i] = None;
                self.echo_pins[i] = None;
            }
        }

        self.hardware_enabled = false;
        ctx.log_info("Ultrasonic sensor resources released safely");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Process measurements for all sensors
        for sensor_id in 0..self.num_sensors {
            // Simulate/read measurement from hardware
            if let Some(distance) =
                self.simulate_measurement(sensor_id, current_time, ctx.as_deref_mut())
            {
                self.process_measurement(sensor_id, distance, ctx.as_deref_mut());
            }
        }

        // Periodic health check logging
        let health_check_interval = 10_000_000_000; // 10 seconds
        if current_time - self.last_health_check > health_check_interval {
            for sensor_id in 0..self.num_sensors {
                if let Some((total, errors, error_rate)) = self.get_statistics(sensor_id) {
                    if total > 0 {
                        let health = if self.is_healthy(sensor_id) {
                            "HEALTHY"
                        } else {
                            "DEGRADED"
                        };
                        ctx.log_info(&format!(
                            "Sensor {}: {} measurements, {} errors ({:.1}%) - {}",
                            sensor_id, total, errors, error_rate, health
                        ));
                    }
                }
            }
            self.last_health_check = current_time;
        }
    }
}

/// Builder for UltrasonicNode with fluent API for processor configuration
pub struct UltrasonicNodeBuilder<P>
where
    P: Processor<Range>,
{
    topic: String,
    processor: P,
}

impl UltrasonicNodeBuilder<PassThrough<Range>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "ultrasonic.range".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl<P> UltrasonicNodeBuilder<P>
where
    P: Processor<Range>,
{
    /// Set the topic for publishing range measurements
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> UltrasonicNodeBuilder<P2>
    where
        P2: Processor<Range>,
    {
        UltrasonicNodeBuilder {
            topic: self.topic,
            processor,
        }
    }

    /// Add a closure processor for transformations
    pub fn with_closure<F>(self, f: F) -> UltrasonicNodeBuilder<ClosureProcessor<Range, Range, F>>
    where
        F: FnMut(Range) -> Range + Send + 'static,
    {
        UltrasonicNodeBuilder {
            topic: self.topic,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> UltrasonicNodeBuilder<FilterProcessor<Range, Range, F>>
    where
        F: FnMut(Range) -> Option<Range> + Send + 'static,
    {
        UltrasonicNodeBuilder {
            topic: self.topic,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(self, next: P2) -> UltrasonicNodeBuilder<Pipeline<Range, Range, Range, P, P2>>
    where
        P2: Processor<Range, Output = Range>,
        P: Processor<Range, Output = Range>,
    {
        UltrasonicNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the UltrasonicNode
    #[cfg(feature = "gpio-hardware")]
    pub fn build(self) -> Result<UltrasonicNode<P>> {
        const NONE_PIN: Option<Pin> = None;
        let mut node = UltrasonicNode {
            publisher: Hub::new(&self.topic)?,
            processor: self.processor,
            num_sensors: 1,
            sensor_names: [[0; 32]; 16],
            measurement_rate: 10.0,
            speed_of_sound: 343.0,
            temperature_celsius: 20.0,
            min_range: [0.02; 16],
            max_range: [4.0; 16],
            field_of_view: [0.26; 16],
            enable_median_filter: true,
            median_filter_size: 5,
            last_measurement_time: [0; 16],
            current_range: [0.0; 16],
            raw_range_history: [[0.0; 5]; 16],
            history_index: [0; 16],
            measurement_count: [0; 16],
            error_count: [0; 16],
            trigger_sent: [false; 16],
            echo_start_time: [0; 16],
            echo_end_time: [0; 16],
            trigger_duration_ns: 10_000,
            max_echo_time_ns: 38_000_000,
            measurement_interval_ns: 100_000_000,
            last_trigger_time: 0,
            trigger_pins: [NONE_PIN; 16],
            echo_pins: [NONE_PIN; 16],
            hardware_enabled: false,
            gpio_pin_numbers: [(0, 0); 16],
            last_health_check: 0,
        };
        node.set_sensor_name(0, "ultrasonic_0");
        node.update_measurement_interval();
        node.update_speed_of_sound();
        Ok(node)
    }

    /// Build the UltrasonicNode (non-gpio version)
    #[cfg(not(feature = "gpio-hardware"))]
    pub fn build(self) -> Result<UltrasonicNode<P>> {
        let mut node = UltrasonicNode {
            publisher: Hub::new(&self.topic)?,
            processor: self.processor,
            num_sensors: 1,
            sensor_names: [[0; 32]; 16],
            measurement_rate: 10.0,
            speed_of_sound: 343.0,
            temperature_celsius: 20.0,
            min_range: [0.02; 16],
            max_range: [4.0; 16],
            field_of_view: [0.26; 16],
            enable_median_filter: true,
            median_filter_size: 5,
            last_measurement_time: [0; 16],
            current_range: [0.0; 16],
            raw_range_history: [[0.0; 5]; 16],
            history_index: [0; 16],
            measurement_count: [0; 16],
            error_count: [0; 16],
            trigger_sent: [false; 16],
            echo_start_time: [0; 16],
            echo_end_time: [0; 16],
            trigger_duration_ns: 10_000,
            max_echo_time_ns: 38_000_000,
            measurement_interval_ns: 100_000_000,
            last_trigger_time: 0,
            hardware_enabled: false,
            gpio_pin_numbers: [(0, 0); 16],
            last_health_check: 0,
        };
        node.set_sensor_name(0, "ultrasonic_0");
        node.update_measurement_interval();
        node.update_speed_of_sound();
        Ok(node)
    }
}

/// Configuration for specific ultrasonic sensor models
impl UltrasonicNode {
    /// Configure for HC-SR04 sensor
    pub fn configure_hc_sr04(&mut self, sensor_id: u8) {
        self.set_range_limits(sensor_id, 0.02, 4.0);
        self.set_field_of_view_degrees(sensor_id, 15.0);
        if sensor_id < 16 {
            self.max_range[sensor_id as usize] = 4.0;
        }
    }

    /// Configure for JSN-SR04T waterproof sensor
    pub fn configure_jsn_sr04t(&mut self, sensor_id: u8) {
        self.set_range_limits(sensor_id, 0.20, 6.0);
        self.set_field_of_view_degrees(sensor_id, 75.0);
        if sensor_id < 16 {
            self.max_range[sensor_id as usize] = 6.0;
        }
    }

    /// Configure for US-100 sensor
    pub fn configure_us_100(&mut self, sensor_id: u8) {
        self.set_range_limits(sensor_id, 0.02, 4.5);
        self.set_field_of_view_degrees(sensor_id, 15.0);
        if sensor_id < 16 {
            self.max_range[sensor_id as usize] = 4.5;
        }
    }

    /// Configure for Maxbotix MB1xxx series
    pub fn configure_maxbotix_mb1xxx(&mut self, sensor_id: u8, max_range: f32) {
        self.set_range_limits(sensor_id, 0.20, max_range);
        self.set_field_of_view_degrees(sensor_id, 30.0);
        if sensor_id < 16 {
            self.max_range[sensor_id as usize] = max_range;
        }
    }
}
