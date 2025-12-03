use crate::DigitalIO;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

/// Digital I/O Node - Basic digital input/output for industrial sensors and actuators
///
/// Manages digital I/O pins for reading sensors (limit switches, proximity sensors)
/// and controlling actuators (relays, solenoids, LEDs). Supports both GPIO and
/// industrial I/O modules.
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = DigitalIONode::builder()
///     .with_filter(|io| {
///         // Only forward on state changes
///         if io.pin_count > 0 {
///             Some(io)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct DigitalIONode<P = PassThrough<DigitalIO>>
where
    P: Processor<DigitalIO>,
{
    input_publisher: Hub<DigitalIO>,
    output_subscriber: Hub<DigitalIO>,
    status_publisher: Hub<DigitalIO>,

    // Configuration
    input_pin_count: u8,
    output_pin_count: u8,
    update_rate: f32, // Hz

    // State
    input_states: HashMap<u8, bool>,
    output_states: HashMap<u8, bool>,
    last_input_states: HashMap<u8, bool>,
    input_pin_names: HashMap<u8, String>,
    output_pin_names: HashMap<u8, String>,

    // Timing
    last_update_time: u64,
    publish_interval: u64, // milliseconds

    // Simulation
    simulate_inputs: bool,
    sim_input_pattern: u8,

    // Processor for hybrid pattern
    processor: P,
}

impl DigitalIONode {
    /// Create a new digital I/O node with default topics
    pub fn new() -> Result<Self> {
        Self::new_with_topics("digital_input", "digital_output", "io_status")
    }

    /// Create a new digital I/O node with custom topics
    pub fn new_with_topics(
        input_topic: &str,
        output_topic: &str,
        status_topic: &str,
    ) -> Result<Self> {
        Ok(Self {
            input_publisher: Hub::new(input_topic)?,
            output_subscriber: Hub::new(output_topic)?,
            status_publisher: Hub::new(status_topic)?,

            input_pin_count: 8,  // Default 8 input pins
            output_pin_count: 8, // Default 8 output pins
            update_rate: 10.0,   // 10 Hz default

            input_states: HashMap::new(),
            output_states: HashMap::new(),
            last_input_states: HashMap::new(),
            input_pin_names: HashMap::new(),
            output_pin_names: HashMap::new(),

            last_update_time: 0,
            publish_interval: 100, // 100ms = 10 Hz

            simulate_inputs: true,
            sim_input_pattern: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> DigitalIONodeBuilder<PassThrough<DigitalIO>> {
        DigitalIONodeBuilder::new()
    }
}

impl<P> DigitalIONode<P>
where
    P: Processor<DigitalIO>,
{
    /// Set number of I/O pins
    pub fn set_pin_counts(&mut self, input_count: u8, output_count: u8) {
        self.input_pin_count = input_count;
        self.output_pin_count = output_count;

        // Initialize pin states
        for i in 0..input_count {
            self.input_states.insert(i, false);
            self.last_input_states.insert(i, false);
            self.input_pin_names.insert(i, format!("DI{}", i));
        }

        for i in 0..output_count {
            self.output_states.insert(i, false);
            self.output_pin_names.insert(i, format!("DO{}", i));
        }
    }

    /// Set update rate in Hz
    pub fn set_update_rate(&mut self, rate: f32) {
        self.update_rate = rate.clamp(0.1, 1000.0);
        self.publish_interval = (1000.0 / self.update_rate) as u64;
    }

    /// Set input pin name
    pub fn set_input_pin_name(&mut self, pin: u8, name: &str) {
        if pin < self.input_pin_count {
            self.input_pin_names.insert(pin, name.to_string());
        }
    }

    /// Set output pin name
    pub fn set_output_pin_name(&mut self, pin: u8, name: &str) {
        if pin < self.output_pin_count {
            self.output_pin_names.insert(pin, name.to_string());
        }
    }

    /// Get input pin state
    pub fn get_input(&self, pin: u8) -> Option<bool> {
        self.input_states.get(&pin).copied()
    }

    /// Get output pin state
    pub fn get_output(&self, pin: u8) -> Option<bool> {
        self.output_states.get(&pin).copied()
    }

    /// Set output pin state (programmatic control)
    pub fn set_output(&mut self, pin: u8, state: bool) {
        if pin < self.output_pin_count {
            self.output_states.insert(pin, state);
            self.write_output_pin(pin, state);
        }
    }

    /// Enable/disable input simulation
    pub fn set_simulation(&mut self, enabled: bool) {
        self.simulate_inputs = enabled;
    }

    #[cfg(feature = "raspberry-pi")]
    fn read_gpio_pin(&self, pin: u8) -> bool {
        use rppal::gpio::{Gpio, Level};

        if let Ok(gpio) = Gpio::new() {
            if let Ok(input_pin) = gpio.get(pin) {
                let input = input_pin.into_input_pullup();
                return input.read() == Level::High;
            }
        }
        false
    }

    #[cfg(not(feature = "raspberry-pi"))]
    fn read_gpio_pin(&self, _pin: u8) -> bool {
        false // GPIO not available
    }

    #[cfg(feature = "raspberry-pi")]
    fn write_gpio_pin(&self, pin: u8, state: bool) {
        use rppal::gpio::{Gpio, Level};

        if let Ok(gpio) = Gpio::new() {
            if let Ok(output_pin) = gpio.get(pin) {
                let mut output = output_pin.into_output();
                let _ = output.write(if state { Level::High } else { Level::Low });
            }
        }
    }

    #[cfg(not(feature = "raspberry-pi"))]
    fn write_gpio_pin(&self, _pin: u8, _state: bool) {
        // GPIO not available - would control industrial I/O module here
    }

    fn read_input_pins(&mut self) {
        for pin in 0..self.input_pin_count {
            let state = if self.simulate_inputs {
                // Generate simulated input pattern
                self.sim_input_pattern += 1;
                (self.sim_input_pattern >> pin) & 1 == 1
            } else {
                // Read actual hardware
                self.read_gpio_pin(pin)
            };

            self.input_states.insert(pin, state);
        }
    }

    fn write_output_pin(&self, pin: u8, state: bool) {
        // Write to actual hardware
        self.write_gpio_pin(pin, state);
    }

    fn handle_output_command(&mut self, command: DigitalIO) {
        // Extract pin number and state from command
        if command.pin_count > 0 && (command.pin_count as usize) <= command.pins.len() {
            for (i, &state) in command
                .pins
                .iter()
                .take(command.pin_count as usize)
                .enumerate()
            {
                let pin = i as u8;
                if pin < self.output_pin_count {
                    self.output_states.insert(pin, state);
                    self.write_output_pin(pin, state);
                }
            }
        }
    }

    fn publish_input_states(&mut self) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Create pin state array
        let mut pins = [false; 32];
        for i in 0..self.input_pin_count {
            pins[i as usize] = self.input_states.get(&i).copied().unwrap_or(false);
        }

        // Create pin labels
        let mut pin_labels = [[0u8; 16]; 32];
        for i in 0..self.input_pin_count {
            let label = self
                .input_pin_names
                .get(&i)
                .cloned()
                .unwrap_or_else(|| format!("DI{}", i));
            let label_bytes = label.as_bytes();
            let len = label_bytes.len().min(15);
            pin_labels[i as usize][..len].copy_from_slice(&label_bytes[..len]);
        }

        let digital_input = DigitalIO {
            pin_count: self.input_pin_count,
            pins,
            pin_labels,
            timestamp: current_time,
            ..Default::default()
        };

        // Process through pipeline
        if let Some(processed) = self.processor.process(digital_input) {
            let _ = self.input_publisher.send(processed, &mut None);
        }
    }

    fn publish_status(&mut self) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Combine input and output states for status
        let total_pins = self.input_pin_count + self.output_pin_count;
        let mut all_pins = [false; 32];
        let mut all_labels = [[0u8; 16]; 32];

        // Add input states
        for i in 0..self.input_pin_count {
            all_pins[i as usize] = self.input_states.get(&i).copied().unwrap_or(false);
            let label = self
                .input_pin_names
                .get(&i)
                .cloned()
                .unwrap_or_else(|| format!("DI{}", i));
            let label_bytes = label.as_bytes();
            let len = label_bytes.len().min(15);
            all_labels[i as usize][..len].copy_from_slice(&label_bytes[..len]);
        }

        // Add output states
        for i in 0..self.output_pin_count {
            let idx = (self.input_pin_count + i) as usize;
            all_pins[idx] = self.output_states.get(&i).copied().unwrap_or(false);
            let label = self
                .output_pin_names
                .get(&i)
                .cloned()
                .unwrap_or_else(|| format!("DO{}", i));
            let label_bytes = label.as_bytes();
            let len = label_bytes.len().min(15);
            all_labels[idx][..len].copy_from_slice(&label_bytes[..len]);
        }

        let status = DigitalIO {
            pin_count: total_pins,
            pins: all_pins,
            pin_labels: all_labels,
            timestamp: current_time,
            ..Default::default()
        };

        // Process through pipeline
        if let Some(processed) = self.processor.process(status) {
            let _ = self.status_publisher.send(processed, &mut None);
        }
    }

    fn detect_input_changes(&mut self) -> bool {
        let mut changes_detected = false;

        for pin in 0..self.input_pin_count {
            let current = self.input_states.get(&pin).copied().unwrap_or(false);
            let last = self.last_input_states.get(&pin).copied().unwrap_or(false);

            if current != last {
                changes_detected = true;
                self.last_input_states.insert(pin, current);
            }
        }

        changes_detected
    }
}

impl<P> Node for DigitalIONode<P>
where
    P: Processor<DigitalIO>,
{
    fn name(&self) -> &'static str {
        "DigitalIONode"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        ctx.log_info("DigitalIONode shutting down - setting all outputs to LOW");

        // Set all outputs to LOW for safety
        for pin in 0..self.output_pin_count {
            self.output_states.insert(pin, false);
            self.write_output_pin(pin, false);
        }

        ctx.log_info("All digital outputs set to LOW safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Handle incoming output commands
        if let Some(output_cmd) = self.output_subscriber.recv(&mut None) {
            self.handle_output_command(output_cmd);
        }

        // Read input pins
        self.read_input_pins();

        // Publish inputs on change or periodic update
        let should_publish = if self.detect_input_changes() {
            true // Publish immediately on input change
        } else {
            current_time - self.last_update_time >= self.publish_interval
        };

        if should_publish {
            self.publish_input_states();
            self.publish_status();
            self.last_update_time = current_time;
        }
    }
}

// Default impl removed - use Node::new() instead which returns HorusResult

/// Builder for DigitalIONode with processor configuration
pub struct DigitalIONodeBuilder<P>
where
    P: Processor<DigitalIO>,
{
    input_topic: String,
    output_topic: String,
    status_topic: String,
    processor: P,
}

impl DigitalIONodeBuilder<PassThrough<DigitalIO>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            input_topic: "digital_input".to_string(),
            output_topic: "digital_output".to_string(),
            status_topic: "io_status".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for DigitalIONodeBuilder<PassThrough<DigitalIO>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> DigitalIONodeBuilder<P>
where
    P: Processor<DigitalIO>,
{
    /// Set input topic
    pub fn input_topic(mut self, topic: &str) -> Self {
        self.input_topic = topic.to_string();
        self
    }

    /// Set output topic
    pub fn output_topic(mut self, topic: &str) -> Self {
        self.output_topic = topic.to_string();
        self
    }

    /// Set status topic
    pub fn status_topic(mut self, topic: &str) -> Self {
        self.status_topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> DigitalIONodeBuilder<P2>
    where
        P2: Processor<DigitalIO>,
    {
        DigitalIONodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            status_topic: self.status_topic,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> DigitalIONodeBuilder<ClosureProcessor<DigitalIO, DigitalIO, F>>
    where
        F: FnMut(DigitalIO) -> DigitalIO + Send + 'static,
    {
        DigitalIONodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            status_topic: self.status_topic,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> DigitalIONodeBuilder<FilterProcessor<DigitalIO, DigitalIO, F>>
    where
        F: FnMut(DigitalIO) -> Option<DigitalIO> + Send + 'static,
    {
        DigitalIONodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            status_topic: self.status_topic,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> DigitalIONodeBuilder<Pipeline<DigitalIO, DigitalIO, DigitalIO, P, P2>>
    where
        P2: Processor<DigitalIO, DigitalIO>,
    {
        DigitalIONodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            status_topic: self.status_topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<DigitalIONode<P>> {
        Ok(DigitalIONode {
            input_publisher: Hub::new(&self.input_topic)?,
            output_subscriber: Hub::new(&self.output_topic)?,
            status_publisher: Hub::new(&self.status_topic)?,

            input_pin_count: 8,
            output_pin_count: 8,
            update_rate: 10.0,

            input_states: HashMap::new(),
            output_states: HashMap::new(),
            last_input_states: HashMap::new(),
            input_pin_names: HashMap::new(),
            output_pin_names: HashMap::new(),

            last_update_time: 0,
            publish_interval: 100,

            simulate_inputs: true,
            sim_input_pattern: 0,
            processor: self.processor,
        })
    }
}
