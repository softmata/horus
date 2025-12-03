use crate::{EmergencyStop, SafetyStatus};
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

/// Emergency Stop Node - Hardware emergency stop handler for industrial safety
///
/// Monitors emergency stop buttons, software triggers, and system conditions.
/// Publishes EmergencyStop messages when triggered and maintains safety state.
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = EmergencyStopNode::builder()
///     .with_closure(|estop| {
///         // Log all emergency stops
///         println!("E-Stop: {}", estop.engaged);
///         estop
///     })
///     .build()?;
/// ```
pub struct EmergencyStopNode<P = PassThrough<EmergencyStop>>
where
    P: Processor<EmergencyStop>,
{
    publisher: Hub<EmergencyStop>,
    safety_publisher: Hub<SafetyStatus>,
    is_stopped: Arc<AtomicBool>,
    stop_reason: String,
    gpio_pin: Option<u8>,
    #[allow(dead_code)]
    last_gpio_state: bool,
    auto_reset: bool,
    stop_timeout_ms: u64,
    last_stop_time: u64,
    processor: P,
}

impl EmergencyStopNode {
    /// Create a new emergency stop node with default topic "emergency_stop"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("emergency_stop")
    }

    /// Create a new emergency stop node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        let safety_topic = format!("{}_safety", topic);
        Ok(Self {
            publisher: Hub::new(topic)?,
            safety_publisher: Hub::new(&safety_topic)?,
            is_stopped: Arc::new(AtomicBool::new(false)),
            stop_reason: String::new(),
            gpio_pin: None,
            last_gpio_state: true, // Assume normal state (not pressed)
            auto_reset: false,
            stop_timeout_ms: 5000, // 5 second timeout for auto-reset
            last_stop_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> EmergencyStopNodeBuilder<PassThrough<EmergencyStop>> {
        EmergencyStopNodeBuilder::new()
    }
}

impl<P> EmergencyStopNode<P>
where
    P: Processor<EmergencyStop>,
{
    /// Set GPIO pin for hardware emergency stop button (Raspberry Pi)
    pub fn set_gpio_pin(&mut self, pin: u8) {
        self.gpio_pin = Some(pin);
    }

    /// Enable or disable automatic reset after timeout
    pub fn set_auto_reset(&mut self, enabled: bool) {
        self.auto_reset = enabled;
    }

    /// Set timeout for automatic reset in milliseconds
    pub fn set_reset_timeout(&mut self, timeout_ms: u64) {
        self.stop_timeout_ms = timeout_ms;
    }

    /// Trigger emergency stop with reason
    pub fn trigger_stop(&mut self, reason: &str) {
        self.is_stopped.store(true, Ordering::Relaxed);
        self.stop_reason = reason.to_string();
        self.last_stop_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        self.publish_emergency_stop(true, reason);
        self.publish_safety_status();
    }

    /// Reset emergency stop (manual reset)
    pub fn reset(&mut self) {
        if self.is_stopped.load(Ordering::Relaxed) {
            self.is_stopped.store(false, Ordering::Relaxed);
            self.stop_reason.clear();
            self.publish_emergency_stop(false, "Reset");
            self.publish_safety_status();
        }
    }

    /// Check if emergency stop is active
    pub fn is_emergency_stopped(&self) -> bool {
        self.is_stopped.load(Ordering::Relaxed)
    }

    /// Get current stop reason
    pub fn get_stop_reason(&self) -> String {
        self.stop_reason.clone()
    }

    #[cfg(feature = "raspberry-pi")]
    fn check_gpio_pin(&mut self) -> bool {
        use rppal::gpio::{Gpio, Level};

        if let Some(pin_num) = self.gpio_pin {
            if let Some(gpio) = Gpio::new() {
                if let Some(pin) = gpio.get(pin_num) {
                    let input = pin.into_input_pullup();
                    let current_state = input.read() == Level::High;

                    // Emergency stop button pressed (assuming active low)
                    if self.last_gpio_state && !current_state {
                        self.last_gpio_state = current_state;
                        return true; // Emergency stop triggered
                    }

                    self.last_gpio_state = current_state;
                }
            }
        }
        false
    }

    #[cfg(not(feature = "raspberry-pi"))]
    fn check_gpio_pin(&mut self) -> bool {
        false // No GPIO support in simulation
    }

    fn check_auto_reset(&mut self) {
        if self.auto_reset && self.is_stopped.load(Ordering::Relaxed) {
            let current_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;

            if current_time - self.last_stop_time > self.stop_timeout_ms {
                self.reset();
            }
        }
    }

    fn publish_emergency_stop(&mut self, is_emergency: bool, reason: &str) {
        let emergency_stop = if is_emergency {
            EmergencyStop::engage(reason)
        } else {
            EmergencyStop::release()
        };
        // Process through pipeline
        if let Some(processed) = self.processor.process(emergency_stop) {
            let _ = self.publisher.send(processed, &mut None);
        }
    }

    fn publish_safety_status(&self) {
        let status = if self.is_stopped.load(Ordering::Relaxed) {
            {
                let mut status = SafetyStatus::new();
                status.estop_engaged = true;
                status.mode = SafetyStatus::MODE_SAFE_STOP;
                status.set_fault(1); // Emergency stop fault code
                status
            }
        } else {
            SafetyStatus::new()
        };
        let _ = self.safety_publisher.send(status, &mut None);
    }
}

impl<P> Node for EmergencyStopNode<P>
where
    P: Processor<EmergencyStop>,
{
    fn name(&self) -> &'static str {
        "EmergencyStopNode"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        Ok(())
    }

    fn shutdown(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        // Check GPIO pin for hardware button
        if self.check_gpio_pin() {
            self.trigger_stop("Hardware emergency stop button pressed");
        }

        // Check for auto-reset timeout
        self.check_auto_reset();

        // Periodically publish safety status
        self.publish_safety_status();
    }
}

// Default impl removed - use Node::new() instead which returns HorusResult

/// Builder for EmergencyStopNode with processor configuration
pub struct EmergencyStopNodeBuilder<P>
where
    P: Processor<EmergencyStop>,
{
    topic: String,
    processor: P,
}

impl EmergencyStopNodeBuilder<PassThrough<EmergencyStop>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            topic: "emergency_stop".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for EmergencyStopNodeBuilder<PassThrough<EmergencyStop>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> EmergencyStopNodeBuilder<P>
where
    P: Processor<EmergencyStop>,
{
    /// Set output topic
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> EmergencyStopNodeBuilder<P2>
    where
        P2: Processor<EmergencyStop>,
    {
        EmergencyStopNodeBuilder {
            topic: self.topic,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> EmergencyStopNodeBuilder<ClosureProcessor<EmergencyStop, EmergencyStop, F>>
    where
        F: FnMut(EmergencyStop) -> EmergencyStop + Send + 'static,
    {
        EmergencyStopNodeBuilder {
            topic: self.topic,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> EmergencyStopNodeBuilder<FilterProcessor<EmergencyStop, EmergencyStop, F>>
    where
        F: FnMut(EmergencyStop) -> Option<EmergencyStop> + Send + 'static,
    {
        EmergencyStopNodeBuilder {
            topic: self.topic,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> EmergencyStopNodeBuilder<Pipeline<EmergencyStop, EmergencyStop, EmergencyStop, P, P2>>
    where
        P2: Processor<EmergencyStop, EmergencyStop>,
    {
        EmergencyStopNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<EmergencyStopNode<P>> {
        let safety_topic = format!("{}_safety", self.topic);
        Ok(EmergencyStopNode {
            publisher: Hub::new(&self.topic)?,
            safety_publisher: Hub::new(&safety_topic)?,
            is_stopped: Arc::new(AtomicBool::new(false)),
            stop_reason: String::new(),
            gpio_pin: None,
            last_gpio_state: true,
            auto_reset: false,
            stop_timeout_ms: 5000,
            last_stop_time: 0,
            processor: self.processor,
        })
    }
}
