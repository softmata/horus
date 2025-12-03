use crate::JoystickInput;
use horus_core::error::HorusResult;
use std::collections::HashMap;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(feature = "gilrs")]
use gilrs::{Axis, Button, Event, EventType, Gilrs};

#[cfg(not(feature = "gilrs"))]
use std::time::{SystemTime, UNIX_EPOCH};

/// Button mapping profile type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ButtonMapping {
    Xbox360,
    PlayStation4,
    Generic,
}

/// Axis calibration data
#[derive(Debug, Clone, Copy)]
pub struct AxisCalibration {
    pub center: f32,
    pub min: f32,
    pub max: f32,
}

impl Default for AxisCalibration {
    fn default() -> Self {
        Self {
            center: 0.0,
            min: -1.0,
            max: 1.0,
        }
    }
}

/// Joystick Input Node - Real gamepad/joystick input capture
///
/// Captures real joystick/gamepad input using the gilrs library.
/// Publishes button presses and axis movements to the Hub.
///
/// # Hybrid Pattern
///
/// This node supports the hybrid pattern for custom processing:
///
/// ```rust,ignore
/// // Default mode - just captures joystick input
/// let node = JoystickInputNode::new()?;
///
/// // With custom processing - filter and transform
/// let node = JoystickInputNode::builder()
///     .with_topic("custom_joystick")
///     .with_filter(|input| {
///         // Only pass through significant axis movements
///         if input.axis_value.abs() > 0.5 {
///             Some(input)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct JoystickInputNode<P = PassThrough<JoystickInput>>
where
    P: Processor<JoystickInput>,
{
    publisher: Hub<JoystickInput>,
    #[cfg(feature = "gilrs")]
    gilrs: Gilrs,
    #[cfg(not(feature = "gilrs"))]
    last_input_time: u64,

    // Configuration
    device_id: u32,
    deadzone: f32,
    axis_invert_x: bool,
    axis_invert_y: bool,
    axis_invert_rx: bool,
    axis_invert_ry: bool,
    button_mapping: ButtonMapping,

    // Custom mappings
    custom_button_names: HashMap<u32, String>,
    custom_axis_names: HashMap<u32, String>,

    // Calibration data
    axis_calibrations: HashMap<u32, AxisCalibration>,

    // Per-axis deadzones
    per_axis_deadzones: HashMap<u32, f32>,

    // Processor for hybrid pattern
    processor: P,
}

impl JoystickInputNode {
    /// Create a new joystick input node with default topic "joystick_input"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("joystick_input")
    }

    /// Create a new joystick input node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        #[cfg(feature = "gilrs")]
        {
            let gilrs = Gilrs::new().map_err(|e| {
                horus_core::error::HorusError::InitializationFailed(format!(
                    "Failed to initialize gilrs: {}",
                    e
                ))
            })?;

            Ok(Self {
                publisher: Hub::new(topic)?,
                gilrs,
                device_id: 0,
                deadzone: 0.1,
                axis_invert_x: false,
                axis_invert_y: false,
                axis_invert_rx: false,
                axis_invert_ry: false,
                button_mapping: ButtonMapping::Generic,
                custom_button_names: HashMap::new(),
                custom_axis_names: HashMap::new(),
                axis_calibrations: HashMap::new(),
                per_axis_deadzones: HashMap::new(),
                processor: PassThrough::new(),
            })
        }

        #[cfg(not(feature = "gilrs"))]
        {
            Ok(Self {
                publisher: Hub::new(topic)?,
                last_input_time: 0,
                device_id: 0,
                deadzone: 0.1,
                axis_invert_x: false,
                axis_invert_y: false,
                axis_invert_rx: false,
                axis_invert_ry: false,
                button_mapping: ButtonMapping::Generic,
                custom_button_names: HashMap::new(),
                custom_axis_names: HashMap::new(),
                axis_calibrations: HashMap::new(),
                per_axis_deadzones: HashMap::new(),
                processor: PassThrough::new(),
            })
        }
    }

    /// Create a builder for configuring the node with custom processing
    pub fn builder() -> JoystickInputNodeBuilder<PassThrough<JoystickInput>> {
        JoystickInputNodeBuilder::new()
    }
}

impl<P> JoystickInputNode<P>
where
    P: Processor<JoystickInput>,
{
    /// Set the device ID for multi-controller setups
    pub fn set_device_id(&mut self, device_id: u32) {
        self.device_id = device_id;
    }

    /// Get the current device ID
    pub fn get_device_id(&self) -> u32 {
        self.device_id
    }

    /// Set global deadzone for all axes (0.0 to 1.0)
    pub fn set_deadzone(&mut self, deadzone: f32) {
        self.deadzone = deadzone.clamp(0.0, 1.0);
    }

    /// Get the current global deadzone
    pub fn get_deadzone(&self) -> f32 {
        self.deadzone
    }

    /// Set per-axis deadzone
    pub fn set_axis_deadzone(&mut self, axis_id: u32, deadzone: f32) {
        self.per_axis_deadzones
            .insert(axis_id, deadzone.clamp(0.0, 1.0));
    }

    /// Get per-axis deadzone (returns global deadzone if not set)
    pub fn get_axis_deadzone(&self, axis_id: u32) -> f32 {
        self.per_axis_deadzones
            .get(&axis_id)
            .copied()
            .unwrap_or(self.deadzone)
    }

    /// Set axis inversion for specific axes
    pub fn set_axis_inversion(
        &mut self,
        invert_x: bool,
        invert_y: bool,
        invert_rx: bool,
        invert_ry: bool,
    ) {
        self.axis_invert_x = invert_x;
        self.axis_invert_y = invert_y;
        self.axis_invert_rx = invert_rx;
        self.axis_invert_ry = invert_ry;
    }

    /// Set button mapping profile
    pub fn set_button_mapping(&mut self, mapping: ButtonMapping) {
        self.button_mapping = mapping;
    }

    /// Get current button mapping profile
    pub fn get_button_mapping(&self) -> ButtonMapping {
        self.button_mapping
    }

    /// Set custom button name for a specific button ID
    pub fn set_custom_button_name(&mut self, button_id: u32, name: String) {
        self.custom_button_names.insert(button_id, name);
    }

    /// Set custom axis name for a specific axis ID
    pub fn set_custom_axis_name(&mut self, axis_id: u32, name: String) {
        self.custom_axis_names.insert(axis_id, name);
    }

    /// Calibrate a specific axis
    pub fn calibrate_axis(&mut self, axis_id: u32, center: f32, min: f32, max: f32) {
        self.axis_calibrations
            .insert(axis_id, AxisCalibration { center, min, max });
    }

    /// Start automatic axis calibration (call this and move all sticks to extremes)
    pub fn calibrate_axes(&mut self) {
        // Reset calibrations to defaults
        self.axis_calibrations.clear();
    }

    /// Check if a controller is connected
    #[cfg(feature = "gilrs")]
    pub fn is_connected(&self) -> bool {
        self.gilrs.gamepads().count() > 0
    }

    #[cfg(not(feature = "gilrs"))]
    pub fn is_connected(&self) -> bool {
        false
    }

    /// Get battery level (0.0 to 1.0) if supported
    #[cfg(feature = "gilrs")]
    pub fn get_battery_level(&self) -> Option<f32> {
        use gilrs::PowerInfo;

        // Try to get the connected gamepad
        if let Some((_id, gamepad)) = self.gilrs.gamepads().find(|(_, gp)| gp.is_connected()) {
            // Get power info from gamepad (returns PowerInfo directly in gilrs 0.10+)
            let power = gamepad.power_info();
            return match power {
                PowerInfo::Unknown => None,
                PowerInfo::Wired => Some(1.0), // Wired controllers at full power
                PowerInfo::Discharging(level) => Some(level as f32 / 100.0),
                PowerInfo::Charging(level) => Some(level as f32 / 100.0),
                PowerInfo::Charged => Some(1.0),
            };
        }
        None
    }

    #[cfg(not(feature = "gilrs"))]
    pub fn get_battery_level(&self) -> Option<f32> {
        None
    }

    /// Apply deadzone to an axis value
    fn apply_deadzone(&self, value: f32, axis_id: u32) -> f32 {
        let deadzone = self.get_axis_deadzone(axis_id);

        if value.abs() < deadzone {
            0.0
        } else {
            // Scale to maintain full range after deadzone
            let sign = value.signum();
            let abs_value = value.abs();
            sign * (abs_value - deadzone) / (1.0 - deadzone)
        }
    }

    /// Apply calibration to an axis value
    fn apply_calibration(&self, value: f32, axis_id: u32) -> f32 {
        if let Some(cal) = self.axis_calibrations.get(&axis_id) {
            // Apply center offset
            let centered = value - cal.center;

            // Normalize to -1.0 to 1.0 range
            if centered < 0.0 {
                (centered / (cal.center - cal.min)).clamp(-1.0, 0.0)
            } else {
                (centered / (cal.max - cal.center)).clamp(0.0, 1.0)
            }
        } else {
            value
        }
    }

    /// Apply axis inversion
    fn apply_inversion(&self, value: f32, axis_id: u32) -> f32 {
        let should_invert = match axis_id {
            0 => self.axis_invert_x,  // LeftStickX
            1 => self.axis_invert_y,  // LeftStickY
            3 => self.axis_invert_rx, // RightStickX
            4 => self.axis_invert_ry, // RightStickY
            _ => false,
        };

        if should_invert {
            -value
        } else {
            value
        }
    }

    /// Process axis value through all filters (calibration, deadzone, inversion)
    fn process_axis_value(&self, value: f32, axis_id: u32) -> f32 {
        let calibrated = self.apply_calibration(value, axis_id);
        let deadzone_applied = self.apply_deadzone(calibrated, axis_id);
        self.apply_inversion(deadzone_applied, axis_id)
    }

    /// Get button name based on mapping profile
    #[cfg(feature = "gilrs")]
    fn get_button_name(&self, button: Button, button_id: u32) -> String {
        // Check custom mapping first
        if let Some(custom_name) = self.custom_button_names.get(&button_id) {
            return custom_name.clone();
        }

        // Apply profile-specific naming
        match self.button_mapping {
            ButtonMapping::Xbox360 => match button {
                Button::South => "A".to_string(),
                Button::East => "B".to_string(),
                Button::North => "X".to_string(),
                Button::West => "Y".to_string(),
                Button::LeftTrigger => "LB".to_string(),
                Button::LeftTrigger2 => "LT".to_string(),
                Button::RightTrigger => "RB".to_string(),
                Button::RightTrigger2 => "RT".to_string(),
                Button::Select => "Back".to_string(),
                Button::Start => "Start".to_string(),
                Button::Mode => "Xbox".to_string(),
                Button::LeftThumb => "LS".to_string(),
                Button::RightThumb => "RS".to_string(),
                _ => format!("{:?}", button),
            },
            ButtonMapping::PlayStation4 => match button {
                Button::South => "Cross".to_string(),
                Button::East => "Circle".to_string(),
                Button::North => "Square".to_string(),
                Button::West => "Triangle".to_string(),
                Button::LeftTrigger => "L1".to_string(),
                Button::LeftTrigger2 => "L2".to_string(),
                Button::RightTrigger => "R1".to_string(),
                Button::RightTrigger2 => "R2".to_string(),
                Button::Select => "Share".to_string(),
                Button::Start => "Options".to_string(),
                Button::Mode => "PS".to_string(),
                Button::LeftThumb => "L3".to_string(),
                Button::RightThumb => "R3".to_string(),
                _ => format!("{:?}", button),
            },
            ButtonMapping::Generic => format!("{:?}", button),
        }
    }

    /// Get axis name based on mapping
    #[cfg(feature = "gilrs")]
    fn get_axis_name(&self, axis: Axis, axis_id: u32) -> String {
        // Check custom mapping first
        if let Some(custom_name) = self.custom_axis_names.get(&axis_id) {
            return custom_name.clone();
        }

        // Use standard naming
        format!("{:?}", axis)
    }
}

impl<P> Node for JoystickInputNode<P>
where
    P: Processor<JoystickInput>,
{
    fn name(&self) -> &'static str {
        "JoystickInputNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Call processor hook
        self.processor.on_start();

        #[cfg(feature = "gilrs")]
        {
            let connected = self.gilrs.gamepads().count();
            ctx.log_info(&format!(
                "Joystick input node initialized - {} gamepad(s) connected",
                connected
            ));
        }

        #[cfg(not(feature = "gilrs"))]
        {
            ctx.log_warn("Joystick input node in placeholder mode - build with 'gilrs' feature for real gamepad support");
        }

        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("JoystickInputNode shutting down");
        self.processor.on_shutdown();
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        #[cfg(feature = "gilrs")]
        {
            // Poll for gamepad events
            while let Some(Event { id, event, time: _ }) = self.gilrs.next_event() {
                let gamepad_id: u32 = usize::from(id) as u32;

                match event {
                    EventType::ButtonPressed(button, _) => {
                        let button_id = button_to_id(button);
                        let button_name = self.get_button_name(button, button_id);

                        let joystick_input = JoystickInput::new_button(
                            gamepad_id,
                            button_id,
                            button_name.clone(),
                            true,
                        );

                        // Process through processor pipeline
                        if let Some(processed) = self.processor.process(joystick_input) {
                            self.publisher.send(processed, &mut ctx).ok();
                        }
                        ctx.log_debug(&format!(
                            "Button pressed: {} (gamepad {})",
                            button_name, gamepad_id
                        ));
                    }
                    EventType::ButtonReleased(button, _) => {
                        let button_id = button_to_id(button);
                        let button_name = self.get_button_name(button, button_id);

                        let joystick_input =
                            JoystickInput::new_button(gamepad_id, button_id, button_name, false);

                        // Process through processor pipeline
                        if let Some(processed) = self.processor.process(joystick_input) {
                            self.publisher.send(processed, &mut ctx).ok();
                        }
                    }
                    EventType::AxisChanged(axis, value, _) => {
                        let axis_id = axis_to_id(axis);
                        let axis_name = self.get_axis_name(axis, axis_id);

                        // Process axis value through calibration, deadzone, and inversion
                        let processed_value = self.process_axis_value(value, axis_id);

                        let joystick_input = JoystickInput::new_axis(
                            gamepad_id,
                            axis_id,
                            axis_name.clone(),
                            processed_value,
                        );

                        // Process through processor pipeline
                        if let Some(processed) = self.processor.process(joystick_input) {
                            self.publisher.send(processed, &mut ctx).ok();
                        }

                        // Only log significant axis movements to avoid spam
                        if processed_value.abs() > 0.5 {
                            ctx.log_debug(&format!(
                                "Axis {}: {:.2} (gamepad {})",
                                axis_name, processed_value, gamepad_id
                            ));
                        }
                    }
                    EventType::Connected => {
                        ctx.log_info(&format!("Gamepad {} connected", gamepad_id));

                        // Publish connection event
                        let connection_event = JoystickInput::new_connection(gamepad_id, true);
                        if let Some(processed) = self.processor.process(connection_event) {
                            self.publisher.send(processed, &mut ctx).ok();
                        }
                    }
                    EventType::Disconnected => {
                        ctx.log_info(&format!("Gamepad {} disconnected", gamepad_id));

                        // Publish disconnection event
                        let disconnection_event = JoystickInput::new_connection(gamepad_id, false);
                        if let Some(processed) = self.processor.process(disconnection_event) {
                            self.publisher.send(processed, &mut ctx).ok();
                        }
                    }
                    _ => {}
                }
            }
        }

        #[cfg(not(feature = "gilrs"))]
        {
            // Placeholder implementation when gilrs feature is not enabled
            let current_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;

            if current_time - self.last_input_time > 3000 {
                let joystick_input =
                    JoystickInput::new_button(1, 0, "ButtonA (placeholder)".to_string(), true);
                // Process through processor pipeline
                if let Some(processed) = self.processor.process(joystick_input) {
                    self.publisher.send(processed, &mut ctx).ok();
                }
                ctx.log_debug("Published placeholder joystick input");
                self.last_input_time = current_time;
            }
        }
    }
}

#[cfg(feature = "gilrs")]
fn button_to_id(button: Button) -> u32 {
    match button {
        Button::South => 0,
        Button::East => 1,
        Button::North => 2,
        Button::West => 3,
        Button::LeftTrigger => 4,
        Button::LeftTrigger2 => 5,
        Button::RightTrigger => 6,
        Button::RightTrigger2 => 7,
        Button::Select => 8,
        Button::Start => 9,
        Button::Mode => 10,
        Button::LeftThumb => 11,
        Button::RightThumb => 12,
        Button::DPadUp => 13,
        Button::DPadDown => 14,
        Button::DPadLeft => 15,
        Button::DPadRight => 16,
        _ => 255,
    }
}

#[cfg(feature = "gilrs")]
fn axis_to_id(axis: Axis) -> u32 {
    match axis {
        Axis::LeftStickX => 0,
        Axis::LeftStickY => 1,
        Axis::LeftZ => 2,
        Axis::RightStickX => 3,
        Axis::RightStickY => 4,
        Axis::RightZ => 5,
        Axis::DPadX => 6,
        Axis::DPadY => 7,
        _ => 255,
    }
}

/// Builder for JoystickInputNode with custom processor
pub struct JoystickInputNodeBuilder<P>
where
    P: Processor<JoystickInput>,
{
    topic: String,
    processor: P,
}

impl JoystickInputNodeBuilder<PassThrough<JoystickInput>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "joystick_input".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for JoystickInputNodeBuilder<PassThrough<JoystickInput>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> JoystickInputNodeBuilder<P>
where
    P: Processor<JoystickInput>,
{
    /// Set the topic for publishing joystick input
    pub fn with_topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> JoystickInputNodeBuilder<P2>
    where
        P2: Processor<JoystickInput>,
    {
        JoystickInputNodeBuilder {
            topic: self.topic,
            processor,
        }
    }

    /// Add a closure-based processor
    #[allow(clippy::type_complexity)]
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> JoystickInputNodeBuilder<
        Pipeline<
            JoystickInput,
            JoystickInput,
            JoystickInput,
            P,
            ClosureProcessor<JoystickInput, JoystickInput, F>,
        >,
    >
    where
        F: FnMut(JoystickInput) -> JoystickInput + Send + 'static,
    {
        JoystickInputNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, ClosureProcessor::new(f)),
        }
    }

    /// Add a filter processor
    #[allow(clippy::type_complexity)]
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> JoystickInputNodeBuilder<
        Pipeline<
            JoystickInput,
            JoystickInput,
            JoystickInput,
            P,
            FilterProcessor<JoystickInput, JoystickInput, F>,
        >,
    >
    where
        F: FnMut(JoystickInput) -> Option<JoystickInput> + Send + 'static,
    {
        JoystickInputNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, FilterProcessor::new(f)),
        }
    }

    /// Pipe to another processor (output must remain JoystickInput)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> JoystickInputNodeBuilder<Pipeline<JoystickInput, JoystickInput, JoystickInput, P, P2>>
    where
        P2: Processor<JoystickInput, JoystickInput>,
    {
        JoystickInputNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<JoystickInputNode<P>> {
        #[cfg(feature = "gilrs")]
        {
            let gilrs = Gilrs::new().map_err(|e| {
                horus_core::error::HorusError::InitializationFailed(format!(
                    "Failed to initialize gilrs: {}",
                    e
                ))
            })?;

            Ok(JoystickInputNode {
                publisher: Hub::new(&self.topic)?,
                gilrs,
                device_id: 0,
                deadzone: 0.1,
                axis_invert_x: false,
                axis_invert_y: false,
                axis_invert_rx: false,
                axis_invert_ry: false,
                button_mapping: ButtonMapping::Generic,
                custom_button_names: HashMap::new(),
                custom_axis_names: HashMap::new(),
                axis_calibrations: HashMap::new(),
                per_axis_deadzones: HashMap::new(),
                processor: self.processor,
            })
        }

        #[cfg(not(feature = "gilrs"))]
        {
            Ok(JoystickInputNode {
                publisher: Hub::new(&self.topic)?,
                last_input_time: 0,
                device_id: 0,
                deadzone: 0.1,
                axis_invert_x: false,
                axis_invert_y: false,
                axis_invert_rx: false,
                axis_invert_ry: false,
                button_mapping: ButtonMapping::Generic,
                custom_button_names: HashMap::new(),
                custom_axis_names: HashMap::new(),
                axis_calibrations: HashMap::new(),
                per_axis_deadzones: HashMap::new(),
                processor: self.processor,
            })
        }
    }
}
