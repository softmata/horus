use horus_macros::LogSummary;
// Joystick/gamepad input message type for HORUS framework
//
// Provides a standardized way to represent gamepad and joystick events across all HORUS components.

use serde::{Deserialize, Serialize};

/// Standardized joystick/gamepad input message for cross-platform controller events.
///
/// This structure captures joystick and gamepad input events in a consistent format that can be
/// used across different input systems and platforms within the HORUS framework.
///
/// # Event Types
/// - `"button"` - Button press/release events
/// - `"axis"` - Analog stick and trigger movements  
/// - `"hat"` - D-pad directional input
/// - `"connection"` - Controller connect/disconnect events
///
/// # Examples
///
/// ```
/// use horus_library::JoystickInput;
///
/// // Create a button press event
/// let button_press = JoystickInput::new_button(0, 0, "A".to_string(), true);
/// assert!(button_press.is_button());
/// assert!(button_press.is_pressed());
///
/// // Create an axis movement event
/// let stick_move = JoystickInput::new_axis(0, 0, "LeftStickX".to_string(), 0.75);
/// assert!(stick_move.is_axis());
/// assert_eq!(stick_move.value, 0.75);
///
/// // Create a connection event
/// let connected = JoystickInput::new_connection(0, true);
/// assert!(connected.is_connection_event());
/// assert!(connected.is_connected());
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, LogSummary)]
pub struct JoystickInput {
    pub joystick_id: u32,       // Joystick/controller ID (0, 1, 2, 3...)
    pub event_type: [u8; 16],   // Fixed-size event type buffer (null-terminated)
    pub element_id: u32,        // Button/axis/hat number
    pub element_name: [u8; 32], // Fixed-size element name buffer (null-terminated)
    pub value: f32,             // Button: 0.0/1.0, Axis: -1.0 to 1.0, Hat: directional
    pub pressed: u8,            // For buttons: 1 when pressed, 0 when released
    pub timestamp_ms: u64,      // Unix timestamp_ms in milliseconds
}

impl JoystickInput {
    /// Core constructor — all public constructors delegate here.
    fn new_event(
        joystick_id: u32,
        event_type_str: &[u8],
        element_id: u32,
        name: &str,
        value: f32,
        pressed: u8,
    ) -> Self {
        let mut event_type = [0u8; 16];
        let len = event_type_str.len().min(15);
        event_type[..len].copy_from_slice(&event_type_str[..len]);

        let mut element_name = [0u8; 32];
        let name_bytes = name.as_bytes();
        let copy_len = name_bytes.len().min(31);
        element_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        Self {
            joystick_id,
            event_type,
            element_id,
            element_name,
            value,
            pressed,
            timestamp_ms: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64,
        }
    }

    pub fn new_button(
        joystick_id: u32,
        button_id: u32,
        button_name: String,
        pressed: bool,
    ) -> Self {
        Self::new_event(
            joystick_id,
            b"button",
            button_id,
            &button_name,
            if pressed { 1.0 } else { 0.0 },
            pressed as u8,
        )
    }

    pub fn new_axis(joystick_id: u32, axis_id: u32, axis_name: String, value: f32) -> Self {
        Self::new_event(joystick_id, b"axis", axis_id, &axis_name, value, 0)
    }

    pub fn new_hat(joystick_id: u32, hat_id: u32, hat_name: String, value: f32) -> Self {
        Self::new_event(joystick_id, b"hat", hat_id, &hat_name, value, 0)
    }

    pub fn new_connection(joystick_id: u32, connected: bool) -> Self {
        Self::new_event(
            joystick_id,
            b"connection",
            0,
            if connected {
                "Connected"
            } else {
                "Disconnected"
            },
            if connected { 1.0 } else { 0.0 },
            0,
        )
    }

    /// Get the event type as a String
    pub fn event_type(&self) -> String {
        let end = self
            .event_type
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.event_type.len());
        String::from_utf8_lossy(&self.event_type[..end])
            .into_owned()
            .to_string()
    }

    /// Get the element name as a String
    pub fn element_name(&self) -> String {
        let end = self
            .element_name
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.element_name.len());
        String::from_utf8_lossy(&self.element_name[..end])
            .into_owned()
            .to_string()
    }

    pub fn is_button(&self) -> bool {
        self.event_type() == "button"
    }

    pub fn is_axis(&self) -> bool {
        self.event_type() == "axis"
    }

    pub fn is_hat(&self) -> bool {
        self.event_type() == "hat"
    }

    pub fn is_connection_event(&self) -> bool {
        self.event_type() == "connection"
    }

    pub fn is_connected(&self) -> bool {
        self.is_connection_event() && self.value > 0.0
    }

    /// Check if button is pressed
    pub fn is_pressed(&self) -> bool {
        self.pressed != 0
    }
}

// =============================================================================
// POD (Plain Old Data) Message Support
// =============================================================================

crate::messages::impl_pod_message!(JoystickInput);
