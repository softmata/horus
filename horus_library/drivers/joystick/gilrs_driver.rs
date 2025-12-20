//! Gilrs-based Joystick driver
//!
//! Real gamepad input using the gilrs library.
//! Requires the `gilrs` feature to be enabled.

use std::collections::VecDeque;
use std::sync::Mutex;

use gilrs::{Axis, Button, Event, EventType, Gilrs, PowerInfo};

use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

use super::{AxisCalibration, ButtonMapping, JoystickConfig};
use crate::JoystickInput;
use std::collections::HashMap;

/// Gilrs driver configuration
#[derive(Debug, Clone, Default)]
pub struct GilrsConfig {
    /// Base joystick configuration
    pub joystick: JoystickConfig,
    /// Per-axis deadzones
    pub axis_deadzones: HashMap<u32, f32>,
    /// Axis calibrations
    pub axis_calibrations: HashMap<u32, AxisCalibration>,
}

/// Gilrs-based Joystick driver
///
/// Captures real gamepad input using the gilrs library.
/// The Gilrs instance is wrapped in a Mutex to satisfy Sync requirements.
pub struct GilrsJoystickDriver {
    config: GilrsConfig,
    status: DriverStatus,
    gilrs: Mutex<Gilrs>,
    event_queue: VecDeque<JoystickInput>,
}

impl GilrsJoystickDriver {
    /// Create a new gilrs joystick driver
    pub fn new() -> HorusResult<Self> {
        let gilrs = Gilrs::new().map_err(|e| {
            horus_core::error::HorusError::driver(format!("Failed to initialize gilrs: {}", e))
        })?;

        Ok(Self {
            config: GilrsConfig::default(),
            status: DriverStatus::Uninitialized,
            gilrs: Mutex::new(gilrs),
            event_queue: VecDeque::new(),
        })
    }

    /// Create with custom configuration
    pub fn with_config(config: GilrsConfig) -> HorusResult<Self> {
        let gilrs = Gilrs::new().map_err(|e| {
            horus_core::error::HorusError::driver(format!("Failed to initialize gilrs: {}", e))
        })?;

        Ok(Self {
            config,
            status: DriverStatus::Uninitialized,
            gilrs: Mutex::new(gilrs),
            event_queue: VecDeque::new(),
        })
    }

    /// Check if a controller is connected
    pub fn is_controller_connected(&self) -> bool {
        if let Ok(gilrs) = self.gilrs.lock() {
            gilrs.gamepads().any(|(_, gp)| gp.is_connected())
        } else {
            false
        }
    }

    /// Get battery level if supported
    pub fn get_battery_level(&self) -> Option<f32> {
        if let Ok(gilrs) = self.gilrs.lock() {
            if let Some((_, gamepad)) = gilrs.gamepads().find(|(_, gp)| gp.is_connected()) {
                let power = gamepad.power_info();
                return match power {
                    PowerInfo::Unknown => None,
                    PowerInfo::Wired => Some(1.0),
                    PowerInfo::Discharging(level) => Some(level as f32 / 100.0),
                    PowerInfo::Charging(level) => Some(level as f32 / 100.0),
                    PowerInfo::Charged => Some(1.0),
                };
            }
        }
        None
    }

    /// Poll for next event (non-blocking)
    pub fn poll_event(&mut self) -> Option<JoystickInput> {
        // Process pending gilrs events
        self.process_events();
        self.event_queue.pop_front()
    }

    /// Process gilrs events into the queue
    fn process_events(&mut self) {
        let Ok(mut gilrs) = self.gilrs.lock() else {
            return;
        };

        while let Some(Event { id, event, time: _ }) = gilrs.next_event() {
            let gamepad_id: u32 = usize::from(id) as u32;

            match event {
                EventType::ButtonPressed(button, _) => {
                    let button_id = button_to_id(button);
                    let button_name = self.get_button_name(button, button_id);

                    let input = JoystickInput::new_button(gamepad_id, button_id, button_name, true);
                    self.event_queue.push_back(input);
                }
                EventType::ButtonReleased(button, _) => {
                    let button_id = button_to_id(button);
                    let button_name = self.get_button_name(button, button_id);

                    let input =
                        JoystickInput::new_button(gamepad_id, button_id, button_name, false);
                    self.event_queue.push_back(input);
                }
                EventType::AxisChanged(axis, value, _) => {
                    let axis_id = axis_to_id(axis);
                    let axis_name = self.get_axis_name(axis, axis_id);

                    // Process axis value through calibration, deadzone, and inversion
                    let processed_value = self.process_axis_value(value, axis_id);

                    let input =
                        JoystickInput::new_axis(gamepad_id, axis_id, axis_name, processed_value);
                    self.event_queue.push_back(input);
                }
                EventType::Connected => {
                    let input = JoystickInput::new_connection(gamepad_id, true);
                    self.event_queue.push_back(input);
                }
                EventType::Disconnected => {
                    let input = JoystickInput::new_connection(gamepad_id, false);
                    self.event_queue.push_back(input);
                }
                _ => {}
            }
        }
    }

    /// Get deadzone for axis
    fn get_axis_deadzone(&self, axis_id: u32) -> f32 {
        self.config
            .axis_deadzones
            .get(&axis_id)
            .copied()
            .unwrap_or(self.config.joystick.deadzone)
    }

    /// Apply deadzone to an axis value
    fn apply_deadzone(&self, value: f32, axis_id: u32) -> f32 {
        let deadzone = self.get_axis_deadzone(axis_id);

        if value.abs() < deadzone {
            0.0
        } else {
            let sign = value.signum();
            let abs_value = value.abs();
            sign * (abs_value - deadzone) / (1.0 - deadzone)
        }
    }

    /// Apply calibration to an axis value
    fn apply_calibration(&self, value: f32, axis_id: u32) -> f32 {
        if let Some(cal) = self.config.axis_calibrations.get(&axis_id) {
            let centered = value - cal.center;

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
            0 => self.config.joystick.invert_x,
            1 => self.config.joystick.invert_y,
            3 => self.config.joystick.invert_rx,
            4 => self.config.joystick.invert_ry,
            _ => false,
        };

        if should_invert {
            -value
        } else {
            value
        }
    }

    /// Process axis value through all filters
    fn process_axis_value(&self, value: f32, axis_id: u32) -> f32 {
        let calibrated = self.apply_calibration(value, axis_id);
        let deadzone_applied = self.apply_deadzone(calibrated, axis_id);
        self.apply_inversion(deadzone_applied, axis_id)
    }

    /// Get button name based on mapping profile
    fn get_button_name(&self, button: Button, _button_id: u32) -> String {
        match self.config.joystick.button_mapping {
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

    /// Get axis name
    fn get_axis_name(&self, axis: Axis, _axis_id: u32) -> String {
        format!("{:?}", axis)
    }
}

// ========================================================================
// Lifecycle methods
// ========================================================================

impl GilrsJoystickDriver {
    pub fn init(&mut self) -> HorusResult<()> {
        self.event_queue.clear();
        self.status = DriverStatus::Ready;
        Ok(())
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        self.status = DriverStatus::Shutdown;
        Ok(())
    }

    pub fn is_available(&self) -> bool {
        true // gilrs initializes successfully if we get here
    }

    pub fn status(&self) -> DriverStatus {
        self.status.clone()
    }

    // ========================================================================
    // Sensor methods
    // ========================================================================

    pub fn read(&mut self) -> HorusResult<JoystickInput> {
        if self.status != DriverStatus::Ready && self.status != DriverStatus::Running {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }
        self.status = DriverStatus::Running;

        // Process events and return next one
        self.process_events();

        self.event_queue.pop_front().ok_or_else(|| {
            // Return a neutral event if no real events available
            horus_core::error::HorusError::driver("No joystick events available")
        })
    }

    pub fn has_data(&self) -> bool {
        !self.event_queue.is_empty()
            || matches!(self.status, DriverStatus::Ready | DriverStatus::Running)
    }

    pub fn sample_rate(&self) -> Option<f32> {
        Some(1000.0) // Poll-based, effectively unlimited
    }
}

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
