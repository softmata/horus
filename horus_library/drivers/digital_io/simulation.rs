//! Simulation Digital I/O driver

use std::collections::HashMap;

use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

use super::{DigitalIoConfig, PinMode};

/// Simulation digital I/O driver
///
/// Simulates GPIO pins with in-memory state.
pub struct SimulationDigitalIoDriver {
    config: DigitalIoConfig,
    status: DriverStatus,
    /// Pin values (pin -> value)
    pin_values: HashMap<u64, bool>,
    /// Pin modes (pin -> mode)
    pin_modes: HashMap<u64, PinMode>,
}

impl SimulationDigitalIoDriver {
    pub fn new(config: DigitalIoConfig) -> Self {
        let mut pin_values = HashMap::new();
        let mut pin_modes = HashMap::new();

        // Initialize from config
        for pin_config in &config.pins {
            pin_modes.insert(pin_config.pin, pin_config.mode);
            pin_values.insert(pin_config.pin, pin_config.initial_value);
        }

        Self {
            config,
            status: DriverStatus::Uninitialized,
            pin_values,
            pin_modes,
        }
    }

    /// Set a simulated input value (for testing)
    pub fn set_input(&mut self, pin: u64, value: bool) {
        self.pin_values.insert(pin, value);
    }

    /// Get the current value of a pin
    pub fn get_value(&self, pin: u64) -> Option<bool> {
        self.pin_values.get(&pin).copied()
    }

    /// Get the mode of a pin
    pub fn get_mode(&self, pin: u64) -> Option<PinMode> {
        self.pin_modes.get(&pin).copied()
    }

    pub fn init(&mut self) -> HorusResult<()> {
        // Reset to initial values from config
        for pin_config in &self.config.pins {
            self.pin_modes.insert(pin_config.pin, pin_config.mode);
            self.pin_values
                .insert(pin_config.pin, pin_config.initial_value);
        }
        self.status = DriverStatus::Ready;
        Ok(())
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        // Set all outputs to low
        for (pin, mode) in &self.pin_modes {
            if *mode == PinMode::Output {
                self.pin_values.insert(*pin, false);
            }
        }
        self.status = DriverStatus::Shutdown;
        Ok(())
    }

    pub fn is_available(&self) -> bool {
        true
    }

    pub fn status(&self) -> DriverStatus {
        self.status.clone()
    }

    pub fn read_pin(&mut self, pin: u64) -> HorusResult<bool> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }
        self.status = DriverStatus::Running;

        // Get the pin config to check for inversion
        let inverted = self
            .config
            .pins
            .iter()
            .find(|p| p.pin == pin)
            .map(|p| p.inverted)
            .unwrap_or(false);

        let value = self.pin_values.get(&pin).copied().unwrap_or(false);
        Ok(if inverted { !value } else { value })
    }

    pub fn write_pin(&mut self, pin: u64, value: bool) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }
        self.status = DriverStatus::Running;

        // Check if pin is configured as output
        let mode = self.pin_modes.get(&pin).copied().unwrap_or(PinMode::Input);
        if mode != PinMode::Output {
            return Err(horus_core::error::HorusError::driver(format!(
                "Pin {} is not configured as output",
                pin
            )));
        }

        // Get the pin config to check for inversion
        let inverted = self
            .config
            .pins
            .iter()
            .find(|p| p.pin == pin)
            .map(|p| p.inverted)
            .unwrap_or(false);

        let actual_value = if inverted { !value } else { value };
        self.pin_values.insert(pin, actual_value);
        Ok(())
    }

    pub fn read_all(&mut self) -> HorusResult<Vec<(u64, bool)>> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }
        self.status = DriverStatus::Running;

        let mut results = Vec::new();
        for pin_config in &self.config.pins {
            if matches!(
                pin_config.mode,
                PinMode::Input | PinMode::InputPullUp | PinMode::InputPullDown
            ) {
                let value = self
                    .pin_values
                    .get(&pin_config.pin)
                    .copied()
                    .unwrap_or(false);
                let actual = if pin_config.inverted { !value } else { value };
                results.push((pin_config.pin, actual));
            }
        }
        Ok(results)
    }

    pub fn set_mode(&mut self, pin: u64, mode: PinMode) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }

        self.pin_modes.insert(pin, mode);

        // Initialize value based on mode
        self.pin_values
            .entry(pin)
            .or_insert_with(|| matches!(mode, PinMode::InputPullUp));

        Ok(())
    }
}

impl Default for SimulationDigitalIoDriver {
    fn default() -> Self {
        Self::new(DigitalIoConfig::default())
    }
}
