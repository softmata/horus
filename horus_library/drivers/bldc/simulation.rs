//! Simulation BLDC driver

use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

use super::{BldcCommand, BldcConfig, BldcProtocol};

/// Simulation BLDC motor driver
///
/// Simulates BLDC motor/ESC behavior without hardware.
pub struct SimulationBldcDriver {
    config: BldcConfig,
    status: DriverStatus,
    /// Current throttle per motor (0.0 to 1.0)
    current_throttle: [f64; 8],
    /// Current direction per motor
    current_direction: [bool; 8],
    /// Armed state per motor
    armed: [bool; 8],
    /// Simulated RPM per motor
    simulated_rpm: [f64; 8],
}

impl SimulationBldcDriver {
    pub fn new(config: BldcConfig) -> Self {
        Self {
            config,
            status: DriverStatus::Uninitialized,
            current_throttle: [0.0; 8],
            current_direction: [true; 8],
            armed: [false; 8],
            simulated_rpm: [0.0; 8],
        }
    }

    /// Get current throttle for a motor
    pub fn get_throttle(&self, motor_id: u8) -> Option<f64> {
        if motor_id < 8 {
            Some(self.current_throttle[motor_id as usize])
        } else {
            None
        }
    }

    /// Get simulated RPM for a motor
    pub fn get_rpm(&self, motor_id: u8) -> Option<f64> {
        if motor_id < 8 {
            Some(self.simulated_rpm[motor_id as usize])
        } else {
            None
        }
    }

    /// Check if motor is armed
    pub fn is_armed(&self, motor_id: u8) -> bool {
        if motor_id < 8 {
            self.armed[motor_id as usize]
        } else {
            false
        }
    }

    /// Get the protocol being used
    pub fn protocol(&self) -> BldcProtocol {
        self.config.protocol
    }

    /// Calculate PWM value from throttle
    #[allow(dead_code)]
    fn throttle_to_pwm(&self, throttle: f64) -> u16 {
        let range = (self.config.pwm_max_us - self.config.pwm_min_us) as f64;
        let pwm = self.config.pwm_min_us as f64 + (throttle * range);
        pwm as u16
    }

    pub fn init(&mut self) -> HorusResult<()> {
        // Reset all motors to stopped state
        for i in 0..8 {
            self.current_throttle[i] = 0.0;
            self.current_direction[i] = true;
            self.armed[i] = false;
            self.simulated_rpm[i] = 0.0;
        }
        self.status = DriverStatus::Ready;
        Ok(())
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        // Stop all motors
        for i in 0..8 {
            self.current_throttle[i] = 0.0;
            self.armed[i] = false;
            self.simulated_rpm[i] = 0.0;
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

    pub fn write(&mut self, cmd: BldcCommand) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }
        self.status = DriverStatus::Running;

        let idx = cmd.motor_id as usize;
        if idx >= self.config.num_motors as usize {
            return Err(horus_core::error::HorusError::driver(format!(
                "Invalid motor ID: {} (max: {})",
                cmd.motor_id,
                self.config.num_motors - 1
            )));
        }

        // Update state
        self.armed[idx] = cmd.armed;
        self.current_direction[idx] = cmd.direction;

        if cmd.armed {
            self.current_throttle[idx] = cmd.throttle;
            // Simulate RPM based on throttle (assuming max 10000 RPM)
            self.simulated_rpm[idx] = cmd.throttle * 10000.0;
        } else {
            self.current_throttle[idx] = 0.0;
            self.simulated_rpm[idx] = 0.0;
        }

        Ok(())
    }

    pub fn stop(&mut self) -> HorusResult<()> {
        for i in 0..self.config.num_motors as usize {
            self.current_throttle[i] = 0.0;
            self.armed[i] = false;
            self.simulated_rpm[i] = 0.0;
        }
        Ok(())
    }
}

impl Default for SimulationBldcDriver {
    fn default() -> Self {
        Self::new(BldcConfig::default())
    }
}
