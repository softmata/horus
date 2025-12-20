//! Simulation RoboClaw driver

use horus_core::driver::DriverStatus;
use horus_core::error::{HorusError, HorusResult};

use super::{RoboclawCommand, RoboclawConfig, RoboclawMode};

/// Motor channel state
#[derive(Debug, Clone, Default)]
struct MotorState {
    duty: f64,     // Current duty cycle (-1.0 to 1.0)
    velocity: f64, // Current velocity in counts/sec
    position: i64, // Current position in encoder counts
    target: f64,   // Target value
    mode: RoboclawMode,
    current: f32, // Simulated current draw (amps)
}

/// Simulation RoboClaw driver
///
/// Simulates RoboClaw motor controller behavior without hardware.
pub struct SimulationRoboclawDriver {
    #[allow(dead_code)]
    config: RoboclawConfig,
    status: DriverStatus,
    /// Motor 1 state
    motor1: MotorState,
    /// Motor 2 state
    motor2: MotorState,
    /// Simulated battery voltage
    battery_voltage: f32,
    /// Simulated temperature
    temperature: f32,
}

impl SimulationRoboclawDriver {
    pub fn new(config: RoboclawConfig) -> Self {
        Self {
            config,
            status: DriverStatus::Uninitialized,
            motor1: MotorState::default(),
            motor2: MotorState::default(),
            battery_voltage: 12.0,
            temperature: 25.0,
        }
    }

    // ========================================================================
    // Lifecycle methods
    // ========================================================================

    pub fn init(&mut self) -> HorusResult<()> {
        self.motor1 = MotorState::default();
        self.motor2 = MotorState::default();
        self.battery_voltage = 12.0;
        self.temperature = 25.0;
        self.status = DriverStatus::Ready;
        Ok(())
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        self.motor1.duty = 0.0;
        self.motor1.velocity = 0.0;
        self.motor2.duty = 0.0;
        self.motor2.velocity = 0.0;
        self.status = DriverStatus::Shutdown;
        Ok(())
    }

    pub fn is_available(&self) -> bool {
        true // Simulation is always available
    }

    pub fn status(&self) -> DriverStatus {
        self.status.clone()
    }

    // ========================================================================
    // Actuator methods
    // ========================================================================

    pub fn write(&mut self, cmd: RoboclawCommand) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        let motor = self.get_motor_mut(cmd.channel).ok_or_else(|| {
            HorusError::driver(format!(
                "Invalid motor channel: {} (use 1 or 2)",
                cmd.channel
            ))
        })?;

        motor.mode = cmd.mode;
        motor.target = cmd.target;

        // For duty mode, apply immediately
        if cmd.mode == RoboclawMode::Duty {
            motor.duty = cmd.target;
        }

        Ok(())
    }

    pub fn stop(&mut self) -> HorusResult<()> {
        self.motor1.duty = 0.0;
        self.motor1.velocity = 0.0;
        self.motor1.target = 0.0;
        self.motor2.duty = 0.0;
        self.motor2.velocity = 0.0;
        self.motor2.target = 0.0;
        Ok(())
    }

    // ========================================================================
    // Query methods
    // ========================================================================

    /// Get encoder position for a motor channel
    pub fn get_position(&self, channel: u8) -> Option<i64> {
        match channel {
            1 => Some(self.motor1.position),
            2 => Some(self.motor2.position),
            _ => None,
        }
    }

    /// Get velocity for a motor channel
    pub fn get_velocity(&self, channel: u8) -> Option<f64> {
        match channel {
            1 => Some(self.motor1.velocity),
            2 => Some(self.motor2.velocity),
            _ => None,
        }
    }

    /// Get current duty cycle for a motor channel
    pub fn get_duty(&self, channel: u8) -> Option<f64> {
        match channel {
            1 => Some(self.motor1.duty),
            2 => Some(self.motor2.duty),
            _ => None,
        }
    }

    /// Get battery voltage
    pub fn get_battery_voltage(&self) -> f32 {
        self.battery_voltage
    }

    /// Get temperature
    pub fn get_temperature(&self) -> f32 {
        self.temperature
    }

    /// Reset encoder position
    pub fn reset_encoder(&mut self, channel: u8) {
        match channel {
            1 => self.motor1.position = 0,
            2 => self.motor2.position = 0,
            _ => {}
        }
    }

    fn get_motor_mut(&mut self, channel: u8) -> Option<&mut MotorState> {
        match channel {
            1 => Some(&mut self.motor1),
            2 => Some(&mut self.motor2),
            _ => None,
        }
    }

    /// Simulate a tick of motion (call periodically)
    pub fn simulate_tick(&mut self, dt: f64) {
        for motor in [&mut self.motor1, &mut self.motor2] {
            match motor.mode {
                RoboclawMode::Duty => {
                    // Direct duty cycle - simulate velocity proportional to duty
                    let max_velocity = 5000.0; // counts/sec at full duty
                    motor.velocity = motor.duty * max_velocity;
                    motor.position += (motor.velocity * dt) as i64;
                }
                RoboclawMode::Velocity => {
                    // Velocity control - ramp toward target
                    let diff = motor.target - motor.velocity;
                    let max_accel = 10000.0; // counts/sec²
                    let max_delta = max_accel * dt;

                    if diff.abs() <= max_delta {
                        motor.velocity = motor.target;
                    } else {
                        motor.velocity += diff.signum() * max_delta;
                    }

                    motor.position += (motor.velocity * dt) as i64;
                    motor.duty = motor.velocity / 5000.0;
                }
                RoboclawMode::Position => {
                    // Position control - move toward target
                    let target_pos = motor.target as i64;
                    let diff = target_pos - motor.position;
                    let max_velocity = 3000.0; // counts/sec
                    let max_delta = (max_velocity * dt) as i64;

                    if diff.abs() <= max_delta {
                        motor.position = target_pos;
                        motor.velocity = 0.0;
                    } else {
                        motor.position += diff.signum() * max_delta;
                        motor.velocity = diff.signum() as f64 * max_velocity;
                    }

                    motor.duty = motor.velocity / 5000.0;
                }
            }

            // Simulate current draw based on duty
            motor.current = motor.duty.abs() as f32 * 10.0; // Up to 10A at full duty
        }

        // Simulate temperature increase based on current draw
        let total_current = self.motor1.current + self.motor2.current;
        self.temperature = 25.0 + total_current * 1.5;

        // Simulate battery sag
        self.battery_voltage = 12.6 - total_current * 0.05;
    }
}

impl Default for SimulationRoboclawDriver {
    fn default() -> Self {
        Self::new(RoboclawConfig::default())
    }
}
