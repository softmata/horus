//! Simulation Dynamixel driver

use std::collections::HashMap;

use horus_core::driver::DriverStatus;
use horus_core::error::{HorusError, HorusResult};

use super::{DynamixelCommand, DynamixelConfig, DynamixelMode};

/// Simulated servo state
#[derive(Debug, Clone, Default)]
struct ServoState {
    position: f64, // Current position in degrees
    velocity: f64, // Current velocity in RPM
    target: f64,   // Target value
    mode: DynamixelMode,
    torque_enabled: bool,
    #[allow(dead_code)]
    temperature: f32, // Simulated temperature
    #[allow(dead_code)]
    voltage: f32, // Simulated voltage
    load: f32, // Simulated load percentage
}

/// Simulation Dynamixel driver
///
/// Simulates Dynamixel servo behavior without hardware.
pub struct SimulationDynamixelDriver {
    #[allow(dead_code)]
    config: DynamixelConfig,
    status: DriverStatus,
    /// Servo states by ID
    servos: HashMap<u8, ServoState>,
}

impl SimulationDynamixelDriver {
    pub fn new(config: DynamixelConfig) -> Self {
        let mut servos = HashMap::new();

        // Initialize servo states for configured IDs
        for &id in &config.servo_ids {
            servos.insert(
                id,
                ServoState {
                    position: 180.0, // Center position
                    velocity: 0.0,
                    target: 180.0,
                    mode: DynamixelMode::Position,
                    torque_enabled: false,
                    temperature: 30.0,
                    voltage: 12.0,
                    load: 0.0,
                },
            );
        }

        Self {
            config,
            status: DriverStatus::Uninitialized,
            servos,
        }
    }

    // ========================================================================
    // Lifecycle methods
    // ========================================================================

    pub fn init(&mut self) -> HorusResult<()> {
        // Reset all servos to initial state
        for servo in self.servos.values_mut() {
            servo.position = 180.0;
            servo.velocity = 0.0;
            servo.target = 180.0;
            servo.torque_enabled = false;
            servo.temperature = 30.0;
            servo.load = 0.0;
        }
        self.status = DriverStatus::Ready;
        Ok(())
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        // Disable torque on all servos
        for servo in self.servos.values_mut() {
            servo.torque_enabled = false;
            servo.velocity = 0.0;
        }
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

    pub fn write(&mut self, cmd: DynamixelCommand) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        let servo = self.servos.get_mut(&cmd.servo_id).ok_or_else(|| {
            HorusError::driver(format!("Servo ID {} not configured", cmd.servo_id))
        })?;

        servo.mode = cmd.mode;
        servo.target = cmd.target;
        servo.torque_enabled = cmd.torque_enable;

        Ok(())
    }

    pub fn stop(&mut self) -> HorusResult<()> {
        for servo in self.servos.values_mut() {
            servo.torque_enabled = false;
            servo.velocity = 0.0;
        }
        Ok(())
    }

    // ========================================================================
    // Query methods
    // ========================================================================

    /// Get current position for a servo (in degrees)
    pub fn get_position(&self, servo_id: u8) -> Option<f64> {
        self.servos.get(&servo_id).map(|s| s.position)
    }

    /// Get current velocity for a servo (in RPM)
    pub fn get_velocity(&self, servo_id: u8) -> Option<f64> {
        self.servos.get(&servo_id).map(|s| s.velocity)
    }

    /// Get simulated temperature for a servo
    pub fn get_temperature(&self, servo_id: u8) -> Option<f32> {
        self.servos.get(&servo_id).map(|s| s.temperature)
    }

    /// Check if torque is enabled for a servo
    pub fn is_torque_enabled(&self, servo_id: u8) -> bool {
        self.servos
            .get(&servo_id)
            .map(|s| s.torque_enabled)
            .unwrap_or(false)
    }

    /// Simulate a tick of motion (call periodically)
    pub fn simulate_tick(&mut self, dt: f64) {
        for servo in self.servos.values_mut() {
            if !servo.torque_enabled {
                continue;
            }

            match servo.mode {
                DynamixelMode::Position
                | DynamixelMode::ExtendedPosition
                | DynamixelMode::CurrentBasedPosition => {
                    // Move toward target position
                    let diff = servo.target - servo.position;
                    let max_speed = 60.0; // degrees per second (simulated)
                    let max_delta = max_speed * dt;

                    if diff.abs() <= max_delta {
                        servo.position = servo.target;
                        servo.velocity = 0.0;
                    } else {
                        servo.position += diff.signum() * max_delta;
                        servo.velocity = (diff.signum() * max_speed) / 6.0; // Convert to RPM approximation
                    }

                    // Simulate load based on movement
                    servo.load = (diff.abs() / 180.0 * 50.0).min(100.0) as f32;
                }
                DynamixelMode::Velocity => {
                    // Continuous rotation at target velocity
                    servo.velocity = servo.target;
                    servo.position += servo.velocity * 6.0 * dt; // RPM to degrees/s
                    servo.position = servo.position.rem_euclid(360.0);
                    servo.load = (servo.velocity.abs() / 100.0 * 30.0).min(100.0) as f32;
                }
                DynamixelMode::PWM => {
                    // PWM mode - direct duty cycle control
                    servo.velocity = servo.target * 100.0; // Approximate
                }
            }

            // Simulate temperature increase based on load
            servo.temperature = 30.0 + servo.load * 0.2;
        }
    }
}

impl Default for SimulationDynamixelDriver {
    fn default() -> Self {
        Self::new(DynamixelConfig::default())
    }
}
