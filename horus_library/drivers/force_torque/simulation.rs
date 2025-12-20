//! Simulation Force/Torque driver
//!
//! Always-available simulation driver that generates synthetic force/torque data.
//! Useful for testing and development without hardware.

use std::time::{SystemTime, UNIX_EPOCH};

use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

use super::FtSensorModel;
use crate::{Point3, Vector3, WrenchStamped};

/// Simulation force/torque driver configuration
#[derive(Debug, Clone)]
pub struct SimulationFtConfig {
    /// Sensor model for range specifications
    pub sensor_model: FtSensorModel,
    /// Sample rate in Hz
    pub sample_rate: f32,
    /// Tool mass for gravity simulation (kg)
    pub tool_mass: f64,
    /// Tool center of mass offset [x, y, z] in meters
    pub tool_com: [f64; 3],
    /// Gravity vector [gx, gy, gz] in m/s²
    pub gravity_vector: [f64; 3],
    /// Noise amplitude (fraction of range)
    pub noise_amplitude: f64,
    /// Frame ID for output messages
    pub frame_id: String,
}

impl Default for SimulationFtConfig {
    fn default() -> Self {
        Self {
            sensor_model: FtSensorModel::Generic,
            sample_rate: 1000.0, // 1kHz
            tool_mass: 0.0,
            tool_com: [0.0, 0.0, 0.0],
            gravity_vector: [0.0, 0.0, -9.81], // Z-up frame
            noise_amplitude: 0.001,            // 0.1% of range
            frame_id: "ft_sensor".to_string(),
        }
    }
}

/// Simulation Force/Torque driver
///
/// Generates synthetic force/torque data with configurable noise
/// and optional tool gravity compensation simulation.
pub struct SimulationForceTorqueDriver {
    config: SimulationFtConfig,
    status: DriverStatus,
    measurement_count: u64,
    last_read_time: u64,

    // Force and torque ranges
    force_range: [f64; 3],
    torque_range: [f64; 3],
}

impl SimulationForceTorqueDriver {
    /// Create a new simulation force/torque driver with default configuration
    pub fn new() -> Self {
        let (force_range, torque_range) = FtSensorModel::Generic.get_ranges();
        Self {
            config: SimulationFtConfig::default(),
            status: DriverStatus::Uninitialized,
            measurement_count: 0,
            last_read_time: 0,
            force_range,
            torque_range,
        }
    }

    /// Create a new simulation driver with custom configuration
    pub fn with_config(config: SimulationFtConfig) -> Self {
        let (force_range, torque_range) = config.sensor_model.get_ranges();
        Self {
            config,
            status: DriverStatus::Uninitialized,
            measurement_count: 0,
            last_read_time: 0,
            force_range,
            torque_range,
        }
    }

    /// Set sensor model
    pub fn set_sensor_model(&mut self, model: FtSensorModel) {
        self.config.sensor_model = model;
        let (force_range, torque_range) = model.get_ranges();
        self.force_range = force_range;
        self.torque_range = torque_range;
    }

    /// Set tool mass and center of mass
    pub fn set_tool(&mut self, mass: f64, com_x: f64, com_y: f64, com_z: f64) {
        self.config.tool_mass = mass;
        self.config.tool_com = [com_x, com_y, com_z];
    }

    /// Get the current timestamp in nanoseconds
    fn now_nanos(&self) -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }

    /// Simple pseudo-random number generator
    fn pseudo_random(&self) -> f64 {
        let seed = self
            .measurement_count
            .wrapping_mul(1103515245)
            .wrapping_add(12345);
        let time_seed = self
            .last_read_time
            .wrapping_mul(214013)
            .wrapping_add(2531011);
        let combined = seed.wrapping_add(time_seed);
        ((combined % 1000) as f64 / 1000.0) - 0.5
    }

    /// Initialize the driver
    pub fn init(&mut self) -> HorusResult<()> {
        self.measurement_count = 0;
        self.last_read_time = 0;
        self.status = DriverStatus::Ready;
        Ok(())
    }

    /// Shutdown the driver
    pub fn shutdown(&mut self) -> HorusResult<()> {
        self.status = DriverStatus::Shutdown;
        Ok(())
    }

    /// Check if driver is available
    pub fn is_available(&self) -> bool {
        true // Simulation is always available
    }

    /// Get driver status
    pub fn status(&self) -> DriverStatus {
        self.status.clone()
    }

    /// Read force/torque data
    pub fn read(&mut self) -> HorusResult<WrenchStamped> {
        if self.status != DriverStatus::Ready && self.status != DriverStatus::Running {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }
        self.status = DriverStatus::Running;
        Ok(self.generate_reading())
    }

    /// Check if data is available
    pub fn has_data(&self) -> bool {
        matches!(self.status, DriverStatus::Ready | DriverStatus::Running)
    }

    /// Get sample rate
    pub fn sample_rate(&self) -> Option<f32> {
        Some(self.config.sample_rate)
    }

    /// Generate a simulated reading
    fn generate_reading(&mut self) -> WrenchStamped {
        self.measurement_count += 1;
        let timestamp = self.now_nanos();
        self.last_read_time = timestamp;

        // Calculate tool gravity effects
        let mut force = [0.0f64; 3];
        let mut torque = [0.0f64; 3];

        if self.config.tool_mass > 0.0 {
            // Force from tool weight
            force[0] = self.config.tool_mass * self.config.gravity_vector[0];
            force[1] = self.config.tool_mass * self.config.gravity_vector[1];
            force[2] = self.config.tool_mass * self.config.gravity_vector[2];

            // Torque from tool center of mass offset (r × F)
            let com = &self.config.tool_com;
            torque[0] = com[1] * force[2] - com[2] * force[1];
            torque[1] = com[2] * force[0] - com[0] * force[2];
            torque[2] = com[0] * force[1] - com[1] * force[0];
        }

        // Add noise
        for i in 0..3 {
            force[i] += self.pseudo_random() * self.force_range[i] * self.config.noise_amplitude;
            torque[i] += self.pseudo_random() * self.torque_range[i] * self.config.noise_amplitude;
        }

        // Create wrench message
        let mut wrench = WrenchStamped {
            force: Vector3 {
                x: force[0],
                y: force[1],
                z: force[2],
            },
            torque: Vector3 {
                x: torque[0],
                y: torque[1],
                z: torque[2],
            },
            point_of_application: Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            frame_id: [0; 32],
            timestamp,
        };

        // Set frame_id
        let frame_bytes = self.config.frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        wrench.frame_id[..len].copy_from_slice(&frame_bytes[..len]);

        wrench
    }
}

impl Default for SimulationForceTorqueDriver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_driver_lifecycle() {
        let mut driver = SimulationForceTorqueDriver::new();

        assert_eq!(driver.status(), DriverStatus::Uninitialized);
        assert!(driver.is_available());

        driver.init().unwrap();
        assert_eq!(driver.status(), DriverStatus::Ready);

        let wrench = driver.read().unwrap();
        assert_eq!(driver.status(), DriverStatus::Running);

        // Check wrench data exists
        assert!(wrench.timestamp > 0);

        driver.shutdown().unwrap();
        assert_eq!(driver.status(), DriverStatus::Shutdown);
    }

    #[test]
    fn test_tool_gravity_simulation() {
        let config = SimulationFtConfig {
            tool_mass: 1.0,       // 1kg tool
            noise_amplitude: 0.0, // No noise for predictable testing
            ..Default::default()
        };

        let mut driver = SimulationForceTorqueDriver::with_config(config);
        driver.init().unwrap();

        let wrench = driver.read().unwrap();

        // Should see ~-9.81N in Z direction (gravity)
        assert!(wrench.force.z < -9.0);
    }
}
