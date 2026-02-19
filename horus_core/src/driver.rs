//! Driver utilities for HORUS
//!
//! This module provides common types and configuration utilities for hardware drivers.
//! Drivers are standalone structs - no trait hierarchy required.
//!
//! # Philosophy
//!
//! HORUS drivers are simple structs with direct methods. No abstract traits, no
//! ceremony - just write the code you need. This keeps things simple and avoids
//! premature abstraction.
//!
//! # Example Driver
//!
//! ```rust,ignore
//! pub struct MyMotorDriver {
//!     port: String,
//!     status: DriverStatus,
//! }
//!
//! impl MyMotorDriver {
//!     pub fn new(port: &str) -> Result<Self, Error> { ... }
//!     pub fn set_speed(&mut self, speed: f64) -> Result<(), Error> { ... }
//!     pub fn stop(&mut self) -> Result<(), Error> { ... }
//! }
//! ```

use crate::error::{HorusError, HorusResult};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Driver status for lifecycle tracking
#[derive(Debug, Clone, PartialEq, Default)]
pub enum DriverStatus {
    /// Driver has not been initialized yet
    #[default]
    Uninitialized,
    /// Driver is ready to operate
    Ready,
    /// Driver is actively running/streaming
    Running,
    /// Driver encountered an error
    Error(String),
    /// Driver has been shut down
    Shutdown,
}

impl std::fmt::Display for DriverStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Uninitialized => write!(f, "Uninitialized"),
            Self::Ready => write!(f, "Ready"),
            Self::Running => write!(f, "Running"),
            Self::Error(msg) => write!(f, "Error: {}", msg),
            Self::Shutdown => write!(f, "Shutdown"),
        }
    }
}

/// Driver category for classification (informational only)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DriverCategory {
    /// Sensors (LiDAR, cameras, IMU, GPS, etc.)
    Sensor,
    /// Actuators (motors, servos, grippers, etc.)
    Actuator,
    /// Communication buses (I2C, SPI, CAN, Serial, etc.)
    Bus,
    /// Input devices (joystick, keyboard, etc.)
    Input,
    /// Simulation backends
    Simulation,
}

impl std::fmt::Display for DriverCategory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Sensor => write!(f, "Sensor"),
            Self::Actuator => write!(f, "Actuator"),
            Self::Bus => write!(f, "Bus"),
            Self::Input => write!(f, "Input"),
            Self::Simulation => write!(f, "Simulation"),
        }
    }
}

// ============================================================================
// Driver Configuration (YAML/TOML support)
// ============================================================================

/// Configuration for a single driver instance
///
/// # Example YAML
///
/// ```yaml
/// backend: rplidar
/// port: /dev/ttyUSB0
/// baud_rate: 115200
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleDriverConfig {
    /// Driver backend identifier (e.g., "simulation", "rplidar", "v4l2")
    pub backend: String,

    /// Enable/disable the driver (default: true)
    #[serde(default = "default_enabled")]
    pub enabled: bool,

    /// Serial port path (for serial devices)
    #[serde(default)]
    pub port: Option<String>,

    /// Baud rate for serial communication
    #[serde(default)]
    pub baud_rate: Option<u32>,

    /// I2C bus number
    #[serde(default)]
    pub i2c_bus: Option<u8>,

    /// I2C device address
    #[serde(default)]
    pub i2c_address: Option<u8>,

    /// Device path (for cameras, etc.)
    #[serde(default)]
    pub device: Option<String>,

    /// Width (for cameras)
    #[serde(default)]
    pub width: Option<u32>,

    /// Height (for cameras)
    #[serde(default)]
    pub height: Option<u32>,

    /// Frame rate / sample rate
    #[serde(default)]
    pub fps: Option<f32>,

    /// Additional driver-specific options
    #[serde(flatten)]
    pub options: std::collections::HashMap<String, serde_yaml::Value>,
}

fn default_enabled() -> bool {
    true
}

impl Default for SingleDriverConfig {
    fn default() -> Self {
        Self {
            backend: "simulation".to_string(),
            enabled: true,
            port: None,
            baud_rate: None,
            i2c_bus: None,
            i2c_address: None,
            device: None,
            width: None,
            height: None,
            fps: None,
            options: std::collections::HashMap::new(),
        }
    }
}

impl SingleDriverConfig {
    /// Create a new simulation driver config
    pub fn simulation() -> Self {
        Self {
            backend: "simulation".to_string(),
            ..Default::default()
        }
    }

    /// Create a new serial driver config
    pub fn serial(backend: &str, port: &str, baud_rate: u32) -> Self {
        Self {
            backend: backend.to_string(),
            port: Some(port.to_string()),
            baud_rate: Some(baud_rate),
            ..Default::default()
        }
    }

    /// Create a new I2C driver config
    pub fn i2c(backend: &str, bus: u8, address: u8) -> Self {
        Self {
            backend: backend.to_string(),
            i2c_bus: Some(bus),
            i2c_address: Some(address),
            ..Default::default()
        }
    }

    /// Check if this is a simulation backend
    pub fn is_simulation(&self) -> bool {
        self.backend == "simulation" || self.backend.starts_with("sim")
    }

}

/// Full driver configuration file with multiple driver definitions
///
/// # Example YAML
///
/// ```yaml
/// drivers:
///   lidar:
///     backend: rplidar
///     port: /dev/ttyUSB0
///     baud_rate: 115200
///
///   camera:
///     backend: v4l2
///     device: /dev/video0
///     width: 640
///     height: 480
///
///   imu:
///     backend: simulation
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DriversConfig {
    /// Map of driver name -> driver config
    pub drivers: std::collections::HashMap<String, SingleDriverConfig>,
}

impl DriversConfig {
    /// Create a new empty config
    pub fn new() -> Self {
        Self {
            drivers: std::collections::HashMap::new(),
        }
    }

    /// Load config from a file (auto-detect format)
    pub fn from_file<P: AsRef<Path>>(path: P) -> HorusResult<Self> {
        let path = path.as_ref();
        let contents = std::fs::read_to_string(path)
            .map_err(|e| HorusError::config(format!("Failed to read driver config: {}", e)))?;

        let extension = path.extension().and_then(|s| s.to_str());
        match extension {
            Some("toml") => Self::from_toml(&contents),
            Some("yaml") | Some("yml") => Self::from_yaml(&contents),
            _ => Self::from_yaml(&contents).or_else(|_| Self::from_toml(&contents)),
        }
    }

    /// Parse config from YAML string
    pub fn from_yaml(contents: &str) -> HorusResult<Self> {
        serde_yaml::from_str(contents)
            .map_err(|e| HorusError::config(format!("Failed to parse driver YAML: {}", e)))
    }

    /// Parse config from TOML string
    pub fn from_toml(contents: &str) -> HorusResult<Self> {
        toml::from_str(contents)
            .map_err(|e| HorusError::config(format!("Failed to parse driver TOML: {}", e)))
    }

    /// Get a driver config by name
    pub fn get_driver(&self, name: &str) -> HorusResult<&SingleDriverConfig> {
        self.drivers
            .get(name)
            .ok_or_else(|| HorusError::config(format!("Driver '{}' not found in config", name)))
    }

    /// Get a driver config by name, or return default simulation config
    pub fn get_driver_or_default(&self, name: &str) -> SingleDriverConfig {
        self.drivers
            .get(name)
            .cloned()
            .unwrap_or_else(SingleDriverConfig::simulation)
    }

    /// List all configured driver names
    pub fn list_drivers(&self) -> Vec<&str> {
        self.drivers.keys().map(|s| s.as_str()).collect()
    }

    /// Check if a driver is configured
    pub fn has_driver(&self, name: &str) -> bool {
        self.drivers.contains_key(name)
    }

    /// Get all enabled drivers
    pub fn enabled_drivers(&self) -> Vec<(&str, &SingleDriverConfig)> {
        self.drivers
            .iter()
            .filter(|(_, c)| c.enabled)
            .map(|(n, c)| (n.as_str(), c))
            .collect()
    }

    /// Save config to a file
    pub fn save<P: AsRef<Path>>(&self, path: P) -> HorusResult<()> {
        let path = path.as_ref();
        let extension = path.extension().and_then(|s| s.to_str());

        let contents = match extension {
            Some("toml") => toml::to_string_pretty(self)
                .map_err(|e| HorusError::config(format!("Failed to serialize TOML: {}", e)))?,
            _ => serde_yaml::to_string(self)
                .map_err(|e| HorusError::config(format!("Failed to serialize YAML: {}", e)))?,
        };

        std::fs::write(path, contents)
            .map_err(|e| HorusError::config(format!("Failed to write driver config: {}", e)))
    }
}

impl Default for DriversConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_driver_status_display() {
        assert_eq!(DriverStatus::Ready.to_string(), "Ready");
        assert_eq!(
            DriverStatus::Error("test".to_string()).to_string(),
            "Error: test"
        );
    }

    #[test]
    fn test_single_driver_config() {
        let config = SingleDriverConfig::simulation();
        assert_eq!(config.backend, "simulation");
        assert!(config.is_simulation());
        assert!(config.enabled);

        let serial = SingleDriverConfig::serial("rplidar", "/dev/ttyUSB0", 115200);
        assert_eq!(serial.backend, "rplidar");
        assert_eq!(serial.port, Some("/dev/ttyUSB0".to_string()));
        assert_eq!(serial.baud_rate, Some(115200));
        assert!(!serial.is_simulation());

        let i2c = SingleDriverConfig::i2c("mpu6050", 1, 0x68);
        assert_eq!(i2c.backend, "mpu6050");
        assert_eq!(i2c.i2c_bus, Some(1));
        assert_eq!(i2c.i2c_address, Some(0x68));
    }

    #[test]
    fn test_drivers_config_yaml() {
        let yaml = r#"
drivers:
  lidar:
    backend: rplidar
    port: /dev/ttyUSB0
    baud_rate: 115200
  camera:
    backend: v4l2
    device: /dev/video0
    width: 640
    height: 480
  imu:
    backend: simulation
"#;
        let config = DriversConfig::from_yaml(yaml).unwrap();

        let lidar = config.get_driver("lidar").unwrap();
        assert_eq!(lidar.backend, "rplidar");
        assert_eq!(lidar.port, Some("/dev/ttyUSB0".to_string()));
        assert_eq!(lidar.baud_rate, Some(115200));

        let camera = config.get_driver("camera").unwrap();
        assert_eq!(camera.backend, "v4l2");
        assert_eq!(camera.width, Some(640));
        assert_eq!(camera.height, Some(480));

        let imu = config.get_driver("imu").unwrap();
        assert!(imu.is_simulation());

        assert!(config.has_driver("lidar"));
        assert!(!config.has_driver("nonexistent"));
    }

    #[test]
    fn test_drivers_config_toml() {
        let toml = r#"
[drivers.lidar]
backend = "simulation"
port = "/dev/ttyUSB0"

[drivers.imu]
backend = "mpu6050"
i2c_bus = 1
i2c_address = 104
"#;
        let config = DriversConfig::from_toml(toml).unwrap();

        let lidar = config.get_driver("lidar").unwrap();
        assert_eq!(lidar.backend, "simulation");

        let imu = config.get_driver("imu").unwrap();
        assert_eq!(imu.i2c_bus, Some(1));
        assert_eq!(imu.i2c_address, Some(104));
    }

    #[test]
    fn test_drivers_config_default() {
        let config = DriversConfig::new();
        let default = config.get_driver_or_default("nonexistent");
        assert_eq!(default.backend, "simulation");
    }

    #[test]
    fn test_enabled_drivers() {
        let yaml = r#"
drivers:
  active:
    backend: simulation
    enabled: true
  inactive:
    backend: simulation
    enabled: false
"#;
        let config = DriversConfig::from_yaml(yaml).unwrap();
        let enabled = config.enabled_drivers();
        assert_eq!(enabled.len(), 1);
        assert_eq!(enabled[0].0, "active");
    }
}
