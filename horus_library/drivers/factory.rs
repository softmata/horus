//! Driver Factory - Create drivers from configuration
//!
//! This module provides factory functions to create drivers at runtime
//! based on configuration. This enables YAML-based driver configuration
//! without compile-time feature selection.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::driver::SingleDriverConfig;
//! use horus_library::drivers::factory::create_imu_driver;
//!
//! // From config file
//! let config = SingleDriverConfig::serial("mpu6050", "/dev/i2c-1", 0x68);
//! let driver = create_imu_driver(&config)?;
//!
//! // Use in node
//! let node = ImuNode::with_driver("imu", driver)?;
//! ```

use horus_core::driver::SingleDriverConfig;
use horus_core::error::{HorusError, HorusResult};

use super::{
    BatteryDriver, CameraDriver, EncoderDriver, ForceTorqueDriver, GpsDriver, ImuDriver,
    JoystickDriver, KeyboardDriver, LidarDriver, MotorDriver, ServoDriver, UltrasonicDriver,
};

use super::battery::BatteryDriverBackend;
use super::camera::CameraDriverBackend;
use super::encoder::EncoderDriverBackend;
use super::force_torque::ForceTorqueDriverBackend;
use super::gps::GpsDriverBackend;
use super::imu::ImuDriverBackend;
use super::joystick::JoystickDriverBackend;
use super::keyboard::KeyboardDriverBackend;
use super::lidar::LidarDriverBackend;
use super::motor::MotorDriverBackend;
use super::servo::ServoDriverBackend;
use super::ultrasonic::UltrasonicDriverBackend;

// ============================================================================
// IMU Driver Factory
// ============================================================================

/// Create an IMU driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic data
/// - `mpu6050` - MPU6050 I2C IMU (requires `mpu6050-imu` feature)
/// - `bno055` - BNO055 I2C IMU (requires `bno055-imu` feature)
pub fn create_imu_driver(config: &SingleDriverConfig) -> HorusResult<ImuDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => ImuDriverBackend::Simulation,

        #[cfg(feature = "mpu6050-imu")]
        "mpu6050" => ImuDriverBackend::Mpu6050,

        #[cfg(feature = "bno055-imu")]
        "bno055" => ImuDriverBackend::Bno055,

        other => {
            return Err(HorusError::driver(format!(
                "IMU backend '{}' is not available. Available: simulation{}{}",
                other,
                if cfg!(feature = "mpu6050-imu") {
                    ", mpu6050"
                } else {
                    ""
                },
                if cfg!(feature = "bno055-imu") {
                    ", bno055"
                } else {
                    ""
                },
            )));
        }
    };

    ImuDriver::new(backend)
}

// ============================================================================
// Camera Driver Factory
// ============================================================================

/// Create a Camera driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic images
/// - `opencv` - OpenCV-based camera (requires `opencv-backend` feature)
/// - `v4l2` - Video4Linux2 camera (requires `v4l2-backend` feature)
pub fn create_camera_driver(config: &SingleDriverConfig) -> HorusResult<CameraDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => CameraDriverBackend::Simulation,

        #[cfg(feature = "opencv-backend")]
        "opencv" => CameraDriverBackend::OpenCv,

        #[cfg(feature = "v4l2-backend")]
        "v4l2" => CameraDriverBackend::V4l2,

        other => {
            return Err(HorusError::driver(format!(
                "Camera backend '{}' is not available. Available: simulation{}{}",
                other,
                if cfg!(feature = "opencv-backend") {
                    ", opencv"
                } else {
                    ""
                },
                if cfg!(feature = "v4l2-backend") {
                    ", v4l2"
                } else {
                    ""
                },
            )));
        }
    };

    CameraDriver::new(backend)
}

// ============================================================================
// LiDAR Driver Factory
// ============================================================================

/// Create a LiDAR driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic scans
/// - `rplidar` - Slamtec RPLidar (requires `rplidar` feature)
pub fn create_lidar_driver(config: &SingleDriverConfig) -> HorusResult<LidarDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => LidarDriverBackend::Simulation,

        #[cfg(feature = "rplidar")]
        "rplidar" => LidarDriverBackend::Rplidar,

        other => {
            return Err(HorusError::driver(format!(
                "LiDAR backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "rplidar") {
                    ", rplidar"
                } else {
                    ""
                },
            )));
        }
    };

    LidarDriver::new(backend)
}

// ============================================================================
// GPS Driver Factory
// ============================================================================

/// Create a GPS driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic positions
/// - `nmea` - NMEA serial GPS (requires `nmea-gps` feature)
pub fn create_gps_driver(config: &SingleDriverConfig) -> HorusResult<GpsDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => GpsDriverBackend::Simulation,

        #[cfg(feature = "nmea-gps")]
        "nmea" => GpsDriverBackend::Nmea,

        other => {
            return Err(HorusError::driver(format!(
                "GPS backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "nmea-gps") {
                    ", nmea"
                } else {
                    ""
                },
            )));
        }
    };

    GpsDriver::new(backend)
}

// ============================================================================
// Encoder Driver Factory
// ============================================================================

/// Create an Encoder driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic odometry
/// - `gpio` - GPIO quadrature encoder (requires `gpio-hardware` feature)
pub fn create_encoder_driver(config: &SingleDriverConfig) -> HorusResult<EncoderDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => EncoderDriverBackend::Simulation,

        #[cfg(feature = "gpio-hardware")]
        "gpio" => EncoderDriverBackend::Gpio,

        other => {
            return Err(HorusError::driver(format!(
                "Encoder backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "gpio-hardware") {
                    ", gpio"
                } else {
                    ""
                },
            )));
        }
    };

    EncoderDriver::new(backend)
}

// ============================================================================
// Motor Driver Factory
// ============================================================================

/// Create a Motor driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, simulates motor behavior
/// - `gpio` - GPIO PWM motor (requires `gpio-hardware` feature)
pub fn create_motor_driver(config: &SingleDriverConfig) -> HorusResult<MotorDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => MotorDriverBackend::Simulation,

        #[cfg(feature = "gpio-hardware")]
        "gpio" => MotorDriverBackend::Gpio,

        other => {
            return Err(HorusError::driver(format!(
                "Motor backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "gpio-hardware") {
                    ", gpio"
                } else {
                    ""
                },
            )));
        }
    };

    MotorDriver::new(backend)
}

// ============================================================================
// Servo Driver Factory
// ============================================================================

/// Create a Servo driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, simulates servo behavior
/// - `pca9685` - PCA9685 I2C PWM controller (requires `i2c-hardware` feature)
pub fn create_servo_driver(config: &SingleDriverConfig) -> HorusResult<ServoDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => ServoDriverBackend::Simulation,

        #[cfg(feature = "i2c-hardware")]
        "pca9685" => ServoDriverBackend::Pca9685,

        other => {
            return Err(HorusError::driver(format!(
                "Servo backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "i2c-hardware") {
                    ", pca9685"
                } else {
                    ""
                },
            )));
        }
    };

    ServoDriver::new(backend)
}

// ============================================================================
// Ultrasonic Driver Factory
// ============================================================================

/// Create an Ultrasonic driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic range data
/// - `gpio` - GPIO echo/trigger driver (requires `gpio-hardware` feature)
pub fn create_ultrasonic_driver(config: &SingleDriverConfig) -> HorusResult<UltrasonicDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => UltrasonicDriverBackend::Simulation,

        #[cfg(feature = "gpio-hardware")]
        "gpio" => UltrasonicDriverBackend::Gpio,

        other => {
            return Err(HorusError::driver(format!(
                "Ultrasonic backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "gpio-hardware") {
                    ", gpio"
                } else {
                    ""
                },
            )));
        }
    };

    UltrasonicDriver::new(backend)
}

// ============================================================================
// Battery Driver Factory
// ============================================================================

/// Create a Battery driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic battery data
/// - `i2c` - I2C power monitor (INA219/INA226, requires `i2c-hardware` feature)
pub fn create_battery_driver(config: &SingleDriverConfig) -> HorusResult<BatteryDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => BatteryDriverBackend::Simulation,

        #[cfg(feature = "i2c-hardware")]
        "i2c" | "ina219" | "ina226" => BatteryDriverBackend::I2c,

        other => {
            return Err(HorusError::driver(format!(
                "Battery backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "i2c-hardware") {
                    ", i2c"
                } else {
                    ""
                },
            )));
        }
    };

    BatteryDriver::new(backend)
}

// ============================================================================
// Force/Torque Driver Factory
// ============================================================================

/// Create a Force/Torque driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic force/torque data
/// - `ati_netft` - ATI NetFT Ethernet sensor (requires `netft` feature)
/// - `robotiq` - Robotiq FT-300 serial sensor (requires `robotiq-serial` feature)
pub fn create_force_torque_driver(config: &SingleDriverConfig) -> HorusResult<ForceTorqueDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => ForceTorqueDriverBackend::Simulation,

        #[cfg(feature = "netft")]
        "ati_netft" | "netft" | "ati" => ForceTorqueDriverBackend::AtiNetFt,

        #[cfg(feature = "robotiq-serial")]
        "robotiq" | "robotiq_serial" | "ft300" => ForceTorqueDriverBackend::RobotiqSerial,

        other => {
            return Err(HorusError::driver(format!(
                "Force/Torque backend '{}' is not available. Available: simulation{}{}",
                other,
                if cfg!(feature = "netft") {
                    ", ati_netft"
                } else {
                    ""
                },
                if cfg!(feature = "robotiq-serial") {
                    ", robotiq"
                } else {
                    ""
                },
            )));
        }
    };

    ForceTorqueDriver::new(backend)
}

// ============================================================================
// Joystick Driver Factory
// ============================================================================

/// Create a Joystick driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic input
/// - `gilrs` - Real gamepad input via gilrs (requires `gilrs` feature)
pub fn create_joystick_driver(config: &SingleDriverConfig) -> HorusResult<JoystickDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => JoystickDriverBackend::Simulation,

        #[cfg(feature = "gilrs")]
        "gilrs" | "gamepad" => JoystickDriverBackend::Gilrs,

        other => {
            return Err(HorusError::driver(format!(
                "Joystick backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "gilrs") {
                    ", gilrs"
                } else {
                    ""
                },
            )));
        }
    };

    JoystickDriver::new(backend)
}

// ============================================================================
// Keyboard Driver Factory
// ============================================================================

/// Create a Keyboard driver from configuration
///
/// # Supported Backends
///
/// - `simulation` - Always available, generates synthetic input
/// - `crossterm` - Terminal keyboard input (requires `crossterm` feature)
pub fn create_keyboard_driver(config: &SingleDriverConfig) -> HorusResult<KeyboardDriver> {
    let backend = match config.backend.as_str() {
        "simulation" | "sim" => KeyboardDriverBackend::Simulation,

        #[cfg(feature = "crossterm")]
        "crossterm" | "terminal" => KeyboardDriverBackend::Crossterm,

        other => {
            return Err(HorusError::driver(format!(
                "Keyboard backend '{}' is not available. Available: simulation{}",
                other,
                if cfg!(feature = "crossterm") {
                    ", crossterm"
                } else {
                    ""
                },
            )));
        }
    };

    KeyboardDriver::new(backend)
}

// ============================================================================
// Convenience: Create all drivers from DriversConfig
// ============================================================================

use horus_core::driver::DriversConfig;

/// Result of creating all drivers from a config file
#[derive(Default)]
pub struct CreatedDrivers {
    pub imu: Option<ImuDriver>,
    pub camera: Option<CameraDriver>,
    pub lidar: Option<LidarDriver>,
    pub gps: Option<GpsDriver>,
    pub encoder: Option<EncoderDriver>,
    pub motor: Option<MotorDriver>,
    pub servo: Option<ServoDriver>,
    pub ultrasonic: Option<UltrasonicDriver>,
    pub battery: Option<BatteryDriver>,
    pub force_torque: Option<ForceTorqueDriver>,
    pub joystick: Option<JoystickDriver>,
    pub keyboard: Option<KeyboardDriver>,
}

/// Create all drivers from a DriversConfig
///
/// This function reads a DriversConfig and creates the appropriate drivers
/// for each configured device.
///
/// # Example
///
/// ```rust,ignore
/// let config = DriversConfig::from_file("drivers.yaml")?;
/// let drivers = create_drivers_from_config(&config)?;
///
/// if let Some(imu) = drivers.imu {
///     let node = ImuNode::with_driver("imu", imu)?;
/// }
/// ```
pub fn create_drivers_from_config(config: &DriversConfig) -> HorusResult<CreatedDrivers> {
    let mut drivers = CreatedDrivers::default();

    for (name, driver_config) in &config.drivers {
        if !driver_config.enabled {
            continue;
        }

        match name.as_str() {
            "imu" => {
                drivers.imu = Some(create_imu_driver(driver_config)?);
            }
            "camera" => {
                drivers.camera = Some(create_camera_driver(driver_config)?);
            }
            "lidar" => {
                drivers.lidar = Some(create_lidar_driver(driver_config)?);
            }
            "gps" => {
                drivers.gps = Some(create_gps_driver(driver_config)?);
            }
            "encoder" => {
                drivers.encoder = Some(create_encoder_driver(driver_config)?);
            }
            "motor" => {
                drivers.motor = Some(create_motor_driver(driver_config)?);
            }
            "servo" => {
                drivers.servo = Some(create_servo_driver(driver_config)?);
            }
            "ultrasonic" => {
                drivers.ultrasonic = Some(create_ultrasonic_driver(driver_config)?);
            }
            "battery" => {
                drivers.battery = Some(create_battery_driver(driver_config)?);
            }
            "force_torque" | "ft" => {
                drivers.force_torque = Some(create_force_torque_driver(driver_config)?);
            }
            "joystick" | "gamepad" => {
                drivers.joystick = Some(create_joystick_driver(driver_config)?);
            }
            "keyboard" => {
                drivers.keyboard = Some(create_keyboard_driver(driver_config)?);
            }
            _ => {
                // Unknown driver name, skip (could log a warning here)
            }
        }
    }

    Ok(drivers)
}

/// List available backends for each driver type
#[allow(unused_mut)]
pub fn list_available_backends() -> std::collections::HashMap<&'static str, Vec<&'static str>> {
    let mut backends = std::collections::HashMap::new();

    // IMU backends
    let mut imu_backends = vec!["simulation"];
    #[cfg(feature = "mpu6050-imu")]
    imu_backends.push("mpu6050");
    #[cfg(feature = "bno055-imu")]
    imu_backends.push("bno055");
    backends.insert("imu", imu_backends);

    // Camera backends
    let mut camera_backends = vec!["simulation"];
    #[cfg(feature = "opencv-backend")]
    camera_backends.push("opencv");
    #[cfg(feature = "v4l2-backend")]
    camera_backends.push("v4l2");
    backends.insert("camera", camera_backends);

    // LiDAR backends
    let mut lidar_backends = vec!["simulation"];
    #[cfg(feature = "rplidar")]
    lidar_backends.push("rplidar");
    backends.insert("lidar", lidar_backends);

    // GPS backends
    let mut gps_backends = vec!["simulation"];
    #[cfg(feature = "nmea-gps")]
    gps_backends.push("nmea");
    backends.insert("gps", gps_backends);

    // Encoder backends
    let mut encoder_backends = vec!["simulation"];
    #[cfg(feature = "gpio-hardware")]
    encoder_backends.push("gpio");
    backends.insert("encoder", encoder_backends);

    // Motor backends
    let mut motor_backends = vec!["simulation"];
    #[cfg(feature = "gpio-hardware")]
    motor_backends.push("gpio");
    backends.insert("motor", motor_backends);

    // Servo backends
    let mut servo_backends = vec!["simulation"];
    #[cfg(feature = "i2c-hardware")]
    servo_backends.push("pca9685");
    backends.insert("servo", servo_backends);

    // Ultrasonic backends
    let mut ultrasonic_backends = vec!["simulation"];
    #[cfg(feature = "gpio-hardware")]
    ultrasonic_backends.push("gpio");
    backends.insert("ultrasonic", ultrasonic_backends);

    // Battery backends
    let mut battery_backends = vec!["simulation"];
    #[cfg(feature = "i2c-hardware")]
    battery_backends.push("i2c");
    backends.insert("battery", battery_backends);

    // Force/Torque backends
    let mut ft_backends = vec!["simulation"];
    #[cfg(feature = "netft")]
    ft_backends.push("ati_netft");
    #[cfg(feature = "robotiq-serial")]
    ft_backends.push("robotiq");
    backends.insert("force_torque", ft_backends);

    // Joystick backends
    let mut joystick_backends = vec!["simulation"];
    #[cfg(feature = "gilrs")]
    joystick_backends.push("gilrs");
    backends.insert("joystick", joystick_backends);

    // Keyboard backends
    let mut keyboard_backends = vec!["simulation"];
    #[cfg(feature = "crossterm")]
    keyboard_backends.push("crossterm");
    backends.insert("keyboard", keyboard_backends);

    backends
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_simulation_imu() {
        let config = SingleDriverConfig::simulation();
        let driver = create_imu_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_camera() {
        let config = SingleDriverConfig::simulation();
        let driver = create_camera_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_lidar() {
        let config = SingleDriverConfig::simulation();
        let driver = create_lidar_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_gps() {
        let config = SingleDriverConfig::simulation();
        let driver = create_gps_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_encoder() {
        let config = SingleDriverConfig::simulation();
        let driver = create_encoder_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_motor() {
        let config = SingleDriverConfig::simulation();
        let driver = create_motor_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_servo() {
        let config = SingleDriverConfig::simulation();
        let driver = create_servo_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_ultrasonic() {
        let config = SingleDriverConfig::simulation();
        let driver = create_ultrasonic_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_create_simulation_battery() {
        let config = SingleDriverConfig::simulation();
        let driver = create_battery_driver(&config).unwrap();
        assert!(driver.is_available());
    }

    #[test]
    fn test_unknown_backend_error() {
        let config = SingleDriverConfig {
            backend: "nonexistent".to_string(),
            ..Default::default()
        };
        assert!(create_imu_driver(&config).is_err());
        assert!(create_camera_driver(&config).is_err());
        assert!(create_lidar_driver(&config).is_err());
    }

    #[test]
    fn test_list_available_backends() {
        let backends = list_available_backends();
        assert!(backends.get("imu").unwrap().contains(&"simulation"));
        assert!(backends.get("camera").unwrap().contains(&"simulation"));
        assert!(backends.get("lidar").unwrap().contains(&"simulation"));
    }

    #[test]
    fn test_create_drivers_from_config() {
        let yaml = r#"
drivers:
  imu:
    backend: simulation
  camera:
    backend: simulation
    enabled: false
"#;
        let config = DriversConfig::from_yaml(yaml).unwrap();
        let drivers = create_drivers_from_config(&config).unwrap();

        assert!(drivers.imu.is_some());
        assert!(drivers.camera.is_none()); // disabled
        assert!(drivers.lidar.is_none()); // not configured
    }
}
