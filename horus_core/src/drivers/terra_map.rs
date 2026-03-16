//! Terra driver name → crate/feature mapping.
//!
//! Maps short names like `"dynamixel"` used in `[drivers]` config to the
//! Terra crate and feature that provides the implementation.

/// Information about a Terra crate for a given driver shortname.
#[derive(Debug, Clone)]
pub struct TerraCrateInfo {
    /// Crate name (e.g., `"terra-serial"`).
    pub crate_name: &'static str,
    /// Crate version constraint.
    pub version: &'static str,
    /// Feature to enable (e.g., `"dynamixel"`), or None if no feature needed.
    pub feature: Option<&'static str>,
    /// The bus type category for typed accessor selection.
    pub accessor: DriverAccessor,
}

/// Which typed accessor method this driver maps to on `HardwareSet`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverAccessor {
    Dynamixel,
    RpLidar,
    Serial,
    I2c,
    Spi,
    Can,
    Gpio,
    Pwm,
    Adc,
    Usb,
    Webcam,
    RealSense,
    Lidar,
    Input,
    Bluetooth,
    Net,
    EtherCat,
    EthernetIp,
    Profinet,
    Virtual,
}

/// Resolve a terra driver shortname to its crate info.
///
/// Returns `None` for unrecognized names.
pub fn resolve(name: &str) -> Option<TerraCrateInfo> {
    Some(match name {
        // Serial protocol drivers
        "dynamixel" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: Some("dynamixel"),
            accessor: DriverAccessor::Dynamixel,
        },
        "rplidar" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: Some("rplidar"),
            accessor: DriverAccessor::RpLidar,
        },
        "vesc" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: Some("vesc"),
            accessor: DriverAccessor::Serial,
        },
        "modbus" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: Some("modbus"),
            accessor: DriverAccessor::Serial,
        },
        "mavlink" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: Some("mavlink"),
            accessor: DriverAccessor::Serial,
        },
        "robotiq" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: Some("robotiq"),
            accessor: DriverAccessor::Serial,
        },
        "ublox" | "nmea" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: Some("ublox"),
            accessor: DriverAccessor::Serial,
        },

        // Embedded chip drivers
        "mpu6050" | "icm20689" => TerraCrateInfo {
            crate_name: "terra-embedded",
            version: "0.2",
            feature: Some("mpu6050"),
            accessor: DriverAccessor::I2c,
        },
        "bno055" => TerraCrateInfo {
            crate_name: "terra-embedded",
            version: "0.2",
            feature: Some("bno055"),
            accessor: DriverAccessor::I2c,
        },
        "bme280" => TerraCrateInfo {
            crate_name: "terra-embedded",
            version: "0.2",
            feature: Some("bme280"),
            accessor: DriverAccessor::I2c,
        },

        // CAN drivers
        "canopen" => TerraCrateInfo {
            crate_name: "terra-can",
            version: "0.2",
            feature: Some("canopen"),
            accessor: DriverAccessor::Can,
        },
        "odrive" => TerraCrateInfo {
            crate_name: "terra-can",
            version: "0.2",
            feature: Some("odrive"),
            accessor: DriverAccessor::Can,
        },

        // Standalone crate drivers
        "realsense" => TerraCrateInfo {
            crate_name: "terra-realsense",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::RealSense,
        },
        "webcam" => TerraCrateInfo {
            crate_name: "terra-webcam",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Webcam,
        },
        "velodyne" => TerraCrateInfo {
            crate_name: "terra-lidar",
            version: "0.2",
            feature: Some("velodyne"),
            accessor: DriverAccessor::Lidar,
        },
        "ouster" => TerraCrateInfo {
            crate_name: "terra-lidar",
            version: "0.2",
            feature: Some("ouster"),
            accessor: DriverAccessor::Lidar,
        },
        "livox" => TerraCrateInfo {
            crate_name: "terra-lidar",
            version: "0.2",
            feature: Some("livox"),
            accessor: DriverAccessor::Lidar,
        },
        "hesai" => TerraCrateInfo {
            crate_name: "terra-lidar",
            version: "0.2",
            feature: Some("hesai"),
            accessor: DriverAccessor::Lidar,
        },
        "ethercat" => TerraCrateInfo {
            crate_name: "terra-ethercat",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::EtherCat,
        },
        "ethernetip" => TerraCrateInfo {
            crate_name: "terra-ethernetip",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::EthernetIp,
        },
        "profinet" => TerraCrateInfo {
            crate_name: "terra-profinet",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Profinet,
        },

        // Bus-level drivers
        "i2c" => TerraCrateInfo {
            crate_name: "terra-i2c",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::I2c,
        },
        "spi" => TerraCrateInfo {
            crate_name: "terra-spi",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Spi,
        },
        "serial" | "uart" => TerraCrateInfo {
            crate_name: "terra-serial",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Serial,
        },
        "can" => TerraCrateInfo {
            crate_name: "terra-can",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Can,
        },
        "gpio" => TerraCrateInfo {
            crate_name: "terra-gpio",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Gpio,
        },
        "pwm" => TerraCrateInfo {
            crate_name: "terra-pwm",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Pwm,
        },
        "adc" => TerraCrateInfo {
            crate_name: "terra-adc",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Adc,
        },
        "usb" => TerraCrateInfo {
            crate_name: "terra-usb",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Usb,
        },
        "bluetooth" | "ble" => TerraCrateInfo {
            crate_name: "terra-bluetooth",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Bluetooth,
        },
        "input" | "gamepad" => TerraCrateInfo {
            crate_name: "terra-input",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Input,
        },
        "net" | "tcp" | "udp" => TerraCrateInfo {
            crate_name: "terra-net",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Net,
        },
        "virtual" | "mock" => TerraCrateInfo {
            crate_name: "terra-virtual",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::Virtual,
        },
        "zed" => TerraCrateInfo {
            crate_name: "terra-zed",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::RealSense,
        },
        "oakd" => TerraCrateInfo {
            crate_name: "terra-oakd",
            version: "0.2",
            feature: None,
            accessor: DriverAccessor::RealSense,
        },

        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_dynamixel() {
        let info = resolve("dynamixel").unwrap();
        assert_eq!(info.crate_name, "terra-serial");
        assert_eq!(info.feature, Some("dynamixel"));
        assert_eq!(info.accessor, DriverAccessor::Dynamixel);
    }

    #[test]
    fn resolve_rplidar() {
        let info = resolve("rplidar").unwrap();
        assert_eq!(info.crate_name, "terra-serial");
        assert_eq!(info.feature, Some("rplidar"));
        assert_eq!(info.accessor, DriverAccessor::RpLidar);
    }

    #[test]
    fn resolve_realsense() {
        let info = resolve("realsense").unwrap();
        assert_eq!(info.crate_name, "terra-realsense");
        assert!(info.feature.is_none());
        assert_eq!(info.accessor, DriverAccessor::RealSense);
    }

    #[test]
    fn resolve_i2c_bus() {
        let info = resolve("i2c").unwrap();
        assert_eq!(info.crate_name, "terra-i2c");
        assert!(info.feature.is_none());
    }

    #[test]
    fn resolve_virtual() {
        let info = resolve("virtual").unwrap();
        assert_eq!(info.crate_name, "terra-virtual");
        assert_eq!(info.accessor, DriverAccessor::Virtual);
    }

    #[test]
    fn resolve_unknown_returns_none() {
        assert!(resolve("nonexistent").is_none());
        assert!(resolve("").is_none());
    }

    #[test]
    fn resolve_all_serial_protocols() {
        for name in ["dynamixel", "rplidar", "vesc", "modbus", "mavlink", "robotiq"] {
            let info = resolve(name).unwrap();
            assert_eq!(info.crate_name, "terra-serial", "failed for {}", name);
        }
    }

    #[test]
    fn resolve_all_lidar_variants() {
        for name in ["velodyne", "ouster", "livox", "hesai"] {
            let info = resolve(name).unwrap();
            assert_eq!(info.crate_name, "terra-lidar", "failed for {}", name);
            assert_eq!(info.accessor, DriverAccessor::Lidar);
        }
    }
}
