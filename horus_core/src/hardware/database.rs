//! Device database for HORUS hardware discovery.
//!
//! Maps USB VID/PID pairs and I2C addresses to known devices,
//! providing device identification and driver matching.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Device category
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DeviceCategory {
    /// Microcontroller/development board
    Microcontroller,
    /// Serial adapter (FTDI, CP210x, CH340)
    SerialAdapter,
    /// Motor controller
    MotorController,
    /// Servo controller
    ServoController,
    /// LiDAR sensor
    Lidar,
    /// Depth camera
    DepthCamera,
    /// Regular camera
    Camera,
    /// IMU/accelerometer/gyroscope
    Imu,
    /// GPS/GNSS receiver
    Gps,
    /// CAN bus adapter
    CanAdapter,
    /// Joystick/gamepad
    Joystick,
    /// Bluetooth adapter
    Bluetooth,
    /// Audio device
    Audio,
    /// Network adapter
    Network,
    /// Power management
    Power,
    /// Display
    Display,
    /// Storage
    Storage,
    /// Other/unknown
    Other,
}

impl DeviceCategory {
    /// Get human-readable name
    pub fn name(&self) -> &'static str {
        match self {
            Self::Microcontroller => "Microcontroller",
            Self::SerialAdapter => "Serial Adapter",
            Self::MotorController => "Motor Controller",
            Self::ServoController => "Servo Controller",
            Self::Lidar => "LiDAR",
            Self::DepthCamera => "Depth Camera",
            Self::Camera => "Camera",
            Self::Imu => "IMU",
            Self::Gps => "GPS",
            Self::CanAdapter => "CAN Adapter",
            Self::Joystick => "Joystick",
            Self::Bluetooth => "Bluetooth",
            Self::Audio => "Audio",
            Self::Network => "Network",
            Self::Power => "Power",
            Self::Display => "Display",
            Self::Storage => "Storage",
            Self::Other => "Other",
        }
    }
}

/// Confidence level for driver matching
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum MatchConfidence {
    /// Perfect match by VID/PID
    Exact,
    /// Match by vendor + device class
    High,
    /// Match by device class only
    Medium,
    /// Guessed based on heuristics
    Low,
    /// No match found
    None,
}

/// Information about a known device
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    /// Device name
    pub name: String,
    /// Manufacturer name
    pub manufacturer: String,
    /// Device category
    pub category: DeviceCategory,
    /// Description
    pub description: Option<String>,
    /// Suggested HORUS driver/node
    pub horus_driver: Option<String>,
    /// Documentation URL
    pub docs_url: Option<String>,
    /// Whether the device is commonly used in robotics
    pub robotics_common: bool,
}

/// Driver match result
#[derive(Debug, Clone)]
pub struct DriverMatch {
    /// Device information
    pub device_info: DeviceInfo,
    /// Match confidence
    pub confidence: MatchConfidence,
    /// Suggested HORUS node type
    pub suggested_node: Option<String>,
    /// Configuration hints
    pub config_hints: HashMap<String, String>,
}

/// Device database
pub struct DeviceDatabase {
    /// USB devices by VID:PID
    usb_devices: HashMap<(u16, u16), DeviceInfo>,
    /// I2C devices by address
    i2c_devices: HashMap<u8, DeviceInfo>,
    /// Vendor information
    vendors: HashMap<u16, &'static str>,
}

impl DeviceDatabase {
    /// Create a new device database with built-in entries
    pub fn new() -> Self {
        let mut db = Self {
            usb_devices: HashMap::new(),
            i2c_devices: HashMap::new(),
            vendors: HashMap::new(),
        };
        db.populate_usb_devices();
        db.populate_i2c_devices();
        db.populate_vendors();
        db
    }

    /// Look up a USB device by VID/PID
    pub fn lookup_usb(&self, vid: u16, pid: u16) -> Option<&DeviceInfo> {
        self.usb_devices.get(&(vid, pid))
    }

    /// Look up an I2C device by address
    pub fn lookup_i2c(&self, address: u8) -> Option<&DeviceInfo> {
        self.i2c_devices.get(&address)
    }

    /// Get vendor name
    pub fn vendor_name(&self, vid: u16) -> Option<&str> {
        self.vendors.get(&vid).copied()
    }

    /// Match USB device to HORUS driver
    pub fn match_usb_driver(&self, vid: u16, pid: u16) -> DriverMatch {
        if let Some(info) = self.lookup_usb(vid, pid) {
            DriverMatch {
                device_info: info.clone(),
                confidence: MatchConfidence::Exact,
                suggested_node: info.horus_driver.clone(),
                config_hints: self.get_config_hints(vid, pid),
            }
        } else if let Some(vendor) = self.vendor_name(vid) {
            // Known vendor but unknown product
            DriverMatch {
                device_info: DeviceInfo {
                    name: format!("Unknown {} Device", vendor),
                    manufacturer: vendor.to_string(),
                    category: DeviceCategory::Other,
                    description: None,
                    horus_driver: None,
                    docs_url: None,
                    robotics_common: false,
                },
                confidence: MatchConfidence::Low,
                suggested_node: None,
                config_hints: HashMap::new(),
            }
        } else {
            DriverMatch {
                device_info: DeviceInfo {
                    name: format!("Unknown Device {:04X}:{:04X}", vid, pid),
                    manufacturer: "Unknown".to_string(),
                    category: DeviceCategory::Other,
                    description: None,
                    horus_driver: None,
                    docs_url: None,
                    robotics_common: false,
                },
                confidence: MatchConfidence::None,
                suggested_node: None,
                config_hints: HashMap::new(),
            }
        }
    }

    /// Match I2C device to HORUS driver
    pub fn match_i2c_driver(&self, address: u8) -> DriverMatch {
        if let Some(info) = self.lookup_i2c(address) {
            DriverMatch {
                device_info: info.clone(),
                confidence: MatchConfidence::Medium, // I2C addresses can be shared
                suggested_node: info.horus_driver.clone(),
                config_hints: self.get_i2c_config_hints(address),
            }
        } else {
            DriverMatch {
                device_info: DeviceInfo {
                    name: format!("Unknown I2C Device @ 0x{:02X}", address),
                    manufacturer: "Unknown".to_string(),
                    category: DeviceCategory::Other,
                    description: None,
                    horus_driver: None,
                    docs_url: None,
                    robotics_common: false,
                },
                confidence: MatchConfidence::None,
                suggested_node: None,
                config_hints: HashMap::new(),
            }
        }
    }

    /// Get all robotics-related devices
    pub fn robotics_devices(&self) -> Vec<(&(u16, u16), &DeviceInfo)> {
        self.usb_devices
            .iter()
            .filter(|(_, info)| info.robotics_common)
            .collect()
    }

    fn get_config_hints(&self, vid: u16, pid: u16) -> HashMap<String, String> {
        let mut hints = HashMap::new();

        // Serial adapter hints
        if vid == 0x0403 {
            // FTDI
            hints.insert("baud_rate".to_string(), "115200".to_string());
            hints.insert("driver".to_string(), "ftdi_sio".to_string());
        } else if vid == 0x10C4 {
            // Silicon Labs
            hints.insert("baud_rate".to_string(), "115200".to_string());
            hints.insert("driver".to_string(), "cp210x".to_string());
        } else if vid == 0x1A86 {
            // WCH CH340
            hints.insert("baud_rate".to_string(), "115200".to_string());
            hints.insert("driver".to_string(), "ch341".to_string());
        }

        // RPLidar hints
        if (vid, pid) == (0x10C4, 0xEA60) {
            hints.insert("baud_rate".to_string(), "115200".to_string());
            hints.insert("suggested_node".to_string(), "Lidar".to_string());
        }

        // ODrive hints
        if vid == 0x1209 && pid == 0x0D32 {
            hints.insert("protocol".to_string(), "odrive".to_string());
            hints.insert("suggested_node".to_string(), "DcMotor".to_string());
        }

        hints
    }

    fn get_i2c_config_hints(&self, address: u8) -> HashMap<String, String> {
        let mut hints = HashMap::new();

        match address {
            0x68 | 0x69 => {
                hints.insert("suggested_node".to_string(), "Imu".to_string());
            }
            0x76 | 0x77 => {
                hints.insert("data_type".to_string(), "pressure".to_string());
            }
            0x40..=0x4F => {
                hints.insert("suggested_node".to_string(), "ServoController".to_string());
            }
            _ => {}
        }

        hints
    }

    fn populate_usb_devices(&mut self) {
        // Arduino devices
        self.add_usb(
            0x2341,
            0x0043,
            "Arduino Uno",
            "Arduino",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x2341,
            0x0042,
            "Arduino Mega 2560",
            "Arduino",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x2341,
            0x8036,
            "Arduino Leonardo",
            "Arduino",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x2341,
            0x003D,
            "Arduino Due (Programming)",
            "Arduino",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x2341,
            0x003E,
            "Arduino Due (Native)",
            "Arduino",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x2341,
            0x8053,
            "Arduino Nano Every",
            "Arduino",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );

        // Arduino alternate VID
        self.add_usb(
            0x2A03,
            0x0043,
            "Arduino Uno (clone)",
            "Arduino",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );

        // ESP32 devices
        self.add_usb(
            0x303A,
            0x0002,
            "ESP32-S2",
            "Espressif",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x303A,
            0x1001,
            "ESP32-S3",
            "Espressif",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x303A,
            0x80D1,
            "ESP32-C3",
            "Espressif",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );

        // STM32 devices
        self.add_usb(
            0x0483,
            0x5740,
            "STM32 Virtual COM Port",
            "STMicroelectronics",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x0483,
            0xDF11,
            "STM32 Bootloader",
            "STMicroelectronics",
            DeviceCategory::Microcontroller,
            None,
            true,
        );

        // Teensy
        self.add_usb(
            0x16C0,
            0x0483,
            "Teensy",
            "PJRC",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );

        // Raspberry Pi Pico
        self.add_usb(
            0x2E8A,
            0x000A,
            "Raspberry Pi Pico",
            "Raspberry Pi",
            DeviceCategory::Microcontroller,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x2E8A,
            0x0005,
            "Raspberry Pi Pico Bootloader",
            "Raspberry Pi",
            DeviceCategory::Microcontroller,
            None,
            true,
        );

        // Serial adapters
        self.add_usb(
            0x0403,
            0x6001,
            "FT232R",
            "FTDI",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x0403,
            0x6010,
            "FT2232D",
            "FTDI",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x0403,
            0x6011,
            "FT4232H",
            "FTDI",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x0403,
            0x6014,
            "FT232H",
            "FTDI",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x0403,
            0x6015,
            "FT231X",
            "FTDI",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );

        self.add_usb(
            0x10C4,
            0xEA60,
            "CP2102/CP2109",
            "Silicon Labs",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x10C4,
            0xEA61,
            "CP2104",
            "Silicon Labs",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x10C4,
            0xEA70,
            "CP2105",
            "Silicon Labs",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x10C4,
            0xEA80,
            "CP2108",
            "Silicon Labs",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );

        self.add_usb(
            0x1A86,
            0x7523,
            "CH340",
            "WCH",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x1A86,
            0x5523,
            "CH341",
            "WCH",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );

        self.add_usb(
            0x067B,
            0x2303,
            "PL2303",
            "Prolific",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );
        self.add_usb(
            0x067B,
            0x23A3,
            "PL2303HXD",
            "Prolific",
            DeviceCategory::SerialAdapter,
            Some("Serial"),
            true,
        );

        // Motor controllers
        self.add_usb(
            0x1209,
            0x0D32,
            "ODrive",
            "ODrive Robotics",
            DeviceCategory::MotorController,
            Some("DcMotor"),
            true,
        );
        self.add_usb(
            0x1FFB,
            0x00B3,
            "Pololu Jrk G2",
            "Pololu",
            DeviceCategory::MotorController,
            Some("DcMotor"),
            true,
        );
        self.add_usb(
            0x1FFB,
            0x0089,
            "Pololu Tic T825",
            "Pololu",
            DeviceCategory::MotorController,
            Some("DcMotor"),
            true,
        );

        // Servo controllers
        self.add_usb(
            0x1FFB,
            0x0089,
            "Pololu Maestro 6",
            "Pololu",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );
        self.add_usb(
            0x1FFB,
            0x008A,
            "Pololu Maestro 12",
            "Pololu",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );
        self.add_usb(
            0x1FFB,
            0x008B,
            "Pololu Maestro 18",
            "Pololu",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );
        self.add_usb(
            0x1FFB,
            0x008C,
            "Pololu Maestro 24",
            "Pololu",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );

        // LiDAR sensors
        // Note: Many LiDARs use generic serial chips, so VID/PID may not identify them uniquely
        // RPLidar uses CP2102 (0x10C4:0xEA60) - handled in config hints

        // Depth cameras
        self.add_usb(
            0x8086,
            0x0AD1,
            "Intel RealSense D400",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0AD2,
            "Intel RealSense D410",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0AD3,
            "Intel RealSense D415",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0AD4,
            "Intel RealSense D430",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0AD5,
            "Intel RealSense D435",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0AD6,
            "Intel RealSense D435i",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0B3A,
            "Intel RealSense D455",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0B07,
            "Intel RealSense D405",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x8086,
            0x0B4D,
            "Intel RealSense L515",
            "Intel",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );

        self.add_usb(
            0x2BC5,
            0x0401,
            "Orbbec Astra",
            "Orbbec",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x2BC5,
            0x0402,
            "Orbbec Astra Pro",
            "Orbbec",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x2BC5,
            0x0636,
            "Orbbec Femto",
            "Orbbec",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );

        self.add_usb(
            0x2B03,
            0xF580,
            "Stereolabs ZED",
            "Stereolabs",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x2B03,
            0xF582,
            "Stereolabs ZED 2",
            "Stereolabs",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );
        self.add_usb(
            0x2B03,
            0xF680,
            "Stereolabs ZED 2i",
            "Stereolabs",
            DeviceCategory::DepthCamera,
            Some("Camera"),
            true,
        );

        // GPS receivers
        self.add_usb(
            0x1546,
            0x01A5,
            "u-blox 5",
            "u-blox",
            DeviceCategory::Gps,
            Some("Gps"),
            true,
        );
        self.add_usb(
            0x1546,
            0x01A6,
            "u-blox 6",
            "u-blox",
            DeviceCategory::Gps,
            Some("Gps"),
            true,
        );
        self.add_usb(
            0x1546,
            0x01A7,
            "u-blox 7",
            "u-blox",
            DeviceCategory::Gps,
            Some("Gps"),
            true,
        );
        self.add_usb(
            0x1546,
            0x01A8,
            "u-blox 8",
            "u-blox",
            DeviceCategory::Gps,
            Some("Gps"),
            true,
        );
        self.add_usb(
            0x1546,
            0x01A9,
            "u-blox 9",
            "u-blox",
            DeviceCategory::Gps,
            Some("Gps"),
            true,
        );

        // CAN adapters
        self.add_usb(
            0x0C72,
            0x000C,
            "PEAK PCAN-USB",
            "Peak Systems",
            DeviceCategory::CanAdapter,
            Some("CanBus"),
            true,
        );
        self.add_usb(
            0x0C72,
            0x0014,
            "PEAK PCAN-USB Pro",
            "Peak Systems",
            DeviceCategory::CanAdapter,
            Some("CanBus"),
            true,
        );
        self.add_usb(
            0x0BFD,
            0x0120,
            "Kvaser Leaf Light v2",
            "Kvaser",
            DeviceCategory::CanAdapter,
            Some("CanBus"),
            true,
        );
        self.add_usb(
            0x1D50,
            0x606F,
            "CANable/candlelight",
            "OpenMoko",
            DeviceCategory::CanAdapter,
            Some("CanBus"),
            true,
        );
        self.add_usb(
            0x1D50,
            0x60CA,
            "CANABLE 2.0",
            "CANABLE",
            DeviceCategory::CanAdapter,
            Some("CanBus"),
            true,
        );

        // Joysticks/Gamepads
        self.add_usb(
            0x045E,
            0x028E,
            "Xbox 360 Controller",
            "Microsoft",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );
        self.add_usb(
            0x045E,
            0x02D1,
            "Xbox One Controller",
            "Microsoft",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );
        self.add_usb(
            0x054C,
            0x05C4,
            "DualShock 4",
            "Sony",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );
        self.add_usb(
            0x054C,
            0x09CC,
            "DualShock 4 v2",
            "Sony",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );
        self.add_usb(
            0x054C,
            0x0CE6,
            "DualSense",
            "Sony",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );
        self.add_usb(
            0x046D,
            0xC21D,
            "Logitech F310",
            "Logitech",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );
        self.add_usb(
            0x046D,
            0xC21E,
            "Logitech F510",
            "Logitech",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );
        self.add_usb(
            0x046D,
            0xC21F,
            "Logitech F710",
            "Logitech",
            DeviceCategory::Joystick,
            Some("Joystick"),
            true,
        );

        // Bluetooth adapters
        // Intel
        self.add_usb(
            0x8087,
            0x0A2A,
            "Intel Wireless Bluetooth",
            "Intel",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x8087,
            0x0A2B,
            "Intel Wireless Bluetooth",
            "Intel",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x8087,
            0x0AAA,
            "Intel Wireless Bluetooth AX200",
            "Intel",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x8087,
            0x0029,
            "Intel AX201 Bluetooth",
            "Intel",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x8087,
            0x0032,
            "Intel AX210/211 Bluetooth",
            "Intel",
            DeviceCategory::Bluetooth,
            None,
            false,
        );

        // Broadcom
        self.add_usb(
            0x0A5C,
            0x21E8,
            "BCM20702A0 Bluetooth",
            "Broadcom",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x0A5C,
            0x21E1,
            "HP Broadcom Bluetooth",
            "Broadcom",
            DeviceCategory::Bluetooth,
            None,
            false,
        );

        // Realtek
        self.add_usb(
            0x0BDA,
            0x8771,
            "Realtek RTL8761B Bluetooth",
            "Realtek",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x0BDA,
            0xB00A,
            "Realtek Bluetooth",
            "Realtek",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x0BDA,
            0xB009,
            "Realtek Bluetooth 5.0",
            "Realtek",
            DeviceCategory::Bluetooth,
            None,
            false,
        );

        // Qualcomm/Atheros
        self.add_usb(
            0x0CF3,
            0xE300,
            "Qualcomm Atheros Bluetooth",
            "Qualcomm",
            DeviceCategory::Bluetooth,
            None,
            false,
        );
        self.add_usb(
            0x0CF3,
            0x3004,
            "Atheros AR3012 Bluetooth",
            "Qualcomm",
            DeviceCategory::Bluetooth,
            None,
            false,
        );

        // Cambridge Silicon Radio (CSR)
        self.add_usb(
            0x0A12,
            0x0001,
            "CSR Bluetooth Adapter",
            "Cambridge Silicon Radio",
            DeviceCategory::Bluetooth,
            None,
            false,
        );

        // Network adapters (USB WiFi/Ethernet)
        // Realtek WiFi
        self.add_usb(
            0x0BDA,
            0x8812,
            "Realtek RTL8812AU WiFi",
            "Realtek",
            DeviceCategory::Network,
            None,
            false,
        );
        self.add_usb(
            0x0BDA,
            0xB812,
            "Realtek RTL8812BU WiFi",
            "Realtek",
            DeviceCategory::Network,
            None,
            false,
        );
        self.add_usb(
            0x0BDA,
            0x8179,
            "Realtek RTL8188EUS WiFi",
            "Realtek",
            DeviceCategory::Network,
            None,
            false,
        );
        self.add_usb(
            0x0BDA,
            0xC811,
            "Realtek RTL8811CU WiFi",
            "Realtek",
            DeviceCategory::Network,
            None,
            false,
        );
        self.add_usb(
            0x0BDA,
            0x1A2B,
            "Realtek WiFi 6 AX",
            "Realtek",
            DeviceCategory::Network,
            None,
            false,
        );

        // Intel WiFi
        self.add_usb(
            0x8086,
            0x0082,
            "Intel Centrino Advanced-N 6205",
            "Intel",
            DeviceCategory::Network,
            None,
            false,
        );
        self.add_usb(
            0x8086,
            0x2723,
            "Intel Wi-Fi 6 AX200",
            "Intel",
            DeviceCategory::Network,
            None,
            false,
        );

        // Atheros WiFi
        self.add_usb(
            0x0CF3,
            0x9271,
            "Atheros AR9271 WiFi",
            "Qualcomm",
            DeviceCategory::Network,
            None,
            true, // Common in robotics for long-range WiFi
        );

        // Ralink/MediaTek WiFi
        self.add_usb(
            0x148F,
            0x5370,
            "Ralink RT5370 WiFi",
            "MediaTek",
            DeviceCategory::Network,
            None,
            false,
        );
        self.add_usb(
            0x148F,
            0x7601,
            "MediaTek MT7601U WiFi",
            "MediaTek",
            DeviceCategory::Network,
            None,
            false,
        );

        // USB Ethernet adapters
        self.add_usb(
            0x0B95,
            0x772B,
            "ASIX AX88772B Ethernet",
            "ASIX",
            DeviceCategory::Network,
            None,
            true, // Common for SBC networking
        );
        self.add_usb(
            0x0B95,
            0x1790,
            "ASIX AX88179 Gigabit Ethernet",
            "ASIX",
            DeviceCategory::Network,
            None,
            true,
        );
        self.add_usb(
            0x0BDA,
            0x8153,
            "Realtek RTL8153 Gigabit Ethernet",
            "Realtek",
            DeviceCategory::Network,
            None,
            true,
        );

        // Audio devices
        // Generic USB audio class
        self.add_usb(
            0x08BB,
            0x2902,
            "Texas Instruments PCM2902 Audio",
            "Texas Instruments",
            DeviceCategory::Audio,
            None,
            false,
        );
        self.add_usb(
            0x08BB,
            0x2912,
            "Texas Instruments PCM2912 Audio",
            "Texas Instruments",
            DeviceCategory::Audio,
            None,
            false,
        );

        // C-Media
        self.add_usb(
            0x0D8C,
            0x0014,
            "C-Media USB Audio",
            "C-Media",
            DeviceCategory::Audio,
            None,
            false,
        );
        self.add_usb(
            0x0D8C,
            0x0102,
            "C-Media USB Sound Device",
            "C-Media",
            DeviceCategory::Audio,
            None,
            false,
        );
        self.add_usb(
            0x0D8C,
            0x013C,
            "C-Media USB Headphone Set",
            "C-Media",
            DeviceCategory::Audio,
            None,
            false,
        );

        // Logitech Audio
        self.add_usb(
            0x046D,
            0x0A44,
            "Logitech USB Headset",
            "Logitech",
            DeviceCategory::Audio,
            None,
            false,
        );
        self.add_usb(
            0x046D,
            0x0A29,
            "Logitech USB Headset H600",
            "Logitech",
            DeviceCategory::Audio,
            None,
            false,
        );

        // Blue Microphones
        self.add_usb(
            0x0D8C,
            0x016C,
            "Blue Yeti Microphone",
            "Blue Microphones",
            DeviceCategory::Audio,
            None,
            false,
        );
        self.add_usb(
            0xB58E,
            0x9E84,
            "Blue Yeti X Microphone",
            "Blue Microphones",
            DeviceCategory::Audio,
            None,
            false,
        );

        // Focusrite
        self.add_usb(
            0x1235,
            0x8200,
            "Focusrite Scarlett 2i2",
            "Focusrite",
            DeviceCategory::Audio,
            None,
            false,
        );
        self.add_usb(
            0x1235,
            0x8204,
            "Focusrite Scarlett Solo",
            "Focusrite",
            DeviceCategory::Audio,
            None,
            false,
        );

        // Audio-specific for robotics (ReSpeaker, etc.)
        self.add_usb(
            0x2886,
            0x0018,
            "Seeed ReSpeaker Mic Array v2.0",
            "Seeed",
            DeviceCategory::Audio,
            None,
            true, // Common for voice-controlled robotics
        );
        self.add_usb(
            0x2886,
            0x0007,
            "Seeed ReSpeaker USB Mic Array",
            "Seeed",
            DeviceCategory::Audio,
            None,
            true,
        );

        // PlayStation Audio (for VR/AR robotics applications)
        self.add_usb(
            0x054C,
            0x0CE6,
            "Sony DualSense Audio",
            "Sony",
            DeviceCategory::Audio,
            None,
            false,
        );
    }

    fn populate_i2c_devices(&mut self) {
        // IMUs
        self.add_i2c(
            0x68,
            "MPU-6050/6500/9250",
            "InvenSense",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x69,
            "MPU-6050/6500/9250 (alt)",
            "InvenSense",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x6A,
            "LSM6DS3",
            "STMicroelectronics",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x6B,
            "LSM6DS3 (alt)",
            "STMicroelectronics",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x28,
            "BNO055",
            "Bosch",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x29,
            "BNO055 (alt)",
            "Bosch",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x4A,
            "BNO085/BNO086",
            "Bosch",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x4B,
            "BNO085/BNO086 (alt)",
            "Bosch",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x1C,
            "LIS3DH",
            "STMicroelectronics",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x1D,
            "ADXL345",
            "Analog Devices",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );

        // Magnetometers
        self.add_i2c(
            0x0C,
            "AK8963",
            "AKM",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x0D,
            "QMC5883L",
            "QST",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );
        self.add_i2c(
            0x1E,
            "HMC5883L",
            "Honeywell",
            DeviceCategory::Imu,
            Some("Imu"),
            true,
        );

        // Pressure/altitude sensors
        self.add_i2c(
            0x76,
            "BMP280/BME280",
            "Bosch",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x77,
            "BMP280/BME280 (alt)",
            "Bosch",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x5C,
            "LPS22HB",
            "STMicroelectronics",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x5D,
            "LPS22HB (alt)",
            "STMicroelectronics",
            DeviceCategory::Other,
            None,
            true,
        );

        // PWM/Servo controllers
        self.add_i2c(
            0x40,
            "PCA9685",
            "NXP",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );
        self.add_i2c(
            0x41,
            "PCA9685 (A0)",
            "NXP",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );
        self.add_i2c(
            0x42,
            "PCA9685 (A1)",
            "NXP",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );
        self.add_i2c(
            0x43,
            "PCA9685 (A0+A1)",
            "NXP",
            DeviceCategory::ServoController,
            Some("ServoController"),
            true,
        );

        // Motor drivers
        self.add_i2c(
            0x60,
            "TB6612/DRV8830",
            "Various",
            DeviceCategory::MotorController,
            Some("DcMotor"),
            true,
        );
        self.add_i2c(
            0x64,
            "Adafruit Motor Shield v2",
            "Adafruit",
            DeviceCategory::MotorController,
            Some("DcMotor"),
            true,
        );

        // ADC
        self.add_i2c(
            0x48,
            "ADS1115",
            "Texas Instruments",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x49,
            "ADS1115 (ADDR)",
            "Texas Instruments",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x4A,
            "ADS1115 (ADDR)",
            "Texas Instruments",
            DeviceCategory::Other,
            None,
            false,
        );
        self.add_i2c(
            0x4B,
            "ADS1115 (ADDR)",
            "Texas Instruments",
            DeviceCategory::Other,
            None,
            false,
        );

        // DAC
        self.add_i2c(
            0x62,
            "MCP4725",
            "Microchip",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x63,
            "MCP4725 (A0)",
            "Microchip",
            DeviceCategory::Other,
            None,
            true,
        );

        // Display
        self.add_i2c(
            0x3C,
            "SSD1306 OLED",
            "Solomon Systech",
            DeviceCategory::Display,
            None,
            true,
        );
        self.add_i2c(
            0x3D,
            "SSD1306 OLED (alt)",
            "Solomon Systech",
            DeviceCategory::Display,
            None,
            true,
        );

        // Distance sensors
        self.add_i2c(
            0x29,
            "VL53L0X",
            "STMicroelectronics",
            DeviceCategory::Lidar,
            Some("Lidar"),
            true,
        );
        self.add_i2c(
            0x52,
            "VL53L1X",
            "STMicroelectronics",
            DeviceCategory::Lidar,
            Some("Lidar"),
            true,
        );

        // RTC
        self.add_i2c(0x68, "DS3231", "Maxim", DeviceCategory::Other, None, false);
        self.add_i2c(0x51, "PCF8563", "NXP", DeviceCategory::Other, None, true);

        // GPIO expanders
        self.add_i2c(
            0x20,
            "PCF8574/MCP23017",
            "Various",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x21,
            "PCF8574/MCP23017 (A0)",
            "Various",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x22,
            "PCF8574/MCP23017 (A1)",
            "Various",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x23,
            "PCF8574/MCP23017 (A0+A1)",
            "Various",
            DeviceCategory::Other,
            None,
            true,
        );
        self.add_i2c(
            0x27,
            "PCF8574 (default LCD)",
            "NXP",
            DeviceCategory::Display,
            None,
            true,
        );

        // Current sensors
        self.add_i2c(
            0x40,
            "INA219/INA226",
            "Texas Instruments",
            DeviceCategory::Power,
            None,
            false,
        );
        self.add_i2c(
            0x44,
            "INA219/INA226 (A0)",
            "Texas Instruments",
            DeviceCategory::Power,
            None,
            true,
        );
        self.add_i2c(
            0x45,
            "INA219/INA226 (A1)",
            "Texas Instruments",
            DeviceCategory::Power,
            None,
            true,
        );
    }

    fn populate_vendors(&mut self) {
        self.vendors.insert(0x2341, "Arduino");
        self.vendors.insert(0x2A03, "Arduino (clone)");
        self.vendors.insert(0x303A, "Espressif");
        self.vendors.insert(0x0483, "STMicroelectronics");
        self.vendors.insert(0x16C0, "PJRC (Teensy)");
        self.vendors.insert(0x2E8A, "Raspberry Pi");
        self.vendors.insert(0x0403, "FTDI");
        self.vendors.insert(0x10C4, "Silicon Labs");
        self.vendors.insert(0x1A86, "WCH");
        self.vendors.insert(0x067B, "Prolific");
        self.vendors.insert(0x1209, "pid.codes");
        self.vendors.insert(0x1FFB, "Pololu");
        self.vendors.insert(0x8086, "Intel");
        self.vendors.insert(0x2BC5, "Orbbec");
        self.vendors.insert(0x2B03, "Stereolabs");
        self.vendors.insert(0x1546, "u-blox");
        self.vendors.insert(0x0C72, "Peak Systems");
        self.vendors.insert(0x0BFD, "Kvaser");
        self.vendors.insert(0x1D50, "OpenMoko");
        self.vendors.insert(0x045E, "Microsoft");
        self.vendors.insert(0x054C, "Sony");
        self.vendors.insert(0x046D, "Logitech");
        self.vendors.insert(0x0451, "Texas Instruments");
        // Bluetooth vendors
        self.vendors.insert(0x8087, "Intel");
        self.vendors.insert(0x0A5C, "Broadcom");
        self.vendors.insert(0x0BDA, "Realtek");
        self.vendors.insert(0x0CF3, "Qualcomm/Atheros");
        self.vendors.insert(0x0A12, "Cambridge Silicon Radio");
        // Network vendors
        self.vendors.insert(0x148F, "MediaTek/Ralink");
        self.vendors.insert(0x0B95, "ASIX");
        // Audio vendors
        self.vendors.insert(0x08BB, "Texas Instruments");
        self.vendors.insert(0x0D8C, "C-Media");
        self.vendors.insert(0xB58E, "Blue Microphones");
        self.vendors.insert(0x1235, "Focusrite");
        self.vendors.insert(0x2886, "Seeed");
    }

    #[allow(clippy::too_many_arguments)]
    fn add_usb(
        &mut self,
        vid: u16,
        pid: u16,
        name: &str,
        manufacturer: &str,
        category: DeviceCategory,
        horus_driver: Option<&str>,
        robotics_common: bool,
    ) {
        self.usb_devices.insert(
            (vid, pid),
            DeviceInfo {
                name: name.to_string(),
                manufacturer: manufacturer.to_string(),
                category,
                description: None,
                horus_driver: horus_driver.map(|s| s.to_string()),
                docs_url: None,
                robotics_common,
            },
        );
    }

    fn add_i2c(
        &mut self,
        address: u8,
        name: &str,
        manufacturer: &str,
        category: DeviceCategory,
        horus_driver: Option<&str>,
        robotics_common: bool,
    ) {
        self.i2c_devices.insert(
            address,
            DeviceInfo {
                name: name.to_string(),
                manufacturer: manufacturer.to_string(),
                category,
                description: None,
                horus_driver: horus_driver.map(|s| s.to_string()),
                docs_url: None,
                robotics_common,
            },
        );
    }
}

impl Default for DeviceDatabase {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lookup_usb() {
        let db = DeviceDatabase::new();

        // Arduino Uno
        let device = db.lookup_usb(0x2341, 0x0043);
        assert!(device.is_some());
        let device = device.unwrap();
        assert_eq!(device.name, "Arduino Uno");
        assert_eq!(device.category, DeviceCategory::Microcontroller);

        // Intel RealSense D435
        let device = db.lookup_usb(0x8086, 0x0AD5);
        assert!(device.is_some());
        let device = device.unwrap();
        assert_eq!(device.category, DeviceCategory::DepthCamera);
    }

    #[test]
    fn test_lookup_i2c() {
        let db = DeviceDatabase::new();

        // Test that common I2C addresses have registered devices
        // Note: Many I2C addresses are shared by multiple devices,
        // the database stores the last registered device for each address.

        // Address 0x68 - shared by MPU-6050/9250, DS3231 RTC, etc.
        let device = db.lookup_i2c(0x68);
        assert!(device.is_some(), "Should find device at address 0x68");
        let device = device.unwrap();
        assert!(!device.name.is_empty());
        assert!(!device.manufacturer.is_empty());

        // Address 0x40 - shared by PCA9685, INA219/INA226, etc.
        let device = db.lookup_i2c(0x40);
        assert!(device.is_some(), "Should find device at address 0x40");
        let device = device.unwrap();
        assert!(!device.name.is_empty());
        assert!(!device.manufacturer.is_empty());

        // Verify we can look up a known unique address
        // PCF8563 RTC is at 0x51
        let device = db.lookup_i2c(0x51);
        assert!(device.is_some(), "Should find PCF8563 at address 0x51");
        let device = device.unwrap();
        assert!(device.name.contains("PCF8563"));
    }

    #[test]
    fn test_driver_matching() {
        let db = DeviceDatabase::new();

        // Known device
        let result = db.match_usb_driver(0x2341, 0x0043);
        assert_eq!(result.confidence, MatchConfidence::Exact);
        assert!(result.suggested_node.is_some());

        // Unknown device from known vendor
        let result = db.match_usb_driver(0x0403, 0xFFFF);
        assert_eq!(result.confidence, MatchConfidence::Low);

        // Completely unknown
        let result = db.match_usb_driver(0xFFFF, 0xFFFF);
        assert_eq!(result.confidence, MatchConfidence::None);
    }
}
