//! Unified hardware discovery for HORUS.
//!
//! Provides a single entry point for discovering all connected hardware
//! and generating comprehensive discovery reports.

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use super::database::{DeviceCategory, DeviceDatabase, DriverMatch};
use super::platform::{Platform, PlatformCapabilities, PlatformDetector};
use super::serial::{SerialDiscovery, SerialPort};
use super::usb::{UsbDevice, UsbDiscovery};

#[cfg(target_os = "linux")]
use super::camera::{Camera, CameraDiscovery};
#[cfg(target_os = "linux")]
use super::gpio::{GpioChip, GpioDiscovery};
#[cfg(target_os = "linux")]
use super::i2c::{I2cBus, I2cDevice, I2cDiscovery};

/// A discovered device with identification and driver matching
#[derive(Debug, Clone)]
pub struct DiscoveredDevice {
    /// Device category
    pub category: DeviceCategory,
    /// Device name
    pub name: String,
    /// Manufacturer
    pub manufacturer: Option<String>,
    /// Device path (if applicable)
    pub path: Option<PathBuf>,
    /// USB VID:PID (if USB device)
    pub vid_pid: Option<(u16, u16)>,
    /// I2C address (if I2C device)
    pub i2c_address: Option<u8>,
    /// I2C bus (if I2C device)
    pub i2c_bus: Option<u8>,
    /// Suggested HORUS driver/node
    pub suggested_driver: Option<String>,
    /// Driver match result
    pub driver_match: Option<DriverMatch>,
    /// Additional properties
    pub properties: HashMap<String, String>,
}

impl DiscoveredDevice {
    /// Get a display string for this device
    pub fn display_string(&self) -> String {
        let mut parts = vec![self.name.clone()];

        if let Some((vid, pid)) = self.vid_pid {
            parts.push(format!("[{:04X}:{:04X}]", vid, pid));
        }

        if let Some(addr) = self.i2c_address {
            parts.push(format!("[I2C 0x{:02X}]", addr));
        }

        if let Some(path) = &self.path {
            parts.push(format!("@ {}", path.display()));
        }

        parts.join(" ")
    }
}

/// Comprehensive hardware discovery report
#[derive(Debug)]
pub struct DiscoveryReport {
    /// Platform information
    pub platform: Platform,
    /// Platform capabilities
    pub capabilities: PlatformCapabilities,
    /// USB devices
    pub usb_devices: Vec<UsbDevice>,
    /// Serial ports
    pub serial_ports: Vec<SerialPort>,
    /// I2C buses (Linux only)
    #[cfg(target_os = "linux")]
    pub i2c_buses: Vec<I2cBus>,
    /// I2C devices (Linux only)
    #[cfg(target_os = "linux")]
    pub i2c_devices: Vec<I2cDevice>,
    /// GPIO chips (Linux only)
    #[cfg(target_os = "linux")]
    pub gpio_chips: Vec<GpioChip>,
    /// Cameras (Linux only)
    #[cfg(target_os = "linux")]
    pub cameras: Vec<Camera>,
    /// All discovered devices with driver matching
    pub all_devices: Vec<DiscoveredDevice>,
    /// Discovery duration
    pub duration: Duration,
    /// Any errors encountered during discovery
    pub errors: Vec<String>,
}

impl DiscoveryReport {
    /// Get devices by category
    pub fn devices_by_category(&self, category: DeviceCategory) -> Vec<&DiscoveredDevice> {
        self.all_devices
            .iter()
            .filter(|d| d.category == category)
            .collect()
    }

    /// Get devices with suggested drivers
    pub fn devices_with_drivers(&self) -> Vec<&DiscoveredDevice> {
        self.all_devices
            .iter()
            .filter(|d| d.suggested_driver.is_some())
            .collect()
    }

    /// Get summary statistics
    pub fn summary(&self) -> DiscoverySummary {
        let mut category_counts: HashMap<DeviceCategory, usize> = HashMap::new();
        for device in &self.all_devices {
            *category_counts.entry(device.category.clone()).or_insert(0) += 1;
        }

        DiscoverySummary {
            platform: self.platform.clone(),
            usb_count: self.usb_devices.len(),
            serial_count: self.serial_ports.len(),
            #[cfg(target_os = "linux")]
            i2c_bus_count: self.i2c_buses.len(),
            #[cfg(target_os = "linux")]
            i2c_device_count: self.i2c_devices.len(),
            #[cfg(target_os = "linux")]
            gpio_chip_count: self.gpio_chips.len(),
            #[cfg(target_os = "linux")]
            camera_count: self.cameras.len(),
            #[cfg(not(target_os = "linux"))]
            i2c_bus_count: 0,
            #[cfg(not(target_os = "linux"))]
            i2c_device_count: 0,
            #[cfg(not(target_os = "linux"))]
            gpio_chip_count: 0,
            #[cfg(not(target_os = "linux"))]
            camera_count: 0,
            total_devices: self.all_devices.len(),
            devices_with_drivers: self.devices_with_drivers().len(),
            category_counts,
            duration: self.duration,
        }
    }
}

/// Summary statistics for discovery
#[derive(Debug)]
pub struct DiscoverySummary {
    /// Platform
    pub platform: Platform,
    /// USB device count
    pub usb_count: usize,
    /// Serial port count
    pub serial_count: usize,
    /// I2C bus count
    pub i2c_bus_count: usize,
    /// I2C device count
    pub i2c_device_count: usize,
    /// GPIO chip count
    pub gpio_chip_count: usize,
    /// Camera count
    pub camera_count: usize,
    /// Total discovered devices
    pub total_devices: usize,
    /// Devices with driver suggestions
    pub devices_with_drivers: usize,
    /// Device counts by category
    pub category_counts: HashMap<DeviceCategory, usize>,
    /// Discovery duration
    pub duration: Duration,
}

/// Hardware discovery options
#[derive(Debug, Clone)]
pub struct DiscoveryOptions {
    /// Scan USB devices
    pub scan_usb: bool,
    /// Scan serial ports
    pub scan_serial: bool,
    /// Scan I2C buses (may require root on some systems)
    pub scan_i2c: bool,
    /// Probe I2C addresses (may interfere with some devices)
    pub probe_i2c: bool,
    /// Scan GPIO chips
    pub scan_gpio: bool,
    /// Scan cameras
    pub scan_cameras: bool,
    /// Timeout for probing operations
    pub probe_timeout: Duration,
}

impl Default for DiscoveryOptions {
    fn default() -> Self {
        Self {
            scan_usb: true,
            scan_serial: true,
            scan_i2c: true,
            probe_i2c: false, // Off by default - can interfere with devices
            scan_gpio: true,
            scan_cameras: true,
            probe_timeout: Duration::from_millis(100),
        }
    }
}

/// Main hardware discovery interface
pub struct HardwareDiscovery {
    /// USB discovery
    usb_discovery: UsbDiscovery,
    /// Serial discovery
    serial_discovery: SerialDiscovery,
    /// I2C discovery (Linux only)
    #[cfg(target_os = "linux")]
    i2c_discovery: I2cDiscovery,
    /// GPIO discovery (Linux only)
    #[cfg(target_os = "linux")]
    gpio_discovery: GpioDiscovery,
    /// Camera discovery (Linux only)
    #[cfg(target_os = "linux")]
    camera_discovery: CameraDiscovery,
    /// Device database
    database: DeviceDatabase,
    /// Discovery options
    options: DiscoveryOptions,
}

impl HardwareDiscovery {
    /// Create a new hardware discovery instance
    pub fn new() -> Result<Self, String> {
        Ok(Self {
            usb_discovery: UsbDiscovery::new(),
            serial_discovery: SerialDiscovery::new(),
            #[cfg(target_os = "linux")]
            i2c_discovery: I2cDiscovery::new(),
            #[cfg(target_os = "linux")]
            gpio_discovery: GpioDiscovery::new(),
            #[cfg(target_os = "linux")]
            camera_discovery: CameraDiscovery::new(),
            database: DeviceDatabase::new(),
            options: DiscoveryOptions::default(),
        })
    }

    /// Create with custom options
    pub fn with_options(options: DiscoveryOptions) -> Result<Self, String> {
        let mut discovery = Self::new()?;
        discovery.options = options;
        Ok(discovery)
    }

    /// Set discovery options
    pub fn set_options(&mut self, options: DiscoveryOptions) {
        self.options = options;
    }

    /// Get the current platform
    pub fn platform(&self) -> Platform {
        PlatformDetector::detect()
    }

    /// Get platform capabilities
    pub fn capabilities(&self) -> PlatformCapabilities {
        PlatformDetector::capabilities(&PlatformDetector::detect())
    }

    /// Get the device database
    pub fn database(&self) -> &DeviceDatabase {
        &self.database
    }

    /// Scan all hardware and return a comprehensive report
    pub fn scan_all(&mut self) -> DiscoveryReport {
        let start = Instant::now();
        let mut errors = Vec::new();
        let mut all_devices = Vec::new();

        // Detect platform
        let platform = PlatformDetector::detect();
        let capabilities = PlatformDetector::capabilities(&platform);

        // Scan USB
        let usb_devices = if self.options.scan_usb {
            self.usb_discovery.enumerate()
        } else {
            Vec::new()
        };

        // Add USB devices to all_devices
        for usb in &usb_devices {
            let driver_match = self.database.match_usb_driver(usb.vendor_id, usb.product_id);
            let mut properties = HashMap::new();

            if let Some(serial) = &usb.serial {
                properties.insert("serial".to_string(), serial.clone());
            }
            if let Some(speed) = usb.speed_mbps {
                properties.insert("speed_mbps".to_string(), speed.to_string());
            }

            all_devices.push(DiscoveredDevice {
                category: driver_match.device_info.category.clone(),
                name: usb.display_name(),
                manufacturer: usb.manufacturer.clone(),
                path: usb.tty_path.clone().or_else(|| usb.video_path.clone()),
                vid_pid: Some((usb.vendor_id, usb.product_id)),
                i2c_address: None,
                i2c_bus: None,
                suggested_driver: driver_match.device_info.horus_driver.clone(),
                driver_match: Some(driver_match),
                properties,
            });
        }

        // Scan serial ports
        let serial_ports = if self.options.scan_serial {
            self.serial_discovery.enumerate()
        } else {
            Vec::new()
        };

        // Add serial ports that aren't already covered by USB
        for serial in &serial_ports {
            // Skip if this serial port is already covered by a USB device
            if serial.is_usb() && serial.vendor_id.is_some() {
                continue;
            }

            let mut properties = HashMap::new();
            if let Some(driver) = &serial.driver {
                properties.insert("driver".to_string(), driver.clone());
            }

            all_devices.push(DiscoveredDevice {
                category: DeviceCategory::SerialAdapter,
                name: serial.display_name(),
                manufacturer: serial.manufacturer.clone(),
                path: Some(serial.path.clone()),
                vid_pid: match (serial.vendor_id, serial.product_id) {
                    (Some(vid), Some(pid)) => Some((vid, pid)),
                    _ => None,
                },
                i2c_address: None,
                i2c_bus: None,
                suggested_driver: Some("Serial".to_string()),
                driver_match: None,
                properties,
            });
        }

        // Linux-specific discovery
        #[cfg(target_os = "linux")]
        let i2c_buses;
        #[cfg(target_os = "linux")]
        let i2c_devices;
        #[cfg(target_os = "linux")]
        let gpio_chips;
        #[cfg(target_os = "linux")]
        let cameras;

        #[cfg(target_os = "linux")]
        {
            // Scan I2C
            i2c_buses = if self.options.scan_i2c {
                self.i2c_discovery.enumerate_buses()
            } else {
                Vec::new()
            };

            i2c_devices = if self.options.scan_i2c && self.options.probe_i2c {
                match self.i2c_discovery.scan_all_buses() {
                    Ok(devices) => devices,
                    Err(e) => {
                        errors.push(format!("I2C scan error: {}", e));
                        Vec::new()
                    }
                }
            } else {
                Vec::new()
            };

            // Add I2C devices to all_devices
            for i2c in &i2c_devices {
                let driver_match = self.database.match_i2c_driver(i2c.address);
                all_devices.push(DiscoveredDevice {
                    category: driver_match.device_info.category.clone(),
                    name: driver_match.device_info.name.clone(),
                    manufacturer: Some(driver_match.device_info.manufacturer.clone()),
                    path: Some(i2c.bus_path.clone()),
                    vid_pid: None,
                    i2c_address: Some(i2c.address),
                    i2c_bus: Some(i2c.bus_number),
                    suggested_driver: driver_match.device_info.horus_driver.clone(),
                    driver_match: Some(driver_match),
                    properties: HashMap::new(),
                });
            }

            // Scan GPIO
            gpio_chips = if self.options.scan_gpio {
                self.gpio_discovery.enumerate()
            } else {
                Vec::new()
            };

            // Scan cameras
            cameras = if self.options.scan_cameras {
                self.camera_discovery.enumerate()
            } else {
                Vec::new()
            };

            // Add cameras to all_devices
            for camera in &cameras {
                let mut properties = HashMap::new();
                properties.insert("driver".to_string(), camera.driver.clone());
                if let Some(card) = &camera.card {
                    properties.insert("card".to_string(), card.clone());
                }

                all_devices.push(DiscoveredDevice {
                    category: match camera.camera_type {
                        super::camera::CameraType::Depth => DeviceCategory::DepthCamera,
                        _ => DeviceCategory::Camera,
                    },
                    name: camera.name.clone(),
                    manufacturer: None,
                    path: Some(camera.device_path.clone()),
                    vid_pid: None,
                    i2c_address: None,
                    i2c_bus: None,
                    suggested_driver: Some("Camera".to_string()),
                    driver_match: None,
                    properties,
                });
            }
        }

        let duration = start.elapsed();

        DiscoveryReport {
            platform,
            capabilities,
            usb_devices,
            serial_ports,
            #[cfg(target_os = "linux")]
            i2c_buses,
            #[cfg(target_os = "linux")]
            i2c_devices,
            #[cfg(target_os = "linux")]
            gpio_chips,
            #[cfg(target_os = "linux")]
            cameras,
            all_devices,
            duration,
            errors,
        }
    }

    /// Quick scan - only USB and serial
    pub fn quick_scan(&mut self) -> DiscoveryReport {
        let original_options = self.options.clone();
        self.options = DiscoveryOptions {
            scan_usb: true,
            scan_serial: true,
            scan_i2c: false,
            probe_i2c: false,
            scan_gpio: false,
            scan_cameras: false,
            probe_timeout: Duration::from_millis(50),
        };

        let result = self.scan_all();
        self.options = original_options;
        result
    }

    /// Find USB devices by VID/PID
    pub fn find_usb_by_vid_pid(&mut self, vid: u16, pid: u16) -> Vec<UsbDevice> {
        self.usb_discovery.enumerate();
        self.usb_discovery
            .devices()
            .iter()
            .filter(|d| d.vendor_id == vid && d.product_id == pid)
            .cloned()
            .collect()
    }

    /// Find serial ports for a specific device type
    pub fn find_serial_for_device(&mut self, vid: u16, pid: u16) -> Vec<SerialPort> {
        self.serial_discovery.enumerate();
        self.serial_discovery.find_by_vid_pid(vid, pid)
            .into_iter()
            .cloned()
            .collect()
    }

    /// Generate a configuration suggestion based on discovered hardware
    pub fn suggest_configuration(&mut self) -> ConfigurationSuggestion {
        let report = self.scan_all();
        let mut nodes = Vec::new();
        let mut warnings = Vec::new();

        for device in &report.all_devices {
            if let Some(driver) = &device.suggested_driver {
                let mut config = HashMap::new();

                if let Some(path) = &device.path {
                    config.insert("device".to_string(), path.display().to_string());
                }

                if let Some((vid, pid)) = device.vid_pid {
                    config.insert("vid".to_string(), format!("0x{:04X}", vid));
                    config.insert("pid".to_string(), format!("0x{:04X}", pid));
                }

                if let Some(addr) = device.i2c_address {
                    config.insert("address".to_string(), format!("0x{:02X}", addr));
                }

                if let Some(bus) = device.i2c_bus {
                    config.insert("bus".to_string(), bus.to_string());
                }

                nodes.push(NodeSuggestion {
                    node_type: driver.clone(),
                    device_name: device.name.clone(),
                    config,
                });
            }
        }

        // Add platform-specific warnings
        if report.capabilities.gpio_chips.is_empty() {
            warnings.push("Platform does not support GPIO".to_string());
        }
        if report.capabilities.i2c_buses.is_empty() {
            warnings.push("Platform does not support I2C".to_string());
        }

        ConfigurationSuggestion {
            platform: report.platform,
            nodes,
            warnings,
        }
    }
}

impl Default for HardwareDiscovery {
    fn default() -> Self {
        Self::new().expect("Failed to create HardwareDiscovery")
    }
}

/// Suggested node configuration
#[derive(Debug, Clone)]
pub struct NodeSuggestion {
    /// HORUS node type
    pub node_type: String,
    /// Device name
    pub device_name: String,
    /// Configuration values
    pub config: HashMap<String, String>,
}

/// Configuration suggestion based on discovered hardware
#[derive(Debug)]
pub struct ConfigurationSuggestion {
    /// Platform
    pub platform: Platform,
    /// Suggested nodes
    pub nodes: Vec<NodeSuggestion>,
    /// Warnings
    pub warnings: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hardware_discovery() {
        let mut discovery = HardwareDiscovery::new().unwrap();
        let report = discovery.scan_all();

        println!("Platform: {:?}", report.platform);
        println!("USB devices: {}", report.usb_devices.len());
        println!("Serial ports: {}", report.serial_ports.len());
        println!("All devices: {}", report.all_devices.len());

        for device in &report.all_devices {
            println!("  {}", device.display_string());
            if let Some(driver) = &device.suggested_driver {
                println!("    -> Suggested driver: {}", driver);
            }
        }
    }

    #[test]
    fn test_quick_scan() {
        let mut discovery = HardwareDiscovery::new().unwrap();
        let report = discovery.quick_scan();
        let summary = report.summary();

        println!("Quick scan found {} devices in {:?}",
                 summary.total_devices,
                 summary.duration);
    }
}
