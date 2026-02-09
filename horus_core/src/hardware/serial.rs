//! Serial port discovery for HORUS.
//!
//! Discovers serial ports and identifies connected devices by their
//! USB VID/PID or platform-specific paths.

use std::fs;
use std::path::{Path, PathBuf};

/// Type of serial port
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SerialPortType {
    /// USB CDC-ACM device (Arduino, STM32, etc.)
    UsbCdcAcm,
    /// USB serial adapter (FTDI, CP210x, CH340, PL2303)
    UsbSerial,
    /// Platform UART (Raspberry Pi, Jetson, etc.)
    PlatformUart,
    /// Bluetooth serial (rfcomm)
    Bluetooth,
    /// Virtual/pseudo terminal
    PseudoTerminal,
    /// Unknown type
    Unknown,
}

impl SerialPortType {
    /// Get a human-readable description
    pub fn description(&self) -> &'static str {
        match self {
            Self::UsbCdcAcm => "USB CDC-ACM",
            Self::UsbSerial => "USB Serial Adapter",
            Self::PlatformUart => "Platform UART",
            Self::Bluetooth => "Bluetooth Serial",
            Self::PseudoTerminal => "Pseudo Terminal",
            Self::Unknown => "Unknown",
        }
    }
}

/// Serial port information
#[derive(Debug, Clone)]
pub struct SerialPort {
    /// Device path (e.g., /dev/ttyUSB0)
    pub path: PathBuf,
    /// Port type
    pub port_type: SerialPortType,
    /// USB Vendor ID (if USB device)
    pub vendor_id: Option<u16>,
    /// USB Product ID (if USB device)
    pub product_id: Option<u16>,
    /// USB serial number (if available)
    pub serial_number: Option<String>,
    /// Manufacturer string
    pub manufacturer: Option<String>,
    /// Product string
    pub product: Option<String>,
    /// Driver name (e.g., "cp210x", "ch341", "ftdi_sio")
    pub driver: Option<String>,
    /// sysfs path for the device
    pub sysfs_path: Option<PathBuf>,
    /// Description for display
    pub description: Option<String>,
}

impl SerialPort {
    /// Get display name
    pub fn display_name(&self) -> String {
        if let Some(product) = &self.product {
            format!("{} ({})", product, self.path.display())
        } else if let Some(desc) = &self.description {
            format!("{} ({})", desc, self.path.display())
        } else {
            self.path.display().to_string()
        }
    }

    /// Get VID:PID string if USB device
    pub fn vid_pid_string(&self) -> Option<String> {
        match (self.vendor_id, self.product_id) {
            (Some(vid), Some(pid)) => Some(format!("{:04X}:{:04X}", vid, pid)),
            _ => None,
        }
    }

    /// Check if this is a USB device
    pub fn is_usb(&self) -> bool {
        matches!(
            self.port_type,
            SerialPortType::UsbCdcAcm | SerialPortType::UsbSerial
        )
    }

    /// Check if this is a platform UART
    pub fn is_platform_uart(&self) -> bool {
        matches!(self.port_type, SerialPortType::PlatformUart)
    }
}

/// Serial port discovery
pub struct SerialDiscovery {
    /// Discovered ports
    ports: Vec<SerialPort>,
}

impl SerialDiscovery {
    /// Create a new serial discovery instance
    pub fn new() -> Self {
        Self { ports: Vec::new() }
    }

    /// Enumerate all serial ports
    pub fn enumerate(&mut self) -> Vec<SerialPort> {
        self.ports.clear();

        #[cfg(target_os = "linux")]
        {
            self.ports = self.enumerate_linux();
        }

        #[cfg(target_os = "macos")]
        {
            self.ports = self.enumerate_macos();
        }

        #[cfg(target_os = "windows")]
        {
            self.ports = self.enumerate_windows();
        }

        self.ports.clone()
    }

    /// Get cached ports
    pub fn ports(&self) -> &[SerialPort] {
        &self.ports
    }

    /// Find port by path
    pub fn find_by_path(&self, path: &Path) -> Option<&SerialPort> {
        self.ports.iter().find(|p| p.path == path)
    }

    /// Find ports by VID/PID
    pub fn find_by_vid_pid(&self, vid: u16, pid: u16) -> Vec<&SerialPort> {
        self.ports
            .iter()
            .filter(|p| p.vendor_id == Some(vid) && p.product_id == Some(pid))
            .collect()
    }

    /// Find USB serial ports
    pub fn find_usb_ports(&self) -> Vec<&SerialPort> {
        self.ports.iter().filter(|p| p.is_usb()).collect()
    }

    /// Find platform UARTs
    pub fn find_platform_uarts(&self) -> Vec<&SerialPort> {
        self.ports.iter().filter(|p| p.is_platform_uart()).collect()
    }

    #[cfg(target_os = "linux")]
    fn enumerate_linux(&self) -> Vec<SerialPort> {
        let mut ports = Vec::new();

        // Enumerate /dev/tty* devices
        if let Ok(entries) = fs::read_dir("/dev") {
            for entry in entries.flatten() {
                let path = entry.path();
                let name = entry.file_name().to_string_lossy().to_string();

                // Check for serial port patterns
                if self.is_serial_device(&name) {
                    if let Some(port) = self.parse_serial_port_linux(&path, &name) {
                        ports.push(port);
                    }
                }
            }
        }

        // Sort by path for consistent ordering
        ports.sort_by(|a, b| a.path.cmp(&b.path));
        ports
    }

    #[cfg(target_os = "linux")]
    fn is_serial_device(&self, name: &str) -> bool {
        // USB serial devices
        name.starts_with("ttyUSB")
            || name.starts_with("ttyACM")
            // Platform UARTs
            || name.starts_with("ttyAMA")  // Raspberry Pi
            || name.starts_with("ttyS")    // Standard serial (but filter virtual ones)
            || name.starts_with("ttymxc")  // i.MX
            || name.starts_with("ttyTHS")  // Tegra (Jetson)
            || name.starts_with("ttyO")    // OMAP (BeagleBone)
            || name.starts_with("ttySAC")  // Samsung
            || name.starts_with("ttyPS")   // Xilinx
            // Bluetooth
            || name.starts_with("rfcomm")
    }

    #[cfg(target_os = "linux")]
    fn parse_serial_port_linux(&self, path: &Path, name: &str) -> Option<SerialPort> {
        let (port_type, sysfs_path, usb_info) = if name.starts_with("ttyUSB") {
            // USB serial adapter
            let sysfs = self.find_sysfs_for_tty(name);
            let usb_info = sysfs.as_ref().and_then(|p| self.read_usb_info(p));
            (SerialPortType::UsbSerial, sysfs, usb_info)
        } else if name.starts_with("ttyACM") {
            // USB CDC-ACM
            let sysfs = self.find_sysfs_for_tty(name);
            let usb_info = sysfs.as_ref().and_then(|p| self.read_usb_info(p));
            (SerialPortType::UsbCdcAcm, sysfs, usb_info)
        } else if name.starts_with("ttyAMA")
            || name.starts_with("ttyTHS")
            || name.starts_with("ttyO")
            || name.starts_with("ttymxc")
            || name.starts_with("ttySAC")
            || name.starts_with("ttyPS")
        {
            // Platform UART
            let description = self.get_platform_uart_description(name);
            (
                SerialPortType::PlatformUart,
                None,
                Some(UsbInfo {
                    vendor_id: None,
                    product_id: None,
                    serial_number: None,
                    manufacturer: None,
                    product: None,
                    driver: None,
                    description,
                }),
            )
        } else if name.starts_with("ttyS") {
            // Standard serial - skip virtual/non-existent ports
            // Check if it's a real port by verifying sysfs
            let sysfs_path = format!("/sys/class/tty/{}", name);
            if !Path::new(&sysfs_path).exists() {
                return None;
            }
            // Check if it has a device subdirectory (real hardware)
            let device_path = format!("{}/device", sysfs_path);
            if !Path::new(&device_path).exists() {
                return None;
            }
            (SerialPortType::PlatformUart, None, None)
        } else if name.starts_with("rfcomm") {
            (SerialPortType::Bluetooth, None, None)
        } else {
            (SerialPortType::Unknown, None, None)
        };

        Some(SerialPort {
            path: path.to_path_buf(),
            port_type,
            vendor_id: usb_info.as_ref().and_then(|i| i.vendor_id),
            product_id: usb_info.as_ref().and_then(|i| i.product_id),
            serial_number: usb_info.as_ref().and_then(|i| i.serial_number.clone()),
            manufacturer: usb_info.as_ref().and_then(|i| i.manufacturer.clone()),
            product: usb_info.as_ref().and_then(|i| i.product.clone()),
            driver: usb_info.as_ref().and_then(|i| i.driver.clone()),
            sysfs_path,
            description: usb_info.and_then(|i| i.description),
        })
    }

    #[cfg(target_os = "linux")]
    fn find_sysfs_for_tty(&self, tty_name: &str) -> Option<PathBuf> {
        // The sysfs path is at /sys/class/tty/<tty_name>/device
        let tty_sysfs = PathBuf::from(format!("/sys/class/tty/{}/device", tty_name));
        if tty_sysfs.exists() {
            // Follow symlink to get the actual device path
            if let Ok(real_path) = fs::canonicalize(&tty_sysfs) {
                // Walk up to find the USB device (parent of the interface)
                let mut current = real_path.as_path();
                while let Some(parent) = current.parent() {
                    // Check if this is a USB device (has idVendor file)
                    if parent.join("idVendor").exists() {
                        return Some(parent.to_path_buf());
                    }
                    current = parent;
                }
            }
        }
        None
    }

    #[cfg(target_os = "linux")]
    fn read_usb_info(&self, sysfs_path: &Path) -> Option<UsbInfo> {
        let vendor_id = self.read_hex_file(sysfs_path, "idVendor").map(|v| v as u16);
        let product_id = self
            .read_hex_file(sysfs_path, "idProduct")
            .map(|v| v as u16);
        let serial_number = self.read_string_file(sysfs_path, "serial");
        let manufacturer = self.read_string_file(sysfs_path, "manufacturer");
        let product = self.read_string_file(sysfs_path, "product");

        // Try to determine driver
        let driver = self.determine_driver(sysfs_path);

        Some(UsbInfo {
            vendor_id,
            product_id,
            serial_number,
            manufacturer,
            product,
            driver,
            description: None,
        })
    }

    #[cfg(target_os = "linux")]
    fn determine_driver(&self, sysfs_path: &Path) -> Option<String> {
        // Look at interface directories for driver info
        if let Ok(entries) = fs::read_dir(sysfs_path) {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                // Interface directories have format X-Y:Z.N
                if name.contains(':') {
                    let driver_link = entry.path().join("driver");
                    if let Ok(driver_path) = fs::read_link(&driver_link) {
                        if let Some(driver_name) = driver_path.file_name() {
                            return Some(driver_name.to_string_lossy().to_string());
                        }
                    }
                }
            }
        }
        None
    }

    #[cfg(target_os = "linux")]
    fn read_hex_file(&self, base: &Path, name: &str) -> Option<u32> {
        let path = base.join(name);
        let content = fs::read_to_string(path).ok()?;
        u32::from_str_radix(content.trim(), 16).ok()
    }

    #[cfg(target_os = "linux")]
    fn read_string_file(&self, base: &Path, name: &str) -> Option<String> {
        let path = base.join(name);
        let content = fs::read_to_string(path).ok()?;
        let trimmed = content.trim();
        if trimmed.is_empty() {
            None
        } else {
            Some(trimmed.to_string())
        }
    }

    #[cfg(target_os = "linux")]
    fn get_platform_uart_description(&self, name: &str) -> Option<String> {
        if name.starts_with("ttyAMA") {
            Some("Raspberry Pi UART".to_string())
        } else if name.starts_with("ttyTHS") {
            Some("NVIDIA Tegra UART".to_string())
        } else if name.starts_with("ttyO") {
            Some("OMAP/BeagleBone UART".to_string())
        } else if name.starts_with("ttymxc") {
            Some("i.MX UART".to_string())
        } else if name.starts_with("ttySAC") {
            Some("Samsung UART".to_string())
        } else if name.starts_with("ttyPS") {
            Some("Xilinx Zynq UART".to_string())
        } else {
            None
        }
    }

    #[cfg(target_os = "macos")]
    fn enumerate_macos(&self) -> Vec<SerialPort> {
        let mut ports = Vec::new();

        // macOS uses /dev/tty.* and /dev/cu.* for serial devices
        if let Ok(entries) = fs::read_dir("/dev") {
            for entry in entries.flatten() {
                let path = entry.path();
                let name = entry.file_name().to_string_lossy().to_string();

                // Check for serial port patterns
                if (name.starts_with("tty.") || name.starts_with("cu."))
                    && !name.contains("Bluetooth")
                {
                    let port_type = if name.contains("usbserial") || name.contains("usbmodem") {
                        SerialPortType::UsbSerial
                    } else {
                        SerialPortType::Unknown
                    };

                    ports.push(SerialPort {
                        path,
                        port_type,
                        vendor_id: None,
                        product_id: None,
                        serial_number: None,
                        manufacturer: None,
                        product: None,
                        driver: None,
                        sysfs_path: None,
                        description: None,
                    });
                }
            }
        }

        ports
    }

    #[cfg(target_os = "windows")]
    fn enumerate_windows(&self) -> Vec<SerialPort> {
        // Windows implementation would use SetupAPI
        // For now, return empty
        Vec::new()
    }
}

impl Default for SerialDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

/// Internal struct for USB device info
#[cfg(target_os = "linux")]
struct UsbInfo {
    vendor_id: Option<u16>,
    product_id: Option<u16>,
    serial_number: Option<String>,
    manufacturer: Option<String>,
    product: Option<String>,
    driver: Option<String>,
    description: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_serial() {
        let mut discovery = SerialDiscovery::new();
        let ports = discovery.enumerate();

        println!("Found {} serial ports:", ports.len());
        for port in &ports {
            println!(
                "  {} - {} ({})",
                port.path.display(),
                port.display_name(),
                port.port_type.description()
            );
            if let Some(vid_pid) = port.vid_pid_string() {
                println!("    VID:PID: {}", vid_pid);
            }
            if let Some(driver) = &port.driver {
                println!("    Driver: {}", driver);
            }
        }
    }

    #[test]
    fn test_find_usb_ports() {
        let mut discovery = SerialDiscovery::new();
        discovery.enumerate();
        let usb_ports = discovery.find_usb_ports();

        println!("Found {} USB serial ports:", usb_ports.len());
        for port in usb_ports {
            println!("  {}", port.display_name());
        }
    }
}
