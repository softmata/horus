//! USB device discovery for HORUS.
//!
//! Enumerates USB devices and provides VID/PID identification.

use std::fs;
use std::path::{Path, PathBuf};

/// USB device information
#[derive(Debug, Clone)]
pub struct UsbDevice {
    /// USB Vendor ID
    pub vendor_id: u16,
    /// USB Product ID
    pub product_id: u16,
    /// Device serial number (if available)
    pub serial: Option<String>,
    /// Manufacturer string
    pub manufacturer: Option<String>,
    /// Product string
    pub product: Option<String>,
    /// USB bus number
    pub bus_number: u8,
    /// Device address on bus
    pub device_address: u8,
    /// Device class
    pub device_class: u8,
    /// Device subclass
    pub device_subclass: u8,
    /// Device protocol
    pub device_protocol: u8,
    /// sysfs path
    pub sysfs_path: PathBuf,
    /// TTY device path (for serial devices)
    pub tty_path: Option<PathBuf>,
    /// Video device path (for cameras)
    pub video_path: Option<PathBuf>,
    /// USB speed (1.5, 12, 480, 5000, 10000, 20000 Mbps)
    pub speed_mbps: Option<u32>,
    /// Number of interfaces
    pub num_interfaces: u8,
    /// USB version (e.g., "2.0", "3.0")
    pub usb_version: Option<String>,
}

impl UsbDevice {
    /// Get VID:PID string
    pub fn vid_pid_string(&self) -> String {
        format!("{:04X}:{:04X}", self.vendor_id, self.product_id)
    }

    /// Get display name (product or VID:PID)
    pub fn display_name(&self) -> String {
        self.product
            .clone()
            .unwrap_or_else(|| self.vid_pid_string())
    }

    /// Check if device has a serial port
    pub fn has_serial(&self) -> bool {
        self.tty_path.is_some()
    }

    /// Check if device is a camera
    pub fn has_video(&self) -> bool {
        self.video_path.is_some()
    }

    /// Check if device is a HID device
    pub fn is_hid(&self) -> bool {
        self.device_class == 3 || self.device_subclass == 3
    }

    /// Check if device is a mass storage device
    pub fn is_storage(&self) -> bool {
        self.device_class == 8
    }

    /// Check if device is an audio device
    pub fn is_audio(&self) -> bool {
        self.device_class == 1
    }

    /// Check if device is a CDC device (serial)
    pub fn is_cdc(&self) -> bool {
        self.device_class == 2 || self.device_class == 10
    }
}

/// USB device discovery
pub struct UsbDiscovery {
    /// Cache of discovered devices
    devices: Vec<UsbDevice>,
}

impl UsbDiscovery {
    /// Create a new USB discovery instance
    pub fn new() -> Self {
        Self {
            devices: Vec::new(),
        }
    }

    /// Enumerate all USB devices
    pub fn enumerate(&mut self) -> Vec<UsbDevice> {
        self.devices.clear();

        #[cfg(target_os = "linux")]
        {
            self.devices = self.enumerate_linux();
        }

        #[cfg(not(target_os = "linux"))]
        {
            // On non-Linux, return empty for now
            self.devices = Vec::new();
        }

        self.devices.clone()
    }

    /// Get cached devices
    pub fn devices(&self) -> &[UsbDevice] {
        &self.devices
    }

    /// Find device by VID/PID
    pub fn find_by_vid_pid(&self, vid: u16, pid: u16) -> Option<&UsbDevice> {
        self.devices
            .iter()
            .find(|d| d.vendor_id == vid && d.product_id == pid)
    }

    /// Find all devices by vendor
    pub fn find_by_vendor(&self, vid: u16) -> Vec<&UsbDevice> {
        self.devices.iter().filter(|d| d.vendor_id == vid).collect()
    }

    /// Find devices with serial ports
    pub fn find_serial_devices(&self) -> Vec<&UsbDevice> {
        self.devices.iter().filter(|d| d.has_serial()).collect()
    }

    /// Find video/camera devices
    pub fn find_video_devices(&self) -> Vec<&UsbDevice> {
        self.devices.iter().filter(|d| d.has_video()).collect()
    }

    #[cfg(target_os = "linux")]
    fn enumerate_linux(&self) -> Vec<UsbDevice> {
        let mut devices = Vec::new();
        let usb_path = Path::new("/sys/bus/usb/devices");

        if !usb_path.exists() {
            return devices;
        }

        if let Ok(entries) = fs::read_dir(usb_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                let name = entry.file_name().to_string_lossy().to_string();

                // Skip non-device entries (like usb1, usb2 - these are hubs)
                // Real devices have format like 1-1, 1-1.1, 2-1, etc.
                if !name.contains('-') {
                    continue;
                }

                // Skip interfaces (they have :X.Y suffix)
                if name.contains(':') {
                    continue;
                }

                if let Some(device) = self.parse_usb_device(&path) {
                    devices.push(device);
                }
            }
        }

        devices
    }

    #[cfg(target_os = "linux")]
    fn parse_usb_device(&self, path: &Path) -> Option<UsbDevice> {
        // Read vendor and product IDs
        let vendor_id = self.read_hex_file(path, "idVendor")?;
        let product_id = self.read_hex_file(path, "idProduct")?;

        // Read device class info
        let device_class = self.read_hex_file(path, "bDeviceClass").unwrap_or(0) as u8;
        let device_subclass = self.read_hex_file(path, "bDeviceSubClass").unwrap_or(0) as u8;
        let device_protocol = self.read_hex_file(path, "bDeviceProtocol").unwrap_or(0) as u8;

        // Read bus/device numbers
        let bus_number = self.read_decimal_file(path, "busnum").unwrap_or(0) as u8;
        let device_address = self.read_decimal_file(path, "devnum").unwrap_or(0) as u8;

        // Read optional strings
        let manufacturer = self.read_string_file(path, "manufacturer");
        let product = self.read_string_file(path, "product");
        let serial = self.read_string_file(path, "serial");

        // Read USB speed
        let speed_mbps = self
            .read_string_file(path, "speed")
            .and_then(|s| s.parse().ok());

        // Read USB version
        let usb_version = self
            .read_string_file(path, "version")
            .map(|s| s.trim().to_string());

        // Read number of interfaces
        let num_interfaces = self.read_decimal_file(path, "bNumInterfaces").unwrap_or(0) as u8;

        // Find associated tty device
        let tty_path = self.find_tty_for_device(path);

        // Find associated video device
        let video_path = self.find_video_for_device(path);

        Some(UsbDevice {
            vendor_id: vendor_id as u16,
            product_id: product_id as u16,
            serial,
            manufacturer,
            product,
            bus_number,
            device_address,
            device_class,
            device_subclass,
            device_protocol,
            sysfs_path: path.to_path_buf(),
            tty_path,
            video_path,
            speed_mbps,
            num_interfaces,
            usb_version,
        })
    }

    #[cfg(target_os = "linux")]
    fn read_hex_file(&self, base: &Path, name: &str) -> Option<u32> {
        let path = base.join(name);
        let content = fs::read_to_string(path).ok()?;
        u32::from_str_radix(content.trim(), 16).ok()
    }

    #[cfg(target_os = "linux")]
    fn read_decimal_file(&self, base: &Path, name: &str) -> Option<u32> {
        let path = base.join(name);
        let content = fs::read_to_string(path).ok()?;
        content.trim().parse().ok()
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
    fn find_tty_for_device(&self, device_path: &Path) -> Option<PathBuf> {
        // Look for tty subdirectories in the device tree
        // Pattern: device_path/*/tty/tty*
        if let Ok(entries) = fs::read_dir(device_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    // Check for tty subdirectory
                    let tty_dir = path.join("tty");
                    if tty_dir.exists() {
                        if let Ok(tty_entries) = fs::read_dir(&tty_dir) {
                            for tty_entry in tty_entries.flatten() {
                                let tty_name = tty_entry.file_name().to_string_lossy().to_string();
                                if tty_name.starts_with("tty") {
                                    return Some(PathBuf::from(format!("/dev/{}", tty_name)));
                                }
                            }
                        }
                    }

                    // Recurse one level (for interfaces)
                    if let Some(tty) = self.find_tty_for_device(&path) {
                        return Some(tty);
                    }
                }
            }
        }
        None
    }

    #[cfg(target_os = "linux")]
    fn find_video_for_device(&self, device_path: &Path) -> Option<PathBuf> {
        // Look for video4linux subdirectories
        // Pattern: device_path/*/video4linux/video*
        if let Ok(entries) = fs::read_dir(device_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    // Check for video4linux subdirectory
                    let v4l_dir = path.join("video4linux");
                    if v4l_dir.exists() {
                        if let Ok(v4l_entries) = fs::read_dir(&v4l_dir) {
                            for v4l_entry in v4l_entries.flatten() {
                                let video_name =
                                    v4l_entry.file_name().to_string_lossy().to_string();
                                if video_name.starts_with("video") {
                                    return Some(PathBuf::from(format!("/dev/{}", video_name)));
                                }
                            }
                        }
                    }

                    // Recurse one level (for interfaces)
                    if let Some(video) = self.find_video_for_device(&path) {
                        return Some(video);
                    }
                }
            }
        }
        None
    }
}

impl Default for UsbDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

/// Well-known USB Vendor IDs
#[allow(dead_code)]
pub mod vendors {
    /// Arduino LLC
    pub const ARDUINO: u16 = 0x2341;
    /// Arduino LLC (alternate)
    pub const ARDUINO_ALT: u16 = 0x2A03;
    /// Espressif (ESP32)
    pub const ESPRESSIF: u16 = 0x303A;
    /// Silicon Labs (CP210x - common for ESP32)
    pub const SILICON_LABS: u16 = 0x10C4;
    /// WCH (CH340/CH341 - Arduino clones)
    pub const WCH: u16 = 0x1A86;
    /// FTDI
    pub const FTDI: u16 = 0x0403;
    /// STMicroelectronics
    pub const STM: u16 = 0x0483;
    /// Texas Instruments
    pub const TI: u16 = 0x0451;
    /// Prolific (PL2303)
    pub const PROLIFIC: u16 = 0x067B;
    /// Intel
    pub const INTEL: u16 = 0x8086;
    /// Microsoft
    pub const MICROSOFT: u16 = 0x045E;
    /// Sony
    pub const SONY: u16 = 0x054C;
    /// Logitech
    pub const LOGITECH: u16 = 0x046D;
    /// u-blox (GPS)
    pub const UBLOX: u16 = 0x1546;
    /// Slamtec (RPLidar)
    pub const SLAMTEC: u16 = 0x10C4; // Uses Silicon Labs chip
    /// ODrive Robotics
    pub const ODRIVE: u16 = 0x1209;
    /// Peak Systems (CAN)
    pub const PEAK: u16 = 0x0C72;
    /// Kvaser (CAN)
    pub const KVASER: u16 = 0x0BFD;
    /// Orbbec (depth cameras)
    pub const ORBBEC: u16 = 0x2BC5;
    /// Stereolabs (ZED cameras)
    pub const STEREOLABS: u16 = 0x2B03;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore] // May hang on some systems due to USB enumeration
    fn test_enumerate_usb() {
        let mut discovery = UsbDiscovery::new();
        let devices = discovery.enumerate();

        println!("Found {} USB devices:", devices.len());
        for device in &devices {
            println!(
                "  {} - {} ({})",
                device.vid_pid_string(),
                device.display_name(),
                device
                    .manufacturer
                    .as_deref()
                    .unwrap_or("Unknown manufacturer")
            );
            if let Some(tty) = &device.tty_path {
                println!("    TTY: {}", tty.display());
            }
            if let Some(video) = &device.video_path {
                println!("    Video: {}", video.display());
            }
        }
    }

    #[test]
    #[ignore] // May hang on some systems due to USB enumeration
    fn test_find_serial_devices() {
        let mut discovery = UsbDiscovery::new();
        discovery.enumerate();
        let serial_devices = discovery.find_serial_devices();

        println!("Found {} USB serial devices:", serial_devices.len());
        for device in serial_devices {
            println!(
                "  {} - {} @ {}",
                device.vid_pid_string(),
                device.display_name(),
                device.tty_path.as_ref().unwrap().display()
            );
        }
    }
}
