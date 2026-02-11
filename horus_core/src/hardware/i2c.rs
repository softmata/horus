//! I2C bus and device discovery for HORUS (Linux only).
//!
//! Scans for available I2C buses and probes for connected devices.

use std::fs;
use std::os::unix::io::AsRawFd;
use std::path::PathBuf;

/// I2C bus information
#[derive(Debug, Clone)]
pub struct I2cBus {
    /// Bus number
    pub bus_number: u8,
    /// Device path
    pub device_path: PathBuf,
    /// Bus name/description
    pub name: Option<String>,
    /// Adapter type
    pub adapter_type: Option<String>,
}

impl I2cBus {
    /// Get the device path as a string
    pub fn device(&self) -> String {
        self.device_path.display().to_string()
    }
}

/// I2C device found on a bus
#[derive(Debug, Clone)]
pub struct I2cDevice {
    /// I2C address (7-bit)
    pub address: u8,
    /// Bus number
    pub bus_number: u8,
    /// Bus path
    pub bus_path: PathBuf,
    /// Device name (if identified)
    pub name: Option<String>,
}

impl I2cDevice {
    /// Get full device specification
    pub fn device_spec(&self) -> String {
        format!("/dev/i2c-{}@0x{:02X}", self.bus_number, self.address)
    }
}

/// I2C discovery
pub struct I2cDiscovery {
    /// Discovered buses
    buses: Vec<I2cBus>,
}

impl I2cDiscovery {
    /// Create a new I2C discovery instance
    pub fn new() -> Self {
        Self { buses: Vec::new() }
    }

    /// Enumerate I2C buses
    pub fn enumerate_buses(&mut self) -> Vec<I2cBus> {
        self.buses.clear();

        // Scan /dev for i2c-* devices
        if let Ok(entries) = fs::read_dir("/dev") {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                if let Some(suffix) = name.strip_prefix("i2c-") {
                    if let Ok(bus_num) = suffix.parse::<u8>() {
                        let device_path = entry.path();

                        // Try to get adapter name from sysfs
                        let adapter_name = self.get_adapter_name(bus_num);
                        let adapter_type = self.get_adapter_type(bus_num);

                        self.buses.push(I2cBus {
                            bus_number: bus_num,
                            device_path,
                            name: adapter_name,
                            adapter_type,
                        });
                    }
                }
            }
        }

        // Also check /sys/class/i2c-adapter for buses without /dev entries
        if let Ok(entries) = fs::read_dir("/sys/class/i2c-adapter") {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                if let Some(suffix) = name.strip_prefix("i2c-") {
                    if let Ok(bus_num) = suffix.parse::<u8>() {
                        // Skip if already found
                        if self.buses.iter().any(|b| b.bus_number == bus_num) {
                            continue;
                        }

                        let device_path = PathBuf::from(format!("/dev/i2c-{}", bus_num));
                        let adapter_name = self.get_adapter_name(bus_num);
                        let adapter_type = self.get_adapter_type(bus_num);

                        self.buses.push(I2cBus {
                            bus_number: bus_num,
                            device_path,
                            name: adapter_name,
                            adapter_type,
                        });
                    }
                }
            }
        }

        // Sort by bus number
        self.buses.sort_by_key(|b| b.bus_number);
        self.buses.clone()
    }

    /// Get cached buses
    pub fn buses(&self) -> &[I2cBus] {
        &self.buses
    }

    /// Scan a specific bus for devices
    pub fn scan_bus(&self, bus_number: u8) -> Result<Vec<I2cDevice>, String> {
        let device_path = PathBuf::from(format!("/dev/i2c-{}", bus_number));

        if !device_path.exists() {
            return Err(format!("I2C bus {} not found", bus_number));
        }

        let file = fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&device_path)
            .map_err(|e| format!("Failed to open I2C bus: {}", e))?;

        let mut devices = Vec::new();

        // Scan addresses 0x08 to 0x77 (standard I2C address range)
        for addr in 0x08..=0x77 {
            // Skip reserved addresses
            if self.is_reserved_address(addr) {
                continue;
            }

            if self.probe_address(&file, addr) {
                devices.push(I2cDevice {
                    address: addr,
                    bus_number,
                    bus_path: device_path.clone(),
                    name: None,
                });
            }
        }

        Ok(devices)
    }

    /// Scan all buses
    pub fn scan_all_buses(&mut self) -> Result<Vec<I2cDevice>, String> {
        self.enumerate_buses();

        let mut all_devices = Vec::new();
        for bus in &self.buses {
            match self.scan_bus(bus.bus_number) {
                Ok(devices) => all_devices.extend(devices),
                Err(_) => {
                    // Skip buses we can't access
                    continue;
                }
            }
        }

        Ok(all_devices)
    }

    fn get_adapter_name(&self, bus_number: u8) -> Option<String> {
        let name_path = format!("/sys/class/i2c-adapter/i2c-{}/name", bus_number);
        fs::read_to_string(&name_path)
            .ok()
            .map(|s| s.trim().to_string())
    }

    fn get_adapter_type(&self, bus_number: u8) -> Option<String> {
        // Try to determine adapter type from name or device tree
        let name = self.get_adapter_name(bus_number)?;

        if name.contains("bcm2835") || name.contains("bcm2711") {
            Some("Raspberry Pi I2C".to_string())
        } else if name.contains("tegra") {
            Some("NVIDIA Tegra I2C".to_string())
        } else if name.contains("omap") {
            Some("TI OMAP I2C".to_string())
        } else if name.contains("i.MX") || name.contains("imx") {
            Some("NXP i.MX I2C".to_string())
        } else if name.contains("designware") {
            Some("Synopsys DesignWare I2C".to_string())
        } else if name.contains("i801") {
            Some("Intel I2C".to_string())
        } else {
            None
        }
    }

    fn is_reserved_address(&self, addr: u8) -> bool {
        // Reserved I2C addresses
        matches!(addr, 0x00..=0x07 | 0x78..=0x7F)
    }

    fn probe_address(&self, file: &fs::File, addr: u8) -> bool {
        // Use I2C_SLAVE ioctl to set slave address
        const I2C_SLAVE: u64 = 0x0703;

        let fd = file.as_raw_fd();
        // SAFETY: fd is a valid open I2C device. I2C_SLAVE sets the target slave address for subsequent operations.
        let result = unsafe { libc::ioctl(fd, I2C_SLAVE as libc::c_ulong, addr as libc::c_int) };

        if result < 0 {
            return false;
        }

        // Try a quick read to see if device responds
        // We use SMBus quick write (just address, no data)
        const I2C_SMBUS: u64 = 0x0720;
        const I2C_SMBUS_QUICK: u8 = 0;
        const I2C_SMBUS_WRITE: u8 = 0;

        #[repr(C)]
        struct I2cSmbusIoctlData {
            read_write: u8,
            command: u8,
            size: u32,
            data: *mut u8,
        }

        let mut data = I2cSmbusIoctlData {
            read_write: I2C_SMBUS_WRITE,
            command: 0,
            size: I2C_SMBUS_QUICK as u32,
            data: std::ptr::null_mut(),
        };

        // SAFETY: fd is a valid open I2C device. data is a properly initialized I2cSmbusIoctlData struct.
        let result = unsafe { libc::ioctl(fd, I2C_SMBUS as libc::c_ulong, &mut data as *mut _) };

        result >= 0
    }
}

impl Default for I2cDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_buses() {
        let mut discovery = I2cDiscovery::new();
        let buses = discovery.enumerate_buses();

        println!("Found {} I2C buses:", buses.len());
        for bus in &buses {
            println!(
                "  Bus {}: {} - {}",
                bus.bus_number,
                bus.device_path.display(),
                bus.name.as_deref().unwrap_or("Unknown")
            );
        }
    }

    #[test]
    #[ignore] // Requires I2C hardware and possibly root
    fn test_scan_bus() {
        let discovery = I2cDiscovery::new();
        match discovery.scan_bus(1) {
            Ok(devices) => {
                println!("Found {} devices on bus 1:", devices.len());
                for device in &devices {
                    println!("  0x{:02X}", device.address);
                }
            }
            Err(e) => {
                println!("Failed to scan bus: {}", e);
            }
        }
    }
}
