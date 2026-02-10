//! Bluetooth adapter and device discovery for HORUS (Linux only).
//!
//! Scans for Bluetooth adapters and their connected devices via sysfs and hciconfig.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Bluetooth adapter state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BluetoothState {
    /// Adapter is powered on and ready
    Up,
    /// Adapter is powered off
    Down,
    /// Adapter is initializing
    Init,
    /// Unknown state
    Unknown,
}

/// Bluetooth adapter type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BluetoothAdapterType {
    /// USB Bluetooth adapter
    Usb,
    /// UART/Serial Bluetooth (common on SBCs)
    Uart,
    /// SDIO Bluetooth (WiFi combo chips)
    Sdio,
    /// Built-in/integrated Bluetooth
    Builtin,
    /// Unknown type
    Unknown,
}

/// Bluetooth device class category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BluetoothDeviceClass {
    /// Computer/laptop
    Computer,
    /// Phone/smartphone
    Phone,
    /// Audio device (headphones, speakers)
    Audio,
    /// Peripheral (keyboard, mouse, gamepad)
    Peripheral,
    /// Imaging device (camera, scanner)
    Imaging,
    /// Wearable device
    Wearable,
    /// Health device
    Health,
    /// Unknown device type
    Unknown,
}

/// Bluetooth adapter information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BluetoothAdapter {
    /// Adapter name (e.g., "hci0")
    pub name: String,
    /// HCI device number
    pub hci_number: u8,
    /// Bluetooth address (MAC)
    pub address: Option<String>,
    /// Adapter type
    pub adapter_type: BluetoothAdapterType,
    /// Current state
    pub state: BluetoothState,
    /// Device manufacturer
    pub manufacturer: Option<String>,
    /// Device name/alias
    pub device_name: Option<String>,
    /// Bluetooth version (4.0, 5.0, etc.)
    pub bt_version: Option<String>,
    /// Whether the adapter supports BLE
    pub ble_support: bool,
    /// Sysfs path
    pub sysfs_path: PathBuf,
    /// Connected devices
    pub connected_devices: Vec<BluetoothDevice>,
}

impl BluetoothAdapter {
    /// Get device specification string
    pub fn device_spec(&self) -> String {
        self.name.clone()
    }

    /// Check if adapter is ready for use
    pub fn is_ready(&self) -> bool {
        matches!(self.state, BluetoothState::Up)
    }
}

/// Connected Bluetooth device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BluetoothDevice {
    /// Device Bluetooth address
    pub address: String,
    /// Device name
    pub name: Option<String>,
    /// Device class category
    pub device_class: BluetoothDeviceClass,
    /// Whether currently connected
    pub connected: bool,
    /// Whether paired
    pub paired: bool,
    /// Whether trusted
    pub trusted: bool,
    /// Signal strength (RSSI) if available
    pub rssi: Option<i16>,
}

impl BluetoothDevice {
    /// Get device specification string
    pub fn device_spec(&self) -> String {
        format!(
            "{}:{}",
            self.address,
            self.name.as_deref().unwrap_or("Unknown")
        )
    }
}

/// Bluetooth discovery
pub struct BluetoothDiscovery {
    /// Discovered adapters
    adapters: Vec<BluetoothAdapter>,
}

impl BluetoothDiscovery {
    /// Create a new Bluetooth discovery instance
    pub fn new() -> Self {
        Self {
            adapters: Vec::new(),
        }
    }

    /// Enumerate Bluetooth adapters
    pub fn enumerate_adapters(&mut self) -> Vec<BluetoothAdapter> {
        self.adapters.clear();

        // Scan /sys/class/bluetooth for hci* entries
        if let Ok(entries) = fs::read_dir("/sys/class/bluetooth") {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                if let Some(suffix) = name.strip_prefix("hci") {
                    if let Ok(hci_num) = suffix.parse::<u8>() {
                        if let Some(adapter) = self.probe_adapter(hci_num, entry.path()) {
                            self.adapters.push(adapter);
                        }
                    }
                }
            }
        }

        // Sort by HCI number
        self.adapters.sort_by_key(|a| a.hci_number);
        self.adapters.clone()
    }

    /// Get cached adapters
    pub fn adapters(&self) -> &[BluetoothAdapter] {
        &self.adapters
    }

    /// Get adapter count
    pub fn adapter_count(&self) -> usize {
        self.adapters.len()
    }

    /// Get total connected device count
    pub fn connected_device_count(&self) -> usize {
        self.adapters
            .iter()
            .map(|a| a.connected_devices.iter().filter(|d| d.connected).count())
            .sum()
    }

    fn probe_adapter(&self, hci_number: u8, sysfs_path: PathBuf) -> Option<BluetoothAdapter> {
        let name = format!("hci{}", hci_number);

        // Get Bluetooth address
        let address = self.read_sysfs_string(&sysfs_path.join("address"));

        // Determine adapter type
        let adapter_type = self.detect_adapter_type(&sysfs_path);

        // Get state from operstate or via hciconfig
        let state = self.get_adapter_state(&name);

        // Get manufacturer
        let manufacturer = self.get_manufacturer(&sysfs_path);

        // Get device name
        let device_name = self.read_sysfs_string(&sysfs_path.join("name"));

        // Get Bluetooth version
        let bt_version = self.get_bt_version(&sysfs_path);

        // Check BLE support
        let ble_support = self.check_ble_support(&sysfs_path);

        // Scan for connected devices
        let connected_devices = self.scan_connected_devices(&sysfs_path);

        Some(BluetoothAdapter {
            name,
            hci_number,
            address,
            adapter_type,
            state,
            manufacturer,
            device_name,
            bt_version,
            ble_support,
            sysfs_path,
            connected_devices,
        })
    }

    fn detect_adapter_type(&self, sysfs_path: &Path) -> BluetoothAdapterType {
        // Check device subsystem
        let device_link = sysfs_path.join("device");
        if let Ok(target) = fs::read_link(&device_link) {
            let path_str = target.to_string_lossy().to_string();

            if path_str.contains("/usb") {
                return BluetoothAdapterType::Usb;
            }
            if path_str.contains("/serial") || path_str.contains("/tty") {
                return BluetoothAdapterType::Uart;
            }
            if path_str.contains("/sdio") || path_str.contains("/mmc") {
                return BluetoothAdapterType::Sdio;
            }
            if path_str.contains("/platform") {
                return BluetoothAdapterType::Builtin;
            }
        }

        BluetoothAdapterType::Unknown
    }

    fn get_adapter_state(&self, name: &str) -> BluetoothState {
        // Try to get state via hciconfig
        if let Ok(output) = Command::new("hciconfig").arg(name).output() {
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.contains("UP RUNNING") {
                return BluetoothState::Up;
            }
            if stdout.contains("DOWN") {
                return BluetoothState::Down;
            }
            if stdout.contains("INIT") {
                return BluetoothState::Init;
            }
        }

        BluetoothState::Unknown
    }

    fn get_manufacturer(&self, sysfs_path: &Path) -> Option<String> {
        // Try USB manufacturer
        let usb_mfr = sysfs_path.join("device/manufacturer");
        if let Some(mfr) = self.read_sysfs_string(&usb_mfr) {
            return Some(mfr);
        }

        // Try to identify from device ID
        let device_id = sysfs_path.join("device/device_id");
        if let Some(id) = self.read_sysfs_string(&device_id) {
            return self.manufacturer_from_id(&id);
        }

        // Try modalias
        let modalias = sysfs_path.join("device/modalias");
        if let Some(alias) = self.read_sysfs_string(&modalias) {
            return self.manufacturer_from_modalias(&alias);
        }

        None
    }

    fn manufacturer_from_id(&self, id: &str) -> Option<String> {
        // Common Bluetooth chip manufacturers by device ID patterns
        if id.contains("8087") {
            Some("Intel".to_string())
        } else if id.contains("0489") {
            Some("Foxconn".to_string())
        } else if id.contains("0a5c") {
            Some("Broadcom".to_string())
        } else if id.contains("0cf3") {
            Some("Qualcomm/Atheros".to_string())
        } else if id.contains("13d3") {
            Some("IMC Networks".to_string())
        } else if id.contains("0bda") {
            Some("Realtek".to_string())
        } else if id.contains("8086") {
            Some("Intel".to_string())
        } else {
            None
        }
    }

    fn manufacturer_from_modalias(&self, alias: &str) -> Option<String> {
        if alias.contains("v8087") || alias.contains("intel") {
            Some("Intel".to_string())
        } else if alias.contains("v0A5C") || alias.contains("broadcom") {
            Some("Broadcom".to_string())
        } else if alias.contains("v0CF3") || alias.contains("qualcomm") {
            Some("Qualcomm/Atheros".to_string())
        } else if alias.contains("v0BDA") || alias.contains("realtek") {
            Some("Realtek".to_string())
        } else if alias.contains("cypress") || alias.contains("infineon") {
            Some("Infineon/Cypress".to_string())
        } else {
            None
        }
    }

    fn get_bt_version(&self, sysfs_path: &Path) -> Option<String> {
        // Try to read HCI version
        let hci_version = sysfs_path.join("hci_version");
        if let Some(ver) = self.read_sysfs_string(&hci_version) {
            if let Ok(v) = ver.parse::<u8>() {
                return Some(self.version_string(v));
            }
        }

        // Check device tree for version hints
        let compat = sysfs_path.join("device/of_node/compatible");
        if let Some(c) = self.read_sysfs_string(&compat) {
            if c.contains("bluetooth-5") {
                return Some("5.x".to_string());
            }
            if c.contains("bluetooth-4") {
                return Some("4.x".to_string());
            }
        }

        None
    }

    fn version_string(&self, hci_version: u8) -> String {
        match hci_version {
            0 => "1.0b".to_string(),
            1 => "1.1".to_string(),
            2 => "1.2".to_string(),
            3 => "2.0".to_string(),
            4 => "2.1".to_string(),
            5 => "3.0".to_string(),
            6 => "4.0".to_string(),
            7 => "4.1".to_string(),
            8 => "4.2".to_string(),
            9 => "5.0".to_string(),
            10 => "5.1".to_string(),
            11 => "5.2".to_string(),
            12 => "5.3".to_string(),
            13 => "5.4".to_string(),
            _ => format!("HCI {}", hci_version),
        }
    }

    fn check_ble_support(&self, sysfs_path: &Path) -> bool {
        // BLE is supported from Bluetooth 4.0+
        if let Some(ver) = self.get_bt_version(sysfs_path) {
            if let Some(major) = ver.chars().next().and_then(|c| c.to_digit(10)) {
                return major >= 4;
            }
        }

        // Check for LE features file
        let le_states = sysfs_path.join("le_states");
        le_states.exists()
    }

    fn scan_connected_devices(&self, sysfs_path: &Path) -> Vec<BluetoothDevice> {
        let mut devices = Vec::new();

        // Scan for device entries in the adapter's sysfs directory
        if let Ok(entries) = fs::read_dir(sysfs_path) {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                // Bluetooth addresses are in format XX:XX:XX:XX:XX:XX
                if name.contains(':') && name.len() == 17 {
                    if let Some(device) = self.probe_device(&entry.path(), &name) {
                        devices.push(device);
                    }
                }
            }
        }

        devices
    }

    fn probe_device(&self, device_path: &Path, address: &str) -> Option<BluetoothDevice> {
        let name = self.read_sysfs_string(&device_path.join("name"));

        let connected = self
            .read_sysfs_string(&device_path.join("connected"))
            .map(|s| s == "1")
            .unwrap_or(false);

        let paired = self
            .read_sysfs_string(&device_path.join("paired"))
            .map(|s| s == "1")
            .unwrap_or(false);

        let trusted = self
            .read_sysfs_string(&device_path.join("trusted"))
            .map(|s| s == "1")
            .unwrap_or(false);

        // Parse device class
        let device_class = self
            .read_sysfs_string(&device_path.join("class"))
            .map(|s| self.parse_device_class(&s))
            .unwrap_or(BluetoothDeviceClass::Unknown);

        Some(BluetoothDevice {
            address: address.to_string(),
            name,
            device_class,
            connected,
            paired,
            trusted,
            rssi: None, // Would need active query
        })
    }

    fn parse_device_class(&self, class_str: &str) -> BluetoothDeviceClass {
        // Parse Bluetooth device class (24-bit value)
        // Major device class is in bits 8-12
        if let Ok(class) = u32::from_str_radix(class_str.trim_start_matches("0x"), 16) {
            let major_class = (class >> 8) & 0x1F;
            match major_class {
                1 => BluetoothDeviceClass::Computer,
                2 => BluetoothDeviceClass::Phone,
                4 => BluetoothDeviceClass::Audio,
                5 => BluetoothDeviceClass::Peripheral,
                6 => BluetoothDeviceClass::Imaging,
                7 => BluetoothDeviceClass::Wearable,
                9 => BluetoothDeviceClass::Health,
                _ => BluetoothDeviceClass::Unknown,
            }
        } else {
            BluetoothDeviceClass::Unknown
        }
    }

    fn read_sysfs_string(&self, path: &PathBuf) -> Option<String> {
        fs::read_to_string(path)
            .ok()
            .map(|s| s.trim().trim_end_matches('\0').to_string())
            .filter(|s| !s.is_empty())
    }
}

impl Default for BluetoothDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_adapters() {
        let mut discovery = BluetoothDiscovery::new();
        let adapters = discovery.enumerate_adapters();

        println!("Found {} Bluetooth adapters:", adapters.len());
        for adapter in &adapters {
            println!("  {}: {:?}", adapter.name, adapter.state);
            if let Some(ref addr) = adapter.address {
                println!("    Address: {}", addr);
            }
            if let Some(ref mfr) = adapter.manufacturer {
                println!("    Manufacturer: {}", mfr);
            }
            if let Some(ref ver) = adapter.bt_version {
                println!("    Bluetooth: {}", ver);
            }
            println!("    Type: {:?}", adapter.adapter_type);
            println!("    BLE support: {}", adapter.ble_support);
            println!("    Connected devices: {}", adapter.connected_devices.len());

            for device in &adapter.connected_devices {
                println!(
                    "      {} - {} ({:?})",
                    device.address,
                    device.name.as_deref().unwrap_or("Unknown"),
                    device.device_class
                );
            }
        }
    }

    #[test]
    fn test_device_class_parsing() {
        let discovery = BluetoothDiscovery::new();

        // Audio device (major class 4)
        assert_eq!(
            discovery.parse_device_class("0x240404"),
            BluetoothDeviceClass::Audio
        );

        // Peripheral (major class 5)
        assert_eq!(
            discovery.parse_device_class("0x002540"),
            BluetoothDeviceClass::Peripheral
        );

        // Computer (major class 1)
        assert_eq!(
            discovery.parse_device_class("0x3e0100"),
            BluetoothDeviceClass::Computer
        );
    }
}
