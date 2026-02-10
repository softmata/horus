//! Network interface discovery for HORUS (Linux only).
//!
//! Scans for available network interfaces including Ethernet, WiFi, and virtual interfaces.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Network interface type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NetworkInterfaceType {
    /// Physical Ethernet
    Ethernet,
    /// WiFi/Wireless
    Wifi,
    /// Loopback (lo)
    Loopback,
    /// Virtual Ethernet (veth)
    Veth,
    /// Bridge interface
    Bridge,
    /// Bond/team interface
    Bond,
    /// VLAN interface
    Vlan,
    /// TUN/TAP virtual interface
    TunTap,
    /// Docker/container virtual interface
    Docker,
    /// USB Ethernet adapter
    UsbEthernet,
    /// USB WiFi adapter
    UsbWifi,
    /// Unknown type
    Unknown,
}

/// Network interface state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NetworkState {
    /// Interface is up and carrier detected
    Up,
    /// Interface is up but no carrier
    NoCarrier,
    /// Interface is administratively down
    Down,
    /// Interface state unknown
    Unknown,
}

/// WiFi-specific information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WifiInfo {
    /// Connected SSID
    pub ssid: Option<String>,
    /// Signal strength (dBm)
    pub signal_dbm: Option<i32>,
    /// Link quality (percentage)
    pub quality_percent: Option<u8>,
    /// Current frequency (MHz)
    pub frequency_mhz: Option<u32>,
    /// WiFi mode (managed, AP, monitor, etc.)
    pub mode: Option<String>,
    /// Supported standards (802.11a/b/g/n/ac/ax)
    pub standards: Vec<String>,
}

/// Network interface information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkInterface {
    /// Interface name (e.g., "eth0", "wlan0")
    pub name: String,
    /// Interface type
    pub interface_type: NetworkInterfaceType,
    /// Current state
    pub state: NetworkState,
    /// MAC address
    pub mac_address: Option<String>,
    /// IPv4 addresses
    pub ipv4_addresses: Vec<String>,
    /// IPv6 addresses
    pub ipv6_addresses: Vec<String>,
    /// Maximum transmission unit
    pub mtu: Option<u32>,
    /// Link speed in Mbps (for Ethernet)
    pub speed_mbps: Option<u32>,
    /// Driver name
    pub driver: Option<String>,
    /// Device description
    pub description: Option<String>,
    /// WiFi-specific info
    pub wifi_info: Option<WifiInfo>,
    /// TX bytes
    pub tx_bytes: u64,
    /// RX bytes
    pub rx_bytes: u64,
    /// TX packets
    pub tx_packets: u64,
    /// RX packets
    pub rx_packets: u64,
    /// Sysfs path
    pub sysfs_path: PathBuf,
}

impl NetworkInterface {
    /// Get device specification string
    pub fn device_spec(&self) -> String {
        self.name.clone()
    }

    /// Check if interface is physical (not virtual)
    pub fn is_physical(&self) -> bool {
        matches!(
            self.interface_type,
            NetworkInterfaceType::Ethernet
                | NetworkInterfaceType::Wifi
                | NetworkInterfaceType::UsbEthernet
                | NetworkInterfaceType::UsbWifi
        )
    }

    /// Check if interface is up and has carrier
    pub fn is_connected(&self) -> bool {
        matches!(self.state, NetworkState::Up)
    }

    /// Get formatted speed string
    pub fn speed_string(&self) -> Option<String> {
        self.speed_mbps.map(|s| {
            if s >= 1000 {
                format!("{} Gbps", s / 1000)
            } else {
                format!("{} Mbps", s)
            }
        })
    }
}

/// Network interface discovery
pub struct NetworkDiscovery {
    /// Discovered interfaces
    interfaces: Vec<NetworkInterface>,
}

impl NetworkDiscovery {
    /// Create a new network discovery instance
    pub fn new() -> Self {
        Self {
            interfaces: Vec::new(),
        }
    }

    /// Enumerate network interfaces
    pub fn enumerate_interfaces(&mut self) -> Vec<NetworkInterface> {
        self.interfaces.clear();

        // Scan /sys/class/net for interfaces
        if let Ok(entries) = fs::read_dir("/sys/class/net") {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                if let Some(iface) = self.probe_interface(&name, entry.path()) {
                    self.interfaces.push(iface);
                }
            }
        }

        // Sort: physical first, then by name
        self.interfaces.sort_by(|a, b| {
            match (a.is_physical(), b.is_physical()) {
                (true, false) => std::cmp::Ordering::Less,
                (false, true) => std::cmp::Ordering::Greater,
                _ => {
                    // Put loopback last among virtual
                    if matches!(a.interface_type, NetworkInterfaceType::Loopback) {
                        std::cmp::Ordering::Greater
                    } else if matches!(b.interface_type, NetworkInterfaceType::Loopback) {
                        std::cmp::Ordering::Less
                    } else {
                        a.name.cmp(&b.name)
                    }
                }
            }
        });

        self.interfaces.clone()
    }

    /// Get cached interfaces
    pub fn interfaces(&self) -> &[NetworkInterface] {
        &self.interfaces
    }

    /// Get physical interfaces only
    pub fn physical_interfaces(&self) -> Vec<&NetworkInterface> {
        self.interfaces.iter().filter(|i| i.is_physical()).collect()
    }

    /// Get connected interfaces
    pub fn connected_interfaces(&self) -> Vec<&NetworkInterface> {
        self.interfaces
            .iter()
            .filter(|i| i.is_connected())
            .collect()
    }

    fn probe_interface(&self, name: &str, sysfs_path: PathBuf) -> Option<NetworkInterface> {
        // Determine interface type
        let interface_type = self.detect_interface_type(name, &sysfs_path);

        // Get state
        let state = self.get_state(&sysfs_path);

        // Get MAC address
        let mac_address = self.read_sysfs_string(&sysfs_path.join("address"));

        // Get MTU
        let mtu = self.read_sysfs_u32(&sysfs_path.join("mtu"));

        // Get speed (Ethernet only, returns -1 for wireless)
        let speed_mbps = self
            .read_sysfs_u32(&sysfs_path.join("speed"))
            .filter(|&s| s <= 400000);

        // Get driver
        let driver = self.get_driver(&sysfs_path);

        // Get description
        let description = self.get_description(&interface_type, &driver);

        // Get IP addresses
        let (ipv4_addresses, ipv6_addresses) = self.get_ip_addresses(name);

        // Get WiFi info if applicable
        let wifi_info = if matches!(
            interface_type,
            NetworkInterfaceType::Wifi | NetworkInterfaceType::UsbWifi
        ) {
            self.get_wifi_info(name)
        } else {
            None
        };

        // Get statistics
        let tx_bytes = self
            .read_sysfs_u64(&sysfs_path.join("statistics/tx_bytes"))
            .unwrap_or(0);
        let rx_bytes = self
            .read_sysfs_u64(&sysfs_path.join("statistics/rx_bytes"))
            .unwrap_or(0);
        let tx_packets = self
            .read_sysfs_u64(&sysfs_path.join("statistics/tx_packets"))
            .unwrap_or(0);
        let rx_packets = self
            .read_sysfs_u64(&sysfs_path.join("statistics/rx_packets"))
            .unwrap_or(0);

        Some(NetworkInterface {
            name: name.to_string(),
            interface_type,
            state,
            mac_address,
            ipv4_addresses,
            ipv6_addresses,
            mtu,
            speed_mbps,
            driver,
            description,
            wifi_info,
            tx_bytes,
            rx_bytes,
            tx_packets,
            rx_packets,
            sysfs_path,
        })
    }

    fn detect_interface_type(&self, name: &str, sysfs_path: &Path) -> NetworkInterfaceType {
        // Check by name pattern first
        if name == "lo" {
            return NetworkInterfaceType::Loopback;
        }
        if name.starts_with("docker") || name.starts_with("br-") {
            return NetworkInterfaceType::Docker;
        }
        if name.starts_with("veth") {
            return NetworkInterfaceType::Veth;
        }
        if name.starts_with("virbr") || name.starts_with("br") {
            return NetworkInterfaceType::Bridge;
        }
        if name.starts_with("bond") || name.starts_with("team") {
            return NetworkInterfaceType::Bond;
        }
        if name.contains('.') && !name.contains(':') {
            return NetworkInterfaceType::Vlan;
        }
        if name.starts_with("tun") || name.starts_with("tap") {
            return NetworkInterfaceType::TunTap;
        }

        // Check device type
        let type_path = sysfs_path.join("type");
        if let Some(type_str) = self.read_sysfs_string(&type_path) {
            if type_str == "1" {
                // Ethernet type, but could be WiFi
                let wireless_path = sysfs_path.join("wireless");
                let phy80211_path = sysfs_path.join("phy80211");
                if wireless_path.exists() || phy80211_path.exists() {
                    // Check if USB WiFi
                    if self.is_usb_device(sysfs_path) {
                        return NetworkInterfaceType::UsbWifi;
                    }
                    return NetworkInterfaceType::Wifi;
                }

                // Check if USB Ethernet
                if self.is_usb_device(sysfs_path) {
                    return NetworkInterfaceType::UsbEthernet;
                }

                return NetworkInterfaceType::Ethernet;
            }
        }

        // Check for wireless by looking for phy80211
        if sysfs_path.join("phy80211").exists() {
            if self.is_usb_device(sysfs_path) {
                return NetworkInterfaceType::UsbWifi;
            }
            return NetworkInterfaceType::Wifi;
        }

        NetworkInterfaceType::Unknown
    }

    fn is_usb_device(&self, sysfs_path: &Path) -> bool {
        let device_link = sysfs_path.join("device");
        if let Ok(target) = fs::read_link(&device_link) {
            return target.to_string_lossy().contains("/usb");
        }
        false
    }

    fn get_state(&self, sysfs_path: &Path) -> NetworkState {
        let operstate = self.read_sysfs_string(&sysfs_path.join("operstate"));
        let carrier = self.read_sysfs_string(&sysfs_path.join("carrier"));

        match operstate.as_deref() {
            Some("up") => {
                if carrier.as_deref() == Some("1") {
                    NetworkState::Up
                } else {
                    NetworkState::NoCarrier
                }
            }
            Some("down") => NetworkState::Down,
            Some("dormant") => NetworkState::NoCarrier,
            _ => NetworkState::Unknown,
        }
    }

    fn get_driver(&self, sysfs_path: &Path) -> Option<String> {
        let driver_link = sysfs_path.join("device/driver");
        fs::read_link(&driver_link)
            .ok()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().to_string()))
    }

    fn get_description(
        &self,
        itype: &NetworkInterfaceType,
        driver: &Option<String>,
    ) -> Option<String> {
        match itype {
            NetworkInterfaceType::Loopback => Some("Loopback interface".to_string()),
            NetworkInterfaceType::Docker => Some("Docker network".to_string()),
            NetworkInterfaceType::Bridge => Some("Network bridge".to_string()),
            NetworkInterfaceType::Bond => Some("Bonded interface".to_string()),
            NetworkInterfaceType::Veth => Some("Virtual Ethernet pair".to_string()),
            NetworkInterfaceType::TunTap => Some("TUN/TAP interface".to_string()),
            NetworkInterfaceType::Vlan => Some("VLAN interface".to_string()),
            NetworkInterfaceType::UsbEthernet => Some("USB Ethernet adapter".to_string()),
            NetworkInterfaceType::UsbWifi => Some("USB WiFi adapter".to_string()),
            _ => driver.as_ref().map(|d| self.friendly_driver_name(d)),
        }
    }

    fn friendly_driver_name(&self, driver: &str) -> String {
        match driver {
            "e1000" | "e1000e" => "Intel Gigabit Ethernet".to_string(),
            "igb" | "igc" => "Intel Ethernet".to_string(),
            "r8169" | "r8168" => "Realtek Gigabit Ethernet".to_string(),
            "ath9k" | "ath10k_pci" | "ath11k" => "Qualcomm/Atheros WiFi".to_string(),
            "iwlwifi" | "iwldvm" | "iwlmvm" => "Intel WiFi".to_string(),
            "brcmfmac" | "brcmsmac" => "Broadcom WiFi".to_string(),
            "rtw88_8822ce" | "rtw89" | "rtl8xxxu" => "Realtek WiFi".to_string(),
            "mt7921e" | "mt76" => "MediaTek WiFi".to_string(),
            "smsc95xx" | "smsc75xx" => "SMSC USB Ethernet".to_string(),
            "ax88179_178a" => "ASIX USB Ethernet".to_string(),
            "r8152" => "Realtek USB Ethernet".to_string(),
            "cdc_ether" | "cdc_ncm" => "USB CDC Ethernet".to_string(),
            "bcmgenet" => "Broadcom Gigabit Ethernet".to_string(),
            "stmmac" | "dwmac" => "Synopsys Ethernet".to_string(),
            _ => format!("{} driver", driver),
        }
    }

    fn get_ip_addresses(&self, name: &str) -> (Vec<String>, Vec<String>) {
        let mut ipv4 = Vec::new();
        let mut ipv6 = Vec::new();

        // Try to read from /proc/net/fib_trie for IPv4
        // This is complex, so we'll use ip command if available
        if let Ok(output) = std::process::Command::new("ip")
            .args(["addr", "show", name])
            .output()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);
            for line in stdout.lines() {
                let line = line.trim();
                if line.starts_with("inet ") {
                    // Parse IPv4 address
                    if let Some(addr) = line.split_whitespace().nth(1) {
                        ipv4.push(addr.to_string());
                    }
                } else if line.starts_with("inet6 ") {
                    // Parse IPv6 address
                    if let Some(addr) = line.split_whitespace().nth(1) {
                        ipv6.push(addr.to_string());
                    }
                }
            }
        }

        (ipv4, ipv6)
    }

    fn get_wifi_info(&self, name: &str) -> Option<WifiInfo> {
        let mut info = WifiInfo {
            ssid: None,
            signal_dbm: None,
            quality_percent: None,
            frequency_mhz: None,
            mode: None,
            standards: Vec::new(),
        };

        // Try iw command for modern WiFi info
        if let Ok(output) = std::process::Command::new("iw")
            .args(["dev", name, "link"])
            .output()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);
            for line in stdout.lines() {
                let line = line.trim();
                if let Some(ssid) = line.strip_prefix("SSID:") {
                    info.ssid = Some(ssid.trim().to_string());
                } else if line.starts_with("signal:") {
                    if let Some(sig) = line.split_whitespace().nth(1) {
                        info.signal_dbm = sig.parse().ok();
                    }
                } else if line.starts_with("freq:") {
                    if let Some(freq) = line.split_whitespace().nth(1) {
                        info.frequency_mhz = freq.parse().ok();
                    }
                }
            }
        }

        // Get mode from iw
        if let Ok(output) = std::process::Command::new("iw")
            .args(["dev", name, "info"])
            .output()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);
            for line in stdout.lines() {
                let line = line.trim();
                if line.starts_with("type") {
                    info.mode = line.split_whitespace().nth(1).map(|s| s.to_string());
                }
            }
        }

        // Calculate quality from signal (rough approximation)
        if let Some(signal) = info.signal_dbm {
            // Typical range: -30 dBm (excellent) to -90 dBm (very weak)
            let quality = ((signal + 90) * 100 / 60).clamp(0, 100);
            info.quality_percent = Some(quality as u8);
        }

        Some(info)
    }

    fn read_sysfs_string(&self, path: &PathBuf) -> Option<String> {
        fs::read_to_string(path)
            .ok()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
    }

    fn read_sysfs_u32(&self, path: &PathBuf) -> Option<u32> {
        fs::read_to_string(path)
            .ok()
            .and_then(|s| s.trim().parse().ok())
    }

    fn read_sysfs_u64(&self, path: &PathBuf) -> Option<u64> {
        fs::read_to_string(path)
            .ok()
            .and_then(|s| s.trim().parse().ok())
    }
}

impl Default for NetworkDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

/// Format bytes for display
pub fn format_bytes(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;
    const TB: u64 = GB * 1024;

    if bytes >= TB {
        format!("{:.2} TB", bytes as f64 / TB as f64)
    } else if bytes >= GB {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.2} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.2} KB", bytes as f64 / KB as f64)
    } else {
        format!("{} B", bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_interfaces() {
        let mut discovery = NetworkDiscovery::new();
        let interfaces = discovery.enumerate_interfaces();

        println!("Found {} network interfaces:", interfaces.len());
        for iface in &interfaces {
            println!(
                "  {}: {:?} ({:?})",
                iface.name, iface.interface_type, iface.state
            );
            if let Some(ref mac) = iface.mac_address {
                println!("    MAC: {}", mac);
            }
            if !iface.ipv4_addresses.is_empty() {
                println!("    IPv4: {}", iface.ipv4_addresses.join(", "));
            }
            if let Some(speed) = iface.speed_string() {
                println!("    Speed: {}", speed);
            }
            if let Some(ref driver) = iface.driver {
                println!("    Driver: {}", driver);
            }
            if let Some(ref wifi) = iface.wifi_info {
                if let Some(ref ssid) = wifi.ssid {
                    println!("    SSID: {}", ssid);
                }
                if let Some(signal) = wifi.signal_dbm {
                    println!("    Signal: {} dBm", signal);
                }
            }
            println!(
                "    Traffic: TX {} / RX {}",
                format_bytes(iface.tx_bytes),
                format_bytes(iface.rx_bytes)
            );
        }
    }

    #[test]
    fn test_format_bytes() {
        assert_eq!(format_bytes(500), "500 B");
        assert_eq!(format_bytes(1024), "1.00 KB");
        assert_eq!(format_bytes(1_500_000), "1.43 MB");
        assert_eq!(format_bytes(2_500_000_000), "2.33 GB");
    }
}
