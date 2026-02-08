//! Industrial network status message type

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};

/// Industrial network status
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct NetworkStatus {
    /// Network interface name
    pub interface_name: [u8; 16],
    /// IP address (IPv4 as u32)
    pub ip_address: u32,
    /// Subnet mask
    pub subnet_mask: u32,
    /// Gateway address
    pub gateway: u32,
    /// Link status (true = up, false = down)
    pub link_up: bool,
    /// Link speed in Mbps
    pub link_speed: u16,
    /// Duplex mode (true = full, false = half)
    pub full_duplex: bool,
    /// Packets transmitted
    pub tx_packets: u64,
    /// Packets received
    pub rx_packets: u64,
    /// Transmission errors
    pub tx_errors: u32,
    /// Reception errors
    pub rx_errors: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl NetworkStatus {
    /// Create new network status
    pub fn new(interface: &str) -> Self {
        let mut status = Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        };

        let interface_bytes = interface.as_bytes();
        let len = interface_bytes.len().min(15);
        status.interface_name[..len].copy_from_slice(&interface_bytes[..len]);
        status.interface_name[len] = 0;

        status
    }

    /// Convert IP address to string
    pub fn ip_to_string(&self) -> String {
        format!(
            "{}.{}.{}.{}",
            (self.ip_address >> 24) & 0xFF,
            (self.ip_address >> 16) & 0xFF,
            (self.ip_address >> 8) & 0xFF,
            self.ip_address & 0xFF
        )
    }

    /// Set IP address from string
    pub fn set_ip_from_string(&mut self, ip_str: &str) -> Result<(), &'static str> {
        let parts: Vec<&str> = ip_str.split('.').collect();
        if parts.len() != 4 {
            return Err("Invalid IP address format");
        }

        let mut ip = 0u32;
        for (i, part) in parts.iter().enumerate() {
            let octet: u8 = part.parse().map_err(|_| "Invalid IP octet")?;
            ip |= (octet as u32) << (24 - i * 8);
        }

        self.ip_address = ip;
        Ok(())
    }

    /// Calculate packet loss percentage
    pub fn packet_loss_percent(&self) -> f32 {
        if self.tx_packets == 0 {
            return 0.0;
        }
        (self.tx_errors as f32 / self.tx_packets as f32) * 100.0
    }
}

impl LogSummary for NetworkStatus {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}
