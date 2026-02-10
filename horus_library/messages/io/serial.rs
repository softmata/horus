//! Serial/UART communication message type

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Serial/UART Data Message
///
/// Raw byte data for serial/UART communication with devices like
/// GPS modules, sensors, Arduino boards, and other serial peripherals.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerialData {
    /// Serial port identifier (e.g., "/dev/ttyUSB0", "COM3")
    #[serde(with = "serde_arrays")]
    pub port_id: [u8; 64],
    /// Raw data bytes (maximum 1024 bytes per message)
    #[serde(with = "serde_arrays")]
    pub data: [u8; 1024],
    /// Number of valid bytes in data array
    pub data_length: u16,
    /// Baud rate
    pub baud_rate: u32,
    /// Data bits (5, 6, 7, or 8)
    pub data_bits: u8,
    /// Stop bits (1 or 2)
    pub stop_bits: u8,
    /// Parity (0=none, 1=odd, 2=even)
    pub parity: u8,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for SerialData {
    fn default() -> Self {
        Self {
            port_id: [0; 64],
            data: [0; 1024],
            data_length: 0,
            baud_rate: 9600,
            data_bits: 8,
            stop_bits: 1,
            parity: 0,
            timestamp_ns: 0,
        }
    }
}

impl SerialData {
    pub const PARITY_NONE: u8 = 0;
    pub const PARITY_ODD: u8 = 1;
    pub const PARITY_EVEN: u8 = 2;

    /// Create a new serial data message
    pub fn new(port: &str) -> Self {
        let mut msg = Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        };
        msg.set_port(port);
        msg
    }

    /// Set port ID from string
    pub fn set_port(&mut self, port: &str) {
        let port_bytes = port.as_bytes();
        let len = port_bytes.len().min(63);
        self.port_id[..len].copy_from_slice(&port_bytes[..len]);
        self.port_id[len] = 0;
    }

    /// Get port ID as string
    pub fn get_port(&self) -> String {
        let end = self.port_id.iter().position(|&b| b == 0).unwrap_or(64);
        String::from_utf8_lossy(&self.port_id[..end]).into_owned()
    }

    /// Set data from byte slice
    pub fn set_data(&mut self, data: &[u8]) -> bool {
        if data.len() > 1024 {
            return false;
        }
        self.data[..data.len()].copy_from_slice(data);
        self.data_length = data.len() as u16;
        true
    }

    /// Get data as byte slice
    pub fn get_data(&self) -> &[u8] {
        &self.data[..self.data_length as usize]
    }

    /// Set data from string (UTF-8)
    pub fn set_string(&mut self, text: &str) -> bool {
        self.set_data(text.as_bytes())
    }

    /// Get data as string (if valid UTF-8)
    pub fn get_string(&self) -> Option<String> {
        std::str::from_utf8(self.get_data())
            .ok()
            .map(|s| s.to_string())
    }
}

impl LogSummary for SerialData {
    fn log_summary(&self) -> String {
        format!(
            "Serial[{}]: {} bytes @ {}bps",
            self.get_port(),
            self.data_length,
            self.baud_rate
        )
    }
}
