//! I2C bus communication message type

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// I2C Bus Transaction Message
///
/// I2C read/write transaction for communicating with I2C devices like
/// sensors, displays, EEPROMs, and other peripherals.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct I2cMessage {
    /// I2C device address (7-bit or 10-bit)
    pub device_address: u16,
    /// Register address to read/write
    pub register_address: u8,
    /// Transaction type (0=read, 1=write, 2=read_register, 3=write_register)
    pub transaction_type: u8,
    /// Data bytes (maximum 256 bytes)
    #[serde(with = "serde_arrays")]
    pub data: [u8; 256],
    /// Number of bytes to read/write
    pub data_length: u8,
    /// I2C bus number (0, 1, 2, etc.)
    pub bus_number: u8,
    /// Clock speed in Hz (typically 100kHz or 400kHz)
    pub clock_speed: u32,
    /// Transaction successful
    pub success: bool,
    /// Error code (0=no error)
    pub error_code: u8,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for I2cMessage {
    fn default() -> Self {
        Self {
            device_address: 0,
            register_address: 0,
            transaction_type: Self::TYPE_READ,
            data: [0; 256],
            data_length: 0,
            bus_number: 1,
            clock_speed: 100000, // 100kHz default
            success: false,
            error_code: 0,
            timestamp_ns: 0,
        }
    }
}

impl I2cMessage {
    // Transaction types
    pub const TYPE_READ: u8 = 0;
    pub const TYPE_WRITE: u8 = 1;
    pub const TYPE_READ_REGISTER: u8 = 2;
    pub const TYPE_WRITE_REGISTER: u8 = 3;

    // Common clock speeds
    pub const SPEED_STANDARD: u32 = 100000; // 100 kHz
    pub const SPEED_FAST: u32 = 400000; // 400 kHz
    pub const SPEED_FAST_PLUS: u32 = 1000000; // 1 MHz

    /// Create a new I2C read message
    pub fn read(bus: u8, address: u16, length: u8) -> Self {
        Self {
            bus_number: bus,
            device_address: address,
            transaction_type: Self::TYPE_READ,
            data_length: length,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Create a new I2C write message
    pub fn write(bus: u8, address: u16, data: &[u8]) -> Self {
        let mut msg = Self {
            bus_number: bus,
            device_address: address,
            transaction_type: Self::TYPE_WRITE,
            data_length: data.len().min(256) as u8,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        };
        let len = data.len().min(256);
        msg.data[..len].copy_from_slice(&data[..len]);
        msg
    }

    /// Create a register read message
    pub fn read_register(bus: u8, address: u16, register: u8, length: u8) -> Self {
        Self {
            bus_number: bus,
            device_address: address,
            register_address: register,
            transaction_type: Self::TYPE_READ_REGISTER,
            data_length: length,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Create a register write message
    pub fn write_register(bus: u8, address: u16, register: u8, data: &[u8]) -> Self {
        let mut msg = Self {
            bus_number: bus,
            device_address: address,
            register_address: register,
            transaction_type: Self::TYPE_WRITE_REGISTER,
            data_length: data.len().min(256) as u8,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        };
        let len = data.len().min(256);
        msg.data[..len].copy_from_slice(&data[..len]);
        msg
    }

    /// Get data as slice
    pub fn get_data(&self) -> &[u8] {
        &self.data[..self.data_length as usize]
    }

    /// Set data from slice
    pub fn set_data(&mut self, data: &[u8]) -> bool {
        if data.len() > 256 {
            return false;
        }
        self.data[..data.len()].copy_from_slice(data);
        self.data_length = data.len() as u8;
        true
    }
}

impl LogSummary for I2cMessage {
    fn log_summary(&self) -> String {
        format!(
            "I2C[0x{:02X}]: {} {} bytes",
            self.device_address,
            match self.transaction_type {
                Self::TYPE_READ => "read",
                Self::TYPE_WRITE => "write",
                Self::TYPE_READ_REGISTER => "read_reg",
                Self::TYPE_WRITE_REGISTER => "write_reg",
                _ => "unknown",
            },
            self.data_length
        )
    }
}
