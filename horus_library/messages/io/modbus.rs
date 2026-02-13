//! Modbus communication message type

use horus_macros::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Modbus communication message
///
/// Standard industrial protocol message for communicating with
/// PLCs, sensors, and other Modbus-compatible devices.
#[derive(Debug, Clone, Serialize, Deserialize, LogSummary)]
pub struct ModbusMessage {
    /// Slave/unit address (1-255)
    pub unit_id: u8,
    /// Function code (1=read coils, 3=read holding registers, etc.)
    pub function_code: u8,
    /// Starting register/coil address
    pub start_address: u16,
    /// Number of registers/coils to read/write
    pub quantity: u16,
    /// Data payload (registers for function codes 3,4,6,16)
    #[serde(with = "serde_arrays")]
    pub data: [u16; 32],
    /// Data length (number of valid entries in data array)
    pub data_length: u8,
    /// Exception code if error occurred
    pub exception_code: u8,
    /// Transaction ID for matching requests/responses
    pub transaction_id: u16,
    /// Message direction (true = request, false = response)
    pub is_request: bool,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for ModbusMessage {
    fn default() -> Self {
        Self {
            unit_id: 1,
            function_code: 0,
            start_address: 0,
            quantity: 0,
            data: [0; 32],
            data_length: 0,
            exception_code: 0,
            transaction_id: 0,
            is_request: true,
            timestamp_ns: 0,
        }
    }
}

impl ModbusMessage {
    // Standard Modbus function codes
    pub const FUNC_READ_COILS: u8 = 1;
    pub const FUNC_READ_DISCRETE_INPUTS: u8 = 2;
    pub const FUNC_READ_HOLDING_REGISTERS: u8 = 3;
    pub const FUNC_READ_INPUT_REGISTERS: u8 = 4;
    pub const FUNC_WRITE_SINGLE_COIL: u8 = 5;
    pub const FUNC_WRITE_SINGLE_REGISTER: u8 = 6;
    pub const FUNC_WRITE_MULTIPLE_COILS: u8 = 15;
    pub const FUNC_WRITE_MULTIPLE_REGISTERS: u8 = 16;

    /// Create a read holding registers request
    pub fn read_holding_registers(unit_id: u8, start_addr: u16, count: u16) -> Self {
        Self {
            unit_id,
            function_code: Self::FUNC_READ_HOLDING_REGISTERS,
            start_address: start_addr,
            quantity: count,
            is_request: true,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Create a write single register request
    pub fn write_single_register(unit_id: u8, address: u16, value: u16) -> Self {
        let mut msg = Self {
            unit_id,
            function_code: Self::FUNC_WRITE_SINGLE_REGISTER,
            start_address: address,
            quantity: 1,
            data_length: 1,
            is_request: true,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        };
        msg.data[0] = value;
        msg
    }

    /// Create a write multiple registers request
    pub fn write_multiple_registers(unit_id: u8, start_addr: u16, values: &[u16]) -> Self {
        let mut msg = Self {
            unit_id,
            function_code: Self::FUNC_WRITE_MULTIPLE_REGISTERS,
            start_address: start_addr,
            quantity: values.len() as u16,
            data_length: values.len().min(32) as u8,
            is_request: true,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        };

        let copy_len = values.len().min(32);
        msg.data[..copy_len].copy_from_slice(&values[..copy_len]);
        msg
    }

    /// Create a response message
    pub fn create_response(&self, data: &[u16]) -> Self {
        let mut response = self.clone();
        response.is_request = false;
        response.data_length = data.len().min(32) as u8;
        response.data[..response.data_length as usize]
            .copy_from_slice(&data[..response.data_length as usize]);
        response.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        response
    }

    /// Create an exception response
    pub fn create_exception(&self, exception_code: u8) -> Self {
        Self {
            unit_id: self.unit_id,
            function_code: self.function_code | 0x80, // Set exception bit
            exception_code,
            transaction_id: self.transaction_id,
            is_request: false,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Check if this is an exception response
    pub fn is_exception(&self) -> bool {
        (self.function_code & 0x80) != 0
    }
}
