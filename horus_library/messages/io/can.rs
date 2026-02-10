//! CAN (Controller Area Network) message type

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// CAN (Controller Area Network) Frame Message
///
/// Standard CAN 2.0A/B frame format for automotive and industrial communication.
/// Supports both standard (11-bit) and extended (29-bit) identifiers.
///
/// # Frame Types
/// - Data Frame: Carries data payload (0-8 bytes)
/// - Remote Frame: Requests data from another node
/// - Error Frame: Signals transmission errors
/// - Overload Frame: Indicates receiver overload
///
/// # Common Applications
/// - Automotive ECU communication (OBD-II, J1939, CANopen)
/// - Industrial automation (DeviceNet, CANopen)
/// - Mobile robotics (motor controllers, sensors)
/// - Aerospace systems
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CanFrame {
    /// CAN identifier (11-bit standard or 29-bit extended)
    pub id: u32,
    /// Extended frame format (29-bit ID)
    pub is_extended: bool,
    /// Remote transmission request
    pub is_rtr: bool,
    /// Error frame
    pub is_error: bool,
    /// Data payload (0-8 bytes for CAN 2.0, 0-64 for CAN-FD)
    #[serde(with = "serde_arrays")]
    pub data: [u8; 64],
    /// Data length (0-8 for CAN 2.0, up to 64 for CAN-FD)
    pub dlc: u8,
    /// CAN-FD frame format
    pub is_fd: bool,
    /// Bit rate switch (CAN-FD only)
    pub is_brs: bool,
    /// Error state indicator (CAN-FD only)
    pub is_esi: bool,
    /// CAN interface name (e.g., "can0", "vcan0")
    pub interface: [u8; 16],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for CanFrame {
    fn default() -> Self {
        Self {
            id: 0,
            is_extended: false,
            is_rtr: false,
            is_error: false,
            data: [0; 64],
            dlc: 0,
            is_fd: false,
            is_brs: false,
            is_esi: false,
            interface: [0; 16],
            timestamp_ns: 0,
        }
    }
}

impl CanFrame {
    /// CAN 2.0 maximum data length
    pub const MAX_DLC: u8 = 8;
    /// CAN-FD maximum data length
    pub const MAX_FD_DLC: u8 = 64;
    /// Standard ID maximum value (11-bit)
    pub const MAX_STANDARD_ID: u32 = 0x7FF;
    /// Extended ID maximum value (29-bit)
    pub const MAX_EXTENDED_ID: u32 = 0x1FFFFFFF;

    /// Create a new standard CAN frame
    pub fn new(id: u32, data: &[u8]) -> Self {
        let mut frame = Self {
            id: id & Self::MAX_STANDARD_ID,
            is_extended: false,
            is_rtr: false,
            is_error: false,
            data: [0; 64],
            dlc: data.len().min(8) as u8,
            is_fd: false,
            is_brs: false,
            is_esi: false,
            interface: [0; 16],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };
        frame.data[..data.len().min(8)].copy_from_slice(&data[..data.len().min(8)]);
        frame
    }

    /// Create a new extended CAN frame
    pub fn new_extended(id: u32, data: &[u8]) -> Self {
        let mut frame = Self::new(id & Self::MAX_EXTENDED_ID, data);
        frame.id = id & Self::MAX_EXTENDED_ID;
        frame.is_extended = true;
        frame
    }

    /// Create a remote transmission request frame
    pub fn new_rtr(id: u32, dlc: u8) -> Self {
        Self {
            id: id & Self::MAX_STANDARD_ID,
            is_extended: false,
            is_rtr: true,
            is_error: false,
            data: [0; 64],
            dlc: dlc.min(8),
            is_fd: false,
            is_brs: false,
            is_esi: false,
            interface: [0; 16],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Create a CAN-FD frame
    pub fn new_fd(id: u32, data: &[u8], brs: bool) -> Self {
        let mut frame = Self {
            id: id & Self::MAX_STANDARD_ID,
            is_extended: false,
            is_rtr: false,
            is_error: false,
            data: [0; 64],
            dlc: data.len().min(64) as u8,
            is_fd: true,
            is_brs: brs,
            is_esi: false,
            interface: [0; 16],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };
        frame.data[..data.len().min(64)].copy_from_slice(&data[..data.len().min(64)]);
        frame
    }

    /// Set CAN interface name
    pub fn set_interface(&mut self, interface: &str) {
        let bytes = interface.as_bytes();
        let len = bytes.len().min(15);
        self.interface[..len].copy_from_slice(&bytes[..len]);
        self.interface[len] = 0;
    }

    /// Get CAN interface name as string
    pub fn get_interface(&self) -> String {
        let end = self.interface.iter().position(|&b| b == 0).unwrap_or(16);
        String::from_utf8_lossy(&self.interface[..end]).into_owned()
    }

    /// Get data as slice
    pub fn data_slice(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }

    /// Check if frame is valid
    pub fn is_valid(&self) -> bool {
        let max_id = if self.is_extended {
            Self::MAX_EXTENDED_ID
        } else {
            Self::MAX_STANDARD_ID
        };

        let max_dlc = if self.is_fd {
            Self::MAX_FD_DLC
        } else {
            Self::MAX_DLC
        };

        self.id <= max_id && self.dlc <= max_dlc && (!self.is_rtr || self.dlc == 0)
    }

    /// Pack 8-bit values into CAN data (little-endian)
    pub fn pack_u8(&mut self, offset: usize, value: u8) -> bool {
        if offset < self.dlc as usize {
            self.data[offset] = value;
            true
        } else {
            false
        }
    }

    /// Pack 16-bit values into CAN data (little-endian)
    pub fn pack_u16(&mut self, offset: usize, value: u16) -> bool {
        if offset + 1 < self.dlc as usize {
            self.data[offset] = (value & 0xFF) as u8;
            self.data[offset + 1] = ((value >> 8) & 0xFF) as u8;
            true
        } else {
            false
        }
    }

    /// Pack 32-bit values into CAN data (little-endian)
    pub fn pack_u32(&mut self, offset: usize, value: u32) -> bool {
        if offset + 3 < self.dlc as usize {
            self.data[offset] = (value & 0xFF) as u8;
            self.data[offset + 1] = ((value >> 8) & 0xFF) as u8;
            self.data[offset + 2] = ((value >> 16) & 0xFF) as u8;
            self.data[offset + 3] = ((value >> 24) & 0xFF) as u8;
            true
        } else {
            false
        }
    }

    /// Unpack 8-bit value from CAN data
    pub fn unpack_u8(&self, offset: usize) -> Option<u8> {
        if offset < self.dlc as usize {
            Some(self.data[offset])
        } else {
            None
        }
    }

    /// Unpack 16-bit value from CAN data (little-endian)
    pub fn unpack_u16(&self, offset: usize) -> Option<u16> {
        if offset + 1 < self.dlc as usize {
            Some((self.data[offset] as u16) | ((self.data[offset + 1] as u16) << 8))
        } else {
            None
        }
    }

    /// Unpack 32-bit value from CAN data (little-endian)
    pub fn unpack_u32(&self, offset: usize) -> Option<u32> {
        if offset + 3 < self.dlc as usize {
            Some(
                (self.data[offset] as u32)
                    | ((self.data[offset + 1] as u32) << 8)
                    | ((self.data[offset + 2] as u32) << 16)
                    | ((self.data[offset + 3] as u32) << 24),
            )
        } else {
            None
        }
    }
}

impl LogSummary for CanFrame {
    fn log_summary(&self) -> String {
        format!(
            "CAN[0x{:03X}]: {} bytes{}{}",
            self.id,
            self.dlc,
            if self.is_extended { " EXT" } else { "" },
            if self.is_fd { " FD" } else { "" }
        )
    }
}
