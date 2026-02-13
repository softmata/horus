//! Ethernet/IP communication message type

use horus_macros::LogSummary;
use serde::{Deserialize, Serialize};

/// Ethernet/IP communication message
///
/// Industrial Ethernet protocol commonly used with Allen-Bradley PLCs.
#[derive(Debug, Clone, Serialize, Deserialize, LogSummary)]
pub struct EtherNetIPMessage {
    /// Service code
    pub service: u8,
    /// Class ID
    pub class_id: u16,
    /// Instance ID
    pub instance_id: u16,
    /// Attribute ID
    pub attribute_id: u16,
    /// Data payload
    pub data: Vec<u8>,
    /// Session handle
    pub session_handle: u32,
    /// Context data
    pub context: [u8; 8],
    /// Status code
    pub status: u16,
    /// Message direction (true = request, false = response)
    pub is_request: bool,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for EtherNetIPMessage {
    fn default() -> Self {
        Self {
            service: 0,
            class_id: 0,
            instance_id: 0,
            attribute_id: 0,
            data: Vec::new(),
            session_handle: 0,
            context: [0; 8],
            status: 0,
            is_request: true,
            timestamp_ns: 0,
        }
    }
}
