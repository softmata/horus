//! Safety relay status message type

use horus_macros::LogSummary;
use serde::{Deserialize, Serialize};

/// Safety relay status
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct SafetyRelayStatus {
    /// Relay identifier
    pub relay_id: [u8; 16],
    /// Safety output states
    pub safety_outputs: [bool; 8],
    /// Input channel states
    pub input_channels: [bool; 16],
    /// Diagnostic information
    pub diagnostics: u16,
    /// Safety function active
    pub safety_active: bool,
    /// Reset required
    pub reset_required: bool,
    /// Fault present
    pub fault_present: bool,
    /// Test mode active
    pub test_mode: bool,
    /// Operating hours
    pub operating_hours: u32,
    /// Switch cycles count
    pub switch_cycles: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl SafetyRelayStatus {
    /// Create new safety relay status
    pub fn new(relay_id: &str) -> Self {
        let mut status = Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            safety_active: true, // Default to active/safe state
            ..Default::default()
        };

        let id_bytes = relay_id.as_bytes();
        let len = id_bytes.len().min(15);
        status.relay_id[..len].copy_from_slice(&id_bytes[..len]);
        status.relay_id[len] = 0;

        status
    }

    /// Check if system is in safe state
    pub fn is_safe_state(&self) -> bool {
        !self.fault_present && self.safety_active && !self.reset_required
    }

    /// Get active safety output count
    pub fn active_output_count(&self) -> u8 {
        self.safety_outputs.iter().filter(|&&state| state).count() as u8
    }

    /// Get active input count
    pub fn active_input_count(&self) -> u8 {
        self.input_channels.iter().filter(|&&state| state).count() as u8
    }
}
