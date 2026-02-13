//! Safety relay status message type

use horus_macros::LogSummary;
use serde::{Deserialize, Serialize};

/// Safety relay status
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct SafetyRelayStatus {
    /// Relay identifier
    pub relay_id: [u8; 16],
    /// Safety output states (0 = off, 1 = on)
    pub safety_outputs: [u8; 8],
    /// Input channel states (0 = off, 1 = on)
    pub input_channels: [u8; 16],
    /// Diagnostic information
    pub diagnostics: u16,
    /// Safety function active (0 = inactive, 1 = active)
    pub safety_active: u8,
    /// Reset required (0 = no, 1 = yes)
    pub reset_required: u8,
    /// Fault present (0 = no, 1 = yes)
    pub fault_present: u8,
    /// Test mode active (0 = inactive, 1 = active)
    pub test_mode: u8,
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
            safety_active: 1, // Default to active/safe state
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
        self.fault_present == 0 && self.safety_active != 0 && self.reset_required == 0
    }

    /// Get active safety output count
    pub fn active_output_count(&self) -> u8 {
        self.safety_outputs.iter().filter(|&&state| state != 0).count() as u8
    }

    /// Get active input count
    pub fn active_input_count(&self) -> u8 {
        self.input_channels.iter().filter(|&&state| state != 0).count() as u8
    }
}

// =============================================================================
// POD (Plain Old Data) Message Support
// =============================================================================

unsafe impl horus_core::bytemuck::Pod for SafetyRelayStatus {}
unsafe impl horus_core::bytemuck::Zeroable for SafetyRelayStatus {}
unsafe impl horus_core::communication::PodMessage for SafetyRelayStatus {}
