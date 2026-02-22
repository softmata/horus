use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};

/// Command velocity message for robot control
///
/// Standard message type used across the HORUS ecosystem for controlling
/// robot movement. Contains linear and angular velocity commands.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct CmdVel {
    pub stamp_nanos: u64,
    pub linear: f32,  // m/s forward velocity
    pub angular: f32, // rad/s turning velocity
}

impl CmdVel {
    /// Create a new CmdVel message with current timestamp
    pub fn new(linear: f32, angular: f32) -> Self {
        Self {
            stamp_nanos: crate::hframe::timestamp_now(),
            linear,
            angular,
        }
    }

    /// Create a zero velocity command (stop)
    pub fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    /// Create a CmdVel with explicit timestamp
    pub fn with_timestamp(linear: f32, angular: f32, stamp_nanos: u64) -> Self {
        Self {
            stamp_nanos,
            linear,
            angular,
        }
    }
}

impl Default for CmdVel {
    fn default() -> Self {
        Self::zero()
    }
}

crate::messages::impl_pod_message!(CmdVel);

// Note: POD types are now auto-detected via needs_drop, no registration needed

// LogSummary implementation for zero-copy logging
impl LogSummary for CmdVel {
    fn log_summary(&self) -> String {
        format!("CmdVel(lin={:.2}, ang={:.2})", self.linear, self.angular)
    }
}
