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
            stamp_nanos: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
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

// Enable zero-copy serialization with bytemuck
unsafe impl horus_core::bytemuck::Pod for CmdVel {}
unsafe impl horus_core::bytemuck::Zeroable for CmdVel {}

// Enable ultra-fast POD messaging (~50ns vs ~250ns with bincode)
unsafe impl horus_core::communication::PodMessage for CmdVel {}

// LogSummary implementation for zero-copy logging
impl LogSummary for CmdVel {
    fn log_summary(&self) -> String {
        format!("CmdVel(lin={:.2}, ang={:.2})", self.linear, self.angular)
    }
}
