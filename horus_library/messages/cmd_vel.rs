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

/// Convert from 3D 6-DOF Twist to 2D CmdVel.
///
/// Maps `linear[0]` (forward) to `linear` and `angular[2]` (yaw) to `angular`.
/// Precision is reduced from f64 to f32.
impl From<crate::messages::geometry::Twist> for CmdVel {
    fn from(twist: crate::messages::geometry::Twist) -> Self {
        Self {
            stamp_nanos: twist.timestamp_ns,
            linear: twist.linear[0] as f32,
            angular: twist.angular[2] as f32,
        }
    }
}

/// Convert from 2D CmdVel to 3D 6-DOF Twist.
///
/// Sets `linear[0]` from `linear` and `angular[2]` from `angular`;
/// all other components are zero.
impl From<CmdVel> for crate::messages::geometry::Twist {
    fn from(cmd: CmdVel) -> Self {
        Self {
            linear: [cmd.linear as f64, 0.0, 0.0],
            angular: [0.0, 0.0, cmd.angular as f64],
            timestamp_ns: cmd.stamp_nanos,
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::geometry::Twist;

    #[test]
    fn test_twist_to_cmdvel() {
        let twist = Twist {
            linear: [1.5, 0.2, 0.0],
            angular: [0.0, 0.0, 0.8],
            timestamp_ns: 12345,
        };
        let cmd: CmdVel = twist.into();
        assert!((cmd.linear - 1.5).abs() < 1e-6);
        assert!((cmd.angular - 0.8).abs() < 1e-6);
        assert_eq!(cmd.stamp_nanos, 12345);
    }

    #[test]
    fn test_cmdvel_to_twist() {
        let cmd = CmdVel::with_timestamp(2.0, -0.5, 67890);
        let twist: Twist = cmd.into();
        assert!((twist.linear[0] - 2.0).abs() < 1e-6);
        assert_eq!(twist.linear[1], 0.0);
        assert_eq!(twist.linear[2], 0.0);
        assert_eq!(twist.angular[0], 0.0);
        assert_eq!(twist.angular[1], 0.0);
        assert!((twist.angular[2] - (-0.5)).abs() < 1e-6);
        assert_eq!(twist.timestamp_ns, 67890);
    }

    #[test]
    fn test_twist_cmdvel_roundtrip() {
        let original = CmdVel::with_timestamp(1.0, 0.5, 11111);
        let twist: Twist = original.into();
        let roundtripped: CmdVel = twist.into();
        assert!((roundtripped.linear - original.linear).abs() < 1e-6);
        assert!((roundtripped.angular - original.angular).abs() < 1e-6);
        assert_eq!(roundtripped.stamp_nanos, original.stamp_nanos);
    }
}
