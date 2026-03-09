use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};

/// 2D velocity command for differential-drive / unicycle robots.
///
/// Use `CmdVel` for **2D mobile robots** (ground robots, AMRs) where you only
/// need forward velocity and yaw rate. For **3D robots** (drones, manipulators),
/// use [`Twist`](crate::messages::geometry::Twist) instead.
///
/// `CmdVel` is a POD type (12 bytes + timestamp) — zero-copy transfer at ~50ns.
///
/// # CmdVel vs Twist
///
/// | | `CmdVel` | `Twist` |
/// |---|---|---|
/// | DOF | 2 (linear x, angular z) | 6 (full linear + angular) |
/// | Precision | f32 | f64 |
/// | Use case | Ground robots | Drones, arms, general 3D |
/// | Size | 16 bytes | 56 bytes |
///
/// Conversions are provided in both directions via `From`:
/// ```rust
/// # use horus_library::messages::{CmdVel, geometry::Twist};
/// let cmd = CmdVel::new(1.0, 0.5);
/// let twist: Twist = cmd.into();  // linear[0]=1.0, angular[2]=0.5
/// let back: CmdVel = twist.into();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct CmdVel {
    pub timestamp_ns: u64,
    pub linear: f32,  // m/s forward velocity
    pub angular: f32, // rad/s turning velocity
}

impl CmdVel {
    /// Create a new CmdVel message with current timestamp
    pub fn new(linear: f32, angular: f32) -> Self {
        Self {
            timestamp_ns: crate::hframe::timestamp_now(),
            linear,
            angular,
        }
    }

    /// Create a zero velocity command (stop)
    pub fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    /// Create a CmdVel with explicit timestamp
    pub fn with_timestamp(linear: f32, angular: f32, timestamp_ns: u64) -> Self {
        Self {
            timestamp_ns,
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
            timestamp_ns: twist.timestamp_ns,
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
            timestamp_ns: cmd.timestamp_ns,
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
        assert_eq!(cmd.timestamp_ns, 12345);
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
        assert_eq!(roundtripped.timestamp_ns, original.timestamp_ns);
    }
}
