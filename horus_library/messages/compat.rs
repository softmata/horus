//! Conversions between softmata-core canonical types and HORUS message types.
//!
//! HORUS messages have extra fields (covariances, frame IDs, etc.) that the
//! canonical softmata-core types don't include. These conversions map between
//! them, filling defaults for missing fields.

use softmata_core::messages as sc;

use super::cmd_vel::CmdVel;
use super::diagnostics::EmergencyStop as HorusEmergencyStop;
use super::sensor::{Imu, Odometry as HorusOdometry};

// IMU
impl From<&sc::ImuReading> for Imu {
    fn from(src: &sc::ImuReading) -> Self {
        Self {
            orientation: src.orientation,
            orientation_covariance: [0.0; 9],
            angular_velocity: src.angular_velocity,
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: src.linear_acceleration,
            linear_acceleration_covariance: [0.0; 9],
            timestamp_ns: src.timestamp_ns,
        }
    }
}

impl From<&Imu> for sc::ImuReading {
    fn from(src: &Imu) -> Self {
        Self {
            linear_acceleration: src.linear_acceleration,
            angular_velocity: src.angular_velocity,
            orientation: src.orientation,
            timestamp_ns: src.timestamp_ns,
        }
    }
}

// CmdVel
impl From<&sc::CmdVel> for CmdVel {
    fn from(src: &sc::CmdVel) -> Self {
        Self {
            timestamp_ns: src.timestamp_ns,
            linear: src.linear as f32,
            angular: src.angular as f32,
        }
    }
}

impl From<&CmdVel> for sc::CmdVel {
    fn from(src: &CmdVel) -> Self {
        Self {
            linear: src.linear as f64,
            angular: src.angular as f64,
            timestamp_ns: src.timestamp_ns,
        }
    }
}

// Odometry
impl From<&sc::Odometry> for HorusOdometry {
    fn from(src: &sc::Odometry) -> Self {
        let mut odom = Self::default();
        odom.pose.x = src.x;
        odom.pose.y = src.y;
        odom.pose.theta = src.theta;
        odom.twist.linear[0] = src.linear_velocity;
        odom.twist.angular[2] = src.angular_velocity;
        odom.timestamp_ns = src.timestamp_ns;
        odom
    }
}

impl From<&HorusOdometry> for sc::Odometry {
    fn from(src: &HorusOdometry) -> Self {
        Self {
            x: src.pose.x,
            y: src.pose.y,
            theta: src.pose.theta,
            linear_velocity: src.twist.linear[0],
            angular_velocity: src.twist.angular[2],
            timestamp_ns: src.timestamp_ns,
        }
    }
}

// EmergencyStop
impl From<&sc::EmergencyStop> for HorusEmergencyStop {
    fn from(src: &sc::EmergencyStop) -> Self {
        let mut msg = Self::default();
        msg.engaged = src.engaged;
        msg.auto_reset = src.auto_reset;
        msg.reason = src.reason;
        msg.source = src.source;
        msg.timestamp_ns = src.timestamp_ns;
        msg
    }
}

impl From<&HorusEmergencyStop> for sc::EmergencyStop {
    fn from(src: &HorusEmergencyStop) -> Self {
        Self {
            engaged: src.engaged,
            auto_reset: src.auto_reset,
            _pad: [0; 6],
            reason: src.reason,
            source: src.source,
            timestamp_ns: src.timestamp_ns,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_roundtrip() {
        let sc_imu = sc::ImuReading::new([1.0, 2.0, 9.81], [0.0, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0], 42);
        let horus_imu: Imu = (&sc_imu).into();
        let back: sc::ImuReading = (&horus_imu).into();
        assert_eq!(back.linear_acceleration, sc_imu.linear_acceleration);
        assert_eq!(back.timestamp_ns, sc_imu.timestamp_ns);
    }

    #[test]
    fn cmd_vel_roundtrip() {
        let sc_cmd = sc::CmdVel::new(1.5, -0.3, 99);
        let horus_cmd: CmdVel = (&sc_cmd).into();
        let back: sc::CmdVel = (&horus_cmd).into();
        assert!((back.linear - 1.5).abs() < 0.001);
    }

    #[test]
    fn estop_roundtrip() {
        let sc_stop = sc::EmergencyStop::engage("collision", "safety", 42);
        let horus_stop: HorusEmergencyStop = (&sc_stop).into();
        let back: sc::EmergencyStop = (&horus_stop).into();
        assert_eq!(back.engaged, sc_stop.engaged);
        assert_eq!(back.reason_str(), "collision");
    }
}
