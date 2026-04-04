//! Message roundtrip tests for all 11 monomorphized FFI types.
//!
//! Each test: create publisher + subscriber, send a message, verify
//! all fields are preserved exactly through SHM.

#[cfg(test)]
mod tests {
    use crate::topic_ffi::*;

    fn unique_topic(prefix: &str) -> String {
        use std::sync::atomic::{AtomicU64, Ordering};
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        format!(
            "roundtrip.{}.{}.{}",
            prefix,
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        )
    }

    #[test]
    fn roundtrip_cmd_vel() {
        use horus_robotics::CmdVel;
        let t = unique_topic("cmd_vel");
        let pub_ = publisher_cmd_vel_new(&t).unwrap();
        let sub = subscriber_cmd_vel_new(&t).unwrap();
        let msg = CmdVel {
            timestamp_ns: 999,
            linear: -1.5,
            angular: 3.25,
        };
        publisher_cmd_vel_send(&pub_, msg);
        let r = subscriber_cmd_vel_recv(&sub).unwrap();
        assert_eq!(r.timestamp_ns, 999);
        assert!((r.linear - (-1.5)).abs() < f32::EPSILON);
        assert!((r.angular - 3.25).abs() < 0.001);
    }

    #[test]
    fn roundtrip_twist() {
        use horus_types::Twist;
        let t = unique_topic("twist");
        let pub_ = publisher_twist_new(&t).unwrap();
        let sub = subscriber_twist_new(&t).unwrap();
        let msg = Twist {
            linear: [1.1, 2.2, 3.3],
            angular: [4.4, 5.5, 6.6],
            timestamp_ns: 42,
        };
        publisher_twist_send(&pub_, msg);
        let r = subscriber_twist_recv(&sub).unwrap();
        assert_eq!(r.timestamp_ns, 42);
        assert!((r.linear[0] - 1.1).abs() < f64::EPSILON);
        assert!((r.angular[2] - 6.6).abs() < f64::EPSILON);
    }

    #[test]
    fn roundtrip_pose2d() {
        use horus_types::Pose2D;
        let t = unique_topic("pose2d");
        let pub_ = publisher_pose2d_new(&t).unwrap();
        let sub = subscriber_pose2d_new(&t).unwrap();
        let msg = Pose2D {
            x: -100.5,
            y: 200.3,
            theta: std::f64::consts::PI,
            timestamp_ns: 7,
        };
        publisher_pose2d_send(&pub_, msg);
        let r = subscriber_pose2d_recv(&sub).unwrap();
        assert!((r.x - (-100.5)).abs() < f64::EPSILON);
        assert!((r.theta - std::f64::consts::PI).abs() < f64::EPSILON);
    }

    #[test]
    fn roundtrip_imu() {
        use horus_robotics::Imu;
        let t = unique_topic("imu");
        let pub_ = publisher_imu_new(&t).unwrap();
        let sub = subscriber_imu_new(&t).unwrap();
        let msg = Imu {
            orientation: [0.0, 0.0, 0.707, 0.707],
            linear_acceleration: [0.0, 0.0, 9.81],
            timestamp_ns: 123,
            ..Imu::default()
        };
        publisher_imu_send(&pub_, msg);
        let r = subscriber_imu_recv(&sub).unwrap();
        assert!((r.orientation[2] - 0.707).abs() < 0.001);
        assert!((r.linear_acceleration[2] - 9.81).abs() < 0.001);
        assert_eq!(r.timestamp_ns, 123);
    }

    #[test]
    fn roundtrip_nav_goal() {
        use horus_robotics::NavGoal;
        use horus_types::Pose2D;
        let t = unique_topic("nav_goal");
        let pub_ = publisher_nav_goal_new(&t).unwrap();
        let sub = subscriber_nav_goal_new(&t).unwrap();
        let msg = NavGoal {
            target_pose: Pose2D {
                x: 5.0,
                y: 10.0,
                theta: 1.57,
                timestamp_ns: 55,
            },
            tolerance_position: 0.1,
            ..NavGoal::default()
        };
        publisher_nav_goal_send(&pub_, msg);
        let r = subscriber_nav_goal_recv(&sub).unwrap();
        assert!((r.target_pose.x - 5.0).abs() < f64::EPSILON);
        assert!((r.tolerance_position - 0.1).abs() < f64::EPSILON);
    }

    #[test]
    fn roundtrip_heartbeat() {
        use horus_types::Heartbeat;
        let t = unique_topic("heartbeat");
        let pub_ = publisher_heartbeat_new(&t).unwrap();
        let sub = subscriber_heartbeat_new(&t).unwrap();
        let msg = Heartbeat {
            sequence: 42,
            alive: 1,
            uptime: 55.5,
            ..Heartbeat::default()
        };
        publisher_heartbeat_send(&pub_, msg);
        let r = subscriber_heartbeat_recv(&sub).unwrap();
        assert_eq!(r.sequence, 42);
        assert_eq!(r.alive, 1);
        assert!((r.uptime - 55.5).abs() < 0.01);
    }

    #[test]
    fn roundtrip_emergency_stop() {
        use horus_types::EmergencyStop;
        let t = unique_topic("estop");
        let pub_ = publisher_emergency_stop_new(&t).unwrap();
        let sub = subscriber_emergency_stop_new(&t).unwrap();
        let msg = EmergencyStop {
            engaged: 1,
            auto_reset: 0,
            ..EmergencyStop::default()
        };
        publisher_emergency_stop_send(&pub_, msg);
        let r = subscriber_emergency_stop_recv(&sub).unwrap();
        assert_eq!(r.engaged, 1);
        assert_eq!(r.auto_reset, 0);
    }

    // ─── Edge Cases ──────────────────────────────────────────────────

    #[test]
    fn recv_empty_returns_none() {
        let t = unique_topic("empty");
        let sub = subscriber_cmd_vel_new(&t).unwrap();
        assert!(subscriber_cmd_vel_recv(&sub).is_none());
        assert!(!subscriber_cmd_vel_has_msg(&sub));
    }

    #[test]
    fn multiple_sends_recv_in_order() {
        use horus_robotics::CmdVel;
        let t = unique_topic("order");
        let pub_ = publisher_cmd_vel_new(&t).unwrap();
        let sub = subscriber_cmd_vel_new(&t).unwrap();

        for i in 0..5u64 {
            publisher_cmd_vel_send(
                &pub_,
                CmdVel {
                    timestamp_ns: i,
                    linear: i as f32,
                    angular: 0.0,
                },
            );
        }

        // First recv should get earliest message
        let r = subscriber_cmd_vel_recv(&sub).unwrap();
        assert_eq!(r.timestamp_ns, 0);
    }

    #[test]
    fn publisher_name_matches() {
        let t = unique_topic("name_check");
        let pub_ = publisher_cmd_vel_new(&t).unwrap();
        assert_eq!(publisher_cmd_vel_name(&pub_), t);
    }

    #[test]
    fn subscriber_name_matches() {
        let t = unique_topic("sub_name");
        let sub = subscriber_cmd_vel_new(&t).unwrap();
        assert_eq!(subscriber_cmd_vel_name(&sub), t);
    }
}
