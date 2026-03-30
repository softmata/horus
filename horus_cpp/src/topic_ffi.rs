//! FFI wrappers for Topic<T> publishing and subscribing.
//!
//! ## Design
//!
//! For POD message types, the FFI uses typed wrappers per message type.
//! Each concrete type gets its own FFI functions:
//!   - `publisher_cmd_vel_new(name)` → `Box<FfiPublisher<CmdVel>>`
//!   - `publisher_cmd_vel_send(pub, msg)` → sends via Topic<CmdVel>
//!   - `subscriber_cmd_vel_new(name)` → `Box<FfiSubscriber<CmdVel>>`
//!   - `subscriber_cmd_vel_recv(sub)` → Option<CmdVel>
//!
//! This monomorphized approach avoids generic FFI (which CXX can't handle)
//! while maintaining type safety and zero-copy for POD messages.

use horus_core::communication::Topic;

// ─── Opaque Publisher Wrapper ────────────────────────────────────────────────

/// FFI-safe publisher wrapping a horus_core Topic<T>.
///
/// One instance per (topic_name, message_type) pair.
/// Thread-local — must be used from the thread that created it.
pub struct FfiPublisher<T: horus_core::communication::TopicMessage> {
    topic: Topic<T>,
}

/// FFI-safe subscriber wrapping a horus_core Topic<T>.
pub struct FfiSubscriber<T: horus_core::communication::TopicMessage> {
    topic: Topic<T>,
}

// ─── Macro for Monomorphized FFI ─────────────────────────────────────────────

/// Generate typed publisher/subscriber FFI functions for a concrete message type.
///
/// Usage: `impl_topic_ffi!(cmd_vel, CmdVel, horus_library::CmdVel);`
/// Generates:
///   - publisher_cmd_vel_new(name) → Box<FfiPublisher<CmdVel>>
///   - publisher_cmd_vel_send(pub, msg) → Result<()>
///   - subscriber_cmd_vel_new(name) → Box<FfiSubscriber<CmdVel>>
///   - subscriber_cmd_vel_recv(sub) → *const T (null if no message)
///   - subscriber_cmd_vel_has_msg(sub) → bool
macro_rules! impl_topic_ffi {
    ($snake:ident, $type_name:ident, $type_path:path) => {
        paste::paste! {
            // ─── Publisher ───────────────────────────────────────────

            /// Create a new publisher for this message type.
            pub fn [<publisher_ $snake _new>](name: &str) -> Result<Box<FfiPublisher<$type_path>>, String> {
                let topic = Topic::<$type_path>::new(name).map_err(|e| e.to_string())?;
                Ok(Box::new(FfiPublisher { topic }))
            }

            /// Send a message through this publisher.
            pub fn [<publisher_ $snake _send>](
                publisher: &FfiPublisher<$type_path>,
                msg: $type_path,
            ) {
                publisher.topic.send(msg);
            }

            /// Get the topic name.
            pub fn [<publisher_ $snake _name>](publisher: &FfiPublisher<$type_path>) -> &str {
                publisher.topic.name()
            }

            // ─── Subscriber ─────────────────────────────────────────

            /// Create a new subscriber for this message type.
            pub fn [<subscriber_ $snake _new>](name: &str) -> Result<Box<FfiSubscriber<$type_path>>, String> {
                let topic = Topic::<$type_path>::new(name).map_err(|e| e.to_string())?;
                Ok(Box::new(FfiSubscriber { topic }))
            }

            /// Receive a message (returns None if no message available).
            pub fn [<subscriber_ $snake _recv>](
                subscriber: &FfiSubscriber<$type_path>,
            ) -> Option<$type_path> {
                subscriber.topic.recv()
            }

            /// Check if a message is available without consuming it.
            pub fn [<subscriber_ $snake _has_msg>](
                subscriber: &FfiSubscriber<$type_path>,
            ) -> bool {
                subscriber.topic.has_message()
            }

            /// Get the topic name.
            pub fn [<subscriber_ $snake _name>](subscriber: &FfiSubscriber<$type_path>) -> &str {
                subscriber.topic.name()
            }
        }
    };
}

// ─── Concrete Type Implementations ───────────────────────────────────────────

// ─── Wire Format (JSON transport) ────────────────────────────────────────────
impl_topic_ffi!(
    json_wire,
    JsonWireMessage,
    crate::types_ffi::JsonWireMessage
);

// ─── Core Message Types (11 total) ───────────────────────────────────────────

// Application
impl_topic_ffi!(cmd_vel, CmdVel, horus_library::CmdVel);

// Sensor
impl_topic_ffi!(laser_scan, LaserScan, horus_library::LaserScan);
impl_topic_ffi!(imu, Imu, horus_library::Imu);
impl_topic_ffi!(odometry, Odometry, horus_library::Odometry);
impl_topic_ffi!(joint_state, JointState, horus_library::JointState);

// Geometry
impl_topic_ffi!(twist, Twist, horus_library::Twist);
impl_topic_ffi!(pose2d, Pose2D, horus_library::Pose2D);
impl_topic_ffi!(
    transform_stamped,
    TransformStamped,
    horus_library::TransformStamped
);

// Navigation
impl_topic_ffi!(nav_goal, NavGoal, horus_library::NavGoal);

// Diagnostics
impl_topic_ffi!(heartbeat, Heartbeat, horus_library::Heartbeat);
impl_topic_ffi!(emergency_stop, EmergencyStop, horus_library::EmergencyStop);

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use horus_library::CmdVel;

    #[test]
    fn publisher_cmd_vel_creates_topic() {
        let pub_ = publisher_cmd_vel_new("test.cpp.cmd_vel").unwrap();
        assert_eq!(publisher_cmd_vel_name(&pub_), "test.cpp.cmd_vel");
    }

    #[test]
    fn publisher_cmd_vel_send_no_panic() {
        let pub_ = publisher_cmd_vel_new("test.cpp.send").unwrap();
        let msg = CmdVel {
            timestamp_ns: 1000,
            linear: 0.5,
            angular: 0.1,
        };
        publisher_cmd_vel_send(&pub_, msg);
        // No panic = success (message sent to SHM)
    }

    #[test]
    fn subscriber_cmd_vel_creates_topic() {
        let sub = subscriber_cmd_vel_new("test.cpp.sub").unwrap();
        assert_eq!(subscriber_cmd_vel_name(&sub), "test.cpp.sub");
    }

    #[test]
    fn subscriber_recv_none_when_empty() {
        let sub = subscriber_cmd_vel_new("test.cpp.empty").unwrap();
        assert!(subscriber_cmd_vel_recv(&sub).is_none());
    }

    #[test]
    fn subscriber_has_msg_false_when_empty() {
        let sub = subscriber_cmd_vel_new("test.cpp.no_msg").unwrap();
        assert!(!subscriber_cmd_vel_has_msg(&sub));
    }

    #[test]
    fn pub_sub_roundtrip() {
        let topic_name = format!("test.cpp.roundtrip.{}", std::process::id());

        let pub_ = publisher_cmd_vel_new(&topic_name).unwrap();
        let sub = subscriber_cmd_vel_new(&topic_name).unwrap();

        let msg = CmdVel {
            timestamp_ns: 42,
            linear: 1.5,
            angular: -0.3,
        };
        publisher_cmd_vel_send(&pub_, msg);

        let received = subscriber_cmd_vel_recv(&sub);
        assert!(received.is_some(), "should receive the sent message");
        let received = received.unwrap();
        assert_eq!(received.timestamp_ns, 42);
        assert!((received.linear - 1.5).abs() < f32::EPSILON);
        assert!((received.angular - (-0.3)).abs() < f32::EPSILON);
    }

    #[test]
    fn multiple_sends() {
        let topic_name = format!("test.cpp.multi.{}", std::process::id());

        let pub_ = publisher_cmd_vel_new(&topic_name).unwrap();
        let sub = subscriber_cmd_vel_new(&topic_name).unwrap();

        for i in 0..5 {
            let msg = CmdVel {
                timestamp_ns: i,
                linear: i as f32,
                angular: 0.0,
            };
            publisher_cmd_vel_send(&pub_, msg);
        }

        // Should receive at least the first message
        let received = subscriber_cmd_vel_recv(&sub);
        assert!(received.is_some());
    }

    // ─── FFI Overhead Benchmarks ─────────────────────────────────────

    #[test]
    fn benchmark_ffi_send_overhead() {
        use std::time::Instant;

        let iterations = 10_000;
        let topic_name = format!("bench.ffi.send.{}", std::process::id());

        // Direct Rust path
        let direct_topic = Topic::<CmdVel>::new(&format!("{}.direct", topic_name)).unwrap();
        let msg = CmdVel {
            timestamp_ns: 1,
            linear: 0.5,
            angular: 0.1,
        };

        let start = Instant::now();
        for _ in 0..iterations {
            direct_topic.send(msg);
        }
        let direct_elapsed = start.elapsed();

        // FFI wrapper path
        let ffi_pub = publisher_cmd_vel_new(&format!("{}.ffi", topic_name)).unwrap();

        let start = Instant::now();
        for _ in 0..iterations {
            publisher_cmd_vel_send(&ffi_pub, msg);
        }
        let ffi_elapsed = start.elapsed();

        let direct_ns = direct_elapsed.as_nanos() as f64 / iterations as f64;
        let ffi_ns = ffi_elapsed.as_nanos() as f64 / iterations as f64;
        let overhead_ns = ffi_ns - direct_ns;

        eprintln!("\n╔══════════════════════════════════════════════════╗");
        eprintln!(
            "║  FFI Overhead Benchmark (CmdVel, {} iters)  ║",
            iterations
        );
        eprintln!("╠══════════════════════════════════════════════════╣");
        eprintln!(
            "║  Direct Rust:  {:>8.1} ns/send                 ║",
            direct_ns
        );
        eprintln!("║  FFI wrapper:  {:>8.1} ns/send                 ║", ffi_ns);
        eprintln!(
            "║  FFI overhead: {:>8.1} ns/send                 ║",
            overhead_ns
        );
        eprintln!("╚══════════════════════════════════════════════════╝\n");

        // The FFI wrapper is a thin inline function call — overhead should be minimal.
        // Allow generous threshold for CI (actual overhead is typically <10ns).
        assert!(
            overhead_ns < 500.0,
            "FFI overhead {:.1}ns exceeds 500ns threshold — \
             check for unexpected allocations in the wrapper",
            overhead_ns,
        );
    }
}
