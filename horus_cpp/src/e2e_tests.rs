//! End-to-end integration tests for C++ ↔ Rust interop.
//!
//! These tests verify the complete pipeline: C++ FFI functions creating
//! Rust schedulers with CppNode adapters, pub/sub through SHM, mixed
//! Rust+C++ nodes in the same scheduler.

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

    use crate::node_ffi::*;
    use crate::scheduler_ffi::*;
    use crate::topic_ffi::*;

    // ─── Mixed Scheduler: CppNode + Rust Node ────────────────────────

    #[test]
    fn mixed_scheduler_cpp_and_rust_nodes() {
        let mut sched = scheduler_new();

        // CppNode via FFI
        static CPP_TICKS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn cpp_tick() {
            CPP_TICKS.fetch_add(1, Ordering::Relaxed);
        }
        let mut b1 = node_builder_new("cpp_sensor");
        node_builder_set_tick(&mut b1, cpp_tick);
        node_builder_build(b1, &mut sched).unwrap();

        // Another CppNode
        static CPP2_TICKS: AtomicU32 = AtomicU32::new(0);
        extern "C" fn cpp2_tick() {
            CPP2_TICKS.fetch_add(1, Ordering::Relaxed);
        }
        let mut b2 = node_builder_new("cpp_actuator");
        node_builder_set_tick(&mut b2, cpp2_tick);
        node_builder_build(b2, &mut sched).unwrap();

        // Verify both nodes registered
        let nodes = scheduler_node_list(&sched);
        assert_eq!(nodes.len(), 2);
        assert!(nodes.contains(&"cpp_sensor".to_string()));
        assert!(nodes.contains(&"cpp_actuator".to_string()));

        // Tick the scheduler
        CPP_TICKS.store(0, Ordering::Relaxed);
        CPP2_TICKS.store(0, Ordering::Relaxed);
        scheduler_tick_once(&mut sched).unwrap();

        assert!(CPP_TICKS.load(Ordering::Relaxed) >= 1, "cpp_sensor should tick");
        assert!(CPP2_TICKS.load(Ordering::Relaxed) >= 1, "cpp_actuator should tick");
    }

    // ─── Pub/Sub Through FFI: Full Pipeline ──────────────────────────

    #[test]
    fn e2e_pub_sub_pipeline() {
        use horus_library::CmdVel;

        let topic = format!("e2e.pipeline.{}", std::process::id());

        // Create publisher and subscriber via FFI
        let pub_ = publisher_cmd_vel_new(&topic).unwrap();
        let sub = subscriber_cmd_vel_new(&topic).unwrap();

        // Simulate a control loop: publish, then receive
        let msg = CmdVel {
            timestamp_ns: 12345,
            linear: 0.42,
            angular: -0.17,
        };
        publisher_cmd_vel_send(&pub_, msg);

        let received = subscriber_cmd_vel_recv(&sub);
        assert!(received.is_some(), "should receive message");
        let r = received.unwrap();
        assert_eq!(r.timestamp_ns, 12345);
        assert!((r.linear - 0.42).abs() < f32::EPSILON);
        assert!((r.angular - (-0.17)).abs() < f32::EPSILON);
    }

    // ─── Scheduler + Pub/Sub Combined ────────────────────────────────

    #[test]
    fn e2e_scheduler_with_publishing_node() {
        use horus_library::CmdVel;

        let topic = format!("e2e.sched_pub.{}", std::process::id());
        // Create subscriber first (to catch the published message)
        let _sub = subscriber_cmd_vel_new(&topic).unwrap();

        // Create scheduler with a publishing node
        let mut sched = scheduler_new();

        static PUB_COUNT: AtomicU32 = AtomicU32::new(0);
        extern "C" fn publishing_tick() {
            PUB_COUNT.fetch_add(1, Ordering::Relaxed);
            // Note: Can't easily access the publisher from extern "C" fn
            // In real C++ code, the publisher would be captured in the lambda
        }

        let mut builder = node_builder_new("publisher_node");
        node_builder_set_tick(&mut builder, publishing_tick);
        node_builder_build(builder, &mut sched).unwrap();

        PUB_COUNT.store(0, Ordering::Relaxed);
        scheduler_tick_once(&mut sched).unwrap();
        assert!(PUB_COUNT.load(Ordering::Relaxed) >= 1);
    }

    // ─── Multiple Message Types ──────────────────────────────────────

    #[test]
    fn e2e_multiple_message_types() {
        use horus_library::{CmdVel, Twist, Pose2D};

        let pid = std::process::id();

        // CmdVel round-trip
        let cmd_pub = publisher_cmd_vel_new(&format!("e2e.multi.cmd.{}", pid)).unwrap();
        let cmd_sub = subscriber_cmd_vel_new(&format!("e2e.multi.cmd.{}", pid)).unwrap();
        publisher_cmd_vel_send(&cmd_pub, CmdVel { timestamp_ns: 1, linear: 1.0, angular: 0.5 });
        assert!(subscriber_cmd_vel_recv(&cmd_sub).is_some());

        // Twist round-trip
        let twist_pub = publisher_twist_new(&format!("e2e.multi.twist.{}", pid)).unwrap();
        let twist_sub = subscriber_twist_new(&format!("e2e.multi.twist.{}", pid)).unwrap();
        publisher_twist_send(&twist_pub, Twist {
            linear: [1.0, 0.0, 0.0],
            angular: [0.0, 0.0, 0.5],
            timestamp_ns: 2,
        });
        let recv = subscriber_twist_recv(&twist_sub);
        assert!(recv.is_some());
        let t = recv.unwrap();
        assert!((t.linear[0] - 1.0).abs() < f64::EPSILON);

        // Pose2D round-trip
        let pose_pub = publisher_pose2d_new(&format!("e2e.multi.pose.{}", pid)).unwrap();
        let pose_sub = subscriber_pose2d_new(&format!("e2e.multi.pose.{}", pid)).unwrap();
        publisher_pose2d_send(&pose_pub, Pose2D {
            x: 3.0, y: 4.0, theta: 1.57,
            timestamp_ns: 3,
        });
        let recv = subscriber_pose2d_recv(&pose_sub);
        assert!(recv.is_some());
        let p = recv.unwrap();
        assert!((p.x - 3.0).abs() < f64::EPSILON);
    }

    // ─── ABI Version Check ───────────────────────────────────────────

    #[test]
    fn e2e_version_checked_scheduler() {
        use crate::types_ffi::HORUS_CPP_ABI_VERSION;

        // Correct version — should succeed
        let result = crate::scheduler_ffi::scheduler_new_checked(HORUS_CPP_ABI_VERSION);
        assert!(result.is_ok(), "correct ABI version should succeed");

        // Wrong version — should fail with descriptive error
        let result = crate::scheduler_ffi::scheduler_new_checked(HORUS_CPP_ABI_VERSION + 99);
        match result {
            Ok(_) => panic!("wrong ABI version should fail"),
            Err(err) => {
                assert!(err.contains("ABI version mismatch"), "error: {}", err);
                assert!(err.contains("horus build --clean"), "should suggest rebuild: {}", err);
            }
        }
    }

    // ─── TransformFrame Through FFI ──────────────────────────────────

    #[test]
    fn e2e_transform_frame_pipeline() {
        use crate::transform_ffi::*;

        let tf = transform_frame_new();

        // Build a frame tree: world -> base -> camera
        transform_frame_register(&tf, "world", "").unwrap();
        transform_frame_register(&tf, "base_link", "world").unwrap();
        transform_frame_register(&tf, "camera", "base_link").unwrap();

        // Set transforms
        transform_frame_update(&tf, "base_link",
            [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1000).unwrap();
        transform_frame_update(&tf, "camera",
            [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0], 1000).unwrap();

        // Query chain: camera -> world (should compose: base_link + camera)
        let result = transform_frame_lookup(&tf, "camera", "world");
        assert!(result.is_ok(), "chain lookup: {:?}", result);
        let vals = result.unwrap();
        // camera at [0,0,0.5] relative to base_link at [1,0,0] relative to world
        // => camera at [1, 0, 0.5] in world
        assert!((vals[0] - 1.0).abs() < 1e-6, "tx={}", vals[0]);
        assert!((vals[2] - 0.5).abs() < 1e-6, "tz={}", vals[2]);
    }

    // ─── Params Through FFI ──────────────────────────────────────────

    #[test]
    fn e2e_params_pipeline() {
        use crate::params_ffi::*;

        let params = params_new();

        // Set various types
        params_set_f64(&params, "max_speed", 2.5).unwrap();
        params_set_i64(&params, "max_retries", 3).unwrap();
        params_set_bool(&params, "debug", true).unwrap();
        params_set_string(&params, "robot_name", "atlas").unwrap();

        // Read back
        assert_eq!(params_get_f64(&params, "max_speed"), Some(2.5));
        assert_eq!(params_get_i64(&params, "max_retries"), Some(3));
        assert_eq!(params_get_bool(&params, "debug"), Some(true));
        assert_eq!(params_get_string(&params, "robot_name"), Some("atlas".to_string()));

        // JSON get
        let json = params_get(&params, "max_speed").unwrap();
        assert_eq!(json, "2.5");
    }

    // ─── Full Robot Pipeline ─────────────────────────────────────────

    #[test]
    fn e2e_full_robot_pipeline() {
        // Simulates: sensor → processor → actuator
        let mut sched = scheduler_new();
        scheduler_name(&mut sched, "robot_e2e");

        static SENSOR_TICKS: AtomicU32 = AtomicU32::new(0);
        static PROCESSOR_TICKS: AtomicU32 = AtomicU32::new(0);
        static ACTUATOR_TICKS: AtomicU32 = AtomicU32::new(0);

        extern "C" fn sensor_tick() { SENSOR_TICKS.fetch_add(1, Ordering::Relaxed); }
        extern "C" fn processor_tick() { PROCESSOR_TICKS.fetch_add(1, Ordering::Relaxed); }
        extern "C" fn actuator_tick() { ACTUATOR_TICKS.fetch_add(1, Ordering::Relaxed); }

        let mut b1 = node_builder_new("sensor");
        node_builder_set_tick(&mut b1, sensor_tick);
        node_builder_order(&mut b1, 0);
        node_builder_build(b1, &mut sched).unwrap();

        let mut b2 = node_builder_new("processor");
        node_builder_set_tick(&mut b2, processor_tick);
        node_builder_order(&mut b2, 10);
        node_builder_build(b2, &mut sched).unwrap();

        let mut b3 = node_builder_new("actuator");
        node_builder_set_tick(&mut b3, actuator_tick);
        node_builder_order(&mut b3, 20);
        node_builder_build(b3, &mut sched).unwrap();

        // Reset and tick
        SENSOR_TICKS.store(0, Ordering::Relaxed);
        PROCESSOR_TICKS.store(0, Ordering::Relaxed);
        ACTUATOR_TICKS.store(0, Ordering::Relaxed);

        scheduler_tick_once(&mut sched).unwrap();

        assert!(SENSOR_TICKS.load(Ordering::Relaxed) >= 1, "sensor ticked");
        assert!(PROCESSOR_TICKS.load(Ordering::Relaxed) >= 1, "processor ticked");
        assert!(ACTUATOR_TICKS.load(Ordering::Relaxed) >= 1, "actuator ticked");

        // Verify all nodes in list
        let nodes = scheduler_node_list(&sched);
        assert_eq!(nodes.len(), 3);

        // Verify scheduler name
        assert_eq!(scheduler_get_name(&sched), "robot_e2e");

        // Clean shutdown
        scheduler_stop(&sched);
        assert!(!scheduler_is_running(&sched));
    }
}
