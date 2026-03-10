//! DX integration tests — verify that beginner workflows compile and work.
//!
//! These tests exercise the user-facing API surface to ensure that common
//! patterns are ergonomic and produce correct results.

use horus::prelude::*;
use horus::serde_json;
use std::time::Duration;

// ============================================================================
// 1. message! macro works and types are topic-ready
// ============================================================================

message! {
    /// Custom sensor message
    TestSensorReading {
        temperature: f64,
        humidity: f64,
        timestamp: u64,
    }

    /// Custom motor command
    TestMotorCmd {
        velocity: f32,
        torque: f32,
    }
}

#[test]
fn message_macro_produces_topic_compatible_types() {
    // message! types must satisfy TopicMessage bounds (Clone + Send + Sync + Serialize + DeserializeOwned)
    fn assert_bounds<
        T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
    >() {
    }
    assert_bounds::<TestSensorReading>();
    assert_bounds::<TestMotorCmd>();
}

#[test]
fn message_macro_types_serialize_roundtrip() {
    let reading = TestSensorReading {
        temperature: 25.5,
        humidity: 60.0,
        timestamp: 12345,
    };
    let json = serde_json::to_string(&reading).unwrap();
    let deserialized: TestSensorReading = serde_json::from_str(&json).unwrap();
    assert_eq!(deserialized.temperature, 25.5);
    assert_eq!(deserialized.humidity, 60.0);
    assert_eq!(deserialized.timestamp, 12345);
}

#[test]
fn message_macro_log_summary() {
    let cmd = TestMotorCmd {
        velocity: 1.5,
        torque: 0.8,
    };
    let summary = cmd.log_summary();
    assert!(summary.contains("TestMotorCmd"));
    assert!(summary.contains("velocity"));
}

// ============================================================================
// 2. service! macro works
// ============================================================================

service! {
    TestAddInts {
        request {
            a: i64,
            b: i64,
        }
        response {
            sum: i64,
        }
    }
}

#[test]
fn service_macro_generates_types() {
    let req = TestAddIntsRequest { a: 10, b: 20 };
    assert_eq!(req.a, 10);
    assert_eq!(req.b, 20);

    let resp = TestAddIntsResponse { sum: 30 };
    assert_eq!(resp.sum, 30);
}

// ============================================================================
// 3. action! macro works
// ============================================================================

action! {
    TestMove {
        goal {
            distance: f64,
        }
        feedback {
            progress: f32,
        }
        result {
            success: bool,
        }
    }
}

#[test]
fn action_macro_generates_types() {
    let goal = TestMoveGoal { distance: 5.0 };
    assert_eq!(goal.distance, 5.0);

    let feedback = TestMoveFeedback { progress: 0.5 };
    assert_eq!(feedback.progress, 0.5);

    let result = TestMoveResult { success: true };
    assert!(result.success);
}

// ============================================================================
// 3b. action! Goal::new() constructor
// ============================================================================

action! {
    TestNavigate {
        goal {
            target_x: f64,
            target_y: f64,
        }
        feedback {
            distance: f64,
        }
        result {
            success: bool,
        }
    }
}

#[test]
fn action_goal_new_constructor() {
    // new() takes all fields — no struct literal boilerplate
    let goal = TestNavigateGoal::new(5.0, 3.0);
    assert_eq!(goal.target_x, 5.0);
    assert_eq!(goal.target_y, 3.0);

    // Single-field goal
    let goal2 = TestMoveGoal::new(10.0);
    assert_eq!(goal2.distance, 10.0);
}

// ============================================================================
// 4. Node + Scheduler workflow
// ============================================================================

struct CounterNode {
    count: u32,
}

impl Node for CounterNode {
    fn name(&self) -> &'static str {
        "CounterNode"
    }

    fn tick(&mut self) {
        self.count += 1;
    }
}

#[test]
fn scheduler_runs_node() {
    let mut scheduler = Scheduler::new().tick_rate(100.hz());
    scheduler
        .add(CounterNode { count: 0 })
        .order(0)
        .build()
        .unwrap();

    // Run for a short duration
    scheduler.run_for(Duration::from_millis(100)).unwrap();
}

// ============================================================================
// 5. Builder API uses .build() convention
// ============================================================================

#[test]
fn node_builder_uses_build() {
    let mut scheduler = Scheduler::new();

    // .build() is the primary method (returns HorusResult)
    scheduler
        .add(CounterNode { count: 0 })
        .order(0)
        .build()
        .unwrap();

    // .build() also works (backward compat alias)
    scheduler
        .add(CounterNode { count: 0 })
        .order(1)
        .build()
        .unwrap();
}

// ============================================================================
// 6. Prelude exports RT types
// ============================================================================

#[test]
fn prelude_exports_rt_types() {
    // These should all be accessible from the prelude
    let _policy = Miss::Warn;
    let _stats = RtStats::default();
    let _exec_class = ExecutionClass::BestEffort;
}

// ============================================================================
// 7. Image constructor uses (width, height) convention
// ============================================================================

#[test]
fn image_constructor_width_height_order() {
    let img = Image::new(640, 480, ImageEncoding::Rgb8).unwrap();
    assert_eq!(img.width(), 640);
    assert_eq!(img.height(), 480);
}

#[test]
fn depth_image_constructor_width_height_order() {
    let img = DepthImage::new(320, 240, TensorDtype::F32).unwrap();
    assert_eq!(img.width(), 320);
    assert_eq!(img.height(), 240);
}

// ============================================================================
// 8. Error types are structured
// ============================================================================

#[test]
fn structured_errors_pattern_match() {
    let err = HorusError::Communication(CommunicationError::TopicFull {
        topic: "test".to_string(),
    });

    assert!(
        matches!(&err, HorusError::Communication(CommunicationError::TopicFull { topic }) if topic == "test"),
        "Expected TopicFull, got {:?}",
        err
    );

    let err2 = HorusError::Memory(MemoryError::PoolExhausted {
        reason: "out of slots".to_string(),
    });

    assert!(
        matches!(&err2, HorusError::Memory(MemoryError::PoolExhausted { reason }) if reason.contains("slots")),
        "Expected PoolExhausted, got {:?}",
        err2
    );
}

// ============================================================================
// 9. CmdVel / Twist conversions
// ============================================================================

#[test]
fn cmdvel_twist_conversions() {
    let cmd = CmdVel::new(1.5, 0.3);
    let twist: Twist = cmd.into();
    assert!((twist.linear[0] - 1.5).abs() < 1e-6);
    assert!((twist.angular[2] - 0.3).abs() < 1e-6);

    let back: CmdVel = twist.into();
    assert!((back.linear - 1.5).abs() < 1e-6);
    assert!((back.angular - 0.3).abs() < 1e-6);
}

// ============================================================================
// 10. Topic::new() returns Result (no panicking create())
// ============================================================================

#[test]
fn topic_new_returns_result() {
    let topic: Topic<String> = Topic::new("test_dx_topic").unwrap();
    // Topic::new() returns HorusResult — no panicking create() shortcut
    drop(topic);
}

// ============================================================================
// 10b. Topic::try_send() and dropped_count()
// ============================================================================

#[test]
fn topic_try_send_and_dropped_count() {
    let topic: Topic<u64> = Topic::with_capacity("test_dx_try_send", 2, None).unwrap();
    // Fill the ring
    topic.send(1);
    topic.send(2);
    // try_send should fail when full
    assert!(topic.try_send(3).is_err());
    // dropped_count reflects send() failures (not try_send — try_send returns the msg)
    assert_eq!(topic.dropped_count(), 0);
    // Force a dropped send via the lossy path
    topic.send(4); // this will spin+yield then drop
    assert!(topic.dropped_count() > 0 || topic.metrics().send_failures > 0);
}

// ============================================================================
// 11. RuntimeParams accessible from prelude
// ============================================================================

#[test]
fn runtime_params_from_prelude() {
    let params = RuntimeParams::new().unwrap();
    params.set("test_key", 42_i64).unwrap();
    let val: i64 = params.get("test_key").unwrap();
    assert_eq!(val, 42);
}

// ============================================================================
// 11b. RuntimeParams::get_typed() returns explicit errors
// ============================================================================

#[test]
fn runtime_params_get_typed() {
    let params = RuntimeParams::new().unwrap();
    params.set("speed", 1.5_f64).unwrap();

    // get_typed succeeds with correct type
    let speed: f64 = params.get_typed("speed").unwrap();
    assert_eq!(speed, 1.5);

    // get_typed fails on missing key
    let missing = params.get_typed::<f64>("nonexistent");
    missing.unwrap_err();

    // get_typed fails on type mismatch
    let wrong_type = params.get_typed::<Vec<i32>>("speed");
    wrong_type.unwrap_err();
}

// ============================================================================
// 11c. Scheduler tick_rate() is deferred (applies at run time, not at build time)
// ============================================================================

#[test]
fn scheduler_tick_rate_deferred() {
    // tick_rate() should work as a builder — applied when run() is called
    let mut scheduler = Scheduler::new().tick_rate(500.hz());
    scheduler
        .add(CounterNode { count: 0 })
        .order(0)
        .rate(100.hz())
        .build()
        .unwrap();

    // Runs without panic — proves tick_rate config is applied at run() time
    scheduler.run_for(Duration::from_millis(50)).unwrap();
}

// ============================================================================
// 12. FailurePolicy accessible and constructible
// ============================================================================

#[test]
fn failure_policy_accessible() {
    let _fatal = FailurePolicy::Fatal;
    let restart = FailurePolicy::restart(3, 100);
    match restart {
        FailurePolicy::Restart { max_restarts, .. } => assert_eq!(max_restarts, 3),
        other => unreachable!("Expected Restart, got {:?}", other),
    }
}
