#![allow(dead_code)]
//! DX integration tests — verify that beginner workflows compile and work.
//!
//! These tests exercise the user-facing API surface to ensure that common
//! patterns are ergonomic and produce correct results.

use horus::prelude::*;
use horus::serde_json;

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
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(CounterNode { count: 0 })
        .order(0)
        .build()
        .unwrap();

    // Run for a short duration
    scheduler.run_for(100_u64.ms()).unwrap();
}

// ============================================================================
// 5. Builder API uses .build() convention
// ============================================================================

#[test]
fn node_builder_uses_build() {
    let mut scheduler = Scheduler::new();

    // .build() is the primary method (returns Result)
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
    // Verify Miss enum variants are accessible and default is Warn
    let policy = Miss::Warn;
    assert_eq!(
        policy,
        Miss::default(),
        "Miss::Warn should be the default variant"
    );
    assert_ne!(policy, Miss::Skip);
    assert_ne!(policy, Miss::SafeMode);
    assert_ne!(policy, Miss::Stop);

    // Verify RtStats default has zeroed counters
    let stats = RtStats::default();
    assert_eq!(stats.sampled_ticks(), 0);
    assert_eq!(stats.deadline_misses(), 0);
    assert_eq!(stats.budget_violations(), 0);
    assert_eq!(stats.worst_execution(), std::time::Duration::ZERO);
    assert_eq!(stats.avg_execution_us(), 0.0);
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
    let err = Error::Communication(CommunicationError::TopicFull {
        topic: "test".to_string(),
    });

    assert!(
        matches!(&err, Error::Communication(CommunicationError::TopicFull { topic }) if topic == "test"),
        "Expected TopicFull, got {:?}",
        err
    );

    let err2 = Error::Memory(MemoryError::PoolExhausted {
        reason: "out of slots".to_string(),
    });

    assert!(
        matches!(&err2, Error::Memory(MemoryError::PoolExhausted { reason }) if reason.contains("slots")),
        "Expected PoolExhausted, got {:?}",
        err2
    );
}

// ============================================================================
// 9. Twist 2D convenience
// ============================================================================
// CmdVel lives in horus-robotics (a git dep, not available here).
// Test the Twist::new_2d helper which is the prelude-available equivalent.

#[test]
fn twist_2d_roundtrip() {
    let twist = Twist::new_2d(1.5, 0.3);
    assert!((twist.linear[0] - 1.5).abs() < 1e-6);
    assert!((twist.angular[2] - 0.3).abs() < 1e-6);
    // Other axes are zero for 2D motion
    assert!((twist.linear[1]).abs() < 1e-6);
    assert!((twist.linear[2]).abs() < 1e-6);
    assert!((twist.angular[0]).abs() < 1e-6);
    assert!((twist.angular[1]).abs() < 1e-6);
}

// ============================================================================
// 10. Topic::new() returns Result (no panicking create())
// ============================================================================

#[test]
fn topic_new_returns_result() {
    let topic: Topic<String> = Topic::new("test_dx_topic").unwrap();

    // Verify the topic was created with the correct name
    assert_eq!(topic.name(), "test_dx_topic");

    // No messages pending on a fresh topic
    assert!(!topic.has_message());
    assert_eq!(topic.pending_count(), 0);
    assert_eq!(topic.dropped_count(), 0);
}

// ============================================================================
// 10b. Topic::try_send() and dropped_count()
// ============================================================================

#[test]
fn topic_try_send_and_dropped_count() {
    // A full ring only drops if somebody is subscribed to it.
    //
    // `send()` is the lossy publish, and on a ring that is full with NOTHING
    // draining it, `send_lossy_retry` keeps the last N: it retires the oldest
    // slot and takes it, so the message is delivered and no drop is counted.
    // That is deliberate — a producer whose subscriber died (or never existed)
    // must not hold the newest data hostage — and horus_core pins it in
    // `send_fire_and_forget_keeps_the_newest_on_a_full_unread_ring`. This test
    // predates that behaviour and used to lean on the old drop-newest path, so
    // with no subscriber registered it asserted a drop that can no longer
    // happen.
    //
    // Backpressure — and the drop this test is about — exists to protect a real
    // consumer's unread data, so the subscriber has to be registered BEFORE the
    // ring fills: `nothing_is_draining()` is only consulted on the full-ring
    // path. A unique topic name keeps that decision from being made against a
    // header some earlier run left in /dev/shm (participant registrations and
    // the stall clock outlive the process that wrote them).
    let name = format!("test_dx_try_send_{}", std::process::id());
    let topic: Topic<u64> = Topic::with_capacity(&name, 2, None).unwrap();
    let subscriber: Topic<u64> = Topic::with_capacity(&name, 2, None).unwrap();
    // First recv() registers the handle as a consumer; it then never drains,
    // which is exactly the slow-subscriber case dropped_count() reports on.
    assert_eq!(subscriber.recv(), None, "a fresh ring starts empty");

    // Fill the ring by sending until it says it is full, rather than assuming a
    // capacity request of 2 yields exactly two usable slots — `with_capacity`
    // rounds to a power of two, and the usable window is a property of the
    // backend that gets selected. The bound only exists so a ring that stopped
    // applying backpressure fails the assert below instead of looping forever.
    const FILL_LIMIT: u64 = 64;
    let mut sent = 0u64;
    while sent < FILL_LIMIT && topic.try_send(sent).is_ok() {
        sent += 1;
    }
    assert!(
        sent < FILL_LIMIT,
        "ring accepted {sent} messages without ever reporting full — try_send has lost its backpressure"
    );

    // try_send hands the message back instead of dropping it, so a rejected
    // try_send is not a drop and must not be counted as one.
    assert_eq!(topic.try_send(sent).unwrap_err(), sent);
    assert_eq!(topic.dropped_count(), 0);

    // send() on a full ring whose subscriber is not draining: spin, yield, then
    // drop — and that drop is what dropped_count() reports.
    topic.send(sent);
    assert!(
        topic.dropped_count() > 0,
        "send() on a full ring with a registered subscriber must count the dropped message"
    );
    // dropped_count() is the publisher-facing name for the same counter.
    assert_eq!(topic.metrics().send_failures(), topic.dropped_count());
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
    let mut scheduler = Scheduler::new().tick_rate(500_u64.hz());
    scheduler
        .add(CounterNode { count: 0 })
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    // Runs without panic — proves tick_rate config is applied at run() time
    scheduler.run_for(50_u64.ms()).unwrap();
}

// ============================================================================
// 12. FailurePolicy accessible and constructible
// ============================================================================

#[test]
fn failure_policy_accessible() {
    let _fatal = FailurePolicy::Fatal;
    let restart = FailurePolicy::restart(3, 100_u64.ms());
    match restart {
        FailurePolicy::Restart { max_restarts, .. } => assert_eq!(max_restarts, 3),
        other => unreachable!("Expected Restart, got {:?}", other),
    }
}
