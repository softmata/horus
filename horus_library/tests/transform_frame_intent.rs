//! TransformFrame Intent Verification Tests (Level 7)
//!
//! These tests verify that TransformFrame achieves its **purpose** — not merely
//! that it doesn't crash, but that the transforms it produces are geometrically
//! correct. Each test encodes a specific intent with hand-computed expected values.
//!
//! ## Why these tests exist
//!
//! Lower-level tests prove that individual operations work. These tests prove
//! the system **does the right thing** end-to-end:
//!
//! - Chained transforms compose correctly (translation + rotation)
//! - Inverse lookups return the geometric inverse
//! - Same-frame lookups return identity
//! - Static frames are immutable under dynamic updates
//! - Rotated translations follow the right-hand rule
//! - Missing frames produce clear errors (not panics)

use horus_core::error::HorusError;
use horus_library::transform_frame::{Transform, TransformFrame};

const EPSILON: f64 = 1e-10;

/// Assert two 3D points are equal within tolerance, with a descriptive message.
fn assert_point_eq(actual: [f64; 3], expected: [f64; 3], msg: &str) {
    let dx = (actual[0] - expected[0]).abs();
    let dy = (actual[1] - expected[1]).abs();
    let dz = (actual[2] - expected[2]).abs();
    assert!(
        dx < EPSILON && dy < EPSILON && dz < EPSILON,
        "{msg}: expected [{:.10}, {:.10}, {:.10}], got [{:.10}, {:.10}, {:.10}]",
        expected[0],
        expected[1],
        expected[2],
        actual[0],
        actual[1],
        actual[2],
    );
}

/// Assert two quaternions represent the same rotation (accounting for q == -q).
fn assert_rotation_eq(actual: [f64; 4], expected: [f64; 4], msg: &str) {
    let dq: f64 = actual
        .iter()
        .zip(expected.iter())
        .map(|(a, e)| (a - e).abs())
        .sum();
    let dq_neg: f64 = actual
        .iter()
        .zip(expected.iter())
        .map(|(a, e)| (a + e).abs())
        .sum();
    assert!(
        dq < EPSILON * 4.0 || dq_neg < EPSILON * 4.0,
        "{msg}: expected [{:.10}, {:.10}, {:.10}, {:.10}], got [{:.10}, {:.10}, {:.10}, {:.10}]",
        expected[0],
        expected[1],
        expected[2],
        expected[3],
        actual[0],
        actual[1],
        actual[2],
        actual[3],
    );
}

// ==========================================================================
// 1. Transform chain resolution is geometrically correct
// ==========================================================================
//
// INTENT: "Chaining transforms produces geometrically correct results."
//
// Setup:
//   world --[+1,0,0]--> base --[0,+2,0]--> sensor
//
// Hand-computed:
//   T(world→base)  = translate(1, 0, 0), identity rotation
//   T(base→sensor) = translate(0, 2, 0), identity rotation
//
//   T(world→sensor) = T(world→base) ∘ T(base→sensor)
//     = translate(1, 0, 0) + R_identity * translate(0, 2, 0)
//     = translate(1, 2, 0), identity rotation
//
// We look up tf("sensor", "world") which returns the transform mapping
// points from sensor frame into world frame. That is the composed chain.

#[test]
fn test_transform_chain_resolution_is_correct() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("base", Some("world")).unwrap();
    tf.register_frame("sensor", Some("base")).unwrap();

    let t = 1_000_000;
    tf.update_transform("base", &Transform::from_translation([1.0, 0.0, 0.0]), t)
        .unwrap();
    tf.update_transform("sensor", &Transform::from_translation([0.0, 2.0, 0.0]), t)
        .unwrap();

    let result = tf.tf("sensor", "world").unwrap();

    // The composed translation must be (1, 2, 0) — not just "some value"
    assert_point_eq(
        result.translation,
        [1.0, 2.0, 0.0],
        "chained translation world→base→sensor",
    );

    // Rotation must be identity (no rotation in the chain)
    assert_rotation_eq(
        result.rotation,
        [0.0, 0.0, 0.0, 1.0],
        "chained rotation should be identity",
    );
}

// ==========================================================================
// 2. Inverse transform is geometrically correct
// ==========================================================================
//
// INTENT: "Looking up child→parent returns the inverse of parent→child."
//
// Setup:
//   world --[+3,0,0]--> base
//
// Hand-computed:
//   T(world→base) = translate(3, 0, 0), identity rotation
//   T(base→world) = inverse = translate(-3, 0, 0), identity rotation
//
// tf("world", "base") gives T mapping world-frame points into base frame.
// A point at the world origin (0,0,0) is at (-3,0,0) in base frame.

#[test]
fn test_inverse_transform_is_correct() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("base", Some("world")).unwrap();

    let t = 1_000_000;
    tf.update_transform("base", &Transform::from_translation([3.0, 0.0, 0.0]), t)
        .unwrap();

    // Forward: sensor→world direction
    let forward = tf.tf("base", "world").unwrap();
    assert_point_eq(
        forward.translation,
        [3.0, 0.0, 0.0],
        "forward base→world translation",
    );

    // Inverse: world→base direction (should negate the translation)
    let inverse = tf.tf("world", "base").unwrap();
    assert_point_eq(
        inverse.translation,
        [-3.0, 0.0, 0.0],
        "inverse world→base translation",
    );

    // Verify the inverse composes back to identity
    let roundtrip = forward.compose(&inverse);
    assert_point_eq(
        roundtrip.translation,
        [0.0, 0.0, 0.0],
        "forward ∘ inverse should be identity translation",
    );
    assert_rotation_eq(
        roundtrip.rotation,
        [0.0, 0.0, 0.0, 1.0],
        "forward ∘ inverse should be identity rotation",
    );
}

// ==========================================================================
// 3. Same-frame lookup returns identity
// ==========================================================================
//
// INTENT: "Looking up a frame to itself returns identity."
//
// This is a fundamental invariant: T(A→A) = I for any frame A.
// If this fails, every downstream computation is wrong.

#[test]
fn test_identity_transform_for_same_frame() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("base", Some("world")).unwrap();

    let t = 1_000_000;
    tf.update_transform("base", &Transform::from_translation([5.0, 3.0, 1.0]), t)
        .unwrap();

    // world→world must be identity regardless of what transforms exist
    let self_tf = tf.tf("world", "world").unwrap();
    assert_point_eq(
        self_tf.translation,
        [0.0, 0.0, 0.0],
        "world→world translation must be zero",
    );
    assert_rotation_eq(
        self_tf.rotation,
        [0.0, 0.0, 0.0, 1.0],
        "world→world rotation must be identity",
    );

    // Also check a non-root frame: base→base
    let base_self = tf.tf("base", "base").unwrap();
    assert_point_eq(
        base_self.translation,
        [0.0, 0.0, 0.0],
        "base→base translation must be zero",
    );
    assert_rotation_eq(
        base_self.rotation,
        [0.0, 0.0, 0.0, 1.0],
        "base→base rotation must be identity",
    );
}

// ==========================================================================
// 4. Static frames persist across dynamic updates
// ==========================================================================
//
// INTENT: "Static frames are write-once and survive dynamic updates."
//
// A static frame (e.g., a rigidly mounted sensor) must never change its
// transform even when dynamic frames in the same tree are updated repeatedly.
// If static frames silently drift, SLAM and calibration break.

#[test]
fn test_static_frames_persist_across_updates() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();

    // Register a static frame with a known transform
    let static_transform = Transform::from_translation([0.5, 0.0, 0.3]);
    tf.register_static_frame("camera", Some("world"), &static_transform)
        .unwrap();

    // Register a dynamic sibling
    tf.register_frame("base", Some("world")).unwrap();

    // Verify the static frame's initial value
    let initial = tf.tf("camera", "world").unwrap();
    assert_point_eq(
        initial.translation,
        [0.5, 0.0, 0.3],
        "static frame initial value",
    );

    // Hammer the dynamic frame with many updates
    for i in 0..100 {
        let t = (i + 1) * 1_000_000;
        let dynamic_tf = Transform::from_translation([i as f64, 0.0, 0.0]);
        tf.update_transform("base", &dynamic_tf, t).unwrap();
    }

    // The static frame MUST still have the original value
    let after_updates = tf.tf("camera", "world").unwrap();
    assert_point_eq(
        after_updates.translation,
        [0.5, 0.0, 0.3],
        "static frame after 100 dynamic updates",
    );
    assert_rotation_eq(
        after_updates.rotation,
        [0.0, 0.0, 0.0, 1.0],
        "static frame rotation after dynamic updates",
    );
}

// ==========================================================================
// 5. Rotation composition follows the right-hand rule
// ==========================================================================
//
// INTENT: "Rotation composition follows right-hand rule."
//
// Setup:
//   world --[yaw 90°]--> base --[translate +1,0,0 in base]--> sensor
//
// Hand-computed:
//   T(world→base) = (translation=[0,0,0], rotation=yaw(π/2))
//   T(base→sensor) = (translation=[1,0,0], rotation=identity)
//
//   T(world→sensor) = T(world→base) ∘ T(base→sensor)
//     rotation = yaw(π/2) * I = yaw(π/2)
//     translation = R_z(π/2) * [1,0,0] + [0,0,0]
//                 = [0,1,0]
//
//   R_z(π/2) rotates the +X axis to +Y:
//     R_z(π/2) = [[0,-1,0],[1,0,0],[0,0,1]]
//     R_z(π/2) * [1,0,0] = [0,1,0]
//
//   So the sensor, which is at [1,0,0] in the base frame, appears at
//   [0,1,0] in the world frame. This is the right-hand rule: a 90° yaw
//   (counterclockwise around Z when viewed from above) maps +X to +Y.

#[test]
fn test_transform_with_rotation_composes_correctly() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("base", Some("world")).unwrap();
    tf.register_frame("sensor", Some("base")).unwrap();

    let t = 1_000_000;

    // world→base: 90° yaw, no translation
    tf.update_transform("base", &Transform::yaw(std::f64::consts::FRAC_PI_2), t)
        .unwrap();

    // base→sensor: 1m along base's X axis, no rotation
    tf.update_transform(
        "sensor",
        &Transform::from_translation([1.0, 0.0, 0.0]),
        t,
    )
    .unwrap();

    let result = tf.tf("sensor", "world").unwrap();

    // The sensor at [1,0,0] in base frame should be at [0,1,0] in world frame
    assert_point_eq(
        result.translation,
        [0.0, 1.0, 0.0],
        "90° yaw rotates +X to +Y (right-hand rule)",
    );

    // Verify by transforming the origin of the sensor frame into world
    let origin_in_world = tf.transform_point("sensor", "world", [0.0, 0.0, 0.0]).unwrap();
    assert_point_eq(
        origin_in_world,
        [0.0, 1.0, 0.0],
        "sensor origin in world frame",
    );

    // Verify a point 1m ahead of sensor (+X in sensor frame) ends up
    // at [0,2,0] in world: the sensor's +X is world's +Y after 90° yaw
    let ahead_in_world = tf.transform_point("sensor", "world", [1.0, 0.0, 0.0]).unwrap();
    assert_point_eq(
        ahead_in_world,
        [0.0, 2.0, 0.0],
        "1m ahead of sensor in world frame",
    );
}

// ==========================================================================
// 6. Nonexistent frame returns a clear error
// ==========================================================================
//
// INTENT: "Looking up a frame that doesn't exist returns a clear error."
//
// A robotics system must never panic on a missing frame — the sensor driver
// might not have started yet, or a frame name might be misspelled. The
// error must be recoverable and identify which frame is missing.

#[test]
fn test_nonexistent_frame_returns_error() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();

    // Source frame doesn't exist
    let result = tf.tf("doesnt_exist", "world");
    assert!(result.is_err(), "lookup of nonexistent source must fail");
    match &result.unwrap_err() {
        HorusError::NotFound(_) => {} // correct error variant
        other => panic!(
            "expected NotFound for missing source, got: {:?}",
            other
        ),
    }

    // Destination frame doesn't exist
    let result = tf.tf("world", "doesnt_exist");
    assert!(
        result.is_err(),
        "lookup of nonexistent destination must fail"
    );
    match &result.unwrap_err() {
        HorusError::NotFound(_) => {}
        other => panic!(
            "expected NotFound for missing destination, got: {:?}",
            other
        ),
    }

    // Both frames don't exist
    let result = tf.tf("ghost_a", "ghost_b");
    assert!(result.is_err(), "lookup of two nonexistent frames must fail");
    match &result.unwrap_err() {
        HorusError::NotFound(_) => {}
        other => panic!(
            "expected NotFound when both frames missing, got: {:?}",
            other
        ),
    }

    // Also verify can_transform returns false (not panic)
    assert!(
        !tf.can_transform("doesnt_exist", "world"),
        "can_transform must return false for missing frame",
    );

    // transform_point must also return Err, not panic
    let point_result = tf.transform_point("doesnt_exist", "world", [1.0, 0.0, 0.0]);
    assert!(
        point_result.is_err(),
        "transform_point must return Err for missing frame",
    );
}
