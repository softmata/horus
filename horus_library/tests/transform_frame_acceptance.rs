//! TF2 Behavioral Equivalence Acceptance Tests
//!
//! These tests validate that TransformFrame produces the **same results** as ROS2 TF2
//! for common robot scenarios. All expected values are **hand-computed** using
//! matrix math — never derived by running TransformFrame and recording output.
//!
//! ## Convention Mapping
//!
//! | TransformFrame                          | TF2                                     |
//! |---------------------------------|-----------------------------------------|
//! | `tf("src", "dst")`              | `lookupTransform("dst", "src")`         |
//! | `update_transform("child", tf)` | `TransformBroadcaster.sendTransform()`  |
//! | Quaternion: `[x, y, z, w]`      | Same: `[x, y, z, w]`                   |
//! | `transform_point(src, dst, p)`  | Apply lookupTransform result to point   |
//!
//! **Argument order is reversed**: TransformFrame uses (source, destination),
//! TF2 uses (target/destination, source). The transform semantics are identical:
//! both return T such that `T * p_source = p_destination`.

use horus_core::error::HorusError;
use horus_library::transform_frame::{TransformFrame, Transform};

const EPSILON: f64 = 1e-10;
const FRAC_1_SQRT_2: f64 = std::f64::consts::FRAC_1_SQRT_2;
const PI: f64 = std::f64::consts::PI;

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

fn assert_transform_eq(actual: &Transform, expected: &Transform, msg: &str) {
    assert_point_eq(
        actual.translation,
        expected.translation,
        &format!("{msg} (translation)"),
    );
    let dq: f64 = actual
        .rotation
        .iter()
        .zip(expected.rotation.iter())
        .map(|(a, e)| (a - e).abs())
        .sum();
    // Quaternions q and -q represent the same rotation — check both signs
    let dq_neg: f64 = actual
        .rotation
        .iter()
        .zip(expected.rotation.iter())
        .map(|(a, e)| (a + e).abs())
        .sum();
    assert!(
        dq < EPSILON * 4.0 || dq_neg < EPSILON * 4.0,
        "{msg} (rotation): expected [{:.10}, {:.10}, {:.10}, {:.10}], got [{:.10}, {:.10}, {:.10}, {:.10}]",
        expected.rotation[0], expected.rotation[1], expected.rotation[2], expected.rotation[3],
        actual.rotation[0], actual.rotation[1], actual.rotation[2], actual.rotation[3],
    );
}

// ==========================================================================
// A. Mobile Robot Navigation (TurtleBot-like)
// ==========================================================================
//
// Tree: map → odom → base_link → laser
//
// Stored transforms (child_frame → parent_frame):
//   odom:      translate [1.0, 0, 0], identity rotation
//              (robot odometry: 1m forward along map's x-axis)
//   base_link: translate [0, 0, 0], yaw(π/2)
//              (robot turned left 90°)
//   laser:     translate [0.2, 0, 0], identity rotation
//              (laser 20cm ahead of base in base frame)
//
// Hand-computed tf("laser", "map"):
//   laser→base_link: T_laser = ([0.2, 0, 0], I)
//   base_link→odom:  T_base  = ([0, 0, 0], yaw(π/2))
//   odom→map:        T_odom  = ([1, 0, 0], I)
//
//   Step 1: T_base ∘ T_laser
//     R_z(π/2) = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
//     R_z(π/2) * [0.2, 0, 0] = [0, 0.2, 0]
//     translation = [0, 0.2, 0] + [0, 0, 0] = [0, 0.2, 0]
//     rotation = R_z(π/2)
//
//   Step 2: T_odom ∘ (T_base ∘ T_laser)
//     I * [0, 0.2, 0] + [1, 0, 0] = [1.0, 0.2, 0]
//     rotation = R_z(π/2)
//
// Result: tf("laser", "map") = { t: [1.0, 0.2, 0.0], r: yaw(π/2) }

fn setup_mobile_robot() -> TransformFrame {
    let tf = TransformFrame::new();
    tf.register_frame("map", None).unwrap();
    tf.register_frame("odom", Some("map")).unwrap();
    tf.register_frame("base_link", Some("odom")).unwrap();
    tf.register_frame("laser", Some("base_link")).unwrap();

    let t = 1_000_000;
    tf.update_transform("odom", &Transform::from_translation([1.0, 0.0, 0.0]), t)
        .unwrap();
    tf.update_transform("base_link", &Transform::yaw(PI / 2.0), t)
        .unwrap();
    tf.update_transform("laser", &Transform::from_translation([0.2, 0.0, 0.0]), t)
        .unwrap();
    tf
}

#[test]
fn accept_mobile_robot_laser_to_map_transform() {
    let tf = setup_mobile_robot();
    let tf = tf.tf("laser", "map").unwrap();

    // Hand-computed: translation [1.0, 0.2, 0.0], yaw(π/2)
    assert_point_eq(tf.translation, [1.0, 0.2, 0.0], "laser→map translation");

    // yaw(π/2) quaternion = [0, 0, sin(π/4), cos(π/4)] = [0, 0, √2/2, √2/2]
    let expected_quat = [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2];
    assert_transform_eq(
        &tf,
        &Transform::new([1.0, 0.2, 0.0], expected_quat),
        "laser→map",
    );
}

#[test]
fn accept_mobile_robot_point_transform() {
    let tf = setup_mobile_robot();

    // Point [1.0, 0.0, 0.0] in laser frame (1m ahead of laser)
    // tf("laser","map") = { t: [1.0, 0.2, 0], r: yaw(π/2) }
    // R_z(π/2) * [1, 0, 0] = [0, 1, 0]
    // result = [0, 1, 0] + [1.0, 0.2, 0] = [1.0, 1.2, 0.0]
    let result = tf.transform_point("laser", "map", [1.0, 0.0, 0.0]).unwrap();
    assert_point_eq(result, [1.0, 1.2, 0.0], "laser point [1,0,0] in map");
}

#[test]
fn accept_mobile_robot_vector_transform() {
    let tf = setup_mobile_robot();

    // Vector [1.0, 0.0, 0.0] in laser frame (forward direction)
    // Vectors only get rotation, not translation.
    // R_z(π/2) * [1, 0, 0] = [0, 1, 0]
    let result = tf
        .transform_vector("laser", "map", [1.0, 0.0, 0.0])
        .unwrap();
    assert_point_eq(result, [0.0, 1.0, 0.0], "laser forward vector in map");
}

#[test]
fn accept_mobile_robot_reverse_direction() {
    let tf = setup_mobile_robot();

    // tf("map", "laser") should be inverse of tf("laser", "map")
    let tf_fwd = tf.tf("laser", "map").unwrap();
    let tf_rev = tf.tf("map", "laser").unwrap();

    assert_transform_eq(
        &tf_rev,
        &tf_fwd.inverse(),
        "map→laser should equal inverse of laser→map",
    );
}

// ==========================================================================
// B. Multi-Sensor Fusion (Camera Extrinsics)
// ==========================================================================
//
// Tree:     base_link
//           /       \
//    camera_rgb  camera_depth
//
// camera_rgb:   translate [0.1, 0.05, 0.3], identity
// camera_depth: translate [0.1, -0.05, 0.3], identity
//
// tf("camera_rgb", "camera_depth"):
//   Chain: camera_rgb → base_link → camera_depth
//   UP:   T_rgb = ([0.1, 0.05, 0.3], I)
//   DOWN: T_depth⁻¹ = ([-0.1, 0.05, -0.3], I)
//   Compose: I*[-0.1,0.05,-0.3] + [0.1,0.05,0.3] = [0.0, 0.1, 0.0]
//
// The cameras are 10cm apart along the Y axis.

fn setup_multi_sensor() -> TransformFrame {
    let tf = TransformFrame::new();
    tf.register_frame("base_link", None).unwrap();
    tf.register_frame("camera_rgb", Some("base_link")).unwrap();
    tf.register_frame("camera_depth", Some("base_link"))
        .unwrap();

    let t = 1_000_000;
    let tf_rgb = Transform::from_translation([0.1, 0.05, 0.3]);
    let tf_depth = Transform::from_translation([0.1, -0.05, 0.3]);
    tf.update_transform("camera_rgb", &tf_rgb, t).unwrap();
    tf.update_transform("camera_depth", &tf_depth, t).unwrap();
    tf
}

#[test]
fn accept_multi_sensor_rgb_to_depth() {
    let tf = setup_multi_sensor();
    let tf = tf.tf("camera_rgb", "camera_depth").unwrap();

    // Hand-computed: cameras 10cm apart along Y
    assert_point_eq(tf.translation, [0.0, 0.1, 0.0], "rgb→depth translation");
}

#[test]
fn accept_multi_sensor_point_projection() {
    let tf = setup_multi_sensor();

    // Point detected at [1.5, 0.3, 2.0] in camera_rgb
    // Same point in camera_depth:
    //   tf("camera_rgb", "camera_depth") * [1.5, 0.3, 2.0]
    //   = I * [1.5, 0.3, 2.0] + [0.0, 0.1, 0.0]
    //   = [1.5, 0.4, 2.0]
    let result = tf
        .transform_point("camera_rgb", "camera_depth", [1.5, 0.3, 2.0])
        .unwrap();
    assert_point_eq(result, [1.5, 0.4, 2.0], "RGB point in depth frame");
}

// ==========================================================================
// C. Robot Arm (2-Link Planar)
// ==========================================================================
//
// Tree: world → shoulder → elbow → gripper
//
// shoulder: translate [0, 0, 1.0], pitch(π/2)
//   (shoulder 1m up, arm points along +x in world)
//   R_y(π/2) = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
//
// elbow: translate [0, 0, 1.0], pitch(-π/2)
//   (1m along shoulder's z = world's +x, then bend back down)
//   R_y(-π/2) = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]
//
// gripper: translate [0, 0, 0.5], identity
//   (0.5m along elbow's z)
//
// tf("gripper", "world"):
//   Step 1: T_elbow ∘ T_gripper
//     R_y(-π/2) * [0, 0, 0.5] = [-0.5, 0, 0]... wait:
//     R_y(-π/2) * [0, 0, 0.5]:
//       x' = 0*0 + 0*0 + (-1)*0.5 = -0.5
//       y' = 0*0 + 1*0 + 0*0.5 = 0
//       z' = 1*0 + 0*0 + 0*0.5 = 0
//     translation = [-0.5, 0, 0] + [0, 0, 1.0] = [-0.5, 0, 1.0]
//     rotation = R_y(-π/2) * I = R_y(-π/2)
//
//   Step 2: T_shoulder ∘ (T_elbow ∘ T_gripper)
//     R_y(π/2) * [-0.5, 0, 1.0]:
//       x' = 0*(-0.5) + 0*0 + 1*1.0 = 1.0
//       y' = 0*(-0.5) + 1*0 + 0*1.0 = 0
//       z' = (-1)*(-0.5) + 0*0 + 0*1.0 = 0.5
//     translation = [1.0, 0, 0.5] + [0, 0, 1.0] = [1.0, 0, 1.5]
//     rotation = R_y(π/2) * R_y(-π/2) = R_y(0) = I
//
// Result: tf("gripper", "world") = { t: [1.0, 0, 1.5], r: I }
//
// Physical interpretation: shoulder at height 1m, arm extends 1m horizontally,
// elbow bends up, gripper ends at [1.0, 0, 1.5] with identity orientation
// because the two 90° rotations cancel.

fn setup_arm() -> TransformFrame {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("shoulder", Some("world")).unwrap();
    tf.register_frame("elbow", Some("shoulder")).unwrap();
    tf.register_frame("gripper", Some("elbow")).unwrap();

    let t = 1_000_000;
    // shoulder: 1m up, pitch 90°
    tf.update_transform(
        "shoulder",
        &Transform::from_euler([0.0, 0.0, 1.0], [0.0, PI / 2.0, 0.0]),
        t,
    )
    .unwrap();
    // elbow: 1m along shoulder's z-axis, pitch -90°
    tf.update_transform(
        "elbow",
        &Transform::from_euler([0.0, 0.0, 1.0], [0.0, -PI / 2.0, 0.0]),
        t,
    )
    .unwrap();
    // gripper: 0.5m along elbow's z-axis
    tf.update_transform("gripper", &Transform::from_translation([0.0, 0.0, 0.5]), t)
        .unwrap();
    tf
}

#[test]
fn accept_arm_forward_kinematics() {
    let tf = setup_arm();
    let tf = tf.tf("gripper", "world").unwrap();

    // Hand-computed: [1.0, 0.0, 1.5], identity rotation
    assert_point_eq(
        tf.translation,
        [1.0, 0.0, 1.5],
        "gripper→world FK translation",
    );

    // The two pitch rotations cancel → identity
    let identity = Transform::identity();
    assert_transform_eq(
        &tf,
        &Transform::new([1.0, 0.0, 1.5], identity.rotation),
        "gripper→world FK",
    );
}

#[test]
fn accept_arm_tool_tip_in_world() {
    let tf = setup_arm();

    // Gripper tip at [0, 0, 0.1] in gripper frame (10cm beyond gripper origin)
    // tf("gripper", "world") has identity rotation + translation [1.0, 0, 1.5]
    // result = I * [0, 0, 0.1] + [1.0, 0, 1.5] = [1.0, 0.0, 1.6]
    let result = tf
        .transform_point("gripper", "world", [0.0, 0.0, 0.1])
        .unwrap();
    assert_point_eq(result, [1.0, 0.0, 1.6], "gripper tip in world");
}

// ==========================================================================
// D. TF2 Convention Mapping
// ==========================================================================

#[test]
fn accept_convention_bidirectional_inverse() {
    // tf("A", "B") must equal tf("B", "A").inverse()
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("sensor", Some("world")).unwrap();

    let tf_sensor = Transform::from_euler([1.0, 2.0, 3.0], [0.3, 0.5, 0.7]);
    tf.update_transform("sensor", &tf_sensor, 1_000_000)
        .unwrap();

    let tf_fwd = tf.tf("sensor", "world").unwrap();
    let tf_rev = tf.tf("world", "sensor").unwrap();

    assert_transform_eq(&tf_fwd, &tf_rev.inverse(), "tf(A,B) == tf(B,A).inverse()");
}

#[test]
fn accept_convention_self_transform_is_identity() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("sensor", Some("world")).unwrap();
    tf.update_transform(
        "sensor",
        &Transform::from_euler([5.0, 3.0, 1.0], [0.1, 0.2, 0.3]),
        1_000_000,
    )
    .unwrap();

    let tf_self = tf.tf("sensor", "sensor").unwrap();
    assert_transform_eq(&tf_self, &Transform::identity(), "tf(A,A) == identity");
}

#[test]
fn accept_convention_transitive_composition() {
    // tf("A", "C") == tf("B", "C") ∘ tf("A", "B")
    //
    // For any point p in frame A:
    //   tf("A","C").transform_point(p)
    //     == tf("B","C").transform_point(tf("A","B").transform_point(p))
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("body", Some("world")).unwrap();
    tf.register_frame("sensor", Some("body")).unwrap();

    let t = 1_000_000;
    tf.update_transform(
        "body",
        &Transform::from_euler([2.0, 0.0, 0.0], [0.0, 0.0, PI / 6.0]),
        t,
    )
    .unwrap();
    tf.update_transform("sensor", &Transform::from_translation([0.5, 0.1, 0.0]), t)
        .unwrap();

    let tf_ac = tf.tf("sensor", "world").unwrap();
    let tf_ab = tf.tf("sensor", "body").unwrap();
    let tf_bc = tf.tf("body", "world").unwrap();

    let composed = tf_bc.compose(&tf_ab);
    assert_transform_eq(&tf_ac, &composed, "transitive: tf(A,C) == tf(B,C)∘tf(A,B)");

    // Also verify with a concrete point
    let p = [1.0, -0.5, 0.3];
    let direct = tf_ac.transform_point(p);
    let stepped = tf_bc.transform_point(tf_ab.transform_point(p));
    assert_point_eq(direct, stepped, "transitive point transform");
}

#[test]
fn accept_convention_point_round_trip() {
    // Transforming a point A→B then B→A must return the original
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("robot", Some("world")).unwrap();

    tf.update_transform(
        "robot",
        &Transform::from_euler([3.0, -1.0, 0.5], [0.2, -0.3, 1.1]),
        1_000_000,
    )
    .unwrap();

    let original = [7.5, -2.3, 4.1];
    let in_world = tf.transform_point("robot", "world", original).unwrap();
    let back = tf.transform_point("world", "robot", in_world).unwrap();
    assert_point_eq(back, original, "point round-trip robot→world→robot");
}

// ==========================================================================
// E. Time-Travel Query (Interpolation)
// ==========================================================================
//
// Robot moves linearly from [0,0,0] at t=0 to [2.0, 0, 0] at t=1000ns.
// Query at t=500 should interpolate to [1.0, 0, 0].

#[test]
fn accept_time_interpolation_midpoint() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("robot", Some("world")).unwrap();

    tf.update_transform("robot", &Transform::from_translation([0.0, 0.0, 0.0]), 0)
        .unwrap();
    tf.update_transform("robot", &Transform::from_translation([2.0, 0.0, 0.0]), 1000)
        .unwrap();

    let tf = tf.tf_at("robot", "world", 500).unwrap();
    assert_point_eq(tf.translation, [1.0, 0.0, 0.0], "midpoint interpolation");
}

#[test]
fn accept_time_interpolation_quarter() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("robot", Some("world")).unwrap();

    tf.update_transform("robot", &Transform::from_translation([0.0, 0.0, 0.0]), 0)
        .unwrap();
    tf.update_transform("robot", &Transform::from_translation([4.0, 0.0, 0.0]), 1000)
        .unwrap();

    let tf = tf.tf_at("robot", "world", 250).unwrap();
    assert_point_eq(tf.translation, [1.0, 0.0, 0.0], "quarter interpolation");
}

#[test]
fn accept_time_interpolation_with_rotation() {
    // Interpolating rotation uses SLERP.
    // From yaw=0 to yaw=π/2, at t=0.5 → yaw=π/4
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("robot", Some("world")).unwrap();

    tf.update_transform("robot", &Transform::yaw(0.0), 0)
        .unwrap();
    tf.update_transform("robot", &Transform::yaw(PI / 2.0), 1000)
        .unwrap();

    let tf = tf.tf_at("robot", "world", 500).unwrap();

    // Expected: yaw(π/4) quaternion = [0, 0, sin(π/8), cos(π/8)]
    let expected = Transform::yaw(PI / 4.0);
    assert_transform_eq(&tf, &expected, "SLERP rotation at midpoint");
}

// ==========================================================================
// F. Static Transform Equivalence
// ==========================================================================

#[test]
fn accept_static_available_at_any_time() {
    // Static transforms (like camera calibration) must be queryable
    // at any timestamp — past, present, or future.
    // This matches tf2_ros::StaticTransformBroadcaster behavior.
    let tf = TransformFrame::new();
    tf.register_frame("base_link", None).unwrap();
    tf.register_static_frame(
        "camera",
        Some("base_link"),
        &Transform::from_translation([0.1, 0.0, 0.5]),
    )
    .unwrap();

    // Update base_link at t=1000 so the tree has data
    tf.update_transform("base_link", &Transform::identity(), 1000)
        .unwrap();

    // Query at various times — all should succeed
    for ts in [0, 500, 1000, 2000, 1_000_000_000] {
        let tf = tf.tf_at("camera", "base_link", ts);
        assert!(tf.is_ok(), "Static transform should be available at t={ts}");
        assert_point_eq(
            tf.unwrap().translation,
            [0.1, 0.0, 0.5],
            &format!("static tf at t={ts}"),
        );
    }
}

#[test]
fn accept_static_strict_no_extrapolation() {
    // Static frames should never trigger extrapolation errors even with strict mode.
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_static_frame(
        "fixed_sensor",
        Some("world"),
        &Transform::from_translation([1.0, 0.0, 0.0]),
    )
    .unwrap();

    // tf_at_strict should succeed at any time for pure static chains
    let tf = tf.tf_at_strict("fixed_sensor", "world", 999_999_999);
    assert!(tf.is_ok(), "Static-only chain should not extrapolate");
}

// ==========================================================================
// G. Branching Tree Traversal
// ==========================================================================
//
//          map
//         /   \
//      odom    world_fixed
//       |
//   base_link
//    /       \
//  left     right
//
// tf("left", "right") goes through base_link (sibling traversal).
// tf("left", "world_fixed") goes through map (cross-branch traversal).

fn setup_branching_tree() -> TransformFrame {
    let tf = TransformFrame::new();
    tf.register_frame("map", None).unwrap();
    tf.register_frame("odom", Some("map")).unwrap();
    tf.register_frame("world_fixed", Some("map")).unwrap();
    tf.register_frame("base_link", Some("odom")).unwrap();
    tf.register_frame("left_wheel", Some("base_link")).unwrap();
    tf.register_frame("right_wheel", Some("base_link")).unwrap();

    let t = 1_000_000;
    tf.update_transform("odom", &Transform::from_translation([1.0, 0.0, 0.0]), t)
        .unwrap();
    tf.update_transform(
        "world_fixed",
        &Transform::from_translation([0.0, 5.0, 0.0]),
        t,
    )
    .unwrap();
    tf.update_transform("base_link", &Transform::identity(), t)
        .unwrap();
    tf.update_transform(
        "left_wheel",
        &Transform::from_translation([0.0, 0.15, 0.0]),
        t,
    )
    .unwrap();
    tf.update_transform(
        "right_wheel",
        &Transform::from_translation([0.0, -0.15, 0.0]),
        t,
    )
    .unwrap();
    tf
}

#[test]
fn accept_branching_sibling_traversal() {
    let tf = setup_branching_tree();

    // tf("left_wheel", "right_wheel"):
    //   left_wheel → base_link → right_wheel
    //   UP:   T_left = ([0, 0.15, 0], I)
    //   DOWN: T_right⁻¹ = ([0, 0.15, 0], I)
    //   result = [0, 0.15, 0] + [0, 0.15, 0] = [0, 0.3, 0]
    let tf = tf.tf("left_wheel", "right_wheel").unwrap();
    assert_point_eq(tf.translation, [0.0, 0.3, 0.0], "left→right wheels");
}

#[test]
fn accept_branching_cross_branch_traversal() {
    let tf = setup_branching_tree();

    // tf("left_wheel", "world_fixed"):
    //   left_wheel → base_link → odom → map → world_fixed
    //   UP:   T_left∘T_base∘T_odom = [1.0, 0.15, 0.0]
    //   DOWN: T_world_fixed⁻¹ = [0.0, -5.0, 0.0]
    //   result = [1.0, 0.15, 0] + [0, -5.0, 0] = [1.0, -4.85, 0]
    let tf = tf.tf("left_wheel", "world_fixed").unwrap();
    assert_point_eq(
        tf.translation,
        [1.0, -4.85, 0.0],
        "left_wheel→world_fixed cross-branch",
    );
}

// ==========================================================================
// H. Quaternion Convention Validation
// ==========================================================================

#[test]
fn accept_quaternion_90deg_z_rotation() {
    // 90° rotation around Z should map [1,0,0] → [0,1,0]
    // Quaternion [x,y,z,w] for 90° around Z: [0, 0, sin(π/4), cos(π/4)]
    let tf = Transform::new([0.0, 0.0, 0.0], [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2]);
    let result = tf.transform_point([1.0, 0.0, 0.0]);
    assert_point_eq(result, [0.0, 1.0, 0.0], "90° Z rotation maps x→y");
}

#[test]
fn accept_quaternion_90deg_y_rotation() {
    // 90° rotation around Y should map [1,0,0] → [0,0,-1]
    // Quaternion for 90° around Y: [0, sin(π/4), 0, cos(π/4)]
    let tf = Transform::new([0.0, 0.0, 0.0], [0.0, FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2]);
    let result = tf.transform_point([1.0, 0.0, 0.0]);
    assert_point_eq(result, [0.0, 0.0, -1.0], "90° Y rotation maps x→-z");
}

#[test]
fn accept_quaternion_90deg_x_rotation() {
    // 90° rotation around X should map [0,1,0] → [0,0,1]
    // Quaternion for 90° around X: [sin(π/4), 0, 0, cos(π/4)]
    let tf = Transform::new([0.0, 0.0, 0.0], [FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2]);
    let result = tf.transform_point([0.0, 1.0, 0.0]);
    assert_point_eq(result, [0.0, 0.0, 1.0], "90° X rotation maps y→z");
}

#[test]
fn accept_quaternion_composed_rotation() {
    // Yaw 90° then pitch 90° applied to [1, 0, 0]:
    //   Yaw(90°): [1,0,0] → [0,1,0]
    //   Pitch(90°): [0,1,0] → [0,1,0] (pitch doesn't affect Y)
    //
    // compose(yaw, pitch): first apply pitch (inner), then yaw (outer)
    // So yaw.compose(pitch).transform_point(p) = yaw(pitch(p))
    let yaw90 = Transform::yaw(PI / 2.0);
    let pitch90 = Transform::pitch(PI / 2.0);

    // yaw.compose(pitch) applies pitch first, then yaw
    let combined = yaw90.compose(&pitch90);

    // pitch(90°) * [1,0,0]: R_y(90°) * [1,0,0] = [0,0,-1]
    // yaw(90°) * [0,0,-1]: R_z(90°) * [0,0,-1] = [0,0,-1]
    let result = combined.transform_point([1.0, 0.0, 0.0]);
    assert_point_eq(result, [0.0, 0.0, -1.0], "yaw∘pitch applied to [1,0,0]");
}

// ==========================================================================
// I. Input Validation (Safety-Critical)
// ==========================================================================

#[test]
fn accept_rejects_nan_in_transform() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("bad", Some("world")).unwrap();

    let bad_tf = Transform {
        translation: [f64::NAN, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };
    let result = tf.update_transform("bad", &bad_tf, 1000);
    assert!(matches!(result, Err(HorusError::InvalidInput(_))));
}

#[test]
fn accept_rejects_inf_in_rotation() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("bad", Some("world")).unwrap();

    let bad_tf = Transform {
        translation: [0.0, 0.0, 0.0],
        rotation: [f64::INFINITY, 0.0, 0.0, 1.0],
    };
    let result = tf.update_transform("bad", &bad_tf, 1000);
    assert!(matches!(result, Err(HorusError::InvalidInput(_))));
}

#[test]
fn accept_rejects_zero_quaternion() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("bad", Some("world")).unwrap();

    let bad_tf = Transform {
        translation: [0.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 0.0],
    };
    let result = tf.update_transform("bad", &bad_tf, 1000);
    assert!(matches!(result, Err(HorusError::InvalidInput(_))));
}

// ==========================================================================
// J. Determinism & Precision
// ==========================================================================

#[test]
fn accept_deterministic_repeated_queries() {
    // The same query must return bit-identical results every time
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("robot", Some("world")).unwrap();

    tf.update_transform(
        "robot",
        &Transform::from_euler(
            [1.23456789, -0.98765432, std::f64::consts::PI],
            [0.1, 0.2, 0.3],
        ),
        1_000_000,
    )
    .unwrap();

    let tf1 = tf.tf("robot", "world").unwrap();
    let tf2 = tf.tf("robot", "world").unwrap();

    // Bit-identical, not just epsilon-close
    assert_eq!(
        tf1.translation, tf2.translation,
        "translation must be bit-identical"
    );
    assert_eq!(tf1.rotation, tf2.rotation, "rotation must be bit-identical");
}

#[test]
fn accept_large_coordinate_precision() {
    // GPS-scale coordinates (hundreds of km) should maintain precision
    let tf = TransformFrame::new();
    tf.register_frame("earth", None).unwrap();
    tf.register_frame("utm_zone", Some("earth")).unwrap();

    let large_offset = Transform::from_translation([500_000.0, 4_500_000.0, 100.0]);
    tf.update_transform("utm_zone", &large_offset, 1_000_000)
        .unwrap();

    let tf = tf.tf("utm_zone", "earth").unwrap();
    assert_point_eq(
        tf.translation,
        [500_000.0, 4_500_000.0, 100.0],
        "GPS-scale coordinates",
    );

    // Small offset within large coordinates
    let p = tf.transform_point([0.001, 0.001, 0.0]);
    assert_point_eq(
        p,
        [500_000.001, 4_500_000.001, 100.0],
        "mm-precision at GPS scale",
    );
}

// ==========================================================================
// K. Complex Scenario: Mobile Manipulator
// ==========================================================================
//
// A mobile robot with a 2-link arm mounted on top:
//
//   map → odom → base_link → arm_base → link1 → link2 → tool
//                    |
//                  lidar
//
// This tests the full pipeline: odometry + arm FK + sensor offsets.

#[test]
fn accept_mobile_manipulator_tool_in_map() {
    let tf = TransformFrame::new();
    tf.register_frame("map", None).unwrap();
    tf.register_frame("odom", Some("map")).unwrap();
    tf.register_frame("base_link", Some("odom")).unwrap();
    tf.register_frame("arm_base", Some("base_link")).unwrap();
    tf.register_frame("link1", Some("arm_base")).unwrap();
    tf.register_frame("link2", Some("link1")).unwrap();
    tf.register_frame("tool", Some("link2")).unwrap();
    tf.register_frame("lidar", Some("base_link")).unwrap();

    let t = 1_000_000;

    // odom: robot at [3.0, 1.0, 0.0] in map, heading yaw=0
    tf.update_transform("odom", &Transform::from_translation([3.0, 1.0, 0.0]), t)
        .unwrap();
    // base_link at odom origin (no slip)
    tf.update_transform("base_link", &Transform::identity(), t)
        .unwrap();
    // arm mounted 0.5m above base
    tf.update_transform("arm_base", &Transform::from_translation([0.0, 0.0, 0.5]), t)
        .unwrap();
    // link1: 0.3m arm, pitch 90° (pointing horizontally forward)
    tf.update_transform(
        "link1",
        &Transform::from_euler([0.0, 0.0, 0.3], [0.0, PI / 2.0, 0.0]),
        t,
    )
    .unwrap();
    // link2: 0.2m along link1's z (= world's x due to pitch90)
    tf.update_transform("link2", &Transform::from_translation([0.0, 0.0, 0.2]), t)
        .unwrap();
    // tool: at end of link2
    tf.update_transform("tool", &Transform::from_translation([0.0, 0.0, 0.1]), t)
        .unwrap();
    // lidar: 0.2m above base, 0.1m forward
    tf.update_transform("lidar", &Transform::from_translation([0.1, 0.0, 0.2]), t)
        .unwrap();

    // Verify tool position in map:
    //   tool→link2: [0,0,0.1], I
    //   link2→link1: [0,0,0.2], I
    //   Compose: [0,0,0.3], I
    //
    //   link1→arm_base: [0,0,0.3], pitch(π/2)
    //   R_y(π/2) * [0,0,0.3] = [0.3, 0, 0]
    //   translation = [0.3, 0, 0] + [0, 0, 0.3] = [0.3, 0, 0.3]
    //   rotation = pitch(π/2)
    //
    //   arm_base→base_link: [0,0,0.5], I
    //   I * [0.3, 0, 0.3] + [0, 0, 0.5] = [0.3, 0, 0.8]
    //   rotation = pitch(π/2)
    //
    //   base_link→odom: [0,0,0], I → still [0.3, 0, 0.8], pitch(π/2)
    //
    //   odom→map: [3,1,0], I
    //   I * [0.3, 0, 0.8] + [3, 1, 0] = [3.3, 1.0, 0.8]
    //   rotation = pitch(π/2)
    let tool_tf = tf.tf("tool", "map").unwrap();
    assert_point_eq(
        tool_tf.translation,
        [3.3, 1.0, 0.8],
        "mobile manipulator tool in map",
    );

    // Verify lidar position in map:
    //   lidar→base_link: [0.1, 0, 0.2], I
    //   base_link→odom: I → [0.1, 0, 0.2]
    //   odom→map: [3,1,0] → [3.1, 1.0, 0.2]
    let lidar_tf = tf.tf("lidar", "map").unwrap();
    assert_point_eq(
        lidar_tf.translation,
        [3.1, 1.0, 0.2],
        "mobile manipulator lidar in map",
    );

    // Cross-branch: tool to lidar
    //   tf("tool","lidar") = tf("lidar","map")⁻¹ ∘ tf("tool","map")
    //   Just verify transitivity with a concrete point
    let p_tool = [0.0, 0.0, 0.0]; // tool origin
    let p_map = tf.transform_point("tool", "map", p_tool).unwrap();
    let p_lidar = tf.transform_point("map", "lidar", p_map).unwrap();
    let p_lidar_direct = tf.transform_point("tool", "lidar", p_tool).unwrap();
    assert_point_eq(p_lidar, p_lidar_direct, "tool→lidar via map == direct");
}
