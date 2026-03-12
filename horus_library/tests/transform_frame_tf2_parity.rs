//! Integration tests for TransformFrame TF2 parity features.
//!
//! These tests mirror common ROS2 TF2 usage patterns to verify TransformFrame
//! provides equivalent functionality.

use horus_core::error::{HorusError, NotFoundError, TransformError};
use horus_library::transform_frame::{TransformFrame, Transform};

// ==========================================================================
// 1. Extrapolation Detection
// ==========================================================================

#[test]
fn tf2_parity_extrapolation_past() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("sensor", Some("world")).unwrap();
    tf.update_transform("sensor", &Transform::xyz(1.0, 0.0, 0.0), 5000)
        .unwrap();

    let result = tf.tf_at_strict("sensor", "world", 1000);
    assert!(matches!(
        result,
        Err(HorusError::Transform(TransformError::Extrapolation { .. }))
    ));
}

#[test]
fn tf2_parity_extrapolation_future() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("sensor", Some("world")).unwrap();
    tf.update_transform("sensor", &Transform::xyz(1.0, 0.0, 0.0), 1000)
        .unwrap();

    let result = tf.tf_at_strict("sensor", "world", 99999);
    assert!(matches!(
        result,
        Err(HorusError::Transform(TransformError::Extrapolation { .. }))
    ));
}

#[test]
fn tf2_parity_interpolation_within_range() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("arm", Some("world")).unwrap();
    tf.update_transform("arm", &Transform::xyz(0.0, 0.0, 0.0), 1000)
        .unwrap();
    tf.update_transform("arm", &Transform::xyz(10.0, 0.0, 0.0), 3000)
        .unwrap();

    let tf = tf.tf_at_strict("arm", "world", 2000).unwrap();
    assert!((tf.translation[0] - 5.0).abs() < 1e-6);
}

#[test]
fn tf2_parity_extrapolation_chain_any_hop() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.register_frame("b", Some("a")).unwrap();

    // "a" has data at 1000-2000, "b" at 1000-5000
    tf.update_transform("a", &Transform::xyz(1.0, 0.0, 0.0), 1000)
        .unwrap();
    tf.update_transform("a", &Transform::xyz(2.0, 0.0, 0.0), 2000)
        .unwrap();
    tf.update_transform("b", &Transform::xyz(0.5, 0.0, 0.0), 1000)
        .unwrap();
    tf.update_transform("b", &Transform::xyz(1.5, 0.0, 0.0), 5000)
        .unwrap();

    // ts=3000 in b's range but outside a's → Extrapolation
    let result = tf.tf_at_strict("b", "world", 3000);
    assert!(matches!(
        result,
        Err(HorusError::Transform(TransformError::Extrapolation { .. }))
    ));
}

#[test]
fn tf2_parity_static_never_extrapolates() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_static_frame("fixed", Some("world"), &Transform::xyz(1.0, 0.0, 0.0))
        .unwrap();

    tf.tf_at_strict("fixed", "world", 0).unwrap();
    tf.tf_at_strict("fixed", "world", u64::MAX).unwrap();
}

// ==========================================================================
// 2. Staleness
// ==========================================================================

#[test]
fn tf2_parity_is_stale_basic() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("imu", Some("world")).unwrap();
    tf.update_transform("imu", &Transform::identity(), 10_000)
        .unwrap();

    // Fresh
    assert!(!tf.is_stale("imu", 5_000, 12_000));
    // Stale
    assert!(tf.is_stale("imu", 1_000, 20_000));
}

#[test]
fn tf2_parity_is_stale_never_updated() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("sensor", Some("world")).unwrap();

    assert!(tf.is_stale("sensor", 1000, 10_000));
}

#[test]
fn tf2_parity_is_stale_static_never_stale() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_static_frame("fixed", Some("world"), &Transform::identity())
        .unwrap();

    assert!(!tf.is_stale("fixed", 0, u64::MAX));
}

#[test]
fn tf2_parity_time_since_last_update() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("cam", Some("world")).unwrap();
    tf.update_transform("cam", &Transform::identity(), 10_000)
        .unwrap();

    let age = tf.time_since_last_update("cam", 15_000).unwrap();
    assert_eq!(age, 5000);
}

#[test]
fn tf2_parity_time_range() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();

    assert!(tf.time_range("a").is_none());

    tf.update_transform("a", &Transform::identity(), 1000)
        .unwrap();
    tf.update_transform("a", &Transform::identity(), 5000)
        .unwrap();

    let (oldest, newest) = tf.time_range("a").unwrap();
    assert_eq!(oldest, 1000);
    assert_eq!(newest, 5000);
}

// ==========================================================================
// 3. Time Tolerance
// ==========================================================================

#[test]
fn tf2_parity_tolerance_within() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.update_transform("a", &Transform::xyz(1.0, 0.0, 0.0), 1000)
        .unwrap();

    // Gap=500, tolerance=1000 → ok
    tf.tf_at_with_tolerance("a", "world", 1500, 1000).unwrap();
}

#[test]
fn tf2_parity_tolerance_exceeded() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.update_transform("a", &Transform::xyz(1.0, 0.0, 0.0), 1000)
        .unwrap();

    // Gap=4000, tolerance=1000 → Extrapolation
    let result = tf.tf_at_with_tolerance("a", "world", 5000, 1000);
    assert!(matches!(
        result,
        Err(HorusError::Transform(TransformError::Extrapolation { .. }))
    ));
}

#[test]
fn tf2_parity_tolerance_max_unlimited() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.update_transform("a", &Transform::xyz(1.0, 0.0, 0.0), 1000)
        .unwrap();

    // u64::MAX tolerance = always succeed (same as tf_at)
    tf.tf_at_with_tolerance("a", "world", 999_999, u64::MAX)
        .unwrap();
}

// ==========================================================================
// 4. can_transform_at
// ==========================================================================

#[test]
fn tf2_parity_can_transform_in_range() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.update_transform("a", &Transform::identity(), 1000)
        .unwrap();
    tf.update_transform("a", &Transform::identity(), 5000)
        .unwrap();

    assert!(tf.can_transform_at("a", "world", 3000));
    assert!(!tf.can_transform_at("a", "world", 99999));
}

#[test]
fn tf2_parity_can_transform_no_path() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("isolated", None).unwrap();
    tf.update_transform("isolated", &Transform::identity(), 1000)
        .unwrap();

    assert!(!tf.can_transform_at("isolated", "world", 1000));
}

#[test]
fn tf2_parity_can_transform_with_tolerance() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.update_transform("a", &Transform::identity(), 1000)
        .unwrap();

    assert!(tf.can_transform_at_with_tolerance("a", "world", 2000, 2000));
    assert!(!tf.can_transform_at_with_tolerance("a", "world", 5000, 2000));
}

// ==========================================================================
// 5. Input Validation
// ==========================================================================

#[test]
fn tf2_parity_rejects_nan_translation() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();

    let bad = Transform {
        translation: [f64::NAN, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };
    assert!(matches!(
        tf.update_transform("a", &bad, 1000),
        Err(HorusError::InvalidInput(_))
    ));
}

#[test]
fn tf2_parity_rejects_inf_rotation() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    let id = tf.register_frame("a", Some("world")).unwrap();

    let bad = Transform {
        translation: [0.0, 0.0, 0.0],
        rotation: [f64::INFINITY, 0.0, 0.0, 1.0],
    };
    assert!(matches!(
        tf.update_transform_by_id(id, &bad, 1000),
        Err(HorusError::InvalidInput(_))
    ));
}

#[test]
fn tf2_parity_rejects_zero_quaternion() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();

    let bad = Transform {
        translation: [0.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 0.0],
    };
    assert!(matches!(
        tf.update_transform("a", &bad, 1000),
        Err(HorusError::InvalidInput(_))
    ));
}

// ==========================================================================
// 6. Tree Visualization
// ==========================================================================

#[test]
fn tf2_parity_frames_as_dot() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("base", Some("world")).unwrap();
    tf.register_static_frame("cam", Some("base"), &Transform::xyz(0.0, 0.0, 0.5))
        .unwrap();

    let dot = tf.frames_as_dot();
    assert!(dot.starts_with("digraph transform_frame {"));
    assert!(dot.contains("world"));
    assert!(dot.contains("base"));
    assert!(dot.contains("cam"));
    assert!(dot.contains("doubleoctagon")); // static frame shape
    assert!(dot.ends_with("}\n"));
}

#[test]
fn tf2_parity_frames_as_yaml() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("arm", Some("world")).unwrap();
    tf.update_transform("arm", &Transform::identity(), 5000)
        .unwrap();

    let yaml = tf.frames_as_yaml();
    assert!(yaml.contains("world:"));
    assert!(yaml.contains("arm:"));
    assert!(yaml.contains("parent: world"));
    assert!(yaml.contains("type: dynamic"));
    assert!(yaml.contains("5000ns"));
}

#[test]
fn tf2_parity_format_tree() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.register_frame("b", Some("a")).unwrap();

    let tree = tf.format_tree();
    assert!(tree.contains("world"));
    assert!(tree.contains("TransformFrame Tree:"));
    assert!(tree.contains("[D]")); // dynamic tag
}

// ==========================================================================
// 7. Chain Diagnostics
// ==========================================================================

#[test]
fn tf2_parity_error_frame_not_registered() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();

    let err = tf.tf("nonexistent", "world").unwrap_err();
    match err {
        HorusError::NotFound(NotFoundError::Frame { ref name }) => {
            assert_eq!(name, "nonexistent");
        }
        other => unreachable!("Expected NotFound, got: {:?}", other),
    }
}

#[test]
fn tf2_parity_error_disconnected_trees() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("map", None).unwrap();
    tf.register_frame("robot", Some("world")).unwrap();
    tf.register_frame("landmark", Some("map")).unwrap();
    tf.update_transform("robot", &Transform::identity(), 1000)
        .unwrap();
    tf.update_transform("landmark", &Transform::identity(), 1000)
        .unwrap();

    let err = tf.tf("robot", "landmark").unwrap_err();
    match err {
        HorusError::Communication(ref e) => {
            let msg = e.to_string();
            assert!(msg.contains("disconnected"));
        }
        other => unreachable!("Expected Communication, got: {:?}", other),
    }
}

// ==========================================================================
// 8. FrameInfo & Stats
// ==========================================================================

#[test]
fn tf2_parity_frame_info_dynamic() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("sensor", Some("world")).unwrap();
    tf.update_transform("sensor", &Transform::identity(), 5000)
        .unwrap();

    let info = tf.frame_info("sensor").unwrap();
    assert_eq!(info.name, "sensor");
    assert_eq!(info.parent, Some("world".to_string()));
    assert!(!info.is_static);
    assert_eq!(info.depth, 1);
}

#[test]
fn tf2_parity_frame_info_static() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_static_frame("fixed", Some("world"), &Transform::identity())
        .unwrap();

    let info = tf.frame_info("fixed").unwrap();
    assert!(info.is_static);
    assert_eq!(info.time_range, None);
}

#[test]
fn tf2_parity_frame_info_all() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.register_frame("b", Some("world")).unwrap();

    assert_eq!(tf.frame_info_all().len(), 3);
}

#[test]
fn tf2_parity_stats_depth_and_roots() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("base", Some("world")).unwrap();
    tf.register_frame("arm", Some("base")).unwrap();
    tf.register_frame("gripper", Some("arm")).unwrap();
    tf.register_frame("map", None).unwrap(); // second root

    let stats = tf.stats();
    assert_eq!(stats.total_frames, 5);
    assert_eq!(stats.root_count, 2);
    assert_eq!(stats.tree_depth, 3);
}

// ==========================================================================
// 9. Query Builder
// ==========================================================================

#[test]
fn tf2_parity_query_builder_lookup() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("cam", Some("world")).unwrap();
    tf.update_transform("cam", &Transform::xyz(1.0, 2.0, 3.0), 1000)
        .unwrap();

    let tf = tf.query("cam").to("world").lookup().unwrap();
    assert!((tf.translation[0] - 1.0).abs() < 1e-10);
    assert!((tf.translation[1] - 2.0).abs() < 1e-10);
    assert!((tf.translation[2] - 3.0).abs() < 1e-10);
}

#[test]
fn tf2_parity_query_builder_point() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("cam", Some("world")).unwrap();
    tf.update_transform("cam", &Transform::xyz(10.0, 0.0, 0.0), 1000)
        .unwrap();

    let pt = tf.query("cam").to("world").point([1.0, 0.0, 0.0]).unwrap();
    assert!((pt[0] - 11.0).abs() < 1e-10);
}

#[test]
fn tf2_parity_query_builder_can_at() {
    let tf = TransformFrame::new();
    tf.register_frame("world", None).unwrap();
    tf.register_frame("a", Some("world")).unwrap();
    tf.update_transform("a", &Transform::identity(), 1000)
        .unwrap();
    tf.update_transform("a", &Transform::identity(), 5000)
        .unwrap();

    assert!(tf.query("a").to("world").can_at(3000));
    assert!(!tf.query("a").to("world").can_at(99999));
}

// ==========================================================================
// 10. Frame Builder
// ==========================================================================

#[test]
fn tf2_parity_frame_builder_dynamic() {
    let tf = TransformFrame::new();
    tf.add_frame("world").build().unwrap();
    tf.add_frame("base").parent("world").build().unwrap();

    assert!(tf.has_frame("world"));
    assert!(tf.has_frame("base"));
    assert_eq!(tf.parent("base"), Some("world".to_string()));
}

#[test]
fn tf2_parity_frame_builder_static() {
    let tf = TransformFrame::new();
    tf.add_frame("world").build().unwrap();
    tf.add_frame("cam")
        .parent("world")
        .static_transform(&Transform::xyz(0.0, 0.0, 0.5))
        .build()
        .unwrap();

    let tf = tf.tf("cam", "world").unwrap();
    assert!((tf.translation[2] - 0.5).abs() < 1e-10);
}

// ==========================================================================
// 11. Short Transform Constructors
// ==========================================================================

#[test]
fn tf2_parity_short_constructors() {
    // xyz equivalence
    let a = Transform::xyz(1.0, 2.0, 3.0);
    let b = Transform::from_translation([1.0, 2.0, 3.0]);
    assert_eq!(a.translation, b.translation);

    // Single-axis
    assert_eq!(Transform::x(5.0).translation, [5.0, 0.0, 0.0]);
    assert_eq!(Transform::y(3.0).translation, [0.0, 3.0, 0.0]);
    assert_eq!(Transform::z(1.5).translation, [0.0, 0.0, 1.5]);
}

#[test]
fn tf2_parity_yaw_rotation() {
    let tf = Transform::yaw(std::f64::consts::FRAC_PI_2);
    let pt = tf.transform_point([1.0, 0.0, 0.0]);
    assert!((pt[0]).abs() < 1e-10);
    assert!((pt[1] - 1.0).abs() < 1e-10);
}

#[test]
fn tf2_parity_with_yaw_composition() {
    let tf = Transform::xyz(1.0, 0.0, 0.0).with_yaw(std::f64::consts::FRAC_PI_2);
    // Translation preserved
    assert!((tf.translation[0] - 1.0).abs() < 1e-10);
    // Rotation applied
    let v = tf.transform_vector([1.0, 0.0, 0.0]);
    assert!((v[0]).abs() < 1e-10);
    assert!((v[1] - 1.0).abs() < 1e-10);
}

// ==========================================================================
// 12. Real Robot Scenario: PR2-like arm chain
// ==========================================================================

#[test]
fn tf2_parity_pr2_arm_chain() {
    let tf = TransformFrame::new();

    // PR2-like chain: world -> base_link -> shoulder -> upper_arm -> forearm -> gripper
    tf.add_frame("world").build().unwrap();
    tf.add_frame("base_link").parent("world").build().unwrap();
    tf.add_frame("shoulder")
        .parent("base_link")
        .build()
        .unwrap();
    tf.add_frame("upper_arm")
        .parent("shoulder")
        .build()
        .unwrap();
    tf.add_frame("forearm").parent("upper_arm").build().unwrap();
    tf.add_frame("gripper").parent("forearm").build().unwrap();

    // Static sensor mount
    tf.add_frame("camera")
        .parent("gripper")
        .static_transform(&Transform::xyz(0.0, 0.0, 0.05))
        .build()
        .unwrap();

    // Set transforms (identity rotation, translation only)
    let ts = 1_000_000_000; // 1 second
    tf.update_transform("base_link", &Transform::xyz(0.0, 0.0, 0.8), ts)
        .unwrap();
    tf.update_transform("shoulder", &Transform::xyz(0.0, 0.19, 0.4), ts)
        .unwrap();
    tf.update_transform("upper_arm", &Transform::xyz(0.4, 0.0, 0.0), ts)
        .unwrap();
    tf.update_transform("forearm", &Transform::xyz(0.321, 0.0, 0.0), ts)
        .unwrap();
    tf.update_transform("gripper", &Transform::xyz(0.18, 0.0, 0.0), ts)
        .unwrap();

    // Query camera → world (depth 6 chain)
    let cam_to_world = tf.query("camera").to("world").lookup().unwrap();
    // Expected: sum of all X translations
    let expected_x = 0.0 + 0.0 + 0.4 + 0.321 + 0.18 + 0.0;
    assert!((cam_to_world.translation[0] - expected_x).abs() < 1e-6);

    // Verify stats
    let stats = tf.stats();
    assert_eq!(stats.total_frames, 7);
    assert_eq!(stats.tree_depth, 6); // world -> base -> shoulder -> upper -> forearm -> gripper -> camera

    // Verify frame info
    let info = tf.frame_info("camera").unwrap();
    assert!(info.is_static);
    assert_eq!(info.depth, 6);
    assert_eq!(info.parent, Some("gripper".to_string()));

    // Tree export shouldn't panic
    let _dot = tf.frames_as_dot();
    let _yaml = tf.frames_as_yaml();
    let _tree = tf.format_tree();
}
