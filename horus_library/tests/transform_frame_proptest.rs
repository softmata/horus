//! Property-based tests for Transform and TransformFrame invariants.
//!
//! Uses proptest to verify mathematical properties hold for random inputs,
//! catching edge cases that hand-picked examples would miss: denormalized
//! floats, near-antipodal quaternions, extreme scales, etc.

use horus_library::transform_frame::{TransformFrame, Transform};
use proptest::prelude::*;

// ========================================================================
// Strategies
// ========================================================================

/// Generate a random f64 in a robotics-relevant range.
fn arb_coord() -> impl Strategy<Value = f64> {
    prop::num::f64::NORMAL
        .prop_filter("finite", |x| x.is_finite())
        .prop_map(|x| x.clamp(-1e6, 1e6))
}

/// Generate a random translation [x, y, z].
fn arb_translation() -> impl Strategy<Value = [f64; 3]> {
    (arb_coord(), arb_coord(), arb_coord()).prop_map(|(x, y, z)| [x, y, z])
}

/// Generate a random unit quaternion [x, y, z, w] via uniform sphere sampling.
fn arb_unit_quaternion() -> impl Strategy<Value = [f64; 4]> {
    // Generate 4 normal-distributed components, then normalize
    (
        prop::num::f64::NORMAL,
        prop::num::f64::NORMAL,
        prop::num::f64::NORMAL,
        prop::num::f64::NORMAL,
    )
        .prop_filter("non-zero norm", |(a, b, c, d)| {
            let n = a * a + b * b + c * c + d * d;
            n > 1e-10 && n.is_finite()
        })
        .prop_map(|(a, b, c, d)| {
            let norm = (a * a + b * b + c * c + d * d).sqrt();
            [a / norm, b / norm, c / norm, d / norm]
        })
}

/// Generate a random valid Transform.
fn arb_transform() -> impl Strategy<Value = Transform> {
    (arb_translation(), arb_unit_quaternion()).prop_map(|(t, r)| Transform::new(t, r))
}

/// Generate a random 3D point in a large range.
fn arb_point() -> impl Strategy<Value = [f64; 3]> {
    arb_translation()
}

// ========================================================================
// Helper
// ========================================================================

fn quat_norm(q: [f64; 4]) -> f64 {
    (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt()
}

fn point_dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn translation_norm(t: [f64; 3]) -> f64 {
    (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt()
}

// ========================================================================
// 1. Inverse Roundtrip: T.compose(T.inverse()) ≈ identity
// ========================================================================

proptest! {
    #[test]
    fn prop_inverse_roundtrip(tf in arb_transform()) {
        let composed = tf.compose(&tf.inverse());
        let id = Transform::identity();

        // Translation should be near zero (relative to transform scale)
        let t_err = translation_norm(composed.translation);
        let scale = 1.0 + translation_norm(tf.translation);
        let tol = 1e-10 * scale;
        prop_assert!(t_err < tol,
            "inverse roundtrip translation error {t_err} > {tol} (scale={scale})");

        // Rotation should be near identity [0,0,0,1] or [0,0,0,-1]
        let q = composed.rotation;
        let dot = q[0] * id.rotation[0] + q[1] * id.rotation[1]
            + q[2] * id.rotation[2] + q[3] * id.rotation[3];
        prop_assert!(dot.abs() > 1.0 - 1e-10,
            "inverse roundtrip rotation error: dot={dot}");
    }
}

// ========================================================================
// 2. Compose Associativity: (A*B)*C ≈ A*(B*C)
// ========================================================================

proptest! {
    #[test]
    fn prop_compose_associativity(
        a in arb_transform(),
        b in arb_transform(),
        c in arb_transform(),
    ) {
        let ab_c = a.compose(&b).compose(&c);
        let a_bc = a.compose(&b.compose(&c));

        let t_err = point_dist(ab_c.translation, a_bc.translation);
        prop_assert!(t_err < 1e-8,
            "associativity translation error {t_err} > 1e-8");

        // Compare rotations via dot product
        let dot: f64 = ab_c.rotation.iter()
            .zip(a_bc.rotation.iter())
            .map(|(x, y)| x * y)
            .sum();
        prop_assert!(dot.abs() > 1.0 - 1e-8,
            "associativity rotation error: dot={dot}");
    }
}

// ========================================================================
// 3. Quaternion Normalization Preservation
// ========================================================================

proptest! {
    #[test]
    fn prop_compose_preserves_unit_quaternion(
        a in arb_transform(),
        b in arb_transform(),
    ) {
        let composed = a.compose(&b);
        let norm = quat_norm(composed.rotation);
        prop_assert!((norm - 1.0).abs() < 1e-10,
            "compose produced non-unit quaternion: norm={norm}");
    }

    #[test]
    fn prop_inverse_preserves_unit_quaternion(tf in arb_transform()) {
        let inv = tf.inverse();
        let norm = quat_norm(inv.rotation);
        prop_assert!((norm - 1.0).abs() < 1e-10,
            "inverse produced non-unit quaternion: norm={norm}");
    }
}

// ========================================================================
// 4. Point Transform Roundtrip
// ========================================================================

proptest! {
    #[test]
    fn prop_point_roundtrip(
        tf in arb_transform(),
        point in arb_point(),
    ) {
        let transformed = tf.transform_point(point);
        let back = tf.inverse().transform_point(transformed);
        let err = point_dist(back, point);

        // Relative tolerance for large coordinates
        let scale = 1.0 + translation_norm(point) + translation_norm(tf.translation);
        let tol = 1e-10 * scale;
        prop_assert!(err < tol,
            "point roundtrip error {err} > {tol} (scale={scale})");
    }
}

// ========================================================================
// 5. SLERP Boundaries
// ========================================================================

proptest! {
    #[test]
    fn prop_slerp_at_zero(
        a in arb_transform(),
        b in arb_transform(),
    ) {
        let result = a.interpolate(&b, 0.0);
        let t_err = point_dist(result.translation, a.translation);
        let scale = 1.0 + translation_norm(a.translation);
        let tol = 1e-10 * scale;
        prop_assert!(t_err < tol,
            "slerp(A,B,0) translation != A: error={t_err}");

        let dot: f64 = result.rotation.iter()
            .zip(a.rotation.iter())
            .map(|(x, y)| x * y)
            .sum();
        prop_assert!(dot.abs() > 1.0 - 1e-10,
            "slerp(A,B,0) rotation != A: dot={dot}");
    }

    #[test]
    fn prop_slerp_at_one(
        a in arb_transform(),
        b in arb_transform(),
    ) {
        let result = a.interpolate(&b, 1.0);
        let t_err = point_dist(result.translation, b.translation);
        let scale = 1.0 + translation_norm(b.translation);
        let tol = 1e-10 * scale;
        prop_assert!(t_err < tol,
            "slerp(A,B,1) translation != B: error={t_err}");

        let dot: f64 = result.rotation.iter()
            .zip(b.rotation.iter())
            .map(|(x, y)| x * y)
            .sum();
        prop_assert!(dot.abs() > 1.0 - 1e-10,
            "slerp(A,B,1) rotation != B: dot={dot}");
    }
}

// ========================================================================
// 6. SLERP Midpoint Symmetry
// ========================================================================

proptest! {
    #[test]
    fn prop_slerp_midpoint_symmetry(
        a in arb_transform(),
        b in arb_transform(),
    ) {
        let ab = a.interpolate(&b, 0.5);
        let ba = b.interpolate(&a, 0.5);

        // Translation midpoint should be identical regardless of direction
        let t_err = point_dist(ab.translation, ba.translation);
        let scale = 1.0 + translation_norm(a.translation) + translation_norm(b.translation);
        let tol = 1e-10 * scale;
        prop_assert!(t_err < tol,
            "slerp midpoint translation asymmetry: error={t_err} > {tol}");

        // Rotation should match (up to sign — q and -q are the same rotation)
        let dot: f64 = ab.rotation.iter()
            .zip(ba.rotation.iter())
            .map(|(x, y)| x * y)
            .sum();
        prop_assert!(dot.abs() > 1.0 - 1e-10,
            "slerp midpoint rotation asymmetry: dot={dot}");
    }
}

// ========================================================================
// 7. Matrix Roundtrip: to_matrix → from_matrix ≈ original
// ========================================================================

proptest! {
    #[test]
    fn prop_matrix_roundtrip(tf in arb_transform()) {
        let matrix = tf.to_matrix();
        let back = Transform::from_matrix(matrix);

        let t_err = point_dist(back.translation, tf.translation);
        prop_assert!(t_err < 1e-10,
            "matrix roundtrip translation error: {t_err}");

        let dot: f64 = back.rotation.iter()
            .zip(tf.rotation.iter())
            .map(|(x, y)| x * y)
            .sum();
        prop_assert!(dot.abs() > 1.0 - 1e-10,
            "matrix roundtrip rotation error: dot={dot}");
    }
}

// ========================================================================
// 8. Chain Resolution Consistency (random tree)
// ========================================================================
//
// For a 4-frame chain A → B → C → D:
//   tf("A", "D") ≈ tf("C", "D").compose(tf("B", "C").compose(tf("A", "B")))
//
// This tests the full TransformFrame pipeline with random transforms.

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]
    #[test]
    fn prop_chain_transitivity(
        t_b in arb_transform(),
        t_c in arb_transform(),
        t_d in arb_transform(),
    ) {
        let hf = TransformFrame::new();
        hf.register_frame("A", None).unwrap();
        hf.register_frame("B", Some("A")).unwrap();
        hf.register_frame("C", Some("B")).unwrap();
        hf.register_frame("D", Some("C")).unwrap();

        let ts = 1_000_000u64;
        hf.update_transform("B", &t_b, ts).unwrap();
        hf.update_transform("C", &t_c, ts).unwrap();
        hf.update_transform("D", &t_d, ts).unwrap();

        let tf_ad = hf.tf("A", "D").unwrap();
        let tf_ab = hf.tf("A", "B").unwrap();
        let tf_bc = hf.tf("B", "C").unwrap();
        let tf_cd = hf.tf("C", "D").unwrap();

        let composed = tf_cd.compose(&tf_bc.compose(&tf_ab));

        let t_err = point_dist(tf_ad.translation, composed.translation);
        let scale = 1.0 + translation_norm(tf_ad.translation);
        let tol = 1e-8 * scale;
        prop_assert!(t_err < tol,
            "chain transitivity translation error {t_err} > {tol}");

        let dot: f64 = tf_ad.rotation.iter()
            .zip(composed.rotation.iter())
            .map(|(x, y)| x * y)
            .sum();
        prop_assert!(dot.abs() > 1.0 - 1e-8,
            "chain transitivity rotation error: dot={dot}");
    }
}

// ========================================================================
// 9. Vector Transform is Rotation-Only
// ========================================================================

proptest! {
    #[test]
    fn prop_vector_ignores_translation(
        tf in arb_transform(),
        vec in arb_point(),
    ) {
        let result = tf.transform_vector(vec);
        let rotation_only = Transform::from_rotation(tf.rotation);
        let expected = rotation_only.transform_point(vec);

        let err = point_dist(result, expected);
        prop_assert!(err < 1e-10,
            "transform_vector should equal rotation-only transform_point: error={err}");
    }
}

// ========================================================================
// 10. Euler Roundtrip
// ========================================================================

proptest! {
    #[test]
    fn prop_euler_roundtrip(
        roll in -std::f64::consts::PI..std::f64::consts::PI,
        pitch in -1.5..1.5f64,  // Avoid gimbal lock at ±π/2
        yaw in -std::f64::consts::PI..std::f64::consts::PI,
    ) {
        let tf = Transform::from_euler([0.0, 0.0, 0.0], [roll, pitch, yaw]);
        let rpy = tf.to_euler();

        // Verify by constructing from the extracted euler and comparing rotations
        let tf2 = Transform::from_euler([0.0, 0.0, 0.0], rpy);
        let dot: f64 = tf.rotation.iter()
            .zip(tf2.rotation.iter())
            .map(|(a, b)| a * b)
            .sum();
        prop_assert!(dot.abs() > 1.0 - 1e-10,
            "euler roundtrip failed: dot={dot}, original=[{roll},{pitch},{yaw}], recovered=[{},{},{}]",
            rpy[0], rpy[1], rpy[2]);
    }
}
