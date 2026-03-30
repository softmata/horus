//! TransformFrame Property-Based Tests
//!
//! Uses proptest to verify algebraic invariants of the transform frame system:
//! - Inverse is truly inverse (T * T^-1 = I)
//! - Chain composition is associative
//! - Self-lookup always returns identity
//! - Pure translations compose additively

use horus_library::transform_frame::{Transform, TransformFrame};
use proptest::prelude::*;

const EPSILON: f64 = 1e-6;

// ── Strategies ──────────────────────────────────────────────────────────────

fn finite_f64() -> impl Strategy<Value = f64> {
    -1e6f64..1e6f64
}

fn arb_translation() -> impl Strategy<Value = [f64; 3]> {
    [finite_f64(), finite_f64(), finite_f64()]
}

/// Generate a unit quaternion [x, y, z, w] from an axis-angle representation.
/// This avoids degenerate quaternions and always produces valid rotations.
fn arb_unit_quaternion() -> impl Strategy<Value = [f64; 4]> {
    (
        -1.0f64..1.0f64,
        -1.0f64..1.0f64,
        -1.0f64..1.0f64,
        -std::f64::consts::PI..std::f64::consts::PI,
    )
        .prop_map(|(ax, ay, az, angle)| {
            let norm = (ax * ax + ay * ay + az * az).sqrt();
            if norm < 1e-12 {
                // Degenerate axis — return identity
                return [0.0, 0.0, 0.0, 1.0];
            }
            let (ax, ay, az) = (ax / norm, ay / norm, az / norm);
            let half = angle / 2.0;
            let s = half.sin();
            let c = half.cos();
            [ax * s, ay * s, az * s, c]
        })
}

fn arb_transform() -> impl Strategy<Value = Transform> {
    (arb_translation(), arb_unit_quaternion()).prop_map(|(t, r)| Transform::new(t, r))
}

fn arb_frame_name() -> impl Strategy<Value = String> {
    "[a-z]{3,8}".prop_map(|s| s)
}

// ── Helpers ─────────────────────────────────────────────────────────────────

fn is_near_identity(t: &Transform) -> bool {
    t.translation[0].abs() < EPSILON
        && t.translation[1].abs() < EPSILON
        && t.translation[2].abs() < EPSILON
        && is_identity_rotation(&t.rotation)
}

fn is_identity_rotation(q: &[f64; 4]) -> bool {
    // q == [0,0,0,1] or q == [0,0,0,-1] (same rotation)
    let dq: f64 = q[0].abs() + q[1].abs() + q[2].abs() + (q[3].abs() - 1.0).abs();
    dq < EPSILON * 4.0
}

fn translations_approx_eq(a: &[f64; 3], b: &[f64; 3]) -> bool {
    (a[0] - b[0]).abs() < EPSILON && (a[1] - b[1]).abs() < EPSILON && (a[2] - b[2]).abs() < EPSILON
}

fn rotations_approx_eq(a: &[f64; 4], b: &[f64; 4]) -> bool {
    // q and -q represent the same rotation
    let dq: f64 = a.iter().zip(b.iter()).map(|(x, y)| (x - y).abs()).sum();
    let dq_neg: f64 = a.iter().zip(b.iter()).map(|(x, y)| (x + y).abs()).sum();
    dq < EPSILON * 4.0 || dq_neg < EPSILON * 4.0
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Inverse is truly inverse: compose(T, inverse(T)) = identity
// ═══════════════════════════════════════════════════════════════════════════

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    #[test]
    fn test_tf_lookup_inverse_is_inverse(t in arb_transform()) {
        let tf = TransformFrame::new();
        tf.register_frame("world", None).unwrap();
        tf.register_frame("child", Some("world")).unwrap();

        let ts = 1_000_000;
        tf.update_transform("child", &t, ts).unwrap();

        // Forward: child -> world
        let forward = tf.tf("child", "world").unwrap();
        // Inverse: world -> child
        let inverse = tf.tf("world", "child").unwrap();

        // compose(forward, inverse) should be identity
        let roundtrip = forward.compose(&inverse);
        prop_assert!(
            is_near_identity(&roundtrip),
            "compose(T, T^-1) is not identity: translation={:?}, rotation={:?}",
            roundtrip.translation,
            roundtrip.rotation,
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Chain associativity: lookup(A,C) = compose(lookup(A,B), lookup(B,C))
// ═══════════════════════════════════════════════════════════════════════════

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    #[test]
    fn test_tf_chain_associativity(
        t_ab in arb_transform(),
        t_bc in arb_transform(),
    ) {
        let tf = TransformFrame::new();
        tf.register_frame("a", None).unwrap();
        tf.register_frame("b", Some("a")).unwrap();
        tf.register_frame("c", Some("b")).unwrap();

        let ts = 1_000_000;
        tf.update_transform("b", &t_ab, ts).unwrap();
        tf.update_transform("c", &t_bc, ts).unwrap();

        // Direct lookup: c -> a (full chain)
        let direct = tf.tf("c", "a").unwrap();

        // Stepped: (b -> a) then compose with (c -> b)
        let ab = tf.tf("b", "a").unwrap();
        let bc = tf.tf("c", "b").unwrap();
        let composed = ab.compose(&bc);

        prop_assert!(
            translations_approx_eq(&direct.translation, &composed.translation),
            "Chain translation mismatch: direct={:?}, composed={:?}",
            direct.translation,
            composed.translation,
        );
        prop_assert!(
            rotations_approx_eq(&direct.rotation, &composed.rotation),
            "Chain rotation mismatch: direct={:?}, composed={:?}",
            direct.rotation,
            composed.rotation,
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Identity lookup: lookup(frame, frame) = identity for any frame
// ═══════════════════════════════════════════════════════════════════════════

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    #[test]
    fn test_tf_identity_lookup(
        name in arb_frame_name(),
        t in arb_transform(),
    ) {
        let tf = TransformFrame::new();
        tf.register_frame("root", None).unwrap();
        tf.register_frame(&name, Some("root")).unwrap();

        let ts = 1_000_000;
        tf.update_transform(&name, &t, ts).unwrap();

        // Self-lookup must always be identity, regardless of the transform set
        let self_tf = tf.tf(&name, &name).unwrap();
        prop_assert!(
            is_near_identity(&self_tf),
            "Self-lookup for '{}' is not identity: translation={:?}, rotation={:?}",
            name,
            self_tf.translation,
            self_tf.rotation,
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Pure translations compose additively
// ═══════════════════════════════════════════════════════════════════════════

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    #[test]
    fn test_tf_translation_composition_is_additive(
        a in finite_f64(),
        b in finite_f64(),
    ) {
        let tf = TransformFrame::new();
        tf.register_frame("world", None).unwrap();
        tf.register_frame("mid", Some("world")).unwrap();
        tf.register_frame("end", Some("mid")).unwrap();

        let ts = 1_000_000;
        tf.update_transform("mid", &Transform::from_translation([a, 0.0, 0.0]), ts)
            .unwrap();
        tf.update_transform("end", &Transform::from_translation([b, 0.0, 0.0]), ts)
            .unwrap();

        let result = tf.tf("end", "world").unwrap();

        prop_assert!(
            (result.translation[0] - (a + b)).abs() < EPSILON,
            "Expected x={}, got x={} for a={}, b={}",
            a + b,
            result.translation[0],
            a,
            b,
        );
        prop_assert!(
            result.translation[1].abs() < EPSILON,
            "Expected y=0, got y={}",
            result.translation[1],
        );
        prop_assert!(
            result.translation[2].abs() < EPSILON,
            "Expected z=0, got z={}",
            result.translation[2],
        );
        // Rotation should still be identity
        prop_assert!(
            is_identity_rotation(&result.rotation),
            "Expected identity rotation, got {:?}",
            result.rotation,
        );
    }
}
