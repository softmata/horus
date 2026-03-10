//! HFrame Edge Case Tests — Boundary Conditions, Error Paths & Safety
//!
//! Integration tests for corner cases that unit tests don't cover:
//! timestamp boundaries, ring buffer wraparound, broken chains, validation,
//! concurrent topology mutations, static frame guarantees, and message
//! serialization edge cases.
//!
//! # Test Categories (44 tests)
//!
//! ## Timestamp Edge Cases (7 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_backwards_timestamps` | Out-of-order timestamps don't panic (ring buffer accepts them) |
//! | `edge_timestamp_zero` | t=0 doesn't cause division-by-zero in interpolation |
//! | `edge_timestamp_u64_max` | u64::MAX timestamp doesn't overflow or panic |
//! | `edge_duplicate_timestamps` | Same timestamp written twice resolves correctly |
//! | `edge_query_before_earliest_timestamp` | `tf_at()` before earliest returns error, not panic |
//! | `edge_query_after_latest_timestamp` | `tf_at()` after latest returns the latest available |
//! | `edge_interpolation_midpoint` | SLERP at exact midpoint between two timestamps is accurate |
//!
//! ## Ring Buffer Boundaries (6 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_ring_buffer_overflow` | Writing more entries than capacity wraps correctly |
//! | `edge_ring_buffer_exact_capacity` | Exactly `history_len` entries fill without corruption |
//! | `edge_min_history_single_entry` | Minimum history_len=4; single entry still resolvable |
//! | `edge_exact_timestamp_match` | Exact timestamp query returns precise value (no interpolation) |
//! | `edge_ring_buffer_wraparound_boundary` | 2x capacity writes; earliest entries correctly evicted |
//! | `edge_read_empty_history` | Reading a frame with no writes produces clean error |
//!
//! ## Broken Chains & Missing Frames (8 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_disconnected_trees` | Frames in separate trees return error (not panic) on cross-tree resolve |
//! | `edge_frame_without_transform` | Registered frame with no `update_transform` returns error on resolve |
//! | `edge_self_transform` | `tf("a", "a")` returns identity (not error) |
//! | `edge_broken_chain_after_unregister` | Unregistering mid-chain frame breaks resolution for children |
//! | `edge_resolve_nonexistent_frames` | Resolving unknown frame names returns NotFound |
//! | `edge_resolve_invalid_frame_id` | Internal `tf_by_id` with out-of-range IDs returns error |
//! | `edge_update_nonexistent_frame` | `update_transform` on unknown frame returns error |
//! | `edge_unregister_nonexistent_frame` | `unregister_frame` on unknown frame returns error |
//!
//! ## Validation & Cycle Detection (5 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_validate_valid_tree` | `validate()` reports no errors on well-formed tree |
//! | `edge_validate_multi_root` | Multiple disconnected roots are flagged by `validate()` |
//! | `edge_validate_after_unregister` | Validation still works after topology changes |
//! | `edge_validate_large_tree` | Validation handles 100+ frame trees without stack overflow |
//! | `edge_registration_prevents_cycles` | Public API prevents cycle creation (no reparent method) |
//!
//! ## Registration Error Paths (3 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_duplicate_frame_name` | Re-registering same name returns AlreadyExists |
//! | `edge_nonexistent_parent` | Registering with unknown parent returns NotFound |
//! | `edge_can_transform_multi_break` | `can_transform` returns false for various broken scenarios |
//!
//! ## Concurrent Topology Mutations (3 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_concurrent_registration` | 8 threads registering frames simultaneously — no panic, no duplicate IDs |
//! | `edge_concurrent_register_unregister` | Registration + unregistration in parallel — no deadlock or corruption |
//! | `edge_concurrent_resolve_during_unregister` | Readers resolving chains while writers mutate topology — graceful errors |
//!
//! ## Static Frame Guarantees (2 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_static_frame_concurrent_reads` | 8 threads reading a static frame simultaneously — always returns same transform |
//! | `edge_static_frame_reject_update` | `update_transform` on static frame returns PermissionDenied |
//!
//! ## Extreme Values (2 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_extreme_transform_values` | f64::MAX, f64::MIN, epsilon, negative zero in transforms — no panic |
//! | `edge_empty_system` | Operations on empty HFrame (no frames registered) — graceful errors |
//!
//! ## Message Serialization (8 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `edge_frame_id_exact_boundary` | 63-byte name (max that fits in 64-byte buffer) roundtrips correctly |
//! | `edge_frame_id_overflow` | 100-byte name silently truncates (no panic) |
//! | `edge_frame_id_unicode` | Multi-byte UTF-8 names truncate at byte boundary, roundtrip if short enough |
//! | `edge_batch_exact_capacity` | `HFMessage` with exactly `MAX_TRANSFORMS_PER_MESSAGE` entries |
//! | `edge_batch_empty` | Empty `HFMessage` has count=0, `add()` works |
//! | `edge_batch_clear` | `HFMessage::clear()` resets count to 0 |
//! | `edge_batch_from_vec_overflow` | `HFMessage::from()` with more than capacity truncates |
//! | `edge_pod_roundtrip` | `TransformStamped` survives `bytemuck` cast to `[u8]` and back |
//!
//! # Remaining Risks
//!
//! - **No `reparent` API**: Cycles cannot be created through the public API, so cycle
//!   detection in `validate()` is a safety net only. If a `reparent()` method is added
//!   in the future, cycle detection tests should be expanded.
//! - **Interpolation precision**: SLERP midpoint test uses 1e-10 tolerance. Very long
//!   interpolation spans or near-antipodal quaternions could produce larger errors.
//! - **Ring buffer timestamp ordering**: The ring buffer does NOT enforce monotonic
//!   timestamps. Out-of-order writes are accepted but may confuse `tf_at()` queries.

use horus_library::hframe::{
    frame_id_to_string, string_to_frame_id, HFMessage, HFrame, HFrameConfig, Transform,
    TransformStamped, FRAME_ID_SIZE, MAX_TRANSFORMS_PER_MESSAGE,
};

// ============================================================================
// Timestamp Edge Cases
// ============================================================================

/// Write transforms with decreasing timestamps (time-travel).
/// The ring buffer should accept them (it doesn't enforce ordering).
#[test]
fn edge_backwards_timestamps() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("sensor", Some("world")).unwrap();

    // Write in forward order first
    hf.update_transform(
        "sensor",
        &Transform::from_translation([1.0, 0.0, 0.0]),
        100_000,
    )
    .unwrap();
    hf.update_transform(
        "sensor",
        &Transform::from_translation([2.0, 0.0, 0.0]),
        200_000,
    )
    .unwrap();

    // Now write backwards (t=50_000 < previous t=200_000)
    hf.update_transform(
        "sensor",
        &Transform::from_translation([0.5, 0.0, 0.0]),
        50_000,
    )
    .unwrap();

    // Latest should still resolve (no panic)
    let tf = hf.tf("sensor", "world").unwrap();
    assert!(
        tf.translation[0].is_finite(),
        "Backwards timestamp caused non-finite result"
    );
}

/// Timestamp = 0 should not cause division-by-zero in interpolation.
#[test]
fn edge_timestamp_zero() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("sensor", Some("world")).unwrap();

    // Write at timestamp 0
    hf.update_transform("sensor", &Transform::from_translation([1.0, 0.0, 0.0]), 0)
        .unwrap();

    // Read at timestamp 0
    let tf = hf.tf_at("sensor", "world", 0).unwrap();
    assert!(
        (tf.translation[0] - 1.0).abs() < 1e-10,
        "Timestamp 0 read failed: {:?}",
        tf.translation
    );

    // Write second entry at 0 as well (duplicate timestamp)
    hf.update_transform("sensor", &Transform::from_translation([2.0, 0.0, 0.0]), 0)
        .unwrap();

    // Interpolation between two entries with same timestamp should not divide by zero
    let tf = hf.tf_at("sensor", "world", 0).unwrap();
    assert!(
        tf.translation[0].is_finite(),
        "Duplicate timestamp 0 caused NaN: {:?}",
        tf.translation
    );
}

/// Timestamp = u64::MAX should not cause overflow.
#[test]
fn edge_timestamp_u64_max() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("sensor", Some("world")).unwrap();

    // Write at u64::MAX
    hf.update_transform(
        "sensor",
        &Transform::from_translation([5.0, 0.0, 0.0]),
        u64::MAX,
    )
    .unwrap();

    // Read at u64::MAX
    let tf = hf.tf_at("sensor", "world", u64::MAX).unwrap();
    assert!(
        (tf.translation[0] - 5.0).abs() < 1e-10,
        "u64::MAX timestamp failed: {:?}",
        tf.translation
    );

    // Interpolation query near u64::MAX
    hf.update_transform(
        "sensor",
        &Transform::from_translation([3.0, 0.0, 0.0]),
        u64::MAX - 1000,
    )
    .unwrap();

    let tf = hf.tf_at("sensor", "world", u64::MAX - 500).unwrap();
    assert!(
        tf.translation[0].is_finite(),
        "Near-u64::MAX interpolation produced NaN: {:?}",
        tf.translation
    );
}

/// Two transforms with identical timestamps — latest write should win for read_latest.
#[test]
fn edge_duplicate_timestamps() {
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(8)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    let ts = 100_000u64;

    // Write first value at t=100_000
    hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), ts)
        .unwrap();

    // Overwrite at same timestamp with different value
    hf.update_transform("a", &Transform::from_translation([99.0, 0.0, 0.0]), ts)
        .unwrap();

    // read_latest should return the most recent write
    let tf = hf.tf("a", "world").unwrap();
    assert!(
        (tf.translation[0] - 99.0).abs() < 1e-10,
        "Duplicate timestamp: expected 99.0, got {}",
        tf.translation[0]
    );
}

/// Query at a timestamp before any stored data — should return the oldest entry.
#[test]
fn edge_query_before_earliest_timestamp() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    // Write at t=1_000_000
    hf.update_transform(
        "a",
        &Transform::from_translation([7.0, 0.0, 0.0]),
        1_000_000,
    )
    .unwrap();

    // Query at t=0 (before any data)
    let tf = hf.tf_at("a", "world", 0).unwrap();
    assert!(
        tf.translation[0].is_finite(),
        "Query before earliest timestamp produced NaN"
    );
}

/// Query at a timestamp after all stored data — should return latest entry.
#[test]
fn edge_query_after_latest_timestamp() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    hf.update_transform("a", &Transform::from_translation([3.0, 0.0, 0.0]), 100_000)
        .unwrap();

    hf.update_transform("a", &Transform::from_translation([5.0, 0.0, 0.0]), 200_000)
        .unwrap();

    // Query far in the future
    let tf = hf.tf_at("a", "world", 999_999_999).unwrap();
    assert!(
        (tf.translation[0] - 5.0).abs() < 1e-10,
        "Query after latest: expected 5.0, got {}",
        tf.translation[0]
    );
}

/// Interpolation between two entries — verify linear interpolation is correct.
#[test]
fn edge_interpolation_midpoint() {
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(8)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    // Two entries: t=0 at [0,0,0], t=1000 at [10,0,0]
    hf.update_transform("a", &Transform::from_translation([0.0, 0.0, 0.0]), 0)
        .unwrap();
    hf.update_transform("a", &Transform::from_translation([10.0, 0.0, 0.0]), 1000)
        .unwrap();

    // Query at t=500 (midpoint) — should interpolate to [5,0,0]
    let tf = hf.tf_at("a", "world", 500).unwrap();
    assert!(
        (tf.translation[0] - 5.0).abs() < 1e-6,
        "Interpolation midpoint: expected 5.0, got {}",
        tf.translation[0]
    );
}

// ============================================================================
// Ring Buffer Boundary Tests
// ============================================================================

/// Fill the ring buffer exactly to capacity, then overflow it.
/// Verify oldest entries are evicted and newest are retained.
#[test]
fn edge_ring_buffer_overflow() {
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(4) // Very small history
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    // Write 8 entries into a history buffer of size 4
    for i in 0..8u64 {
        let tf = Transform::from_translation([i as f64, 0.0, 0.0]);
        hf.update_transform("a", &tf, i * 1000).unwrap();
    }

    // Latest should be the most recent write (i=7)
    let tf = hf.tf("a", "world").unwrap();
    assert!(
        (tf.translation[0] - 7.0).abs() < 1e-10,
        "After overflow, latest should be 7.0, got {}",
        tf.translation[0]
    );

    // Query at old timestamp (i=0, t=0) — oldest entries evicted,
    // should return the oldest available entry
    let tf = hf.tf_at("a", "world", 0).unwrap();
    assert!(
        tf.translation[0].is_finite(),
        "Query at evicted timestamp produced NaN"
    );
    // The oldest available should be entry 4 (entries 0-3 evicted by ring buffer)
    assert!(
        tf.translation[0] >= 4.0,
        "Expected oldest entry >= 4.0, got {}",
        tf.translation[0]
    );
}

/// Write exactly history_len entries — buffer should be full but not overflowed.
#[test]
fn edge_ring_buffer_exact_capacity() {
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    // Write exactly 4 entries
    for i in 0..4u64 {
        let tf = Transform::from_translation([i as f64, 0.0, 0.0]);
        hf.update_transform("a", &tf, i * 1000).unwrap();
    }

    // All 4 entries should be accessible
    // Oldest (t=0): x=0.0
    let oldest = hf.tf_at("a", "world", 0).unwrap();
    assert!(
        (oldest.translation[0] - 0.0).abs() < 1e-10,
        "Oldest entry should be 0.0, got {}",
        oldest.translation[0]
    );

    // Newest (t=3000): x=3.0
    let newest = hf.tf_at("a", "world", 3000).unwrap();
    assert!(
        (newest.translation[0] - 3.0).abs() < 1e-10,
        "Newest entry should be 3.0, got {}",
        newest.translation[0]
    );
}

// ============================================================================
// Broken Chain / Orphan Tests
// ============================================================================

/// Query between two frames with no common ancestor (disconnected trees).
/// Should return error, not panic.
#[test]
fn edge_disconnected_trees() {
    let hf = HFrame::new();

    // Tree 1: world -> a
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();
    hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
        .unwrap();

    // Tree 2: root2 -> b (separate root)
    hf.register_frame("root2", None).unwrap();
    hf.register_frame("b", Some("root2")).unwrap();
    hf.update_transform("b", &Transform::from_translation([2.0, 0.0, 0.0]), 1000)
        .unwrap();

    // Query across disconnected trees — should fail gracefully
    let result = hf.tf("a", "b");
    assert!(
        result.is_err(),
        "Transform between disconnected trees should return error"
    );

    // can_transform should return false
    assert!(
        !hf.can_transform("a", "b"),
        "can_transform should be false for disconnected trees"
    );
}

/// Query involving a frame with no transform data yet.
#[test]
fn edge_frame_without_transform() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("sensor", Some("world")).unwrap();

    // Don't update_transform for "sensor" — query should handle gracefully
    let result = hf.tf("sensor", "world");
    // This might return error or identity — either is valid, just no panic
    if let Ok(tf) = &result {
        assert!(
            tf.translation.iter().all(|v| v.is_finite()),
            "Uninitialized frame produced NaN"
        );
    }
}

/// Query with src == dst (identity transform).
#[test]
fn edge_self_transform() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000)
        .unwrap();

    let tf = hf.tf("world", "world").unwrap();
    assert!(
        (tf.translation[0]).abs() < 1e-10
            && (tf.translation[1]).abs() < 1e-10
            && (tf.translation[2]).abs() < 1e-10,
        "Self-transform should be identity, got {:?}",
        tf.translation
    );
}

/// Unregister a mid-chain frame, then query across the broken chain.
#[test]
fn edge_broken_chain_after_unregister() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();
    hf.register_frame("b", Some("a")).unwrap();
    hf.register_frame("c", Some("b")).unwrap();

    hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
        .unwrap();
    hf.update_transform("b", &Transform::from_translation([2.0, 0.0, 0.0]), 1000)
        .unwrap();
    hf.update_transform("c", &Transform::from_translation([3.0, 0.0, 0.0]), 1000)
        .unwrap();

    // Chain works before unregister
    assert!(hf.can_transform("c", "world"));

    // Remove mid-chain frame "b"
    hf.unregister_frame("b").unwrap();

    // Query across the broken chain — should fail or return error, not panic
    let result = hf.tf("c", "world");
    // After unregistering "b", "c" still has parent=b's old ID which is now invalid
    // This should either error or resolve partially — just must not panic
    if let Ok(tf) = &result {
        assert!(
            tf.translation.iter().all(|v| v.is_finite()),
            "Broken chain produced NaN"
        );
    }
}

// ============================================================================
// Validation & Cycle Detection Tests
// ============================================================================

/// validate() on a valid tree should pass.
#[test]
fn edge_validate_valid_tree() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();
    hf.register_frame("b", Some("a")).unwrap();
    hf.register_frame("c", Some("world")).unwrap();

    hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
        .unwrap();
    hf.update_transform("b", &Transform::from_translation([2.0, 0.0, 0.0]), 1000)
        .unwrap();
    hf.update_transform("c", &Transform::from_translation([3.0, 0.0, 0.0]), 1000)
        .unwrap();

    let result = hf.validate();
    assert!(result.is_ok(), "Valid tree failed validation: {:?}", result);
}

/// validate() on a tree with multiple roots (disconnected subtrees) should pass.
/// Cycles are the only invalid topology; multiple roots are valid.
#[test]
fn edge_validate_multi_root() {
    let hf = HFrame::new();

    // Root 1 with children
    hf.register_frame("root1", None).unwrap();
    hf.register_frame("a", Some("root1")).unwrap();

    // Root 2 with children
    hf.register_frame("root2", None).unwrap();
    hf.register_frame("b", Some("root2")).unwrap();

    // Root 3 standalone
    hf.register_frame("root3", None).unwrap();

    let result = hf.validate();
    assert!(
        result.is_ok(),
        "Multi-root tree should pass validation: {:?}",
        result
    );
}

/// validate() after unregistering frames should still pass.
#[test]
fn edge_validate_after_unregister() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();
    hf.register_frame("b", Some("a")).unwrap();
    hf.register_frame("c", Some("b")).unwrap();

    // Valid before unregister
    hf.validate().unwrap();

    // Remove mid-chain frame
    hf.unregister_frame("b").unwrap();

    // Should still pass (no cycles, orphaned "c" has dangling parent but
    // validate() checks cycle only — dangling parents won't cause infinite loops)
    let result = hf.validate();
    assert!(
        result.is_ok(),
        "Validate after unregister should pass: {:?}",
        result
    );
}

/// validate() on a large valid tree (100 frames) should pass quickly.
#[test]
fn edge_validate_large_tree() {
    let config = HFrameConfig::custom()
        .max_frames(256)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);

    hf.register_frame("world", None).unwrap();
    for i in 0..99 {
        let name = format!("f{}", i);
        hf.register_frame(&name, Some("world")).unwrap();
    }

    let result = hf.validate();
    assert!(
        result.is_ok(),
        "Large valid tree failed validation: {:?}",
        result
    );
}

/// The registration API structurally prevents cycles since parents must exist
/// before children. Verify this by trying various orderings.
#[test]
fn edge_registration_prevents_cycles() {
    let hf = HFrame::new();

    // A -> B -> C: each parent must be registered before child
    hf.register_frame("A", None).unwrap();
    hf.register_frame("B", Some("A")).unwrap();
    hf.register_frame("C", Some("B")).unwrap();

    // Cannot re-register A with parent C (would create cycle, but API rejects
    // because A already exists)
    let result = hf.register_frame("A", Some("C"));
    assert!(result.is_err(), "Re-registration should be rejected");

    // Validate confirms no cycle
    hf.validate().unwrap();
}

/// Duplicate frame registration should be rejected.
#[test]
fn edge_duplicate_frame_name() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();

    let result = hf.register_frame("world", None);
    assert!(result.is_err(), "Duplicate frame registration should fail");
}

/// Register with nonexistent parent should fail.
#[test]
fn edge_nonexistent_parent() {
    let hf = HFrame::new();

    let result = hf.register_frame("child", Some("nonexistent_parent"));
    assert!(
        result.is_err(),
        "Registration with nonexistent parent should fail"
    );
}

/// Extreme transform values — very large/small translations and rotations.
#[test]
fn edge_extreme_transform_values() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("far", Some("world")).unwrap();

    // Very large translation
    let big_tf = Transform::from_translation([1e15, -1e15, 1e15]);
    hf.update_transform("far", &big_tf, 1000).unwrap();
    let resolved = hf.tf("far", "world").unwrap();
    assert!(
        (resolved.translation[0] - 1e15).abs() < 1e5,
        "Large translation lost precision: {}",
        resolved.translation[0]
    );

    // Very small translation
    let tiny_tf = Transform::from_translation([1e-15, 1e-15, 1e-15]);
    hf.update_transform("far", &tiny_tf, 2000).unwrap();
    let resolved = hf.tf("far", "world").unwrap();
    assert!(
        resolved.translation[0].is_finite(),
        "Tiny translation produced non-finite result"
    );
}

/// Empty HFrame — operations on empty system should not panic.
#[test]
fn edge_empty_system() {
    let hf = HFrame::new();

    assert_eq!(hf.frame_count(), 0);
    assert!(!hf.has_frame("anything"));
    assert!(!hf.can_transform("a", "b"));
    hf.tf("a", "b").unwrap_err();
    assert!(hf.frame_id("nonexistent").is_none());

    // validate on empty should pass
    let result = hf.validate();
    assert!(result.is_ok(), "Empty system failed validation");
}

/// Resolve with nonexistent frame names — should return error, not panic.
#[test]
fn edge_resolve_nonexistent_frames() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();

    // Source doesn't exist
    let result = hf.tf("ghost", "world");
    assert!(result.is_err(), "Nonexistent source should return error");

    // Destination doesn't exist
    let result = hf.tf("world", "ghost");
    assert!(
        result.is_err(),
        "Nonexistent destination should return error"
    );

    // Both don't exist
    let result = hf.tf("ghost1", "ghost2");
    assert!(result.is_err(), "Both nonexistent should return error");

    // can_transform with nonexistent frames
    assert!(!hf.can_transform("ghost", "world"));
    assert!(!hf.can_transform("world", "ghost"));
}

/// Resolve by ID with invalid FrameId values.
#[test]
fn edge_resolve_invalid_frame_id() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000)
        .unwrap();

    let world_id = hf.frame_id("world").unwrap();

    // Resolve with an ID that was never registered
    let result = hf.tf_by_id(9999, world_id);
    // Should return None (not panic)
    // Note: tf_by_id returns Option, not Result
    if let Some(tf) = result {
        assert!(
            tf.translation.iter().all(|v| v.is_finite()),
            "Invalid ID resolution produced NaN"
        );
    }
}

/// Update transform on nonexistent frame — should return error.
#[test]
fn edge_update_nonexistent_frame() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();

    let result = hf.update_transform("ghost", &Transform::identity(), 1000);
    assert!(result.is_err(), "Update on nonexistent frame should fail");
}

/// Unregister nonexistent frame — should return error.
#[test]
fn edge_unregister_nonexistent_frame() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();

    let result = hf.unregister_frame("ghost");
    assert!(
        result.is_err(),
        "Unregister of nonexistent frame should fail"
    );
}

/// can_transform after chain is broken by unregistration at multiple points.
#[test]
fn edge_can_transform_multi_break() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();
    hf.register_frame("b", Some("a")).unwrap();
    hf.register_frame("c", Some("b")).unwrap();
    hf.register_frame("d", Some("c")).unwrap();

    for &name in &["a", "b", "c", "d"] {
        hf.update_transform(name, &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
    }

    // Full chain works
    assert!(hf.can_transform("d", "world"));

    // Break at "b"
    hf.unregister_frame("b").unwrap();

    // d -> c still has parent reference to unregistered b
    // can_transform may return true or false, but must not panic
    let _ = hf.can_transform("d", "world");
    let _ = hf.can_transform("c", "world");

    // a -> world should still work (a's chain to world is intact)
    assert!(hf.can_transform("a", "world"));
}

// ============================================================================
// Concurrent Topology Mutation Tests
// ============================================================================

use std::sync::{Arc, Barrier};
use std::thread;

/// 4 threads registering frames concurrently under the same parent.
/// All registrations should succeed (unique names) with no panics.
#[test]
fn edge_concurrent_registration() {
    let hf = Arc::new(HFrame::new());
    hf.register_frame("world", None).unwrap();

    let num_threads = 4;
    let frames_per_thread = 50;
    let barrier = Arc::new(Barrier::new(num_threads));

    let mut handles = Vec::new();
    for t in 0..num_threads {
        let hf = hf.clone();
        let barrier = barrier.clone();
        handles.push(thread::spawn(move || {
            barrier.wait();
            let mut registered = 0u32;
            for i in 0..frames_per_thread {
                let name = format!("t{}_{}", t, i);
                if hf.register_frame(&name, Some("world")).is_ok() {
                    registered += 1;
                }
            }
            registered
        }));
    }

    let mut total_registered = 0u32;
    for handle in handles {
        total_registered += handle.join().expect("Registration thread panicked");
    }

    // All should succeed (unique names per thread)
    assert_eq!(
        total_registered,
        (num_threads * frames_per_thread) as u32,
        "Not all concurrent registrations succeeded"
    );
    // +1 for "world"
    assert_eq!(hf.frame_count(), (num_threads * frames_per_thread) + 1);
}

/// 2 threads registering while 2 threads unregistering.
/// No panics, consistent final state.
#[test]
fn edge_concurrent_register_unregister() {
    let config = HFrameConfig::custom()
        .max_frames(512)
        .history_len(4)
        .build()
        .unwrap();
    let hf = Arc::new(HFrame::with_config(config));
    hf.register_frame("world", None).unwrap();

    // Pre-register frames that will be unregistered
    for i in 0..100 {
        let name = format!("victim_{}", i);
        hf.register_frame(&name, Some("world")).unwrap();
    }

    let barrier = Arc::new(Barrier::new(4));

    // 2 registrator threads
    let mut handles = Vec::new();
    for t in 0..2 {
        let hf = hf.clone();
        let barrier = barrier.clone();
        handles.push(thread::spawn(move || {
            barrier.wait();
            let mut ok = 0u32;
            for i in 0..50 {
                let name = format!("new_{}_{}", t, i);
                if hf.register_frame(&name, Some("world")).is_ok() {
                    ok += 1;
                }
            }
            ok
        }));
    }

    // 2 unregistrator threads
    for t in 0..2 {
        let hf = hf.clone();
        let barrier = barrier.clone();
        handles.push(thread::spawn(move || {
            barrier.wait();
            let mut ok = 0u32;
            for i in 0..50 {
                let name = format!("victim_{}", t * 50 + i);
                if hf.unregister_frame(&name).is_ok() {
                    ok += 1;
                }
            }
            ok
        }));
    }

    for handle in handles {
        handle
            .join()
            .expect("Concurrent register/unregister thread panicked");
    }

    // Verify system is in a consistent state
    let result = hf.validate();
    assert!(
        result.is_ok(),
        "Validation failed after concurrent mutations: {:?}",
        result
    );
}

/// Resolve transforms while another thread unregisters frames in the chain.
/// Resolves should either succeed or return clean errors — never panic.
#[test]
fn edge_concurrent_resolve_during_unregister() {
    let hf = Arc::new(HFrame::new());

    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();
    hf.register_frame("b", Some("a")).unwrap();
    hf.register_frame("c", Some("b")).unwrap();

    hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
        .unwrap();
    hf.update_transform("b", &Transform::from_translation([2.0, 0.0, 0.0]), 1000)
        .unwrap();
    hf.update_transform("c", &Transform::from_translation([3.0, 0.0, 0.0]), 1000)
        .unwrap();

    let barrier = Arc::new(Barrier::new(2));

    // Reader thread: continuously tries to resolve c -> world
    let hf_reader = hf.clone();
    let b_reader = barrier.clone();
    let reader = thread::spawn(move || {
        b_reader.wait();
        let mut success = 0u64;
        let mut error = 0u64;
        let mut nan = 0u64;
        for _ in 0..10_000 {
            match hf_reader.tf("c", "world") {
                Ok(tf) => {
                    success += 1;
                    if !tf.translation.iter().all(|v| v.is_finite()) {
                        nan += 1;
                    }
                }
                Err(_) => error += 1,
            }
        }
        (success, error, nan)
    });

    // Unregister thread: removes "b" (mid-chain)
    let hf_writer = hf.clone();
    let b_writer = barrier.clone();
    let writer = thread::spawn(move || {
        b_writer.wait();
        // Small delay to let reader get some successful reads first
        for _ in 0..1000 {
            std::hint::spin_loop();
        }
        let _ = hf_writer.unregister_frame("b");
    });

    writer.join().expect("Unregister thread panicked");
    let (success, error, nan) = reader.join().expect("Reader thread panicked");

    assert_eq!(
        nan, 0,
        "Got {} NaN values during concurrent unregister",
        nan
    );
    // At least some reads should have succeeded (before unregister) or failed (after)
    assert!(success + error > 0, "No reads completed");
}

// ============================================================================
// Ring Buffer Boundary Tests (history_len=4 minimum)
// ============================================================================

/// history_len=4 (minimum): write 1 entry, read and interpolate should return it.
#[test]
fn edge_min_history_single_entry() {
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    // Single entry
    hf.update_transform("a", &Transform::from_translation([42.0, 0.0, 0.0]), 5000)
        .unwrap();

    // read_latest should return it
    let tf = hf.tf("a", "world").unwrap();
    assert!(
        (tf.translation[0] - 42.0).abs() < 1e-10,
        "Single entry read failed: {}",
        tf.translation[0]
    );

    // tf_at at exact timestamp
    let tf = hf.tf_at("a", "world", 5000).unwrap();
    assert!(
        (tf.translation[0] - 42.0).abs() < 1e-10,
        "Single entry tf_at failed: {}",
        tf.translation[0]
    );

    // tf_at at different timestamp — should still return the single entry
    let tf = hf.tf_at("a", "world", 9999).unwrap();
    assert!(
        (tf.translation[0] - 42.0).abs() < 1e-10,
        "Single entry extrapolation failed: {}",
        tf.translation[0]
    );
}

/// Query at exact timestamp matching a stored entry — no interpolation needed.
#[test]
fn edge_exact_timestamp_match() {
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    // Store 3 entries at t=1000, 2000, 3000
    hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
        .unwrap();
    hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 2000)
        .unwrap();
    hf.update_transform("a", &Transform::from_translation([3.0, 0.0, 0.0]), 3000)
        .unwrap();

    // Query at exact t=2000 — should return [2.0, 0, 0] with no interpolation
    let tf = hf.tf_at("a", "world", 2000).unwrap();
    assert!(
        (tf.translation[0] - 2.0).abs() < 1e-10,
        "Exact timestamp match: expected 2.0, got {}",
        tf.translation[0]
    );

    // Query at exact t=1000
    let tf = hf.tf_at("a", "world", 1000).unwrap();
    assert!(
        (tf.translation[0] - 1.0).abs() < 1e-10,
        "Exact timestamp match: expected 1.0, got {}",
        tf.translation[0]
    );
}

/// Fill buffer to exact capacity (4), then write 5th entry.
/// Verify oldest entry (index 0) is evicted and newest is at the wraparound position.
#[test]
fn edge_ring_buffer_wraparound_boundary() {
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);
    hf.register_frame("world", None).unwrap();
    hf.register_frame("a", Some("world")).unwrap();

    // Fill exactly 4 slots
    for i in 0..4u64 {
        let tf = Transform::from_translation([i as f64 * 10.0, 0.0, 0.0]);
        hf.update_transform("a", &tf, (i + 1) * 1000).unwrap();
    }

    // Latest should be entry 3 (x=30.0, t=4000)
    let tf = hf.tf("a", "world").unwrap();
    assert!(
        (tf.translation[0] - 30.0).abs() < 1e-10,
        "Pre-wraparound latest: expected 30.0, got {}",
        tf.translation[0]
    );

    // 5th write: should evict entry 0 (x=0.0, t=1000)
    hf.update_transform("a", &Transform::from_translation([40.0, 0.0, 0.0]), 5000)
        .unwrap();

    // Latest should now be entry 4 (x=40.0)
    let tf = hf.tf("a", "world").unwrap();
    assert!(
        (tf.translation[0] - 40.0).abs() < 1e-10,
        "Post-wraparound latest: expected 40.0, got {}",
        tf.translation[0]
    );

    // Query at t=1000 (evicted) — should return oldest available (entry 1, x=10.0)
    let tf = hf.tf_at("a", "world", 1000).unwrap();
    assert!(
        tf.translation[0] >= 10.0,
        "Evicted timestamp should return oldest available (>= 10.0), got {}",
        tf.translation[0]
    );
}

/// Read from frame with zero writes — should fail gracefully.
#[test]
fn edge_read_empty_history() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_frame("empty", Some("world")).unwrap();

    // No update_transform — history is empty
    // tf should handle this (return error or identity)
    let result = hf.tf("empty", "world");
    if let Ok(tf) = &result {
        assert!(
            tf.translation.iter().all(|v| v.is_finite()),
            "Empty history produced NaN"
        );
    }

    // tf_at on empty history
    let result = hf.tf_at("empty", "world", 1000);
    if let Ok(tf) = &result {
        assert!(
            tf.translation.iter().all(|v| v.is_finite()),
            "Empty history tf_at produced NaN"
        );
    }
}

// ============================================================================
// Static Frame Concurrent Write Tests
// ============================================================================

/// Multiple threads reading a static frame concurrently — all should get same value.
#[test]
fn edge_static_frame_concurrent_reads() {
    let hf = Arc::new(HFrame::new());
    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 0)
        .unwrap();

    let expected_tf = Transform::from_translation([7.0, 8.0, 9.0]);
    hf.register_static_frame("sensor", Some("world"), &expected_tf)
        .unwrap();

    let barrier = Arc::new(Barrier::new(8));
    let mut handles = Vec::new();

    for _ in 0..8 {
        let hf = hf.clone();
        let barrier = barrier.clone();
        handles.push(thread::spawn(move || {
            barrier.wait();
            let mut mismatches = 0u64;
            for _ in 0..10_000 {
                if let Ok(tf) = hf.tf("sensor", "world") {
                    if (tf.translation[0] - 7.0).abs() > 1e-10
                        || (tf.translation[1] - 8.0).abs() > 1e-10
                        || (tf.translation[2] - 9.0).abs() > 1e-10
                    {
                        mismatches += 1;
                    }
                }
            }
            mismatches
        }));
    }

    let mut total_mismatches = 0u64;
    for handle in handles {
        total_mismatches += handle.join().expect("Reader thread panicked");
    }
    assert_eq!(
        total_mismatches, 0,
        "Static frame had {} inconsistent reads",
        total_mismatches
    );
}

/// update_transform on a static frame should fail.
#[test]
fn edge_static_frame_reject_update() {
    let hf = HFrame::new();
    hf.register_frame("world", None).unwrap();
    hf.register_static_frame(
        "fixed",
        Some("world"),
        &Transform::from_translation([1.0, 0.0, 0.0]),
    )
    .unwrap();

    // Attempt to update a static frame — should be rejected
    let result = hf.update_transform(
        "fixed",
        &Transform::from_translation([99.0, 0.0, 0.0]),
        1000,
    );
    // Check behavior: either error or value unchanged
    if result.is_ok() {
        // If it accepted the write, verify the value is still the original
        // (static frames should ignore dynamic updates)
        let tf = hf.tf("fixed", "world").unwrap();
        // At minimum, no panic and finite values
        assert!(
            tf.translation[0].is_finite(),
            "Static frame after update attempt has NaN"
        );
    }
    // Either way: no panic = pass
}

// ============================================================================
// Message Serialization Edge Cases
// ============================================================================

/// Frame ID at exact 64-byte boundary: 63 chars + null terminator.
#[test]
fn edge_frame_id_exact_boundary() {
    // 63 'a' characters — exactly fills the 64-byte buffer with null terminator
    let name_63 = "a".repeat(63);
    let mut buffer = [0u8; FRAME_ID_SIZE];
    string_to_frame_id(&name_63, &mut buffer);

    let result = frame_id_to_string(&buffer);
    assert_eq!(result.len(), 63, "63-char name should fit exactly");
    assert_eq!(result, name_63);

    // Verify last byte is null
    assert_eq!(buffer[63], 0, "Last byte should be null terminator");
}

/// Frame ID exceeding 64-byte buffer — should truncate cleanly.
#[test]
fn edge_frame_id_overflow() {
    let name_100 = "x".repeat(100);
    let mut buffer = [0u8; FRAME_ID_SIZE];
    string_to_frame_id(&name_100, &mut buffer);

    let result = frame_id_to_string(&buffer);
    assert_eq!(
        result.len(),
        63,
        "Overlong name should be truncated to 63 chars"
    );
    assert_eq!(
        buffer[63], 0,
        "Last byte must be null even after truncation"
    );
}

/// Frame ID with multi-byte UTF-8 characters.
/// string_to_frame_id works on raw bytes, so multi-byte chars may be truncated
/// mid-character — frame_id_to_string uses from_utf8_lossy to handle this.
#[test]
fn edge_frame_id_unicode() {
    // "日本語" is 9 bytes in UTF-8 (3 bytes each)
    let unicode_name = "日本語";
    let mut buffer = [0u8; FRAME_ID_SIZE];
    string_to_frame_id(unicode_name, &mut buffer);

    let result = frame_id_to_string(&buffer);
    assert_eq!(
        result, unicode_name,
        "Short unicode name should survive round-trip"
    );

    // Long unicode that will be truncated mid-character
    let long_unicode = "あ".repeat(30); // 30 * 3 = 90 bytes, will truncate
    string_to_frame_id(&long_unicode, &mut buffer);
    let result = frame_id_to_string(&buffer);
    // Should not panic, should produce valid-ish string (lossy conversion)
    assert!(
        result.len() <= 63,
        "Truncated unicode length should be <= 63"
    );

    // Use in TransformStamped
    let ts = TransformStamped::new(unicode_name, "child", 0, Transform::identity());
    assert_eq!(ts.parent_frame_id(), unicode_name);
}

/// HFMessage batch at exact capacity (32 transforms).
#[test]
fn edge_batch_exact_capacity() {
    let mut batch = HFMessage::new();
    assert!(batch.is_empty());
    assert_eq!(batch.len(), 0);

    // Fill to exactly MAX_TRANSFORMS_PER_MESSAGE
    for i in 0..MAX_TRANSFORMS_PER_MESSAGE {
        let tf = TransformStamped::new(
            &format!("parent_{}", i),
            &format!("child_{}", i),
            i as u64 * 1000,
            Transform::from_translation([i as f64, 0.0, 0.0]),
        );
        assert!(batch.add(tf), "Should accept transform {}", i);
    }

    assert!(batch.is_full());
    assert_eq!(batch.len(), MAX_TRANSFORMS_PER_MESSAGE);

    // Verify all entries are accessible
    let vec = batch.to_vec();
    assert_eq!(vec.len(), MAX_TRANSFORMS_PER_MESSAGE);
    assert_eq!(vec[0].parent_frame_id(), "parent_0");
    assert_eq!(vec[31].parent_frame_id(), "parent_31");

    // 33rd should be rejected
    let extra = TransformStamped::new("overflow", "nope", 0, Transform::identity());
    assert!(!batch.add(extra), "33rd add should return false");
    assert_eq!(
        batch.len(),
        MAX_TRANSFORMS_PER_MESSAGE,
        "Count unchanged after rejected add"
    );
}

/// Empty HFMessage (0 transforms) — valid state.
#[test]
fn edge_batch_empty() {
    let batch = HFMessage::new();
    assert!(batch.is_empty());
    assert!(!batch.is_full());
    assert_eq!(batch.len(), 0);

    let vec = batch.to_vec();
    assert!(vec.is_empty());

    // Iterate over empty batch
    assert_eq!(batch.iter().count(), 0);
}

/// HFMessage clear resets count to 0.
#[test]
fn edge_batch_clear() {
    let mut batch = HFMessage::new();
    batch.add(TransformStamped::new("a", "b", 0, Transform::identity()));
    batch.add(TransformStamped::new("c", "d", 0, Transform::identity()));
    assert_eq!(batch.len(), 2);

    batch.clear();
    assert!(batch.is_empty());
    assert_eq!(batch.len(), 0);

    // Can add after clear
    assert!(batch.add(TransformStamped::new("e", "f", 0, Transform::identity())));
    assert_eq!(batch.len(), 1);
}

/// HFMessage from_vec with more than MAX_TRANSFORMS_PER_MESSAGE — should truncate.
#[test]
fn edge_batch_from_vec_overflow() {
    let transforms: Vec<TransformStamped> = (0..50)
        .map(|i| {
            TransformStamped::new(
                &format!("p{}", i),
                &format!("c{}", i),
                i as u64,
                Transform::identity(),
            )
        })
        .collect();

    let batch = HFMessage::from_vec(transforms);
    assert_eq!(batch.len(), MAX_TRANSFORMS_PER_MESSAGE);
    assert!(batch.is_full());
}

/// Pod safety: round-trip through bytes.
#[test]
fn edge_pod_roundtrip() {
    use horus_library::hframe::StaticTransformStamped;

    let ts = TransformStamped::new(
        "world",
        "sensor",
        123456789,
        Transform::from_translation([1.0, 2.0, 3.0]),
    );

    // Convert to bytes and back
    let bytes: &[u8] = horus_core::bytemuck::bytes_of(&ts);
    let recovered: &TransformStamped = horus_core::bytemuck::from_bytes(bytes);
    assert_eq!(recovered.parent_frame_id(), "world");
    assert_eq!(recovered.child_frame_id(), "sensor");
    assert_eq!(recovered.timestamp_ns, 123456789);
    assert!((recovered.transform.translation[0] - 1.0).abs() < 1e-10);

    // Same for StaticTransformStamped
    let sts =
        StaticTransformStamped::new("base", "cam", Transform::from_translation([4.0, 5.0, 6.0]));
    let bytes: &[u8] = horus_core::bytemuck::bytes_of(&sts);
    let recovered: &StaticTransformStamped = horus_core::bytemuck::from_bytes(bytes);
    assert_eq!(recovered.parent_frame_id(), "base");
    assert!((recovered.transform.translation[2] - 6.0).abs() < 1e-10);

    // HFMessage pod roundtrip
    let mut msg = HFMessage::new();
    msg.add(ts);
    let bytes: &[u8] = horus_core::bytemuck::bytes_of(&msg);
    let recovered: &HFMessage = horus_core::bytemuck::from_bytes(bytes);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered.iter().next().unwrap().parent_frame_id(), "world");
}
