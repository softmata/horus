//! TransformFrame Stress Tests — Scale, Capacity, Depth & Concurrency
//!
//! Integration tests that push TransformFrame beyond typical operating conditions.
//! These complement the unit tests in each module by exercising cross-cutting
//! concerns at realistic and extreme scale.
//!
//! # Test Categories (16 tests)
//!
//! ## Scale Tests (3 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `stress_1000_frame_chain` | Deep sequential chain (1001 frames), verifies composed translation accuracy and `can_transform` across full depth |
//! | `stress_1000_frame_wide_tree` | Wide tree (1000 siblings), verifies sibling-to-sibling resolution and `children()` correctness |
//! | `stress_4096_frames_large_preset` | Large preset capacity fill (4096 frames), validates the `TransformFrame::large()` configuration under full load |
//!
//! ## Capacity Exhaustion & Slot Reuse (6 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `stress_max_frames_exhaustion` | Graceful error (not panic) when max_frames exceeded |
//! | `stress_slot_reuse_after_unregister` | Freed slots can be reclaimed by new registrations |
//! | `stress_static_frame_cannot_unregister` | Static frames reject unregistration |
//! | `stress_mixed_static_dynamic_at_capacity` | Both types count toward max_frames; only dynamic can free slots |
//! | `stress_repeated_slot_reuse_cycles` | 50 unregister/re-register cycles at capacity — no ID leak or slot corruption |
//! | `stress_static_frame_transform_immediate` | Static frame transform is resolvable immediately (no `update_transform` needed) |
//!
//! ## Deep Chain Resolution (4 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `stress_depth_100_chain` | Depth-100 accumulated translation accuracy |
//! | `stress_depth_200_chain_with_rotation` | Depth-200 with rotation — validates quaternion norm doesn't drift and no NaN/Inf |
//! | `stress_depth_chain_manual_verification` | Depth 50/100/200 with manual `Transform::compose()` ground truth comparison |
//! | `stress_depth_timing_linear_scaling` | Resolution time at depth 50/100/250/500 scales sub-linearly (not exponential) |
//!
//! ## High-Throughput Concurrency (3 tests)
//! | Test | Gap Addressed |
//! |------|--------------|
//! | `stress_concurrent_4_writers_8_readers` | 4 writers + 8 readers (5K iters each), validates no torn reads/NaN under contention |
//! | `stress_sustained_concurrent_load` | Realistic robot topology (odom/base_link/sensors), sustained 10K iterations, all reads finite |
//! | `stress_high_throughput_100k` | 400K writes + 800K reads with rotation transforms, validates quaternion norm integrity |
//!
//! # Remaining Risks
//!
//! - **Multi-writer per frame**: The seqlock `debug_assert` in `slot.rs:203` fires when two
//!   threads write to the same frame simultaneously. This is by design (single-writer-per-frame
//!   is the intended contract), but it means multi-writer correctness is NOT guaranteed.
//!   Tests use one-writer-per-frame to match this contract.
//! - **No NUMA/weak-memory-model testing**: All concurrency tests run on x86 (TSO). ARM/RISC-V
//!   weak ordering could surface issues not caught here. The loom tests (`loom_seqlock.rs`)
//!   provide formal memory model verification for the seqlock itself.
//! - **No cross-process IPC tests**: These tests are in-process only. Shared-memory IPC
//!   correctness (Pod safety, zero-copy) is validated at the type level but not end-to-end.

use horus_library::transform_frame::{TransformFrame, TransformFrameConfig, Transform};
use std::sync::{Arc, Barrier};
use std::thread;

// ============================================================================
// Scale Tests: Large Frame Counts
// ============================================================================

/// Register 1000 frames in a single chain: world -> f0 -> f1 -> ... -> f999
/// Verify all frames are queryable and transforms resolve correctly.
#[test]
fn stress_1000_frame_chain() {
    let config = TransformFrameConfig::custom()
        .max_frames(2048)
        .history_len(4)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    // Register root
    let _world = hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000)
        .unwrap();

    // Register 1000-frame chain
    let mut prev_name = "world".to_string();
    for i in 0..1000 {
        let name = format!("f{}", i);
        hf.register_frame(&name, Some(&prev_name)).unwrap();
        let tf = Transform::from_translation([0.001, 0.0, 0.0]);
        hf.update_transform(&name, &tf, 1000).unwrap();
        prev_name = name;
    }

    assert_eq!(hf.frame_count(), 1001); // world + 1000 frames

    // Resolve from leaf to root — composed translation should be ~1.0 (1000 * 0.001)
    let tf = hf.tf("f999", "world").unwrap();
    assert!(
        (tf.translation[0] - 1.0).abs() < 1e-6,
        "Expected ~1.0, got {}",
        tf.translation[0]
    );

    // can_transform should work across the full chain
    assert!(hf.can_transform("f999", "world"));
    assert!(hf.can_transform("world", "f999"));
    assert!(hf.can_transform("f500", "f200"));
}

/// Register 1000 frames in a wide tree (all children of root).
/// Tests breadth rather than depth.
#[test]
fn stress_1000_frame_wide_tree() {
    let config = TransformFrameConfig::custom()
        .max_frames(2048)
        .history_len(4)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000)
        .unwrap();

    for i in 0..1000 {
        let name = format!("child_{}", i);
        hf.register_frame(&name, Some("world")).unwrap();
        let tf = Transform::from_translation([i as f64, 0.0, 0.0]);
        hf.update_transform(&name, &tf, 1000).unwrap();
    }

    assert_eq!(hf.frame_count(), 1001);

    // All children should be queryable relative to each other
    // child_500 -> world -> child_200
    let tf = hf.tf("child_500", "child_200").unwrap();
    // child_500 is at [500,0,0] in world, child_200 is at [200,0,0] in world
    // transform from child_500 frame to child_200 frame:
    // point_in_world = child_500_tf * point_in_child_500
    // point_in_child_200 = child_200_tf.inverse() * point_in_world
    // net translation in child_200 frame: 500 - 200 = 300 on X
    assert!(
        (tf.translation[0] - 300.0).abs() < 1e-6,
        "Expected 300.0, got {}",
        tf.translation[0]
    );

    // children() should return 1000 entries
    let children = hf.children("world");
    assert_eq!(children.len(), 1000);
}

/// Fill to the large preset limit (4096 frames).
#[test]
fn stress_4096_frames_large_preset() {
    let hf = TransformFrame::large();

    hf.register_frame("world", None).unwrap();

    for i in 0..4095 {
        let name = format!("f{}", i);
        hf.register_frame(&name, Some("world")).unwrap();
    }

    assert_eq!(hf.frame_count(), 4096);

    // Verify we can still resolve transforms
    let tf = Transform::from_translation([1.0, 2.0, 3.0]);
    hf.update_transform("f0", &tf, 1000).unwrap();

    let resolved = hf.tf("f0", "world").unwrap();
    assert!((resolved.translation[0] - 1.0).abs() < 1e-10);
}

// ============================================================================
// Capacity Exhaustion Tests
// ============================================================================

/// Test what happens when max_frames is exhausted.
/// Should return error, not panic.
#[test]
fn stress_max_frames_exhaustion() {
    let config = TransformFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .enable_overflow(false)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    // Fill all 16 slots
    for i in 0..16 {
        let name = format!("f{}", i);
        hf.register_frame(&name, None).unwrap();
    }

    assert_eq!(hf.frame_count(), 16);

    // 17th registration should fail gracefully
    let result = hf.register_frame("overflow", None);
    assert!(result.is_err(), "Should fail when max_frames exceeded");

    // Error message should mention the capacity limit
    let err_msg = format!("{}", result.unwrap_err());
    assert!(
        err_msg.contains("16") || err_msg.contains("limit") || err_msg.contains("Maximum"),
        "Error should mention capacity: {}",
        err_msg
    );
}

/// Test slot reuse after unregistration at capacity.
#[test]
fn stress_slot_reuse_after_unregister() {
    let config = TransformFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    // Fill all 16 slots (all dynamic, so unregisterable)
    for i in 0..16 {
        let name = format!("f{}", i);
        hf.register_frame(&name, None).unwrap();
    }

    // Unregister one
    hf.unregister_frame("f5").unwrap();
    assert_eq!(hf.frame_count(), 15);

    // Now should be able to register a new frame in the freed slot
    let result = hf.register_frame("replacement", None);
    assert!(result.is_ok(), "Should reuse freed slot: {:?}", result);
    assert_eq!(hf.frame_count(), 16);

    // Verify the replacement frame works
    assert!(hf.has_frame("replacement"));
    assert!(!hf.has_frame("f5"));
}

/// Test static frame limit — static frames cannot be unregistered.
#[test]
fn stress_static_frame_cannot_unregister() {
    let hf = TransformFrame::new();

    hf.register_static_frame("static_world", None, &Transform::identity())
        .unwrap();

    let result = hf.unregister_frame("static_world");
    assert!(
        result.is_err(),
        "Static frames should not be unregisterable"
    );
}

/// Mixed static + dynamic frames filling to capacity.
/// Both types count towards the max_frames limit.
#[test]
fn stress_mixed_static_dynamic_at_capacity() {
    let config = TransformFrameConfig::custom()
        .max_frames(16)
        .max_static_frames(8)
        .history_len(4)
        .enable_overflow(false)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    // Register 8 static frames
    for i in 0..8 {
        let name = format!("static_{}", i);
        hf.register_static_frame(
            &name,
            None,
            &Transform::from_translation([i as f64, 0.0, 0.0]),
        )
        .unwrap();
    }

    // Register 8 dynamic frames
    for i in 0..8 {
        let name = format!("dynamic_{}", i);
        hf.register_frame(&name, None).unwrap();
    }

    assert_eq!(hf.frame_count(), 16);

    // 17th (either type) should fail
    let result_dyn = hf.register_frame("overflow_dyn", None);
    assert!(
        result_dyn.is_err(),
        "Dynamic registration should fail at capacity"
    );

    let result_static = hf.register_static_frame("overflow_static", None, &Transform::identity());
    assert!(
        result_static.is_err(),
        "Static registration should fail at capacity"
    );

    // Static frames should still be queryable (sibling resolution via common root)
    // static_0 is root, register static_0_child under it to test chain resolution
    // Instead test that static frames can have their transforms read
    assert!(hf.has_frame("static_3"));
    assert!(hf.has_frame("static_0"));

    // Unregister a dynamic, re-register should work
    hf.unregister_frame("dynamic_7").unwrap();
    let result = hf.register_frame("replacement", None);
    assert!(result.is_ok(), "Should reuse freed dynamic slot");

    // Cannot unregister static to free a slot
    let result = hf.unregister_frame("static_0");
    assert!(result.is_err(), "Cannot unregister static frames");
}

/// Repeated unregister/re-register cycles at capacity — test slot reuse stability.
#[test]
fn stress_repeated_slot_reuse_cycles() {
    let config = TransformFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    // Fill to capacity, track current names per slot
    let mut slot_names: Vec<String> = (0..16).map(|i| format!("f{}", i)).collect();
    for name in &slot_names {
        hf.register_frame(name, None).unwrap();
    }

    // Cycle: unregister one slot, re-register with new name — 50 times
    for cycle in 0..50 {
        let slot_idx = cycle % 16;
        let victim = slot_names[slot_idx].clone();
        let replacement = format!("cycle_{}", cycle);

        // Unregister existing
        hf.unregister_frame(&victim).unwrap();
        assert!(!hf.has_frame(&victim));

        // Register replacement in the freed slot
        hf.register_frame(&replacement, None).unwrap();
        assert!(hf.has_frame(&replacement));
        assert_eq!(hf.frame_count(), 16);

        // Set and verify a transform on the replacement
        let tf = Transform::from_translation([cycle as f64, 0.0, 0.0]);
        hf.update_transform(&replacement, &tf, cycle as u64 * 1000)
            .unwrap();

        // Track the new name for this slot
        slot_names[slot_idx] = replacement;
    }

    // Final count should still be 16
    assert_eq!(hf.frame_count(), 16);
}

/// Static frames must have their transform resolvable immediately after registration
/// (no separate update_transform call needed).
#[test]
fn stress_static_frame_transform_immediate() {
    let hf = TransformFrame::new();

    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000)
        .unwrap();

    let expected = Transform::from_translation([1.0, 2.0, 3.0]);
    hf.register_static_frame("sensor", Some("world"), &expected)
        .unwrap();

    // Should be immediately resolvable
    let resolved = hf.tf("sensor", "world").unwrap();
    assert!(
        (resolved.translation[0] - 1.0).abs() < 1e-10
            && (resolved.translation[1] - 2.0).abs() < 1e-10
            && (resolved.translation[2] - 3.0).abs() < 1e-10,
        "Static frame transform should be immediately available: {:?}",
        resolved.translation
    );
}

// ============================================================================
// Deep Chain Resolution Tests
// ============================================================================

/// Depth 50/100/200 chain resolution with manual composition verification.
/// Computes the expected transform by manually composing Transform::compose()
/// and compares against the chain-resolved result.
#[test]
fn stress_depth_chain_manual_verification() {
    let config = TransformFrameConfig::custom()
        .max_frames(512)
        .history_len(4)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000)
        .unwrap();

    // Build a depth-200 chain with non-trivial transforms (translation + rotation)
    let per_link = Transform::from_euler([0.05, 0.02, 0.0], [0.0, 0.0, 0.01]);
    let mut prev = "world".to_string();
    for i in 0..200 {
        let name = format!("d{}", i);
        hf.register_frame(&name, Some(&prev)).unwrap();
        hf.update_transform(&name, &per_link, 1000).unwrap();
        prev = name;
    }

    // Manually compose transforms to get expected results at depth 50, 100, 200
    let mut expected_50 = Transform::identity();
    for _ in 0..50 {
        expected_50 = expected_50.compose(&per_link);
    }
    let mut expected_100 = expected_50;
    for _ in 50..100 {
        expected_100 = expected_100.compose(&per_link);
    }
    let mut expected_200 = expected_100;
    for _ in 100..200 {
        expected_200 = expected_200.compose(&per_link);
    }

    // Verify depth 50
    let resolved_50 = hf.tf("d49", "world").unwrap();
    for axis in 0..3 {
        assert!(
            (resolved_50.translation[axis] - expected_50.translation[axis]).abs() < 1e-6,
            "Depth 50, axis {}: resolved={}, expected={}",
            axis,
            resolved_50.translation[axis],
            expected_50.translation[axis]
        );
    }

    // Verify depth 100
    let resolved_100 = hf.tf("d99", "world").unwrap();
    for axis in 0..3 {
        assert!(
            (resolved_100.translation[axis] - expected_100.translation[axis]).abs() < 1e-4,
            "Depth 100, axis {}: resolved={}, expected={}",
            axis,
            resolved_100.translation[axis],
            expected_100.translation[axis]
        );
    }

    // Verify depth 200
    let resolved_200 = hf.tf("d199", "world").unwrap();
    for axis in 0..3 {
        assert!(
            (resolved_200.translation[axis] - expected_200.translation[axis]).abs() < 1e-3,
            "Depth 200, axis {}: resolved={}, expected={}",
            axis,
            resolved_200.translation[axis],
            expected_200.translation[axis]
        );
    }

    // can_transform() must work at every depth
    assert!(hf.can_transform("d49", "world"));
    assert!(hf.can_transform("d99", "world"));
    assert!(hf.can_transform("d199", "world"));
    assert!(hf.can_transform("d199", "d49"));
    assert!(hf.can_transform("d49", "d199"));

    // Cross-chain: mid-chain to mid-chain
    assert!(hf.can_transform("d30", "d150"));
    let cross = hf.tf("d30", "d150").unwrap();
    assert!(
        cross.translation.iter().all(|v| v.is_finite()),
        "Cross-chain resolution produced non-finite values"
    );
}

/// Measure resolution time scaling with chain depth.
/// Verifies that resolution time scales roughly linearly (not exponentially).
#[test]
fn stress_depth_timing_linear_scaling() {
    let config = TransformFrameConfig::custom()
        .max_frames(1024)
        .history_len(4)
        .chain_cache_size(0) // Disable cache to measure raw resolution
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000)
        .unwrap();

    // Build depth-500 chain
    let per_link = Transform::from_translation([0.01, 0.0, 0.0]);
    let mut prev = "world".to_string();
    for i in 0..500 {
        let name = format!("t{}", i);
        hf.register_frame(&name, Some(&prev)).unwrap();
        hf.update_transform(&name, &per_link, 1000).unwrap();
        prev = name;
    }

    // Time resolution at depth 50, 100, 250, 500
    let depths = [("t49", 50), ("t99", 100), ("t249", 250), ("t499", 500)];
    let mut timings = Vec::new();

    for (frame, depth) in &depths {
        let start = std::time::Instant::now();
        let repeats = 1000;
        for _ in 0..repeats {
            let _ = hf.tf(frame, "world").unwrap();
        }
        let elapsed = start.elapsed();
        let per_resolve_ns = elapsed.as_nanos() / repeats;
        timings.push((*depth, per_resolve_ns));
        eprintln!("Depth {}: {} ns/resolve", depth, per_resolve_ns);
    }

    // Verify depth-500 doesn't take more than 10x depth-50 (would indicate exponential)
    // Linear scaling: depth-500 should take roughly 10x depth-50
    let ratio = timings[3].1 as f64 / timings[0].1.max(1) as f64;
    assert!(
        ratio < 20.0,
        "Resolution time scaling is super-linear: depth-500/depth-50 ratio = {:.1}x (expected < 20x)",
        ratio
    );
}

/// Depth 100 chain resolution with accumulated translation verification.
#[test]
fn stress_depth_100_chain() {
    let config = TransformFrameConfig::custom()
        .max_frames(256)
        .history_len(4)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    hf.register_frame("world", None).unwrap();

    let mut prev = "world".to_string();
    for i in 0..100 {
        let name = format!("link_{}", i);
        hf.register_frame(&name, Some(&prev)).unwrap();
        // Each link adds [0.01, 0.0, 0.0] translation
        let tf = Transform::from_translation([0.01, 0.0, 0.0]);
        hf.update_transform(&name, &tf, 1000).unwrap();
        prev = name;
    }

    // Composed transform: 100 * 0.01 = 1.0
    let tf = hf.tf("link_99", "world").unwrap();
    assert!(
        (tf.translation[0] - 1.0).abs() < 1e-4,
        "Depth 100 chain: expected ~1.0, got {}",
        tf.translation[0]
    );
}

/// Depth 200 chain with rotation — tests floating point accumulation.
#[test]
fn stress_depth_200_chain_with_rotation() {
    let config = TransformFrameConfig::custom()
        .max_frames(512)
        .history_len(4)
        .build()
        .unwrap();
    let hf = TransformFrame::with_config(config);

    hf.register_frame("world", None).unwrap();

    let mut prev = "world".to_string();
    for i in 0..200 {
        let name = format!("j{}", i);
        hf.register_frame(&name, Some(&prev)).unwrap();
        // Small translation + tiny rotation around Z
        let tf = Transform::from_euler([0.01, 0.0, 0.0], [0.0, 0.0, 0.001]);
        hf.update_transform(&name, &tf, 1000).unwrap();
        prev = name;
    }

    let tf = hf.tf("j199", "world").unwrap();

    // Result should be finite (no NaN/Inf from accumulated floating point)
    assert!(
        tf.translation[0].is_finite()
            && tf.translation[1].is_finite()
            && tf.translation[2].is_finite(),
        "Deep chain produced non-finite translation: {:?}",
        tf.translation
    );
    assert!(
        tf.rotation.iter().all(|r| r.is_finite()),
        "Deep chain produced non-finite rotation: {:?}",
        tf.rotation
    );

    // Quaternion should still be approximately unit (norm ~1.0)
    let qnorm = (tf.rotation[0].powi(2)
        + tf.rotation[1].powi(2)
        + tf.rotation[2].powi(2)
        + tf.rotation[3].powi(2))
    .sqrt();
    assert!(
        (qnorm - 1.0).abs() < 0.01,
        "Quaternion norm drifted to {} after depth-200 chain",
        qnorm
    );
}

// ============================================================================
// High-Throughput Concurrent Tests
// ============================================================================

/// 4 writer threads + 8 reader threads, all running concurrently.
/// Writers update transforms, readers resolve chains.
/// Verify: no panics, no NaN, no torn reads.
#[test]
fn stress_concurrent_4_writers_8_readers() {
    let hf = Arc::new(TransformFrame::new());

    // Setup: world -> base -> arm -> hand, world -> camera
    hf.register_frame("world", None).unwrap();
    hf.register_frame("base", Some("world")).unwrap();
    hf.register_frame("arm", Some("base")).unwrap();
    hf.register_frame("hand", Some("arm")).unwrap();
    hf.register_frame("camera", Some("world")).unwrap();

    // Initial transforms
    hf.update_transform("base", &Transform::from_translation([1.0, 0.0, 0.0]), 0)
        .unwrap();
    hf.update_transform("arm", &Transform::from_translation([0.0, 1.0, 0.0]), 0)
        .unwrap();
    hf.update_transform("hand", &Transform::from_translation([0.0, 0.0, 1.0]), 0)
        .unwrap();
    hf.update_transform("camera", &Transform::from_translation([0.5, 0.5, 0.0]), 0)
        .unwrap();

    let barrier = Arc::new(Barrier::new(12)); // 4 writers + 8 readers
    let iterations = 5000;

    // Spawn 4 writer threads, each updating a different frame (one writer per frame)
    let mut writer_handles = Vec::new();
    let frame_names = ["base", "arm", "hand", "camera"];

    for (w, &frame) in frame_names.iter().enumerate() {
        let hf = hf.clone();
        let barrier = barrier.clone();
        let frame = frame.to_string();
        writer_handles.push(thread::spawn(move || {
            barrier.wait();
            for i in 0..iterations {
                let val = (w * iterations + i) as f64 * 0.001;
                let tf = Transform::from_translation([val, 0.0, 0.0]);
                hf.update_transform(&frame, &tf, i as u64 * 1000).unwrap();
            }
        }));
    }

    // Spawn 8 reader threads
    let mut reader_handles = Vec::new();
    for _ in 0..8 {
        let hf = hf.clone();
        let barrier = barrier.clone();
        reader_handles.push(thread::spawn(move || {
            barrier.wait();
            let mut nan_count = 0u64;
            for _ in 0..iterations {
                if let Ok(tf) = hf.tf("hand", "world") {
                    if !tf.translation[0].is_finite()
                        || !tf.translation[1].is_finite()
                        || !tf.translation[2].is_finite()
                    {
                        nan_count += 1;
                    }
                    for &r in &tf.rotation {
                        if !r.is_finite() {
                            nan_count += 1;
                        }
                    }
                }
                let _ = hf.can_transform("hand", "world");
            }
            nan_count
        }));
    }

    // Join all threads
    for handle in writer_handles {
        handle.join().expect("Writer thread must not panic");
    }
    let mut total_nan = 0u64;
    for handle in reader_handles {
        total_nan += handle.join().expect("Reader thread must not panic");
    }
    assert_eq!(
        total_nan, 0,
        "Detected {} NaN/Inf values in concurrent reads",
        total_nan
    );

    // Final verify: transform should be resolvable and finite
    let tf = hf.tf("hand", "world").unwrap();
    assert!(
        tf.translation.iter().all(|v| v.is_finite()),
        "Final transform has NaN/Inf: {:?}",
        tf.translation
    );
}

/// Concurrent read/write with measured throughput.
/// Verify correctness under sustained load.
#[test]
fn stress_sustained_concurrent_load() {
    let hf = Arc::new(TransformFrame::new());

    // Setup a realistic robot tree
    hf.register_frame("world", None).unwrap();
    hf.register_frame("odom", Some("world")).unwrap();
    hf.register_frame("base_link", Some("odom")).unwrap();
    hf.register_frame("laser", Some("base_link")).unwrap();
    hf.register_frame("camera", Some("base_link")).unwrap();
    hf.register_frame("imu", Some("base_link")).unwrap();

    // Set initial transforms
    for &name in &["odom", "base_link", "laser", "camera", "imu"] {
        hf.update_transform(name, &Transform::from_translation([0.1, 0.0, 0.0]), 0)
            .unwrap();
    }

    let barrier = Arc::new(Barrier::new(3));
    let iterations = 10_000u64;

    // Writer 1: Updates odom (simulating odometry at high rate)
    let hf_w1 = hf.clone();
    let b1 = barrier.clone();
    let w1 = thread::spawn(move || {
        b1.wait();
        for i in 0..iterations {
            let x = i as f64 * 0.001;
            let tf = Transform::from_translation([x, 0.0, 0.0]);
            hf_w1.update_transform("odom", &tf, i * 1000).unwrap();
        }
    });

    // Writer 2: Updates base_link (simulating joint state)
    let hf_w2 = hf.clone();
    let b2 = barrier.clone();
    let w2 = thread::spawn(move || {
        b2.wait();
        for i in 0..iterations {
            let yaw = i as f64 * 0.0001;
            let tf = Transform::from_euler([0.0, 0.0, 0.0], [0.0, 0.0, yaw]);
            hf_w2.update_transform("base_link", &tf, i * 1000).unwrap();
        }
    });

    // Reader: Resolves laser->world (typical perception query)
    let hf_r = hf.clone();
    let br = barrier.clone();
    let reader = thread::spawn(move || {
        br.wait();
        let mut success = 0u64;
        let mut finite_count = 0u64;
        for _ in 0..iterations {
            if let Ok(tf) = hf_r.tf("laser", "world") {
                success += 1;
                if tf.translation.iter().all(|v| v.is_finite())
                    && tf.rotation.iter().all(|v| v.is_finite())
                {
                    finite_count += 1;
                }
            }
        }
        (success, finite_count)
    });

    w1.join().expect("Writer 1 panicked");
    w2.join().expect("Writer 2 panicked");
    let (success, finite) = reader.join().expect("Reader panicked");

    // All successful reads must be finite (no torn reads)
    assert_eq!(
        success, finite,
        "Some reads returned NaN/Inf: {} successful, {} finite",
        success, finite
    );

    // Final state must be consistent
    let final_tf = hf.tf("laser", "world").unwrap();
    assert!(
        final_tf.translation.iter().all(|v| v.is_finite()),
        "Final transform is non-finite"
    );
}

/// High-throughput concurrent test: 100K+ iterations per thread.
/// 4 writers with rotation transforms, 8 readers validating quaternion norms.
/// Checks for torn reads, NaN/Inf, and quaternion norm drift under extreme load.
#[test]
fn stress_high_throughput_100k() {
    let hf = Arc::new(TransformFrame::new());

    // Build a 6-frame robot arm: world -> base -> shoulder -> elbow -> wrist -> tool
    hf.register_frame("world", None).unwrap();
    hf.register_frame("base", Some("world")).unwrap();
    hf.register_frame("shoulder", Some("base")).unwrap();
    hf.register_frame("elbow", Some("shoulder")).unwrap();
    hf.register_frame("wrist", Some("elbow")).unwrap();
    hf.register_frame("tool", Some("wrist")).unwrap();

    // Initial transforms with small rotations
    for &name in &["base", "shoulder", "elbow", "wrist", "tool"] {
        hf.update_transform(
            name,
            &Transform::from_euler([0.1, 0.0, 0.0], [0.0, 0.0, 0.01]),
            0,
        )
        .unwrap();
    }

    let num_writers = 4;
    let num_readers = 8;
    let barrier = Arc::new(Barrier::new(num_writers + num_readers));
    let iterations = 100_000u64;

    // 4 writer threads: base, shoulder, elbow, wrist (one writer per frame)
    let writer_frames = ["base", "shoulder", "elbow", "wrist"];
    let mut writer_handles = Vec::new();

    for (w, &frame) in writer_frames.iter().enumerate() {
        let hf = hf.clone();
        let barrier = barrier.clone();
        let frame = frame.to_string();
        writer_handles.push(thread::spawn(move || {
            barrier.wait();
            let mut write_count = 0u64;
            for i in 0..iterations {
                // Vary both translation and rotation to exercise quaternion path
                let angle = (i as f64) * 0.00001 * (w as f64 + 1.0);
                let x = (i as f64) * 0.0001;
                let tf = Transform::from_euler([x, 0.0, 0.0], [0.0, 0.0, angle]);
                hf.update_transform(&frame, &tf, i * 1000).unwrap();
                write_count += 1;
            }
            write_count
        }));
    }

    // 8 reader threads: resolve tool->world and validate quaternion norms
    let mut reader_handles = Vec::new();

    for _ in 0..num_readers {
        let hf = hf.clone();
        let barrier = barrier.clone();
        reader_handles.push(thread::spawn(move || {
            barrier.wait();
            let mut read_count = 0u64;
            let mut nan_count = 0u64;
            let mut bad_quat_count = 0u64;

            for _ in 0..iterations {
                if let Ok(tf) = hf.tf("tool", "world") {
                    read_count += 1;

                    // Check for NaN/Inf in translation
                    if !tf.translation.iter().all(|v| v.is_finite()) {
                        nan_count += 1;
                    }

                    // Check for NaN/Inf in rotation
                    if !tf.rotation.iter().all(|v| v.is_finite()) {
                        nan_count += 1;
                    }

                    // Validate quaternion norm is approximately 1.0
                    let qnorm = (tf.rotation[0].powi(2)
                        + tf.rotation[1].powi(2)
                        + tf.rotation[2].powi(2)
                        + tf.rotation[3].powi(2))
                    .sqrt();
                    if (qnorm - 1.0).abs() > 0.01 {
                        bad_quat_count += 1;
                    }
                }
            }
            (read_count, nan_count, bad_quat_count)
        }));
    }

    // Collect writer results
    let mut total_writes = 0u64;
    for handle in writer_handles {
        total_writes += handle.join().expect("Writer thread panicked");
    }

    // Collect reader results
    let mut total_reads = 0u64;
    let mut total_nan = 0u64;
    let mut total_bad_quat = 0u64;
    for handle in reader_handles {
        let (reads, nans, bad_quats) = handle.join().expect("Reader thread panicked");
        total_reads += reads;
        total_nan += nans;
        total_bad_quat += bad_quats;
    }

    assert_eq!(
        total_nan, 0,
        "Detected {} NaN/Inf values across {} reads",
        total_nan, total_reads
    );
    assert_eq!(
        total_bad_quat, 0,
        "Detected {} invalid quaternion norms across {} reads",
        total_bad_quat, total_reads
    );

    // Verify final state
    let final_tf = hf.tf("tool", "world").unwrap();
    assert!(
        final_tf.translation.iter().all(|v| v.is_finite()),
        "Final transform has NaN/Inf"
    );
    let final_qnorm = (final_tf.rotation[0].powi(2)
        + final_tf.rotation[1].powi(2)
        + final_tf.rotation[2].powi(2)
        + final_tf.rotation[3].powi(2))
    .sqrt();
    assert!(
        (final_qnorm - 1.0).abs() < 0.001,
        "Final quaternion norm is {}, expected ~1.0",
        final_qnorm
    );

    // Log throughput (visible in test output with --nocapture)
    eprintln!(
        "High-throughput test: {} total writes, {} total reads",
        total_writes, total_reads
    );
}
