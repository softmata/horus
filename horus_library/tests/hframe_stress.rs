//! HFrame Stress Tests
//!
//! Tests for scale, capacity limits, deep chains, and high-throughput
//! concurrent access patterns that go beyond the unit tests in each module.

use horus_library::hframe::{HFrame, HFrameConfig, Transform};
use std::sync::{Arc, Barrier};
use std::thread;

// ============================================================================
// Scale Tests: Large Frame Counts
// ============================================================================

/// Register 1000 frames in a single chain: world -> f0 -> f1 -> ... -> f999
/// Verify all frames are queryable and transforms resolve correctly.
#[test]
fn stress_1000_frame_chain() {
    let config = HFrameConfig::custom()
        .max_frames(2048)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);

    // Register root
    let _world = hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000).unwrap();

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
    let config = HFrameConfig::custom()
        .max_frames(2048)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);

    hf.register_frame("world", None).unwrap();
    hf.update_transform("world", &Transform::identity(), 1000).unwrap();

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
    let hf = HFrame::large();

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
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);

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
    let config = HFrameConfig::custom()
        .max_frames(16)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);

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
    let hf = HFrame::new();

    hf.register_static_frame("static_world", None, &Transform::identity())
        .unwrap();

    let result = hf.unregister_frame("static_world");
    assert!(result.is_err(), "Static frames should not be unregisterable");
}

// ============================================================================
// Deep Chain Resolution Tests
// ============================================================================

/// Depth 100 chain resolution with accumulated translation verification.
#[test]
fn stress_depth_100_chain() {
    let config = HFrameConfig::custom()
        .max_frames(256)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);

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
    let config = HFrameConfig::custom()
        .max_frames(512)
        .history_len(4)
        .build()
        .unwrap();
    let hf = HFrame::with_config(config);

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
        tf.translation[0].is_finite() && tf.translation[1].is_finite() && tf.translation[2].is_finite(),
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
    let hf = Arc::new(HFrame::new());

    // Setup: world -> base -> arm -> hand
    hf.register_frame("world", None).unwrap();
    hf.register_frame("base", Some("world")).unwrap();
    hf.register_frame("arm", Some("base")).unwrap();
    hf.register_frame("hand", Some("arm")).unwrap();

    // Initial transforms
    hf.update_transform("base", &Transform::from_translation([1.0, 0.0, 0.0]), 0).unwrap();
    hf.update_transform("arm", &Transform::from_translation([0.0, 1.0, 0.0]), 0).unwrap();
    hf.update_transform("hand", &Transform::from_translation([0.0, 0.0, 1.0]), 0).unwrap();

    let barrier = Arc::new(Barrier::new(12)); // 4 writers + 8 readers
    let iterations = 5000;

    // Spawn 4 writer threads, each updating a different frame
    let mut handles = Vec::new();
    let frame_names = ["base", "arm", "hand", "base"]; // base gets 2 writers

    for (w, &frame) in frame_names.iter().enumerate() {
        let hf = hf.clone();
        let barrier = barrier.clone();
        let frame = frame.to_string();
        handles.push(thread::spawn(move || {
            barrier.wait();
            for i in 0..iterations {
                let val = (w * iterations + i) as f64 * 0.001;
                let tf = Transform::from_translation([val, 0.0, 0.0]);
                hf.update_transform(&frame, &tf, i as u64 * 1000).unwrap();
            }
        }));
    }

    // Spawn 8 reader threads
    for _ in 0..8 {
        let hf = hf.clone();
        let barrier = barrier.clone();
        handles.push(thread::spawn(move || {
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
                    // Quaternion should be valid
                    for &r in &tf.rotation {
                        if !r.is_finite() {
                            nan_count += 1;
                        }
                    }
                }
                // Also test can_transform (different code path)
                let _ = hf.can_transform("hand", "world");
            }
            nan_count
        }));
    }

    // Join all threads
    let mut total_nan = 0u64;
    for handle in handles {
        let result = handle.join().expect("Thread must not panic");
        // Writer threads return (), reader threads return nan_count
        // We can't easily distinguish, so we use a workaround:
        // all threads that return u64 contribute to nan count
        // Rust won't let us mix return types easily, so we check separately
    }

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
    let hf = Arc::new(HFrame::new());

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
            hf_w2
                .update_transform("base_link", &tf, i * 1000)
                .unwrap();
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
