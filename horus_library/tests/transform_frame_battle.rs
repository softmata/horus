// TransformFrame battle tests — scale beyond existing 16 stress tests.

use horus_library::transform_frame::{Transform, TransformFrame, TransformFrameConfig};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// ============================================================================
// Test: 8 writers + 16 readers — no NaN, no Inf, no corruption
// ============================================================================

#[test]
fn test_8_writers_16_readers_no_corruption() {
    let config = TransformFrameConfig::medium();
    let tf = Arc::new(TransformFrame::with_config(config));

    tf.register_frame("world", None).unwrap();
    for i in 0..8 {
        tf.register_frame(&format!("sensor_{}", i), Some("world"))
            .unwrap();
        tf.update_transform(
            &format!("sensor_{}", i),
            &Transform::from_translation([i as f64, 0.0, 0.0]),
            0,
        )
        .unwrap();
    }

    let running = Arc::new(AtomicBool::new(true));
    let corruptions = Arc::new(AtomicU64::new(0));
    let total_reads = Arc::new(AtomicU64::new(0));
    let mut handles = Vec::new();

    // 8 writers
    for w in 0..8 {
        let tf = tf.clone();
        let running = running.clone();
        handles.push(std::thread::spawn(move || {
            let name = format!("sensor_{}", w);
            let mut ts = 1u64;
            while running.load(Ordering::Relaxed) {
                let x = (ts as f64) * 0.001 + (w as f64);
                let _ = tf.update_transform(&name, &Transform::from_translation([x, 0.0, 0.0]), ts);
                ts += 1;
            }
        }));
    }

    // 16 readers
    for r in 0..16 {
        let tf = tf.clone();
        let running = running.clone();
        let corruptions = corruptions.clone();
        let reads = total_reads.clone();
        handles.push(std::thread::spawn(move || {
            let src = format!("sensor_{}", r % 8);
            while running.load(Ordering::Relaxed) {
                if let Ok(result) = tf.tf(&src, "world") {
                    reads.fetch_add(1, Ordering::Relaxed);
                    if !result.translation[0].is_finite() || !result.translation[1].is_finite() {
                        corruptions.fetch_add(1, Ordering::Relaxed);
                    }
                    let q = result.rotation;
                    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
                    if (norm - 1.0).abs() > 0.1 {
                        corruptions.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }));
    }

    std::thread::sleep(Duration::from_millis(500));
    running.store(false, Ordering::SeqCst);
    for h in handles {
        h.join().unwrap();
    }

    let total = total_reads.load(Ordering::SeqCst);
    let corrupt = corruptions.load(Ordering::SeqCst);

    assert!(total > 100, "Should complete >100 reads, got {}", total);
    assert_eq!(corrupt, 0, "Zero corruptions in {} reads", total);
}

// ============================================================================
// Test: Memory stable after register/unregister cycles
// ============================================================================

#[test]
fn test_memory_stable_after_cycles() {
    let config = TransformFrameConfig::medium();
    let tf = TransformFrame::with_config(config);
    tf.register_frame("world", None).unwrap();

    for cycle in 0..500 {
        let name = format!("cycle_{}", cycle % 50);
        let _ = tf.register_frame(&name, Some("world"));
        let _ = tf.unregister_frame(&name);
    }

    // Tree still works
    let fresh = tf.register_frame("fresh", Some("world"));
    assert!(fresh.is_ok(), "Registration after 500 cycles: {:?}", fresh);
    tf.update_transform("fresh", &Transform::from_translation([42.0, 0.0, 0.0]), 1)
        .unwrap();
    let result = tf.tf("fresh", "world");
    assert!(result.is_ok(), "Query after 500 cycles should work");
}

// ============================================================================
// Test: Deep chain 500 resolves in bounded time
// ============================================================================

#[test]
fn test_deep_chain_500_bounded_time() {
    let config = TransformFrameConfig::large();
    let tf = TransformFrame::with_config(config);

    tf.register_frame("d0", None).unwrap();
    for i in 1..=500 {
        tf.register_frame(&format!("d{}", i), Some(&format!("d{}", i - 1)))
            .unwrap();
        tf.update_transform(
            &format!("d{}", i),
            &Transform::from_translation([0.001, 0.0, 0.0]),
            i as u64,
        )
        .unwrap();
    }

    let start = Instant::now();
    let result = tf.tf("d500", "d0");
    let elapsed = start.elapsed();

    assert!(result.is_ok(), "Should resolve 500-deep chain");
    assert!(
        elapsed < Duration::from_millis(50),
        "Resolution took {:?}",
        elapsed
    );

    if let Ok(xform) = result {
        let expected = 500.0 * 0.001;
        assert!(
            (xform.translation[0] - expected).abs() < 0.01,
            "Expected x≈{}, got {}",
            expected,
            xform.translation[0]
        );
    }
}

// ============================================================================
// Test: Concurrent registration doesn't corrupt tree
// ============================================================================

#[test]
fn test_concurrent_registration() {
    let config = TransformFrameConfig::large();
    let tf = Arc::new(TransformFrame::with_config(config));
    tf.register_frame("world", None).unwrap();

    let mut handles = Vec::new();
    for t in 0..4 {
        let tf = tf.clone();
        handles.push(std::thread::spawn(move || {
            for i in 0..50 {
                let _ = tf.register_frame(&format!("t{}_f{}", t, i), Some("world"));
            }
        }));
    }
    for h in handles {
        h.join().unwrap();
    }

    let count = tf.frame_count();
    assert!(count >= 50, "Should register many frames, got {}", count);
    assert!(tf.has_frame("world"));
}
