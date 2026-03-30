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

// ============================================================================
// Test: 16 writers + 32 readers — scaled up stress
// ============================================================================

#[test]
fn test_16_writers_32_readers_no_corruption() {
    let config = TransformFrameConfig::large();
    let tf = Arc::new(TransformFrame::with_config(config));

    tf.register_frame("world", None).unwrap();
    for i in 0..16 {
        tf.register_frame(&format!("big_s{}", i), Some("world"))
            .unwrap();
        tf.update_transform(
            &format!("big_s{}", i),
            &Transform::from_translation([i as f64, 0.0, 0.0]),
            0,
        )
        .unwrap();
    }

    let running = Arc::new(AtomicBool::new(true));
    let corruptions = Arc::new(AtomicU64::new(0));
    let total_reads = Arc::new(AtomicU64::new(0));
    let mut handles = Vec::new();

    // 16 writers
    for w in 0..16 {
        let tf = tf.clone();
        let running = running.clone();
        handles.push(std::thread::spawn(move || {
            let name = format!("big_s{}", w);
            let mut ts = 1u64;
            while running.load(Ordering::Relaxed) {
                let x = (ts as f64) * 0.001 + (w as f64);
                let y = (ts as f64) * 0.0005;
                let _ = tf.update_transform(&name, &Transform::from_translation([x, y, 0.0]), ts);
                ts += 1;
            }
        }));
    }

    // 32 readers
    for r in 0..32 {
        let tf = tf.clone();
        let running = running.clone();
        let corruptions = corruptions.clone();
        let reads = total_reads.clone();
        handles.push(std::thread::spawn(move || {
            let src = format!("big_s{}", r % 16);
            while running.load(Ordering::Relaxed) {
                if let Ok(result) = tf.tf(&src, "world") {
                    reads.fetch_add(1, Ordering::Relaxed);
                    if !result.translation[0].is_finite()
                        || !result.translation[1].is_finite()
                        || !result.translation[2].is_finite()
                    {
                        corruptions.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }));
    }

    // Run for 2 seconds
    std::thread::sleep(Duration::from_secs(2));
    running.store(false, Ordering::SeqCst);
    for h in handles {
        h.join().unwrap();
    }

    let total = total_reads.load(Ordering::SeqCst);
    let corrupt = corruptions.load(Ordering::SeqCst);

    assert!(
        total > 1000,
        "Should complete >1000 reads in 2s, got {}",
        total
    );
    assert_eq!(corrupt, 0, "Zero corruptions in {} reads (16W+32R)", total);
}

// ============================================================================
// Test: Chain lookup during concurrent writes — A->B->C->D
// ============================================================================

#[test]
fn test_chain_lookup_during_writes() {
    let config = TransformFrameConfig::medium();
    let tf = Arc::new(TransformFrame::with_config(config));

    // Set up chain: world -> A -> B -> C -> D
    tf.register_frame("chain_world", None).unwrap();
    for (name, parent) in [
        ("cA", "chain_world"),
        ("cB", "cA"),
        ("cC", "cB"),
        ("cD", "cC"),
    ] {
        tf.register_frame(name, Some(parent)).unwrap();
        tf.update_transform(name, &Transform::from_translation([1.0, 0.0, 0.0]), 0)
            .unwrap();
    }

    let running = Arc::new(AtomicBool::new(true));
    let corruptions = Arc::new(AtomicU64::new(0));
    let total_reads = Arc::new(AtomicU64::new(0));

    // Writer: continuously updates B->C transform
    let tf_w = tf.clone();
    let r_w = running.clone();
    let writer = std::thread::spawn(move || {
        let mut ts = 1u64;
        while r_w.load(Ordering::Relaxed) {
            let x = 1.0 + (ts as f64 % 100.0) * 0.01;
            let _ = tf_w.update_transform("cB", &Transform::from_translation([x, 0.0, 0.0]), ts);
            ts += 1;
        }
    });

    // Reader: continuously looks up world -> D (4-link chain)
    let tf_r = tf.clone();
    let r_r = running.clone();
    let c = corruptions.clone();
    let reads = total_reads.clone();
    let reader = std::thread::spawn(move || {
        while r_r.load(Ordering::Relaxed) {
            if let Ok(result) = tf_r.tf("cD", "chain_world") {
                reads.fetch_add(1, Ordering::Relaxed);
                // Chain: world->A(1.0) + A->B(varying) + B->C(1.0) + C->D(1.0) = ~4.0
                if !result.translation[0].is_finite() {
                    c.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    });

    std::thread::sleep(Duration::from_secs(1));
    running.store(false, Ordering::SeqCst);
    writer.join().unwrap();
    reader.join().unwrap();

    let total = total_reads.load(Ordering::SeqCst);
    let corrupt = corruptions.load(Ordering::SeqCst);

    assert!(
        total > 100,
        "Should complete >100 chain lookups, got {}",
        total
    );
    assert_eq!(
        corrupt, 0,
        "Zero corruption in {} chain lookups during concurrent writes",
        total
    );
}
