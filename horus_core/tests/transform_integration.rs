//! TransformFrame integration tests.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test transform_integration -- --ignored --nocapture

use horus_tf::{Transform, TransformFrame};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ════════════════════════════════════════════════════════════════════════
// TEST 1: 3-hop chain resolution (world→base→arm→tool)
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn chain_world_base_arm_tool() {
    let tf = TransformFrame::medium();

    tf.register_frame("world", None).unwrap();
    tf.register_frame("base_link", Some("world")).unwrap();
    tf.register_frame("arm_link", Some("base_link")).unwrap();
    tf.register_frame("tool_link", Some("arm_link")).unwrap();

    // Set translations: world→base [1,0,0], base→arm [0,1,0], arm→tool [0,0,1]
    tf.update_transform("base_link", &Transform::from_translation([1.0, 0.0, 0.0]), horus_library::transform_frame::timestamp_now()).unwrap();
    tf.update_transform("arm_link", &Transform::from_translation([0.0, 1.0, 0.0]), horus_library::transform_frame::timestamp_now()).unwrap();
    tf.update_transform("tool_link", &Transform::from_translation([0.0, 0.0, 1.0]), horus_library::transform_frame::timestamp_now()).unwrap();

    // Lookup tool→world gives the tool's position in world frame = [1,1,1]
    // tf(src, dst) convention: returns transform FROM src TO dst
    let result = tf.tf("tool_link", "world").unwrap();
    println!("tool→world translation: {:?}", result.translation);

    assert!((result.translation[0] - 1.0).abs() < 0.01, "x: {}", result.translation[0]);
    assert!((result.translation[1] - 1.0).abs() < 0.01, "y: {}", result.translation[1]);
    assert!((result.translation[2] - 1.0).abs() < 0.01, "z: {}", result.translation[2]);

    // Also test transform_point (transform origin of tool into world frame)
    let point = tf.transform_point("tool_link", "world", [0.0, 0.0, 0.0]).unwrap();
    println!("Point [0,0,0] in world = {:?} in tool frame", point);

    println!("✓ chain_world_base_arm_tool — 3-hop composition correct");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: 20-frame deep hierarchy
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn deep_hierarchy_20_frames() {
    let tf = TransformFrame::medium();

    tf.register_frame("link_0", None).unwrap();
    for i in 1..20 {
        tf.register_frame(
            &format!("link_{}", i),
            Some(&format!("link_{}", i - 1)),
        ).unwrap();
        tf.update_transform(
            &format!("link_{}", i),
            &Transform::from_translation([0.1, 0.0, 0.0]),
            horus_library::transform_frame::timestamp_now(),
        ).unwrap();
    }

    let result = tf.tf("link_19", "link_0").unwrap();
    let expected_x = 19.0 * 0.1; // 1.9m
    println!("link_19→link_0 translation: {:?} (expected x={:.1})", result.translation, expected_x);

    assert!((result.translation[0] - expected_x).abs() < 0.01,
        "20-hop chain: expected x={}, got {}", expected_x, result.translation[0]);

    println!("✓ deep_hierarchy_20_frames — chain resolves correctly");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: Concurrent writers (2 threads updating different frames)
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn concurrent_writers_no_corruption() {
    let tf = Arc::new(TransformFrame::medium());

    tf.register_frame("base", None).unwrap();
    tf.register_frame("left_wheel", Some("base")).unwrap();
    tf.register_frame("right_wheel", Some("base")).unwrap();
    tf.update_transform("left_wheel", &Transform::from_translation([-0.15, 0.0, 0.0]), horus_library::transform_frame::timestamp_now()).unwrap();
    tf.update_transform("right_wheel", &Transform::from_translation([0.15, 0.0, 0.0]), horus_library::transform_frame::timestamp_now()).unwrap();

    let running = Arc::new(AtomicBool::new(true));
    let mut corrupted = Arc::new(std::sync::atomic::AtomicU64::new(0));

    // Writer 1: updates left_wheel rapidly
    let tf1 = tf.clone();
    let r1 = running.clone();
    let h1 = std::thread::spawn(move || {
        let mut i = 0u64;
        while r1.load(Ordering::Relaxed) {
            let x = -0.15 + (i as f64 * 0.001).sin() * 0.01;
            let _ = tf1.update_transform("left_wheel", &Transform::from_translation([x, 0.0, 0.0]), horus_library::transform_frame::timestamp_now());
            i += 1;
        }
        i
    });

    // Writer 2: updates right_wheel rapidly
    let tf2 = tf.clone();
    let r2 = running.clone();
    let h2 = std::thread::spawn(move || {
        let mut i = 0u64;
        while r2.load(Ordering::Relaxed) {
            let x = 0.15 + (i as f64 * 0.001).cos() * 0.01;
            let _ = tf2.update_transform("right_wheel", &Transform::from_translation([x, 0.0, 0.0]), horus_library::transform_frame::timestamp_now());
            i += 1;
        }
        i
    });

    // Reader: queries both frames from main thread
    let mut reads = 0u64;
    let mut bad = 0u64;
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    while std::time::Instant::now() < deadline {
        if let Ok(result) = tf.tf("base", "left_wheel") {
            if !result.translation[0].is_finite() { bad += 1; }
            reads += 1;
        }
        if let Ok(result) = tf.tf("base", "right_wheel") {
            if !result.translation[0].is_finite() { bad += 1; }
            reads += 1;
        }
    }

    running.store(false, Ordering::Relaxed);
    let w1 = h1.join().unwrap();
    let w2 = h2.join().unwrap();

    println!("Concurrent: {} reads, {} writes, {} NaN/Inf", reads, w1 + w2, bad);
    assert_eq!(bad, 0, "Concurrent writes produced NaN/Inf in {} reads", bad);
    assert!(reads > 1000, "Should read >1000 transforms in 3s");
    println!("✓ concurrent_writers_no_corruption — {} reads, zero NaN/Inf", reads);
}

// ════════════════════════════════════════════════════════════════════════
// TEST 4: Cross-scheduler transform sharing
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn cross_scheduler_transform_sharing() {
    use horus_core::core::{DurationExt, Node};
    use horus_core::scheduling::Scheduler;

    cleanup_stale_shm();

    let tf = Arc::new(TransformFrame::medium());
    tf.register_frame("world", None).unwrap();
    tf.register_frame("robot", Some("world")).unwrap();

    struct TfWriterNode { tf: Arc<TransformFrame>, tick: u64 }
    impl Node for TfWriterNode {
        fn name(&self) -> &str { "tf_writer" }
        fn tick(&mut self) {
            let x = self.tick as f64 * 0.01;
            let _ = self.tf.update_transform("robot", &Transform::from_translation([x, 0.0, 0.0]), horus_library::transform_frame::timestamp_now());
            self.tick += 1;
        }
    }

    let tf_read_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let tf_valid = Arc::new(std::sync::atomic::AtomicU64::new(0));

    struct TfReaderNode {
        tf: Arc<TransformFrame>,
        read_count: Arc<std::sync::atomic::AtomicU64>,
        valid: Arc<std::sync::atomic::AtomicU64>,
    }
    impl Node for TfReaderNode {
        fn name(&self) -> &str { "tf_reader" }
        fn tick(&mut self) {
            if let Ok(result) = self.tf.tf("robot", "world") {
                self.read_count.fetch_add(1, Ordering::Relaxed);
                if result.translation[0].is_finite() && result.translation[0] >= 0.0 {
                    self.valid.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    let running = Arc::new(AtomicBool::new(true));

    // Sched1: writer at 200Hz
    let tf1 = tf.clone();
    let r1 = running.clone();
    let h1 = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(200_u64.hz());
        let _ = sched.add(TfWriterNode { tf: tf1, tick: 0 }).rate(200_u64.hz()).order(0).build();
        while r1.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(4));
        }
    });

    // Sched2: reader at 100Hz
    let tf2 = tf.clone();
    let rc = tf_read_count.clone();
    let vc = tf_valid.clone();
    let r2 = running.clone();
    let h2 = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        let _ = sched.add(TfReaderNode { tf: tf2, read_count: rc, valid: vc }).rate(100_u64.hz()).order(0).build();
        while r2.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    std::thread::sleep(Duration::from_secs(3));
    running.store(false, Ordering::Relaxed);
    h1.join().unwrap();
    h2.join().unwrap();

    let reads = tf_read_count.load(Ordering::Relaxed);
    let valid = tf_valid.load(Ordering::Relaxed);

    println!("Cross-scheduler TF: {} reads, {} valid ({:.0}%)", reads, valid, valid as f64 / reads.max(1) as f64 * 100.0);
    assert!(reads > 50, "Reader should get >50 transform lookups");
    assert_eq!(reads, valid, "All reads should return valid (finite, non-negative) transforms");
    println!("✓ cross_scheduler_transform_sharing — {} reads, 100% valid", reads);
}
