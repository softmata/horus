//! HFrame Benchmarks
//!
//! Simple benchmarks to measure HFrame performance.
//! Run with: cargo test --release -p horus_library hframe_benchmark -- --nocapture --ignored

use super::*;
use std::time::Instant;

/// Number of iterations for benchmarks
const ITERATIONS: u64 = 100_000;

/// Benchmark transform lookup by ID
#[test]
#[ignore]
fn hframe_benchmark_lookup_by_id() {
    let hf = HFrame::new();

    // Setup: create a chain of frames
    let world = hf.register_frame("world", None).unwrap();
    let base = hf.register_frame("base_link", Some("world")).unwrap();
    let camera = hf.register_frame("camera", Some("base_link")).unwrap();

    // Update transforms
    hf.update_transform_by_id(base, &Transform::from_translation([1.0, 0.0, 0.0]), 1000);
    hf.update_transform_by_id(camera, &Transform::from_translation([0.0, 0.0, 0.5]), 1000);

    // Warm up
    for _ in 0..1000 {
        let _ = hf.core.resolve(camera, world);
    }

    // Benchmark
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = hf.core.resolve(camera, world);
    }
    let elapsed = start.elapsed();

    let ns_per_op = elapsed.as_nanos() / ITERATIONS as u128;
    println!("HFrame lookup by ID (depth 2): {} ns/op", ns_per_op);
    println!("  Total time: {:?} for {} ops", elapsed, ITERATIONS);
    println!(
        "  Throughput: {:.2} M ops/sec",
        ITERATIONS as f64 / elapsed.as_secs_f64() / 1_000_000.0
    );
}

/// Benchmark transform lookup by name
#[test]
#[ignore]
fn hframe_benchmark_lookup_by_name() {
    let hf = HFrame::new();

    // Setup
    hf.register_frame("world", None).unwrap();
    hf.register_frame("base_link", Some("world")).unwrap();
    hf.register_frame("camera", Some("base_link")).unwrap();

    hf.update_transform(
        "base_link",
        &Transform::from_translation([1.0, 0.0, 0.0]),
        1000,
    )
    .unwrap();
    hf.update_transform(
        "camera",
        &Transform::from_translation([0.0, 0.0, 0.5]),
        1000,
    )
    .unwrap();

    // Warm up
    for _ in 0..1000 {
        let _ = hf.tf("camera", "world");
    }

    // Benchmark
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = hf.tf("camera", "world");
    }
    let elapsed = start.elapsed();

    let ns_per_op = elapsed.as_nanos() / ITERATIONS as u128;
    println!("HFrame lookup by name (depth 2): {} ns/op", ns_per_op);
    println!("  Total time: {:?} for {} ops", elapsed, ITERATIONS);
    println!(
        "  Throughput: {:.2} M ops/sec",
        ITERATIONS as f64 / elapsed.as_secs_f64() / 1_000_000.0
    );
}

/// Benchmark transform update
#[test]
#[ignore]
fn hframe_benchmark_update() {
    let hf = HFrame::new();

    // Setup
    hf.register_frame("world", None).unwrap();
    let base = hf.register_frame("base_link", Some("world")).unwrap();

    let tf = Transform::from_translation([1.0, 0.0, 0.0]);

    // Warm up
    for i in 0..1000 {
        hf.update_transform_by_id(base, &tf, i);
    }

    // Benchmark
    let start = Instant::now();
    for i in 0..ITERATIONS {
        hf.update_transform_by_id(base, &tf, i);
    }
    let elapsed = start.elapsed();

    let ns_per_op = elapsed.as_nanos() / ITERATIONS as u128;
    println!("HFrame update by ID: {} ns/op", ns_per_op);
    println!("  Total time: {:?} for {} ops", elapsed, ITERATIONS);
    println!(
        "  Throughput: {:.2} M ops/sec",
        ITERATIONS as f64 / elapsed.as_secs_f64() / 1_000_000.0
    );
}

/// Benchmark frame registration
#[test]
#[ignore]
fn hframe_benchmark_register() {
    let count = 200; // Stay within default 256 frame limit

    let start = Instant::now();
    for _ in 0..100 {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        for i in 0..count {
            hf.register_frame(&format!("frame_{}", i), Some("world"))
                .unwrap();
        }
    }
    let elapsed = start.elapsed();

    let total_ops = 100 * (count + 1);
    let ns_per_op = elapsed.as_nanos() / total_ops as u128;
    println!("HFrame register frame: {} ns/op", ns_per_op);
    println!("  Registered {} frames per iteration", count + 1);
    println!("  Total time: {:?} for {} ops", elapsed, total_ops);
}

/// Benchmark deep chain lookup
#[test]
#[ignore]
fn hframe_benchmark_deep_chain() {
    let hf = HFrame::new();

    // Create a deep chain: world -> link_0 -> link_1 -> ... -> link_9
    hf.register_frame("world", None).unwrap();
    let mut parent = "world".to_string();
    for i in 0..10 {
        let name = format!("link_{}", i);
        hf.register_frame(&name, Some(&parent)).unwrap();
        hf.update_transform(&name, &Transform::from_translation([0.1, 0.0, 0.0]), 1000)
            .unwrap();
        parent = name;
    }

    // Cache frame IDs
    let world = hf.frame_id("world").unwrap();
    let end = hf.frame_id("link_9").unwrap();

    // Warm up
    for _ in 0..1000 {
        let _ = hf.core.resolve(end, world);
    }

    // Benchmark
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = hf.core.resolve(end, world);
    }
    let elapsed = start.elapsed();

    let ns_per_op = elapsed.as_nanos() / ITERATIONS as u128;
    println!("HFrame lookup by ID (depth 10): {} ns/op", ns_per_op);
    println!("  Total time: {:?} for {} ops", elapsed, ITERATIONS);
    println!(
        "  Throughput: {:.2} M ops/sec",
        ITERATIONS as f64 / elapsed.as_secs_f64() / 1_000_000.0
    );
}

/// Benchmark concurrent reads
#[test]
#[ignore]
fn hframe_benchmark_concurrent_reads() {
    use std::sync::Arc;
    use std::thread;

    let hf = Arc::new(HFrame::new());

    // Setup
    hf.register_frame("world", None).unwrap();
    let base = hf.register_frame("base_link", Some("world")).unwrap();
    let camera = hf.register_frame("camera", Some("base_link")).unwrap();

    hf.update_transform_by_id(base, &Transform::from_translation([1.0, 0.0, 0.0]), 1000);
    hf.update_transform_by_id(camera, &Transform::from_translation([0.0, 0.0, 0.5]), 1000);

    let world = hf.frame_id("world").unwrap();

    let num_threads = 4;
    let ops_per_thread = ITERATIONS / num_threads as u64;

    let start = Instant::now();
    let handles: Vec<_> = (0..num_threads)
        .map(|_| {
            let hf = hf.clone();
            thread::spawn(move || {
                for _ in 0..ops_per_thread {
                    let _ = hf.core.resolve(camera, world);
                }
            })
        })
        .collect();

    for handle in handles {
        handle.join().unwrap();
    }
    let elapsed = start.elapsed();

    let total_ops = num_threads as u64 * ops_per_thread;
    let ns_per_op = elapsed.as_nanos() / total_ops as u128;
    println!(
        "HFrame concurrent reads ({} threads): {} ns/op",
        num_threads, ns_per_op
    );
    println!("  Total time: {:?} for {} ops", elapsed, total_ops);
    println!(
        "  Throughput: {:.2} M ops/sec",
        total_ops as f64 / elapsed.as_secs_f64() / 1_000_000.0
    );
}

/// Print all benchmark results
#[test]
#[ignore]
fn hframe_benchmark_all() {
    println!("\n========================================");
    println!("HFrame Performance Benchmarks");
    println!("========================================\n");

    // Re-run all benchmarks
    hframe_benchmark_update();
    println!();
    hframe_benchmark_lookup_by_id();
    println!();
    hframe_benchmark_lookup_by_name();
    println!();
    hframe_benchmark_deep_chain();
    println!();
    hframe_benchmark_concurrent_reads();
    println!();
    hframe_benchmark_register();

    println!("\n========================================");
    println!("Benchmark complete!");
    println!("========================================\n");
}
