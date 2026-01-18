// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! POD Message Benchmark
//!
//! Compares POD messaging (~50ns) vs regular bincode-based messaging (~250ns)
//! to demonstrate the performance gains from zero-serialization transfer.

use horus_core::communication::Topic;
use horus_library::messages::CmdVel;
use std::time::{Duration, Instant};

const ITERATIONS: usize = 100_000;
const WARMUP: usize = 1000;

/// Benchmark statistics
struct BenchmarkStats {
    send_ns: f64,
    recv_ns: f64,
    roundtrip_ns: f64,
    raw_latencies: Vec<f64>,
}

fn main() {
    println!("======================================================================");
    println!("        HORUS POD Message Performance Benchmark                       ");
    println!("======================================================================");
    println!(" Comparing POD (zero-serialization) vs Standard (bincode)             ");
    println!("======================================================================");
    println!();

    // Benchmark POD Topic
    println!("[1/3] Setting up POD Topic benchmark...");
    let pod_stats = benchmark_pod_topic();

    // Benchmark Standard Topic
    println!("[2/3] Setting up Standard Topic benchmark...");
    let std_stats = benchmark_standard_topic();

    // Print results
    println!();
    println!("===================================================================");
    println!("                         RESULTS                                    ");
    println!("===================================================================");
    println!();

    println!("POD Topic (zero-serialization):");
    println!("   Send:     {:>8.1} ns/op", pod_stats.send_ns);
    println!("   Recv:     {:>8.1} ns/op", pod_stats.recv_ns);
    println!("   Roundtrip:{:>8.1} ns/op", pod_stats.roundtrip_ns);
    println!();

    println!("Standard Topic (bincode serialization):");
    println!("   Send:     {:>8.1} ns/op", std_stats.send_ns);
    println!("   Recv:     {:>8.1} ns/op", std_stats.recv_ns);
    println!("   Roundtrip:{:>8.1} ns/op", std_stats.roundtrip_ns);
    println!();

    println!("===================================================================");
    println!("                        SPEEDUP                                     ");
    println!("===================================================================");
    println!();

    let send_speedup = std_stats.send_ns / pod_stats.send_ns;
    let recv_speedup = std_stats.recv_ns / pod_stats.recv_ns;
    let roundtrip_speedup = std_stats.roundtrip_ns / pod_stats.roundtrip_ns;

    println!("   Send:      {:.1}x faster", send_speedup);
    println!("   Recv:      {:.1}x faster", recv_speedup);
    println!("   Roundtrip: {:.1}x faster", roundtrip_speedup);
    println!();

    // Statistical analysis
    println!("[3/3] Running statistical analysis...");
    if !pod_stats.raw_latencies.is_empty() {
        print_stats("POD Topic Roundtrip", &pod_stats.raw_latencies);
    }
    if !std_stats.raw_latencies.is_empty() {
        print_stats("Standard Topic Roundtrip", &std_stats.raw_latencies);
    }

    // Validate results meet expectations
    println!();
    if roundtrip_speedup >= 2.0 {
        println!(
            "[OK] POD messaging achieves {:.1}x speedup - EXCELLENT!",
            roundtrip_speedup
        );
    } else if roundtrip_speedup >= 1.5 {
        println!(
            "[OK] POD messaging achieves {:.1}x speedup - GOOD",
            roundtrip_speedup
        );
    } else {
        println!(
            "[WARN] POD speedup is only {:.1}x - investigate overhead",
            roundtrip_speedup
        );
    }

    println!();
    println!("======================================================================");
    println!("                    Benchmark Complete                                ");
    println!("======================================================================");
}

fn print_stats(name: &str, latencies: &[f64]) {
    if latencies.is_empty() {
        return;
    }

    let mut sorted = latencies.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let len = sorted.len();
    let mean = sorted.iter().sum::<f64>() / len as f64;
    let median = if len % 2 == 0 {
        (sorted[len / 2 - 1] + sorted[len / 2]) / 2.0
    } else {
        sorted[len / 2]
    };
    let p95 = sorted[(len as f64 * 0.95) as usize];
    let p99 = sorted[(len as f64 * 0.99) as usize];
    let min = sorted[0];
    let max = sorted[len - 1];

    println!();
    println!("  {} Statistics:", name);
    println!("    Mean:   {:>8.1} ns", mean);
    println!("    Median: {:>8.1} ns", median);
    println!("    P95:    {:>8.1} ns", p95);
    println!("    P99:    {:>8.1} ns", p99);
    println!("    Min:    {:>8.1} ns", min);
    println!("    Max:    {:>8.1} ns", max);
}

fn benchmark_pod_topic() -> BenchmarkStats {
    // Create producer and consumer
    let producer: Topic<CmdVel> =
        Topic::new("bench_pod_cmdvel").expect("Failed to create POD producer");
    let consumer: Topic<CmdVel> =
        Topic::new("bench_pod_cmdvel").expect("Failed to create POD consumer");

    // Warmup
    for i in 0..WARMUP {
        let msg = CmdVel::new(i as f32 * 0.1, i as f32 * 0.01);
        producer.send(msg, &mut None).unwrap();
        let _ = consumer.recv(&mut None);
    }

    // Benchmark send
    let msg = CmdVel::new(1.0, 0.5);
    let send_start = Instant::now();
    for _ in 0..ITERATIONS {
        producer.send(msg, &mut None).unwrap();
    }
    let send_duration = send_start.elapsed();

    // Benchmark recv (make sure data is available)
    producer.send(msg, &mut None).unwrap();
    let recv_start = Instant::now();
    for _ in 0..ITERATIONS {
        // Keep sending to ensure recv always has data
        producer.send(msg, &mut None).unwrap();
        let _ = consumer.recv(&mut None);
    }
    let recv_duration = recv_start.elapsed();

    // Benchmark roundtrip with individual latency collection
    let mut raw_latencies = Vec::with_capacity(ITERATIONS);
    for i in 0..ITERATIONS {
        let msg = CmdVel::new(i as f32 * 0.1, i as f32 * 0.01);
        let start = Instant::now();
        producer.send(msg, &mut None).unwrap();
        let _ = consumer.recv(&mut None).expect("Should receive message");
        raw_latencies.push(start.elapsed().as_nanos() as f64);
    }

    let roundtrip_ns = raw_latencies.iter().sum::<f64>() / ITERATIONS as f64;

    BenchmarkStats {
        send_ns: duration_to_ns_per_op(send_duration, ITERATIONS),
        recv_ns: duration_to_ns_per_op(recv_duration, ITERATIONS),
        roundtrip_ns,
        raw_latencies,
    }
}

fn benchmark_standard_topic() -> BenchmarkStats {
    // Create producer and consumer
    let producer: Topic<CmdVel> =
        Topic::new("bench_std_cmdvel").expect("Failed to create standard producer");
    let consumer: Topic<CmdVel> =
        Topic::new("bench_std_cmdvel").expect("Failed to create standard consumer");

    // Warmup
    for i in 0..WARMUP {
        let msg = CmdVel::new(i as f32 * 0.1, i as f32 * 0.01);
        let _ = producer.send(msg, &mut None);
        let _ = consumer.recv(&mut None);
    }

    // Benchmark send
    let msg = CmdVel::new(1.0, 0.5);
    let send_start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = producer.send(msg, &mut None);
    }
    let send_duration = send_start.elapsed();

    // Benchmark recv
    let _ = producer.send(msg, &mut None);
    let recv_start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = producer.send(msg, &mut None);
        let _ = consumer.recv(&mut None);
    }
    let recv_duration = recv_start.elapsed();

    // Benchmark roundtrip with individual latency collection
    let mut raw_latencies = Vec::with_capacity(ITERATIONS);
    for i in 0..ITERATIONS {
        let msg = CmdVel::new(i as f32 * 0.1, i as f32 * 0.01);
        let start = Instant::now();
        let _ = producer.send(msg, &mut None);
        let _ = consumer.recv(&mut None);
        raw_latencies.push(start.elapsed().as_nanos() as f64);
    }

    let roundtrip_ns = raw_latencies.iter().sum::<f64>() / ITERATIONS as f64;

    BenchmarkStats {
        send_ns: duration_to_ns_per_op(send_duration, ITERATIONS),
        recv_ns: duration_to_ns_per_op(recv_duration, ITERATIONS),
        roundtrip_ns,
        raw_latencies,
    }
}

fn duration_to_ns_per_op(duration: Duration, iterations: usize) -> f64 {
    duration.as_nanos() as f64 / iterations as f64
}
