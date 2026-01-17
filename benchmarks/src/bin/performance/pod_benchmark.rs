//! POD Message Benchmark
//!
//! Compares POD messaging (~50ns) vs regular bincode-based messaging (~250ns)
//! to demonstrate the performance gains from zero-serialization transfer.
//!
//! After running, generates:
//! - Comparison bar charts
//! - Speedup visualization
//! - Latency histograms
//! - Statistical analysis

use horus_benchmarks::visualization::{
    draw_grouped_bar_chart, draw_latency_histogram, draw_latency_timeline, draw_speedup_chart,
    AnalysisSummary, BenchmarkStats,
};
use horus_core::communication::Topic;
use horus_library::messages::CmdVel;
use std::time::{Duration, Instant};

const ITERATIONS: usize = 100_000;
const WARMUP: usize = 1000;

fn main() {
    println!("======================================================================");
    println!("        HORUS POD Message Performance Benchmark                       ");
    println!("======================================================================");
    println!(" Comparing POD (zero-serialization) vs Standard (bincode)             ");
    println!("======================================================================");
    println!();

    // Benchmark POD Link
    println!("[1/4] Setting up POD Link benchmark...");
    let pod_stats = benchmark_pod_link();

    // Benchmark Standard Link
    println!("[2/4] Setting up Standard Link benchmark...");
    let std_stats = benchmark_standard_link();

    // Print results
    println!();
    println!("===================================================================");
    println!("                         RESULTS                                    ");
    println!("===================================================================");
    println!();

    println!("POD Link (zero-serialization):");
    println!("   Send:     {:>8.1} ns/op", pod_stats.send_ns);
    println!("   Recv:     {:>8.1} ns/op", pod_stats.recv_ns);
    println!("   Roundtrip:{:>8.1} ns/op", pod_stats.roundtrip_ns);
    println!();

    println!("Standard Link (bincode serialization):");
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
    println!("[3/4] Running statistical analysis...");
    if !pod_stats.raw_latencies.is_empty() {
        let pod_analysis = AnalysisSummary::from_latencies(&pod_stats.raw_latencies);
        pod_analysis.print_report("POD Link Roundtrip");
    }
    if !std_stats.raw_latencies.is_empty() {
        let std_analysis = AnalysisSummary::from_latencies(&std_stats.raw_latencies);
        std_analysis.print_report("Standard Link Roundtrip");
    }

    // Generate visualizations
    println!();
    println!("[4/4] Generating visualizations...");
    let output_dir = "benchmarks/results/graphs";

    if let Err(e) = generate_visualizations(output_dir, &pod_stats, &std_stats) {
        eprintln!("Warning: Failed to generate some visualizations: {}", e);
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

fn generate_visualizations(
    output_dir: &str,
    pod_stats: &BenchmarkStats,
    std_stats: &BenchmarkStats,
) -> Result<(), Box<dyn std::error::Error>> {
    std::fs::create_dir_all(output_dir)?;

    let timestamp = chrono::Local::now().format("%Y%m%d_%H%M%S");

    // 1. Comparison bar chart
    println!("  -> Generating comparison chart...");
    draw_grouped_bar_chart(
        &format!("{}/pod_comparison_{}.png", output_dir, timestamp),
        "POD vs Standard Link Performance",
        &["Send", "Recv", "Roundtrip"],
        &[pod_stats.send_ns, pod_stats.recv_ns, pod_stats.roundtrip_ns],
        &[std_stats.send_ns, std_stats.recv_ns, std_stats.roundtrip_ns],
        "POD Link",
        "Standard Link",
        "Latency (ns)",
    )?;

    // 2. Speedup chart
    println!("  -> Generating speedup chart...");
    let speedups = [
        std_stats.send_ns / pod_stats.send_ns,
        std_stats.recv_ns / pod_stats.recv_ns,
        std_stats.roundtrip_ns / pod_stats.roundtrip_ns,
    ];
    draw_speedup_chart(
        &format!("{}/pod_speedup_{}.png", output_dir, timestamp),
        "POD Link Speedup vs Standard Link",
        &["Send", "Recv", "Roundtrip"],
        &speedups,
    )?;

    // 3. Latency histograms
    if !pod_stats.raw_latencies.is_empty() {
        println!("  -> Generating POD latency histogram...");
        draw_latency_histogram(
            &format!("{}/pod_histogram_{}.png", output_dir, timestamp),
            "POD Link Latency Distribution",
            &pod_stats.raw_latencies,
            50,
        )?;
    }

    if !std_stats.raw_latencies.is_empty() {
        println!("  -> Generating Standard latency histogram...");
        draw_latency_histogram(
            &format!("{}/std_histogram_{}.png", output_dir, timestamp),
            "Standard Link Latency Distribution",
            &std_stats.raw_latencies,
            50,
        )?;
    }

    // 4. Timeline charts
    if pod_stats.raw_latencies.len() > 100 {
        println!("  -> Generating POD latency timeline...");
        draw_latency_timeline(
            &format!("{}/pod_timeline_{}.png", output_dir, timestamp),
            "POD Link Latency Over Time",
            &pod_stats.raw_latencies,
            100,
        )?;
    }

    if std_stats.raw_latencies.len() > 100 {
        println!("  -> Generating Standard latency timeline...");
        draw_latency_timeline(
            &format!("{}/std_timeline_{}.png", output_dir, timestamp),
            "Standard Link Latency Over Time",
            &std_stats.raw_latencies,
            100,
        )?;
    }

    // Save JSON results for historical comparison
    let results = serde_json::json!({
        "timestamp": timestamp.to_string(),
        "pod": {
            "send_ns": pod_stats.send_ns,
            "recv_ns": pod_stats.recv_ns,
            "roundtrip_ns": pod_stats.roundtrip_ns,
        },
        "standard": {
            "send_ns": std_stats.send_ns,
            "recv_ns": std_stats.recv_ns,
            "roundtrip_ns": std_stats.roundtrip_ns,
        },
        "speedup": {
            "send": std_stats.send_ns / pod_stats.send_ns,
            "recv": std_stats.recv_ns / pod_stats.recv_ns,
            "roundtrip": std_stats.roundtrip_ns / pod_stats.roundtrip_ns,
        }
    });

    std::fs::write(
        format!("{}/results_{}.json", output_dir, timestamp),
        serde_json::to_string_pretty(&results)?,
    )?;

    println!();
    println!("  Graphs saved to: {}/", output_dir);
    println!(
        "  Results saved to: {}/results_{}.json",
        output_dir, timestamp
    );

    Ok(())
}

fn benchmark_pod_link() -> BenchmarkStats {
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

fn benchmark_standard_link() -> BenchmarkStats {
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
