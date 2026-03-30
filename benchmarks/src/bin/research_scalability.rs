//! Scalability Benchmark — Node Count + Topic Count
//!
//! Measures overhead scaling with increasing node count and topic count.
//!
//! Run: cargo run --release -p horus_benchmarks --bin research_scalability
//! CSV: cargo run --release -p horus_benchmarks --bin research_scalability -- --csv scale.csv

use horus_benchmarks::detect_platform;
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

#[repr(C)]
#[derive(Copy, Clone, Serialize, Deserialize)]
struct Msg8 {
    data: [u8; 8],
}
unsafe impl Send for Msg8 {}
unsafe impl Sync for Msg8 {}

fn cleanup_shm() {
    let _ = std::fs::remove_dir_all(horus_core::memory::shm_topics_dir());
    let mut n = horus_core::memory::shm_base_dir();
    n.push("nodes");
    let _ = std::fs::remove_dir_all(n);
}

// ── Load Node ──────────────────────────────────────────────────────────

struct LoadNode {
    name: String,
    counter: Arc<AtomicU64>,
}

impl LoadNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            counter: Arc::new(AtomicU64::new(0)),
        }
    }
}

impl Node for LoadNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        // Minimal work — just increment counter
        self.counter.fetch_add(1, Ordering::Relaxed);
    }
}

use std::sync::Arc;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let csv_path = args
        .iter()
        .position(|a| a == "--csv")
        .map(|i| args[i + 1].clone());

    let platform = detect_platform();

    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║          Research Scalability — Nodes + Topics              ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();
    println!(
        "Platform: {}, {} cores",
        platform.cpu.model, platform.cpu.logical_cores
    );
    println!();

    let mut csv_rows: Vec<(String, usize, String, u64)> = Vec::new();

    // ── Node Scaling ──────────────────────────────────────────────────

    println!("── Node Scaling (tick overhead per node) ─────────────────────");
    println!(
        "{:>6} {:>12} {:>12} {:>12}",
        "Nodes", "Total (μs)", "Per-Node", "Ticks"
    );

    for node_count in [1, 2, 5, 10, 20, 50, 100] {
        cleanup_shm();

        let mut scheduler = Scheduler::new().tick_rate(10000_u64.hz());
        let mut counters = Vec::new();

        for i in 0..node_count {
            let name: &'static str = Box::leak(format!("scale_n_{i}").into_boxed_str());
            let node = LoadNode::new(name);
            counters.push(node.counter.clone());
            let _ = scheduler.add(node).order(i as u32).build();
        }

        let start = Instant::now();
        let _ = scheduler.run_for(1_u64.secs());
        let elapsed = start.elapsed();

        let total_ticks: u64 = counters.iter().map(|c| c.load(Ordering::Relaxed)).sum();
        let ticks_per_node = total_ticks / node_count as u64;
        let us_per_tick = if ticks_per_node > 0 {
            elapsed.as_micros() as u64 / ticks_per_node
        } else {
            0
        };
        let us_per_node_per_tick = if ticks_per_node > 0 && node_count > 0 {
            (elapsed.as_micros() as u64 * node_count as u64) / total_ticks
        } else {
            0
        };

        println!(
            "{:>6} {:>11}μs {:>11}μs {:>12}",
            node_count, us_per_tick, us_per_node_per_tick, ticks_per_node
        );

        csv_rows.push((
            "node_scaling".into(),
            node_count,
            "total_tick_us".into(),
            us_per_tick,
        ));
        csv_rows.push((
            "node_scaling".into(),
            node_count,
            "per_node_us".into(),
            us_per_node_per_tick,
        ));
        csv_rows.push((
            "node_scaling".into(),
            node_count,
            "ticks_per_node".into(),
            ticks_per_node,
        ));
    }

    println!();

    // ── Topic Scaling ─────────────────────────────────────────────────

    println!("── Topic Scaling (send+recv latency) ─────────────────────────");
    println!(
        "{:>6} {:>12} {:>12} {:>12}",
        "Topics", "p50 (ns)", "p99 (ns)", "Samples"
    );

    for topic_count in [1, 10, 50, 100, 500, 1000] {
        cleanup_shm();

        // Create N topics
        let mut topics_pub: Vec<Topic<Msg8>> = Vec::new();
        let mut topics_sub: Vec<Topic<Msg8>> = Vec::new();
        for i in 0..topic_count {
            let name = format!("scale_t_{}_{}", std::process::id(), i);
            topics_pub.push(Topic::new(&name).expect("pub"));
            topics_sub.push(Topic::new(&name).expect("sub"));
        }

        let msg = Msg8 { data: [0u8; 8] };

        // Warmup on last topic
        for _ in 0..1000 {
            topics_pub.last().unwrap().send(msg);
            while topics_sub.last_mut().unwrap().recv().is_none() {}
        }

        // Measure send+recv on last topic (with all others existing)
        let mut samples = Vec::with_capacity(100_000);
        let deadline = Instant::now() + std::time::Duration::from_secs(2);
        while Instant::now() < deadline {
            let start = Instant::now();
            topics_pub.last().unwrap().send(msg);
            while topics_sub.last_mut().unwrap().recv().is_none() {}
            samples.push(start.elapsed().as_nanos() as u64);
        }

        samples.sort_unstable();
        let n = samples.len();
        if n > 0 {
            println!(
                "{:>6} {:>11}ns {:>11}ns {:>12}",
                topic_count,
                samples[n / 2],
                samples[n * 99 / 100],
                n
            );

            csv_rows.push((
                "topic_scaling".into(),
                topic_count,
                "p50_ns".into(),
                samples[n / 2],
            ));
            csv_rows.push((
                "topic_scaling".into(),
                topic_count,
                "p99_ns".into(),
                samples[n * 99 / 100],
            ));
            csv_rows.push((
                "topic_scaling".into(),
                topic_count,
                "samples".into(),
                n as u64,
            ));
        }
    }

    // ── CSV Output ────────────────────────────────────────────────────

    if let Some(path) = csv_path {
        let mut f = std::io::BufWriter::new(std::fs::File::create(&path).unwrap());
        writeln!(f, "test,count,metric,value").unwrap();
        for (test, count, metric, value) in &csv_rows {
            writeln!(f, "{},{},{},{}", test, count, metric, value).unwrap();
        }
        f.flush().unwrap();
        println!("\nCSV written: {path}");
    }

    println!("\nDone.");
}
