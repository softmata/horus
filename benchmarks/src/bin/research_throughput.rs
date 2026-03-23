//! Sustained Throughput Benchmark
//!
//! Measures messages/sec over time (per-second granularity) for 60s+.
//! Reveals GC pauses, page faults, and allocation spikes.
//!
//! Run: cargo run --release -p horus_benchmarks --bin research_throughput
//! CSV: cargo run --release -p horus_benchmarks --bin research_throughput -- --csv throughput.csv

use horus_benchmarks::detect_platform;
use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

#[repr(C)]
#[derive(Copy, Clone, Serialize, Deserialize)]
struct Msg8 { data: [u8; 8] }
unsafe impl Send for Msg8 {}
unsafe impl Sync for Msg8 {}

fn unique(prefix: &str) -> String {
    static C: AtomicU64 = AtomicU64::new(0);
    format!("{}_{}_{}",prefix,std::process::id(),C.fetch_add(1,Ordering::Relaxed))
}

fn cleanup_shm() {
    let _ = std::fs::remove_dir_all(horus_core::memory::shm_topics_dir());
    let mut n = horus_core::memory::shm_base_dir(); n.push("nodes");
    let _ = std::fs::remove_dir_all(n);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let duration: u64 = args.iter().position(|a| a == "--duration")
        .map(|i| args[i+1].parse().unwrap_or(10)).unwrap_or(10);
    let csv_path = args.iter().position(|a| a == "--csv").map(|i| args[i+1].clone());

    let platform = detect_platform();
    cleanup_shm();

    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║          Research Throughput — Sustained                    ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();
    println!("Platform: {}, {} cores", platform.cpu.model, platform.cpu.logical_cores);
    println!("Duration: {}s", duration);
    println!();

    let topic_name = unique("rt_throughput");
    let pub_topic: Topic<Msg8> = Topic::new(&topic_name).expect("pub topic");
    let sub_topic: Topic<Msg8> = Topic::new(&topic_name).expect("sub topic");
    let msg = Msg8 { data: [0u8; 8] };

    // Warmup 1 second
    let warmup_end = Instant::now() + Duration::from_secs(1);
    while Instant::now() < warmup_end {
        pub_topic.send(msg);
        let _ = sub_topic.recv();
    }

    // Measure per-second throughput
    let mut per_second: Vec<(u64, u64)> = Vec::with_capacity(duration as usize + 1);
    let start = Instant::now();
    let mut second_start = start;
    let mut second_count: u64 = 0;
    let mut total: u64 = 0;

    while start.elapsed() < Duration::from_secs(duration) {
        pub_topic.send(msg);
        while sub_topic.recv().is_none() {}
        second_count += 1;
        total += 1;

        if second_start.elapsed() >= Duration::from_secs(1) {
            let sec = per_second.len() as u64 + 1;
            per_second.push((sec, second_count));
            second_count = 0;
            second_start = Instant::now();
        }
    }
    // Flush remaining partial second
    if second_count > 0 {
        per_second.push((per_second.len() as u64 + 1, second_count));
    }

    let elapsed = start.elapsed().as_secs_f64();
    let mean_mps = total as f64 / elapsed;
    let rates: Vec<u64> = per_second.iter().map(|(_, c)| *c).collect();
    let min_mps = rates.iter().copied().min().unwrap_or(0);
    let max_mps = rates.iter().copied().max().unwrap_or(0);

    println!("{:<12} {:>14} {:>14} {:>14} {:>14}", "Metric", "Total", "Mean/sec", "Min/sec", "Max/sec");
    println!("{:<12} {:>14} {:>13.0} {:>14} {:>14}",
        "8B msg", total, mean_mps, min_mps, max_mps);
    println!();

    println!("Per-second throughput:");
    for (sec, count) in &per_second {
        let bar_len = (*count as f64 / max_mps as f64 * 40.0) as usize;
        println!("  {:>3}s: {:>12} msgs/s {}", sec, count, "█".repeat(bar_len));
    }

    if let Some(path) = csv_path {
        let mut f = std::io::BufWriter::new(std::fs::File::create(&path).unwrap());
        writeln!(f, "second,messages_count,msgs_per_sec").unwrap();
        for (sec, count) in &per_second {
            writeln!(f, "{},{},{}", sec, count, count).unwrap();
        }
        f.flush().unwrap();
        println!("\nCSV written: {path}");
    }

    println!("\nDone. {total} messages in {elapsed:.1}s.");
}
