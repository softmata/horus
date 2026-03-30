//! Jitter Benchmark — RT Tick + IPC Under Contention
//!
//! Produces tick-interval and IPC latency histograms for production analysis.
//!
//! Run: cargo run --release -p horus_benchmarks --bin research_jitter
//! CSV: cargo run --release -p horus_benchmarks --bin research_jitter -- --csv jitter.csv

use horus_benchmarks::detect_platform;
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

#[repr(C)]
#[derive(Copy, Clone, Serialize, Deserialize)]
struct Msg8 {
    data: [u8; 8],
}
unsafe impl Send for Msg8 {}
unsafe impl Sync for Msg8 {}

fn unique(prefix: &str) -> String {
    static C: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        C.fetch_add(1, Ordering::Relaxed)
    )
}

fn cleanup_shm() {
    let _ = std::fs::remove_dir_all(horus_core::memory::shm_topics_dir());
    let mut n = horus_core::memory::shm_base_dir();
    n.push("nodes");
    let _ = std::fs::remove_dir_all(n);
}

// ── RT Jitter Node ─────────────────────────────────────────────────────

struct JitterNode {
    name: String,
    intervals: Arc<Mutex<Vec<u64>>>,
    last_tick: Option<Instant>,
}

impl JitterNode {
    fn new(name: &str) -> (Self, Arc<Mutex<Vec<u64>>>) {
        let intervals = Arc::new(Mutex::new(Vec::with_capacity(20_000)));
        (
            Self {
                name: name.to_string(),
                intervals: intervals.clone(),
                last_tick: None,
            },
            intervals,
        )
    }
}

impl Node for JitterNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        let now = Instant::now();
        if let Some(last) = self.last_tick {
            let interval = now.duration_since(last).as_nanos() as u64;
            if let Ok(mut v) = self.intervals.try_lock() {
                v.push(interval);
            }
        }
        self.last_tick = Some(now);
    }
}

// ── CPU Load Thread ────────────────────────────────────────────────────

fn spawn_cpu_load(running: Arc<std::sync::atomic::AtomicBool>) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut x = 1u64;
        while running.load(Ordering::Relaxed) {
            for _ in 0..10_000 {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1);
            }
            std::hint::black_box(x);
        }
    })
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let duration: u64 = args
        .iter()
        .position(|a| a == "--duration")
        .map(|i| args[i + 1].parse().unwrap_or(5))
        .unwrap_or(5);
    let csv_path = args
        .iter()
        .position(|a| a == "--csv")
        .map(|i| args[i + 1].clone());

    let platform = detect_platform();
    cleanup_shm();

    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║          Research Jitter — RT Tick + IPC Contention         ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();
    println!(
        "Platform: {}, {} cores",
        platform.cpu.model, platform.cpu.logical_cores
    );
    println!("Duration: {}s per test", duration);
    println!();

    let mut csv_rows: Vec<(String, u64, u64)> = Vec::new();

    // ── Test 1: RT Tick Jitter at 1kHz ────────────────────────────────

    println!("── RT Tick Jitter (1kHz) ────────────────────────────────────");
    cleanup_shm();

    let (jitter_node, intervals) = JitterNode::new("jitter_rt");
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    let _ = scheduler
        .add(jitter_node)
        .order(0)
        .rate(1000_u64.hz())
        .build();
    let _ = scheduler.run_for(std::time::Duration::from_secs(duration));

    let mut ticks = intervals.lock().unwrap().clone();
    if !ticks.is_empty() {
        ticks.sort_unstable();
        let n = ticks.len();
        let expected_us = 1000; // 1kHz = 1000μs period
        println!("  Samples: {}", n);
        println!("  Expected: {}μs", expected_us);
        println!("  p50:  {}μs", ticks[n / 2] / 1000);
        println!("  p95:  {}μs", ticks[n * 95 / 100] / 1000);
        println!("  p99:  {}μs", ticks[n * 99 / 100] / 1000);
        println!("  p999: {}μs", ticks[n * 999 / 1000] / 1000);
        println!("  max:  {}μs", ticks[n - 1] / 1000);
        for (i, &v) in ticks.iter().enumerate() {
            csv_rows.push(("rt_tick_1khz".into(), i as u64, v));
        }
    }
    println!();

    // ── Test 2: IPC Latency Under CPU Contention ──────────────────────

    println!("── IPC Latency Under Contention (4 CPU threads) ─────────────");
    cleanup_shm();

    let topic_name = unique("jitter_ipc");
    let pub_topic: Topic<Msg8> = Topic::new(&topic_name).expect("pub");
    let sub_topic: Topic<Msg8> = Topic::new(&topic_name).expect("sub");
    let msg = Msg8 { data: [0u8; 8] };

    // Start 4 CPU contention threads
    let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let load_threads: Vec<_> = (0..4).map(|_| spawn_cpu_load(running.clone())).collect();

    std::thread::sleep(100_u64.ms());

    // Measure IPC latency under load
    let deadline = Instant::now() + std::time::Duration::from_secs(duration);
    let mut ipc_samples = Vec::with_capacity(500_000);
    while Instant::now() < deadline {
        let start = Instant::now();
        pub_topic.send(msg);
        while sub_topic.recv().is_none() {}
        ipc_samples.push(start.elapsed().as_nanos() as u64);
    }

    running.store(false, Ordering::SeqCst);
    for h in load_threads {
        let _ = h.join();
    }

    if !ipc_samples.is_empty() {
        ipc_samples.sort_unstable();
        let n = ipc_samples.len();
        println!("  Samples: {}", n);
        println!("  p50:  {}ns", ipc_samples[n / 2]);
        println!("  p95:  {}ns", ipc_samples[n * 95 / 100]);
        println!("  p99:  {}ns", ipc_samples[n * 99 / 100]);
        println!("  p999: {}ns", ipc_samples[n * 999 / 1000]);
        println!("  max:  {}ns", ipc_samples[n - 1]);
        for (i, &v) in ipc_samples.iter().enumerate() {
            csv_rows.push(("ipc_under_contention".into(), i as u64, v));
        }
    }

    // ── CSV Output ────────────────────────────────────────────────────

    if let Some(path) = csv_path {
        let mut f = std::io::BufWriter::new(std::fs::File::create(&path).unwrap());
        writeln!(f, "test,sample_id,value_ns").unwrap();
        for (test, id, val) in &csv_rows {
            writeln!(f, "{},{},{}", test, id, val).unwrap();
        }
        f.flush().unwrap();
        println!("\nCSV written: {path} ({} rows)", csv_rows.len());
    }

    println!("\nDone.");
}
