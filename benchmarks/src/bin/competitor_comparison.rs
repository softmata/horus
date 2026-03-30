//! Competitor Comparison Benchmark
//!
//! Measures identical workloads on HORUS, raw UDP, and optionally Zenoh.
//! Uses the same timing method for all competitors for fair comparison.
//!
//! Run:   cargo run --release -p horus_benchmarks --bin competitor_comparison
//! Zenoh: cargo run --release -p horus_benchmarks --bin competitor_comparison --features zenoh
//! CSV:   cargo run --release -p horus_benchmarks --bin competitor_comparison -- --csv comparison.csv

use horus_benchmarks::detect_platform;
use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

// ============================================================================
// Message types (same for all competitors)
// ============================================================================

macro_rules! define_msg {
    ($name:ident, $size:expr) => {
        #[repr(C)]
        #[derive(Copy, Clone, Serialize, Deserialize)]
        struct $name {
            data: [u8; $size],
        }
        unsafe impl Send for $name {}
        unsafe impl Sync for $name {}
    };
}

define_msg!(Bench8, 8);
define_msg!(Bench32, 32);

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

// ============================================================================
// Statistics
// ============================================================================

struct Stats {
    count: usize,
    p50: u64,
    p95: u64,
    p99: u64,
    p999: u64,
    max: u64,
    mean: u64,
}

fn compute_stats(samples: &mut [u64]) -> Stats {
    samples.sort_unstable();
    let n = samples.len();
    if n == 0 {
        return Stats {
            count: 0,
            p50: 0,
            p95: 0,
            p99: 0,
            p999: 0,
            max: 0,
            mean: 0,
        };
    }
    let sum: u64 = samples.iter().sum();
    Stats {
        count: n,
        p50: samples[n / 2],
        p95: samples[n * 95 / 100],
        p99: samples[n * 99 / 100],
        p999: samples[n * 999 / 1000],
        max: samples[n - 1],
        mean: sum / n as u64,
    }
}

// ============================================================================
// HORUS benchmark
// ============================================================================

fn bench_horus(size: usize, duration_secs: u64) -> Vec<u64> {
    cleanup_shm();
    let topic_name = unique("comp_horus");

    match size {
        8 => {
            let pub_t: Topic<Bench8> = Topic::new(&topic_name).expect("horus pub");
            let sub_t: Topic<Bench8> = Topic::new(&topic_name).expect("horus sub");
            let msg = Bench8 { data: [0u8; 8] };
            // Warmup
            for _ in 0..5000 {
                pub_t.send(msg);
                while sub_t.recv().is_none() {}
            }
            // Measure
            let deadline = Instant::now() + std::time::Duration::from_secs(duration_secs);
            let mut samples = Vec::with_capacity(2_000_000);
            while Instant::now() < deadline {
                let s = Instant::now();
                pub_t.send(msg);
                while sub_t.recv().is_none() {}
                samples.push(s.elapsed().as_nanos() as u64);
            }
            samples
        }
        _ => {
            let pub_t: Topic<Bench32> = Topic::new(&topic_name).expect("horus pub");
            let sub_t: Topic<Bench32> = Topic::new(&topic_name).expect("horus sub");
            let msg = Bench32 { data: [0u8; 32] };
            for _ in 0..5000 {
                pub_t.send(msg);
                while sub_t.recv().is_none() {}
            }
            let deadline = Instant::now() + std::time::Duration::from_secs(duration_secs);
            let mut samples = Vec::with_capacity(2_000_000);
            while Instant::now() < deadline {
                let s = Instant::now();
                pub_t.send(msg);
                while sub_t.recv().is_none() {}
                samples.push(s.elapsed().as_nanos() as u64);
            }
            samples
        }
    }
}

// ============================================================================
// Raw UDP benchmark (loopback)
// ============================================================================

fn bench_raw_udp(size: usize, duration_secs: u64) -> Vec<u64> {
    let send_sock = UdpSocket::bind("127.0.0.1:0").expect("bind send");
    let recv_sock = UdpSocket::bind("127.0.0.1:0").expect("bind recv");
    let recv_addr = recv_sock.local_addr().unwrap();
    recv_sock.set_nonblocking(false).unwrap();

    let buf_send = vec![0xABu8; size];
    let mut buf_recv = vec![0u8; size + 64]; // extra for safety

    // Warmup
    for _ in 0..1000 {
        send_sock.send_to(&buf_send, recv_addr).unwrap();
        recv_sock.recv_from(&mut buf_recv).unwrap();
    }

    // Measure
    let deadline = Instant::now() + std::time::Duration::from_secs(duration_secs);
    let mut samples = Vec::with_capacity(500_000);
    while Instant::now() < deadline {
        let s = Instant::now();
        send_sock.send_to(&buf_send, recv_addr).unwrap();
        recv_sock.recv_from(&mut buf_recv).unwrap();
        samples.push(s.elapsed().as_nanos() as u64);
    }
    samples
}

// ============================================================================
// Zenoh benchmark (feature-gated)
// ============================================================================

#[cfg(feature = "zenoh")]
fn bench_zenoh(size: usize, duration_secs: u64) -> Vec<u64> {
    use zenoh::prelude::r#async::*;

    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let session = zenoh::open(zenoh::config::default()).res().await.unwrap();
        let key = format!("horus_bench/{size}");
        let publisher = session.declare_publisher(&key).res().await.unwrap();
        let subscriber = session.declare_subscriber(&key).res().await.unwrap();

        let payload = vec![0xCDu8; size];

        // Warmup
        for _ in 0..1000 {
            publisher.put(&payload).res().await.unwrap();
            let _ = subscriber.recv_async().await;
        }

        // Measure
        let deadline = Instant::now() + std::time::Duration::from_secs(duration_secs);
        let mut samples = Vec::with_capacity(200_000);
        while Instant::now() < deadline {
            let s = Instant::now();
            publisher.put(&payload).res().await.unwrap();
            let _ = subscriber.recv_async().await;
            samples.push(s.elapsed().as_nanos() as u64);
        }
        samples
    })
}

#[cfg(not(feature = "zenoh"))]
fn bench_zenoh(_size: usize, _duration_secs: u64) -> Vec<u64> {
    Vec::new() // Zenoh not available
}

// ============================================================================
// Main
// ============================================================================

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
    let has_zenoh = cfg!(feature = "zenoh");

    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║          Competitor Comparison                              ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();
    println!(
        "Platform: {}, {} cores",
        platform.cpu.model, platform.cpu.logical_cores
    );
    println!("Duration: {}s per test", duration);
    println!(
        "Zenoh: {}",
        if has_zenoh {
            "enabled"
        } else {
            "not available (compile with --features zenoh)"
        }
    );
    println!();

    let sizes = [8usize, 32];
    let mut csv_rows: Vec<(String, usize, Stats)> = Vec::new();

    // Header
    println!(
        "{:<16} {:>6} {:>10} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "Competitor", "Size", "Samples", "p50", "p95", "p99", "p999", "max"
    );
    println!("{}", "─".repeat(80));

    for &size in &sizes {
        // HORUS
        let mut horus_samples = bench_horus(size, duration);
        let horus_stats = compute_stats(&mut horus_samples);
        let label = format!("{}B", size);
        println!(
            "{:<16} {:>6} {:>9}K {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
            "HORUS SHM",
            label,
            horus_stats.count / 1000,
            horus_stats.p50,
            horus_stats.p95,
            horus_stats.p99,
            horus_stats.p999,
            horus_stats.max
        );
        csv_rows.push(("HORUS_SHM".into(), size, horus_stats));

        // Raw UDP
        let mut udp_samples = bench_raw_udp(size, duration);
        let udp_stats = compute_stats(&mut udp_samples);
        println!(
            "{:<16} {:>6} {:>9}K {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
            "Raw UDP",
            label,
            udp_stats.count / 1000,
            udp_stats.p50,
            udp_stats.p95,
            udp_stats.p99,
            udp_stats.p999,
            udp_stats.max
        );
        csv_rows.push(("Raw_UDP".into(), size, udp_stats));

        // Zenoh
        let mut zenoh_samples = bench_zenoh(size, duration);
        if !zenoh_samples.is_empty() {
            let zenoh_stats = compute_stats(&mut zenoh_samples);
            println!(
                "{:<16} {:>6} {:>9}K {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
                "Zenoh",
                label,
                zenoh_stats.count / 1000,
                zenoh_stats.p50,
                zenoh_stats.p95,
                zenoh_stats.p99,
                zenoh_stats.p999,
                zenoh_stats.max
            );
            csv_rows.push(("Zenoh".into(), size, zenoh_stats));
        }

        println!("{}", "─".repeat(80));
    }

    // Speedup summary
    println!();
    println!("Speedup (HORUS vs Raw UDP):");
    for i in (0..csv_rows.len()).step_by(if has_zenoh { 3 } else { 2 }) {
        let horus = &csv_rows[i];
        let udp = &csv_rows[i + 1];
        if udp.2.p50 > 0 {
            println!(
                "  {}B: {:.0}x faster ({}ns vs {}ns p50)",
                horus.1,
                udp.2.p50 as f64 / horus.2.p50.max(1) as f64,
                horus.2.p50,
                udp.2.p50
            );
        }
    }

    // CSV
    if let Some(path) = csv_path {
        let mut f = std::io::BufWriter::new(std::fs::File::create(&path).unwrap());
        writeln!(
            f,
            "competitor,msg_size_bytes,samples,p50_ns,p95_ns,p99_ns,p999_ns,max_ns,mean_ns"
        )
        .unwrap();
        for (name, size, stats) in &csv_rows {
            writeln!(
                f,
                "{},{},{},{},{},{},{},{},{}",
                name,
                size,
                stats.count,
                stats.p50,
                stats.p95,
                stats.p99,
                stats.p999,
                stats.max,
                stats.mean
            )
            .unwrap();
        }
        f.flush().unwrap();
        println!("\nCSV written: {path}");
    }

    println!("\nDone.");
}
