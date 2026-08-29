//! Competitor Comparison Benchmark
//!
//! Measures HORUS against raw UDP on the loopback interface. Both arms use the
//! same timing method and the same loop shape.
//!
//! There is no Zenoh arm. The header used to say "and optionally Zenoh"; no
//! such code has ever existed in this file, and there is no feature flag that
//! adds one.
//!
//! # Topology — read before quoting the speedup
//!
//! Both arms run **entirely on one thread**: send, then block until the message
//! comes back to the same thread. So:
//!
//! - the HORUS arm is a userspace ring write followed by a read of a line that
//!   thread just wrote, almost always an L1 hit. There is no cross-core cache
//!   transfer and no second process in it. It is a lower bound on IPC cost, not
//!   IPC cost.
//! - the UDP arm is a real trip through the kernel network stack (two sockets,
//!   two syscalls, softirq) even though it too stays on one thread.
//!
//! The ratio between them is therefore "userspace ring vs kernel loopback",
//! which is a real and large difference, but it is NOT "HORUS end-to-end vs UDP
//! end-to-end". A HORUS publisher and subscriber in two processes on two cores
//! pay a cache-line transfer this benchmark never makes them pay. Use
//! `cross_process_benchmark` for that number.
//!
//! Run:   cargo run --release -p horus_benchmarks --bin competitor_comparison
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
    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║          Competitor Comparison                              ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();
    println!(
        "Platform: {}, {} cores",
        platform.cpu.model, platform.cpu.logical_cores
    );
    println!("Duration: {}s per test", duration);
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
            "HORUS same-thd",
            label,
            horus_stats.count / 1000,
            horus_stats.p50,
            horus_stats.p95,
            horus_stats.p99,
            horus_stats.p999,
            horus_stats.max
        );
        csv_rows.push(("HORUS_same_thread".into(), size, horus_stats));

        // Raw UDP
        let mut udp_samples = bench_raw_udp(size, duration);
        let udp_stats = compute_stats(&mut udp_samples);
        println!(
            "{:<16} {:>6} {:>9}K {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
            "Raw UDP loopbk",
            label,
            udp_stats.count / 1000,
            udp_stats.p50,
            udp_stats.p95,
            udp_stats.p99,
            udp_stats.p999,
            udp_stats.max
        );
        csv_rows.push(("Raw_UDP_loopback".into(), size, udp_stats));

        println!("{}", "─".repeat(80));
    }

    // Ratio summary.
    //
    // This used to print a single "Nx faster" from the p50 alone. A median-only
    // ratio cannot show a tail regression: HORUS could double its p999 and this
    // line would not move. The p99, p999 and max ratios are printed beside it.
    println!();
    println!("Ratio, UDP loopback / HORUS same-thread (higher = HORUS cheaper).");
    println!("Different topologies — see the module header before quoting these.");
    for i in (0..csv_rows.len()).step_by(2) {
        let horus = &csv_rows[i];
        let udp = &csv_rows[i + 1];
        if udp.2.p50 > 0 {
            let r = |u: u64, h: u64| u as f64 / h.max(1) as f64;
            println!(
                "  {:>4}B: p50 {:>6.0}x  p99 {:>6.0}x  p999 {:>6.0}x  max {:>6.1}x",
                horus.1,
                r(udp.2.p50, horus.2.p50),
                r(udp.2.p99, horus.2.p99),
                r(udp.2.p999, horus.2.p999),
                r(udp.2.max, horus.2.max),
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
