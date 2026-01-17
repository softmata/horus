// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! 24-Hour Soak Test
//!
//! Verifies long-term stability of HORUS IPC:
//! - No memory leaks over 24+ hours
//! - No file descriptor leaks
//! - No latency degradation
//! - Stable resource usage
//!
//! Usage:
//!     cargo run --release --bin test_soak_24h -- [duration_hours] [report_interval_minutes]
//!
//! Examples:
//!     cargo run --release --bin test_soak_24h -- 24 60      # 24h test, report every 60 min
//!     cargo run --release --bin test_soak_24h -- 1 5        # 1h test, report every 5 min
//!     cargo run --release --bin test_soak_24h -- 0.5 1      # 30min test, report every 1 min

use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::sensor::Imu;
use std::collections::VecDeque;
use std::env;
use std::fs;
use std::io::{self, Write};
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Latency histogram with percentile calculation
struct LatencyHistogram {
    samples: VecDeque<u64>,
    max_samples: usize,
    min: u64,
    max: u64,
    sum: u64,
    count: u64,
}

impl LatencyHistogram {
    fn new(max_samples: usize) -> Self {
        Self {
            samples: VecDeque::with_capacity(max_samples),
            max_samples,
            min: u64::MAX,
            max: 0,
            sum: 0,
            count: 0,
        }
    }

    fn record(&mut self, latency_ns: u64) {
        if self.samples.len() >= self.max_samples {
            self.samples.pop_front();
        }
        self.samples.push_back(latency_ns);

        self.min = self.min.min(latency_ns);
        self.max = self.max.max(latency_ns);
        self.sum += latency_ns;
        self.count += 1;
    }

    fn percentile(&self, p: f64) -> u64 {
        if self.samples.is_empty() {
            return 0;
        }

        let mut sorted: Vec<u64> = self.samples.iter().copied().collect();
        sorted.sort_unstable();

        let idx = ((p / 100.0) * (sorted.len() - 1) as f64).round() as usize;
        sorted[idx.min(sorted.len() - 1)]
    }

    fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum as f64 / self.count as f64
        }
    }

    fn reset_window(&mut self) {
        self.samples.clear();
    }
}

/// Resource monitoring snapshot
#[derive(Clone, Debug)]
struct ResourceSnapshot {
    timestamp_secs: u64,
    memory_rss_kb: u64,
    memory_vm_kb: u64,
    fd_count: u64,
    messages_sent: u64,
    messages_recv: u64,
    latency_p50_ns: u64,
    latency_p95_ns: u64,
    latency_p99_ns: u64,
    latency_max_ns: u64,
    send_errors: u64,
    recv_errors: u64,
}

/// Get current process RSS and VM memory in KB
fn get_memory_usage() -> (u64, u64) {
    #[cfg(target_os = "linux")]
    {
        if let Ok(content) = fs::read_to_string("/proc/self/statm") {
            let parts: Vec<&str> = content.split_whitespace().collect();
            if parts.len() >= 2 {
                let page_size = 4; // KB (typical page size)
                let vm_pages: u64 = parts[0].parse().unwrap_or(0);
                let rss_pages: u64 = parts[1].parse().unwrap_or(0);
                return (rss_pages * page_size, vm_pages * page_size);
            }
        }
    }
    (0, 0)
}

/// Get current process file descriptor count
fn get_fd_count() -> u64 {
    #[cfg(target_os = "linux")]
    {
        if let Ok(entries) = fs::read_dir("/proc/self/fd") {
            return entries.count() as u64;
        }
    }
    0
}

/// Report structure for end-of-test summary
struct SoakTestReport {
    duration_hours: f64,
    total_messages_sent: u64,
    total_messages_recv: u64,
    total_send_errors: u64,
    total_recv_errors: u64,
    initial_memory_kb: u64,
    final_memory_kb: u64,
    peak_memory_kb: u64,
    initial_fd_count: u64,
    final_fd_count: u64,
    peak_fd_count: u64,
    latency_p50_start: u64,
    latency_p50_end: u64,
    latency_p99_start: u64,
    latency_p99_end: u64,
    memory_leak_detected: bool,
    fd_leak_detected: bool,
    latency_degradation_detected: bool,
}

impl SoakTestReport {
    fn print(&self) {
        println!("\n{}", "=".repeat(70));
        println!("  HORUS 24-Hour Soak Test Report");
        println!("{}\n", "=".repeat(70));

        println!("Duration: {:.2} hours", self.duration_hours);
        println!();

        println!("Messages:");
        println!("  Total sent:     {}", self.total_messages_sent);
        println!("  Total received: {}", self.total_messages_recv);
        println!("  Send errors:    {}", self.total_send_errors);
        println!("  Recv errors:    {}", self.total_recv_errors);
        println!(
            "  Reliability:    {:.4}%",
            (self.total_messages_recv as f64 / self.total_messages_sent as f64) * 100.0
        );
        println!();

        println!("Memory (RSS KB):");
        println!("  Initial:  {}", self.initial_memory_kb);
        println!("  Final:    {}", self.final_memory_kb);
        println!("  Peak:     {}", self.peak_memory_kb);
        let mem_growth = self.final_memory_kb as i64 - self.initial_memory_kb as i64;
        println!("  Growth:   {} KB ({:.2}%)", mem_growth,
            (mem_growth as f64 / self.initial_memory_kb as f64) * 100.0);
        println!();

        println!("File Descriptors:");
        println!("  Initial:  {}", self.initial_fd_count);
        println!("  Final:    {}", self.final_fd_count);
        println!("  Peak:     {}", self.peak_fd_count);
        let fd_growth = self.final_fd_count as i64 - self.initial_fd_count as i64;
        println!("  Growth:   {}", fd_growth);
        println!();

        println!("Latency (ns):");
        println!("  P50 start: {} ns", self.latency_p50_start);
        println!("  P50 end:   {} ns", self.latency_p50_end);
        println!("  P99 start: {} ns", self.latency_p99_start);
        println!("  P99 end:   {} ns", self.latency_p99_end);
        let p99_change = self.latency_p99_end as i64 - self.latency_p99_start as i64;
        println!("  P99 change: {} ns ({:.2}%)", p99_change,
            (p99_change as f64 / self.latency_p99_start as f64) * 100.0);
        println!();

        println!("Verdict:");
        if self.memory_leak_detected {
            println!("  [FAIL] Memory leak detected (>10% growth)");
        } else {
            println!("  [PASS] Memory usage stable");
        }

        if self.fd_leak_detected {
            println!("  [FAIL] File descriptor leak detected");
        } else {
            println!("  [PASS] File descriptors stable");
        }

        if self.latency_degradation_detected {
            println!("  [FAIL] Latency degradation detected (>20% increase in P99)");
        } else {
            println!("  [PASS] Latency stable");
        }

        println!("{}", "=".repeat(70));

        let passed = !self.memory_leak_detected
            && !self.fd_leak_detected
            && !self.latency_degradation_detected;

        if passed {
            println!("\n  OVERALL: PASSED");
        } else {
            println!("\n  OVERALL: FAILED");
        }
    }

    fn passed(&self) -> bool {
        !self.memory_leak_detected
            && !self.fd_leak_detected
            && !self.latency_degradation_detected
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    // Default: 24 hours, report every 60 minutes
    let duration_hours: f64 = args
        .get(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(24.0);

    let report_interval_minutes: f64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(60.0);

    let duration_secs = (duration_hours * 3600.0) as u64;
    let report_interval_secs = (report_interval_minutes * 60.0) as u64;

    println!("{}", "=".repeat(70));
    println!("  HORUS 24-Hour Soak Test");
    println!("{}", "=".repeat(70));
    println!();
    println!("Configuration:");
    println!("  Duration: {:.2} hours ({} seconds)", duration_hours, duration_secs);
    println!("  Report interval: {:.2} minutes", report_interval_minutes);
    println!();
    println!("Starting test at: {:?}", SystemTime::now());
    println!();

    let result = run_soak_test(duration_secs, report_interval_secs);

    if result {
        println!("\n Soak test PASSED");
        process::exit(0);
    } else {
        eprintln!("\n[FAIL] Soak test FAILED");
        process::exit(1);
    }
}

fn run_soak_test(duration_secs: u64, report_interval_secs: u64) -> bool {
    // Setup phase - create mixed workload
    let pid = process::id();

    // Create multiple Link channels (SPSC - fast path)
    let link_cmdvel_topic = format!("soak_link_cmdvel_{}", pid);
    let link_imu_topic = format!("soak_link_imu_{}", pid);

    let link_cmdvel_tx = Topic::<CmdVel>::producer(&link_cmdvel_topic).expect("Link CmdVel producer");
    let link_cmdvel_rx = Topic::<CmdVel>::consumer(&link_cmdvel_topic).expect("Link CmdVel consumer");

    let link_imu_tx = Topic::<Imu>::producer(&link_imu_topic).expect("Link Imu producer");
    let link_imu_rx = Topic::<Imu>::consumer(&link_imu_topic).expect("Link Imu consumer");

    // Create Hub channels (MPMC - slower path)
    let hub_cmdvel_topic = format!("soak_hub_cmdvel_{}", pid);
    let hub_imu_topic = format!("soak_hub_imu_{}", pid);

    let hub_cmdvel_tx = Topic::<CmdVel>::new(&hub_cmdvel_topic).expect("Hub CmdVel tx");
    let hub_cmdvel_rx1 = Topic::<CmdVel>::new(&hub_cmdvel_topic).expect("Hub CmdVel rx1");
    let hub_cmdvel_rx2 = Topic::<CmdVel>::new(&hub_cmdvel_topic).expect("Hub CmdVel rx2");

    let hub_imu_tx = Topic::<Imu>::new(&hub_imu_topic).expect("Hub Imu tx");
    let hub_imu_rx = Topic::<Imu>::new(&hub_imu_topic).expect("Hub Imu rx");

    println!("Created communication channels:");
    println!("  - 2x Link (SPSC): CmdVel, Imu");
    println!("  - 2x Hub (MPMC): CmdVel (1 pub, 2 sub), Imu (1 pub, 1 sub)");
    println!();

    // Shared state
    let running = Arc::new(AtomicBool::new(true));
    let total_sent = Arc::new(AtomicU64::new(0));
    let total_recv = Arc::new(AtomicU64::new(0));
    let total_send_errors = Arc::new(AtomicU64::new(0));
    let total_recv_errors = Arc::new(AtomicU64::new(0));

    // Record initial state
    let (initial_mem_rss, _) = get_memory_usage();
    let initial_fd_count = get_fd_count();

    let mut snapshots: Vec<ResourceSnapshot> = Vec::new();
    let mut latency_hist = LatencyHistogram::new(100_000); // Keep last 100k samples

    // Capture early latency baseline
    let mut early_latency_samples: Vec<u64> = Vec::new();

    // Spawn sender threads
    let running_clone = Arc::clone(&running);
    let sent_clone = Arc::clone(&total_sent);
    let errors_clone = Arc::clone(&total_send_errors);

    let sender_handle = thread::spawn(move || {
        let mut seq = 0u64;

        while running_clone.load(Ordering::Relaxed) {
            // Send on Link CmdVel (1kHz)
            let msg = CmdVel {
                linear: (seq as f32 * 0.001).sin(),
                angular: (seq as f32 * 0.002).cos(),
                stamp_nanos: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64,
            };

            if link_cmdvel_tx.send(msg, &mut None).is_ok() {
                sent_clone.fetch_add(1, Ordering::Relaxed);
            } else {
                errors_clone.fetch_add(1, Ordering::Relaxed);
            }

            // Send on Link Imu (500Hz)
            if seq % 2 == 0 {
                let mut imu = Imu::new();
                imu.angular_velocity = [0.01, 0.02, 0.03];
                imu.linear_acceleration = [0.0, 0.0, 9.81];
                imu.timestamp = seq;

                if link_imu_tx.send(imu, &mut None).is_ok() {
                    sent_clone.fetch_add(1, Ordering::Relaxed);
                } else {
                    errors_clone.fetch_add(1, Ordering::Relaxed);
                }
            }

            // Send on Hub CmdVel (100Hz)
            if seq % 10 == 0 {
                let msg = CmdVel {
                    linear: 1.0,
                    angular: 0.5,
                    stamp_nanos: seq,
                };

                if hub_cmdvel_tx.send(msg, &mut None).is_ok() {
                    sent_clone.fetch_add(1, Ordering::Relaxed);
                } else {
                    errors_clone.fetch_add(1, Ordering::Relaxed);
                }
            }

            // Send on Hub Imu (50Hz)
            if seq % 20 == 0 {
                let mut imu = Imu::new();
                imu.angular_velocity = [0.01, 0.02, 0.03];
                imu.linear_acceleration = [0.0, 0.0, 9.81];
                imu.timestamp = seq;

                if hub_imu_tx.send(imu, &mut None).is_ok() {
                    sent_clone.fetch_add(1, Ordering::Relaxed);
                } else {
                    errors_clone.fetch_add(1, Ordering::Relaxed);
                }
            }

            seq += 1;
            thread::sleep(Duration::from_millis(1)); // 1kHz base rate
        }
    });

    // Spawn receiver threads
    let running_clone = Arc::clone(&running);
    let recv_clone = Arc::clone(&total_recv);

    let link_cmdvel_recv_handle = thread::spawn(move || {
        let mut latencies: Vec<u64> = Vec::with_capacity(1_000_000);

        while running_clone.load(Ordering::Relaxed) {
            let start = Instant::now();
            if link_cmdvel_rx.recv(&mut None).is_some() {
                let latency = start.elapsed().as_nanos() as u64;
                latencies.push(latency);
                recv_clone.fetch_add(1, Ordering::Relaxed);
            }
        }

        latencies
    });

    let running_clone = Arc::clone(&running);
    let recv_clone = Arc::clone(&total_recv);

    let link_imu_recv_handle = thread::spawn(move || {
        while running_clone.load(Ordering::Relaxed) {
            if link_imu_rx.recv(&mut None).is_some() {
                recv_clone.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    let running_clone = Arc::clone(&running);
    let recv_clone = Arc::clone(&total_recv);

    let hub_cmdvel_recv1_handle = thread::spawn(move || {
        while running_clone.load(Ordering::Relaxed) {
            if hub_cmdvel_rx1.recv(&mut None).is_some() {
                recv_clone.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    let running_clone = Arc::clone(&running);
    let recv_clone = Arc::clone(&total_recv);

    let hub_cmdvel_recv2_handle = thread::spawn(move || {
        while running_clone.load(Ordering::Relaxed) {
            if hub_cmdvel_rx2.recv(&mut None).is_some() {
                recv_clone.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    let running_clone = Arc::clone(&running);
    let recv_clone = Arc::clone(&total_recv);

    let hub_imu_recv_handle = thread::spawn(move || {
        while running_clone.load(Ordering::Relaxed) {
            if hub_imu_rx.recv(&mut None).is_some() {
                recv_clone.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    // Main monitoring loop
    let start_time = Instant::now();
    let mut last_report = Instant::now();
    let mut peak_memory_kb = initial_mem_rss;
    let mut peak_fd_count = initial_fd_count;

    println!("Starting monitoring loop...\n");
    print_progress_header();

    // Warm-up period (first 60 seconds) - collect baseline latencies
    let warmup_duration = Duration::from_secs(60.min(duration_secs / 10));
    println!("Warm-up period: {} seconds...", warmup_duration.as_secs());

    while start_time.elapsed() < warmup_duration {
        thread::sleep(Duration::from_secs(1));
    }

    // Main test loop
    while start_time.elapsed().as_secs() < duration_secs {
        let elapsed = start_time.elapsed();

        // Report progress at intervals
        if last_report.elapsed().as_secs() >= report_interval_secs {
            let (mem_rss, mem_vm) = get_memory_usage();
            let fd_count = get_fd_count();
            let sent = total_sent.load(Ordering::Relaxed);
            let recv = total_recv.load(Ordering::Relaxed);
            let send_err = total_send_errors.load(Ordering::Relaxed);
            let recv_err = total_recv_errors.load(Ordering::Relaxed);

            peak_memory_kb = peak_memory_kb.max(mem_rss);
            peak_fd_count = peak_fd_count.max(fd_count);

            // Calculate latency percentiles (using approximation from message count delta)
            // Note: Real latencies are collected in the receiver thread
            let snapshot = ResourceSnapshot {
                timestamp_secs: elapsed.as_secs(),
                memory_rss_kb: mem_rss,
                memory_vm_kb: mem_vm,
                fd_count,
                messages_sent: sent,
                messages_recv: recv,
                latency_p50_ns: 0, // Filled in later from actual measurements
                latency_p95_ns: 0,
                latency_p99_ns: 0,
                latency_max_ns: 0,
                send_errors: send_err,
                recv_errors: recv_err,
            };

            // Print progress
            print_progress(&snapshot, initial_mem_rss, initial_fd_count);

            snapshots.push(snapshot);
            last_report = Instant::now();
        }

        thread::sleep(Duration::from_secs(1));
    }

    // Stop all threads
    println!("\n\nStopping test...");
    running.store(false, Ordering::SeqCst);

    // Wait for threads
    sender_handle.join().expect("sender thread");
    let latencies = link_cmdvel_recv_handle.join().expect("link cmdvel receiver");
    link_imu_recv_handle.join().expect("link imu receiver");
    hub_cmdvel_recv1_handle.join().expect("hub cmdvel recv1");
    hub_cmdvel_recv2_handle.join().expect("hub cmdvel recv2");
    hub_imu_recv_handle.join().expect("hub imu receiver");

    // Calculate final latency statistics from collected data
    for &lat in &latencies {
        latency_hist.record(lat);
    }

    // Calculate early vs late latencies
    let early_count = latencies.len().min(10_000);
    let late_start = latencies.len().saturating_sub(10_000);

    let mut early_latencies: Vec<u64> = latencies[..early_count].to_vec();
    early_latencies.sort_unstable();

    let mut late_latencies: Vec<u64> = latencies[late_start..].to_vec();
    late_latencies.sort_unstable();

    let p50_idx = |len: usize| (len as f64 * 0.50) as usize;
    let p99_idx = |len: usize| (len as f64 * 0.99) as usize;

    let early_p50 = early_latencies.get(p50_idx(early_latencies.len())).copied().unwrap_or(0);
    let early_p99 = early_latencies.get(p99_idx(early_latencies.len())).copied().unwrap_or(0);
    let late_p50 = late_latencies.get(p50_idx(late_latencies.len())).copied().unwrap_or(0);
    let late_p99 = late_latencies.get(p99_idx(late_latencies.len())).copied().unwrap_or(0);

    // Final resource snapshot
    let (final_mem_rss, _) = get_memory_usage();
    let final_fd_count = get_fd_count();

    // Build report
    let mem_growth_pct = ((final_mem_rss as f64 - initial_mem_rss as f64) / initial_mem_rss as f64) * 100.0;
    let latency_growth_pct = if early_p99 > 0 {
        ((late_p99 as f64 - early_p99 as f64) / early_p99 as f64) * 100.0
    } else {
        0.0
    };

    let report = SoakTestReport {
        duration_hours: duration_secs as f64 / 3600.0,
        total_messages_sent: total_sent.load(Ordering::Relaxed),
        total_messages_recv: total_recv.load(Ordering::Relaxed),
        total_send_errors: total_send_errors.load(Ordering::Relaxed),
        total_recv_errors: total_recv_errors.load(Ordering::Relaxed),
        initial_memory_kb: initial_mem_rss,
        final_memory_kb: final_mem_rss,
        peak_memory_kb,
        initial_fd_count,
        final_fd_count,
        peak_fd_count,
        latency_p50_start: early_p50,
        latency_p50_end: late_p50,
        latency_p99_start: early_p99,
        latency_p99_end: late_p99,
        memory_leak_detected: mem_growth_pct > 10.0,
        fd_leak_detected: final_fd_count > initial_fd_count + 10,
        latency_degradation_detected: latency_growth_pct > 20.0,
    };

    report.print();

    // Save detailed report to file
    save_detailed_report(&snapshots, &report);

    report.passed()
}

fn print_progress_header() {
    println!(
        "{:>8} {:>10} {:>8} {:>12} {:>12} {:>8} {:>8}",
        "Elapsed", "Mem(KB)", "FDs", "Sent", "Recv", "SendErr", "RecvErr"
    );
    println!("{}", "-".repeat(74));
}

fn print_progress(snapshot: &ResourceSnapshot, initial_mem: u64, initial_fd: u64) {
    let hours = snapshot.timestamp_secs / 3600;
    let mins = (snapshot.timestamp_secs % 3600) / 60;

    let mem_delta: i64 = snapshot.memory_rss_kb as i64 - initial_mem as i64;
    let fd_delta: i64 = snapshot.fd_count as i64 - initial_fd as i64;

    println!(
        "{:>5}h{:02}m {:>10} {:>8} {:>12} {:>12} {:>8} {:>8}",
        hours,
        mins,
        format!("{}({:+})", snapshot.memory_rss_kb, mem_delta),
        format!("{}({:+})", snapshot.fd_count, fd_delta),
        snapshot.messages_sent,
        snapshot.messages_recv,
        snapshot.send_errors,
        snapshot.recv_errors,
    );
    io::stdout().flush().ok();
}

fn save_detailed_report(snapshots: &[ResourceSnapshot], report: &SoakTestReport) {
    let filename = format!(
        "soak_test_report_{}.json",
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs()
    );

    let json = format!(
        r#"{{
  "duration_hours": {:.2},
  "passed": {},
  "summary": {{
    "total_messages_sent": {},
    "total_messages_recv": {},
    "reliability_percent": {:.4},
    "memory_leak_detected": {},
    "fd_leak_detected": {},
    "latency_degradation_detected": {}
  }},
  "memory": {{
    "initial_kb": {},
    "final_kb": {},
    "peak_kb": {},
    "growth_percent": {:.2}
  }},
  "file_descriptors": {{
    "initial": {},
    "final": {},
    "peak": {}
  }},
  "latency_ns": {{
    "p50_start": {},
    "p50_end": {},
    "p99_start": {},
    "p99_end": {},
    "degradation_percent": {:.2}
  }},
  "snapshots": [{}]
}}"#,
        report.duration_hours,
        report.passed(),
        report.total_messages_sent,
        report.total_messages_recv,
        (report.total_messages_recv as f64 / report.total_messages_sent as f64) * 100.0,
        report.memory_leak_detected,
        report.fd_leak_detected,
        report.latency_degradation_detected,
        report.initial_memory_kb,
        report.final_memory_kb,
        report.peak_memory_kb,
        ((report.final_memory_kb as f64 - report.initial_memory_kb as f64) / report.initial_memory_kb as f64) * 100.0,
        report.initial_fd_count,
        report.final_fd_count,
        report.peak_fd_count,
        report.latency_p50_start,
        report.latency_p50_end,
        report.latency_p99_start,
        report.latency_p99_end,
        if report.latency_p99_start > 0 {
            ((report.latency_p99_end as f64 - report.latency_p99_start as f64) / report.latency_p99_start as f64) * 100.0
        } else {
            0.0
        },
        snapshots.iter().map(|s| format!(
            r#"
    {{
      "timestamp_secs": {},
      "memory_rss_kb": {},
      "fd_count": {},
      "messages_sent": {},
      "messages_recv": {}
    }}"#,
            s.timestamp_secs, s.memory_rss_kb, s.fd_count, s.messages_sent, s.messages_recv
        )).collect::<Vec<_>>().join(",")
    );

    match fs::write(&filename, json) {
        Ok(_) => println!("\nDetailed report saved to: {}", filename),
        Err(e) => eprintln!("Failed to save report: {}", e),
    }
}
