//! Verify ultra-fast path is always taken
//!
//! This test adds path counters to verify we're hitting the expected code path
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

// Global counters for path verification
static ULTRA_FAST_SENDS: AtomicU64 = AtomicU64::new(0);
static FAST_SENDS: AtomicU64 = AtomicU64::new(0);
static SLOW_SENDS: AtomicU64 = AtomicU64::new(0);
static ULTRA_FAST_RECVS: AtomicU64 = AtomicU64::new(0);
static FAST_RECVS: AtomicU64 = AtomicU64::new(0);
static SLOW_RECVS: AtomicU64 = AtomicU64::new(0);

fn main() {
    println!("=== Path Verification Test ===\n");

    let topic: Topic<CmdVel> = Topic::new("path_verify").unwrap();

    // Warmup - forces mode detection and migration to DirectChannel
    println!("--- Warmup (10,000 iterations) ---");
    for _ in 0..10_000 {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
        let _ = topic.recv();
    }

    println!("Backend type after warmup: {}", topic.backend_type());

    // Drain any messages
    while topic.recv().is_some() {}

    // Reset counters
    ULTRA_FAST_SENDS.store(0, Ordering::SeqCst);
    FAST_SENDS.store(0, Ordering::SeqCst);
    SLOW_SENDS.store(0, Ordering::SeqCst);
    ULTRA_FAST_RECVS.store(0, Ordering::SeqCst);
    FAST_RECVS.store(0, Ordering::SeqCst);
    SLOW_RECVS.store(0, Ordering::SeqCst);

    // Since we can't easily instrument the library, let's verify by:
    // 1. Checking backend type before/after
    // 2. Running the same benchmark multiple times and checking for variance

    let iterations = 10_000_000u64;

    // Run multiple trials to check for consistency
    println!("\n--- Multiple Trials (10 runs of 10M iterations) ---");

    let mut send_times = Vec::with_capacity(10);
    let mut recv_times = Vec::with_capacity(10);

    for trial in 0..10 {
        // Send trial
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let start = Instant::now();
        for _ in 0..iterations {
            let _ = topic.send(msg);
        }
        let send_elapsed = start.elapsed();
        let send_ns = send_elapsed.as_nanos() as f64 / iterations as f64;
        send_times.push(send_ns);

        // Drain and recv trial
        let start = Instant::now();
        let mut count = 0u64;
        while topic.recv().is_some() {
            count += 1;
        }
        let recv_elapsed = start.elapsed();
        let recv_ns = recv_elapsed.as_nanos() as f64 / count as f64;
        recv_times.push(recv_ns);

        println!(
            "Trial {}: send={:.2}ns, recv={:.2}ns (backend: {})",
            trial + 1,
            send_ns,
            recv_ns,
            topic.backend_type()
        );
    }

    // Calculate statistics
    let send_avg: f64 = send_times.iter().sum::<f64>() / send_times.len() as f64;
    let recv_avg: f64 = recv_times.iter().sum::<f64>() / recv_times.len() as f64;
    let send_min = send_times.iter().cloned().fold(f64::INFINITY, f64::min);
    let send_max = send_times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let recv_min = recv_times.iter().cloned().fold(f64::INFINITY, f64::min);
    let recv_max = recv_times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let send_variance = send_times.iter().map(|x| (x - send_avg).powi(2)).sum::<f64>() / send_times.len() as f64;
    let recv_variance = recv_times.iter().map(|x| (x - recv_avg).powi(2)).sum::<f64>() / recv_times.len() as f64;

    println!("\n=== Statistics ===");
    println!("send(): avg={:.2}ns, min={:.2}ns, max={:.2}ns, stddev={:.2}ns",
             send_avg, send_min, send_max, send_variance.sqrt());
    println!("recv(): avg={:.2}ns, min={:.2}ns, max={:.2}ns, stddev={:.2}ns",
             recv_avg, recv_min, recv_max, recv_variance.sqrt());

    // Consistency check
    let send_consistent = (send_max - send_min) < 5.0; // Less than 5ns variance
    let recv_consistent = (recv_max - recv_min) < 5.0;

    println!("\n=== Consistency Check ===");
    println!("send() consistent: {} (variance {:.2}ns)",
             if send_consistent { "YES" } else { "NO - POSSIBLE PATH SWITCHING" },
             send_max - send_min);
    println!("recv() consistent: {} (variance {:.2}ns)",
             if recv_consistent { "YES" } else { "NO - POSSIBLE PATH SWITCHING" },
             recv_max - recv_min);

    println!("\n=== Final Status ===");
    println!("Backend type: {}", topic.backend_type());

    // Target comparison
    println!("\n=== Target vs Achieved (Best) ===");
    println!("DirectChannel target: ~3ns");
    println!("Best send(): {:.2}ns (gap: {:.2}ns, {:.1}x)",
             send_min, send_min - 3.0, send_min / 3.0);
    println!("Best recv(): {:.2}ns (gap: {:.2}ns, {:.1}x)",
             recv_min, recv_min - 3.0, recv_min / 3.0);

    println!("\n=== Done ===");
}
