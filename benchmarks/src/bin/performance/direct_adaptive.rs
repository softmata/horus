//! Direct AdaptiveTopic benchmark - bypass Topic wrapper
//!
//! Tests if using AdaptiveTopic directly (without Topic wrapper) achieves lower latency
use horus_core::communication::adaptive_topic::AdaptiveTopic;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Instant;

fn main() {
    println!("=== Direct AdaptiveTopic Latency Test ===\n");

    // Create AdaptiveTopic directly, bypassing Topic wrapper
    let topic: AdaptiveTopic<CmdVel> = AdaptiveTopic::new("direct_test").unwrap();

    // Warmup - forces mode detection and migration to DirectChannel
    println!("--- Warmup (10,000 iterations) ---");
    for _ in 0..10_000 {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
        let _ = topic.recv();
    }

    println!("Backend type after warmup: {:?}", topic.mode());

    // Drain any messages
    while topic.recv().is_some() {}

    let iterations = 10_000_000u64;

    // Run multiple trials
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
            "Trial {}: send={:.2}ns, recv={:.2}ns",
            trial + 1,
            send_ns,
            recv_ns
        );
    }

    // Calculate statistics
    let send_avg: f64 = send_times.iter().sum::<f64>() / send_times.len() as f64;
    let recv_avg: f64 = recv_times.iter().sum::<f64>() / recv_times.len() as f64;
    let send_min = send_times.iter().cloned().fold(f64::INFINITY, f64::min);
    let send_max = send_times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let recv_min = recv_times.iter().cloned().fold(f64::INFINITY, f64::min);
    let recv_max = recv_times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    println!("\n=== Direct AdaptiveTopic Statistics ===");
    println!("send(): avg={:.2}ns, min={:.2}ns, max={:.2}ns", send_avg, send_min, send_max);
    println!("recv(): avg={:.2}ns, min={:.2}ns, max={:.2}ns", recv_avg, recv_min, recv_max);

    println!("\n=== Target vs Achieved (Best) ===");
    println!("DirectChannel target: ~3ns");
    println!("Direct AdaptiveTopic send(): {:.2}ns (gap: {:.2}ns, {:.1}x)",
             send_min, send_min - 3.0, send_min / 3.0);
    println!("Direct AdaptiveTopic recv(): {:.2}ns (gap: {:.2}ns, {:.1}x)",
             recv_min, recv_min - 3.0, recv_min / 3.0);

    println!("\n=== Done ===");
}
