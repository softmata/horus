//! Minimal latency test - absolute minimum overhead measurement
//!
//! Tests send/recv without any black_box or extra constructs that might add overhead
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Instant;

fn main() {
    println!("=== Minimal Ultra-Fast Path Latency Test ===\n");

    let topic: Topic<CmdVel> = Topic::new("minimal_latency").unwrap();

    // Warmup - forces mode detection and migration to DirectChannel
    for _ in 0..10_000 {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
        let _ = topic.recv();
    }

    println!("Backend type after warmup: {}", topic.backend_type());

    // Drain any messages
    while topic.recv().is_some() {}

    let iterations = 10_000_000u64;

    // ============================================================
    // Test 1: send() without black_box
    // ============================================================
    println!("\n--- Test 1: send() without black_box ---");

    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = topic.send(msg);
    }
    let send_elapsed = start.elapsed();
    let send_ns = send_elapsed.as_nanos() as f64 / iterations as f64;
    println!("send(): {:.2}ns", send_ns);

    // Drain
    let mut drain_count = 0u64;
    while topic.recv().is_some() {
        drain_count += 1;
    }
    println!("(drained {} messages)", drain_count);

    // ============================================================
    // Test 2: recv() without black_box (same structure as send)
    // ============================================================
    println!("\n--- Test 2: recv() without black_box ---");

    // Send messages first
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    for _ in 0..iterations {
        let _ = topic.send(msg);
    }

    let start = Instant::now();
    let mut count = 0u64;
    while topic.recv().is_some() {
        count += 1;
    }
    let recv_elapsed = start.elapsed();
    let recv_ns = recv_elapsed.as_nanos() as f64 / count as f64;
    println!("recv(): {:.2}ns ({} messages)", recv_ns, count);

    // ============================================================
    // Test 3: Round-trip without black_box
    // ============================================================
    println!("\n--- Test 3: Round-trip without black_box ---");

    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = topic.send(msg);
        let _ = topic.recv();
    }
    let rt_elapsed = start.elapsed();
    let rt_ns = rt_elapsed.as_nanos() as f64 / iterations as f64;
    println!("Round-trip: {:.2}ns ({:.2}ns per op)", rt_ns, rt_ns / 2.0);

    // ============================================================
    // Test 4: Batch of 10 inline (tests loop overhead)
    // ============================================================
    println!("\n--- Test 4: Unrolled batch send (10 at a time) ---");

    let batch_iterations = iterations / 10;
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..batch_iterations {
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
        let _ = topic.send(msg);
    }
    let batch_elapsed = start.elapsed();
    let batch_ns = batch_elapsed.as_nanos() as f64 / iterations as f64;
    println!("Unrolled send(): {:.2}ns", batch_ns);

    // Drain
    while topic.recv().is_some() {}

    // ============================================================
    // Test 5: Compare send() with msg creation inside vs outside loop
    // ============================================================
    println!("\n--- Test 5: send() with msg creation INSIDE loop ---");

    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
    }
    let inside_elapsed = start.elapsed();
    let inside_ns = inside_elapsed.as_nanos() as f64 / iterations as f64;
    println!("send() with inside creation: {:.2}ns", inside_ns);
    println!("Struct creation overhead: {:.2}ns", inside_ns - send_ns);

    // Drain
    while topic.recv().is_some() {}

    // ============================================================
    // Test 6: Raw loop overhead
    // ============================================================
    println!("\n--- Test 6: Empty loop overhead ---");

    let mut dummy = 0u64;
    let start = Instant::now();
    for i in 0..iterations {
        dummy = dummy.wrapping_add(i);
    }
    let loop_elapsed = start.elapsed();
    let loop_ns = loop_elapsed.as_nanos() as f64 / iterations as f64;
    println!("Empty loop: {:.2}ns (dummy={})", loop_ns, dummy % 1000);

    // ============================================================
    // Summary
    // ============================================================
    println!("\n=== Summary ===");
    println!("send() latency:      {:.2}ns", send_ns);
    println!("recv() latency:      {:.2}ns", recv_ns);
    println!("Unrolled send():     {:.2}ns", batch_ns);
    println!("Round-trip:          {:.2}ns ({:.2}ns per op)", rt_ns, rt_ns / 2.0);
    println!("Loop overhead:       {:.2}ns", loop_ns);

    println!("\n=== Target vs Actual ===");
    println!("DirectChannel target: ~3ns");
    println!("Achieved send():      {:.2}ns (gap: {:.2}ns)", send_ns, send_ns - 3.0);
    println!("Achieved recv():      {:.2}ns (gap: {:.2}ns)", recv_ns, recv_ns - 3.0);

    println!("\n=== Done ===");
}
