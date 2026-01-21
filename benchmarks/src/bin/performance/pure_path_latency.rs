//! Pure path latency test - isolates the ultra-fast path from message creation overhead
//!
//! This benchmark properly measures send/recv WITHOUT including SystemTime::now() overhead
//! from CmdVel::new().
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Instant;

fn main() {
    println!("=== Pure Ultra-Fast Path Latency Test ===\n");

    let topic: Topic<CmdVel> = Topic::new("pure_latency").unwrap();

    // Warmup - forces mode detection and migration to DirectChannel
    for _ in 0..10_000 {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
        let _ = topic.recv();
    }

    println!("Backend type after warmup: {}", topic.backend_type());

    // Drain any messages
    while topic.recv().is_some() {}

    // ============================================================
    // Test 1: Pure send() with pre-created message (no timestamp)
    // ============================================================
    println!("\n--- Test 1: Pure send() with with_timestamp() ---");

    let iterations = 1_000_000u64;
    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
    }
    let send_elapsed = start.elapsed();
    println!("Pure send (with_timestamp): {:.2}ns", send_elapsed.as_nanos() as f64 / iterations as f64);

    // Drain
    while topic.recv().is_some() {}

    // ============================================================
    // Test 2: Pure send() with message created outside loop
    // ============================================================
    println!("\n--- Test 2: Pre-created message reuse ---");

    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
    }
    let send_elapsed = start.elapsed();
    println!("Pre-created send: {:.2}ns", send_elapsed.as_nanos() as f64 / iterations as f64);

    // Drain
    while topic.recv().is_some() {}

    // ============================================================
    // Test 3: recv() with pre-sent messages
    // ============================================================
    println!("\n--- Test 3: Pure recv() ---");

    // Send all messages first
    for _ in 0..iterations {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
    }

    let start = Instant::now();
    let mut count = 0u64;
    while topic.recv().is_some() {
        count += 1;
    }
    let recv_elapsed = start.elapsed();
    println!("Pure recv: {:.2}ns ({} messages)", recv_elapsed.as_nanos() as f64 / count as f64, count);

    // ============================================================
    // Test 4: Round-trip (send+recv) with pre-created message
    // ============================================================
    println!("\n--- Test 4: Round-trip with pre-created message ---");

    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
        let _ = std::hint::black_box(topic.recv());
    }
    let rt_elapsed = start.elapsed();
    println!("Round-trip: {:.2}ns ({:.2}ns per op)",
             rt_elapsed.as_nanos() as f64 / iterations as f64,
             rt_elapsed.as_nanos() as f64 / iterations as f64 / 2.0);

    // ============================================================
    // Test 5: Compare with CmdVel::new() to show overhead
    // ============================================================
    println!("\n--- Test 5: Send with CmdVel::new() (includes SystemTime::now()) ---");

    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);  // Includes SystemTime::now()
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
    }
    let send_new_elapsed = start.elapsed();
    println!("Send with ::new(): {:.2}ns", send_new_elapsed.as_nanos() as f64 / iterations as f64);

    // Drain
    while topic.recv().is_some() {}

    // ============================================================
    // Test 6: Measure just SystemTime::now() overhead
    // ============================================================
    println!("\n--- Test 6: SystemTime::now() overhead ---");

    let start = Instant::now();
    for _ in 0..iterations {
        let _ = std::hint::black_box(std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64);
    }
    let syscall_elapsed = start.elapsed();
    println!("SystemTime::now(): {:.2}ns", syscall_elapsed.as_nanos() as f64 / iterations as f64);

    // ============================================================
    // Summary
    // ============================================================
    let pure_send_ns = send_elapsed.as_nanos() as f64 / iterations as f64;
    let pure_recv_ns = recv_elapsed.as_nanos() as f64 / count as f64;
    let syscall_ns = syscall_elapsed.as_nanos() as f64 / iterations as f64;
    let send_with_new_ns = send_new_elapsed.as_nanos() as f64 / iterations as f64;

    println!("\n=== Summary ===");
    println!("Pure send() latency:       {:.2}ns", pure_send_ns);
    println!("Pure recv() latency:       {:.2}ns", pure_recv_ns);
    println!("SystemTime::now() cost:    {:.2}ns", syscall_ns);
    println!("Calculated send+syscall:   {:.2}ns (expected ~{:.2}ns)", pure_send_ns + syscall_ns, send_with_new_ns);
    println!("Actual send with ::new():  {:.2}ns", send_with_new_ns);

    println!("\n=== Target vs Actual ===");
    println!("DirectChannel target: ~3ns");
    println!("Achieved send():      {:.2}ns", pure_send_ns);
    println!("Achieved recv():      {:.2}ns", pure_recv_ns);

    println!("\n=== Done ===");
}
