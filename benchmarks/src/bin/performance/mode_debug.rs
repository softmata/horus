//! Mode detection debug - verifies DirectChannel is being used for same-thread
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Instant;

fn main() {
    println!("=== Mode Detection Debug ===\n");

    // Create topic
    let topic: Topic<CmdVel> = Topic::new("mode_debug").unwrap();

    println!("Backend type: {}", topic.backend_type());

    // First send - triggers producer registration
    println!("\n--- First send (registration) ---");
    let start = Instant::now();
    topic.send(CmdVel::new(1.0, 0.0)).unwrap();
    let first_send = start.elapsed();
    println!("First send took: {:?}", first_send);
    println!("Backend type: {}", topic.backend_type());

    // First recv - triggers consumer registration and should migrate to DirectChannel
    println!("\n--- First recv (registration + migration) ---");
    let start = Instant::now();
    let _ = topic.recv();
    let first_recv = start.elapsed();
    println!("First recv took: {:?}", first_recv);
    println!("Backend type: {}", topic.backend_type());

    // Second send - should be on ultra-fast path
    println!("\n--- Second send (should be fast) ---");
    let start = Instant::now();
    topic.send(CmdVel::new(1.0, 0.0)).unwrap();
    let second_send = start.elapsed();
    println!("Second send took: {:?}", second_send);

    // Second recv
    let start = Instant::now();
    let _ = topic.recv();
    let second_recv = start.elapsed();
    println!("Second recv took: {:?}", second_recv);

    // Do warmup
    println!("\n--- Warmup (1000 iterations) ---");
    for _ in 0..1000 {
        topic.send(CmdVel::new(1.0, 0.0)).unwrap();
        let _ = topic.recv();
    }
    println!("Backend type after warmup: {}", topic.backend_type());

    // Measure individual send times
    println!("\n--- Individual send timing (10 samples) ---");
    for i in 0..10 {
        let start = Instant::now();
        topic.send(CmdVel::new(1.0, 0.0)).unwrap();
        let elapsed = start.elapsed();
        let _ = topic.recv();
        println!("Send {}: {:?} ({:.1}ns)", i, elapsed, elapsed.as_nanos() as f64);
    }

    // Batch measurement
    println!("\n--- Batch send timing (100,000 iterations) ---");
    let iterations = 100_000u64;

    // Drain first
    while topic.recv().is_some() {}

    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);
        let _ = topic.send(msg);
    }
    let send_elapsed = start.elapsed();

    println!("Total time: {:?}", send_elapsed);
    println!("Per send: {:.2}ns", send_elapsed.as_nanos() as f64 / iterations as f64);
    println!("Final backend type: {}", topic.backend_type());

    // Also test recv batch
    println!("\n--- Batch recv timing ---");
    let start = Instant::now();
    let mut count = 0u64;
    while topic.recv().is_some() {
        count += 1;
    }
    let recv_elapsed = start.elapsed();
    println!("Received {} messages in {:?}", count, recv_elapsed);
    println!("Per recv: {:.2}ns", recv_elapsed.as_nanos() as f64 / count.max(1) as f64);

    println!("\n=== Done ===");
}
