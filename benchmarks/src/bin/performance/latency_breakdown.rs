//! Latency breakdown - identify where time is spent in Topic::send/recv
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Instant;

fn main() {
    println!("=== Latency Breakdown ===\n");

    let topic: Topic<CmdVel> = Topic::new("breakdown_test").unwrap();

    // Warmup to trigger backend detection
    for _ in 0..2000 {
        let msg = CmdVel::new(1.5, 0.8);
        topic.send(msg).unwrap();
        topic.recv();
    }
    while topic.recv().is_some() {}

    // Detailed send timing
    let iterations = 100_000u64;

    // Time just constructing messages (baseline)
    let start = Instant::now();
    for i in 0..iterations {
        let _msg = CmdVel::new(1.0 + (i as f32) * 0.001, 0.5);
    }
    let msg_construct = start.elapsed();
    println!("Message construction: {:.2}ns", msg_construct.as_nanos() as f64 / iterations as f64);

    // Time send with pre-constructed messages
    let msgs: Vec<_> = (0..iterations).map(|i| CmdVel::new(1.0 + (i as f32) * 0.001, 0.5)).collect();
    let start = Instant::now();
    for msg in msgs {
        let _ = topic.send(msg);
    }
    let send_time = start.elapsed();
    println!("Send (pre-constructed): {:.2}ns", send_time.as_nanos() as f64 / iterations as f64);

    // Drain
    while topic.recv().is_some() {}

    // Pre-fill and time recv
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);
        let _ = topic.send(msg);
    }
    let start = Instant::now();
    let mut count = 0u64;
    while topic.recv().is_some() {
        count += 1;
    }
    let recv_time = start.elapsed();
    println!("Recv (data available): {:.2}ns", recv_time.as_nanos() as f64 / count as f64);

    // Test with black_box to prevent optimization
    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
    }
    let send_bb = start.elapsed();
    println!("\nSend (with black_box): {:.2}ns", send_bb.as_nanos() as f64 / iterations as f64);

    // Drain and test round-trip
    while topic.recv().is_some() {}

    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
        let _ = std::hint::black_box(topic.recv());
    }
    let rt_time = start.elapsed();
    println!("Round-trip (with black_box): {:.2}ns", rt_time.as_nanos() as f64 / iterations as f64);

    // Check metrics via public API
    let metrics = topic.get_metrics();
    println!("\n=== Final Metrics ===");
    println!("Messages sent: {}", metrics.messages_sent);
    println!("Messages received: {}", metrics.messages_received);

    println!("\n=== Done ===");
}
