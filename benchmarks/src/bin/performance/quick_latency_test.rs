//! Quick latency test - measures actual Topic send/recv overhead
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Instant;

fn main() {
    println!("=== Quick Latency Test ===\n");

    let topic: Topic<CmdVel> = Topic::new("quick_test").unwrap();

    // Warmup
    for _ in 0..1000 {
        let msg = CmdVel::new(1.5, 0.8);
        topic.send(msg).unwrap();
        topic.recv();
    }

    // Drain any remaining
    while topic.recv().is_some() {}

    // Measure send only
    let iterations = 100_000u64;
    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);
        let _ = topic.send(msg);
    }
    let send_elapsed = start.elapsed();
    println!("Send only: {:.2}ns per message", send_elapsed.as_nanos() as f64 / iterations as f64);

    // Drain
    while topic.recv().is_some() {}

    // Measure recv only (with messages pre-loaded)
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);
        let _ = topic.send(msg);
    }
    let start = Instant::now();
    let mut count = 0u64;
    while topic.recv().is_some() {
        count += 1;
    }
    let recv_elapsed = start.elapsed();
    println!("Recv only: {:.2}ns per message (received {})",
             recv_elapsed.as_nanos() as f64 / count.max(1) as f64, count);

    // Measure round-trip (send + recv, same-thread)
    let start = Instant::now();
    for _ in 0..iterations {
        let msg = CmdVel::new(1.5, 0.8);
        let _ = topic.send(msg);
        let _ = topic.recv();
    }
    let rt_elapsed = start.elapsed();
    println!("Send+Recv: {:.2}ns per round-trip", rt_elapsed.as_nanos() as f64 / iterations as f64);

    println!("\n=== Done ===");
}
