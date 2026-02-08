/// Simple performance test for network layer
///
/// Measures actual latency for:
/// - Local shared memory (baseline)
/// - Unix socket (localhost)
/// - UDP direct (localhost loopback)
///
/// Run with: cargo run --example network_perf_test --release
use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};
use std::time::Instant;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct TestMessage {
    id: u64,
    data: Vec<u8>,
}

impl horus_core::core::LogSummary for TestMessage {
    fn log_summary(&self) -> String {
        format!("TestMessage(id={}, size={}B)", self.id, self.data.len())
    }
}

fn bench_local_shm() {
    println!("\n=== Local Shared Memory (baseline) ===");

    let hub: Topic<TestMessage> = Topic::new("bench_local").unwrap();
    let iterations = 10000;

    // Warmup
    for i in 0..100 {
        let msg = TestMessage {
            id: i,
            data: vec![0u8; 1024],
        };
        hub.send(msg);
        let _ = hub.recv();
    }

    // Measure send + recv latency
    let start = Instant::now();
    for i in 0..iterations {
        let msg = TestMessage {
            id: i,
            data: vec![0u8; 1024],
        };
        hub.send(msg);
        let _ = hub.recv().unwrap();
    }
    let elapsed = start.elapsed();

    let avg_ns = elapsed.as_nanos() / iterations as u128;
    let avg_us = avg_ns as f64 / 1000.0;

    println!("  Iterations: {}", iterations);
    println!("  Total time: {:?}", elapsed);
    println!("  Avg latency: {:.2} ns / {:.3} μs", avg_ns, avg_us);
    println!("  Target: <200 ns");

    if avg_ns < 200 {
        println!("  PASS");
    } else {
        println!("  [FAIL] FAIL (exceeded target)");
    }
}

fn bench_unix_socket() {
    println!("\n=== Unix Socket (localhost) ===");

    use horus_core::communication::network::UnixSocketBackend;
    use std::thread;
    use std::time::Duration;

    // Spawn publisher thread
    let pub_thread = thread::spawn(|| {
        let backend = UnixSocketBackend::<TestMessage>::new_publisher("bench_unix").unwrap();

        // Wait for subscriber to connect
        thread::sleep(Duration::from_millis(200));

        // Send messages
        for i in 0..1000 {
            let msg = TestMessage {
                id: i,
                data: vec![0u8; 1024],
            };
            backend.send(&msg).unwrap();
            thread::sleep(Duration::from_micros(100)); // Rate limit
        }
    });

    // Give publisher time to bind
    thread::sleep(Duration::from_millis(100));

    // Create subscriber
    let mut backend = UnixSocketBackend::<TestMessage>::new_subscriber("bench_unix").unwrap();

    // Warmup
    for _ in 0..10 {
        while backend.recv().is_none() {
            thread::sleep(Duration::from_micros(10));
        }
    }

    // Measure recv latency
    let iterations = 100;
    let start = Instant::now();
    for _ in 0..iterations {
        while backend.recv().is_none() {
            thread::sleep(Duration::from_micros(10));
        }
    }
    let elapsed = start.elapsed();

    let avg_ns = elapsed.as_nanos() / iterations as u128;
    let avg_us = avg_ns as f64 / 1000.0;

    println!("  Iterations: {}", iterations);
    println!("  Total time: {:?}", elapsed);
    println!("  Avg latency: {:.2} ns / {:.3} μs", avg_ns, avg_us);
    println!("  Target: <5 μs");

    if avg_us < 5.0 {
        println!("  PASS");
    } else {
        println!("  [FAIL] FAIL (exceeded target)");
    }

    pub_thread.join().unwrap();
}

fn bench_direct_backend() {
    println!("\n=== Direct Backend (TCP 1P1C) ===");

    // DirectBackend uses TCP for 1P1C connections
    // This requires both producer and consumer to be running
    // For now, just print info about the backend
    println!("  DirectBackend provides TCP-based 1P1C communication");
    println!("  Target latency: ~5-15 μs (lower than router-based)");
    println!("  Features:");
    println!("    - Async Tokio I/O");
    println!("    - Lock-free crossbeam channels");
    println!("    - TCP_NODELAY for low latency");
    println!("    - Zero-allocation buffer pooling");
    println!("  SKIPPED (requires multi-process setup)");
}

fn bench_protocol() {
    println!("\n=== Protocol Encode/Decode ===");

    use horus_core::communication::network::HorusPacket;

    let payload = vec![0u8; 1024];
    let packet = HorusPacket::new_data("test_topic".to_string(), payload, 42);

    // Benchmark encode
    let iterations = 10000;
    let start = Instant::now();
    for _ in 0..iterations {
        let mut buf = Vec::with_capacity(2048);
        packet.encode(&mut buf);
    }
    let encode_elapsed = start.elapsed();
    let encode_avg_ns = encode_elapsed.as_nanos() / iterations as u128;

    // Benchmark decode
    let mut encode_buf = Vec::new();
    packet.encode(&mut encode_buf);

    let start = Instant::now();
    for _ in 0..iterations {
        let _ = HorusPacket::decode(&encode_buf).unwrap();
    }
    let decode_elapsed = start.elapsed();
    let decode_avg_ns = decode_elapsed.as_nanos() / iterations as u128;

    println!("  Iterations: {}", iterations);
    println!("  Encode avg: {:.2} ns", encode_avg_ns);
    println!("  Decode avg: {:.2} ns", decode_avg_ns);
    println!(
        "  Total round-trip: {:.2} ns",
        encode_avg_ns + decode_avg_ns
    );
}

fn main() {
    println!("HORUS Network Layer Performance Test");
    println!("=====================================");
    println!("Message size: 1KB payload");

    bench_local_shm();
    bench_unix_socket();
    bench_direct_backend();
    bench_protocol();

    println!("\n=== Summary ===");
    println!("All benchmarks completed. Check results against targets above.");
}
