/// Performance benchmarks for HORUS network layer
///
/// Targets:
/// - Local shared memory: <200ns
/// - Unix socket (localhost): <5μs
/// - UDP direct (localhost): <50μs
///
/// Run with: cargo bench --bench network_bench
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct TestMessage {
    id: u64,
    data: Vec<u8>, // 1KB payload
}

impl horus_core::core::LogSummary for TestMessage {
    fn log_summary(&self) -> String {
        format!("TestMessage(id={})", self.id)
    }
}

impl TestMessage {
    fn new(id: u64) -> Self {
        Self {
            id,
            data: vec![0u8; 1024],
        }
    }
}

/// Benchmark local shared memory (baseline)
fn bench_local_shm(c: &mut Criterion) {
    let hub: Topic<TestMessage> = Topic::new("bench_local").unwrap();

    c.bench_function("local_shm_send_recv", |b| {
        let mut counter = 0u64;
        b.iter(|| {
            let msg = TestMessage::new(counter);
            hub.send(black_box(msg), &mut None).unwrap();
            counter += 1;

            let received = hub.recv(&mut None).unwrap();
            black_box(received);
        });
    });

    println!("Local shared memory benchmark completed");
}

/// Benchmark Unix socket (localhost)
fn bench_unix_socket(c: &mut Criterion) {
    // Create publisher in background thread
    let pub_handle = std::thread::spawn(|| {
        use horus_core::communication::network::UnixSocketBackend;

        let backend = UnixSocketBackend::<TestMessage>::new_publisher("bench_unix").unwrap();

        // Wait for subscriber
        std::thread::sleep(Duration::from_millis(100));

        // Send messages
        for i in 0..1000 {
            let msg = TestMessage::new(i);
            backend.send(&msg).unwrap();
        }

        backend
    });

    // Give publisher time to bind
    std::thread::sleep(Duration::from_millis(50));

    // Create subscriber
    use horus_core::communication::network::UnixSocketBackend;
    let mut sub_backend = UnixSocketBackend::<TestMessage>::new_subscriber("bench_unix").unwrap();

    c.bench_function("unix_socket_recv", |b| {
        b.iter(|| {
            let received = sub_backend.recv().unwrap();
            black_box(received);
        });
    });

    pub_handle.join().unwrap();
    println!("Unix socket benchmark completed");
}

/// Benchmark UDP direct (localhost loopback)
fn bench_udp_loopback(c: &mut Criterion) {
    use horus_core::communication::network::UdpDirectBackend;
    use std::net::UdpSocket;

    // Create listener on fixed port
    let _listener = UdpSocket::bind("127.0.0.1:29870").unwrap();

    // Create backend that sends to listener
    let sender =
        UdpDirectBackend::<TestMessage>::new("bench_udp", "127.0.0.1".parse().unwrap(), 29870)
            .unwrap();

    // Send some messages to warm up
    for i in 0..10 {
        sender.send(&TestMessage::new(i)).unwrap();
        std::thread::sleep(Duration::from_micros(100));
    }

    c.bench_function("udp_direct_send", |b| {
        let mut counter = 0u64;
        b.iter(|| {
            let msg = TestMessage::new(counter);
            sender.send(black_box(&msg)).unwrap();
            counter += 1;
        });
    });

    println!("UDP direct benchmark completed");
}

/// Benchmark protocol encode/decode
fn bench_protocol(c: &mut Criterion) {
    use horus_core::communication::network::HorusPacket;

    let payload = vec![0u8; 1024]; // 1KB payload
    let packet = HorusPacket::new_data("test_topic".to_string(), payload, 42);

    c.bench_function("protocol_encode", |b| {
        let mut buf = Vec::with_capacity(2048);
        b.iter(|| {
            packet.encode(black_box(&mut buf));
        });
    });

    let mut encode_buf = Vec::new();
    packet.encode(&mut encode_buf);

    c.bench_function("protocol_decode", |b| {
        b.iter(|| {
            let decoded = HorusPacket::decode(black_box(&encode_buf)).unwrap();
            black_box(decoded);
        });
    });

    println!("Protocol encode/decode benchmark completed");
}

/// Comparison benchmark across different message sizes
fn bench_message_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_sizes");

    for size in [64, 256, 1024, 4096].iter() {
        #[derive(Debug, Clone, Serialize, Deserialize)]
        struct VarMessage {
            data: Vec<u8>,
        }

        impl horus_core::core::LogSummary for VarMessage {
            fn log_summary(&self) -> String {
                format!("VarMessage({}B)", self.data.len())
            }
        }

        group.bench_with_input(BenchmarkId::new("local_shm", size), size, |b, &size| {
            let hub: Topic<VarMessage> = Topic::new(&format!("bench_size_{}", size)).unwrap();
            b.iter(|| {
                let msg = VarMessage {
                    data: vec![0u8; size],
                };
                hub.send(msg, &mut None).unwrap();
                let received = hub.recv(&mut None).unwrap();
                black_box(received);
            });
        });
    }

    group.finish();
    println!("Message size comparison completed");
}

/// Benchmark DirectChannel backend (Tier 0: ~3-5ns target)
///
/// This is the fastest possible IPC path for same-thread communication.
/// Uses no atomics, no barriers - just direct slot write/read.
fn bench_direct_channel(c: &mut Criterion) {
    let topic: Topic<TestMessage> = Topic::direct("bench_direct");

    c.bench_function("direct_channel_send_recv", |b| {
        let mut counter = 0u64;
        b.iter(|| {
            let msg = TestMessage::new(counter);
            topic.send(black_box(msg), &mut None).unwrap();
            counter += 1;

            let received = topic.recv(&mut None).unwrap();
            black_box(received);
        });
    });

    // Benchmark send_sync callback (zero-copy path)
    c.bench_function("direct_channel_send_sync", |b| {
        let mut counter = 0u64;
        b.iter(|| {
            let msg = TestMessage::new(counter);
            let result = topic.send_sync(black_box(msg), |m| {
                black_box(m.id)
            });
            counter += 1;
            black_box(result);
        });
    });

    println!("DirectChannel benchmark completed (target: ~3-5ns)");
}

/// Benchmark DirectChannel with tiny message (just u64)
fn bench_direct_channel_tiny(c: &mut Criterion) {
    #[derive(Debug, Clone)]
    struct TinyMessage(u64);

    let topic: Topic<TinyMessage> = Topic::direct("bench_direct_tiny");

    c.bench_function("direct_channel_tiny", |b| {
        let mut counter = 0u64;
        b.iter(|| {
            topic.send(black_box(TinyMessage(counter)), &mut None).unwrap();
            counter = counter.wrapping_add(1);
            let received = topic.recv(&mut None).unwrap();
            black_box(received);
        });
    });

    println!("DirectChannel tiny message benchmark completed");
}

criterion_group!(
    benches,
    bench_local_shm,
    bench_direct_channel,
    bench_direct_channel_tiny,
    bench_unix_socket,
    bench_udp_loopback,
    bench_protocol,
    bench_message_sizes
);
criterion_main!(benches);
