//! Network Transport Benchmarks
//!
//! Benchmarks for HORUS network transport layer comparing:
//! - Standard UDP loopback
//! - Batch UDP (sendmmsg/recvmmsg on Linux)
//! - Shared memory vs network performance
//!
//! ## Expected Latencies
//!
//! | Transport       | Latency      | Notes                    |
//! |-----------------|--------------|--------------------------|
//! | Shared Memory   | ~150-200ns   | Topic (local IPC)        |
//! | UDP Loopback    | ~5-10µs      | Standard socket syscall  |
//! | Batch UDP       | ~3-5µs       | Linux sendmmsg/recvmmsg  |
//!
//! ## Running Benchmarks
//!
//! ```bash
//! cargo bench --bench network_transport
//! cargo bench --bench network_transport -- "udp_loopback"
//! cargo bench --bench network_transport -- "transport_comparison"
//! ```

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

// =============================================================================
// Section 1: UDP Loopback Latency
// =============================================================================

/// Test payload sizes for network benchmarks
const PAYLOAD_SIZES: &[usize] = &[64, 256, 1024, 4096];

fn create_payload(size: usize) -> Vec<u8> {
    (0..size).map(|i| (i & 0xFF) as u8).collect()
}

/// Benchmark standard UDP loopback latency across payload sizes
fn bench_udp_loopback(c: &mut Criterion) {
    let mut group = c.benchmark_group("udp_loopback");
    group.measurement_time(Duration::from_secs(5));

    for &size in PAYLOAD_SIZES {
        group.bench_with_input(BenchmarkId::new("latency", size), &size, |b, &size| {
            let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
            let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
            let recv_addr = receiver.local_addr().unwrap();

            sender.set_nonblocking(false).unwrap();
            receiver
                .set_read_timeout(Some(Duration::from_millis(100)))
                .unwrap();

            let payload = create_payload(size);
            let mut recv_buf = vec![0u8; size + 64];

            b.iter(|| {
                sender.send_to(black_box(&payload), recv_addr).unwrap();
                black_box(receiver.recv(&mut recv_buf))
            });
        });
    }

    group.finish();
}

// =============================================================================
// Section 2: UDP Round-Trip
// =============================================================================

/// Benchmark UDP round-trip latency (send + echo back)
fn bench_udp_roundtrip(c: &mut Criterion) {
    let mut group = c.benchmark_group("udp_roundtrip");
    group.measurement_time(Duration::from_secs(5));

    let size = 64;

    group.bench_function("64B", |b| {
        let socket_a = UdpSocket::bind("127.0.0.1:0").unwrap();
        let socket_b = UdpSocket::bind("127.0.0.1:0").unwrap();
        let addr_a = socket_a.local_addr().unwrap();
        let addr_b = socket_b.local_addr().unwrap();

        socket_a
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();
        socket_b
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();

        let payload = create_payload(size);
        let mut recv_buf = vec![0u8; size + 64];

        b.iter(|| {
            // A -> B
            socket_a.send_to(black_box(&payload), addr_b).unwrap();
            let _ = socket_b.recv(&mut recv_buf);
            // B -> A (echo)
            socket_b.send_to(&recv_buf[..size], addr_a).unwrap();
            black_box(socket_a.recv(&mut recv_buf))
        });
    });

    group.finish();
}

// =============================================================================
// Section 3: UDP Send Latency (Syscall Overhead)
// =============================================================================

/// Measure raw UDP send latency (isolate syscall overhead)
fn bench_udp_send_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("udp_send_latency");
    group.measurement_time(Duration::from_secs(5));

    let payload = create_payload(64);

    group.bench_function("64B", |b| {
        let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
        let target: SocketAddr = "127.0.0.1:12345".parse().unwrap();
        sender.set_nonblocking(true).unwrap();

        b.iter(|| {
            let _ = sender.send_to(black_box(&payload), target);
        });
    });

    group.finish();
}

// =============================================================================
// Section 4: Batch Send Performance
// =============================================================================

/// Benchmark batch send performance (simulates sendmmsg benefits)
fn bench_batch_send(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_send");
    group.measurement_time(Duration::from_secs(5));

    let batch_sizes: &[usize] = &[1, 8, 16, 32, 64];
    let payload_size = 256;

    for &batch in batch_sizes {
        group.bench_with_input(BenchmarkId::new("batch_size", batch), &batch, |b, &batch| {
            let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
            let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
            let recv_addr = receiver.local_addr().unwrap();

            sender.set_nonblocking(true).unwrap();
            receiver.set_nonblocking(true).unwrap();

            let payloads: Vec<Vec<u8>> =
                (0..batch).map(|_| create_payload(payload_size)).collect();

            b.iter(|| {
                for payload in &payloads {
                    let _ = sender.send_to(black_box(payload), recv_addr);
                }
            });
        });
    }

    group.finish();
}

// =============================================================================
// Section 5: Transport Comparison (SHM vs Network)
// =============================================================================

/// Compare shared memory Topic vs UDP network transport
fn bench_transport_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("transport_comparison");
    group.measurement_time(Duration::from_secs(5));

    // Shared Memory Topic - CmdVel (16 bytes)
    group.bench_function("SharedMemory_Topic_16B", |b| {
        let topic = format!("bench_shm_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // UDP loopback (16 bytes)
    group.bench_function("UDP_Loopback_16B", |b| {
        let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
        let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
        let recv_addr = receiver.local_addr().unwrap();

        receiver
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();

        let payload = create_payload(16);
        let mut recv_buf = [0u8; 64];

        b.iter(|| {
            sender.send_to(black_box(&payload), recv_addr).unwrap();
            black_box(receiver.recv(&mut recv_buf))
        });
    });

    // Shared Memory SpscIntra (fastest same-process)
    group.bench_function("SharedMemory_SpscIntra_16B", |b| {
        let topic = format!("bench_intra_{}", std::process::id());
        let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::spsc_intra(&topic);

        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 6: Network Throughput
// =============================================================================

/// Benchmark UDP throughput (messages per second)
fn bench_udp_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("udp_throughput");
    group.measurement_time(Duration::from_secs(5));

    let message_count = 10000;
    let payload = create_payload(64);

    group.bench_function("10k_messages_64B", |b| {
        let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
        let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
        let recv_addr = receiver.local_addr().unwrap();

        sender.set_nonblocking(true).unwrap();
        receiver.set_nonblocking(true).unwrap();

        let mut recv_buf = vec![0u8; 128];

        b.iter(|| {
            // Send all
            for _ in 0..message_count {
                let _ = sender.send_to(&payload, recv_addr);
            }
            // Drain receiver (best effort)
            for _ in 0..message_count {
                let _ = receiver.recv(&mut recv_buf);
            }
        });
    });

    group.finish();
}

// =============================================================================
// Linux-Specific: Batch UDP with sendmmsg
// =============================================================================

#[cfg(target_os = "linux")]
fn bench_linux_batch_udp(c: &mut Criterion) {
    use horus_core::communication::network::batch_udp::{BatchUdpConfig, BatchUdpSender};

    let mut group = c.benchmark_group("linux_batch_udp");
    group.measurement_time(Duration::from_secs(5));

    group.bench_function("sendmmsg_batch_16", |b| {
        let config = BatchUdpConfig {
            batch_size: 16,
            ..Default::default()
        };

        let bind_addr: SocketAddr = "127.0.0.1:0".parse().unwrap();
        if let Ok(mut sender) = BatchUdpSender::new(bind_addr, config) {
            let target: SocketAddr = "127.0.0.1:12345".parse().unwrap();
            let payloads: Vec<Vec<u8>> = (0..16).map(|_| create_payload(64)).collect();

            b.iter(|| {
                for payload in &payloads {
                    let _ = sender.send(payload, target);
                }
                let _ = sender.flush();
            });
        }
    });

    group.finish();
}

#[cfg(target_os = "linux")]
fn bench_io_uring_check(c: &mut Criterion) {
    use horus_core::communication::network::io_uring::is_real_io_uring_available;

    let mut group = c.benchmark_group("io_uring_status");

    group.bench_function("availability_check", |b| {
        b.iter(|| black_box(is_real_io_uring_available()));
    });

    // Print io_uring status once
    let available = is_real_io_uring_available();
    if available {
        eprintln!("\nio_uring: AVAILABLE (Linux 5.1+)");
    } else {
        eprintln!("\nio_uring: NOT AVAILABLE (requires Linux 5.1+)");
    }

    group.finish();
}

// =============================================================================
// Criterion Configuration
// =============================================================================

criterion_group!(
    name = benches;
    config = Criterion::default()
        .sample_size(100)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(5));
    targets =
        bench_udp_loopback,
        bench_udp_roundtrip,
        bench_udp_send_latency,
        bench_batch_send,
        bench_transport_comparison,
        bench_udp_throughput,
);

#[cfg(target_os = "linux")]
criterion_group!(
    name = linux_benches;
    config = Criterion::default()
        .sample_size(100)
        .measurement_time(Duration::from_secs(5));
    targets = bench_linux_batch_udp, bench_io_uring_check,
);

#[cfg(target_os = "linux")]
criterion_main!(benches, linux_benches);

#[cfg(not(target_os = "linux"))]
criterion_main!(benches);
