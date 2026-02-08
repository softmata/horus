//! Zenoh Throughput Benchmarks
//!
//! Measures message throughput for Zenoh transport under sustained load.
//! Tests various message sizes and concurrency levels.
//!
//! Run with: cargo bench --features zenoh -- zenoh_throughput

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use horus::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

/// Small payload for high-frequency messaging
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct ThroughputPayload16 {
    sequence: u64,
    data: u64,
}

unsafe impl bytemuck::Pod for ThroughputPayload16 {}
unsafe impl bytemuck::Zeroable for ThroughputPayload16 {}

/// Medium payload
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct ThroughputPayload256 {
    sequence: u64,
    data: [u64; 31],
}

unsafe impl bytemuck::Pod for ThroughputPayload256 {}
unsafe impl bytemuck::Zeroable for ThroughputPayload256 {}

/// Large payload (1KB)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
struct ThroughputPayload1K {
    sequence: u64,
    #[serde(with = "serde_arrays")]
    data: [u64; 127],
}

impl Default for ThroughputPayload1K {
    fn default() -> Self {
        Self {
            sequence: 0,
            data: [0u64; 127],
        }
    }
}

unsafe impl bytemuck::Pod for ThroughputPayload1K {}
unsafe impl bytemuck::Zeroable for ThroughputPayload1K {}

/// Benchmark sustained throughput - how many messages per second
fn bench_sustained_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("sustained_throughput");
    group.measurement_time(Duration::from_secs(10));
    group.sample_size(100);

    // 16-byte messages - target: >1M msg/sec for native
    let size_16 = std::mem::size_of::<ThroughputPayload16>() as u64;
    group.throughput(Throughput::Elements(1000)); // Measure 1000 messages per iteration
    group.bench_function(BenchmarkId::new("native_16B", "1000_msgs"), |b| {
        let topic = Topic::<ThroughputPayload16>::new("bench.throughput.16").expect("create topic");

        b.iter(|| {
            for i in 0..1000u64 {
                let msg = ThroughputPayload16 {
                    sequence: i,
                    data: i * 2,
                };
                topic.send(black_box(msg));
            }
        });
    });

    // 256-byte messages
    group.throughput(Throughput::Bytes(size_16 * 1000));
    group.bench_function(BenchmarkId::new("native_256B", "1000_msgs"), |b| {
        let topic =
            Topic::<ThroughputPayload256>::new("bench.throughput.256").expect("create topic");

        b.iter(|| {
            for i in 0..1000u64 {
                let msg = ThroughputPayload256 {
                    sequence: i,
                    ..Default::default()
                };
                topic.send(black_box(msg));
            }
        });
    });

    // 1KB messages
    let size_1k = std::mem::size_of::<ThroughputPayload1K>() as u64;
    group.throughput(Throughput::Bytes(size_1k * 100));
    group.bench_function(BenchmarkId::new("native_1KB", "100_msgs"), |b| {
        let topic = Topic::<ThroughputPayload1K>::new("bench.throughput.1k").expect("create topic");

        b.iter(|| {
            for i in 0..100u64 {
                let msg = ThroughputPayload1K {
                    sequence: i,
                    ..Default::default()
                };
                topic.send(black_box(msg));
            }
        });
    });

    group.finish();
}

/// Benchmark burst throughput - short bursts of high traffic
fn bench_burst_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("burst_throughput");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(100);

    // Small burst (100 messages)
    group.throughput(Throughput::Elements(100));
    group.bench_function("native_burst_100", |b| {
        let topic = Topic::<ThroughputPayload16>::new("bench.burst.100").expect("create topic");

        b.iter(|| {
            for i in 0..100u64 {
                let msg = ThroughputPayload16 {
                    sequence: i,
                    data: i,
                };
                topic.send(black_box(msg));
            }
        });
    });

    // Large burst (10000 messages)
    group.throughput(Throughput::Elements(10000));
    group.bench_function("native_burst_10000", |b| {
        let topic = Topic::<ThroughputPayload16>::new("bench.burst.10000").expect("create topic");

        b.iter(|| {
            for i in 0..10000u64 {
                let msg = ThroughputPayload16 {
                    sequence: i,
                    data: i,
                };
                topic.send(black_box(msg));
            }
        });
    });

    group.finish();
}

/// Benchmark multi-topic throughput (simulates real robot scenarios)
fn bench_multi_topic_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("multi_topic_throughput");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(50);

    // Simulate robot with multiple topics (sensors, commands, state)
    group.bench_function("robot_simulation_10_topics", |b| {
        // Create 10 topics
        let topics: Vec<_> = (0..10)
            .map(|i| {
                Topic::<ThroughputPayload16>::new(format!("bench.multi.{}", i))
                    .expect("create topic")
            })
            .collect();

        let counter = AtomicU64::new(0);

        b.iter(|| {
            let seq = counter.fetch_add(1, Ordering::Relaxed);
            // Send to all topics
            for topic in &topics {
                let msg = ThroughputPayload16 {
                    sequence: seq,
                    data: seq * 2,
                };
                topic.send(black_box(msg));
            }
        });
    });

    // High-frequency sensor simulation (100 topics)
    group.bench_function("sensor_array_100_topics", |b| {
        let topics: Vec<_> = (0..100)
            .map(|i| {
                Topic::<ThroughputPayload16>::new(format!("bench.sensors.{}", i))
                    .expect("create topic")
            })
            .collect();

        let counter = AtomicU64::new(0);

        b.iter(|| {
            let seq = counter.fetch_add(1, Ordering::Relaxed);
            // Round-robin through topics
            let topic = &topics[(seq % 100) as usize];
            let msg = ThroughputPayload16 {
                sequence: seq,
                data: seq * 2,
            };
            topic.send(black_box(msg));
        });
    });

    group.finish();
}

/// Benchmark pub-sub round trip (send + receive)
fn bench_pubsub_roundtrip(c: &mut Criterion) {
    let mut group = c.benchmark_group("pubsub_roundtrip");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(500);

    group.bench_function("native_roundtrip_16B", |b| {
        let topic = Topic::<ThroughputPayload16>::new("bench.roundtrip.16").expect("create topic");

        b.iter(|| {
            let msg = ThroughputPayload16 {
                sequence: 1,
                data: 42,
            };
            topic.send(black_box(msg));
            let received = topic.recv();
            black_box(received)
        });
    });

    group.bench_function("native_roundtrip_1KB", |b| {
        let topic = Topic::<ThroughputPayload1K>::new("bench.roundtrip.1k").expect("create topic");

        b.iter(|| {
            let msg = ThroughputPayload1K {
                sequence: 1,
                ..Default::default()
            };
            topic.send(black_box(msg));
            let received = topic.recv();
            black_box(received)
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_sustained_throughput,
    bench_burst_throughput,
    bench_multi_topic_throughput,
    bench_pubsub_roundtrip
);
criterion_main!(benches);
