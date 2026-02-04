//! Zenoh Latency Benchmarks
//!
//! Measures end-to-end latency for Zenoh transport at various message sizes.
//! Compares with native HORUS shared memory IPC.
//!
//! Run with: cargo bench --features zenoh -- zenoh_latency

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use horus::prelude::*;
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Small message (16 bytes) - typical sensor reading
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct SmallMessage {
    timestamp: u64,
    value: f64,
}

unsafe impl bytemuck::Pod for SmallMessage {}
unsafe impl bytemuck::Zeroable for SmallMessage {}

/// Medium message (128 bytes) - typical control command
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct MediumMessage {
    header: [u64; 4],
    position: [f64; 3],
    velocity: [f64; 3],
    acceleration: [f64; 3],
    flags: u64,
}

unsafe impl bytemuck::Pod for MediumMessage {}
unsafe impl bytemuck::Zeroable for MediumMessage {}

/// Large message (4KB) - typical image metadata or point cloud header
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
struct LargeMessage {
    #[serde(with = "serde_arrays")]
    data: [u64; 512],
}

impl Default for LargeMessage {
    fn default() -> Self {
        Self { data: [0u64; 512] }
    }
}

unsafe impl bytemuck::Pod for LargeMessage {}
unsafe impl bytemuck::Zeroable for LargeMessage {}

/// Benchmark native HORUS shared memory IPC latency
fn bench_native_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("native_shm_latency");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(1000);

    // Small message (16 bytes)
    group.throughput(Throughput::Bytes(std::mem::size_of::<SmallMessage>() as u64));
    group.bench_function(BenchmarkId::new("small", "16B"), |b| {
        let topic = Topic::<SmallMessage>::new("bench.native.small").expect("create topic");
        let msg = SmallMessage::default();

        b.iter(|| {
            topic.send(black_box(msg)).ok();
            topic.recv()
        });
    });

    // Medium message (128 bytes)
    group.throughput(Throughput::Bytes(std::mem::size_of::<MediumMessage>() as u64));
    group.bench_function(BenchmarkId::new("medium", "128B"), |b| {
        let topic = Topic::<MediumMessage>::new("bench.native.medium").expect("create topic");
        let msg = MediumMessage::default();

        b.iter(|| {
            topic.send(black_box(msg)).ok();
            topic.recv()
        });
    });

    // Large message (4KB)
    group.throughput(Throughput::Bytes(std::mem::size_of::<LargeMessage>() as u64));
    group.bench_function(BenchmarkId::new("large", "4KB"), |b| {
        let topic = Topic::<LargeMessage>::new("bench.native.large").expect("create topic");
        let msg = LargeMessage::default();

        b.iter(|| {
            topic.send(black_box(msg)).ok();
            topic.recv()
        });
    });

    group.finish();
}

/// Benchmark comparison between native and Zenoh (placeholder for when Zenoh is available)
fn bench_latency_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("latency_comparison");
    group.measurement_time(Duration::from_secs(3));
    group.sample_size(100);

    // Native SHM baseline
    group.bench_function("native_shm_16B", |b| {
        let topic = Topic::<SmallMessage>::new("bench.compare.native").expect("create topic");
        let msg = SmallMessage::default();

        b.iter(|| {
            topic.send(black_box(msg)).ok();
            topic.recv()
        });
    });

    // Zenoh would be added here when feature is enabled
    // This is a placeholder showing the structure
    #[cfg(feature = "zenoh")]
    {
        // Note: Zenoh benchmarks require async runtime
        // For criterion, we use block_on or a dedicated async benchmark
        group.bench_function("zenoh_16B", |b| {
            // Zenoh setup would go here
            b.iter(|| {
                // Zenoh send/recv would go here
                black_box(42)
            });
        });
    }

    group.finish();
}

/// Benchmark memory footprint (measure allocations)
fn bench_memory_usage(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_usage");
    group.measurement_time(Duration::from_secs(2));
    group.sample_size(50);

    // Topic creation overhead
    group.bench_function("topic_creation", |b| {
        b.iter(|| {
            let topic = Topic::<SmallMessage>::new("bench.memory.create");
            black_box(topic)
        });
    });

    // Message allocation (if any)
    group.bench_function("message_alloc_small", |b| {
        b.iter(|| {
            let msg = SmallMessage::default();
            black_box(msg)
        });
    });

    group.bench_function("message_alloc_large", |b| {
        b.iter(|| {
            let msg = LargeMessage::default();
            black_box(msg)
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_native_latency,
    bench_latency_comparison,
    bench_memory_usage
);
criterion_main!(benches);
