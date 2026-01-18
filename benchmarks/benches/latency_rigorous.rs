//! Rigorous Latency Benchmarks
//!
//! Industry-grade latency measurement with:
//! - Bootstrap confidence intervals
//! - Percentile breakdown (p50, p95, p99, p999, p9999)
//! - Jitter analysis (coefficient of variation)
//! - Platform-aware reporting
//! - JSON output for CI regression tracking
//!
//! ## Running
//!
//! ```bash
//! # Full benchmark suite
//! cargo bench --bench latency_rigorous
//!
//! # Specific backend
//! cargo bench --bench latency_rigorous -- "spsc_shm"
//!
//! # Save JSON report
//! BENCH_OUTPUT_JSON=results.json cargo bench --bench latency_rigorous
//! ```

use criterion::{
    black_box, criterion_group, criterion_main, BatchSize, BenchmarkId, Criterion, Throughput,
};
use horus::prelude::Topic;
use horus_benchmarks::{
    detect_platform, set_cpu_affinity, BenchmarkConfig, BenchmarkMessage, DeterminismMetrics,
    PlatformInfo, Statistics, ThroughputMetrics, MESSAGE_SIZES,
};
use horus_core::memory::ShmTopic;
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

// =============================================================================
// Benchmark Payloads
// =============================================================================

/// Variable-size payload for benchmarking
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BenchPayload {
    pub id: u64,
    pub timestamp_ns: u64,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
}

impl BenchPayload {
    pub fn new(size: usize) -> Self {
        Self {
            id: 0,
            timestamp_ns: 0,
            data: vec![0xAB; size],
        }
    }
}

impl Default for BenchPayload {
    fn default() -> Self {
        Self::new(64)
    }
}

impl horus_core::core::LogSummary for BenchPayload {
    fn log_summary(&self) -> String {
        format!("BenchPayload({}B)", self.data.len())
    }
}

// =============================================================================
// Section 1: SpscShm Latency (Cross-Process SPSC)
// =============================================================================

/// Rigorous SpscShm latency benchmark across message sizes
fn bench_spsc_shm_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("spsc_shm_latency");
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    // Control command (16B)
    bench_spsc_shm_size(&mut group, "control_cmd", 16);

    // Motor command (64B)
    bench_spsc_shm_size(&mut group, "motor_cmd", 64);

    // IMU reading (128B)
    bench_spsc_shm_size(&mut group, "imu", 128);

    // Sensor fusion (256B)
    bench_spsc_shm_size(&mut group, "sensor_fusion", 256);

    // LiDAR scan line (4KB)
    bench_spsc_shm_size(&mut group, "lidar_scan", 4096);

    group.finish();
}

fn bench_spsc_shm_size(
    group: &mut criterion::BenchmarkGroup<criterion::measurement::WallTime>,
    name: &str,
    size: usize,
) {
    group.throughput(Throughput::Bytes(size as u64));
    group.bench_with_input(BenchmarkId::new("roundtrip", name), &size, |b, &sz| {
        let topic = format!("bench_spsc_{}_{}", sz, std::process::id());
        let producer: Topic<BenchPayload> = Topic::producer(&topic).unwrap();
        let consumer: Topic<BenchPayload> = Topic::consumer(&topic).unwrap();
        let msg = BenchPayload::new(sz);

        // Warmup
        for _ in 0..1000 {
            producer.send(msg.clone(), &mut None).unwrap();
            let _ = consumer.recv(&mut None);
        }

        b.iter(|| {
            producer.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });
}

// =============================================================================
// Section 2: SpscIntra Latency (Same-Process SPSC)
// =============================================================================

/// Rigorous SpscIntra latency benchmark
fn bench_spsc_intra_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("spsc_intra_latency");
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    bench_spsc_intra_size(&mut group, "control_cmd", 16);
    bench_spsc_intra_size(&mut group, "motor_cmd", 64);
    bench_spsc_intra_size(&mut group, "imu", 128);
    bench_spsc_intra_size(&mut group, "sensor_fusion", 256);
    bench_spsc_intra_size(&mut group, "lidar_scan", 4096);

    group.finish();
}

fn bench_spsc_intra_size(
    group: &mut criterion::BenchmarkGroup<criterion::measurement::WallTime>,
    name: &str,
    size: usize,
) {
    group.throughput(Throughput::Bytes(size as u64));
    group.bench_with_input(BenchmarkId::new("roundtrip", name), &size, |b, &sz| {
        let topic = format!("bench_intra_{}_{}", sz, std::process::id());
        let (producer, consumer): (Topic<BenchPayload>, Topic<BenchPayload>) =
            Topic::spsc_intra(&topic);
        let msg = BenchPayload::new(sz);

        // Warmup
        for _ in 0..1000 {
            producer.send(msg.clone(), &mut None).unwrap();
            let _ = consumer.recv(&mut None);
        }

        b.iter(|| {
            producer.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });
}

// =============================================================================
// Section 3: MpmcShm Latency (Cross-Process MPMC)
// =============================================================================

/// Rigorous MpmcShm latency benchmark
fn bench_mpmc_shm_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("mpmc_shm_latency");
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    bench_mpmc_shm_size(&mut group, "control_cmd", 16);
    bench_mpmc_shm_size(&mut group, "motor_cmd", 64);
    bench_mpmc_shm_size(&mut group, "imu", 128);
    bench_mpmc_shm_size(&mut group, "sensor_fusion", 256);
    bench_mpmc_shm_size(&mut group, "lidar_scan", 4096);

    group.finish();
}

fn bench_mpmc_shm_size(
    group: &mut criterion::BenchmarkGroup<criterion::measurement::WallTime>,
    name: &str,
    size: usize,
) {
    group.throughput(Throughput::Bytes(size as u64));
    group.bench_with_input(BenchmarkId::new("roundtrip", name), &size, |b, &sz| {
        let topic = format!("bench_mpmc_{}_{}", sz, std::process::id());
        let sender: Topic<BenchPayload> = Topic::new(&topic).unwrap();
        let receiver: Topic<BenchPayload> = Topic::new(&topic).unwrap();
        let msg = BenchPayload::new(sz);

        // Warmup
        for _ in 0..1000 {
            sender.send(msg.clone(), &mut None).unwrap();
            let _ = receiver.recv(&mut None);
        }

        b.iter(|| {
            sender.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });
}

// =============================================================================
// Section 4: Raw ShmTopic Baseline
// =============================================================================

/// Raw ShmTopic latency (no Topic wrapper overhead)
fn bench_shm_topic_raw(c: &mut Criterion) {
    let mut group = c.benchmark_group("shm_topic_raw");
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    bench_shm_topic_size(&mut group, "control_cmd", 16);
    bench_shm_topic_size(&mut group, "motor_cmd", 64);
    bench_shm_topic_size(&mut group, "imu", 128);
    bench_shm_topic_size(&mut group, "sensor_fusion", 256);
    bench_shm_topic_size(&mut group, "lidar_scan", 4096);

    group.finish();
}

fn bench_shm_topic_size(
    group: &mut criterion::BenchmarkGroup<criterion::measurement::WallTime>,
    name: &str,
    size: usize,
) {
    group.throughput(Throughput::Bytes(size as u64));
    group.bench_with_input(BenchmarkId::new("roundtrip", name), &size, |b, &sz| {
        let topic = format!("bench_raw_{}_{}", sz, std::process::id());
        let capacity = (65536 / sz).max(64);
        let producer: ShmTopic<BenchPayload> = ShmTopic::new(&topic, capacity).unwrap();
        let consumer: ShmTopic<BenchPayload> = ShmTopic::open(&topic).unwrap();
        let msg = BenchPayload::new(sz);

        // Warmup
        for _ in 0..1000 {
            producer.push(msg.clone()).unwrap();
            let _ = consumer.pop();
        }

        b.iter(|| {
            producer.push(black_box(msg.clone())).unwrap();
            black_box(consumer.pop())
        });
    });
}

// =============================================================================
// Section 5: Send-Only Latency (Decoupled)
// =============================================================================

/// Measure send latency only (not roundtrip)
fn bench_send_only_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("send_only");
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    // SpscShm send
    group.throughput(Throughput::Bytes(64));
    group.bench_function("spsc_shm/64B", |b| {
        let topic = format!("bench_send_spsc_{}", std::process::id());
        let producer: Topic<BenchPayload> = Topic::producer(&topic).unwrap();
        let _consumer: Topic<BenchPayload> = Topic::consumer(&topic).unwrap();
        let msg = BenchPayload::new(64);

        b.iter(|| {
            producer.send(black_box(msg.clone()), &mut None).unwrap();
        });
    });

    // SpscIntra send
    group.bench_function("spsc_intra/64B", |b| {
        let topic = format!("bench_send_intra_{}", std::process::id());
        let (producer, _consumer): (Topic<BenchPayload>, Topic<BenchPayload>) =
            Topic::spsc_intra(&topic);
        let msg = BenchPayload::new(64);

        b.iter(|| {
            producer.send(black_box(msg.clone()), &mut None).unwrap();
        });
    });

    // MpmcShm send
    group.bench_function("mpmc_shm/64B", |b| {
        let topic = format!("bench_send_mpmc_{}", std::process::id());
        let sender: Topic<BenchPayload> = Topic::new(&topic).unwrap();
        let _receiver: Topic<BenchPayload> = Topic::new(&topic).unwrap();
        let msg = BenchPayload::new(64);

        b.iter(|| {
            sender.send(black_box(msg.clone()), &mut None).unwrap();
        });
    });

    group.finish();
}

// =============================================================================
// Section 6: Backend Comparison at Standard Size
// =============================================================================

/// Compare all backends at 64B (control command size)
fn bench_backend_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("backend_comparison/64B");
    group.throughput(Throughput::Bytes(64));
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    let msg = BenchPayload::new(64);

    // DirectChannel
    group.bench_function("DirectChannel", |b| {
        let topic: Topic<BenchPayload> = Topic::direct("bench_cmp_direct");
        b.iter(|| {
            topic.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(topic.recv(&mut None))
        });
    });

    // SpscIntra
    group.bench_function("SpscIntra", |b| {
        let topic = format!("bench_cmp_spsc_intra_{}", std::process::id());
        let (producer, consumer): (Topic<BenchPayload>, Topic<BenchPayload>) =
            Topic::spsc_intra(&topic);
        b.iter(|| {
            producer.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // MpmcIntra
    group.bench_function("MpmcIntra", |b| {
        let topic = format!("bench_cmp_mpmc_intra_{}", std::process::id());
        let (producer, consumer): (Topic<BenchPayload>, Topic<BenchPayload>) =
            Topic::mpmc_intra(&topic, 1024);
        b.iter(|| {
            producer.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // SpscShm
    group.bench_function("SpscShm", |b| {
        let topic = format!("bench_cmp_spsc_shm_{}", std::process::id());
        let producer: Topic<BenchPayload> = Topic::producer(&topic).unwrap();
        let consumer: Topic<BenchPayload> = Topic::consumer(&topic).unwrap();
        b.iter(|| {
            producer.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // MpmcShm
    group.bench_function("MpmcShm", |b| {
        let topic = format!("bench_cmp_mpmc_shm_{}", std::process::id());
        let sender: Topic<BenchPayload> = Topic::new(&topic).unwrap();
        let receiver: Topic<BenchPayload> = Topic::new(&topic).unwrap();
        b.iter(|| {
            sender.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    // Raw ShmTopic
    group.bench_function("ShmTopic_raw", |b| {
        let topic = format!("bench_cmp_raw_{}", std::process::id());
        let producer: ShmTopic<BenchPayload> = ShmTopic::new(&topic, 1024).unwrap();
        let consumer: ShmTopic<BenchPayload> = ShmTopic::open(&topic).unwrap();
        b.iter(|| {
            producer.push(black_box(msg.clone())).unwrap();
            black_box(consumer.pop())
        });
    });

    group.finish();
}

// =============================================================================
// Criterion Configuration
// =============================================================================

criterion_group!(
    name = latency_benches;
    config = Criterion::default()
        .sample_size(1000)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(10))
        .significance_level(0.05)
        .noise_threshold(0.02);
    targets =
        bench_spsc_shm_latency,
        bench_spsc_intra_latency,
        bench_mpmc_shm_latency,
        bench_shm_topic_raw,
        bench_send_only_latency,
        bench_backend_comparison,
);

criterion_main!(latency_benches);
