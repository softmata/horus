//! Payload Size Latency Matrix Benchmarks
//!
//! Deep analysis of how latency scales with payload size across Topic backends.
//! Uses high sample counts and percentile analysis for accurate measurements.
//!
//! ## Payload Sizes Tested
//!
//! | Size   | Use Case                           |
//! |--------|-----------------------------------|
//! | 64B    | Control commands, status updates  |
//! | 1KB    | Typical sensor data               |
//! | 4KB    | Dense sensor / small images       |
//! | 64KB   | Compressed images                 |
//!
//! ## Running Benchmarks
//!
//! ```bash
//! cargo bench --bench latency_matrix
//! cargo bench --bench latency_matrix -- "spsc_shm"
//! cargo bench --bench latency_matrix -- "ipc_comparison"
//! ```

use criterion::{
    black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput,
};
use horus::prelude::Topic;
use horus_core::memory::ShmTopic;
use serde::{Deserialize, Serialize};
use std::time::Duration;

// =============================================================================
// Payload Types
// =============================================================================

/// Serde helper for large arrays
mod big_array {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S, const N: usize>(arr: &[u8; N], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        arr.as_slice().serialize(serializer)
    }

    pub fn deserialize<'de, D, const N: usize>(deserializer: D) -> Result<[u8; N], D::Error>
    where
        D: Deserializer<'de>,
    {
        let vec = Vec::<u8>::deserialize(deserializer)?;
        vec.try_into()
            .map_err(|_| serde::de::Error::custom("array size mismatch"))
    }
}

/// 64-byte payload - minimal control message
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Payload64B {
    #[serde(with = "big_array")]
    data: [u8; 64],
}

impl Default for Payload64B {
    fn default() -> Self {
        Self { data: [0u8; 64] }
    }
}

impl horus_core::core::LogSummary for Payload64B {
    fn log_summary(&self) -> String {
        "Payload64B".to_string()
    }
}

/// 1KB payload - typical sensor reading
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Payload1KB {
    #[serde(with = "big_array")]
    data: [u8; 1024],
}

impl Default for Payload1KB {
    fn default() -> Self {
        Self { data: [0u8; 1024] }
    }
}

impl horus_core::core::LogSummary for Payload1KB {
    fn log_summary(&self) -> String {
        "Payload1KB".to_string()
    }
}

/// 4KB payload - small image / dense sensor
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Payload4KB {
    #[serde(with = "big_array")]
    data: [u8; 4096],
}

impl Default for Payload4KB {
    fn default() -> Self {
        Self { data: [0u8; 4096] }
    }
}

impl horus_core::core::LogSummary for Payload4KB {
    fn log_summary(&self) -> String {
        "Payload4KB".to_string()
    }
}

/// 64KB payload - compressed image
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Payload64KB {
    #[serde(with = "big_array")]
    data: [u8; 65536],
}

impl Default for Payload64KB {
    fn default() -> Self {
        Self { data: [0u8; 65536] }
    }
}

impl horus_core::core::LogSummary for Payload64KB {
    fn log_summary(&self) -> String {
        "Payload64KB".to_string()
    }
}

// =============================================================================
// Section 1: SpscShm Payload Matrix
// =============================================================================

/// SpscShm latency across payload sizes (cross-process SPSC, ~80-100ns base)
fn bench_spsc_shm_matrix(c: &mut Criterion) {
    let mut group = c.benchmark_group("spsc_shm_matrix");
    group.sample_size(500);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(5));

    // 64B
    group.throughput(Throughput::Bytes(64));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64B"), &(), |b, _| {
        let topic = format!("bench_spsc_64b_{}", std::process::id());
        let producer: Topic<Payload64B> = Topic::producer(&topic).unwrap();
        let consumer: Topic<Payload64B> = Topic::consumer(&topic).unwrap();
        let msg = Payload64B::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 1KB
    group.throughput(Throughput::Bytes(1024));
    group.bench_with_input(BenchmarkId::new("roundtrip", "1KB"), &(), |b, _| {
        let topic = format!("bench_spsc_1kb_{}", std::process::id());
        let producer: Topic<Payload1KB> = Topic::producer(&topic).unwrap();
        let consumer: Topic<Payload1KB> = Topic::consumer(&topic).unwrap();
        let msg = Payload1KB::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 4KB
    group.throughput(Throughput::Bytes(4096));
    group.bench_with_input(BenchmarkId::new("roundtrip", "4KB"), &(), |b, _| {
        let topic = format!("bench_spsc_4kb_{}", std::process::id());
        let producer: Topic<Payload4KB> = Topic::producer(&topic).unwrap();
        let consumer: Topic<Payload4KB> = Topic::consumer(&topic).unwrap();
        let msg = Payload4KB::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 64KB
    group.throughput(Throughput::Bytes(65536));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64KB"), &(), |b, _| {
        let topic = format!("bench_spsc_64kb_{}", std::process::id());
        let producer: Topic<Payload64KB> = Topic::producer(&topic).unwrap();
        let consumer: Topic<Payload64KB> = Topic::consumer(&topic).unwrap();
        let msg = Payload64KB::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 2: MpmcShm Payload Matrix
// =============================================================================

/// MpmcShm latency across payload sizes (cross-process MPMC, ~150-200ns base)
fn bench_mpmc_shm_matrix(c: &mut Criterion) {
    let mut group = c.benchmark_group("mpmc_shm_matrix");
    group.sample_size(500);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(5));

    // 64B
    group.throughput(Throughput::Bytes(64));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64B"), &(), |b, _| {
        let topic = format!("bench_mpmc_64b_{}", std::process::id());
        let sender: Topic<Payload64B> = Topic::new(&topic).unwrap();
        let receiver: Topic<Payload64B> = Topic::new(&topic).unwrap();
        let msg = Payload64B::default();

        b.iter(|| {
            sender.send(black_box(msg), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    // 1KB
    group.throughput(Throughput::Bytes(1024));
    group.bench_with_input(BenchmarkId::new("roundtrip", "1KB"), &(), |b, _| {
        let topic = format!("bench_mpmc_1kb_{}", std::process::id());
        let sender: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        let receiver: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        let msg = Payload1KB::default();

        b.iter(|| {
            sender.send(black_box(msg), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    // 4KB
    group.throughput(Throughput::Bytes(4096));
    group.bench_with_input(BenchmarkId::new("roundtrip", "4KB"), &(), |b, _| {
        let topic = format!("bench_mpmc_4kb_{}", std::process::id());
        let sender: Topic<Payload4KB> = Topic::new(&topic).unwrap();
        let receiver: Topic<Payload4KB> = Topic::new(&topic).unwrap();
        let msg = Payload4KB::default();

        b.iter(|| {
            sender.send(black_box(msg), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    // 64KB
    group.throughput(Throughput::Bytes(65536));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64KB"), &(), |b, _| {
        let topic = format!("bench_mpmc_64kb_{}", std::process::id());
        let sender: Topic<Payload64KB> = Topic::new(&topic).unwrap();
        let receiver: Topic<Payload64KB> = Topic::new(&topic).unwrap();
        let msg = Payload64KB::default();

        b.iter(|| {
            sender.send(black_box(msg), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 3: SpscIntra Payload Matrix
// =============================================================================

/// SpscIntra latency across payload sizes (same-process SPSC, ~15-25ns base)
fn bench_spsc_intra_matrix(c: &mut Criterion) {
    let mut group = c.benchmark_group("spsc_intra_matrix");
    group.sample_size(500);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(5));

    // 64B
    group.throughput(Throughput::Bytes(64));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64B"), &(), |b, _| {
        let topic = format!("bench_intra_64b_{}", std::process::id());
        let (producer, consumer): (Topic<Payload64B>, Topic<Payload64B>) = Topic::spsc_intra(&topic);
        let msg = Payload64B::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 1KB
    group.throughput(Throughput::Bytes(1024));
    group.bench_with_input(BenchmarkId::new("roundtrip", "1KB"), &(), |b, _| {
        let topic = format!("bench_intra_1kb_{}", std::process::id());
        let (producer, consumer): (Topic<Payload1KB>, Topic<Payload1KB>) = Topic::spsc_intra(&topic);
        let msg = Payload1KB::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 4KB
    group.throughput(Throughput::Bytes(4096));
    group.bench_with_input(BenchmarkId::new("roundtrip", "4KB"), &(), |b, _| {
        let topic = format!("bench_intra_4kb_{}", std::process::id());
        let (producer, consumer): (Topic<Payload4KB>, Topic<Payload4KB>) = Topic::spsc_intra(&topic);
        let msg = Payload4KB::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 64KB
    group.throughput(Throughput::Bytes(65536));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64KB"), &(), |b, _| {
        let topic = format!("bench_intra_64kb_{}", std::process::id());
        let (producer, consumer): (Topic<Payload64KB>, Topic<Payload64KB>) =
            Topic::spsc_intra(&topic);
        let msg = Payload64KB::default();

        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 4: Raw ShmTopic Comparison
// =============================================================================

/// Low-level ShmTopic latency (ring buffer only, no Topic wrapper)
fn bench_shm_topic_matrix(c: &mut Criterion) {
    let mut group = c.benchmark_group("shm_topic_matrix");
    group.sample_size(500);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(5));

    // 64B
    group.throughput(Throughput::Bytes(64));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64B"), &(), |b, _| {
        let topic = format!("bench_shm_64b_{}", std::process::id());
        let producer: ShmTopic<Payload64B> = ShmTopic::new(&topic, 1024).unwrap();
        let consumer: ShmTopic<Payload64B> = ShmTopic::open(&topic).unwrap();
        let msg = Payload64B::default();

        b.iter(|| {
            producer.push(black_box(msg)).unwrap();
            black_box(consumer.pop())
        });
    });

    // 1KB
    group.throughput(Throughput::Bytes(1024));
    group.bench_with_input(BenchmarkId::new("roundtrip", "1KB"), &(), |b, _| {
        let topic = format!("bench_shm_1kb_{}", std::process::id());
        let producer: ShmTopic<Payload1KB> = ShmTopic::new(&topic, 256).unwrap();
        let consumer: ShmTopic<Payload1KB> = ShmTopic::open(&topic).unwrap();
        let msg = Payload1KB::default();

        b.iter(|| {
            producer.push(black_box(msg)).unwrap();
            black_box(consumer.pop())
        });
    });

    // 4KB
    group.throughput(Throughput::Bytes(4096));
    group.bench_with_input(BenchmarkId::new("roundtrip", "4KB"), &(), |b, _| {
        let topic = format!("bench_shm_4kb_{}", std::process::id());
        let producer: ShmTopic<Payload4KB> = ShmTopic::new(&topic, 128).unwrap();
        let consumer: ShmTopic<Payload4KB> = ShmTopic::open(&topic).unwrap();
        let msg = Payload4KB::default();

        b.iter(|| {
            producer.push(black_box(msg)).unwrap();
            black_box(consumer.pop())
        });
    });

    // 64KB
    group.throughput(Throughput::Bytes(65536));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64KB"), &(), |b, _| {
        let topic = format!("bench_shm_64kb_{}", std::process::id());
        let producer: ShmTopic<Payload64KB> = ShmTopic::new(&topic, 32).unwrap();
        let consumer: ShmTopic<Payload64KB> = ShmTopic::open(&topic).unwrap();
        let msg = Payload64KB::default();

        b.iter(|| {
            producer.push(black_box(msg)).unwrap();
            black_box(consumer.pop())
        });
    });

    group.finish();
}

// =============================================================================
// Section 5: Full Backend Comparison at 1KB
// =============================================================================

/// Compare all IPC mechanisms at 1KB payload (standard reference size)
fn bench_ipc_comparison_1kb(c: &mut Criterion) {
    let mut group = c.benchmark_group("ipc_comparison_1KB");
    group.throughput(Throughput::Bytes(1024));
    group.sample_size(500);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(5));

    let msg = Payload1KB::default();

    // DirectChannel (same-thread only)
    group.bench_function("DirectChannel", |b| {
        let topic: Topic<Payload1KB> = Topic::direct("bench_cmp_direct");
        b.iter(|| {
            topic.send(black_box(msg), &mut None).unwrap();
            black_box(topic.recv(&mut None))
        });
    });

    // SpscIntra (fastest cross-thread)
    group.bench_function("SpscIntra", |b| {
        let topic = format!("bench_cmp_intra_{}", std::process::id());
        let (producer, consumer): (Topic<Payload1KB>, Topic<Payload1KB>) = Topic::spsc_intra(&topic);
        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // MpmcIntra (flexible same-process)
    group.bench_function("MpmcIntra", |b| {
        let topic = format!("bench_cmp_mpmc_intra_{}", std::process::id());
        let (producer, consumer): (Topic<Payload1KB>, Topic<Payload1KB>) =
            Topic::mpmc_intra(&topic, 256);
        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // SpscShm (fastest cross-process)
    group.bench_function("SpscShm", |b| {
        let topic = format!("bench_cmp_spsc_{}", std::process::id());
        let producer: Topic<Payload1KB> = Topic::producer(&topic).unwrap();
        let consumer: Topic<Payload1KB> = Topic::consumer(&topic).unwrap();
        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // MpmcShm (most flexible)
    group.bench_function("MpmcShm", |b| {
        let topic = format!("bench_cmp_mpmc_{}", std::process::id());
        let sender: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        let receiver: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        b.iter(|| {
            sender.send(black_box(msg), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    // Raw ShmTopic (low-level baseline)
    group.bench_function("ShmTopic_raw", |b| {
        let topic = format!("bench_cmp_shm_{}", std::process::id());
        let producer: ShmTopic<Payload1KB> = ShmTopic::new(&topic, 256).unwrap();
        let consumer: ShmTopic<Payload1KB> = ShmTopic::open(&topic).unwrap();
        b.iter(|| {
            producer.push(black_box(msg)).unwrap();
            black_box(consumer.pop())
        });
    });

    group.finish();
}

// =============================================================================
// Criterion Configuration
// =============================================================================

criterion_group!(
    name = benches;
    config = Criterion::default()
        .sample_size(500)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(5));
    targets =
        bench_spsc_shm_matrix,
        bench_mpmc_shm_matrix,
        bench_spsc_intra_matrix,
        bench_shm_topic_matrix,
        bench_ipc_comparison_1kb,
);

criterion_main!(benches);
