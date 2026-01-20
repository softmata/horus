//! Competitive Comparison Benchmarks
//!
//! Compares HORUS IPC performance against industry-standard alternatives:
//! - crossbeam-channel (industry-standard Rust MPMC)
//! - flume (popular async-compatible channel)
//! - std::sync::mpsc (standard library)
//!
//! ## Methodology
//!
//! All comparisons use identical:
//! - Message sizes (64B control, 1KB sensor, 4KB image)
//! - Warmup periods (1000 iterations)
//! - Sample counts (1000 samples)
//! - Measurement time (10 seconds per benchmark)
//!
//! ## Running
//!
//! ```bash
//! cargo bench --bench competitive_comparison
//! cargo bench --bench competitive_comparison -- "64B"
//! cargo bench --bench competitive_comparison -- "crossbeam"
//! ```

use criterion::{
    black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput,
};
use crossbeam_channel::{bounded, unbounded};
use flume;
use horus::prelude::Topic;
use serde::{Deserialize, Serialize};
use std::sync::mpsc;
use std::time::Duration;

// =============================================================================
// Benchmark Payloads
// =============================================================================

/// 64-byte payload (control command size)
#[derive(Clone, Copy, Debug)]
pub struct Payload64B {
    data: [u8; 64],
}

impl Default for Payload64B {
    fn default() -> Self {
        Self { data: [0u8; 64] }
    }
}

/// 1KB payload (sensor data size)
#[derive(Clone, Debug)]
pub struct Payload1KB {
    data: Vec<u8>,
}

impl Default for Payload1KB {
    fn default() -> Self {
        Self {
            data: vec![0xAB; 1024],
        }
    }
}

/// 4KB payload (image chunk size)
#[derive(Clone, Debug)]
pub struct Payload4KB {
    data: Vec<u8>,
}

impl Default for Payload4KB {
    fn default() -> Self {
        Self {
            data: vec![0xAB; 4096],
        }
    }
}

// Serde implementations for HORUS compatibility
impl Serialize for Payload64B {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        // Serialize as a byte slice
        serializer.serialize_bytes(&self.data)
    }
}

impl<'de> Deserialize<'de> for Payload64B {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        // Deserialize from bytes
        struct Visitor;
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = Payload64B;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("64 bytes")
            }

            fn visit_bytes<E>(self, v: &[u8]) -> Result<Payload64B, E>
            where
                E: serde::de::Error,
            {
                if v.len() != 64 {
                    return Err(E::custom(format!("expected 64 bytes, got {}", v.len())));
                }
                let mut data = [0u8; 64];
                data.copy_from_slice(v);
                Ok(Payload64B { data })
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Payload64B, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mut data = [0u8; 64];
                for i in 0..64 {
                    data[i] = seq
                        .next_element()?
                        .ok_or_else(|| serde::de::Error::invalid_length(i, &self))?;
                }
                Ok(Payload64B { data })
            }
        }
        deserializer.deserialize_bytes(Visitor)
    }
}

impl Serialize for Payload1KB {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.data.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Payload1KB {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = Vec::<u8>::deserialize(deserializer)?;
        Ok(Self { data })
    }
}

impl Serialize for Payload4KB {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.data.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Payload4KB {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = Vec::<u8>::deserialize(deserializer)?;
        Ok(Self { data })
    }
}

impl horus_core::core::LogSummary for Payload64B {
    fn log_summary(&self) -> String {
        "Payload64B".to_string()
    }
}

impl horus_core::core::LogSummary for Payload1KB {
    fn log_summary(&self) -> String {
        "Payload1KB".to_string()
    }
}

impl horus_core::core::LogSummary for Payload4KB {
    fn log_summary(&self) -> String {
        "Payload4KB".to_string()
    }
}

// =============================================================================
// Section 1: 64-Byte Control Message Comparison
// =============================================================================

/// Compare all IPC mechanisms at 64B (control command, most latency-sensitive)
fn bench_64b_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("comparison/64B");
    group.throughput(Throughput::Bytes(64));
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    let msg = Payload64B::default();

    // --- HORUS Backends ---

    // HORUS SpscIntra (fastest same-process)
    group.bench_function("horus_spsc_intra", |b| {
        let topic = format!("cmp64_spsc_intra_{}", std::process::id());
        let (tx, rx): (Topic<Payload64B>, Topic<Payload64B>) = Topic::spsc_intra(&topic);

        for _ in 0..1000 {
            tx.send(msg, &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    // HORUS MpmcIntra
    group.bench_function("horus_mpmc_intra", |b| {
        let topic = format!("cmp64_mpmc_intra_{}", std::process::id());
        let (tx, rx): (Topic<Payload64B>, Topic<Payload64B>) = Topic::mpmc_intra(&topic, 1024);

        for _ in 0..1000 {
            tx.send(msg, &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    // HORUS SpscShm (cross-process capable)
    group.bench_function("horus_spsc_shm", |b| {
        let topic = format!("cmp64_spsc_shm_{}", std::process::id());
        let tx: Topic<Payload64B> = Topic::new(&topic).unwrap();
        let rx: Topic<Payload64B> = Topic::new(&topic).unwrap();

        for _ in 0..1000 {
            tx.send(msg, &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    // HORUS MpmcShm (most flexible)
    group.bench_function("horus_mpmc_shm", |b| {
        let topic = format!("cmp64_mpmc_shm_{}", std::process::id());
        let tx: Topic<Payload64B> = Topic::new(&topic).unwrap();
        let rx: Topic<Payload64B> = Topic::new(&topic).unwrap();

        for _ in 0..1000 {
            tx.send(msg, &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    // --- Competitors ---

    // crossbeam bounded (industry standard)
    group.bench_function("crossbeam_bounded", |b| {
        let (tx, rx) = bounded::<Payload64B>(1024);

        for _ in 0..1000 {
            tx.send(msg).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg)).unwrap();
            black_box(rx.recv())
        });
    });

    // crossbeam unbounded
    group.bench_function("crossbeam_unbounded", |b| {
        let (tx, rx) = unbounded::<Payload64B>();

        for _ in 0..1000 {
            tx.send(msg).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg)).unwrap();
            black_box(rx.recv())
        });
    });

    // flume bounded
    group.bench_function("flume_bounded", |b| {
        let (tx, rx) = flume::bounded::<Payload64B>(1024);

        for _ in 0..1000 {
            tx.send(msg).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg)).unwrap();
            black_box(rx.recv())
        });
    });

    // flume unbounded
    group.bench_function("flume_unbounded", |b| {
        let (tx, rx) = flume::unbounded::<Payload64B>();

        for _ in 0..1000 {
            tx.send(msg).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg)).unwrap();
            black_box(rx.recv())
        });
    });

    // std mpsc (baseline)
    group.bench_function("std_mpsc", |b| {
        let (tx, rx) = mpsc::channel::<Payload64B>();

        for _ in 0..1000 {
            tx.send(msg).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg)).unwrap();
            black_box(rx.recv())
        });
    });

    // std sync_channel (bounded)
    group.bench_function("std_sync_channel", |b| {
        let (tx, rx) = mpsc::sync_channel::<Payload64B>(1024);

        for _ in 0..1000 {
            tx.send(msg).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg)).unwrap();
            black_box(rx.recv())
        });
    });

    group.finish();
}

// =============================================================================
// Section 2: 1KB Sensor Data Comparison
// =============================================================================

/// Compare all IPC mechanisms at 1KB (typical sensor payload)
fn bench_1kb_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("comparison/1KB");
    group.throughput(Throughput::Bytes(1024));
    group.sample_size(1000);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    // --- HORUS Backends ---

    group.bench_function("horus_spsc_intra", |b| {
        let topic = format!("cmp1k_spsc_intra_{}", std::process::id());
        let (tx, rx): (Topic<Payload1KB>, Topic<Payload1KB>) = Topic::spsc_intra(&topic);
        let msg = Payload1KB::default();

        for _ in 0..1000 {
            tx.send(msg.clone(), &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    group.bench_function("horus_spsc_shm", |b| {
        let topic = format!("cmp1k_spsc_shm_{}", std::process::id());
        let tx: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        let rx: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        let msg = Payload1KB::default();

        for _ in 0..1000 {
            tx.send(msg.clone(), &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    group.bench_function("horus_mpmc_shm", |b| {
        let topic = format!("cmp1k_mpmc_shm_{}", std::process::id());
        let tx: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        let rx: Topic<Payload1KB> = Topic::new(&topic).unwrap();
        let msg = Payload1KB::default();

        for _ in 0..1000 {
            tx.send(msg.clone(), &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    // --- Competitors ---

    group.bench_function("crossbeam_bounded", |b| {
        let (tx, rx) = bounded::<Payload1KB>(1024);
        let msg = Payload1KB::default();

        for _ in 0..1000 {
            tx.send(msg.clone()).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg.clone())).unwrap();
            black_box(rx.recv())
        });
    });

    group.bench_function("flume_bounded", |b| {
        let (tx, rx) = flume::bounded::<Payload1KB>(1024);
        let msg = Payload1KB::default();

        for _ in 0..1000 {
            tx.send(msg.clone()).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg.clone())).unwrap();
            black_box(rx.recv())
        });
    });

    group.bench_function("std_sync_channel", |b| {
        let (tx, rx) = mpsc::sync_channel::<Payload1KB>(1024);
        let msg = Payload1KB::default();

        for _ in 0..1000 {
            tx.send(msg.clone()).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg.clone())).unwrap();
            black_box(rx.recv())
        });
    });

    group.finish();
}

// =============================================================================
// Section 3: 4KB Image Chunk Comparison
// =============================================================================

/// Compare all IPC mechanisms at 4KB (compressed image chunk)
fn bench_4kb_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("comparison/4KB");
    group.throughput(Throughput::Bytes(4096));
    group.sample_size(500); // Fewer samples for larger payloads
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));

    // --- HORUS Backends ---

    group.bench_function("horus_spsc_intra", |b| {
        let topic = format!("cmp4k_spsc_intra_{}", std::process::id());
        let (tx, rx): (Topic<Payload4KB>, Topic<Payload4KB>) = Topic::spsc_intra(&topic);
        let msg = Payload4KB::default();

        for _ in 0..500 {
            tx.send(msg.clone(), &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    group.bench_function("horus_spsc_shm", |b| {
        let topic = format!("cmp4k_spsc_shm_{}", std::process::id());
        let tx: Topic<Payload4KB> = Topic::new(&topic).unwrap();
        let rx: Topic<Payload4KB> = Topic::new(&topic).unwrap();
        let msg = Payload4KB::default();

        for _ in 0..500 {
            tx.send(msg.clone(), &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    group.bench_function("horus_mpmc_shm", |b| {
        let topic = format!("cmp4k_mpmc_shm_{}", std::process::id());
        let tx: Topic<Payload4KB> = Topic::new(&topic).unwrap();
        let rx: Topic<Payload4KB> = Topic::new(&topic).unwrap();
        let msg = Payload4KB::default();

        for _ in 0..500 {
            tx.send(msg.clone(), &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        b.iter(|| {
            tx.send(black_box(msg.clone()), &mut None).unwrap();
            black_box(rx.recv(&mut None))
        });
    });

    // --- Competitors ---

    group.bench_function("crossbeam_bounded", |b| {
        let (tx, rx) = bounded::<Payload4KB>(256);
        let msg = Payload4KB::default();

        for _ in 0..500 {
            tx.send(msg.clone()).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg.clone())).unwrap();
            black_box(rx.recv())
        });
    });

    group.bench_function("flume_bounded", |b| {
        let (tx, rx) = flume::bounded::<Payload4KB>(256);
        let msg = Payload4KB::default();

        for _ in 0..500 {
            tx.send(msg.clone()).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg.clone())).unwrap();
            black_box(rx.recv())
        });
    });

    group.bench_function("std_sync_channel", |b| {
        let (tx, rx) = mpsc::sync_channel::<Payload4KB>(256);
        let msg = Payload4KB::default();

        for _ in 0..500 {
            tx.send(msg.clone()).unwrap();
            let _ = rx.recv();
        }

        b.iter(|| {
            tx.send(black_box(msg.clone())).unwrap();
            black_box(rx.recv())
        });
    });

    group.finish();
}

// =============================================================================
// Section 4: Throughput Comparison (Sustained Load)
// =============================================================================

/// Measure sustained throughput (messages/sec) at 64B
fn bench_throughput_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("throughput/64B");
    group.throughput(Throughput::Elements(1));
    group.sample_size(100);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(5));

    let msg = Payload64B::default();
    let batch_size = 10_000;

    // HORUS SpscIntra burst
    group.bench_function("horus_spsc_intra_burst", |b| {
        let topic = format!("tput_spsc_intra_{}", std::process::id());
        let (tx, rx): (Topic<Payload64B>, Topic<Payload64B>) = Topic::spsc_intra(&topic);

        b.iter(|| {
            for _ in 0..batch_size {
                tx.send(black_box(msg), &mut None).unwrap();
            }
            for _ in 0..batch_size {
                black_box(rx.recv(&mut None));
            }
        });
    });

    // HORUS SpscShm burst
    group.bench_function("horus_spsc_shm_burst", |b| {
        let topic = format!("tput_spsc_shm_{}", std::process::id());
        let tx: Topic<Payload64B> = Topic::new(&topic).unwrap();
        let rx: Topic<Payload64B> = Topic::new(&topic).unwrap();

        b.iter(|| {
            for _ in 0..batch_size {
                tx.send(black_box(msg), &mut None).unwrap();
            }
            for _ in 0..batch_size {
                black_box(rx.recv(&mut None));
            }
        });
    });

    // crossbeam burst
    group.bench_function("crossbeam_bounded_burst", |b| {
        let (tx, rx) = bounded::<Payload64B>(batch_size + 1000);

        b.iter(|| {
            for _ in 0..batch_size {
                tx.send(black_box(msg)).unwrap();
            }
            for _ in 0..batch_size {
                black_box(rx.recv());
            }
        });
    });

    // flume burst
    group.bench_function("flume_bounded_burst", |b| {
        let (tx, rx) = flume::bounded::<Payload64B>(batch_size + 1000);

        b.iter(|| {
            for _ in 0..batch_size {
                tx.send(black_box(msg)).unwrap();
            }
            for _ in 0..batch_size {
                black_box(rx.recv());
            }
        });
    });

    group.finish();
}

// =============================================================================
// Criterion Configuration
// =============================================================================

criterion_group!(
    name = competitive_benches;
    config = Criterion::default()
        .sample_size(1000)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(10))
        .significance_level(0.05)
        .noise_threshold(0.02);
    targets =
        bench_64b_comparison,
        bench_1kb_comparison,
        bench_4kb_comparison,
        bench_throughput_comparison,
);

criterion_main!(competitive_benches);
