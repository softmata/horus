//! HORUS Topic Backend Performance Benchmarks
//!
//! Comprehensive benchmarks measuring IPC latency and throughput across all Topic backends.
//!
//! ## Backend Hierarchy (fastest to most flexible)
//!
//! | Backend       | Latency  | Use Case                          |
//! |---------------|----------|-----------------------------------|
//! | DirectChannel | ~3-5ns   | Same-thread pipelines             |
//! | SpscIntra     | ~15-25ns | Cross-thread point-to-point       |
//! | MpmcIntra     | ~30-50ns | Same-process pub/sub              |
//! | SpscShm       | ~80-100ns| Cross-process point-to-point      |
//! | MpmcShm       | ~150-200ns| Cross-process pub/sub (default)  |
//!
//! ## Benchmark Categories
//!
//! 1. **Backend Comparison** - Compare all backends with same payload
//! 2. **Cross-Thread Latency** - True IPC with separate producer/consumer threads
//! 3. **Throughput** - Sustained message rate under load
//! 4. **Payload Scaling** - How latency scales with message size
//!
//! ## Running Benchmarks
//!
//! ```bash
//! # Run all Topic benchmarks
//! cargo bench --bench topic_performance
//!
//! # Run specific group
//! cargo bench --bench topic_performance -- "backend_comparison"
//! cargo bench --bench topic_performance -- "cross_thread"
//! cargo bench --bench topic_performance -- "throughput"
//! ```

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// =============================================================================
// Section 1: Backend Comparison (Same-Thread Round-Trip)
// =============================================================================

/// Compare all Topic backends with 16-byte CmdVel payload.
/// Measures same-thread send+recv round-trip latency.
fn bench_backend_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("backend_comparison");
    group.throughput(Throughput::Bytes(std::mem::size_of::<CmdVel>() as u64));

    // DirectChannel - same-thread only (~3-5ns)
    group.bench_function("DirectChannel", |b| {
        let topic: Topic<CmdVel> = Topic::direct("bench_direct");
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            topic.send(black_box(msg), &mut None).unwrap();
            black_box(topic.recv(&mut None))
        });
    });

    // DirectChannel unchecked - absolute minimum (~3ns)
    group.bench_function("DirectChannel_unchecked", |b| {
        let topic: Topic<CmdVel> = Topic::direct("bench_direct_unc");
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            // SAFETY: Same thread, DirectChannel backend
            unsafe {
                topic.send_unchecked(black_box(msg));
                black_box(topic.recv_unchecked())
            }
        });
    });

    // SpscIntra - fastest cross-thread (~15-25ns)
    group.bench_function("SpscIntra", |b| {
        let topic = format!("bench_spsc_intra_{}", std::process::id());
        let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::spsc_intra(&topic);
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // MpmcIntra - flexible same-process (~30-50ns)
    group.bench_function("MpmcIntra", |b| {
        let topic = format!("bench_mpmc_intra_{}", std::process::id());
        let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::mpmc_intra(&topic, 64);
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // SpscShm - fastest cross-process (~80-100ns)
    group.bench_function("SpscShm", |b| {
        let topic = format!("bench_spsc_shm_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // MpmcShm - default, most flexible (~150-200ns)
    group.bench_function("MpmcShm", |b| {
        let topic = format!("bench_mpmc_shm_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 2: Cross-Thread Latency (True IPC)
// =============================================================================

/// Measure TRUE IPC latency with producer and consumer on separate threads.
/// This reflects real-world usage where threads communicate asynchronously.
fn bench_cross_thread_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("cross_thread_latency");
    group.measurement_time(Duration::from_secs(10));

    // SpscIntra cross-thread
    group.bench_function("SpscIntra", |b| {
        b.iter_custom(|iters| {
            let topic = format!("bench_cross_intra_{}", std::process::id());
            let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::spsc_intra(&topic);

            let running = Arc::new(AtomicBool::new(true));
            let running_clone = running.clone();
            let iters_to_send = iters;

            let producer_handle = thread::spawn(move || {
                for _ in 0..iters_to_send {
                    let msg = CmdVel::new(1.5, 0.8);
                    while producer.send(msg, &mut None).is_err() {
                        thread::yield_now();
                    }
                }
            });

            let start = Instant::now();
            let mut received = 0u64;
            while received < iters {
                if consumer.recv(&mut None).is_some() {
                    received += 1;
                } else if !running_clone.load(Ordering::Relaxed) {
                    break;
                } else {
                    thread::yield_now();
                }
            }
            let elapsed = start.elapsed();

            running.store(false, Ordering::Relaxed);
            let _ = producer_handle.join();
            elapsed
        });
    });

    // MpmcIntra cross-thread
    group.bench_function("MpmcIntra", |b| {
        b.iter_custom(|iters| {
            let topic = format!("bench_cross_mpmc_intra_{}", std::process::id());
            let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::mpmc_intra(&topic, 1024);

            let running = Arc::new(AtomicBool::new(true));
            let running_clone = running.clone();
            let iters_to_send = iters;

            let producer_handle = thread::spawn(move || {
                for _ in 0..iters_to_send {
                    let msg = CmdVel::new(1.5, 0.8);
                    while producer.send(msg, &mut None).is_err() {
                        thread::yield_now();
                    }
                }
            });

            let start = Instant::now();
            let mut received = 0u64;
            while received < iters {
                if consumer.recv(&mut None).is_some() {
                    received += 1;
                } else if !running_clone.load(Ordering::Relaxed) {
                    break;
                } else {
                    thread::yield_now();
                }
            }
            let elapsed = start.elapsed();

            running.store(false, Ordering::Relaxed);
            let _ = producer_handle.join();
            elapsed
        });
    });

    // SpscShm cross-thread
    group.bench_function("SpscShm", |b| {
        b.iter_custom(|iters| {
            let topic = format!("bench_cross_spsc_{}", std::process::id());
            let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
            let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();

            let running = Arc::new(AtomicBool::new(true));
            let running_clone = running.clone();
            let iters_to_send = iters;

            let producer_handle = thread::spawn(move || {
                for _ in 0..iters_to_send {
                    let msg = CmdVel::new(1.5, 0.8);
                    while producer.send(msg, &mut None).is_err() {
                        thread::yield_now();
                    }
                }
            });

            let start = Instant::now();
            let mut received = 0u64;
            while received < iters {
                if consumer.recv(&mut None).is_some() {
                    received += 1;
                } else if !running_clone.load(Ordering::Relaxed) {
                    break;
                } else {
                    thread::yield_now();
                }
            }
            let elapsed = start.elapsed();

            running.store(false, Ordering::Relaxed);
            let _ = producer_handle.join();
            elapsed
        });
    });

    // MpmcShm cross-thread
    group.bench_function("MpmcShm", |b| {
        b.iter_custom(|iters| {
            let topic = format!("bench_cross_mpmc_{}", std::process::id());
            let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
            let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();

            let running = Arc::new(AtomicBool::new(true));
            let running_clone = running.clone();
            let iters_to_send = iters;

            let producer_handle = thread::spawn(move || {
                for _ in 0..iters_to_send {
                    let msg = CmdVel::new(1.5, 0.8);
                    while producer.send(msg, &mut None).is_err() {
                        thread::yield_now();
                    }
                }
            });

            let start = Instant::now();
            let mut received = 0u64;
            while received < iters {
                if consumer.recv(&mut None).is_some() {
                    received += 1;
                } else if !running_clone.load(Ordering::Relaxed) {
                    break;
                } else {
                    thread::yield_now();
                }
            }
            let elapsed = start.elapsed();

            running.store(false, Ordering::Relaxed);
            let _ = producer_handle.join();
            elapsed
        });
    });

    group.finish();
}

// =============================================================================
// Section 3: Throughput Benchmarks
// =============================================================================

/// Measure sustained throughput - messages per second under continuous load.
fn bench_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("throughput");
    group.measurement_time(Duration::from_secs(10));

    // SpscIntra throughput (1000 messages per iteration)
    group.bench_function("SpscIntra_1k_msgs", |b| {
        let topic = format!("bench_throughput_intra_{}", std::process::id());
        let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::spsc_intra(&topic);

        b.iter(|| {
            for i in 0..1000 {
                let msg = CmdVel::new(1.0 + i as f32 * 0.001, 0.8);
                producer.send(black_box(msg), &mut None).unwrap();
            }
            for _ in 0..1000 {
                black_box(consumer.recv(&mut None));
            }
        });
    });

    // MpmcIntra throughput
    group.bench_function("MpmcIntra_1k_msgs", |b| {
        let topic = format!("bench_throughput_mpmc_intra_{}", std::process::id());
        let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::mpmc_intra(&topic, 2048);

        b.iter(|| {
            for i in 0..1000 {
                let msg = CmdVel::new(1.0 + i as f32 * 0.001, 0.8);
                producer.send(black_box(msg), &mut None).unwrap();
            }
            for _ in 0..1000 {
                black_box(consumer.recv(&mut None));
            }
        });
    });

    // SpscShm throughput
    group.bench_function("SpscShm_1k_msgs", |b| {
        let topic = format!("bench_throughput_spsc_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter(|| {
            for i in 0..1000 {
                let msg = CmdVel::new(1.0 + i as f32 * 0.001, 0.8);
                producer.send(black_box(msg), &mut None).unwrap();
            }
            for _ in 0..1000 {
                black_box(consumer.recv(&mut None));
            }
        });
    });

    // MpmcShm throughput
    group.bench_function("MpmcShm_1k_msgs", |b| {
        let topic = format!("bench_throughput_mpmc_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter(|| {
            for i in 0..1000 {
                let msg = CmdVel::new(1.0 + i as f32 * 0.001, 0.8);
                producer.send(black_box(msg), &mut None).unwrap();
            }
            for _ in 0..1000 {
                black_box(consumer.recv(&mut None));
            }
        });
    });

    group.finish();
}

// =============================================================================
// Section 4: Send/Recv Isolation
// =============================================================================

/// Measure send-only latency (isolate write performance).
fn bench_send_only(c: &mut Criterion) {
    let mut group = c.benchmark_group("send_only");
    group.throughput(Throughput::Bytes(std::mem::size_of::<CmdVel>() as u64));

    group.bench_function("SpscIntra", |b| {
        let topic = format!("bench_send_intra_{}", std::process::id());
        let (producer, _consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::spsc_intra(&topic);
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None)
        });
    });

    group.bench_function("MpmcIntra", |b| {
        let topic = format!("bench_send_mpmc_intra_{}", std::process::id());
        let (producer, _consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::mpmc_intra(&topic, 1024);
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None)
        });
    });

    group.bench_function("SpscShm", |b| {
        let topic = format!("bench_send_spsc_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let _consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None)
        });
    });

    group.bench_function("MpmcShm", |b| {
        let topic = format!("bench_send_mpmc_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let _consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            producer.send(black_box(msg), &mut None)
        });
    });

    group.finish();
}

/// Measure recv-only latency (isolate read performance with pre-sent message).
fn bench_recv_only(c: &mut Criterion) {
    let mut group = c.benchmark_group("recv_only");
    group.throughput(Throughput::Bytes(std::mem::size_of::<CmdVel>() as u64));

    group.bench_function("SpscIntra", |b| {
        let topic = format!("bench_recv_intra_{}", std::process::id());
        let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::spsc_intra(&topic);

        b.iter_batched(
            || producer.send(CmdVel::new(1.5, 0.8), &mut None).unwrap(),
            |_| black_box(consumer.recv(&mut None)),
            criterion::BatchSize::SmallInput,
        );
    });

    group.bench_function("MpmcIntra", |b| {
        let topic = format!("bench_recv_mpmc_intra_{}", std::process::id());
        let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::mpmc_intra(&topic, 1024);

        b.iter_batched(
            || producer.send(CmdVel::new(1.5, 0.8), &mut None).unwrap(),
            |_| black_box(consumer.recv(&mut None)),
            criterion::BatchSize::SmallInput,
        );
    });

    group.bench_function("SpscShm", |b| {
        let topic = format!("bench_recv_spsc_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter_batched(
            || producer.send(CmdVel::new(1.5, 0.8), &mut None).unwrap(),
            |_| black_box(consumer.recv(&mut None)),
            criterion::BatchSize::SmallInput,
        );
    });

    group.bench_function("MpmcShm", |b| {
        let topic = format!("bench_recv_mpmc_{}", std::process::id());
        let producer: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let consumer: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter_batched(
            || producer.send(CmdVel::new(1.5, 0.8), &mut None).unwrap(),
            |_| black_box(consumer.recv(&mut None)),
            criterion::BatchSize::SmallInput,
        );
    });

    group.finish();
}

// =============================================================================
// Section 5: Payload Size Scaling
// =============================================================================

/// Fixed-size payloads for benchmarking
#[derive(Clone, Copy, Debug)]
struct Payload64([u8; 64]);
#[derive(Clone, Copy, Debug)]
struct Payload256([u8; 256]);
#[derive(Clone, Copy, Debug)]
struct Payload1KB([u8; 1024]);
#[derive(Clone, Copy, Debug)]
struct Payload4KB([u8; 4096]);

/// Measure how latency scales with payload size using SpscIntra.
fn bench_payload_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("payload_scaling_SpscIntra");

    // 64 bytes
    group.throughput(Throughput::Bytes(64));
    group.bench_with_input(BenchmarkId::new("roundtrip", "64B"), &(), |b, _| {
        let topic = format!("bench_scale_64_{}", std::process::id());
        let (producer, consumer): (Topic<Payload64>, Topic<Payload64>) = Topic::spsc_intra(&topic);
        let msg = Payload64([0u8; 64]);
        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 256 bytes
    group.throughput(Throughput::Bytes(256));
    group.bench_with_input(BenchmarkId::new("roundtrip", "256B"), &(), |b, _| {
        let topic = format!("bench_scale_256_{}", std::process::id());
        let (producer, consumer): (Topic<Payload256>, Topic<Payload256>) = Topic::spsc_intra(&topic);
        let msg = Payload256([0u8; 256]);
        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 1 KB
    group.throughput(Throughput::Bytes(1024));
    group.bench_with_input(BenchmarkId::new("roundtrip", "1KB"), &(), |b, _| {
        let topic = format!("bench_scale_1k_{}", std::process::id());
        let (producer, consumer): (Topic<Payload1KB>, Topic<Payload1KB>) = Topic::spsc_intra(&topic);
        let msg = Payload1KB([0u8; 1024]);
        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    // 4 KB
    group.throughput(Throughput::Bytes(4096));
    group.bench_with_input(BenchmarkId::new("roundtrip", "4KB"), &(), |b, _| {
        let topic = format!("bench_scale_4k_{}", std::process::id());
        let (producer, consumer): (Topic<Payload4KB>, Topic<Payload4KB>) = Topic::spsc_intra(&topic);
        let msg = Payload4KB([0u8; 4096]);
        b.iter(|| {
            producer.send(black_box(msg), &mut None).unwrap();
            black_box(consumer.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 6: Batch Operations
// =============================================================================

/// Measure batch send/recv performance (amortize overhead).
fn bench_batch_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_operations");

    for batch_size in [10, 50, 100, 500] {
        group.bench_with_input(
            BenchmarkId::new("SpscIntra_batch", batch_size),
            &batch_size,
            |b, &batch_size| {
                let topic = format!("bench_batch_{}_{}", batch_size, std::process::id());
                let (producer, consumer): (Topic<CmdVel>, Topic<CmdVel>) = Topic::spsc_intra(&topic);

                b.iter(|| {
                    // Send batch
                    for i in 0..batch_size {
                        let msg = CmdVel::new(1.0 + i as f32 * 0.01, 0.8);
                        producer.send(black_box(msg), &mut None).unwrap();
                    }
                    // Receive batch
                    for _ in 0..batch_size {
                        black_box(consumer.recv(&mut None));
                    }
                });
            },
        );
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
        .warm_up_time(Duration::from_secs(2))
        .measurement_time(Duration::from_secs(5));
    targets =
        bench_backend_comparison,
        bench_cross_thread_latency,
        bench_throughput,
        bench_send_only,
        bench_recv_only,
        bench_payload_scaling,
        bench_batch_operations,
);

criterion_main!(benches);
