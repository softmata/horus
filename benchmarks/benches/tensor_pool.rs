//! TensorPool Benchmarks
//!
//! Measures allocation, release, and data access performance for the
//! shared memory tensor pool used in zero-copy tensor IPC.
//!
//! Run with: cargo bench -p horus_benchmarks -- tensor_pool

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use horus_core::core::DurationExt;
use horus_core::memory::{shm_base_dir, TensorPool, TensorPoolConfig};
use horus_core::types::{Device, TensorDtype};

/// Clean up tensor SHM files to avoid interference between bench iterations.
fn cleanup_tensor_shm() {
    let tensors_dir = shm_base_dir().join("tensors");
    let _ = std::fs::remove_dir_all(&tensors_dir);
}

fn make_pool(pool_id: u32, size_mb: usize, max_slots: usize) -> TensorPool {
    TensorPool::new(
        pool_id,
        TensorPoolConfig {
            pool_size: size_mb * 1024 * 1024,
            max_slots,
            slot_alignment: 64,
            ..Default::default()
        },
    )
    .expect("Failed to create tensor pool")
}

/// Benchmark: alloc + release cycle for various tensor sizes.
fn bench_alloc_release(c: &mut Criterion) {
    let mut group = c.benchmark_group("tensor_alloc_release");
    group.measurement_time(5_u64.secs());

    // Each iteration creates a fresh pool to avoid data region exhaustion
    let sizes: &[(&str, &[u64], TensorDtype)] = &[
        ("64B_u8", &[64], TensorDtype::U8),
        ("1KB_f32", &[256], TensorDtype::F32),
        ("16KB_f32", &[4096], TensorDtype::F32),
        ("256KB_f32", &[256, 256], TensorDtype::F32),
        ("1MB_f32", &[512, 512], TensorDtype::F32),
        ("6MB_u8_1080p", &[1080, 1920, 3], TensorDtype::U8),
    ];

    for (name, shape, dtype) in sizes {
        group.bench_function(BenchmarkId::new("alloc_release", name), |b| {
            cleanup_tensor_shm();
            let pool_id = 9500 + (std::process::id() % 100) as u32;
            let pool = make_pool(pool_id, 512, 1024);

            b.iter(|| {
                let tensor = pool
                    .alloc(black_box(shape), *dtype, Device::cpu())
                    .expect("alloc failed");
                pool.release(black_box(&tensor));
            });

            cleanup_tensor_shm();
        });
    }

    group.finish();
}

/// Benchmark: alloc only (no release) to measure pure allocation cost.
/// Uses a large pool to avoid exhaustion within the benchmark window.
fn bench_alloc_only(c: &mut Criterion) {
    let mut group = c.benchmark_group("tensor_alloc_only");
    group.measurement_time(3_u64.secs());
    group.sample_size(50);

    group.bench_function("1KB_f32", |b| {
        cleanup_tensor_shm();
        let pool_id = 9510 + (std::process::id() % 100) as u32;
        // Large pool: each 1KB alloc consumes 1KB permanently,
        // need enough for all benchmark iterations
        let pool = make_pool(pool_id, 512, 65536);
        let mut tensors = Vec::with_capacity(100_000);

        b.iter(|| {
            match pool.alloc(&[256], TensorDtype::F32, Device::cpu()) {
                Ok(t) => tensors.push(t),
                Err(_) => {} // data region exhausted, expected
            }
        });

        // Release all at end
        for t in &tensors {
            pool.release(t);
        }
        cleanup_tensor_shm();
    });

    group.finish();
}

/// Benchmark: data_slice_mut access (write to allocated tensor).
fn bench_data_access(c: &mut Criterion) {
    let mut group = c.benchmark_group("tensor_data_access");
    group.measurement_time(3_u64.secs());

    cleanup_tensor_shm();
    let pool_id = 9520 + (std::process::id() % 100) as u32;
    let pool = make_pool(pool_id, 128, 64);

    // Pre-allocate tensors of various sizes
    let small = pool.alloc(&[256], TensorDtype::F32, Device::cpu()).unwrap();
    let medium = pool
        .alloc(&[256, 256], TensorDtype::F32, Device::cpu())
        .unwrap();

    group.bench_function("data_slice_1KB", |b| {
        b.iter(|| {
            let data = pool.data_slice(black_box(&small)).unwrap();
            black_box(data[0]);
        });
    });

    group.bench_function("data_slice_mut_1KB", |b| {
        b.iter(|| {
            let data = pool.data_slice_mut(black_box(&small)).unwrap();
            data[0] = black_box(0xAB);
        });
    });

    group.bench_function("data_slice_256KB", |b| {
        b.iter(|| {
            let data = pool.data_slice(black_box(&medium)).unwrap();
            black_box(data[0]);
        });
    });

    pool.release(&small);
    pool.release(&medium);

    group.finish();
    cleanup_tensor_shm();
}

/// Benchmark: concurrent alloc/release from 4 threads.
fn bench_concurrent_alloc(c: &mut Criterion) {
    let mut group = c.benchmark_group("tensor_concurrent");
    group.measurement_time(5_u64.secs());
    group.sample_size(50);

    group.bench_function("4_thread_alloc_release", |b| {
        cleanup_tensor_shm();
        let pool_id = 9530 + (std::process::id() % 100) as u32;
        let pool = std::sync::Arc::new(make_pool(pool_id, 512, 4096));

        b.iter(|| {
            let mut handles = Vec::new();
            for _ in 0..4 {
                let pool = pool.clone();
                handles.push(std::thread::spawn(move || {
                    for _ in 0..100 {
                        if let Ok(t) = pool.alloc(&[64], TensorDtype::U8, Device::cpu()) {
                            pool.release(&t);
                        }
                    }
                }));
            }
            for h in handles {
                h.join().unwrap();
            }
        });

        cleanup_tensor_shm();
    });

    group.finish();
}

/// Benchmark: pool.stats() call overhead.
fn bench_pool_stats(c: &mut Criterion) {
    let mut group = c.benchmark_group("tensor_stats");

    cleanup_tensor_shm();
    let pool_id = 9540 + (std::process::id() % 100) as u32;
    let pool = make_pool(pool_id, 64, 256);

    // Pre-allocate some tensors so stats has work to do
    let mut tensors = Vec::new();
    for _ in 0..100 {
        if let Ok(t) = pool.alloc(&[64], TensorDtype::U8, Device::cpu()) {
            tensors.push(t);
        }
    }

    group.bench_function("stats_100_slots", |b| {
        b.iter(|| {
            black_box(pool.stats());
        });
    });

    for t in &tensors {
        pool.release(t);
    }

    group.finish();
    cleanup_tensor_shm();
}

criterion_group!(
    benches,
    bench_alloc_release,
    bench_alloc_only,
    bench_data_access,
    bench_concurrent_alloc,
    bench_pool_stats,
);
criterion_main!(benches);
