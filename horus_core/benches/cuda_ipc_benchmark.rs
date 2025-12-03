//! CUDA IPC Benchmark Suite
//!
//! Benchmarks for GPU tensor sharing via CUDA IPC.
//! Measures allocation, IPC handle generation, and handle import latency.
//!
//! Run with: cargo bench -p horus_benchmarks --bench cuda_ipc_benchmark --features cuda

#![cfg(feature = "cuda")]

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use horus_core::memory::tensor_pool::TensorDtype;
use horus_core::memory::{cuda_available, CudaTensorPool, CudaTensorPoolConfig};
use std::time::Duration;

/// Benchmark CUDA tensor allocation performance
fn bench_cuda_alloc(c: &mut Criterion) {
    if !cuda_available() {
        eprintln!("Skipping CUDA benchmarks - no GPU available");
        return;
    }

    let mut group = c.benchmark_group("cuda_alloc");
    group.measurement_time(Duration::from_secs(5));

    // Different tensor sizes to benchmark
    let sizes: Vec<(u64, u64, u64, &str)> = vec![
        (64, 64, 3, "64x64 RGB"),
        (640, 480, 3, "VGA"),
        (1280, 720, 3, "720p"),
        (1920, 1080, 3, "1080p"),
        (3840, 2160, 3, "4K"),
    ];

    for (h, w, c, name) in sizes {
        let bytes = h * w * c * 4; // f32 = 4 bytes
        group.throughput(Throughput::Bytes(bytes));

        group.bench_with_input(
            BenchmarkId::new("f32", name),
            &(h, w, c),
            |b, &(h, w, c)| {
                let pool =
                    CudaTensorPool::new(rand::random::<u32>(), 0, CudaTensorPoolConfig::default())
                        .expect("Failed to create pool");

                b.iter(|| {
                    let tensor = pool
                        .alloc(&[h, w, c], TensorDtype::F32)
                        .expect("Failed to allocate");
                    black_box(&tensor);
                    pool.release(&tensor).expect("Failed to release");
                });
            },
        );
    }

    group.finish();
}

/// Benchmark IPC handle generation (cudaIpcGetMemHandle)
fn bench_ipc_handle_generation(c: &mut Criterion) {
    if !cuda_available() {
        return;
    }

    let mut group = c.benchmark_group("cuda_ipc_handle");
    group.measurement_time(Duration::from_secs(5));

    let pool = CudaTensorPool::new(rand::random::<u32>(), 0, CudaTensorPoolConfig::default())
        .expect("Failed to create pool");

    // 1080p tensor
    let tensor = pool
        .alloc(&[1920, 1080, 3], TensorDtype::F32)
        .expect("Failed to allocate");

    group.bench_function("get_ipc_handle_1080p", |b| {
        b.iter(|| {
            let handle = tensor.ipc_handle_bytes();
            black_box(handle);
        });
    });

    // Just measure IPC handle size verification
    group.bench_function("ipc_handle_size_check", |b| {
        b.iter(|| {
            let handle = tensor.ipc_handle_bytes();
            assert_eq!(black_box(handle.len()), 64);
        });
    });

    pool.release(&tensor).expect("Failed to release");
    group.finish();
}

/// Benchmark tensor pool creation/opening
fn bench_pool_operations(c: &mut Criterion) {
    if !cuda_available() {
        return;
    }

    let mut group = c.benchmark_group("cuda_pool");
    group.measurement_time(Duration::from_secs(3));

    // Pool creation benchmark
    group.bench_function("create_pool", |b| {
        b.iter(|| {
            let pool_id = rand::random::<u32>();
            let pool = CudaTensorPool::new(pool_id, 0, CudaTensorPoolConfig::default())
                .expect("Failed to create pool");
            black_box(&pool);
            // Pool is dropped here, cleaning up
        });
    });

    // Create a pool first for opening
    let pool_id = 9999u32;
    let _owner_pool = CudaTensorPool::new(pool_id, 0, CudaTensorPoolConfig::default())
        .expect("Failed to create owner pool");

    // Pool open benchmark (simulates consumer process)
    group.bench_function("open_pool", |b| {
        b.iter(|| {
            let pool = CudaTensorPool::open(pool_id, 0).expect("Failed to open pool");
            black_box(&pool);
        });
    });

    group.finish();
}

/// Benchmark multiple concurrent allocations
fn bench_concurrent_alloc(c: &mut Criterion) {
    if !cuda_available() {
        return;
    }

    let mut group = c.benchmark_group("cuda_concurrent");
    group.measurement_time(Duration::from_secs(5));

    let batch_sizes = vec![1, 4, 8, 16, 32];

    for batch_size in batch_sizes {
        group.bench_with_input(
            BenchmarkId::new("alloc_batch", batch_size),
            &batch_size,
            |b, &batch_size| {
                let pool =
                    CudaTensorPool::new(rand::random::<u32>(), 0, CudaTensorPoolConfig::default())
                        .expect("Failed to create pool");

                b.iter(|| {
                    // Allocate batch
                    let tensors: Vec<_> = (0..batch_size)
                        .map(|_| {
                            pool.alloc(&[640, 480, 3], TensorDtype::F32)
                                .expect("Failed to allocate")
                        })
                        .collect();

                    black_box(&tensors);

                    // Release all
                    for tensor in &tensors {
                        pool.release(tensor).expect("Failed to release");
                    }
                });
            },
        );
    }

    group.finish();
}

/// Compare GPU tensor IPC handle transfer vs simulated CPU memcpy
fn bench_ipc_vs_memcpy(c: &mut Criterion) {
    if !cuda_available() {
        return;
    }

    let mut group = c.benchmark_group("ipc_vs_memcpy");
    group.measurement_time(Duration::from_secs(5));

    // 1080p RGB f32 tensor
    let tensor_size = 1920 * 1080 * 3 * 4; // ~24MB
    group.throughput(Throughput::Bytes(tensor_size as u64));

    // IPC handle transfer (64 bytes regardless of tensor size)
    let pool = CudaTensorPool::new(rand::random::<u32>(), 0, CudaTensorPoolConfig::default())
        .expect("Failed to create pool");

    let tensor = pool
        .alloc(&[1920, 1080, 3], TensorDtype::F32)
        .expect("Failed to allocate");

    group.bench_function("ipc_handle_transfer_1080p", |b| {
        b.iter(|| {
            // In real use, this 64-byte handle would be sent to another process
            let handle = tensor.ipc_handle_bytes();
            // Simulate copying handle through shared memory (64 bytes)
            let mut dest = [0u8; 64];
            dest.copy_from_slice(handle);
            black_box(dest);
        });
    });

    // Simulated CPU memcpy for comparison (what you'd need without IPC)
    let cpu_buffer = vec![0u8; tensor_size];
    let mut dest_buffer = vec![0u8; tensor_size];

    group.bench_function("simulated_memcpy_1080p", |b| {
        b.iter(|| {
            dest_buffer.copy_from_slice(&cpu_buffer);
            black_box(&dest_buffer);
        });
    });

    pool.release(&tensor).expect("Failed to release");
    group.finish();
}

/// Benchmark release performance
fn bench_release(c: &mut Criterion) {
    if !cuda_available() {
        return;
    }

    let mut group = c.benchmark_group("cuda_release");
    group.measurement_time(Duration::from_secs(3));

    let pool = CudaTensorPool::new(rand::random::<u32>(), 0, CudaTensorPoolConfig::default())
        .expect("Failed to create pool");

    // Simple alloc+release cycle benchmark
    group.bench_function("alloc_release_vga", |b| {
        b.iter(|| {
            let tensor = pool
                .alloc(&[640, 480, 3], TensorDtype::F32)
                .expect("Failed to allocate");
            black_box(&tensor);
            pool.release(&tensor).expect("Failed to release");
        });
    });

    group.finish();
}

// Conditional criterion group based on CUDA availability
criterion_group!(
    cuda_benches,
    bench_cuda_alloc,
    bench_ipc_handle_generation,
    bench_pool_operations,
    bench_concurrent_alloc,
    bench_ipc_vs_memcpy,
    bench_release,
);

criterion_main!(cuda_benches);
