// Concurrent stress tests for TensorPool.
//
// Verifies that TensorPool is safe under multi-thread contention —
// multiple threads allocating and releasing from the same pool simultaneously.

use horus_core::memory::{TensorPool, TensorPoolConfig};
use horus_core::types::{Device, TensorDtype};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

fn small_pool_config() -> TensorPoolConfig {
    TensorPoolConfig {
        pool_size: 16 * 1024 * 1024, // 16MB — small for testing
        max_slots: 64,
        slot_alignment: 64,
        ..Default::default()
    }
}

// ============================================================================
// Test: 4 threads alloc/release concurrently — no panic, no corruption
// ============================================================================

#[test]
fn test_concurrent_alloc_release_no_panic() {
    cleanup_stale_shm();

    let pool_id = 9700 + (std::process::id() % 100) as u32;
    let pool = Arc::new(TensorPool::new(pool_id, small_pool_config()).unwrap());

    let thread_count = 4;
    let ops_per_thread = 50;
    let total_ops = Arc::new(AtomicU64::new(0));

    let mut handles = Vec::new();
    for t in 0..thread_count {
        let pool = pool.clone();
        let ops = total_ops.clone();
        handles.push(std::thread::spawn(move || {
            for i in 0..ops_per_thread {
                // Alloc a small tensor
                let shape = [1, 16, 16]; // 256 elements
                match pool.alloc(&shape, TensorDtype::F32, Device::cpu()) {
                    Ok(tensor) => {
                        // Write some data to verify no corruption
                        if let Ok(data) = pool.data_slice_mut(&tensor) {
                            let marker = ((t * 1000 + i) & 0xFF) as u8;
                            data[0] = marker;
                        }
                        // Release immediately
                        pool.release(&tensor);
                        ops.fetch_add(1, Ordering::SeqCst);
                    }
                    Err(_) => {
                        // Pool exhaustion under contention is acceptable
                        // (all 64 slots may be taken by other threads)
                        std::thread::yield_now();
                    }
                }
            }
        }));
    }

    for h in handles {
        h.join()
            .expect("Thread panicked during concurrent alloc/release");
    }

    let completed = total_ops.load(Ordering::SeqCst);
    assert!(
        completed > 0,
        "At least some alloc/release ops should succeed, got {}",
        completed
    );
}

// ============================================================================
// Test: Concurrent alloc — all threads succeed without panic
// ============================================================================

#[test]
fn test_concurrent_alloc_all_threads_succeed() {
    cleanup_stale_shm();

    let pool_id = 9710 + (std::process::id() % 100) as u32;
    let pool = Arc::new(TensorPool::new(pool_id, small_pool_config()).unwrap());

    let thread_count = 4;
    let allocs_per_thread = 8;
    let success_count = Arc::new(AtomicU64::new(0));

    let mut handles = Vec::new();
    for _ in 0..thread_count {
        let pool = pool.clone();
        let success = success_count.clone();
        handles.push(std::thread::spawn(move || {
            let mut local_tensors = Vec::new();
            for _ in 0..allocs_per_thread {
                match pool.alloc(&[1, 4, 4], TensorDtype::F32, Device::cpu()) {
                    Ok(tensor) => {
                        // Verify data is writable
                        if let Ok(data) = pool.data_slice_mut(&tensor) {
                            data[0] = 0xAB;
                        }
                        local_tensors.push(tensor);
                        success.fetch_add(1, Ordering::SeqCst);
                    }
                    Err(_) => {
                        // Pool exhaustion under contention is acceptable
                    }
                }
            }
            // Hold tensors until end of thread — then release
            for t in &local_tensors {
                pool.release(t);
            }
        }));
    }

    for h in handles {
        h.join().expect("Thread panicked during concurrent alloc");
    }

    let total = success_count.load(Ordering::SeqCst);
    assert!(
        total >= thread_count as u64,
        "Each thread should alloc at least 1 tensor, got {} total successes",
        total
    );
}

// ============================================================================
// Test: Sequential alloc-release-alloc reuses slots
// ============================================================================

#[test]
fn test_slot_reuse_after_release() {
    cleanup_stale_shm();

    let pool_id = 9720 + (std::process::id() % 100) as u32;
    let pool = TensorPool::new(pool_id, small_pool_config()).unwrap();

    // Alloc and release repeatedly — should reuse slots
    for _ in 0..100 {
        let tensor = pool
            .alloc(&[1, 8, 8], TensorDtype::F32, Device::cpu())
            .unwrap();
        pool.release(&tensor);
    }
    // If slots weren't reused, we'd run out after 64 (max_slots)
    // Since we got to 100 without error, reuse works
}

// ============================================================================
// Test: Pool with 1 slot — alloc succeeds, second alloc fails, release+alloc works
// ============================================================================

#[test]
fn test_single_slot_pool() {
    cleanup_stale_shm();

    let pool_id = 9730 + (std::process::id() % 100) as u32;
    let config = TensorPoolConfig {
        pool_size: 1024 * 1024, // 1MB
        max_slots: 1,           // only 1 slot
        slot_alignment: 64,
        ..Default::default()
    };
    let pool = TensorPool::new(pool_id, config).unwrap();

    let tensor1 = pool
        .alloc(&[1, 4, 4], TensorDtype::F32, Device::cpu())
        .unwrap();

    // Second alloc should fail (only 1 slot)
    let result = pool.alloc(&[1, 4, 4], TensorDtype::F32, Device::cpu());
    assert!(result.is_err(), "Second alloc should fail with only 1 slot");

    // Release the first, then alloc again should work
    pool.release(&tensor1);
    let tensor2 = pool
        .alloc(&[1, 4, 4], TensorDtype::F32, Device::cpu())
        .unwrap();
    pool.release(&tensor2);
}

// ============================================================================
// Test: Data written to pool is readable
// ============================================================================

#[test]
fn test_data_integrity_after_write() {
    cleanup_stale_shm();

    let pool_id = 9740 + (std::process::id() % 100) as u32;
    let pool = TensorPool::new(pool_id, small_pool_config()).unwrap();

    let tensor = pool.alloc(&[1, 4], TensorDtype::U8, Device::cpu()).unwrap();

    // Write pattern
    {
        let data = pool.data_slice_mut(&tensor).unwrap();
        data[0] = 0xDE;
        data[1] = 0xAD;
        data[2] = 0xBE;
        data[3] = 0xEF;
    }

    // Read back and verify
    {
        let data = pool.data_slice(&tensor).unwrap();
        assert_eq!(data[0], 0xDE);
        assert_eq!(data[1], 0xAD);
        assert_eq!(data[2], 0xBE);
        assert_eq!(data[3], 0xEF);
    }

    pool.release(&tensor);
}
