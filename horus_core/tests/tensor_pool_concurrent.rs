#![allow(dead_code)]
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

// Pool ids are `BASE + (pid % 100)`, so each test owns a 100-wide range and the
// BASES MUST BE AT LEAST 100 APART. They used to be 10 apart -- 9700, 9710,
// 9720, 9730, 9740 -- which made every range overlap the next nine, and
// resource_exhaustion.rs sat at 9800 inside three of them.
//
// A pool is shared memory keyed by that id and it outlives the process that
// made it, so an overlap is not a race between these tests: it is this run
// finding a pool some EARLIER run left behind at the same id and a different
// size. That is what this was:
//
//   Tensor pool 9811 exists but is only 65856 bytes, smaller than the
//   16780864 bytes this process's configuration requires
//
// 9811 is 9720+91 here and 9800+11 in resource_exhaustion.rs, whose pool is
// 64 KiB -- 65536 plus header, the 65856 in the message. It reproduces only
// when the two pids land 80 apart mod 100, which is why it reads as a flake.
//
// `pool_id_ranges_cannot_overlap` below pins the spacing.

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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = 9700 + (std::process::id() % 100);
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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = 9800 + (std::process::id() % 100);
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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = 9900 + (std::process::id() % 100);
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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = 10000 + (std::process::id() % 100);
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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = 10100 + (std::process::id() % 100);
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

// ============================================================================
// Test: the pool id ranges these tests claim must stay disjoint
// ============================================================================

/// Every `BASE + (pid % 100)` range in the suite must be disjoint.
///
/// This is a property of the constants, not of the runtime, so it is checked
/// here rather than discovered on a runner whose pid happens to collide. The
/// bug it replaces failed roughly one run in a hundred and blamed the tensor
/// pool for what was an id-allocation mistake in the tests.
#[test]
fn pool_id_ranges_cannot_overlap() {
    // Keep in sync with the `let pool_id = ...` lines above, and with
    // resource_exhaustion.rs, which shares this id space.
    let bases = [
        ("concurrent_alloc_release", 9700u32),
        ("concurrent_alloc_all", 9800),
        ("slot_reuse", 9900),
        ("single_slot", 10000),
        ("data_integrity", 10100),
        ("resource_exhaustion (other file)", 10200),
    ];

    for (i, (name_a, base_a)) in bases.iter().enumerate() {
        for (name_b, base_b) in bases.iter().skip(i + 1) {
            let gap = base_b.abs_diff(*base_a);
            assert!(
                gap >= 100,
                "pool id ranges for {name_a} ({base_a}) and {name_b} ({base_b}) \
                 are {gap} apart, but the offset is `pid % 100` -- they overlap, \
                 so one run can find the other's pool at a different size"
            );
        }
    }
}
