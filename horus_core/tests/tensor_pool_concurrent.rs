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
use common::{
    cleanup_stale_shm, test_pool_id, POOL_BAND_END, POOL_BAND_START,
    POOL_BASE_CONCURRENT_ALLOC_ALL, POOL_BASE_CONCURRENT_ALLOC_RELEASE, POOL_BASE_DATA_INTEGRITY,
    POOL_BASE_SINGLE_SLOT, POOL_BASE_SLOT_REUSE, POOL_PID_SPREAD, TENSOR_POOL_BASES,
};

// Pool ids here are `BASE + (pid % POOL_PID_SPREAD)`, so each test owns a
// range `POOL_PID_SPREAD` wide and the BASES MUST BE AT LEAST THAT FAR APART.
// They used to be 10 apart -- 9700, 9710, 9720, 9730, 9740 -- which made every
// range overlap the next nine, and resource_exhaustion.rs sat at 9800 inside
// three of them.
//
// A pool is shared memory keyed by that id and it outlives the process that
// made it, so an overlap is not a race between these tests: it is this run
// finding a pool some EARLIER run left behind at the same id and a different
// size. That is what this was:
//
//   Tensor pool 9811 exists but is only 65856 bytes, smaller than the
//   16780864 bytes this process's configuration requires
//
// 9811 was 9720+91 here and 9800+11 in resource_exhaustion.rs, whose pool is
// 64 KiB -- 65536 plus header, the 65856 in the message. It reproduces only
// when the two pids land 80 apart mod 100, which is why it reads as a flake.
//
// Spacing the bases is necessary but not sufficient: a FIXED id sitting inside
// one of these ranges collides whenever `pid % POOL_PID_SPREAD` selects it, and
// the 9500-10200 neighbourhood is full of them (tensor_pool.rs's unit tests,
// regressions.rs, the benchmarks). So the whole family moved to its own band at
// 20_000. The bases live in tests/common so this file and resource_exhaustion.rs
// compile against the same values, and `pool_id_ranges_cannot_overlap` below
// checks the constants themselves rather than a copy of them.

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

    let pool_id = test_pool_id(POOL_BASE_CONCURRENT_ALLOC_RELEASE);
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

    let pool_id = test_pool_id(POOL_BASE_CONCURRENT_ALLOC_ALL);
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

    let pool_id = test_pool_id(POOL_BASE_SLOT_REUSE);
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

    let pool_id = test_pool_id(POOL_BASE_SINGLE_SLOT);
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

    let pool_id = test_pool_id(POOL_BASE_DATA_INTEGRITY);
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

/// Every `BASE + (pid % POOL_PID_SPREAD)` range in the family must be disjoint,
/// and the family must stay inside the band reserved for it.
///
/// This is a property of the constants, not of the runtime, so it is checked
/// here rather than discovered on a runner whose pid happens to collide. The
/// bug it replaces failed roughly one run in a hundred and blamed the tensor
/// pool for what was an id-allocation mistake in the tests.
///
/// It reads [`TENSOR_POOL_BASES`], which is built from the same constants the
/// `test_pool_id(..)` call sites use, so this cannot pass while disagreeing
/// with the ids the tests actually allocate.
#[test]
fn pool_id_ranges_cannot_overlap() {
    // Pairwise: no two ranges may touch.
    for (i, (name_a, base_a)) in TENSOR_POOL_BASES.iter().enumerate() {
        for (name_b, base_b) in TENSOR_POOL_BASES.iter().skip(i + 1) {
            let gap = base_b.abs_diff(*base_a);
            assert!(
                gap >= POOL_PID_SPREAD,
                "pool id ranges for {name_a} ({base_a}) and {name_b} ({base_b}) \
                 are {gap} apart, but the offset is `pid % {POOL_PID_SPREAD}` -- \
                 they overlap, so one run can find the other's pool at a \
                 different size"
            );
        }
    }

    // Containment: the band is what makes "no fixed id collides with us" a
    // claim about one range instead of about every literal in the workspace.
    for (name, base) in TENSOR_POOL_BASES.iter() {
        assert!(
            *base >= POOL_BAND_START && base + POOL_PID_SPREAD <= POOL_BAND_END,
            "pool id range for {name} ({base}..{}) escapes the reserved band \
             {POOL_BAND_START}..{POOL_BAND_END}; ids outside it are not known \
             to be free",
            base + POOL_PID_SPREAD
        );
    }

    // Exact fit: a base added without widening the band (or a band widened
    // without a base) is a gap nobody owns, which is how the next fixed id
    // gets parked inside this family. Fail on both.
    assert_eq!(
        TENSOR_POOL_BASES.len() as u32 * POOL_PID_SPREAD,
        POOL_BAND_END - POOL_BAND_START,
        "{} bases of width {POOL_PID_SPREAD} do not exactly fill the reserved \
         band {POOL_BAND_START}..{POOL_BAND_END}; add the missing base or \
         adjust POOL_BAND_END",
        TENSOR_POOL_BASES.len()
    );
}
