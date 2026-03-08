//! Memory-safety regression tests for the HORUS tensor pool and handle API.
//!
//! These tests exercise the critical safety properties fixed during the security
//! hardening audit (HORUS Security & Safety Hardening roadmap):
//!
//! - Slot data is zeroed on release (no cross-tenant data leak).
//! - Stale/double-release on a freed descriptor returns `Err`, not UB.
//! - `alloc_with_timeout` wakes up when a slot becomes available.
//! - Pool exhaustion produces `HorusError::Timeout`, not a hang.
//! - `TensorHandle::from_owned` rejects a pool-ID mismatch.
//!
//! # Running under MIRI
//!
//! ```sh
//! cargo +nightly miri test --package horus_core --test memory_safety
//! ```
//!
//! MIRI intercepts all memory accesses through the file-backed mmap used by
//! `TensorPool` and will catch:
//! - Use-after-free (accessing data from a freed slot without going through
//!   the generation-check path).
//! - Uninitialized memory reads (data that was never written).
//! - Invalid pointer arithmetic in `data_slice()`.
//!
//! Thread-based tests (`alloc_with_timeout_unblocks_on_release`,
//! `pool_exhaustion_triggers_timeout`) are gated with `#[cfg(not(miri))]`
//! because MIRI is several orders of magnitude slower per thread-switch, making
//! timeout-based tests unreliable.  The underlying condvar logic IS tested by
//! the loom tests in `loom_ring_buffers.rs`.

use horus_core::error::HorusError;
use horus_core::memory::tensor_pool::{Device, TensorDtype};
use horus_core::memory::{TensorPool, TensorPoolConfig};
use std::sync::Arc;

// ── helpers ────────────────────────────────────────────────────────────────

/// Unique pool-ID range for this test file.
/// Offset by 50000 to avoid collisions with unit tests in tensor_pool.rs
/// (which use IDs 9990–9999) and other integration test files.
const POOL_BASE: u32 = 50_000;

fn small_pool(pool_id: u32) -> TensorPool {
    TensorPool::new(
        pool_id,
        TensorPoolConfig {
            pool_size: 64 * 1024, // 64 KiB — minimal footprint under MIRI
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        },
    )
    .expect("failed to create TensorPool")
}

fn small_pool_arc(pool_id: u32) -> Arc<TensorPool> {
    Arc::new(small_pool(pool_id))
}

// ── test 1: slot data is zeroed on release ─────────────────────────────────

/// After a tensor slot is released its backing bytes must be zero.
///
/// Before the security fix, `return_slot()` returned the slot to the free list
/// without wiping the data region, allowing the next tenant to read stale
/// (potentially sensitive) bytes.
///
/// This test writes a recognisable pattern, releases the slot, then reads the
/// raw data region through `data_slice()` (which only bounds-checks, not
/// liveness-checks) and asserts every byte is zero.
#[test]
fn slot_data_zeroed_after_release() {
    let pool = small_pool(POOL_BASE);

    let tensor = pool
        .alloc(&[16], TensorDtype::U8, Device::cpu())
        .expect("alloc failed");

    // Write a recognisable sentinel pattern.
    {
        let data = pool.data_slice_mut(&tensor).expect("data_slice_mut failed");
        for b in data.iter_mut() {
            *b = 0xDE;
        }
    }

    // Release: triggers `return_slot` → `volatile_zero`.
    pool.release(&tensor);

    // Read the same memory address via the (now stale) descriptor.
    // `data_slice` uses the recorded offset/size from the descriptor rather
    // than checking the slot-live flag, giving us a direct view of the bytes.
    let scrubbed = pool
        .data_slice(&tensor)
        .expect("data_slice on freed slot failed");
    let non_zero: Vec<usize> = scrubbed
        .iter()
        .enumerate()
        .filter(|(_, &b)| b != 0)
        .map(|(i, _)| i)
        .collect();

    assert!(
        non_zero.is_empty(),
        "slot data must be zeroed after release; non-zero offsets: {:?}",
        non_zero
    );

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 2: double-release on stale descriptor returns Err ─────────────────

/// `try_release()` on a freed (stale) descriptor must return a generation
/// mismatch error, not silently corrupt the refcount of whatever process
/// currently owns that slot.
///
/// This regression test guards against the ABA problem: without the 64-bit
/// generation counter, a second release of a stale descriptor could decrement
/// the refcount of a live tensor that happened to be allocated in the same slot.
#[test]
fn stale_double_release_returns_err() {
    let pool = small_pool(POOL_BASE + 1);

    // First allocation — capture a copy of the descriptor.
    let first = pool
        .alloc(&[8], TensorDtype::U8, Device::cpu())
        .expect("first alloc failed");
    let stale = first; // copy of descriptor (Tensor is Copy)

    // Release — slot is now free; generation is bumped.
    pool.release(&first);

    // Reallocate — same slot is reused with a new, higher generation.
    let _second = pool
        .alloc(&[8], TensorDtype::U8, Device::cpu())
        .expect("second alloc failed");

    // `try_release` on the stale descriptor must fail.
    let result = pool.try_release(&stale);
    assert!(
        result.is_err(),
        "try_release with stale descriptor must return Err; got Ok"
    );
    match result.unwrap_err() {
        HorusError::Memory(ref e) => {
            let msg = e.to_string();
            assert!(
                msg.contains("generation") || msg.contains("Generation"),
                "error must mention generation mismatch: {msg}"
            );
        }
        other => unreachable!("expected HorusError::Memory, got {:?}", other),
    }

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 3: stale retain also detected ─────────────────────────────────────

/// `try_retain()` on a stale descriptor must also return an error.
///
/// This ensures a receiver holding an old descriptor cannot increment the
/// refcount of a slot that has since been recycled to a different tensor.
#[test]
fn stale_retain_returns_err() {
    let pool = small_pool(POOL_BASE + 2);

    let first = pool
        .alloc(&[8], TensorDtype::F32, Device::cpu())
        .expect("alloc failed");
    let stale = first;

    pool.release(&first);

    // Reallocate to bump the generation.
    let _second = pool
        .alloc(&[8], TensorDtype::F32, Device::cpu())
        .expect("second alloc failed");

    let result = pool.try_retain(&stale);
    assert!(
        result.is_err(),
        "try_retain with stale descriptor must return Err; got Ok"
    );

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 4: validate_descriptor rejects freed slot ─────────────────────────

/// `validate_descriptor()` must reject a descriptor for a slot that has been
/// freed, catching tampered or stale descriptors before any data access.
#[test]
fn validate_descriptor_rejects_freed_slot() {
    let pool = small_pool(POOL_BASE + 3);

    let tensor = pool
        .alloc(&[32], TensorDtype::U8, Device::cpu())
        .expect("alloc failed");

    // Release → slot is free but descriptor still refers to it.
    pool.release(&tensor);

    let result = pool.validate_descriptor(&tensor);
    assert!(
        result.is_err(),
        "validate_descriptor must reject a descriptor for a freed slot"
    );

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 5: generation increments monotonically across slot reuse ──────────

/// The 64-bit generation counter must increase strictly on every
/// alloc-release cycle so that old descriptors can never be confused with
/// newly-allocated ones (ABA prevention).
#[test]
fn generation_monotonically_increases_on_reuse() {
    let pool = TensorPool::new(
        POOL_BASE + 4,
        TensorPoolConfig {
            pool_size: 64 * 1024,
            max_slots: 1, // Force the same slot to be reused every cycle
            slot_alignment: 64,
            allocator: Default::default(),
        },
    )
    .expect("failed to create pool");

    let mut last_gen = 0u64;
    for cycle in 0..8u64 {
        let tensor = pool
            .alloc(&[4], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        let gen = tensor.generation_full();
        assert!(
            gen > last_gen,
            "cycle {cycle}: generation_full must be strictly increasing; \
             prev={last_gen}, curr={gen}"
        );
        last_gen = gen;
        pool.release(&tensor);
    }

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 6: TensorHandle refcount is 1 after alloc, 0 after drop ───────────

/// RAII handle must decrement the refcount exactly once on drop.
///
/// This is the basic memory-management contract that the rest of the
/// safety model depends on.  MIRI can track the atomic stores and loads
/// through the mmap and will catch double-decrements.
#[test]
fn tensor_handle_refcount_lifecycle() {
    use horus_core::memory::TensorHandle;

    let pool = small_pool_arc(POOL_BASE + 5);

    let handle = TensorHandle::alloc(pool.clone(), &[10], TensorDtype::U8, Device::cpu())
        .expect("TensorHandle::alloc failed");

    assert_eq!(
        handle.refcount(),
        1,
        "freshly allocated handle must have refcount 1"
    );

    // Clone: refcount goes to 2.
    let handle2 = handle.clone();
    assert_eq!(handle.refcount(), 2, "after clone refcount must be 2");

    // Drop first clone: back to 1.
    drop(handle2);
    assert_eq!(
        handle.refcount(),
        1,
        "after dropping clone refcount must be 1"
    );

    // Drop the original: slot returned to free list (refcount becomes 0 internally).
    let tensor_desc = *handle.tensor(); // copy descriptor before drop
    drop(handle);

    // Slot is now free; refcount must read as 0 (generation mismatch path returns 0).
    assert_eq!(
        pool.refcount(&tensor_desc),
        0,
        "refcount must be 0 after last handle is dropped"
    );

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 7: pool exhaustion with short timeout → HorusError::Timeout ───────
//
// Heavy thread tests are skipped under MIRI: they would take hours because
// MIRI serialises thread operations.  The condvar wakeup logic itself is
// exercised by the alloc_with_timeout_unblocks_on_release test below.

/// Exhausting all pool slots and calling `alloc_with_timeout` with a very
/// short deadline must return `HorusError::Timeout`, not block forever.
#[test]
#[cfg(not(miri))] // MIRI: thread timing unreliable; logic tested separately
fn pool_exhaustion_triggers_timeout() {
    let pool = Arc::new(
        TensorPool::new(
            POOL_BASE + 6,
            TensorPoolConfig {
                pool_size: 64 * 1024,
                max_slots: 2, // Only 2 slots: easy to exhaust
                slot_alignment: 64,
                allocator: Default::default(),
            },
        )
        .expect("failed to create pool"),
    );

    // Exhaust all slots.
    let _t1 = pool
        .alloc(&[4], TensorDtype::U8, Device::cpu())
        .expect("first alloc failed");
    let _t2 = pool
        .alloc(&[4], TensorDtype::U8, Device::cpu())
        .expect("second alloc failed");

    // Next alloc must time out.
    let result = pool.alloc_with_timeout(
        &[4],
        TensorDtype::U8,
        Device::cpu(),
        std::time::Duration::from_millis(50),
    );

    assert!(
        matches!(result, Err(HorusError::Timeout(_))),
        "exhausted pool alloc_with_timeout must return Err(Timeout); got {:?}",
        result
    );

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 8: alloc_with_timeout unblocks when a slot is released ─────────────

/// `alloc_with_timeout` must wake up and succeed when another thread releases
/// a slot within the timeout window.
///
/// This tests the condvar-based wakeup path in `alloc_with_timeout()`:
/// `return_slot()` must call `notify_all()` so the waiting thread is
/// unblocked rather than sleeping the full timeout duration.
#[test]
#[cfg(not(miri))] // MIRI: thread timing unreliable
fn alloc_with_timeout_unblocks_on_release() {
    let pool = Arc::new(
        TensorPool::new(
            POOL_BASE + 7,
            TensorPoolConfig {
                pool_size: 64 * 1024,
                max_slots: 1, // Single slot: easiest to exhaust/release
                slot_alignment: 64,
                allocator: Default::default(),
            },
        )
        .expect("failed to create pool"),
    );

    // Occupy the only slot.
    let tensor = pool
        .alloc(&[4], TensorDtype::U8, Device::cpu())
        .expect("initial alloc failed");

    let pool_for_thread = pool.clone();
    let timeout = std::time::Duration::from_millis(500);

    // Spawn a thread that releases the slot after a short delay.
    let release_thread = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(50));
        pool_for_thread.release(&tensor);
    });

    // Main thread waits for a slot — should wake within ~50ms.
    let t_start = std::time::Instant::now();
    let result = pool.alloc_with_timeout(&[4], TensorDtype::U8, Device::cpu(), timeout);
    let elapsed = t_start.elapsed();

    release_thread.join().expect("release thread panicked");

    assert!(
        result.is_ok(),
        "alloc_with_timeout must succeed after slot is released; got {:?}",
        result
    );
    assert!(
        elapsed < timeout,
        "alloc_with_timeout must unblock before the full timeout; elapsed={:?}",
        elapsed
    );

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── test 9: data_slice bounds check prevents out-of-bounds access ───────────

/// `data_slice` must return `Err` for a descriptor whose size/offset would
/// extend beyond the pool data region, not silently expose out-of-bounds memory.
#[test]
fn data_slice_rejects_out_of_bounds_descriptor() {
    use horus_core::types::Tensor;

    let pool = small_pool(POOL_BASE + 8);

    // Allocate a real tensor, then manufacture a descriptor with a
    // grossly out-of-bounds offset/size to test the bounds check.
    let _real = pool
        .alloc(&[4], TensorDtype::U8, Device::cpu())
        .expect("alloc failed");

    // Tensor: pool_id, slot_id, generation_hi, generation_lo,
    // offset, size, shape, dtype, device
    let bogus = Tensor::new(
        pool.pool_id(),
        0,
        1, // generation_full = 1 (matches slot 0 after first alloc)
        // offset so large that offset + size overflows the pool data region
        u64::MAX - 4,
        &[4],
        TensorDtype::U8,
        Device::cpu(),
    );

    let result = pool.data_slice(&bogus);
    assert!(
        result.is_err(),
        "data_slice with out-of-bounds descriptor must return Err; got Ok"
    );

    std::fs::remove_file(pool.shm_path()).ok();
}

// ── clean up helper ─────────────────────────────────────────────────────────
//
// Each test removes its own SHM file via `pool.shm_path()`.
// TensorPool exposes shm_path only in its public surface; see shm_path() below.

// We need `pool.shm_path()` to clean up test artefacts.
// Since `shm_path` is a private field, we use the public `pool_id()` to
// reconstruct the path.  This mirrors what the test-cleanup helpers in
// tensor_pool.rs unit tests do.
#[allow(dead_code)]
trait ShmPath {
    fn shm_path(&self) -> std::path::PathBuf;
}

impl ShmPath for TensorPool {
    fn shm_path(&self) -> std::path::PathBuf {
        horus_core::memory::shm_base_dir()
            .join("tensors")
            .join(format!("tensor_pool_{}", self.pool_id()))
    }
}
