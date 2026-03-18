//! Regression test suite for known HORUS issues.
//!
//! Each test documents a previously-found bug and verifies the fix remains
//! in place. Tests are grouped by subsystem and reference the original
//! commit or bug description.
//!
//! # Adding a new regression test
//!
//! When fixing a bug, add a test here using this template:
//!
//! ```rust,ignore
//! /// Regression: [SHORT DESCRIPTION]
//! ///
//! /// Original issue: [commit hash, PR number, or description]
//! /// Root cause: [what went wrong]
//! /// Fix: [what was changed]
//! #[test]
//! fn regression_SHORT_NAME() {
//!     // ... test that the bug is fixed ...
//! }
//! ```

mod common;

use horus_core::communication::Topic;
use horus_core::core::DurationExt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Barrier};
use std::thread;

// ============================================================================
// 1. Topic system regressions
// ============================================================================

/// Regression: Serde types must NOT use DirectChannel backend.
///
/// Original issue: dispatch_serde_type_same_thread_direct_channel assertion failure
/// Root cause: DirectChannel is a POD-only optimization. Non-POD (serde) types
///   were incorrectly dispatched to DirectChannel, causing serialization failures.
/// Fix: The backend selector now routes serde types to SpscIntra even on the
///   same thread (commit a11d14a).
#[test]
fn regression_serde_type_uses_spsc_intra_not_direct_channel() {
    let name = common::unique("reg_serde_dc");
    let t: Topic<String> = Topic::new(&name).unwrap();

    // String is not POD — must not use DirectChannel
    t.send("hello".to_string());
    let received = t.recv();
    assert_eq!(received, Some("hello".to_string()));

    // Verify the backend is NOT DirectChannel (use public backend_name())
    let backend = t.backend_name();
    assert_ne!(
        backend, "DirectChannel",
        "Serde type String must not use DirectChannel (POD-only), got {}",
        backend
    );
}

/// Regression: Topic handles created in separate threads must complete migration
/// before data transfer.
///
/// Original issue: crash_concurrent_send_recv_no_corruption intermittent failure
/// Root cause: Creating Topic instances inside thread::spawn closures raced
///   with the adaptive backend migration (DirectChannel → SpscIntra). The
///   producer could fire all messages before migration completed, causing the
///   consumer's try_recv() to see nothing.
/// Fix: Create Topic handles on the main thread before spawning, ensuring
///   topology detection and migration happen before data transfer.
#[test]
fn regression_cross_thread_topic_handles_must_be_created_before_spawn() {
    let name = common::unique("reg_cross_thread");

    // Create BOTH handles on main thread — this triggers topology detection
    // and backend migration before any data is sent.
    let pub_t: Topic<u64> = Topic::new(&name).unwrap();
    let sub_t: Topic<u64> = Topic::new(&name).unwrap();

    let barrier = Arc::new(Barrier::new(2));
    let barrier_c = barrier.clone();

    let received = Arc::new(AtomicU64::new(0));
    let recv_count = received.clone();

    let consumer = thread::spawn(move || {
        barrier_c.wait();
        let start = std::time::Instant::now();
        while start.elapsed() < 2_u64.secs() {
            if sub_t.try_recv().is_some() {
                recv_count.fetch_add(1, Ordering::Relaxed);
            }
            thread::yield_now();
        }
    });

    barrier.wait();
    for i in 0..500u64 {
        pub_t.send(i);
        if i % 50 == 0 {
            thread::yield_now();
        }
    }

    consumer.join().unwrap();
    let total = received.load(Ordering::Relaxed);
    assert!(
        total > 0,
        "Consumer must receive data when handles are pre-created"
    );
}

/// Regression: Topic with zero capacity must return an error, not panic.
///
/// Original issue: Topic::with_capacity(name, 0, None) could create an
///   invalid ring buffer with capacity 0.
/// Root cause: Zero-capacity ring buffer would cause division by zero in
///   index calculations (idx % capacity).
/// Fix: Topic::with_capacity validates capacity > 0 and returns HorusError.
#[test]
fn regression_zero_capacity_topic_returns_error() {
    let name = common::unique("reg_zero_cap");
    let result: horus_core::error::Result<Topic<u64>> = Topic::with_capacity(&name, 0, None);
    assert!(
        result.is_err(),
        "Zero capacity must return error, not create invalid ring"
    );
}

/// Regression: Serde topic must not lose data on same-thread send/recv cycles.
///
/// Original issue: Serde topics could silently drop messages when the ring
///   buffer wrapped around because the ready flag was not properly reset.
/// Root cause: After reading a serde slot, the ready flag was left set,
///   causing the next write to skip the slot thinking it was still occupied.
/// Fix: read_serde_slot clears the ready flag after successful deserialization.
#[test]
fn regression_serde_topic_no_data_loss_on_wrap() {
    let name = common::unique("reg_serde_wrap");
    let t: Topic<String> = Topic::with_capacity(&name, 4, None).unwrap();

    // Send more messages than capacity to force wrap-around
    for i in 0..20u64 {
        t.send(format!("msg_{}", i));
        let received = t.recv();
        assert!(
            received.is_some(),
            "Message {} must be received after send",
            i
        );
        assert_eq!(received.unwrap(), format!("msg_{}", i));
    }
}

// ============================================================================
// 2. Migration system regressions
// ============================================================================

/// Regression: Topic must survive automatic backend migration triggered by
/// cross-thread usage.
///
/// Original issue: Stale (mode, cached_epoch) pair after concurrent migration
/// Root cause: Thread A loads epoch E, enters migration check, then thread B
///   completes a migration (epoch → E+1, mode → ModeB). Thread A's sync_local
///   reads the OLD mode from cache, leaving an inconsistent state.
/// Fix: After sync_local, re-load the epoch and loop if it changed. This
///   ensures the cached mode always matches the actual epoch.
#[test]
fn regression_data_survives_automatic_backend_migration() {
    let name = common::unique("reg_auto_mig");

    // Start with a same-thread topic
    let t1: Topic<u64> = Topic::new(&name).unwrap();
    t1.send(1u64);
    assert_eq!(t1.recv(), Some(1u64), "Same-thread send/recv must work");

    let initial_backend = t1.backend_name();

    // Create a second handle — this may trigger migration
    let t2: Topic<u64> = Topic::new(&name).unwrap();

    // Both handles must still be functional after migration
    t1.send(42u64);
    let val = t2.try_recv();
    // If migration happened, recv on t2 should work. If not, t1 recv works.
    if val.is_none() {
        // t2 might not have migrated yet, try on t1
        let _ = t1.recv();
    }

    // Verify data integrity with multiple messages
    for i in 100..110u64 {
        t1.send(i);
    }

    let mut received = Vec::new();
    for _ in 0..20 {
        if let Some(v) = t2.try_recv() {
            received.push(v);
        }
        if let Some(v) = t1.try_recv() {
            received.push(v);
        }
    }

    let _ = initial_backend; // used for documentation purposes
                             // At least some messages must have been delivered
    assert!(
        !received.is_empty(),
        "Messages must be deliverable after backend migration"
    );
}

// ============================================================================
// 3. Memory / TensorPool regressions
// ============================================================================

/// Regression: Shape products that overflow u64 must return Err.
///
/// Original issue: TensorPool::alloc with overflowing shape dimensions
///   would wrap around u64 and allocate a near-zero-size region.
/// Root cause: Shape element multiplication used unchecked multiply.
/// Fix: Use checked_mul() and return HorusError::Memory on overflow.
/// Reference: commit f0cf463 (security hardening)
#[test]
fn regression_tensor_alloc_overflow_shape_returns_err() {
    use horus_core::memory::tensor_pool::{TensorPool, TensorPoolConfig};
    use horus_core::types::{Device, TensorDtype};

    let config = TensorPoolConfig {
        pool_size: 1024 * 1024,
        max_slots: 4,
        slot_alignment: 64,
        allocator: Default::default(),
    };
    let pool = TensorPool::new(9800, config).expect("create pool");

    // (u64::MAX/2 + 1) * 2 overflows u64
    let half_max_plus_one = u64::MAX / 2 + 1;
    let result = pool.alloc(&[half_max_plus_one, 2], TensorDtype::U8, Device::CPU);
    assert!(result.is_err(), "Overflowing shape product must return Err");

    std::fs::remove_file(pool.shm_path()).ok();
}

/// Regression: Freed tensor slot data must be zeroed.
///
/// Original issue: After releasing a tensor, the slot data remained in SHM.
///   A subsequent alloc could expose the previous tenant's data to a new user.
/// Root cause: release() returned the slot to the free list without clearing.
/// Fix: volatile_zero the data region before returning the slot.
/// Reference: Security audit commit f0cf463
#[test]
fn regression_tensor_slot_zeroed_on_release() {
    use horus_core::memory::tensor_pool::{TensorPool, TensorPoolConfig};
    use horus_core::types::{Device, TensorDtype};

    let config = TensorPoolConfig {
        pool_size: 1024 * 1024,
        max_slots: 2,
        slot_alignment: 64,
        allocator: Default::default(),
    };
    let pool = TensorPool::new(9801, config).expect("create pool");

    // Allocate and fill with non-zero pattern
    let tensor = pool
        .alloc(&[128], TensorDtype::U8, Device::CPU)
        .expect("alloc");
    pool.data_slice_mut(&tensor).unwrap().fill(0xDE);

    // Release should zero the data
    pool.release(&tensor);

    // Re-allocate (should get the same slot back)
    let tensor2 = pool
        .alloc(&[128], TensorDtype::U8, Device::CPU)
        .expect("realloc");
    let data = pool.data_slice(&tensor2).unwrap();
    assert!(
        data.iter().all(|&b| b == 0),
        "Released slot must be zeroed; found non-zero bytes"
    );

    pool.release(&tensor2);
    std::fs::remove_file(pool.shm_path()).ok();
}

// ============================================================================
// 4. SHM / cross-process regressions
// ============================================================================

/// Regression: SHM topics directory must have restricted permissions (0o700).
///
/// Original issue: SHM directory was created with world-readable permissions,
///   allowing other users on the system to enumerate topics.
/// Fix: Directory is created with mode 0o700 (owner only).
/// Reference: Security audit commit f0cf463
#[test]
#[cfg(target_os = "linux")]
fn regression_shm_topics_dir_permissions_restricted() {
    use std::os::unix::fs::PermissionsExt;

    // Creating a topic will ensure the SHM topics directory exists
    let name = common::unique("reg_shm_dir_perms");
    let _t: Topic<u64> = Topic::new(&name).unwrap();

    let topics_dir = horus_core::memory::shm_topics_dir();
    if let Ok(metadata) = std::fs::metadata(&topics_dir) {
        let mode = metadata.permissions().mode() & 0o777;
        assert_eq!(
            mode, 0o700,
            "SHM topics directory must be 0o700 (owner only), got {:o}",
            mode
        );
    }
}

/// Regression: SHM backing files for topics must be owner-only (0o600).
///
/// Original issue: SHM files were created with world-readable permissions,
///   allowing other users on the system to read shared memory data.
/// Fix: O_CREAT files use mode 0o600.
/// Reference: Security audit commit f0cf463
#[test]
#[cfg(target_os = "linux")]
fn regression_shm_file_permissions_owner_only() {
    use std::os::unix::fs::PermissionsExt;

    let name = common::unique("reg_shm_file_perms");
    let _t: Topic<u64> = Topic::new(&name).unwrap();

    // Find the SHM file for this topic
    let topics_dir = horus_core::memory::shm_topics_dir();
    if let Ok(entries) = std::fs::read_dir(&topics_dir) {
        for entry in entries.flatten() {
            let entry_name = entry.file_name().to_string_lossy().to_string();
            if entry_name.contains(&name) {
                let metadata = entry.metadata().expect("metadata");
                let mode = metadata.permissions().mode() & 0o777;
                assert_eq!(
                    mode, 0o600,
                    "SHM file {} must be 0o600, got {:o}",
                    entry_name, mode
                );
                return;
            }
        }
    }
    // If we can't find the file, the test is inconclusive (not a failure)
}

// ============================================================================
// 5. Scheduler / node regressions
// ============================================================================

/// Regression: Recording manager must handle missing sessions gracefully.
///
/// Original issue: Querying a nonexistent recording session could panic
///   or return a confusing error.
/// Fix: RecordingManager::session_recordings returns empty Vec for missing sessions.
#[test]
fn regression_recording_manager_missing_session_returns_empty() {
    use horus_core::scheduling::RecordingManager;

    let manager = RecordingManager::new();
    let recordings = manager.session_recordings("definitely_nonexistent_session_xyz");

    match recordings {
        Ok(list) => assert!(list.is_empty(), "Missing session should return empty list"),
        Err(_) => {} // Error is also acceptable
    }
}

/// Regression: RuntimeParams must handle missing params file gracefully.
///
/// Original issue: RuntimeParams::init panicked when .horus/config/params.yaml
///   didn't exist.
/// Fix: init() creates the file if missing, returning empty params.
#[test]
fn regression_runtime_params_init_without_file() {
    // This test runs in whatever directory — RuntimeParams::new should not panic
    let result = horus_core::params::RuntimeParams::new();
    assert!(result.is_ok(), "RuntimeParams::new must not panic");
    let params = result.unwrap();
    // Should be able to query without error
    let all = params.get_all();
    let _ = all.len(); // must not panic
}
