//! Process-local epoch notification for topic migration.
//!
//! Every `Topic` instance sharing a name within this process shares one atomic
//! epoch counter. A migrating instance bumps it so the others detect the
//! migration with a ~1ns L1 heap read instead of a ~20ns SHM mmap read.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

// ============================================================================
// Process-local epoch notification
// ============================================================================

/// Per-topic-name atomic epoch, shared between all same-name Topic instances
/// in this process. Read on every try_send/try_recv (~1ns L1 heap read)
/// instead of ~20ns SHM mmap read.
static EPOCH_NOTIFY: OnceLock<Mutex<HashMap<String, Arc<AtomicU64>>>> = OnceLock::new();

fn epoch_registry() -> &'static Mutex<HashMap<String, Arc<AtomicU64>>> {
    EPOCH_NOTIFY.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Get or create a process-local epoch atomic for a topic name.
pub(crate) fn get_or_create_process_epoch(name: &str) -> Arc<AtomicU64> {
    let mut map = epoch_registry().lock().unwrap_or_else(|e| e.into_inner());
    map.entry(name.to_string())
        .or_insert_with(|| Arc::new(AtomicU64::new(0)))
        .clone()
}

/// Notify all same-process Topics of an epoch change.
/// Uses try_lock to avoid blocking the hot path — if contended,
/// the periodic SHM check in dispatch functions will catch it.
pub(crate) fn notify_epoch_change(name: &str, new_epoch: u64) {
    if let Ok(map) = epoch_registry().try_lock() {
        if let Some(epoch) = map.get(name) {
            epoch.store(new_epoch, Ordering::Release);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU64 as StdAtomicU64;
    use std::sync::Barrier;

    // Unique topic names to avoid cross-test interference from global statics
    static TEST_COUNTER: StdAtomicU64 = StdAtomicU64::new(0);
    fn unique(prefix: &str) -> String {
        let id = TEST_COUNTER.fetch_add(1, Ordering::Relaxed);
        format!("registry_test_{}_{}_{}", prefix, std::process::id(), id)
    }

    // ── get_or_create_process_epoch ──

    #[test]
    fn test_get_or_create_epoch_initial_zero() {
        let name = unique("epoch_init");
        let epoch = get_or_create_process_epoch(&name);
        assert_eq!(epoch.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_get_or_create_epoch_idempotent() {
        let name = unique("epoch_idem");
        let e1 = get_or_create_process_epoch(&name);
        let e2 = get_or_create_process_epoch(&name);

        // Both should point to the same Arc (same allocation)
        assert!(Arc::ptr_eq(&e1, &e2));
    }

    #[test]
    fn test_get_or_create_epoch_different_topics() {
        let a = unique("epoch_a");
        let b = unique("epoch_b");
        let ea = get_or_create_process_epoch(&a);
        let eb = get_or_create_process_epoch(&b);

        // Different topics get different atomics
        assert!(!Arc::ptr_eq(&ea, &eb));
    }

    // ── notify_epoch_change ──

    #[test]
    fn test_notify_epoch_change_updates_value() {
        let name = unique("notify");
        let epoch = get_or_create_process_epoch(&name);
        assert_eq!(epoch.load(Ordering::Relaxed), 0);

        // notify_epoch_change uses try_lock() to avoid blocking the hot path.
        // Under heavy parallel test load the mutex may be contended, so retry
        // a few times to give the lock a chance to become available.
        for _ in 0..50 {
            notify_epoch_change(&name, 5);
            if epoch.load(Ordering::Acquire) == 5 {
                break;
            }
            std::thread::yield_now();
        }
        assert_eq!(epoch.load(Ordering::Acquire), 5);
    }

    #[test]
    fn test_notify_nonexistent_topic_no_panic() {
        notify_epoch_change(&unique("no_exist"), 42);
        // Should not panic
    }

    // ── Concurrent access tests ──

    #[test]
    fn test_concurrent_epoch_creation() {
        let name = unique("concurrent_epoch");
        let barrier = Arc::new(Barrier::new(8));
        let epochs: Arc<Mutex<Vec<Arc<AtomicU64>>>> = Arc::new(Mutex::new(Vec::new()));

        let handles: Vec<_> = (0..8)
            .map(|_| {
                let name = name.clone();
                let barrier = Arc::clone(&barrier);
                let epochs = Arc::clone(&epochs);
                std::thread::spawn(move || {
                    barrier.wait();
                    let epoch = get_or_create_process_epoch(&name);
                    epochs.lock().unwrap().push(epoch);
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        let epochs = epochs.lock().unwrap();
        // All threads should get the same Arc (same pointer)
        let first = &epochs[0];
        assert!(epochs.iter().all(|e| Arc::ptr_eq(first, e)));
    }

    #[test]
    fn test_concurrent_notify_and_read() {
        let name = unique("concurrent_notify");
        let epoch = get_or_create_process_epoch(&name);
        let barrier = Arc::new(Barrier::new(4));

        // 2 writers, 2 readers
        let mut handles = Vec::new();
        for i in 0..2 {
            let name = name.clone();
            let barrier = Arc::clone(&barrier);
            handles.push(std::thread::spawn(move || {
                barrier.wait();
                for v in 0..100 {
                    notify_epoch_change(&name, (i * 100 + v) as u64);
                }
            }));
        }
        for _ in 0..2 {
            let epoch = Arc::clone(&epoch);
            let barrier = Arc::clone(&barrier);
            handles.push(std::thread::spawn(move || {
                barrier.wait();
                for _ in 0..100 {
                    let _ = epoch.load(Ordering::Acquire);
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        // After all writes, epoch should be one of the written values
        let final_val = epoch.load(Ordering::Acquire);
        assert!(final_val < 200);
    }
}

