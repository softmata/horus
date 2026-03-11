//! Global backend registry for intra-process migration.
//!
//! When multiple `Topic` instances on different threads share the same
//! topic name (intra-process), they need to share the same heap-backed backend
//! (e.g., the same `Arc<SpscRing<T>>`). This registry provides that shared
//! lookup, keyed by (topic_name, epoch).

use std::any::Any;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

type BackendMap = HashMap<(String, u64), Arc<dyn Any + Send + Sync>>;

/// Global registry mapping (topic_name, epoch) → Arc<dyn Any>.
///
/// Each entry is a type-erased Arc to the backend (e.g., `Arc<SpscRing<T>>`).
/// When a participant detects an epoch change, it looks up the new backend here.
static REGISTRY: OnceLock<Mutex<BackendMap>> = OnceLock::new();

fn registry() -> &'static Mutex<BackendMap> {
    REGISTRY.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Store a backend if not already present, or return the existing one.
///
/// This prevents races where two participants both try to create a ring
/// for the same (topic, epoch) — the first one wins and both get the same Arc.
pub(crate) fn store_or_get_backend(
    name: &str,
    epoch: u64,
    backend: Arc<dyn Any + Send + Sync>,
) -> Arc<dyn Any + Send + Sync> {
    let mut map = registry().lock().unwrap_or_else(|e| e.into_inner());
    map.retain(|(n, e), _| n != name || *e >= epoch.saturating_sub(1));
    map.entry((name.to_string(), epoch))
        .or_insert(backend)
        .clone()
}

/// Look up a backend from the global registry.
pub(crate) fn lookup_backend(name: &str, epoch: u64) -> Option<Arc<dyn Any + Send + Sync>> {
    let map = registry().lock().unwrap_or_else(|e| e.into_inner());
    map.get(&(name.to_string(), epoch)).cloned()
}

/// Remove all entries for a topic (called on cleanup).
pub(crate) fn remove_topic(name: &str) {
    let mut map = registry().lock().unwrap_or_else(|e| e.into_inner());
    map.retain(|(n, _), _| n != name);
}

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

    // ── store_or_get_backend ──

    #[test]
    fn test_store_backend_returns_stored() {
        let name = unique("store");
        let backend: Arc<dyn Any + Send + Sync> = Arc::new(42u64);
        let got = store_or_get_backend(&name, 1, backend);
        assert_eq!(*got.downcast_ref::<u64>().unwrap(), 42);
    }

    #[test]
    fn test_store_same_key_returns_first() {
        let name = unique("first_wins");
        let first: Arc<dyn Any + Send + Sync> = Arc::new(1u64);
        let second: Arc<dyn Any + Send + Sync> = Arc::new(2u64);

        let got1 = store_or_get_backend(&name, 1, first);
        let got2 = store_or_get_backend(&name, 1, second);

        // First one wins
        assert_eq!(*got1.downcast_ref::<u64>().unwrap(), 1);
        assert_eq!(*got2.downcast_ref::<u64>().unwrap(), 1);
    }

    #[test]
    fn test_store_different_epochs() {
        let name = unique("epochs");
        let e1: Arc<dyn Any + Send + Sync> = Arc::new(10u64);
        let e2: Arc<dyn Any + Send + Sync> = Arc::new(20u64);

        store_or_get_backend(&name, 1, e1);
        store_or_get_backend(&name, 2, e2);

        // Both epochs accessible
        assert_eq!(
            *lookup_backend(&name, 1)
                .unwrap()
                .downcast_ref::<u64>()
                .unwrap(),
            10
        );
        assert_eq!(
            *lookup_backend(&name, 2)
                .unwrap()
                .downcast_ref::<u64>()
                .unwrap(),
            20
        );
    }

    #[test]
    fn test_store_cleans_old_epochs() {
        let name = unique("cleanup");
        let e1: Arc<dyn Any + Send + Sync> = Arc::new(1u64);
        let e3: Arc<dyn Any + Send + Sync> = Arc::new(3u64);

        store_or_get_backend(&name, 1, e1);
        // Storing epoch 3 should clean epochs < 2 (epoch - 1)
        store_or_get_backend(&name, 3, e3);

        // Epoch 1 should be cleaned (3 - 1 = 2, retains >= 2)
        assert!(lookup_backend(&name, 1).is_none());
        assert_eq!(
            *lookup_backend(&name, 3)
                .unwrap()
                .downcast_ref::<u64>()
                .unwrap(),
            3
        );
    }

    #[test]
    fn test_store_different_topics_independent() {
        let name_a = unique("topic_a");
        let name_b = unique("topic_b");

        let a: Arc<dyn Any + Send + Sync> = Arc::new(100u64);
        let b: Arc<dyn Any + Send + Sync> = Arc::new(200u64);

        store_or_get_backend(&name_a, 1, a);
        store_or_get_backend(&name_b, 1, b);

        assert_eq!(
            *lookup_backend(&name_a, 1)
                .unwrap()
                .downcast_ref::<u64>()
                .unwrap(),
            100
        );
        assert_eq!(
            *lookup_backend(&name_b, 1)
                .unwrap()
                .downcast_ref::<u64>()
                .unwrap(),
            200
        );
    }

    // ── lookup_backend ──

    #[test]
    fn test_lookup_nonexistent_returns_none() {
        let name = unique("missing");
        assert!(lookup_backend(&name, 1).is_none());
    }

    #[test]
    fn test_lookup_wrong_epoch_returns_none() {
        let name = unique("wrong_epoch");
        let b: Arc<dyn Any + Send + Sync> = Arc::new(42u64);
        store_or_get_backend(&name, 1, b);
        assert!(lookup_backend(&name, 2).is_none());
    }

    // ── remove_topic ──

    #[test]
    fn test_remove_topic_clears_all_epochs() {
        let name = unique("remove");
        let e1: Arc<dyn Any + Send + Sync> = Arc::new(1u64);
        let e2: Arc<dyn Any + Send + Sync> = Arc::new(2u64);

        store_or_get_backend(&name, 1, e1);
        store_or_get_backend(&name, 2, e2);

        remove_topic(&name);

        assert!(lookup_backend(&name, 1).is_none());
        assert!(lookup_backend(&name, 2).is_none());
    }

    #[test]
    fn test_remove_nonexistent_topic_no_panic() {
        remove_topic(&unique("never_existed"));
        // Should not panic
    }

    #[test]
    fn test_remove_doesnt_affect_other_topics() {
        let name_a = unique("rem_a");
        let name_b = unique("rem_b");

        let a: Arc<dyn Any + Send + Sync> = Arc::new(1u64);
        let b: Arc<dyn Any + Send + Sync> = Arc::new(2u64);

        store_or_get_backend(&name_a, 1, a);
        store_or_get_backend(&name_b, 1, b);

        remove_topic(&name_a);

        assert!(lookup_backend(&name_a, 1).is_none());
        assert!(lookup_backend(&name_b, 1).is_some());
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
    fn test_concurrent_store_same_key_first_wins() {
        let name = unique("concurrent_store");
        let barrier = Arc::new(Barrier::new(8));
        let results: Arc<Mutex<Vec<u64>>> = Arc::new(Mutex::new(Vec::new()));

        let handles: Vec<_> = (0..8)
            .map(|i| {
                let name = name.clone();
                let barrier = Arc::clone(&barrier);
                let results = Arc::clone(&results);
                std::thread::spawn(move || {
                    let backend: Arc<dyn Any + Send + Sync> = Arc::new(i as u64);
                    barrier.wait();
                    let got = store_or_get_backend(&name, 1, backend);
                    results
                        .lock()
                        .unwrap()
                        .push(*got.downcast_ref::<u64>().unwrap());
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        let results = results.lock().unwrap();
        // All threads should get the same value (first-wins semantics)
        let first = results[0];
        assert!(results.iter().all(|&v| v == first));
    }

    #[test]
    fn test_concurrent_lookup_during_store() {
        let name = unique("concurrent_lookup");
        let barrier = Arc::new(Barrier::new(8));

        // Pre-store a value
        let backend: Arc<dyn Any + Send + Sync> = Arc::new(99u64);
        store_or_get_backend(&name, 1, backend);

        let handles: Vec<_> = (0..8)
            .map(|_| {
                let name = name.clone();
                let barrier = Arc::clone(&barrier);
                std::thread::spawn(move || {
                    barrier.wait();
                    // Half threads lookup, half threads store new epochs
                    let val = lookup_backend(&name, 1);
                    assert!(val.is_some());
                    assert_eq!(*val.unwrap().downcast_ref::<u64>().unwrap(), 99);
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }
    }

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

    #[test]
    fn test_concurrent_store_and_remove() {
        let name = unique("store_remove");
        let barrier = Arc::new(Barrier::new(4));

        let mut handles = Vec::new();
        // 2 threads store
        for i in 0..2 {
            let name = name.clone();
            let barrier = Arc::clone(&barrier);
            handles.push(std::thread::spawn(move || {
                barrier.wait();
                for e in 0..50 {
                    let b: Arc<dyn Any + Send + Sync> = Arc::new((i * 50 + e) as u64);
                    store_or_get_backend(&name, e as u64, b);
                }
            }));
        }
        // 2 threads remove
        for _ in 0..2 {
            let name = name.clone();
            let barrier = Arc::clone(&barrier);
            handles.push(std::thread::spawn(move || {
                barrier.wait();
                for _ in 0..50 {
                    remove_topic(&name);
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }
        // Should not panic or deadlock
    }
}
