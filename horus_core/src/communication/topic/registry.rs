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
