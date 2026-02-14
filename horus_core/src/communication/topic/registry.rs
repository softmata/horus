//! Global backend registry for intra-process migration.
//!
//! When multiple `Topic` instances on different threads share the same
//! topic name (intra-process), they need to share the same heap-backed backend
//! (e.g., the same `Arc<SpscRing<T>>`). This registry provides that shared
//! lookup, keyed by (topic_name, epoch).

use std::any::Any;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

/// Global registry mapping (topic_name, epoch) → Arc<dyn Any>.
///
/// Each entry is a type-erased Arc to the backend (e.g., `Arc<SpscRing<T>>`).
/// When a participant detects an epoch change, it looks up the new backend here.
static REGISTRY: OnceLock<Mutex<HashMap<(String, u64), Arc<dyn Any + Send + Sync>>>> =
    OnceLock::new();

fn registry() -> &'static Mutex<HashMap<(String, u64), Arc<dyn Any + Send + Sync>>> {
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
