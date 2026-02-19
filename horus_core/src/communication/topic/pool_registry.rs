//! Shared pool registry for topic tensor extensions
//!
//! All topic extensions (`Topic<Image>`, `Topic<PointCloud>`, `Topic<DepthImage>`,
//! `Topic<HorusTensor>`) share a single global registry of per-topic tensor pools.
//! The pool_id is derived deterministically from the topic name using FNV-1a so
//! that publisher and subscriber processes converge on the same shared memory file.

use std::collections::HashMap;
use std::sync::Arc;

use lazy_static::lazy_static;
use parking_lot::Mutex;

use crate::memory::{TensorPool, TensorPoolConfig};

lazy_static! {
    /// Global registry of per-topic tensor pools.
    ///
    /// Keyed by topic name. The pool_id is derived deterministically from the
    /// name so that different processes converge on the same shared memory file.
    pub(crate) static ref TOPIC_POOLS: Mutex<HashMap<String, Arc<TensorPool>>> =
        Mutex::new(HashMap::new());
}

/// Derive a deterministic pool_id from a topic name using FNV-1a (32-bit).
///
/// Consistent across processes so publisher and subscriber open the same pool.
pub(crate) fn pool_id_from_name(name: &str) -> u32 {
    let mut hash: u32 = 0x811c_9dc5; // FNV-1a offset basis
    for byte in name.as_bytes() {
        hash ^= *byte as u32;
        hash = hash.wrapping_mul(0x0100_0193); // FNV-1a prime
    }
    // Reserve 0 as sentinel — shift to 1
    if hash == 0 { 1 } else { hash }
}

/// Get or create the auto-managed tensor pool for a topic name.
///
/// On first call, opens an existing pool (if another process created it)
/// or creates a new one backed by shared memory. Subsequent calls return
/// the cached pool.
pub(crate) fn get_or_create_pool(topic_name: &str) -> Arc<TensorPool> {
    let mut pools = TOPIC_POOLS.lock();
    if let Some(pool) = pools.get(topic_name) {
        return Arc::clone(pool);
    }

    let pid = pool_id_from_name(topic_name);

    let pool = Arc::new(match TensorPool::open(pid) {
        Ok(p) => p,
        Err(_) => TensorPool::new(pid, TensorPoolConfig::default())
            .expect("failed to create tensor pool for topic"),
    });

    pools.insert(topic_name.to_string(), Arc::clone(&pool));
    pool
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_id_deterministic() {
        let id1 = pool_id_from_name("camera/rgb");
        let id2 = pool_id_from_name("camera/rgb");
        assert_eq!(id1, id2);
    }

    #[test]
    fn test_pool_id_different_names() {
        let id1 = pool_id_from_name("camera/rgb");
        let id2 = pool_id_from_name("lidar/points");
        assert_ne!(id1, id2);
    }

    #[test]
    fn test_pool_id_nonzero() {
        let id = pool_id_from_name("");
        assert_ne!(id, 0);
    }
}
