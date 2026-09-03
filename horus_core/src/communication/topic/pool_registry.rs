//! Shared pool registry for topic tensor extensions
//!
//! All topic extensions (`Topic<Image>`, `Topic<PointCloud>`, `Topic<DepthImage>`,
//! `Topic<Tensor>`) share a single global registry of per-topic tensor pools.
//! The pool_id is derived deterministically from the topic name using FNV-1a so
//! that publisher and subscriber processes converge on the same shared memory file.

use std::collections::HashMap;
use std::sync::Arc;

use lazy_static::lazy_static;
use parking_lot::Mutex;

use crate::error::{HorusContext, HorusResult};
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
    if hash == 0 {
        1
    } else {
        hash
    }
}

/// Build the `TensorPoolConfig` — currently always mmap-backed.
fn auto_pool_config() -> TensorPoolConfig {
    TensorPoolConfig::default()
}

/// Get or create the auto-managed tensor pool for a topic name.
///
/// On first call, opens an existing pool (if another process created it)
/// or creates a new one backed by shared memory. Subsequent calls return
/// the cached pool.
///
/// When creating a new pool, GPU hardware is auto-detected and the optimal
/// allocator backend is selected (managed memory on Jetson, pinned memory
/// on discrete GPU, mmap on CPU-only).
///
/// # Errors
///
/// Returns an error when neither opening nor creating the pool works, carrying
/// *both* halves: the `TensorPool::open` failure as context and the
/// `TensorPool::new` failure as its cause. The two say different things — the
/// first names why the file on disk was rejected (header magic, `POOL_VERSION`),
/// the second only what the recreate attempt tripped over (geometry, size) —
/// and it is usually the first that identifies the stale file.
///
/// Both fail together whenever the file on disk disagrees with
/// this process: a `POOL_VERSION` mismatch, a geometry mismatch, or a file
/// shorter than this process's geometry. That disagreement is not exotic —
/// `TensorPool::drop` never unlinks and the filename carries no version, so an
/// in-place upgrade across any of the four `POOL_VERSION` bumps leaves a
/// poisoned file behind for the next run to find.
///
/// This used to `.expect()`, which turned every one of those into a panic on
/// the per-frame allocation path, out of constructors (`Topic::new`,
/// `Image::new`, ...) that return `HorusResult` and so advertise an error
/// channel the panic bypassed. Inside a tick `NodeRunner::run_tick`'s
/// `catch_unwind` swallowed it — a measured run had the camera node enter 8
/// ticks, complete 0, and the process still exit 0 — and outside one it killed
/// the process.
pub(crate) fn get_or_create_pool(topic_name: &str) -> HorusResult<Arc<TensorPool>> {
    let mut pools = TOPIC_POOLS.lock();
    if let Some(pool) = pools.get(topic_name) {
        return Ok(Arc::clone(pool));
    }

    let pid = pool_id_from_name(topic_name);

    let pool = Arc::new(match TensorPool::open(pid) {
        Ok(p) => p,
        // Keep the `open` error. When both halves fail on the same stale file
        // they fail for different reasons, and `open`'s is the one that names
        // the file as stale — dropping it left the caller with only the
        // recreate error ("exists but is only N bytes"), which reads like a
        // race with a concurrent creator rather than a leftover from an older
        // build. `horus_context_with` only allocates on the error path.
        Err(open_err) => TensorPool::new(pid, auto_pool_config()).horus_context_with(|| {
            format!(
                "tensor pool for topic '{topic_name}' (pool_id {pid}): the existing pool \
                 file could not be opened ({open_err}), and creating it failed too"
            )
        })?,
    });

    pools.insert(topic_name.to_string(), Arc::clone(&pool));
    Ok(pool)
}

/// Get or create a device-specific tensor pool for a topic name.
///
/// Default pool name for standalone type creation (Image::new, PointCloud::new, etc.).
const GLOBAL_POOL_NAME: &str = "__horus_global__";

/// Get the global pool for standalone type creation.
///
/// Used by `Image::new()`, `PointCloud::new()`, `DepthImage::new()` when
/// creating types outside of a Topic context. Also used by Python bindings.
pub(crate) fn global_pool() -> HorusResult<Arc<TensorPool>> {
    get_or_create_pool(GLOBAL_POOL_NAME)
}

/// The pool a pool-backed handle falls back to when it carries none.
///
/// Unreachable for the types that call it: `Topic::<T>::new` builds the pool
/// for every `T::needs_pool()` type before the handle exists and `Clone` copies
/// it, so an `Image`/`PointCloud`/`DepthImage`/`CostMap`/`TensorHandle` handle
/// reaches `from_wire` or a keep-alive publish with `Some(pool)` — and by then
/// the pool has already been opened once, so this call hits the cache above
/// rather than the file.
///
/// It exists only because `TopicMessage::from_wire` returns `Self`: it has no
/// error channel to propagate into, and giving it one would change a trait
/// every message type implements. Every caller that *does* have a channel —
/// `try_from_wire`, the constructors, the spill paths — takes the fallible
/// [`global_pool`] instead.
pub(crate) fn fallback_pool() -> Arc<TensorPool> {
    global_pool().expect(
        "a pool-backed handle always carries its pool (Topic::new builds it), so the \
         global pool was already opened successfully before this point",
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_id_deterministic() {
        let id1 = pool_id_from_name("camera.rgb");
        let id2 = pool_id_from_name("camera.rgb");
        assert_eq!(id1, id2);
    }

    #[test]
    fn test_pool_id_different_names() {
        let id1 = pool_id_from_name("camera.rgb");
        let id2 = pool_id_from_name("lidar.points");
        assert_ne!(id1, id2);
    }

    #[test]
    fn test_pool_id_nonzero() {
        let id = pool_id_from_name("");
        assert_ne!(id, 0);
    }

    /// A pool file this process disagrees with must be an error, not a panic.
    ///
    /// `TensorPool::drop` never unlinks and the filename carries no version, so
    /// a pool file outlives the process that made it, and an in-place upgrade
    /// across a `POOL_VERSION` bump leaves one behind for the next run to find
    /// (a stale 1 GB `tensor_pool_1` was sitting in the default namespace on the
    /// machine this was written on). Both halves of `get_or_create_pool` then
    /// fail on it, which is what the planted file below reproduces: `open()`
    /// rejects the header version, and the `TensorPool::new` fallback rejects
    /// the geometry the older build laid the file out with.
    ///
    /// The `.expect()` this replaces made that a panic out of `Topic::new`,
    /// `Image::new` and friends — all of which return `HorusResult` — and it
    /// fired on every call, because the failure happens before `pools.insert`
    /// and so is never cached. Inside a scheduler tick `catch_unwind` swallowed
    /// it and the node produced nothing while the process still exited 0;
    /// outside one it killed the process.
    ///
    /// NOTE: without the fix this test does not fail on an assert — it panics
    /// inside `get_or_create_pool`, which is the defect itself.
    #[test]
    fn a_pool_file_this_build_disagrees_with_errors_instead_of_panicking() {
        use std::io::{Read, Seek, SeekFrom, Write};

        /// Unlinks the planted file however this test leaves — including the
        /// panic it exists to rule out, which is the one exit that would
        /// otherwise leave a poisoned pool file in the namespace.
        struct Planted(std::path::PathBuf);
        impl Drop for Planted {
            fn drop(&mut self) {
                let _ = std::fs::remove_file(&self.0);
            }
        }

        // Per-process name. A test binary's SHM namespace is keyed on the cargo
        // *target directory* (`shm::cargo_test_namespace`), not on the process,
        // so two runs out of one target dir share it — and this test plants,
        // corrupts and unlinks a fixed path. The pid keeps concurrent runs off
        // each other's file.
        let topic = format!(
            "__horus_test_pool_registry_version_mismatch_{}__",
            std::process::id()
        );
        let topic = topic.as_str();
        let pid = pool_id_from_name(topic);
        let path = crate::memory::platform::shm_base_dir()
            .join("tensors")
            .join(format!("tensor_pool_{}", pid));
        let _ = std::fs::remove_file(&path);
        let _planted = Planted(path.clone());

        // Plant a real, fully initialized pool whose geometry is NOT
        // `auto_pool_config()`'s, then let it drop — the file stays.
        {
            let config = TensorPoolConfig {
                pool_size: 64 * 1024,
                max_slots: 4,
                ..TensorPoolConfig::default()
            };
            TensorPool::new(pid, config).expect("planting the stale pool file");
        }

        // Age it by one `POOL_VERSION`. `PoolHeader` is `repr(C)` with
        // `magic: u64` first, so `version: u32` sits at offset 8.
        {
            let mut f = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .open(&path)
                .expect("reopening the planted pool file");
            let mut version = [0u8; 4];
            f.seek(SeekFrom::Start(8)).expect("seek to header version");
            f.read_exact(&mut version).expect("read header version");
            let older = u32::from_le_bytes(version).wrapping_sub(1);
            f.seek(SeekFrom::Start(8)).expect("seek to header version");
            f.write_all(&older.to_le_bytes())
                .expect("write header version");
            f.flush().expect("flush header version");
        }

        let result = get_or_create_pool(topic);

        let err = result
            .err()
            .expect("a pool file from another POOL_VERSION must fail, not open");

        // Both halves of the failure reach the caller. `open`'s is the one that
        // identifies the file as stale ("version mismatch: expected 4, got 3");
        // `new`'s only reports the geometry the second attempt tripped over,
        // which on its own reads like a race with a concurrent creator.
        let rendered = err.to_string();
        assert!(
            rendered.contains("version mismatch"),
            "the `open` failure names the file as stale and must survive into \
             the error the caller sees: {rendered}"
        );
        assert!(
            rendered.contains("Caused by:"),
            "the `new` failure must survive too, as the cause: {rendered}"
        );

        // Nothing was cached, so the next caller gets the same error rather
        // than a half-built pool.
        assert!(
            !TOPIC_POOLS.lock().contains_key(topic),
            "a pool that could not be opened must not be registered: {}",
            err
        );
    }
}
