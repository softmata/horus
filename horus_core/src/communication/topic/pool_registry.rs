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

use crate::error::{HorusContext, HorusError, HorusResult};
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

/// [`global_pool`] for a caller with no error channel. See [`pool_or_report`].
pub(crate) fn global_pool_or_report() -> Option<Arc<TensorPool>> {
    pool_or_report(GLOBAL_POOL_NAME)
}

/// [`get_or_create_pool`] for the callers that have no error channel to
/// propagate into: `recv_handle`, the spill paths, `try_from_wire`. They answer
/// `None`, which at the call site means "no message" — so an unusable pool,
/// which is a permanent configuration fault, arrives looking exactly like an
/// idle topic, and the caller polls forever with nothing to go on.
///
/// That is the same invisibility the panic this replaced had (swallowed by
/// `run_tick`'s `catch_unwind`, node completing no ticks, process exiting 0),
/// and this module's neighbours already refuse it: a dropped message on the
/// abandoned-claim path is counted *and* "said out loud, because 'my control
/// topic has a hole in it' is not something an operator should have to go
/// looking for a counter to discover". So say it.
///
/// Nothing is cached on the failure path, so the failure recurs on every call —
/// at tick rate. Hence the throttle in [`report_unusable_pool`].
pub(crate) fn pool_or_report(topic_name: &str) -> Option<Arc<TensorPool>> {
    match get_or_create_pool(topic_name) {
        Ok(pool) => Some(pool),
        Err(err) => {
            report_unusable_pool(topic_name, &err);
            None
        }
    }
}

/// One line per topic per window. Fast enough that an operator watching a
/// terminal sees the fault immediately, slow enough that a 1 kHz subscriber
/// cannot outrun the log.
const REPORT_WINDOW_MS: u64 = 1_000;

/// Throttle state for [`report_unusable_pool`], keyed by topic name.
///
/// Per topic, not per call site. `hlog_every!` keeps its timestamp in a
/// `static` at the expansion point, and every caller here funnels through one
/// helper — so a call-site gate would let the first broken topic silence every
/// other broken topic, which is worse than no throttle at all, because the one
/// that got through looks like the only one in trouble. `DiagThrottle` exists
/// in the scheduler for exactly this reason.
///
/// Only ever touched on the failure path.
struct PoolReport {
    /// Wall clock of the last line emitted for this topic, ms since the epoch.
    /// `0` is "never reported" and always emits, so a fault is never delayed.
    last_ms: u64,
    /// Failures swallowed since that line.
    suppressed: u64,
}

lazy_static! {
    static ref POOL_REPORTS: Mutex<HashMap<String, PoolReport>> = Mutex::new(HashMap::new());
}

/// `Some(suppressed_since_the_last_line)` when this failure should be emitted.
///
/// Takes `now_ms` rather than reading the clock so the test is deterministic.
fn allow_report(topic_name: &str, now_ms: u64) -> Option<u64> {
    let mut reports = POOL_REPORTS.lock();
    let entry = reports.entry(topic_name.to_string()).or_insert(PoolReport {
        last_ms: 0,
        suppressed: 0,
    });
    if entry.last_ms != 0 && now_ms.saturating_sub(entry.last_ms) < REPORT_WINDOW_MS {
        entry.suppressed = entry.suppressed.saturating_add(1);
        return None;
    }
    entry.last_ms = now_ms;
    Some(std::mem::take(&mut entry.suppressed))
}

/// What the operator reads. Split out from the `log::warn!` so a test can hold
/// it to its claims without installing a logger.
///
/// Carries the suppressed count for the same reason `DiagThrottle` does:
/// throttling may hide the volume of the output, never the volume of the fault.
fn unusable_pool_message(
    topic_name: &str,
    pool_id: u32,
    suppressed: u64,
    err: &HorusError,
) -> String {
    let also = if suppressed == 0 {
        String::new()
    } else {
        format!(" ({suppressed} further failures suppressed since the last line)")
    };
    format!(
        "Topic '{topic_name}': its tensor pool (pool_id {pool_id}) can be neither \
         opened nor created, so every message that needs it is being \
         DROPPED{also} — the topic is faulted, not idle. This does not clear \
         by itself: {err}"
    )
}

/// Say an unusable pool out loud, at most once per [`REPORT_WINDOW_MS`] per
/// topic.
///
/// Cold by construction — reached only when the pool file on disk disagrees
/// with this build — so the clock read, the map lookup and the formatting are
/// paid by a robot that is already producing nothing.
#[cold]
#[inline(never)]
fn report_unusable_pool(topic_name: &str, err: &HorusError) {
    let now_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;
    if let Some(suppressed) = allow_report(topic_name, now_ms) {
        log::warn!(
            "{}",
            unusable_pool_message(topic_name, pool_id_from_name(topic_name), suppressed, err)
        );
    }
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
/// error channel to propagate into at all, not even an `Option`, and giving it
/// one would change a trait every message type implements. Every other caller
/// has somewhere to put the failure — the constructors take the fallible
/// [`global_pool`]; `try_from_wire` and the spill paths take
/// [`global_pool_or_report`] / [`pool_or_report`], which answer `None` and say
/// why.
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
        use crate::memory::tensor_pool::HEADER_VERSION_OFFSET as VERSION_OFFSET;
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

        // Age it by one `POOL_VERSION`. The offset comes from the struct
        // (`offset_of!`), not from a literal, so a header field added ahead of
        // `version` moves the write instead of leaving this corrupting a
        // neighbouring field and failing on the assertions below.
        {
            let mut f = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .open(&path)
                .expect("reopening the planted pool file");
            let mut version = [0u8; 4];
            f.seek(SeekFrom::Start(VERSION_OFFSET))
                .expect("seek to header version");
            f.read_exact(&mut version).expect("read header version");
            let older = u32::from_le_bytes(version).wrapping_sub(1);
            f.seek(SeekFrom::Start(VERSION_OFFSET))
                .expect("seek to header version");
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

        // And the wrapper the no-error-channel callers use (`recv_handle`, the
        // spill paths) answers `None` on the same file rather than panicking or
        // handing back a pool it could not open.
        assert!(
            pool_or_report(topic).is_none(),
            "pool_or_report must refuse a pool file this build disagrees with"
        );
    }

    /// The throttle that keeps the report above from following a 1 kHz
    /// subscriber into the log.
    ///
    /// Failures are not cached — `get_or_create_pool` returns before
    /// `pools.insert` — so a faulted topic fails on *every* call, which is what
    /// makes the throttle load-bearing rather than decorative.
    #[test]
    fn an_unusable_pool_is_reported_once_per_window_and_carries_what_it_hid() {
        let topic = format!("__horus_test_pool_report_window_{}__", std::process::id());
        let t = topic.as_str();

        // The first occurrence always speaks: a fault is never delayed by up to
        // a window the first time it happens.
        assert_eq!(allow_report(t, 10_000), Some(0));
        for i in 1..REPORT_WINDOW_MS {
            assert_eq!(allow_report(t, 10_000 + i), None);
        }
        // The next line through carries the count, so the throttle hides the
        // volume of the output and never the volume of the fault.
        assert_eq!(
            allow_report(t, 10_000 + REPORT_WINDOW_MS),
            Some(REPORT_WINDOW_MS - 1)
        );
        // ...and the count resets once reported.
        assert_eq!(allow_report(t, 10_000 + 2 * REPORT_WINDOW_MS), Some(0));
    }

    /// Per topic, not per call site.
    ///
    /// Every caller funnels through one helper, so a call-site gate
    /// (`hlog_every!`) would let the first broken topic silence the rest — and
    /// the survivor would read as the only topic in trouble. Same reason the
    /// scheduler has `DiagThrottle` instead.
    #[test]
    fn one_faulted_topic_does_not_silence_another() {
        let pid = std::process::id();
        let a = format!("__horus_test_pool_report_a_{pid}__");
        let b = format!("__horus_test_pool_report_b_{pid}__");

        assert_eq!(allow_report(&a, 20_000), Some(0));
        assert_eq!(allow_report(&a, 20_001), None);
        assert_eq!(
            allow_report(&b, 20_001),
            Some(0),
            "a second faulted topic must still be able to report itself"
        );
    }

    /// The line an operator actually reads has to distinguish the fault from
    /// the "no message" it is delivered as, and carry the cause.
    #[test]
    fn the_report_names_the_topic_the_fault_and_the_cause() {
        let err = HorusError::Config(crate::error::ConfigError::Other(
            "Tensor pool version mismatch: expected 4, got 3".to_string(),
        ));

        let quiet = unusable_pool_message("camera.rgb", 4242, 0, &err);
        assert!(quiet.contains("camera.rgb"), "{quiet}");
        assert!(
            quiet.contains("4242"),
            "the pool_id names the file: {quiet}"
        );
        assert!(
            quiet.contains("DROPPED") && quiet.contains("not idle"),
            "the whole point is that this is not an empty topic: {quiet}"
        );
        assert!(
            quiet.contains("version mismatch: expected 4, got 3"),
            "the cause must survive into the line: {quiet}"
        );
        assert!(
            !quiet.contains("suppressed"),
            "nothing was suppressed, so nothing to report: {quiet}"
        );

        let throttled = unusable_pool_message("camera.rgb", 4242, 999, &err);
        assert!(
            throttled.contains("999") && throttled.contains("suppressed"),
            "a throttled line must still report the volume of the fault: {throttled}"
        );
    }
}
