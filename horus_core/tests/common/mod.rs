//! Shared test utilities for horus_core integration tests
//!
//! # Test Isolation Requirements
//!
//! HORUS tests share global state through shared memory (managed by `horus_sys`),
//! the `OnceLock`-based `REGISTRY`, and `EPOCH_NOTIFY`. To avoid interference
//! when running tests in parallel (`--test-threads=N`):
//!
//! 1. **Always use unique topic/pool names.** Call [`unique()`] or the
//!    unit-test helper `unique(prefix)` (in `topic/tests.rs`) to generate
//!    names that include the PID and a monotonic counter. Never hardcode
//!    topic names unless the test specifically validates the `topics!` macro.
//!
//! 2. **Create Topic handles on the main thread** when testing cross-thread
//!    send/recv. This ensures the adaptive backend migration (e.g. SpscShm
//!    → MpscShm) completes before threads start transferring data. Creating
//!    topics inside `thread::spawn` closures can race with migration.
//!
//! 3. **Use unique pool IDs** for `TensorPool` tests. Do NOT pick 9600–9699:
//!    that range is taken by `horus_core/src/memory/tensor_pool.rs`'s own unit
//!    tests, which is what this note used to recommend. For a pool that needs
//!    to be isolated from other *processes*, use a `POOL_BASE_*` constant with
//!    [`test_pool_id`] — see the "Tensor pool id space" section below.
//!
//! 4. **Call `cleanup_stale_shm()`** at the start of integration tests that
//!    create SHM files, to remove leftover state from previous runs.
//!
//! 5. **Do not depend on test execution order.** Tests must be independent
//!    and pass when run individually or in any order with any thread count.
//!
//! # Running the integration suite
//!
//! [`cleanup_stale_shm`] returns a lock guard that serializes the SHM-touching
//! tests which call it, so those are deterministic at any `--test-threads`.
//! Some integration tests, however, share process-global state WITHOUT calling
//! `cleanup_stale_shm` (service registries, RT CPU timing, cross-process SHM),
//! and still race under the default in-binary parallelism. For a fully
//! deterministic run, execute the integration suite serially:
//!
//! ```sh
//! cargo test --workspace -- --test-threads=1
//! # or: RUST_TEST_THREADS=1 cargo test --workspace
//! ```
//!
//! (A `.cargo/config.toml` `[env] RUST_TEST_THREADS` was deliberately NOT added:
//! `.cargo/*` is gitignored except `audit.toml`, and an `[env]` block there
//! breaks the `horus_py` pyo3 link step in this workspace. Use the invocation
//! above — or a CI job flag — instead.)

/// Process-wide lock serializing tests that touch the shared "default" SHM
/// namespace (see [`cleanup_stale_shm`]).
fn shm_test_lock() -> &'static std::sync::Mutex<()> {
    static LOCK: std::sync::OnceLock<std::sync::Mutex<()>> = std::sync::OnceLock::new();
    LOCK.get_or_init(|| std::sync::Mutex::new(()))
}

thread_local! {
    // Reentrancy depth + the held guard for THIS thread. A test may call
    // cleanup_stale_shm() more than once; only the first acquisition takes the
    // global lock, and it is released when the last guard on this thread drops.
    static SHM_DEPTH: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
    static SHM_HELD: std::cell::RefCell<Option<std::sync::MutexGuard<'static, ()>>> =
        const { std::cell::RefCell::new(None) };
}

/// RAII guard returned by [`cleanup_stale_shm`]. While any guard is alive on a
/// test thread, that thread exclusively holds the shared-SHM test lock.
#[must_use = "bind the guard (e.g. `let _shm = cleanup_stale_shm();`) so the \
              shared-SHM lock is held for the whole test; dropping it immediately \
              re-exposes the test to cross-test SHM races"]
pub struct ShmTestGuard {
    _priv: (),
}

impl Drop for ShmTestGuard {
    fn drop(&mut self) {
        SHM_DEPTH.with(|d| {
            let next = d.get().saturating_sub(1);
            d.set(next);
            if next == 0 {
                // Release the global lock (drops the stored MutexGuard).
                SHM_HELD.with(|h| *h.borrow_mut() = None);
            }
        });
    }
}

/// Clean up shared memory files before each test AND serialize the test against
/// other SHM-touching tests in the same binary.
///
/// # Why this returns a guard
///
/// Integration tests share the process-global "default" SHM namespace, and this
/// function `remove_dir_all`s the entire topics/nodes directory. Run in parallel
/// (the default within a test binary), one test's cleanup wipes another's live
/// SHM state — a pervasive source of spurious failures (lost messages, event
/// nodes that never tick, etc.). Binding the returned guard for the whole test
/// body serializes SHM-touching tests, so the suite is deterministic regardless
/// of `--test-threads`. The guard is reentrant: calling this twice within one
/// test does not deadlock.
///
/// Stale files from previous *runs* can also cause SIGSEGV when mapped with an
/// incompatible layout, so the cleanup itself is still performed here.
pub fn cleanup_stale_shm() -> ShmTestGuard {
    SHM_DEPTH.with(|d| {
        if d.get() == 0 {
            let guard = shm_test_lock().lock().unwrap_or_else(|e| e.into_inner());
            SHM_HELD.with(|h| *h.borrow_mut() = Some(guard));
        }
        d.set(d.get() + 1);
    });

    let topics = horus_sys::shm::shm_topics_dir();
    let nodes = horus_sys::shm::shm_nodes_dir();
    let _ = std::fs::remove_dir_all(&topics);
    let _ = std::fs::remove_dir_all(&nodes);

    ShmTestGuard { _priv: () }
}

/// Generate a unique name for integration tests.
///
/// Includes the PID and a monotonic counter to avoid collisions between
/// parallel test threads and concurrent test processes.
#[allow(dead_code)]
pub fn unique(prefix: &str) -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

/// RAII guard for temporary directories used in tests.
///
/// Creates a unique temporary directory on construction and removes it
/// (including all contents) on drop — even if the test panics. This
/// prevents leftover test artifacts from accumulating in `/tmp`.
///
/// # Example
/// ```ignore
/// let tmp = TestTempDir::new("my_test");
/// // use tmp.path() for file operations
/// // directory is automatically cleaned up when `tmp` goes out of scope
/// ```
#[allow(dead_code)]
pub struct TestTempDir {
    path: std::path::PathBuf,
}

#[allow(dead_code)]
impl TestTempDir {
    /// Create a new temporary directory with the given prefix.
    ///
    /// The directory name includes the PID and a counter to avoid collisions.
    pub fn new(prefix: &str) -> Self {
        let path = std::env::temp_dir().join(unique(prefix));
        let _ = std::fs::remove_dir_all(&path); // clean stale data
        std::fs::create_dir_all(&path).expect("failed to create test temp dir");
        Self { path }
    }

    /// Return the path to the temporary directory.
    pub fn path(&self) -> &std::path::Path {
        &self.path
    }
}

impl Drop for TestTempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.path);
    }
}

/// Current wall-clock time as nanoseconds since the UNIX epoch.
///
/// Replacement for `horus_library::transform_frame::timestamp_now()` that
/// avoids pulling in the `horus_library` crate as a test dependency.
#[allow(dead_code)]
pub fn timestamp_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock before UNIX epoch")
        .as_nanos() as u64
}

/// RAII guard that cleans up HORUS shared memory on drop.
///
/// Removes SHM topics and nodes directories when the guard is dropped,
/// ensuring no stale SHM state leaks between tests.
/// Also cleans on construction so the test starts with a fresh state.
///
/// # Example
/// ```ignore
/// let _shm = ShmCleanupGuard::new();
/// // SHM is cleaned on creation and again on drop (even on panic)
/// ```
#[allow(dead_code)]
pub struct ShmCleanupGuard {
    // Holds the shared-SHM test lock for the guard's whole lifetime (i.e. the
    // test body), serializing this test against other SHM-touching tests.
    _shm: ShmTestGuard,
}

#[allow(dead_code)]
impl ShmCleanupGuard {
    pub fn new() -> Self {
        Self {
            _shm: cleanup_stale_shm(),
        }
    }
}
// No custom Drop: the held `ShmTestGuard` releases the lock on drop, and the
// next test's `cleanup_stale_shm()` scrubs SHM under the lock.

// ============================================================================
// Tensor pool id space
// ============================================================================
//
// A `TensorPool` is shared memory keyed by a numeric id, and its `Drop`
// deliberately does NOT unlink the backing file (`cleanup_stale_shm` only
// clears topics/ and nodes/). So an id is not just a within-run handle: it
// names a file that outlives the process, and a later run asking for the same
// id gets whatever geometry an earlier run left there. `TensorPool::new`
// refuses to attach when the existing file is smaller than the mapping the
// caller's config needs, which surfaces as
//
//     Tensor pool 9811 exists but is only 65856 bytes, smaller than the
//     16780864 bytes this process's configuration requires
//
// Tests that want isolation between concurrently running processes derive the
// id as `BASE + (std::process::id() % POOL_PID_SPREAD)`, so each base owns the
// half-open range `[BASE, BASE + POOL_PID_SPREAD)`. Two consequences:
//
//   * Bases must be at least `POOL_PID_SPREAD` apart, or the ranges overlap and
//     two tests with different pool geometries can land on one id.
//   * The whole family must live where no FIXED id is used, because a fixed id
//     inside a range collides whenever `pid % POOL_PID_SPREAD` selects it.
//
// The second point is why this band is 20_000 and not somewhere in the 9500s:
// 9500-10200 is densely occupied by fixed ids that are easy to miss --
// `horus_core/src/memory/tensor_pool.rs` unit tests (9600-9614, 9782-9796,
// 9900-9902, 9974-9998, 10100-10105), `tests/regressions.rs` (9800, 9801) and
// `benchmarks/benches/tensor_pool.rs` (9500-9639). Keeping the pid-spread
// family in its own band means "is this id free?" is answered by the band, not
// by re-auditing every literal in the workspace.
//
// `tensor_pool_concurrent::pool_id_ranges_cannot_overlap` enforces all of it.

/// Width of the pid-derived offset added to each `POOL_BASE_*` below.
#[allow(dead_code)]
pub const POOL_PID_SPREAD: u32 = 100;

/// First id of the band reserved for the `POOL_BASE_*` family.
#[allow(dead_code)]
pub const POOL_BAND_START: u32 = 20_000;

/// One past the last id of that band. Every base's full spread fits inside it,
/// and nothing else in the workspace allocates a tensor pool here.
#[allow(dead_code)]
pub const POOL_BAND_END: u32 = 20_600;

/// `tensor_pool_concurrent::test_concurrent_alloc_release_no_panic`
#[allow(dead_code)]
pub const POOL_BASE_CONCURRENT_ALLOC_RELEASE: u32 = 20_000;

/// `tensor_pool_concurrent::test_concurrent_alloc_all_threads_succeed`
#[allow(dead_code)]
pub const POOL_BASE_CONCURRENT_ALLOC_ALL: u32 = 20_100;

/// `tensor_pool_concurrent::test_slot_reuse_after_release`
#[allow(dead_code)]
pub const POOL_BASE_SLOT_REUSE: u32 = 20_200;

/// `tensor_pool_concurrent::test_single_slot_pool`
#[allow(dead_code)]
pub const POOL_BASE_SINGLE_SLOT: u32 = 20_300;

/// `tensor_pool_concurrent::test_data_integrity_after_write`
#[allow(dead_code)]
pub const POOL_BASE_DATA_INTEGRITY: u32 = 20_400;

/// `resource_exhaustion::test_tensor_pool_alloc_returns_error_when_full`
#[allow(dead_code)]
pub const POOL_BASE_RESOURCE_EXHAUSTION: u32 = 20_500;

/// Every base in the family, for the invariant test.
///
/// This is built FROM the constants above rather than repeating their values,
/// so the checked list cannot drift from what the call sites actually use.
/// Adding a base means adding it here and widening [`POOL_BAND_END`] --
/// `pool_id_ranges_cannot_overlap` asserts the band is exactly consumed, so
/// forgetting either half fails the test.
#[allow(dead_code)]
pub const TENSOR_POOL_BASES: [(&str, u32); 6] = [
    (
        "concurrent_alloc_release",
        POOL_BASE_CONCURRENT_ALLOC_RELEASE,
    ),
    ("concurrent_alloc_all", POOL_BASE_CONCURRENT_ALLOC_ALL),
    ("slot_reuse", POOL_BASE_SLOT_REUSE),
    ("single_slot", POOL_BASE_SINGLE_SLOT),
    ("data_integrity", POOL_BASE_DATA_INTEGRITY),
    (
        "resource_exhaustion (other file)",
        POOL_BASE_RESOURCE_EXHAUSTION,
    ),
];

/// Pool id for this process: a base plus this process's spread slot.
#[allow(dead_code)]
pub fn test_pool_id(base: u32) -> u32 {
    base + (std::process::id() % POOL_PID_SPREAD)
}
