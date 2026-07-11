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
//!    send/recv. This ensures the adaptive backend migration (DirectChannel
//!    → SpscIntra) completes before threads start transferring data. Creating
//!    topics inside `thread::spawn` closures can race with migration.
//!
//! 3. **Use unique pool IDs** for `TensorPool` tests (e.g., 9600–9699) to
//!    avoid SHM collisions with other test pools.
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
            let guard = shm_test_lock()
                .lock()
                .unwrap_or_else(|e| e.into_inner());
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
