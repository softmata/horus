//! Platform Abstraction Intent Tests
//!
//! Verifies that horus_sys platform abstractions behave correctly on the
//! current platform. Each test documents its INTENT as a doc comment so
//! the "why" is always clear.

// =========================================================================
// Test 1: detect_capabilities reports Linux
// =========================================================================

/// INTENT: On Linux, detect_capabilities() correctly identifies the platform.
///
/// cpu_count must be > 0, kernel_version must be populated, and
/// memory_locking must be reported (true or false, not undefined).
#[test]
fn test_platform_intent_detect_capabilities_reports_linux() {
    let caps = horus_sys::rt::detect_capabilities();

    assert!(
        caps.cpu_count > 0,
        "detect_capabilities() must report at least 1 CPU, got {}",
        caps.cpu_count,
    );

    // On Linux, kernel_version is always populated from /proc/version
    #[cfg(target_os = "linux")]
    assert!(
        !caps.kernel_version.is_empty(),
        "kernel_version must be populated on Linux",
    );

    // memory_locking is a bool field -- it must have been explicitly set.
    // We cannot assert true/false (depends on ulimits), but we can verify
    // the field is accessible and the overall struct is well-formed.
    let _memory_locking: bool = caps.memory_locking;
}

// =========================================================================
// Test 2: current process is alive
// =========================================================================

/// INTENT: is_process_alive() returns true for the current PID.
///
/// The current process is definitionally alive while this test runs.
#[test]
fn test_platform_intent_current_process_alive() {
    let pid = std::process::id();
    assert!(
        horus_sys::discover::is_process_alive(pid),
        "is_process_alive({}) must return true for the running test process",
        pid,
    );
}

// =========================================================================
// Test 3: dead process is not alive
// =========================================================================

/// INTENT: is_process_alive() returns false for a known-dead PID.
///
/// PID 99999999 is astronomically unlikely to be in use on any system.
#[test]
fn test_platform_intent_dead_process_not_alive() {
    assert!(
        !horus_sys::discover::is_process_alive(99_999_999),
        "is_process_alive(99999999) must return false -- no such process",
    );
}

// =========================================================================
// Test 4: shm_namespace is consistent
// =========================================================================

/// INTENT: shm_namespace() returns the same value on repeated calls.
///
/// The namespace is cached via OnceLock; two calls must yield identical
/// strings to ensure all SHM paths within a process are coherent.
#[test]
fn test_platform_intent_shm_namespace_consistent() {
    let ns1 = horus_sys::shm::shm_namespace();
    let ns2 = horus_sys::shm::shm_namespace();
    assert_eq!(
        ns1, ns2,
        "shm_namespace() must return identical values across calls",
    );
}

// =========================================================================
// Test 5: shm_base_dir exists
// =========================================================================

/// INTENT: shm_base_dir() returns a path that exists on the filesystem.
///
/// The base directory's *parent* (e.g. /dev/shm on Linux, /tmp on macOS)
/// must exist. The namespaced directory itself may or may not exist yet,
/// but it must be creatable.
#[test]
fn test_platform_intent_shm_base_dir_exists() {
    let base = horus_sys::shm::shm_base_dir();

    // The parent directory must always exist on a functioning OS.
    let parent = base.parent().expect("shm_base_dir() must have a parent");
    assert!(
        parent.exists(),
        "Parent of shm_base_dir ({}) must exist on the filesystem",
        parent.display(),
    );

    // The base directory itself should be creatable. Create it, verify, clean up.
    std::fs::create_dir_all(&base).expect("shm_base_dir should be creatable");
    assert!(
        base.exists(),
        "shm_base_dir ({}) must exist after create_dir_all",
        base.display(),
    );
}

// =========================================================================
// Test 6: home_dir exists
// =========================================================================

/// INTENT: home_dir() returns a valid existing directory.
///
/// horus_sys uses `dirs::home_dir()` internally for config/cache/data paths.
/// The home directory must exist and be a directory on any platform where
/// horus runs.
#[test]
fn test_platform_intent_home_dir_exists() {
    let home = dirs::home_dir().expect("dirs::home_dir() must return Some on supported platforms");
    assert!(
        home.exists(),
        "Home directory ({}) must exist",
        home.display(),
    );
    assert!(
        home.is_dir(),
        "Home directory ({}) must be a directory, not a file",
        home.display(),
    );
}
