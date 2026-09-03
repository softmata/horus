//! A malformed `.horus/config/params.yaml` must stop a C++ node from starting.
//!
//! Run: cargo test --no-default-features -p horus_cpp --test params_malformed_yaml
//!
//! Its own test binary on purpose: `RuntimeParams::new` resolves the parameter
//! file relative to the process working directory, and `set_current_dir` is
//! process-global, so this cannot share a binary with tests running in parallel
//! next to it. Alone in a binary it owns the process cwd outright, and no
//! ordering guarantee from the test runner is needed for that: every other test
//! binary is a separate process with its own cwd, so they cannot see this one's
//! chdir whether the runner interleaves them or not.

use horus_cpp::{params_get_f64, params_new};
use std::path::PathBuf;

/// Enters a scratch directory and restores the process cwd on the way out, even
/// if the test panics while inside it. Removing the directory afterwards is
/// best effort; the cwd restore is not — see `Drop`.
///
/// Order matters on drop: the cwd is restored BEFORE the directory is removed,
/// so the process is never left sitting in a deleted working directory, where
/// every later relative path in this process fails.
struct ScratchCwd {
    previous: PathBuf,
    dir: PathBuf,
}

impl ScratchCwd {
    fn enter(dir: PathBuf) -> Self {
        let previous = std::env::current_dir().expect("current working directory is readable");
        std::env::set_current_dir(&dir)
            .unwrap_or_else(|e| panic!("cannot enter {}: {e}", dir.display()));
        Self { previous, dir }
    }
}

impl Drop for ScratchCwd {
    fn drop(&mut self) {
        let restored = std::env::set_current_dir(&self.previous);

        // Removing the scratch directory stays best effort. It is named per
        // pid and nanosecond, so a leaked one is inert and never adopted by a
        // later run; failing the test over it would turn an already-decided
        // result red for a reason the test does not assert.
        let _ = std::fs::remove_dir_all(&self.dir);

        // The cwd is not best effort. A silent failure here leaves the whole
        // process in the scratch directory — deleted, after the line above —
        // and every relative path the rest of this binary touches then fails
        // somewhere far from the cause. That is exactly the confusing
        // follow-on failure this guard exists to prevent, so say it here.
        // Not while unwinding, though: a panic during a panic aborts the
        // process and buries the assertion failure that started it.
        if !std::thread::panicking() {
            restored.unwrap_or_else(|e| {
                panic!(
                    "could not restore the working directory to {}: {e}",
                    self.previous.display()
                )
            });
        }
    }
}

/// A directory this run has to itself. The pid separates concurrent instances
/// of this binary; the timestamp additionally separates this run from a stale
/// directory an earlier killed run left behind under a since-recycled pid,
/// which `create_dir_all` would otherwise silently adopt.
fn scratch_dir() -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    std::env::temp_dir().join(format!(
        "horus_cpp_params_malformed_{}_{nanos}",
        std::process::id()
    ))
}

#[test]
fn params_new_refuses_malformed_params_yaml() {
    let dir = scratch_dir();
    let config = dir.join(".horus").join("config");
    std::fs::create_dir_all(&config).unwrap();
    // Unterminated flow sequence: serde_yaml rejects it, and it is the shape of
    // typo an operator makes editing limits by hand.
    std::fs::write(config.join("params.yaml"), "max_speed: [1.0\n").unwrap();

    let params = {
        let _cwd = ScratchCwd::enter(dir.clone());
        params_new()
    }; // cwd restored and `dir` removed here, panic or not.

    // Name what the caller would otherwise have been handed: the built-in
    // `max_speed` of 1.0, which is looser than whatever the operator wrote.
    let served = params.as_ref().and_then(|p| params_get_f64(p, "max_speed"));
    assert!(
        params.is_none(),
        "params_new built a store from a malformed params.yaml, serving \
         max_speed={served:?} from the built-in safety limits. Rust \
         (RuntimeParams::new) and Python (Params()) both refuse to start here; \
         the C++ bindings must too."
    );
}
