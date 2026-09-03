//! A malformed `.horus/config/params.yaml` must stop a C++ node from starting.
//!
//! Run: cargo test --no-default-features -p horus_cpp --test params_malformed_yaml
//!
//! Its own test binary on purpose: `RuntimeParams::new` resolves the parameter
//! file relative to the process working directory, and `set_current_dir` is
//! process-global, so this cannot share a binary with tests running in parallel
//! next to it. Cargo runs test binaries one at a time, so this one owns the
//! process cwd for as long as it holds it.

use horus_cpp::{params_get_f64, params_new};
use std::path::PathBuf;

/// Enters a scratch directory and guarantees the process cwd is restored, and
/// the directory removed, even if the test panics while inside it.
///
/// Order matters on drop: the cwd is restored BEFORE the directory is removed,
/// so the process is never left sitting in a deleted working directory, where
/// every later relative path — including the next test binary's — fails.
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
        let _ = std::env::set_current_dir(&self.previous);
        let _ = std::fs::remove_dir_all(&self.dir);
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
