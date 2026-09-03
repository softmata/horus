//! A malformed `.horus/config/params.yaml` must stop a C++ node from starting.
//!
//! Run: cargo test --no-default-features -p horus_cpp --test params_malformed_yaml
//!
//! Its own test binary on purpose: `RuntimeParams::new` resolves the parameter
//! file relative to the process working directory, and `set_current_dir` is
//! process-global, so this cannot share a binary with tests running in parallel
//! next to it.

use horus_cpp::{params_get_f64, params_new};

#[test]
fn params_new_refuses_malformed_params_yaml() {
    let dir =
        std::env::temp_dir().join(format!("horus_cpp_params_malformed_{}", std::process::id()));
    let config = dir.join(".horus").join("config");
    std::fs::create_dir_all(&config).unwrap();
    // Unterminated flow sequence: serde_yaml rejects it, and it is the shape of
    // typo an operator makes editing limits by hand.
    std::fs::write(config.join("params.yaml"), "max_speed: [1.0\n").unwrap();

    let previous = std::env::current_dir().unwrap();
    std::env::set_current_dir(&dir).unwrap();
    let params = params_new();
    std::env::set_current_dir(&previous).unwrap();
    let _ = std::fs::remove_dir_all(&dir);

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
