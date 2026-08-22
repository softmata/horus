//! `horus param list` must say where the values came from.
//!
//! The footer named `.horus/config/params.yaml` unconditionally:
//!
//! ```text
//!   Total: 13 parameter(s)
//!
//!   Location: Stored in: .horus/config/params.yaml
//! ```
//!
//! Run in a directory with no project it still said that, over thirteen values
//! that came from the built-in defaults — a file that does not exist, credited
//! for values it does not contain. Someone editing that path to change what
//! they just saw would create a file HORUS then layers *over* the defaults,
//! which happens to work, and would be right by accident.
//!
//! Run: `cargo test -p horus_manager --test param_source_contract`

use std::process::Command;

fn param_list(dir: &std::path::Path) -> String {
    let out = Command::new(env!("CARGO_BIN_EXE_horus"))
        .args(["param", "list"])
        .current_dir(dir)
        .output()
        .expect("horus param list must run");
    String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr)
}

#[test]
fn without_a_params_file_the_values_are_named_as_defaults() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let text = param_list(tmp.path());

    assert!(
        text.contains("Built-in defaults"),
        "with no params.yaml the values are defaults and should say so:\n{text}"
    );
    assert!(
        !text.contains("Stored in:"),
        "nothing is stored in a file that does not exist:\n{text}"
    );
}

#[test]
fn with_a_params_file_the_path_is_named() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::create_dir_all(tmp.path().join(".horus/config")).expect("mkdir");
    std::fs::write(
        tmp.path().join(".horus/config/params.yaml"),
        "max_speed: 2.5\n",
    )
    .expect("write");

    let text = param_list(tmp.path());
    assert!(
        text.contains("Stored in:"),
        "the file exists and is what the values came from:\n{text}"
    );
    assert!(!text.contains("Built-in defaults"), "{text}");
}

/// `HORUS_PARAM_*` beats both the file and the defaults, so a value on screen
/// can come from the environment with nothing saying so.
#[test]
fn an_environment_override_is_called_out() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let out = Command::new(env!("CARGO_BIN_EXE_horus"))
        .args(["param", "list"])
        .current_dir(tmp.path())
        .env("HORUS_PARAM_MAX_SPEED", "9.0")
        .output()
        .expect("horus param list must run");
    let text = String::from_utf8_lossy(&out.stdout);

    assert!(
        text.contains("HORUS_PARAM_"),
        "an exported override silently beats the file and the defaults; the \
         listing should say so:\n{text}"
    );
    assert!(text.contains("max_speed"), "{text}");
}
