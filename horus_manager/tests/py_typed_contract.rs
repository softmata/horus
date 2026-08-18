//! The Python package must ship its PEP 561 marker.
//!
//! `horus_py` is a pyo3 extension with a hand-written `_horus.pyi` stub — 1,100
//! lines of type information that no type checker could see, because the
//! package had no `py.typed` marker. PEP 561 says a checker must ignore types
//! from an installed package that does not carry one, and mypy did exactly
//! that:
//!
//! ```text
//! $ mypy t.py
//! t.py:1: error: Skipping analyzing "horus": module is installed, but missing
//!     library stubs or py.typed marker  [import-untyped]
//! Success: no issues found in 1 source file
//! ```
//!
//! "Success" — while the file it just checked called `n.nonexistent_method()`
//! and `horus.completely_fake_function()`. Both were invisible. Python users
//! got no editor completion and no type errors from a framework that had
//! written the types out in full.
//!
//! With the marker in place and the wheel reinstalled:
//!
//! ```text
//! t.py:3: error: "Node" has no attribute "nonexistent_method"  [attr-defined]
//! t.py:4: error: Module has no attribute "completely_fake_function"  [attr-defined]
//! ```
//!
//! Two things have to hold, and both are checked here: the marker exists in the
//! source tree, and `pyproject.toml` tells maturin to put it in the wheel. A
//! marker that is present in git and absent from the wheel helps nobody.
//!
//! Run: `cargo test -p horus_manager --test py_typed_contract`

use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent")
        .to_path_buf()
}

fn pyproject() -> String {
    std::fs::read_to_string(repo_root().join("horus_py/pyproject.toml"))
        .expect("horus_py/pyproject.toml must exist")
}

#[test]
fn the_py_typed_marker_exists() {
    let marker = repo_root().join("horus_py/horus/py.typed");
    assert!(
        marker.is_file(),
        "{} is missing. Without it, PEP 561 requires type checkers to ignore \
         _horus.pyi entirely, so the shipped stubs do nothing.",
        marker.display()
    );
}

/// The marker is defined by its presence, not its contents; an empty file is
/// correct. This checks it is not accidentally holding something.
#[test]
fn the_marker_is_empty_as_the_spec_expects() {
    let marker = repo_root().join("horus_py/horus/py.typed");
    let contents = std::fs::read_to_string(&marker).unwrap_or_default();
    assert!(
        contents.trim().is_empty(),
        "py.typed should be empty; found: {contents:?}"
    );
}

/// Existing in the repo is not enough — maturin only packages what it is told
/// to. Without this line the wheel ships without the marker and the bug is
/// back, invisibly.
#[test]
fn maturin_is_told_to_include_the_marker() {
    let text = pyproject();
    assert!(
        text.contains("horus/py.typed"),
        "pyproject.toml does not list horus/py.typed in [tool.maturin].include, \
         so the built wheel will not contain it:\n{text}"
    );
}

/// The stub is the payload the marker unlocks; shipping one without the other
/// is useless in either direction.
#[test]
fn maturin_is_told_to_include_the_stub() {
    let text = pyproject();
    assert!(
        text.contains("horus/_horus.pyi"),
        "pyproject.toml does not list horus/_horus.pyi in [tool.maturin].include"
    );
}

#[test]
fn the_stub_file_actually_exists() {
    let stub = repo_root().join("horus_py/horus/_horus.pyi");
    assert!(
        stub.is_file(),
        "{} is missing, so the py.typed marker promises types that are not there",
        stub.display()
    );
}

/// `include` must be an array. An earlier attempt wrote it as a
/// `[tool.maturin.include]` table, which parses as valid TOML and silently
/// includes nothing.
#[test]
fn the_include_key_is_an_array_not_a_table() {
    let text = pyproject();
    assert!(
        !text.contains("[tool.maturin.include]"),
        "`[tool.maturin.include]` is a table; maturin expects `include = [...]`. \
         The table form is valid TOML, so nothing errors — the files just are \
         not packaged."
    );
    assert!(
        text.contains("include = ["),
        "expected `include = [...]` under [tool.maturin]:\n{text}"
    );
}
