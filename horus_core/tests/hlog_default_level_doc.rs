//! The `hlog!` doc must not name a default level the constant contradicts.
//!
//! `hlog!`'s doc-comment claimed "The default level is `DEFAULT_LEVEL` (`info`,
//! i.e. `debug` is off)" while `DEFAULT_LEVEL` in the same file was — and still
//! is — `LEVEL_DEBUG`. Three other places in that file say so outright,
//! including the constant's own doc and the test `nothing_is_suppressed_by_default`.
//!
//! That is the doc a node author reads before deciding whether a
//! `hlog!(debug, ...)` in a tick loop is free. It is not: in an unconfigured
//! process every one of them runs `format!`, `chrono::Local::now()` and a
//! bincode serialise into the shared buffer.
//!
//! This guard lives in its own file, not in `hlog.rs`'s test module, so it
//! cannot match its own text.

use std::path::PathBuf;

fn hlog_source() -> String {
    let path: PathBuf = [env!("CARGO_MANIFEST_DIR"), "src", "core", "hlog.rs"]
        .iter()
        .collect();
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

/// The macro doc and the constant must agree about the shipped default.
#[test]
fn the_hlog_macro_doc_agrees_with_default_level() {
    let src = hlog_source();

    let debug_default = src.contains("pub const DEFAULT_LEVEL: u8 = LEVEL_DEBUG;");
    let info_default = src.contains("pub const DEFAULT_LEVEL: u8 = LEVEL_INFO;");
    assert!(
        debug_default ^ info_default,
        "could not find the DEFAULT_LEVEL declaration — has it been renamed or \
         given a different form? This guard reads it literally."
    );

    // Only the `hlog!` macro's own doc block, so the constant's doc (which
    // legitimately discusses `Info` as the conventional-but-rejected default)
    // is not scanned.
    let start = src
        .find("/// # Filtering")
        .expect("the hlog! doc's Filtering section is the anchor for this guard");
    let end = src[start..]
        .find("/// # Example")
        .map(|i| start + i)
        .expect("the Filtering section should be followed by Example");
    let filtering = &src[start..end];

    if debug_default {
        assert!(
            filtering.contains("it is `debug`"),
            "DEFAULT_LEVEL is LEVEL_DEBUG but the hlog! doc does not say so:\n{filtering}"
        );
        assert!(
            !filtering.contains("(`info`, i.e. `debug`\n/// is off)"),
            "DEFAULT_LEVEL is LEVEL_DEBUG and the hlog! doc still claims `info`, \
             debug off — the exact claim this guard exists for:\n{filtering}"
        );
    } else {
        assert!(
            !filtering.contains("it is `debug`"),
            "DEFAULT_LEVEL is LEVEL_INFO but the hlog! doc still says `debug`:\n{filtering}"
        );
    }
}
