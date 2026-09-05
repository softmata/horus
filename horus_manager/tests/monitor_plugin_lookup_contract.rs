//! `horus monitor` must look where `horus install` writes.
//!
//! These were two hardcoded paths in two files about 800 lines apart, and they
//! disagreed: `horus install horus-monitor` symlinks into
//! `PluginRegistry::global_bin_dir()` — the platform CONFIG dir — while the
//! lookup checked `~/.horus/bin`, which nothing writes to (`~/.horus` holds
//! installer STATE: `installed_version`, `install_manifest.toml`, `cache/`).
//! So the documented install-then-run flow could not work on any platform, and
//! all the user got was "Monitor plugin is not installed." (#184)
//!
//! `monitor` is a declared clap subcommand, so it never reaches the generic
//! plugin dispatch that resolves this correctly — which is why the drift was
//! invisible to every other plugin command.
//!
//! The fix was to call the accessor instead of restating the path. Nothing
//! pinned that, so this reads the source of `which_monitor_binary` and asserts
//! it. `which_monitor_binary` lives in the binary crate, so an integration test
//! cannot call it; reading it is what is available, and it is enough to catch
//! the one thing that went wrong — a second hardcoded copy of a path.
//!
//! Run: `cargo test -p horus_manager --test monitor_plugin_lookup_contract`

use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).to_path_buf()
}

/// The CODE of `which_monitor_binary`, with comment lines removed.
///
/// Stripping comments is the whole point. The first version of this guard
/// matched the raw body, and the function's own comment explains what
/// `global_bin_dir()` is — so the assertion was satisfied by the prose
/// describing the fix rather than by the call performing it, and would have
/// stayed green if the call were replaced by a hardcoded path with the comment
/// left in place. That is the failure mode a source-reading guard has to be
/// built against.
fn which_monitor_binary_code() -> String {
    which_monitor_binary_body()
        .lines()
        .filter(|l| !l.trim_start().starts_with("//"))
        .collect::<Vec<_>>()
        .join("\n")
}

/// The body of `which_monitor_binary`, from its signature to the matching brace.
fn which_monitor_binary_body() -> String {
    let src = std::fs::read_to_string(repo_root().join("src/main.rs")).expect("main.rs must exist");
    let at = src
        .find("fn which_monitor_binary()")
        .expect("which_monitor_binary must exist; this test would otherwise cover nothing");
    let rest = &src[at..];
    let open = rest.find('{').expect("function body");
    let mut depth = 0usize;
    for (i, ch) in rest[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return rest[open..open + i + 1].to_string();
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced braces in which_monitor_binary");
}

/// The lookup resolves the global plugin directory through the accessor.
#[test]
fn the_monitor_lookup_uses_the_accessor_the_installer_writes_through() {
    let body = which_monitor_binary_code();
    assert!(
        body.contains("global_bin_dir()"),
        "which_monitor_binary must resolve the global plugin directory through \
         PluginRegistry::global_bin_dir() — the same accessor `horus install` \
         links into, `horus remove` unlinks from and `horus plugin trust` \
         resolves against. Restating the path is what let the two drift.\n\
         Body was:\n{body}"
    );
}

/// And it does not restate a home-relative plugin path of its own.
///
/// The project-local `.horus/bin/horus-monitor` check is a different thing and
/// stays: it is CWD-relative and gated behind `HORUS_ALLOW_LOCAL_PLUGINS`. What
/// must not come back is a HOME-relative `~/.horus/bin`, which is the copy that
/// disagreed with the installer.
#[test]
fn the_monitor_lookup_does_not_hardcode_a_home_relative_plugin_dir() {
    let body = which_monitor_binary_body();
    let offenders: Vec<&str> = body
        .lines()
        .filter(|l| !l.trim_start().starts_with("//"))
        .filter(|l| l.contains("home_dir") || l.contains("HOME"))
        .collect();
    assert!(
        offenders.is_empty(),
        "which_monitor_binary must not build a plugin path from the home \
         directory — `~/.horus` holds installer state, not plugin binaries. \
         Offending lines:\n{}",
        offenders.join("\n")
    );
}

/// And the project-local path stays behind its opt-in.
///
/// `.horus/bin/horus-monitor` is CWD-relative. It was once checked
/// unconditionally and BEFORE `PATH`, so `cd`-ing into a cloned repository that
/// ships that file and running `horus monitor` executed it directly — no trust
/// gate, no signature check, none of the sandbox `plugins::executor` applies —
/// and it SHADOWED a properly installed binary. Cloning a repository must not
/// be sufficient to run its code.
///
/// The two tests above would both still pass with the gate deleted: one checks
/// the accessor is used, the other that no HOME-relative path is built, and
/// neither looks at the local branch. So the property is asserted here.
#[test]
fn the_project_local_lookup_stays_gated_behind_the_opt_in() {
    let body = which_monitor_binary_body();

    let mentions_local = body.contains(".horus/bin");
    if !mentions_local {
        // The local branch was removed entirely — the shadowing risk is gone
        // with it, which satisfies the property this test exists to protect.
        return;
    }

    assert!(
        body.contains("HORUS_ALLOW_LOCAL_PLUGINS"),
        "which_monitor_binary still consults the project-local .horus/bin, so \
         that branch must remain gated behind HORUS_ALLOW_LOCAL_PLUGINS. \
         Without the gate, cloning a repository that ships \
         .horus/bin/horus-monitor is enough to execute it, ahead of PATH and \
         ahead of a properly installed binary.\nBody was:\n{body}"
    );
}
