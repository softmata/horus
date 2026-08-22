//! Everything that prints an install command must print one that resolves.
//!
//! TRUST-2 was "README and install.sh disagree on the branch". It was fixed in
//! the six READMEs, in install.sh's own header and in the docs — and reported
//! as "every surface now says raw/main", on the strength of
//! `grep -rn "raw/release"` returning nothing. It did return something:
//!
//!   horus_manager/src/registry/helpers.rs:1065
//!     Install horus from source: curl -fsSL https://github.com/softmata/horus/raw/release/install.sh | bash
//!
//! Live code, not a comment: `inject_horus_path_overrides()` emits it from
//! `precompile_package()` when `find_horus_source_dir()` fails — the one moment
//! the binary itself has to tell a stuck user how to install HORUS. Origin has
//! no `release` branch, so that is the same 404 the finding was about, printed
//! from inside the tool instead of from the README.
//!
//! And the guards that had been added for the README half were deleted a few
//! commits later when `install_contract.rs` was rewritten, so nothing was
//! holding any of it. These are back, and they cover source strings too.

use std::fs;
use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

/// The branch `install.sh` actually clones: `BRANCH="${HORUS_INSTALL_BRANCH:-main}"`.
/// Every URL below is checked against this rather than against a literal, so
/// moving the installer to another branch moves the whole tree with it.
fn installer_branch() -> String {
    let src = fs::read_to_string(repo_root().join("install.sh")).expect("install.sh must exist");
    let line = src
        .lines()
        .find(|l| l.trim_start().starts_with("BRANCH="))
        .expect("install.sh must define BRANCH");
    let at = line
        .find("HORUS_INSTALL_BRANCH:-")
        .expect("BRANCH must have a default");
    let rest = &line[at + "HORUS_INSTALL_BRANCH:-".len()..];
    let branch: String = rest.chars().take_while(|c| *c != '}').collect();
    assert!(
        !branch.is_empty(),
        "install.sh's BRANCH default parsed as empty from: {line}"
    );
    branch
}

const SKIP_DIRS: [&str; 8] = [
    "target",
    ".git",
    "node_modules",
    ".horus",
    "dist",
    "build",
    ".venv",
    "__pycache__",
];

const SCANNED_EXTENSIONS: [&str; 13] = [
    "rs", "md", "mdx", "sh", "ps1", "toml", "yml", "yaml", "json", "py", "cpp", "hpp", "txt",
];

fn scanned_files() -> Vec<PathBuf> {
    fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(dir) else { return };
        for entry in entries.flatten() {
            let path = entry.path();
            let name = entry.file_name().to_string_lossy().to_string();
            if path.is_dir() {
                if SKIP_DIRS.contains(&name.as_str()) {
                    continue;
                }
                walk(&path, out);
            } else if path
                .extension()
                .and_then(|e| e.to_str())
                .is_some_and(|e| SCANNED_EXTENSIONS.contains(&e))
                || name == "Dockerfile"
                || name == "Cargo.lock"
            {
                // Cargo.lock is huge and contains no install URLs; skip it but
                // keep Dockerfile, which has none of the two.
                if name != "Cargo.lock" {
                    out.push(path);
                }
            }
        }
    }
    let mut out = Vec::new();
    walk(&repo_root(), &mut out);
    assert!(
        out.len() > 50,
        "only {} files walked — the scanner is broken and every test here is vacuous",
        out.len()
    );
    out
}

/// Pull the branch segment out of every `softmata/horus/<kind>/<branch>/...`.
fn refs_in(body: &str, kind: &str) -> Vec<String> {
    let needle = format!("softmata/horus/{kind}/");
    let mut out = Vec::new();
    let mut rest = body;
    while let Some(at) = rest.find(&needle) {
        rest = &rest[at + needle.len()..];
        let seg: String = rest
            .chars()
            .take_while(|c| !matches!(c, '/' | '"' | '\'' | ' ' | ')' | '`' | '\n'))
            .collect();
        if !seg.is_empty() {
            out.push(seg);
        }
    }
    out
}

/// The one the audit reported, extended to every file type in the tree.
///
/// A `grep -rn "raw/release"` over the repo was the evidence offered that this
/// was fixed everywhere. It was run and read as returning nothing while the
/// string was still there, which is exactly why this is a test and not a grep.
#[test]
fn every_raw_url_in_the_tree_uses_the_branch_install_sh_clones() {
    let branch = installer_branch();
    let mut wrong: Vec<String> = Vec::new();
    let mut seen = 0usize;

    for file in scanned_files() {
        let Ok(body) = fs::read_to_string(&file) else { continue };
        // This test file names the bad URL on purpose, in the doc comment above.
        if file.ends_with("install_url_contract.rs") {
            continue;
        }
        for got in refs_in(&body, "raw") {
            seen += 1;
            if got != branch {
                wrong.push(format!(
                    "{}: raw/{got}/ (install.sh clones {branch})",
                    file.strip_prefix(repo_root()).unwrap_or(&file).display()
                ));
            }
        }
    }

    assert!(
        seen > 0,
        "no raw.githubusercontent-style URLs found at all — the extractor is broken"
    );
    assert!(
        wrong.is_empty(),
        "these point at a branch install.sh does not clone:\n  {}",
        wrong.join("\n  ")
    );
}

/// `release` has never existed on origin. Not in a raw URL, not in a blob link,
/// not in a source string.
#[test]
fn nothing_references_a_release_branch() {
    let mut hits = Vec::new();
    for file in scanned_files() {
        if file.ends_with("install_url_contract.rs") {
            continue;
        }
        let Ok(body) = fs::read_to_string(&file) else { continue };
        for kind in ["raw", "blob", "tree"] {
            for got in refs_in(&body, kind) {
                if got == "release" {
                    hits.push(format!(
                        "{}: {kind}/release/",
                        file.strip_prefix(repo_root()).unwrap_or(&file).display()
                    ));
                }
            }
        }
    }
    assert!(
        hits.is_empty(),
        "`git ls-remote --heads` lists main, dev and topic branches — there is no \
         release branch, so every one of these is a 404:\n  {}",
        hits.join("\n  ")
    );
}

/// The binary's own "how do I install horus" message is the surface the fix
/// missed. It is emitted by `inject_horus_path_overrides()` when the source tree
/// cannot be found, which is precisely when the user needs it to work.
#[test]
fn the_install_command_the_binary_prints_is_installable() {
    let helpers = fs::read_to_string(repo_root().join("horus_manager/src/registry/helpers.rs"))
        .expect("registry/helpers.rs must exist");
    let branch = installer_branch();

    let refs = refs_in(&helpers, "raw");
    assert!(
        !refs.is_empty(),
        "registry/helpers.rs no longer prints an install URL; if the message moved, \
         move this test with it rather than deleting it"
    );
    for got in refs {
        assert_eq!(
            got, branch,
            "the install command horus prints when it cannot find its source tree \
             points at branch {got:?}, but install.sh clones {branch:?}"
        );
    }
}

/// install.sh's header quotes the command users are told to run. It must match
/// the branch the same file then clones.
#[test]
fn the_installer_header_matches_its_own_default() {
    let src = fs::read_to_string(repo_root().join("install.sh")).expect("install.sh");
    let branch = installer_branch();
    let header: String = src.lines().take_while(|l| !l.starts_with("set -e")).collect::<Vec<_>>().join("\n");
    let refs = refs_in(&header, "raw");
    assert!(
        !refs.is_empty(),
        "install.sh's header no longer shows the one-line install command"
    );
    for got in refs {
        assert_eq!(got, branch, "install.sh's header advertises raw/{got} but clones {branch}");
    }
}

fn readmes() -> Vec<(&'static str, String)> {
    [
        "README.md",
        "README.zh-CN.md",
        "README.pt-BR.md",
        "README.ja.md",
        "README.es.md",
        "README.de.md",
    ]
    .iter()
    .map(|name| {
        (
            *name,
            fs::read_to_string(repo_root().join(name)).unwrap_or_else(|_| panic!("{name} must exist")),
        )
    })
    .collect()
}

/// The English README and all five translations. This test existed, caught the
/// original TRUST-2 drift, and was deleted three commits later.
#[test]
fn every_readme_points_at_the_branch_the_installer_uses() {
    let branch = installer_branch();
    for (name, body) in readmes() {
        let refs = refs_in(&body, "raw");
        assert!(
            !refs.is_empty(),
            "{name} shows no `curl ... install.sh` command at all"
        );
        for got in refs {
            assert_eq!(
                got, branch,
                "{name} tells users to curl raw/{got}/install.sh; install.sh clones {branch}"
            );
        }
    }
}

/// The command has to be the whole command, not just the URL.
#[test]
fn the_primary_readme_advertises_the_install_command() {
    let branch = installer_branch();
    let readme = fs::read_to_string(repo_root().join("README.md")).expect("README.md");
    let expected =
        format!("curl -fsSL https://github.com/softmata/horus/raw/{branch}/install.sh | bash");
    assert!(
        readme.contains(&expected),
        "README.md does not carry the install one-liner verbatim:\n  {expected}"
    );
}
