//! No shipped code may point at an endpoint that is known not to answer.
//!
//! Five hosts were compiled into this crate at once, and every one of them was
//! dead:
//!
//! | reference | state |
//! |---|---|
//! | `api.horusrobotics.dev` | HTTP 503, suspended |
//! | `plugins.horusrobotics.dev` | no DNS record |
//! | `status.horusrobotics.dev` | no DNS record, printed as the "check here" hint on a 5xx |
//! | `horusrobotics.dev/install` | 404, printed as the "reinstall like this" hint |
//! | `horusrobotics.dev/api/packages/<n>/latest` | 404, so plugin upgrade checks never fired |
//!
//! None of them failed loudly. A dead URL in a *hint* is invisible until a user
//! is already stuck and follows the advice; a dead URL in a *default* is worse,
//! because it makes an outage look like an empty ecosystem.
//!
//! This is a contract test rather than a network test on purpose: it must stay
//! green with no network at all, and it must fail on a machine where the host
//! happens to resolve. What it enforces is "the tree does not name a host we
//! have established is dead" — restoring one means deleting its row here, in the
//! same commit that makes it answer.
//!
//! Comments are stripped before scanning. Explaining *why* an endpoint is not
//! used is exactly what the surrounding code should do, and a test that
//! punished those comments would push the explanation out of the codebase.
//!
//! Tracked in <https://github.com/softmata/horus/issues/173>.

use std::fs;
use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

/// Hosts and paths established as non-answering, with the evidence.
const DEAD_ENDPOINTS: [(&str, &str); 5] = [
    (
        "api.horusrobotics.dev",
        "HTTP 503 'This service has been suspended by its owner'",
    ),
    ("plugins.horusrobotics.dev", "no DNS record"),
    ("status.horusrobotics.dev", "no DNS record"),
    ("horusrobotics.dev/install", "HTTP 404"),
    (
        "horusrobotics.dev/api/",
        "HTTP 404 — the apex serves no API",
    ),
];

const SKIP_DIRS: [&str; 7] = [
    "target",
    ".git",
    "node_modules",
    ".horus",
    "dist",
    "build",
    ".venv",
];

/// Source files that actually ship behaviour. Docs and changelogs are excluded:
/// a changelog entry naming the endpoint it removed is a correct changelog.
const SCANNED_EXTENSIONS: [&str; 4] = ["rs", "sh", "ps1", "toml"];

fn scanned_files() -> Vec<PathBuf> {
    fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(dir) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let name = entry.file_name().to_string_lossy().to_string();
            if path.is_dir() {
                if !SKIP_DIRS.contains(&name.as_str()) {
                    walk(&path, out);
                }
            } else if path
                .extension()
                .and_then(|e| e.to_str())
                .is_some_and(|e| SCANNED_EXTENSIONS.contains(&e))
                && name != "Cargo.lock"
            {
                out.push(path);
            }
        }
    }
    let mut out = Vec::new();
    walk(&repo_root(), &mut out);
    assert!(
        out.len() > 50,
        "only {} files walked — the scanner is broken and this test is vacuous",
        out.len()
    );
    out
}

/// Drop comment lines so an explanation of a dead endpoint is not itself a
/// violation. Line-oriented and deliberately conservative: it removes only
/// whole lines that are unambiguously comments.
fn strip_comments(body: &str, ext: &str) -> String {
    body.lines()
        .filter(|line| {
            let t = line.trim_start();
            match ext {
                "rs" => !(t.starts_with("//") || t.starts_with("* ") || t.starts_with("*/")),
                "sh" | "ps1" | "toml" => !t.starts_with('#'),
                _ => true,
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn no_shipped_code_references_a_dead_endpoint() {
    let mut hits: Vec<String> = Vec::new();

    for file in scanned_files() {
        // This file names all of them on purpose, in its table above.
        if file.ends_with("registry_endpoint_contract.rs") {
            continue;
        }
        let Ok(body) = fs::read_to_string(&file) else {
            continue;
        };
        let ext = file.extension().and_then(|e| e.to_str()).unwrap_or("");
        let code = strip_comments(&body, ext);

        for (endpoint, why) in DEAD_ENDPOINTS {
            if code.contains(endpoint) {
                let rel = file.strip_prefix(repo_root()).unwrap_or(&file);
                hits.push(format!("{}: {endpoint} ({why})", rel.display()));
            }
        }
    }

    assert!(
        hits.is_empty(),
        "these are live references to endpoints that do not answer:\n  {}\n\n\
         If one of them is back up, delete its row from DEAD_ENDPOINTS in this \
         file in the same commit — do not silence it here while it is still \
         returning an error.",
        hits.join("\n  ")
    );
}

/// The scanner must be able to see a violation at all.
///
/// `no_shipped_code_references_a_dead_endpoint` passes both when the tree is
/// clean and when `strip_comments` accidentally eats everything. This tells
/// those apart.
#[test]
fn the_scanner_detects_a_planted_reference() {
    let planted = "let url = \"https://api.horusrobotics.dev/api/packages\";";
    let code = strip_comments(planted, "rs");
    assert!(
        code.contains("api.horusrobotics.dev"),
        "a bare code line must survive comment stripping"
    );

    let commented = "// api.horusrobotics.dev is suspended, so we do not use it";
    assert!(
        !strip_comments(commented, "rs").contains("api.horusrobotics.dev"),
        "an explanatory comment must NOT count as a violation"
    );

    let shell_comment = "#   https://horusrobotics.dev/install used to be advertised";
    assert!(
        !strip_comments(shell_comment, "sh").contains("horusrobotics.dev/install"),
        "a shell comment must NOT count as a violation"
    );
}

/// The default registry must never be a URL that is known to be dead.
///
/// Distinct from the scan above: this reads the value the binary would actually
/// use, so it still bites if the constant is moved or renamed.
#[test]
fn the_compiled_default_registry_is_not_a_dead_host() {
    if let Some(url) = horus_manager::config::DEFAULT_REGISTRY_URL {
        for (endpoint, why) in DEAD_ENDPOINTS {
            assert!(
                !url.contains(endpoint),
                "DEFAULT_REGISTRY_URL is {url}, which contains {endpoint} ({why})"
            );
        }
    }
    if let Some(url) = horus_manager::config::DEFAULT_PLUGIN_REGISTRY_URL {
        for (endpoint, why) in DEAD_ENDPOINTS {
            assert!(
                !url.contains(endpoint),
                "DEFAULT_PLUGIN_REGISTRY_URL is {url}, which contains {endpoint} ({why})"
            );
        }
    }
}
