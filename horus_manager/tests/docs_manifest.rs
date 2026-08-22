//! Manifest and environment contracts — the `horus.toml` keys and `HORUS_*`
//! variables the documentation tells readers to use must actually work.
//!
//! # Why these two together
//!
//! Both are *configuration* promises, and both fail silently in a way code
//! examples never do. A wrong TOML key does not raise; it is either rejected
//! wholesale (taking the entire manifest with it) or quietly dropped, so the
//! reader's setting simply has no effect and nothing says why. A documented
//! environment variable that no code reads behaves identically.
//!
//! Confirmed instances this suite locks down:
//!
//! * `tutorials/06-write-a-driver.mdx:56` documents `[package] type = "driver"`.
//!   `type` deserializes into `TargetType`, which accepts only `bin`/`lib`/`both`,
//!   so the value makes the *whole* manifest unparseable — `horus check`,
//!   `horus run` and `horus publish` all fail on a project that followed the page.
//! * The same page documents `[package] keywords`, which is not a manifest field.
//!   `PackageInfo` carries no `deny_unknown_fields`, so it is silently ignored
//!   while the publishing checklist implies it feeds registry search.
//! * `getting-started/quick-start-python.mdx:168` cites `HORUS_PROJECT_DIR` and
//!   `getting-started/migrating-to-horus-toml.mdx:420` cites `HORUS_NO_PROXY=1`.
//!   Neither appears anywhere in the source, so reader code reading them gets
//!   nothing.
//!
//! Hermetic and fast (no compilation, no network); runs on every PR.

use assert_cmd::cargo::cargo_bin_cmd;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

// ─── Locating things ────────────────────────────────────────────────────────

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn docs_dir() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("HORUS_DOCS_DIR") {
        let p = PathBuf::from(p);
        return p.join("content/docs").is_dir().then_some(p);
    }
    let sibling = repo_root().parent()?.join("horus-docs");
    sibling.join("content/docs").is_dir().then_some(sibling)
}

fn doc_files(docs: &Path) -> Vec<PathBuf> {
    fn walk(dir: &Path, acc: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                walk(&p, acc);
            } else if p.extension().is_some_and(|x| x == "mdx" || x == "md") {
                acc.push(p);
            }
        }
    }
    let mut v = Vec::new();
    walk(&docs.join("content/docs"), &mut v);
    v.sort();
    v
}

fn rel(docs: &Path, f: &Path) -> String {
    f.strip_prefix(docs)
        .unwrap_or(f)
        .to_string_lossy()
        .replace('\\', "/")
}

// ─── TOML extraction ────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
struct TomlBlock {
    doc_file: String,
    line: usize,
    code: String,
}

fn toml_blocks(text: &str, rel: &str) -> Vec<TomlBlock> {
    let mut out = Vec::new();
    let mut cur: Option<(usize, String)> = None;
    for (idx, raw) in text.lines().enumerate() {
        if let Some(rest) = raw.trim_start().strip_prefix("```") {
            match cur.take() {
                Some((start, code)) => out.push(TomlBlock {
                    doc_file: rel.to_string(),
                    line: start,
                    code,
                }),
                None => {
                    let lang = rest
                        .trim()
                        .split([':', ' ', ','])
                        .next()
                        .unwrap_or("")
                        .to_ascii_lowercase();
                    if lang == "toml" {
                        cur = Some((idx + 1, String::new()));
                    }
                }
            }
            continue;
        }
        if let Some((_, code)) = cur.as_mut() {
            code.push_str(raw);
            code.push('\n');
        }
    }
    // MDX indents fences nested in JSX; TOML tolerates it but the parser is
    // happier without, and it keeps error columns meaningful.
    for b in &mut out {
        b.code = dedent(&b.code);
    }
    out
}

fn dedent(code: &str) -> String {
    let indent = code
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| l.len() - l.trim_start().len())
        .min()
        .unwrap_or(0);
    if indent == 0 {
        return code.to_string();
    }
    code.lines()
        .map(|l| {
            if l.len() >= indent {
                &l[indent..]
            } else {
                l.trim_start()
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// A block that is a whole `horus.toml`, not a fragment of one.
///
/// The docs also print bare `[dependencies]` excerpts and `Cargo.toml` samples;
/// only a block with a `[package]` carrying `name` and `version` is a manifest a
/// reader could actually save.
fn is_full_manifest(code: &str) -> bool {
    code.contains("[package]")
        && code.contains("name")
        && code.contains("version")
        && !code.contains("...")
        && !code.contains("<your")
        // Cargo manifests appear on the migration pages for contrast, and on
        // the plugin pages as the plugin's own `Cargo.toml` — a plugin is a
        // Rust binary, so "Step 2: Set Up Cargo.toml" prints one. `[lib]` and
        // `crate-type` caught the migration pages; `[[bin]]` and `edition` are
        // what the plugin pages use, and both blocks were being written to a
        // `horus.toml` and handed to `horus check`. That reported the plugin
        // guide as documenting an unloadable project when what it documents is
        // a correct Cargo manifest. None of these four keys exists in
        // `KNOWN_TOP_LEVEL`, so a block carrying one is a Cargo manifest.
        && !code.contains("[lib]")
        && !code.contains("crate-type")
        && !code.contains("[[bin]]")
        && !code.contains("edition")
}

// ─── Tests ──────────────────────────────────────────────────────────────────

/// Every complete `horus.toml` printed in the docs must satisfy `horus check`.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn documented_manifests_are_valid() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let mut manifests = Vec::new();
    for f in doc_files(&docs) {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        let r = rel(&docs, &f);
        manifests.extend(
            toml_blocks(&text, &r)
                .into_iter()
                .filter(|b| is_full_manifest(&b.code)),
        );
    }
    assert!(
        !manifests.is_empty(),
        "extracted no complete horus.toml blocks — the extractor is broken, \
         which would make this test vacuous"
    );
    eprintln!(
        "complete horus.toml blocks in the docs: {}",
        manifests.len()
    );

    let tmp = tempfile::tempdir().expect("temp dir");
    let mut failures = Vec::new();
    for (i, m) in manifests.iter().enumerate() {
        let proj = tmp.path().join(format!("m{i}"));
        std::fs::create_dir_all(proj.join("src")).unwrap();
        std::fs::write(proj.join("horus.toml"), &m.code).unwrap();
        std::fs::write(proj.join("src/main.rs"), "fn main() {}\n").unwrap();

        let out = cargo_bin_cmd!("horus")
            .args(["check", "."])
            .current_dir(&proj)
            .output()
            .expect("horus check runs");
        if !out.status.success() {
            let combined = format!(
                "{}{}",
                String::from_utf8_lossy(&out.stdout),
                String::from_utf8_lossy(&out.stderr)
            );
            let detail = combined
                .lines()
                .find(|l| l.to_ascii_lowercase().contains("error") || l.contains("invalid"))
                .unwrap_or_else(|| combined.lines().last().unwrap_or(""))
                .trim()
                .to_string();
            failures.push(format!("  {}:{}\n      {detail}", m.doc_file, m.line));
        }
    }

    assert!(
        failures.is_empty(),
        "{} documented horus.toml block(s) are rejected by `horus check`:\n{}\n\n\
         A reader who saves one of these has a project no horus command can load. \
         Fix the doc page or accept the key.",
        failures.len(),
        failures.join("\n")
    );
    eprintln!("all {} documented manifests validate", manifests.len());
}

/// Keys the docs put under `[package]` must be keys the manifest actually reads.
///
/// `PackageInfo` has no `deny_unknown_fields`, so an unknown key parses cleanly
/// and is then discarded — the reader's setting has no effect and nothing warns
/// them. `keywords` is documented on the driver-publishing page as if it feeds
/// registry search; the real field is `categories`.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn documented_package_keys_are_real() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let known = package_info_fields();
    assert!(
        known.contains("name") && known.contains("version") && known.len() >= 8,
        "parsed only {:?} out of PackageInfo — the reader is broken, making this vacuous",
        known
    );

    let mut used: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for f in doc_files(&docs) {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        let r = rel(&docs, &f);
        for b in toml_blocks(&text, &r) {
            if !b.code.contains("[package]") {
                continue;
            }
            for key in keys_in_section(&b.code, "package") {
                used.entry(key)
                    .or_default()
                    .push(format!("{}:{}", b.doc_file, b.line));
            }
        }
    }
    assert!(
        !used.is_empty(),
        "extracted no [package] keys — extractor broken"
    );
    eprintln!("distinct [package] keys used in the docs: {}", used.len());

    let unknown: Vec<String> = used
        .iter()
        .filter(|(k, _)| !known.contains(k.as_str()))
        .map(|(k, sites)| {
            format!(
                "  [package] {k}  — documented at {}",
                sites.iter().take(2).cloned().collect::<Vec<_>>().join(", ")
            )
        })
        .collect();

    assert!(
        unknown.is_empty(),
        "documented [package] keys the manifest does not read:\n{}\n\n\
         PackageInfo does not deny unknown fields, so these parse and are then \
         thrown away — the reader's setting silently does nothing. Known keys: \
         {known:?}",
        unknown.join("\n")
    );
}

/// Every `HORUS_*` variable the docs tell a reader to set must be read somewhere.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn documented_env_vars_are_read() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let mut cited: BTreeMap<String, Vec<String>> = BTreeMap::new();

    // Names a CI example defines for itself. `HORUS_REGISTRY_KEY:
    // ${{ secrets.HORUS_REGISTRY_KEY }}` in a GitHub Actions block is a secret
    // the *reader* names, passed to a shell that writes ~/.horus/auth.json — it
    // is not a variable horus reads, so requiring the source to mention it is
    // backwards. Collected first so every citation of such a name is ignored.
    let mut ci_secrets: BTreeSet<String> = BTreeSet::new();
    for f in doc_files(&docs) {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        let lines: Vec<&str> = text.lines().collect();
        for (i, line) in lines.iter().enumerate() {
            if !line.contains("secrets.") {
                continue;
            }
            let prev = if i > 0 { lines[i - 1] } else { "" };
            for name in documented_env_names(prev, line) {
                ci_secrets.insert(name);
            }
        }
    }

    for f in doc_files(&docs) {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        let r = rel(&docs, &f);
        let lines: Vec<&str> = text.lines().collect();
        // A page may list the variables HORUS sets for itself, or that its own
        // test suite reads, precisely so a reader who meets one can recognise
        // it. `consumer_sources` deliberately skips `tests/` so this suite
        // cannot satisfy its own assertions, which means a genuinely test-only
        // variable looks unread. Naming a section "internal" or "test-only" is
        // the page saying "do not set these" — take it at its word rather than
        // demanding the runtime implement them.
        let mut in_internal_section = false;
        for (i, line) in lines.iter().enumerate() {
            if let Some(h) = line.strip_prefix('#') {
                let h = h.trim_start_matches('#').trim().to_ascii_lowercase();
                in_internal_section = h.contains("internal") || h.contains("test-only");
            }
            if in_internal_section {
                continue;
            }
            let prev = if i > 0 { lines[i - 1] } else { "" };
            for name in documented_env_names(prev, line) {
                if ci_secrets.contains(&name) {
                    continue;
                }
                cited
                    .entry(name)
                    .or_default()
                    .push(format!("{}:{}", r, i + 1));
            }
        }
    }
    assert!(
        !cited.is_empty(),
        "found no HORUS_* variables in the docs — the scanner is broken"
    );
    eprintln!(
        "distinct HORUS_* variables cited in the docs: {}",
        cited.len()
    );

    let sources = consumer_sources(&repo_root());
    assert!(
        sources.len() > 100,
        "only {} source files found — the walker is broken, making this vacuous",
        sources.len()
    );
    let haystack: String = sources
        .iter()
        .filter_map(|p| std::fs::read_to_string(p).ok())
        .collect();
    let prefixes = dynamic_prefixes(&haystack);

    let missing: Vec<String> = cited
        .iter()
        .filter(|(name, _)| {
            !haystack.contains(name.as_str())
                && !prefixes.iter().any(|p| name.starts_with(p.as_str()))
        })
        .map(|(name, sites)| {
            format!(
                "  {name}  — documented at {}",
                sites.iter().take(2).cloned().collect::<Vec<_>>().join(", ")
            )
        })
        .collect();

    assert!(
        missing.is_empty(),
        "documented HORUS_* variables that nothing in the source reads:\n{}\n\n\
         Setting one of these has no effect whatsoever, and the reader gets no \
         warning. Implement it or correct the doc page.",
        missing.join("\n")
    );
}

// ─── Helpers ────────────────────────────────────────────────────────────────

/// Field names serde will accept under `[package]`, read from `manifest.rs` so
/// the list cannot drift away from the struct it describes.
fn package_info_fields() -> BTreeSet<String> {
    let src =
        std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/manifest.rs"))
            .expect("manifest.rs is readable");
    let start = src
        .find("pub struct PackageInfo")
        .expect("PackageInfo still exists");
    let body = &src[start..];
    let end = body.find("\n}").unwrap_or(body.len());
    let body = &body[..end];

    let mut out = BTreeSet::new();
    for line in body.lines() {
        let t = line.trim();
        // An explicit rename wins over the field name.
        if let Some(r) = t.find("rename = \"") {
            if let Some(name) = t[r + "rename = \"".len()..].split('"').next() {
                out.insert(name.to_string());
                continue;
            }
        }
        if let Some(rest) = t.strip_prefix("pub ") {
            if let Some(name) = rest.split(':').next() {
                let name = name.trim();
                if !name.is_empty() && name.chars().all(|c| c.is_alphanumeric() || c == '_') {
                    out.insert(name.to_string());
                }
            }
        }
    }
    out
}

/// Top-level keys assigned inside `[section]` of a TOML document.
fn keys_in_section(code: &str, section: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut inside = false;
    for line in code.lines() {
        let t = line.trim();
        if t.starts_with('[') {
            inside = t == format!("[{section}]");
            continue;
        }
        if !inside || t.is_empty() || t.starts_with('#') {
            continue;
        }
        if let Some(eq) = t.find('=') {
            let key = t[..eq].trim().trim_matches('"');
            if !key.is_empty()
                && key
                    .chars()
                    .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
            {
                out.push(key.to_string());
            }
        }
    }
    out
}

/// Conventional Rust variables the docs tell readers to set that HORUS does
/// **not** honour.
///
/// `RUST_LOG` is the one that matters: horus installs its own log bridge which
/// "replaces env_logger" (horus_manager/src/main.rs:1944) and takes verbosity
/// from `-v`/`-q`, so `RUST_LOG=debug horus run …` silently does nothing. Three
/// doc sites recommended it. Scanning only `HORUS_*` missed the whole class.
///
/// `RUST_BACKTRACE` is deliberately absent — the standard library's panic
/// runtime reads it, so it works regardless of what horus does.
const FOREIGN_ENV_NAMES: &[&str] = &["RUST_LOG"];

/// Every environment variable name a doc line mentions that horus is expected
/// to honour: `HORUS_*` plus [`FOREIGN_ENV_NAMES`].
fn documented_env_names(prev: &str, line: &str) -> Vec<String> {
    // A line that names a variable in order to say it does NOT work is good
    // documentation — a reader who tries `RUST_LOG` and searches for it should
    // find the answer. Only lines that tell the reader to *use* a variable are
    // promises the source has to keep.
    //
    // The disclaimer routinely wraps across a line break:
    //
    //     HORUS reads the key from `auth.json` and nowhere else — there is no
    //     `HORUS_API_KEY` environment variable.
    //
    // so the negation must be looked for over the previous line too, not just
    // the one the name appears on.
    if is_negated_mention(line) || is_negated_mention(&format!("{prev} {line}")) {
        return Vec::new();
    }
    let mut out = horus_env_names(line);
    for name in FOREIGN_ENV_NAMES {
        if line.contains(name) && !out.iter().any(|n| n == name) {
            out.push((*name).to_string());
        }
    }
    out
}

/// Whether a line mentions a variable only to disclaim it.
fn is_negated_mention(line: &str) -> bool {
    let mut l = line.to_ascii_lowercase();
    // Writers naturally slip an adverb into the disclaimer — "there is
    // *deliberately* no HORUS_API_KEY" — which defeats a plain substring match
    // and makes the suite report a correctly-documented non-variable as a
    // defect. Drop the adverbs before matching so the phrase list stays short.
    for adverb in [
        " deliberately",
        " intentionally",
        " explicitly",
        " currently",
        " actually",
        " simply",
        " ever",
    ] {
        if l.contains(adverb) {
            l = l.replace(adverb, "");
        }
    }
    [
        "no effect",
        "has no",
        "does not",
        "doesn't",
        "not supported",
        "is ignored",
        "there is no",
        "there's no",
        "no such",
        "nothing reads",
        "nothing in horus reads",
        "instead of",
        "in place of",
    ]
    .iter()
    .any(|p| l.contains(p))
}

/// `HORUS_*` identifiers mentioned in a line of documentation.
fn horus_env_names(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let chars: Vec<char> = line.chars().collect();
    let needle: Vec<char> = "HORUS_".chars().collect();
    let mut i = 0;
    while i + needle.len() <= chars.len() {
        if chars[i..i + needle.len()] != needle[..] {
            i += 1;
            continue;
        }
        // Must not be part of a longer identifier (e.g. `MY_HORUS_X`).
        if i > 0 && (chars[i - 1].is_alphanumeric() || chars[i - 1] == '_') {
            i += 1;
            continue;
        }
        let mut j = i;
        while j < chars.len()
            && (chars[j].is_ascii_uppercase() || chars[j].is_ascii_digit() || chars[j] == '_')
        {
            j += 1;
        }
        let name: String = chars[i..j].iter().collect();
        // `HORUS_` alone, or a trailing underscore, is prose rather than a name.
        if name.len() > "HORUS_".len() && !name.ends_with('_') && !out.contains(&name) {
            out.push(name);
        }
        i = j.max(i + 1);
    }
    out
}

/// Every file that could plausibly *consume* a `HORUS_*` name.
///
/// Two exclusions matter, and both were found by this test reporting nonsense:
///
/// * `tests/` is skipped. This very file names `HORUS_PROJECT_DIR` and
///   `HORUS_NO_PROXY` in its module docs as examples of variables nothing reads
///   — which put them in the haystack and silently suppressed the finding. A
///   test that cites a defect must not thereby hide it.
/// * C++ headers are included. `HORUS_TOPIC_IMPL` is a macro in
///   `horus_cpp/include/horus/impl/topic_impl.hpp`, not an environment variable
///   at all, and scanning only Rust reported it as missing.
fn consumer_sources(root: &Path) -> Vec<PathBuf> {
    fn walk(dir: &Path, acc: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            let name = p
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .to_string();
            if p.is_dir() {
                // `tests` is excluded so this suite cannot satisfy its own
                // assertions by writing about them.
                if name != "target" && name != "tests" && !name.starts_with('.') {
                    walk(&p, acc);
                }
            } else if p.extension().is_some_and(|x| {
                matches!(
                    x.to_string_lossy().as_ref(),
                    "rs" | "toml" | "sh" | "h" | "hpp" | "hh" | "cpp" | "cc" | "py" | "yml"
                )
            }) {
                acc.push(p);
            }
        }
    }
    let mut v = Vec::new();
    walk(root, &mut v);
    v
}

/// Prefixes the source builds variable names from at runtime, e.g.
/// `format!("HORUS_PARAM_{}", key.to_uppercase())` in `commands/launch.rs`.
///
/// A documented `HORUS_PARAM_TICK_RATE` is genuinely read even though that exact
/// string appears nowhere; matching only literals reported it as missing.
fn dynamic_prefixes(haystack: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = haystack;
    while let Some(pos) = rest.find("\"HORUS_") {
        // Step onto the opening quote, take the literal, then step past it.
        let after_quote = &rest[pos + 1..];
        if let Some(lit) = after_quote.split('"').next() {
            // `HORUS_PARAM_{}` → prefix `HORUS_PARAM`. A literal with no `{`
            // is a complete name, not a prefix.
            if let Some(brace) = lit.find('{') {
                let prefix = lit[..brace].trim_end_matches('_');
                if prefix.len() > "HORUS".len() && !out.contains(&prefix.to_string()) {
                    out.push(prefix.to_string());
                }
            }
        }
        rest = after_quote;
    }
    out
}

// ─── Unit tests ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod unit {
    use super::*;

    #[test]
    fn package_fields_are_readable() {
        let f = package_info_fields();
        for expected in ["name", "version", "description", "authors", "license"] {
            assert!(f.contains(expected), "missing {expected} in {f:?}");
        }
        // Confirms the finding: `keywords` is not a manifest field.
        assert!(
            !f.contains("keywords"),
            "PackageInfo gained a `keywords` field — drop it from the docs finding"
        );
    }

    #[test]
    fn section_keys_are_scoped() {
        let toml = "[package]\nname = \"a\"\nversion = \"1\"\n\n[dependencies]\nserde = \"1\"\n";
        let k = keys_in_section(toml, "package");
        assert_eq!(k, vec!["name", "version"], "{k:?}");
        assert!(!k.contains(&"serde".to_string()));
    }

    #[test]
    fn env_names_are_extracted() {
        assert_eq!(
            horus_env_names("Set `HORUS_PROJECT_DIR` before running"),
            vec!["HORUS_PROJECT_DIR"]
        );
        assert_eq!(
            horus_env_names("HORUS_NO_PROXY=1 cargo build"),
            vec!["HORUS_NO_PROXY"]
        );
    }

    #[test]
    fn negated_mentions_are_not_promises() {
        // Naming a variable to say it does not work is good documentation, not
        // a promise the source must keep.
        assert!(documented_env_names(
            "",
            "# horus installs its own log bridge in place of env_logger, so RUST_LOG has no effect"
        )
        .is_empty());
        assert!(documented_env_names("", "there is no HORUS_API_KEY variable").is_empty());
        // An adverb between "is" and "no" must not defeat the match — writers
        // reach for one naturally, and every occurrence in the corpus has one.
        assert!(documented_env_names(
            "",
            "There is deliberately **no `HORUS_API_KEY` environment variable** — nothing reads one."
        )
        .is_empty());
        // The disclaimer routinely wraps, leaving the name on a line that reads
        // as a plain instruction on its own.
        assert!(documented_env_names(
            "HORUS reads the key from `auth.json` and nowhere else — there is no",
            "`HORUS_API_KEY` environment variable. For non-interactive environments, write"
        )
        .is_empty());
        // A genuine instruction still counts — including when the previous line
        // is unrelated prose that happens to contain no negation.
        assert_eq!(
            documented_env_names("Run the node like this:", "RUST_LOG=debug horus run"),
            vec!["RUST_LOG"]
        );
        assert_eq!(
            documented_env_names("", "RUST_LOG=debug horus run"),
            vec!["RUST_LOG"]
        );
    }

    #[test]
    fn ci_secret_names_are_recognized() {
        // package-management/publishing.mdx documents a GitHub Actions job that
        // writes ~/.horus/auth.json from a secret the reader names. That name is
        // not a variable horus reads, and demanding the source mention it made
        // the suite reject a correct doc fix.
        let line = "          HORUS_REGISTRY_KEY: ${{ secrets.HORUS_REGISTRY_KEY }}";
        assert!(line.contains("secrets."));
        assert_eq!(horus_env_names(line), vec!["HORUS_REGISTRY_KEY"]);
    }

    #[test]
    fn env_scanner_ignores_prose_and_suffixes() {
        // A bare prefix in prose is not a variable name.
        assert!(horus_env_names("the HORUS_ prefix is reserved").is_empty());
        // Nor is an embedded occurrence.
        assert!(horus_env_names("MY_HORUS_THING=1").is_empty());
    }

    #[test]
    fn haystack_excludes_this_suite() {
        // Regression: the module docs above name HORUS_PROJECT_DIR and
        // HORUS_NO_PROXY as variables nothing reads. Scanning `tests/` put them
        // in the haystack and suppressed the very finding being described.
        let files = consumer_sources(&repo_root());
        assert!(
            !files
                .iter()
                .any(|p| p.to_string_lossy().contains("/tests/")),
            "consumer_sources must not scan tests/, or this suite can satisfy \
             its own assertions by mentioning a name"
        );
        assert!(
            files
                .iter()
                .any(|p| p.extension().is_some_and(|e| e == "hpp")),
            "C++ headers must be scanned — HORUS_TOPIC_IMPL is a macro there"
        );
    }

    #[test]
    fn dynamic_prefixes_are_detected() {
        let src = r#"let env_key = format!("HORUS_PARAM_{}", k.to_uppercase());"#;
        let p = dynamic_prefixes(src);
        assert_eq!(p, vec!["HORUS_PARAM"], "{p:?}");
        // A complete literal name is not a prefix.
        assert!(dynamic_prefixes(r#"env::var("HORUS_SOURCE")"#).is_empty());
    }

    #[test]
    fn cargo_manifests_are_not_horus_manifests() {
        // The plugin guide's "Set Up Cargo.toml" block. It has a `[package]`
        // with a name and a version, which is all the old check looked for.
        let cargo = "[package]\nname = \"horus-mycommand\"\nversion = \"0.1.0\"\n\
                     edition = \"2021\"\n\n[[bin]]\nname = \"mycommand\"\n\
                     path = \"src/main.rs\"\n";
        assert!(
            !is_full_manifest(cargo),
            "a Cargo.toml must not be validated as a horus.toml"
        );

        let horus = "[package]\nname = \"my_robot\"\nversion = \"0.1.0\"\n\
                     [dependencies]\n";
        assert!(
            is_full_manifest(horus),
            "a real horus.toml must still be checked"
        );
    }

    #[test]
    fn manifest_completeness_detection() {
        assert!(is_full_manifest(
            "[package]\nname = \"a\"\nversion = \"0.1.0\"\n"
        ));
        // A dependency excerpt is not a manifest.
        assert!(!is_full_manifest("[dependencies]\nserde = \"1\"\n"));
        // Nor is a Cargo.toml shown for contrast.
        assert!(!is_full_manifest(
            "[package]\nname = \"a\"\nversion = \"1\"\n[lib]\ncrate-type = [\"cdylib\"]\n"
        ));
    }

    #[test]
    fn toml_blocks_are_dedented() {
        let md = "```toml\n    [package]\n    name = \"a\"\n```\n";
        let b = toml_blocks(md, "p.mdx");
        assert!(b[0].code.starts_with("[package]"), "{:?}", b[0].code);
    }
}

// ─── The published JSON Schema (CFG-5) ──────────────────────────────────────
//
// `horus schema` printed a schema that no URL served, so every project had to
// generate and vendor its own copy — and a vendored schema goes stale on its
// own schedule, which is the failure mode a schema exists to prevent. The
// documentation site deploys from horus-docs, so a file under its `public/`
// directory is live at a stable address; these tests keep that file honest.

/// The committed copy must be exactly what the current structs generate.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn the_published_schema_matches_the_generator() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");

    // Where the URL says the file is served from — `public/` is the site root.
    let url = horus_manager::manifest::MANIFEST_SCHEMA_URL;
    let path_part = url
        .split_once("://")
        .and_then(|(_, rest)| rest.split_once('/'))
        .map(|(_, p)| p)
        .expect("MANIFEST_SCHEMA_URL must have a path");
    let published = docs.join("public").join(path_part);

    let committed = std::fs::read_to_string(&published).unwrap_or_else(|e| {
        panic!(
            "{} is missing ({e}).\n\n{url} is documented as the schema's home and is \
             served from that file. Regenerate it with:\n    horus schema -o {}",
            published.display(),
            published.display()
        )
    });

    let out = cargo_bin_cmd!("horus")
        .arg("schema")
        .output()
        .expect("horus schema runs");
    assert!(out.status.success(), "`horus schema` must succeed");
    let generated = String::from_utf8_lossy(&out.stdout).into_owned();

    assert_eq!(
        committed.trim_end(),
        generated.trim_end(),
        "the published schema has drifted from the manifest structs.\n\
         An editor pointed at {url} is validating against a HORUS that no longer \
         exists. Regenerate it:\n    horus schema -o {}",
        published.display()
    );

    // A schema that does not name itself cannot be told apart from a stale
    // vendored copy.
    assert!(
        committed.contains(url),
        "the published schema must carry its own $id ({url})"
    );
}

/// And the configuration page must point at it, or nobody finds it.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn the_configuration_page_publishes_the_schema_url() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let page = docs.join("content/docs/package-management/configuration.mdx");
    let text = std::fs::read_to_string(&page).expect("the configuration reference must exist");
    let url = horus_manager::manifest::MANIFEST_SCHEMA_URL;

    assert!(
        text.contains(url),
        "{} never mentions {url}, so a reader has no way to associate the schema \
         without generating and vendoring their own copy — which is the finding.",
        page.display()
    );
}

/// Version spellings the resolver accepts must be spellings the docs explain
/// (CFG-2).
///
/// `"latest"` is accepted by `horus add pkg@latest` and by the registry
/// resolver, HORUS's own root manifest used one, and the dependency grammar in
/// the docs described `"*"` and semver constraints only. A reader had no way to
/// learn that the two are the same to the registry — or that they are *not* the
/// same to cargo, which rejects `latest` outright.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn the_latest_version_spelling_is_documented() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let resolver =
        std::fs::read_to_string(repo_root().join("horus_manager/src/dependency_resolver.rs"))
            .expect("the resolver must be readable");
    let page = docs.join("content/docs/package-management/configuration.mdx");
    let text = std::fs::read_to_string(&page).expect("the configuration reference must exist");

    assert!(
        resolver.contains("== \"latest\""),
        "the resolver no longer special-cases `latest`. That is fine — but the \
         configuration reference documents it, and that section has to go with it."
    );
    assert!(
        text.contains("latest"),
        "{} documents the dependency grammar and never mentions `latest`, which \
         the resolver accepts.",
        page.display()
    );
    assert!(
        text.contains("not portable") || text.contains("HORUS registry only"),
        "documenting `latest` without saying cargo and pip reject it would teach \
         a spelling that breaks the generated build file: {}",
        page.display()
    );
}
