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
        // Cargo manifests appear on the migration pages for contrast.
        && !code.contains("[lib]")
        && !code.contains("crate-type")
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

    for f in doc_files(&docs) {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        let r = rel(&docs, &f);
        for (i, line) in text.lines().enumerate() {
            for name in horus_env_names(line) {
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
