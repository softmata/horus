//! Detection of keys in `horus.toml` that HORUS does not understand.
//!
//! # Why this exists
//!
//! `HorusManifest` deserializes without `#[serde(deny_unknown_fields)]`, so any
//! key serde does not recognise is silently discarded. A manifest containing a
//! misspelled key, an unknown key and an entire unknown section validated
//! cleanly and exited 0:
//!
//! ```text
//! $ horus check
//!   > horus.toml
//!       * manifest valid
//!   Status: * All checks passed!
//! ```
//!
//! That is the worst failure mode a config file has: the setting does nothing,
//! and the tool actively confirms the file is correct. HORUS's own root
//! `horus.toml` carries `language = "rust"`, which is not a field of
//! [`PackageInfo`](crate::manifest::PackageInfo) and has never had any effect.
//!
//! # Why not `deny_unknown_fields`
//!
//! Because it would turn every such manifest into a hard error on upgrade —
//! including HORUS's own — and there is no changelog for a user to discover
//! why. This module reports unknown keys as *warnings* with a did-you-mean, so
//! 0.2.x tells the truth without breaking anyone. Promotion to a hard error is
//! a 0.3.0 decision, and [`UnknownKey::deprecation_notice`] carries that
//! message so the warning states it up front.
//!
//! # Keeping the key lists honest
//!
//! The lists below are hand-written, which is precisely the kind of second copy
//! that drifts. `manifest_lint_covers_all_manifest_fields` in the test module
//! round-trips a fully-populated manifest through TOML and asserts every key it
//! emits is listed here, so adding a field to `HorusManifest` without updating
//! this module fails the build rather than silently narrowing validation.

use std::collections::BTreeSet;

/// Top-level tables and keys accepted in `horus.toml`.
///
/// Mirrors the fields of [`crate::manifest::HorusManifest`].
pub const KNOWN_TOP_LEVEL: &[&str] = &[
    "package",
    "workspace",
    "robot",
    "dependencies",
    "dev-dependencies",
    "sim-dependencies",
    "hardware",
    "drivers",
    "sim-drivers",
    "scripts",
    "ignore",
    "enable",
    "cpp",
    "rust",
    "hooks",
    "network",
];

/// Keys accepted inside `[package]`.
///
/// Mirrors the fields of [`crate::manifest::PackageInfo`].
pub const KNOWN_PACKAGE: &[&str] = &[
    "name",
    "version",
    "description",
    "authors",
    "license",
    "edition",
    "repository",
    "package-type",
    "categories",
    "standard",
    "rust_edition",
    "type",
];

/// Keys accepted inside `[cpp]`.
pub const KNOWN_CPP: &[&str] = &["compiler", "cmake_args", "toolchain"];

/// Keys accepted inside `[rust]`.
///
/// These are spliced verbatim into the generated `.horus/Cargo.toml`, so the
/// list is exactly the set of Cargo sections HORUS does not write itself.
/// `dependencies` is absent on purpose: `horus.toml` already has one, and a
/// second channel would let the same crate be declared twice.
pub const KNOWN_RUST: &[&str] = &[
    "edition",
    "features",
    "profile",
    "patch",
    "build-dependencies",
    "lints",
    "target",
];

/// Keys accepted inside `[hooks]`.
pub const KNOWN_HOOKS: &[&str] = &[
    "pre_run",
    "post_run",
    "pre_build",
    "post_build",
    "pre_test",
    "post_test",
];

/// Keys developers reasonably expect to exist but which HORUS deliberately
/// does not have, mapped to the reason.
///
/// Levenshtein cannot help here: `language` is not a misspelling of anything,
/// it is a key people assume is real — HORUS's own root `horus.toml` carries
/// one and it has never done anything. A fuzzy matcher stays silent on exactly
/// the mistakes that are most common, so these get a tailored explanation.
const KNOWN_NON_FIELDS: &[(&str, &str)] = &[
    (
        "language",
        "HORUS detects the language from your source files — remove this key",
    ),
    (
        "languages",
        "HORUS detects the language from your source files — remove this key",
    ),
    (
        "build",
        "build settings live in `[cpp]` for C++; Rust build settings are not yet configurable",
    ),
    (
        "main",
        "the entry point is found by convention (src/main.rs, main.py, src/main.cpp)",
    ),
    (
        "entry",
        "the entry point is found by convention (src/main.rs, main.py, src/main.cpp)",
    ),
    (
        "nodes",
        "nodes are declared in code, not in horus.toml — see `horus launch` for multi-node startup",
    ),
    ("topics", "topics are declared in code, not in horus.toml"),
    (
        "features",
        "use the top-level `enable = [...]` key for capabilities",
    ),
    (
        "dependencies-dev",
        "the section is spelled `[dev-dependencies]`",
    ),
    (
        "devDependencies",
        "the section is spelled `[dev-dependencies]`",
    ),
];

/// A key present in the manifest that HORUS does not understand.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownKey {
    /// Dotted path as the user would search for it, e.g. `package.langauge`.
    pub path: String,
    /// 1-based line in the source file, or 0 when it could not be located.
    pub line: usize,
    /// Closest known key, when one is near enough to be worth suggesting.
    pub suggestion: Option<String>,
    /// Explanation for a key that is commonly assumed to exist but does not.
    pub known_non_field: Option<&'static str>,
}

impl UnknownKey {
    /// The warning line shown by `horus check`.
    ///
    /// Names the location, says plainly that the key has no effect (the part
    /// users most need and are least likely to infer), and offers the
    /// correction when there is a plausible one.
    pub fn message(&self) -> String {
        let where_ = if self.line > 0 {
            format!("line {}: ", self.line)
        } else {
            String::new()
        };
        // A tailored explanation beats a fuzzy guess, so it wins when present.
        if let Some(why) = self.known_non_field {
            return format!(
                "{where_}unknown key `{}` — {why}. It has no effect.",
                self.path
            );
        }
        match &self.suggestion {
            Some(s) => format!(
                "{where_}unknown key `{}` — did you mean `{}`? This key has no effect.",
                self.path, s
            ),
            None => format!(
                "{where_}unknown key `{}` — HORUS ignores it, so it has no effect.",
                self.path
            ),
        }
    }

    /// Notice appended once to a batch of warnings, stating the upgrade path.
    pub fn deprecation_notice() -> &'static str {
        "Unknown keys become an error in HORUS 0.3.0. Remove them or correct the spelling."
    }
}

/// Find every key in `content` that HORUS does not understand.
///
/// Parses independently of `serde` because by the time deserialization has run,
/// unknown keys are already gone. Returns keys in source order.
pub fn find_unknown_keys(content: &str) -> Vec<UnknownKey> {
    // Deserialize as a document, not a value: in toml 1.x `str::parse::<Value>`
    // parses a bare value and rejects `[section]` headers outright, which would
    // make this function silently find nothing on every real manifest.
    let Ok(table) = toml::from_str::<toml::Table>(content) else {
        // Malformed TOML is reported by the parser with a real span; producing
        // speculative key warnings on top would only add noise.
        return Vec::new();
    };

    let mut found = Vec::new();

    for key in table.keys() {
        if !KNOWN_TOP_LEVEL.contains(&key.as_str()) {
            found.push(UnknownKey {
                path: key.clone(),
                line: locate(content, key, None),
                suggestion: suggest(key, KNOWN_TOP_LEVEL),
                known_non_field: known_non_field(key),
            });
        }
    }

    // Nested tables with a closed key set. Open-ended maps — [dependencies],
    // [hardware], [scripts] and friends — are user-namespaced by design and
    // are deliberately not checked.
    for (section, known) in [
        ("package", KNOWN_PACKAGE),
        ("cpp", KNOWN_CPP),
        ("rust", KNOWN_RUST),
        ("hooks", KNOWN_HOOKS),
    ] {
        let Some(sub) = table.get(section).and_then(|v| v.as_table()) else {
            continue;
        };
        for key in sub.keys() {
            if !known.contains(&key.as_str()) {
                found.push(UnknownKey {
                    path: format!("{section}.{key}"),
                    line: locate(content, key, Some(section)),
                    suggestion: suggest(key, known),
                    known_non_field: known_non_field(key),
                });
            }
        }
    }

    found.sort_by_key(|u| (u.line, u.path.clone()));
    found
}

/// Locate the 1-based line where `key` is assigned.
///
/// When `section` is given, only lines after that section header are
/// considered, so `name` under `[package]` is not confused with `name`
/// elsewhere. Returns 0 when the key cannot be located, which keeps the
/// warning useful even if the scan fails.
fn locate(content: &str, key: &str, section: Option<&str>) -> usize {
    let mut in_section = section.is_none();

    for (idx, raw) in content.lines().enumerate() {
        let line = raw.trim();

        if line.starts_with('[') {
            if let Some(want) = section {
                let header = line
                    .trim_start_matches('[')
                    .trim_end_matches(']')
                    .trim_matches('"')
                    .trim();
                in_section = header == want;
            } else if line == format!("[{key}]") || line == format!("[[{key}]]") {
                return idx + 1;
            }
            continue;
        }

        if !in_section {
            continue;
        }

        // `key = value`, tolerating whitespace and quoted keys.
        if let Some((lhs, _)) = line.split_once('=') {
            if lhs.trim().trim_matches('"') == key {
                return idx + 1;
            }
        }
    }
    0
}

/// Look up a key that developers commonly assume exists.
fn known_non_field(key: &str) -> Option<&'static str> {
    KNOWN_NON_FIELDS
        .iter()
        .find(|(k, _)| *k == key)
        .map(|(_, why)| *why)
}

/// Suggest the closest known key, if one is close enough to be helpful.
///
/// The threshold scales with word length so short keys do not attract wild
/// suggestions: a two-edit distance on a four-letter key is a different word,
/// while on a twelve-letter key it is a typo.
fn suggest(key: &str, known: &[&str]) -> Option<String> {
    let max_distance = match key.len() {
        0..=3 => 1,
        4..=7 => 2,
        _ => 3,
    };

    known
        .iter()
        .map(|k| (*k, levenshtein(key, k)))
        .filter(|(_, d)| *d <= max_distance)
        .min_by_key(|(k, d)| (*d, k.len()))
        .map(|(k, _)| k.to_string())
}

/// Levenshtein edit distance, two-row variant.
///
/// Implemented here rather than pulled in as a dependency: it is fifteen lines
/// and only ever runs over manifest key names.
fn levenshtein(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    if a.is_empty() {
        return b.len();
    }
    if b.is_empty() {
        return a.len();
    }

    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut curr = vec![0usize; b.len() + 1];

    for (i, ca) in a.iter().enumerate() {
        curr[0] = i + 1;
        for (j, cb) in b.iter().enumerate() {
            let cost = if ca == cb { 0 } else { 1 };
            curr[j + 1] = (prev[j] + cost).min(prev[j + 1] + 1).min(curr[j] + 1);
        }
        std::mem::swap(&mut prev, &mut curr);
    }
    prev[b.len()]
}

/// Every key name this module recognises, for cross-checking against the
/// manifest structs in tests.
pub fn all_known_keys() -> BTreeSet<String> {
    KNOWN_TOP_LEVEL
        .iter()
        .chain(KNOWN_PACKAGE)
        .chain(KNOWN_CPP)
        .chain(KNOWN_RUST)
        .chain(KNOWN_HOOKS)
        .map(|s| s.to_string())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── The exact manifest from the DX audit (CFG-1) ────────────────────────

    const AUDIT_MANIFEST: &str = r#"[package]
name = "goodname"
version = "0.1.0"
langauge = "rust"
totally_unknown_key = 42

[bogus_section]
foo = 1
"#;

    #[test]
    fn audit_manifest_reports_all_three_unknown_keys() {
        let found = find_unknown_keys(AUDIT_MANIFEST);
        let paths: Vec<&str> = found.iter().map(|u| u.path.as_str()).collect();
        assert!(
            paths.contains(&"package.langauge"),
            "misspelled key must be caught: {paths:?}"
        );
        assert!(
            paths.contains(&"package.totally_unknown_key"),
            "unknown key must be caught: {paths:?}"
        );
        assert!(
            paths.contains(&"bogus_section"),
            "unknown section must be caught: {paths:?}"
        );
        assert_eq!(found.len(), 3, "no extra findings: {paths:?}");
    }

    #[test]
    fn misspelled_key_suggests_the_correction() {
        let toml = "[package]\nname = \"robot\"\nversion = \"0.1.0\"\nversoin = \"0.2.0\"\n";
        let found = find_unknown_keys(toml);
        let versoin = found
            .iter()
            .find(|u| u.path == "package.versoin")
            .expect("versoin finding");
        assert_eq!(versoin.suggestion.as_deref(), Some("version"));
        assert!(versoin.message().contains("did you mean `version`"));
    }

    /// `language` is not a typo — it is a key people assume exists, and HORUS's
    /// own root manifest carries one. Levenshtein finds nothing close, so
    /// without the known-non-field table the most common real mistake in the
    /// codebase would get the least helpful message.
    #[test]
    fn language_key_gets_a_tailored_explanation_not_a_guess() {
        let toml = "[package]\nname = \"robot\"\nversion = \"0.1.0\"\nlanguage = \"rust\"\n";
        let found = find_unknown_keys(toml);
        let lang = found
            .iter()
            .find(|u| u.path == "package.language")
            .expect("language must be reported as unknown");

        assert!(
            lang.known_non_field.is_some(),
            "language must resolve to a tailored explanation"
        );
        let m = lang.message();
        assert!(
            m.contains("detects the language from your source files"),
            "must explain why the key does not exist: {m}"
        );
        assert!(
            !m.contains("did you mean"),
            "must not offer a fuzzy guess when a real explanation exists: {m}"
        );
        assert!(m.contains("no effect"), "{m}");
    }

    /// The repository's own root manifest is the canonical reproduction.
    #[test]
    fn horus_own_root_manifest_shape_is_flagged() {
        let toml = "[package]\nname = \"HORUS\"\nversion = \"0.2.0\"\nlanguage = \"rust\"\n\n\
                    [dependencies]\nhorus = \"*\"\n";
        let found = find_unknown_keys(toml);
        assert_eq!(
            found.len(),
            1,
            "exactly the language key should be flagged: {found:?}"
        );
        assert_eq!(found[0].path, "package.language");
    }

    #[test]
    fn unknown_keys_carry_line_numbers() {
        let found = find_unknown_keys(AUDIT_MANIFEST);
        let langauge = found.iter().find(|u| u.path == "package.langauge").unwrap();
        assert_eq!(langauge.line, 4, "line number must point at the key");
        let bogus = found.iter().find(|u| u.path == "bogus_section").unwrap();
        assert_eq!(
            bogus.line, 7,
            "section line number must point at the header"
        );
    }

    #[test]
    fn message_states_the_key_has_no_effect() {
        let u = UnknownKey {
            path: "package.langauge".into(),
            line: 4,
            suggestion: Some("language".into()),
            known_non_field: None,
        };
        let m = u.message();
        assert!(m.contains("line 4"), "{m}");
        assert!(m.contains("langauge"), "{m}");
        assert!(m.contains("language"), "{m}");
        assert!(
            m.contains("no effect"),
            "the consequence is the part users cannot infer: {m}"
        );
    }

    // ── No false positives ──────────────────────────────────────────────────

    #[test]
    fn a_fully_valid_manifest_produces_no_warnings() {
        let toml = r#"
[package]
name = "robot"
version = "0.1.0"
description = "d"
authors = ["a"]
license = "Apache-2.0"
edition = "2024"
repository = "https://example.com"
package-type = "library"
categories = ["lidar"]
standard = "c++17"
rust_edition = "2021"
type = "bin"

[dependencies]
serde = "1"

[dev-dependencies]
approx = "0.5"

[cpp]
compiler = "clang++"
cmake_args = ["-DFOO=1"]

[hooks]
pre_run = ["fmt"]

[scripts]
deploy = "echo hi"
"#;
        let found = find_unknown_keys(toml);
        assert!(found.is_empty(), "false positives: {found:?}");
    }

    #[test]
    fn open_ended_sections_are_not_checked() {
        // [dependencies], [hardware], [scripts] are user-namespaced by design;
        // flagging their contents would make the check useless.
        let toml = r#"
[package]
name = "robot"
version = "0.1.0"

[dependencies]
anything_at_all = "1"

[hardware]
my_weird_lidar = { sim = true }

[scripts]
whatever = "true"
"#;
        assert!(find_unknown_keys(toml).is_empty());
    }

    #[test]
    fn malformed_toml_yields_no_key_warnings() {
        // The parser reports syntax errors with a real span; speculative key
        // warnings on top would be noise.
        assert!(find_unknown_keys("[package\nname = \"x\"\n").is_empty());
    }

    #[test]
    fn duplicate_key_names_across_sections_resolve_to_the_right_line() {
        let toml = r#"
[hooks]
pre_run = ["fmt"]

[package]
name = "robot"
version = "0.1.0"
bogus = 1
"#;
        let found = find_unknown_keys(toml);
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].path, "package.bogus");
        assert_eq!(found[0].line, 8, "must find the key inside [package]");
    }

    // ── Suggestion quality ──────────────────────────────────────────────────

    #[test]
    fn suggestions_are_offered_for_real_typos() {
        assert_eq!(
            suggest("versoin", KNOWN_PACKAGE).as_deref(),
            Some("version")
        );
        assert_eq!(
            suggest("licence", KNOWN_PACKAGE).as_deref(),
            Some("license")
        );
        assert_eq!(
            suggest("dependencis", KNOWN_TOP_LEVEL).as_deref(),
            Some("dependencies")
        );
    }

    #[test]
    fn unrelated_keys_get_no_suggestion() {
        assert_eq!(suggest("completely_unrelated_key", KNOWN_TOP_LEVEL), None);
        assert_eq!(suggest("zzz", KNOWN_PACKAGE), None);
    }

    #[test]
    fn levenshtein_is_correct() {
        assert_eq!(levenshtein("", ""), 0);
        assert_eq!(levenshtein("abc", "abc"), 0);
        assert_eq!(levenshtein("abc", ""), 3);
        assert_eq!(levenshtein("", "abc"), 3);
        assert_eq!(levenshtein("kitten", "sitting"), 3);
        assert_eq!(levenshtein("langauge", "language"), 2);
    }

    // ── Contract: the key lists must not drift from the structs ─────────────

    /// The published JSON Schema and this module's key lists must agree.
    ///
    /// Both are derived from `HorusManifest`, and both are consumed by users —
    /// the schema by their editor, these lists by `horus check`. If they
    /// disagree, one of them is telling the user something false: either the
    /// editor accepts a key `check` rejects, or `check` accepts a key the
    /// editor flags. This is the cross-check that keeps two second copies
    /// honest.
    #[test]
    #[cfg(feature = "schema")]
    fn json_schema_and_lint_agree_on_top_level_keys() {
        let schema: serde_json::Value =
            serde_json::from_str(&crate::manifest::generate_manifest_schema())
                .expect("generate_manifest_schema must emit valid JSON");

        let props = schema
            .get("properties")
            .and_then(|p| p.as_object())
            .expect("schema must describe top-level properties");

        for key in props.keys() {
            assert!(
                KNOWN_TOP_LEVEL.contains(&key.as_str()),
                "the JSON Schema exposes `{key}` but manifest_lint would warn \
                 that it is unknown — an editor would accept what `horus check` \
                 rejects. Add it to KNOWN_TOP_LEVEL."
            );
        }

        for known in KNOWN_TOP_LEVEL {
            assert!(
                props.contains_key(*known),
                "manifest_lint accepts `{known}` but the JSON Schema does not \
                 describe it — an editor would flag a key `horus check` allows."
            );
        }
    }

    /// Serializing a fully-populated manifest must not produce a key this
    /// module would flag as unknown.
    ///
    /// This is the guard against the second-copy problem: adding a field to
    /// `HorusManifest` or `PackageInfo` without listing it here fails the
    /// build instead of silently narrowing validation.
    #[test]
    fn manifest_lint_covers_all_manifest_fields() {
        use crate::manifest::HorusManifest;

        let mut m = HorusManifest::default();
        m.package.name = "robot".into();
        m.package.version = "0.1.0".into();
        m.package.description = Some("d".into());
        m.package.authors = vec!["a".into()];
        m.package.license = Some("Apache-2.0".into());
        m.package.repository = Some("https://example.com".into());
        m.package.categories = vec!["lidar".into()];
        m.package.standard = Some("c++17".into());
        m.package.rust_edition = Some("2021".into());
        m.enable = vec!["cuda".into()];

        let serialized = toml::to_string_pretty(&m).expect("manifest must serialize");
        let unknown = find_unknown_keys(&serialized);
        assert!(
            unknown.is_empty(),
            "manifest_lint key lists have drifted from the manifest structs.\n\
             Serializing a populated HorusManifest produced keys this module \
             does not know: {unknown:?}\n\
             Add them to KNOWN_TOP_LEVEL / KNOWN_PACKAGE in manifest_lint.rs.\n\
             --- serialized manifest ---\n{serialized}"
        );
    }
}
