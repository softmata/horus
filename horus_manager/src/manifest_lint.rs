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
//! Because serde's own error aborts the whole load: one stray key and `horus
//! run` cannot read the manifest at all, with no line, no suggestion and no way
//! to see the other nine keys that were also wrong. This module reads the keys
//! back off the source text instead, so every unknown key in the file is
//! reported at once, with its line and a did-you-mean.
//!
//! What changed in 0.3.0 is the *severity*, which is what the 0.2.x warning
//! announced: `horus check` now counts an unknown key as an error and exits
//! non-zero, because a validation gate that says "valid" about a file whose
//! settings do nothing is the defect, not the fix. Every other command — `run`,
//! `build`, `test`, `add` — still only warns (see
//! [`crate::manifest::HorusManifest::load_from`]), so a typo tells you the
//! truth without stopping work mid-session.
//!
//! # Keeping the key lists honest
//!
//! The lists below are hand-written, which is precisely the kind of second copy
//! that drifts. Two tests hold them to the structs:
//! `manifest_lint_covers_all_manifest_fields` round-trips a fully-populated
//! manifest through TOML and asserts every key it emits is listed here, and
//! `json_schema_and_lint_agree_on_every_closed_table` compares each list
//! against the JSON Schema generated from the structs themselves — including
//! the nested tables, which is where the first version of this module went
//! blind: `[network]`, `[robot]`, `[workspace]` and `[ignore]` are closed
//! structs whose misspelled keys were dropped in silence.

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
/// All but one of these are spliced verbatim into the generated
/// `.horus/Cargo.toml`, so the list is the set of Cargo sections HORUS does not
/// write itself. `dependencies` is absent on purpose: `horus.toml` already has
/// one, and a second channel would let the same crate be declared twice.
///
/// `rustflags` is the exception — it has no Cargo.toml section, and is emitted
/// to a generated `.horus/.cargo/config.toml`. It is listed here because a key
/// this module rejects is a key `horus check` fails on, and rejecting a setting
/// that does work would be the same lie as accepting one that does not.
pub const KNOWN_RUST: &[&str] = &[
    "edition",
    "features",
    "profile",
    "patch",
    "build-dependencies",
    "lints",
    "target",
    "rustflags",
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

/// Keys accepted inside `[workspace]`.
///
/// Mirrors the fields of [`crate::manifest::WorkspaceConfig`].
pub const KNOWN_WORKSPACE: &[&str] = &["members", "exclude", "dependencies"];

/// Keys accepted inside `[robot]`.
///
/// Mirrors the fields of [`crate::manifest::RobotConfig`].
pub const KNOWN_ROBOT: &[&str] = &["name", "description", "simulator"];

/// Keys accepted inside `[network]`.
///
/// Mirrors the fields of [`crate::manifest::NetworkConfig`]. A misspelled key
/// here is the most expensive kind in the file: `sekret` instead of `secret`
/// leaves a fleet unauthenticated while the manifest looks configured.
pub const KNOWN_NETWORK: &[&str] = &[
    "enabled",
    "import",
    "deny_export",
    "secret",
    "optimize",
    "safety",
];

/// Keys accepted inside `[ignore]`.
///
/// Mirrors the fields of [`crate::manifest::IgnoreConfig`].
pub const KNOWN_IGNORE: &[&str] = &["files", "directories", "packages"];

/// Keys accepted inside `[network.safety]`.
///
/// Mirrors the fields of [`crate::manifest::NetworkSafetyConfig`].
pub const KNOWN_NETWORK_SAFETY: &[&str] = &["heartbeat_ms", "missed_threshold", "on_link_lost"];

/// Every table in `horus.toml` with a fixed set of keys, by dotted path.
///
/// The open-ended maps are deliberately absent: `[dependencies]`,
/// `[hardware]`, `[drivers]`, `[sim-drivers]`, `[sim-dependencies]`,
/// `[dev-dependencies]`, `[scripts]` and the `[rust]` sub-tables are
/// user-namespaced, and flagging their contents would make the check useless.
pub const CLOSED_TABLES: &[(&str, &[&str])] = &[
    ("package", KNOWN_PACKAGE),
    ("workspace", KNOWN_WORKSPACE),
    ("robot", KNOWN_ROBOT),
    ("cpp", KNOWN_CPP),
    ("rust", KNOWN_RUST),
    ("hooks", KNOWN_HOOKS),
    ("ignore", KNOWN_IGNORE),
    ("network", KNOWN_NETWORK),
    ("network.safety", KNOWN_NETWORK_SAFETY),
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
        "build settings live in `[cpp]` for C++ and `[rust]` for Rust",
    ),
    (
        // The key people reach for once they find `[rust]` and want one more
        // cargo flag. Nothing forwards extra arguments to the cargo spawn, so
        // the useful answer is the two routes that do work — "unknown key"
        // leaves them where they started.
        "cargo_args",
        "HORUS does not forward extra cargo arguments; run `horus cargo <args>` \
         for a raw cargo invocation on the generated manifest, or set \
         `[rust].rustflags` for flags rustc should always get",
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
    /// The finding as `horus check` states it.
    ///
    /// Says plainly that the key has no effect (the part users most need and
    /// are least likely to infer), and offers the correction when there is a
    /// plausible one. The *position* is deliberately not in here: every caller
    /// renders it as `file:line:col:` alongside every other finding, so one
    /// location format serves the whole tool instead of this one message
    /// carrying its own.
    pub fn message(&self) -> String {
        // A tailored explanation beats a fuzzy guess, so it wins when present.
        if let Some(why) = self.known_non_field {
            return format!("unknown key `{}` — {why}. It has no effect.", self.path);
        }
        match &self.suggestion {
            Some(s) => format!(
                "unknown key `{}` — did you mean `{}`? This key has no effect.",
                self.path, s
            ),
            None => format!(
                "unknown key `{}` — HORUS ignores it, so it has no effect.",
                self.path
            ),
        }
    }

    /// `file:line: message`, for callers with nowhere better to put the
    /// position.
    pub fn at(&self, file: &std::path::Path) -> String {
        if self.line > 0 {
            format!("{}:{}: {}", file.display(), self.line, self.message())
        } else {
            format!("{}: {}", file.display(), self.message())
        }
    }

    /// Notice appended once to a batch of findings, stating what it costs.
    ///
    /// 0.2.x said "becomes an error in HORUS 0.3.0". 0.3.0 is here, so the
    /// notice states the behaviour rather than promising it again.
    pub fn severity_notice() -> &'static str {
        "Unknown keys are an error as of HORUS 0.3.0: `horus check` fails while \
         they are present, and every other command warns. Remove them or correct \
         the spelling."
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
            found.push(describe(content, None, key, KNOWN_TOP_LEVEL));
        }
    }

    // Every nested table with a closed key set, including the ones reached
    // through a parent (`[network.safety]`). Open-ended maps —
    // [dependencies], [hardware], [scripts] and friends — are user-namespaced
    // by design and are deliberately not checked; see CLOSED_TABLES.
    for (path, known) in CLOSED_TABLES {
        let Some(sub) = descend(&table, path) else {
            continue;
        };
        for key in sub.keys() {
            if !known.contains(&key.as_str()) {
                found.push(describe(content, Some(path), key, known));
            }
        }
    }

    found.sort_by_key(|u| (u.line, u.path.clone()));
    found
}

/// Walk a dotted path down to the table it names.
///
/// `[network.safety]` and `network = { safety = { .. } }` are the same table in
/// TOML, and both have to be reached the same way — off the parsed document,
/// not by matching a header in the text.
fn descend<'t>(table: &'t toml::Table, path: &str) -> Option<&'t toml::Table> {
    let mut current = table;
    for segment in path.split('.') {
        current = current.get(segment)?.as_table()?;
    }
    Some(current)
}

/// Build the finding for one unrecognised key.
fn describe(content: &str, section: Option<&str>, key: &str, known: &[&str]) -> UnknownKey {
    let suggestion = suggest(key, known);
    UnknownKey {
        path: match section {
            Some(s) => format!("{s}.{key}"),
            None => key.to_string(),
        },
        line: locate(content, key, section),
        known_non_field: known_non_field(key, suggestion.is_some()),
        suggestion,
    }
}

/// The 1-based line a dotted manifest key sits on, or 0 when it cannot be
/// found.
///
/// `package.version` resolves to the line of `version` inside `[package]`;
/// a bare `package` resolves to the `[package]` header itself. Exposed so
/// `horus check` can attach a position to a *validation* failure — "version is
/// not valid semver" knew which key it was about and printed no line at all.
pub fn locate_key(content: &str, dotted: &str) -> usize {
    match dotted.rsplit_once('.') {
        Some((section, key)) => locate(content, key, Some(section)),
        None => locate(content, dotted, None),
    }
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

    // The section may have been written as an inline table
    // (`network = { sekret = "x" }`), which has no header line to scan from.
    // Falling back to the line the table itself is on beats reporting none.
    section.map(|s| locate_inline(content, s, key)).unwrap_or(0)
}

/// Line of an inline table that contains `key`, or 0.
fn locate_inline(content: &str, section: &str, key: &str) -> usize {
    let Some(last) = section.rsplit('.').next() else {
        return 0;
    };
    for (idx, raw) in content.lines().enumerate() {
        let line = raw.trim();
        if let Some((lhs, rhs)) = line.split_once('=') {
            if lhs.trim().trim_matches('"') == last && rhs.contains(key) {
                return idx + 1;
            }
        }
    }
    0
}

/// Look up a key that developers commonly assume exists.
///
/// `has_suggestion` decides the tie: a key close to a *real* field is a typo of
/// that field and gets the did-you-mean. Only when nothing real is close does a
/// near miss on an assumed key count — `langauge` is a misspelling of
/// `language`, which does not exist either, and the tailored explanation is the
/// answer the user needs. Without this, the most common mistake in the file got
/// the least useful message: "HORUS ignores it".
fn known_non_field(key: &str, has_suggestion: bool) -> Option<&'static str> {
    if let Some((_, why)) = KNOWN_NON_FIELDS.iter().find(|(k, _)| *k == key) {
        return Some(why);
    }
    if has_suggestion {
        return None;
    }
    let names: Vec<&str> = KNOWN_NON_FIELDS.iter().map(|(k, _)| *k).collect();
    let near = suggest(key, &names)?;
    KNOWN_NON_FIELDS
        .iter()
        .find(|(k, _)| *k == near)
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
        .chain(CLOSED_TABLES.iter().flat_map(|(_, keys)| keys.iter()))
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
        assert_eq!(versoin.line, 4, "the line travels on the finding");
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
        assert!(m.contains("langauge"), "{m}");
        assert!(m.contains("language"), "{m}");
        assert!(
            m.contains("no effect"),
            "the consequence is the part users cannot infer: {m}"
        );
        // The position travels with the finding, not inside its prose.
        assert_eq!(u.line, 4);
        assert!(
            u.at(std::path::Path::new("horus.toml"))
                .starts_with("horus.toml:4: "),
            "a caller with nowhere to put the line must still be able to say it"
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

    /// The same cross-check, one level down.
    ///
    /// The top-level version of this test passed for years while `[network]`,
    /// `[robot]`, `[workspace]` and `[ignore]` were not linted at all: their
    /// *tables* were known, so nothing was missing from `KNOWN_TOP_LEVEL`, and
    /// the keys inside them were dropped in silence. This is the guard that
    /// would have caught it — every closed table's key list against the schema
    /// generated from the struct itself.
    #[test]
    #[cfg(feature = "schema")]
    fn json_schema_and_lint_agree_on_every_closed_table() {
        let schema: serde_json::Value =
            serde_json::from_str(&crate::manifest::generate_manifest_schema())
                .expect("generate_manifest_schema must emit valid JSON");
        let defs = schema
            .get("$defs")
            .and_then(|d| d.as_object())
            .expect("the schema must define the nested tables");

        // Walk the schema instead of a hand-written list of structs: the
        // original defect was a table nobody had thought to write down, so a
        // test that starts from what this module already knows about could not
        // have found it. Anything the schema describes as a table with fixed
        // properties *is* a closed table, whether or not CLOSED_TABLES says so.
        let root = schema
            .get("properties")
            .and_then(|p| p.as_object())
            .expect("the schema must describe the top level");

        /// `#/$defs/NetworkConfig` -> `NetworkConfig`, through `anyOf` when the
        /// field is optional.
        fn referenced(property: &serde_json::Value) -> Option<String> {
            if let Some(r) = property.get("$ref").and_then(|r| r.as_str()) {
                return r.strip_prefix("#/$defs/").map(str::to_string);
            }
            property
                .get("anyOf")?
                .as_array()?
                .iter()
                .find_map(|v| v.get("$ref").and_then(|r| r.as_str()))
                .and_then(|r| r.strip_prefix("#/$defs/"))
                .map(str::to_string)
        }

        let mut queue: Vec<(String, &serde_json::Map<String, serde_json::Value>)> =
            vec![(String::new(), root)];
        let mut tables: Vec<String> = Vec::new();
        let mut seen_types: Vec<String> = Vec::new();

        while let Some((prefix, props)) = queue.pop() {
            for (name, property) in props {
                let Some(type_name) = referenced(property) else {
                    continue;
                };
                let Some(fields) = defs
                    .get(&type_name)
                    .and_then(|d| d.get("properties"))
                    .and_then(|p| p.as_object())
                else {
                    // Not a table with fixed keys (an enum, a map value type).
                    continue;
                };
                let table = if prefix.is_empty() {
                    name.clone()
                } else {
                    format!("{prefix}.{name}")
                };

                let keys = CLOSED_TABLES
                    .iter()
                    .find(|(t, _)| *t == table)
                    .map(|(_, k)| *k)
                    .unwrap_or_else(|| {
                        panic!(
                            "[{table}] is a {type_name} — a struct with a fixed set of \
                             fields — but it is not in CLOSED_TABLES, so a misspelled \
                             key inside it is dropped in silence. That is exactly how \
                             [network], [robot], [workspace] and [ignore] went \
                             unchecked."
                        )
                    });

                for key in fields.keys() {
                    assert!(
                        keys.contains(&key.as_str()),
                        "the JSON Schema exposes `{table}.{key}` but manifest_lint would \
                         warn that it is unknown — an editor would accept what \
                         `horus check` rejects."
                    );
                }
                for known in keys {
                    assert!(
                        fields.contains_key(*known),
                        "manifest_lint accepts `{table}.{known}` but {type_name} has no \
                         such field — `horus check` would allow a key that does nothing."
                    );
                }

                tables.push(table.clone());
                if !seen_types.contains(&type_name) {
                    seen_types.push(type_name);
                    queue.push((table, fields));
                }
            }
        }

        tables.sort();
        let mut declared: Vec<String> = CLOSED_TABLES.iter().map(|(t, _)| t.to_string()).collect();
        declared.sort();
        assert_eq!(
            tables, declared,
            "the schema and CLOSED_TABLES must describe the same set of closed \
             tables — a table in one and not the other is either an unchecked \
             struct or a list entry nothing backs"
        );
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

    /// `rustflags` is the one `[rust]` key with no Cargo.toml section behind
    /// it, so it was the one most likely to be left off this module's list —
    /// and a key `find_unknown_keys` reports is a key `horus check` fails on,
    /// which would make a working setting look like a typo.
    #[test]
    fn rustflags_is_accepted_inside_the_rust_section() {
        let manifest = "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n\
                        [rust]\nrustflags = [\"-C\", \"target-cpu=native\"]\n";
        assert!(
            find_unknown_keys(manifest).is_empty(),
            "{:?}",
            find_unknown_keys(manifest)
        );
    }

    /// `[rust]` covers the Cargo.toml sections, so the next thing a user
    /// wants is an extra cargo flag — and nothing forwards one. Saying only
    /// "unknown key" leaves them exactly where they started.
    #[test]
    fn cargo_args_is_answered_with_the_routes_that_do_work() {
        let manifest = "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n\
                        [rust]\ncargo_args = [\"--offline\"]\n";
        let unknown = find_unknown_keys(manifest);
        assert_eq!(unknown.len(), 1, "{unknown:?}");
        let message = unknown[0].message();
        assert!(message.contains("horus cargo"), "{message}");
        assert!(message.contains("rustflags"), "{message}");
    }

    /// And a near miss must still be caught, or the list above is doing
    /// nothing.
    #[test]
    fn a_misspelled_rustflags_is_still_reported() {
        let manifest = "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n\
                        [rust]\nrustflag = [\"-C\"]\n";
        let unknown = find_unknown_keys(manifest);
        assert_eq!(unknown.len(), 1, "{unknown:?}");
        assert_eq!(unknown[0].suggestion.as_deref(), Some("rustflags"));
    }
}
