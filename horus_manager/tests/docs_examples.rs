//! Compile the Rust examples printed in the documentation.
//!
//! # Why this is separate from `docs_contract.rs`
//!
//! `docs_contract` checks the CLI surface: cheap, hermetic, runs on every PR.
//! This file checks the *code* users copy out of the docs, which means invoking
//! `cargo check` against the real `horus` crate. That is minutes, not
//! milliseconds, so every test here is `#[ignore]`d and driven by a scheduled
//! CI job (see `.github/workflows/docs-contract.yml`).
//!
//! # Why it does not reuse the docs repo's own verifier
//!
//! `horus-docs` ships `scripts/verify-rust-local.mjs`, but it only compiles
//! blocks its extractor marks `verifiable`, and that filter currently excludes
//! **869 of 932** Rust blocks (94%): `extract-code-blocks.mjs` treats the
//! substring `// simplified` as a non-executable marker, and that exact line was
//! bulk-inserted as the first line of those blocks. C++ is excluded wholesale —
//! `VERIFIABLE_LANGUAGES = ['rust', 'python']`. So the docs' own gate compiles
//! ~44 Rust examples.
//!
//! A reader does not care that a block is annotated "simplified"; they copy it
//! and expect it to build. This harness therefore ignores that opt-out and
//! judges each block on whether it is *structurally* compilable, skipping only
//! blocks that are genuinely fragments (ellipses, placeholders, `rust,ignore`).
//!
//! # Running it
//!
//! ```bash
//! HORUS_DOCS_DIR=../horus-docs cargo test -p horus_manager \
//!     --test docs_examples -- --ignored --nocapture
//! ```
//!
//! Scope it while iterating with `HORUS_DOCS_FILTER=tutorials`.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;

// ─── Model ──────────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
struct Block {
    doc_file: String,
    line: usize,
    code: String,
    /// How this snippet has to be surrounded to become compilable Rust.
    ///
    /// A method body printed without its `impl`, or a few loose statements meant
    /// to be pasted into a `tick()`, is not standalone — but it is not
    /// unverifiable either. Supplying the context the prose implies is enough to
    /// put the horus calls inside it in front of the compiler, which is the
    /// whole point. Names the reader owns still fail to resolve, and those land
    /// in the advisory bucket exactly as they do for any other block.
    ctx: Ctx,
}

/// The scaffolding a block needs before it will compile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum Ctx {
    /// Self-contained: items at the top level, or statements in a function.
    #[default]
    Standalone,
    /// Loose statements — wrap in a function body.
    Statements,
    /// A bare method — wrap in `impl` on a synthetic type.
    Method,
}

/// Why a block is not compiled. Reported so coverage stays auditable — a filter
/// that silently swallows everything would make this suite vacuous.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Skip {
    /// `rust,ignore` — the rustdoc convention for "do not compile".
    RustdocIgnore,
    /// Contains `...`, so it is not valid Rust and never claimed to be.
    Ellipsis,
    /// `<your_project>`, `YOUR_KEY`, `PLACEHOLDER` — user must substitute.
    Placeholder,
    /// Too small to carry meaning (a type name, a single expression).
    Trivial,
    /// Loose statements meant to be pasted into surrounding code ("Add to
    /// `DisplayNode::tick()`"). They reference names from a scope that only
    /// exists in the prose, so compiling them standalone proves nothing.
    Fragment,
    /// Deliberately broken code shown to illustrate a mistake ("**The
    /// Problem:**", `// Error: the trait bound … is not satisfied`). These
    /// *must not* compile; requiring it would invert their meaning.
    CounterExample,
    /// A method (`fn tick(&mut self)`) printed without its `impl` block.
    MethodFragment,
    /// A bodyless signature printed as API reference, e.g.
    /// `pub fn send_goal_and_wait(&self, …) -> Result<A::Result, ActionError>`.
    /// It is not a program and cannot compile on its own.
    SignatureNotation,
    /// Brackets do not balance — an excerpt cut mid-block, e.g. an `impl` shown
    /// with its opening brace and no closing one. No wrapper can rescue this:
    /// it does not parse, and rustc reports only the *first* unclosed delimiter
    /// per crate before giving up, so leaving one in a batch hides every other
    /// diagnostic behind it.
    Unbalanced,
}

/// Whether `(`/`[`/`{` pair up across the whole snippet.
///
/// Strings, chars and comments are skipped, because a brace inside `"{}"` or a
/// commented-out line is not structure. Lifetimes (`&'a`) and the char literal
/// `\'` are the two places a naive quote scanner goes wrong, so both are
/// handled explicitly.
fn delimiters_balance(code: &str) -> bool {
    let b: Vec<char> = code.chars().collect();
    let mut stack: Vec<char> = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        match c {
            '/' if i + 1 < b.len() && b[i + 1] == '/' => {
                while i < b.len() && b[i] != '\n' {
                    i += 1;
                }
            }
            '/' if i + 1 < b.len() && b[i + 1] == '*' => {
                i += 2;
                while i + 1 < b.len() && !(b[i] == '*' && b[i + 1] == '/') {
                    i += 1;
                }
                i += 2;
            }
            '"' => {
                // Raw strings (r"…", r#"…"#) are rare in these snippets and a
                // plain scan handles the common `r"…"` correctly anyway.
                i += 1;
                while i < b.len() {
                    if b[i] == '\\' {
                        i += 2;
                        continue;
                    }
                    if b[i] == '"' {
                        break;
                    }
                    i += 1;
                }
                i += 1;
            }
            '\'' => {
                // A lifetime, not a char literal, if what follows is an
                // identifier that is not immediately closed by another quote.
                let is_char_lit = i + 2 < b.len() && ((b[i + 1] == '\\') || b[i + 2] == '\'');
                if is_char_lit {
                    i += 1;
                    while i < b.len() {
                        if b[i] == '\\' {
                            i += 2;
                            continue;
                        }
                        if b[i] == '\'' {
                            break;
                        }
                        i += 1;
                    }
                }
                i += 1;
            }
            '(' | '[' | '{' => {
                stack.push(c);
                i += 1;
            }
            ')' | ']' | '}' => {
                let want = match c {
                    ')' => '(',
                    ']' => '[',
                    _ => '{',
                };
                if stack.pop() != Some(want) {
                    return false;
                }
                i += 1;
            }
            _ => i += 1,
        }
    }
    stack.is_empty()
}

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

// ─── Extraction ─────────────────────────────────────────────────────────────

/// Pull ```rust fenced blocks out of one page.
fn rust_blocks(text: &str, rel: &str) -> Vec<(Block, Option<Skip>)> {
    let mut out = Vec::new();
    let mut cur: Option<(usize, String, bool)> = None; // (start line, code, is_ignore)

    for (idx, raw) in text.lines().enumerate() {
        let trimmed = raw.trim_start();
        if let Some(rest) = trimmed.strip_prefix("```") {
            match cur.take() {
                Some((start, code, is_ignore)) => {
                    let b = Block {
                        doc_file: rel.to_string(),
                        line: start,
                        code,
                        ctx: Ctx::Standalone,
                    };
                    let skip = if is_ignore {
                        Some(Skip::RustdocIgnore)
                    } else {
                        classify(&b.code)
                    };
                    out.push((b, skip));
                }
                None => {
                    let tag = rest.trim();
                    let lang = tag
                        .split([':', ' ', ','])
                        .next()
                        .unwrap_or("")
                        .to_ascii_lowercase();
                    if lang == "rust" {
                        // `rust,ignore` / `rust,no_run` / `rust,compile_fail`
                        let is_ignore = tag.split(',').skip(1).any(|s| {
                            matches!(s.trim(), "ignore" | "compile_fail" | "should_panic")
                        });
                        cur = Some((idx + 1, String::new(), is_ignore));
                    }
                }
            }
            continue;
        }
        if let Some((_, code, _)) = cur.as_mut() {
            code.push_str(raw);
            code.push('\n');
        }
    }
    out
}

/// The block with string/char-literal bodies and comment bodies blanked out.
///
/// `classify` tested `code.contains("...")` over the raw block to spot an
/// outline. A `...` inside a string is prose, not an omission:
/// `tutorials/02-motor-controller-rust.mdx` logs
/// `"Waiting for encoder feedback..."`, and that single string skipped the
/// tutorial's entire main example — three nodes, the PID loop and the
/// safe-state hook — as an outline. A deliberate typo introduced into it still
/// passed the suite.
///
/// Comment bodies are blanked for the same reason in reverse: `// ...` marks an
/// omission in prose but does not stop the block compiling, so such a block can
/// and should be held to the guarantee.
fn strip_strings_and_comments(code: &str) -> String {
    let b: Vec<char> = code.chars().collect();
    let mut out = String::with_capacity(code.len());
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        match c {
            '/' if i + 1 < b.len() && b[i + 1] == '/' => {
                while i < b.len() && b[i] != '\n' {
                    out.push(' ');
                    i += 1;
                }
            }
            '/' if i + 1 < b.len() && b[i + 1] == '*' => {
                out.push_str("  ");
                i += 2;
                while i + 1 < b.len() && !(b[i] == '*' && b[i + 1] == '/') {
                    out.push(if b[i] == '\n' { '\n' } else { ' ' });
                    i += 1;
                }
                out.push_str("  ");
                i += 2;
            }
            '"' => {
                out.push('"');
                i += 1;
                while i < b.len() {
                    if b[i] == '\\' {
                        out.push_str("  ");
                        i += 2;
                        continue;
                    }
                    if b[i] == '"' {
                        break;
                    }
                    out.push(' ');
                    i += 1;
                }
                out.push('"');
                i += 1;
            }
            '\'' => {
                // A lifetime, not a char literal, if what follows is an
                // identifier that is not immediately closed by another quote.
                let is_char_lit = i + 2 < b.len() && ((b[i + 1] == '\\') || b[i + 2] == '\'');
                out.push('\'');
                if is_char_lit {
                    i += 1;
                    while i < b.len() {
                        if b[i] == '\\' {
                            out.push_str("  ");
                            i += 2;
                            continue;
                        }
                        if b[i] == '\'' {
                            break;
                        }
                        out.push(' ');
                        i += 1;
                    }
                    out.push('\'');
                }
                i += 1;
            }
            _ => {
                out.push(c);
                i += 1;
            }
        }
    }
    out
}

/// Decide whether a block can be held to a compile guarantee.
///
/// Deliberately does **not** honour the docs' `// simplified` marker — see the
/// module docs.
fn classify(code: &str) -> Option<Skip> {
    let t = code.trim();
    // Specific reasons first: a block containing `...` is an outline whatever
    // its length, and reporting it as `Trivial` would misattribute the skip.
    // Only code counts — see strip_strings_and_comments.
    let bare = strip_strings_and_comments(t);
    if bare.contains("...") {
        return Some(Skip::Ellipsis);
    }
    let lower = t.to_ascii_lowercase();
    for p in ["<your_", "your_", "placeholder", "<path", "/path/to"] {
        if lower.contains(p) {
            return Some(Skip::Placeholder);
        }
    }
    if t.len() < 24 || t.lines().count() < 2 {
        return Some(Skip::Trivial);
    }
    if is_counter_example(t) {
        return Some(Skip::CounterExample);
    }
    if is_signature_notation(t) {
        return Some(Skip::SignatureNotation);
    }
    if has_bare_method(t) {
        return Some(Skip::MethodFragment);
    }
    if !is_self_contained(t) {
        return Some(Skip::Fragment);
    }
    None
}

/// A block that documents a *signature* rather than showing runnable code.
///
/// The Rust API pages print bodyless declarations under headings like
/// `#### send_goal_and_wait_with_feedback`:
///
/// ```text
/// pub fn send_goal_and_wait_with_feedback(
///     &self, goal: A::Goal, timeout: Duration,
/// ) -> Result<A::Result, ActionError>
/// ```
///
/// There is no body, so it can never compile; and because these use
/// `std::result::Result` with two parameters, compiling them under an injected
/// `horus::prelude::*` (whose `Result<T>` takes one) reported a two-generic
/// error the page never made.
fn is_signature_notation(code: &str) -> bool {
    let body_lines: Vec<&str> = code
        .lines()
        .map(str::trim_start)
        .filter(|l| !l.is_empty() && !l.starts_with("//") && !l.starts_with("#["))
        .collect();
    if body_lines.is_empty() {
        return false;
    }
    let starts_decl = body_lines[0].starts_with("pub fn ")
        || body_lines[0].starts_with("fn ")
        || body_lines[0].starts_with("pub async fn ");
    // A real definition opens a body; a signature reference never does.
    starts_decl && !code.contains('{')
}

/// A block whose own comments announce that it is broken on purpose.
///
/// The docs pair these with a working version ("**The Problem:**" then "**The
/// Fix:**"). Demanding that the broken half compile would be backwards.
fn is_counter_example(code: &str) -> bool {
    code.lines()
        .map(str::trim_start)
        .filter(|l| l.starts_with("//"))
        .any(|l| {
            let c = l.trim_start_matches('/').trim().to_ascii_lowercase();
            c.starts_with("error")
                || c.starts_with("wrong")
                || c.starts_with("bad")
                || c.starts_with("don't")
                || c.starts_with("dont")
                || c.starts_with("fail")
                || c.starts_with("compile error")
                || c.starts_with("this will not compile")
                || c.starts_with("does not compile")
        })
}

/// A method printed without its surrounding `impl`.
///
/// `fn tick(&mut self)` is valid Rust only inside an `impl`; on its own rustc
/// rejects it with "`self` parameter is only allowed in associated functions".
/// The docs show these to discuss a single method's body.
fn has_bare_method(code: &str) -> bool {
    if code.contains("impl ") || code.contains("trait ") {
        return false; // the impl is present, so the method is fine
    }
    code.lines().map(str::trim_start).any(|l| {
        (l.starts_with("fn ") || l.starts_with("pub fn ") || l.starts_with("async fn "))
            && l.contains("self")
    })
}

/// Whether a snippet presents itself as a complete unit a reader could build.
///
/// The bar is deliberately conservative. A block that defines something —
/// a `main`, a node, a type with an impl — is a promise that it works. Loose
/// statements under prose like "add this to `tick()`" are not, and holding them
/// to a compile guarantee produces false alarms that get the whole gate turned
/// off. Bare `use` lines do not count: fragments routinely open with an import.
fn is_self_contained(code: &str) -> bool {
    // An `impl` whose target type is never defined here is illustrating a shape,
    // not offering a program: `impl Node for SafetyMonitor { … }` with no
    // `SafetyMonitor` in sight cannot compile anywhere, and the reader knows it.
    if !impl_targets_are_defined(code) {
        return false;
    }
    let has = |k: &str| code.contains(k);
    if has("fn main") || has("node!") || has("message!") || has("service!") {
        return true;
    }
    let defines_type = has("struct ") || has("enum ") || has("trait ") || has("impl ");
    let defines_fn = code
        .lines()
        .map(str::trim_start)
        .any(|l| l.starts_with("fn ") || l.starts_with("pub fn ") || l.starts_with("async fn "));
    defines_type || defines_fn
}

/// Every `impl … for Type` / `impl Type` target must be defined in the block.
fn impl_targets_are_defined(code: &str) -> bool {
    for line in code.lines().map(str::trim_start) {
        let Some(rest) = line.strip_prefix("impl ") else {
            continue;
        };
        // `impl<T> Trait for Type {` → drop generics on the impl itself
        let rest = rest.strip_prefix('<').map_or(rest, |r| {
            r.split_once('>').map_or(r, |(_, after)| after.trim_start())
        });
        // `Trait for Type` → `Type`; plain `impl Type` → `Type`
        let target = rest.rsplit(" for ").next().unwrap_or(rest);
        let name = target
            .split([' ', '{', '<', '(', ':'])
            .find(|s| !s.is_empty())
            .unwrap_or("");
        if name.is_empty() {
            continue;
        }
        let defined = [
            format!("struct {name}"),
            format!("enum {name}"),
            format!("type {name}"),
            format!("union {name}"),
            format!("trait {name}"),
        ]
        .iter()
        .any(|d| code.contains(d.as_str()));
        if !defined {
            return false;
        }
    }
    true
}

fn collect(docs: &Path, filter: Option<&str>) -> (Vec<Block>, BTreeMap<Skip, usize>) {
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
    let mut files = Vec::new();
    walk(&docs.join("content/docs"), &mut files);
    files.sort();

    let mut keep = Vec::new();
    let mut skipped: BTreeMap<Skip, usize> = BTreeMap::new();
    for f in files {
        let rel = f
            .strip_prefix(docs)
            .unwrap_or(&f)
            .to_string_lossy()
            .replace('\\', "/");
        if filter.is_some_and(|needle| !rel.contains(needle)) {
            continue;
        }
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        for (mut b, skip) in rust_blocks(&text, &rel) {
            match skip {
                // These two were skipped for being non-standalone, which is
                // true and beside the point: the prose says what surrounds
                // them, so supply it and compile them anyway. Counted in both
                // places so the coverage report stays honest about what needed
                // help to build.
                Some(s @ (Skip::MethodFragment | Skip::Fragment)) => {
                    *skipped.entry(s).or_default() += 1;
                    if delimiters_balance(&b.code) {
                        b.ctx = if s == Skip::MethodFragment {
                            Ctx::Method
                        } else {
                            Ctx::Statements
                        };
                        keep.push(b);
                    } else {
                        *skipped.entry(Skip::Unbalanced).or_default() += 1;
                    }
                }
                Some(s) => *skipped.entry(s).or_default() += 1,
                None if !delimiters_balance(&b.code) => {
                    *skipped.entry(Skip::Unbalanced).or_default() += 1;
                }
                None => keep.push(b),
            }
        }
    }
    (keep, skipped)
}

// ─── Wrapping ───────────────────────────────────────────────────────────────

/// True when the snippet's top level is Rust *items* (so it can be a module
/// body) rather than loose statements.
fn is_item_level(code: &str) -> bool {
    // A `let` in column 0 is a statement, and statements cannot sit in a module
    // body. Docs mix the two freely ("here is the type, here is how you use
    // it"), so such a block has to be wrapped in a function instead — where
    // `use`, `struct` and `let` are all legal.
    let has_top_level_statement = code
        .lines()
        .any(|l| !l.starts_with(char::is_whitespace) && l.trim_start().starts_with("let "));
    if has_top_level_statement {
        return false;
    }

    code.lines()
        .map(str::trim_start)
        .filter(|l| !l.is_empty() && !l.starts_with("//") && !l.starts_with("#["))
        .take(6)
        .any(|l| {
            [
                "use ",
                "pub ",
                "fn ",
                "struct ",
                "enum ",
                "impl ",
                "trait ",
                "mod ",
                "const ",
                "static ",
                "type ",
                "macro_rules!",
                "node!",
                "message!",
                "extern ",
            ]
            .iter()
            .any(|k| l.starts_with(k))
        })
}

/// Imports a reader has in scope but the snippet does not restate.
///
/// Doc pages establish `use horus::prelude::*;` (and serde's derives) once and
/// then show later snippets without repeating them. Judging a block on imports
/// the surrounding page already provided would flag the page's *style*, not a
/// defect, and drown the real failures. Only added when absent, since a
/// duplicate named import is itself an error (E0252).
/// Only `horus::prelude::*` is implied — every page establishes it in its first
/// block, so a later block omitting it is a formatting choice, not a defect.
///
/// serde's derives are deliberately **not** injected. `horus::prelude` does not
/// re-export `Serialize`/`Deserialize` (in `horus/src/lib.rs` they appear only
/// inside doc comments), and no tutorial page ever writes
/// `use serde::{Serialize, Deserialize};`. Injecting them papered over the very
/// first code block of tutorial 01 failing to compile — precisely the breakage a
/// reader hits on their first attempt.
fn implied_prelude(code: &str) -> String {
    if code.contains("use horus::prelude") {
        return String::new();
    }
    // A block written against std's two-generic Result must not have horus's
    // one-generic `Result<T>` alias shadow it. `plugins/creating-plugins.mdx:173`
    // is a clap CLI returning `Result<(), Box<dyn std::error::Error>>`; injecting
    // the prelude turned its correct signature into a two-generic error the page
    // never made.
    //
    // Keyed on the std error idiom rather than on "does the block mention
    // horus", because horus blocks routinely use `Result<()>` and `Error::node`
    // without spelling the crate name — dropping the prelude for those broke
    // eight pages that had been passing.
    // Matched on the error idiom specifically, not on any `Box<dyn`:
    // `tutorials/06-write-a-driver.mdx:315` has the comment
    // `// hardware::load() returns Vec<(String, Box<dyn Node>)>`, and a bare
    // `Box<dyn` test suppressed the prelude there, turning its horus
    // `Result<()>` into a two-generic error.
    if code.contains("std::error::Error") || code.contains("Box<dyn Error") {
        return String::new();
    }
    "    use horus::prelude::*;\n".to_string()
}

/// Crates a block imports that a generated project does not depend on.
///
/// Distinct from an unresolved *name*: a reader can define `SensorNode`
/// themselves, but they cannot conjure a crate their manifest never declares.
/// `use horus_robotics::prelude::*` inside a `horus new -r` project is a hard
/// stop, so this gates rather than merely reporting.
/// Crate names the page declares in a `[dependencies]` block.
///
/// Pages that reach outside the framework normally show the manifest first —
/// `recipes/real-hardware.mdx:31` prints `[dependencies]` with
/// `i2cdev = { version = "0.6", source = "crates.io" }` immediately above the
/// code that imports it. That *is* the instruction to add the dependency, so
/// the import is satisfied and flagging it reports the page's teaching order
/// rather than a defect.
fn declared_dependencies(page_source: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();

    // `horus add <crate>` is the other way a page tells the reader to install
    // something, and the more common one outside recipe pages.
    for line in page_source.lines() {
        let t = line.trim();
        let Some(rest) = t.strip_prefix("horus add ") else {
            continue;
        };
        if let Some(name) = rest.split_whitespace().next() {
            // strip a `name@version` pin
            let name = name.split('@').next().unwrap_or(name);
            if !name.is_empty()
                && name
                    .chars()
                    .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
            {
                out.insert(name.replace('-', "_"));
                out.insert(name.to_string());
            }
        }
    }

    let mut in_deps = false;
    for line in page_source.lines() {
        let t = line.trim();
        if t.starts_with('[') {
            in_deps = t.contains("dependencies");
            continue;
        }
        if !in_deps || t.is_empty() || t.starts_with('#') || t.starts_with("```") {
            if t.starts_with("```") {
                in_deps = false;
            }
            continue;
        }
        if let Some(eq) = t.find('=') {
            let name = t[..eq].trim().trim_matches('"');
            if !name.is_empty()
                && name
                    .chars()
                    .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
            {
                // Cargo and horus accept either spelling on the import side.
                out.insert(name.replace('-', "_"));
                out.insert(name.to_string());
            }
        }
    }
    out
}

fn missing_dependencies(code: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for line in code.lines().map(str::trim_start) {
        let Some(rest) = line.strip_prefix("use ") else {
            continue;
        };
        let root = rest
            .trim_start_matches("::")
            .split([':', ';', ' ', '{', ','])
            .find(|s| !s.is_empty())
            .unwrap_or("");
        if root.is_empty() || ALWAYS_AVAILABLE.contains(&root) || PROJECT_DEPS.contains(&root) {
            continue;
        }
        if !out.iter().any(|x| x == root) {
            out.push(root.to_string());
        }
    }
    out
}

/// Wrap one block so it can live beside others in a single crate.
///
/// Items become a module; loose statements become a function body inside one.
/// `fn main` inside a module is just an ordinary function, so blocks that are
/// whole programs still type-check without becoming entry points.
fn wrap(idx: usize, b: &Block) -> String {
    let name = format!("docblock_{idx}");
    // A bare method needs an `impl` to sit in. The receiver decides the shape:
    // `&mut self` / `&self` need a type, an associated function does not care.
    // `_DocSelf` carries the fields a doc method is likely to touch only in the
    // sense that it carries none — an unresolved `self.foo` is a
    // reader-owned-name failure, which the advisory split already handles.
    if b.ctx == Ctx::Method {
        return format!(
            "// {}:{}\n#[allow(dead_code, unused_imports, unused_variables, unused_mut)]\nmod {name} {{\n    use horus::prelude::*;\n    struct _DocSelf;\n    impl _DocSelf {{\n{}\n    }}\n}}\n",
            b.doc_file, b.line, b.code
        );
    }
    if b.ctx == Ctx::Statements {
        // The closure returns horus's own `Result`, not `Box<dyn Error>`. These
        // fragments are lifted out of horus functions, so they contain bare
        // `return Err(Error::node(..))` as often as they contain `?`, and a
        // boxed error type accepts the second while rejecting the first — which
        // reported correct documentation as a contradiction. `?` still works on
        // anything horus has a `From` impl for, which is what the surrounding
        // function would have offered anyway.
        return format!(
            "// {}:{}\n#[allow(dead_code, unused_imports, unused_variables, unused_mut)]\nmod {name} {{\n    use horus::prelude::*;\n    fn _snippet() {{\n        let _f = || -> horus::error::Result<()> {{\n{}\n            Ok(())\n        }};\n    }}\n}}\n",
            b.doc_file, b.line, b.code
        );
    }
    if is_item_level(&b.code) {
        format!(
            "// {}:{}\n#[allow(dead_code, unused_imports, unused_variables, unused_mut)]\nmod {name} {{\n{}{}\n}}\n",
            b.doc_file,
            b.line,
            implied_prelude(&b.code),
            b.code
        )
    } else {
        // The inner closure returns `Result`, because doc snippets routinely use
        // `?` on `Topic::new(...)`. A plain `fn _snippet()` body would reject it
        // with "cannot use the `?` operator in a function that returns `()`".
        // `use`, `struct` and `impl` are all legal in statement position, so
        // mixed item/statement blocks compile here too.
        format!(
            "// {}:{}\n#[allow(dead_code, unused_imports, unused_variables, unused_mut)]\nmod {name} {{\n    use horus::prelude::*;\n    fn _snippet() {{\n        let _f = || -> std::result::Result<(), Box<dyn std::error::Error>> {{\n{}\n            Ok(())\n        }};\n    }}\n}}\n",
            b.doc_file, b.line, b.code
        )
    }
}

// ─── Compiling ──────────────────────────────────────────────────────────────

/// Crates a `horus new -r` project can actually `use`.
///
/// Kept identical to what `cargo_gen` emits — see `write_horus_path_deps` and
/// `write_implicit_deps` in `horus_manager/src/cargo_gen.rs`. A doc that tells
/// the reader to import anything outside this set is telling them to import a
/// crate they do not have.
const PROJECT_DEPS: &[&str] = &["horus", "horus_core", "horus_macros", "serde"];

/// Crates always available regardless of the manifest.
const ALWAYS_AVAILABLE: &[&str] = &["std", "core", "alloc", "crate", "self", "super"];

/// Imports a block makes that neither the generated project nor its own page
/// provides. This is the gating form; `missing_dependencies` is the raw scan.
fn unsatisfied_dependencies(b: &Block, docs: &Path) -> Vec<String> {
    let raw = missing_dependencies(&b.code);
    if raw.is_empty() {
        return raw;
    }
    let page = std::fs::read_to_string(docs.join(&b.doc_file)).unwrap_or_default();
    let declared = declared_dependencies(&page);
    raw.into_iter().filter(|d| !declared.contains(d)).collect()
}

/// Build a scratch crate containing `blocks` and run `cargo check` on it.
/// Returns `(ok, stderr, line_spans)` where `line_spans[i]` is the 1-indexed
/// line range of block `i` inside the generated `src/lib.rs`.
fn check_batch(dir: &Path, blocks: &[(usize, &Block)]) -> (bool, String, Vec<(usize, usize)>) {
    let horus = repo_root();
    let h = horus.display();
    let manifest = format!(
        r#"[package]
name = "docs_examples_scratch"
version = "0.0.0"
edition = "2021"

# Own workspace: this crate must not be adopted by the horus workspace.
[workspace]

[lib]
path = "src/lib.rs"

[dependencies]
horus = {{ path = "{h}/horus" }}
horus_core = {{ path = "{h}/horus_core" }}
horus_macros = {{ path = "{h}/horus_macros" }}
serde = {{ version = "1", features = ["derive"] }}
# NOTHING ELSE. This list must mirror exactly what cargo_gen writes for a
# `horus new -r` project (see write_horus_path_deps + write_implicit_deps) —
# `PROJECT_DEPS` below pins that, and a test asserts the two agree.
#
# Adding conveniences here silently weakens every assertion in this file. An
# earlier revision carried `anyhow` and `horus-robotics`, which made the scratch
# crate strictly more capable than a real user's project and hid two confirmed
# blockers: the flagship `use horus_robotics::prelude::*` in
# concepts/what-is-horus.mdx and tutorials/02, which no generated project can
# resolve. Model the reader's environment, not a comfortable one.

# Declaring [workspace] above makes this a workspace root, so the root
# workspace's patch tables do not carry over and the git-sourced horus crates
# would fail to resolve `horus_core`. Same reason cargo_gen emits these into
# generated projects — see write_patch_sections there.
[patch."https://github.com/softmata/horus-robotics.git"]
horus_core = {{ path = "{h}/horus_core" }}
horus_types = {{ path = "{h}/horus_types" }}
horus_macros = {{ path = "{h}/horus_macros" }}

[patch."https://github.com/softmata/horus-tf.git"]
horus_core = {{ path = "{h}/horus_core" }}
horus_macros = {{ path = "{h}/horus_macros" }}
"#
    );
    std::fs::create_dir_all(dir.join("src")).unwrap();
    std::fs::write(dir.join("Cargo.toml"), manifest).unwrap();

    let mut src = String::from("#![allow(dead_code, unused)]\n");
    let mut spans = Vec::new();
    for (i, b) in blocks {
        let start = src.lines().count() + 1;
        src.push_str(&wrap(*i, b));
        spans.push((start, src.lines().count()));
    }
    std::fs::write(dir.join("src/lib.rs"), &src).unwrap();

    let out = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()))
        .args(["check", "--quiet", "--message-format", "short"])
        .current_dir(dir)
        // Share the workspace target dir so horus's deps are compiled once.
        .env("CARGO_TARGET_DIR", horus.join("target/docs-verify"))
        .output()
        .expect("cargo check runs");

    (
        out.status.success(),
        String::from_utf8_lossy(&out.stderr).to_string(),
        spans,
    )
}

/// rustc codes that mean the snippet contradicts a real horus API.
///
/// These cannot be explained away by missing reader context: the type resolved,
/// and *then* rustc found the doc using it wrongly — a field that does not
/// exist, a trait method with the wrong signature, a struct literal missing
/// required fields. Every one is something a reader hits verbatim, so these gate.
///
/// Unresolved-name codes (E0433/E0425/E0412/E0432 …) are deliberately absent:
/// a name can be missing simply because the reader defines it in their own
/// project or an earlier page, so they are reported rather than enforced.
const API_MISMATCH_CODES: &[&str] = &[
    "E0046", // not all trait items implemented
    "E0050", // method has wrong number of parameters
    "E0053", // method has an incompatible type for trait
    "E0061", // wrong number of arguments
    "E0063", // missing fields in struct initializer
    "E0107", // wrong number of generic arguments
    "E0277", // trait bound not satisfied
    "E0308", // mismatched types
    "E0560", // struct has no such field
    "E0609", // no field on type
];

fn is_api_mismatch(error: &str) -> bool {
    // `_DocSelf` is the synthetic receiver supplied to a bare method fragment.
    // It has no fields by design, so every `self.whatever` in such a block
    // produces E0609 "no field … on type … _DocSelf". That is a name the
    // reader's own struct owns — the same class as an undefined local type —
    // not a wrong field on a horus struct, and gating on it would fail dozens
    // of blocks that document perfectly good code.
    if error.contains("_DocSelf") {
        return false;
    }
    API_MISMATCH_CODES.iter().any(|c| error.contains(c))
}

/// Names rustc reported as undefined, pulled out of `cannot find …` messages.
fn undefined_names(errors: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    for e in errors {
        // Take the first backtick-quoted token after any "cannot find" phrasing.
        // Matching fixed prefixes missed rustc's compound wordings — notably
        // "cannot find struct, variant or union type `MotorState`", whose comma
        // defeated a "cannot find struct `" marker and left the block reported
        // as a real finding.
        let mut rest = e.as_str();
        while let Some(pos) = rest.find("cannot find") {
            rest = &rest[pos + "cannot find".len()..];
            let Some(open) = rest.find('`') else { break };
            let after = &rest[open + 1..];
            if let Some(name) = after.split('`').next() {
                if !name.is_empty() && name.len() < 64 {
                    out.push(name.to_string());
                }
            }
            rest = after;
        }
        // `use of unresolved module or unlinked crate `ort``
        let mut rest = e.as_str();
        while let Some(pos) = rest.find("unlinked crate `") {
            rest = &rest[pos + "unlinked crate `".len()..];
            if let Some(name) = rest.split('`').next() {
                if !name.is_empty() {
                    out.push(name.to_string());
                }
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// A "no field X on type T" / "T has no field named X" error where the page
/// defines its own `T`, so the prelude's same-named type was picked only because
/// the block was compiled alone.
/// True when `page_source` declares `<keyword> <name>` as a whole word.
///
/// A plain `contains` was the original test, and it silently excused real
/// defects. A page that declares `struct ImuSensor` contains the substring
/// `struct Imu`, so every wrong-field error against the standard `Imu` message
/// was absolved as "the page defines its own Imu" — the block never compiled
/// and the suite still passed. The same shadowing holds for `Pose`/`PoseStamped`
/// and `Twist`/`TwistWithCovariance`. The declared name must end where the
/// searched-for name ends.
fn declares(page_source: &str, keyword: &str, name: &str) -> bool {
    let needle = format!("{keyword} {name}");
    let bytes = page_source.as_bytes();
    let mut from = 0;
    while let Some(pos) = page_source[from..].find(&needle) {
        let end = from + pos + needle.len();
        if !bytes
            .get(end)
            .is_some_and(|c| c.is_ascii_alphanumeric() || *c == b'_')
        {
            return true;
        }
        from = end;
    }
    false
}

fn is_shadowed_field_error(error: &str, page_source: &str) -> bool {
    if !error.contains("E0560") && !error.contains("E0609") {
        return false;
    }
    // The type is the last backtick-quoted path in the message.
    let ty = error
        .split('`')
        .filter(|s| !s.is_empty())
        .filter(|s| s.contains("::") || s.chars().next().is_some_and(char::is_uppercase))
        .filter_map(|s| s.rsplit("::").next())
        .find(|s| s.chars().next().is_some_and(char::is_uppercase));
    let Some(ty) = ty else { return false };
    declares(page_source, "struct", ty) || declares(page_source, "enum", ty)
}

/// True when every failure is an identifier the *same page* defines elsewhere.
///
/// Tutorials build up across blocks: `SensorNode` is defined in step 2 and used
/// by the `fn main` in step 5. In isolation that `main` cannot compile, but a
/// reader working through the page has the type. Blocks are compiled separately
/// (concatenating a page would collide on the before/after versions of a type
/// that pages routinely show), so this recovers the page context after the fact.
fn only_missing_page_local_names(errors: &[String], page_source: &str) -> bool {
    if errors.is_empty() {
        return false;
    }
    // Absolve the block only when EVERY error is explained by page context.
    // The two explanations are checked per-error rather than as separate
    // all-or-nothing passes: a block often mixes them (an undefined type from an
    // earlier step *and* a field error against a type the page defines), and
    // requiring one uniform kind let those through as false findings.
    errors.iter().all(|e| {
        is_shadowed_field_error(e, page_source) || is_page_local_name_error(e, page_source)
    })
}

/// A "cannot find X" error where the page defines `X` in another block.
fn is_page_local_name_error(error: &str, page_source: &str) -> bool {
    if !error.contains("cannot find") && !error.contains("use of undeclared") {
        return false;
    }
    let names = undefined_names(std::slice::from_ref(&error.to_string()));
    if names.is_empty() {
        return false;
    }
    names.iter().all(|n| {
        [
            "struct", "enum", "trait", "type", "fn", "mod",
            // A page may only ever `impl Node for MotorController`, treating the
            // type as one the reader already has in their project.
            "impl", "for",
        ]
        .iter()
        .any(|kw| declares(page_source, kw, n))
    })
}

/// Map `src/lib.rs:LINE: error…` diagnostics back to the blocks they came from.
fn attribute(
    stderr: &str,
    blocks: &[(usize, &Block)],
    spans: &[(usize, usize)],
) -> BTreeMap<usize, Vec<String>> {
    let mut per_block: BTreeMap<usize, Vec<String>> = BTreeMap::new();
    for line in stderr.lines() {
        if !line.contains("error") {
            continue;
        }
        // short format: `src/lib.rs:123:9: error[E0433]: …`
        let Some(rest) = line.split("src/lib.rs:").nth(1) else {
            continue;
        };
        let Some(num) = rest.split(':').next().and_then(|n| n.parse::<usize>().ok()) else {
            continue;
        };
        if let Some(pos) = spans.iter().position(|(s, e)| num >= *s && num <= *e) {
            per_block
                .entry(blocks[pos].0)
                .or_default()
                .push(line.trim().to_string());
        }
    }
    per_block
}

// ─── Tests ──────────────────────────────────────────────────────────────────

/// Every documented Rust example that claims to be complete must compile.
#[test]
#[ignore = "slow: compiles every Rust example in the docs; run in the scheduled docs-contract job"]
fn documented_rust_examples_compile() {
    let docs = docs_dir().expect(
        "set HORUS_DOCS_DIR=/path/to/horus-docs (or check horus-docs out beside this repo)",
    );
    let filter = std::env::var("HORUS_DOCS_FILTER").ok();
    let (blocks, skipped) = collect(&docs, filter.as_deref());

    let total_skipped: usize = skipped.values().sum();
    eprintln!("docs: {}", docs.display());
    eprintln!(
        "rust blocks: {} compiled, {} skipped ({})",
        blocks.len(),
        total_skipped,
        skipped
            .iter()
            .map(|(k, v)| format!("{k:?}={v}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    assert!(
        !blocks.is_empty(),
        "no compilable Rust blocks found — the extractor or the filter is broken, \
         which would make this test vacuous"
    );

    let tmp = tempfile::tempdir().expect("temp dir");
    let indexed: Vec<(usize, &Block)> = blocks.iter().enumerate().collect();

    // Batching keeps `cargo check` invocations down, but a batch is one crate,
    // and rustc stops after name resolution if resolution failed anywhere in
    // it. One block naming a type the reader owns therefore suppresses every
    // type error in the other thirty-nine — and which block that is depends on
    // the batch boundary, so the same docs reported a different set of API
    // contradictions run to run. That is a flaky gate, which is worse than no
    // gate.
    //
    // So: compile in rounds. Each round batches whatever is still unexplained,
    // records every block that produced a diagnostic, and drops those from the
    // next round. Round one clears out the resolution failures; round two type
    // checks the survivors that they were hiding. It converges when a round
    // finds nothing new, which is also the proof that nothing is still masked.
    // Two or three rounds in practice, and the batch count shrinks each time.
    const BATCH: usize = 40;
    const MAX_ROUNDS: usize = 8;
    let mut failures: BTreeMap<usize, Vec<String>> = BTreeMap::new();
    let mut pending: Vec<(usize, &Block)> = indexed.clone();

    for round in 0..MAX_ROUNDS {
        let mut newly_failed: Vec<usize> = Vec::new();
        for (n, chunk) in pending.chunks(BATCH).enumerate() {
            let dir = tmp.path().join(format!("r{round}b{n}"));
            let (ok, stderr, spans) = check_batch(&dir, chunk);
            if ok {
                continue;
            }
            let attributed = attribute(&stderr, chunk, &spans);
            if attributed.is_empty() {
                // A crate-level failure (not attributable to a line) — bisect.
                for one in chunk {
                    let d = tmp.path().join(format!("r{round}b{n}_{}", one.0));
                    let (ok1, err1, sp1) = check_batch(&d, std::slice::from_ref(one));
                    if !ok1 {
                        let a = attribute(&err1, std::slice::from_ref(one), &sp1);
                        failures.entry(one.0).or_default().extend(
                            a.get(&one.0).cloned().unwrap_or_else(|| {
                                vec![err1.lines().take(3).collect::<Vec<_>>().join(" | ")]
                            }),
                        );
                        newly_failed.push(one.0);
                    }
                }
            } else {
                for (idx, errs) in attributed {
                    failures.entry(idx).or_default().extend(errs);
                    newly_failed.push(idx);
                }
            }
        }
        eprintln!(
            "round {round}: checked {} block(s), {} newly failing, {} total",
            pending.len(),
            newly_failed.len(),
            failures.len()
        );
        if newly_failed.is_empty() {
            break;
        }
        assert!(
            round + 1 < MAX_ROUNDS,
            "still finding new failures after {MAX_ROUNDS} rounds — masking has not              converged, so the report below would be incomplete"
        );
        pending.retain(|(idx, _)| !newly_failed.contains(idx));
    }

    // Drop blocks whose only complaint is a name the rest of their page defines.
    let mut page_cache: BTreeMap<String, String> = BTreeMap::new();
    let mut page_scoped = 0usize;
    failures.retain(|idx, errs| {
        let b = &blocks[*idx];
        let page = page_cache
            .entry(b.doc_file.clone())
            .or_insert_with(|| std::fs::read_to_string(docs.join(&b.doc_file)).unwrap_or_default());
        let keep = !only_missing_page_local_names(errs, page);
        if !keep {
            page_scoped += 1;
        }
        keep
    });
    if page_scoped > 0 {
        eprintln!(
            "{page_scoped} block(s) referenced types defined elsewhere on their page (not counted)"
        );
    }

    // Split enforcement from reporting. An unresolved name may just be a type
    // the reader owns; a wrong field on a horus struct, or an import of a crate
    // the generated project never depends on, is unambiguously wrong.
    let (gating, advisory): (Vec<_>, Vec<_>) = failures.iter().partition(|(idx, errs)| {
        errs.iter().any(|e| is_api_mismatch(e))
            || !unsatisfied_dependencies(&blocks[**idx], &docs).is_empty()
    });

    if !advisory.is_empty() {
        eprintln!(
            "\n{} example(s) did not compile only because of names defined outside the \
             block (reader's project, or an earlier page). Advisory, not enforced:",
            advisory.len()
        );
        for (idx, errs) in &advisory {
            let b = &blocks[**idx];
            eprintln!("  {}:{}  {}", b.doc_file, b.line, errs[0]);
        }
    }

    if !gating.is_empty() {
        let mut report = String::new();
        for (idx, errs) in &gating {
            let b = &blocks[**idx];
            report.push_str(&format!("\n  {}:{}\n", b.doc_file, b.line));
            let missing = unsatisfied_dependencies(b, &docs);
            if !missing.is_empty() {
                report.push_str(&format!(
                    "      imports {missing:?} — not a dependency of a `horus new -r` project\n"
                ));
                if missing.iter().any(|m| m == "horus_robotics") {
                    report.push_str(
                        "      (horus/src/lib.rs re-exports horus_robotics::prelude::* through \
                         horus::prelude, so the types exist — only the import path is wrong. \
                         Either write `use horus::prelude::*;` in the doc, or have cargo_gen \
                         add horus-robotics to generated manifests.)\n",
                    );
                }
            }
            for e in errs.iter().filter(|e| is_api_mismatch(e)).take(3) {
                report.push_str(&format!("      {e}\n"));
            }
        }
        panic!(
            "{} documented Rust example(s) contradict the real horus API:{report}\n\
             These resolved against horus and then failed — a field, signature or \
             trait bound the docs get wrong. A reader copying them cannot build. \
             Fix the example, or mark the block ```rust,ignore if it is illustrative.\n\
             ({} of {} compiled examples had failures overall; the rest are advisory.)",
            gating.len(),
            failures.len(),
            blocks.len()
        );
    }

    eprintln!(
        "no documented Rust example contradicts the horus API ({} compiled, {} advisory)",
        blocks.len(),
        advisory.len()
    );
}

// ─── Extractor unit tests (fast, always run) ────────────────────────────────

#[cfg(test)]
mod extractor {
    use super::*;

    /// `PROJECT_DEPS` must stay equal to the dependency set `cargo_gen` writes.
    ///
    /// If cargo_gen starts emitting another crate and this list does not follow,
    /// the sweep reports a missing dependency users do not actually have — and
    /// worse, if this list gains a crate cargo_gen does not emit, the sweep
    /// silently stops catching that whole class of breakage.
    #[test]
    fn project_deps_match_cargo_gen() {
        let src =
            std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/cargo_gen.rs"))
                .expect("cargo_gen.rs is readable");

        // write_horus_path_deps iterates this exact literal list.
        let emitted = src
            .split("for dep_name in &[")
            .nth(1)
            .and_then(|s| s.split(']').next())
            .expect("cargo_gen still iterates a literal dep list");
        for d in ["horus", "horus_core", "horus_macros"] {
            assert!(
                emitted.contains(&format!("\"{d}\"")),
                "cargo_gen no longer emits {d}; update PROJECT_DEPS"
            );
            assert!(
                PROJECT_DEPS.contains(&d),
                "cargo_gen emits {d} but PROJECT_DEPS omits it"
            );
        }
        // write_implicit_deps adds serde.
        assert!(
            src.contains("fn write_implicit_deps") && src.contains("serde"),
            "cargo_gen no longer adds serde implicitly; update PROJECT_DEPS"
        );
        assert!(PROJECT_DEPS.contains(&"serde"));
    }

    #[test]
    fn shadowed_field_error_is_recognized() {
        // tutorials/02-motor-controller.mdx defines its own MotorCommand; the
        // prelude's same-named type is only picked because blocks compile alone.
        let e = "src/lib.rs:883:29: error[E0609]: no field `velocity` on type \
                 `horus::prelude::MotorCommand`: this is an associated function, \
                 not a method, unknown field";
        let page = "#[repr(C)]\nstruct MotorCommand {\n    velocity: f32,\n}\n";
        assert!(
            is_shadowed_field_error(e, page),
            "should absolve a field error on a page-defined type"
        );
        // A type the page does NOT define stays a real finding.
        assert!(!is_shadowed_field_error(e, "no definitions here"));
    }

    #[test]
    fn an_ellipsis_in_prose_does_not_skip_the_block() {
        // Regression: tutorials/02-motor-controller-rust.mdx logs
        // "Waiting for encoder feedback...". classify() tested the raw block
        // for "...", so that one string skipped the tutorial's entire main
        // example as an outline — a deliberate typo in it still passed.
        let in_string = "fn main() {\n    println!(\"Waiting...\");\n}";
        assert_eq!(
            classify(in_string),
            None,
            "a `...` inside a string is prose"
        );

        let in_comment = "fn main() {\n    // more setup ...\n    let x = 1;\n}";
        assert_eq!(
            classify(in_comment),
            None,
            "a `...` in a comment does not stop the block compiling"
        );

        // A real outline is still skipped.
        let outline = "fn main() {\n    let cfg = ...;\n    run(cfg);\n}";
        assert_eq!(classify(outline), Some(Skip::Ellipsis));
    }

    #[test]
    fn strip_strings_and_comments_blanks_only_prose() {
        let stripped = strip_strings_and_comments("let s = \"a...b\"; let n = 1;");
        assert!(!stripped.contains("..."), "string body must be blanked");
        assert!(
            stripped.contains("let n = 1;"),
            "code must survive: {stripped}"
        );

        let stripped = strip_strings_and_comments("let n = 1; // trailing ...\nlet m = 2;");
        assert!(!stripped.contains("..."));
        assert!(stripped.contains("let m = 2;"));

        // Bare code is untouched.
        assert!(strip_strings_and_comments("let x = ...;").contains("..."));
        // A lifetime is not a char literal, so what follows it must survive.
        let lt = strip_strings_and_comments("fn f<'a>(x: &'a str) -> &'a str { x }");
        assert!(lt.contains("str"), "lifetime scan ate the code: {lt}");
    }

    #[test]
    fn a_longer_type_name_does_not_excuse_a_field_error() {
        // Regression: tutorials/01-sensor-node-rust.mdx defines `struct ImuSensor`
        // and uses the prelude's `Imu`. A `contains("struct Imu")` test matched
        // `struct ImuSensor`, so a genuinely wrong field on `Imu` was reported as
        // "referenced types defined elsewhere on their page" and the suite passed
        // with an example that did not compile.
        let e = "src/lib.rs:12:9: error[E0609]: no field `linear_accel` on type `Imu`";
        assert!(
            !is_shadowed_field_error(e, "struct ImuSensor {\n    imu: Topic<Imu>,\n}"),
            "a struct whose name merely starts with the message name must not absolve it"
        );
        // The page really defining the type still absolves it.
        assert!(is_shadowed_field_error(
            e,
            "struct Imu {\n    linear_accel: f64,\n}"
        ));
        // And the boundary check holds for the name-resolution path too.
        let missing = "src/lib.rs:3:5: error[E0433]: cannot find struct `Imu` in this scope";
        assert!(!is_page_local_name_error(missing, "struct ImuSensor {}"));
        assert!(is_page_local_name_error(missing, "struct Imu {}"));
    }

    #[test]
    fn declares_requires_a_word_boundary() {
        assert!(declares("struct Imu {}", "struct", "Imu"));
        assert!(declares("struct Imu<T> {}", "struct", "Imu"));
        assert!(declares("impl Node for Robot {}", "for", "Robot"));
        assert!(!declares("struct ImuSensor {}", "struct", "Imu"));
        assert!(!declares("struct Imu_Raw {}", "struct", "Imu"));
        assert!(!declares("struct Imu2 {}", "struct", "Imu"));
        // A later, genuine declaration is still found after a near-miss.
        assert!(declares(
            "struct ImuSensor {}\nstruct Imu {}",
            "struct",
            "Imu"
        ));
    }

    #[test]
    fn missing_dependency_detection() {
        // The confirmed blocker: concepts/what-is-horus.mdx and tutorials/02
        // both open with this, but no generated project depends on it.
        assert_eq!(
            missing_dependencies("use horus_robotics::prelude::*;\nstruct A;"),
            vec!["horus_robotics"]
        );
        // Available in a generated project.
        assert!(missing_dependencies("use horus::prelude::*;").is_empty());
        assert!(missing_dependencies("use serde::{Serialize, Deserialize};").is_empty());
        assert!(missing_dependencies("use std::time::Duration;").is_empty());
    }

    #[test]
    fn serde_derives_are_not_injected() {
        // horus::prelude does not re-export them; injecting the import would
        // hide tutorial 01's first block failing to compile.
        assert!(!implied_prelude("#[derive(Serialize)] struct A;").contains("use serde"));
    }

    /// A block long enough to clear the `Trivial` threshold.
    const REAL: &str =
        "use horus::prelude::*;\nstruct Sensor { t: u32 }\nimpl Sensor { fn n() {} }";

    #[test]
    fn extracts_rust_fence() {
        let md = format!("text\n```rust\n{REAL}\n```\n");
        let got = rust_blocks(&md, "p.mdx");
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].0.line, 2);
        assert!(got[0].0.code.contains("struct Sensor"));
        assert_eq!(got[0].1, None);
    }

    #[test]
    fn honours_rustdoc_ignore() {
        let md = format!("```rust,ignore\n{REAL}\n```\n");
        assert_eq!(rust_blocks(&md, "p.mdx")[0].1, Some(Skip::RustdocIgnore));
    }

    #[test]
    fn trivial_blocks_are_skipped() {
        let md = "```rust\nTopic<f32>\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, Some(Skip::Trivial));
    }

    #[test]
    fn ellipsis_wins_over_trivial() {
        // Ordering regression: a short outline must report why it is an
        // outline, not merely that it is short.
        let md = "```rust\nfn a() {\n  ...\n}\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, Some(Skip::Ellipsis));
    }

    #[test]
    fn skips_placeholders() {
        let md = "```rust\nlet k = \"YOUR_API_KEY\";\nlet j = 2;\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, Some(Skip::Placeholder));
    }

    #[test]
    fn does_not_honour_the_simplified_marker() {
        // The docs bulk-inserted `// simplified` to opt 869 blocks out of their
        // own verifier. A reader still copies the code, so we still compile it.
        let md = "```rust\n// simplified\nuse horus::prelude::*;\nfn go() {}\n```\n";
        assert_eq!(
            rust_blocks(md, "p.mdx")[0].1,
            None,
            "`// simplified` must not exempt a block from compiling"
        );
    }

    #[test]
    fn ignores_non_rust_fences() {
        let md = "```python\nprint(1)\nprint(2)\n```\n";
        assert!(rust_blocks(md, "p.mdx").is_empty());
    }

    #[test]
    fn loose_statements_are_fragments() {
        // "Add to `DisplayNode::tick()`" — references `temp` from a scope that
        // only exists in the prose. Compiling it standalone proves nothing.
        let md = "```rust\nif temp > 30.0 {\n    println!(\"warn\");\n}\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, Some(Skip::Fragment));
    }

    #[test]
    fn imports_alone_do_not_make_a_block_self_contained() {
        // Regression: an opening `use` line used to be read as "this is a
        // module", which wrapped loose statements at item level and produced
        // a bogus `expected item, found keyword let` failure.
        let md = "```rust\nuse std::fs::OpenOptions;\nlet mut f = OpenOptions::new();\nf.append(true);\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, Some(Skip::Fragment));
    }

    #[test]
    fn counter_examples_are_skipped() {
        // getting-started/common-mistakes.mdx:177 — shown under "The Problem:"
        // to demonstrate a trait-bound error. Requiring it to compile inverts
        // the point of the page.
        let md = "```rust\nstruct MyMessage { x: f32 }\n// Error: the trait bound `MyMessage: Clone` is not satisfied\nlet t: Topic<MyMessage> = Topic::new(\"d\")?;\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, Some(Skip::CounterExample));
    }

    #[test]
    fn bare_methods_are_skipped() {
        // getting-started/troubleshooting.mdx:924 — a `tick` body shown without
        // its impl; rustc rejects `&mut self` in a free function.
        let md = "```rust\nfn tick(&mut self) {\n    let d = self.sensor.read();\n    self.topic.send(d);\n}\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, Some(Skip::MethodFragment));
    }

    #[test]
    fn methods_inside_an_impl_are_kept() {
        let md = "```rust\nuse horus::prelude::*;\nstruct S;\nimpl S {\n    fn tick(&mut self) { let _ = 1; }\n}\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, None);
    }

    #[test]
    fn complete_programs_are_kept() {
        let md = "```rust\nuse horus::prelude::*;\nstruct S;\nimpl Node for S {\n  fn name(&self) -> &str { \"s\" }\n}\n```\n";
        assert_eq!(rust_blocks(md, "p.mdx")[0].1, None);
    }

    #[test]
    fn item_level_detection() {
        assert!(is_item_level("use horus::prelude::*;\nstruct A;"));
        assert!(is_item_level("// c\nimpl Node for A {}"));
        assert!(!is_item_level("let x = 1;\nlet y = 2;"));
    }

    #[test]
    fn balance_ignores_braces_inside_strings_and_comments() {
        assert!(delimiters_balance("fn a() { println!(\"{}\", 1); }"));
        assert!(delimiters_balance("// } stray\nfn a() {}"));
        assert!(delimiters_balance("/* { */ fn a() {}"));
        assert!(delimiters_balance("let c = '}';"));
        assert!(delimiters_balance("fn a<'x>(v: &'x str) {}"));
        assert!(delimiters_balance("let s = \"a\\\"}\";"));
    }

    /// The case that made this necessary: rustc reports only the first unclosed
    /// delimiter in a crate and then stops, so one truncated excerpt in a batch
    /// suppressed every other diagnostic in it.
    #[test]
    fn unbalanced_excerpts_are_rejected() {
        assert!(!delimiters_balance(
            "impl Node for A {\n    fn tick(&mut self) {"
        ));
        assert!(!delimiters_balance("let x = foo(1, 2;"));
        assert!(!delimiters_balance("}"));
    }

    #[test]
    fn wraps_statements_in_a_function() {
        let b = Block {
            doc_file: "p.mdx".into(),
            line: 1,
            code: "let x = 1;\nlet y = 2;".into(),
            ctx: Ctx::Standalone,
        };
        let w = wrap(0, &b);
        assert!(w.contains("fn _snippet()"), "{w}");
    }

    #[test]
    fn wraps_items_directly_in_a_module() {
        let b = Block {
            doc_file: "p.mdx".into(),
            line: 1,
            code: "use horus::prelude::*;\nstruct A;".into(),
            ctx: Ctx::Standalone,
        };
        let w = wrap(0, &b);
        assert!(!w.contains("fn _snippet()"), "{w}");
        assert!(w.contains("mod docblock_0"), "{w}");
    }
}
