//! Verify the Python examples printed in the documentation.
//!
//! # Why this exists
//!
//! The docs repo does compile-check Python blocks — but through
//! `.github/workflows/verify-docs.yml`, which clones `horus@main`. As with the
//! Rust and C++ suites here, that answers "have the docs drifted from released
//! horus?" and never "does this PR break the docs?".
//!
//! It also stops at *syntax*. `horus.Scheduler(blackbox=64)` is perfectly valid
//! Python; it fails at runtime with `TypeError: unexpected keyword argument`.
//! Catching that needs the module's actual shape, so this suite introspects the
//! installed `horus` package rather than merely parsing the snippet.
//!
//! # What it checks
//!
//! 1. **Syntax** — every Python block must parse (`compile()`), which needs no
//!    horus install and so always runs.
//! 2. **API shape** — for blocks that import horus: every `horus.X` attribute
//!    exists, every `from horus.Y import` submodule exists, and every keyword
//!    passed to `horus.Scheduler(...)` / `horus.Node(...)` is real.
//!
//! Nothing is *executed*: doc snippets open shared memory, spin schedulers and
//! talk to hardware. Attributes and signatures are read reflectively instead, so
//! the suite is safe to run anywhere.
//!
//! # Running it
//!
//! ```bash
//! HORUS_DOCS_DIR=../horus-docs cargo test -p horus_manager \
//!     --test docs_examples_python -- --ignored --nocapture
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;

// ─── Model ──────────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
struct Block {
    doc_file: String,
    line: usize,
    code: String,
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

fn python() -> Option<String> {
    for p in ["python3", "python"] {
        if Command::new(p)
            .arg("--version")
            .output()
            .is_ok_and(|o| o.status.success())
        {
            return Some(p.to_string());
        }
    }
    None
}

/// Whether the real `horus` extension module is importable.
///
/// Guarded because the repo root contains a `horus/` directory that Python
/// happily treats as a namespace package: importing from the wrong working
/// directory yields a module with no attributes at all, which would make every
/// API assertion below "fail" for a reason that has nothing to do with the docs.
fn horus_importable(py: &str) -> bool {
    Command::new(py)
        .args(["-c", "import horus; assert hasattr(horus, 'Scheduler')"])
        // Run somewhere without a shadowing `horus/` directory.
        .current_dir(std::env::temp_dir())
        .output()
        .is_ok_and(|o| o.status.success())
}

// ─── Extraction ─────────────────────────────────────────────────────────────

fn python_blocks(text: &str, rel: &str) -> Vec<Block> {
    let mut out = Vec::new();
    let mut cur: Option<(usize, String)> = None;

    for (idx, raw) in text.lines().enumerate() {
        if let Some(rest) = raw.trim_start().strip_prefix("```") {
            match cur.take() {
                Some((start, code)) => out.push(Block {
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
                    if matches!(lang.as_str(), "python" | "py") {
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
    out.iter_mut().for_each(|b| b.code = dedent(&b.code));
    out
}

/// Strip the indentation shared by every non-blank line.
///
/// MDX nests fenced blocks inside JSX (`<LangTab language="Python">`), which
/// indents the code by two or four spaces. Python is whitespace-significant, so
/// those blocks raise `IndentationError: unexpected indent` on the very first
/// line — a property of the page's markup, not of the example. 182 of the docs'
/// Python blocks are affected; without this they would all read as broken.
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

/// A block is an outline/placeholder rather than something a reader runs.
fn is_illustrative(code: &str) -> bool {
    let t = code.trim();
    if t.len() < 20 || t.contains("...") {
        return true;
    }
    let lower = t.to_ascii_lowercase();
    if ["<your_", "your_", "placeholder", "/path/to"]
        .iter()
        .any(|p| lower.contains(p))
    {
        return true;
    }
    if is_signature_notation(t) {
        return true;
    }
    // A method body shown without its class: `return` / `yield` / `self` are
    // only legal inside the enclosing `def` the prose describes. Checked
    // anywhere in the block, not just the first line — `python/messages/
    // geometry.mdx:61` shows `if v.magnitude() < 0.01:\n    return`, where the
    // giveaway is on line 2.
    let defines_fn = t
        .lines()
        .map(str::trim_start)
        .any(|l| l.starts_with("def ") || l.starts_with("async def ") || l.starts_with("class "));
    if !defines_fn
        && t.lines()
            .map(str::trim_start)
            .any(|l| l == "return" || l.starts_with("return ") || l.starts_with("yield "))
    {
        return true;
    }

    let first = first_code_line(t);
    first.starts_with("self.")
        || first.starts_with("elif ")
        || first.starts_with("else:")
        || first.starts_with("except")
}

/// A block whose content is shell, mis-tagged as ```python.
///
/// `python/api/python-bindings.mdx:2127` and `:2137` do this with
/// `cd horus_py` / `maturin develop --release`. A reader pasting them into a
/// `.py` file gets a SyntaxError, so the fence tag is the defect.
fn looks_like_shell(code: &str) -> bool {
    let lines: Vec<&str> = code
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect();
    if lines.is_empty() {
        return false;
    }
    lines.iter().all(|l| {
        let first = l.split_whitespace().next().unwrap_or("");
        matches!(
            first,
            "cd" | "ls"
                | "pip"
                | "pip3"
                | "python"
                | "python3"
                | "maturin"
                | "cargo"
                | "horus"
                | "export"
                | "sudo"
                | "mkdir"
                | "rm"
                | "cp"
                | "mv"
                | "source"
                | "apt"
                | "apt-get"
                | "brew"
        )
    })
}

fn first_code_line(code: &str) -> &str {
    code.lines()
        .map(str::trim_start)
        .find(|l| !l.is_empty() && !l.starts_with('#'))
        .unwrap_or("")
}

/// A block that documents a *signature* rather than showing runnable code.
///
/// The Python API pages use this idiom throughout, under headings like
/// `### send()`:
///
/// ```text
/// node.send(topic: str, data: Any) -> bool
/// ```
///
/// and bodyless `def Node(name: str = "", …)` parameter tables. Both are
/// deliberately not valid Python — annotating the arguments of a *call* is a
/// syntax error, and a `def` with no body is incomplete. Reporting them as
/// broken examples would be reporting the docs' reference style.
fn is_signature_notation(code: &str) -> bool {
    let first = first_code_line(code);

    // A bare `*,` marks keyword-only parameters — legal in a `def`, a syntax
    // error in a call. `python/api/scheduler.mdx:24` documents the constructor
    // this way: `horus.Scheduler(\n    *,   # keyword-only\n    tick_rate=…`.
    if code
        .lines()
        .map(str::trim_start)
        .any(|l| l == "*," || l.starts_with("*,") || l.starts_with("*, "))
    {
        return true;
    }

    // `name(args) -> ret` with no assignment: a signature, not a call.
    if first.contains("->") && first.contains('(') && !first.contains('=') {
        return true;
    }
    // A call whose arguments carry type annotations is not valid Python.
    if let Some(open) = first.find('(') {
        if !first.starts_with("def ") && !first.starts_with("class ") {
            let args = &first[open..];
            if args.contains(": str")
                || args.contains(": int")
                || args.contains(": float")
                || args.contains(": bool")
                || args.contains(": Any")
                || args.contains(": Callable")
            {
                return true;
            }
        }
    }
    // `def f(` … that never closes into a body.
    if first.starts_with("def ") || first.starts_with("class ") {
        let has_body = code
            .lines()
            .skip(1)
            .any(|l| l.starts_with("    ") && !l.trim_start().starts_with('#'));
        let closes = code.lines().any(|l| l.trim_end().ends_with(':'));
        if !has_body || !closes {
            return true;
        }
    }
    false
}

fn collect(docs: &Path, filter: Option<&str>) -> Vec<Block> {
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

    let mut out = Vec::new();
    for f in files {
        let rel = f
            .strip_prefix(docs)
            .unwrap_or(&f)
            .to_string_lossy()
            .replace('\\', "/");
        if filter.is_some_and(|n| !rel.contains(n)) {
            continue;
        }
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        out.extend(
            python_blocks(&text, &rel)
                .into_iter()
                .filter(|b| !is_illustrative(&b.code)),
        );
    }
    out
}

// ─── API-reference extraction ───────────────────────────────────────────────

/// One thing a snippet asserts about the `horus` module.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Claim {
    /// `horus.Foo` — attribute must exist.
    Attr(String),
    /// `from horus.foo import ...` — submodule must exist.
    Submodule(String),
    /// `horus.Scheduler(foo=...)` — keyword must be accepted.
    Kwarg { callable: String, kwarg: String },
}

/// Pull API claims out of one snippet.
///
/// Scans over `char`s throughout. An earlier revision mixed byte offsets from
/// `str::find` with indices into a `Vec<char>`; the two diverge as soon as a
/// page contains a non-ASCII character (the docs are full of `—` and `μs`), and
/// it panicked with an out-of-range slice on the very first real run.
fn claims_in(code: &str) -> BTreeSet<Claim> {
    let mut out = BTreeSet::new();
    let stripped = strip_comments_and_strings(code);
    let chars: Vec<char> = stripped.chars().collect();
    let needle: Vec<char> = "horus.".chars().collect();

    let mut i = 0usize;
    while i + needle.len() <= chars.len() {
        if chars[i..i + needle.len()] != needle[..] {
            i += 1;
            continue;
        }
        let start = i + needle.len();
        let mut j = start;
        while j < chars.len() && (chars[j].is_alphanumeric() || chars[j] == '_') {
            j += 1;
        }
        let name: String = chars[start..j].iter().collect();
        if !name.is_empty() {
            // `horus.messages.X` reads as a submodule; `horus.Scheduler` as an attr.
            if j < chars.len() && chars[j] == '.' {
                out.insert(Claim::Submodule(name.clone()));
            } else {
                out.insert(Claim::Attr(name.clone()));
            }

            // Keyword arguments, when this is a call.
            if j < chars.len() && chars[j] == '(' {
                if let Some(args) = balanced_args(&chars, j) {
                    for kw in keyword_names(&args) {
                        out.insert(Claim::Kwarg {
                            callable: name.clone(),
                            kwarg: kw,
                        });
                    }
                }
            }
        }
        i = start.max(i + 1);
    }

    // `from horus.<sub> import ...`
    for line in stripped.lines().map(str::trim_start) {
        if let Some(rest) = line.strip_prefix("from horus.") {
            if let Some(sub) = rest.split_whitespace().next() {
                let sub = sub.trim_end_matches(|c: char| !c.is_alphanumeric() && c != '_');
                if !sub.is_empty() {
                    out.insert(Claim::Submodule(sub.to_string()));
                }
            }
        }
    }

    out
}

/// Blank out `#` comments and string literals, keeping line structure.
///
/// Without this, prose inside code produces phantom API claims: `horus.toml` is
/// the manifest *filename* and appears in comments like
/// `# From horus.toml [hardware]`, which read as an attribute access on the
/// module. Three of the first run's 27 "missing APIs" were exactly this.
fn strip_comments_and_strings(code: &str) -> String {
    let mut out = String::with_capacity(code.len());
    for line in code.lines() {
        let mut quote: Option<char> = None;
        let mut prev_backslash = false;
        for c in line.chars() {
            match quote {
                Some(q) => {
                    // Inside a literal: keep length, drop content.
                    out.push(' ');
                    if c == q && !prev_backslash {
                        quote = None;
                    }
                    prev_backslash = c == '\\' && !prev_backslash;
                }
                None => {
                    if c == '#' {
                        break; // rest of the line is a comment
                    }
                    if c == '\'' || c == '"' {
                        quote = Some(c);
                        out.push(' ');
                    } else {
                        out.push(c);
                    }
                    prev_backslash = false;
                }
            }
        }
        out.push('\n');
    }
    out
}

/// Text between the parenthesis at `open_idx` and its match.
fn balanced_args(chars: &[char], open_idx: usize) -> Option<String> {
    let mut depth = 0i32;
    let mut buf = String::new();
    for (k, &c) in chars.iter().enumerate().skip(open_idx) {
        match c {
            '(' | '[' | '{' => depth += 1,
            ')' | ']' | '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(buf);
                }
            }
            _ => {}
        }
        if k > open_idx {
            buf.push(c);
        }
        if buf.len() > 4000 {
            return None;
        }
    }
    None
}

/// Top-level `name=` keywords in an argument list.
fn keyword_names(args: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut depth = 0i32;
    let mut token = String::new();
    let mut chars = args.chars().peekable();
    let mut segment = String::new();

    while let Some(c) = chars.next() {
        match c {
            '(' | '[' | '{' => {
                depth += 1;
                segment.push(c);
            }
            ')' | ']' | '}' => {
                depth -= 1;
                segment.push(c);
            }
            ',' if depth == 0 => {
                take_kwarg(&segment, &mut out);
                segment.clear();
            }
            _ => segment.push(c),
        }
        token.clear();
        let _ = chars.peek();
    }
    take_kwarg(&segment, &mut out);
    out
}

fn take_kwarg(segment: &str, out: &mut Vec<String>) {
    let s = segment.trim();
    // `name=value`, but not `==`, `>=`, `!=`
    if let Some(eq) = s.find('=') {
        if s.as_bytes().get(eq + 1) == Some(&b'=') {
            return;
        }
        if eq > 0 && matches!(s.as_bytes()[eq - 1], b'!' | b'<' | b'>' | b'=') {
            return;
        }
        let name = s[..eq].trim();
        if !name.is_empty()
            && name.chars().all(|c| c.is_alphanumeric() || c == '_')
            && name
                .chars()
                .next()
                .is_some_and(|c| c.is_alphabetic() || c == '_')
        {
            out.push(name.to_string());
        }
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

/// Every documented Python snippet must at least parse.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn documented_python_examples_parse() {
    let Some(py) = python() else {
        eprintln!("skipping: no python interpreter");
        return;
    };
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let filter = std::env::var("HORUS_DOCS_FILTER").ok();
    let blocks = collect(&docs, filter.as_deref());
    assert!(
        !blocks.is_empty(),
        "no Python blocks found — the extractor is broken, making this vacuous"
    );
    eprintln!("python blocks: {}", blocks.len());

    let tmp = tempfile::tempdir().expect("temp dir");
    let mut failures = Vec::new();
    for (i, b) in blocks.iter().enumerate() {
        let f = tmp.path().join(format!("b{i}.py"));
        std::fs::write(&f, &b.code).unwrap();
        let out = Command::new(&py)
            .arg("-c")
            .arg("import sys; compile(open(sys.argv[1]).read(), sys.argv[1], 'exec')")
            .arg(&f)
            .output()
            .expect("python runs");
        if !out.status.success() {
            let err = String::from_utf8_lossy(&out.stderr);
            let last = err.lines().last().unwrap_or("").to_string();
            let hint = if looks_like_shell(&b.code) {
                "\n      (this looks like shell, not Python — the fence is tagged ```python)"
            } else {
                ""
            };
            failures.push(format!("  {}:{}\n      {last}{hint}", b.doc_file, b.line));
        }
    }

    assert!(
        failures.is_empty(),
        "{} documented Python example(s) are not valid Python:\n{}",
        failures.len(),
        failures.join("\n")
    );
    eprintln!("all {} documented Python examples parse", blocks.len());
}

/// Every `horus.X` the docs reference must exist in the installed module.
#[test]
#[ignore = "needs a horus-docs checkout and an installed horus wheel"]
fn documented_python_api_exists() {
    let Some(py) = python() else {
        eprintln!("skipping: no python interpreter");
        return;
    };
    if !horus_importable(&py) {
        eprintln!(
            "skipping: the `horus` module is not importable. Build and install it:\n  \
             pip install maturin && (cd horus_py && maturin build --release) && \
             pip install horus_py/target/wheels/*.whl"
        );
        return;
    }
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let filter = std::env::var("HORUS_DOCS_FILTER").ok();
    let blocks = collect(&docs, filter.as_deref());

    // Claim -> where the docs make it.
    let mut sites: BTreeMap<Claim, Vec<String>> = BTreeMap::new();
    for b in &blocks {
        if !b.code.contains("horus") {
            continue;
        }
        for c in claims_in(&b.code) {
            sites
                .entry(c)
                .or_default()
                .push(format!("{}:{}", b.doc_file, b.line));
        }
    }
    assert!(
        !sites.is_empty(),
        "extracted no API claims — the extractor is broken, making this vacuous"
    );
    eprintln!("distinct horus API claims in the docs: {}", sites.len());

    // One probe process, fed every claim as JSON.
    let probe = r#"
import json, sys, inspect
import horus

results = []
for c in json.load(sys.stdin):
    kind = c["kind"]
    if kind == "attr":
        ok = hasattr(horus, c["name"])
        detail = "" if ok else "horus.%s does not exist" % c["name"]
    elif kind == "submodule":
        try:
            __import__("horus." + c["name"])
            ok, detail = True, ""
        except Exception:
            ok = hasattr(horus, c["name"])
            detail = "" if ok else "no module or attribute horus.%s" % c["name"]
    else:  # kwarg
        target = getattr(horus, c["callable"], None)
        if target is None:
            ok, detail = True, ""      # the attr test owns this failure
        else:
            try:
                sig = inspect.signature(target)
            except (TypeError, ValueError):
                ok, detail = True, ""  # not introspectable; do not guess
            else:
                params = sig.parameters
                has_var_kw = any(p.kind == p.VAR_KEYWORD for p in params.values())
                ok = has_var_kw or c["kwarg"] in params
                detail = "" if ok else "%s() has no keyword '%s' (accepts: %s)" % (
                    c["callable"], c["kwarg"],
                    ", ".join(k for k in params if k != "self"))
    results.append({"ok": ok, "detail": detail})
json.dump(results, sys.stdout)
"#;

    let claims: Vec<&Claim> = sites.keys().collect();
    let payload: Vec<serde_json::Value> = claims
        .iter()
        .map(|c| match c {
            Claim::Attr(n) => serde_json::json!({"kind":"attr","name":n}),
            Claim::Submodule(n) => serde_json::json!({"kind":"submodule","name":n}),
            Claim::Kwarg { callable, kwarg } => {
                serde_json::json!({"kind":"kwarg","callable":callable,"kwarg":kwarg})
            }
        })
        .collect();

    let tmp = tempfile::tempdir().expect("temp dir");
    let script = tmp.path().join("probe.py");
    std::fs::write(&script, probe).unwrap();
    let input = tmp.path().join("claims.json");
    std::fs::write(&input, serde_json::to_string(&payload).unwrap()).unwrap();

    let out = Command::new("sh")
        .arg("-c")
        .arg(format!("{py} {} < {}", script.display(), input.display()))
        // Anywhere but the repo root, so the `horus/` source directory does not
        // shadow the installed package.
        .current_dir(std::env::temp_dir())
        .output()
        .expect("probe runs");
    assert!(
        out.status.success(),
        "probe failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let verdicts: Vec<serde_json::Value> =
        serde_json::from_slice(&out.stdout).expect("probe emits JSON");
    assert_eq!(verdicts.len(), claims.len());

    let mut broken = Vec::new();
    for (c, v) in claims.iter().zip(&verdicts) {
        if v["ok"].as_bool() == Some(false) {
            let where_ = sites[*c]
                .iter()
                .take(2)
                .cloned()
                .collect::<Vec<_>>()
                .join(", ");
            broken.push(format!(
                "  {}\n      documented at {where_}",
                v["detail"].as_str().unwrap_or("?")
            ));
        }
    }

    assert!(
        broken.is_empty(),
        "{} documented Python API reference(s) do not exist:\n{}\n\n\
         These are valid Python that raises AttributeError / TypeError the moment \
         a reader runs it — syntax checking cannot catch them. Fix the doc page or \
         restore the API.",
        broken.len(),
        broken.join("\n")
    );
}

// ─── Extractor unit tests ───────────────────────────────────────────────────

#[cfg(test)]
mod extractor {
    use super::*;

    #[test]
    fn extracts_python_fence() {
        let md = "t\n```python\nimport horus\nsched = horus.Scheduler()\n```\n";
        let got = python_blocks(md, "p.mdx");
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].line, 2);
    }

    #[test]
    fn ignores_other_languages() {
        assert!(python_blocks("```rust\nfn main() {}\n```\n", "p.mdx").is_empty());
    }

    #[test]
    fn finds_attribute_claims() {
        let c = claims_in("t = horus.Topic(horus.CmdVel)");
        assert!(c.contains(&Claim::Attr("Topic".into())), "{c:?}");
        assert!(c.contains(&Claim::Attr("CmdVel".into())), "{c:?}");
    }

    #[test]
    fn finds_keyword_claims() {
        // The confirmed finding: `blackbox=` does not exist (it is `blackbox_mb`).
        let c = claims_in("horus.Scheduler(tick_rate=1000, blackbox=64)");
        assert!(
            c.contains(&Claim::Kwarg {
                callable: "Scheduler".into(),
                kwarg: "blackbox".into()
            }),
            "{c:?}"
        );
        assert!(
            c.contains(&Claim::Kwarg {
                callable: "Scheduler".into(),
                kwarg: "tick_rate".into()
            }),
            "{c:?}"
        );
    }

    #[test]
    fn finds_submodule_claims() {
        // Confirmed finding: there is no `horus.messages` submodule.
        let c = claims_in("from horus.messages import CmdVel");
        assert!(c.contains(&Claim::Submodule("messages".into())), "{c:?}");
    }

    #[test]
    fn comparison_operators_are_not_keywords() {
        let c = claims_in("horus.wait(timeout == 5)");
        assert!(
            !c.iter().any(|x| matches!(x, Claim::Kwarg { .. })),
            "`==` must not be read as a keyword argument: {c:?}"
        );
    }

    #[test]
    fn nested_calls_do_not_leak_keywords() {
        // `mode=` belongs to inner(), not to Scheduler().
        let kws: Vec<String> = keyword_names("tick_rate=100, node=make(mode=2)");
        assert!(kws.contains(&"tick_rate".to_string()), "{kws:?}");
        assert!(kws.contains(&"node".to_string()), "{kws:?}");
        assert!(!kws.contains(&"mode".to_string()), "{kws:?}");
    }

    #[test]
    fn dedents_mdx_indented_blocks() {
        // MDX indents fences nested inside <LangTab>; Python cares.
        let md = "```python\n    import horus\n    s = horus.Scheduler()\n```\n";
        let got = python_blocks(md, "p.mdx");
        assert!(
            got[0].code.starts_with("import horus"),
            "not dedented: {:?}",
            got[0].code
        );
    }

    #[test]
    fn signature_notation_is_skipped() {
        // python/api/node.mdx:156 — a signature, not a call. Annotating the
        // arguments of a call is a syntax error by construction.
        assert!(is_signature_notation(
            "node.send(topic: str, data: Any) -> bool"
        ));
        // python/api/python-bindings.mdx:83 — a bodyless parameter table.
        assert!(is_signature_notation(
            "def Node(\n    name: str = \"\",\n    rate: int = 30,"
        ));
        // A real call must not be mistaken for one.
        assert!(!is_signature_notation(
            "sched = horus.Scheduler(tick_rate=100)\nsched.run()"
        ));
        // A real def with a body must not either.
        assert!(!is_signature_notation(
            "def tick(node):\n    node.send(1)\n"
        ));
    }

    #[test]
    fn keyword_only_marker_is_signature_notation() {
        // python/api/scheduler.mdx:24 documents the constructor with a bare `*`.
        let code = "horus.Scheduler(\n    *,          # keyword-only\n    tick_rate=1000.0,\n)";
        assert!(is_signature_notation(code));
    }

    #[test]
    fn top_level_return_marks_a_body_excerpt() {
        // python/messages/geometry.mdx:61 — the `return` is on line 2.
        let code = "if force_vec.magnitude() < 0.01:\n    return  # ignore\n";
        assert!(is_illustrative(code));
        // A `return` inside a real def must not trip it.
        assert!(!is_illustrative(
            "def tick(node):\n    if node.x:\n        return\n    node.send(1)\n"
        ));
    }

    #[test]
    fn comments_and_strings_do_not_produce_claims() {
        // `horus.toml` is the manifest filename, not an attribute.
        let c = claims_in("entries = horus.drivers.load()   # From horus.toml [hardware]\n");
        assert!(
            !c.contains(&Claim::Attr("toml".into()))
                && !c.contains(&Claim::Submodule("toml".into())),
            "comment text leaked into claims: {c:?}"
        );
        assert!(c.contains(&Claim::Submodule("drivers".into())), "{c:?}");

        let s = claims_in("path = \"horus.fake_thing\"\nx = horus.Topic\n");
        assert!(!s.contains(&Claim::Attr("fake_thing".into())), "{s:?}");
        assert!(s.contains(&Claim::Attr("Topic".into())), "{s:?}");
    }

    #[test]
    fn claims_survive_non_ascii() {
        // Regression: byte offsets from `find` were indexed into a Vec<char>,
        // which panicked on the first page containing an em dash.
        let c = claims_in("# budget — 800μs\nsched = horus.Scheduler(tick_rate=100)\n");
        assert!(c.contains(&Claim::Attr("Scheduler".into())), "{c:?}");
    }

    #[test]
    fn illustrative_blocks_are_dropped() {
        assert!(is_illustrative("sched = horus.Scheduler(...)"));
        assert!(is_illustrative("x = 1"));
        assert!(!is_illustrative(
            "import horus\nsched = horus.Scheduler()\n"
        ));
    }
}
