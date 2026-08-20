//! Compile the C++ examples printed in the documentation.
//!
//! # Why this exists
//!
//! C++ is the largest unverified surface in the documentation. The docs repo's
//! extractor declares `VERIFIABLE_LANGUAGES = ['rust', 'python']`, so **not one**
//! of its 260 C++ blocks is compiled by anything, in either repository. Every
//! C++ page is published on trust.
//!
//! Checking them needs no cmake and no linking: `g++ -fsyntax-only` against
//! `horus_cpp/include` type-checks a translation unit outright, which is where
//! the interesting breakage lives (missing includes, renamed fields, wrong
//! smart-pointer hops). That makes this cheap enough to run per-PR, unlike the
//! Rust sweep.
//!
//! # What it deliberately does not do
//!
//! It does not inject standard-library includes. A confirmed finding is that
//! `06-services-actions-cpp.mdx` and `08-multi-process-cpp.mdx` assume
//! `<horus/horus.hpp>` transitively drags in `<thread>`, `<atomic>` and
//! `<cmath>`; it does not, and both pages fail to compile exactly as printed.
//! Injecting those headers would hide the defect rather than surface it.
//!
//! # Running it
//!
//! ```bash
//! HORUS_DOCS_DIR=../horus-docs cargo test -p horus_manager \
//!     --test docs_examples_cpp -- --ignored --nocapture
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

// ─── Model ──────────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
struct Block {
    doc_file: String,
    line: usize,
    code: String,
    /// True when the block had no `#include` of its own and `<horus/horus.hpp>`
    /// was supplied for it.
    ///
    /// A body excerpt is not a translation unit, which is why these used to be
    /// skipped — but the prose around them always implies the same preamble,
    /// and the harness already knows how to wrap a bare body in a function.
    /// Supplying the umbrella header puts the horus calls inside them in front
    /// of the compiler instead of taking them on trust; anything the reader's
    /// own file would have declared still fails to resolve, and lands in the
    /// same advisory bucket as before.
    supplied_preamble: bool,
}

/// Why a block is not compiled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Skip {
    /// Contains `...`, so it is an outline rather than a program.
    Ellipsis,
    /// `<your_project>` / `YOUR_KEY` — the reader must substitute.
    Placeholder,
    /// Too small to carry meaning.
    Trivial,
    /// No `#include` and no definition: a body excerpt that only makes sense
    /// inside the surrounding class the prose describes.
    Fragment,
    /// Shown to demonstrate a mistake; requiring it to compile inverts it.
    CounterExample,
    /// Another framework's code, printed for comparison (`learn/coming-from-ros2`
    /// shows the ROS2 version beside the HORUS one). Requiring `rclcpp` to be
    /// installed would be nonsense.
    ForeignFramework,
    /// Quotes a header the reader supplies, not one this repo ships — e.g.
    /// `#include "vendor_sdk/lidar.hpp"` in the passage about linking an
    /// existing vendor SDK straight into `tick()`. The header's absence IS the
    /// point of the example; inventing one to satisfy the compiler would
    /// document a vendor SDK that does not exist.
    ReaderSuppliedHeader,
}

/// Headers that belong to a framework the docs are comparing against, not to
/// anything a HORUS user is expected to have.
const FOREIGN_HEADERS: &[&str] = &[
    "rclcpp/",
    "rclpy/",
    "ros/",
    "ros2/",
    "sensor_msgs/",
    "geometry_msgs/",
    "std_msgs/",
    "nav_msgs/",
    "tf2_ros/",
];

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

fn include_dir() -> PathBuf {
    repo_root().join("horus_cpp/include")
}

/// The C++ compiler to drive, or `None` when none is installed.
fn cxx() -> Option<String> {
    if let Ok(c) = std::env::var("CXX") {
        return Some(c);
    }
    for c in ["g++", "clang++", "c++"] {
        if Command::new(c)
            .arg("--version")
            .output()
            .is_ok_and(|o| o.status.success())
        {
            return Some(c.to_string());
        }
    }
    None
}

// ─── Extraction ─────────────────────────────────────────────────────────────

fn cpp_blocks(text: &str, rel: &str) -> Vec<(Block, Option<Skip>)> {
    let mut out = Vec::new();
    let mut cur: Option<(usize, String)> = None;

    for (idx, raw) in text.lines().enumerate() {
        if let Some(rest) = raw.trim_start().strip_prefix("```") {
            match cur.take() {
                Some((start, code)) => {
                    let skip = classify(&code);
                    out.push((
                        Block {
                            doc_file: rel.to_string(),
                            line: start,
                            code,
                            supplied_preamble: false,
                        },
                        skip,
                    ));
                }
                None => {
                    let lang = rest
                        .trim()
                        .split([':', ' ', ','])
                        .next()
                        .unwrap_or("")
                        .to_ascii_lowercase();
                    if matches!(lang.as_str(), "cpp" | "c++" | "cc") {
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
    out
}

fn classify(code: &str) -> Option<Skip> {
    let t = code.trim();
    if t.contains("...") {
        return Some(Skip::Ellipsis);
    }
    let lower = t.to_ascii_lowercase();
    for p in ["<your_", "your_", "placeholder", "/path/to"] {
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
    if FOREIGN_HEADERS.iter().any(|h| t.contains(h)) {
        return Some(Skip::ForeignFramework);
    }
    if quotes_unshipped_header(t) {
        return Some(Skip::ReaderSuppliedHeader);
    }
    // A translation unit needs at least one include; without one the block is an
    // excerpt of a body, not something a reader could compile.
    if !t.contains("#include") {
        return Some(Skip::Fragment);
    }
    None
}

/// Whether the block quotes a header this repository does not ship.
///
/// `#include <horus/...>` is ours and must resolve. A *quoted* include that is
/// neither a horus header nor a real file under `horus_cpp/include/` is the
/// reader's own code — the vendor-SDK passages exist precisely to show that
/// such a header drops straight in. Resolving it is impossible by design, so
/// the block is illustrative rather than broken.
fn quotes_unshipped_header(code: &str) -> bool {
    let include_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .map(|r| r.join("horus_cpp/include"));
    code.lines()
        .map(str::trim_start)
        .filter_map(|l| l.strip_prefix("#include"))
        .filter_map(|rest| {
            let rest = rest.trim_start();
            rest.strip_prefix('"')
                .and_then(|r| r.split('"').next())
                .map(str::to_string)
        })
        .any(|path| match &include_root {
            Some(root) => !root.join(&path).is_file(),
            None => !path.starts_with("horus/"),
        })
}

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
                || c.starts_with("does not compile")
                || c.starts_with("will not compile")
        })
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
        if filter.is_some_and(|n| !rel.contains(n)) {
            continue;
        }
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        for (mut b, skip) in cpp_blocks(&text, &rel) {
            match skip {
                Some(s @ Skip::Fragment) => {
                    *skipped.entry(s).or_default() += 1;
                    b.supplied_preamble = true;
                    keep.push(b);
                }
                Some(s) => *skipped.entry(s).or_default() += 1,
                None => keep.push(b),
            }
        }
    }
    (keep, skipped)
}

// ─── Compiling ──────────────────────────────────────────────────────────────

/// Split a snippet into its preamble (`#include` / `using` / `#define`) and the
/// rest, so the remainder can be moved inside a function when needed.
fn split_preamble(code: &str) -> (String, String) {
    let mut pre = String::new();
    let mut body = String::new();
    for line in code.lines() {
        let t = line.trim_start();
        if t.starts_with("#include")
            || t.starts_with("#define")
            || t.starts_with("#pragma")
            || t.starts_with("using ")
            || t.starts_with("namespace ")
        {
            pre.push_str(line);
            pre.push('\n');
        } else {
            body.push_str(line);
            body.push('\n');
        }
    }
    (pre, body)
}

/// Diagnostics that mean "these are statements, and they are at file scope" —
/// the snippet is an API-usage excerpt the reader would paste into a function.
fn is_file_scope_complaint(stderr: &str) -> bool {
    stderr.contains("expected unqualified-id")
        || stderr.contains("does not name a type")
        || stderr.contains("expected constructor, destructor, or type conversion")
}

fn run_cxx(cxx: &str, src: &Path) -> Option<(bool, String)> {
    let out = Command::new(cxx)
        .args(["-fsyntax-only", "-std=c++17", "-w"])
        .arg("-I")
        .arg(include_dir())
        .arg(src)
        .output()
        .ok()?;
    Some((
        out.status.success(),
        String::from_utf8_lossy(&out.stderr).to_string(),
    ))
}

/// Type-check one block. Returns the compiler's diagnostics on failure.
///
/// Compiled verbatim first, so a block that *is* a whole program is judged
/// exactly as a reader would copy it. Only if that fails specifically because
/// statements sit at file scope is the body retried inside a function — docs
/// routinely show "here is how you call it" without wrapping it in `main`, and
/// reporting that as broken would be reporting their prose style.
fn syntax_check(cxx: &str, dir: &Path, idx: usize, b: &Block) -> Option<String> {
    let src = dir.join(format!("block{idx}.cpp"));
    // A block with no include of its own gets the umbrella header the
    // surrounding prose assumes. `<horus/horus.hpp>` is exactly what the C++
    // guide tells readers to write, so this is the reader's own preamble, not a
    // convenience the harness invented.
    let code = if b.supplied_preamble {
        format!("#include <horus/horus.hpp>\nusing namespace horus::literals;\n{}", b.code)
    } else {
        b.code.clone()
    };
    std::fs::write(&src, &code).ok()?;
    let (ok, stderr) = run_cxx(cxx, &src)?;
    if ok {
        return None;
    }
    if !is_file_scope_complaint(&stderr) {
        return Some(stderr);
    }

    let (pre, body) = split_preamble(&code);
    // A method printed without its class — `void tick() override { … }`, or a
    // `const` member — is not a free function and will not compile as one.
    // `override` and a trailing `const` are both illegal outside a class, so
    // give it a class to sit in rather than reporting the page for showing a
    // method the way reference documentation always shows methods.
    let looks_like_method = body.contains(" override")
        || body.contains(") const {")
        || body.contains(") const\n");
    if looks_like_method {
        let wrapped_m = dir.join(format!("block{idx}_member.cpp"));
        std::fs::write(
            &wrapped_m,
            format!("{pre}\nstruct _DocSelf {{\n{body}\n}};\n"),
        )
        .ok()?;
        if let Some((ok, err)) = run_cxx(cxx, &wrapped_m) {
            if ok {
                return None;
            }
            return Some(err);
        }
    }
    let wrapped = dir.join(format!("block{idx}_wrapped.cpp"));
    std::fs::write(
        &wrapped,
        format!("{pre}\nvoid _horus_docs_snippet() {{\n{body}\n}}\n"),
    )
    .ok()?;
    let (ok2, stderr2) = run_cxx(cxx, &wrapped)?;
    if ok2 {
        None
    } else {
        Some(stderr2)
    }
}

/// First few `error:` lines, trimmed to the compiler's own message.
fn first_errors(stderr: &str, n: usize) -> Vec<String> {
    stderr
        .lines()
        .filter(|l| l.contains("error:"))
        .take(n)
        .map(|l| {
            // strip the leading temp path so output is stable across runs
            match l.find("error:") {
                Some(i) => {
                    let file = l[..i].rsplit('/').next().unwrap_or("");
                    format!("{file}{}", &l[i..])
                }
                None => l.to_string(),
            }
        })
        .collect()
}

/// The `horus_cpp` static library, if it has been built.
///
/// The C++ API is header-only over a C ABI: `include/horus/impl/*.hpp` wrap the
/// 129 `extern "C"` entry points in `horus_cpp/src/c_api.rs`, which land in this
/// archive. Linking against it needs no cmake — that is only required for the
/// *project* build path (`horus build` on a C++ project), which is a separate,
/// separately-documented prerequisite.
fn horus_cpp_staticlib() -> Option<PathBuf> {
    for profile in ["debug", "release"] {
        let p = repo_root()
            .join("target")
            .join(profile)
            .join("libhorus_cpp.a");
        if p.is_file() {
            return Some(p);
        }
    }
    None
}

/// Link a block into a real executable. Returns linker diagnostics on failure.
fn link_check(cxx: &str, lib: &Path, dir: &Path, idx: usize, b: &Block) -> Option<String> {
    let src = dir.join(format!("link{idx}.cpp"));
    // Same preamble `syntax_check` supplies. A body excerpt that happens to
    // contain `int main` is still an excerpt: handing it to the linker without
    // the umbrella header reports a missing include as a link failure, which is
    // both wrong and unactionable.
    let code = if b.supplied_preamble {
        format!("#include <horus/horus.hpp>\nusing namespace horus::literals;\n{}", b.code)
    } else {
        b.code.clone()
    };
    std::fs::write(&src, &code).ok()?;
    let exe = dir.join(format!("link{idx}"));

    let out = Command::new(cxx)
        .args(["-std=c++17", "-w"])
        .arg("-I")
        .arg(include_dir())
        .arg(&src)
        .arg("-o")
        .arg(&exe)
        .arg(lib)
        .args(["-lpthread", "-ldl", "-lm"])
        .output()
        .ok()?;

    // A linked HORUS binary is ~100 MB. Keeping all of them alive needs ~3 GB of
    // temp space, and on a tmpfs that runs out partway through — reporting
    // programs that link perfectly well as link failures, with a bare
    // "ld returned 1 exit status" and nothing to act on. Drop each one as soon
    // as its exit status has been read.
    let _ = std::fs::remove_file(&exe);
    let _ = std::fs::remove_file(&src);

    if out.status.success() {
        None
    } else {
        Some(String::from_utf8_lossy(&out.stderr).to_string())
    }
}

/// Names the compiler could not find, taken from `'X' is not a member of 'Y'`
/// and `'X' was not declared` diagnostics.
fn cpp_undefined_names(stderr: &str) -> Vec<String> {
    let mut out = Vec::new();
    for line in stderr.lines().filter(|l| l.contains("error:")) {
        for quoted in line.split('\u{2018}').skip(1) {
            if let Some(name) = quoted.split('\u{2019}').next() {
                let last = name.rsplit("::").next().unwrap_or(name);
                if !last.is_empty() && last.len() < 64 {
                    out.push(last.to_string());
                }
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// True when every name the block could not resolve is one its own page defines.
///
/// `tutorials/04-custom-messages-cpp.mdx` walks the reader through adding a
/// `WeatherData` message to the framework; the later blocks use
/// `horus::msg::WeatherData`, which exists only after those steps. Compiling a
/// block alone cannot see it, so reporting it would be reporting the tutorial's
/// premise. Mirrors the same absolution in docs_examples.rs.
fn only_page_defined_names(stderr: &str, page_source: &str) -> bool {
    let names = cpp_undefined_names(stderr);
    if names.is_empty() {
        return false;
    }
    names.iter().any(|n| {
        page_source.contains(&format!("struct {n}"))
            || page_source.contains(&format!("class {n}"))
            || page_source.contains(&format!("enum {n}"))
    })
}

// ─── Tests ──────────────────────────────────────────────────────────────────

/// Every public header must compile on its own.
///
/// Hermetic — needs no docs checkout. A header that only builds when some other
/// header happened to be included first breaks the moment a user includes it
/// first, and no C++ example in the docs would reveal that.
///
/// It has already caught one: `topic.hpp` used `std::function` in a constructor
/// signature without including `<functional>`, so `#include <horus/topic.hpp>`
/// (or anything pulling it in, such as `node.hpp`) failed outright on GCC 15.
#[test]
fn public_headers_are_self_contained() {
    let Some(cxx) = cxx() else {
        eprintln!("skipping: no C++ compiler found");
        return;
    };
    let inc = include_dir();
    let dir = inc.join("horus");
    let Ok(rd) = std::fs::read_dir(&dir) else {
        panic!("cannot read {}", dir.display());
    };

    let mut headers: Vec<PathBuf> = rd
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "hpp" || x == "h"))
        .collect();
    headers.sort();
    assert!(
        headers.len() >= 5,
        "found only {} public headers — the glob is wrong, making this vacuous",
        headers.len()
    );

    let tmp = tempfile::tempdir().expect("temp dir");
    let mut broken = Vec::new();
    for h in &headers {
        let name = h.file_name().unwrap().to_string_lossy().to_string();
        let src = tmp.path().join("probe.cpp");
        std::fs::write(&src, format!("#include <horus/{name}>\nint main() {{}}\n")).unwrap();
        let out = Command::new(&cxx)
            .args(["-fsyntax-only", "-std=c++17"])
            .arg("-I")
            .arg(&inc)
            .arg(&src)
            .output()
            .expect("compiler runs");
        if !out.status.success() {
            let errs = first_errors(&String::from_utf8_lossy(&out.stderr), 2);
            broken.push(format!("  <horus/{name}>\n      {}", errs.join("\n      ")));
        }
    }

    assert!(
        broken.is_empty(),
        "public headers that do not compile standalone:\n{}\n\n\
         Each is a header a user may include first. Add the missing #include to \
         the header itself rather than relying on include order.",
        broken.join("\n")
    );
}

/// True when a diagnostic is about a name the reader's own file would declare,
/// rather than a mistake about the HORUS API.
///
/// Body excerpts are compiled with `<horus/horus.hpp>` supplied, so their horus
/// calls are checked — but they still reference the surrounding scope the prose
/// describes (`sched`, `estop_pub_`, a member the reader's node owns). Failing a
/// page for that would be failing it for being an excerpt, which is the thing
/// the supplied preamble exists to allow. A wrong member on a horus type, or a
/// call that does not match a horus signature, is a different matter and stays
/// enforced.
fn is_reader_scope_error(err: &str) -> bool {
    const READER_SCOPE: &[&str] = &[
        "was not declared in this scope",
        "use of undeclared identifier",
        "has not been declared",
        "expected primary-expression",
        "expected unqualified-id",
        "does not name a type",
    ];
    const API_MISUSE: &[&str] = &[
        "no member named",
        "no matching function for call",
        "no member function",
        "too few arguments",
        "too many arguments",
        "cannot convert",
        "invalid conversion",
        "is private within this context",
        "no matching constructor",
    ];
    if API_MISUSE.iter().any(|m| err.contains(m)) {
        return false;
    }
    READER_SCOPE.iter().any(|m| err.contains(m))
}

/// Documented C++ examples must type-check.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]

fn documented_cpp_examples_compile() {
    let Some(cxx) = cxx() else {
        eprintln!("skipping: no C++ compiler found");
        return;
    };
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let filter = std::env::var("HORUS_DOCS_FILTER").ok();
    let (blocks, skipped) = collect(&docs, filter.as_deref());

    eprintln!("docs: {}", docs.display());
    eprintln!(
        "cpp blocks: {} compiled, {} skipped ({})",
        blocks.len(),
        skipped.values().sum::<usize>(),
        skipped
            .iter()
            .map(|(k, v)| format!("{k:?}={v}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    // Distinguish "the docs have no C++" from "the extractor stopped working".
    // The rewrite that landed on docs main deleted content/docs/cpp/ entirely,
    // so a bare non-empty assertion failed for a reason that is not a defect in
    // this suite — while still needing to be loud, because horus_cpp ships.
    if blocks.is_empty() && skipped.is_empty() {
        eprintln!(
            "SKIPPING: the documentation contains no C++ code blocks at all.\n  \
             horus_cpp is still a shipped crate, so its public API is now \
             entirely undocumented. If that is intended, this suite has nothing \
             to guard; if not, the C++ pages need restoring."
        );
        return;
    }
    assert!(
        !blocks.is_empty(),
        "found {} C++ blocks but none are compilable — every one was skipped \
         ({}). That is the extractor misclassifying, not the docs.",
        skipped.values().sum::<usize>(),
        skipped
            .iter()
            .map(|(k, v)| format!("{k:?}={v}"))
            .collect::<Vec<_>>()
            .join(" ")
    );

    let tmp = tempfile::tempdir().expect("temp dir");
    let mut failures: Vec<(usize, Vec<String>)> = Vec::new();
    let mut page_cache: BTreeMap<String, String> = BTreeMap::new();
    let mut page_scoped = 0usize;
    let mut advisory = 0usize;
    for (i, b) in blocks.iter().enumerate() {
        if let Some(stderr) = syntax_check(&cxx, tmp.path(), i, b) {
            let page = page_cache.entry(b.doc_file.clone()).or_insert_with(|| {
                std::fs::read_to_string(docs.join(&b.doc_file)).unwrap_or_default()
            });
            if only_page_defined_names(&stderr, page) {
                page_scoped += 1;
                continue;
            }
            let errs = first_errors(&stderr, 3);
            if !errs.is_empty() && errs.iter().all(|e| is_reader_scope_error(e)) {
                advisory += 1;
                continue;
            }
            failures.push((i, errs));
        }
    }
    if page_scoped > 0 {
        eprintln!("{page_scoped} block(s) used types their own page defines (not counted)");
    }
    if advisory > 0 {
        eprintln!(
            "{advisory} block(s) referenced only names from their surrounding scope \
             (reader's own code) — advisory, not enforced"
        );
    }

    if !failures.is_empty() {
        let mut report = String::new();
        for (i, errs) in &failures {
            let b = &blocks[*i];
            report.push_str(&format!("\n  {}:{}\n", b.doc_file, b.line));
            for e in errs {
                report.push_str(&format!("      {e}\n"));
            }
        }
        panic!(
            "{} of {} documented C++ examples do not compile:{report}\n\
             Nothing else in either repository compiles these — the docs' own \
             extractor lists only rust and python as verifiable. Fix the example, \
             add the missing #include, or mark the block as illustrative.",
            failures.len(),
            blocks.len()
        );
    }

    eprintln!("all {} documented C++ examples compile", blocks.len());
}

/// Complete C++ programs in the docs must *link*, not merely type-check.
///
/// `-fsyntax-only` proves a translation unit is well-formed; it says nothing
/// about whether the functions it calls exist. A header can declare an overload
/// the library never defines, and a template can instantiate against a signature
/// that resolves at parse time and vanishes at link time. Both produce a program
/// a reader cannot build, and both are invisible to the type-check sweep.
#[test]
#[ignore = "slow: links against the horus_cpp staticlib; runs in the scheduled docs-contract job"]
fn documented_cpp_programs_link() {
    let Some(cxx) = cxx() else {
        eprintln!("skipping: no C++ compiler found");
        return;
    };
    let Some(lib) = horus_cpp_staticlib() else {
        eprintln!("skipping: libhorus_cpp.a not built. Run `cargo build -p horus_cpp` first.");
        return;
    };
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let filter = std::env::var("HORUS_DOCS_FILTER").ok();
    let (blocks, _) = collect(&docs, filter.as_deref());

    // Only whole programs can be linked; everything else has no entry point.
    let programs: Vec<(usize, &Block)> = blocks
        .iter()
        .enumerate()
        .filter(|(_, b)| b.code.contains("int main"))
        .collect();
    eprintln!(
        "linking {} complete C++ program(s) against {}",
        programs.len(),
        lib.display()
    );
    if programs.is_empty() {
        eprintln!(
            "SKIPPING: no complete C++ programs in the documentation. See the \
             note in documented_cpp_examples_compile — the C++ pages were \
             removed from the docs while horus_cpp still ships."
        );
        return;
    }

    let tmp = tempfile::tempdir().expect("temp dir");
    let mut failures = Vec::new();
    for (i, b) in &programs {
        // A program that does not even type-check is the type-check sweep's
        // finding; reporting it twice adds noise.
        if syntax_check(&cxx, tmp.path(), *i, b).is_some() {
            continue;
        }
        if let Some(stderr) = link_check(&cxx, &lib, tmp.path(), *i, b) {
            let errs: Vec<String> = stderr
                .lines()
                .filter(|l| l.contains("undefined reference") || l.contains("error:"))
                .take(3)
                .map(|l| l.trim().to_string())
                .collect();
            failures.push(format!(
                "  {}:{}\n      {}",
                b.doc_file,
                b.line,
                if errs.is_empty() {
                    stderr.lines().take(2).collect::<Vec<_>>().join(" | ")
                } else {
                    errs.join("\n      ")
                }
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "{} documented C++ program(s) type-check but do not link:\n{}\n\n\
         The reader gets an executable that never builds. An undefined reference \
         here means a header declares something horus_cpp does not define.",
        failures.len(),
        failures.join("\n")
    );
    eprintln!("all {} complete C++ programs link", programs.len());
}

// ─── Extractor unit tests ───────────────────────────────────────────────────

#[cfg(test)]
mod extractor {
    use super::*;

    #[test]
    fn extracts_cpp_fence() {
        let md = "t\n```cpp\n#include <horus/horus.hpp>\nint main() {}\n```\n";
        let got = cpp_blocks(md, "p.mdx");
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].0.line, 2);
        assert_eq!(got[0].1, None);
    }

    #[test]
    fn ignores_other_languages() {
        let md = "```rust\nfn main() {}\nlet x = 1;\n```\n";
        assert!(cpp_blocks(md, "p.mdx").is_empty());
    }

    #[test]
    fn blocks_without_includes_are_fragments() {
        let md = "```cpp\nvoid tick() {\n  publish(x);\n}\n```\n";
        assert_eq!(cpp_blocks(md, "p.mdx")[0].1, Some(Skip::Fragment));
    }

    #[test]
    fn ellipsis_outlines_are_skipped() {
        let md = "```cpp\n#include <horus/horus.hpp>\nint main() { ... }\n```\n";
        assert_eq!(cpp_blocks(md, "p.mdx")[0].1, Some(Skip::Ellipsis));
    }

    #[test]
    fn counter_examples_are_skipped() {
        let md = "```cpp\n#include <horus/horus.hpp>\n// WRONG: this leaks\nint main() { new int; }\n```\n";
        assert_eq!(cpp_blocks(md, "p.mdx")[0].1, Some(Skip::CounterExample));
    }

    #[test]
    fn foreign_framework_blocks_are_skipped() {
        // learn/coming-from-ros2.mdx prints the ROS2 version beside the HORUS
        // one. Requiring rclcpp to be installed would be nonsense.
        let md = "```cpp\n#include <rclcpp/rclcpp.hpp>\nclass N : public rclcpp::Node {};\n```\n";
        assert_eq!(cpp_blocks(md, "p.mdx")[0].1, Some(Skip::ForeignFramework));
    }

    #[test]
    fn preamble_split_keeps_includes_outside() {
        let (pre, body) = split_preamble("#include <a.hpp>\nusing namespace x;\nif (a) { b(); }\n");
        assert!(pre.contains("#include <a.hpp>"), "{pre}");
        assert!(pre.contains("using namespace x;"), "{pre}");
        assert!(body.contains("if (a)"), "{body}");
        assert!(!body.contains("#include"), "{body}");
    }

    #[test]
    fn file_scope_complaints_are_recognized() {
        assert!(is_file_scope_complaint(
            "x.cpp:9:1: error: expected unqualified-id before 'if'"
        ));
        assert!(is_file_scope_complaint(
            "x.cpp:7:1: error: 'server' does not name a type"
        ));
        // A genuine API error must not be mistaken for one.
        assert!(!is_file_scope_complaint(
            "x.cpp:45:21: error: 'struct horus::msg::ServoCommand' has no member named 'target_position'"
        ));
    }

    #[test]
    fn first_errors_strips_temp_paths() {
        let e = "/tmp/xyz/block3.cpp:7:5: error: 'Scheduler' is not a member of 'horus'";
        assert_eq!(
            first_errors(e, 1),
            vec!["block3.cpp:7:5: error: 'Scheduler' is not a member of 'horus'"]
        );
    }
}
