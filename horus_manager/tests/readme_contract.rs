//! The README's Rust must compile, and must teach what `horus new` writes.
//!
//! The front-page Quick Start — the first HORUS code anyone sees — did not
//! compile:
//!
//! ```text
//! error[E0277]: the `?` operator can only be applied to values that implement `Try`
//!   --> src/main.rs:34:15
//!    |
//! 34 |     sched.add(Sensor::new()?).order(0).build()?;
//!    |               ^^^^^^^^^^^^^^ the `?` operator cannot be applied to type `Sensor`
//! ```
//!
//! `node!` generates `pub fn new() -> Self`, and the README wrote `new()?`,
//! twice. It survived because `docs_examples.rs` — which exists precisely to
//! cargo-check documentation Rust — enumerates `$HORUS_DOCS_DIR/content/docs`
//! and never reads the repository's own README.
//!
//! The second half is style. The README taught the `message!`/`node!` DSL;
//! `horus new` (with no flag, with `--rust`, and as the quick-start tells
//! interactive readers to answer) emitted plain structs with an
//! `Option<Topic<_>>` built in `init()`. The two shared `use horus::prelude::*`
//! and little else, so a reader who copied the README and a reader who ran the
//! README's own command met different languages.
//!
//! The third part is the links, and they went dead as a side effect of fixing
//! something else. Lowering the concept load on the Quick Start meant deferring
//! execution classes, `.on_miss()` and safe mode to "a second page" — and the
//! pointer written for that deferral, `/concepts/execution-classes`, has never
//! existed. `/learn/coming-from-ros2`, `/tutorials` and `/recipes` were dead the
//! same way. `horus-docs` sets `dynamicParams = false` and pre-renders only the
//! slugs it has an `.mdx` file for, so every one of those was a hard 404 rather
//! than a redirect. Nothing caught it because `docs_contract.rs` checks the
//! site's own navigation components and never the repository's front page.
//!
//! The fourth is the Safety section, which described a ladder the scheduler does
//! not have (`warn -> skip -> isolate -> safe state`; `Skip` and `SafeMode` are
//! `Miss` policies, and the real stages between and after are rate reduction and
//! kill) and carried none of the caveats `Miss::SafeMode`'s own doc comment
//! carries.
//!
//! Run: `cargo test -p horus_manager --test readme_contract`

use std::path::{Path, PathBuf};
use std::process::Command;

/// The English README and its five translations.
const ALL_READMES: &[&str] = &[
    "README.md",
    "README.zh-CN.md",
    "README.pt-BR.md",
    "README.ja.md",
    "README.es.md",
    "README.de.md",
];

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn readme() -> String {
    std::fs::read_to_string(repo_root().join("README.md")).expect("README.md must exist")
}

/// A file under `horus_core/`, so a claim in the README can be checked against
/// the code that has to honour it rather than against another document.
fn core_source(rel: &str) -> String {
    let path = repo_root().join("horus_core").join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

/// Markdown and doc comments both hard-wrap, so a phrase a reader sees as one
/// sentence can straddle a newline. Compare against the text with its line
/// breaks flattened out, or the check depends on where the paragraph happened
/// to wrap.
fn flatten(text: &str) -> String {
    text.replace("///", " ")
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

/// `horus-docs` checked out beside `horus`, if it is.
fn docs_content_dir() -> Option<PathBuf> {
    repo_root()
        .parent()
        .map(|p| p.join("horus-docs/content/docs"))
        .filter(|p| p.is_dir())
}

/// The first ```rust fence in the README — the Quick Start.
fn quick_start() -> String {
    let text = readme();
    let start = text
        .find("```rust")
        .expect("README must contain a Rust example")
        + 7;
    let rest = &text[start..];
    let end = rest.find("```").expect("unterminated fence");
    rest[..end].trim_start_matches('\n').to_string()
}

/// The Python `horus new --python` writes, obtained by running it.
fn generated_main_py() -> String {
    let tmp = tempfile::tempdir().expect("tempdir");
    let out = Command::new(env!("CARGO_BIN_EXE_horus"))
        .args(["new", "demo", "--python", "--yes"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(
        out.status.success(),
        "horus new --python failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    // Python's scaffold puts `main.py` at the project root, not under `src/`.
    std::fs::read_to_string(tmp.path().join("demo/main.py")).expect("generated main.py")
}

/// The Rust `horus new` writes, obtained by running it.
fn generated_main_rs(extra: &[&str]) -> String {
    let tmp = tempfile::tempdir().expect("tempdir");
    let mut args = vec!["new", "demo", "--rust"];
    args.extend_from_slice(extra);
    let out = Command::new(env!("CARGO_BIN_EXE_horus"))
        .args(&args)
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(
        out.status.success(),
        "horus new {extra:?} failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    std::fs::read_to_string(tmp.path().join("demo/src/main.rs")).expect("generated main.rs")
}

// ─── It has to compile ──────────────────────────────────────────────────────

/// Compiles the Quick Start against exactly the dependencies a `horus new -r`
/// project has — no more, or the test is easier than reality.
#[test]
#[ignore = "slow: builds a scratch crate; run in the docs-contract job"]
fn the_readme_quick_start_compiles() {
    let root = repo_root();
    let h = root.display();
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::create_dir_all(tmp.path().join("src")).expect("mkdir");
    std::fs::write(
        tmp.path().join("Cargo.toml"),
        format!(
            r#"[package]
name = "readme_quick_start"
version = "0.0.0"
edition = "2021"

[workspace]

[dependencies]
horus = {{ path = "{h}/horus" }}
horus_core = {{ path = "{h}/horus_core" }}
horus_macros = {{ path = "{h}/horus_macros" }}
serde = {{ version = "1", features = ["derive"] }}

[patch."https://github.com/softmata/horus-robotics.git"]
horus_core = {{ path = "{h}/horus_core" }}
horus_types = {{ path = "{h}/horus_types" }}
horus_macros = {{ path = "{h}/horus_macros" }}

[patch."https://github.com/softmata/horus-tf.git"]
horus_core = {{ path = "{h}/horus_core" }}
horus_macros = {{ path = "{h}/horus_macros" }}
"#
        ),
    )
    .expect("write manifest");
    std::fs::write(tmp.path().join("src/main.rs"), quick_start()).expect("write source");

    let out = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()))
        .args(["check", "--quiet", "--message-format", "short"])
        .current_dir(tmp.path())
        .env("CARGO_TARGET_DIR", root.join("target/readme-contract"))
        .output()
        .expect("cargo check must run");

    assert!(
        out.status.success(),
        "the README's Quick Start does not compile — it is the first HORUS code \
         anyone sees:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

// ─── It has to teach what the tool writes ───────────────────────────────────

/// The constructs a reader carries from the README into their own project. If
/// the README shows them and the template does not, the reader's first edit is
/// against an API the page never mentioned.
const SHARED_CONSTRUCTS: &[&str] = &[
    "use horus::prelude::*;",
    "impl Node for",
    "fn tick(&mut self)",
    "Topic::new(",
    "Scheduler::new()",
    ".order(0)",
    ".build()?",
    "scheduler.run()",
];

#[test]
fn the_readme_and_the_default_template_speak_the_same_language() {
    let readme = quick_start();
    let template = generated_main_rs(&[]);

    let mut missing = Vec::new();
    for construct in SHARED_CONSTRUCTS {
        // `sched` vs `scheduler` is a naming difference, not a language one.
        let in_readme =
            readme.contains(construct) || readme.contains(&construct.replace("scheduler", "sched"));
        let in_template = template.contains(construct)
            || template.contains(&construct.replace("scheduler", "sched"));
        if in_readme != in_template {
            missing.push(format!(
                "{construct:?}: README={in_readme} template={in_template}"
            ));
        }
    }

    assert!(
        missing.is_empty(),
        "the README teaches constructs the generated project does not use, or \
         the reverse:\n  {}\n\nREADME:\n{readme}\n\nTEMPLATE:\n{template}",
        missing.join("\n  ")
    );
}

/// The specific triad that appeared nowhere in the docs or in `examples/`, and
/// forced `if let Some(ref topic)` around every publish.
#[test]
fn the_template_does_not_wrap_its_topic_in_an_option() {
    let template = generated_main_rs(&[]);
    assert!(
        !template.contains("Option<Topic<"),
        "a fallible constructor says this without the ceremony:\n{template}"
    );
    assert!(
        !template.contains("if let Some(ref"),
        "no unwrapping should be needed to publish:\n{template}"
    );
}

/// The scheduler vocabulary the README introduces has to appear somewhere the
/// reader can edit it, or those words are only ever seen in prose.
#[test]
fn the_template_shows_the_scheduler_constructs_the_readme_introduces() {
    let template = generated_main_rs(&[]);
    for construct in ["tick_rate(", ".rate(", ".on_miss("] {
        assert!(
            template.contains(construct),
            "the README explains {construct} but no generated project contains \
             it:\n{template}"
        );
    }
}

/// Both templates must build. The macro arm is opt-in, not unmaintained.
#[test]
fn the_macro_template_constructor_is_infallible() {
    let template = generated_main_rs(&["--macro"]);
    assert!(
        template.contains("node!"),
        "--macro must emit the DSL:\n{template}"
    );
    // `node!` generates `pub fn new() -> Self`. Writing `new()?` against it is
    // exactly the error the README carried.
    assert!(
        !template.contains("Controller::new()?"),
        "`node!` generates an infallible constructor; `?` on it does not \
         compile:\n{template}"
    );
    assert!(
        template.contains(".build()?"),
        "`build()` returns a Result; discarding it warns:\n{template}"
    );
}

/// The DSL stays reachable — it is an alternative, not a deletion.
#[test]
fn the_readme_points_at_the_macro_alternative() {
    let text = readme();
    assert!(
        text.contains("--macro"),
        "the README should say how to get the shorter form"
    );
}

// ─── Its links have to go somewhere ─────────────────────────────────────────

const DOCS_ORIGIN: &str = "https://docs.horusrobotics.dev";

/// Every `docs.horusrobotics.dev` reference a README makes, as
/// `(path, fragment)`. The router keys on the path; the fragment has to match a
/// heading on the page it lands on.
fn docs_links(body: &str) -> Vec<(String, String)> {
    let mut links = Vec::new();
    let mut rest = body;
    while let Some(at) = rest.find(DOCS_ORIGIN) {
        rest = &rest[at + DOCS_ORIGIN.len()..];
        let end = rest
            .find(|c: char| c.is_whitespace() || matches!(c, ')' | '"' | '\'' | '>' | ']' | ','))
            .unwrap_or(rest.len());
        let (path, fragment) = match rest[..end].split_once('#') {
            Some((p, f)) => (p, f),
            None => (&rest[..end], ""),
        };
        let link = (path.trim_end_matches('/').to_string(), fragment.to_string());
        if !links.contains(&link) {
            links.push(link);
        }
    }
    links
}

/// The `.mdx` `app/[...slug]/page.tsx` renders a slug from, if there is one. It
/// builds its routes from the `.mdx` tree, with `index.mdx` standing for its
/// directory.
fn page_for(content: &Path, path: &str) -> Option<PathBuf> {
    let slug = path.trim_start_matches('/');
    if slug.is_empty() {
        // The bare origin is the site's own landing page, not a routed slug.
        return None;
    }
    let direct = content.join(format!("{slug}.mdx"));
    if direct.is_file() {
        return Some(direct);
    }
    let index = content.join(slug).join("index.mdx");
    index.is_file().then_some(index)
}

/// `lib/mdx.tsx`'s `headingId`, in Rust: lowercase, every run of non-alphanumeric
/// characters becomes one `-`, and leading and trailing `-` are dropped. Inline
/// markdown is stripped first, the way `headingText` flattens a heading's React
/// children before the same transform runs on the site.
fn heading_id(heading: &str) -> String {
    let text: String = heading
        .trim_start_matches('#')
        .trim()
        .chars()
        .filter(|c| !matches!(c, '`' | '*' | '_' | '[' | ']'))
        .collect();
    let mut id = String::new();
    for ch in text.to_lowercase().chars() {
        if ch.is_ascii_alphanumeric() {
            id.push(ch);
        } else if !id.ends_with('-') {
            id.push('-');
        }
    }
    id.trim_matches('-').to_string()
}

/// A 404 from the front page is worse than a missing page, because the reader
/// concludes the docs are broken rather than that the topic is unwritten. And
/// `dynamicParams = false` means there is no soft landing: an unknown slug is
/// not redirected anywhere, it is a hard 404.
#[test]
fn every_docs_link_in_the_readmes_resolves_to_a_page() {
    let Some(content) = docs_content_dir() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let mut dead = Vec::new();
    let mut checked = 0usize;
    for name in ALL_READMES {
        let body = std::fs::read_to_string(repo_root().join(name))
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        for (path, fragment) in docs_links(&body) {
            checked += 1;
            if path.is_empty() && fragment.is_empty() {
                continue;
            }
            let Some(page) = page_for(&content, &path) else {
                dead.push(format!("{name}: {DOCS_ORIGIN}{path} — no page"));
                continue;
            };
            if fragment.is_empty() {
                continue;
            }
            // A fragment that matches nothing lands the reader at the top of the
            // page with no sign that the section they were sent to is missing.
            let text = std::fs::read_to_string(&page).unwrap_or_default();
            let found = text
                .lines()
                .filter(|l| l.starts_with('#'))
                .any(|l| heading_id(l) == fragment);
            if !found {
                dead.push(format!(
                    "{name}: {DOCS_ORIGIN}{path}#{fragment} — page exists, no \
                     heading with that id"
                ));
            }
        }
    }

    assert!(
        checked > 0,
        "no docs links were found in any README — the extractor is broken and \
         this test is vacuous"
    );
    assert!(
        dead.is_empty(),
        "these README links do not land where they say. `horus-docs` renders \
         `<slug>` from `content/docs/<slug>.mdx` (or `<slug>/index.mdx`) and \
         sets `dynamicParams = false`, so a slug with no file behind it is a \
         hard 404 rather than a redirect:\n  {}",
        dead.join("\n  ")
    );
}

// ─── Its safety claims have to be the ones the scheduler makes ──────────────

/// The `DegradationStage` variants, read out of the enum body so the test
/// notices a stage being added rather than only one being renamed.
fn degradation_stages(monitor: &str) -> Vec<String> {
    let start = monitor
        .find("enum DegradationStage {")
        .expect("safety_monitor.rs must define DegradationStage");
    let body = &monitor[start..];
    let end = body.find("\n}").expect("unterminated enum");
    body[..end]
        .lines()
        .skip(1)
        .map(str::trim)
        .filter(|l| !l.starts_with("//") && l.ends_with(','))
        .map(|l| l.trim_end_matches(',').to_string())
        .collect()
}

/// The word the README is expected to use for each stage. `Normal` is the
/// absence of degradation and has nothing to appear as.
const STAGE_WORDS: &[(&str, &str)] = &[
    ("Warned", "warn"),
    ("RateReduced", "rate"),
    ("Isolated", "isolate"),
    ("Killed", "kill"),
];

/// The first line of the graduated-watchdog bullet — the ladder itself.
fn watchdog_ladder() -> String {
    readme()
        .lines()
        .find(|l| l.contains("**Graduated watchdog**"))
        .expect("the README's Safety section must describe the graduated watchdog")
        .to_string()
}

/// The README advertised `warn → skip → isolate → safe state`. Two of those
/// four are not stages of the graduated watchdog at all — `Skip` and `SafeMode`
/// are `Miss` policies, chosen per node with `.on_miss()`, and they neither
/// escalate nor recover. The two stages the line left out are the ones a reader
/// most needs to know about: the watchdog halves a node's rate, and then kills
/// it.
#[test]
fn the_readme_watchdog_ladder_matches_the_degradation_stages() {
    let monitor = core_source("src/scheduling/safety_monitor.rs");
    let stages = degradation_stages(&monitor);
    let ladder = watchdog_ladder();
    let lower = ladder.to_lowercase();

    for stage in &stages {
        if stage == "Normal" {
            continue;
        }
        let (_, word) = STAGE_WORDS
            .iter()
            .find(|(variant, _)| variant == stage)
            .unwrap_or_else(|| {
                panic!(
                    "DegradationStage::{stage} is new — decide what the README's \
                     ladder should call it and add it to STAGE_WORDS"
                )
            });
        assert!(
            lower.contains(word),
            "the README's watchdog ladder never mentions \
             DegradationStage::{stage}:\n  {ladder}"
        );
    }

    // A `Miss` policy in the ladder is the original defect: it reads as a stage
    // the watchdog escalates through, and it is a per-node choice that does not
    // escalate at all.
    for policy in ["skip", "safe state", "safe-state"] {
        assert!(
            !lower.contains(policy),
            "the README's watchdog ladder lists `{policy}`, which is a `Miss` \
             policy set with `.on_miss()`, not a stage of the graduated \
             watchdog:\n  {ladder}"
        );
    }
}

/// `Miss::SafeMode` sounds like a latched state machine with isolation and a
/// recovery path. It is a hook: the scheduler calls `enter_safe_state()` once on
/// the way in and the node goes on ticking. The variant's doc comment has said
/// so for a while and the docs site says so too; the README, which is where the
/// comparison table's "safe-state" cell sends a reader, said nothing — so the
/// one surface that sells the feature was the one surface that did not qualify
/// it.
#[test]
fn the_readme_carries_the_safe_mode_caveats_the_variant_documents() {
    let rt_node = flatten(&core_source("src/core/rt_node.rs"));
    let readme = flatten(&readme());

    const CAVEATS: &[(&str, &str)] = &[
        (
            "keeps ticking",
            "the node goes on ticking after it is safed",
        ),
        ("not isolated", "the node is not isolated"),
        (
            "is_safe_state()",
            "the scheduler never polls `is_safe_state()`",
        ),
    ];

    for (phrase, what) in CAVEATS {
        assert!(
            rt_node.contains(phrase),
            "`Miss::SafeMode`'s doc comment no longer says {what} — check what \
             the scheduler actually does before relaxing the README"
        );
        assert!(
            readme.contains(phrase),
            "the README's Safety section does not say {what}. A reader who sees \
             `safe-state` in the comparison table and nothing else writes a \
             handler for a latched state machine that isolates the node, and \
             gets a hook that fires once and leaves the node running"
        );
    }
}

// ─── Its Python has to be a spelling the tool also writes ───────────────────

/// How a `pubs=`/`subs=` argument is spelled, by the character that opens it.
fn topic_arg_shapes(source: &str) -> Vec<&'static str> {
    let mut shapes: Vec<&'static str> = Vec::new();
    for key in ["pubs=", "subs="] {
        let mut rest = source;
        while let Some(at) = rest.find(key) {
            rest = &rest[at + key.len()..];
            let shape = match rest.chars().next() {
                Some('"') | Some('\'') => "a bare topic string",
                Some('[') => "a list of topics",
                Some('{') => "a dict of typed topics",
                _ => continue,
            };
            if !shapes.contains(&shape) {
                shapes.push(shape);
            }
        }
    }
    shapes
}

/// The README wrote `pubs=["sensor.data"]` and `horus new --python` wrote
/// `pubs="motors.cmd_vel"`. Both work — the parameter is typed
/// `Optional[Union[List[str], str, Dict[str, Dict]]]` — but a reader who copied
/// one and ran the other had no way to find that out short of reading the
/// binding source, so the difference read as a bug in whichever they trusted
/// less.
#[test]
fn the_readme_documents_every_topic_shape_the_python_template_uses() {
    let template = generated_main_py();
    let template_shapes = topic_arg_shapes(&template);
    assert!(
        !template_shapes.is_empty(),
        "the generated Python passes no `pubs=`/`subs=` argument — the \
         extractor is broken and this test is vacuous:\n{template}"
    );

    let readme = readme();
    let readme_shapes = topic_arg_shapes(&readme);
    for shape in &template_shapes {
        assert!(
            readme_shapes.contains(shape),
            "`horus new --python` writes {shape} for `pubs=`/`subs=` and the \
             README never shows that spelling, so nothing tells a reader the two \
             are the same node. Either match the template or say both are \
             accepted.\n\nREADME uses: {readme_shapes:?}\n\nTEMPLATE:\n{template}"
        );
    }
}

/// The disagreement is closed at the template end — `horus new --python` now
/// writes the list form the README always used — but both spellings stay legal
/// and both stay in circulation: the docs site has 52 list-form `pubs=` call
/// sites against 22 bare-string ones. A reader who meets the other one still has
/// to be able to find out from the README that it is the same call, which is the
/// part of GEN-2 no template change can cover.
#[test]
fn the_readme_says_a_bare_topic_string_and_a_one_element_list_are_the_same() {
    let shapes = topic_arg_shapes(&flatten(&readme()));
    for shape in ["a list of topics", "a bare topic string"] {
        assert!(
            shapes.contains(&shape),
            "the README never shows {shape} for `pubs=`/`subs=`. Both are \
             accepted — the parameter is typed `Optional[Union[List[str], str, \
             Dict[str, Dict]]]` — and a reader who finds the spelling the README \
             omits has nowhere to check whether the difference matters.\n\n\
             README uses: {shapes:?}"
        );
    }
}
