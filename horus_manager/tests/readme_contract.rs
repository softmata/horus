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
//! Run: `cargo test -p horus_manager --test readme_contract`

use std::path::{Path, PathBuf};
use std::process::Command;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn readme() -> String {
    std::fs::read_to_string(repo_root().join("README.md")).expect("README.md must exist")
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
