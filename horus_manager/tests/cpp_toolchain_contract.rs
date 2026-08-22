//! C++ must be a first-class language, not a special case that falls through.
//!
//! `detect_languages` gave Rust and Python three ways each to be recognised —
//! legacy root manifest, generated `.horus/` manifest, or a source file — and
//! gave C++ exactly one: a root `CMakeLists.txt`. No HORUS C++ project has one.
//! `horus new --cpp` scaffolds `horus.toml` + `src/main.cpp`, and `cmake_gen`
//! writes its CMakeLists into `.horus/`.
//!
//! So every generated C++ project detected as *no language at all*, and each
//! caller drew its own wrong conclusion:
//!
//! ```text
//! $ ls src/
//! main.cpp
//! $ horus fmt --check
//! Error: No source files detected. Nothing to format.
//! $ horus lint
//! Error: No source files detected. Nothing to lint.
//! $ horus deploy user@robot
//! Error: No Cargo.toml found.          # fell through to the Rust builder
//! ```
//!
//! `horus build` worked throughout, because it uses a third detector that does
//! check extensions — which is why this survived: the language that could not
//! be formatted, linted or deployed could still be compiled.
//!
//! Run: `cargo test -p horus_manager --test cpp_toolchain_contract`

use std::path::Path;
use std::process::Command;

use horus_manager::dispatch;
use horus_manager::manifest::{detect_languages, Language};

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

fn tool_available(bin: &str) -> bool {
    Command::new(bin)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// The exact tree `horus new --cpp` produces before anything is built.
fn scaffold_cpp(dir: &Path) {
    std::fs::create_dir_all(dir.join("src")).expect("mkdir src");
    std::fs::write(
        dir.join("horus.toml"),
        "[package]\nname = \"demo\"\nversion = \"0.1.0\"\n",
    )
    .expect("write manifest");
    std::fs::write(
        dir.join("src/main.cpp"),
        "#include <horus/horus.hpp>\nint main() { return 0; }\n",
    )
    .expect("write source");
}

// ─── Detection ──────────────────────────────────────────────────────────────

/// A freshly generated C++ project, with no build output at all.
#[test]
fn a_cpp_project_is_detected_from_its_source_files_alone() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold_cpp(tmp.path());

    let langs = detect_languages(tmp.path());
    assert!(
        langs.contains(&Language::Cpp),
        "src/main.cpp is present and this is the state `horus new --cpp` \
         leaves behind; got {langs:?}"
    );
}

/// The same, through the other detector — `dispatch::detect_context` is what
/// `fmt`, `lint` and `doctor` actually consult.
#[test]
fn the_project_context_detects_cpp_from_source_files() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold_cpp(tmp.path());

    let ctx = dispatch::detect_context(tmp.path());
    assert!(
        ctx.has_cpp(),
        "ProjectContext missed a C++ project; languages = {:?}",
        ctx.languages
    );
}

/// Two detectors that disagree is how this bug survived. They must not.
#[test]
fn both_language_detectors_agree_about_cpp() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold_cpp(tmp.path());

    let via_manifest = detect_languages(tmp.path()).contains(&Language::Cpp);
    let via_dispatch = dispatch::detect_context(tmp.path()).has_cpp();
    assert_eq!(
        via_manifest, via_dispatch,
        "manifest::detect_languages and dispatch::detect_context disagree"
    );
}

/// After a build, the generated CMakeLists lives in `.horus/`, never the root.
#[test]
fn a_generated_cmakelists_counts_as_cpp() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::create_dir_all(tmp.path().join(".horus")).expect("mkdir");
    std::fs::write(tmp.path().join(".horus/CMakeLists.txt"), "project(x)\n").expect("write");

    assert!(detect_languages(tmp.path()).contains(&Language::Cpp));
}

/// A header-only project is still C++.
#[test]
fn headers_alone_are_enough() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::create_dir_all(tmp.path().join("include")).expect("mkdir");
    std::fs::write(tmp.path().join("include/lib.hpp"), "#pragma once\n").expect("write");
    // `include/` is not scanned by the shallow detector, so put one at the root
    // too — the point of this test is the extension, not the directory.
    std::fs::write(tmp.path().join("lib.hpp"), "#pragma once\n").expect("write");

    assert!(detect_languages(tmp.path()).contains(&Language::Cpp));
}

/// Detection must not become so eager that a Rust project claims to be C++.
#[test]
fn a_rust_project_is_not_mistaken_for_cpp() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::create_dir_all(tmp.path().join("src")).expect("mkdir");
    std::fs::write(tmp.path().join("src/main.rs"), "fn main() {}\n").expect("write");

    let langs = detect_languages(tmp.path());
    assert!(langs.contains(&Language::Rust));
    assert!(!langs.contains(&Language::Cpp), "{langs:?}");
}

// ─── File arguments ─────────────────────────────────────────────────────────

/// clang-format and clang-tidy are registered with no file arguments, and
/// `default_args` is fixed at detection time. Without a source path clang-tidy
/// errors outright, and clang-format reads stdin — where `--dry-run --Werror`
/// sees EOF, prints nothing and exits 0. A green check over unchecked code.
#[test]
fn cpp_sources_are_collected_for_the_tools() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold_cpp(tmp.path());
    std::fs::create_dir_all(tmp.path().join("include")).expect("mkdir");
    std::fs::write(tmp.path().join("include/robot.hpp"), "#pragma once\n").expect("write");

    let files = dispatch::cpp_source_files(tmp.path());
    let names: Vec<String> = files
        .iter()
        .map(|p| p.file_name().unwrap().to_string_lossy().to_string())
        .collect();

    assert!(names.contains(&"main.cpp".to_string()), "{names:?}");
    assert!(names.contains(&"robot.hpp".to_string()), "{names:?}");
}

/// Build output is not project source. Handing clang-tidy a generated file in
/// `.horus/cpp-build/` produces diagnostics nobody can act on.
#[test]
fn build_directories_are_not_collected() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold_cpp(tmp.path());
    for dir in [".horus/cpp-build", "build", "target"] {
        std::fs::create_dir_all(tmp.path().join(dir)).expect("mkdir");
        std::fs::write(tmp.path().join(dir).join("generated.cpp"), "int x;\n").expect("write");
    }

    let files = dispatch::cpp_source_files(tmp.path());
    assert!(
        files
            .iter()
            .all(|p| !p.to_string_lossy().contains("generated.cpp")),
        "collected build output: {files:?}"
    );
    assert_eq!(files.len(), 1, "{files:?}");
}

/// Stable ordering, so tool output diffs between runs are real.
#[test]
fn collected_sources_are_sorted() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::create_dir_all(tmp.path().join("src")).expect("mkdir");
    for name in ["z.cpp", "a.cpp", "m.cpp"] {
        std::fs::write(tmp.path().join("src").join(name), "int x;\n").expect("write");
    }
    let files = dispatch::cpp_source_files(tmp.path());
    let mut sorted = files.clone();
    sorted.sort();
    assert_eq!(files, sorted);
}

// ─── The generated project formats cleanly ──────────────────────────────────

/// `--style=file` with no config silently falls back to LLVM style, which does
/// not match the template this same command writes. The first thing `horus fmt`
/// did to a brand-new project was reformat code the user had not written.
#[test]
fn new_cpp_writes_a_clang_format_config() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let out = Command::new(horus())
        .args(["new", "demo", "--cpp"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(
        out.status.success(),
        "horus new --cpp failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );

    let cfg = tmp.path().join("demo/.clang-format");
    assert!(cfg.is_file(), "{} was not written", cfg.display());
    let text = std::fs::read_to_string(&cfg).expect("read config");
    assert!(text.contains("IndentWidth: 4"), "{text}");
}

/// The template and the config are one artefact. Changing either without the
/// other makes `horus fmt --check` fail on a project the user has not touched.
#[test]
fn the_generated_template_satisfies_the_generated_config() {
    if !tool_available("clang-format") {
        eprintln!("SKIP: clang-format not installed");
        return;
    }
    let tmp = tempfile::tempdir().expect("tempdir");
    let out = Command::new(horus())
        .args(["new", "demo", "--cpp"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(out.status.success());

    let project = tmp.path().join("demo");
    let check = Command::new("clang-format")
        .args(["--style=file", "--dry-run", "--Werror", "src/main.cpp"])
        .current_dir(&project)
        .output()
        .expect("clang-format must run");

    assert!(
        check.status.success(),
        "a freshly generated C++ project does not satisfy its own .clang-format:\n{}",
        String::from_utf8_lossy(&check.stderr)
    );
}

/// End to end, through the CLI: clean project passes, mangled project fails.
/// The second half is the one that matters — before the fix, `horus fmt
/// --check` exited 0 no matter what the file contained.
#[test]
fn fmt_check_passes_on_a_clean_project_and_fails_on_a_dirty_one() {
    if !tool_available("clang-format") {
        eprintln!("SKIP: clang-format not installed");
        return;
    }
    let tmp = tempfile::tempdir().expect("tempdir");
    let out = Command::new(horus())
        .args(["new", "demo", "--cpp"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(out.status.success());
    let project = tmp.path().join("demo");

    let clean = Command::new(horus())
        .args(["fmt", "--check"])
        .current_dir(&project)
        .stdin(std::process::Stdio::null())
        .output()
        .expect("horus fmt must run");
    assert!(
        clean.status.success(),
        "a generated project must be formatted:\n{}\n{}",
        String::from_utf8_lossy(&clean.stdout),
        String::from_utf8_lossy(&clean.stderr)
    );

    let main = project.join("src/main.cpp");
    let mut src = std::fs::read_to_string(&main).expect("read");
    src.push_str("\nvoid   badly_formatted(  int x ){return;}\n");
    std::fs::write(&main, src).expect("write");

    let dirty = Command::new(horus())
        .args(["fmt", "--check"])
        .current_dir(&project)
        .stdin(std::process::Stdio::null())
        .output()
        .expect("horus fmt must run");
    assert!(
        !dirty.status.success(),
        "unformatted C++ passed the check — this is the silent-false-pass the \
         fix exists to remove:\n{}\n{}",
        String::from_utf8_lossy(&dirty.stdout),
        String::from_utf8_lossy(&dirty.stderr)
    );
}

// ─── Parity with the Rust path ──────────────────────────────────────────────

/// Rust lint is `clippy -- -D warnings`: a finding fails the build. clang-tidy
/// exits 0 on findings by default, so the identical C++ problem was invisible
/// to CI.
#[test]
fn cpp_lint_treats_warnings_as_errors_like_the_rust_path() {
    let src = std::fs::read_to_string("src/dispatch.rs").expect("read dispatch.rs");
    let at = src
        .find("bin: \"clang-tidy\".to_string()")
        .expect("clang-tidy must still be registered");
    let block: String = src[at..].lines().take(10).collect::<Vec<_>>().join("\n");
    assert!(
        block.contains("--warnings-as-errors"),
        "clang-tidy exits 0 on findings without this:\n{block}"
    );
}

// ─── Binary size ────────────────────────────────────────────────────────────

/// A hello-world C++ node linked to a 101 MB debug executable: the whole Rust
/// runtime arrives as a static archive and the linker keeps every section.
/// `--gc-sections` needs the sections to be separate in the first place.
#[test]
fn the_generated_cmake_drops_unused_sections() {
    let src = std::fs::read_to_string("src/cmake_gen.rs").expect("read cmake_gen.rs");
    assert!(
        src.contains("--gc-sections"),
        "without this the release binary is 17.6 MB instead of 2.0 MB"
    );
    assert!(
        src.contains("-ffunction-sections"),
        "--gc-sections can only drop a section that is separate to begin with"
    );
}

/// Stripping is Release-only: taking gdb away from the debug workflow to save
/// space in the build nobody ships would be a bad trade.
#[test]
fn stripping_is_release_only() {
    let src = std::fs::read_to_string("src/cmake_gen.rs").expect("read cmake_gen.rs");
    let at = src
        .find(":-s>")
        .expect("release strip flag must be present");
    let line = src[..at].lines().last().unwrap_or_default();
    assert!(
        line.contains("CONFIG:Release"),
        "the strip flag must be guarded by the Release config: {line}"
    );
}

// ─── deploy ─────────────────────────────────────────────────────────────────

/// `deploy` had its own cmake invocation that configured with `-S .` (the
/// CMakeLists is in `.horus/`), skipped configure whenever a cache existed (so
/// `CMAKE_BUILD_TYPE` stayed Debug after `horus run` and deploy shipped a
/// 101 MB debug binary while printing "Release"), and passed neither
/// `HORUS_CPP_INCLUDE` nor `HORUS_CPP_LIB`.
#[test]
fn deploy_builds_cpp_through_the_same_path_as_run() {
    let src = std::fs::read_to_string("src/commands/deploy.rs").expect("read deploy.rs");
    let at = src
        .find("fn build_for_target_cpp")
        .expect("the C++ deploy build must still exist");
    let body: String = src[at..].lines().take(40).collect::<Vec<_>>().join("\n");

    assert!(
        body.contains("run_cpp::build_cpp"),
        "deploy must reuse the builder `horus run` and `horus build` use:\n{body}"
    );
    assert!(
        !body.contains("CMakeCache.txt"),
        "a configure guarded on an existing cache cannot change CMAKE_BUILD_TYPE, \
         which is configure-time only:\n{body}"
    );
}

/// A C++ project must not be routed to the Rust builder. Before the detector
/// fix, `horus deploy` on a generated C++ project failed with "No Cargo.toml
/// found" — a message about a language the project is not written in.
#[test]
fn deploy_does_not_route_a_cpp_project_to_the_rust_builder() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold_cpp(tmp.path());

    let ctx = dispatch::detect_context(tmp.path());
    assert_eq!(
        ctx.languages.first(),
        Some(&Language::Cpp),
        "the first detected language decides which builder deploy uses; \
         got {:?}",
        ctx.languages
    );
}

// ─── Cross-compilation ──────────────────────────────────────────────────────

/// `libhorus_cpp.a` is Rust compiled to a static archive, so a C++ program
/// cross-compiled for aarch64 cannot link a host one:
///
/// ```text
/// /usr/bin/aarch64-linux-gnu-ld.bfd: libhorus_cpp.a(...): Relocations in
///     generic ELF (EM: 62)
/// ```
///
/// The CMake toolchain file was honoured and the Rust archive was always built
/// for the host, so `horus deploy --arch aarch64` on a C++ project could not
/// link. It was invisible because deploy failed earlier and more confusingly,
/// complaining about a missing Cargo.toml — a message about a language the
/// project is not written in.
#[test]
fn the_bindings_are_built_for_the_target_architecture() {
    let src = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/commands/run/run_cpp.rs"),
    )
    .expect("run_cpp.rs");

    let at = src
        .find("fn ensure_horus_cpp(")
        .expect("the bindings resolver must exist");
    let body: String = src[at..].lines().take(70).collect::<Vec<_>>().join("\n");

    assert!(
        body.contains("target_arch"),
        "the bindings are resolved without reference to the target architecture, \
         so a cross build links a host archive"
    );
    assert!(
        body.contains("--target"),
        "cargo must be told the target triple, or it builds for the host"
    );
    assert!(
        src.contains("CARGO_TARGET_") && src.contains("_LINKER"),
        "cargo links this crate's cdylib with the host `cc` unless told \
         otherwise, which fails before the staticlib is reached"
    );
}

/// The artifact directory differs between a native and a cross build
/// (`target/<profile>` versus `target/<triple>/<profile>`). Looking in the
/// wrong one silently reuses the host archive.
#[test]
fn a_cross_build_looks_in_the_target_specific_directory() {
    let src = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/commands/run/run_cpp.rs"),
    )
    .expect("run_cpp.rs");
    let at = src.find("fn ensure_horus_cpp(").expect("resolver");
    let body: String = src[at..].lines().take(70).collect::<Vec<_>>().join("\n");
    assert!(
        body.contains(".join(t)"),
        "the library path must include the target triple for a cross build"
    );
}

/// End to end. Slow — it compiles horus_cpp and its dependency tree for the
/// target — so it is opt-in.
#[test]
#[ignore = "slow: cross-compiles horus_cpp; run with --ignored"]
fn a_cpp_project_cross_compiles_to_an_aarch64_binary() {
    let have_target = Command::new("rustup")
        .args(["target", "list", "--installed"])
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).contains("aarch64-unknown-linux-gnu"))
        .unwrap_or(false);
    if !have_target || !tool_available("aarch64-linux-gnu-gcc") {
        eprintln!("SKIP: needs the aarch64 Rust target and aarch64-linux-gnu-gcc");
        return;
    }

    let tmp = tempfile::tempdir().expect("tempdir");
    assert!(Command::new(horus())
        .args(["new", "xdemo", "--cpp"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());
    let project = tmp.path().join("xdemo");

    let out = Command::new(horus())
        .args(["deploy", "user@nonexistent-host", "--arch", "aarch64"])
        .current_dir(&project)
        .output()
        .expect("horus deploy must run");
    let text =
        String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr);

    let binary = project.join(".horus/cpp-build/xdemo");
    assert!(binary.is_file(), "no binary was produced:\n{text}");

    // The bytes decide, not the message. A build that silently produced a host
    // binary would print exactly the same thing.
    let kind = Command::new("file")
        .arg(&binary)
        .output()
        .expect("file(1) must run");
    let kind = String::from_utf8_lossy(&kind.stdout);
    assert!(
        kind.contains("aarch64"),
        "deploy --arch aarch64 produced: {kind}"
    );
}
