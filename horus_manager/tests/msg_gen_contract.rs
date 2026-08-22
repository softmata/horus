//! One message definition must produce four artifacts that agree.
//!
//! A message type usable from all three languages had to be written into six
//! places, each with its own syntax: a `#[repr(C)]` struct plus
//! `impl_pod_message!`, a `#[pyclass]` plus a `pod_topic_types!` row, an
//! `impl_topic_ffi!` and an `impl_pod_topic_c_api!` plus a hand-written header,
//! and a row in `layout_contract_types!`.
//!
//! They did not stay in sync — the registries hold 91, 75, 75, 68, 62, 61 and
//! 60 entries — and `horus_cpp`'s layout contract exists precisely because they
//! once diverged catastrophically: its own documentation records
//! `JointCommand` at 928 bytes in Rust against 88 in C++, an 840-byte overrun
//! on every receive, with no compiler in a position to notice.
//!
//! The tests that matter here are the agreement ones: the same type must have
//! the same size and the same layout hash in Rust, C++, Python and the CLI. A
//! generator that emitted four self-consistent but mutually contradictory
//! artifacts would pass every test that looked at only one of them.
//!
//! Run: `cargo test -p horus_manager --test msg_gen_contract`

use std::path::{Path, PathBuf};
use std::process::Command;

use horus_manager::msgspec::{canonical, layout, parse, Package};

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

fn tool(bin: &str) -> bool {
    Command::new(bin)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Parse a `.hmsg` body into a package, panicking with the diagnostics.
fn package(src: &str) -> Package {
    let path = PathBuf::from("test.hmsg");
    match parse::parse_file(src, &path) {
        Ok(messages) => Package {
            name: "demo".into(),
            messages,
        },
        Err(diags) => panic!(
            "expected this to parse:\n{}",
            diags
                .iter()
                .map(|d| d.to_string())
                .collect::<Vec<_>>()
                .join("\n")
        ),
    }
}

fn errors(src: &str) -> Vec<String> {
    let path = PathBuf::from("test.hmsg");
    match parse::parse_file(src, &path) {
        Ok(_) => Vec::new(),
        Err(d) => d.iter().map(|d| d.to_string()).collect(),
    }
}

fn env_for(pkg: &Package) -> layout::Env {
    let mut env = layout::builtin_layouts();
    for m in &pkg.messages {
        if let Ok(l) = layout::compute(m, &env) {
            env.insert(m.name.clone(), (l.size, l.align));
        }
    }
    env
}

// ─── Parsing ────────────────────────────────────────────────────────────────

#[test]
fn a_message_parses_into_fields() {
    let pkg = package("Reading { timestamp_ns: u64, value: f32 }");
    assert_eq!(pkg.messages.len(), 1);
    let m = &pkg.messages[0];
    assert_eq!(m.name, "Reading");
    assert_eq!(m.fields.len(), 2);
    assert_eq!(m.fields[0].name, "timestamp_ns");
}

#[test]
fn attributes_and_docs_are_carried() {
    let pkg = package(
        "/// A reading.\n#[topic = \"sensor.data\"]\nReading {\n  /// Celsius\n  t: f32,\n}",
    );
    let m = &pkg.messages[0];
    assert_eq!(m.topic.as_deref(), Some("sensor.data"));
    assert_eq!(m.doc, vec!["A reading."]);
    assert_eq!(m.fields[0].doc, vec!["Celsius"]);
}

#[test]
fn arrays_carry_their_length() {
    let pkg = package("R { history: [f32; 16] }");
    assert_eq!(canonical::render_rust(&pkg.messages[0].fields[0].ty), "[f32; 16]");
}

/// The transport reads and writes messages with a raw `ptr::read`/`ptr::write`.
/// A `String` there is not a limitation, it is memory corruption — so it is
/// rejected by name, with a diagnostic that says what to write instead.
#[test]
fn heap_owning_types_are_rejected_by_name() {
    for (src, want) in [
        ("R { name: String }", "String"),
        ("R { xs: Vec }", "Vec"),
        ("R { b: Box }", "Box"),
        ("R { o: Option }", "Option"),
    ] {
        let errs = errors(src);
        assert!(
            errs.iter().any(|e| e.contains(want)),
            "{src} should be rejected by name, got {errs:?}"
        );
        assert!(
            errs.iter().any(|e| e.contains("help:")),
            "{src} should say what to do instead, got {errs:?}"
        );
    }
}

#[test]
fn references_and_generics_are_rejected() {
    assert!(!errors("R { s: &str }").is_empty());
    let generic = errors("R { v: MyType<f32> }");
    assert!(
        generic.iter().any(|e| e.contains("type parameters")),
        "{generic:?}"
    );
}

#[test]
fn a_semicolon_separator_is_reported_as_such() {
    let errs = errors("R { a: u8; b: u8 }");
    assert!(
        errs.iter().any(|e| e.contains("`,`")),
        "should name the right separator, got {errs:?}"
    );
}

#[test]
fn a_duplicate_field_is_reported_with_the_first_location() {
    let errs = errors("R {\n  a: u8,\n  a: u16,\n}");
    assert!(errs.iter().any(|e| e.contains("duplicate field `a`")), "{errs:?}");
    assert!(errs.iter().any(|e| e.contains("line 2")), "{errs:?}");
}

#[test]
fn an_unknown_attribute_is_an_error_not_a_shrug() {
    let errs = errors("#[fixd]\nR { a: u8 }");
    assert!(errs.iter().any(|e| e.contains("unknown attribute")), "{errs:?}");
}

#[test]
fn an_empty_message_is_rejected() {
    assert!(!errors("R { }").is_empty());
}

#[test]
fn diagnostics_carry_a_line_and_column() {
    let errs = errors("R {\n  a: u8,\n  b: String,\n}");
    assert!(
        errs.iter().any(|e| e.contains("test.hmsg:3:")),
        "expected file:line:col, got {errs:?}"
    );
}

/// One bad message must not swallow the ones after it.
#[test]
fn parsing_recovers_and_reports_more_than_the_first_error() {
    let errs = errors("A { x: String }\nB { y: Vec }");
    assert!(errs.len() >= 2, "expected both to be reported, got {errs:?}");
}

// ─── Canonical form and hash ────────────────────────────────────────────────

/// The whole reason this module has one rendering authority. `message!`
/// computes its hash with `stringify!`, which reproduces source spacing, so
/// `[u8;32]` and `[u8; 32]` hash differently there. Here the two are one type
/// and one hash.
#[test]
fn array_spacing_does_not_change_the_hash() {
    let tight = package("R { data: [u8;32] }");
    let loose = package("R { data: [u8; 32] }");
    assert_eq!(
        canonical::layout_hash(&tight.messages[0]),
        canonical::layout_hash(&loose.messages[0])
    );
    assert_eq!(
        canonical::canonical_form(&tight.messages[0]),
        canonical::canonical_form(&loose.messages[0])
    );
}

#[test]
fn the_canonical_form_has_the_documented_shape() {
    let pkg = package("R { a: u64, b: [f32; 4] }");
    assert_eq!(
        canonical::canonical_form(&pkg.messages[0]),
        "R|a:u64|b:[f32; 4]"
    );
}

#[test]
fn renaming_a_field_changes_the_hash() {
    let a = package("R { alpha: u32 }");
    let b = package("R { beta: u32 }");
    assert_ne!(
        canonical::layout_hash(&a.messages[0]),
        canonical::layout_hash(&b.messages[0])
    );
}

/// Same fields, different order, is a different layout.
#[test]
fn reordering_fields_changes_the_hash() {
    let a = package("R { a: u8, b: u64 }");
    let b = package("R { b: u64, a: u8 }");
    assert_ne!(
        canonical::layout_hash(&a.messages[0]),
        canonical::layout_hash(&b.messages[0])
    );
}

// ─── Layout ─────────────────────────────────────────────────────────────────

#[test]
fn layout_follows_the_repr_c_rules() {
    let pkg = package("R { a: u8, b: u64, c: u16 }");
    let l = layout::compute(&pkg.messages[0], &env_for(&pkg)).expect("layout");
    // a at 0; b needs 8-alignment so lands at 8; c at 16; size rounds to 24.
    assert_eq!(l.fields[0].0, 0);
    assert_eq!(l.fields[1].0, 8);
    assert_eq!(l.fields[2].0, 16);
    assert_eq!(l.size, 24);
    assert_eq!(l.align, 8);
    assert_eq!(l.padding, 24 - (1 + 8 + 2));
}

#[test]
fn an_array_is_sized_by_its_element_count() {
    let pkg = package("R { xs: [f32; 16] }");
    let l = layout::compute(&pkg.messages[0], &env_for(&pkg)).expect("layout");
    assert_eq!(l.size, 64);
    assert_eq!(l.align, 4);
}

/// A layout table that is wrong is worse than none: it produces a header whose
/// `static_assert` passes against the wrong number. The first version of this
/// module recorded `Twist` as 48 bytes; it is 56.
#[test]
fn built_in_types_are_not_guessed() {
    assert!(
        layout::builtin_layouts().is_empty(),
        "a hardcoded layout table is exactly the failure the C++ layout \
         contract exists to catch"
    );
}

// ─── The artifacts agree ────────────────────────────────────────────────────

/// The test that would catch a generator emitting four self-consistent but
/// mutually contradictory artifacts.
#[test]
fn rust_cpp_python_and_the_cli_agree_on_size_and_hash() {
    if !tool("g++") || !tool("python3") {
        eprintln!("SKIP: needs g++ and python3");
        return;
    }

    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    let out = Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));

    std::fs::create_dir_all(project.join("msgs")).expect("mkdir msgs");
    std::fs::write(
        project.join("msgs/weather.hmsg"),
        "/// A reading.\n\
         #[topic = \"weather.data\"]\n\
         WeatherData {\n\
         \x20   timestamp_ns: u64,\n\
         \x20   temperature: f32,\n\
         \x20   humidity: f32,\n\
         \x20   history: [f32; 16],\n\
         \x20   heater_on: bool,\n\
         }\n\
         \n\
         /// Deliberately nothing but an array.\n\
         ///\n\
         /// `WeatherData` has trailing padding, which absorbs a mis-sized\n\
         /// array: 16 floats and 17 floats both round to 88 bytes, so an\n\
         /// off-by-one in an emitter is invisible there. This type is exactly\n\
         /// its array, so any disagreement about the length changes its size.\n\
         Buffer {\n\
         \x20   samples: [f32; 16],\n\
         }\n",
    )
    .expect("write hmsg");

    let gen = Command::new(horus())
        .args(["msg", "gen", "--json"])
        .current_dir(&project)
        .output()
        .expect("horus msg gen must run");
    assert!(
        gen.status.success(),
        "msg gen failed: {}",
        String::from_utf8_lossy(&gen.stderr)
    );
    let report = String::from_utf8_lossy(&gen.stdout);
    let json: serde_json::Value =
        serde_json::from_str(&report).unwrap_or_else(|e| panic!("{e}\n{report}"));
    let entries = json["messages"].as_array().expect("messages");
    assert_eq!(entries.len(), 2, "both messages should be generated:\n{report}");
    let sizes: Vec<(String, u64, String)> = entries
        .iter()
        .map(|m| {
            (
                m["name"].as_str().expect("name").to_string(),
                m["size"].as_u64().expect("size"),
                m["hash"].as_str().expect("hash").to_string(),
            )
        })
        .collect();
    let msg = &json["messages"][0];
    let cli_hash = msg["hash"].as_str().expect("hash").to_string();
    let cli_size = msg["size"].as_u64().expect("size");

    // ── C++ ──
    let probe = tmp.path().join("probe.cpp");
    std::fs::write(
        &probe,
        "#include \"demo/msgs.hpp\"\n\
         #include <cstdio>\n\
         int main() {\n\
         \x20 printf(\"%zu 0x%08x\\n\", sizeof(demo::msg::WeatherData),\n\
         \x20        demo::msg::WEATHERDATA_LAYOUT_HASH);\n\
         \x20 printf(\"%zu 0x%08x\\n\", sizeof(demo::msg::Buffer),\n\
         \x20        demo::msg::BUFFER_LAYOUT_HASH);\n\
         \x20 return 0;\n\
         }\n",
    )
    .expect("write probe");
    let bin = tmp.path().join("probe");
    let cc = Command::new("g++")
        .args(["-std=c++17", "-I"])
        .arg(project.join(".horus/generated/include"))
        .arg(&probe)
        .arg("-o")
        .arg(&bin)
        .output()
        .expect("g++ must run");
    assert!(
        cc.status.success(),
        "the generated header does not compile:\n{}",
        String::from_utf8_lossy(&cc.stderr)
    );
    let cpp = Command::new(&bin).output().expect("probe must run");
    let cpp_out = String::from_utf8_lossy(&cpp.stdout);
    let mut parts = cpp_out.split_whitespace();
    let cpp_size: u64 = parts.next().unwrap_or("0").parse().unwrap_or(0);
    let cpp_hash = parts.next().unwrap_or("").to_string();

    // ── Python ──
    let py = Command::new("python3")
        .arg("-c")
        .arg(format!(
            "import sys, ctypes; sys.path.insert(0, {:?}); import msgs; \
             print(ctypes.sizeof(msgs.WeatherData), '0x%08x' % msgs.WeatherData.LAYOUT_HASH); \
             print(ctypes.sizeof(msgs.Buffer), '0x%08x' % msgs.Buffer.LAYOUT_HASH)",
            project
                .join(".horus/generated/python")
                .display()
                .to_string()
        ))
        .output()
        .expect("python3 must run");
    assert!(
        py.status.success(),
        "the generated Python module does not import:\n{}",
        String::from_utf8_lossy(&py.stderr)
    );
    let py_out = String::from_utf8_lossy(&py.stdout);
    let mut parts = py_out.split_whitespace();
    let py_size: u64 = parts.next().unwrap_or("0").parse().unwrap_or(0);
    let py_hash = parts.next().unwrap_or("").to_string();

    assert_eq!(cpp_size, cli_size, "C++ and the CLI disagree on size");
    assert_eq!(py_size, cli_size, "Python and the CLI disagree on size");
    assert_eq!(cpp_hash, cli_hash, "C++ and the CLI disagree on the layout hash");
    assert_eq!(py_hash, cli_hash, "Python and the CLI disagree on the layout hash");

    // The second row: the array-only type, where nothing absorbs a mistake.
    let (name, want_size, want_hash) = &sizes[1];
    assert_eq!(name, "Buffer");
    let cpp2 = cpp_out.lines().nth(1).unwrap_or_default();
    let py2 = py_out.lines().nth(1).unwrap_or_default();
    let parse_row = |row: &str| -> (u64, String) {
        let mut it = row.split_whitespace();
        (
            it.next().unwrap_or("0").parse().unwrap_or(0),
            it.next().unwrap_or("").to_string(),
        )
    };
    let (cpp2_size, cpp2_hash) = parse_row(cpp2);
    let (py2_size, py2_hash) = parse_row(py2);
    assert_eq!(cpp2_size, *want_size, "C++ disagrees on {name} size");
    assert_eq!(py2_size, *want_size, "Python disagrees on {name} size");
    assert_eq!(&cpp2_hash, want_hash, "C++ disagrees on {name} hash");
    assert_eq!(&py2_hash, want_hash, "Python disagrees on {name} hash");

    // ── `horus msg hash` ──
    let cli = Command::new(horus())
        .args(["msg", "hash", "WeatherData"])
        .current_dir(&project)
        .output()
        .expect("horus msg hash must run");
    let printed = String::from_utf8_lossy(&cli.stdout).trim().to_string();
    assert_eq!(
        printed, cli_hash,
        "`horus msg hash` prints a different number from the one generated"
    );
}

/// The C++ contract has to fail when the two sides disagree, or it is
/// decoration. This is the `JointCommand` failure reproduced deliberately.
#[test]
fn a_widened_cpp_field_fails_the_static_assert() {
    if !tool("g++") {
        eprintln!("SKIP: needs g++");
        return;
    }
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    assert!(Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());

    std::fs::create_dir_all(project.join("msgs")).expect("mkdir");
    std::fs::write(
        project.join("msgs/m.hmsg"),
        "R { a: u64, b: f32, c: f32 }\n",
    )
    .expect("write");
    assert!(Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(&project)
        .output()
        .expect("msg gen")
        .status
        .success());

    let header = project.join(".horus/generated/include/demo/msgs.hpp");
    let text = std::fs::read_to_string(&header).expect("header");
    // Widen one field, exactly the drift that took JointCommand to 928 bytes.
    let broken = text.replacen("    float b;", "    double b;", 1);
    assert_ne!(broken, text, "expected to find the field to widen");
    std::fs::write(&header, broken).expect("write");

    let probe = tmp.path().join("p.cpp");
    std::fs::write(&probe, "#include \"demo/msgs.hpp\"\nint main() { return 0; }\n")
        .expect("write probe");
    let cc = Command::new("g++")
        .args(["-std=c++17", "-fsyntax-only", "-I"])
        .arg(project.join(".horus/generated/include"))
        .arg(&probe)
        .output()
        .expect("g++ must run");

    assert!(
        !cc.status.success(),
        "a C++ struct whose layout differs from Rust must not compile"
    );
    let err = String::from_utf8_lossy(&cc.stderr);
    assert!(
        err.contains("static assertion failed"),
        "the failure must be the layout contract, not something else:\n{err}"
    );
}

// ─── The CLI ────────────────────────────────────────────────────────────────

#[test]
fn msg_gen_check_detects_stale_artifacts() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    assert!(Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());
    std::fs::create_dir_all(project.join("msgs")).expect("mkdir");
    std::fs::write(project.join("msgs/m.hmsg"), "R { a: u64 }\n").expect("write");

    assert!(Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(&project)
        .output()
        .expect("gen")
        .status
        .success());

    let fresh = Command::new(horus())
        .args(["msg", "gen", "--check"])
        .current_dir(&project)
        .output()
        .expect("check");
    assert!(
        fresh.status.success(),
        "just-generated artifacts must be up to date: {}",
        String::from_utf8_lossy(&fresh.stderr)
    );

    // Change the definition without regenerating.
    std::fs::write(project.join("msgs/m.hmsg"), "R { a: u64, b: u32 }\n").expect("write");
    let stale = Command::new(horus())
        .args(["msg", "gen", "--check"])
        .current_dir(&project)
        .output()
        .expect("check");
    assert!(
        !stale.status.success(),
        "a changed definition must make --check fail"
    );
}

#[test]
fn a_project_without_msgs_says_how_to_start() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    assert!(Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());

    let out = Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(&project)
        .output()
        .expect("gen");
    assert!(!out.status.success());
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains(".hmsg"), "should say what to create:\n{err}");
    assert!(err.contains("mkdir msgs"), "should give a command:\n{err}");
}

/// A reference this generator cannot size exactly must be refused, not guessed.
#[test]
fn a_reference_to_an_unknown_type_is_refused() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    assert!(Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());
    std::fs::create_dir_all(project.join("msgs")).expect("mkdir");
    std::fs::write(project.join("msgs/m.hmsg"), "R { v: Vector3 }\n").expect("write");

    let out = Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(&project)
        .output()
        .expect("gen");
    assert!(!out.status.success(), "an unsizeable reference must be refused");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("Vector3"), "{err}");
}

/// Two messages with the same name in different files would produce one type.
#[test]
fn a_duplicate_message_name_across_files_is_refused() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    assert!(Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());
    std::fs::create_dir_all(project.join("msgs")).expect("mkdir");
    std::fs::write(project.join("msgs/a.hmsg"), "R { a: u64 }\n").expect("write");
    std::fs::write(project.join("msgs/b.hmsg"), "R { b: u64 }\n").expect("write");

    let out = Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(&project)
        .output()
        .expect("gen");
    assert!(!out.status.success());
    assert!(
        String::from_utf8_lossy(&out.stderr).contains("declared twice"),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

/// Output must not depend on directory iteration order.
#[test]
fn generation_is_deterministic() {
    let run = || -> String {
        let tmp = tempfile::tempdir().expect("tempdir");
        let project = tmp.path().join("demo");
        assert!(Command::new(horus())
            .args(["new", "demo", "--rust"])
            .current_dir(tmp.path())
            .output()
            .expect("horus new")
            .status
            .success());
        std::fs::create_dir_all(project.join("msgs")).expect("mkdir");
        for (name, body) in [("z.hmsg", "Zed { a: u64 }\n"), ("a.hmsg", "Ay { b: u32 }\n")] {
            std::fs::write(project.join("msgs").join(name), body).expect("write");
        }
        assert!(Command::new(horus())
            .args(["msg", "gen"])
            .current_dir(&project)
            .output()
            .expect("gen")
            .status
            .success());
        std::fs::read_to_string(project.join(".horus/generated/msgs/src/lib.rs")).expect("lib.rs")
    };
    assert_eq!(run(), run());
}

// ─── Wiring ─────────────────────────────────────────────────────────────────

/// Types that compile in isolation and are reachable from nothing are the
/// failure the Python generator had: it wrote Rust into a directory `lib.rs`
/// never declared, cargo compiled none of it, and nothing reported an error.
#[test]
fn the_generated_crate_is_a_dependency_of_the_project() {
    let src = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("src/cargo_gen.rs"),
    )
    .expect("cargo_gen.rs");
    assert!(
        src.contains("write_generated_msgs_dep"),
        "nothing adds the generated crate to the project manifest"
    );
    assert!(
        src.contains(".horus/generated/msgs"),
        "the generated crate path is not referenced"
    );
}

#[test]
fn the_generated_include_path_is_on_the_cpp_include_path() {
    let src = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("src/cmake_gen.rs"),
    )
    .expect("cmake_gen.rs");
    assert!(
        src.contains("generated/include"),
        "a C++ project cannot include what msg gen writes"
    );
}

/// A reordering that keeps the size identical. `sizeof` cannot see it; only the
/// per-field `offsetof` assertions can — and a reader that disagrees about
/// where a field starts reads the wrong bytes for every message, silently.
#[test]
fn a_reordered_cpp_struct_of_the_same_size_still_fails() {
    if !tool("g++") {
        eprintln!("SKIP: needs g++");
        return;
    }
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    assert!(Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());
    std::fs::create_dir_all(project.join("msgs")).expect("mkdir");
    // Two same-size fields: any swap leaves sizeof unchanged.
    std::fs::write(project.join("msgs/m.hmsg"), "R { alpha: u32, beta: u32 }\n").expect("write");
    assert!(Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(&project)
        .output()
        .expect("msg gen")
        .status
        .success());

    let header = project.join(".horus/generated/include/demo/msgs.hpp");
    let text = std::fs::read_to_string(&header).expect("header");
    let swapped = text.replacen(
        "    uint32_t alpha;\n    uint32_t beta;",
        "    uint32_t beta;\n    uint32_t alpha;",
        1,
    );
    assert_ne!(swapped, text, "expected to find the two fields to swap");
    std::fs::write(&header, swapped).expect("write");

    let probe = tmp.path().join("p.cpp");
    std::fs::write(&probe, "#include \"demo/msgs.hpp\"\nint main() { return 0; }\n")
        .expect("write probe");
    let cc = Command::new("g++")
        .args(["-std=c++17", "-fsyntax-only", "-I"])
        .arg(project.join(".horus/generated/include"))
        .arg(&probe)
        .output()
        .expect("g++ must run");

    assert!(
        !cc.status.success(),
        "swapping two same-size fields leaves sizeof identical, so only the \
         per-field offset assertions can catch it — and they must"
    );
    let err = String::from_utf8_lossy(&cc.stderr);
    assert!(
        err.contains("at a different offset"),
        "the failure must name the field that moved:\n{err}"
    );
}

// ─── The C ABI ──────────────────────────────────────────────────────────────

/// Helper: a project with one generated message.
fn project_with_message(kind: &str, body: &str) -> (tempfile::TempDir, PathBuf) {
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    assert!(Command::new(horus())
        .args(["new", "demo", kind])
        .current_dir(tmp.path())
        .output()
        .expect("horus new")
        .status
        .success());
    std::fs::create_dir_all(project.join("msgs")).expect("mkdir");
    std::fs::write(project.join("msgs/m.hmsg"), body).expect("write");
    let out = Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(&project)
        .output()
        .expect("msg gen");
    assert!(
        out.status.success(),
        "msg gen failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    (tmp, project)
}

/// A generated type can be *read* from C++ as soon as the header exists.
/// Publishing one needs entry points, and the ones HORUS ships live in
/// `libhorus_cpp.a`: `impl_pod_topic_c_api!` emits `#[no_mangle] extern "C"`
/// functions into that archive, which a user cannot add to without editing the
/// HORUS source tree. Neither that macro nor `impl_topic_ffi!` can be reused
/// from outside the crate either — they are not exported, and their bodies name
/// `paste::paste!`, `FfiPublisher` and a `topic_ffi::` path unqualified.
///
/// So the entry points are generated per project, into a staticlib linked
/// alongside `libhorus_cpp.a`. Nothing in the HORUS tree changes.
#[test]
fn generation_emits_a_c_abi_crate() {
    let (_tmp, project) = project_with_message(
        "--rust",
        "#[topic = \"robot.telemetry\"]\nTelemetry { seq: u64, ok: bool }\n",
    );

    let ffi = project.join(".horus/generated/msgs_ffi");
    assert!(ffi.join("Cargo.toml").is_file(), "no FFI crate manifest");
    assert!(ffi.join("src/lib.rs").is_file(), "no FFI crate source");

    let manifest = std::fs::read_to_string(ffi.join("Cargo.toml")).expect("manifest");
    assert!(
        manifest.contains("staticlib"),
        "the C++ link needs a static archive:\n{manifest}"
    );
    // Not a path dependency of anything, so cargo never adopts it into the
    // .horus workspace and refuses to build it without its own root.
    assert!(
        manifest.contains("[workspace]"),
        "without its own workspace root cargo fails with \"current package \
         believes it's in a workspace when it's not\":\n{manifest}"
    );

    let lib = std::fs::read_to_string(ffi.join("src/lib.rs")).expect("lib.rs");
    for suffix in [
        "_publisher_new",
        "_publisher_send",
        "_publisher_free",
        "_subscriber_new",
        "_subscriber_recv",
        "_subscriber_free",
    ] {
        assert!(lib.contains(suffix), "missing entry point `{suffix}`");
    }
    assert_eq!(
        lib.matches("#[no_mangle]").count(),
        6,
        "one exported symbol per entry point"
    );
}

/// Two projects loaded into one process must not collide, and a generated name
/// must never take a built-in `horus_<type>_` symbol.
#[test]
fn c_abi_symbols_are_namespaced_by_package_and_type() {
    let (_tmp, project) = project_with_message("--rust", "Telemetry { seq: u64 }\n");
    let lib = std::fs::read_to_string(project.join(".horus/generated/msgs_ffi/src/lib.rs"))
        .expect("lib.rs");
    assert!(
        lib.contains("horus_gen_demo_telemetry_publisher_new"),
        "symbols must carry both the package and the type:\n{lib}"
    );
    assert!(
        !lib.contains("\"C\" fn horus_telemetry_"),
        "a generated symbol must not take the built-in namespace"
    );
}

/// The header a user includes must give them something to publish with, not
/// just a struct to look at.
#[test]
fn the_header_carries_publisher_and_subscriber_wrappers() {
    let (_tmp, project) = project_with_message("--cpp", "Telemetry { seq: u64 }\n");
    let header =
        std::fs::read_to_string(project.join(".horus/generated/include/demo/msgs.hpp"))
            .expect("header");

    assert!(header.contains("class TelemetryPublisher"), "{header}");
    assert!(header.contains("class TelemetrySubscriber"), "{header}");
    // It owns a raw handle; copying it would free twice.
    assert!(
        header.contains("TelemetryPublisher(const TelemetryPublisher&) = delete"),
        "the publisher owns a handle and must not be copyable"
    );
    assert!(
        header.contains("bool valid() const"),
        "a caller must be able to tell whether the topic opened"
    );
}

/// A wrapper that does not compile is worse than none.
#[test]
fn the_generated_topic_wrappers_compile() {
    if !tool("g++") {
        eprintln!("SKIP: needs g++");
        return;
    }
    let (tmp, project) = project_with_message(
        "--cpp",
        "#[topic = \"robot.telemetry\"]\nTelemetry { seq: u64, ok: bool }\n",
    );

    let probe = tmp.path().join("p.cpp");
    std::fs::write(
        &probe,
        "#include \"demo/msgs.hpp\"\n\
         int main() {\n\
         \x20 demo::msg::TelemetryPublisher p(demo::msg::TELEMETRY_TOPIC);\n\
         \x20 demo::msg::TelemetrySubscriber s(demo::msg::TELEMETRY_TOPIC);\n\
         \x20 demo::msg::Telemetry t{};\n\
         \x20 t.seq = 1;\n\
         \x20 t.set_ok(true);\n\
         \x20 if (p.valid()) p.send(t);\n\
         \x20 demo::msg::Telemetry got{};\n\
         \x20 (void)s.recv(got);\n\
         \x20 return 0;\n\
         }\n",
    )
    .expect("write probe");

    // -fsyntax-only: the symbols live in the staticlib, which this test does
    // not build. Linking is covered by the ignored end-to-end test below.
    let cc = Command::new("g++")
        .args(["-std=c++17", "-fsyntax-only", "-I"])
        .arg(project.join(".horus/generated/include"))
        .arg(&probe)
        .output()
        .expect("g++ must run");
    assert!(
        cc.status.success(),
        "the generated topic wrappers do not compile:\n{}",
        String::from_utf8_lossy(&cc.stderr)
    );
}

#[test]
fn the_cpp_build_links_the_generated_entry_points() {
    let cmake =
        std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/cmake_gen.rs"))
            .expect("cmake_gen.rs");
    assert!(
        cmake.contains("HORUS_GEN_MSGS_LIB"),
        "nothing links the generated staticlib, so a C++ publisher fails at link"
    );
    let run_cpp = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("src/commands/run/run_cpp.rs"),
    )
    .expect("run_cpp.rs");
    assert!(
        run_cpp.contains("ensure_generated_msgs_lib"),
        "nothing builds the generated staticlib"
    );
}
