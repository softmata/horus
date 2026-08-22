//! A `.hmsg` must be usable from every language HORUS ships, and the topic it
//! opens must be layout-checked in all of them.
//!
//! Two defects, one generator.
//!
//! **Python could not publish.** `horus msg gen` emitted a `ctypes.Structure`
//! and nothing else, so a generated type was readable from Python and
//! unpublishable: the C++ header got `<Type>Publisher`/`<Type>Subscriber`
//! classes over the generated C ABI, and the Python module got a layout mirror.
//! A message definition usable from two of three languages is not a message
//! definition.
//!
//! **The generated C ABI opened topics unchecked.** `Topic::new` validates the
//! message type's short name and, for POD types, its size. Neither describes
//! field layout, so two builds of `Pose { x, y }` and `Pose { y, x }` share a
//! name and eight bytes, open the same topic without complaint, and swap the
//! coordinates on the way through. `LAYOUT_HASH` and `Topic::new_checked`
//! existed; the generated entry points — the one place C++ and Python actually
//! reach shared memory — called `Topic::<T>::new` and passed no hash, which
//! leaves the check inert.
//!
//! The expensive tests share one built crate. A debug build of the generated
//! FFI crate pulls in `horus_core` and takes about 45 seconds; per test it
//! would put minutes on the suite for one artifact.
//!
//! Run: `cargo test -p horus_manager --test msg_gen_interop`

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;

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

/// A suffix unique to this process, so two runs of the suite do not meet each
/// other on a topic left in `/dev/shm`.
fn run_id() -> &'static str {
    static ID: OnceLock<String> = OnceLock::new();
    ID.get_or_init(|| {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.subsec_nanos())
            .unwrap_or(0);
        format!("{}x{}", std::process::id(), nanos)
    })
}

/// The message set, with `Pose`'s two fields in the given order.
///
/// `Pose` is two `f32`s on purpose. Reordering them keeps the type's name and
/// its size, so the only thing that distinguishes the two revisions is the
/// field layout — exactly what the topic open cannot see without a hash.
fn definitions(pose_fields: &str) -> String {
    format!(
        "/// A weather reading.\n\
         #[topic = \"interop.telemetry.{id}\"]\n\
         Telemetry {{\n\
         \x20   timestamp_ns: u64,\n\
         \x20   temperature: f32,\n\
         \x20   humidity: f32,\n\
         \x20   history: [f32; 4],\n\
         \x20   heater_on: bool,\n\
         }}\n\
         \n\
         /// Same name and same size in both revisions below.\n\
         Pose {{\n\
         {pose_fields}\
         }}\n",
        id = run_id()
    )
}

const POSE_XY: &str = "    x: f32,\n    y: f32,\n";
const POSE_YX: &str = "    y: f32,\n    x: f32,\n";

/// Generate a project without building anything. Cheap: no cargo.
fn generated_project(defs: &str) -> (tempfile::TempDir, PathBuf) {
    let tmp = tempfile::tempdir().expect("tempdir");
    let project = tmp.path().join("demo");
    let out = Command::new(horus())
        .args(["new", "demo", "--python"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(
        out.status.success(),
        "horus new failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    std::fs::create_dir_all(project.join("msgs")).expect("mkdir msgs");
    std::fs::write(project.join("msgs/m.hmsg"), defs).expect("write hmsg");
    regenerate(&project);
    (tmp, project)
}

fn regenerate(project: &Path) {
    let out = Command::new(horus())
        .args(["msg", "gen"])
        .current_dir(project)
        .output()
        .expect("horus msg gen must run");
    assert!(
        out.status.success(),
        "msg gen failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
}

/// Build the generated FFI crate the way `run_cpp.rs` does, and return the
/// cdylib.
///
/// The cargo environment of the *test* is stripped: a jobserver descriptor and
/// a manifest directory belonging to `horus_manager` are not ours to hand to a
/// different crate's build.
fn build_ffi(project: &Path) -> PathBuf {
    let crate_dir = project.join(".horus/generated/msgs_ffi");
    let target = crate_dir.join("target");
    let status = Command::new("cargo")
        .arg("build")
        .current_dir(&crate_dir)
        .env("CARGO_TARGET_DIR", &target)
        .env_remove("CARGO_MAKEFLAGS")
        .env_remove("CARGO_MANIFEST_DIR")
        .env_remove("CARGO_PRIMARY_PACKAGE")
        .status()
        .expect("cargo must run");
    assert!(
        status.success(),
        "cargo build failed in {}",
        crate_dir.display()
    );

    let so = target.join("debug").join(cdylib_name());
    assert!(
        so.is_file(),
        "the generated crate produced no loadable library at {} — `ctypes` \
         cannot open a static archive, so Python would have types it cannot \
         publish",
        so.display()
    );
    so
}

fn cdylib_name() -> String {
    if cfg!(target_os = "macos") {
        "libdemo_msgs_ffi.dylib".to_string()
    } else if cfg!(windows) {
        "demo_msgs_ffi.dll".to_string()
    } else {
        "libdemo_msgs_ffi.so".to_string()
    }
}

/// One project, built three times into one target directory.
///
/// `v1` and `v2` are the same type with its two fields in the two orders. The
/// project itself is left holding `v1`, so the round-trip test and the
/// mismatch test can share this fixture without either mutating it.
struct Built {
    _tmp: tempfile::TempDir,
    project: PathBuf,
    v1: PathBuf,
    v2: PathBuf,
}

fn built() -> Option<&'static Built> {
    static FIXTURE: OnceLock<Option<Built>> = OnceLock::new();
    FIXTURE
        .get_or_init(|| {
            if !tool("python3") || !tool("cargo") {
                eprintln!("SKIP: needs python3 and cargo");
                return None;
            }
            let (tmp, project) = generated_project(&definitions(POSE_XY));

            // First build is the slow one; the two after it recompile only the
            // two generated crates.
            let so = build_ffi(&project);
            let v1 = tmp
                .path()
                .join("v1")
                .with_extension(so.extension().unwrap_or_default());
            std::fs::copy(&so, &v1).expect("copy v1");

            std::fs::write(project.join("msgs/m.hmsg"), definitions(POSE_YX)).expect("write");
            regenerate(&project);
            let so = build_ffi(&project);
            let v2 = tmp
                .path()
                .join("v2")
                .with_extension(so.extension().unwrap_or_default());
            std::fs::copy(&so, &v2).expect("copy v2");

            // Leave the project consistent with v1, so the round-trip test sees
            // a `msgs.py` and a library that agree.
            std::fs::write(project.join("msgs/m.hmsg"), definitions(POSE_XY)).expect("write");
            regenerate(&project);
            build_ffi(&project);

            Some(Built {
                _tmp: tmp,
                project,
                v1,
                v2,
            })
        })
        .as_ref()
}

fn python(project: &Path, script: &str) -> std::process::Output {
    Command::new("python3")
        .arg("-c")
        .arg(script)
        .current_dir(project)
        .output()
        .expect("python3 must run")
}

// ─── INTEROP-1: Python is a first-class consumer of a `.hmsg` ────────────────

/// The decisive one: a Python process publishes a generated type and another
/// Python process reads it back with its fields intact.
///
/// Before this, `horus msg gen` gave Python a `ctypes.Structure` and no way to
/// reach a topic — `grep -c 'ublisher'` over the emitter returned 0 — so a
/// message definition was usable from Rust and C++ and not from Python.
#[test]
fn a_python_node_publishes_and_receives_a_generated_type() {
    let Some(f) = built() else { return };

    let script = format!(
        "import sys, time\n\
         sys.path.insert(0, {path:?})\n\
         import msgs\n\
         tx = msgs.TelemetryPublisher()\n\
         rx = msgs.TelemetrySubscriber()\n\
         m = msgs.Telemetry(timestamp_ns=12345678901234, temperature=21.5, humidity=0.25, heater_on=1)\n\
         m.history[0] = 1.5\n\
         m.history[3] = -2.25\n\
         tx.send(m)\n\
         got = None\n\
         for _ in range(200):\n\
         \x20   got = rx.recv()\n\
         \x20   if got is not None:\n\
         \x20       break\n\
         \x20   time.sleep(0.01)\n\
         assert got is not None, 'nothing arrived on the topic'\n\
         print(got.timestamp_ns, got.temperature, got.humidity, got.heater_on, got.history[0], got.history[3])\n\
         print(tx.topic)\n\
         tx.close(); rx.close()\n",
        path = f
            .project
            .join(".horus/generated/python")
            .display()
            .to_string()
    );

    let out = python(&f.project, &script);
    assert!(
        out.status.success(),
        "a Python node could not publish a generated type:\n{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let mut lines = stdout.lines();
    let values = lines.next().unwrap_or_default().trim();
    assert_eq!(
        values, "12345678901234 21.5 0.25 1 1.5 -2.25",
        "the message arrived with different values than it was sent with — \
         the Python struct and the Rust struct disagree about layout:\n{stdout}"
    );
    assert_eq!(
        lines.next().unwrap_or_default().trim(),
        format!("interop.telemetry.{}", run_id()),
        "`#[topic = \"...\"]` should be the publisher's default topic"
    );
}

/// The module must be importable and usable without a built library — reading
/// and constructing a message needs no FFI at all — and the classes must be
/// real classes with the documented shape, not text that happens to match.
#[test]
fn the_generated_module_exposes_a_topic_api_before_anything_is_built() {
    if !tool("python3") {
        eprintln!("SKIP: needs python3");
        return;
    }
    let (_tmp, project) = generated_project(&definitions(POSE_XY));

    let script = format!(
        "import sys, inspect\n\
         sys.path.insert(0, {path:?})\n\
         import msgs\n\
         assert inspect.isclass(msgs.TelemetryPublisher)\n\
         assert inspect.isclass(msgs.TelemetrySubscriber)\n\
         assert callable(msgs.TelemetryPublisher.send)\n\
         assert callable(msgs.TelemetrySubscriber.recv)\n\
         assert callable(msgs.TelemetryPublisher.close)\n\
         assert '__enter__' in dir(msgs.TelemetryPublisher)\n\
         # `#[topic = \"...\"]` becomes the default argument.\n\
         d = inspect.signature(msgs.TelemetryPublisher.__init__).parameters['topic'].default\n\
         print(d)\n\
         # `Pose` has no #[topic], so its topic is required rather than defaulted.\n\
         p = inspect.signature(msgs.PosePublisher.__init__).parameters['topic']\n\
         print(p.default is inspect.Parameter.empty)\n\
         # Every endpoint class is exported.\n\
         print(sorted(n for n in msgs.__all__ if n.endswith(('Publisher', 'Subscriber'))))\n\
         # Constructing a message still needs no library.\n\
         print(msgs.Telemetry(temperature=1.0).temperature)\n",
        path = project
            .join(".horus/generated/python")
            .display()
            .to_string()
    );

    let out = python(&project, &script);
    assert!(
        out.status.success(),
        "the generated Python module has no topic API:\n{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let lines: Vec<&str> = stdout.lines().map(str::trim).collect();
    assert_eq!(lines[0], format!("interop.telemetry.{}", run_id()));
    assert_eq!(lines[1], "True", "a type without #[topic] must require one");
    assert_eq!(
        lines[2],
        "['PosePublisher', 'PoseSubscriber', 'TelemetryPublisher', 'TelemetrySubscriber']",
        "every generated type needs both endpoints exported"
    );
    assert_eq!(lines[3], "1.0");
}

/// `ctypes.CDLL` cannot open a `.a`. Without a cdylib the emitted Python
/// classes would be decoration.
#[test]
fn the_ffi_crate_is_built_as_a_loadable_library() {
    let (_tmp, project) = generated_project(&definitions(POSE_XY));
    let manifest = std::fs::read_to_string(project.join(".horus/generated/msgs_ffi/Cargo.toml"))
        .expect("manifest");
    let types = manifest
        .lines()
        .find(|l| l.trim_start().starts_with("crate-type"))
        .expect("no crate-type in the generated manifest");
    for kind in ["staticlib", "rlib", "cdylib"] {
        assert!(
            types.contains(kind),
            "`{kind}` is missing from `{types}` — staticlib is the C++ link, \
             rlib is `cargo check`, cdylib is the only one `ctypes` can open"
        );
    }
}

/// A missing library must say what to build, not `OSError: cannot open shared
/// object file`.
#[test]
fn an_unbuilt_library_explains_itself() {
    if !tool("python3") {
        eprintln!("SKIP: needs python3");
        return;
    }
    let (_tmp, project) = generated_project(&definitions(POSE_XY));

    let script = format!(
        "import sys\n\
         sys.path.insert(0, {path:?})\n\
         import msgs\n\
         try:\n\
         \x20   msgs.TelemetryPublisher()\n\
         except msgs.MessageLibraryError as e:\n\
         \x20   print(str(e).replace(chr(10), ' | '))\n\
         else:\n\
         \x20   raise SystemExit('expected MessageLibraryError')\n",
        path = project
            .join(".horus/generated/python")
            .display()
            .to_string()
    );

    let out = Command::new("python3")
        .arg("-c")
        .arg(&script)
        .current_dir(&project)
        // Nothing is built here, and the module must not go build it either:
        // this test is about the message, not the build.
        .env("HORUS_MSGS_FFI_BUILD", "0")
        .output()
        .expect("python3 must run");
    assert!(
        out.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let msg = String::from_utf8_lossy(&out.stdout);
    assert!(
        msg.contains("cargo build --manifest-path"),
        "the error must name the command that fixes it:\n{msg}"
    );
    assert!(
        msg.contains("msgs_ffi"),
        "the error must name the crate to build:\n{msg}"
    );
}

// ─── INTEROP-2: the generated FFI path is layout-checked ─────────────────────

/// The decisive one: two builds of `Pose` that differ only in field order must
/// not both open the same topic.
///
/// Both cdylibs are loaded into one process, which works because `CDLL` uses
/// `RTLD_LOCAL` — each handle resolves its own copy of the entry points. The
/// topic lives in shared memory, so the first open records its layout hash and
/// the second is checked against it.
///
/// With the entry points calling `Topic::<T>::new` this test fails with two
/// non-null handles: same short name, same eight bytes, no complaint, and every
/// `Pose` that crosses the topic arrives with `x` and `y` exchanged.
#[test]
fn a_reordered_build_cannot_open_the_same_topic() {
    let Some(f) = built() else { return };

    let script = format!(
        "import ctypes\n\
         topic = b'interop.pose.{id}'\n\
         def opener(path):\n\
         \x20   lib = ctypes.CDLL(path)\n\
         \x20   fn = lib.horus_gen_demo_pose_publisher_new\n\
         \x20   fn.argtypes = [ctypes.c_char_p]\n\
         \x20   fn.restype = ctypes.c_void_p\n\
         \x20   return fn\n\
         first = opener({v1:?})(topic)\n\
         second = opener({v2:?})(topic)\n\
         print('first', first is not None and first != 0)\n\
         print('second', second is not None and second != 0)\n",
        id = run_id(),
        v1 = f.v1.display().to_string(),
        v2 = f.v2.display().to_string(),
    );

    let out = python(&f.project, &script);
    assert!(
        out.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("first True"),
        "the first build must open the topic normally:\n{stdout}"
    );
    assert!(
        stdout.contains("second False"),
        "a build of `Pose` with its two f32 fields in the other order opened \
         the same topic. The name and the size match, so nothing else can \
         catch this: every message that crosses the topic arrives with x and y \
         exchanged.\n{stdout}"
    );

    // The null is the mechanism; the message is what makes it fixable.
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("layout mismatch"),
        "the C ABI can only return null, so the reason has to reach stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("horus_gen") || stderr.contains("interop.pose"),
        "the diagnostic must name the topic that failed:\n{stderr}"
    );
}

/// The Python endpoint classes are the same door, so they must report the same
/// refusal — as an exception, not a null a caller can ignore.
#[test]
fn python_raises_when_the_layout_disagrees() {
    let Some(f) = built() else { return };

    // The project's own module and library are the `x, y` build; `v2` is the
    // `y, x` one. Pointing the module at v2 with HORUS_MSGS_FFI reproduces a
    // half-updated deployment inside one process.
    let script = format!(
        "import ctypes, sys, os\n\
         topic = 'interop.pose.py.{id}'\n\
         # A peer from the first build takes the topic and holds it.\n\
         first = ctypes.CDLL({v1:?})\n\
         fn = first.horus_gen_demo_pose_publisher_new\n\
         fn.argtypes = [ctypes.c_char_p]\n\
         fn.restype = ctypes.c_void_p\n\
         assert fn(topic.encode()), 'the first build should open the topic'\n\
         # Now the generated Python module, against the second build.\n\
         os.environ['HORUS_MSGS_FFI'] = {v2:?}\n\
         sys.path.insert(0, {path:?})\n\
         import msgs\n\
         try:\n\
         \x20   msgs.PosePublisher(topic)\n\
         except msgs.TopicOpenError as e:\n\
         \x20   print('raised')\n\
         \x20   print(str(e).replace(chr(10), ' | '))\n\
         else:\n\
         \x20   raise SystemExit('PosePublisher opened a topic whose layout it disagrees with')\n",
        id = run_id(),
        v1 = f.v1.display().to_string(),
        v2 = f.v2.display().to_string(),
        path = f
            .project
            .join(".horus/generated/python")
            .display()
            .to_string(),
    );

    let out = python(&f.project, &script);
    assert!(
        out.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("raised"), "{stdout}");
    assert!(
        stdout.contains("horus msg gen"),
        "the exception must say how to get back in sync:\n{stdout}"
    );
}

/// The generated Rust type carries the same checked opener `message!` gives
/// hand-written ones, and the C ABI goes through it rather than around it.
#[test]
fn the_generated_entry_points_open_through_the_checked_constructor() {
    let (_tmp, project) = generated_project(&definitions(POSE_XY));

    let rust = std::fs::read_to_string(project.join(".horus/generated/msgs/src/lib.rs"))
        .expect("generated lib.rs");
    assert!(
        rust.contains("Topic::new_checked(name, Self::LAYOUT_HASH)"),
        "a generated type has a LAYOUT_HASH and no way to use it:\n{rust}"
    );

    let ffi = std::fs::read_to_string(project.join(".horus/generated/msgs_ffi/src/lib.rs"))
        .expect("generated FFI lib.rs");
    for ty in ["Telemetry", "Pose"] {
        assert!(
            ffi.contains(&format!("<demo_msgs::{ty}>::topic(name)")),
            "the C ABI must open `{ty}` through the checked helper:\n{ffi}"
        );
    }
    // The unchecked constructor is the defect, so its absence is the contract.
    assert!(
        !ffi.contains("::new(name)"),
        "an entry point still opens a topic with the unchecked constructor, \
         which passes no layout hash and leaves the check inert:\n{ffi}"
    );
    // Two types, two endpoints each.
    assert_eq!(
        ffi.matches("::topic(name)").count(),
        4,
        "every publisher and subscriber entry point must be checked, not just \
         the first one emitted:\n{ffi}"
    );
}

/// C++ reaches a topic through the same entry points, so it must get the same
/// refusal — and this is the language the finding was about: a hand-written
/// header mirroring a Rust struct is exactly where `horus_cpp`'s layout
/// contract records `JointCommand` at 928 bytes against 88.
///
/// The probe links the project's static archive (the `x, y` build). A peer
/// holding the topic from the `y, x` build is loaded first, from Python,
/// because the archive can only be linked once per binary.
#[test]
fn a_cpp_publisher_is_refused_when_a_peer_disagrees_about_layout() {
    if !cfg!(target_os = "linux") {
        eprintln!("SKIP: the link flags below are Linux's");
        return;
    }
    if !tool("g++") {
        eprintln!("SKIP: needs g++");
        return;
    }
    let Some(f) = built() else { return };

    let probe_src = f.project.join("cpp_layout_probe.cpp");
    std::fs::write(
        &probe_src,
        "#include \"demo/msgs.hpp\"\n\
         #include <cstdio>\n\
         int main(int argc, char** argv) {\n\
         \x20 if (argc < 2) return 2;\n\
         \x20 demo::msg::PosePublisher p(argv[1]);\n\
         \x20 printf(\"valid=%d\\n\", (int)p.valid());\n\
         \x20 return 0;\n\
         }\n",
    )
    .expect("write probe");

    let probe_bin = f.project.join("cpp_layout_probe");
    let cc = Command::new("g++")
        .args(["-std=c++17", "-I"])
        .arg(f.project.join(".horus/generated/include"))
        .arg(&probe_src)
        .arg(
            f.project
                .join(".horus/generated/msgs_ffi/target/debug/libdemo_msgs_ffi.a"),
        )
        .args(["-lpthread", "-ldl", "-lm", "-o"])
        .arg(&probe_bin)
        .output()
        .expect("g++ must run");
    assert!(
        cc.status.success(),
        "the generated C++ wrappers do not link against the generated archive:\n{}",
        String::from_utf8_lossy(&cc.stderr)
    );

    // Control: with nobody else on the topic, the same binary opens it. Without
    // this the assertion below would also pass if the probe were simply broken.
    let alone = Command::new(&probe_bin)
        .arg(format!("interop.cpp.alone.{}", run_id()))
        .output()
        .expect("probe must run");
    assert!(
        String::from_utf8_lossy(&alone.stdout).contains("valid=1"),
        "the C++ publisher cannot open an uncontested topic, so the test below \
         would prove nothing:\n{}\n{}",
        String::from_utf8_lossy(&alone.stdout),
        String::from_utf8_lossy(&alone.stderr)
    );

    // Now with a peer from the other build holding it.
    let script = format!(
        "import ctypes, subprocess\n\
         topic = 'interop.cpp.contested.{id}'\n\
         peer = ctypes.CDLL({v2:?})\n\
         fn = peer.horus_gen_demo_pose_publisher_new\n\
         fn.argtypes = [ctypes.c_char_p]\n\
         fn.restype = ctypes.c_void_p\n\
         assert fn(topic.encode()), 'the peer should open the topic first'\n\
         done = subprocess.run([{probe:?}, topic], capture_output=True, text=True)\n\
         print(done.stdout.strip())\n\
         print(done.stderr.strip().replace(chr(10), ' | '))\n",
        id = run_id(),
        v2 = f.v2.display().to_string(),
        probe = probe_bin.display().to_string(),
    );
    let out = python(&f.project, &script);
    assert!(
        out.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("valid=0"),
        "a C++ publisher opened a topic already held by a build of `Pose` with \
         its fields in the other order. `valid()` said yes and every message \
         sent through it arrives with x and y exchanged:\n{stdout}"
    );
    assert!(
        stdout.contains("layout mismatch"),
        "`valid() == false` is not a diagnosis; the reason has to reach the \
         C++ program's stderr:\n{stdout}"
    );
}
