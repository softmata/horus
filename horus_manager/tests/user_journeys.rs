#![allow(dead_code)]
//! User-journey E2E tests.
//!
//! Each test is a sequence a developer actually runs — create, add, build,
//! test, eject, run, stop — asserted on the OUTCOME the user cares about,
//! not on exit codes or scaffold shapes.
//!
//! WHY THIS FILE EXISTS. The suites that came before check that commands
//! succeed and that files were created. They do not check that the thing the
//! user asked for happened: a test file actually ran, an added dependency
//! actually resolved, an ejected project actually builds with plain cargo.
//! That gap is where the silent "0 tests, success" bug lived — `horus test`
//! reported success while ignoring every file in `tests/` — and where the
//! workspace holes lived, because `[[test]]` discovery was asserted nowhere
//! and a failed build was allowed to be a passing test.
//!
//! Every test is `#[ignore]`d because each one compiles a real project. They
//! run in the `User Journey E2E` CI job:
//!
//! ```text
//! cargo test -p horus_manager --test user_journeys -- --ignored --test-threads=1
//! ```
//!
//! `--test-threads=1` is deliberate: the journeys share `CARGO_TARGET_DIR`
//! (one compiled horus/horus_core tree instead of one per project), and the
//! scheduler takes a process-wide signal handler.

use assert_cmd::cargo::cargo_bin;
use assert_cmd::Command;
use std::fs;
use std::path::PathBuf;
use std::time::Duration;
use tempfile::TempDir;

/// A cold build of the horus dependency tree happens at most once per job;
/// everything after that is the project crate itself. Generous, because a
/// shared runner can be slow.
const BUILD_TIMEOUT: Duration = Duration::from_secs(20 * 60);
/// Commands that do not compile anything.
const QUICK_TIMEOUT: Duration = Duration::from_secs(60);

fn horus_bin() -> PathBuf {
    cargo_bin("horus")
}

fn horus() -> Command {
    Command::new(horus_bin())
}

fn cargo() -> Command {
    Command::new("cargo")
}

fn combined(output: &std::process::Output) -> String {
    format!(
        "stdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    )
}

/// A project scaffolded by `horus new` in a temp directory.
///
/// Dropping it removes the project, its `.horus/target` and everything else
/// the journey created.
struct Project {
    dir: TempDir,
    name: String,
}

impl Project {
    /// `horus new <name> --rust --yes` plus any extra flags.
    fn new(name: &str, extra: &[&str]) -> Project {
        let dir = TempDir::new().expect("temp dir");
        let output = horus()
            .args(["new", name, "--rust", "--yes", "-o"])
            .arg(dir.path())
            .args(extra)
            .timeout(BUILD_TIMEOUT)
            .output()
            .expect("run horus new");
        assert!(
            output.status.success(),
            "horus new failed:\n{}",
            combined(&output)
        );
        Project {
            dir,
            name: name.to_string(),
        }
    }

    fn path(&self) -> PathBuf {
        self.dir.path().join(&self.name)
    }

    fn write(&self, relative: &str, content: &str) {
        let path = self.path().join(relative);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).expect("create parent dir");
        }
        fs::write(&path, content).expect("write file");
    }

    fn run(&self, args: &[&str]) -> std::process::Output {
        horus()
            .args(args)
            .current_dir(self.path())
            .timeout(BUILD_TIMEOUT)
            .output()
            .expect("run horus")
    }

    fn run_cargo(&self, args: &[&str]) -> std::process::Output {
        cargo()
            .args(args)
            .current_dir(self.path())
            .timeout(BUILD_TIMEOUT)
            .output()
            .expect("run cargo")
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Journey 1: the tests I write actually run
// ═══════════════════════════════════════════════════════════════════════════

/// `horus new` → write `tests/smoke.rs` → `horus test` runs it → delete it →
/// the next run does not fail on the stale target.
///
/// This is the journey the silent-zero-tests bug broke: `tests/` was never
/// declared as a cargo target, so `horus test` compiled nothing and reported
/// success. Asserting the test's NAME appears is the difference between
/// "the command exited 0" and "my test ran".
#[test]
#[ignore = "compiles a real project; run by the User Journey E2E job"]
fn journey_written_test_is_discovered_and_run() {
    let project = Project::new("journey-tests", &[]);
    project.write(
        "tests/journey_smoke.rs",
        "#[test]\nfn journey_smoke_marker() { assert_eq!(2 + 2, 4); }\n",
    );

    let output = project.run(&["test"]);
    let text = combined(&output);
    assert!(output.status.success(), "horus test failed:\n{text}");
    assert!(
        text.contains("journey_smoke_marker"),
        "the test in tests/ was not discovered or run:\n{text}"
    );

    // Deleting a test must not leave a target table pointing at a file that is
    // gone: the manifest has to be regenerated, or the next run fails on it.
    fs::remove_file(project.path().join("tests/journey_smoke.rs")).expect("remove test file");
    let output = project.run(&["test"]);
    let text = combined(&output);
    assert!(
        output.status.success(),
        "horus test failed after the test file was deleted:\n{text}"
    );
    assert!(
        !text.contains("journey_smoke_marker"),
        "a deleted test was still compiled:\n{text}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Journey 2: adding a dependency and building with it
// ═══════════════════════════════════════════════════════════════════════════

/// `horus add` → `horus build`. The build is the assertion: cargo compiles
/// every declared dependency, so a build that succeeds proves the added crate
/// resolves — which the manifest-text check alone does not.
#[test]
#[ignore = "compiles a real project; run by the User Journey E2E job"]
fn journey_added_dependency_is_resolved_by_a_build() {
    let project = Project::new("journey-add", &[]);

    let output = project.run(&["add", "cfg-if"]);
    assert!(
        output.status.success(),
        "horus add failed:\n{}",
        combined(&output)
    );
    let manifest = fs::read_to_string(project.path().join("horus.toml")).expect("read horus.toml");
    assert!(
        manifest.contains("cfg-if"),
        "horus.toml does not list the added dependency:\n{manifest}"
    );

    let output = project.run(&["build"]);
    assert!(
        output.status.success(),
        "horus build after horus add failed (the dependency did not resolve):\n{}",
        combined(&output)
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Journey 3: eject, then plain cargo
// ═══════════════════════════════════════════════════════════════════════════

/// Managed project → `horus eject` → `cargo test` and `cargo build` work, and
/// `horus test` keeps working from the manifest the user now owns.
#[test]
#[ignore = "compiles a real project; run by the User Journey E2E job"]
fn journey_eject_hands_the_build_to_plain_cargo() {
    let project = Project::new("journey-eject", &[]);
    project.write(
        "tests/journey_smoke.rs",
        "#[test]\nfn eject_smoke_marker() {}\n",
    );

    let output = project.run(&["eject"]);
    assert!(
        output.status.success(),
        "horus eject failed:\n{}",
        combined(&output)
    );
    assert!(
        project.path().join("Cargo.toml").is_file(),
        "no root Cargo.toml after eject"
    );
    assert!(
        !project.path().join(".horus/Cargo.toml").exists(),
        "the generated manifest was left behind as a second source of truth"
    );

    // Plain cargo, exactly as a developer would run it.
    let output = project.run_cargo(&["test"]);
    let text = combined(&output);
    assert!(
        output.status.success(),
        "cargo test in the ejected project failed:\n{text}"
    );
    assert!(
        text.contains("eject_smoke_marker"),
        "plain cargo did not run the test in tests/:\n{text}"
    );

    let output = project.run_cargo(&["build"]);
    assert!(
        output.status.success(),
        "cargo build in the ejected project failed:\n{}",
        combined(&output)
    );

    let output = project.run(&["test"]);
    assert!(
        output.status.success(),
        "horus test stopped working after the eject:\n{}",
        combined(&output)
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Journey 4: `horus new --cargo` is plain cargo from the first command
// ═══════════════════════════════════════════════════════════════════════════

#[test]
#[ignore = "compiles a real project; run by the User Journey E2E job"]
fn journey_new_cargo_is_plain_cargo_from_the_start() {
    let project = Project::new("journey-carnative", &["--cargo"]);
    assert!(
        project.path().join("Cargo.toml").is_file(),
        "--cargo did not write a root Cargo.toml at creation"
    );

    let output = project.run_cargo(&["build"]);
    assert!(
        output.status.success(),
        "cargo build in a --cargo project failed:\n{}",
        combined(&output)
    );

    project.write(
        "tests/journey_smoke.rs",
        "#[test]\nfn carnative_smoke_marker() {}\n",
    );
    let output = project.run_cargo(&["test"]);
    let text = combined(&output);
    assert!(output.status.success(), "cargo test failed:\n{text}");
    assert!(
        text.contains("carnative_smoke_marker"),
        "plain cargo did not run the test:\n{text}"
    );

    // HORUS still owns the rest of the workflow.
    let output = project.run(&["build"]);
    assert!(
        output.status.success(),
        "horus build in a --cargo project failed:\n{}",
        combined(&output)
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Journey 5: workspaces — member tests run, eject produces a real workspace
// ═══════════════════════════════════════════════════════════════════════════

#[test]
#[ignore = "compiles a real project; run by the User Journey E2E job"]
fn journey_workspace_member_tests_run_and_eject_builds() {
    let project = Project::new("journey-ws", &["--workspace"]);
    let member = project.path().join("crates/journey-ws");
    assert!(
        member.join("src/main.rs").is_file(),
        "workspace member was not scaffolded at {}",
        member.display()
    );
    fs::create_dir_all(member.join("tests")).expect("create member tests dir");
    fs::write(
        member.join("tests/journey_smoke.rs"),
        "#[test]\nfn member_smoke_marker() {}\n",
    )
    .expect("write member test");

    // A member's tests live outside the project root's `tests/`, which is
    // exactly the case that used to compile to nothing.
    let output = project.run(&["test"]);
    let text = combined(&output);
    assert!(
        output.status.success(),
        "horus test in a workspace failed:\n{text}"
    );
    assert!(
        text.contains("member_smoke_marker"),
        "the workspace member's test was not discovered or run:\n{text}"
    );

    let output = project.run(&["eject"]);
    assert!(
        output.status.success(),
        "horus eject on a workspace failed:\n{}",
        combined(&output)
    );
    let root = fs::read_to_string(project.path().join("Cargo.toml")).expect("read root manifest");
    assert!(
        root.contains("members = [\"crates/journey-ws\"]"),
        "the ejected workspace does not point at the member directory:\n{root}"
    );
    assert!(
        member.join("Cargo.toml").is_file(),
        "the member manifest was not written into the member directory"
    );

    let output = project.run_cargo(&["build"]);
    assert!(
        output.status.success(),
        "cargo build of the ejected workspace failed:\n{}",
        combined(&output)
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Journey 6: the cargo proxy, inside and outside a project
// ═══════════════════════════════════════════════════════════════════════════

/// Inside a managed project `horus cargo` generates the manifest and runs
/// cargo against it; outside one it is a passthrough to the real binary.
#[test]
#[ignore = "compiles a real project; run by the User Journey E2E job"]
fn journey_cargo_proxy_builds_managed_projects_and_passes_through() {
    let project = Project::new("journey-proxy", &[]);

    let output = project.run(&["cargo", "build"]);
    assert!(
        output.status.success(),
        "horus cargo build failed in a managed project:\n{}",
        combined(&output)
    );
    assert!(
        project.path().join(".horus/Cargo.toml").is_file(),
        "the proxy did not generate the managed manifest"
    );

    // Outside a HORUS project the proxy must not touch anything.
    let elsewhere = TempDir::new().expect("temp dir");
    let output = Command::new(horus_bin())
        .args(["cargo", "--version"])
        .current_dir(elsewhere.path())
        .timeout(QUICK_TIMEOUT)
        .output()
        .expect("run horus cargo --version");
    let text = combined(&output);
    assert!(
        output.status.success(),
        "horus cargo --version outside a project failed:\n{text}"
    );
    assert!(
        text.contains("cargo"),
        "the proxy did not pass through to the real cargo:\n{text}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Journey 7: run, then stop it the way a user does
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(unix)]
#[test]
#[ignore = "compiles and runs a real project; run by the User Journey E2E job"]
fn journey_running_node_stops_cleanly_on_sigterm() {
    let project = Project::new("journey-run", &[]);
    project.write(
        "src/main.rs",
        r#"
use horus::prelude::*;

struct JourneyTick;

impl Node for JourneyTick {
    fn name(&self) -> &str {
        "journey_tick"
    }

    fn init(&mut self) -> Result<()> {
        println!("JOURNEY_RUNNING");
        Ok(())
    }

    fn tick(&mut self) {}
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();
    scheduler.add(JourneyTick).order(0).build()?;
    // Runs until the signal handler says otherwise — the only way out.
    scheduler.run()?;
    Ok(())
}
"#,
    );

    let log_path = project.path().join("journey-run.log");
    let log = fs::File::create(&log_path).expect("create log");
    let log_err = log.try_clone().expect("clone log handle");
    let mut command = std::process::Command::new(horus_bin());
    command
        .arg("run")
        .current_dir(project.path())
        .stdout(log)
        .stderr(log_err);
    // Its own process group, the way a terminal's foreground group works:
    // Ctrl+C signals every process in it. `horus run` spawns the node as a
    // child and does not forward signals, so a lone SIGTERM to horus kills the
    // parent and orphans the node — which is what the first version of this
    // test did, and the node kept running after the test failed.
    {
        use std::os::unix::process::CommandExt;
        command.process_group(0);
    }
    let mut child = command.spawn().expect("spawn horus run");

    // Wait for the node to be up. The build happens first, so this can take
    // as long as a build does.
    let deadline = std::time::Instant::now() + BUILD_TIMEOUT;
    loop {
        let text = fs::read_to_string(&log_path).unwrap_or_default();
        if text.contains("JOURNEY_RUNNING") {
            break;
        }
        if let Some(status) = child.try_wait().expect("try_wait") {
            panic!("horus run exited before the node started: {status:?}\n{text}");
        }
        if std::time::Instant::now() > deadline {
            let _ = child.kill();
            panic!("horus run never started the node within {BUILD_TIMEOUT:?}\n{text}");
        }
        std::thread::sleep(Duration::from_millis(500));
    }

    // The node horus spawned is its direct child. Remember it now so the check
    // below is about THIS run's node — `pgrep -f journey-run` also matches
    // leftovers from earlier runs, because the shared target directory gives
    // every run the same binary path.
    let node_pid = {
        let output = std::process::Command::new("pgrep")
            .args(["-P", &child.id().to_string()])
            .output()
            .expect("run pgrep -P");
        let text = String::from_utf8_lossy(&output.stdout).to_string();
        text.split_whitespace()
            .next()
            .and_then(|pid| pid.parse::<i32>().ok())
            .unwrap_or_else(|| {
                panic!(
                    "could not find the node process under horus (pid {}):\n{text}",
                    child.id()
                )
            })
    };

    // Ctrl+C semantics: signal the whole group, so the node receives it too.
    // `libc::kill` takes the negative PID directly; shelling out to `kill`
    // would have to dodge its option parser treating `-<pgid>` as a flag.
    let ret = unsafe { libc::kill(-(child.id() as i32), libc::SIGTERM) };
    assert_eq!(
        ret,
        0,
        "kill(-pgid, SIGTERM) failed: {}",
        std::io::Error::last_os_error()
    );

    // Wait for the node to acknowledge and for horus to exit. horus dies from
    // the signal itself (as it does under Ctrl+C), so its exit status is not
    // the assertion — the scheduler's acknowledgement is.
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    loop {
        let text = fs::read_to_string(&log_path).unwrap_or_default();
        if text.contains("SIGTERM received") && child.try_wait().expect("try_wait").is_some() {
            break;
        }
        if std::time::Instant::now() > deadline {
            let _ = child.kill();
            panic!("the node did not acknowledge SIGTERM within 60s:\n{text}");
        }
        std::thread::sleep(Duration::from_millis(250));
    }

    let text = fs::read_to_string(&log_path).unwrap_or_default();
    assert!(
        text.contains("SIGTERM received"),
        "the scheduler did not acknowledge the signal:\n{text}"
    );
    assert!(
        !text.contains("panicked"),
        "the node panicked on shutdown:\n{text}"
    );

    // Nothing may be left running — the failure mode this guards is an orphaned
    // node that outlives the horus process that started it. A zombie is not
    // running: it holds no resources and only waits for a parent to reap it,
    // which in a container may be PID 1 and never happen.
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    loop {
        match process_state(node_pid) {
            None => break,
            Some(state) if state.starts_with('Z') => break,
            Some(state) => {
                if std::time::Instant::now() > deadline {
                    panic!("node {node_pid} still running after SIGTERM (state {state})");
                }
            }
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// `ps` state of a process, or `None` when it no longer exists.
#[cfg(unix)]
fn process_state(pid: i32) -> Option<String> {
    let output = std::process::Command::new("ps")
        .args(["-o", "stat=", "-p", &pid.to_string()])
        .output()
        .ok()?;
    let state = String::from_utf8_lossy(&output.stdout).trim().to_string();
    if state.is_empty() {
        None
    } else {
        Some(state)
    }
}
