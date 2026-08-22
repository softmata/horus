//! The Dockerfile and the devcontainer have to be the images they say they are.
//!
//! TOOL-2 shipped a root `Dockerfile`, a `.devcontainer/`, a man page and a
//! CHANGELOG, and reported them "installed/CI-tested". The container half was
//! not tested by anything:
//!
//!   * The only `docker build` in all 15 workflows is in
//!     `docker-distro-tests.yml`, and it builds `tests/docker/Dockerfile.<distro>`
//!     — a test harness, a different file. Nothing built the root Dockerfile.
//!   * `devcontainer.json` set `"target": "builder"`, skipping the purpose-built
//!     `dev` stage (python3, cmake, g++, clang-format, clang-tidy) and then
//!     re-installing a *different* list — cmake, libeigen3-dev, libfmt-dev,
//!     libgtest-dev — from `postCreateCommand`. Two lists that could disagree,
//!     and the test that claimed to prevent that only grepped the file for the
//!     substrings "Dockerfile" and "shm-size".
//!   * That `postCreateCommand` ran `sudo apt-get` as `remoteUser: root` in an
//!     image derived from `rust:1.90-slim-bookworm`, which ships no sudo. It
//!     failed on every create, so the container came up with no cmake, no g++,
//!     no python3 and none of the three C++ libraries.
//!
//! Everything below reads the real files and models what Docker actually does
//! with them — stage inheritance included, since "the package is installed" is a
//! property of a stage's whole ancestry, not of one RUN line.

use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn dockerfile() -> String {
    fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile must exist")
}

/// `.devcontainer/devcontainer.json` is JSONC. Strip whole-line `//` comments
/// (the only kind the file uses) and parse the rest.
fn devcontainer() -> serde_json::Value {
    let raw = fs::read_to_string(repo_root().join(".devcontainer/devcontainer.json"))
        .expect(".devcontainer/devcontainer.json must exist");
    let stripped: String = raw
        .lines()
        .filter(|l| !l.trim_start().starts_with("//"))
        .collect::<Vec<_>>()
        .join("\n");
    serde_json::from_str(&stripped).expect("devcontainer.json must be parseable JSONC")
}

struct Stage {
    base: String,
    /// Non-comment lines belonging to this stage.
    body: Vec<String>,
}

/// name -> stage, in file order.
fn stages() -> (Vec<String>, HashMap<String, Stage>) {
    let src = dockerfile();
    let mut order = Vec::new();
    let mut map: HashMap<String, Stage> = HashMap::new();
    let mut current: Option<String> = None;

    for line in src.lines() {
        let t = line.trim();
        if t.starts_with('#') {
            continue;
        }
        if let Some(rest) = t.strip_prefix("FROM ") {
            let mut parts = rest.split_whitespace();
            let base = parts.next().unwrap_or("").to_string();
            // `FROM <base> AS <name>`
            let name = match (parts.next(), parts.next()) {
                (Some(kw), Some(n)) if kw.eq_ignore_ascii_case("as") => n.to_string(),
                _ => base.clone(),
            };
            order.push(name.clone());
            map.insert(
                name.clone(),
                Stage {
                    base,
                    body: Vec::new(),
                },
            );
            current = Some(name);
            continue;
        }
        if let Some(name) = &current {
            if !t.is_empty() {
                map.get_mut(name)
                    .expect("stage exists")
                    .body
                    .push(t.to_string());
            }
        }
    }
    (order, map)
}

/// Every line of `stage` and of everything it is built FROM, outermost last.
fn ancestry_body(stage: &str) -> Vec<String> {
    let (_, map) = stages();
    let mut out = Vec::new();
    let mut cursor = stage.to_string();
    let mut guard = 0;
    loop {
        guard += 1;
        assert!(guard < 32, "FROM cycle around {stage}");
        let Some(s) = map.get(&cursor) else { break };
        out.extend(s.body.iter().cloned());
        // A base that is another named stage in this file means we keep walking;
        // an external image (rust:..., debian:...) ends the chain.
        if map.contains_key(&s.base) && s.base != cursor {
            cursor = s.base.clone();
        } else {
            break;
        }
    }
    out
}

fn devcontainer_target() -> String {
    devcontainer()["build"]["target"]
        .as_str()
        .expect("devcontainer.json must name a build target")
        .to_string()
}

/// The stage `devcontainer.json` builds must exist. A missing stage makes
/// `docker build --target` fail with "target stage not found" the first time
/// somebody opens the folder.
#[test]
fn the_devcontainer_builds_a_stage_the_dockerfile_defines() {
    let (order, _) = stages();
    let target = devcontainer_target();
    assert!(
        order.contains(&target),
        "devcontainer.json builds stage {target:?}; the Dockerfile defines {order:?}"
    );

    let file = devcontainer()["build"]["dockerfile"]
        .as_str()
        .expect("devcontainer.json must name a dockerfile")
        .to_string();
    assert!(
        repo_root().join(".devcontainer").join(&file).exists(),
        "devcontainer.json points at .devcontainer/{file}, which does not exist"
    );
}

/// This is the one `"target": "builder"` failed.
///
/// The devcontainer configures rust-analyzer for clippy, opens Python files and
/// is documented as the place the C++ bindings are developed. Each of those
/// needs something installed, and `builder` installs none of it.
#[test]
fn the_devcontainer_stage_installs_every_toolchain_it_configures() {
    let target = devcontainer_target();
    let body = ancestry_body(&target).join("\n");

    // (apt package or rustup component, why the devcontainer needs it)
    let required = [
        ("python3", "devcontainer.json installs ms-python.python"),
        ("cmake", "the C++ bindings are configured with cmake"),
        ("g++", "the C++ examples have to compile"),
        ("clang-format", "`horus fmt` shells out to it for C++"),
        ("clang-tidy", "`horus lint` shells out to it for C++"),
        ("libeigen3-dev", "the C++ examples link against Eigen"),
        ("libfmt-dev", "the C++ examples link against fmt"),
        ("libgtest-dev", "the C++ gtests link against it"),
        ("clippy", "rust-analyzer.check.command is set to clippy"),
        ("rustfmt", "`horus fmt` shells out to rustfmt"),
    ];

    let missing: Vec<String> = required
        .iter()
        .filter(|(pkg, _)| !body.contains(pkg))
        .map(|(pkg, why)| format!("{pkg} ({why})"))
        .collect();

    assert!(
        missing.is_empty(),
        "the devcontainer builds stage {target:?}, whose ancestry never installs: {missing:?}"
    );
}

/// The devcontainer must reuse the Dockerfile's package list rather than keep a
/// second one. `changelog_contract.rs` claims to test this by grepping the file
/// for the substring "Dockerfile" — which stays true while the two lists say
/// completely different things.
#[test]
fn the_devcontainer_does_not_keep_its_own_package_list() {
    let dc = devcontainer();
    let post = dc["postCreateCommand"].as_str().unwrap_or("").to_string();

    assert!(
        !post.contains("apt-get install") && !post.contains("apt install"),
        "postCreateCommand installs packages itself:\n  {post}\n\
         That is a second dependency list beside the Dockerfile's, and the two \
         had already diverged (the Dockerfile installs clang-format/clang-tidy, \
         this installed libeigen3-dev/libfmt-dev/libgtest-dev, neither had both)."
    );
}

/// `remoteUser` is root and the base is a Debian *slim* image, which ships no
/// sudo. Anything create-time must be runnable in the image that was built.
#[test]
fn the_devcontainer_does_not_reach_for_tools_the_image_lacks() {
    let dc = devcontainer();
    let post = dc["postCreateCommand"].as_str().unwrap_or("").to_string();
    if post.is_empty() {
        return;
    }
    let body = ancestry_body(&devcontainer_target()).join("\n");

    for tool in ["sudo", "apt-get", "curl", "wget", "pip"] {
        if post.contains(tool) && !body.contains(tool) {
            panic!(
                "postCreateCommand runs `{tool}`, which nothing in the {:?} stage \
                 ancestry installs. remoteUser is {:?} and the base image is a \
                 Debian slim, so this fails on every container create:\n  {post}",
                devcontainer_target(),
                dc["remoteUser"].as_str().unwrap_or("(unset)"),
            );
        }
    }
}

/// A devcontainer is a place to work in: the editor execs into it. Inheriting
/// `ENTRYPOINT ["horus"]` from the dev stage turns every command the editor
/// runs into an argument to horus.
#[test]
fn the_devcontainer_stage_clears_the_horus_entrypoint() {
    let target = devcontainer_target();
    // ancestry_body is nearest-first, so the first ENTRYPOINT found is the
    // effective one.
    let effective = ancestry_body(&target)
        .into_iter()
        .find(|l| l.starts_with("ENTRYPOINT"));

    match effective {
        None => {}
        Some(line) => assert!(
            line.replace(' ', "") == "ENTRYPOINT[]",
            "the {target:?} stage's effective entrypoint is `{line}`; a devcontainer \
             must not have one, or `docker exec <cmd>` becomes `horus <cmd>`"
        ),
    }
}

fn workflows() -> Vec<(String, String)> {
    let dir = repo_root().join(".github/workflows");
    let mut out = Vec::new();
    for entry in fs::read_dir(&dir).expect(".github/workflows must exist") {
        let path = entry.expect("dir entry").path();
        if path.extension().and_then(|e| e.to_str()) == Some("yml") {
            let name = path.file_name().unwrap().to_string_lossy().to_string();
            out.push((name, fs::read_to_string(&path).expect("workflow readable")));
        }
    }
    assert!(!out.is_empty(), "no workflows found — this test is vacuous");
    out
}

/// Collapse `\\`-continued shell lines into one logical command each, so a flag
/// on a continuation line is visible from the line that starts the command.
fn join_continuations(body: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut acc = String::new();
    for line in body.lines() {
        let t = line.trim();
        if let Some(head) = t.strip_suffix('\\') {
            acc.push_str(head.trim_end());
            acc.push(' ');
        } else {
            acc.push_str(t);
            out.push(std::mem::take(&mut acc));
        }
    }
    if !acc.is_empty() {
        out.push(acc);
    }
    out
}

/// The claim was "installed/CI-tested". Every `docker build` in the repo pointed
/// at `tests/docker/Dockerfile.<distro>`; the root Dockerfile and the
/// devcontainer had never been built by anything.
#[test]
fn ci_builds_the_root_dockerfile_and_the_devcontainer() {
    let wfs = workflows();

    // A build of the root Dockerfile is one with no `-f`/`--file` pointing
    // somewhere else. `tests/docker/Dockerfile.ubuntu` is a different file and
    // does not count — and it is written across four continuation lines, so the
    // `-f` is not on the same physical line as `docker build`. Joining
    // continuations first is the difference between this test and one that
    // passes because it only ever looked at "docker build \\".
    let builds_root = wfs.iter().any(|(_, body)| {
        join_continuations(body)
            .iter()
            .any(|c| c.starts_with("docker build") && !c.contains("-f ") && !c.contains("--file"))
    });
    assert!(
        builds_root,
        "no workflow builds the root Dockerfile; the only `docker build` in the \
         repo used to be docker-distro-tests.yml, which builds \
         tests/docker/Dockerfile.<distro> — a different file"
    );

    let target = devcontainer_target();
    let builds_devcontainer = wfs
        .iter()
        .any(|(_, body)| body.contains("devcontainer.json") && body.contains("docker build"))
        || wfs
            .iter()
            .any(|(_, body)| body.contains(&format!("--target {target}")));
    assert!(
        builds_devcontainer,
        "no workflow builds the {target:?} stage that devcontainer.json names, so \
         nothing would notice it naming the wrong stage again"
    );
}

/// `docker build .` builds the *last* stage. Adding the devcontainer stage must
/// not change what the plain command produces — the Dockerfile header documents
/// it as the slim CLI image.
#[test]
fn the_default_docker_build_is_still_the_slim_image() {
    let (order, _) = stages();
    assert_eq!(
        order.last().map(String::as_str),
        Some("runtime"),
        "`docker build .` builds the last stage; stages are {order:?}"
    );
}
