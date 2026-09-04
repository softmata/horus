//! `horus doctor` — comprehensive ecosystem health check.
//!
//! Checks, in the order they print: the install itself (which binary is
//! running, what the installer recorded, and whether the cached source tree
//! speaks the same topic ABI as this CLI), toolchains, manifest validity,
//! real-time capability, shared memory, plugins, disk, languages, dependency
//! sources, hardware, system deps, and the network — including the package
//! registry `horus search` talks to. Summary by default, --verbose for
//! details.
//!
//! With `--fix`: installs missing toolchains and system dependencies,
//! then pins their versions in `horus.lock`.
//!
//! With `--rt`: runs the RT Readiness Report — system audit, jitter
//! benchmark, IPC benchmark, and actionable recommendations.

use anyhow::Result;
use colored::*;
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::time::Duration;

use horus_core::drivers::NodeParams;
use horus_sys::sync::{self, SyncManifest, SystemDep};

use crate::dispatch;
use crate::lockfile::{HorusLockfile, SystemLock, HORUS_LOCK};
use crate::manifest::HorusManifest;

/// Health status for a check category.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Health {
    Ok,
    Warn,
    Fail,
}

impl Health {
    fn icon(&self) -> colored::ColoredString {
        match self {
            Self::Ok => "*".green(),
            Self::Warn => "!".yellow(),
            Self::Fail => "x".red(),
        }
    }

    /// The worse of two verdicts, for a check that gathers several findings and
    /// must grade on the worst of them rather than the last one written.
    fn worst(self, other: Self) -> Self {
        match (self, other) {
            (Self::Fail, _) | (_, Self::Fail) => Self::Fail,
            (Self::Warn, _) | (_, Self::Warn) => Self::Warn,
            _ => Self::Ok,
        }
    }
}

/// A single check result.
pub(crate) struct CheckResult {
    pub(crate) category: String,
    pub(crate) health: Health,
    pub(crate) summary: String,
    pub(crate) details: Vec<String>,
}

/// Run `horus doctor`.
pub fn run_doctor(verbose: bool, json: bool, fix: bool) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);
    let mut results = vec![
        // First, because it is the only check that can see the install itself:
        // every other one here passes on a machine whose CLI and libraries came
        // from different refs.
        check_installation(),
        check_toolchains(),
        check_manifest(&ctx),
        check_rt(),
        check_shm(),
        check_plugins(),
        check_disk(),
        check_languages(&ctx),
        check_dep_sources(&ctx),
        check_drivers(&ctx),
    ];

    // ── 11. System dependencies (Python, C++, system libs) ───────────────
    if let Some(manifest) = &ctx.manifest {
        results.push(check_system_deps(manifest));
    }

    // ── 12. Network (horus_net) ──────────────────────────────────────────
    results.push(check_network());

    // ── Output ───────────────────────────────────────────────────────────
    if json {
        print_json(&results);
    } else {
        print_summary(&results, verbose);
    }

    // ── Fix mode: install missing deps and pin to horus.lock ─────────────
    if fix {
        if let Some(manifest) = &ctx.manifest {
            run_fix(manifest, &ctx)?;
        } else {
            println!(
                "\n  {} No horus.toml found — nothing to fix. Run {} to create a project.",
                "!".yellow(),
                "horus new".cyan()
            );
        }
    }

    // Exit code
    let has_failures = results.iter().any(|r| r.health == Health::Fail);
    let has_warnings = results.iter().any(|r| r.health == Health::Warn);

    if has_failures && !fix {
        std::process::exit(2);
    } else if has_warnings && !fix {
        std::process::exit(1);
    }
    Ok(())
}

/// Install missing toolchains/system deps and pin versions to horus.lock.
fn run_fix(manifest: &HorusManifest, ctx: &dispatch::ProjectContext) -> Result<()> {
    println!("\n{}", "Fixing environment...".bold());

    let report = sync::sync_environment(manifest)?;

    for item in &report.items {
        if item.installed {
            let version = item.version.as_deref().unwrap_or("installed");
            println!("  {} {} ({})", "*".green(), item.name, version);
        } else {
            println!("  {} {} -- not installed", "x".red(), item.name);
            if let Some(ref cmd) = item.install_cmd {
                println!("    Install: {}", cmd.dimmed());
            }
        }
    }

    // Pin toolchain + system dep versions into horus.lock
    let lock_path = ctx.root.join(HORUS_LOCK);
    let mut lockfile = HorusLockfile::load_from(&lock_path).unwrap_or_default();

    // Build toolchain pins from the sync report
    let mut toolchain = lockfile.toolchain.unwrap_or_default();
    for item in &report.items {
        if let Some(ver) = &item.version {
            match item.name.as_str() {
                "rust" => toolchain.rust = Some(ver.clone()),
                "python" => toolchain.python = Some(ver.clone()),
                "cmake" => toolchain.cmake = Some(ver.clone()),
                _ => {}
            }
        }
    }
    lockfile.toolchain = Some(toolchain);

    // Build system dep pins from the sync report
    let system_dep_names: Vec<String> = manifest
        .system_deps()
        .iter()
        .map(|d| d.name.clone())
        .collect();
    for item in &report.items {
        if system_dep_names.contains(&item.name) {
            if let Some(ver) = &item.version {
                // Find the manifest dep for cross-platform package names
                let manifest_dep = manifest
                    .system_deps()
                    .into_iter()
                    .find(|d| d.name == item.name);
                let existing = lockfile
                    .system_deps
                    .iter_mut()
                    .find(|s| s.name == item.name);
                if let Some(existing) = existing {
                    existing.version = ver.clone();
                } else {
                    lockfile.system_deps.push(SystemLock {
                        name: item.name.clone(),
                        version: ver.clone(),
                        pkg_config: manifest_dep.as_ref().and_then(|d| d.pkg_config.clone()),
                        apt: manifest_dep.as_ref().and_then(|d| d.apt.clone()),
                        brew: manifest_dep.as_ref().and_then(|d| d.brew.clone()),
                        pacman: None,
                        choco: manifest_dep.as_ref().and_then(|d| d.choco.clone()),
                    });
                }
            }
        }
    }

    lockfile.save_to(&lock_path)?;
    println!(
        "\n  {} Environment synced — {} updated",
        "*".green(),
        HORUS_LOCK.bold()
    );

    if !report.all_satisfied {
        println!(
            "\n{} Some dependencies could not be installed automatically.",
            "!".yellow()
        );
    }

    Ok(())
}

pub(crate) fn print_summary(results: &[CheckResult], verbose: bool) {
    println!("{}", "horus doctor".bold());
    println!();

    for result in results {
        println!(
            "  {} {} — {}",
            result.health.icon(),
            result.category.bold(),
            result.summary
        );
        if verbose {
            for detail in &result.details {
                println!("      {}", detail.dimmed());
            }
        }
    }

    println!();
    let ok_count = results.iter().filter(|r| r.health == Health::Ok).count();
    let warn_count = results.iter().filter(|r| r.health == Health::Warn).count();
    let fail_count = results.iter().filter(|r| r.health == Health::Fail).count();

    if fail_count > 0 {
        println!(
            "  {} {ok_count} ok, {warn_count} warnings, {fail_count} failures",
            "Summary:".bold()
        );
    } else if warn_count > 0 {
        println!(
            "  {} {ok_count} ok, {warn_count} warnings",
            "Summary:".bold()
        );
    } else {
        println!("  {} All {ok_count} checks passed", "Summary:".bold());
    }
}

fn print_json(results: &[CheckResult]) {
    let entries: Vec<serde_json::Value> = results
        .iter()
        .map(|r| {
            serde_json::json!({
                "category": r.category,
                "health": format!("{:?}", r.health).to_lowercase(),
                "summary": r.summary,
                "details": r.details,
            })
        })
        .collect();
    println!("{}", serde_json::to_string_pretty(&entries).unwrap());
}

// ─── Individual checks ───────────────────────────────────────────────────────

/// One requirement inside a language toolchain.
///
/// `alternatives` is a set rather than a single binary because the thing a
/// build actually needs is "a C++ compiler", not "g++". `cmake --build` is
/// equally happy with clang or MSVC, so naming one implementation would report
/// a working clang-only machine as broken — the same false verdict as the one
/// this check exists to prevent, just pointing the other way.
struct Tool {
    /// Executables that satisfy this requirement. Any one present is enough;
    /// the order is only the order they are probed in.
    alternatives: &'static [&'static str],
    /// What the requirement is called when none of the alternatives are found.
    label: &'static str,
    /// What it is for.
    purpose: &'static str,
    /// How to get it. Naming a missing tool without naming the command that
    /// installs it just moves the user's search one line further down.
    install: Install,
}

/// Where a missing tool comes from.
///
/// Resolved at print time rather than baked into the table, because the answer
/// depends on the distro underfoot.
enum Install {
    /// A package from the platform's own package manager.
    Package(&'static str),
    /// The compiler + make bundle: build-essential, base-devel, Xcode CLT, …
    BuildTools,
    /// A command that reads the same on every platform.
    Same(&'static str),
}

impl Install {
    fn command(&self) -> String {
        match self {
            Self::Package(pkg) => horus_sys::platform::suggest_install(pkg),
            Self::BuildTools => horus_sys::platform::suggest_build_tools(),
            Self::Same(cmd) => (*cmd).to_string(),
        }
    }
}

impl Tool {
    /// The `-v` line for a tool that is not installed.
    ///
    /// Carries the install command, because "clang-format: not found" leaves
    /// the reader exactly where they started.
    fn missing_detail(&self) -> String {
        format!(
            "{}: not found ({}) - install: {}",
            self.label,
            self.purpose,
            self.install.command()
        )
    }
}

/// A language toolchain and the tools it needs.
struct Toolchain {
    language: &'static str,
    /// Without these, the language cannot build at all.
    required: &'static [Tool],
    /// Without these, specific commands (`horus lint`, `horus test`) fail.
    optional: &'static [Tool],
}

const RUSTUP: Install = Install::Same("curl https://sh.rustup.rs -sSf | sh");

const TOOLCHAINS: &[Toolchain] = &[
    Toolchain {
        language: "Rust",
        required: &[
            Tool {
                alternatives: &["cargo"],
                label: "cargo",
                purpose: "build tool",
                install: RUSTUP,
            },
            Tool {
                alternatives: &["rustc"],
                label: "rustc",
                purpose: "compiler",
                install: RUSTUP,
            },
        ],
        // Rust's optional list used to be empty, so "Rust ready" was printed on
        // machines where `horus lint` died with "no such command: `clippy`".
        // `horus lint` on Rust is `cargo clippy -- -D warnings` and `horus fmt`
        // is `cargo fmt` (dispatch::detect_rust_tools); neither component ships
        // with a minimal-profile rustup or with most distro rust packages, which
        // is exactly the population this check is for.
        optional: &[
            Tool {
                alternatives: &["cargo-clippy"],
                label: "clippy",
                purpose: "linter, used by horus lint",
                install: Install::Same("rustup component add clippy"),
            },
            Tool {
                alternatives: &["rustfmt"],
                label: "rustfmt",
                purpose: "formatter, used by horus fmt",
                install: Install::Same("rustup component add rustfmt"),
            },
        ],
    },
    Toolchain {
        language: "Python",
        // `python` is accepted as well as `python3` because that is the order
        // dispatch::find_python resolves in — reporting "Python needs python3"
        // on a machine HORUS would happily run on is the same disagreement
        // between report and executor as the C++ compiler gap below.
        required: &[Tool {
            alternatives: &["python3", "python"],
            label: "python3",
            purpose: "interpreter",
            install: Install::Package("python3"),
        }],
        optional: &[
            Tool {
                alternatives: &["ruff"],
                label: "ruff",
                purpose: "linter/formatter, used by horus lint",
                install: Install::Same("pip install ruff"),
            },
            Tool {
                alternatives: &["pytest"],
                label: "pytest",
                purpose: "test runner, used by horus test",
                install: Install::Same("pip install pytest"),
            },
        ],
    },
    Toolchain {
        language: "C++",
        // cmake on its own is not a C++ toolchain, and listing only cmake here
        // reproduced the original false green one tool further along: on a PATH
        // with cmake but no compiler and no make, doctor printed "C++ ready" and
        // `horus new --cpp && horus build` then died during configure with
        // "CMAKE_MAKE_PROGRAM is not set" / "CMAKE_CXX_COMPILER not set".
        // horus_sys::sync::cpp already probes the compiler for `doctor --fix`;
        // the report has to agree with the fixer.
        required: &[
            Tool {
                alternatives: &["cmake"],
                label: "cmake",
                purpose: "build system",
                install: Install::Package("cmake"),
            },
            Tool {
                alternatives: &["g++", "clang++", "c++", "cl"],
                label: "a C++ compiler",
                purpose: "g++, clang++ or MSVC — cmake picks one",
                install: Install::BuildTools,
            },
            Tool {
                alternatives: &["make", "gmake", "ninja", "msbuild"],
                label: "make or ninja",
                purpose: "the build program cmake --build drives",
                install: Install::BuildTools,
            },
        ],
        optional: &[
            Tool {
                alternatives: &["clang-format"],
                label: "clang-format",
                purpose: "formatter, used by horus fmt",
                install: Install::Package("clang-format"),
            },
            Tool {
                alternatives: &["clang-tidy"],
                label: "clang-tidy",
                purpose: "linter, used by horus lint",
                install: Install::Package("clang-tidy"),
            },
        ],
    },
];

/// Executables that cannot answer `--version` and can only be detected by
/// presence: MSVC's `cl` treats the flag as a filename and exits non-zero, and
/// `msbuild` spells it `-version`. Calling an installed compiler "not found" is
/// the same wrong verdict this check exists to stop, just pointing the other
/// way — but the list is kept as short as possible, because presence is the
/// weaker signal (see `probe`).
const PRESENCE_ONLY: &[&str] = &["cl", "msbuild"];

/// Detect `bin` and return something to print for it.
///
/// A tool counts as installed when it *runs*, not when a file with its name
/// exists. rustup puts a proxy in `~/.cargo/bin` for every component whether or
/// not the component is installed, so `cargo-clippy` and `rustfmt` are on PATH
/// on every rustup machine:
///
/// ```text
/// $ cargo-miri --version          # miri is not installed
/// error: the 'miri' component which provides the command 'cargo-miri' is not
/// available for the 'stable-x86_64-unknown-linux-gnu' toolchain
/// $ echo $?
/// 1
/// ```
///
/// Grading on presence would print "Rust ready" for precisely the machines this
/// check exists to catch — the ones where `horus lint` then dies with
/// "no such command: `clippy`".
fn probe(bin: &str) -> Option<String> {
    if let Some(version) = dispatch::tool_version(bin) {
        return Some(version);
    }
    if PRESENCE_ONLY.contains(&bin) && on_path(bin) {
        return Some("installed".to_string());
    }
    None
}

/// Is `name` an executable on `PATH`?
fn on_path(name: &str) -> bool {
    !all_on_path(name).is_empty()
}

/// *Every* executable named `name` on `PATH`, in the order `PATH` lists them.
///
/// The whole list rather than the first hit, because a machine can hold a
/// ~/.local/bin/horus from one install and a ~/.cargo/bin/horus from another at
/// a different version: PATH order decides which one `horus` means and nothing
/// says so. A lookup that stopped at the first match could not see the pair it
/// has to report.
fn all_on_path(name: &str) -> Vec<PathBuf> {
    if name.contains(std::path::MAIN_SEPARATOR) {
        let path = PathBuf::from(name);
        return if is_executable(&path) {
            vec![path]
        } else {
            Vec::new()
        };
    }
    let Some(path) = std::env::var_os("PATH") else {
        return Vec::new();
    };
    let exts: Vec<String> = std::env::var("PATHEXT")
        .map(|v| {
            v.split(';')
                .filter(|e| !e.is_empty())
                .map(|e| e.to_string())
                .collect()
        })
        .unwrap_or_default();
    executables_in(name, std::env::split_paths(&path), &exts)
}

/// One hit per directory, the way the OS resolves a bare command name.
///
/// Split from the `PATH` lookup so that "every directory, not just the first"
/// can be tested without mutating this process's environment — the toolchain
/// grading above is split for the same reason.
fn executables_in(
    name: &str,
    dirs: impl Iterator<Item = PathBuf>,
    exts: &[String],
) -> Vec<PathBuf> {
    let mut found = Vec::new();
    for dir in dirs {
        if dir.as_os_str().is_empty() {
            continue;
        }
        let candidates = std::iter::once(dir.join(name))
            .chain(exts.iter().map(|ext| dir.join(format!("{name}{ext}"))));
        if let Some(hit) = candidates.into_iter().find(|c| is_executable(c)) {
            found.push(hit);
        }
    }
    found
}

fn is_executable(path: &Path) -> bool {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::metadata(path)
            .map(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
            .unwrap_or(false)
    }
    #[cfg(not(unix))]
    {
        path.is_file()
    }
}

/// Report toolchain readiness per language.
///
/// The previous check reported a bare fraction and graded it on whether cargo
/// and rustc happened to be present. That made "3/8 tools found" a *success*
/// — rendered with the tick and counted among "7 ok" — for a machine that
/// then failed `horus new --cpp && horus build` on missing cmake. It was also
/// non-monotonic: with cargo and rustc absent but six other tools present,
/// the same check reported "6/8" as a failure. Better coverage graded worse.
///
/// Grading per language makes the verdict mean something and names what is
/// missing, which is the part a user can act on.
fn check_toolchains() -> CheckResult {
    let mut details = Vec::new();
    let mut found: std::collections::HashSet<&'static str> = std::collections::HashSet::new();

    for tc in TOOLCHAINS {
        for tool in tc.required.iter().chain(tc.optional.iter()) {
            match tool
                .alternatives
                .iter()
                .find_map(|bin| Some((*bin, probe(bin)?)))
            {
                Some((bin, version)) => {
                    details.push(format!("{bin}: {version} ({})", tool.purpose));
                    found.insert(bin);
                }
                None => details.push(tool.missing_detail()),
            }
        }
    }

    let (mut health, mut summary) = grade_toolchains(&|tool| found.contains(tool));

    // Present is not the same as adequate. `horus doctor` reported "Rust,
    // Python, C++ ready" on a toolchain too old to build HORUS; the user found
    // out minutes into a build, from a cargo error about a package they had
    // never heard of.
    if let Some(found_version) = dispatch::tool_version("rustc")
        .as_deref()
        .and_then(parse_semver)
    {
        if let Some(required) = MINIMUM_RUSTC.and_then(parse_semver) {
            if found_version < required {
                health = Health::Warn;
                summary = format!(
                    "Rust {}.{}.{} is older than the required {}",
                    found_version.0,
                    found_version.1,
                    found_version.2,
                    MINIMUM_RUSTC.unwrap_or("?")
                );
                details.push(format!(
                    "rustc: {}.{}.{} is below the minimum {} — run `rustup update stable`",
                    found_version.0,
                    found_version.1,
                    found_version.2,
                    MINIMUM_RUSTC.unwrap_or("?")
                ));
            }
        }
    }

    CheckResult {
        category: "Toolchains".to_string(),
        health,
        summary,
        details,
    }
}

/// The workspace's declared minimum Rust version.
///
/// Taken from cargo rather than written here, so it cannot drift from
/// `[workspace.package] rust-version`.
const MINIMUM_RUSTC: Option<&str> = option_env!("CARGO_PKG_RUST_VERSION");

/// `(major, minor, patch)` from the first version-looking token in `text`.
///
/// Compares numerically, not as a string: 1.100 is newer than 1.9, and a string
/// comparison says the opposite.
fn parse_semver(text: &str) -> Option<(u32, u32, u32)> {
    let token = text
        .split_whitespace()
        .find(|t| t.chars().next().is_some_and(|c| c.is_ascii_digit()) && t.contains('.'))?;
    let token = token.split('-').next()?;
    let mut parts = token.split('.').map(|p| p.parse::<u32>().ok());
    Some((
        parts.next().flatten()?,
        parts.next().flatten().unwrap_or(0),
        parts.next().flatten().unwrap_or(0),
    ))
}

/// Decide the toolchain verdict from which tools are present.
///
/// Split from the probing so the grading rules are testable without touching
/// the machine — the previous version could only be checked by manipulating
/// PATH, which is why its non-monotonicity went unnoticed.
fn grade_toolchains(present: &dyn Fn(&str) -> bool) -> (Health, String) {
    let mut ready = Vec::new();
    let mut blocked = Vec::new();
    let mut degraded = Vec::new();

    // `present` answers "is this executable on PATH". A requirement is met by
    // any one of its alternatives, so the fan-out lives here rather than in the
    // caller — that keeps the same rule in front of the live probe and the
    // tests, instead of one table being graded two different ways.
    let satisfied = |tool: &Tool| tool.alternatives.iter().any(|bin| present(bin));

    for tc in TOOLCHAINS {
        let missing_required: Vec<&str> = tc
            .required
            .iter()
            .filter(|t| !satisfied(t))
            .map(|t| t.label)
            .collect();
        let missing_optional: Vec<&str> = tc
            .optional
            .iter()
            .filter(|t| !satisfied(t))
            .map(|t| t.label)
            .collect();

        if !missing_required.is_empty() {
            blocked.push(format!(
                "{} needs {}",
                tc.language,
                missing_required.join(", ")
            ));
        } else if !missing_optional.is_empty() {
            degraded.push(format!(
                "{} missing {}",
                tc.language,
                missing_optional.join(", ")
            ));
        } else {
            ready.push(tc.language);
        }
    }

    // Fail only when nothing can be built; warn while any language is
    // incomplete. A machine that builds Rust but not C++ is not "ok".
    let health = if ready.is_empty() && degraded.is_empty() {
        Health::Fail
    } else if blocked.is_empty() && degraded.is_empty() {
        Health::Ok
    } else {
        Health::Warn
    };

    let mut parts = Vec::new();
    if !ready.is_empty() {
        parts.push(format!("{} ready", ready.join(", ")));
    }
    parts.extend(degraded);
    parts.extend(blocked);
    (health, parts.join(" · "))
}

fn check_manifest(ctx: &dispatch::ProjectContext) -> CheckResult {
    if !ctx.has_horus_toml {
        return CheckResult {
            category: "Manifest".to_string(),
            health: Health::Warn,
            summary: "No horus.toml found".to_string(),
            details: vec!["Run 'horus new' or 'horus init' to create a project".to_string()],
        };
    }

    match &ctx.manifest {
        Some(manifest) => match manifest.validate() {
            Ok(warnings) => {
                let mut details = vec![
                    format!("name: {}", manifest.package.name),
                    format!("version: {}", manifest.package.version),
                    format!("dependencies: {}", manifest.dependencies.len()),
                ];
                let health = if warnings.is_empty() {
                    Health::Ok
                } else {
                    details.extend(warnings.iter().map(|w| format!("warning: {}", w)));
                    Health::Warn
                };
                CheckResult {
                    category: "Manifest".to_string(),
                    health,
                    summary: "horus.toml valid".to_string(),
                    details,
                }
            }
            Err(e) => CheckResult {
                category: "Manifest".to_string(),
                health: Health::Fail,
                summary: "horus.toml invalid".to_string(),
                details: vec![e.to_string()],
            },
        },
        // A manifest was found and rejected. `ctx.manifest_error` carries the
        // field, table, line and column that `parse_str` worked out; printing
        // "Failed to parse horus.toml" instead threw all of it away, and said
        // "parse" for a file that had parsed fine.
        None => match &ctx.manifest_error {
            Some(err) => CheckResult {
                category: "Manifest".to_string(),
                health: Health::Fail,
                summary: err.headline().to_string(),
                details: err.to_string().lines().map(str::to_string).collect(),
            },
            // Unreachable in practice: `has_horus_toml` is true, so a manifest
            // was found. Say what is known rather than inventing a cause.
            None => CheckResult {
                category: "Manifest".to_string(),
                health: Health::Fail,
                summary: "horus.toml could not be read".to_string(),
                details: vec![],
            },
        },
    }
}

fn check_rt() -> CheckResult {
    let caps = horus_sys::rt::detect_capabilities();
    let mut details = Vec::new();

    if caps.preempt_rt {
        details.push("PREEMPT_RT kernel active".to_string());
    } else {
        details
            .push("PREEMPT_RT not detected — jitter may be higher (~200μs vs ~20μs)".to_string());
        details.push("For lower jitter, run: horus setup-rt".to_string());
    }

    if caps.max_priority > 0 {
        details.push(format!(
            "SCHED_FIFO available (priority {}-{})",
            caps.min_priority, caps.max_priority
        ));
    } else {
        details.push("SCHED_FIFO not available".to_string());
    }

    if caps.memory_locking {
        details.push("Memory locking available".to_string());
    } else {
        details.push("Memory locking limited — run: horus setup-rt".to_string());
    }

    let isolated = horus_sys::rt::isolated_cores();
    if !isolated.is_empty() {
        details.push(format!("Isolated CPUs: {:?}", isolated));
    }

    let health = if caps.preempt_rt && caps.memory_locking && caps.max_priority > 0 {
        Health::Ok
    } else if caps.max_priority > 0 {
        Health::Warn
    } else {
        Health::Fail
    };

    let summary = if caps.preempt_rt {
        format!(
            "PREEMPT_RT active, jitter ±{}μs",
            caps.estimated_jitter.as_micros()
        )
    } else {
        format!(
            "Standard kernel, jitter ±{}μs (run `horus setup-rt` for ±20μs)",
            caps.estimated_jitter.as_micros()
        )
    };

    CheckResult {
        category: "Real-Time".to_string(),
        health,
        summary,
        details,
    }
}

fn check_shm() -> CheckResult {
    let shm_parent = horus_sys::shm::shm_parent_dir();
    if !shm_parent.exists() {
        return CheckResult {
            category: "Shared Memory".to_string(),
            health: Health::Warn,
            summary: format!("{} not available", shm_parent.display()),
            details: vec!["SHM IPC may not work on this platform".to_string()],
        };
    }

    // Count horus namespaces
    let count = std::fs::read_dir(&shm_parent)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok())
                .filter(|e| e.file_name().to_string_lossy().starts_with("horus_"))
                .count()
        })
        .unwrap_or(0);

    CheckResult {
        category: "Shared Memory".to_string(),
        health: Health::Ok,
        summary: format!(
            "{} available, {} horus namespaces",
            shm_parent.display(),
            count
        ),
        details: vec![],
    }
}

fn check_plugins() -> CheckResult {
    let global_dir = dirs::home_dir()
        .map(|h| h.join(".horus/plugins"))
        .unwrap_or_default();

    let local_dir = Path::new(".horus/plugins");

    let global_count = count_items(&global_dir);
    let local_count = count_items(local_dir);

    CheckResult {
        category: "Plugins".to_string(),
        health: Health::Ok,
        summary: format!("{} global, {} local", global_count, local_count),
        details: vec![],
    }
}

fn check_disk() -> CheckResult {
    let horus_dir = Path::new(".horus");
    if !horus_dir.exists() {
        return CheckResult {
            category: "Disk".to_string(),
            health: Health::Ok,
            summary: "No .horus/ directory yet".to_string(),
            details: vec![],
        };
    }

    let size = dir_size(horus_dir);
    let formatted = format_bytes(size);

    let health = if size > 5_000_000_000 {
        // > 5GB
        Health::Warn
    } else {
        Health::Ok
    };

    CheckResult {
        category: "Disk".to_string(),
        health,
        summary: format!("Build cache uses {}", formatted),
        details: vec![],
    }
}

fn check_languages(ctx: &dispatch::ProjectContext) -> CheckResult {
    if ctx.languages.is_empty() {
        return CheckResult {
            category: "Languages".to_string(),
            health: Health::Warn,
            summary: "No languages detected".to_string(),
            details: vec!["Create source files or a horus.toml with dependencies".to_string()],
        };
    }

    let langs: Vec<String> = ctx.languages.iter().map(|l| l.to_string()).collect();
    CheckResult {
        category: "Languages".to_string(),
        health: Health::Ok,
        summary: langs.join(", "),
        details: vec![],
    }
}

fn check_dep_sources(ctx: &dispatch::ProjectContext) -> CheckResult {
    let manifest = match &ctx.manifest {
        Some(m) => m,
        None => {
            // "No manifest" is only true when there is no manifest. When one
            // exists and is broken, saying it was skipped for absence sends the
            // reader looking for a missing file instead of at the error the
            // Manifest check just printed.
            let (summary, health) = match &ctx.manifest_error {
                Some(_) => ("Manifest unreadable — skipped", Health::Warn),
                None => ("No manifest — skipped", Health::Ok),
            };
            return CheckResult {
                category: "Dependencies".to_string(),
                health,
                summary: summary.to_string(),
                details: vec![],
            };
        }
    };

    if manifest.dependencies.is_empty() && manifest.dev_dependencies.is_empty() {
        return CheckResult {
            category: "Dependencies".to_string(),
            health: Health::Ok,
            summary: "No dependencies".to_string(),
            details: vec![],
        };
    }

    let issues = crate::source_resolver::validate_deps(&manifest.dependencies, &ctx.languages);
    let dev_issues =
        crate::source_resolver::validate_deps(&manifest.dev_dependencies, &ctx.languages);

    let all_issues: Vec<_> = issues.into_iter().chain(dev_issues).collect();

    if all_issues.is_empty() {
        let dep_count = manifest.dependencies.len() + manifest.dev_dependencies.len();
        CheckResult {
            category: "Dependencies".to_string(),
            health: Health::Ok,
            summary: format!("{} deps, sources look correct", dep_count),
            details: vec![],
        }
    } else {
        let has_errors = all_issues.iter().any(|i| i.is_error);
        let details: Vec<String> = all_issues.iter().map(|i| i.message.clone()).collect();
        CheckResult {
            category: "Dependencies".to_string(),
            health: if has_errors {
                Health::Fail
            } else {
                Health::Warn
            },
            summary: format!("{} issue(s) found", all_issues.len()),
            details,
        }
    }
}

// ─── System dependency check (absorbed from horus sync) ──────────────────────

/// Implement SyncManifest for HorusManifest so horus_sys::sync can extract requirements.
impl SyncManifest for HorusManifest {
    fn rust_edition(&self) -> Option<String> {
        Some(self.package.edition.clone())
    }

    fn python_version(&self) -> Option<String> {
        let has_python = self.dependencies.iter().any(|(_, dep)| {
            if let crate::manifest::DependencyValue::Detailed(d) = dep {
                matches!(d.source, Some(crate::manifest::DepSource::PyPI))
            } else {
                false
            }
        });
        if has_python {
            Some(">=3.9".to_string())
        } else {
            None
        }
    }

    fn system_deps(&self) -> Vec<SystemDep> {
        self.dependencies
            .iter()
            .filter_map(|(name, dep)| {
                if let crate::manifest::DependencyValue::Detailed(d) = dep {
                    if matches!(d.source, Some(crate::manifest::DepSource::System)) {
                        return Some(SystemDep {
                            name: name.clone(),
                            apt: d.apt.clone().or_else(|| Some(name.clone())),
                            brew: Some(name.clone()),
                            choco: Some(name.clone()),
                            pkg_config: d.cmake_package.clone(),
                        });
                    }
                }
                None
            })
            .collect()
    }

    fn needs_cpp(&self) -> bool {
        self.cpp.is_some()
    }

    fn project_name(&self) -> String {
        self.package.name.clone()
    }
}

fn check_system_deps(manifest: &HorusManifest) -> CheckResult {
    let report = sync::check_environment(manifest);

    if report.items.is_empty() {
        return CheckResult {
            category: "System Deps".to_string(),
            health: Health::Ok,
            summary: "No system dependencies required".to_string(),
            details: vec![],
        };
    }

    let mut details = Vec::new();
    let mut missing = Vec::new();

    for item in &report.items {
        if item.installed {
            let version = item.version.as_deref().unwrap_or("installed");
            details.push(format!("{}: {} ({})", item.name, version, "ok"));
        } else {
            details.push(format!("{}: not found", item.name));
            if item.required {
                missing.push(item.name.clone());
            }
        }
    }

    let health = if !missing.is_empty() {
        Health::Fail
    } else {
        Health::Ok
    };

    let found = report.items.iter().filter(|i| i.installed).count();
    let hint = if !missing.is_empty() {
        format!(" — run {} to install", "horus doctor --fix".cyan())
    } else {
        String::new()
    };

    CheckResult {
        category: "System Deps".to_string(),
        health,
        summary: format!("{}/{} satisfied{}", found, report.items.len(), hint),
        details,
    }
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

fn count_items(dir: &Path) -> usize {
    std::fs::read_dir(dir)
        .map(|entries| entries.filter_map(|e| e.ok()).count())
        .unwrap_or(0)
}

use crate::fs_utils::{dir_size, format_bytes};

// ── Driver device reachability ─────────────────────────────────────────────

fn check_drivers(ctx: &dispatch::ProjectContext) -> CheckResult {
    // Uses the manifest the context already found. Loading `./horus.toml`
    // directly meant this check looked in the working directory rather than the
    // project root, so it reported "No horus.toml found" from any subdirectory
    // of a project — and said the same thing when the file was present and
    // simply failed to parse.
    let manifest = match &ctx.manifest {
        Some(m) => m,
        None => {
            let summary = match (&ctx.manifest_error, ctx.has_horus_toml) {
                (Some(_), _) => "Manifest unreadable — skipped",
                (None, true) => "horus.toml unreadable — skipped",
                (None, false) => "No horus.toml found",
            };
            return CheckResult {
                category: "Hardware".into(),
                health: Health::Ok,
                summary: summary.into(),
                details: vec![],
            };
        }
    };

    // Merge [hardware] and legacy [drivers] entries
    let mut all_entries: std::collections::BTreeMap<String, &crate::manifest::DriverValue> =
        std::collections::BTreeMap::new();
    for (k, v) in &manifest.hardware {
        all_entries.insert(k.clone(), v);
    }
    for (k, v) in &manifest.drivers {
        all_entries.entry(k.clone()).or_insert(v);
    }

    if all_entries.is_empty() {
        return CheckResult {
            category: "Hardware".into(),
            health: Health::Ok,
            summary: "No [hardware] configured".into(),
            details: vec![],
        };
    }

    let mut details = Vec::new();
    let mut worst = Health::Ok;

    for (name, value) in &all_entries {
        let params_map = match value {
            crate::manifest::DriverValue::Config(cfg) => &cfg.params,
            _ => continue,
        };
        let params = NodeParams::new(params_map.clone());
        let use_name = match value {
            crate::manifest::DriverValue::Config(cfg) => cfg
                .use_name
                .as_deref()
                .or(cfg.terra.as_deref())
                .or(cfg.node.as_deref())
                .or(cfg.package.as_deref())
                .or(cfg.exec.as_deref())
                .unwrap_or("unknown"),
            crate::manifest::DriverValue::Backend(s) => s.as_str(),
            crate::manifest::DriverValue::Enabled(_) => "enabled",
        };

        let (detail, health) = check_hardware_device(name, &params, use_name);
        if health == Health::Fail && worst != Health::Fail {
            worst = Health::Fail;
        } else if health == Health::Warn && worst == Health::Ok {
            worst = Health::Warn;
        }
        details.push(detail);
    }

    // Validate hardware keys against URDF sensors (if [robot].description exists)
    if let Some(ref robot) = manifest.robot {
        if let Some(ref desc) = robot.description {
            let urdf_path = std::path::Path::new(desc);
            let sensors = crate::urdf::extract_sensors_from_urdf(urdf_path);
            if !sensors.is_empty() {
                let hw_keys: Vec<&str> = all_entries.keys().map(|s| s.as_str()).collect();
                let warnings = crate::urdf::validate_driver_keys(&hw_keys, &sensors);
                for warning in warnings {
                    details.push(format!("  ! {}", warning));
                    if worst == Health::Ok {
                        worst = Health::Warn;
                    }
                }
            }
        }
    }

    let summary = if worst == Health::Ok {
        format!("{} hardware node(s) reachable", details.len())
    } else {
        format!(
            "{} hardware node(s) checked, some issues found",
            details.len()
        )
    };

    CheckResult {
        category: "Hardware".into(),
        health: worst,
        summary,
        details,
    }
}

fn check_hardware_device(name: &str, params: &NodeParams, use_name: &str) -> (String, Health) {
    // Check serial port / device file
    if let Ok(port) = params.get::<String>("port") {
        if port.starts_with("/dev/") {
            return if Path::new(&port).exists() {
                (
                    format!("  {} '{}': {} found", "*".green(), name, port),
                    Health::Ok,
                )
            } else {
                (
                    format!("  {} '{}': {} not found", "x".red(), name, port),
                    Health::Fail,
                )
            };
        }
    }

    // Check I2C bus
    if let Ok(bus) = params.get::<String>("bus") {
        if bus.starts_with("i2c-") || bus.starts_with("/dev/i2c") {
            let path = if bus.starts_with("/dev/") {
                bus.clone()
            } else {
                format!("/dev/{}", bus)
            };
            return if Path::new(&path).exists() {
                (
                    format!("  {} '{}': {} found", "*".green(), name, path),
                    Health::Ok,
                )
            } else {
                (
                    format!("  {} '{}': {} not found", "x".red(), name, path),
                    Health::Fail,
                )
            };
        }
    }

    // Check network address
    if let Ok(address) = params.get::<String>("address") {
        if address.contains('.') || address.contains(':') {
            let addr_str = if address.contains(':') {
                address.clone()
            } else {
                format!("{}:80", address)
            };
            if let Ok(addr) = addr_str.parse::<std::net::SocketAddr>() {
                return match TcpStream::connect_timeout(&addr, Duration::from_secs(2)) {
                    Ok(_) => (
                        format!("  {} '{}': {} reachable", "*".green(), name, address),
                        Health::Ok,
                    ),
                    Err(_) => (
                        format!("  {} '{}': {} unreachable", "!".yellow(), name, address),
                        Health::Warn,
                    ),
                };
            }
        }
    }

    // No checkable params — report node type
    (
        format!("  - '{}': use={} (no device path to check)", name, use_name),
        Health::Ok,
    )
}

// ═══════════════════════════════════════════════════════════════════════════
// Installation check — HORUS's own install, `horus doctor` (always runs)
// ═══════════════════════════════════════════════════════════════════════════

/// Which of `find_horus_source_dir`'s branches produced the tree it handed
/// back, inferred from the path: only its cache branches end in `horus@<ver>`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SourceOrigin {
    /// `HORUS_SOURCE`, or one of the fixed development locations. Whatever is
    /// checked out there is what user projects are compiled against.
    Checkout,
    /// `<cache>/horus@<this CLI's version>` — the tree the installer laid down
    /// for this exact binary.
    CacheExact,
    /// Some other `<cache>/horus@*`. run_rust.rs:1069-1081 accepts any cached
    /// tree once the exact one is gone, so a CLI can end up building against a
    /// version nothing ever asked for, and nothing says so.
    CacheFallback,
}

/// The two halves of an install, gathered so the verdict can be graded without
/// a checkout on disk — the case that shipped (two trees both calling
/// themselves 0.4.0, 93 commits apart) cannot be reproduced on a machine that
/// installed correctly.
struct SourceFacts {
    cli_version: String,
    cli_topic_version: u32,
    source: PathBuf,
    source_version: Option<String>,
    source_topic_version: Option<u32>,
    origin: SourceOrigin,
}

/// Report the install HORUS is running out of, and whether its halves agree.
///
/// install.sh names `horus doctor` as the post-install check, but doctor used
/// to inspect everything except HORUS: a CLI whose binary and source tree came
/// from different refs printed a full row of green ticks while its own nodes
/// could not read each other's topics. Everything here is a comparison rather
/// than a presence test, because every value involved looked right on its own.
fn check_installation() -> CheckResult {
    let cli_version = crate::version::get_cli_version();
    let mut details = Vec::new();
    let mut fragments = vec![cli_version.to_string()];
    let mut health = Health::Ok;

    // ── The binary that is running ──────────────────────────────────────
    match std::env::current_exe() {
        Ok(exe) => details.push(format!("running: {} ({})", exe.display(), cli_version)),
        Err(e) => details.push(format!("running: {cli_version} (current_exe failed: {e})")),
    }

    // ── Every horus on PATH, not just the one that won ──────────────────
    //
    // A ~/.local/bin/horus left by an old install and a ~/.cargo/bin/horus from
    // a new one both answer to `horus`; PATH order picks the winner and says
    // nothing. The user then reads the new version out of the release notes and
    // runs the old binary.
    let binaries = horus_binaries_on_path();
    if binaries.is_empty() {
        details.push(
            "on PATH: no horus — this binary was invoked by path, not through PATH".to_string(),
        );
    }
    for (index, (path, reported)) in binaries.iter().enumerate() {
        let version = reported.as_deref().unwrap_or("did not answer --version");
        let position = if index == 0 {
            "first on PATH — this is the one `horus` runs"
        } else {
            "shadowed"
        };
        details.push(format!(
            "on PATH: {} — {version} ({position})",
            path.display()
        ));
    }
    if let Some(conflict) = path_conflict(&binaries) {
        health = health.worst(Health::Warn);
        details.push(
            "PATH order alone decides which of these `horus` means; delete the stale one \
             or reorder PATH"
                .to_string(),
        );
        fragments.push(conflict);
    }

    // ── What the install recorded about itself ──────────────────────────
    //
    // Absence is not an error: nothing wrote either file between v0.2.0 and the
    // change that added install_manifest.toml, so a large cohort of working
    // installs has neither. Say "unknown" and keep going.
    let manifest = crate::version::get_install_manifest();
    let legacy_version = crate::version::get_installed_version().ok().flatten();
    match &manifest {
        Some(record) => {
            details.push("install record: ~/.horus/install_manifest.toml".to_string());
            details.push(format!(
                "  installed version: {}",
                unknown(record.version.as_deref())
            ));
            details.push(format!("  tag: {}", unknown(record.tag.as_deref())));
            details.push(format!("  commit: {}", unknown(record.commit.as_deref())));
            details.push(format!(
                "  install method: {}",
                unknown(record.install_method.as_deref())
            ));
            details.push(format!(
                "  source tree it points at: {}",
                unknown(record.source_dir.as_deref().and_then(Path::to_str))
            ));
        }
        None => {
            details.push(
                "install record: none — ~/.horus/install_manifest.toml is absent, which is \
                 normal for an install made before install.sh wrote it"
                    .to_string(),
            );
            details.push(format!(
                "  installed version: {} (~/.horus/installed_version)",
                unknown(legacy_version.as_deref())
            ));
            details.push("  tag: unknown".to_string());
            details.push("  commit: unknown".to_string());
            details.push("  install method: unknown".to_string());
            details.push("  source tree it points at: unknown".to_string());
            if legacy_version.is_none() {
                fragments.push("no install record".to_string());
            }
        }
    }

    // ── Tag coherence: the CLI against the tree it builds projects with ──
    let mut claimed: Vec<PathBuf> = manifest
        .as_ref()
        .and_then(|record| record.source_dir.clone())
        .into_iter()
        .collect();
    match crate::commands::run::find_horus_source_dir() {
        Ok(source) => {
            let facts = SourceFacts {
                cli_version: cli_version.to_string(),
                cli_topic_version: crate::version::CLI_TOPIC_VERSION,
                source_version: crate_version_in(&source),
                source_topic_version: topic_version_in(&source),
                origin: source_origin(&source, cli_version),
                source,
            };
            let (source_health, summary, mut source_details) = grade_source_tree(&facts);
            health = health.worst(source_health);
            fragments.push(summary);
            details.append(&mut source_details);
            claimed.push(facts.source);
        }
        Err(_) => {
            // Not fatal for a Python-only user, but `horus run` on a Rust
            // project path-depends on this tree (cargo_gen.rs:114), so it is
            // the difference between building and not.
            health = health.worst(Health::Warn);
            fragments.push("no source tree".to_string());
            details.push(
                "source tree: not found in any location find_horus_source_dir() consults — \
                 Rust builds cannot resolve horus_core; run `horus self update` or reinstall"
                    .to_string(),
            );
        }
    }

    // ── Cached source trees nothing claims ──────────────────────────────
    let roots = source_cache_roots();
    for root in &roots {
        claimed.push(root.join(format!("horus@{cli_version}")));
    }
    let orphans = orphaned_source_caches_in(&roots, &claimed);
    if !orphans.is_empty() {
        let total: u64 = orphans.iter().map(|(_, size)| size).sum();
        fragments.push(format!(
            "{} in {} orphaned source cache{}",
            format_bytes(total),
            orphans.len(),
            if orphans.len() == 1 { "" } else { "s" }
        ));
        for (path, size) in &orphans {
            details.push(format!(
                "orphaned cache: {} ({}) — no installed version claims it; `rm -rf {}` \
                 reclaims the space",
                path.display(),
                format_bytes(*size),
                path.display()
            ));
        }
    }

    CheckResult {
        category: "Installation".to_string(),
        health,
        summary: fragments.join(" · "),
        details,
    }
}

/// "unknown" for a field the install record does not carry, so a partial record
/// degrades one line instead of failing the check.
fn unknown(value: Option<&str>) -> &str {
    value.unwrap_or("unknown")
}

/// Every `horus` on PATH, in PATH order, with what each answers to `--version`.
///
/// Deduplicated by canonical path, so a ~/.local/bin/horus that is a symlink to
/// the ~/.cargo/bin one is one install rather than a reported conflict.
fn horus_binaries_on_path() -> Vec<(PathBuf, Option<String>)> {
    let mut seen = std::collections::HashSet::new();
    all_on_path("horus")
        .into_iter()
        .filter(|path| seen.insert(std::fs::canonicalize(path).unwrap_or_else(|_| path.clone())))
        .map(|path| {
            let version = dispatch::tool_version(&path.to_string_lossy());
            (path, version)
        })
        .collect()
}

/// The summary line for a PATH holding more than one HORUS, or `None` when they
/// agree — two copies of the same version shadow each other harmlessly.
fn path_conflict(binaries: &[(PathBuf, Option<String>)]) -> Option<String> {
    let versions: Vec<&str> = binaries
        .iter()
        .map(|(_, reported)| reported.as_deref().map_or("unknown", version_token))
        .collect();
    let winner = *versions.first()?;
    let mut shadowed: Vec<&str> = Vec::new();
    for version in versions.iter().skip(1).copied() {
        if version != winner && !shadowed.contains(&version) {
            shadowed.push(version);
        }
    }
    if shadowed.is_empty() {
        return None;
    }
    Some(format!(
        "{} horus on PATH: {winner} shadows {}",
        binaries.len(),
        shadowed.join(", ")
    ))
}

/// The bare version out of a `--version` line: "horus 0.4.0" -> "0.4.0".
fn version_token(line: &str) -> &str {
    line.split_whitespace().last().unwrap_or(line)
}

/// The shm wire-format version a source tree speaks.
///
/// Grepped, not linked against: `TOPIC_VERSION` is `pub(crate)` in horus_core,
/// and the tree being read is a *different* checkout from the one this binary
/// was compiled from — that difference is the entire point of the comparison.
/// install.sh:803 and upgrade.rs:717 parse the same line the same way;
/// upgrade's copy is private to that module.
fn topic_version_in(source: &Path) -> Option<u32> {
    let header = source.join("horus_core/src/communication/topic/header.rs");
    let text = std::fs::read_to_string(header).ok()?;
    let line = text
        .lines()
        .find(|l| l.contains("const TOPIC_VERSION") && l.contains('='))?;
    line.rsplit('=')
        .next()?
        .trim()
        .trim_end_matches(';')
        .trim()
        .parse()
        .ok()
}

/// The version a source tree declares for itself.
///
/// horus_manager's is asked for first because it is the number `CARGO_PKG_VERSION`
/// bakes into this binary, so it is the like-for-like comparison. horus_core's
/// is what install.sh:462 reads, and is the fallback for a tree whose
/// horus_manager inherits its version from the workspace.
fn crate_version_in(source: &Path) -> Option<String> {
    ["horus_manager", "horus_core"].iter().find_map(|package| {
        let text = std::fs::read_to_string(source.join(package).join("Cargo.toml")).ok()?;
        text.lines()
            .find(|line| line.trim_start().starts_with("version"))
            .and_then(|line| line.split('"').nth(1))
            .map(str::to_string)
    })
}

fn source_origin(source: &Path, cli_version: &str) -> SourceOrigin {
    let Some(leaf) = source.file_name().and_then(|name| name.to_str()) else {
        return SourceOrigin::Checkout;
    };
    if !leaf.starts_with("horus@") {
        SourceOrigin::Checkout
    } else if leaf == format!("horus@{cli_version}") {
        SourceOrigin::CacheExact
    } else {
        SourceOrigin::CacheFallback
    }
}

/// Grade the CLI against the tree user projects will be compiled against.
///
/// Pure, because the failure it exists to catch — RC1: a binary from a release
/// tag and a source tree from main HEAD, both printing 0.4.0, with topic
/// versions 3 and 4 — cannot be staged on a machine that installed correctly.
/// Returns (verdict, summary fragment, detail lines).
fn grade_source_tree(facts: &SourceFacts) -> (Health, String, Vec<String>) {
    let leaf = facts
        .source
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("source tree");
    let mut details = vec![
        format!("source tree: {}", facts.source.display()),
        format!(
            "  version: {} (this CLI: {})",
            unknown(facts.source_version.as_deref()),
            facts.cli_version
        ),
        format!(
            "  topic_version: {} (this CLI: {})",
            facts
                .source_topic_version
                .map_or("unknown".to_string(), |v| v.to_string()),
            facts.cli_topic_version
        ),
    ];

    if facts.origin == SourceOrigin::CacheFallback {
        details.push(format!(
            "  there is no horus@{} in the cache, so find_horus_source_dir() fell back to \
             this tree (run_rust.rs:1069) — the CLI is building against a version it never \
             asked for",
            facts.cli_version
        ));
    }

    // The ABI, checked before the version string, because it is the half that
    // actually breaks: two trees can share a version and still refuse each
    // other's shared memory, which is exactly what shipped.
    if let Some(topic) = facts.source_topic_version {
        if topic != facts.cli_topic_version {
            details.push(format!(
                "  this CLI writes topic headers at v{} and libraries built from this tree \
                 read v{}; nodes cannot attach to each other's shared memory. \
                 `horus self update` puts both halves back on one tag.",
                facts.cli_topic_version, topic
            ));
            return (
                Health::Fail,
                format!(
                    "topic ABI break: CLI v{} vs source tree v{} — run `horus self update`",
                    facts.cli_topic_version, topic
                ),
                details,
            );
        }
    }

    if facts.origin == SourceOrigin::CacheFallback {
        return (
            Health::Warn,
            format!(
                "source tree {leaf} is not this CLI's {} — run `horus self update`",
                facts.cli_version
            ),
            details,
        );
    }

    if matches!(&facts.source_version, Some(version) if version != &facts.cli_version) {
        let version = facts.source_version.as_deref().unwrap_or("unknown");
        details.push(
            "  the CLI and the tree it builds against came from different refs; that is the \
             shape of the shm break above, even when topic_version happens to agree today"
                .to_string(),
        );
        return (
            Health::Warn,
            format!(
                "source tree says {version}, CLI says {} — run `horus self update`",
                facts.cli_version
            ),
            details,
        );
    }

    if facts.source_topic_version.is_none() {
        return (
            Health::Ok,
            format!("source tree {leaf} (topic_version unreadable)"),
            details,
        );
    }

    (
        Health::Ok,
        format!("source tree {leaf} (topic v{})", facts.cli_topic_version),
        details,
    )
}

/// The cache roots `find_horus_source_dir` searches, in its order.
///
/// Both, because the codebase has historically written to both: XDG
/// (~/.cache/horus) and the ~/.horus/cache that install.sh and `horus clean`
/// manage. Listing only one would call a live tree an orphan.
fn source_cache_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();
    if let Ok(xdg) = crate::paths::cache_dir() {
        roots.push(xdg);
    }
    if let Ok(home) = crate::paths::home_dir() {
        let legacy = home.join(".horus/cache");
        if !roots.contains(&legacy) {
            roots.push(legacy);
        }
    }
    roots
}

/// Cached `horus@*` trees that no install claims, with what they cost.
///
/// Every upgrade leaves the previous tree behind and nothing ever mentions it;
/// on the machine this check was written for that was 727 MB of a version the
/// user had stopped running months earlier.
fn orphaned_source_caches_in(roots: &[PathBuf], claimed: &[PathBuf]) -> Vec<(PathBuf, u64)> {
    let mut orphans = Vec::new();
    for root in roots {
        let Ok(entries) = std::fs::read_dir(root) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let is_source_cache = path
                .file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.starts_with("horus@"));
            if !is_source_cache || !path.is_dir() {
                continue;
            }
            if claimed.iter().any(|c| same_path(c, &path)) {
                continue;
            }
            orphans.push((path.clone(), dir_size(&path)));
        }
    }
    orphans.sort();
    orphans
}

/// Compare two paths by what they resolve to, so a symlinked cache root does
/// not make a claimed tree look unclaimed.
fn same_path(a: &Path, b: &Path) -> bool {
    let resolve = |p: &Path| std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf());
    resolve(a) == resolve(b)
}

// ═══════════════════════════════════════════════════════════════════════════
// Network check — `horus doctor` (always runs)
// ═══════════════════════════════════════════════════════════════════════════

fn check_network() -> CheckResult {
    let mut details = Vec::new();
    let mut health = Health::Ok;

    // Check 1: remote presence files exist (peers discovered)
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    let remote_count = std::fs::read_dir(&nodes_dir)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok())
                .filter(|e| e.file_name().to_string_lossy().starts_with("remote_"))
                .count()
        })
        .unwrap_or(0);

    if remote_count > 0 {
        details.push(format!("{} remote host(s) discovered", remote_count));
    } else {
        details.push("No remote peers (single-machine mode)".to_string());
    }

    // Check 2: the namespace this process is in
    //
    // Ask the accessor, not the environment. `shm_namespace` sanitises the
    // variable and derives a namespace when it is unset, so reading the raw
    // value made `horus doctor` report "Namespace: default" while the tree it
    // had just counted namespaces in was a different one.
    let namespace = horus_sys::shm::shm_namespace();
    details.push(format!("Namespace: {}", namespace));

    // Check 3: horus_net enabled check
    let net_enabled = std::env::var("HORUS_NET")
        .map(|v| v == "1" || v == "true")
        .unwrap_or(false);
    let no_net = std::env::var("HORUS_NO_NETWORK")
        .map(|v| v == "1" || v == "true")
        .unwrap_or(false);

    if no_net {
        details.push("horus_net: DISABLED (HORUS_NO_NETWORK=1)".to_string());
    } else if net_enabled {
        details.push("horus_net: enabled".to_string());
    } else {
        details.push("horus_net: available (use --net or HORUS_NET=1 to enable)".to_string());
    }

    // Check 4: multicast reachability (basic socket test)
    match std::net::UdpSocket::bind("0.0.0.0:0") {
        Ok(_) => {
            details.push("UDP sockets: available".to_string());
        }
        Err(e) => {
            details.push(format!("UDP sockets: FAILED ({})", e));
            health = Health::Warn;
        }
    }

    // Check 5: the package registry `horus search`, `horus add` and
    // `horus install` all talk to.
    //
    // None of the probes above touch it, so a registry that is refusing every
    // request is indistinguishable from a search with no matches: the endpoint
    // has been answering 503 "service suspended" while `horus search` printed
    // an empty list and exited 0. Offline is a normal state, not a fault, so
    // this is short, single-shot and never fatal.
    match crate::config::registry_url() {
        // No registry configured is the shipped default while the public one is
        // down (#173). Reported as information, not a fault: nothing is broken
        // on this machine, and `horus doctor` warning about it on every run
        // would train users to ignore the Network section.
        None => details.push(
            "Registry: none configured — `horus search` returns local results only, \
             and `horus add`/`publish` are unavailable. Set HORUS_REGISTRY_URL to \
             use a self-hosted registry."
                .to_string(),
        ),
        Some(registry_url) if no_net => details.push(format!(
            "Registry: {registry_url} not probed (HORUS_NO_NETWORK=1)"
        )),
        Some(registry_url) => match probe_registry(&registry_url) {
            RegistryProbe::Status(code) if (200..300).contains(&code) => {
                details.push(format!("Registry: {registry_url} answered {code}"));
            }
            RegistryProbe::Status(code) => {
                details.push(format!(
                    "Registry: {registry_url} answered HTTP {code} — `horus search` will come \
                     back empty and `horus add` will fail until it recovers"
                ));
                health = Health::Warn;
            }
            RegistryProbe::Unreachable(error) => {
                details.push(format!(
                    "Registry: {registry_url} unreachable ({error}) — expected when offline"
                ));
            }
        },
    }

    let summary = if remote_count > 0 {
        format!("{} remote peer(s)", remote_count)
    } else {
        "Single-machine mode".to_string()
    };

    CheckResult {
        category: "Network".to_string(),
        health,
        summary,
        details,
    }
}

/// What the registry said, or why it said nothing.
enum RegistryProbe {
    /// It answered. The code is reported verbatim, because "the registry is up"
    /// and "the registry is returning 503" are the two cases this exists to
    /// tell apart.
    Status(u16),
    /// No answer at all: no route, DNS failure, timeout. Normal offline.
    Unreachable(String),
}

/// Ask the registry the same question `horus search` asks, briefly.
///
/// The search endpoint rather than the bare host, so the status reported is the
/// one the command the user is about to run would get. Short timeouts and no
/// retry: `horus doctor` has to finish on a machine with no network at all.
fn probe_registry(base_url: &str) -> RegistryProbe {
    let client = match reqwest::blocking::Client::builder()
        .connect_timeout(Duration::from_secs(2))
        .timeout(Duration::from_secs(3))
        .user_agent("horus-doctor")
        .build()
    {
        Ok(client) => client,
        Err(e) => return RegistryProbe::Unreachable(e.to_string()),
    };

    let url = format!("{}/api/packages/search", base_url.trim_end_matches('/'));
    match client.get(url).query(&[("q", "horus")]).send() {
        Ok(response) => RegistryProbe::Status(response.status().as_u16()),
        Err(e) => RegistryProbe::Unreachable(e.to_string()),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RT Readiness Report — `horus doctor --rt`
// ═══════════════════════════════════════════════════════════════════════════

/// Run the RT readiness report: system audit + jitter benchmark + IPC benchmark.
pub fn run_rt_report() -> Result<()> {
    println!(
        "{}",
        "Running RT Readiness Report (3-second benchmark)...\n".bold()
    );
    let report = horus_core::scheduling::rt_report::RtReport::generate(Duration::from_secs(3));
    report.print();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use std::fs;
    use std::path::PathBuf;

    /// Helper to build a minimal valid HorusManifest for testing.
    fn make_manifest(name: &str) -> crate::manifest::HorusManifest {
        crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: name.to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            rust: None,
            hooks: Default::default(),
            network: None,
        }
    }

    // ── Health enum ─────────────────────────────────────────────────────

    #[test]
    fn health_ok_icon_is_green_check() {
        let icon = Health::Ok.icon();
        assert!(icon.to_string().contains("*"));
    }

    #[test]
    fn health_warn_icon_is_yellow_bang() {
        let icon = Health::Warn.icon();
        assert!(icon.to_string().contains("!"));
    }

    #[test]
    fn health_fail_icon_is_red_x() {
        let icon = Health::Fail.icon();
        assert!(icon.to_string().contains("x"));
    }

    #[test]
    fn health_eq_same() {
        assert_eq!(Health::Ok, Health::Ok);
        assert_eq!(Health::Warn, Health::Warn);
        assert_eq!(Health::Fail, Health::Fail);
    }

    #[test]
    fn health_ne_different() {
        assert_ne!(Health::Ok, Health::Warn);
        assert_ne!(Health::Ok, Health::Fail);
        assert_ne!(Health::Warn, Health::Fail);
    }

    #[test]
    fn health_clone() {
        let h = Health::Warn;
        let h2 = h;
        assert_eq!(h, h2);
    }

    #[test]
    fn health_debug() {
        let dbg = format!("{:?}", Health::Ok);
        assert_eq!(dbg, "Ok");
        assert_eq!(format!("{:?}", Health::Warn), "Warn");
        assert_eq!(format!("{:?}", Health::Fail), "Fail");
    }

    // ── format_bytes ────────────────────────────────────────────────────

    #[test]
    fn format_bytes_zero() {
        assert_eq!(format_bytes(0), "0 B");
    }

    #[test]
    fn format_bytes_small() {
        assert_eq!(format_bytes(512), "512 B");
    }

    #[test]
    fn format_bytes_exactly_1kb() {
        assert_eq!(format_bytes(1024), "1.0 KB");
    }

    #[test]
    fn format_bytes_kilobytes() {
        assert_eq!(format_bytes(1536), "1.5 KB");
    }

    #[test]
    fn format_bytes_megabytes() {
        assert_eq!(format_bytes(1_048_576), "1.0 MB");
    }

    #[test]
    fn format_bytes_megabytes_fractional() {
        assert_eq!(format_bytes(5_242_880), "5.0 MB");
    }

    #[test]
    fn format_bytes_gigabytes() {
        assert_eq!(format_bytes(1_073_741_824), "1.00 GB");
    }

    #[test]
    fn format_bytes_gigabytes_large() {
        assert_eq!(format_bytes(10_737_418_240), "10.00 GB");
    }

    #[test]
    fn format_bytes_boundary_below_kb() {
        assert_eq!(format_bytes(1023), "1023 B");
    }

    #[test]
    fn format_bytes_boundary_below_mb() {
        let val = 1_048_575; // 1MB - 1
        let result = format_bytes(val);
        assert!(result.contains("KB"));
    }

    #[test]
    fn format_bytes_boundary_below_gb() {
        let val = 1_073_741_823; // 1GB - 1
        let result = format_bytes(val);
        assert!(result.contains("MB"));
    }

    // ── count_items ─────────────────────────────────────────────────────

    #[test]
    fn count_items_empty_dir() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(count_items(tmp.path()), 0);
    }

    #[test]
    fn count_items_with_files() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("a.txt"), "hello").unwrap();
        fs::write(tmp.path().join("b.txt"), "world").unwrap();
        assert_eq!(count_items(tmp.path()), 2);
    }

    #[test]
    fn count_items_with_dirs_and_files() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("a.txt"), "").unwrap();
        fs::create_dir(tmp.path().join("subdir")).unwrap();
        assert_eq!(count_items(tmp.path()), 2);
    }

    #[test]
    fn count_items_nonexistent() {
        assert_eq!(count_items(Path::new("/nonexistent/path/99999")), 0);
    }

    // ── dir_size ────────────────────────────────────────────────────────

    #[test]
    fn dir_size_empty() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(dir_size(tmp.path()), 0);
    }

    #[test]
    fn dir_size_single_file() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("data.bin"), vec![0u8; 256]).unwrap();
        assert_eq!(dir_size(tmp.path()), 256);
    }

    #[test]
    fn dir_size_nested() {
        let tmp = tempfile::tempdir().unwrap();
        let sub = tmp.path().join("a").join("b");
        fs::create_dir_all(&sub).unwrap();
        fs::write(sub.join("file.dat"), vec![1u8; 100]).unwrap();
        fs::write(tmp.path().join("root.dat"), vec![2u8; 50]).unwrap();
        assert_eq!(dir_size(tmp.path()), 150);
    }

    #[test]
    fn dir_size_nonexistent_returns_zero() {
        assert_eq!(dir_size(Path::new("/nonexistent/path/99999")), 0);
    }

    // ── check_toolchains ────────────────────────────────────────────────

    #[test]
    fn semver_is_compared_numerically_not_lexically() {
        // "1.100" < "1.9" as strings. The whole point of parsing.
        assert!(
            super::parse_semver("rustc 1.100.0").unwrap()
                > super::parse_semver("rustc 1.9.0").unwrap()
        );
        assert!(
            super::parse_semver("rustc 1.90.0").unwrap()
                < super::parse_semver("rustc 1.92.0").unwrap()
        );
        assert_eq!(
            super::parse_semver("rustc 1.97.1 (8bab26f4f 2026-07-14)"),
            Some((1, 97, 1))
        );
        assert_eq!(super::parse_semver("cmake version 4.2.3"), Some((4, 2, 3)));
        assert_eq!(super::parse_semver("Python 3.14"), Some((3, 14, 0)));
    }

    /// A pre-release suffix must not defeat the parse.
    #[test]
    fn semver_ignores_a_prerelease_suffix() {
        assert_eq!(
            super::parse_semver("rustc 1.98.0-nightly"),
            Some((1, 98, 0))
        );
    }

    #[test]
    fn semver_returns_none_when_there_is_no_version() {
        assert_eq!(super::parse_semver("not found"), None);
        assert_eq!(super::parse_semver(""), None);
    }

    /// The floor doctor grades against must come from cargo, so it cannot
    /// drift from `[workspace.package] rust-version`.
    #[test]
    fn the_minimum_rustc_is_the_declared_workspace_msrv() {
        let declared = super::MINIMUM_RUSTC.expect("horus_manager must declare rust-version");
        let root = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .parent()
                .unwrap()
                .join("Cargo.toml"),
        )
        .expect("read workspace manifest");
        let want = root
            .lines()
            .find_map(|l| l.trim().strip_prefix("rust-version = "))
            .map(|v| v.trim().trim_matches('"').to_string())
            .expect("workspace must declare rust-version");
        assert_eq!(
            declared, want,
            "horus_manager's rust-version must be inherited from the workspace"
        );
    }

    #[test]
    fn check_toolchains_returns_result() {
        let result = check_toolchains();
        assert_eq!(result.category, "Toolchains");

        // The summary used to be a bare fraction ("3/8 tools found"), which is
        // what let a machine missing five tools render as a success. It now
        // names languages and what each is missing, so assert on that shape
        // instead — see `toolchain_grading_tests` for the rules themselves.
        assert!(
            ["Rust", "Python", "C++"]
                .iter()
                .any(|l| result.summary.contains(l)),
            "summary should name languages, got: {}",
            result.summary
        );
        assert!(
            !result.summary.contains("tools found"),
            "the bare fraction is what made 3/8 look like success: {}",
            result.summary
        );
        assert!(!result.details.is_empty());
    }

    #[test]
    fn check_toolchains_details_mention_tool_names() {
        let result = check_toolchains();
        let combined: String = result.details.join("\n");
        assert!(combined.contains("cargo"), "should list cargo");
        assert!(combined.contains("rustc"), "should list rustc");
        assert!(combined.contains("python3"), "should list python3");
    }

    // ── check_manifest ──────────────────────────────────────────────────

    #[test]
    fn check_manifest_no_horus_toml() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: false,
            manifest: None,
            manifest_error: None,
        };
        let result = check_manifest(&ctx);
        assert_eq!(result.category, "Manifest");
        assert_eq!(result.health, Health::Warn);
        assert!(result.summary.contains("No horus.toml"));
    }

    /// A manifest that exists but did not load must report *why*, and the
    /// reason must survive into `details`. Previously every cause produced the
    /// same "Failed to parse horus.toml" with `details: vec![]` — including a
    /// file that had parsed fine and merely omitted a key.
    #[test]
    fn check_manifest_reports_the_reason_a_manifest_failed_to_load() {
        let err = crate::manifest::ManifestError {
            kind: crate::manifest::ManifestErrorKind::MissingField {
                field: "name".into(),
                table: Some("[package]".into()),
            },
            file: PathBuf::from("/tmp/fake/horus.toml"),
            line: Some(1),
            col: Some(1),
            detail: "missing field `name`".into(),
        };
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: None,
            manifest_error: Some(err),
        };
        let result = check_manifest(&ctx);
        assert_eq!(result.health, Health::Fail);
        assert!(
            !result.summary.contains("Failed to parse"),
            "the file parsed; only a key is absent: {}",
            result.summary
        );
        assert!(result.summary.contains("missing a required field"));
        assert!(
            result.details.iter().any(|d| d.contains("`name`")),
            "the field name must reach the user: {:?}",
            result.details
        );
        assert!(
            result.details.iter().any(|d| d.contains("1:1")),
            "the position must reach the user: {:?}",
            result.details
        );
    }

    /// The same check with no recorded cause — it must not invent one.
    #[test]
    fn check_manifest_without_a_recorded_error_does_not_guess() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: None,
            manifest_error: None,
        };
        let result = check_manifest(&ctx);
        assert_eq!(result.health, Health::Fail);
        assert!(!result.summary.contains("parse"), "{}", result.summary);
    }

    #[test]
    fn check_manifest_valid_manifest() {
        let manifest = make_manifest("test-proj");
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
            manifest_error: None,
        };
        let result = check_manifest(&ctx);
        assert_eq!(result.category, "Manifest");
        assert!(result.health == Health::Ok || result.health == Health::Warn);
    }

    #[test]
    fn check_manifest_valid_has_details() {
        let manifest = make_manifest("mybot");
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
            manifest_error: None,
        };
        let result = check_manifest(&ctx);
        let details = result.details.join("\n");
        assert!(
            details.contains("mybot"),
            "details should include package name"
        );
    }

    // ── check_shm ───────────────────────────────────────────────────────

    #[test]
    fn check_shm_returns_result() {
        let result = check_shm();
        assert_eq!(result.category, "Shared Memory");
        let shm_parent = horus_sys::shm::shm_parent_dir();
        if shm_parent.exists() {
            assert_eq!(result.health, Health::Ok);
            assert!(result.summary.contains("available"));
        } else {
            assert_eq!(result.health, Health::Warn);
        }
    }

    // ── check_plugins ───────────────────────────────────────────────────

    #[test]
    fn check_plugins_returns_result() {
        let result = check_plugins();
        assert_eq!(result.category, "Plugins");
        assert_eq!(result.health, Health::Ok);
        assert!(result.summary.contains("global"));
        assert!(result.summary.contains("local"));
    }

    // ── check_disk ──────────────────────────────────────────────────────

    #[test]
    fn check_disk_returns_result() {
        let _lock = crate::CWD_LOCK.lock();
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = check_disk();
        assert_eq!(result.category, "Disk");
        assert_eq!(result.health, Health::Ok);
        assert!(result.summary.contains("No .horus/"));

        std::env::set_current_dir(old_dir).unwrap();
    }

    #[test]
    fn check_disk_with_horus_dir() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        fs::create_dir(tmp.path().join(".horus")).unwrap();
        fs::write(tmp.path().join(".horus/test.dat"), vec![0u8; 1024]).unwrap();

        let result = check_disk();
        assert_eq!(result.category, "Disk");
        assert_eq!(result.health, Health::Ok);

        std::env::set_current_dir(old_dir).unwrap();
    }

    // ── check_languages ─────────────────────────────────────────────────

    #[test]
    fn check_languages_empty() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: false,
            manifest: None,
            manifest_error: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.category, "Languages");
        assert_eq!(result.health, Health::Warn);
        assert!(result.summary.contains("No languages"));
    }

    #[test]
    fn check_languages_rust_only() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Rust],
            has_horus_toml: false,
            manifest: None,
            manifest_error: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
    }

    #[test]
    fn check_languages_python_only() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Python],
            has_horus_toml: false,
            manifest: None,
            manifest_error: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
    }

    #[test]
    fn check_languages_mixed() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![
                crate::manifest::Language::Rust,
                crate::manifest::Language::Python,
            ],
            has_horus_toml: false,
            manifest: None,
            manifest_error: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
        let summary = result.summary.to_lowercase();
        assert!(summary.contains("rust") || summary.contains("python"));
    }

    // ── print_json ──────────────────────────────────────────────────────

    #[test]
    fn print_json_produces_valid_json() {
        let results = vec![
            CheckResult {
                category: "Test".to_string(),
                health: Health::Ok,
                summary: "all good".to_string(),
                details: vec!["detail1".to_string()],
            },
            CheckResult {
                category: "Another".to_string(),
                health: Health::Fail,
                summary: "broken".to_string(),
                details: vec![],
            },
        ];
        print_json(&results);
    }

    #[test]
    fn print_json_empty_results() {
        print_json(&[]);
    }

    // ── print_summary ───────────────────────────────────────────────────

    #[test]
    fn print_summary_no_issues() {
        let results = vec![CheckResult {
            category: "Check1".to_string(),
            health: Health::Ok,
            summary: "fine".to_string(),
            details: vec![],
        }];
        print_summary(&results, false);
    }

    #[test]
    fn print_summary_verbose_shows_details() {
        let results = vec![CheckResult {
            category: "Check1".to_string(),
            health: Health::Warn,
            summary: "warning".to_string(),
            details: vec!["some detail".to_string()],
        }];
        print_summary(&results, true);
    }

    #[test]
    fn print_summary_mixed_health() {
        let results = vec![
            CheckResult {
                category: "Ok".to_string(),
                health: Health::Ok,
                summary: "fine".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "Warn".to_string(),
                health: Health::Warn,
                summary: "warning".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "Fail".to_string(),
                health: Health::Fail,
                summary: "broken".to_string(),
                details: vec![],
            },
        ];
        print_summary(&results, false);
    }

    #[test]
    fn print_summary_all_ok() {
        let results = vec![
            CheckResult {
                category: "A".to_string(),
                health: Health::Ok,
                summary: "ok".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "B".to_string(),
                health: Health::Ok,
                summary: "ok".to_string(),
                details: vec![],
            },
        ];
        print_summary(&results, false);
    }

    #[test]
    fn print_summary_warnings_only() {
        let results = vec![
            CheckResult {
                category: "A".to_string(),
                health: Health::Ok,
                summary: "ok".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "B".to_string(),
                health: Health::Warn,
                summary: "warn".to_string(),
                details: vec![],
            },
        ];
        print_summary(&results, false);
    }

    // ── CheckResult construction ────────────────────────────────────────

    #[test]
    fn check_result_fields() {
        let cr = CheckResult {
            category: "Cat".to_string(),
            health: Health::Ok,
            summary: "Sum".to_string(),
            details: vec!["d1".to_string(), "d2".to_string()],
        };
        assert_eq!(cr.category, "Cat");
        assert_eq!(cr.health, Health::Ok);
        assert_eq!(cr.summary, "Sum");
        assert_eq!(cr.details.len(), 2);
    }

    #[test]
    fn check_result_empty_details() {
        let cr = CheckResult {
            category: "X".to_string(),
            health: Health::Fail,
            summary: "bad".to_string(),
            details: vec![],
        };
        assert!(cr.details.is_empty());
    }

    // ── Edge cases / battle tests ───────────────────────────────────────

    #[test]
    fn format_bytes_max_u64() {
        let result = format_bytes(u64::MAX);
        assert!(result.contains("GB"), "huge value should be GB: {}", result);
    }

    #[test]
    fn format_bytes_one_byte() {
        assert_eq!(format_bytes(1), "1 B");
    }

    #[test]
    fn health_all_variants_covered() {
        let variants = [Health::Ok, Health::Warn, Health::Fail];
        for v in &variants {
            let _ = v.icon();
            let _ = format!("{:?}", v);
        }
    }

    #[test]
    fn check_manifest_with_dependencies() {
        let mut manifest = make_manifest("dep-proj");
        manifest.dependencies.insert(
            "serde".to_string(),
            crate::manifest::DependencyValue::Simple("1.0".to_string()),
        );
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
            manifest_error: None,
        };
        let result = check_manifest(&ctx);
        let details = result.details.join("\n");
        assert!(details.contains("dependencies: 1"));
    }

    #[test]
    fn count_items_deeply_nested() {
        let tmp = tempfile::tempdir().unwrap();
        let deep = tmp.path().join("a").join("b").join("c");
        fs::create_dir_all(&deep).unwrap();
        fs::write(deep.join("file.txt"), "data").unwrap();
        // count_items only counts direct children, not recursive
        assert_eq!(count_items(tmp.path()), 1); // just "a"
    }

    #[test]
    fn dir_size_many_small_files() {
        let tmp = tempfile::tempdir().unwrap();
        for i in 0..50 {
            fs::write(tmp.path().join(format!("f{}.txt", i)), "x").unwrap();
        }
        assert_eq!(dir_size(tmp.path()), 50);
    }

    #[test]
    fn check_disk_large_threshold() {
        let _lock = crate::CWD_LOCK.lock();
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = check_disk();
        assert_eq!(result.health, Health::Ok);

        std::env::set_current_dir(old_dir).unwrap();
    }

    #[test]
    fn check_languages_single_cpp() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Cpp],
            has_horus_toml: false,
            manifest: None,
            manifest_error: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
    }

    #[test]
    fn check_manifest_invalid_name() {
        // name "x" is too short (< 2 chars), should produce validation error or warning
        let manifest = make_manifest("x");
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
            manifest_error: None,
        };
        let result = check_manifest(&ctx);
        // Validation should catch the short name
        assert!(
            result.health == Health::Warn || result.health == Health::Fail,
            "short name should trigger warn or fail"
        );
    }

    #[test]
    fn check_manifest_with_drivers_and_scripts() {
        let mut manifest = make_manifest("robot-arm");
        manifest.drivers.insert(
            "lidar".to_string(),
            crate::manifest::DriverValue::Backend("rplidar".to_string()),
        );
        manifest
            .scripts
            .insert("build".to_string(), "cargo build".to_string());
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Rust],
            has_horus_toml: true,
            manifest: Some(manifest),
            manifest_error: None,
        };
        let result = check_manifest(&ctx);
        // Should parse fine
        assert!(result.health == Health::Ok || result.health == Health::Warn);
    }

    #[test]
    fn check_shm_category_name() {
        let result = check_shm();
        assert_eq!(result.category, "Shared Memory");
    }

    #[test]
    fn check_disk_category_name() {
        let _lock = crate::CWD_LOCK.lock();
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = check_disk();
        assert_eq!(result.category, "Disk");
        std::env::set_current_dir(old_dir).unwrap();
    }

    #[test]
    fn check_languages_all_supported() {
        // Test with all known language variants
        for lang in &[
            crate::manifest::Language::Rust,
            crate::manifest::Language::Python,
            crate::manifest::Language::Cpp,
        ] {
            let ctx = dispatch::ProjectContext {
                root: PathBuf::from("/tmp/fake"),
                languages: vec![*lang],
                has_horus_toml: false,
                manifest: None,
                manifest_error: None,
            };
            let result = check_languages(&ctx);
            assert_eq!(result.health, Health::Ok);
        }
    }

    #[test]
    fn print_json_single_result() {
        let results = vec![CheckResult {
            category: "Solo".to_string(),
            health: Health::Warn,
            summary: "needs attention".to_string(),
            details: vec!["fix this".to_string(), "and this".to_string()],
        }];
        print_json(&results);
    }

    #[test]
    fn check_result_many_details() {
        let cr = CheckResult {
            category: "Verbose".to_string(),
            health: Health::Ok,
            summary: "lots of info".to_string(),
            details: (0..100).map(|i| format!("detail {}", i)).collect(),
        };
        assert_eq!(cr.details.len(), 100);
    }

    #[test]
    fn format_bytes_exact_boundaries() {
        // Exactly at each boundary
        assert!(format_bytes(1024).contains("KB"));
        assert!(format_bytes(1_048_576).contains("MB"));
        assert!(format_bytes(1_073_741_824).contains("GB"));
    }

    // ── Driver device reachability tests ──────────────────────────────

    #[test]
    fn check_hardware_device_existing_path() {
        let mut map = std::collections::HashMap::new();
        map.insert("port".to_string(), toml::Value::String("/dev/null".into()));
        let params = horus_core::drivers::NodeParams::new(map);
        let (detail, health) = check_hardware_device("test", &params, "serial");
        assert_eq!(health, Health::Ok);
        assert!(detail.contains("found"), "detail: {}", detail);
    }

    #[test]
    fn check_hardware_device_missing_path() {
        let mut map = std::collections::HashMap::new();
        map.insert(
            "port".to_string(),
            toml::Value::String("/dev/nonexistent_xyz_test".into()),
        );
        let params = horus_core::drivers::NodeParams::new(map);
        let (detail, health) = check_hardware_device("test", &params, "serial");
        assert_eq!(health, Health::Fail);
        assert!(detail.contains("not found"), "detail: {}", detail);
    }

    #[test]
    fn check_hardware_device_i2c_bus() {
        let mut map = std::collections::HashMap::new();
        map.insert("bus".to_string(), toml::Value::String("i2c-99".into()));
        let params = horus_core::drivers::NodeParams::new(map);
        let (detail, health) = check_hardware_device("imu", &params, "mpu6050");
        // /dev/i2c-99 almost certainly doesn't exist
        assert_eq!(health, Health::Fail);
        assert!(detail.contains("i2c-99"), "detail: {}", detail);
    }

    #[test]
    fn check_hardware_device_no_checkable_params() {
        let params = horus_core::drivers::NodeParams::empty();
        let (detail, health) = check_hardware_device("mystery", &params, "MyDriver");
        assert_eq!(health, Health::Ok);
        assert!(detail.contains("no device path"), "detail: {}", detail);
    }

    #[test]
    fn check_hardware_device_legacy_type() {
        let params = horus_core::drivers::NodeParams::empty();
        let (detail, health) = check_hardware_device("cam", &params, "legacy");
        assert_eq!(health, Health::Ok);
        assert!(detail.contains("legacy"), "detail: {}", detail);
    }

    #[test]
    fn check_hardware_device_network_unreachable() {
        let mut map = std::collections::HashMap::new();
        map.insert(
            "address".to_string(),
            toml::Value::String("192.0.2.1:9999".into()),
        );
        let params = horus_core::drivers::NodeParams::new(map);
        let (detail, health) = check_hardware_device("lidar", &params, "velodyne");
        assert_eq!(health, Health::Warn);
        assert!(detail.contains("unreachable"), "detail: {}", detail);
    }

    // ── CheckResult display & description ──────────────────────────────

    #[test]
    fn test_doctor_check_result_display() {
        let cases = vec![
            CheckResult {
                category: "Toolchains".to_string(),
                health: Health::Ok,
                summary: "8/8 tools found".to_string(),
                details: vec!["cargo: 1.77.0".to_string()],
            },
            CheckResult {
                category: "Manifest".to_string(),
                health: Health::Warn,
                summary: "horus.toml has warnings".to_string(),
                details: vec!["missing license field".to_string()],
            },
            CheckResult {
                category: "System Deps".to_string(),
                health: Health::Fail,
                summary: "2 missing dependencies".to_string(),
                details: vec![
                    "libssl-dev: not found".to_string(),
                    "cmake: not found".to_string(),
                ],
            },
        ];

        for cr in &cases {
            assert!(!cr.category.is_empty(), "category must be non-empty");
            assert!(
                !cr.summary.is_empty(),
                "summary must be non-empty (acts as display)"
            );
            // details act as the description — at least one entry for each case above
            assert!(
                !cr.details.is_empty(),
                "details/description must be non-empty for category '{}'",
                cr.category
            );
            // Verify icon rendering doesn't panic
            let _icon = cr.health.icon();
        }
    }

    // ── All check category names unique ────────────────────────────────

    #[test]
    fn test_doctor_all_check_names_unique() {
        // Collect the category names from all individual check functions.
        // We cannot call check_dep_sources/check_manifest without a real context,
        // but the categories are deterministic string literals, so we gather
        // the ones we can call plus the known constant categories.
        let callable_results = [check_toolchains(), check_shm(), check_plugins()];

        // Known categories from code inspection (check_manifest, check_languages,
        // check_dep_sources, check_disk, check_drivers, check_system_deps):
        let known_categories = vec![
            "Manifest",
            "Languages",
            "Dependencies",
            "Disk",
            "Drivers",
            "System Deps",
        ];

        let mut all_names: Vec<String> = callable_results
            .iter()
            .map(|r| r.category.clone())
            .collect();
        all_names.extend(known_categories.into_iter().map(String::from));

        let unique: std::collections::HashSet<&str> =
            all_names.iter().map(|s| s.as_str()).collect();
        assert_eq!(
            unique.len(),
            all_names.len(),
            "duplicate check category names detected: {:?}",
            all_names
        );
    }

    // ── Fix mode flag propagation ──────────────────────────────────────

    #[test]
    fn test_doctor_fix_mode_flag_propagates() {
        // run_doctor(verbose, json, fix) accepts the fix flag.
        // We cannot run full doctor in tests (it calls process::exit),
        // but we can verify the fix path is reachable by calling run_fix
        // with a manifest that has no system deps — it should succeed
        // without installing anything.
        let manifest = make_manifest("fix-test-proj");
        let tmp = tempfile::tempdir().unwrap();
        let ctx = dispatch::ProjectContext {
            root: tmp.path().to_path_buf(),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest.clone()),
            manifest_error: None,
        };
        // run_fix should not error on a project with zero system deps
        let result = run_fix(&manifest, &ctx);
        assert!(
            result.is_ok(),
            "fix mode should succeed on empty-dep project: {:?}",
            result.err()
        );
        // After fix, a lockfile should have been written
        let lock_path = tmp.path().join(HORUS_LOCK);
        assert!(
            lock_path.exists(),
            "fix mode should create/update {}",
            HORUS_LOCK
        );
    }
}

#[cfg(test)]
mod toolchain_grading_tests {
    use super::*;

    fn only<'a>(tools: &'a [&'a str]) -> impl Fn(&str) -> bool + 'a {
        move |t: &str| tools.contains(&t)
    }

    /// Everything a fully equipped Linux machine has on PATH. Written out once
    /// so that adding a requirement to TOOLCHAINS without adding it here turns
    /// `a_complete_toolchain_is_ok` red instead of silently narrowing what
    /// "complete" means.
    const FULL: &[&str] = &[
        "cargo",
        "rustc",
        "cargo-clippy",
        "rustfmt",
        "python3",
        "ruff",
        "pytest",
        "cmake",
        "g++",
        "make",
        "clang-format",
        "clang-tidy",
    ];

    /// `FULL` minus `drop` — a machine with one thing missing.
    fn without(drop: &[&str]) -> Vec<&'static str> {
        FULL.iter()
            .filter(|t| !drop.contains(*t))
            .copied()
            .collect()
    }

    /// The exact case that used to be reported as success: cargo, rustc and
    /// python3 present, everything else missing. It rendered as
    /// `* Toolchains — 3/8 tools found` and was counted among "7 ok", on a
    /// machine that then failed `horus new --cpp && horus build`.
    #[test]
    fn three_of_eight_is_not_success() {
        let (health, summary) = grade_toolchains(&only(&["cargo", "rustc", "python3"]));
        assert_eq!(health, Health::Warn, "got {summary}");
        assert!(summary.contains("C++ needs cmake"), "{summary}");
        assert!(summary.contains("Python missing"), "{summary}");
        // Rust builds, but `horus lint` and `horus fmt` do not: cargo and rustc
        // alone are not the whole Rust toolchain HORUS drives.
        assert!(summary.contains("Rust missing clippy"), "{summary}");
    }

    /// Grading must be monotonic. The old rule keyed on cargo/rustc alone, so
    /// six tools present without them graded *worse* than three tools with
    /// them — better coverage, worse verdict.
    #[test]
    fn more_tools_never_grades_worse() {
        let sets: [Vec<&str>; 5] = [
            vec![],
            vec!["cargo", "rustc"],
            vec!["cargo", "rustc", "python3", "ruff", "pytest"],
            without(&["cargo-clippy", "rustfmt"]),
            FULL.to_vec(),
        ];
        let rank = |h: &Health| match h {
            Health::Fail => 0,
            Health::Warn => 1,
            Health::Ok => 2,
        };

        let mut previous = 0;
        for set in sets {
            let (health, summary) = grade_toolchains(&only(&set));
            let r = rank(&health);
            assert!(
                r >= previous,
                "adding tools made the verdict worse ({previous} -> {r}) for {set:?}: {summary}"
            );
            previous = r;
        }
    }

    #[test]
    fn a_complete_toolchain_is_ok() {
        let (health, summary) = grade_toolchains(&only(FULL));
        assert_eq!(health, Health::Ok, "got {summary}");
        assert_eq!(summary, "Rust, Python, C++ ready");
    }

    /// Nothing installed must fail, and must name what is missing rather than
    /// printing a fraction.
    #[test]
    fn an_empty_machine_fails_and_says_why() {
        let (health, summary) = grade_toolchains(&only(&[]));
        assert_eq!(health, Health::Fail);
        for needle in ["cargo", "python3", "cmake", "compiler"] {
            assert!(summary.contains(needle), "{summary} should name {needle}");
        }
    }

    /// A missing optional tool degrades that language without blocking it —
    /// `horus build` works, `horus lint` does not.
    #[test]
    fn optional_tools_degrade_rather_than_block() {
        let (health, summary) = grade_toolchains(&only(&without(&[
            "ruff",
            "pytest",
            "clang-format",
            "clang-tidy",
        ])));
        assert_eq!(health, Health::Warn, "got {summary}");
        assert!(summary.contains("Rust ready"), "{summary}");
        assert!(summary.contains("Python missing ruff, pytest"), "{summary}");
        assert!(
            summary.contains("C++ missing clang-format, clang-tidy"),
            "{summary}"
        );
        // Degraded, not blocked: nothing may claim the language "needs" a tool
        // it can still build without.
        assert!(!summary.contains("needs"), "{summary}");
    }

    // ── The table itself, not just the rules over it ────────────────────
    //
    // Per-language grading was already correct; the table it graded was not.
    // cmake was the whole of C++ and Rust had no optional tools at all, so the
    // original false green reproduced verbatim one tool further along.

    /// cmake configures a build, it does not perform one. On a PATH with cmake
    /// but no compiler and no make, `horus doctor` printed
    /// `* Toolchains — Rust, Python, C++ ready` and `horus build` inside a
    /// freshly created `horus new --cpp` project then died during configure:
    ///
    /// ```text
    /// CMake Error: CMAKE_MAKE_PROGRAM is not set.
    /// CMake Error: CMAKE_CXX_COMPILER not set, after EnableLanguage
    /// ```
    #[test]
    fn cmake_alone_is_not_a_cpp_toolchain() {
        let (health, summary) = grade_toolchains(&only(&without(&["g++", "make"])));
        assert_ne!(
            health,
            Health::Ok,
            "cmake with no compiler and no make graded as ready: {summary}"
        );
        assert!(
            !summary.contains("C++ ready"),
            "C++ cannot be ready without a compiler: {summary}"
        );
        assert!(summary.contains("C++ needs"), "{summary}");
        assert!(summary.contains("compiler"), "{summary}");
        assert!(
            summary.contains("make") || summary.contains("ninja"),
            "{summary}"
        );
    }

    /// Each half of the C++ requirement has to bite on its own, or a machine
    /// with a compiler but no make still grades green.
    #[test]
    fn a_missing_make_program_blocks_cpp_on_its_own() {
        let (health, summary) = grade_toolchains(&only(&without(&["make"])));
        assert_eq!(health, Health::Warn, "got {summary}");
        assert!(summary.contains("C++ needs make or ninja"), "{summary}");
    }

    #[test]
    fn a_missing_compiler_blocks_cpp_on_its_own() {
        let (health, summary) = grade_toolchains(&only(&without(&["g++"])));
        assert_eq!(health, Health::Warn, "got {summary}");
        assert!(summary.contains("C++ needs a C++ compiler"), "{summary}");
    }

    /// The requirement is "a C++ compiler", not "g++": clang and ninja are a
    /// complete toolchain and must not be reported as a gap. A fix that hard-
    /// coded g++/make would trade one false verdict for its mirror image.
    #[test]
    fn clang_and_ninja_are_a_complete_cpp_toolchain() {
        let mut set = without(&["g++", "make"]);
        set.push("clang++");
        set.push("ninja");
        let (health, summary) = grade_toolchains(&only(&set));
        assert_eq!(health, Health::Ok, "got {summary}");
        assert!(summary.contains("C++ ready"), "{summary}");
    }

    /// `horus lint` on Rust is `cargo clippy -- -D warnings` and `horus fmt` is
    /// `cargo fmt` (dispatch::detect_rust_tools). Neither component is in a
    /// minimal-profile rustup or in most distro rust packages, and doctor used
    /// to print "Rust ready" on those machines — green tick, then
    /// `error: no such command: `clippy``.
    #[test]
    fn rust_without_clippy_or_rustfmt_is_not_ready() {
        let (health, summary) = grade_toolchains(&only(&without(&["cargo-clippy", "rustfmt"])));
        assert_eq!(health, Health::Warn, "got {summary}");
        assert!(!summary.contains("Rust ready"), "{summary}");
        assert!(
            summary.contains("Rust missing clippy, rustfmt"),
            "{summary}"
        );
        // Still buildable — a missing linter must not read as "needs".
        assert!(!summary.contains("Rust needs"), "{summary}");
    }

    /// `horus doctor --fix` runs horus_sys::sync::sync_environment, which calls
    /// `cpp::ensure_compiler()` and probes g++/clang++/c++. The report has to
    /// probe the same set: when the fixer and the report disagree, one half of
    /// one command tells the user the machine is fine while the other installs
    /// a compiler.
    #[test]
    fn the_report_probes_the_same_compilers_as_doctor_fix() {
        let cpp = TOOLCHAINS
            .iter()
            .find(|tc| tc.language == "C++")
            .expect("C++ toolchain");
        for bin in ["g++", "clang++", "c++"] {
            assert!(
                cpp.required.iter().any(|t| t.alternatives.contains(&bin)),
                "horus_sys::sync::cpp::detect_cpp_compiler accepts {bin}, doctor does not"
            );
        }
    }

    /// Naming the missing tool is half an answer. Every entry has to be able to
    /// say how to get it, on whatever platform the check is running on.
    #[test]
    fn every_tool_says_how_to_install_it() {
        for tc in TOOLCHAINS {
            for tool in tc.required.iter().chain(tc.optional.iter()) {
                let line = tool.missing_detail();
                assert!(
                    line.contains("install: "),
                    "{} has no install command: {line}",
                    tool.label
                );
                let cmd = line.split("install: ").nth(1).unwrap().trim();
                assert!(
                    cmd.split_whitespace().count() >= 2,
                    "{}'s install hint is not a command: {cmd:?}",
                    tool.label
                );
            }
        }
    }

    #[test]
    fn on_path_finds_real_executables_only() {
        assert!(on_path("cargo"), "cargo runs these tests, it is on PATH");
        assert!(!on_path("horus_definitely_not_a_tool_xyz"));
    }

    /// A file on PATH is not an installed tool. rustup ships a proxy in
    /// `~/.cargo/bin` for every component it *could* install, so `cargo-clippy`
    /// and `rustfmt` exist on every rustup machine and exit 1 with
    /// "the 'x' component ... is not available" until the component is added.
    /// Grading Rust on presence would print "Rust ready" for exactly the
    /// machines where `horus lint` dies with "no such command: `clippy`".
    #[test]
    #[cfg(unix)]
    fn a_proxy_that_cannot_run_is_not_an_installed_tool() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();

        let proxy = dir.path().join("cargo-clippy");
        std::fs::write(
            &proxy,
            "#!/bin/sh\necho \"error: not installed for this toolchain\" >&2\nexit 1\n",
        )
        .unwrap();
        std::fs::set_permissions(&proxy, std::fs::Permissions::from_mode(0o755)).unwrap();
        let proxy = proxy.to_str().unwrap();
        assert!(on_path(proxy), "the stub is an executable file");
        assert_eq!(
            probe(proxy),
            None,
            "a proxy that cannot run must not count as installed"
        );

        let real = dir.path().join("clang-format");
        std::fs::write(&real, "#!/bin/sh\necho 'clang-format version 21.1.8'\n").unwrap();
        std::fs::set_permissions(&real, std::fs::Permissions::from_mode(0o755)).unwrap();
        assert_eq!(
            probe(real.to_str().unwrap()).as_deref(),
            Some("clang-format version 21.1.8")
        );
    }

    /// Presence is the weaker signal, so only tools that genuinely cannot
    /// answer `--version` may use it. Nothing rustup proxies belongs here.
    #[test]
    fn only_msvc_tools_are_detected_by_presence() {
        assert_eq!(PRESENCE_ONLY, &["cl", "msbuild"]);
        for proxied in ["cargo-clippy", "rustfmt", "cargo", "rustc"] {
            assert!(
                !PRESENCE_ONLY.contains(&proxied),
                "{proxied} has a rustup proxy on PATH even when uninstalled"
            );
        }
    }

    #[test]
    fn a_non_executable_file_is_not_a_tool() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("pretend-compiler");
        std::fs::write(&file, "not a program").unwrap();
        assert!(!is_executable(&file), "a 0644 file is not an executable");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&file, std::fs::Permissions::from_mode(0o755)).unwrap();
            assert!(is_executable(&file));
        }
        assert!(
            !is_executable(dir.path()),
            "a directory is not an executable"
        );
    }
}

#[cfg(test)]
mod installation_tests {
    use super::*;
    use std::fs;

    fn facts(
        cli: &str,
        source: &str,
        src_version: Option<&str>,
        topic: Option<u32>,
    ) -> SourceFacts {
        let source = PathBuf::from(source);
        SourceFacts {
            cli_version: cli.to_string(),
            cli_topic_version: 4,
            source_version: src_version.map(str::to_string),
            source_topic_version: topic,
            origin: source_origin(&source, cli),
            source,
        }
    }

    /// Write a source tree that looks enough like a HORUS checkout for the two
    /// readers under test.
    fn fake_tree(root: &Path, version: &str, topic: u32) {
        let header = root.join("horus_core/src/communication/topic");
        fs::create_dir_all(&header).unwrap();
        fs::write(
            header.join("header.rs"),
            format!("pub(crate) const TOPIC_MAGIC: u32 = 1;\npub(crate) const TOPIC_VERSION: u32 = {topic};\n"),
        )
        .unwrap();
        for package in ["horus_manager", "horus_core"] {
            fs::create_dir_all(root.join(package)).unwrap();
            fs::write(
                root.join(package).join("Cargo.toml"),
                format!("[package]\nname = \"{package}\"\nversion = \"{version}\"\n"),
            )
            .unwrap();
        }
    }

    // ── The RC1 case ────────────────────────────────────────────────────

    /// The defect this whole check exists for: install.sh took the binary from
    /// a release tag and the source from main HEAD, 93 commits apart. Both
    /// halves said 0.4.0, so every version string on the machine agreed, and
    /// the CLI still could not read the shared memory its own libraries wrote.
    /// A verdict that only compared version strings called this healthy.
    #[test]
    fn matching_versions_do_not_excuse_a_topic_break() {
        let (health, summary, details) = grade_source_tree(&facts(
            "0.4.0",
            "/home/u/.horus/cache/horus@0.4.0",
            Some("0.4.0"),
            Some(3),
        ));
        assert_eq!(health, Health::Fail, "got {summary}");
        assert!(summary.contains("topic ABI break"), "{summary}");
        assert!(summary.contains("v4"), "{summary}");
        assert!(summary.contains("v3"), "{summary}");
        assert!(
            summary.contains("horus self update"),
            "the remedy has to be in the line the user sees by default: {summary}"
        );
        assert!(
            details.iter().any(|d| d.contains("shared memory")),
            "{details:?}"
        );
    }

    #[test]
    fn a_coherent_install_is_ok() {
        let (health, summary, _) = grade_source_tree(&facts(
            "0.4.0",
            "/home/u/.horus/cache/horus@0.4.0",
            Some("0.4.0"),
            Some(4),
        ));
        assert_eq!(health, Health::Ok, "got {summary}");
        assert_eq!(summary, "source tree horus@0.4.0 (topic v4)");
    }

    /// run_rust.rs:1069-1081 takes any cached `horus@*` when the exact version
    /// is gone. That is a silent substitution today; the report has to name it,
    /// because the tree it picked is the one user projects compile against.
    #[test]
    fn the_any_version_cache_fallback_is_named() {
        let facts = facts(
            "0.4.0",
            "/home/u/.horus/cache/horus@0.2.2",
            Some("0.2.2"),
            Some(4),
        );
        assert_eq!(facts.origin, SourceOrigin::CacheFallback);
        let (health, summary, details) = grade_source_tree(&facts);
        assert_eq!(health, Health::Warn, "got {summary}");
        assert!(summary.contains("horus@0.2.2"), "{summary}");
        assert!(summary.contains("horus self update"), "{summary}");
        assert!(
            details.iter().any(|d| d.contains("never asked for")),
            "{details:?}"
        );
    }

    /// A tree older than the header the topic version is parsed from must not
    /// be reported as a break — unknown is unknown.
    #[test]
    fn an_unreadable_topic_version_is_not_a_verdict() {
        let (health, summary, _) = grade_source_tree(&facts(
            "0.4.0",
            "/home/u/.horus/cache/horus@0.4.0",
            Some("0.4.0"),
            None,
        ));
        assert_eq!(health, Health::Ok, "got {summary}");
        assert!(summary.contains("unreadable"), "{summary}");
    }

    /// A development checkout at a different version is the RC1 shape with the
    /// ABI break not yet visible: the halves came from different refs.
    #[test]
    fn a_checkout_at_another_version_warns() {
        let facts = facts("0.4.0", "/home/u/softmata/horus", Some("0.5.0"), Some(4));
        assert_eq!(facts.origin, SourceOrigin::Checkout);
        let (health, summary, _) = grade_source_tree(&facts);
        assert_eq!(health, Health::Warn, "got {summary}");
        assert!(summary.contains("0.5.0"), "{summary}");
    }

    // ── Reading a tree off disk ─────────────────────────────────────────

    #[test]
    fn topic_version_and_crate_version_come_off_a_real_tree() {
        let dir = tempfile::tempdir().unwrap();
        fake_tree(dir.path(), "0.4.0", 4);
        assert_eq!(topic_version_in(dir.path()), Some(4));
        assert_eq!(crate_version_in(dir.path()).as_deref(), Some("0.4.0"));

        let empty = tempfile::tempdir().unwrap();
        assert_eq!(topic_version_in(empty.path()), None);
        assert_eq!(crate_version_in(empty.path()), None);
    }

    /// horus_core is the fallback, and the workspace-inherited form carries no
    /// version to read — treating `version.workspace = true` as an answer would
    /// report the literal string "workspace" as the installed version.
    #[test]
    fn an_inherited_version_falls_through_to_horus_core() {
        let dir = tempfile::tempdir().unwrap();
        fake_tree(dir.path(), "0.4.0", 4);
        fs::write(
            dir.path().join("horus_manager/Cargo.toml"),
            "[package]\nname = \"horus_manager\"\nversion.workspace = true\n",
        )
        .unwrap();
        assert_eq!(crate_version_in(dir.path()).as_deref(), Some("0.4.0"));
    }

    #[test]
    fn the_topic_version_parser_accepts_both_visibilities() {
        let dir = tempfile::tempdir().unwrap();
        let header = dir.path().join("horus_core/src/communication/topic");
        fs::create_dir_all(&header).unwrap();
        fs::write(
            header.join("header.rs"),
            "pub const TOPIC_VERSION: u32 = 7;\n",
        )
        .unwrap();
        assert_eq!(topic_version_in(dir.path()), Some(7));
    }

    // ── Orphaned caches ─────────────────────────────────────────────────

    /// 727 MB of a version the user stopped running months earlier, which
    /// nothing on the machine mentioned.
    #[test]
    fn an_unclaimed_cache_is_reported_with_its_size() {
        let root = tempfile::tempdir().unwrap();
        for version in ["0.2.2", "0.4.0"] {
            let tree = root.path().join(format!("horus@{version}"));
            fs::create_dir_all(&tree).unwrap();
            fs::write(tree.join("payload"), vec![b'x'; 1024]).unwrap();
        }
        fs::create_dir_all(root.path().join("pypi_serial@latest")).unwrap();

        let claimed = vec![root.path().join("horus@0.4.0")];
        let orphans = orphaned_source_caches_in(&[root.path().to_path_buf()], &claimed);
        assert_eq!(orphans.len(), 1, "{orphans:?}");
        assert_eq!(orphans[0].0, root.path().join("horus@0.2.2"));
        assert_eq!(orphans[0].1, 1024, "the size is what makes it actionable");
    }

    #[test]
    fn a_claimed_cache_is_never_called_an_orphan() {
        let root = tempfile::tempdir().unwrap();
        let tree = root.path().join("horus@0.4.0");
        fs::create_dir_all(&tree).unwrap();
        let orphans = orphaned_source_caches_in(&[root.path().to_path_buf()], &[tree]);
        assert!(orphans.is_empty(), "{orphans:?}");
    }

    // ── Duplicate binaries on PATH ──────────────────────────────────────

    /// The support trap: an old ~/.local/bin/horus and a new ~/.cargo/bin/horus,
    /// with PATH order silently deciding which one `horus` means.
    #[test]
    fn two_versions_on_path_are_a_conflict() {
        let binaries = vec![
            (
                PathBuf::from("/h/.local/bin/horus"),
                Some("horus 0.2.2".to_string()),
            ),
            (
                PathBuf::from("/h/.cargo/bin/horus"),
                Some("horus 0.4.0".to_string()),
            ),
        ];
        let conflict = path_conflict(&binaries).expect("two versions disagree");
        assert!(conflict.contains("0.2.2 shadows 0.4.0"), "{conflict}");
    }

    /// Two copies of the same version shadow each other harmlessly; reporting
    /// that as a conflict would train users to ignore the check.
    #[test]
    fn one_version_in_two_places_is_not_a_conflict() {
        let binaries = vec![
            (
                PathBuf::from("/h/.local/bin/horus"),
                Some("horus 0.4.0".to_string()),
            ),
            (
                PathBuf::from("/h/.cargo/bin/horus"),
                Some("horus 0.4.0".to_string()),
            ),
        ];
        assert_eq!(path_conflict(&binaries), None);
        assert_eq!(path_conflict(&[]), None);
    }

    /// `on_path` is the old boolean answer over the new list. Both have to keep
    /// agreeing, or `probe()` starts grading MSVC tools on a different rule.
    #[test]
    fn all_on_path_and_on_path_agree() {
        assert!(!all_on_path("cargo").is_empty());
        assert_eq!(on_path("cargo"), !all_on_path("cargo").is_empty());
        assert!(all_on_path("horus_definitely_not_a_tool_xyz").is_empty());
        assert!(!on_path("horus_definitely_not_a_tool_xyz"));
    }

    /// One hit per PATH directory, the way the OS resolves a command — and
    /// every directory, not just the first, which is the whole point of the
    /// widened lookup. Tested through `executables_in` so it does not have to
    /// mutate this process's PATH, which the toolchain tests above read.
    #[test]
    #[cfg(unix)]
    fn every_directory_contributes_its_own_hit() {
        use std::os::unix::fs::PermissionsExt;
        let first = tempfile::tempdir().unwrap();
        let second = tempfile::tempdir().unwrap();
        let third = tempfile::tempdir().unwrap();
        for dir in [first.path(), second.path()] {
            let bin = dir.join("horus");
            fs::write(&bin, "#!/bin/sh\nexit 0\n").unwrap();
            fs::set_permissions(&bin, fs::Permissions::from_mode(0o755)).unwrap();
        }
        // A name that is present but not executable is not an install.
        fs::write(third.path().join("horus"), "not a program").unwrap();

        let dirs = [first.path(), second.path(), third.path()].map(Path::to_path_buf);
        assert_eq!(
            executables_in("horus", dirs.into_iter(), &[]),
            vec![first.path().join("horus"), second.path().join("horus")],
            "both copies must be reported, in the order the directories were searched"
        );
    }

    // ── Health combination ──────────────────────────────────────────────

    #[test]
    fn the_worst_verdict_wins() {
        assert_eq!(Health::Ok.worst(Health::Warn), Health::Warn);
        assert_eq!(Health::Warn.worst(Health::Ok), Health::Warn);
        assert_eq!(Health::Warn.worst(Health::Fail), Health::Fail);
        assert_eq!(Health::Fail.worst(Health::Ok), Health::Fail);
        assert_eq!(Health::Ok.worst(Health::Ok), Health::Ok);
    }
}
