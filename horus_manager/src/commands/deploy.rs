//! Deploy command - Deploy HORUS projects to remote robots
//!
//! Handles cross-compilation, file transfer, and remote execution.

use crate::cli_output;
use crate::config::CARGO_TOML;
use crate::manifest::{detect_languages, Language};
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

/// The one file `horus deploy` reads its fleet inventory from.
///
/// Named once so every message that has to say where a target came from — or
/// why one was not found — points at the same path.
const DEPLOY_YAML: &str = ".horus/deploy.yaml";

/// Where the robot's own wheels live, relative to the remote project directory.
///
/// `run_python` uses the same path locally and [`remote_run_command`] prepends
/// it to `PYTHONPATH`, so the installer and the runner have to agree on it or
/// the deploy succeeds and every `import` on the robot fails.
const REMOTE_PACKAGES_DIR: &str = ".horus/packages";

/// The fleet inventory: every named target, keyed by the name typed after
/// `horus deploy`.
///
/// Read from the project's [`DEPLOY_YAML`] and nowhere else. This was
/// documented as "~/.horus/deploy.yaml or .horus/deploy.yaml" while
/// [`load_deploy_yaml`] only ever opened the project file, so a fleet written
/// where the comment said to write it resolved nothing and every name in it
/// fell through to [`resolve_target`]'s bare-hostname path — the silent,
/// destructive one. The project file is also what the trust argument in
/// [`validate_ssh_inputs`] rests on: these hosts and identity paths are allowed
/// into an ssh argv because they are part of the checkout, reviewed like the
/// rest of it. `cargo_gen`'s `BUILD_DIR_KEEPS` negates the `*` in
/// `.horus/.gitignore` for exactly this file so that stays true.
#[derive(Debug, serde::Deserialize)]
struct DeployYaml {
    targets: HashMap<String, YamlTarget>,
}

/// A named target entry in the deploy YAML
#[derive(Debug, serde::Deserialize)]
struct YamlTarget {
    host: String,
    arch: Option<String>,
    dir: Option<String>,
    port: Option<u16>,
    identity: Option<String>,
}

/// Result of resolving a target string against the YAML config
struct ResolvedTarget {
    host: String,
    arch: Option<String>,
    dir: Option<String>,
    port: Option<u16>,
    identity: Option<PathBuf>,
    origin: TargetOrigin,
}

/// Where a target's connection details came from.
///
/// [`TargetOrigin::Unlisted`] is the one that has to be visible. A name the
/// inventory does not have is handed to ssh as a hostname, and the sync behind
/// it runs `rsync -avz --delete` against whatever DNS, `/etc/hosts` or
/// `~/.ssh/config` makes of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TargetOrigin {
    /// A `user@host` typed on the command line; no lookup is attempted.
    Direct,
    /// A named entry in [`DEPLOY_YAML`].
    Listed,
    /// Nothing matched — the string is being used as a hostname as-is.
    Unlisted,
}

/// Supported target architectures for robotics platforms
#[derive(Debug, Clone)]
pub enum TargetArch {
    /// ARM64 (Raspberry Pi 4/5, Jetson Nano/Xavier/Orin)
    Aarch64,
    /// ARM 32-bit (Raspberry Pi 3, older boards)
    Armv7,
    /// x86_64 (Intel NUC, standard PCs)
    X86_64,
    /// Current host architecture
    Native,
}

impl TargetArch {
    fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "aarch64" | "arm64" | "jetson" | "pi4" | "pi5" => Some(TargetArch::Aarch64),
            "armv7" | "arm" | "pi3" | "pi2" => Some(TargetArch::Armv7),
            "x86_64" | "x64" | "amd64" | "intel" => Some(TargetArch::X86_64),
            "native" | "host" | "local" => Some(TargetArch::Native),
            _ => None,
        }
    }

    fn rust_target(&self) -> &'static str {
        match self {
            TargetArch::Aarch64 => "aarch64-unknown-linux-gnu",
            TargetArch::Armv7 => "armv7-unknown-linux-gnueabihf",
            TargetArch::X86_64 => "x86_64-unknown-linux-gnu",
            TargetArch::Native => "", // Use default
        }
    }

    fn display_name(&self) -> &'static str {
        match self {
            TargetArch::Aarch64 => "ARM64 (aarch64)",
            TargetArch::Armv7 => "ARM32 (armv7)",
            TargetArch::X86_64 => "x86_64",
            TargetArch::Native => "native",
        }
    }
}

/// Deploy configuration
#[derive(Debug)]
pub struct DeployConfig {
    /// Target host (user@host or just host)
    pub target: String,
    /// Remote directory to deploy to
    pub remote_dir: String,
    /// Target architecture
    pub arch: TargetArch,
    /// Whether to run after deploying
    pub run_after: bool,
    /// Whether to build in release mode
    pub release: bool,
    /// SSH port
    pub port: u16,
    /// SSH identity file
    pub identity: Option<PathBuf>,
    /// Extra rsync excludes
    pub excludes: Vec<String>,
    /// Skip interactive confirmation (for fleet deploy where confirmation is done upfront)
    pub skip_confirm: bool,
}

impl Default for DeployConfig {
    fn default() -> Self {
        Self {
            target: String::new(),
            remote_dir: "~/horus_deploy".to_string(),
            arch: TargetArch::Aarch64,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            excludes: vec![],
            skip_confirm: false,
        }
    }
}

/// Resolve a target string: either a direct `user@host` or a named target from
/// [`DEPLOY_YAML`].
///
/// The `origin` on the result is not bookkeeping: "not in the inventory" is a
/// different answer from "found it", and the caller has to say so before it
/// runs a `--delete` sync against the guess. See [`warn_unlisted_target`].
fn resolve_target(target: &str) -> ResolvedTarget {
    // If target contains '@', treat as direct user@host — no YAML lookup
    if target.contains('@') {
        return ResolvedTarget {
            host: target.to_string(),
            arch: None,
            dir: None,
            port: None,
            identity: None,
            origin: TargetOrigin::Direct,
        };
    }

    // Try to load the inventory and look up the named target
    if let Some(yaml) = load_deploy_yaml() {
        if let Some(entry) = yaml.targets.get(target) {
            return ResolvedTarget {
                host: entry.host.clone(),
                arch: entry.arch.clone(),
                dir: entry.dir.clone(),
                port: entry.port,
                identity: entry.identity.as_ref().map(PathBuf::from),
                origin: TargetOrigin::Listed,
            };
        }
    }

    // Not found in YAML — return as-is (could be a bare hostname)
    ResolvedTarget {
        host: target.to_string(),
        arch: None,
        dir: None,
        port: None,
        identity: None,
        origin: TargetOrigin::Unlisted,
    }
}

/// Say — before anything is built, transferred or deleted — that this target is
/// not one the inventory names.
///
/// [`resolve_target`] falls through to "could be a bare hostname" for anything
/// it cannot find, and said nothing about it. On a checkout with no inventory,
/// or with the name spelled differently, `horus deploy jetson` printed a
/// confident plan for a host called `jetson` and then ran `rsync -avz --delete`
/// against whatever that resolved to, removing everything in the destination
/// directory this project does not have.
fn warn_unlisted_target(name: &str) {
    println!(
        "  {} '{}' is not a target in {} — treating it as a hostname.",
        cli_output::ICON_WARN.yellow(),
        name.yellow(),
        DEPLOY_YAML
    );
    match load_deploy_yaml() {
        Some(yaml) if !yaml.targets.is_empty() => {
            let mut names: Vec<&str> = yaml.targets.keys().map(String::as_str).collect();
            names.sort();
            println!(
                "  {} Configured targets: {}",
                cli_output::ICON_HINT.dimmed(),
                names.join(", ")
            );
        }
        _ => println!(
            "  {} No {} here. `horus deploy --list` shows what is configured.",
            cli_output::ICON_HINT.dimmed(),
            DEPLOY_YAML
        ),
    }
}

/// Reject an ssh destination or identity that the tools downstream would read
/// as an *option* rather than as data.
///
/// `host` and `identity` come from [`DEPLOY_YAML`], which lives inside the
/// project — i.e. inside any checkout the user cloned, and committed there:
/// `cargo_gen`'s `BUILD_DIR_KEEPS` re-includes it from the `*` that ignores the
/// rest of `.horus/`. Both land in the argv of `ssh` and `rsync` with nothing
/// between them and the parser:
///
/// * [`run_on_target`] passes the host as a bare positional
///   (`cmd.arg(&config.target)`). ssh reads a leading `-` as an option, and
///   `-oProxyCommand=…` makes ssh run that string through `/bin/sh`.
/// * [`sync_binary_to_target`] interpolates the identity into rsync's `-e`
///   string. rsync splits `--rsh` on **whitespace only** — no shell, no quote
///   handling — so an identity containing a space contributes further argv
///   elements to the ssh invocation, `-oProxyCommand=…` included. (It is also
///   why the single quotes [`sync_to_target`] wraps the identity in are not a
///   defence: they reach ssh as part of the filename.)
///
/// A hostname and a key path never legitimately begin with `-` or contain
/// whitespace, so refuse rather than attempt to quote.
fn validate_ssh_inputs(host: &str, identity: Option<&Path>) -> HorusResult<()> {
    fn reject(what: &str, value: &str, why: &str) -> HorusError {
        HorusError::Config(ConfigError::Other(format!(
            "Refusing to deploy: the {what} {value:?} {why}.\n\
             Deploy targets are read from {DEPLOY_YAML}, which is part of the \
             checkout, and this value is passed straight into the argv of ssh/rsync — \
             where it would be parsed as an option (for example -oProxyCommand=...) \
             instead of as a destination."
        )))
    }

    if host.trim().is_empty() {
        return Err(reject("target host", host, "is empty"));
    }
    if host.starts_with('-') {
        return Err(reject("target host", host, "begins with '-'"));
    }
    if host.chars().any(char::is_whitespace) {
        return Err(reject("target host", host, "contains whitespace"));
    }

    if let Some(identity) = identity {
        let text = identity.to_string_lossy();
        if text.trim().is_empty() {
            return Err(reject("identity file", &text, "is empty"));
        }
        if text.starts_with('-') {
            return Err(reject("identity file", &text, "begins with '-'"));
        }
        if text.chars().any(char::is_whitespace) {
            return Err(reject(
                "identity file",
                &text,
                "contains whitespace, which rsync turns into extra ssh arguments \
                 because it splits its -e command on whitespace",
            ));
        }
    }

    Ok(())
}

/// Load and parse [`DEPLOY_YAML`] from the current directory.
///
/// A malformed inventory used to be indistinguishable from a missing one: the
/// parse error went into `.ok()` and the deploy carried on treating every name
/// as a bare hostname. Say which one it is — the answer decides whether
/// `horus deploy jetson` reaches the Jetson or something DNS invented.
fn load_deploy_yaml() -> Option<DeployYaml> {
    let content = std::fs::read_to_string(Path::new(DEPLOY_YAML)).ok()?;
    match serde_yaml::from_str(&content) {
        Ok(yaml) => Some(yaml),
        Err(e) => {
            eprintln!(
                "{} {} could not be parsed, so no named target resolves: {}",
                cli_output::ICON_WARN.yellow(),
                DEPLOY_YAML,
                e
            );
            None
        }
    }
}

/// CLI arguments for single-target deploy (internal).
pub struct DeployArgs {
    pub target: String,
    pub remote_dir: Option<String>,
    pub arch: Option<String>,
    pub run_after: bool,
    pub release: bool,
    pub port: u16,
    pub identity: Option<PathBuf>,
    pub dry_run: bool,
    pub print_service: bool,
}

/// CLI arguments for multi-target deploy.
pub struct DeployMultiArgs {
    pub targets: Vec<String>,
    pub all: bool,
    pub parallel: bool,
    pub remote_dir: Option<String>,
    pub arch: Option<String>,
    pub run_after: bool,
    pub release: bool,
    pub port: u16,
    pub identity: Option<PathBuf>,
    pub dry_run: bool,
    pub print_service: bool,
}

/// Run deploy to one or more targets.
pub fn run_deploy_multi(args: DeployMultiArgs) -> HorusResult<()> {
    let DeployMultiArgs {
        targets,
        all,
        parallel,
        remote_dir,
        arch,
        run_after,
        release,
        port,
        identity,
        dry_run,
        print_service,
    } = args;

    // Resolve target list
    let target_names: Vec<String> = if all {
        // Load all targets from deploy.yaml
        match load_deploy_yaml() {
            Some(yaml) if !yaml.targets.is_empty() => {
                let mut names: Vec<String> = yaml.targets.keys().cloned().collect();
                names.sort();
                println!(
                    "{} Deploying to {} target(s): {}",
                    cli_output::ICON_INFO.cyan(),
                    names.len(),
                    names.join(", ").green()
                );
                names
            }
            _ => {
                return Err(HorusError::Config(ConfigError::Other(
                    "No targets configured in deploy.yaml. Create one with target definitions."
                        .to_string(),
                )));
            }
        }
    } else if targets.is_empty() {
        return Err(HorusError::Config(ConfigError::Other(
            "No target specified. Use a target name, user@host, --all, or --list.".to_string(),
        )));
    } else {
        targets
    };

    // A unit file per host, concatenated on one stdout, could not be
    // redirected into anything useful — and each host's unit differs in the
    // target it names. Ask for them one at a time.
    if print_service && target_names.len() > 1 {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "--print-service writes one unit file, but {} targets were given. \
             Run it once per target.",
            target_names.len()
        ))));
    }

    if target_names.len() == 1 {
        // Single target — use existing deploy path
        return run_deploy(DeployArgs {
            target: target_names.into_iter().next().unwrap(),
            remote_dir,
            arch,
            run_after,
            release,
            port,
            identity,
            dry_run,
            print_service,
        });
    }

    // ── Multi-target (fleet) deploy ─────────────────────────────────
    let total = target_names.len();
    println!(
        "\n{} {}",
        "HORUS Fleet Deploy".green().bold(),
        format!("({} targets)", total).dimmed()
    );
    for (i, name) in target_names.iter().enumerate() {
        let resolved = resolve_target(name);
        println!(
            "  {} {} → {}",
            format!("{}.", i + 1).dimmed(),
            name.cyan(),
            resolved.host
        );
    }

    // Step 1: Build once (all targets share the same binary if same arch)
    // Use first target's arch for the build
    let first_resolved = resolve_target(&target_names[0]);
    let build_arch_str = arch.clone().or(first_resolved.arch);
    let build_arch = build_arch_str
        .as_ref()
        .and_then(|a| TargetArch::from_str(a))
        .unwrap_or_else(|| detect_target_arch(&first_resolved.host));

    println!(
        "\n{} Step 1: Building for {} (shared across {} targets)...",
        cli_output::ICON_INFO.cyan(),
        build_arch.display_name().yellow(),
        total
    );

    if !dry_run {
        let build_config = DeployConfig {
            target: first_resolved.host.clone(),
            remote_dir: "~/horus_deploy".to_string(),
            arch: build_arch,
            run_after: false,
            release,
            port,
            identity: identity.clone(),
            excludes: vec![],
            skip_confirm: true,
        };
        build_for_target(&build_config)?;
    }

    // Step 2: Sync + run to each target
    println!(
        "\n{} Step 2: Syncing to {} targets{}...",
        cli_output::ICON_INFO.cyan(),
        total,
        if parallel { " (parallel)" } else { "" }
    );

    // For fleet deploy, skip the interactive confirmation per-target.
    // Ask once for the whole fleet.
    if !dry_run {
        println!(
            "  {} This will sync to {} remote hosts (with --delete)",
            cli_output::ICON_WARN.yellow(),
            total
        );
        print!("  Continue? [y/N] ");
        std::io::Write::flush(&mut std::io::stdout()).ok();
        let mut input = String::new();
        std::io::stdin().read_line(&mut input).ok();
        if !input.trim().eq_ignore_ascii_case("y") {
            println!("  Cancelled.");
            return Ok(());
        }
    }

    let mut failures = 0;

    // Sequential deploy (parallel would need thread-safe output — future work)
    for (i, name) in target_names.iter().enumerate() {
        println!(
            "\n{} [{}/{}] {}",
            "---".cyan(),
            i + 1,
            total,
            name.green().bold()
        );

        let result = run_deploy(DeployArgs {
            target: name.clone(),
            remote_dir: remote_dir.clone(),
            arch: arch.clone(),
            run_after,
            release,
            port,
            identity: identity.clone(),
            dry_run,
            print_service: false,
        });

        match result {
            Ok(()) => println!(
                "{} [{}/{}] {} done",
                cli_output::ICON_SUCCESS.green(),
                i + 1,
                total,
                name
            ),
            Err(e) => {
                println!(
                    "{} [{}/{}] {} failed: {}",
                    cli_output::ICON_ERROR.red(),
                    i + 1,
                    total,
                    name,
                    e
                );
                failures += 1;
                // Continue to next target — don't abort the fleet
            }
        }
    }

    // Summary
    println!("\n{} Fleet Deploy Summary", "===".cyan());
    println!(
        "  {} {}/{} targets deployed successfully",
        if failures == 0 {
            cli_output::ICON_SUCCESS.green()
        } else {
            cli_output::ICON_WARN.yellow()
        },
        total - failures,
        total
    );

    if failures > 0 {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "{} of {} deployments failed",
            failures, total
        ))));
    }

    Ok(())
}

/// Wrap a string so a POSIX shell reads it as exactly one literal word.
///
/// One implementation, because every remote command this module builds is a
/// single string the robot's shell re-parses: the path in `cd`, the Python
/// entry point, each pip requirement.
fn shell_single_quote(s: &str) -> String {
    format!("'{}'", s.replace('\'', "'\\''"))
}

/// Quote a remote path for `sh -c`, leaving a leading `~` unquoted.
///
/// `cd '~/horus_deploy'` does not work: POSIX tilde expansion happens before
/// quote removal only for an *unquoted* tilde, so the shell looks for a
/// literal directory named `~`. rsync's destination form (`host:~/dir/`) is
/// expanded by the remote shell, so files land in `$HOME/dir` — and the run
/// step could never reach them.
fn shell_quote_preserving_tilde(path: &str) -> String {
    if path == "~" {
        return "~".to_string();
    }
    match path.strip_prefix("~/") {
        // `~/rest` -> `~/'rest'`: tilde expands, the rest stays literal.
        Some(rest) if !rest.is_empty() => format!("~/{}", shell_single_quote(rest)),
        Some(_) => "~/".to_string(),
        None => shell_single_quote(path),
    }
}

/// Default rsync excludes.
///
/// `target` is excluded as a bare pattern, and rsync matches such a pattern
/// against *any* path component — so it also matched `.horus/target/`, which
/// is exactly where `horus build` puts the Rust binary. The deploy shipped
/// 37 KB of source, excluded the 2.8 MB executable it had just built, exec'd
/// a path that had never been transferred, and printed "Deployment complete!".
///
/// The exclude stays — the intermediates tree is ~370 MB and must not ship —
/// and the single artifact is transferred by a second, explicit rsync in
/// [`sync_binary_to_target`]. Re-admitting it with `--include` rules was tried
/// and rejected: rsync descends into any directory a rule includes, so the
/// wildcard needed for cross-compiled target triples pulled the whole tree
/// back in (measured: 1,229 files, 367,917,030 bytes).
///
/// The Python environments are excluded for the same reason as the build
/// trees: they are this machine's, not the robot's. See the patterns below.
///
/// Keeping the inventory off the wire is not hygiene either. It is committed
/// config, reviewed like the rest of the checkout — that is what lets
/// [`validate_ssh_inputs`] trust it into an ssh argv — but it also names every
/// robot's host, port and SSH identity path, and every deploy used to copy it
/// onto each robot in turn. Committing it to your repository and replicating it
/// across the fleet are different decisions; only the first one was made.
fn default_excludes() -> Vec<String> {
    [
        // Build trees, including .horus/target. The one artifact we need is
        // transferred separately by sync_binary_to_target.
        "target",
        ".git",
        "node_modules",
        "__pycache__",
        "*.pyc",
        // The developer's Python environments, none of which run on a robot.
        // `.horus/venv` is 13 MB whose pyvenv.cfg holds this machine's absolute
        // paths and whose `bin/python3` is a symlink to an x86 interpreter
        // (registry/install.rs builds it for pip); `.horus/packages` is
        // symlinks into this machine's cache, which dangle the moment they
        // land on the robot. install_python_deps_on_target rebuilds
        // `.horus/packages` there with the robot's own pip and its own wheels.
        ".horus/venv",
        ".venv",
        "venv",
        ".horus/packages",
        // Never ship the fleet inventory to a member of the fleet.
        ".horus/deploy.yaml",
        // Nor credentials that happen to sit in the project directory.
        ".env",
        ".env.*",
        "id_rsa",
        "id_ed25519",
        "*.pem",
        "*.key",
        // Host-specific CMake state; absolute paths from the developer's machine.
        ".horus/cpp-build/CMakeCache.txt",
        ".horus/cpp-build/CMakeFiles",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect()
}

/// Turn CLI arguments into the configuration one deploy runs with, and say
/// where the target came from.
///
/// Shared with [`print_service`] so the unit file names the same host, the same
/// directory and the same binary the deploy itself would use. Computing a path
/// twice is exactly how the transfer step and the run step came to disagree
/// (see [`locate_built_binary`]); there is no reason to repeat it.
fn deploy_config_from_args(args: DeployArgs) -> HorusResult<(DeployConfig, TargetOrigin)> {
    let DeployArgs {
        target,
        remote_dir,
        arch,
        run_after,
        release,
        port,
        identity,
        dry_run: _,
        print_service: _,
    } = args;
    // Resolve named target from the inventory (if applicable)
    let resolved = resolve_target(&target);

    // CLI args win over YAML values. For Option fields, None means "not set by user".
    // For port, 22 is the clap default — treat it as "not explicitly set" so YAML can override.
    let effective_arch_str = arch.or(resolved.arch);
    let effective_host = &resolved.host;
    let effective_dir = remote_dir
        .or(resolved.dir)
        .unwrap_or_else(|| "~/horus_deploy".to_string());
    let effective_port = if port != 22 {
        port
    } else {
        resolved.port.unwrap_or(22)
    };
    let effective_identity = identity.or(resolved.identity);

    // Parse target architecture
    let target_arch = effective_arch_str
        .as_ref()
        .and_then(|a| TargetArch::from_str(a))
        .unwrap_or_else(|| detect_target_arch(effective_host));

    let config = DeployConfig {
        target: effective_host.to_string(),
        remote_dir: effective_dir,
        arch: target_arch,
        run_after,
        release,
        port: effective_port,
        identity: effective_identity,
        excludes: default_excludes(),
        skip_confirm: false,
    };

    // The host and the identity path reach the argv of ssh and rsync with no
    // parser in between, and both can come from the checkout's deploy.yaml.
    // Refuse the shapes that would be read as options there.
    validate_ssh_inputs(&config.target, config.identity.as_deref())?;

    // A name the inventory does not have is not a typo this command can fix for
    // the user — it becomes a hostname, and a --delete sync follows it.
    if resolved.origin == TargetOrigin::Unlisted {
        warn_unlisted_target(&target);
    }

    Ok((config, resolved.origin))
}

/// The exact rsync destination: `host:dir/`.
///
/// Every message that mentions `--delete` prints this rather than the host and
/// the directory on separate lines. "Target: jetson" and "Remote dir:
/// ~/horus_deploy" are the two halves of the path files are deleted from, and
/// the confirmation prompt never put them together.
fn sync_destination(config: &DeployConfig) -> String {
    format!("{}:{}/", config.target, config.remote_dir)
}

pub fn run_deploy(args: DeployArgs) -> HorusResult<()> {
    let dry_run = args.dry_run;
    let wants_service = args.print_service;
    let (config, _origin) = deploy_config_from_args(args)?;

    // Before the banner: this writes a unit file to stdout, so anything else
    // printed first would have to be stripped back out before it could be
    // redirected into a file.
    if wants_service {
        return print_service(&config);
    }

    println!("{}", "HORUS Deploy".green().bold());
    println!();
    println!("  {} {}", "Target:".cyan(), config.target);
    println!("  {} {}", "Remote dir:".cyan(), config.remote_dir);
    println!(
        "  {} {}",
        "Architecture:".cyan(),
        config.arch.display_name()
    );
    println!(
        "  {} {}",
        "Build mode:".cyan(),
        if config.release { "release" } else { "debug" }
    );
    println!("  {} {}", "Run after:".cyan(), config.run_after);
    println!(
        "  {} {} {}",
        "Deletes in:".cyan(),
        sync_destination(&config).yellow(),
        "(rsync --delete removes whatever is there and not here)".dimmed()
    );
    println!();

    if dry_run {
        println!(
            "{}",
            "[DRY RUN] Would perform the following steps:"
                .yellow()
                .bold()
        );
        println!();
        print_deploy_plan(&config);
        return Ok(());
    }

    // Step 1: Build for target
    println!("{}", "Step 1: Building project...".cyan().bold());
    build_for_target(&config)?;

    // Step 2: Sync files
    println!();
    println!("{}", "Step 2: Syncing files to target...".cyan().bold());
    if !sync_to_target(&config)? {
        // Answering "no" to "this deletes files on <host>" used to abort the
        // transfer and nothing else: the deploy went on to run the project over
        // ssh and print "Deployment complete!".
        println!();
        println!(
            "{} Deploy cancelled — nothing was transferred.",
            "!".yellow()
        );
        return Ok(());
    }

    // The steps after the transfer depend on the language, so they are
    // numbered as they happen rather than hardcoded.
    let mut step = 2;

    // Cross-compilation covers the Rust and C++ artifact. A Python project's
    // imports are satisfied by wheels instead, and they have to be the robot's
    // own — see install_python_deps_on_target.
    if detect_deploy_language() == Language::Python {
        step += 1;
        println!();
        println!(
            "{}",
            format!("Step {step}: Installing Python dependencies on target...")
                .cyan()
                .bold()
        );
        install_python_deps_on_target(&config)?;
    }

    // Run if requested
    if config.run_after {
        step += 1;
        println!();
        println!(
            "{}",
            format!("Step {step}: Running on target...").cyan().bold()
        );
        run_on_target(&config)?;
    }

    println!();
    println!("{} Deployment complete!", cli_output::ICON_SUCCESS.green());
    println!();
    println!(
        "  {} ssh {}:{} to access your robot",
        "Tip:".dimmed(),
        config.target,
        config.port
    );
    // What was just deployed dies with the ssh session that started it. Say so
    // where the user is looking, not only in the docs.
    println!(
        "  {} nothing here survives a reboot — `horus deploy {} --print-service` \
         writes a systemd unit that does",
        "Tip:".dimmed(),
        config.target
    );

    Ok(())
}

/// Write a systemd unit for this deploy to stdout.
///
/// Printed, never installed. Nothing HORUS deploys survives a reboot: there is
/// no unit file anywhere in the tree, `launch`'s default restart policy is
/// "never", and `--run` starts the project over the interactive ssh session, so
/// it dies with that session. A robot fleet had no supported way to come back
/// after a power cycle.
///
/// The unit is emitted rather than installed because installing one remotely
/// means choosing between `systemctl --user` and a system unit, deciding
/// whether to enable lingering, and holding privileges this command does not
/// otherwise need. Printing is honest about what it knows — the host, the
/// directory and the exact command `--run` would have used, all read back from
/// the same [`deploy_config_from_args`] the deploy itself uses — and leaves the
/// policy to whoever owns the robot.
fn print_service(config: &DeployConfig) -> HorusResult<()> {
    let (remote_cmd, _) = remote_run_command(config);
    let unit_name = service_unit_name(config);

    // WorkingDirectory is set from the deploy's own remote_dir, and ExecStart
    // re-runs the command through `sh -lc` because remote_run_command produces
    // a shell line (a `cd`, and for Python a PYTHONPATH assignment), not an
    // argv systemd could exec directly.
    println!("# {unit_name}");
    println!("#");
    println!("# Install as a user service (survives reboot only with lingering enabled):");
    println!(
        "#   scp {unit_name} {}:~/.config/systemd/user/",
        config.target
    );
    println!(
        "#   ssh {} 'systemctl --user daemon-reload && systemctl --user enable --now {unit_name}'",
        config.target
    );
    println!(
        "#   ssh {} 'sudo loginctl enable-linger $USER'   # or it stops at logout",
        config.target
    );
    println!("#");
    println!("# Or as a system service:");
    println!("#   scp {unit_name} {}:/tmp/ && ssh {} 'sudo mv /tmp/{unit_name} /etc/systemd/system/ && sudo systemctl enable --now {unit_name}'", config.target, config.target);
    println!();
    println!("[Unit]");
    println!("Description=HORUS project deployed to {}", config.target);
    println!("After=network-online.target");
    println!("Wants=network-online.target");
    println!();
    println!("[Service]");
    println!("Type=simple");
    // Only when it is absolute. systemd does not do tilde expansion, so
    // `WorkingDirectory=~/horus_deploy` fails the unit outright with "Failed to
    // determine working directory". The default remote_dir is `~/horus_deploy`,
    // so that is the common case, not an edge one — and it costs nothing to
    // omit, because the command below is run through a login shell that starts
    // with the same `cd` the deploy itself uses.
    if config.remote_dir.starts_with('/') {
        println!("WorkingDirectory={}", config.remote_dir);
    }
    println!("ExecStart=/bin/sh -lc {}", shell_single_quote(&remote_cmd));
    println!("Restart=on-failure");
    println!("RestartSec=5");
    println!();
    println!("[Install]");
    println!("WantedBy=default.target");

    Ok(())
}

/// The unit filename for a deploy, derived from the remote directory.
///
/// The directory rather than the host: one robot can hold several deployed
/// projects, and naming the unit after the target would make the second deploy
/// silently replace the first one's service.
fn service_unit_name(config: &DeployConfig) -> String {
    let base = config
        .remote_dir
        .rsplit('/')
        .find(|part| !part.is_empty() && *part != "~")
        .unwrap_or("horus");
    let sanitized: String = base
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '-'
            }
        })
        .collect();
    format!("horus-{}.service", sanitized.trim_matches('-'))
}

/// Print what would be done in dry-run mode
fn print_deploy_plan(config: &DeployConfig) {
    let target = config.arch.rust_target();
    let mode = if config.release { "--release" } else { "" };
    let language = detect_deploy_language();

    println!("  1. Build:");
    if language == Language::Python {
        println!("     (nothing — Python)");
    } else if target.is_empty() {
        println!("     cargo build {}", mode);
    } else {
        println!("     cargo build {} --target {}", mode, target);
    }

    println!();
    println!("  2. Sync files:");
    println!(
        "     rsync -avz --delete -e 'ssh -p {}' ./ {}",
        config.port,
        sync_destination(config)
    );
    // Spelling out the direction of --delete, because the plan is the last
    // thing printed before a real run does it: it is the *destination* that
    // loses files, and the destination is a directory on someone's robot.
    println!(
        "     {} everything under {} that is not in this directory is deleted",
        cli_output::ICON_WARN.yellow(),
        sync_destination(config).yellow()
    );

    let mut step = 2;

    if language == Language::Python {
        step += 1;
        println!();
        println!("  {}. Install dependencies on target:", step);
        println!(
            "     ssh -p {} {} '{}'",
            config.port,
            config.target,
            pip_install_command(config)
        );
    }

    if config.run_after {
        step += 1;
        println!();
        println!("  {}. Run on target:", step);
        println!(
            "     ssh -p {} {} '{}'",
            config.port,
            config.target,
            remote_run_command(config).0
        );
    }
}

/// Detect target architecture based on hostname hints
fn detect_target_arch(target: &str) -> TargetArch {
    let lower = target.to_lowercase();
    if lower.contains("jetson")
        || lower.contains("nano")
        || lower.contains("xavier")
        || lower.contains("orin")
        || lower.contains("pi4")
        || lower.contains("pi5")
        || lower.contains("raspberry")
    {
        TargetArch::Aarch64
    } else if lower.contains("pi3") || lower.contains("pi2") {
        TargetArch::Armv7
    } else {
        // Default to aarch64 as most modern robot boards use it
        TargetArch::Aarch64
    }
}

/// Detect the primary project language for deploy purposes.
///
/// Uses `detect_languages()` and returns the first detected language.
/// Falls back to `Language::Rust` if nothing is detected.
fn detect_deploy_language() -> Language {
    let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let languages = detect_languages(&cwd);
    languages.into_iter().next().unwrap_or(Language::Rust)
}

/// Build the project for target architecture
fn build_for_target(config: &DeployConfig) -> HorusResult<()> {
    let language = detect_deploy_language();

    match language {
        Language::Rust => build_for_target_rust(config),
        Language::Python => {
            println!(
                "  {} Python project — no build step needed",
                cli_output::ICON_INFO.cyan()
            );
            Ok(())
        }
        Language::Cpp => build_for_target_cpp(config),
        Language::Ros2 => {
            // ROS2 projects use colcon; for deploy, treat like C++ with cmake
            build_for_target_cpp(config)
        }
    }
}

/// Build a Rust project for the target architecture
fn build_for_target_rust(config: &DeployConfig) -> HorusResult<()> {
    let target = config.arch.rust_target();

    // Check if cross-compilation target is installed
    if !target.is_empty() {
        print!(
            "  {} Checking target {}... ",
            cli_output::ICON_INFO.cyan(),
            target
        );
        let check = Command::new("rustup")
            .args(["target", "list", "--installed"])
            .output();

        match check {
            Ok(output) => {
                let installed = String::from_utf8_lossy(&output.stdout);
                if !installed.contains(target) {
                    println!("{}", "not installed".yellow());
                    println!("  {} Installing target...", cli_output::ICON_INFO.cyan());

                    let install = Command::new("rustup")
                        .args(["target", "add", target])
                        .status();

                    if install.map(|s| !s.success()).unwrap_or(true) {
                        return Err(HorusError::Config(ConfigError::Other(format!(
                            "Failed to install target {}. Run: rustup target add {}",
                            target, target
                        ))));
                    }
                    println!("  {} Target installed", cli_output::ICON_SUCCESS.green());
                } else {
                    println!("{}", "OK".green());
                }
            }
            Err(_) => {
                println!("{}", "rustup not found".yellow());
            }
        }
    }

    // Build the project.
    //
    // A stock `horus new -r` project has no root Cargo.toml — horus generates
    // one into `.horus/` at build time. Invoking bare `cargo build` here
    // therefore failed on the standard project layout, sixty seconds after the
    // user watched `horus build` generate that exact file:
    //
    //     error: could not find `Cargo.toml` in .../rustbot or any parent
    //
    // Point cargo at the generated manifest when it exists, and fall back to
    // the root one for projects that keep a hand-written Cargo.toml.
    let mut cmd = Command::new("cargo");
    cmd.arg("build");

    let generated_manifest = std::path::Path::new(".horus").join("Cargo.toml");
    if generated_manifest.is_file() {
        cmd.args(["--manifest-path", &generated_manifest.to_string_lossy()]);
    } else if !std::path::Path::new("Cargo.toml").is_file() {
        return Err(HorusError::Config(ConfigError::Other(
            "No Cargo.toml found. Run `horus build` first so the manifest in \
             .horus/ is generated, or add a Cargo.toml to the project root."
                .to_string(),
        )));
    }

    if config.release {
        cmd.arg("--release");
    }

    if !target.is_empty() {
        cmd.args(["--target", target]);
    }

    print!("  {} Building", cli_output::ICON_INFO.cyan());
    if !target.is_empty() {
        print!(" for {}", config.arch.display_name());
    }
    println!("...");

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd.status().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!("Failed to run cargo: {}", e)))
    })?;

    if !status.success() {
        return Err(HorusError::Config(ConfigError::Other(
            "Build failed".to_string(),
        )));
    }

    println!("  {} Build complete", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Build a C++ project for deployment.
///
/// Delegates to the same builder `horus run` and `horus build` use. The
/// re-implementation that used to live here was wrong in three ways at once,
/// and every one of them shipped:
///
/// * It configured with `-S .`, but `horus new --cpp` writes the generated
///   `CMakeLists.txt` into `.horus/`. Deploying a freshly generated project
///   therefore failed outright with "does not appear to contain
///   CMakeLists.txt".
/// * It skipped `cmake configure` whenever `CMakeCache.txt` already existed.
///   `CMAKE_BUILD_TYPE` is a configure-time variable and `--config` is inert
///   for single-config generators, so after the documented `horus new && horus
///   run` — which leaves the cache configured Debug — `horus deploy` produced a
///   **Debug** binary while printing "(Release)", and rsync shipped all 101 MB
///   of it to the robot.
/// * It passed neither `HORUS_CPP_INCLUDE` nor `HORUS_CPP_LIB`, so the
///   generated `CMakeLists.txt` could not find the bindings it consumes.
///
/// `build_cpp` configures unconditionally, resolves the bindings, and takes the
/// target architecture, which this path also needs for `--arch`.
fn build_for_target_cpp(config: &DeployConfig) -> HorusResult<()> {
    let project_dir = std::env::current_dir().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to resolve project directory: {e}"
        )))
    })?;

    // `build_cpp` takes the short name the `[cpp].toolchain` table is keyed by;
    // a native build passes None so it falls back to that table.
    let arch = match config.arch {
        TargetArch::Aarch64 => Some("aarch64"),
        TargetArch::Armv7 => Some("armv7"),
        TargetArch::X86_64 => Some("x86_64"),
        TargetArch::Native => None,
    };
    crate::commands::run::run_cpp::build_cpp(&project_dir, config.release, arch)
        .map(|binary| {
            println!(
                "  {} Build complete ({})",
                cli_output::ICON_SUCCESS.green(),
                binary.display()
            );
        })
        .map_err(|e| HorusError::Config(ConfigError::Other(format!("C++ build failed: {e:#}"))))
}

/// Sync files to target using rsync
fn sync_to_target(config: &DeployConfig) -> HorusResult<bool> {
    // Check if rsync is available
    if Command::new("rsync").arg("--version").output().is_err() {
        return Err(HorusError::Config(ConfigError::Other(
            "rsync not found. Please install rsync.".to_string(),
        )));
    }

    // Safety check: show what directory will be synced and confirm
    if !config.skip_confirm {
        let cwd = std::env::current_dir()
            .map(|p| p.display().to_string())
            .unwrap_or_else(|_| ".".to_string());
        println!(
            "  {} Will sync '{}' to {}:{}/ (with --delete)",
            cli_output::ICON_WARN.yellow(),
            cwd,
            config.target,
            config.remote_dir
        );
        println!(
            "  {} Files on remote not present locally will be DELETED",
            cli_output::ICON_WARN.yellow()
        );
        print!("  Continue? [y/N] ");
        std::io::Write::flush(&mut std::io::stdout()).ok();
        let mut input = String::new();
        std::io::stdin().read_line(&mut input).ok();
        if !input.trim().eq_ignore_ascii_case("y") {
            println!("  Cancelled.");
            return Ok(false);
        }
    }

    // Build rsync command
    let mut cmd = Command::new("rsync");
    cmd.args(["-avz", "--delete", "--progress"]);

    // Add excludes
    for exclude in &config.excludes {
        cmd.args(["--exclude", exclude]);
    }

    // SSH options with ConnectTimeout and shell-escaped identity path
    let ssh_cmd = if let Some(ref identity) = config.identity {
        let escaped = identity.display().to_string().replace('\'', "'\\''");
        format!(
            "ssh -p {} -o ConnectTimeout=30 -i '{}'",
            config.port, escaped
        )
    } else {
        format!("ssh -p {} -o ConnectTimeout=30", config.port)
    };
    cmd.args(["-e", &ssh_cmd]);

    // Source and destination
    cmd.arg("./");
    cmd.arg(format!("{}:{}/", config.target, config.remote_dir));

    println!("  {} Syncing files...", cli_output::ICON_INFO.cyan());

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd.status().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!("Failed to run rsync: {}", e)))
    })?;

    if !status.success() {
        return Err(HorusError::Config(ConfigError::Other(
            "rsync failed".to_string(),
        )));
    }

    println!("  {} Files synced", cli_output::ICON_SUCCESS.green());

    sync_binary_to_target(config)?;

    Ok(true)
}

/// Transfer the built Rust binary, which the main sync deliberately excludes.
///
/// The `target` exclude keeps ~370 MB of build intermediates off the wire, but
/// rsync matches that pattern against any path component, so it also caught
/// `.horus/target/` — the one directory holding the artifact. The deploy
/// shipped 37 KB of source, skipped the 2.8 MB executable, then exec'd a path
/// that had never been transferred and reported success.
///
/// A second explicit transfer is used rather than `--include` rules because
/// rsync descends into any directory a rule includes: the wildcard needed to
/// cover cross-compiled target triples re-admitted the entire tree (measured at
/// 1,229 files / 367,917,030 bytes).
///
/// Absent a binary — a Python project, or a build that has not run — this is a
/// no-op, so C++ and Python deploys are unaffected.
fn sync_binary_to_target(config: &DeployConfig) -> HorusResult<()> {
    let Some(local) = locate_built_binary(config) else {
        return Ok(());
    };

    println!(
        "  {} Transferring binary ({})",
        cli_output::ICON_INFO.cyan(),
        local.display()
    );

    // `--relative` sends the path as given and recreates that structure under
    // the destination, so `.horus/target/release/<bin>` lands at exactly the
    // path the run step later execs. It also creates the intermediate
    // directories, which a plain transfer will not — and unlike
    // `--rsync-path "mkdir -p ..."` it works for a local destination too.
    let mut cmd = Command::new("rsync");
    cmd.args(["-avz", "--relative"]);
    // One -e carries both the port and the identity; passing the flag twice
    // would leave only the last, silently dropping the other.
    let mut ssh = String::from("ssh");
    if config.port != 22 {
        ssh.push_str(&format!(" -p {}", config.port));
    }
    if let Some(ref identity) = config.identity {
        ssh.push_str(&format!(" -i {}", identity.display()));
    }
    if ssh != "ssh" {
        cmd.args(["-e", &ssh]);
    }

    cmd.arg(local.to_string_lossy().to_string());
    cmd.arg(format!("{}:{}/", config.target, config.remote_dir));

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd.status().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to run rsync for the binary: {e}"
        )))
    })?;

    if !status.success() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Failed to transfer the built binary ({}). The robot would have \
             received source without an executable.",
            local.display()
        ))));
    }

    println!("  {} Binary transferred", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Find the built Rust binary for the configured profile.
///
/// Checks the cross-compiled location first, then the host one, mirroring
/// where `build_for_target` puts them.
fn locate_built_binary(config: &DeployConfig) -> Option<std::path::PathBuf> {
    let name = find_binary_name()?;
    locate_built_binary_in(std::path::Path::new("."), &name, config)
}

/// [`locate_built_binary`] rooted at an explicit directory.
///
/// Split out so tests can point at a fixture without `set_current_dir`, which
/// is process-global: the first version of these tests raced two unrelated
/// `config` tests that also resolve paths relative to the working directory,
/// and failed them.
fn locate_built_binary_in(
    root: &std::path::Path,
    name: &str,
    config: &DeployConfig,
) -> Option<std::path::PathBuf> {
    let profile = if config.release { "release" } else { "debug" };
    let triple = config.arch.rust_target();

    let mut relatives = Vec::new();
    if !triple.is_empty() {
        relatives.push(
            std::path::PathBuf::from(".horus/target")
                .join(triple)
                .join(profile)
                .join(name),
        );
    }
    relatives.push(
        std::path::PathBuf::from(".horus/target")
            .join(profile)
            .join(name),
    );
    if !triple.is_empty() {
        relatives.push(
            std::path::PathBuf::from("target")
                .join(triple)
                .join(profile)
                .join(name),
        );
    }
    relatives.push(std::path::PathBuf::from("target").join(profile).join(name));

    relatives.into_iter().find(|rel| root.join(rel).is_file())
}

/// What starts this project, as a path inside the deploy directory.
///
/// Resolved once and consumed three ways — the shell command `--run` sends, the
/// dry-run plan, and `ExecStart=` in the unit [`print_service`] emits — so all
/// three name the same file. The run step and the transfer step used to compute
/// their paths independently, and shipped a binary to one place while exec'ing
/// another (see [`locate_built_binary`]).
enum RemoteEntry {
    /// A native executable at this project-relative path.
    Executable(String),
    /// A Python entry point at this project-relative path, run by the robot's
    /// own `python3`.
    Python(String),
}

/// The PyPI requirements this project resolved, as pip argument strings.
///
/// Read from `horus.lock` when there is one, so the robot installs the versions
/// the project was last known to work with rather than whatever PyPI serves on
/// the day it is provisioned — which is most of the point of deploying a fleet
/// from a checkout. Falls back to the manifest's declared dependencies when the
/// project has never been locked; those carry whatever constraint the author
/// wrote, including none.
fn deploy_python_requirements() -> Vec<String> {
    if let Ok(lock) =
        crate::lockfile::HorusLockfile::load_from(Path::new(crate::lockfile::HORUS_LOCK))
    {
        let pinned: Vec<String> = lock
            .packages
            .iter()
            .filter(|p| p.source == "pypi")
            .map(|p| format!("{}=={}", p.name, p.version))
            .collect();
        if !pinned.is_empty() {
            return pinned;
        }
    }

    let Ok(manifest) =
        crate::manifest::HorusManifest::load_from(Path::new(crate::manifest::HORUS_TOML))
    else {
        return Vec::new();
    };
    manifest
        .dependencies
        .iter()
        .filter(|(_, value)| value.is_pypi())
        .map(|(name, value)| match value.version() {
            // A bare version means that version, the way
            // `pyproject_gen::format_pypi_dep` renders it; anything starting
            // with an operator is already a requirement string.
            Some(spec) if !spec.is_empty() && spec != "latest" => {
                if spec.starts_with(['=', '>', '<', '~', '^', '!']) {
                    format!("{name}{spec}")
                } else {
                    format!("{name}=={spec}")
                }
            }
            _ => name.clone(),
        })
        .collect()
}

/// The pip invocation that populates [`REMOTE_PACKAGES_DIR`] on the robot.
///
/// `--target` rather than a virtualenv, and run on the robot rather than
/// shipped from here. The developer's `.horus/venv` is unusable on a robot —
/// its `pyvenv.cfg` records host-absolute paths and an interpreter symlink for
/// the wrong architecture — which is why the sync now excludes it. Wheels the
/// robot's own pip resolved are the only ones the robot's interpreter can load.
fn pip_install_command(config: &DeployConfig) -> String {
    let args = deploy_python_requirements()
        .iter()
        .map(|r| shell_single_quote(r))
        .collect::<Vec<_>>()
        .join(" ");

    format!(
        "cd {} && python3 -m pip install --upgrade --target {} {}",
        shell_quote_preserving_tilde(&config.remote_dir),
        shell_single_quote(REMOTE_PACKAGES_DIR),
        args
    )
}

/// Install the project's Python dependencies on the target.
///
/// Deploy used to be rsync and nothing else: `build_for_target` printed
/// "Python project — no build step needed", the run step invoked the robot's
/// system `python3`, and there was no `pip` anywhere in this file. So a Python
/// deploy reported success and then died on the first `import horus`, because
/// nothing had ever installed anything on the robot.
fn install_python_deps_on_target(config: &DeployConfig) -> HorusResult<()> {
    let requirements = deploy_python_requirements();
    if requirements.is_empty() {
        println!(
            "  {} No Python dependencies declared — nothing to install",
            cli_output::ICON_INFO.cyan()
        );
        return Ok(());
    }

    println!(
        "  {} Installing {} package(s) into {}",
        cli_output::ICON_INFO.cyan(),
        requirements.len(),
        REMOTE_PACKAGES_DIR
    );

    let mut cmd = ssh_base(config);
    cmd.arg(&config.target);
    cmd.arg(pip_install_command(config));
    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd
        .status()
        .map_err(|e| HorusError::Config(ConfigError::Other(format!("Failed to run SSH: {}", e))))?;

    if !status.success() {
        // Worth failing the deploy over. The files are already on the robot and
        // `--run` is about to start a node whose imports cannot resolve, which
        // surfaces as a traceback that never mentions pip.
        return Err(HorusError::Config(ConfigError::Other(format!(
            "pip install failed on {} — the project is synced but its dependencies are not installed",
            config.target
        ))));
    }

    println!(
        "  {} Dependencies installed",
        cli_output::ICON_SUCCESS.green()
    );
    Ok(())
}

/// Resolve [`RemoteEntry`] for the configured language.
fn remote_entry(config: &DeployConfig) -> RemoteEntry {
    match detect_deploy_language() {
        Language::Rust => {
            // Derive the remote path from the artifact that was actually
            // transferred. `sync_binary_to_target` sends it with `--relative`,
            // so the remote layout mirrors the local one exactly.
            let path = match locate_built_binary(config) {
                Some(p) => p.display().to_string(),
                None => {
                    let binary_name =
                        find_binary_name().unwrap_or_else(|| "horus_project".to_string());
                    let target = config.arch.rust_target();
                    let mode = if config.release { "release" } else { "debug" };
                    if target.is_empty() {
                        format!(".horus/target/{}/{}", mode, binary_name)
                    } else {
                        format!(".horus/target/{}/{}/{}", target, mode, binary_name)
                    }
                }
            };
            RemoteEntry::Executable(path)
        }
        Language::Python => {
            RemoteEntry::Python(find_python_entry().unwrap_or_else(|| "main.py".to_string()))
        }
        Language::Cpp | Language::Ros2 => {
            let binary_name = find_cpp_binary().unwrap_or_else(|| "horus_project".to_string());
            RemoteEntry::Executable(format!(".horus/cpp-build/{}", binary_name))
        }
    }
}

/// The full command `--run` sends over ssh, and the short name to print for it.
fn remote_run_command(config: &DeployConfig) -> (String, String) {
    let (start, display) = match remote_entry(config) {
        RemoteEntry::Executable(path) => (
            shell_single_quote(&format!("./{path}")),
            format!("./{path}"),
        ),
        RemoteEntry::Python(entry) => (
            // The robot's system python3 has never heard of `horus`;
            // install_python_deps_on_target put the wheels in
            // `.horus/packages`, and PYTHONPATH is the whole of how the
            // interpreter is told about them. Absolute (`$PWD`, after the cd)
            // so a node that changes directory keeps its imports, and prepended
            // to whatever the robot already sets rather than replacing it.
            format!(
                "PYTHONPATH=\"$PWD/{}${{PYTHONPATH:+:$PYTHONPATH}}\" python3 {}",
                REMOTE_PACKAGES_DIR,
                shell_single_quote(&entry)
            ),
            format!("python3 {entry}"),
        ),
    };

    // A leading `~` must stay outside the quotes or the remote shell will not
    // expand it: `cd '~/horus_deploy'` fails with "can't cd to ~/horus_deploy"
    // even though rsync's own destination (`host:~/horus_deploy/`) *was*
    // expanded, so the files are there and the run step cannot reach them.
    (
        format!(
            "cd {} && {}",
            shell_quote_preserving_tilde(&config.remote_dir),
            start
        ),
        display,
    )
}

/// An `ssh` invocation carrying this deploy's port, timeout and identity.
///
/// The destination is added by the caller, after [`validate_ssh_inputs`] has
/// refused anything ssh would read as an option instead.
fn ssh_base(config: &DeployConfig) -> Command {
    let mut cmd = Command::new("ssh");
    cmd.args(["-p", &config.port.to_string()]);
    cmd.args(["-o", "ConnectTimeout=30"]);
    if let Some(ref identity) = config.identity {
        cmd.args(["-i", &identity.to_string_lossy()]);
    }
    cmd
}

/// Run the project on the target
fn run_on_target(config: &DeployConfig) -> HorusResult<()> {
    let (remote_cmd, display_name) = remote_run_command(config);

    let mut cmd = ssh_base(config);

    // Allocate a TTY for interactive use
    cmd.arg("-t");
    cmd.arg(&config.target);
    cmd.arg(&remote_cmd);

    println!(
        "  {} Running: {}",
        cli_output::ICON_INFO.cyan(),
        display_name
    );
    println!("  {} Press Ctrl+C to stop", cli_output::ICON_HINT.dimmed());
    println!();

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());
    cmd.stdin(Stdio::inherit());

    let status = cmd
        .status()
        .map_err(|e| HorusError::Config(ConfigError::Other(format!("Failed to run SSH: {}", e))))?;

    if !status.success() {
        // Don't treat interrupt as error
        let code = status.code().unwrap_or(0);
        if code != 130 && code != 0 {
            // 130 = Ctrl+C
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Remote execution failed with code {}",
                code
            ))));
        }
    }

    Ok(())
}

/// Find the Python entry point for the project.
///
/// Checks horus.toml [scripts] for a "run" or "start" entry, then
/// falls back to common entry point paths (src/main.py, main.py).
fn find_python_entry() -> Option<String> {
    use crate::manifest::{HorusManifest, HORUS_TOML};

    // Try horus.toml [scripts] for a run/start entry
    let manifest_path = Path::new(HORUS_TOML);
    if manifest_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
            // Check scripts for python entry hints
            for key in &["run", "start", "main"] {
                if let Some(script) = manifest.scripts.get(*key) {
                    // Extract the Python file from the script command
                    // e.g., "python3 src/app.py" -> "src/app.py"
                    if let Some(py_file) = extract_python_file(script) {
                        return Some(py_file);
                    }
                }
            }
        }
    }

    // Fall back to common entry points
    for candidate in &["src/main.py", "main.py"] {
        if Path::new(candidate).exists() {
            return Some(candidate.to_string());
        }
    }

    None
}

/// Extract a Python file path from a script command string.
///
/// Handles patterns like `python3 src/app.py` or `python src/main.py --verbose`.
fn extract_python_file(script: &str) -> Option<String> {
    let parts: Vec<&str> = script.split_whitespace().collect();
    for (i, part) in parts.iter().enumerate() {
        if (*part == "python3" || *part == "python") && i + 1 < parts.len() {
            let candidate = parts[i + 1];
            if candidate.ends_with(".py") {
                return Some(candidate.to_string());
            }
        }
    }
    // Maybe the script is just a .py file path
    if let Some(first) = parts.first() {
        if first.ends_with(".py") {
            return Some(first.to_string());
        }
    }
    None
}

/// Find the C++ binary name from CMakeLists.txt or cmake build artifacts.
///
/// Looks for `add_executable(name ...)` in CMakeLists.txt, then checks the
/// cmake build directory for executables.
fn find_cpp_binary() -> Option<String> {
    // Try to extract from CMakeLists.txt
    let cmake_path = Path::new("CMakeLists.txt");
    if cmake_path.exists() {
        if let Ok(content) = std::fs::read_to_string(cmake_path) {
            for line in content.lines() {
                let trimmed = line.trim();
                if let Some(rest) = trimmed.strip_prefix("add_executable(") {
                    // add_executable(my_app src/main.cpp)
                    if let Some(name) = rest.split_whitespace().next() {
                        let name = name.trim_end_matches(')');
                        if !name.is_empty() {
                            return Some(name.to_string());
                        }
                    }
                }
            }
        }
    }

    // Fall back to horus.toml package name
    use crate::manifest::{HorusManifest, HORUS_TOML};
    let manifest_path = Path::new(HORUS_TOML);
    if manifest_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
            if !manifest.package.name.is_empty() {
                return Some(manifest.package.name.replace('-', "_"));
            }
        }
    }

    None
}

/// Find the binary name from horus.toml, falling back to Cargo.toml
fn find_binary_name() -> Option<String> {
    // Try horus.toml first (single source of truth)
    use crate::manifest::{HorusManifest, HORUS_TOML};
    let manifest_path = Path::new(HORUS_TOML);
    if manifest_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
            if !manifest.package.name.is_empty() {
                return Some(manifest.package.name.replace('-', "_"));
            }
        }
    }

    // Fall back to Cargo.toml (legacy or non-horus projects)
    let cargo_toml = Path::new(CARGO_TOML);
    if !cargo_toml.exists() {
        return None;
    }

    let content = std::fs::read_to_string(cargo_toml).ok()?;
    // toml 1.1 changed `FromStr for Value` to parse a single TOML *value*
    // (ValueDeserializer), not a document — only `Table` still parses a
    // document via FromStr. `toml::from_str` keeps the document semantics.
    let parsed: toml::Value = toml::from_str(&content).ok()?;

    // Try [[bin]] name first
    if let Some(bins) = parsed.get("bin").and_then(|b| b.as_array()) {
        if let Some(first_bin) = bins.first() {
            if let Some(name) = first_bin.get("name").and_then(|n| n.as_str()) {
                return Some(name.to_string());
            }
        }
    }

    // Fall back to [package].name
    parsed
        .get("package")
        .and_then(|p| p.get("name"))
        .and_then(|n| n.as_str())
        .map(|s| s.to_string())
}

/// List available deployment targets from config
pub fn list_targets() -> HorusResult<()> {
    println!("{}", "Deployment Targets".green().bold());
    println!();

    // Try to parse .horus/deploy.yaml
    if let Some(yaml) = load_deploy_yaml() {
        if yaml.targets.is_empty() {
            println!(
                "  {} .horus/deploy.yaml exists but has no targets.",
                cli_output::ICON_WARN.yellow()
            );
        } else {
            // Collect and sort target names for stable output
            let mut names: Vec<&String> = yaml.targets.keys().collect();
            names.sort();

            // Print formatted table
            println!(
                "  {:<14} {:<28} {:<10} {:<20} {:<6} {}",
                "NAME".bold(),
                "HOST".bold(),
                "ARCH".bold(),
                "DIR".bold(),
                "PORT".bold(),
                "IDENTITY".bold(),
            );
            println!("  {}", "-".repeat(86));

            for name in names {
                let t = &yaml.targets[name];
                println!(
                    "  {:<14} {:<28} {:<10} {:<20} {:<6} {}",
                    name.cyan(),
                    t.host,
                    t.arch.as_deref().unwrap_or("-"),
                    t.dir.as_deref().unwrap_or("~/horus_deploy"),
                    t.port.map(|p| p.to_string()).unwrap_or_else(|| "22".into()),
                    t.identity.as_deref().unwrap_or("-"),
                );
            }

            println!();
            println!(
                "  {} horus deploy <NAME> to deploy to a named target",
                "Usage:".cyan()
            );
        }
    } else {
        println!("  {}", "No deployment targets configured.".dimmed());
        println!();
        println!(
            "  {} Create .horus/deploy.yaml to save targets:",
            "Tip:".cyan()
        );
        println!();
        println!("    targets:");
        println!("      robot:");
        println!("        host: pi@192.168.1.100");
        println!("        arch: aarch64");
        println!("        dir: ~/my_robot");
        println!("      jetson:");
        println!("        host: nvidia@jetson.local");
        println!("        arch: aarch64");
        println!("        dir: ~/horus_app");
    }

    println!();
    println!("  {}", "Supported architectures:".cyan());
    println!("    aarch64  - Raspberry Pi 4/5, Jetson Nano/Xavier/Orin");
    println!("    armv7    - Raspberry Pi 2/3, older ARM boards");
    println!("    x86_64   - Intel NUC, standard PCs");
    println!("    native   - Same as build host");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── TargetArch parsing ───────────────────────────────────────────────

    #[test]
    fn target_arch_aarch64_aliases() {
        for alias in &["aarch64", "arm64", "jetson", "pi4", "pi5"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "aarch64-unknown-linux-gnu");
        }
    }

    #[test]
    fn target_arch_armv7_aliases() {
        for alias in &["armv7", "arm", "pi3", "pi2"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "armv7-unknown-linux-gnueabihf");
        }
    }

    #[test]
    fn target_arch_x86_64_aliases() {
        for alias in &["x86_64", "x64", "amd64", "intel"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "x86_64-unknown-linux-gnu");
        }
    }

    #[test]
    fn target_arch_native_aliases() {
        for alias in &["native", "host", "local"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "");
        }
    }

    #[test]
    fn target_arch_unknown_returns_none() {
        assert!(TargetArch::from_str("mips").is_none());
        assert!(TargetArch::from_str("riscv").is_none());
        assert!(TargetArch::from_str("").is_none());
    }

    #[test]
    fn target_arch_case_insensitive() {
        assert!(TargetArch::from_str("AARCH64").is_some());
        assert!(TargetArch::from_str("Pi4").is_some());
        assert!(TargetArch::from_str("X86_64").is_some());
    }

    #[test]
    fn target_arch_display_names() {
        assert_eq!(TargetArch::Aarch64.display_name(), "ARM64 (aarch64)");
        assert_eq!(TargetArch::Armv7.display_name(), "ARM32 (armv7)");
        assert_eq!(TargetArch::X86_64.display_name(), "x86_64");
        assert_eq!(TargetArch::Native.display_name(), "native");
    }

    // ── ssh/rsync argv validation ────────────────────────────────────────

    /// `cmd.arg(&config.target)` is a bare positional to ssh, so a host that
    /// begins with `-` is parsed as an option — and `-oProxyCommand=…` makes
    /// ssh run that string through /bin/sh. The value can come from the
    /// checkout's own .horus/deploy.yaml.
    #[test]
    fn ssh_host_that_would_be_parsed_as_an_option_is_refused() {
        for host in [
            "-oProxyCommand=touch /tmp/pwn",
            "-lroot",
            "pi@robot -oProxyCommand=id",
            "",
            "   ",
        ] {
            assert!(
                validate_ssh_inputs(host, None).is_err(),
                "host {host:?} must not reach ssh argv"
            );
        }
    }

    /// rsync splits its `-e` command on whitespace and honours no quotes, so a
    /// space anywhere in the identity path adds argv elements to the ssh
    /// invocation it builds.
    #[test]
    fn identity_that_injects_ssh_arguments_is_refused() {
        for identity in ["/home/u/k -oProxyCommand=id", "-oProxyCommand=id", ""] {
            assert!(
                validate_ssh_inputs("pi@robot", Some(Path::new(identity))).is_err(),
                "identity {identity:?} must not reach rsync -e"
            );
        }
    }

    /// Everything a real deploy.yaml holds still works.
    #[test]
    fn ordinary_deploy_targets_are_accepted() {
        // `unwrap` rather than `assert!(..is_ok())`: on a regression the panic
        // carries the rejection reason, which is the whole diagnostic.
        validate_ssh_inputs("pi@192.168.1.100", None).unwrap();
        validate_ssh_inputs("robot", None).unwrap();
        validate_ssh_inputs(
            "nvidia@jetson.local",
            Some(Path::new("/home/u/.ssh/robot_key")),
        )
        .unwrap();
    }

    // ── DeployYaml parsing ───────────────────────────────────────────────

    #[test]
    fn deploy_yaml_parse() {
        let yaml = r#"
targets:
  robot:
    host: pi@192.168.1.100
    arch: aarch64
    dir: ~/my_robot
    port: 2222
    identity: ~/.ssh/robot_key
  jetson:
    host: nvidia@jetson.local
"#;
        let config: DeployYaml = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.targets.len(), 2);

        let robot = &config.targets["robot"];
        assert_eq!(robot.host, "pi@192.168.1.100");
        assert_eq!(robot.arch.as_deref(), Some("aarch64"));
        assert_eq!(robot.dir.as_deref(), Some("~/my_robot"));
        assert_eq!(robot.port, Some(2222));
        assert_eq!(robot.identity.as_deref(), Some("~/.ssh/robot_key"));

        let jetson = &config.targets["jetson"];
        assert_eq!(jetson.host, "nvidia@jetson.local");
        assert!(jetson.arch.is_none());
        assert!(jetson.dir.is_none());
        assert!(jetson.port.is_none());
    }

    // ── resolve_target ─────────────────────────────────────────────────

    #[test]
    fn resolve_target_direct_user_at_host() {
        let resolved = resolve_target("pi@192.168.1.100");
        assert_eq!(resolved.host, "pi@192.168.1.100");
        assert!(resolved.arch.is_none());
        assert!(resolved.dir.is_none());
        assert!(resolved.port.is_none());
        assert!(resolved.identity.is_none());
    }

    #[test]
    fn resolve_target_bare_hostname() {
        // Without @ and without YAML, returns as-is
        let resolved = resolve_target("unknown-host-xyz");
        assert_eq!(resolved.host, "unknown-host-xyz");
    }

    // ── detect_target_arch ─────────────────────────────────────────────

    #[test]
    fn detect_arch_jetson_keywords() {
        assert!(matches!(
            detect_target_arch("nvidia@jetson"),
            TargetArch::Aarch64
        ));
        assert!(matches!(
            detect_target_arch("user@nano.local"),
            TargetArch::Aarch64
        ));
        assert!(matches!(
            detect_target_arch("xavier-01"),
            TargetArch::Aarch64
        ));
        assert!(matches!(detect_target_arch("orin-nx"), TargetArch::Aarch64));
    }

    #[test]
    fn detect_arch_raspberry_pi_keywords() {
        assert!(matches!(
            detect_target_arch("pi4-robot"),
            TargetArch::Aarch64
        ));
        assert!(matches!(detect_target_arch("pi5"), TargetArch::Aarch64));
        assert!(matches!(
            detect_target_arch("raspberry-pi"),
            TargetArch::Aarch64
        ));
    }

    #[test]
    fn detect_arch_pi3_is_armv7() {
        assert!(matches!(detect_target_arch("pi3-old"), TargetArch::Armv7));
        assert!(matches!(detect_target_arch("pi2"), TargetArch::Armv7));
    }

    #[test]
    fn detect_arch_unknown_defaults_to_aarch64() {
        // Unknown hostnames default to aarch64 (most modern robot boards)
        assert!(matches!(
            detect_target_arch("some-robot"),
            TargetArch::Aarch64
        ));
    }

    // ── DeployConfig defaults ──────────────────────────────────────────

    #[test]
    fn deploy_config_defaults() {
        let config = DeployConfig::default();
        assert_eq!(config.target, "");
        assert_eq!(config.remote_dir, "~/horus_deploy");
        assert!(matches!(config.arch, TargetArch::Aarch64));
        assert!(!config.run_after);
        assert!(config.release);
        assert_eq!(config.port, 22);
        assert!(config.identity.is_none());
        assert!(config.excludes.is_empty());
    }

    // ── find_binary_name ───────────────────────────────────────────────

    #[test]
    fn find_binary_name_no_cargo_toml() {
        // Test runs from workspace root which is a virtual workspace (no [[bin]]).
        // find_binary_name reads ./Cargo.toml for [[bin]] or [package].name.
        let result = find_binary_name();
        // Workspace root has no [[bin]] target, so expect None.
        // If somehow Some, the name must be a valid non-empty identifier.
        if let Some(ref name) = result {
            assert!(!name.is_empty(), "Binary name must not be empty");
            assert!(
                !name.contains(' '),
                "Binary name must not contain spaces: '{}'",
                name
            );
        }
    }

    // ── Rust target strings ────────────────────────────────────────────

    #[test]
    fn rust_target_strings_are_valid() {
        assert_eq!(
            TargetArch::Aarch64.rust_target(),
            "aarch64-unknown-linux-gnu"
        );
        assert_eq!(
            TargetArch::Armv7.rust_target(),
            "armv7-unknown-linux-gnueabihf"
        );
        assert_eq!(TargetArch::X86_64.rust_target(), "x86_64-unknown-linux-gnu");
        assert_eq!(TargetArch::Native.rust_target(), "");
    }

    // ── DeployYaml edge cases ──────────────────────────────────────────

    #[test]
    fn deploy_yaml_empty_targets() {
        let yaml = "targets: {}";
        let config: DeployYaml = serde_yaml::from_str(yaml).unwrap();
        assert!(config.targets.is_empty());
    }

    #[test]
    fn deploy_yaml_minimal_target() {
        let yaml = r#"
targets:
  dev:
    host: localhost
"#;
        let config: DeployYaml = serde_yaml::from_str(yaml).unwrap();
        let dev = &config.targets["dev"];
        assert_eq!(dev.host, "localhost");
        assert!(dev.arch.is_none());
        assert!(dev.dir.is_none());
        assert!(dev.port.is_none());
        assert!(dev.identity.is_none());
    }

    // ── extract_python_file ────────────────────────────────────────────

    #[test]
    fn extract_python_file_standard() {
        assert_eq!(
            extract_python_file("python3 src/app.py"),
            Some("src/app.py".to_string())
        );
    }

    #[test]
    fn extract_python_file_python2_syntax() {
        assert_eq!(
            extract_python_file("python main.py"),
            Some("main.py".to_string())
        );
    }

    #[test]
    fn extract_python_file_with_flags_after() {
        assert_eq!(
            extract_python_file("python3 main.py --verbose --port 8080"),
            Some("main.py".to_string())
        );
    }

    #[test]
    fn extract_python_file_bare_py_file() {
        assert_eq!(extract_python_file("app.py"), Some("app.py".to_string()));
    }

    #[test]
    fn extract_python_file_non_python_returns_none() {
        assert_eq!(extract_python_file("node index.js"), None);
    }

    #[test]
    fn extract_python_file_empty_string() {
        assert_eq!(extract_python_file(""), None);
    }

    #[test]
    fn extract_python_file_python3_without_file() {
        // "python3" alone without a .py file after it
        assert_eq!(extract_python_file("python3 --version"), None);
    }

    #[test]
    fn extract_python_file_nested_path() {
        assert_eq!(
            extract_python_file("python3 src/nodes/sensor.py"),
            Some("src/nodes/sensor.py".to_string())
        );
    }

    // ── find_binary_name (with tempdir) ────────────────────────────────

    #[test]
    fn find_binary_name_from_horus_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"my-robot\"\nversion = \"0.1.0\"\nedition = \"1\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_binary_name();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        // horus.toml name with hyphens → underscores for binary
        assert_eq!(result, Some("my_robot".to_string()));
    }

    #[test]
    fn find_binary_name_from_cargo_toml_bin() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo = r#"
[[bin]]
name = "sensor_node"
path = "src/main.rs"

[package]
name = "my-project"
version = "0.1.0"
"#;
        std::fs::write(tmp.path().join("Cargo.toml"), cargo).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_binary_name();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        // [[bin]].name takes precedence
        assert_eq!(result, Some("sensor_node".to_string()));
    }

    #[test]
    fn find_binary_name_empty_dir_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_binary_name();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    // ── find_python_entry (with tempdir) ───────────────────────────────

    #[test]
    fn find_python_entry_from_src_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::create_dir_all(tmp.path().join("src")).unwrap();
        std::fs::write(tmp.path().join("src/main.py"), "pass").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("src/main.py".to_string()));
    }

    #[test]
    fn find_python_entry_from_root_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), "pass").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("main.py".to_string()));
    }

    #[test]
    fn find_python_entry_empty_dir_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    #[test]
    fn find_python_entry_from_horus_toml_scripts() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml = r#"
[package]
name = "bot"
version = "0.1.0"
edition = "1"

[scripts]
run = "python3 src/robot.py"
"#;
        std::fs::write(tmp.path().join("horus.toml"), toml).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("src/robot.py".to_string()));
    }

    // ── find_cpp_binary (with tempdir) ─────────────────────────────────

    #[test]
    fn find_cpp_binary_from_cmake() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("CMakeLists.txt"),
            "add_executable(my_robot src/main.cpp)\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_cpp_binary();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("my_robot".to_string()));
    }

    #[test]
    fn find_cpp_binary_from_horus_toml_fallback() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"cpp-bot\"\nversion = \"0.1.0\"\nedition = \"1\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_cpp_binary();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("cpp_bot".to_string()));
    }

    #[test]
    fn find_cpp_binary_empty_dir_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_cpp_binary();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    // ── DeployConfig command construction ──────────────────────────────

    #[test]
    fn deploy_config_excludes_default_patterns() {
        let config = DeployConfig {
            excludes: vec![
                "target".to_string(),
                ".git".to_string(),
                "node_modules".to_string(),
                "__pycache__".to_string(),
                "*.pyc".to_string(),
            ],
            ..Default::default()
        };
        assert!(config.excludes.contains(&"target".to_string()));
        assert!(config.excludes.contains(&".git".to_string()));
        assert!(config.excludes.contains(&"__pycache__".to_string()));
        assert_eq!(config.excludes.len(), 5);
    }

    #[test]
    fn deploy_config_with_identity() {
        let config = DeployConfig {
            target: "pi@10.0.0.1".to_string(),
            identity: Some(PathBuf::from("/home/user/.ssh/robot_key")),
            ..Default::default()
        };
        assert_eq!(
            config.identity.unwrap(),
            PathBuf::from("/home/user/.ssh/robot_key")
        );
    }

    #[test]
    fn deploy_config_custom_port() {
        let config = DeployConfig {
            target: "pi@10.0.0.1".to_string(),
            port: 2222,
            ..Default::default()
        };
        assert_eq!(config.port, 2222);
    }

    #[test]
    fn deploy_config_debug_mode() {
        let config = DeployConfig {
            release: false,
            ..Default::default()
        };
        assert!(!config.release);
    }

    #[test]
    fn resolve_target_with_at_sign_bypasses_yaml() {
        // Any target containing '@' is treated as direct user@host
        let resolved = resolve_target("admin@10.0.0.5");
        assert_eq!(resolved.host, "admin@10.0.0.5");
        // No YAML fields should be populated
        assert!(resolved.arch.is_none());
        assert!(resolved.dir.is_none());
        assert!(resolved.port.is_none());
        assert!(resolved.identity.is_none());
    }

    // ── Fleet deployment (load_deploy_yaml + multi-target) ─────────────

    #[test]
    fn load_deploy_yaml_with_valid_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        std::fs::write(
            tmp.path().join(".horus/deploy.yaml"),
            "targets:\n  robot:\n    host: pi@10.0.0.1\n    arch: aarch64\n  jetson:\n    host: nvidia@10.0.0.2\n",
        ).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_deploy_yaml();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_some());
        let yaml = result.unwrap();
        assert_eq!(yaml.targets.len(), 2);
        assert!(yaml.targets.contains_key("robot"));
        assert!(yaml.targets.contains_key("jetson"));
    }

    #[test]
    fn load_deploy_yaml_missing_file_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_deploy_yaml();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    #[test]
    fn load_deploy_yaml_invalid_yaml_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        std::fs::write(
            tmp.path().join(".horus/deploy.yaml"),
            "{{{{invalid yaml content",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_deploy_yaml();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    #[test]
    fn deploy_multi_args_empty_targets_no_all() {
        let args = DeployMultiArgs {
            targets: vec![],
            all: false,
            parallel: false,
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: false,
            print_service: false,
        };
        let result = run_deploy_multi(args);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("No target"),
            "Expected 'No target' error, got: {err}"
        );
    }

    #[test]
    fn deploy_multi_args_all_without_yaml_errors() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let args = DeployMultiArgs {
            targets: vec![],
            all: true,
            parallel: false,
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: false,
            print_service: false,
        };
        let result = run_deploy_multi(args);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("No targets configured") || err.contains("deploy.yaml"),
            "Expected deploy.yaml error, got: {err}"
        );
    }

    #[test]
    fn deploy_yaml_target_resolution_merges_fields() {
        let yaml_str = r#"
targets:
  robot:
    host: pi@10.0.0.1
    arch: aarch64
    dir: ~/app
    port: 2222
    identity: ~/.ssh/id_robot
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let target = &yaml.targets["robot"];
        assert_eq!(target.host, "pi@10.0.0.1");
        assert_eq!(target.arch.as_deref(), Some("aarch64"));
        assert_eq!(target.dir.as_deref(), Some("~/app"));
        assert_eq!(target.port, Some(2222));
        assert_eq!(target.identity.as_deref(), Some("~/.ssh/id_robot"));
    }

    #[test]
    fn deploy_yaml_multiple_targets_sorted() {
        let yaml_str = r#"
targets:
  zebra:
    host: z@z.local
  alpha:
    host: a@a.local
  middle:
    host: m@m.local
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let mut names: Vec<String> = yaml.targets.keys().cloned().collect();
        names.sort();
        assert_eq!(names, vec!["alpha", "middle", "zebra"]);
    }

    #[test]
    fn deploy_yaml_target_minimal_only_host() {
        let yaml_str = r#"
targets:
  dev:
    host: user@localhost
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let dev = &yaml.targets["dev"];
        assert_eq!(dev.host, "user@localhost");
        assert!(dev.arch.is_none());
        assert!(dev.dir.is_none());
        assert!(dev.port.is_none());
        assert!(dev.identity.is_none());
    }

    #[test]
    fn deploy_args_dry_run_flag() {
        let args = DeployArgs {
            target: "pi@10.0.0.1".to_string(),
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: true,
            print_service: false,
        };
        // dry_run should print plan without executing
        // We can't easily capture stdout, but verify the struct is constructible
        assert!(args.dry_run);
        assert_eq!(args.target, "pi@10.0.0.1");
    }

    #[test]
    fn deploy_multi_args_parallel_flag() {
        let args = DeployMultiArgs {
            targets: vec!["robot1".to_string(), "robot2".to_string()],
            all: false,
            parallel: true,
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: false,
            print_service: false,
        };
        assert!(args.parallel);
        assert_eq!(args.targets.len(), 2);
    }

    // ── Intent tests: deploy target parsing ────────────────────────────────

    /// INTENT: Parse a deploy target string "user@192.168.1.100:/opt/robot".
    /// The user, host, and path components must be extracted correctly from
    /// the YAML config format.
    #[test]
    fn test_deploy_target_parsing() {
        let yaml_str = r#"
targets:
  robot:
    host: user@192.168.1.100
    dir: /opt/robot
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let target = &yaml.targets["robot"];

        // The host field carries user@host
        assert_eq!(target.host, "user@192.168.1.100");
        assert!(
            target.host.contains('@'),
            "Host must contain '@' separator for user@host format"
        );

        // Extract user and host parts
        let parts: Vec<&str> = target.host.splitn(2, '@').collect();
        assert_eq!(parts.len(), 2, "user@host must split into exactly 2 parts");
        assert_eq!(parts[0], "user", "User must be 'user'");
        assert_eq!(parts[1], "192.168.1.100", "Host must be '192.168.1.100'");

        // dir field carries the remote path
        assert_eq!(
            target.dir.as_deref(),
            Some("/opt/robot"),
            "Remote path must be '/opt/robot'"
        );
    }

    /// INTENT: Parse a deploy target with a custom port.
    /// Port must be correctly extracted from the YAML config.
    #[test]
    fn test_deploy_target_with_port() {
        let yaml_str = r#"
targets:
  robot:
    host: user@host
    port: 2222
    dir: /path
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let target = &yaml.targets["robot"];

        assert_eq!(target.port, Some(2222), "Port must be 2222");
        assert_eq!(target.host, "user@host");
        assert_eq!(target.dir.as_deref(), Some("/path"));
    }

    /// INTENT: Target architecture strings ("aarch64", "armv7", "x86_64")
    /// must all be recognized as valid architectures by TargetArch::from_str.
    /// Unrecognized strings must return None.
    #[test]
    fn test_deploy_arch_detection() {
        // All three primary robot architectures must be recognized
        let aarch64 = TargetArch::from_str("aarch64");
        assert!(
            aarch64.is_some(),
            "'aarch64' must be recognized as a valid architecture"
        );
        assert_eq!(
            aarch64.unwrap().rust_target(),
            "aarch64-unknown-linux-gnu",
            "aarch64 must map to the correct Rust target triple"
        );

        let armv7 = TargetArch::from_str("armv7");
        assert!(
            armv7.is_some(),
            "'armv7' must be recognized as a valid architecture"
        );
        assert_eq!(
            armv7.unwrap().rust_target(),
            "armv7-unknown-linux-gnueabihf",
            "armv7 must map to the correct Rust target triple"
        );

        let x86_64 = TargetArch::from_str("x86_64");
        assert!(
            x86_64.is_some(),
            "'x86_64' must be recognized as a valid architecture"
        );
        assert_eq!(
            x86_64.unwrap().rust_target(),
            "x86_64-unknown-linux-gnu",
            "x86_64 must map to the correct Rust target triple"
        );

        // Unknown architectures must return None (not panic or default)
        let unknown = TargetArch::from_str("riscv64");
        assert!(
            unknown.is_none(),
            "Unrecognized architecture 'riscv64' must return None"
        );

        let empty = TargetArch::from_str("");
        assert!(
            empty.is_none(),
            "Empty string must return None for architecture"
        );
    }
}

#[cfg(test)]
mod deploy_transfer_tests {
    use super::*;

    // ── The binary must survive the exclude list ────────────────────────────

    /// rsync matches a bare `target` against *any* path component, so the old
    /// exclude also matched `.horus/target/` — where `horus build` puts the
    /// Rust binary. The deploy shipped 37 KB of source, excluded the 2.8 MB
    /// executable it had just built, exec'd a path that was never transferred,
    /// and printed "Deployment complete!".
    /// The transfer path and the exec path must agree. They were computed
    /// independently — the run step assumed `./target/<mode>/<bin>` while
    /// `horus build` puts the artifact under `.horus/target/` — so even a
    /// fixed transfer would have been exec'd at the wrong path.
    #[test]
    fn the_run_path_matches_where_the_binary_is_shipped() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path().join(".horus/target/release");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("rbot"), b"\x7fELF").unwrap();

        let cfg = DeployConfig {
            release: true,
            ..Default::default()
        };
        let shipped =
            locate_built_binary_in(tmp.path(), "rbot", &cfg).map(|p| format!("./{}", p.display()));

        assert_eq!(
            shipped.as_deref(),
            Some("./.horus/target/release/rbot"),
            "the exec path must be the transferred path"
        );
    }

    /// The artifact the main sync excludes must be found so it can be
    /// transferred separately — otherwise the robot gets source and no
    /// executable.
    #[test]
    fn locate_built_binary_finds_the_generated_layout() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path().join(".horus/target/release");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("rbot"), b"\x7fELF").unwrap();

        let cfg = DeployConfig {
            release: true,
            ..Default::default()
        };
        assert_eq!(
            locate_built_binary_in(tmp.path(), "rbot", &cfg),
            Some(std::path::PathBuf::from(".horus/target/release/rbot"))
        );
    }

    /// A cross-compiled build lands under an extra target-triple directory and
    /// must be preferred over a stale host build.
    #[test]
    fn cross_compiled_layout_is_preferred() {
        let tmp = tempfile::tempdir().unwrap();
        let cfg = DeployConfig {
            release: true,
            arch: TargetArch::Aarch64,
            ..Default::default()
        };
        let triple = cfg.arch.rust_target();

        let host = tmp.path().join(".horus/target/release");
        std::fs::create_dir_all(&host).unwrap();
        std::fs::write(host.join("rbot"), b"stale").unwrap();

        let cross = tmp
            .path()
            .join(".horus/target")
            .join(triple)
            .join("release");
        std::fs::create_dir_all(&cross).unwrap();
        std::fs::write(cross.join("rbot"), b"\x7fELF").unwrap();

        let found = locate_built_binary_in(tmp.path(), "rbot", &cfg).unwrap();
        assert!(
            found.to_string_lossy().contains(triple),
            "cross build must win over a stale host build, got {}",
            found.display()
        );
    }

    /// A Python or C++ project has no Rust artifact; the extra transfer must
    /// be a no-op rather than an error.
    #[test]
    fn locate_built_binary_is_none_without_a_build() {
        let tmp = tempfile::tempdir().unwrap();
        let cfg = DeployConfig::default();
        assert!(locate_built_binary_in(tmp.path(), "pybot", &cfg).is_none());
    }

    #[test]
    fn build_intermediates_stay_excluded() {
        let ex = default_excludes();
        assert!(
            ex.contains(&"target".to_string()),
            "the ~370 MB intermediates tree must not ship: {ex:?}"
        );
    }

    /// The artifact the main sync excludes must be found and transferred
    /// separately — otherwise the robot gets source and no executable.
    #[test]
    fn secrets_and_fleet_inventory_are_excluded() {
        let ex = default_excludes();
        for pattern in [
            ".horus/deploy.yaml",
            ".env",
            "id_rsa",
            "id_ed25519",
            "*.pem",
            "*.key",
        ] {
            assert!(
                ex.contains(&pattern.to_string()),
                "`{pattern}` must be excluded by default: {ex:?}"
            );
        }
    }

    /// Excluding a top-level `target/` is still wanted — a developer who ran
    /// plain `cargo build` should not rsync their whole debug tree.
    #[test]
    fn ordinary_build_trees_are_still_excluded() {
        let ex = default_excludes();
        for pattern in [".git", "node_modules", "__pycache__", "*.pyc"] {
            assert!(
                ex.contains(&pattern.to_string()),
                "{pattern} missing: {ex:?}"
            );
        }
    }

    // ── The remote `cd` must be able to expand `~` ──────────────────────────

    /// `cd '~/horus_deploy'` fails: tilde expansion only happens for an
    /// unquoted tilde, so the shell looks for a literal directory named `~`.
    /// rsync's destination *is* expanded, so the files are there — and every
    /// `horus deploy --run` with default settings could not reach them.
    #[test]
    fn tilde_stays_unquoted_so_the_remote_shell_expands_it() {
        let q = shell_quote_preserving_tilde("~/horus_deploy");
        assert!(
            q.starts_with('~'),
            "a leading ~ must not be quoted, got {q}"
        );
        assert!(!q.starts_with("'~"), "got {q}");
        assert!(q.contains("horus_deploy"), "got {q}");
    }

    #[test]
    fn bare_tilde_is_passed_through() {
        assert_eq!(shell_quote_preserving_tilde("~"), "~");
    }

    /// Absolute and relative paths carry no tilde and must be fully quoted, so
    /// a directory containing spaces still works.
    #[test]
    fn non_tilde_paths_are_quoted() {
        assert_eq!(shell_quote_preserving_tilde("/opt/robot"), "'/opt/robot'");
        assert_eq!(
            shell_quote_preserving_tilde("/opt/my robot"),
            "'/opt/my robot'"
        );
    }

    /// Quoting must still defeat injection through a crafted remote_dir.
    #[test]
    fn single_quotes_in_the_path_cannot_break_out() {
        let q = shell_quote_preserving_tilde("/opt/rm -rf /");
        assert!(q.starts_with('\'') && q.ends_with('\''), "got {q}");

        let evil = shell_quote_preserving_tilde("~/a'; rm -rf /; echo '");
        // The tilde is bare, but everything after it stays inside quotes.
        assert!(evil.starts_with("~/'"), "got {evil}");
        assert!(
            evil.matches("'\\''").count() > 0,
            "embedded quotes must be escaped, got {evil}"
        );
    }
}
