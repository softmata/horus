//! Transparent proxy commands for native tools (cargo, pip, cmake).
//!
//! When invoked as `horus cargo <args>`, `horus pip <args>`, or `horus cmake <args>`,
//! these functions:
//! 1. Ensure `.horus/<native-file>` is up-to-date from `horus.toml`
//! 2. Fingerprint the file before running
//! 3. Execute the real native tool with appropriate path rewrites
//! 4. If the file changed, sync modifications back to `horus.toml`

use anyhow::{bail, Context, Result};
use colored::*;
use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::commands::env::SHADOWED_TOOLS;
use crate::fingerprint::Fingerprints;
use crate::manifest::HorusManifest;
use crate::native_sync::{self, NativeFileType, SyncResult};

const HORUS_TOML: &str = "horus.toml";

/// Find the horus project root by walking up from `start_dir` looking for `horus.toml`.
pub fn find_project_root(start_dir: &Path) -> Option<PathBuf> {
    let mut dir = start_dir.to_path_buf();
    for _ in 0..10 {
        if dir.join(HORUS_TOML).exists() {
            return Some(dir);
        }
        if !dir.pop() {
            break;
        }
    }
    None
}

/// Check whether the current directory is inside a horus project.
/// Used by `horus _is-project` (exit code only, no output).
pub fn run_is_project() -> bool {
    let cwd = env::current_dir().unwrap_or_default();
    find_project_root(&cwd).is_some()
}

// ── First-use notice ───────────────────────────────────────────────────────

/// Environment variable that silences [`announce_proxy`].
const NOTICE_OPT_OUT: &str = "HORUS_NO_PROXY_NOTICE";

/// Environment variable the generated shell functions set before delegating.
///
/// Its value is the name the *user* typed, which is not always the name of the
/// proxy that runs it: the `pip3` function calls `horus pip`, and a notice that
/// talks about `pip` to someone who typed `pip3` points them at the wrong
/// thing. See `commands::env::ENV_SH`.
const SHELL_PROXY_VAR: &str = "HORUS_SHELL_PROXY";

/// The line [`announce_proxy`] prints.
///
/// `typed` is what the user wrote, `subcommand` is the horus proxy it reached,
/// which differ for `pip3`. Split out from the printing so the wording is
/// testable: what matters is that it names the command that was shadowed, says
/// horus is in the path, and carries the way out.
fn proxy_notice_line(typed: &str, subcommand: &str) -> String {
    format!(
        "`{typed}` here runs through horus shell integration (horus {subcommand}). \
         Undo: horus env --uninstall  ·  silence: {NOTICE_OPT_OUT}=1"
    )
}

/// How this proxy came to be running.
#[derive(Debug, PartialEq, Eq)]
enum ShellProxy {
    /// A shell function written by `horus env --init` delegated to us; the
    /// payload is the name the user typed.
    Via(String),
    /// Shell integration is installed on this machine, but by a horus old
    /// enough that its functions do not set [`SHELL_PROXY_VAR`]. Worth
    /// announcing, under the proxy's own name — that is the best we know.
    Legacy,
    /// Nothing is shadowing anything here: `horus cargo` was typed out in full.
    No,
}

/// Work out which of those three cases we are in.
///
/// The first cut of the notice fired from `run_*_proxy` unconditionally, so
/// `horus cmake --version` typed by hand on a machine with no `~/.horus` at all
/// announced that "`cmake` here runs through horus shell integration" and
/// offered `horus env --uninstall` as the way out — a claim about the user's
/// shell that was false, and a remedy that was a no-op. A proxy cannot be
/// invoked by a shell function without that function saying so, so the marker
/// is the authority; the on-disk check exists only so installs written before
/// the marker existed keep disclosing until the next `horus env --init`.
fn classify_shell_proxy(marker: Option<&str>, integration: &[String]) -> ShellProxy {
    if let Some(typed) = marker.filter(|value| !value.is_empty()) {
        return if SHADOWED_TOOLS.contains(&typed) {
            ShellProxy::Via(typed.to_string())
        } else {
            // Set, but not to a name we shadow. Something in the environment is
            // claiming shell integration; believe the claim, not the name — the
            // value ends up in a file name.
            ShellProxy::Legacy
        };
    }
    if integration.is_empty() {
        return ShellProxy::No;
    }
    // Integration is installed. If what is installed knows about the marker and
    // the marker is not set, this invocation did not come through it.
    if integration
        .iter()
        .any(|content| content.contains(SHELL_PROXY_VAR))
    {
        ShellProxy::No
    } else {
        ShellProxy::Legacy
    }
}

/// The horus subcommand a shadowed name delegates to. Only `pip3` differs: its
/// shell function runs `horus pip`, because there is no `horus pip3`.
fn shadow_subcommand(typed: &str) -> &str {
    if typed == "pip3" {
        "pip"
    } else {
        typed
    }
}

/// Contents of whichever shell integration files `horus env --init` has written
/// under `home`. Empty means integration is not installed.
fn installed_integration(home: &Path) -> Vec<String> {
    let horus_dir = home.join(".horus");
    ["env.sh", "env.fish"]
        .iter()
        .filter_map(|name| std::fs::read_to_string(horus_dir.join(name)).ok())
        .collect()
}

/// Identify the shell session this proxy was invoked from.
///
/// The first cut of this used the parent pid, reasoning that the shell function
/// runs `command horus cargo "$@"` so our parent is the interactive shell. That
/// holds only for a bare foreground command. Every pipeline element, command
/// substitution, `( … )` and background job is a *forked copy* of the shell, so
/// `cmake --version | head -1` has a different parent every single time: the
/// notice repeated on every run of exactly the pipelines it was supposed to
/// stay out of.
///
/// The POSIX session id is the thing that actually means "this terminal
/// session". It is set once, by whatever called setsid() — the terminal
/// emulator, sshd, a tmux pane — and every descendant inherits it through all
/// of the forks above, while a new tab or a new ssh login gets a new one.
/// Returns `None` where there is no such notion, in which case no notice is
/// printed at all.
fn shell_session_key() -> Option<String> {
    #[cfg(unix)]
    {
        // SAFETY: getuid() cannot fail and getsid(0) asks about the calling
        // process, which necessarily exists. Neither takes a pointer.
        let uid = unsafe { libc::getuid() };
        let sid = unsafe { libc::getsid(0) };
        if sid < 0 {
            return None;
        }
        Some(session_key_from(uid, sid as u32))
    }
    #[cfg(not(unix))]
    {
        None
    }
}

/// Compose the marker key from the pieces, so a test can build the key for a
/// process other than itself.
fn session_key_from(uid: u32, sid: u32) -> String {
    match session_leader_start_time(sid) {
        Some(started) => format!("{uid}-{sid}-{started}"),
        None => format!("{uid}-{sid}"),
    }
}

/// Boot-relative start time of the session leader, on Linux.
///
/// Pids are recycled, so a session id on its own can be handed to a later,
/// unrelated session, which would then inherit a marker and silently skip its
/// notice. The leader's start time makes the pair unique for as long as the
/// machine is up. Where it cannot be read — not Linux, or the leader has
/// already exited — the key falls back to the session id alone, which is still
/// stable across every fork that matters.
fn session_leader_start_time(sid: u32) -> Option<u64> {
    start_time_from_stat(&std::fs::read_to_string(format!("/proc/{sid}/stat")).ok()?)
}

/// Pull field 22 out of a `/proc/<pid>/stat` line.
///
/// Field 2 is the executable name in parentheses and may itself contain spaces
/// and `)` — a shell named `(ba d)sh` is legal — so everything after it is
/// located from the LAST `)` rather than by splitting the line from the front.
/// starttime is field 22, i.e. the twentieth token after the name.
fn start_time_from_stat(stat: &str) -> Option<u64> {
    let after_comm = stat.get(stat.rfind(')')? + 1..)?;
    after_comm.split_whitespace().nth(19)?.parse().ok()
}

/// Whether this process has a controlling terminal.
///
/// The first cut asked whether *stderr* is a tty, which inverted the notice on
/// the case the whole feature is for: `cargo build 2>&1 | tee build.log` is a
/// person at a terminal keeping the log they will read six months later, and it
/// was the one shape that got no disclosure at all. What separates that from CI
/// or cron is not where stderr points, it is whether there is a session with a
/// terminal behind it — so ask that instead, and let the line land in the pipe.
/// It can therefore reach a `2>&1` capture, once per session; the shell
/// functions only exist in shells that read an rc file, so what is captured
/// there is a command the user typed, not one a configure script spawned.
fn has_controlling_terminal() -> bool {
    #[cfg(unix)]
    {
        // SAFETY: open() with a static NUL-terminated path and no output
        // parameters; the descriptor is closed immediately. O_NONBLOCK because
        // opening a serial console can otherwise wait on carrier detect, and a
        // notice must never be the reason a build hangs.
        unsafe {
            let fd = libc::open(
                c"/dev/tty".as_ptr(),
                libc::O_RDONLY | libc::O_NONBLOCK | libc::O_CLOEXEC,
            );
            if fd < 0 {
                return false;
            }
            libc::close(fd);
            true
        }
    }
    #[cfg(not(unix))]
    {
        false
    }
}

/// Where the once-per-session markers live.
///
/// `$XDG_RUNTIME_DIR` is wiped when the user's last session ends, which is
/// exactly the lifetime wanted here. Where there is none, the temp dir is at
/// least cleared on reboot.
fn notice_marker_dir() -> PathBuf {
    dirs::runtime_dir()
        .unwrap_or_else(env::temp_dir)
        .join("horus")
}

/// Create `marker`, reporting whether this call was the one that created it.
///
/// `create_new` is the atomic test-and-set — two proxies racing inside one
/// session still produce exactly one notice. Any I/O failure counts as "already
/// claimed": a notice is a nicety and must never be the reason a build fails.
fn claim_session_notice(marker: &Path) -> bool {
    if let Some(parent) = marker.parent() {
        if std::fs::create_dir_all(parent).is_err() {
            return false;
        }
    }
    std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(marker)
        .is_ok()
}

/// Everything [`proxy_notice`] needs to know about the world outside it, so the
/// decision can be exercised without a terminal, a home directory or a mutated
/// process environment.
struct NoticeContext {
    opted_out: bool,
    shell_proxy: ShellProxy,
    /// Whether there is a human session behind this process at all — *not*
    /// whether stderr happens to be a tty.
    interactive_session: bool,
    session: Option<String>,
    marker_dir: PathBuf,
}

impl NoticeContext {
    fn from_env() -> Self {
        let opted_out = env::var_os(NOTICE_OPT_OUT).is_some_and(|value| !value.is_empty());
        let marker = env::var(SHELL_PROXY_VAR).ok();
        // Only consult the disk when the marker is absent and someone is going
        // to read the answer; otherwise this is two wasted reads per build.
        let integration = if opted_out || marker.as_deref().is_some_and(|v| !v.is_empty()) {
            Vec::new()
        } else {
            dirs::home_dir()
                .map(|home| installed_integration(&home))
                .unwrap_or_default()
        };
        Self {
            opted_out,
            shell_proxy: classify_shell_proxy(marker.as_deref(), &integration),
            interactive_session: has_controlling_terminal(),
            session: shell_session_key(),
            marker_dir: notice_marker_dir(),
        }
    }
}

/// Decide whether to disclose the indirection, and claim the session if so.
///
/// Returns the line to print, or `None` to stay quiet. Once per shell session
/// per shadowed name rather than per invocation, because a build loop would
/// otherwise repeat it every few seconds. Each proxy run is a separate process,
/// so the state cannot be a `std::sync::Once` — it is a marker file.
fn proxy_notice(ctx: &NoticeContext, subcommand: &str) -> Option<String> {
    if ctx.opted_out {
        return None;
    }
    let typed = match &ctx.shell_proxy {
        ShellProxy::Via(name) if shadow_subcommand(name) == subcommand => name.as_str(),
        // The marker is inherited by everything a proxied build spawns, so a
        // `horus cmake` run out of a build script under `horus cargo build`
        // still sees HORUS_SHELL_PROXY=cargo. That invocation did not come
        // through the cmake function, and "`cargo` here runs through … (horus
        // cmake)" would be nonsense.
        ShellProxy::Via(_) => return None,
        ShellProxy::Legacy => subcommand,
        ShellProxy::No => return None,
    };
    if !ctx.interactive_session {
        return None;
    }
    let session = ctx.session.as_deref()?;
    let marker = ctx
        .marker_dir
        .join(format!("proxy-notice-{session}-{typed}"));
    if !claim_session_notice(&marker) {
        return None;
    }
    Some(proxy_notice_line(typed, subcommand))
}

/// Tell the user, at most once per shell session, that this tool is proxied.
///
/// `horus env --init` shadows `cargo`, `pip`, `pip3`, `cmake`, `conan` and
/// `vcpkg` with shell functions, so months later a plain `cargo build` inside a
/// horus project is quietly a `horus cargo build`. When that behaves in a way
/// plain cargo would not, nothing in the output mentions horus and there is no
/// thread to pull. Disclosing it at install time does not help someone who has
/// forgotten; the indirection has to announce itself where it applies.
pub fn announce_proxy(subcommand: &str) {
    if let Some(line) = proxy_notice(&NoticeContext::from_env(), subcommand) {
        crate::cli_output::info(&line);
    }
}

// ── Proxy entry ────────────────────────────────────────────────────────────

/// Where a proxy stands relative to the project it was invoked in.
enum ProxyEntry {
    /// Inside a horus project: arguments get rewritten and native files synced,
    /// and the first time that happens in a shell session the user is told.
    Project(PathBuf),
    /// Outside one: the real tool runs untouched, so there is nothing to
    /// disclose and nothing to sync.
    PassThrough,
}

/// Decide where a proxy stands, announcing the indirection on the way in.
///
/// All five proxies come through here instead of pairing `find_project_root`
/// with their own `announce_proxy` call, because bolting the notice onto each
/// call site separately meant nothing broke when a site lacked it and no test
/// referenced any of them. Here the project root cannot be obtained without the
/// announcement having happened.
fn enter_proxy(subcommand: &str, cwd: &Path) -> ProxyEntry {
    enter_proxy_with(subcommand, cwd, &mut |tool| announce_proxy(tool))
}

/// [`enter_proxy`] with the announcement injected, so a test can observe it.
fn enter_proxy_with(subcommand: &str, cwd: &Path, announce: &mut dyn FnMut(&str)) -> ProxyEntry {
    match find_project_root(cwd) {
        Some(project_dir) => {
            announce(subcommand);
            ProxyEntry::Project(project_dir)
        }
        None => ProxyEntry::PassThrough,
    }
}

/// Run the real tool with its arguments untouched.
fn run_real_tool(tool: &str, args: &[String]) -> Result<i32> {
    let real = find_real_tool(tool)?;
    let status = Command::new(&real).args(args).status()?;
    Ok(status.code().unwrap_or(1))
}

/// Cargo subcommands that should NOT get `--manifest-path` (global commands).
/// Everything else gets it — this handles third-party subcommands (cargo-audit,
/// cargo-watch, cargo-expand, etc.) automatically.
const CARGO_NO_MANIFEST_PATH: &[&str] = &[
    "install",
    "uninstall",
    "login",
    "logout",
    "search",
    "help",
    "version",
    "new",
    "init",
    "+stable",
    "+nightly",
    "+beta",
];

/// Find the real native tool binary, skipping any horus shim.
fn find_real_tool(tool_name: &str) -> Result<PathBuf> {
    // Check env var override first (set by horus env.sh)
    let env_key = format!("HORUS_REAL_{}", tool_name.to_uppercase());
    if let Ok(path) = env::var(&env_key) {
        let p = PathBuf::from(&path);
        if p.exists() {
            return Ok(p);
        }
    }

    // Search PATH for the real binary (skip entries that are horus itself)
    let path_var = env::var("PATH").unwrap_or_default();
    for dir in path_var.split(':') {
        let candidate = PathBuf::from(dir).join(tool_name);
        if candidate.exists() {
            // Make sure it's not horus itself
            if let Ok(resolved) = std::fs::canonicalize(&candidate) {
                let resolved_name = resolved.file_name().and_then(|n| n.to_str()).unwrap_or("");
                if resolved_name == "horus" {
                    continue;
                }
            }
            return Ok(candidate);
        }
    }

    bail!("Could not find `{}` in PATH. Is it installed?", tool_name)
}

/// Run the sync-before/after cycle: generate, fingerprint, exec, sync-back.
fn run_with_sync(
    project_dir: &Path,
    file_type: NativeFileType,
    tool_name: &str,
    mut cmd: Command,
) -> Result<i32> {
    let manifest_path = project_dir.join(HORUS_TOML);
    let manifest = HorusManifest::load_from(&manifest_path).context("Failed to load horus.toml")?;

    // 1. Generate native file from horus.toml
    let mut fingerprints = Fingerprints::load(project_dir).unwrap_or_default();
    let content = generate_native(&manifest, project_dir, file_type)?;
    fingerprints.record(file_type.filename(), &content);

    // For workspace projects, also fingerprint each member's generated Cargo.toml
    if file_type.filename() == "Cargo.toml" && manifest.is_workspace() {
        if let Some(ref ws) = manifest.workspace {
            if let Ok(members) = crate::manifest::resolve_workspace_members(ws, project_dir) {
                for (_, member_manifest) in &members {
                    let name = crate::cargo_gen::sanitize_cargo_name(&member_manifest.package.name);
                    let member_path = project_dir.join(".horus").join(&name).join("Cargo.toml");
                    if let Ok(member_content) = std::fs::read_to_string(&member_path) {
                        fingerprints.record(&format!("{}/Cargo.toml", name), &member_content);
                    }
                }
            }
        }
    }

    let _ = fingerprints.save(project_dir);

    // 2. Execute the real tool (may take a long time — no lock held here)
    let status = cmd
        .status()
        .with_context(|| format!("Failed to execute `{}`", tool_name))?;

    // 3. Check if the native file was modified by the tool
    if fingerprints.is_modified(file_type.filename(), project_dir) {
        // Acquire lock for the sync-back cycle to prevent race conditions
        let _lock = crate::fingerprint::SyncLock::try_acquire(
            project_dir,
            std::time::Duration::from_secs(5),
        );

        // Re-load manifest inside lock (another terminal may have modified it)
        let mut manifest = HorusManifest::load_from(&manifest_path)
            .context("Failed to reload horus.toml for sync")?;

        match native_sync::sync_from_native(
            project_dir,
            &mut manifest,
            file_type,
            &mut fingerprints,
        )? {
            SyncResult::Synced {
                added,
                removed,
                modified,
            } => {
                manifest.save_to(&manifest_path)?;
                if added > 0 {
                    eprintln!("  {} Synced {} new dep(s) to horus.toml", "⟳".cyan(), added);
                }
                if removed > 0 {
                    eprintln!(
                        "  {} Removed {} dep(s) from horus.toml",
                        "⟳".cyan(),
                        removed
                    );
                }
                if modified > 0 {
                    eprintln!("  {} Updated {} dep(s) in horus.toml", "⟳".cyan(), modified);
                }
                // Re-generate to normalize the file
                let content = generate_native(&manifest, project_dir, file_type)?;
                fingerprints.record(file_type.filename(), &content);
                let _ = fingerprints.save(project_dir);
            }
            SyncResult::NoChanges => {}
        }
        // _lock dropped here, releasing flock
    }

    Ok(status.code().unwrap_or(1))
}

/// Generate a native file and return its content.
fn generate_native(
    manifest: &HorusManifest,
    project_dir: &Path,
    file_type: NativeFileType,
) -> Result<String> {
    match file_type {
        NativeFileType::Cargo => {
            let (_, content) =
                crate::cargo_gen::generate_for_manifest(manifest, project_dir, &[], false)?;
            Ok(content)
        }
        NativeFileType::Pyproject => {
            let (_, content) = crate::pyproject_gen::generate(manifest, project_dir, false)?;
            Ok(content)
        }
        NativeFileType::Cmake => {
            let (_, content) = crate::cmake_gen::generate(manifest, project_dir, false)?;
            Ok(content)
        }
    }
}

/// Check if a cargo subcommand should be excluded from `--manifest-path` injection.
fn is_cargo_global_command(subcmd: &str) -> bool {
    // Toolchain overrides like +nightly
    if subcmd.starts_with('+') {
        return true;
    }
    // Bare flags like --version, --help, -V, -h
    if subcmd.starts_with('-') {
        return true;
    }
    CARGO_NO_MANIFEST_PATH.contains(&subcmd)
}

// ── Cargo proxy ────────────────────────────────────────────────────────────

/// Transparent cargo proxy: `horus cargo <args>`.
pub fn run_cargo_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    // Outside a horus project nothing is rewritten — pass real cargo through.
    let ProxyEntry::Project(project_dir) = enter_proxy("cargo", &cwd) else {
        return run_real_tool("cargo", &args);
    };

    let real_cargo = find_real_tool("cargo")?;
    let manifest_arg = project_dir
        .join(".horus/Cargo.toml")
        .to_string_lossy()
        .to_string();
    let target_dir_arg = project_dir
        .join(".horus/target")
        .to_string_lossy()
        .to_string();

    // Build the command with --manifest-path and CARGO_TARGET_DIR injected
    let mut cmd = Command::new(&real_cargo);

    // Always set CARGO_TARGET_DIR so artifacts go to .horus/target/
    cmd.env("CARGO_TARGET_DIR", &target_dir_arg);

    let subcommand = args.first().map(|s| s.as_str()).unwrap_or("");
    let already_has_manifest = args.iter().any(|a| a.starts_with("--manifest-path"));

    if !is_cargo_global_command(subcommand) && !already_has_manifest {
        // Inject --manifest-path right after the subcommand
        if let Some(subcmd) = args.first() {
            cmd.arg(subcmd);
            cmd.arg("--manifest-path").arg(&manifest_arg);
            cmd.args(&args[1..]);
        }
    } else {
        cmd.args(&args);
    }

    let result = run_with_sync(&project_dir, NativeFileType::Cargo, "cargo", cmd)?;

    // For workspace projects, also sync per-member Cargo.toml changes
    let manifest_path = project_dir.join(HORUS_TOML);
    if let Ok(manifest) = HorusManifest::load_from(&manifest_path) {
        if manifest.is_workspace() {
            let mut fingerprints = Fingerprints::load(&project_dir).unwrap_or_default();
            match native_sync::sync_workspace_members(&project_dir, &manifest, &mut fingerprints) {
                Ok(SyncResult::Synced {
                    added,
                    removed,
                    modified,
                }) => {
                    if added > 0 {
                        eprintln!("  {} Synced {} new member dep(s)", "⟳".cyan(), added);
                    }
                    if removed > 0 {
                        eprintln!("  {} Removed {} member dep(s)", "⟳".cyan(), removed);
                    }
                    if modified > 0 {
                        eprintln!("  {} Updated {} member dep(s)", "⟳".cyan(), modified);
                    }
                    let _ = fingerprints.save(&project_dir);
                }
                Ok(SyncResult::NoChanges) => {}
                Err(e) => log::warn!("Member sync failed: {}", e),
            }
        }
    }

    Ok(result)
}

// ── Pip proxy ──────────────────────────────────────────────────────────────

/// Transparent pip proxy: `horus pip <args>`.
pub fn run_pip_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let ProxyEntry::Project(project_dir) = enter_proxy("pip", &cwd) else {
        return run_real_tool("pip", &args);
    };

    let real_pip = find_real_tool("pip")?;
    let subcmd = args.first().map(|s| s.as_str()).unwrap_or("");

    // --- pip install ---
    if subcmd == "install" {
        return run_pip_install(&project_dir, &real_pip, &args);
    }

    // --- pip uninstall ---
    if subcmd == "uninstall" {
        return run_pip_uninstall(&project_dir, &real_pip, &args);
    }

    // --- everything else: pass through ---
    let mut cmd = Command::new(&real_pip);
    cmd.args(&args);
    let status = cmd.status()?;
    Ok(status.code().unwrap_or(1))
}

/// Handle `pip install` — add installed packages to horus.toml.
fn run_pip_install(project_dir: &Path, real_pip: &Path, args: &[String]) -> Result<i32> {
    let mut cmd = Command::new(real_pip);

    // Rewrite `pip install -e .` → `pip install -e .horus/`
    let mut rewritten_args: Vec<String> = args.to_vec();
    let is_editable = args.iter().any(|a| a == "-e" || a == "--editable");

    for i in 0..rewritten_args.len() {
        if (rewritten_args[i] == "-e" || rewritten_args[i] == "--editable")
            && rewritten_args.get(i + 1).map(|s| s.as_str()) == Some(".")
        {
            rewritten_args[i + 1] = project_dir.join(".horus").to_string_lossy().to_string();
        }
    }

    cmd.args(&rewritten_args);
    let status = cmd.status()?;

    // Sync installed packages to horus.toml (skip for editable installs of self)
    if status.success() && !is_editable {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut synced = 0;

            // Collect all package specs from args + requirements files
            let specs = collect_pip_specs(&args[1..]);
            let is_upgrade = args.iter().any(|a| a == "--upgrade" || a == "-U");

            for spec in &specs {
                let dep = parse_pip_to_dep(spec);
                if let Some((name, dep_value)) = dep {
                    match manifest.dependencies.entry(name) {
                        std::collections::btree_map::Entry::Vacant(e) => {
                            e.insert(dep_value);
                            synced += 1;
                        }
                        std::collections::btree_map::Entry::Occupied(mut e) => {
                            if is_upgrade {
                                // Update existing dep on --upgrade
                                e.insert(dep_value);
                                synced += 1;
                            }
                        }
                    }
                }
            }
            if synced > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Synced {} new dep(s) to horus.toml",
                    "⟳".cyan(),
                    synced
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

/// Handle `pip uninstall` — remove uninstalled packages from horus.toml.
fn run_pip_uninstall(project_dir: &Path, real_pip: &Path, args: &[String]) -> Result<i32> {
    // Collect package names before running uninstall
    let packages: Vec<String> = args[1..]
        .iter()
        .filter(|a| !a.starts_with('-'))
        .cloned()
        .collect();

    let mut cmd = Command::new(real_pip);
    cmd.args(args);
    let status = cmd.status()?;

    if status.success() && !packages.is_empty() {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut removed = 0;
            for pkg in &packages {
                let name = pkg.to_lowercase().replace('-', "_");
                // Try both forms: hyphenated and underscored
                let removed_key = if manifest.dependencies.remove(pkg).is_some() {
                    Some(pkg.clone())
                } else if manifest.dependencies.remove(&name).is_some() {
                    Some(name)
                } else {
                    // Try the other normalization
                    let alt = pkg.replace('_', "-");
                    if manifest.dependencies.remove(&alt).is_some() {
                        Some(alt)
                    } else {
                        None
                    }
                };
                if removed_key.is_some() {
                    removed += 1;
                }
            }
            if removed > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Removed {} dep(s) from horus.toml",
                    "⟳".cyan(),
                    removed
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

/// Collect all pip package specs from command args, expanding -r requirements files.
fn collect_pip_specs(args: &[String]) -> Vec<String> {
    let mut specs = Vec::new();
    let mut skip_next = false;
    let mut i = 0;
    while i < args.len() {
        let arg = &args[i];
        if skip_next {
            skip_next = false;
            i += 1;
            continue;
        }
        if arg.starts_with('-') {
            if arg == "-r" || arg == "--requirement" {
                // Read the requirements file and add each line as a spec
                if let Some(req_file) = args.get(i + 1) {
                    if let Ok(content) = std::fs::read_to_string(req_file) {
                        for line in content.lines() {
                            let line = line.trim();
                            if line.is_empty() || line.starts_with('#') || line.starts_with('-') {
                                continue;
                            }
                            specs.push(line.to_string());
                        }
                    }
                }
                skip_next = true;
            } else if matches!(
                arg.as_str(),
                "-c" | "--constraint"
                    | "-e"
                    | "--editable"
                    | "-t"
                    | "--target"
                    | "-i"
                    | "--index-url"
                    | "--extra-index-url"
                    | "--prefix"
                    | "--root"
                    | "--src"
            ) {
                skip_next = true;
            }
            i += 1;
            continue;
        }
        specs.push(arg.clone());
        i += 1;
    }
    specs
}

/// Parse a pip spec string into a (name, DependencyValue) pair for horus.toml.
/// Handles regular packages, git+https:// URLs, and ./local-path installs.
fn parse_pip_to_dep(spec: &str) -> Option<(String, crate::manifest::DependencyValue)> {
    let spec = spec.trim();

    // Git URL: git+https://github.com/org/repo
    if spec.starts_with("git+") || spec.starts_with("hg+") || spec.starts_with("svn+") {
        let url = spec
            .strip_prefix("git+")
            .or_else(|| spec.strip_prefix("hg+"))
            .or_else(|| spec.strip_prefix("svn+"))
            .unwrap_or(spec);
        // Extract name from URL (last path segment, strip .git)
        let name = url
            .rsplit('/')
            .next()
            .unwrap_or("unknown")
            .split('@') // strip @branch/tag
            .next()
            .unwrap_or("unknown")
            .trim_end_matches(".git")
            .to_string();
        if name.is_empty() || name == "unknown" {
            return None;
        }
        return Some((
            name,
            crate::manifest::DependencyValue::Detailed(crate::manifest::DetailedDependency {
                source: Some(crate::manifest::DepSource::Git),
                git: Some(url.to_string()),
                ..crate::manifest::DetailedDependency::default()
            }),
        ));
    }

    // Local path: ./something or ../something
    if spec.starts_with("./") || spec.starts_with("../") || spec.starts_with('/') {
        let path = std::path::Path::new(spec);
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("local-dep")
            .to_string();
        return Some((
            name,
            crate::manifest::DependencyValue::Detailed(crate::manifest::DetailedDependency {
                source: Some(crate::manifest::DepSource::Path),
                path: Some(spec.to_string()),
                ..crate::manifest::DetailedDependency::default()
            }),
        ));
    }

    // Regular package: numpy>=1.24 or uvicorn[standard]>=0.20
    let (name, version, extras) = parse_pip_spec(spec);
    if name.is_empty() {
        return None;
    }

    Some((
        name,
        crate::manifest::DependencyValue::Detailed(crate::manifest::DetailedDependency {
            version,
            source: Some(crate::manifest::DepSource::PyPI),
            features: extras,
            ..crate::manifest::DetailedDependency::default()
        }),
    ))
}

/// Parse a pip package specifier like "numpy>=1.24" or "uvicorn[standard]>=0.20"
/// into (name, version, extras).
fn parse_pip_spec(spec: &str) -> (String, Option<String>, Vec<String>) {
    let spec = spec.trim();
    let split_at = spec
        .find(['>', '<', '=', '!', '~', '['])
        .unwrap_or(spec.len());
    let name = spec[..split_at].trim().to_string();

    let mut extras = Vec::new();
    let mut version = None;

    if split_at < spec.len() {
        let rest = &spec[split_at..];
        // Extract extras from [extra1,extra2]
        if rest.starts_with('[') {
            if let Some(bracket_end) = rest.find(']') {
                let extras_str = &rest[1..bracket_end];
                extras = extras_str
                    .split(',')
                    .map(|s| s.trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect();
                let after_bracket = rest[bracket_end + 1..].trim();
                if !after_bracket.is_empty() {
                    version = Some(after_bracket.to_string());
                }
            }
        } else {
            version = Some(rest.trim().to_string());
        }
    }

    (name, version, extras)
}

// ── CMake proxy ────────────────────────────────────────────────────────────

/// Transparent cmake proxy: `horus cmake <args>`.
/// Whether these arguments select one of cmake's non-configure modes.
///
/// Mirrors `is_cargo_global_command`: the proxy's rewriting only makes sense
/// for a configure run, and applying it to `cmake -E`, `cmake -P` or
/// `cmake --build` appends a `-B <dir>` those modes do not accept.
fn is_cmake_mode_invocation(args: &[String]) -> bool {
    const MODES: &[&str] = &[
        "-E",
        "-P",
        "--build",
        "--install",
        "--open",
        "--find-package",
        "--system-information",
    ];
    args.first().is_some_and(|a| MODES.contains(&a.as_str()))
}

pub fn run_cmake_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let ProxyEntry::Project(project_dir) = enter_proxy("cmake", &cwd) else {
        return run_real_tool("cmake", &args);
    };

    let real_cmake = find_real_tool("cmake")?;
    let mut cmd = Command::new(&real_cmake);

    // Only a *configure* invocation takes a source and build directory. cmake's
    // other modes take neither, and rewriting their arguments corrupts them:
    //
    //   $ horus cmake -E echo hi
    //   hi -B /path/.horus/cpp-build
    //
    // `-E` runs a command, `-P` runs a script, `--build`/`--install`/`--open`
    // act on an existing build tree. Pass all of them through untouched.
    if is_cmake_mode_invocation(&args) {
        cmd.args(&args);
        return run_with_sync(&project_dir, NativeFileType::Cmake, "cmake", cmd);
    }

    // Rewrite source dir `.` → `.horus/` and ensure build dir
    let mut rewritten_args = args.clone();
    let mut has_build_dir = false;
    for i in 0..rewritten_args.len() {
        if rewritten_args[i] == "-B" {
            has_build_dir = true;
        }
        if rewritten_args[i] == "-S" {
            if let Some(next) = rewritten_args.get(i + 1) {
                if next == "." {
                    rewritten_args[i + 1] =
                        project_dir.join(".horus").to_string_lossy().to_string();
                }
            }
        }
        // Bare "." as source directory
        if rewritten_args[i] == "." && (i == 0 || !rewritten_args[i - 1].starts_with('-')) {
            rewritten_args[i] = project_dir.join(".horus").to_string_lossy().to_string();
        }
    }

    if !has_build_dir {
        rewritten_args.push("-B".to_string());
        rewritten_args.push(
            project_dir
                .join(".horus/cpp-build")
                .to_string_lossy()
                .to_string(),
        );
    }

    cmd.args(&rewritten_args);

    run_with_sync(&project_dir, NativeFileType::Cmake, "cmake", cmd)
}

// ── Conan proxy ────────────────────────────────────────────────────────────

/// Transparent conan proxy: `horus conan <args>`.
pub fn run_conan_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let ProxyEntry::Project(project_dir) = enter_proxy("conan", &cwd) else {
        return run_real_tool("conan", &args);
    };

    let real_conan = find_real_tool("conan")?;
    let mut cmd = Command::new(&real_conan);
    cmd.args(&args);
    let status = cmd.status()?;

    // Sync on `conan install <reference>` success
    let subcmd = args.first().map(|s| s.as_str()).unwrap_or("");
    if status.success() && subcmd == "install" {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut synced = 0;
            for arg in &args[1..] {
                if arg.starts_with('-') {
                    continue;
                }
                // Conan reference: name/version@user/channel or name/version@
                if let Some((name, version)) = parse_conan_ref(arg) {
                    if let std::collections::btree_map::Entry::Vacant(e) =
                        manifest.dependencies.entry(name)
                    {
                        e.insert(crate::manifest::DependencyValue::Detailed(
                            crate::manifest::DetailedDependency {
                                version: Some(version),
                                source: Some(crate::manifest::DepSource::System),
                                lang: Some("cpp".to_string()),
                                ..crate::manifest::DetailedDependency::default()
                            },
                        ));
                        synced += 1;
                    }
                }
            }
            if synced > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Synced {} conan dep(s) to horus.toml",
                    "⟳".cyan(),
                    synced
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

/// Parse a conan reference like "fmt/10.0.0@" or "boost/1.82.0@user/stable".
fn parse_conan_ref(reference: &str) -> Option<(String, String)> {
    let parts: Vec<&str> = reference.split('/').collect();
    if parts.len() >= 2 {
        let name = parts[0].to_string();
        let version = parts[1].split('@').next().unwrap_or("*").to_string();
        if !name.is_empty() && !version.is_empty() {
            return Some((name, version));
        }
    }
    None
}

// ── vcpkg proxy ────────────────────────────────────────────────────────────

/// Transparent vcpkg proxy: `horus vcpkg <args>`.
pub fn run_vcpkg_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let ProxyEntry::Project(project_dir) = enter_proxy("vcpkg", &cwd) else {
        return run_real_tool("vcpkg", &args);
    };

    let real_vcpkg = find_real_tool("vcpkg")?;
    let mut cmd = Command::new(&real_vcpkg);
    cmd.args(&args);
    let status = cmd.status()?;

    // Sync on `vcpkg install <package>` success
    let subcmd = args.first().map(|s| s.as_str()).unwrap_or("");
    if status.success() && subcmd == "install" {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut synced = 0;
            for arg in &args[1..] {
                if arg.starts_with('-') {
                    continue;
                }
                // vcpkg package: "fmt" or "fmt:x64-linux" (strip triplet)
                let name = arg.split(':').next().unwrap_or(arg).to_string();
                if !name.is_empty() && !manifest.dependencies.contains_key(&name) {
                    manifest.dependencies.insert(
                        name,
                        crate::manifest::DependencyValue::Detailed(
                            crate::manifest::DetailedDependency {
                                source: Some(crate::manifest::DepSource::System),
                                lang: Some("cpp".to_string()),
                                ..crate::manifest::DetailedDependency::default()
                            },
                        ),
                    );
                    synced += 1;
                }
            }
            if synced > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Synced {} vcpkg dep(s) to horus.toml",
                    "⟳".cyan(),
                    synced
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn find_project_root_no_project() {
        let tmp = tempfile::tempdir().unwrap();
        assert!(find_project_root(tmp.path()).is_none());
    }

    #[test]
    fn find_project_root_found() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"",
        )
        .unwrap();
        assert_eq!(
            find_project_root(tmp.path()),
            Some(tmp.path().to_path_buf())
        );
    }

    #[test]
    fn parse_pip_spec_simple() {
        let (name, ver, extras) = parse_pip_spec("numpy");
        assert_eq!(name, "numpy");
        assert!(ver.is_none());
        assert!(extras.is_empty());
    }

    #[test]
    fn parse_pip_spec_versioned() {
        let (name, ver, extras) = parse_pip_spec("numpy>=1.24");
        assert_eq!(name, "numpy");
        assert_eq!(ver, Some(">=1.24".to_string()));
        assert!(extras.is_empty());
    }

    #[test]
    fn parse_pip_spec_extras() {
        let (name, ver, extras) = parse_pip_spec("uvicorn[standard]>=0.20");
        assert_eq!(name, "uvicorn");
        assert_eq!(ver, Some(">=0.20".to_string()));
        assert_eq!(extras, vec!["standard"]);
    }

    #[test]
    fn parse_pip_spec_multiple_extras() {
        let (name, ver, extras) = parse_pip_spec("fastapi[all,jinja]>=0.100");
        assert_eq!(name, "fastapi");
        assert_eq!(ver, Some(">=0.100".to_string()));
        assert_eq!(extras, vec!["all", "jinja"]);
    }

    #[test]
    fn parse_pip_spec_extras_no_version() {
        let (name, ver, extras) = parse_pip_spec("uvicorn[standard]");
        assert_eq!(name, "uvicorn");
        assert!(ver.is_none());
        assert_eq!(extras, vec!["standard"]);
    }

    #[test]
    fn parse_pip_to_dep_git_url() {
        let result = parse_pip_to_dep("git+https://github.com/org/my-lib");
        let (name, dep) = result.unwrap();
        assert_eq!(name, "my-lib");
        assert_eq!(dep.effective_source(), crate::manifest::DepSource::Git);
    }

    #[test]
    fn parse_pip_to_dep_local_path() {
        let result = parse_pip_to_dep("./libs/my-pkg");
        let (name, dep) = result.unwrap();
        assert_eq!(name, "my-pkg");
        assert_eq!(dep.effective_source(), crate::manifest::DepSource::Path);
    }

    #[test]
    fn collect_pip_specs_with_requirements() {
        let tmp = tempfile::tempdir().unwrap();
        let req_path = tmp.path().join("requirements.txt");
        std::fs::write(&req_path, "numpy>=1.24\n# comment\nrequests\n-e .\n").unwrap();
        let args = vec![
            "-r".to_string(),
            req_path.to_string_lossy().to_string(),
            "torch".to_string(),
        ];
        let specs = collect_pip_specs(&args);
        assert_eq!(specs, vec!["numpy>=1.24", "requests", "torch"]);
    }

    #[test]
    fn cargo_global_commands_detected() {
        assert!(is_cargo_global_command("install"));
        assert!(is_cargo_global_command("new"));
        assert!(is_cargo_global_command("+nightly"));
        assert!(is_cargo_global_command("--version"));
        assert!(is_cargo_global_command("-V"));
        assert!(!is_cargo_global_command("build"));
        assert!(!is_cargo_global_command("test"));
        assert!(!is_cargo_global_command("audit")); // third-party, gets manifest-path
        assert!(!is_cargo_global_command("watch")); // third-party
        assert!(!is_cargo_global_command("expand")); // third-party
    }

    #[test]
    fn conan_ref_parsing() {
        let (name, ver) = parse_conan_ref("fmt/10.0.0@").unwrap();
        assert_eq!(name, "fmt");
        assert_eq!(ver, "10.0.0");
    }

    #[test]
    fn conan_ref_with_user_channel() {
        let (name, ver) = parse_conan_ref("boost/1.82.0@user/stable").unwrap();
        assert_eq!(name, "boost");
        assert_eq!(ver, "1.82.0");
    }

    // ── First-use notice ───────────────────────────────────────────────────

    /// A context that would announce, pointed at a scratch marker directory.
    /// Tests build one and change the single field under test, so none of them
    /// depends on the process environment, a terminal or `$HOME` — the three
    /// things that made the first version of this feature untestable.
    fn announcing_ctx(marker_dir: &Path, typed: &str) -> NoticeContext {
        NoticeContext {
            opted_out: false,
            shell_proxy: ShellProxy::Via(typed.to_string()),
            interactive_session: true,
            session: Some("1000-4242".to_string()),
            marker_dir: marker_dir.to_path_buf(),
        }
    }

    /// Nothing was claimed: a suppressed notice must not burn the session's one
    /// chance to disclose, in case the next invocation is one that would.
    fn marker_dir_is_empty(dir: &Path) -> bool {
        std::fs::read_dir(dir)
            .map(|mut entries| entries.next().is_none())
            .unwrap_or(true)
    }

    /// The notice exists so a `cargo` that is not cargo has something in its
    /// output pointing back at horus. Losing the undo command loses the point.
    #[test]
    fn proxy_notice_names_the_tool_and_the_way_out() {
        let line = proxy_notice_line("cargo", "cargo");
        assert!(line.contains("cargo"));
        assert!(line.contains("horus env --uninstall"));
        assert!(line.contains(NOTICE_OPT_OUT));
    }

    /// The `pip3` shell function calls `horus pip`, so the proxy that runs is
    /// the pip one. Telling someone who typed `pip3` that `pip` is shadowed
    /// sends them looking for a function they never called.
    #[test]
    fn notice_names_the_command_that_was_typed_not_the_subcommand() {
        let tmp = tempfile::tempdir().unwrap();
        let line = proxy_notice(&announcing_ctx(tmp.path(), "pip3"), "pip").unwrap();
        assert!(line.starts_with("`pip3`"), "{line}");
        assert!(line.contains("horus pip"), "{line}");
    }

    /// The marker is exported into everything a proxied build spawns, so a
    /// `horus cmake` run out of a build script under `horus cargo build` sees
    /// `HORUS_SHELL_PROXY=cargo`. No cmake function was involved there, and the
    /// notice must not invent one.
    #[test]
    fn an_inherited_marker_does_not_announce_a_different_tool() {
        let tmp = tempfile::tempdir().unwrap();
        let ctx = announcing_ctx(tmp.path(), "cargo");
        assert!(proxy_notice(&ctx, "cmake").is_none());
        // ...while pip3 -> `horus pip` is the one legitimate mismatch.
        assert!(proxy_notice(&announcing_ctx(tmp.path(), "pip3"), "pip").is_some());
    }

    /// Once per shell session per shadowed name — a build loop must not repeat
    /// it, and a second terminal must not miss it.
    #[test]
    fn notice_is_claimed_once_per_session_and_name() {
        let tmp = tempfile::tempdir().unwrap();
        let ctx = announcing_ctx(tmp.path(), "cargo");

        assert!(proxy_notice(&ctx, "cargo").is_some(), "first use announces");
        assert!(
            proxy_notice(&ctx, "cargo").is_none(),
            "a second build in the same session stays silent"
        );

        // A different shadowed name is a different surprise.
        assert!(proxy_notice(&announcing_ctx(tmp.path(), "cmake"), "cmake").is_some());

        // ...and a different session starts over.
        let other = NoticeContext {
            session: Some("1000-9999".to_string()),
            ..announcing_ctx(tmp.path(), "cargo")
        };
        assert!(proxy_notice(&other, "cargo").is_some());
    }

    /// Regression: the notice used to be gated on stderr being a tty, which
    /// inverted it on the case the finding is about — `cargo build 2>&1 | tee
    /// build.log` is a person at a terminal keeping the log they will read six
    /// months later, and it was the one shape that got nothing. The decision
    /// now reads `interactive_session` (a controlling terminal) and never looks
    /// at where stderr points; this test runs under a harness that has replaced
    /// stderr with a pipe and still expects the line.
    #[test]
    fn notice_does_not_depend_on_where_stderr_points() {
        let tmp = tempfile::tempdir().unwrap();
        assert!(proxy_notice(&announcing_ctx(tmp.path(), "cargo"), "cargo").is_some());
    }

    /// No terminal anywhere in the picture means CI, cron or a daemon, where an
    /// extra stderr line is noise nobody will read.
    #[test]
    fn nothing_is_announced_without_a_terminal_session() {
        let tmp = tempfile::tempdir().unwrap();
        let ctx = NoticeContext {
            interactive_session: false,
            ..announcing_ctx(tmp.path(), "cargo")
        };
        assert!(proxy_notice(&ctx, "cargo").is_none());
        assert!(marker_dir_is_empty(tmp.path()), "and claims no session");
    }

    /// Regression: the notice fired from the proxies unconditionally, so
    /// `horus cmake --version` typed out by hand on a machine with no
    /// `~/.horus` at all announced that "`cmake` here runs through horus shell
    /// integration" and offered `horus env --uninstall` as the cure — a false
    /// claim about the user's shell and a remedy that does nothing.
    #[test]
    fn nothing_is_announced_when_no_shell_function_is_involved() {
        let tmp = tempfile::tempdir().unwrap();
        let ctx = NoticeContext {
            shell_proxy: ShellProxy::No,
            ..announcing_ctx(tmp.path(), "cmake")
        };
        assert!(proxy_notice(&ctx, "cmake").is_none());
        assert!(marker_dir_is_empty(tmp.path()), "and claims no session");
    }

    #[test]
    fn opting_out_silences_the_notice() {
        let tmp = tempfile::tempdir().unwrap();
        let ctx = NoticeContext {
            opted_out: true,
            ..announcing_ctx(tmp.path(), "cargo")
        };
        assert!(proxy_notice(&ctx, "cargo").is_none());
    }

    #[test]
    fn the_marker_decides_when_it_is_set() {
        assert_eq!(
            classify_shell_proxy(Some("pip3"), &[]),
            ShellProxy::Via("pip3".to_string())
        );
        assert_eq!(classify_shell_proxy(Some(""), &[]), ShellProxy::No);
        // The value arrives from the environment and ends up in a file name, so
        // anything not on the shadow list is believed as a claim but not as a
        // name.
        assert_eq!(
            classify_shell_proxy(Some("../../../etc/passwd"), &[]),
            ShellProxy::Legacy
        );
    }

    /// Shell integration written before the marker existed cannot set it, so
    /// its presence on disk has to stand in — but only until the installed
    /// functions know about the marker, after which its absence is proof that
    /// no function was involved.
    #[test]
    fn an_install_that_predates_the_marker_still_discloses() {
        let old_style = vec!["cargo() {\n  command horus cargo \"$@\"\n}\n".to_string()];
        assert_eq!(classify_shell_proxy(None, &old_style), ShellProxy::Legacy);

        let current = vec![format!(
            "cargo() {{\n  {SHELL_PROXY_VAR}=cargo command horus cargo \"$@\"\n}}\n"
        )];
        assert_eq!(classify_shell_proxy(None, &current), ShellProxy::No);
        assert_eq!(classify_shell_proxy(None, &[]), ShellProxy::No);
    }

    #[test]
    fn installed_integration_reads_only_what_is_there() {
        let tmp = tempfile::tempdir().unwrap();
        assert!(installed_integration(tmp.path()).is_empty());

        std::fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        std::fs::write(tmp.path().join(".horus/env.sh"), "the functions").unwrap();
        assert_eq!(
            installed_integration(tmp.path()),
            vec!["the functions".to_string()]
        );
    }

    /// Regression, and the reason this whole section was rewritten: the session
    /// key was uid + `getppid()`, on the reasoning that the shell function's
    /// `command horus cargo "$@"` makes the interactive shell our parent. Every
    /// pipeline element, command substitution, `( … )` and background job is a
    /// forked copy of that shell, so `cmake --version | head -1` had a brand
    /// new "session" on every run and printed the once-per-session notice every
    /// time. The child spawned here stands in for that fork: its parent pid is
    /// not ours — which is all the old key was made of — while the POSIX
    /// session id, which the new key is made of, is the one we already have.
    #[cfg(target_os = "linux")]
    #[test]
    fn session_key_survives_the_fork_that_broke_the_parent_pid() {
        let out = Command::new("sh")
            .arg("-c")
            .arg("awk '{print $4, $6}' /proc/self/stat")
            .output()
            .expect("spawn a child process in this session");
        let text = String::from_utf8_lossy(&out.stdout);
        let mut fields = text.split_whitespace();
        let child_ppid: u32 = fields.next().unwrap().parse().unwrap();
        let child_sid: u32 = fields.next().unwrap().parse().unwrap();

        // SAFETY: getppid()/getuid() take no arguments and cannot fail.
        let own_ppid = unsafe { libc::getppid() } as u32;
        assert_ne!(
            child_ppid, own_ppid,
            "the old key's only ingredient changes across a fork"
        );

        let uid = unsafe { libc::getuid() };
        assert_eq!(
            session_key_from(uid, child_sid),
            shell_session_key().unwrap(),
            "a forked child must compute the key its session already has"
        );
    }

    /// Pids get recycled; a later, unrelated session inheriting a live marker
    /// would silently skip its notice. The leader's start time makes the pair
    /// unique for as long as the machine is up.
    #[cfg(target_os = "linux")]
    #[test]
    fn session_key_carries_the_leader_start_time() {
        // SAFETY: getsid(0) asks about the calling process, which exists.
        let sid = unsafe { libc::getsid(0) } as u32;
        if session_leader_start_time(sid).is_some() {
            let key = shell_session_key().unwrap();
            assert_eq!(key.matches('-').count(), 2, "uid-sid-starttime, got {key}");
        }
    }

    #[test]
    fn start_time_is_parsed_from_the_end_of_the_process_name() {
        // Field 2 is attacker-chosen: a process really can be called `(ba d)sh`.
        let fields: Vec<String> = (3..=22).map(|n| n.to_string()).collect();
        let stat = format!("4242 ((ba d)sh) {}\n", fields.join(" "));
        assert_eq!(start_time_from_stat(&stat), Some(22));

        assert!(start_time_from_stat("truncated").is_none());
        // pid 0 has no /proc entry, so the key falls back to uid-sid.
        assert!(session_leader_start_time(0).is_none());
    }

    /// The notice used to be a separate statement in each of the five
    /// `run_*_proxy` bodies: deleting any of them broke nothing, and no test
    /// named any of them. Every proxy now learns its project root only by going
    /// through `enter_proxy`, which announces on the way in.
    #[test]
    fn entering_a_project_announces_the_tool() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"p\"\n").unwrap();

        let mut announced: Vec<String> = Vec::new();
        let entry = enter_proxy_with("cmake", tmp.path(), &mut |tool| {
            announced.push(tool.to_string())
        });

        assert!(matches!(entry, ProxyEntry::Project(_)));
        assert_eq!(announced, vec!["cmake".to_string()]);
    }

    #[test]
    fn passing_through_outside_a_project_announces_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let mut announced: Vec<String> = Vec::new();
        let entry = enter_proxy_with("cargo", tmp.path(), &mut |tool| {
            announced.push(tool.to_string())
        });

        assert!(matches!(entry, ProxyEntry::PassThrough));
        assert!(
            announced.is_empty(),
            "a pass-through rewrites nothing, so it has nothing to disclose"
        );
    }

    /// ...and each of the five entry points must actually be wired to it, under
    /// its own name. Deleting the disclosure from one proxy is a source edit
    /// this test sees, which is what "no test referenced any of the five call
    /// sites" cost the last version of this feature.
    #[test]
    fn every_proxy_entry_point_is_wired_to_enter_proxy() {
        let source = include_str!("proxy.rs");
        for (function, tool) in [
            ("run_cargo_proxy", "cargo"),
            ("run_pip_proxy", "pip"),
            ("run_cmake_proxy", "cmake"),
            ("run_conan_proxy", "conan"),
            ("run_vcpkg_proxy", "vcpkg"),
        ] {
            let start = source
                .find(&format!("pub fn {function}("))
                .unwrap_or_else(|| panic!("{function} no longer exists"));
            let body = &source[start..];
            let end = body.find("\n}\n").expect("function has an end");
            assert!(
                body[..end].contains(&format!("enter_proxy(\"{tool}\", &cwd)")),
                "{function} no longer discloses the indirection"
            );
        }
    }

    #[test]
    fn conan_ref_invalid() {
        assert!(parse_conan_ref("just-a-name").is_none());
    }

    #[test]
    fn vcpkg_strips_triplet() {
        let name = "fmt:x64-linux".split(':').next().unwrap();
        assert_eq!(name, "fmt");
    }

    #[test]
    fn vcpkg_no_triplet() {
        let name = "fmt".split(':').next().unwrap();
        assert_eq!(name, "fmt");
    }
}
