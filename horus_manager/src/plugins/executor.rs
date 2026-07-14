//! Plugin Executor - discovers, verifies, and executes plugin binaries
//!
//! The executor handles the actual invocation of plugin binaries,
//! including verification and environment setup.

use anyhow::{anyhow, Context, Result};
use colored::*;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

#[cfg(target_os = "linux")]
use std::os::unix::process::CommandExt as _;

use super::registry::{PluginEntry, PluginRegistry, PluginScope};
use super::resolver::PluginResolver;
use super::HORUS_VERSION;
use crate::cargo_utils::is_executable;

/// Provenance of a plugin binary about to be executed — the input to the
/// execution trust gate ([`PluginExecutor::ensure_execution_allowed`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PluginOrigin {
    /// Out-of-repo: global registry / global bin dir / system `PATH`. Reached
    /// only via a deliberate `horus install` (SEC1-gated) or the user's own
    /// `PATH`, so trusted by location; integrity-checked when an entry exists.
    Trusted,
    /// In-repo `<project>/.horus/` — part of any cloned checkout and therefore
    /// attacker-controlled. Must clear the out-of-repo trust store before it
    /// may execute.
    Project,
}

impl PluginOrigin {
    /// Classify a resolved registry entry by the scope it was read from.
    fn from_scope(scope: PluginScope) -> Self {
        match scope {
            PluginScope::Global => PluginOrigin::Trusted,
            PluginScope::Project => PluginOrigin::Project,
        }
    }
}

/// Executes plugin commands
pub struct PluginExecutor {
    resolver: PluginResolver,
    /// Override path for the out-of-repo trust store. `None` uses the real
    /// user-data location ([`crate::paths::trusted_plugins_path`]); tests inject
    /// a temp path so the execution gate is exercised hermetically.
    trust_store_path: Option<PathBuf>,
}

impl PluginExecutor {
    /// Create a new executor with fresh resolver
    pub fn new() -> Result<Self> {
        Ok(Self {
            resolver: PluginResolver::new()?,
            trust_store_path: None,
        })
    }

    /// Create executor with existing resolver
    pub fn with_resolver(resolver: PluginResolver) -> Self {
        Self {
            resolver,
            trust_store_path: None,
        }
    }

    /// Create an executor with an explicit trust-store path (testing only), so
    /// the provenance gate can be exercised without touching real user data.
    #[cfg(test)]
    pub fn with_resolver_and_trust_store(
        resolver: PluginResolver,
        trust_store_path: PathBuf,
    ) -> Self {
        Self {
            resolver,
            trust_store_path: Some(trust_store_path),
        }
    }

    /// Try to execute a command as a plugin
    ///
    /// Returns:
    /// - `Ok(Some(exit_code))` if plugin was found and executed (exit code as i32)
    /// - `Ok(None)` if command is not a plugin
    /// - `Err(...)` if plugin execution failed OR was refused by the trust gate
    ///
    /// Every path that reaches a binary funnels through [`Self::execute_binary`],
    /// which enforces the provenance gate ([`Self::ensure_execution_allowed`])
    /// before the process is spawned — there is no way to reach the exec sink
    /// that bypasses the gate.
    pub fn try_execute(&self, command: &str, args: &[String]) -> Result<Option<i32>> {
        // Check if command is a registered plugin (carrying its provenance scope).
        let (entry, scope) = match self.resolver.resolve_with_scope(command) {
            Some(resolved) => resolved,
            None => {
                // Not a registered plugin - try path-based discovery
                if let Some((binary, origin)) = self.discover_plugin_binary(command) {
                    return self
                        .execute_binary(&binary, args, None, origin, command)
                        .map(Some);
                }
                return Ok(None);
            }
        };

        // Check if disabled
        if self.resolver.is_disabled(command) {
            return Err(anyhow!(
                "Plugin '{}' is disabled. Run 'horus enable {}' to re-enable.",
                command,
                command
            ));
        }

        let origin = PluginOrigin::from_scope(scope);

        // Existence + compatibility diagnostics. The trust decision itself is
        // made at the exec sink inside execute_binary.
        self.verify_plugin(entry)?;

        // Execute the plugin (gate enforced inside execute_binary).
        self.execute_binary(&entry.binary, args, Some(entry), origin, command)
            .map(Some)
    }

    /// Discover plugin binary by scanning bin directories.
    ///
    /// Returns the binary path **and its provenance** ([`PluginOrigin`]).
    /// The project `.horus/bin` is scanned first (highest precedence) but is
    /// attacker-controlled in a cloned repo, so a match there is classified
    /// [`PluginOrigin::Project`] and must clear the trust gate before running.
    /// Global-bin and `PATH` matches are out-of-repo ([`PluginOrigin::Trusted`]).
    fn discover_plugin_binary(&self, command: &str) -> Option<(PathBuf, PluginOrigin)> {
        let binary_name = format!("horus-{}", command);

        // Check project bin first
        if let Some(root) = self.resolver.project_root() {
            let project_binary = PluginRegistry::project_bin_dir(root).join(&binary_name);
            if project_binary.exists() && is_executable(&project_binary) {
                return Some((project_binary, PluginOrigin::Project));
            }
        }

        // Check global bin
        if let Ok(global_bin) = PluginRegistry::global_bin_dir() {
            let global_binary = global_bin.join(&binary_name);
            if global_binary.exists() && is_executable(&global_binary) {
                return Some((global_binary, PluginOrigin::Trusted));
            }
        }

        // Check PATH as last resort
        if let Ok(path) = std::env::var("PATH") {
            for dir in path.split(':') {
                let binary = PathBuf::from(dir).join(&binary_name);
                if binary.exists() && is_executable(&binary) {
                    return Some((binary, PluginOrigin::Trusted));
                }
            }
        }

        None
    }

    /// Diagnostics run before execution: the binary must exist, and an
    /// incompatible HORUS version is warned about. This is **not** the trust
    /// decision — that is [`Self::ensure_execution_allowed`], enforced at the
    /// exec sink.
    fn verify_plugin(&self, entry: &PluginEntry) -> Result<()> {
        // Check binary exists
        if !entry.binary.exists() {
            return Err(anyhow!(
                "Plugin binary not found: {}\nRun 'horus install {}' to reinstall.",
                entry.binary.display(),
                entry.package
            ));
        }

        // Check compatibility
        if !self.resolver.global().is_compatible(entry) {
            log::warn!(
                "Plugin '{}' v{} may not be compatible with HORUS v{} (requires {} <= horus < {})",
                entry.package,
                entry.version,
                HORUS_VERSION,
                entry.compatibility.horus_min,
                entry.compatibility.horus_max
            );
            eprintln!(
                "{} Plugin '{}' v{} may not be compatible with HORUS v{}",
                crate::cli_output::ICON_WARN.yellow(),
                entry.package,
                entry.version,
                HORUS_VERSION
            );
            eprintln!(
                "       Requires: {} <= horus < {}",
                entry.compatibility.horus_min, entry.compatibility.horus_max
            );
        }

        Ok(())
    }

    /// The plugin-execution provenance gate — **fail closed**.
    ///
    /// The single trust decision every execution path shares (both
    /// [`Self::execute_binary`] and [`Self::spawn_background`] call it).
    ///
    /// * [`PluginOrigin::Trusted`] — out-of-repo binary (global registry/bin or
    ///   `PATH`). When a registry entry is present its checksum is enforced as
    ///   an integrity check; an **empty / absent / mismatched** checksum fails
    ///   closed (closing the "vacuous empty-checksum passes" hole).
    ///
    /// * [`PluginOrigin::Project`] — in-repo `.horus/` binary, attacker-
    ///   controlled. The `.horus/plugins.lock` checksum is NOT a trust anchor
    ///   (the repo controls it) and is ignored here. The binary may execute
    ///   only if its content hash is present in the **out-of-repo**
    ///   [`super::TrustStore`] (written by a deliberate
    ///   `horus plugin install --local` / `horus plugin trust`). Otherwise
    ///   execution is refused.
    fn ensure_execution_allowed(
        &self,
        binary: &Path,
        entry: Option<&PluginEntry>,
        origin: PluginOrigin,
        command: &str,
    ) -> Result<()> {
        match origin {
            PluginOrigin::Trusted => {
                // Out-of-repo. If we know what the binary should hash to, verify
                // it — detects post-install tampering. Fail closed on an empty
                // recorded checksum rather than treating "no checksum" as "any
                // binary is fine".
                if let Some(entry) = entry {
                    let actual = PluginRegistry::calculate_checksum(binary).with_context(|| {
                        format!("failed to hash plugin binary {}", binary.display())
                    })?;
                    if entry.checksum.is_empty() {
                        return Err(anyhow!(
                            "Plugin '{}' has no recorded checksum; refusing to execute.\n\
                             Reinstall it with 'horus install {}'.",
                            entry.package,
                            entry.package
                        ));
                    }
                    if actual != entry.checksum {
                        return Err(anyhow!(
                            "Plugin '{}' checksum mismatch!\nExpected: {}\nActual: {}\n\n\
                             The binary may have been modified. Reinstall with 'horus install {}'.",
                            entry.package,
                            entry.checksum,
                            actual,
                            entry.package
                        ));
                    }
                }
                Ok(())
            }
            PluginOrigin::Project => {
                // In-repo, attacker-controlled. Only an out-of-repo trust record
                // (keyed by content hash) confers trust — proving the binary was
                // deliberately installed/trusted ON THIS MACHINE, not merely
                // shipped inside a cloned repository.
                let actual = PluginRegistry::calculate_checksum(binary).with_context(|| {
                    format!("failed to hash plugin binary {}", binary.display())
                })?;
                let store = self.load_trust_store();
                if store.contains(&actual) {
                    return Ok(());
                }
                Err(anyhow!(
                    "Refusing to execute untrusted project-local plugin '{}'.\n\
                     \n\
                     It lives in this workspace's .horus/ (part of the checkout) and has not\n\
                     been trusted on this machine. A checksum in .horus/plugins.lock is NOT proof\n\
                     of trust — the repository controls that file.\n\
                     \n\
                     If you trust this plugin, run:  horus plugin trust {}\n\
                     Or (re)install it deliberately:  horus plugin install --local <package>",
                    command,
                    command
                ))
            }
        }
    }

    /// Resolve the effective trust-store path (test override or real location).
    fn trust_store_path(&self) -> Result<PathBuf> {
        match &self.trust_store_path {
            Some(p) => Ok(p.clone()),
            None => crate::paths::trusted_plugins_path(),
        }
    }

    /// Load the out-of-repo trust store, failing **closed** (empty store ⇒
    /// nothing trusted) if it is missing, unreadable, or corrupt.
    fn load_trust_store(&self) -> super::trust::TrustStore {
        match self.trust_store_path() {
            Ok(path) => super::trust::TrustStore::load(&path).unwrap_or_else(|e| {
                log::warn!(
                    "Plugin trust store unreadable ({e}); treating all project-local plugins as untrusted"
                );
                super::trust::TrustStore::default()
            }),
            Err(e) => {
                log::warn!(
                    "Cannot locate plugin trust store ({e}); treating all project-local plugins as untrusted"
                );
                super::trust::TrustStore::default()
            }
        }
    }

    /// Execute a binary with sandboxed environment, **after** clearing the
    /// provenance gate.
    ///
    /// Security measures applied:
    /// - **Provenance gate** ([`Self::ensure_execution_allowed`]): refuses to
    ///   run an untrusted project-local binary (fail closed).
    /// - **Linux sandbox** (via `pre_exec` in the child process):
    ///   - `RLIMIT_CPU`, `RLIMIT_FSIZE`, `RLIMIT_NOFILE` caps
    ///   - Inherited FD closure (fd > 2 are closed before exec)
    ///   - `PR_SET_NO_NEW_PRIVS` — no privilege escalation via setuid/setgid bits
    ///   - Seccomp-BPF deny list — blocks socket, ptrace, and container-escape
    ///     syscalls while allowing normal plugin operation
    fn execute_binary(
        &self,
        binary: &Path,
        args: &[String],
        entry: Option<&PluginEntry>,
        origin: PluginOrigin,
        command: &str,
    ) -> Result<i32> {
        // FAIL CLOSED: nothing runs without cleared provenance.
        self.ensure_execution_allowed(binary, entry, origin, command)?;

        let mut cmd = Command::new(binary);

        cmd.args(args);

        // Inherit full environment so library paths (LD_LIBRARY_PATH, MUJOCO_DIR),
        // display vars (DISPLAY, MESA_GL_VERSION_OVERRIDE), and user config are
        // available to plugins. Override specific HORUS vars on top.
        cmd.env("HORUS_VERSION", HORUS_VERSION);
        cmd.env("HORUS_PLUGIN", "1");

        cmd.stdin(Stdio::inherit())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit());

        // On Linux: apply rlimits, FD cleanup, no_new_privs, and seccomp
        // in the child process (between fork and exec).
        #[cfg(target_os = "linux")]
        {
            let plugin_dir = binary.parent().map(|p| p.to_path_buf());
            // SAFETY: we only call async-signal-safe libc functions inside the
            // closure (setrlimit, close, prctl).  No Rust allocator is used.
            unsafe {
                cmd.pre_exec(move || super::sandbox::apply(plugin_dir.as_deref()));
            }
        }

        let status = cmd.status()?;
        Ok(status.code().unwrap_or(1))
    }

    /// Spawn a plugin as a background process (non-blocking).
    ///
    /// Unlike `try_execute()` which blocks until completion, this returns
    /// a `Child` handle immediately. Used by `horus run --sim` to launch
    /// sim3d in the background while the user's robot code runs.
    ///
    /// Extra environment variables can be passed via `extra_env`.
    pub fn spawn_background(
        &self,
        command: &str,
        args: &[String],
        extra_env: &[(String, String)],
    ) -> Result<std::process::Child> {
        // Find the plugin binary AND its provenance so the trust gate can run.
        // `spawn_background` does not go through `execute_binary`, so it must
        // enforce the exact same gate itself — otherwise it is a bypass.
        let (binary, entry_owned, origin) =
            if let Some((entry, scope)) = self.resolver.resolve_with_scope(command) {
                (
                    entry.binary.clone(),
                    Some(entry.clone()),
                    PluginOrigin::from_scope(scope),
                )
            } else if let Some((bin, origin)) = self.discover_plugin_binary(command) {
                (bin, None, origin)
            } else {
                return Err(anyhow!(
                    "Plugin '{}' not found. Install it with: horus install horus-{}",
                    command,
                    command
                ));
            };

        // FAIL CLOSED before spawning: same provenance gate as execute_binary.
        self.ensure_execution_allowed(&binary, entry_owned.as_ref(), origin, command)?;

        let mut cmd = Command::new(&binary);
        cmd.args(args);

        // Inherit full environment so library paths (LD_LIBRARY_PATH, MUJOCO_DIR),
        // display vars (DISPLAY, MESA_GL_VERSION_OVERRIDE), and user config are available.
        // Override specific HORUS vars on top.
        cmd.env("HORUS_VERSION", HORUS_VERSION);
        cmd.env("HORUS_PLUGIN", "1");
        // Pass extra env vars (sim params, driver mode, etc.)
        for (key, value) in extra_env {
            cmd.env(key, value);
        }

        cmd.stdout(Stdio::inherit()).stderr(Stdio::inherit());

        let child = cmd.spawn().map_err(|e| {
            anyhow!(
                "Failed to launch plugin '{}' ({}): {}",
                command,
                binary.display(),
                e
            )
        })?;

        log::info!(
            "Launched plugin '{}' in background (PID: {})",
            command,
            child.id()
        );

        Ok(child)
    }

    /// Get reference to resolver
    pub fn resolver(&self) -> &PluginResolver {
        &self.resolver
    }

    /// List all available plugin commands with info
    pub fn list_plugins(&self) -> Vec<PluginInfo> {
        let mut plugins = Vec::new();

        for resolved in self.resolver.all_plugins() {
            plugins.push(PluginInfo {
                command: resolved.command,
                package: resolved.entry.package,
                version: resolved.entry.version,
                scope: match resolved.scope {
                    super::PluginScope::Global => "global".to_string(),
                    super::PluginScope::Project => "project".to_string(),
                },
                is_overridden: resolved.is_overridden,
                description: resolved
                    .entry
                    .commands
                    .first()
                    .map(|c| c.description.clone())
                    .unwrap_or_default(),
            });
        }

        plugins
    }
}

impl Default for PluginExecutor {
    fn default() -> Self {
        Self::new().unwrap_or_else(|_| Self {
            resolver: PluginResolver::default(),
            trust_store_path: None,
        })
    }
}

/// Information about an installed plugin
#[derive(Debug, Clone)]
pub struct PluginInfo {
    pub command: String,
    pub package: String,
    pub version: String,
    pub scope: String,
    pub is_overridden: bool,
    pub description: String,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugins::registry::{CommandInfo, Compatibility, PluginSource};
    use chrono::Utc;
    use std::fs;
    use tempfile::TempDir;

    fn make_test_entry(package: &str, binary: PathBuf) -> PluginEntry {
        PluginEntry {
            package: package.to_string(),
            version: "1.0.0".to_string(),
            source: PluginSource::Registry,
            binary,
            checksum: "sha256:test".to_string(),
            signature: None,
            installed_at: Utc::now(),
            installed_by: "0.1.0".to_string(),
            compatibility: Compatibility::default(),
            commands: vec![CommandInfo {
                name: "run".to_string(),
                description: "Run command".to_string(),
            }],
            permissions: vec![],
        }
    }

    #[test]
    fn test_executor_not_a_plugin() {
        let executor = PluginExecutor::default();
        let result = executor.try_execute("nonexistent", &[]).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn test_list_plugins() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin(
            "nav2",
            make_test_entry("nav2-lite", PathBuf::from("/bin/nav2")),
        );

        let resolver = PluginResolver::with_registries(global, None, None);
        let executor = PluginExecutor::with_resolver(resolver);

        let plugins = executor.list_plugins();
        assert_eq!(plugins.len(), 1);
        assert_eq!(plugins[0].command, "nav2");
        assert_eq!(plugins[0].scope, "global");
    }

    #[test]
    #[cfg(unix)]
    fn test_is_executable() {
        let temp_dir = TempDir::new().unwrap();
        let file_path = temp_dir.path().join("test_binary");

        // Create non-executable file
        fs::write(&file_path, "test").unwrap();
        assert!(!is_executable(&file_path));

        // Make executable
        use std::os::unix::fs::PermissionsExt;
        let mut perms = fs::metadata(&file_path).unwrap().permissions();
        perms.set_mode(0o755);
        fs::set_permissions(&file_path, perms).unwrap();
        assert!(is_executable(&file_path));
    }

    // ── Additional executor tests ──────────────────────────────────────

    #[test]
    fn test_executor_default_has_no_resolver() {
        let executor = PluginExecutor::default();
        // Default executor should handle try_execute gracefully
        let result = executor.try_execute("anything", &[]);
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_list_plugins_empty() {
        let executor = PluginExecutor::default();
        let plugins = executor.list_plugins();
        assert!(plugins.is_empty());
    }

    #[test]
    fn test_list_plugins_multiple() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin(
            "nav2",
            make_test_entry("nav2-lite", PathBuf::from("/bin/nav2")),
        );
        global.register_plugin(
            "sim",
            make_test_entry("horus-sim", PathBuf::from("/bin/sim")),
        );

        let resolver = PluginResolver::with_registries(global, None, None);
        let executor = PluginExecutor::with_resolver(resolver);

        let plugins = executor.list_plugins();
        assert_eq!(plugins.len(), 2);
    }

    #[test]
    fn test_list_plugins_shows_scope() {
        let global = PluginRegistry::new_global();
        let mut project = PluginRegistry::new_project("test");
        project.register_plugin(
            "local-tool",
            make_test_entry("local-tool", PathBuf::from("/bin/local")),
        );

        let resolver =
            PluginResolver::with_registries(global, Some(project), Some(PathBuf::from("/test")));
        let executor = PluginExecutor::with_resolver(resolver);

        let plugins = executor.list_plugins();
        assert_eq!(plugins.len(), 1);
        assert_eq!(plugins[0].scope, "project");
    }

    #[test]
    fn test_try_execute_unregistered_returns_none() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin(
            "nav2",
            make_test_entry("nav2-lite", PathBuf::from("/bin/nav2")),
        );

        let resolver = PluginResolver::with_registries(global, None, None);
        let executor = PluginExecutor::with_resolver(resolver);

        // "unknown" is not registered
        let result = executor.try_execute("unknown", &[]).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn test_plugin_info_struct() {
        let info = PluginInfo {
            command: "nav2".to_string(),
            package: "nav2-lite".to_string(),
            version: "1.0.0".to_string(),
            scope: "global".to_string(),
            is_overridden: false,
            description: String::new(),
        };
        assert_eq!(info.command, "nav2");
        assert!(!info.is_overridden);
    }

    #[test]
    fn test_plugin_info_override_flag() {
        let info = PluginInfo {
            command: "nav2".to_string(),
            package: "nav2-lite".to_string(),
            version: "2.0.0".to_string(),
            scope: "project".to_string(),
            is_overridden: true,
            description: String::new(),
        };
        assert!(info.is_overridden);
        assert_eq!(info.scope, "project");
    }

    // ── Provenance / trust gate (SEC2 + SEC3) ───────────────────────────

    fn write_temp_binary(dir: &TempDir, name: &str, contents: &[u8]) -> PathBuf {
        let p = dir.path().join(name);
        fs::write(&p, contents).unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mut perms = fs::metadata(&p).unwrap().permissions();
            perms.set_mode(0o755);
            fs::set_permissions(&p, perms).unwrap();
        }
        p
    }

    fn gate_executor(store_path: PathBuf) -> PluginExecutor {
        PluginExecutor::with_resolver_and_trust_store(
            PluginResolver::with_registries(PluginRegistry::new_global(), None, None),
            store_path,
        )
    }

    /// SEC2 (the discriminating case): a project-scoped entry whose binary hash
    /// MATCHES its in-repo `plugins.lock` checksum but is NOT in the out-of-repo
    /// trust store must still be REFUSED. Proves the lockfile checksum is no
    /// longer the trust decision.
    #[test]
    fn gate_project_vacuous_lock_checksum_but_untrusted_is_refused() {
        let tmp = TempDir::new().unwrap();
        let bin = write_temp_binary(&tmp, "horus-evil", b"malicious payload");
        let real_hash = PluginRegistry::calculate_checksum(&bin).unwrap();

        // Attacker-controlled lock records the CORRECT hash of their own binary.
        let mut entry = make_test_entry("evil-pkg", bin.clone());
        entry.checksum = real_hash;

        // Trust store file does not exist => empty => nothing trusted.
        let exec = gate_executor(tmp.path().join("trusted_plugins.json"));
        let res = exec.ensure_execution_allowed(&bin, Some(&entry), PluginOrigin::Project, "evil");

        assert!(
            res.is_err(),
            "a matching in-repo lock checksum must NOT confer trust"
        );
        let msg = format!("{}", res.unwrap_err());
        assert!(
            msg.contains("untrusted project-local plugin"),
            "unexpected error: {msg}"
        );
    }

    /// SEC3: a discovered project-local binary with NO registry entry and no
    /// trust record is refused at the sink.
    #[test]
    fn gate_project_discovered_no_entry_untrusted_is_refused() {
        let tmp = TempDir::new().unwrap();
        let bin = write_temp_binary(&tmp, "horus-x", b"payload bytes");
        let exec = gate_executor(tmp.path().join("trusted_plugins.json"));
        exec.ensure_execution_allowed(&bin, None, PluginOrigin::Project, "x")
            .unwrap_err();
    }

    /// A project-local binary whose content hash is in the out-of-repo trust
    /// store (deliberate `install --local` / `plugin trust`) is allowed.
    #[test]
    fn gate_project_trusted_binary_is_allowed() {
        let tmp = TempDir::new().unwrap();
        let bin = write_temp_binary(&tmp, "horus-devtool", b"my local dev tool");
        let hash = PluginRegistry::calculate_checksum(&bin).unwrap();

        let store_path = tmp.path().join("trusted_plugins.json");
        let mut store = crate::plugins::TrustStore::default();
        store.trust(&hash, "devtool", &bin).unwrap();
        store.save(&store_path).unwrap();

        let exec = gate_executor(store_path);
        exec.ensure_execution_allowed(&bin, None, PluginOrigin::Project, "devtool")
            .unwrap();
    }

    /// Tampering with a trusted project binary invalidates trust (hash changes).
    #[test]
    fn gate_project_trusted_then_tampered_is_refused() {
        let tmp = TempDir::new().unwrap();
        let bin = write_temp_binary(&tmp, "horus-devtool", b"original");
        let hash = PluginRegistry::calculate_checksum(&bin).unwrap();

        let store_path = tmp.path().join("trusted_plugins.json");
        let mut store = crate::plugins::TrustStore::default();
        store.trust(&hash, "devtool", &bin).unwrap();
        store.save(&store_path).unwrap();

        // Swap the binary contents after it was trusted.
        fs::write(&bin, b"swapped-in malware").unwrap();

        let exec = gate_executor(store_path);
        assert!(
            exec.ensure_execution_allowed(&bin, None, PluginOrigin::Project, "devtool")
                .is_err(),
            "a tampered binary must fall out of the trust set"
        );
    }

    /// SEC2 (empty checksum): a trusted-origin entry with an EMPTY recorded
    /// checksum fails closed (empty must never vacuously pass).
    #[test]
    fn gate_trusted_origin_empty_checksum_fails_closed() {
        let tmp = TempDir::new().unwrap();
        let bin = write_temp_binary(&tmp, "horus-g", b"global tool");
        let mut entry = make_test_entry("g-pkg", bin.clone());
        entry.checksum = String::new();

        let exec = gate_executor(tmp.path().join("trusted_plugins.json"));
        exec.ensure_execution_allowed(&bin, Some(&entry), PluginOrigin::Trusted, "g")
            .unwrap_err();
    }

    /// A trusted-origin entry with a MISMATCHED checksum fails closed (tamper).
    #[test]
    fn gate_trusted_origin_mismatched_checksum_fails_closed() {
        let tmp = TempDir::new().unwrap();
        let bin = write_temp_binary(&tmp, "horus-g", b"global tool");
        let mut entry = make_test_entry("g-pkg", bin.clone());
        entry.checksum = "sha256:0000000000000000000000000000000000000000000000000000000000000000"
            .to_string();

        let exec = gate_executor(tmp.path().join("trusted_plugins.json"));
        exec.ensure_execution_allowed(&bin, Some(&entry), PluginOrigin::Trusted, "g")
            .unwrap_err();
    }

    /// A trusted-origin entry with a MATCHING checksum is allowed.
    #[test]
    fn gate_trusted_origin_matching_checksum_allowed() {
        let tmp = TempDir::new().unwrap();
        let bin = write_temp_binary(&tmp, "horus-g", b"global tool");
        let mut entry = make_test_entry("g-pkg", bin.clone());
        entry.checksum = PluginRegistry::calculate_checksum(&bin).unwrap();

        let exec = gate_executor(tmp.path().join("trusted_plugins.json"));
        exec.ensure_execution_allowed(&bin, Some(&entry), PluginOrigin::Trusted, "g")
            .unwrap();
    }

    /// End-to-end: `try_execute` refuses an untrusted project plugin AT the sink
    /// (never spawns it), even though its lock checksum matches the binary.
    #[test]
    fn try_execute_project_plugin_untrusted_is_refused_at_sink() {
        let tmp = TempDir::new().unwrap();
        let proj_root = tmp.path().to_path_buf();
        let bin_dir = PluginRegistry::project_bin_dir(&proj_root);
        fs::create_dir_all(&bin_dir).unwrap();
        let bin = bin_dir.join("horus-evil");
        fs::write(&bin, b"payload").unwrap();
        let hash = PluginRegistry::calculate_checksum(&bin).unwrap();

        let mut project = PluginRegistry::new_project("victim");
        let mut entry = make_test_entry("evil-pkg", bin.clone());
        entry.checksum = hash; // vacuous in-repo match
        project.register_plugin("evil", entry);

        let resolver = PluginResolver::with_registries(
            PluginRegistry::new_global(),
            Some(project),
            Some(proj_root),
        );
        let exec = PluginExecutor::with_resolver_and_trust_store(
            resolver,
            tmp.path().join("trusted_plugins.json"), // empty
        );

        let res = exec.try_execute("evil", &[]);
        assert!(res.is_err(), "untrusted project plugin must be refused");
        let msg = format!("{}", res.unwrap_err());
        assert!(
            msg.contains("untrusted project-local plugin"),
            "expected trust-gate refusal, got: {msg}"
        );
    }
}
