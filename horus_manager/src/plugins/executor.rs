//! Plugin Executor - discovers, verifies, and executes plugin binaries
//!
//! The executor handles the actual invocation of plugin binaries,
//! including verification and environment setup.

use anyhow::{anyhow, Result};
use colored::*;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

#[cfg(target_os = "linux")]
use std::os::unix::process::CommandExt as _;

use super::registry::{PluginEntry, PluginRegistry};
use super::resolver::PluginResolver;
use super::HORUS_VERSION;
use crate::cargo_utils::is_executable;

/// Executes plugin commands
pub struct PluginExecutor {
    resolver: PluginResolver,
}

impl PluginExecutor {
    /// Create a new executor with fresh resolver
    pub fn new() -> Result<Self> {
        Ok(Self {
            resolver: PluginResolver::new()?,
        })
    }

    /// Create executor with existing resolver
    pub fn with_resolver(resolver: PluginResolver) -> Self {
        Self { resolver }
    }

    /// Try to execute a command as a plugin
    ///
    /// Returns:
    /// - `Ok(Some(exit_code))` if plugin was found and executed (exit code as i32)
    /// - `Ok(None)` if command is not a plugin
    /// - `Err(...)` if plugin execution failed
    pub fn try_execute(&self, command: &str, args: &[String]) -> Result<Option<i32>> {
        // Check if command is a registered plugin
        let entry = match self.resolver.resolve(command) {
            Some(e) => e,
            None => {
                // Not a registered plugin - try path-based discovery
                if let Some(binary) = self.discover_plugin_binary(command) {
                    return self.execute_binary(&binary, args).map(Some);
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

        // Verify plugin before execution
        self.verify_plugin(entry)?;

        // Execute the plugin
        self.execute_plugin(entry, args).map(Some)
    }

    /// Discover plugin binary by scanning bin directories
    fn discover_plugin_binary(&self, command: &str) -> Option<PathBuf> {
        let binary_name = format!("horus-{}", command);

        // Check project bin first
        if let Some(root) = self.resolver.project_root() {
            let project_binary = PluginRegistry::project_bin_dir(root).join(&binary_name);
            if project_binary.exists() && is_executable(&project_binary) {
                return Some(project_binary);
            }
        }

        // Check global bin
        if let Ok(global_bin) = PluginRegistry::global_bin_dir() {
            let global_binary = global_bin.join(&binary_name);
            if global_binary.exists() && is_executable(&global_binary) {
                return Some(global_binary);
            }
        }

        // Check PATH as last resort
        if let Ok(path) = std::env::var("PATH") {
            for dir in path.split(':') {
                let binary = PathBuf::from(dir).join(&binary_name);
                if binary.exists() && is_executable(&binary) {
                    return Some(binary);
                }
            }
        }

        None
    }

    /// Verify plugin before execution
    fn verify_plugin(&self, entry: &PluginEntry) -> Result<()> {
        // Check binary exists
        if !entry.binary.exists() {
            return Err(anyhow!(
                "Plugin binary not found: {}\nRun 'horus install {}' to reinstall.",
                entry.binary.display(),
                entry.package
            ));
        }

        // Verify checksum
        let checksum = PluginRegistry::calculate_checksum(&entry.binary)?;
        if checksum != entry.checksum {
            return Err(anyhow!(
                "Plugin '{}' checksum mismatch!\nExpected: {}\nActual: {}\n\nThe binary may have been modified. Run 'horus verify {}' for details.",
                entry.package,
                entry.checksum,
                checksum,
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

    /// Execute a plugin
    fn execute_plugin(&self, entry: &PluginEntry, args: &[String]) -> Result<i32> {
        self.execute_binary(&entry.binary, args)
    }

    /// Execute a binary with sandboxed environment.
    ///
    /// Security measures applied:
    /// - **Environment sanitization**: clears all inherited env vars; passes
    ///   only `HORUS_VERSION`, `HORUS_PLUGIN`, and `PATH` (from parent).
    /// - **Linux sandbox** (via `pre_exec` in the child process):
    ///   - `RLIMIT_CPU`, `RLIMIT_FSIZE`, `RLIMIT_NOFILE` caps
    ///   - Inherited FD closure (fd > 2 are closed before exec)
    ///   - `PR_SET_NO_NEW_PRIVS` — no privilege escalation via setuid/setgid bits
    ///   - Seccomp-BPF deny list — blocks socket, ptrace, and container-escape
    ///     syscalls while allowing normal plugin operation
    fn execute_binary(&self, binary: &Path, args: &[String]) -> Result<i32> {
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
        // Find the plugin binary
        let binary = if let Some(entry) = self.resolver.resolve(command) {
            entry.binary.clone()
        } else if let Some(bin) = self.discover_plugin_binary(command) {
            bin
        } else {
            return Err(anyhow!(
                "Plugin '{}' not found. Install it with: horus install horus-{}",
                command,
                command
            ));
        };

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
}
