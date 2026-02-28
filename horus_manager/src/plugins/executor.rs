//! Plugin Executor - discovers, verifies, and executes plugin binaries
//!
//! The executor handles the actual invocation of plugin binaries,
//! including verification and environment setup.

use anyhow::{anyhow, Result};
use colored::*;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

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

    /// Execute a binary with args
    fn execute_binary(&self, binary: &Path, args: &[String]) -> Result<i32> {
        let status = Command::new(binary)
            .args(args)
            .env("HORUS_VERSION", HORUS_VERSION)
            .env("HORUS_PLUGIN", "1")
            .stdin(Stdio::inherit())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .status()?;

        Ok(status.code().unwrap_or(1))
    }

    /// Get reference to resolver
    pub fn resolver(&self) -> &PluginResolver {
        &self.resolver
    }

    /// Get mutable reference to resolver
    pub fn resolver_mut(&mut self) -> &mut PluginResolver {
        &mut self.resolver
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

    /// Print help including available plugins
    pub fn print_plugin_help(&self) {
        let plugins = self.list_plugins();
        let active: Vec<_> = plugins.iter().filter(|p| !p.is_overridden).collect();

        if active.is_empty() {
            return;
        }

        println!("\n{}:", "INSTALLED PLUGINS".cyan().bold());

        for plugin in active {
            let scope_indicator = if plugin.scope == "project" {
                "(project)".dimmed()
            } else {
                "(global)".dimmed()
            };

            println!(
                "    {}    {}  {}",
                plugin.command.green(),
                plugin.description.dimmed(),
                scope_indicator
            );
        }

        println!(
            "\nRun '{}' for more information on a plugin.",
            "horus <plugin> --help".cyan()
        );
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
}
