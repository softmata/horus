//! Plugin Resolver - resolves plugins from project and global registries
//!
//! The resolver implements a two-tier lookup:
//! 1. Project `.horus/plugins.lock` (highest priority)
//! 2. Global `~/.horus/plugins.lock` (fallback)

use anyhow::{anyhow, Result};
use std::path::{Path, PathBuf};

use super::registry::{PluginEntry, PluginRegistry, PluginScope};

/// Resolves plugins from project and global registries
pub struct PluginResolver {
    /// Global plugin registry
    global_registry: PluginRegistry,

    /// Project plugin registry (if in a project)
    project_registry: Option<PluginRegistry>,

    /// Project root path (if in a project)
    project_root: Option<PathBuf>,
}

impl PluginResolver {
    /// Create a new resolver, loading registries from disk
    pub fn new() -> Result<Self> {
        let global_registry = PluginRegistry::load_global()?;

        let (project_registry, project_root) = if let Some(root) = Self::find_project_root() {
            (PluginRegistry::load_project(&root), Some(root))
        } else {
            (None, None)
        };

        Ok(Self {
            global_registry,
            project_registry,
            project_root,
        })
    }

    /// Create a resolver with explicit registries (for testing)
    pub fn with_registries(
        global: PluginRegistry,
        project: Option<PluginRegistry>,
        project_root: Option<PathBuf>,
    ) -> Self {
        Self {
            global_registry: global,
            project_registry: project,
            project_root,
        }
    }

    /// Find project root by searching upward for markers
    pub fn find_project_root() -> Option<PathBuf> {
        let mut current = std::env::current_dir().ok()?;

        for _ in 0..10 {
            // Check for .horus directory or horus.toml
            if current.join(".horus").exists() || current.join("horus.toml").exists() {
                return Some(current);
            }

            if let Some(parent) = current.parent() {
                current = parent.to_path_buf();
            } else {
                break;
            }
        }

        None
    }

    /// Resolve a plugin command to its entry
    ///
    /// Resolution order:
    /// 1. Project registry (if exists and not disabled)
    /// 2. Global registry (if project inherits global or no project)
    pub fn resolve(&self, command: &str) -> Option<&PluginEntry> {
        // Try project registry first
        if let Some(ref project) = self.project_registry {
            // Check if plugin exists in project
            if let Some(entry) = project.get_plugin(command) {
                return Some(entry);
            }

            // Check if project inherits from global
            if !project.inherit_global {
                return None;
            }
        }

        // Fall back to global registry
        self.global_registry.get_plugin(command)
    }

    /// Check if a command is a registered plugin
    pub fn is_plugin(&self, command: &str) -> bool {
        self.resolve(command).is_some()
    }

    /// Check if a plugin is disabled
    pub fn is_disabled(&self, command: &str) -> bool {
        // Check project first
        if let Some(ref project) = self.project_registry {
            if project.is_disabled(command) {
                return true;
            }
        }

        // Check global
        self.global_registry.is_disabled(command)
    }

    /// Get all available plugin commands (merged from project and global)
    pub fn all_commands(&self) -> Vec<String> {
        let mut commands: Vec<String> = Vec::new();

        // Add global commands first
        for cmd in self.global_registry.active_commands() {
            commands.push(cmd.to_string());
        }

        // Add/override with project commands
        if let Some(ref project) = self.project_registry {
            for cmd in project.active_commands() {
                if !commands.contains(&cmd.to_string()) {
                    commands.push(cmd.to_string());
                }
            }
        }

        commands.sort();
        commands
    }

    /// Get all plugins with their scope information
    pub fn all_plugins(&self) -> Vec<ResolvedPlugin> {
        let mut plugins = Vec::new();

        // Add global plugins
        for (cmd, entry) in &self.global_registry.plugins {
            plugins.push(ResolvedPlugin {
                command: cmd.clone(),
                entry: entry.clone(),
                scope: PluginScope::Global,
                is_overridden: false,
            });
        }

        // Add project plugins, marking overrides
        if let Some(ref project) = self.project_registry {
            for (cmd, entry) in &project.plugins {
                // Check if this overrides a global plugin
                let overrides_global = self.global_registry.plugins.contains_key(cmd);

                if overrides_global {
                    // Mark the global one as overridden
                    if let Some(p) = plugins.iter_mut().find(|p| &p.command == cmd) {
                        p.is_overridden = true;
                    }
                }

                plugins.push(ResolvedPlugin {
                    command: cmd.clone(),
                    entry: entry.clone(),
                    scope: PluginScope::Project,
                    is_overridden: false,
                });
            }
        }

        plugins.sort_by(|a, b| a.command.cmp(&b.command));
        plugins
    }

    /// Get reference to global registry
    pub fn global(&self) -> &PluginRegistry {
        &self.global_registry
    }

    /// Get mutable reference to global registry
    pub fn global_mut(&mut self) -> &mut PluginRegistry {
        &mut self.global_registry
    }

    /// Get reference to project registry
    pub fn project(&self) -> Option<&PluginRegistry> {
        self.project_registry.as_ref()
    }

    /// Get mutable reference to project registry
    pub fn project_mut(&mut self) -> Option<&mut PluginRegistry> {
        self.project_registry.as_mut()
    }

    /// Get or create project registry
    pub fn get_or_create_project(&mut self, project_name: &str) -> &mut PluginRegistry {
        if self.project_registry.is_none() {
            self.project_registry = Some(PluginRegistry::new_project(project_name));
        }
        self.project_registry
            .as_mut()
            .expect("project_registry initialized above")
    }

    /// Get project root path
    pub fn project_root(&self) -> Option<&Path> {
        self.project_root.as_deref()
    }

    /// Save all modified registries
    pub fn save(&self) -> Result<()> {
        // Save global
        self.global_registry.save()?;

        // Save project if exists
        if let (Some(ref project), Some(ref root)) = (&self.project_registry, &self.project_root) {
            let path = PluginRegistry::project_path(root);
            project.save_to(&path)?;
        }

        Ok(())
    }

    /// Save only global registry
    pub fn save_global(&self) -> Result<()> {
        self.global_registry.save()
    }

    /// Save only project registry
    pub fn save_project(&self) -> Result<()> {
        if let (Some(ref project), Some(ref root)) = (&self.project_registry, &self.project_root) {
            let path = PluginRegistry::project_path(root);
            project.save_to(&path)?;
            Ok(())
        } else {
            Err(anyhow!("No project registry to save"))
        }
    }

    /// Check if we're in a project
    pub fn in_project(&self) -> bool {
        self.project_root.is_some()
    }

    /// Verify all plugins
    pub fn verify_all(&self) -> Vec<VerificationResult> {
        let mut results = Vec::new();

        // Verify global plugins
        for cmd in self.global_registry.plugins.keys() {
            let result = match self.global_registry.verify_plugin(cmd) {
                Ok(true) => VerificationResult {
                    command: cmd.clone(),
                    scope: PluginScope::Global,
                    status: VerificationStatus::Valid,
                    error: None,
                },
                Ok(false) => VerificationResult {
                    command: cmd.clone(),
                    scope: PluginScope::Global,
                    status: VerificationStatus::ChecksumMismatch,
                    error: None,
                },
                Err(e) => VerificationResult {
                    command: cmd.clone(),
                    scope: PluginScope::Global,
                    status: VerificationStatus::Error,
                    error: Some(e.to_string()),
                },
            };
            results.push(result);
        }

        // Verify project plugins
        if let Some(ref project) = self.project_registry {
            for cmd in project.plugins.keys() {
                let result = match project.verify_plugin(cmd) {
                    Ok(true) => VerificationResult {
                        command: cmd.clone(),
                        scope: PluginScope::Project,
                        status: VerificationStatus::Valid,
                        error: None,
                    },
                    Ok(false) => VerificationResult {
                        command: cmd.clone(),
                        scope: PluginScope::Project,
                        status: VerificationStatus::ChecksumMismatch,
                        error: None,
                    },
                    Err(e) => VerificationResult {
                        command: cmd.clone(),
                        scope: PluginScope::Project,
                        status: VerificationStatus::Error,
                        error: Some(e.to_string()),
                    },
                };
                results.push(result);
            }
        }

        results
    }
}

impl Default for PluginResolver {
    fn default() -> Self {
        Self::new().unwrap_or_else(|_| Self {
            global_registry: PluginRegistry::new_global(),
            project_registry: None,
            project_root: None,
        })
    }
}

/// A resolved plugin with scope information
#[derive(Debug, Clone)]
pub struct ResolvedPlugin {
    /// Command name
    pub command: String,

    /// Plugin entry
    pub entry: PluginEntry,

    /// Which registry it came from
    pub scope: PluginScope,

    /// Whether this is overridden by a project plugin
    pub is_overridden: bool,
}

/// Result of plugin verification
#[derive(Debug, Clone)]
pub struct VerificationResult {
    pub command: String,
    pub scope: PluginScope,
    pub status: VerificationStatus,
    pub error: Option<String>,
}

/// Status of plugin verification
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VerificationStatus {
    Valid,
    ChecksumMismatch,
    Error,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugins::registry::{CommandInfo, Compatibility, PluginSource};
    use chrono::Utc;

    fn make_test_entry(package: &str, version: &str) -> PluginEntry {
        PluginEntry {
            package: package.to_string(),
            version: version.to_string(),
            source: PluginSource::Registry,
            binary: PathBuf::from("/bin/test"),
            checksum: "sha256:test".to_string(),
            signature: None,
            installed_at: Utc::now(),
            installed_by: "0.1.0".to_string(),
            compatibility: Compatibility::default(),
            commands: vec![CommandInfo {
                name: "run".to_string(),
                description: "Run".to_string(),
            }],
            permissions: vec![],
        }
    }

    #[test]
    fn test_resolve_global_only() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav2", make_test_entry("nav2-lite", "1.0.0"));

        let resolver = PluginResolver::with_registries(global, None, None);

        assert!(resolver.is_plugin("nav2"));
        assert!(!resolver.is_plugin("unknown"));

        let entry = resolver.resolve("nav2").unwrap();
        assert_eq!(entry.package, "nav2-lite");
    }

    #[test]
    fn test_resolve_project_override() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav2", make_test_entry("nav2-lite", "1.0.0"));

        let mut project = PluginRegistry::new_project("test");
        project.register_plugin("nav2", make_test_entry("nav2-lite", "2.0.0"));

        let resolver =
            PluginResolver::with_registries(global, Some(project), Some(PathBuf::from("/test")));

        // Should resolve to project version
        let entry = resolver.resolve("nav2").unwrap();
        assert_eq!(entry.version, "2.0.0");
    }

    #[test]
    fn test_resolve_no_inherit() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav2", make_test_entry("nav2-lite", "1.0.0"));

        let mut project = PluginRegistry::new_project("test");
        project.inherit_global = false;
        project.register_plugin("viz", make_test_entry("horus-viz", "1.0.0"));

        let resolver =
            PluginResolver::with_registries(global, Some(project), Some(PathBuf::from("/test")));

        // nav2 should NOT resolve (inherit_global = false)
        assert!(resolver.resolve("nav2").is_none());

        // viz should resolve
        assert!(resolver.resolve("viz").is_some());
    }

    #[test]
    fn test_all_commands() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav2", make_test_entry("nav2-lite", "1.0.0"));
        global.register_plugin("viz", make_test_entry("horus-viz", "1.0.0"));

        let mut project = PluginRegistry::new_project("test");
        project.register_plugin("slam", make_test_entry("horus-slam", "1.0.0"));

        let resolver =
            PluginResolver::with_registries(global, Some(project), Some(PathBuf::from("/test")));

        let commands = resolver.all_commands();
        assert!(commands.contains(&"nav2".to_string()));
        assert!(commands.contains(&"viz".to_string()));
        assert!(commands.contains(&"slam".to_string()));
    }

    // ── Additional resolver tests ──────────────────────────────────────

    #[test]
    fn test_is_disabled_global() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav", make_test_entry("nav", "1.0.0"));
        global.disable_plugin("nav", "testing").unwrap();

        let resolver = PluginResolver::with_registries(global, None, None);
        assert!(resolver.is_disabled("nav"));
    }

    #[test]
    fn test_is_disabled_false_for_active() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav", make_test_entry("nav", "1.0.0"));

        let resolver = PluginResolver::with_registries(global, None, None);
        assert!(!resolver.is_disabled("nav"));
    }

    #[test]
    fn test_is_disabled_false_for_unknown() {
        let global = PluginRegistry::new_global();
        let resolver = PluginResolver::with_registries(global, None, None);
        assert!(!resolver.is_disabled("nonexistent"));
    }

    #[test]
    fn test_all_plugins_with_override() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav", make_test_entry("nav-global", "1.0.0"));

        let mut project = PluginRegistry::new_project("test");
        project.register_plugin("nav", make_test_entry("nav-project", "2.0.0"));

        let resolver =
            PluginResolver::with_registries(global, Some(project), Some(PathBuf::from("/test")));

        let plugins = resolver.all_plugins();
        let nav_plugins: Vec<_> = plugins.iter().filter(|p| p.command == "nav").collect();
        // Both global and project entries are present
        assert_eq!(nav_plugins.len(), 2);
        // The global one should be marked as overridden
        let global_nav = nav_plugins.iter().find(|p| p.scope == PluginScope::Global).unwrap();
        assert!(global_nav.is_overridden);
        assert_eq!(global_nav.entry.version, "1.0.0");
        // The project one is the active override
        let project_nav = nav_plugins.iter().find(|p| p.scope == PluginScope::Project).unwrap();
        assert!(!project_nav.is_overridden);
        assert_eq!(project_nav.entry.version, "2.0.0");
    }

    #[test]
    fn test_all_plugins_no_override() {
        let mut global = PluginRegistry::new_global();
        global.register_plugin("nav", make_test_entry("nav", "1.0.0"));
        global.register_plugin("viz", make_test_entry("viz", "1.0.0"));

        let resolver = PluginResolver::with_registries(global, None, None);

        let plugins = resolver.all_plugins();
        assert_eq!(plugins.len(), 2);
        assert!(plugins.iter().all(|p| !p.is_overridden));
    }

    #[test]
    fn test_in_project_false_without_project() {
        let global = PluginRegistry::new_global();
        let resolver = PluginResolver::with_registries(global, None, None);
        assert!(!resolver.in_project());
    }

    #[test]
    fn test_in_project_true_with_project() {
        let global = PluginRegistry::new_global();
        let project = PluginRegistry::new_project("test");
        let resolver =
            PluginResolver::with_registries(global, Some(project), Some(PathBuf::from("/test")));
        assert!(resolver.in_project());
    }

    #[test]
    fn test_resolve_nonexistent_returns_none() {
        let global = PluginRegistry::new_global();
        let resolver = PluginResolver::with_registries(global, None, None);
        assert!(resolver.resolve("nonexistent").is_none());
    }

    #[test]
    fn test_verification_status_variants() {
        assert_ne!(VerificationStatus::Valid, VerificationStatus::ChecksumMismatch);
        assert_ne!(VerificationStatus::Valid, VerificationStatus::Error);
        assert_ne!(VerificationStatus::ChecksumMismatch, VerificationStatus::Error);
    }
}
