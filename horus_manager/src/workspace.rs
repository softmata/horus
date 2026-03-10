// Workspace tracking and detection for HORUS projects

use crate::manifest::HORUS_TOML;
use anyhow::{bail, Context, Result};
use chrono::{DateTime, Utc};
use colored::*;
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Workspace {
    pub name: String,
    pub path: PathBuf,
    pub created_at: DateTime<Utc>,
    pub last_used: DateTime<Utc>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct WorkspaceRegistry {
    pub workspaces: Vec<Workspace>,
}

impl WorkspaceRegistry {
    pub fn load() -> Result<Self> {
        let registry_path = Self::registry_path()?;

        if !registry_path.exists() {
            return Ok(Self {
                workspaces: Vec::new(),
            });
        }

        let content =
            fs::read_to_string(&registry_path).context("failed to read workspace registry")?;
        let registry: Self =
            serde_json::from_str(&content).context("failed to parse workspace registry")?;

        Ok(registry)
    }

    pub fn save(&self) -> Result<()> {
        let registry_path = Self::registry_path()?;
        let content =
            serde_json::to_string_pretty(self).context("failed to serialize workspace registry")?;
        fs::write(&registry_path, content).context("failed to write workspace registry")?;
        Ok(())
    }

    fn registry_path() -> Result<PathBuf> {
        let horus_dir = crate::paths::horus_dir()?;
        fs::create_dir_all(&horus_dir).context("failed to create ~/.horus directory")?;
        Ok(horus_dir.join("workspaces.json"))
    }

    pub fn add(&mut self, name: String, path: PathBuf) -> Result<()> {
        // Remove existing entry with same path
        self.workspaces.retain(|w| w.path != path);

        let workspace = Workspace {
            name,
            path,
            created_at: Utc::now(),
            last_used: Utc::now(),
        };

        self.workspaces.push(workspace);
        self.save()?;
        Ok(())
    }

    pub fn update_last_used(&mut self, path: &Path) -> Result<()> {
        if let Some(ws) = self.workspaces.iter_mut().find(|w| w.path == path) {
            ws.last_used = Utc::now();
            self.save()?;
        }
        Ok(())
    }

    pub fn find_by_name(&self, name: &str) -> Option<&Workspace> {
        self.workspaces.iter().find(|w| w.name == name)
    }
}

/// Find workspace root by searching upward for markers
pub fn find_workspace_root() -> Option<PathBuf> {
    let mut current = std::env::current_dir().ok()?;
    log::debug!("searching for workspace root from {:?}", current);

    // Search upwards for workspace markers (limit to 10 levels)
    for _ in 0..10 {
        // Priority 1: .horus/ directory (explicit HORUS workspace)
        if current.join(".horus").exists() {
            log::debug!("found workspace root: {:?}", current);
            return Some(current);
        }

        // Priority 2: horus.toml (workspace config)
        if current.join(HORUS_TOML).exists() {
            log::debug!("found workspace root: {:?}", current);
            return Some(current);
        }

        // Go up one level
        if let Some(parent) = current.parent() {
            current = parent.to_path_buf();
        } else {
            break;
        }
    }

    None
}

/// Detect current workspace or prompt user interactively
pub fn detect_or_select_workspace(allow_global: bool) -> Result<InstallTarget> {
    // Try to find current workspace
    if let Some(root) = find_workspace_root() {
        // Update last used time
        let mut registry = WorkspaceRegistry::load()?;
        registry.update_last_used(&root).ok();

        let name = root
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .to_string();

        println!("  {} Detected workspace: {}", "".green(), name.yellow());
        return Ok(InstallTarget::Local(root));
    }

    // Not in a workspace - show interactive selector
    let registry = WorkspaceRegistry::load()?;

    if registry.workspaces.is_empty() && !allow_global {
        bail!("No HORUS workspaces found. Run 'horus new <name>' or 'horus init' first.");
    }

    interactive_workspace_selector(&registry, allow_global)
}

#[derive(Debug)]
pub enum InstallTarget {
    Local(PathBuf),
    Global,
}

fn interactive_workspace_selector(
    registry: &WorkspaceRegistry,
    allow_global: bool,
) -> Result<InstallTarget> {
    println!("\n{}", "ℹ Not in a HORUS workspace".cyan());
    println!("Where should we install the package?\n");

    let mut options = Vec::new();
    let mut idx = 1;

    // List known workspaces
    for ws in &registry.workspaces {
        println!(
            "  [{}] {}  ({})",
            idx.to_string().cyan(),
            ws.name.yellow(),
            ws.path.display().to_string().dimmed()
        );
        options.push(InstallTarget::Local(ws.path.clone()));
        idx += 1;
    }

    // Global option
    if allow_global {
        println!(
            "  [{}]  Global  (~/.horus/cache) - shared across projects",
            idx.to_string().cyan()
        );
        options.push(InstallTarget::Global);
        idx += 1;
    }

    // Create new workspace option
    let current = std::env::current_dir()?;
    let new_workspace_idx = idx;
    println!(
        "  [{}] [+] Create new workspace here  ({})",
        idx.to_string().cyan(),
        current.display().to_string().dimmed()
    );
    idx += 1;

    // Cancel option
    println!("  [{}]  Cancel\n", idx.to_string().cyan());

    // Get user selection
    print!(" Select target [1-{}]: ", idx);
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    let selection: usize = input.trim().parse().context("Invalid selection")?;

    if selection < 1 || selection > idx {
        bail!("Invalid selection");
    }

    if selection == idx {
        bail!("Installation cancelled");
    }

    // Handle "create new workspace" option
    if selection == new_workspace_idx {
        println!(
            "\n{} Creating new workspace in current directory...",
            "".cyan()
        );

        // Ask for workspace name
        let default_name = current
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("workspace");

        print!("Workspace name [{}]: ", default_name.yellow());
        io::stdout().flush()?;

        let mut name_input = String::new();
        io::stdin().read_line(&mut name_input)?;
        let workspace_name = name_input.trim();
        let workspace_name = if workspace_name.is_empty() {
            default_name.to_string()
        } else {
            workspace_name.to_string()
        };

        // Create .horus/ directory
        let horus_dir = current.join(".horus");
        fs::create_dir_all(&horus_dir).context("Failed to create .horus directory")?;

        // Create minimal horus.toml
        let horus_toml = current.join(HORUS_TOML);
        let toml_content = format!(
            "[package]\nname = \"{}\"\nversion = \"0.1.9\"\n",
            workspace_name
        );
        fs::write(&horus_toml, toml_content).context("Failed to create horus.toml")?;

        // Register in workspace registry
        let mut registry = WorkspaceRegistry::load()?;
        registry.add(workspace_name.clone(), current.clone())?;

        println!(" Created workspace: {}", workspace_name.yellow());
        println!("   Location: {}", current.display());

        return Ok(InstallTarget::Local(current));
    }

    Ok(options[selection - 1].clone())
}

impl Clone for InstallTarget {
    fn clone(&self) -> Self {
        match self {
            InstallTarget::Local(path) => InstallTarget::Local(path.clone()),
            InstallTarget::Global => InstallTarget::Global,
        }
    }
}

/// Register current directory as a workspace
pub fn register_current_workspace(name: Option<String>) -> Result<()> {
    let current = std::env::current_dir()?;

    let workspace_name = if let Some(n) = name {
        n
    } else {
        current
            .file_name()
            .and_then(|s| s.to_str())
            .context("Invalid directory name")?
            .to_string()
    };

    // Create .horus/ directory
    let horus_dir = current.join(".horus");
    if !horus_dir.exists() {
        fs::create_dir_all(&horus_dir)?;
        println!("  {} Created .horus/ directory", "".green());
    }

    // Create minimal horus.toml if it doesn't exist
    let horus_toml = current.join(HORUS_TOML);
    if !horus_toml.exists() {
        let toml_content = format!(
            "[package]\nname = \"{}\"\nversion = \"0.1.9\"\n",
            workspace_name
        );
        fs::write(&horus_toml, toml_content)?;
        println!("  {} Created horus.toml", "".green());
    }

    // Register in workspace registry
    let mut registry = WorkspaceRegistry::load()?;
    registry.add(workspace_name.clone(), current.clone())?;

    println!(" Initialized HORUS workspace: {}", workspace_name.yellow());
    println!("   Location: {}", current.display());
    Ok(())
}

/// Discovered workspace information for monitor
#[derive(Debug, Clone)]
pub struct DiscoveredWorkspace {
    pub name: String,
    pub path: PathBuf,
    pub is_current: bool,
}

/// Unified workspace discovery for both TUI and web monitor
/// Returns all workspaces that should be visible, combining:
/// 1. Registered workspaces from ~/.horus/workspaces.json
/// 2. Nested workspaces within the current workspace (if running from one)
pub fn discover_all_workspaces(current_workspace: &Option<PathBuf>) -> Vec<DiscoveredWorkspace> {
    use std::collections::HashSet;

    let mut discovered = Vec::new();
    let mut seen_canonical: HashSet<PathBuf> = HashSet::new();

    // Helper to add workspace if not already seen
    let mut add_workspace = |path: PathBuf, name: String, is_current: bool| {
        // Canonicalize to handle symlinks
        let canonical = path.canonicalize().unwrap_or_else(|_| path.clone());

        if !seen_canonical.contains(&canonical) {
            seen_canonical.insert(canonical);
            discovered.push(DiscoveredWorkspace {
                name,
                path,
                is_current,
            });
        }
    };

    // 1. Get registered workspaces from registry
    if let Ok(registry) = WorkspaceRegistry::load() {
        for ws in &registry.workspaces {
            let horus_dir = ws.path.join(".horus");

            // Verify workspace still exists
            if horus_dir.exists() && horus_dir.is_dir() {
                let is_current = current_workspace
                    .as_ref()
                    .map(|p| p == &ws.path)
                    .unwrap_or(false);

                add_workspace(ws.path.clone(), ws.name.clone(), is_current);
            }
        }
    }

    // 2. If we have a current workspace, add it and scan for nested .horus/ directories
    if let Some(current_path) = current_workspace {
        // Add the current workspace itself
        let workspace_name = current_path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("workspace")
            .to_string();
        add_workspace(current_path.clone(), workspace_name, true);

        // Then scan for nested workspaces
        scan_for_nested_workspaces(current_path, &mut add_workspace, current_path);
    }

    discovered
}

/// Recursively scan for nested .horus/ directories within a workspace
/// Limited to depth 3 to avoid excessive scanning
fn scan_for_nested_workspaces<F>(base_path: &Path, add_workspace: &mut F, current_workspace: &Path)
where
    F: FnMut(PathBuf, String, bool),
{
    fn scan_recursive<F>(path: &Path, depth: usize, add_workspace: &mut F, current_workspace: &Path)
    where
        F: FnMut(PathBuf, String, bool),
    {
        const MAX_DEPTH: usize = 3;

        if depth > MAX_DEPTH {
            return;
        }

        if let Ok(entries) = fs::read_dir(path) {
            for entry in entries.flatten() {
                let entry_path = entry.path();

                // Skip hidden directories (except .horus), build dirs, etc.
                if let Some(name) = entry_path.file_name() {
                    let name_str = name.to_string_lossy();
                    if name_str.starts_with('.') && name_str != ".horus" {
                        continue;
                    }
                    if name_str == "target"
                        || name_str == "node_modules"
                        || name_str == ".git"
                        || name_str == "build"
                    {
                        continue;
                    }
                }

                if entry_path.is_dir() {
                    let horus_dir = entry_path.join(".horus");

                    if horus_dir.exists() && horus_dir.is_dir() {
                        // Found a workspace - add it
                        let workspace_name = entry_path
                            .file_name()
                            .and_then(|s| s.to_str())
                            .unwrap_or("unknown")
                            .to_string();

                        let is_current = entry_path == current_workspace;
                        add_workspace(entry_path.clone(), workspace_name, is_current);

                        // Don't recurse into workspaces - they're self-contained
                        continue;
                    }

                    // Recurse into regular directories
                    scan_recursive(&entry_path, depth + 1, add_workspace, current_workspace);
                }
            }
        }
    }

    scan_recursive(base_path, 0, add_workspace, current_workspace);
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    // ========================================================================
    // WorkspaceRegistry tests
    // ========================================================================

    #[test]
    fn workspace_registry_empty_on_missing_file() {
        // If registry file doesn't exist, load returns empty
        let registry = WorkspaceRegistry {
            workspaces: Vec::new(),
        };
        assert!(registry.workspaces.is_empty());
    }

    #[test]
    fn workspace_registry_serde_roundtrip() {
        let ws = Workspace {
            name: "test_ws".to_string(),
            path: PathBuf::from("/tmp/test_ws"),
            created_at: Utc::now(),
            last_used: Utc::now(),
        };
        let registry = WorkspaceRegistry {
            workspaces: vec![ws],
        };

        let json = serde_json::to_string(&registry).unwrap();
        let loaded: WorkspaceRegistry = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.workspaces.len(), 1);
        assert_eq!(loaded.workspaces[0].name, "test_ws");
        assert_eq!(loaded.workspaces[0].path, PathBuf::from("/tmp/test_ws"));
    }

    #[test]
    fn workspace_registry_find_by_name() {
        let registry = WorkspaceRegistry {
            workspaces: vec![
                Workspace {
                    name: "alpha".to_string(),
                    path: PathBuf::from("/tmp/alpha"),
                    created_at: Utc::now(),
                    last_used: Utc::now(),
                },
                Workspace {
                    name: "beta".to_string(),
                    path: PathBuf::from("/tmp/beta"),
                    created_at: Utc::now(),
                    last_used: Utc::now(),
                },
            ],
        };

        assert!(registry.find_by_name("alpha").is_some());
        assert_eq!(
            registry.find_by_name("alpha").unwrap().path,
            PathBuf::from("/tmp/alpha")
        );
        assert!(registry.find_by_name("beta").is_some());
        assert!(registry.find_by_name("gamma").is_none());
    }

    #[test]
    fn workspace_registry_add_deduplicates_by_path() {
        let mut registry = WorkspaceRegistry {
            workspaces: Vec::new(),
        };

        // Simulate add without save (avoid touching real filesystem)
        let path = PathBuf::from("/tmp/dedup_test");
        registry.workspaces.push(Workspace {
            name: "first_name".to_string(),
            path: path.clone(),
            created_at: Utc::now(),
            last_used: Utc::now(),
        });

        // Add again with different name but same path
        registry.workspaces.retain(|w| w.path != path);
        registry.workspaces.push(Workspace {
            name: "second_name".to_string(),
            path: path.clone(),
            created_at: Utc::now(),
            last_used: Utc::now(),
        });

        assert_eq!(registry.workspaces.len(), 1);
        assert_eq!(registry.workspaces[0].name, "second_name");
    }

    // ========================================================================
    // find_workspace_root tests
    // ========================================================================

    #[test]
    fn find_workspace_root_with_horus_dir() {
        let tmp = TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // Save and set current dir
        let original_dir = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = find_workspace_root();
        assert!(result.is_some());
        assert_eq!(result.unwrap(), tmp.path());

        // Restore
        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn find_workspace_root_with_horus_toml() {
        let tmp = TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let original_dir = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = find_workspace_root();
        assert!(result.is_some());
        assert_eq!(result.unwrap(), tmp.path());

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn find_workspace_root_from_subdirectory() {
        let tmp = TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let sub_dir = tmp.path().join("src/nodes");
        fs::create_dir_all(&sub_dir).unwrap();

        let original_dir = std::env::current_dir().unwrap();
        std::env::set_current_dir(&sub_dir).unwrap();

        let result = find_workspace_root();
        assert!(result.is_some());
        assert_eq!(result.unwrap(), tmp.path());

        std::env::set_current_dir(original_dir).unwrap();
    }

    // ========================================================================
    // InstallTarget tests
    // ========================================================================

    #[test]
    fn install_target_clone() {
        let local = InstallTarget::Local(PathBuf::from("/tmp/test"));
        let cloned = local.clone();
        match cloned {
            InstallTarget::Local(p) => assert_eq!(p, PathBuf::from("/tmp/test")),
            _ => panic!("Expected Local"),
        }

        let global = InstallTarget::Global;
        let cloned = global.clone();
        assert!(
            matches!(cloned, InstallTarget::Global),
            "Expected Global, got {:?}",
            cloned
        );
    }

    #[test]
    fn install_target_debug() {
        let local = InstallTarget::Local(PathBuf::from("/tmp/test"));
        let debug = format!("{:?}", local);
        assert!(
            debug.contains("Local"),
            "Debug of Local variant should contain 'Local', got: {}",
            debug
        );
        assert!(
            debug.contains("/tmp/test"),
            "Debug of Local variant should contain path, got: {}",
            debug
        );

        let global = InstallTarget::Global;
        let debug = format!("{:?}", global);
        assert_eq!(
            debug, "Global",
            "Debug of Global variant should be exactly 'Global', got: {}",
            debug
        );
    }

    // ========================================================================
    // discover_all_workspaces tests
    // ========================================================================

    #[test]
    fn discover_no_workspaces() {
        // With no current workspace, returns only globally-registered workspaces (if any)
        let result = discover_all_workspaces(&None);
        // Every discovered workspace must have valid fields
        for ws in &result {
            assert!(!ws.name.is_empty(), "Workspace name must not be empty");
            assert!(
                !ws.path.as_os_str().is_empty(),
                "Workspace path must not be empty"
            );
        }
        // None of the discovered workspaces should be marked as current
        // since we passed &None as current_workspace
        for ws in &result {
            assert!(
                !ws.is_current,
                "No workspace should be marked current when current_workspace is None, but '{}' is",
                ws.name
            );
        }
    }

    #[test]
    fn discover_current_workspace() {
        let tmp = TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let current = Some(tmp.path().to_path_buf());
        let result = discover_all_workspaces(&current);

        // Should find at least the current workspace
        assert!(!result.is_empty());
        assert!(result.iter().any(|w| w.is_current));
    }

    #[test]
    fn discover_nested_workspaces() {
        let tmp = TempDir::new().unwrap();
        // Create parent workspace
        let parent_horus = tmp.path().join(".horus");
        fs::create_dir_all(&parent_horus).unwrap();

        // Create nested workspaces
        let nested1 = tmp.path().join("robot_a");
        fs::create_dir_all(nested1.join(".horus")).unwrap();

        let nested2 = tmp.path().join("robot_b");
        fs::create_dir_all(nested2.join(".horus")).unwrap();

        let current = Some(tmp.path().to_path_buf());
        let result = discover_all_workspaces(&current);

        // Should find parent + both nested
        assert!(result.len() >= 3);
        let names: Vec<&str> = result.iter().map(|w| w.name.as_str()).collect();
        assert!(names.contains(&"robot_a"));
        assert!(names.contains(&"robot_b"));
    }

    #[test]
    fn discover_skips_hidden_and_build_dirs() {
        let tmp = TempDir::new().unwrap();
        let parent_horus = tmp.path().join(".horus");
        fs::create_dir_all(&parent_horus).unwrap();

        // Create dirs that should be skipped
        fs::create_dir_all(tmp.path().join(".git")).unwrap();
        fs::create_dir_all(tmp.path().join("target/.horus")).unwrap();
        fs::create_dir_all(tmp.path().join("node_modules/.horus")).unwrap();

        // Create a valid nested workspace
        fs::create_dir_all(tmp.path().join("real_ws/.horus")).unwrap();

        let current = Some(tmp.path().to_path_buf());
        let result = discover_all_workspaces(&current);

        let names: Vec<&str> = result.iter().map(|w| w.name.as_str()).collect();
        assert!(names.contains(&"real_ws"));
        assert!(!names.contains(&"target"));
        assert!(!names.contains(&"node_modules"));
    }

    #[test]
    fn discover_deduplicates_by_canonical_path() {
        let tmp = TempDir::new().unwrap();
        let ws_dir = tmp.path().join("workspace");
        fs::create_dir_all(ws_dir.join(".horus")).unwrap();

        // Use the same path twice as current workspace
        let current = Some(ws_dir.clone());
        let result = discover_all_workspaces(&current);

        // Count how many times "workspace" appears
        let ws_count = result.iter().filter(|w| w.name == "workspace").count();
        assert_eq!(ws_count, 1, "Should deduplicate identical paths");
    }

    // ========================================================================
    // DiscoveredWorkspace tests
    // ========================================================================

    #[test]
    fn discovered_workspace_fields() {
        let dw = DiscoveredWorkspace {
            name: "my_robot".to_string(),
            path: PathBuf::from("/home/user/my_robot"),
            is_current: true,
        };
        assert_eq!(dw.name, "my_robot");
        assert_eq!(dw.path, PathBuf::from("/home/user/my_robot"));
        assert!(dw.is_current);
        let cloned = dw.clone();
        assert_eq!(cloned.name, "my_robot");
        assert_eq!(cloned.path, dw.path);
        assert_eq!(cloned.is_current, dw.is_current);
        let debug = format!("{:?}", dw);
        assert!(
            debug.contains("my_robot"),
            "Debug output should contain name, got: {}",
            debug
        );
        assert!(
            debug.contains("is_current"),
            "Debug output should contain is_current field, got: {}",
            debug
        );
    }
}
