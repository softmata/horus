use crate::config::HORUS_YAML;
use anyhow::{anyhow, Context, Result};
use serde::de::DeserializeOwned;
use std::fs;
use std::path::Path;

/// Load and deserialize a JSON config file.
pub fn load_json<T: DeserializeOwned>(path: &Path) -> Result<T> {
    log::debug!("loading config from {:?}", path);
    let content =
        fs::read_to_string(path).with_context(|| format!("failed to read {}", path.display()))?;
    serde_json::from_str(&content)
        .with_context(|| format!("failed to parse JSON from {}", path.display()))
}

/// Load and deserialize a YAML config file.
pub fn load_yaml<T: DeserializeOwned>(path: &Path) -> Result<T> {
    log::debug!("loading config from {:?}", path);
    let content =
        fs::read_to_string(path).with_context(|| format!("failed to read {}", path.display()))?;
    serde_yaml::from_str(&content)
        .with_context(|| format!("failed to parse YAML from {}", path.display()))
}

/// Add a dependency to horus.yaml
pub fn add_dependency_to_horus_yaml(
    horus_yaml_path: &Path,
    package_name: &str,
    version: &str,
) -> Result<()> {
    let content = fs::read_to_string(horus_yaml_path)
        .with_context(|| format!("failed to read {}", horus_yaml_path.display()))?;
    let mut lines: Vec<String> = content.lines().map(|s| s.to_string()).collect();

    // Find the dependencies section
    let mut deps_line_idx = None;
    for (i, line) in lines.iter().enumerate() {
        if line.trim().starts_with("dependencies:") {
            deps_line_idx = Some(i);
            break;
        }
    }

    // If no dependencies section exists, create one at the end
    let deps_idx = if let Some(idx) = deps_line_idx {
        idx
    } else {
        // Add empty line and dependencies section at the end
        if !lines.is_empty() && !lines.last().map(|l| l.is_empty()).unwrap_or(true) {
            lines.push(String::new());
        }
        lines.push("dependencies:".to_string());
        lines.len() - 1
    };

    // Check if it's an empty array: dependencies: []
    let deps_line = &lines[deps_idx];
    let is_empty_array = deps_line.trim() == "dependencies: []";

    let dependency_entry = format!("  - {}@{}", package_name, version);

    // Check for duplicates
    let dep_prefix = format!("- {}@", package_name);
    let already_exists = lines
        .iter()
        .any(|line| line.trim().starts_with(&dep_prefix) || line.trim() == dependency_entry.trim());

    if already_exists {
        println!("  Dependency {} already exists in horus.yaml", package_name);
        return Ok(());
    }

    if is_empty_array {
        // Convert empty array to list format
        lines[deps_idx] = "dependencies:".to_string();
        lines.insert(deps_idx + 1, dependency_entry);
    } else {
        // Find where to insert (after last dependency entry)
        let mut insert_idx = deps_idx + 1;
        while insert_idx < lines.len() {
            let line = &lines[insert_idx];
            if line.trim().starts_with("- ")
                || line.trim().is_empty()
                || line.trim().starts_with("#")
            {
                insert_idx += 1;
            } else {
                break;
            }
        }
        lines.insert(insert_idx, dependency_entry);
    }

    fs::write(horus_yaml_path, lines.join("\n") + "\n")
        .with_context(|| format!("failed to write {}", horus_yaml_path.display()))?;
    Ok(())
}

/// Remove a dependency from horus.yaml
pub fn remove_dependency_from_horus_yaml(horus_yaml_path: &Path, package_name: &str) -> Result<()> {
    let content = fs::read_to_string(horus_yaml_path)
        .with_context(|| format!("failed to read {}", horus_yaml_path.display()))?;
    let lines: Vec<String> = content.lines().map(|s| s.to_string()).collect();

    // Find and remove the dependency line
    // Match patterns like "- numpy@latest" or "- numpy" (without version)
    let dep_with_version = format!("- {}@", package_name);
    let dep_exact = format!("- {}", package_name);
    let mut new_lines: Vec<String> = lines
        .iter()
        .filter(|line| {
            let trimmed = line.trim();
            // Keep the line if it doesn't match our package
            !(trimmed.starts_with(&dep_with_version) || trimmed == dep_exact)
        })
        .cloned()
        .collect();

    // Check if dependencies section is now empty
    let mut deps_idx = None;
    for (i, line) in new_lines.iter().enumerate() {
        if line.trim().starts_with("dependencies:") {
            deps_idx = Some(i);
            break;
        }
    }

    if let Some(idx) = deps_idx {
        // Check if there are any dependency items after the "dependencies:" line
        let has_items = new_lines
            .iter()
            .skip(idx + 1)
            .take_while(|line| {
                let trimmed = line.trim();
                trimmed.starts_with("- ") || trimmed.is_empty() || trimmed.starts_with("#")
            })
            .any(|line| line.trim().starts_with("- "));

        if !has_items {
            // Convert back to empty array format
            new_lines[idx] = "dependencies: []".to_string();

            // Remove any empty lines or comments immediately after dependencies: []
            let mut final_lines = new_lines[..=idx].to_vec();

            // Skip empty lines and comments that were part of the dependencies section
            let mut i = idx + 1;
            while i < new_lines.len() {
                let line = &new_lines[i];
                let trimmed = line.trim();
                if trimmed.is_empty() || trimmed.starts_with("#") {
                    // Check if the next non-empty line is a new section
                    if let Some(next_line) = new_lines.get(i + 1) {
                        let next_trimmed = next_line.trim();
                        if !next_trimmed.is_empty() && !next_trimmed.starts_with("#") {
                            final_lines.push(line.clone());
                            break;
                        }
                    }
                } else {
                    final_lines.push(line.clone());
                }
                i += 1;
            }

            // Add remaining lines
            for line in new_lines.iter().skip(final_lines.len()) {
                final_lines.push(line.clone());
            }

            new_lines = final_lines;
        }
    }

    fs::write(horus_yaml_path, new_lines.join("\n") + "\n")
        .with_context(|| format!("failed to write {}", horus_yaml_path.display()))?;
    Ok(())
}

/// Add a path dependency to horus.yaml in structured format
pub fn add_path_dependency_to_horus_yaml(
    horus_yaml_path: &Path,
    package_name: &str,
    path: &str,
) -> Result<()> {
    let content = fs::read_to_string(horus_yaml_path)
        .with_context(|| format!("failed to read {}", horus_yaml_path.display()))?;
    let mut lines: Vec<String> = content.lines().map(|s| s.to_string()).collect();

    // Find the dependencies section
    let mut deps_line_idx = None;
    for (i, line) in lines.iter().enumerate() {
        if line.trim().starts_with("dependencies:") {
            deps_line_idx = Some(i);
            break;
        }
    }

    let deps_idx =
        deps_line_idx.ok_or_else(|| anyhow!("No dependencies section found in horus.yaml"))?;

    // Check if it's an empty array: dependencies: []
    let deps_line = &lines[deps_idx];
    let is_empty_array = deps_line.trim() == "dependencies: []";

    // Build path dependency in structured format
    let dependency_entry = vec![
        format!("  {}:", package_name),
        format!("    path: \"{}\"", path),
    ];

    // Check for duplicates
    let already_exists = lines
        .iter()
        .any(|line| line.trim().starts_with(&format!("{}:", package_name)));

    if already_exists {
        println!("  Dependency {} already exists in horus.yaml", package_name);
        return Ok(());
    }

    if is_empty_array {
        // Convert empty array to structured format
        lines[deps_idx] = "dependencies:".to_string();
        for entry in dependency_entry {
            lines.insert(deps_idx + 1, entry);
        }
    } else {
        // Find where to insert (after last dependency entry)
        let mut insert_idx = deps_idx + 1;
        while insert_idx < lines.len() {
            let line = &lines[insert_idx];
            let trimmed = line.trim();
            if trimmed.starts_with("- ")
                || trimmed.starts_with("#")
                || trimmed.is_empty()
                || (trimmed.ends_with(":") && !trimmed.starts_with("dependencies:"))
                || trimmed.starts_with("path:")
                || trimmed.starts_with("version:")
            {
                insert_idx += 1;
            } else {
                break;
            }
        }

        // Insert in reverse order to maintain correct sequence
        for entry in dependency_entry.iter().rev() {
            lines.insert(insert_idx, entry.clone());
        }
    }

    fs::write(horus_yaml_path, lines.join("\n") + "\n")
        .with_context(|| format!("failed to write {}", horus_yaml_path.display()))?;
    Ok(())
}

/// Detect if a string looks like a path (contains /, ./, ../, or starts with /)
pub fn is_path_like(input: &str) -> bool {
    input.contains('/') || input.starts_with("./") || input.starts_with("../")
}

/// Read package name from a directory by checking horus.yaml, Cargo.toml, or package.json
pub fn read_package_name_from_path(path: &Path) -> Result<String> {
    log::debug!("reading package name from path: {:?}", path);

    // Try horus.yaml first
    let horus_yaml = path.join(HORUS_YAML);
    if horus_yaml.exists() {
        log::debug!("trying horus.yaml at {:?}", horus_yaml);
        let content = fs::read_to_string(&horus_yaml)
            .with_context(|| format!("failed to read {}", horus_yaml.display()))?;
        let yaml: serde_yaml::Value = serde_yaml::from_str(&content)
            .with_context(|| format!("failed to parse {}", horus_yaml.display()))?;
        if let Some(name) = yaml.get("name").and_then(|v| v.as_str()) {
            return Ok(name.to_string());
        }
    }

    // Try Cargo.toml (Rust)
    let cargo_toml = path.join("Cargo.toml");
    if cargo_toml.exists() {
        log::debug!("trying Cargo.toml at {:?}", cargo_toml);
        let content = fs::read_to_string(&cargo_toml)
            .with_context(|| format!("failed to read {}", cargo_toml.display()))?;
        if let Ok(toml) = content.parse::<toml::Value>() {
            if let Some(name) = toml
                .get("package")
                .and_then(|p| p.get("name"))
                .and_then(|n| n.as_str())
            {
                return Ok(name.to_string());
            }
        }
    }

    // Try package.json (Python/Node)
    let package_json = path.join("package.json");
    if package_json.exists() {
        log::debug!("trying package.json at {:?}", package_json);
        let content = fs::read_to_string(&package_json)
            .with_context(|| format!("failed to read {}", package_json.display()))?;
        let json: serde_json::Value = serde_json::from_str(&content)
            .with_context(|| format!("failed to parse {}", package_json.display()))?;
        if let Some(name) = json.get("name").and_then(|v| v.as_str()) {
            return Ok(name.to_string());
        }
    }

    Err(anyhow!(
        "could not determine package name from {}\n  expected horus.yaml, Cargo.toml, or package.json",
        path.display()
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    // =====================
    // is_path_like Tests
    // =====================

    #[test]
    fn test_is_path_like_with_slash() {
        assert!(is_path_like("some/path"));
        assert!(is_path_like("/absolute/path"));
    }

    #[test]
    fn test_is_path_like_with_relative_prefix() {
        assert!(is_path_like("./relative"));
        assert!(is_path_like("../parent"));
    }

    #[test]
    fn test_is_path_like_plain_name() {
        assert!(!is_path_like("numpy"));
        assert!(!is_path_like("horus-ros2"));
        assert!(!is_path_like("my_package@1.0"));
    }

    // =====================
    // add_dependency_to_horus_yaml Tests
    // =====================

    #[test]
    fn test_add_dep_to_existing_deps() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\ndependencies:\n  - existing@1.0\n").unwrap();

        add_dependency_to_horus_yaml(&yaml, "numpy", "1.24").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(content.contains("  - numpy@1.24"));
        assert!(content.contains("  - existing@1.0"));
    }

    #[test]
    fn test_add_dep_to_empty_array() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\ndependencies: []\n").unwrap();

        add_dependency_to_horus_yaml(&yaml, "torch", "2.0").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(content.contains("dependencies:"));
        assert!(!content.contains("dependencies: []"));
        assert!(content.contains("  - torch@2.0"));
    }

    #[test]
    fn test_add_dep_creates_section() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\nversion: 0.1.0\n").unwrap();

        add_dependency_to_horus_yaml(&yaml, "numpy", "1.24").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(content.contains("dependencies:"));
        assert!(content.contains("  - numpy@1.24"));
    }

    #[test]
    fn test_add_dep_duplicate_is_noop() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\ndependencies:\n  - numpy@1.24\n").unwrap();

        add_dependency_to_horus_yaml(&yaml, "numpy", "1.24").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        // Should appear exactly once
        assert_eq!(content.matches("numpy@1.24").count(), 1);
    }

    #[test]
    fn test_add_dep_same_name_different_version_is_duplicate() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\ndependencies:\n  - numpy@1.24\n").unwrap();

        // Same package name with different version should be treated as duplicate
        add_dependency_to_horus_yaml(&yaml, "numpy", "2.0").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(content.contains("numpy@1.24"));
        assert_eq!(content.matches("numpy@").count(), 1);
    }

    // =====================
    // remove_dependency_from_horus_yaml Tests
    // =====================

    #[test]
    fn test_remove_dep_with_version() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(
            &yaml,
            "name: test-pkg\ndependencies:\n  - numpy@1.24\n  - torch@2.0\n",
        )
        .unwrap();

        remove_dependency_from_horus_yaml(&yaml, "numpy").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(!content.contains("numpy"));
        assert!(content.contains("  - torch@2.0"));
    }

    #[test]
    fn test_remove_last_dep_converts_to_empty_array() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\ndependencies:\n  - numpy@1.24\n").unwrap();

        remove_dependency_from_horus_yaml(&yaml, "numpy").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(!content.contains("numpy"));
        assert!(content.contains("dependencies: []"));
    }

    #[test]
    fn test_remove_dep_without_version() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(
            &yaml,
            "name: test-pkg\ndependencies:\n  - numpy\n  - torch@2.0\n",
        )
        .unwrap();

        remove_dependency_from_horus_yaml(&yaml, "numpy").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(!content.contains("numpy"));
        assert!(content.contains("torch@2.0"));
    }

    #[test]
    fn test_remove_nonexistent_dep_is_noop() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        let original = "name: test-pkg\ndependencies:\n  - torch@2.0\n";
        fs::write(&yaml, original).unwrap();

        remove_dependency_from_horus_yaml(&yaml, "nonexistent").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(content.contains("torch@2.0"));
    }

    // =====================
    // add_path_dependency_to_horus_yaml Tests
    // =====================

    #[test]
    fn test_add_path_dep() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\ndependencies:\n  - existing@1.0\n").unwrap();

        add_path_dependency_to_horus_yaml(&yaml, "my-lib", "../my-lib").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(content.contains("  my-lib:"));
        assert!(content.contains("    path: \"../my-lib\""));
    }

    #[test]
    fn test_add_path_dep_to_empty_array() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\ndependencies: []\n").unwrap();

        add_path_dependency_to_horus_yaml(&yaml, "my-lib", "./libs/my-lib").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        assert!(!content.contains("[]"));
        assert!(content.contains("  my-lib:"));
        assert!(content.contains("    path: \"./libs/my-lib\""));
    }

    #[test]
    fn test_add_path_dep_duplicate_is_noop() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(
            &yaml,
            "name: test-pkg\ndependencies:\n  my-lib:\n    path: \"../my-lib\"\n",
        )
        .unwrap();

        add_path_dependency_to_horus_yaml(&yaml, "my-lib", "../other-path").unwrap();

        let content = fs::read_to_string(&yaml).unwrap();
        // Original path preserved
        assert!(content.contains("\"../my-lib\""));
        assert_eq!(content.matches("my-lib:").count(), 1);
    }

    #[test]
    fn test_add_path_dep_no_deps_section_errors() {
        let dir = TempDir::new().unwrap();
        let yaml = dir.path().join(HORUS_YAML);
        fs::write(&yaml, "name: test-pkg\nversion: 0.1.0\n").unwrap();

        let result = add_path_dependency_to_horus_yaml(&yaml, "my-lib", "../my-lib");
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("No dependencies section"));
    }

    // =====================
    // read_package_name_from_path Tests
    // =====================

    #[test]
    fn test_read_name_from_horus_yaml() {
        let dir = TempDir::new().unwrap();
        fs::write(
            dir.path().join(HORUS_YAML),
            "name: my-robot-pkg\nversion: 0.1.0\n",
        )
        .unwrap();

        let name = read_package_name_from_path(dir.path()).unwrap();
        assert_eq!(name, "my-robot-pkg");
    }

    #[test]
    fn test_read_name_from_cargo_toml() {
        let dir = TempDir::new().unwrap();
        fs::write(
            dir.path().join("Cargo.toml"),
            "[package]\nname = \"my-rust-crate\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let name = read_package_name_from_path(dir.path()).unwrap();
        assert_eq!(name, "my-rust-crate");
    }

    #[test]
    fn test_read_name_from_package_json() {
        let dir = TempDir::new().unwrap();
        fs::write(
            dir.path().join("package.json"),
            "{\"name\": \"my-node-pkg\", \"version\": \"1.0.0\"}",
        )
        .unwrap();

        let name = read_package_name_from_path(dir.path()).unwrap();
        assert_eq!(name, "my-node-pkg");
    }

    #[test]
    fn test_read_name_horus_yaml_takes_priority() {
        let dir = TempDir::new().unwrap();
        fs::write(
            dir.path().join(HORUS_YAML),
            "name: horus-name\nversion: 0.1.0\n",
        )
        .unwrap();
        fs::write(
            dir.path().join("Cargo.toml"),
            "[package]\nname = \"cargo-name\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let name = read_package_name_from_path(dir.path()).unwrap();
        assert_eq!(name, "horus-name");
    }

    #[test]
    fn test_read_name_no_manifest_errors() {
        let dir = TempDir::new().unwrap();

        let result = read_package_name_from_path(dir.path());
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("could not determine package name"));
    }
}
