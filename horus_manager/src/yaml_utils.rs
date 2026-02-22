use anyhow::{anyhow, Result};
use std::fs;
use std::path::Path;

/// Add a dependency to horus.yaml
pub fn add_dependency_to_horus_yaml(
    horus_yaml_path: &Path,
    package_name: &str,
    version: &str,
) -> Result<()> {
    let content = fs::read_to_string(horus_yaml_path)?;
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
    let dep_prefix = format!("  - {}@", package_name);
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

    fs::write(horus_yaml_path, lines.join("\n") + "\n")?;
    Ok(())
}

/// Remove a dependency from horus.yaml
pub fn remove_dependency_from_horus_yaml(horus_yaml_path: &Path, package_name: &str) -> Result<()> {
    let content = fs::read_to_string(horus_yaml_path)?;
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

    fs::write(horus_yaml_path, new_lines.join("\n") + "\n")?;
    Ok(())
}

/// Add a path dependency to horus.yaml in structured format
pub fn add_path_dependency_to_horus_yaml(
    horus_yaml_path: &Path,
    package_name: &str,
    path: &str,
) -> Result<()> {
    let content = fs::read_to_string(horus_yaml_path)?;
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

    fs::write(horus_yaml_path, lines.join("\n") + "\n")?;
    Ok(())
}

/// Detect if a string looks like a path (contains /, ./, ../, or starts with /)
pub fn is_path_like(input: &str) -> bool {
    input.contains('/') || input.starts_with("./") || input.starts_with("../")
}

/// Read package name from a directory by checking horus.yaml, Cargo.toml, or package.json
pub fn read_package_name_from_path(path: &Path) -> Result<String> {
    // Try horus.yaml first
    let horus_yaml = path.join("horus.yaml");
    if horus_yaml.exists() {
        let content = fs::read_to_string(&horus_yaml)?;
        let yaml: serde_yaml::Value = serde_yaml::from_str(&content)?;
        if let Some(name) = yaml.get("name").and_then(|v| v.as_str()) {
            return Ok(name.to_string());
        }
    }

    // Try Cargo.toml (Rust)
    let cargo_toml = path.join("Cargo.toml");
    if cargo_toml.exists() {
        let content = fs::read_to_string(&cargo_toml)?;
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
        let content = fs::read_to_string(&package_json)?;
        let json: serde_json::Value = serde_json::from_str(&content)?;
        if let Some(name) = json.get("name").and_then(|v| v.as_str()) {
            return Ok(name.to_string());
        }
    }

    Err(anyhow!(
        "Could not determine package name from path: {}\nMake sure the directory contains horus.yaml, Cargo.toml, or package.json",
        path.display()
    ))
}
