use super::*;

// Helper functions for system detection
pub(crate) fn get_python_version() -> Option<String> {
    std::process::Command::new("python3")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout)
                .ok()
                .map(|s| s.trim().replace("Python ", ""))
        })
}

pub(crate) fn get_rust_version() -> Option<String> {
    std::process::Command::new("rustc")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout)
                .ok()
                .and_then(|s| s.split_whitespace().nth(1).map(|v| v.to_string()))
        })
}

pub(crate) fn get_gcc_version() -> Option<String> {
    std::process::Command::new("gcc")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout).ok().and_then(|s| {
                s.lines()
                    .next()
                    .and_then(|line| line.split_whitespace().last())
                    .map(|v| v.to_string())
            })
        })
}

pub(crate) fn get_cuda_version() -> Option<String> {
    // Try nvcc first (CUDA compiler)
    if let Some(version) = std::process::Command::new("nvcc")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout).ok().and_then(|s| {
                // Parse output like: "Cuda compilation tools, release 11.8, V11.8.89"
                // Look for "release X.Y" pattern
                s.lines()
                    .find(|line| line.contains("release"))
                    .and_then(|line| {
                        line.split("release")
                            .nth(1)
                            .and_then(|part| part.split(',').next())
                            .map(|v| v.trim().to_string())
                    })
            })
        })
    {
        return Some(version);
    }

    // Fallback to nvidia-smi
    std::process::Command::new("nvidia-smi")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout).ok().and_then(|s| {
                // Parse output to find CUDA Version line
                // Format: "CUDA Version: 12.0"
                s.lines()
                    .find(|line| line.contains("CUDA Version"))
                    .and_then(|line| {
                        line.split("CUDA Version:")
                            .nth(1)
                            .map(|v| v.split_whitespace().next().unwrap_or("").to_string())
                    })
            })
        })
}

// Check if any version of a package exists in global cache
pub(crate) fn check_global_versions(cache_dir: &Path, package_name: &str) -> Result<bool> {
    if !cache_dir.exists() {
        return Ok(false);
    }

    for entry in fs::read_dir(cache_dir)? {
        let entry = entry?;
        let name = entry.file_name();
        let name_str = name.to_string_lossy();

        // Match package@version pattern
        if name_str == package_name || name_str.starts_with(&format!("{}@", package_name)) {
            return Ok(true);
        }
    }

    Ok(false)
}

/// Add required features to the project's Cargo.toml for horus_library
pub(crate) fn add_features_to_cargo_toml(workspace_path: &Path, features: &[String]) -> Result<()> {
    use toml_edit::{DocumentMut, Item, Value};

    let cargo_toml_path = workspace_path.join(CARGO_TOML);
    if !cargo_toml_path.exists() {
        return Err(anyhow!("Cargo.toml not found in workspace"));
    }

    let content = fs::read_to_string(&cargo_toml_path)?;
    let mut doc = content
        .parse::<DocumentMut>()
        .map_err(|e| anyhow!("failed to parse Cargo.toml: {}", e))?;

    // Look for horus_library in dependencies
    let deps = doc
        .get_mut("dependencies")
        .ok_or_else(|| anyhow!("No [dependencies] section"))?;

    if let Some(horus_lib) = deps.get_mut("horus_library") {
        // Get existing features or create empty array
        match horus_lib {
            Item::Value(Value::String(_)) => {
                // Convert simple version string to table with features
                let version = horus_lib.as_str().unwrap_or("*").to_string();
                let mut table = toml_edit::InlineTable::new();
                table.insert("version", version.into());
                let mut features_array = toml_edit::Array::new();
                for f in features {
                    features_array.push(f.as_str());
                }
                table.insert("features", Value::Array(features_array));
                *horus_lib = Item::Value(Value::InlineTable(table));
            }
            Item::Value(Value::InlineTable(table)) => {
                // Add features to existing inline table
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", Value::Array(new_features));
            }
            Item::Table(table) => {
                // Add features to existing table
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", toml_edit::value(Value::Array(new_features)));
            }
            _ => {
                return Err(anyhow!("Unexpected horus_library format in Cargo.toml"));
            }
        }
    } else if let Some(horus) = deps.get_mut("horus") {
        // Also check for "horus" dependency (the unified crate)
        match horus {
            Item::Value(Value::String(_)) => {
                let version = horus.as_str().unwrap_or("*").to_string();
                let mut table = toml_edit::InlineTable::new();
                table.insert("version", version.into());
                let mut features_array = toml_edit::Array::new();
                for f in features {
                    features_array.push(f.as_str());
                }
                table.insert("features", Value::Array(features_array));
                *horus = Item::Value(Value::InlineTable(table));
            }
            Item::Value(Value::InlineTable(table)) => {
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", Value::Array(new_features));
            }
            Item::Table(table) => {
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", toml_edit::value(Value::Array(new_features)));
            }
            _ => {}
        }
    } else {
        // No horus or horus_library dependency found - skip
        return Ok(());
    }

    // Write back to file
    fs::write(&cargo_toml_path, doc.to_string())?;
    Ok(())
}

/// Detect the system package manager
pub(crate) fn detect_package_manager() -> Option<(&'static str, &'static str, bool)> {
    // Returns (command, install_subcommand, needs_sudo)
    use std::process::Command;

    // Check for apt (Debian/Ubuntu)
    if Command::new("apt").arg("--version").output().is_ok() {
        return Some(("apt", "install -y", true));
    }
    // Check for dnf (Fedora)
    if Command::new("dnf").arg("--version").output().is_ok() {
        return Some(("dnf", "install -y", true));
    }
    // Check for pacman (Arch)
    if Command::new("pacman").arg("--version").output().is_ok() {
        return Some(("pacman", "-S --noconfirm", true));
    }
    // Check for brew (macOS)
    if Command::new("brew").arg("--version").output().is_ok() {
        return Some(("brew", "install", false));
    }
    // Check for apk (Alpine)
    if Command::new("apk").arg("--version").output().is_ok() {
        return Some(("apk", "add", true));
    }
    None
}

/// Check if a system package is installed
pub(crate) fn is_system_package_installed(pkg: &str, pkg_manager: &str) -> bool {
    use std::process::Command;

    let result = match pkg_manager {
        "apt" => Command::new("dpkg").args(["-s", pkg]).output(),
        "dnf" => Command::new("rpm").args(["-q", pkg]).output(),
        "pacman" => Command::new("pacman").args(["-Q", pkg]).output(),
        "brew" => Command::new("brew").args(["list", pkg]).output(),
        "apk" => Command::new("apk").args(["info", "-e", pkg]).output(),
        _ => return false,
    };

    result.map(|o| o.status.success()).unwrap_or(false)
}

/// Handle system dependencies with smart detection and optional installation
pub(crate) fn handle_system_dependencies(deps: &[String]) -> Result<()> {
    use colored::*;
    use std::io::{self, Write};
    use std::process::Command;

    let Some((pkg_manager, install_cmd, needs_sudo)) = detect_package_manager() else {
        // No known package manager, just notify
        println!("  {} System packages required:", "[PKG]".cyan());
        for dep in deps {
            println!("    • {}", dep);
        }
        println!("    Please install these packages manually");
        return Ok(());
    };

    // Check which packages are missing
    let missing: Vec<&String> = deps
        .iter()
        .filter(|pkg| !is_system_package_installed(pkg, pkg_manager))
        .collect();

    if missing.is_empty() {
        println!("  {} All system packages already installed", crate::cli_output::ICON_SUCCESS.green());
        return Ok(());
    }

    println!(
        "  {} System packages required ({}):",
        "[PKG]".cyan(),
        pkg_manager
    );
    for dep in deps {
        let status = if is_system_package_installed(dep, pkg_manager) {
            format!("{}", crate::cli_output::ICON_SUCCESS.green())
        } else {
            format!("{}", "missing".yellow())
        };
        println!("    • {} [{}]", dep, status);
    }

    // Build install command
    let install_args: Vec<&str> = install_cmd.split_whitespace().collect();
    let full_cmd = if needs_sudo {
        format!(
            "sudo {} {} {}",
            pkg_manager,
            install_cmd,
            missing
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join(" ")
        )
    } else {
        format!(
            "{} {} {}",
            pkg_manager,
            install_cmd,
            missing
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join(" ")
        )
    };

    // Prompt user
    print!("  {} Install missing packages? [Y/n]: ", "?".cyan());
    io::stdout().flush().ok();

    let mut input = String::new();
    if io::stdin().read_line(&mut input).is_ok() {
        let input = input.trim().to_lowercase();
        if input.is_empty() || input == "y" || input == "yes" {
            println!("  {} Running: {}", "->".cyan(), full_cmd);

            let status = if needs_sudo {
                Command::new("sudo")
                    .arg(pkg_manager)
                    .args(&install_args)
                    .args(missing.iter().map(|s| s.as_str()))
                    .status()
            } else {
                Command::new(pkg_manager)
                    .args(&install_args)
                    .args(missing.iter().map(|s| s.as_str()))
                    .status()
            };

            match status {
                Ok(s) if s.success() => {
                    println!("  {} System packages installed", crate::cli_output::ICON_SUCCESS.green());
                }
                Ok(_) => {
                    println!(
                        "  {} Installation failed. Run manually: {}",
                        crate::cli_output::ICON_WARN.yellow(),
                        full_cmd
                    );
                }
                Err(e) => {
                    println!("  {} Could not run installer: {}", crate::cli_output::ICON_WARN.yellow(), e);
                    println!("    Run manually: {}", full_cmd);
                }
            }
        } else {
            println!("  {} Skipped. Run manually: {}", "->".cyan(), full_cmd);
        }
    }

    Ok(())
}

/// Add cargo dependencies to Cargo.toml
/// Dependencies are in format "crate_name@version" e.g., "serialport@4.2"
pub(crate) fn add_cargo_deps_to_cargo_toml(workspace_path: &Path, deps: &[String]) -> Result<()> {
    use toml_edit::DocumentMut;

    let cargo_toml_path = workspace_path.join(CARGO_TOML);
    if !cargo_toml_path.exists() {
        return Err(anyhow!("Cargo.toml not found in workspace"));
    }

    let content = fs::read_to_string(&cargo_toml_path)?;
    let mut doc = content
        .parse::<DocumentMut>()
        .map_err(|e| anyhow!("failed to parse Cargo.toml: {}", e))?;

    // Ensure dependencies section exists
    if doc.get("dependencies").is_none() {
        doc["dependencies"] = toml_edit::table();
    }
    let deps_section = doc
        .get_mut("dependencies")
        .ok_or_else(|| anyhow!("No [dependencies] section"))?;

    for dep_spec in deps {
        // Parse "crate_name@version" or just "crate_name"
        let (name, version) = if let Some((n, v)) = dep_spec.split_once('@') {
            (n.trim(), Some(v.trim()))
        } else {
            (dep_spec.trim(), None)
        };

        // Skip if already present
        if deps_section.get(name).is_some() {
            continue;
        }

        // Add the dependency
        let version_str = version.unwrap_or("*");
        deps_section[name] = toml_edit::value(version_str);
    }

    // Write back to file
    fs::write(&cargo_toml_path, doc.to_string())?;
    Ok(())
}

// Copy directory recursively
pub(crate) fn copy_dir_all(src: &Path, dst: &Path) -> Result<()> {
    fs::create_dir_all(dst)?;

    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());

        if ty.is_dir() {
            copy_dir_all(&src_path, &dst_path)?;
        } else {
            fs::copy(&src_path, &dst_path)?;
        }
    }

    Ok(())
}

// Helper function to detect package version from directory
pub(crate) fn detect_package_version(dir: &Path) -> Option<String> {
    // Try horus.yaml first (primary HORUS manifest)
    let horus_yaml = dir.join(HORUS_YAML);
    if horus_yaml.exists() {
        if let Ok(content) = fs::read_to_string(&horus_yaml) {
            // Simple YAML parsing for version
            for line in content.lines() {
                let trimmed = line.trim();
                if trimmed.starts_with("version:") {
                    let version = trimmed.trim_start_matches("version:").trim().to_string();
                    return Some(version);
                }
            }
        }
    }

    // Try package.json
    let package_json = dir.join("package.json");
    if package_json.exists() {
        if let Ok(content) = fs::read_to_string(&package_json) {
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&content) {
                if let Some(version) = json.get("version").and_then(|v| v.as_str()) {
                    return Some(version.to_string());
                }
            }
        }
    }

    // Try Cargo.toml
    let cargo_toml = dir.join(CARGO_TOML);
    if cargo_toml.exists() {
        if let Ok(content) = fs::read_to_string(&cargo_toml) {
            if let Ok(toml) = toml::from_str::<toml::Value>(&content) {
                if let Some(package) = toml.get("package") {
                    if let Some(version) = package.get("version").and_then(|v| v.as_str()) {
                        return Some(version.to_string());
                    }
                }
            }
        }
    }

    None
}

// Detect installed version from PyPI package .dist-info directory
pub(crate) fn detect_pypi_installed_version(pkg_dir: &Path, package_name: &str) -> Option<String> {
    // PyPI packages create .dist-info directories with the format: package_name-version.dist-info
    // We need to find this directory and extract the version

    if let Ok(entries) = fs::read_dir(pkg_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if let Some(dir_name) = path.file_name().and_then(|n| n.to_str()) {
                    // Look for .dist-info directories
                    if dir_name.ends_with(".dist-info") {
                        // Extract package name and version from directory name
                        // Format: package_name-version.dist-info
                        let without_suffix = dir_name.trim_end_matches(".dist-info");

                        // Handle package name normalization (PyPI normalizes names)
                        let normalized_pkg = package_name
                            .replace("-", "_")
                            .replace(".", "_")
                            .to_lowercase();
                        let normalized_dir = without_suffix
                            .replace("-", "_")
                            .replace(".", "_")
                            .to_lowercase();

                        if normalized_dir.starts_with(&normalized_pkg) {
                            // Try to extract version after package name
                            // Format is usually: package_name-version
                            let parts: Vec<&str> = without_suffix.rsplitn(2, '-').collect();
                            if parts.len() == 2 {
                                return Some(parts[0].to_string());
                            }
                        }
                    }
                }
            }
        }
    }

    None
}

// Detect installed version from cargo binary
pub(crate) fn detect_cargo_installed_version(pkg_dir: &Path, package_name: &str) -> Option<String> {
    // Cargo install creates binaries in bin/ subdirectory
    // Try to run the binary with --version flag

    let bin_path = pkg_dir.join("bin").join(package_name);

    if bin_path.exists() {
        use std::process::Command;

        if let Ok(output) = Command::new(&bin_path).arg("--version").output() {
            if output.status.success() {
                let version_output = String::from_utf8_lossy(&output.stdout);
                // Parse version from output (usually: "package_name version")
                let parts: Vec<&str> = version_output.split_whitespace().collect();
                if parts.len() >= 2 {
                    return Some(parts[1].to_string());
                }
            }
        }
    }

    None
}

#[derive(Debug)]
pub(crate) enum ManifestFormat {
    HorusYaml,
    CargoToml,
    PackageJson,
}

impl std::fmt::Display for ManifestFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ManifestFormat::HorusYaml => write!(f, "{}", HORUS_YAML),
            ManifestFormat::CargoToml => write!(f, "{}", CARGO_TOML),
            ManifestFormat::PackageJson => write!(f, "package.json"),
        }
    }
}

#[derive(Debug)]
pub(crate) struct PackageManifest {
    pub name: String,
    pub version: String,
    pub description: Option<String>,
    pub license: Option<String>,
    pub package_type: Option<String>,
    pub categories: Option<String>,
    pub source_url: Option<String>,
    pub manifest_format: ManifestFormat,
}

pub(crate) fn detect_package_info(dir: &Path) -> Result<PackageManifest> {
    // Try horus.yaml first (primary manifest)
    let horus_yaml = dir.join(HORUS_YAML);
    if horus_yaml.exists() {
        let content = fs::read_to_string(&horus_yaml)?;

        let mut name = String::from("unknown");
        let mut version = String::from("0.1.0");
        let mut description: Option<String> = None;
        let mut license: Option<String> = None;
        let mut package_type: Option<String> = None;
        let mut categories: Option<String> = None;

        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.starts_with("name:") {
                name = trimmed.trim_start_matches("name:").trim().to_string();
            } else if trimmed.starts_with("version:") {
                version = trimmed.trim_start_matches("version:").trim().to_string();
            } else if trimmed.starts_with("description:") {
                description = Some(
                    trimmed
                        .trim_start_matches("description:")
                        .trim()
                        .to_string(),
                );
            } else if trimmed.starts_with("license:") {
                license = Some(trimmed.trim_start_matches("license:").trim().to_string());
            } else if trimmed.starts_with("package_type:") {
                package_type = Some(
                    trimmed
                        .trim_start_matches("package_type:")
                        .trim()
                        .to_string(),
                );
            } else if trimmed.starts_with("categories:") {
                categories = Some(trimmed.trim_start_matches("categories:").trim().to_string());
            }
        }

        return Ok(PackageManifest {
            name,
            version,
            description,
            license,
            package_type,
            categories,
            source_url: None,
            manifest_format: ManifestFormat::HorusYaml,
        });
    }

    // Try Cargo.toml
    let cargo_toml = dir.join(CARGO_TOML);
    if cargo_toml.exists() {
        let content = fs::read_to_string(&cargo_toml)?;
        let toml_value: toml::Value =
            toml::from_str(&content).map_err(|e| anyhow!("failed to parse Cargo.toml: {}", e))?;

        let package = toml_value
            .get("package")
            .ok_or_else(|| anyhow!("Cargo.toml missing [package] table"))?;

        let name = package
            .get("name")
            .and_then(|v| v.as_str())
            .ok_or_else(|| anyhow!("Cargo.toml missing package.name"))?
            .to_string();

        let version = package
            .get("version")
            .and_then(|v| v.as_str())
            .ok_or_else(|| anyhow!("Cargo.toml missing package.version"))?
            .to_string();

        let description = package
            .get("description")
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        let license = package
            .get("license")
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        let source_url = package
            .get("repository")
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        // Check [package.metadata.horus] for horus-specific fields
        let horus_meta = package.get("metadata").and_then(|m| m.get("horus"));

        let package_type = horus_meta
            .and_then(|h| h.get("package_type"))
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        let categories = horus_meta
            .and_then(|h| h.get("categories"))
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        return Ok(PackageManifest {
            name,
            version,
            description,
            license,
            package_type,
            categories,
            source_url,
            manifest_format: ManifestFormat::CargoToml,
        });
    }

    // Try package.json
    let package_json = dir.join("package.json");
    if package_json.exists() {
        let content = fs::read_to_string(&package_json)?;
        let json: serde_json::Value = serde_json::from_str(&content)
            .map_err(|e| anyhow!("failed to parse package.json: {}", e))?;

        let name = json
            .get("name")
            .and_then(|v| v.as_str())
            .ok_or_else(|| anyhow!("package.json missing name"))?
            .to_string();

        let version = json
            .get("version")
            .and_then(|v| v.as_str())
            .ok_or_else(|| anyhow!("package.json missing version"))?
            .to_string();

        let description = json
            .get("description")
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        let license = json
            .get("license")
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        // Check "horus" key for horus-specific fields
        let horus_meta = json.get("horus");

        let package_type = horus_meta
            .and_then(|h| h.get("package_type"))
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        let categories = horus_meta
            .and_then(|h| h.get("categories"))
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());

        let source_url = json.get("repository").and_then(|v| {
            // repository can be a string or an object with "url"
            v.as_str()
                .map(|s| s.to_string())
                .or_else(|| v.get("url").and_then(|u| u.as_str()).map(|s| s.to_string()))
        });

        return Ok(PackageManifest {
            name,
            version,
            description,
            license,
            package_type,
            categories,
            source_url,
            manifest_format: ManifestFormat::PackageJson,
        });
    }

    Err(anyhow!(
        "no package manifest found. Supported: horus.yaml, Cargo.toml, package.json\n\
         Run 'horus new <name>' to create a new package with horus.yaml."
    ))
}

// Extract HORUS dependencies from package metadata
pub(crate) fn extract_package_dependencies(dir: &Path) -> Result<Vec<DependencySpec>> {
    let mut dependencies = Vec::new();

    // Try Cargo.toml
    if dir.join(CARGO_TOML).exists() {
        let content = fs::read_to_string(dir.join(CARGO_TOML))?;
        let toml: toml::Value = toml::from_str(&content)?;

        // Extract dependencies from [dependencies] section
        if let Some(deps) = toml.get("dependencies").and_then(|v| v.as_table()) {
            for (dep_name, dep_value) in deps {
                // Only include HORUS packages (start with "horus")
                if dep_name.starts_with("horus") {
                    // Extract version requirement if present
                    let spec_str = if let Some(version) = dep_value.as_str() {
                        format!("{}@{}", dep_name, version)
                    } else if let Some(table) = dep_value.as_table() {
                        if let Some(version) = table.get("version").and_then(|v| v.as_str()) {
                            format!("{}@{}", dep_name, version)
                        } else {
                            dep_name.to_string()
                        }
                    } else {
                        dep_name.to_string()
                    };

                    if let Ok(spec) = DependencySpec::parse(&spec_str) {
                        dependencies.push(spec);
                    }
                }
            }
        }
    }

    // Try package.json
    if dir.join("package.json").exists() {
        let content = fs::read_to_string(dir.join("package.json"))?;
        let json: serde_json::Value = serde_json::from_str(&content)?;

        // Extract dependencies
        if let Some(deps) = json.get("dependencies").and_then(|v| v.as_object()) {
            for (dep_name, dep_value) in deps {
                // Only include HORUS packages
                if dep_name.starts_with("horus") {
                    let spec_str = if let Some(version) = dep_value.as_str() {
                        format!("{}@{}", dep_name, version)
                    } else {
                        dep_name.to_string()
                    };

                    if let Ok(spec) = DependencySpec::parse(&spec_str) {
                        dependencies.push(spec);
                    }
                }
            }
        }
    }

    // Try horus.yaml
    if dir.join(HORUS_YAML).exists() {
        let content = fs::read_to_string(dir.join(HORUS_YAML))?;
        // Simple YAML parsing for dependencies
        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.starts_with("- ") && !trimmed.contains(':') {
                // Simple list item
                let dep = trimmed[2..].trim();
                if dep.starts_with("horus") {
                    if let Ok(spec) = DependencySpec::parse(dep) {
                        dependencies.push(spec);
                    }
                }
            } else if trimmed.starts_with("dependencies:") {
                // Dependencies section marker, items come next
                continue;
            }
        }
    }

    Ok(dependencies)
}

/// Check if a Cargo.toml has dependencies on horus crates (horus_core, horus_library, horus_macros)
fn cargo_toml_has_horus_deps(package_dir: &Path) -> bool {
    let cargo_toml_path = package_dir.join(CARGO_TOML);
    let content = match fs::read_to_string(&cargo_toml_path) {
        Ok(c) => c,
        Err(_) => return false,
    };
    let table: toml::Table = match content.parse() {
        Ok(t) => t,
        Err(_) => return false,
    };
    let deps = match table.get("dependencies").and_then(|d| d.as_table()) {
        Some(d) => d,
        None => return false,
    };
    deps.contains_key("horus_core")
        || deps.contains_key("horus_library")
        || deps.contains_key("horus_macros")
}

/// Inject a temporary `.cargo/config.toml` with `[patch.crates-io]` overrides
/// pointing horus crates to the local HORUS source directory.
///
/// This is needed because installed packages use version deps (e.g. `horus_core = "0.1.9"`)
/// which cargo can't resolve from crates.io (horus crates aren't published there).
/// The patch section redirects resolution to the local HORUS source.
fn inject_horus_path_overrides(package_dir: &Path) -> Result<()> {
    if !cargo_toml_has_horus_deps(package_dir) {
        return Err(anyhow!("Package does not depend on horus crates"));
    }

    // Don't overwrite an existing .cargo/config.toml (e.g. from local dev)
    let cargo_config_dir = package_dir.join(".cargo");
    let cargo_config_path = cargo_config_dir.join("config.toml");
    if cargo_config_path.exists() {
        log::debug!(
            "Skipping horus path override injection: .cargo/config.toml already exists at {}",
            cargo_config_path.display()
        );
        return Err(anyhow!(".cargo/config.toml already exists"));
    }

    let horus_source = crate::commands::run::find_horus_source_dir()?;

    // Handle cache vs dev source directory layout:
    // - Cache: ~/.horus/cache/ with crates at horus@0.1.0/horus_core/
    // - Dev: ~/softmata/horus/ with crates at horus_core/
    let base = if horus_source.ends_with(".horus/cache")
        || horus_source.ends_with(".horus\\cache")
    {
        horus_source.join("horus@0.1.0")
    } else {
        horus_source.clone()
    };

    let horus_core_path = base.join("horus_core");
    let horus_library_path = base.join("horus_library");
    let horus_macros_path = base.join("horus_macros");

    let mut config = String::from("# Auto-generated by horus pkg install for build-time dep resolution\n[patch.crates-io]\n");

    if horus_core_path.exists() {
        config.push_str(&format!(
            "horus_core = {{ path = \"{}\" }}\n",
            horus_core_path.display()
        ));
    }
    if horus_library_path.exists() {
        config.push_str(&format!(
            "horus_library = {{ path = \"{}\" }}\n",
            horus_library_path.display()
        ));
    }
    if horus_macros_path.exists() {
        config.push_str(&format!(
            "horus_macros = {{ path = \"{}\" }}\n",
            horus_macros_path.display()
        ));
    }

    fs::create_dir_all(&cargo_config_dir)?;
    fs::write(&cargo_config_path, config)?;
    println!(
        "  {} Injected horus path overrides from {}",
        "".cyan(),
        horus_source.display()
    );

    Ok(())
}

/// Remove the injected `.cargo/config.toml` after build completes.
fn cleanup_horus_path_overrides(package_dir: &Path) {
    let cargo_config_path = package_dir.join(".cargo/config.toml");
    if cargo_config_path.exists() {
        if let Err(e) = fs::remove_file(&cargo_config_path) {
            log::warn!("Failed to clean up .cargo/config.toml: {}", e);
        }
        // Remove .cargo dir if empty
        let cargo_dir = package_dir.join(".cargo");
        let _ = fs::remove_dir(&cargo_dir); // Only succeeds if empty
    }
}

// Pre-compile Rust packages (Python packages don't need pre-compilation)
pub(crate) fn precompile_package(package_dir: &Path) -> Result<()> {
    use std::process::Command;

    // Detect package language - HORUS supports Rust and Python packages only
    let has_cargo_toml = package_dir.join(CARGO_TOML).exists();

    if has_cargo_toml {
        // Rust package - compile with cargo
        println!("  {} Pre-compiling Rust package...", "".cyan());

        // Check if package depends on horus crates and inject path overrides
        let injected_config = inject_horus_path_overrides(package_dir);
        if let Err(ref e) = injected_config {
            log::debug!("Skipping horus path override injection: {}", e);
        }

        let status = Command::new("cargo")
            .arg("build")
            .arg("--release")
            .arg("--lib")
            .current_dir(package_dir)
            .status();

        // Clean up injected .cargo/config.toml regardless of build result
        if injected_config.is_ok() {
            cleanup_horus_path_overrides(package_dir);
        }

        let status = status?;
        if !status.success() {
            return Err(anyhow!("Cargo build failed"));
        }

        // Copy compiled artifacts to lib/ directory for easy access
        let target_dir = package_dir.join("target/release");
        let lib_dir = package_dir.join("lib");
        fs::create_dir_all(&lib_dir)?;

        // Copy .rlib and .so files
        if target_dir.exists() {
            for entry in fs::read_dir(&target_dir)? {
                let entry = entry?;
                let path = entry.path();
                let name = entry.file_name();
                let name_str = name.to_string_lossy();

                if name_str.ends_with(".rlib")
                    || name_str.ends_with(".so")
                    || name_str.ends_with(".a")
                {
                    let dest = lib_dir.join(&name);
                    fs::copy(&path, &dest)?;
                }
            }

            // Also check deps directory
            let deps_dir = target_dir.join("deps");
            if deps_dir.exists() {
                for entry in fs::read_dir(&deps_dir)? {
                    let entry = entry?;
                    let path = entry.path();
                    let name = entry.file_name();
                    let name_str = name.to_string_lossy();

                    if name_str.ends_with(".rlib")
                        || name_str.ends_with(".so")
                        || name_str.ends_with(".a")
                    {
                        let dest = lib_dir.join(&name);
                        fs::copy(&path, &dest)?;
                    }
                }
            }
        }

        println!("  {} Rust package pre-compiled", "".green());
    } else {
        // Not a compiled package (Python packages don't need pre-compilation)
        return Err(anyhow!("Not a compiled package"));
    }

    Ok(())
}

// Get API key from ~/.horus/auth.json
pub(crate) fn get_api_key() -> Result<String> {
    let auth_file = crate::paths::auth_config_path()?;

    if !auth_file.exists() {
        return Err(anyhow!("not authenticated. Please run: horus auth login"));
    }

    let content = fs::read_to_string(&auth_file)?;
    let auth: serde_json::Value = serde_json::from_str(&content)?;

    let api_key = auth
        .get("api_key")
        .and_then(|v| v.as_str())
        .ok_or_else(|| anyhow!("API key not found in auth.json"))?;

    Ok(api_key.to_string())
}

// Interactive prompts for package documentation, source URLs, categories, and package type
pub(crate) fn prompt_package_metadata(
    dir: &Path,
) -> Result<(String, String, String, String, String)> {
    use std::io::{self, Write};

    let mut docs_url = String::new();
    let mut docs_type = String::new();
    let mut source_url = String::new();
    let mut categories = String::new();
    let mut package_type = String::new();

    // Check if /docs folder exists with .md files
    let docs_dir = dir.join("docs");
    let has_local_docs = docs_dir.exists() && docs_dir.is_dir() && {
        fs::read_dir(&docs_dir)
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .any(|e| e.path().extension().and_then(|s| s.to_str()) == Some("md"))
            })
            .unwrap_or(false)
    };

    // Try to auto-detect Git remote URL
    let git_config_path = dir.join(".git/config");
    let detected_git_url = if git_config_path.exists() {
        fs::read_to_string(&git_config_path)
            .ok()
            .and_then(|content| {
                // Extract URL from git config
                for line in content.lines() {
                    let trimmed = line.trim();
                    if trimmed.starts_with("url = ") {
                        let url = trimmed.trim_start_matches("url = ");
                        // Convert git@github.com:user/repo.git to https://github.com/user/repo
                        if url.starts_with("git@github.com:") {
                            let repo = url
                                .trim_start_matches("git@github.com:")
                                .trim_end_matches(".git");
                            return Some(format!("https://github.com/{}", repo));
                        } else if url.starts_with("https://") {
                            return Some(url.trim_end_matches(".git").to_string());
                        }
                    }
                }
                None
            })
    } else {
        None
    };

    // 1. Documentation prompt
    println!("\n{}", "Documentation".cyan().bold());
    if has_local_docs {
        println!(
            "   {} Found local /docs folder with markdown files",
            "".green()
        );
    }
    print!("   Add documentation? (y/n): ");
    io::stdout().flush()?;

    let mut add_docs = String::new();
    io::stdin().read_line(&mut add_docs)?;

    if add_docs.trim().to_lowercase() == "y" {
        println!("\n   Documentation options:");
        println!(
            "     {} External URL - Link to online documentation (e.g., https://docs.example.com)",
            "1.".cyan()
        );
        println!(
            "     {} Local /docs - Bundle markdown files in a /docs folder",
            "2.".cyan()
        );

        if has_local_docs {
            println!(
                "\n   {} Your /docs folder should contain .md files organized as:",
                "[i]".blue()
            );
            println!("      /docs/README.md          (main documentation)");
            println!("      /docs/getting-started.md (guides)");
            println!("      /docs/api.md             (API reference)");
        } else {
            println!(
                "\n   {} To use local docs, create a /docs folder with .md files:",
                "[i]".blue()
            );
            println!("      • Add README.md as the main page");
            println!("      • Use markdown formatting");
            println!("      • Organize by topic (getting-started.md, api.md, etc.)");
        }

        print!("\n   Choose option (1/2/skip): ");
        io::stdout().flush()?;

        let mut docs_choice = String::new();
        io::stdin().read_line(&mut docs_choice)?;

        match docs_choice.trim() {
            "1" => {
                print!("   Enter documentation URL: ");
                io::stdout().flush()?;
                io::stdin().read_line(&mut docs_url)?;
                docs_url = docs_url.trim().to_string();
                docs_type = "external".to_string();

                if !docs_url.is_empty() {
                    println!("   {} Documentation URL: {}", "".green(), docs_url);
                }
            }
            "2" => {
                if has_local_docs {
                    docs_url = "docs/".to_string();
                    docs_type = "local".to_string();
                    println!(
                        "   {} Will bundle local /docs folder with package",
                        "".green()
                    );
                } else {
                    println!(
                        "   {} No /docs folder found. Please create one with .md files first.",
                        "".yellow()
                    );
                }
            }
            _ => {
                println!("   {} Skipping documentation", "".dimmed());
            }
        }
    }

    // 2. Source repository prompt
    println!("\n{}", "Source Repository".cyan().bold());
    if let Some(ref git_url) = detected_git_url {
        println!("   {} Auto-detected: {}", "".green(), git_url);
    }
    print!("   Add source repository? (y/n): ");
    io::stdout().flush()?;

    let mut add_source = String::new();
    io::stdin().read_line(&mut add_source)?;

    if add_source.trim().to_lowercase() == "y" {
        if let Some(git_url) = detected_git_url {
            print!("   Use detected URL? (y/n): ");
            io::stdout().flush()?;

            let mut use_detected = String::new();
            io::stdin().read_line(&mut use_detected)?;

            if use_detected.trim().to_lowercase() == "y" {
                source_url = git_url;
                println!("   {} Source repository: {}", "".green(), source_url);
            } else {
                print!("   Enter source repository URL (e.g., https://github.com/user/repo): ");
                io::stdout().flush()?;
                io::stdin().read_line(&mut source_url)?;
                source_url = source_url.trim().to_string();

                if !source_url.is_empty() {
                    println!("   {} Source repository: {}", "".green(), source_url);
                }
            }
        } else {
            println!(
                "   {} Enter the URL where your code is hosted:",
                "[i]".blue()
            );
            println!("      • GitHub: https://github.com/username/repo");
            println!("      • GitLab: https://gitlab.com/username/repo");
            println!("      • Other: Any public repository URL");
            print!("\n   Enter source repository URL: ");
            io::stdout().flush()?;
            io::stdin().read_line(&mut source_url)?;
            source_url = source_url.trim().to_string();

            if !source_url.is_empty() {
                println!("   {} Source repository: {}", "".green(), source_url);
            }
        }
    }

    // 3. Categories prompt
    println!("\n{}", "Categories".cyan().bold());
    println!(
        "   {} Help users discover your package by selecting relevant categories",
        "[i]".blue()
    );
    println!("   Available categories:");
    println!(
        "     {} Navigation    - Path planning, localization, mapping",
        "1.".cyan()
    );
    println!(
        "     {} Vision        - Computer vision, image processing",
        "2.".cyan()
    );
    println!(
        "     {} Perception    - Sensor fusion, object detection",
        "3.".cyan()
    );
    println!(
        "     {} Control       - Motion control, PID, dynamics",
        "4.".cyan()
    );
    println!(
        "     {} App           - Complete applications, demos",
        "5.".cyan()
    );
    println!(
        "     {} Manipulation  - Arm control, grasping, kinematics",
        "6.".cyan()
    );
    println!(
        "     {} Simulation    - Simulators, testing tools",
        "7.".cyan()
    );
    println!(
        "     {} Utilities     - Tools, helpers, common functions",
        "8.".cyan()
    );
    print!("\n   Select categories (comma-separated numbers, e.g., 1,3,5) or skip: ");
    io::stdout().flush()?;

    let mut category_input = String::new();
    io::stdin().read_line(&mut category_input)?;
    let category_input = category_input.trim();

    if !category_input.is_empty() {
        let category_map = [
            "Navigation",
            "Vision",
            "Perception",
            "Control",
            "App",
            "Manipulation",
            "Simulation",
            "Utilities",
        ];

        let selected: Vec<&str> = category_input
            .split(',')
            .filter_map(|s| {
                let num = s.trim().parse::<usize>().ok()?;
                if num > 0 && num <= category_map.len() {
                    Some(category_map[num - 1])
                } else {
                    None
                }
            })
            .collect();

        if !selected.is_empty() {
            categories = selected.join(",");
            println!(
                "   {} Selected categories: {}",
                "".green(),
                selected.join(", ")
            );
        }
    }

    // 4. Package Type prompt
    println!("\n{}", "Package Type".cyan().bold());
    println!("   {} What type of package is this?", "[i]".blue());
    println!("   Available types:");
    println!(
        "     {} node       - HORUS node that processes data (pub/sub)",
        "1.".cyan()
    );
    println!(
        "     {} driver     - Hardware driver (sensors, actuators)",
        "2.".cyan()
    );
    println!("     {} tool       - CLI tool or utility", "3.".cyan());
    println!(
        "     {} algorithm  - Reusable algorithm implementation",
        "4.".cyan()
    );
    println!(
        "     {} model      - AI/ML model (ONNX, TensorFlow, etc.)",
        "5.".cyan()
    );
    println!("     {} message    - Message type definitions", "6.".cyan());
    println!(
        "     {} app        - Complete multi-node application",
        "7.".cyan()
    );
    print!("\n   Select package type (1-7) or skip for default [node]: ");
    io::stdout().flush()?;

    let mut type_input = String::new();
    io::stdin().read_line(&mut type_input)?;
    let type_input = type_input.trim();

    if !type_input.is_empty() {
        let type_map = [
            "node",
            "driver",
            "tool",
            "algorithm",
            "model",
            "message",
            "app",
        ];

        if let Ok(num) = type_input.parse::<usize>() {
            if num > 0 && num <= type_map.len() {
                package_type = type_map[num - 1].to_string();
                println!("   {} Package type: {}", "".green(), package_type);
            }
        }
    }

    // Default to "node" if not specified
    if package_type.is_empty() {
        package_type = "node".to_string();
        println!("   {} Using default package type: node", "".blue());
    }

    Ok((docs_url, docs_type, source_url, categories, package_type))
}

// Implement PackageProvider trait for RegistryClient to enable dependency resolution
impl PackageProvider for RegistryClient {
    fn get_available_versions(&self, package: &str) -> Result<Vec<Version>> {
        // Query registry for available versions
        let url = format!("{}/api/packages/{}/versions", self.base_url, package);

        let response = self.client.get(&url).send();

        match response {
            Ok(resp) if resp.status().is_success() => {
                #[derive(Deserialize)]
                struct VersionsResponse {
                    versions: Vec<String>,
                }

                let versions_resp: VersionsResponse =
                    resp.json().unwrap_or(VersionsResponse { versions: vec![] });

                // Parse version strings to semver::Version
                let mut versions: Vec<Version> = versions_resp
                    .versions
                    .iter()
                    .filter_map(|v| Version::parse(v).ok())
                    .collect();

                versions.sort();

                // If registry has versions, return them
                // If empty, fall back to local cache (for built-in packages like "horus")
                if !versions.is_empty() {
                    return Ok(versions);
                }

                // Fall through to cache check below
            }
            _ => {}
        }

        // Fallback: check local/global cache for versions
        let global_cache = crate::paths::cache_dir()?;
        let local_packages = PathBuf::from(".horus/packages");

        let mut versions = Vec::new();

        // Check global cache
        if let Ok(entries) = fs::read_dir(&global_cache) {
            for entry in entries.flatten() {
                let name = entry.file_name();
                let name_str = name.to_string_lossy();

                // Match "package@version" pattern
                if name_str.starts_with(&format!("{}@", package)) {
                    if let Some(version_str) = name_str.split('@').nth(1) {
                        if let Ok(version) = Version::parse(version_str) {
                            versions.push(version);
                        }
                    }
                }
            }
        }

        // Check local packages
        if let Ok(entries) = fs::read_dir(&local_packages) {
            for entry in entries.flatten() {
                let name = entry.file_name();
                let name_str = name.to_string_lossy();

                if name_str == package {
                    // Read version from metadata
                    if let Some(version) = detect_package_version(&entry.path()) {
                        if let Ok(v) = Version::parse(&version) {
                            versions.push(v);
                        }
                    }
                }
            }
        }

        versions.sort();
        versions.dedup();

        if versions.is_empty() {
            Err(anyhow!("No versions found for package: {}", package))
        } else {
            Ok(versions)
        }
    }

    fn get_dependencies(&self, package: &str, version: &Version) -> Result<Vec<DependencySpec>> {
        // Query registry for package dependencies
        let url = format!(
            "{}/api/packages/{}/{}/metadata",
            self.base_url, package, version
        );

        let response = self.client.get(&url).send();

        match response {
            Ok(resp) if resp.status().is_success() => {
                #[derive(Deserialize)]
                struct MetadataResponse {
                    dependencies: Option<Vec<DependencyInfo>>,
                }

                #[derive(Deserialize)]
                struct DependencyInfo {
                    name: String,
                    version_req: Option<String>,
                }

                let metadata: MetadataResponse = resp
                    .json()
                    .unwrap_or(MetadataResponse { dependencies: None });

                let deps: Vec<DependencySpec> = metadata
                    .dependencies
                    .unwrap_or_default()
                    .into_iter()
                    .filter_map(|dep| {
                        let spec_str = if let Some(req) = dep.version_req {
                            format!("{}@{}", dep.name, req)
                        } else {
                            dep.name
                        };
                        DependencySpec::parse(&spec_str).ok()
                    })
                    .collect();

                Ok(deps)
            }
            _ => {
                // Fallback: read from local cache
                let global_cache = crate::paths::cache_dir()?;
                let package_dir_name = format!("{}@{}", package, version);
                let package_dir = global_cache.join(&package_dir_name);

                if package_dir.exists() {
                    extract_package_dependencies(&package_dir)
                } else {
                    // Check local
                    let local_packages = PathBuf::from(".horus/packages");
                    let local_dir = local_packages.join(package);

                    if local_dir.exists() {
                        extract_package_dependencies(&local_dir)
                    } else {
                        Ok(vec![]) // No dependencies
                    }
                }
            }
        }
    }
}

// ============================================================================
// Package Update & Parallel Downloads
// ============================================================================

impl RegistryClient {
    /// Update installed packages to their latest versions
    pub fn update_packages(
        &self,
        package: Option<&str>,
        global: bool,
        dry_run: bool,
    ) -> Result<()> {
        let global_cache = crate::paths::cache_dir()?;

        // Determine which directories to scan
        let mut scan_dirs: Vec<(PathBuf, &str)> = Vec::new();

        if global {
            if global_cache.exists() {
                scan_dirs.push((global_cache, "global"));
            }
        } else {
            let local_packages = PathBuf::from(".horus/packages");
            if local_packages.exists() {
                scan_dirs.push((local_packages, "local"));
            }
            if global_cache.exists() {
                scan_dirs.push((global_cache, "global"));
            }
        }

        if scan_dirs.is_empty() {
            println!("No installed packages found.");
            return Ok(());
        }

        let mut updates: Vec<(String, String, String, PathBuf)> = Vec::new(); // (name, old_ver, new_ver, path)

        for (dir, _location) in &scan_dirs {
            if !dir.exists() {
                continue;
            }
            let entries = fs::read_dir(dir)?;
            for entry in entries {
                let entry = entry?;
                let path = entry.path();
                if !path.is_dir() {
                    continue;
                }

                let metadata_path = path.join("metadata.json");
                if !metadata_path.exists() {
                    continue;
                }

                let metadata_str = fs::read_to_string(&metadata_path)?;
                let metadata: serde_json::Value = serde_json::from_str(&metadata_str)?;

                let pkg_name = metadata.get("name").and_then(|v| v.as_str()).unwrap_or("");
                let pkg_version = metadata
                    .get("version")
                    .and_then(|v| v.as_str())
                    .unwrap_or("");

                if pkg_name.is_empty() || pkg_version.is_empty() {
                    continue;
                }

                // If a specific package was requested, skip others
                if let Some(filter) = package {
                    if pkg_name != filter {
                        continue;
                    }
                }

                // Check for newer version from registry
                let encoded_name = url_encode_package_name(pkg_name);
                let url = format!("{}/api/packages/{}", self.base_url, encoded_name);
                match self.client.get(&url).send() {
                    Ok(response) if response.status().is_success() => {
                        if let Ok(info) = response.json::<serde_json::Value>() {
                            let latest = info.get("version").and_then(|v| v.as_str()).unwrap_or("");
                            if !latest.is_empty() {
                                if let (Ok(current), Ok(latest_ver)) =
                                    (Version::parse(pkg_version), Version::parse(latest))
                                {
                                    if latest_ver > current {
                                        updates.push((
                                            pkg_name.to_string(),
                                            pkg_version.to_string(),
                                            latest.to_string(),
                                            path.clone(),
                                        ));
                                    }
                                }
                            }
                        }
                    }
                    _ => {
                        // Skip packages we can't check
                    }
                }
            }
        }

        if updates.is_empty() {
            println!(" All packages are up to date!");
            return Ok(());
        }

        // Print summary
        println!("\n{} packages can be updated:\n", updates.len());
        println!("  {:<30} {:<15} Latest", "Package", "Current");
        println!("  {:<30} {:<15} ------", "-------", "-------");
        for (name, old_ver, new_ver, _) in &updates {
            println!(
                "  {:<30} {:<15} {}",
                name.cyan(),
                old_ver.yellow(),
                new_ver.green()
            );
        }

        if dry_run {
            println!("\n {} Dry run - no changes made.", "".cyan());
            return Ok(());
        }

        // Perform updates
        println!();
        for (name, old_ver, new_ver, old_path) in &updates {
            let spinner = progress::robot_download_spinner(&format!(
                "Updating {} {} -> {}...",
                name, old_ver, new_ver
            ));

            let target = if global {
                crate::workspace::InstallTarget::Global
            } else {
                crate::workspace::InstallTarget::Local(PathBuf::from("."))
            };

            match self.install_from_registry(name, Some(new_ver.as_str()), target) {
                Ok(_) => {
                    // Remove old version directory if it differs
                    if old_path.exists() {
                        let _ = fs::remove_dir_all(old_path);
                    }
                    finish_success(
                        &spinner,
                        &format!("Updated {} {} -> {}", name, old_ver, new_ver),
                    );
                }
                Err(e) => {
                    finish_error(&spinner, &format!("Failed to update {}: {}", name, e));
                }
            }
        }

        println!("\n {} Update complete!", "".green());
        Ok(())
    }

    /// Install dependencies in parallel using rayon
    pub fn install_dependencies_parallel(
        &self,
        deps: &[crate::dependency_resolver::DependencySpec],
        target: &crate::workspace::InstallTarget,
    ) -> Result<()> {
        use rayon::prelude::*;

        if deps.is_empty() {
            return Ok(());
        }

        println!(
            "  {} Installing {} dependencies in parallel...",
            "".cyan(),
            deps.len()
        );

        // Cap at 8 parallel threads
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(8.min(deps.len()))
            .build()
            .map_err(|e| anyhow!("Failed to create thread pool: {}", e))?;

        let results: Vec<Result<()>> = pool.install(|| {
            deps.par_iter()
                .map(|dep| {
                    // Each thread gets its own RegistryClient (cheap — just a reqwest::blocking::Client)
                    let client = RegistryClient::new();
                    let version_req = dep.requirement.to_string();
                    let version = if version_req == "*" {
                        None
                    } else {
                        Some(version_req.as_str())
                    };
                    match client.install_from_registry(&dep.name, version, target.clone()) {
                        Ok(_) => Ok(()),
                        Err(e) => {
                            log::error!("Failed to install {}: {}", dep.name, e);
                            Err(e)
                        }
                    }
                })
                .collect()
        });

        let failures: Vec<_> = results.iter().filter(|r| r.is_err()).collect();
        if !failures.is_empty() {
            log::warn!("{} dependencies failed to install", failures.len());
        }

        Ok(())
    }
}

// ============================================================================
// Package Signing
// ============================================================================

/// Generate an Ed25519 signing keypair for package signing
pub fn generate_signing_keypair() -> Result<()> {
    use ed25519_dalek::SigningKey;
    use rand::rngs::OsRng;

    let keys_dir = crate::paths::keys_dir()?;
    fs::create_dir_all(&keys_dir)?;

    let secret_path = keys_dir.join("signing_key");
    let public_path = keys_dir.join("signing_key.pub");

    if secret_path.exists() {
        println!(" Signing key already exists at {}", secret_path.display());
        println!("   Delete it first if you want to regenerate.");
        return Ok(());
    }

    let signing_key = SigningKey::generate(&mut OsRng);
    let verifying_key = signing_key.verifying_key();

    // Write secret key (64 bytes: 32 secret + 32 public)
    fs::write(&secret_path, signing_key.to_bytes())?;

    // Restrict permissions on secret key
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        fs::set_permissions(&secret_path, fs::Permissions::from_mode(0o600))?;
    }

    // Write public key (32 bytes, hex encoded for readability)
    let pub_hex = hex::encode(verifying_key.to_bytes());
    fs::write(&public_path, &pub_hex)?;

    println!(" Generated signing keypair:");
    println!("   Secret key: {}", secret_path.display());
    println!("   Public key: {}", public_path.display());
    println!("   Public key (hex): {}", pub_hex);
    println!(
        "\n   {} Keep your secret key safe! Anyone with it can sign packages as you.",
        "".yellow()
    );

    Ok(())
}
