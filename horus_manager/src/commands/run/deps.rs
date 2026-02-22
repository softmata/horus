use crate::config::HORUS_YAML;
use crate::dependency_resolver::DependencySpec;
use anyhow::{anyhow, bail, Result};
use colored::*;
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

use super::features::IgnorePatterns;

/// Python package dependency for pip
#[derive(Debug, Clone)]
pub(crate) struct PipPackage {
    pub(crate) name: String,
    pub(crate) version: Option<String>, // None means latest
}

impl PipPackage {
    pub(crate) fn from_string(s: &str) -> Result<Self> {
        // Parse formats:
        // - "numpy>=1.24.0"
        // - "numpy==1.24.0"
        // - "numpy~=1.24"
        // - "numpy@1.24.0" (HORUS-style)
        // - "numpy"

        let s = s.trim();

        // Handle @ separator (HORUS-style: numpy@1.24.0)
        if let Some(at_pos) = s.find('@') {
            let name = s[..at_pos].trim().to_string();
            let version_str = s[at_pos + 1..].trim();
            let version = if !version_str.is_empty() {
                Some(format!("=={}", version_str))
            } else {
                None
            };
            return Ok(PipPackage { name, version });
        }

        // Handle comparison operators (>=, ==, ~=, etc.)
        let operators = [">=", "<=", "==", "~=", ">", "<", "!="];
        for op in &operators {
            if let Some(op_pos) = s.find(op) {
                let name = s[..op_pos].trim().to_string();
                let version = Some(s[op_pos..].trim().to_string());
                return Ok(PipPackage { name, version });
            }
        }

        // No version specified
        Ok(PipPackage {
            name: s.to_string(),
            version: None,
        })
    }

    pub(crate) fn requirement_string(&self) -> String {
        match &self.version {
            Some(v) => format!("{}{}", self.name, v),
            None => self.name.clone(),
        }
    }
}

/// Cargo package dependency for Rust binaries
#[derive(Debug, Clone)]
pub(crate) struct CargoPackage {
    pub(crate) name: String,
    pub(crate) version: Option<String>, // None means latest
    pub(crate) features: Vec<String>,   // Cargo features to enable
}

impl CargoPackage {
    pub(crate) fn from_string(s: &str) -> Result<Self> {
        // Parse formats: "bat@0.24.0:features=derive,serde" or "bat@0.24.0" or "bat"
        let s = s.trim();

        // Check for features
        let (pkg_part, features) = if let Some(colon_pos) = s.find(':') {
            let pkg = &s[..colon_pos];
            let features_part = &s[colon_pos + 1..];

            // Parse features=a,b,c
            let features = if let Some(equals_pos) = features_part.find('=') {
                let features_str = &features_part[equals_pos + 1..].trim();
                if features_str.is_empty() {
                    return Err(anyhow!(
                        "Empty features list (remove ':features=' or provide feature names)"
                    ));
                }
                features_str
                    .split(',')
                    .map(|f| f.trim().to_string())
                    .filter(|f| !f.is_empty()) // Filter out empty strings
                    .collect()
            } else if !features_part.is_empty() {
                return Err(anyhow!(
                    "Invalid features syntax '{}' - use 'features=feat1,feat2'",
                    features_part
                ));
            } else {
                Vec::new()
            };
            (pkg, features)
        } else {
            (s, Vec::new())
        };

        // Parse name@version
        if let Some(at_pos) = pkg_part.find('@') {
            let name = pkg_part[..at_pos].trim().to_string();
            let version_str = pkg_part[at_pos + 1..].trim();

            // Validate package name
            if name.is_empty() {
                return Err(anyhow!("Package name cannot be empty"));
            }

            let version = if !version_str.is_empty() {
                // Basic semver validation
                if version_str.contains(char::is_whitespace) {
                    return Err(anyhow!(
                        "Version cannot contain whitespace: '{}'",
                        version_str
                    ));
                }
                // Check for common mistakes
                if version_str == "latest" {
                    return Err(anyhow!("Version 'latest' is not valid - specify a version like '1.0' or omit version"));
                }
                Some(version_str.to_string())
            } else {
                None
            };
            return Ok(CargoPackage {
                name,
                version,
                features,
            });
        }

        // No version specified
        let name = pkg_part.trim().to_string();
        if name.is_empty() {
            return Err(anyhow!("Package name cannot be empty"));
        }

        Ok(CargoPackage {
            name,
            version: None,
            features,
        })
    }
}

pub(crate) fn detect_language(file: &Path) -> Result<String> {
    match file.extension().and_then(|s| s.to_str()) {
        Some("rs") => Ok("rust".to_string()),
        Some("py") => Ok("python".to_string()),
        _ => bail!(
            "Unsupported file type: {}\n\n{}\n  {} Supported: {}\n  {} Got: {}",
            file.display(),
            "Supported file types:".yellow(),
            "•".cyan(),
            ".rs (Rust), .py (Python)".green(),
            "•".cyan(),
            file.extension()
                .and_then(|s| s.to_str())
                .unwrap_or("no extension")
                .red()
        ),
    }
}

pub(crate) fn scan_imports(
    file: &Path,
    language: &str,
    ignore: &IgnorePatterns,
) -> Result<HashSet<String>> {
    let content = fs::read_to_string(file)?;
    let mut dependencies = HashSet::new();

    // First, check if horus.yaml exists and use it
    let from_yaml = Path::new(HORUS_YAML).exists();

    if from_yaml {
        log::debug!("Reading dependencies from horus.yaml");
        eprintln!("  {} Reading dependencies from horus.yaml", "".cyan());
        let yaml_deps = parse_horus_yaml_dependencies(HORUS_YAML)?;
        dependencies.extend(yaml_deps);
    } else {
        // Fallback: scan imports from source code
        match language {
            "rust" => {
                // Scan for: use horus::*, use horus_library::*, extern crate
                for line in content.lines() {
                    if let Some(dep) = parse_rust_import(line) {
                        dependencies.insert(dep);
                    }
                }

                // Also check Cargo.toml if exists (legacy support)
                if Path::new("Cargo.toml").exists() {
                    let cargo_deps = parse_cargo_dependencies("Cargo.toml")?;
                    dependencies.extend(cargo_deps);
                }
            }
            "python" => {
                // Scan for ALL imports (not just HORUS)
                for line in content.lines() {
                    if let Some(dep) = parse_all_python_imports(line) {
                        dependencies.insert(dep);
                    }
                }
            }
            _ => {}
        }
    }

    // Filter out ignored packages
    dependencies.retain(|dep| !ignore.should_ignore_package(dep));

    // Auto-create or update horus.yaml if we scanned from source
    if !from_yaml && !dependencies.is_empty() {
        auto_update_horus_yaml(file, language, &dependencies)?;
    }

    Ok(dependencies)
}

fn parse_rust_import(line: &str) -> Option<String> {
    let line = line.trim();

    // use horus_library::*
    if let Some(rest) = line.strip_prefix("use ") {
        let parts: Vec<&str> = rest.split("::").collect();
        if !parts.is_empty() {
            let package = parts[0].trim_end_matches(';');
            if package.starts_with("horus") {
                return Some(package.to_string());
            }
        }
    }

    // extern crate horus_library
    if let Some(rest) = line.strip_prefix("extern crate ") {
        let package = rest.trim_end_matches(';').trim();
        if package.starts_with("horus") {
            return Some(package.to_string());
        }
    }

    None
}

/// Parse ALL Python imports (not just HORUS)
fn parse_all_python_imports(line: &str) -> Option<String> {
    let line = line.trim();

    // Skip comments
    if line.starts_with('#') {
        return None;
    }

    // import numpy
    // import numpy as np
    if let Some(rest) = line.strip_prefix("import ") {
        let package = rest.split_whitespace().next()?.split('.').next()?;
        // Skip relative imports and standard library
        if !is_stdlib_package(package) && !package.starts_with('.') {
            return Some(package.to_string());
        }
    }

    // from numpy import something
    if let Some(rest) = line.strip_prefix("from ") {
        let parts: Vec<&str> = rest.split(" import ").collect();
        if !parts.is_empty() {
            let package = parts[0].trim().split('.').next()?;
            // Skip relative imports and standard library
            if !is_stdlib_package(package) && !package.starts_with('.') {
                return Some(package.to_string());
            }
        }
    }

    None
}

/// Check if package is Python standard library
fn is_stdlib_package(name: &str) -> bool {
    let stdlib = [
        "os",
        "sys",
        "re",
        "json",
        "time",
        "datetime",
        "math",
        "random",
        "collections",
        "itertools",
        "functools",
        "pathlib",
        "typing",
        "asyncio",
        "threading",
        "multiprocessing",
        "subprocess",
        "logging",
        "argparse",
        "configparser",
        "io",
        "pickle",
        "csv",
        "xml",
        "html",
        "http",
        "urllib",
        "socket",
        "email",
        "base64",
        "hashlib",
        "hmac",
        "secrets",
        "uuid",
        "dataclasses",
        "enum",
        "abc",
        "contextlib",
    ];
    stdlib.contains(&name)
}

// Removed: parse_c_include() - C support no longer provided

/// Parse a single YAML dependency and convert to Cargo.toml format
/// Handles: - horus, - name: serde with version: "1" features: [derive]
pub(crate) fn parse_yaml_cargo_dependency(value: &serde_yaml::Value) -> Option<String> {
    match value {
        serde_yaml::Value::String(dep_str) => {
            // Simple string: - horus, - cargo:serde@1.0:features=derive, etc.
            let dep = dep_str.trim();

            // Skip horus packages - they're already added as path dependencies
            if dep == "horus" || dep.starts_with("horus_") || dep.starts_with("horus@") {
                return None;
            }

            // Parse cargo: or pip: prefixed dependencies
            let dep_clean = if let Some(rest) = dep.strip_prefix("cargo:") {
                rest // Remove "cargo:" prefix
            } else if dep.starts_with("pip:") {
                // Skip pip dependencies in Cargo.toml
                return None;
            } else {
                dep
            };

            // Parse package@version:features=feat1,feat2 format
            if let Some(at_pos) = dep_clean.find('@') {
                let pkg_name = dep_clean[..at_pos].trim();
                let rest = &dep_clean[at_pos + 1..];

                // Split version and features
                if let Some(features_pos) = rest.find(":features=") {
                    let version = rest[..features_pos].trim();
                    let features_str = rest[features_pos + 10..].trim(); // Skip ":features="
                    let features: Vec<&str> = features_str.split(',').map(|s| s.trim()).collect();

                    return Some(format!(
                        "{} = {{ version = \"{}\", features = [{}] }}",
                        pkg_name,
                        version,
                        features
                            .iter()
                            .map(|f| format!("\"{}\"", f))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ));
                } else {
                    // Just version, no features
                    let version = rest.trim();
                    return Some(format!("{} = \"{}\"", pkg_name, version));
                }
            }

            // No version specified, use "*"
            Some(format!("{} = \"*\"", dep_clean))
        }
        serde_yaml::Value::Mapping(map) => {
            // Map format: - name: serde, version: "1", features: [derive]
            let name = map.get("name")?.as_str()?;

            if name == "horus" || name.starts_with("horus_") {
                return None; // Skip, already added
            }

            let mut cargo_dep = format!("{} = ", name);

            // Check for simple version string
            if let Some(version) = map.get("version").and_then(|v| v.as_str()) {
                // Check if features exist
                if let Some(features_val) = map.get("features") {
                    if let Some(features_list) = features_val.as_sequence() {
                        let features: Vec<&str> =
                            features_list.iter().filter_map(|f| f.as_str()).collect();

                        if !features.is_empty() {
                            cargo_dep.push_str(&format!(
                                "{{ version = \"{}\", features = [{}] }}",
                                version,
                                features
                                    .iter()
                                    .map(|f| format!("\"{}\"", f))
                                    .collect::<Vec<_>>()
                                    .join(", ")
                            ));
                            return Some(cargo_dep);
                        }
                    }
                }

                // No features, just version
                cargo_dep.push_str(&format!("\"{}\"", version));
                Some(cargo_dep)
            } else {
                // No version specified
                Some(format!("{} = \"*\"", name))
            }
        }
        _ => None,
    }
}

pub(crate) fn parse_horus_yaml_dependencies(path: &str) -> Result<HashSet<String>> {
    let content = fs::read_to_string(path)?;

    // Try to parse as proper YAML first (supports complex table syntax)
    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut dependencies = HashSet::new();

            // Get language context
            let language = yaml
                .get("language")
                .and_then(|v| v.as_str())
                .map(|s| s.to_string());

            // Parse dependencies
            if let Some(deps_value) = yaml.get("dependencies") {
                match deps_value {
                    // Array format: dependencies: [- pkg, - pkg = "version"]
                    serde_yaml::Value::Sequence(list) => {
                        for item in list {
                            if let Some(dep_str) =
                                parse_dependency_value(item, language.as_deref())?
                            {
                                dependencies.insert(dep_str);
                            }
                        }
                    }

                    // Map format: dependencies: { pkg: version, pkg: {path: ...} }
                    serde_yaml::Value::Mapping(map) => {
                        for (key, value) in map {
                            if let serde_yaml::Value::String(pkg_name) = key {
                                if let Some(dep_str) = parse_dependency_map_entry(
                                    pkg_name,
                                    value,
                                    language.as_deref(),
                                )? {
                                    dependencies.insert(dep_str);
                                }
                            }
                        }
                    }

                    _ => {}
                }
            }

            Ok(dependencies)
        }

        // Fallback to simple line-by-line parsing for malformed YAML
        Err(_) => parse_horus_yaml_dependencies_simple(path),
    }
}

/// Parse a single dependency value from YAML (array item)
fn parse_dependency_value(
    value: &serde_yaml::Value,
    language: Option<&str>,
) -> Result<Option<String>> {
    match value {
        serde_yaml::Value::String(dep_str) => {
            let dep_str = dep_str.trim();
            if dep_str.is_empty() || dep_str.starts_with('#') {
                return Ok(None);
            }

            // Skip normalization for strings that already have package manager prefixes
            // (cargo: or pip:) to avoid corrupting feature syntax like ":features=derive"
            if dep_str.starts_with("cargo:") || dep_str.starts_with("pip:") {
                return Ok(Some(dep_str.to_string()));
            }

            // Normalize "pkg = version" syntax
            if let Some(equals_pos) = dep_str.find('=') {
                let pkg_name = dep_str[..equals_pos].trim();
                let version_part = dep_str[equals_pos + 1..].trim();
                let version = version_part.trim_matches('\'').trim_matches('"').trim();

                if !pkg_name.contains(':') {
                    let prefix = match language {
                        Some("python") => "pip",
                        Some("rust") => "cargo",
                        _ => "cargo",
                    };
                    return Ok(Some(format!("{}:{}@{}", prefix, pkg_name, version)));
                } else {
                    return Ok(Some(format!("{}@{}", pkg_name, version)));
                }
            }

            Ok(Some(dep_str.to_string()))
        }
        _ => Ok(None),
    }
}

/// Parse dependency from map format: pkg: {version: "1.0", features: [...]}
fn parse_dependency_map_entry(
    pkg_name: &str,
    value: &serde_yaml::Value,
    language: Option<&str>,
) -> Result<Option<String>> {
    match value {
        // Simple string version: pkg: "1.0"
        serde_yaml::Value::String(version_str) => {
            let version = version_str.trim_matches('\'').trim_matches('"').trim();
            let prefix = if pkg_name.contains(':') {
                ""
            } else {
                match language {
                    Some("python") => "pip:",
                    Some("rust") => "cargo:",
                    _ => "cargo:",
                }
            };
            Ok(Some(format!("{}{}@{}", prefix, pkg_name, version)))
        }

        // Table format: pkg: {version: "1.0", features: ["full"], path: "...", git: "..."}
        serde_yaml::Value::Mapping(map) => {
            // Check for empty map (pkg: {}) - treat as simple dependency
            if map.is_empty() {
                return Ok(Some(pkg_name.to_string()));
            }

            // Extract version
            let version = map
                .get("version")
                .and_then(|v| v.as_str())
                .map(|s| s.trim_matches('\'').trim_matches('"').trim());

            // Extract features
            let features: Vec<String> = map
                .get("features")
                .and_then(|v| v.as_sequence())
                .map(|seq| {
                    seq.iter()
                        .filter_map(|f| f.as_str().map(|s| s.to_string()))
                        .collect()
                })
                .unwrap_or_default();

            // Check for path dependency
            if let Some(path_value) = map.get("path") {
                if let Some(path_str) = path_value.as_str() {
                    // Path dependencies are now supported! Return special format: path:pkg_name:path_str
                    return Ok(Some(format!("path:{}:{}", pkg_name, path_str)));
                }
            }

            // Check for git dependency
            if let Some(git_value) = map.get("git") {
                if let Some(git_str) = git_value.as_str() {
                    // Extract optional branch, tag, or rev
                    let branch = map.get("branch").and_then(|v| v.as_str());
                    let tag = map.get("tag").and_then(|v| v.as_str());
                    let rev = map.get("rev").and_then(|v| v.as_str());

                    // Build git reference string
                    let ref_str = if let Some(b) = branch {
                        format!(":branch={}", b)
                    } else if let Some(t) = tag {
                        format!(":tag={}", t)
                    } else if let Some(r) = rev {
                        format!(":rev={}", r)
                    } else {
                        String::new()
                    };

                    // Return format: git:pkg_name:git_url[:branch=X|tag=X|rev=X]
                    return Ok(Some(format!("git:{}:{}{}", pkg_name, git_str, ref_str)));
                }
            }

            // Build dependency string
            let prefix = if pkg_name.contains(':') {
                ""
            } else {
                match language {
                    Some("python") => "pip:",
                    Some("rust") => "cargo:",
                    _ => "cargo:",
                }
            };

            if let Some(ver) = version {
                if !features.is_empty() {
                    Ok(Some(format!(
                        "{}{}@{}:features={}",
                        prefix,
                        pkg_name,
                        ver,
                        features.join(",")
                    )))
                } else {
                    Ok(Some(format!("{}{}@{}", prefix, pkg_name, ver)))
                }
            } else {
                // No version specified
                if !features.is_empty() {
                    Ok(Some(format!(
                        "{}{}:features={}",
                        prefix,
                        pkg_name,
                        features.join(",")
                    )))
                } else {
                    Ok(Some(format!("{}{}", prefix, pkg_name)))
                }
            }
        }

        _ => Ok(None),
    }
}

/// Fallback simple line-by-line parser for malformed YAML
fn parse_horus_yaml_dependencies_simple(path: &str) -> Result<HashSet<String>> {
    let content = fs::read_to_string(path)?;
    let mut dependencies = HashSet::new();

    // First, detect language from horus.yaml to determine default prefix
    let mut language = None;
    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("language:") {
            language = trimmed
                .strip_prefix("language:")
                .map(|s| s.trim().to_string());
            break;
        }
    }

    // Simple YAML parsing for dependencies section
    let mut in_deps = false;
    for line in content.lines() {
        let trimmed = line.trim();

        if trimmed.starts_with("dependencies:") {
            in_deps = true;
            continue;
        }

        // Exit dependencies section if we hit another top-level key
        if in_deps
            && !trimmed.is_empty()
            && !trimmed.starts_with("- ")
            && !trimmed.starts_with("#")
            && trimmed.contains(':')
        {
            in_deps = false;
        }

        if in_deps && trimmed.starts_with("- ") {
            // Extract full dependency string "package@version" or "package"
            let dep_str = trimmed[2..].trim();
            if dep_str.starts_with("#") {
                continue; // Skip comments
            }

            // Strip inline comments (handle both quoted and unquoted strings)
            let dep_str = if let Some(comment_pos) = dep_str.find('#') {
                // Check if the # is inside quotes
                let before_comment = &dep_str[..comment_pos];
                let single_quotes = before_comment.matches('\'').count();
                let double_quotes = before_comment.matches('"').count();

                // If quotes are balanced, # is a comment. Otherwise, it's part of the string
                if single_quotes.is_multiple_of(2) && double_quotes.is_multiple_of(2) {
                    dep_str[..comment_pos].trim()
                } else {
                    dep_str
                }
            } else {
                dep_str
            };

            // Remove surrounding quotes if present
            let dep_str = dep_str.trim_matches('\'').trim_matches('"');

            // Normalize package manager syntax: "eframe = \"0.29\"" -> "cargo:eframe@0.29" or "pip:numpy@1.24"
            let dep_str = if let Some(equals_pos) = dep_str.find('=') {
                let pkg_name = dep_str[..equals_pos].trim();
                let version_part = dep_str[equals_pos + 1..].trim();
                // Remove quotes from version
                let version = version_part.trim_matches('\'').trim_matches('"').trim();

                // If no prefix (cargo:/pip:/horus), infer from language context
                if !pkg_name.contains(':') {
                    let prefix = match language.as_deref() {
                        Some("python") => "pip",
                        Some("rust") => "cargo",
                        _ => "cargo", // Default to cargo if unknown
                    };
                    format!("{}:{}@{}", prefix, pkg_name, version)
                } else {
                    // Already has prefix, just reformat
                    format!("{}@{}", pkg_name, version)
                }
            } else {
                dep_str.to_string()
            };

            // Insert the full dependency string (including version)
            // This will be split later into HORUS vs pip packages
            dependencies.insert(dep_str);
        }
    }

    Ok(dependencies)
}

/// Parse horus.yaml dependencies with support for path, git, and registry sources
/// Returns `Vec<DependencySpec>` which includes source information
pub fn parse_horus_yaml_dependencies_v2(path: &str) -> Result<Vec<DependencySpec>> {
    let content = fs::read_to_string(path)?;

    // Try parsing as proper YAML first (supports structured format)
    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut deps = Vec::new();

            if let Some(deps_value) = yaml.get("dependencies") {
                match deps_value {
                    // List format: dependencies: [- pkg@version]
                    serde_yaml::Value::Sequence(list) => {
                        for item in list {
                            if let serde_yaml::Value::String(dep_str) = item {
                                // Parse simple string format
                                deps.push(DependencySpec::parse(dep_str)?);
                            }
                        }
                    }

                    // Map format: dependencies: { pkg: version, pkg: {path: ...} }
                    serde_yaml::Value::Mapping(map) => {
                        for (key, value) in map {
                            if let serde_yaml::Value::String(name) = key {
                                deps.push(DependencySpec::from_yaml_value(name.clone(), value)?);
                            }
                        }
                    }

                    _ => {}
                }
            }

            Ok(deps)
        }

        // Fallback to simple parsing for backward compatibility
        Err(_) => {
            let old_deps = parse_horus_yaml_dependencies(path)?;
            let mut deps = Vec::new();
            for dep_str in old_deps {
                deps.push(DependencySpec::parse(&dep_str)?);
            }
            Ok(deps)
        }
    }
}

pub(crate) fn parse_cargo_dependencies(path: &str) -> Result<HashSet<String>> {
    let content = fs::read_to_string(path)?;
    let mut dependencies = HashSet::new();

    // Simple TOML parsing for dependencies section
    let mut in_deps = false;
    for line in content.lines() {
        if line.starts_with("[dependencies]") {
            in_deps = true;
            continue;
        }
        if line.starts_with('[') {
            in_deps = false;
        }

        if in_deps {
            if let Some(eq_pos) = line.find('=') {
                let dep = line[..eq_pos].trim();
                // Check if this is a HORUS package or resolvable package
                if dep.starts_with("horus") || is_horus_package(dep) {
                    dependencies.insert(dep.to_string());
                }
            }
        }
    }

    Ok(dependencies)
}

pub(crate) fn is_horus_package(package: &str) -> bool {
    // Only HORUS packages start with "horus" prefix
    // Everything else will be handled by pip integration
    package.starts_with("horus")
}

pub(crate) fn is_cargo_package(package_name: &str) -> bool {
    use reqwest::blocking::Client;

    // Common CLI tools from crates.io
    let common_cli_tools = [
        "bat",
        "fd-find",
        "ripgrep",
        "exa",
        "tokei",
        "hyperfine",
        "starship",
        "zoxide",
        "delta",
        "dust",
        "procs",
        "bottom",
        "tealdeer",
        "sd",
        "grex",
        "xsv",
        "bandwhich",
    ];

    if common_cli_tools.contains(&package_name) {
        return true;
    }

    // Check crates.io API for less common packages
    let client = Client::new();
    let url = format!("{}/{}", crate::config::CRATES_IO_API_URL, package_name);
    if let Ok(response) = client
        .get(&url)
        .header("User-Agent", "horus-pkg-manager")
        .send()
    {
        return response.status().is_success();
    }

    false
}

/// Separate HORUS packages, pip packages, and cargo packages
///
/// # Arguments
/// * `deps` - Set of dependency strings from horus.yaml
/// * `context_language` - Optional language context ("rust", "python", "cpp") to guide auto-detection
pub(crate) fn split_dependencies_with_context(
    deps: HashSet<String>,
    context_language: Option<&str>,
) -> (Vec<String>, Vec<PipPackage>, Vec<CargoPackage>) {
    let mut horus_packages = Vec::new();
    let mut pip_packages = Vec::new();
    let mut cargo_packages = Vec::new();

    for dep in deps {
        let dep = dep.trim();

        // Check for explicit prefixes
        if let Some(pkg_str) = dep.strip_prefix("pip:") {
            match PipPackage::from_string(pkg_str) {
                Ok(pkg) => pip_packages.push(pkg),
                Err(e) => {
                    log::warn!("Failed to parse pip dependency '{}': {}", dep, e);
                    eprintln!(
                        "  {} Failed to parse pip dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                    eprintln!("     Syntax: pip:PACKAGE@VERSION or pip:PACKAGE");
                    eprintln!("     Example: pip:numpy@1.24.0");
                }
            }
            continue;
        }

        if let Some(pkg_str) = dep.strip_prefix("cargo:") {
            match CargoPackage::from_string(pkg_str) {
                Ok(pkg) => cargo_packages.push(pkg),
                Err(e) => {
                    log::warn!("Failed to parse cargo dependency '{}': {}", dep, e);
                    eprintln!(
                        "  {} Failed to parse cargo dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                    eprintln!("     Syntax: cargo:PACKAGE@VERSION:features=FEAT1,FEAT2");
                    eprintln!("     Examples:");
                    eprintln!("       - 'cargo:serde@1.0:features=derive'");
                    eprintln!("       - 'cargo:tokio@1.35:features=full,macros'");
                    eprintln!("       - cargo:rand@0.8");
                }
            }
            continue;
        }

        // Auto-detect: if starts with "horus"  HORUS registry
        if dep.starts_with("horus") {
            // Special handling for horus_py: ALWAYS map to 'horus-robotics' pip package
            // This is because horus_py is not a HORUS registry package - it's the Python bindings
            // installed via pip as 'horus-robotics'. This fixes Issue #25.
            if dep == "horus_py" || dep.starts_with("horus_py@") {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            // For bare "horus" in Python context, also map to horus-robotics pip package
            if context_language == Some("python") && dep == "horus" {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            horus_packages.push(dep.to_string());
            continue;
        }

        // Check if it's a known HORUS package using registry
        if is_horus_package(dep) {
            horus_packages.push(dep.to_string());
            continue;
        }

        // For unprefixed dependencies, use language context to determine type
        if let Some(lang) = context_language {
            match lang {
                "rust" => {
                    // Rust context: unprefixed deps are cargo packages
                    if let Ok(pkg) = CargoPackage::from_string(dep) {
                        cargo_packages.push(pkg);
                    }
                }
                "python" => {
                    // Python context: check if it's a known cargo crate first
                    let dep_name = if let Some(at_pos) = dep.find('@') {
                        &dep[..at_pos]
                    } else {
                        dep
                    };

                    // Skip Rust library crates when running Python code
                    if is_cargo_package(dep_name) {
                        // This is a Rust crate - skip it for Python runtime
                        continue;
                    }

                    // Otherwise treat as pip package
                    if let Ok(pkg) = PipPackage::from_string(dep) {
                        pip_packages.push(pkg);
                    }
                }
                _ => {
                    // Unknown context: fall back to old auto-detection
                    let dep_name = if let Some(at_pos) = dep.find('@') {
                        &dep[..at_pos]
                    } else {
                        dep
                    };

                    if is_cargo_package(dep_name) {
                        if let Ok(pkg) = CargoPackage::from_string(dep) {
                            cargo_packages.push(pkg);
                        }
                    } else if let Ok(pkg) = PipPackage::from_string(dep) {
                        pip_packages.push(pkg);
                    }
                }
            }
        } else {
            // No context: use old auto-detection behavior
            let dep_name = if let Some(at_pos) = dep.find('@') {
                &dep[..at_pos]
            } else {
                dep
            };

            if is_cargo_package(dep_name) {
                if let Ok(pkg) = CargoPackage::from_string(dep) {
                    cargo_packages.push(pkg);
                }
            } else if let Ok(pkg) = PipPackage::from_string(dep) {
                pip_packages.push(pkg);
            }
        }
    }

    (horus_packages, pip_packages, cargo_packages)
}

/// Git dependency reference type
#[derive(Debug, Clone)]
pub(crate) enum GitRef {
    Branch(String),
    Tag(String),
    Rev(String),
    Default,
}

/// Git package dependency
#[derive(Debug, Clone)]
pub(crate) struct GitPackage {
    pub(crate) name: String,
    pub(crate) url: String,
    pub(crate) git_ref: GitRef,
}

impl GitPackage {
    /// Parse from string format: git:pkg_name:url[:branch=X|tag=X|rev=X]
    pub(crate) fn from_string(s: &str) -> Option<Self> {
        let s = s.strip_prefix("git:")?;
        let parts: Vec<&str> = s.splitn(3, ':').collect();
        if parts.len() < 2 {
            return None;
        }

        let name = parts[0].to_string();
        let url_and_ref = if parts.len() == 3 {
            format!("{}:{}", parts[1], parts[2])
        } else {
            parts[1].to_string()
        };

        // Parse URL and optional ref
        let (url, git_ref) = if let Some(idx) = url_and_ref.find(":branch=") {
            let (url, rest) = url_and_ref.split_at(idx);
            let branch = &rest[":branch=".len()..];
            (url.to_string(), GitRef::Branch(branch.to_string()))
        } else if let Some(idx) = url_and_ref.find(":tag=") {
            let (url, rest) = url_and_ref.split_at(idx);
            let tag = &rest[":tag=".len()..];
            (url.to_string(), GitRef::Tag(tag.to_string()))
        } else if let Some(idx) = url_and_ref.find(":rev=") {
            let (url, rest) = url_and_ref.split_at(idx);
            let rev = &rest[":rev=".len()..];
            (url.to_string(), GitRef::Rev(rev.to_string()))
        } else {
            (url_and_ref, GitRef::Default)
        };

        Some(GitPackage { name, url, git_ref })
    }

    /// Get the cache directory name for this git package
    pub(crate) fn cache_dir_name(&self) -> String {
        // Create a unique cache directory based on URL and ref
        let url_hash = {
            use std::collections::hash_map::DefaultHasher;
            use std::hash::{Hash, Hasher};
            let mut hasher = DefaultHasher::new();
            self.url.hash(&mut hasher);
            format!("{:x}", hasher.finish())[..8].to_string()
        };

        let ref_suffix = match &self.git_ref {
            GitRef::Branch(b) => format!("_branch_{}", b),
            GitRef::Tag(t) => format!("_tag_{}", t),
            GitRef::Rev(r) => format!("_rev_{}", &r[..8.min(r.len())]),
            GitRef::Default => String::new(),
        };

        format!("git_{}_{}{}", self.name, url_hash, ref_suffix)
    }
}

/// Split dependencies including path and git dependencies
/// Returns: (horus_packages, pip_packages, cargo_packages, path_packages, git_packages)
/// path_packages is Vec<(name, path)>
pub(crate) type SplitDependencies = (
    Vec<String>,
    Vec<PipPackage>,
    Vec<CargoPackage>,
    Vec<(String, String)>,
    Vec<GitPackage>,
);

pub(crate) fn split_dependencies_with_path_context(
    deps: HashSet<String>,
    context_language: Option<&str>,
) -> SplitDependencies {
    let mut horus_packages = Vec::new();
    let mut pip_packages = Vec::new();
    let mut cargo_packages = Vec::new();
    let mut path_packages = Vec::new();
    let mut git_packages = Vec::new();

    for dep in deps {
        let dep = dep.trim();

        // Handle path dependencies: path:pkg_name:path_str
        if dep.starts_with("path:") {
            let parts: Vec<&str> = dep.splitn(3, ':').collect();
            if parts.len() == 3 {
                let pkg_name = parts[1].to_string();
                let pkg_path = parts[2].to_string();
                path_packages.push((pkg_name, pkg_path));
                continue;
            }
        }

        // Handle git dependencies: git:pkg_name:url[:branch=X|tag=X|rev=X]
        if dep.starts_with("git:") {
            if let Some(git_pkg) = GitPackage::from_string(dep) {
                git_packages.push(git_pkg);
                continue;
            } else {
                eprintln!(
                    "  {} Failed to parse git dependency '{}'",
                    "⚠".yellow(),
                    dep
                );
            }
        }

        // Check for explicit prefixes
        if let Some(pkg_str) = dep.strip_prefix("pip:") {
            match PipPackage::from_string(pkg_str) {
                Ok(pkg) => pip_packages.push(pkg),
                Err(e) => {
                    eprintln!(
                        "  {} Failed to parse pip dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                }
            }
            continue;
        }

        if let Some(pkg_str) = dep.strip_prefix("cargo:") {
            match CargoPackage::from_string(pkg_str) {
                Ok(pkg) => cargo_packages.push(pkg),
                Err(e) => {
                    eprintln!(
                        "  {} Failed to parse cargo dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                }
            }
            continue;
        }

        // Auto-detect: if starts with "horus"  HORUS registry
        if dep.starts_with("horus") {
            // Special handling for horus_py: ALWAYS map to 'horus-robotics' pip package
            // This is because horus_py is not a HORUS registry package - it's the Python bindings
            // installed via pip as 'horus-robotics'. This fixes Issue #25.
            if dep == "horus_py" || dep.starts_with("horus_py@") {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            // For bare "horus" in Python context, also map to horus-robotics pip package
            if context_language == Some("python") && dep == "horus" {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            horus_packages.push(dep.to_string());
            continue;
        }

        // Check if it's a known HORUS package using registry
        if is_horus_package(dep) {
            horus_packages.push(dep.to_string());
            continue;
        }

        // For unprefixed dependencies, use language context to determine type
        if let Some(lang) = context_language {
            match lang {
                "rust" => {
                    // Rust context: unprefixed deps are cargo packages
                    if let Ok(pkg) = CargoPackage::from_string(dep) {
                        cargo_packages.push(pkg);
                    }
                }
                "python" => {
                    // Python context: check if it's a known cargo crate first
                    let dep_name = if let Some(at_pos) = dep.find('@') {
                        &dep[..at_pos]
                    } else {
                        dep
                    };

                    // Skip Rust library crates when running Python code
                    if is_cargo_package(dep_name) {
                        // This is a Rust crate - skip it for Python runtime
                        continue;
                    }

                    // Otherwise treat as pip package
                    if let Ok(pkg) = PipPackage::from_string(dep) {
                        pip_packages.push(pkg);
                    }
                }
                _ => {}
            }
        }
    }

    (
        horus_packages,
        pip_packages,
        cargo_packages,
        path_packages,
        git_packages,
    )
}

/// Auto-create or update horus.yaml with detected dependencies
pub(crate) fn auto_update_horus_yaml(
    file: &Path,
    language: &str,
    dependencies: &HashSet<String>,
) -> Result<()> {
    let yaml_path = PathBuf::from(HORUS_YAML);

    if yaml_path.exists() {
        // Update existing horus.yaml
        update_existing_horus_yaml(&yaml_path, language, dependencies)?;
    } else {
        // Create new horus.yaml
        create_horus_yaml(&yaml_path, file, language, dependencies)?;
    }

    Ok(())
}

/// Create new horus.yaml from scratch
fn create_horus_yaml(
    yaml_path: &Path,
    file: &Path,
    language: &str,
    dependencies: &HashSet<String>,
) -> Result<()> {
    // Derive project name from directory or file
    let project_name = std::env::current_dir()
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().to_string()))
        .unwrap_or_else(|| {
            file.file_stem()
                .and_then(|n| n.to_str())
                .unwrap_or("my-project")
                .to_string()
        });

    // Categorize dependencies based on language context
    let mut horus_deps = Vec::new();
    let mut pip_deps = Vec::new();
    let mut cargo_deps = Vec::new();

    for dep in dependencies {
        if dep.starts_with("horus") {
            horus_deps.push(dep.clone());
        } else {
            // Default based on language
            match language {
                "python" => pip_deps.push(format!("pip:{}", dep)),
                "rust" => cargo_deps.push(format!("cargo:{}", dep)),
                _ => pip_deps.push(format!("pip:{}", dep)), // Default to pip
            }
        }
    }

    // Sort for consistent output
    horus_deps.sort();
    pip_deps.sort();
    cargo_deps.sort();

    // Build YAML content
    let mut content = String::new();
    content.push_str(&format!("name: {}\n", project_name));
    content.push_str("version: 0.1.9\n");
    content.push_str(&format!("language: {}\n", language));
    content.push_str("\ndependencies:\n");

    // Add HORUS packages first
    for dep in horus_deps {
        content.push_str(&format!("  - {}\n", dep));
    }

    // Add pip packages
    for dep in pip_deps {
        content.push_str(&format!("  - {}\n", dep));
    }

    // Add cargo packages
    for dep in cargo_deps {
        content.push_str(&format!("  - {}\n", dep));
    }

    // Write file
    fs::write(yaml_path, content)?;

    eprintln!(
        "  {} Created horus.yaml with {} dependencies",
        "".green(),
        dependencies.len()
    );

    Ok(())
}

/// Update existing horus.yaml with new dependencies
fn update_existing_horus_yaml(
    yaml_path: &Path,
    language: &str,
    new_dependencies: &HashSet<String>,
) -> Result<()> {
    // Parse existing yaml to get current dependencies
    let existing_content = fs::read_to_string(yaml_path)?;
    let existing_deps = parse_horus_yaml_dependencies_from_content(&existing_content)?;

    // Find new dependencies not in existing
    let mut added = Vec::new();
    for dep in new_dependencies {
        let dep_str = if dep.starts_with("horus") {
            dep.clone()
        } else {
            // Categorize based on language context
            match language {
                "python" => format!("pip:{}", dep),
                "rust" => format!("cargo:{}", dep),
                _ => format!("pip:{}", dep), // Default to pip
            }
        };

        // Check if dependency already exists (in any form)
        let base_name = dep_str
            .split(':')
            .next_back()
            .unwrap_or(&dep_str)
            .split('@')
            .next()
            .unwrap_or(&dep_str);

        let already_exists = existing_deps.iter().any(|e| {
            let e_base = e
                .split(':')
                .next_back()
                .unwrap_or(e)
                .split('@')
                .next()
                .unwrap_or(e);
            e_base == base_name
        });

        if !already_exists {
            added.push(dep_str);
        }
    }

    if added.is_empty() {
        return Ok(()); // No new dependencies
    }

    // Append new dependencies to file
    let mut content = existing_content;
    if !content.ends_with('\n') {
        content.push('\n');
    }

    for dep in &added {
        content.push_str(&format!("  - {}\n", dep));
    }

    fs::write(yaml_path, content)?;

    eprintln!(
        "  {} Added {} new dependencies to horus.yaml",
        "".green(),
        added.len()
    );
    for dep in &added {
        eprintln!("     + {}", dep);
    }

    Ok(())
}

/// Parse dependencies from YAML content string
fn parse_horus_yaml_dependencies_from_content(content: &str) -> Result<HashSet<String>> {
    let mut dependencies = HashSet::new();
    let mut in_deps = false;

    for line in content.lines() {
        if line.trim() == "dependencies:" {
            in_deps = true;
            continue;
        }

        if in_deps {
            if line.starts_with("  -") {
                let dep = line.trim_start_matches("  -").trim();
                if !dep.is_empty() && !dep.starts_with('#') {
                    dependencies.insert(dep.to_string());
                }
            } else if !line.starts_with("  ") && !line.trim().is_empty() {
                // End of dependencies section
                in_deps = false;
            }
        }
    }

    Ok(dependencies)
}
