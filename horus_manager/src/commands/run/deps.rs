use crate::manifest::{DependencyValue, DetailedDependency, DepSource, HorusManifest, HORUS_TOML};
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

    // First, check if horus.toml exists and use it
    let from_manifest = Path::new(HORUS_TOML).exists();

    if from_manifest {
        log::debug!("Reading dependencies from horus.toml");
        eprintln!("  {} Reading dependencies from horus.toml", "".cyan());
        if let Ok((manifest, _)) = HorusManifest::load_from(Path::new(HORUS_TOML)) {
            dependencies.extend(manifest_deps_to_string_set(&manifest));
        }
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

    // Auto-create horus.toml if we scanned from source and none exists
    if !from_manifest && !dependencies.is_empty() {
        auto_create_horus_toml(file, language, &dependencies)?;
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

/// Convert a manifest dependency to a Cargo.toml dependency line.
///
/// Returns `None` for horus packages (already added as path deps) and pip-only packages.
/// Returns a string like `serde = "1.0"` or `serde = { version = "1.0", features = ["derive"] }`.
pub(crate) fn manifest_dep_to_cargo_line(name: &str, value: &DependencyValue) -> Option<String> {
    // Skip horus packages — they're already added as path dependencies
    if name == "horus" || name.starts_with("horus_") {
        return None;
    }

    match value {
        DependencyValue::Simple(version_str) => {
            let v = if version_str == "latest" { "*" } else { version_str.as_str() };
            Some(format!("{} = \"{}\"", name, v))
        }
        DependencyValue::Detailed(detail) => {
            // Skip pip-only deps
            if matches!(detail.source, Some(DepSource::Pypi)) {
                return None;
            }
            // Skip system deps
            if matches!(detail.source, Some(DepSource::System)) {
                return None;
            }

            // Path dependency
            if let Some(ref path) = detail.path {
                return Some(format!("{} = {{ path = \"{}\" }}", name, path));
            }

            // Git dependency
            if let Some(ref git) = detail.git {
                let mut parts = vec![format!("git = \"{}\"", git)];
                if let Some(ref branch) = detail.branch {
                    parts.push(format!("branch = \"{}\"", branch));
                }
                if let Some(ref tag) = detail.tag {
                    parts.push(format!("tag = \"{}\"", tag));
                }
                if let Some(ref rev) = detail.rev {
                    parts.push(format!("rev = \"{}\"", rev));
                }
                return Some(format!("{} = {{ {} }}", name, parts.join(", ")));
            }

            // Version-based dependency
            let version = detail.version.as_deref().unwrap_or("*");
            if detail.features.is_empty() {
                Some(format!("{} = \"{}\"", name, version))
            } else {
                let features_str = detail
                    .features
                    .iter()
                    .map(|f| format!("\"{}\"", f))
                    .collect::<Vec<_>>()
                    .join(", ");
                Some(format!(
                    "{} = {{ version = \"{}\", features = [{}] }}",
                    name, version, features_str
                ))
            }
        }
    }
}

/// Convert manifest dependencies to a set of dependency strings (for compatibility
/// with existing code that expects `HashSet<String>`).
///
/// Produces strings in the old format: `"horus"`, `"pip:numpy@^1.24"`, `"cargo:serde@1.0"`, etc.
pub(crate) fn manifest_deps_to_string_set(manifest: &HorusManifest) -> HashSet<String> {
    let mut deps = HashSet::new();
    for (name, value) in &manifest.dependencies {
        match value {
            DependencyValue::Simple(version) => {
                if version == "*" || version == "latest" {
                    deps.insert(name.clone());
                } else {
                    deps.insert(format!("{}@{}", name, version));
                }
            }
            DependencyValue::Detailed(detail) => {
                let version = detail.version.as_deref().unwrap_or("*");
                let prefix = match &detail.source {
                    Some(DepSource::Pypi) => "pip:",
                    Some(DepSource::CratesIo) => "cargo:",
                    Some(DepSource::System) => "system:",
                    _ => "",
                };
                if version == "*" {
                    deps.insert(format!("{}{}", prefix, name));
                } else {
                    deps.insert(format!("{}{}@{}", prefix, name, version));
                }
            }
        }
    }
    deps
}

/// Append all cargo-compatible dependencies from a manifest to a Cargo.toml string.
///
/// Skips horus framework packages and pip/system-only deps.
pub(crate) fn append_manifest_cargo_deps(
    cargo_toml: &mut String,
    manifest: &HorusManifest,
    skip_names: &HashSet<String>,
) {
    for (name, value) in &manifest.dependencies {
        if skip_names.contains(name) {
            continue;
        }
        if let Some(line) = manifest_dep_to_cargo_line(name, value) {
            cargo_toml.push_str(&format!("{}\n", line));
        }
    }
}

/// Auto-create horus.toml when none exists and dependencies were detected from source.
pub(crate) fn auto_create_horus_toml(
    file: &Path,
    language: &str,
    dependencies: &HashSet<String>,
) -> Result<()> {
    use crate::manifest::{PackageInfo, IgnoreConfig};
    use std::collections::BTreeMap;

    let toml_path = PathBuf::from(HORUS_TOML);
    if toml_path.exists() {
        // If manifest exists, add missing deps via toml_edit
        let (existing, _) = HorusManifest::load_from(&toml_path)?;
        let mut added = 0;
        for dep in dependencies {
            let base_name = dep.split('@').next().unwrap_or(dep);
            let base_name = base_name.strip_prefix("pip:").or_else(|| base_name.strip_prefix("cargo:")).unwrap_or(base_name);
            if !existing.dependencies.contains_key(base_name) {
                let value = DependencyValue::Simple("*".to_string());
                crate::manifest::add_dependency_to_file(&toml_path, base_name, &value, "dependencies")?;
                added += 1;
            }
        }
        if added > 0 {
            eprintln!("  {} Added {} new dependencies to horus.toml", "\u{2713}".green(), added);
        }
        return Ok(());
    }

    // Create new horus.toml
    let project_name = std::env::current_dir()
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().to_string()))
        .unwrap_or_else(|| {
            file.file_stem()
                .and_then(|n| n.to_str())
                .unwrap_or("my-project")
                .to_string()
        });

    let mut deps = BTreeMap::new();
    for dep in dependencies {
        let (name, source) = if let Some(rest) = dep.strip_prefix("pip:") {
            let (n, v) = split_dep_spec(rest);
            (n, Some((DepSource::Pypi, v)))
        } else if let Some(rest) = dep.strip_prefix("cargo:") {
            let (n, v) = split_dep_spec(rest);
            (n, Some((DepSource::CratesIo, v)))
        } else {
            let (n, v) = split_dep_spec(dep);
            (n, if v == "*" { None } else { Some((DepSource::Registry, v.to_string())) })
        };

        let value = match source {
            Some((src, v)) => DependencyValue::Detailed(DetailedDependency {
                version: Some(v.to_string()),
                source: Some(src),
                path: None, git: None, branch: None, tag: None, rev: None,
                features: vec![], target: None,
            }),
            None => DependencyValue::Simple("*".to_string()),
        };
        deps.insert(name.to_string(), value);
    }

    let manifest = HorusManifest {
        package: PackageInfo {
            name: project_name,
            version: "0.1.0".to_string(),
            description: None,
            authors: vec![],
            license: None,
            language: Some(language.to_string()),
            edition: "1".to_string(),
            repository: None,
            package_type: None,
            categories: vec![],
        },
        dependencies: deps,
        dev_dependencies: BTreeMap::new(),
        build_dependencies: BTreeMap::new(),
        drivers: BTreeMap::new(),
        ignore: IgnoreConfig::default(),
        enable: Vec::new(),
    };

    manifest.save_to(&toml_path)?;
    eprintln!(
        "  {} Created horus.toml with {} dependencies",
        "\u{2713}".green(),
        dependencies.len()
    );

    Ok(())
}

/// Split "name@version" into (name, version) or (name, "*")
fn split_dep_spec(s: &str) -> (&str, String) {
    if let Some(pos) = s.find('@') {
        let v = &s[pos + 1..];
        (&s[..pos], if v == "latest" { "*".to_string() } else { v.to_string() })
    } else {
        (s, "*".to_string())
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
/// * `deps` - Set of dependency strings from horus.toml
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
                    crate::cli_output::ICON_WARN.yellow(),
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

