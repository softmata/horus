use crate::manifest::HORUS_TOML;
use anyhow::{anyhow, bail, Result};
use colored::*;
use std::collections::HashSet;
use std::fs;
use std::path::Path;

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
}

impl CargoPackage {
    pub(crate) fn from_string(s: &str) -> Result<Self> {
        // Parse formats: "bat@0.24.0:features=derive,serde" or "bat@0.24.0" or "bat"
        let s = s.trim();

        // Strip and validate optional features suffix (not stored, but syntax is checked)
        let pkg_part = if let Some(colon_pos) = s.find(':') {
            let pkg = &s[..colon_pos];
            let features_part = &s[colon_pos + 1..];

            if let Some(equals_pos) = features_part.find('=') {
                let features_str = &features_part[equals_pos + 1..].trim();
                if features_str.is_empty() {
                    return Err(anyhow!(
                        "Empty features list (remove ':features=' or provide feature names)"
                    ));
                }
            } else if !features_part.is_empty() {
                return Err(anyhow!(
                    "Invalid features syntax '{}' - use 'features=feat1,feat2'",
                    features_part
                ));
            }
            pkg
        } else {
            s
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
            return Ok(CargoPackage { name, version });
        }

        // No version specified
        let name = pkg_part.trim().to_string();
        if name.is_empty() {
            return Err(anyhow!("Package name cannot be empty"));
        }

        Ok(CargoPackage {
            name,
            version: None,
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
            "-".cyan(),
            ".rs (Rust), .py (Python)".green(),
            "-".cyan(),
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

    // Check if horus.toml exists — but deps now live in native build files,
    // so we just scan the source code for imports regardless.
    if Path::new(HORUS_TOML).exists() {
        log::debug!("horus.toml exists (deps now in native build files)");
    }

    // Scan imports from source code
    match language {
        "rust" => {
            // Scan for: use horus::*, use horus_library::*, extern crate
            for line in content.lines() {
                if let Some(dep) = parse_rust_import(line) {
                    dependencies.insert(dep);
                }
            }

            // Also check Cargo.toml if exists
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

    // Filter out ignored packages
    dependencies.retain(|dep| !ignore.should_ignore_package(dep));

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

/// Check if a package name matches a known Cargo CLI tool.
///
/// This only checks a hardcoded list of well-known cargo-installable binaries.
/// It does NOT make HTTP requests to crates.io — that was removed because it
/// caused false positives (e.g. `numpy`, `requests`, `flask` all exist as
/// crate names on crates.io, causing them to be silently skipped in Python
/// context). If a user wants a cargo binary not in this list, they should
/// use the explicit `cargo:` prefix.
pub(crate) fn is_cargo_package(package_name: &str) -> bool {
    // Well-known CLI tools installable via `cargo install`
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
        "cargo-watch",
        "cargo-expand",
        "cargo-edit",
        "cargo-outdated",
        "cargo-audit",
    ];

    common_cli_tools.contains(&package_name)
}

/// Separate HORUS packages, pip packages, and cargo packages
///
/// # Arguments
/// * `deps` - Set of dependency strings
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
                    // Python context: all non-horus deps are pip packages.
                    // We intentionally do NOT call is_cargo_package() here —
                    // many Python packages (numpy, requests, flask, etc.) have
                    // same-named crates on crates.io, which caused false-positive
                    // skipping. If a user wants a cargo binary, they should use
                    // the explicit `cargo:` prefix.
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

#[cfg(test)]
mod tests {
    use super::*;

    // ── PipPackage ─────────────────────────────────────────────────────────

    #[test]
    fn pip_package_simple_name() {
        let pkg = PipPackage::from_string("numpy").unwrap();
        assert_eq!(pkg.name, "numpy");
        assert!(pkg.version.is_none());
        assert_eq!(pkg.requirement_string(), "numpy");
    }

    #[test]
    fn pip_package_with_gte_version() {
        let pkg = PipPackage::from_string("numpy>=1.24.0").unwrap();
        assert_eq!(pkg.name, "numpy");
        assert_eq!(pkg.version, Some(">=1.24.0".to_string()));
        assert_eq!(pkg.requirement_string(), "numpy>=1.24.0");
    }

    #[test]
    fn pip_package_with_exact_version() {
        let pkg = PipPackage::from_string("requests==2.31.0").unwrap();
        assert_eq!(pkg.name, "requests");
        assert_eq!(pkg.version, Some("==2.31.0".to_string()));
    }

    #[test]
    fn pip_package_with_tilde_version() {
        let pkg = PipPackage::from_string("flask~=2.3").unwrap();
        assert_eq!(pkg.name, "flask");
        assert_eq!(pkg.version, Some("~=2.3".to_string()));
    }

    #[test]
    fn pip_package_with_at_version() {
        let pkg = PipPackage::from_string("torch@2.0.1").unwrap();
        assert_eq!(pkg.name, "torch");
        assert_eq!(pkg.version, Some("==2.0.1".to_string()));
    }

    #[test]
    fn pip_package_at_no_version() {
        let pkg = PipPackage::from_string("torch@").unwrap();
        assert_eq!(pkg.name, "torch");
        assert!(pkg.version.is_none());
    }

    // ── CargoPackage ───────────────────────────────────────────────────────

    #[test]
    fn cargo_package_simple() {
        let pkg = CargoPackage::from_string("serde").unwrap();
        assert_eq!(pkg.name, "serde");
        assert!(pkg.version.is_none());
    }

    #[test]
    fn cargo_package_with_version() {
        let pkg = CargoPackage::from_string("serde@1.0.193").unwrap();
        assert_eq!(pkg.name, "serde");
        assert_eq!(pkg.version, Some("1.0.193".to_string()));
    }

    #[test]
    fn cargo_package_with_features() {
        let pkg = CargoPackage::from_string("serde@1.0:features=derive").unwrap();
        assert_eq!(pkg.name, "serde");
        assert_eq!(pkg.version, Some("1.0".to_string()));
    }

    #[test]
    fn cargo_package_empty_name_fails() {
        CargoPackage::from_string("").unwrap_err();
    }

    #[test]
    fn cargo_package_empty_features_fails() {
        CargoPackage::from_string("serde@1.0:features=").unwrap_err();
    }

    #[test]
    fn cargo_package_latest_version_fails() {
        CargoPackage::from_string("serde@latest").unwrap_err();
    }

    #[test]
    fn cargo_package_whitespace_version_fails() {
        CargoPackage::from_string("serde@1.0 beta").unwrap_err();
    }

    // ── detect_language ────────────────────────────────────────────────────

    #[test]
    fn detect_language_rust() {
        assert_eq!(detect_language(Path::new("main.rs")).unwrap(), "rust");
    }

    #[test]
    fn detect_language_python() {
        assert_eq!(detect_language(Path::new("main.py")).unwrap(), "python");
    }

    #[test]
    fn detect_language_unsupported() {
        detect_language(Path::new("main.cpp")).unwrap_err();
        detect_language(Path::new("main.js")).unwrap_err();
        detect_language(Path::new("noext")).unwrap_err();
    }

    // ── parse_rust_import ──────────────────────────────────────────────────

    #[test]
    fn parse_rust_import_use() {
        assert_eq!(
            parse_rust_import("use horus_core::Node;"),
            Some("horus_core".to_string())
        );
    }

    #[test]
    fn parse_rust_import_extern() {
        assert_eq!(
            parse_rust_import("extern crate horus_library;"),
            Some("horus_library".to_string())
        );
    }

    #[test]
    fn parse_rust_import_non_horus() {
        assert!(parse_rust_import("use serde::Serialize;").is_none());
        assert!(parse_rust_import("use std::io;").is_none());
    }

    #[test]
    fn parse_rust_import_comment_line() {
        assert!(parse_rust_import("// use horus_core::Node;").is_none());
    }

    // ── parse_all_python_imports ───────────────────────────────────────────

    #[test]
    fn parse_python_import() {
        assert_eq!(
            parse_all_python_imports("import numpy"),
            Some("numpy".to_string())
        );
    }

    #[test]
    fn parse_python_import_as() {
        assert_eq!(
            parse_all_python_imports("import numpy as np"),
            Some("numpy".to_string())
        );
    }

    #[test]
    fn parse_python_from_import() {
        assert_eq!(
            parse_all_python_imports("from torch import nn"),
            Some("torch".to_string())
        );
    }

    #[test]
    fn parse_python_stdlib_skipped() {
        assert!(parse_all_python_imports("import os").is_none());
        assert!(parse_all_python_imports("import sys").is_none());
        assert!(parse_all_python_imports("from pathlib import Path").is_none());
    }

    #[test]
    fn parse_python_comment_skipped() {
        assert!(parse_all_python_imports("# import numpy").is_none());
    }

    // ── is_stdlib_package ──────────────────────────────────────────────────

    #[test]
    fn stdlib_packages() {
        assert!(is_stdlib_package("os"));
        assert!(is_stdlib_package("sys"));
        assert!(is_stdlib_package("json"));
        assert!(is_stdlib_package("dataclasses"));
    }

    #[test]
    fn non_stdlib_packages() {
        assert!(!is_stdlib_package("numpy"));
        assert!(!is_stdlib_package("torch"));
        assert!(!is_stdlib_package("horus"));
    }

    // ── is_horus_package ───────────────────────────────────────────────────

    #[test]
    fn horus_package_detection() {
        assert!(is_horus_package("horus"));
        assert!(is_horus_package("horus_core"));
        assert!(is_horus_package("horus_library"));
        assert!(!is_horus_package("serde"));
        assert!(!is_horus_package("numpy"));
    }

    // ── is_cargo_package ───────────────────────────────────────────────────

    #[test]
    fn cargo_cli_tools() {
        assert!(is_cargo_package("bat"));
        assert!(is_cargo_package("ripgrep"));
        assert!(is_cargo_package("cargo-watch"));
    }

    #[test]
    fn non_cargo_packages() {
        assert!(!is_cargo_package("numpy"));
        assert!(!is_cargo_package("flask"));
        assert!(!is_cargo_package("random_name_xyz"));
    }

    // ── split_dependencies_with_context ────────────────────────────────────

    #[test]
    fn split_deps_explicit_prefixes() {
        let deps: HashSet<String> = [
            "pip:numpy>=1.24".to_string(),
            "cargo:serde@1.0".to_string(),
            "horus_core".to_string(),
        ]
        .into_iter()
        .collect();

        let (horus, pip, cargo) = split_dependencies_with_context(deps, None);
        assert_eq!(horus.len(), 1);
        assert_eq!(pip.len(), 1);
        assert_eq!(cargo.len(), 1);
        assert_eq!(pip[0].name, "numpy");
        assert_eq!(cargo[0].name, "serde");
    }

    #[test]
    fn split_deps_python_context() {
        let deps: HashSet<String> = ["numpy".to_string(), "requests".to_string()]
            .into_iter()
            .collect();

        let (horus, pip, cargo) = split_dependencies_with_context(deps, Some("python"));
        assert!(horus.is_empty());
        assert_eq!(pip.len(), 2);
        assert!(cargo.is_empty());
    }

    #[test]
    fn split_deps_rust_context() {
        let deps: HashSet<String> = ["serde".to_string(), "tokio".to_string()]
            .into_iter()
            .collect();

        let (horus, pip, cargo) = split_dependencies_with_context(deps, Some("rust"));
        assert!(horus.is_empty());
        assert!(pip.is_empty());
        assert_eq!(cargo.len(), 2);
    }

    #[test]
    fn split_deps_horus_py_maps_to_pip() {
        let deps: HashSet<String> = ["horus_py".to_string()].into_iter().collect();
        let (horus, pip, _cargo) = split_dependencies_with_context(deps, Some("python"));
        assert!(horus.is_empty());
        assert_eq!(pip.len(), 1);
        assert_eq!(pip[0].name, "horus-robotics");
    }

    #[test]
    fn split_deps_bare_horus_in_python() {
        let deps: HashSet<String> = ["horus".to_string()].into_iter().collect();
        let (horus, pip, _cargo) = split_dependencies_with_context(deps, Some("python"));
        assert!(horus.is_empty());
        assert_eq!(pip.len(), 1);
        assert_eq!(pip[0].name, "horus-robotics");
    }

    #[test]
    fn split_deps_bare_horus_in_rust() {
        let deps: HashSet<String> = ["horus".to_string()].into_iter().collect();
        let (horus, pip, _cargo) = split_dependencies_with_context(deps, Some("rust"));
        // In Rust context, "horus" is a horus registry package
        assert_eq!(horus.len(), 1);
        assert!(pip.is_empty());
    }
}
