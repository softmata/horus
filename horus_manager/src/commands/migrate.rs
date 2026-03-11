//! `horus migrate` — convert existing projects to unified horus.toml format.
//!
//! Reads root Cargo.toml/pyproject.toml, extracts dependencies,
//! writes them into horus.toml [dependencies], optionally moves/removes
//! old native build files.

use anyhow::{Context, Result};
use colored::*;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use crate::manifest::{
    DepSource, DependencyValue, DetailedDependency, HorusManifest, HORUS_TOML,
};

/// Run `horus migrate`.
///
/// - `dry_run`: Show what would change without modifying.
/// - `force`: Skip confirmation prompts.
pub fn run_migrate(dry_run: bool, _force: bool) -> Result<()> {
    let cwd = std::env::current_dir()?;

    // Check if horus.toml exists
    let has_horus_toml = cwd.join(HORUS_TOML).exists();
    if !has_horus_toml {
        anyhow::bail!(
            "No horus.toml found. Run {} first.",
            "horus init".cyan()
        );
    }

    let (mut manifest, _, _) = HorusManifest::find_and_load()?;
    let mut changes = Vec::new();

    // ── Parse Cargo.toml ─────────────────────────────────────────────────
    let cargo_path = cwd.join("Cargo.toml");
    if cargo_path.exists() {
        let cargo_deps = extract_cargo_deps(&cargo_path)?;
        if !cargo_deps.is_empty() {
            changes.push(format!(
                "Import {} dependencies from Cargo.toml",
                cargo_deps.len()
            ));
            for (name, _dep) in &cargo_deps {
                changes.push(format!("  + {} (crates.io)", name));
            }
            if !dry_run {
                manifest.dependencies.extend(cargo_deps);
            }
        }
    }

    // ── Parse pyproject.toml ─────────────────────────────────────────────
    let pyproject_path = cwd.join("pyproject.toml");
    if pyproject_path.exists() {
        let pypi_deps = extract_pyproject_deps(&pyproject_path)?;
        if !pypi_deps.is_empty() {
            changes.push(format!(
                "Import {} dependencies from pyproject.toml",
                pypi_deps.len()
            ));
            for (name, _dep) in &pypi_deps {
                changes.push(format!("  + {} (pypi)", name));
            }
            if !dry_run {
                manifest.dependencies.extend(pypi_deps);
            }
        }
    }

    // ── Move source files ────────────────────────────────────────────────
    let src_main = cwd.join("src/main.rs");
    let root_main = cwd.join("main.rs");
    if src_main.exists() && !root_main.exists() {
        // Check if src/ only contains main.rs
        let src_count = fs::read_dir(cwd.join("src"))
            .map(|e| e.count())
            .unwrap_or(0);
        if src_count == 1 {
            changes.push("Move src/main.rs → main.rs".to_string());
            if !dry_run {
                fs::rename(&src_main, &root_main)?;
                // Remove empty src/
                let _ = fs::remove_dir(cwd.join("src"));
            }
        }
    }

    // ── Backup and remove old build files ────────────────────────────────
    let backup_dir = cwd.join(".horus/backup");
    if cargo_path.exists() {
        changes.push("Backup Cargo.toml → .horus/backup/Cargo.toml".to_string());
        if !dry_run {
            fs::create_dir_all(&backup_dir)?;
            fs::rename(&cargo_path, backup_dir.join("Cargo.toml"))?;
        }
    }
    if pyproject_path.exists() {
        changes.push("Backup pyproject.toml → .horus/backup/pyproject.toml".to_string());
        if !dry_run {
            fs::create_dir_all(&backup_dir)?;
            fs::rename(&pyproject_path, backup_dir.join("pyproject.toml"))?;
        }
    }

    // ── Report ───────────────────────────────────────────────────────────
    if changes.is_empty() {
        println!("{} Nothing to migrate. Project is already in unified format.", "✓".green());
        return Ok(());
    }

    if dry_run {
        println!("{}", "Dry run — these changes would be made:".yellow().bold());
    } else {
        println!("{}", "Migrating project:".bold());
    }

    for change in &changes {
        println!("  {}", change);
    }

    if !dry_run {
        // Save updated manifest
        manifest.save_to(&cwd.join(HORUS_TOML))?;
        println!();
        println!("{} Migration complete. Old files backed up to .horus/backup/", "✓".green());
        println!("  Run {} to verify the project builds.", "horus build".cyan());
    }

    Ok(())
}

/// Extract dependencies from a Cargo.toml file.
fn extract_cargo_deps(path: &Path) -> Result<BTreeMap<String, DependencyValue>> {
    let content = fs::read_to_string(path).context("Failed to read Cargo.toml")?;
    let doc = content
        .parse::<toml_edit::DocumentMut>()
        .context("Failed to parse Cargo.toml")?;

    let mut deps = BTreeMap::new();

    if let Some(dep_table) = doc.get("dependencies").and_then(|d| d.as_table()) {
        for (name, item) in dep_table.iter() {
            // Skip horus internal deps
            if name.starts_with("horus") {
                continue;
            }

            let dep = match item {
                toml_edit::Item::Value(toml_edit::Value::String(s)) => {
                    DependencyValue::Detailed(DetailedDependency {
                        version: Some(s.value().to_string()),
                        source: Some(DepSource::CratesIo),
                        features: vec![],
                        optional: false,
                        path: None,
                        git: None,
                        branch: None,
                        tag: None,
                        rev: None,
                    })
                }
                toml_edit::Item::Value(toml_edit::Value::InlineTable(t)) => {
                    parse_cargo_inline_dep(t)
                }
                toml_edit::Item::Table(t) => parse_cargo_table_dep(t),
                _ => continue,
            };
            deps.insert(name.to_string(), dep);
        }
    }

    Ok(deps)
}

fn parse_cargo_inline_dep(t: &toml_edit::InlineTable) -> DependencyValue {
    let version = t
        .get("version")
        .and_then(|v| v.as_str())
        .map(String::from);
    let features: Vec<String> = t
        .get("features")
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| v.as_str().map(String::from))
                .collect()
        })
        .unwrap_or_default();
    let optional = t
        .get("optional")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let path = t.get("path").and_then(|v| v.as_str()).map(String::from);
    let git = t.get("git").and_then(|v| v.as_str()).map(String::from);
    let branch = t.get("branch").and_then(|v| v.as_str()).map(String::from);

    let source = if path.is_some() {
        Some(DepSource::Path)
    } else if git.is_some() {
        Some(DepSource::Git)
    } else {
        Some(DepSource::CratesIo)
    };

    DependencyValue::Detailed(DetailedDependency {
        version,
        source,
        features,
        optional,
        path,
        git,
        branch,
        tag: None,
        rev: None,
    })
}

fn parse_cargo_table_dep(t: &toml_edit::Table) -> DependencyValue {
    let version = t
        .get("version")
        .and_then(|v| v.as_str())
        .map(String::from);
    let features: Vec<String> = t
        .get("features")
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| v.as_str().map(String::from))
                .collect()
        })
        .unwrap_or_default();
    let optional = t
        .get("optional")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let path = t.get("path").and_then(|v| v.as_str()).map(String::from);
    let git = t.get("git").and_then(|v| v.as_str()).map(String::from);
    let branch = t.get("branch").and_then(|v| v.as_str()).map(String::from);

    let source = if path.is_some() {
        Some(DepSource::Path)
    } else if git.is_some() {
        Some(DepSource::Git)
    } else {
        Some(DepSource::CratesIo)
    };

    DependencyValue::Detailed(DetailedDependency {
        version,
        source,
        features,
        optional,
        path,
        git,
        branch,
        tag: None,
        rev: None,
    })
}

/// Extract dependencies from a pyproject.toml file.
fn extract_pyproject_deps(path: &Path) -> Result<BTreeMap<String, DependencyValue>> {
    let content = fs::read_to_string(path).context("Failed to read pyproject.toml")?;
    let doc = content
        .parse::<toml_edit::DocumentMut>()
        .context("Failed to parse pyproject.toml")?;

    let mut deps = BTreeMap::new();

    // PEP 621 format: [project] dependencies = ["numpy>=1.24", "requests"]
    if let Some(project) = doc.get("project").and_then(|p| p.as_table()) {
        if let Some(dep_array) = project.get("dependencies").and_then(|d| d.as_array()) {
            for item in dep_array.iter() {
                if let Some(spec) = item.as_str() {
                    let (name, version) = parse_pep_dep(spec);
                    // Skip horus internal packages
                    if name.starts_with("horus") {
                        continue;
                    }
                    deps.insert(
                        name,
                        DependencyValue::Detailed(DetailedDependency {
                            version,
                            source: Some(DepSource::PyPI),
                            features: vec![],
                            optional: false,
                            path: None,
                            git: None,
                            branch: None,
                            tag: None,
                            rev: None,
                        }),
                    );
                }
            }
        }
    }

    Ok(deps)
}

/// Parse a PEP 508 dependency string like "numpy>=1.24" into (name, version).
fn parse_pep_dep(spec: &str) -> (String, Option<String>) {
    let spec = spec.trim();
    // Find first version operator character
    let split_at = spec
        .find(|c: char| c == '>' || c == '<' || c == '=' || c == '!' || c == '~')
        .unwrap_or(spec.len());

    let name = spec[..split_at].trim().to_string();
    let version = if split_at < spec.len() {
        Some(spec[split_at..].trim().to_string())
    } else {
        None
    };

    (name, version)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_pep_dep_with_version() {
        let (name, ver) = parse_pep_dep("numpy>=1.24");
        assert_eq!(name, "numpy");
        assert_eq!(ver, Some(">=1.24".to_string()));
    }

    #[test]
    fn parse_pep_dep_without_version() {
        let (name, ver) = parse_pep_dep("requests");
        assert_eq!(name, "requests");
        assert!(ver.is_none());
    }

    #[test]
    fn parse_pep_dep_complex() {
        let (name, ver) = parse_pep_dep("torch>=2.0,<3.0");
        assert_eq!(name, "torch");
        assert_eq!(ver, Some(">=2.0,<3.0".to_string()));
    }
}
