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

use crate::manifest::{DepSource, DependencyValue, DetailedDependency, HorusManifest, HORUS_TOML};

/// Run `horus migrate`.
///
/// - `dry_run`: Show what would change without modifying.
/// - `force`: Skip confirmation prompts.
pub fn run_migrate(dry_run: bool, _force: bool) -> Result<()> {
    let cwd = std::env::current_dir()?;

    // Check if horus.toml exists
    let has_horus_toml = cwd.join(HORUS_TOML).exists();
    if !has_horus_toml {
        anyhow::bail!("No horus.toml found. Run {} first.", "horus init".cyan());
    }

    let (mut manifest, _) = HorusManifest::find_and_load()?;
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

    // ── Parse CMakeLists.txt ─────────────────────────────────────────────
    let cmake_path = cwd.join("CMakeLists.txt");
    if cmake_path.exists() {
        let cmake_deps = extract_cmake_deps(&cmake_path)?;
        if !cmake_deps.is_empty() {
            changes.push(format!(
                "Import {} dependencies from CMakeLists.txt",
                cmake_deps.len()
            ));
            for (name, _dep) in &cmake_deps {
                changes.push(format!("  + {} (system)", name));
            }
            if !dry_run {
                manifest.dependencies.extend(cmake_deps);
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
    if cmake_path.exists() {
        changes.push("Backup CMakeLists.txt → .horus/backup/CMakeLists.txt".to_string());
        if !dry_run {
            fs::create_dir_all(&backup_dir)?;
            fs::rename(&cmake_path, backup_dir.join("CMakeLists.txt"))?;
        }
    }

    // ── Report ───────────────────────────────────────────────────────────
    if changes.is_empty() {
        println!(
            "{} Nothing to migrate. Project is already in unified format.",
            "✓".green()
        );
        return Ok(());
    }

    if dry_run {
        println!(
            "{}",
            "Dry run — these changes would be made:".yellow().bold()
        );
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
        println!("{} Migration complete. Old files backed up.", "✓".green());
        println!(
            "  Run {} to verify the project builds.",
            "horus build".cyan()
        );
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
                        apt: None,
                        cmake_package: None,
                        lang: None,
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
    let version = t.get("version").and_then(|v| v.as_str()).map(String::from);
    let features: Vec<String> = t
        .get("features")
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| v.as_str().map(String::from))
                .collect()
        })
        .unwrap_or_default();
    let optional = t.get("optional").and_then(|v| v.as_bool()).unwrap_or(false);
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
        apt: None,
        cmake_package: None,
        lang: None,
    })
}

fn parse_cargo_table_dep(t: &toml_edit::Table) -> DependencyValue {
    let version = t.get("version").and_then(|v| v.as_str()).map(String::from);
    let features: Vec<String> = t
        .get("features")
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| v.as_str().map(String::from))
                .collect()
        })
        .unwrap_or_default();
    let optional = t.get("optional").and_then(|v| v.as_bool()).unwrap_or(false);
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
        apt: None,
        cmake_package: None,
        lang: None,
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
                            apt: None,
                            cmake_package: None,
                            lang: None,
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

/// Extract dependencies from a CMakeLists.txt file.
///
/// Parses `find_package(PackageName ...)` calls and adds them as system deps.
fn extract_cmake_deps(path: &Path) -> Result<BTreeMap<String, DependencyValue>> {
    let content = fs::read_to_string(path).context("Failed to read CMakeLists.txt")?;
    let mut deps = BTreeMap::new();

    for line in content.lines() {
        let trimmed = line.trim();
        // Skip comments
        if trimmed.starts_with('#') {
            continue;
        }

        // Match find_package(PackageName ...) — package name is the first token
        if let Some(rest) = trimmed.strip_prefix("find_package(") {
            let rest = rest.trim();
            // Extract the package name (first token before whitespace or ')')
            let pkg_end = rest
                .find(|c: char| c.is_whitespace() || c == ')')
                .unwrap_or(rest.len());
            let pkg_name = &rest[..pkg_end];

            if pkg_name.is_empty() {
                continue;
            }

            // Skip CMake built-in pseudo-packages
            if pkg_name.starts_with("horus") {
                continue;
            }

            deps.insert(
                pkg_name.to_string(),
                DependencyValue::Detailed(DetailedDependency {
                    version: None,
                    source: Some(DepSource::System),
                    features: vec![],
                    optional: false,
                    path: None,
                    git: None,
                    branch: None,
                    tag: None,
                    rev: None,
                    apt: None,
                    cmake_package: Some(pkg_name.to_string()),
                    lang: None,
                }),
            );
        }
    }

    Ok(deps)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── parse_pep_dep ──────────────────────────────────────────────────────

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

    #[test]
    fn parse_pep_dep_tilde() {
        let (name, ver) = parse_pep_dep("flask~=2.3");
        assert_eq!(name, "flask");
        assert_eq!(ver, Some("~=2.3".to_string()));
    }

    #[test]
    fn parse_pep_dep_exact() {
        let (name, ver) = parse_pep_dep("pytest==7.4.0");
        assert_eq!(name, "pytest");
        assert_eq!(ver, Some("==7.4.0".to_string()));
    }

    #[test]
    fn parse_pep_dep_not_equal() {
        let (name, ver) = parse_pep_dep("setuptools!=60.0");
        assert_eq!(name, "setuptools");
        assert_eq!(ver, Some("!=60.0".to_string()));
    }

    #[test]
    fn parse_pep_dep_whitespace_trimmed() {
        let (name, ver) = parse_pep_dep("  numpy >= 1.24  ");
        assert_eq!(name, "numpy");
        assert_eq!(ver, Some(">= 1.24".to_string()));
    }

    // ── extract_cargo_deps ─────────────────────────────────────────────────

    #[test]
    fn extract_cargo_deps_simple_versions() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo_path = tmp.path().join("Cargo.toml");
        fs::write(
            &cargo_path,
            r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
serde = "1.0"
tokio = "1.35"
"#,
        )
        .unwrap();

        let deps = extract_cargo_deps(&cargo_path).unwrap();
        assert_eq!(deps.len(), 2);
        assert!(deps.contains_key("serde"));
        assert!(deps.contains_key("tokio"));

        match &deps["serde"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.version.as_deref(), Some("1.0"));
                assert_eq!(d.source, Some(DepSource::CratesIo));
            }
            _ => panic!("Expected Detailed variant"),
        }
    }

    #[test]
    fn extract_cargo_deps_inline_table_with_features() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo_path = tmp.path().join("Cargo.toml");
        fs::write(
            &cargo_path,
            r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
serde = { version = "1.0", features = ["derive"] }
tokio = { version = "1.35", features = ["full"], optional = true }
"#,
        )
        .unwrap();

        let deps = extract_cargo_deps(&cargo_path).unwrap();
        assert_eq!(deps.len(), 2);

        match &deps["serde"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.version.as_deref(), Some("1.0"));
                assert_eq!(d.features, vec!["derive".to_string()]);
                assert!(!d.optional);
            }
            _ => panic!("Expected Detailed"),
        }

        match &deps["tokio"] {
            DependencyValue::Detailed(d) => {
                assert!(d.optional);
                assert_eq!(d.features, vec!["full".to_string()]);
            }
            _ => panic!("Expected Detailed"),
        }
    }

    #[test]
    fn extract_cargo_deps_table_style() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo_path = tmp.path().join("Cargo.toml");
        fs::write(
            &cargo_path,
            r#"
[package]
name = "test"
version = "0.1.0"

[dependencies.serde]
version = "1.0"
features = ["derive"]

[dependencies.my-lib]
path = "../my-lib"
"#,
        )
        .unwrap();

        let deps = extract_cargo_deps(&cargo_path).unwrap();
        assert_eq!(deps.len(), 2);

        match &deps["my-lib"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.source, Some(DepSource::Path));
                assert_eq!(d.path.as_deref(), Some("../my-lib"));
            }
            _ => panic!("Expected Detailed"),
        }
    }

    #[test]
    fn extract_cargo_deps_skips_horus_internal() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo_path = tmp.path().join("Cargo.toml");
        fs::write(
            &cargo_path,
            r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
horus_core = "0.1.9"
horus_library = { version = "0.1.9", path = "../horus_library" }
serde = "1.0"
"#,
        )
        .unwrap();

        let deps = extract_cargo_deps(&cargo_path).unwrap();
        assert_eq!(deps.len(), 1);
        assert!(deps.contains_key("serde"));
        assert!(!deps.contains_key("horus_core"));
        assert!(!deps.contains_key("horus_library"));
    }

    #[test]
    fn extract_cargo_deps_git_dependency() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo_path = tmp.path().join("Cargo.toml");
        fs::write(
            &cargo_path,
            r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
my-crate = { git = "https://github.com/user/repo", branch = "main" }
"#,
        )
        .unwrap();

        let deps = extract_cargo_deps(&cargo_path).unwrap();
        assert_eq!(deps.len(), 1);

        match &deps["my-crate"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.source, Some(DepSource::Git));
                assert_eq!(d.git.as_deref(), Some("https://github.com/user/repo"));
                assert_eq!(d.branch.as_deref(), Some("main"));
            }
            _ => panic!("Expected Detailed"),
        }
    }

    #[test]
    fn extract_cargo_deps_empty_deps_section() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo_path = tmp.path().join("Cargo.toml");
        fs::write(
            &cargo_path,
            r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
"#,
        )
        .unwrap();

        let deps = extract_cargo_deps(&cargo_path).unwrap();
        assert!(deps.is_empty());
    }

    #[test]
    fn extract_cargo_deps_no_deps_section() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo_path = tmp.path().join("Cargo.toml");
        fs::write(
            &cargo_path,
            r#"
[package]
name = "test"
version = "0.1.0"
"#,
        )
        .unwrap();

        let deps = extract_cargo_deps(&cargo_path).unwrap();
        assert!(deps.is_empty());
    }

    // ── extract_pyproject_deps ─────────────────────────────────────────────

    #[test]
    fn extract_pyproject_deps_pep621() {
        let tmp = tempfile::TempDir::new().unwrap();
        let pyproject_path = tmp.path().join("pyproject.toml");
        fs::write(
            &pyproject_path,
            r#"
[project]
name = "my-robot"
version = "0.1.0"
dependencies = [
    "numpy>=1.24",
    "requests",
    "torch>=2.0,<3.0",
]
"#,
        )
        .unwrap();

        let deps = extract_pyproject_deps(&pyproject_path).unwrap();
        assert_eq!(deps.len(), 3);
        assert!(deps.contains_key("numpy"));
        assert!(deps.contains_key("requests"));
        assert!(deps.contains_key("torch"));

        match &deps["numpy"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.version.as_deref(), Some(">=1.24"));
                assert_eq!(d.source, Some(DepSource::PyPI));
            }
            _ => panic!("Expected Detailed"),
        }

        match &deps["requests"] {
            DependencyValue::Detailed(d) => {
                assert!(d.version.is_none());
                assert_eq!(d.source, Some(DepSource::PyPI));
            }
            _ => panic!("Expected Detailed"),
        }
    }

    #[test]
    fn extract_pyproject_deps_skips_horus_internal() {
        let tmp = tempfile::TempDir::new().unwrap();
        let pyproject_path = tmp.path().join("pyproject.toml");
        fs::write(
            &pyproject_path,
            r#"
[project]
name = "my-robot"
dependencies = [
    "horus-py>=0.1.9",
    "numpy>=1.24",
]
"#,
        )
        .unwrap();

        let deps = extract_pyproject_deps(&pyproject_path).unwrap();
        assert_eq!(deps.len(), 1);
        assert!(deps.contains_key("numpy"));
        assert!(!deps.contains_key("horus-py"));
    }

    #[test]
    fn extract_pyproject_deps_empty() {
        let tmp = tempfile::TempDir::new().unwrap();
        let pyproject_path = tmp.path().join("pyproject.toml");
        fs::write(
            &pyproject_path,
            r#"
[project]
name = "my-robot"
dependencies = []
"#,
        )
        .unwrap();

        let deps = extract_pyproject_deps(&pyproject_path).unwrap();
        assert!(deps.is_empty());
    }

    #[test]
    fn extract_pyproject_deps_no_project_section() {
        let tmp = tempfile::TempDir::new().unwrap();
        let pyproject_path = tmp.path().join("pyproject.toml");
        fs::write(
            &pyproject_path,
            r#"
[build-system]
requires = ["setuptools"]
"#,
        )
        .unwrap();

        let deps = extract_pyproject_deps(&pyproject_path).unwrap();
        assert!(deps.is_empty());
    }

    // ── run_migrate end-to-end ─────────────────────────────────────────────

    #[test]
    fn migrate_fails_without_horus_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No horus.toml found"),
            "Expected 'No horus.toml found', got: {}",
            err
        );
    }

    #[test]
    fn migrate_nothing_to_migrate() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Create minimal horus.toml only
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus")).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
    }

    #[test]
    fn migrate_dry_run_does_not_modify_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        let cargo_content = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
serde = "1.0"
"#;
        fs::write(tmp.path().join("Cargo.toml"), cargo_content).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(true, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // Cargo.toml should NOT be moved
        assert!(tmp.path().join("Cargo.toml").exists());
        // horus.toml should NOT have deps added
        let horus = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(
            !horus.contains("serde"),
            "Dry run should not modify horus.toml"
        );
        // No backup dir
        assert!(!tmp.path().join(".horus/backup").exists());
    }

    #[test]
    fn migrate_imports_cargo_deps_and_backs_up() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        fs::write(
            tmp.path().join("Cargo.toml"),
            r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
serde = "1.0"
tokio = "1.35"
"#,
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // Cargo.toml should be moved to backup
        assert!(!tmp.path().join("Cargo.toml").exists());
        assert!(tmp.path().join(".horus/backup/Cargo.toml").exists());
        // horus.toml should now contain imported deps
        let horus = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(
            horus.contains("serde"),
            "horus.toml should contain serde dep"
        );
        assert!(
            horus.contains("tokio"),
            "horus.toml should contain tokio dep"
        );
    }

    #[test]
    fn migrate_imports_pyproject_deps_and_backs_up() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        fs::write(
            tmp.path().join("pyproject.toml"),
            r#"
[project]
name = "test"
dependencies = ["numpy>=1.24", "requests"]
"#,
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert!(!tmp.path().join("pyproject.toml").exists());
        assert!(tmp.path().join(".horus/backup/pyproject.toml").exists());
        let horus = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(
            horus.contains("numpy"),
            "horus.toml should contain numpy dep"
        );
        assert!(
            horus.contains("requests"),
            "horus.toml should contain requests dep"
        );
    }

    #[test]
    fn migrate_moves_src_main_rs_when_only_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/main.rs"), "fn main() {}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // main.rs moved to root
        assert!(tmp.path().join("main.rs").exists());
        assert!(!tmp.path().join("src/main.rs").exists());
        // src/ should be removed (was empty after move)
        assert!(!tmp.path().join("src").exists());
        // Content preserved
        let content = fs::read_to_string(tmp.path().join("main.rs")).unwrap();
        assert_eq!(content, "fn main() {}");
    }

    #[test]
    fn migrate_does_not_move_src_main_when_other_files_exist() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/main.rs"), "fn main() {}").unwrap();
        fs::write(tmp.path().join("src/lib.rs"), "pub mod foo;").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // main.rs should NOT be moved — src/ has multiple files
        assert!(tmp.path().join("src/main.rs").exists());
        assert!(!tmp.path().join("main.rs").exists());
    }

    #[test]
    fn migrate_does_not_move_src_main_when_root_main_exists() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/main.rs"), "fn main() { old }").unwrap();
        fs::write(tmp.path().join("main.rs"), "fn main() { new }").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_migrate(false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // Neither moved — root main.rs already exists
        assert!(tmp.path().join("src/main.rs").exists());
        let content = fs::read_to_string(tmp.path().join("main.rs")).unwrap();
        assert_eq!(content, "fn main() { new }");
    }
}
