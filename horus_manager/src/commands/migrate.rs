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
            for name in cargo_deps.keys() {
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
            for name in pypi_deps.keys() {
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
            for name in cmake_deps.keys() {
                changes.push(format!("  + {} (system)", name));
            }
            if !dry_run {
                manifest.dependencies.extend(cmake_deps);
            }
        }
    }

    // ── Move source files ────────────────────────────────────────────────
    // Only move files if there's actually an old-layout project to migrate
    // (i.e. Cargo.toml or pyproject.toml exists in root alongside horus.toml)
    let has_old_build_files = cargo_path.exists() || pyproject_path.exists() || cmake_path.exists();
    let src_main = cwd.join("src/main.rs");
    let root_main = cwd.join("main.rs");
    if has_old_build_files && src_main.exists() && !root_main.exists() {
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
            "*".green()
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
        println!("{} Migration complete. Old files backed up.", "*".green());
        println!(
            "  Run {} to verify the project builds.",
            "horus build".cyan()
        );
    }

    Ok(())
}

/// Extracted deps and dev-deps from a Cargo.toml.
pub(crate) struct ExtractedCargoDeps {
    pub deps: BTreeMap<String, DependencyValue>,
    pub dev_deps: BTreeMap<String, DependencyValue>,
}

/// Extract dependencies from a Cargo.toml file (both `[dependencies]` and `[dev-dependencies]`).
pub(crate) fn extract_cargo_all_deps(path: &Path) -> Result<ExtractedCargoDeps> {
    let content = fs::read_to_string(path).context("Failed to read Cargo.toml")?;
    let doc = content
        .parse::<toml_edit::DocumentMut>()
        .context("Failed to parse Cargo.toml")?;

    let deps = parse_cargo_dep_table(&doc, "dependencies");
    let dev_deps = parse_cargo_dep_table(&doc, "dev-dependencies");

    Ok(ExtractedCargoDeps { deps, dev_deps })
}

/// Extract only `[dependencies]` (backward-compat wrapper).
pub(crate) fn extract_cargo_deps(path: &Path) -> Result<BTreeMap<String, DependencyValue>> {
    Ok(extract_cargo_all_deps(path)?.deps)
}

/// Parse a single TOML dependency table section by name.
fn parse_cargo_dep_table(
    doc: &toml_edit::DocumentMut,
    section: &str,
) -> BTreeMap<String, DependencyValue> {
    let mut deps = BTreeMap::new();

    if let Some(dep_table) = doc.get(section).and_then(|d| d.as_table()) {
        for (name, item) in dep_table.iter() {
            if crate::native_sync::is_horus_internal(name) {
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
                        workspace: false,
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

    deps
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
    let tag = t.get("tag").and_then(|v| v.as_str()).map(String::from);
    let rev = t.get("rev").and_then(|v| v.as_str()).map(String::from);
    let workspace = t
        .get("workspace")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

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
        tag,
        rev,
        apt: None,
        cmake_package: None,
        lang: None,
        workspace,
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
    let tag = t.get("tag").and_then(|v| v.as_str()).map(String::from);
    let rev = t.get("rev").and_then(|v| v.as_str()).map(String::from);
    let workspace = t
        .get("workspace")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

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
        tag,
        rev,
        apt: None,
        cmake_package: None,
        lang: None,
        workspace,
    })
}

/// Extract dependencies from a pyproject.toml file.
pub(crate) fn extract_pyproject_deps(path: &Path) -> Result<BTreeMap<String, DependencyValue>> {
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
                    // Skip horus internal packages (exact match)
                    if crate::native_sync::is_horus_internal(&name) {
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
                            workspace: false,
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
    let split_at = spec.find(['>', '<', '=', '!', '~']).unwrap_or(spec.len());

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
pub(crate) fn extract_cmake_deps(path: &Path) -> Result<BTreeMap<String, DependencyValue>> {
    let content = fs::read_to_string(path).context("Failed to read CMakeLists.txt")?;
    let mut deps = BTreeMap::new();

    // Accumulator for multi-line FetchContent/ExternalProject blocks
    let mut fetch_block: Option<String> = None;

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('#') {
            continue;
        }

        // ── Multi-line block accumulation ──────────────────────────────────
        if let Some(ref mut block) = fetch_block {
            block.push(' ');
            block.push_str(trimmed);
            if trimmed.contains(')') {
                // Block complete — parse it
                parse_fetch_content_block(block, &mut deps);
                fetch_block = None;
            }
            continue;
        }

        // ── FetchContent_Declare( ──────────────────────────────────────────
        if trimmed.starts_with("FetchContent_Declare(")
            || trimmed.starts_with("ExternalProject_Add(")
        {
            let block = trimmed.to_string();
            if trimmed.contains(')') {
                parse_fetch_content_block(&block, &mut deps);
            } else {
                fetch_block = Some(block);
            }
            continue;
        }

        // ── pkg_check_modules / pkg_search_module ──────────────────────────
        if let Some(rest) = trimmed
            .strip_prefix("pkg_check_modules(")
            .or_else(|| trimmed.strip_prefix("pkg_search_module("))
        {
            let inner = rest.trim_end_matches(')');
            let tokens: Vec<&str> = inner.split_whitespace().collect();
            // First token is the prefix variable, rest are package names (skip REQUIRED/QUIET/IMPORTED_TARGET)
            for tok in tokens.iter().skip(1) {
                if matches!(
                    *tok,
                    "REQUIRED" | "QUIET" | "IMPORTED_TARGET" | "NO_CMAKE_PATH"
                ) {
                    continue;
                }
                if !tok.is_empty() && !crate::native_sync::is_horus_internal(tok) {
                    deps.insert(
                        tok.to_string(),
                        DependencyValue::Detailed(DetailedDependency {
                            source: Some(DepSource::System),
                            cmake_package: Some(tok.to_string()),
                            lang: Some("cpp".to_string()),
                            ..DetailedDependency::default()
                        }),
                    );
                }
            }
            continue;
        }

        // ── find_package(PackageName ...) ──────────────────────────────────
        if let Some(rest) = trimmed.strip_prefix("find_package(") {
            let rest = rest.trim();
            let pkg_end = rest
                .find(|c: char| c.is_whitespace() || c == ')')
                .unwrap_or(rest.len());
            let pkg_name = &rest[..pkg_end];

            if pkg_name.is_empty() || pkg_name == "PkgConfig" {
                continue;
            }

            if crate::native_sync::is_horus_internal(pkg_name) {
                continue;
            }

            deps.insert(
                pkg_name.to_string(),
                DependencyValue::Detailed(DetailedDependency {
                    source: Some(DepSource::System),
                    cmake_package: Some(pkg_name.to_string()),
                    ..DetailedDependency::default()
                }),
            );
        }
    }

    Ok(deps)
}

/// Parse a FetchContent_Declare or ExternalProject_Add block to extract a git dependency.
fn parse_fetch_content_block(block: &str, deps: &mut BTreeMap<String, DependencyValue>) {
    // Extract name: first token after opening paren
    let inner = block
        .find('(')
        .map(|i| &block[i + 1..])
        .unwrap_or("")
        .trim_end_matches(')')
        .trim();

    let tokens: Vec<&str> = inner.split_whitespace().collect();
    if tokens.is_empty() {
        return;
    }
    let name = tokens[0].to_lowercase();
    if name.is_empty() || crate::native_sync::is_horus_internal(&name) {
        return;
    }

    // Find GIT_REPOSITORY or URL
    let mut git_url = None;
    let mut git_tag = None;
    for i in 0..tokens.len() {
        match tokens[i] {
            "GIT_REPOSITORY" | "URL" => {
                if let Some(url) = tokens.get(i + 1) {
                    git_url = Some(url.to_string());
                }
            }
            "GIT_TAG" => {
                if let Some(tag) = tokens.get(i + 1) {
                    git_tag = Some(tag.to_string());
                }
            }
            _ => {}
        }
    }

    if let Some(url) = git_url {
        deps.insert(
            name.clone(),
            DependencyValue::Detailed(DetailedDependency {
                source: Some(DepSource::Git),
                git: Some(url),
                tag: git_tag,
                cmake_package: Some(name),
                lang: Some("cpp".to_string()),
                ..DetailedDependency::default()
            }),
        );
    }
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
horus_core = "0.2.0"
horus_library = { version = "0.2.0", path = "../horus_library" }
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
    "horus-py>=0.2.0",
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
        // Migration requires an old build file (Cargo.toml) to detect legacy layout
        fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
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

    // ── extract_cmake_deps ────────────────────────────────────────────────

    #[test]
    fn test_extract_cmake_deps_find_package() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cmake_path = tmp.path().join("CMakeLists.txt");
        fs::write(
            &cmake_path,
            r#"
cmake_minimum_required(VERSION 3.14)
project(mybot)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system)
"#,
        )
        .unwrap();

        let deps = extract_cmake_deps(&cmake_path).unwrap();
        assert_eq!(deps.len(), 2);
        assert!(deps.contains_key("OpenCV"), "Should contain OpenCV");
        assert!(deps.contains_key("Boost"), "Should contain Boost");

        match &deps["OpenCV"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.source, Some(DepSource::System));
                assert_eq!(d.cmake_package.as_deref(), Some("OpenCV"));
            }
            _ => panic!("Expected Detailed variant for OpenCV"),
        }

        match &deps["Boost"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.source, Some(DepSource::System));
                assert_eq!(d.cmake_package.as_deref(), Some("Boost"));
            }
            _ => panic!("Expected Detailed variant for Boost"),
        }
    }

    #[test]
    fn test_extract_cmake_deps_fetch_content() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cmake_path = tmp.path().join("CMakeLists.txt");
        fs::write(
            &cmake_path,
            r#"
cmake_minimum_required(VERSION 3.14)
include(FetchContent)

FetchContent_Declare(json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.3)
FetchContent_MakeAvailable(json)
"#,
        )
        .unwrap();

        let deps = extract_cmake_deps(&cmake_path).unwrap();
        assert_eq!(deps.len(), 1);
        assert!(deps.contains_key("json"), "Should contain json");

        match &deps["json"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.source, Some(DepSource::Git));
                assert_eq!(
                    d.git.as_deref(),
                    Some("https://github.com/nlohmann/json.git")
                );
                assert_eq!(d.tag.as_deref(), Some("v3.11.3"));
                assert_eq!(d.cmake_package.as_deref(), Some("json"));
                assert_eq!(d.lang.as_deref(), Some("cpp"));
            }
            _ => panic!("Expected Detailed variant for json"),
        }
    }

    #[test]
    fn test_extract_cmake_deps_pkg_check_modules() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cmake_path = tmp.path().join("CMakeLists.txt");
        fs::write(
            &cmake_path,
            r#"
cmake_minimum_required(VERSION 3.14)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB REQUIRED glib-2.0)
"#,
        )
        .unwrap();

        let deps = extract_cmake_deps(&cmake_path).unwrap();
        // find_package(PkgConfig) is skipped by the parser
        assert_eq!(deps.len(), 1);
        assert!(deps.contains_key("glib-2.0"), "Should contain glib-2.0");

        match &deps["glib-2.0"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.source, Some(DepSource::System));
                assert_eq!(d.cmake_package.as_deref(), Some("glib-2.0"));
                assert_eq!(d.lang.as_deref(), Some("cpp"));
            }
            _ => panic!("Expected Detailed variant for glib-2.0"),
        }
    }

    #[test]
    fn test_extract_cmake_deps_empty_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cmake_path = tmp.path().join("CMakeLists.txt");
        fs::write(&cmake_path, "").unwrap();

        let deps = extract_cmake_deps(&cmake_path).unwrap();
        assert!(deps.is_empty(), "Empty file should produce no deps");
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
