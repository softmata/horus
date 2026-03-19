//! `horus doctor` — comprehensive ecosystem health check.
//!
//! Checks: toolchains, SHM, plugins, manifest validity, registry,
//! system deps, disk space. Summary by default, --verbose for details.
//!
//! With `--fix`: installs missing toolchains and system dependencies,
//! then pins their versions in `horus.lock`.

use anyhow::Result;
use colored::*;
use std::net::TcpStream;
use std::path::Path;
use std::time::Duration;

use horus_core::drivers::{self, DriverType};
use horus_sys::sync::{self, SyncManifest, SystemDep};

use crate::dispatch;
use crate::lockfile::{HorusLockfile, SystemLock, HORUS_LOCK};
use crate::manifest::HorusManifest;

/// Health status for a check category.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Health {
    Ok,
    Warn,
    Fail,
}

impl Health {
    fn icon(&self) -> colored::ColoredString {
        match self {
            Self::Ok => "✓".green(),
            Self::Warn => "!".yellow(),
            Self::Fail => "✗".red(),
        }
    }
}

/// A single check result.
pub(crate) struct CheckResult {
    pub(crate) category: String,
    pub(crate) health: Health,
    pub(crate) summary: String,
    pub(crate) details: Vec<String>,
}

/// Run `horus doctor`.
pub fn run_doctor(verbose: bool, json: bool, fix: bool) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);
    let mut results = Vec::new();

    // ── 1. Toolchains ────────────────────────────────────────────────────
    results.push(check_toolchains());

    // ── 2. Horus manifest ────────────────────────────────────────────────
    results.push(check_manifest(&ctx));

    // ── 3. Shared memory ─────────────────────────────────────────────────
    results.push(check_shm());

    // ── 4. Plugins ───────────────────────────────────────────────────────
    results.push(check_plugins());

    // ── 5. Disk usage ────────────────────────────────────────────────────
    results.push(check_disk());

    // ── 6. Project languages ─────────────────────────────────────────────
    results.push(check_languages(&ctx));

    // ── 7. Dependency sources ─────────────────────────────────────────────
    results.push(check_dep_sources(&ctx));

    // ── 8. Driver device reachability ────────────────────────────────────
    results.push(check_drivers());

    // ── 9. System dependencies (Python, C++, system libs) ────────────────
    if let Some(manifest) = &ctx.manifest {
        results.push(check_system_deps(manifest));
    }

    // ── Output ───────────────────────────────────────────────────────────
    if json {
        print_json(&results);
    } else {
        print_summary(&results, verbose);
    }

    // ── Fix mode: install missing deps and pin to horus.lock ─────────────
    if fix {
        if let Some(manifest) = &ctx.manifest {
            run_fix(manifest, &ctx)?;
        } else {
            println!(
                "\n  {} No horus.toml found — nothing to fix. Run {} to create a project.",
                "!".yellow(),
                "horus new".cyan()
            );
        }
    }

    // Exit code
    let has_failures = results.iter().any(|r| r.health == Health::Fail);
    let has_warnings = results.iter().any(|r| r.health == Health::Warn);

    if has_failures && !fix {
        std::process::exit(2);
    } else if has_warnings && !fix {
        std::process::exit(1);
    }
    Ok(())
}

/// Install missing toolchains/system deps and pin versions to horus.lock.
fn run_fix(manifest: &HorusManifest, ctx: &dispatch::ProjectContext) -> Result<()> {
    println!("\n{}", "Fixing environment...".bold());

    let report = sync::sync_environment(manifest)?;

    for item in &report.items {
        if item.installed {
            let version = item.version.as_deref().unwrap_or("installed");
            println!("  {} {} ({})", "✓".green(), item.name, version);
        } else {
            println!("  {} {} — not installed", "✗".red(), item.name);
            if let Some(ref cmd) = item.install_cmd {
                println!("    Install: {}", cmd.dimmed());
            }
        }
    }

    // Pin toolchain + system dep versions into horus.lock
    let lock_path = ctx.root.join(HORUS_LOCK);
    let mut lockfile = HorusLockfile::load_from(&lock_path).unwrap_or_default();

    // Build toolchain pins from the sync report
    let mut toolchain = lockfile.toolchain.unwrap_or_default();
    for item in &report.items {
        if let Some(ver) = &item.version {
            match item.name.as_str() {
                "rust" => toolchain.rust = Some(ver.clone()),
                "python" => toolchain.python = Some(ver.clone()),
                "cmake" => toolchain.cmake = Some(ver.clone()),
                _ => {}
            }
        }
    }
    lockfile.toolchain = Some(toolchain);

    // Build system dep pins from the sync report
    let system_dep_names: Vec<String> = manifest
        .system_deps()
        .iter()
        .map(|d| d.name.clone())
        .collect();
    for item in &report.items {
        if system_dep_names.contains(&item.name) {
            if let Some(ver) = &item.version {
                // Find the manifest dep for cross-platform package names
                let manifest_dep = manifest.system_deps().into_iter().find(|d| d.name == item.name);
                let existing = lockfile.system_deps.iter_mut().find(|s| s.name == item.name);
                if let Some(existing) = existing {
                    existing.version = ver.clone();
                } else {
                    lockfile.system_deps.push(SystemLock {
                        name: item.name.clone(),
                        version: ver.clone(),
                        pkg_config: manifest_dep.as_ref().and_then(|d| d.pkg_config.clone()),
                        apt: manifest_dep.as_ref().and_then(|d| d.apt.clone()),
                        brew: manifest_dep.as_ref().and_then(|d| d.brew.clone()),
                        pacman: None,
                        choco: manifest_dep.as_ref().and_then(|d| d.choco.clone()),
                    });
                }
            }
        }
    }

    lockfile.save_to(&lock_path)?;
    println!(
        "\n  {} Environment synced — {} updated",
        "✓".green(),
        HORUS_LOCK.bold()
    );

    if !report.all_satisfied {
        println!(
            "\n{} Some dependencies could not be installed automatically.",
            "!".yellow()
        );
    }

    Ok(())
}

pub(crate) fn print_summary(results: &[CheckResult], verbose: bool) {
    println!("{}", "horus doctor".bold());
    println!();

    for result in results {
        println!(
            "  {} {} — {}",
            result.health.icon(),
            result.category.bold(),
            result.summary
        );
        if verbose {
            for detail in &result.details {
                println!("      {}", detail.dimmed());
            }
        }
    }

    println!();
    let ok_count = results.iter().filter(|r| r.health == Health::Ok).count();
    let warn_count = results.iter().filter(|r| r.health == Health::Warn).count();
    let fail_count = results.iter().filter(|r| r.health == Health::Fail).count();

    if fail_count > 0 {
        println!(
            "  {} {ok_count} ok, {warn_count} warnings, {fail_count} failures",
            "Summary:".bold()
        );
    } else if warn_count > 0 {
        println!(
            "  {} {ok_count} ok, {warn_count} warnings",
            "Summary:".bold()
        );
    } else {
        println!("  {} All {ok_count} checks passed", "Summary:".bold());
    }
}

fn print_json(results: &[CheckResult]) {
    let entries: Vec<serde_json::Value> = results
        .iter()
        .map(|r| {
            serde_json::json!({
                "category": r.category,
                "health": format!("{:?}", r.health).to_lowercase(),
                "summary": r.summary,
                "details": r.details,
            })
        })
        .collect();
    println!("{}", serde_json::to_string_pretty(&entries).unwrap());
}

// ─── Individual checks ───────────────────────────────────────────────────────

fn check_toolchains() -> CheckResult {
    let mut details = Vec::new();
    let mut missing = Vec::new();

    let tools = [
        ("cargo", "Rust build tool"),
        ("rustc", "Rust compiler"),
        ("python3", "Python interpreter"),
        ("ruff", "Python linter/formatter"),
        ("pytest", "Python test runner"),
        ("cmake", "C++ build system"),
        ("clang-format", "C++ formatter"),
        ("clang-tidy", "C++ linter"),
    ];

    for (tool, desc) in &tools {
        match dispatch::tool_version(tool) {
            Some(version) => {
                details.push(format!("{}: {} ({})", tool, version, desc));
            }
            None => {
                details.push(format!("{}: not found ({})", tool, desc));
                if *tool == "cargo" || *tool == "rustc" {
                    missing.push(*tool);
                }
            }
        }
    }

    let health = if !missing.is_empty() {
        Health::Fail
    } else {
        Health::Ok
    };

    let found = tools.len() - details.iter().filter(|d| d.contains("not found")).count();
    CheckResult {
        category: "Toolchains".to_string(),
        health,
        summary: format!("{}/{} tools found", found, tools.len()),
        details,
    }
}

fn check_manifest(ctx: &dispatch::ProjectContext) -> CheckResult {
    if !ctx.has_horus_toml {
        return CheckResult {
            category: "Manifest".to_string(),
            health: Health::Warn,
            summary: "No horus.toml found".to_string(),
            details: vec!["Run 'horus new' or 'horus init' to create a project".to_string()],
        };
    }

    match &ctx.manifest {
        Some(manifest) => match manifest.validate() {
            Ok(warnings) => {
                let mut details = vec![
                    format!("name: {}", manifest.package.name),
                    format!("version: {}", manifest.package.version),
                    format!("dependencies: {}", manifest.dependencies.len()),
                ];
                let health = if warnings.is_empty() {
                    Health::Ok
                } else {
                    details.extend(warnings.iter().map(|w| format!("warning: {}", w)));
                    Health::Warn
                };
                CheckResult {
                    category: "Manifest".to_string(),
                    health,
                    summary: "horus.toml valid".to_string(),
                    details,
                }
            }
            Err(e) => CheckResult {
                category: "Manifest".to_string(),
                health: Health::Fail,
                summary: "horus.toml invalid".to_string(),
                details: vec![e.to_string()],
            },
        },
        None => CheckResult {
            category: "Manifest".to_string(),
            health: Health::Fail,
            summary: "Failed to parse horus.toml".to_string(),
            details: vec![],
        },
    }
}

fn check_shm() -> CheckResult {
    let shm_parent = horus_sys::shm::shm_parent_dir();
    if !shm_parent.exists() {
        return CheckResult {
            category: "Shared Memory".to_string(),
            health: Health::Warn,
            summary: format!("{} not available", shm_parent.display()),
            details: vec!["SHM IPC may not work on this platform".to_string()],
        };
    }

    // Count horus namespaces
    let count = std::fs::read_dir(&shm_parent)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok())
                .filter(|e| e.file_name().to_string_lossy().starts_with("horus_"))
                .count()
        })
        .unwrap_or(0);

    CheckResult {
        category: "Shared Memory".to_string(),
        health: Health::Ok,
        summary: format!(
            "{} available, {} horus namespaces",
            shm_parent.display(),
            count
        ),
        details: vec![],
    }
}

fn check_plugins() -> CheckResult {
    let global_dir = dirs::home_dir()
        .map(|h| h.join(".horus/plugins"))
        .unwrap_or_default();

    let local_dir = Path::new(".horus/plugins");

    let global_count = count_items(&global_dir);
    let local_count = count_items(local_dir);

    CheckResult {
        category: "Plugins".to_string(),
        health: Health::Ok,
        summary: format!("{} global, {} local", global_count, local_count),
        details: vec![],
    }
}

fn check_disk() -> CheckResult {
    let horus_dir = Path::new(".horus");
    if !horus_dir.exists() {
        return CheckResult {
            category: "Disk".to_string(),
            health: Health::Ok,
            summary: "No .horus/ directory yet".to_string(),
            details: vec![],
        };
    }

    let size = dir_size(horus_dir);
    let formatted = format_bytes(size);

    let health = if size > 5_000_000_000 {
        // > 5GB
        Health::Warn
    } else {
        Health::Ok
    };

    CheckResult {
        category: "Disk".to_string(),
        health,
        summary: format!("Build cache uses {}", formatted),
        details: vec![],
    }
}

fn check_languages(ctx: &dispatch::ProjectContext) -> CheckResult {
    if ctx.languages.is_empty() {
        return CheckResult {
            category: "Languages".to_string(),
            health: Health::Warn,
            summary: "No languages detected".to_string(),
            details: vec!["Create source files or a horus.toml with dependencies".to_string()],
        };
    }

    let langs: Vec<String> = ctx.languages.iter().map(|l| l.to_string()).collect();
    CheckResult {
        category: "Languages".to_string(),
        health: Health::Ok,
        summary: langs.join(", "),
        details: vec![],
    }
}

fn check_dep_sources(ctx: &dispatch::ProjectContext) -> CheckResult {
    let manifest = match &ctx.manifest {
        Some(m) => m,
        None => {
            return CheckResult {
                category: "Dependencies".to_string(),
                health: Health::Ok,
                summary: "No manifest — skipped".to_string(),
                details: vec![],
            };
        }
    };

    if manifest.dependencies.is_empty() && manifest.dev_dependencies.is_empty() {
        return CheckResult {
            category: "Dependencies".to_string(),
            health: Health::Ok,
            summary: "No dependencies".to_string(),
            details: vec![],
        };
    }

    let issues = crate::source_resolver::validate_deps(&manifest.dependencies, &ctx.languages);
    let dev_issues =
        crate::source_resolver::validate_deps(&manifest.dev_dependencies, &ctx.languages);

    let all_issues: Vec<_> = issues.into_iter().chain(dev_issues).collect();

    if all_issues.is_empty() {
        let dep_count = manifest.dependencies.len() + manifest.dev_dependencies.len();
        CheckResult {
            category: "Dependencies".to_string(),
            health: Health::Ok,
            summary: format!("{} deps, sources look correct", dep_count),
            details: vec![],
        }
    } else {
        let has_errors = all_issues.iter().any(|i| i.is_error);
        let details: Vec<String> = all_issues.iter().map(|i| i.message.clone()).collect();
        CheckResult {
            category: "Dependencies".to_string(),
            health: if has_errors {
                Health::Fail
            } else {
                Health::Warn
            },
            summary: format!("{} issue(s) found", all_issues.len()),
            details,
        }
    }
}

// ─── System dependency check (absorbed from horus sync) ──────────────────────

/// Implement SyncManifest for HorusManifest so horus_sys::sync can extract requirements.
impl SyncManifest for HorusManifest {
    fn rust_edition(&self) -> Option<String> {
        Some(self.package.edition.clone())
    }

    fn python_version(&self) -> Option<String> {
        let has_python = self.dependencies.iter().any(|(_, dep)| {
            if let crate::manifest::DependencyValue::Detailed(d) = dep {
                matches!(d.source, Some(crate::manifest::DepSource::PyPI))
            } else {
                false
            }
        });
        if has_python {
            Some(">=3.9".to_string())
        } else {
            None
        }
    }

    fn system_deps(&self) -> Vec<SystemDep> {
        self.dependencies
            .iter()
            .filter_map(|(name, dep)| {
                if let crate::manifest::DependencyValue::Detailed(d) = dep {
                    if matches!(d.source, Some(crate::manifest::DepSource::System)) {
                        return Some(SystemDep {
                            name: name.clone(),
                            apt: d.apt.clone().or_else(|| Some(name.clone())),
                            brew: Some(name.clone()),
                            choco: Some(name.clone()),
                            pkg_config: d.cmake_package.clone(),
                        });
                    }
                }
                None
            })
            .collect()
    }

    fn needs_cpp(&self) -> bool {
        self.cpp.is_some()
    }

    fn project_name(&self) -> String {
        self.package.name.clone()
    }
}

fn check_system_deps(manifest: &HorusManifest) -> CheckResult {
    let report = sync::check_environment(manifest);

    if report.items.is_empty() {
        return CheckResult {
            category: "System Deps".to_string(),
            health: Health::Ok,
            summary: "No system dependencies required".to_string(),
            details: vec![],
        };
    }

    let mut details = Vec::new();
    let mut missing = Vec::new();

    for item in &report.items {
        if item.installed {
            let version = item.version.as_deref().unwrap_or("installed");
            details.push(format!("{}: {} ({})", item.name, version, "ok"));
        } else {
            details.push(format!("{}: not found", item.name));
            if item.required {
                missing.push(item.name.clone());
            }
        }
    }

    let health = if !missing.is_empty() {
        Health::Fail
    } else {
        Health::Ok
    };

    let found = report.items.iter().filter(|i| i.installed).count();
    let hint = if !missing.is_empty() {
        format!(
            " — run {} to install",
            "horus doctor --fix".cyan()
        )
    } else {
        String::new()
    };

    CheckResult {
        category: "System Deps".to_string(),
        health,
        summary: format!("{}/{} satisfied{}", found, report.items.len(), hint),
        details,
    }
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

fn count_items(dir: &Path) -> usize {
    std::fs::read_dir(dir)
        .map(|entries| entries.filter_map(|e| e.ok()).count())
        .unwrap_or(0)
}

use crate::fs_utils::{dir_size, format_bytes};

// ── Driver device reachability ─────────────────────────────────────────────

fn check_drivers() -> CheckResult {
    let hw = match drivers::load() {
        Ok(hw) if !hw.is_empty() => hw,
        _ => {
            return CheckResult {
                category: "Drivers".into(),
                health: Health::Ok,
                summary: "No [drivers] configured".into(),
                details: vec![],
            };
        }
    };

    let mut details = Vec::new();
    let mut worst = Health::Ok;

    for name in hw.list() {
        let params = match hw.params(name) {
            Some(p) => p,
            None => continue,
        };
        let dtype = hw.driver_type(name);

        let (detail, health) = check_driver_device(name, params, dtype);
        if health == Health::Fail && worst != Health::Fail {
            worst = Health::Fail;
        } else if health == Health::Warn && worst == Health::Ok {
            worst = Health::Warn;
        }
        details.push(detail);
    }

    let summary = if worst == Health::Ok {
        format!("{} driver(s) reachable", details.len())
    } else {
        format!("{} driver(s) checked, some unreachable", details.len())
    };

    CheckResult {
        category: "Drivers".into(),
        health: worst,
        summary,
        details,
    }
}

fn check_driver_device(
    name: &str,
    params: &horus_core::drivers::DriverParams,
    dtype: Option<&DriverType>,
) -> (String, Health) {
    // Check serial port / device file
    if let Ok(port) = params.get::<String>("port") {
        if port.starts_with("/dev/") {
            return if Path::new(&port).exists() {
                (
                    format!("  {} driver '{}': {} found", "✓".green(), name, port),
                    Health::Ok,
                )
            } else {
                (
                    format!("  {} driver '{}': {} not found", "✗".red(), name, port),
                    Health::Fail,
                )
            };
        }
    }

    // Check I2C bus
    if let Ok(bus) = params.get::<String>("bus") {
        if bus.starts_with("i2c-") || bus.starts_with("/dev/i2c") {
            let path = if bus.starts_with("/dev/") {
                bus.clone()
            } else {
                format!("/dev/{}", bus)
            };
            return if Path::new(&path).exists() {
                (
                    format!("  {} driver '{}': {} found", "✓".green(), name, path),
                    Health::Ok,
                )
            } else {
                (
                    format!("  {} driver '{}': {} not found", "✗".red(), name, path),
                    Health::Fail,
                )
            };
        }
    }

    // Check network address
    if let Ok(address) = params.get::<String>("address") {
        if address.contains('.') || address.contains(':') {
            let addr_str = if address.contains(':') {
                address.clone()
            } else {
                format!("{}:80", address)
            };
            if let Ok(addr) = addr_str.parse::<std::net::SocketAddr>() {
                return match TcpStream::connect_timeout(&addr, Duration::from_secs(2)) {
                    Ok(_) => (
                        format!("  {} driver '{}': {} reachable", "✓".green(), name, address),
                        Health::Ok,
                    ),
                    Err(_) => (
                        format!(
                            "  {} driver '{}': {} unreachable",
                            "!".yellow(),
                            name,
                            address
                        ),
                        Health::Warn,
                    ),
                };
            }
        }
    }

    // No checkable params — report driver type
    let type_str = match dtype {
        Some(DriverType::Terra(t)) => format!("terra={}", t),
        Some(DriverType::Package(p)) => format!("package={}", p),
        Some(DriverType::Local(n)) => format!("node={}", n),
        Some(DriverType::Legacy) => "legacy".into(),
        None => "unknown".into(),
    };
    (
        format!(
            "  - driver '{}': {} (no device path to check)",
            name, type_str
        ),
        Health::Ok,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use std::fs;
    use std::path::PathBuf;

    /// Helper to build a minimal valid HorusManifest for testing.
    fn make_manifest(name: &str) -> crate::manifest::HorusManifest {
        crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: name.to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
        }
    }

    // ── Health enum ─────────────────────────────────────────────────────

    #[test]
    fn health_ok_icon_is_green_check() {
        let icon = Health::Ok.icon();
        assert!(icon.to_string().contains("✓"));
    }

    #[test]
    fn health_warn_icon_is_yellow_bang() {
        let icon = Health::Warn.icon();
        assert!(icon.to_string().contains("!"));
    }

    #[test]
    fn health_fail_icon_is_red_x() {
        let icon = Health::Fail.icon();
        assert!(icon.to_string().contains("✗"));
    }

    #[test]
    fn health_eq_same() {
        assert_eq!(Health::Ok, Health::Ok);
        assert_eq!(Health::Warn, Health::Warn);
        assert_eq!(Health::Fail, Health::Fail);
    }

    #[test]
    fn health_ne_different() {
        assert_ne!(Health::Ok, Health::Warn);
        assert_ne!(Health::Ok, Health::Fail);
        assert_ne!(Health::Warn, Health::Fail);
    }

    #[test]
    fn health_clone() {
        let h = Health::Warn;
        let h2 = h;
        assert_eq!(h, h2);
    }

    #[test]
    fn health_debug() {
        let dbg = format!("{:?}", Health::Ok);
        assert_eq!(dbg, "Ok");
        assert_eq!(format!("{:?}", Health::Warn), "Warn");
        assert_eq!(format!("{:?}", Health::Fail), "Fail");
    }

    // ── format_bytes ────────────────────────────────────────────────────

    #[test]
    fn format_bytes_zero() {
        assert_eq!(format_bytes(0), "0 B");
    }

    #[test]
    fn format_bytes_small() {
        assert_eq!(format_bytes(512), "512 B");
    }

    #[test]
    fn format_bytes_exactly_1kb() {
        assert_eq!(format_bytes(1024), "1.0 KB");
    }

    #[test]
    fn format_bytes_kilobytes() {
        assert_eq!(format_bytes(1536), "1.5 KB");
    }

    #[test]
    fn format_bytes_megabytes() {
        assert_eq!(format_bytes(1_048_576), "1.0 MB");
    }

    #[test]
    fn format_bytes_megabytes_fractional() {
        assert_eq!(format_bytes(5_242_880), "5.0 MB");
    }

    #[test]
    fn format_bytes_gigabytes() {
        assert_eq!(format_bytes(1_073_741_824), "1.00 GB");
    }

    #[test]
    fn format_bytes_gigabytes_large() {
        assert_eq!(format_bytes(10_737_418_240), "10.00 GB");
    }

    #[test]
    fn format_bytes_boundary_below_kb() {
        assert_eq!(format_bytes(1023), "1023 B");
    }

    #[test]
    fn format_bytes_boundary_below_mb() {
        let val = 1_048_575; // 1MB - 1
        let result = format_bytes(val);
        assert!(result.contains("KB"));
    }

    #[test]
    fn format_bytes_boundary_below_gb() {
        let val = 1_073_741_823; // 1GB - 1
        let result = format_bytes(val);
        assert!(result.contains("MB"));
    }

    // ── count_items ─────────────────────────────────────────────────────

    #[test]
    fn count_items_empty_dir() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(count_items(tmp.path()), 0);
    }

    #[test]
    fn count_items_with_files() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("a.txt"), "hello").unwrap();
        fs::write(tmp.path().join("b.txt"), "world").unwrap();
        assert_eq!(count_items(tmp.path()), 2);
    }

    #[test]
    fn count_items_with_dirs_and_files() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("a.txt"), "").unwrap();
        fs::create_dir(tmp.path().join("subdir")).unwrap();
        assert_eq!(count_items(tmp.path()), 2);
    }

    #[test]
    fn count_items_nonexistent() {
        assert_eq!(count_items(Path::new("/nonexistent/path/99999")), 0);
    }

    // ── dir_size ────────────────────────────────────────────────────────

    #[test]
    fn dir_size_empty() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(dir_size(tmp.path()), 0);
    }

    #[test]
    fn dir_size_single_file() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("data.bin"), vec![0u8; 256]).unwrap();
        assert_eq!(dir_size(tmp.path()), 256);
    }

    #[test]
    fn dir_size_nested() {
        let tmp = tempfile::tempdir().unwrap();
        let sub = tmp.path().join("a").join("b");
        fs::create_dir_all(&sub).unwrap();
        fs::write(sub.join("file.dat"), vec![1u8; 100]).unwrap();
        fs::write(tmp.path().join("root.dat"), vec![2u8; 50]).unwrap();
        assert_eq!(dir_size(tmp.path()), 150);
    }

    #[test]
    fn dir_size_nonexistent_returns_zero() {
        assert_eq!(dir_size(Path::new("/nonexistent/path/99999")), 0);
    }

    // ── check_toolchains ────────────────────────────────────────────────

    #[test]
    fn check_toolchains_returns_result() {
        let result = check_toolchains();
        assert_eq!(result.category, "Toolchains");
        assert!(
            result.summary.contains("tools found"),
            "summary should mention tools found: {}",
            result.summary
        );
        assert!(!result.details.is_empty());
    }

    #[test]
    fn check_toolchains_details_mention_tool_names() {
        let result = check_toolchains();
        let combined: String = result.details.join("\n");
        assert!(combined.contains("cargo"), "should list cargo");
        assert!(combined.contains("rustc"), "should list rustc");
        assert!(combined.contains("python3"), "should list python3");
    }

    // ── check_manifest ──────────────────────────────────────────────────

    #[test]
    fn check_manifest_no_horus_toml() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: false,
            manifest: None,
        };
        let result = check_manifest(&ctx);
        assert_eq!(result.category, "Manifest");
        assert_eq!(result.health, Health::Warn);
        assert!(result.summary.contains("No horus.toml"));
    }

    #[test]
    fn check_manifest_has_toml_but_no_manifest() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: None,
        };
        let result = check_manifest(&ctx);
        assert_eq!(result.health, Health::Fail);
        assert!(result.summary.contains("Failed to parse"));
    }

    #[test]
    fn check_manifest_valid_manifest() {
        let manifest = make_manifest("test-proj");
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
        };
        let result = check_manifest(&ctx);
        assert_eq!(result.category, "Manifest");
        assert!(result.health == Health::Ok || result.health == Health::Warn);
    }

    #[test]
    fn check_manifest_valid_has_details() {
        let manifest = make_manifest("mybot");
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
        };
        let result = check_manifest(&ctx);
        let details = result.details.join("\n");
        assert!(
            details.contains("mybot"),
            "details should include package name"
        );
    }

    // ── check_shm ───────────────────────────────────────────────────────

    #[test]
    fn check_shm_returns_result() {
        let result = check_shm();
        assert_eq!(result.category, "Shared Memory");
        let shm_parent = horus_sys::shm::shm_parent_dir();
        if shm_parent.exists() {
            assert_eq!(result.health, Health::Ok);
            assert!(result.summary.contains("available"));
        } else {
            assert_eq!(result.health, Health::Warn);
        }
    }

    // ── check_plugins ───────────────────────────────────────────────────

    #[test]
    fn check_plugins_returns_result() {
        let result = check_plugins();
        assert_eq!(result.category, "Plugins");
        assert_eq!(result.health, Health::Ok);
        assert!(result.summary.contains("global"));
        assert!(result.summary.contains("local"));
    }

    // ── check_disk ──────────────────────────────────────────────────────

    #[test]
    fn check_disk_returns_result() {
        let _lock = crate::CWD_LOCK.lock();
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = check_disk();
        assert_eq!(result.category, "Disk");
        assert_eq!(result.health, Health::Ok);
        assert!(result.summary.contains("No .horus/"));

        std::env::set_current_dir(old_dir).unwrap();
    }

    #[test]
    fn check_disk_with_horus_dir() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        fs::create_dir(tmp.path().join(".horus")).unwrap();
        fs::write(tmp.path().join(".horus/test.dat"), vec![0u8; 1024]).unwrap();

        let result = check_disk();
        assert_eq!(result.category, "Disk");
        assert_eq!(result.health, Health::Ok);

        std::env::set_current_dir(old_dir).unwrap();
    }

    // ── check_languages ─────────────────────────────────────────────────

    #[test]
    fn check_languages_empty() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: false,
            manifest: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.category, "Languages");
        assert_eq!(result.health, Health::Warn);
        assert!(result.summary.contains("No languages"));
    }

    #[test]
    fn check_languages_rust_only() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Rust],
            has_horus_toml: false,
            manifest: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
    }

    #[test]
    fn check_languages_python_only() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Python],
            has_horus_toml: false,
            manifest: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
    }

    #[test]
    fn check_languages_mixed() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![
                crate::manifest::Language::Rust,
                crate::manifest::Language::Python,
            ],
            has_horus_toml: false,
            manifest: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
        let summary = result.summary.to_lowercase();
        assert!(summary.contains("rust") || summary.contains("python"));
    }

    // ── print_json ──────────────────────────────────────────────────────

    #[test]
    fn print_json_produces_valid_json() {
        let results = vec![
            CheckResult {
                category: "Test".to_string(),
                health: Health::Ok,
                summary: "all good".to_string(),
                details: vec!["detail1".to_string()],
            },
            CheckResult {
                category: "Another".to_string(),
                health: Health::Fail,
                summary: "broken".to_string(),
                details: vec![],
            },
        ];
        print_json(&results);
    }

    #[test]
    fn print_json_empty_results() {
        print_json(&[]);
    }

    // ── print_summary ───────────────────────────────────────────────────

    #[test]
    fn print_summary_no_issues() {
        let results = vec![CheckResult {
            category: "Check1".to_string(),
            health: Health::Ok,
            summary: "fine".to_string(),
            details: vec![],
        }];
        print_summary(&results, false);
    }

    #[test]
    fn print_summary_verbose_shows_details() {
        let results = vec![CheckResult {
            category: "Check1".to_string(),
            health: Health::Warn,
            summary: "warning".to_string(),
            details: vec!["some detail".to_string()],
        }];
        print_summary(&results, true);
    }

    #[test]
    fn print_summary_mixed_health() {
        let results = vec![
            CheckResult {
                category: "Ok".to_string(),
                health: Health::Ok,
                summary: "fine".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "Warn".to_string(),
                health: Health::Warn,
                summary: "warning".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "Fail".to_string(),
                health: Health::Fail,
                summary: "broken".to_string(),
                details: vec![],
            },
        ];
        print_summary(&results, false);
    }

    #[test]
    fn print_summary_all_ok() {
        let results = vec![
            CheckResult {
                category: "A".to_string(),
                health: Health::Ok,
                summary: "ok".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "B".to_string(),
                health: Health::Ok,
                summary: "ok".to_string(),
                details: vec![],
            },
        ];
        print_summary(&results, false);
    }

    #[test]
    fn print_summary_warnings_only() {
        let results = vec![
            CheckResult {
                category: "A".to_string(),
                health: Health::Ok,
                summary: "ok".to_string(),
                details: vec![],
            },
            CheckResult {
                category: "B".to_string(),
                health: Health::Warn,
                summary: "warn".to_string(),
                details: vec![],
            },
        ];
        print_summary(&results, false);
    }

    // ── CheckResult construction ────────────────────────────────────────

    #[test]
    fn check_result_fields() {
        let cr = CheckResult {
            category: "Cat".to_string(),
            health: Health::Ok,
            summary: "Sum".to_string(),
            details: vec!["d1".to_string(), "d2".to_string()],
        };
        assert_eq!(cr.category, "Cat");
        assert_eq!(cr.health, Health::Ok);
        assert_eq!(cr.summary, "Sum");
        assert_eq!(cr.details.len(), 2);
    }

    #[test]
    fn check_result_empty_details() {
        let cr = CheckResult {
            category: "X".to_string(),
            health: Health::Fail,
            summary: "bad".to_string(),
            details: vec![],
        };
        assert!(cr.details.is_empty());
    }

    // ── Edge cases / battle tests ───────────────────────────────────────

    #[test]
    fn format_bytes_max_u64() {
        let result = format_bytes(u64::MAX);
        assert!(result.contains("GB"), "huge value should be GB: {}", result);
    }

    #[test]
    fn format_bytes_one_byte() {
        assert_eq!(format_bytes(1), "1 B");
    }

    #[test]
    fn health_all_variants_covered() {
        let variants = [Health::Ok, Health::Warn, Health::Fail];
        for v in &variants {
            let _ = v.icon();
            let _ = format!("{:?}", v);
        }
    }

    #[test]
    fn check_manifest_with_dependencies() {
        let mut manifest = make_manifest("dep-proj");
        manifest.dependencies.insert(
            "serde".to_string(),
            crate::manifest::DependencyValue::Simple("1.0".to_string()),
        );
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
        };
        let result = check_manifest(&ctx);
        let details = result.details.join("\n");
        assert!(details.contains("dependencies: 1"));
    }

    #[test]
    fn count_items_deeply_nested() {
        let tmp = tempfile::tempdir().unwrap();
        let deep = tmp.path().join("a").join("b").join("c");
        fs::create_dir_all(&deep).unwrap();
        fs::write(deep.join("file.txt"), "data").unwrap();
        // count_items only counts direct children, not recursive
        assert_eq!(count_items(tmp.path()), 1); // just "a"
    }

    #[test]
    fn dir_size_many_small_files() {
        let tmp = tempfile::tempdir().unwrap();
        for i in 0..50 {
            fs::write(tmp.path().join(format!("f{}.txt", i)), "x").unwrap();
        }
        assert_eq!(dir_size(tmp.path()), 50);
    }

    #[test]
    fn check_disk_large_threshold() {
        let _lock = crate::CWD_LOCK.lock();
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = check_disk();
        assert_eq!(result.health, Health::Ok);

        std::env::set_current_dir(old_dir).unwrap();
    }

    #[test]
    fn check_languages_single_cpp() {
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Cpp],
            has_horus_toml: false,
            manifest: None,
        };
        let result = check_languages(&ctx);
        assert_eq!(result.health, Health::Ok);
    }

    #[test]
    fn check_manifest_invalid_name() {
        // name "x" is too short (< 2 chars), should produce validation error or warning
        let manifest = make_manifest("x");
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![],
            has_horus_toml: true,
            manifest: Some(manifest),
        };
        let result = check_manifest(&ctx);
        // Validation should catch the short name
        assert!(
            result.health == Health::Warn || result.health == Health::Fail,
            "short name should trigger warn or fail"
        );
    }

    #[test]
    fn check_manifest_with_drivers_and_scripts() {
        let mut manifest = make_manifest("robot-arm");
        manifest.drivers.insert(
            "lidar".to_string(),
            crate::manifest::DriverValue::Backend("rplidar".to_string()),
        );
        manifest
            .scripts
            .insert("build".to_string(), "cargo build".to_string());
        let ctx = dispatch::ProjectContext {
            root: PathBuf::from("/tmp/fake"),
            languages: vec![crate::manifest::Language::Rust],
            has_horus_toml: true,
            manifest: Some(manifest),
        };
        let result = check_manifest(&ctx);
        // Should parse fine
        assert!(result.health == Health::Ok || result.health == Health::Warn);
    }

    #[test]
    fn check_shm_category_name() {
        let result = check_shm();
        assert_eq!(result.category, "Shared Memory");
    }

    #[test]
    fn check_disk_category_name() {
        let _lock = crate::CWD_LOCK.lock();
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = check_disk();
        assert_eq!(result.category, "Disk");
        std::env::set_current_dir(old_dir).unwrap();
    }

    #[test]
    fn check_languages_all_supported() {
        // Test with all known language variants
        for lang in &[
            crate::manifest::Language::Rust,
            crate::manifest::Language::Python,
            crate::manifest::Language::Cpp,
        ] {
            let ctx = dispatch::ProjectContext {
                root: PathBuf::from("/tmp/fake"),
                languages: vec![lang.clone()],
                has_horus_toml: false,
                manifest: None,
            };
            let result = check_languages(&ctx);
            assert_eq!(result.health, Health::Ok);
        }
    }

    #[test]
    fn print_json_single_result() {
        let results = vec![CheckResult {
            category: "Solo".to_string(),
            health: Health::Warn,
            summary: "needs attention".to_string(),
            details: vec!["fix this".to_string(), "and this".to_string()],
        }];
        print_json(&results);
    }

    #[test]
    fn check_result_many_details() {
        let cr = CheckResult {
            category: "Verbose".to_string(),
            health: Health::Ok,
            summary: "lots of info".to_string(),
            details: (0..100).map(|i| format!("detail {}", i)).collect(),
        };
        assert_eq!(cr.details.len(), 100);
    }

    #[test]
    fn format_bytes_exact_boundaries() {
        // Exactly at each boundary
        assert!(format_bytes(1024).contains("KB"));
        assert!(format_bytes(1_048_576).contains("MB"));
        assert!(format_bytes(1_073_741_824).contains("GB"));
    }

    // ── Driver device reachability tests ──────────────────────────────

    #[test]
    fn check_driver_device_existing_path() {
        let mut map = std::collections::HashMap::new();
        map.insert("port".to_string(), toml::Value::String("/dev/null".into()));
        let params = horus_core::drivers::DriverParams::new(map);
        let (detail, health) =
            check_driver_device("test", &params, Some(&DriverType::Terra("serial".into())));
        assert_eq!(health, Health::Ok);
        assert!(detail.contains("found"), "detail: {}", detail);
    }

    #[test]
    fn check_driver_device_missing_path() {
        let mut map = std::collections::HashMap::new();
        map.insert(
            "port".to_string(),
            toml::Value::String("/dev/nonexistent_xyz_test".into()),
        );
        let params = horus_core::drivers::DriverParams::new(map);
        let (detail, health) =
            check_driver_device("test", &params, Some(&DriverType::Terra("serial".into())));
        assert_eq!(health, Health::Fail);
        assert!(detail.contains("not found"), "detail: {}", detail);
    }

    #[test]
    fn check_driver_device_i2c_bus() {
        let mut map = std::collections::HashMap::new();
        map.insert("bus".to_string(), toml::Value::String("i2c-99".into()));
        let params = horus_core::drivers::DriverParams::new(map);
        let (detail, health) =
            check_driver_device("imu", &params, Some(&DriverType::Terra("mpu6050".into())));
        // /dev/i2c-99 almost certainly doesn't exist
        assert_eq!(health, Health::Fail);
        assert!(detail.contains("i2c-99"), "detail: {}", detail);
    }

    #[test]
    fn check_driver_device_no_checkable_params() {
        let params = horus_core::drivers::DriverParams::empty();
        let (detail, health) = check_driver_device(
            "mystery",
            &params,
            Some(&DriverType::Local("MyDriver".into())),
        );
        assert_eq!(health, Health::Ok);
        assert!(detail.contains("no device path"), "detail: {}", detail);
    }

    #[test]
    fn check_driver_device_legacy_type() {
        let params = horus_core::drivers::DriverParams::empty();
        let (detail, health) = check_driver_device("cam", &params, Some(&DriverType::Legacy));
        assert_eq!(health, Health::Ok);
        assert!(detail.contains("legacy"), "detail: {}", detail);
    }

    #[test]
    fn check_driver_device_network_unreachable() {
        let mut map = std::collections::HashMap::new();
        // Use a non-routable address so connect_timeout fails fast
        map.insert(
            "address".to_string(),
            toml::Value::String("192.0.2.1:9999".into()),
        );
        let params = horus_core::drivers::DriverParams::new(map);
        let (detail, health) = check_driver_device(
            "lidar",
            &params,
            Some(&DriverType::Terra("velodyne".into())),
        );
        assert_eq!(health, Health::Warn);
        assert!(detail.contains("unreachable"), "detail: {}", detail);
    }
}
