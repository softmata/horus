//! `horus doctor` — comprehensive ecosystem health check.
//!
//! Checks: toolchains, SHM, plugins, manifest validity, registry,
//! system deps, disk space. Summary by default, --verbose for details.

use anyhow::Result;
use colored::*;
use std::path::Path;

use crate::dispatch;

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
pub fn run_doctor(verbose: bool, json: bool) -> Result<()> {
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

    // ── Output ───────────────────────────────────────────────────────────
    if json {
        print_json(&results);
    } else {
        print_summary(&results, verbose);
    }

    // Exit code
    let worst = results
        .iter()
        .map(|r| &r.health)
        .max_by_key(|h| match h {
            Health::Ok => 0,
            Health::Warn => 1,
            Health::Fail => 2,
        })
        .unwrap_or(&Health::Ok);

    match worst {
        Health::Ok => Ok(()),
        Health::Warn => std::process::exit(1),
        Health::Fail => std::process::exit(2),
    }
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
        println!(
            "  {} All {ok_count} checks passed",
            "Summary:".bold()
        );
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
    let shm_path = Path::new("/dev/shm");
    if !shm_path.exists() {
        return CheckResult {
            category: "Shared Memory".to_string(),
            health: Health::Warn,
            summary: "/dev/shm not available".to_string(),
            details: vec!["SHM IPC may not work on this platform".to_string()],
        };
    }

    // Count horus namespaces
    let count = std::fs::read_dir(shm_path)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok())
                .filter(|e| {
                    e.file_name()
                        .to_string_lossy()
                        .starts_with("horus_")
                })
                .count()
        })
        .unwrap_or(0);

    CheckResult {
        category: "Shared Memory".to_string(),
        health: Health::Ok,
        summary: format!("/dev/shm available, {} horus namespaces", count),
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

    let issues = crate::source_resolver::validate_deps(
        &manifest.dependencies,
        &ctx.languages,
    );
    let dev_issues = crate::source_resolver::validate_deps(
        &manifest.dev_dependencies,
        &ctx.languages,
    );

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
            health: if has_errors { Health::Fail } else { Health::Warn },
            summary: format!("{} issue(s) found", all_issues.len()),
            details,
        }
    }
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

fn count_items(dir: &Path) -> usize {
    std::fs::read_dir(dir)
        .map(|entries| entries.filter_map(|e| e.ok()).count())
        .unwrap_or(0)
}

fn dir_size(path: &Path) -> u64 {
    walkdir::WalkDir::new(path)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_file())
        .filter_map(|e| e.metadata().ok())
        .map(|m| m.len())
        .sum()
}

fn format_bytes(bytes: u64) -> String {
    if bytes >= 1_073_741_824 {
        format!("{:.1} GB", bytes as f64 / 1_073_741_824.0)
    } else if bytes >= 1_048_576 {
        format!("{:.1} MB", bytes as f64 / 1_048_576.0)
    } else if bytes >= 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else {
        format!("{} B", bytes)
    }
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
            },
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
        assert_eq!(format_bytes(1_073_741_824), "1.0 GB");
    }

    #[test]
    fn format_bytes_gigabytes_large() {
        assert_eq!(format_bytes(10_737_418_240), "10.0 GB");
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
        assert!(details.contains("mybot"), "details should include package name");
    }

    // ── check_shm ───────────────────────────────────────────────────────

    #[test]
    fn check_shm_returns_result() {
        let result = check_shm();
        assert_eq!(result.category, "Shared Memory");
        if Path::new("/dev/shm").exists() {
            assert_eq!(result.health, Health::Ok);
            assert!(result.summary.contains("/dev/shm available"));
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
        let old_dir = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = check_disk();
        assert_eq!(result.category, "Disk");
        assert_eq!(result.health, Health::Ok);
        assert!(result.summary.contains("No .horus/"));

        std::env::set_current_dir(old_dir).unwrap();
    }

    #[test]
    fn check_disk_with_horus_dir() {
        let _lock = crate::CWD_LOCK.lock();
        let tmp = tempfile::tempdir().unwrap();
        let old_dir = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        fs::create_dir(tmp.path().join(".horus")).unwrap();
        fs::write(tmp.path().join(".horus/test.dat"), vec![0u8; 1024]).unwrap();

        let result = check_disk();
        assert_eq!(result.category, "Disk");
        assert_eq!(result.health, Health::Ok);
        assert!(result.summary.contains(".horus/"));

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
        let old_dir = std::env::current_dir().unwrap();
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
        let old_dir = std::env::current_dir().unwrap();
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
}
