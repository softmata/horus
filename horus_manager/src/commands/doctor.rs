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
enum Health {
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
struct CheckResult {
    category: String,
    health: Health,
    summary: String,
    details: Vec<String>,
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

fn print_summary(results: &[CheckResult], verbose: bool) {
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
        summary: format!(".horus/ uses {}", formatted),
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
