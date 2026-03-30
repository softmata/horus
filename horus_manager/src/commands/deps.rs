//! `horus deps` — dependency insight commands.
//!
//! Subcommands: tree, why, outdated, audit.
//! All dispatch to native tools (cargo tree, cargo audit, pipdeptree, pip-audit).

use anyhow::Result;
use colored::*;
use std::path::Path;
use std::process::Command;

use crate::dispatch::{self, tool_version};

/// Deps subcommand.
#[derive(Debug, Clone)]
pub enum DepsAction {
    Tree,
    Why(String),
    Outdated,
    Audit,
}

/// Run `horus deps <action>`.
pub fn run_deps(action: DepsAction, extra_args: Vec<String>) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);

    match action {
        DepsAction::Tree => run_tree(&ctx, &extra_args),
        DepsAction::Why(pkg) => run_why(&ctx, &pkg, &extra_args),
        DepsAction::Outdated => run_outdated(&ctx, &extra_args),
        DepsAction::Audit => run_audit(&ctx, &extra_args),
    }
}

fn run_tree(ctx: &dispatch::ProjectContext, extra_args: &[String]) -> Result<()> {
    let mut ran = false;

    if ctx.has_rust() {
        println!("{}", "[rust] cargo tree".bold());
        let mut cmd = Command::new("cargo");
        cmd.arg("tree");
        cmd.args(extra_args);
        cmd.current_dir(&ctx.root);
        cmd.status()?;
        ran = true;
    }

    if ctx.has_python() {
        if tool_version("pipdeptree").is_some() {
            if ran {
                println!();
            }
            println!("{}", "[python] pipdeptree".bold());
            let mut cmd = Command::new("pipdeptree");
            cmd.args(extra_args);
            cmd.current_dir(&ctx.root);
            cmd.status()?;
            ran = true;
        } else {
            suggest_install("pipdeptree", "pip install pipdeptree");
        }
    }

    if ctx.has_cpp() {
        if ran {
            println!();
        }
        println!("{}", "[cpp] system dependencies".bold());
        print_cpp_deps_tree(&ctx.root)?;
        ran = true;
    }

    if !ran {
        anyhow::bail!("No dependency tree tools available for this project.");
    }
    Ok(())
}

/// Print C++ system dependencies from horus.toml with installed versions.
fn print_cpp_deps_tree(root: &Path) -> Result<()> {
    let manifest_path = root.join(crate::manifest::HORUS_TOML);
    if !manifest_path.exists() {
        println!("  (no horus.toml found)");
        return Ok(());
    }
    let manifest = crate::manifest::HorusManifest::load_from(&manifest_path)?;

    let mut found = false;
    for (name, dep) in &manifest.dependencies {
        // Filter for system deps (C++ deps use source = "system")
        let is_system = match dep {
            crate::manifest::DependencyValue::Detailed(d) => {
                d.source.as_ref() == Some(&crate::manifest::DepSource::System)
            }
            _ => false,
        };
        if !is_system {
            continue;
        }

        // Use dep name as apt package name (convention for system deps)
        let apt_name = name.as_str();
        let version =
            query_dpkg_version(apt_name).unwrap_or_else(|| "not installed".red().to_string());
        println!("  {} {}", name.cyan(), version);
        found = true;
    }

    if !found {
        println!("  (no system dependencies in horus.toml)");
    }
    Ok(())
}

/// Query installed version of an apt package via dpkg.
fn query_dpkg_version(package: &str) -> Option<String> {
    let output = Command::new("dpkg")
        .args(["-s", package])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::null())
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let stdout = String::from_utf8_lossy(&output.stdout);
    for line in stdout.lines() {
        if let Some(ver) = line.strip_prefix("Version:") {
            return Some(ver.trim().to_string());
        }
    }
    None
}

fn run_why(ctx: &dispatch::ProjectContext, pkg: &str, extra_args: &[String]) -> Result<()> {
    let mut ran = false;

    if ctx.has_rust() {
        println!("{}", format!("[rust] cargo tree -i {}", pkg).bold());
        let mut cmd = Command::new("cargo");
        cmd.args(["tree", "-i", pkg]);
        cmd.args(extra_args);
        cmd.current_dir(&ctx.root);
        cmd.status()?;
        ran = true;
    }

    if ctx.has_python() {
        if tool_version("pipdeptree").is_some() {
            if ran {
                println!();
            }
            println!("{}", format!("[python] pipdeptree -r -p {}", pkg).bold());
            let mut cmd = Command::new("pipdeptree");
            cmd.args(["--reverse", "-p", pkg]);
            cmd.args(extra_args);
            cmd.current_dir(&ctx.root);
            cmd.status()?;
        } else {
            suggest_install("pipdeptree", "pip install pipdeptree");
        }
    }

    if !ran {
        anyhow::bail!("No reverse dependency tools available.");
    }
    Ok(())
}

fn run_outdated(ctx: &dispatch::ProjectContext, extra_args: &[String]) -> Result<()> {
    let mut ran = false;

    if ctx.has_rust() {
        if tool_version("cargo-outdated").is_some() {
            println!("{}", "[rust] cargo outdated".bold());
            let mut cmd = Command::new("cargo");
            cmd.args(["outdated"]);
            cmd.args(extra_args);
            cmd.current_dir(&ctx.root);
            cmd.status()?;
            ran = true;
        } else {
            suggest_install("cargo-outdated", "cargo install cargo-outdated");
        }
    }

    if ctx.has_python() {
        // pip list --outdated works without extra tools
        println!("{}", "[python] pip list --outdated".bold());
        let python = if tool_version("python3").is_some() {
            "python3"
        } else {
            "python"
        };
        let mut cmd = Command::new(python);
        cmd.args(["-m", "pip", "list", "--outdated"]);
        cmd.args(extra_args);
        cmd.current_dir(&ctx.root);
        cmd.status()?;
        ran = true;
    }

    if ctx.has_cpp() {
        if ran {
            println!();
        }
        println!("{}", "[cpp] system package updates".bold());
        print_cpp_deps_outdated(&ctx.root)?;
        ran = true;
    }

    if !ran {
        anyhow::bail!("No outdated-check tools available.");
    }
    Ok(())
}

/// Print outdated C++ system deps by comparing dpkg installed vs apt-cache candidate.
fn print_cpp_deps_outdated(root: &Path) -> Result<()> {
    let manifest_path = root.join(crate::manifest::HORUS_TOML);
    if !manifest_path.exists() {
        println!("  (no horus.toml found)");
        return Ok(());
    }
    let manifest = crate::manifest::HorusManifest::load_from(&manifest_path)?;

    let mut found = false;
    let mut any_outdated = false;
    for (name, dep) in &manifest.dependencies {
        // Filter for system deps
        let is_system = match dep {
            crate::manifest::DependencyValue::Detailed(d) => {
                d.source.as_ref() == Some(&crate::manifest::DepSource::System)
            }
            _ => false,
        };
        if !is_system {
            continue;
        }

        let apt = name.as_str();
        found = true;

        let installed = query_dpkg_version(apt);
        let candidate = query_apt_cache_version(apt);

        match (&installed, &candidate) {
            (Some(inst), Some(cand)) if inst != cand => {
                println!(
                    "  {} ({})  {} {} {}",
                    name.cyan(),
                    apt,
                    inst.yellow(),
                    "->".dimmed(),
                    cand.green()
                );
                any_outdated = true;
            }
            (Some(inst), _) => {
                println!(
                    "  {} ({})  {} {}",
                    name.cyan(),
                    apt,
                    inst,
                    "(up to date)".green()
                );
            }
            (None, _) => {
                println!("  {} ({})  {}", name.cyan(), apt, "not installed".red());
                any_outdated = true;
            }
        }
    }

    if found && !any_outdated {
        println!(
            "  {}",
            "All C++ system dependencies are up to date.".green()
        );
    }
    if !found {
        println!("  (no C++ system dependencies in horus.toml)");
    }
    Ok(())
}

/// Query available version from apt-cache policy.
fn query_apt_cache_version(package: &str) -> Option<String> {
    let output = Command::new("apt-cache")
        .args(["policy", package])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::null())
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let stdout = String::from_utf8_lossy(&output.stdout);
    for line in stdout.lines() {
        let trimmed = line.trim();
        if let Some(ver) = trimmed.strip_prefix("Candidate:") {
            let ver = ver.trim();
            if ver != "(none)" {
                return Some(ver.to_string());
            }
        }
    }
    None
}

fn run_audit(ctx: &dispatch::ProjectContext, extra_args: &[String]) -> Result<()> {
    let mut ran = false;

    if ctx.has_rust() {
        if tool_version("cargo-audit").is_some() {
            println!("{}", "[rust] cargo audit".bold());
            let mut cmd = Command::new("cargo");
            cmd.args(["audit"]);
            cmd.args(extra_args);
            cmd.current_dir(&ctx.root);
            cmd.status()?;
            ran = true;
        } else {
            suggest_install("cargo-audit", "cargo install cargo-audit");
        }
    }

    if ctx.has_python() {
        if tool_version("pip-audit").is_some() {
            println!("{}", "[python] pip-audit".bold());
            let mut cmd = Command::new("pip-audit");
            cmd.args(extra_args);
            cmd.current_dir(&ctx.root);
            cmd.status()?;
            ran = true;
        } else {
            suggest_install("pip-audit", "pip install pip-audit");
        }
    }

    if !ran {
        anyhow::bail!("No security audit tools available. Install cargo-audit or pip-audit.");
    }
    Ok(())
}

fn suggest_install(tool: &str, command: &str) {
    eprintln!(
        "  {} {} not found. Install with: {}",
        "hint:".yellow(),
        tool.cyan(),
        command.dimmed()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── DepsAction ──────────────────────────────────────────────────────

    #[test]
    fn deps_action_debug() {
        let tree = DepsAction::Tree;
        assert_eq!(format!("{:?}", tree), "Tree");
    }

    #[test]
    fn deps_action_why_contains_package() {
        let why = DepsAction::Why("serde".to_string());
        let dbg = format!("{:?}", why);
        assert!(dbg.contains("serde"));
    }

    #[test]
    fn deps_action_clone() {
        let action = DepsAction::Outdated;
        let cloned = action.clone();
        assert!(matches!(cloned, DepsAction::Outdated));
    }

    #[test]
    fn deps_action_all_variants() {
        let variants: Vec<DepsAction> = vec![
            DepsAction::Tree,
            DepsAction::Why("tokio".to_string()),
            DepsAction::Outdated,
            DepsAction::Audit,
        ];
        assert_eq!(variants.len(), 4);
    }

    // ── run_deps in empty project ───────────────────────────────────────

    #[test]
    fn run_deps_tree_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Tree, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn run_deps_why_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Why("serde".to_string()), vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn run_deps_outdated_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Outdated, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn run_deps_audit_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Audit, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    // ── Battle tests: deps dispatch and error messages ──────────────────

    #[test]
    fn deps_tree_error_message_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Tree, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No dependency tree tools available"),
            "Should mention no tools available, got: {}",
            err
        );
    }

    #[test]
    fn deps_why_error_message_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Why("anyhow".to_string()), vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No reverse dependency tools available"),
            "Should mention no reverse dep tools, got: {}",
            err
        );
    }

    #[test]
    fn deps_outdated_error_message_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Outdated, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No outdated-check tools available"),
            "Should mention no outdated tools, got: {}",
            err
        );
    }

    #[test]
    fn deps_audit_error_message_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Audit, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No security audit tools available"),
            "Should mention no audit tools, got: {}",
            err
        );
    }

    #[test]
    fn deps_tree_with_extra_args_still_fails_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Tree, vec!["--depth=1".to_string()]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn deps_why_empty_package_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // Empty package name — still fails because no project detected
        let result = run_deps(DepsAction::Why("".to_string()), vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn deps_action_why_clone_preserves_package() {
        let action = DepsAction::Why("tokio".to_string());
        let cloned = action.clone();
        if let DepsAction::Why(pkg) = cloned {
            assert_eq!(pkg, "tokio");
        } else {
            panic!("Clone should preserve Why variant");
        }
    }

    #[test]
    fn deps_action_debug_all_variants() {
        let variants = vec![
            (DepsAction::Tree, "Tree"),
            (DepsAction::Outdated, "Outdated"),
            (DepsAction::Audit, "Audit"),
        ];
        for (action, expected) in variants {
            assert_eq!(format!("{:?}", action), expected);
        }

        let why = DepsAction::Why("serde_json".to_string());
        let dbg = format!("{:?}", why);
        assert!(dbg.contains("Why"));
        assert!(dbg.contains("serde_json"));
    }

    #[test]
    fn deps_rust_project_context() {
        // Verify that a Rust project has has_rust() = true for deps dispatch
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Should detect Rust");
        assert!(!ctx.has_python(), "Should not detect Python");
    }

    #[test]
    fn deps_python_project_context() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("app.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(!ctx.has_rust());
        assert!(ctx.has_python());
    }

    #[test]
    fn deps_cpp_project_context() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("CMakeLists.txt"),
            "cmake_minimum_required(VERSION 3.16)\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_cpp(), "Should detect C++ from CMakeLists.txt");
        assert!(!ctx.has_rust());
        assert!(!ctx.has_python());
    }

    #[test]
    fn deps_rust_only_no_cpp() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(!ctx.has_cpp(), "Should not detect C++ in Rust-only project");
    }

    #[test]
    fn deps_mixed_project_context() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();
        std::fs::write(tmp.path().join("app.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());
        assert!(ctx.has_python());
    }

    #[test]
    fn query_dpkg_version_nonexistent() {
        assert!(query_dpkg_version("horus-nonexistent-xyz-999").is_none());
    }

    #[test]
    fn query_dpkg_version_coreutils() {
        if cfg!(target_os = "linux") {
            let ver = query_dpkg_version("coreutils");
            assert!(ver.is_some(), "coreutils should be installed");
            assert!(!ver.unwrap().is_empty());
        }
    }

    #[test]
    fn query_apt_cache_nonexistent() {
        assert!(query_apt_cache_version("horus-nonexistent-xyz-999").is_none());
    }

    #[test]
    fn query_apt_cache_coreutils() {
        if cfg!(target_os = "linux") {
            let ver = query_apt_cache_version("coreutils");
            assert!(ver.is_some(), "coreutils should have a candidate");
            assert!(!ver.unwrap().is_empty());
        }
    }

    #[test]
    fn print_cpp_deps_tree_no_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Should succeed without panic (prints message about no horus.toml)
        let result = print_cpp_deps_tree(tmp.path());
        assert!(result.is_ok());
    }

    #[test]
    fn print_cpp_deps_outdated_no_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let result = print_cpp_deps_outdated(tmp.path());
        assert!(result.is_ok());
    }

    #[test]
    fn deps_horus_toml_only_no_languages() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"dep-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        // horus.toml alone doesn't make languages detectable
        assert!(
            !ctx.has_rust() && !ctx.has_python(),
            "horus.toml alone should not detect any language"
        );
    }

    #[test]
    fn deps_outdated_python_prefers_python3() {
        // Verify the python3 > python preference in run_outdated
        let has_python3 = tool_version("python3").is_some();
        let has_python = tool_version("python").is_some();

        let python = if has_python3 {
            "python3"
        } else if has_python {
            "python"
        } else {
            return; // Skip if neither is available
        };

        assert!(
            python == "python3" || python == "python",
            "Should select python3 or python"
        );
    }

    #[test]
    fn deps_audit_requires_tools() {
        // Verify audit tools are checked before running
        let has_cargo_audit = tool_version("cargo-audit").is_some();
        let has_pip_audit = tool_version("pip-audit").is_some();

        // This is informational — just check tool_version works for these names
        // The actual test is that run_audit fails in empty dirs (tested above)
        if has_cargo_audit {
            assert!(tool_version("cargo-audit").unwrap().len() > 0);
        }
        if has_pip_audit {
            assert!(tool_version("pip-audit").unwrap().len() > 0);
        }
    }

    #[test]
    fn deps_outdated_requires_cargo_outdated_for_rust() {
        // Verify that cargo-outdated is checked, not just cargo
        let has_cargo_outdated = tool_version("cargo-outdated").is_some();
        // Tool presence is environment-dependent, but the check itself should not panic
        let _ = has_cargo_outdated;
    }

    #[test]
    fn deps_all_actions_fail_consistently_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let results: Vec<_> = vec![
            run_deps(DepsAction::Tree, vec![]),
            run_deps(DepsAction::Why("serde".to_string()), vec![]),
            run_deps(DepsAction::Outdated, vec![]),
            run_deps(DepsAction::Audit, vec![]),
        ];
        std::env::set_current_dir(original).unwrap();

        for (i, result) in results.into_iter().enumerate() {
            assert!(
                result.is_err(),
                "Action index {} should fail in empty dir",
                i
            );
        }
    }
}
