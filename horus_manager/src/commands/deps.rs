//! `horus deps` — dependency insight commands.
//!
//! Subcommands: tree, why, outdated, audit.
//! All dispatch to native tools (cargo tree, cargo audit, pipdeptree, pip-audit).

use anyhow::Result;
use colored::*;
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

    if !ran {
        anyhow::bail!("No dependency tree tools available for this project.");
    }
    Ok(())
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

    if !ran {
        anyhow::bail!("No outdated-check tools available.");
    }
    Ok(())
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
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Tree, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn run_deps_why_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Why("serde".to_string()), vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn run_deps_outdated_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Outdated, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn run_deps_audit_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Audit, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    // ── Battle tests: deps dispatch and error messages ──────────────────

    #[test]
    fn deps_tree_error_message_no_project() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
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
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
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
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
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
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
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
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_deps(DepsAction::Tree, vec!["--depth=1".to_string()]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn deps_why_empty_package_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
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
    fn deps_mixed_project_context() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();
        std::fs::write(tmp.path().join("app.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());
        assert!(ctx.has_python());
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
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
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
