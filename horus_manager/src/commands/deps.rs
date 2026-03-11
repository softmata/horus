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
