//! `horus lint` — lint all code in the project.
//!
//! Dispatches to `cargo clippy` (Rust) and `ruff check` / `pylint` (Python).
//! Supports `--fix` for auto-fix and `--types` for Python type checking.

use anyhow::Result;
use colored::*;

use crate::dispatch::{self, Operation, ProjectContext};
use crate::run_with_prefix::{self, PrefixedCommand};

/// Run `horus lint`.
///
/// - `fix`: If true, auto-fix lint issues where possible.
/// - `types`: If true, also run Python type checker (mypy/pyright).
/// - `extra_args`: Additional arguments passed through to the underlying tools.
pub fn run_lint(fix: bool, types: bool, extra_args: Vec<String>) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);

    if ctx.languages.is_empty() {
        anyhow::bail!("No source files detected. Nothing to lint.");
    }

    let toolchain = dispatch::detect_toolchain(&ctx);
    let tools = toolchain.tools_for(Operation::Lint);

    if tools.is_empty() {
        eprintln!(
            "{} No linting tools found. Install {} or {}.",
            "warn:".yellow(),
            "clippy".cyan(),
            "ruff".cyan()
        );
        return Ok(());
    }

    let mut commands: Vec<PrefixedCommand> = tools
        .into_iter()
        .map(|tool| {
            let mut args = tool.default_args.clone();
            if fix {
                match tool.bin.as_str() {
                    "cargo" => {
                        // Replace default args with fix variant
                        args = vec![
                            "clippy".to_string(),
                            "--fix".to_string(),
                            "--allow-dirty".to_string(),
                            "--".to_string(),
                            "-D".to_string(),
                            "warnings".to_string(),
                        ];
                    }
                    "ruff" => args.push("--fix".to_string()),
                    _ => {}
                }
            }
            args.extend(extra_args.clone());

            PrefixedCommand {
                label: tool.label.clone(),
                bin: tool.bin.clone(),
                args,
                working_dir: Some(ctx.root.to_string_lossy().to_string()),
                env: Vec::new(),
            }
        })
        .collect();

    // Add type checking if requested
    if types && ctx.has_python() {
        if let Some(type_cmd) = detect_type_checker(&ctx) {
            commands.push(type_cmd);
        }
    }

    let results = run_with_prefix::run_prefixed(commands);
    run_with_prefix::print_summary(&results);

    if !run_with_prefix::all_succeeded(&results) {
        std::process::exit(run_with_prefix::worst_exit_code(&results));
    }

    Ok(())
}

/// Detect Python type checker: mypy > pyright > skip.
fn detect_type_checker(ctx: &ProjectContext) -> Option<PrefixedCommand> {
    let working_dir = ctx.root.to_string_lossy().to_string();

    if dispatch::tool_version("mypy").is_some() {
        Some(PrefixedCommand {
            label: "[types]".to_string(),
            bin: "mypy".to_string(),
            args: vec![".".to_string()],
            working_dir: Some(working_dir),
            env: Vec::new(),
        })
    } else if dispatch::tool_version("pyright").is_some() {
        Some(PrefixedCommand {
            label: "[types]".to_string(),
            bin: "pyright".to_string(),
            args: vec![".".to_string()],
            working_dir: Some(working_dir),
            env: Vec::new(),
        })
    } else {
        eprintln!(
            "{} No type checker found. Install {} or {}.",
            "warn:".yellow(),
            "mypy".cyan(),
            "pyright".cyan()
        );
        None
    }
}
