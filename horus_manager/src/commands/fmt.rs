//! `horus fmt` — format all code in the project.
//!
//! Dispatches to `cargo fmt` (Rust) and `ruff format` / `black` (Python).
//! For mixed projects, runs both in parallel with prefixed output.

use anyhow::Result;
use colored::*;

use crate::dispatch::{self, Operation};
use crate::run_with_prefix::{self, PrefixedCommand};

/// Run `horus fmt`.
///
/// - `check`: If true, check formatting without modifying files (exit 1 if unformatted).
/// - `extra_args`: Additional arguments passed through to the underlying tools.
pub fn run_fmt(check: bool, extra_args: Vec<String>) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);

    if ctx.languages.is_empty() {
        anyhow::bail!("No source files detected. Nothing to format.");
    }

    let toolchain = dispatch::detect_toolchain(&ctx);
    let tools = toolchain.tools_for(Operation::Fmt);

    if tools.is_empty() {
        eprintln!(
            "{} No formatting tools found. Install {} or {}.",
            "warn:".yellow(),
            "rustfmt".cyan(),
            "ruff".cyan()
        );
        return Ok(());
    }

    let commands: Vec<PrefixedCommand> = tools
        .into_iter()
        .map(|tool| {
            let mut args = tool.default_args.clone();
            if check {
                match tool.bin.as_str() {
                    "cargo" => args.push("--check".to_string()),
                    "ruff" => args.push("--check".to_string()),
                    "black" => args.push("--check".to_string()),
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

    let results = run_with_prefix::run_prefixed(commands);
    run_with_prefix::print_summary(&results);

    if !run_with_prefix::all_succeeded(&results) {
        std::process::exit(run_with_prefix::worst_exit_code(&results));
    }

    Ok(())
}
