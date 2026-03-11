//! `horus doc` — generate documentation for the project.
//!
//! Dispatches to `cargo doc` (Rust) and `pdoc` / `sphinx-build` (Python).
//! Supports `--open` to launch browser after generation.

use anyhow::Result;
use colored::*;

use crate::dispatch::{self, Operation};
use crate::run_with_prefix::{self, PrefixedCommand};

/// Run `horus doc`.
///
/// - `open`: If true, open generated docs in browser.
/// - `extra_args`: Additional arguments passed through to the underlying tools.
pub fn run_doc(open: bool, extra_args: Vec<String>) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);

    if ctx.languages.is_empty() {
        anyhow::bail!("No source files detected. Nothing to document.");
    }

    let toolchain = dispatch::detect_toolchain(&ctx);
    let tools = toolchain.tools_for(Operation::Doc);

    if tools.is_empty() {
        eprintln!(
            "{} No documentation tools found. Install {} or {}.",
            "warn:".yellow(),
            "rustdoc".cyan(),
            "pdoc".cyan()
        );
        return Ok(());
    }

    let commands: Vec<PrefixedCommand> = tools
        .into_iter()
        .map(|tool| {
            let mut args = tool.default_args.clone();
            if open && tool.bin == "cargo" {
                args.push("--open".to_string());
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
