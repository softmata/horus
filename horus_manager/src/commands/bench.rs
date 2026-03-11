//! `horus bench` — run benchmarks for the project.
//!
//! Dispatches to `cargo bench` (Rust) and `pytest --benchmark-only` (Python).
//! Runs sequentially (benchmarks shouldn't compete for resources).

use anyhow::Result;
use colored::*;

use crate::dispatch::{self, Operation};

/// Run `horus bench`.
///
/// - `filter`: Optional benchmark name filter.
/// - `extra_args`: Additional arguments passed through to the underlying tools.
pub fn run_bench(filter: Option<String>, extra_args: Vec<String>) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);

    if ctx.languages.is_empty() {
        anyhow::bail!("No source files detected. Nothing to benchmark.");
    }

    let toolchain = dispatch::detect_toolchain(&ctx);
    let tools = toolchain.tools_for(Operation::Bench);

    if tools.is_empty() {
        eprintln!(
            "{} No benchmark tools found.",
            "warn:".yellow(),
        );
        return Ok(());
    }

    // Run benchmarks sequentially to avoid resource contention
    let mut any_failed = false;
    for tool in &tools {
        let mut args = tool.default_args.clone();
        if let Some(ref f) = filter {
            match tool.bin.as_str() {
                "cargo" => args.push(f.clone()),
                "pytest" => {
                    args.push("-k".to_string());
                    args.push(f.clone());
                }
                _ => args.push(f.clone()),
            }
        }
        args.extend(extra_args.clone());

        let result = dispatch::dispatch_tool(tool, &args[tool.default_args.len()..], &ctx.root)?;
        if !result.success() {
            any_failed = true;
        }
    }

    if any_failed {
        std::process::exit(1);
    }

    Ok(())
}
