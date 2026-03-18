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
        eprintln!("{} No benchmark tools found.", "warn:".yellow(),);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bench_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_bench(None, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("No source files"));
    }

    #[test]
    fn bench_with_filter_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_bench(Some("my_bench".to_string()), vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    // ── Battle tests: bench dispatch and filter logic ────────────────────

    #[test]
    fn bench_error_message_is_descriptive() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_bench(None, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No source files detected"),
            "Error should contain 'No source files detected', got: {}",
            err
        );
        assert!(
            err.contains("Nothing to benchmark"),
            "Error should contain 'Nothing to benchmark', got: {}",
            err
        );
    }

    #[test]
    fn bench_with_extra_args_still_fails_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_bench(None, vec!["--nocapture".to_string()]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn bench_filter_and_extra_args_still_fails_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_bench(
            Some("bench_name".to_string()),
            vec!["--nocapture".to_string()],
        );
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn bench_rust_project_detects_cargo_bench() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Bench);

        if dispatch::tool_version("cargo").is_some() {
            let rust_tool = tools
                .iter()
                .find(|t| t.bin == "cargo")
                .expect("Should have cargo bench");
            assert!(
                rust_tool.default_args.contains(&"bench".to_string()),
                "Default args should include 'bench': {:?}",
                rust_tool.default_args
            );
        }
    }

    #[test]
    fn bench_python_project_detects_pytest_benchmark() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_python());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Bench);

        if dispatch::tool_version("pytest").is_some() {
            let py_tool = tools
                .iter()
                .find(|t| t.bin == "pytest")
                .expect("Should have pytest benchmark");
            assert!(
                py_tool
                    .default_args
                    .contains(&"--benchmark-only".to_string()),
                "Default args should include '--benchmark-only': {:?}",
                py_tool.default_args
            );
        }
    }

    #[test]
    fn bench_cargo_filter_appended_directly() {
        let filter = Some("my_bench".to_string());
        let bin = "cargo";
        let mut args = vec!["bench".to_string()];

        if let Some(ref f) = filter {
            match bin {
                "cargo" => args.push(f.clone()),
                "pytest" => {
                    args.push("-k".to_string());
                    args.push(f.clone());
                }
                _ => args.push(f.clone()),
            }
        }

        assert_eq!(args, vec!["bench", "my_bench"]);
    }

    #[test]
    fn bench_pytest_filter_uses_dash_k() {
        let filter = Some("test_perf".to_string());
        let bin = "pytest";
        let mut args = vec!["--benchmark-only".to_string()];

        if let Some(ref f) = filter {
            match bin {
                "cargo" => args.push(f.clone()),
                "pytest" => {
                    args.push("-k".to_string());
                    args.push(f.clone());
                }
                _ => args.push(f.clone()),
            }
        }

        assert_eq!(args, vec!["--benchmark-only", "-k", "test_perf"]);
    }

    #[test]
    fn bench_no_filter_no_extra_args() {
        let filter: Option<String> = None;
        let bin = "cargo";
        let mut args = vec!["bench".to_string()];

        if let Some(ref f) = filter {
            match bin {
                "cargo" => args.push(f.clone()),
                _ => {}
            }
        }

        assert_eq!(args, vec!["bench"]);
    }

    #[test]
    fn bench_extra_args_extend_after_filter() {
        let filter = Some("perf".to_string());
        let extra_args = vec!["--nocapture".to_string()];
        let mut args = vec!["bench".to_string()];

        if let Some(ref f) = filter {
            args.push(f.clone());
        }
        args.extend(extra_args);

        assert_eq!(args, vec!["bench", "perf", "--nocapture"]);
    }

    #[test]
    fn bench_mixed_project_detects_tools() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("lib.rs"), "pub fn foo() {}").unwrap();
        std::fs::write(tmp.path().join("test_bench.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());
        assert!(ctx.has_python());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Bench);

        if dispatch::tool_version("cargo").is_some() {
            assert!(
                tools.iter().any(|t| t.bin == "cargo"),
                "Mixed project should have cargo bench"
            );
        }
    }

    #[test]
    fn bench_horus_toml_only_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"bench-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_bench(None, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn bench_unknown_tool_filter_appended_directly() {
        let filter = Some("perf".to_string());
        let bin = "custom-bench";
        let mut args = vec!["run".to_string()];

        if let Some(ref f) = filter {
            match bin {
                "cargo" => args.push(f.clone()),
                "pytest" => {
                    args.push("-k".to_string());
                    args.push(f.clone());
                }
                _ => args.push(f.clone()),
            }
        }

        assert_eq!(args, vec!["run", "perf"]);
    }

    #[test]
    fn bench_runs_sequentially_tool_count() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Bench);

        // At most Rust + Python bench tools expected
        assert!(
            tools.len() <= 2,
            "At most 2 bench tools expected, got: {}",
            tools.len()
        );
    }

    #[test]
    fn bench_dispatch_tool_passes_only_extra_args() {
        // Verify that dispatch_tool receives only the extra args beyond default_args
        let default_args = vec!["bench".to_string()];
        let filter_arg = "my_filter".to_string();
        let extra_arg = "--nocapture".to_string();

        let mut all_args = default_args.clone();
        all_args.push(filter_arg);
        all_args.push(extra_arg);

        // The slice passed to dispatch_tool should be args[default_args.len()..]
        let dispatched = &all_args[default_args.len()..];
        assert_eq!(
            dispatched,
            &["my_filter", "--nocapture"],
            "Only non-default args should be dispatched"
        );
    }
}
