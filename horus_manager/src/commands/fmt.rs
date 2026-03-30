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

    // For horus projects (horus.toml without root Cargo.toml), point cargo at .horus/Cargo.toml
    let horus_manifest = ctx.root.join(".horus/Cargo.toml");
    let use_horus_manifest =
        ctx.has_horus_toml && !ctx.root.join("Cargo.toml").exists() && horus_manifest.exists();

    let commands: Vec<PrefixedCommand> = tools
        .into_iter()
        .map(|tool| {
            let mut args = tool.default_args.clone();
            if tool.bin == "cargo" && use_horus_manifest {
                args.push("--manifest-path".to_string());
                args.push(horus_manifest.to_string_lossy().to_string());
            }
            if check {
                match tool.bin.as_str() {
                    "cargo" => args.push("--check".to_string()),
                    "ruff" => args.push("--check".to_string()),
                    "black" => args.push("--check".to_string()),
                    "clang-format" => {
                        // Replace -i (in-place) with --dry-run --Werror (check mode)
                        args.retain(|a| a != "-i");
                        args.push("--dry-run".to_string());
                        args.push("--Werror".to_string());
                    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fmt_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
        assert!(
            result.unwrap_err().to_string().contains("No source files"),
            "Should mention no source files"
        );
    }

    #[test]
    fn fmt_check_mode_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(true, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    // ── Battle tests: dispatch and context ──────────────────────────────

    #[test]
    fn fmt_error_message_is_descriptive() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No source files detected"),
            "Error should mention 'No source files detected', got: {}",
            err
        );
        assert!(
            err.contains("Nothing to format"),
            "Error should mention 'Nothing to format', got: {}",
            err
        );
    }

    #[test]
    fn fmt_check_error_message_matches_non_check() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let r1 = run_fmt(false, vec![]);
        let r2 = run_fmt(true, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert_eq!(
            r1.unwrap_err().to_string(),
            r2.unwrap_err().to_string(),
            "Both check and non-check should produce the same error for empty dir"
        );
    }

    #[test]
    fn fmt_with_extra_args_still_fails_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec!["--verbose".to_string()]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn fmt_empty_extra_args_is_valid() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, Vec::new());
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn fmt_rust_project_detects_cargo_fmt() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(
            ctx.languages.contains(&crate::manifest::Language::Rust),
            "Should detect Rust from .rs file"
        );

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        if dispatch::tool_version("cargo").is_some() {
            assert!(
                !tools.is_empty(),
                "Rust project should have at least cargo fmt"
            );
            let rust_tool = tools
                .iter()
                .find(|t| t.bin == "cargo")
                .expect("Should have cargo as fmt tool");
            assert_eq!(rust_tool.default_args, vec!["fmt"]);
            assert_eq!(rust_tool.label, "[rust]");
        }
    }

    #[test]
    fn fmt_python_project_detects_formatter() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), "print('hello')").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(
            ctx.languages.contains(&crate::manifest::Language::Python),
            "Should detect Python from .py file"
        );

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        if !tools.is_empty() {
            let py_tool = &tools[0];
            assert!(
                py_tool.bin == "ruff" || py_tool.bin == "black",
                "Python formatter should be ruff or black, got: {}",
                py_tool.bin
            );
        }
    }

    #[test]
    fn fmt_mixed_project_detects_both_formatters() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("lib.rs"), "pub fn foo() {}").unwrap();
        std::fs::write(tmp.path().join("helper.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Should detect Rust");
        assert!(ctx.has_python(), "Should detect Python");

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        if dispatch::tool_version("cargo").is_some() {
            let has_rust_tool = tools.iter().any(|t| t.bin == "cargo");
            assert!(has_rust_tool, "Mixed project should include cargo fmt tool");
        }
    }

    #[test]
    fn fmt_check_flag_adds_check_arg_for_cargo() {
        let check = true;
        let bin = "cargo";
        let mut args = vec!["fmt".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec!["fmt", "--check"]);
    }

    #[test]
    fn fmt_check_flag_adds_check_arg_for_ruff() {
        let check = true;
        let bin = "ruff";
        let mut args = vec!["format".to_string(), ".".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec!["format", ".", "--check"]);
    }

    #[test]
    fn fmt_check_flag_adds_check_arg_for_black() {
        let check = true;
        let bin = "black";
        let mut args = vec![".".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec![".", "--check"]);
    }

    #[test]
    fn fmt_no_check_flag_for_unknown_tool() {
        let check = true;
        let bin = "clang-format";
        let mut args = vec!["-i".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec!["-i"]);
    }

    #[test]
    fn fmt_extra_args_appended_after_defaults() {
        let default_args = vec!["fmt".to_string()];
        let extra_args = vec!["--verbose".to_string(), "--color=always".to_string()];

        let mut args = default_args;
        args.extend(extra_args.clone());
        assert_eq!(args, vec!["fmt", "--verbose", "--color=always"]);
    }

    #[test]
    fn fmt_horus_toml_only_project_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"empty-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn fmt_project_root_from_subdirectory() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"sub-test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();
        let sub = tmp.path().join("subdir");
        std::fs::create_dir(&sub).unwrap();

        let ctx = dispatch::detect_context(&sub);
        assert!(
            ctx.has_horus_toml,
            "Should find horus.toml from subdirectory"
        );
        assert!(ctx.has_rust(), "Should detect Rust from project root");
    }

    #[test]
    fn fmt_src_dir_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        let src = tmp.path().join("src");
        std::fs::create_dir(&src).unwrap();
        std::fs::write(src.join("lib.rs"), "pub fn foo() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Should detect Rust from src/*.rs");
    }

    #[test]
    fn fmt_pyproject_triggers_python_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("pyproject.toml"),
            "[project]\nname = \"mybot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_python(), "Should detect Python from pyproject.toml");
    }

    #[test]
    fn fmt_requirements_txt_triggers_python_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("requirements.txt"), "numpy>=1.24\n").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(
            ctx.has_python(),
            "Should detect Python from requirements.txt"
        );
    }

    #[test]
    fn fmt_cargo_toml_triggers_rust_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Should detect Rust from Cargo.toml");
    }

    #[test]
    fn fmt_commands_have_correct_working_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        let expected_dir = ctx.root.to_string_lossy().to_string();

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        for tool in &tools {
            let cmd = run_with_prefix::PrefixedCommand {
                label: tool.label.clone(),
                bin: tool.bin.clone(),
                args: tool.default_args.clone(),
                working_dir: Some(ctx.root.to_string_lossy().to_string()),
                env: Vec::new(),
            };
            assert_eq!(
                cmd.working_dir.as_deref(),
                Some(expected_dir.as_str()),
                "Working dir should be the project root"
            );
        }
    }
}
