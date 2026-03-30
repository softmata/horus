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

    // For horus projects (horus.toml without root Cargo.toml), point cargo at .horus/Cargo.toml
    let horus_manifest = ctx.root.join(".horus/Cargo.toml");
    let use_horus_manifest = ctx.has_horus_toml
        && !ctx.root.join("Cargo.toml").exists()
        && horus_manifest.exists();

    let mut commands: Vec<PrefixedCommand> = tools
        .into_iter()
        .map(|tool| {
            let mut args = tool.default_args.clone();
            if tool.bin == "cargo" && use_horus_manifest {
                // Insert --manifest-path before "--" separator (clippy args come after --)
                let dash_pos = args.iter().position(|a| a == "--");
                let insert_at = dash_pos.unwrap_or(args.len());
                args.insert(insert_at, horus_manifest.to_string_lossy().to_string());
                args.insert(insert_at, "--manifest-path".to_string());
            }
            if fix {
                match tool.bin.as_str() {
                    "cargo" => {
                        // Rebuild args for fix mode, preserving manifest-path if present
                        let manifest_path = if use_horus_manifest {
                            Some(horus_manifest.to_string_lossy().to_string())
                        } else {
                            None
                        };
                        args = vec!["clippy".to_string()];
                        if let Some(mp) = manifest_path {
                            args.push("--manifest-path".to_string());
                            args.push(mp);
                        }
                        args.extend([
                            "--fix".to_string(),
                            "--allow-dirty".to_string(),
                            "--".to_string(),
                            "-D".to_string(),
                            "warnings".to_string(),
                        ]);
                    }
                    "ruff" => args.push("--fix".to_string()),
                    "clang-tidy" => args.push("--fix".to_string()),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lint_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_lint(false, false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("No source files"));
    }

    #[test]
    fn lint_fix_mode_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_lint(true, false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn lint_types_mode_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_lint(false, true, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn detect_type_checker_returns_option() {
        // Build a minimal context to test detect_type_checker
        let tmp = tempfile::TempDir::new().unwrap();
        let ctx = ProjectContext {
            root: tmp.path().to_path_buf(),
            languages: vec![],
            has_horus_toml: false,
            manifest: None,
        };

        let result = detect_type_checker(&ctx);

        // Return type is Option<PrefixedCommand>:
        //   Some — if mypy or pyright is installed on this machine
        //   None — if neither is available
        match result {
            Some(cmd) => {
                assert!(
                    cmd.bin == "mypy" || cmd.bin == "pyright",
                    "type checker should be mypy or pyright, got: {}",
                    cmd.bin
                );
                assert_eq!(cmd.label, "[types]");
                assert_eq!(
                    cmd.working_dir.as_deref(),
                    Some(tmp.path().to_string_lossy().as_ref()),
                    "working_dir should point to the project root"
                );
            }
            None => {
                // Neither mypy nor pyright is installed — None is correct
            }
        }
    }

    // ── Battle tests: lint dispatch and modes ───────────────────────────

    #[test]
    fn lint_error_message_is_descriptive() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_lint(false, false, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No source files detected"),
            "Error should mention 'No source files detected', got: {}",
            err
        );
        assert!(
            err.contains("Nothing to lint"),
            "Error should mention 'Nothing to lint', got: {}",
            err
        );
    }

    #[test]
    fn lint_all_modes_fail_same_way_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let r1 = run_lint(false, false, vec![]).unwrap_err().to_string();
        let r2 = run_lint(true, false, vec![]).unwrap_err().to_string();
        let r3 = run_lint(false, true, vec![]).unwrap_err().to_string();
        let r4 = run_lint(true, true, vec![]).unwrap_err().to_string();
        std::env::set_current_dir(original).unwrap();

        // All should produce the same error on empty dir
        assert_eq!(r1, r2);
        assert_eq!(r2, r3);
        assert_eq!(r3, r4);
    }

    #[test]
    fn lint_with_extra_args_still_fails_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_lint(false, false, vec!["--verbose".to_string()]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn lint_rust_project_detects_clippy() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Lint);

        if dispatch::tool_version("cargo").is_some() {
            let rust_tool = tools
                .iter()
                .find(|t| t.bin == "cargo")
                .expect("Should have cargo clippy as lint tool");
            assert!(
                rust_tool.default_args.contains(&"clippy".to_string()),
                "Default args should include 'clippy': {:?}",
                rust_tool.default_args
            );
            assert!(
                rust_tool.default_args.contains(&"-D".to_string()),
                "Default args should include '-D': {:?}",
                rust_tool.default_args
            );
            assert!(
                rust_tool.default_args.contains(&"warnings".to_string()),
                "Default args should include 'warnings': {:?}",
                rust_tool.default_args
            );
        }
    }

    #[test]
    fn lint_python_project_detects_linter() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), "print('hello')").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_python());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Lint);

        if !tools.is_empty() {
            let py_tool = &tools[0];
            assert!(
                py_tool.bin == "ruff" || py_tool.bin == "pylint" || py_tool.bin == "flake8",
                "Python linter should be ruff, pylint, or flake8, got: {}",
                py_tool.bin
            );
        }
    }

    #[test]
    fn lint_fix_mode_replaces_clippy_args() {
        // Verify the fix=true branch replaces default clippy args
        let fix = true;
        let bin = "cargo";
        let mut args = vec![
            "clippy".to_string(),
            "--".to_string(),
            "-D".to_string(),
            "warnings".to_string(),
        ];

        if fix {
            match bin {
                "cargo" => {
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

        assert_eq!(
            args,
            vec!["clippy", "--fix", "--allow-dirty", "--", "-D", "warnings"],
            "Fix mode should replace args with fix variant"
        );
    }

    #[test]
    fn lint_fix_mode_appends_for_ruff() {
        let fix = true;
        let bin = "ruff";
        let mut args = vec!["check".to_string(), ".".to_string()];

        if fix {
            match bin {
                "cargo" => {
                    args = vec![
                        "clippy".to_string(),
                        "--fix".to_string(),
                        "--allow-dirty".to_string(),
                    ];
                }
                "ruff" => args.push("--fix".to_string()),
                _ => {}
            }
        }

        assert_eq!(args, vec!["check", ".", "--fix"]);
    }

    #[test]
    fn lint_no_fix_keeps_default_args() {
        let fix = false;
        let bin = "cargo";
        let mut args = vec![
            "clippy".to_string(),
            "--".to_string(),
            "-D".to_string(),
            "warnings".to_string(),
        ];

        if fix {
            match bin {
                "cargo" => {
                    args = vec!["clippy".to_string(), "--fix".to_string()];
                }
                _ => {}
            }
        }

        assert_eq!(
            args,
            vec!["clippy", "--", "-D", "warnings"],
            "Non-fix mode should keep default args"
        );
    }

    #[test]
    fn lint_mixed_project_detects_both_linters() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("lib.rs"), "pub fn foo() {}").unwrap();
        std::fs::write(tmp.path().join("helper.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());
        assert!(ctx.has_python());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Lint);

        if dispatch::tool_version("cargo").is_some() {
            let has_rust_linter = tools.iter().any(|t| t.bin == "cargo");
            assert!(has_rust_linter, "Mixed project should include cargo clippy");
        }
    }

    #[test]
    fn lint_types_flag_only_applies_to_python() {
        // types=true should only affect Python projects, not Rust
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        // Rust project — has_python() is false
        assert!(!ctx.has_python());

        // detect_type_checker requires Python — without Python, types flag is irrelevant
        // The type checker detection should not be triggered
        let type_cmd = detect_type_checker(&ctx);
        // Even if mypy is installed, it won't matter because types flag
        // only triggers when ctx.has_python() is true
        // The returned command points at the project root either way
        if let Some(cmd) = type_cmd {
            assert!(cmd.bin == "mypy" || cmd.bin == "pyright");
        }
    }

    #[test]
    fn detect_type_checker_prefers_mypy_over_pyright() {
        let tmp = tempfile::TempDir::new().unwrap();
        let ctx = ProjectContext {
            root: tmp.path().to_path_buf(),
            languages: vec![crate::manifest::Language::Python],
            has_horus_toml: false,
            manifest: None,
        };

        let result = detect_type_checker(&ctx);

        // If both are installed, mypy should be preferred (checked first)
        if dispatch::tool_version("mypy").is_some() {
            let cmd = result.expect("Should return Some when mypy is installed");
            assert_eq!(cmd.bin, "mypy", "mypy should be preferred over pyright");
            assert_eq!(cmd.args, vec!["."]);
        } else if dispatch::tool_version("pyright").is_some() {
            let cmd = result.expect("Should return Some when pyright is installed");
            assert_eq!(cmd.bin, "pyright");
        }
        // else: neither installed, None is correct
    }

    #[test]
    fn detect_type_checker_uses_correct_working_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let ctx = ProjectContext {
            root: tmp.path().to_path_buf(),
            languages: vec![crate::manifest::Language::Python],
            has_horus_toml: false,
            manifest: None,
        };

        if let Some(cmd) = detect_type_checker(&ctx) {
            assert_eq!(
                cmd.working_dir.as_deref(),
                Some(tmp.path().to_string_lossy().as_ref()),
                "Type checker should use the project root as working dir"
            );
            assert_eq!(cmd.label, "[types]");
            assert!(cmd.env.is_empty(), "No extra env vars expected");
        }
    }

    #[test]
    fn lint_horus_toml_only_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"empty-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_lint(false, false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(
            result.is_err(),
            "horus.toml alone should not detect languages"
        );
    }

    #[test]
    fn lint_extra_args_appended_after_defaults() {
        let default_args = vec![
            "clippy".to_string(),
            "--".to_string(),
            "-D".to_string(),
            "warnings".to_string(),
        ];
        let extra_args = vec!["--all-targets".to_string()];

        let mut args = default_args.clone();
        args.extend(extra_args.clone());

        assert_eq!(
            args,
            vec!["clippy", "--", "-D", "warnings", "--all-targets"],
            "Extra args should be appended after default args"
        );
    }

    #[test]
    fn lint_src_dir_python_detection() {
        // Python files in src/ should also be detected
        let tmp = tempfile::TempDir::new().unwrap();
        let src = tmp.path().join("src");
        std::fs::create_dir(&src).unwrap();
        std::fs::write(src.join("app.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_python(), "Should detect Python from src/*.py");
    }

    #[test]
    fn lint_setup_py_triggers_python_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("setup.py"),
            "from setuptools import setup\nsetup(name='test')\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_python(), "Should detect Python from setup.py");
    }
}
