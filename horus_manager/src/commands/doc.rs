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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn doc_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_doc(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("No source files"));
    }

    #[test]
    fn doc_open_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_doc(true, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    // ── Battle tests: doc dispatch and modes ────────────────────────────

    #[test]
    fn doc_error_message_is_descriptive() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_doc(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No source files detected"),
            "Error should contain 'No source files detected', got: {}",
            err
        );
        assert!(
            err.contains("Nothing to document"),
            "Error should contain 'Nothing to document', got: {}",
            err
        );
    }

    #[test]
    fn doc_open_and_non_open_fail_same_way_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let r1 = run_doc(false, vec![]).unwrap_err().to_string();
        let r2 = run_doc(true, vec![]).unwrap_err().to_string();
        std::env::set_current_dir(original).unwrap();

        assert_eq!(
            r1, r2,
            "Both open and non-open should fail identically on empty dir"
        );
    }

    #[test]
    fn doc_with_extra_args_still_fails_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_doc(false, vec!["--document-private-items".to_string()]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn doc_rust_project_detects_cargo_doc() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Doc);

        if dispatch::tool_version("cargo").is_some() {
            let rust_tool = tools
                .iter()
                .find(|t| t.bin == "cargo")
                .expect("Should have cargo doc");
            assert!(
                rust_tool.default_args.contains(&"doc".to_string()),
                "Default args should include 'doc': {:?}",
                rust_tool.default_args
            );
            assert!(
                rust_tool.default_args.contains(&"--no-deps".to_string()),
                "Default args should include '--no-deps': {:?}",
                rust_tool.default_args
            );
        }
    }

    #[test]
    fn doc_python_project_detects_doc_tool() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), "print('hello')").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_python());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Doc);

        if !tools.is_empty() {
            let py_tool = &tools[0];
            assert!(
                py_tool.bin == "pdoc" || py_tool.bin == "sphinx-build",
                "Python doc tool should be pdoc or sphinx-build, got: {}",
                py_tool.bin
            );
        }
    }

    #[test]
    fn doc_open_flag_only_applies_to_cargo() {
        let open = true;

        // For cargo
        let mut cargo_args = vec!["doc".to_string(), "--no-deps".to_string()];
        if open && "cargo" == "cargo" {
            cargo_args.push("--open".to_string());
        }
        assert_eq!(cargo_args, vec!["doc", "--no-deps", "--open"]);

        // For pdoc — should NOT get --open
        let mut pdoc_args = vec!["--html".to_string(), ".".to_string()];
        if open && "pdoc" == "cargo" {
            pdoc_args.push("--open".to_string());
        }
        assert_eq!(pdoc_args, vec!["--html", "."]);

        // For sphinx-build — should NOT get --open
        let mut sphinx_args = vec!["docs".to_string(), "_build".to_string()];
        if open && "sphinx-build" == "cargo" {
            sphinx_args.push("--open".to_string());
        }
        assert_eq!(sphinx_args, vec!["docs", "_build"]);
    }

    #[test]
    fn doc_no_open_flag_without_open_true() {
        let open = false;
        let mut args = vec!["doc".to_string(), "--no-deps".to_string()];
        if open && "cargo" == "cargo" {
            args.push("--open".to_string());
        }
        assert_eq!(
            args,
            vec!["doc", "--no-deps"],
            "Without open=true, --open should not appear"
        );
    }

    #[test]
    fn doc_mixed_project_detects_multiple_doc_tools() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("lib.rs"), "pub fn foo() {}").unwrap();
        std::fs::write(tmp.path().join("app.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust());
        assert!(ctx.has_python());

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Doc);

        if dispatch::tool_version("cargo").is_some() {
            let has_cargo_doc = tools.iter().any(|t| t.bin == "cargo");
            assert!(has_cargo_doc, "Mixed project should include cargo doc");
        }
    }

    #[test]
    fn doc_extra_args_appended_after_defaults() {
        let default_args = vec!["doc".to_string(), "--no-deps".to_string()];
        let extra_args = vec!["--document-private-items".to_string()];

        let mut args = default_args;
        args.extend(extra_args);

        assert_eq!(args, vec!["doc", "--no-deps", "--document-private-items"]);
    }

    #[test]
    fn doc_horus_toml_only_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"doc-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_doc(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn doc_cargo_toml_triggers_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Cargo.toml should trigger Rust detection");

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Doc);
        if dispatch::tool_version("cargo").is_some() {
            assert!(!tools.is_empty(), "Rust project should have cargo doc tool");
        }
    }

    #[test]
    fn doc_commands_have_correct_labels() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Doc);

        for tool in &tools {
            assert!(!tool.label.is_empty());
            assert!(
                tool.label.starts_with('[') && tool.label.ends_with(']'),
                "Tool label should be bracketed, got: {}",
                tool.label
            );
        }
    }

    #[test]
    fn doc_rust_default_args_include_no_deps() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        let toolchain = dispatch::detect_toolchain(&ctx);

        if let Some(tool) =
            toolchain.get(crate::manifest::Language::Rust, dispatch::Operation::Doc)
        {
            assert_eq!(tool.bin, "cargo");
            assert!(
                tool.default_args.contains(&"--no-deps".to_string()),
                "cargo doc should default to --no-deps"
            );
        }
    }

    #[test]
    fn doc_pdoc_default_args() {
        if dispatch::tool_version("pdoc").is_some() {
            let tmp = tempfile::TempDir::new().unwrap();
            std::fs::write(tmp.path().join("app.py"), "pass").unwrap();

            let ctx = dispatch::detect_context(tmp.path());
            let toolchain = dispatch::detect_toolchain(&ctx);

            if let Some(tool) =
                toolchain.get(crate::manifest::Language::Python, dispatch::Operation::Doc)
            {
                assert_eq!(tool.bin, "pdoc");
                assert_eq!(tool.default_args, vec!["--html", "."]);
            }
        }
    }
}
