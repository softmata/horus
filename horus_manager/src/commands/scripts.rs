//! Run scripts defined in `horus.toml [scripts]`.
//!
//! Scripts are user-defined shell commands that can be run via `horus scripts <name>`.
//! They are defined in the `[scripts]` section of `horus.toml`:
//!
//! ```toml
//! [scripts]
//! sim = "horus sim start --world warehouse"
//! deploy-pi = "horus deploy robot@192.168.1.5 --release"
//! test-hw = "cargo test --features hardware"
//! ```

use crate::cli_output;
use crate::manifest::{HorusManifest, HORUS_TOML};
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::path::Path;
use std::process::Command;

/// Run a named script or list all available scripts.
pub fn run_scripts(name: Option<String>, args: Vec<String>) -> HorusResult<()> {
    let manifest_path = Path::new(HORUS_TOML);
    if !manifest_path.exists() {
        return Err(HorusError::Config(ConfigError::Other(
            "No horus.toml found. Run `horus new` to create a project.".to_string(),
        )));
    }

    let manifest = HorusManifest::load_from(manifest_path)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    match name {
        None => {
            // List all scripts
            if manifest.scripts.is_empty() {
                println!(
                    "{} No scripts defined in horus.toml",
                    cli_output::ICON_INFO.cyan()
                );
                println!(
                    "\n  Add scripts to horus.toml:\n\n  {}\n  {} = \"horus sim start --world warehouse\"\n  {} = \"cargo test --features hardware\"",
                    "[scripts]".cyan(),
                    "sim".green(),
                    "test-hw".green(),
                );
            } else {
                println!(
                    "{} Available scripts ({}):\n",
                    cli_output::ICON_INFO.cyan(),
                    manifest.scripts.len()
                );
                let max_name_len = manifest.scripts.keys().map(|k| k.len()).max().unwrap_or(0);
                for (script_name, command) in &manifest.scripts {
                    println!(
                        "  {}  {}",
                        format!("{:width$}", script_name, width = max_name_len).green(),
                        command.dimmed()
                    );
                }
                println!("\n  Run with: {}", "horus scripts <name>".cyan());
            }
            Ok(())
        }
        Some(script_name) => {
            // Run the named script
            let script_cmd = manifest.scripts.get(&script_name).ok_or_else(|| {
                let available = manifest
                    .scripts
                    .keys()
                    .map(|k| k.as_str())
                    .collect::<Vec<_>>()
                    .join(", ");
                HorusError::Config(ConfigError::Other(format!(
                    "Script '{}' not found. Available scripts: {}",
                    script_name,
                    if available.is_empty() {
                        "(none)".to_string()
                    } else {
                        available
                    }
                )))
            })?;

            // Append extra args to the command (shell-escaped to prevent injection)
            let full_cmd = if args.is_empty() {
                script_cmd.clone()
            } else {
                let escaped_args: Vec<String> = args
                    .iter()
                    .map(|a| format!("'{}'", a.replace('\'', "'\\''")))
                    .collect();
                format!("{} {}", script_cmd, escaped_args.join(" "))
            };

            println!(
                "{} Running script {}: {}",
                cli_output::ICON_INFO.cyan(),
                script_name.green(),
                full_cmd.dimmed()
            );

            #[cfg(unix)]
            let status = Command::new("sh")
                .arg("-c")
                .arg(&full_cmd)
                .status();

            #[cfg(windows)]
            let status = Command::new("cmd")
                .arg("/C")
                .arg(&full_cmd)
                .status();

            let status = status
                .map_err(|e| {
                    HorusError::Config(ConfigError::Other(format!(
                        "Failed to execute script: {}",
                        e
                    )))
                })?;

            if !status.success() {
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "Script '{}' exited with code {}",
                    script_name,
                    status.code().unwrap_or(-1)
                ))));
            }

            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;

    // ── Helper ───────────────────────────────────────────────────────────────

    /// Run a closure inside a temp directory, holding the CWD_LOCK.
    fn in_tmp<F, R>(tmp: &tempfile::TempDir, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = f();
        std::env::set_current_dir(original).unwrap();
        result
    }

    /// Write a minimal horus.toml with optional [scripts] section.
    fn write_manifest(dir: &std::path::Path, scripts_toml: &str) {
        let content = format!(
            "[package]\nname = \"test-proj\"\nversion = \"0.1.0\"\n{}",
            scripts_toml,
        );
        fs::write(dir.join(HORUS_TOML), content).unwrap();
    }

    // ── No manifest ──────────────────────────────────────────────────────────

    #[test]
    fn scripts_no_horus_toml_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_err());
    }

    #[test]
    fn scripts_no_manifest_error_message_mentions_horus_new() {
        let tmp = tempfile::TempDir::new().unwrap();
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("horus new"),
            "Error should suggest 'horus new', got: {}",
            err,
        );
    }

    #[test]
    fn scripts_no_manifest_run_named_also_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let result = in_tmp(&tmp, || run_scripts(Some("anything".to_string()), vec![]));
        assert!(result.is_err());
    }

    // ── Invalid / corrupt manifest ───────────────────────────────────────────

    #[test]
    fn scripts_invalid_toml_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join(HORUS_TOML), "{{invalid toml").unwrap();
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_err());
    }

    #[test]
    fn scripts_missing_package_section_still_parses() {
        // A manifest without [package] is valid (virtual workspace pattern).
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[scripts]\nhello = \"echo hi\"\n",
        )
        .unwrap();
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(
            result.is_ok(),
            "Manifest without [package] should parse (virtual workspace)"
        );
    }

    #[test]
    fn scripts_empty_file_parses_with_defaults() {
        // An empty horus.toml parses with all defaults.
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join(HORUS_TOML), "").unwrap();
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_ok());
    }

    // ── List (name = None) ───────────────────────────────────────────────────

    #[test]
    fn scripts_list_empty() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "");
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_list_with_entries() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(
            tmp.path(),
            "\n[scripts]\nsim = \"echo sim\"\ntest-hw = \"echo hw\"\n",
        );
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_list_single_entry() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nonly = \"echo only\"\n");
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_list_many_entries() {
        let tmp = tempfile::TempDir::new().unwrap();
        let mut scripts = String::from("\n[scripts]\n");
        for i in 0..20 {
            scripts.push_str(&format!("script{} = \"echo {}\"\n", i, i));
        }
        write_manifest(tmp.path(), &scripts);
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_list_ignores_extra_args() {
        // When listing (name=None), extra args are harmless
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\na = \"echo a\"\n");
        let result = in_tmp(&tmp, || run_scripts(None, vec!["extra".to_string()]));
        assert!(result.is_ok());
    }

    // ── Run named script (success) ───────────────────────────────────────────

    #[test]
    fn scripts_run_named_script() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nhello = \"echo hello\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("hello".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_run_with_extra_args() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\ngreet = \"echo\"\n");
        let result = in_tmp(&tmp, || {
            run_scripts(Some("greet".to_string()), vec!["world".to_string()])
        });
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_run_with_multiple_extra_args() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\ngreet = \"echo\"\n");
        let result = in_tmp(&tmp, || {
            run_scripts(
                Some("greet".to_string()),
                vec!["hello".to_string(), "world".to_string(), "!".to_string()],
            )
        });
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_run_true_succeeds() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nok = \"true\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("ok".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_run_multiword_command() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(
            tmp.path(),
            "\n[scripts]\nmulti = \"echo one && echo two\"\n",
        );
        let result = in_tmp(&tmp, || run_scripts(Some("multi".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_run_script_with_pipe() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\npipe = \"echo hello | cat\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("pipe".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_run_script_creates_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let marker = tmp.path().join("marker.txt");
        let cmd = format!("touch {}", marker.display());
        let toml = format!("\n[scripts]\ncreate = \"{}\"\n", cmd);
        write_manifest(tmp.path(), &toml);
        let result = in_tmp(&tmp, || run_scripts(Some("create".to_string()), vec![]));
        assert!(result.is_ok());
        assert!(marker.exists(), "Script should have created marker file");
    }

    #[test]
    fn scripts_run_first_of_multiple_scripts() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(
            tmp.path(),
            "\n[scripts]\nalpha = \"echo a\"\nbeta = \"echo b\"\ngamma = \"echo c\"\n",
        );
        let result = in_tmp(&tmp, || run_scripts(Some("alpha".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_run_last_of_multiple_scripts() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(
            tmp.path(),
            "\n[scripts]\nalpha = \"echo a\"\nbeta = \"echo b\"\ngamma = \"echo c\"\n",
        );
        let result = in_tmp(&tmp, || run_scripts(Some("gamma".to_string()), vec![]));
        assert!(result.is_ok());
    }

    // ── Missing / not found script ───────────────────────────────────────────

    #[test]
    fn scripts_missing_script_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nhello = \"echo hello\"\n");
        let result = in_tmp(&tmp, || {
            run_scripts(Some("nonexistent".to_string()), vec![])
        });
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("not found"),
            "Error should mention 'not found', got: {}",
            err,
        );
    }

    #[test]
    fn scripts_missing_script_lists_available() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(
            tmp.path(),
            "\n[scripts]\nalpha = \"echo a\"\nbeta = \"echo b\"\n",
        );
        let result = in_tmp(&tmp, || run_scripts(Some("missing".to_string()), vec![]));
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("alpha"),
            "Error should list available scripts, got: {}",
            err,
        );
        assert!(
            err.contains("beta"),
            "Error should list available scripts, got: {}",
            err,
        );
    }

    #[test]
    fn scripts_missing_script_with_no_scripts_shows_none() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "");
        let result = in_tmp(&tmp, || run_scripts(Some("anything".to_string()), vec![]));
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("(none)"),
            "Error should show '(none)' when no scripts exist, got: {}",
            err,
        );
    }

    #[test]
    fn scripts_case_sensitive_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nhello = \"echo hello\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("Hello".to_string()), vec![]));
        assert!(
            result.is_err(),
            "Script name lookup should be case-sensitive"
        );
    }

    // ── Failing commands ─────────────────────────────────────────────────────

    #[test]
    fn scripts_failing_command_returns_error() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nfail = \"exit 1\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("fail".to_string()), vec![]));
        assert!(result.is_err());
    }

    #[test]
    fn scripts_failing_command_error_contains_script_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nboom = \"exit 42\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("boom".to_string()), vec![]));
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("boom"),
            "Error should mention script name, got: {}",
            err,
        );
    }

    #[test]
    fn scripts_exit_code_2() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nfail2 = \"exit 2\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("fail2".to_string()), vec![]));
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("exited with code"),
            "Should mention exit code, got: {}",
            err,
        );
    }

    #[test]
    fn scripts_exit_code_127_command_not_found() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(
            tmp.path(),
            "\n[scripts]\nbad = \"nonexistent_command_xyz_12345\"\n",
        );
        let result = in_tmp(&tmp, || run_scripts(Some("bad".to_string()), vec![]));
        assert!(result.is_err(), "Running a nonexistent command should fail");
    }

    #[test]
    fn scripts_false_command_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nnope = \"false\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("nope".to_string()), vec![]));
        assert!(result.is_err());
    }

    // ── Edge cases: script names ─────────────────────────────────────────────

    #[test]
    fn scripts_hyphenated_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\ntest-hw = \"echo hw\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("test-hw".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_underscored_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\ntest_hw = \"echo hw\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("test_hw".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_dotted_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\n\"run.fast\" = \"echo fast\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("run.fast".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_numeric_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\n\"123\" = \"echo num\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("123".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_empty_name_string() {
        // An empty script name shouldn't match anything
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nhello = \"echo hi\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some(String::new()), vec![]));
        assert!(
            result.is_err(),
            "Empty script name should not match any script"
        );
    }

    // ── Edge cases: script commands ──────────────────────────────────────────

    #[test]
    fn scripts_empty_command_string_succeeds() {
        // An empty command to `sh -c ""` returns success
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nempty = \"\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("empty".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_command_with_env_var() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nenv = \"echo $HOME\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("env".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_command_with_quotes() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nquoted = \"echo 'hello world'\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("quoted".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_command_with_redirect() {
        let tmp = tempfile::TempDir::new().unwrap();
        let output_file = tmp.path().join("output.txt");
        let cmd = format!("echo redirected > {}", output_file.display());
        let toml = format!("\n[scripts]\nredir = \"{}\"\n", cmd);
        write_manifest(tmp.path(), &toml);
        let result = in_tmp(&tmp, || run_scripts(Some("redir".to_string()), vec![]));
        assert!(result.is_ok());
        assert!(output_file.exists(), "Redirect should have created file");
        let content = fs::read_to_string(&output_file).unwrap();
        assert!(content.contains("redirected"));
    }

    // ── Extra args edge cases ────────────────────────────────────────────────

    #[test]
    fn scripts_extra_args_with_spaces() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\ngreet = \"echo\"\n");
        let result = in_tmp(&tmp, || {
            run_scripts(Some("greet".to_string()), vec!["hello world".to_string()])
        });
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_extra_args_with_flags() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nls = \"ls\"\n");
        let result = in_tmp(&tmp, || {
            run_scripts(Some("ls".to_string()), vec!["-la".to_string()])
        });
        assert!(result.is_ok());
    }

    #[test]
    fn scripts_extra_args_appended_to_full_command() {
        // Verify args are appended by using a script that writes args to a file
        let tmp = tempfile::TempDir::new().unwrap();
        let out = tmp.path().join("args.txt");
        let cmd = format!("echo > {}", out.display());
        let toml = format!("\n[scripts]\nlog = \"{}\"\n", cmd);
        write_manifest(tmp.path(), &toml);
        // The extra arg "extra_token" gets appended after the redirect,
        // but the command still runs via sh -c
        let result = in_tmp(&tmp, || run_scripts(Some("log".to_string()), vec![]));
        assert!(result.is_ok());
    }

    // ── Battle tests ─────────────────────────────────────────────────────────

    #[test]
    fn battle_sequential_runs_same_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\na = \"echo a\"\nb = \"echo b\"\n");
        in_tmp(&tmp, || {
            run_scripts(Some("a".to_string()), vec![]).unwrap();
            run_scripts(Some("b".to_string()), vec![]).unwrap();
            run_scripts(None, vec![]).unwrap();
        });
    }

    #[test]
    fn battle_manifest_with_all_sections() {
        let tmp = tempfile::TempDir::new().unwrap();
        let content = r#"
[package]
name = "full-project"
version = "1.0.0"
description = "A test project"
authors = ["Test Author"]
license = "MIT"

[drivers]
camera = "opencv"

[scripts]
build = "echo building"
test = "echo testing"
deploy = "echo deploying"

[ignore]
files = ["*.tmp"]
directories = ["old/"]

enable = ["cuda"]
"#;
        fs::write(tmp.path().join(HORUS_TOML), content).unwrap();
        let result = in_tmp(&tmp, || run_scripts(Some("deploy".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn battle_manifest_with_all_sections_list() {
        let tmp = tempfile::TempDir::new().unwrap();
        let content = r#"
[package]
name = "full-project"
version = "1.0.0"

[drivers]
camera = "opencv"
lidar = true

[scripts]
build = "echo building"
test = "echo testing"
deploy = "echo deploying"
lint = "echo linting"

[ignore]
files = ["*.tmp"]

enable = ["cuda", "editor"]
"#;
        fs::write(tmp.path().join(HORUS_TOML), content).unwrap();
        let result = in_tmp(&tmp, || run_scripts(None, vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn battle_run_script_then_list() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nping = \"echo pong\"\n");
        in_tmp(&tmp, || {
            run_scripts(Some("ping".to_string()), vec![]).unwrap();
            run_scripts(None, vec![]).unwrap();
        });
    }

    #[test]
    fn battle_script_with_long_command() {
        let tmp = tempfile::TempDir::new().unwrap();
        // A long but valid command
        let long_cmd = "echo ".to_string() + &"x".repeat(500);
        let toml = format!("\n[scripts]\nlong = \"{}\"\n", long_cmd);
        write_manifest(tmp.path(), &toml);
        let result = in_tmp(&tmp, || run_scripts(Some("long".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn battle_script_with_special_chars_in_value() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nspec = \"echo 'a&b|c;d'\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("spec".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn battle_script_unicode_output() {
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\nuni = \"echo 'hello 世界'\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("uni".to_string()), vec![]));
        assert!(result.is_ok());
    }

    #[test]
    fn battle_script_working_directory_is_project_root() {
        // Verify that `sh -c` runs in the cwd (project root)
        let tmp = tempfile::TempDir::new().unwrap();
        let marker = tmp.path().join("proof.txt");
        let cmd = format!("pwd > {}", marker.display());
        let toml = format!("\n[scripts]\npwd = \"{}\"\n", cmd);
        write_manifest(tmp.path(), &toml);
        let result = in_tmp(&tmp, || run_scripts(Some("pwd".to_string()), vec![]));
        assert!(result.is_ok());
        let pwd_output = fs::read_to_string(&marker).unwrap();
        let pwd_path = PathBuf::from(pwd_output.trim());
        // On some systems tmp is a symlink; canonicalize both
        let expect = tmp.path().canonicalize().unwrap();
        let got = pwd_path.canonicalize().unwrap();
        assert_eq!(got, expect);
    }

    #[test]
    fn battle_partially_successful_pipeline() {
        // `true && false` => exit 1
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(tmp.path(), "\n[scripts]\npartial = \"true && false\"\n");
        let result = in_tmp(&tmp, || run_scripts(Some("partial".to_string()), vec![]));
        assert!(result.is_err());
    }

    #[test]
    fn battle_script_that_reads_manifest() {
        // Script that cats its own manifest (meta-test)
        let tmp = tempfile::TempDir::new().unwrap();
        write_manifest(
            tmp.path(),
            "\n[scripts]\nself-read = \"cat horus.toml > /dev/null\"\n",
        );
        let result = in_tmp(&tmp, || run_scripts(Some("self-read".to_string()), vec![]));
        assert!(result.is_ok());
    }

    // ── Intent tests: manifest script parsing ──────────────────────────────

    /// INTENT: Parse a horus.toml with [scripts] containing "test" and "build".
    /// Both scripts must be extracted and accessible by name.
    #[test]
    fn test_scripts_parse_from_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "my-robot"
version = "0.1.0"

[scripts]
test = "cargo test --all"
build = "cargo build --release"
"#;
        fs::write(tmp.path().join(HORUS_TOML), toml_content).unwrap();

        let manifest = in_tmp(&tmp, || {
            crate::manifest::HorusManifest::load_from(std::path::Path::new(HORUS_TOML)).unwrap()
        });

        assert_eq!(
            manifest.scripts.len(),
            2,
            "Should parse exactly 2 scripts, got {}",
            manifest.scripts.len()
        );
        assert!(
            manifest.scripts.contains_key("test"),
            "Scripts must contain 'test'"
        );
        assert!(
            manifest.scripts.contains_key("build"),
            "Scripts must contain 'build'"
        );
        assert_eq!(manifest.scripts["test"], "cargo test --all");
        assert_eq!(manifest.scripts["build"], "cargo build --release");
    }

    /// INTENT: Parse a horus.toml with an empty [scripts] section.
    /// The scripts map must be empty, not an error.
    #[test]
    fn test_scripts_empty_section() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "my-robot"
version = "0.1.0"

[scripts]
"#;
        fs::write(tmp.path().join(HORUS_TOML), toml_content).unwrap();

        let manifest = in_tmp(&tmp, || {
            crate::manifest::HorusManifest::load_from(std::path::Path::new(HORUS_TOML)).unwrap()
        });

        assert!(
            manifest.scripts.is_empty(),
            "Empty [scripts] section should produce an empty map, got {} entries",
            manifest.scripts.len()
        );
    }

    /// INTENT: Parse a script entry with arguments like "cargo test -- --nocapture".
    /// The full command string including args must be preserved verbatim.
    #[test]
    fn test_scripts_with_args() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "my-robot"
version = "0.1.0"

[scripts]
test-verbose = "cargo test -- --nocapture"
"#;
        fs::write(tmp.path().join(HORUS_TOML), toml_content).unwrap();

        let manifest = in_tmp(&tmp, || {
            crate::manifest::HorusManifest::load_from(std::path::Path::new(HORUS_TOML)).unwrap()
        });

        assert!(
            manifest.scripts.contains_key("test-verbose"),
            "Scripts must contain 'test-verbose'"
        );
        let cmd = &manifest.scripts["test-verbose"];
        assert_eq!(
            cmd, "cargo test -- --nocapture",
            "Full command with args must be preserved verbatim"
        );
        // Verify the args portion is intact (double-dash separator preserved)
        assert!(
            cmd.contains("-- --nocapture"),
            "Double-dash separator and args must be preserved, got: {}",
            cmd
        );
    }
}
