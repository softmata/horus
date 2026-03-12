//! `horus shell` — activated subshell with horus environment.
//!
//! Spawns the user's shell with horus env vars (PATH, PYTHONPATH,
//! LD_LIBRARY_PATH) pre-set. Like `poetry shell` or `pipenv shell`.

use anyhow::Result;
use colored::*;

/// Detect the user's preferred shell from the `SHELL` environment variable.
/// Falls back to `/bin/bash` if the variable is unset.
pub(crate) fn detect_shell() -> String {
    std::env::var("SHELL").unwrap_or_else(|_| "/bin/bash".to_string())
}

/// Get the project name from `horus.toml`, falling back to `"horus"` if
/// no manifest is found or it cannot be loaded.
pub(crate) fn get_project_name() -> String {
    crate::manifest::HorusManifest::find_and_load()
        .map(|(m, _)| m.package.name)
        .unwrap_or_else(|_| "horus".to_string())
}

/// Build the shell [`std::process::Command`] with all horus environment
/// variables pre-set.  Returns the command ready to be spawned.
pub(crate) fn build_shell_command(
    shell: &str,
    project_name: &str,
    env_vars: &[(String, String)],
) -> std::process::Command {
    let mut cmd = std::process::Command::new(shell);

    // Set horus env vars
    for (key, value) in env_vars {
        cmd.env(key, value);
    }

    // Set a custom prompt indicator
    cmd.env("HORUS_SHELL", "1");
    cmd.env("HORUS_PROJECT", project_name);

    cmd
}

/// Run `horus shell`.
pub fn run_shell() -> Result<()> {
    let shell = detect_shell();

    // Build horus environment
    let env_vars = crate::commands::run::build_child_env()?;

    // Get project name from horus.toml if available
    let project_name = get_project_name();

    println!(
        "Entering horus shell for {}. Type {} to exit.",
        project_name.cyan(),
        "exit".bold()
    );

    let status = build_shell_command(&shell, &project_name, &env_vars).status()?;

    if !status.success() {
        std::process::exit(status.code().unwrap_or(1));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    // ── Helper: create a minimal horus.toml in a temp dir ───────────────

    fn write_horus_toml(dir: &std::path::Path, name: &str) {
        let content = format!(
            "[package]\nname = \"{}\"\nversion = \"0.1.0\"\n",
            name
        );
        fs::write(dir.join("horus.toml"), content).unwrap();
    }

    /// Set up the `.horus/` directory structure that `build_child_env` expects.
    fn setup_horus_dirs(dir: &std::path::Path) {
        fs::create_dir_all(dir.join(".horus/bin")).unwrap();
        fs::create_dir_all(dir.join(".horus/lib")).unwrap();
        fs::create_dir_all(dir.join(".horus/packages")).unwrap();
    }

    // ── detect_shell ────────────────────────────────────────────────────

    #[test]
    fn detect_shell_returns_env_shell_when_set() {
        // Save + restore SHELL to avoid polluting other tests.
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::var("SHELL").ok();

        std::env::set_var("SHELL", "/usr/bin/zsh");
        let result = detect_shell();

        // Restore
        match original {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        assert_eq!(result, "/usr/bin/zsh");
    }

    #[test]
    fn detect_shell_falls_back_to_bash_when_unset() {
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::var("SHELL").ok();

        std::env::remove_var("SHELL");
        let result = detect_shell();

        // Restore
        match original {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        assert_eq!(result, "/bin/bash");
    }

    #[test]
    fn detect_shell_preserves_unusual_paths() {
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::var("SHELL").ok();

        std::env::set_var("SHELL", "/usr/local/bin/fish");
        let result = detect_shell();

        match original {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        assert_eq!(result, "/usr/local/bin/fish");
    }

    // ── get_project_name ────────────────────────────────────────────────

    #[test]
    fn get_project_name_returns_manifest_name_when_present() {
        let tmp = tempfile::tempdir().unwrap();
        write_horus_toml(tmp.path(), "my-robot");

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        assert_eq!(name, "my-robot");
    }

    #[test]
    fn get_project_name_falls_back_to_horus_without_manifest() {
        let tmp = tempfile::tempdir().unwrap();
        // No horus.toml written

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        assert_eq!(name, "horus");
    }

    #[test]
    fn get_project_name_falls_back_on_invalid_toml() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("horus.toml"), "not valid toml {{{{").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        assert_eq!(name, "horus");
    }

    #[test]
    fn get_project_name_falls_back_on_missing_package_name() {
        let tmp = tempfile::tempdir().unwrap();
        // Valid TOML but missing required `name` field
        fs::write(tmp.path().join("horus.toml"), "[package]\nversion = \"0.1.0\"\n").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        assert_eq!(name, "horus");
    }

    #[test]
    fn get_project_name_reads_name_with_special_chars() {
        let tmp = tempfile::tempdir().unwrap();
        write_horus_toml(tmp.path(), "my_robot-v2");

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        assert_eq!(name, "my_robot-v2");
    }

    // ── build_shell_command ─────────────────────────────────────────────

    #[test]
    fn build_shell_command_uses_given_shell() {
        let cmd = build_shell_command("/usr/bin/zsh", "test-proj", &[]);
        assert_eq!(cmd.get_program(), "/usr/bin/zsh");
    }

    #[test]
    fn build_shell_command_sets_horus_shell_env() {
        let cmd = build_shell_command("/bin/bash", "proj", &[]);
        let envs: Vec<_> = cmd.get_envs().collect();
        let horus_shell = envs.iter().find(|(k, _)| *k == "HORUS_SHELL");
        assert!(horus_shell.is_some(), "HORUS_SHELL env var must be set");
        assert_eq!(
            horus_shell.unwrap().1,
            Some(std::ffi::OsStr::new("1"))
        );
    }

    #[test]
    fn build_shell_command_sets_horus_project_env() {
        let cmd = build_shell_command("/bin/bash", "nav-robot", &[]);
        let envs: Vec<_> = cmd.get_envs().collect();
        let horus_proj = envs.iter().find(|(k, _)| *k == "HORUS_PROJECT");
        assert!(horus_proj.is_some(), "HORUS_PROJECT env var must be set");
        assert_eq!(
            horus_proj.unwrap().1,
            Some(std::ffi::OsStr::new("nav-robot"))
        );
    }

    #[test]
    fn build_shell_command_forwards_child_env_vars() {
        let env_vars = vec![
            ("PATH".to_string(), "/horus/bin:/usr/bin".to_string()),
            ("LD_LIBRARY_PATH".to_string(), "/horus/lib".to_string()),
            ("PYTHONPATH".to_string(), "/horus/packages".to_string()),
        ];
        let cmd = build_shell_command("/bin/bash", "proj", &env_vars);
        let envs: Vec<_> = cmd.get_envs().collect();

        let path = envs.iter().find(|(k, _)| *k == "PATH");
        assert!(path.is_some(), "PATH must be forwarded");
        assert_eq!(
            path.unwrap().1,
            Some(std::ffi::OsStr::new("/horus/bin:/usr/bin"))
        );

        let ld = envs.iter().find(|(k, _)| *k == "LD_LIBRARY_PATH");
        assert!(ld.is_some(), "LD_LIBRARY_PATH must be forwarded");

        let py = envs.iter().find(|(k, _)| *k == "PYTHONPATH");
        assert!(py.is_some(), "PYTHONPATH must be forwarded");
    }

    #[test]
    fn build_shell_command_empty_env_vars() {
        let cmd = build_shell_command("/bin/bash", "proj", &[]);
        let envs: Vec<_> = cmd.get_envs().collect();
        // Should still have HORUS_SHELL and HORUS_PROJECT
        assert_eq!(envs.len(), 2);
    }

    #[test]
    fn build_shell_command_many_env_vars() {
        let env_vars: Vec<_> = (0..20)
            .map(|i| (format!("VAR_{}", i), format!("val_{}", i)))
            .collect();
        let cmd = build_shell_command("/bin/bash", "proj", &env_vars);
        let envs: Vec<_> = cmd.get_envs().collect();
        // 20 custom + HORUS_SHELL + HORUS_PROJECT = 22
        assert_eq!(envs.len(), 22);
    }

    #[test]
    fn build_shell_command_env_var_ordering_horus_overrides_last() {
        // HORUS_SHELL and HORUS_PROJECT should be set after user env vars
        // so that even if user env contains them, horus values win.
        let env_vars = vec![
            ("HORUS_SHELL".to_string(), "0".to_string()),
            ("HORUS_PROJECT".to_string(), "old-proj".to_string()),
        ];
        let cmd = build_shell_command("/bin/bash", "new-proj", &env_vars);
        let envs: Vec<_> = cmd.get_envs().collect();

        // The Command API stores envs as a map keyed by name.
        // The last `env()` call for a given key wins.
        let horus_shell = envs.iter().find(|(k, _)| *k == "HORUS_SHELL");
        assert_eq!(
            horus_shell.unwrap().1,
            Some(std::ffi::OsStr::new("1")),
            "HORUS_SHELL should be overridden to 1"
        );

        let horus_project = envs.iter().find(|(k, _)| *k == "HORUS_PROJECT");
        assert_eq!(
            horus_project.unwrap().1,
            Some(std::ffi::OsStr::new("new-proj")),
            "HORUS_PROJECT should be overridden to new-proj"
        );
    }

    #[test]
    fn build_shell_command_preserves_env_values_with_special_chars() {
        let env_vars = vec![
            ("PATH".to_string(), "/path with spaces:/other".to_string()),
            ("CUSTOM".to_string(), "value=with=equals".to_string()),
        ];
        let cmd = build_shell_command("/bin/bash", "proj", &env_vars);
        let envs: Vec<_> = cmd.get_envs().collect();

        let path = envs.iter().find(|(k, _)| *k == "PATH");
        assert_eq!(
            path.unwrap().1,
            Some(std::ffi::OsStr::new("/path with spaces:/other"))
        );

        let custom = envs.iter().find(|(k, _)| *k == "CUSTOM");
        assert_eq!(
            custom.unwrap().1,
            Some(std::ffi::OsStr::new("value=with=equals"))
        );
    }

    #[test]
    fn build_shell_command_empty_project_name() {
        let cmd = build_shell_command("/bin/bash", "", &[]);
        let envs: Vec<_> = cmd.get_envs().collect();
        let horus_project = envs.iter().find(|(k, _)| *k == "HORUS_PROJECT");
        assert_eq!(
            horus_project.unwrap().1,
            Some(std::ffi::OsStr::new(""))
        );
    }

    // ── Integration: run_shell with a fast-exiting shell ────────────────

    #[test]
    fn run_shell_builds_env_and_launches_true_command() {
        // Use `/bin/true` as the "shell" — it exits immediately with 0.
        // This exercises the full run_shell path (build_child_env + spawn)
        // without blocking on an interactive shell.
        let tmp = tempfile::tempdir().unwrap();
        setup_horus_dirs(tmp.path());
        write_horus_toml(tmp.path(), "shell-test");

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original_dir = std::env::current_dir().unwrap();
        let original_shell = std::env::var("SHELL").ok();

        std::env::set_current_dir(tmp.path()).unwrap();
        std::env::set_var("SHELL", "/bin/true");

        let result = run_shell();

        std::env::set_current_dir(original_dir).unwrap();
        match original_shell {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        assert!(result.is_ok(), "run_shell with /bin/true should succeed");
    }

    #[test]
    fn run_shell_without_manifest_uses_fallback_name() {
        // No horus.toml — project name should default to "horus"
        let tmp = tempfile::tempdir().unwrap();
        setup_horus_dirs(tmp.path());

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original_dir = std::env::current_dir().unwrap();
        let original_shell = std::env::var("SHELL").ok();

        std::env::set_current_dir(tmp.path()).unwrap();
        std::env::set_var("SHELL", "/bin/true");

        let result = run_shell();

        std::env::set_current_dir(original_dir).unwrap();
        match original_shell {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        assert!(result.is_ok());
    }

    #[test]
    fn run_shell_with_manifest_reads_project_name() {
        let tmp = tempfile::tempdir().unwrap();
        setup_horus_dirs(tmp.path());
        write_horus_toml(tmp.path(), "cool-robot");

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original_dir = std::env::current_dir().unwrap();
        let original_shell = std::env::var("SHELL").ok();

        std::env::set_current_dir(tmp.path()).unwrap();
        std::env::set_var("SHELL", "/bin/true");

        // We can't easily capture the println output in-process, but we can
        // verify the function succeeds and returns Ok.
        let result = run_shell();

        std::env::set_current_dir(original_dir).unwrap();
        match original_shell {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        assert!(result.is_ok());
    }

    // ── build_child_env interaction ─────────────────────────────────────

    #[test]
    fn build_child_env_available_from_shell_module() {
        // Ensure build_child_env is accessible from this module's path
        let tmp = tempfile::tempdir().unwrap();
        setup_horus_dirs(tmp.path());

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = crate::commands::run::build_child_env();

        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());
        let vars = result.unwrap();
        let keys: Vec<&str> = vars.iter().map(|(k, _)| k.as_str()).collect();
        assert!(keys.contains(&"PATH"));
        assert!(keys.contains(&"LD_LIBRARY_PATH"));
        assert!(keys.contains(&"PYTHONPATH"));
    }

    // ── Edge cases ──────────────────────────────────────────────────────

    #[test]
    fn detect_shell_empty_string() {
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::var("SHELL").ok();

        std::env::set_var("SHELL", "");
        let result = detect_shell();

        match original {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        // Empty string is a valid env value — it won't trigger the fallback
        assert_eq!(result, "");
    }

    #[test]
    fn build_shell_command_different_shells() {
        for shell in &["/bin/bash", "/bin/sh", "/usr/bin/zsh", "/usr/bin/fish"] {
            let cmd = build_shell_command(shell, "proj", &[]);
            assert_eq!(
                cmd.get_program().to_str().unwrap(),
                *shell,
                "Command should use the provided shell"
            );
        }
    }

    #[test]
    fn build_shell_command_env_keys_are_unique() {
        let env_vars = vec![
            ("FOO".to_string(), "bar".to_string()),
            ("BAZ".to_string(), "qux".to_string()),
        ];
        let cmd = build_shell_command("/bin/bash", "proj", &env_vars);
        let envs: Vec<_> = cmd.get_envs().collect();
        let keys: Vec<_> = envs.iter().map(|(k, _)| k.to_owned()).collect();
        // Check no duplicate keys — Command deduplicates by key
        let mut sorted = keys.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(keys.len(), sorted.len(), "No duplicate env var keys");
    }

    #[test]
    fn get_project_name_empty_name_in_manifest() {
        let tmp = tempfile::tempdir().unwrap();
        // Write a manifest with an empty name
        fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        // Manifest with empty name still loads — the name field is just empty
        // (validation is separate from loading).
        assert_eq!(name, "");
    }

    #[test]
    fn run_shell_nonexistent_shell_returns_error() {
        let tmp = tempfile::tempdir().unwrap();
        setup_horus_dirs(tmp.path());

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original_dir = std::env::current_dir().unwrap();
        let original_shell = std::env::var("SHELL").ok();

        std::env::set_current_dir(tmp.path()).unwrap();
        std::env::set_var("SHELL", "/nonexistent/shell/binary");

        let result = run_shell();

        std::env::set_current_dir(original_dir).unwrap();
        match original_shell {
            Some(v) => std::env::set_var("SHELL", v),
            None => std::env::remove_var("SHELL"),
        }

        // Spawning a nonexistent binary should return an IO error
        assert!(result.is_err(), "Nonexistent shell should cause an error");
    }

    // ── Manifest-based project name with various fields ─────────────────

    #[test]
    fn get_project_name_with_full_manifest() {
        let tmp = tempfile::tempdir().unwrap();
        // Top-level fields (enable) must come before table sections in TOML.
        let content = "\
[package]\n\
name = \"full-robot\"\n\
version = \"1.2.3\"\n\
description = \"A full robot package\"\n\
authors = [\"Alice <alice@example.com>\"]\n\
license = \"MIT\"\n\
\n\
enable = [\"cuda\"]\n\
\n\
[drivers]\n\
camera = \"opencv\"\n\
\n\
[scripts]\n\
sim = \"horus sim start\"\n";
        fs::write(tmp.path().join("horus.toml"), content).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        assert_eq!(name, "full-robot");
    }

    #[test]
    fn get_project_name_searches_parent_directories() {
        // horus.toml is found by searching upward from cwd
        let tmp = tempfile::tempdir().unwrap();
        write_horus_toml(tmp.path(), "parent-project");
        let subdir = tmp.path().join("src/nested");
        fs::create_dir_all(&subdir).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(&subdir).unwrap();

        let name = get_project_name();

        std::env::set_current_dir(original).unwrap();

        assert_eq!(name, "parent-project");
    }

    // ── build_shell_command with real build_child_env ────────────────────

    #[test]
    fn build_shell_command_with_real_env_sets_all_required() {
        let tmp = tempfile::tempdir().unwrap();
        setup_horus_dirs(tmp.path());

        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let env_vars = crate::commands::run::build_child_env().unwrap();

        std::env::set_current_dir(original).unwrap();

        let cmd = build_shell_command("/bin/bash", "test-proj", &env_vars);
        let envs: Vec<_> = cmd.get_envs().collect();
        let keys: Vec<_> = envs.iter().map(|(k, _)| k.to_str().unwrap()).collect();

        assert!(keys.contains(&"PATH"), "must have PATH");
        assert!(keys.contains(&"LD_LIBRARY_PATH"), "must have LD_LIBRARY_PATH");
        assert!(keys.contains(&"PYTHONPATH"), "must have PYTHONPATH");
        assert!(keys.contains(&"HORUS_SHELL"), "must have HORUS_SHELL");
        assert!(keys.contains(&"HORUS_PROJECT"), "must have HORUS_PROJECT");
    }
}
