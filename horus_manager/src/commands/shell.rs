//! `horus shell` — activated subshell with horus environment.
//!
//! Spawns the user's shell with horus env vars (PATH, PYTHONPATH,
//! LD_LIBRARY_PATH) pre-set. Like `poetry shell` or `pipenv shell`.

use anyhow::Result;
use colored::*;

/// Run `horus shell`.
pub fn run_shell() -> Result<()> {
    let shell = std::env::var("SHELL").unwrap_or_else(|_| "/bin/bash".to_string());

    // Build horus environment
    let env_vars = crate::commands::run::build_child_env()?;

    // Get project name from horus.toml if available
    let project_name = crate::manifest::HorusManifest::find_and_load()
        .map(|(m, _, _)| m.package.name)
        .unwrap_or_else(|_| "horus".to_string());

    println!(
        "Entering horus shell for {}. Type {} to exit.",
        project_name.cyan(),
        "exit".bold()
    );

    let mut cmd = std::process::Command::new(&shell);

    // Set horus env vars
    for (key, value) in &env_vars {
        cmd.env(key, value);
    }

    // Set a custom prompt indicator
    cmd.env("HORUS_SHELL", "1");
    cmd.env("HORUS_PROJECT", &project_name);

    let status = cmd.status()?;

    if !status.success() {
        std::process::exit(status.code().unwrap_or(1));
    }

    Ok(())
}
