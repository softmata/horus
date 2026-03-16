//! Hook execution engine for [hooks] section in horus.toml.

use crate::manifest::HorusManifest;
use anyhow::Result;

/// Execute hooks for a given phase.
///
/// Maps hook names to existing command functions. Unknown names are tried
/// as [scripts] entries.
pub fn run_hooks(phase: &str, manifest: &HorusManifest) -> Result<()> {
    let hooks = match phase {
        "pre_run" => &manifest.hooks.pre_run,
        "pre_build" => &manifest.hooks.pre_build,
        "pre_test" => &manifest.hooks.pre_test,
        "post_test" => &manifest.hooks.post_test,
        _ => return Ok(()),
    };
    if hooks.is_empty() {
        return Ok(());
    }
    for hook in hooks {
        log::info!("[hooks] Running {}: {}", phase, hook);
        match hook.as_str() {
            "fmt" => super::fmt::run_fmt(false, vec![])?,
            "lint" => super::lint::run_lint(false, false, vec![])?,
            "check" => {
                let _ = super::check::run_check(None, true, false);
            }
            other => {
                // Try as a [scripts] entry
                if manifest.scripts.contains_key(other) {
                    super::scripts::run_scripts(Some(other.to_string()), vec![])
                        .map_err(|e| anyhow::anyhow!("{}", e))?;
                } else {
                    anyhow::bail!("Unknown hook '{}'. Must be 'fmt', 'lint', 'check', or a [scripts] entry.", other);
                }
            }
        }
    }
    Ok(())
}
