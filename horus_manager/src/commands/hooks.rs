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
                    anyhow::bail!(
                        "Unknown hook '{}'. Must be 'fmt', 'lint', 'check', or a [scripts] entry.",
                        other
                    );
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifest::{HooksConfig, HorusManifest, IgnoreConfig, PackageInfo, TargetType};
    use std::collections::BTreeMap;

    fn empty_manifest() -> HorusManifest {
        HorusManifest {
            package: PackageInfo {
                name: "test".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
                target_type: TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: HooksConfig::default(),
        }
    }

    fn manifest_with_hooks(phase: &str, hooks: Vec<&str>) -> HorusManifest {
        let mut m = empty_manifest();
        let hook_strings: Vec<String> = hooks.iter().map(|s| s.to_string()).collect();
        match phase {
            "pre_run" => m.hooks.pre_run = hook_strings,
            "pre_build" => m.hooks.pre_build = hook_strings,
            "pre_test" => m.hooks.pre_test = hook_strings,
            "post_test" => m.hooks.post_test = hook_strings,
            _ => {}
        }
        m
    }

    #[test]
    fn empty_hooks_pre_run_is_noop() {
        let m = empty_manifest();
        assert!(run_hooks("pre_run", &m).is_ok());
    }

    #[test]
    fn empty_hooks_pre_build_is_noop() {
        let m = empty_manifest();
        assert!(run_hooks("pre_build", &m).is_ok());
    }

    #[test]
    fn empty_hooks_pre_test_is_noop() {
        let m = empty_manifest();
        assert!(run_hooks("pre_test", &m).is_ok());
    }

    #[test]
    fn empty_hooks_post_test_is_noop() {
        let m = empty_manifest();
        assert!(run_hooks("post_test", &m).is_ok());
    }

    #[test]
    fn unknown_phase_returns_ok() {
        // Unknown phase is NOT an error — it returns Ok(()) with no hooks
        let m = empty_manifest();
        assert!(run_hooks("nonexistent_phase", &m).is_ok());
    }

    #[test]
    fn unknown_phase_with_hooks_still_ok() {
        // Even with hooks configured, unknown phase skips them
        let m = manifest_with_hooks("pre_run", vec!["fmt"]);
        assert!(run_hooks("unknown", &m).is_ok());
    }

    #[test]
    fn unknown_hook_name_errors() {
        let m = manifest_with_hooks("pre_run", vec!["nonexistent_command"]);
        let result = run_hooks("pre_run", &m);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("Unknown hook") && err.contains("nonexistent_command"),
            "Expected 'Unknown hook' error mentioning 'nonexistent_command', got: {err}"
        );
    }

    #[test]
    fn unknown_hook_name_in_pre_build_errors() {
        let m = manifest_with_hooks("pre_build", vec!["does_not_exist"]);
        let result = run_hooks("pre_build", &m);
        assert!(result.is_err());
    }

    #[test]
    fn script_name_not_in_scripts_section_errors() {
        // Hook name that's not fmt/lint/check AND not in [scripts] → error
        let m = manifest_with_hooks("pre_test", vec!["my_custom_hook"]);
        let result = run_hooks("pre_test", &m);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(err.contains("Unknown hook"));
    }

    #[test]
    fn script_name_in_scripts_section_dispatches() {
        // Hook name that matches a [scripts] entry should attempt to run it
        let mut m = manifest_with_hooks("post_test", vec!["my_script"]);
        m.scripts
            .insert("my_script".to_string(), "echo hello".to_string());
        // This will try to run "echo hello" — which may fail depending on environment
        // but should NOT fail with "Unknown hook" error
        let result = run_hooks("post_test", &m);
        match result {
            Ok(()) => {} // Script ran successfully
            Err(e) => {
                let msg = format!("{e}");
                assert!(
                    !msg.contains("Unknown hook"),
                    "Script entry should be recognized, not 'Unknown hook': {msg}"
                );
            }
        }
    }

    #[test]
    fn hooks_config_is_empty_default() {
        let h = HooksConfig::default();
        assert!(h.is_empty());
    }

    #[test]
    fn hooks_config_is_not_empty_with_pre_run() {
        let mut h = HooksConfig::default();
        h.pre_run = vec!["fmt".to_string()];
        assert!(!h.is_empty());
    }
}
