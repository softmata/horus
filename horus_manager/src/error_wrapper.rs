//! Error translation for cargo and pip output.
//!
//! Translates cryptic native tool errors into actionable HORUS hints.
//! Used by `horus run`, `horus build`, `horus test` when cargo/pip fails.

use colored::*;

/// Wrap a cargo error message with HORUS-specific hints.
///
/// Scans stderr for known patterns and appends actionable suggestions.
/// Returns `Some(hint)` if a known pattern was found, `None` otherwise.
pub fn cargo_error_hint(stderr: &str) -> Option<String> {
    // Missing crate
    if let Some(crate_name) = extract_pattern(stderr, r"no matching package named `([^`]+)`") {
        return Some(format!(
            "Crate '{}' not found on crates.io. Check the name or add it with:\n  {}",
            crate_name.cyan(),
            format!("horus add {}", crate_name).green()
        ));
    }

    // Version not found
    if let Some(crate_name) = extract_pattern(stderr, r"failed to select a version for the requirement `([^`]+)`") {
        return Some(format!(
            "No matching version for '{}'. Try:\n  {}",
            crate_name.cyan(),
            format!("horus add {} (without version to use latest)", crate_name).green()
        ));
    }

    // Missing feature
    if stderr.contains("feature") && stderr.contains("does not exist") {
        if let Some(feature) = extract_pattern(stderr, r"feature `([^`]+)` does not exist") {
            return Some(format!(
                "Feature '{}' does not exist in the crate. Check available features with:\n  {}",
                feature.cyan(),
                "cargo doc --open".green()
            ));
        }
    }

    // Linker error — missing system library
    if stderr.contains("ld: cannot find -l") || stderr.contains("cannot find -l") {
        if let Some(lib) = extract_pattern(stderr, r"cannot find -l(\w+)") {
            return Some(format!(
                "Missing system library: lib{}. Install it with:\n  {}",
                lib.cyan(),
                format!("sudo apt install lib{}-dev", lib).green()
            ));
        }
    }

    // pkg-config missing
    if stderr.contains("pkg-config") && (stderr.contains("not found") || stderr.contains("Could not run")) {
        return Some(format!(
            "pkg-config not found. Install with:\n  {}",
            "sudo apt install pkg-config".green()
        ));
    }

    // OpenSSL missing (very common)
    if stderr.contains("openssl") && stderr.contains("Could not find directory") {
        return Some(format!(
            "OpenSSL development headers not found. Install with:\n  {}",
            "sudo apt install libssl-dev pkg-config".green()
        ));
    }

    // cc / C compiler missing
    if stderr.contains("failed to run custom build command") && stderr.contains("cc") {
        return Some(format!(
            "C compiler not found. Install build essentials:\n  {}",
            "sudo apt install build-essential".green()
        ));
    }

    None
}

/// Wrap a pip error message with HORUS-specific hints.
///
/// Scans stderr for known patterns and appends actionable suggestions.
/// Returns `Some(hint)` if a known pattern was found, `None` otherwise.
pub fn pip_error_hint(stderr: &str) -> Option<String> {
    // Package not found
    if stderr.contains("No matching distribution found for") {
        if let Some(pkg) = extract_pattern(stderr, r"No matching distribution found for ([^\s]+)") {
            return Some(format!(
                "Package '{}' not found on PyPI. Check the name or install from a different source:\n  {}",
                pkg.cyan(),
                format!("horus install {} --source pypi", pkg).green()
            ));
        }
    }

    // Version conflict
    if stderr.contains("ResolutionImpossible") || stderr.contains("incompatible versions") {
        return Some(
            "Python dependency conflict detected. Try:\n  \
             1. Relax version constraints in horus.toml\n  \
             2. Run `horus deps tree` to see the full dependency graph"
                .to_string(),
        );
    }

    // Build wheel failure (usually missing C headers)
    if stderr.contains("Failed building wheel for") {
        if let Some(pkg) = extract_pattern(stderr, r"Failed building wheel for ([^\s]+)") {
            return Some(format!(
                "Failed to build '{}'. This package likely needs C/C++ build tools:\n  {}",
                pkg.cyan(),
                "sudo apt install build-essential python3-dev".green()
            ));
        }
    }

    // externally-managed-environment (Python 3.11+ / Debian)
    if stderr.contains("externally-managed-environment") {
        return Some(format!(
            "System Python is externally managed. Use:\n  {} or \n  {}",
            "horus install --global <package>".green(),
            "python3 -m venv .venv && source .venv/bin/activate".green()
        ));
    }

    // Permission denied
    if stderr.contains("Permission denied") && stderr.contains("pip") {
        return Some(format!(
            "Permission denied. Don't use sudo with pip. Instead:\n  {}",
            "python3 -m venv .venv && source .venv/bin/activate".green()
        ));
    }

    None
}

/// Extract a capture group from stderr using a regex pattern.
fn extract_pattern(text: &str, pattern: &str) -> Option<String> {
    let re = regex::Regex::new(pattern).ok()?;
    re.captures(text)
        .and_then(|caps| caps.get(1))
        .map(|m| m.as_str().to_string())
}

/// Format a HORUS diagnostic prefix for wrapped errors.
pub fn format_diagnostic(tool: &str, hint: &str) -> String {
    format!(
        "\n{} {} {}\n{}",
        "horus".bold().cyan(),
        "hint".bold().yellow(),
        format!("[{}]", tool).dimmed(),
        hint
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Cargo error hints ───────────────────────────────────────────────

    #[test]
    fn cargo_missing_crate() {
        let stderr = "error: no matching package named `nonexistent-crate` found";
        let hint = cargo_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("nonexistent-crate"));
    }

    #[test]
    fn cargo_version_mismatch() {
        let stderr = "error: failed to select a version for the requirement `tokio = \"^99.0\"`";
        let hint = cargo_error_hint(stderr);
        assert!(hint.is_some());
    }

    #[test]
    fn cargo_missing_feature() {
        let stderr = "error: feature `nonexistent` does not exist for package `serde`";
        let hint = cargo_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("nonexistent"));
    }

    #[test]
    fn cargo_missing_linker_lib() {
        let stderr = "/usr/bin/ld: cannot find -lssl: No such file";
        let hint = cargo_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("libssl-dev"));
    }

    #[test]
    fn cargo_missing_pkg_config() {
        let stderr = "Could not run `pkg-config`";
        let hint = cargo_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("apt install pkg-config"));
    }

    #[test]
    fn cargo_missing_openssl() {
        let stderr = "Could not find directory of openssl installation";
        let hint = cargo_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("libssl-dev"));
    }

    #[test]
    fn cargo_unrecognized_error_returns_none() {
        let stderr = "error[E0308]: mismatched types";
        assert!(cargo_error_hint(stderr).is_none());
    }

    // ── Pip error hints ─────────────────────────────────────────────────

    #[test]
    fn pip_package_not_found() {
        let stderr = "ERROR: No matching distribution found for nonexistent-pkg";
        let hint = pip_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("nonexistent-pkg"));
    }

    #[test]
    fn pip_version_conflict() {
        let stderr = "ERROR: ResolutionImpossible: requirements conflict";
        let hint = pip_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("dependency conflict"));
    }

    #[test]
    fn pip_build_wheel_failure() {
        let stderr = "ERROR: Failed building wheel for numpy";
        let hint = pip_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("numpy"));
    }

    #[test]
    fn pip_externally_managed() {
        let stderr = "error: externally-managed-environment";
        let hint = pip_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("venv"));
    }

    #[test]
    fn pip_permission_denied() {
        let stderr = "ERROR: Could not install packages due to an OSError: Permission denied (pip)";
        let hint = pip_error_hint(stderr);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("Don't use sudo"));
    }

    #[test]
    fn pip_unrecognized_error_returns_none() {
        let stderr = "SyntaxError: invalid syntax";
        assert!(pip_error_hint(stderr).is_none());
    }

    // ── Diagnostic formatting ───────────────────────────────────────────

    #[test]
    fn format_diagnostic_includes_tool() {
        let output = format_diagnostic("cargo", "Install libssl-dev");
        assert!(output.contains("cargo"));
        assert!(output.contains("hint"));
        assert!(output.contains("Install libssl-dev"));
    }
}
