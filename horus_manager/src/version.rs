use anyhow::{bail, Context, Result};
use colored::*;
use serde::Deserialize;
use std::fs;
use std::path::{Path, PathBuf};

/// The canonical one-line installer, quoted in the mismatch message.
///
/// Kept next to the message it appears in so a URL change is one edit. This is
/// the form README.md:33 publishes; the version pin has to go on `bash`, not on
/// `curl`, because `VAR=x curl ... | bash` exports the variable to *curl* and
/// the installer never sees it.
const INSTALL_ONE_LINER: &str = "curl -fsSL https://github.com/softmata/horus/raw/main/install.sh";

/// The topic-header version this binary was compiled against.
///
/// Mirrors `TOPIC_VERSION` in horus_core/src/communication/topic/header.rs,
/// which is `pub(crate)` and so cannot be imported from here. This — not the
/// version string — is what decides whether the CLI can attach to the shared
/// memory the installed libraries create: a CLI and a source tree that both
/// called themselves "0.4.0" were once 93 commits apart with topic versions 3
/// and 4, and the version string had no way to say so.
/// `cli_topic_version_matches_horus_core` fails if the two ever drift.
pub const CLI_TOPIC_VERSION: u32 = 4;

/// Get the CLI version from Cargo.toml at compile time
pub fn get_cli_version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// Get the installed library version from ~/.horus/installed_version
pub fn get_installed_version() -> Result<Option<String>> {
    let version_file = get_version_file_path()?;

    if !version_file.exists() {
        return Ok(None);
    }

    let version = fs::read_to_string(&version_file)
        .context("failed to read installed version file")?
        .trim()
        .to_string();

    Ok(Some(version))
}

/// Get the path to the version tracking file
pub fn get_version_file_path() -> Result<PathBuf> {
    Ok(crate::paths::home_dir()?.join(".horus/installed_version"))
}

/// Get the path to the richer install record written alongside it.
pub fn get_install_manifest_path() -> Result<PathBuf> {
    Ok(crate::paths::home_dir()?.join(".horus/install_manifest.toml"))
}

/// What install.sh and `horus self update` record about the install they made.
///
/// Every field is optional and unknown keys are ignored on purpose: this file
/// is written by a shell script and read by a binary that may be older or newer
/// than the script that wrote it, so a field that is missing, renamed or added
/// must degrade the diagnostic rather than fail the command the user asked for.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct InstallManifest {
    pub version: Option<String>,
    pub tag: Option<String>,
    pub commit: Option<String>,
    /// The horus_core topic-header version of the installed tree. Compared
    /// against [`CLI_TOPIC_VERSION`]; this is the only field that can prove an
    /// ABI break, since two different trees can share one version string.
    pub topic_version: Option<u32>,
    pub source_dir: Option<PathBuf>,
    pub binary: Option<PathBuf>,
    pub install_method: Option<String>,
    pub installed_at: Option<String>,
}

/// Read ~/.horus/install_manifest.toml, or `None` when it is absent or garbled.
///
/// Returns `Option`, not `Result`, so that no caller can turn "no manifest" into
/// a failure: every install made before this file existed lacks it, and doctor.rs
/// and the version gate both have to keep working for those users.
pub fn get_install_manifest() -> Option<InstallManifest> {
    let path = get_install_manifest_path().ok()?;
    read_install_manifest_at(&path)
}

fn read_install_manifest_at(path: &Path) -> Option<InstallManifest> {
    if !path.exists() {
        return None;
    }

    let text = match fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) => {
            log::debug!("could not read {}: {}", path.display(), e);
            return None;
        }
    };

    match toml::from_str::<InstallManifest>(&text) {
        Ok(manifest) => Some(manifest),
        Err(e) => {
            // A truncated or hand-edited manifest is a diagnostic we lose, not a
            // reason to refuse to run.
            log::warn!("ignoring malformed {}: {}", path.display(), e);
            None
        }
    }
}

/// Which file supplied the installed-side numbers, so the message can point at
/// the thing the user would have to look at or delete.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StateSource {
    Manifest,
    LegacyVersionFile,
}

impl StateSource {
    fn display_path(self) -> &'static str {
        match self {
            StateSource::Manifest => "~/.horus/install_manifest.toml",
            StateSource::LegacyVersionFile => "~/.horus/installed_version",
        }
    }
}

/// The comparison the gate makes, gathered once so that the printer, the strict
/// gate and the tests all reason about the same values.
#[derive(Debug, Clone)]
struct VersionCheck {
    cli_version: String,
    cli_topic_version: u32,
    installed_version: Option<String>,
    installed_topic_version: Option<u32>,
    source: Option<StateSource>,
}

impl VersionCheck {
    fn version_mismatch(&self) -> bool {
        matches!(&self.installed_version, Some(v) if v != &self.cli_version)
    }

    /// A differing topic version is not a cosmetic mismatch: it is the shm ABI.
    fn topic_mismatch(&self) -> bool {
        matches!(self.installed_topic_version, Some(t) if t != self.cli_topic_version)
    }

    fn is_compatible(&self) -> bool {
        !self.version_mismatch() && !self.topic_mismatch()
    }
}

/// Read whatever install state exists. Nothing here is fatal: a machine with no
/// record at all yields a check that compares clean, which is the pre-existing
/// behaviour for the (large) cohort that never had these files written.
fn collect_version_check() -> VersionCheck {
    let cli_version = get_cli_version().to_string();
    log::debug!("CLI version: {}", cli_version);

    // An unreadable version file (permissions, a directory in its place) must not
    // take down `horus new`; downgrade it to a missing record.
    let legacy_version = match get_installed_version() {
        Ok(version) => version,
        Err(e) => {
            log::warn!("could not read the installed version file: {}", e);
            None
        }
    };

    // Prefer the manifest: it is the only record carrying topic_version, and the
    // bare installed_version string provably cannot express the break that
    // actually shipped.
    let manifest = get_install_manifest();
    let installed_topic_version = manifest.as_ref().and_then(|m| m.topic_version);
    let (installed_version, source) = match manifest.as_ref().and_then(|m| m.version.clone()) {
        Some(version) => (Some(version), Some(StateSource::Manifest)),
        None => match legacy_version {
            Some(version) => (Some(version), Some(StateSource::LegacyVersionFile)),
            None => (None, manifest.as_ref().map(|_| StateSource::Manifest)),
        },
    };

    log::debug!(
        "installed HORUS: version={:?} topic_version={:?} source={:?}",
        installed_version,
        installed_topic_version,
        source
    );

    VersionCheck {
        cli_version,
        cli_topic_version: CLI_TOPIC_VERSION,
        installed_version,
        installed_topic_version,
        source,
    }
}

/// `HORUS_STRICT_VERSION=1` turns the warning back into a hard failure.
///
/// The default is a warning because the old unconditional `bail!` permanently
/// bricked the alpha..v0.1.9 cohort: `horus new` and any dependency-resolving
/// `horus run` exited 1, and the remedy the message printed (./install.sh) could
/// not clear it, because nothing had written ~/.horus/installed_version since
/// v0.2.0. CI, which wants a mismatched toolchain to fail loudly, opts back in.
fn strict_version_gate() -> bool {
    std::env::var("HORUS_STRICT_VERSION")
        .as_deref()
        .map(env_flag_is_set)
        .unwrap_or(false)
}

fn env_flag_is_set(value: &str) -> bool {
    value == "1" || value.eq_ignore_ascii_case("true") || value.eq_ignore_ascii_case("yes")
}

/// Check if the CLI version matches the installed library version.
///
/// Never fails on a *missing* record — only on a mismatch, and only when
/// HORUS_STRICT_VERSION is set. The old shape had this inverted: no file passed
/// silently while a stale file was unrecoverably fatal.
pub fn check_version_compatibility() -> Result<()> {
    report_mismatch(&collect_version_check(), strict_version_gate())
}

/// Same check, kept as its own entry point for the `horus new` dispatch in
/// main.rs. The two used to differ only in which remedy they printed, and there
/// is one remedy block now.
pub fn check_and_prompt_update() -> Result<()> {
    check_version_compatibility()
}

fn report_mismatch(check: &VersionCheck, strict: bool) -> Result<()> {
    if check.is_compatible() {
        return Ok(());
    }

    print_version_mismatch(check);

    if strict {
        if check.topic_mismatch() {
            bail!(
                "HORUS topic ABI mismatch: CLI topic_version {} vs installed {} \
                 (HORUS_STRICT_VERSION is set)",
                check.cli_topic_version,
                check
                    .installed_topic_version
                    .map(|t| t.to_string())
                    .unwrap_or_else(|| "unknown".to_string())
            );
        }
        bail!(
            "HORUS version mismatch: CLI {} vs installed {} (HORUS_STRICT_VERSION is set)",
            check.cli_version,
            check.installed_version.as_deref().unwrap_or("unknown")
        );
    }

    eprintln!(
        "{} Continuing anyway. Set {} to make this a hard error (CI).",
        "Note:".cyan(),
        "HORUS_STRICT_VERSION=1".cyan()
    );
    eprintln!();
    Ok(())
}

/// Print the version mismatch warning: both sides, both topic versions when
/// known, and the exact command that reconciles them.
fn print_version_mismatch(check: &VersionCheck) {
    let installed = check.installed_version.as_deref().unwrap_or("unknown");
    log::warn!(
        "Version mismatch: CLI={} (topic {}), installed={} (topic {:?}). Run `horus self update`.",
        check.cli_version,
        check.cli_topic_version,
        installed,
        check.installed_topic_version
    );

    eprintln!();
    eprintln!(
        "{} {}",
        "!".yellow().bold(),
        "HORUS version mismatch".yellow().bold()
    );
    eprintln!();

    let from = match check.source {
        Some(source) => format!("(from {})", source.display_path()),
        None => String::new(),
    };
    eprintln!("  CLI version:         {}", check.cli_version.green());
    if check.version_mismatch() {
        eprintln!(
            "  Installed HORUS:     {} {}",
            installed.red(),
            from.dimmed()
        );
    } else {
        eprintln!(
            "  Installed HORUS:     {} {}",
            installed.green(),
            from.dimmed()
        );
    }

    if let Some(installed_topic) = check.installed_topic_version {
        eprintln!(
            "  CLI topic ABI:       {}",
            check.cli_topic_version.to_string().green()
        );
        if check.topic_mismatch() {
            eprintln!(
                "  Installed topic ABI: {}",
                installed_topic.to_string().red()
            );
        } else {
            eprintln!(
                "  Installed topic ABI: {}",
                installed_topic.to_string().green()
            );
        }
    }
    eprintln!();

    if check.topic_mismatch() {
        eprintln!(
            "{} The shared-memory layout differs between the two.",
            "ABI break:".red().bold()
        );
        eprintln!("  Nodes built against the installed libraries publish topic segments this");
        eprintln!("  CLI's runtime refuses to attach to. `horus run` will fail with a topic");
        eprintln!("  version error rather than corrupt data, but it will fail.");
    } else {
        eprintln!(
            "{} The CLI and the libraries your projects build against are",
            "Note:".cyan()
        );
        eprintln!("  different installs. The version string alone does not prove they agree:");
        eprintln!("  0.4.0 shipped with two different topic ABIs, which is why topic_version");
        eprintln!("  is recorded in ~/.horus/install_manifest.toml.");
    }
    eprintln!();

    eprintln!("{}", "Reconcile with:".cyan());
    eprintln!("  {}", "horus self update".cyan());
    eprintln!("or install the exact version this CLI came from:");
    eprintln!(
        "  {}",
        format!(
            "{} | HORUS_VERSION=v{} bash",
            INSTALL_ONE_LINER, check.cli_version
        )
        .cyan()
    );
    if let Some(horus_root) = find_horus_source() {
        eprintln!("or, from your HORUS checkout:");
        eprintln!(
            "  {}",
            format!("cd {} && ./install.sh", horus_root.display()).cyan()
        );
    }
    eprintln!();
}

/// Find the HORUS source directory by looking for install.sh
fn find_horus_source() -> Option<PathBuf> {
    // Common locations to check
    let search_paths = vec![
        PathBuf::from("."),
        PathBuf::from(".."),
        PathBuf::from("../.."),
        dirs::home_dir()?.join("softmata/horus"),
        dirs::home_dir()?.join("HORUS"),
    ];

    for path in search_paths {
        let install_script = path.join("install.sh");
        if install_script.exists() {
            // The first three candidates are relative, and this path is printed
            // as a `cd` the user pastes later from some other directory, where
            // "cd .." names a different place than it did here. Resolve it while
            // we still know the process cwd.
            return Some(path.canonicalize().unwrap_or(path));
        }
    }

    None
}

/// Extract version from package directory name (e.g., "horus@0.1.0" -> "0.1.0")
#[cfg(test)]
fn extract_version_from_path(path: &std::path::Path) -> Option<String> {
    path.file_name()?
        .to_str()?
        .split('@')
        .nth(1)
        .map(|s| s.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A check with no install state at all — the shape every install made
    /// before install.sh started writing these files produces.
    fn empty_check() -> VersionCheck {
        VersionCheck {
            cli_version: "0.4.0".to_string(),
            cli_topic_version: CLI_TOPIC_VERSION,
            installed_version: None,
            installed_topic_version: None,
            source: None,
        }
    }

    // ========================================================================
    // get_cli_version tests
    // ========================================================================

    #[test]
    fn test_get_cli_version_returns_valid_semver() {
        let version = get_cli_version();
        assert!(!version.is_empty(), "CLI version should not be empty");
        // Should be a valid semver-like string (e.g., "0.2.0")
        let parts: Vec<&str> = version.split('.').collect();
        assert!(
            parts.len() >= 2,
            "Version '{}' should have at least major.minor",
            version
        );
        // Major and minor should be numeric
        assert!(
            parts[0].parse::<u32>().is_ok(),
            "Major version '{}' should be numeric",
            parts[0]
        );
        assert!(
            parts[1].parse::<u32>().is_ok(),
            "Minor version '{}' should be numeric",
            parts[1]
        );
    }

    #[test]
    fn test_get_cli_version_is_stable() {
        // Calling multiple times should return the same value
        let v1 = get_cli_version();
        let v2 = get_cli_version();
        assert_eq!(v1, v2);
    }

    // ========================================================================
    // get_version_file_path tests
    // ========================================================================

    #[test]
    fn test_get_version_file_path_ends_with_expected() {
        let path = get_version_file_path().unwrap();
        assert!(
            path.ends_with(".horus/installed_version"),
            "Path should end with .horus/installed_version, got: {:?}",
            path
        );
    }

    #[test]
    fn test_get_version_file_path_is_absolute() {
        let path = get_version_file_path().unwrap();
        assert!(
            path.is_absolute(),
            "Version file path should be absolute, got: {:?}",
            path
        );
    }

    #[test]
    fn test_get_install_manifest_path_ends_with_expected() {
        // install.sh and `horus self update` write to this exact path; if it
        // moves, the gate silently stops reading anything.
        let path = get_install_manifest_path().unwrap();
        assert!(
            path.ends_with(".horus/install_manifest.toml"),
            "Path should end with .horus/install_manifest.toml, got: {:?}",
            path
        );
        assert!(path.is_absolute(), "Manifest path should be absolute");
    }

    // ========================================================================
    // extract_version_from_path tests
    // ========================================================================

    #[test]
    fn test_extract_version_simple() {
        let path = Path::new("/packages/horus@0.1.0");
        assert_eq!(extract_version_from_path(path), Some("0.1.0".to_string()));
    }

    #[test]
    fn test_extract_version_complex_semver() {
        let path = Path::new("/packages/my-package@1.2.3-beta.1");
        assert_eq!(
            extract_version_from_path(path),
            Some("1.2.3-beta.1".to_string())
        );
    }

    #[test]
    fn test_extract_version_no_at_symbol() {
        let path = Path::new("/packages/horus");
        assert_eq!(extract_version_from_path(path), None);
    }

    #[test]
    fn test_extract_version_empty_version() {
        let path = Path::new("/packages/horus@");
        // split('@').nth(1) returns Some("") for "horus@"
        assert_eq!(extract_version_from_path(path), Some("".to_string()));
    }

    #[test]
    fn test_extract_version_multiple_at_symbols() {
        // Only the first @ matters for the split
        let path = Path::new("/packages/name@1.0@extra");
        assert_eq!(extract_version_from_path(path), Some("1.0".to_string()));
    }

    #[test]
    fn test_extract_version_nested_path() {
        let path = Path::new("/home/user/.horus/packages/lidar-driver@2.0.0");
        assert_eq!(extract_version_from_path(path), Some("2.0.0".to_string()));
    }

    // ========================================================================
    // get_installed_version tests (with temp files)
    // ========================================================================

    #[test]
    fn test_get_installed_version_no_file() {
        // The actual function reads from ~/.horus/installed_version
        // If the file doesn't exist, it should return Ok(None)
        // This is hard to test in isolation without mocking the filesystem
        // but we can at least verify it doesn't panic
        let result = get_installed_version();
        assert!(
            result.is_ok(),
            "Should not error even if file doesn't exist"
        );
    }

    // ========================================================================
    // install_manifest reader tests
    // ========================================================================

    #[test]
    fn test_read_install_manifest_missing_file_is_none() {
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("install_manifest.toml");
        assert!(
            read_install_manifest_at(&missing).is_none(),
            "a missing manifest must read as None, never as an error"
        );
    }

    #[test]
    fn test_read_install_manifest_full_record() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("install_manifest.toml");
        fs::write(
            &path,
            r#"
version = "0.4.0"
tag = "v0.4.0"
commit = "0123456789abcdef0123456789abcdef01234567"
topic_version = 4
source_dir = "/home/u/.horus/cache/horus@0.4.0"
binary = "/home/u/.cargo/bin/horus"
install_method = "release-binary"
installed_at = "2026-08-31T00:00:00Z"
"#,
        )
        .unwrap();

        let manifest = read_install_manifest_at(&path).expect("well-formed manifest should parse");
        assert_eq!(manifest.version.as_deref(), Some("0.4.0"));
        assert_eq!(manifest.tag.as_deref(), Some("v0.4.0"));
        assert_eq!(manifest.topic_version, Some(4));
        assert_eq!(manifest.install_method.as_deref(), Some("release-binary"));
        assert_eq!(
            manifest.source_dir,
            Some(PathBuf::from("/home/u/.horus/cache/horus@0.4.0"))
        );
    }

    #[test]
    fn test_read_install_manifest_garbled_is_none() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("install_manifest.toml");
        // Half-written file: install.sh interrupted mid-write leaves exactly this.
        fs::write(&path, "version = \"0.4.0\"\ntopic_ver").unwrap();
        assert!(
            read_install_manifest_at(&path).is_none(),
            "a garbled manifest must degrade to None, not blow up the command"
        );
    }

    #[test]
    fn test_read_install_manifest_partial_record() {
        // An older install.sh that wrote fewer fields still gives us a version.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("install_manifest.toml");
        fs::write(&path, "version = \"0.3.1\"\n").unwrap();

        let manifest = read_install_manifest_at(&path).expect("partial manifest should parse");
        assert_eq!(manifest.version.as_deref(), Some("0.3.1"));
        assert_eq!(manifest.topic_version, None);
    }

    #[test]
    fn test_read_install_manifest_ignores_unknown_keys() {
        // A newer install.sh must not brick an older CLI's diagnostics.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("install_manifest.toml");
        fs::write(
            &path,
            "version = \"0.4.0\"\ntopic_version = 4\nfuture_field = \"whatever\"\n",
        )
        .unwrap();

        let manifest = read_install_manifest_at(&path).expect("unknown keys should be ignored");
        assert_eq!(manifest.version.as_deref(), Some("0.4.0"));
        assert_eq!(manifest.topic_version, Some(4));
    }

    #[test]
    fn test_get_install_manifest_never_panics() {
        // Reads the real ~/.horus; whatever is there, it may not panic and may
        // not be an error type.
        let _ = get_install_manifest();
    }

    // ========================================================================
    // gate semantics
    // ========================================================================

    #[test]
    fn test_missing_install_state_is_not_fatal() {
        // The pre-fix code returned Ok here too, but for the wrong reason: a
        // missing file was the *silent* path while a stale file was fatal.
        let check = empty_check();
        assert!(check.is_compatible());
        report_mismatch(&check, false).expect("no install record must never fail the command");
        report_mismatch(&check, true).expect("nothing to compare is not a mismatch, even in CI");
    }

    #[test]
    fn test_version_mismatch_warns_but_does_not_error() {
        let check = VersionCheck {
            installed_version: Some("0.1.9".to_string()),
            source: Some(StateSource::LegacyVersionFile),
            ..empty_check()
        };
        assert!(check.version_mismatch());
        assert!(!check.is_compatible());
        // This is the alpha..v0.1.9 brick: it used to bail!, with a printed
        // remedy that could not clear it.
        report_mismatch(&check, false)
            .expect("a stale installed_version must not fail the command");
    }

    #[test]
    fn test_version_mismatch_is_fatal_under_strict() {
        let check = VersionCheck {
            installed_version: Some("0.1.9".to_string()),
            source: Some(StateSource::LegacyVersionFile),
            ..empty_check()
        };
        let err = report_mismatch(&check, true).expect_err("HORUS_STRICT_VERSION=1 must hard-fail");
        let msg = err.to_string();
        assert!(
            msg.contains("0.4.0"),
            "error should name the CLI version: {msg}"
        );
        assert!(
            msg.contains("0.1.9"),
            "error should name the installed version: {msg}"
        );
    }

    #[test]
    fn test_topic_version_mismatch_is_reported_as_an_abi_break() {
        // The break that actually shipped: both trees said 0.4.0, the binary
        // came from the tag (topic 3) and the source from main (topic 4).
        let check = VersionCheck {
            cli_topic_version: 3,
            installed_version: Some("0.4.0".to_string()),
            installed_topic_version: Some(4),
            source: Some(StateSource::Manifest),
            ..empty_check()
        };
        assert!(
            !check.version_mismatch(),
            "the version strings are equal — that is the whole point"
        );
        assert!(check.topic_mismatch());
        assert!(!check.is_compatible());

        report_mismatch(&check, false).expect("an ABI break still warns rather than bails");
        let err = report_mismatch(&check, true).expect_err("strict mode must fail on an ABI break");
        let msg = err.to_string();
        assert!(
            msg.contains("topic"),
            "strict error should name the topic ABI, not just the version: {msg}"
        );
        assert!(msg.contains('3') && msg.contains('4'), "both sides: {msg}");
    }

    #[test]
    fn test_matching_manifest_is_silent() {
        let check = VersionCheck {
            installed_version: Some("0.4.0".to_string()),
            installed_topic_version: Some(CLI_TOPIC_VERSION),
            source: Some(StateSource::Manifest),
            ..empty_check()
        };
        assert!(check.is_compatible());
        report_mismatch(&check, true).expect("a matching install passes even in strict mode");
    }

    #[test]
    fn test_check_version_compatibility_reads_real_state_without_erroring() {
        // Whatever this machine has in ~/.horus, the default (non-strict) gate
        // returns Ok. Skipped when the ambient environment has opted into the
        // hard gate, which is the one case where a mismatch legitimately fails.
        if strict_version_gate() {
            return;
        }
        check_version_compatibility().expect("the default gate must never fail a command");
        check_and_prompt_update().expect("the default gate must never fail a command");
    }

    #[test]
    fn test_env_flag_is_set_accepts_documented_spellings() {
        assert!(env_flag_is_set("1"));
        assert!(env_flag_is_set("true"));
        assert!(env_flag_is_set("TRUE"));
        assert!(env_flag_is_set("yes"));
        assert!(!env_flag_is_set("0"));
        assert!(!env_flag_is_set(""));
        assert!(!env_flag_is_set("no"));
    }

    // ========================================================================
    // find_horus_source tests
    // ========================================================================

    #[test]
    fn test_find_horus_source_returns_option() {
        // find_horus_source searches common locations
        // Result depends on the system, but should not panic
        let result = find_horus_source();
        // If we're running from the horus source tree, it might find it
        if let Some(path) = &result {
            assert!(
                path.join("install.sh").exists(),
                "Found path should contain install.sh"
            );
        }
    }

    // ========================================================================
    // print_version_mismatch tests (smoke test)
    // ========================================================================

    #[test]
    fn test_print_version_mismatch_does_not_panic() {
        // Smoke test: print_version_mismatch is a pure side-effect function
        // (writes to stderr). We verify it handles edge cases without panicking,
        // including empty strings, pre-release versions and a record that has a
        // topic_version but no version string.
        print_version_mismatch(&VersionCheck {
            cli_version: "1.0.0".to_string(),
            installed_version: Some("0.9.0".to_string()),
            source: Some(StateSource::LegacyVersionFile),
            ..empty_check()
        });
        print_version_mismatch(&VersionCheck {
            cli_version: String::new(),
            installed_version: Some(String::new()),
            ..empty_check()
        });
        print_version_mismatch(&VersionCheck {
            cli_version: "1.0.0-beta".to_string(),
            installed_version: Some("1.0.0".to_string()),
            installed_topic_version: Some(9),
            source: Some(StateSource::Manifest),
            ..empty_check()
        });
        print_version_mismatch(&VersionCheck {
            installed_version: None,
            installed_topic_version: Some(9),
            source: Some(StateSource::Manifest),
            ..empty_check()
        });
    }

    // ========================================================================
    // topic ABI mirror
    // ========================================================================

    #[test]
    fn cli_topic_version_matches_horus_core() {
        // CLI_TOPIC_VERSION mirrors a pub(crate) constant this crate cannot
        // import, so nothing but this test stops the two from drifting — and a
        // stale mirror would report "compatible" for exactly the shm break it
        // exists to catch. Skipped only when the sibling crate's source is not
        // on disk, which cannot happen in this workspace.
        let header = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../horus_core/src/communication/topic/header.rs");
        let Ok(source) = fs::read_to_string(&header) else {
            return;
        };
        let declared = source
            .lines()
            .find_map(|line| {
                let line = line.trim();
                let rest = line
                    .strip_prefix("pub(crate) const TOPIC_VERSION: u32 = ")
                    .or_else(|| line.strip_prefix("pub const TOPIC_VERSION: u32 = "))?;
                rest.trim_end_matches(';').trim().parse::<u32>().ok()
            })
            .expect("could not find TOPIC_VERSION in horus_core topic/header.rs");
        assert_eq!(
            declared, CLI_TOPIC_VERSION,
            "horus_core TOPIC_VERSION moved to {} — bump CLI_TOPIC_VERSION and the \
             topic_version install.sh writes, or the gate will call an ABI break compatible",
            declared
        );
    }

    // ========================================================================
    // Semver comparison tests
    // ========================================================================

    #[test]
    fn test_version_comparison() {
        use semver::Version;

        let v010 = Version::parse("0.1.0").unwrap();
        let v020 = Version::parse("0.2.0").unwrap();
        let v100 = Version::parse("1.0.0").unwrap();

        // Strict ordering: 0.1.0 < 0.2.0 < 1.0.0
        assert!(v010 < v020, "0.1.0 should be less than 0.2.0");
        assert!(v020 < v100, "0.2.0 should be less than 1.0.0");
        assert!(
            v010 < v100,
            "0.1.0 should be less than 1.0.0 (transitivity)"
        );

        // Pre-release is less than release
        let v100_beta = Version::parse("1.0.0-beta").unwrap();
        assert!(
            v100_beta < v100,
            "1.0.0-beta should be less than 1.0.0 (pre-release < release)"
        );

        // Patch ordering
        let v019 = Version::parse("0.1.9").unwrap();
        assert!(v010 < v019, "0.1.0 should be less than 0.1.9");
        assert!(v019 < v020, "0.1.9 should be less than 0.2.0");
    }

    #[test]
    fn test_version_parse_valid() {
        use semver::Version;

        // Standard semver
        let v1 = Version::parse("1.2.3");
        assert!(
            v1.is_ok(),
            "1.2.3 should parse as valid semver, got: {:?}",
            v1.err()
        );
        let v1 = v1.unwrap();
        assert_eq!(v1.major, 1);
        assert_eq!(v1.minor, 2);
        assert_eq!(v1.patch, 3);

        // Pre-release version
        let v2 = Version::parse("0.1.0-beta");
        assert!(
            v2.is_ok(),
            "0.1.0-beta should parse as valid semver, got: {:?}",
            v2.err()
        );
        let v2 = v2.unwrap();
        assert_eq!(v2.major, 0);
        assert_eq!(v2.minor, 1);
        assert_eq!(v2.patch, 0);
        assert!(
            !v2.pre.is_empty(),
            "Pre-release field should be non-empty for 0.1.0-beta"
        );

        // Build metadata version
        let v3 = Version::parse("2.0.0+build123");
        assert!(
            v3.is_ok(),
            "2.0.0+build123 should parse as valid semver, got: {:?}",
            v3.err()
        );
        let v3 = v3.unwrap();
        assert_eq!(v3.major, 2);
        assert_eq!(v3.minor, 0);
        assert_eq!(v3.patch, 0);
        assert!(
            !v3.build.is_empty(),
            "Build metadata should be non-empty for 2.0.0+build123"
        );

        // Pre-release + build metadata combined
        let v4 = Version::parse("1.0.0-alpha.1+build.456");
        assert!(
            v4.is_ok(),
            "1.0.0-alpha.1+build.456 should parse as valid semver, got: {:?}",
            v4.err()
        );
    }
}
