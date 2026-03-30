//! Environment synchronization — toolchains, system dependencies, lockfiles.
//!
//! Implements `horus sync` — one command to ensure identical dev environments:
//! - Rust toolchain via `rustup` (cross-platform)
//! - Python via `pyenv` (Unix) or python.org (Windows)
//! - C++ toolchain (cmake, g++/clang++) via platform package manager
//! - System libraries via apt/brew/choco per `horus.toml` dependency specs
//! - Version pinning via `horus.lock`

pub mod cpp;
pub mod lockfile;
pub mod python;
pub mod rust;
pub mod system;

use std::collections::BTreeMap;

/// Trait for extracting sync requirements from a project manifest.
///
/// Implemented by `HorusManifest` in horus_manager. Object-safe for future SDK extraction.
pub trait SyncManifest {
    /// Rust edition/toolchain requirement (e.g., "2021", "1.83.0").
    fn rust_edition(&self) -> Option<String>;
    /// Python version requirement (e.g., ">=3.9").
    fn python_version(&self) -> Option<String>;
    /// System dependencies (name → apt/brew/choco package names).
    fn system_deps(&self) -> Vec<SystemDep>;
    /// Whether C++ support is needed.
    fn needs_cpp(&self) -> bool;
    /// Project name for lockfile.
    fn project_name(&self) -> String;
}

/// A system dependency with per-platform package names.
#[derive(Debug, Clone)]
pub struct SystemDep {
    /// Logical name (e.g., "opencv", "eigen3").
    pub name: String,
    /// Debian/Ubuntu package name (e.g., "libopencv-dev").
    pub apt: Option<String>,
    /// Homebrew package name (e.g., "opencv").
    pub brew: Option<String>,
    /// Chocolatey/winget package name.
    pub choco: Option<String>,
    /// pkg-config name for detection (e.g., "opencv4").
    pub pkg_config: Option<String>,
}

/// Status of a single dependency check.
#[derive(Debug, Clone)]
pub struct DepStatus {
    pub name: String,
    pub required: bool,
    pub installed: bool,
    pub version: Option<String>,
    pub install_cmd: Option<String>,
}

/// Report from sync_environment or check_environment.
#[derive(Debug, Clone)]
pub struct SyncReport {
    pub items: Vec<DepStatus>,
    pub all_satisfied: bool,
}

impl SyncReport {
    pub fn new() -> Self {
        Self {
            items: Vec::new(),
            all_satisfied: true,
        }
    }

    pub fn add(&mut self, status: DepStatus) {
        if status.required && !status.installed {
            self.all_satisfied = false;
        }
        self.items.push(status);
    }
}

impl Default for SyncReport {
    fn default() -> Self {
        Self::new()
    }
}

/// Check the development environment without installing anything.
///
/// Returns a report of what's present and what's missing.
pub fn check_environment(manifest: &dyn SyncManifest) -> SyncReport {
    let mut report = SyncReport::new();

    // Check Rust
    report.add(rust::check_rust(manifest.rust_edition().as_deref()));

    // Check Python if needed
    if manifest.python_version().is_some() {
        report.add(python::check_python(manifest.python_version().as_deref()));
    }

    // Check C++ if needed
    if manifest.needs_cpp() {
        report.add(cpp::check_cmake());
        report.add(cpp::check_compiler());
    }

    // Check system deps
    for dep in manifest.system_deps() {
        report.add(system::check_system_dep(&dep));
    }

    report
}

/// Synchronize the development environment: check and install what's missing.
///
/// Returns a report of all actions taken.
pub fn sync_environment(manifest: &dyn SyncManifest) -> anyhow::Result<SyncReport> {
    let mut report = SyncReport::new();

    // Sync Rust
    report.add(rust::ensure_rust(manifest.rust_edition().as_deref())?);

    // Sync Python if needed
    if manifest.python_version().is_some() {
        report.add(python::ensure_python(manifest.python_version().as_deref())?);
    }

    // Sync C++ if needed
    if manifest.needs_cpp() {
        report.add(cpp::ensure_cmake()?);
        report.add(cpp::ensure_compiler()?);
    }

    // Sync system deps
    for dep in manifest.system_deps() {
        report.add(system::ensure_system_dep(&dep)?);
    }

    Ok(report)
}

/// Generate a lockfile from the current environment state.
pub fn generate_lockfile(manifest: &dyn SyncManifest) -> lockfile::SyncLockfile {
    let report = check_environment(manifest);
    let mut pinned = BTreeMap::new();

    for item in &report.items {
        if let Some(ver) = &item.version {
            pinned.insert(item.name.clone(), ver.clone());
        }
    }

    lockfile::SyncLockfile {
        version: 1,
        project: manifest.project_name(),
        pinned,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestManifest;

    impl SyncManifest for TestManifest {
        fn rust_edition(&self) -> Option<String> {
            Some("2021".to_string())
        }
        fn python_version(&self) -> Option<String> {
            None
        }
        fn system_deps(&self) -> Vec<SystemDep> {
            Vec::new()
        }
        fn needs_cpp(&self) -> bool {
            false
        }
        fn project_name(&self) -> String {
            "test-project".to_string()
        }
    }

    #[test]
    fn check_environment_detects_rust() {
        let manifest = TestManifest;
        let report = check_environment(&manifest);
        assert!(!report.items.is_empty());
        // Rust should be detected on this machine
        let rust_item = report.items.iter().find(|i| i.name == "rust").unwrap();
        assert!(
            rust_item.installed,
            "Rust should be installed on dev machine"
        );
    }

    #[test]
    fn generate_lockfile_creates_valid_lockfile() {
        let manifest = TestManifest;
        let lockfile = generate_lockfile(&manifest);
        assert_eq!(lockfile.version, 1);
        assert_eq!(lockfile.project, "test-project");
    }
}
