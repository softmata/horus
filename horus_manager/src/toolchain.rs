//! Shared cross-compilation toolchain support for C++ projects.
//!
//! Provides architecture detection, embedded CMake toolchain files, and
//! resolution logic used by both `horus deploy` and `horus build`/`horus run`.

use anyhow::{bail, Result};
use std::fs;
use std::path::{Path, PathBuf};

/// Supported target architectures for robotics platforms.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TargetArch {
    /// ARM64 (Raspberry Pi 4/5, Jetson Nano/Xavier/Orin)
    Aarch64,
    /// ARM 32-bit (Raspberry Pi 3, older boards)
    Armv7,
    /// x86_64 (Intel NUC, standard PCs)
    X86_64,
    /// Current host architecture
    Native,
}

impl TargetArch {
    /// Parse a target architecture from a string. Accepts common aliases.
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "aarch64" | "arm64" | "jetson" | "pi4" | "pi5" => Some(TargetArch::Aarch64),
            "armv7" | "arm" | "pi3" | "pi2" => Some(TargetArch::Armv7),
            "x86_64" | "x64" | "amd64" | "intel" => Some(TargetArch::X86_64),
            "native" | "host" | "local" => Some(TargetArch::Native),
            _ => None,
        }
    }

    /// Rust target triple for cross-compilation.
    pub fn rust_target(&self) -> &'static str {
        match self {
            TargetArch::Aarch64 => "aarch64-unknown-linux-gnu",
            TargetArch::Armv7 => "armv7-unknown-linux-gnueabihf",
            TargetArch::X86_64 => "x86_64-unknown-linux-gnu",
            TargetArch::Native => "", // Use default
        }
    }

    /// Human-readable display name.
    pub fn display_name(&self) -> &'static str {
        match self {
            TargetArch::Aarch64 => "ARM64 (aarch64)",
            TargetArch::Armv7 => "ARM32 (armv7)",
            TargetArch::X86_64 => "x86_64",
            TargetArch::Native => "native",
        }
    }

    /// Returns embedded CMake toolchain file content for cross-compilation.
    /// Returns `None` for native/x86_64 (no toolchain file needed).
    pub fn cmake_toolchain(&self) -> Option<&'static str> {
        match self {
            TargetArch::Aarch64 => Some(AARCH64_CMAKE_TOOLCHAIN),
            TargetArch::Armv7 => Some(ARMV7_CMAKE_TOOLCHAIN),
            TargetArch::X86_64 | TargetArch::Native => None,
        }
    }
}

pub const AARCH64_CMAKE_TOOLCHAIN: &str = r#"set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
"#;

pub const ARMV7_CMAKE_TOOLCHAIN: &str = r#"set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
"#;

/// Result of resolving a toolchain specification.
#[derive(Debug)]
pub enum ToolchainResolution {
    /// Use embedded toolchain content (write to `.horus/toolchains/`).
    Embedded {
        arch: TargetArch,
        content: &'static str,
    },
    /// Use a custom file path directly.
    CustomFile(PathBuf),
}

/// Resolve a toolchain specification string into a `ToolchainResolution`.
///
/// Accepts:
/// - Known arch names: "aarch64", "arm64", "jetson", "armv7", "arm", etc.
/// - Custom `.cmake` file paths: "/path/to/my-toolchain.cmake"
/// - Native/x86_64: returns `Ok(None)` (no toolchain file needed)
pub fn resolve_toolchain(spec: &str) -> Result<Option<ToolchainResolution>> {
    // Check if it's a file path (contains path separator or ends in .cmake)
    if spec.contains('/') || spec.contains('\\') || spec.ends_with(".cmake") {
        let path = PathBuf::from(spec);
        if !path.exists() {
            bail!(
                "Custom toolchain file not found: {}\n\
                 Provide an absolute path to a .cmake toolchain file, \
                 or use a built-in arch: aarch64, armv7, x86_64, native",
                spec
            );
        }
        return Ok(Some(ToolchainResolution::CustomFile(path)));
    }

    // Try to parse as a known architecture
    match TargetArch::from_str(spec) {
        Some(arch) => match arch.cmake_toolchain() {
            Some(content) => Ok(Some(ToolchainResolution::Embedded { arch, content })),
            None => Ok(None), // native/x86_64 — no toolchain needed
        },
        None => bail!(
            "Unknown toolchain target: '{}'\n\
             Supported targets: aarch64, arm64, jetson, pi4, pi5, armv7, arm, pi3, pi2, \
             x86_64, native\n\
             Or provide a path to a custom .cmake toolchain file",
            spec
        ),
    }
}

/// Resolve a toolchain spec and write the toolchain file to `.horus/toolchains/`.
/// Returns the path to the toolchain file, or `None` if no toolchain is needed.
pub fn write_toolchain_file(spec: &str, project_dir: &Path) -> Result<Option<PathBuf>> {
    match resolve_toolchain(spec)? {
        Some(ToolchainResolution::Embedded { content, .. }) => {
            let tc_dir = project_dir.join(".horus/toolchains");
            fs::create_dir_all(&tc_dir)?;
            let tc_path = tc_dir.join("toolchain.cmake");
            fs::write(&tc_path, content)?;
            Ok(Some(tc_path))
        }
        Some(ToolchainResolution::CustomFile(path)) => Ok(Some(path)),
        None => Ok(None),
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── TargetArch parsing ───────────────────────────────────────────────

    #[test]
    fn target_arch_aarch64_aliases() {
        for alias in &["aarch64", "arm64", "jetson", "pi4", "pi5"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "aarch64-unknown-linux-gnu");
        }
    }

    #[test]
    fn target_arch_armv7_aliases() {
        for alias in &["armv7", "arm", "pi3", "pi2"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "armv7-unknown-linux-gnueabihf");
        }
    }

    #[test]
    fn target_arch_x86_64_aliases() {
        for alias in &["x86_64", "x64", "amd64", "intel"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "x86_64-unknown-linux-gnu");
        }
    }

    #[test]
    fn target_arch_native_aliases() {
        for alias in &["native", "host", "local"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "");
        }
    }

    #[test]
    fn target_arch_unknown_returns_none() {
        assert!(TargetArch::from_str("mips").is_none());
        assert!(TargetArch::from_str("riscv").is_none());
        assert!(TargetArch::from_str("").is_none());
    }

    #[test]
    fn target_arch_case_insensitive() {
        assert!(TargetArch::from_str("AARCH64").is_some());
        assert!(TargetArch::from_str("Pi4").is_some());
        assert!(TargetArch::from_str("X86_64").is_some());
    }

    #[test]
    fn target_arch_display_names() {
        assert_eq!(TargetArch::Aarch64.display_name(), "ARM64 (aarch64)");
        assert_eq!(TargetArch::Armv7.display_name(), "ARM32 (armv7)");
        assert_eq!(TargetArch::X86_64.display_name(), "x86_64");
        assert_eq!(TargetArch::Native.display_name(), "native");
    }

    // ── cmake_toolchain ──────────────────────────────────────────────────

    #[test]
    fn cmake_toolchain_aarch64_content() {
        let tc = TargetArch::Aarch64.cmake_toolchain().unwrap();
        assert!(tc.contains("aarch64-linux-gnu-g++"));
        assert!(tc.contains("CMAKE_SYSTEM_PROCESSOR aarch64"));
        assert!(tc.contains("CMAKE_SYSTEM_NAME Linux"));
    }

    #[test]
    fn cmake_toolchain_armv7_content() {
        let tc = TargetArch::Armv7.cmake_toolchain().unwrap();
        assert!(tc.contains("arm-linux-gnueabihf-g++"));
        assert!(tc.contains("CMAKE_SYSTEM_PROCESSOR arm"));
        assert!(tc.contains("CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY"));
    }

    #[test]
    fn cmake_toolchain_native_none() {
        assert!(TargetArch::Native.cmake_toolchain().is_none());
    }

    #[test]
    fn cmake_toolchain_x86_64_none() {
        assert!(TargetArch::X86_64.cmake_toolchain().is_none());
    }

    // ── resolve_toolchain ────────────────────────────────────────────────

    #[test]
    fn resolve_known_arch_aarch64() {
        let result = resolve_toolchain("aarch64").unwrap();
        assert!(result.is_some());
        match result.unwrap() {
            ToolchainResolution::Embedded { arch, content } => {
                assert_eq!(arch, TargetArch::Aarch64);
                assert!(content.contains("aarch64-linux-gnu-g++"));
            }
            _ => panic!("expected Embedded"),
        }
    }

    #[test]
    fn resolve_known_arch_armv7() {
        let result = resolve_toolchain("armv7").unwrap();
        assert!(result.is_some());
        match result.unwrap() {
            ToolchainResolution::Embedded { arch, .. } => {
                assert_eq!(arch, TargetArch::Armv7);
            }
            _ => panic!("expected Embedded"),
        }
    }

    #[test]
    fn resolve_native_returns_none() {
        let result = resolve_toolchain("native").unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn resolve_x86_64_returns_none() {
        let result = resolve_toolchain("x86_64").unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn resolve_unknown_arch_errors() {
        let result = resolve_toolchain("mips");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Unknown toolchain target"));
        assert!(err.contains("mips"));
    }

    #[test]
    fn resolve_custom_file_nonexistent_errors() {
        let result = resolve_toolchain("/tmp/nonexistent-toolchain.cmake");
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("not found"));
    }

    #[test]
    fn resolve_custom_file_existing() {
        let dir = tempfile::tempdir().unwrap();
        let tc_file = dir.path().join("my-toolchain.cmake");
        fs::write(&tc_file, "set(CMAKE_SYSTEM_NAME Linux)").unwrap();

        let result = resolve_toolchain(tc_file.to_str().unwrap()).unwrap();
        assert!(result.is_some());
        match result.unwrap() {
            ToolchainResolution::CustomFile(path) => {
                assert_eq!(path, tc_file);
            }
            _ => panic!("expected CustomFile"),
        }
    }

    #[test]
    fn resolve_alias_jetson() {
        let result = resolve_toolchain("jetson").unwrap();
        assert!(result.is_some());
        match result.unwrap() {
            ToolchainResolution::Embedded { arch, .. } => {
                assert_eq!(arch, TargetArch::Aarch64);
            }
            _ => panic!("expected Embedded"),
        }
    }

    // ── write_toolchain_file ─────────────────────────────────────────────

    #[test]
    fn write_toolchain_aarch64_creates_file() {
        let dir = tempfile::tempdir().unwrap();
        let result = write_toolchain_file("aarch64", dir.path()).unwrap();
        assert!(result.is_some());

        let tc_path = result.unwrap();
        assert!(tc_path.exists());
        assert_eq!(
            tc_path,
            dir.path().join(".horus/toolchains/toolchain.cmake")
        );

        let content = fs::read_to_string(&tc_path).unwrap();
        assert!(content.contains("aarch64-linux-gnu-g++"));
    }

    #[test]
    fn write_toolchain_armv7_creates_file() {
        let dir = tempfile::tempdir().unwrap();
        let result = write_toolchain_file("armv7", dir.path()).unwrap();
        assert!(result.is_some());

        let content = fs::read_to_string(result.unwrap()).unwrap();
        assert!(content.contains("arm-linux-gnueabihf-g++"));
    }

    #[test]
    fn write_toolchain_native_returns_none() {
        let dir = tempfile::tempdir().unwrap();
        let result = write_toolchain_file("native", dir.path()).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn write_toolchain_custom_file_returns_path() {
        let dir = tempfile::tempdir().unwrap();
        let tc_file = dir.path().join("custom.cmake");
        fs::write(&tc_file, "set(CMAKE_SYSTEM_NAME Linux)").unwrap();

        let result = write_toolchain_file(tc_file.to_str().unwrap(), dir.path()).unwrap();
        assert!(result.is_some());
        assert_eq!(result.unwrap(), tc_file);
    }

    #[test]
    fn write_toolchain_overwrites_existing() {
        let dir = tempfile::tempdir().unwrap();

        // Write aarch64 first
        write_toolchain_file("aarch64", dir.path()).unwrap();

        // Overwrite with armv7
        let result = write_toolchain_file("armv7", dir.path()).unwrap();
        let content = fs::read_to_string(result.unwrap()).unwrap();
        assert!(
            content.contains("arm-linux-gnueabihf"),
            "should have overwritten with armv7 content"
        );
    }

    #[test]
    fn write_toolchain_unknown_errors() {
        let dir = tempfile::tempdir().unwrap();
        let result = write_toolchain_file("riscv", dir.path());
        result.unwrap_err();
    }

    // ── Toolchain detection (system probing) ────────────────────────────

    #[test]
    fn test_toolchain_detect_rust() {
        // On any system where this test suite runs, rustc and cargo must be
        // available (we compiled this binary with them).
        let rust_version = crate::registry::helpers::get_rust_version();
        assert!(
            rust_version.is_some(),
            "rustc should be detectable on the build host"
        );
        let version = rust_version.unwrap();
        assert!(
            !version.is_empty(),
            "Rust version string should be non-empty"
        );
        // Version should look like semver: at least "major.minor"
        assert!(
            version.contains('.'),
            "Rust version '{}' should contain a dot (e.g., '1.78.0')",
            version
        );

        // Verify cargo is also detectable by running it
        let cargo_output = std::process::Command::new("cargo")
            .arg("--version")
            .output();
        assert!(
            cargo_output.is_ok(),
            "cargo --version should succeed on the build host"
        );
        let output = cargo_output.unwrap();
        let cargo_stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            cargo_stdout.contains("cargo"),
            "cargo --version output should contain 'cargo', got: {}",
            cargo_stdout
        );
    }

    #[test]
    fn test_toolchain_detect_python() {
        // Python3 may or may not be installed. The detection function
        // should return Some(version) or None — never panic.
        let python_version = crate::registry::helpers::get_python_version();
        // Either it finds python3 or it gracefully returns None
        if let Some(version) = python_version {
            assert!(
                !version.is_empty(),
                "Python version string should be non-empty when detected"
            );
            // Python versions look like "3.11.2"
            assert!(
                version.contains('.'),
                "Python version '{}' should contain a dot",
                version
            );
        }
        // If None, that's fine — python3 is not required for horus builds
    }

    #[test]
    fn test_toolchain_version_parsing() {
        // Test parsing version strings like "1.78.0", "3.11.2"
        // using the semver crate (already a dependency of horus_manager).
        use semver::Version;

        // Standard three-part version
        let v1 = Version::parse("1.78.0").expect("should parse 1.78.0");
        assert_eq!(v1.major, 1);
        assert_eq!(v1.minor, 78);
        assert_eq!(v1.patch, 0);

        // Python-style version
        let v2 = Version::parse("3.11.2").expect("should parse 3.11.2");
        assert_eq!(v2.major, 3);
        assert_eq!(v2.minor, 11);
        assert_eq!(v2.patch, 2);

        // Pre-release version
        let v3 = Version::parse("1.80.0-nightly").expect("should parse 1.80.0-nightly");
        assert_eq!(v3.major, 1);
        assert_eq!(v3.minor, 80);
        assert_eq!(v3.patch, 0);
        assert!(!v3.pre.is_empty(), "pre-release should be non-empty");

        // Invalid version strings should fail
        assert!(
            Version::parse("not-a-version").is_err(),
            "garbage string should not parse as semver"
        );
        assert!(
            Version::parse("1.2").is_err(),
            "two-part version should not parse as strict semver"
        );
    }
}
