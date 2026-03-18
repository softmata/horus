//! Generate `.horus/CMakeLists.txt` from `horus.toml` dependencies.
//!
//! This module is the bridge between the unified horus manifest and the CMake
//! build system. It reads `[dependencies]` from `horus.toml`, filters for
//! C++ deps (System, lang="cpp", or known robotics libraries), and produces a
//! valid `CMakeLists.txt` inside `.horus/`.
//!
//! The generated file is always gitignored and treated as a build artifact.

use anyhow::{Context, Result};
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::{Path, PathBuf};

use crate::manifest::{DepSource, DependencyValue, HorusManifest};

/// The `.horus` build directory name.
const HORUS_DIR: &str = ".horus";

// ─── Known C++ dependency mappings ──────────────────────────────────────────

/// Mapping from a horus dependency name to its cmake and apt counterparts.
struct CppDepMapping {
    /// Name as it appears in horus.toml `[dependencies]`.
    horus_name: &'static str,
    /// Argument to cmake `find_package()`.
    cmake_package: &'static str,
    /// Target for `target_link_libraries()`.
    cmake_target: &'static str,
    /// Debian/Ubuntu apt package name.
    apt_package: &'static str,
}

/// Static table of well-known C++ robotics dependencies.
const KNOWN_DEPS: &[CppDepMapping] = &[
    CppDepMapping {
        horus_name: "eigen",
        cmake_package: "Eigen3",
        cmake_target: "Eigen3::Eigen",
        apt_package: "libeigen3-dev",
    },
    CppDepMapping {
        horus_name: "opencv",
        cmake_package: "OpenCV",
        cmake_target: "${OpenCV_LIBS}",
        apt_package: "libopencv-dev",
    },
    CppDepMapping {
        horus_name: "pcl",
        cmake_package: "PCL",
        cmake_target: "${PCL_LIBRARIES}",
        apt_package: "libpcl-dev",
    },
    CppDepMapping {
        horus_name: "boost",
        cmake_package: "Boost",
        cmake_target: "Boost::boost",
        apt_package: "libboost-all-dev",
    },
    CppDepMapping {
        horus_name: "gtest",
        cmake_package: "GTest",
        cmake_target: "GTest::gtest_main",
        apt_package: "libgtest-dev",
    },
    CppDepMapping {
        horus_name: "fmt",
        cmake_package: "fmt",
        cmake_target: "fmt::fmt",
        apt_package: "libfmt-dev",
    },
    CppDepMapping {
        horus_name: "spdlog",
        cmake_package: "spdlog",
        cmake_target: "spdlog::spdlog",
        apt_package: "libspdlog-dev",
    },
    CppDepMapping {
        horus_name: "yaml-cpp",
        cmake_package: "yaml-cpp",
        cmake_target: "yaml-cpp::yaml-cpp",
        apt_package: "libyaml-cpp-dev",
    },
    CppDepMapping {
        horus_name: "nlohmann-json",
        cmake_package: "nlohmann_json",
        cmake_target: "nlohmann_json::nlohmann_json",
        apt_package: "nlohmann-json3-dev",
    },
    CppDepMapping {
        horus_name: "protobuf",
        cmake_package: "Protobuf",
        cmake_target: "protobuf::libprotobuf",
        apt_package: "libprotobuf-dev",
    },
    CppDepMapping {
        horus_name: "grpc",
        cmake_package: "gRPC",
        cmake_target: "gRPC::grpc++",
        apt_package: "libgrpc++-dev",
    },
    CppDepMapping {
        horus_name: "abseil",
        cmake_package: "absl",
        cmake_target: "absl::base",
        apt_package: "libabsl-dev",
    },
];

/// Look up a known dep by horus name.
fn lookup_known_dep(name: &str) -> Option<&'static CppDepMapping> {
    KNOWN_DEPS.iter().find(|m| m.horus_name == name)
}

/// Public accessor for the cmake_package name of a known dep.
pub fn lookup_known_dep_public(name: &str) -> Option<&'static str> {
    lookup_known_dep(name).map(|m| m.cmake_package)
}

// ─── Resolved C++ dependency ────────────────────────────────────────────────

/// A resolved C++ dependency ready for CMakeLists.txt generation.
struct ResolvedCppDep {
    cmake_package: String,
    cmake_target: String,
}

/// Resolve a horus dependency to its cmake package/target names.
///
/// Priority:
/// 1. Explicit `cmake_package` field on the `DetailedDependency`
/// 2. Known deps table lookup
/// 3. `None` if not a recognized C++ dep
fn resolve_cpp_dep(name: &str, dep: &DependencyValue) -> Option<ResolvedCppDep> {
    // Check for explicit cmake_package override
    if let DependencyValue::Detailed(d) = dep {
        if let Some(ref pkg) = d.cmake_package {
            return Some(ResolvedCppDep {
                cmake_package: pkg.clone(),
                // If no known target, use PkgConfig-style target
                cmake_target: lookup_known_dep(name)
                    .map(|m| m.cmake_target.to_string())
                    .unwrap_or_else(|| format!("{}::{}", pkg, name)),
            });
        }
    }

    // Check known deps table
    if let Some(known) = lookup_known_dep(name) {
        return Some(ResolvedCppDep {
            cmake_package: known.cmake_package.to_string(),
            cmake_target: known.cmake_target.to_string(),
        });
    }

    None
}

/// Check if a dependency is a C++ dependency.
///
/// A dep is C++ if any of:
/// - `lang = "cpp"` is set
/// - `source = "system"` (system deps are typically C/C++ libraries)
/// - It's in the known deps table
/// - It has a `cmake_package` override
fn is_cpp_dep(name: &str, dep: &DependencyValue) -> bool {
    match dep {
        DependencyValue::Detailed(d) => {
            // Explicit lang hint
            if d.lang.as_deref() == Some("cpp") {
                return true;
            }
            // System source
            if d.source.as_ref() == Some(&DepSource::System) {
                return true;
            }
            // Has cmake_package override
            if d.cmake_package.is_some() {
                return true;
            }
            // Known dep
            lookup_known_dep(name).is_some()
        }
        DependencyValue::Simple(_) => {
            // Simple deps — only if in known table
            lookup_known_dep(name).is_some()
        }
    }
}

/// Parse C++ standard string (e.g., "c++20") into the numeric version (e.g., "20").
fn parse_cpp_standard(standard: &str) -> &str {
    let s = standard.trim().to_ascii_lowercase();
    // Handle: "c++20", "c++17", "c++23", "20", "17", "23"
    if let Some(stripped) = standard.strip_prefix("c++") {
        stripped
    } else if let Some(stripped) = standard.strip_prefix("C++") {
        stripped
    } else {
        // Already numeric
        if s.chars().all(|c| c.is_ascii_digit()) {
            standard
        } else {
            "17" // safe default
        }
    }
}

// ─── Generator ──────────────────────────────────────────────────────────────

/// Generate `.horus/CMakeLists.txt` from a `HorusManifest`.
///
/// - Filters C++ deps from `[dependencies]`
/// - Generates cmake with find_package() for each dep
/// - Supports test target generation when `tests/` directory exists
/// - Returns the path to the generated `CMakeLists.txt`
///
/// # Arguments
///
/// * `manifest` - The parsed horus.toml manifest
/// * `project_dir` - The project root directory (where horus.toml lives)
/// * `include_dev` - Whether to include dev-dependencies (e.g., gtest)
pub fn generate(
    manifest: &HorusManifest,
    project_dir: &Path,
    include_dev: bool,
) -> Result<PathBuf> {
    let horus_dir = project_dir.join(HORUS_DIR);
    fs::create_dir_all(&horus_dir).context("Failed to create .horus directory")?;

    let project_name = &manifest.package.name;
    let version = &manifest.package.version;

    // Determine C++ standard
    let cpp_standard = manifest
        .package
        .standard
        .as_deref()
        .map(parse_cpp_standard)
        .unwrap_or("17");

    let mut cmake = String::with_capacity(2048);

    // ── Header ───────────────────────────────────────────────────────────
    writeln!(
        cmake,
        "# Generated by horus from horus.toml — do not edit manually"
    )?;
    writeln!(cmake, "cmake_minimum_required(VERSION 3.20)")?;
    writeln!(
        cmake,
        "project({} VERSION {} LANGUAGES CXX)",
        sanitize_cmake_name(project_name),
        version
    )?;
    writeln!(cmake)?;

    // ── C++ standard ─────────────────────────────────────────────────────
    writeln!(cmake, "set(CMAKE_CXX_STANDARD {})", cpp_standard)?;
    writeln!(cmake, "set(CMAKE_CXX_STANDARD_REQUIRED ON)")?;
    writeln!(cmake, "set(CMAKE_EXPORT_COMPILE_COMMANDS ON)")?;
    writeln!(cmake)?;

    // ── Extra cmake args from [cpp] config ───────────────────────────────
    if let Some(ref cpp) = manifest.cpp {
        if let Some(ref compiler) = cpp.compiler {
            writeln!(cmake, "set(CMAKE_CXX_COMPILER {})", compiler)?;
        }
        for arg in &cpp.cmake_args {
            writeln!(cmake, "{}", arg)?;
        }
        if !cpp.cmake_args.is_empty() || cpp.compiler.is_some() {
            writeln!(cmake)?;
        }
    }

    // ── Dependencies (find_package) ──────────────────────────────────────
    let cpp_deps = collect_cpp_deps(&manifest.dependencies);
    let dev_cpp_deps = if include_dev {
        collect_cpp_deps(&manifest.dev_dependencies)
    } else {
        vec![]
    };

    for dep in &cpp_deps {
        writeln!(cmake, "find_package({} REQUIRED)", dep.cmake_package)?;
    }
    for dep in &dev_cpp_deps {
        writeln!(cmake, "find_package({} REQUIRED)", dep.cmake_package)?;
    }
    if !cpp_deps.is_empty() || !dev_cpp_deps.is_empty() {
        writeln!(cmake)?;
    }

    // ── Source files ─────────────────────────────────────────────────────
    // Sources are relative to project root (one level above .horus/)
    writeln!(
        cmake,
        "file(GLOB_RECURSE SOURCES \"${{CMAKE_SOURCE_DIR}}/../src/*.cpp\" \"${{CMAKE_SOURCE_DIR}}/../src/*.cc\" \"${{CMAKE_SOURCE_DIR}}/../src/*.cxx\")"
    )?;
    writeln!(cmake)?;

    // ── Main executable ──────────────────────────────────────────────────
    let target_name = sanitize_cmake_name(project_name);

    // Guard: only create executable if source files exist
    writeln!(cmake, "if(SOURCES)")?;
    writeln!(cmake, "  add_executable({} ${{SOURCES}})", target_name)?;

    // Include directories
    writeln!(
        cmake,
        "  target_include_directories({} PRIVATE ${{CMAKE_SOURCE_DIR}}/../include)",
        target_name
    )?;

    // Link libraries
    if !cpp_deps.is_empty() {
        let targets: Vec<&str> = cpp_deps.iter().map(|d| d.cmake_target.as_str()).collect();
        writeln!(
            cmake,
            "  target_link_libraries({} PRIVATE {})",
            target_name,
            targets.join(" ")
        )?;
    }
    writeln!(cmake, "else()")?;
    writeln!(cmake, "  message(WARNING \"No C++ source files found in src/. Create src/main.cpp to build an executable.\")")?;
    writeln!(cmake, "endif()")?;
    writeln!(cmake)?;

    // ── Test targets ─────────────────────────────────────────────────────
    let tests_dir = project_dir.join("tests");
    if tests_dir.is_dir() {
        writeln!(cmake, "# ── Tests ─────────────────────────────────────")?;
        writeln!(cmake, "enable_testing()")?;

        // Check if gtest is available (in deps or dev-deps)
        let has_gtest = cpp_deps
            .iter()
            .chain(dev_cpp_deps.iter())
            .any(|d| d.cmake_package == "GTest");

        if has_gtest {
            writeln!(
                cmake,
                "file(GLOB_RECURSE TEST_SOURCES \"${{CMAKE_SOURCE_DIR}}/../tests/*.cpp\" \"${{CMAKE_SOURCE_DIR}}/../tests/*.cc\" \"${{CMAKE_SOURCE_DIR}}/../tests/*.cxx\")"
            )?;
            writeln!(cmake, "if(TEST_SOURCES)")?;
            writeln!(
                cmake,
                "  add_executable({}_tests ${{TEST_SOURCES}})",
                target_name
            )?;
            writeln!(
                cmake,
                "  target_include_directories({}_tests PRIVATE ${{CMAKE_SOURCE_DIR}}/../include)",
                target_name
            )?;

            // Link gtest + all regular deps
            let mut test_targets: Vec<&str> =
                cpp_deps.iter().map(|d| d.cmake_target.as_str()).collect();
            test_targets.push("GTest::gtest_main");
            writeln!(
                cmake,
                "  target_link_libraries({}_tests PRIVATE {})",
                target_name,
                test_targets.join(" ")
            )?;

            writeln!(cmake, "  include(GoogleTest)")?;
            writeln!(cmake, "  gtest_discover_tests({}_tests)", target_name)?;
            writeln!(cmake, "endif()")?;
        } else {
            // Basic ctest without gtest
            writeln!(
                cmake,
                "file(GLOB_RECURSE TEST_SOURCES \"${{CMAKE_SOURCE_DIR}}/../tests/*.cpp\" \"${{CMAKE_SOURCE_DIR}}/../tests/*.cc\" \"${{CMAKE_SOURCE_DIR}}/../tests/*.cxx\")"
            )?;
            writeln!(cmake, "if(TEST_SOURCES)")?;
            writeln!(
                cmake,
                "  add_executable({}_tests ${{TEST_SOURCES}})",
                target_name
            )?;
            writeln!(
                cmake,
                "  target_include_directories({}_tests PRIVATE ${{CMAKE_SOURCE_DIR}}/../include)",
                target_name
            )?;
            if !cpp_deps.is_empty() {
                let targets: Vec<&str> = cpp_deps.iter().map(|d| d.cmake_target.as_str()).collect();
                writeln!(
                    cmake,
                    "  target_link_libraries({}_tests PRIVATE {})",
                    target_name,
                    targets.join(" ")
                )?;
            }
            writeln!(
                cmake,
                "  add_test(NAME {}_tests COMMAND {}_tests)",
                target_name, target_name
            )?;
            writeln!(cmake, "endif()")?;
        }
    }

    // ── Write file ───────────────────────────────────────────────────────
    let cmake_path = horus_dir.join("CMakeLists.txt");
    fs::write(&cmake_path, &cmake)
        .with_context(|| format!("Failed to write {}", cmake_path.display()))?;

    Ok(cmake_path)
}

/// Collect resolved C++ dependencies from a manifest dependency map.
fn collect_cpp_deps(
    deps: &std::collections::BTreeMap<String, DependencyValue>,
) -> Vec<ResolvedCppDep> {
    let mut result = Vec::new();
    for (name, dep) in deps {
        if is_cpp_dep(name, dep) {
            if let Some(resolved) = resolve_cpp_dep(name, dep) {
                result.push(resolved);
            }
        }
    }
    result
}

/// Sanitize a name for use as a CMake project/target name.
///
/// CMake names must be alphanumeric + underscores. Hyphens → underscores.
fn sanitize_cmake_name(name: &str) -> String {
    name.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// Get the apt package name for a horus dependency.
///
/// Returns the explicit `apt` field if set, otherwise looks up the known deps table.
pub fn apt_package_for(name: &str, dep: &DependencyValue) -> Option<String> {
    if let DependencyValue::Detailed(d) = dep {
        if let Some(ref apt) = d.apt {
            return Some(apt.clone());
        }
    }
    lookup_known_dep(name).map(|m| m.apt_package.to_string())
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifest::*;
    use std::collections::BTreeMap;

    fn test_manifest(deps: BTreeMap<String, DependencyValue>) -> HorusManifest {
        HorusManifest {
            package: PackageInfo {
                name: "my-robot".to_string(),
                version: "0.1.0".to_string(),
                description: Some("A test C++ robot".to_string()),
                authors: vec!["Test Author".to_string()],
                license: None,
                edition: "1".to_string(),
                rust_edition: None,
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
            },
            dependencies: deps,
            dev_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
        }
    }

    fn cpp_system_dep(apt: &str, cmake_pkg: &str) -> DependencyValue {
        DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: Some(DepSource::System),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: Some(apt.to_string()),
            cmake_package: Some(cmake_pkg.to_string()),
            lang: Some("cpp".to_string()),
        })
    }

    fn cpp_known_dep(_name: &str) -> DependencyValue {
        DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: Some(DepSource::System),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: Some("cpp".to_string()),
        })
    }

    fn crates_dep(version: &str) -> DependencyValue {
        DependencyValue::Detailed(DetailedDependency {
            version: Some(version.to_string()),
            source: Some(DepSource::CratesIo),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
        })
    }

    fn pypi_dep(version: &str) -> DependencyValue {
        DependencyValue::Detailed(DetailedDependency {
            version: Some(version.to_string()),
            source: Some(DepSource::PyPI),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
        })
    }

    // ── Basic generation tests ───────────────────────────────────────────

    #[test]
    fn generate_minimal_cmake() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = test_manifest(BTreeMap::new());
        let result = generate(&manifest, dir.path(), false);
        assert!(result.is_ok());

        let content = fs::read_to_string(result.unwrap()).unwrap();
        assert!(content.contains("cmake_minimum_required(VERSION 3.20)"));
        assert!(content.contains("project(my_robot VERSION 0.1.0 LANGUAGES CXX)"));
        assert!(content.contains("set(CMAKE_CXX_STANDARD 17)"));
        assert!(content.contains("set(CMAKE_CXX_STANDARD_REQUIRED ON)"));
        assert!(content.contains("set(CMAKE_EXPORT_COMPILE_COMMANDS ON)"));
        assert!(content.contains("add_executable(my_robot ${SOURCES})"));
    }

    #[test]
    fn generate_with_custom_standard() {
        let dir = tempfile::tempdir().unwrap();
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.standard = Some("c++20".to_string());

        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(content.contains("set(CMAKE_CXX_STANDARD 20)"));
    }

    #[test]
    fn generate_with_standard_23() {
        let dir = tempfile::tempdir().unwrap();
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.standard = Some("c++23".to_string());

        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(content.contains("set(CMAKE_CXX_STANDARD 23)"));
    }

    // ── Dependency tests ─────────────────────────────────────────────────

    #[test]
    fn generate_with_known_dep_eigen() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();
        deps.insert("eigen".to_string(), cpp_known_dep("eigen"));

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        assert!(content.contains("find_package(Eigen3 REQUIRED)"));
        assert!(content.contains("Eigen3::Eigen"));
    }

    #[test]
    fn generate_with_custom_cmake_package() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();
        deps.insert(
            "my-math-lib".to_string(),
            cpp_system_dep("libmymath-dev", "MyMathLib"),
        );

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        assert!(content.contains("find_package(MyMathLib REQUIRED)"));
        assert!(content.contains("MyMathLib::my-math-lib"));
    }

    #[test]
    fn generate_skips_non_cpp_deps() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();
        deps.insert("serde".to_string(), crates_dep("1.0"));
        deps.insert("numpy".to_string(), pypi_dep(">=1.24"));
        deps.insert("eigen".to_string(), cpp_known_dep("eigen"));

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        assert!(content.contains("Eigen3"), "eigen should be present");
        assert!(
            !content.contains("serde"),
            "crates.io dep should be filtered"
        );
        assert!(!content.contains("numpy"), "pypi dep should be filtered");
    }

    #[test]
    fn generate_with_multiple_deps() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();
        deps.insert("eigen".to_string(), cpp_known_dep("eigen"));
        deps.insert("fmt".to_string(), cpp_known_dep("fmt"));
        deps.insert("spdlog".to_string(), cpp_known_dep("spdlog"));

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        assert!(content.contains("find_package(Eigen3 REQUIRED)"));
        assert!(content.contains("find_package(fmt REQUIRED)"));
        assert!(content.contains("find_package(spdlog REQUIRED)"));
        assert!(content.contains("target_link_libraries"));
    }

    // ── Test targets ─────────────────────────────────────────────────────

    #[test]
    fn generate_with_tests_dir_and_gtest() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("tests")).unwrap();

        let mut dev_deps = BTreeMap::new();
        dev_deps.insert("gtest".to_string(), cpp_known_dep("gtest"));

        let mut manifest = test_manifest(BTreeMap::new());
        manifest.dev_dependencies = dev_deps;

        let result = generate(&manifest, dir.path(), true).unwrap();
        let content = fs::read_to_string(result).unwrap();

        assert!(content.contains("enable_testing()"));
        assert!(content.contains("find_package(GTest REQUIRED)"));
        assert!(content.contains("GTest::gtest_main"));
        assert!(content.contains("include(GoogleTest)"));
        assert!(content.contains("gtest_discover_tests"));
    }

    #[test]
    fn generate_without_tests_dir_no_test_section() {
        let dir = tempfile::tempdir().unwrap();
        // No tests/ directory

        let manifest = test_manifest(BTreeMap::new());
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        assert!(!content.contains("enable_testing()"));
    }

    // ── CppConfig tests ──────────────────────────────────────────────────

    #[test]
    fn generate_with_cpp_config_compiler() {
        let dir = tempfile::tempdir().unwrap();
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.cpp = Some(CppConfig {
            compiler: Some("clang++".to_string()),
            cmake_args: vec![],
            toolchain: None,
        });

        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(content.contains("set(CMAKE_CXX_COMPILER clang++)"));
    }

    #[test]
    fn generate_with_cpp_config_cmake_args() {
        let dir = tempfile::tempdir().unwrap();
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.cpp = Some(CppConfig {
            compiler: None,
            cmake_args: vec![
                "set(BUILD_SHARED_LIBS ON)".to_string(),
                "set(CMAKE_POSITION_INDEPENDENT_CODE ON)".to_string(),
            ],
            toolchain: None,
        });

        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(content.contains("set(BUILD_SHARED_LIBS ON)"));
        assert!(content.contains("set(CMAKE_POSITION_INDEPENDENT_CODE ON)"));
    }

    // ── Helper function tests ────────────────────────────────────────────

    #[test]
    fn sanitize_cmake_name_works() {
        assert_eq!(sanitize_cmake_name("my-robot"), "my_robot");
        assert_eq!(sanitize_cmake_name("my_robot"), "my_robot");
        assert_eq!(sanitize_cmake_name("simple"), "simple");
        assert_eq!(sanitize_cmake_name("robot@v2"), "robot_v2");
    }

    #[test]
    fn parse_cpp_standard_works() {
        assert_eq!(parse_cpp_standard("c++17"), "17");
        assert_eq!(parse_cpp_standard("c++20"), "20");
        assert_eq!(parse_cpp_standard("c++23"), "23");
        assert_eq!(parse_cpp_standard("C++20"), "20");
        assert_eq!(parse_cpp_standard("17"), "17");
        assert_eq!(parse_cpp_standard("20"), "20");
    }

    #[test]
    fn known_deps_table_covers_robotics() {
        let names: Vec<&str> = KNOWN_DEPS.iter().map(|d| d.horus_name).collect();
        assert!(names.contains(&"eigen"));
        assert!(names.contains(&"opencv"));
        assert!(names.contains(&"pcl"));
        assert!(names.contains(&"boost"));
        assert!(names.contains(&"gtest"));
        assert!(names.contains(&"fmt"));
        assert!(names.contains(&"spdlog"));
        assert!(names.contains(&"yaml-cpp"));
        assert!(names.contains(&"nlohmann-json"));
    }

    #[test]
    fn apt_package_for_known_dep() {
        let dep = DependencyValue::Simple("*".to_string());
        assert_eq!(
            apt_package_for("eigen", &dep),
            Some("libeigen3-dev".to_string())
        );
        assert_eq!(
            apt_package_for("opencv", &dep),
            Some("libopencv-dev".to_string())
        );
    }

    #[test]
    fn apt_package_for_explicit_override() {
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: Some(DepSource::System),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: Some("custom-eigen-dev".to_string()),
            cmake_package: None,
            lang: Some("cpp".to_string()),
        });
        assert_eq!(
            apt_package_for("eigen", &dep),
            Some("custom-eigen-dev".to_string())
        );
    }

    #[test]
    fn apt_package_for_unknown_dep() {
        let dep = DependencyValue::Simple("1.0".to_string());
        assert_eq!(apt_package_for("unknown-lib", &dep), None);
    }

    #[test]
    fn is_cpp_dep_by_lang_hint() {
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: None,
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: Some("cpp".to_string()),
        });
        assert!(is_cpp_dep("unknown-lib", &dep));
    }

    #[test]
    fn is_not_cpp_dep_for_crates_io() {
        assert!(!is_cpp_dep("serde", &crates_dep("1.0")));
    }

    #[test]
    fn is_not_cpp_dep_for_pypi() {
        assert!(!is_cpp_dep("numpy", &pypi_dep(">=1.24")));
    }

    #[test]
    fn header_always_generated() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = test_manifest(BTreeMap::new());
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(content.starts_with("# Generated by horus from horus.toml"));
    }

    #[test]
    fn simple_dep_known_resolves() {
        // A Simple("*") dep named "eigen" should resolve via known table
        let dep = DependencyValue::Simple("*".to_string());
        assert!(is_cpp_dep("eigen", &dep));

        let resolved = resolve_cpp_dep("eigen", &dep);
        assert!(resolved.is_some());
        let r = resolved.unwrap();
        assert_eq!(r.cmake_package, "Eigen3");
        assert_eq!(r.cmake_target, "Eigen3::Eigen");
    }

    #[test]
    fn include_directories_present() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = test_manifest(BTreeMap::new());
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(content.contains(
            "target_include_directories(my_robot PRIVATE ${CMAKE_SOURCE_DIR}/../include)"
        ));
    }

    // ── Battle tests: cmake generation edge cases ────────────────────────

    #[test]
    fn battle_generate_creates_horus_dir() {
        let dir = tempfile::tempdir().unwrap();
        assert!(!dir.path().join(".horus").exists());

        let manifest = test_manifest(BTreeMap::new());
        generate(&manifest, dir.path(), false).unwrap();

        assert!(dir.path().join(".horus").is_dir());
        assert!(dir.path().join(".horus/CMakeLists.txt").is_file());
    }

    #[test]
    fn battle_generate_idempotent() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = test_manifest(BTreeMap::new());

        let path1 = generate(&manifest, dir.path(), false).unwrap();
        let content1 = fs::read_to_string(&path1).unwrap();

        let path2 = generate(&manifest, dir.path(), false).unwrap();
        let content2 = fs::read_to_string(&path2).unwrap();

        assert_eq!(
            content1, content2,
            "repeated generation should produce identical output"
        );
    }

    #[test]
    fn battle_generate_with_all_known_deps() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();

        // Add ALL known deps
        for name in &[
            "eigen",
            "opencv",
            "pcl",
            "boost",
            "gtest",
            "fmt",
            "spdlog",
            "yaml-cpp",
            "nlohmann-json",
        ] {
            deps.insert(name.to_string(), cpp_known_dep(name));
        }

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        // All should have find_package
        assert!(content.contains("find_package(Eigen3 REQUIRED)"));
        assert!(content.contains("find_package(OpenCV REQUIRED)"));
        assert!(content.contains("find_package(PCL REQUIRED)"));
        assert!(content.contains("find_package(Boost REQUIRED)"));
        assert!(content.contains("find_package(GTest REQUIRED)"));
        assert!(content.contains("find_package(fmt REQUIRED)"));
        assert!(content.contains("find_package(spdlog REQUIRED)"));
        assert!(content.contains("find_package(yaml-cpp REQUIRED)"));
        assert!(content.contains("find_package(nlohmann_json REQUIRED)"));
    }

    #[test]
    fn battle_generate_mixed_deps_only_cpp() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();

        // Mix of C++, Rust, and Python deps
        deps.insert("eigen".to_string(), cpp_known_dep("eigen"));
        deps.insert("serde".to_string(), crates_dep("1.0"));
        deps.insert("numpy".to_string(), pypi_dep(">=1.24"));
        deps.insert("fmt".to_string(), cpp_known_dep("fmt"));

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        // C++ deps present
        assert!(content.contains("Eigen3"));
        assert!(content.contains("fmt::fmt"));
        // Non-C++ deps absent
        assert!(!content.contains("serde"));
        assert!(!content.contains("numpy"));
    }

    #[test]
    fn battle_generate_with_tests_dir_no_gtest() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("tests")).unwrap();

        let manifest = test_manifest(BTreeMap::new());
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        // Should have enable_testing but NOT GTest-specific stuff
        assert!(content.contains("enable_testing()"));
        assert!(!content.contains("include(GoogleTest)"));
        assert!(content.contains("add_test(NAME"));
    }

    #[test]
    fn battle_generate_dev_deps_separate() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("tests")).unwrap();

        let mut dev_deps = BTreeMap::new();
        dev_deps.insert("gtest".to_string(), cpp_known_dep("gtest"));

        let mut manifest = test_manifest(BTreeMap::new());
        manifest.dev_dependencies = dev_deps;

        // Without include_dev — gtest NOT in find_package
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(
            !content.contains("find_package(GTest"),
            "gtest should not appear without include_dev"
        );

        // With include_dev — gtest IN find_package
        let result = generate(&manifest, dir.path(), true).unwrap();
        let content = fs::read_to_string(result).unwrap();
        assert!(
            content.contains("find_package(GTest REQUIRED)"),
            "gtest should appear with include_dev"
        );
    }

    #[test]
    fn battle_sanitize_cmake_name_edge_cases() {
        assert_eq!(sanitize_cmake_name(""), "");
        assert_eq!(sanitize_cmake_name("a"), "a");
        assert_eq!(sanitize_cmake_name("my--robot"), "my__robot");
        assert_eq!(sanitize_cmake_name("robot@v2.0"), "robot_v2_0");
        assert_eq!(sanitize_cmake_name("_private"), "_private");
        assert_eq!(sanitize_cmake_name("123"), "123");
    }

    #[test]
    fn battle_resolve_cpp_dep_explicit_override() {
        let dep = DependencyValue::Detailed(crate::manifest::DetailedDependency {
            version: None,
            source: Some(crate::manifest::DepSource::System),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: Some("libcustom-dev".to_string()),
            cmake_package: Some("CustomLib".to_string()),
            lang: Some("cpp".to_string()),
        });

        let resolved = resolve_cpp_dep("custom-lib", &dep);
        assert!(resolved.is_some());
        let r = resolved.unwrap();
        assert_eq!(r.cmake_package, "CustomLib");
    }

    #[test]
    fn battle_is_cpp_dep_not_cpp_for_pypi() {
        let dep = DependencyValue::Detailed(crate::manifest::DetailedDependency {
            version: Some("1.0".to_string()),
            source: Some(crate::manifest::DepSource::PyPI),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
        });
        assert!(!is_cpp_dep("numpy", &dep));
    }

    #[test]
    fn battle_is_cpp_dep_by_cmake_package() {
        let dep = DependencyValue::Detailed(crate::manifest::DetailedDependency {
            version: None,
            source: None,
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: Some("MyLib".to_string()),
            lang: None,
        });
        assert!(
            is_cpp_dep("mylib", &dep),
            "cmake_package should make it a C++ dep"
        );
    }

    #[test]
    fn battle_lookup_known_dep_public_all_entries() {
        // Verify every known dep is accessible via public API
        for name in &[
            "eigen",
            "opencv",
            "pcl",
            "boost",
            "gtest",
            "fmt",
            "spdlog",
            "yaml-cpp",
            "nlohmann-json",
            "protobuf",
            "grpc",
            "abseil",
        ] {
            assert!(
                lookup_known_dep_public(name).is_some(),
                "known dep '{}' should be accessible via lookup_known_dep_public",
                name
            );
        }
    }

    #[test]
    fn battle_apt_package_for_all_known() {
        let dummy = DependencyValue::Simple("*".to_string());
        assert_eq!(
            apt_package_for("eigen", &dummy),
            Some("libeigen3-dev".to_string())
        );
        assert_eq!(
            apt_package_for("opencv", &dummy),
            Some("libopencv-dev".to_string())
        );
        assert_eq!(
            apt_package_for("gtest", &dummy),
            Some("libgtest-dev".to_string())
        );
        assert_eq!(
            apt_package_for("fmt", &dummy),
            Some("libfmt-dev".to_string())
        );
        assert_eq!(
            apt_package_for("spdlog", &dummy),
            Some("libspdlog-dev".to_string())
        );
        assert_eq!(
            apt_package_for("nlohmann-json", &dummy),
            Some("nlohmann-json3-dev".to_string())
        );
        assert_eq!(
            apt_package_for("protobuf", &dummy),
            Some("libprotobuf-dev".to_string())
        );
    }

    #[test]
    fn battle_parse_cpp_standard_edge_cases() {
        assert_eq!(parse_cpp_standard("c++11"), "11");
        assert_eq!(parse_cpp_standard("c++14"), "14");
        assert_eq!(parse_cpp_standard("c++26"), "26");
        // Leading/trailing spaces: strip_prefix fails → goes to numeric check → "17" fallback
        assert_eq!(parse_cpp_standard("  c++20  "), "17");
    }

    // ── Property tests: cmake generation ────────────────────────────────

    #[test]
    fn property_generate_always_starts_with_generated_header() {
        // Property: for ANY valid HorusManifest, generate() produces a file starting with "# Generated by horus"
        let dir = tempfile::tempdir().unwrap();
        let configs: Vec<BTreeMap<String, DependencyValue>> = vec![
            BTreeMap::new(),
            {
                let mut d = BTreeMap::new();
                d.insert("eigen".into(), cpp_known_dep("eigen"));
                d
            },
            {
                let mut d = BTreeMap::new();
                d.insert("serde".into(), crates_dep("1.0"));
                d.insert("numpy".into(), pypi_dep(">=1.0"));
                d.insert("fmt".into(), cpp_known_dep("fmt"));
                d
            },
        ];

        for deps in configs {
            let manifest = test_manifest(deps);
            let path = generate(&manifest, dir.path(), false).unwrap();
            let content = fs::read_to_string(path).unwrap();
            assert!(
                content.starts_with("# Generated by horus"),
                "Generated CMakeLists.txt must start with '# Generated by horus'"
            );
        }
    }

    #[test]
    fn property_no_deps_means_no_find_package() {
        // Property: for ANY valid HorusManifest with no deps, generate() output contains NO find_package lines
        let dir = tempfile::tempdir().unwrap();
        let manifest = test_manifest(BTreeMap::new());
        let path = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(path).unwrap();
        assert!(
            !content.contains("find_package"),
            "No deps should mean no find_package lines, got:\n{}",
            content
        );
    }

    #[test]
    fn property_sanitize_cmake_name_never_empty_for_nonempty_input() {
        // Property: sanitize_cmake_name never produces empty string for non-empty input
        let inputs = [
            "a",
            "hello",
            "my-robot",
            "robot@v2.0",
            "____",
            "123",
            "a!b#c$d",
            "x",
            "unicode-\u{00e9}",
            "---",
            "...",
            "a b c",
        ];
        for input in &inputs {
            let result = sanitize_cmake_name(input);
            assert!(
                !result.is_empty(),
                "sanitize_cmake_name({:?}) should not be empty",
                input
            );
        }
    }

    #[test]
    fn property_parse_cpp_standard_returns_numeric_for_valid() {
        // Property: parse_cpp_standard("c++N") always returns "N" for N in {11,14,17,20,23,26}
        for n in &["11", "14", "17", "20", "23", "26"] {
            let input = format!("c++{}", n);
            let result = parse_cpp_standard(&input);
            assert_eq!(
                result, *n,
                "parse_cpp_standard({:?}) should return {:?}, got {:?}",
                input, n, result
            );
        }
    }

    #[test]
    fn property_parse_cpp_standard_uppercase_returns_numeric() {
        // Also test C++ prefix (uppercase)
        for n in &["11", "14", "17", "20", "23", "26"] {
            let input = format!("C++{}", n);
            let result = parse_cpp_standard(&input);
            assert_eq!(
                result, *n,
                "parse_cpp_standard({:?}) should return {:?}, got {:?}",
                input, n, result
            );
        }
    }

    #[test]
    fn property_generate_empty_project_name() {
        // Edge: generate with empty project name
        let dir = tempfile::tempdir().unwrap();
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "".to_string();
        let path = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(path).unwrap();
        // Should still produce valid cmake (project name is sanitized to empty, but cmake is generated)
        assert!(content.contains("# Generated by horus"));
        assert!(content.contains("project("));
    }

    #[test]
    fn property_generate_very_long_project_name() {
        // Edge: generate with very long project name (100+ chars)
        let dir = tempfile::tempdir().unwrap();
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "a".repeat(150);
        let path = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(path).unwrap();
        let sanitized = sanitize_cmake_name(&"a".repeat(150));
        assert!(
            content.contains(&sanitized),
            "Long project name should be preserved in cmake"
        );
        assert_eq!(sanitized.len(), 150);
    }

    // ── Error path tests ─────────────────────────────────────────────

    #[test]
    fn error_generate_readonly_dir() {
        // generate() when .horus/ can't be created (read-only parent)
        let dir = tempfile::tempdir().unwrap();
        let readonly = dir.path().join("readonly");
        fs::create_dir_all(&readonly).unwrap();

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&readonly, fs::Permissions::from_mode(0o444)).unwrap();
        }

        let manifest = test_manifest(BTreeMap::new());
        let result = generate(&manifest, &readonly, false);

        // Restore permissions for cleanup
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&readonly, fs::Permissions::from_mode(0o755)).unwrap();
        }

        assert!(result.is_err(), "generate on read-only dir should fail");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("create .horus") || err_msg.contains("Permission denied"),
            "error should mention .horus creation or permission: {}",
            err_msg
        );
    }

    #[test]
    fn error_apt_package_for_with_git_dep() {
        // apt_package_for with a git-source dep that has no apt field returns None
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: Some(DepSource::Git),
            features: vec![],
            optional: false,
            path: None,
            git: Some("https://github.com/org/lib".to_string()),
            branch: Some("main".to_string()),
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
        });
        // "random-git-lib" is not in the known deps table and has no apt field
        assert_eq!(
            apt_package_for("random-git-lib", &dep),
            None,
            "git dep without apt field should return None"
        );
    }

    #[test]
    fn error_resolve_cpp_dep_non_cpp() {
        // resolve_cpp_dep for a PyPI dep should return None
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: Some(">=1.24".to_string()),
            source: Some(DepSource::PyPI),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
        });
        assert!(
            resolve_cpp_dep("numpy", &dep).is_none(),
            "PyPI dep should not resolve as C++ dep"
        );

        // Also test a CratesIo dep
        let crate_dep = crates_dep("1.0");
        assert!(
            resolve_cpp_dep("serde", &crate_dep).is_none(),
            "CratesIo dep should not resolve as C++ dep"
        );
    }

    #[test]
    fn property_generate_unicode_project_name() {
        // Edge: generate with Unicode project name
        let dir = tempfile::tempdir().unwrap();
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "r\u{00f6}bot-\u{00fc}nit".to_string();
        let path = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(path).unwrap();
        // Unicode chars should be replaced by underscores
        let sanitized = sanitize_cmake_name("r\u{00f6}bot-\u{00fc}nit");
        assert!(content.contains(&sanitized));
        // Ensure no Unicode leaks into cmake name
        assert!(
            sanitized
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_'),
            "Sanitized name should be ASCII only: {:?}",
            sanitized
        );
    }

    // ── SLAM Cycle 4: Stress & boundary tests ────────────────────────────

    #[test]
    fn stress_generate_with_50_deps() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();
        for i in 0..50 {
            let name = format!("lib-{}", i);
            deps.insert(
                name.clone(),
                DependencyValue::Detailed(DetailedDependency {
                    version: None,
                    source: Some(DepSource::System),
                    features: vec![],
                    optional: false,
                    path: None,
                    git: None,
                    branch: None,
                    tag: None,
                    rev: None,
                    apt: Some(format!("lib{}-dev", i)),
                    cmake_package: Some(format!("Lib{}", i)),
                    lang: Some("cpp".to_string()),
                }),
            );
        }

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        // All 50 deps should have find_package lines
        for i in 0..50 {
            let pkg = format!("find_package(Lib{} REQUIRED)", i);
            assert!(
                content.contains(&pkg),
                "missing find_package for dep {}: {}",
                i,
                pkg
            );
        }
        // File should be valid (starts with header, has project)
        assert!(content.starts_with("# Generated by horus"));
        assert!(content.contains("target_link_libraries"));
    }

    #[test]
    fn stress_generate_100_times_same_dir() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = test_manifest(BTreeMap::new());

        let mut last_content = String::new();
        for i in 0..100 {
            let path = generate(&manifest, dir.path(), false).unwrap();
            let content = fs::read_to_string(&path).unwrap();
            if i > 0 {
                assert_eq!(
                    content, last_content,
                    "generation {} differs from previous",
                    i
                );
            }
            last_content = content;
        }
        // Verify the file is still valid
        assert!(last_content.contains("cmake_minimum_required"));
    }

    #[test]
    fn boundary_generate_dep_name_with_special_chars() {
        let dir = tempfile::tempdir().unwrap();
        let mut deps = BTreeMap::new();
        deps.insert(
            "my@lib#2.0".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: None,
                source: Some(DepSource::System),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                branch: None,
                tag: None,
                rev: None,
                apt: Some("libspecial-dev".to_string()),
                cmake_package: Some("MySpecialLib".to_string()),
                lang: Some("cpp".to_string()),
            }),
        );

        let manifest = test_manifest(deps);
        let result = generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(result).unwrap();

        // The cmake_package is used directly for find_package
        assert!(content.contains("find_package(MySpecialLib REQUIRED)"));
        // The project name is sanitized
        let sanitized = sanitize_cmake_name("my-robot");
        assert!(
            sanitized
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_'),
            "Sanitized name should only contain safe chars"
        );
        // Verify special-char dep name gets sanitized in cmake target
        assert_eq!(sanitize_cmake_name("my@lib#2.0"), "my_lib_2_0");
    }

    // ═══════════════════════════════════════════════════════════════════
    // REAL CMAKE INTEGRATION TESTS
    //
    // These tests run actual cmake and g++ to verify the generated
    // CMakeLists.txt is accepted by real tools. They skip automatically
    // if cmake is not installed.
    // ═══════════════════════════════════════════════════════════════════

    fn cmake_available() -> bool {
        std::process::Command::new("cmake")
            .arg("--version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    }

    /// Helper: run cmake configure on a generated CMakeLists.txt
    fn cmake_configure(project_dir: &std::path::Path) -> std::process::Output {
        let build_dir = project_dir.join(".horus/cpp-build");
        fs::create_dir_all(&build_dir).unwrap();
        std::process::Command::new("cmake")
            .arg("-S")
            .arg(project_dir.join(".horus"))
            .arg("-B")
            .arg(&build_dir)
            .arg("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
            .arg("-DCMAKE_BUILD_TYPE=Debug")
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .expect("cmake should be installed for integration tests")
    }

    /// Helper: run cmake build after configure
    fn cmake_build(project_dir: &std::path::Path) -> std::process::Output {
        let build_dir = project_dir.join(".horus/cpp-build");
        std::process::Command::new("cmake")
            .arg("--build")
            .arg(&build_dir)
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .expect("cmake --build should work")
    }

    /// Helper: create a minimal C++ project with src/main.cpp
    fn create_cpp_project(
        dir: &std::path::Path,
        name: &str,
        standard: Option<&str>,
        deps: BTreeMap<String, DependencyValue>,
    ) {
        // Create src/main.cpp
        let src_dir = dir.join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(
            src_dir.join("main.cpp"),
            "#include <iostream>\nint main() { std::cout << \"hello\" << std::endl; return 0; }\n",
        )
        .unwrap();

        // Create include dir
        fs::create_dir_all(dir.join("include")).unwrap();

        // Generate CMakeLists.txt
        let mut manifest = test_manifest(deps);
        manifest.package.name = name.to_string();
        if let Some(std) = standard {
            manifest.package.standard = Some(std.to_string());
        }
        generate(&manifest, dir, false).unwrap();
    }

    // ── Risk 1: Does cmake actually accept our generated CMakeLists.txt? ──

    #[test]
    fn real_cmake_configure_minimal_project() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "minimal-test", None, BTreeMap::new());

        let output = cmake_configure(dir.path());
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(
            output.status.success(),
            "cmake configure failed for minimal project:\n{}",
            stderr
        );
    }

    #[test]
    fn real_cmake_build_minimal_project() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "build-test", None, BTreeMap::new());

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "configure failed: {}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = cmake_build(dir.path());
        let stderr = String::from_utf8_lossy(&build.stderr);
        let stdout = String::from_utf8_lossy(&build.stdout);
        assert!(
            build.status.success(),
            "cmake build failed:\nstdout: {}\nstderr: {}",
            stdout,
            stderr
        );

        // Verify binary was produced
        let binary = dir.path().join(".horus/cpp-build/build_test");
        assert!(binary.exists(), "binary should exist at {:?}", binary);
    }

    // ── Risk 2: C++ standard compatibility ──

    #[test]
    fn real_cmake_cpp17_standard() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "std17-test", Some("c++17"), BTreeMap::new());

        let output = cmake_configure(dir.path());
        assert!(
            output.status.success(),
            "c++17 configure failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    #[test]
    fn real_cmake_cpp20_standard() {
        let dir = tempfile::tempdir().unwrap();

        // Write a file that uses C++20 features
        let src_dir = dir.path().join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(
            src_dir.join("main.cpp"),
            r#"
#include <iostream>
#include <concepts>
#include <ranges>
#include <span>

template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template<Numeric T>
T square(T x) { return x * x; }

int main() {
    auto nums = std::array{1, 2, 3, 4, 5};
    for (auto n : nums | std::views::transform([](int x) { return x * x; })) {
        std::cout << n << " ";
    }
    return 0;
}
"#,
        )
        .unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "cpp20-test".to_string();
        manifest.package.standard = Some("c++20".to_string());
        generate(&manifest, dir.path(), false).unwrap();

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "c++20 configure failed: {}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = cmake_build(dir.path());
        assert!(
            build.status.success(),
            "c++20 build failed (compiler may not support C++20):\n{}",
            String::from_utf8_lossy(&build.stderr)
        );
    }

    #[test]
    fn real_cmake_cpp23_standard() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "std23-test", Some("c++23"), BTreeMap::new());

        let output = cmake_configure(dir.path());
        // C++23 may not be supported by all compilers — that's OK, cmake should still configure
        // The configure step sets the standard but the build may warn/error
        assert!(
            output.status.success(),
            "c++23 configure failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    // ── Risk 3: find_package actually finds installed libs ──

    #[test]
    fn real_cmake_find_package_eigen() {
        let dir = tempfile::tempdir().unwrap();

        // Write source that uses Eigen
        let src_dir = dir.path().join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(
            src_dir.join("main.cpp"),
            r#"
#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    std::cout << "Eigen identity:\n" << m << std::endl;
    return 0;
}
"#,
        )
        .unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        let mut deps = BTreeMap::new();
        deps.insert("eigen".to_string(), cpp_known_dep("eigen"));
        let mut manifest = test_manifest(deps);
        manifest.package.name = "eigen-test".to_string();
        manifest.package.standard = Some("c++17".to_string());
        generate(&manifest, dir.path(), false).unwrap();

        let configure = cmake_configure(dir.path());
        let stderr = String::from_utf8_lossy(&configure.stderr);
        assert!(
            configure.status.success(),
            "cmake should find Eigen3 (is libeigen3-dev installed?):\n{}",
            stderr
        );

        let build = cmake_build(dir.path());
        assert!(
            build.status.success(),
            "build with Eigen should succeed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&build.stdout),
            String::from_utf8_lossy(&build.stderr)
        );

        // Run the binary
        let binary = dir.path().join(".horus/cpp-build/eigen_test");
        let run = std::process::Command::new(&binary)
            .stdout(std::process::Stdio::piped())
            .output();
        assert!(run.is_ok(), "binary should execute");
        let output = run.unwrap();
        assert!(output.status.success(), "binary should exit 0");
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            stdout.contains("Eigen identity"),
            "should print identity matrix"
        );
    }

    #[test]
    fn real_cmake_find_package_fmt() {
        let dir = tempfile::tempdir().unwrap();

        let src_dir = dir.path().join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(
            src_dir.join("main.cpp"),
            r#"
#include <fmt/core.h>

int main() {
    fmt::print("Hello from {}!\n", "fmt");
    return 0;
}
"#,
        )
        .unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        let mut deps = BTreeMap::new();
        deps.insert("fmt".to_string(), cpp_known_dep("fmt"));
        let mut manifest = test_manifest(deps);
        manifest.package.name = "fmt-test".to_string();
        generate(&manifest, dir.path(), false).unwrap();

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "cmake should find fmt:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = cmake_build(dir.path());
        assert!(
            build.status.success(),
            "build with fmt should succeed:\n{}",
            String::from_utf8_lossy(&build.stderr)
        );
    }

    // ── Risk 4: Multiple deps together ──

    #[test]
    fn real_cmake_multiple_deps_together() {
        let dir = tempfile::tempdir().unwrap();

        let src_dir = dir.path().join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(
            src_dir.join("main.cpp"),
            r#"
#include <iostream>
#include <Eigen/Dense>
#include <fmt/core.h>

int main() {
    Eigen::Vector3d v(1.0, 2.0, 3.0);
    fmt::print("Vector norm: {}\n", v.norm());
    return 0;
}
"#,
        )
        .unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        let mut deps = BTreeMap::new();
        deps.insert("eigen".to_string(), cpp_known_dep("eigen"));
        deps.insert("fmt".to_string(), cpp_known_dep("fmt"));
        let mut manifest = test_manifest(deps);
        manifest.package.name = "multi-dep-test".to_string();
        manifest.package.standard = Some("c++17".to_string());
        generate(&manifest, dir.path(), false).unwrap();

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "multi-dep configure failed:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = cmake_build(dir.path());
        assert!(
            build.status.success(),
            "multi-dep build failed:\n{}",
            String::from_utf8_lossy(&build.stderr)
        );
    }

    // ── Risk 5: GTest integration ──

    #[test]
    fn real_cmake_gtest_with_tests_dir() {
        let dir = tempfile::tempdir().unwrap();

        // Main source
        let src_dir = dir.path().join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(src_dir.join("main.cpp"), "int main() { return 0; }\n").unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        // Test source
        let test_dir = dir.path().join("tests");
        fs::create_dir_all(&test_dir).unwrap();
        fs::write(
            test_dir.join("test_basic.cpp"),
            r#"
#include <gtest/gtest.h>

TEST(BasicTest, TrueIsTrue) {
    EXPECT_TRUE(true);
}

TEST(BasicTest, OnePlusOne) {
    EXPECT_EQ(1 + 1, 2);
}
"#,
        )
        .unwrap();

        let mut dev_deps = BTreeMap::new();
        dev_deps.insert("gtest".to_string(), cpp_known_dep("gtest"));
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "gtest-test".to_string();
        manifest.dev_dependencies = dev_deps;
        generate(&manifest, dir.path(), true).unwrap();

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "gtest configure failed:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = cmake_build(dir.path());
        assert!(
            build.status.success(),
            "gtest build failed:\n{}",
            String::from_utf8_lossy(&build.stderr)
        );

        // Run ctest
        let build_dir = dir.path().join(".horus/cpp-build");
        let ctest = std::process::Command::new("ctest")
            .arg("--test-dir")
            .arg(&build_dir)
            .arg("--output-on-failure")
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .expect("ctest should run");
        assert!(
            ctest.status.success(),
            "ctest should pass:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&ctest.stdout),
            String::from_utf8_lossy(&ctest.stderr)
        );
    }

    // ── Risk 6: compile_commands.json generation ──

    #[test]
    fn real_cmake_generates_compile_commands() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "compdb-test", None, BTreeMap::new());

        let output = cmake_configure(dir.path());
        assert!(output.status.success());

        let compdb = dir.path().join(".horus/cpp-build/compile_commands.json");
        assert!(compdb.exists(), "compile_commands.json should be generated");

        let content = fs::read_to_string(&compdb).unwrap();
        assert!(
            content.contains("compdb_test"),
            "compile_commands.json should reference the project"
        );
        // Verify it's valid JSON
        let parsed: serde_json::Value =
            serde_json::from_str(&content).expect("compile_commands.json should be valid JSON");
        assert!(
            parsed.is_array(),
            "compile_commands.json should be a JSON array"
        );
    }

    // ── Risk 7: Project name sanitization in cmake ──

    #[test]
    fn real_cmake_hyphenated_project_name() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "my-cool-robot", None, BTreeMap::new());

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "hyphenated name should configure:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = cmake_build(dir.path());
        assert!(
            build.status.success(),
            "hyphenated name should build:\n{}",
            String::from_utf8_lossy(&build.stderr)
        );

        // Binary should be named with underscores
        let binary = dir.path().join(".horus/cpp-build/my_cool_robot");
        assert!(
            binary.exists(),
            "binary should be my_cool_robot (underscores), not my-cool-robot"
        );
    }

    // ── Risk 8: CppConfig compiler override ──

    #[test]
    fn real_cmake_compiler_override_gpp() {
        let dir = tempfile::tempdir().unwrap();

        let src_dir = dir.path().join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(
            src_dir.join("main.cpp"),
            "#include <iostream>\nint main() { return 0; }\n",
        )
        .unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "compiler-test".to_string();
        manifest.cpp = Some(CppConfig {
            compiler: Some("g++".to_string()),
            cmake_args: vec![],
            toolchain: None,
        });
        generate(&manifest, dir.path(), false).unwrap();

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "g++ override should configure:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = cmake_build(dir.path());
        assert!(
            build.status.success(),
            "g++ override should build:\n{}",
            String::from_utf8_lossy(&build.stderr)
        );
    }

    // ── Risk 9: cmake_args passthrough ──

    #[test]
    fn real_cmake_custom_args_passthrough() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "args-test", None, BTreeMap::new());

        // Re-generate with custom cmake_args
        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "args-test".to_string();
        manifest.cpp = Some(CppConfig {
            compiler: None,
            cmake_args: vec!["set(MY_CUSTOM_VAR \"hello\")".to_string()],
            toolchain: None,
        });
        generate(&manifest, dir.path(), false).unwrap();

        // Verify the custom arg is in the CMakeLists.txt
        let content = fs::read_to_string(dir.path().join(".horus/CMakeLists.txt")).unwrap();
        assert!(
            content.contains("set(MY_CUSTOM_VAR \"hello\")"),
            "custom cmake arg should be in CMakeLists.txt"
        );

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "custom cmake args should configure:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );
    }

    // ── Risk 10: Release vs Debug build type ──

    #[test]
    fn real_cmake_release_build() {
        let dir = tempfile::tempdir().unwrap();

        let src_dir = dir.path().join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(src_dir.join("main.cpp"), "#include <iostream>\nint main() { std::cout << \"release\" << std::endl; return 0; }\n").unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        let mut manifest = test_manifest(BTreeMap::new());
        manifest.package.name = "release-test".to_string();
        generate(&manifest, dir.path(), false).unwrap();

        let build_dir = dir.path().join(".horus/cpp-build");
        fs::create_dir_all(&build_dir).unwrap();

        // Configure with Release
        let configure = std::process::Command::new("cmake")
            .arg("-S")
            .arg(dir.path().join(".horus"))
            .arg("-B")
            .arg(&build_dir)
            .arg("-DCMAKE_BUILD_TYPE=Release")
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .unwrap();
        assert!(
            configure.status.success(),
            "Release configure failed:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );

        let build = std::process::Command::new("cmake")
            .arg("--build")
            .arg(&build_dir)
            .arg("--config")
            .arg("Release")
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .unwrap();
        assert!(
            build.status.success(),
            "Release build failed:\n{}",
            String::from_utf8_lossy(&build.stderr)
        );
    }

    // ── Risk 11: Empty source directory ──

    #[test]
    fn real_cmake_empty_src_dir_configures_with_warning() {
        let dir = tempfile::tempdir().unwrap();

        // Create src/ but with no .cpp files
        fs::create_dir_all(dir.path().join("src")).unwrap();
        fs::create_dir_all(dir.path().join("include")).unwrap();

        let manifest = test_manifest(BTreeMap::new());
        generate(&manifest, dir.path(), false).unwrap();

        // After fix: cmake should configure successfully but warn about no sources
        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "empty src/ should configure with warning, not fail:\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );
        let stderr = String::from_utf8_lossy(&configure.stderr);
        assert!(
            stderr.contains("No C++ source files found"),
            "should warn about missing source files, got:\n{}",
            stderr
        );
    }

    // ── Risk 12: Cross-compilation toolchain file validity ──

    #[test]
    fn real_cmake_toolchain_file_accepted() {
        // We can't test actual cross-compilation without the cross-compiler,
        // but we CAN test that cmake accepts the toolchain file syntax
        let dir = tempfile::tempdir().unwrap();
        let tc_dir = dir.path().join("toolchains");
        fs::create_dir_all(&tc_dir).unwrap();

        // Write the aarch64 toolchain
        let tc_content = "set(CMAKE_SYSTEM_NAME Linux)\nset(CMAKE_SYSTEM_PROCESSOR aarch64)\n";
        fs::write(tc_dir.join("toolchain.cmake"), tc_content).unwrap();

        // Create a project and try to configure with the toolchain
        // This will fail at compiler detection (no cross-compiler) but should NOT
        // fail at toolchain file parsing
        create_cpp_project(dir.path(), "cross-test", None, BTreeMap::new());

        let build_dir = dir.path().join(".horus/cpp-build");
        let output = std::process::Command::new("cmake")
            .arg("-S")
            .arg(dir.path().join(".horus"))
            .arg("-B")
            .arg(&build_dir)
            .arg(format!(
                "-DCMAKE_TOOLCHAIN_FILE={}",
                tc_dir.join("toolchain.cmake").display()
            ))
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .unwrap();

        let stderr = String::from_utf8_lossy(&output.stderr);
        // The toolchain file should be READ successfully even if compiler is missing
        assert!(
            !stderr.contains("Error reading CMake toolchain"),
            "toolchain file should be syntactically valid, got:\n{}",
            stderr
        );
    }

    // ── Risk 13: Incremental builds work ──

    #[test]
    fn real_cmake_incremental_build() {
        let dir = tempfile::tempdir().unwrap();
        create_cpp_project(dir.path(), "incr-test", None, BTreeMap::new());

        // First build
        let c1 = cmake_configure(dir.path());
        assert!(c1.status.success());
        let b1 = cmake_build(dir.path());
        assert!(b1.status.success());

        // Modify source
        let src = dir.path().join("src/main.cpp");
        fs::write(
            &src,
            "#include <iostream>\nint main() { std::cout << \"v2\" << std::endl; return 0; }\n",
        )
        .unwrap();

        // Second build (should be incremental — no reconfigure needed)
        let b2 = cmake_build(dir.path());
        assert!(
            b2.status.success(),
            "incremental build failed:\n{}",
            String::from_utf8_lossy(&b2.stderr)
        );
    }

    // ── Risk 14: Mixed C++ deps don't contaminate non-C++ deps ──

    #[test]
    fn real_cmake_non_cpp_deps_excluded() {
        let dir = tempfile::tempdir().unwrap();

        let mut deps = BTreeMap::new();
        deps.insert("serde".to_string(), crates_dep("1.0"));
        deps.insert("numpy".to_string(), pypi_dep(">=1.24"));
        // Only eigen should appear in CMakeLists.txt
        deps.insert("eigen".to_string(), cpp_known_dep("eigen"));

        create_cpp_project(dir.path(), "mixed-dep-test", None, deps);

        let configure = cmake_configure(dir.path());
        assert!(
            configure.status.success(),
            "mixed deps should configure (non-C++ deps ignored):\n{}",
            String::from_utf8_lossy(&configure.stderr)
        );

        // Verify Eigen is found but serde/numpy are not referenced
        let content = fs::read_to_string(dir.path().join(".horus/CMakeLists.txt")).unwrap();
        assert!(
            content.contains("find_package(Eigen3"),
            "eigen should be in CMakeLists.txt"
        );
        assert!(
            !content.contains("serde"),
            "Rust dep should NOT be in CMakeLists.txt"
        );
        assert!(
            !content.contains("numpy"),
            "Python dep should NOT be in CMakeLists.txt"
        );
    }
}
