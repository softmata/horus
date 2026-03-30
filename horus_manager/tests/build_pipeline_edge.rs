#![allow(dead_code)]
//! Edge-case tests for the build pipeline generators (cargo_gen, pyproject_gen, cmake_gen).
//!
//! These tests exercise the pure-function file generators that transform a
//! `HorusManifest` into native build files (Cargo.toml, pyproject.toml,
//! CMakeLists.txt). No running system or network access required.

use horus_manager::manifest::{
    CppConfig, DepSource, DependencyValue, DetailedDependency, HorusManifest, IgnoreConfig,
    PackageInfo, TargetType,
};
use std::collections::BTreeMap;
use std::path::PathBuf;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Build a minimal `HorusManifest` with the given name, version, and deps.
fn make_manifest(
    name: &str,
    version: &str,
    deps: BTreeMap<String, DependencyValue>,
) -> HorusManifest {
    HorusManifest {
        package: PackageInfo {
            name: name.to_string(),
            version: version.to_string(),
            description: None,
            authors: vec![],
            license: None,
            edition: "1".to_string(),
            rust_edition: None,
            repository: None,
            package_type: None,
            categories: vec![],
            standard: None,
            target_type: TargetType::default(),
        },
        workspace: None,
        robot: None,
        dependencies: deps,
        dev_dependencies: BTreeMap::new(),
        sim_dependencies: BTreeMap::new(),
        hardware: BTreeMap::new(),
        drivers: BTreeMap::new(),
        sim_drivers: BTreeMap::new(),
        scripts: BTreeMap::new(),
        ignore: IgnoreConfig::default(),
        enable: vec![],
        cpp: None,
        hooks: Default::default(),
        network: None,
    }
}

/// Create a crates.io `DependencyValue` with a version and optional features.
fn crates_dep(version: &str, features: Vec<&str>) -> DependencyValue {
    DependencyValue::Detailed(DetailedDependency {
        version: Some(version.to_string()),
        source: Some(DepSource::CratesIo),
        features: features.into_iter().map(|s| s.to_string()).collect(),
        optional: false,
        path: None,
        git: None,
        branch: None,
        tag: None,
        rev: None,
        apt: None,
        cmake_package: None,
        lang: None,
        workspace: false,
    })
}

/// Create a path `DependencyValue`.
fn path_dep(path: &str) -> DependencyValue {
    DependencyValue::Detailed(DetailedDependency {
        version: None,
        source: Some(DepSource::Path),
        features: vec![],
        optional: false,
        path: Some(path.to_string()),
        git: None,
        branch: None,
        tag: None,
        rev: None,
        apt: None,
        cmake_package: None,
        lang: None,
        workspace: false,
    })
}

/// Create a git `DependencyValue` with optional tag.
fn git_dep(url: &str, tag: Option<&str>) -> DependencyValue {
    DependencyValue::Detailed(DetailedDependency {
        version: None,
        source: Some(DepSource::Git),
        features: vec![],
        optional: false,
        path: None,
        git: Some(url.to_string()),
        branch: None,
        tag: tag.map(|s| s.to_string()),
        rev: None,
        apt: None,
        cmake_package: None,
        lang: None,
        workspace: false,
    })
}

/// Create a PyPI `DependencyValue`.
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
        workspace: false,
    })
}

/// Create a C++ system `DependencyValue` with cmake_package override.
fn cpp_system_dep(cmake_pkg: &str) -> DependencyValue {
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
        cmake_package: Some(cmake_pkg.to_string()),
        lang: Some("cpp".to_string()),
        workspace: false,
    })
}

// ─── cargo_gen tests ─────────────────────────────────────────────────────────

#[test]
fn test_cargo_gen_minimal_manifest() {
    let dir = tempfile::tempdir().unwrap();

    // Create a src/main.rs so the [[bin]] section is generated
    std::fs::create_dir_all(dir.path().join("src")).unwrap();
    std::fs::write(dir.path().join("src/main.rs"), "fn main() {}\n").unwrap();

    let mut deps = BTreeMap::new();
    deps.insert("serde".to_string(), crates_dep("1.0", vec![]));

    let manifest = make_manifest("test-project", "0.1.0", deps);
    let source_files: Vec<PathBuf> = vec![];
    let (path, content) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &source_files, false).unwrap();

    // File was written to disk
    assert!(path.exists(), "Cargo.toml should exist at {:?}", path);
    assert!(
        path.ends_with(".horus/Cargo.toml"),
        "should be inside .horus/"
    );

    // [package] section with correct name and version
    assert!(
        content.contains("[package]"),
        "should contain [package] section"
    );
    assert!(
        content.contains("name = \"test-project\""),
        "should contain project name"
    );
    assert!(
        content.contains("version = \"0.1.0\""),
        "should contain version"
    );
    assert!(
        content.contains("edition = \"2021\""),
        "should contain edition"
    );

    // [dependencies] section with serde
    assert!(
        content.contains("[dependencies]"),
        "should contain [dependencies] section"
    );
    assert!(
        content.contains("serde = \"1.0\""),
        "should contain serde dep, got:\n{}",
        content
    );
}

#[test]
fn test_cargo_gen_path_dependency() {
    let dir = tempfile::tempdir().unwrap();

    // Create the path dep target directory so the path is valid
    let dep_dir = dir.path().join("libs/my-lib");
    std::fs::create_dir_all(&dep_dir).unwrap();

    let mut deps = BTreeMap::new();
    deps.insert("my-lib".to_string(), path_dep("libs/my-lib"));

    let manifest = make_manifest("path-project", "0.2.0", deps);
    let (_, content) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();

    // Should contain path = "..." syntax for the path dep
    assert!(
        content.contains("[dependencies]"),
        "should contain [dependencies]"
    );
    assert!(
        content.contains("my-lib = { path ="),
        "should contain path dependency syntax, got:\n{}",
        content
    );
    // The path should reference something relative (going up from .horus/)
    assert!(
        content.contains("path = \""),
        "path dep should use path = \"...\" syntax"
    );
}

#[test]
fn test_cargo_gen_git_dependency() {
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    deps.insert(
        "my-crate".to_string(),
        git_dep("https://github.com/org/my-crate.git", Some("v1.2.3")),
    );

    let manifest = make_manifest("git-project", "0.3.0", deps);
    let (_, content) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();

    // Should contain git = "..." with the URL
    assert!(
        content.contains("git = \"https://github.com/org/my-crate.git\""),
        "should contain git URL, got:\n{}",
        content
    );
    // Should contain tag = "v1.2.3"
    assert!(
        content.contains("tag = \"v1.2.3\""),
        "should contain git tag, got:\n{}",
        content
    );
    // The whole line should be in Cargo.toml inline table format
    assert!(
        content.contains("my-crate = {"),
        "should use inline table syntax for git dep"
    );
}

#[test]
fn test_cargo_gen_no_deps_produces_valid_toml() {
    let dir = tempfile::tempdir().unwrap();

    let manifest = make_manifest("empty-project", "1.0.0", BTreeMap::new());
    let (_, content) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();

    // Should still have [package] section
    assert!(
        content.contains("[package]"),
        "should contain [package] even with no deps"
    );
    assert!(
        content.contains("name = \"empty-project\""),
        "should contain project name"
    );

    // [dependencies] header should still be present (may have horus path deps)
    assert!(
        content.contains("[dependencies]"),
        "should contain [dependencies] even when empty"
    );

    // The generated header should be present
    assert!(
        content.contains("# Generated by horus"),
        "should start with generated header"
    );

    // Should be parseable as TOML (the [dependencies] section may contain
    // comments for registry deps that aren't installed, which is valid TOML)
    let parsed: Result<toml::Value, _> = toml::from_str(&content);
    assert!(
        parsed.is_ok(),
        "generated Cargo.toml should be valid TOML, got error: {:?}\ncontent:\n{}",
        parsed.err(),
        content
    );
}

// ─── pyproject_gen tests ─────────────────────────────────────────────────────

#[test]
fn test_pyproject_gen_minimal() {
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    deps.insert("numpy".to_string(), pypi_dep(">=1.24"));

    let manifest = make_manifest("py-project", "0.1.0", deps);
    let (path, content) =
        horus_manager::pyproject_gen::generate(&manifest, dir.path(), false).unwrap();

    // File written to disk
    assert!(path.exists(), "pyproject.toml should exist at {:?}", path);
    assert!(
        path.ends_with(".horus/pyproject.toml"),
        "should be inside .horus/"
    );

    // [project] section with correct metadata
    assert!(
        content.contains("[project]"),
        "should contain [project] section"
    );
    assert!(
        content.contains("name = \"py-project\""),
        "should contain project name, got:\n{}",
        content
    );
    assert!(
        content.contains("version = \"0.1.0\""),
        "should contain version"
    );

    // PEP 621 dependencies list with numpy
    assert!(
        content.contains("dependencies = ["),
        "should contain dependencies array"
    );
    assert!(
        content.contains("numpy"),
        "should contain numpy dependency, got:\n{}",
        content
    );
    // Version specifier should be present (PEP 508 format)
    assert!(
        content.contains(">=1.24"),
        "should contain version specifier for numpy"
    );

    // Build system section
    assert!(
        content.contains("[build-system]"),
        "should contain [build-system]"
    );
    assert!(
        content.contains("setuptools"),
        "should reference setuptools"
    );
}

// ─── cmake_gen tests ─────────────────────────────────────────────────────────

#[test]
fn test_cmake_gen_minimal() {
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    deps.insert("my-math-lib".to_string(), cpp_system_dep("MyMathLib"));

    let manifest = make_manifest("cpp-project", "0.1.0", deps);
    let (path, content) = horus_manager::cmake_gen::generate(&manifest, dir.path(), false).unwrap();

    // File written to disk
    assert!(path.exists(), "CMakeLists.txt should exist at {:?}", path);
    assert!(
        path.ends_with(".horus/CMakeLists.txt"),
        "should be inside .horus/"
    );

    // cmake header
    assert!(
        content.contains("cmake_minimum_required(VERSION 3.20)"),
        "should contain cmake_minimum_required"
    );
    // Project declaration with sanitized name (hyphens to underscores)
    assert!(
        content.contains("project(cpp_project VERSION 0.1.0 LANGUAGES CXX)"),
        "should contain project() with sanitized name, got:\n{}",
        content
    );

    // find_package for the system dep
    assert!(
        content.contains("find_package(MyMathLib REQUIRED)"),
        "should contain find_package for the system dep, got:\n{}",
        content
    );

    // C++ standard defaults to 17
    assert!(
        content.contains("set(CMAKE_CXX_STANDARD 17)"),
        "should default to C++17"
    );
}

// ─── Cross-generator consistency tests ───────────────────────────────────────

#[test]
fn test_generators_filter_deps_by_ecosystem() {
    // A manifest with one dep from each ecosystem. Each generator should
    // only include deps relevant to its ecosystem.
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    deps.insert("serde".to_string(), crates_dep("1.0", vec!["derive"]));
    deps.insert("numpy".to_string(), pypi_dep(">=1.24"));
    deps.insert("eigen".to_string(), cpp_system_dep("Eigen3"));

    let manifest = make_manifest("multi-lang", "0.1.0", deps);

    // cargo_gen: should include serde, skip numpy and eigen
    let (_, cargo_content) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();
    assert!(
        cargo_content.contains("serde"),
        "cargo_gen should include serde"
    );
    assert!(
        !cargo_content.contains("numpy"),
        "cargo_gen should NOT include numpy"
    );
    // eigen is a system dep, cargo_gen should skip it
    assert!(
        !cargo_content.contains("find_package"),
        "cargo_gen should not produce cmake syntax"
    );

    // pyproject_gen: should include numpy, skip serde and eigen
    let (_, py_content) =
        horus_manager::pyproject_gen::generate(&manifest, dir.path(), false).unwrap();
    assert!(
        py_content.contains("numpy"),
        "pyproject_gen should include numpy"
    );
    assert!(
        !py_content.contains("serde"),
        "pyproject_gen should NOT include serde"
    );
    // eigen is system, not pypi
    assert!(
        !py_content.contains("Eigen3"),
        "pyproject_gen should NOT include C++ deps"
    );

    // cmake_gen: should include eigen, skip serde and numpy
    let (_, cmake_content) =
        horus_manager::cmake_gen::generate(&manifest, dir.path(), false).unwrap();
    assert!(
        cmake_content.contains("find_package(Eigen3 REQUIRED)"),
        "cmake_gen should include eigen"
    );
    assert!(
        !cmake_content.contains("serde"),
        "cmake_gen should NOT include serde"
    );
    assert!(
        !cmake_content.contains("numpy"),
        "cmake_gen should NOT include numpy"
    );
}

#[test]
fn test_cargo_gen_git_dep_with_branch_not_tag() {
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    deps.insert(
        "branch-crate".to_string(),
        DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: Some(DepSource::Git),
            features: vec![],
            optional: false,
            path: None,
            git: Some("https://github.com/org/branch-crate.git".to_string()),
            branch: Some("develop".to_string()),
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
            workspace: false,
        }),
    );

    let manifest = make_manifest("branch-project", "0.1.0", deps);
    let (_, content) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();

    assert!(
        content.contains("git = \"https://github.com/org/branch-crate.git\""),
        "should contain git URL"
    );
    assert!(
        content.contains("branch = \"develop\""),
        "should contain branch, got:\n{}",
        content
    );
    assert!(
        !content.contains("tag ="),
        "should NOT contain tag when branch is used"
    );
}

#[test]
fn test_cargo_gen_crates_dep_with_features() {
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    deps.insert(
        "serde".to_string(),
        crates_dep("1.0", vec!["derive", "alloc"]),
    );

    let manifest = make_manifest("feat-project", "0.1.0", deps);
    let (_, content) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();

    assert!(
        content.contains("features = ["),
        "should contain features array, got:\n{}",
        content
    );
    assert!(
        content.contains("\"derive\""),
        "should contain derive feature"
    );
    assert!(
        content.contains("\"alloc\""),
        "should contain alloc feature"
    );
}

#[test]
fn test_pyproject_gen_no_deps_still_valid() {
    let dir = tempfile::tempdir().unwrap();

    let manifest = make_manifest("bare-py", "0.1.0", BTreeMap::new());
    let (_, content) =
        horus_manager::pyproject_gen::generate(&manifest, dir.path(), false).unwrap();

    // Should have empty dependencies array
    assert!(
        content.contains("dependencies = []"),
        "should have empty dependencies list, got:\n{}",
        content
    );
    // Should still have [project] and [build-system]
    assert!(content.contains("[project]"), "should have [project]");
    assert!(
        content.contains("[build-system]"),
        "should have [build-system]"
    );
}

#[test]
fn test_cmake_gen_known_dep_resolves() {
    // "eigen" is in the known deps table and should resolve to Eigen3
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    // Use a known dep by name with source=system + lang=cpp (no cmake_package override)
    deps.insert(
        "eigen".to_string(),
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
            cmake_package: None, // should resolve via known deps table
            lang: Some("cpp".to_string()),
            workspace: false,
        }),
    );

    let manifest = make_manifest("eigen-proj", "0.1.0", deps);
    let (_, content) = horus_manager::cmake_gen::generate(&manifest, dir.path(), false).unwrap();

    assert!(
        content.contains("find_package(Eigen3 REQUIRED)"),
        "known dep 'eigen' should resolve to Eigen3, got:\n{}",
        content
    );
    assert!(
        content.contains("Eigen3::Eigen"),
        "should use Eigen3::Eigen as link target"
    );
}

#[test]
fn test_cmake_gen_cpp_standard_override() {
    let dir = tempfile::tempdir().unwrap();

    let mut manifest = make_manifest("std20-proj", "0.1.0", BTreeMap::new());
    manifest.package.standard = Some("c++20".to_string());

    let (_, content) = horus_manager::cmake_gen::generate(&manifest, dir.path(), false).unwrap();

    assert!(
        content.contains("set(CMAKE_CXX_STANDARD 20)"),
        "should set C++20 standard, got:\n{}",
        content
    );
}

#[test]
fn test_cmake_gen_compiler_override() {
    let dir = tempfile::tempdir().unwrap();

    let mut manifest = make_manifest("clang-proj", "0.1.0", BTreeMap::new());
    manifest.cpp = Some(CppConfig {
        compiler: Some("clang++".to_string()),
        cmake_args: vec![],
        toolchain: None,
    });

    let (_, content) = horus_manager::cmake_gen::generate(&manifest, dir.path(), false).unwrap();

    assert!(
        content.contains("set(CMAKE_CXX_COMPILER clang++)"),
        "should set clang++ compiler, got:\n{}",
        content
    );
}

#[test]
fn test_generators_are_idempotent() {
    let dir = tempfile::tempdir().unwrap();

    let mut deps = BTreeMap::new();
    deps.insert("serde".to_string(), crates_dep("1.0", vec![]));
    deps.insert("numpy".to_string(), pypi_dep(">=1.24"));
    deps.insert("eigen".to_string(), cpp_system_dep("Eigen3"));

    let manifest = make_manifest("idempotent-proj", "0.1.0", deps);

    // Run each generator twice, content should be identical
    let (_, cargo1) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();
    let (_, cargo2) =
        horus_manager::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();
    assert_eq!(cargo1, cargo2, "cargo_gen should be idempotent");

    let (_, py1) = horus_manager::pyproject_gen::generate(&manifest, dir.path(), false).unwrap();
    let (_, py2) = horus_manager::pyproject_gen::generate(&manifest, dir.path(), false).unwrap();
    assert_eq!(py1, py2, "pyproject_gen should be idempotent");

    let (_, cmake1) = horus_manager::cmake_gen::generate(&manifest, dir.path(), false).unwrap();
    let (_, cmake2) = horus_manager::cmake_gen::generate(&manifest, dir.path(), false).unwrap();
    assert_eq!(cmake1, cmake2, "cmake_gen should be idempotent");
}

#[test]
fn test_all_generators_create_horus_dir() {
    // Each generator should create .horus/ if it doesn't exist
    let dir1 = tempfile::tempdir().unwrap();
    let dir2 = tempfile::tempdir().unwrap();
    let dir3 = tempfile::tempdir().unwrap();

    let manifest = make_manifest("dir-test", "0.1.0", BTreeMap::new());

    assert!(!dir1.path().join(".horus").exists());
    horus_manager::cargo_gen::generate(&manifest, dir1.path(), &[], false).unwrap();
    assert!(
        dir1.path().join(".horus").is_dir(),
        "cargo_gen should create .horus/"
    );

    assert!(!dir2.path().join(".horus").exists());
    horus_manager::pyproject_gen::generate(&manifest, dir2.path(), false).unwrap();
    assert!(
        dir2.path().join(".horus").is_dir(),
        "pyproject_gen should create .horus/"
    );

    assert!(!dir3.path().join(".horus").exists());
    horus_manager::cmake_gen::generate(&manifest, dir3.path(), false).unwrap();
    assert!(
        dir3.path().join(".horus").is_dir(),
        "cmake_gen should create .horus/"
    );
}
