use super::helpers::*;
use super::*;
use std::fs;
use std::path::Path;
use tempfile::TempDir;

// ============================================================================
// url_encode_package_name / package_name_to_path tests
// ============================================================================

#[test]
fn test_url_encode_simple_name() {
    assert_eq!(url_encode_package_name("my-package"), "my-package");
}

#[test]
fn test_url_encode_scoped_name() {
    assert_eq!(url_encode_package_name("@org/pkg"), "@org/pkg");
}

#[test]
fn test_package_name_to_path_simple() {
    assert_eq!(package_name_to_path("lidar-driver"), "lidar-driver");
}

// ============================================================================
// ManifestFormat Display tests
// ============================================================================

#[test]
fn test_manifest_format_display_horus_yaml() {
    assert_eq!(format!("{}", ManifestFormat::HorusYaml), "horus.yaml");
}

#[test]
fn test_manifest_format_display_cargo_toml() {
    assert_eq!(format!("{}", ManifestFormat::CargoToml), "Cargo.toml");
}

#[test]
fn test_manifest_format_display_package_json() {
    assert_eq!(format!("{}", ManifestFormat::PackageJson), "package.json");
}

// ============================================================================
// PackageSource serde roundtrip tests
// ============================================================================

#[test]
fn test_package_source_serde_registry() {
    let source = PackageSource::Registry;
    let json = serde_json::to_string(&source).unwrap();
    let deserialized: PackageSource = serde_json::from_str(&json).unwrap();
    assert_eq!(deserialized, PackageSource::Registry);
}

#[test]
fn test_package_source_serde_pypi() {
    let source = PackageSource::PyPI;
    let json = serde_json::to_string(&source).unwrap();
    let deserialized: PackageSource = serde_json::from_str(&json).unwrap();
    assert_eq!(deserialized, PackageSource::PyPI);
}

#[test]
fn test_package_source_serde_path() {
    let source = PackageSource::Path {
        path: "/home/user/my-pkg".to_string(),
    };
    let json = serde_json::to_string(&source).unwrap();
    let deserialized: PackageSource = serde_json::from_str(&json).unwrap();
    assert_eq!(
        deserialized,
        PackageSource::Path {
            path: "/home/user/my-pkg".to_string()
        }
    );
}

#[test]
fn test_package_source_serde_all_variants() {
    let variants = vec![
        PackageSource::Registry,
        PackageSource::PyPI,
        PackageSource::CratesIO,
        PackageSource::System,
        PackageSource::Path {
            path: "./local".to_string(),
        },
    ];
    for source in variants {
        let json = serde_json::to_string(&source).unwrap();
        let roundtripped: PackageSource = serde_json::from_str(&json).unwrap();
        assert_eq!(roundtripped, source);
    }
}

// ============================================================================
// DriverMetadata serde tests
// ============================================================================

#[test]
fn test_driver_metadata_default() {
    let meta = DriverMetadata::default();
    assert!(meta.bus_type.is_none());
    assert!(meta.required_features.is_none());
    assert!(meta.cargo_dependencies.is_none());
    assert!(meta.python_dependencies.is_none());
    assert!(meta.system_dependencies.is_none());
}

#[test]
fn test_driver_metadata_serde_roundtrip() {
    let meta = DriverMetadata {
        bus_type: Some("serial".to_string()),
        driver_category: Some("sensor".to_string()),
        required_features: Some(vec!["serial-hardware".to_string()]),
        cargo_dependencies: Some(vec!["serialport@4.2".to_string()]),
        python_dependencies: Some(vec!["pyserial>=3.5".to_string()]),
        system_dependencies: Some(vec!["libudev-dev".to_string()]),
    };
    let json = serde_json::to_string(&meta).unwrap();
    let roundtripped: DriverMetadata = serde_json::from_str(&json).unwrap();
    assert_eq!(roundtripped.bus_type, Some("serial".to_string()));
    assert_eq!(
        roundtripped.required_features,
        Some(vec!["serial-hardware".to_string()])
    );
}

#[test]
fn test_driver_metadata_skip_none_fields() {
    let meta = DriverMetadata {
        bus_type: Some("i2c".to_string()),
        ..Default::default()
    };
    let json = serde_json::to_string(&meta).unwrap();
    // None fields should be skipped
    assert!(!json.contains("required_features"));
    assert!(!json.contains("cargo_dependencies"));
    assert!(json.contains("bus_type"));
}

// ============================================================================
// check_global_versions tests
// ============================================================================

#[test]
fn test_check_global_versions_nonexistent_dir() {
    let result = check_global_versions(Path::new("/nonexistent/cache/dir"), "my-package");
    assert!(result.is_ok());
    assert!(!result.unwrap());
}

#[test]
fn test_check_global_versions_empty_dir() {
    let tmp = TempDir::new().unwrap();
    let result = check_global_versions(tmp.path(), "my-package").unwrap();
    assert!(!result);
}

#[test]
fn test_check_global_versions_exact_match() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("my-package")).unwrap();
    let result = check_global_versions(tmp.path(), "my-package").unwrap();
    assert!(result);
}

#[test]
fn test_check_global_versions_versioned_match() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("my-package@1.0.0")).unwrap();
    let result = check_global_versions(tmp.path(), "my-package").unwrap();
    assert!(result);
}

#[test]
fn test_check_global_versions_no_match() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("other-package@1.0.0")).unwrap();
    let result = check_global_versions(tmp.path(), "my-package").unwrap();
    assert!(!result);
}

#[test]
fn test_check_global_versions_partial_name_no_match() {
    let tmp = TempDir::new().unwrap();
    // "my-package-extra@1.0" should NOT match "my-package"
    fs::create_dir(tmp.path().join("my-package-extra@1.0.0")).unwrap();
    let result = check_global_versions(tmp.path(), "my-package").unwrap();
    assert!(!result);
}

// ============================================================================
// detect_package_version tests
// ============================================================================

#[test]
fn test_detect_package_version_horus_yaml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.yaml"),
        "name: my-node\nversion: 2.3.1\ndescription: A test node\n",
    )
    .unwrap();
    let version = detect_package_version(tmp.path());
    assert_eq!(version, Some("2.3.1".to_string()));
}

#[test]
fn test_detect_package_version_cargo_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"my-crate\"\nversion = \"0.5.0\"\n",
    )
    .unwrap();
    let version = detect_package_version(tmp.path());
    assert_eq!(version, Some("0.5.0".to_string()));
}

#[test]
fn test_detect_package_version_package_json() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("package.json"),
        r#"{"name": "my-pkg", "version": "1.2.3"}"#,
    )
    .unwrap();
    let version = detect_package_version(tmp.path());
    assert_eq!(version, Some("1.2.3".to_string()));
}

#[test]
fn test_detect_package_version_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let version = detect_package_version(tmp.path());
    assert_eq!(version, None);
}

#[test]
fn test_detect_package_version_horus_yaml_takes_priority() {
    let tmp = TempDir::new().unwrap();
    // Both horus.yaml and Cargo.toml exist - horus.yaml should win
    fs::write(
        tmp.path().join("horus.yaml"),
        "name: my-node\nversion: 1.0.0\n",
    )
    .unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"my-crate\"\nversion = \"2.0.0\"\n",
    )
    .unwrap();
    let version = detect_package_version(tmp.path());
    assert_eq!(version, Some("1.0.0".to_string()));
}

// ============================================================================
// detect_pypi_installed_version tests
// ============================================================================

#[test]
fn test_detect_pypi_version_from_dist_info() {
    let tmp = TempDir::new().unwrap();
    // Create a .dist-info directory like pip would
    fs::create_dir(tmp.path().join("numpy-1.24.3.dist-info")).unwrap();
    let version = detect_pypi_installed_version(tmp.path(), "numpy");
    assert_eq!(version, Some("1.24.3".to_string()));
}

#[test]
fn test_detect_pypi_version_normalized_name() {
    let tmp = TempDir::new().unwrap();
    // PyPI normalizes dashes to underscores
    fs::create_dir(tmp.path().join("my_package-0.9.1.dist-info")).unwrap();
    let version = detect_pypi_installed_version(tmp.path(), "my-package");
    assert_eq!(version, Some("0.9.1".to_string()));
}

#[test]
fn test_detect_pypi_version_no_dist_info() {
    let tmp = TempDir::new().unwrap();
    let version = detect_pypi_installed_version(tmp.path(), "numpy");
    assert_eq!(version, None);
}

// ============================================================================
// detect_package_info tests
// ============================================================================

#[test]
fn test_detect_package_info_horus_yaml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.yaml"),
        "name: lidar-driver\nversion: 1.0.0\ndescription: A LiDAR driver\nlicense: MIT\npackage_type: driver\ncategories: Perception\n",
    )
    .unwrap();
    let manifest = detect_package_info(tmp.path()).unwrap();
    assert_eq!(manifest.name, "lidar-driver");
    assert_eq!(manifest.version, "1.0.0");
    assert_eq!(manifest.description, Some("A LiDAR driver".to_string()));
    assert_eq!(manifest.license, Some("MIT".to_string()));
    assert_eq!(manifest.package_type, Some("driver".to_string()));
    assert_eq!(manifest.categories, Some("Perception".to_string()));
    assert!(matches!(
        manifest.manifest_format,
        ManifestFormat::HorusYaml
    ));
}

#[test]
fn test_detect_package_info_cargo_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        r#"[package]
name = "my-crate"
version = "0.2.0"
description = "A Rust crate"
license = "Apache-2.0"
repository = "https://github.com/user/repo"

[package.metadata.horus]
package_type = "node"
categories = "Control"
"#,
    )
    .unwrap();
    let manifest = detect_package_info(tmp.path()).unwrap();
    assert_eq!(manifest.name, "my-crate");
    assert_eq!(manifest.version, "0.2.0");
    assert_eq!(manifest.description, Some("A Rust crate".to_string()));
    assert_eq!(manifest.license, Some("Apache-2.0".to_string()));
    assert_eq!(
        manifest.source_url,
        Some("https://github.com/user/repo".to_string())
    );
    assert_eq!(manifest.package_type, Some("node".to_string()));
    assert!(matches!(
        manifest.manifest_format,
        ManifestFormat::CargoToml
    ));
}

#[test]
fn test_detect_package_info_package_json() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("package.json"),
        r#"{
  "name": "my-js-pkg",
  "version": "3.0.0",
  "description": "A JS package",
  "license": "ISC",
  "repository": "https://github.com/user/js-repo",
  "horus": {
    "package_type": "tool",
    "categories": "Utilities"
  }
}"#,
    )
    .unwrap();
    let manifest = detect_package_info(tmp.path()).unwrap();
    assert_eq!(manifest.name, "my-js-pkg");
    assert_eq!(manifest.version, "3.0.0");
    assert_eq!(manifest.description, Some("A JS package".to_string()));
    assert_eq!(manifest.package_type, Some("tool".to_string()));
    assert!(matches!(
        manifest.manifest_format,
        ManifestFormat::PackageJson
    ));
}

#[test]
fn test_detect_package_info_no_manifest_errors() {
    let tmp = TempDir::new().unwrap();
    let result = detect_package_info(tmp.path());
    assert!(result.is_err());
    let err_msg = result.unwrap_err().to_string();
    assert!(err_msg.contains("no package manifest found"));
}

// ============================================================================
// copy_dir_all tests
// ============================================================================

#[test]
fn test_copy_dir_all_basic() {
    let src = TempDir::new().unwrap();
    let dst = TempDir::new().unwrap();
    let dst_path = dst.path().join("copy_target");

    // Create source structure
    fs::write(src.path().join("file1.txt"), "hello").unwrap();
    fs::create_dir(src.path().join("subdir")).unwrap();
    fs::write(src.path().join("subdir/file2.txt"), "world").unwrap();

    copy_dir_all(src.path(), &dst_path).unwrap();

    assert!(dst_path.join("file1.txt").exists());
    assert_eq!(
        fs::read_to_string(dst_path.join("file1.txt")).unwrap(),
        "hello"
    );
    assert!(dst_path.join("subdir/file2.txt").exists());
    assert_eq!(
        fs::read_to_string(dst_path.join("subdir/file2.txt")).unwrap(),
        "world"
    );
}

#[test]
fn test_copy_dir_all_empty_dir() {
    let src = TempDir::new().unwrap();
    let dst = TempDir::new().unwrap();
    let dst_path = dst.path().join("empty_copy");

    copy_dir_all(src.path(), &dst_path).unwrap();
    assert!(dst_path.exists());
    assert!(dst_path.is_dir());
}

// ============================================================================
// add_cargo_deps_to_cargo_toml tests
// ============================================================================

#[test]
fn test_add_cargo_deps_new_dep() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"test\"\nversion = \"0.1.0\"\n\n[dependencies]\n",
    )
    .unwrap();

    add_cargo_deps_to_cargo_toml(tmp.path(), &["serialport@4.2".to_string()]).unwrap();

    let content = fs::read_to_string(tmp.path().join("Cargo.toml")).unwrap();
    assert!(content.contains("serialport"));
    assert!(content.contains("4.2"));
}

#[test]
fn test_add_cargo_deps_skip_existing() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"test\"\nversion = \"0.1.0\"\n\n[dependencies]\nserialport = \"4.0\"\n",
    )
    .unwrap();

    // Trying to add serialport again should not change the version
    add_cargo_deps_to_cargo_toml(tmp.path(), &["serialport@4.2".to_string()]).unwrap();

    let content = fs::read_to_string(tmp.path().join("Cargo.toml")).unwrap();
    // Should still have the original version
    assert!(content.contains("4.0"));
}

#[test]
fn test_add_cargo_deps_no_version() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"test\"\nversion = \"0.1.0\"\n\n[dependencies]\n",
    )
    .unwrap();

    // No @ means wildcard version
    add_cargo_deps_to_cargo_toml(tmp.path(), &["serde".to_string()]).unwrap();

    let content = fs::read_to_string(tmp.path().join("Cargo.toml")).unwrap();
    assert!(content.contains("serde"));
    assert!(content.contains("*"));
}

#[test]
fn test_add_cargo_deps_no_cargo_toml() {
    let tmp = TempDir::new().unwrap();
    let result = add_cargo_deps_to_cargo_toml(tmp.path(), &["serde@1.0".to_string()]);
    assert!(result.is_err());
    assert!(result
        .unwrap_err()
        .to_string()
        .contains("Cargo.toml not found"));
}

// ============================================================================
// extract_package_dependencies tests
// ============================================================================

#[test]
fn test_extract_deps_from_cargo_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        r#"[package]
name = "test"
version = "0.1.0"

[dependencies]
horus_core = "0.2.0"
horus_types = { version = "0.1.0" }
serde = "1.0"
"#,
    )
    .unwrap();

    let deps = extract_package_dependencies(tmp.path()).unwrap();
    // Should only include horus-* packages
    assert!(deps.iter().any(|d| d.name == "horus_core"));
    assert!(deps.iter().any(|d| d.name == "horus_types"));
    // serde is NOT a horus package
    assert!(!deps.iter().any(|d| d.name == "serde"));
}

#[test]
fn test_extract_deps_empty_dir() {
    let tmp = TempDir::new().unwrap();
    let deps = extract_package_dependencies(tmp.path()).unwrap();
    assert!(deps.is_empty());
}

#[test]
fn test_extract_deps_from_package_json() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("package.json"),
        r#"{
  "name": "test",
  "version": "0.1.0",
  "dependencies": {
    "horus-vision": "^1.0.0",
    "express": "4.18.0"
  }
}"#,
    )
    .unwrap();

    let deps = extract_package_dependencies(tmp.path()).unwrap();
    assert!(deps.iter().any(|d| d.name == "horus-vision"));
    assert!(!deps.iter().any(|d| d.name == "express"));
}

// ============================================================================
// Package/PackageMetadata/LockedPackage serde tests
// ============================================================================

#[test]
fn test_package_serde_roundtrip() {
    let pkg = Package {
        name: "nav-stack".to_string(),
        version: "1.0.0".to_string(),
        description: Some("Navigation stack".to_string()),
    };
    let json = serde_json::to_string(&pkg).unwrap();
    let roundtripped: Package = serde_json::from_str(&json).unwrap();
    assert_eq!(roundtripped.name, "nav-stack");
    assert_eq!(roundtripped.version, "1.0.0");
    assert_eq!(
        roundtripped.description,
        Some("Navigation stack".to_string())
    );
}

#[test]
fn test_locked_package_serde_roundtrip() {
    let locked = LockedPackage {
        name: "horus_core".to_string(),
        version: "0.2.0".to_string(),
        checksum: "abc123".to_string(),
        source: PackageSource::Registry,
    };
    let json = serde_json::to_string(&locked).unwrap();
    let roundtripped: LockedPackage = serde_json::from_str(&json).unwrap();
    assert_eq!(roundtripped.name, "horus_core");
    assert_eq!(roundtripped.checksum, "abc123");
    assert_eq!(roundtripped.source, PackageSource::Registry);
}

#[test]
fn test_package_metadata_serde() {
    let meta = PackageMetadata {
        name: "test-pkg".to_string(),
        version: "0.1.0".to_string(),
        checksum: Some("deadbeef".to_string()),
    };
    let json = serde_json::to_string(&meta).unwrap();
    assert!(json.contains("deadbeef"));
    let roundtripped: PackageMetadata = serde_json::from_str(&json).unwrap();
    assert_eq!(roundtripped.checksum, Some("deadbeef".to_string()));
}

// ============================================================================
// SystemInfo serde tests
// ============================================================================

#[test]
fn test_system_info_serde() {
    let info = SystemInfo {
        os: "linux".to_string(),
        arch: "x86_64".to_string(),
        python_version: Some("3.12.0".to_string()),
        rust_version: Some("1.75.0".to_string()),
        gcc_version: None,
        cuda_version: None,
    };
    let json = serde_json::to_string(&info).unwrap();
    let roundtripped: SystemInfo = serde_json::from_str(&json).unwrap();
    assert_eq!(roundtripped.os, "linux");
    assert_eq!(roundtripped.python_version, Some("3.12.0".to_string()));
    assert!(roundtripped.gcc_version.is_none());
}

// ============================================================================
// RegistryClient basic tests
// ============================================================================

#[test]
fn test_registry_client_default() {
    let client = RegistryClient::default();
    // Should have a non-empty base_url
    assert!(!client.base_url().is_empty());
}

#[test]
fn test_registry_client_new() {
    let client = RegistryClient::new();
    assert!(!client.base_url().is_empty());
    // http_client should be available
    let _ = client.http_client();
}

// ============================================================================
// add_features_to_cargo_toml tests
// ============================================================================

#[test]
fn test_add_features_string_dependency() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        r#"[package]
name = "test"
version = "0.1.0"

[dependencies]
horus_library = "0.2.0"
"#,
    )
    .unwrap();

    add_features_to_cargo_toml(tmp.path(), &["serial-hardware".to_string()]).unwrap();

    let content = fs::read_to_string(tmp.path().join("Cargo.toml")).unwrap();
    assert!(content.contains("serial-hardware"));
}

#[test]
fn test_add_features_no_dependencies_section() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();

    // No [dependencies] section at all
    let result = add_features_to_cargo_toml(tmp.path(), &["some-feature".to_string()]);
    assert!(result.is_err());
    assert!(result
        .unwrap_err()
        .to_string()
        .contains("No [dependencies] section"));
}

// ============================================================================
// SHA256 checksum verification tests
// ============================================================================

#[test]
fn test_sha256_checksum_known_value() {
    // SHA256 of empty byte slice is a well-known constant
    let mut hasher = Sha256::new();
    hasher.update(b"");
    let checksum = format!("{:x}", hasher.finalize());
    assert_eq!(
        checksum,
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
}

#[test]
fn test_sha256_checksum_deterministic() {
    // Same input must always produce the same checksum
    let data = b"horus robotics package v1.0.0";
    let mut h1 = Sha256::new();
    h1.update(data);
    let c1 = format!("{:x}", h1.finalize());

    let mut h2 = Sha256::new();
    h2.update(data);
    let c2 = format!("{:x}", h2.finalize());

    assert_eq!(c1, c2);
    assert_eq!(c1.len(), 64); // SHA256 hex is always 64 chars
}

#[test]
fn test_sha256_checksum_differs_on_corruption() {
    let original = b"horus-nav-stack-1.0.0.tar.gz contents here";
    let mut corrupted = original.to_vec();
    corrupted[10] ^= 0xFF; // flip one byte

    let mut h1 = Sha256::new();
    h1.update(original);
    let original_checksum = format!("{:x}", h1.finalize());

    let mut h2 = Sha256::new();
    h2.update(&corrupted);
    let corrupted_checksum = format!("{:x}", h2.finalize());

    assert_ne!(original_checksum, corrupted_checksum);
}

#[test]
fn test_sha256_incremental_update_matches_single() {
    // Incremental updates should produce the same result as a single update
    // This is how install_from_registry computes checksums on streamed bytes
    let chunk1 = b"first chunk of package data";
    let chunk2 = b"second chunk of package data";

    let mut incremental = Sha256::new();
    incremental.update(chunk1);
    incremental.update(chunk2);
    let incremental_sum = format!("{:x}", incremental.finalize());

    let mut combined = Sha256::new();
    let mut all_data = Vec::new();
    all_data.extend_from_slice(chunk1);
    all_data.extend_from_slice(chunk2);
    combined.update(&all_data);
    let combined_sum = format!("{:x}", combined.finalize());

    assert_eq!(incremental_sum, combined_sum);
}

// ============================================================================
// Platform filtering tests (DependencySpec target field)
// ============================================================================

#[test]
fn test_platform_filter_no_target_passes_all() {
    use crate::dependency_resolver::{DependencySource, DependencySpec};
    use semver::VersionReq;

    let dep = DependencySpec {
        name: "horus_core".to_string(),
        requirement: VersionReq::STAR,
        source: DependencySource::Registry,
        target: None, // No target = all platforms
    };

    let current_platform = format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH);
    let passes = match &dep.target {
        Some(t) => t == &current_platform || t == std::env::consts::OS,
        None => true,
    };
    assert!(passes);
}

#[test]
fn test_platform_filter_matching_os_arch() {
    use crate::dependency_resolver::{DependencySource, DependencySpec};
    use semver::VersionReq;

    let current_platform = format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH);

    let dep = DependencySpec {
        name: "platform-specific-pkg".to_string(),
        requirement: VersionReq::STAR,
        source: DependencySource::Registry,
        target: Some(current_platform.clone()),
    };

    let passes = match &dep.target {
        Some(t) => t == &current_platform || t == std::env::consts::OS,
        None => true,
    };
    assert!(passes);
}

#[test]
fn test_platform_filter_matching_os_only() {
    use crate::dependency_resolver::{DependencySource, DependencySpec};
    use semver::VersionReq;

    let current_platform = format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH);

    let dep = DependencySpec {
        name: "os-specific-pkg".to_string(),
        requirement: VersionReq::STAR,
        source: DependencySource::Registry,
        target: Some(std::env::consts::OS.to_string()),
    };

    let passes = match &dep.target {
        Some(t) => t == &current_platform || t == std::env::consts::OS,
        None => true,
    };
    assert!(passes);
}

#[test]
fn test_platform_filter_non_matching_target_excluded() {
    use crate::dependency_resolver::{DependencySource, DependencySpec};
    use semver::VersionReq;

    let current_platform = format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH);

    let dep = DependencySpec {
        name: "windows-only-pkg".to_string(),
        requirement: VersionReq::STAR,
        source: DependencySource::Registry,
        target: Some("windows-x86_64".to_string()),
    };

    let passes = match &dep.target {
        Some(t) => t == &current_platform || t == std::env::consts::OS,
        None => true,
    };
    // On Linux, a windows-only dep should be excluded
    if std::env::consts::OS != "windows" {
        assert!(!passes);
    }
}

#[test]
fn test_platform_filter_mixed_deps() {
    use crate::dependency_resolver::{DependencySource, DependencySpec};
    use semver::VersionReq;

    let current_platform = format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH);

    let deps = vec![
        DependencySpec {
            name: "universal-pkg".to_string(),
            requirement: VersionReq::STAR,
            source: DependencySource::Registry,
            target: None,
        },
        DependencySpec {
            name: "current-platform-pkg".to_string(),
            requirement: VersionReq::STAR,
            source: DependencySource::Registry,
            target: Some(current_platform.clone()),
        },
        DependencySpec {
            name: "other-platform-pkg".to_string(),
            requirement: VersionReq::STAR,
            source: DependencySource::Registry,
            target: Some("mips-unknown".to_string()),
        },
    ];

    let filtered: Vec<&DependencySpec> = deps
        .iter()
        .filter(|dep| match &dep.target {
            Some(t) => t == &current_platform || t == std::env::consts::OS,
            None => true,
        })
        .collect();

    // universal + current-platform pass; mips excluded
    assert_eq!(filtered.len(), 2);
    assert_eq!(filtered[0].name, "universal-pkg");
    assert_eq!(filtered[1].name, "current-platform-pkg");
}

// ============================================================================
// EnvironmentManifest serde tests
// ============================================================================

#[test]
fn test_environment_manifest_serde_roundtrip() {
    let manifest = EnvironmentManifest {
        horus_id: "abc123".to_string(),
        name: Some("my-robot-env".to_string()),
        description: Some("Robot navigation environment".to_string()),
        packages: vec![
            LockedPackage {
                name: "horus_core".to_string(),
                version: "0.2.0".to_string(),
                checksum: "deadbeef".to_string(),
                source: PackageSource::Registry,
            },
            LockedPackage {
                name: "numpy".to_string(),
                version: "1.24.0".to_string(),
                checksum: "cafebabe".to_string(),
                source: PackageSource::PyPI,
            },
        ],
        system: SystemInfo {
            os: "linux".to_string(),
            arch: "x86_64".to_string(),
            python_version: Some("3.12.0".to_string()),
            rust_version: Some("1.75.0".to_string()),
            gcc_version: None,
            cuda_version: Some("12.3".to_string()),
        },
        created_at: chrono::Utc::now(),
        horus_version: "0.3.0".to_string(),
    };

    let json = serde_json::to_string(&manifest).unwrap();
    let roundtripped: EnvironmentManifest = serde_json::from_str(&json).unwrap();

    assert_eq!(roundtripped.horus_id, "abc123");
    assert_eq!(roundtripped.name, Some("my-robot-env".to_string()));
    assert_eq!(roundtripped.packages.len(), 2);
    assert_eq!(roundtripped.packages[0].name, "horus_core");
    assert_eq!(roundtripped.packages[1].source, PackageSource::PyPI);
    assert_eq!(roundtripped.system.os, "linux");
    assert_eq!(roundtripped.system.cuda_version, Some("12.3".to_string()));
    assert_eq!(roundtripped.horus_version, "0.3.0");
}

#[test]
fn test_environment_manifest_empty_packages() {
    let manifest = EnvironmentManifest {
        horus_id: "empty-env".to_string(),
        name: None,
        description: None,
        packages: vec![],
        system: SystemInfo {
            os: "macos".to_string(),
            arch: "aarch64".to_string(),
            python_version: None,
            rust_version: None,
            gcc_version: None,
            cuda_version: None,
        },
        created_at: chrono::Utc::now(),
        horus_version: "0.1.0".to_string(),
    };

    let json = serde_json::to_string(&manifest).unwrap();
    let roundtripped: EnvironmentManifest = serde_json::from_str(&json).unwrap();
    assert!(roundtripped.packages.is_empty());
    assert!(roundtripped.name.is_none());
}

// ============================================================================
// DriverListEntry serde tests
// ============================================================================

#[test]
fn test_driver_list_entry_deserialize() {
    let json = r#"{
        "name": "rplidar-driver",
        "description": "RPLidar A1/A2/A3 driver",
        "bus_type": "serial",
        "category": "sensor"
    }"#;
    let entry: DriverListEntry = serde_json::from_str(json).unwrap();
    assert_eq!(entry.name, "rplidar-driver");
    assert_eq!(
        entry.description,
        Some("RPLidar A1/A2/A3 driver".to_string())
    );
    assert_eq!(entry.bus_type, Some("serial".to_string()));
    assert_eq!(entry.category, Some("sensor".to_string()));
}

#[test]
fn test_driver_list_entry_minimal() {
    let json = r#"{"name": "basic-driver"}"#;
    let entry: DriverListEntry = serde_json::from_str(json).unwrap();
    assert_eq!(entry.name, "basic-driver");
    assert!(entry.description.is_none());
    assert!(entry.bus_type.is_none());
    assert!(entry.category.is_none());
}

// ============================================================================
// copy_dir_all deep nesting test
// ============================================================================

#[test]
fn test_copy_dir_all_nested_structure() {
    let src = TempDir::new().unwrap();
    let dst = TempDir::new().unwrap();
    let dst_path = dst.path().join("deep_copy");

    // Create nested source structure mimicking a real package
    fs::create_dir_all(src.path().join("src/nodes")).unwrap();
    fs::create_dir_all(src.path().join("config")).unwrap();
    fs::write(
        src.path().join("horus.yaml"),
        "name: test\nversion: 0.1.0\n",
    )
    .unwrap();
    fs::write(
        src.path().join("src/nodes/lidar.rs"),
        "pub struct LidarNode;",
    )
    .unwrap();
    fs::write(src.path().join("config/params.yaml"), "rate: 10.0\n").unwrap();

    copy_dir_all(src.path(), &dst_path).unwrap();

    assert!(dst_path.join("horus.yaml").exists());
    assert!(dst_path.join("src/nodes/lidar.rs").exists());
    assert!(dst_path.join("config/params.yaml").exists());
    assert_eq!(
        fs::read_to_string(dst_path.join("src/nodes/lidar.rs")).unwrap(),
        "pub struct LidarNode;"
    );
}
