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
    assert_eq!(url_encode_package_name("@org/pkg"), "%40org%2Fpkg");
}

#[test]
fn test_url_encode_special_chars() {
    assert_eq!(url_encode_package_name("a b"), "a%20b");
    assert_eq!(url_encode_package_name("a#b"), "a%23b");
    assert_eq!(url_encode_package_name("a?b"), "a%3Fb");
}

#[test]
fn test_url_encode_percent_itself() {
    // % must be encoded first to avoid double-encoding
    assert_eq!(url_encode_package_name("a%b"), "a%25b");
}

#[test]
fn test_package_name_to_path_simple() {
    assert_eq!(package_name_to_path("lidar-driver"), "lidar-driver");
}

#[test]
fn test_package_name_to_path_scoped() {
    assert_eq!(package_name_to_path("@org/pkg"), "org__pkg");
}

#[test]
fn test_package_name_to_path_nested_scope() {
    assert_eq!(package_name_to_path("@org/sub/pkg"), "org__sub__pkg");
}

// ============================================================================
// validate_package_name tests
// ============================================================================

#[test]
fn test_validate_name_valid() {
    assert!(validate_package_name("lidar-driver").is_ok());
    assert!(validate_package_name("my_package").is_ok());
    assert!(validate_package_name("nav2").is_ok());
    assert!(validate_package_name("ab").is_ok()); // min length
}

#[test]
fn test_validate_name_scoped_ok() {
    assert!(validate_package_name("@org/pkg").is_ok());
    assert!(validate_package_name("@my-team/sensor-driver").is_ok());
}

#[test]
fn test_validate_name_scoped_missing_slash() {
    assert!(validate_package_name("@orgpkg").is_err());
}

#[test]
fn test_validate_name_too_short() {
    assert!(validate_package_name("a").is_err());
}

#[test]
fn test_validate_name_too_long() {
    let long_name: String = std::iter::repeat('a').take(65).collect();
    assert!(validate_package_name(&long_name).is_err());
}

#[test]
fn test_validate_name_uppercase_rejected() {
    assert!(validate_package_name("MyPackage").is_err());
}

#[test]
fn test_validate_name_starts_with_digit() {
    assert!(validate_package_name("1package").is_err());
}

#[test]
fn test_validate_name_reserved() {
    assert!(validate_package_name("horus").is_err());
    assert!(validate_package_name("core").is_err());
    assert!(validate_package_name("admin").is_err());
}

#[test]
fn test_validate_name_special_chars_rejected() {
    assert!(validate_package_name("my package").is_err());
    assert!(validate_package_name("my.package").is_err());
    assert!(validate_package_name("my+package").is_err());
}

// ============================================================================
// ManifestFormat Display tests
// ============================================================================

#[test]
fn test_manifest_format_display_horus_toml() {
    assert_eq!(format!("{}", ManifestFormat::HorusToml), "horus.toml");
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
fn test_detect_package_version_horus_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"my-node\"\nversion = \"2.3.1\"\ndescription = \"A test node\"\n",
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
fn test_detect_package_version_horus_toml_takes_priority() {
    let tmp = TempDir::new().unwrap();
    // Both horus.toml and Cargo.toml exist - horus.toml should win
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"my-node\"\nversion = \"1.0.0\"\n",
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
fn test_detect_package_info_horus_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"lidar-driver\"\nversion = \"1.0.0\"\ndescription = \"A LiDAR driver\"\nlicense = \"MIT\"\npackage-type = \"driver\"\ncategories = [\"Perception\"]\n",
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
        ManifestFormat::HorusToml
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
        src.path().join("horus.toml"),
        "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    fs::write(
        src.path().join("src/nodes/lidar.rs"),
        "pub struct LidarNode;",
    )
    .unwrap();
    fs::write(src.path().join("config/params.yaml"), "rate: 10.0\n").unwrap();

    copy_dir_all(src.path(), &dst_path).unwrap();

    assert!(dst_path.join("horus.toml").exists());
    assert!(dst_path.join("src/nodes/lidar.rs").exists());
    assert!(dst_path.join("config/params.yaml").exists());
    assert_eq!(
        fs::read_to_string(dst_path.join("src/nodes/lidar.rs")).unwrap(),
        "pub struct LidarNode;"
    );
}

// ============================================================================
// Publish pipeline tests
// ============================================================================

#[test]
fn test_publish_tarball_creation_roundtrip() {
    // Simulate the publish flow: create project files, tar+gz, then extract and verify
    let project = TempDir::new().unwrap();
    let project_dir = project.path();

    // Create a minimal horus project
    fs::write(
        project_dir.join("horus.toml"),
        "[package]\nname = \"test-pkg\"\nenable = true\n",
    )
    .unwrap();
    fs::create_dir_all(project_dir.join("src")).unwrap();
    fs::write(project_dir.join("src/main.rs"), "fn main() {}").unwrap();

    // Build tarball (same logic as publish.rs)
    let tar_path = project_dir.join("test-pkg.tar.gz");
    {
        let tar_file = fs::File::create(&tar_path).unwrap();
        let enc = flate2::write::GzEncoder::new(tar_file, flate2::Compression::default());
        let mut tar = tar::Builder::new(enc);
        tar.append_path_with_name(project_dir.join("horus.toml"), "horus.toml")
            .unwrap();
        tar.append_path_with_name(project_dir.join("src/main.rs"), "src/main.rs")
            .unwrap();
        tar.finish().unwrap();
    }

    // Verify tarball can be extracted and contains expected files
    let extract_dir = TempDir::new().unwrap();
    let tar_data = fs::read(&tar_path).unwrap();
    let decoder = flate2::read::GzDecoder::new(&tar_data[..]);
    let mut archive = tar::Archive::new(decoder);
    archive.unpack(extract_dir.path()).unwrap();

    assert!(extract_dir.path().join("horus.toml").exists());
    assert!(extract_dir.path().join("src/main.rs").exists());

    let toml_content = fs::read_to_string(extract_dir.path().join("horus.toml")).unwrap();
    assert!(toml_content.contains("test-pkg"));
}

#[test]
fn test_publish_detect_info_from_project() {
    let project = TempDir::new().unwrap();
    let project_dir = project.path();

    fs::write(
        project_dir.join("Cargo.toml"),
        r#"[package]
name = "my-robot-driver"
version = "2.1.0"
description = "A test driver"
license = "MIT"
"#,
    )
    .unwrap();

    let info = detect_package_info(project_dir).unwrap();
    assert_eq!(info.name, "my-robot-driver");
    assert_eq!(info.version, "2.1.0");
    assert_eq!(info.description, Some("A test driver".to_string()));
    assert_eq!(info.license, Some("MIT".to_string()));
}

#[test]
fn test_publish_sha256_matches_downloaded() {
    // Verify that checksum computed during publish matches one computed on download
    let data = b"package binary contents here";
    let checksum1 = {
        use sha2::{Digest, Sha256};
        let mut hasher = Sha256::new();
        hasher.update(data);
        format!("{:x}", hasher.finalize())
    };
    let checksum2 = {
        use sha2::{Digest, Sha256};
        let mut hasher = Sha256::new();
        hasher.update(data);
        format!("{:x}", hasher.finalize())
    };
    assert_eq!(checksum1, checksum2);
    assert_eq!(checksum1.len(), 64); // SHA256 hex = 64 chars
}

// ============================================================================
// Version-aware global cache check tests
// ============================================================================

#[test]
fn test_check_global_version_satisfies_any() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("my-pkg@1.0.0")).unwrap();
    assert!(check_global_version_satisfies(tmp.path(), "my-pkg", None).unwrap());
}

#[test]
fn test_check_global_version_satisfies_matching() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("my-pkg@1.2.3")).unwrap();
    let req = semver::VersionReq::parse(">=1.0.0, <2.0.0").unwrap();
    assert!(check_global_version_satisfies(tmp.path(), "my-pkg", Some(&req)).unwrap());
}

#[test]
fn test_check_global_version_satisfies_not_matching() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("my-pkg@1.2.3")).unwrap();
    let req = semver::VersionReq::parse(">=2.0.0").unwrap();
    assert!(!check_global_version_satisfies(tmp.path(), "my-pkg", Some(&req)).unwrap());
}

#[test]
fn test_check_global_version_satisfies_multiple_versions() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("my-pkg@1.0.0")).unwrap();
    fs::create_dir(tmp.path().join("my-pkg@2.5.0")).unwrap();
    let req = semver::VersionReq::parse(">=2.0.0").unwrap();
    assert!(check_global_version_satisfies(tmp.path(), "my-pkg", Some(&req)).unwrap());
}

#[test]
fn test_check_global_version_unversioned_dir_matches_any_req() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("my-pkg")).unwrap();
    let req = semver::VersionReq::parse(">=5.0.0").unwrap();
    // Unversioned directory always matches (legacy/manual installs)
    assert!(check_global_version_satisfies(tmp.path(), "my-pkg", Some(&req)).unwrap());
}

// ============================================================================
// semver_req_to_pip conversion tests
// ============================================================================

#[test]
fn test_semver_req_to_pip_star_returns_none() {
    let req = semver::VersionReq::STAR;
    assert_eq!(super::install::semver_req_to_pip(&req), None);
}

#[test]
fn test_semver_req_to_pip_caret() {
    // semver crate preserves ^1.2 as "^1.2" in to_string()
    let req = semver::VersionReq::parse("^1.2").unwrap();
    let pip = super::install::semver_req_to_pip(&req).unwrap();
    assert!(!pip.is_empty());
    assert!(pip.contains("1.2"), "expected 1.2 in: {}", pip);
}

#[test]
fn test_semver_req_to_pip_tilde() {
    // semver crate preserves ~1.2 as "~1.2" in to_string()
    let req = semver::VersionReq::parse("~1.2").unwrap();
    let pip = super::install::semver_req_to_pip(&req).unwrap();
    assert!(!pip.is_empty());
    assert!(pip.contains("1.2"), "expected 1.2 in: {}", pip);
}

#[test]
fn test_semver_req_to_pip_exact() {
    let req = semver::VersionReq::parse("=3.0.0").unwrap();
    let pip = super::install::semver_req_to_pip(&req).unwrap();
    assert!(pip.contains("3.0.0"), "expected 3.0.0 in: {}", pip);
}

#[test]
fn test_semver_req_to_pip_range() {
    let req = semver::VersionReq::parse(">=2.0.0, <3.0.0").unwrap();
    let pip = super::install::semver_req_to_pip(&req).unwrap();
    assert!(pip.contains(">=2.0.0"), "expected >=2.0.0 in: {}", pip);
    assert!(pip.contains("<3.0.0"), "expected <3.0.0 in: {}", pip);
}

// ============================================================================
// TempDirGuard RAII tests
// ============================================================================

#[test]
fn test_temp_dir_guard_armed_cleans_up() {
    let tmp = TempDir::new().unwrap();
    let guard_dir = tmp.path().join("guard_test");
    fs::create_dir(&guard_dir).unwrap();
    fs::write(guard_dir.join("file.txt"), "test").unwrap();

    let saved_path = guard_dir.clone();
    {
        let _guard = super::install::TempDirGuard::new(guard_dir);
        assert!(saved_path.exists());
        // guard drops here — should clean up
    }
    assert!(!saved_path.exists(), "armed guard should clean up on drop");
}

#[test]
fn test_temp_dir_guard_disarmed_does_not_clean() {
    let tmp = TempDir::new().unwrap();
    let guard_dir = tmp.path().join("guard_test_disarm");
    fs::create_dir(&guard_dir).unwrap();
    fs::write(guard_dir.join("file.txt"), "test").unwrap();

    let saved_path = guard_dir.clone();
    {
        let mut guard = super::install::TempDirGuard::new(guard_dir);
        guard.disarm();
        // guard drops here — should NOT clean up
    }
    assert!(
        saved_path.exists(),
        "disarmed guard should not clean up on drop"
    );
}

#[test]
fn test_temp_dir_guard_nonexistent_path_no_panic() {
    // Dropping a guard for a non-existent path should not panic
    let _guard = super::install::TempDirGuard::new(PathBuf::from("/nonexistent/path/xyz"));
    // drops silently
}

// ============================================================================
// Ed25519 signature verification tests
// ============================================================================

#[test]
fn test_verify_signature_valid() {
    use ed25519_dalek::{Signer, SigningKey};
    use rand::rngs::OsRng;

    let mut csprng = OsRng;
    let signing_key = SigningKey::generate(&mut csprng);
    let verifying_key = signing_key.verifying_key();

    let data = b"package tarball contents";
    let signature = signing_key.sign(data);
    let sig_hex = hex::encode(signature.to_bytes());

    // Write public key to temp file
    let tmp = TempDir::new().unwrap();
    let pub_key_path = tmp.path().join("signing_key.pub");
    fs::write(&pub_key_path, hex::encode(verifying_key.to_bytes())).unwrap();

    let result = super::install::verify_package_signature(data, &sig_hex, &pub_key_path).unwrap();
    assert!(result, "valid signature should verify");
}

#[test]
fn test_verify_signature_invalid() {
    use ed25519_dalek::{Signer, SigningKey};
    use rand::rngs::OsRng;

    let mut csprng = OsRng;
    let signing_key = SigningKey::generate(&mut csprng);
    let verifying_key = signing_key.verifying_key();

    let data = b"package tarball contents";
    let tampered_data = b"TAMPERED tarball contents";
    let signature = signing_key.sign(data);
    let sig_hex = hex::encode(signature.to_bytes());

    let tmp = TempDir::new().unwrap();
    let pub_key_path = tmp.path().join("signing_key.pub");
    fs::write(&pub_key_path, hex::encode(verifying_key.to_bytes())).unwrap();

    // Verify against tampered data — should return false
    let result =
        super::install::verify_package_signature(tampered_data, &sig_hex, &pub_key_path).unwrap();
    assert!(!result, "signature of different data should not verify");
}

#[test]
fn test_verify_signature_bad_hex() {
    let tmp = TempDir::new().unwrap();
    let pub_key_path = tmp.path().join("signing_key.pub");
    fs::write(
        &pub_key_path,
        "0000000000000000000000000000000000000000000000000000000000000000",
    )
    .unwrap();

    let result =
        super::install::verify_package_signature(b"data", "not-valid-hex!!!", &pub_key_path);
    assert!(result.is_err(), "invalid hex should return error");
}

#[test]
fn test_verify_signature_missing_key_file() {
    let result = super::install::verify_package_signature(
        b"data",
        "abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
        Path::new("/nonexistent/key.pub"),
    );
    assert!(result.is_err(), "missing key file should return error");
}

#[test]
fn test_verify_signature_wrong_key() {
    use ed25519_dalek::{Signer, SigningKey};
    use rand::rngs::OsRng;

    let mut csprng = OsRng;
    let signing_key = SigningKey::generate(&mut csprng);
    let other_key = SigningKey::generate(&mut csprng);

    let data = b"package data";
    let signature = signing_key.sign(data);
    let sig_hex = hex::encode(signature.to_bytes());

    let tmp = TempDir::new().unwrap();
    let pub_key_path = tmp.path().join("signing_key.pub");
    // Write the OTHER key's public key — should fail verification
    fs::write(
        &pub_key_path,
        hex::encode(other_key.verifying_key().to_bytes()),
    )
    .unwrap();

    let result = super::install::verify_package_signature(data, &sig_hex, &pub_key_path).unwrap();
    assert!(!result, "wrong key should not verify");
}

// ============================================================================
// Tarball extraction security tests
// ============================================================================

#[test]
fn test_extract_tarball_normal_files() {
    // Normal tarball should extract without issues
    let tmp = TempDir::new().unwrap();
    let tar_path = tmp.path().join("normal.tar.gz");

    {
        let file = fs::File::create(&tar_path).unwrap();
        let enc = flate2::write::GzEncoder::new(file, flate2::Compression::default());
        let mut tar = tar::Builder::new(enc);

        let mut header = tar::Header::new_gnu();
        header.set_size(5);
        header.set_mode(0o644);
        header.set_cksum();
        tar.append_data(&mut header, "src/main.rs", b"hello" as &[u8])
            .unwrap();
        tar.finish().unwrap();
    }

    let extract_dir = tmp.path().join("extracted");
    fs::create_dir(&extract_dir).unwrap();
    let data = fs::read(&tar_path).unwrap();
    let decoder = flate2::read::GzDecoder::new(&data[..]);
    let mut archive = tar::Archive::new(decoder);
    archive.unpack(&extract_dir).unwrap();

    assert!(extract_dir.join("src/main.rs").exists());
    assert_eq!(
        fs::read_to_string(extract_dir.join("src/main.rs")).unwrap(),
        "hello"
    );
}

#[test]
fn test_corrupt_tarball_fails_gracefully() {
    // Random bytes should fail decompression
    let corrupt_data = vec![0u8, 1, 2, 3, 4, 5, 6, 7, 8, 9];
    let decoder = flate2::read::GzDecoder::new(&corrupt_data[..]);
    let mut archive = tar::Archive::new(decoder);

    let tmp = TempDir::new().unwrap();
    let result = archive.unpack(tmp.path());
    assert!(result.is_err(), "corrupt tarball should fail extraction");
}

#[test]
fn test_empty_tarball_extracts_nothing() {
    let tmp = TempDir::new().unwrap();
    let tar_path = tmp.path().join("empty.tar.gz");

    {
        let file = fs::File::create(&tar_path).unwrap();
        let enc = flate2::write::GzEncoder::new(file, flate2::Compression::default());
        let mut tar = tar::Builder::new(enc);
        tar.finish().unwrap();
    }

    let extract_dir = tmp.path().join("extracted");
    fs::create_dir(&extract_dir).unwrap();
    let data = fs::read(&tar_path).unwrap();
    let decoder = flate2::read::GzDecoder::new(&data[..]);
    let mut archive = tar::Archive::new(decoder);
    archive.unpack(&extract_dir).unwrap();

    // Should be an empty directory
    let entries: Vec<_> = fs::read_dir(&extract_dir).unwrap().collect();
    assert!(entries.is_empty(), "empty tarball should extract no files");
}

#[test]
fn test_signing_keypair_roundtrip_with_verify() {
    // Simulate generate_signing_keypair() format + verify_package_signature() compatibility
    use ed25519_dalek::{Signer, SigningKey};
    use rand::rngs::OsRng;

    let signing_key = SigningKey::generate(&mut OsRng);
    let verifying_key = signing_key.verifying_key();

    let tmp = TempDir::new().unwrap();

    // Write keys in the same format as generate_signing_keypair()
    let secret_path = tmp.path().join("signing_key");
    let public_path = tmp.path().join("signing_key.pub");
    fs::write(&secret_path, signing_key.to_bytes()).unwrap();
    fs::write(&public_path, hex::encode(verifying_key.to_bytes())).unwrap();

    // Sign data using the secret key (simulating publish)
    let package_data = b"full package tarball binary data here";
    let signature = signing_key.sign(package_data);
    let sig_hex = hex::encode(signature.to_bytes());

    // Verify using verify_package_signature (simulating install)
    let result =
        super::install::verify_package_signature(package_data, &sig_hex, &public_path).unwrap();
    assert!(result, "keypair roundtrip should verify successfully");

    // Also verify that reading back the secret key works
    let secret_bytes = fs::read(&secret_path).unwrap();
    assert_eq!(secret_bytes.len(), 32, "secret key should be 32 bytes");
    let pub_hex = fs::read_to_string(&public_path).unwrap();
    assert_eq!(pub_hex.len(), 64, "public key hex should be 64 chars");
}

// ============================================================================
// Package name validation parity test matrix (client == server rules)
// ============================================================================
// Both client (mod.rs:validate_package_name) and server (main.rs:validate_package_name)
// enforce identical rules. This matrix documents and tests all edge cases.
// If you change validation rules, update BOTH implementations and this matrix.

#[test]
fn test_validation_parity_matrix() {
    let cases: Vec<(&str, bool)> = vec![
        // --- Valid unscoped names ---
        ("lidar-driver", true),
        ("my_package", true),
        ("nav2", true),
        ("ab", true), // min length (2)
        ("a-b", true),
        ("sensor-fusion-v2", true),
        ("ros2-bridge", true),
        ("horus-nav-stack", true), // contains "horus" but not equal
        (
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            true,
        ), // 64 chars = max
        // --- Valid scoped names ---
        ("@org/pkg", true),
        ("@my-team/sensor-driver", true),
        ("@ab/cd", true), // min segments (2 chars each)
        ("@org/my-package-v2", true),
        ("@my_org/my_pkg", true), // underscores ok
        // --- Invalid: too short ---
        ("a", false), // 1 char
        ("", false),  // empty
        // --- Invalid: too long ---
        // 65 chars for unscoped
        (
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            false,
        ),
        // --- Invalid: starts with digit ---
        ("1package", false),
        ("2nav", false),
        // --- Invalid: uppercase ---
        ("MyPackage", false),
        ("ALLCAPS", false),
        ("camelCase", false),
        // --- Invalid: special characters ---
        ("my package", false), // space
        ("my.package", false), // dot
        ("my+package", false), // plus
        ("my@package", false), // @ in middle
        ("my!pkg", false),     // exclamation
        ("pkg#1", false),      // hash
        // --- Invalid: path traversal ---
        ("../etc/passwd", false),
        ("my..pkg", false),
        ("pkg\\name", false),
        // --- Invalid: reserved names ---
        ("horus", false),
        ("core", false),
        ("admin", false),
        ("test", false),
        ("main", false),
        ("api", false),
        ("root", false),
        ("system", false),
        ("std", false),
        ("lib", false),
        // --- Invalid: scoped format errors ---
        ("@orgpkg", false), // missing slash
        ("@/pkg", false),   // empty org (1 char)
        ("@a/pkg", false),  // org too short (1 char)
        ("@org/a", false),  // pkg too short (1 char)
        // --- Invalid: scoped reserved segments ---
        ("@admin/pkg", false),  // org is reserved
        ("@org/core", false),   // pkg is reserved
        ("@horus/test", false), // both reserved
        // --- Invalid: scoped with bad chars ---
        ("@Org/pkg", false), // uppercase org
        ("@org/Pkg", false), // uppercase pkg
        // --- Invalid: unicode ---
        ("pàckage", false), // accented char
        ("日本語", false),  // CJK
    ];

    for (name, expected_valid) in &cases {
        let result = validate_package_name(name);
        assert_eq!(
            result.is_ok(),
            *expected_valid,
            "validate_package_name({:?}) = {:?}, expected valid={}",
            name,
            result,
            expected_valid
        );
    }
}

#[test]
fn test_validation_parity_reserved_names_complete() {
    // All reserved names must be rejected
    let reserved = &[
        "horus",
        "core",
        "std",
        "lib",
        "test",
        "main",
        "mod",
        "pub",
        "use",
        "crate",
        "self",
        "super",
        "extern",
        "fn",
        "let",
        "const",
        "static",
        "mut",
        "ref",
        "type",
        "impl",
        "trait",
        "struct",
        "enum",
        "union",
        "admin",
        "api",
        "www",
        "mail",
        "ftp",
        "localhost",
        "root",
        "system",
    ];
    for name in reserved {
        assert!(
            validate_package_name(name).is_err(),
            "reserved name {:?} should be rejected",
            name
        );
    }
}

#[test]
fn test_validation_parity_scoped_reserved_as_org() {
    // Reserved names as org segment should be rejected
    let reserved = &["horus", "core", "admin", "api", "root", "system"];
    for name in reserved {
        let scoped = format!("@{}/my-pkg", name);
        assert!(
            validate_package_name(&scoped).is_err(),
            "reserved org {:?} in {:?} should be rejected",
            name,
            scoped
        );
    }
}

#[test]
fn test_validation_parity_scoped_reserved_as_pkg() {
    // Reserved names as package segment should be rejected
    let reserved = &["horus", "core", "admin", "test", "main", "std"];
    for name in reserved {
        let scoped = format!("@my-org/{}", name);
        assert!(
            validate_package_name(&scoped).is_err(),
            "reserved pkg {:?} in @my-org/{:?} should be rejected",
            name,
            name
        );
    }
}
