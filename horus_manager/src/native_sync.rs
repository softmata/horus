//! Bidirectional sync between native build files and `horus.toml`.
//!
//! When a native tool (cargo, pip, cmake) modifies `.horus/Cargo.toml` etc.,
//! this module detects the change via fingerprints, parses the native file,
//! diffs against `horus.toml`, and merges new/removed dependencies back.

use anyhow::Result;
use std::path::Path;

use crate::fingerprint::Fingerprints;
use crate::manifest::{DepSource, DependencyValue, HorusManifest};

/// Which native file type to sync.
#[derive(Debug, Clone, Copy)]
pub enum NativeFileType {
    Cargo,
    Pyproject,
    Cmake,
}

impl NativeFileType {
    /// Filename within `.horus/`.
    pub fn filename(&self) -> &str {
        match self {
            Self::Cargo => "Cargo.toml",
            Self::Pyproject => "pyproject.toml",
            Self::Cmake => "CMakeLists.txt",
        }
    }
}

/// Result of a sync operation.
pub enum SyncResult {
    /// No external modifications detected.
    NoChanges,
    /// Changes were synced back to horus.toml.
    Synced {
        added: usize,
        removed: usize,
        modified: usize,
    },
}

/// Check whether a dependency name is a horus-internal crate that should
/// never be synced back to `horus.toml`.
pub fn is_horus_internal(name: &str) -> bool {
    matches!(
        name,
        "horus"
            | "horus_core"
            | "horus_library"
            | "horus_macros"
            | "horus_sys"
            | "horus-py"
            | "horus-robotics"
    )
}

/// Sync changes from a native file back to the manifest if modified externally.
///
/// Returns the number of added/removed deps, or `NoChanges` if the fingerprint
/// matches (no external modification).
pub fn sync_from_native(
    project_dir: &Path,
    manifest: &mut HorusManifest,
    file_type: NativeFileType,
    fingerprints: &mut Fingerprints,
) -> Result<SyncResult> {
    let filename = file_type.filename();

    // Check fingerprint — was the file modified externally?
    if !fingerprints.is_modified(filename, project_dir) {
        return Ok(SyncResult::NoChanges);
    }

    let native_path = project_dir.join(".horus").join(filename);
    if !native_path.exists() {
        return Ok(SyncResult::NoChanges);
    }

    // Parse the native file to extract deps (+ dev deps for Cargo)
    let (native_deps, native_dev_deps) = match file_type {
        NativeFileType::Cargo => {
            let extracted = crate::commands::migrate::extract_cargo_all_deps(&native_path)?;
            (extracted.deps, Some(extracted.dev_deps))
        }
        NativeFileType::Pyproject => {
            (crate::commands::migrate::extract_pyproject_deps(&native_path)?, None)
        }
        NativeFileType::Cmake => {
            (crate::commands::migrate::extract_cmake_deps(&native_path)?, None)
        }
    };

    // Build set of driver-generated crate names to exclude
    let driver_crates = collect_driver_crates(manifest);

    // Sync regular deps
    let (added, modified, removed) =
        sync_dep_map(&native_deps, &mut manifest.dependencies, &driver_crates, file_type);

    // Sync dev deps (Cargo only)
    let (dev_added, dev_modified, dev_removed) = if let Some(ref dev) = native_dev_deps {
        sync_dep_map(dev, &mut manifest.dev_dependencies, &driver_crates, file_type)
    } else {
        (0, 0, 0)
    };

    let total_added = added + dev_added;
    let total_removed = removed + dev_removed;
    let total_modified = modified + dev_modified;

    if total_added > 0 || total_removed > 0 || total_modified > 0 {
        Ok(SyncResult::Synced {
            added: total_added,
            removed: total_removed,
            modified: total_modified,
        })
    } else {
        Ok(SyncResult::NoChanges)
    }
}

/// Sync a native dep map against a manifest dep map.
/// Returns (added, modified, removed) counts.
fn sync_dep_map(
    native_deps: &std::collections::BTreeMap<String, crate::manifest::DependencyValue>,
    manifest_deps: &mut std::collections::BTreeMap<String, crate::manifest::DependencyValue>,
    driver_crates: &[String],
    file_type: NativeFileType,
) -> (usize, usize, usize) {
    let mut added = 0usize;
    let mut modified = 0usize;

    for (name, dep) in native_deps {
        if is_horus_internal(name) || driver_crates.contains(name) {
            continue;
        }
        if let Some(existing) = manifest_deps.get(name) {
            if !deps_equal(existing, dep) {
                manifest_deps.insert(name.clone(), dep.clone());
                modified += 1;
            }
        } else {
            manifest_deps.insert(name.clone(), dep.clone());
            added += 1;
        }
    }

    // Removals: deps in manifest but not in native, belonging to this ecosystem
    let mut removed = 0usize;
    let to_remove: Vec<String> = manifest_deps
        .iter()
        .filter(|(name, dep)| {
            if is_horus_internal(name) || driver_crates.contains(name) {
                return false;
            }
            let source = dep.effective_source();
            let belongs = match file_type {
                NativeFileType::Cargo => matches!(
                    source,
                    DepSource::CratesIo | DepSource::Path | DepSource::Git
                ),
                NativeFileType::Pyproject => matches!(source, DepSource::PyPI),
                NativeFileType::Cmake => matches!(source, DepSource::System),
            };
            belongs && !native_deps.contains_key(name.as_str())
        })
        .map(|(name, _)| name.clone())
        .collect();

    for name in to_remove {
        manifest_deps.remove(&name);
        removed += 1;
    }

    (added, modified, removed)
}

/// Sync changes from workspace member Cargo.toml files back to their horus.toml manifests.
///
/// Checks each member's `.horus/{name}/Cargo.toml` fingerprint. If modified,
/// parses changes and syncs to the member's `horus.toml`.
pub fn sync_workspace_members(
    project_dir: &std::path::Path,
    root_manifest: &HorusManifest,
    fingerprints: &mut crate::fingerprint::Fingerprints,
) -> Result<SyncResult> {
    let ws = match &root_manifest.workspace {
        Some(ws) => ws,
        None => return Ok(SyncResult::NoChanges),
    };

    let members = crate::manifest::resolve_workspace_members(ws, project_dir)?;
    let mut total_added = 0;
    let mut total_removed = 0;
    let mut total_modified = 0;

    for (rel_path, mut member_manifest) in members {
        let member_name = crate::cargo_gen::sanitize_cargo_name(&member_manifest.package.name);
        let fp_key = format!("{}/Cargo.toml", member_name);

        if !fingerprints.is_modified(&fp_key, project_dir) {
            continue;
        }

        let native_path = project_dir
            .join(".horus")
            .join(&member_name)
            .join("Cargo.toml");
        if !native_path.exists() {
            continue;
        }

        // Parse the member's native Cargo.toml
        let extracted = crate::commands::migrate::extract_cargo_all_deps(&native_path)?;
        let driver_crates = collect_driver_crates(&member_manifest);

        // Sync regular deps
        let (a, m, r) = sync_dep_map(
            &extracted.deps,
            &mut member_manifest.dependencies,
            &driver_crates,
            NativeFileType::Cargo,
        );
        // Sync dev deps
        let (da, dm, dr) = sync_dep_map(
            &extracted.dev_deps,
            &mut member_manifest.dev_dependencies,
            &driver_crates,
            NativeFileType::Cargo,
        );

        let changed = a + m + r + da + dm + dr;
        if changed > 0 {
            let member_toml = project_dir.join(&rel_path).join("horus.toml");
            member_manifest.save_to(&member_toml)?;
            total_added += a + da;
            total_removed += r + dr;
            total_modified += m + dm;

            log::info!(
                "Synced {} change(s) to {}/horus.toml",
                changed,
                rel_path.display()
            );
        }

        // Re-fingerprint after sync
        if let Ok(content) = std::fs::read_to_string(&native_path) {
            fingerprints.record(&fp_key, &content);
        }
    }

    if total_added > 0 || total_removed > 0 || total_modified > 0 {
        Ok(SyncResult::Synced {
            added: total_added,
            removed: total_removed,
            modified: total_modified,
        })
    } else {
        Ok(SyncResult::NoChanges)
    }
}

/// Compare two dependency values for equality (ignoring field ordering).
fn deps_equal(a: &DependencyValue, b: &DependencyValue) -> bool {
    // Compare versions
    if a.version() != b.version() {
        return false;
    }
    // Compare effective source
    if a.effective_source() != b.effective_source() {
        return false;
    }
    // Compare features (sorted)
    let mut af: Vec<&str> = a.features().iter().map(|s| s.as_str()).collect();
    let mut bf: Vec<&str> = b.features().iter().map(|s| s.as_str()).collect();
    af.sort();
    bf.sort();
    if af != bf {
        return false;
    }
    // Compare detailed fields if both are Detailed
    match (a, b) {
        (DependencyValue::Detailed(da), DependencyValue::Detailed(db)) => {
            da.optional == db.optional
                && da.path == db.path
                && da.git == db.git
                && da.branch == db.branch
                && da.tag == db.tag
                && da.rev == db.rev
        }
        _ => true, // Simple vs Detailed with same version/source/features is "equal enough"
    }
}

/// Collect crate names that are auto-generated from the `[hardware]`/`[drivers]` section.
///
/// Terra drivers no longer auto-resolve to crates — users add terra-horus as a
/// normal dependency. Only package drivers generate Cargo deps.
fn collect_driver_crates(manifest: &HorusManifest) -> Vec<String> {
    let mut crates = Vec::new();
    // Check both [hardware] and legacy [drivers]
    let all_drivers = manifest.hardware.values().chain(manifest.drivers.values());
    for driver_val in all_drivers {
        if let crate::manifest::DriverValue::Config(cfg) = driver_val {
            if let Some(package) = &cfg.package {
                crates.push(package.clone());
            }
            if let Some(crate_name) = &cfg.crate_name {
                crates.push(crate_name.clone());
            }
        }
    }
    crates
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deps_equal_same() {
        let a = DependencyValue::Simple("1.0".to_string());
        let b = DependencyValue::Simple("1.0".to_string());
        assert!(deps_equal(&a, &b));
    }

    #[test]
    fn deps_equal_different_version() {
        let a = DependencyValue::Simple("1.0".to_string());
        let b = DependencyValue::Simple("2.0".to_string());
        assert!(!deps_equal(&a, &b));
    }

    #[test]
    fn deps_equal_different_features() {
        use crate::manifest::DetailedDependency;
        let a = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            features: vec!["derive".to_string()],
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        let b = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            features: vec!["derive".to_string(), "alloc".to_string()],
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        assert!(!deps_equal(&a, &b));
    }

    #[test]
    fn deps_equal_different_source() {
        use crate::manifest::DetailedDependency;
        let a = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        let b = DependencyValue::Detailed(DetailedDependency {
            git: Some("https://github.com/org/repo".to_string()),
            source: Some(DepSource::Git),
            ..DetailedDependency::default()
        });
        assert!(!deps_equal(&a, &b));
    }

    #[test]
    fn deps_equal_features_order_independent() {
        use crate::manifest::DetailedDependency;
        let a = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            features: vec!["b".to_string(), "a".to_string()],
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        let b = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            features: vec!["a".to_string(), "b".to_string()],
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        assert!(deps_equal(&a, &b));
    }

    #[test]
    fn horus_internal_detection() {
        assert!(is_horus_internal("horus"));
        assert!(is_horus_internal("horus_core"));
        assert!(is_horus_internal("horus_library"));
        assert!(is_horus_internal("horus_macros"));
        assert!(is_horus_internal("horus_sys"));
        assert!(!is_horus_internal("horus-nav"));
        assert!(!is_horus_internal("serde"));
    }

    #[test]
    fn test_sync_from_native_no_changes_when_fingerprints_match() {
        use crate::manifest::{HorusManifest, PackageInfo, IgnoreConfig};

        let tmp = tempfile::tempdir().unwrap();
        let horus_dir = tmp.path().join(".horus");
        std::fs::create_dir_all(&horus_dir).unwrap();

        // Write a Cargo.toml in .horus/
        let cargo_content = r#"[package]
name = "test-project"
version = "0.1.0"

[dependencies]
serde = "1.0"
"#;
        std::fs::write(horus_dir.join("Cargo.toml"), cargo_content).unwrap();

        // Record fingerprint matching the file content
        let mut fingerprints = Fingerprints::default();
        fingerprints.record("Cargo.toml", cargo_content);
        fingerprints.save(tmp.path()).unwrap();

        let mut manifest = HorusManifest {
            package: PackageInfo {
                name: "test-project".to_string(),
                version: "0.1.0".to_string(),
                ..PackageInfo::default()
            },
            workspace: None,
            robot: None,
            dependencies: std::collections::BTreeMap::new(),
            dev_dependencies: std::collections::BTreeMap::new(),
            sim_dependencies: std::collections::BTreeMap::new(),
            hardware: std::collections::BTreeMap::new(),
            drivers: std::collections::BTreeMap::new(),
            sim_drivers: std::collections::BTreeMap::new(),
            scripts: std::collections::BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };

        let result = sync_from_native(
            tmp.path(),
            &mut manifest,
            NativeFileType::Cargo,
            &mut fingerprints,
        )
        .unwrap();

        assert!(matches!(result, SyncResult::NoChanges));
    }

    #[test]
    fn test_is_horus_internal_identifies_workspace_crates() {
        // All known horus internal crate names must return true
        let internal = [
            "horus",
            "horus_core",
            "horus_library",
            "horus_sys",
            "horus_macros",
            "horus-py",
            "horus-robotics",
        ];
        for name in &internal {
            assert!(
                is_horus_internal(name),
                "Expected '{}' to be recognized as horus internal",
                name
            );
        }

        // External crates must return false
        let external = ["serde", "tokio", "bevy", "rand", "anyhow", "horus-nav", "horus_slam"];
        for name in &external {
            assert!(
                !is_horus_internal(name),
                "Expected '{}' to NOT be recognized as horus internal",
                name
            );
        }
    }

    #[test]
    fn test_deps_equal_detects_version_change() {
        // Different versions => not equal
        let a = DependencyValue::Simple("1.0".to_string());
        let b = DependencyValue::Simple("1.1".to_string());
        assert!(!deps_equal(&a, &b));

        // Same versions => equal
        let c = DependencyValue::Simple("2.5".to_string());
        let d = DependencyValue::Simple("2.5".to_string());
        assert!(deps_equal(&c, &d));

        // Detailed with different versions => not equal
        use crate::manifest::DetailedDependency;
        let e = DependencyValue::Detailed(DetailedDependency {
            version: Some("0.8".to_string()),
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        let f = DependencyValue::Detailed(DetailedDependency {
            version: Some("0.9".to_string()),
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        assert!(!deps_equal(&e, &f));

        // Detailed with same version => equal
        let g = DependencyValue::Detailed(DetailedDependency {
            version: Some("3.0".to_string()),
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        let h = DependencyValue::Detailed(DetailedDependency {
            version: Some("3.0".to_string()),
            source: Some(DepSource::CratesIo),
            ..DetailedDependency::default()
        });
        assert!(deps_equal(&g, &h));

        // Simple vs Detailed with same version => equal (cross-variant)
        let i = DependencyValue::Simple("1.0".to_string());
        let j = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            ..DetailedDependency::default()
        });
        assert!(deps_equal(&i, &j));
    }
}
