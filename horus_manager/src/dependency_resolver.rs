// Dependency resolution with version conflict detection
// Solves dependency hell by finding compatible versions

use anyhow::{anyhow, bail, Result};
use colored::*;
use semver::{Version, VersionReq};
use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt;

pub type PackageName = String;

#[derive(Debug, Clone)]
pub struct ResolvedDependency {
    pub name: String,
    pub version: Version,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub enum DependencySource {
    #[default]
    Registry, // HORUS registry (default)
    Path(std::path::PathBuf), // Local filesystem path
    Git {
        // Git repository
        url: String,
        branch: Option<String>,
        tag: Option<String>,
        rev: Option<String>,
    },
    Pip {
        // PyPI package
        package_name: String, // The actual PyPI package name (e.g., "horus-robotics")
    },
    CratesIO, // Explicitly pinned to crates.io
    System,   // System package (apt, brew, etc.)
}

#[derive(Debug, Clone)]
pub struct DependencySpec {
    pub name: String,
    pub requirement: VersionReq,  // Semver requirement like "^1.2.3"
    pub source: DependencySource, // Where to get this dependency
    /// Optional target platform filter (e.g., "linux-x86_64")
    /// If set, this dependency is only required on the specified platform
    pub target: Option<String>,
}

impl fmt::Display for DependencySpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.name, self.requirement)
    }
}

impl DependencySpec {
    pub fn parse(spec: &str) -> Result<Self> {
        // Check for pip: prefix (e.g., "pip:horus-robotics" or "pip:numpy@^1.20")
        if let Some(pip_spec) = spec.strip_prefix("pip:") {
            return Self::parse_pip_spec(pip_spec);
        }

        // Check for cargo: prefix (e.g., "cargo:serde@1.0")
        if let Some(cargo_spec) = spec.strip_prefix("cargo:") {
            return Self::parse_cargo_spec(cargo_spec);
        }

        // Special case: horus_py always maps to pip:horus-robotics
        if spec == "horus_py" || spec.starts_with("horus_py@") {
            let requirement = if let Some(pos) = spec.find('@') {
                let constraint = &spec[pos + 1..];
                if constraint == "latest" || constraint == "*" {
                    VersionReq::STAR
                } else {
                    VersionReq::parse(constraint).unwrap_or(VersionReq::STAR)
                }
            } else {
                VersionReq::STAR
            };
            return Ok(Self {
                name: "horus_py".to_string(),
                requirement,
                source: DependencySource::Pip {
                    package_name: "horus-robotics".to_string(),
                },
                target: None,
            });
        }

        // Parse "name@constraint" or just "name"
        if let Some(pos) = spec.find('@') {
            let name = spec[..pos].to_string();
            let constraint = &spec[pos + 1..];

            // Parse version requirement
            let requirement = if constraint == "latest" || constraint == "*" {
                VersionReq::STAR
            } else {
                VersionReq::parse(constraint)
                    .map_err(|e| anyhow!("Invalid version constraint '{}': {}", constraint, e))?
            };

            Ok(Self {
                name,
                requirement,
                source: DependencySource::Registry,
                target: None,
            })
        } else {
            Ok(Self {
                name: spec.to_string(),
                requirement: VersionReq::STAR, // Any version
                source: DependencySource::Registry,
                target: None,
            })
        }
    }

    /// Parse a pip dependency spec (after stripping "pip:" prefix)
    fn parse_pip_spec(spec: &str) -> Result<Self> {
        if let Some(pos) = spec.find('@') {
            let pip_package = spec[..pos].to_string();
            let constraint = &spec[pos + 1..];

            let requirement = if constraint == "latest" || constraint == "*" {
                VersionReq::STAR
            } else {
                VersionReq::parse(constraint).map_err(|e| {
                    anyhow!("Invalid pip version constraint '{}': {}", constraint, e)
                })?
            };

            Ok(Self {
                name: pip_package.clone(),
                requirement,
                source: DependencySource::Pip {
                    package_name: pip_package,
                },
                target: None,
            })
        } else {
            Ok(Self {
                name: spec.to_string(),
                requirement: VersionReq::STAR,
                source: DependencySource::Pip {
                    package_name: spec.to_string(),
                },
                target: None,
            })
        }
    }

    /// Parse a crates.io dependency spec (after stripping "cargo:" prefix)
    fn parse_cargo_spec(spec: &str) -> Result<Self> {
        // Strip optional :features=... suffix (e.g., "serde@1.0:features=derive")
        let (spec_part, _features) = if let Some(pos) = spec.find(":features=") {
            (&spec[..pos], Some(&spec[pos + 10..]))
        } else {
            (spec, None)
        };

        if let Some(pos) = spec_part.find('@') {
            let name = spec_part[..pos].to_string();
            let constraint = &spec_part[pos + 1..];
            let requirement = if constraint == "latest" || constraint == "*" {
                VersionReq::STAR
            } else {
                VersionReq::parse(constraint).map_err(|e| {
                    anyhow!("Invalid cargo version constraint '{}': {}", constraint, e)
                })?
            };
            Ok(Self {
                name,
                requirement,
                source: DependencySource::CratesIO,
                target: None,
            })
        } else {
            Ok(Self {
                name: spec_part.to_string(),
                requirement: VersionReq::STAR,
                source: DependencySource::CratesIO,
                target: None,
            })
        }
    }
}

/// Package metadata provider
pub trait PackageProvider {
    /// Get all available versions for a package
    fn get_available_versions(&self, package: &str) -> Result<Vec<Version>>;

    /// Get dependencies for a specific package version
    fn get_dependencies(&self, package: &str, version: &Version) -> Result<Vec<DependencySpec>>;
}

/// Dependency resolver with conflict detection
pub struct DependencyResolver<'a> {
    provider: &'a dyn PackageProvider,
    resolved: HashMap<PackageName, Version>,
    requirements: HashMap<PackageName, Vec<VersionReq>>,
}

impl<'a> DependencyResolver<'a> {
    pub fn new(provider: &'a dyn PackageProvider) -> Self {
        Self {
            provider,
            resolved: HashMap::new(),
            requirements: HashMap::new(),
        }
    }

    /// Resolve dependencies starting from root requirements
    pub fn resolve(&mut self, root_deps: Vec<DependencySpec>) -> Result<Vec<ResolvedDependency>> {
        println!("Resolving dependencies...");

        // Collect all requirements via BFS
        let mut queue: VecDeque<(String, DependencySpec)> = VecDeque::new();

        for dep in root_deps {
            println!(
                "  {} Root dependency: {} {}",
                "".cyan(),
                dep.name,
                dep.requirement
            );
            queue.push_back(("root".to_string(), dep));
        }

        // BFS to collect all requirements
        let mut visited = HashSet::new();

        while let Some((_parent, dep)) = queue.pop_front() {
            let key = format!("{}@{}", dep.name, dep.requirement);
            if visited.contains(&key) {
                continue;
            }
            visited.insert(key);

            // Add requirement
            self.requirements
                .entry(dep.name.clone())
                .or_default()
                .push(dep.requirement.clone());

            // Temporarily resolve to latest matching version to fetch its dependencies
            if let Ok(versions) = self.provider.get_available_versions(&dep.name) {
                if let Some(version) = self.find_best_version(&dep.name, &versions) {
                    // Get transitive dependencies
                    if let Ok(transitive_deps) = self.provider.get_dependencies(&dep.name, &version)
                    {
                        for trans_dep in transitive_deps {
                            queue.push_back((dep.name.clone(), trans_dep));
                        }
                    }
                }
            }
        }

        // Now resolve all packages
        println!(
            "  {} Found {} unique packages",
            "".cyan(),
            self.requirements.len()
        );

        // Clone requirements to avoid borrow checker issues
        let requirements_snapshot: Vec<(String, Vec<_>)> = self
            .requirements
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        for (package, reqs) in requirements_snapshot {
            self.resolve_package(&package, &reqs)?;
        }

        // Convert to result format
        let mut result: Vec<ResolvedDependency> = self
            .resolved
            .iter()
            .map(|(name, version)| ResolvedDependency {
                name: name.clone(),
                version: version.clone(),
            })
            .collect();

        result.sort_by(|a, b| a.name.cmp(&b.name));

        println!(
            "  {} Successfully resolved {} packages!",
            "".green(),
            result.len()
        );
        for dep in &result {
            println!("    • {} v{}", dep.name.cyan(), dep.version);
        }

        Ok(result)
    }

    fn resolve_package(&mut self, package: &str, requirements: &[VersionReq]) -> Result<Version> {
        // Check if already resolved
        if let Some(version) = self.resolved.get(package) {
            // Verify it satisfies all requirements
            for req in requirements {
                if !req.matches(version) {
                    bail!(
                        "Version conflict for {}: resolved v{} doesn't satisfy requirement {}",
                        package,
                        version,
                        req
                    );
                }
            }
            return Ok(version.clone());
        }

        // Get available versions
        let versions = self
            .provider
            .get_available_versions(package)
            .map_err(|e| anyhow!("Cannot fetch versions for {}: {}", package, e))?;

        if versions.is_empty() {
            bail!("No versions available for package: {}", package);
        }

        // Find compatible version
        let version = self.find_best_version_with_requirements(package, &versions, requirements)
            .ok_or_else(|| {
                let req_strs: Vec<String> = requirements.iter().map(|r| r.to_string()).collect();
                anyhow!("Cannot find compatible version for {}: requirements are {:?}, available versions: {:?}",
                    package, req_strs, versions)
            })?;

        println!(
            "    {} {} v{} (satisfies {} constraints)",
            "".green(),
            package,
            version,
            requirements.len()
        );

        self.resolved.insert(package.to_string(), version.clone());
        Ok(version)
    }

    fn find_best_version(&self, package: &str, versions: &[Version]) -> Option<Version> {
        let requirements = self.requirements.get(package)?;
        self.find_best_version_with_requirements(package, versions, requirements)
    }

    fn find_best_version_with_requirements(
        &self,
        _package: &str,
        versions: &[Version],
        requirements: &[VersionReq],
    ) -> Option<Version> {
        // Find the highest version that satisfies all requirements
        let mut candidates: Vec<Version> = versions
            .iter()
            .filter(|v| requirements.iter().all(|req| req.matches(v)))
            .cloned()
            .collect();

        candidates.sort();
        candidates.pop() // Return highest version
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ========================================================================
    // DependencySpec::parse tests
    // ========================================================================

    #[test]
    fn parse_name_only() {
        let spec = DependencySpec::parse("my_package").unwrap();
        assert_eq!(spec.name, "my_package");
        assert_eq!(spec.requirement, VersionReq::STAR);
        assert_eq!(spec.source, DependencySource::Registry);
        assert!(spec.target.is_none());
    }

    #[test]
    fn parse_name_at_version() {
        let spec = DependencySpec::parse("sensor_driver@^1.2.3").unwrap();
        assert_eq!(spec.name, "sensor_driver");
        assert!(spec.requirement.matches(&Version::new(1, 3, 0)));
        assert!(!spec.requirement.matches(&Version::new(2, 0, 0)));
        assert_eq!(spec.source, DependencySource::Registry);
    }

    #[test]
    fn parse_name_at_latest() {
        let spec = DependencySpec::parse("cam@latest").unwrap();
        assert_eq!(spec.name, "cam");
        assert_eq!(spec.requirement, VersionReq::STAR);
    }

    #[test]
    fn parse_name_at_star() {
        let spec = DependencySpec::parse("foo@*").unwrap();
        assert_eq!(spec.requirement, VersionReq::STAR);
    }

    #[test]
    fn parse_pip_prefix_name_only() {
        let spec = DependencySpec::parse("pip:numpy").unwrap();
        assert_eq!(spec.name, "numpy");
        assert_eq!(spec.requirement, VersionReq::STAR);
        assert_eq!(
            spec.source,
            DependencySource::Pip {
                package_name: "numpy".to_string()
            }
        );
    }

    #[test]
    fn parse_pip_prefix_with_version() {
        let spec = DependencySpec::parse("pip:torch@^2.0").unwrap();
        assert_eq!(spec.name, "torch");
        assert!(spec.requirement.matches(&Version::new(2, 1, 0)));
        assert!(!spec.requirement.matches(&Version::new(3, 0, 0)));
        assert_eq!(
            spec.source,
            DependencySource::Pip {
                package_name: "torch".to_string()
            }
        );
    }

    #[test]
    fn parse_horus_py_special_case() {
        let spec = DependencySpec::parse("horus_py").unwrap();
        assert_eq!(spec.name, "horus_py");
        assert_eq!(spec.requirement, VersionReq::STAR);
        assert_eq!(
            spec.source,
            DependencySource::Pip {
                package_name: "horus-robotics".to_string()
            }
        );
    }

    #[test]
    fn parse_horus_py_at_version() {
        let spec = DependencySpec::parse("horus_py@^1.0").unwrap();
        assert_eq!(spec.name, "horus_py");
        assert_eq!(
            spec.source,
            DependencySource::Pip {
                package_name: "horus-robotics".to_string()
            }
        );
    }

    #[test]
    fn parse_invalid_version_constraint() {
        let result = DependencySpec::parse("pkg@not_a_version");
        assert!(result.is_err());
    }

    // ========================================================================
    // DependencySpec Display
    // ========================================================================

    #[test]
    fn display_format() {
        let spec = DependencySpec::parse("my_pkg@^1.0").unwrap();
        let displayed = format!("{}", spec);
        assert!(displayed.contains("my_pkg"));
    }

    // ========================================================================
    // DependencyResolver tests (with mock provider)
    // ========================================================================

    /// Mock package provider for testing
    struct MockProvider {
        packages: HashMap<String, Vec<Version>>,
        dependencies: HashMap<(String, Version), Vec<DependencySpec>>,
    }

    impl MockProvider {
        fn new() -> Self {
            Self {
                packages: HashMap::new(),
                dependencies: HashMap::new(),
            }
        }

        fn add_package(&mut self, name: &str, versions: Vec<&str>) {
            let parsed: Vec<Version> = versions
                .iter()
                .map(|v| Version::parse(v).unwrap())
                .collect();
            self.packages.insert(name.to_string(), parsed);
        }

        fn add_deps(&mut self, name: &str, version: &str, deps: Vec<(&str, &str)>) {
            let ver = Version::parse(version).unwrap();
            let dep_specs: Vec<DependencySpec> = deps
                .into_iter()
                .map(|(dep_name, req)| DependencySpec {
                    name: dep_name.to_string(),
                    requirement: VersionReq::parse(req).unwrap(),
                    source: DependencySource::Registry,
                    target: None,
                })
                .collect();
            self.dependencies.insert((name.to_string(), ver), dep_specs);
        }
    }

    impl PackageProvider for MockProvider {
        fn get_available_versions(&self, package: &str) -> Result<Vec<Version>> {
            self.packages
                .get(package)
                .cloned()
                .ok_or_else(|| anyhow!("Package '{}' not found", package))
        }

        fn get_dependencies(
            &self,
            package: &str,
            version: &Version,
        ) -> Result<Vec<DependencySpec>> {
            Ok(self
                .dependencies
                .get(&(package.to_string(), version.clone()))
                .cloned()
                .unwrap_or_default())
        }
    }

    #[test]
    fn resolve_single_package() {
        let mut provider = MockProvider::new();
        provider.add_package("sensor", vec!["1.0.0", "1.1.0", "2.0.0"]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("sensor@^1.0").unwrap()];
        let result = resolver.resolve(deps).unwrap();

        assert_eq!(result.len(), 1);
        assert_eq!(result[0].name, "sensor");
        // Should pick highest compatible version (1.1.0, not 2.0.0)
        assert_eq!(result[0].version, Version::new(1, 1, 0));
    }

    #[test]
    fn resolve_multiple_packages() {
        let mut provider = MockProvider::new();
        provider.add_package("sensor", vec!["1.0.0"]);
        provider.add_package("motor", vec!["2.0.0", "2.1.0"]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![
            DependencySpec::parse("sensor@^1.0").unwrap(),
            DependencySpec::parse("motor@^2.0").unwrap(),
        ];
        let result = resolver.resolve(deps).unwrap();

        assert_eq!(result.len(), 2);
        // Results are sorted by name
        assert_eq!(result[0].name, "motor");
        assert_eq!(result[0].version, Version::new(2, 1, 0));
        assert_eq!(result[1].name, "sensor");
    }

    #[test]
    fn resolve_transitive_dependencies() {
        let mut provider = MockProvider::new();
        provider.add_package("app", vec!["1.0.0"]);
        provider.add_package("middleware", vec!["1.0.0", "1.1.0"]);
        provider.add_package("core_lib", vec!["0.5.0", "1.0.0"]);

        // app@1.0.0 depends on middleware@^1.0
        provider.add_deps("app", "1.0.0", vec![("middleware", "^1.0")]);
        // middleware@1.1.0 depends on core_lib@^0.5
        provider.add_deps("middleware", "1.1.0", vec![("core_lib", "^0.5")]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("app@^1.0").unwrap()];
        let result = resolver.resolve(deps).unwrap();

        // Should resolve all three: app, middleware, and core_lib
        assert_eq!(result.len(), 3);
        let names: Vec<&str> = result.iter().map(|d| d.name.as_str()).collect();
        assert!(names.contains(&"app"));
        assert!(names.contains(&"middleware"));
        assert!(names.contains(&"core_lib"));
    }

    #[test]
    fn resolve_missing_package_errors() {
        let provider = MockProvider::new(); // empty

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("nonexistent").unwrap()];
        let result = resolver.resolve(deps);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("nonexistent"),
            "Error should mention the missing package: {}",
            err
        );
    }

    #[test]
    fn resolve_no_compatible_version() {
        let mut provider = MockProvider::new();
        provider.add_package("sensor", vec!["1.0.0", "1.1.0"]);

        let mut resolver = DependencyResolver::new(&provider);
        // Require >= 2.0 but only 1.x available
        let deps = vec![DependencySpec::parse("sensor@>=2.0.0").unwrap()];
        let result = resolver.resolve(deps);
        assert!(result.is_err());
    }

    #[test]
    fn resolve_picks_highest_compatible() {
        let mut provider = MockProvider::new();
        provider.add_package("lib", vec!["1.0.0", "1.2.0", "1.5.0", "2.0.0"]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("lib@>=1.0, <2.0").unwrap()];
        let result = resolver.resolve(deps).unwrap();

        assert_eq!(result[0].version, Version::new(1, 5, 0));
    }

    #[test]
    fn resolve_circular_dependency_does_not_loop() {
        let mut provider = MockProvider::new();
        provider.add_package("a", vec!["1.0.0"]);
        provider.add_package("b", vec!["1.0.0"]);

        // a depends on b, b depends on a (circular)
        provider.add_deps("a", "1.0.0", vec![("b", "^1.0")]);
        provider.add_deps("b", "1.0.0", vec![("a", "^1.0")]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("a@^1.0").unwrap()];
        // The visited set in BFS should prevent infinite loops
        let result = resolver.resolve(deps).unwrap();

        assert_eq!(result.len(), 2);
        let names: Vec<&str> = result.iter().map(|d| d.name.as_str()).collect();
        assert!(names.contains(&"a"));
        assert!(names.contains(&"b"));
    }

    #[test]
    fn resolve_diamond_dependency() {
        let mut provider = MockProvider::new();
        provider.add_package("app", vec!["1.0.0"]);
        provider.add_package("left", vec!["1.0.0"]);
        provider.add_package("right", vec!["1.0.0"]);
        provider.add_package("base", vec!["1.0.0", "1.1.0", "1.2.0"]);

        // app → left, right
        provider.add_deps("app", "1.0.0", vec![("left", "^1.0"), ("right", "^1.0")]);
        // left → base@^1.0
        provider.add_deps("left", "1.0.0", vec![("base", "^1.0")]);
        // right → base@>=1.1
        provider.add_deps("right", "1.0.0", vec![("base", ">=1.1.0")]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("app@^1.0").unwrap()];
        let result = resolver.resolve(deps).unwrap();

        // base should be resolved to 1.2.0 (satisfies both ^1.0 and >=1.1)
        let base = result.iter().find(|d| d.name == "base").unwrap();
        assert_eq!(base.version, Version::new(1, 2, 0));
    }

    #[test]
    fn resolve_empty_deps_returns_empty() {
        let provider = MockProvider::new();
        let mut resolver = DependencyResolver::new(&provider);
        let result = resolver.resolve(vec![]).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn resolve_star_requirement() {
        let mut provider = MockProvider::new();
        provider.add_package("any_ver", vec!["0.1.0", "1.0.0", "99.0.0"]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("any_ver").unwrap()]; // star requirement
        let result = resolver.resolve(deps).unwrap();

        // Should pick highest
        assert_eq!(result[0].version, Version::new(99, 0, 0));
    }

    // ========================================================================
    // DependencySource tests
    // ========================================================================

    #[test]
    fn dependency_source_default_is_registry() {
        let source: DependencySource = Default::default();
        assert_eq!(source, DependencySource::Registry);
    }

    #[test]
    fn dependency_source_debug_and_clone() {
        let src = DependencySource::Git {
            url: "https://example.com".to_string(),
            branch: Some("main".to_string()),
            tag: None,
            rev: None,
        };
        let cloned = src.clone();
        assert_eq!(src, cloned);
        let _ = format!("{:?}", src); // should not panic
    }

    // ========================================================================
    // Source pinning tests
    // ========================================================================

    #[test]
    fn parse_cargo_prefix_name_only() {
        let spec = DependencySpec::parse("cargo:serde").unwrap();
        assert_eq!(spec.name, "serde");
        assert_eq!(spec.requirement, VersionReq::STAR);
        assert_eq!(spec.source, DependencySource::CratesIO);
    }

    #[test]
    fn parse_cargo_prefix_with_version() {
        let spec = DependencySpec::parse("cargo:tokio@^1.0").unwrap();
        assert_eq!(spec.name, "tokio");
        assert!(spec.requirement.matches(&Version::new(1, 35, 0)));
        assert!(!spec.requirement.matches(&Version::new(2, 0, 0)));
        assert_eq!(spec.source, DependencySource::CratesIO);
    }

    #[test]
    fn parse_cargo_prefix_with_features() {
        let spec = DependencySpec::parse("cargo:serde@1.0:features=derive").unwrap();
        assert_eq!(spec.name, "serde");
        assert_eq!(spec.source, DependencySource::CratesIO);
    }

    // ========================================================================
    // Diamond conflict detection tests
    // ========================================================================

    #[test]
    fn resolve_diamond_conflict_incompatible() {
        // A -> C@^1.0, B -> C@^2.0 — no version of C satisfies both
        let mut provider = MockProvider::new();
        provider.add_package("app", vec!["1.0.0"]);
        provider.add_package("left", vec!["1.0.0"]);
        provider.add_package("right", vec!["1.0.0"]);
        provider.add_package("base", vec!["1.0.0", "1.5.0", "2.0.0", "2.1.0"]);

        // app depends on left and right
        provider.add_deps("app", "1.0.0", vec![("left", "^1.0"), ("right", "^1.0")]);
        // left requires base@^1.0 (1.x only)
        provider.add_deps("left", "1.0.0", vec![("base", "^1.0")]);
        // right requires base@^2.0 (2.x only)
        provider.add_deps("right", "1.0.0", vec![("base", "^2.0")]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("app@^1.0").unwrap()];
        let result = resolver.resolve(deps);

        assert!(result.is_err(), "incompatible diamond should error");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("base"),
            "error should mention the conflicting package 'base': {}",
            err
        );
    }

    #[test]
    fn resolve_diamond_compatible_picks_highest() {
        // A -> C@^1.0, B -> C@>=1.2 — compatible, should pick highest match
        let mut provider = MockProvider::new();
        provider.add_package("app", vec!["1.0.0"]);
        provider.add_package("left", vec!["1.0.0"]);
        provider.add_package("right", vec!["1.0.0"]);
        provider.add_package("base", vec!["1.0.0", "1.2.0", "1.5.0"]);

        provider.add_deps("app", "1.0.0", vec![("left", "^1.0"), ("right", "^1.0")]);
        provider.add_deps("left", "1.0.0", vec![("base", "^1.0")]);
        provider.add_deps("right", "1.0.0", vec![("base", ">=1.2.0")]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![DependencySpec::parse("app@^1.0").unwrap()];
        let result = resolver.resolve(deps).unwrap();

        let base = result.iter().find(|d| d.name == "base").unwrap();
        // Must satisfy both ^1.0 and >=1.2.0 — highest is 1.5.0
        assert_eq!(base.version, Version::new(1, 5, 0));
    }

    #[test]
    fn resolve_direct_conflict_two_root_deps() {
        // Root asks for pkg@^1.0 AND pkg@^2.0 directly — conflict
        let mut provider = MockProvider::new();
        provider.add_package("pkg", vec!["1.0.0", "1.5.0", "2.0.0"]);

        let mut resolver = DependencyResolver::new(&provider);
        let deps = vec![
            DependencySpec {
                name: "pkg".to_string(),
                requirement: VersionReq::parse("^1.0").unwrap(),
                source: DependencySource::Registry,
                target: None,
            },
            DependencySpec {
                name: "pkg".to_string(),
                requirement: VersionReq::parse("^2.0").unwrap(),
                source: DependencySource::Registry,
                target: None,
            },
        ];
        let result = resolver.resolve(deps);

        assert!(result.is_err(), "direct conflict should error");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("pkg"),
            "error should mention conflicting package: {}",
            err
        );
    }
}
