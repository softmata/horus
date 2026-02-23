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

        // Special case: horus_py always maps to pip:horus-robotics
        if spec == "horus_py" || spec.starts_with("horus_py@") {
            return Ok(Self {
                name: "horus_py".to_string(),
                requirement: VersionReq::STAR,
                source: DependencySource::Pip {
                    package_name: "horus-robotics".to_string(),
                },
                target: None,
            });
        }

        // Parse "name@constraint" or just "name" (simple string format from YAML list)
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

    /// Parse from structured YAML (supports path, git, pip, etc.)
    pub fn from_yaml_value(name: String, value: &serde_yaml::Value) -> Result<Self> {
        use serde_yaml::Value;

        // Special case: horus_py always maps to pip:horus-robotics
        if name == "horus_py" {
            return Ok(Self {
                name: "horus_py".to_string(),
                requirement: VersionReq::STAR,
                source: DependencySource::Pip {
                    package_name: "horus-robotics".to_string(),
                },
                target: None,
            });
        }

        match value {
            // Simple version string: "package: '1.0.0'" or "package: '*'"
            Value::String(version_str) => {
                let requirement = if version_str == "latest" || version_str == "*" {
                    VersionReq::STAR
                } else {
                    VersionReq::parse(version_str)
                        .map_err(|e| anyhow!("Invalid version '{}': {}", version_str, e))?
                };
                Ok(Self {
                    name,
                    requirement,
                    source: DependencySource::Registry,
                    target: None,
                })
            }

            // Structured dependency: path, git, pip, or registry
            Value::Mapping(map) => {
                // Parse optional target platform
                let dep_target = map
                    .get(Value::String("target".to_string()))
                    .and_then(|v| v.as_str().map(String::from));

                // Check for path dependency
                if let Some(Value::String(path_str)) = map.get(Value::String("path".to_string())) {
                    let path = std::path::PathBuf::from(path_str);
                    Ok(Self {
                        name,
                        requirement: VersionReq::STAR, // Path deps don't use versions
                        source: DependencySource::Path(path),
                        target: dep_target,
                    })
                }
                // Check for git dependency
                else if let Some(Value::String(git_url)) =
                    map.get(Value::String("git".to_string()))
                {
                    let branch = map
                        .get(Value::String("branch".to_string()))
                        .and_then(|v| v.as_str().map(String::from));
                    let tag = map
                        .get(Value::String("tag".to_string()))
                        .and_then(|v| v.as_str().map(String::from));
                    let rev = map
                        .get(Value::String("rev".to_string()))
                        .and_then(|v| v.as_str().map(String::from));

                    Ok(Self {
                        name,
                        requirement: VersionReq::STAR, // Git deps don't use versions
                        source: DependencySource::Git {
                            url: git_url.clone(),
                            branch,
                            tag,
                            rev,
                        },
                        target: dep_target,
                    })
                }
                // Check for pip dependency: { pip: "package-name" } or { pip: "package-name", version: "^1.0" }
                else if let Some(Value::String(pip_package)) =
                    map.get(Value::String("pip".to_string()))
                {
                    let requirement = if let Some(Value::String(version_str)) =
                        map.get(Value::String("version".to_string()))
                    {
                        VersionReq::parse(version_str)
                            .map_err(|e| anyhow!("Invalid pip version '{}': {}", version_str, e))?
                    } else {
                        VersionReq::STAR
                    };
                    Ok(Self {
                        name,
                        requirement,
                        source: DependencySource::Pip {
                            package_name: pip_package.clone(),
                        },
                        target: dep_target,
                    })
                }
                // Registry with explicit version
                else if let Some(Value::String(version_str)) =
                    map.get(Value::String("version".to_string()))
                {
                    let requirement = VersionReq::parse(version_str)
                        .map_err(|e| anyhow!("Invalid version '{}': {}", version_str, e))?;
                    Ok(Self {
                        name,
                        requirement,
                        source: DependencySource::Registry,
                        target: dep_target,
                    })
                } else {
                    Err(anyhow!("Invalid dependency specification for '{}'", name))
                }
            }

            _ => Err(anyhow!("Invalid dependency format for '{}'", name)),
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
