use super::*;
use super::helpers::*;

impl RegistryClient {
    // Install a package to a specific target (used by install_to_target)
    // Returns the actual installed version
    pub fn install(&self, package_name: &str, version: Option<&str>) -> Result<String> {
        log::debug!("installing package: {}", package_name);
        // Default: auto-detect global/local
        use crate::workspace;
        let target = workspace::detect_or_select_workspace(true)?;
        self.install_to_target(package_name, version, target)
    }

    // Install a package from registry to a specific target
    // Returns the actual installed version
    pub fn install_to_target(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        log::debug!("installing package: {} to target: {:?}", package_name, target);
        // Detect package source
        let source = self.detect_package_source(package_name)?;
        log::debug!("resolved package source for {}: {:?}", package_name, source);

        match source {
            PackageSource::Registry => self.install_from_registry(package_name, version, target),
            PackageSource::PyPI => self.install_from_pypi(package_name, version, target),
            PackageSource::CratesIO => self.install_from_cratesio(package_name, version, target),
            PackageSource::System => Err(anyhow!(
                "System packages not supported via horus pkg install"
            )),
            PackageSource::Path { .. } => Err(anyhow!(
                "Path dependencies must be specified in horus.yaml.\n\
                     Use 'horus run' to install dependencies from horus.yaml."
            )),
        }
    }

    fn detect_package_source(&self, package_name: &str) -> Result<PackageSource> {
        // Scoped packages (@org/name) are always from HORUS registry
        if package_name.starts_with('@') {
            return Ok(PackageSource::Registry);
        }

        // Check if it's a HORUS package
        if package_name.starts_with("horus") {
            return Ok(PackageSource::Registry);
        }

        // Try HORUS registry first (URL encode for safety)
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}", self.base_url, encoded_name);
        if let Ok(response) = self.client.get(&url).send() {
            if response.status().is_success() {
                return Ok(PackageSource::Registry);
            }
        }

        // Check BOTH PyPI and crates.io to detect ambiguity
        let in_pypi = self.check_pypi_exists(package_name);
        let in_crates = self.check_crates_exists(package_name);

        // Handle ambiguity - package exists in both registries
        if in_pypi && in_crates {
            println!(
                "\n{} Package '{}' found in BOTH PyPI and crates.io",
                "[WARNING]".yellow(),
                package_name.green()
            );
            return self.prompt_package_source_choice(package_name);
        }

        // Package only in crates.io
        if in_crates {
            return Ok(PackageSource::CratesIO);
        }

        // Package only in PyPI or not found (default to PyPI)
        Ok(PackageSource::PyPI)
    }

    fn check_pypi_exists(&self, package_name: &str) -> bool {
        // Check PyPI API
        let pypi_url = format!("{}/{}/json", crate::config::PYPI_API_URL, package_name);
        if let Ok(response) = self.client.get(&pypi_url).send() {
            return response.status().is_success();
        }
        false
    }

    fn check_crates_exists(&self, package_name: &str) -> bool {
        // Check crates.io API
        let crates_url = format!("{}/{}", crate::config::CRATES_IO_API_URL, package_name);
        if let Ok(response) = self
            .client
            .get(&crates_url)
            .header("User-Agent", "horus-pkg-manager")
            .send()
        {
            return response.status().is_success();
        }
        false
    }

    fn prompt_package_source_choice(&self, _package_name: &str) -> Result<PackageSource> {
        use std::io::{self, Write};

        println!("\nWhich package source do you want to use?");
        println!("  [1] {} PyPI (Python package)", "[PYTHON]".cyan());
        println!("  [2] {} crates.io (Rust binary)", "[RUST]".cyan());
        println!("  [3] {} Cancel installation", "[FAIL]".red());

        print!("\nChoice [1-3]: ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;

        match input.trim() {
            "1" => {
                println!("   Using PyPI (Python)");
                Ok(PackageSource::PyPI)
            }
            "2" => {
                println!("   Using crates.io (Rust)");
                Ok(PackageSource::CratesIO)
            }
            "3" => {
                bail!("installation cancelled by user")
            }
            _ => {
                println!("Invalid choice, defaulting to PyPI");
                Ok(PackageSource::PyPI)
            }
        }
    }

    // Install a dependency from DependencySpec (supports path and registry)
    pub fn install_dependency_spec(
        &self,
        spec: &crate::dependency_resolver::DependencySpec,
        target: crate::workspace::InstallTarget,
        base_dir: Option<&Path>,
    ) -> Result<()> {
        use crate::dependency_resolver::DependencySource;

        match &spec.source {
            DependencySource::Registry => {
                // For registry dependencies, use version from requirement if specific
                let version_str = if spec.requirement.to_string() != "*" {
                    Some(spec.requirement.to_string())
                } else {
                    None
                };
                self.install_from_registry(&spec.name, version_str.as_deref(), target)
                    .map(|_| ()) // Ignore version for dependency spec
            }
            DependencySource::Path(path) => {
                self.install_from_path(&spec.name, path, target, base_dir)
                    .map(|_| ()) // Ignore version for dependency spec
            }
            DependencySource::Git {
                url,
                branch,
                tag,
                rev,
            } => {
                // Git dependencies are handled by horus run - skip for pkg install
                // They are cloned to ~/.horus/cache/git_* and used as path deps
                eprintln!(
                    "  {} Git dependency '{}' from {} - use 'horus run' to clone automatically",
                    "ℹ".cyan(),
                    spec.name,
                    url
                );
                if let Some(b) = branch {
                    eprintln!("      Branch: {}", b);
                } else if let Some(t) = tag {
                    eprintln!("      Tag: {}", t);
                } else if let Some(r) = rev {
                    eprintln!("      Rev: {}", r);
                }
                Ok(())
            }
            DependencySource::Pip { package_name } => {
                // Install from PyPI
                let version_str = if spec.requirement.to_string() != "*" {
                    Some(spec.requirement.to_string())
                } else {
                    None
                };
                self.install_from_pypi(package_name, version_str.as_deref(), target)
                    .map(|_| ()) // Ignore version for dependency spec
            }
        }
    }

    pub(crate) fn install_from_registry(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        log::debug!("installing {} from HORUS registry (version: {:?})", package_name, version);
        let spinner = progress::robot_download_spinner(&format!(
            "Downloading {} from HORUS registry...",
            package_name
        ));

        let version_str = version.unwrap_or("latest");
        // URL-encode scoped package names for API calls
        let encoded_name = url_encode_package_name(package_name);
        let url = format!(
            "{}/api/packages/{}/{}/download",
            self.base_url, encoded_name, version_str
        );

        // Download package
        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            // Check if the package was yanked (410 Gone)
            if response.status() == reqwest::StatusCode::GONE {
                let body = response.text().unwrap_or_default();
                if let Ok(json) = serde_json::from_str::<serde_json::Value>(&body) {
                    let reason = json
                        .get("reason")
                        .and_then(|v| v.as_str())
                        .unwrap_or("No reason given");
                    return Err(anyhow!(
                        "Package {} has been yanked: {}",
                        package_name,
                        reason
                    ));
                }
            }
            return Err(anyhow!("Package not found: {}", package_name));
        }

        // Check for package signature header
        let pkg_signature = response
            .headers()
            .get("x-horus-signature")
            .and_then(|v| v.to_str().ok())
            .map(String::from);

        let bytes = response.bytes()?;

        // Calculate checksum
        let mut hasher = Sha256::new();
        hasher.update(&bytes);
        let checksum = format!("{:x}", hasher.finalize());

        // Convert scoped package name to safe path (e.g., @org/pkg -> org--pkg)
        let safe_pkg_name = package_name_to_path(package_name);

        // Determine installation directory based on target
        use crate::workspace::InstallTarget;
        let global_cache = crate::paths::cache_dir()?;

        let (install_dir, install_type, local_packages_dir) = match &target {
            InstallTarget::Global => {
                // Force global installation
                fs::create_dir_all(&global_cache)?;
                let current_local = PathBuf::from(".horus/packages");
                (global_cache.clone(), "global", Some(current_local))
            }
            InstallTarget::Local(workspace_path) => {
                // Install to specific workspace
                let local_packages = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local_packages)?;

                // Check if any version exists in global cache (use safe name)
                let has_global_versions = check_global_versions(&global_cache, &safe_pkg_name)?;

                if has_global_versions {
                    // Install to global and symlink
                    fs::create_dir_all(&global_cache)?;
                    (global_cache.clone(), "global", Some(local_packages))
                } else {
                    // Install locally
                    (local_packages.clone(), "local", None)
                }
            }
        };

        // Create package directory with version
        let tar = GzDecoder::new(&bytes[..]);
        let mut archive = Archive::new(tar);

        // Extract to temporary location first to detect version (use safe name)
        let temp_dir = std::env::temp_dir().join(format!("horus_pkg_{}", safe_pkg_name));
        fs::create_dir_all(&temp_dir)?;
        archive.unpack(&temp_dir)?;

        // Get actual version from package info (for "latest" downloads)
        let actual_version = if version_str == "latest" {
            detect_package_version(&temp_dir).unwrap_or_else(|| version_str.to_string())
        } else {
            version_str.to_string()
        };

        // Move to final location with version info (use safe path name)
        let package_dir = if install_type == "global" {
            install_dir.join(format!("{}@{}", safe_pkg_name, actual_version))
        } else {
            install_dir.join(&safe_pkg_name)
        };

        // Remove existing if present
        if package_dir.exists() {
            fs::remove_dir_all(&package_dir)?;
        }
        fs::create_dir_all(&package_dir)?;

        // Move from temp to final location
        copy_dir_all(&temp_dir, &package_dir)?;
        fs::remove_dir_all(&temp_dir)?;

        // Create metadata.json for tracking
        let metadata = PackageMetadata {
            name: package_name.to_string(),
            version: actual_version.clone(),
            checksum: Some(checksum),
        };

        let metadata_path = package_dir.join("metadata.json");
        fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        // If installed to global, create symlink in local workspace
        if install_type == "global" {
            if let Some(local_pkg_dir) = local_packages_dir {
                fs::create_dir_all(&local_pkg_dir)?;
                let local_link = local_pkg_dir.join(&safe_pkg_name);

                // Remove existing symlink/dir if present
                if local_link.exists() || local_link.symlink_metadata().is_ok() {
                    #[cfg(unix)]
                    {
                        if local_link.symlink_metadata()?.is_symlink() {
                            fs::remove_file(&local_link)?;
                        } else {
                            fs::remove_dir_all(&local_link)?;
                        }
                    }
                    #[cfg(windows)]
                    {
                        if local_link.is_dir() {
                            fs::remove_dir_all(&local_link)?;
                        } else {
                            fs::remove_file(&local_link)?;
                        }
                    }
                }

                // Create symlink
                #[cfg(unix)]
                std::os::unix::fs::symlink(&package_dir, &local_link)?;
                #[cfg(windows)]
                std::os::windows::fs::symlink_dir(&package_dir, &local_link)?;

                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!(
                    "   {} Linked: {} -> {}",
                    "".dimmed(),
                    local_link.display(),
                    package_dir.display()
                );
            } else {
                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!("   {} Location: {}", "".dimmed(), package_dir.display());
            }
        } else {
            finish_success(
                &spinner,
                &format!("Installed {} v{} locally", package_name, actual_version),
            );
            println!("   {} Location: {}", "".dimmed(), package_dir.display());
        }

        // Log signature if present
        if let Some(sig) = &pkg_signature {
            println!(
                "   {} Signed package (signature: {}...)",
                "".green(),
                &sig[..16.min(sig.len())]
            );
        }

        // Pre-compile if installed to global cache and is Rust/C package
        if install_type == "global" {
            if let Err(e) = precompile_package(&package_dir) {
                println!("  {} Pre-compilation skipped: {}", "".yellow(), e);
            }
        }

        // Resolve transitive dependencies
        if let Ok(deps) = extract_package_dependencies(&package_dir) {
            if !deps.is_empty() {
                println!("  {} Found {} dependencies", "".cyan(), deps.len());
                for dep in &deps {
                    println!("    • {} {}", dep.name, dep.requirement);
                }

                // Recursively install dependencies
                self.install_dependencies(&deps, &target)?;
            }
        }

        // Fetch and apply driver metadata if this is a driver package
        if let Some(driver_meta) = self.fetch_driver_metadata_opt(package_name) {
            self.apply_driver_requirements(&driver_meta, &target)?;
        }

        Ok(actual_version)
    }

    /// Apply driver requirements (features, dependencies, system packages)
    fn apply_driver_requirements(
        &self,
        driver_meta: &DriverMetadata,
        target: &crate::workspace::InstallTarget,
    ) -> Result<()> {
        use crate::workspace::InstallTarget;

        // Get workspace path for local installations
        let workspace_path = match target {
            InstallTarget::Local(path) => Some(path.clone()),
            InstallTarget::Global => None,
        };

        // Apply required Cargo features
        if let Some(features) = &driver_meta.required_features {
            if !features.is_empty() {
                println!("  {} Driver requires features:", "[BUILD]".cyan());
                for feature in features {
                    println!("    • {}", feature.yellow());
                }

                // Update Cargo.toml if we have a workspace path
                if let Some(ws_path) = &workspace_path {
                    if let Err(e) = add_features_to_cargo_toml(ws_path, features) {
                        println!(
                            "  {} Could not auto-add features to Cargo.toml: {}",
                            "[WARN]".yellow(),
                            e
                        );
                        println!(
                            "    Add manually: horus_library = {{ features = {:?} }}",
                            features
                        );
                    } else {
                        println!("  {} Added features to Cargo.toml", "[OK]".green());
                    }
                }
            }
        }

        // Add Cargo dependencies (crates.io)
        if let Some(cargo_deps) = &driver_meta.cargo_dependencies {
            if !cargo_deps.is_empty() {
                println!("  {} Cargo dependencies required:", "[PKG]".cyan());
                for dep in cargo_deps {
                    println!("    • {}", dep.yellow());
                }

                // Update Cargo.toml if we have a workspace path
                if let Some(ws_path) = &workspace_path {
                    if let Err(e) = add_cargo_deps_to_cargo_toml(ws_path, cargo_deps) {
                        println!(
                            "  {} Could not auto-add dependencies to Cargo.toml: {}",
                            "[WARN]".yellow(),
                            e
                        );
                        println!("    Add manually to [dependencies]");
                    } else {
                        println!("  {} Added dependencies to Cargo.toml", "[OK]".green());
                    }
                }
            }
        }

        // Install Python dependencies (PyPI)
        if let Some(py_deps) = &driver_meta.python_dependencies {
            if !py_deps.is_empty() {
                println!("  {} Python dependencies required:", "[PY]".cyan());
                for dep in py_deps {
                    println!("    • {}", dep);
                }
                // Auto-install with pip
                for dep in py_deps {
                    let status = std::process::Command::new("pip3")
                        .args(["install", "--quiet", dep])
                        .status();
                    if status.is_ok() {
                        println!("  {} Installed {} via pip", "[OK]".green(), dep);
                    }
                }
            }
        }

        // Handle system dependencies with smart detection
        if let Some(sys_deps) = &driver_meta.system_dependencies {
            if !sys_deps.is_empty() {
                handle_system_dependencies(sys_deps)?;
            }
        }

        Ok(())
    }

    fn install_from_pypi(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        use std::process::Command;
        let spinner =
            progress::robot_download_spinner(&format!("Installing {} from PyPI...", package_name));

        // Check if package exists in system first
        if let Ok(Some(system_version)) = self.detect_system_python_package(package_name) {
            let choice =
                self.prompt_system_package_choice(package_name, &system_version, "PyPI")?;

            match choice {
                SystemPackageChoice::Cancel => {
                    return Err(anyhow!("installation cancelled by user"));
                }
                SystemPackageChoice::UseSystem => {
                    // Create reference to system package instead of installing
                    return self.create_system_reference_python(
                        package_name,
                        &system_version,
                        &target,
                    );
                }
                SystemPackageChoice::InstallHORUS => {
                    // Continue with installation below
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                }
            }
        }

        // Determine installation location based on target
        use crate::workspace::InstallTarget;
        let global_cache = crate::paths::cache_dir()?;

        let (install_dir, is_global, local_packages_dir) = match &target {
            InstallTarget::Global => {
                // Install to global cache
                fs::create_dir_all(&global_cache)?;
                let current_local = PathBuf::from(".horus/packages");
                (global_cache.clone(), true, Some(current_local))
            }
            InstallTarget::Local(workspace_path) => {
                // Install to workspace packages
                let local_packages = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local_packages)?;
                (local_packages.clone(), false, None)
            }
        };

        // Create temp venv for pip operations
        let temp_venv = PathBuf::from(".horus/venv");
        if !temp_venv.exists() {
            fs::create_dir_all(&temp_venv)?;
            let python_cmd = if Command::new("python3").arg("--version").output().is_ok() {
                "python3"
            } else {
                "python"
            };
            Command::new(python_cmd)
                .args(["-m", "venv", &*temp_venv.to_string_lossy()])
                .status()?;
        }

        let pip_path = temp_venv.join("bin/pip");

        // Build version string
        let version_str = version.unwrap_or("latest");
        let requirement = if version_str == "latest" {
            package_name.to_string()
        } else {
            format!("{}=={}", package_name, version_str)
        };

        // Install to target directory
        let pkg_dir = if is_global {
            install_dir.join(format!("pypi_{}@{}", package_name, version_str))
        } else {
            install_dir.join(package_name)
        };

        if pkg_dir.exists() {
            fs::remove_dir_all(&pkg_dir)?;
        }
        fs::create_dir_all(&pkg_dir)?;

        spinner.set_message(format!("Installing {} with pip...", package_name));
        let output = Command::new(&pip_path)
            .args(["install", "--target", &*pkg_dir.to_string_lossy()])
            .arg(&requirement)
            .output()?;

        if !output.status.success() {
            finish_error(
                &spinner,
                &format!("pip install failed for {}", package_name),
            );
            let stderr = String::from_utf8_lossy(&output.stderr);
            return Err(anyhow!("pip install failed:\n{}", stderr));
        }

        // Detect actual installed version from .dist-info directory
        let actual_version = detect_pypi_installed_version(&pkg_dir, package_name)
            .unwrap_or_else(|| version_str.to_string());

        // Create metadata.json
        let metadata = serde_json::json!({
            "name": package_name,
            "version": actual_version,
            "source": "PyPI"
        });
        let metadata_path = pkg_dir.join("metadata.json");
        fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        // If global, create symlink
        if is_global {
            if let Some(local_pkg_dir) = local_packages_dir {
                fs::create_dir_all(&local_pkg_dir)?;
                let local_link = local_pkg_dir.join(package_name);

                // Remove existing
                if local_link.exists() || local_link.symlink_metadata().is_ok() {
                    #[cfg(unix)]
                    {
                        if local_link.symlink_metadata()?.is_symlink() {
                            fs::remove_file(&local_link)?;
                        } else {
                            fs::remove_dir_all(&local_link)?;
                        }
                    }
                    #[cfg(windows)]
                    {
                        if local_link.is_dir() {
                            fs::remove_dir_all(&local_link)?;
                        } else {
                            fs::remove_file(&local_link)?;
                        }
                    }
                }

                // Create symlink
                #[cfg(unix)]
                std::os::unix::fs::symlink(&pkg_dir, &local_link)?;
                #[cfg(windows)]
                std::os::windows::fs::symlink_dir(&pkg_dir, &local_link)?;

                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!(
                    "   {} Linked: {} -> {}",
                    "".dimmed(),
                    local_link.display(),
                    pkg_dir.display()
                );
            }
        } else {
            finish_success(
                &spinner,
                &format!("Installed {} v{} locally", package_name, actual_version),
            );
            println!("   {} Location: {}", "".dimmed(), pkg_dir.display());
        }

        Ok(actual_version)
    }

    fn install_from_cratesio(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        use std::process::Command;
        let spinner = progress::robot_download_spinner(&format!(
            "Installing {} from crates.io...",
            package_name
        ));

        // Check if cargo is available
        if Command::new("cargo").arg("--version").output().is_err() {
            return Err(anyhow!(
                "cargo not found. Please install Rust toolchain from https://rustup.rs"
            ));
        }

        // Check if binary exists in system first
        if let Ok(Some(system_version)) = self.detect_system_cargo_binary(package_name) {
            let choice =
                self.prompt_system_package_choice(package_name, &system_version, "crates.io")?;

            match choice {
                SystemPackageChoice::Cancel => {
                    return Err(anyhow!("installation cancelled by user"));
                }
                SystemPackageChoice::UseSystem => {
                    // Create reference to system binary instead of installing
                    return self.create_system_reference_cargo(
                        package_name,
                        &system_version,
                        &target,
                    );
                }
                SystemPackageChoice::InstallHORUS => {
                    // Continue with installation below
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                }
            }
        }

        // Determine installation location based on target
        use crate::workspace::InstallTarget;
        let global_cache = crate::paths::cache_dir()?;

        let (install_root, is_global, local_packages_dir) = match &target {
            InstallTarget::Global => {
                // Install to global cache
                fs::create_dir_all(&global_cache)?;
                let current_local = PathBuf::from(".horus/packages");
                (global_cache.clone(), true, Some(current_local))
            }
            InstallTarget::Local(workspace_path) => {
                // Install to workspace packages
                let local_packages = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local_packages)?;
                (local_packages.clone(), false, None)
            }
        };

        // Build version string
        let version_str = version.unwrap_or("latest");
        let crate_spec = if version_str == "latest" {
            package_name.to_string()
        } else {
            format!("{}@{}", package_name, version_str)
        };

        // Install directory
        let pkg_dir = if is_global {
            install_root.join(format!("cratesio_{}@{}", package_name, version_str))
        } else {
            install_root.join(package_name)
        };

        if pkg_dir.exists() {
            fs::remove_dir_all(&pkg_dir)?;
        }
        fs::create_dir_all(&pkg_dir)?;

        spinner.set_message(format!("Installing {} with cargo...", package_name));

        // Use cargo install with --root to install to specific directory
        let mut cmd = Command::new("cargo");
        cmd.arg("install");
        cmd.arg(&crate_spec);
        cmd.arg("--root");
        cmd.arg(&pkg_dir);

        let output = cmd.output()?;

        if !output.status.success() {
            finish_error(
                &spinner,
                &format!("cargo install failed for {}", package_name),
            );
            let stderr = String::from_utf8_lossy(&output.stderr);
            return Err(anyhow!("cargo install failed:\n{}", stderr));
        }

        // Detect actual installed version from the binary
        let actual_version = detect_cargo_installed_version(&pkg_dir, package_name)
            .unwrap_or_else(|| version_str.to_string());

        // Create metadata.json
        let metadata = serde_json::json!({
            "name": package_name,
            "version": actual_version,
            "source": "CratesIO"
        });
        let metadata_path = pkg_dir.join("metadata.json");
        fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        // If global, create symlink
        if is_global {
            if let Some(local_pkg_dir) = local_packages_dir {
                fs::create_dir_all(&local_pkg_dir)?;
                let local_link = local_pkg_dir.join(package_name);

                // Remove existing
                if local_link.exists() || local_link.symlink_metadata().is_ok() {
                    #[cfg(unix)]
                    {
                        if local_link.symlink_metadata()?.is_symlink() {
                            fs::remove_file(&local_link)?;
                        } else {
                            fs::remove_dir_all(&local_link)?;
                        }
                    }
                    #[cfg(windows)]
                    {
                        if local_link.is_dir() {
                            fs::remove_dir_all(&local_link)?;
                        } else {
                            fs::remove_file(&local_link)?;
                        }
                    }
                }

                // Create symlink
                #[cfg(unix)]
                std::os::unix::fs::symlink(&pkg_dir, &local_link)?;
                #[cfg(windows)]
                std::os::windows::fs::symlink_dir(&pkg_dir, &local_link)?;

                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!(
                    "   {} Linked: {} -> {}",
                    "".dimmed(),
                    local_link.display(),
                    pkg_dir.display()
                );
                println!("   {} Binaries: {}/bin/", "".dimmed(), pkg_dir.display());
            }
        } else {
            finish_success(
                &spinner,
                &format!("Installed {} v{} locally", package_name, actual_version),
            );
            println!("   {} Location: {}", "".dimmed(), pkg_dir.display());
            println!("   {} Binaries: {}/bin/", "".dimmed(), pkg_dir.display());
        }

        Ok(actual_version)
    }

    // Install multiple dependencies recursively
    fn install_dependencies(
        &self,
        dependencies: &[DependencySpec],
        target: &crate::workspace::InstallTarget,
    ) -> Result<()> {
        // Filter dependencies by target platform
        let current_platform = format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH);
        let dependencies: Vec<DependencySpec> = dependencies
            .iter()
            .filter(|dep| {
                match &dep.target {
                    Some(t) => t == &current_platform || t == std::env::consts::OS,
                    None => true, // No target means all platforms
                }
            })
            .cloned()
            .collect();

        // Use dependency resolver for version resolution
        use crate::dependency_resolver::{DependencyResolver, ResolvedDependency};

        println!("  {} Resolving dependency versions...", "".cyan());

        // Create resolver with this registry client as provider
        let mut resolver = DependencyResolver::new(self);

        // Resolve all dependencies with version constraints
        let resolved: Vec<ResolvedDependency> = match resolver.resolve(dependencies.to_vec()) {
            Ok(r) => r,
            Err(e) => {
                println!("  {} Dependency resolution failed: {}", "".red(), e);
                println!("  {} Falling back to simple installation...", "".yellow());

                // Fallback: install without version resolution
                for dep in dependencies {
                    let dep_name = &dep.name;

                    // Check if already installed
                    let is_installed = match target {
                        crate::workspace::InstallTarget::Global => {
                            let global_cache = crate::paths::cache_dir()?;
                            check_global_versions(&global_cache, dep_name)?
                        }
                        crate::workspace::InstallTarget::Local(workspace_path) => {
                            let local_packages = workspace_path.join(".horus/packages");
                            local_packages.join(dep_name).exists()
                        }
                    };

                    if is_installed {
                        println!("  {} {} (already installed)", "".green(), dep_name);
                        continue;
                    }

                    // Install latest version
                    println!("  {} Installing dependency: {}...", "".cyan(), dep_name);
                    self.install_to_target(dep_name, None, target.clone())?;
                }
                return Ok(());
            }
        };

        // Install resolved versions
        for resolved_dep in resolved {
            let version_str = resolved_dep.version.to_string();
            let home = dirs::home_dir().ok_or_else(|| anyhow!("could not find home directory"))?;
            let global_cache = home.join(".horus/cache");

            // Check if already installed
            let (is_installed_local, is_installed_global) = match &target {
                crate::workspace::InstallTarget::Global => {
                    let has_global = check_global_versions(&global_cache, &resolved_dep.name)?;
                    (has_global, has_global)
                }
                crate::workspace::InstallTarget::Local(workspace_path) => {
                    let local_packages = workspace_path.join(".horus/packages");
                    let has_local = local_packages.join(&resolved_dep.name).exists();
                    let has_global =
                        check_global_versions(&global_cache, &resolved_dep.name).unwrap_or(false);
                    (has_local, has_global)
                }
            };

            if is_installed_local {
                println!(
                    "  {} {} v{} (already installed)",
                    "".green(),
                    resolved_dep.name,
                    resolved_dep.version
                );
                continue;
            }

            // If package exists in global cache but not local, create symlink instead of downloading
            if !is_installed_local && is_installed_global {
                if let crate::workspace::InstallTarget::Local(workspace_path) = &target {
                    println!(
                        "  {} Linking {} v{} from global cache...",
                        "".cyan(),
                        resolved_dep.name,
                        resolved_dep.version
                    );

                    // Find the global package directory
                    let package_dir_name = format!("{}@{}", resolved_dep.name, version_str);
                    let global_package_dir = global_cache.join(&package_dir_name);

                    if global_package_dir.exists() {
                        let local_packages = workspace_path.join(".horus/packages");
                        fs::create_dir_all(&local_packages)?;
                        let local_link = local_packages.join(&resolved_dep.name);

                        // Create symlink
                        #[cfg(unix)]
                        std::os::unix::fs::symlink(&global_package_dir, &local_link)?;
                        #[cfg(windows)]
                        std::os::windows::fs::symlink_dir(&global_package_dir, &local_link)?;

                        println!(
                            "  {} {} v{} (linked from global cache)",
                            "".green(),
                            resolved_dep.name,
                            resolved_dep.version
                        );
                        continue;
                    }
                }
            }

            // Install the resolved version from registry
            println!(
                "  {} Installing {} v{}...",
                "".cyan(),
                resolved_dep.name,
                resolved_dep.version
            );
            self.install_to_target(&resolved_dep.name, Some(&version_str), target.clone())?;
        }

        Ok(())
    }
}

impl RegistryClient {
    // Install a package from local filesystem path
    // Returns the detected version
    pub fn install_from_path(
        &self,
        package_name: &str,
        path: &Path,
        target: crate::workspace::InstallTarget,
        base_dir: Option<&Path>,
    ) -> Result<String> {
        use crate::workspace::InstallTarget;
        log::debug!("installing package: {} from path: {:?}", package_name, path);

        println!(
            " Installing {} from path: {}...",
            package_name,
            path.display()
        );

        // Resolve relative path to absolute
        let source_path = if path.is_absolute() {
            path.to_path_buf()
        } else {
            // Resolve relative to base_dir (horus.yaml location) or current directory
            let base = base_dir
                .map(|p| p.to_path_buf())
                .or_else(|| std::env::current_dir().ok())
                .unwrap_or_else(|| PathBuf::from("."));
            base.join(path)
        };

        log::debug!("resolved package to: {:?}", source_path);

        if !source_path.exists() {
            return Err(anyhow!("Path does not exist: {}", source_path.display()));
        }

        if !source_path.is_dir() {
            return Err(anyhow!(
                "Path is not a directory: {}",
                source_path.display()
            ));
        }

        // Detect version from package manifest
        let version = detect_package_version(&source_path).unwrap_or_else(|| "dev".to_string());

        // Determine packages directory based on target
        let packages_dir = match &target {
            InstallTarget::Global => {
                let current = PathBuf::from(".horus/packages");
                fs::create_dir_all(&current)?;
                current
            }
            InstallTarget::Local(workspace_path) => {
                let local = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local)?;
                local
            }
        };

        let link_path = packages_dir.join(package_name);

        // Remove existing symlink/directory if present
        if link_path.exists() || link_path.symlink_metadata().is_ok() {
            #[cfg(unix)]
            {
                if link_path.symlink_metadata()?.is_symlink() {
                    fs::remove_file(&link_path)?;
                } else {
                    fs::remove_dir_all(&link_path)?;
                }
            }
            #[cfg(windows)]
            {
                if link_path.is_dir() {
                    fs::remove_dir_all(&link_path)?;
                } else {
                    fs::remove_file(&link_path)?;
                }
            }
        }

        // Create symlink to source path
        #[cfg(unix)]
        std::os::unix::fs::symlink(&source_path, &link_path)?;
        #[cfg(windows)]
        std::os::windows::fs::symlink_dir(&source_path, &link_path)?;

        // Create metadata for tracking
        let metadata = serde_json::json!({
            "name": package_name,
            "version": version,
            "source": "Path",
            "source_path": source_path.display().to_string()
        });

        let metadata_file = packages_dir.join(format!("{}.path.json", package_name));
        fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

        println!(" Installed {} v{} from path", package_name, version);
        println!(
            "   Link: {} -> {}",
            link_path.display(),
            source_path.display()
        );
        println!(
            "   {} Path dependencies are live-linked - changes take effect immediately",
            "ℹ".cyan()
        );

        Ok(version)
    }
}

impl RegistryClient {
    // Detect if a Python package exists in system site-packages
    fn detect_system_python_package(&self, package_name: &str) -> Result<Option<String>> {
        use std::process::Command;

        // Try to find package using pip show
        let output = Command::new("python3")
            .args(["-m", "pip", "show", package_name])
            .output();

        if let Ok(output) = output {
            if output.status.success() {
                let stdout = String::from_utf8_lossy(&output.stdout);
                // Parse version from pip show output
                for line in stdout.lines() {
                    if line.starts_with("Version:") {
                        let version = line.trim_start_matches("Version:").trim();
                        return Ok(Some(version.to_string()));
                    }
                }
            }
        }

        // Fallback: check site-packages directly
        let mut site_packages_paths = vec![
            PathBuf::from(format!(
                "/usr/lib/python3.12/site-packages/{}",
                package_name
            )),
            PathBuf::from(format!(
                "/usr/local/lib/python3.12/site-packages/{}",
                package_name
            )),
        ];

        if let Some(home) = dirs::home_dir() {
            site_packages_paths.push(home.join(format!(
                ".local/lib/python3.12/site-packages/{}",
                package_name
            )));
        }

        for path in site_packages_paths {
            if path.exists() {
                // Try to get version from __init__.py or metadata
                let version_file = path.join("__init__.py");
                if version_file.exists() {
                    // Found package, but version unknown
                    return Ok(Some("unknown".to_string()));
                }
            }
        }

        Ok(None)
    }

    // Detect if a Rust binary exists in system cargo bin
    fn detect_system_cargo_binary(&self, package_name: &str) -> Result<Option<String>> {
        crate::cargo_utils::detect_system_cargo_binary(package_name)
    }

    // Prompt user for what to do with system package
    fn prompt_system_package_choice(
        &self,
        package_name: &str,
        system_version: &str,
        source_type: &str, // "PyPI" or "crates.io"
    ) -> Result<SystemPackageChoice> {
        use std::io::{self, Write};

        println!(
            "\n{} {} {} found in system (version: {})",
            "".yellow(),
            source_type,
            package_name.green(),
            system_version.cyan()
        );
        println!("\nWhat would you like to do?");
        println!("  [1] {} Use system package (create reference)", "".green());
        println!(
            "  [2] {} Install to HORUS (isolated environment)",
            "".blue()
        );
        println!("  [3] {} Cancel installation", "[FAIL]".red());

        print!("\nChoice [1-3]: ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;

        match input.trim() {
            "1" => Ok(SystemPackageChoice::UseSystem),
            "2" => Ok(SystemPackageChoice::InstallHORUS),
            "3" => Ok(SystemPackageChoice::Cancel),
            _ => {
                println!("Invalid choice, defaulting to Install to HORUS");
                Ok(SystemPackageChoice::InstallHORUS)
            }
        }
    }

    // Create reference to system Python package
    fn create_system_reference_python(
        &self,
        package_name: &str,
        system_version: &str,
        target: &crate::workspace::InstallTarget,
    ) -> Result<String> {
        use crate::workspace::InstallTarget;
        use std::process::Command;

        println!("  {} Creating reference to system package...", "".green());

        // Find actual system package location
        let output = Command::new("python3")
            .args([
                "-c",
                &format!("import {}; print({}.__file__)", package_name, package_name),
            ])
            .output()?;

        if !output.status.success() {
            return Err(anyhow!("Failed to locate system package"));
        }

        let package_file = String::from_utf8_lossy(&output.stdout).trim().to_string();
        let package_path = PathBuf::from(&package_file)
            .parent()
            .ok_or_else(|| anyhow!("Invalid package path"))?
            .to_path_buf();

        // Create metadata file in .horus/packages/ with system reference
        let packages_dir = match target {
            InstallTarget::Global => {
                let current = PathBuf::from(".horus/packages");
                fs::create_dir_all(&current)?;
                current
            }
            InstallTarget::Local(workspace_path) => {
                let local = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local)?;
                local
            }
        };

        let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
        let metadata = serde_json::json!({
            "name": package_name,
            "version": system_version,
            "source": "System",
            "system_path": package_path.display().to_string(),
            "package_type": "PyPI"
        });

        fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

        println!(
            "  {} Using system package at {}",
            "".green(),
            package_path.display()
        );
        println!(
            "  {} Reference created: {}",
            "".cyan(),
            metadata_file.display()
        );

        Ok(system_version.to_string())
    }

    // Create reference to system cargo binary
    fn create_system_reference_cargo(
        &self,
        package_name: &str,
        system_version: &str,
        target: &crate::workspace::InstallTarget,
    ) -> Result<String> {
        use crate::workspace::InstallTarget;

        println!("  {} Creating reference to system binary...", "".green());

        // Find actual system binary location
        let cargo_bin = crate::paths::home_dir()?.join(".cargo/bin").join(package_name);

        if !cargo_bin.exists() {
            return Err(anyhow!("System binary not found at expected location"));
        }

        // Create metadata file in .horus/packages/ with system reference
        let packages_dir = match target {
            InstallTarget::Global => {
                let current = PathBuf::from(".horus/packages");
                fs::create_dir_all(&current)?;
                current
            }
            InstallTarget::Local(workspace_path) => {
                let local = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local)?;
                local
            }
        };

        let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
        let metadata = serde_json::json!({
            "name": package_name,
            "version": system_version,
            "source": "System",
            "system_path": cargo_bin.display().to_string(),
            "package_type": "CratesIO"
        });

        fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

        // Create symlink in .horus/bin to system binary
        let bin_dir = match target {
            InstallTarget::Global => PathBuf::from(".horus/bin"),
            InstallTarget::Local(workspace_path) => workspace_path.join(".horus/bin"),
        };
        fs::create_dir_all(&bin_dir)?;

        let bin_link = bin_dir.join(package_name);
        if bin_link.exists() {
            fs::remove_file(&bin_link)?;
        }
        #[cfg(unix)]
        std::os::unix::fs::symlink(&cargo_bin, &bin_link)?;
        #[cfg(windows)]
        {
            // On Windows, create a .cmd wrapper for executables
            let cmd_link = bin_dir.join(format!("{}.cmd", package_name));
            let cmd_content = format!("@echo off\n\"{}\" %*\n", cargo_bin.display());
            fs::write(&cmd_link, cmd_content)?;
        }

        println!(
            "  {} Using system binary at {}",
            "".green(),
            cargo_bin.display()
        );
        println!(
            "  {} Reference created: {}",
            "".cyan(),
            metadata_file.display()
        );
        println!("  {} Binary linked: {}", "".cyan(), bin_link.display());

        Ok(system_version.to_string())
    }
}
