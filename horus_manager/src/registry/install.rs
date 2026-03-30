use super::helpers::*;
use super::*;

/// Returns the Rust target triple for the current platform.
fn current_target_triple() -> &'static str {
    match (std::env::consts::OS, std::env::consts::ARCH) {
        ("linux", "x86_64") => "x86_64-unknown-linux-gnu",
        ("linux", "aarch64") => "aarch64-unknown-linux-gnu",
        ("macos", "x86_64") => "x86_64-apple-darwin",
        ("macos", "aarch64") => "aarch64-apple-darwin",
        ("windows", "x86_64") => "x86_64-pc-windows-msvc",
        _ => "unknown",
    }
}

/// Convert a semver `VersionReq` to a pip-compatible version specifier string.
/// Returns `None` for `*` (any version), or a pip specifier like `>=3.0,<4.0`.
///
/// Semver `VersionReq` is rendered as a comma-separated list of comparators
/// (e.g., `>=1.2.0, <2.0.0`). These are already pip-compatible since pip
/// understands `>=`, `<=`, `>`, `<`, `==`, `!=` operators.
pub(crate) fn semver_req_to_pip(req: &semver::VersionReq) -> Option<String> {
    let s = req.to_string();
    if s == "*" {
        return None;
    }
    // semver crate renders `^1.2` as `>=1.2.0, <2.0.0` and `~1.2` as `>=1.2.0, <1.3.0`
    // which are valid pip specifiers. Just return the rendered string.
    Some(s)
}

/// RAII guard that cleans up a temporary directory when dropped.
/// Call `disarm()` on success to prevent cleanup.
pub(crate) struct TempDirGuard {
    path: PathBuf,
    armed: bool,
}

impl TempDirGuard {
    pub(crate) fn new(path: PathBuf) -> Self {
        Self { path, armed: true }
    }

    pub(crate) fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for TempDirGuard {
    fn drop(&mut self) {
        if self.armed && self.path.exists() {
            if let Err(e) = fs::remove_dir_all(&self.path) {
                log::warn!("Failed to clean up temp dir {}: {}", self.path.display(), e);
            }
        }
    }
}

/// Verify an Ed25519 signature against a local public key file.
/// Returns Ok(true) if valid, Ok(false) if invalid, Err on format/IO errors.
pub(crate) fn verify_package_signature(
    package_data: &[u8],
    signature_hex: &str,
    public_key_path: &Path,
) -> Result<bool> {
    use ed25519_dalek::{Signature, Verifier, VerifyingKey};

    let sig_bytes =
        hex::decode(signature_hex).map_err(|_| anyhow!("Invalid signature hex encoding"))?;
    let signature = Signature::from_bytes(
        sig_bytes
            .as_slice()
            .try_into()
            .map_err(|_| anyhow!("Invalid signature length"))?,
    );

    let pub_hex = fs::read_to_string(public_key_path)
        .map_err(|e| anyhow!("Failed to read public key: {}", e))?;
    let pub_bytes =
        hex::decode(pub_hex.trim()).map_err(|_| anyhow!("Invalid public key hex encoding"))?;
    let verifying_key = VerifyingKey::from_bytes(
        pub_bytes
            .as_slice()
            .try_into()
            .map_err(|_| anyhow!("Invalid public key length"))?,
    )
    .map_err(|_| anyhow!("Invalid Ed25519 public key"))?;

    Ok(verifying_key.verify(package_data, &signature).is_ok())
}

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
        log::debug!(
            "installing package: {} to target: {:?}",
            package_name,
            target
        );
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
                "Path dependencies should be added to horus.toml [dependencies].\n\
                     Example: my-lib = {{ source = \"path\", path = \"../my-lib\" }}"
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

        // Package only in PyPI
        if in_pypi {
            return Ok(PackageSource::PyPI);
        }

        // Not found in any registry
        Err(anyhow!(
            "Package '{}' not found in any registry.\n\
             Checked: HORUS registry, PyPI, crates.io\n\n\
             Suggestions:\n\
             - Check the spelling of the package name\n\
             - Search available packages: horus search {}",
            package_name,
            package_name
        ))
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
        println!(
            "  [3] {} Cancel installation",
            crate::cli_output::ICON_ERROR.red()
        );

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
        use crate::dependency_resolver::{DependencySource, PackageProvider};

        match &spec.source {
            DependencySource::Registry => {
                // For registry dependencies, resolve semver requirement to an exact version
                let version_str = if spec.requirement.to_string() != "*" {
                    // Query available versions and find best match
                    match self.get_available_versions(&spec.name) {
                        Ok(versions) if !versions.is_empty() => {
                            let matching: Vec<&semver::Version> = versions
                                .iter()
                                .filter(|v| spec.requirement.matches(v))
                                .collect();
                            match matching.last() {
                                Some(v) => Some(v.to_string()),
                                None => {
                                    return Err(anyhow::anyhow!(
                                        "No version of '{}' satisfies requirement {}. Available: {}",
                                        spec.name,
                                        spec.requirement,
                                        versions.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ")
                                    ));
                                }
                            }
                        }
                        _ => None, // Fall back to "latest" if registry unreachable
                    }
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
                    crate::cli_output::ICON_INFO.cyan(),
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
                // Install from PyPI — convert semver requirement to pip specifier
                let version_str = semver_req_to_pip(&spec.requirement);
                self.install_from_pypi(package_name, version_str.as_deref(), target)
                    .map(|_| ()) // Ignore version for dependency spec
            }
            DependencySource::CratesIO => {
                // Install from crates.io — cargo install accepts semver @^1.2.3 syntax
                let version_str = if spec.requirement.to_string() != "*" {
                    Some(spec.requirement.to_string())
                } else {
                    None
                };
                self.install_from_cratesio(&spec.name, version_str.as_deref(), target)
                    .map(|_| ())
            }
            DependencySource::System => {
                eprintln!(
                    "  {} System dependency '{}' - install with your system package manager",
                    crate::cli_output::ICON_INFO.cyan(),
                    spec.name,
                );
                Ok(())
            }
        }
    }

    pub(crate) fn install_from_registry(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        log::debug!(
            "installing {} from HORUS registry (version: {:?})",
            package_name,
            version
        );
        let spinner = progress::download_spinner(&format!(
            "Downloading {} from HORUS registry...",
            package_name
        ));

        let version_str = version.unwrap_or("latest");
        // URL-encode scoped package names for API calls
        let encoded_name = url_encode_package_name(package_name);

        // Try pre-built binary first (skip source download + compile)
        let target_triple = current_target_triple();
        if target_triple != "unknown" {
            let bin_url = format!(
                "{}/api/packages/{}/{}/binaries/{}/download",
                self.base_url, encoded_name, version_str, target_triple
            );
            if let Ok(resp) = self.client.get(&bin_url).send() {
                if resp.status().is_success() {
                    // Binary found — fast path
                    return self.install_binary_artifact(
                        package_name,
                        version_str,
                        &target,
                        resp,
                        target_triple,
                        &spinner,
                    );
                }
                // 404 = no binary for this platform, fall through to source
                log::debug!(
                    "No pre-built binary for {} on {}, falling back to source",
                    package_name,
                    target_triple
                );
            }
        }

        let url = format!(
            "{}/api/packages/{}/{}/download",
            self.base_url, encoded_name, version_str
        );

        // Download source package
        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            let status = response.status();
            let body = response.text().unwrap_or_default();

            // Check if the package was yanked (410 Gone)
            if status == reqwest::StatusCode::GONE {
                if let Ok(json) = serde_json::from_str::<serde_json::Value>(&body) {
                    let reason = json
                        .get("reason")
                        .and_then(|v| v.as_str())
                        .unwrap_or("No reason given");
                    return Err(anyhow!(
                        "Package {}@{} has been yanked: {}\n\n\
                         The maintainer has withdrawn this version.\n\
                         Try installing a different version:\n\
                         - horus install {}          (latest non-yanked)\n\
                         - horus search {}           (see available versions)",
                        package_name,
                        version_str,
                        reason,
                        package_name,
                        package_name
                    ));
                }
            }

            if status == reqwest::StatusCode::NOT_FOUND {
                return Err(anyhow!(
                    "Package '{}' v{} not found in registry.\n\
                     Check the name and version, or run `horus search {}` to find available packages.",
                    package_name, version_str, package_name
                ));
            } else if status == reqwest::StatusCode::UNAUTHORIZED {
                return Err(anyhow!(
                    "Authentication required to download '{}'.\n\
                     Run `horus auth login` to authenticate.",
                    package_name
                ));
            } else {
                return Err(anyhow!(
                    "Failed to download '{}' v{}: HTTP {}\n\
                     The registry may be temporarily unavailable. Try again in a few minutes.",
                    package_name,
                    version_str,
                    status.as_u16()
                ));
            }
        }

        // Check for package signature header
        let pkg_signature = response
            .headers()
            .get("x-horus-signature")
            .and_then(|v| v.to_str().ok())
            .map(String::from);

        // Maximum download size (100 MB) to prevent OOM from malicious Content-Length
        const MAX_DOWNLOAD_SIZE: u64 = 100 * 1024 * 1024;

        // Download with progress bar for large packages (>512KB)
        let content_length = response.content_length();

        // Reject downloads that claim to exceed the size limit
        if let Some(total) = content_length {
            if total > MAX_DOWNLOAD_SIZE {
                return Err(anyhow!(
                    "Package {}@{} is too large ({} bytes, max {} MB). \
                     This may indicate a corrupted or malicious package.",
                    package_name,
                    version_str,
                    total,
                    MAX_DOWNLOAD_SIZE / (1024 * 1024)
                ));
            }
        }

        let bytes = if let Some(total) = content_length.filter(|&s| s > 512 * 1024) {
            // Replace spinner with progress bar for large downloads
            spinner.finish_and_clear();
            println!(
                "  {} Downloading {} ({})...",
                crate::cli_output::ICON_INFO.cyan(),
                package_name,
                progress::format_bytes(total)
            );
            use indicatif::{ProgressBar as DlBar, ProgressStyle as DlStyle};
            use std::io::Read;
            let pb = DlBar::new(total);
            pb.set_style(
                DlStyle::default_bar()
                    .template("   {bar:30.cyan/dim} {bytes}/{total_bytes} ({eta})")
                    .unwrap_or_else(|_| DlStyle::default_bar())
                    .progress_chars("##-"),
            );
            let mut downloaded = Vec::with_capacity(total as usize);
            let mut stream = response;
            let mut buf = [0u8; 8192];
            loop {
                match stream.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        downloaded.extend_from_slice(&buf[..n]);
                        if downloaded.len() as u64 > MAX_DOWNLOAD_SIZE {
                            return Err(anyhow!(
                                "Download exceeded maximum size ({} MB)",
                                MAX_DOWNLOAD_SIZE / (1024 * 1024)
                            ));
                        }
                        pb.set_position(downloaded.len() as u64);
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                    Err(e) => return Err(anyhow::anyhow!("Download interrupted: {}", e)),
                }
            }
            pb.finish_and_clear();
            downloaded
        } else {
            // Small download path — still enforce size limit
            let bytes = response.bytes()?.to_vec();
            if bytes.len() as u64 > MAX_DOWNLOAD_SIZE {
                return Err(anyhow!(
                    "Download exceeded maximum size ({} MB)",
                    MAX_DOWNLOAD_SIZE / (1024 * 1024)
                ));
            }
            bytes
        };

        // Verify Ed25519 signature
        let pub_key_path = crate::paths::keys_dir()
            .ok()
            .map(|d| d.join("signing_key.pub"));
        let has_public_key = pub_key_path.as_ref().is_some_and(|p| p.exists());

        if let Some(ref sig_hex) = pkg_signature {
            if let Some(pub_path) = pub_key_path.filter(|p| p.exists()) {
                match verify_package_signature(&bytes, sig_hex, &pub_path) {
                    Ok(true) => {
                        log::info!("Package signature verified for {}", package_name);
                    }
                    Ok(false) => {
                        return Err(anyhow!(
                            "Package signature verification FAILED for {}. \
                             The package may have been tampered with. \
                             If you trust this package, remove your local public key.",
                            package_name
                        ));
                    }
                    Err(e) => {
                        // Crypto/IO errors are hard failures — don't silently continue
                        return Err(anyhow!(
                            "Signature verification error for {}: {}. \
                             Check your public key file at ~/.horus/signing_key.pub",
                            package_name,
                            e
                        ));
                    }
                }
            } else {
                return Err(anyhow!(
                    "Package {} is signed but no local public key found to verify the signature.\n\
                     Install the publisher's public key to ~/.horus/signing_key.pub before installing.\n\
                     If you trust this package without verification, use --skip-verify.",
                    package_name
                ));
            }
        } else if has_public_key {
            // Public key configured but package is unsigned — warn the user
            log::warn!(
                "Package {} is NOT signed, but you have a signing key configured. \
                 This package's integrity cannot be verified.",
                package_name
            );
        }

        // Calculate checksum and verify against server
        let mut hasher = Sha256::new();
        hasher.update(&bytes);
        let checksum = format!("{:x}", hasher.finalize());

        // Fetch expected checksum from server and compare
        let checksum_url = format!(
            "{}/api/packages/{}/{}/checksum",
            self.base_url, encoded_name, version_str
        );
        match self.client.get(&checksum_url).send() {
            Ok(resp) if resp.status().is_success() => {
                if let Ok(json) = resp.json::<serde_json::Value>() {
                    if let Some(expected) = json.get("checksum").and_then(|v| v.as_str()) {
                        if !expected.is_empty() && expected != checksum {
                            return Err(anyhow!(
                                "Checksum mismatch for {}@{}!\n\
                                 Expected: {}\n\
                                 Got:      {}\n\n\
                                 The download may be corrupted or tampered with.\n\
                                 Try again, and if this persists, report it to the package maintainer.",
                                package_name, version_str, expected, checksum
                            ));
                        }
                        log::debug!("Checksum verified for {}@{}", package_name, version_str);
                    }
                }
            }
            _ => {
                log::warn!(
                    "Could not fetch checksum from server for {}@{}. \
                     Package integrity cannot be verified.",
                    package_name,
                    version_str
                );
                eprintln!(
                    "\n{} Warning: Could not verify package checksum for {}@{}.\n\
                     The registry may be unreachable or the package may have been tampered with.\n\
                     Use --skip-verify to install anyway.",
                    crate::cli_output::ICON_WARN.yellow(),
                    package_name,
                    version_str
                );
                return Err(anyhow!(
                    "Checksum verification failed: could not fetch checksum from server for {}@{}",
                    package_name,
                    version_str
                ));
            }
        }

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

        // Extract to temporary location first to detect version.
        // Use PID + random suffix to avoid collisions with concurrent installs.
        let temp_dir = std::env::temp_dir().join(format!(
            "horus_pkg_{}_{}_{}",
            safe_pkg_name,
            std::process::id(),
            rand::random::<u32>()
        ));
        fs::create_dir_all(&temp_dir)?;
        let mut temp_guard = TempDirGuard::new(temp_dir.clone());

        // Safe extraction: validate each entry to prevent path traversal and symlink attacks
        let canonical_temp = temp_dir.canonicalize()?;
        for entry in archive.entries()? {
            let mut entry = entry?;
            let entry_path = entry.path()?;
            let entry_str = entry_path.to_string_lossy();

            // Reject null bytes
            if entry_str.contains('\0') {
                log::warn!("Skipping tar entry with null byte in path: {:?}", entry_str);
                continue;
            }

            // Reject absolute paths
            if entry_path.is_absolute() {
                log::warn!("Skipping absolute tar entry: {:?}", entry_str);
                continue;
            }

            // Reject path traversal
            if entry_path
                .components()
                .any(|c| matches!(c, std::path::Component::ParentDir))
            {
                log::warn!("Skipping tar entry with path traversal: {:?}", entry_str);
                continue;
            }

            // Reject symlinks pointing outside the extraction directory
            if entry.header().entry_type().is_symlink()
                || entry.header().entry_type().is_hard_link()
            {
                if let Ok(Some(target)) = entry.link_name() {
                    let target_path = target.to_path_buf();
                    if target_path.is_absolute()
                        || target_path
                            .components()
                            .any(|c| matches!(c, std::path::Component::ParentDir))
                    {
                        log::warn!(
                            "Skipping symlink with unsafe target: {:?} -> {:?}",
                            entry_str,
                            target_path
                        );
                        continue;
                    }
                }
            }

            // Resolve final path and verify it's within temp_dir
            let dest = canonical_temp.join(
                entry_path
                    .components()
                    .filter(|c| !matches!(c, std::path::Component::CurDir))
                    .collect::<PathBuf>(),
            );
            if !dest.starts_with(&canonical_temp) {
                log::warn!(
                    "Skipping tar entry escaping extraction dir: {:?}",
                    entry_str
                );
                continue;
            }

            entry.unpack(&dest)?;
        }

        // Get actual version from package info (for "latest" downloads)
        let actual_version = if version_str == "latest" {
            detect_package_version(&temp_dir).ok_or_else(|| {
                anyhow!("Could not detect version for {}. The package may be malformed (missing horus.toml/Cargo.toml/package.json).", package_name)
            })?
        } else {
            version_str.to_string()
        };

        // Move to final location with version info (use safe path name)
        let package_dir = if install_type == "global" {
            install_dir.join(format!("{}@{}", safe_pkg_name, actual_version))
        } else {
            install_dir.join(&safe_pkg_name)
        };

        // Atomic install: copy to staging dir, then rename into place.
        // This prevents partial installs if copy fails mid-way.
        let staging_dir = package_dir.with_extension("staging");
        if staging_dir.exists() {
            fs::remove_dir_all(&staging_dir)?;
        }
        fs::create_dir_all(&staging_dir)?;

        // Copy from temp to staging
        copy_dir_all(&temp_dir, &staging_dir)?;
        // Temp dir successfully copied to staging — disarm the cleanup guard
        temp_guard.disarm();
        fs::remove_dir_all(&temp_dir)?;

        // Swap staging into final location atomically
        if package_dir.exists() {
            fs::remove_dir_all(&package_dir)?;
        }
        fs::rename(&staging_dir, &package_dir)?;

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
                // Try remove_file first (handles symlinks atomically without TOCTOU),
                // fall back to remove_dir_all for actual directories.
                if (local_link.exists() || local_link.symlink_metadata().is_ok())
                    && fs::remove_file(&local_link).is_err()
                {
                    fs::remove_dir_all(&local_link)?;
                }

                // Create symlink
                #[cfg(unix)]
                horus_sys::fs::symlink(&package_dir, &local_link)?;
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
                // Plugin packages MUST have a working binary — propagate the error
                let is_plugin = crate::commands::pkg::detect_plugin_metadata(&package_dir).is_some();
                if is_plugin {
                    return Err(anyhow!(
                        "Pre-compilation failed for plugin package '{}': {}\n\
                         Plugins require a working binary. Install a pre-built binary instead:\n\
                         horus install {}",
                        package_name,
                        e,
                        package_name
                    ));
                }
                // Non-plugin (library) packages: warn and continue
                println!("  {} Pre-compilation skipped: {}", "".yellow(), e);
            }
        }

        // Resolve transitive dependencies
        if let Ok(deps) = extract_package_dependencies(&package_dir) {
            if !deps.is_empty() {
                println!("  {} Found {} dependencies", "".cyan(), deps.len());
                for dep in &deps {
                    println!("    - {} {}", dep.name, dep.requirement);
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

    /// Install a pre-built binary artifact from the registry (fast path).
    /// Skips source download + compilation entirely.
    fn install_binary_artifact(
        &self,
        package_name: &str,
        version_str: &str,
        target: &crate::workspace::InstallTarget,
        response: reqwest::blocking::Response,
        target_triple: &str,
        spinner: &indicatif::ProgressBar,
    ) -> Result<String> {
        log::info!(
            "Installing pre-built binary for {} ({}) on {}",
            package_name,
            version_str,
            target_triple
        );

        // Read checksum from header (optional — registry may or may not provide it)
        let expected_checksum = response
            .headers()
            .get("x-horus-checksum")
            .and_then(|v| v.to_str().ok())
            .map(String::from);

        // Maximum download size (100 MB)
        const MAX_DOWNLOAD_SIZE: u64 = 100 * 1024 * 1024;

        let content_length = response.content_length();

        if let Some(total) = content_length {
            if total > MAX_DOWNLOAD_SIZE {
                return Err(anyhow!(
                    "Binary artifact for {}@{} is too large ({} bytes, max {} MB).",
                    package_name,
                    version_str,
                    total,
                    MAX_DOWNLOAD_SIZE / (1024 * 1024)
                ));
            }
        }

        // Download with progress bar for large binaries (>512KB)
        let bytes = if let Some(total) = content_length.filter(|&s| s > 512 * 1024) {
            spinner.finish_and_clear();
            println!(
                "  {} Downloading {} binary ({})...",
                crate::cli_output::ICON_INFO.cyan(),
                package_name,
                progress::format_bytes(total)
            );
            use indicatif::{ProgressBar as DlBar, ProgressStyle as DlStyle};
            use std::io::Read;
            let pb = DlBar::new(total);
            pb.set_style(
                DlStyle::default_bar()
                    .template("   {bar:30.cyan/dim} {bytes}/{total_bytes} ({eta})")
                    .unwrap_or_else(|_| DlStyle::default_bar())
                    .progress_chars("##-"),
            );
            let mut downloaded = Vec::with_capacity(total as usize);
            let mut stream = response;
            let mut buf = [0u8; 8192];
            loop {
                match stream.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        downloaded.extend_from_slice(&buf[..n]);
                        if downloaded.len() as u64 > MAX_DOWNLOAD_SIZE {
                            return Err(anyhow!(
                                "Download exceeded maximum size ({} MB)",
                                MAX_DOWNLOAD_SIZE / (1024 * 1024)
                            ));
                        }
                        pb.set_position(downloaded.len() as u64);
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                    Err(e) => return Err(anyhow::anyhow!("Download interrupted: {}", e)),
                }
            }
            pb.finish_and_clear();
            downloaded
        } else {
            let bytes = response.bytes()?.to_vec();
            if bytes.len() as u64 > MAX_DOWNLOAD_SIZE {
                return Err(anyhow!(
                    "Download exceeded maximum size ({} MB)",
                    MAX_DOWNLOAD_SIZE / (1024 * 1024)
                ));
            }
            bytes
        };

        // Verify checksum from X-Horus-Checksum header
        let mut hasher = Sha256::new();
        hasher.update(&bytes);
        let actual_checksum = format!("{:x}", hasher.finalize());

        if let Some(ref expected) = expected_checksum {
            if !expected.is_empty() && expected != &actual_checksum {
                return Err(anyhow!(
                    "Checksum mismatch for binary artifact {}@{} ({})!\n\
                     Expected: {}\n\
                     Got:      {}\n\n\
                     The download may be corrupted or tampered with.",
                    package_name,
                    version_str,
                    target_triple,
                    expected,
                    actual_checksum
                ));
            }
            log::debug!(
                "Binary checksum verified for {}@{} ({})",
                package_name,
                version_str,
                target_triple
            );
        }

        // Determine installation directory (same logic as source install)
        let safe_pkg_name = package_name_to_path(package_name);

        use crate::workspace::InstallTarget;
        let global_cache = crate::paths::cache_dir()?;

        let (install_dir, install_type, local_packages_dir) = match target {
            InstallTarget::Global => {
                fs::create_dir_all(&global_cache)?;
                let current_local = PathBuf::from(".horus/packages");
                (global_cache.clone(), "global", Some(current_local))
            }
            InstallTarget::Local(workspace_path) => {
                let local_packages = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local_packages)?;

                let has_global_versions = check_global_versions(&global_cache, &safe_pkg_name)?;

                if has_global_versions {
                    fs::create_dir_all(&global_cache)?;
                    (global_cache.clone(), "global", Some(local_packages))
                } else {
                    (local_packages.clone(), "local", None)
                }
            }
        };

        // Extract tar.gz to temporary location first
        let tar = GzDecoder::new(&bytes[..]);
        let mut archive = Archive::new(tar);

        let temp_dir = std::env::temp_dir().join(format!(
            "horus_bin_{}_{}_{}",
            safe_pkg_name,
            std::process::id(),
            rand::random::<u32>()
        ));
        fs::create_dir_all(&temp_dir)?;
        let mut temp_guard = TempDirGuard::new(temp_dir.clone());

        // Safe extraction: validate each entry to prevent path traversal
        let canonical_temp = temp_dir.canonicalize()?;
        for entry in archive.entries()? {
            let mut entry = entry?;
            let entry_path = entry.path()?;
            let entry_str = entry_path.to_string_lossy();

            if entry_str.contains('\0') {
                log::warn!("Skipping tar entry with null byte in path: {:?}", entry_str);
                continue;
            }
            if entry_path.is_absolute() {
                log::warn!("Skipping absolute tar entry: {:?}", entry_str);
                continue;
            }
            if entry_path
                .components()
                .any(|c| matches!(c, std::path::Component::ParentDir))
            {
                log::warn!("Skipping tar entry with path traversal: {:?}", entry_str);
                continue;
            }
            if entry.header().entry_type().is_symlink()
                || entry.header().entry_type().is_hard_link()
            {
                if let Ok(Some(link_target)) = entry.link_name() {
                    let target_path = link_target.to_path_buf();
                    if target_path.is_absolute()
                        || target_path
                            .components()
                            .any(|c| matches!(c, std::path::Component::ParentDir))
                    {
                        log::warn!(
                            "Skipping symlink with unsafe target: {:?} -> {:?}",
                            entry_str,
                            target_path
                        );
                        continue;
                    }
                }
            }

            let dest = canonical_temp.join(
                entry_path
                    .components()
                    .filter(|c| !matches!(c, std::path::Component::CurDir))
                    .collect::<PathBuf>(),
            );
            if !dest.starts_with(&canonical_temp) {
                log::warn!(
                    "Skipping tar entry escaping extraction dir: {:?}",
                    entry_str
                );
                continue;
            }

            entry.unpack(&dest)?;
        }

        // Detect actual version for "latest" downloads
        let actual_version = if version_str == "latest" {
            detect_package_version(&temp_dir).ok_or_else(|| {
                anyhow!(
                    "Could not detect version for {} binary artifact.",
                    package_name
                )
            })?
        } else {
            version_str.to_string()
        };

        // Move to final location
        let package_dir = if install_type == "global" {
            install_dir.join(format!("{}@{}", safe_pkg_name, actual_version))
        } else {
            install_dir.join(&safe_pkg_name)
        };

        // Atomic install via staging directory
        let staging_dir = package_dir.with_extension("staging");
        if staging_dir.exists() {
            fs::remove_dir_all(&staging_dir)?;
        }
        fs::create_dir_all(&staging_dir)?;

        copy_dir_all(&temp_dir, &staging_dir)?;
        temp_guard.disarm();
        fs::remove_dir_all(&temp_dir)?;

        if package_dir.exists() {
            fs::remove_dir_all(&package_dir)?;
        }
        fs::rename(&staging_dir, &package_dir)?;

        // Create metadata.json
        let metadata = PackageMetadata {
            name: package_name.to_string(),
            version: actual_version.clone(),
            checksum: Some(actual_checksum),
        };
        let metadata_path = package_dir.join("metadata.json");
        fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        // Create symlink in local workspace if installed to global cache
        if install_type == "global" {
            if let Some(local_pkg_dir) = local_packages_dir {
                fs::create_dir_all(&local_pkg_dir)?;
                let local_link = local_pkg_dir.join(&safe_pkg_name);

                if (local_link.exists() || local_link.symlink_metadata().is_ok())
                    && fs::remove_file(&local_link).is_err()
                {
                    fs::remove_dir_all(&local_link)?;
                }

                #[cfg(unix)]
                horus_sys::fs::symlink(&package_dir, &local_link)?;
                #[cfg(windows)]
                std::os::windows::fs::symlink_dir(&package_dir, &local_link)?;

                finish_success(
                    spinner,
                    &format!(
                        "Installed {} v{} (pre-built for {})",
                        package_name, actual_version, target_triple
                    ),
                );
                println!(
                    "   {} Linked: {} -> {}",
                    "".dimmed(),
                    local_link.display(),
                    package_dir.display()
                );
            } else {
                finish_success(
                    spinner,
                    &format!(
                        "Installed {} v{} (pre-built for {})",
                        package_name, actual_version, target_triple
                    ),
                );
                println!("   {} Location: {}", "".dimmed(), package_dir.display());
            }
        } else {
            finish_success(
                spinner,
                &format!(
                    "Installed {} v{} (pre-built for {})",
                    package_name, actual_version, target_triple
                ),
            );
            println!("   {} Location: {}", "".dimmed(), package_dir.display());
        }

        // Resolve transitive dependencies
        if let Ok(deps) = extract_package_dependencies(&package_dir) {
            if !deps.is_empty() {
                println!("  {} Found {} dependencies", "".cyan(), deps.len());
                for dep in &deps {
                    println!("    {} {} {}", "".dimmed(), dep.name, dep.requirement);
                }
                self.install_dependencies(&deps, target)?;
            }
        }

        // Fetch and apply driver metadata if this is a driver package
        if let Some(driver_meta) = self.fetch_driver_metadata_opt(package_name) {
            self.apply_driver_requirements(&driver_meta, target)?;
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
                    println!("    - {}", feature.yellow());
                }

                // Update Cargo.toml if we have a workspace path
                if let Some(ws_path) = &workspace_path {
                    if let Err(e) = add_features_to_cargo_toml(ws_path, features) {
                        println!(
                            "  {} Could not auto-add features to Cargo.toml: {}",
                            crate::cli_output::ICON_WARN.yellow(),
                            e
                        );
                        println!(
                            "    Add manually: horus_library = {{ features = {:?} }}",
                            features
                        );
                    } else {
                        println!(
                            "  {} Added features to Cargo.toml",
                            crate::cli_output::ICON_SUCCESS.green()
                        );
                    }
                }
            }
        }

        // Add Cargo dependencies (crates.io)
        if let Some(cargo_deps) = &driver_meta.cargo_dependencies {
            if !cargo_deps.is_empty() {
                println!("  {} Cargo dependencies required:", "[PKG]".cyan());
                for dep in cargo_deps {
                    println!("    - {}", dep.yellow());
                }

                // Update Cargo.toml if we have a workspace path
                if let Some(ws_path) = &workspace_path {
                    if let Err(e) = add_cargo_deps_to_cargo_toml(ws_path, cargo_deps) {
                        println!(
                            "  {} Could not auto-add dependencies to Cargo.toml: {}",
                            crate::cli_output::ICON_WARN.yellow(),
                            e
                        );
                        println!("    Add manually to [dependencies]");
                    } else {
                        println!(
                            "  {} Added dependencies to Cargo.toml",
                            crate::cli_output::ICON_SUCCESS.green()
                        );
                    }
                }
            }
        }

        // Install Python dependencies (PyPI)
        if let Some(py_deps) = &driver_meta.python_dependencies {
            if !py_deps.is_empty() {
                println!("  {} Python dependencies required:", "[PY]".cyan());
                for dep in py_deps {
                    println!("    - {}", dep);
                }
                // Auto-install with pip
                let mut failed_deps = Vec::new();
                for dep in py_deps {
                    let status = std::process::Command::new("pip3")
                        .args(["install", "--quiet", dep])
                        .status();
                    match status {
                        Ok(s) if s.success() => {
                            println!(
                                "  {} Installed {} via pip",
                                crate::cli_output::ICON_SUCCESS.green(),
                                dep
                            );
                        }
                        Ok(s) => {
                            eprintln!(
                                "  {} Failed to install {} via pip (exit code: {})",
                                crate::cli_output::ICON_ERROR.red(),
                                dep,
                                s.code().unwrap_or(-1)
                            );
                            failed_deps.push(dep.as_str());
                        }
                        Err(e) => {
                            eprintln!(
                                "  {} Failed to run pip3 for {}: {}",
                                crate::cli_output::ICON_ERROR.red(),
                                dep,
                                e
                            );
                            failed_deps.push(dep.as_str());
                        }
                    }
                }
                if !failed_deps.is_empty() {
                    return Err(anyhow!(
                        "Failed to install {} Python dependenc{}: {}",
                        failed_deps.len(),
                        if failed_deps.len() == 1 { "y" } else { "ies" },
                        failed_deps.join(", ")
                    ));
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

    pub(crate) fn install_from_pypi(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        use std::process::Command;
        let spinner =
            progress::download_spinner(&format!("Installing {} from PyPI...", package_name));

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

        // Create temp venv for pip operations (with file lock to prevent concurrent races)
        let venv_parent = PathBuf::from(".horus");
        fs::create_dir_all(&venv_parent)?;
        let temp_venv = venv_parent.join("venv");
        if !temp_venv.exists() {
            let lock_path = venv_parent.join(".venv.lock");
            let lock_file = fs::OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open(&lock_path)?;
            #[cfg(unix)]
            {
                use std::os::unix::io::AsRawFd;
                // SAFETY: `lock_file` is a valid open `File`, so `as_raw_fd()`
                // returns a valid file descriptor. `flock` with `LOCK_EX` blocks
                // until the exclusive lock is acquired and does not modify the
                // file contents or invalidate the fd.
                unsafe {
                    libc::flock(lock_file.as_raw_fd(), libc::LOCK_EX);
                }
            }
            // Double-check after acquiring lock (another process may have created it)
            if !temp_venv.exists() {
                let python_cmd = if Command::new("python3").arg("--version").output().is_ok() {
                    "python3"
                } else {
                    "python"
                };
                Command::new(python_cmd)
                    .args(["-m", "venv", &*temp_venv.to_string_lossy()])
                    .status()?;
            }
            drop(lock_file);
        }

        let pip_path = temp_venv.join("bin/pip");

        // Build version string
        let version_str = version.unwrap_or("latest");
        let requirement = if version_str == "latest" {
            package_name.to_string()
        } else if version_str.starts_with(">=")
            || version_str.starts_with("<=")
            || version_str.starts_with('>')
            || version_str.starts_with('<')
            || version_str.starts_with("==")
            || version_str.starts_with("!=")
            || version_str.contains(',')
        {
            // Already a pip-compatible specifier (e.g., ">=1.2.0, <2.0.0")
            format!("{}{}", package_name, version_str)
        } else {
            // Bare version number — pin exactly
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

        // Update pyproject.toml if present in workspace
        if let crate::workspace::InstallTarget::Local(ref ws) = target {
            if let Err(e) = add_dep_to_pyproject_toml(ws, package_name, &actual_version) {
                log::warn!("Could not update pyproject.toml: {}", e);
            } else if ws.join("pyproject.toml").exists() {
                println!(
                    "  {} Updated pyproject.toml",
                    crate::cli_output::ICON_SUCCESS.green()
                );
            }
        }

        // Write tracking JSON
        let tracking_dir = match &target {
            crate::workspace::InstallTarget::Local(ws) => ws.join(".horus/packages"),
            crate::workspace::InstallTarget::Global => PathBuf::from(".horus/packages"),
        };
        fs::create_dir_all(&tracking_dir)?;
        let tracking_path = tracking_dir.join(format!("{}.pypi.json", package_name));
        let tracking = serde_json::json!({
            "name": package_name,
            "version": actual_version,
            "source": "PyPI",
        });
        fs::write(&tracking_path, serde_json::to_string_pretty(&tracking)?)?;

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
                horus_sys::fs::symlink(&pkg_dir, &local_link)?;
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

    pub(crate) fn install_from_cratesio(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        use std::process::Command;
        let spinner =
            progress::download_spinner(&format!("Installing {} from crates.io...", package_name));

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

        // Detect if this is a library crate (workspace has Cargo.toml) or binary.
        // Check root Cargo.toml first, then fall back to .horus/Cargo.toml
        // (generated for standalone .rs files by `horus run`).
        let workspace_cargo_toml = match &target {
            crate::workspace::InstallTarget::Local(workspace_path) => {
                let root = workspace_path.join("Cargo.toml");
                let horus_dir = workspace_path.join(".horus/Cargo.toml");
                if root.exists() {
                    Some(workspace_path.clone())
                } else if horus_dir.exists() {
                    Some(workspace_path.join(".horus"))
                } else {
                    None
                }
            }
            crate::workspace::InstallTarget::Global => None,
        };
        let is_library_install = workspace_cargo_toml.is_some();

        if is_library_install {
            // Library crate: use `cargo add` to add as a dependency to Cargo.toml
            let ws_path = workspace_cargo_toml.as_ref().unwrap();
            spinner.set_message(format!("Adding {} to Cargo.toml...", package_name));

            let mut cmd = Command::new("cargo");
            cmd.arg("add");
            if version_str != "latest" {
                cmd.arg(format!("{}@{}", package_name, version_str));
            } else {
                cmd.arg(package_name);
            }
            cmd.current_dir(ws_path);

            let output = cmd.output()?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                // If cargo add fails because it's not a lib (binary-only crate),
                // fall through to cargo install
                if !stderr.contains("doesn't have a library") {
                    finish_error(&spinner, &format!("cargo add failed for {}", package_name));
                    return Err(anyhow!("cargo add failed:\n{}", stderr));
                }
                // Fall through to binary install below
            } else {
                // Read actual version from Cargo.toml after cargo add
                let cargo_toml_path = ws_path.join("Cargo.toml");
                let actual_version = if let Ok(content) = fs::read_to_string(&cargo_toml_path) {
                    if let Ok(doc) = content.parse::<toml::Value>() {
                        doc.get("dependencies")
                            .and_then(|deps| deps.get(package_name))
                            .and_then(|dep| {
                                dep.as_str().map(|s| s.to_string()).or_else(|| {
                                    dep.get("version")
                                        .and_then(|v| v.as_str())
                                        .map(|s| s.to_string())
                                })
                            })
                            .unwrap_or_else(|| version_str.to_string())
                    } else {
                        version_str.to_string()
                    }
                } else {
                    version_str.to_string()
                };

                // Write tracking JSON
                let tracking_dir = ws_path.join(".horus/packages");
                fs::create_dir_all(&tracking_dir)?;
                let tracking_path = tracking_dir.join(format!("{}.crates-io.json", package_name));
                let tracking = serde_json::json!({
                    "name": package_name,
                    "version": actual_version,
                    "source": "CratesIO",
                    "install_type": "lib",
                });
                fs::write(&tracking_path, serde_json::to_string_pretty(&tracking)?)?;

                finish_success(
                    &spinner,
                    &format!("Added {} v{} to Cargo.toml", package_name, actual_version),
                );
                return Ok(actual_version);
            }
        }

        // Binary crate: use `cargo install`
        spinner.set_message(format!("Installing {} with cargo...", package_name));

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

        // Write tracking JSON for binary installs too
        let tracking_dir = match &target {
            crate::workspace::InstallTarget::Local(ws) => ws.join(".horus/packages"),
            crate::workspace::InstallTarget::Global => PathBuf::from(".horus/packages"),
        };
        fs::create_dir_all(&tracking_dir)?;
        let tracking_path = tracking_dir.join(format!("{}.crates-io.json", package_name));
        let tracking = serde_json::json!({
            "name": package_name,
            "version": actual_version,
            "source": "CratesIO",
            "install_type": "bin",
        });
        fs::write(&tracking_path, serde_json::to_string_pretty(&tracking)?)?;

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
                horus_sys::fs::symlink(&pkg_dir, &local_link)?;
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

                // Fallback: install using dependency specs (preserves version constraints and source)
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

                    println!("  {} Installing dependency: {}...", "".cyan(), dep_name);
                    self.install_dependency_spec(&dep, target.clone(), None)?;
                }
                return Ok(());
            }
        };

        // Install resolved versions
        for resolved_dep in resolved {
            let version_str = resolved_dep.version.to_string();
            let home = dirs::home_dir().ok_or_else(|| anyhow!("could not find home directory"))?;
            let global_cache = home.join(".horus/cache");

            // Check if already installed at a version satisfying the requirement
            let version_req = semver::VersionReq::parse(&format!("={}", resolved_dep.version))
                .unwrap_or(semver::VersionReq::STAR);
            let (is_installed_local, is_installed_global) = match &target {
                crate::workspace::InstallTarget::Global => {
                    let has_global = check_global_version_satisfies(
                        &global_cache,
                        &resolved_dep.name,
                        Some(&version_req),
                    )?;
                    (has_global, has_global)
                }
                crate::workspace::InstallTarget::Local(workspace_path) => {
                    let local_packages = workspace_path.join(".horus/packages");
                    let has_local = local_packages.join(&resolved_dep.name).exists();
                    let has_global = check_global_version_satisfies(
                        &global_cache,
                        &resolved_dep.name,
                        Some(&version_req),
                    )
                    .unwrap_or(false);
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
                        horus_sys::fs::symlink(&global_package_dir, &local_link)?;
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
            // Resolve relative to base_dir (horus.toml location) or current directory
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
        horus_sys::fs::symlink(&source_path, &link_path)?;
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
            crate::cli_output::ICON_INFO.cyan()
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

        // Fallback: check site-packages directly by globbing python3.* dirs
        let mut site_packages_paths = Vec::new();
        let site_pkg_roots = [PathBuf::from("/usr/lib"), PathBuf::from("/usr/local/lib")];
        for root in &site_pkg_roots {
            if let Ok(entries) = fs::read_dir(root) {
                for entry in entries.flatten() {
                    let name = entry.file_name();
                    let name_str = name.to_string_lossy();
                    if name_str.starts_with("python3") && entry.path().is_dir() {
                        site_packages_paths
                            .push(entry.path().join("site-packages").join(package_name));
                    }
                }
            }
        }
        // Also check user-local site-packages
        if let Some(home) = dirs::home_dir() {
            let local_lib = home.join(".local/lib");
            if let Ok(entries) = fs::read_dir(&local_lib) {
                for entry in entries.flatten() {
                    let name = entry.file_name();
                    let name_str = name.to_string_lossy();
                    if name_str.starts_with("python3") && entry.path().is_dir() {
                        site_packages_paths
                            .push(entry.path().join("site-packages").join(package_name));
                    }
                }
            }
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
        println!(
            "  [3] {} Cancel installation",
            crate::cli_output::ICON_ERROR.red()
        );

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
        let cargo_bin = crate::paths::home_dir()?
            .join(".cargo/bin")
            .join(package_name);

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
        horus_sys::fs::symlink(&cargo_bin, &bin_link)?;
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
