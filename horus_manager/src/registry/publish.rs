use super::helpers::*;
use super::*;
use std::cmp::Reverse;
use std::io::IsTerminal;
use walkdir::WalkDir;

/// Build a user-friendly error from an HTTP response status and optional body text.
/// `action` describes what the user was trying to do (e.g., "publish package", "search drivers").
fn registry_error(status: reqwest::StatusCode, body: &str, action: &str) -> anyhow::Error {
    let hint = match status {
        s if s == reqwest::StatusCode::UNAUTHORIZED => {
            "Your API key is invalid or expired.\n  Fix: run `horus auth login` to get a new key."
                .to_string()
        }
        s if s == reqwest::StatusCode::FORBIDDEN => {
            "You don't have permission for this action.\n  Check your API key scope or package ownership."
                .to_string()
        }
        s if s == reqwest::StatusCode::NOT_FOUND => {
            "The requested resource was not found on the registry.".to_string()
        }
        s if s == reqwest::StatusCode::TOO_MANY_REQUESTS => {
            "Rate limit exceeded. Please wait a few minutes and try again.".to_string()
        }
        s if s == reqwest::StatusCode::CONFLICT => {
            "A conflicting version already exists. Bump your version and try again.".to_string()
        }
        s if s.is_server_error() => {
            "The registry server encountered an error. This is likely temporary.\n  Try again in a few minutes. If the problem persists, check https://status.horusrobotics.dev"
                .to_string()
        }
        _ => String::new(),
    };

    let detail = if body.is_empty() || body == "Unknown error" {
        String::new()
    } else {
        format!("\n  Server response: {}", body)
    };

    if hint.is_empty() {
        anyhow!("Failed to {}: HTTP {}{}", action, status.as_u16(), detail)
    } else {
        anyhow!(
            "Failed to {}: {}{}\n  (HTTP {})",
            action,
            hint,
            detail,
            status.as_u16()
        )
    }
}

/// Read the response body text, returning empty string on failure instead of "Unknown error".
fn read_response_body(response: reqwest::blocking::Response) -> (reqwest::StatusCode, String) {
    let status = response.status();
    let body = response.text().unwrap_or_default();
    (status, body)
}

/// Strip `[patch.*]` sections from Cargo.toml content.
///
/// Published packages must not contain `[patch.crates-io]` entries with local
/// path overrides (e.g., `path = "../horus/horus_core"`) because those paths
/// don't exist on the user's machine. The install system injects correct
/// overrides via `inject_horus_path_overrides()` at build time.
fn strip_cargo_patch_sections(content: &str) -> String {
    let mut result = String::with_capacity(content.len());
    let mut in_patch_section = false;

    for line in content.lines() {
        let trimmed = line.trim();

        // Detect start of [patch.*] section
        if trimmed.starts_with("[patch.") || trimmed.starts_with("[patch]") {
            in_patch_section = true;
            continue;
        }

        // Detect start of a new top-level section (exits patch section)
        if in_patch_section && trimmed.starts_with('[') && !trimmed.starts_with("[patch") {
            in_patch_section = false;
        }

        if !in_patch_section {
            result.push_str(line);
            result.push('\n');
        }
    }

    // Remove trailing blank lines left by stripping
    while result.ends_with("\n\n") {
        result.pop();
    }

    result
}

/// Default patterns excluded from published tarballs (security + size)
const DEFAULT_EXCLUDES: &[&str] = &[
    ".git",
    "target",
    ".horus",
    "node_modules",
    "__pycache__",
    ".env",
    ".venv",
    "venv",
    ".tox",
    ".mypy_cache",
    ".pytest_cache",
];

/// File patterns that are always excluded (secrets)
const SECRET_PATTERNS: &[&str] = &[
    ".env",
    ".env.local",
    ".env.production",
    ".env.development",
    "signing_key",
    "id_rsa",
    "id_ed25519",
    "id_ecdsa",
    "credentials.json",
    ".aws",
];

/// Check if a path component matches any exclude pattern
fn should_exclude(
    path: &std::path::Path,
    base: &std::path::Path,
    custom_excludes: &[String],
) -> bool {
    let relative = path.strip_prefix(base).unwrap_or(path);

    for component in relative.components() {
        let name = component.as_os_str().to_string_lossy();

        // Check default directory excludes
        for pattern in DEFAULT_EXCLUDES {
            if name == *pattern {
                return true;
            }
        }

        // Check .env file variants
        if name.starts_with(".env") {
            return true;
        }

        // Check custom excludes from .horusignore
        for pattern in custom_excludes {
            if name == *pattern {
                return true;
            }
            // Simple glob: *.ext matching
            if let Some(ext_pattern) = pattern.strip_prefix("*.") {
                if let Some(ext) = Path::new(&*name).extension() {
                    if ext.to_string_lossy() == ext_pattern {
                        return true;
                    }
                }
            }
        }
    }

    // Check secret file patterns against the file name
    if let Some(file_name) = path.file_name() {
        let name = file_name.to_string_lossy();
        for pattern in SECRET_PATTERNS {
            if name == *pattern {
                return true;
            }
        }
        // Key files
        if name.ends_with(".pem") || name.ends_with(".key") || name.ends_with(".p12") {
            return true;
        }
    }

    false
}

/// Load .horusignore patterns from the project directory
fn load_horusignore(dir: &std::path::Path) -> Vec<String> {
    let ignore_path = dir.join(".horusignore");
    if !ignore_path.exists() {
        return vec![];
    }
    match fs::read_to_string(&ignore_path) {
        Ok(content) => content
            .lines()
            .map(|l| l.trim())
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
            .map(|l| l.to_string())
            .collect(),
        Err(_) => vec![],
    }
}

impl RegistryClient {
    /// Fetch driver metadata for a package from the registry
    /// Returns error if the package is not a driver or not found in the registry
    pub fn fetch_driver_metadata(&self, package_name: &str) -> Result<DriverMetadata> {
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/drivers/{}", self.base_url, encoded_name);

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to fetch driver metadata: {}", e))?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("fetch driver metadata for '{}'", package_name),
            ));
        }

        let resp: DriverMetadataResponse = response
            .json()
            .map_err(|e| anyhow!("failed to parse driver metadata: {}", e))?;

        resp.driver_metadata
            .ok_or_else(|| anyhow!("No driver metadata for '{}'", package_name))
    }

    /// Fetch driver metadata, returning Option (for backwards compatibility)
    pub fn fetch_driver_metadata_opt(&self, package_name: &str) -> Option<DriverMetadata> {
        self.fetch_driver_metadata(package_name).ok()
    }

    /// Fetch driver metadata by querying the drivers search API
    pub fn query_driver_features(&self, driver_name: &str) -> Option<Vec<String>> {
        if let Some(meta) = self.fetch_driver_metadata_opt(driver_name) {
            return meta.required_features;
        }

        let url = format!("{}/api/drivers/search", self.base_url);
        if let Ok(response) = self.client.get(&url).query(&[("q", driver_name)]).send() {
            if response.status().is_success() {
                if let Ok(resp) = response.json::<DriverSearchResponse>() {
                    for driver in resp.results {
                        if driver.name.eq_ignore_ascii_case(driver_name) {
                            return driver.driver_metadata.and_then(|m| m.required_features);
                        }
                    }
                }
            }
        }

        None
    }

    pub fn publish(&self, path: Option<&Path>, dry_run: bool, org: Option<&str>) -> Result<()> {
        let current_dir = path.unwrap_or_else(|| Path::new("."));

        let manifest = detect_package_info(current_dir)?;
        let name = manifest.name;
        let version = manifest.version;
        let description = manifest.description;
        let license = manifest.license;

        // Validate package name before proceeding
        validate_package_name(&name)?;

        println!(
            " Detected {} manifest",
            format!("{}", manifest.manifest_format).cyan()
        );

        match &manifest.manifest_format {
            ManifestFormat::HorusToml => {
                // Check for path dependencies in horus.toml — cannot publish those
                use crate::manifest::{HorusManifest, HORUS_TOML};
                let horus_path = current_dir.join(HORUS_TOML);
                if horus_path.exists() {
                    if let Ok(horus_manifest) = HorusManifest::load_from(&horus_path) {
                        let mut has_path_deps = false;
                        for (dep_name, dep_value) in &horus_manifest.dependencies {
                            if dep_value.is_path() {
                                println!(
                                    "\n{} Cannot publish package with path dependencies!",
                                    "Error:".red()
                                );
                                println!("  Path dependency: {}", dep_name);
                                has_path_deps = true;
                            }
                        }
                        if has_path_deps {
                            println!(
                                "\n{}",
                                "Path dependencies are not reproducible and cannot be published."
                                    .yellow()
                            );
                            println!("{}", "Please publish path dependencies to the registry first, then use version requirements in horus.toml.".yellow());
                            return Err(anyhow!("Cannot publish package with path dependencies"));
                        }
                    }
                }
            }
            ManifestFormat::CargoToml => {
                let cargo_path = current_dir.join(CARGO_TOML);
                let content = fs::read_to_string(&cargo_path)?;
                let toml_value: toml::Value = toml::from_str(&content)?;

                if let Some(deps) = toml_value.get("dependencies").and_then(|v| v.as_table()) {
                    let mut has_path_deps = false;
                    for (dep_name, dep_value) in deps {
                        if let Some(table) = dep_value.as_table() {
                            if table.contains_key("path") {
                                println!(
                                    "\n{} Cannot publish package with path dependencies!",
                                    "Error:".red()
                                );
                                let path_val =
                                    table.get("path").and_then(|v| v.as_str()).unwrap_or("?");
                                println!("  Path dependency: {} -> {}", dep_name, path_val);
                                has_path_deps = true;
                            }
                        }
                    }
                    if has_path_deps {
                        println!(
                            "\n{}",
                            "Path dependencies are not reproducible and cannot be published."
                                .yellow()
                        );
                        println!("{}", "Please publish path dependencies to the registry first, then use version requirements in Cargo.toml.".yellow());
                        return Err(anyhow!("Cannot publish package with path dependencies"));
                    }
                }
            }
            ManifestFormat::PackageJson => {
                let pkg_path = current_dir.join("package.json");
                let content = fs::read_to_string(&pkg_path)?;
                let json: serde_json::Value = serde_json::from_str(&content)?;

                if let Some(deps) = json.get("dependencies").and_then(|v| v.as_object()) {
                    let mut has_path_deps = false;
                    for (dep_name, dep_value) in deps {
                        if let Some(val) = dep_value.as_str() {
                            if val.starts_with("file:") {
                                println!(
                                    "\n{} Cannot publish package with file dependencies!",
                                    "Error:".red()
                                );
                                println!("  File dependency: {} -> {}", dep_name, val);
                                has_path_deps = true;
                            }
                        }
                    }
                    if has_path_deps {
                        println!(
                            "\n{}",
                            "File dependencies are not reproducible and cannot be published."
                                .yellow()
                        );
                        println!("{}", "Please publish file dependencies to the registry first, then use version ranges in package.json.".yellow());
                        return Err(anyhow!("Cannot publish package with file dependencies"));
                    }
                }
            }
        }

        println!(" Publishing {} v{}...", name, version);

        let api_key = match get_api_key() {
            Ok(key) => key,
            Err(_) => {
                println!("\n Not authenticated with HORUS registry.");
                println!("\nTo publish packages, you need to authenticate:");
                println!("  1. Run: horus auth login");
                println!("  2. Authorize in your browser");
                println!("  3. The registry will show your API key");
                println!("  4. Save it to ~/.horus/auth.json");
                println!("\nThen try publishing again!");
                return Err(anyhow!("authentication required"));
            }
        };

        let safe_name = package_name_to_path(&name);
        let tar_path = std::env::temp_dir().join(format!("{}-{}.tar.gz", safe_name, version));

        let included_files: Vec<(String, u64)>;
        {
            let custom_excludes = load_horusignore(current_dir);
            let tar_file = fs::File::create(&tar_path)?;
            let encoder = GzEncoder::new(tar_file, Compression::default());
            let mut tar_builder = Builder::new(encoder);
            let mut file_count = 0u64;
            let mut files_list: Vec<(String, u64)> = Vec::new();

            for entry in WalkDir::new(current_dir)
                .follow_links(false)
                .into_iter()
                .filter_entry(|e| !should_exclude(e.path(), current_dir, &custom_excludes))
            {
                let entry = entry?;
                let path = entry.path();
                let relative = path.strip_prefix(current_dir).unwrap_or(path);

                // Skip the root directory itself
                if relative == Path::new("") {
                    continue;
                }

                if path.is_file() {
                    let rel_str = relative.to_string_lossy();

                    // Strip [patch.*] sections from Cargo.toml before packaging.
                    // These contain local path overrides (e.g., path = "../horus/horus_core")
                    // that don't exist on the user's machine. The install system injects
                    // correct overrides via inject_horus_path_overrides() at build time.
                    if rel_str == "Cargo.toml" {
                        let content = fs::read_to_string(path)?;
                        let cleaned = strip_cargo_patch_sections(&content);
                        let data = cleaned.as_bytes();
                        let mut header = tar::Header::new_gnu();
                        header.set_size(data.len() as u64);
                        header.set_mode(0o644);
                        header.set_mtime(
                            path.metadata()
                                .and_then(|m| m.modified())
                                .ok()
                                .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
                                .map(|d| d.as_secs())
                                .unwrap_or(0),
                        );
                        header.set_cksum();
                        tar_builder.append_data(&mut header, relative, data)?;
                        files_list.push((rel_str.to_string(), data.len() as u64));
                        file_count += 1;
                    } else {
                        let file_size = path.metadata().map(|m| m.len()).unwrap_or(0);
                        files_list.push((rel_str.to_string(), file_size));
                        tar_builder.append_path_with_name(path, relative)?;
                        file_count += 1;
                    }
                } else if path.is_dir() {
                    tar_builder.append_dir(relative, path)?;
                }
            }

            tar_builder.finish()?;
            println!(
                "   Packaged {} files (excluded .git, target, .env, etc.)",
                file_count
            );
            included_files = files_list;
        }

        // Safety scan: check for accidentally included secrets
        // Uses SECRET_PATTERNS plus extension-based checks for comprehensive coverage
        let secret_files: Vec<&str> = included_files
            .iter()
            .filter(|(path, _)| {
                let p = Path::new(path);
                let name = p
                    .file_name()
                    .map(|n| n.to_string_lossy())
                    .unwrap_or_default();
                // Check all SECRET_PATTERNS
                SECRET_PATTERNS.iter().any(|pat| {
                    name == *pat || name.starts_with(&format!("{}.", pat))
                })
                // Also catch secret file extensions not in SECRET_PATTERNS
                    || name.ends_with(".pem")
                    || name.ends_with(".key")
                    || name.ends_with(".p12")
            })
            .map(|(path, _)| path.as_str())
            .collect();

        if !secret_files.is_empty() {
            println!(
                "\n{} Potential secrets detected in package!",
                crate::cli_output::ICON_ERROR.red()
            );
            for f in &secret_files {
                println!("   - {}", f);
            }
            println!("\nAdd these to .horusignore or remove them from your project.");
            return Err(anyhow!(
                "Publish aborted: {} potential secret file(s) detected. Use .horusignore to exclude them.",
                secret_files.len()
            ));
        }

        let package_data = fs::read(&tar_path)?;
        fs::remove_file(&tar_path)?;

        let size_mb = package_data.len() as f64 / (1024.0 * 1024.0);
        println!("   Package size: {:.2} MB", size_mb);
        if size_mb > 40.0 {
            println!(
                "   {} Package is large ({:.1} MB). Registry limit is 50 MB.",
                crate::cli_output::ICON_WARN.yellow(),
                size_mb
            );
            println!("   Consider adding exclusions to .horusignore");
        }

        let resolved_license = match license {
            Some(l) => l,
            None => {
                let default = "Apache-2.0".to_string();
                println!(
                    "  {} No license specified, defaulting to {}. Set 'license' in your manifest to change this.",
                    crate::cli_output::ICON_WARN.yellow(),
                    default.cyan()
                );
                default
            }
        };

        let mut form = reqwest::blocking::multipart::Form::new()
            .text("name", name.clone())
            .text("version", version.clone())
            .text("description", description.unwrap_or_default())
            .text("license", resolved_license)
            .part(
                "package",
                reqwest::blocking::multipart::Part::bytes(package_data.clone())
                    .file_name(format!("{}-{}.tar.gz", safe_name, version)),
            );

        // Resolve metadata: manifest values as defaults, interactive prompts as overrides
        let manifest_source_url = manifest.source_url.clone().unwrap_or_default();
        let manifest_categories = manifest.categories.clone().unwrap_or_default();
        let manifest_package_type = manifest.package_type.clone().unwrap_or_default();

        let (docs_url, docs_type, final_source_url, final_categories, final_package_type) =
            if std::io::stdin().is_terminal() && !dry_run {
                println!("\n{}", "[#] Package Metadata (optional)".cyan().bold());
                println!("   Help users discover and use your package by adding:");
                let (du, dt, psu, pc, ppt) = prompt_package_metadata(current_dir)?;
                (
                    du,
                    dt,
                    if !psu.is_empty() {
                        psu
                    } else {
                        manifest_source_url
                    },
                    if !pc.is_empty() {
                        pc
                    } else {
                        manifest_categories
                    },
                    if !ppt.is_empty() {
                        ppt
                    } else {
                        manifest_package_type
                    },
                )
            } else {
                (
                    String::new(),
                    String::new(),
                    manifest_source_url,
                    manifest_categories,
                    manifest_package_type,
                )
            };

        if !final_categories.is_empty() {
            form = form.text("categories", final_categories.clone());
        }
        if !final_package_type.is_empty() {
            form = form.text("package_type", final_package_type.clone());
        }
        if !final_source_url.is_empty() {
            form = form.text("source_url", final_source_url.clone());
        }
        if !docs_url.is_empty() {
            form = form.text("docs_url", docs_url.clone());
        }
        if !docs_type.is_empty() {
            form = form.text("docs_type", docs_type.clone());
        }

        // Extended metadata from [package.metadata.horus]
        if let Some(ref pt) = manifest.plugin_type {
            form = form.text("plugin_type", pt.clone());
        }
        if let Some(ref entry) = manifest.plugin_entry {
            form = form.text("plugin_entry", entry.clone());
        }
        if let Some(ref dc) = manifest.driver_category {
            form = form.text("driver_category", dc.clone());
        }
        if let Some(ref bt) = manifest.bus_type {
            form = form.text("bus_type", bt.clone());
        }
        if let Some(ref ot) = manifest.output_type {
            form = form.text("output_type", ot.clone());
        }
        if let Some(ref it) = manifest.input_type {
            form = form.text("input_type", it.clone());
        }
        if let Some(ref pl) = manifest.platforms {
            form = form.text("platforms", pl.clone());
        }

        if dry_run {
            form = form.text("dry_run", "true");
            println!(
                " {} Running in dry-run mode (no changes will be made)",
                crate::cli_output::ICON_INFO.cyan()
            );

            // Validate that [patch] sections were stripped from the tarball's Cargo.toml
            if current_dir.join(CARGO_TOML).exists() {
                let cargo_content = fs::read_to_string(current_dir.join(CARGO_TOML))?;
                let cleaned = strip_cargo_patch_sections(&cargo_content);
                if cleaned.len() < cargo_content.len() {
                    let stripped_lines = cargo_content.lines().count() - cleaned.lines().count();
                    println!(
                        "   {} Stripped [patch] section ({} lines) from Cargo.toml in tarball",
                        crate::cli_output::ICON_SUCCESS.green(),
                        stripped_lines
                    );
                }
                // Check for remaining path dependencies that would break remote builds
                if cleaned.contains("path = \"") {
                    println!(
                        "   {} Warning: Cargo.toml still contains path dependencies after stripping [patch].",
                        crate::cli_output::ICON_WARN.yellow()
                    );
                    println!("     These may prevent the package from building on user machines.");
                }
            }

            // Show file listing in dry-run mode
            println!("\n{}", "   Files to be published:".cyan());
            let mut sorted_files = included_files.clone();
            sorted_files.sort_by_key(|item| Reverse(item.1)); // Sort by size descending
            let show_count = sorted_files.len().min(20);
            for (path, size) in &sorted_files[..show_count] {
                if *size > 1024 * 1024 {
                    println!("     {:.1} MB  {}", *size as f64 / (1024.0 * 1024.0), path);
                } else if *size > 1024 {
                    println!("     {:.1} KB  {}", *size as f64 / 1024.0, path);
                } else {
                    println!("     {} B   {}", size, path);
                }
            }
            if sorted_files.len() > 20 {
                println!("     ... and {} more files", sorted_files.len() - 20);
            }
            let total_uncompressed: u64 = included_files.iter().map(|(_, s)| s).sum();
            println!(
                "\n   Total uncompressed: {:.2} MB | Compressed: {:.2} MB",
                total_uncompressed as f64 / (1024.0 * 1024.0),
                size_mb
            );
        }

        if let Some(org_name) = org {
            form = form.text("organization", org_name.to_string());
            println!(
                " {} Publishing to organization: {}",
                crate::cli_output::ICON_INFO.cyan(),
                org_name.cyan()
            );
        }

        let signing_key_path = crate::paths::keys_dir()?.join("signing_key");
        if signing_key_path.exists() {
            use ed25519_dalek::{Signer, SigningKey};

            let key_bytes = fs::read(&signing_key_path)?;
            if key_bytes.len() == 32 {
                let signing_key = SigningKey::from_bytes(
                    key_bytes
                        .as_slice()
                        .try_into()
                        .map_err(|_| anyhow!("Invalid signing key format"))?,
                );
                let signature = signing_key.sign(&package_data);
                let sig_hex = hex::encode(signature.to_bytes());
                form = form.text("signature", sig_hex);
                println!(
                    " {} Package signed with Ed25519 key",
                    crate::cli_output::ICON_SUCCESS.green()
                );
            } else {
                log::warn!("Signing key has invalid length, skipping signature");
                eprintln!(
                    " {} Signing key has invalid length, skipping signature",
                    crate::cli_output::ICON_WARN.yellow()
                );
            }
        }

        let upload_spinner = progress::spinner(&format!(
            "Uploading {} v{} ({:.1} MB)...",
            name, version, size_mb
        ));

        let response = self
            .client
            .post(format!("{}/api/packages/upload", self.base_url))
            .header("Authorization", format!("Bearer {}", api_key))
            .multipart(form)
            .send();

        match response {
            Ok(ref r) if r.status().is_success() => {
                progress::finish_success(
                    &upload_spinner,
                    &format!("Uploaded {} v{}", name, version),
                );
            }
            Ok(_) => {
                progress::finish_error(&upload_spinner, "Upload failed");
            }
            Err(ref e) => {
                progress::finish_error(&upload_spinner, &format!("Upload error: {}", e));
            }
        }

        let response = response?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(status, &body, "publish package"));
        }

        let response_text = response.text().unwrap_or_default();
        let response_json: serde_json::Value =
            serde_json::from_str(&response_text).unwrap_or(serde_json::json!({}));

        if response_json.get("success") == Some(&serde_json::json!(false)) {
            let error_type = response_json
                .get("error")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let message = response_json
                .get("message")
                .and_then(|v| v.as_str())
                .unwrap_or("Package verification failed");

            match error_type {
                // duplicate_version is now returned as HTTP 409 CONFLICT and caught
                // by the status check above, handled via registry_error().
                "verification_failed" => {
                    println!("\n{} Package verification failed!", "Error:".red());
                    println!("   {}", message);
                    println!(
                        "\n{}",
                        "The package failed pre-upload verification.".yellow()
                    );
                    println!("{}", "Please fix the issues above and try again.".yellow());

                    if let Some(warnings) = response_json.get("warnings").and_then(|v| v.as_array())
                    {
                        if !warnings.is_empty() {
                            println!("\n{}", "Warnings:".yellow());
                            for warning in warnings {
                                if let Some(w) = warning.as_str() {
                                    println!("   - {}", w);
                                }
                            }
                        }
                    }
                    return Err(anyhow!("Package verification failed: {}", message));
                }
                _ => {
                    println!("\n{} {}", "Error:".red(), message);
                    return Err(anyhow!("Publish failed: {}", message));
                }
            }
        }

        if dry_run {
            if response_json.get("dry_run") == Some(&serde_json::json!(true)) {
                println!(
                    " {} Dry run passed! Package {} v{} is valid and ready to publish.",
                    crate::cli_output::ICON_SUCCESS.green(),
                    name,
                    version
                );
                if let Some(size) = response_json.get("size").and_then(|v| v.as_u64()) {
                    println!("   Package size: {:.2} MB", size as f64 / (1024.0 * 1024.0));
                }
            }
            return Ok(());
        }

        println!(" Published {} v{} successfully!", name, version);

        if let Some(verification) = response_json.get("verification") {
            let status = verification
                .get("status")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let job_id = verification.get("job_id").and_then(|v| v.as_str());

            match status {
                "pending" => {
                    println!(
                        "\n{} Build verification in progress...",
                        crate::cli_output::ICON_INFO.cyan()
                    );
                    if let Some(id) = job_id {
                        println!(
                            "   Track status: horus pkg status {} v{} --verification",
                            name, version
                        );
                        println!("   Job ID: {}", id);
                    }
                    println!(
                        "\n{}",
                        "Your package will be fully available once verification completes.".cyan()
                    );
                }
                "passed" => {
                    println!(
                        "\n{} Build verification passed!",
                        crate::cli_output::ICON_SUCCESS.green()
                    );
                }
                "failed" => {
                    println!(
                        "\n{} Build verification failed!",
                        crate::cli_output::ICON_ERROR.red()
                    );
                    if let Some(msg) = verification.get("message").and_then(|v| v.as_str()) {
                        println!("   {}", msg);
                    }
                }
                _ => {}
            }
        }
        let encoded_name = url_encode_package_name(&name);
        println!("   View at: {}/packages/{}", self.base_url, encoded_name);

        Ok(())
    }

    pub fn unpublish(&self, package_name: &str, version: &str) -> Result<()> {
        let api_key = match get_api_key() {
            Ok(key) => key,
            Err(_) => {
                println!("\n Not authenticated with HORUS registry.");
                println!("\nTo unpublish packages, you need to authenticate:");
                println!("  1. Run: horus auth login");
                println!("  2. Authorize in your browser");
                println!("  3. The registry will show your API key");
                println!("  4. Save it to ~/.horus/auth.json");
                return Err(anyhow!("authentication required"));
            }
        };

        let encoded_name = url_encode_package_name(package_name);
        let url = format!(
            "{}/api/packages/{}/{}",
            self.base_url, encoded_name, version
        );
        let response = self
            .client
            .delete(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("unpublish {} v{}", package_name, version),
            ));
        }

        Ok(())
    }

    pub fn search(
        &self,
        query: &str,
        package_type: Option<&str>,
        category: Option<&str>,
    ) -> Result<Vec<Package>> {
        let url = format!("{}/api/packages/search", self.base_url);
        let mut params: Vec<(&str, &str)> = vec![("q", query)];
        if let Some(pt) = package_type {
            params.push(("type", pt));
        }
        if let Some(cat) = category {
            params.push(("category", cat));
        }

        let response = self.client.get(&url).query(&params).send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(status, &body, "search packages"));
        }

        #[derive(Deserialize)]
        struct SearchResponse {
            results: Vec<SearchResultItem>,
        }

        #[derive(Deserialize)]
        struct SearchResultItem {
            name: String,
            version: Option<String>,
            metadata: Option<SearchMetadata>,
        }

        #[derive(Deserialize)]
        struct SearchMetadata {
            description: Option<String>,
        }

        let search_response: SearchResponse = response.json()?;

        let packages = search_response
            .results
            .into_iter()
            .map(|item| Package {
                name: item.name,
                version: item.version.unwrap_or_else(|| "latest".to_string()),
                description: item.metadata.and_then(|m| m.description),
            })
            .collect();

        Ok(packages)
    }

    // ── Yank / Unyank ──────────────────────────────────────────────────

    pub fn yank(&self, package_name: &str, version: &str, reason: Option<&str>) -> Result<()> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!(
            "{}/api/packages/{}/{}/yank",
            self.base_url, encoded_name, version
        );

        let mut req = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key));
        if let Some(r) = reason {
            req = req.json(&serde_json::json!({"reason": r}));
        }
        let response = req.send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("yank {} v{}", package_name, version),
            ));
        }
        Ok(())
    }

    pub fn unyank(&self, package_name: &str, version: &str) -> Result<()> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!(
            "{}/api/packages/{}/{}/unyank",
            self.base_url, encoded_name, version
        );

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("unyank {} v{}", package_name, version),
            ));
        }
        Ok(())
    }

    // ── Deprecate / Undeprecate ─────────────────────────────────────────

    pub fn deprecate(&self, package_name: &str, message: Option<&str>) -> Result<()> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}/deprecate", self.base_url, encoded_name);

        let mut body = serde_json::json!({});
        if let Some(msg) = message {
            body["message"] = serde_json::Value::String(msg.to_string());
        }

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .json(&body)
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("deprecate {}", package_name),
            ));
        }
        Ok(())
    }

    pub fn undeprecate(&self, package_name: &str) -> Result<()> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!(
            "{}/api/packages/{}/undeprecate",
            self.base_url, encoded_name
        );

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("undeprecate {}", package_name),
            ));
        }
        Ok(())
    }

    // ── Owner management ────────────────────────────────────────────────

    pub fn list_owners(&self, package_name: &str) -> Result<Vec<serde_json::Value>> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}/owners", self.base_url, encoded_name);

        let response = self
            .client
            .get(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("list owners of {}", package_name),
            ));
        }

        let data: serde_json::Value = response.json()?;
        Ok(data
            .get("owners")
            .and_then(|v| v.as_array())
            .cloned()
            .unwrap_or_default())
    }

    pub fn add_owner(&self, package_name: &str, user_id: &str) -> Result<()> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}/owners", self.base_url, encoded_name);

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .json(&serde_json::json!({"user_id": user_id}))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("add owner to {}", package_name),
            ));
        }
        Ok(())
    }

    pub fn remove_owner(&self, package_name: &str, user_id: &str) -> Result<()> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!(
            "{}/api/packages/{}/owners/{}",
            self.base_url, encoded_name, user_id
        );

        let response = self
            .client
            .delete(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("remove owner from {}", package_name),
            ));
        }
        Ok(())
    }

    // ── Transfer ────────────────────────────────────────────────────────

    pub fn transfer_to_user(
        &self,
        package_name: &str,
        target_username: &str,
    ) -> Result<serde_json::Value> {
        let api_key = get_api_key()?;
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}/transfer", self.base_url, encoded_name);

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .json(&serde_json::json!({"target_username": target_username}))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("transfer {}", package_name),
            ));
        }
        Ok(response.json()?)
    }

    pub fn transfer_to_org(&self, package_name: &str, org: &str) -> Result<serde_json::Value> {
        let api_key = get_api_key()?;
        let url = format!("{}/api/orgs/{}/transfer-package", self.base_url, org);

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .json(&serde_json::json!({"package_name": package_name}))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(
                status,
                &body,
                &format!("transfer {} to org {}", package_name, org),
            ));
        }
        Ok(response.json()?)
    }

    pub fn pending_transfers(&self) -> Result<Vec<serde_json::Value>> {
        let api_key = get_api_key()?;
        let url = format!("{}/api/user/transfers", self.base_url);

        let response = self
            .client
            .get(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(status, &body, "list pending transfers"));
        }

        let data: serde_json::Value = response.json()?;
        Ok(data
            .get("transfers")
            .and_then(|v| v.as_array())
            .cloned()
            .unwrap_or_default())
    }

    pub fn accept_transfer(&self, transfer_id: &str) -> Result<()> {
        let api_key = get_api_key()?;
        let url = format!(
            "{}/api/user/transfers/{}/accept",
            self.base_url, transfer_id
        );

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(status, &body, "accept transfer"));
        }
        Ok(())
    }

    pub fn reject_transfer(&self, transfer_id: &str) -> Result<()> {
        let api_key = get_api_key()?;
        let url = format!(
            "{}/api/user/transfers/{}/reject",
            self.base_url, transfer_id
        );

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let (status, body) = read_response_body(response);
            return Err(registry_error(status, &body, "reject transfer"));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    // ── registry_error formatting ───────────────────────────────────────

    #[test]
    fn test_registry_error_401_shows_auth_hint() {
        let err = registry_error(reqwest::StatusCode::UNAUTHORIZED, "", "publish");
        let msg = format!("{err}");
        assert!(
            msg.contains("horus auth login"),
            "Should suggest auth login: {msg}"
        );
    }

    #[test]
    fn test_registry_error_403_shows_permission_hint() {
        let err = registry_error(reqwest::StatusCode::FORBIDDEN, "", "publish");
        let msg = format!("{err}");
        assert!(
            msg.contains("permission"),
            "Should mention permission: {msg}"
        );
    }

    #[test]
    fn test_registry_error_404_shows_not_found() {
        let err = registry_error(reqwest::StatusCode::NOT_FOUND, "", "search");
        let msg = format!("{err}");
        assert!(msg.contains("not found"), "Should say not found: {msg}");
    }

    #[test]
    fn test_registry_error_409_shows_conflict() {
        let err = registry_error(reqwest::StatusCode::CONFLICT, "", "publish");
        let msg = format!("{err}");
        assert!(
            msg.contains("version") || msg.contains("conflict"),
            "Should mention version conflict: {msg}"
        );
    }

    #[test]
    fn test_registry_error_429_shows_rate_limit() {
        let err = registry_error(reqwest::StatusCode::TOO_MANY_REQUESTS, "", "search");
        let msg = format!("{err}");
        assert!(
            msg.contains("Rate limit") || msg.contains("wait"),
            "Should mention rate limit: {msg}"
        );
    }

    #[test]
    fn test_registry_error_500_shows_server_error() {
        let err = registry_error(reqwest::StatusCode::INTERNAL_SERVER_ERROR, "", "publish");
        let msg = format!("{err}");
        assert!(
            msg.contains("server") || msg.contains("temporary"),
            "Should mention server error: {msg}"
        );
    }

    #[test]
    fn test_registry_error_unknown_status() {
        let err = registry_error(reqwest::StatusCode::IM_A_TEAPOT, "", "brew");
        let msg = format!("{err}");
        assert!(msg.contains("418"), "Should include status code: {msg}");
    }

    #[test]
    fn test_registry_error_includes_body() {
        let err = registry_error(
            reqwest::StatusCode::BAD_REQUEST,
            "invalid name format",
            "publish",
        );
        let msg = format!("{err}");
        assert!(
            msg.contains("invalid name format"),
            "Should include body: {msg}"
        );
    }

    #[test]
    fn test_registry_error_empty_body_no_detail() {
        let err = registry_error(reqwest::StatusCode::BAD_REQUEST, "", "publish");
        let msg = format!("{err}");
        assert!(
            !msg.contains("Server response:"),
            "Empty body should not show detail line: {msg}"
        );
    }

    // ── should_exclude ──────────────────────────────────────────────────

    #[test]
    fn test_should_exclude_git_dir() {
        let base = Path::new("/project");
        assert!(should_exclude(Path::new("/project/.git/config"), base, &[]));
    }

    #[test]
    fn test_should_exclude_target_dir() {
        let base = Path::new("/project");
        assert!(should_exclude(
            Path::new("/project/target/debug/binary"),
            base,
            &[]
        ));
    }

    #[test]
    fn test_should_exclude_horus_dir() {
        let base = Path::new("/project");
        assert!(should_exclude(
            Path::new("/project/.horus/Cargo.toml"),
            base,
            &[]
        ));
    }

    #[test]
    fn test_should_exclude_env_file() {
        let base = Path::new("/project");
        assert!(should_exclude(Path::new("/project/.env"), base, &[]));
    }

    #[test]
    fn test_should_exclude_env_variants() {
        let base = Path::new("/project");
        assert!(should_exclude(Path::new("/project/.env.local"), base, &[]));
        assert!(should_exclude(
            Path::new("/project/.env.production"),
            base,
            &[]
        ));
    }

    #[test]
    fn test_should_exclude_credentials() {
        let base = Path::new("/project");
        assert!(should_exclude(
            Path::new("/project/credentials.json"),
            base,
            &[]
        ));
    }

    #[test]
    fn test_should_exclude_ssh_keys() {
        let base = Path::new("/project");
        assert!(should_exclude(Path::new("/project/id_rsa"), base, &[]));
        assert!(should_exclude(Path::new("/project/id_ed25519"), base, &[]));
    }

    #[test]
    fn test_should_not_exclude_source_code() {
        let base = Path::new("/project");
        assert!(!should_exclude(
            Path::new("/project/src/main.rs"),
            base,
            &[]
        ));
    }

    #[test]
    fn test_should_not_exclude_horus_toml() {
        let base = Path::new("/project");
        assert!(!should_exclude(Path::new("/project/horus.toml"), base, &[]));
    }

    #[test]
    fn test_should_exclude_custom_pattern() {
        let base = Path::new("/project");
        let custom = vec!["experiments".to_string()];
        assert!(should_exclude(
            Path::new("/project/experiments/data.csv"),
            base,
            &custom
        ));
    }

    #[test]
    fn test_should_exclude_glob_pattern() {
        let base = Path::new("/project");
        let custom = vec!["*.log".to_string()];
        assert!(should_exclude(
            Path::new("/project/debug.log"),
            base,
            &custom
        ));
    }

    // ── DEFAULT_EXCLUDES and SECRET_PATTERNS constants ──────────────────

    #[test]
    fn test_default_excludes_count() {
        assert!(
            DEFAULT_EXCLUDES.len() >= 11,
            "Should have at least 11 default excludes"
        );
    }

    #[test]
    fn test_secret_patterns_count() {
        assert!(
            SECRET_PATTERNS.len() >= 10,
            "Should have at least 10 secret patterns"
        );
    }

    #[test]
    fn test_default_excludes_contains_critical_dirs() {
        assert!(DEFAULT_EXCLUDES.contains(&".git"));
        assert!(DEFAULT_EXCLUDES.contains(&"target"));
        assert!(DEFAULT_EXCLUDES.contains(&".horus"));
        assert!(DEFAULT_EXCLUDES.contains(&"node_modules"));
        assert!(DEFAULT_EXCLUDES.contains(&"__pycache__"));
    }

    #[test]
    fn test_secret_patterns_contains_credentials() {
        assert!(SECRET_PATTERNS.contains(&"credentials.json"));
        assert!(SECRET_PATTERNS.contains(&".aws"));
        assert!(SECRET_PATTERNS.contains(&"id_rsa"));
        assert!(SECRET_PATTERNS.contains(&"signing_key"));
    }
}
