use super::*;
use super::helpers::*;

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
            return Err(anyhow!("Driver '{}' not found in registry", package_name));
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

    /// Fetch package type from registry (node, driver, plugin, tool, etc.)
    pub fn fetch_package_type(&self, package_name: &str) -> Result<String> {
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}", self.base_url, encoded_name);

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to fetch package info: {}", e))?;

        if !response.status().is_success() {
            return Err(anyhow!("Package '{}' not found in registry", package_name));
        }

        let resp: serde_json::Value = response
            .json()
            .map_err(|e| anyhow!("failed to parse package info: {}", e))?;

        let pkg_type = resp
            .get("package_type")
            .and_then(|v| v.as_str())
            .unwrap_or("node")
            .to_string();

        Ok(pkg_type)
    }

    /// Fetch driver metadata by querying the drivers list API
    pub fn query_driver_features(&self, driver_name: &str) -> Option<Vec<String>> {
        if let Some(meta) = self.fetch_driver_metadata_opt(driver_name) {
            return meta.required_features;
        }

        let url = format!("{}/api/drivers?search={}", self.base_url, driver_name);
        if let Ok(response) = self.client.get(&url).send() {
            if response.status().is_success() {
                if let Ok(list) = response.json::<DriverListResponse>() {
                    for driver in list.drivers {
                        if driver.name.eq_ignore_ascii_case(driver_name) {
                            return driver.driver_metadata.and_then(|m| m.required_features);
                        }
                    }
                }
            }
        }

        None
    }

    /// List all drivers from the registry, optionally filtered by category
    pub fn list_drivers(&self, category: Option<&str>) -> Result<Vec<DriverListEntry>> {
        let url = if let Some(cat) = category {
            format!("{}/api/drivers?category={}", self.base_url, cat)
        } else {
            format!("{}/api/drivers", self.base_url)
        };

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to fetch drivers: {}", e))?;

        if !response.status().is_success() {
            return Err(anyhow!("Registry returned error: {}", response.status()));
        }

        let text = response
            .text()
            .map_err(|e| anyhow!("Failed to read response: {}", e))?;

        if let Ok(list) = serde_json::from_str::<DriverListResponse>(&text) {
            return Ok(list.drivers);
        }

        if let Ok(drivers) = serde_json::from_str::<Vec<DriverListEntry>>(&text) {
            return Ok(drivers);
        }

        Ok(vec![])
    }

    /// Search for drivers by query string and optional bus type
    pub fn search_drivers(
        &self,
        query: &str,
        bus_type: Option<&str>,
    ) -> Result<Vec<DriverListEntry>> {
        let mut url = format!("{}/api/drivers?search={}", self.base_url, query);
        if let Some(bus) = bus_type {
            url.push_str(&format!("&bus_type={}", bus));
        }

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to search drivers: {}", e))?;

        if !response.status().is_success() {
            return Err(anyhow!("Registry returned error: {}", response.status()));
        }

        let text = response
            .text()
            .map_err(|e| anyhow!("Failed to read response: {}", e))?;

        if let Ok(list) = serde_json::from_str::<DriverListResponse>(&text) {
            return Ok(list.drivers);
        }

        if let Ok(drivers) = serde_json::from_str::<Vec<DriverListEntry>>(&text) {
            return Ok(drivers);
        }

        Ok(vec![])
    }

    pub fn publish(&self, path: Option<&Path>, dry_run: bool, org: Option<&str>) -> Result<()> {
        let current_dir = path.unwrap_or_else(|| Path::new("."));

        let manifest = detect_package_info(current_dir)?;
        let name = manifest.name;
        let version = manifest.version;
        let description = manifest.description;
        let license = manifest.license;

        println!(
            " Detected {} manifest",
            format!("{}", manifest.manifest_format).cyan()
        );

        match &manifest.manifest_format {
            ManifestFormat::HorusYaml => {
                let yaml_path = current_dir.join(HORUS_YAML);
                if yaml_path.exists() {
                    use crate::commands::run::parse_horus_yaml_dependencies_v2;
                    use crate::dependency_resolver::DependencySource;

                    match parse_horus_yaml_dependencies_v2(&yaml_path.to_string_lossy()) {
                        Ok(deps) => {
                            let mut has_path_deps = false;
                            for dep in deps {
                                if let DependencySource::Path(p) = dep.source {
                                    println!(
                                        "\n{} Cannot publish package with path dependencies!",
                                        "Error:".red()
                                    );
                                    println!("  Path dependency: {} -> {}", dep.name, p.display());
                                    println!(
                                        "\n{}",
                                        "Path dependencies are not reproducible and cannot be published."
                                            .yellow()
                                    );
                                    println!("{}", "Please publish the path dependency to the registry first, then update horus.yaml.".yellow());
                                    has_path_deps = true;
                                }
                            }
                            if has_path_deps {
                                return Err(anyhow!(
                                    "Cannot publish package with path dependencies"
                                ));
                            }
                        }
                        Err(e) => {
                            log::warn!("Failed to parse horus.yaml dependencies: {}", e);
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

        {
            let tar_file = fs::File::create(&tar_path)?;
            let encoder = GzEncoder::new(tar_file, Compression::default());
            let mut tar_builder = Builder::new(encoder);
            tar_builder.append_dir_all(".", current_dir)?;
            tar_builder.finish()?;
        }

        let package_data = fs::read(&tar_path)?;
        fs::remove_file(&tar_path)?;

        let mut form = reqwest::blocking::multipart::Form::new()
            .text("name", name.clone())
            .text("version", version.clone())
            .text("description", description.unwrap_or_default())
            .text(
                "license",
                license.unwrap_or_else(|| "Apache-2.0".to_string()),
            )
            .part(
                "package",
                reqwest::blocking::multipart::Part::bytes(package_data.clone())
                    .file_name(format!("{}-{}.tar.gz", safe_name, version)),
            );

        if let Some(ref cats) = manifest.categories {
            if !cats.is_empty() {
                form = form.text("categories", cats.clone());
            }
        }
        if let Some(ref pt) = manifest.package_type {
            if !pt.is_empty() {
                form = form.text("package_type", pt.clone());
            }
        }
        if let Some(ref src) = manifest.source_url {
            if !src.is_empty() {
                form = form.text("source_url", src.clone());
            }
        }

        if dry_run {
            form = form.text("dry_run", "true");
            println!(
                " {} Running in dry-run mode (no changes will be made)",
                "ℹ".cyan()
            );
        }

        if let Some(org_name) = org {
            form = form.text("organization", org_name.to_string());
            println!(
                " {} Publishing to organization: {}",
                "ℹ".cyan(),
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
                println!(" {} Package signed with Ed25519 key", "✓".green());
            } else {
                log::warn!("Signing key has invalid length, skipping signature");
                eprintln!(
                    " {} Signing key has invalid length, skipping signature",
                    "⚠".yellow()
                );
            }
        }

        let response = self
            .client
            .post(format!("{}/api/packages/upload", self.base_url))
            .header("Authorization", format!("Bearer {}", api_key))
            .multipart(form)
            .send()?;

        if !response.status().is_success() {
            let status = response.status();
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());

            if status == reqwest::StatusCode::UNAUTHORIZED {
                println!("\n Authentication failed!");
                println!("\nYour API key may be invalid or expired.");
                println!("\nTo fix this:");
                println!("  1. Run: horus auth login");
                println!("  2. Get a new API key from the registry");
                println!("  3. Try publishing again");
                return Err(anyhow!("Unauthorized - invalid or expired API key"));
            }

            return Err(anyhow!("Failed to publish: {} - {}", status, error_text));
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

            println!("\n{} Package verification failed!", "Error:".red());
            println!("   {}", message);

            if error_type == "verification_failed" {
                println!(
                    "\n{}",
                    "The package failed pre-upload verification.".yellow()
                );
                println!("{}", "Please fix the issues above and try again.".yellow());

                if let Some(warnings) = response_json.get("warnings").and_then(|v| v.as_array()) {
                    if !warnings.is_empty() {
                        println!("\n{}", "Warnings:".yellow());
                        for warning in warnings {
                            if let Some(w) = warning.as_str() {
                                println!("   - {}", w);
                            }
                        }
                    }
                }
            }

            return Err(anyhow!("Package verification failed: {}", message));
        }

        if dry_run {
            if response_json.get("dry_run") == Some(&serde_json::json!(true)) {
                println!(
                    " {} Dry run passed! Package {} v{} is valid and ready to publish.",
                    "✓".green(),
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
                    println!("\n{} Build verification in progress...", "ℹ".cyan());
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
                    println!("\n{} Build verification passed!", "✓".green());
                }
                "failed" => {
                    println!("\n{} Build verification failed!", "✗".red());
                    if let Some(msg) = verification.get("message").and_then(|v| v.as_str()) {
                        println!("   {}", msg);
                    }
                }
                _ => {}
            }
        }
        let encoded_name = url_encode_package_name(&name);
        println!("   View at: {}/packages/{}", self.base_url, encoded_name);

        let manifest_source_url = manifest.source_url.unwrap_or_default();
        let manifest_categories = manifest.categories.unwrap_or_default();
        let manifest_package_type = manifest.package_type.unwrap_or_default();

        if !manifest_source_url.is_empty()
            || !manifest_categories.is_empty()
            || !manifest_package_type.is_empty()
        {
            println!(
                "\n{} Auto-detected metadata from {}:",
                "ℹ".green(),
                format!("{}", manifest.manifest_format).cyan()
            );
            if !manifest_source_url.is_empty() {
                println!("   Source URL: {}", manifest_source_url);
            }
            if !manifest_categories.is_empty() {
                println!("   Categories: {}", manifest_categories);
            }
            if !manifest_package_type.is_empty() {
                println!("   Package type: {}", manifest_package_type);
            }
        }

        println!("\n{}", "[#] Package Metadata (optional)".cyan().bold());
        println!("   Help users discover and use your package by adding:");

        let (docs_url, docs_type, prompted_source_url, prompted_categories, prompted_package_type) =
            prompt_package_metadata(current_dir)?;

        let final_source_url = if !prompted_source_url.is_empty() {
            prompted_source_url
        } else {
            manifest_source_url
        };
        let final_categories = if !prompted_categories.is_empty() {
            prompted_categories
        } else {
            manifest_categories
        };
        let final_package_type = if !prompted_package_type.is_empty() {
            prompted_package_type
        } else {
            manifest_package_type
        };

        if !docs_url.is_empty()
            || !final_source_url.is_empty()
            || !final_categories.is_empty()
            || !final_package_type.is_empty()
        {
            println!("\n{} Updating package metadata...", "ℹ".cyan());
            self.update_package_metadata(
                &name,
                &version,
                &docs_url,
                &docs_type,
                &final_source_url,
                &final_categories,
                &final_package_type,
                &api_key,
            )?;
            println!(" Package metadata updated!");
        }

        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn update_package_metadata(
        &self,
        name: &str,
        version: &str,
        docs_url: &str,
        docs_type: &str,
        source_url: &str,
        categories: &str,
        package_type: &str,
        api_key: &str,
    ) -> Result<()> {
        let categories_vec: Vec<String> = if categories.is_empty() {
            vec![]
        } else {
            categories
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect()
        };

        let body = serde_json::json!({
            "docs_url": if docs_url.is_empty() { None } else { Some(docs_url) },
            "docs_type": if docs_type.is_empty() { None } else { Some(docs_type) },
            "source_url": if source_url.is_empty() { None } else { Some(source_url) },
            "categories": if categories_vec.is_empty() { None } else { Some(&categories_vec) },
            "package_type": if package_type.is_empty() { None } else { Some(package_type) },
        });

        let response = self
            .client
            .post(format!(
                "{}/api/packages/{}/{}/metadata",
                self.base_url, name, version
            ))
            .header("Authorization", format!("Bearer {}", api_key))
            .json(&body)
            .send()?;

        if !response.status().is_success() {
            return Err(anyhow!("Failed to update package metadata"));
        }

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

        let url = format!(
            "{}/api/packages/{}/{}",
            self.base_url, package_name, version
        );
        let response = self
            .client
            .delete(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let status = response.status();
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());

            if status == reqwest::StatusCode::UNAUTHORIZED {
                return Err(anyhow!(
                    "Authentication failed - invalid or expired API key"
                ));
            } else if status == reqwest::StatusCode::FORBIDDEN {
                return Err(anyhow!("You do not have permission to unpublish this package. Only the package owner can unpublish it."));
            } else if status == reqwest::StatusCode::NOT_FOUND {
                return Err(anyhow!(
                    "Package {} v{} not found in registry",
                    package_name,
                    version
                ));
            }

            return Err(anyhow!("Failed to unpublish: {} - {}", status, error_text));
        }

        Ok(())
    }

    pub fn search(
        &self,
        query: &str,
        package_type: Option<&str>,
        category: Option<&str>,
    ) -> Result<Vec<Package>> {
        let mut url = format!("{}/api/packages/search?q={}", self.base_url, query);
        if let Some(pt) = package_type {
            url.push_str(&format!("&type={}", pt));
        }
        if let Some(cat) = category {
            url.push_str(&format!("&category={}", cat));
        }

        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            return Err(anyhow!("Search failed"));
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

    pub fn resolve_import(&self, import_name: &str, language: &str) -> Result<Option<String>> {
        let url = format!(
            "{}/api/imports/resolve?import={}&language={}",
            self.base_url, import_name, language
        );

        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            return Ok(None);
        }

        #[derive(Deserialize)]
        struct ResolveResult {
            package_name: String,
        }

        let result: Option<ResolveResult> = response.json()?;
        Ok(result.map(|r| r.package_name))
    }

    pub fn freeze(&self) -> Result<EnvironmentManifest> {
        let packages_dir = PathBuf::from(".horus/packages");
        let mut locked_packages = Vec::new();

        if packages_dir.exists() {
            for entry in fs::read_dir(&packages_dir)? {
                let entry = entry?;
                let entry_path = entry.path();

                if entry_path.extension().and_then(|s| s.to_str()) == Some("json")
                    && entry_path.to_string_lossy().contains(".path.")
                {
                    let content = fs::read_to_string(&entry_path)?;
                    let metadata: serde_json::Value = serde_json::from_str(&content)?;

                    let name = metadata["name"].as_str().unwrap_or("unknown").to_string();
                    let version = metadata["version"].as_str().unwrap_or("dev").to_string();
                    let path = metadata["source_path"].as_str().unwrap_or("").to_string();

                    locked_packages.push(LockedPackage {
                        name,
                        version,
                        checksum: String::new(),
                        source: PackageSource::Path { path },
                    });
                    continue;
                }

                if entry_path.extension().and_then(|s| s.to_str()) == Some("json")
                    && entry_path.to_string_lossy().contains(".system.")
                {
                    let content = fs::read_to_string(&entry_path)?;
                    let metadata: serde_json::Value = serde_json::from_str(&content)?;

                    let name = metadata["name"].as_str().unwrap_or("unknown").to_string();
                    let version = metadata["version"]
                        .as_str()
                        .unwrap_or("unknown")
                        .to_string();

                    locked_packages.push(LockedPackage {
                        name,
                        version,
                        checksum: String::new(),
                        source: PackageSource::System,
                    });
                    continue;
                }

                let is_package = entry.file_type()?.is_dir() || entry.file_type()?.is_symlink();

                if is_package {
                    let package_name = entry.file_name().to_string_lossy().to_string();

                    let actual_path = if entry_path.is_symlink() {
                        entry_path.read_link().unwrap_or(entry_path.clone())
                    } else {
                        entry_path.clone()
                    };

                    let metadata_path = actual_path.join("metadata.json");
                    if metadata_path.exists() {
                        let content = fs::read_to_string(&metadata_path)?;
                        let metadata_value: serde_json::Value = serde_json::from_str(&content)?;

                        let name = metadata_value["name"]
                            .as_str()
                            .unwrap_or(&package_name)
                            .to_string();
                        let version = metadata_value["version"]
                            .as_str()
                            .unwrap_or("unknown")
                            .to_string();
                        let checksum = metadata_value["checksum"]
                            .as_str()
                            .unwrap_or("")
                            .to_string();
                        let source_str = metadata_value["source"].as_str().unwrap_or("Registry");

                        let source = if source_str == "PyPI"
                            || actual_path.to_string_lossy().contains("pypi_")
                        {
                            PackageSource::PyPI
                        } else {
                            PackageSource::Registry
                        };

                        locked_packages.push(LockedPackage {
                            name,
                            version,
                            checksum,
                            source,
                        });
                    } else {
                        let source = if actual_path.to_string_lossy().contains("pypi_") {
                            PackageSource::PyPI
                        } else {
                            PackageSource::Registry
                        };

                        let version = detect_package_version(&actual_path)
                            .unwrap_or_else(|| "unknown".to_string());

                        locked_packages.push(LockedPackage {
                            name: package_name.clone(),
                            version,
                            checksum: String::new(),
                            source,
                        });
                    }
                }
            }
        }

        let system_info = SystemInfo {
            os: std::env::consts::OS.to_string(),
            arch: std::env::consts::ARCH.to_string(),
            python_version: get_python_version(),
            rust_version: get_rust_version(),
            gcc_version: get_gcc_version(),
            cuda_version: get_cuda_version(),
        };

        let mut hasher = Sha256::new();
        for pkg in &locked_packages {
            hasher.update(&pkg.name);
            hasher.update(&pkg.version);
            hasher.update(&pkg.checksum);
        }
        hasher.update(&system_info.os);
        hasher.update(&system_info.arch);
        let horus_id = format!("env-{}", &format!("{:x}", hasher.finalize())[..12]);

        let manifest = EnvironmentManifest {
            horus_id,
            name: None,
            description: Some("Frozen environment manifest".to_string()),
            packages: locked_packages,
            system: system_info,
            created_at: chrono::Utc::now(),
            horus_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        Ok(manifest)
    }

    pub fn save_environment(&self, manifest: &EnvironmentManifest) -> Result<()> {
        let response = self
            .client
            .post(format!("{}/api/environments", self.base_url))
            .json(manifest)
            .send()?;

        if !response.status().is_success() {
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(anyhow!("Failed to save environment: {}", error_text));
        }

        println!(" Environment saved with ID: {}", manifest.horus_id);
        Ok(())
    }

    pub fn restore_environment(&self, horus_id: &str) -> Result<()> {
        println!(" Restoring environment {}...", horus_id);

        let url = format!("{}/api/environments/{}", self.base_url, horus_id);
        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            return Err(anyhow!("Environment not found: {}", horus_id));
        }

        let manifest: EnvironmentManifest = response.json()?;

        for package in &manifest.packages {
            println!("  Installing {} v{}...", package.name, package.version);
            self.install(&package.name, Some(&package.version))?;
        }

        println!(" Environment {} restored successfully!", horus_id);
        Ok(())
    }

    pub fn upload_environment(&self, manifest: &EnvironmentManifest) -> Result<()> {
        println!(
            "Publishing environment {} to registry...",
            manifest.horus_id
        );

        let api_key = get_api_key()?;

        let url = format!("{}/api/environments", self.base_url);
        let response = self
            .client
            .post(&url)
            .header("x-api-key", api_key)
            .json(&serde_json::json!({
                "horus_id": manifest.horus_id,
                "name": manifest.name,
                "description": manifest.description,
                "manifest": manifest
            }))
            .send()?;

        if !response.status().is_success() {
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(anyhow!("Failed to publish environment: {}", error_text));
        }

        println!(" Environment published successfully!");
        println!(
            "   Anyone can now restore with: horus env restore {}",
            manifest.horus_id
        );
        Ok(())
    }

    pub fn list_environments(&self) -> Result<Vec<EnvironmentListItem>> {
        let api_key = get_api_key()?;
        let url = format!("{}/api/environments", self.base_url);
        let response = self
            .client
            .get(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let status = response.status();
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(anyhow!(
                "Failed to list environments ({}): {}",
                status,
                error_text
            ));
        }

        let list_response: EnvironmentListResponse = response.json()?;
        Ok(list_response.environments)
    }

    pub fn get_environment(&self, horus_id: &str) -> Result<EnvironmentManifest> {
        let url = format!("{}/api/environments/{}", self.base_url, horus_id);
        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            let status = response.status();
            if status == reqwest::StatusCode::NOT_FOUND {
                return Err(anyhow!("Environment not found: {}", horus_id));
            }
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(anyhow!(
                "Failed to get environment ({}): {}",
                status,
                error_text
            ));
        }

        let manifest: EnvironmentManifest = response.json()?;
        Ok(manifest)
    }
}
