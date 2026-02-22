use axum::{
    extract::Query,
    http::StatusCode,
    response::IntoResponse,
    Json,
};

use crate::monitor::{is_safe_path_component, workspace_cache};

#[derive(serde::Deserialize)]
pub struct SearchQuery {
    pub q: String,
}

// Registry: Search available packages from remote registry
pub async fn packages_registry_handler(Query(query): Query<SearchQuery>) -> impl IntoResponse {
    use crate::registry::RegistryClient;

    let result = tokio::task::spawn_blocking(move || {
        let client = RegistryClient::new();
        client.search(&query.q, None, None)
    })
    .await;

    match result {
        Ok(Ok(packages)) => {
            let pkgs = packages
                .into_iter()
                .map(|p| {
                    serde_json::json!({
                        "name": p.name,
                        "version": p.version,
                        "description": p.description.unwrap_or_default(),
                    })
                })
                .collect::<Vec<_>>();

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "packages": pkgs
                })),
            )
                .into_response()
        }
        _ => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": "Failed to search packages"
            })),
        )
            .into_response(),
    }
}

// Environments: Show global packages and local environments
pub async fn packages_environments_handler() -> impl IntoResponse {
    use std::collections::HashSet;
    use std::fs;

    let result = tokio::task::spawn_blocking(move || {
        let mut global_packages_set: HashSet<String> = HashSet::new();
        let mut global_packages = Vec::new();
        let mut local_envs = Vec::new();

        // 1. Global Environment: All packages in ~/.horus/cache (deduplicated)
        if let Some(home) = dirs::home_dir() {
            let global_cache = home.join(".horus/cache");
            if global_cache.exists() {
                if let Ok(entries) = fs::read_dir(&global_cache) {
                    for entry in entries.flatten() {
                        if entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                            let name = entry.file_name().to_string_lossy().to_string();

                            // Skip if already added
                            if global_packages_set.contains(&name) {
                                continue;
                            }

                            // Try to read metadata
                            let metadata_path = entry.path().join("metadata.json");
                            let version = if metadata_path.exists() {
                                fs::read_to_string(&metadata_path)
                                    .ok()
                                    .and_then(|s| {
                                        serde_json::from_str::<serde_json::Value>(&s).ok()
                                    })
                                    .and_then(|j| {
                                        j.get("version")
                                            .and_then(|v| v.as_str())
                                            .map(|s| s.to_string())
                                    })
                                    .unwrap_or_else(|| "unknown".to_string())
                            } else {
                                "unknown".to_string()
                            };

                            global_packages_set.insert(name.clone());
                            global_packages.push(serde_json::json!({
                                "name": name,
                                "version": version,
                            }));
                        }
                    }
                }
            }
        }

        // 2. Local Environments: Use cached workspace discovery (5min TTL)
        let current_workspace = crate::workspace::find_workspace_root();

        // Use cache to avoid repeated scanning (16ms → <1ms after first load)
        let discovered_workspaces = workspace_cache()
            .write()
            .expect("workspace cache lock poisoned")
            .get_or_refresh(&current_workspace);

        for ws in discovered_workspaces {
            let env_path = ws.path;
            let env_name = ws.name;
            let horus_dir = env_path.join(".horus");

            // Try to read dependencies from horus.yaml
            let horus_yaml_path = env_path.join("horus.yaml");
            let yaml_dependencies = if horus_yaml_path.exists() {
                fs::read_to_string(&horus_yaml_path)
                    .ok()
                    .and_then(|content| serde_yaml::from_str::<serde_yaml::Value>(&content).ok())
                    .and_then(|yaml| {
                        yaml.get("dependencies")
                            .and_then(|deps| deps.as_sequence())
                            .map(|seq| {
                                seq.iter()
                                    .filter_map(|v| v.as_str().map(|s| s.to_string()))
                                    .collect::<Vec<String>>()
                            })
                    })
                    .unwrap_or_default()
            } else {
                Vec::new()
            };

            // Get packages inside this environment (deduplicated)
            let packages_dir = horus_dir.join("packages");
            let mut packages = Vec::new();
            let mut local_packages_set: HashSet<String> = HashSet::new();

            if packages_dir.exists() {
                if let Ok(pkg_entries) = fs::read_dir(&packages_dir) {
                    for pkg_entry in pkg_entries.flatten() {
                        let file_type = pkg_entry.file_type().ok();
                        let is_dir = file_type.as_ref().map(|t| t.is_dir()).unwrap_or(false);
                        let is_symlink =
                            file_type.as_ref().map(|t| t.is_symlink()).unwrap_or(false);
                        let is_pkg_entry = is_dir || is_symlink;

                        if is_pkg_entry {
                            let pkg_name = pkg_entry.file_name().to_string_lossy().to_string();
                            let pkg_path = pkg_entry.path();

                            if local_packages_set.contains(&pkg_name) {
                                continue;
                            }

                            let symlink_broken = is_symlink && !pkg_path.exists();

                            let metadata_path = pkg_path.join("metadata.json");
                            let version = if symlink_broken {
                                "broken link".to_string()
                            } else if metadata_path.exists() {
                                fs::read_to_string(&metadata_path)
                                    .ok()
                                    .and_then(|s| {
                                        serde_json::from_str::<serde_json::Value>(&s).ok()
                                    })
                                    .and_then(|j| {
                                        j.get("version")
                                            .and_then(|v| v.as_str())
                                            .map(|s| s.to_string())
                                    })
                                    .unwrap_or_else(|| "unknown".to_string())
                            } else {
                                "unknown".to_string()
                            };

                            local_packages_set.insert(pkg_name.clone());
                            packages.push(serde_json::json!({
                                "name": pkg_name,
                                "version": version,
                            }));
                        }
                    }
                }
            }

            let workspace_dependencies: Vec<serde_json::Value> = yaml_dependencies
                .iter()
                .filter_map(|dep_str| {
                    let dep_name = dep_str.split('@').next().unwrap_or(dep_str);

                    if local_packages_set.contains(dep_name) {
                        return None;
                    }

                    let dep_path = packages_dir.join(dep_str);
                    if dep_path.exists() || dep_path.read_link().is_ok() {
                        let mut metadata_path = dep_path.join("metadata.json");

                        if let Ok(real_path) = std::fs::canonicalize(&dep_path) {
                            metadata_path = real_path.join("metadata.json");
                        }

                        let version = if metadata_path.exists() {
                            fs::read_to_string(&metadata_path)
                                .ok()
                                .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
                                .and_then(|j| {
                                    j.get("version")
                                        .and_then(|v| v.as_str())
                                        .map(|s| s.to_string())
                                })
                                .unwrap_or_else(|| {
                                    dep_str.split('@').nth(1).unwrap_or("unknown").to_string()
                                })
                        } else {
                            dep_str.split('@').nth(1).unwrap_or("unknown").to_string()
                        };

                        Some(serde_json::json!({
                            "name": dep_name,
                            "version": version,
                        }))
                    } else {
                        None
                    }
                })
                .collect();

            local_envs.push(serde_json::json!({
                "name": env_name,
                "path": env_path.to_string_lossy(),
                "packages": packages,
                "package_count": packages.len(),
                "dependencies": workspace_dependencies,
            }));
        }

        serde_json::json!({
            "global": global_packages,
            "local": local_envs
        })
    })
    .await;

    match result {
        Ok(data) => (StatusCode::OK, Json(data)).into_response(),
        Err(_) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": "Failed to list environments"
            })),
        )
            .into_response(),
    }
}

#[derive(serde::Deserialize)]
pub struct InstallRequest {
    pub package: String,
    #[serde(default)]
    pub target: Option<String>,
}

pub async fn packages_install_handler(Json(req): Json<InstallRequest>) -> impl IntoResponse {
    use crate::registry::RegistryClient;
    use std::path::PathBuf;

    // Validate package name to prevent directory traversal
    if !is_safe_path_component(&req.package) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid package name"
            })),
        )
            .into_response();
    }

    let package_name = req.package.clone();
    let target = req.target.clone();

    let result = tokio::task::spawn_blocking(move || -> anyhow::Result<(String, String)> {
        let client = RegistryClient::new();

        let (horus_yaml_path, version) = if let Some(target_str) = &target {
            if target_str == "global" {
                let version = client.install_to_target(
                    &req.package,
                    None,
                    crate::workspace::InstallTarget::Global,
                )?;
                (None, version)
            } else {
                let target_path = PathBuf::from(target_str);

                let parent_path = if target_path.ends_with(".horus/packages") {
                    target_path.parent().and_then(|p| p.parent())
                } else {
                    target_path
                        .parent()
                        .and_then(|p| p.parent())
                        .and_then(|p| p.parent())
                        .and_then(|p| p.parent())
                };

                let yaml_path = parent_path.map(|p| p.join("horus.yaml"));

                let version = client.install_to_target(
                    &req.package,
                    None,
                    crate::workspace::InstallTarget::Local(target_path),
                )?;
                (yaml_path, version)
            }
        } else {
            let yaml_path = PathBuf::from("horus.yaml");
            let yaml_path = if yaml_path.exists() {
                Some(yaml_path)
            } else {
                None
            };
            let version = client.install(&req.package, None)?;
            (yaml_path, version)
        };

        if let Some(yaml_path) = horus_yaml_path {
            if yaml_path.exists() {
                crate::yaml_utils::add_dependency_to_horus_yaml(
                    &yaml_path,
                    &req.package,
                    &version,
                )?;
            }
        }

        Ok((req.package.clone(), version))
    })
    .await;

    match result {
        Ok(Ok(_)) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Successfully installed {}", package_name)
            })),
        )
            .into_response(),
        Ok(Err(e)) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "success": false,
                "error": e.to_string()
            })),
        )
            .into_response(),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "success": false,
                "error": format!("Task failed: {}", e)
            })),
        )
            .into_response(),
    }
}

#[derive(serde::Deserialize)]
pub struct UninstallRequest {
    pub parent_package: String,
    pub package: String,
}

pub async fn packages_uninstall_handler(Json(req): Json<UninstallRequest>) -> impl IntoResponse {
    use std::fs;
    use std::path::PathBuf;

    if !is_safe_path_component(&req.parent_package) || !is_safe_path_component(&req.package) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid package name"
            })),
        )
            .into_response();
    }

    let parent_package = req.parent_package.clone();
    let package = req.package.clone();
    let parent_package_msg = parent_package.clone();
    let package_msg = package.clone();

    let result = tokio::task::spawn_blocking(move || -> anyhow::Result<()> {
        let search_paths = vec![PathBuf::from("."), dirs::home_dir().unwrap_or_default()];

        for base_path in search_paths {
            if !base_path.exists() {
                continue;
            }

            if let Ok(entries) = fs::read_dir(&base_path) {
                for entry in entries.flatten() {
                    if !entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                        continue;
                    }

                    let horus_dir = entry.path().join(".horus");
                    if !horus_dir.exists() {
                        continue;
                    }

                    let parent_pkg_path = horus_dir.join("packages").join(&parent_package);
                    if !parent_pkg_path.exists() {
                        continue;
                    }

                    let nested_pkg_path = parent_pkg_path.join(".horus/packages").join(&package);
                    if nested_pkg_path.exists() {
                        fs::remove_dir_all(&nested_pkg_path)?;

                        let project_root = parent_pkg_path.parent().and_then(|p| p.parent());
                        if let Some(root) = project_root {
                            let horus_yaml_path = root.join("horus.yaml");
                            if horus_yaml_path.exists() {
                                let _ = crate::yaml_utils::remove_dependency_from_horus_yaml(
                                    &horus_yaml_path,
                                    &package,
                                );
                            }
                        }

                        return Ok(());
                    }
                }
            }
        }

        anyhow::bail!("package not found")
    })
    .await;

    match result {
        Ok(Ok(_)) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Successfully uninstalled {} from {}", package_msg, parent_package_msg)
            }))
        ).into_response(),
        Ok(Err(e)) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "success": false,
                "error": e.to_string()
            }))
        ).into_response(),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "success": false,
                "error": format!("Task failed: {}", e)
            }))
        ).into_response(),
    }
}

pub async fn packages_publish_handler() -> impl IntoResponse {
    use crate::registry::RegistryClient;

    let result = tokio::task::spawn_blocking(move || {
        let client = RegistryClient::new();
        client.publish(None, false, None)
    })
    .await;

    match result {
        Ok(Ok(_)) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": "Package published successfully"
            })),
        )
            .into_response(),
        Ok(Err(e)) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "success": false,
                "error": e.to_string()
            })),
        )
            .into_response(),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "success": false,
                "error": format!("Task failed: {}", e)
            })),
        )
            .into_response(),
    }
}
