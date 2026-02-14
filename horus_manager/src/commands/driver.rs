//! Driver command - list, search, info, probe, install, remove, and manage hardware drivers

use crate::{registry, workspace, yaml_utils};
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::path::PathBuf;

struct DriverInfo {
    id: &'static str,
    name: &'static str,
    category: &'static str,
    description: &'static str,
}

pub(crate) fn resolve_driver_alias(alias: &str) -> Option<Vec<&'static str>> {
    match alias {
        "vision" => Some(vec!["camera", "depth-camera"]),
        "navigation" => Some(vec!["lidar", "gps", "imu"]),
        "manipulation" => Some(vec!["servo", "motor", "force-torque"]),
        "locomotion" => Some(vec!["motor", "encoder", "imu"]),
        "sensing" => Some(vec!["camera", "lidar", "ultrasonic", "imu"]),
        _ => None,
    }
}

fn built_in_drivers() -> Vec<DriverInfo> {
    vec![
        DriverInfo {
            id: "camera",
            name: "Camera Driver",
            category: "Sensor",
            description: "RGB camera support (OpenCV, V4L2)",
        },
        DriverInfo {
            id: "depth-camera",
            name: "Depth Camera Driver",
            category: "Sensor",
            description: "Depth sensing (RealSense, Kinect)",
        },
        DriverInfo {
            id: "lidar",
            name: "LiDAR Driver",
            category: "Sensor",
            description: "2D/3D LiDAR (RPLidar, Velodyne)",
        },
        DriverInfo {
            id: "imu",
            name: "IMU Driver",
            category: "Sensor",
            description: "Inertial measurement (MPU6050, BNO055)",
        },
        DriverInfo {
            id: "gps",
            name: "GPS Driver",
            category: "Sensor",
            description: "GPS/GNSS receivers",
        },
        DriverInfo {
            id: "ultrasonic",
            name: "Ultrasonic Driver",
            category: "Sensor",
            description: "Ultrasonic rangefinders",
        },
        DriverInfo {
            id: "motor",
            name: "Motor Driver",
            category: "Actuator",
            description: "DC/BLDC motors",
        },
        DriverInfo {
            id: "servo",
            name: "Servo Driver",
            category: "Actuator",
            description: "Servo motors (PWM, Dynamixel)",
        },
        DriverInfo {
            id: "stepper",
            name: "Stepper Driver",
            category: "Actuator",
            description: "Stepper motors",
        },
        DriverInfo {
            id: "encoder",
            name: "Encoder Driver",
            category: "Sensor",
            description: "Rotary/linear encoders",
        },
        DriverInfo {
            id: "force-torque",
            name: "Force/Torque Driver",
            category: "Sensor",
            description: "Force/torque sensors",
        },
        DriverInfo {
            id: "serial",
            name: "Serial Driver",
            category: "Bus",
            description: "UART/RS232/RS485",
        },
        DriverInfo {
            id: "i2c",
            name: "I2C Driver",
            category: "Bus",
            description: "I2C bus communication",
        },
        DriverInfo {
            id: "spi",
            name: "SPI Driver",
            category: "Bus",
            description: "SPI bus communication",
        },
        DriverInfo {
            id: "can",
            name: "CAN Driver",
            category: "Bus",
            description: "CAN bus communication",
        },
        DriverInfo {
            id: "modbus",
            name: "Modbus Driver",
            category: "Bus",
            description: "Modbus RTU/TCP",
        },
        DriverInfo {
            id: "joystick",
            name: "Joystick Driver",
            category: "Input",
            description: "Gamepad/joystick input",
        },
        DriverInfo {
            id: "keyboard",
            name: "Keyboard Driver",
            category: "Input",
            description: "Keyboard input",
        },
    ]
}

/// List available drivers (built-in, plugins, and registry)
pub fn run_list(
    category: Option<String>,
    registry_only: bool,
    plugins_only: bool,
    mode: String,
) -> HorusResult<()> {
    let built_in = built_in_drivers();

    // Show local built-in drivers unless --registry or --plugins flag
    if !registry_only && !plugins_only {
        let drivers: Vec<_> = if let Some(cat_str) = &category {
            let cat_lower = cat_str.to_lowercase();
            let cat_match = match cat_lower.as_str() {
                "sensor" | "sensors" => "Sensor",
                "actuator" | "actuators" => "Actuator",
                "bus" | "buses" => "Bus",
                "input" | "inputs" => "Input",
                "simulation" | "sim" => "Simulation",
                _ => {
                    println!("{} Unknown category: {}", "[WARN]".yellow(), cat_str);
                    println!("       Valid categories: sensor, actuator, bus, input, simulation");
                    return Ok(());
                }
            };
            built_in
                .iter()
                .filter(|d| d.category == cat_match)
                .collect()
        } else {
            built_in.iter().collect()
        };

        if !drivers.is_empty() {
            println!("{} Built-in Drivers:\n", "[BUILD]".cyan().bold());
            for driver in &drivers {
                println!(
                    "  {} {} ({})",
                    "-".green(),
                    driver.id.yellow(),
                    driver.category
                );
                if !driver.description.is_empty() {
                    println!("     {}", driver.description.dimmed());
                }
            }
            println!();
        }
    }

    // Show loaded plugins
    if !registry_only {
        use horus_core::plugin::{DriverLoader, DriverLoaderConfig, DriverMode};

        let driver_mode = match mode.to_lowercase().as_str() {
            "static" => DriverMode::Static,
            "dynamic" => DriverMode::Dynamic,
            _ => DriverMode::Hybrid,
        };

        let loader_config = DriverLoaderConfig {
            mode: driver_mode,
            auto_discover: true,
            ..Default::default()
        };
        let mut loader = DriverLoader::new(loader_config);

        // Try to load plugins from standard locations
        if let Err(e) = loader.initialize() {
            if !plugins_only {
                // Only warn if not plugins-only mode
                println!("{} Could not load plugins: {}", "[WARN]".yellow(), e);
            }
        }

        let loaded = loader.available_drivers();
        if !loaded.is_empty() {
            println!(
                "{} Loaded Plugins ({}):\n",
                "[PLUGIN]".magenta().bold(),
                loaded.len()
            );

            for plugin_id in loaded {
                if let Some(manifest) = loader.get_plugin(&plugin_id).ok().map(|p| p.manifest()) {
                    // Filter by category if specified
                    let category_match = if let Some(cat_str) = &category {
                        let cat_lower = cat_str.to_lowercase();
                        let manifest_cat = format!("{:?}", manifest.category).to_lowercase();
                        manifest_cat.contains(&cat_lower)
                    } else {
                        true
                    };

                    if category_match {
                        println!(
                            "  {} {} v{} ({:?})",
                            "◆".magenta(),
                            manifest.name.yellow(),
                            manifest.version,
                            manifest.category
                        );
                        println!("     {}", manifest.description.dimmed());

                        // Show backends provided by this plugin
                        if !manifest.backends.is_empty() {
                            let backend_ids: Vec<_> =
                                manifest.backends.iter().map(|b| b.id.as_str()).collect();
                            println!("     Backends: {}", backend_ids.join(", ").cyan());
                        }
                    }
                }
            }
            println!();
        } else if plugins_only {
            println!("{} No plugins loaded.", "[INFO]".cyan());
            println!("  Place plugins in ~/.horus/drivers/ or use static linking.");
        }
    }

    // Also fetch from registry (unless plugins_only)
    if !plugins_only {
        println!("{} Fetching drivers from registry...", "[NET]".cyan());
        let client = registry::RegistryClient::new();
        match client.list_drivers(category.as_deref()) {
            Ok(registry_drivers) => {
                if registry_drivers.is_empty() {
                    println!("  No additional drivers in registry.");
                } else {
                    println!(
                        "\n{} Registry Drivers ({} available):\n",
                        "[PKG]".cyan().bold(),
                        registry_drivers.len()
                    );
                    for d in registry_drivers {
                        let bus = d.bus_type.as_deref().unwrap_or("unknown");
                        let cat = d.category.as_deref().unwrap_or("driver");
                        println!(
                            "  {} {} ({}, {})",
                            "-".green(),
                            d.name.yellow(),
                            cat,
                            bus.dimmed()
                        );
                        if let Some(desc) = &d.description {
                            println!("     {}", desc.dimmed());
                        }
                    }
                }
            }
            Err(e) => {
                println!("  {} Could not fetch registry: {}", "[WARN]".yellow(), e);
            }
        }
    }
    Ok(())
}

/// Show information about a specific driver
pub fn run_info(driver: String) -> HorusResult<()> {
    let built_in = built_in_drivers();

    // Check if it's an alias first
    if let Some(expanded) = resolve_driver_alias(&driver) {
        println!(
            "{} Driver Alias: {}\n",
            "[LINK]".cyan().bold(),
            driver.yellow()
        );
        println!("  Expands to: {}", expanded.join(", ").green());
        return Ok(());
    }

    // Look up in local built-ins first
    if let Some(info) = built_in.iter().find(|d| d.id == driver) {
        println!(
            "{} Driver: {} (built-in)\n",
            "[BUILD]".cyan().bold(),
            info.name.yellow()
        );
        println!("  ID:       {}", info.id);
        println!("  Category: {}", info.category);
        if !info.description.is_empty() {
            println!("  Desc:     {}", info.description);
        }
        return Ok(());
    }

    // Try registry
    println!("{} Looking up '{}' in registry...", "[NET]".cyan(), driver);
    let client = registry::RegistryClient::new();
    match client.fetch_driver_metadata(&driver) {
        Ok(meta) => {
            println!("\n{} Driver: {}\n", "[PKG]".cyan().bold(), driver.yellow());
            if let Some(bus) = &meta.bus_type {
                println!("  Bus Type:  {}", bus);
            }
            if let Some(cat) = &meta.driver_category {
                println!("  Category:  {}", cat);
            }

            // Show dependencies
            if let Some(features) = &meta.required_features {
                if !features.is_empty() {
                    println!("\n  {} Cargo Features:", "[BUILD]".cyan());
                    for f in features {
                        println!("    • {}", f.yellow());
                    }
                }
            }
            if let Some(cargo_deps) = &meta.cargo_dependencies {
                if !cargo_deps.is_empty() {
                    println!("\n  {} Cargo Dependencies:", "[PKG]".cyan());
                    for d in cargo_deps {
                        println!("    • {}", d);
                    }
                }
            }
            if let Some(py_deps) = &meta.python_dependencies {
                if !py_deps.is_empty() {
                    println!("\n  {} Python Dependencies:", "[PY]".cyan());
                    for d in py_deps {
                        println!("    • {}", d);
                    }
                }
            }
            if let Some(sys_deps) = &meta.system_dependencies {
                if !sys_deps.is_empty() {
                    println!("\n  {} System Packages:", "[SYS]".cyan());
                    for d in sys_deps {
                        println!("    • {}", d);
                    }
                }
            }

            println!(
                "\n  {} To add: horus driver add {}",
                "[TIP]".green(),
                driver
            );
        }
        Err(_) => {
            println!("{} Driver '{}' not found.", "[WARN]".yellow(), driver);
            println!("\nUse 'horus driver list' to see available drivers.");
            println!("Use 'horus driver search <query>' to search.");
        }
    }
    Ok(())
}

/// Search for drivers by query
pub fn run_search(query: String, bus_type: Option<String>) -> HorusResult<()> {
    let built_in = built_in_drivers();

    // Search local first
    let query_lower = query.to_lowercase();
    let local_matches: Vec<_> = built_in
        .iter()
        .filter(|d| {
            d.id.to_lowercase().contains(&query_lower)
                || d.name.to_lowercase().contains(&query_lower)
                || d.description.to_lowercase().contains(&query_lower)
        })
        .collect();

    if !local_matches.is_empty() {
        println!("{} Built-in matches:\n", "[BUILD]".cyan().bold());
        for driver in &local_matches {
            println!("  {} {} - {}", "-".green(), driver.id.yellow(), driver.name);
        }
        println!();
    }

    // Search registry
    println!("{} Searching registry for '{}'...", "[NET]".cyan(), query);
    let client = registry::RegistryClient::new();
    match client.search_drivers(&query, bus_type.as_deref()) {
        Ok(results) => {
            if results.is_empty() {
                println!("  No matches in registry.");
            } else {
                println!(
                    "\n{} Registry matches ({}):\n",
                    "[PKG]".cyan().bold(),
                    results.len()
                );
                for d in results {
                    let bus = d.bus_type.as_deref().unwrap_or("?");
                    println!("  {} {} [{}]", "-".green(), d.name.yellow(), bus.dimmed());
                    if let Some(desc) = &d.description {
                        println!("     {}", desc.dimmed());
                    }
                }
            }
        }
        Err(e) => {
            println!("  {} Could not search registry: {}", "[WARN]".yellow(), e);
        }
    }
    Ok(())
}

/// Probe for hardware using driver plugins
pub fn run_probe(plugin: Option<String>, backend: Option<String>, mode: String) -> HorusResult<()> {
    use horus_core::plugin::{DriverLoader, DriverLoaderConfig, DriverMode};

    let driver_mode = match mode.to_lowercase().as_str() {
        "static" => DriverMode::Static,
        "dynamic" => DriverMode::Dynamic,
        _ => DriverMode::Hybrid,
    };

    println!(
        "{} Probing for hardware (mode: {})...\n",
        "[PROBE]".cyan().bold(),
        mode
    );

    let loader_config = DriverLoaderConfig {
        mode: driver_mode,
        auto_discover: true,
        ..Default::default()
    };
    let mut loader = DriverLoader::new(loader_config);

    if let Err(e) = loader.initialize() {
        return Err(HorusError::Driver(format!(
            "Failed to initialize plugin loader: {}",
            e
        )));
    }

    let plugins_to_probe: Vec<String> = if let Some(p) = plugin {
        vec![p]
    } else {
        loader.available_drivers()
    };

    if plugins_to_probe.is_empty() {
        println!("{} No plugins loaded to probe.", "[WARN]".yellow());
        println!("  Place plugins in ~/.horus/drivers/ or use static linking.");
        return Ok(());
    }

    let backend_filter = backend.as_deref();

    for plugin_id in plugins_to_probe {
        let results = match loader.probe(&plugin_id) {
            Ok(r) => r,
            Err(e) => {
                println!("  {} Plugin {}: {}", "[ERROR]".red(), plugin_id.yellow(), e);
                continue;
            }
        };

        // Filter by backend if specified
        let results: Vec<_> = if let Some(filter) = backend_filter {
            results
                .into_iter()
                .filter(|r| r.backend_id.contains(filter))
                .collect()
        } else {
            results
        };

        if results.is_empty() {
            continue;
        }

        println!("  {} Plugin: {}", "◆".magenta(), plugin_id.yellow());

        for result in results {
            let status = if result.detected {
                format!("✓ Detected (confidence: {:.0}%)", result.confidence * 100.0).green()
            } else {
                "✗ Not detected".red().to_string().into()
            };

            println!(
                "    {} {} - {}",
                "-".cyan(),
                result.backend_id.yellow(),
                status
            );

            if let Some(device_path) = &result.device_path {
                println!("      Device: {}", device_path.dimmed());
            }

            if let Some(msg) = &result.message {
                println!("      Info: {}", msg.dimmed());
            }
        }
        println!();
    }

    Ok(())
}

/// List and manage loaded driver plugins
pub fn run_plugins(reload: bool, mode: String) -> HorusResult<()> {
    use horus_core::plugin::{DriverLoader, DriverLoaderConfig, DriverMode};

    let driver_mode = match mode.to_lowercase().as_str() {
        "static" => DriverMode::Static,
        "dynamic" => DriverMode::Dynamic,
        _ => DriverMode::Hybrid,
    };

    if reload {
        println!(
            "{} Reloading plugins (mode: {})...",
            "[PLUGIN]".magenta().bold(),
            mode
        );
    } else {
        println!(
            "{} Loaded Plugins (mode: {}):\n",
            "[PLUGIN]".magenta().bold(),
            mode
        );
    }

    let loader_config = DriverLoaderConfig {
        mode: driver_mode,
        auto_discover: true,
        ..Default::default()
    };
    let mut loader = DriverLoader::new(loader_config);

    if let Err(e) = loader.initialize() {
        return Err(HorusError::Driver(format!(
            "Failed to initialize plugin loader: {}",
            e
        )));
    }

    let loaded = loader.available_drivers();

    if loaded.is_empty() {
        println!("{} No plugins loaded.", "[INFO]".cyan());
        println!("\n  Plugin search paths:");
        println!("    - ~/.horus/drivers/");
        println!("    - ./drivers/");
        println!("\n  To create a plugin:");
        println!("    1. Build plugin as cdylib: crate-type = [\"cdylib\"]");
        println!("    2. Export entry point: horus_driver_entry()");
        println!("    3. Place .so file in search path");
        return Ok(());
    }

    // Get plugin health status once
    let health_map = loader.health();

    for plugin_id in loaded {
        if let Some(manifest) = loader.get_plugin(&plugin_id).ok().map(|p| p.manifest()) {
            println!(
                "  {} {} v{} ({:?})",
                "◆".magenta(),
                manifest.name.yellow(),
                manifest.version,
                manifest.category
            );
            println!("     ID:          {}", manifest.id);
            println!("     Description: {}", manifest.description.dimmed());

            if let Some(author) = &manifest.author {
                println!("     Author:      {}", author);
            }

            if !manifest.backends.is_empty() {
                println!("     Backends:");
                for backend in &manifest.backends {
                    println!(
                        "       - {} ({})",
                        backend.id.cyan(),
                        backend.description.dimmed()
                    );
                }
            }

            // Check plugin health
            if let Some(health) = health_map.get(&plugin_id) {
                let status = if health.healthy {
                    "Healthy".green()
                } else {
                    "Unhealthy".red()
                };
                println!("     Status:      {}", status);
            }
            println!();
        }
    }

    Ok(())
}

/// Resolve the package directory after installation for plugin registration
fn resolve_package_dir(name: &str, version: &str, global: bool) -> Option<PathBuf> {
    if global {
        let home = dirs::home_dir()?;
        let versioned = home
            .join(".horus/cache")
            .join(format!("{}@{}", name, version));
        if versioned.exists() {
            return Some(versioned);
        }
        let plain = home.join(".horus/cache").join(name);
        if plain.exists() {
            return Some(plain);
        }
    } else {
        let workspace_root =
            workspace::find_workspace_root().unwrap_or_else(|| PathBuf::from("."));
        let local = workspace_root.join(".horus/packages").join(name);
        if local.exists() {
            return Some(local);
        }
    }
    None
}

/// Install a driver package from registry
pub fn run_install(driver: String, ver: Option<String>, global: bool) -> HorusResult<()> {
    println!(
        "{} Installing driver: {}{}",
        "[DRIVER]".cyan().bold(),
        driver.yellow(),
        ver.as_ref()
            .map(|v| format!("@{}", v))
            .unwrap_or_default()
    );

    // Fetch and display driver metadata if available
    let client = registry::RegistryClient::new();
    if let Ok(meta) = client.fetch_driver_metadata(&driver) {
        if let Some(cat) = &meta.driver_category {
            println!("  Category:  {}", cat);
        }
        if let Some(bus) = &meta.bus_type {
            println!("  Bus type:  {}", bus);
        }
        if let Some(sys_deps) = &meta.system_dependencies {
            if !sys_deps.is_empty() {
                println!(
                    "  {} System deps: {}",
                    "[SYS]".dimmed(),
                    sys_deps.join(", ")
                );
            }
        }
        println!();
    }

    // Determine install target
    let install_target = if global {
        println!("  Scope: {}", "global (~/.horus/cache/)".dimmed());
        workspace::InstallTarget::Global
    } else {
        let target = workspace::detect_or_select_workspace(true)
            .map_err(|e| HorusError::Config(e.to_string()))?;
        match &target {
            workspace::InstallTarget::Local(p) => {
                println!("  Scope: {}", format!("local ({})", p.display()).dimmed());
            }
            workspace::InstallTarget::Global => {
                println!("  Scope: {}", "global (~/.horus/cache/)".dimmed());
            }
        }
        target
    };

    // Install via the existing package install flow
    // This internally calls apply_driver_requirements() for driver packages
    let installed_version = client
        .install_to_target(&driver, ver.as_deref(), install_target.clone())
        .map_err(|e| HorusError::Config(e.to_string()))?;

    // Try to register as CLI plugin if the driver package also provides one
    let is_global = matches!(install_target, workspace::InstallTarget::Global);
    let project_root = match &install_target {
        workspace::InstallTarget::Local(p) => Some(p.as_path()),
        workspace::InstallTarget::Global => None,
    };

    if let Some(pkg_dir) = resolve_package_dir(&driver, &installed_version, is_global) {
        let source = crate::plugins::PluginSource::Registry;
        if let Ok(Some(cmd)) =
            super::pkg::register_plugin_after_install(&pkg_dir, source, is_global, project_root)
        {
            println!(
                "  {} Also registered CLI plugin: horus {}",
                "🔌".cyan(),
                cmd.green()
            );
        }
    }

    // Update horus.yaml with driver-prefixed dependency
    if let workspace::InstallTarget::Local(workspace_path) = &install_target {
        let horus_yaml_path = workspace_path.join("horus.yaml");
        if horus_yaml_path.exists() {
            let dep_string = format!("driver:{}", driver);
            let version = ver.as_deref().unwrap_or(&installed_version);
            if let Err(e) =
                yaml_utils::add_dependency_to_horus_yaml(&horus_yaml_path, &dep_string, version)
            {
                println!("  {} Failed to update horus.yaml: {}", "⚠".yellow(), e);
            } else {
                println!("  {} Updated horus.yaml", "✓".green());
            }
        }
    }

    println!();
    println!(
        "{} Driver {} v{} installed successfully!",
        "✓".green().bold(),
        driver.green(),
        installed_version
    );
    println!();
    println!("  {} Usage hints:", "[TIP]".green());
    println!(
        "    • Configure in drivers.yaml: {}",
        format!("driver: {}", driver).dimmed()
    );
    println!(
        "    • Import in code:           {}",
        format!("use horus::drivers::{};", driver.replace('-', "_")).dimmed()
    );

    Ok(())
}

/// Remove an installed driver package
pub fn run_remove(driver: String, global: bool) -> HorusResult<()> {
    println!(
        "{} Removing driver: {}",
        "[DRIVER]".cyan().bold(),
        driver.yellow()
    );

    // Delegate to pkg::run_remove
    super::pkg::run_remove(driver, global, None)
}
