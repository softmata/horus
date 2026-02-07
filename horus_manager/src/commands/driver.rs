//! Driver command - list, search, info, probe, and manage hardware drivers

use colored::*;
use horus_core::error::{HorusError, HorusResult};
use crate::registry;

struct DriverInfo {
    id: &'static str,
    name: &'static str,
    category: &'static str,
    description: &'static str,
}

fn resolve_driver_alias_local(alias: &str) -> Option<Vec<&'static str>> {
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
                    println!(
                        "       Valid categories: sensor, actuator, bus, input, simulation"
                    );
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
                if let Some(manifest) =
                    loader.get_plugin(&plugin_id).ok().map(|p| p.manifest())
                {
                    // Filter by category if specified
                    let category_match = if let Some(cat_str) = &category {
                        let cat_lower = cat_str.to_lowercase();
                        let manifest_cat =
                            format!("{:?}", manifest.category).to_lowercase();
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
                            let backend_ids: Vec<_> = manifest
                                .backends
                                .iter()
                                .map(|b| b.id.as_str())
                                .collect();
                            println!(
                                "     Backends: {}",
                                backend_ids.join(", ").cyan()
                            );
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
    if let Some(expanded) = resolve_driver_alias_local(&driver) {
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
                    println!(
                        "  {} {} [{}]",
                        "-".green(),
                        d.name.yellow(),
                        bus.dimmed()
                    );
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
pub fn run_probe(
    plugin: Option<String>,
    backend: Option<String>,
    mode: String,
) -> HorusResult<()> {
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
                println!(
                    "  {} Plugin {}: {}",
                    "[ERROR]".red(),
                    plugin_id.yellow(),
                    e
                );
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
                format!(
                    "✓ Detected (confidence: {:.0}%)",
                    result.confidence * 100.0
                )
                .green()
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
        if let Some(manifest) =
            loader.get_plugin(&plugin_id).ok().map(|p| p.manifest())
        {
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
