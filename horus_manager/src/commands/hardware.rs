//! Hardware discovery command for HORUS
//!
//! Discovers and displays connected hardware devices.
//!
//! Features:
//! - Timeout-protected scanning (prevents hangs on faulty devices)
//! - Progress indicators for long scans
//! - JSON output for machine-readable results
//! - Category filtering
//! - Watch mode for hotplug events

use colored::*;
use horus_core::error::HorusResult;
use horus_core::hardware::{
    CategoryFilter, DeviceCategory, DiscoveredDevice, DiscoveryOptions, HardwareDiscovery,
    PlatformDetector, ProgressCallback,
};
use indicatif::{ProgressBar, ProgressStyle};
use notify::{Config, RecommendedWatcher, RecursiveMode, Watcher};
use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::Duration;

/// Run hardware scan
pub fn run_scan(options: HardwareScanOptions) -> HorusResult<()> {
    // JSON mode: minimal output, just return the JSON
    if options.json {
        return run_scan_json(options);
    }

    println!("{}", "HORUS Hardware Discovery".green().bold());
    println!();

    // Set up timeout - use a longer timeout for more reliable scanning
    let timeout = Duration::from_millis(options.timeout_ms.unwrap_or(500));

    let discovery_opts = DiscoveryOptions {
        scan_usb: options.usb,
        scan_serial: options.serial,
        scan_i2c: options.i2c,
        probe_i2c: options.probe_i2c,
        scan_gpio: options.gpio,
        scan_cameras: options.cameras,
        probe_timeout: timeout,
    };

    let mut discovery = HardwareDiscovery::with_options(discovery_opts)
        .map_err(|e| horus_core::error::HorusError::Config(e))?;

    // Set up progress bar
    let pb = ProgressBar::new(6);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("{spinner:.green} [{bar:40.cyan/blue}] {pos}/{len} {msg}")
            .unwrap_or_else(|_| ProgressStyle::default_bar())
            .progress_chars("█▓▒░"),
    );
    pb.set_message("Scanning...");

    // Create progress callback
    let pb_clone = pb.clone();
    let progress: ProgressCallback = Box::new(move |phase: &str, current: usize, _total: usize| {
        pb_clone.set_position(current as u64);
        pb_clone.set_message(phase.to_string());
    });

    let report = discovery.scan_all_with_progress(Some(progress));
    pb.finish_with_message("Done!");
    println!();

    // Apply category filter if specified
    let _filtered_devices = if let Some(ref filter_str) = options.filter {
        let filter = CategoryFilter::from_str(filter_str);
        report
            .all_devices
            .iter()
            .filter(|d| match_category_filter(d, &filter))
            .collect::<Vec<_>>()
    } else {
        report.all_devices.iter().collect::<Vec<_>>()
    };

    let summary = report.summary();

    // Platform info
    println!("{}", "Platform".cyan().bold());
    println!("  {} {}", "Detected:".dimmed(), report.platform.name());
    if report.platform.is_raspberry_pi() {
        println!("  {} Raspberry Pi family", "Type:".dimmed());
    } else if report.platform.is_jetson() {
        println!(
            "  {} NVIDIA Jetson family (CUDA supported)",
            "Type:".dimmed()
        );
    }
    println!();

    // USB Devices
    if options.usb && !report.usb_devices.is_empty() {
        println!("{}", "USB Devices".cyan().bold());
        for device in &report.usb_devices {
            let name = device.display_name();
            let vid_pid = device.vid_pid_string();

            let category = discovery
                .database()
                .lookup_usb(device.vendor_id, device.product_id)
                .map(|info| info.category.name())
                .unwrap_or("Unknown");

            println!("  {} {} [{}]", vid_pid.yellow(), name, category.dimmed());

            if options.verbose {
                if let Some(manufacturer) = &device.manufacturer {
                    println!("      {} {}", "Manufacturer:".dimmed(), manufacturer);
                }
                if let Some(serial) = &device.serial {
                    println!("      {} {}", "Serial:".dimmed(), serial);
                }
                if let Some(speed) = device.speed_mbps {
                    println!("      {} {} Mbps", "Speed:".dimmed(), speed);
                }
                if let Some(tty) = &device.tty_path {
                    println!("      {} {}", "TTY:".dimmed(), tty.display());
                }
                if let Some(video) = &device.video_path {
                    println!("      {} {}", "Video:".dimmed(), video.display());
                }
            }
        }
        println!();
    }

    // Serial Ports
    if options.serial && !report.serial_ports.is_empty() {
        println!("{}", "Serial Ports".cyan().bold());
        for port in &report.serial_ports {
            let name = port.display_name();
            let port_type = port.port_type.description();

            println!(
                "  {} {} [{}]",
                port.path.display().to_string().yellow(),
                name,
                port_type.dimmed()
            );

            if options.verbose {
                if let Some(vid_pid) = port.vid_pid_string() {
                    println!("      {} {}", "VID:PID:".dimmed(), vid_pid);
                }
                if let Some(driver) = &port.driver {
                    println!("      {} {}", "Driver:".dimmed(), driver);
                }
            }
        }
        println!();
    }

    // I2C Buses (Linux only)
    #[cfg(target_os = "linux")]
    if options.i2c && !report.i2c_buses.is_empty() {
        println!("{}", "I2C Buses".cyan().bold());
        for bus in &report.i2c_buses {
            let name = bus.name.as_deref().unwrap_or("Unknown");
            println!(
                "  {} {} [{}]",
                format!("i2c-{}", bus.bus_number).yellow(),
                bus.device_path.display(),
                name.dimmed()
            );
        }

        // I2C Devices
        if options.probe_i2c && !report.i2c_devices.is_empty() {
            println!();
            println!("{}", "I2C Devices".cyan().bold());
            for device in &report.i2c_devices {
                let driver_match = discovery.database().match_i2c_driver(device.address);
                let name = &driver_match.device_info.name;
                let category = driver_match.device_info.category.name();

                println!(
                    "  {} @ {} [{}]",
                    format!("0x{:02X}", device.address).yellow(),
                    format!("i2c-{}", device.bus_number),
                    name
                );

                if options.verbose {
                    println!("      {} {}", "Category:".dimmed(), category);
                    if let Some(driver) = &driver_match.device_info.horus_driver {
                        println!("      {} {}", "HORUS Driver:".dimmed(), driver);
                    }
                }
            }
        }
        println!();
    }

    // GPIO Chips (Linux only)
    #[cfg(target_os = "linux")]
    if options.gpio && !report.gpio_chips.is_empty() {
        println!("{}", "GPIO Chips".cyan().bold());
        for chip in &report.gpio_chips {
            println!(
                "  {} {} [{} lines]",
                chip.name.yellow(),
                chip.label,
                chip.num_lines
            );
        }
        println!();
    }

    // Cameras (Linux only)
    #[cfg(target_os = "linux")]
    if options.cameras && !report.cameras.is_empty() {
        println!("{}", "Cameras".cyan().bold());
        for camera in &report.cameras {
            let cam_type = camera.camera_type.name();
            println!(
                "  {} {} [{}]",
                camera.device_path.display().to_string().yellow(),
                camera.display_name(),
                cam_type.dimmed()
            );

            if options.verbose {
                println!("      {} {}", "Driver:".dimmed(), camera.driver);
                if !camera.formats.is_empty() {
                    let formats: Vec<_> = camera.formats.iter().map(|f| &f.fourcc).collect();
                    println!("      {} {:?}", "Formats:".dimmed(), formats);
                }
            }
        }
        println!();
    }

    // Summary
    println!("{}", "Summary".cyan().bold());
    println!(
        "  {} USB devices, {} serial ports",
        summary.usb_count, summary.serial_count
    );
    #[cfg(target_os = "linux")]
    println!(
        "  {} I2C buses, {} GPIO chips, {} cameras",
        summary.i2c_bus_count, summary.gpio_chip_count, summary.camera_count
    );
    println!("  {} total devices discovered", summary.total_devices);
    println!(
        "  {} devices with HORUS driver suggestions",
        summary.devices_with_drivers
    );
    println!("  {} in {:?}", "Scan completed".green(), summary.duration);

    // Errors
    if !report.errors.is_empty() {
        println!();
        println!("{}", "Warnings".yellow().bold());
        for error in &report.errors {
            println!("  {} {}", "".yellow(), error);
        }
    }

    Ok(())
}

/// Run platform detection
pub fn run_platform(verbose: bool) -> HorusResult<()> {
    let platform = PlatformDetector::detect();
    let capabilities = PlatformDetector::capabilities(&platform);

    println!("{}", "Platform Information".green().bold());
    println!();

    println!("{}", "Detected Platform".cyan().bold());
    println!("  {} {}", "Name:".dimmed(), platform.name());
    println!("  {} {:?}", "Variant:".dimmed(), platform);

    if platform.is_raspberry_pi() {
        println!("  {} {}", "Family:".dimmed(), "Raspberry Pi");
    } else if platform.is_jetson() {
        println!("  {} {}", "Family:".dimmed(), "NVIDIA Jetson");
        println!("  {} {}", "CUDA:".dimmed(), "Supported".green());
    } else if platform.is_beaglebone() {
        println!("  {} {}", "Family:".dimmed(), "BeagleBone");
    }
    println!();

    println!("{}", "Capabilities".cyan().bold());
    println!("  {} {} cores", "CPU:".dimmed(), capabilities.cpu_cores);
    println!("  {} {}", "Architecture:".dimmed(), capabilities.cpu_arch);
    println!("  {} {} MB", "RAM:".dimmed(), capabilities.ram_mb);

    if let Some(gpu) = &capabilities.gpu_type {
        println!("  {} {:?}", "GPU:".dimmed(), gpu);
    }
    println!();

    if verbose {
        println!("{}", "Interfaces".cyan().bold());

        if !capabilities.gpio_chips.is_empty() {
            println!(
                "  {} {:?} ({} pins)",
                "GPIO:".dimmed(),
                capabilities.gpio_chips,
                capabilities.gpio_pins
            );
        }

        if !capabilities.i2c_buses.is_empty() {
            println!("  {} Bus {:?}", "I2C:".dimmed(), capabilities.i2c_buses);
        }

        if !capabilities.spi_buses.is_empty() {
            println!("  {} Bus {:?}", "SPI:".dimmed(), capabilities.spi_buses);
        }

        if !capabilities.uart_ports.is_empty() {
            println!("  {} {:?}", "UART:".dimmed(), capabilities.uart_ports);
        }

        if !capabilities.pwm_chips.is_empty() {
            println!("  {} Chip {:?}", "PWM:".dimmed(), capabilities.pwm_chips);
        }

        if !capabilities.can_interfaces.is_empty() {
            println!("  {} {:?}", "CAN:".dimmed(), capabilities.can_interfaces);
        }

        if capabilities.has_camera_csi {
            println!("  {} {}", "CSI:".dimmed(), "Available".green());
        }

        if capabilities.has_display_dsi {
            println!("  {} {}", "DSI:".dimmed(), "Available".green());
        }

        if capabilities.has_pcie {
            println!("  {} {}", "PCIe:".dimmed(), "Available".green());
        }

        if capabilities.has_nvme {
            println!("  {} {}", "NVMe:".dimmed(), "Available".green());
        }
    }

    Ok(())
}

/// Run configuration suggestion
pub fn run_suggest(verbose: bool) -> HorusResult<()> {
    println!("{}", "HORUS Configuration Suggestion".green().bold());
    println!();

    let mut discovery =
        HardwareDiscovery::new().map_err(|e| horus_core::error::HorusError::Config(e))?;

    let suggestion = discovery.suggest_configuration();

    println!(
        "{} {}",
        "Platform:".cyan().bold(),
        suggestion.platform.name()
    );
    println!();

    if suggestion.nodes.is_empty() {
        println!(
            "{}",
            "No robotics hardware detected that maps to HORUS nodes.".yellow()
        );
        println!("  Connect sensors, motors, or development boards to get suggestions.");
    } else {
        println!("{}", "Suggested Nodes".cyan().bold());
        for node in &suggestion.nodes {
            println!(
                "  {} {} - {}",
                "[+]".green(),
                node.node_type.yellow(),
                node.device_name
            );

            if verbose && !node.config.is_empty() {
                for (key, value) in &node.config {
                    println!("      {}: {}", key.dimmed(), value);
                }
            }
        }
    }

    if !suggestion.warnings.is_empty() {
        println!();
        println!("{}", "Platform Warnings".yellow().bold());
        for warning in &suggestion.warnings {
            println!("  {} {}", "".yellow(), warning);
        }
    }

    Ok(())
}

/// Options for hardware scan command
#[derive(Debug, Clone)]
pub struct HardwareScanOptions {
    /// Scan USB devices
    pub usb: bool,
    /// Scan serial ports
    pub serial: bool,
    /// Scan I2C buses
    pub i2c: bool,
    /// Probe I2C addresses (may require root)
    pub probe_i2c: bool,
    /// Scan GPIO chips
    pub gpio: bool,
    /// Scan cameras
    pub cameras: bool,
    /// Verbose output
    pub verbose: bool,
    /// Output as JSON
    pub json: bool,
    /// Category filter (comma-separated: usb,serial,i2c,gpio,cameras,sensors,motors)
    pub filter: Option<String>,
    /// Timeout per probe in milliseconds
    pub timeout_ms: Option<u64>,
    /// Watch mode for hotplug events
    pub watch: bool,
}

impl Default for HardwareScanOptions {
    fn default() -> Self {
        Self {
            usb: true,
            serial: true,
            i2c: true,
            probe_i2c: false,
            gpio: true,
            cameras: true,
            verbose: false,
            json: false,
            filter: None,
            timeout_ms: Some(500),
            watch: false,
        }
    }
}

/// Helper function to match a device against a category filter
fn match_category_filter(
    device: &horus_core::hardware::DiscoveredDevice,
    filter: &CategoryFilter,
) -> bool {
    match &device.category {
        DeviceCategory::SerialAdapter => filter.serial,
        DeviceCategory::Camera | DeviceCategory::DepthCamera => filter.cameras,
        DeviceCategory::Imu | DeviceCategory::Gps | DeviceCategory::Lidar => filter.sensors,
        DeviceCategory::MotorController | DeviceCategory::ServoController => filter.motors,
        _ => filter.usb, // Default to USB filter for other categories
    }
}

/// Run hardware scan with JSON output
fn run_scan_json(options: HardwareScanOptions) -> HorusResult<()> {
    let timeout = Duration::from_millis(options.timeout_ms.unwrap_or(500));

    let discovery_opts = DiscoveryOptions {
        scan_usb: options.usb,
        scan_serial: options.serial,
        scan_i2c: options.i2c,
        probe_i2c: options.probe_i2c,
        scan_gpio: options.gpio,
        scan_cameras: options.cameras,
        probe_timeout: timeout,
    };

    let mut discovery = HardwareDiscovery::with_options(discovery_opts)
        .map_err(|e| horus_core::error::HorusError::Config(e))?;

    let report = discovery.scan_all();

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&report)
        .map_err(|e| horus_core::error::HorusError::Config(e.to_string()))?;

    println!("{}", json);
    Ok(())
}

/// Export hardware configuration to a TOML file
pub fn run_export(output_path: Option<String>, _verbose: bool) -> HorusResult<()> {
    println!("{}", "HORUS Hardware Export".green().bold());
    println!();

    let mut discovery =
        HardwareDiscovery::new().map_err(|e| horus_core::error::HorusError::Config(e))?;

    let suggestion = discovery.suggest_configuration();

    // Build TOML configuration
    let mut config = String::new();
    config.push_str("# HORUS Hardware Configuration\n");
    config.push_str(&format!(
        "# Generated for platform: {}\n",
        suggestion.platform.name()
    ));
    config.push_str("# Auto-detected from connected hardware\n\n");

    config.push_str("[platform]\n");
    config.push_str(&format!("name = \"{}\"\n", suggestion.platform.name()));
    config.push_str("\n");

    if !suggestion.nodes.is_empty() {
        config.push_str("[nodes]\n\n");
        for node in &suggestion.nodes {
            config.push_str(&format!(
                "[[nodes.{}]]\n",
                node.node_type.to_lowercase().replace(' ', "_")
            ));
            config.push_str(&format!("device_name = \"{}\"\n", node.device_name));
            for (key, value) in &node.config {
                config.push_str(&format!("{} = \"{}\"\n", key, value));
            }
            config.push_str("\n");
        }
    }

    if !suggestion.warnings.is_empty() {
        config.push_str("# Warnings:\n");
        for warning in &suggestion.warnings {
            config.push_str(&format!("#   - {}\n", warning));
        }
    }

    // Output
    if let Some(path) = output_path {
        std::fs::write(&path, &config).map_err(|e| horus_core::error::HorusError::Io(e))?;
        println!("  {} Configuration written to {}", "[+]".green(), path);
    } else {
        println!("{}", config);
    }

    Ok(())
}

/// Get detailed information about a specific device
pub fn run_device_info(device_path: &str, verbose: bool) -> HorusResult<()> {
    println!("{}", "HORUS Device Info".green().bold());
    println!();

    let mut discovery =
        HardwareDiscovery::new().map_err(|e| horus_core::error::HorusError::Config(e))?;

    let report = discovery.scan_all();

    // Find the device by path
    let device = report.all_devices.iter().find(|d| {
        d.path
            .as_ref()
            .map_or(false, |p| p.to_string_lossy().contains(device_path))
    });

    match device {
        Some(d) => {
            println!("{} {}", "Device:".cyan().bold(), d.name);
            println!("  {} {:?}", "Category:".dimmed(), d.category);

            if let Some(path) = &d.path {
                println!("  {} {}", "Path:".dimmed(), path.display());
            }

            if let Some((vid, pid)) = d.vid_pid {
                println!("  {} {:04X}:{:04X}", "VID:PID:".dimmed(), vid, pid);
            }

            if let Some(mfr) = &d.manufacturer {
                println!("  {} {}", "Manufacturer:".dimmed(), mfr);
            }

            if let Some(driver) = &d.suggested_driver {
                println!(
                    "  {} {}",
                    "Suggested HORUS Driver:".dimmed(),
                    driver.green()
                );
            }

            if !d.properties.is_empty() {
                println!("\n{}", "Properties".cyan().bold());
                for (key, value) in &d.properties {
                    println!("  {} {}", format!("{}:", key).dimmed(), value);
                }
            }

            if verbose {
                if let Some(ref dm) = d.driver_match {
                    println!("\n{}", "Driver Match Details".cyan().bold());
                    println!("  {} {:?}", "Confidence:".dimmed(), dm.confidence);
                    println!(
                        "  {} {}",
                        "Description:".dimmed(),
                        dm.device_info.description.as_deref().unwrap_or("N/A")
                    );
                }
            }
        }
        None => {
            println!("{}", format!("Device not found: {}", device_path).red());
            println!("Use 'horus hardware scan' to see available devices.");
        }
    }

    Ok(())
}

/// Run hardware watch mode for hotplug events
pub fn run_watch(options: HardwareScanOptions) -> HorusResult<()> {
    println!("{}", "HORUS Hardware Watch Mode".green().bold());
    println!();
    println!(
        "{}",
        "Monitoring for device connect/disconnect events...".cyan()
    );
    println!("{}", "Press Ctrl+C to stop watching.".dimmed());
    println!();

    // Get initial device snapshot
    let mut current_devices = get_device_snapshot(&options)?;

    println!(
        "  {} {} devices currently connected",
        "[+]".green(),
        current_devices.len()
    );
    println!();

    // Create channel for filesystem events
    let (tx, rx) = mpsc::channel();

    // Create a watcher with a debounce of 500ms
    let config = Config::default().with_poll_interval(Duration::from_millis(500));

    let mut watcher = RecommendedWatcher::new(
        move |res: Result<notify::Event, notify::Error>| {
            if let Ok(event) = res {
                let _ = tx.send(event);
            }
        },
        config,
    )
    .map_err(|e| {
        horus_core::error::HorusError::Config(format!("Failed to create watcher: {}", e))
    })?;

    // Watch /dev directory for device changes
    let dev_path = PathBuf::from("/dev");
    watcher
        .watch(&dev_path, RecursiveMode::NonRecursive)
        .map_err(|e| {
            horus_core::error::HorusError::Config(format!("Failed to watch /dev: {}", e))
        })?;

    // Also watch /dev/serial for serial port changes
    let serial_path = PathBuf::from("/dev/serial");
    if serial_path.exists() {
        let _ = watcher.watch(&serial_path, RecursiveMode::Recursive);
    }

    // Also watch /dev/bus/usb for USB changes
    let usb_bus_path = PathBuf::from("/dev/bus/usb");
    if usb_bus_path.exists() {
        let _ = watcher.watch(&usb_bus_path, RecursiveMode::Recursive);
    }

    // Set up Ctrl+C handler
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .map_err(|e| {
        horus_core::error::HorusError::Config(format!("Failed to set Ctrl+C handler: {}", e))
    })?;

    // Debounce timer - only rescan after events settle
    let mut last_event_time = std::time::Instant::now();
    let debounce_duration = Duration::from_millis(1000);

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        // Wait for events with timeout
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(event) => {
                // Filter for relevant event types
                use notify::EventKind;
                match event.kind {
                    EventKind::Create(_) | EventKind::Remove(_) | EventKind::Modify(_) => {
                        last_event_time = std::time::Instant::now();
                    }
                    _ => {}
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                // Check if we should rescan after debounce period
                if last_event_time.elapsed() >= debounce_duration
                    && last_event_time.elapsed() < debounce_duration + Duration::from_millis(200)
                {
                    // Rescan and compare
                    if let Ok(new_devices) = get_device_snapshot(&options) {
                        show_device_changes(&current_devices, &new_devices);
                        current_devices = new_devices;
                    }
                }
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    println!();
    println!("{}", "Watch mode stopped.".yellow());

    Ok(())
}

/// Get a snapshot of currently connected devices
fn get_device_snapshot(options: &HardwareScanOptions) -> HorusResult<HashSet<DeviceKey>> {
    let timeout = Duration::from_millis(options.timeout_ms.unwrap_or(500));

    let discovery_opts = DiscoveryOptions {
        scan_usb: options.usb,
        scan_serial: options.serial,
        scan_i2c: options.i2c,
        probe_i2c: options.probe_i2c,
        scan_gpio: options.gpio,
        scan_cameras: options.cameras,
        probe_timeout: timeout,
    };

    let mut discovery = HardwareDiscovery::with_options(discovery_opts)
        .map_err(|e| horus_core::error::HorusError::Config(e))?;

    let report = discovery.scan_all();

    Ok(report
        .all_devices
        .iter()
        .map(|d| DeviceKey::from_device(d))
        .collect())
}

/// Show changes between device snapshots
fn show_device_changes(old: &HashSet<DeviceKey>, new: &HashSet<DeviceKey>) {
    let timestamp = chrono::Local::now().format("%H:%M:%S");

    // Find new devices (connected)
    for device in new.difference(old) {
        println!(
            "  {} [{}] {} {}",
            "[+]".green().bold(),
            timestamp.to_string().dimmed(),
            "Connected:".green(),
            device.display_name()
        );
    }

    // Find removed devices (disconnected)
    for device in old.difference(new) {
        println!(
            "  {} [{}] {} {}",
            "[-]".red().bold(),
            timestamp.to_string().dimmed(),
            "Disconnected:".red(),
            device.display_name()
        );
    }
}

/// Key for identifying unique devices
#[derive(Debug, Clone, Hash, Eq, PartialEq)]
struct DeviceKey {
    name: String,
    path: Option<String>,
    vid_pid: Option<(u16, u16)>,
    category: String,
}

impl DeviceKey {
    fn from_device(device: &DiscoveredDevice) -> Self {
        Self {
            name: device.name.clone(),
            path: device
                .path
                .as_ref()
                .map(|p| p.to_string_lossy().to_string()),
            vid_pid: device.vid_pid,
            category: format!("{:?}", device.category),
        }
    }

    fn display_name(&self) -> String {
        if let Some((vid, pid)) = self.vid_pid {
            format!("{} [{:04X}:{:04X}]", self.name, vid, pid)
        } else if let Some(ref path) = self.path {
            format!("{} ({})", self.name, path)
        } else {
            self.name.clone()
        }
    }
}
