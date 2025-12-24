//! Hardware discovery command for HORUS
//!
//! Discovers and displays connected hardware devices.

use colored::*;
use horus_core::error::HorusResult;
use horus_core::hardware::{
    DeviceCategory, DiscoveryOptions, HardwareDiscovery, Platform, PlatformDetector,
};

/// Run hardware scan
pub fn run_scan(options: HardwareScanOptions) -> HorusResult<()> {
    println!("{}", "HORUS Hardware Discovery".green().bold());
    println!();

    let discovery_opts = DiscoveryOptions {
        scan_usb: options.usb,
        scan_serial: options.serial,
        scan_i2c: options.i2c,
        probe_i2c: options.probe_i2c,
        scan_gpio: options.gpio,
        scan_cameras: options.cameras,
        ..Default::default()
    };

    let mut discovery = HardwareDiscovery::with_options(discovery_opts)
        .map_err(|e| horus_core::error::HorusError::Config(e))?;

    let report = discovery.scan_all();
    let summary = report.summary();

    // Platform info
    println!("{}", "Platform".cyan().bold());
    println!("  {} {}", "Detected:".dimmed(), report.platform.name());
    if report.platform.is_raspberry_pi() {
        println!("  {} Raspberry Pi family", "Type:".dimmed());
    } else if report.platform.is_jetson() {
        println!("  {} NVIDIA Jetson family (CUDA supported)", "Type:".dimmed());
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

            println!(
                "  {} {} [{}]",
                vid_pid.yellow(),
                name,
                category.dimmed()
            );

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

    let mut discovery = HardwareDiscovery::new()
        .map_err(|e| horus_core::error::HorusError::Config(e))?;

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
        }
    }
}
