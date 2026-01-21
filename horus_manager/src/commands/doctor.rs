//! Doctor command - System diagnostics for HORUS
//!
//! Checks system configuration and reports on environment health.

use colored::*;
use horus_core::error::HorusResult;
use std::path::Path;
use std::process::Command;

/// Represents a diagnostic check result
#[derive(Debug)]
pub enum CheckStatus {
    Ok,
    Warning,
    Error,
}

/// Run all diagnostic checks
pub fn run_doctor(verbose: bool) -> HorusResult<()> {
    println!("{}", "HORUS System Diagnostics".green().bold());
    println!();

    let mut warnings = 0;
    let mut errors = 0;

    // Rust toolchain
    let (status, msg) = check_rust_toolchain(verbose);
    print_check("Rust toolchain", status, &msg, &mut warnings, &mut errors);

    // Cargo
    let (status, msg) = check_cargo(verbose);
    print_check("Cargo", status, &msg, &mut warnings, &mut errors);

    // Shared memory
    let (status, msg) = check_shared_memory(verbose);
    print_check("Shared memory", status, &msg, &mut warnings, &mut errors);

    // Python bindings
    let (status, msg) = check_python(verbose);
    print_check("Python bindings", status, &msg, &mut warnings, &mut errors);

    // GPU availability
    let (status, msg) = check_gpu(verbose);
    print_check("GPU (sim3d)", status, &msg, &mut warnings, &mut errors);

    // Network
    let (status, msg) = check_network(verbose);
    print_check("Network", status, &msg, &mut warnings, &mut errors);

    // Registry
    let (status, msg) = check_registry(verbose);
    print_check("Registry", status, &msg, &mut warnings, &mut errors);

    // Node.js (for monitor)
    let (status, msg) = check_nodejs(verbose);
    print_check(
        "Node.js (monitor)",
        status,
        &msg,
        &mut warnings,
        &mut errors,
    );

    // Summary
    println!();
    if errors > 0 {
        println!("{} {} error(s), {} warning(s)", "".red(), errors, warnings);
        println!(
            "  {} Run `horus doctor --verbose` for more details",
            "Tip:".dimmed()
        );
    } else if warnings > 0 {
        println!("{} {} warning(s), no errors", "".yellow(), warnings);
        println!(
            "  {} Some features may not work optimally",
            "Note:".dimmed()
        );
    } else {
        println!("{} All checks passed!", "".green());
    }

    Ok(())
}

fn print_check(name: &str, status: CheckStatus, msg: &str, warnings: &mut i32, errors: &mut i32) {
    let (icon, color_msg) = match status {
        CheckStatus::Ok => ("[OK]".green(), msg.normal()),
        CheckStatus::Warning => {
            *warnings += 1;
            ("[WARN]".yellow(), msg.yellow())
        }
        CheckStatus::Error => {
            *errors += 1;
            ("[ERR]".red(), msg.red())
        }
    };
    println!("  {} {} {}", icon, format!("{:20}", name).cyan(), color_msg);
}

fn check_rust_toolchain(_verbose: bool) -> (CheckStatus, String) {
    match Command::new("rustc").arg("--version").output() {
        Ok(output) => {
            if output.status.success() {
                let version = String::from_utf8_lossy(&output.stdout);
                let version = version.trim();

                // Extract version number
                if let Some(ver) = version.split_whitespace().nth(1) {
                    let parts: Vec<&str> = ver.split('.').collect();
                    if parts.len() >= 2 {
                        if let (Ok(major), Ok(minor)) =
                            (parts[0].parse::<u32>(), parts[1].parse::<u32>())
                        {
                            // HORUS requires Rust 1.70+
                            if major >= 1 && minor >= 70 {
                                return (CheckStatus::Ok, ver.to_string());
                            } else {
                                return (
                                    CheckStatus::Warning,
                                    format!("{} (1.70+ recommended)", ver),
                                );
                            }
                        }
                    }
                }
                (CheckStatus::Ok, version.to_string())
            } else {
                (CheckStatus::Error, "Not installed".to_string())
            }
        }
        Err(_) => (CheckStatus::Error, "Not found in PATH".to_string()),
    }
}

fn check_cargo(_verbose: bool) -> (CheckStatus, String) {
    match Command::new("cargo").arg("--version").output() {
        Ok(output) => {
            if output.status.success() {
                let version = String::from_utf8_lossy(&output.stdout);
                let version = version.trim();
                if let Some(ver) = version.split_whitespace().nth(1) {
                    return (CheckStatus::Ok, ver.to_string());
                }
                (CheckStatus::Ok, version.to_string())
            } else {
                (CheckStatus::Error, "Not working".to_string())
            }
        }
        Err(_) => (CheckStatus::Error, "Not found in PATH".to_string()),
    }
}

fn check_shared_memory(verbose: bool) -> (CheckStatus, String) {
    let shm_path = if cfg!(target_os = "macos") {
        "/tmp/horus"
    } else {
        "/dev/shm"
    };

    let path = Path::new(shm_path);
    if path.exists() {
        // Try to create a test file
        let test_path = path.join("horus_doctor_test");
        match std::fs::write(&test_path, "test") {
            Ok(_) => {
                let _ = std::fs::remove_file(&test_path);
                if verbose {
                    (CheckStatus::Ok, format!("{} (writable)", shm_path))
                } else {
                    (CheckStatus::Ok, "Accessible".to_string())
                }
            }
            Err(e) => {
                if verbose {
                    (
                        CheckStatus::Error,
                        format!("{} not writable: {}", shm_path, e),
                    )
                } else {
                    (CheckStatus::Error, "Not writable".to_string())
                }
            }
        }
    } else {
        (CheckStatus::Error, format!("{} not found", shm_path))
    }
}

fn check_python(verbose: bool) -> (CheckStatus, String) {
    // Check if horus-robotics is installed (module name is 'horus')
    match Command::new("python3")
        .args(["-c", "import horus; print(horus.__version__)"])
        .output()
    {
        Ok(output) => {
            if output.status.success() {
                let version = String::from_utf8_lossy(&output.stdout);
                (CheckStatus::Ok, format!("horus-robotics {}", version.trim()))
            } else {
                // Try to check if maturin is available for building
                match Command::new("pip3").args(["show", "maturin"]).output() {
                    Ok(maturin_output) if maturin_output.status.success() => (
                        CheckStatus::Warning,
                        "Not installed (maturin available)".to_string(),
                    ),
                    _ => {
                        if verbose {
                            (
                                CheckStatus::Warning,
                                "Not installed (pip install horus-robotics)".to_string(),
                            )
                        } else {
                            (CheckStatus::Warning, "Not installed".to_string())
                        }
                    }
                }
            }
        }
        Err(_) => (CheckStatus::Warning, "Python3 not found".to_string()),
    }
}

fn check_gpu(_verbose: bool) -> (CheckStatus, String) {
    // Check for NVIDIA GPU
    if let Ok(output) = Command::new("nvidia-smi")
        .arg("--query-gpu=name")
        .arg("--format=csv,noheader")
        .output()
    {
        if output.status.success() {
            let gpu_name = String::from_utf8_lossy(&output.stdout);
            let gpu_name = gpu_name.trim();
            if !gpu_name.is_empty() {
                return (
                    CheckStatus::Ok,
                    gpu_name.lines().next().unwrap_or("NVIDIA GPU").to_string(),
                );
            }
        }
    }

    // Check for Vulkan support (cross-platform)
    if let Ok(output) = Command::new("vulkaninfo").arg("--summary").output() {
        if output.status.success() {
            let info = String::from_utf8_lossy(&output.stdout);
            if info.contains("GPU") || info.contains("deviceName") {
                return (CheckStatus::Ok, "Vulkan available".to_string());
            }
        }
    }

    // Check for OpenGL (basic)
    if let Ok(output) = Command::new("glxinfo").args(["-B"]).output() {
        if output.status.success() {
            return (CheckStatus::Ok, "OpenGL available".to_string());
        }
    }

    // On macOS, check for Metal
    #[cfg(target_os = "macos")]
    {
        return (CheckStatus::Ok, "Metal (macOS)".to_string());
    }

    #[cfg(not(target_os = "macos"))]
    (
        CheckStatus::Warning,
        "Not detected (software rendering)".to_string(),
    )
}

fn check_network(_verbose: bool) -> (CheckStatus, String) {
    // Check if we can bind to localhost
    match std::net::TcpListener::bind("127.0.0.1:0") {
        Ok(listener) => {
            let port = listener.local_addr().map(|a| a.port()).unwrap_or(0);
            drop(listener);
            (CheckStatus::Ok, format!("Localhost OK (port {})", port))
        }
        Err(e) => (CheckStatus::Error, format!("Cannot bind: {}", e)),
    }
}

fn check_registry(_verbose: bool) -> (CheckStatus, String) {
    // Try to connect to the registry
    match std::net::TcpStream::connect_timeout(
        &"registry.horus-robotics.dev:443"
            .parse()
            .unwrap_or_else(|_| "0.0.0.0:443".parse().unwrap()),
        std::time::Duration::from_secs(3),
    ) {
        Ok(_) => (CheckStatus::Ok, "Connected".to_string()),
        Err(_) => {
            // DNS might not resolve, try a different approach
            match Command::new("curl")
                .args([
                    "--connect-timeout",
                    "3",
                    "-s",
                    "-o",
                    "/dev/null",
                    "-w",
                    "%{http_code}",
                    "https://registry.horus-robotics.dev/api/health",
                ])
                .output()
            {
                Ok(output) => {
                    let code = String::from_utf8_lossy(&output.stdout);
                    if code.trim() == "200" {
                        (CheckStatus::Ok, "Connected".to_string())
                    } else {
                        (
                            CheckStatus::Warning,
                            "Unreachable (offline mode available)".to_string(),
                        )
                    }
                }
                Err(_) => (
                    CheckStatus::Warning,
                    "Unreachable (offline mode available)".to_string(),
                ),
            }
        }
    }
}

fn check_nodejs(_verbose: bool) -> (CheckStatus, String) {
    match Command::new("node").arg("--version").output() {
        Ok(output) => {
            if output.status.success() {
                let version = String::from_utf8_lossy(&output.stdout);
                let version = version.trim();

                // Check if version is 18+
                if let Some(ver) = version.strip_prefix('v') {
                    if let Some(major) = ver.split('.').next() {
                        if let Ok(major_num) = major.parse::<u32>() {
                            if major_num >= 18 {
                                return (CheckStatus::Ok, version.to_string());
                            } else {
                                return (
                                    CheckStatus::Warning,
                                    format!("{} (18+ recommended)", version),
                                );
                            }
                        }
                    }
                }
                (CheckStatus::Ok, version.to_string())
            } else {
                (CheckStatus::Warning, "Not working".to_string())
            }
        }
        Err(_) => (
            CheckStatus::Warning,
            "Not found (dashboard optional)".to_string(),
        ),
    }
}
