//! `horus setup-rt` — Configure real-time kernel and system settings.
//!
//! Detects the Linux distribution, installs the RT kernel package,
//! configures memory lock limits, and optionally isolates CPU cores.
//!
//! All operations require root (sudo). The command is interactive by default.

use colored::*;
use std::process::Command;

/// Run the setup-rt command.
pub fn run_setup_rt(check_only: bool, undo: bool) -> anyhow::Result<()> {
    if undo {
        return run_undo();
    }

    println!("{}", "HORUS Real-Time Setup".green().bold());
    println!();

    // Step 1: Detect current RT status
    let caps = horus_sys::rt::detect_capabilities();
    print_rt_status(&caps);

    if check_only {
        return Ok(());
    }

    if caps.preempt_rt {
        println!();
        println!(
            "  {} PREEMPT_RT already active. No kernel install needed.",
            "✓".green()
        );
        // Still check limits and isolation
        ensure_memlock_limits()?;
        suggest_cpu_isolation(&caps)?;
        return Ok(());
    }

    println!();

    // Step 2: Detect distro and install RT kernel
    let distro = detect_distro();
    match distro.as_str() {
        "ubuntu" | "debian" => install_rt_debian()?,
        "fedora" => install_rt_fedora()?,
        "arch" => install_rt_arch()?,
        other => {
            println!(
                "  {} Unsupported distro '{}'. Install an RT kernel manually:",
                "✗".red(),
                other
            );
            println!("    - Ubuntu/Debian: sudo apt install linux-image-rt-amd64");
            println!("    - Fedora: sudo dnf install kernel-rt kernel-rt-core");
            println!("    - Arch: sudo pacman -S linux-rt linux-rt-headers");
            println!();
            println!("    After installing, reboot and run `horus setup-rt --check`.");
            return Ok(());
        }
    }

    // Step 3: Configure memory lock limits
    ensure_memlock_limits()?;

    // Step 4: Suggest CPU isolation
    suggest_cpu_isolation(&caps)?;

    // Step 5: Done
    println!();
    println!("{}", "Setup complete.".green().bold());
    println!();
    println!("  {} Reboot to activate the RT kernel:", "→".cyan());
    println!("    sudo reboot");
    println!();
    println!("  {} After reboot, verify with:", "→".cyan());
    println!("    horus setup-rt --check");
    println!();

    Ok(())
}

/// Print current RT capabilities.
fn print_rt_status(caps: &horus_sys::rt::RtCapabilities) {
    println!("  {} {}", "Kernel:".dimmed(), caps.kernel_version);

    if caps.preempt_rt {
        println!("  {} PREEMPT_RT: {}", "✓".green(), "active".green().bold());
    } else {
        println!("  {} PREEMPT_RT: {}", "⚠".yellow(), "not detected".yellow());
    }

    if caps.max_priority > 0 {
        println!(
            "  {} SCHED_FIFO: available (priority {}-{})",
            "✓".green(),
            caps.min_priority,
            caps.max_priority
        );
    } else {
        println!("  {} SCHED_FIFO: {}", "✗".red(), "not available".red());
    }

    if caps.memory_locking {
        println!("  {} Memory locking: {}", "✓".green(), "available");
    } else {
        println!(
            "  {} Memory locking: {}",
            "⚠".yellow(),
            "limited (may cause page faults)".yellow()
        );
    }

    println!("  {} CPU cores: {}", "ℹ".dimmed(), caps.cpu_count);

    let isolated = horus_sys::rt::isolated_cores();
    if isolated.is_empty() {
        println!("  {} Isolated CPUs: {}", "ℹ".dimmed(), "none");
    } else {
        println!("  {} Isolated CPUs: {:?}", "✓".green(), isolated);
    }

    println!(
        "  {} Estimated jitter: ±{}μs",
        "ℹ".dimmed(),
        caps.estimated_jitter.as_micros()
    );
}

/// Detect Linux distribution from /etc/os-release.
fn detect_distro() -> String {
    if let Ok(content) = std::fs::read_to_string("/etc/os-release") {
        for line in content.lines() {
            if let Some(id) = line.strip_prefix("ID=") {
                return id.trim_matches('"').to_lowercase();
            }
        }
    }
    "unknown".to_string()
}

/// Install RT kernel on Debian/Ubuntu.
fn install_rt_debian() -> anyhow::Result<()> {
    let arch = std::env::consts::ARCH;
    let pkg = match arch {
        "x86_64" => "linux-image-rt-amd64",
        "aarch64" => "linux-image-rt-arm64",
        _ => {
            println!(
                "  {} No RT kernel package for architecture '{}'",
                "✗".red(),
                arch
            );
            return Ok(());
        }
    };

    println!("  {} Installing {} ...", "→".cyan(), pkg.white().bold());

    let status = Command::new("sudo")
        .args(["apt", "install", "-y", pkg])
        .status()?;

    if status.success() {
        println!("  {} {} installed", "✓".green(), pkg);
    } else {
        println!("  {} Failed to install {}. Try manually:", "✗".red(), pkg);
        println!("    sudo apt install {}", pkg);
    }

    Ok(())
}

/// Install RT kernel on Fedora.
fn install_rt_fedora() -> anyhow::Result<()> {
    println!("  {} Installing kernel-rt ...", "→".cyan());

    let status = Command::new("sudo")
        .args(["dnf", "install", "-y", "kernel-rt", "kernel-rt-core"])
        .status()?;

    if status.success() {
        println!("  {} kernel-rt installed", "✓".green());
    } else {
        println!(
            "  {} Failed. Try manually: sudo dnf install kernel-rt kernel-rt-core",
            "✗".red()
        );
    }

    Ok(())
}

/// Install RT kernel on Arch Linux.
fn install_rt_arch() -> anyhow::Result<()> {
    println!("  {} Installing linux-rt ...", "→".cyan());

    let status = Command::new("sudo")
        .args([
            "pacman",
            "-S",
            "--noconfirm",
            "linux-rt",
            "linux-rt-headers",
        ])
        .status()?;

    if status.success() {
        println!("  {} linux-rt installed", "✓".green());
    } else {
        println!(
            "  {} Failed. Try manually: sudo pacman -S linux-rt linux-rt-headers",
            "✗".red()
        );
    }

    Ok(())
}

/// Ensure /etc/security/limits.d/99-horus-rt.conf exists with unlimited memlock.
fn ensure_memlock_limits() -> anyhow::Result<()> {
    let limits_path = "/etc/security/limits.d/99-horus-rt.conf";
    let content = "# HORUS real-time memory locking\n* - memlock unlimited\n* - rtprio 99\n";

    if std::path::Path::new(limits_path).exists() {
        println!(
            "  {} Memory limits already configured ({})",
            "✓".green(),
            limits_path
        );
        return Ok(());
    }

    println!("  {} Configuring memory lock limits ...", "→".cyan());

    let status = Command::new("sudo")
        .args(["tee", limits_path])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::null())
        .spawn()
        .and_then(|mut child| {
            use std::io::Write;
            if let Some(ref mut stdin) = child.stdin {
                stdin.write_all(content.as_bytes())?;
            }
            child.wait()
        });

    match status {
        Ok(s) if s.success() => {
            println!("  {} {} created", "✓".green(), limits_path);
        }
        _ => {
            println!(
                "  {} Failed to create {}. Create manually with:",
                "⚠".yellow(),
                limits_path
            );
            println!(
                "    echo '* - memlock unlimited' | sudo tee {}",
                limits_path
            );
        }
    }

    Ok(())
}

/// Suggest CPU isolation if not already configured.
fn suggest_cpu_isolation(caps: &horus_sys::rt::RtCapabilities) -> anyhow::Result<()> {
    let isolated = horus_sys::rt::isolated_cores();
    if !isolated.is_empty() {
        println!(
            "  {} CPU isolation already configured: cores {:?}",
            "✓".green(),
            isolated
        );
        return Ok(());
    }

    if caps.cpu_count < 4 {
        println!(
            "  {} CPU isolation skipped ({} cores — need at least 4)",
            "ℹ".dimmed(),
            caps.cpu_count
        );
        return Ok(());
    }

    // Suggest isolating the last 2 cores
    let start = caps.cpu_count - 2;
    let end = caps.cpu_count - 1;

    println!();
    println!(
        "  {} CPU isolation recommended for best RT performance.",
        "ℹ".cyan()
    );
    println!(
        "    This dedicates cores {}-{} to HORUS RT threads (other processes won't use them).",
        start, end
    );
    println!("    To enable, add to /etc/default/grub:");
    println!(
        "      GRUB_CMDLINE_LINUX=\"isolcpus={},{} nohz_full={},{} rcu_nocbs={},{}\"",
        start, end, start, end, start, end
    );
    println!("    Then run: sudo update-grub && sudo reboot");

    Ok(())
}

/// Undo RT setup.
fn run_undo() -> anyhow::Result<()> {
    println!("{}", "HORUS Real-Time Undo".yellow().bold());
    println!();

    let limits_path = "/etc/security/limits.d/99-horus-rt.conf";
    if std::path::Path::new(limits_path).exists() {
        println!("  {} Removing {} ...", "→".cyan(), limits_path);
        let _ = Command::new("sudo").args(["rm", limits_path]).status();
        println!("  {} Removed", "✓".green());
    } else {
        println!(
            "  {} {} not found (nothing to remove)",
            "ℹ".dimmed(),
            limits_path
        );
    }

    println!();
    println!(
        "  {} RT kernel package not removed (manual action).",
        "ℹ".dimmed()
    );
    println!("    To remove: sudo apt remove linux-image-rt-*");
    println!();
    println!(
        "  {} CPU isolation not removed (manual action).",
        "ℹ".dimmed()
    );
    println!(
        "    Edit /etc/default/grub, remove isolcpus/nohz_full/rcu_nocbs, run sudo update-grub."
    );

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_distro_returns_string() {
        let distro = detect_distro();
        assert!(!distro.is_empty());
    }

    #[test]
    fn print_rt_status_does_not_panic() {
        let caps = horus_sys::rt::detect_capabilities();
        print_rt_status(&caps);
    }

    #[test]
    fn check_only_mode_succeeds() {
        // --check mode should never modify system state
        let result = run_setup_rt(true, false);
        assert!(result.is_ok());
    }
}
