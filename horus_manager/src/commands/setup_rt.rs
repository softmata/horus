//! `horus setup-rt` — Configure real-time kernel and system settings.
//!
//! Detects the Linux distribution, installs the RT kernel package,
//! configures memory lock limits, and optionally isolates CPU cores.
//!
//! Every step that changes the host asks first — the kernel install and the
//! limits file alike. `HORUS_ASSUME_YES=1` accepts on behalf of a provisioning
//! script; without a TTY and without that variable the answer is no.
//!
//! All operations require root (sudo).

use colored::*;
use std::process::Command;

/// The limits file `setup-rt` owns. Written by setup, removed by `--undo`.
const LIMITS_PATH: &str = "/etc/security/limits.d/99-horus-rt.conf";

/// Runs a privileged command and reports whether it succeeded.
///
/// Injected so the install and undo paths can be tested without a package
/// manager and without root: the previous tests could only reach `confirm()`
/// and `rt_kernel_package()`, neither of which the Fedora and Arch installs
/// called, so those two paths were guarded by tests that never touched them.
type Runner<'a> = &'a dyn Fn(&[&str]) -> anyhow::Result<bool>;

/// Run `argv` (with `sudo` as `argv[0]`) and report success.
fn run_privileged(argv: &[&str]) -> anyhow::Result<bool> {
    let (bin, args) = argv.split_first().expect("argv must not be empty");
    Ok(Command::new(bin).args(args).status()?.success())
}

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
    let arch = std::env::consts::ARCH;
    if rt_kernel(&distro, arch).is_none() {
        println!(
            "  {} No RT kernel package known for '{}' on {}. Install one manually:",
            "✗".red(),
            distro,
            arch
        );
        for line in manual_install_hints() {
            println!("    {line}");
        }
        println!();
        println!("    After installing, reboot and run `horus setup-rt --check`.");
        return Ok(());
    }
    install_rt_kernel(&distro, arch, &run_privileged)?;

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

    // Permission, not the kernel's priority range — see the note in
    // doctor.rs. `max_priority` is non-zero even when the call is refused.
    if caps.rt_priority_permitted {
        println!(
            "  {} SCHED_FIFO: available (priority {}-{})",
            "✓".green(),
            caps.min_priority,
            caps.max_priority
        );
    } else {
        // "REFUSED" is what the probe established; the cause is not.
        // `can_set_rt_priority()` returns false for a missing CAP_SYS_NICE,
        // for RLIMIT_RTPRIO=0, for a seccomp filter that rejects
        // sched_setscheduler, and for an unreadable thread policy — so the
        // line offers the fixes without asserting which one is needed.
        println!(
            "  {} SCHED_FIFO: {} (kernel offers {}-{}; this process may not use it)",
            "✗".red(),
            "REFUSED — not permitted to set an RT policy".red(),
            caps.min_priority,
            caps.max_priority
        );
        // Not "run `horus setup-rt`" — this *is* setup-rt, and in `--check`
        // mode nothing is written. Name the two mechanisms instead, and the
        // file a full run writes for the second.
        println!(
            "      {} sudo setcap cap_sys_nice+ep $(which horus), or an rtprio limit \
             for this user (a full `setup-rt` run writes {})",
            "fix:".bold(),
            LIMITS_PATH
        );
    }

    if caps.memory_locking {
        println!("  {} Memory locking: available", "✓".green());
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
        println!("  {} Isolated CPUs: none", "ℹ".dimmed());
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

/// Ask before an operation that changes the machine.
///
/// Returns `true` when the user accepts. Non-interactive runs (no TTY) decline;
/// `HORUS_ASSUME_YES=1` accepts, so scripted provisioning still works.
fn confirm(question: &str) -> anyhow::Result<bool> {
    use std::io::{IsTerminal, Write};

    if std::env::var("HORUS_ASSUME_YES")
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true") || v.eq_ignore_ascii_case("yes"))
        .unwrap_or(false)
    {
        return Ok(true);
    }
    if !std::io::stdin().is_terminal() {
        println!("  {} {question}", "?".cyan());
        println!(
            "  {} Not a terminal — declining. Re-run interactively or set HORUS_ASSUME_YES=1.",
            "→".cyan()
        );
        return Ok(false);
    }

    print!("  {} {question} [y/N]: ", "?".cyan());
    std::io::stdout().flush().ok();
    let mut input = String::new();
    std::io::stdin().read_line(&mut input)?;
    Ok(matches!(input.trim().to_lowercase().as_str(), "y" | "yes"))
}

/// Everything distro-specific about the RT kernel, in one table.
///
/// Ubuntu and Debian do not agree on the package name. Ubuntu has never
/// shipped `linux-image-rt-amd64` — its RT kernel is `linux-image-realtime`:
///
/// ```text
/// $ apt-cache policy linux-image-rt-amd64      # on Ubuntu 26.04
///   Candidate: (none)
/// $ apt-cache policy linux-image-realtime
///   Candidate: 7.0.0-29.29.1
/// ```
///
/// Both distros were routed to the Debian name, so on Ubuntu the install
/// always failed — and, because the failure was swallowed, `setup-rt` went on
/// to print "Setup complete" and "Reboot to activate the RT kernel". A user
/// rebooted believing they had PREEMPT_RT. For a framework whose value
/// proposition is deterministic timing, a false positive on the RT guarantee
/// is the worst available failure shape.
///
/// Fedora and Arch then kept their own package names and their own install
/// commands inline in their own functions, so that fix reached two of the four
/// paths: `--undo` still printed an apt glob that matches neither of the other
/// two (nor Ubuntu's own package), and the dnf/pacman installs still ran
/// unprompted. Everything distro-shaped lives here now so a fix cannot land on
/// one path and miss its siblings.
struct RtKernel {
    /// `ID=` from /etc/os-release.
    distro: &'static str,
    /// Architectures this entry is valid for.
    arches: &'static [&'static str],
    /// Human-readable distro name, for the manual-instructions list.
    label: &'static str,
    /// Packages that carry the RT kernel.
    packages: &'static [&'static str],
    /// argv that installs them, `sudo` first.
    install: &'static [&'static str],
    /// argv that removes them again, `sudo` first. Printed by `--undo`.
    remove: &'static [&'static str],
}

const RT_KERNELS: &[RtKernel] = &[
    RtKernel {
        distro: "ubuntu",
        arches: &["x86_64", "aarch64"],
        label: "Ubuntu",
        packages: &["linux-image-realtime"],
        install: &["sudo", "apt", "install", "-y", "linux-image-realtime"],
        remove: &["sudo", "apt", "remove", "linux-image-realtime"],
    },
    RtKernel {
        distro: "debian",
        arches: &["x86_64"],
        label: "Debian (x86_64)",
        packages: &["linux-image-rt-amd64"],
        install: &["sudo", "apt", "install", "-y", "linux-image-rt-amd64"],
        remove: &["sudo", "apt", "remove", "linux-image-rt-amd64"],
    },
    RtKernel {
        distro: "debian",
        arches: &["aarch64"],
        label: "Debian (aarch64)",
        packages: &["linux-image-rt-arm64"],
        install: &["sudo", "apt", "install", "-y", "linux-image-rt-arm64"],
        remove: &["sudo", "apt", "remove", "linux-image-rt-arm64"],
    },
    RtKernel {
        distro: "fedora",
        arches: &["x86_64", "aarch64"],
        label: "Fedora",
        packages: &["kernel-rt", "kernel-rt-core"],
        install: &[
            "sudo",
            "dnf",
            "install",
            "-y",
            "kernel-rt",
            "kernel-rt-core",
        ],
        remove: &["sudo", "dnf", "remove", "kernel-rt", "kernel-rt-core"],
    },
    RtKernel {
        distro: "arch",
        arches: &["x86_64", "aarch64"],
        label: "Arch",
        packages: &["linux-rt", "linux-rt-headers"],
        install: &[
            "sudo",
            "pacman",
            "-S",
            "--noconfirm",
            "linux-rt",
            "linux-rt-headers",
        ],
        remove: &["sudo", "pacman", "-R", "linux-rt", "linux-rt-headers"],
    },
];

/// The RT kernel entry for a distro/architecture, if HORUS knows one.
fn rt_kernel(distro: &str, arch: &str) -> Option<&'static RtKernel> {
    RT_KERNELS
        .iter()
        .find(|k| k.distro == distro && k.arches.contains(&arch))
}

/// `- Ubuntu: sudo apt install -y linux-image-realtime` for every known distro.
///
/// Generated from the table rather than written out again, so the list shown to
/// a user on an unsupported distro cannot drift from what HORUS actually runs.
fn manual_install_hints() -> Vec<String> {
    let mut seen = Vec::new();
    for k in RT_KERNELS {
        let line = format!("- {}: {}", k.label, k.install.join(" "));
        if !seen.contains(&line) {
            seen.push(line);
        }
    }
    seen
}

/// Install the distribution's RT kernel, after asking.
///
/// One path for all four distros. Ubuntu and Debian asked first and propagated
/// a failed install; Fedora ran `dnf install -y` and Arch ran
/// `pacman -S --noconfirm` with no prompt at all, and both returned `Ok` after
/// printing "Failed" — which put the caller straight back on the "Setup
/// complete. Reboot to activate the RT kernel" path for a kernel that was never
/// installed.
fn install_rt_kernel(distro: &str, arch: &str, run: Runner) -> anyhow::Result<()> {
    let Some(kernel) = rt_kernel(distro, arch) else {
        anyhow::bail!(
            "No RT kernel package known for {distro} on '{arch}'. \
             Install one manually, then run `horus setup-rt --check`."
        );
    };
    let packages = kernel.packages.join(", ");
    let command = kernel.install.join(" ");

    // Installing a different kernel is about the largest blast radius a CLI
    // has, and the docs promise a prompt before it happens.
    if !confirm(&format!(
        "Install the real-time kernel package(s) {packages} with `{command}`? \
         This changes which kernel the machine boots."
    ))? {
        println!("  {} Skipped. To do it manually:", "→".cyan());
        println!("    {command}");
        anyhow::bail!("RT kernel installation declined");
    }

    println!(
        "  {} Installing {} ...",
        "→".cyan(),
        packages.white().bold()
    );

    if !run(kernel.install)? {
        anyhow::bail!(
            "Failed to install {packages}. Install it manually and re-run:\n    {command}"
        );
    }

    println!("  {} {} installed", "✓".green(), packages);
    Ok(())
}

/// The contents of the limits file for `user`, or `None` when the caller
/// cannot be identified safely.
///
/// Scoped to the invoking user, not `*`. This file used to read
/// `* - memlock unlimited` / `* - rtprio 99`, which hands EVERY account on the
/// machine the ability to lock all of RAM (a trivial local DoS) and to run
/// SCHED_FIFO at priority 99 — high enough to starve the kernel's own threads,
/// and on a robot high enough to preempt the very control loops these limits
/// exist to protect.
fn memlock_limits_content(user: &str) -> Option<String> {
    let user = user.trim();
    // A limits.conf domain is a username, group (@name), or uid range. Anything
    // outside that charset means we cannot identify the caller safely.
    let sane = !user.is_empty()
        && user != "root"
        && user.len() <= 32
        && user
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-' | b'.'));
    if !sane {
        return None;
    }
    Some(format!(
        "# HORUS real-time limits — scoped to the user that ran `horus setup-rt`.\n\
         # Do NOT widen this to `*`: rtprio 99 for every account lets any local user\n\
         # preempt the control loops these limits exist to protect.\n\
         {user} - memlock unlimited\n\
         {user} - rtprio 99\n"
    ))
}

/// Write `content` to `path` as root.
fn sudo_write(path: &str, content: &str) -> anyhow::Result<bool> {
    let status = Command::new("sudo")
        .args(["tee", path])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::null())
        .spawn()
        .and_then(|mut child| {
            use std::io::Write;
            if let Some(ref mut stdin) = child.stdin {
                stdin.write_all(content.as_bytes())?;
            }
            child.wait()
        })?;
    Ok(status.success())
}

/// Ensure the limits file exists with an unlimited memlock for the caller.
fn ensure_memlock_limits() -> anyhow::Result<()> {
    // Under sudo, SUDO_USER is the human who invoked us; USER would be "root".
    let user = std::env::var("SUDO_USER")
        .or_else(|_| std::env::var("USER"))
        .unwrap_or_default();
    ensure_memlock_limits_at(LIMITS_PATH, &user, &|path, content| {
        sudo_write(path, content)
    })
}

/// `ensure_memlock_limits` with the path, the user and the writer injected.
///
/// The prompt is the point: this file grants `rtprio 99` and unlimited memlock
/// through sudo, and the documentation says `setup-rt` asks before changing the
/// host. It used to go straight to `sudo tee`, so the only prompt in the whole
/// command was the kernel one — and only on the apt path.
fn ensure_memlock_limits_at(
    path: &str,
    user: &str,
    write: &dyn Fn(&str, &str) -> anyhow::Result<bool>,
) -> anyhow::Result<()> {
    if std::path::Path::new(path).exists() {
        println!(
            "  {} Memory limits already configured ({})",
            "✓".green(),
            path
        );
        return Ok(());
    }

    let Some(content) = memlock_limits_content(user) else {
        println!(
            "  {} Could not determine the invoking user (SUDO_USER/USER), so RT limits\n    \
             were NOT configured. Re-run as a normal user via sudo, or add this by hand\n    \
             for the account that will run HORUS nodes:\n      \
             <user> - memlock unlimited\n      <user> - rtprio 99",
            "!".yellow()
        );
        return Ok(());
    };

    if !confirm(&format!(
        "Write {path} granting {} unlimited memlock and rtprio 99, with sudo?",
        user.trim()
    ))? {
        println!("  {} Skipped. To do it by hand:", "→".cyan());
        for line in content.lines().filter(|l| !l.starts_with('#')) {
            println!("      {line}");
        }
        return Ok(());
    }

    println!("  {} Configuring memory lock limits ...", "→".cyan());

    match write(path, &content) {
        Ok(true) => println!("  {} {} created", "✓".green(), path),
        _ => {
            println!(
                "  {} Failed to create {}. Create manually with:",
                "⚠".yellow(),
                path
            );
            for line in content.lines().filter(|l| !l.starts_with('#')) {
                println!("      {line}");
            }
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
    for line in undo_report(
        LIMITS_PATH,
        &detect_distro(),
        std::env::consts::ARCH,
        &run_privileged,
    )? {
        println!("{line}");
    }
    Ok(())
}

/// The lines `--undo` prints, built as data so they can be asserted on.
///
/// Two bugs lived in the printed text. It said `sudo apt remove
/// linux-image-rt-*` on every distro — a glob that matches neither Ubuntu's
/// `linux-image-realtime` nor Fedora's `kernel-rt` nor Arch's `linux-rt`, i.e.
/// the Ubuntu package-name fix was applied to install and not to undo. And it
/// printed "✓ Removed" from a `let _ = ...status()`, so a failed `rm` still
/// reported success.
fn undo_report(
    limits_path: &str,
    distro: &str,
    arch: &str,
    run: Runner,
) -> anyhow::Result<Vec<String>> {
    let mut lines = Vec::new();

    if std::path::Path::new(limits_path).exists() {
        lines.push(format!("  {} Removing {} ...", "→".cyan(), limits_path));
        if run(&["sudo", "rm", limits_path])? {
            lines.push(format!("  {} Removed {}", "✓".green(), limits_path));
        } else {
            lines.push(format!(
                "  {} Failed to remove {} — it is still in place. Remove it by hand:\n    sudo rm {}",
                "✗".red(),
                limits_path,
                limits_path
            ));
        }
    } else {
        lines.push(format!(
            "  {} {} not found (nothing to remove)",
            "ℹ".dimmed(),
            limits_path
        ));
    }

    lines.push(String::new());
    lines.push(format!(
        "  {} RT kernel package not removed (manual action).",
        "ℹ".dimmed()
    ));
    match rt_kernel(distro, arch) {
        Some(kernel) => lines.push(format!("    To remove: {}", kernel.remove.join(" "))),
        None => {
            lines
                .push("    To remove, uninstall whichever RT kernel this machine has:".to_string());
            for hint in manual_install_hints() {
                lines.push(format!("      {hint}"));
            }
        }
    }
    lines.push(String::new());
    lines.push(format!(
        "  {} CPU isolation not removed (manual action).",
        "ℹ".dimmed()
    ));
    lines.push(
        "    Edit /etc/default/grub, remove isolcpus/nohz_full/rcu_nocbs, run sudo update-grub."
            .to_string(),
    );

    Ok(lines)
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
        result.unwrap();
    }
}

#[cfg(test)]
mod rt_kernel_tests {
    use super::*;

    /// HORUS_ASSUME_YES is process-global, so every test that touches it — or
    /// that depends on `confirm()` declining — takes this lock.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Every distro `setup-rt` will act on without being told to.
    const SUPPORTED: &[&str] = &["ubuntu", "debian", "fedora", "arch"];

    /// A stand-in for `run_privileged` that records instead of executing.
    struct Recorder {
        calls: std::cell::RefCell<Vec<Vec<String>>>,
        succeed: bool,
    }

    impl Recorder {
        fn new(succeed: bool) -> Self {
            Self {
                calls: std::cell::RefCell::new(Vec::new()),
                succeed,
            }
        }
        fn runner(&self) -> impl Fn(&[&str]) -> anyhow::Result<bool> + '_ {
            move |argv: &[&str]| {
                self.calls
                    .borrow_mut()
                    .push(argv.iter().map(|s| s.to_string()).collect());
                Ok(self.succeed)
            }
        }
        fn commands(&self) -> Vec<String> {
            self.calls.borrow().iter().map(|c| c.join(" ")).collect()
        }
    }

    // ── The package table ───────────────────────────────────────────────

    /// Ubuntu has never shipped `linux-image-rt-amd64`. Both distros were
    /// routed to the Debian name, so on Ubuntu the install always failed —
    /// and because the failure was swallowed, `setup-rt` printed
    /// "Setup complete" and told the user to reboot into an RT kernel that was
    /// never installed. Confirmed on Ubuntu 26.04:
    ///
    ///     $ apt-cache policy linux-image-rt-amd64  ->  Candidate: (none)
    ///     $ apt-cache policy linux-image-realtime  ->  Candidate: 7.0.0-29.29.1
    #[test]
    fn ubuntu_uses_its_own_rt_kernel_package() {
        for arch in ["x86_64", "aarch64"] {
            assert_eq!(
                rt_kernel("ubuntu", arch).map(|k| k.packages),
                Some(&["linux-image-realtime"][..])
            );
        }
    }

    /// Debian keeps the arch-suffixed names, which are correct there.
    #[test]
    fn debian_keeps_the_arch_suffixed_names() {
        assert_eq!(
            rt_kernel("debian", "x86_64").map(|k| k.packages),
            Some(&["linux-image-rt-amd64"][..])
        );
        assert_eq!(
            rt_kernel("debian", "aarch64").map(|k| k.packages),
            Some(&["linux-image-rt-arm64"][..])
        );
    }

    /// Ubuntu and Debian must never resolve to the same package: that
    /// conflation is the whole bug.
    #[test]
    fn ubuntu_and_debian_do_not_share_a_package() {
        assert_ne!(
            rt_kernel("ubuntu", "x86_64").map(|k| k.packages),
            rt_kernel("debian", "x86_64").map(|k| k.packages),
            "routing Ubuntu to the Debian package is what made setup-rt lie"
        );
    }

    /// Fedora and Arch are live install paths, so the table has to know them.
    /// It used to know only Ubuntu and Debian while `install_rt_fedora` and
    /// `install_rt_arch` carried their own package names inline — which is how
    /// the guard test came to assert `("arch", "x86_64") == None` about a
    /// distro `setup-rt` installs a kernel on.
    #[test]
    fn every_distro_setup_rt_acts_on_is_in_the_table() {
        for distro in SUPPORTED {
            let kernel = rt_kernel(distro, "x86_64")
                .unwrap_or_else(|| panic!("{distro} is installed on but not in RT_KERNELS"));
            assert!(!kernel.packages.is_empty(), "{distro} has no packages");
        }
    }

    /// An unknown distro or architecture must be reported, not silently
    /// installed onto with someone else's package manager.
    #[test]
    fn unknown_targets_have_no_package() {
        assert!(rt_kernel("ubuntu", "riscv64").is_none());
        assert!(rt_kernel("debian", "mips").is_none());
        assert!(rt_kernel("gentoo", "x86_64").is_none());
        assert!(rt_kernel("unknown", "x86_64").is_none());
    }

    /// Install and remove must name the same packages, or `--undo` tells the
    /// user to uninstall something that was never installed. That is exactly
    /// how `--undo` came to print `sudo apt remove linux-image-rt-*` on a
    /// machine that had `linux-image-realtime`.
    #[test]
    fn install_and_remove_agree_on_packages() {
        for kernel in RT_KERNELS {
            for pkg in kernel.packages {
                assert!(
                    kernel.install.contains(pkg),
                    "{}'s install command does not mention {pkg}",
                    kernel.distro
                );
                assert!(
                    kernel.remove.contains(pkg),
                    "{}'s remove command does not mention {pkg}",
                    kernel.distro
                );
            }
            assert_eq!(kernel.install.first(), Some(&"sudo"));
            assert_eq!(kernel.remove.first(), Some(&"sudo"));
        }
    }

    // ── The prompt, at the call sites that matter ───────────────────────

    /// The documentation says `setup-rt` asks before installing a kernel. Two
    /// of the four paths never did: `install_rt_fedora` ran
    /// `sudo dnf install -y kernel-rt kernel-rt-core` and `install_rt_arch` ran
    /// `sudo pacman -S --noconfirm linux-rt linux-rt-headers` with no prompt
    /// anywhere. `confirm()` was called exactly once in the file, inside the
    /// apt path — so the existing "cannot silently install a kernel" test
    /// guarded a helper that half the install paths did not use.
    #[test]
    fn no_distro_installs_a_kernel_without_asking() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::remove_var("HORUS_ASSUME_YES");
        // The test harness runs without a terminal on stdin, so `confirm`
        // declines — the scripted-provisioning case.
        for distro in SUPPORTED {
            let rec = Recorder::new(true);
            let run = rec.runner();
            let err = install_rt_kernel(distro, "x86_64", &run)
                .expect_err("a declined install must not report success");
            assert!(err.to_string().contains("declined"), "{distro}: {err}");
            assert!(
                rec.commands().is_empty(),
                "{distro} ran {:?} without asking",
                rec.commands()
            );
        }
    }

    /// A provisioning script opts in explicitly, and then gets the package
    /// manager its own distro uses.
    #[test]
    fn each_distro_installs_its_own_packages() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::set_var("HORUS_ASSUME_YES", "1");
        let expected = [
            ("ubuntu", "sudo apt install -y linux-image-realtime"),
            ("debian", "sudo apt install -y linux-image-rt-amd64"),
            ("fedora", "sudo dnf install -y kernel-rt kernel-rt-core"),
            (
                "arch",
                "sudo pacman -S --noconfirm linux-rt linux-rt-headers",
            ),
        ];
        let mut got = Vec::new();
        for (distro, _) in expected {
            let rec = Recorder::new(true);
            let run = rec.runner();
            let result = install_rt_kernel(distro, "x86_64", &run);
            got.push((distro, result.is_ok(), rec.commands()));
        }
        std::env::remove_var("HORUS_ASSUME_YES");

        for ((distro, want), (_, ok, commands)) in expected.iter().zip(got) {
            assert!(ok, "{distro} install failed");
            assert_eq!(commands, vec![want.to_string()], "{distro}");
        }
    }

    /// A failed install must propagate. Fedora and Arch printed "✗ Failed" and
    /// returned `Ok`, which put `run_setup_rt` straight back onto
    /// "Setup complete. Reboot to activate the RT kernel" — the same false RT
    /// guarantee the Ubuntu package fix was written to remove.
    #[test]
    fn a_failed_install_is_never_reported_as_success() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::set_var("HORUS_ASSUME_YES", "1");
        let mut results = Vec::new();
        for distro in SUPPORTED {
            let rec = Recorder::new(false); // package manager exits non-zero
            let run = rec.runner();
            results.push((distro, install_rt_kernel(distro, "x86_64", &run)));
        }
        std::env::remove_var("HORUS_ASSUME_YES");

        for (distro, result) in results {
            let err = result
                .err()
                .unwrap_or_else(|| panic!("{distro} swallowed a failed kernel install"));
            assert!(
                err.to_string().contains("Failed to install"),
                "{distro}: {err}"
            );
        }
    }

    // ── The limits file ────────────────────────────────────────────────

    /// `ensure_memlock_limits` went straight to `sudo tee`, so the only prompt
    /// in the command was the kernel one. This file grants rtprio 99 and
    /// unlimited memlock through sudo; the docs say `setup-rt` asks first.
    #[test]
    fn the_limits_file_is_not_written_without_asking() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::remove_var("HORUS_ASSUME_YES");
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("99-horus-rt.conf");
        let written = std::cell::RefCell::new(Vec::new());

        ensure_memlock_limits_at(path.to_str().unwrap(), "alice", &|p, c| {
            written.borrow_mut().push((p.to_string(), c.to_string()));
            Ok(true)
        })
        .unwrap();

        assert!(
            written.borrow().is_empty(),
            "wrote the limits file without a prompt: {:?}",
            written.borrow()
        );
        assert!(!path.exists());
    }

    /// With consent it writes, scoped to the invoking user and never to `*`.
    #[test]
    fn the_limits_file_is_scoped_to_the_invoking_user() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::set_var("HORUS_ASSUME_YES", "1");
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("99-horus-rt.conf");
        let written = std::cell::RefCell::new(Vec::new());

        let result = ensure_memlock_limits_at(path.to_str().unwrap(), "alice", &|p, c| {
            written.borrow_mut().push((p.to_string(), c.to_string()));
            Ok(true)
        });
        std::env::remove_var("HORUS_ASSUME_YES");
        result.unwrap();

        let calls = written.borrow();
        assert_eq!(calls.len(), 1, "expected exactly one write");
        assert_eq!(calls[0].0, path.to_str().unwrap());
        assert!(
            calls[0].1.contains("alice - memlock unlimited"),
            "{}",
            calls[0].1
        );
        assert!(calls[0].1.contains("alice - rtprio 99"), "{}", calls[0].1);
        assert!(
            !calls[0].1.lines().any(|l| l.starts_with('*')),
            "must not grant rtprio 99 to every account: {}",
            calls[0].1
        );
    }

    /// An unusable SUDO_USER/USER must not become a limits.conf domain.
    #[test]
    fn an_unidentifiable_user_gets_no_limits_file() {
        for user in ["", "root", "someone; rm -rf /", "a".repeat(33).as_str()] {
            assert!(
                memlock_limits_content(user).is_none(),
                "{user:?} should not be written into limits.conf"
            );
        }
        assert!(memlock_limits_content("robot-1").is_some());
    }

    // ── --undo ─────────────────────────────────────────────────────────

    /// `--undo` printed `sudo apt remove linux-image-rt-*` on every distro. On
    /// Ubuntu the installed package is `linux-image-realtime`, which that glob
    /// does not match; on Fedora and Arch apt does not exist at all. The
    /// package-name fix reached install and not undo.
    #[test]
    fn undo_names_the_package_this_distro_actually_installed() {
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("not-there.conf");
        let expected = [
            ("ubuntu", "sudo apt remove linux-image-realtime"),
            ("debian", "sudo apt remove linux-image-rt-amd64"),
            ("fedora", "sudo dnf remove kernel-rt kernel-rt-core"),
            ("arch", "sudo pacman -R linux-rt linux-rt-headers"),
        ];
        for (distro, want) in expected {
            let rec = Recorder::new(true);
            let run = rec.runner();
            let text = undo_report(missing.to_str().unwrap(), distro, "x86_64", &run)
                .unwrap()
                .join("\n");
            assert!(
                text.contains(want),
                "{distro} undo should say `{want}`:\n{text}"
            );
            assert!(
                !text.contains("linux-image-rt-*"),
                "{distro} undo still prints the glob that matches nothing:\n{text}"
            );
        }
    }

    /// A failed `rm` must not print "Removed". The old code ran
    /// `let _ = Command::new("sudo").args(["rm", limits_path]).status()` and
    /// then printed the tick unconditionally, so a machine that still had
    /// rtprio 99 granted was told it had been cleaned up.
    #[test]
    fn undo_does_not_claim_success_when_the_remove_fails() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("99-horus-rt.conf");
        std::fs::write(&path, "robot - rtprio 99\n").unwrap();

        let rec = Recorder::new(false); // rm exits non-zero
        let run = rec.runner();
        let lines = undo_report(path.to_str().unwrap(), "ubuntu", "x86_64", &run).unwrap();
        let text = lines.join("\n");

        assert_eq!(
            rec.commands(),
            vec![format!("sudo rm {}", path.to_str().unwrap())]
        );
        assert!(text.contains("Failed to remove"), "{text}");
        assert!(
            !text.contains("Removed"),
            "reported success for a failed rm:\n{text}"
        );
    }

    #[test]
    fn undo_reports_success_when_the_remove_works() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("99-horus-rt.conf");
        std::fs::write(&path, "robot - rtprio 99\n").unwrap();

        let rec = Recorder::new(true);
        let run = rec.runner();
        let text = undo_report(path.to_str().unwrap(), "ubuntu", "x86_64", &run)
            .unwrap()
            .join("\n");
        assert!(text.contains("Removed"), "{text}");
        assert!(!text.contains("Failed"), "{text}");
    }

    /// On a distro HORUS has no entry for, `--undo` cannot name a package, so
    /// it must list the alternatives instead of inventing an apt command.
    #[test]
    fn undo_on_an_unknown_distro_lists_the_alternatives() {
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("not-there.conf");
        let rec = Recorder::new(true);
        let run = rec.runner();
        let text = undo_report(missing.to_str().unwrap(), "gentoo", "x86_64", &run)
            .unwrap()
            .join("\n");
        assert!(text.contains("Ubuntu"), "{text}");
        assert!(text.contains("Fedora"), "{text}");
        assert!(!text.contains("linux-image-rt-*"), "{text}");
    }

    /// The manual-instruction list an unsupported distro is shown has to come
    /// from the same table the installs use.
    #[test]
    fn manual_hints_cover_every_known_distro() {
        let hints = manual_install_hints().join("\n");
        for label in ["Ubuntu", "Debian", "Fedora", "Arch"] {
            assert!(hints.contains(label), "{label} missing from:\n{hints}");
        }
        for pkg in [
            "linux-image-realtime",
            "linux-image-rt-amd64",
            "kernel-rt",
            "linux-rt",
        ] {
            assert!(hints.contains(pkg), "{pkg} missing from:\n{hints}");
        }
    }

    // ── confirm() itself ───────────────────────────────────────────────

    /// Without a TTY the prompt must decline rather than proceeding, so a
    /// scripted run cannot silently install a kernel.
    #[test]
    fn confirm_declines_when_not_a_terminal() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // The test harness runs without a controlling terminal on stdin.
        std::env::remove_var("HORUS_ASSUME_YES");
        assert!(!confirm("install something drastic").unwrap());
    }

    /// Provisioning scripts opt in explicitly.
    #[test]
    fn confirm_accepts_with_assume_yes() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::set_var("HORUS_ASSUME_YES", "1");
        let got = confirm("install something drastic").unwrap();
        std::env::remove_var("HORUS_ASSUME_YES");
        assert!(got);
    }
}
