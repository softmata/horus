//! OS detection, standard directories, shell integration, package management.
//!
//! The most cross-cutting module — provides:
//! - [`detect()`]: OS/distro/arch detection (cached via OnceLock)
//! - [`config_dir()`], [`cache_dir()`], [`data_dir()`], [`temp_dir()`]: Platform-standard paths
//! - [`default_shell()`]: User's shell (`$SHELL` / `powershell.exe`)
//! - [`suggest_install()`], [`suggest_dev_install()`]: Distro-aware package manager commands
//! - [`init_terminal_colors()`]: Windows ANSI color initialization
//! - [`set_hidden()`]: Hidden file/directory attribute
//! - [`write_script()`]: Script file with correct shebang / .cmd wrapper

use std::path::PathBuf;
use std::sync::OnceLock;

// ── Platform backends ───────────────────────────────────────────────────────

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "windows")]
mod windows;

// ── Types ───────────────────────────────────────────────────────────────────

/// Detected operating system information.
#[derive(Debug, Clone)]
pub struct OsInfo {
    pub os: Os,
    pub distro: Distro,
    pub arch: Arch,
    pub version: String,
    pub kernel: String,
}

/// Operating system family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Os {
    Linux,
    MacOS,
    Windows,
}

/// Linux distribution or OS variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Distro {
    // Linux
    Ubuntu,
    Debian,
    Fedora,
    Arch,
    NixOS,
    Alpine,
    // Non-Linux
    MacOS,
    Windows,
    Unknown,
}

/// CPU architecture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arch {
    X86_64,
    Aarch64,
    Armv7,
    Riscv64,
    Unknown,
}

// ── OS Detection ────────────────────────────────────────────────────────────

/// Detect the current OS, distro, and architecture. Cached — only runs once.
pub fn detect() -> &'static OsInfo {
    static INFO: OnceLock<OsInfo> = OnceLock::new();
    INFO.get_or_init(|| OsInfo {
        os: detect_os(),
        distro: detect_distro(),
        arch: detect_arch(),
        version: detect_version(),
        kernel: detect_kernel(),
    })
}

fn detect_os() -> Os {
    if cfg!(target_os = "linux") {
        Os::Linux
    } else if cfg!(target_os = "macos") {
        Os::MacOS
    } else if cfg!(target_os = "windows") {
        Os::Windows
    } else {
        Os::Linux // default for other Unix
    }
}

/// Detect Linux distro from /etc/os-release, or return MacOS/Windows.
pub fn detect_distro() -> Distro {
    static DISTRO: OnceLock<Distro> = OnceLock::new();
    *DISTRO.get_or_init(|| {
        if cfg!(target_os = "macos") {
            return Distro::MacOS;
        }
        if cfg!(target_os = "windows") {
            return Distro::Windows;
        }
        if let Ok(content) = std::fs::read_to_string("/etc/os-release") {
            for line in content.lines() {
                if let Some(id) = line.strip_prefix("ID=") {
                    let id = id.trim_matches('"').to_lowercase();
                    return match id.as_str() {
                        "ubuntu" | "linuxmint" | "pop" | "raspbian" => Distro::Ubuntu,
                        "debian" => Distro::Debian,
                        "fedora" | "rhel" | "centos" | "rocky" | "alma" => Distro::Fedora,
                        "arch" | "manjaro" | "endeavouros" => Distro::Arch,
                        "nixos" => Distro::NixOS,
                        "alpine" => Distro::Alpine,
                        _ => Distro::Unknown,
                    };
                }
            }
        }
        Distro::Unknown
    })
}

fn detect_arch() -> Arch {
    if cfg!(target_arch = "x86_64") {
        Arch::X86_64
    } else if cfg!(target_arch = "aarch64") {
        Arch::Aarch64
    } else if cfg!(target_arch = "arm") {
        Arch::Armv7
    } else if cfg!(target_arch = "riscv64") {
        Arch::Riscv64
    } else {
        Arch::Unknown
    }
}

fn detect_version() -> String {
    #[cfg(target_os = "linux")]
    {
        if let Ok(content) = std::fs::read_to_string("/etc/os-release") {
            for line in content.lines() {
                if let Some(ver) = line.strip_prefix("VERSION_ID=") {
                    return ver.trim_matches('"').to_string();
                }
            }
        }
    }
    String::new()
}

fn detect_kernel() -> String {
    #[cfg(target_os = "linux")]
    {
        if let Ok(content) = std::fs::read_to_string("/proc/version") {
            if let Some(first_line) = content.lines().next() {
                return first_line.to_string();
            }
        }
    }
    String::new()
}

// ── Standard Directories ────────────────────────────────────────────────────

/// Global horus config directory.
/// - Linux: `$XDG_CONFIG_HOME/horus` or `~/.config/horus`
/// - macOS: `~/Library/Application Support/horus`
/// - Windows: `%APPDATA%\horus`
pub fn config_dir() -> PathBuf {
    if let Ok(xdg) = std::env::var("XDG_CONFIG_HOME") {
        return PathBuf::from(xdg).join("horus");
    }
    #[cfg(target_os = "macos")]
    {
        if let Some(home) = dirs::home_dir() {
            return home.join("Library/Application Support/horus");
        }
    }
    #[cfg(target_os = "windows")]
    {
        if let Some(appdata) = dirs::config_dir() {
            return appdata.join("horus");
        }
    }
    // Linux default + fallback for all
    dirs::home_dir()
        .unwrap_or_else(|| PathBuf::from("/tmp"))
        .join(".config/horus")
}

/// Global package cache directory.
/// - Linux: `$XDG_CACHE_HOME/horus` or `~/.cache/horus`
/// - macOS: `~/Library/Caches/horus`
/// - Windows: `%LOCALAPPDATA%\horus\cache`
pub fn cache_dir() -> PathBuf {
    if let Ok(xdg) = std::env::var("XDG_CACHE_HOME") {
        return PathBuf::from(xdg).join("horus");
    }
    #[cfg(target_os = "macos")]
    {
        if let Some(home) = dirs::home_dir() {
            return home.join("Library/Caches/horus");
        }
    }
    #[cfg(target_os = "windows")]
    {
        if let Some(local) = dirs::cache_dir() {
            return local.join("horus");
        }
    }
    dirs::home_dir()
        .unwrap_or_else(|| PathBuf::from("/tmp"))
        .join(".cache/horus")
}

/// Global data directory (recordings, blackbox, keys).
/// - Linux: `$XDG_DATA_HOME/horus` or `~/.local/share/horus`
/// - macOS: `~/Library/Application Support/horus`
/// - Windows: `%LOCALAPPDATA%\horus\data`
pub fn data_dir() -> PathBuf {
    if let Ok(xdg) = std::env::var("XDG_DATA_HOME") {
        return PathBuf::from(xdg).join("horus");
    }
    #[cfg(target_os = "macos")]
    {
        if let Some(home) = dirs::home_dir() {
            return home.join("Library/Application Support/horus");
        }
    }
    #[cfg(target_os = "windows")]
    {
        if let Some(local) = dirs::data_local_dir() {
            return local.join("horus/data");
        }
    }
    dirs::home_dir()
        .unwrap_or_else(|| PathBuf::from("/tmp"))
        .join(".local/share/horus")
}

/// Temporary directory for build artifacts and lockfiles.
pub fn temp_dir() -> PathBuf {
    std::env::temp_dir().join("horus")
}

// ── Shell Integration ───────────────────────────────────────────────────────

/// Get the user's default shell command.
/// - Linux/macOS: `$SHELL` or `/bin/bash` (Linux) / `/bin/zsh` (macOS)
/// - Windows: `%COMSPEC%` or `powershell.exe`
pub fn default_shell() -> PathBuf {
    #[cfg(unix)]
    {
        if let Ok(shell) = std::env::var("SHELL") {
            return PathBuf::from(shell);
        }
        #[cfg(target_os = "macos")]
        return PathBuf::from("/bin/zsh");
        #[cfg(not(target_os = "macos"))]
        return PathBuf::from("/bin/bash");
    }
    #[cfg(windows)]
    {
        if let Ok(comspec) = std::env::var("COMSPEC") {
            return PathBuf::from(comspec);
        }
        PathBuf::from("powershell.exe")
    }
    #[cfg(not(any(unix, windows)))]
    PathBuf::from("/bin/sh")
}

/// Create a script file with the correct format for the current platform.
/// - Unix: writes shebang + content, chmod +x
/// - Windows: writes .cmd wrapper
pub fn write_script(
    path: &std::path::Path,
    interpreter: &str,
    content: &str,
) -> anyhow::Result<()> {
    use std::io::Write;
    // Validate interpreter: must be a simple name or path, no arguments or shell metacharacters
    anyhow::ensure!(
        !interpreter.is_empty()
            && !interpreter.contains([';', '&', '|', '`', '$', '\n']),
        "Interpreter '{}' contains invalid characters",
        interpreter
    );

    #[cfg(unix)]
    {
        let mut file = std::fs::File::create(path)?;
        writeln!(file, "#!/usr/bin/env {}", interpreter)?;
        write!(file, "{}", content)?;
        // chmod +x
        use std::os::unix::fs::PermissionsExt;
        let perms = std::fs::Permissions::from_mode(0o755);
        std::fs::set_permissions(path, perms)?;
    }

    #[cfg(windows)]
    {
        // Write a .cmd wrapper that invokes the interpreter
        let cmd_path = path.with_extension("cmd");
        let mut file = std::fs::File::create(&cmd_path)?;
        writeln!(file, "@echo off")?;
        writeln!(file, "{} \"{}\" %*", interpreter, path.display())?;
        // Also write the actual script content
        let mut script_file = std::fs::File::create(path)?;
        write!(script_file, "{}", content)?;
    }

    #[cfg(not(any(unix, windows)))]
    {
        let mut file = std::fs::File::create(path)?;
        writeln!(file, "#!/usr/bin/env {}", interpreter)?;
        write!(file, "{}", content)?;
    }

    Ok(())
}

// ── Terminal Colors ─────────────────────────────────────────────────────────

/// Initialize terminal for ANSI color support.
/// - Linux/macOS: no-op (ANSI supported natively)
/// - Windows: `SetConsoleMode(ENABLE_VIRTUAL_TERMINAL_PROCESSING)`
pub fn init_terminal_colors() -> anyhow::Result<()> {
    #[cfg(windows)]
    {
        use windows_sys::Win32::System::Console::{
            GetConsoleMode, GetStdHandle, SetConsoleMode, ENABLE_VIRTUAL_TERMINAL_PROCESSING,
            STD_OUTPUT_HANDLE,
        };
        unsafe {
            let handle = GetStdHandle(STD_OUTPUT_HANDLE);
            let mut mode = 0;
            if GetConsoleMode(handle, &mut mode) != 0 {
                SetConsoleMode(handle, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
            }
        }
    }
    Ok(())
}

// ── Hidden Files ────────────────────────────────────────────────────────────

/// Mark a directory/file as hidden.
/// - Unix: no-op (dot-prefix hides it)
/// - Windows: `attrib +H`
pub fn set_hidden(path: &std::path::Path) -> anyhow::Result<()> {
    #[cfg(windows)]
    {
        use std::os::windows::ffi::OsStrExt;
        extern "system" {
            fn GetFileAttributesW(path: *const u16) -> u32;
            fn SetFileAttributesW(path: *const u16, attrs: u32) -> i32;
        }
        const FILE_ATTRIBUTE_HIDDEN: u32 = 0x2;
        const INVALID_FILE_ATTRIBUTES: u32 = 0xFFFF_FFFF;
        let wide: Vec<u16> = path.as_os_str().encode_wide().chain(Some(0)).collect();
        // SAFETY: wide is a valid null-terminated UTF-16 path
        let attrs = unsafe { GetFileAttributesW(wide.as_ptr()) };
        if attrs != INVALID_FILE_ATTRIBUTES {
            unsafe { SetFileAttributesW(wide.as_ptr(), attrs | FILE_ATTRIBUTE_HIDDEN) };
        }
    }
    let _ = path;
    Ok(())
}

// ── Package Management ──────────────────────────────────────────────────────

/// Suggest a package install command for the detected distro.
pub fn suggest_install(pkg: &str) -> String {
    match detect_distro() {
        Distro::Ubuntu | Distro::Debian | Distro::Unknown => {
            format!("sudo apt install {}", pkg)
        }
        Distro::Fedora => format!("sudo dnf install {}", pkg),
        Distro::Arch => format!("sudo pacman -S {}", pkg),
        Distro::NixOS => format!("nix-env -iA nixpkgs.{}", pkg),
        Distro::Alpine => format!("sudo apk add {}", pkg),
        Distro::MacOS => format!("brew install {}", pkg),
        Distro::Windows => format!("winget install {}", pkg),
    }
}

/// Suggest a development library install command.
/// Handles -dev (Debian) vs -devel (Fedora) vs plain (Arch/macOS) naming.
pub fn suggest_dev_install(lib_name: &str) -> String {
    match detect_distro() {
        Distro::Ubuntu | Distro::Debian | Distro::Unknown => {
            suggest_install(&format!("lib{}-dev", lib_name))
        }
        Distro::Fedora => suggest_install(&format!("{}-devel", lib_name)),
        Distro::Arch | Distro::NixOS | Distro::Alpine | Distro::MacOS | Distro::Windows => {
            suggest_install(lib_name)
        }
    }
}

/// Suggest a build-essential / base-devel install command.
pub fn suggest_build_tools() -> String {
    match detect_distro() {
        Distro::Ubuntu | Distro::Debian | Distro::Unknown => suggest_install("build-essential"),
        Distro::Fedora => "sudo dnf groupinstall \"Development Tools\"".to_string(),
        Distro::Arch => suggest_install("base-devel"),
        Distro::NixOS => "nix-env -iA nixpkgs.gcc nixpkgs.gnumake".to_string(),
        Distro::Alpine => suggest_install("build-base"),
        Distro::MacOS => "xcode-select --install".to_string(),
        Distro::Windows => {
            "Install Visual Studio Build Tools: https://visualstudio.microsoft.com/downloads/"
                .to_string()
        }
    }
}

// ── Hostname ────────────────────────────────────────────────────────────

/// Get the local hostname.
///
/// - **Unix**: `gethostname()` syscall
/// - **Windows**: `GetComputerNameW`
///
/// Falls back to `"localhost"` on failure.
pub fn hostname() -> String {
    #[cfg(unix)]
    {
        let mut buf = [0u8; 256];
        // SAFETY: buf is a valid 256-byte buffer; gethostname writes null-terminated.
        let ret = unsafe { libc::gethostname(buf.as_mut_ptr() as *mut libc::c_char, buf.len()) };
        if ret != 0 {
            return "localhost".to_string();
        }
        buf[buf.len() - 1] = 0; // ensure null termination
                                // SAFETY: we just ensured null-termination above.
        let cstr = unsafe { std::ffi::CStr::from_ptr(buf.as_ptr() as *const libc::c_char) };
        cstr.to_str().unwrap_or("localhost").to_string()
    }
    #[cfg(windows)]
    {
        extern "system" {
            fn GetComputerNameW(buf: *mut u16, size: *mut u32) -> i32;
        }
        let mut buf = [0u16; 256];
        let mut size = buf.len() as u32;
        // SAFETY: buf is properly sized; size is in/out.
        let ret = unsafe { GetComputerNameW(buf.as_mut_ptr(), &mut size) };
        if ret != 0 && size > 0 {
            String::from_utf16_lossy(&buf[..size as usize])
        } else {
            "localhost".to_string()
        }
    }
    #[cfg(not(any(unix, windows)))]
    {
        "localhost".to_string()
    }
}

// ── Display Impls ───────────────────────────────────────────────────────────

impl std::fmt::Display for Os {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Os::Linux => write!(f, "linux"),
            Os::MacOS => write!(f, "macos"),
            Os::Windows => write!(f, "windows"),
        }
    }
}

impl std::fmt::Display for Distro {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Distro::Ubuntu => write!(f, "ubuntu"),
            Distro::Debian => write!(f, "debian"),
            Distro::Fedora => write!(f, "fedora"),
            Distro::Arch => write!(f, "arch"),
            Distro::NixOS => write!(f, "nixos"),
            Distro::Alpine => write!(f, "alpine"),
            Distro::MacOS => write!(f, "macos"),
            Distro::Windows => write!(f, "windows"),
            Distro::Unknown => write!(f, "unknown"),
        }
    }
}

impl std::fmt::Display for Arch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Arch::X86_64 => write!(f, "x86_64"),
            Arch::Aarch64 => write!(f, "aarch64"),
            Arch::Armv7 => write!(f, "armv7"),
            Arch::Riscv64 => write!(f, "riscv64"),
            Arch::Unknown => write!(f, "unknown"),
        }
    }
}

// ── Disk Space ──────────────────────────────────────────────────────────────

/// Get available disk space in megabytes for the filesystem containing `path`.
///
/// - **Unix**: `statvfs()` → `f_bavail * f_frsize`
/// - **Windows**: `GetDiskFreeSpaceExW()`
///
/// Returns `None` if the query fails (e.g., path doesn't exist).
pub fn disk_available_mb(path: &std::path::Path) -> Option<u64> {
    #[cfg(unix)]
    {
        let c_path = std::ffi::CString::new(path.to_str()?).ok()?;
        // SAFETY: c_path is a valid null-terminated string; stat is zeroed POD struct
        let mut stat: libc::statvfs = unsafe { std::mem::zeroed() };
        if unsafe { libc::statvfs(c_path.as_ptr(), &mut stat) } == 0 {
            stat.f_bavail
                .checked_mul(stat.f_frsize)
                .map(|bytes| bytes / (1024 * 1024))
        } else {
            None
        }
    }
    #[cfg(windows)]
    {
        use std::os::windows::ffi::OsStrExt;
        extern "system" {
            fn GetDiskFreeSpaceExW(
                dir: *const u16,
                caller_free: *mut u64,
                total: *mut u64,
                total_free: *mut u64,
            ) -> i32;
        }
        let wide: Vec<u16> = path.as_os_str().encode_wide().chain(Some(0)).collect();
        let mut free_bytes: u64 = 0;
        // SAFETY: wide is a valid null-terminated UTF-16 string; free_bytes is a valid pointer
        if unsafe {
            GetDiskFreeSpaceExW(
                wide.as_ptr(),
                &mut free_bytes,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        } != 0
        {
            Some(free_bytes / (1024 * 1024))
        } else {
            None
        }
    }
    #[cfg(not(any(unix, windows)))]
    {
        let _ = path;
        None
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_returns_valid_os() {
        let info = detect();
        assert!(matches!(info.os, Os::Linux | Os::MacOS | Os::Windows));
    }

    #[test]
    fn detect_returns_valid_arch() {
        let info = detect();
        assert!(matches!(
            info.arch,
            Arch::X86_64 | Arch::Aarch64 | Arch::Armv7 | Arch::Riscv64 | Arch::Unknown
        ));
    }

    #[test]
    fn detect_is_cached() {
        let a = detect() as *const OsInfo;
        let b = detect() as *const OsInfo;
        assert_eq!(
            a, b,
            "detect() should return same pointer (OnceLock cached)"
        );
    }

    #[test]
    fn config_dir_is_absolute() {
        let path = config_dir();
        assert!(
            path.is_absolute(),
            "config_dir should be absolute: {:?}",
            path
        );
    }

    #[test]
    fn cache_dir_is_absolute() {
        let path = cache_dir();
        assert!(
            path.is_absolute(),
            "cache_dir should be absolute: {:?}",
            path
        );
    }

    #[test]
    fn data_dir_is_absolute() {
        let path = data_dir();
        assert!(
            path.is_absolute(),
            "data_dir should be absolute: {:?}",
            path
        );
    }

    #[test]
    fn temp_dir_contains_horus() {
        let path = temp_dir();
        assert!(
            path.to_string_lossy().contains("horus"),
            "temp_dir should contain 'horus': {:?}",
            path
        );
    }

    #[test]
    fn default_shell_is_nonempty() {
        let shell = default_shell();
        assert!(
            !shell.to_string_lossy().is_empty(),
            "default_shell should not be empty"
        );
    }

    #[test]
    fn suggest_install_contains_package() {
        let cmd = suggest_install("cmake");
        assert!(
            cmd.contains("cmake"),
            "suggest_install should contain package name"
        );
        assert!(!cmd.is_empty());
    }

    #[test]
    fn suggest_dev_install_contains_lib() {
        let cmd = suggest_dev_install("ssl");
        assert!(
            cmd.contains("ssl"),
            "suggest_dev_install should contain lib name"
        );
    }

    #[test]
    fn suggest_build_tools_nonempty() {
        let cmd = suggest_build_tools();
        assert!(!cmd.is_empty());
    }

    #[test]
    fn init_terminal_colors_does_not_panic() {
        init_terminal_colors().ok();
    }

    #[test]
    fn set_hidden_does_not_panic() {
        let tmp = std::env::temp_dir().join("horus_sys_test_hidden");
        std::fs::create_dir_all(&tmp).ok();
        set_hidden(&tmp).ok();
        std::fs::remove_dir_all(&tmp).ok();
    }

    #[test]
    fn distro_display() {
        assert_eq!(format!("{}", Distro::Ubuntu), "ubuntu");
        assert_eq!(format!("{}", Distro::MacOS), "macos");
        assert_eq!(format!("{}", Distro::Windows), "windows");
    }

    #[test]
    fn os_display() {
        assert_eq!(format!("{}", Os::Linux), "linux");
    }

    #[test]
    fn arch_display() {
        let info = detect();
        let s = format!("{}", info.arch);
        assert!(!s.is_empty());
    }

    #[test]
    fn write_script_creates_file() {
        let tmp = std::env::temp_dir().join("horus_sys_test_script.sh");
        write_script(&tmp, "bash", "echo hello\n").unwrap();
        assert!(tmp.exists());
        let content = std::fs::read_to_string(&tmp).unwrap();
        assert!(content.contains("echo hello"));
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let perms = std::fs::metadata(&tmp).unwrap().permissions();
            assert!(perms.mode() & 0o111 != 0, "script should be executable");
        }
        std::fs::remove_file(&tmp).ok();
    }

    // ── disk_available_mb tests ─────────────────────────────────────

    #[test]
    fn disk_available_mb_returns_some_for_temp_dir() {
        let result = disk_available_mb(&std::env::temp_dir());
        assert!(
            result.is_some(),
            "temp_dir should have measurable disk space"
        );
        assert!(result.unwrap() > 0, "available space should be positive");
    }

    #[test]
    fn disk_available_mb_returns_none_for_nonexistent() {
        let result = disk_available_mb(std::path::Path::new("/nonexistent/path/xyz"));
        assert!(result.is_none(), "nonexistent path should return None");
    }

    #[test]
    fn disk_available_mb_reasonable_range() {
        if let Some(mb) = disk_available_mb(&std::env::temp_dir()) {
            // Should be between 1MB and 100TB
            assert!(mb >= 1, "available space should be >= 1MB");
            assert!(mb < 100_000_000, "available space should be < 100TB");
        }
    }

    // ── hostname tests ──────────────────────────────────────────────

    #[test]
    fn hostname_is_nonempty() {
        let h = hostname();
        assert!(!h.is_empty(), "hostname should not be empty");
    }

    #[test]
    fn hostname_is_stable() {
        let h1 = hostname();
        let h2 = hostname();
        assert_eq!(h1, h2, "hostname should be stable across calls");
    }

    // ── OsInfo intent tests ─────────────────────────────────────────

    #[test]
    fn detect_returns_consistent_os() {
        let info = detect();
        match info.os {
            Os::Linux => assert!(cfg!(target_os = "linux")),
            Os::MacOS => assert!(cfg!(target_os = "macos")),
            Os::Windows => assert!(cfg!(target_os = "windows")),
        }
    }

    #[test]
    fn detect_returns_consistent_arch() {
        let info = detect();
        match info.arch {
            Arch::X86_64 => assert!(cfg!(target_arch = "x86_64")),
            Arch::Aarch64 => assert!(cfg!(target_arch = "aarch64")),
            Arch::Armv7 => assert!(cfg!(target_arch = "arm")),
            _ => {} // Unknown is acceptable on exotic architectures
        }
    }

    // ── Directory path intent tests ─────────────────────────────────

    #[test]
    fn all_standard_dirs_are_distinct() {
        let config = config_dir();
        let cache = cache_dir();
        let data = data_dir();
        let temp = temp_dir();
        // config, cache, data should be different paths
        assert_ne!(config, cache, "config and cache dirs should differ");
        assert_ne!(config, temp, "config and temp dirs should differ");
        assert_ne!(cache, temp, "cache and temp dirs should differ");
    }

    #[test]
    fn all_standard_dirs_contain_horus() {
        assert!(config_dir().to_string_lossy().contains("horus"));
        assert!(cache_dir().to_string_lossy().contains("horus"));
        assert!(data_dir().to_string_lossy().contains("horus"));
        assert!(temp_dir().to_string_lossy().contains("horus"));
    }
}
