//! Platform detection for reproducible benchmarks.
//!
//! Captures hardware and OS information to contextualize benchmark results:
//! - CPU model, cores, cache sizes
//! - Memory configuration
//! - OS version and kernel
//! - CPU frequency (measured, not just reported)

use serde::{Deserialize, Serialize};
use std::fs;

/// CPU information for benchmark context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpuInfo {
    /// CPU model name (e.g., "AMD Ryzen 9 5900X")
    pub model: String,
    /// Number of physical cores
    pub physical_cores: usize,
    /// Number of logical cores (with hyperthreading)
    pub logical_cores: usize,
    /// L1 data cache size per core (KB)
    pub l1d_cache_kb: Option<usize>,
    /// L2 cache size per core (KB)
    pub l2_cache_kb: Option<usize>,
    /// L3 cache size total (KB)
    pub l3_cache_kb: Option<usize>,
    /// Base frequency (MHz) as reported
    pub base_freq_mhz: Option<u32>,
    /// Measured frequency during calibration (MHz)
    pub measured_freq_mhz: Option<f64>,
    /// CPU features relevant to benchmarking
    pub features: Vec<String>,
}

/// Complete platform information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlatformInfo {
    /// CPU information
    pub cpu: CpuInfo,
    /// Total system memory (MB)
    pub memory_mb: usize,
    /// Operating system
    pub os: String,
    /// Kernel version (Linux) or OS build (Windows/macOS)
    pub kernel: String,
    /// Architecture (x86_64, aarch64, etc.)
    pub arch: String,
    /// Hostname (for identifying machines in CI)
    pub hostname: String,
    /// Whether running in a VM/container
    pub virtualized: bool,
    /// NUMA node count (1 = UMA)
    pub numa_nodes: usize,
    /// CPU governor setting (Linux)
    pub cpu_governor: Option<String>,
}

/// Detect platform information
pub fn detect_platform() -> PlatformInfo {
    PlatformInfo {
        cpu: detect_cpu_info(),
        memory_mb: detect_memory_mb(),
        os: detect_os(),
        kernel: detect_kernel(),
        arch: std::env::consts::ARCH.to_string(),
        hostname: detect_hostname(),
        virtualized: detect_virtualization(),
        numa_nodes: detect_numa_nodes(),
        cpu_governor: detect_cpu_governor(),
    }
}

/// Detect CPU information
fn detect_cpu_info() -> CpuInfo {
    #[cfg(target_os = "linux")]
    {
        detect_cpu_info_linux()
    }
    #[cfg(not(target_os = "linux"))]
    {
        CpuInfo {
            model: "Unknown".to_string(),
            physical_cores: num_cpus::get_physical(),
            logical_cores: num_cpus::get(),
            l1d_cache_kb: None,
            l2_cache_kb: None,
            l3_cache_kb: None,
            base_freq_mhz: None,
            measured_freq_mhz: None,
            features: vec![],
        }
    }
}

#[cfg(target_os = "linux")]
fn detect_cpu_info_linux() -> CpuInfo {
    let cpuinfo = fs::read_to_string("/proc/cpuinfo").unwrap_or_default();

    let model = cpuinfo
        .lines()
        .find(|l| l.starts_with("model name"))
        .and_then(|l| l.split(':').nth(1))
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "Unknown".to_string());

    let base_freq_mhz = cpuinfo
        .lines()
        .find(|l| l.starts_with("cpu MHz"))
        .and_then(|l| l.split(':').nth(1))
        .and_then(|s| s.trim().parse::<f64>().ok())
        .map(|f| f as u32);

    // Detect CPU features
    let features: Vec<String> = cpuinfo
        .lines()
        .find(|l| l.starts_with("flags"))
        .map(|l| {
            l.split(':')
                .nth(1)
                .unwrap_or("")
                .split_whitespace()
                .filter(|f| {
                    // Relevant features for benchmarking
                    matches!(
                        *f,
                        "sse4_2" | "avx" | "avx2" | "avx512f" | "rdtsc" | "constant_tsc" | "nonstop_tsc"
                    )
                })
                .map(|s| s.to_string())
                .collect()
        })
        .unwrap_or_default();

    // Try to get cache info from sysfs
    let l1d_cache_kb = read_cache_size("/sys/devices/system/cpu/cpu0/cache/index0/size");
    let l2_cache_kb = read_cache_size("/sys/devices/system/cpu/cpu0/cache/index2/size");
    let l3_cache_kb = read_cache_size("/sys/devices/system/cpu/cpu0/cache/index3/size");

    CpuInfo {
        model,
        physical_cores: num_cpus::get_physical(),
        logical_cores: num_cpus::get(),
        l1d_cache_kb,
        l2_cache_kb,
        l3_cache_kb,
        base_freq_mhz,
        measured_freq_mhz: None, // Set later by calibration
        features,
    }
}

#[cfg(target_os = "linux")]
fn read_cache_size(path: &str) -> Option<usize> {
    fs::read_to_string(path)
        .ok()
        .and_then(|s| {
            let s = s.trim();
            if s.ends_with('K') {
                s[..s.len() - 1].parse::<usize>().ok()
            } else if s.ends_with('M') {
                s[..s.len() - 1].parse::<usize>().ok().map(|v| v * 1024)
            } else {
                s.parse::<usize>().ok().map(|v| v / 1024)
            }
        })
}

/// Detect CPU frequency by measuring RDTSC cycles over a known duration
#[cfg(target_arch = "x86_64")]
pub fn detect_cpu_frequency() -> Option<f64> {
    use std::time::{Duration, Instant};

    // Measure over 100ms for reasonable accuracy
    let measure_duration = Duration::from_millis(100);

    let start_cycles = unsafe { core::arch::x86_64::_rdtsc() };
    let start_time = Instant::now();

    // Busy wait
    while start_time.elapsed() < measure_duration {
        std::hint::spin_loop();
    }

    let end_cycles = unsafe { core::arch::x86_64::_rdtsc() };
    let elapsed = start_time.elapsed();

    let cycles = end_cycles.wrapping_sub(start_cycles);
    let freq_hz = cycles as f64 / elapsed.as_secs_f64();
    let freq_mhz = freq_hz / 1_000_000.0;

    Some(freq_mhz)
}

#[cfg(not(target_arch = "x86_64"))]
pub fn detect_cpu_frequency() -> Option<f64> {
    None
}

fn detect_memory_mb() -> usize {
    #[cfg(target_os = "linux")]
    {
        fs::read_to_string("/proc/meminfo")
            .ok()
            .and_then(|s| {
                s.lines()
                    .find(|l| l.starts_with("MemTotal:"))
                    .and_then(|l| {
                        l.split_whitespace()
                            .nth(1)
                            .and_then(|v| v.parse::<usize>().ok())
                    })
            })
            .map(|kb| kb / 1024)
            .unwrap_or(0)
    }
    #[cfg(not(target_os = "linux"))]
    {
        0
    }
}

fn detect_os() -> String {
    #[cfg(target_os = "linux")]
    {
        fs::read_to_string("/etc/os-release")
            .ok()
            .and_then(|s| {
                s.lines()
                    .find(|l| l.starts_with("PRETTY_NAME="))
                    .map(|l| l.trim_start_matches("PRETTY_NAME=").trim_matches('"').to_string())
            })
            .unwrap_or_else(|| "Linux".to_string())
    }
    #[cfg(target_os = "macos")]
    {
        "macOS".to_string()
    }
    #[cfg(target_os = "windows")]
    {
        "Windows".to_string()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        std::env::consts::OS.to_string()
    }
}

fn detect_kernel() -> String {
    #[cfg(target_os = "linux")]
    {
        fs::read_to_string("/proc/version")
            .ok()
            .and_then(|s| s.split_whitespace().nth(2).map(|v| v.to_string()))
            .unwrap_or_else(|| "unknown".to_string())
    }
    #[cfg(not(target_os = "linux"))]
    {
        "unknown".to_string()
    }
}

fn detect_hostname() -> String {
    #[cfg(target_os = "linux")]
    {
        fs::read_to_string("/etc/hostname")
            .map(|s| s.trim().to_string())
            .unwrap_or_else(|_| "unknown".to_string())
    }
    #[cfg(not(target_os = "linux"))]
    {
        "unknown".to_string()
    }
}

fn detect_virtualization() -> bool {
    #[cfg(target_os = "linux")]
    {
        // Check for common virtualization indicators
        let cpuinfo = fs::read_to_string("/proc/cpuinfo").unwrap_or_default();
        let has_hypervisor = cpuinfo.contains("hypervisor");

        let dmi = fs::read_to_string("/sys/class/dmi/id/product_name").unwrap_or_default();
        let is_vm = dmi.to_lowercase().contains("virtual")
            || dmi.to_lowercase().contains("vmware")
            || dmi.to_lowercase().contains("kvm")
            || dmi.to_lowercase().contains("qemu");

        has_hypervisor || is_vm
    }
    #[cfg(not(target_os = "linux"))]
    {
        false
    }
}

fn detect_numa_nodes() -> usize {
    #[cfg(target_os = "linux")]
    {
        fs::read_dir("/sys/devices/system/node")
            .map(|entries| {
                entries
                    .filter_map(|e| e.ok())
                    .filter(|e| {
                        e.file_name()
                            .to_str()
                            .map(|s| s.starts_with("node"))
                            .unwrap_or(false)
                    })
                    .count()
            })
            .unwrap_or(1)
    }
    #[cfg(not(target_os = "linux"))]
    {
        1
    }
}

fn detect_cpu_governor() -> Option<String> {
    #[cfg(target_os = "linux")]
    {
        fs::read_to_string("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
            .ok()
            .map(|s| s.trim().to_string())
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

/// Check if constant TSC is available (required for accurate RDTSC timing)
#[cfg(target_arch = "x86_64")]
pub fn has_constant_tsc() -> bool {
    #[cfg(target_os = "linux")]
    {
        fs::read_to_string("/proc/cpuinfo")
            .map(|s| s.contains("constant_tsc"))
            .unwrap_or(false)
    }
    #[cfg(not(target_os = "linux"))]
    {
        // Assume modern x86_64 has constant TSC
        true
    }
}

#[cfg(not(target_arch = "x86_64"))]
pub fn has_constant_tsc() -> bool {
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_platform() {
        let platform = detect_platform();
        assert!(!platform.cpu.model.is_empty());
        assert!(platform.cpu.logical_cores > 0);
        assert!(platform.cpu.physical_cores > 0);
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_detect_cpu_frequency() {
        let freq = detect_cpu_frequency();
        assert!(freq.is_some());
        let freq_mhz = freq.unwrap();
        // Sanity check: frequency should be between 100 MHz and 10 GHz
        assert!(freq_mhz > 100.0 && freq_mhz < 10000.0);
    }

    #[test]
    fn test_detect_numa_nodes() {
        let nodes = detect_numa_nodes();
        assert!(nodes >= 1);
    }
}
