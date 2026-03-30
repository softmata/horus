//! GPU platform detection.
//!
//! Detects what GPU hardware is present: Jetson (unified memory, specific chip)
//! vs discrete GPU (separate memory, PCIe). Cached via `OnceLock` — detection
//! runs once per process.

use super::cuda_ffi;
use std::sync::OnceLock;

/// Jetson SoC chip variant.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum JetsonChip {
    Orin,
    Xavier,
    Nano,
    /// Unknown Jetson (detected via device-tree but chip not recognized).
    Unknown(String),
}

impl std::fmt::Display for JetsonChip {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            JetsonChip::Orin => write!(f, "orin"),
            JetsonChip::Xavier => write!(f, "xavier"),
            JetsonChip::Nano => write!(f, "nano"),
            JetsonChip::Unknown(s) => write!(f, "unknown({})", s),
        }
    }
}

/// Detected GPU platform.
#[derive(Debug, Clone)]
pub enum GpuPlatform {
    /// NVIDIA Jetson with unified CPU/GPU memory.
    Jetson {
        chip: JetsonChip,
        name: String,
        compute_capability: (i32, i32),
        vram_bytes: usize,
    },
    /// Discrete NVIDIA GPU (separate memory, PCIe).
    Discrete {
        name: String,
        compute_capability: (i32, i32),
        vram_bytes: usize,
    },
}

impl GpuPlatform {
    /// Human-readable platform name for diagnostics.
    pub fn display_name(&self) -> String {
        match self {
            GpuPlatform::Jetson { chip, name, .. } => format!("Jetson {} ({})", chip, name),
            GpuPlatform::Discrete { name, .. } => name.clone(),
        }
    }

    /// Whether this platform has unified CPU/GPU memory.
    pub fn is_unified_memory(&self) -> bool {
        matches!(self, GpuPlatform::Jetson { .. })
    }

    /// GPU name.
    pub fn name(&self) -> &str {
        match self {
            GpuPlatform::Jetson { name, .. } => name,
            GpuPlatform::Discrete { name, .. } => name,
        }
    }

    /// Compute capability (major, minor).
    pub fn compute_capability(&self) -> (i32, i32) {
        match self {
            GpuPlatform::Jetson {
                compute_capability, ..
            } => *compute_capability,
            GpuPlatform::Discrete {
                compute_capability, ..
            } => *compute_capability,
        }
    }

    /// Total GPU memory in bytes.
    pub fn vram_bytes(&self) -> usize {
        match self {
            GpuPlatform::Jetson { vram_bytes, .. } => *vram_bytes,
            GpuPlatform::Discrete { vram_bytes, .. } => *vram_bytes,
        }
    }
}

impl std::fmt::Display for GpuPlatform {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GpuPlatform::Jetson {
                chip,
                name,
                compute_capability: (maj, min),
                vram_bytes,
            } => write!(
                f,
                "Jetson {} ({}, sm_{}{}, {:.1} GB)",
                chip,
                name,
                maj,
                min,
                *vram_bytes as f64 / (1024.0 * 1024.0 * 1024.0),
            ),
            GpuPlatform::Discrete {
                name,
                compute_capability: (maj, min),
                vram_bytes,
            } => write!(
                f,
                "{} (sm_{}{}, {:.1} GB)",
                name,
                maj,
                min,
                *vram_bytes as f64 / (1024.0 * 1024.0 * 1024.0),
            ),
        }
    }
}

/// Cached detection result.
static GPU_PLATFORM: OnceLock<Option<GpuPlatform>> = OnceLock::new();

/// Detect the GPU platform. Cached after first call.
///
/// Returns `None` if no CUDA GPU is available.
pub fn detect_gpu_platform() -> Option<&'static GpuPlatform> {
    GPU_PLATFORM.get_or_init(detect_gpu_platform_inner).as_ref()
}

fn detect_gpu_platform_inner() -> Option<GpuPlatform> {
    // Try to init CUDA
    cuda_ffi::init().ok()?;

    let count = cuda_ffi::device_count().ok()?;
    if count == 0 {
        return None;
    }

    // Use device 0 for platform detection
    let dev = cuda_ffi::device_get(0).ok()?;
    let name = cuda_ffi::device_name(dev).ok()?;
    let cc = cuda_ffi::compute_capability(dev).ok()?;
    let vram = cuda_ffi::device_total_mem(dev).ok()?;

    // Detect Jetson vs discrete
    let is_jetson = detect_jetson();

    if is_jetson {
        let chip = detect_jetson_chip(&name);
        Some(GpuPlatform::Jetson {
            chip,
            name,
            compute_capability: cc,
            vram_bytes: vram,
        })
    } else {
        Some(GpuPlatform::Discrete {
            name,
            compute_capability: cc,
            vram_bytes: vram,
        })
    }
}

/// Detect if we're running on Jetson hardware.
///
/// Strategy:
/// 1. Check /proc/device-tree/compatible for "nvidia,tegra" or "nvidia,jetson"
/// 2. Check /proc/device-tree/model for "Jetson"
/// 3. Check for unified memory support (iGPU indicator)
fn detect_jetson() -> bool {
    // Method 1: device-tree compatible string (most reliable on ARM Jetson)
    if let Ok(compat) = std::fs::read_to_string("/proc/device-tree/compatible") {
        let lower = compat.to_lowercase();
        if lower.contains("nvidia,tegra") || lower.contains("nvidia,jetson") {
            return true;
        }
    }

    // Method 2: device-tree model string
    if let Ok(model) = std::fs::read_to_string("/proc/device-tree/model") {
        if model.to_lowercase().contains("jetson") {
            return true;
        }
    }

    // Method 3: check if GPU reports integrated (no separate VRAM bus)
    // On x86, this is never true for NVIDIA GPUs. On ARM Jetson, it is.
    if let Ok(dev) = cuda_ffi::device_get(0) {
        // Attribute 18 = CU_DEVICE_ATTRIBUTE_INTEGRATED
        if let Ok(integrated) = cuda_ffi::device_attribute(18, dev) {
            if integrated != 0 {
                return true;
            }
        }
    }

    false
}

/// Identify the Jetson chip from the GPU name.
fn detect_jetson_chip(gpu_name: &str) -> JetsonChip {
    let lower = gpu_name.to_lowercase();
    if lower.contains("orin") {
        JetsonChip::Orin
    } else if lower.contains("xavier") {
        JetsonChip::Xavier
    } else if lower.contains("nano") || lower.contains("tegra") {
        JetsonChip::Nano
    } else {
        // Try device-tree model for more info
        let model = std::fs::read_to_string("/proc/device-tree/model")
            .unwrap_or_default()
            .trim_end_matches('\0')
            .to_string();
        if model.to_lowercase().contains("orin") {
            JetsonChip::Orin
        } else if model.to_lowercase().contains("xavier") {
            JetsonChip::Xavier
        } else if model.to_lowercase().contains("nano") {
            JetsonChip::Nano
        } else {
            JetsonChip::Unknown(model)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_gpu_platform() {
        let platform = detect_gpu_platform();
        match platform {
            Some(p) => {
                eprintln!("Detected GPU platform: {}", p);
                eprintln!("  Unified memory: {}", p.is_unified_memory());
                eprintln!("  VRAM: {:.1} GB", p.vram_bytes() as f64 / 1e9);
                assert!(!p.name().is_empty());
                let (maj, _) = p.compute_capability();
                assert!(maj >= 3, "compute capability too low");
            }
            None => {
                eprintln!("No GPU detected (CPU-only machine)");
            }
        }
    }

    #[test]
    fn test_platform_is_cached() {
        let p1 = detect_gpu_platform();
        let p2 = detect_gpu_platform();
        // Both should point to the same OnceLock storage
        match (p1, p2) {
            (Some(a), Some(b)) => {
                assert_eq!(a.name(), b.name());
            }
            (None, None) => {} // Both None is fine
            _ => panic!("Inconsistent detection results"),
        }
    }
}
