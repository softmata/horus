//! GPU hardware detection for HORUS
//!
//! Detects GPU capabilities at startup and caches the result in a `OnceLock`
//! for zero-overhead access on subsequent calls. This drives allocation strategy:
//!
//! - **No GPU**: CPU-only mmap pools (default path)
//! - **Unified memory (Jetson)**: `cudaMallocManaged` — CPU and GPU share physical RAM
//! - **Discrete GPU**: `cudaMalloc` for device memory + pinned host for transfers

use std::sync::OnceLock;

use super::cuda_ffi::{self, CudaDeviceAttr};

/// GPU capability level detected at runtime.
///
/// Determines the memory allocation strategy for tensor pools.
/// Query the current system's capability with [`super::gpu_capability()`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpuCapability {
    /// No CUDA-capable GPU detected — pure CPU allocation via mmap.
    None,

    /// Integrated GPU with unified memory (e.g. NVIDIA Jetson).
    ///
    /// CPU and GPU share the same physical memory, so `cudaMallocManaged`
    /// gives both sides direct access with zero copies.
    UnifiedMemory {
        device_id: i32,
        /// Whether concurrent managed access is supported (Jetson Xavier+).
        /// If true, no explicit `cudaDeviceSynchronize` needed between CPU/GPU access.
        concurrent_access: bool,
    },

    /// Discrete GPU with dedicated VRAM (desktop/server GPUs).
    ///
    /// CPU and GPU have separate memory. Use `cudaMalloc` for device memory
    /// and `cudaHostAlloc` (pinned) for fast DMA transfers.
    DiscreteGpu {
        device_id: i32,
        /// Number of CUDA devices available.
        device_count: usize,
    },
}

impl GpuCapability {
    /// Returns true if any GPU is available.
    pub fn has_gpu(&self) -> bool {
        !matches!(self, GpuCapability::None)
    }

    /// Returns true if this is a unified memory device (Jetson).
    pub fn is_unified(&self) -> bool {
        matches!(self, GpuCapability::UnifiedMemory { .. })
    }

    /// Returns true if explicit coherency sync is needed (non-ConcurrentManagedAccess Jetson).
    ///
    /// On Xavier+ (ConcurrentManagedAccess = true), hardware maintains CPU/GPU coherency.
    /// On older Jetson without CMA, applications need explicit `cudaDeviceSynchronize`
    /// before GPU kernel launch if data was written by CPU.
    pub fn needs_coherency_sync(&self) -> bool {
        matches!(
            self,
            GpuCapability::UnifiedMemory {
                concurrent_access: false,
                ..
            }
        )
    }

    /// Returns the primary device ID, or None if no GPU.
    pub fn device_id(&self) -> Option<i32> {
        match self {
            GpuCapability::None => None,
            GpuCapability::UnifiedMemory { device_id, .. } => Some(*device_id),
            GpuCapability::DiscreteGpu { device_id, .. } => Some(*device_id),
        }
    }
}

/// Cached GPU detection result — initialized once, read forever.
static GPU_CAPABILITY: OnceLock<GpuCapability> = OnceLock::new();

/// Detect GPU hardware and cache the result.
///
/// First call probes CUDA. All subsequent calls return the cached result
/// with zero overhead (just an atomic load).
pub(crate) fn detect_gpu() -> GpuCapability {
    *GPU_CAPABILITY.get_or_init(probe_gpu_hardware)
}

/// Actually probe the hardware. Called exactly once.
fn probe_gpu_hardware() -> GpuCapability {
    // Check if CUDA runtime is available at all
    if !cuda_ffi::cuda_available() {
        return GpuCapability::None;
    }

    let device_count = match cuda_ffi::get_device_count() {
        Ok(count) if count > 0 => count as usize,
        _ => return GpuCapability::None,
    };

    // Use device 0 as the primary GPU for capability detection
    let device_id: i32 = 0;

    // Check if the device is integrated (shares memory with host = Jetson/iGPU)
    let is_integrated = cuda_ffi::device_get_attribute(CudaDeviceAttr::Integrated, device_id)
        .map(|v| v != 0)
        .unwrap_or(false);

    if is_integrated {
        // Integrated GPU — check if it supports managed memory
        let supports_managed =
            cuda_ffi::device_get_attribute(CudaDeviceAttr::ManagedMemory, device_id)
                .map(|v| v != 0)
                .unwrap_or(false);

        if supports_managed {
            let concurrent_access = cuda_ffi::device_get_attribute(
                CudaDeviceAttr::ConcurrentManagedAccess,
                device_id,
            )
            .map(|v| v != 0)
            .unwrap_or(false);

            return GpuCapability::UnifiedMemory {
                device_id,
                concurrent_access,
            };
        }
    }

    // Discrete GPU (or integrated without managed memory support)
    GpuCapability::DiscreteGpu {
        device_id,
        device_count,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpu_capability_none() {
        let cap = GpuCapability::None;
        assert!(!cap.has_gpu());
        assert!(!cap.is_unified());
        assert_eq!(cap.device_id(), None);
    }

    #[test]
    fn test_gpu_capability_unified() {
        let cap = GpuCapability::UnifiedMemory {
            device_id: 0,
            concurrent_access: true,
        };
        assert!(cap.has_gpu());
        assert!(cap.is_unified());
        assert_eq!(cap.device_id(), Some(0));
        assert!(!cap.needs_coherency_sync()); // Xavier+ has hardware coherency
    }

    #[test]
    fn test_gpu_capability_unified_needs_sync() {
        let cap = GpuCapability::UnifiedMemory {
            device_id: 0,
            concurrent_access: false,
        };
        assert!(cap.is_unified());
        assert!(cap.needs_coherency_sync()); // Non-CMA needs explicit sync
    }

    #[test]
    fn test_gpu_capability_discrete() {
        let cap = GpuCapability::DiscreteGpu {
            device_id: 0,
            device_count: 2,
        };
        assert!(cap.has_gpu());
        assert!(!cap.is_unified());
        assert_eq!(cap.device_id(), Some(0));
        assert!(!cap.needs_coherency_sync()); // Discrete GPU doesn't use unified memory
    }

    #[test]
    fn test_detect_gpu_is_deterministic() {
        // detect_gpu() should return the same result on repeated calls
        let first = detect_gpu();
        let second = detect_gpu();
        assert_eq!(first, second);
    }

    #[test]
    fn test_detect_gpu_returns_valid_capability() {
        let cap = detect_gpu();
        // On CI (no GPU): should be None
        // On GPU machine: should be UnifiedMemory or DiscreteGpu
        match cap {
            GpuCapability::None => {
                println!("No GPU detected (CPU-only)");
            }
            GpuCapability::UnifiedMemory {
                device_id,
                concurrent_access,
            } => {
                println!(
                    "Unified memory GPU (Jetson) detected: device={}, concurrent={}",
                    device_id, concurrent_access
                );
                assert!(device_id >= 0);
            }
            GpuCapability::DiscreteGpu {
                device_id,
                device_count,
            } => {
                println!(
                    "Discrete GPU detected: device={}, count={}",
                    device_id, device_count
                );
                assert!(device_id >= 0);
                assert!(device_count > 0);
            }
        }
    }

    #[test]
    fn test_gpu_capability_detects_hardware() {
        use super::super::cuda_available;

        if !cuda_available() {
            return;
        }

        let cap = detect_gpu();
        assert!(cap.has_gpu(), "CUDA available but gpu_capability() says no GPU");
        assert!(cap.device_id().is_some(), "GPU detected but no device_id");
    }

    #[test]
    fn test_unified_memory_coherency_sync() {
        use super::super::cuda_available;

        if !cuda_available() {
            return;
        }

        let cap = detect_gpu();
        match cap {
            GpuCapability::UnifiedMemory {
                concurrent_access, ..
            } => {
                if concurrent_access {
                    assert!(!cap.needs_coherency_sync());
                } else {
                    assert!(cap.needs_coherency_sync());
                }
            }
            GpuCapability::DiscreteGpu { .. } => {
                assert!(!cap.needs_coherency_sync());
            }
            GpuCapability::None => {}
        }
    }
}
