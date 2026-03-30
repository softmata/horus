//! GPU support for HORUS tensor pools and scheduling.
//!
//! This module provides CUDA integration via the driver API (`libcuda.so`),
//! loaded at runtime via dlopen. No link-time CUDA dependency.
//!
//! # Feature gate
//!
//! All GPU code is behind the `cuda` feature flag. Without it, the module
//! compiles to an empty stub.

// Internal modules — not user-facing
pub(crate) mod backends;
pub(crate) mod cuda_ffi;
pub(crate) mod kernels;
pub(crate) mod stream;

// Public modules — user-facing APIs
pub mod image_ops;
pub mod platform;

// Internal re-exports (used by pool_registry, gpu_executor, etc.)
pub(crate) use backends::{CudaDeviceBackend, CudaManagedBackend, CudaPinnedBackend};
pub(crate) use stream::{CudaEvent, CudaStream};

// Public re-exports — user-facing
pub use image_ops::{ColorFormat, GpuImageOps};
pub use platform::{detect_gpu_platform, GpuPlatform, JetsonChip};

/// Check if CUDA GPU support is available.
///
/// Returns `true` if the CUDA driver can be loaded and initialized.
/// This does NOT require the CUDA toolkit — only the GPU driver.
///
/// The result is cached after the first call.
#[inline]
pub fn cuda_available() -> bool {
    cuda_ffi::is_available()
}

/// Get the number of CUDA-capable GPU devices.
///
/// Returns 0 if CUDA is not available.
#[inline]
pub fn cuda_device_count() -> usize {
    cuda_ffi::device_count().unwrap_or(0) as usize
}

/// Synchronize the GPU — wait for all pending GPU operations to complete.
///
/// Call this after launching GPU kernels (via `GpuImageOps`) if you need
/// to read the results from CPU immediately. Inside a GPU node's `tick()`,
/// the executor handles synchronization automatically.
///
/// Returns `Ok(())` if CUDA is available and sync succeeded.
/// Returns `Err` if CUDA is not available.
pub fn gpu_synchronize() -> Result<(), String> {
    cuda_ffi::init().map_err(|e| e.to_string())?;
    let ctx = cuda_ffi::CudaContext::new(0).map_err(|e| e.to_string())?;
    ctx.synchronize().map_err(|e| e.to_string())
}

/// Get the detected GPU platform, if any.
///
/// Returns `None` on CPU-only machines. On GPU machines, returns
/// details about the GPU (Jetson vs discrete, name, compute capability, VRAM).
///
/// Cached after the first call.
#[inline]
pub fn gpu_platform() -> Option<&'static GpuPlatform> {
    detect_gpu_platform()
}

/// Create the optimal [`PoolBackend`] for a given device.
///
/// - `Device::cpu()` → [`MmapBackend`] (always, even on GPU machines)
/// - `Device::cuda(n)` on Jetson → [`CudaManagedBackend`] (unified memory)
/// - `Device::cuda(n)` on discrete → [`CudaManagedBackend`] (page migration, simpler than pinned for pool use)
/// - `Device::cuda(n)` with no GPU → falls back to `MmapBackend` with warning
///
/// Override via `HORUS_GPU_BACKEND=managed|pinned|device` env var.
///
/// [`PoolBackend`]: crate::memory::backend::PoolBackend
/// [`MmapBackend`]: crate::memory::backend::MmapBackend
pub(crate) fn auto_backend_for_device(
    device: crate::types::Device,
) -> Box<dyn crate::memory::backend::PoolBackend> {
    use crate::types::Device;

    if device.is_cpu() {
        // CPU — cannot create MmapBackend standalone (needs mmap pointers).
        // Callers should use TensorPool::new() for CPU pools, which creates
        // MmapBackend internally. This path returns a managed backend as
        // fallback, but CPU callers should never reach here.
        panic!(
            "auto_backend_for_device called with Device::cpu(). \
             Use TensorPool::new() for CPU pools — MmapBackend is created \
             internally from the mmap."
        );
    }

    let device_id = device.device_id;

    // Check environment override
    if let Ok(override_str) = std::env::var("HORUS_GPU_BACKEND") {
        match override_str.to_lowercase().as_str() {
            "managed" => {
                return Box::new(
                    CudaManagedBackend::new(device_id)
                        .expect("HORUS_GPU_BACKEND=managed but CudaManagedBackend creation failed"),
                );
            }
            "pinned" => {
                return Box::new(
                    CudaPinnedBackend::new(device_id)
                        .expect("HORUS_GPU_BACKEND=pinned but CudaPinnedBackend creation failed"),
                );
            }
            "device" => {
                return Box::new(
                    CudaDeviceBackend::new(device_id)
                        .expect("HORUS_GPU_BACKEND=device but CudaDeviceBackend creation failed"),
                );
            }
            other => {
                eprintln!(
                    "[horus] warning: unknown HORUS_GPU_BACKEND='{}', ignoring (valid: managed, pinned, device)",
                    other
                );
            }
        }
    }

    // Auto-detect based on platform
    if !cuda_available() {
        eprintln!(
            "[horus] warning: GPU backend requested for {} but CUDA not available; \
             tensor pool will not be functional for GPU operations.",
            device
        );
        // Return a managed backend attempt anyway — it will fail at alloc time
        // with a clear error rather than silently corrupting data.
        panic!(
            "Cannot create GPU backend: CUDA not available. \
             Check GPU driver installation."
        );
    }

    // Both Jetson and discrete get managed memory by default.
    // Managed is simpler for pool use: one pointer works on both CPU and GPU.
    // On Jetson it's true zero-copy; on discrete, driver migrates pages.
    match CudaManagedBackend::new(device_id) {
        Ok(backend) => Box::new(backend),
        Err(e) => {
            panic!(
                "Failed to create CUDA managed backend for device {}: {}",
                device_id, e
            );
        }
    }
}
