//! CUDA Driver API wrapper loaded via dlopen at runtime.
//!
//! No link-time CUDA dependency — `libcuda.so` is loaded dynamically.
//! All functions return `Result<_, CudaError>` for graceful failure
//! when CUDA is not installed.
//!
//! Uses the **Driver API** (`cuXxx` functions from `libcuda.so`), not the
//! Runtime API (`cudaXxx` from `libcudart.so`), because the driver library
//! is always present when an NVIDIA GPU driver is installed — no CUDA
//! toolkit installation required.
//!
//! # Safety
//!
//! All FFI calls are wrapped in safe Rust functions. Raw pointers from
//! CUDA are never exposed to callers — they're wrapped in typed structs.

#![allow(dead_code)]

use std::ffi::{c_char, c_int, c_uint, c_void, CStr};
use std::sync::OnceLock;

/// CUDA driver API result code.
type CUresult = c_int;

/// Opaque device handle.
type CUdevice = c_int;

/// Opaque context handle.
type CUcontext = *mut c_void;

/// Device pointer (GPU address).
pub type CUdeviceptr = u64;

/// CUDA device attribute enum values (subset we need).
pub const CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR: c_int = 75;
pub const CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR: c_int = 76;
pub const CU_DEVICE_ATTRIBUTE_TOTAL_MEMORY: c_int = 0; // not an attribute; use cuDeviceTotalMem
pub const CU_DEVICE_ATTRIBUTE_UNIFIED_ADDRESSING: c_int = 41;
pub const CU_DEVICE_ATTRIBUTE_MANAGED_MEMORY: c_int = 83;

/// Memory attach flags for cuMemAllocManaged.
const CU_MEM_ATTACH_GLOBAL: c_uint = 0x1;

/// CUDA error type.
#[derive(Debug, Clone)]
pub struct CudaError {
    pub code: i32,
    pub message: String,
}

impl std::fmt::Display for CudaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "CUDA error {}: {}", self.code, self.message)
    }
}

impl std::error::Error for CudaError {}

pub type CudaResult<T> = Result<T, CudaError>;

fn cuda_err(code: CUresult, context: &str) -> CudaError {
    CudaError {
        code,
        message: format!("{} (CUresult={})", context, code),
    }
}

/// Check CUresult and convert to Result.
fn check(code: CUresult, context: &str) -> CudaResult<()> {
    if code == 0 {
        Ok(())
    } else {
        Err(cuda_err(code, context))
    }
}

// ---------------------------------------------------------------------------
// Function pointer table loaded via dlopen
// ---------------------------------------------------------------------------

/// All CUDA driver API function pointers, loaded once from libcuda.so.
struct CudaApi {
    // Init
    cu_init: unsafe extern "C" fn(c_uint) -> CUresult,

    // Device management
    cu_device_get_count: unsafe extern "C" fn(*mut c_int) -> CUresult,
    cu_device_get: unsafe extern "C" fn(*mut CUdevice, c_int) -> CUresult,
    cu_device_get_name: unsafe extern "C" fn(*mut c_char, c_int, CUdevice) -> CUresult,
    cu_device_get_attribute: unsafe extern "C" fn(*mut c_int, c_int, CUdevice) -> CUresult,
    cu_device_total_mem_v2: unsafe extern "C" fn(*mut usize, CUdevice) -> CUresult,

    // Context
    cu_ctx_create_v2: unsafe extern "C" fn(*mut CUcontext, c_uint, CUdevice) -> CUresult,
    cu_ctx_destroy_v2: unsafe extern "C" fn(CUcontext) -> CUresult,
    cu_ctx_set_current: unsafe extern "C" fn(CUcontext) -> CUresult,

    // Memory allocation
    cu_mem_alloc_v2: unsafe extern "C" fn(*mut CUdeviceptr, usize) -> CUresult,
    cu_mem_alloc_managed: unsafe extern "C" fn(*mut CUdeviceptr, usize, c_uint) -> CUresult,
    cu_mem_alloc_host_v2: unsafe extern "C" fn(*mut *mut c_void, usize) -> CUresult,
    cu_mem_free_v2: unsafe extern "C" fn(CUdeviceptr) -> CUresult,
    cu_mem_free_host: unsafe extern "C" fn(*mut c_void) -> CUresult,

    // Memory operations
    cu_memcpy_dtoh_v2: unsafe extern "C" fn(*mut c_void, CUdeviceptr, usize) -> CUresult,
    cu_memcpy_htod_v2: unsafe extern "C" fn(CUdeviceptr, *const c_void, usize) -> CUresult,
    cu_memset_d8_v2: unsafe extern "C" fn(CUdeviceptr, u8, usize) -> CUresult,

    // Synchronization
    cu_ctx_synchronize: unsafe extern "C" fn() -> CUresult,
}

// SAFETY: Function pointers are immutable after loading and point to
// thread-safe CUDA driver functions.
unsafe impl Send for CudaApi {}
unsafe impl Sync for CudaApi {}

/// Global singleton — loaded once, cached forever.
static CUDA_API: OnceLock<Result<CudaApi, String>> = OnceLock::new();

/// Load a function pointer from the library handle.
///
/// # Safety
/// The symbol must exist and have the correct function signature.
unsafe fn load_sym<T>(handle: *mut c_void, name: &str) -> Result<T, String> {
    let cname = std::ffi::CString::new(name).unwrap();
    let ptr = libc::dlsym(handle, cname.as_ptr());
    if ptr.is_null() {
        let err = libc::dlerror();
        let msg = if err.is_null() {
            format!("symbol '{}' not found", name)
        } else {
            let s = CStr::from_ptr(err).to_string_lossy().to_string();
            format!("symbol '{}': {}", name, s)
        };
        return Err(msg);
    }
    // SAFETY: Caller guarantees T matches the symbol's actual type.
    Ok(std::mem::transmute_copy(&ptr))
}

/// Attempt to load the CUDA driver API from libcuda.so.
fn load_cuda_api() -> Result<CudaApi, String> {
    // Clear any prior dlerror
    unsafe { libc::dlerror() };

    let lib_name = std::ffi::CString::new("libcuda.so.1").unwrap();
    let handle = unsafe { libc::dlopen(lib_name.as_ptr(), libc::RTLD_NOW | libc::RTLD_GLOBAL) };
    if handle.is_null() {
        let err = unsafe { libc::dlerror() };
        let msg = if err.is_null() {
            "dlopen(libcuda.so.1) returned null".to_string()
        } else {
            unsafe { CStr::from_ptr(err).to_string_lossy().to_string() }
        };
        return Err(format!("CUDA driver not available: {}", msg));
    }

    // NOTE: We intentionally never dlclose() — the library stays loaded for
    // the process lifetime. This is standard practice for GPU drivers.

    unsafe {
        Ok(CudaApi {
            cu_init: load_sym(handle, "cuInit")?,
            cu_device_get_count: load_sym(handle, "cuDeviceGetCount")?,
            cu_device_get: load_sym(handle, "cuDeviceGet")?,
            cu_device_get_name: load_sym(handle, "cuDeviceGetName")?,
            cu_device_get_attribute: load_sym(handle, "cuDeviceGetAttribute")?,
            cu_device_total_mem_v2: load_sym(handle, "cuDeviceTotalMem_v2")?,
            cu_ctx_create_v2: load_sym(handle, "cuCtxCreate_v2")?,
            cu_ctx_destroy_v2: load_sym(handle, "cuCtxDestroy_v2")?,
            cu_ctx_set_current: load_sym(handle, "cuCtxSetCurrent")?,
            cu_mem_alloc_v2: load_sym(handle, "cuMemAlloc_v2")?,
            cu_mem_alloc_managed: load_sym(handle, "cuMemAllocManaged")?,
            cu_mem_alloc_host_v2: load_sym(handle, "cuMemAllocHost_v2")?,
            cu_mem_free_v2: load_sym(handle, "cuMemFree_v2")?,
            cu_mem_free_host: load_sym(handle, "cuMemFreeHost")?,
            cu_memcpy_dtoh_v2: load_sym(handle, "cuMemcpyDtoH_v2")?,
            cu_memcpy_htod_v2: load_sym(handle, "cuMemcpyHtoD_v2")?,
            cu_memset_d8_v2: load_sym(handle, "cuMemsetD8_v2")?,
            cu_ctx_synchronize: load_sym(handle, "cuCtxSynchronize")?,
        })
    }
}

/// Get the loaded CUDA API, or the error from the load attempt.
fn api() -> CudaResult<&'static CudaApi> {
    CUDA_API
        .get_or_init(load_cuda_api)
        .as_ref()
        .map_err(|msg| CudaError {
            code: -1,
            message: msg.clone(),
        })
}

// ---------------------------------------------------------------------------
// Public safe API
// ---------------------------------------------------------------------------

/// Check if CUDA is disabled via environment variable.
///
/// `HORUS_GPU=0` disables all GPU support. Any other value (or unset) allows GPU.
fn is_gpu_disabled() -> bool {
    matches!(
        std::env::var("HORUS_GPU").as_deref(),
        Ok("0") | Ok("false") | Ok("off")
    )
}

/// Initialize the CUDA driver. Must be called before any other function.
/// Safe to call multiple times (idempotent).
///
/// Returns error if `HORUS_GPU=0` is set.
pub fn init() -> CudaResult<()> {
    if is_gpu_disabled() {
        return Err(CudaError {
            code: -2,
            message: "GPU disabled via HORUS_GPU=0".to_string(),
        });
    }
    let api = api()?;
    check(unsafe { (api.cu_init)(0) }, "cuInit")
}

/// Check if CUDA is available (driver loadable and initializable).
///
/// Returns `false` if `HORUS_GPU=0` is set.
pub fn is_available() -> bool {
    if is_gpu_disabled() {
        return false;
    }
    init().is_ok()
}

/// Get the number of CUDA-capable devices.
pub fn device_count() -> CudaResult<i32> {
    let api = api()?;
    let mut count: c_int = 0;
    check(
        unsafe { (api.cu_device_get_count)(&mut count) },
        "cuDeviceGetCount",
    )?;
    Ok(count)
}

/// Get a device handle by index.
pub fn device_get(index: i32) -> CudaResult<i32> {
    let api = api()?;
    let mut dev: CUdevice = 0;
    check(
        unsafe { (api.cu_device_get)(&mut dev, index) },
        "cuDeviceGet",
    )?;
    Ok(dev)
}

/// Get the device name (up to 255 chars).
pub fn device_name(device: i32) -> CudaResult<String> {
    let api = api()?;
    let mut name = [0u8; 256];
    check(
        unsafe { (api.cu_device_get_name)(name.as_mut_ptr() as *mut c_char, 256, device) },
        "cuDeviceGetName",
    )?;
    let cstr = unsafe { CStr::from_ptr(name.as_ptr() as *const c_char) };
    Ok(cstr.to_string_lossy().to_string())
}

/// Get a device attribute.
pub fn device_attribute(attribute: i32, device: i32) -> CudaResult<i32> {
    let api = api()?;
    let mut value: c_int = 0;
    check(
        unsafe { (api.cu_device_get_attribute)(&mut value, attribute, device) },
        "cuDeviceGetAttribute",
    )?;
    Ok(value)
}

/// Get total device memory in bytes.
pub fn device_total_mem(device: i32) -> CudaResult<usize> {
    let api = api()?;
    let mut bytes: usize = 0;
    check(
        unsafe { (api.cu_device_total_mem_v2)(&mut bytes, device) },
        "cuDeviceTotalMem_v2",
    )?;
    Ok(bytes)
}

/// Compute capability as (major, minor).
pub fn compute_capability(device: i32) -> CudaResult<(i32, i32)> {
    let major = device_attribute(CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, device)?;
    let minor = device_attribute(CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, device)?;
    Ok((major, minor))
}

/// Whether the device supports managed (unified) memory.
pub fn supports_managed_memory(device: i32) -> CudaResult<bool> {
    Ok(device_attribute(CU_DEVICE_ATTRIBUTE_MANAGED_MEMORY, device)? != 0)
}

// ---------------------------------------------------------------------------
// Context management
// ---------------------------------------------------------------------------

/// Opaque CUDA context handle (RAII).
pub struct CudaContext {
    ctx: CUcontext,
}

// SAFETY: CUDA contexts can be shared across threads when using
// cuCtxSetCurrent to bind per-thread. The context handle is an opaque
// pointer that CUDA serializes internally.
unsafe impl Send for CudaContext {}
unsafe impl Sync for CudaContext {}

impl CudaContext {
    /// Create a new CUDA context on the given device.
    pub fn new(device: i32) -> CudaResult<Self> {
        let api = api()?;
        let mut ctx: CUcontext = std::ptr::null_mut();
        check(
            unsafe { (api.cu_ctx_create_v2)(&mut ctx, 0, device) },
            "cuCtxCreate_v2",
        )?;
        Ok(Self { ctx })
    }

    /// Make this context current on the calling thread.
    pub fn set_current(&self) -> CudaResult<()> {
        let api = api()?;
        check(
            unsafe { (api.cu_ctx_set_current)(self.ctx) },
            "cuCtxSetCurrent",
        )
    }

    /// Synchronize the context (wait for all GPU work to complete).
    pub fn synchronize(&self) -> CudaResult<()> {
        let api = api()?;
        check(unsafe { (api.cu_ctx_synchronize)() }, "cuCtxSynchronize")
    }
}

impl Drop for CudaContext {
    fn drop(&mut self) {
        if !self.ctx.is_null() {
            if let Ok(api) = api() {
                let _ = unsafe { (api.cu_ctx_destroy_v2)(self.ctx) };
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Memory allocation
// ---------------------------------------------------------------------------

/// Allocate device memory (GPU-only). Returns a device pointer.
pub fn mem_alloc(size: usize) -> CudaResult<CUdeviceptr> {
    let api = api()?;
    let mut dptr: CUdeviceptr = 0;
    check(
        unsafe { (api.cu_mem_alloc_v2)(&mut dptr, size) },
        "cuMemAlloc_v2",
    )?;
    Ok(dptr)
}

/// Allocate managed (unified) memory. Accessible from both CPU and GPU.
/// On Jetson, CPU and GPU pointers are the same physical address.
pub fn mem_alloc_managed(size: usize) -> CudaResult<CUdeviceptr> {
    let api = api()?;
    let mut dptr: CUdeviceptr = 0;
    check(
        unsafe { (api.cu_mem_alloc_managed)(&mut dptr, size, CU_MEM_ATTACH_GLOBAL) },
        "cuMemAllocManaged",
    )?;
    Ok(dptr)
}

/// Allocate page-locked (pinned) host memory. CPU-accessible, fast DMA to GPU.
pub fn mem_alloc_host(size: usize) -> CudaResult<*mut u8> {
    let api = api()?;
    let mut ptr: *mut c_void = std::ptr::null_mut();
    check(
        unsafe { (api.cu_mem_alloc_host_v2)(&mut ptr, size) },
        "cuMemAllocHost_v2",
    )?;
    Ok(ptr as *mut u8)
}

/// Free device or managed memory.
pub fn mem_free(dptr: CUdeviceptr) -> CudaResult<()> {
    let api = api()?;
    check(unsafe { (api.cu_mem_free_v2)(dptr) }, "cuMemFree_v2")
}

/// Free pinned host memory.
pub fn mem_free_host(ptr: *mut u8) -> CudaResult<()> {
    let api = api()?;
    check(
        unsafe { (api.cu_mem_free_host)(ptr as *mut c_void) },
        "cuMemFreeHost",
    )
}

/// Copy from device to host.
pub fn memcpy_dtoh(dst: *mut u8, src: CUdeviceptr, size: usize) -> CudaResult<()> {
    let api = api()?;
    check(
        unsafe { (api.cu_memcpy_dtoh_v2)(dst as *mut c_void, src, size) },
        "cuMemcpyDtoH_v2",
    )
}

/// Copy from host to device.
pub fn memcpy_htod(dst: CUdeviceptr, src: *const u8, size: usize) -> CudaResult<()> {
    let api = api()?;
    check(
        unsafe { (api.cu_memcpy_htod_v2)(dst, src as *const c_void, size) },
        "cuMemcpyHtoD_v2",
    )
}

/// Set device memory to a byte value.
pub fn memset_d8(dptr: CUdeviceptr, value: u8, size: usize) -> CudaResult<()> {
    let api = api()?;
    check(
        unsafe { (api.cu_memset_d8_v2)(dptr, value, size) },
        "cuMemsetD8_v2",
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cuda_available() {
        // This test always passes — it just checks whether CUDA loads.
        // On GPU machines: is_available() == true
        // On CPU machines: is_available() == false (graceful)
        let available = is_available();
        eprintln!("CUDA available: {}", available);
    }

    #[test]
    fn test_device_count() {
        if !is_available() {
            eprintln!("CUDA not available, skipping");
            return;
        }
        let count = device_count().unwrap();
        eprintln!("CUDA device count: {}", count);
        assert!(count > 0);
    }

    #[test]
    fn test_device_info() {
        if !is_available() {
            return;
        }
        let dev = device_get(0).unwrap();
        let name = device_name(dev).unwrap();
        let (major, minor) = compute_capability(dev).unwrap();
        let mem = device_total_mem(dev).unwrap();
        let managed = supports_managed_memory(dev).unwrap();
        eprintln!(
            "GPU 0: {} (sm_{}{}, {:.1} GB VRAM, managed={})",
            name,
            major,
            minor,
            mem as f64 / (1024.0 * 1024.0 * 1024.0),
            managed,
        );
    }

    #[test]
    fn test_managed_alloc_free() {
        if !is_available() {
            return;
        }
        let _ctx = CudaContext::new(0).unwrap();

        let size = 1024;
        let dptr = mem_alloc_managed(size).unwrap();
        assert_ne!(dptr, 0);

        // Managed memory is CPU-accessible — write and read back
        let ptr = dptr as *mut u8;
        unsafe {
            std::ptr::write_bytes(ptr, 0xAB, size);
            assert_eq!(*ptr, 0xAB);
        }

        mem_free(dptr).unwrap();
    }

    #[test]
    fn test_pinned_alloc_free() {
        if !is_available() {
            return;
        }
        let _ctx = CudaContext::new(0).unwrap();

        let size = 1024;
        let ptr = mem_alloc_host(size).unwrap();
        assert!(!ptr.is_null());

        // Pinned memory is CPU-accessible
        unsafe {
            std::ptr::write_bytes(ptr, 0xCD, size);
            assert_eq!(*ptr, 0xCD);
        }

        mem_free_host(ptr).unwrap();
    }

    #[test]
    fn test_device_alloc_memset_free() {
        if !is_available() {
            return;
        }
        let _ctx = CudaContext::new(0).unwrap();

        let size = 4096;
        let dptr = mem_alloc(size).unwrap();
        assert_ne!(dptr, 0);

        // Set device memory to zero
        memset_d8(dptr, 0, size).unwrap();

        // Copy to host and verify
        let mut host_buf = vec![0xFFu8; size];
        memcpy_dtoh(host_buf.as_mut_ptr(), dptr, size).unwrap();
        assert!(host_buf.iter().all(|&b| b == 0));

        mem_free(dptr).unwrap();
    }

    #[test]
    fn test_htod_dtoh_roundtrip() {
        if !is_available() {
            return;
        }
        let _ctx = CudaContext::new(0).unwrap();

        let data: Vec<u8> = (0..256).map(|i| i as u8).collect();
        let size = data.len();

        let dptr = mem_alloc(size).unwrap();
        memcpy_htod(dptr, data.as_ptr(), size).unwrap();

        let mut result = vec![0u8; size];
        memcpy_dtoh(result.as_mut_ptr(), dptr, size).unwrap();

        assert_eq!(data, result);
        mem_free(dptr).unwrap();
    }
}
