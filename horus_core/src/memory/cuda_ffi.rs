//! Custom CUDA FFI bindings for IPC memory sharing
//!
//! This module provides minimal, hand-crafted bindings to the CUDA Runtime API
//! for inter-process GPU memory sharing. We avoid external crate dependencies
//! to ensure stability and control.
//!
//! # CUDA IPC Overview
//!
//! CUDA IPC allows sharing GPU memory between processes on Linux:
//! 1. Process A allocates GPU memory with `cudaMalloc`
//! 2. Process A exports handle with `cudaIpcGetMemHandle`
//! 3. Handle (64 bytes) is shared via shared memory/socket
//! 4. Process B imports with `cudaIpcOpenMemHandle`
//! 5. Process B gets a device pointer to the SAME GPU memory
//!
//! # Platform Support
//! - Linux: Full support
//! - Windows: Limited (performance penalty)
//! - macOS: Not supported (no CUDA)

use std::ffi::c_void;
use std::ptr;

/// CUDA IPC handle size (64 bytes, defined by NVIDIA)
pub const CUDA_IPC_HANDLE_SIZE: usize = 64;

/// CUDA error codes
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CudaError {
    Success = 0,
    InvalidValue = 1,
    MemoryAllocation = 2,
    InitializationError = 3,
    LaunchFailure = 4,
    InvalidDevice = 10,
    InvalidMemcpyDirection = 21,
    NotSupported = 71,
    InvalidHandle = 400,
    NotReady = 600,
    Unknown = 999,
}

impl CudaError {
    pub fn from_code(code: i32) -> Self {
        match code {
            0 => CudaError::Success,
            1 => CudaError::InvalidValue,
            2 => CudaError::MemoryAllocation,
            3 => CudaError::InitializationError,
            4 => CudaError::LaunchFailure,
            10 => CudaError::InvalidDevice,
            21 => CudaError::InvalidMemcpyDirection,
            71 => CudaError::NotSupported,
            400 => CudaError::InvalidHandle,
            600 => CudaError::NotReady,
            _ => CudaError::Unknown,
        }
    }

    pub fn is_success(&self) -> bool {
        *self == CudaError::Success
    }
}

impl std::fmt::Display for CudaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CudaError::Success => write!(f, "CUDA Success"),
            CudaError::InvalidValue => write!(f, "CUDA Invalid Value"),
            CudaError::MemoryAllocation => write!(f, "CUDA Memory Allocation Failed"),
            CudaError::InitializationError => write!(f, "CUDA Initialization Error"),
            CudaError::LaunchFailure => write!(f, "CUDA Launch Failure"),
            CudaError::InvalidDevice => write!(f, "CUDA Invalid Device"),
            CudaError::InvalidMemcpyDirection => write!(f, "CUDA Invalid Memcpy Direction"),
            CudaError::NotSupported => write!(f, "CUDA Operation Not Supported"),
            CudaError::InvalidHandle => write!(f, "CUDA Invalid Handle"),
            CudaError::NotReady => write!(f, "CUDA Not Ready"),
            CudaError::Unknown => write!(f, "CUDA Unknown Error"),
        }
    }
}

impl std::error::Error for CudaError {}

/// CUDA IPC memory handle - 64 bytes opaque data
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CudaIpcMemHandle {
    pub reserved: [u8; CUDA_IPC_HANDLE_SIZE],
}

impl Default for CudaIpcMemHandle {
    fn default() -> Self {
        Self {
            reserved: [0u8; CUDA_IPC_HANDLE_SIZE],
        }
    }
}

impl std::fmt::Debug for CudaIpcMemHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Show first 8 bytes as hex for debugging
        write!(f, "CudaIpcMemHandle({:02x?}...)", &self.reserved[..8])
    }
}

/// Flags for cudaIpcOpenMemHandle
#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum CudaIpcMemFlags {
    /// Automatically enable peer access if needed
    LazyEnablePeerAccess = 0x1,
}

/// CUDA memory copy direction
#[repr(i32)]
#[derive(Debug, Clone, Copy)]
pub enum CudaMemcpyKind {
    HostToHost = 0,
    HostToDevice = 1,
    DeviceToHost = 2,
    DeviceToDevice = 3,
    Default = 4,
}

// FFI declarations - linked against libcudart.so at runtime
#[cfg(feature = "cuda")]
extern "C" {
    // Device management
    fn cudaGetDeviceCount(count: *mut i32) -> i32;
    fn cudaSetDevice(device: i32) -> i32;
    fn cudaGetDevice(device: *mut i32) -> i32;
    fn cudaDeviceSynchronize() -> i32;

    // Memory allocation
    fn cudaMalloc(dev_ptr: *mut *mut c_void, size: usize) -> i32;
    fn cudaFree(dev_ptr: *mut c_void) -> i32;
    fn cudaMemcpy(dst: *mut c_void, src: *const c_void, count: usize, kind: i32) -> i32;
    fn cudaMemset(dev_ptr: *mut c_void, value: i32, count: usize) -> i32;

    // IPC functions
    fn cudaIpcGetMemHandle(handle: *mut CudaIpcMemHandle, dev_ptr: *mut c_void) -> i32;
    fn cudaIpcOpenMemHandle(dev_ptr: *mut *mut c_void, handle: CudaIpcMemHandle, flags: u32)
        -> i32;
    fn cudaIpcCloseMemHandle(dev_ptr: *mut c_void) -> i32;

    // Error handling
    fn cudaGetLastError() -> i32;
    fn cudaPeekAtLastError() -> i32;
}

/// Result type for CUDA operations
pub type CudaResult<T> = Result<T, CudaError>;

/// Check if CUDA is available at runtime
#[cfg(feature = "cuda")]
pub fn cuda_available() -> bool {
    let mut count: i32 = 0;
    unsafe {
        let err = cudaGetDeviceCount(&mut count);
        err == 0 && count > 0
    }
}

#[cfg(not(feature = "cuda"))]
pub fn cuda_available() -> bool {
    false
}

/// Get number of CUDA devices
#[cfg(feature = "cuda")]
pub fn get_device_count() -> CudaResult<i32> {
    let mut count: i32 = 0;
    unsafe {
        let err = CudaError::from_code(cudaGetDeviceCount(&mut count));
        if err.is_success() {
            Ok(count)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn get_device_count() -> CudaResult<i32> {
    Ok(0)
}

/// Set current CUDA device
#[cfg(feature = "cuda")]
pub fn set_device(device: i32) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaSetDevice(device));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn set_device(_device: i32) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Get current CUDA device
#[cfg(feature = "cuda")]
pub fn get_device() -> CudaResult<i32> {
    let mut device: i32 = 0;
    unsafe {
        let err = CudaError::from_code(cudaGetDevice(&mut device));
        if err.is_success() {
            Ok(device)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn get_device() -> CudaResult<i32> {
    Err(CudaError::NotSupported)
}

/// Synchronize current device
#[cfg(feature = "cuda")]
pub fn device_synchronize() -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaDeviceSynchronize());
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn device_synchronize() -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Allocate GPU memory
#[cfg(feature = "cuda")]
pub fn malloc(size: usize) -> CudaResult<*mut c_void> {
    let mut ptr: *mut c_void = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaMalloc(&mut ptr, size));
        if err.is_success() {
            Ok(ptr)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn malloc(_size: usize) -> CudaResult<*mut c_void> {
    Err(CudaError::NotSupported)
}

/// Free GPU memory
#[cfg(feature = "cuda")]
pub fn free(ptr: *mut c_void) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaFree(ptr));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn free(_ptr: *mut c_void) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Copy memory between host and device
#[cfg(feature = "cuda")]
pub fn memcpy(
    dst: *mut c_void,
    src: *const c_void,
    size: usize,
    kind: CudaMemcpyKind,
) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaMemcpy(dst, src, size, kind as i32));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn memcpy(
    _dst: *mut c_void,
    _src: *const c_void,
    _size: usize,
    _kind: CudaMemcpyKind,
) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Set GPU memory to a value
#[cfg(feature = "cuda")]
pub fn memset(ptr: *mut c_void, value: i32, size: usize) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaMemset(ptr, value, size));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn memset(_ptr: *mut c_void, _value: i32, _size: usize) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Get IPC handle for GPU memory (for sharing with other processes)
#[cfg(feature = "cuda")]
pub fn ipc_get_mem_handle(dev_ptr: *mut c_void) -> CudaResult<CudaIpcMemHandle> {
    let mut handle = CudaIpcMemHandle::default();
    unsafe {
        let err = CudaError::from_code(cudaIpcGetMemHandle(&mut handle, dev_ptr));
        if err.is_success() {
            Ok(handle)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn ipc_get_mem_handle(_dev_ptr: *mut c_void) -> CudaResult<CudaIpcMemHandle> {
    Err(CudaError::NotSupported)
}

/// Open IPC handle from another process (get device pointer to shared GPU memory)
#[cfg(feature = "cuda")]
pub fn ipc_open_mem_handle(handle: CudaIpcMemHandle) -> CudaResult<*mut c_void> {
    let mut ptr: *mut c_void = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaIpcOpenMemHandle(
            &mut ptr,
            handle,
            CudaIpcMemFlags::LazyEnablePeerAccess as u32,
        ));
        if err.is_success() {
            Ok(ptr)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn ipc_open_mem_handle(_handle: CudaIpcMemHandle) -> CudaResult<*mut c_void> {
    Err(CudaError::NotSupported)
}

/// Close IPC handle (must be called when done with shared memory)
#[cfg(feature = "cuda")]
pub fn ipc_close_mem_handle(dev_ptr: *mut c_void) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaIpcCloseMemHandle(dev_ptr));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn ipc_close_mem_handle(_dev_ptr: *mut c_void) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Get last CUDA error and clear it
#[cfg(feature = "cuda")]
pub fn get_last_error() -> CudaError {
    unsafe { CudaError::from_code(cudaGetLastError()) }
}

#[cfg(not(feature = "cuda"))]
pub fn get_last_error() -> CudaError {
    CudaError::NotSupported
}

/// Peek at last CUDA error without clearing
#[cfg(feature = "cuda")]
pub fn peek_at_last_error() -> CudaError {
    unsafe { CudaError::from_code(cudaPeekAtLastError()) }
}

#[cfg(not(feature = "cuda"))]
pub fn peek_at_last_error() -> CudaError {
    CudaError::NotSupported
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cuda_ipc_handle_size() {
        assert_eq!(std::mem::size_of::<CudaIpcMemHandle>(), 64);
    }

    #[test]
    fn test_cuda_error_display() {
        assert_eq!(format!("{}", CudaError::Success), "CUDA Success");
        assert_eq!(
            format!("{}", CudaError::MemoryAllocation),
            "CUDA Memory Allocation Failed"
        );
    }

    #[test]
    #[cfg(feature = "cuda")]
    fn test_get_device_count() {
        // This will work on machines with CUDA
        let result = get_device_count();
        // Either succeeds with count >= 0 or fails with error
        match result {
            Ok(count) => assert!(count >= 0),
            Err(e) => println!("CUDA not available: {}", e),
        }
    }
}
