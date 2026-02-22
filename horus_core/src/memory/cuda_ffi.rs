// Raw pointer arguments are intentional in this CUDA FFI module —
// all pointers are passed directly to the CUDA Runtime API.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

//! CUDA FFI bindings for IPC memory sharing
//!
//! Minimal bindings to the CUDA Runtime API for inter-process GPU memory sharing.
//! This module is only compiled when the `cuda` feature is enabled (gated in `mod.rs`).
//!
//! # CUDA IPC Overview
//!
//! CUDA IPC allows sharing GPU memory between processes on Linux:
//! 1. Process A allocates GPU memory with `cudaMalloc`
//! 2. Process A exports handle with `cudaIpcGetMemHandle`
//! 3. Handle (64 bytes) is shared via shared memory
//! 4. Process B imports with `cudaIpcOpenMemHandle`
//! 5. Process B gets a device pointer to the SAME GPU memory
//!
//! # Safety
//!
//! All functions in this module call CUDA Runtime API C functions via FFI.
//! Pointer arguments must be valid, properly aligned, and point to caller-owned memory.
//! Device pointers must come from prior `cudaMalloc` or `cudaIpcOpenMemHandle` calls.
//! Error codes are checked after each call; resources are not used on failure.

use std::ffi::c_void;
use std::ptr;

// =============================================================================
// Types
// =============================================================================

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
            0 => Self::Success,
            1 => Self::InvalidValue,
            2 => Self::MemoryAllocation,
            3 => Self::InitializationError,
            4 => Self::LaunchFailure,
            10 => Self::InvalidDevice,
            21 => Self::InvalidMemcpyDirection,
            71 => Self::NotSupported,
            400 => Self::InvalidHandle,
            600 => Self::NotReady,
            _ => Self::Unknown,
        }
    }

    pub fn is_success(&self) -> bool {
        *self == Self::Success
    }
}

impl std::fmt::Display for CudaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Success => write!(f, "CUDA Success"),
            Self::InvalidValue => write!(f, "CUDA Invalid Value"),
            Self::MemoryAllocation => write!(f, "CUDA Memory Allocation Failed"),
            Self::InitializationError => write!(f, "CUDA Initialization Error"),
            Self::LaunchFailure => write!(f, "CUDA Launch Failure"),
            Self::InvalidDevice => write!(f, "CUDA Invalid Device"),
            Self::InvalidMemcpyDirection => write!(f, "CUDA Invalid Memcpy Direction"),
            Self::NotSupported => write!(f, "CUDA Operation Not Supported"),
            Self::InvalidHandle => write!(f, "CUDA Invalid Handle"),
            Self::NotReady => write!(f, "CUDA Not Ready"),
            Self::Unknown => write!(f, "CUDA Unknown Error"),
        }
    }
}

impl std::error::Error for CudaError {}

impl From<CudaError> for crate::error::HorusError {
    fn from(err: CudaError) -> Self {
        crate::error::HorusError::Memory(err.to_string())
    }
}

/// CUDA IPC memory handle — 64 bytes opaque data
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
        write!(f, "CudaIpcMemHandle({:02x?}...)", &self.reserved[..8])
    }
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

/// Opaque CUDA stream handle
pub type CudaStream = *mut c_void;

/// Opaque CUDA event handle
pub type CudaEvent = *mut c_void;

/// Result type for CUDA operations
pub type CudaResult<T> = Result<T, CudaError>;

// =============================================================================
// FFI declarations — linked against libcudart.so at runtime
// =============================================================================

extern "C" {
    fn cudaGetDeviceCount(count: *mut i32) -> i32;
    fn cudaSetDevice(device: i32) -> i32;
    fn cudaGetDevice(device: *mut i32) -> i32;
    fn cudaDeviceSynchronize() -> i32;

    fn cudaMalloc(dev_ptr: *mut *mut c_void, size: usize) -> i32;
    fn cudaFree(dev_ptr: *mut c_void) -> i32;
    fn cudaMemcpy(dst: *mut c_void, src: *const c_void, count: usize, kind: i32) -> i32;
    fn cudaMemset(dev_ptr: *mut c_void, value: i32, count: usize) -> i32;

    fn cudaIpcGetMemHandle(handle: *mut CudaIpcMemHandle, dev_ptr: *mut c_void) -> i32;
    fn cudaIpcOpenMemHandle(dev_ptr: *mut *mut c_void, handle: CudaIpcMemHandle, flags: u32)
        -> i32;
    fn cudaIpcCloseMemHandle(dev_ptr: *mut c_void) -> i32;

    fn cudaGetLastError() -> i32;
    fn cudaPeekAtLastError() -> i32;

    fn cudaStreamCreate(stream: *mut CudaStream) -> i32;
    fn cudaStreamCreateWithFlags(stream: *mut CudaStream, flags: u32) -> i32;
    fn cudaStreamDestroy(stream: CudaStream) -> i32;
    fn cudaStreamSynchronize(stream: CudaStream) -> i32;
    fn cudaStreamQuery(stream: CudaStream) -> i32;

    fn cudaEventCreate(event: *mut CudaEvent) -> i32;
    fn cudaEventCreateWithFlags(event: *mut CudaEvent, flags: u32) -> i32;
    fn cudaEventDestroy(event: CudaEvent) -> i32;
    fn cudaEventRecord(event: CudaEvent, stream: CudaStream) -> i32;
    fn cudaEventSynchronize(event: CudaEvent) -> i32;
    fn cudaEventQuery(event: CudaEvent) -> i32;
    fn cudaEventElapsedTime(ms: *mut f32, start: CudaEvent, end: CudaEvent) -> i32;
    fn cudaStreamWaitEvent(stream: CudaStream, event: CudaEvent, flags: u32) -> i32;

    fn cudaMallocHost(ptr: *mut *mut c_void, size: usize) -> i32;
    fn cudaFreeHost(ptr: *mut c_void) -> i32;
    fn cudaHostRegister(ptr: *mut c_void, size: usize, flags: u32) -> i32;
    fn cudaHostUnregister(ptr: *mut c_void) -> i32;
    fn cudaHostGetDevicePointer(
        dev_ptr: *mut *mut c_void,
        host_ptr: *mut c_void,
        flags: u32,
    ) -> i32;

    fn cudaMemcpyAsync(
        dst: *mut c_void,
        src: *const c_void,
        count: usize,
        kind: i32,
        stream: CudaStream,
    ) -> i32;
    fn cudaMemsetAsync(dev_ptr: *mut c_void, value: i32, count: usize, stream: CudaStream) -> i32;

    fn cudaDeviceCanAccessPeer(can_access: *mut i32, device: i32, peer_device: i32) -> i32;
    fn cudaDeviceEnablePeerAccess(peer_device: i32, flags: u32) -> i32;
    fn cudaDeviceDisablePeerAccess(peer_device: i32) -> i32;
}

// =============================================================================
// Helper
// =============================================================================

/// Check a CUDA return code, converting non-zero to Err
#[inline]
fn check(code: i32) -> CudaResult<()> {
    let err = CudaError::from_code(code);
    if err.is_success() {
        Ok(())
    } else {
        Err(err)
    }
}

// =============================================================================
// Device Management
// =============================================================================

/// Check if CUDA is available at runtime
pub fn cuda_available() -> bool {
    let mut count: i32 = 0;
    // SAFETY: count is a valid mutable i32 on the stack. cudaGetDeviceCount
    // writes the device count and returns 0 on success. If CUDA is not
    // installed or the driver is unavailable, the FFI call may fail to link
    // at load time (handled by the cuda feature gate).
    unsafe { cudaGetDeviceCount(&mut count) == 0 && count > 0 }
}

/// Get number of CUDA devices
pub fn get_device_count() -> CudaResult<i32> {
    let mut count = 0;
    unsafe { check(cudaGetDeviceCount(&mut count))? };
    Ok(count)
}

/// Set current CUDA device
pub fn set_device(device: i32) -> CudaResult<()> {
    unsafe { check(cudaSetDevice(device)) }
}

/// Get current CUDA device
pub fn get_device() -> CudaResult<i32> {
    let mut device = 0;
    unsafe { check(cudaGetDevice(&mut device))? };
    Ok(device)
}

/// Synchronize current device
pub fn device_synchronize() -> CudaResult<()> {
    unsafe { check(cudaDeviceSynchronize()) }
}

// =============================================================================
// Memory
// =============================================================================

/// Allocate GPU memory
pub fn malloc(size: usize) -> CudaResult<*mut c_void> {
    let mut ptr = ptr::null_mut();
    unsafe { check(cudaMalloc(&mut ptr, size))? };
    Ok(ptr)
}

/// Free GPU memory
pub fn free(ptr: *mut c_void) -> CudaResult<()> {
    unsafe { check(cudaFree(ptr)) }
}

/// Copy memory between host and device
pub fn memcpy(
    dst: *mut c_void,
    src: *const c_void,
    size: usize,
    kind: CudaMemcpyKind,
) -> CudaResult<()> {
    unsafe { check(cudaMemcpy(dst, src, size, kind as i32)) }
}

/// Set GPU memory to a value
pub fn memset(ptr: *mut c_void, value: i32, size: usize) -> CudaResult<()> {
    unsafe { check(cudaMemset(ptr, value, size)) }
}

// =============================================================================
// IPC
// =============================================================================

/// Get IPC handle for GPU memory (for sharing with other processes)
pub fn ipc_get_mem_handle(dev_ptr: *mut c_void) -> CudaResult<CudaIpcMemHandle> {
    let mut handle = CudaIpcMemHandle::default();
    unsafe { check(cudaIpcGetMemHandle(&mut handle, dev_ptr))? };
    Ok(handle)
}

/// Open IPC handle from another process
pub fn ipc_open_mem_handle(handle: CudaIpcMemHandle) -> CudaResult<*mut c_void> {
    let mut ptr = ptr::null_mut();
    unsafe { check(cudaIpcOpenMemHandle(&mut ptr, handle, 0x1))? }; // LazyEnablePeerAccess
    Ok(ptr)
}

/// Close IPC handle
pub fn ipc_close_mem_handle(dev_ptr: *mut c_void) -> CudaResult<()> {
    unsafe { check(cudaIpcCloseMemHandle(dev_ptr)) }
}

// =============================================================================
// Error Handling
// =============================================================================

/// Get last CUDA error and clear it
pub fn get_last_error() -> CudaError {
    unsafe { CudaError::from_code(cudaGetLastError()) }
}

/// Peek at last CUDA error without clearing
pub fn peek_at_last_error() -> CudaError {
    unsafe { CudaError::from_code(cudaPeekAtLastError()) }
}

// =============================================================================
// Streams
// =============================================================================

/// Create a new CUDA stream
pub fn stream_create() -> CudaResult<CudaStream> {
    let mut stream = ptr::null_mut();
    unsafe { check(cudaStreamCreate(&mut stream))? };
    Ok(stream)
}

/// Create a CUDA stream with flags (0 = default, 1 = non-blocking)
pub fn stream_create_with_flags(flags: u32) -> CudaResult<CudaStream> {
    let mut stream = ptr::null_mut();
    unsafe { check(cudaStreamCreateWithFlags(&mut stream, flags))? };
    Ok(stream)
}

/// Destroy a CUDA stream
pub fn stream_destroy(stream: CudaStream) -> CudaResult<()> {
    unsafe { check(cudaStreamDestroy(stream)) }
}

/// Synchronize a CUDA stream (wait for all operations to complete)
pub fn stream_synchronize(stream: CudaStream) -> CudaResult<()> {
    unsafe { check(cudaStreamSynchronize(stream)) }
}

/// Query if stream operations are complete (non-blocking)
pub fn stream_query(stream: CudaStream) -> CudaResult<bool> {
    let err = unsafe { CudaError::from_code(cudaStreamQuery(stream)) };
    match err {
        CudaError::Success => Ok(true),
        CudaError::NotReady => Ok(false),
        _ => Err(err),
    }
}

// =============================================================================
// Events
// =============================================================================

/// Create a new CUDA event
pub fn event_create() -> CudaResult<CudaEvent> {
    let mut event = ptr::null_mut();
    unsafe { check(cudaEventCreate(&mut event))? };
    Ok(event)
}

/// Create a CUDA event with flags (0=default, 1=blocking_sync, 2=disable_timing, 4=interprocess)
pub fn event_create_with_flags(flags: u32) -> CudaResult<CudaEvent> {
    let mut event = ptr::null_mut();
    unsafe { check(cudaEventCreateWithFlags(&mut event, flags))? };
    Ok(event)
}

/// Destroy a CUDA event
pub fn event_destroy(event: CudaEvent) -> CudaResult<()> {
    unsafe { check(cudaEventDestroy(event)) }
}

/// Record an event on a stream
pub fn event_record(event: CudaEvent, stream: CudaStream) -> CudaResult<()> {
    unsafe { check(cudaEventRecord(event, stream)) }
}

/// Synchronize on an event (wait for it to complete)
pub fn event_synchronize(event: CudaEvent) -> CudaResult<()> {
    unsafe { check(cudaEventSynchronize(event)) }
}

/// Query if event has completed (non-blocking)
pub fn event_query(event: CudaEvent) -> CudaResult<bool> {
    let err = unsafe { CudaError::from_code(cudaEventQuery(event)) };
    match err {
        CudaError::Success => Ok(true),
        CudaError::NotReady => Ok(false),
        _ => Err(err),
    }
}

/// Get elapsed time between two events in milliseconds
pub fn event_elapsed_time(start: CudaEvent, end: CudaEvent) -> CudaResult<f32> {
    let mut ms = 0.0;
    unsafe { check(cudaEventElapsedTime(&mut ms, start, end))? };
    Ok(ms)
}

/// Make a stream wait for an event
pub fn stream_wait_event(stream: CudaStream, event: CudaEvent) -> CudaResult<()> {
    unsafe { check(cudaStreamWaitEvent(stream, event, 0)) }
}

// =============================================================================
// Pinned (Page-Locked) Host Memory
// =============================================================================

/// Allocate pinned host memory for faster CPU<->GPU transfers
pub fn malloc_host(size: usize) -> CudaResult<*mut c_void> {
    let mut ptr = ptr::null_mut();
    unsafe { check(cudaMallocHost(&mut ptr, size))? };
    Ok(ptr)
}

/// Free pinned host memory
pub fn free_host(ptr: *mut c_void) -> CudaResult<()> {
    unsafe { check(cudaFreeHost(ptr)) }
}

/// Register existing host memory as pinned (0=default, 1=portable, 2=mapped, 4=io_memory)
pub fn host_register(ptr: *mut c_void, size: usize, flags: u32) -> CudaResult<()> {
    unsafe { check(cudaHostRegister(ptr, size, flags)) }
}

/// Unregister previously registered pinned memory
pub fn host_unregister(ptr: *mut c_void) -> CudaResult<()> {
    unsafe { check(cudaHostUnregister(ptr)) }
}

/// Get device pointer for mapped pinned memory
pub fn host_get_device_pointer(host_ptr: *mut c_void) -> CudaResult<*mut c_void> {
    let mut dev_ptr = ptr::null_mut();
    unsafe { check(cudaHostGetDevicePointer(&mut dev_ptr, host_ptr, 0))? };
    Ok(dev_ptr)
}

// =============================================================================
// Async Memory Operations
// =============================================================================

/// Async memory copy (non-blocking, uses stream)
pub fn memcpy_async(
    dst: *mut c_void,
    src: *const c_void,
    size: usize,
    kind: CudaMemcpyKind,
    stream: CudaStream,
) -> CudaResult<()> {
    unsafe { check(cudaMemcpyAsync(dst, src, size, kind as i32, stream)) }
}

/// Async memset (non-blocking, uses stream)
pub fn memset_async(
    ptr: *mut c_void,
    value: i32,
    size: usize,
    stream: CudaStream,
) -> CudaResult<()> {
    unsafe { check(cudaMemsetAsync(ptr, value, size, stream)) }
}

// =============================================================================
// Multi-GPU Peer Access
// =============================================================================

/// Check if peer access is possible between two devices
pub fn device_can_access_peer(device: i32, peer_device: i32) -> CudaResult<bool> {
    let mut can_access = 0;
    unsafe {
        check(cudaDeviceCanAccessPeer(
            &mut can_access,
            device,
            peer_device,
        ))?
    };
    Ok(can_access != 0)
}

/// Enable peer access from current device to peer device
pub fn device_enable_peer_access(peer_device: i32) -> CudaResult<()> {
    unsafe { check(cudaDeviceEnablePeerAccess(peer_device, 0)) }
}

/// Disable peer access from current device to peer device
pub fn device_disable_peer_access(peer_device: i32) -> CudaResult<()> {
    unsafe { check(cudaDeviceDisablePeerAccess(peer_device)) }
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
    fn test_get_device_count() {
        // Works on machines with CUDA, gracefully fails without
        match get_device_count() {
            Ok(count) => assert!(count >= 0),
            Err(e) => println!("CUDA not available: {}", e),
        }
    }
}
