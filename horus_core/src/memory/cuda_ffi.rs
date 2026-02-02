#![allow(clippy::not_unsafe_ptr_arg_deref)]
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

/// Opaque CUDA stream handle
pub type CudaStream = *mut c_void;

/// Opaque CUDA event handle
pub type CudaEvent = *mut c_void;

/// Stream creation flags
#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum CudaStreamFlags {
    Default = 0,
    NonBlocking = 1,
}

/// Event creation flags
#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum CudaEventFlags {
    Default = 0,
    BlockingSync = 1,
    DisableTiming = 2,
    Interprocess = 4,
}

/// Host register flags for pinned memory
#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum CudaHostRegisterFlags {
    Default = 0,
    Portable = 1,
    Mapped = 2,
    IoMemory = 4,
}

/// Peer access flags
#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum CudaPeerAccessFlags {
    Default = 0,
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

    // Stream management
    fn cudaStreamCreate(stream: *mut CudaStream) -> i32;
    fn cudaStreamCreateWithFlags(stream: *mut CudaStream, flags: u32) -> i32;
    fn cudaStreamDestroy(stream: CudaStream) -> i32;
    fn cudaStreamSynchronize(stream: CudaStream) -> i32;
    fn cudaStreamQuery(stream: CudaStream) -> i32;

    // Event management
    fn cudaEventCreate(event: *mut CudaEvent) -> i32;
    fn cudaEventCreateWithFlags(event: *mut CudaEvent, flags: u32) -> i32;
    fn cudaEventDestroy(event: CudaEvent) -> i32;
    fn cudaEventRecord(event: CudaEvent, stream: CudaStream) -> i32;
    fn cudaEventSynchronize(event: CudaEvent) -> i32;
    fn cudaEventQuery(event: CudaEvent) -> i32;
    fn cudaEventElapsedTime(ms: *mut f32, start: CudaEvent, end: CudaEvent) -> i32;
    fn cudaStreamWaitEvent(stream: CudaStream, event: CudaEvent, flags: u32) -> i32;

    // Pinned (page-locked) host memory
    fn cudaMallocHost(ptr: *mut *mut c_void, size: usize) -> i32;
    fn cudaFreeHost(ptr: *mut c_void) -> i32;
    fn cudaHostRegister(ptr: *mut c_void, size: usize, flags: u32) -> i32;
    fn cudaHostUnregister(ptr: *mut c_void) -> i32;
    fn cudaHostGetDevicePointer(
        dev_ptr: *mut *mut c_void,
        host_ptr: *mut c_void,
        flags: u32,
    ) -> i32;

    // Async memory operations
    fn cudaMemcpyAsync(
        dst: *mut c_void,
        src: *const c_void,
        count: usize,
        kind: i32,
        stream: CudaStream,
    ) -> i32;
    fn cudaMemsetAsync(dev_ptr: *mut c_void, value: i32, count: usize, stream: CudaStream) -> i32;

    // Multi-GPU peer access
    fn cudaDeviceCanAccessPeer(can_access: *mut i32, device: i32, peer_device: i32) -> i32;
    fn cudaDeviceEnablePeerAccess(peer_device: i32, flags: u32) -> i32;
    fn cudaDeviceDisablePeerAccess(peer_device: i32) -> i32;
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
    Err(CudaError::NotSupported)
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

// ============================================================================
// Stream Management
// ============================================================================

/// Create a new CUDA stream
#[cfg(feature = "cuda")]
pub fn stream_create() -> CudaResult<CudaStream> {
    let mut stream: CudaStream = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaStreamCreate(&mut stream));
        if err.is_success() {
            Ok(stream)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn stream_create() -> CudaResult<CudaStream> {
    Err(CudaError::NotSupported)
}

/// Create a CUDA stream with flags
#[cfg(feature = "cuda")]
pub fn stream_create_with_flags(flags: CudaStreamFlags) -> CudaResult<CudaStream> {
    let mut stream: CudaStream = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaStreamCreateWithFlags(&mut stream, flags as u32));
        if err.is_success() {
            Ok(stream)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn stream_create_with_flags(_flags: CudaStreamFlags) -> CudaResult<CudaStream> {
    Err(CudaError::NotSupported)
}

/// Destroy a CUDA stream
#[cfg(feature = "cuda")]
pub fn stream_destroy(stream: CudaStream) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaStreamDestroy(stream));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn stream_destroy(_stream: CudaStream) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Synchronize a CUDA stream (wait for all operations to complete)
#[cfg(feature = "cuda")]
pub fn stream_synchronize(stream: CudaStream) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaStreamSynchronize(stream));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn stream_synchronize(_stream: CudaStream) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Query if stream operations are complete (non-blocking)
#[cfg(feature = "cuda")]
pub fn stream_query(stream: CudaStream) -> CudaResult<bool> {
    unsafe {
        let err = CudaError::from_code(cudaStreamQuery(stream));
        match err {
            CudaError::Success => Ok(true),
            CudaError::NotReady => Ok(false),
            _ => Err(err),
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn stream_query(_stream: CudaStream) -> CudaResult<bool> {
    Err(CudaError::NotSupported)
}

// ============================================================================
// Event Management
// ============================================================================

/// Create a new CUDA event
#[cfg(feature = "cuda")]
pub fn event_create() -> CudaResult<CudaEvent> {
    let mut event: CudaEvent = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaEventCreate(&mut event));
        if err.is_success() {
            Ok(event)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn event_create() -> CudaResult<CudaEvent> {
    Err(CudaError::NotSupported)
}

/// Create a CUDA event with flags
#[cfg(feature = "cuda")]
pub fn event_create_with_flags(flags: CudaEventFlags) -> CudaResult<CudaEvent> {
    let mut event: CudaEvent = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaEventCreateWithFlags(&mut event, flags as u32));
        if err.is_success() {
            Ok(event)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn event_create_with_flags(_flags: CudaEventFlags) -> CudaResult<CudaEvent> {
    Err(CudaError::NotSupported)
}

/// Destroy a CUDA event
#[cfg(feature = "cuda")]
pub fn event_destroy(event: CudaEvent) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaEventDestroy(event));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn event_destroy(_event: CudaEvent) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Record an event on a stream
#[cfg(feature = "cuda")]
pub fn event_record(event: CudaEvent, stream: CudaStream) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaEventRecord(event, stream));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn event_record(_event: CudaEvent, _stream: CudaStream) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Synchronize on an event (wait for it to complete)
#[cfg(feature = "cuda")]
pub fn event_synchronize(event: CudaEvent) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaEventSynchronize(event));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn event_synchronize(_event: CudaEvent) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Query if event has completed (non-blocking)
#[cfg(feature = "cuda")]
pub fn event_query(event: CudaEvent) -> CudaResult<bool> {
    unsafe {
        let err = CudaError::from_code(cudaEventQuery(event));
        match err {
            CudaError::Success => Ok(true),
            CudaError::NotReady => Ok(false),
            _ => Err(err),
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn event_query(_event: CudaEvent) -> CudaResult<bool> {
    Err(CudaError::NotSupported)
}

/// Get elapsed time between two events in milliseconds
#[cfg(feature = "cuda")]
pub fn event_elapsed_time(start: CudaEvent, end: CudaEvent) -> CudaResult<f32> {
    let mut ms: f32 = 0.0;
    unsafe {
        let err = CudaError::from_code(cudaEventElapsedTime(&mut ms, start, end));
        if err.is_success() {
            Ok(ms)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn event_elapsed_time(_start: CudaEvent, _end: CudaEvent) -> CudaResult<f32> {
    Err(CudaError::NotSupported)
}

/// Make a stream wait for an event
#[cfg(feature = "cuda")]
pub fn stream_wait_event(stream: CudaStream, event: CudaEvent) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaStreamWaitEvent(stream, event, 0));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn stream_wait_event(_stream: CudaStream, _event: CudaEvent) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

// ============================================================================
// Pinned (Page-Locked) Host Memory
// ============================================================================

/// Allocate pinned (page-locked) host memory for faster CPU↔GPU transfers
#[cfg(feature = "cuda")]
pub fn malloc_host(size: usize) -> CudaResult<*mut c_void> {
    let mut ptr: *mut c_void = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaMallocHost(&mut ptr, size));
        if err.is_success() {
            Ok(ptr)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn malloc_host(_size: usize) -> CudaResult<*mut c_void> {
    Err(CudaError::NotSupported)
}

/// Free pinned host memory
#[cfg(feature = "cuda")]
pub fn free_host(ptr: *mut c_void) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaFreeHost(ptr));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn free_host(_ptr: *mut c_void) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Register existing host memory as pinned (page-lock it)
#[cfg(feature = "cuda")]
pub fn host_register(
    ptr: *mut c_void,
    size: usize,
    flags: CudaHostRegisterFlags,
) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaHostRegister(ptr, size, flags as u32));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn host_register(
    _ptr: *mut c_void,
    _size: usize,
    _flags: CudaHostRegisterFlags,
) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Unregister previously registered pinned memory
#[cfg(feature = "cuda")]
pub fn host_unregister(ptr: *mut c_void) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaHostUnregister(ptr));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn host_unregister(_ptr: *mut c_void) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Get device pointer for mapped pinned memory
#[cfg(feature = "cuda")]
pub fn host_get_device_pointer(host_ptr: *mut c_void) -> CudaResult<*mut c_void> {
    let mut dev_ptr: *mut c_void = ptr::null_mut();
    unsafe {
        let err = CudaError::from_code(cudaHostGetDevicePointer(&mut dev_ptr, host_ptr, 0));
        if err.is_success() {
            Ok(dev_ptr)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn host_get_device_pointer(_host_ptr: *mut c_void) -> CudaResult<*mut c_void> {
    Err(CudaError::NotSupported)
}

// ============================================================================
// Async Memory Operations
// ============================================================================

/// Async memory copy (non-blocking, uses stream)
#[cfg(feature = "cuda")]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn memcpy_async(
    dst: *mut c_void,
    src: *const c_void,
    size: usize,
    kind: CudaMemcpyKind,
    stream: CudaStream,
) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaMemcpyAsync(dst, src, size, kind as i32, stream));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn memcpy_async(
    _dst: *mut c_void,
    _src: *const c_void,
    _size: usize,
    _kind: CudaMemcpyKind,
    _stream: CudaStream,
) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Async memset (non-blocking, uses stream)
#[cfg(feature = "cuda")]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn memset_async(
    ptr: *mut c_void,
    value: i32,
    size: usize,
    stream: CudaStream,
) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaMemsetAsync(ptr, value, size, stream));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn memset_async(
    _ptr: *mut c_void,
    _value: i32,
    _size: usize,
    _stream: CudaStream,
) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

// ============================================================================
// Multi-GPU Peer Access
// ============================================================================

/// Check if peer access is possible between two devices
#[cfg(feature = "cuda")]
pub fn device_can_access_peer(device: i32, peer_device: i32) -> CudaResult<bool> {
    let mut can_access: i32 = 0;
    unsafe {
        let err = CudaError::from_code(cudaDeviceCanAccessPeer(
            &mut can_access,
            device,
            peer_device,
        ));
        if err.is_success() {
            Ok(can_access != 0)
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn device_can_access_peer(_device: i32, _peer_device: i32) -> CudaResult<bool> {
    Err(CudaError::NotSupported)
}

/// Enable peer access from current device to peer device
#[cfg(feature = "cuda")]
pub fn device_enable_peer_access(peer_device: i32) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaDeviceEnablePeerAccess(peer_device, 0));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn device_enable_peer_access(_peer_device: i32) -> CudaResult<()> {
    Err(CudaError::NotSupported)
}

/// Disable peer access from current device to peer device
#[cfg(feature = "cuda")]
pub fn device_disable_peer_access(peer_device: i32) -> CudaResult<()> {
    unsafe {
        let err = CudaError::from_code(cudaDeviceDisablePeerAccess(peer_device));
        if err.is_success() {
            Ok(())
        } else {
            Err(err)
        }
    }
}

#[cfg(not(feature = "cuda"))]
pub fn device_disable_peer_access(_peer_device: i32) -> CudaResult<()> {
    Err(CudaError::NotSupported)
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
