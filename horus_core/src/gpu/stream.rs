//! CUDA stream and event wrappers for GPU execution scheduling.
//!
//! Streams allow overlapping GPU kernel execution. Events provide
//! GPU-side timing and synchronization between streams.

use super::cuda_ffi::{self, CudaError, CudaResult};
use std::ffi::{c_int, c_uint, c_void};
use std::sync::OnceLock;
use std::time::Duration;

// CUDA handle types (opaque pointers)
type CUstream = *mut c_void;
type CUevent = *mut c_void;
type CUresult = c_int;

// Event flags
const CU_EVENT_DEFAULT: c_uint = 0x0;

/// Stream/Event API function pointers, loaded once from libcuda.so.
struct StreamApi {
    cu_stream_create: unsafe extern "C" fn(*mut CUstream, c_uint) -> CUresult,
    cu_stream_destroy_v2: unsafe extern "C" fn(CUstream) -> CUresult,
    cu_stream_synchronize: unsafe extern "C" fn(CUstream) -> CUresult,
    cu_event_create: unsafe extern "C" fn(*mut CUevent, c_uint) -> CUresult,
    cu_event_destroy_v2: unsafe extern "C" fn(CUevent) -> CUresult,
    cu_event_record: unsafe extern "C" fn(CUevent, CUstream) -> CUresult,
    cu_event_synchronize: unsafe extern "C" fn(CUevent) -> CUresult,
    cu_event_elapsed_time: unsafe extern "C" fn(*mut f32, CUevent, CUevent) -> CUresult,
    cu_stream_wait_event: unsafe extern "C" fn(CUstream, CUevent, c_uint) -> CUresult,
}

unsafe impl Send for StreamApi {}
unsafe impl Sync for StreamApi {}

static STREAM_API: OnceLock<Result<StreamApi, String>> = OnceLock::new();

fn load_stream_api() -> Result<StreamApi, String> {
    let lib_name = std::ffi::CString::new("libcuda.so.1").unwrap();
    let handle = unsafe { libc::dlopen(lib_name.as_ptr(), libc::RTLD_NOW | libc::RTLD_GLOBAL) };
    if handle.is_null() {
        return Err("libcuda.so.1 not available for stream API".to_string());
    }

    unsafe {
        Ok(StreamApi {
            cu_stream_create: load_sym(handle, "cuStreamCreate")?,
            cu_stream_destroy_v2: load_sym(handle, "cuStreamDestroy_v2")?,
            cu_stream_synchronize: load_sym(handle, "cuStreamSynchronize")?,
            cu_event_create: load_sym(handle, "cuEventCreate")?,
            cu_event_destroy_v2: load_sym(handle, "cuEventDestroy_v2")?,
            cu_event_record: load_sym(handle, "cuEventRecord")?,
            cu_event_synchronize: load_sym(handle, "cuEventSynchronize")?,
            cu_event_elapsed_time: load_sym(handle, "cuEventElapsedTime_v2")?,
            cu_stream_wait_event: load_sym(handle, "cuStreamWaitEvent")?,
        })
    }
}

unsafe fn load_sym<T>(handle: *mut c_void, name: &str) -> Result<T, String> {
    let cname = std::ffi::CString::new(name).unwrap();
    let ptr = libc::dlsym(handle, cname.as_ptr());
    if ptr.is_null() {
        return Err(format!("symbol '{}' not found in libcuda.so", name));
    }
    Ok(std::mem::transmute_copy(&ptr))
}

fn api() -> CudaResult<&'static StreamApi> {
    STREAM_API
        .get_or_init(load_stream_api)
        .as_ref()
        .map_err(|msg| CudaError {
            code: -1,
            message: msg.clone(),
        })
}

fn check(code: CUresult, context: &str) -> CudaResult<()> {
    if code == 0 {
        Ok(())
    } else {
        Err(CudaError {
            code,
            message: format!("{} (CUresult={})", context, code),
        })
    }
}

// ---------------------------------------------------------------------------
// CudaStream — RAII wrapper around a CUDA stream
// ---------------------------------------------------------------------------

/// A CUDA stream for asynchronous kernel execution.
///
/// Kernels launched on the same stream execute sequentially.
/// Kernels on different streams may execute concurrently.
/// Use [`CudaEvent`] for inter-stream synchronization.
pub struct CudaStream {
    stream: CUstream,
}

// SAFETY: CUDA streams are thread-safe — kernel launches from any thread
// are serialized by the driver within the stream.
unsafe impl Send for CudaStream {}
unsafe impl Sync for CudaStream {}

impl CudaStream {
    /// Create a new CUDA stream.
    pub fn new() -> CudaResult<Self> {
        let api = api()?;
        let mut stream: CUstream = std::ptr::null_mut();
        check(
            unsafe { (api.cu_stream_create)(&mut stream, 0) },
            "cuStreamCreate",
        )?;
        Ok(Self { stream })
    }

    /// Synchronize — block until all operations on this stream complete.
    pub fn synchronize(&self) -> CudaResult<()> {
        let api = api()?;
        check(
            unsafe { (api.cu_stream_synchronize)(self.stream) },
            "cuStreamSynchronize",
        )
    }

    /// Record an event on this stream.
    ///
    /// The event is "recorded" at the current point in the stream's
    /// execution queue. It completes when all preceding operations finish.
    pub fn record_event(&self) -> CudaResult<CudaEvent> {
        let event = CudaEvent::new()?;
        event.record(self)?;
        Ok(event)
    }

    /// Make this stream wait for an event from another stream.
    ///
    /// All future operations on this stream will wait until the event
    /// is complete. This enables inter-stream dependency without
    /// CPU synchronization.
    pub fn wait_for_event(&self, event: &CudaEvent) -> CudaResult<()> {
        let api = api()?;
        check(
            unsafe { (api.cu_stream_wait_event)(self.stream, event.event, 0) },
            "cuStreamWaitEvent",
        )
    }

    /// Get the raw CUDA stream handle (for passing to CUDA kernels).
    #[inline]
    pub fn raw(&self) -> *mut c_void {
        self.stream
    }
}

impl Drop for CudaStream {
    fn drop(&mut self) {
        if !self.stream.is_null() {
            if let Ok(api) = api() {
                let _ = unsafe { (api.cu_stream_destroy_v2)(self.stream) };
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CudaEvent — RAII wrapper around a CUDA event
// ---------------------------------------------------------------------------

/// A CUDA event for GPU-side timing and synchronization.
///
/// Events are recorded on a stream and can be used to:
/// 1. Measure GPU execution time between two events
/// 2. Synchronize streams (one stream waits for another's event)
/// 3. Wait on the CPU for GPU work to complete
pub struct CudaEvent {
    event: CUevent,
}

// SAFETY: CUDA events are thread-safe.
unsafe impl Send for CudaEvent {}
unsafe impl Sync for CudaEvent {}

impl CudaEvent {
    /// Create a new CUDA event.
    pub fn new() -> CudaResult<Self> {
        let api = api()?;
        let mut event: CUevent = std::ptr::null_mut();
        check(
            unsafe { (api.cu_event_create)(&mut event, CU_EVENT_DEFAULT) },
            "cuEventCreate",
        )?;
        Ok(Self { event })
    }

    /// Record this event on a stream.
    pub fn record(&self, stream: &CudaStream) -> CudaResult<()> {
        let api = api()?;
        check(
            unsafe { (api.cu_event_record)(self.event, stream.stream) },
            "cuEventRecord",
        )
    }

    /// Block the CPU until this event completes.
    pub fn synchronize(&self) -> CudaResult<()> {
        let api = api()?;
        check(
            unsafe { (api.cu_event_synchronize)(self.event) },
            "cuEventSynchronize",
        )
    }

    /// Compute elapsed time between this event (start) and another event (end).
    ///
    /// Both events must have been recorded and completed.
    pub fn elapsed_since(&self, start: &CudaEvent) -> CudaResult<Duration> {
        let api = api()?;
        let mut ms: f32 = 0.0;
        check(
            unsafe { (api.cu_event_elapsed_time)(&mut ms, start.event, self.event) },
            "cuEventElapsedTime",
        )?;
        Ok(Duration::from_secs_f64(ms as f64 / 1000.0))
    }

    /// Get the raw event handle.
    #[inline]
    pub fn raw(&self) -> *mut c_void {
        self.event
    }
}

impl Drop for CudaEvent {
    fn drop(&mut self) {
        if !self.event.is_null() {
            if let Ok(api) = api() {
                let _ = unsafe { (api.cu_event_destroy_v2)(self.event) };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpu::cuda_ffi;

    #[test]
    fn test_stream_create_destroy() {
        if !cuda_ffi::is_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();
        let stream = CudaStream::new().unwrap();
        assert!(!stream.raw().is_null());
        stream.synchronize().unwrap();
        // Drop cleans up
    }

    #[test]
    fn test_event_create_destroy() {
        if !cuda_ffi::is_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();
        let event = CudaEvent::new().unwrap();
        assert!(!event.raw().is_null());
        // Drop cleans up
    }

    #[test]
    fn test_event_record_synchronize() {
        if !cuda_ffi::is_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();
        let stream = CudaStream::new().unwrap();

        // Record event, synchronize both
        let event = stream.record_event().unwrap();
        event.synchronize().unwrap();
        stream.synchronize().unwrap();
    }

    #[test]
    fn test_event_elapsed_time() {
        if !cuda_ffi::is_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();
        let stream = CudaStream::new().unwrap();

        let start = CudaEvent::new().unwrap();
        start.record(&stream).unwrap();

        // Do some GPU work (memset a large buffer)
        let dptr = cuda_ffi::mem_alloc(64 * 1024 * 1024).unwrap(); // 64MB
        cuda_ffi::memset_d8(dptr, 0xAB, 64 * 1024 * 1024).unwrap();

        let end = CudaEvent::new().unwrap();
        end.record(&stream).unwrap();
        end.synchronize().unwrap();

        let elapsed = end.elapsed_since(&start).unwrap();
        eprintln!("GPU memset 64MB took: {:?}", elapsed);
        // Should be measurable (>0) but fast (<100ms)
        assert!(elapsed < Duration::from_millis(100));

        cuda_ffi::mem_free(dptr).unwrap();
    }

    #[test]
    fn test_stream_wait_event() {
        if !cuda_ffi::is_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();

        let stream_a = CudaStream::new().unwrap();
        let stream_b = CudaStream::new().unwrap();

        // Record event on stream A
        let event = stream_a.record_event().unwrap();

        // Stream B waits for stream A's event
        stream_b.wait_for_event(&event).unwrap();

        // Synchronize stream B — should complete (event was on empty stream)
        stream_b.synchronize().unwrap();
    }

    #[test]
    fn test_multiple_streams_independent() {
        if !cuda_ffi::is_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();

        // Create 4 streams (simulating 4 GPU nodes)
        let streams: Vec<CudaStream> = (0..4).map(|_| CudaStream::new().unwrap()).collect();

        // Each stream does independent work
        for stream in &streams {
            let _ = stream.record_event().unwrap();
        }

        // Synchronize all
        for stream in &streams {
            stream.synchronize().unwrap();
        }
    }
}
