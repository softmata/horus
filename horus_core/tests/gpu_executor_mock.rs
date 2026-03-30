//! Mock GPU executor tests — verify context injection and stream lifecycle.
//!
//! Tests run on any machine (GPU or CPU-only).

/// gpu_stream() returns None outside of tick context.
#[test]
fn test_gpu_stream_none_outside_tick() {
    let ptr = horus_core::core::tick_context::ctx_gpu_stream();
    assert!(ptr.is_null(), "gpu_stream should be null outside tick");
}

/// set_gpu_stream / clear_gpu_stream roundtrip.
#[test]
fn test_gpu_stream_set_clear_roundtrip() {
    use horus_core::core::tick_context::{clear_gpu_stream, ctx_gpu_stream, set_gpu_stream};

    assert!(ctx_gpu_stream().is_null());

    let fake_stream = 0xDEADBEEFu64 as *const std::ffi::c_void;
    set_gpu_stream(fake_stream);
    assert_eq!(ctx_gpu_stream(), fake_stream);

    clear_gpu_stream();
    assert!(ctx_gpu_stream().is_null());
}

/// Set/clear is thread-local — doesn't leak across threads.
#[test]
fn test_gpu_stream_thread_isolation() {
    use horus_core::core::tick_context::{clear_gpu_stream, ctx_gpu_stream, set_gpu_stream};

    let fake = 0x1234u64 as *const std::ffi::c_void;
    set_gpu_stream(fake);

    let handle = std::thread::spawn(|| {
        let ptr = ctx_gpu_stream();
        assert!(ptr.is_null(), "GPU stream leaked to other thread");
    });
    handle.join().unwrap();

    assert_eq!(ctx_gpu_stream(), fake);
    clear_gpu_stream();
}

/// Multiple set/clear cycles don't leak.
#[test]
fn test_gpu_stream_repeated_cycles() {
    use horus_core::core::tick_context::{clear_gpu_stream, ctx_gpu_stream, set_gpu_stream};

    for i in 1..=100u64 {
        let ptr = i as *const std::ffi::c_void;
        set_gpu_stream(ptr);
        assert_eq!(ctx_gpu_stream(), ptr);
        clear_gpu_stream();
        assert!(ctx_gpu_stream().is_null());
    }
}
