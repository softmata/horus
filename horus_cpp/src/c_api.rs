//! C-compatible API for calling from C++ without CXX bridge.
//!
//! These are `#[no_mangle] extern "C"` wrappers around the FFI functions,
//! making them callable from any C/C++ code that links against libhorus_cpp.
//!
//! This is a temporary bridge until the full CXX integration is wired.

use std::ffi::CStr;
use std::os::raw::c_char;

use crate::scheduler_ffi;
use crate::types_ffi::FfiScheduler;

// ─── Scheduler C API ─────────────────────────────────────────────────────────

/// Create a new scheduler. Caller owns the returned pointer.
/// Free with `horus_scheduler_destroy`.
#[no_mangle]
pub extern "C" fn horus_scheduler_new() -> *mut FfiScheduler {
    Box::into_raw(scheduler_ffi::scheduler_new())
}

/// Destroy a scheduler created by `horus_scheduler_new`.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_destroy(sched: *mut FfiScheduler) {
    if !sched.is_null() {
        drop(Box::from_raw(sched));
    }
}

/// Set tick rate in Hz.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_tick_rate(sched: *mut FfiScheduler, hz: f64) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_tick_rate(s, hz);
    }
}

/// Prefer real-time scheduling.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_prefer_rt(sched: *mut FfiScheduler) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_prefer_rt(s);
    }
}

/// Stop the scheduler.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_stop(sched: *const FfiScheduler) {
    if let Some(s) = sched.as_ref() {
        scheduler_ffi::scheduler_stop(s);
    }
}

/// Check if running.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_is_running(sched: *const FfiScheduler) -> bool {
    sched.as_ref().map_or(false, |s| scheduler_ffi::scheduler_is_running(s))
}

/// Execute a single tick. Returns 0 on success, -1 on error.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_tick_once(sched: *mut FfiScheduler) -> i32 {
    match sched.as_mut() {
        Some(s) => match scheduler_ffi::scheduler_tick_once(s) {
            Ok(()) => 0,
            Err(_) => -1,
        },
        None => -1,
    }
}

/// Get number of registered nodes.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_node_count(sched: *const FfiScheduler) -> u32 {
    sched.as_ref().map_or(0, |s| scheduler_ffi::scheduler_node_list(s).len() as u32)
}

/// Get node name at index. Writes to buf, returns length. -1 if index out of range.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_node_name_at(
    sched: *const FfiScheduler,
    index: u32,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    let Some(s) = sched.as_ref() else { return -1 };
    let nodes = scheduler_ffi::scheduler_node_list(s);
    let idx = index as usize;
    if idx >= nodes.len() { return -1; }
    let name = &nodes[idx];
    let bytes = name.as_bytes();
    let copy_len = bytes.len().min(buf_len.saturating_sub(1));
    if !buf.is_null() && buf_len > 0 {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), buf, copy_len);
        *buf.add(copy_len) = 0;
    }
    copy_len as i32
}

/// Get ABI version.
#[no_mangle]
pub extern "C" fn horus_get_abi_version() -> u32 {
    crate::types_ffi::HORUS_CPP_ABI_VERSION
}

// ─── Node C API ──────────────────────────────────────────────────────────────

use crate::types_ffi::FfiNodeBuilder;
use crate::node_ffi;

/// Create a node builder. Caller owns the returned pointer.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_new(name: *const c_char) -> *mut FfiNodeBuilder {
    let name = CStr::from_ptr(name).to_str().unwrap_or("unnamed");
    Box::into_raw(node_ffi::node_builder_new(name))
}

/// Set node tick rate.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_rate(builder: *mut FfiNodeBuilder, hz: f64) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_rate(b, hz);
    }
}

/// Set tick callback.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_set_tick(
    builder: *mut FfiNodeBuilder,
    callback: extern "C" fn(),
) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_set_tick(b, callback);
    }
}

/// Build the node and add to scheduler. Returns 0 on success, -1 on error.
/// Consumes the builder pointer (do not use after calling).
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_build(
    builder: *mut FfiNodeBuilder,
    sched: *mut FfiScheduler,
) -> i32 {
    if builder.is_null() || sched.is_null() {
        return -1;
    }
    let builder = Box::from_raw(builder);
    let sched = &mut *sched;
    match node_ffi::node_builder_build(builder, sched) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

// ─── Scheduler Query C API ───────────────────────────────────────────────────

/// Get scheduler name. Writes into buffer, returns length. -1 on error.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_get_name(
    sched: *const FfiScheduler,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    let Some(s) = sched.as_ref() else { return -1 };
    let name = scheduler_ffi::scheduler_get_name(s);
    let bytes = name.as_bytes();
    let copy_len = bytes.len().min(buf_len.saturating_sub(1));
    if !buf.is_null() && buf_len > 0 {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), buf, copy_len);
        *buf.add(copy_len) = 0; // null terminate
    }
    copy_len as i32
}

/// Check if full RT available.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_has_full_rt(sched: *const FfiScheduler) -> bool {
    sched.as_ref().map_or(false, |s| scheduler_ffi::scheduler_has_full_rt(s))
}

/// Set blackbox size in MB.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_blackbox(sched: *mut FfiScheduler, size_mb: usize) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_blackbox(s, size_mb);
    }
}

/// Enable network replication.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_enable_network(sched: *mut FfiScheduler) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_enable_network(s);
    }
}

// ─── Additional Node Builder C API ───────────────────────────────────────────

/// Set async_io execution class.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_async_io(builder: *mut FfiNodeBuilder) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_async_io(b);
    }
}

/// Set event-triggered execution on a topic.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_on_topic(builder: *mut FfiNodeBuilder, topic: *const c_char) {
    if let (Some(b), Ok(t)) = (builder.as_mut(), CStr::from_ptr(topic).to_str()) {
        node_ffi::node_builder_on_topic(b, t);
    }
}

/// Pin to CPU core.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_pin_core(builder: *mut FfiNodeBuilder, cpu_id: usize) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_pin_core(b, cpu_id);
    }
}

/// Set OS priority.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_priority(builder: *mut FfiNodeBuilder, prio: i32) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_priority(b, prio);
    }
}

/// Set per-node watchdog.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_watchdog(builder: *mut FfiNodeBuilder, timeout_us: u64) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_watchdog(b, timeout_us);
    }
}

// ─── Additional Scheduler C API ──────────────────────────────────────────────

/// Set scheduler name.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_name(sched: *mut FfiScheduler, name: *const c_char) {
    if let (Some(s), Ok(n)) = (sched.as_mut(), CStr::from_ptr(name).to_str()) {
        scheduler_ffi::scheduler_name(s, n);
    }
}

/// Enable deterministic mode.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_deterministic(sched: *mut FfiScheduler, enabled: bool) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_deterministic(s, enabled);
    }
}

/// Set watchdog timeout in microseconds.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_watchdog(sched: *mut FfiScheduler, timeout_us: u64) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_watchdog(s, timeout_us);
    }
}

/// Require real-time scheduling.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_require_rt(sched: *mut FfiScheduler) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_require_rt(s);
    }
}

/// Enable verbose logging.
#[no_mangle]
pub unsafe extern "C" fn horus_scheduler_verbose(sched: *mut FfiScheduler, enabled: bool) {
    if let Some(s) = sched.as_mut() {
        scheduler_ffi::scheduler_verbose(s, enabled);
    }
}

// ─── Additional Node Builder C API ───────────────────────────────────────────

/// Set tick budget in microseconds.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_budget(builder: *mut FfiNodeBuilder, budget_us: u64) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_budget(b, budget_us);
    }
}

/// Set deadline in microseconds.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_deadline(builder: *mut FfiNodeBuilder, deadline_us: u64) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_deadline(b, deadline_us);
    }
}

/// Set miss policy (0=Warn, 1=Skip, 2=SafeMode, 3=Stop).
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_on_miss(builder: *mut FfiNodeBuilder, policy: u8) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_on_miss(b, policy);
    }
}

/// Set execution class to Compute.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_compute(builder: *mut FfiNodeBuilder) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_compute(b);
    }
}

/// Set execution order.
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_order(builder: *mut FfiNodeBuilder, order: u32) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_order(b, order);
    }
}

// ─── Topic C API (CmdVel) ────────────────────────────────────────────────────
//
// Each message type needs its own set of C API functions.
// Starting with CmdVel — the most common type.

use crate::topic_ffi;

/// Opaque publisher handle for C.
pub struct HorusPublisher {
    _opaque: [u8; 0],
}
/// Opaque subscriber handle for C.
pub struct HorusSubscriber {
    _opaque: [u8; 0],
}

/// CmdVel message layout matching Rust #[repr(C)].
#[repr(C)]
pub struct HorusCmdVel {
    pub timestamp_ns: u64,
    pub linear: f32,
    pub angular: f32,
}

/// Create CmdVel publisher. Returns null on error.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_cmd_vel_new(name: *const c_char) -> *mut HorusPublisher {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    match topic_ffi::publisher_cmd_vel_new(name) {
        Ok(pub_) => Box::into_raw(pub_) as *mut HorusPublisher,
        Err(_) => std::ptr::null_mut(),
    }
}

/// Destroy CmdVel publisher.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_cmd_vel_destroy(pub_: *mut HorusPublisher) {
    if !pub_.is_null() {
        drop(Box::from_raw(pub_ as *mut topic_ffi::FfiPublisher<horus_library::CmdVel>));
    }
}

/// Send CmdVel message.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_cmd_vel_send(
    pub_: *const HorusPublisher,
    msg: *const HorusCmdVel,
) {
    if pub_.is_null() || msg.is_null() { return; }
    let pub_ = &*(pub_ as *const topic_ffi::FfiPublisher<horus_library::CmdVel>);
    let msg = &*msg;
    topic_ffi::publisher_cmd_vel_send(pub_, horus_library::CmdVel {
        timestamp_ns: msg.timestamp_ns,
        linear: msg.linear,
        angular: msg.angular,
    });
}

/// Create CmdVel subscriber. Returns null on error.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_cmd_vel_new(name: *const c_char) -> *mut HorusSubscriber {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    match topic_ffi::subscriber_cmd_vel_new(name) {
        Ok(sub) => Box::into_raw(sub) as *mut HorusSubscriber,
        Err(_) => std::ptr::null_mut(),
    }
}

/// Destroy CmdVel subscriber.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_cmd_vel_destroy(sub: *mut HorusSubscriber) {
    if !sub.is_null() {
        drop(Box::from_raw(sub as *mut topic_ffi::FfiSubscriber<horus_library::CmdVel>));
    }
}

/// Receive CmdVel message. Returns 1 if message received, 0 if empty.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_cmd_vel_recv(
    sub: *const HorusSubscriber,
    out: *mut HorusCmdVel,
) -> i32 {
    if sub.is_null() || out.is_null() { return 0; }
    let sub = &*(sub as *const topic_ffi::FfiSubscriber<horus_library::CmdVel>);
    match topic_ffi::subscriber_cmd_vel_recv(sub) {
        Some(msg) => {
            (*out).timestamp_ns = msg.timestamp_ns;
            (*out).linear = msg.linear;
            (*out).angular = msg.angular;
            1
        }
        None => 0,
    }
}

/// Check if CmdVel message available.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_cmd_vel_has_msg(sub: *const HorusSubscriber) -> bool {
    if sub.is_null() { return false; }
    let sub = &*(sub as *const topic_ffi::FfiSubscriber<horus_library::CmdVel>);
    topic_ffi::subscriber_cmd_vel_has_msg(sub)
}

// ─── JsonWireMessage Topic C API ─────────────────────────────────────────────

/// JsonWireMessage layout for C — matches Rust #[repr(C)].
#[repr(C)]
pub struct HorusJsonWireMsg {
    pub data: [u8; 3968],
    pub data_len: u32,
    pub msg_id: u64,
    pub msg_type: u8,
    pub _padding: [u8; 11],
}

/// Create JsonWireMessage publisher.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_json_wire_new(name: *const c_char) -> *mut HorusPublisher {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    match topic_ffi::publisher_json_wire_new(name) {
        Ok(pub_) => Box::into_raw(pub_) as *mut HorusPublisher,
        Err(_) => std::ptr::null_mut(),
    }
}

/// Destroy JsonWireMessage publisher.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_json_wire_destroy(pub_: *mut HorusPublisher) {
    if !pub_.is_null() {
        drop(Box::from_raw(pub_ as *mut topic_ffi::FfiPublisher<crate::types_ffi::JsonWireMessage>));
    }
}

/// Send JsonWireMessage.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_json_wire_send(
    pub_: *const HorusPublisher,
    msg: *const HorusJsonWireMsg,
) {
    if pub_.is_null() || msg.is_null() { return; }
    let pub_ = &*(pub_ as *const topic_ffi::FfiPublisher<crate::types_ffi::JsonWireMessage>);
    // Direct memcpy — both are #[repr(C)] with identical layout
    let wire: crate::types_ffi::JsonWireMessage = std::ptr::read(msg as *const crate::types_ffi::JsonWireMessage);
    topic_ffi::publisher_json_wire_send(pub_, wire);
}

/// Create JsonWireMessage subscriber.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_json_wire_new(name: *const c_char) -> *mut HorusSubscriber {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    match topic_ffi::subscriber_json_wire_new(name) {
        Ok(sub) => Box::into_raw(sub) as *mut HorusSubscriber,
        Err(_) => std::ptr::null_mut(),
    }
}

/// Destroy JsonWireMessage subscriber.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_json_wire_destroy(sub: *mut HorusSubscriber) {
    if !sub.is_null() {
        drop(Box::from_raw(sub as *mut topic_ffi::FfiSubscriber<crate::types_ffi::JsonWireMessage>));
    }
}

/// Receive JsonWireMessage. Returns 1 if received, 0 if empty.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_json_wire_recv(
    sub: *const HorusSubscriber,
    out: *mut HorusJsonWireMsg,
) -> i32 {
    if sub.is_null() || out.is_null() { return 0; }
    let sub = &*(sub as *const topic_ffi::FfiSubscriber<crate::types_ffi::JsonWireMessage>);
    match topic_ffi::subscriber_json_wire_recv(sub) {
        Some(msg) => {
            std::ptr::write(out as *mut crate::types_ffi::JsonWireMessage, msg);
            1
        }
        None => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CString;

    #[test]
    fn c_api_scheduler_lifecycle() {
        unsafe {
            let sched = horus_scheduler_new();
            assert!(!sched.is_null());
            assert!(horus_scheduler_is_running(sched));

            horus_scheduler_tick_rate(sched, 100.0);
            horus_scheduler_prefer_rt(sched);

            assert_eq!(horus_scheduler_tick_once(sched), 0);

            horus_scheduler_stop(sched);
            assert!(!horus_scheduler_is_running(sched));

            horus_scheduler_destroy(sched);
        }
    }

    #[test]
    fn c_api_node_builder() {
        unsafe {
            let sched = horus_scheduler_new();
            let name = CString::new("test_node").unwrap();
            let builder = horus_node_builder_new(name.as_ptr());
            assert!(!builder.is_null());

            horus_node_builder_rate(builder, 50.0);

            extern "C" fn noop() {}
            horus_node_builder_set_tick(builder, noop);

            assert_eq!(horus_node_builder_build(builder, sched), 0);

            // Tick should invoke the node
            assert_eq!(horus_scheduler_tick_once(sched), 0);

            horus_scheduler_destroy(sched);
        }
    }

    #[test]
    fn c_api_abi_version() {
        assert_eq!(horus_get_abi_version(), crate::types_ffi::HORUS_CPP_ABI_VERSION);
    }

    #[test]
    fn c_api_null_safety() {
        unsafe {
            // All functions should handle null gracefully
            horus_scheduler_destroy(std::ptr::null_mut());
            horus_scheduler_tick_rate(std::ptr::null_mut(), 100.0);
            horus_scheduler_stop(std::ptr::null());
            assert!(!horus_scheduler_is_running(std::ptr::null()));
            assert_eq!(horus_scheduler_tick_once(std::ptr::null_mut()), -1);
        }
    }
}
