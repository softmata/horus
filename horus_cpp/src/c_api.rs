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
    sched
        .as_ref()
        .is_some_and(scheduler_ffi::scheduler_is_running)
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
    sched
        .as_ref()
        .map_or(0, |s| scheduler_ffi::scheduler_node_list(s).len() as u32)
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
    if idx >= nodes.len() {
        return -1;
    }
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

use crate::node_ffi;
use crate::types_ffi::FfiNodeBuilder;

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

/// Set init callback (called once before first tick).
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_set_init(
    builder: *mut FfiNodeBuilder,
    callback: extern "C" fn(),
) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_set_init(b, callback);
    }
}

/// Set enter_safe_state callback (called by safety monitor).
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_set_safe_state(
    builder: *mut FfiNodeBuilder,
    callback: extern "C" fn(),
) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_set_safe_state(b, callback);
    }
}

/// Set on_shutdown callback (called when scheduler stops).
#[no_mangle]
pub unsafe extern "C" fn horus_node_builder_set_shutdown(
    builder: *mut FfiNodeBuilder,
    callback: extern "C" fn(),
) {
    if let Some(b) = builder.as_mut() {
        node_ffi::node_builder_set_shutdown(b, callback);
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
    sched
        .as_ref()
        .is_some_and(scheduler_ffi::scheduler_has_full_rt)
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
pub unsafe extern "C" fn horus_node_builder_on_topic(
    builder: *mut FfiNodeBuilder,
    topic: *const c_char,
) {
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
pub unsafe extern "C" fn horus_node_builder_watchdog(
    builder: *mut FfiNodeBuilder,
    timeout_us: u64,
) {
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
pub unsafe extern "C" fn horus_node_builder_deadline(
    builder: *mut FfiNodeBuilder,
    deadline_us: u64,
) {
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
        drop(Box::from_raw(
            pub_ as *mut topic_ffi::FfiPublisher<horus_library::CmdVel>,
        ));
    }
}

/// Send CmdVel message.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_cmd_vel_send(
    pub_: *const HorusPublisher,
    msg: *const HorusCmdVel,
) {
    if pub_.is_null() || msg.is_null() {
        return;
    }
    let pub_ = &*(pub_ as *const topic_ffi::FfiPublisher<horus_library::CmdVel>);
    let msg = &*msg;
    topic_ffi::publisher_cmd_vel_send(
        pub_,
        horus_library::CmdVel {
            timestamp_ns: msg.timestamp_ns,
            linear: msg.linear,
            angular: msg.angular,
        },
    );
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
        drop(Box::from_raw(
            sub as *mut topic_ffi::FfiSubscriber<horus_library::CmdVel>,
        ));
    }
}

/// Receive CmdVel message. Returns 1 if message received, 0 if empty.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_cmd_vel_recv(
    sub: *const HorusSubscriber,
    out: *mut HorusCmdVel,
) -> i32 {
    if sub.is_null() || out.is_null() {
        return 0;
    }
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
    if sub.is_null() {
        return false;
    }
    let sub = &*(sub as *const topic_ffi::FfiSubscriber<horus_library::CmdVel>);
    topic_ffi::subscriber_cmd_vel_has_msg(sub)
}

// ─── Generic Pod Topic C API Macro ───────────────────────────────────────
//
// Generates new/destroy/send/recv/has_msg for any #[repr(C)] Pod message type.
// The C and Rust types must have identical memory layout (both #[repr(C)]).

macro_rules! impl_pod_topic_c_api {
    ($snake:ident, $rust_type:path) => {
        paste::paste! {
            #[no_mangle]
            pub unsafe extern "C" fn [<horus_publisher_ $snake _new>](
                name: *const c_char,
            ) -> *mut HorusPublisher {
                let name = match CStr::from_ptr(name).to_str() {
                    Ok(n) => n,
                    Err(_) => return std::ptr::null_mut(),
                };
                match topic_ffi::[<publisher_ $snake _new>](name) {
                    Ok(pub_) => Box::into_raw(pub_) as *mut HorusPublisher,
                    Err(_) => std::ptr::null_mut(),
                }
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<horus_publisher_ $snake _destroy>](pub_: *mut HorusPublisher) {
                if !pub_.is_null() {
                    drop(Box::from_raw(pub_ as *mut topic_ffi::FfiPublisher<$rust_type>));
                }
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<horus_publisher_ $snake _send>](
                pub_: *const HorusPublisher,
                msg: *const std::ffi::c_void,
            ) {
                if pub_.is_null() || msg.is_null() { return; }
                let pub_ = &*(pub_ as *const topic_ffi::FfiPublisher<$rust_type>);
                let msg: $rust_type = std::ptr::read(msg as *const $rust_type);
                topic_ffi::[<publisher_ $snake _send>](pub_, msg);
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<horus_subscriber_ $snake _new>](
                name: *const c_char,
            ) -> *mut HorusSubscriber {
                let name = match CStr::from_ptr(name).to_str() {
                    Ok(n) => n,
                    Err(_) => return std::ptr::null_mut(),
                };
                match topic_ffi::[<subscriber_ $snake _new>](name) {
                    Ok(sub) => Box::into_raw(sub) as *mut HorusSubscriber,
                    Err(_) => std::ptr::null_mut(),
                }
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<horus_subscriber_ $snake _destroy>](sub: *mut HorusSubscriber) {
                if !sub.is_null() {
                    drop(Box::from_raw(sub as *mut topic_ffi::FfiSubscriber<$rust_type>));
                }
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<horus_subscriber_ $snake _recv>](
                sub: *const HorusSubscriber,
                out: *mut std::ffi::c_void,
            ) -> i32 {
                if sub.is_null() || out.is_null() { return 0; }
                let sub = &*(sub as *const topic_ffi::FfiSubscriber<$rust_type>);
                match topic_ffi::[<subscriber_ $snake _recv>](sub) {
                    Some(msg) => {
                        std::ptr::write(out as *mut $rust_type, msg);
                        1
                    }
                    None => 0,
                }
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<horus_subscriber_ $snake _has_msg>](
                sub: *const HorusSubscriber,
            ) -> bool {
                if sub.is_null() { return false; }
                let sub = &*(sub as *const topic_ffi::FfiSubscriber<$rust_type>);
                topic_ffi::[<subscriber_ $snake _has_msg>](sub)
            }
        }
    };
}

// Generate C API for ALL Pod message types (55 total, excluding CmdVel which has hand-written C API)
// Core 10 (already had Rust FFI)
impl_pod_topic_c_api!(laser_scan, horus_library::LaserScan);
impl_pod_topic_c_api!(imu, horus_library::Imu);
impl_pod_topic_c_api!(odometry, horus_library::Odometry);
impl_pod_topic_c_api!(joint_state, horus_library::JointState);
impl_pod_topic_c_api!(twist, horus_library::Twist);
impl_pod_topic_c_api!(pose2d, horus_library::Pose2D);
impl_pod_topic_c_api!(transform_stamped, horus_library::TransformStamped);
impl_pod_topic_c_api!(nav_goal, horus_library::NavGoal);
impl_pod_topic_c_api!(heartbeat, horus_library::Heartbeat);
impl_pod_topic_c_api!(emergency_stop, horus_library::EmergencyStop);
// Sensor (7)
impl_pod_topic_c_api!(range_sensor, horus_library::RangeSensor);
impl_pod_topic_c_api!(battery_state, horus_library::BatteryState);
impl_pod_topic_c_api!(nav_sat_fix, horus_library::NavSatFix);
impl_pod_topic_c_api!(magnetic_field, horus_library::MagneticField);
impl_pod_topic_c_api!(temperature, horus_library::Temperature);
impl_pod_topic_c_api!(fluid_pressure, horus_library::FluidPressure);
impl_pod_topic_c_api!(illuminance, horus_library::Illuminance);
// Control (6)
impl_pod_topic_c_api!(motor_command, horus_library::MotorCommand);
impl_pod_topic_c_api!(
    differential_drive_command,
    horus_library::DifferentialDriveCommand
);
impl_pod_topic_c_api!(servo_command, horus_library::ServoCommand);
impl_pod_topic_c_api!(pid_config, horus_library::PidConfig);
impl_pod_topic_c_api!(trajectory_point, horus_library::TrajectoryPoint);
impl_pod_topic_c_api!(joint_command, horus_library::JointCommand);
// Geometry (9)
impl_pod_topic_c_api!(point3, horus_library::Point3);
impl_pod_topic_c_api!(vector3, horus_library::Vector3);
impl_pod_topic_c_api!(quaternion, horus_library::Quaternion);
impl_pod_topic_c_api!(pose3d, horus_library::Pose3D);
impl_pod_topic_c_api!(pose_stamped, horus_library::PoseStamped);
impl_pod_topic_c_api!(pose_with_covariance, horus_library::PoseWithCovariance);
impl_pod_topic_c_api!(twist_with_covariance, horus_library::TwistWithCovariance);
impl_pod_topic_c_api!(accel, horus_library::Accel);
impl_pod_topic_c_api!(accel_stamped, horus_library::AccelStamped);
// Detection (4)
impl_pod_topic_c_api!(bounding_box_2d, horus_library::BoundingBox2D);
impl_pod_topic_c_api!(bounding_box_3d, horus_library::BoundingBox3D);
impl_pod_topic_c_api!(detection, horus_library::Detection);
impl_pod_topic_c_api!(detection_3d, horus_library::Detection3D);
// Vision (3 — CompressedImage has Vec)
impl_pod_topic_c_api!(camera_info, horus_library::CameraInfo);
impl_pod_topic_c_api!(region_of_interest, horus_library::RegionOfInterest);
impl_pod_topic_c_api!(stereo_info, horus_library::StereoInfo);
// Navigation (5 — OccupancyGrid/CostMap have Vec)
impl_pod_topic_c_api!(goal_result, horus_library::GoalResult);
impl_pod_topic_c_api!(waypoint, horus_library::Waypoint);
impl_pod_topic_c_api!(nav_path, horus_library::NavPath);
impl_pod_topic_c_api!(velocity_obstacle, horus_library::VelocityObstacle);
impl_pod_topic_c_api!(path_plan, horus_library::PathPlan);
// Diagnostics (4)
impl_pod_topic_c_api!(diagnostic_status, horus_library::DiagnosticStatus);
impl_pod_topic_c_api!(resource_usage, horus_library::ResourceUsage);
impl_pod_topic_c_api!(node_heartbeat, horus_library::NodeHeartbeat);
impl_pod_topic_c_api!(safety_status, horus_library::SafetyStatus);
// Force/Tactile (5 — TactileArray has Vec)
impl_pod_topic_c_api!(wrench_stamped, horus_library::WrenchStamped);
impl_pod_topic_c_api!(force_command, horus_library::ForceCommand);
impl_pod_topic_c_api!(contact_info, horus_library::ContactInfo);
impl_pod_topic_c_api!(haptic_feedback, horus_library::HapticFeedback);
impl_pod_topic_c_api!(impedance_parameters, horus_library::ImpedanceParameters);
// Tracking/Segmentation (3)
impl_pod_topic_c_api!(tracked_object, horus_library::TrackedObject);
impl_pod_topic_c_api!(tracking_header, horus_library::TrackingHeader);
impl_pod_topic_c_api!(segmentation_mask, horus_library::SegmentationMask);
// Landmark (3)
impl_pod_topic_c_api!(landmark, horus_library::Landmark);
impl_pod_topic_c_api!(landmark_3d, horus_library::Landmark3D);
impl_pod_topic_c_api!(landmark_array, horus_library::LandmarkArray);
// Input (2)
impl_pod_topic_c_api!(keyboard_input, horus_library::KeyboardInput);
impl_pod_topic_c_api!(joystick_input, horus_library::JoystickInput);
// Audio (1)
impl_pod_topic_c_api!(audio_frame, horus_library::AudioFrame);
// Clock (2)
impl_pod_topic_c_api!(clock, horus_library::Clock);
impl_pod_topic_c_api!(time_reference, horus_library::TimeReference);
// Perception (2)
impl_pod_topic_c_api!(point_field, horus_library::PointField);
impl_pod_topic_c_api!(plane_detection, horus_library::PlaneDetection);

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
        drop(Box::from_raw(
            pub_ as *mut topic_ffi::FfiPublisher<crate::types_ffi::JsonWireMessage>,
        ));
    }
}

/// Send JsonWireMessage.
#[no_mangle]
pub unsafe extern "C" fn horus_publisher_json_wire_send(
    pub_: *const HorusPublisher,
    msg: *const HorusJsonWireMsg,
) {
    if pub_.is_null() || msg.is_null() {
        return;
    }
    let pub_ = &*(pub_ as *const topic_ffi::FfiPublisher<crate::types_ffi::JsonWireMessage>);
    // Direct memcpy — both are #[repr(C)] with identical layout
    let wire: crate::types_ffi::JsonWireMessage =
        std::ptr::read(msg as *const crate::types_ffi::JsonWireMessage);
    topic_ffi::publisher_json_wire_send(pub_, wire);
}

/// Create JsonWireMessage subscriber.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_json_wire_new(
    name: *const c_char,
) -> *mut HorusSubscriber {
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
        drop(Box::from_raw(
            sub as *mut topic_ffi::FfiSubscriber<crate::types_ffi::JsonWireMessage>,
        ));
    }
}

/// Receive JsonWireMessage. Returns 1 if received, 0 if empty.
#[no_mangle]
pub unsafe extern "C" fn horus_subscriber_json_wire_recv(
    sub: *const HorusSubscriber,
    out: *mut HorusJsonWireMsg,
) -> i32 {
    if sub.is_null() || out.is_null() {
        return 0;
    }
    let sub = &*(sub as *const topic_ffi::FfiSubscriber<crate::types_ffi::JsonWireMessage>);
    match topic_ffi::subscriber_json_wire_recv(sub) {
        Some(msg) => {
            std::ptr::write(out as *mut crate::types_ffi::JsonWireMessage, msg);
            1
        }
        None => 0,
    }
}

// ─── TensorPool C API ────────────────────────────────────────────────────────

use crate::pool_ffi;

/// Opaque TensorPool handle for C.
pub struct HorusTensorPool {
    _opaque: [u8; 0],
}
/// Opaque Tensor handle for C.
pub struct HorusTensor {
    _opaque: [u8; 0],
}
/// Opaque Image handle for C.
pub struct HorusImage {
    _opaque: [u8; 0],
}
/// Opaque PointCloud handle for C.
pub struct HorusPointCloud {
    _opaque: [u8; 0],
}

/// Create a TensorPool. Returns null on error.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_pool_new(
    pool_id: u32,
    pool_size_bytes: usize,
    max_slots: usize,
) -> *mut HorusTensorPool {
    match pool_ffi::tensor_pool_new(pool_id, pool_size_bytes, max_slots) {
        Some(pool) => Box::into_raw(pool) as *mut HorusTensorPool,
        None => std::ptr::null_mut(),
    }
}

/// Destroy a TensorPool.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_pool_destroy(pool: *mut HorusTensorPool) {
    if !pool.is_null() {
        drop(Box::from_raw(pool as *mut pool_ffi::FfiTensorPool));
    }
}

/// Get pool stats. Writes to out pointers. Returns 0 on success.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_pool_stats(
    pool: *const HorusTensorPool,
    out_allocated: *mut usize,
    out_used_bytes: *mut usize,
    out_free_bytes: *mut usize,
) -> i32 {
    if pool.is_null() {
        return -1;
    }
    let pool = &*(pool as *const pool_ffi::FfiTensorPool);
    let (alloc, _refcount, used, free) = pool_ffi::tensor_pool_stats(pool);
    if !out_allocated.is_null() {
        *out_allocated = alloc;
    }
    if !out_used_bytes.is_null() {
        *out_used_bytes = used;
    }
    if !out_free_bytes.is_null() {
        *out_free_bytes = free;
    }
    0
}

/// Allocate a Tensor. dtype: 0=f32, 1=f64, 2=u8, 3=i32. Returns null on error.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_alloc(
    pool: *const HorusTensorPool,
    shape: *const u64,
    ndim: usize,
    dtype: u8,
) -> *mut HorusTensor {
    if pool.is_null() || shape.is_null() {
        return std::ptr::null_mut();
    }
    let pool = &*(pool as *const pool_ffi::FfiTensorPool);
    let shape_slice = std::slice::from_raw_parts(shape, ndim);
    match pool_ffi::tensor_alloc(pool, shape_slice, dtype) {
        Some(tensor) => Box::into_raw(tensor) as *mut HorusTensor,
        None => std::ptr::null_mut(),
    }
}

/// Get tensor data pointer.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_data_ptr(
    pool: *const HorusTensorPool,
    tensor: *const HorusTensor,
) -> *mut u8 {
    if pool.is_null() || tensor.is_null() {
        return std::ptr::null_mut();
    }
    let pool = &*(pool as *const pool_ffi::FfiTensorPool);
    let tensor = &*(tensor as *const pool_ffi::FfiTensor);
    pool_ffi::tensor_data_ptr(pool, tensor)
}

/// Get tensor size in bytes.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_nbytes(tensor: *const HorusTensor) -> u64 {
    if tensor.is_null() {
        return 0;
    }
    let tensor = &*(tensor as *const pool_ffi::FfiTensor);
    pool_ffi::tensor_nbytes(tensor)
}

/// Release tensor back to pool.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_release(
    pool: *const HorusTensorPool,
    tensor: *const HorusTensor,
) {
    if pool.is_null() || tensor.is_null() {
        return;
    }
    let pool = &*(pool as *const pool_ffi::FfiTensorPool);
    let tensor = &*(tensor as *const pool_ffi::FfiTensor);
    pool_ffi::tensor_release(pool, tensor);
}

/// Destroy a Tensor handle.
#[no_mangle]
pub unsafe extern "C" fn horus_tensor_destroy(tensor: *mut HorusTensor) {
    if !tensor.is_null() {
        drop(Box::from_raw(tensor as *mut pool_ffi::FfiTensor));
    }
}

// ─── Image C API ─────────────────────────────────────────────────────────────

/// Allocate an Image from pool. encoding: 0=RGB8, 1=RGBA8, 2=GRAY8, 3=BGR8.
#[no_mangle]
pub unsafe extern "C" fn horus_image_new(
    pool: *const HorusTensorPool,
    width: u32,
    height: u32,
    encoding: u8,
) -> *mut HorusImage {
    if pool.is_null() {
        return std::ptr::null_mut();
    }
    let pool = &*(pool as *const pool_ffi::FfiTensorPool);
    match pool_ffi::image_new(pool, width, height, encoding) {
        Some(img) => Box::into_raw(img) as *mut HorusImage,
        None => std::ptr::null_mut(),
    }
}

/// Get image width.
#[no_mangle]
pub unsafe extern "C" fn horus_image_width(img: *const HorusImage) -> u32 {
    if img.is_null() {
        return 0;
    }
    let img = &*(img as *const pool_ffi::FfiImage);
    pool_ffi::image_width(img)
}

/// Get image height.
#[no_mangle]
pub unsafe extern "C" fn horus_image_height(img: *const HorusImage) -> u32 {
    if img.is_null() {
        return 0;
    }
    let img = &*(img as *const pool_ffi::FfiImage);
    pool_ffi::image_height(img)
}

/// Get image data size in bytes.
#[no_mangle]
pub unsafe extern "C" fn horus_image_data_size(img: *const HorusImage) -> usize {
    if img.is_null() {
        return 0;
    }
    let img = &*(img as *const pool_ffi::FfiImage);
    pool_ffi::image_data_size(img)
}

/// Destroy an Image.
#[no_mangle]
pub unsafe extern "C" fn horus_image_destroy(img: *mut HorusImage) {
    if !img.is_null() {
        drop(Box::from_raw(img as *mut pool_ffi::FfiImage));
    }
}

// ─── PointCloud C API ────────────────────────────────────────────────────────

/// Allocate a PointCloud from pool. fields_per_point: 3=XYZ, 4=XYZI, 6=XYZRGB.
#[no_mangle]
pub unsafe extern "C" fn horus_pointcloud_new(
    pool: *const HorusTensorPool,
    num_points: u32,
    fields_per_point: u32,
) -> *mut HorusPointCloud {
    if pool.is_null() {
        return std::ptr::null_mut();
    }
    let pool = &*(pool as *const pool_ffi::FfiTensorPool);
    match pool_ffi::pointcloud_new(pool, num_points, fields_per_point) {
        Some(pc) => Box::into_raw(pc) as *mut HorusPointCloud,
        None => std::ptr::null_mut(),
    }
}

/// Get point count.
#[no_mangle]
pub unsafe extern "C" fn horus_pointcloud_num_points(pc: *const HorusPointCloud) -> u64 {
    if pc.is_null() {
        return 0;
    }
    let pc = &*(pc as *const pool_ffi::FfiPointCloud);
    pool_ffi::pointcloud_num_points(pc)
}

/// Get fields per point.
#[no_mangle]
pub unsafe extern "C" fn horus_pointcloud_fields(pc: *const HorusPointCloud) -> u32 {
    if pc.is_null() {
        return 0;
    }
    let pc = &*(pc as *const pool_ffi::FfiPointCloud);
    pool_ffi::pointcloud_fields(pc)
}

/// Destroy a PointCloud.
#[no_mangle]
pub unsafe extern "C" fn horus_pointcloud_destroy(pc: *mut HorusPointCloud) {
    if !pc.is_null() {
        drop(Box::from_raw(pc as *mut pool_ffi::FfiPointCloud));
    }
}

// ─── RuntimeParams C API ─────────────────────────────────────────────────────

use crate::params_ffi;

/// Opaque Params handle for C.
pub struct HorusParams {
    _opaque: [u8; 0],
}

/// Create a new params store.
#[no_mangle]
pub extern "C" fn horus_params_new() -> *mut HorusParams {
    Box::into_raw(params_ffi::params_new()) as *mut HorusParams
}

/// Destroy params store.
#[no_mangle]
pub unsafe extern "C" fn horus_params_destroy(params: *mut HorusParams) {
    if !params.is_null() {
        drop(Box::from_raw(params as *mut params_ffi::FfiParams));
    }
}

/// Get param as f64. Returns 1 if found, 0 if not.
#[no_mangle]
pub unsafe extern "C" fn horus_params_get_f64(
    params: *const HorusParams,
    key: *const c_char,
    out: *mut f64,
) -> i32 {
    if params.is_null() || key.is_null() || out.is_null() {
        return 0;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return 0,
    };
    match params_ffi::params_get_f64(params, key) {
        Some(v) => {
            *out = v;
            1
        }
        None => 0,
    }
}

/// Set param as f64. Returns 0 on success, -1 on error.
#[no_mangle]
pub unsafe extern "C" fn horus_params_set_f64(
    params: *const HorusParams,
    key: *const c_char,
    value: f64,
) -> i32 {
    if params.is_null() || key.is_null() {
        return -1;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match params_ffi::params_set_f64(params, key, value) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Get param as i64. Returns 1 if found, 0 if not.
#[no_mangle]
pub unsafe extern "C" fn horus_params_get_i64(
    params: *const HorusParams,
    key: *const c_char,
    out: *mut i64,
) -> i32 {
    if params.is_null() || key.is_null() || out.is_null() {
        return 0;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return 0,
    };
    match params_ffi::params_get_i64(params, key) {
        Some(v) => {
            *out = v;
            1
        }
        None => 0,
    }
}

/// Set param as i64. Returns 0 on success, -1 on error.
#[no_mangle]
pub unsafe extern "C" fn horus_params_set_i64(
    params: *const HorusParams,
    key: *const c_char,
    value: i64,
) -> i32 {
    if params.is_null() || key.is_null() {
        return -1;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match params_ffi::params_set_i64(params, key, value) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Get param as bool. Returns 1 if found (writes to out), 0 if not.
#[no_mangle]
pub unsafe extern "C" fn horus_params_get_bool(
    params: *const HorusParams,
    key: *const c_char,
    out: *mut bool,
) -> i32 {
    if params.is_null() || key.is_null() || out.is_null() {
        return 0;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return 0,
    };
    match params_ffi::params_get_bool(params, key) {
        Some(v) => {
            *out = v;
            1
        }
        None => 0,
    }
}

/// Set param as bool.
#[no_mangle]
pub unsafe extern "C" fn horus_params_set_bool(
    params: *const HorusParams,
    key: *const c_char,
    value: bool,
) -> i32 {
    if params.is_null() || key.is_null() {
        return -1;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match params_ffi::params_set_bool(params, key, value) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Get param as string. Writes to buf, returns length. -1 if not found.
#[no_mangle]
pub unsafe extern "C" fn horus_params_get_string(
    params: *const HorusParams,
    key: *const c_char,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if params.is_null() || key.is_null() {
        return -1;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match params_ffi::params_get_string(params, key) {
        Some(v) => {
            let bytes = v.as_bytes();
            let copy_len = bytes.len().min(buf_len.saturating_sub(1));
            if !buf.is_null() && buf_len > 0 {
                std::ptr::copy_nonoverlapping(bytes.as_ptr(), buf, copy_len);
                *buf.add(copy_len) = 0;
            }
            copy_len as i32
        }
        None => -1,
    }
}

/// Set param as string.
#[no_mangle]
pub unsafe extern "C" fn horus_params_set_string(
    params: *const HorusParams,
    key: *const c_char,
    value: *const c_char,
) -> i32 {
    if params.is_null() || key.is_null() || value.is_null() {
        return -1;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    let value = match CStr::from_ptr(value).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match params_ffi::params_set_string(params, key, value) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Check if param exists.
#[no_mangle]
pub unsafe extern "C" fn horus_params_has(params: *const HorusParams, key: *const c_char) -> bool {
    if params.is_null() || key.is_null() {
        return false;
    }
    let params = &*(params as *const params_ffi::FfiParams);
    let key = match CStr::from_ptr(key).to_str() {
        Ok(s) => s,
        Err(_) => return false,
    };
    params_ffi::params_has(params, key)
}

// ─── Service C API ───────────────────────────────────────────────────────────

use crate::service_ffi;

/// Opaque ServiceClient handle for C.
pub struct HorusServiceClient {
    _opaque: [u8; 0],
}
/// Opaque ServiceServer handle for C.
pub struct HorusServiceServer {
    _opaque: [u8; 0],
}

/// Create a service client.
#[no_mangle]
pub unsafe extern "C" fn horus_service_client_new(name: *const c_char) -> *mut HorusServiceClient {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    Box::into_raw(service_ffi::service_client_new(name)) as *mut HorusServiceClient
}

/// Destroy a service client.
#[no_mangle]
pub unsafe extern "C" fn horus_service_client_destroy(client: *mut HorusServiceClient) {
    if !client.is_null() {
        drop(Box::from_raw(client as *mut service_ffi::FfiServiceClient));
    }
}

/// Call service with JSON request. Writes JSON response to out_buf.
/// Returns response length on success, -1 on error.
#[no_mangle]
pub unsafe extern "C" fn horus_service_client_call(
    client: *const HorusServiceClient,
    request_json: *const c_char,
    timeout_us: u64,
    out_buf: *mut u8,
    out_buf_len: usize,
) -> i32 {
    if client.is_null() || request_json.is_null() {
        return -1;
    }
    let client = &*(client as *const service_ffi::FfiServiceClient);
    let req = match CStr::from_ptr(request_json).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match service_ffi::service_client_call(client, req, timeout_us) {
        Ok(response) => {
            let bytes = response.as_bytes();
            let copy_len = bytes.len().min(out_buf_len.saturating_sub(1));
            if !out_buf.is_null() && out_buf_len > 0 {
                std::ptr::copy_nonoverlapping(bytes.as_ptr(), out_buf, copy_len);
                *out_buf.add(copy_len) = 0;
            }
            copy_len as i32
        }
        Err(_) => -1,
    }
}

/// Create a service server.
#[no_mangle]
pub unsafe extern "C" fn horus_service_server_new(name: *const c_char) -> *mut HorusServiceServer {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    Box::into_raw(service_ffi::service_server_new(name)) as *mut HorusServiceServer
}

/// Destroy a service server.
#[no_mangle]
pub unsafe extern "C" fn horus_service_server_destroy(server: *mut HorusServiceServer) {
    if !server.is_null() {
        drop(Box::from_raw(server as *mut service_ffi::FfiServiceServer));
    }
}

/// Set service server handler.
/// handler(req_ptr, req_len, res_ptr, res_len_ptr) -> bool
#[no_mangle]
pub unsafe extern "C" fn horus_service_server_set_handler(
    server: *mut HorusServiceServer,
    handler: extern "C" fn(*const u8, usize, *mut u8, *mut usize) -> bool,
) {
    if server.is_null() {
        return;
    }
    let server = &mut *(server as *mut service_ffi::FfiServiceServer);
    service_ffi::service_server_set_handler(server, handler);
}

// ─── Action C API ────────────────────────────────────────────────────────────

use crate::action_ffi;

/// Opaque ActionClient handle for C.
pub struct HorusActionClient {
    _opaque: [u8; 0],
}
/// Opaque ActionServer handle for C.
pub struct HorusActionServer {
    _opaque: [u8; 0],
}
/// Opaque GoalHandle for C.
pub struct HorusGoalHandle {
    _opaque: [u8; 0],
}

/// Create an action client.
#[no_mangle]
pub unsafe extern "C" fn horus_action_client_new(name: *const c_char) -> *mut HorusActionClient {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    Box::into_raw(action_ffi::action_client_new(name)) as *mut HorusActionClient
}

/// Destroy an action client.
#[no_mangle]
pub unsafe extern "C" fn horus_action_client_destroy(client: *mut HorusActionClient) {
    if !client.is_null() {
        drop(Box::from_raw(client as *mut action_ffi::FfiActionClient));
    }
}

/// Send a goal. Returns goal handle on success, null on error.
#[no_mangle]
pub unsafe extern "C" fn horus_action_client_send_goal(
    client: *const HorusActionClient,
    goal_json: *const c_char,
) -> *mut HorusGoalHandle {
    if client.is_null() || goal_json.is_null() {
        return std::ptr::null_mut();
    }
    let client = &*(client as *const action_ffi::FfiActionClient);
    let json = match CStr::from_ptr(goal_json).to_str() {
        Ok(s) => s,
        Err(_) => return std::ptr::null_mut(),
    };
    match action_ffi::action_client_send_goal(client, json) {
        Ok(handle) => Box::into_raw(handle) as *mut HorusGoalHandle,
        Err(_) => std::ptr::null_mut(),
    }
}

/// Cancel a goal.
#[no_mangle]
pub unsafe extern "C" fn horus_action_client_cancel(handle: *mut HorusGoalHandle) {
    if !handle.is_null() {
        let handle = &mut *(handle as *mut action_ffi::FfiGoalHandle);
        action_ffi::action_client_cancel(handle);
    }
}

/// Get goal status. Returns status code (0=Pending..5=Rejected).
#[no_mangle]
pub unsafe extern "C" fn horus_goal_handle_status(handle: *const HorusGoalHandle) -> u8 {
    if handle.is_null() {
        return 5;
    } // Rejected as safe default
    let handle = &*(handle as *const action_ffi::FfiGoalHandle);
    action_ffi::goal_handle_status(handle) as u8
}

/// Get goal ID.
#[no_mangle]
pub unsafe extern "C" fn horus_goal_handle_id(handle: *const HorusGoalHandle) -> u64 {
    if handle.is_null() {
        return 0;
    }
    let handle = &*(handle as *const action_ffi::FfiGoalHandle);
    action_ffi::goal_handle_id(handle)
}

/// Check if goal is still active.
#[no_mangle]
pub unsafe extern "C" fn horus_goal_handle_is_active(handle: *const HorusGoalHandle) -> bool {
    if handle.is_null() {
        return false;
    }
    let handle = &*(handle as *const action_ffi::FfiGoalHandle);
    action_ffi::goal_handle_is_active(handle)
}

/// Destroy a goal handle.
#[no_mangle]
pub unsafe extern "C" fn horus_goal_handle_destroy(handle: *mut HorusGoalHandle) {
    if !handle.is_null() {
        drop(Box::from_raw(handle as *mut action_ffi::FfiGoalHandle));
    }
}

/// Create an action server.
#[no_mangle]
pub unsafe extern "C" fn horus_action_server_new(name: *const c_char) -> *mut HorusActionServer {
    let name = match CStr::from_ptr(name).to_str() {
        Ok(n) => n,
        Err(_) => return std::ptr::null_mut(),
    };
    Box::into_raw(action_ffi::action_server_new(name)) as *mut HorusActionServer
}

/// Destroy an action server.
#[no_mangle]
pub unsafe extern "C" fn horus_action_server_destroy(server: *mut HorusActionServer) {
    if !server.is_null() {
        drop(Box::from_raw(server as *mut action_ffi::FfiActionServer));
    }
}

/// Set accept handler. handler(goal_bytes, goal_len) -> 0=accept, 1=reject.
#[no_mangle]
pub unsafe extern "C" fn horus_action_server_set_accept_handler(
    server: *mut HorusActionServer,
    handler: extern "C" fn(*const u8, usize) -> u8,
) {
    if server.is_null() {
        return;
    }
    let server = &mut *(server as *mut action_ffi::FfiActionServer);
    action_ffi::action_server_set_accept_handler(server, handler);
}

/// Set execute handler. handler(goal_id, goal_bytes, goal_len).
#[no_mangle]
pub unsafe extern "C" fn horus_action_server_set_execute_handler(
    server: *mut HorusActionServer,
    handler: extern "C" fn(u64, *const u8, usize),
) {
    if server.is_null() {
        return;
    }
    let server = &mut *(server as *mut action_ffi::FfiActionServer);
    action_ffi::action_server_set_execute_handler(server, handler);
}

/// Check if server is ready (both handlers set).
#[no_mangle]
pub unsafe extern "C" fn horus_action_server_is_ready(server: *const HorusActionServer) -> bool {
    if server.is_null() {
        return false;
    }
    let server = &*(server as *const action_ffi::FfiActionServer);
    action_ffi::action_server_is_ready(server)
}

// ─── TransformFrame C API ────────────────────────────────────────────────────

use crate::transform_ffi;

/// Opaque TransformFrame handle for C.
pub struct HorusTransformFrame {
    _opaque: [u8; 0],
}

/// Create a new TransformFrame.
#[no_mangle]
pub extern "C" fn horus_transform_frame_new() -> *mut HorusTransformFrame {
    Box::into_raw(transform_ffi::transform_frame_new()) as *mut HorusTransformFrame
}

/// Create with custom max frames.
#[no_mangle]
pub extern "C" fn horus_transform_frame_with_capacity(
    max_frames: usize,
) -> *mut HorusTransformFrame {
    Box::into_raw(transform_ffi::transform_frame_with_capacity(max_frames))
        as *mut HorusTransformFrame
}

/// Destroy a TransformFrame.
#[no_mangle]
pub unsafe extern "C" fn horus_transform_frame_destroy(tf: *mut HorusTransformFrame) {
    if !tf.is_null() {
        drop(Box::from_raw(tf as *mut transform_ffi::FfiTransformFrame));
    }
}

/// Register a frame. parent=null for root. Returns frame ID or -1 on error.
#[no_mangle]
pub unsafe extern "C" fn horus_transform_frame_register(
    tf: *const HorusTransformFrame,
    name: *const c_char,
    parent: *const c_char,
) -> i32 {
    if tf.is_null() || name.is_null() {
        return -1;
    }
    let tf = &*(tf as *const transform_ffi::FfiTransformFrame);
    let name = match CStr::from_ptr(name).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    let parent = if parent.is_null() {
        ""
    } else {
        match CStr::from_ptr(parent).to_str() {
            Ok(s) => s,
            Err(_) => return -1,
        }
    };
    match transform_ffi::transform_frame_register(tf, name, parent) {
        Ok(id) => id as i32,
        Err(_) => -1,
    }
}

/// Update transform. rotation is [qx, qy, qz, qw].
#[no_mangle]
pub unsafe extern "C" fn horus_transform_frame_update(
    tf: *const HorusTransformFrame,
    frame_name: *const c_char,
    tx: f64,
    ty: f64,
    tz: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    qw: f64,
    timestamp_ns: u64,
) -> i32 {
    if tf.is_null() || frame_name.is_null() {
        return -1;
    }
    let tf = &*(tf as *const transform_ffi::FfiTransformFrame);
    let name = match CStr::from_ptr(frame_name).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match transform_ffi::transform_frame_update(
        tf,
        name,
        [tx, ty, tz],
        [qx, qy, qz, qw],
        timestamp_ns,
    ) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Lookup transform. Writes [tx, ty, tz, qx, qy, qz, qw] to out (7 doubles).
#[no_mangle]
pub unsafe extern "C" fn horus_transform_frame_lookup(
    tf: *const HorusTransformFrame,
    source: *const c_char,
    target: *const c_char,
    out: *mut f64,
) -> i32 {
    if tf.is_null() || source.is_null() || target.is_null() || out.is_null() {
        return -1;
    }
    let tf = &*(tf as *const transform_ffi::FfiTransformFrame);
    let source = match CStr::from_ptr(source).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    let target = match CStr::from_ptr(target).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    match transform_ffi::transform_frame_lookup(tf, source, target) {
        Ok(vals) => {
            for (i, val) in vals.iter().enumerate().take(7) {
                *out.add(i) = *val;
            }
            0
        }
        Err(_) => -1,
    }
}

/// Check if transform path exists.
#[no_mangle]
pub unsafe extern "C" fn horus_transform_frame_can_transform(
    tf: *const HorusTransformFrame,
    source: *const c_char,
    target: *const c_char,
) -> bool {
    if tf.is_null() || source.is_null() || target.is_null() {
        return false;
    }
    let tf = &*(tf as *const transform_ffi::FfiTransformFrame);
    let source = match CStr::from_ptr(source).to_str() {
        Ok(s) => s,
        Err(_) => return false,
    };
    let target = match CStr::from_ptr(target).to_str() {
        Ok(s) => s,
        Err(_) => return false,
    };
    transform_ffi::transform_frame_can_transform(tf, source, target)
}

// ─── BlackBox C API ──────────────────────────────────────────────────────────

/// Record a custom event visible in `horus log` and `horus blackbox` output.
/// Uses the log buffer (always available) which feeds into BlackBox when enabled.
#[no_mangle]
pub unsafe extern "C" fn horus_blackbox_record(category: *const c_char, message: *const c_char) {
    let cat = if category.is_null() {
        "cpp"
    } else {
        CStr::from_ptr(category).to_str().unwrap_or("cpp")
    };
    let msg = if message.is_null() {
        ""
    } else {
        CStr::from_ptr(message).to_str().unwrap_or("")
    };
    // Record as a Warning-level log entry with category as node name
    let entry = horus_core::core::log_buffer::LogEntry {
        timestamp: String::new(),
        tick_number: 0,
        node_name: cat.to_string(),
        log_type: horus_core::core::log_buffer::LogType::Warning,
        topic: None,
        message: format!("[blackbox] {}", msg),
        tick_us: 0,
        ipc_ns: 0,
    };
    horus_core::core::log_buffer::publish_log(entry);
}

// ─── Logging C API ───────────────────────────────────────────────────────────

/// Log a message. level: 0=Info, 1=Warning, 2=Error. node_name and message are C strings.
#[no_mangle]
pub unsafe extern "C" fn horus_log(level: u8, node_name: *const c_char, message: *const c_char) {
    let node = if node_name.is_null() {
        "cpp"
    } else {
        CStr::from_ptr(node_name).to_str().unwrap_or("cpp")
    };
    let msg = if message.is_null() {
        ""
    } else {
        CStr::from_ptr(message).to_str().unwrap_or("")
    };
    let log_type = match level {
        0 => horus_core::core::log_buffer::LogType::Info,
        1 => horus_core::core::log_buffer::LogType::Warning,
        2 => horus_core::core::log_buffer::LogType::Error,
        _ => horus_core::core::log_buffer::LogType::Info,
    };
    let entry = horus_core::core::log_buffer::LogEntry {
        timestamp: String::new(), // filled by log_buffer if empty
        tick_number: 0,
        node_name: node.to_string(),
        log_type,
        topic: None,
        message: msg.to_string(),
        tick_us: 0,
        ipc_ns: 0,
    };
    horus_core::core::log_buffer::publish_log(entry);
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
        assert_eq!(
            horus_get_abi_version(),
            crate::types_ffi::HORUS_CPP_ABI_VERSION
        );
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

    // ─── Pool C API Tests ───────────────────────────────────────────

    use std::sync::atomic::{AtomicU32, Ordering};
    static TEST_POOL_ID: AtomicU32 = AtomicU32::new(0);
    fn next_test_pool_id() -> u32 {
        let base = (std::process::id() % 10000) * 100 + 5000;
        base + TEST_POOL_ID.fetch_add(1, Ordering::Relaxed)
    }

    #[test]
    fn c_api_tensor_pool_lifecycle() {
        unsafe {
            let pool = horus_tensor_pool_new(next_test_pool_id(), 1024 * 1024, 64);
            assert!(!pool.is_null());

            let mut alloc: usize = 0;
            let mut used: usize = 0;
            let mut free: usize = 0;
            assert_eq!(
                horus_tensor_pool_stats(pool, &mut alloc, &mut used, &mut free),
                0
            );
            assert_eq!(alloc, 0);
            assert!(free > 0);

            horus_tensor_pool_destroy(pool);
        }
    }

    #[test]
    fn c_api_tensor_alloc_and_access() {
        unsafe {
            let pool = horus_tensor_pool_new(next_test_pool_id(), 1024 * 1024, 64);
            assert!(!pool.is_null());

            let shape: [u64; 3] = [10, 20, 3];
            let tensor = horus_tensor_alloc(pool, shape.as_ptr(), 3, 2); // u8
            assert!(!tensor.is_null());
            assert_eq!(horus_tensor_nbytes(tensor), 10 * 20 * 3);

            let ptr = horus_tensor_data_ptr(pool, tensor);
            assert!(!ptr.is_null());

            horus_tensor_release(pool, tensor);
            horus_tensor_destroy(tensor);
            horus_tensor_pool_destroy(pool);
        }
    }

    #[test]
    fn c_api_image_lifecycle() {
        unsafe {
            let pool = horus_tensor_pool_new(next_test_pool_id(), 4 * 1024 * 1024, 64);
            let img = horus_image_new(pool, 320, 240, 0); // RGB8
            assert!(!img.is_null());
            assert_eq!(horus_image_width(img), 320);
            assert_eq!(horus_image_height(img), 240);
            assert_eq!(horus_image_data_size(img), 320 * 240 * 3);

            horus_image_destroy(img);
            horus_tensor_pool_destroy(pool);
        }
    }

    #[test]
    fn c_api_pointcloud_lifecycle() {
        unsafe {
            let pool = horus_tensor_pool_new(next_test_pool_id(), 4 * 1024 * 1024, 64);
            let pc = horus_pointcloud_new(pool, 500, 4); // 500 XYZI points
            assert!(!pc.is_null());
            assert_eq!(horus_pointcloud_num_points(pc), 500);
            assert_eq!(horus_pointcloud_fields(pc), 4);

            horus_pointcloud_destroy(pc);
            horus_tensor_pool_destroy(pool);
        }
    }

    // ─── Params C API Tests ─────────────────────────────────────────

    #[test]
    fn c_api_params_lifecycle() {
        unsafe {
            let params = horus_params_new();
            assert!(!params.is_null());

            let key = CString::new("max_speed").unwrap();
            assert_eq!(horus_params_set_f64(params, key.as_ptr(), 1.5), 0);

            let mut out: f64 = 0.0;
            assert_eq!(horus_params_get_f64(params, key.as_ptr(), &mut out), 1);
            assert!((out - 1.5).abs() < f64::EPSILON);

            assert!(horus_params_has(params, key.as_ptr()));

            let missing = CString::new("missing").unwrap();
            assert!(!horus_params_has(params, missing.as_ptr()));

            horus_params_destroy(params);
        }
    }

    #[test]
    fn c_api_params_all_types() {
        unsafe {
            let params = horus_params_new();

            // i64
            let k = CString::new("count").unwrap();
            assert_eq!(horus_params_set_i64(params, k.as_ptr(), 42), 0);
            let mut iv: i64 = 0;
            assert_eq!(horus_params_get_i64(params, k.as_ptr(), &mut iv), 1);
            assert_eq!(iv, 42);

            // bool
            let k = CString::new("enabled").unwrap();
            assert_eq!(horus_params_set_bool(params, k.as_ptr(), true), 0);
            let mut bv: bool = false;
            assert_eq!(horus_params_get_bool(params, k.as_ptr(), &mut bv), 1);
            assert!(bv);

            // string
            let k = CString::new("name").unwrap();
            let v = CString::new("robot1").unwrap();
            assert_eq!(horus_params_set_string(params, k.as_ptr(), v.as_ptr()), 0);
            let mut buf = [0u8; 64];
            let len = horus_params_get_string(params, k.as_ptr(), buf.as_mut_ptr(), 64);
            assert_eq!(len, 6);
            assert_eq!(&buf[..6], b"robot1");

            horus_params_destroy(params);
        }
    }

    // ─── Service C API Tests ────────────────────────────────────────

    #[test]
    fn c_api_service_client_lifecycle() {
        unsafe {
            let name = CString::new("test_svc_c_api").unwrap();
            let client = horus_service_client_new(name.as_ptr());
            assert!(!client.is_null());
            horus_service_client_destroy(client);
        }
    }

    #[test]
    fn c_api_service_server_lifecycle() {
        unsafe {
            let name = CString::new("test_svc_server_c_api").unwrap();
            let server = horus_service_server_new(name.as_ptr());
            assert!(!server.is_null());
            horus_service_server_destroy(server);
        }
    }

    // ─── Action C API Tests ─────────────────────────────────────────

    #[test]
    fn c_api_action_client_send_goal() {
        unsafe {
            let name = CString::new("test_action_c_api").unwrap();
            let client = horus_action_client_new(name.as_ptr());
            assert!(!client.is_null());

            let goal = CString::new(r#"{"x": 1.0}"#).unwrap();
            let handle = horus_action_client_send_goal(client, goal.as_ptr());
            assert!(!handle.is_null());

            assert_eq!(horus_goal_handle_id(handle), 1);
            assert_eq!(horus_goal_handle_status(handle), 0); // Pending
            assert!(horus_goal_handle_is_active(handle));

            horus_action_client_cancel(handle);
            assert_eq!(horus_goal_handle_status(handle), 4); // Canceled
            assert!(!horus_goal_handle_is_active(handle));

            horus_goal_handle_destroy(handle);
            horus_action_client_destroy(client);
        }
    }

    #[test]
    fn c_api_action_server_lifecycle() {
        unsafe {
            let name = CString::new("test_action_srv_c_api").unwrap();
            let server = horus_action_server_new(name.as_ptr());
            assert!(!server.is_null());
            assert!(!horus_action_server_is_ready(server));

            extern "C" fn accept(_: *const u8, _: usize) -> u8 {
                0
            }
            extern "C" fn execute(_: u64, _: *const u8, _: usize) {}

            horus_action_server_set_accept_handler(server, accept);
            horus_action_server_set_execute_handler(server, execute);
            assert!(horus_action_server_is_ready(server));

            horus_action_server_destroy(server);
        }
    }

    // ─── Transform C API Tests ──────────────────────────────────────

    #[test]
    fn c_api_transform_frame_lifecycle() {
        unsafe {
            let tf = horus_transform_frame_new();
            assert!(!tf.is_null());

            let world = CString::new("world").unwrap();
            let base = CString::new("base").unwrap();

            // Register root
            assert!(horus_transform_frame_register(tf, world.as_ptr(), std::ptr::null()) >= 0);
            // Register child
            assert!(horus_transform_frame_register(tf, base.as_ptr(), world.as_ptr()) >= 0);

            // Update transform
            assert_eq!(
                horus_transform_frame_update(
                    tf,
                    base.as_ptr(),
                    1.0,
                    2.0,
                    3.0, // translation
                    0.0,
                    0.0,
                    0.0,
                    1.0, // identity rotation
                    1000,
                ),
                0
            );

            // Lookup
            let mut out = [0.0f64; 7];
            assert_eq!(
                horus_transform_frame_lookup(tf, base.as_ptr(), world.as_ptr(), out.as_mut_ptr()),
                0
            );
            assert!((out[0] - 1.0).abs() < 1e-10);
            assert!((out[1] - 2.0).abs() < 1e-10);
            assert!((out[2] - 3.0).abs() < 1e-10);

            // Can transform
            assert!(horus_transform_frame_can_transform(
                tf,
                base.as_ptr(),
                world.as_ptr()
            ));

            let other = CString::new("nonexistent").unwrap();
            assert!(!horus_transform_frame_can_transform(
                tf,
                world.as_ptr(),
                other.as_ptr()
            ));

            horus_transform_frame_destroy(tf);
        }
    }

    // ─── Null safety for new APIs ───────────────────────────────────

    #[test]
    fn c_api_new_apis_null_safety() {
        unsafe {
            // Pool
            horus_tensor_pool_destroy(std::ptr::null_mut());
            assert_eq!(
                horus_tensor_pool_stats(
                    std::ptr::null(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut()
                ),
                -1
            );
            assert!(horus_tensor_alloc(std::ptr::null(), std::ptr::null(), 0, 0).is_null());
            assert!(horus_tensor_data_ptr(std::ptr::null(), std::ptr::null()).is_null());
            assert_eq!(horus_tensor_nbytes(std::ptr::null()), 0);
            horus_tensor_destroy(std::ptr::null_mut());

            // Image
            assert!(horus_image_new(std::ptr::null(), 0, 0, 0).is_null());
            assert_eq!(horus_image_width(std::ptr::null()), 0);
            assert_eq!(horus_image_height(std::ptr::null()), 0);
            horus_image_destroy(std::ptr::null_mut());

            // PointCloud
            assert!(horus_pointcloud_new(std::ptr::null(), 0, 0).is_null());
            assert_eq!(horus_pointcloud_num_points(std::ptr::null()), 0);
            horus_pointcloud_destroy(std::ptr::null_mut());

            // Params
            horus_params_destroy(std::ptr::null_mut());
            assert!(!horus_params_has(std::ptr::null(), std::ptr::null()));

            // Service
            horus_service_client_destroy(std::ptr::null_mut());
            horus_service_server_destroy(std::ptr::null_mut());

            // Action
            horus_action_client_destroy(std::ptr::null_mut());
            assert!(horus_action_client_send_goal(std::ptr::null(), std::ptr::null()).is_null());
            assert_eq!(horus_goal_handle_status(std::ptr::null()), 5);
            assert!(!horus_goal_handle_is_active(std::ptr::null()));
            horus_goal_handle_destroy(std::ptr::null_mut());
            horus_action_server_destroy(std::ptr::null_mut());
            assert!(!horus_action_server_is_ready(std::ptr::null()));

            // Transform
            horus_transform_frame_destroy(std::ptr::null_mut());
            assert_eq!(
                horus_transform_frame_register(
                    std::ptr::null(),
                    std::ptr::null(),
                    std::ptr::null()
                ),
                -1
            );
            assert!(!horus_transform_frame_can_transform(
                std::ptr::null(),
                std::ptr::null(),
                std::ptr::null()
            ));
        }
    }
}
