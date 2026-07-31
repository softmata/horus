//! Generated cross-language layout contract for the C ABI message types.
//!
//! # What this is for
//!
//! `impl_pod_topic_c_api!` in `c_api.rs` implements every `_send` / `_recv`
//! entry point as a raw `std::ptr::read` / `std::ptr::write` of the **Rust**
//! type through a `*mut c_void` the C++ caller supplied. Nothing checks that
//! the C++ struct on the other side has the same size — or the same fields.
//!
//! The 2026-07-30 audit found that it very often does not. Two distinct
//! failure modes, both silent:
//!
//!   * **Size divergence.** `JointCommand` is 928 bytes in Rust and 88 in the
//!     C++ header, so `horus_subscriber_joint_command_recv` writes 840 bytes
//!     past the caller's object. `AudioFrame` overruns by 11040.
//!   * **Semantic divergence.** `ServoCommand` is 24 bytes on *both* sides, but
//!     the C++ struct is `{target_position, target_velocity, max_force,
//!     timestamp_ns}` and the Rust struct is `{servo_id, position, speed,
//!     enable, timestamp_ns}`. No size check can catch this: an actuator
//!     command is simply reinterpreted, field for field, into different
//!     meanings.
//!
//! A `sizeof` sweep would therefore pass cleanly on exactly the types where a
//! servo command silently becomes garbage. So this contract asserts **every
//! field offset by name**, which makes both failure modes a compile error: a
//! C++ struct that lacks a field, renames one, or moves one no longer builds.
//!
//! # How it works
//!
//! `layout_contract()` renders `include/horus/layout_contract.hpp` from the
//! Rust types themselves, using `offset_of!`. `contract_file_is_current`
//! compares that rendering against the committed header and fails if they have
//! drifted, so regenerating is a deliberate act with a reviewable diff.
//!
//! ## Regenerating
//!
//! ```text
//! cargo test -p horus_cpp --no-default-features regenerate_layout_contract -- --ignored --nocapture
//! ```
//!
//! # Why the field list is transcribed rather than derived
//!
//! `offset_of!` needs field identifiers at compile time, and these types live
//! in other crates (`horus_types`, and `horus_robotics` behind a pinned git
//! rev), so there is no derive to hang this off. The list below is therefore
//! transcribed — but it cannot silently rot: a renamed or removed field fails
//! to compile here, and a field ADDED upstream changes the struct size, which
//! the emitted `sizeof` assertion catches. Both directions are covered.

/// Every type reachable through `impl_pod_topic_c_api!`, with its fields in
/// declaration order.
macro_rules! layout_contract_types {
    ($mac:ident) => {
        // NOTE: horus_robotics::{CameraInfo, RegionOfInterest, StereoInfo,
        // PointField, PlaneDetection} are deliberately ABSENT. They are
        // repr(Rust), so they have no guaranteed layout and cannot be pinned by
        // any C++ struct; they were withdrawn from the C ABI in c_api.rs rather
        // than mirrored. See the note at the end of the impl_pod_topic_c_api!
        // list for how to restore them.
        $mac! {
    horus_robotics::AudioFrame => { samples num_samples sample_rate channels encoding _pad timestamp_ns frame_id },
    horus_robotics::BatteryState => { voltage current charge capacity percentage power_supply_status temperature cell_voltages cell_count timestamp_ns },
    horus_robotics::BoundingBox2D => { x y width height },
    horus_robotics::BoundingBox3D => { cx cy cz length width height roll pitch yaw },
    horus_robotics::ContactInfo => { state contact_force contact_normal contact_point stiffness damping confidence contact_start_time frame_id timestamp_ns },
    horus_robotics::Detection => { bbox confidence class_id class_name instance_id },
    horus_robotics::Detection3D => { bbox confidence class_id class_name velocity_x velocity_y velocity_z instance_id },
    horus_robotics::FluidPressure => { fluid_pressure variance frame_id timestamp_ns },
    horus_robotics::ForceCommand => { target_force target_torque force_mode position_setpoint orientation_setpoint max_deviation gains timeout_seconds frame_id timestamp_ns },
    horus_robotics::GoalResult => { goal_id status distance_to_goal eta_seconds progress error_message timestamp_ns },
    horus_robotics::HapticFeedback => { vibration_intensity vibration_frequency duration_seconds force_feedback pattern_type enabled timestamp_ns },
    horus_robotics::Illuminance => { illuminance variance frame_id timestamp_ns },
    horus_robotics::ImpedanceParameters => { stiffness damping inertia force_limits enabled timestamp_ns },
    horus_robotics::Imu => { orientation orientation_covariance angular_velocity angular_velocity_covariance linear_acceleration linear_acceleration_covariance timestamp_ns },
    horus_robotics::JointCommand => { joint_names joint_count positions velocities efforts modes timestamp_ns },
    horus_robotics::JointState => { names joint_count positions velocities efforts timestamp_ns },
    horus_robotics::JoystickInput => { joystick_id event_type element_id element_name value pressed timestamp_ms },
    horus_robotics::KeyboardInput => { key_name code modifier_flags pressed timestamp_ms _padding },
    horus_robotics::Landmark => { x y visibility index },
    horus_robotics::Landmark3D => { x y z visibility index },
    horus_robotics::LandmarkArray => { num_landmarks dimension instance_id confidence timestamp_ns bbox_x bbox_y bbox_width bbox_height },
    horus_robotics::LaserScan => { ranges angle_min angle_max range_min range_max angle_increment time_increment scan_time timestamp_ns },
    horus_robotics::MagneticField => { magnetic_field magnetic_field_covariance frame_id timestamp_ns },
    horus_robotics::MotorCommand => { motor_id mode target max_velocity max_acceleration feed_forward enable timestamp_ns },
    horus_robotics::NavGoal => { target_pose tolerance_position tolerance_angle timeout_seconds priority goal_id timestamp_ns },
    horus_robotics::NavPath => { waypoints waypoint_count total_length duration_seconds frame_id algorithm timestamp_ns },
    horus_robotics::NavSatFix => { latitude longitude altitude position_covariance position_covariance_type status satellites_visible hdop vdop speed heading timestamp_ns },
    horus_robotics::Odometry => { pose twist pose_covariance twist_covariance frame_id child_frame_id timestamp_ns },
    horus_robotics::PathPlan => { waypoint_data goal_pose waypoint_count timestamp_ns },
    horus_robotics::PidConfig => { controller_id kp ki kd integral_limit output_limit anti_windup timestamp_ns },
    horus_robotics::RangeSensor => { sensor_type field_of_view min_range max_range range timestamp_ns },
    horus_robotics::SegmentationMask => { width height num_classes mask_type timestamp_ns seq frame_id },
    horus_robotics::ServoCommand => { servo_id position speed enable timestamp_ns },
    horus_robotics::Temperature => { temperature variance frame_id timestamp_ns },
    horus_robotics::TrackedObject => { bbox predicted_bbox track_id confidence class_id velocity_x velocity_y accel_x accel_y age hits time_since_update state class_name },
    horus_robotics::TrackingHeader => { num_tracks frame_id timestamp_ns total_tracks active_tracks },
    horus_robotics::TrajectoryPoint => { position velocity acceleration orientation angular_velocity time_from_start },
    horus_robotics::VelocityObstacle => { position velocity radius time_horizon obstacle_id },
    horus_robotics::Waypoint => { pose velocity time_from_start curvature stop_required },
    horus_robotics::WrenchStamped => { force torque point_of_application frame_id timestamp_ns },
    horus_types::Accel => { linear angular timestamp_ns },
    horus_types::AccelStamped => { accel frame_id timestamp_ns },
    horus_types::Clock => { clock_ns realtime_ns sim_speed paused source timestamp_ns },
    horus_types::DiagnosticStatus => { level code message component timestamp_ns },
    horus_types::EmergencyStop => { engaged reason source auto_reset timestamp_ns },
    horus_types::Heartbeat => { node_name node_id sequence alive uptime timestamp_ns },
    horus_types::NodeHeartbeat => { state health tick_count target_rate_hz actual_rate_hz error_count last_tick_timestamp heartbeat_timestamp },
    horus_types::Point3 => { x y z },
    horus_types::Pose2D => { x y theta timestamp_ns },
    horus_types::Pose3D => { position orientation timestamp_ns },
    horus_types::PoseStamped => { pose frame_id timestamp_ns },
    horus_types::PoseWithCovariance => { pose covariance frame_id timestamp_ns },
    horus_types::Quaternion => { x y z w },
    horus_types::ResourceUsage => { cpu_percent memory_bytes memory_percent disk_bytes disk_percent network_tx_bytes network_rx_bytes temperature thread_count timestamp_ns },
    horus_types::SafetyStatus => { enabled estop_engaged watchdog_ok limits_ok comms_ok mode fault_code timestamp_ns },
    horus_types::TimeReference => { time_ref_ns source offset_ns timestamp_ns },
    horus_types::TransformStamped => { translation rotation timestamp_ns },
    horus_types::Twist => { linear angular timestamp_ns },
    horus_types::TwistWithCovariance => { twist covariance frame_id timestamp_ns },
    horus_types::Vector3 => { x y z },
        }
    };
}

/// Render the C++ side of the contract.
pub fn layout_contract() -> String {
    let mut out = String::new();
    out.push_str(CONTRACT_HEADER);

    macro_rules! emit {
        ($($path:path => { $($field:ident)* },)*) => {
            $(
                {
                    // `stringify!` renders a path with spaces around `::`
                    // ("horus_robotics :: AudioFrame"), so normalise before
                    // splitting or the C++ identifier comes out malformed.
                    let full_raw = stringify!($path);
                    let full = full_raw.replace(" :: ", "::");
                    let name = full.rsplit("::").next().unwrap_or(&full).to_string();
                    out.push_str(&format!(
                        "\n// {full}\nstatic_assert(sizeof(horus::msg::{name}) == {size}, \n    \"horus::msg::{name} must match Rust {full} ({size} bytes)\");\n",
                        full = full,
                        name = name,
                        size = ::std::mem::size_of::<$path>(),
                    ));
                    $(
                        out.push_str(&format!(
                            "static_assert(offsetof(horus::msg::{name}, {field}) == {off}, \n    \"horus::msg::{name}::{field} must be at Rust offset {off}\");\n",
                            name = name,
                            field = stringify!($field),
                            off = ::std::mem::offset_of!($path, $field),
                        ));
                    )*
                }
            )*
        };
    }

    layout_contract_types!(emit);
    out.push_str(CONTRACT_FOOTER);
    out
}

const CONTRACT_HEADER: &str = r#"// GENERATED FILE — DO NOT EDIT BY HAND.
//
// Rendered from the Rust message types by horus_cpp's `layout_contract()`.
// Regenerate with:
//   cargo test -p horus_cpp --no-default-features regenerate_layout_contract -- --ignored --nocapture
//
// Every `_send`/`_recv` entry point in the C ABI is a raw memory read/write of
// the RUST struct through a pointer the C++ caller supplied. If the C++ struct
// differs in size, the call overruns the caller's object; if it differs only in
// FIELD MEANING — same size, different fields — the data is silently
// reinterpreted, which no size check can detect.
//
// These assertions pin both. Include this after your message headers; any
// divergence becomes a compile error instead of memory corruption.

#pragma once

#include <cstddef>
#include "horus/messages.hpp"

"#;

const CONTRACT_FOOTER: &str = "";

#[cfg(test)]
mod tests {
    use super::*;

    /// Path of the committed contract header.
    fn contract_path() -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("include/horus/layout_contract.hpp")
    }

    /// Rewrite the committed contract. Run explicitly after changing a message
    /// type, and review the diff — a change here is a change to a published ABI.
    #[test]
    #[ignore = "generator: run explicitly to rewrite include/horus/layout_contract.hpp"]
    fn regenerate_layout_contract() {
        let path = contract_path();
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, layout_contract()).unwrap();
        println!("wrote {}", path.display());
    }

    /// The committed contract must match what the Rust types say today.
    ///
    /// This is the gate. If a message type gains, loses, renames or reorders a
    /// field, this fails and the C++ header has to be reconciled deliberately.
    #[test]
    fn contract_file_is_current() {
        let path = contract_path();
        let committed = std::fs::read_to_string(&path).unwrap_or_default();
        let rendered = layout_contract();
        assert_eq!(
            committed, rendered,
            "\n{} is stale — a Rust message type changed shape.\nRegenerate with:\n  \
             cargo test -p horus_cpp --no-default-features regenerate_layout_contract -- --ignored\n\
             and reconcile the C++ headers with the new offsets.",
            path.display()
        );
    }

    /// Compile the contract with a real C++ compiler and require zero errors.
    ///
    /// The generated header is only meaningful if someone actually compiles it,
    /// and nothing in the cargo build does — `build.rs` is a stub, so the C++
    /// side is built entirely out-of-tree. Without this test the contract would
    /// be a file that documents a mismatch rather than a gate that stops one.
    ///
    /// Skips (rather than fails) when no C++ compiler is present, so the Rust
    /// test suite still runs on a machine without one.
    #[test]
    fn cpp_headers_satisfy_the_contract() {
        let cxx = ["g++", "clang++"].into_iter().find(|c| {
            std::process::Command::new(c)
                .arg("--version")
                .output()
                .is_ok()
        });
        let Some(cxx) = cxx else {
            eprintln!("skipping: no C++ compiler on PATH");
            return;
        };

        let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let tu = std::env::temp_dir().join(format!("horus_contract_{}.cpp", std::process::id()));
        std::fs::write(&tu, "#include \"horus/layout_contract.hpp\"\n").unwrap();

        let out = std::process::Command::new(cxx)
            .arg("-std=c++17")
            .arg("-fsyntax-only")
            .arg("-I")
            .arg(manifest.join("include"))
            .arg(&tu)
            .output()
            .expect("failed to run the C++ compiler");
        let _ = std::fs::remove_file(&tu);

        if !out.status.success() {
            let stderr = String::from_utf8_lossy(&out.stderr);
            let errors: Vec<&str> = stderr
                .lines()
                .filter(|l| l.contains("error:"))
                .take(25)
                .collect();
            panic!(
                "The C++ message headers do not match the Rust types the C ABI reads and \
                 writes through them ({} errors). Every one of these is either a buffer \
                 overrun or a silent field-for-field reinterpretation in \
                 horus_publisher_*_send / horus_subscriber_*_recv.\n\nFirst errors:\n{}",
                stderr.matches("error:").count(),
                errors.join("\n")
            );
        }
    }

    /// The contract must actually cover every type the C ABI exposes.
    ///
    /// Without this, deleting a line from the macro list would silently drop a
    /// type back to unchecked — which is how 57 of 70 types went unverified in
    /// the first place.
    #[test]
    fn contract_covers_every_bound_type() {
        let c_api = include_str!("c_api.rs");
        let rendered = layout_contract();

        let mut missing = Vec::new();
        for line in c_api.lines() {
            let Some(rest) = line.trim().strip_prefix("impl_pod_topic_c_api!(") else {
                continue;
            };
            let Some((_, ty)) = rest.split_once(',') else {
                continue;
            };
            let ty = ty.trim().trim_end_matches(");").trim();
            let name = ty.rsplit("::").next().unwrap_or(ty);
            if !rendered.contains(&format!("sizeof(horus::msg::{name})")) {
                missing.push(name.to_string());
            }
        }
        assert!(
            missing.is_empty(),
            "these types are exposed over the C ABI but absent from the layout contract, \
             so nothing checks their C++ struct: {missing:?}"
        );
    }
}
