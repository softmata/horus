// Hand-maintained C++ mirror of the Rust `#[repr(C)]` message types.
// Source of truth: horus-robotics/src/messages/control.rs, cmd_vel.rs
// Verified against the generated `horus/layout_contract.hpp` — regenerate and
// recompile that file after any change here.
#pragma once

#include <cstddef>
#include <cstdint>

// These structs are exact mirrors of the Rust `#[repr(C)]` message types.
// The C ABI (`horus_publisher_*_send` / `horus_subscriber_*_recv`) does a raw
// `std::ptr::read` / `std::ptr::write` of `size_of::<RustType>()` bytes through
// the pointer you pass, so ANY divergence here is memory corruption, not a type
// error. Field names, order, widths and implicit padding must match the Rust
// definition byte-for-byte; `horus/layout_contract.hpp` pins this at compile
// time. Rust `bool` is mirrored as `uint8_t` (1 byte, guaranteed) rather than
// C++ `bool`, whose size is implementation-defined.
//
// Keep every struct a plain public aggregate — no constructors, no methods, no
// inheritance — or `offsetof` in the layout contract becomes invalid.

namespace horus { namespace msg {

/// Mirrors Rust `horus_robotics::CmdVel` (16 bytes).
/// 2D velocity command — the most common robotics message.
struct CmdVel {
    uint64_t timestamp_ns;
    float linear;   // m/s forward velocity
    float angular;  // rad/s turning velocity
};

/// Mirrors Rust `horus_robotics::MotorCommand` (56 bytes).
/// Direct motor control; supports velocity/position/torque/voltage modes.
struct MotorCommand {
    uint8_t motor_id;           // offset 0  — motor ID (multi-motor systems)
    uint8_t mode;               // offset 1  — 0=velocity, 1=position, 2=torque, 3=voltage
                                // 6 bytes implicit padding (f64 alignment)
    double target;              // offset 8  — target value (units depend on mode)
    double max_velocity;        // offset 16 — max velocity (position mode)
    double max_acceleration;    // offset 24
    double feed_forward;        // offset 32
    uint8_t enable;             // offset 40 — 0 = brake/coast depending on config
                                // 7 bytes implicit padding (u64 alignment)
    uint64_t timestamp_ns;      // offset 48
};

/// Mirrors Rust `horus_robotics::DifferentialDriveCommand` (40 bytes).
/// Two-wheeled differential drive command.
struct DifferentialDriveCommand {
    double left_velocity;       // offset 0  — rad/s
    double right_velocity;      // offset 8  — rad/s
    double max_acceleration;    // offset 16 — rad/s^2
    uint8_t enable;             // offset 24
                                // 7 bytes implicit padding (u64 alignment)
    uint64_t timestamp_ns;      // offset 32
};

/// Mirrors Rust `horus_robotics::ServoCommand` (24 bytes).
///
/// NOTE: the previous hand-written definition here was
/// `{float target_position; float target_velocity; float max_force; uint64_t
/// timestamp_ns;}` — the same 24 bytes but entirely different fields, so servo
/// commands were silently reinterpreted across the FFI boundary. The Rust
/// definition below is authoritative.
struct ServoCommand {
    uint8_t servo_id;           // offset 0  — servo ID (multi-servo systems)
                                // 3 bytes implicit padding (f32 alignment)
    float position;             // offset 4  — target position in radians
    float speed;                // offset 8  — movement speed (0-1, 0 = max speed)
    uint8_t enable;             // offset 12 — torque enable
                                // 3 bytes implicit padding (u64 alignment)
    uint64_t timestamp_ns;      // offset 16
};

/// Mirrors Rust `horus_robotics::PidConfig` (64 bytes).
/// PID gains configuration message.
struct PidConfig {
    uint8_t controller_id;      // offset 0
                                // 7 bytes implicit padding (f64 alignment)
    double kp;                  // offset 8  — proportional gain
    double ki;                  // offset 16 — integral gain
    double kd;                  // offset 24 — derivative gain
    double integral_limit;      // offset 32 — integral windup limit
    double output_limit;        // offset 40
    uint8_t anti_windup;        // offset 48 — enable anti-windup
                                // 7 bytes implicit padding (u64 alignment)
    uint64_t timestamp_ns;      // offset 56
};

/// Mirrors Rust `horus_robotics::TrajectoryPoint` (136 bytes).
/// Trajectory point for path following.
struct TrajectoryPoint {
    double position[3];         // offset 0   — [x, y, z]
    double velocity[3];         // offset 24  — [vx, vy, vz]
    double acceleration[3];     // offset 48  — [ax, ay, az]
    double orientation[4];      // offset 72  — quaternion [x, y, z, w]
    double angular_velocity[3]; // offset 104 — [wx, wy, wz]
    double time_from_start;     // offset 128 — seconds from trajectory start
};

/// Mirrors Rust `horus_robotics::JointCommand` (928 bytes).
///
/// NOTE: the previous hand-written definition here was 88 bytes, so every
/// `horus_subscriber_joint_command_recv` wrote 840 bytes past the caller's
/// object. Fixed-capacity 16-joint arrays as in Rust.
struct JointCommand {
    uint8_t joint_names[16][32];// offset 0   — 16 joints, 32-byte NUL-padded names
    uint8_t joint_count;        // offset 512 — number of active joints
                                // 7 bytes implicit padding (f64 alignment)
    double positions[16];       // offset 520 — radians
    double velocities[16];      // offset 648 — rad/s
    double efforts[16];         // offset 776 — Nm
    uint8_t modes[16];          // offset 904 — 0=position, 1=velocity, 2=effort
    uint64_t timestamp_ns;      // offset 920
};

// The generated contract only pins `offsetof(JointCommand, joint_names) == 0`,
// which any 512-byte array shape at offset 0 satisfies — a transposed
// `[32][16]` would pass every existing check. Pin the dimensions here.
static_assert(sizeof(JointCommand::joint_names) == 512, "JointCommand::joint_names must be 512 bytes");
static_assert(sizeof(JointCommand::joint_names[0]) == 32, "JointCommand::joint_names inner dim must be 32 (Rust [[u8; 32]; 16])");

// CmdVel is not covered by the generated layout_contract.hpp: it does not go
// through `impl_pod_topic_c_api!` but through the hand-written HorusCmdVel C
// API, which copies field by field rather than reinterpreting raw bytes. Pin it
// here anyway.
//
// DifferentialDriveCommand IS now in the generated contract (it was missing
// because the coverage test that is supposed to catch that could only parse
// single-line macro invocations, and this one is rustfmt-wrapped). These
// duplicates are kept as a local pin for translation units that include this
// header without layout_contract.hpp.
static_assert(sizeof(CmdVel) == 16, "CmdVel size mismatch");
static_assert(offsetof(CmdVel, timestamp_ns) == 0, "CmdVel::timestamp_ns offset");
static_assert(offsetof(CmdVel, linear) == 8, "CmdVel::linear offset");
static_assert(offsetof(CmdVel, angular) == 12, "CmdVel::angular offset");

static_assert(sizeof(DifferentialDriveCommand) == 40, "DifferentialDriveCommand size mismatch");
static_assert(offsetof(DifferentialDriveCommand, left_velocity) == 0, "DifferentialDriveCommand::left_velocity offset");
static_assert(offsetof(DifferentialDriveCommand, right_velocity) == 8, "DifferentialDriveCommand::right_velocity offset");
static_assert(offsetof(DifferentialDriveCommand, max_acceleration) == 16, "DifferentialDriveCommand::max_acceleration offset");
static_assert(offsetof(DifferentialDriveCommand, enable) == 24, "DifferentialDriveCommand::enable offset");
static_assert(offsetof(DifferentialDriveCommand, timestamp_ns) == 32, "DifferentialDriveCommand::timestamp_ns offset");

}} // namespace horus::msg
