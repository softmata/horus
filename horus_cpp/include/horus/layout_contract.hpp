// GENERATED FILE — DO NOT EDIT BY HAND.
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


// horus_robotics::AudioFrame
static_assert(sizeof(horus::msg::AudioFrame) == 19256,
    "horus::msg::AudioFrame must match Rust horus_robotics::AudioFrame (19256 bytes)");
static_assert(offsetof(horus::msg::AudioFrame, samples) == 0,
    "horus::msg::AudioFrame::samples must be at Rust offset 0");
static_assert(offsetof(horus::msg::AudioFrame, num_samples) == 19200,
    "horus::msg::AudioFrame::num_samples must be at Rust offset 19200");
static_assert(offsetof(horus::msg::AudioFrame, sample_rate) == 19204,
    "horus::msg::AudioFrame::sample_rate must be at Rust offset 19204");
static_assert(offsetof(horus::msg::AudioFrame, channels) == 19208,
    "horus::msg::AudioFrame::channels must be at Rust offset 19208");
static_assert(offsetof(horus::msg::AudioFrame, encoding) == 19209,
    "horus::msg::AudioFrame::encoding must be at Rust offset 19209");
static_assert(offsetof(horus::msg::AudioFrame, _pad) == 19210,
    "horus::msg::AudioFrame::_pad must be at Rust offset 19210");
static_assert(offsetof(horus::msg::AudioFrame, timestamp_ns) == 19216,
    "horus::msg::AudioFrame::timestamp_ns must be at Rust offset 19216");
static_assert(offsetof(horus::msg::AudioFrame, frame_id) == 19224,
    "horus::msg::AudioFrame::frame_id must be at Rust offset 19224");

// horus_robotics::BatteryState
static_assert(sizeof(horus::msg::BatteryState) == 104,
    "horus::msg::BatteryState must match Rust horus_robotics::BatteryState (104 bytes)");
static_assert(offsetof(horus::msg::BatteryState, voltage) == 0,
    "horus::msg::BatteryState::voltage must be at Rust offset 0");
static_assert(offsetof(horus::msg::BatteryState, current) == 4,
    "horus::msg::BatteryState::current must be at Rust offset 4");
static_assert(offsetof(horus::msg::BatteryState, charge) == 8,
    "horus::msg::BatteryState::charge must be at Rust offset 8");
static_assert(offsetof(horus::msg::BatteryState, capacity) == 12,
    "horus::msg::BatteryState::capacity must be at Rust offset 12");
static_assert(offsetof(horus::msg::BatteryState, percentage) == 16,
    "horus::msg::BatteryState::percentage must be at Rust offset 16");
static_assert(offsetof(horus::msg::BatteryState, power_supply_status) == 20,
    "horus::msg::BatteryState::power_supply_status must be at Rust offset 20");
static_assert(offsetof(horus::msg::BatteryState, temperature) == 24,
    "horus::msg::BatteryState::temperature must be at Rust offset 24");
static_assert(offsetof(horus::msg::BatteryState, cell_voltages) == 28,
    "horus::msg::BatteryState::cell_voltages must be at Rust offset 28");
static_assert(offsetof(horus::msg::BatteryState, cell_count) == 92,
    "horus::msg::BatteryState::cell_count must be at Rust offset 92");
static_assert(offsetof(horus::msg::BatteryState, timestamp_ns) == 96,
    "horus::msg::BatteryState::timestamp_ns must be at Rust offset 96");

// horus_robotics::BoundingBox2D
static_assert(sizeof(horus::msg::BoundingBox2D) == 16,
    "horus::msg::BoundingBox2D must match Rust horus_robotics::BoundingBox2D (16 bytes)");
static_assert(offsetof(horus::msg::BoundingBox2D, x) == 0,
    "horus::msg::BoundingBox2D::x must be at Rust offset 0");
static_assert(offsetof(horus::msg::BoundingBox2D, y) == 4,
    "horus::msg::BoundingBox2D::y must be at Rust offset 4");
static_assert(offsetof(horus::msg::BoundingBox2D, width) == 8,
    "horus::msg::BoundingBox2D::width must be at Rust offset 8");
static_assert(offsetof(horus::msg::BoundingBox2D, height) == 12,
    "horus::msg::BoundingBox2D::height must be at Rust offset 12");

// horus_robotics::BoundingBox3D
static_assert(sizeof(horus::msg::BoundingBox3D) == 48,
    "horus::msg::BoundingBox3D must match Rust horus_robotics::BoundingBox3D (48 bytes)");
static_assert(offsetof(horus::msg::BoundingBox3D, cx) == 0,
    "horus::msg::BoundingBox3D::cx must be at Rust offset 0");
static_assert(offsetof(horus::msg::BoundingBox3D, cy) == 4,
    "horus::msg::BoundingBox3D::cy must be at Rust offset 4");
static_assert(offsetof(horus::msg::BoundingBox3D, cz) == 8,
    "horus::msg::BoundingBox3D::cz must be at Rust offset 8");
static_assert(offsetof(horus::msg::BoundingBox3D, length) == 12,
    "horus::msg::BoundingBox3D::length must be at Rust offset 12");
static_assert(offsetof(horus::msg::BoundingBox3D, width) == 16,
    "horus::msg::BoundingBox3D::width must be at Rust offset 16");
static_assert(offsetof(horus::msg::BoundingBox3D, height) == 20,
    "horus::msg::BoundingBox3D::height must be at Rust offset 20");
static_assert(offsetof(horus::msg::BoundingBox3D, roll) == 24,
    "horus::msg::BoundingBox3D::roll must be at Rust offset 24");
static_assert(offsetof(horus::msg::BoundingBox3D, pitch) == 28,
    "horus::msg::BoundingBox3D::pitch must be at Rust offset 28");
static_assert(offsetof(horus::msg::BoundingBox3D, yaw) == 32,
    "horus::msg::BoundingBox3D::yaw must be at Rust offset 32");

// horus_robotics::ContactInfo
static_assert(sizeof(horus::msg::ContactInfo) == 136,
    "horus::msg::ContactInfo must match Rust horus_robotics::ContactInfo (136 bytes)");
static_assert(offsetof(horus::msg::ContactInfo, state) == 0,
    "horus::msg::ContactInfo::state must be at Rust offset 0");
static_assert(offsetof(horus::msg::ContactInfo, contact_force) == 8,
    "horus::msg::ContactInfo::contact_force must be at Rust offset 8");
static_assert(offsetof(horus::msg::ContactInfo, contact_normal) == 16,
    "horus::msg::ContactInfo::contact_normal must be at Rust offset 16");
static_assert(offsetof(horus::msg::ContactInfo, contact_point) == 40,
    "horus::msg::ContactInfo::contact_point must be at Rust offset 40");
static_assert(offsetof(horus::msg::ContactInfo, stiffness) == 64,
    "horus::msg::ContactInfo::stiffness must be at Rust offset 64");
static_assert(offsetof(horus::msg::ContactInfo, damping) == 72,
    "horus::msg::ContactInfo::damping must be at Rust offset 72");
static_assert(offsetof(horus::msg::ContactInfo, confidence) == 80,
    "horus::msg::ContactInfo::confidence must be at Rust offset 80");
static_assert(offsetof(horus::msg::ContactInfo, contact_start_time) == 88,
    "horus::msg::ContactInfo::contact_start_time must be at Rust offset 88");
static_assert(offsetof(horus::msg::ContactInfo, frame_id) == 96,
    "horus::msg::ContactInfo::frame_id must be at Rust offset 96");
static_assert(offsetof(horus::msg::ContactInfo, timestamp_ns) == 128,
    "horus::msg::ContactInfo::timestamp_ns must be at Rust offset 128");

// horus_robotics::Detection
static_assert(sizeof(horus::msg::Detection) == 72,
    "horus::msg::Detection must match Rust horus_robotics::Detection (72 bytes)");
static_assert(offsetof(horus::msg::Detection, bbox) == 0,
    "horus::msg::Detection::bbox must be at Rust offset 0");
static_assert(offsetof(horus::msg::Detection, confidence) == 16,
    "horus::msg::Detection::confidence must be at Rust offset 16");
static_assert(offsetof(horus::msg::Detection, class_id) == 20,
    "horus::msg::Detection::class_id must be at Rust offset 20");
static_assert(offsetof(horus::msg::Detection, class_name) == 24,
    "horus::msg::Detection::class_name must be at Rust offset 24");
static_assert(offsetof(horus::msg::Detection, instance_id) == 56,
    "horus::msg::Detection::instance_id must be at Rust offset 56");

// horus_robotics::Detection3D
static_assert(sizeof(horus::msg::Detection3D) == 104,
    "horus::msg::Detection3D must match Rust horus_robotics::Detection3D (104 bytes)");
static_assert(offsetof(horus::msg::Detection3D, bbox) == 0,
    "horus::msg::Detection3D::bbox must be at Rust offset 0");
static_assert(offsetof(horus::msg::Detection3D, confidence) == 48,
    "horus::msg::Detection3D::confidence must be at Rust offset 48");
static_assert(offsetof(horus::msg::Detection3D, class_id) == 52,
    "horus::msg::Detection3D::class_id must be at Rust offset 52");
static_assert(offsetof(horus::msg::Detection3D, class_name) == 56,
    "horus::msg::Detection3D::class_name must be at Rust offset 56");
static_assert(offsetof(horus::msg::Detection3D, velocity_x) == 88,
    "horus::msg::Detection3D::velocity_x must be at Rust offset 88");
static_assert(offsetof(horus::msg::Detection3D, velocity_y) == 92,
    "horus::msg::Detection3D::velocity_y must be at Rust offset 92");
static_assert(offsetof(horus::msg::Detection3D, velocity_z) == 96,
    "horus::msg::Detection3D::velocity_z must be at Rust offset 96");
static_assert(offsetof(horus::msg::Detection3D, instance_id) == 100,
    "horus::msg::Detection3D::instance_id must be at Rust offset 100");

// horus_robotics::DifferentialDriveCommand
static_assert(sizeof(horus::msg::DifferentialDriveCommand) == 40,
    "horus::msg::DifferentialDriveCommand must match Rust horus_robotics::DifferentialDriveCommand (40 bytes)");
static_assert(offsetof(horus::msg::DifferentialDriveCommand, left_velocity) == 0,
    "horus::msg::DifferentialDriveCommand::left_velocity must be at Rust offset 0");
static_assert(offsetof(horus::msg::DifferentialDriveCommand, right_velocity) == 8,
    "horus::msg::DifferentialDriveCommand::right_velocity must be at Rust offset 8");
static_assert(offsetof(horus::msg::DifferentialDriveCommand, max_acceleration) == 16,
    "horus::msg::DifferentialDriveCommand::max_acceleration must be at Rust offset 16");
static_assert(offsetof(horus::msg::DifferentialDriveCommand, enable) == 24,
    "horus::msg::DifferentialDriveCommand::enable must be at Rust offset 24");
static_assert(offsetof(horus::msg::DifferentialDriveCommand, timestamp_ns) == 32,
    "horus::msg::DifferentialDriveCommand::timestamp_ns must be at Rust offset 32");

// horus_robotics::FluidPressure
static_assert(sizeof(horus::msg::FluidPressure) == 56,
    "horus::msg::FluidPressure must match Rust horus_robotics::FluidPressure (56 bytes)");
static_assert(offsetof(horus::msg::FluidPressure, fluid_pressure) == 0,
    "horus::msg::FluidPressure::fluid_pressure must be at Rust offset 0");
static_assert(offsetof(horus::msg::FluidPressure, variance) == 8,
    "horus::msg::FluidPressure::variance must be at Rust offset 8");
static_assert(offsetof(horus::msg::FluidPressure, frame_id) == 16,
    "horus::msg::FluidPressure::frame_id must be at Rust offset 16");
static_assert(offsetof(horus::msg::FluidPressure, timestamp_ns) == 48,
    "horus::msg::FluidPressure::timestamp_ns must be at Rust offset 48");

// horus_robotics::ForceCommand
static_assert(sizeof(horus::msg::ForceCommand) == 224,
    "horus::msg::ForceCommand must match Rust horus_robotics::ForceCommand (224 bytes)");
static_assert(offsetof(horus::msg::ForceCommand, target_force) == 0,
    "horus::msg::ForceCommand::target_force must be at Rust offset 0");
static_assert(offsetof(horus::msg::ForceCommand, target_torque) == 24,
    "horus::msg::ForceCommand::target_torque must be at Rust offset 24");
static_assert(offsetof(horus::msg::ForceCommand, force_mode) == 48,
    "horus::msg::ForceCommand::force_mode must be at Rust offset 48");
static_assert(offsetof(horus::msg::ForceCommand, position_setpoint) == 56,
    "horus::msg::ForceCommand::position_setpoint must be at Rust offset 56");
static_assert(offsetof(horus::msg::ForceCommand, orientation_setpoint) == 80,
    "horus::msg::ForceCommand::orientation_setpoint must be at Rust offset 80");
static_assert(offsetof(horus::msg::ForceCommand, max_deviation) == 104,
    "horus::msg::ForceCommand::max_deviation must be at Rust offset 104");
static_assert(offsetof(horus::msg::ForceCommand, gains) == 128,
    "horus::msg::ForceCommand::gains must be at Rust offset 128");
static_assert(offsetof(horus::msg::ForceCommand, timeout_seconds) == 176,
    "horus::msg::ForceCommand::timeout_seconds must be at Rust offset 176");
static_assert(offsetof(horus::msg::ForceCommand, frame_id) == 184,
    "horus::msg::ForceCommand::frame_id must be at Rust offset 184");
static_assert(offsetof(horus::msg::ForceCommand, timestamp_ns) == 216,
    "horus::msg::ForceCommand::timestamp_ns must be at Rust offset 216");

// horus_robotics::GoalResult
static_assert(sizeof(horus::msg::GoalResult) == 104,
    "horus::msg::GoalResult must match Rust horus_robotics::GoalResult (104 bytes)");
static_assert(offsetof(horus::msg::GoalResult, goal_id) == 0,
    "horus::msg::GoalResult::goal_id must be at Rust offset 0");
static_assert(offsetof(horus::msg::GoalResult, status) == 4,
    "horus::msg::GoalResult::status must be at Rust offset 4");
static_assert(offsetof(horus::msg::GoalResult, distance_to_goal) == 8,
    "horus::msg::GoalResult::distance_to_goal must be at Rust offset 8");
static_assert(offsetof(horus::msg::GoalResult, eta_seconds) == 16,
    "horus::msg::GoalResult::eta_seconds must be at Rust offset 16");
static_assert(offsetof(horus::msg::GoalResult, progress) == 24,
    "horus::msg::GoalResult::progress must be at Rust offset 24");
static_assert(offsetof(horus::msg::GoalResult, error_message) == 28,
    "horus::msg::GoalResult::error_message must be at Rust offset 28");
static_assert(offsetof(horus::msg::GoalResult, timestamp_ns) == 96,
    "horus::msg::GoalResult::timestamp_ns must be at Rust offset 96");

// horus_robotics::HapticFeedback
static_assert(sizeof(horus::msg::HapticFeedback) == 56,
    "horus::msg::HapticFeedback must match Rust horus_robotics::HapticFeedback (56 bytes)");
static_assert(offsetof(horus::msg::HapticFeedback, vibration_intensity) == 0,
    "horus::msg::HapticFeedback::vibration_intensity must be at Rust offset 0");
static_assert(offsetof(horus::msg::HapticFeedback, vibration_frequency) == 4,
    "horus::msg::HapticFeedback::vibration_frequency must be at Rust offset 4");
static_assert(offsetof(horus::msg::HapticFeedback, duration_seconds) == 8,
    "horus::msg::HapticFeedback::duration_seconds must be at Rust offset 8");
static_assert(offsetof(horus::msg::HapticFeedback, force_feedback) == 16,
    "horus::msg::HapticFeedback::force_feedback must be at Rust offset 16");
static_assert(offsetof(horus::msg::HapticFeedback, pattern_type) == 40,
    "horus::msg::HapticFeedback::pattern_type must be at Rust offset 40");
static_assert(offsetof(horus::msg::HapticFeedback, enabled) == 41,
    "horus::msg::HapticFeedback::enabled must be at Rust offset 41");
static_assert(offsetof(horus::msg::HapticFeedback, timestamp_ns) == 48,
    "horus::msg::HapticFeedback::timestamp_ns must be at Rust offset 48");

// horus_robotics::Illuminance
static_assert(sizeof(horus::msg::Illuminance) == 56,
    "horus::msg::Illuminance must match Rust horus_robotics::Illuminance (56 bytes)");
static_assert(offsetof(horus::msg::Illuminance, illuminance) == 0,
    "horus::msg::Illuminance::illuminance must be at Rust offset 0");
static_assert(offsetof(horus::msg::Illuminance, variance) == 8,
    "horus::msg::Illuminance::variance must be at Rust offset 8");
static_assert(offsetof(horus::msg::Illuminance, frame_id) == 16,
    "horus::msg::Illuminance::frame_id must be at Rust offset 16");
static_assert(offsetof(horus::msg::Illuminance, timestamp_ns) == 48,
    "horus::msg::Illuminance::timestamp_ns must be at Rust offset 48");

// horus_robotics::ImpedanceParameters
static_assert(sizeof(horus::msg::ImpedanceParameters) == 208,
    "horus::msg::ImpedanceParameters must match Rust horus_robotics::ImpedanceParameters (208 bytes)");
static_assert(offsetof(horus::msg::ImpedanceParameters, stiffness) == 0,
    "horus::msg::ImpedanceParameters::stiffness must be at Rust offset 0");
static_assert(offsetof(horus::msg::ImpedanceParameters, damping) == 48,
    "horus::msg::ImpedanceParameters::damping must be at Rust offset 48");
static_assert(offsetof(horus::msg::ImpedanceParameters, inertia) == 96,
    "horus::msg::ImpedanceParameters::inertia must be at Rust offset 96");
static_assert(offsetof(horus::msg::ImpedanceParameters, force_limits) == 144,
    "horus::msg::ImpedanceParameters::force_limits must be at Rust offset 144");
static_assert(offsetof(horus::msg::ImpedanceParameters, enabled) == 192,
    "horus::msg::ImpedanceParameters::enabled must be at Rust offset 192");
static_assert(offsetof(horus::msg::ImpedanceParameters, timestamp_ns) == 200,
    "horus::msg::ImpedanceParameters::timestamp_ns must be at Rust offset 200");

// horus_robotics::Imu
static_assert(sizeof(horus::msg::Imu) == 304,
    "horus::msg::Imu must match Rust horus_robotics::Imu (304 bytes)");
static_assert(offsetof(horus::msg::Imu, orientation) == 0,
    "horus::msg::Imu::orientation must be at Rust offset 0");
static_assert(offsetof(horus::msg::Imu, orientation_covariance) == 32,
    "horus::msg::Imu::orientation_covariance must be at Rust offset 32");
static_assert(offsetof(horus::msg::Imu, angular_velocity) == 104,
    "horus::msg::Imu::angular_velocity must be at Rust offset 104");
static_assert(offsetof(horus::msg::Imu, angular_velocity_covariance) == 128,
    "horus::msg::Imu::angular_velocity_covariance must be at Rust offset 128");
static_assert(offsetof(horus::msg::Imu, linear_acceleration) == 200,
    "horus::msg::Imu::linear_acceleration must be at Rust offset 200");
static_assert(offsetof(horus::msg::Imu, linear_acceleration_covariance) == 224,
    "horus::msg::Imu::linear_acceleration_covariance must be at Rust offset 224");
static_assert(offsetof(horus::msg::Imu, timestamp_ns) == 296,
    "horus::msg::Imu::timestamp_ns must be at Rust offset 296");

// horus_robotics::JointCommand
static_assert(sizeof(horus::msg::JointCommand) == 928,
    "horus::msg::JointCommand must match Rust horus_robotics::JointCommand (928 bytes)");
static_assert(offsetof(horus::msg::JointCommand, joint_names) == 0,
    "horus::msg::JointCommand::joint_names must be at Rust offset 0");
static_assert(offsetof(horus::msg::JointCommand, joint_count) == 512,
    "horus::msg::JointCommand::joint_count must be at Rust offset 512");
static_assert(offsetof(horus::msg::JointCommand, positions) == 520,
    "horus::msg::JointCommand::positions must be at Rust offset 520");
static_assert(offsetof(horus::msg::JointCommand, velocities) == 648,
    "horus::msg::JointCommand::velocities must be at Rust offset 648");
static_assert(offsetof(horus::msg::JointCommand, efforts) == 776,
    "horus::msg::JointCommand::efforts must be at Rust offset 776");
static_assert(offsetof(horus::msg::JointCommand, modes) == 904,
    "horus::msg::JointCommand::modes must be at Rust offset 904");
static_assert(offsetof(horus::msg::JointCommand, timestamp_ns) == 920,
    "horus::msg::JointCommand::timestamp_ns must be at Rust offset 920");

// horus_robotics::JointState
static_assert(sizeof(horus::msg::JointState) == 912,
    "horus::msg::JointState must match Rust horus_robotics::JointState (912 bytes)");
static_assert(offsetof(horus::msg::JointState, names) == 0,
    "horus::msg::JointState::names must be at Rust offset 0");
static_assert(offsetof(horus::msg::JointState, joint_count) == 512,
    "horus::msg::JointState::joint_count must be at Rust offset 512");
static_assert(offsetof(horus::msg::JointState, positions) == 520,
    "horus::msg::JointState::positions must be at Rust offset 520");
static_assert(offsetof(horus::msg::JointState, velocities) == 648,
    "horus::msg::JointState::velocities must be at Rust offset 648");
static_assert(offsetof(horus::msg::JointState, efforts) == 776,
    "horus::msg::JointState::efforts must be at Rust offset 776");
static_assert(offsetof(horus::msg::JointState, timestamp_ns) == 904,
    "horus::msg::JointState::timestamp_ns must be at Rust offset 904");

// horus_robotics::JoystickInput
static_assert(sizeof(horus::msg::JoystickInput) == 72,
    "horus::msg::JoystickInput must match Rust horus_robotics::JoystickInput (72 bytes)");
static_assert(offsetof(horus::msg::JoystickInput, joystick_id) == 0,
    "horus::msg::JoystickInput::joystick_id must be at Rust offset 0");
static_assert(offsetof(horus::msg::JoystickInput, event_type) == 4,
    "horus::msg::JoystickInput::event_type must be at Rust offset 4");
static_assert(offsetof(horus::msg::JoystickInput, element_id) == 20,
    "horus::msg::JoystickInput::element_id must be at Rust offset 20");
static_assert(offsetof(horus::msg::JoystickInput, element_name) == 24,
    "horus::msg::JoystickInput::element_name must be at Rust offset 24");
static_assert(offsetof(horus::msg::JoystickInput, value) == 56,
    "horus::msg::JoystickInput::value must be at Rust offset 56");
static_assert(offsetof(horus::msg::JoystickInput, pressed) == 60,
    "horus::msg::JoystickInput::pressed must be at Rust offset 60");
static_assert(offsetof(horus::msg::JoystickInput, timestamp_ms) == 64,
    "horus::msg::JoystickInput::timestamp_ms must be at Rust offset 64");

// horus_robotics::KeyboardInput
static_assert(sizeof(horus::msg::KeyboardInput) == 72,
    "horus::msg::KeyboardInput must match Rust horus_robotics::KeyboardInput (72 bytes)");
static_assert(offsetof(horus::msg::KeyboardInput, key_name) == 0,
    "horus::msg::KeyboardInput::key_name must be at Rust offset 0");
static_assert(offsetof(horus::msg::KeyboardInput, code) == 32,
    "horus::msg::KeyboardInput::code must be at Rust offset 32");
static_assert(offsetof(horus::msg::KeyboardInput, modifier_flags) == 36,
    "horus::msg::KeyboardInput::modifier_flags must be at Rust offset 36");
static_assert(offsetof(horus::msg::KeyboardInput, pressed) == 40,
    "horus::msg::KeyboardInput::pressed must be at Rust offset 40");
static_assert(offsetof(horus::msg::KeyboardInput, timestamp_ms) == 48,
    "horus::msg::KeyboardInput::timestamp_ms must be at Rust offset 48");
static_assert(offsetof(horus::msg::KeyboardInput, _padding) == 56,
    "horus::msg::KeyboardInput::_padding must be at Rust offset 56");

// horus_robotics::Landmark
static_assert(sizeof(horus::msg::Landmark) == 16,
    "horus::msg::Landmark must match Rust horus_robotics::Landmark (16 bytes)");
static_assert(offsetof(horus::msg::Landmark, x) == 0,
    "horus::msg::Landmark::x must be at Rust offset 0");
static_assert(offsetof(horus::msg::Landmark, y) == 4,
    "horus::msg::Landmark::y must be at Rust offset 4");
static_assert(offsetof(horus::msg::Landmark, visibility) == 8,
    "horus::msg::Landmark::visibility must be at Rust offset 8");
static_assert(offsetof(horus::msg::Landmark, index) == 12,
    "horus::msg::Landmark::index must be at Rust offset 12");

// horus_robotics::Landmark3D
static_assert(sizeof(horus::msg::Landmark3D) == 20,
    "horus::msg::Landmark3D must match Rust horus_robotics::Landmark3D (20 bytes)");
static_assert(offsetof(horus::msg::Landmark3D, x) == 0,
    "horus::msg::Landmark3D::x must be at Rust offset 0");
static_assert(offsetof(horus::msg::Landmark3D, y) == 4,
    "horus::msg::Landmark3D::y must be at Rust offset 4");
static_assert(offsetof(horus::msg::Landmark3D, z) == 8,
    "horus::msg::Landmark3D::z must be at Rust offset 8");
static_assert(offsetof(horus::msg::Landmark3D, visibility) == 12,
    "horus::msg::Landmark3D::visibility must be at Rust offset 12");
static_assert(offsetof(horus::msg::Landmark3D, index) == 16,
    "horus::msg::Landmark3D::index must be at Rust offset 16");

// horus_robotics::LandmarkArray
static_assert(sizeof(horus::msg::LandmarkArray) == 40,
    "horus::msg::LandmarkArray must match Rust horus_robotics::LandmarkArray (40 bytes)");
static_assert(offsetof(horus::msg::LandmarkArray, num_landmarks) == 0,
    "horus::msg::LandmarkArray::num_landmarks must be at Rust offset 0");
static_assert(offsetof(horus::msg::LandmarkArray, dimension) == 4,
    "horus::msg::LandmarkArray::dimension must be at Rust offset 4");
static_assert(offsetof(horus::msg::LandmarkArray, instance_id) == 8,
    "horus::msg::LandmarkArray::instance_id must be at Rust offset 8");
static_assert(offsetof(horus::msg::LandmarkArray, confidence) == 12,
    "horus::msg::LandmarkArray::confidence must be at Rust offset 12");
static_assert(offsetof(horus::msg::LandmarkArray, timestamp_ns) == 16,
    "horus::msg::LandmarkArray::timestamp_ns must be at Rust offset 16");
static_assert(offsetof(horus::msg::LandmarkArray, bbox_x) == 24,
    "horus::msg::LandmarkArray::bbox_x must be at Rust offset 24");
static_assert(offsetof(horus::msg::LandmarkArray, bbox_y) == 28,
    "horus::msg::LandmarkArray::bbox_y must be at Rust offset 28");
static_assert(offsetof(horus::msg::LandmarkArray, bbox_width) == 32,
    "horus::msg::LandmarkArray::bbox_width must be at Rust offset 32");
static_assert(offsetof(horus::msg::LandmarkArray, bbox_height) == 36,
    "horus::msg::LandmarkArray::bbox_height must be at Rust offset 36");

// horus_robotics::LaserScan
static_assert(sizeof(horus::msg::LaserScan) == 1480,
    "horus::msg::LaserScan must match Rust horus_robotics::LaserScan (1480 bytes)");
static_assert(offsetof(horus::msg::LaserScan, ranges) == 0,
    "horus::msg::LaserScan::ranges must be at Rust offset 0");
static_assert(offsetof(horus::msg::LaserScan, angle_min) == 1440,
    "horus::msg::LaserScan::angle_min must be at Rust offset 1440");
static_assert(offsetof(horus::msg::LaserScan, angle_max) == 1444,
    "horus::msg::LaserScan::angle_max must be at Rust offset 1444");
static_assert(offsetof(horus::msg::LaserScan, range_min) == 1448,
    "horus::msg::LaserScan::range_min must be at Rust offset 1448");
static_assert(offsetof(horus::msg::LaserScan, range_max) == 1452,
    "horus::msg::LaserScan::range_max must be at Rust offset 1452");
static_assert(offsetof(horus::msg::LaserScan, angle_increment) == 1456,
    "horus::msg::LaserScan::angle_increment must be at Rust offset 1456");
static_assert(offsetof(horus::msg::LaserScan, time_increment) == 1460,
    "horus::msg::LaserScan::time_increment must be at Rust offset 1460");
static_assert(offsetof(horus::msg::LaserScan, scan_time) == 1464,
    "horus::msg::LaserScan::scan_time must be at Rust offset 1464");
static_assert(offsetof(horus::msg::LaserScan, timestamp_ns) == 1472,
    "horus::msg::LaserScan::timestamp_ns must be at Rust offset 1472");

// horus_robotics::MagneticField
static_assert(sizeof(horus::msg::MagneticField) == 136,
    "horus::msg::MagneticField must match Rust horus_robotics::MagneticField (136 bytes)");
static_assert(offsetof(horus::msg::MagneticField, magnetic_field) == 0,
    "horus::msg::MagneticField::magnetic_field must be at Rust offset 0");
static_assert(offsetof(horus::msg::MagneticField, magnetic_field_covariance) == 24,
    "horus::msg::MagneticField::magnetic_field_covariance must be at Rust offset 24");
static_assert(offsetof(horus::msg::MagneticField, frame_id) == 96,
    "horus::msg::MagneticField::frame_id must be at Rust offset 96");
static_assert(offsetof(horus::msg::MagneticField, timestamp_ns) == 128,
    "horus::msg::MagneticField::timestamp_ns must be at Rust offset 128");

// horus_robotics::MotorCommand
static_assert(sizeof(horus::msg::MotorCommand) == 56,
    "horus::msg::MotorCommand must match Rust horus_robotics::MotorCommand (56 bytes)");
static_assert(offsetof(horus::msg::MotorCommand, motor_id) == 0,
    "horus::msg::MotorCommand::motor_id must be at Rust offset 0");
static_assert(offsetof(horus::msg::MotorCommand, mode) == 1,
    "horus::msg::MotorCommand::mode must be at Rust offset 1");
static_assert(offsetof(horus::msg::MotorCommand, target) == 8,
    "horus::msg::MotorCommand::target must be at Rust offset 8");
static_assert(offsetof(horus::msg::MotorCommand, max_velocity) == 16,
    "horus::msg::MotorCommand::max_velocity must be at Rust offset 16");
static_assert(offsetof(horus::msg::MotorCommand, max_acceleration) == 24,
    "horus::msg::MotorCommand::max_acceleration must be at Rust offset 24");
static_assert(offsetof(horus::msg::MotorCommand, feed_forward) == 32,
    "horus::msg::MotorCommand::feed_forward must be at Rust offset 32");
static_assert(offsetof(horus::msg::MotorCommand, enable) == 40,
    "horus::msg::MotorCommand::enable must be at Rust offset 40");
static_assert(offsetof(horus::msg::MotorCommand, timestamp_ns) == 48,
    "horus::msg::MotorCommand::timestamp_ns must be at Rust offset 48");

// horus_robotics::NavGoal
static_assert(sizeof(horus::msg::NavGoal) == 72,
    "horus::msg::NavGoal must match Rust horus_robotics::NavGoal (72 bytes)");
static_assert(offsetof(horus::msg::NavGoal, target_pose) == 0,
    "horus::msg::NavGoal::target_pose must be at Rust offset 0");
static_assert(offsetof(horus::msg::NavGoal, tolerance_position) == 32,
    "horus::msg::NavGoal::tolerance_position must be at Rust offset 32");
static_assert(offsetof(horus::msg::NavGoal, tolerance_angle) == 40,
    "horus::msg::NavGoal::tolerance_angle must be at Rust offset 40");
static_assert(offsetof(horus::msg::NavGoal, timeout_seconds) == 48,
    "horus::msg::NavGoal::timeout_seconds must be at Rust offset 48");
static_assert(offsetof(horus::msg::NavGoal, priority) == 56,
    "horus::msg::NavGoal::priority must be at Rust offset 56");
static_assert(offsetof(horus::msg::NavGoal, goal_id) == 60,
    "horus::msg::NavGoal::goal_id must be at Rust offset 60");
static_assert(offsetof(horus::msg::NavGoal, timestamp_ns) == 64,
    "horus::msg::NavGoal::timestamp_ns must be at Rust offset 64");

// horus_robotics::NavPath
static_assert(sizeof(horus::msg::NavPath) == 26720,
    "horus::msg::NavPath must match Rust horus_robotics::NavPath (26720 bytes)");
static_assert(offsetof(horus::msg::NavPath, waypoints) == 0,
    "horus::msg::NavPath::waypoints must be at Rust offset 0");
static_assert(offsetof(horus::msg::NavPath, waypoint_count) == 26624,
    "horus::msg::NavPath::waypoint_count must be at Rust offset 26624");
static_assert(offsetof(horus::msg::NavPath, total_length) == 26632,
    "horus::msg::NavPath::total_length must be at Rust offset 26632");
static_assert(offsetof(horus::msg::NavPath, duration_seconds) == 26640,
    "horus::msg::NavPath::duration_seconds must be at Rust offset 26640");
static_assert(offsetof(horus::msg::NavPath, frame_id) == 26648,
    "horus::msg::NavPath::frame_id must be at Rust offset 26648");
static_assert(offsetof(horus::msg::NavPath, algorithm) == 26680,
    "horus::msg::NavPath::algorithm must be at Rust offset 26680");
static_assert(offsetof(horus::msg::NavPath, timestamp_ns) == 26712,
    "horus::msg::NavPath::timestamp_ns must be at Rust offset 26712");

// horus_robotics::NavSatFix
static_assert(sizeof(horus::msg::NavSatFix) == 128,
    "horus::msg::NavSatFix must match Rust horus_robotics::NavSatFix (128 bytes)");
static_assert(offsetof(horus::msg::NavSatFix, latitude) == 0,
    "horus::msg::NavSatFix::latitude must be at Rust offset 0");
static_assert(offsetof(horus::msg::NavSatFix, longitude) == 8,
    "horus::msg::NavSatFix::longitude must be at Rust offset 8");
static_assert(offsetof(horus::msg::NavSatFix, altitude) == 16,
    "horus::msg::NavSatFix::altitude must be at Rust offset 16");
static_assert(offsetof(horus::msg::NavSatFix, position_covariance) == 24,
    "horus::msg::NavSatFix::position_covariance must be at Rust offset 24");
static_assert(offsetof(horus::msg::NavSatFix, position_covariance_type) == 96,
    "horus::msg::NavSatFix::position_covariance_type must be at Rust offset 96");
static_assert(offsetof(horus::msg::NavSatFix, status) == 97,
    "horus::msg::NavSatFix::status must be at Rust offset 97");
static_assert(offsetof(horus::msg::NavSatFix, satellites_visible) == 98,
    "horus::msg::NavSatFix::satellites_visible must be at Rust offset 98");
static_assert(offsetof(horus::msg::NavSatFix, hdop) == 100,
    "horus::msg::NavSatFix::hdop must be at Rust offset 100");
static_assert(offsetof(horus::msg::NavSatFix, vdop) == 104,
    "horus::msg::NavSatFix::vdop must be at Rust offset 104");
static_assert(offsetof(horus::msg::NavSatFix, speed) == 108,
    "horus::msg::NavSatFix::speed must be at Rust offset 108");
static_assert(offsetof(horus::msg::NavSatFix, heading) == 112,
    "horus::msg::NavSatFix::heading must be at Rust offset 112");
static_assert(offsetof(horus::msg::NavSatFix, timestamp_ns) == 120,
    "horus::msg::NavSatFix::timestamp_ns must be at Rust offset 120");

// horus_robotics::Odometry
static_assert(sizeof(horus::msg::Odometry) == 736,
    "horus::msg::Odometry must match Rust horus_robotics::Odometry (736 bytes)");
static_assert(offsetof(horus::msg::Odometry, pose) == 0,
    "horus::msg::Odometry::pose must be at Rust offset 0");
static_assert(offsetof(horus::msg::Odometry, twist) == 32,
    "horus::msg::Odometry::twist must be at Rust offset 32");
static_assert(offsetof(horus::msg::Odometry, pose_covariance) == 88,
    "horus::msg::Odometry::pose_covariance must be at Rust offset 88");
static_assert(offsetof(horus::msg::Odometry, twist_covariance) == 376,
    "horus::msg::Odometry::twist_covariance must be at Rust offset 376");
static_assert(offsetof(horus::msg::Odometry, frame_id) == 664,
    "horus::msg::Odometry::frame_id must be at Rust offset 664");
static_assert(offsetof(horus::msg::Odometry, child_frame_id) == 696,
    "horus::msg::Odometry::child_frame_id must be at Rust offset 696");
static_assert(offsetof(horus::msg::Odometry, timestamp_ns) == 728,
    "horus::msg::Odometry::timestamp_ns must be at Rust offset 728");

// horus_robotics::PathPlan
static_assert(sizeof(horus::msg::PathPlan) == 3096,
    "horus::msg::PathPlan must match Rust horus_robotics::PathPlan (3096 bytes)");
static_assert(offsetof(horus::msg::PathPlan, waypoint_data) == 0,
    "horus::msg::PathPlan::waypoint_data must be at Rust offset 0");
static_assert(offsetof(horus::msg::PathPlan, goal_pose) == 3072,
    "horus::msg::PathPlan::goal_pose must be at Rust offset 3072");
static_assert(offsetof(horus::msg::PathPlan, waypoint_count) == 3084,
    "horus::msg::PathPlan::waypoint_count must be at Rust offset 3084");
static_assert(offsetof(horus::msg::PathPlan, timestamp_ns) == 3088,
    "horus::msg::PathPlan::timestamp_ns must be at Rust offset 3088");

// horus_robotics::PidConfig
static_assert(sizeof(horus::msg::PidConfig) == 64,
    "horus::msg::PidConfig must match Rust horus_robotics::PidConfig (64 bytes)");
static_assert(offsetof(horus::msg::PidConfig, controller_id) == 0,
    "horus::msg::PidConfig::controller_id must be at Rust offset 0");
static_assert(offsetof(horus::msg::PidConfig, kp) == 8,
    "horus::msg::PidConfig::kp must be at Rust offset 8");
static_assert(offsetof(horus::msg::PidConfig, ki) == 16,
    "horus::msg::PidConfig::ki must be at Rust offset 16");
static_assert(offsetof(horus::msg::PidConfig, kd) == 24,
    "horus::msg::PidConfig::kd must be at Rust offset 24");
static_assert(offsetof(horus::msg::PidConfig, integral_limit) == 32,
    "horus::msg::PidConfig::integral_limit must be at Rust offset 32");
static_assert(offsetof(horus::msg::PidConfig, output_limit) == 40,
    "horus::msg::PidConfig::output_limit must be at Rust offset 40");
static_assert(offsetof(horus::msg::PidConfig, anti_windup) == 48,
    "horus::msg::PidConfig::anti_windup must be at Rust offset 48");
static_assert(offsetof(horus::msg::PidConfig, timestamp_ns) == 56,
    "horus::msg::PidConfig::timestamp_ns must be at Rust offset 56");

// horus_robotics::RangeSensor
static_assert(sizeof(horus::msg::RangeSensor) == 32,
    "horus::msg::RangeSensor must match Rust horus_robotics::RangeSensor (32 bytes)");
static_assert(offsetof(horus::msg::RangeSensor, sensor_type) == 0,
    "horus::msg::RangeSensor::sensor_type must be at Rust offset 0");
static_assert(offsetof(horus::msg::RangeSensor, field_of_view) == 4,
    "horus::msg::RangeSensor::field_of_view must be at Rust offset 4");
static_assert(offsetof(horus::msg::RangeSensor, min_range) == 8,
    "horus::msg::RangeSensor::min_range must be at Rust offset 8");
static_assert(offsetof(horus::msg::RangeSensor, max_range) == 12,
    "horus::msg::RangeSensor::max_range must be at Rust offset 12");
static_assert(offsetof(horus::msg::RangeSensor, range) == 16,
    "horus::msg::RangeSensor::range must be at Rust offset 16");
static_assert(offsetof(horus::msg::RangeSensor, timestamp_ns) == 24,
    "horus::msg::RangeSensor::timestamp_ns must be at Rust offset 24");

// horus_robotics::SegmentationMask
static_assert(sizeof(horus::msg::SegmentationMask) == 64,
    "horus::msg::SegmentationMask must match Rust horus_robotics::SegmentationMask (64 bytes)");
static_assert(offsetof(horus::msg::SegmentationMask, width) == 0,
    "horus::msg::SegmentationMask::width must be at Rust offset 0");
static_assert(offsetof(horus::msg::SegmentationMask, height) == 4,
    "horus::msg::SegmentationMask::height must be at Rust offset 4");
static_assert(offsetof(horus::msg::SegmentationMask, num_classes) == 8,
    "horus::msg::SegmentationMask::num_classes must be at Rust offset 8");
static_assert(offsetof(horus::msg::SegmentationMask, mask_type) == 12,
    "horus::msg::SegmentationMask::mask_type must be at Rust offset 12");
static_assert(offsetof(horus::msg::SegmentationMask, timestamp_ns) == 16,
    "horus::msg::SegmentationMask::timestamp_ns must be at Rust offset 16");
static_assert(offsetof(horus::msg::SegmentationMask, seq) == 24,
    "horus::msg::SegmentationMask::seq must be at Rust offset 24");
static_assert(offsetof(horus::msg::SegmentationMask, frame_id) == 32,
    "horus::msg::SegmentationMask::frame_id must be at Rust offset 32");

// horus_robotics::ServoCommand
static_assert(sizeof(horus::msg::ServoCommand) == 24,
    "horus::msg::ServoCommand must match Rust horus_robotics::ServoCommand (24 bytes)");
static_assert(offsetof(horus::msg::ServoCommand, servo_id) == 0,
    "horus::msg::ServoCommand::servo_id must be at Rust offset 0");
static_assert(offsetof(horus::msg::ServoCommand, position) == 4,
    "horus::msg::ServoCommand::position must be at Rust offset 4");
static_assert(offsetof(horus::msg::ServoCommand, speed) == 8,
    "horus::msg::ServoCommand::speed must be at Rust offset 8");
static_assert(offsetof(horus::msg::ServoCommand, enable) == 12,
    "horus::msg::ServoCommand::enable must be at Rust offset 12");
static_assert(offsetof(horus::msg::ServoCommand, timestamp_ns) == 16,
    "horus::msg::ServoCommand::timestamp_ns must be at Rust offset 16");

// horus_robotics::Temperature
static_assert(sizeof(horus::msg::Temperature) == 56,
    "horus::msg::Temperature must match Rust horus_robotics::Temperature (56 bytes)");
static_assert(offsetof(horus::msg::Temperature, temperature) == 0,
    "horus::msg::Temperature::temperature must be at Rust offset 0");
static_assert(offsetof(horus::msg::Temperature, variance) == 8,
    "horus::msg::Temperature::variance must be at Rust offset 8");
static_assert(offsetof(horus::msg::Temperature, frame_id) == 16,
    "horus::msg::Temperature::frame_id must be at Rust offset 16");
static_assert(offsetof(horus::msg::Temperature, timestamp_ns) == 48,
    "horus::msg::Temperature::timestamp_ns must be at Rust offset 48");

// horus_robotics::TrackedObject
static_assert(sizeof(horus::msg::TrackedObject) == 96,
    "horus::msg::TrackedObject must match Rust horus_robotics::TrackedObject (96 bytes)");
static_assert(offsetof(horus::msg::TrackedObject, bbox) == 0,
    "horus::msg::TrackedObject::bbox must be at Rust offset 0");
static_assert(offsetof(horus::msg::TrackedObject, predicted_bbox) == 16,
    "horus::msg::TrackedObject::predicted_bbox must be at Rust offset 16");
static_assert(offsetof(horus::msg::TrackedObject, track_id) == 32,
    "horus::msg::TrackedObject::track_id must be at Rust offset 32");
static_assert(offsetof(horus::msg::TrackedObject, confidence) == 40,
    "horus::msg::TrackedObject::confidence must be at Rust offset 40");
static_assert(offsetof(horus::msg::TrackedObject, class_id) == 44,
    "horus::msg::TrackedObject::class_id must be at Rust offset 44");
static_assert(offsetof(horus::msg::TrackedObject, velocity_x) == 48,
    "horus::msg::TrackedObject::velocity_x must be at Rust offset 48");
static_assert(offsetof(horus::msg::TrackedObject, velocity_y) == 52,
    "horus::msg::TrackedObject::velocity_y must be at Rust offset 52");
static_assert(offsetof(horus::msg::TrackedObject, accel_x) == 56,
    "horus::msg::TrackedObject::accel_x must be at Rust offset 56");
static_assert(offsetof(horus::msg::TrackedObject, accel_y) == 60,
    "horus::msg::TrackedObject::accel_y must be at Rust offset 60");
static_assert(offsetof(horus::msg::TrackedObject, age) == 64,
    "horus::msg::TrackedObject::age must be at Rust offset 64");
static_assert(offsetof(horus::msg::TrackedObject, hits) == 68,
    "horus::msg::TrackedObject::hits must be at Rust offset 68");
static_assert(offsetof(horus::msg::TrackedObject, time_since_update) == 72,
    "horus::msg::TrackedObject::time_since_update must be at Rust offset 72");
static_assert(offsetof(horus::msg::TrackedObject, state) == 76,
    "horus::msg::TrackedObject::state must be at Rust offset 76");
static_assert(offsetof(horus::msg::TrackedObject, class_name) == 80,
    "horus::msg::TrackedObject::class_name must be at Rust offset 80");

// horus_robotics::TrackingHeader
static_assert(sizeof(horus::msg::TrackingHeader) == 32,
    "horus::msg::TrackingHeader must match Rust horus_robotics::TrackingHeader (32 bytes)");
static_assert(offsetof(horus::msg::TrackingHeader, num_tracks) == 0,
    "horus::msg::TrackingHeader::num_tracks must be at Rust offset 0");
static_assert(offsetof(horus::msg::TrackingHeader, frame_id) == 4,
    "horus::msg::TrackingHeader::frame_id must be at Rust offset 4");
static_assert(offsetof(horus::msg::TrackingHeader, timestamp_ns) == 8,
    "horus::msg::TrackingHeader::timestamp_ns must be at Rust offset 8");
static_assert(offsetof(horus::msg::TrackingHeader, total_tracks) == 16,
    "horus::msg::TrackingHeader::total_tracks must be at Rust offset 16");
static_assert(offsetof(horus::msg::TrackingHeader, active_tracks) == 24,
    "horus::msg::TrackingHeader::active_tracks must be at Rust offset 24");

// horus_robotics::TrajectoryPoint
static_assert(sizeof(horus::msg::TrajectoryPoint) == 136,
    "horus::msg::TrajectoryPoint must match Rust horus_robotics::TrajectoryPoint (136 bytes)");
static_assert(offsetof(horus::msg::TrajectoryPoint, position) == 0,
    "horus::msg::TrajectoryPoint::position must be at Rust offset 0");
static_assert(offsetof(horus::msg::TrajectoryPoint, velocity) == 24,
    "horus::msg::TrajectoryPoint::velocity must be at Rust offset 24");
static_assert(offsetof(horus::msg::TrajectoryPoint, acceleration) == 48,
    "horus::msg::TrajectoryPoint::acceleration must be at Rust offset 48");
static_assert(offsetof(horus::msg::TrajectoryPoint, orientation) == 72,
    "horus::msg::TrajectoryPoint::orientation must be at Rust offset 72");
static_assert(offsetof(horus::msg::TrajectoryPoint, angular_velocity) == 104,
    "horus::msg::TrajectoryPoint::angular_velocity must be at Rust offset 104");
static_assert(offsetof(horus::msg::TrajectoryPoint, time_from_start) == 128,
    "horus::msg::TrajectoryPoint::time_from_start must be at Rust offset 128");

// horus_robotics::VelocityObstacle
static_assert(sizeof(horus::msg::VelocityObstacle) == 48,
    "horus::msg::VelocityObstacle must match Rust horus_robotics::VelocityObstacle (48 bytes)");
static_assert(offsetof(horus::msg::VelocityObstacle, position) == 0,
    "horus::msg::VelocityObstacle::position must be at Rust offset 0");
static_assert(offsetof(horus::msg::VelocityObstacle, velocity) == 16,
    "horus::msg::VelocityObstacle::velocity must be at Rust offset 16");
static_assert(offsetof(horus::msg::VelocityObstacle, radius) == 32,
    "horus::msg::VelocityObstacle::radius must be at Rust offset 32");
static_assert(offsetof(horus::msg::VelocityObstacle, time_horizon) == 36,
    "horus::msg::VelocityObstacle::time_horizon must be at Rust offset 36");
static_assert(offsetof(horus::msg::VelocityObstacle, obstacle_id) == 40,
    "horus::msg::VelocityObstacle::obstacle_id must be at Rust offset 40");

// horus_robotics::Waypoint
static_assert(sizeof(horus::msg::Waypoint) == 104,
    "horus::msg::Waypoint must match Rust horus_robotics::Waypoint (104 bytes)");
static_assert(offsetof(horus::msg::Waypoint, pose) == 0,
    "horus::msg::Waypoint::pose must be at Rust offset 0");
static_assert(offsetof(horus::msg::Waypoint, velocity) == 32,
    "horus::msg::Waypoint::velocity must be at Rust offset 32");
static_assert(offsetof(horus::msg::Waypoint, time_from_start) == 88,
    "horus::msg::Waypoint::time_from_start must be at Rust offset 88");
static_assert(offsetof(horus::msg::Waypoint, curvature) == 96,
    "horus::msg::Waypoint::curvature must be at Rust offset 96");
static_assert(offsetof(horus::msg::Waypoint, stop_required) == 100,
    "horus::msg::Waypoint::stop_required must be at Rust offset 100");

// horus_robotics::WrenchStamped
static_assert(sizeof(horus::msg::WrenchStamped) == 112,
    "horus::msg::WrenchStamped must match Rust horus_robotics::WrenchStamped (112 bytes)");
static_assert(offsetof(horus::msg::WrenchStamped, force) == 0,
    "horus::msg::WrenchStamped::force must be at Rust offset 0");
static_assert(offsetof(horus::msg::WrenchStamped, torque) == 24,
    "horus::msg::WrenchStamped::torque must be at Rust offset 24");
static_assert(offsetof(horus::msg::WrenchStamped, point_of_application) == 48,
    "horus::msg::WrenchStamped::point_of_application must be at Rust offset 48");
static_assert(offsetof(horus::msg::WrenchStamped, frame_id) == 72,
    "horus::msg::WrenchStamped::frame_id must be at Rust offset 72");
static_assert(offsetof(horus::msg::WrenchStamped, timestamp_ns) == 104,
    "horus::msg::WrenchStamped::timestamp_ns must be at Rust offset 104");

// horus_types::Accel
static_assert(sizeof(horus::msg::Accel) == 56,
    "horus::msg::Accel must match Rust horus_types::Accel (56 bytes)");
static_assert(offsetof(horus::msg::Accel, linear) == 0,
    "horus::msg::Accel::linear must be at Rust offset 0");
static_assert(offsetof(horus::msg::Accel, angular) == 24,
    "horus::msg::Accel::angular must be at Rust offset 24");
static_assert(offsetof(horus::msg::Accel, timestamp_ns) == 48,
    "horus::msg::Accel::timestamp_ns must be at Rust offset 48");

// horus_types::AccelStamped
static_assert(sizeof(horus::msg::AccelStamped) == 96,
    "horus::msg::AccelStamped must match Rust horus_types::AccelStamped (96 bytes)");
static_assert(offsetof(horus::msg::AccelStamped, accel) == 0,
    "horus::msg::AccelStamped::accel must be at Rust offset 0");
static_assert(offsetof(horus::msg::AccelStamped, frame_id) == 56,
    "horus::msg::AccelStamped::frame_id must be at Rust offset 56");
static_assert(offsetof(horus::msg::AccelStamped, timestamp_ns) == 88,
    "horus::msg::AccelStamped::timestamp_ns must be at Rust offset 88");

// horus_types::Clock
static_assert(sizeof(horus::msg::Clock) == 40,
    "horus::msg::Clock must match Rust horus_types::Clock (40 bytes)");
static_assert(offsetof(horus::msg::Clock, clock_ns) == 0,
    "horus::msg::Clock::clock_ns must be at Rust offset 0");
static_assert(offsetof(horus::msg::Clock, realtime_ns) == 8,
    "horus::msg::Clock::realtime_ns must be at Rust offset 8");
static_assert(offsetof(horus::msg::Clock, sim_speed) == 16,
    "horus::msg::Clock::sim_speed must be at Rust offset 16");
static_assert(offsetof(horus::msg::Clock, paused) == 24,
    "horus::msg::Clock::paused must be at Rust offset 24");
static_assert(offsetof(horus::msg::Clock, source) == 25,
    "horus::msg::Clock::source must be at Rust offset 25");
static_assert(offsetof(horus::msg::Clock, timestamp_ns) == 32,
    "horus::msg::Clock::timestamp_ns must be at Rust offset 32");

// horus_types::DiagnosticStatus
static_assert(sizeof(horus::msg::DiagnosticStatus) == 176,
    "horus::msg::DiagnosticStatus must match Rust horus_types::DiagnosticStatus (176 bytes)");
static_assert(offsetof(horus::msg::DiagnosticStatus, level) == 0,
    "horus::msg::DiagnosticStatus::level must be at Rust offset 0");
static_assert(offsetof(horus::msg::DiagnosticStatus, code) == 4,
    "horus::msg::DiagnosticStatus::code must be at Rust offset 4");
static_assert(offsetof(horus::msg::DiagnosticStatus, message) == 8,
    "horus::msg::DiagnosticStatus::message must be at Rust offset 8");
static_assert(offsetof(horus::msg::DiagnosticStatus, component) == 136,
    "horus::msg::DiagnosticStatus::component must be at Rust offset 136");
static_assert(offsetof(horus::msg::DiagnosticStatus, timestamp_ns) == 168,
    "horus::msg::DiagnosticStatus::timestamp_ns must be at Rust offset 168");

// horus_types::EmergencyStop
static_assert(sizeof(horus::msg::EmergencyStop) == 112,
    "horus::msg::EmergencyStop must match Rust horus_types::EmergencyStop (112 bytes)");
static_assert(offsetof(horus::msg::EmergencyStop, engaged) == 0,
    "horus::msg::EmergencyStop::engaged must be at Rust offset 0");
static_assert(offsetof(horus::msg::EmergencyStop, reason) == 1,
    "horus::msg::EmergencyStop::reason must be at Rust offset 1");
static_assert(offsetof(horus::msg::EmergencyStop, source) == 65,
    "horus::msg::EmergencyStop::source must be at Rust offset 65");
static_assert(offsetof(horus::msg::EmergencyStop, auto_reset) == 97,
    "horus::msg::EmergencyStop::auto_reset must be at Rust offset 97");
static_assert(offsetof(horus::msg::EmergencyStop, timestamp_ns) == 104,
    "horus::msg::EmergencyStop::timestamp_ns must be at Rust offset 104");

// horus_types::Heartbeat
static_assert(sizeof(horus::msg::Heartbeat) == 72,
    "horus::msg::Heartbeat must match Rust horus_types::Heartbeat (72 bytes)");
static_assert(offsetof(horus::msg::Heartbeat, node_name) == 0,
    "horus::msg::Heartbeat::node_name must be at Rust offset 0");
static_assert(offsetof(horus::msg::Heartbeat, node_id) == 32,
    "horus::msg::Heartbeat::node_id must be at Rust offset 32");
static_assert(offsetof(horus::msg::Heartbeat, sequence) == 40,
    "horus::msg::Heartbeat::sequence must be at Rust offset 40");
static_assert(offsetof(horus::msg::Heartbeat, alive) == 48,
    "horus::msg::Heartbeat::alive must be at Rust offset 48");
static_assert(offsetof(horus::msg::Heartbeat, uptime) == 56,
    "horus::msg::Heartbeat::uptime must be at Rust offset 56");
static_assert(offsetof(horus::msg::Heartbeat, timestamp_ns) == 64,
    "horus::msg::Heartbeat::timestamp_ns must be at Rust offset 64");

// horus_types::NodeHeartbeat
static_assert(sizeof(horus::msg::NodeHeartbeat) == 48,
    "horus::msg::NodeHeartbeat must match Rust horus_types::NodeHeartbeat (48 bytes)");
static_assert(offsetof(horus::msg::NodeHeartbeat, state) == 0,
    "horus::msg::NodeHeartbeat::state must be at Rust offset 0");
static_assert(offsetof(horus::msg::NodeHeartbeat, health) == 1,
    "horus::msg::NodeHeartbeat::health must be at Rust offset 1");
static_assert(offsetof(horus::msg::NodeHeartbeat, tick_count) == 8,
    "horus::msg::NodeHeartbeat::tick_count must be at Rust offset 8");
static_assert(offsetof(horus::msg::NodeHeartbeat, target_rate_hz) == 16,
    "horus::msg::NodeHeartbeat::target_rate_hz must be at Rust offset 16");
static_assert(offsetof(horus::msg::NodeHeartbeat, actual_rate_hz) == 20,
    "horus::msg::NodeHeartbeat::actual_rate_hz must be at Rust offset 20");
static_assert(offsetof(horus::msg::NodeHeartbeat, error_count) == 24,
    "horus::msg::NodeHeartbeat::error_count must be at Rust offset 24");
static_assert(offsetof(horus::msg::NodeHeartbeat, last_tick_timestamp) == 32,
    "horus::msg::NodeHeartbeat::last_tick_timestamp must be at Rust offset 32");
static_assert(offsetof(horus::msg::NodeHeartbeat, heartbeat_timestamp) == 40,
    "horus::msg::NodeHeartbeat::heartbeat_timestamp must be at Rust offset 40");

// horus_types::Point3
static_assert(sizeof(horus::msg::Point3) == 24,
    "horus::msg::Point3 must match Rust horus_types::Point3 (24 bytes)");
static_assert(offsetof(horus::msg::Point3, x) == 0,
    "horus::msg::Point3::x must be at Rust offset 0");
static_assert(offsetof(horus::msg::Point3, y) == 8,
    "horus::msg::Point3::y must be at Rust offset 8");
static_assert(offsetof(horus::msg::Point3, z) == 16,
    "horus::msg::Point3::z must be at Rust offset 16");

// horus_types::Pose2D
static_assert(sizeof(horus::msg::Pose2D) == 32,
    "horus::msg::Pose2D must match Rust horus_types::Pose2D (32 bytes)");
static_assert(offsetof(horus::msg::Pose2D, x) == 0,
    "horus::msg::Pose2D::x must be at Rust offset 0");
static_assert(offsetof(horus::msg::Pose2D, y) == 8,
    "horus::msg::Pose2D::y must be at Rust offset 8");
static_assert(offsetof(horus::msg::Pose2D, theta) == 16,
    "horus::msg::Pose2D::theta must be at Rust offset 16");
static_assert(offsetof(horus::msg::Pose2D, timestamp_ns) == 24,
    "horus::msg::Pose2D::timestamp_ns must be at Rust offset 24");

// horus_types::Pose3D
static_assert(sizeof(horus::msg::Pose3D) == 64,
    "horus::msg::Pose3D must match Rust horus_types::Pose3D (64 bytes)");
static_assert(offsetof(horus::msg::Pose3D, position) == 0,
    "horus::msg::Pose3D::position must be at Rust offset 0");
static_assert(offsetof(horus::msg::Pose3D, orientation) == 24,
    "horus::msg::Pose3D::orientation must be at Rust offset 24");
static_assert(offsetof(horus::msg::Pose3D, timestamp_ns) == 56,
    "horus::msg::Pose3D::timestamp_ns must be at Rust offset 56");

// horus_types::PoseStamped
static_assert(sizeof(horus::msg::PoseStamped) == 104,
    "horus::msg::PoseStamped must match Rust horus_types::PoseStamped (104 bytes)");
static_assert(offsetof(horus::msg::PoseStamped, pose) == 0,
    "horus::msg::PoseStamped::pose must be at Rust offset 0");
static_assert(offsetof(horus::msg::PoseStamped, frame_id) == 64,
    "horus::msg::PoseStamped::frame_id must be at Rust offset 64");
static_assert(offsetof(horus::msg::PoseStamped, timestamp_ns) == 96,
    "horus::msg::PoseStamped::timestamp_ns must be at Rust offset 96");

// horus_types::PoseWithCovariance
static_assert(sizeof(horus::msg::PoseWithCovariance) == 392,
    "horus::msg::PoseWithCovariance must match Rust horus_types::PoseWithCovariance (392 bytes)");
static_assert(offsetof(horus::msg::PoseWithCovariance, pose) == 0,
    "horus::msg::PoseWithCovariance::pose must be at Rust offset 0");
static_assert(offsetof(horus::msg::PoseWithCovariance, covariance) == 64,
    "horus::msg::PoseWithCovariance::covariance must be at Rust offset 64");
static_assert(offsetof(horus::msg::PoseWithCovariance, frame_id) == 352,
    "horus::msg::PoseWithCovariance::frame_id must be at Rust offset 352");
static_assert(offsetof(horus::msg::PoseWithCovariance, timestamp_ns) == 384,
    "horus::msg::PoseWithCovariance::timestamp_ns must be at Rust offset 384");

// horus_types::Quaternion
static_assert(sizeof(horus::msg::Quaternion) == 32,
    "horus::msg::Quaternion must match Rust horus_types::Quaternion (32 bytes)");
static_assert(offsetof(horus::msg::Quaternion, x) == 0,
    "horus::msg::Quaternion::x must be at Rust offset 0");
static_assert(offsetof(horus::msg::Quaternion, y) == 8,
    "horus::msg::Quaternion::y must be at Rust offset 8");
static_assert(offsetof(horus::msg::Quaternion, z) == 16,
    "horus::msg::Quaternion::z must be at Rust offset 16");
static_assert(offsetof(horus::msg::Quaternion, w) == 24,
    "horus::msg::Quaternion::w must be at Rust offset 24");

// horus_types::ResourceUsage
static_assert(sizeof(horus::msg::ResourceUsage) == 72,
    "horus::msg::ResourceUsage must match Rust horus_types::ResourceUsage (72 bytes)");
static_assert(offsetof(horus::msg::ResourceUsage, cpu_percent) == 0,
    "horus::msg::ResourceUsage::cpu_percent must be at Rust offset 0");
static_assert(offsetof(horus::msg::ResourceUsage, memory_bytes) == 8,
    "horus::msg::ResourceUsage::memory_bytes must be at Rust offset 8");
static_assert(offsetof(horus::msg::ResourceUsage, memory_percent) == 16,
    "horus::msg::ResourceUsage::memory_percent must be at Rust offset 16");
static_assert(offsetof(horus::msg::ResourceUsage, disk_bytes) == 24,
    "horus::msg::ResourceUsage::disk_bytes must be at Rust offset 24");
static_assert(offsetof(horus::msg::ResourceUsage, disk_percent) == 32,
    "horus::msg::ResourceUsage::disk_percent must be at Rust offset 32");
static_assert(offsetof(horus::msg::ResourceUsage, network_tx_bytes) == 40,
    "horus::msg::ResourceUsage::network_tx_bytes must be at Rust offset 40");
static_assert(offsetof(horus::msg::ResourceUsage, network_rx_bytes) == 48,
    "horus::msg::ResourceUsage::network_rx_bytes must be at Rust offset 48");
static_assert(offsetof(horus::msg::ResourceUsage, temperature) == 56,
    "horus::msg::ResourceUsage::temperature must be at Rust offset 56");
static_assert(offsetof(horus::msg::ResourceUsage, thread_count) == 60,
    "horus::msg::ResourceUsage::thread_count must be at Rust offset 60");
static_assert(offsetof(horus::msg::ResourceUsage, timestamp_ns) == 64,
    "horus::msg::ResourceUsage::timestamp_ns must be at Rust offset 64");

// horus_types::SafetyStatus
static_assert(sizeof(horus::msg::SafetyStatus) == 24,
    "horus::msg::SafetyStatus must match Rust horus_types::SafetyStatus (24 bytes)");
static_assert(offsetof(horus::msg::SafetyStatus, enabled) == 0,
    "horus::msg::SafetyStatus::enabled must be at Rust offset 0");
static_assert(offsetof(horus::msg::SafetyStatus, estop_engaged) == 1,
    "horus::msg::SafetyStatus::estop_engaged must be at Rust offset 1");
static_assert(offsetof(horus::msg::SafetyStatus, watchdog_ok) == 2,
    "horus::msg::SafetyStatus::watchdog_ok must be at Rust offset 2");
static_assert(offsetof(horus::msg::SafetyStatus, limits_ok) == 3,
    "horus::msg::SafetyStatus::limits_ok must be at Rust offset 3");
static_assert(offsetof(horus::msg::SafetyStatus, comms_ok) == 4,
    "horus::msg::SafetyStatus::comms_ok must be at Rust offset 4");
static_assert(offsetof(horus::msg::SafetyStatus, mode) == 5,
    "horus::msg::SafetyStatus::mode must be at Rust offset 5");
static_assert(offsetof(horus::msg::SafetyStatus, fault_code) == 8,
    "horus::msg::SafetyStatus::fault_code must be at Rust offset 8");
static_assert(offsetof(horus::msg::SafetyStatus, timestamp_ns) == 16,
    "horus::msg::SafetyStatus::timestamp_ns must be at Rust offset 16");

// horus_types::TimeReference
static_assert(sizeof(horus::msg::TimeReference) == 56,
    "horus::msg::TimeReference must match Rust horus_types::TimeReference (56 bytes)");
static_assert(offsetof(horus::msg::TimeReference, time_ref_ns) == 0,
    "horus::msg::TimeReference::time_ref_ns must be at Rust offset 0");
static_assert(offsetof(horus::msg::TimeReference, source) == 8,
    "horus::msg::TimeReference::source must be at Rust offset 8");
static_assert(offsetof(horus::msg::TimeReference, offset_ns) == 40,
    "horus::msg::TimeReference::offset_ns must be at Rust offset 40");
static_assert(offsetof(horus::msg::TimeReference, timestamp_ns) == 48,
    "horus::msg::TimeReference::timestamp_ns must be at Rust offset 48");

// horus_types::TransformStamped
static_assert(sizeof(horus::msg::TransformStamped) == 64,
    "horus::msg::TransformStamped must match Rust horus_types::TransformStamped (64 bytes)");
static_assert(offsetof(horus::msg::TransformStamped, translation) == 0,
    "horus::msg::TransformStamped::translation must be at Rust offset 0");
static_assert(offsetof(horus::msg::TransformStamped, rotation) == 24,
    "horus::msg::TransformStamped::rotation must be at Rust offset 24");
static_assert(offsetof(horus::msg::TransformStamped, timestamp_ns) == 56,
    "horus::msg::TransformStamped::timestamp_ns must be at Rust offset 56");

// horus_types::Twist
static_assert(sizeof(horus::msg::Twist) == 56,
    "horus::msg::Twist must match Rust horus_types::Twist (56 bytes)");
static_assert(offsetof(horus::msg::Twist, linear) == 0,
    "horus::msg::Twist::linear must be at Rust offset 0");
static_assert(offsetof(horus::msg::Twist, angular) == 24,
    "horus::msg::Twist::angular must be at Rust offset 24");
static_assert(offsetof(horus::msg::Twist, timestamp_ns) == 48,
    "horus::msg::Twist::timestamp_ns must be at Rust offset 48");

// horus_types::TwistWithCovariance
static_assert(sizeof(horus::msg::TwistWithCovariance) == 384,
    "horus::msg::TwistWithCovariance must match Rust horus_types::TwistWithCovariance (384 bytes)");
static_assert(offsetof(horus::msg::TwistWithCovariance, twist) == 0,
    "horus::msg::TwistWithCovariance::twist must be at Rust offset 0");
static_assert(offsetof(horus::msg::TwistWithCovariance, covariance) == 56,
    "horus::msg::TwistWithCovariance::covariance must be at Rust offset 56");
static_assert(offsetof(horus::msg::TwistWithCovariance, frame_id) == 344,
    "horus::msg::TwistWithCovariance::frame_id must be at Rust offset 344");
static_assert(offsetof(horus::msg::TwistWithCovariance, timestamp_ns) == 376,
    "horus::msg::TwistWithCovariance::timestamp_ns must be at Rust offset 376");

// horus_types::Vector3
static_assert(sizeof(horus::msg::Vector3) == 24,
    "horus::msg::Vector3 must match Rust horus_types::Vector3 (24 bytes)");
static_assert(offsetof(horus::msg::Vector3, x) == 0,
    "horus::msg::Vector3::x must be at Rust offset 0");
static_assert(offsetof(horus::msg::Vector3, y) == 8,
    "horus::msg::Vector3::y must be at Rust offset 8");
static_assert(offsetof(horus::msg::Vector3, z) == 16,
    "horus::msg::Vector3::z must be at Rust offset 16");
