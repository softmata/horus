// HORUS Topic Implementation — template specializations for all 11 message types.
//
// Each message type gets a Publisher<T> and Subscriber<T> specialization
// that routes through the type-specific C API functions.
#pragma once

#include "../horus_c.h"
#include "../topic.hpp"
#include "../msg/control.hpp"
#include "../msg/sensor.hpp"
#include "../msg/geometry.hpp"
#include "../msg/navigation.hpp"
#include "../msg/diagnostics.hpp"

#include <string>
#include <optional>
#include <cstring>

namespace horus {

// ─── Macro: Generate Publisher<T> + Subscriber<T> specialization ────────────
//
// For any Pod message type where C struct == Rust struct (identical #[repr(C)]).
// Uses void*-based C API (horus_publisher_<name>_send takes void*).

#define HORUS_TOPIC_IMPL(CppType, c_snake) \
template<> \
class Publisher<CppType> { \
public: \
    explicit Publisher(std::string_view topic_name) { \
        std::string s(topic_name); \
        inner_ = horus_publisher_##c_snake##_new(s.c_str()); \
        name_ = std::string(topic_name); \
    } \
    ~Publisher() { if (inner_) horus_publisher_##c_snake##_destroy(inner_); } \
    Publisher(Publisher&& o) noexcept : inner_(o.inner_), name_(std::move(o.name_)) { o.inner_ = nullptr; } \
    Publisher& operator=(Publisher&& o) noexcept { \
        if (this != &o) { if (inner_) horus_publisher_##c_snake##_destroy(inner_); inner_ = o.inner_; name_ = std::move(o.name_); o.inner_ = nullptr; } \
        return *this; \
    } \
    Publisher(const Publisher&) = delete; \
    Publisher& operator=(const Publisher&) = delete; \
    [[nodiscard]] LoanedSample<CppType> loan() { return LoanedSample<CppType>(); } \
    void publish(LoanedSample<CppType>&& sample) { send(*sample); } \
    void send(const CppType& msg) { \
        if (inner_) horus_publisher_##c_snake##_send(inner_, &msg); \
    } \
    const std::string& name() const { return name_; } \
    bool is_valid() const { return inner_ != nullptr; } \
private: \
    HorusPublisher* inner_ = nullptr; \
    std::string name_; \
}; \
\
template<> \
class Subscriber<CppType> { \
public: \
    explicit Subscriber(std::string_view topic_name) { \
        std::string s(topic_name); \
        inner_ = horus_subscriber_##c_snake##_new(s.c_str()); \
        name_ = std::string(topic_name); \
    } \
    ~Subscriber() { if (inner_) horus_subscriber_##c_snake##_destroy(inner_); } \
    Subscriber(Subscriber&& o) noexcept : inner_(o.inner_), name_(std::move(o.name_)) { o.inner_ = nullptr; } \
    Subscriber& operator=(Subscriber&& o) noexcept { \
        if (this != &o) { if (inner_) horus_subscriber_##c_snake##_destroy(inner_); inner_ = o.inner_; name_ = std::move(o.name_); o.inner_ = nullptr; } \
        return *this; \
    } \
    Subscriber(const Subscriber&) = delete; \
    Subscriber& operator=(const Subscriber&) = delete; \
    std::optional<BorrowedSample<CppType>> recv() { \
        if (!inner_) return std::nullopt; \
        CppType msg; \
        std::memset(&msg, 0, sizeof(msg)); \
        if (horus_subscriber_##c_snake##_recv(inner_, &msg)) { \
            return BorrowedSample<CppType>(msg); \
        } \
        return std::nullopt; \
    } \
    bool has_msg() const { \
        return inner_ ? horus_subscriber_##c_snake##_has_msg(inner_) : false; \
    } \
    const std::string& name() const { return name_; } \
    bool is_valid() const { return inner_ != nullptr; } \
private: \
    HorusSubscriber* inner_ = nullptr; \
    std::string name_; \
};

// ─── All 11 Message Type Specializations ────────────────────────────────────

// CmdVel uses typed C API (HorusCmdVel struct) — keep hand-written for backward compat
template<>
class Publisher<msg::CmdVel> {
public:
    explicit Publisher(std::string_view topic_name) {
        std::string s(topic_name);
        inner_ = horus_publisher_cmd_vel_new(s.c_str());
        name_ = std::string(topic_name);
    }
    ~Publisher() { if (inner_) horus_publisher_cmd_vel_destroy(inner_); }
    Publisher(Publisher&& o) noexcept : inner_(o.inner_), name_(std::move(o.name_)) { o.inner_ = nullptr; }
    Publisher& operator=(Publisher&& o) noexcept {
        if (this != &o) { if (inner_) horus_publisher_cmd_vel_destroy(inner_); inner_ = o.inner_; name_ = std::move(o.name_); o.inner_ = nullptr; }
        return *this;
    }
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;
    [[nodiscard]] LoanedSample<msg::CmdVel> loan() { return LoanedSample<msg::CmdVel>(); }
    void publish(LoanedSample<msg::CmdVel>&& sample) { send(*sample); }
    void send(const msg::CmdVel& msg) {
        if (!inner_) return;
        HorusCmdVel c_msg;
        c_msg.timestamp_ns = msg.timestamp_ns;
        c_msg.linear = msg.linear;
        c_msg.angular = msg.angular;
        horus_publisher_cmd_vel_send(inner_, &c_msg);
    }
    const std::string& name() const { return name_; }
    bool is_valid() const { return inner_ != nullptr; }
private:
    HorusPublisher* inner_ = nullptr;
    std::string name_;
};

template<>
class Subscriber<msg::CmdVel> {
public:
    explicit Subscriber(std::string_view topic_name) {
        std::string s(topic_name);
        inner_ = horus_subscriber_cmd_vel_new(s.c_str());
        name_ = std::string(topic_name);
    }
    ~Subscriber() { if (inner_) horus_subscriber_cmd_vel_destroy(inner_); }
    Subscriber(Subscriber&& o) noexcept : inner_(o.inner_), name_(std::move(o.name_)) { o.inner_ = nullptr; }
    Subscriber& operator=(Subscriber&& o) noexcept {
        if (this != &o) { if (inner_) horus_subscriber_cmd_vel_destroy(inner_); inner_ = o.inner_; name_ = std::move(o.name_); o.inner_ = nullptr; }
        return *this;
    }
    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;
    std::optional<BorrowedSample<msg::CmdVel>> recv() {
        if (!inner_) return std::nullopt;
        HorusCmdVel c_msg;
        if (horus_subscriber_cmd_vel_recv(inner_, &c_msg)) {
            msg::CmdVel msg;
            msg.timestamp_ns = c_msg.timestamp_ns;
            msg.linear = c_msg.linear;
            msg.angular = c_msg.angular;
            return BorrowedSample<msg::CmdVel>(msg);
        }
        return std::nullopt;
    }
    bool has_msg() const { return inner_ ? horus_subscriber_cmd_vel_has_msg(inner_) : false; }
    const std::string& name() const { return name_; }
    bool is_valid() const { return inner_ != nullptr; }
private:
    HorusSubscriber* inner_ = nullptr;
    std::string name_;
};

// ─── All Pod Message Types ──────────────────────────────────────────────────
// Core 10
HORUS_TOPIC_IMPL(msg::LaserScan,        laser_scan)
HORUS_TOPIC_IMPL(msg::Imu,              imu)
HORUS_TOPIC_IMPL(msg::Odometry,         odometry)
HORUS_TOPIC_IMPL(msg::JointState,       joint_state)
HORUS_TOPIC_IMPL(msg::Twist,            twist)
HORUS_TOPIC_IMPL(msg::Pose2D,           pose2d)
HORUS_TOPIC_IMPL(msg::TransformStamped, transform_stamped)
HORUS_TOPIC_IMPL(msg::NavGoal,          nav_goal)
HORUS_TOPIC_IMPL(msg::Heartbeat,        heartbeat)
HORUS_TOPIC_IMPL(msg::EmergencyStop,    emergency_stop)
// Sensor (7)
HORUS_TOPIC_IMPL(msg::RangeSensor,      range_sensor)
HORUS_TOPIC_IMPL(msg::BatteryState,     battery_state)
HORUS_TOPIC_IMPL(msg::NavSatFix,        nav_sat_fix)
HORUS_TOPIC_IMPL(msg::MagneticField,    magnetic_field)
HORUS_TOPIC_IMPL(msg::Temperature,      temperature)
HORUS_TOPIC_IMPL(msg::FluidPressure,    fluid_pressure)
HORUS_TOPIC_IMPL(msg::Illuminance,      illuminance)
// Control (6)
HORUS_TOPIC_IMPL(msg::MotorCommand,              motor_command)
HORUS_TOPIC_IMPL(msg::DifferentialDriveCommand,   differential_drive_command)
HORUS_TOPIC_IMPL(msg::ServoCommand,               servo_command)
HORUS_TOPIC_IMPL(msg::PidConfig,                  pid_config)
HORUS_TOPIC_IMPL(msg::TrajectoryPoint,            trajectory_point)
HORUS_TOPIC_IMPL(msg::JointCommand,               joint_command)
// Geometry (9)
HORUS_TOPIC_IMPL(msg::Point3,              point3)
HORUS_TOPIC_IMPL(msg::Vector3,             vector3)
HORUS_TOPIC_IMPL(msg::Quaternion,          quaternion)
HORUS_TOPIC_IMPL(msg::Pose3D,              pose3d)
HORUS_TOPIC_IMPL(msg::PoseStamped,         pose_stamped)
HORUS_TOPIC_IMPL(msg::PoseWithCovariance,  pose_with_covariance)
HORUS_TOPIC_IMPL(msg::TwistWithCovariance, twist_with_covariance)
HORUS_TOPIC_IMPL(msg::Accel,               accel)
HORUS_TOPIC_IMPL(msg::AccelStamped,        accel_stamped)
// Detection (4)
HORUS_TOPIC_IMPL(msg::BoundingBox2D, bounding_box_2d)
HORUS_TOPIC_IMPL(msg::BoundingBox3D, bounding_box_3d)
HORUS_TOPIC_IMPL(msg::Detection,     detection)
HORUS_TOPIC_IMPL(msg::Detection3D,   detection_3d)
// Vision (3)
HORUS_TOPIC_IMPL(msg::CameraInfo,       camera_info)
HORUS_TOPIC_IMPL(msg::RegionOfInterest, region_of_interest)
HORUS_TOPIC_IMPL(msg::StereoInfo,       stereo_info)
// Navigation (5)
HORUS_TOPIC_IMPL(msg::GoalResult,       goal_result)
HORUS_TOPIC_IMPL(msg::Waypoint,         waypoint)
HORUS_TOPIC_IMPL(msg::VelocityObstacle, velocity_obstacle)
HORUS_TOPIC_IMPL(msg::PathPlan,         path_plan)
// Diagnostics (6)
HORUS_TOPIC_IMPL(msg::DiagnosticStatus, diagnostic_status)
HORUS_TOPIC_IMPL(msg::ResourceUsage,    resource_usage)
HORUS_TOPIC_IMPL(msg::NodeHeartbeat,    node_heartbeat)
HORUS_TOPIC_IMPL(msg::SafetyStatus,     safety_status)
HORUS_TOPIC_IMPL(msg::DiagnosticValue,  diagnostic_value)
HORUS_TOPIC_IMPL(msg::DiagnosticReport, diagnostic_report)
// Force (5)
HORUS_TOPIC_IMPL(msg::WrenchStamped,      wrench_stamped)
HORUS_TOPIC_IMPL(msg::ForceCommand,       force_command)
HORUS_TOPIC_IMPL(msg::ContactInfo,        contact_info)
HORUS_TOPIC_IMPL(msg::HapticFeedback,     haptic_feedback)
HORUS_TOPIC_IMPL(msg::ImpedanceParameters, impedance_parameters)
// Tracking (3)
HORUS_TOPIC_IMPL(msg::TrackedObject,    tracked_object)
HORUS_TOPIC_IMPL(msg::TrackingHeader,   tracking_header)
HORUS_TOPIC_IMPL(msg::SegmentationMask, segmentation_mask)
// Landmark (3)
HORUS_TOPIC_IMPL(msg::Landmark,      landmark)
HORUS_TOPIC_IMPL(msg::Landmark3D,    landmark_3d)
HORUS_TOPIC_IMPL(msg::LandmarkArray, landmark_array)
// Input (2)
HORUS_TOPIC_IMPL(msg::KeyboardInput, keyboard_input)
HORUS_TOPIC_IMPL(msg::JoystickInput, joystick_input)
// Audio (1)
HORUS_TOPIC_IMPL(msg::AudioFrame, audio_frame)
// Clock (2)
HORUS_TOPIC_IMPL(msg::Clock,         clock)
HORUS_TOPIC_IMPL(msg::TimeReference, time_reference)
// Perception (2)
HORUS_TOPIC_IMPL(msg::PointField,     point_field)
HORUS_TOPIC_IMPL(msg::PlaneDetection, plane_detection)

#undef HORUS_TOPIC_IMPL

// ─── Scheduler::advertise/subscribe template definitions ────────────────────
// Must be after all HORUS_TOPIC_IMPL specializations so the compiler can find them.

template<typename T>
Publisher<T> Scheduler::advertise(std::string_view topic_name) {
    return Publisher<T>(topic_name);
}

template<typename T>
Subscriber<T> Scheduler::subscribe(std::string_view topic_name) {
    return Subscriber<T>(topic_name);
}

} // namespace horus
