/* HORUS C API — callable from C and C++.
 *
 * This header declares all #[no_mangle] extern "C" functions exported by
 * libhorus_cpp. Link against libhorus_cpp.so (or .a) to use.
 *
 * GENERATED from horus_cpp/src/c_api.rs — keep in sync.
 */
#ifndef HORUS_C_H
#define HORUS_C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── ABI Version ─────────────────────────────────────────────────────────── */

#define HORUS_CPP_ABI_VERSION 1

uint32_t horus_get_abi_version(void);

/* ── Opaque Types ────────────────────────────────────────────────────────── */

typedef struct HorusScheduler HorusScheduler;
typedef struct HorusNodeBuilder HorusNodeBuilder;
typedef struct HorusPublisher HorusPublisher;
typedef struct HorusSubscriber HorusSubscriber;

/* ── Scheduler ───────────────────────────────────────────────────────────── */

HorusScheduler* horus_scheduler_new(void);
void            horus_scheduler_destroy(HorusScheduler* sched);
void            horus_scheduler_tick_rate(HorusScheduler* sched, double hz);
void            horus_scheduler_name(HorusScheduler* sched, const char* name);
void            horus_scheduler_prefer_rt(HorusScheduler* sched);
void            horus_scheduler_require_rt(HorusScheduler* sched);
void            horus_scheduler_deterministic(HorusScheduler* sched, bool enabled);
void            horus_scheduler_verbose(HorusScheduler* sched, bool enabled);
void            horus_scheduler_watchdog(HorusScheduler* sched, uint64_t timeout_us);
void            horus_scheduler_blackbox(HorusScheduler* sched, size_t size_mb);
void            horus_scheduler_enable_network(HorusScheduler* sched);
void            horus_scheduler_stop(const HorusScheduler* sched);
bool            horus_scheduler_is_running(const HorusScheduler* sched);
bool            horus_scheduler_has_full_rt(const HorusScheduler* sched);
int             horus_scheduler_get_name(const HorusScheduler* sched, char* buf, size_t buf_len);
int             horus_scheduler_tick_once(HorusScheduler* sched);
uint32_t        horus_scheduler_node_count(const HorusScheduler* sched);
int             horus_scheduler_node_name_at(const HorusScheduler* sched, uint32_t index, char* buf, size_t buf_len);

/* ── Node Builder ────────────────────────────────────────────────────────── */

HorusNodeBuilder* horus_node_builder_new(const char* name);
void              horus_node_builder_rate(HorusNodeBuilder* builder, double hz);
void              horus_node_builder_budget(HorusNodeBuilder* builder, uint64_t budget_us);
void              horus_node_builder_deadline(HorusNodeBuilder* builder, uint64_t deadline_us);
void              horus_node_builder_on_miss(HorusNodeBuilder* builder, uint8_t policy);
void              horus_node_builder_compute(HorusNodeBuilder* builder);
void              horus_node_builder_order(HorusNodeBuilder* builder, uint32_t order);
void              horus_node_builder_set_tick(HorusNodeBuilder* builder, void (*callback)(void));
void              horus_node_builder_set_init(HorusNodeBuilder* builder, void (*callback)(void));
void              horus_node_builder_set_safe_state(HorusNodeBuilder* builder, void (*callback)(void));
void              horus_node_builder_set_shutdown(HorusNodeBuilder* builder, void (*callback)(void));
void              horus_node_builder_async_io(HorusNodeBuilder* builder);
void              horus_node_builder_on_topic(HorusNodeBuilder* builder, const char* topic);
void              horus_node_builder_pin_core(HorusNodeBuilder* builder, size_t cpu_id);
void              horus_node_builder_priority(HorusNodeBuilder* builder, int32_t prio);
void              horus_node_builder_watchdog(HorusNodeBuilder* builder, uint64_t timeout_us);
int               horus_node_builder_build(HorusNodeBuilder* builder, HorusScheduler* sched);

/* ── CmdVel Topic ────────────────────────────────────────────────────────── */

typedef struct {
    uint64_t timestamp_ns;
    float    linear;   /* m/s */
    float    angular;  /* rad/s */
} HorusCmdVel;

HorusPublisher*  horus_publisher_cmd_vel_new(const char* topic_name);
void             horus_publisher_cmd_vel_destroy(HorusPublisher* pub_);
void             horus_publisher_cmd_vel_send(const HorusPublisher* pub_, const HorusCmdVel* msg);

HorusSubscriber* horus_subscriber_cmd_vel_new(const char* topic_name);
void             horus_subscriber_cmd_vel_destroy(HorusSubscriber* sub);
int              horus_subscriber_cmd_vel_recv(const HorusSubscriber* sub, HorusCmdVel* out);
bool             horus_subscriber_cmd_vel_has_msg(const HorusSubscriber* sub);

/* ── Pod Message Topics (all 11 types) ───────────────────────────────────── */
/* All Pod types use void* for send/recv — C++ templates handle type safety. */
/* Layout: identical #[repr(C)] in Rust and C/C++. Direct memcpy.            */

/* LaserScan */
HorusPublisher*  horus_publisher_laser_scan_new(const char* topic_name);
void             horus_publisher_laser_scan_destroy(HorusPublisher* pub_);
void             horus_publisher_laser_scan_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_laser_scan_new(const char* topic_name);
void             horus_subscriber_laser_scan_destroy(HorusSubscriber* sub);
int              horus_subscriber_laser_scan_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_laser_scan_has_msg(const HorusSubscriber* sub);

/* Imu */
HorusPublisher*  horus_publisher_imu_new(const char* topic_name);
void             horus_publisher_imu_destroy(HorusPublisher* pub_);
void             horus_publisher_imu_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_imu_new(const char* topic_name);
void             horus_subscriber_imu_destroy(HorusSubscriber* sub);
int              horus_subscriber_imu_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_imu_has_msg(const HorusSubscriber* sub);

/* Odometry */
HorusPublisher*  horus_publisher_odometry_new(const char* topic_name);
void             horus_publisher_odometry_destroy(HorusPublisher* pub_);
void             horus_publisher_odometry_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_odometry_new(const char* topic_name);
void             horus_subscriber_odometry_destroy(HorusSubscriber* sub);
int              horus_subscriber_odometry_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_odometry_has_msg(const HorusSubscriber* sub);

/* JointState */
HorusPublisher*  horus_publisher_joint_state_new(const char* topic_name);
void             horus_publisher_joint_state_destroy(HorusPublisher* pub_);
void             horus_publisher_joint_state_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_joint_state_new(const char* topic_name);
void             horus_subscriber_joint_state_destroy(HorusSubscriber* sub);
int              horus_subscriber_joint_state_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_joint_state_has_msg(const HorusSubscriber* sub);

/* Twist */
HorusPublisher*  horus_publisher_twist_new(const char* topic_name);
void             horus_publisher_twist_destroy(HorusPublisher* pub_);
void             horus_publisher_twist_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_twist_new(const char* topic_name);
void             horus_subscriber_twist_destroy(HorusSubscriber* sub);
int              horus_subscriber_twist_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_twist_has_msg(const HorusSubscriber* sub);

/* Pose2D */
HorusPublisher*  horus_publisher_pose2d_new(const char* topic_name);
void             horus_publisher_pose2d_destroy(HorusPublisher* pub_);
void             horus_publisher_pose2d_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_pose2d_new(const char* topic_name);
void             horus_subscriber_pose2d_destroy(HorusSubscriber* sub);
int              horus_subscriber_pose2d_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_pose2d_has_msg(const HorusSubscriber* sub);

/* TransformStamped */
HorusPublisher*  horus_publisher_transform_stamped_new(const char* topic_name);
void             horus_publisher_transform_stamped_destroy(HorusPublisher* pub_);
void             horus_publisher_transform_stamped_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_transform_stamped_new(const char* topic_name);
void             horus_subscriber_transform_stamped_destroy(HorusSubscriber* sub);
int              horus_subscriber_transform_stamped_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_transform_stamped_has_msg(const HorusSubscriber* sub);

/* NavGoal */
HorusPublisher*  horus_publisher_nav_goal_new(const char* topic_name);
void             horus_publisher_nav_goal_destroy(HorusPublisher* pub_);
void             horus_publisher_nav_goal_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_nav_goal_new(const char* topic_name);
void             horus_subscriber_nav_goal_destroy(HorusSubscriber* sub);
int              horus_subscriber_nav_goal_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_nav_goal_has_msg(const HorusSubscriber* sub);

/* Heartbeat */
HorusPublisher*  horus_publisher_heartbeat_new(const char* topic_name);
void             horus_publisher_heartbeat_destroy(HorusPublisher* pub_);
void             horus_publisher_heartbeat_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_heartbeat_new(const char* topic_name);
void             horus_subscriber_heartbeat_destroy(HorusSubscriber* sub);
int              horus_subscriber_heartbeat_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_heartbeat_has_msg(const HorusSubscriber* sub);

/* EmergencyStop */
HorusPublisher*  horus_publisher_emergency_stop_new(const char* topic_name);
void             horus_publisher_emergency_stop_destroy(HorusPublisher* pub_);
void             horus_publisher_emergency_stop_send(const HorusPublisher* pub_, const void* msg);
HorusSubscriber* horus_subscriber_emergency_stop_new(const char* topic_name);
void             horus_subscriber_emergency_stop_destroy(HorusSubscriber* sub);
int              horus_subscriber_emergency_stop_recv(const HorusSubscriber* sub, void* out);
bool             horus_subscriber_emergency_stop_has_msg(const HorusSubscriber* sub);

/* ── Additional Pod Types (void* API — all generated by impl_pod_topic_c_api! macro) */
/* Each type has: _new, _destroy, _send, _new (sub), _destroy (sub), _recv, _has_msg */
/* Sensor */
HorusPublisher*  horus_publisher_range_sensor_new(const char* t);
void             horus_publisher_range_sensor_destroy(HorusPublisher* p);
void             horus_publisher_range_sensor_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_range_sensor_new(const char* t);
void             horus_subscriber_range_sensor_destroy(HorusSubscriber* s);
int              horus_subscriber_range_sensor_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_range_sensor_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_battery_state_new(const char* t);
void             horus_publisher_battery_state_destroy(HorusPublisher* p);
void             horus_publisher_battery_state_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_battery_state_new(const char* t);
void             horus_subscriber_battery_state_destroy(HorusSubscriber* s);
int              horus_subscriber_battery_state_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_battery_state_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_nav_sat_fix_new(const char* t);
void             horus_publisher_nav_sat_fix_destroy(HorusPublisher* p);
void             horus_publisher_nav_sat_fix_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_nav_sat_fix_new(const char* t);
void             horus_subscriber_nav_sat_fix_destroy(HorusSubscriber* s);
int              horus_subscriber_nav_sat_fix_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_nav_sat_fix_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_magnetic_field_new(const char* t);
void             horus_publisher_magnetic_field_destroy(HorusPublisher* p);
void             horus_publisher_magnetic_field_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_magnetic_field_new(const char* t);
void             horus_subscriber_magnetic_field_destroy(HorusSubscriber* s);
int              horus_subscriber_magnetic_field_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_magnetic_field_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_temperature_new(const char* t);
void             horus_publisher_temperature_destroy(HorusPublisher* p);
void             horus_publisher_temperature_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_temperature_new(const char* t);
void             horus_subscriber_temperature_destroy(HorusSubscriber* s);
int              horus_subscriber_temperature_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_temperature_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_fluid_pressure_new(const char* t);
void             horus_publisher_fluid_pressure_destroy(HorusPublisher* p);
void             horus_publisher_fluid_pressure_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_fluid_pressure_new(const char* t);
void             horus_subscriber_fluid_pressure_destroy(HorusSubscriber* s);
int              horus_subscriber_fluid_pressure_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_fluid_pressure_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_illuminance_new(const char* t);
void             horus_publisher_illuminance_destroy(HorusPublisher* p);
void             horus_publisher_illuminance_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_illuminance_new(const char* t);
void             horus_subscriber_illuminance_destroy(HorusSubscriber* s);
int              horus_subscriber_illuminance_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_illuminance_has_msg(const HorusSubscriber* s);

/* Control */
HorusPublisher*  horus_publisher_motor_command_new(const char* t);
void             horus_publisher_motor_command_destroy(HorusPublisher* p);
void             horus_publisher_motor_command_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_motor_command_new(const char* t);
void             horus_subscriber_motor_command_destroy(HorusSubscriber* s);
int              horus_subscriber_motor_command_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_motor_command_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_differential_drive_command_new(const char* t);
void             horus_publisher_differential_drive_command_destroy(HorusPublisher* p);
void             horus_publisher_differential_drive_command_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_differential_drive_command_new(const char* t);
void             horus_subscriber_differential_drive_command_destroy(HorusSubscriber* s);
int              horus_subscriber_differential_drive_command_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_differential_drive_command_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_servo_command_new(const char* t);
void             horus_publisher_servo_command_destroy(HorusPublisher* p);
void             horus_publisher_servo_command_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_servo_command_new(const char* t);
void             horus_subscriber_servo_command_destroy(HorusSubscriber* s);
int              horus_subscriber_servo_command_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_servo_command_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_pid_config_new(const char* t);
void             horus_publisher_pid_config_destroy(HorusPublisher* p);
void             horus_publisher_pid_config_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_pid_config_new(const char* t);
void             horus_subscriber_pid_config_destroy(HorusSubscriber* s);
int              horus_subscriber_pid_config_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_pid_config_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_trajectory_point_new(const char* t);
void             horus_publisher_trajectory_point_destroy(HorusPublisher* p);
void             horus_publisher_trajectory_point_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_trajectory_point_new(const char* t);
void             horus_subscriber_trajectory_point_destroy(HorusSubscriber* s);
int              horus_subscriber_trajectory_point_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_trajectory_point_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_joint_command_new(const char* t);
void             horus_publisher_joint_command_destroy(HorusPublisher* p);
void             horus_publisher_joint_command_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_joint_command_new(const char* t);
void             horus_subscriber_joint_command_destroy(HorusSubscriber* s);
int              horus_subscriber_joint_command_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_joint_command_has_msg(const HorusSubscriber* s);

/* Geometry */
HorusPublisher*  horus_publisher_point3_new(const char* t);
void             horus_publisher_point3_destroy(HorusPublisher* p);
void             horus_publisher_point3_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_point3_new(const char* t);
void             horus_subscriber_point3_destroy(HorusSubscriber* s);
int              horus_subscriber_point3_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_point3_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_vector3_new(const char* t);
void             horus_publisher_vector3_destroy(HorusPublisher* p);
void             horus_publisher_vector3_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_vector3_new(const char* t);
void             horus_subscriber_vector3_destroy(HorusSubscriber* s);
int              horus_subscriber_vector3_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_vector3_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_quaternion_new(const char* t);
void             horus_publisher_quaternion_destroy(HorusPublisher* p);
void             horus_publisher_quaternion_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_quaternion_new(const char* t);
void             horus_subscriber_quaternion_destroy(HorusSubscriber* s);
int              horus_subscriber_quaternion_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_quaternion_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_pose3d_new(const char* t);
void             horus_publisher_pose3d_destroy(HorusPublisher* p);
void             horus_publisher_pose3d_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_pose3d_new(const char* t);
void             horus_subscriber_pose3d_destroy(HorusSubscriber* s);
int              horus_subscriber_pose3d_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_pose3d_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_pose_stamped_new(const char* t);
void             horus_publisher_pose_stamped_destroy(HorusPublisher* p);
void             horus_publisher_pose_stamped_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_pose_stamped_new(const char* t);
void             horus_subscriber_pose_stamped_destroy(HorusSubscriber* s);
int              horus_subscriber_pose_stamped_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_pose_stamped_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_pose_with_covariance_new(const char* t);
void             horus_publisher_pose_with_covariance_destroy(HorusPublisher* p);
void             horus_publisher_pose_with_covariance_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_pose_with_covariance_new(const char* t);
void             horus_subscriber_pose_with_covariance_destroy(HorusSubscriber* s);
int              horus_subscriber_pose_with_covariance_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_pose_with_covariance_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_twist_with_covariance_new(const char* t);
void             horus_publisher_twist_with_covariance_destroy(HorusPublisher* p);
void             horus_publisher_twist_with_covariance_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_twist_with_covariance_new(const char* t);
void             horus_subscriber_twist_with_covariance_destroy(HorusSubscriber* s);
int              horus_subscriber_twist_with_covariance_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_twist_with_covariance_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_accel_new(const char* t);
void             horus_publisher_accel_destroy(HorusPublisher* p);
void             horus_publisher_accel_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_accel_new(const char* t);
void             horus_subscriber_accel_destroy(HorusSubscriber* s);
int              horus_subscriber_accel_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_accel_has_msg(const HorusSubscriber* s);

HorusPublisher*  horus_publisher_accel_stamped_new(const char* t);
void             horus_publisher_accel_stamped_destroy(HorusPublisher* p);
void             horus_publisher_accel_stamped_send(const HorusPublisher* p, const void* m);
HorusSubscriber* horus_subscriber_accel_stamped_new(const char* t);
void             horus_subscriber_accel_stamped_destroy(HorusSubscriber* s);
int              horus_subscriber_accel_stamped_recv(const HorusSubscriber* s, void* o);
bool             horus_subscriber_accel_stamped_has_msg(const HorusSubscriber* s);

/* Detection + Vision + Nav + Diagnostics + Force + Tracking + Landmark + Input + Audio + Clock + Perception */
/* All follow same pattern: horus_publisher_<name>_new/_destroy/_send + horus_subscriber_<name>_new/_destroy/_recv/_has_msg */
#define HORUS_DECLARE_TOPIC(name) \
    HorusPublisher*  horus_publisher_##name##_new(const char* t); \
    void             horus_publisher_##name##_destroy(HorusPublisher* p); \
    void             horus_publisher_##name##_send(const HorusPublisher* p, const void* m); \
    HorusSubscriber* horus_subscriber_##name##_new(const char* t); \
    void             horus_subscriber_##name##_destroy(HorusSubscriber* s); \
    int              horus_subscriber_##name##_recv(const HorusSubscriber* s, void* o); \
    bool             horus_subscriber_##name##_has_msg(const HorusSubscriber* s);

HORUS_DECLARE_TOPIC(bounding_box_2d)
HORUS_DECLARE_TOPIC(bounding_box_3d)
HORUS_DECLARE_TOPIC(detection)
HORUS_DECLARE_TOPIC(detection_3d)
HORUS_DECLARE_TOPIC(camera_info)
HORUS_DECLARE_TOPIC(region_of_interest)
HORUS_DECLARE_TOPIC(stereo_info)
HORUS_DECLARE_TOPIC(goal_result)
HORUS_DECLARE_TOPIC(waypoint)
HORUS_DECLARE_TOPIC(velocity_obstacle)
HORUS_DECLARE_TOPIC(path_plan)
HORUS_DECLARE_TOPIC(diagnostic_status)
HORUS_DECLARE_TOPIC(diagnostic_value)
HORUS_DECLARE_TOPIC(diagnostic_report)
HORUS_DECLARE_TOPIC(resource_usage)
HORUS_DECLARE_TOPIC(node_heartbeat)
HORUS_DECLARE_TOPIC(safety_status)
HORUS_DECLARE_TOPIC(wrench_stamped)
HORUS_DECLARE_TOPIC(force_command)
HORUS_DECLARE_TOPIC(contact_info)
HORUS_DECLARE_TOPIC(haptic_feedback)
HORUS_DECLARE_TOPIC(impedance_parameters)
HORUS_DECLARE_TOPIC(tracked_object)
HORUS_DECLARE_TOPIC(tracking_header)
HORUS_DECLARE_TOPIC(segmentation_mask)
HORUS_DECLARE_TOPIC(landmark)
HORUS_DECLARE_TOPIC(landmark_3d)
HORUS_DECLARE_TOPIC(landmark_array)
HORUS_DECLARE_TOPIC(keyboard_input)
HORUS_DECLARE_TOPIC(joystick_input)
HORUS_DECLARE_TOPIC(audio_frame)
HORUS_DECLARE_TOPIC(clock)
HORUS_DECLARE_TOPIC(time_reference)
HORUS_DECLARE_TOPIC(point_field)
HORUS_DECLARE_TOPIC(plane_detection)

#undef HORUS_DECLARE_TOPIC

/* ── JsonWireMessage Topic (for services/actions) ────────────────────────── */

typedef struct {
    uint8_t  data[3968];
    uint32_t data_len;
    uint64_t msg_id;
    uint8_t  msg_type;  /* 0=request, 1=response_ok, 2=response_err, 3=feedback, 4=goal, 5=result */
    uint8_t  _padding[11];
} HorusJsonWireMsg;

HorusPublisher*  horus_publisher_json_wire_new(const char* topic_name);
void             horus_publisher_json_wire_destroy(HorusPublisher* pub_);
void             horus_publisher_json_wire_send(const HorusPublisher* pub_, const HorusJsonWireMsg* msg);

HorusSubscriber* horus_subscriber_json_wire_new(const char* topic_name);
void             horus_subscriber_json_wire_destroy(HorusSubscriber* sub);
int              horus_subscriber_json_wire_recv(const HorusSubscriber* sub, HorusJsonWireMsg* out);

/* ── Miss Policy Constants ───────────────────────────────────────────────── */

#define HORUS_MISS_WARN      0
#define HORUS_MISS_SKIP      1
#define HORUS_MISS_SAFE_MODE 2
#define HORUS_MISS_STOP      3

/* ── TensorPool ─────────────────────────────────────────────────────────── */

typedef struct HorusTensorPool   HorusTensorPool;
typedef struct HorusTensor       HorusTensor;
typedef struct HorusImage        HorusImage;
typedef struct HorusPointCloud   HorusPointCloud;

/* dtype: 0=f32, 1=f64, 2=u8, 3=i32 */
#define HORUS_DTYPE_F32  0
#define HORUS_DTYPE_F64  1
#define HORUS_DTYPE_U8   2
#define HORUS_DTYPE_I32  3

/* encoding: 0=RGB8, 1=RGBA8, 2=GRAY8, 3=BGR8 */
#define HORUS_ENC_RGB8   0
#define HORUS_ENC_RGBA8  1
#define HORUS_ENC_GRAY8  2
#define HORUS_ENC_BGR8   3

HorusTensorPool* horus_tensor_pool_new(uint32_t pool_id, size_t pool_size_bytes, size_t max_slots);
void             horus_tensor_pool_destroy(HorusTensorPool* pool);
int              horus_tensor_pool_stats(const HorusTensorPool* pool, size_t* out_allocated, size_t* out_used, size_t* out_free);

HorusTensor*     horus_tensor_alloc(const HorusTensorPool* pool, const uint64_t* shape, size_t ndim, uint8_t dtype);
uint8_t*         horus_tensor_data_ptr(const HorusTensorPool* pool, const HorusTensor* tensor);
uint64_t         horus_tensor_nbytes(const HorusTensor* tensor);
void             horus_tensor_release(const HorusTensorPool* pool, const HorusTensor* tensor);
void             horus_tensor_destroy(HorusTensor* tensor);

HorusImage*      horus_image_new(const HorusTensorPool* pool, uint32_t width, uint32_t height, uint8_t encoding);
uint32_t         horus_image_width(const HorusImage* img);
uint32_t         horus_image_height(const HorusImage* img);
size_t           horus_image_data_size(const HorusImage* img);
void             horus_image_destroy(HorusImage* img);

HorusPointCloud* horus_pointcloud_new(const HorusTensorPool* pool, uint32_t num_points, uint32_t fields_per_point);
uint64_t         horus_pointcloud_num_points(const HorusPointCloud* pc);
uint32_t         horus_pointcloud_fields(const HorusPointCloud* pc);
void             horus_pointcloud_destroy(HorusPointCloud* pc);

/* ── RuntimeParams ──────────────────────────────────────────────────────── */

typedef struct HorusParams HorusParams;

HorusParams* horus_params_new(void);
void         horus_params_destroy(HorusParams* params);
int          horus_params_get_f64(const HorusParams* params, const char* key, double* out);
int          horus_params_set_f64(const HorusParams* params, const char* key, double value);
int          horus_params_get_i64(const HorusParams* params, const char* key, int64_t* out);
int          horus_params_set_i64(const HorusParams* params, const char* key, int64_t value);
int          horus_params_get_bool(const HorusParams* params, const char* key, bool* out);
int          horus_params_set_bool(const HorusParams* params, const char* key, bool value);
int          horus_params_get_string(const HorusParams* params, const char* key, char* buf, size_t buf_len);
int          horus_params_set_string(const HorusParams* params, const char* key, const char* value);
bool         horus_params_has(const HorusParams* params, const char* key);

/* ── Service ────────────────────────────────────────────────────────────── */

typedef struct HorusServiceClient HorusServiceClient;
typedef struct HorusServiceServer HorusServiceServer;

HorusServiceClient* horus_service_client_new(const char* name);
void                horus_service_client_destroy(HorusServiceClient* client);
int                 horus_service_client_call(const HorusServiceClient* client, const char* request_json, uint64_t timeout_us, char* out_buf, size_t out_buf_len);

HorusServiceServer* horus_service_server_new(const char* name);
void                horus_service_server_destroy(HorusServiceServer* server);
void                horus_service_server_set_handler(HorusServiceServer* server, bool (*handler)(const uint8_t*, size_t, uint8_t*, size_t*));

/* ── Action ─────────────────────────────────────────────────────────────── */

typedef struct HorusActionClient HorusActionClient;
typedef struct HorusActionServer HorusActionServer;
typedef struct HorusGoalHandle   HorusGoalHandle;

/* Goal status: 0=Pending, 1=Active, 2=Succeeded, 3=Aborted, 4=Canceled, 5=Rejected */
#define HORUS_GOAL_PENDING    0
#define HORUS_GOAL_ACTIVE     1
#define HORUS_GOAL_SUCCEEDED  2
#define HORUS_GOAL_ABORTED    3
#define HORUS_GOAL_CANCELED   4
#define HORUS_GOAL_REJECTED   5

HorusActionClient* horus_action_client_new(const char* name);
void               horus_action_client_destroy(HorusActionClient* client);
HorusGoalHandle*   horus_action_client_send_goal(const HorusActionClient* client, const char* goal_json);
void               horus_action_client_cancel(HorusGoalHandle* handle);

uint8_t            horus_goal_handle_status(const HorusGoalHandle* handle);
uint64_t           horus_goal_handle_id(const HorusGoalHandle* handle);
bool               horus_goal_handle_is_active(const HorusGoalHandle* handle);
void               horus_goal_handle_destroy(HorusGoalHandle* handle);

HorusActionServer* horus_action_server_new(const char* name);
void               horus_action_server_destroy(HorusActionServer* server);
void               horus_action_server_set_accept_handler(HorusActionServer* server, uint8_t (*handler)(const uint8_t*, size_t));
void               horus_action_server_set_execute_handler(HorusActionServer* server, void (*handler)(uint64_t, const uint8_t*, size_t));
bool               horus_action_server_is_ready(const HorusActionServer* server);

/* ── TransformFrame ─────────────────────────────────────────────────────── */

typedef struct HorusTransformFrame HorusTransformFrame;

HorusTransformFrame* horus_transform_frame_new(void);
HorusTransformFrame* horus_transform_frame_with_capacity(size_t max_frames);
void                 horus_transform_frame_destroy(HorusTransformFrame* tf);
int                  horus_transform_frame_register(const HorusTransformFrame* tf, const char* name, const char* parent);
int                  horus_transform_frame_update(const HorusTransformFrame* tf, const char* frame_name, double tx, double ty, double tz, double qx, double qy, double qz, double qw, uint64_t timestamp_ns);
int                  horus_transform_frame_lookup(const HorusTransformFrame* tf, const char* source, const char* target, double* out);
bool                 horus_transform_frame_can_transform(const HorusTransformFrame* tf, const char* source, const char* target);

/* ── BlackBox Flight Recorder ────────────────────────────────────────────── */

void horus_blackbox_record(const char* category, const char* message);

/* ── Logging ─────────────────────────────────────────────────────────────── */

#define HORUS_LOG_INFO    0
#define HORUS_LOG_WARNING 1
#define HORUS_LOG_ERROR   2

void horus_log(uint8_t level, const char* node_name, const char* message);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HORUS_C_H */
