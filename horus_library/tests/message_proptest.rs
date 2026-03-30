//! Property-based tests for message serialization roundtrips.
//!
//! Verifies that all standard HORUS message types survive serialize→deserialize
//! roundtrips via bincode and MessagePack (rmp-serde). Uses proptest to generate
//! thousands of random message instances.
//!
//! Comparison is done via POD byte equality (bytemuck::bytes_of) since most
//! message types don't derive PartialEq.

use horus_core::communication::PodMessage;
use horus_library::messages::*;
use proptest::prelude::*;

// ── Strategies ──────────────────────────────────────────────────────────────

fn finite_f64() -> impl Strategy<Value = f64> {
    -1e15f64..1e15f64
}

fn finite_f32() -> impl Strategy<Value = f32> {
    -1e7f32..1e7f32
}

fn f64_array3() -> impl Strategy<Value = [f64; 3]> {
    [finite_f64(), finite_f64(), finite_f64()]
}

fn f64_array4() -> impl Strategy<Value = [f64; 4]> {
    [finite_f64(), finite_f64(), finite_f64(), finite_f64()]
}

fn f64_array9() -> impl Strategy<Value = [f64; 9]> {
    [
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
    ]
}

fn name_bytes_32() -> impl Strategy<Value = [u8; 32]> {
    proptest::array::uniform32(0u8..128u8)
}

fn name_bytes_64() -> impl Strategy<Value = [u8; 64]> {
    proptest::collection::vec(0u8..128u8, 64).prop_map(|v| {
        let mut arr = [0u8; 64];
        arr.copy_from_slice(&v);
        arr
    })
}

// ── Roundtrip assertions ────────────────────────────────────────────────────

/// Compare via serde_json::Value (ignores padding bytes, compares field values)
fn assert_field_eq<T: serde::Serialize + std::fmt::Debug>(a: &T, b: &T, codec: &str) {
    let va = serde_json::to_value(a).expect("json serialize a");
    let vb = serde_json::to_value(b).expect("json serialize b");
    assert_eq!(va, vb, "{} roundtrip field mismatch for {:?}", codec, a);
}

fn assert_bincode_roundtrip<T>(msg: &T)
where
    T: serde::Serialize + serde::de::DeserializeOwned + std::fmt::Debug,
{
    let encoded = bincode::serialize(msg).expect("bincode serialize failed");
    let decoded: T = bincode::deserialize(&encoded).expect("bincode deserialize failed");
    assert_field_eq(msg, &decoded, "bincode");
}

fn assert_msgpack_roundtrip<T>(msg: &T)
where
    T: serde::Serialize + serde::de::DeserializeOwned + std::fmt::Debug,
{
    let encoded = rmp_serde::to_vec(msg).expect("msgpack serialize failed");
    let decoded: T = rmp_serde::from_slice(&encoded).expect("msgpack deserialize failed");
    assert_field_eq(msg, &decoded, "msgpack");
}

fn assert_pod_roundtrip<T>(msg: &T)
where
    T: PodMessage + std::fmt::Debug,
{
    let bytes = msg.as_bytes();
    let decoded = T::from_bytes(bytes).expect("POD from_bytes failed");
    // POD roundtrip preserves exact bytes (including padding)
    assert_eq!(
        msg.as_bytes(),
        decoded.as_bytes(),
        "POD roundtrip byte mismatch"
    );
}

macro_rules! roundtrip_test {
    ($name:ident, $ty:ty, $strategy:expr) => {
        proptest! {
            #![proptest_config(ProptestConfig::with_cases(1000))]

            #[test]
            fn $name(msg in $strategy) {
                assert_bincode_roundtrip(&msg);
                assert_msgpack_roundtrip(&msg);
                assert_pod_roundtrip(&msg);
            }
        }
    };
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry messages
// ═══════════════════════════════════════════════════════════════════════════

fn arb_twist() -> impl Strategy<Value = Twist> {
    (f64_array3(), f64_array3(), any::<u64>()).prop_map(|(linear, angular, ts)| Twist {
        linear,
        angular,
        timestamp_ns: ts,
    })
}

fn arb_pose2d() -> impl Strategy<Value = Pose2D> {
    (finite_f64(), finite_f64(), finite_f64(), any::<u64>()).prop_map(|(x, y, theta, ts)| Pose2D {
        x,
        y,
        theta,
        timestamp_ns: ts,
    })
}

fn arb_point3() -> impl Strategy<Value = Point3> {
    (finite_f64(), finite_f64(), finite_f64()).prop_map(|(x, y, z)| Point3 { x, y, z })
}

fn arb_vector3() -> impl Strategy<Value = Vector3> {
    (finite_f64(), finite_f64(), finite_f64()).prop_map(|(x, y, z)| Vector3 { x, y, z })
}

fn arb_quaternion() -> impl Strategy<Value = Quaternion> {
    (finite_f64(), finite_f64(), finite_f64(), finite_f64()).prop_map(|(x, y, z, w)| Quaternion {
        x,
        y,
        z,
        w,
    })
}

fn arb_transform_stamped() -> impl Strategy<Value = TransformStamped> {
    (f64_array3(), f64_array4(), any::<u64>()).prop_map(|(translation, rotation, ts)| {
        TransformStamped {
            translation,
            rotation,
            timestamp_ns: ts,
        }
    })
}

roundtrip_test!(twist_roundtrip, Twist, arb_twist());
roundtrip_test!(pose2d_roundtrip, Pose2D, arb_pose2d());
roundtrip_test!(point3_roundtrip, Point3, arb_point3());
roundtrip_test!(vector3_roundtrip, Vector3, arb_vector3());
roundtrip_test!(quaternion_roundtrip, Quaternion, arb_quaternion());
roundtrip_test!(
    transform_stamped_roundtrip,
    TransformStamped,
    arb_transform_stamped()
);

// ═══════════════════════════════════════════════════════════════════════════
// Control messages
// ═══════════════════════════════════════════════════════════════════════════

fn arb_cmd_vel() -> impl Strategy<Value = CmdVel> {
    (any::<u64>(), finite_f32(), finite_f32()).prop_map(|(ts, lin, ang)| CmdVel {
        timestamp_ns: ts,
        linear: lin,
        angular: ang,
    })
}

fn arb_motor_command() -> impl Strategy<Value = MotorCommand> {
    (
        any::<u8>(),
        0u8..4u8,
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(
            |(id, mode, target, max_vel, max_acc, ff, enable, ts)| MotorCommand {
                motor_id: id,
                mode,
                target,
                max_velocity: max_vel,
                max_acceleration: max_acc,
                feed_forward: ff,
                enable,
                timestamp_ns: ts,
            },
        )
}

fn arb_servo_command() -> impl Strategy<Value = ServoCommand> {
    (
        any::<u8>(),
        finite_f32(),
        finite_f32(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(|(id, position, speed, enable, ts)| ServoCommand {
            servo_id: id,
            position,
            speed,
            enable,
            timestamp_ns: ts,
        })
}

fn arb_pid_config() -> impl Strategy<Value = PidConfig> {
    (
        any::<u8>(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(
            |(id, kp, ki, kd, int_limit, out_limit, anti_windup, ts)| PidConfig {
                controller_id: id,
                kp,
                ki,
                kd,
                integral_limit: int_limit,
                output_limit: out_limit,
                anti_windup,
                timestamp_ns: ts,
            },
        )
}

roundtrip_test!(cmd_vel_roundtrip, CmdVel, arb_cmd_vel());
roundtrip_test!(motor_command_roundtrip, MotorCommand, arb_motor_command());
roundtrip_test!(servo_command_roundtrip, ServoCommand, arb_servo_command());
roundtrip_test!(pid_config_roundtrip, PidConfig, arb_pid_config());

// ═══════════════════════════════════════════════════════════════════════════
// Sensor messages
// ═══════════════════════════════════════════════════════════════════════════

fn arb_imu() -> impl Strategy<Value = Imu> {
    (
        f64_array4(),
        f64_array9(),
        f64_array3(),
        f64_array9(),
        f64_array3(),
        f64_array9(),
        any::<u64>(),
    )
        .prop_map(
            |(orientation, orient_cov, angular_vel, ang_cov, linear_acc, lin_cov, ts)| Imu {
                orientation,
                orientation_covariance: orient_cov,
                angular_velocity: angular_vel,
                angular_velocity_covariance: ang_cov,
                linear_acceleration: linear_acc,
                linear_acceleration_covariance: lin_cov,
                timestamp_ns: ts,
            },
        )
}

fn arb_range_sensor() -> impl Strategy<Value = RangeSensor> {
    (
        any::<u8>(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        any::<u64>(),
    )
        .prop_map(|(sensor_type, fov, min, max, range, ts)| RangeSensor {
            sensor_type,
            field_of_view: fov,
            min_range: min,
            max_range: max,
            range,
            timestamp_ns: ts,
        })
}

fn arb_temperature() -> impl Strategy<Value = Temperature> {
    (finite_f64(), finite_f64(), name_bytes_32(), any::<u64>()).prop_map(
        |(temp, var, frame, ts)| Temperature {
            temperature: temp,
            variance: var,
            frame_id: frame,
            timestamp_ns: ts,
        },
    )
}

roundtrip_test!(imu_roundtrip, Imu, arb_imu());
roundtrip_test!(range_sensor_roundtrip, RangeSensor, arb_range_sensor());
roundtrip_test!(temperature_roundtrip, Temperature, arb_temperature());

// ═══════════════════════════════════════════════════════════════════════════
// Diagnostics messages
// ═══════════════════════════════════════════════════════════════════════════

fn arb_heartbeat() -> impl Strategy<Value = Heartbeat> {
    (
        name_bytes_32(),
        any::<u32>(),
        any::<u64>(),
        any::<u8>(),
        finite_f64(),
        any::<u64>(),
    )
        .prop_map(|(name, node_id, seq, alive, uptime, ts)| Heartbeat {
            node_name: name,
            node_id,
            sequence: seq,
            alive,
            uptime,
            timestamp_ns: ts,
        })
}

fn arb_emergency_stop() -> impl Strategy<Value = EmergencyStop> {
    (
        any::<u8>(),
        name_bytes_64(),
        name_bytes_32(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(|(engaged, reason, source, auto_reset, ts)| EmergencyStop {
            engaged,
            reason,
            source,
            auto_reset,
            timestamp_ns: ts,
        })
}

roundtrip_test!(heartbeat_roundtrip, Heartbeat, arb_heartbeat());
roundtrip_test!(
    emergency_stop_roundtrip,
    EmergencyStop,
    arb_emergency_stop()
);

// ═══════════════════════════════════════════════════════════════════════════
// SLAM Cycle 1 — High-priority missing proptest strategies
// ═══════════════════════════════════════════════════════════════════════════

// ── Geometry (missing) ───────────────────────────────────────────────────

fn arb_accel() -> impl Strategy<Value = Accel> {
    (f64_array3(), f64_array3(), any::<u64>()).prop_map(|(linear, angular, ts)| Accel {
        linear,
        angular,
        timestamp_ns: ts,
    })
}

fn arb_accel_stamped() -> impl Strategy<Value = AccelStamped> {
    (arb_accel(), name_bytes_32(), any::<u64>()).prop_map(|(accel, frame, ts)| AccelStamped {
        accel,
        frame_id: frame,
        timestamp_ns: ts,
    })
}

fn arb_pose3d() -> impl Strategy<Value = Pose3D> {
    (arb_point3(), arb_quaternion(), any::<u64>()).prop_map(|(position, orientation, ts)| Pose3D {
        position,
        orientation,
        timestamp_ns: ts,
    })
}

fn arb_pose_stamped() -> impl Strategy<Value = PoseStamped> {
    (arb_pose3d(), name_bytes_32(), any::<u64>()).prop_map(|(pose, frame, ts)| PoseStamped {
        pose,
        frame_id: frame,
        timestamp_ns: ts,
    })
}

roundtrip_test!(accel_roundtrip, Accel, arb_accel());
roundtrip_test!(accel_stamped_roundtrip, AccelStamped, arb_accel_stamped());
roundtrip_test!(pose3d_roundtrip, Pose3D, arb_pose3d());
roundtrip_test!(pose_stamped_roundtrip, PoseStamped, arb_pose_stamped());

// ── Sensor (missing) ─────────────────────────────────────────────────────

fn arb_odometry() -> impl Strategy<Value = Odometry> {
    (
        arb_pose2d(),
        arb_twist(),
        name_bytes_32(),
        name_bytes_32(),
        any::<u64>(),
    )
        .prop_map(|(pose, twist, frame, child_frame, ts)| Odometry {
            pose,
            twist,
            pose_covariance: [0.0; 36],
            twist_covariance: [0.0; 36],
            frame_id: frame,
            child_frame_id: child_frame,
            timestamp_ns: ts,
        })
}

fn arb_laser_scan() -> impl Strategy<Value = LaserScan> {
    (
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        any::<u64>(),
    )
        .prop_map(
            |(amin, amax, rmin, rmax, ainc, tinc, stime, ts)| LaserScan {
                ranges: [0.0f32; 360],
                angle_min: amin,
                angle_max: amax,
                range_min: rmin,
                range_max: rmax,
                angle_increment: ainc,
                time_increment: tinc,
                scan_time: stime,
                timestamp_ns: ts,
            },
        )
}

roundtrip_test!(odometry_roundtrip, Odometry, arb_odometry());
roundtrip_test!(laser_scan_roundtrip, LaserScan, arb_laser_scan());

// ── Control (missing) ────────────────────────────────────────────────────

fn arb_differential_drive() -> impl Strategy<Value = DifferentialDriveCommand> {
    (
        finite_f64(),
        finite_f64(),
        finite_f64(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(
            |(left, right, max_acc, enable, ts)| DifferentialDriveCommand {
                left_velocity: left,
                right_velocity: right,
                max_acceleration: max_acc,
                enable,
                timestamp_ns: ts,
            },
        )
}

roundtrip_test!(
    differential_drive_roundtrip,
    DifferentialDriveCommand,
    arb_differential_drive()
);

// ── Navigation (missing) ─────────────────────────────────────────────────

fn arb_nav_goal() -> impl Strategy<Value = NavGoal> {
    (
        arb_pose2d(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        any::<u8>(),
        any::<u32>(),
        any::<u64>(),
    )
        .prop_map(|(pose, tol_pos, tol_ang, timeout, prio, gid, ts)| NavGoal {
            target_pose: pose,
            tolerance_position: tol_pos,
            tolerance_angle: tol_ang,
            timeout_seconds: timeout,
            priority: prio,
            goal_id: gid,
            timestamp_ns: ts,
        })
}

roundtrip_test!(nav_goal_roundtrip, NavGoal, arb_nav_goal());

// ── Detection / Perception (missing) ─────────────────────────────────────

fn arb_bounding_box_2d() -> impl Strategy<Value = BoundingBox2D> {
    (finite_f32(), finite_f32(), finite_f32(), finite_f32()).prop_map(|(x, y, w, h)| {
        BoundingBox2D {
            x,
            y,
            width: w,
            height: h,
        }
    })
}

fn arb_detection() -> impl Strategy<Value = Detection> {
    (
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
    )
        .prop_map(|(conf, x, y, w, h)| Detection::new("object", conf, x, y, w, h))
}

// BoundingBox2D and Detection are serde-only (not PodMessage) — use serde-only macro
macro_rules! serde_roundtrip_test {
    ($name:ident, $ty:ty, $strategy:expr) => {
        proptest! {
            #![proptest_config(ProptestConfig::with_cases(1000))]

            #[test]
            fn $name(msg in $strategy) {
                assert_bincode_roundtrip(&msg);
                assert_msgpack_roundtrip(&msg);
            }
        }
    };
}

serde_roundtrip_test!(
    bounding_box_2d_roundtrip,
    BoundingBox2D,
    arb_bounding_box_2d()
);
serde_roundtrip_test!(detection_roundtrip, Detection, arb_detection());

// ═══════════════════════════════════════════════════════════════════════════
// Force / Haptics messages
// ═══════════════════════════════════════════════════════════════════════════

fn arb_wrench_stamped() -> impl Strategy<Value = WrenchStamped> {
    (
        arb_vector3(),
        arb_vector3(),
        arb_point3(),
        name_bytes_32(),
        any::<u64>(),
    )
        .prop_map(|(force, torque, point, frame, ts)| WrenchStamped {
            force,
            torque,
            point_of_application: point,
            frame_id: frame,
            timestamp_ns: ts,
        })
}

fn arb_force_command() -> impl Strategy<Value = ForceCommand> {
    (
        arb_vector3(),
        arb_vector3(),
        proptest::array::uniform6(0u8..2u8),
        arb_vector3(),
        arb_vector3(),
        arb_vector3(),
        [
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
        ],
        finite_f64(),
        any::<u64>(),
    )
        .prop_map(
            |(tf, tt, fm, ps, os, md, gains, timeout, ts)| ForceCommand {
                target_force: tf,
                target_torque: tt,
                force_mode: fm,
                position_setpoint: ps,
                orientation_setpoint: os,
                max_deviation: md,
                gains,
                timeout_seconds: timeout,
                frame_id: [0; 32],
                timestamp_ns: ts,
            },
        )
}

fn arb_impedance_parameters() -> impl Strategy<Value = ImpedanceParameters> {
    (
        [
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
        ],
        [
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
        ],
        [
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
        ],
        [
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
            finite_f64(),
        ],
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(|(stiff, damp, inert, flim, en, ts)| ImpedanceParameters {
            stiffness: stiff,
            damping: damp,
            inertia: inert,
            force_limits: flim,
            enabled: en,
            timestamp_ns: ts,
        })
}

fn arb_haptic_feedback() -> impl Strategy<Value = HapticFeedback> {
    (
        finite_f32(),
        finite_f32(),
        finite_f32(),
        arb_vector3(),
        any::<u8>(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(
            |(intensity, freq, dur, force_fb, pattern, en, ts)| HapticFeedback {
                vibration_intensity: intensity,
                vibration_frequency: freq,
                duration_seconds: dur,
                force_feedback: force_fb,
                pattern_type: pattern,
                enabled: en,
                timestamp_ns: ts,
            },
        )
}

roundtrip_test!(
    wrench_stamped_roundtrip,
    WrenchStamped,
    arb_wrench_stamped()
);
roundtrip_test!(force_command_roundtrip, ForceCommand, arb_force_command());
roundtrip_test!(
    impedance_parameters_roundtrip,
    ImpedanceParameters,
    arb_impedance_parameters()
);
roundtrip_test!(
    haptic_feedback_roundtrip,
    HapticFeedback,
    arb_haptic_feedback()
);

// ═══════════════════════════════════════════════════════════════════════════
// Sensor messages (remaining)
// ═══════════════════════════════════════════════════════════════════════════

fn arb_magnetic_field() -> impl Strategy<Value = MagneticField> {
    (f64_array3(), f64_array9(), name_bytes_32(), any::<u64>()).prop_map(
        |(field, cov, frame, ts)| MagneticField {
            magnetic_field: field,
            magnetic_field_covariance: cov,
            frame_id: frame,
            timestamp_ns: ts,
        },
    )
}

fn arb_fluid_pressure() -> impl Strategy<Value = FluidPressure> {
    (finite_f64(), finite_f64(), name_bytes_32(), any::<u64>()).prop_map(
        |(pressure, var, frame, ts)| FluidPressure {
            fluid_pressure: pressure,
            variance: var,
            frame_id: frame,
            timestamp_ns: ts,
        },
    )
}

fn arb_illuminance() -> impl Strategy<Value = Illuminance> {
    (finite_f64(), finite_f64(), name_bytes_32(), any::<u64>()).prop_map(|(lux, var, frame, ts)| {
        Illuminance {
            illuminance: lux,
            variance: var,
            frame_id: frame,
            timestamp_ns: ts,
        }
    })
}

fn arb_nav_sat_fix() -> impl Strategy<Value = NavSatFix> {
    (
        finite_f64(),
        finite_f64(),
        finite_f64(),
        f64_array9(),
        any::<u8>(),
        any::<u8>(),
        any::<u16>(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
    )
        .prop_map(
            |(lat, lon, alt, cov, cov_type, status, sats, hdop, vdop, speed)| {
                let mut fix = NavSatFix::default();
                fix.latitude = lat;
                fix.longitude = lon;
                fix.altitude = alt;
                fix.position_covariance = cov;
                fix.position_covariance_type = cov_type;
                fix.status = status;
                fix.satellites_visible = sats;
                fix.hdop = hdop;
                fix.vdop = vdop;
                fix.speed = speed;
                fix
            },
        )
}

roundtrip_test!(
    magnetic_field_roundtrip,
    MagneticField,
    arb_magnetic_field()
);
roundtrip_test!(
    fluid_pressure_roundtrip,
    FluidPressure,
    arb_fluid_pressure()
);
roundtrip_test!(illuminance_roundtrip, Illuminance, arb_illuminance());
roundtrip_test!(nav_sat_fix_roundtrip, NavSatFix, arb_nav_sat_fix());

// ═══════════════════════════════════════════════════════════════════════════
// Navigation messages (remaining)
// ═══════════════════════════════════════════════════════════════════════════

fn arb_waypoint() -> impl Strategy<Value = Waypoint> {
    (
        arb_pose2d(),
        arb_twist(),
        finite_f64(),
        finite_f32(),
        any::<u8>(),
    )
        .prop_map(|(pose, velocity, time, curvature, stop)| Waypoint {
            pose,
            velocity,
            time_from_start: time,
            curvature,
            stop_required: stop,
        })
}

fn arb_goal_result() -> impl Strategy<Value = GoalResult> {
    (
        any::<u32>(),
        any::<u8>(),
        finite_f64(),
        finite_f64(),
        finite_f32(),
        any::<u64>(),
    )
        .prop_map(|(goal_id, status, dist, eta, progress, ts)| {
            let mut gr = GoalResult::default();
            gr.goal_id = goal_id;
            gr.status = status;
            gr.distance_to_goal = dist;
            gr.eta_seconds = eta;
            gr.progress = progress;
            gr.timestamp_ns = ts;
            gr
        })
}

roundtrip_test!(waypoint_roundtrip, Waypoint, arb_waypoint());
roundtrip_test!(goal_result_roundtrip, GoalResult, arb_goal_result());

// ═══════════════════════════════════════════════════════════════════════════
// Diagnostics messages (extended)
// ═══════════════════════════════════════════════════════════════════════════

fn name_bytes_128() -> impl Strategy<Value = [u8; 128]> {
    proptest::collection::vec(0u8..128u8, 128).prop_map(|v| {
        let mut arr = [0u8; 128];
        arr.copy_from_slice(&v);
        arr
    })
}

fn arb_diagnostic_status() -> impl Strategy<Value = DiagnosticStatus> {
    (
        any::<u8>(),
        any::<u32>(),
        name_bytes_128(),
        name_bytes_32(),
        any::<u64>(),
    )
        .prop_map(|(level, code, message, component, ts)| DiagnosticStatus {
            level,
            code,
            message,
            component,
            timestamp_ns: ts,
        })
}

fn arb_resource_usage() -> impl Strategy<Value = ResourceUsage> {
    (
        finite_f32(),
        any::<u64>(),
        finite_f32(),
        any::<u64>(),
        finite_f32(),
        any::<u64>(),
        any::<u64>(),
        finite_f32(),
        any::<u32>(),
        any::<u64>(),
    )
        .prop_map(
            |(cpu, mem_bytes, mem_pct, disk_bytes, disk_pct, net_tx, net_rx, temp, threads, ts)| {
                ResourceUsage {
                    cpu_percent: cpu,
                    memory_bytes: mem_bytes,
                    memory_percent: mem_pct,
                    disk_bytes,
                    disk_percent: disk_pct,
                    network_tx_bytes: net_tx,
                    network_rx_bytes: net_rx,
                    temperature: temp,
                    thread_count: threads,
                    timestamp_ns: ts,
                }
            },
        )
}

fn arb_safety_status() -> impl Strategy<Value = SafetyStatus> {
    (
        any::<u8>(),
        any::<u8>(),
        any::<u8>(),
        any::<u8>(),
        any::<u8>(),
        any::<u8>(),
        any::<u32>(),
        any::<u64>(),
    )
        .prop_map(
            |(enabled, estop, watchdog, limits, comms, mode, fault, ts)| SafetyStatus {
                enabled,
                estop_engaged: estop,
                watchdog_ok: watchdog,
                limits_ok: limits,
                comms_ok: comms,
                mode,
                fault_code: fault,
                timestamp_ns: ts,
            },
        )
}

fn arb_node_heartbeat() -> impl Strategy<Value = NodeHeartbeat> {
    (
        any::<u8>(),
        any::<u8>(),
        any::<u64>(),
        any::<u32>(),
        any::<u32>(),
        any::<u32>(),
        any::<u64>(),
        any::<u64>(),
    )
        .prop_map(
            |(state, health, tick_count, target_rate, actual_rate, err_count, last_tick, hb_ts)| {
                NodeHeartbeat {
                    state,
                    health,
                    tick_count,
                    target_rate_hz: target_rate,
                    actual_rate_hz: actual_rate,
                    error_count: err_count,
                    last_tick_timestamp: last_tick,
                    heartbeat_timestamp: hb_ts,
                }
            },
        )
}

roundtrip_test!(
    diagnostic_status_roundtrip,
    DiagnosticStatus,
    arb_diagnostic_status()
);
roundtrip_test!(
    resource_usage_roundtrip,
    ResourceUsage,
    arb_resource_usage()
);
roundtrip_test!(safety_status_roundtrip, SafetyStatus, arb_safety_status());
roundtrip_test!(
    node_heartbeat_roundtrip,
    NodeHeartbeat,
    arb_node_heartbeat()
);

// ═══════════════════════════════════════════════════════════════════════════
// Clock messages
// ═══════════════════════════════════════════════════════════════════════════

fn arb_clock() -> impl Strategy<Value = Clock> {
    (
        any::<u64>(),
        any::<u64>(),
        finite_f64(),
        any::<u8>(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(|(clock_ns, realtime_ns, sim_speed, paused, source, ts)| {
            let mut c = Clock::default();
            c.clock_ns = clock_ns;
            c.realtime_ns = realtime_ns;
            c.sim_speed = sim_speed;
            c.paused = paused;
            c.source = source;
            c.timestamp_ns = ts;
            c
        })
}

fn arb_sim_sync() -> impl Strategy<Value = SimSync> {
    (any::<u64>(), any::<u64>(), any::<u64>(), any::<u8>()).prop_map(
        |(step, sim_time_ns, dt_ns, state)| SimSync {
            step,
            sim_time_ns,
            dt_ns,
            state,
            _pad: [0; 7],
        },
    )
}

roundtrip_test!(clock_roundtrip, Clock, arb_clock());
roundtrip_test!(sim_sync_roundtrip, SimSync, arb_sim_sync());

// ═══════════════════════════════════════════════════════════════════════════
// Remaining message types — full coverage
// ═══════════════════════════════════════════════════════════════════════════

// ── Sensor: BatteryState (Pod) ──────────────────────────────────────────

fn arb_battery_state() -> impl Strategy<Value = BatteryState> {
    (
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        any::<u8>(),
        finite_f32(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(
            |(voltage, current, charge, capacity, pct, status, temp, cell_count, ts)| {
                let mut bs = BatteryState::default();
                bs.voltage = voltage;
                bs.current = current;
                bs.charge = charge;
                bs.capacity = capacity;
                bs.percentage = pct;
                bs.power_supply_status = status;
                bs.temperature = temp;
                bs.cell_voltages = [0.0; 16];
                bs.cell_count = cell_count;
                bs.timestamp_ns = ts;
                bs
            },
        )
}

roundtrip_test!(battery_state_roundtrip, BatteryState, arb_battery_state());

// ── Sensor: JointState (Pod) ────────────────────────────────────────────

fn arb_joint_state() -> impl Strategy<Value = JointState> {
    (any::<u8>(), any::<u64>()).prop_map(|(count, ts)| {
        let mut js = JointState::default();
        js.joint_count = count.min(16);
        js.timestamp_ns = ts;
        js
    })
}

roundtrip_test!(joint_state_roundtrip, JointState, arb_joint_state());

// ── Geometry: PoseWithCovariance (Pod) ──────────────────────────────────

fn arb_pose_with_covariance() -> impl Strategy<Value = PoseWithCovariance> {
    (arb_pose3d(), name_bytes_32(), any::<u64>()).prop_map(|(pose, frame, ts)| PoseWithCovariance {
        pose,
        covariance: [0.0; 36],
        frame_id: frame,
        timestamp_ns: ts,
    })
}

roundtrip_test!(
    pose_with_covariance_roundtrip,
    PoseWithCovariance,
    arb_pose_with_covariance()
);

// ── Geometry: TwistWithCovariance (Pod) ─────────────────────────────────

fn arb_twist_with_covariance() -> impl Strategy<Value = TwistWithCovariance> {
    (arb_twist(), name_bytes_32(), any::<u64>()).prop_map(|(twist, frame, ts)| {
        TwistWithCovariance {
            twist,
            covariance: [0.0; 36],
            frame_id: frame,
            timestamp_ns: ts,
        }
    })
}

roundtrip_test!(
    twist_with_covariance_roundtrip,
    TwistWithCovariance,
    arb_twist_with_covariance()
);

// ── Navigation: NavPath (Pod — large fixed-size, reduced cases to avoid stack overflow) ──

fn arb_nav_path() -> impl Strategy<Value = Box<NavPath>> {
    (
        0u16..4u16,
        finite_f64(),
        finite_f64(),
        name_bytes_32(),
        name_bytes_32(),
        any::<u64>(),
    )
        .prop_map(|(wpc, total_len, dur, frame, algo, ts)| {
            let mut np = Box::new(NavPath::default());
            np.waypoint_count = wpc;
            np.total_length = total_len;
            np.duration_seconds = dur;
            np.frame_id = frame;
            np.algorithm = algo;
            np.timestamp_ns = ts;
            np
        })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(50))]

    #[test]
    fn nav_path_roundtrip(msg in arb_nav_path()) {
        assert_bincode_roundtrip(&*msg);
        assert_msgpack_roundtrip(&*msg);
        assert_pod_roundtrip(&*msg);
    }
}

// ── Navigation: PathPlan (Pod — large fixed-size) ───────────────────────

fn arb_path_plan() -> impl Strategy<Value = Box<PathPlan>> {
    (
        [finite_f32(), finite_f32(), finite_f32()],
        0u16..4u16,
        any::<u64>(),
    )
        .prop_map(|(goal, wpc, ts)| {
            let mut pp = Box::new(PathPlan::default());
            pp.goal_pose = goal;
            pp.waypoint_count = wpc;
            pp.timestamp_ns = ts;
            pp
        })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(100))]

    #[test]
    fn path_plan_roundtrip(msg in arb_path_plan()) {
        assert_bincode_roundtrip(&*msg);
        assert_msgpack_roundtrip(&*msg);
        assert_pod_roundtrip(&*msg);
    }
}

// ── Navigation: VelocityObstacle (Pod) ──────────────────────────────────

fn arb_velocity_obstacle() -> impl Strategy<Value = VelocityObstacle> {
    (
        [finite_f64(), finite_f64()],
        [finite_f64(), finite_f64()],
        finite_f32(),
        finite_f32(),
        any::<u32>(),
    )
        .prop_map(|(pos, vel, radius, th, id)| VelocityObstacle {
            position: pos,
            velocity: vel,
            radius,
            time_horizon: th,
            obstacle_id: id,
        })
}

roundtrip_test!(
    velocity_obstacle_roundtrip,
    VelocityObstacle,
    arb_velocity_obstacle()
);

// ── Navigation: VelocityObstacles (Pod) ─────────────────────────────────

fn arb_velocity_obstacles() -> impl Strategy<Value = VelocityObstacles> {
    (any::<u8>(), any::<u64>()).prop_map(|(count, ts)| {
        let mut vo = VelocityObstacles::default();
        vo.count = count.min(32);
        vo.timestamp_ns = ts;
        vo
    })
}

roundtrip_test!(
    velocity_obstacles_roundtrip,
    VelocityObstacles,
    arb_velocity_obstacles()
);

// ── Navigation: OccupancyGrid (serde-only — Vec field) ──────────────────

fn arb_occupancy_grid() -> impl Strategy<Value = OccupancyGrid> {
    (
        finite_f32(),
        0u32..4u32,
        0u32..4u32,
        arb_pose2d(),
        name_bytes_32(),
        any::<u64>(),
    )
        .prop_map(|(res, w, h, origin, frame, ts)| {
            let data_size = (w as usize) * (h as usize);
            OccupancyGrid {
                resolution: res,
                width: w,
                height: h,
                origin,
                data: vec![0i8; data_size],
                frame_id: frame,
                metadata: [0; 64],
                timestamp_ns: ts,
            }
        })
}

serde_roundtrip_test!(
    occupancy_grid_roundtrip,
    OccupancyGrid,
    arb_occupancy_grid()
);

// ── Navigation: CostMap (serde-only — Vec fields) ───────────────────────

fn arb_cost_map() -> impl Strategy<Value = CostMap> {
    (
        arb_occupancy_grid(),
        finite_f32(),
        finite_f32(),
        any::<u8>(),
    )
        .prop_map(|(grid, inflation, scaling, lethal)| {
            let data_size = (grid.width * grid.height) as usize;
            CostMap {
                occupancy_grid: grid,
                costs: vec![0u8; data_size],
                inflation_radius: inflation,
                cost_scaling_factor: scaling,
                lethal_cost: lethal,
            }
        })
}

serde_roundtrip_test!(cost_map_roundtrip, CostMap, arb_cost_map());

// ── Vision: CameraInfo (serde-only — no Pod) ───────────────────────────

fn arb_camera_info() -> impl Strategy<Value = CameraInfo> {
    (
        any::<u32>(),
        any::<u32>(),
        f64_array9(),
        f64_array9(),
        name_bytes_32(),
        any::<u64>(),
    )
        .prop_map(|(w, h, camera_matrix, rect_matrix, frame, ts)| {
            let mut ci = CameraInfo::default();
            ci.width = w;
            ci.height = h;
            ci.camera_matrix = camera_matrix;
            ci.rectification_matrix = rect_matrix;
            ci.frame_id = frame;
            ci.timestamp_ns = ts;
            ci
        })
}

serde_roundtrip_test!(camera_info_roundtrip, CameraInfo, arb_camera_info());

// ── Vision: CompressedImage (serde-only — Vec field) ────────────────────

fn arb_compressed_image() -> impl Strategy<Value = CompressedImage> {
    (any::<u32>(), any::<u32>(), name_bytes_32(), any::<u64>()).prop_map(|(w, h, frame, ts)| {
        CompressedImage {
            format: [0; 8],
            data: vec![],
            width: w,
            height: h,
            frame_id: frame,
            timestamp_ns: ts,
        }
    })
}

serde_roundtrip_test!(
    compressed_image_roundtrip,
    CompressedImage,
    arb_compressed_image()
);

// ── Vision: RegionOfInterest (serde-only — has bool, no Pod) ────────────

fn arb_region_of_interest() -> impl Strategy<Value = RegionOfInterest> {
    (
        any::<u32>(),
        any::<u32>(),
        any::<u32>(),
        any::<u32>(),
        any::<bool>(),
    )
        .prop_map(|(x, y, w, h, rectify)| RegionOfInterest {
            x_offset: x,
            y_offset: y,
            width: w,
            height: h,
            do_rectify: rectify,
        })
}

serde_roundtrip_test!(
    region_of_interest_roundtrip,
    RegionOfInterest,
    arb_region_of_interest()
);

// ── Vision: StereoInfo (serde-only) ─────────────────────────────────────

fn arb_stereo_info() -> impl Strategy<Value = StereoInfo> {
    (
        arb_camera_info(),
        arb_camera_info(),
        finite_f64(),
        finite_f64(),
    )
        .prop_map(|(left, right, baseline, depth_scale)| StereoInfo {
            left_camera: left,
            right_camera: right,
            baseline,
            depth_scale,
        })
}

serde_roundtrip_test!(stereo_info_roundtrip, StereoInfo, arb_stereo_info());

// ── Detection: BoundingBox3D (Pod) ──────────────────────────────────────

fn arb_bounding_box_3d() -> impl Strategy<Value = BoundingBox3D> {
    (
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
    )
        .prop_map(|(cx, cy, cz, l, w, h, yaw)| BoundingBox3D::new(cx, cy, cz, l, w, h, yaw))
}

serde_roundtrip_test!(
    bounding_box_3d_roundtrip,
    BoundingBox3D,
    arb_bounding_box_3d()
);

// ── Detection: Detection3D (Pod) ────────────────────────────────────────

fn arb_detection3d() -> impl Strategy<Value = Detection3D> {
    (
        arb_bounding_box_3d(),
        finite_f32(),
        any::<u32>(),
        name_bytes_32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        any::<u32>(),
    )
        .prop_map(
            |(bbox, conf, cls_id, cls_name, vx, vy, vz, inst_id)| Detection3D {
                bbox,
                confidence: conf,
                class_id: cls_id,
                class_name: cls_name,
                velocity_x: vx,
                velocity_y: vy,
                velocity_z: vz,
                instance_id: inst_id,
            },
        )
}

serde_roundtrip_test!(detection3d_roundtrip, Detection3D, arb_detection3d());

// ── Landmark: Landmark (Pod) ────────────────────────────────────────────

fn arb_landmark() -> impl Strategy<Value = Landmark> {
    (finite_f32(), finite_f32(), finite_f32(), any::<u32>()).prop_map(|(x, y, vis, idx)| Landmark {
        x,
        y,
        visibility: vis,
        index: idx,
    })
}

serde_roundtrip_test!(landmark_roundtrip, Landmark, arb_landmark());

// ── Landmark: Landmark3D (Pod, packed) ──────────────────────────────────

fn arb_landmark3d() -> impl Strategy<Value = Landmark3D> {
    (
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        any::<u32>(),
    )
        .prop_map(|(x, y, z, vis, idx)| Landmark3D {
            x,
            y,
            z,
            visibility: vis,
            index: idx,
        })
}

serde_roundtrip_test!(landmark3d_roundtrip, Landmark3D, arb_landmark3d());

// ── Landmark: LandmarkArray (Pod) ───────────────────────────────────────

fn arb_landmark_array() -> impl Strategy<Value = LandmarkArray> {
    (
        any::<u32>(),
        any::<u32>(),
        any::<u32>(),
        finite_f32(),
        any::<u64>(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
    )
        .prop_map(
            |(num, dim, inst_id, conf, ts, bx, by, bw, bh)| LandmarkArray {
                num_landmarks: num,
                dimension: dim,
                instance_id: inst_id,
                confidence: conf,
                timestamp_ns: ts,
                bbox_x: bx,
                bbox_y: by,
                bbox_width: bw,
                bbox_height: bh,
            },
        )
}

serde_roundtrip_test!(
    landmark_array_roundtrip,
    LandmarkArray,
    arb_landmark_array()
);

// ── Tracking: TrackedObject (Pod) ───────────────────────────────────────

fn arb_tracked_object() -> impl Strategy<Value = TrackedObject> {
    (
        arb_bounding_box_2d(),
        arb_bounding_box_2d(),
        any::<u64>(),
        finite_f32(),
        any::<u32>(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        finite_f32(),
        any::<u32>(),
    )
        .prop_map(
            |(bbox, pred, track_id, conf, cls_id, vx, vy, ax, ay, age)| {
                let mut to = TrackedObject::default();
                to.bbox = bbox;
                to.predicted_bbox = pred;
                to.track_id = track_id;
                to.confidence = conf;
                to.class_id = cls_id;
                to.velocity_x = vx;
                to.velocity_y = vy;
                to.accel_x = ax;
                to.accel_y = ay;
                to.age = age;
                to
            },
        )
}

serde_roundtrip_test!(
    tracked_object_roundtrip,
    TrackedObject,
    arb_tracked_object()
);

// ── Tracking: TrackingHeader (Pod) ──────────────────────────────────────

fn arb_tracking_header() -> impl Strategy<Value = TrackingHeader> {
    (
        any::<u32>(),
        any::<u32>(),
        any::<u64>(),
        any::<u64>(),
        any::<u32>(),
    )
        .prop_map(|(num_tracks, frame_id, ts, total, active)| {
            let mut th = TrackingHeader::default();
            th.num_tracks = num_tracks;
            th.frame_id = frame_id;
            th.timestamp_ns = ts;
            th.total_tracks = total;
            th.active_tracks = active;
            th
        })
}

serde_roundtrip_test!(
    tracking_header_roundtrip,
    TrackingHeader,
    arb_tracking_header()
);

// ── Force: ContactInfo (Pod) ────────────────────────────────────────────

fn arb_contact_info() -> impl Strategy<Value = ContactInfo> {
    (
        any::<u8>(),
        finite_f64(),
        arb_vector3(),
        arb_point3(),
        finite_f64(),
        finite_f64(),
        finite_f32(),
        any::<u64>(),
        name_bytes_32(),
        any::<u64>(),
    )
        .prop_map(
            |(state, force, normal, point, stiff, damp, conf, start_time, frame, ts)| ContactInfo {
                state,
                contact_force: force,
                contact_normal: normal,
                contact_point: point,
                stiffness: stiff,
                damping: damp,
                confidence: conf,
                contact_start_time: start_time,
                frame_id: frame,
                timestamp_ns: ts,
            },
        )
}

roundtrip_test!(contact_info_roundtrip, ContactInfo, arb_contact_info());

// ── Control: TrajectoryPoint (Pod) ──────────────────────────────────────

fn arb_trajectory_point() -> impl Strategy<Value = TrajectoryPoint> {
    (
        f64_array3(),
        f64_array3(),
        f64_array3(),
        f64_array4(),
        f64_array3(),
        finite_f64(),
    )
        .prop_map(|(pos, vel, acc, orient, ang_vel, time)| TrajectoryPoint {
            position: pos,
            velocity: vel,
            acceleration: acc,
            orientation: orient,
            angular_velocity: ang_vel,
            time_from_start: time,
        })
}

roundtrip_test!(
    trajectory_point_roundtrip,
    TrajectoryPoint,
    arb_trajectory_point()
);

// ── Control: JointCommand (Pod) ─────────────────────────────────────────

fn arb_joint_command() -> impl Strategy<Value = JointCommand> {
    (any::<u8>(), any::<u64>()).prop_map(|(count, ts)| {
        let mut jc = JointCommand::default();
        jc.joint_count = count.min(16);
        jc.timestamp_ns = ts;
        jc
    })
}

roundtrip_test!(joint_command_roundtrip, JointCommand, arb_joint_command());

// ── Clock: RateRequest (Pod) ────────────────────────────────────────────

fn arb_rate_request() -> impl Strategy<Value = RateRequest> {
    (
        name_bytes_32(),
        finite_f64(),
        finite_f64(),
        finite_f64(),
        any::<u64>(),
        any::<u64>(),
    )
        .prop_map(
            |(topic_name, desired, min, max, requester, ts)| RateRequest {
                topic_name,
                desired_hz: desired,
                min_hz: min,
                max_hz: max,
                requester_id: requester,
                timestamp_ns: ts,
            },
        )
}

roundtrip_test!(rate_request_roundtrip, RateRequest, arb_rate_request());

// ── Clock: TimeReference (Pod) ──────────────────────────────────────────

fn arb_time_reference() -> impl Strategy<Value = TimeReference> {
    (any::<u64>(), name_bytes_32(), any::<i64>(), any::<u64>()).prop_map(
        |(time_ref, source, offset, ts)| TimeReference {
            time_ref_ns: time_ref,
            source,
            offset_ns: offset,
            timestamp_ns: ts,
        },
    )
}

roundtrip_test!(
    time_reference_roundtrip,
    TimeReference,
    arb_time_reference()
);

// ── Diagnostics: DiagnosticValue (Pod) ──────────────────────────────────

fn arb_diagnostic_value() -> impl Strategy<Value = DiagnosticValue> {
    (name_bytes_32(), name_bytes_64(), any::<u8>()).prop_map(|(key, value, vtype)| {
        DiagnosticValue {
            key,
            value,
            value_type: vtype,
        }
    })
}

roundtrip_test!(
    diagnostic_value_roundtrip,
    DiagnosticValue,
    arb_diagnostic_value()
);

// ── Diagnostics: DiagnosticReport (Pod) ─────────────────────────────────

fn arb_diagnostic_report() -> impl Strategy<Value = DiagnosticReport> {
    (name_bytes_32(), any::<u8>(), any::<u8>(), any::<u64>()).prop_map(
        |(component, value_count, level, ts)| {
            let mut dr = DiagnosticReport::default();
            dr.component = component;
            dr.value_count = value_count.min(16);
            dr.level = level;
            dr.timestamp_ns = ts;
            dr
        },
    )
}

roundtrip_test!(
    diagnostic_report_roundtrip,
    DiagnosticReport,
    arb_diagnostic_report()
);

// ── Audio: AudioFrame (Pod — large fixed-size buffer) ───────────────────

fn arb_audio_frame() -> impl Strategy<Value = Box<AudioFrame>> {
    (
        any::<u32>(),
        any::<u32>(),
        any::<u8>(),
        any::<u8>(),
        any::<u64>(),
        name_bytes_32(),
    )
        .prop_map(
            |(num_samples, sample_rate, channels, encoding, ts, frame)| {
                let mut af = Box::new(AudioFrame::default());
                af.num_samples = num_samples;
                af.sample_rate = sample_rate;
                af.channels = channels;
                af.encoding = encoding;
                af.timestamp_ns = ts;
                af.frame_id = frame;
                // samples stays zeroed — don't generate 4800 random floats
                af
            },
        )
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(100))]

    #[test]
    fn audio_frame_roundtrip(msg in arb_audio_frame()) {
        assert_bincode_roundtrip(&*msg);
        assert_msgpack_roundtrip(&*msg);
        assert_pod_roundtrip(&*msg);
    }
}

// ── Input: JoystickInput (Pod) ──────────────────────────────────────────

fn name_bytes_16() -> impl Strategy<Value = [u8; 16]> {
    proptest::collection::vec(0u8..128u8, 16).prop_map(|v| {
        let mut arr = [0u8; 16];
        arr.copy_from_slice(&v);
        arr
    })
}

fn arb_joystick_input() -> impl Strategy<Value = JoystickInput> {
    (
        any::<u32>(),
        name_bytes_16(),
        any::<u32>(),
        name_bytes_32(),
        finite_f32(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(
            |(joy_id, event_type, elem_id, elem_name, value, pressed, ts)| JoystickInput {
                joystick_id: joy_id,
                event_type,
                element_id: elem_id,
                element_name: elem_name,
                value,
                pressed,
                timestamp_ms: ts,
            },
        )
}

roundtrip_test!(
    joystick_input_roundtrip,
    JoystickInput,
    arb_joystick_input()
);

// ── Input: KeyboardInput (Pod) ──────────────────────────────────────────

fn arb_keyboard_input() -> impl Strategy<Value = KeyboardInput> {
    (
        name_bytes_32(),
        any::<u32>(),
        any::<u32>(),
        any::<u8>(),
        any::<u64>(),
    )
        .prop_map(
            |(key_name, code, modifier_flags, pressed, ts)| KeyboardInput {
                key_name,
                code,
                modifier_flags,
                pressed,
                timestamp_ms: ts,
                _padding: [0; 16],
            },
        )
}

roundtrip_test!(
    keyboard_input_roundtrip,
    KeyboardInput,
    arb_keyboard_input()
);

// ── Segmentation: SegmentationMask (Pod) ────────────────────────────────

fn arb_segmentation_mask() -> impl Strategy<Value = SegmentationMask> {
    (
        any::<u32>(),
        any::<u32>(),
        any::<u32>(),
        any::<u32>(),
        any::<u64>(),
        any::<u64>(),
        name_bytes_32(),
    )
        .prop_map(
            |(w, h, num_classes, mask_type, ts, seq, frame)| SegmentationMask {
                width: w,
                height: h,
                num_classes,
                mask_type,
                timestamp_ns: ts,
                seq,
                frame_id: frame,
            },
        )
}

serde_roundtrip_test!(
    segmentation_mask_roundtrip,
    SegmentationMask,
    arb_segmentation_mask()
);

// ── Perception: PlaneDetection (serde-only — no Pod) ────────────────────

fn arb_plane_detection() -> impl Strategy<Value = PlaneDetection> {
    (
        f64_array4(),
        arb_point3(),
        arb_vector3(),
        [finite_f64(), finite_f64()],
        any::<u32>(),
        finite_f32(),
        name_bytes_16(),
        any::<u64>(),
    )
        .prop_map(
            |(coeff, center, normal, size, inliers, conf, ptype, ts)| PlaneDetection {
                coefficients: coeff,
                center,
                normal,
                size,
                inlier_count: inliers,
                confidence: conf,
                plane_type: ptype,
                timestamp_ns: ts,
            },
        )
}

serde_roundtrip_test!(
    plane_detection_roundtrip,
    PlaneDetection,
    arb_plane_detection()
);

// ── Perception: PlaneArray (serde-only — contains PlaneDetection array) ─

fn arb_plane_array() -> impl Strategy<Value = PlaneArray> {
    (any::<u8>(), name_bytes_32(), name_bytes_32(), any::<u64>()).prop_map(
        |(count, frame, algo, ts)| PlaneArray {
            planes: [PlaneDetection::default(); 16],
            count: count.min(16),
            frame_id: frame,
            algorithm: algo,
            timestamp_ns: ts,
        },
    )
}

serde_roundtrip_test!(plane_array_roundtrip, PlaneArray, arb_plane_array());

// ═══════════════════════════════════════════════════════════════════════════
// Remaining non-enum struct types — PointField, TactileArray
// ═══════════════════════════════════════════════════════════════════════════

// ── Perception: PointField (serde-only — no Pod) ────────────────────────

fn arb_point_field() -> impl Strategy<Value = PointField> {
    (name_bytes_16(), any::<u32>(), any::<u32>()).prop_map(|(name, offset, count)| PointField {
        name,
        offset,
        datatype: horus_core::types::TensorDtype::F32,
        count,
    })
}

serde_roundtrip_test!(point_field_roundtrip, PointField, arb_point_field());

// ── Force: TactileArray (serde-only — Vec field, no Pod) ────────────────

fn arb_tactile_array() -> impl Strategy<Value = TactileArray> {
    (
        0u32..4u32,
        0u32..4u32,
        [finite_f32(), finite_f32(), finite_f32()],
        [finite_f32(), finite_f32()],
        any::<bool>(),
        [finite_f32(), finite_f32()],
        any::<u64>(),
    )
        .prop_map(
            |(rows, cols, total_force, cop, in_contact, phys_size, ts)| {
                let data_size = (rows as usize) * (cols as usize);
                TactileArray {
                    rows,
                    cols,
                    forces: vec![0.0f32; data_size],
                    total_force,
                    center_of_pressure: cop,
                    in_contact,
                    physical_size: phys_size,
                    timestamp_ns: ts,
                }
            },
        )
}

serde_roundtrip_test!(tactile_array_roundtrip, TactileArray, arb_tactile_array());

// ═══════════════════════════════════════════════════════════════════════════
// REALISTIC STRATEGIES — physically plausible sensor data
// ═══════════════════════════════════════════════════════════════════════════
//
// The strategies above use arbitrary finite floats for serialization fuzz testing.
// These strategies generate data within PHYSICAL BOUNDS — what real sensors produce.
// A test that passes with realistic data proves the system handles actual robot data.

/// Unit quaternion: norm ≈ 1.0 (valid rotation)
fn realistic_quaternion() -> impl Strategy<Value = Quaternion> {
    (finite_f64(), finite_f64(), finite_f64(), finite_f64()).prop_map(|(x, y, z, w)| {
        let norm = (x * x + y * y + z * z + w * w).sqrt();
        if norm < 1e-10 {
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            } // identity
        } else {
            Quaternion {
                x: x / norm,
                y: y / norm,
                z: z / norm,
                w: w / norm,
            }
        }
    })
}

/// IMU with gravity on Z axis ± noise, unit quaternion orientation
fn realistic_imu() -> impl Strategy<Value = Imu> {
    (
        realistic_quaternion().prop_map(|q| [q.x, q.y, q.z, q.w]),
        // Angular velocity: ±10 rad/s (typical gyroscope range)
        [-10.0f64..10.0, -10.0f64..10.0, -10.0f64..10.0],
        // Linear acceleration: gravity ± 5 m/s² (accelerometer with motion)
        [-5.0f64..5.0, -5.0f64..5.0, 4.81f64..14.81], // Z includes gravity ~9.81
        any::<u64>(),
    )
        .prop_map(|(orientation, angular_vel, linear_acc, ts)| Imu {
            orientation,
            orientation_covariance: [0.0; 9],
            angular_velocity: angular_vel,
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: linear_acc,
            linear_acceleration_covariance: [0.0; 9],
            timestamp_ns: ts,
        })
}

/// LaserScan with valid geometry: angle_min < angle_max, range_min < range_max > 0
#[allow(clippy::approx_constant)]
fn realistic_laser_scan() -> impl Strategy<Value = LaserScan> {
    (
        -3.2f32..0.0f32,  // angle_min (negative, approximately -PI)
        0.01f32..3.2f32,  // angle_max (positive, approximately PI)
        0.05f32..0.5f32,  // range_min (5cm-50cm)
        1.0f32..100.0f32, // range_max (1m-100m)
        any::<u64>(),
    )
        .prop_map(|(amin, amax, rmin, rmax, ts)| {
            let angle_inc = (amax - amin) / 360.0;
            LaserScan {
                ranges: [rmax * 0.5; 360], // mid-range readings
                angle_min: amin,
                angle_max: amax,
                range_min: rmin,
                range_max: rmax,
                angle_increment: angle_inc,
                time_increment: 0.0001,
                scan_time: 0.1,
                timestamp_ns: ts,
            }
        })
}

/// Odometry with reasonable velocities and positions
#[allow(clippy::approx_constant)]
fn realistic_odometry() -> impl Strategy<Value = Odometry> {
    (
        -1000.0f64..1000.0, // x position (meters)
        -1000.0f64..1000.0, // y position
        -3.2f64..3.2,       // theta (radians, approximately -PI..PI)
        -10.0f64..10.0,     // linear_x velocity (m/s)
        -5.0f64..5.0,       // angular_z velocity (rad/s)
        any::<u64>(),
    )
        .prop_map(|(x, y, theta, lin_x, ang_z, ts)| {
            let mut odom = Odometry::default();
            odom.pose = Pose2D {
                x,
                y,
                theta,
                timestamp_ns: ts,
            };
            odom.twist = Twist {
                linear: [lin_x, 0.0, 0.0],
                angular: [0.0, 0.0, ang_z],
                timestamp_ns: ts,
            };
            odom.timestamp_ns = ts;
            odom
        })
}

/// NavSatFix with valid GPS coordinates
fn realistic_nav_sat_fix() -> impl Strategy<Value = NavSatFix> {
    (
        -90.0f64..90.0,     // latitude
        -180.0f64..180.0,   // longitude
        -500.0f64..50000.0, // altitude (Dead Sea to Everest×5)
        0u8..4u8,           // status (0=no_fix to 3=gbas)
        0u16..32u16,        // satellites
        0.5f32..20.0f32,    // hdop
        0.5f32..20.0f32,    // vdop
        0.0f32..100.0f32,   // speed (m/s)
        any::<u64>(),
    )
        .prop_map(|(lat, lon, alt, status, sats, hdop, vdop, speed, ts)| {
            let mut fix = NavSatFix::default();
            fix.latitude = lat;
            fix.longitude = lon;
            fix.altitude = alt;
            fix.status = status;
            fix.satellites_visible = sats;
            fix.hdop = hdop;
            fix.vdop = vdop;
            fix.speed = speed;
            fix.timestamp_ns = ts;
            fix
        })
}

/// CmdVel with typical mobile robot velocity bounds
fn realistic_cmd_vel() -> impl Strategy<Value = CmdVel> {
    (
        -3.0f32..3.0, // linear (m/s) — typical mobile robot
        -3.2f32..3.2, // angular (rad/s)
        any::<u64>(),
    )
        .prop_map(|(lin, ang, ts)| CmdVel {
            linear: lin,
            angular: ang,
            timestamp_ns: ts,
        })
}

/// BoundingBox2D with valid positive dimensions in typical image coords
fn realistic_bbox2d() -> impl Strategy<Value = BoundingBox2D> {
    (
        0.0f32..1920.0, // x (pixels)
        0.0f32..1080.0, // y (pixels)
        1.0f32..500.0,  // width (positive)
        1.0f32..500.0,  // height (positive)
    )
        .prop_map(|(x, y, w, h)| BoundingBox2D {
            x,
            y,
            width: w,
            height: h,
        })
}

// ── Realistic roundtrip tests ────────────────────────────────────────────
// These verify serialization works with physically plausible data.

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    #[test]
    fn realistic_quaternion_is_unit(q in realistic_quaternion()) {
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        prop_assert!((norm - 1.0).abs() < 1e-6, "quaternion norm {} != 1.0", norm);
    }

    #[test]
    fn realistic_imu_gravity_present(imu in realistic_imu()) {
        let acc_mag = (imu.linear_acceleration[0].powi(2)
            + imu.linear_acceleration[1].powi(2)
            + imu.linear_acceleration[2].powi(2))
        .sqrt();
        // Gravity ≈ 9.81 m/s². With ±5 noise on each axis, magnitude should be 4..20
        prop_assert!(acc_mag > 1.0, "IMU acceleration magnitude {} too low", acc_mag);
        prop_assert!(acc_mag < 25.0, "IMU acceleration magnitude {} too high", acc_mag);

        // Orientation is unit quaternion
        let q_norm = (imu.orientation[0].powi(2)
            + imu.orientation[1].powi(2)
            + imu.orientation[2].powi(2)
            + imu.orientation[3].powi(2))
        .sqrt();
        prop_assert!((q_norm - 1.0).abs() < 1e-6, "IMU orientation not unit: {}", q_norm);
    }

    #[test]
    fn realistic_laser_scan_geometry_valid(scan in realistic_laser_scan()) {
        prop_assert!(scan.angle_min < scan.angle_max,
            "angle_min {} >= angle_max {}", scan.angle_min, scan.angle_max);
        prop_assert!(scan.range_min < scan.range_max,
            "range_min {} >= range_max {}", scan.range_min, scan.range_max);
        prop_assert!(scan.range_min > 0.0, "range_min {} <= 0", scan.range_min);
        prop_assert!(scan.angle_increment > 0.0,
            "angle_increment {} <= 0", scan.angle_increment);
    }

    #[test]
    fn realistic_nav_sat_fix_coordinates_valid(fix in realistic_nav_sat_fix()) {
        prop_assert!((-90.0..=90.0).contains(&fix.latitude),
            "latitude {} out of range", fix.latitude);
        prop_assert!((-180.0..=180.0).contains(&fix.longitude),
            "longitude {} out of range", fix.longitude);
        prop_assert!(fix.hdop > 0.0, "hdop {} <= 0", fix.hdop);
        prop_assert!(fix.speed >= 0.0, "speed {} < 0", fix.speed);
    }

    #[test]
    fn realistic_odometry_velocities_bounded(odom in realistic_odometry()) {
        let lin_speed = odom.twist.linear[0].abs();
        let ang_speed = odom.twist.angular[2].abs();
        prop_assert!(lin_speed <= 10.0, "linear speed {} > 10 m/s", lin_speed);
        prop_assert!(ang_speed <= 5.0, "angular speed {} > 5 rad/s", ang_speed);
    }

    #[test]
    fn realistic_cmd_vel_bounded(cmd in realistic_cmd_vel()) {
        prop_assert!(cmd.linear.abs() <= 3.0, "linear {} > 3 m/s", cmd.linear);
        prop_assert!(cmd.angular.abs() <= 3.2, "angular {} > pi rad/s", cmd.angular);
    }

    #[test]
    fn realistic_bbox2d_positive_dimensions(bbox in realistic_bbox2d()) {
        prop_assert!(bbox.width > 0.0, "width {} <= 0", bbox.width);
        prop_assert!(bbox.height > 0.0, "height {} <= 0", bbox.height);
        let area = bbox.width * bbox.height;
        prop_assert!(area > 0.0, "area {} <= 0", area);
    }
}

// Realistic roundtrip tests — verify serialization with plausible data
roundtrip_test!(realistic_imu_roundtrip, Imu, realistic_imu());
roundtrip_test!(
    realistic_laser_scan_roundtrip,
    LaserScan,
    realistic_laser_scan()
);
roundtrip_test!(realistic_odometry_roundtrip, Odometry, realistic_odometry());
roundtrip_test!(
    realistic_nav_sat_fix_roundtrip,
    NavSatFix,
    realistic_nav_sat_fix()
);
roundtrip_test!(realistic_cmd_vel_roundtrip, CmdVel, realistic_cmd_vel());
serde_roundtrip_test!(
    realistic_bbox2d_roundtrip,
    BoundingBox2D,
    realistic_bbox2d()
);
