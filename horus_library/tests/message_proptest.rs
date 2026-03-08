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
        stamp_nanos: ts,
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
