//! Macro for defining HORUS message types.
//!
//! The [`message!`] macro provides a declarative way to define message types
//! for use with `Topic<T>`. It auto-derives all required traits so messages
//! work with zero configuration.
//!
//! # Example
//!
//! ```rust
//! use horus_core::message;
//!
//! message! {
//!     /// Motor command sent to actuators
//!     MotorCommand {
//!         velocity: f32,
//!         torque: f32,
//!     }
//! }
//!
//! let cmd = MotorCommand { velocity: 1.0, torque: 0.5 };
//! // Ready for: Topic<MotorCommand>
//! ```
//!
//! # Fixed Messages (Zero-Copy)
//!
//! Add `#[fixed]` to enable zero-copy shared memory transport (~50ns vs ~167ns).
//! Fixed messages must only contain primitive/Copy fields — no `String`, `Vec`, etc.
//!
//! ```rust
//! use horus_core::message;
//!
//! message! {
//!     #[fixed]
//!     /// Fast sensor reading — zero-copy via shared memory
//!     ImuReading {
//!         accel_x: f64,
//!         accel_y: f64,
//!         accel_z: f64,
//!         timestamp_ns: u64,
//!     }
//! }
//! ```
//!
//! # What It Generates
//!
//! **Without `#[fixed]`** (flexible — any fields):
//! ```rust,ignore
//! #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
//! pub struct Foo { pub x: f32, pub y: f32 }
//! impl LogSummary for Foo { ... }
//! ```
//!
//! **With `#[fixed]`** (zero-copy — primitives only):
//! ```rust,ignore
//! #[repr(C)]
//! #[derive(Clone, Copy, Default, Debug, serde::Serialize, serde::Deserialize)]
//! pub struct Foo { pub x: f32, pub y: f32 }
//! impl LogSummary for Foo { ... }
//! ```
//!
//! The struct automatically satisfies `TopicMessage` via the blanket impl —
//! no additional trait implementation is needed. Fixed messages additionally
//! get the zero-copy SHM path automatically (detected via `!needs_drop`).

/// Macro for defining HORUS message types for use with `Topic<T>`.
///
/// Generates a struct with all required derives and a `LogSummary` implementation.
/// The resulting type is immediately usable with `Topic<T>`.
///
/// # Flexible Messages (default)
///
/// Any field types — uses serialization transport (~167ns).
///
/// ```rust
/// use horus_core::message;
///
/// message! {
///     /// A sensor reading
///     SensorReading {
///         temperature: f64,
///         humidity: f64,
///         timestamp_ns: u64,
///     }
/// }
/// ```
///
/// # Fixed Messages (zero-copy)
///
/// Add `#[fixed]` for zero-copy shared memory transport (~50ns).
/// All fields must be `Copy` — no `String`, `Vec`, `Box`, etc.
///
/// ```rust
/// use horus_core::message;
///
/// message! {
///     #[fixed]
///     CmdVel {
///         linear: f32,
///         angular: f32,
///     }
/// }
/// ```
///
/// # Extra Derives
///
/// Use `#[derive(...)]` on individual messages:
///
/// ```rust
/// use horus_core::message;
///
/// message! {
///     #[derive(PartialEq)]
///     JointState {
///         position: f64,
///         velocity: f64,
///         effort: f64,
///     }
/// }
/// ```
#[macro_export]
macro_rules! message {
    // Internal: munch #[fixed] message
    (@munch #[fixed] $(#[$meta:meta])* $name:ident { $( $(#[$field_meta:meta])* $field:ident : $ty:ty ),* $(,)? } $($rest:tt)*) => {
        $crate::message!(@fixed $(#[$meta])* $name { $( $(#[$field_meta])* $field : $ty ),* });
        $crate::message!(@munch $($rest)*);
    };

    // Internal: munch regular message
    (@munch $(#[$meta:meta])* $name:ident { $( $(#[$field_meta:meta])* $field:ident : $ty:ty ),* $(,)? } $($rest:tt)*) => {
        $crate::message!(@single $(#[$meta])* $name { $( $(#[$field_meta])* $field : $ty ),* });
        $crate::message!(@munch $($rest)*);
    };

    // Internal: munch done
    (@munch) => {};

    // Internal: flexible message (default — any fields)
    (@single
        $(#[$meta:meta])*
        $name:ident {
            $(
                $(#[$field_meta:meta])*
                $field:ident : $ty:ty
            ),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        pub struct $name {
            $(
                $(#[$field_meta])*
                pub $field: $ty,
            )*
        }

        impl $crate::core::LogSummary for $name {
            fn log_summary(&self) -> String {
                let mut s = String::from(stringify!($name));
                s.push('(');
                let mut _first = true;
                $(
                    if !_first { s.push_str(", "); }
                    _first = false;
                    s.push_str(stringify!($field));
                    s.push('=');
                    s.push_str(&format!("{:?}", self.$field));
                )*
                s.push(')');
                s
            }
        }
    };

    // Internal: fixed message (zero-copy — Copy fields only, repr(C))
    (@fixed
        $(#[$meta:meta])*
        $name:ident {
            $(
                $(#[$field_meta:meta])*
                $field:ident : $ty:ty
            ),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[repr(C)]
        #[derive(Clone, Copy, Default, Debug, serde::Serialize, serde::Deserialize)]
        pub struct $name {
            $(
                $(#[$field_meta])*
                pub $field: $ty,
            )*
        }

        impl $crate::core::LogSummary for $name {
            fn log_summary(&self) -> String {
                let mut s = String::from(stringify!($name));
                s.push('(');
                let mut _first = true;
                $(
                    if !_first { s.push_str(", "); }
                    _first = false;
                    s.push_str(stringify!($field));
                    s.push('=');
                    s.push_str(&format!("{:?}", self.$field));
                )*
                s.push(')');
                s
            }
        }
    };

    // Entry point: start token-tree munching
    ($($input:tt)+) => {
        $crate::message!(@munch $($input)+);
    };
}

#[cfg(test)]
mod tests {
    use crate::core::LogSummary;

    message! {
        /// Test motor command
        TestMotorCmd {
            velocity: f32,
            torque: f32,
        }
    }

    message! {
        TestSensor {
            temperature: f64,
            humidity: f64,
        }

        TestPose {
            x: f64,
            y: f64,
            theta: f64,
        }
    }

    #[test]
    fn test_message_macro_basic() {
        let cmd = TestMotorCmd {
            velocity: 1.0,
            torque: 0.5,
        };
        assert_eq!(cmd.velocity, 1.0);
        assert_eq!(cmd.torque, 0.5);
    }

    #[test]
    fn test_message_macro_clone() {
        let cmd = TestMotorCmd {
            velocity: 2.0,
            torque: 1.0,
        };
        let cmd2 = cmd.clone();
        assert_eq!(cmd.velocity, cmd2.velocity);
        assert_eq!(cmd.torque, cmd2.torque);
    }

    #[test]
    fn test_message_macro_debug() {
        let cmd = TestMotorCmd {
            velocity: 1.0,
            torque: 0.5,
        };
        let debug = format!("{:?}", cmd);
        assert!(debug.contains("velocity"));
        assert!(debug.contains("torque"));
    }

    #[test]
    fn test_message_macro_serde() {
        let cmd = TestMotorCmd {
            velocity: 3.0,
            torque: 2.0,
        };
        let json = serde_json::to_string(&cmd).unwrap();
        let deserialized: TestMotorCmd = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.velocity, 3.0);
        assert_eq!(deserialized.torque, 2.0);
    }

    #[test]
    fn test_message_macro_log_summary() {
        let cmd = TestMotorCmd {
            velocity: 1.5,
            torque: 0.8,
        };
        let summary = cmd.log_summary();
        assert!(summary.starts_with("TestMotorCmd("));
        assert!(summary.contains("velocity"));
        assert!(summary.contains("torque"));
    }

    #[test]
    fn test_message_macro_multiple() {
        let sensor = TestSensor {
            temperature: 25.0,
            humidity: 60.0,
        };
        let pose = TestPose {
            x: 1.0,
            y: 2.0,
            theta: 0.5,
        };
        assert_eq!(sensor.temperature, 25.0);
        assert_eq!(pose.x, 1.0);
    }

    #[test]
    fn test_message_satisfies_topic_message_bounds() {
        // The blanket TopicMessage impl requires: Clone + Send + Sync + Serialize + DeserializeOwned + 'static
        // Verify our generated type satisfies these bounds
        fn assert_topic_message_bounds<
            T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
        >() {
        }
        assert_topic_message_bounds::<TestMotorCmd>();
        assert_topic_message_bounds::<TestSensor>();
        assert_topic_message_bounds::<TestPose>();
    }

    // ── #[fixed] message tests ─────────────────────────────────────────

    message! {
        #[fixed]
        /// A fixed zero-copy message
        TestFixedCmd {
            velocity: f32,
            torque: f32,
        }
    }

    message! {
        #[fixed]
        TestFixedImu {
            accel_x: f64,
            accel_y: f64,
            accel_z: f64,
            timestamp_ns: u64,
        }
    }

    #[test]
    fn test_fixed_message_is_copy() {
        let cmd = TestFixedCmd {
            velocity: 1.0,
            torque: 0.5,
        };
        let cmd2 = cmd; // Copy, not move
        assert_eq!(cmd.velocity, cmd2.velocity);
        assert_eq!(cmd.torque, cmd2.torque);
    }

    #[test]
    fn test_fixed_message_is_default() {
        let cmd = TestFixedCmd::default();
        assert_eq!(cmd.velocity, 0.0);
        assert_eq!(cmd.torque, 0.0);
    }

    #[test]
    fn test_fixed_message_is_repr_c() {
        // repr(C) means fields are laid out in declaration order with no reordering
        // We verify the size is predictable (no padding surprises for these types)
        assert_eq!(std::mem::size_of::<TestFixedCmd>(), 8); // 4 + 4
        assert_eq!(std::mem::size_of::<TestFixedImu>(), 32); // 8 + 8 + 8 + 8
    }

    #[test]
    fn test_fixed_message_is_pod_detected() {
        // Fixed messages should be auto-detected as POD for zero-copy transport
        assert!(super::super::pod::is_pod::<TestFixedCmd>());
        assert!(super::super::pod::is_pod::<TestFixedImu>());
    }

    #[test]
    fn test_fixed_message_debug() {
        let cmd = TestFixedCmd {
            velocity: 1.0,
            torque: 0.5,
        };
        let debug = format!("{:?}", cmd);
        assert!(debug.contains("velocity"));
        assert!(debug.contains("torque"));
    }

    #[test]
    fn test_fixed_message_serde() {
        let imu = TestFixedImu {
            accel_x: 0.1,
            accel_y: 0.2,
            accel_z: 9.8,
            timestamp_ns: 12345,
        };
        let json = serde_json::to_string(&imu).unwrap();
        let restored: TestFixedImu = serde_json::from_str(&json).unwrap();
        assert_eq!(restored.accel_x, 0.1);
        assert_eq!(restored.timestamp_ns, 12345);
    }

    #[test]
    fn test_fixed_message_log_summary() {
        let cmd = TestFixedCmd {
            velocity: 1.5,
            torque: 0.8,
        };
        let summary = cmd.log_summary();
        assert!(summary.starts_with("TestFixedCmd("));
        assert!(summary.contains("velocity"));
    }

    #[test]
    fn test_fixed_message_satisfies_topic_bounds() {
        fn assert_topic_message_bounds<
            T: Clone
                + Copy
                + Default
                + Send
                + Sync
                + serde::Serialize
                + serde::de::DeserializeOwned
                + 'static,
        >() {
        }
        assert_topic_message_bounds::<TestFixedCmd>();
        assert_topic_message_bounds::<TestFixedImu>();
    }

    // ── Mixed block: #[fixed] and flexible in same message! ────────────

    message! {
        #[fixed]
        TestMixedFixed {
            x: f32,
            y: f32,
        }

        TestMixedFlex {
            name: String,
            value: f64,
        }
    }

    #[test]
    fn test_mixed_block_fixed_is_copy() {
        let a = TestMixedFixed { x: 1.0, y: 2.0 };
        let b = a; // Copy
        assert_eq!(a.x, b.x);
    }

    #[test]
    fn test_mixed_block_flex_is_not_copy() {
        let a = TestMixedFlex {
            name: "hello".to_string(),
            value: 2.75,
        };
        let b = a.clone(); // Must clone, not copy
        assert_eq!(b.name, "hello");
    }
}
