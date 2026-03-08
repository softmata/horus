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
//! # What It Generates
//!
//! For `message! { Foo { x: f32, y: f32 } }`, the macro generates:
//!
//! ```rust,ignore
//! #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
//! pub struct Foo {
//!     pub x: f32,
//!     pub y: f32,
//! }
//!
//! impl LogSummary for Foo { ... }
//! ```
//!
//! The struct automatically satisfies `TopicMessage` via the blanket impl —
//! no additional trait implementation is needed.

/// Macro for defining HORUS message types for use with `Topic<T>`.
///
/// Generates a struct with all required derives (`Clone`, `Debug`,
/// `Serialize`, `Deserialize`) and a `LogSummary` implementation.
/// The resulting type is immediately usable with `Topic<T>`.
///
/// # Single Message
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
/// # Multiple Messages
///
/// ```rust
/// use horus_core::message;
///
/// message! {
///     /// Velocity command
///     CmdVel {
///         linear_x: f64,
///         angular_z: f64,
///     }
///
///     /// Battery status
///     BatteryStatus {
///         voltage: f32,
///         current: f32,
///         percentage: f32,
///     }
/// }
/// ```
///
/// # PartialEq / Default
///
/// Use `#[derive(...)]` on individual messages for extra derives:
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
    // Multiple messages in one block
    (
        $(
            $(#[$meta:meta])*
            $name:ident {
                $(
                    $(#[$field_meta:meta])*
                    $field:ident : $ty:ty
                ),* $(,)?
            }
        )+
    ) => {
        $(
            $crate::message!(@single $(#[$meta])* $name { $( $(#[$field_meta])* $field : $ty ),* });
        )+
    };

    // Internal: single message
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
                // Build a compact summary: "TypeName(field1=val, field2=val)"
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
}
