#![allow(dead_code)]
//! Exhaustive error handling tests for HorusError.
//!
//! Tests all 11+ variants for Display, Severity classification,
//! and ? operator ergonomics across error type conversions.

use horus_core::core::DurationExt;
use horus_core::error::{
    CommunicationError, ConfigError, HorusError, HorusResult, MemoryError, NodeError,
    NotFoundError, ParseError, ResourceError, Result, SerializationError, Severity, TimeoutError,
    TransformError, ValidationError,
};

/// Create one instance of each HorusError variant and verify each produces
/// a non-empty Display message. Covers all 11+ top-level variants.
#[test]
fn test_error_variants_all_display() {
    let variants: Vec<HorusError> = vec![
        // 1. Io
        HorusError::Io(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "device not found",
        )),
        // 2. Config
        HorusError::Config(ConfigError::MissingField {
            field: "rate".to_string(),
            context: Some("robot.toml".to_string()),
        }),
        // 3. Communication
        HorusError::Communication(CommunicationError::TopicFull {
            topic: "cmd_vel".to_string(),
        }),
        // 4. Node
        HorusError::Node(NodeError::InitPanic {
            node: "motor_ctrl".to_string(),
        }),
        // 5. Memory
        HorusError::Memory(MemoryError::PoolExhausted {
            reason: "all 64 slots occupied".to_string(),
        }),
        // 6. Serialization
        HorusError::Serialization(SerializationError::Other {
            format: "binary".to_string(),
            reason: "unexpected EOF".to_string(),
        }),
        // 7. NotFound
        HorusError::NotFound(NotFoundError::Topic {
            name: "odom".to_string(),
        }),
        // 8. Resource
        HorusError::Resource(ResourceError::AlreadyExists {
            resource_type: "frame".to_string(),
            name: "base_link".to_string(),
        }),
        // 9. InvalidInput
        HorusError::InvalidInput(ValidationError::OutOfRange {
            field: "frequency".to_string(),
            min: "1".to_string(),
            max: "1000".to_string(),
            actual: "-5".to_string(),
        }),
        // 10. Parse
        HorusError::Parse(ParseError::Custom {
            type_name: "Duration".to_string(),
            input: "abc".to_string(),
            reason: "not a valid duration".to_string(),
        }),
        // 11. InvalidDescriptor
        HorusError::InvalidDescriptor("bad magic bytes 0xDEAD".to_string()),
        // 12. Transform
        HorusError::Transform(TransformError::Extrapolation {
            frame: "camera_rgb".to_string(),
            requested_ns: 1000,
            oldest_ns: 5000,
            newest_ns: 10000,
        }),
        // 13. Timeout
        HorusError::Timeout(TimeoutError {
            resource: "tensor_pool".to_string(),
            elapsed: 500_u64.ms(),
            deadline: Some(100_u64.ms()),
        }),
        // 14. Internal
        HorusError::Internal {
            message: "unexpected scheduler state".to_string(),
            file: "src/scheduler.rs",
            line: 42,
        },
        // 15. Contextual
        HorusError::Contextual {
            message: "initializing IMU driver".to_string(),
            source: Box::new(std::io::Error::new(
                std::io::ErrorKind::PermissionDenied,
                "no access to /dev/i2c-1",
            )),
        },
    ];

    // Must have at least 8 variants (the requirement says "at least 8 of the 11")
    assert!(
        variants.len() >= 8,
        "Must test at least 8 variants, got {}",
        variants.len()
    );

    for (i, err) in variants.iter().enumerate() {
        let display = format!("{}", err);
        assert!(
            !display.is_empty(),
            "Variant #{} ({:?}) should produce non-empty Display message",
            i,
            std::mem::discriminant(err)
        );
        // Also verify Debug works
        let debug = format!("{:?}", err);
        assert!(
            !debug.is_empty(),
            "Variant #{} should produce non-empty Debug output",
            i
        );
    }
}

/// Test that each error variant has the correct Severity classification.
/// Minimum requirements:
///   - Communication(TopicFull) is Transient
///   - Timeout is Transient
///   - NotFound is Permanent
///   - InvalidDescriptor is Fatal
#[test]
fn test_error_severity_classification() {
    // === Transient errors (retry may succeed) ===

    let err = HorusError::Communication(CommunicationError::TopicFull {
        topic: "cmd_vel".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Transient,
        "TopicFull should be Transient"
    );

    let err = HorusError::Communication(CommunicationError::NetworkFault {
        peer: "192.168.1.10".to_string(),
        reason: "connection refused".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Transient,
        "NetworkFault should be Transient"
    );

    let err = HorusError::Timeout(TimeoutError {
        resource: "sensor_pool".to_string(),
        elapsed: 100_u64.ms(),
        deadline: None,
    });
    assert_eq!(
        err.severity(),
        Severity::Transient,
        "Timeout should be Transient"
    );

    let err = HorusError::Transform(TransformError::Stale {
        frame: "imu".to_string(),
        age: 2_u64.secs(),
        threshold: 500_u64.ms(),
    });
    assert_eq!(
        err.severity(),
        Severity::Transient,
        "Transform::Stale should be Transient"
    );

    let err = HorusError::Memory(MemoryError::PoolExhausted {
        reason: "all slots used".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Transient,
        "PoolExhausted should be Transient"
    );

    // === Permanent errors (won't succeed on retry, but system continues) ===

    let err = HorusError::NotFound(NotFoundError::Frame {
        name: "world".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Permanent,
        "NotFound should be Permanent"
    );

    let err = HorusError::Config(ConfigError::Other("bad config".to_string()));
    assert_eq!(
        err.severity(),
        Severity::Permanent,
        "Config should be Permanent"
    );

    let err = HorusError::InvalidInput(ValidationError::Other("negative frequency".to_string()));
    assert_eq!(
        err.severity(),
        Severity::Permanent,
        "InvalidInput should be Permanent"
    );

    let err = HorusError::Parse(ParseError::Custom {
        type_name: "f32".to_string(),
        input: "abc".to_string(),
        reason: "not a number".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Permanent,
        "Parse should be Permanent"
    );

    let err = HorusError::Resource(ResourceError::AlreadyExists {
        resource_type: "topic".to_string(),
        name: "cmd_vel".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Permanent,
        "Resource::AlreadyExists should be Permanent"
    );

    // === Fatal errors (data integrity or unrecoverable) ===

    let err = HorusError::InvalidDescriptor("corrupt IPC tensor".to_string());
    assert_eq!(
        err.severity(),
        Severity::Fatal,
        "InvalidDescriptor should be Fatal"
    );

    let err = HorusError::Memory(MemoryError::ShmCreateFailed {
        path: "/dev/shm/horus".to_string(),
        reason: "EACCES".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Fatal,
        "ShmCreateFailed should be Fatal"
    );

    let err = HorusError::Memory(MemoryError::MmapFailed {
        reason: "out of memory".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Fatal,
        "MmapFailed should be Fatal"
    );

    let err = HorusError::Node(NodeError::InitPanic {
        node: "motor".to_string(),
    });
    assert_eq!(err.severity(), Severity::Fatal, "InitPanic should be Fatal");

    let err = HorusError::Node(NodeError::ReInitPanic {
        node: "motor".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Fatal,
        "ReInitPanic should be Fatal"
    );

    let err = HorusError::Node(NodeError::ShutdownPanic {
        node: "motor".to_string(),
    });
    assert_eq!(
        err.severity(),
        Severity::Fatal,
        "ShutdownPanic should be Fatal"
    );
}

/// Verify that HorusResult<T> works ergonomically with the ? operator
/// across multiple error types. This is primarily a compile-time test
/// demonstrating that From conversions allow seamless ? usage.
#[test]
fn test_horus_result_question_mark_ergonomics() {
    // Function using ? with std::io::Error => HorusError::Io
    fn read_sensor_config() -> Result<String> {
        let _ = std::fs::File::open("/nonexistent/sensor_config.yaml")?;
        Ok("config loaded".to_string())
    }

    // Function using ? with ParseIntError => HorusError::Parse
    fn parse_sensor_rate() -> HorusResult<i32> {
        let val: i32 = "not_a_number".parse()?;
        Ok(val)
    }

    // Function using ? with serde_json::Error => HorusError::Serialization
    fn parse_sensor_json() -> Result<serde_json::Value> {
        let val: serde_json::Value = serde_json::from_str("{{invalid json")?;
        Ok(val)
    }

    // Function using ? with ConfigError => HorusError::Config (via From)
    fn validate_config() -> Result<()> {
        let err = ConfigError::MissingField {
            field: "rate".to_string(),
            context: None,
        };
        Err(err)?;
        Ok(())
    }

    // Function using ? with CommunicationError => HorusError::Communication
    fn publish_message() -> Result<()> {
        let err = CommunicationError::TopicNotFound {
            topic: "cmd_vel".to_string(),
        };
        Err(err)?;
        Ok(())
    }

    // Function chaining multiple ? operators in one body
    fn complex_operation() -> Result<String> {
        // Each of these ? operators uses a different From impl
        let _io_result: std::result::Result<std::fs::File, std::io::Error> =
            std::fs::File::open("/this/does/not/exist");
        if let Err(e) = _io_result {
            // Manual conversion works too
            let _: HorusError = e.into();
        }

        // ParseFloatError => HorusError::Parse
        let _float_result: std::result::Result<f64, std::num::ParseFloatError> =
            "not_float".parse();
        if let Err(e) = _float_result {
            let _: HorusError = e.into();
        }

        // ParseBoolError => HorusError::Parse
        let _bool_result: std::result::Result<bool, std::str::ParseBoolError> = "maybe".parse();
        if let Err(e) = _bool_result {
            let _: HorusError = e.into();
        }

        Ok("success".to_string())
    }

    // Verify all functions compile and return the expected error types
    let io_err = read_sensor_config();
    assert!(io_err.is_err());
    assert!(
        matches!(io_err.unwrap_err(), HorusError::Io(_)),
        "io::Error should convert to HorusError::Io via ?"
    );

    let parse_err = parse_sensor_rate();
    assert!(parse_err.is_err());
    assert!(
        matches!(parse_err.unwrap_err(), HorusError::Parse(_)),
        "ParseIntError should convert to HorusError::Parse via ?"
    );

    let json_err = parse_sensor_json();
    assert!(json_err.is_err());
    assert!(
        matches!(
            json_err.unwrap_err(),
            HorusError::Serialization(SerializationError::Json { .. })
        ),
        "serde_json::Error should convert to HorusError::Serialization(Json) via ?"
    );

    let config_err = validate_config();
    assert!(config_err.is_err());
    assert!(
        matches!(
            config_err.unwrap_err(),
            HorusError::Config(ConfigError::MissingField { .. })
        ),
        "ConfigError should convert to HorusError::Config via ?"
    );

    let comm_err = publish_message();
    assert!(comm_err.is_err());
    assert!(
        matches!(
            comm_err.unwrap_err(),
            HorusError::Communication(CommunicationError::TopicNotFound { .. })
        ),
        "CommunicationError should convert to HorusError::Communication via ?"
    );

    let complex_result = complex_operation();
    assert!(
        complex_result.is_ok(),
        "complex_operation should succeed (only tests conversions)"
    );
}
