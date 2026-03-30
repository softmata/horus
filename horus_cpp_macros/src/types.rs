//! Intermediate representation types for C++ binding metadata.
//!
//! These types capture the information extracted from `#[horus_api]` annotations
//! and `#[cpp(...)]` hints, which is then consumed by the CXX bridge generator
//! and C++ header generator.

use proc_macro2::Ident;
use syn::{Generics, Type, Visibility};

/// Top-level binding metadata for a single annotated item.
#[derive(Debug, Clone)]
pub enum BindingItem {
    /// `#[horus_api]` on a struct definition.
    Struct(StructBinding),
    /// `#[horus_api]` on an impl block.
    Impl(ImplBinding),
}

/// Metadata extracted from `#[horus_api]` on a struct.
#[derive(Debug, Clone)]
pub struct StructBinding {
    pub name: Ident,
    pub generics: Generics,
    pub fields: Vec<FieldBinding>,
    pub hints: CppHints,
    pub vis: Visibility,
}

/// Metadata extracted from `#[horus_api]` on an impl block.
#[derive(Debug, Clone)]
pub struct ImplBinding {
    pub self_ty: Type,
    pub generics: Generics,
    pub methods: Vec<MethodBinding>,
}

/// A single struct field with its C++ hints.
#[derive(Debug, Clone)]
pub struct FieldBinding {
    pub name: Ident,
    pub ty: Type,
    pub vis: Visibility,
    pub hints: CppHints,
}

/// A single method from an impl block with its C++ hints.
#[derive(Debug, Clone)]
pub struct MethodBinding {
    pub name: Ident,
    pub sig: MethodSig,
    pub hints: CppHints,
    pub vis: Visibility,
}

/// Simplified method signature for code generation.
#[derive(Debug, Clone)]
pub struct MethodSig {
    pub receiver: Receiver,
    pub params: Vec<ParamBinding>,
    pub return_ty: Option<Type>,
    pub is_result: bool,
    pub is_option: bool,
}

/// Method receiver type.
#[derive(Debug, Clone, PartialEq)]
pub enum Receiver {
    /// No self — static method / associated function.
    None,
    /// `&self`
    Ref,
    /// `&mut self`
    RefMut,
    /// `self` — consumes ownership.
    Owned,
}

/// A single method parameter.
#[derive(Debug, Clone)]
pub struct ParamBinding {
    pub name: Ident,
    pub ty: Type,
    pub hints: CppHints,
}

/// C++ code generation hints from `#[cpp(...)]` attributes.
#[derive(Debug, Default, Clone)]
pub struct CppHints {
    /// Generate `operator->` for direct field access.
    pub direct_field_access: bool,
    /// Parameter uses move semantics (`T&&`) in C++.
    pub move_semantics: bool,
    /// Return type wrapped in `std::unique_ptr<T>`.
    pub returns_unique_ptr: bool,
    /// Destructor calls Rust Drop (RAII pattern).
    pub raii: bool,
    /// Custom C++ type name override.
    pub cpp_name: Option<String>,
    /// Custom C++ namespace override.
    pub cpp_namespace: Option<String>,
}

impl CppHints {
    /// Returns true if any hint is set.
    pub fn has_any(&self) -> bool {
        self.direct_field_access
            || self.move_semantics
            || self.returns_unique_ptr
            || self.raii
            || self.cpp_name.is_some()
            || self.cpp_namespace.is_some()
    }
}

// ─── Monomorphization ────────────────────────────────────────────────────────

/// A concrete type to monomorphize a generic over.
///
/// When the codegen encounters `Publisher<T>`, it generates concrete FFI functions
/// for each `MonomorphTarget` in the active set.
#[derive(Debug, Clone)]
pub struct MonomorphTarget {
    /// Rust type name (e.g., "CmdVel", "LaserScan").
    pub rust_name: &'static str,
    /// Snake-case name for FFI function naming (e.g., "cmd_vel", "laser_scan").
    pub snake_name: &'static str,
    /// Full Rust path for use in generated code (e.g., "horus_library::CmdVel").
    pub rust_path: &'static str,
}

/// Default set of message types to monomorphize.
/// Covers 95%+ of typical robotics applications.
pub const DEFAULT_MONOMORPH_TYPES: &[MonomorphTarget] = &[
    // ─── Geometry ────────────────────────
    MonomorphTarget {
        rust_name: "Twist",
        snake_name: "twist",
        rust_path: "horus_library::Twist",
    },
    MonomorphTarget {
        rust_name: "Pose2D",
        snake_name: "pose2d",
        rust_path: "horus_library::Pose2D",
    },
    MonomorphTarget {
        rust_name: "Pose3D",
        snake_name: "pose3d",
        rust_path: "horus_library::Pose3D",
    },
    MonomorphTarget {
        rust_name: "PoseStamped",
        snake_name: "pose_stamped",
        rust_path: "horus_library::PoseStamped",
    },
    MonomorphTarget {
        rust_name: "TransformStamped",
        snake_name: "transform_stamped",
        rust_path: "horus_library::TransformStamped",
    },
    // ─── Sensor ──────────────────────────
    MonomorphTarget {
        rust_name: "CmdVel",
        snake_name: "cmd_vel",
        rust_path: "horus_library::CmdVel",
    },
    MonomorphTarget {
        rust_name: "LaserScan",
        snake_name: "laser_scan",
        rust_path: "horus_library::LaserScan",
    },
    MonomorphTarget {
        rust_name: "Imu",
        snake_name: "imu",
        rust_path: "horus_library::Imu",
    },
    MonomorphTarget {
        rust_name: "Odometry",
        snake_name: "odometry",
        rust_path: "horus_library::Odometry",
    },
    MonomorphTarget {
        rust_name: "JointState",
        snake_name: "joint_state",
        rust_path: "horus_library::JointState",
    },
    MonomorphTarget {
        rust_name: "BatteryState",
        snake_name: "battery_state",
        rust_path: "horus_library::BatteryState",
    },
    MonomorphTarget {
        rust_name: "NavSatFix",
        snake_name: "nav_sat_fix",
        rust_path: "horus_library::NavSatFix",
    },
    // ─── Control ─────────────────────────
    MonomorphTarget {
        rust_name: "MotorCommand",
        snake_name: "motor_command",
        rust_path: "horus_library::MotorCommand",
    },
    MonomorphTarget {
        rust_name: "ServoCommand",
        snake_name: "servo_command",
        rust_path: "horus_library::ServoCommand",
    },
    MonomorphTarget {
        rust_name: "TrajectoryPoint",
        snake_name: "trajectory_point",
        rust_path: "horus_library::TrajectoryPoint",
    },
    MonomorphTarget {
        rust_name: "JointCommand",
        snake_name: "joint_command",
        rust_path: "horus_library::JointCommand",
    },
    // ─── Navigation ──────────────────────
    MonomorphTarget {
        rust_name: "NavGoal",
        snake_name: "nav_goal",
        rust_path: "horus_library::NavGoal",
    },
    MonomorphTarget {
        rust_name: "NavPath",
        snake_name: "nav_path",
        rust_path: "horus_library::NavPath",
    },
    // ─── Detection ───────────────────────
    MonomorphTarget {
        rust_name: "Detection",
        snake_name: "detection",
        rust_path: "horus_library::Detection",
    },
    MonomorphTarget {
        rust_name: "Detection3D",
        snake_name: "detection3d",
        rust_path: "horus_library::Detection3D",
    },
    // ─── Vision ──────────────────────────
    MonomorphTarget {
        rust_name: "CameraInfo",
        snake_name: "camera_info",
        rust_path: "horus_library::CameraInfo",
    },
    // ─── Diagnostics ─────────────────────
    MonomorphTarget {
        rust_name: "Heartbeat",
        snake_name: "heartbeat",
        rust_path: "horus_library::Heartbeat",
    },
    MonomorphTarget {
        rust_name: "EmergencyStop",
        snake_name: "emergency_stop",
        rust_path: "horus_library::EmergencyStop",
    },
    // ─── Transform ───────────────────────
    MonomorphTarget {
        rust_name: "TFMessage",
        snake_name: "tf_message",
        rust_path: "horus_library::TFMessage",
    },
    // ─── Force ───────────────────────────
    MonomorphTarget {
        rust_name: "WrenchStamped",
        snake_name: "wrench_stamped",
        rust_path: "horus_library::WrenchStamped",
    },
];
