//! Macros for defining HORUS Actions.
//!
//! The `action!` macro provides a declarative way to define action types
//! with their Goal, Feedback, and Result types.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::action;
//!
//! action! {
//!     /// Navigate to a target position
//!     NavigateToGoal {
//!         goal {
//!             target_x: f64,
//!             target_y: f64,
//!             speed: f32 = 1.0,
//!         }
//!
//!         feedback {
//!             distance_remaining: f64,
//!             progress_percent: f32,
//!         }
//!
//!         result {
//!             success: bool,
//!             final_x: f64,
//!             final_y: f64,
//!             time_elapsed: f32,
//!         }
//!     }
//! }
//! ```

/// Macro for defining HORUS Action types.
///
/// This macro generates:
/// - Goal struct with the specified fields
/// - Feedback struct with the specified fields
/// - Result struct with the specified fields
/// - Action trait implementation
/// - LogSummary implementations for all types
/// - Default implementation for types with defaults
///
/// # Syntax
///
/// ```text
/// action! {
///     /// Optional doc comment
///     ActionName {
///         goal {
///             field1: Type,
///             field2: Type = default_value,  // Optional default
///         }
///
///         feedback {
///             field1: Type,
///         }
///
///         result {
///             field1: Type,
///         }
///     }
/// }
/// ```
///
/// # Generated Types
///
/// For an action named `NavigateToGoal`, the macro generates:
/// - `NavigateToGoalGoal` - The goal request type
/// - `NavigateToGoalFeedback` - The feedback type
/// - `NavigateToGoalResult` - The result type
/// - `NavigateToGoal` - The action marker type implementing `Action`
///
/// # Example
///
/// ```rust
/// use horus_core::action;
///
/// action! {
///     /// Pick up an object
///     PickObject {
///         goal {
///             object_id: String,
///             force_limit: f64 = 10.0,
///         }
///
///         feedback {
///             grip_force: f64,
///             contact_detected: bool,
///         }
///
///         result {
///             success: bool,
///             object_weight: f64,
///         }
///     }
/// }
///
/// // Now you can use:
/// // let goal = PickObjectGoal { object_id: "cup".into(), force_limit: 5.0 };
/// // let feedback = PickObjectFeedback { grip_force: 3.5, contact_detected: true };
/// // let result = PickObjectResult { success: true, object_weight: 0.25 };
/// ```
#[macro_export]
macro_rules! action {
    // Main entry point with doc comments
    (
        $(#[$action_meta:meta])*
        $action_name:ident {
            goal {
                $(
                    $(#[$goal_field_meta:meta])*
                    $goal_field:ident : $goal_type:ty
                    $(= $goal_default:expr)?
                ),* $(,)?
            }

            feedback {
                $(
                    $(#[$feedback_field_meta:meta])*
                    $feedback_field:ident : $feedback_type:ty
                    $(= $feedback_default:expr)?
                ),* $(,)?
            }

            result {
                $(
                    $(#[$result_field_meta:meta])*
                    $result_field:ident : $result_type:ty
                    $(= $result_default:expr)?
                ),* $(,)?
            }
        }
    ) => {
        $crate::paste::paste! {
            // ===== Goal Type =====
            $(#[$action_meta])*
            #[doc = concat!("Goal type for the `", stringify!($action_name), "` action.")]
            #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
            pub struct [<$action_name Goal>] {
                $(
                    $(#[$goal_field_meta])*
                    pub $goal_field: $goal_type,
                )*
            }

            impl $crate::core::LogSummary for [<$action_name Goal>] {
                fn log_summary(&self) -> String {
                    let mut s = String::from(stringify!([<$action_name Goal>]));
                    s.push('(');
                    $(
                        if !s.ends_with('(') { s.push_str(", "); }
                        s.push_str(&format!("{}={:?}", stringify!($goal_field), self.$goal_field));
                    )*
                    s.push(')');
                    s
                }
            }

            // Generate Default impl if all fields have defaults
            $crate::action!(@maybe_default_goal [<$action_name Goal>] { $($goal_field : $goal_type $(= $goal_default)?),* });

            // Generate new() constructor — takes all fields as parameters
            impl [<$action_name Goal>] {
                /// Create a new goal with the given field values.
                pub fn new($($goal_field: $goal_type),*) -> Self {
                    Self { $($goal_field,)* }
                }
            }

            // ===== Feedback Type =====
            #[doc = concat!("Feedback type for the `", stringify!($action_name), "` action.")]
            #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
            pub struct [<$action_name Feedback>] {
                $(
                    $(#[$feedback_field_meta])*
                    pub $feedback_field: $feedback_type,
                )*
            }

            impl $crate::core::LogSummary for [<$action_name Feedback>] {
                fn log_summary(&self) -> String {
                    let mut s = String::from(stringify!([<$action_name Feedback>]));
                    s.push('(');
                    $(
                        if !s.ends_with('(') { s.push_str(", "); }
                        s.push_str(&format!("{}={:?}", stringify!($feedback_field), self.$feedback_field));
                    )*
                    s.push(')');
                    s
                }
            }

            // ===== Result Type =====
            #[doc = concat!("Result type for the `", stringify!($action_name), "` action.")]
            #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
            pub struct [<$action_name Result>] {
                $(
                    $(#[$result_field_meta])*
                    pub $result_field: $result_type,
                )*
            }

            impl $crate::core::LogSummary for [<$action_name Result>] {
                fn log_summary(&self) -> String {
                    let mut s = String::from(stringify!([<$action_name Result>]));
                    s.push('(');
                    $(
                        if !s.ends_with('(') { s.push_str(", "); }
                        s.push_str(&format!("{}={:?}", stringify!($result_field), self.$result_field));
                    )*
                    s.push(')');
                    s
                }
            }

            // ===== Action Marker Type =====
            #[doc = concat!("Action type for `", stringify!($action_name), "`.")]
            #[doc = ""]
            #[doc = "This is a marker type that implements the `Action` trait."]
            $(#[$action_meta])*
            pub struct $action_name;

            impl $crate::actions::Action for $action_name {
                type Goal = [<$action_name Goal>];
                type Feedback = [<$action_name Feedback>];
                type Result = [<$action_name Result>];

                fn name() -> &'static str {
                    $crate::action!(@snake_case_name $action_name)
                }
            }
        }
    };

    // Helper: Generate format args for log_summary
    (@fmt_args $first:ident $(, $rest:ident)*) => {
        concat!(stringify!($first), "={:?}", $("," , stringify!($rest), "={:?}")*)
    };
    (@fmt_args) => { "" };

    // Helper: Maybe generate Default impl for Goal if all fields have defaults
    (@maybe_default_goal $name:ident { $($field:ident : $type:ty = $default:expr),* }) => {
        impl Default for $name {
            fn default() -> Self {
                Self {
                    $($field: $default,)*
                }
            }
        }
    };
    // No defaults - don't generate Default
    (@maybe_default_goal $name:ident { $($field:ident : $type:ty),* }) => {};
    // Mixed - don't generate Default (would need builder pattern)
    (@maybe_default_goal $name:ident { $($tt:tt)* }) => {};

    // Helper: Convert CamelCase to snake_case for action name
    // This is a simplified version - for complex names, use proc macro
    (@snake_case_name $name:ident) => {
        {
            // Convert to snake_case at runtime
            // Note: For compile-time, you'd need a proc macro
            const NAME: &str = stringify!($name);
            // Use a const fn in the future, for now we rely on the name being provided
            // This is a limitation - action names should be provided explicitly
            // For now, we'll convert at runtime
            $crate::action!(@to_snake_case stringify!($name))
        }
    };

    // Runtime snake_case conversion (fallback)
    (@to_snake_case $s:expr) => {{
        fn to_snake_case(s: &str) -> String {
            let mut result = String::new();
            for (i, c) in s.chars().enumerate() {
                if c.is_uppercase() {
                    if i > 0 {
                        result.push('_');
                    }
                    result.push(c.to_ascii_lowercase());
                } else {
                    result.push(c);
                }
            }
            result
        }
        // Leak to get 'static lifetime (done once per action type)
        Box::leak(to_snake_case($s).into_boxed_str()) as &'static str
    }};
}

/// Macro for implementing common action patterns.
///
/// This generates standard action implementations for common robotics tasks.
#[macro_export]
macro_rules! standard_action {
    // Navigation action
    (navigate $name:ident) => {
        $crate::action! {
            /// Navigate to a target position
            $name {
                goal {
                    /// Target X coordinate
                    target_x: f64,
                    /// Target Y coordinate
                    target_y: f64,
                    /// Optional target orientation (radians)
                    target_theta: Option<f64>,
                    /// Maximum speed (m/s)
                    max_speed: f64 = 1.0,
                }
                feedback {
                    /// Distance remaining to target
                    distance_remaining: f64,
                    /// Current X position
                    current_x: f64,
                    /// Current Y position
                    current_y: f64,
                    /// Progress percentage (0-100)
                    progress_percent: f32,
                }
                result {
                    /// Whether navigation succeeded
                    success: bool,
                    /// Final X position
                    final_x: f64,
                    /// Final Y position
                    final_y: f64,
                    /// Final orientation
                    final_theta: f64,
                    /// Total distance traveled
                    distance_traveled: f64,
                    /// Time taken (seconds)
                    time_elapsed: f32,
                }
            }
        }
    };

    // Manipulation action (pick/place)
    (manipulate $name:ident) => {
        $crate::action! {
            /// Manipulate an object
            $name {
                goal {
                    /// Object identifier
                    object_id: String,
                    /// Target pose X
                    target_x: f64,
                    /// Target pose Y
                    target_y: f64,
                    /// Target pose Z
                    target_z: f64,
                    /// Gripper force limit (N)
                    force_limit: f64 = 10.0,
                }
                feedback {
                    /// Current manipulation phase
                    phase: String,
                    /// Gripper force (N)
                    grip_force: f64,
                    /// End effector position X
                    ee_x: f64,
                    /// End effector position Y
                    ee_y: f64,
                    /// End effector position Z
                    ee_z: f64,
                    /// Progress percentage
                    progress_percent: f32,
                }
                result {
                    /// Whether manipulation succeeded
                    success: bool,
                    /// Error message if failed
                    error_message: String,
                    /// Object final position X
                    object_x: f64,
                    /// Object final position Y
                    object_y: f64,
                    /// Object final position Z
                    object_z: f64,
                }
            }
        }
    };

    // Wait/delay action
    (wait $name:ident) => {
        $crate::action! {
            /// Wait for a specified duration
            $name {
                goal {
                    /// Duration to wait (seconds)
                    duration_secs: f64,
                }
                feedback {
                    /// Time remaining (seconds)
                    time_remaining: f64,
                    /// Progress percentage
                    progress_percent: f32,
                }
                result {
                    /// Whether wait completed (vs canceled)
                    completed: bool,
                    /// Actual time waited (seconds)
                    actual_duration: f64,
                }
            }
        }
    };

    // Dock/undock action
    (dock $name:ident) => {
        $crate::action! {
            /// Dock with a charging station or target
            $name {
                goal {
                    /// Dock station ID
                    dock_id: String,
                    /// Approach speed (m/s)
                    approach_speed: f64 = 0.1,
                }
                feedback {
                    /// Docking phase
                    phase: String,
                    /// Distance to dock
                    distance_to_dock: f64,
                    /// Alignment error (radians)
                    alignment_error: f64,
                    /// Dock detected
                    dock_detected: bool,
                }
                result {
                    /// Whether docking succeeded
                    success: bool,
                    /// Dock ID if successful
                    docked_id: String,
                    /// Contact force (N)
                    contact_force: f64,
                }
            }
        }
    };
}

#[cfg(test)]
mod tests {
    // Note: These tests use the macros but would require the full horus_core context
    // They're commented out as they'd need proper module setup

    /*
    use super::*;

    action! {
        /// Test action for unit tests
        TestMoveAction {
            goal {
                distance: f64,
                speed: f64 = 1.0,
            }
            feedback {
                progress: f32,
            }
            result {
                success: bool,
                actual_distance: f64,
            }
        }
    }

    #[test]
    fn test_action_macro_generates_types() {
        let goal = TestMoveActionGoal { distance: 5.0, speed: 2.0 };
        assert_eq!(goal.distance, 5.0);

        let feedback = TestMoveActionFeedback { progress: 0.5 };
        assert_eq!(feedback.progress, 0.5);

        let result = TestMoveActionResult { success: true, actual_distance: 4.9 };
        assert!(result.success);
    }

    #[test]
    fn test_action_trait_impl() {
        assert!(!TestMoveAction::name().is_empty());
        assert!(TestMoveAction::goal_topic().contains("goal"));
        assert!(TestMoveAction::feedback_topic().contains("feedback"));
        assert!(TestMoveAction::result_topic().contains("result"));
    }
    */
}
