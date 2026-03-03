//! Managed node lifecycle — ROS2 lifecycle node parity.
//!
//! Provides the [`LifecycleNode`] trait and [`LifecycleState`] enum so HORUS
//! nodes can follow the same managed lifecycle as `rclcpp::LifecycleNode`.
//!
//! # Overview
//!
//! The ROS2 managed lifecycle defines a state machine:
//!
//! ```text
//!                       configure()
//!  Unconfigured ───────────────────► Inactive ◄──────────────────┐
//!       ▲                                │                       │
//!       │       cleanup()               │ activate()  deactivate()│
//!       │  ◄─────────────────────────── │ ──────────────────────► │
//!       │                               ▼                         │
//!       │                             Active                      │
//!       │                                                         │
//!   Finalized                                                     │
//!       ▲                                                         │
//!       │            on_error()/shutdown()                        │
//!       └─────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus::prelude::*;
//! use horus_core::core::lifecycle::{LifecycleNode, LifecycleState};
//!
//! pub struct CameraNode {
//!     camera: Option<Camera>,
//!     publisher: Option<Topic<Image>>,
//!     state: LifecycleState,
//! }
//!
//! impl Node for CameraNode {
//!     fn name(&self) -> &str { "camera_node" }
//!     fn tick(&mut self) {
//!         if self.state != LifecycleState::Active { return; }
//!         // publish frame...
//!     }
//! }
//!
//! impl LifecycleNode for CameraNode {
//!     fn on_configure(&mut self) -> HorusResult<()> {
//!         self.publisher = Some(Topic::new("camera/image")?);
//!         Ok(())
//!     }
//!
//!     fn on_activate(&mut self) -> HorusResult<()> {
//!         self.camera = Some(Camera::open()?);
//!         self.state = LifecycleState::Active;
//!         Ok(())
//!     }
//!
//!     fn on_deactivate(&mut self) -> HorusResult<()> {
//!         self.state = LifecycleState::Inactive;
//!         Ok(())
//!     }
//!
//!     fn on_cleanup(&mut self) -> HorusResult<()> {
//!         self.camera = None;
//!         self.publisher = None;
//!         Ok(())
//!     }
//!
//!     fn lifecycle_state(&self) -> LifecycleState {
//!         self.state.clone()
//!     }
//! }
//! ```
//!
//! # Lifecycle Commands
//!
//! Lifecycle transitions can be triggered externally via the HORUS CLI or via a
//! control topic.  The node should subscribe to
//! `_horus/lifecycle/{node_name}/cmd` to receive [`LifecycleCommand`] messages.
//!
//! # ROS2 Equivalents
//!
//! | ROS2 | HORUS |
//! |------|-------|
//! | `rclcpp::LifecycleNode` | `impl LifecycleNode for MyNode` |
//! | `on_configure` | `fn on_configure(&mut self)` |
//! | `on_activate` | `fn on_activate(&mut self)` |
//! | `on_deactivate` | `fn on_deactivate(&mut self)` |
//! | `on_cleanup` | `fn on_cleanup(&mut self)` |
//! | `ros2 lifecycle set /node configure` | `horus node configure <name>` |
//! | `ros2 lifecycle get /node` | `horus node lifecycle <name>` |

use serde::{Deserialize, Serialize};

use crate::core::Node;
use crate::error::HorusResult;

// ─── LifecycleState ───────────────────────────────────────────────────────────

/// Managed node lifecycle state.
///
/// Mirrors the ROS2 managed node lifecycle defined in REP-2015.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleState {
    /// The node has been created but not yet configured.
    ///
    /// Topics, parameters, and resources are NOT allocated.
    Unconfigured,

    /// The node is configured and resources are allocated, but it is not
    /// actively processing data.
    ///
    /// Topics exist but nothing is being published/subscribed.
    Inactive,

    /// The node is fully operational and processing data.
    ///
    /// This is the normal running state — topics are active.
    Active,

    /// The node is being shut down.
    ///
    /// Resources are being released.  This state is transient.
    ShuttingDown,

    /// The node has been fully shut down and its resources released.
    Finalized,

    /// An error occurred during a lifecycle transition.
    ///
    /// The node requires intervention: either `cleanup()` to return to
    /// `Unconfigured`, or `destroy()` to finalize.
    ErrorProcessing(String),
}

impl LifecycleState {
    /// Whether this node is actively processing data.
    #[inline]
    pub fn is_active(&self) -> bool {
        matches!(self, Self::Active)
    }

    /// Whether the node is usable (not finalized or in error).
    #[inline]
    pub fn is_alive(&self) -> bool {
        !matches!(self, Self::Finalized | Self::ErrorProcessing(_))
    }

    /// Human-readable label.
    pub fn label(&self) -> &str {
        match self {
            Self::Unconfigured => "unconfigured",
            Self::Inactive => "inactive",
            Self::Active => "active",
            Self::ShuttingDown => "shutting_down",
            Self::Finalized => "finalized",
            Self::ErrorProcessing(_) => "error_processing",
        }
    }
}

impl std::fmt::Display for LifecycleState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ErrorProcessing(msg) => write!(f, "error_processing({})", msg),
            other => write!(f, "{}", other.label()),
        }
    }
}

impl Default for LifecycleState {
    fn default() -> Self {
        Self::Unconfigured
    }
}

// ─── LifecycleCommand ────────────────────────────────────────────────────────

/// External lifecycle command sent via the control topic.
///
/// Published on `_horus/lifecycle/{node_name}/cmd`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LifecycleCommand {
    Configure,
    Activate,
    Deactivate,
    Cleanup,
    Shutdown,
}

impl std::fmt::Display for LifecycleCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Configure => write!(f, "configure"),
            Self::Activate => write!(f, "activate"),
            Self::Deactivate => write!(f, "deactivate"),
            Self::Cleanup => write!(f, "cleanup"),
            Self::Shutdown => write!(f, "shutdown"),
        }
    }
}

// ─── LifecycleTransition result ───────────────────────────────────────────────

/// Result of a lifecycle transition, published on
/// `_horus/lifecycle/{node_name}/state`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LifecycleTransitionResult {
    /// The command that triggered this transition.
    pub command: String,
    /// New lifecycle state after the transition.
    pub new_state: LifecycleState,
    /// Whether the transition succeeded.
    pub success: bool,
    /// Error message if `success == false`.
    pub error: Option<String>,
}

// ─── LifecycleNode trait ──────────────────────────────────────────────────────

/// Extension trait for nodes that support the ROS2 managed lifecycle.
///
/// Implementing this trait gives a node explicit configure/activate/deactivate/
/// cleanup transitions.  Default implementations do nothing (backward
/// compatible).
///
/// # Integration with the scheduler
///
/// When a node implementing `LifecycleNode` is registered with the scheduler:
/// 1. The scheduler calls `init()` (inherited from `Node`).
/// 2. If `init()` succeeds, the node starts in `Inactive` state — `tick()` is
///    **not** called until `on_activate()` is invoked.
/// 3. External agents (CLI, another node) send a `Configure` command to the
///    node's lifecycle control topic to trigger `on_configure()`, and an
///    `Activate` command to trigger `on_activate()`.
///
/// # Embedding in tick()
///
/// The simplest approach is to check `lifecycle_state()` at the top of `tick()`:
///
/// ```rust,ignore
/// fn tick(&mut self) {
///     if !self.lifecycle.is_active() { return; }
///     // Normal processing...
/// }
/// ```
pub trait LifecycleNode: Node {
    /// Called during the `Unconfigured → Inactive` transition.
    ///
    /// Allocate topics, load parameters, initialize resources.
    /// Return `Err` to abort the transition and enter `ErrorProcessing`.
    fn on_configure(&mut self) -> HorusResult<()> {
        Ok(())
    }

    /// Called during the `Inactive → Active` transition.
    ///
    /// Start processing: enable topic publishing/subscribing, open hardware
    /// connections.  Return `Err` to abort and enter `ErrorProcessing`.
    fn on_activate(&mut self) -> HorusResult<()> {
        Ok(())
    }

    /// Called during the `Active → Inactive` transition.
    ///
    /// Stop processing but keep resources allocated.
    fn on_deactivate(&mut self) -> HorusResult<()> {
        Ok(())
    }

    /// Called during the `Inactive → Unconfigured` transition.
    ///
    /// Release all resources (topics, hardware connections, allocated memory).
    fn on_cleanup(&mut self) -> HorusResult<()> {
        Ok(())
    }

    /// Called when entering `ErrorProcessing`.
    ///
    /// Attempt to recover.  Returns the next state to transition to:
    /// `Inactive` (recovered) or `Finalized` (unrecoverable, default).
    fn on_error(&mut self, error: &str) -> LifecycleState {
        crate::hlog!(
            error,
            "Lifecycle error (entering error_processing): {}",
            error
        );
        LifecycleState::Finalized
    }

    /// Current lifecycle state.
    ///
    /// Override to expose your node's internal state tracker.  Used by the
    /// monitor, CLI, and introspection tools.
    fn lifecycle_state(&self) -> LifecycleState {
        // Default implementation assumes nodes start as Active (backward compat)
        LifecycleState::Active
    }

    /// Topic name for receiving lifecycle commands.
    ///
    /// Override to customise.  Default: `_horus/lifecycle/{name}/cmd`
    fn lifecycle_command_topic(&self) -> String {
        format!("_horus/lifecycle/{}/cmd", self.name())
    }

    /// Topic name for publishing lifecycle state changes.
    ///
    /// Default: `_horus/lifecycle/{name}/state`
    fn lifecycle_state_topic(&self) -> String {
        format!("_horus/lifecycle/{}/state", self.name())
    }

    /// Apply a lifecycle command, calling the appropriate transition method.
    ///
    /// Returns the new state.  Override if you want custom transition logic.
    fn apply_command(&mut self, cmd: &LifecycleCommand) -> LifecycleTransitionResult {
        let result = match cmd {
            LifecycleCommand::Configure => self.on_configure(),
            LifecycleCommand::Activate => self.on_activate(),
            LifecycleCommand::Deactivate => self.on_deactivate(),
            LifecycleCommand::Cleanup => self.on_cleanup(),
            LifecycleCommand::Shutdown => self.on_cleanup(),
        };

        match result {
            Ok(()) => LifecycleTransitionResult {
                command: cmd.to_string(),
                new_state: self.lifecycle_state(),
                success: true,
                error: None,
            },
            Err(e) => {
                let err_msg = e.to_string();
                let next_state = LifecycleNode::on_error(self, &err_msg);
                LifecycleTransitionResult {
                    command: cmd.to_string(),
                    new_state: next_state,
                    success: false,
                    error: Some(err_msg),
                }
            }
        }
    }
}

// ─── LifecycleManager ────────────────────────────────────────────────────────

/// Helper struct for managing lifecycle state in a node implementation.
///
/// Store this as a field in your node and delegate `lifecycle_state()` to it:
///
/// ```rust,ignore
/// pub struct MyNode {
///     lifecycle: LifecycleManager,
/// }
///
/// impl LifecycleNode for MyNode {
///     fn lifecycle_state(&self) -> LifecycleState {
///         self.lifecycle.state().clone()
///     }
/// }
/// ```
#[derive(Debug, Default)]
pub struct LifecycleManager {
    state: LifecycleState,
}

impl LifecycleManager {
    /// Create a manager in the `Unconfigured` state.
    pub fn new() -> Self {
        Self {
            state: LifecycleState::Unconfigured,
        }
    }

    /// Create a manager that starts in `Active` state (for legacy nodes).
    pub fn always_active() -> Self {
        Self {
            state: LifecycleState::Active,
        }
    }

    /// Current state.
    #[inline]
    pub fn state(&self) -> &LifecycleState {
        &self.state
    }

    /// Whether the node is in the `Active` state.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.state.is_active()
    }

    /// Transition to a new state.
    ///
    /// Validates that the transition is legal; returns `true` on success.
    pub fn transition_to(&mut self, new_state: LifecycleState) -> bool {
        let valid = matches!(
            (&self.state, &new_state),
            // Legal transitions
            (LifecycleState::Unconfigured, LifecycleState::Inactive) // configure
            | (LifecycleState::Inactive, LifecycleState::Active)      // activate
            | (LifecycleState::Active, LifecycleState::Inactive)      // deactivate
            | (LifecycleState::Inactive, LifecycleState::Unconfigured) // cleanup
            | (LifecycleState::Active, LifecycleState::ShuttingDown)  // shutdown
            | (LifecycleState::Inactive, LifecycleState::ShuttingDown)
            | (LifecycleState::Unconfigured, LifecycleState::ShuttingDown)
            | (LifecycleState::ShuttingDown, LifecycleState::Finalized)
            | (_, LifecycleState::ErrorProcessing(_))                 // errors from anywhere
            | (LifecycleState::ErrorProcessing(_), LifecycleState::Unconfigured) // recovery
            | (LifecycleState::ErrorProcessing(_), LifecycleState::Finalized)
        );
        if valid {
            self.state = new_state;
        }
        valid
    }

    /// Force-set the state regardless of validity (use with care).
    pub fn set_state(&mut self, state: LifecycleState) {
        self.state = state;
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lifecycle_state_display() {
        assert_eq!(LifecycleState::Active.label(), "active");
        assert_eq!(LifecycleState::Unconfigured.label(), "unconfigured");
        assert_eq!(LifecycleState::Inactive.label(), "inactive");
    }

    #[test]
    fn test_lifecycle_manager_transitions() {
        let mut mgr = LifecycleManager::new();
        assert_eq!(mgr.state(), &LifecycleState::Unconfigured);

        assert!(mgr.transition_to(LifecycleState::Inactive)); // configure
        assert!(mgr.transition_to(LifecycleState::Active));   // activate
        assert!(mgr.transition_to(LifecycleState::Inactive)); // deactivate
        assert!(mgr.transition_to(LifecycleState::Unconfigured)); // cleanup
    }

    #[test]
    fn test_invalid_transition() {
        let mut mgr = LifecycleManager::new();
        // Can't activate from Unconfigured — must configure first
        assert!(!mgr.transition_to(LifecycleState::Active));
        assert_eq!(mgr.state(), &LifecycleState::Unconfigured);
    }

    #[test]
    fn test_is_active() {
        let mgr = LifecycleManager::always_active();
        assert!(mgr.is_active());

        let mgr2 = LifecycleManager::new();
        assert!(!mgr2.is_active());
    }
}
