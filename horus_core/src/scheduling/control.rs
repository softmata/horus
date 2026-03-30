//! Control commands for runtime scheduler management.
//!
//! The scheduler subscribes to a well-known control topic
//! `horus.ctl.{scheduler_name}` and processes commands at the end of
//! each tick (deterministic — no mid-tick state changes).
//!
//! CLI tools publish commands to this topic to manage running nodes.

use serde::{Deserialize, Serialize};

/// Commands that can be sent to a running scheduler via the control topic.
///
/// Processed at tick boundary for deterministic behavior.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ControlCommand {
    /// Pause a node — scheduler skips its tick() calls.
    PauseNode {
        /// Name of the node to pause.
        name: String,
    },
    /// Resume a paused node — scheduler resumes tick() calls.
    ResumeNode {
        /// Name of the node to resume.
        name: String,
    },
    /// Change a node's tick rate at runtime.
    SetNodeRate {
        /// Name of the node.
        name: String,
        /// New rate in Hz.
        rate_hz: f64,
    },
    /// Request graceful shutdown of the entire scheduler.
    GracefulShutdown,
    /// Force a metrics flush to the SHM registry (useful for debugging).
    RequestMetricsFlush,
}

impl ControlCommand {
    /// Human-readable description for logging.
    pub fn description(&self) -> String {
        match self {
            Self::PauseNode { name } => format!("Pause node '{}'", name),
            Self::ResumeNode { name } => format!("Resume node '{}'", name),
            Self::SetNodeRate { name, rate_hz } => {
                format!("Set node '{}' rate to {} Hz", name, rate_hz)
            }
            Self::GracefulShutdown => "Graceful shutdown".to_string(),
            Self::RequestMetricsFlush => "Flush metrics".to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_command_serde_roundtrip() {
        let commands = vec![
            ControlCommand::PauseNode {
                name: "motor".to_string(),
            },
            ControlCommand::ResumeNode {
                name: "sensor".to_string(),
            },
            ControlCommand::SetNodeRate {
                name: "planner".to_string(),
                rate_hz: 50.0,
            },
            ControlCommand::GracefulShutdown,
            ControlCommand::RequestMetricsFlush,
        ];

        for cmd in &commands {
            let json = serde_json::to_string(cmd).unwrap();
            let deserialized: ControlCommand = serde_json::from_str(&json).unwrap();
            assert_eq!(cmd.description(), deserialized.description());
        }
    }

    #[test]
    fn control_command_descriptions() {
        assert!(ControlCommand::PauseNode {
            name: "x".to_string()
        }
        .description()
        .contains("Pause"));
        assert!(ControlCommand::GracefulShutdown
            .description()
            .contains("shutdown"));
    }
}
