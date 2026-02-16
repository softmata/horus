//! Node tier annotation for scheduling priority hints
//!
//! Provides the `NodeTier` enum for developers to declare a node's
//! execution characteristics at compile time.

use serde::{Deserialize, Serialize};

/// Node tier for explicit annotation by developers
///
/// Use this to declare a node's execution characteristics at compile time.
/// The tier influences the default failure policy applied to the node.
///
/// # Example
/// ```ignore
/// use horus_core::scheduling::{NodeTier, Scheduler};
///
/// let mut scheduler = Scheduler::new();
/// scheduler.add(pid_node).order(0).tier(NodeTier::UltraFast).done();
/// scheduler.add(sensor_node).order(1).tier(NodeTier::Fast).done();
/// scheduler.add(logger_node).order(5).tier(NodeTier::Normal).done();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum NodeTier {
    /// Ultra-fast nodes (<1us) - highest priority inline execution
    /// Use for: PID controllers, simple math, data transformations
    UltraFast,

    /// Fast nodes (<10us) - high priority inline execution
    /// Use for: Sensor readers, filter calculations, state machines
    #[default]
    Fast,

    /// Normal nodes (<100us+) - standard scheduling
    /// Use for: Complex algorithms, data processing, logging, diagnostics
    Normal,
}

impl NodeTier {
    /// Get human-readable description
    pub fn description(&self) -> &'static str {
        match self {
            NodeTier::UltraFast => "Ultra-fast inline (<1us)",
            NodeTier::Fast => "Inline execution (<10us)",
            NodeTier::Normal => "Standard scheduling (<100us+)",
        }
    }

    /// Get the default failure policy for this tier.
    ///
    /// Each tier has a sensible default that matches its criticality:
    /// - **UltraFast/Fast**: `Fatal` — these are control-loop nodes; failure = stop.
    /// - **Normal**: `Restart` with exponential backoff (5 attempts, 100ms initial).
    pub fn default_failure_policy(
        &self,
    ) -> super::super::fault_tolerance::failure_policy::FailurePolicy {
        use super::super::fault_tolerance::failure_policy::FailurePolicy;
        match self {
            NodeTier::UltraFast | NodeTier::Fast => FailurePolicy::Fatal,
            NodeTier::Normal => FailurePolicy::restart(5, 100),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_tier_default() {
        assert_eq!(NodeTier::default(), NodeTier::Fast);
    }

    #[test]
    fn test_node_tier_descriptions() {
        assert!(!NodeTier::UltraFast.description().is_empty());
        assert!(!NodeTier::Fast.description().is_empty());
        assert!(!NodeTier::Normal.description().is_empty());
    }
}
