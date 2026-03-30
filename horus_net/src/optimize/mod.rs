//! Optimizer layer — fusion, delta, spatial, predict.
//!
//! All optimizers OFF by default. Immediate priority ALWAYS bypasses.
//! See blueprint section 12.

pub mod delta;
pub mod fusion;
pub mod predict;
pub mod spatial;

use crate::priority::Priority;
use crate::wire::{InMessage, OutMessage};

/// Optimizer trait — pluggable bandwidth optimization stage.
///
/// Implementations process messages before they hit the wire (outgoing)
/// or after they arrive (incoming). Immediate priority messages ALWAYS
/// bypass the entire chain (hardcoded in OptimizerChain).
pub trait Optimizer: Send {
    /// Process outgoing messages before they hit the wire.
    /// May batch, filter, compress, or drop messages.
    fn on_outgoing(&mut self, messages: &mut Vec<OutMessage>);

    /// Process incoming messages after they arrive from the wire.
    /// May expand deltas, reconstruct predictions, etc.
    fn on_incoming(&mut self, messages: &mut Vec<InMessage>);

    /// Name for logging/diagnostics.
    fn name(&self) -> &str;

    /// Priority threshold: optimizer only runs on messages at or below this priority.
    /// Default: Normal. Immediate messages ALWAYS bypass all optimizers.
    fn max_priority(&self) -> Priority {
        Priority::Normal
    }
}

/// Chain of optimizers applied in order.
pub struct OptimizerChain {
    optimizers: Vec<Box<dyn Optimizer>>,
}

impl OptimizerChain {
    /// Create an empty chain (zero overhead — passthrough).
    pub fn new() -> Self {
        Self {
            optimizers: Vec::new(),
        }
    }

    /// Build a chain from config optimizer names.
    pub fn from_config(names: &[String]) -> Self {
        let mut chain = Self::new();
        for name in names {
            match name.as_str() {
                "fusion" => chain.add(Box::new(fusion::FusionOptimizer::new())),
                "delta" => chain.add(Box::new(delta::DeltaOptimizer::new())),
                "spatial" => chain.add(Box::new(spatial::SpatialOptimizer::new())),
                "predict" => chain.add(Box::new(predict::PredictOptimizer::new())),
                other => eprintln!("[horus_net] Unknown optimizer: {other}"),
            }
        }
        chain
    }

    /// Add an optimizer to the chain.
    pub fn add(&mut self, optimizer: Box<dyn Optimizer>) {
        self.optimizers.push(optimizer);
    }

    /// Process outgoing messages through the chain.
    ///
    /// **Immediate priority messages are extracted before the chain and
    /// re-appended after** — they NEVER touch any optimizer.
    pub fn process_outgoing(&mut self, messages: &mut Vec<OutMessage>) {
        if self.optimizers.is_empty() {
            return; // Zero overhead
        }

        // Extract Immediate messages — bypass all optimizers
        let mut immediate: Vec<OutMessage> = Vec::new();
        let mut rest: Vec<OutMessage> = Vec::new();
        for msg in messages.drain(..) {
            if msg.priority == Priority::Immediate {
                immediate.push(msg);
            } else {
                rest.push(msg);
            }
        }

        // Run chain on non-Immediate messages
        for opt in &mut self.optimizers {
            opt.on_outgoing(&mut rest);
        }

        // Reassemble: Immediate first (sent immediately), then rest
        messages.extend(immediate);
        messages.extend(rest);
    }

    /// Process incoming messages through the chain (reverse order).
    pub fn process_incoming(&mut self, messages: &mut Vec<InMessage>) {
        if self.optimizers.is_empty() {
            return;
        }

        // Incoming: run in reverse order (last encoder = first decoder)
        for opt in self.optimizers.iter_mut().rev() {
            opt.on_incoming(messages);
        }
    }

    /// Number of optimizers in the chain.
    pub fn len(&self) -> usize {
        self.optimizers.len()
    }

    /// Whether the chain is empty (zero overhead passthrough).
    pub fn is_empty(&self) -> bool {
        self.optimizers.is_empty()
    }
}

impl Default for OptimizerChain {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priority::{Encoding, Reliability};
    use crate::wire::topic_hash;

    fn make_msg(name: &str, priority: Priority) -> OutMessage {
        OutMessage {
            topic_name: name.into(),
            topic_hash: topic_hash(name),
            payload: vec![1, 2, 3],
            timestamp_ns: 0,
            sequence: 1,
            priority,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        }
    }

    /// Test optimizer that counts calls.
    struct CountingOptimizer {
        outgoing_calls: std::cell::Cell<u32>,
        incoming_calls: std::cell::Cell<u32>,
    }

    impl CountingOptimizer {
        fn new() -> Self {
            Self {
                outgoing_calls: std::cell::Cell::new(0),
                incoming_calls: std::cell::Cell::new(0),
            }
        }
    }

    impl Optimizer for CountingOptimizer {
        fn on_outgoing(&mut self, _messages: &mut Vec<OutMessage>) {
            self.outgoing_calls.set(self.outgoing_calls.get() + 1);
        }
        fn on_incoming(&mut self, _messages: &mut Vec<InMessage>) {
            self.incoming_calls.set(self.incoming_calls.get() + 1);
        }
        fn name(&self) -> &str {
            "counting"
        }
    }

    #[test]
    fn empty_chain_zero_overhead() {
        let mut chain = OptimizerChain::new();
        assert!(chain.is_empty());

        let mut msgs = vec![make_msg("imu", Priority::Normal)];
        chain.process_outgoing(&mut msgs);
        assert_eq!(msgs.len(), 1); // Unchanged
    }

    #[test]
    fn immediate_bypasses_chain() {
        let mut chain = OptimizerChain::new();
        let counter = CountingOptimizer::new();
        chain.add(Box::new(counter));

        let mut msgs = vec![
            make_msg("estop", Priority::Immediate),
            make_msg("imu", Priority::Normal),
        ];
        chain.process_outgoing(&mut msgs);

        // Both messages present
        assert_eq!(msgs.len(), 2);
        // Immediate message is first (sent first)
        assert_eq!(msgs[0].priority, Priority::Immediate);
        assert_eq!(msgs[1].priority, Priority::Normal);
    }

    #[test]
    fn chain_calls_optimizers_in_order() {
        let mut chain = OptimizerChain::new();
        chain.add(Box::new(CountingOptimizer::new()));
        chain.add(Box::new(CountingOptimizer::new()));

        let mut msgs = vec![make_msg("imu", Priority::Normal)];
        chain.process_outgoing(&mut msgs);
        assert_eq!(chain.len(), 2);
    }

    #[test]
    fn from_config_builds_chain() {
        let chain = OptimizerChain::from_config(&["fusion".into(), "delta".into()]);
        assert_eq!(chain.len(), 2);
    }

    #[test]
    fn from_config_ignores_unknown() {
        let chain = OptimizerChain::from_config(&["fusion".into(), "bogus".into()]);
        assert_eq!(chain.len(), 1); // Only fusion
    }

    #[test]
    fn all_immediate_bypass() {
        let mut chain = OptimizerChain::new();
        chain.add(Box::new(CountingOptimizer::new()));

        let mut msgs = vec![
            make_msg("estop1", Priority::Immediate),
            make_msg("estop2", Priority::Immediate),
        ];
        chain.process_outgoing(&mut msgs);
        assert_eq!(msgs.len(), 2);
        // Both are Immediate — optimizer received empty vec
    }
}
