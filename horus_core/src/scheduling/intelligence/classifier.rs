use super::profiler::{NodeStats, RuntimeProfiler};
use std::collections::HashMap;

/// Execution tier for a node based on characteristics
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExecutionTier {
    /// Ultra-fast deterministic nodes (<5μs) - inline execution
    UltraFast,
    /// Fast nodes (<1ms) - Inline execution
    Fast,
    /// I/O heavy nodes - Async/await
    AsyncIO,
    /// High failure rate - Process isolation
    Isolated,
    /// Everything else - Standard execution
    Background,
}

impl ExecutionTier {
    /// Get human-readable name
    pub fn name(&self) -> &'static str {
        match self {
            ExecutionTier::UltraFast => "UltraFast (Inline)",
            ExecutionTier::Fast => "Fast (Inline)",
            ExecutionTier::AsyncIO => "Async I/O",
            ExecutionTier::Isolated => "Isolated",
            ExecutionTier::Background => "Background",
        }
    }

    /// Get expected latency range
    pub fn latency_range(&self) -> &'static str {
        match self {
            ExecutionTier::UltraFast => "20-50ns",
            ExecutionTier::Fast => "50-100ns",
            ExecutionTier::AsyncIO => "10-100μs",
            ExecutionTier::Isolated => "1-10ms",
            ExecutionTier::Background => "1-100ms",
        }
    }
}

/// Classifier that assigns nodes to execution tiers
#[derive(Debug, Clone)]
pub struct TierClassifier {
    /// Tier assignments for each node
    pub assignments: HashMap<String, ExecutionTier>,
}

impl TierClassifier {
    /// Create new classifier and assign tiers based on profiler data
    pub fn from_profiler(profiler: &RuntimeProfiler) -> Self {
        let mut assignments = HashMap::new();

        for (node_name, stats) in &profiler.node_stats {
            let tier = Self::classify_node(stats);
            assignments.insert(node_name.clone(), tier);
        }

        Self { assignments }
    }

    /// Classify a single node based on its statistics
    fn classify_node(stats: &NodeStats) -> ExecutionTier {
        // Priority 1: High-failure nodes → Isolated tier (for process isolation)
        if stats.has_high_failure_rate() {
            return ExecutionTier::Isolated;
        }

        // Priority 2: Ultra-fast deterministic nodes → UltraFast tier
        if stats.avg_us < 5.0 && stats.is_deterministic {
            return ExecutionTier::UltraFast;
        }

        // Priority 3: I/O heavy nodes → Async tier
        if stats.is_io_heavy {
            return ExecutionTier::AsyncIO;
        }

        // Priority 4: Fast nodes (<1ms) → Inline tier
        if stats.avg_us < 1000.0 {
            return ExecutionTier::Fast;
        }

        // Priority 5: Default to background
        ExecutionTier::Background
    }

    /// Get tier for a specific node
    pub fn get_tier(&self, node_name: &str) -> Option<ExecutionTier> {
        self.assignments.get(node_name).copied()
    }

    /// Get all nodes in a specific tier
    pub fn get_nodes_in_tier(&self, tier: ExecutionTier) -> Vec<String> {
        self.assignments
            .iter()
            .filter(|(_, &t)| t == tier)
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// Print classification results
    pub fn print_classification(&self) {
        println!("\n=== Execution Tier Classification ===");

        let total_nodes = self.assignments.len();
        let mut ultra_fast = 0usize;
        let mut fast = 0usize;
        let mut async_io = 0usize;
        let mut isolated = 0usize;
        let mut background = 0usize;

        for tier in self.assignments.values() {
            match tier {
                ExecutionTier::UltraFast => ultra_fast += 1,
                ExecutionTier::Fast => fast += 1,
                ExecutionTier::AsyncIO => async_io += 1,
                ExecutionTier::Isolated => isolated += 1,
                ExecutionTier::Background => background += 1,
            }
        }

        println!("Total Nodes: {}", total_nodes);
        println!("\nTier Distribution:");
        println!(
            "{:<20} {:>8} {:>8} {:>15}",
            "Tier", "Count", "Percent", "Latency"
        );
        println!("{}", "-".repeat(60));

        let tiers = [
            (ExecutionTier::UltraFast, ultra_fast),
            (ExecutionTier::Fast, fast),
            (ExecutionTier::AsyncIO, async_io),
            (ExecutionTier::Isolated, isolated),
            (ExecutionTier::Background, background),
        ];

        for (tier, count) in tiers {
            let percent = if total_nodes > 0 {
                (count as f64 / total_nodes as f64) * 100.0
            } else {
                0.0
            };

            println!(
                "{:<20} {:>8} {:>7.1}% {:>15}",
                tier.name(),
                count,
                percent,
                tier.latency_range()
            );
        }

        // Print node assignments by tier
        println!("\n=== Nodes by Tier ===");
        for tier in &[
            ExecutionTier::UltraFast,
            ExecutionTier::Fast,
            ExecutionTier::AsyncIO,
            ExecutionTier::Isolated,
            ExecutionTier::Background,
        ] {
            let nodes = self.get_nodes_in_tier(*tier);
            if !nodes.is_empty() {
                println!("\n{}:", tier.name());
                for node in nodes {
                    println!("  - {}", node);
                }
            }
        }
        println!();
    }
}
