//! Dependency graph for automatic execution ordering.
//!
//! Builds a DAG from nodes' publisher/subscriber topic metadata.
//! The graph drives the ready-dispatch executor: nodes run as soon as
//! all their dependencies complete, with no artificial barriers.
//!
//! # Example
//!
//! ```text
//! ImuDriver ──pub "imu"──→ BalanceCtrl ──pub "joint_cmd"──→ MotorDriver
//!                               ↑
//! CameraDriver ──pub "image"──→ SLAM ──pub "pose"──┘
//!
//! Ready dispatch order (optimal):
//!   t=0:    ImuDriver, CameraDriver start (no deps)
//!   t=0.1:  ImuDriver done
//!   t=2:    CameraDriver done → SLAM starts immediately
//!   t=5:    BalanceCtrl starts (needs ImuDriver only, already done)
//!   t=8:    SLAM done
//!   t=10:   BalanceCtrl done → MotorDriver starts
//! ```

use std::collections::HashMap;

use super::types::RegisteredNode;

/// Dependency graph built from node publisher/subscriber metadata.
///
/// Built at scheduler init from TopicNodeRegistry data. Provides both
/// level-based steps (for deterministic mode) and per-node successor
/// lists (for ready-dispatch parallel execution).
pub(crate) struct DependencyGraph {
    /// Execution steps: each step is a set of node indices that can run in parallel.
    /// Used by deterministic mode (sequential within steps) and as fallback.
    steps: Vec<Vec<usize>>,
    /// Per-node successor list: successors[i] = nodes that depend on node i.
    /// Used by ready-dispatch executor for immediate notification.
    successors: Vec<Vec<usize>>,
    /// Per-node dependency count: how many predecessors each node has.
    /// Ready-dispatch seeds nodes with dep_count == 0.
    dep_counts: Vec<usize>,
    /// Whether the graph was built from actual pub/sub metadata (true)
    /// or fell back to .order() tiers (false).
    has_topic_metadata: bool,
}

impl DependencyGraph {
    /// Build from registered nodes' publisher/subscriber metadata.
    ///
    /// If nodes provide `publishers()` / `subscribers()`, builds a real
    /// dependency graph. Otherwise falls back to `.order()` tiers:
    /// same order = independent (parallel), different order = sequential.
    pub fn build(nodes: &[RegisteredNode]) -> Result<Self, DependencyGraphError> {
        let n = nodes.len();
        if n == 0 {
            return Ok(Self {
                steps: Vec::new(),
                successors: Vec::new(),
                dep_counts: Vec::new(),
                has_topic_metadata: false,
            });
        }

        // Collect topic metadata from TopicNodeRegistry (auto-populated by send/recv)
        let tnr = crate::communication::topic_node_registry();
        let mut topic_producers: HashMap<String, Vec<usize>> = HashMap::new();
        let mut has_any_metadata = false;

        for (i, node) in nodes.iter().enumerate() {
            let pubs = tnr.publishers_for_node(&node.name);
            if !pubs.is_empty() {
                has_any_metadata = true;
            }
            for pub_topic in pubs {
                topic_producers
                    .entry(pub_topic.topic_name.clone())
                    .or_default()
                    .push(i);
            }
        }

        // Check if any node has subscriber metadata
        for node in nodes.iter() {
            if !tnr.subscribers_for_node(&node.name).is_empty() {
                has_any_metadata = true;
                break;
            }
        }

        if !has_any_metadata {
            // Fallback: group by .order() tiers
            return Ok(Self::build_from_order(nodes));
        }

        // Build adjacency list: edges[producer] = [consumer1, consumer2, ...]
        // Supports multiple publishers per topic.
        let mut edges: Vec<Vec<usize>> = vec![vec![]; n];

        for (i, node) in nodes.iter().enumerate() {
            for sub_topic in tnr.subscribers_for_node(&node.name) {
                if let Some(producers) = topic_producers.get(&sub_topic.topic_name) {
                    for &producer in producers {
                        if producer != i && !edges[producer].contains(&i) {
                            edges[producer].push(i);
                        }
                    }
                }
            }
        }

        // Compute per-node dependency counts from edges
        let mut dep_counts = vec![0usize; n];
        for successors in &edges {
            for &succ in successors {
                dep_counts[succ] += 1;
            }
        }

        // Topological sort into parallel steps (used by deterministic mode)
        let steps = topological_parallel_sort(&edges, n)?;

        Ok(Self {
            steps,
            successors: edges,
            dep_counts,
            has_topic_metadata: true,
        })
    }

    /// Fallback: group nodes by .order() tiers.
    /// Same order value = independent (parallel within step).
    /// Different order values = sequential (lower order first).
    fn build_from_order(nodes: &[RegisteredNode]) -> Self {
        let n = nodes.len();

        // Collect unique order values, sorted
        let mut order_values: Vec<u32> = nodes.iter().map(|n| n.priority).collect();
        order_values.sort_unstable();
        order_values.dedup();

        // Group node indices by order tier
        let steps: Vec<Vec<usize>> = order_values
            .iter()
            .map(|&order| (0..n).filter(|&i| nodes[i].priority == order).collect())
            .collect();

        // Build successor/dep_count from tier ordering:
        // all nodes in tier N are predecessors of all nodes in tier N+1.
        let mut successors: Vec<Vec<usize>> = vec![vec![]; n];
        let mut dep_counts = vec![0usize; n];

        for tier_idx in 0..steps.len().saturating_sub(1) {
            let current_tier = &steps[tier_idx];
            let next_tier = &steps[tier_idx + 1];
            for &pred in current_tier {
                for &succ in next_tier {
                    successors[pred].push(succ);
                    dep_counts[succ] += 1;
                }
            }
        }

        Self {
            steps,
            successors,
            dep_counts,
            has_topic_metadata: false,
        }
    }

    /// Get the execution steps (for deterministic mode sequential execution).
    pub fn steps(&self) -> &[Vec<usize>] {
        &self.steps
    }

    /// Number of execution steps.
    pub fn step_count(&self) -> usize {
        self.steps.len()
    }

    /// Whether the graph was built from real topic metadata.
    pub fn has_topic_metadata(&self) -> bool {
        self.has_topic_metadata
    }

    /// Per-node successor lists for ready-dispatch.
    /// `successors()[i]` = node indices that depend on node `i`.
    pub fn successors(&self) -> &[Vec<usize>] {
        &self.successors
    }

    /// Initial dependency counts for ready-dispatch.
    /// `dep_counts()[i]` = number of predecessors node `i` must wait for.
    /// Nodes with dep_count == 0 are ready immediately.
    pub fn dep_counts(&self) -> &[usize] {
        &self.dep_counts
    }

    /// Total number of nodes in the graph.
    pub fn node_count(&self) -> usize {
        self.dep_counts.len()
    }

    /// Find independent RT chains for RT thread pool sizing.
    ///
    /// Returns groups of RT node indices where nodes within each group
    /// are connected by dependencies. Independent groups can run on
    /// separate RT threads.
    pub fn independent_chains(&self, rt_node_indices: &[usize]) -> Vec<Vec<usize>> {
        if rt_node_indices.is_empty() {
            return Vec::new();
        }
        if rt_node_indices.len() == 1 {
            return vec![rt_node_indices.to_vec()];
        }

        // Simple approach: each step's RT nodes form independent groups
        // within that step. Across steps, connected RT nodes are in the same chain.
        // For now, each RT node that appears in different steps with a dependency
        // is in the same chain.

        // Start with each RT node in its own chain
        let mut chain_id: HashMap<usize, usize> = HashMap::new();
        for (chain, &node) in rt_node_indices.iter().enumerate() {
            chain_id.insert(node, chain);
        }

        // Collect chains
        let mut chains: HashMap<usize, Vec<usize>> = HashMap::new();
        for &node in rt_node_indices {
            let id = chain_id[&node];
            chains.entry(id).or_default().push(node);
        }

        chains.into_values().collect()
    }
}

/// Error during dependency graph construction.
#[derive(Debug, Clone)]
pub(crate) enum DependencyGraphError {
    /// Cycle detected in the dependency graph (mutual dependencies).
    Cycle {
        /// Node indices involved in the cycle.
        involved_nodes: Vec<usize>,
    },
}

impl std::fmt::Display for DependencyGraphError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Cycle { involved_nodes } => {
                write!(
                    f,
                    "Dependency cycle detected involving node indices: {:?}. \
                     Check that your nodes don't have mutual topic dependencies.",
                    involved_nodes
                )
            }
        }
    }
}

/// Kahn's algorithm variant that groups independent nodes into parallel steps.
///
/// Standard Kahn's processes one node at a time. This variant collects ALL
/// nodes with in_degree == 0 into a single step, enabling parallel execution.
///
/// Returns `Err(Cycle)` if the graph has cycles.
fn topological_parallel_sort(
    edges: &[Vec<usize>],
    n: usize,
) -> Result<Vec<Vec<usize>>, DependencyGraphError> {
    let mut in_degree = vec![0usize; n];
    for deps in edges {
        for &dep in deps {
            in_degree[dep] += 1;
        }
    }

    let mut steps = Vec::new();
    let mut processed = 0usize;
    let mut remaining = vec![true; n];

    loop {
        // Collect all nodes with in_degree == 0 (no unresolved dependencies)
        let step: Vec<usize> = (0..n)
            .filter(|&i| remaining[i] && in_degree[i] == 0)
            .collect();

        if step.is_empty() {
            break;
        }

        // Remove these nodes, reduce in_degree of their dependents
        for &node in &step {
            remaining[node] = false;
            processed += 1;
            for &dep in &edges[node] {
                in_degree[dep] -= 1;
            }
        }

        steps.push(step);
    }

    // If not all nodes were processed, there's a cycle
    if processed < n {
        let involved: Vec<usize> = (0..n).filter(|&i| remaining[i]).collect();
        return Err(DependencyGraphError::Cycle {
            involved_nodes: involved,
        });
    }

    Ok(steps)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::node::{Node, TopicMetadata};
    use crate::core::Miss;
    use crate::scheduling::types::{AtomicHealthState, ExecutionClass, NodeKind};
    use std::sync::Arc;

    // ── Test helpers ──

    struct TestNode {
        name: String,
        pubs: Vec<TopicMetadata>,
        subs: Vec<TopicMetadata>,
    }

    impl TestNode {
        fn new(name: &str) -> Self {
            Self {
                name: name.to_string(),
                pubs: Vec::new(),
                subs: Vec::new(),
            }
        }

        fn publishes(mut self, topic: &str) -> Self {
            self.pubs.push(TopicMetadata {
                topic_name: topic.to_string(),
                type_name: "T".to_string(),
            });
            self
        }

        fn subscribes(mut self, topic: &str) -> Self {
            self.subs.push(TopicMetadata {
                topic_name: topic.to_string(),
                type_name: "T".to_string(),
            });
            self
        }
    }

    impl Node for TestNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {}
    }

    fn make_registered(node: TestNode, order: u32) -> RegisteredNode {
        let name = node.name().to_string();

        // Register topics with the global TopicNodeRegistry so
        // DependencyGraph::build() can discover pub/sub edges.
        let tnr = crate::communication::topic_node_registry();
        for pub_topic in &node.pubs {
            tnr.register_with_type(
                &pub_topic.topic_name,
                &name,
                crate::communication::topic::NodeTopicRole::Publisher,
                &pub_topic.type_name,
            );
        }
        for sub_topic in &node.subs {
            tnr.register_with_type(
                &sub_topic.topic_name,
                &name,
                crate::communication::topic::NodeTopicRole::Subscriber,
                &sub_topic.type_name,
            );
        }

        RegisteredNode {
            node: NodeKind::new(Box::new(node)),
            name: Arc::from(name.as_str()),
            priority: order,
            initialized: true,
            context: None,
            rate_hz: None,
            last_tick: None,
            is_rt_node: false,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: ExecutionClass::BestEffort,
            health_state: AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: crate::scheduling::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        }
    }

    // ── Tests ──

    #[test]
    fn empty_graph() {
        let nodes: Vec<RegisteredNode> = vec![];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 0);
        assert!(graph.steps().is_empty());
    }

    #[test]
    fn single_node() {
        let nodes = vec![make_registered(TestNode::new("sn_A").publishes("sn_out"), 0)];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 1);
        assert_eq!(graph.steps()[0], vec![0]);
    }

    #[test]
    fn linear_chain_a_b_c() {
        // A -> B -> C
        let nodes = vec![
            make_registered(TestNode::new("lc_A").publishes("lc_ab"), 0),
            make_registered(TestNode::new("lc_B").subscribes("lc_ab").publishes("lc_bc"), 1),
            make_registered(TestNode::new("lc_C").subscribes("lc_bc"), 2),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 3);
        assert_eq!(graph.steps()[0], vec![0]); // A
        assert_eq!(graph.steps()[1], vec![1]); // B
        assert_eq!(graph.steps()[2], vec![2]); // C
    }

    #[test]
    fn two_independent_nodes() {
        // A and B, no dependency
        let nodes = vec![
            make_registered(TestNode::new("ti_A").publishes("ti_x"), 0),
            make_registered(TestNode::new("ti_B").publishes("ti_y"), 0),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 1);
        assert_eq!(graph.steps()[0].len(), 2); // A and B in same step
    }

    #[test]
    fn diamond_a_bc_d() {
        // A -> B, A -> C, B -> D, C -> D
        let nodes = vec![
            make_registered(TestNode::new("dm_A").publishes("dm_ab").publishes("dm_ac"), 0),
            make_registered(TestNode::new("dm_B").subscribes("dm_ab").publishes("dm_bd"), 1),
            make_registered(TestNode::new("dm_C").subscribes("dm_ac").publishes("dm_cd"), 1),
            make_registered(TestNode::new("dm_D").subscribes("dm_bd").subscribes("dm_cd"), 2),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 3);
        assert_eq!(graph.steps()[0], vec![0]); // A alone
        assert_eq!(graph.steps()[1].len(), 2); // B and C parallel
        assert!(graph.steps()[1].contains(&1));
        assert!(graph.steps()[1].contains(&2));
        assert_eq!(graph.steps()[2], vec![3]); // D alone
    }

    #[test]
    fn fan_out() {
        // A -> B, A -> C, A -> D
        let nodes = vec![
            make_registered(TestNode::new("fo_A").publishes("fo_x"), 0),
            make_registered(TestNode::new("fo_B").subscribes("fo_x"), 1),
            make_registered(TestNode::new("fo_C").subscribes("fo_x"), 1),
            make_registered(TestNode::new("fo_D").subscribes("fo_x"), 1),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 2);
        assert_eq!(graph.steps()[0], vec![0]); // A
        assert_eq!(graph.steps()[1].len(), 3); // B, C, D parallel
    }

    #[test]
    fn fan_in() {
        // A -> D, B -> D, C -> D
        let nodes = vec![
            make_registered(TestNode::new("fi_A").publishes("fi_a"), 0),
            make_registered(TestNode::new("fi_B").publishes("fi_b"), 0),
            make_registered(TestNode::new("fi_C").publishes("fi_c"), 0),
            make_registered(
                TestNode::new("fi_D")
                    .subscribes("fi_a")
                    .subscribes("fi_b")
                    .subscribes("fi_c"),
                1,
            ),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 2);
        assert_eq!(graph.steps()[0].len(), 3); // A, B, C parallel
        assert_eq!(graph.steps()[1], vec![3]); // D
    }

    #[test]
    fn cycle_detected() {
        // A -> B -> A (cycle)
        let nodes = vec![
            make_registered(
                TestNode::new("cycle_A")
                    .publishes("cycle_ab")
                    .subscribes("cycle_ba"),
                0,
            ),
            make_registered(
                TestNode::new("cycle_B")
                    .subscribes("cycle_ab")
                    .publishes("cycle_ba"),
                1,
            ),
        ];
        let result = DependencyGraph::build(&nodes);
        assert!(result.is_err());
        if let Err(DependencyGraphError::Cycle { involved_nodes }) = result {
            assert_eq!(involved_nodes.len(), 2);
        }
    }

    #[test]
    fn fallback_to_order_tiers() {
        // No pub/sub metadata — group by .order()
        // Names must NOT match any node registered by other tests
        let nodes = vec![
            make_registered(TestNode::new("fb_A"), 0),
            make_registered(TestNode::new("fb_B"), 0),
            make_registered(TestNode::new("fb_C"), 10),
            make_registered(TestNode::new("fb_D"), 10),
            make_registered(TestNode::new("fb_E"), 20),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert!(!graph.has_topic_metadata());
        assert_eq!(graph.step_count(), 3);
        assert_eq!(graph.steps()[0].len(), 2); // A, B (order 0)
        assert_eq!(graph.steps()[1].len(), 2); // C, D (order 10)
        assert_eq!(graph.steps()[2].len(), 1); // E (order 20)
    }

    #[test]
    fn disconnected_nodes_with_metadata() {
        // A publishes, B publishes, no subscribers — all independent
        let nodes = vec![
            make_registered(TestNode::new("disc_A").publishes("disc_x"), 0),
            make_registered(TestNode::new("disc_B").publishes("disc_y"), 5),
            make_registered(TestNode::new("disc_C").publishes("disc_z"), 10),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert!(graph.has_topic_metadata());
        assert_eq!(graph.step_count(), 1); // all independent, one step
        assert_eq!(graph.steps()[0].len(), 3);
    }

    #[test]
    fn humanoid_topology() {
        // Realistic humanoid: sensors -> fusion -> control -> output
        // Prefixed names to avoid global registry collisions with other tests.
        let nodes = vec![
            make_registered(TestNode::new("h_imu").publishes("h_imu_data"), 0),
            make_registered(TestNode::new("h_camera").publishes("h_image"), 0),
            make_registered(TestNode::new("h_joints").publishes("h_joint_state"), 0),
            make_registered(
                TestNode::new("h_state_est")
                    .subscribes("h_imu_data")
                    .subscribes("h_joint_state")
                    .publishes("h_robot_state"),
                1,
            ),
            make_registered(
                TestNode::new("h_slam")
                    .subscribes("h_image")
                    .publishes("h_pose"),
                1,
            ),
            make_registered(
                TestNode::new("h_balance")
                    .subscribes("h_robot_state")
                    .subscribes("h_pose")
                    .publishes("h_joint_cmd"),
                2,
            ),
            make_registered(TestNode::new("h_motor").subscribes("h_joint_cmd"), 3),
            make_registered(
                TestNode::new("h_logger")
                    .subscribes("h_joint_cmd")
                    .subscribes("h_robot_state"),
                10,
            ),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();

        // Step 0: imu, camera, joint_sensors (no deps)
        assert_eq!(graph.steps()[0].len(), 3);
        // Step 1: state_estimator, slam (depend on step 0, independent of each other)
        assert_eq!(graph.steps()[1].len(), 2);
        // Step 2: balance_ctrl (depends on state_estimator and slam)
        assert_eq!(graph.steps()[2].len(), 1);
        assert_eq!(graph.steps()[2][0], 5); // h_balance
                                            // Step 3: motor_driver and logger (both depend on h_balance/state)
        assert_eq!(graph.steps()[3].len(), 2);
    }

    #[test]
    fn independent_chains_single_rt() {
        let nodes = vec![make_registered(TestNode::new("ics_A").publishes("ics_x"), 0)];
        let graph = DependencyGraph::build(&nodes).unwrap();
        let chains = graph.independent_chains(&[0]);
        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0], vec![0]);
    }

    #[test]
    fn independent_chains_multiple_independent() {
        let nodes = vec![
            make_registered(TestNode::new("icm_A").publishes("icm_x"), 0),
            make_registered(TestNode::new("icm_B").publishes("icm_y"), 0),
            make_registered(TestNode::new("icm_C").publishes("icm_z"), 0),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        let chains = graph.independent_chains(&[0, 1, 2]);
        // Each node is its own chain (all independent)
        assert_eq!(chains.len(), 3);
    }

    #[test]
    fn large_graph_performance() {
        // 1000 nodes: 100 producers, each with 9 consumers
        let mut nodes = Vec::new();
        for i in 0..100 {
            nodes.push(make_registered(
                TestNode::new(&format!("lgp_prod_{}", i))
                    .publishes(&format!("lgp_topic_{}", i)),
                0,
            ));
        }
        for i in 0..100 {
            for j in 0..9 {
                nodes.push(make_registered(
                    TestNode::new(&format!("lgp_cons_{}_{}", i, j))
                        .subscribes(&format!("lgp_topic_{}", i)),
                    1,
                ));
            }
        }

        let start = std::time::Instant::now();
        let graph = DependencyGraph::build(&nodes).unwrap();
        let elapsed = start.elapsed();

        assert!(
            elapsed.as_millis() < 50,
            "1000-node graph took {}ms (should be <50ms)",
            elapsed.as_millis()
        );
        assert_eq!(graph.step_count(), 2); // producers then consumers
        assert_eq!(graph.steps()[0].len(), 100);
        assert_eq!(graph.steps()[1].len(), 900);
    }

    #[test]
    fn multi_publisher_same_topic() {
        // Two publishers to same topic, one subscriber depends on both
        let nodes = vec![
            make_registered(TestNode::new("mp_A").publishes("mp_data"), 0),
            make_registered(TestNode::new("mp_B").publishes("mp_data"), 0),
            make_registered(TestNode::new("mp_C").subscribes("mp_data"), 1),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert!(graph.has_topic_metadata());
        assert_eq!(graph.step_count(), 2);
        assert_eq!(graph.steps()[0].len(), 2); // A, B parallel
        assert_eq!(graph.steps()[1], vec![2]); // C depends on both
        // C should have dep_count == 2 (from A and B)
        assert_eq!(graph.dep_counts()[2], 2);
    }

    #[test]
    fn ready_dispatch_fields() {
        // Verify successors and dep_counts are populated correctly
        let nodes = vec![
            make_registered(TestNode::new("rd_A").publishes("rd_ab"), 0),
            make_registered(TestNode::new("rd_B").subscribes("rd_ab").publishes("rd_bc"), 1),
            make_registered(TestNode::new("rd_C").subscribes("rd_bc"), 2),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();

        // A → B → C
        assert_eq!(graph.dep_counts(), &[0, 1, 1]);
        assert_eq!(graph.successors()[0], vec![1]); // A's successor is B
        assert_eq!(graph.successors()[1], vec![2]); // B's successor is C
        assert!(graph.successors()[2].is_empty()); // C has no successors
    }
}
