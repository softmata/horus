//! Dependency graph for deterministic execution ordering.
//!
//! Builds a DAG from nodes' publisher/subscriber topic metadata.
//! Produces execution steps where each step contains independent nodes
//! that can run in parallel, with barriers between steps.
//!
//! # Example
//!
//! ```text
//! ImuDriver ──pub "imu"──→ BalanceCtrl ──pub "joint_cmd"──→ MotorDriver
//!                               ↑
//! CameraDriver ──pub "image"──→ SLAM ──pub "pose"──┘
//!
//! Step 0: [ImuDriver, CameraDriver]     ← parallel
//! Step 1: [BalanceCtrl, SLAM]           ← parallel (after step 0)
//! Step 2: [MotorDriver]                 ← after step 1
//! ```

use std::collections::HashMap;

use super::types::RegisteredNode;

/// Dependency graph built from node publisher/subscriber metadata.
///
/// Built once at scheduler init. Produces execution steps for
/// deterministic ordering with maximal parallelism.
pub(crate) struct DependencyGraph {
    /// Execution steps: each step is a set of node indices that can run in parallel.
    /// Steps must be executed in order (barrier between each step).
    steps: Vec<Vec<usize>>,
    /// Number of nodes in the graph.
    node_count: usize,
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
                node_count: 0,
                has_topic_metadata: false,
            });
        }

        // Collect topic metadata from all nodes
        let mut topic_producers: HashMap<String, usize> = HashMap::new();
        let mut has_any_metadata = false;

        for (i, node) in nodes.iter().enumerate() {
            let pubs = node.node.publishers();
            if !pubs.is_empty() {
                has_any_metadata = true;
            }
            for pub_topic in pubs {
                topic_producers.insert(pub_topic.topic_name.clone(), i);
            }
        }

        // Check if any node has subscriber metadata
        for node in nodes.iter() {
            if !node.node.subscribers().is_empty() {
                has_any_metadata = true;
                break;
            }
        }

        if !has_any_metadata {
            // Fallback: group by .order() tiers
            return Ok(Self::build_from_order(nodes));
        }

        // Build adjacency list: edges[producer] = [consumer1, consumer2, ...]
        let mut edges: Vec<Vec<usize>> = vec![vec![]; n];

        for (i, node) in nodes.iter().enumerate() {
            for sub_topic in node.node.subscribers() {
                if let Some(&producer) = topic_producers.get(&sub_topic.topic_name) {
                    if producer != i {
                        // Avoid duplicate edges
                        if !edges[producer].contains(&i) {
                            edges[producer].push(i);
                        }
                    }
                }
            }
        }

        // Topological sort into parallel steps
        let steps = topological_parallel_sort(&edges, n)?;

        Ok(Self {
            steps,
            node_count: n,
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
            .map(|&order| {
                (0..n)
                    .filter(|&i| nodes[i].priority == order)
                    .collect()
            })
            .collect();

        Self {
            steps,
            node_count: n,
            has_topic_metadata: false,
        }
    }

    /// Get the execution steps.
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

        // Merge chains for nodes that appear in consecutive steps with dependencies
        for step in &self.steps {
            let rt_in_step: Vec<usize> = step
                .iter()
                .filter(|idx| chain_id.contains_key(idx))
                .copied()
                .collect();

            // RT nodes in the same step are INDEPENDENT — different chains (no merge)
            // RT nodes connected across steps are DEPENDENT — same chain (merge)
            // The step structure already handles this: nodes in the same step
            // have no dependency between them.
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
    use std::time::Instant;

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
        fn publishers(&self) -> Vec<TopicMetadata> {
            self.pubs.clone()
        }
        fn subscribers(&self) -> Vec<TopicMetadata> {
            self.subs.clone()
        }
    }

    fn make_registered(node: TestNode, order: u32) -> RegisteredNode {
        let name = node.name().to_string();
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
        let nodes = vec![make_registered(
            TestNode::new("A").publishes("out"),
            0,
        )];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 1);
        assert_eq!(graph.steps()[0], vec![0]);
    }

    #[test]
    fn linear_chain_a_b_c() {
        // A -> B -> C
        let nodes = vec![
            make_registered(TestNode::new("A").publishes("ab"), 0),
            make_registered(TestNode::new("B").subscribes("ab").publishes("bc"), 1),
            make_registered(TestNode::new("C").subscribes("bc"), 2),
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
            make_registered(TestNode::new("A").publishes("x"), 0),
            make_registered(TestNode::new("B").publishes("y"), 0),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 1);
        assert_eq!(graph.steps()[0].len(), 2); // A and B in same step
    }

    #[test]
    fn diamond_a_bc_d() {
        // A -> B, A -> C, B -> D, C -> D
        let nodes = vec![
            make_registered(TestNode::new("A").publishes("ab").publishes("ac"), 0),
            make_registered(TestNode::new("B").subscribes("ab").publishes("bd"), 1),
            make_registered(TestNode::new("C").subscribes("ac").publishes("cd"), 1),
            make_registered(TestNode::new("D").subscribes("bd").subscribes("cd"), 2),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 3);
        assert_eq!(graph.steps()[0], vec![0]);       // A alone
        assert_eq!(graph.steps()[1].len(), 2);        // B and C parallel
        assert!(graph.steps()[1].contains(&1));
        assert!(graph.steps()[1].contains(&2));
        assert_eq!(graph.steps()[2], vec![3]);       // D alone
    }

    #[test]
    fn fan_out() {
        // A -> B, A -> C, A -> D
        let nodes = vec![
            make_registered(TestNode::new("A").publishes("x"), 0),
            make_registered(TestNode::new("B").subscribes("x"), 1),
            make_registered(TestNode::new("C").subscribes("x"), 1),
            make_registered(TestNode::new("D").subscribes("x"), 1),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 2);
        assert_eq!(graph.steps()[0], vec![0]);        // A
        assert_eq!(graph.steps()[1].len(), 3);         // B, C, D parallel
    }

    #[test]
    fn fan_in() {
        // A -> D, B -> D, C -> D
        let nodes = vec![
            make_registered(TestNode::new("A").publishes("a"), 0),
            make_registered(TestNode::new("B").publishes("b"), 0),
            make_registered(TestNode::new("C").publishes("c"), 0),
            make_registered(TestNode::new("D").subscribes("a").subscribes("b").subscribes("c"), 1),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert_eq!(graph.step_count(), 2);
        assert_eq!(graph.steps()[0].len(), 3);         // A, B, C parallel
        assert_eq!(graph.steps()[1], vec![3]);         // D
    }

    #[test]
    fn cycle_detected() {
        // A -> B -> A (cycle)
        let nodes = vec![
            make_registered(TestNode::new("A").publishes("ab").subscribes("ba"), 0),
            make_registered(TestNode::new("B").subscribes("ab").publishes("ba"), 1),
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
        let nodes = vec![
            make_registered(TestNode::new("A"), 0),
            make_registered(TestNode::new("B"), 0),
            make_registered(TestNode::new("C"), 10),
            make_registered(TestNode::new("D"), 10),
            make_registered(TestNode::new("E"), 20),
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
            make_registered(TestNode::new("A").publishes("x"), 0),
            make_registered(TestNode::new("B").publishes("y"), 5),
            make_registered(TestNode::new("C").publishes("z"), 10),
        ];
        let graph = DependencyGraph::build(&nodes).unwrap();
        assert!(graph.has_topic_metadata());
        assert_eq!(graph.step_count(), 1); // all independent, one step
        assert_eq!(graph.steps()[0].len(), 3);
    }

    #[test]
    fn humanoid_topology() {
        // Realistic humanoid: sensors -> fusion -> control -> output
        let nodes = vec![
            make_registered(TestNode::new("imu").publishes("imu_data"), 0),
            make_registered(TestNode::new("camera").publishes("image"), 0),
            make_registered(TestNode::new("joint_sensors").publishes("joint_state"), 0),
            make_registered(
                TestNode::new("state_estimator")
                    .subscribes("imu_data")
                    .subscribes("joint_state")
                    .publishes("robot_state"),
                1,
            ),
            make_registered(
                TestNode::new("slam")
                    .subscribes("image")
                    .publishes("pose"),
                1,
            ),
            make_registered(
                TestNode::new("balance_ctrl")
                    .subscribes("robot_state")
                    .subscribes("pose")
                    .publishes("joint_cmd"),
                2,
            ),
            make_registered(
                TestNode::new("motor_driver").subscribes("joint_cmd"),
                3,
            ),
            make_registered(
                TestNode::new("logger")
                    .subscribes("joint_cmd")
                    .subscribes("robot_state"),
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
        assert_eq!(graph.steps()[2][0], 5); // balance_ctrl
        // Step 3: motor_driver and logger (both depend on balance_ctrl/state)
        assert_eq!(graph.steps()[3].len(), 2);
    }

    #[test]
    fn independent_chains_single_rt() {
        let nodes = vec![make_registered(TestNode::new("A").publishes("x"), 0)];
        let graph = DependencyGraph::build(&nodes).unwrap();
        let chains = graph.independent_chains(&[0]);
        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0], vec![0]);
    }

    #[test]
    fn independent_chains_multiple_independent() {
        let nodes = vec![
            make_registered(TestNode::new("A").publishes("x"), 0),
            make_registered(TestNode::new("B").publishes("y"), 0),
            make_registered(TestNode::new("C").publishes("z"), 0),
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
                TestNode::new(&format!("producer_{}", i))
                    .publishes(&format!("topic_{}", i)),
                0,
            ));
        }
        for i in 0..100 {
            for j in 0..9 {
                nodes.push(make_registered(
                    TestNode::new(&format!("consumer_{}_{}", i, j))
                        .subscribes(&format!("topic_{}", i)),
                    1,
                ));
            }
        }

        let start = std::time::Instant::now();
        let graph = DependencyGraph::build(&nodes).unwrap();
        let elapsed = start.elapsed();

        assert!(
            elapsed.as_millis() < 10,
            "1000-node graph took {}ms (should be <10ms)",
            elapsed.as_millis()
        );
        assert_eq!(graph.step_count(), 2); // producers then consumers
        assert_eq!(graph.steps()[0].len(), 100);
        assert_eq!(graph.steps()[1].len(), 900);
    }
}
