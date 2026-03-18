#![allow(dead_code)]
//! Node dependency graph for the scheduler.
//!
//! Models execution ordering constraints between nodes.  Each edge `(A, B)`
//! means "A must tick before B."  The graph validates that no cycles exist
//! and produces a topological ordering that the scheduler can follow.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::scheduling::graph::DependencyGraph;
//!
//! let mut g = DependencyGraph::new();
//! g.add_node("sensor");
//! g.add_node("filter");
//! g.add_node("planner");
//! g.add_edge("sensor", "filter");   // sensor before filter
//! g.add_edge("filter", "planner");  // filter before planner
//!
//! let order = g.topological_sort().unwrap();
//! assert_eq!(order, vec!["sensor", "filter", "planner"]);
//! ```

use std::collections::{HashMap, HashSet, VecDeque};

/// Errors produced by [`DependencyGraph`] operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GraphError {
    /// A dependency cycle was detected; the contained `Vec` lists the nodes
    /// that form the cycle (in traversal order).
    CycleDetected(Vec<String>),
    /// An edge references a node that has not been added.
    UnknownNode(String),
    /// Duplicate edge between the same pair.
    DuplicateEdge { from: String, to: String },
}

impl std::fmt::Display for GraphError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GraphError::CycleDetected(cycle) => {
                write!(f, "dependency cycle: {}", cycle.join(" -> "))
            }
            GraphError::UnknownNode(name) => write!(f, "unknown node: {}", name),
            GraphError::DuplicateEdge { from, to } => {
                write!(f, "duplicate edge: {} -> {}", from, to)
            }
        }
    }
}

impl std::error::Error for GraphError {}

/// A directed acyclic graph of node execution dependencies.
///
/// Nodes are identified by name (`String`).  Edges encode "must run before"
/// relationships.  The graph supports:
///
/// - **Cycle detection** via Kahn's algorithm during topological sort.
/// - **Topological ordering** for deterministic scheduling.
/// - **Dependency queries**: transitive predecessors / successors of a node.
#[derive(Debug, Clone)]
pub struct DependencyGraph {
    /// Set of node names.
    nodes: HashSet<String>,
    /// Adjacency list: `from` -> set of `to` nodes.
    edges: HashMap<String, HashSet<String>>,
    /// Reverse adjacency list: `to` -> set of `from` nodes.
    reverse_edges: HashMap<String, HashSet<String>>,
}

impl Default for DependencyGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl DependencyGraph {
    /// Create an empty dependency graph.
    pub fn new() -> Self {
        Self {
            nodes: HashSet::new(),
            edges: HashMap::new(),
            reverse_edges: HashMap::new(),
        }
    }

    /// Number of nodes in the graph.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of directed edges.
    pub fn edge_count(&self) -> usize {
        self.edges.values().map(|s| s.len()).sum()
    }

    /// Returns `true` if the graph contains no nodes.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Returns `true` if `name` has been added as a node.
    pub fn contains_node(&self, name: &str) -> bool {
        self.nodes.contains(name)
    }

    /// Add a node.  Returns `true` if the node was new, `false` if it
    /// already existed.
    pub fn add_node(&mut self, name: &str) -> bool {
        if self.nodes.insert(name.to_string()) {
            self.edges.entry(name.to_string()).or_default();
            self.reverse_edges.entry(name.to_string()).or_default();
            true
        } else {
            false
        }
    }

    /// Remove a node and all edges touching it.  Returns `true` if the node
    /// existed.
    pub fn remove_node(&mut self, name: &str) -> bool {
        if !self.nodes.remove(name) {
            return false;
        }

        // Remove outgoing edges.
        if let Some(successors) = self.edges.remove(name) {
            for succ in &successors {
                if let Some(preds) = self.reverse_edges.get_mut(succ.as_str()) {
                    preds.remove(name);
                }
            }
        }

        // Remove incoming edges.
        if let Some(predecessors) = self.reverse_edges.remove(name) {
            for pred in &predecessors {
                if let Some(succs) = self.edges.get_mut(pred.as_str()) {
                    succs.remove(name);
                }
            }
        }

        true
    }

    /// Add a directed edge `from -> to` (meaning `from` must tick before `to`).
    ///
    /// Both nodes must already exist.  Duplicate edges are rejected.
    pub fn add_edge(&mut self, from: &str, to: &str) -> Result<(), GraphError> {
        if !self.nodes.contains(from) {
            return Err(GraphError::UnknownNode(from.to_string()));
        }
        if !self.nodes.contains(to) {
            return Err(GraphError::UnknownNode(to.to_string()));
        }
        let succs = self.edges.entry(from.to_string()).or_default();
        if !succs.insert(to.to_string()) {
            return Err(GraphError::DuplicateEdge {
                from: from.to_string(),
                to: to.to_string(),
            });
        }
        self.reverse_edges
            .entry(to.to_string())
            .or_default()
            .insert(from.to_string());
        Ok(())
    }

    /// Remove an edge.  Returns `true` if the edge existed.
    pub fn remove_edge(&mut self, from: &str, to: &str) -> bool {
        let removed = self.edges.get_mut(from).map_or(false, |s| s.remove(to));
        if removed {
            if let Some(preds) = self.reverse_edges.get_mut(to) {
                preds.remove(from);
            }
        }
        removed
    }

    /// Direct successors of `node` (nodes that depend on it).
    pub fn successors(&self, node: &str) -> Vec<&str> {
        self.edges
            .get(node)
            .map(|s| s.iter().map(String::as_str).collect())
            .unwrap_or_default()
    }

    /// Direct predecessors of `node` (nodes it depends on).
    pub fn predecessors(&self, node: &str) -> Vec<&str> {
        self.reverse_edges
            .get(node)
            .map(|s| s.iter().map(String::as_str).collect())
            .unwrap_or_default()
    }

    /// In-degree of a node (number of incoming edges).
    pub fn in_degree(&self, node: &str) -> usize {
        self.reverse_edges.get(node).map_or(0, |s| s.len())
    }

    /// All transitive predecessors (ancestors) of `node`, via BFS.
    pub fn transitive_predecessors(&self, node: &str) -> HashSet<String> {
        let mut visited = HashSet::new();
        let mut queue = VecDeque::new();

        if let Some(preds) = self.reverse_edges.get(node) {
            for p in preds {
                queue.push_back(p.as_str());
            }
        }

        while let Some(current) = queue.pop_front() {
            if visited.insert(current.to_string()) {
                if let Some(preds) = self.reverse_edges.get(current) {
                    for p in preds {
                        if !visited.contains(p.as_str()) {
                            queue.push_back(p.as_str());
                        }
                    }
                }
            }
        }

        visited
    }

    /// All transitive successors (descendants) of `node`, via BFS.
    pub fn transitive_successors(&self, node: &str) -> HashSet<String> {
        let mut visited = HashSet::new();
        let mut queue = VecDeque::new();

        if let Some(succs) = self.edges.get(node) {
            for s in succs {
                queue.push_back(s.as_str());
            }
        }

        while let Some(current) = queue.pop_front() {
            if visited.insert(current.to_string()) {
                if let Some(succs) = self.edges.get(current) {
                    for s in succs {
                        if !visited.contains(s.as_str()) {
                            queue.push_back(s.as_str());
                        }
                    }
                }
            }
        }

        visited
    }

    /// Compute a topological ordering using Kahn's algorithm.
    ///
    /// Returns `Ok(vec)` with nodes in dependency-safe order, or
    /// `Err(GraphError::CycleDetected(_))` if the graph contains a cycle.
    ///
    /// When multiple nodes have the same in-degree, they are emitted in
    /// alphabetical order for determinism.
    pub fn topological_sort(&self) -> Result<Vec<String>, GraphError> {
        // Local in-degree map (will be mutated).
        let mut in_deg: HashMap<&str, usize> = HashMap::new();
        for node in &self.nodes {
            in_deg.insert(node.as_str(), self.in_degree(node));
        }

        // Seed the queue with zero-in-degree nodes (sorted for determinism).
        let mut queue: VecDeque<&str> = VecDeque::new();
        let mut roots: Vec<&str> = in_deg
            .iter()
            .filter(|(_, &d)| d == 0)
            .map(|(&n, _)| n)
            .collect();
        roots.sort();
        for r in roots {
            queue.push_back(r);
        }

        let mut result: Vec<String> = Vec::with_capacity(self.nodes.len());

        while let Some(node) = queue.pop_front() {
            result.push(node.to_string());

            // Collect and sort successors for deterministic ordering.
            let mut succs: Vec<&str> = self.successors(node);
            succs.sort();

            for succ in succs {
                if let Some(d) = in_deg.get_mut(succ) {
                    *d -= 1;
                    if *d == 0 {
                        queue.push_back(succ);
                    }
                }
            }
        }

        if result.len() != self.nodes.len() {
            // Some nodes were never emitted — they must be in a cycle.
            let cycle_nodes: Vec<String> = self
                .nodes
                .iter()
                .filter(|n| !result.contains(n))
                .cloned()
                .collect();
            Err(GraphError::CycleDetected(cycle_nodes))
        } else {
            Ok(result)
        }
    }

    /// Returns `true` if the graph is acyclic.
    pub fn is_acyclic(&self) -> bool {
        self.topological_sort().is_ok()
    }

    /// Nodes that have no incoming edges (execution roots).
    pub fn root_nodes(&self) -> Vec<&str> {
        let mut roots: Vec<&str> = self
            .nodes
            .iter()
            .filter(|n| self.in_degree(n) == 0)
            .map(String::as_str)
            .collect();
        roots.sort();
        roots
    }

    /// Nodes that have no outgoing edges (execution leaves).
    pub fn leaf_nodes(&self) -> Vec<&str> {
        let mut leaves: Vec<&str> = self
            .nodes
            .iter()
            .filter(|n| self.edges.get(n.as_str()).map_or(true, |s| s.is_empty()))
            .map(String::as_str)
            .collect();
        leaves.sort();
        leaves
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ======================================================================
    // Construction & basic properties
    // ======================================================================

    #[test]
    fn empty_graph_properties() {
        let g = DependencyGraph::new();
        assert!(g.is_empty());
        assert_eq!(g.node_count(), 0);
        assert_eq!(g.edge_count(), 0);
        assert!(g.is_acyclic());
        assert_eq!(g.topological_sort().unwrap(), Vec::<String>::new());
        assert!(g.root_nodes().is_empty());
        assert!(g.leaf_nodes().is_empty());
    }

    #[test]
    fn default_equals_new() {
        let a = DependencyGraph::new();
        let b = DependencyGraph::default();
        assert_eq!(a.node_count(), b.node_count());
        assert_eq!(a.edge_count(), b.edge_count());
    }

    // ======================================================================
    // Node operations
    // ======================================================================

    #[test]
    fn add_node_returns_true_for_new_false_for_duplicate() {
        let mut g = DependencyGraph::new();
        assert!(g.add_node("a"));
        assert!(!g.add_node("a"), "duplicate add should return false");
        assert_eq!(g.node_count(), 1);
    }

    #[test]
    fn contains_node_reflects_additions_and_removals() {
        let mut g = DependencyGraph::new();
        assert!(!g.contains_node("x"));
        g.add_node("x");
        assert!(g.contains_node("x"));
        g.remove_node("x");
        assert!(!g.contains_node("x"));
    }

    #[test]
    fn remove_nonexistent_node_returns_false() {
        let mut g = DependencyGraph::new();
        assert!(!g.remove_node("phantom"));
    }

    #[test]
    fn remove_node_cleans_up_edges_in_both_directions() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_node("c");
        g.add_edge("a", "b").unwrap();
        g.add_edge("b", "c").unwrap();
        assert_eq!(g.edge_count(), 2);

        g.remove_node("b");
        assert_eq!(g.edge_count(), 0);
        assert_eq!(g.node_count(), 2);

        // "a" should have no successors, "c" should have no predecessors.
        assert!(g.successors("a").is_empty());
        assert!(g.predecessors("c").is_empty());

        // Graph should still sort cleanly.
        let order = g.topological_sort().unwrap();
        assert_eq!(order.len(), 2);
    }

    // ======================================================================
    // Edge operations
    // ======================================================================

    #[test]
    fn add_edge_unknown_from_errors() {
        let mut g = DependencyGraph::new();
        g.add_node("b");
        let err = g.add_edge("ghost", "b").unwrap_err();
        assert_eq!(err, GraphError::UnknownNode("ghost".to_string()));
    }

    #[test]
    fn add_edge_unknown_to_errors() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        let err = g.add_edge("a", "ghost").unwrap_err();
        assert_eq!(err, GraphError::UnknownNode("ghost".to_string()));
    }

    #[test]
    fn add_duplicate_edge_errors() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_edge("a", "b").unwrap();
        let err = g.add_edge("a", "b").unwrap_err();
        assert_eq!(
            err,
            GraphError::DuplicateEdge {
                from: "a".to_string(),
                to: "b".to_string(),
            }
        );
    }

    #[test]
    fn remove_edge_returns_false_when_absent() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        assert!(!g.remove_edge("a", "b"));
    }

    #[test]
    fn remove_edge_cleans_up_both_adjacency_lists() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_edge("a", "b").unwrap();
        assert_eq!(g.edge_count(), 1);

        assert!(g.remove_edge("a", "b"));
        assert_eq!(g.edge_count(), 0);
        assert!(g.successors("a").is_empty());
        assert!(g.predecessors("b").is_empty());
    }

    // ======================================================================
    // Topological sort — linear chains
    // ======================================================================

    #[test]
    fn linear_chain_sorts_in_dependency_order() {
        let mut g = DependencyGraph::new();
        for name in &["sensor", "filter", "planner", "actuator"] {
            g.add_node(name);
        }
        g.add_edge("sensor", "filter").unwrap();
        g.add_edge("filter", "planner").unwrap();
        g.add_edge("planner", "actuator").unwrap();

        let order = g.topological_sort().unwrap();
        assert_eq!(order, vec!["sensor", "filter", "planner", "actuator"]);
    }

    #[test]
    fn single_node_sorts_trivially() {
        let mut g = DependencyGraph::new();
        g.add_node("solo");
        let order = g.topological_sort().unwrap();
        assert_eq!(order, vec!["solo"]);
    }

    // ======================================================================
    // Topological sort — diamond and DAG shapes
    // ======================================================================

    #[test]
    fn diamond_dag_produces_valid_topological_order() {
        //     A
        //    / \
        //   B   C
        //    \ /
        //     D
        let mut g = DependencyGraph::new();
        for name in &["A", "B", "C", "D"] {
            g.add_node(name);
        }
        g.add_edge("A", "B").unwrap();
        g.add_edge("A", "C").unwrap();
        g.add_edge("B", "D").unwrap();
        g.add_edge("C", "D").unwrap();

        let order = g.topological_sort().unwrap();

        // A must come first.
        assert_eq!(order[0], "A");
        // D must come last.
        assert_eq!(order[3], "D");
        // B and C must both come after A and before D.
        let b_pos = order.iter().position(|n| n == "B").unwrap();
        let c_pos = order.iter().position(|n| n == "C").unwrap();
        assert!(b_pos > 0 && b_pos < 3);
        assert!(c_pos > 0 && c_pos < 3);
    }

    #[test]
    fn disconnected_nodes_sorted_alphabetically() {
        let mut g = DependencyGraph::new();
        g.add_node("z");
        g.add_node("a");
        g.add_node("m");

        let order = g.topological_sort().unwrap();
        // All roots, sorted alphabetically for determinism.
        assert_eq!(order, vec!["a", "m", "z"]);
    }

    // ======================================================================
    // Cycle detection
    // ======================================================================

    #[test]
    fn self_loop_is_detected() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_edge("a", "a").unwrap();

        assert!(!g.is_acyclic());
        let err = g.topological_sort().unwrap_err();
        match err {
            GraphError::CycleDetected(nodes) => {
                assert!(nodes.contains(&"a".to_string()));
            }
            other => panic!("expected CycleDetected, got {:?}", other),
        }
    }

    #[test]
    fn two_node_cycle_detected() {
        let mut g = DependencyGraph::new();
        g.add_node("x");
        g.add_node("y");
        g.add_edge("x", "y").unwrap();
        g.add_edge("y", "x").unwrap();

        assert!(!g.is_acyclic());
        let err = g.topological_sort().unwrap_err();
        match err {
            GraphError::CycleDetected(nodes) => {
                assert_eq!(nodes.len(), 2);
                assert!(nodes.contains(&"x".to_string()));
                assert!(nodes.contains(&"y".to_string()));
            }
            other => panic!("expected CycleDetected, got {:?}", other),
        }
    }

    #[test]
    fn three_node_cycle_detected() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_node("c");
        g.add_edge("a", "b").unwrap();
        g.add_edge("b", "c").unwrap();
        g.add_edge("c", "a").unwrap();

        assert!(!g.is_acyclic());
        let err = g.topological_sort().unwrap_err();
        match err {
            GraphError::CycleDetected(nodes) => {
                assert_eq!(nodes.len(), 3);
            }
            other => panic!("expected CycleDetected, got {:?}", other),
        }
    }

    #[test]
    fn cycle_among_subset_does_not_include_acyclic_nodes() {
        // Graph:  root -> a -> b -> c -> a  (cycle in {a, b, c})
        //         root has no cycle itself.
        let mut g = DependencyGraph::new();
        for name in &["root", "a", "b", "c"] {
            g.add_node(name);
        }
        g.add_edge("root", "a").unwrap();
        g.add_edge("a", "b").unwrap();
        g.add_edge("b", "c").unwrap();
        g.add_edge("c", "a").unwrap();

        let err = g.topological_sort().unwrap_err();
        match err {
            GraphError::CycleDetected(nodes) => {
                assert!(
                    !nodes.contains(&"root".to_string()),
                    "root is not part of the cycle"
                );
                assert_eq!(nodes.len(), 3, "exactly a, b, c are in the cycle");
            }
            other => panic!("expected CycleDetected, got {:?}", other),
        }
    }

    // ======================================================================
    // In-degree, predecessors, successors
    // ======================================================================

    #[test]
    fn in_degree_reflects_incoming_edges() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_node("c");
        g.add_edge("a", "c").unwrap();
        g.add_edge("b", "c").unwrap();

        assert_eq!(g.in_degree("a"), 0);
        assert_eq!(g.in_degree("b"), 0);
        assert_eq!(g.in_degree("c"), 2);
    }

    #[test]
    fn predecessors_and_successors_are_consistent() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_edge("a", "b").unwrap();

        let a_succs = g.successors("a");
        let b_preds = g.predecessors("b");
        assert_eq!(a_succs, vec!["b"]);
        assert_eq!(b_preds, vec!["a"]);
    }

    // ======================================================================
    // Transitive queries
    // ======================================================================

    #[test]
    fn transitive_predecessors_of_leaf_include_all_ancestors() {
        //  a -> b -> c -> d
        let mut g = DependencyGraph::new();
        for name in &["a", "b", "c", "d"] {
            g.add_node(name);
        }
        g.add_edge("a", "b").unwrap();
        g.add_edge("b", "c").unwrap();
        g.add_edge("c", "d").unwrap();

        let preds = g.transitive_predecessors("d");
        assert_eq!(preds.len(), 3);
        assert!(preds.contains("a"));
        assert!(preds.contains("b"));
        assert!(preds.contains("c"));
    }

    #[test]
    fn transitive_successors_of_root_include_all_descendants() {
        let mut g = DependencyGraph::new();
        for name in &["a", "b", "c", "d"] {
            g.add_node(name);
        }
        g.add_edge("a", "b").unwrap();
        g.add_edge("b", "c").unwrap();
        g.add_edge("c", "d").unwrap();

        let succs = g.transitive_successors("a");
        assert_eq!(succs.len(), 3);
        assert!(succs.contains("b"));
        assert!(succs.contains("c"));
        assert!(succs.contains("d"));
    }

    #[test]
    fn transitive_queries_on_isolated_node_are_empty() {
        let mut g = DependencyGraph::new();
        g.add_node("isolated");

        assert!(g.transitive_predecessors("isolated").is_empty());
        assert!(g.transitive_successors("isolated").is_empty());
    }

    // ======================================================================
    // Root & leaf queries
    // ======================================================================

    #[test]
    fn root_and_leaf_nodes_in_linear_chain() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_node("c");
        g.add_edge("a", "b").unwrap();
        g.add_edge("b", "c").unwrap();

        assert_eq!(g.root_nodes(), vec!["a"]);
        assert_eq!(g.leaf_nodes(), vec!["c"]);
    }

    #[test]
    fn all_disconnected_nodes_are_both_roots_and_leaves() {
        let mut g = DependencyGraph::new();
        g.add_node("x");
        g.add_node("y");

        let roots = g.root_nodes();
        let leaves = g.leaf_nodes();
        assert_eq!(roots, vec!["x", "y"]);
        assert_eq!(leaves, vec!["x", "y"]);
    }

    // ======================================================================
    // Clone produces independent graph
    // ======================================================================

    #[test]
    fn clone_is_independent() {
        let mut g = DependencyGraph::new();
        g.add_node("a");
        g.add_node("b");
        g.add_edge("a", "b").unwrap();

        let mut cloned = g.clone();
        cloned.add_node("c");
        cloned.add_edge("b", "c").unwrap();

        assert_eq!(g.node_count(), 2);
        assert_eq!(g.edge_count(), 1);
        assert_eq!(cloned.node_count(), 3);
        assert_eq!(cloned.edge_count(), 2);
    }

    // ======================================================================
    // Error Display
    // ======================================================================

    #[test]
    fn error_display_messages_are_informative() {
        let cycle_err = GraphError::CycleDetected(vec!["a".into(), "b".into()]);
        assert_eq!(cycle_err.to_string(), "dependency cycle: a -> b");

        let unknown_err = GraphError::UnknownNode("phantom".into());
        assert_eq!(unknown_err.to_string(), "unknown node: phantom");

        let dup_err = GraphError::DuplicateEdge {
            from: "x".into(),
            to: "y".into(),
        };
        assert_eq!(dup_err.to_string(), "duplicate edge: x -> y");
    }

    // ======================================================================
    // Determinism: repeated sorts produce the same order
    // ======================================================================

    #[test]
    fn topological_sort_is_deterministic_across_calls() {
        let mut g = DependencyGraph::new();
        for name in &["c", "a", "b", "d"] {
            g.add_node(name);
        }
        g.add_edge("a", "c").unwrap();
        g.add_edge("b", "d").unwrap();

        let order1 = g.topological_sort().unwrap();
        let order2 = g.topological_sort().unwrap();
        assert_eq!(order1, order2, "sort must be deterministic");
    }

    // ======================================================================
    // Robotics-realistic scenario
    // ======================================================================

    #[test]
    fn robotics_pipeline_sorts_correctly() {
        // Typical robotics pipeline:
        //   imu_driver ─────────┐
        //   lidar_driver ──> fusion ──> planner ──> motor_ctrl
        //   camera_driver ──┘
        let mut g = DependencyGraph::new();
        for name in &[
            "imu_driver",
            "lidar_driver",
            "camera_driver",
            "fusion",
            "planner",
            "motor_ctrl",
        ] {
            g.add_node(name);
        }
        g.add_edge("imu_driver", "fusion").unwrap();
        g.add_edge("lidar_driver", "fusion").unwrap();
        g.add_edge("camera_driver", "fusion").unwrap();
        g.add_edge("fusion", "planner").unwrap();
        g.add_edge("planner", "motor_ctrl").unwrap();

        let order = g.topological_sort().unwrap();

        // All drivers must appear before fusion.
        let fusion_pos = order.iter().position(|n| n == "fusion").unwrap();
        for driver in &["imu_driver", "lidar_driver", "camera_driver"] {
            let driver_pos = order.iter().position(|n| n == *driver).unwrap();
            assert!(
                driver_pos < fusion_pos,
                "{} (pos {}) must come before fusion (pos {})",
                driver,
                driver_pos,
                fusion_pos
            );
        }

        // Fusion < planner < motor_ctrl.
        let planner_pos = order.iter().position(|n| n == "planner").unwrap();
        let motor_pos = order.iter().position(|n| n == "motor_ctrl").unwrap();
        assert!(fusion_pos < planner_pos);
        assert!(planner_pos < motor_pos);

        // Root nodes are the three drivers.
        let roots = g.root_nodes();
        assert_eq!(roots.len(), 3);
        assert!(roots.contains(&"imu_driver"));

        // Leaf node is motor_ctrl.
        assert_eq!(g.leaf_nodes(), vec!["motor_ctrl"]);

        // Transitive predecessors of motor_ctrl = everyone else.
        let preds = g.transitive_predecessors("motor_ctrl");
        assert_eq!(preds.len(), 5);

        // Transitive successors of lidar_driver = fusion, planner, motor_ctrl.
        let succs = g.transitive_successors("lidar_driver");
        assert_eq!(succs.len(), 3);
        assert!(succs.contains("fusion"));
        assert!(succs.contains("planner"));
        assert!(succs.contains("motor_ctrl"));
    }
}
