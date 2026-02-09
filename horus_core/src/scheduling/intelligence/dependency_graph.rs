use std::collections::{HashMap, HashSet, VecDeque};

/// Represents a dependency graph for nodes based on pub/sub relationships
#[derive(Debug, Clone)]
pub struct DependencyGraph {
    /// All node names in the graph
    pub nodes: Vec<String>,
    /// Directed edges (publisher -> subscriber)
    pub edges: Vec<(String, String)>,
    /// Topologically sorted levels (nodes at same level can run in parallel)
    pub levels: Vec<Vec<String>>,
    /// Groups of nodes that can execute in parallel
    pub parallel_groups: Vec<Vec<String>>,
}

impl DependencyGraph {
    /// Build dependency graph from node pub/sub metadata
    /// Uses Kahn's algorithm for topological sort to identify parallel execution groups
    pub fn from_nodes(nodes: &[(&str, Vec<String>, Vec<String>)]) -> Self {
        let mut graph = DependencyGraph {
            nodes: Vec::new(),
            edges: Vec::new(),
            levels: Vec::new(),
            parallel_groups: Vec::new(),
        };

        // Build node list
        let node_names: Vec<String> = nodes.iter().map(|(name, _, _)| name.to_string()).collect();
        graph.nodes = node_names.clone();

        // Build edges: topic -> (publishers, subscribers)
        let mut topic_map: HashMap<String, (Vec<String>, Vec<String>)> = HashMap::new();

        for (node_name, pubs, subs) in nodes {
            let name = node_name.to_string();

            // Register publishers
            for topic in pubs {
                topic_map
                    .entry(topic.clone())
                    .or_insert_with(|| (Vec::new(), Vec::new()))
                    .0
                    .push(name.clone());
            }

            // Register subscribers
            for topic in subs {
                topic_map
                    .entry(topic.clone())
                    .or_insert_with(|| (Vec::new(), Vec::new()))
                    .1
                    .push(name.clone());
            }
        }

        // Create edges: for each topic, connect all publishers to all subscribers
        for (_topic, (publishers, subscribers)) in topic_map.iter() {
            for pub_node in publishers {
                for sub_node in subscribers {
                    if pub_node != sub_node {
                        graph.edges.push((pub_node.clone(), sub_node.clone()));
                    }
                }
            }
        }

        // Compute topological levels using Kahn's algorithm
        graph.compute_levels();

        // Identify parallel groups (nodes with no dependencies between them)
        graph.find_parallel_groups();

        graph
    }

    /// Compute topological levels using Kahn's algorithm
    /// Nodes at the same level have no dependencies on each other
    fn compute_levels(&mut self) {
        let mut in_degree: HashMap<String, usize> = HashMap::new();
        let mut adjacency: HashMap<String, Vec<String>> = HashMap::new();

        // Initialize in-degree and adjacency list
        for node in &self.nodes {
            in_degree.insert(node.clone(), 0);
            adjacency.insert(node.clone(), Vec::new());
        }

        // Build adjacency list and in-degree count
        for (from, to) in &self.edges {
            adjacency.get_mut(from).unwrap().push(to.clone());
            *in_degree.get_mut(to).unwrap() += 1;
        }

        let mut queue: VecDeque<String> = VecDeque::new();
        let mut levels: Vec<Vec<String>> = Vec::new();

        // Start with nodes that have no dependencies (in-degree = 0)
        for (node, &degree) in &in_degree {
            if degree == 0 {
                queue.push_back(node.clone());
            }
        }

        // Process nodes level by level
        while !queue.is_empty() {
            let level_size = queue.len();
            let mut current_level = Vec::new();

            for _ in 0..level_size {
                if let Some(node) = queue.pop_front() {
                    current_level.push(node.clone());

                    // Reduce in-degree for neighbors
                    if let Some(neighbors) = adjacency.get(&node) {
                        for neighbor in neighbors {
                            let degree = in_degree.get_mut(neighbor).unwrap();
                            *degree -= 1;
                            if *degree == 0 {
                                queue.push_back(neighbor.clone());
                            }
                        }
                    }
                }
            }

            if !current_level.is_empty() {
                levels.push(current_level);
            }
        }

        self.levels = levels;
    }

    /// Find groups of nodes that can execute in parallel
    /// Two nodes can be in the same group if there's no path between them
    fn find_parallel_groups(&mut self) {
        let mut groups: Vec<Vec<String>> = Vec::new();

        // Build reachability map (which nodes can reach which)
        let reachability = self.build_reachability_map();

        // Group nodes that don't have dependencies on each other
        let mut remaining: HashSet<String> = self.nodes.iter().cloned().collect();

        while !remaining.is_empty() {
            let mut group: Vec<String> = Vec::new();
            let mut to_remove = Vec::new();

            for node in &remaining {
                // Check if this node can be added to current group
                let mut can_add = true;
                for group_member in &group {
                    // Check if there's a path in either direction
                    if self.has_path(&reachability, node, group_member)
                        || self.has_path(&reachability, group_member, node)
                    {
                        can_add = false;
                        break;
                    }
                }

                if can_add {
                    group.push(node.clone());
                    to_remove.push(node.clone());
                }
            }

            for node in to_remove {
                remaining.remove(&node);
            }

            if !group.is_empty() {
                groups.push(group);
            }
        }

        self.parallel_groups = groups;
    }

    /// Build reachability map using BFS from each node
    fn build_reachability_map(&self) -> HashMap<String, HashSet<String>> {
        let mut adjacency: HashMap<String, Vec<String>> = HashMap::new();

        // Build adjacency list
        for node in &self.nodes {
            adjacency.insert(node.clone(), Vec::new());
        }
        for (from, to) in &self.edges {
            adjacency.get_mut(from).unwrap().push(to.clone());
        }

        let mut reachability: HashMap<String, HashSet<String>> = HashMap::new();

        // For each node, find all reachable nodes using BFS
        for start_node in &self.nodes {
            let mut visited = HashSet::new();
            let mut queue = VecDeque::new();
            queue.push_back(start_node.clone());

            while let Some(node) = queue.pop_front() {
                if visited.contains(&node) {
                    continue;
                }
                visited.insert(node.clone());

                if let Some(neighbors) = adjacency.get(&node) {
                    for neighbor in neighbors {
                        if !visited.contains(neighbor) {
                            queue.push_back(neighbor.clone());
                        }
                    }
                }
            }

            visited.remove(start_node); // Don't include self
            reachability.insert(start_node.clone(), visited);
        }

        reachability
    }

    /// Check if there's a path from 'from' to 'to' node
    fn has_path(
        &self,
        reachability: &HashMap<String, HashSet<String>>,
        from: &str,
        to: &str,
    ) -> bool {
        reachability
            .get(from)
            .is_some_and(|reachable| reachable.contains(to))
    }

    /// Check if graph has cycles (should be acyclic for valid pub/sub)
    pub fn has_cycles(&self) -> bool {
        let total_nodes_in_levels: usize = self.levels.iter().map(|l| l.len()).sum();
        total_nodes_in_levels != self.nodes.len()
    }
}


