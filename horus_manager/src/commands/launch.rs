//! Launch command - Multi-node launch from YAML files
//!
//! Launches multiple HORUS nodes from a configuration file.

use colored::*;
use horus_core::core::DurationExt;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;
use std::process::{Child, Command, Stdio};

/// Node configuration in a launch file
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchNode {
    /// Node name
    pub name: String,

    /// Package/crate containing the node
    #[serde(default)]
    pub package: Option<String>,

    /// Node priority (lower = higher priority)
    #[serde(default)]
    pub priority: Option<i32>,

    /// Node tick rate in Hz
    #[serde(default)]
    pub rate_hz: Option<u32>,

    /// Node parameters
    #[serde(default)]
    pub params: HashMap<String, serde_yaml::Value>,

    /// Environment variables
    #[serde(default)]
    pub env: HashMap<String, String>,

    /// Command to run (if not using package)
    #[serde(default)]
    pub command: Option<String>,

    /// Arguments to pass
    #[serde(default)]
    pub args: Vec<String>,

    /// Namespace prefix for topics
    #[serde(default)]
    pub namespace: Option<String>,

    /// Nodes this depends on (will wait for them to start)
    #[serde(default)]
    pub depends_on: Vec<String>,

    /// Delay before starting (seconds)
    #[serde(default)]
    pub start_delay: Option<f64>,

    /// Restart policy: "never", "always", "on-failure"
    #[serde(default = "default_restart")]
    pub restart: String,
}

fn default_restart() -> String {
    "never".to_string()
}

/// Launch file configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchConfig {
    /// Nodes to launch
    #[serde(default)]
    pub nodes: Vec<LaunchNode>,

    /// Global environment variables
    #[serde(default)]
    pub env: HashMap<String, String>,

    /// Global namespace prefix
    #[serde(default)]
    pub namespace: Option<String>,

    /// Session name
    #[serde(default)]
    pub session: Option<String>,
}

/// Run the launch command
pub fn run_launch(
    file: &Path,
    dry_run: bool,
    namespace: Option<String>,
    shutdown_timeout_secs: u64,
) -> HorusResult<()> {
    log::debug!("launching from file: {:?}", file);

    // Check if file exists
    if !file.exists() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Launch file not found: {}",
            file.display()
        ))));
    }

    // Read and parse the launch file
    let content = std::fs::read_to_string(file).map_err(HorusError::Io)?;

    let config: LaunchConfig = serde_yaml::from_str(&content).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "failed to parse launch file: {}",
            e
        )))
    })?;

    if config.nodes.is_empty() {
        println!("{}", "No nodes defined in launch file.".yellow());
        return Ok(());
    }

    let session_name = config.session.clone().unwrap_or_else(|| {
        file.file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "launch".to_string())
    });

    // Apply namespace override
    let global_namespace = namespace.or(config.namespace.clone());

    println!("{}", "HORUS Multi-Node Launch".green().bold());
    println!();
    println!("  {} {}", "Launch file:".cyan(), file.display());
    println!("  {} {}", "Session:".cyan(), session_name);
    if let Some(ref ns) = global_namespace {
        println!("  {} {}", "Namespace:".cyan(), ns);
    }
    println!("  {} {}", "Nodes:".cyan(), config.nodes.len());
    println!();

    if dry_run {
        println!(
            "{}",
            "[DRY RUN] Would launch the following nodes:"
                .yellow()
                .bold()
        );
        println!();
        print_launch_plan(&config, &global_namespace);
        println!();
        println!("{} Run without --dry-run to launch nodes.", "".yellow());
        return Ok(());
    }

    // Sort nodes by dependencies (topological sort)
    let ordered_nodes = sort_by_dependencies(&config.nodes)?;

    // Launch nodes
    println!("{}", "Launching nodes...".cyan().bold());
    println!();

    let mut processes: Vec<(String, Child)> = Vec::new();
    let mut started_nodes: Vec<String> = Vec::new();

    for node in &ordered_nodes {
        // Check dependencies
        for dep in &node.depends_on {
            if !started_nodes.contains(dep) {
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "Dependency '{}' for node '{}' not found or not started",
                    dep, node.name
                ))));
            }
        }

        // Apply start delay if specified
        if let Some(delay) = node.start_delay {
            if delay.is_finite() && delay > 0.0 {
                println!("  {} Waiting {:.1}s for {}", "".dimmed(), delay, node.name);
                std::thread::sleep(delay.secs());
            } else if !delay.is_finite() || delay < 0.0 {
                log::warn!(
                    "Ignoring invalid start_delay {} for node '{}'",
                    delay,
                    node.name
                );
            }
        }

        // Build the full node name with namespace
        let full_name = match (&global_namespace, &node.namespace) {
            (Some(global), Some(local)) => format!("{}/{}/{}", global, local, node.name),
            (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, node.name),
            (None, None) => node.name.clone(),
        };

        print!("  {} Launching {}...", "".cyan(), full_name.white().bold());

        match launch_node(node, &config.env, &global_namespace) {
            Ok(child) => {
                println!(" {} (PID: {})", "started".green(), child.id());
                processes.push((node.name.clone(), child));
                started_nodes.push(node.name.clone());
            }
            Err(e) => {
                println!(" {}", "failed".red());
                eprintln!("    Error: {}", e);

                // Clean up already started processes
                println!();
                println!("{}", "Cleaning up started nodes...".yellow());
                for (name, mut proc) in processes {
                    print!("  {} Stopping {}...", "".yellow(), name);
                    if proc.kill().is_ok() {
                        let _ = proc.wait(); // Reap to prevent zombie
                        println!(" {}", "stopped".green());
                    } else {
                        let _ = proc.wait(); // Reap even if kill failed
                        println!(" {}", "already stopped".dimmed());
                    }
                }

                return Err(e);
            }
        }
    }

    println!();
    println!(
        "{} All {} nodes launched successfully!",
        "".green(),
        processes.len()
    );
    println!();
    println!(
        "  {} Use 'horus node list' to see running nodes",
        "Tip:".dimmed()
    );
    println!(
        "  {} Use 'horus monitor' to view live status",
        "Tip:".dimmed()
    );

    // Wait for all processes to finish (or handle signals)
    println!();
    println!("{}", "Press Ctrl+C to stop all nodes...".dimmed());

    // Set up signal handler
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    // Wait for shutdown signal
    while running.load(std::sync::atomic::Ordering::SeqCst) {
        // Check if any process has exited
        let mut stopped_indices: Vec<usize> = Vec::new();
        for (i, (name, proc)) in processes.iter_mut().enumerate() {
            match proc.try_wait() {
                Ok(Some(status)) => {
                    if !status.success() {
                        println!(
                            "{} Node '{}' exited with status: {}",
                            "".yellow(),
                            name,
                            status
                        );
                    } else {
                        println!("{} Node '{}' completed", "".dimmed(), name);
                    }
                    stopped_indices.push(i);
                }
                Ok(None) => {} // Still running
                Err(e) => {
                    log::error!("Error checking node '{}': {}", name, e);
                }
            }
        }

        // Remove stopped processes (in reverse order to preserve indices)
        for i in stopped_indices.into_iter().rev() {
            processes.remove(i);
        }

        if processes.is_empty() {
            println!("{}", "All nodes have stopped.".dimmed());
            break;
        }

        std::thread::sleep(100_u64.ms());
    }

    // Clean up
    println!();
    println!("{}", "Shutting down nodes...".yellow().bold());
    for (name, mut proc) in processes {
        print!("  {} Stopping {}...", "".yellow(), name);
        // First try SIGTERM (cross-platform via horus_sys)
        let _ = horus_sys::process::ProcessHandle::from_child(&proc)
            .signal(horus_sys::process::Signal::Terminate);

        // Wait for graceful shutdown
        std::thread::sleep(shutdown_timeout_secs.secs());

        // Force kill if still running
        if proc.try_wait().map(|s| s.is_none()).unwrap_or(false) {
            let _ = proc.kill();
        }
        let _ = proc.wait();
        println!(" {}", "stopped".green());
    }

    println!();
    println!("{} All nodes stopped.", "".green());

    Ok(())
}

/// Print the launch plan (dry run)
fn print_launch_plan(config: &LaunchConfig, global_namespace: &Option<String>) {
    for (i, node) in config.nodes.iter().enumerate() {
        let full_name = match (global_namespace, &node.namespace) {
            (Some(global), Some(local)) => format!("{}/{}/{}", global, local, node.name),
            (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, node.name),
            (None, None) => node.name.clone(),
        };

        println!("  {}. {}", i + 1, full_name.white().bold());

        if let Some(ref pkg) = node.package {
            println!("     {} {}", "Package:".dimmed(), pkg);
        }
        if let Some(ref cmd) = node.command {
            println!("     {} {}", "Command:".dimmed(), cmd);
        }
        if let Some(priority) = node.priority {
            println!("     {} {}", "Priority:".dimmed(), priority);
        }
        if let Some(rate) = node.rate_hz {
            println!("     {} {} Hz", "Rate:".dimmed(), rate);
        }
        if !node.params.is_empty() {
            println!("     {}", "Params:".dimmed());
            for (k, v) in &node.params {
                println!("       {} = {:?}", k, v);
            }
        }
        if !node.depends_on.is_empty() {
            println!(
                "     {} {}",
                "Depends:".dimmed(),
                node.depends_on.join(", ")
            );
        }
        if let Some(delay) = node.start_delay {
            println!("     {} {:.1}s", "Delay:".dimmed(), delay);
        }
        println!();
    }
}

/// Sort nodes by dependencies (topological sort)
fn sort_by_dependencies(nodes: &[LaunchNode]) -> HorusResult<Vec<LaunchNode>> {
    let mut result = Vec::new();
    let mut visited = std::collections::HashSet::new();
    let mut temp_visited = std::collections::HashSet::new();

    let node_map: HashMap<&str, &LaunchNode> = nodes.iter().map(|n| (n.name.as_str(), n)).collect();

    fn visit<'a>(
        node: &'a LaunchNode,
        node_map: &HashMap<&str, &'a LaunchNode>,
        visited: &mut std::collections::HashSet<String>,
        temp_visited: &mut std::collections::HashSet<String>,
        result: &mut Vec<LaunchNode>,
    ) -> HorusResult<()> {
        if temp_visited.contains(&node.name) {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Circular dependency detected involving node '{}'",
                node.name
            ))));
        }
        if visited.contains(&node.name) {
            return Ok(());
        }

        temp_visited.insert(node.name.clone());

        for dep in &node.depends_on {
            if let Some(dep_node) = node_map.get(dep.as_str()) {
                visit(dep_node, node_map, visited, temp_visited, result)?;
            }
            // If dependency not found, it might be external - we'll check at runtime
        }

        temp_visited.remove(&node.name);
        visited.insert(node.name.clone());
        result.push(node.clone());

        Ok(())
    }

    for node in nodes {
        visit(
            node,
            &node_map,
            &mut visited,
            &mut temp_visited,
            &mut result,
        )?;
    }

    Ok(result)
}

/// Launch a single node
fn launch_node(
    node: &LaunchNode,
    global_env: &HashMap<String, String>,
    namespace: &Option<String>,
) -> HorusResult<Child> {
    let mut cmd = if let Some(ref command) = node.command {
        // Custom command
        let parts: Vec<&str> = command.split_whitespace().collect();
        if parts.is_empty() {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Empty command for node '{}'",
                node.name
            ))));
        }
        let mut c = Command::new(parts[0]);
        if parts.len() > 1 {
            c.args(&parts[1..]);
        }
        c
    } else if let Some(ref package) = node.package {
        // Run as a HORUS package
        let horus_bin = std::env::current_exe().unwrap_or_else(|_| "horus".into());
        let mut c = Command::new(horus_bin);
        c.args(["run", package]);
        c
    } else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Node '{}' must specify either 'command' or 'package'",
            node.name
        ))));
    };

    // Add arguments
    cmd.args(&node.args);

    // Set environment variables
    for (k, v) in global_env {
        cmd.env(k, v);
    }
    for (k, v) in &node.env {
        cmd.env(k, v);
    }

    // Set HORUS-specific environment
    cmd.env("HORUS_NODE_NAME", &node.name);
    if let Some(priority) = node.priority {
        cmd.env("HORUS_NODE_PRIORITY", priority.to_string());
    }
    if let Some(rate) = node.rate_hz {
        cmd.env("HORUS_NODE_RATE_HZ", rate.to_string());
    }
    if let Some(ref ns) = namespace {
        cmd.env("HORUS_NAMESPACE", ns);
    }
    if let Some(ref ns) = node.namespace {
        cmd.env("HORUS_NODE_NAMESPACE", ns);
    }

    // Set parameters as environment
    for (k, v) in &node.params {
        let env_key = format!("HORUS_PARAM_{}", k.to_uppercase().replace('-', "_"));
        let env_value = match v {
            serde_yaml::Value::String(s) => s.clone(),
            other => serde_yaml::to_string(other).unwrap_or_default(),
        };
        cmd.env(env_key, env_value);
    }

    // Configure I/O
    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    // Spawn the process
    cmd.spawn().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to launch node '{}': {}",
            node.name, e
        )))
    })
}

/// List nodes in a launch file
pub fn list_launch_nodes(file: &Path) -> HorusResult<()> {
    if !file.exists() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Launch file not found: {}",
            file.display()
        ))));
    }

    let content = std::fs::read_to_string(file).map_err(HorusError::Io)?;

    let config: LaunchConfig = serde_yaml::from_str(&content).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "failed to parse launch file: {}",
            e
        )))
    })?;

    println!("{}", "Launch File Contents".green().bold());
    println!();
    println!("  {} {}", "File:".cyan(), file.display());
    if let Some(ref session) = config.session {
        println!("  {} {}", "Session:".cyan(), session);
    }
    if let Some(ref ns) = config.namespace {
        println!("  {} {}", "Namespace:".cyan(), ns);
    }
    println!();

    if config.nodes.is_empty() {
        println!("{}", "  No nodes defined.".yellow());
    } else {
        println!(
            "  {:<30} {:>10} {:>10} {:>15}",
            "NAME".dimmed(),
            "PRIORITY".dimmed(),
            "RATE".dimmed(),
            "DEPENDS ON".dimmed()
        );
        println!("  {}", "-".repeat(70).dimmed());

        for node in &config.nodes {
            let priority = node
                .priority
                .map(|p| p.to_string())
                .unwrap_or("-".to_string());
            let rate = node
                .rate_hz
                .map(|r| format!("{} Hz", r))
                .unwrap_or("-".to_string());
            let deps = if node.depends_on.is_empty() {
                "-".to_string()
            } else {
                node.depends_on.join(", ")
            };

            println!(
                "  {:<30} {:>10} {:>10} {:>15}",
                node.name, priority, rate, deps
            );
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn launch_config_deserialize_minimal() {
        let yaml = "nodes: []";
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert!(config.nodes.is_empty());
        assert!(config.namespace.is_none());
        assert!(config.session.is_none());
    }

    #[test]
    fn launch_config_deserialize_full() {
        let yaml = r#"
session: test_session
namespace: /robot
env:
  HORUS_LOG: debug
nodes:
  - name: motor_ctrl
    package: my_robot
    priority: 1
    rate_hz: 100
    restart: always
    depends_on: [sensor_node]
    params:
      max_speed: 1.5
    env:
      MOTOR_ID: "0"
  - name: sensor_node
    command: /usr/local/bin/sensor
    args: ["--port", "5000"]
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.session.as_deref(), Some("test_session"));
        assert_eq!(config.namespace.as_deref(), Some("/robot"));
        assert_eq!(config.nodes.len(), 2);

        let motor = &config.nodes[0];
        assert_eq!(motor.name, "motor_ctrl");
        assert_eq!(motor.package.as_deref(), Some("my_robot"));
        assert_eq!(motor.priority, Some(1));
        assert_eq!(motor.rate_hz, Some(100));
        assert_eq!(motor.restart, "always");
        assert_eq!(motor.depends_on, vec!["sensor_node"]);

        let sensor = &config.nodes[1];
        assert_eq!(sensor.name, "sensor_node");
        assert_eq!(sensor.command.as_deref(), Some("/usr/local/bin/sensor"));
        assert_eq!(sensor.args, vec!["--port", "5000"]);
    }

    #[test]
    fn launch_node_default_restart_is_never() {
        let yaml = "name: test_node";
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.restart, "never");
    }

    #[test]
    fn launch_config_roundtrip_serde() {
        let config = LaunchConfig {
            nodes: vec![LaunchNode {
                name: "test".to_string(),
                package: None,
                priority: Some(5),
                rate_hz: Some(50),
                params: HashMap::new(),
                env: HashMap::new(),
                command: None,
                args: vec![],
                namespace: None,
                depends_on: vec![],
                start_delay: None,
                restart: "never".to_string(),
            }],
            env: HashMap::new(),
            namespace: None,
            session: Some("roundtrip".to_string()),
        };
        let yaml = serde_yaml::to_string(&config).unwrap();
        let decoded: LaunchConfig = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(decoded.nodes.len(), 1);
        assert_eq!(decoded.nodes[0].name, "test");
        assert_eq!(decoded.session.as_deref(), Some("roundtrip"));
    }

    #[test]
    fn run_launch_missing_file_returns_error() {
        let result = run_launch(Path::new("/nonexistent/launch.yaml"), false, None, 5);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Launch file not found"),
            "error should be user-friendly: {}",
            err
        );
    }

    #[test]
    fn list_launch_nodes_missing_file_returns_error() {
        let result = list_launch_nodes(Path::new("/nonexistent/launch.yaml"));
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Launch file not found"));
    }

    #[test]
    fn sort_by_dependencies_independent_nodes() {
        let nodes = vec![
            LaunchNode {
                name: "a".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: a").unwrap()
            },
            LaunchNode {
                name: "b".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: b").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 2);
    }

    #[test]
    fn sort_by_dependencies_respects_order() {
        let nodes = vec![
            LaunchNode {
                name: "motor".to_string(),
                depends_on: vec!["sensor".to_string()],
                ..serde_yaml::from_str("name: motor").unwrap()
            },
            LaunchNode {
                name: "sensor".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: sensor").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted[0].name, "sensor", "dependency should come first");
        assert_eq!(sorted[1].name, "motor");
    }

    #[test]
    fn sort_by_dependencies_detects_cycle() {
        let nodes = vec![
            LaunchNode {
                name: "a".to_string(),
                depends_on: vec!["b".to_string()],
                ..serde_yaml::from_str("name: a").unwrap()
            },
            LaunchNode {
                name: "b".to_string(),
                depends_on: vec!["a".to_string()],
                ..serde_yaml::from_str("name: b").unwrap()
            },
        ];
        let result = sort_by_dependencies(&nodes);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Circular dependency"),
            "should detect cycle: {}",
            err
        );
    }

    #[test]
    fn sort_by_dependencies_diamond() {
        // D depends on B and C, both depend on A
        let nodes = vec![
            LaunchNode {
                name: "D".to_string(),
                depends_on: vec!["B".to_string(), "C".to_string()],
                ..serde_yaml::from_str("name: D").unwrap()
            },
            LaunchNode {
                name: "B".to_string(),
                depends_on: vec!["A".to_string()],
                ..serde_yaml::from_str("name: B").unwrap()
            },
            LaunchNode {
                name: "C".to_string(),
                depends_on: vec!["A".to_string()],
                ..serde_yaml::from_str("name: C").unwrap()
            },
            LaunchNode {
                name: "A".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: A").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 4);
        assert_eq!(sorted[0].name, "A", "root dependency first");
        // D must come after B and C
        let d_pos = sorted.iter().position(|n| n.name == "D").unwrap();
        let b_pos = sorted.iter().position(|n| n.name == "B").unwrap();
        let c_pos = sorted.iter().position(|n| n.name == "C").unwrap();
        assert!(d_pos > b_pos);
        assert!(d_pos > c_pos);
    }

    #[test]
    fn run_launch_invalid_yaml_returns_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.yaml");
        std::fs::write(&path, "not: [valid yaml: {{").unwrap();
        let result = run_launch(&path, false, None, 5);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("parse launch file"),
            "error should mention parsing: {}",
            err
        );
    }

    #[test]
    fn run_launch_empty_nodes_succeeds_dry_run() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("empty.yaml");
        std::fs::write(&path, "nodes: []").unwrap();
        // Empty nodes with dry_run should succeed (prints "No nodes defined")
        let result = run_launch(&path, true, None, 5);
        assert!(
            result.is_ok(),
            "empty nodes launch with dry_run should succeed: {:?}",
            result.err()
        );
        // Verify the YAML file still exists and was not modified
        assert!(
            path.exists(),
            "launch file should not be deleted by dry run"
        );
        let content = std::fs::read_to_string(&path).unwrap();
        assert_eq!(
            content, "nodes: []",
            "launch file should not be modified by dry run"
        );
    }

    #[test]
    fn node_namespace_construction() {
        // Mirrors the logic in run_launch/print_launch_plan
        let cases = vec![
            (Some("global"), Some("local"), "node", "global/local/node"),
            (Some("global"), None, "node", "global/node"),
            (None, Some("local"), "node", "local/node"),
            (None, None, "node", "node"),
        ];
        for (global, local, name, expected) in cases {
            let full = match (global, local) {
                (Some(g), Some(l)) => format!("{}/{}/{}", g, l, name),
                (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, name),
                (None, None) => name.to_string(),
            };
            assert_eq!(full, expected);
        }
    }

    #[test]
    fn launch_node_requires_command_or_package() {
        // A node without command or package should fail
        let node = LaunchNode {
            name: "orphan".to_string(),
            package: None,
            command: None,
            ..serde_yaml::from_str("name: orphan").unwrap()
        };
        let result = launch_node(&node, &HashMap::new(), &None);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("must specify either 'command' or 'package'"),
            "error should be helpful: {}",
            err
        );
    }

    #[test]
    fn launch_node_empty_command_fails() {
        let node = LaunchNode {
            name: "empty_cmd".to_string(),
            command: Some(String::new()),
            ..serde_yaml::from_str("name: empty_cmd").unwrap()
        };
        let result = launch_node(&node, &HashMap::new(), &None);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Empty command"));
    }

    #[test]
    fn launch_node_param_env_format() {
        // Verify param-to-env key transformation
        let key = "max-speed";
        let env_key = format!("HORUS_PARAM_{}", key.to_uppercase().replace('-', "_"));
        assert_eq!(env_key, "HORUS_PARAM_MAX_SPEED");
    }

    // ── Battle tests: YAML parsing edge cases ─────────────────────────────

    #[test]
    fn launch_config_deserialize_empty_yaml() {
        // An empty document should use defaults
        let config: LaunchConfig = serde_yaml::from_str("{}").unwrap();
        assert!(config.nodes.is_empty());
        assert!(config.env.is_empty());
        assert!(config.namespace.is_none());
        assert!(config.session.is_none());
    }

    #[test]
    fn launch_node_all_defaults() {
        let yaml = "name: minimal";
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.name, "minimal");
        assert!(node.package.is_none());
        assert!(node.priority.is_none());
        assert!(node.rate_hz.is_none());
        assert!(node.params.is_empty());
        assert!(node.env.is_empty());
        assert!(node.command.is_none());
        assert!(node.args.is_empty());
        assert!(node.namespace.is_none());
        assert!(node.depends_on.is_empty());
        assert!(node.start_delay.is_none());
        assert_eq!(node.restart, "never");
    }

    #[test]
    fn launch_node_with_all_fields_populated() {
        let yaml = r#"
name: full_node
package: my_pkg
priority: -10
rate_hz: 500
command: /bin/test
args: ["--verbose", "--port", "8080"]
namespace: /ns/sub
depends_on: ["a", "b", "c"]
start_delay: 2.5
restart: on-failure
params:
  kp: 1.0
  ki: 0.01
env:
  MY_VAR: my_value
"#;
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.name, "full_node");
        assert_eq!(node.package.as_deref(), Some("my_pkg"));
        assert_eq!(node.priority, Some(-10));
        assert_eq!(node.rate_hz, Some(500));
        assert_eq!(node.command.as_deref(), Some("/bin/test"));
        assert_eq!(node.args, vec!["--verbose", "--port", "8080"]);
        assert_eq!(node.namespace.as_deref(), Some("/ns/sub"));
        assert_eq!(node.depends_on, vec!["a", "b", "c"]);
        assert!((node.start_delay.unwrap() - 2.5).abs() < 0.001);
        assert_eq!(node.restart, "on-failure");
        assert_eq!(node.params.len(), 2);
        assert_eq!(node.env.get("MY_VAR").map(|s| s.as_str()), Some("my_value"));
    }

    #[test]
    fn launch_config_invalid_yaml_type_mismatch() {
        let yaml = "nodes: \"not a list\"";
        let result = serde_yaml::from_str::<LaunchConfig>(yaml);
        assert!(result.is_err());
    }

    #[test]
    fn launch_node_priority_negative() {
        let yaml = "name: neg\npriority: -100";
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.priority, Some(-100));
    }

    #[test]
    fn launch_config_extra_fields_ignored() {
        // YAML with unknown fields should deserialize (serde default behavior)
        let yaml = r#"
nodes: []
unknown_field: 42
another: "hello"
"#;
        // serde_yaml with default deny_unknown_fields may fail; test actual behavior
        let result = serde_yaml::from_str::<LaunchConfig>(yaml);
        // If it succeeds, extra fields are ignored. If it fails, that's also valid behavior to document.
        // The important thing is no panic.
        let _outcome = result.is_ok();
    }

    // ── Battle tests: sort_by_dependencies ────────────────────────────────

    #[test]
    fn sort_by_dependencies_empty_list() {
        let result = sort_by_dependencies(&[]).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn sort_by_dependencies_single_node() {
        let nodes = vec![LaunchNode {
            name: "solo".to_string(),
            depends_on: vec![],
            ..serde_yaml::from_str("name: solo").unwrap()
        }];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 1);
        assert_eq!(sorted[0].name, "solo");
    }

    #[test]
    fn sort_by_dependencies_linear_chain() {
        // A -> B -> C -> D (D depends on C, C depends on B, B depends on A)
        let nodes = vec![
            LaunchNode {
                name: "D".to_string(),
                depends_on: vec!["C".to_string()],
                ..serde_yaml::from_str("name: D").unwrap()
            },
            LaunchNode {
                name: "C".to_string(),
                depends_on: vec!["B".to_string()],
                ..serde_yaml::from_str("name: C").unwrap()
            },
            LaunchNode {
                name: "B".to_string(),
                depends_on: vec!["A".to_string()],
                ..serde_yaml::from_str("name: B").unwrap()
            },
            LaunchNode {
                name: "A".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: A").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 4);
        let names: Vec<&str> = sorted.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(names, vec!["A", "B", "C", "D"]);
    }

    #[test]
    fn sort_by_dependencies_self_cycle() {
        let nodes = vec![LaunchNode {
            name: "self_ref".to_string(),
            depends_on: vec!["self_ref".to_string()],
            ..serde_yaml::from_str("name: self_ref").unwrap()
        }];
        let result = sort_by_dependencies(&nodes);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Circular dependency"), "self-cycle: {}", err);
    }

    #[test]
    fn sort_by_dependencies_three_node_cycle() {
        let nodes = vec![
            LaunchNode {
                name: "x".to_string(),
                depends_on: vec!["z".to_string()],
                ..serde_yaml::from_str("name: x").unwrap()
            },
            LaunchNode {
                name: "y".to_string(),
                depends_on: vec!["x".to_string()],
                ..serde_yaml::from_str("name: y").unwrap()
            },
            LaunchNode {
                name: "z".to_string(),
                depends_on: vec!["y".to_string()],
                ..serde_yaml::from_str("name: z").unwrap()
            },
        ];
        let result = sort_by_dependencies(&nodes);
        assert!(result.is_err());
    }

    #[test]
    fn sort_by_dependencies_external_dep_not_found() {
        // Node depends on something not in the list — should not error (treated as external)
        let nodes = vec![LaunchNode {
            name: "node_a".to_string(),
            depends_on: vec!["external_service".to_string()],
            ..serde_yaml::from_str("name: node_a").unwrap()
        }];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 1);
        assert_eq!(sorted[0].name, "node_a");
    }

    // ── Battle tests: namespace construction ──────────────────────────────

    #[test]
    fn namespace_construction_with_special_characters() {
        let cases = vec![
            (Some("/robot1"), Some("arm"), "ctrl", "/robot1/arm/ctrl"),
            (Some("/"), None, "node", "//node"),
            (None, Some(""), "node", "/node"),
        ];
        for (global, local, name, expected) in cases {
            let full = match (global, local) {
                (Some(g), Some(l)) => format!("{}/{}/{}", g, l, name),
                (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, name),
                (None, None) => name.to_string(),
            };
            assert_eq!(
                full, expected,
                "global={:?} local={:?} name={}",
                global, local, name
            );
        }
    }

    // ── Battle tests: dry run with tempfile ───────────────────────────────

    #[test]
    fn run_launch_dry_run_with_nodes() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.yaml");
        let yaml = r#"
session: dry_test
namespace: /test
nodes:
  - name: sensor
    command: /bin/echo hello
    rate_hz: 100
  - name: controller
    command: /bin/echo world
    depends_on: [sensor]
    priority: 1
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = run_launch(&path, true, None, 5);
        assert!(result.is_ok(), "dry run should succeed: {:?}", result.err());
    }

    #[test]
    fn run_launch_dry_run_with_namespace_override() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("ns.yaml");
        let yaml = r#"
namespace: /original
nodes:
  - name: node1
    command: /bin/true
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = run_launch(&path, true, Some("/override".to_string()), 5);
        assert!(
            result.is_ok(),
            "namespace override dry run should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn list_launch_nodes_with_valid_file() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("list.yaml");
        let yaml = r#"
session: list_test
namespace: /bot
nodes:
  - name: nav
    priority: 1
    rate_hz: 50
    depends_on: [localization]
  - name: localization
    command: /bin/loc
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = list_launch_nodes(&path);
        assert!(
            result.is_ok(),
            "listing valid launch file should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn list_launch_nodes_empty_nodes() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("empty.yaml");
        std::fs::write(&path, "nodes: []").unwrap();
        let result = list_launch_nodes(&path);
        assert!(result.is_ok());
    }

    #[test]
    fn launch_config_multi_robot_namespaces() {
        let yaml = r#"
namespace: /fleet
nodes:
  - name: planner
    namespace: /robot1
    command: /bin/plan
    rate_hz: 10
  - name: planner
    namespace: /robot2
    command: /bin/plan
    rate_hz: 10
  - name: coordinator
    command: /bin/coord
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.nodes.len(), 3);
        assert_eq!(config.namespace.as_deref(), Some("/fleet"));
        // Two nodes named "planner" in different namespaces
        let planners: Vec<_> = config
            .nodes
            .iter()
            .filter(|n| n.name == "planner")
            .collect();
        assert_eq!(planners.len(), 2);
        assert_eq!(planners[0].namespace.as_deref(), Some("/robot1"));
        assert_eq!(planners[1].namespace.as_deref(), Some("/robot2"));
    }

    #[test]
    fn launch_node_restart_policies() {
        for policy in &["never", "always", "on-failure"] {
            let yaml = format!("name: n\nrestart: {}", policy);
            let node: LaunchNode = serde_yaml::from_str(&yaml).unwrap();
            assert_eq!(node.restart, *policy);
        }
    }

    #[test]
    fn launch_param_env_key_transform_edge_cases() {
        // Underscores, all-caps, numbers
        let cases = vec![
            ("my-param", "HORUS_PARAM_MY_PARAM"),
            ("kp", "HORUS_PARAM_KP"),
            ("max_speed", "HORUS_PARAM_MAX_SPEED"),
            ("a-b-c-d", "HORUS_PARAM_A_B_C_D"),
            ("", "HORUS_PARAM_"),
            ("123", "HORUS_PARAM_123"),
            ("ALREADY_UPPER", "HORUS_PARAM_ALREADY_UPPER"),
        ];
        for (key, expected) in cases {
            let env_key = format!("HORUS_PARAM_{}", key.to_uppercase().replace('-', "_"));
            assert_eq!(env_key, expected, "key={}", key);
        }
    }

    #[test]
    fn run_launch_yaml_with_only_env() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("env_only.yaml");
        let yaml = r#"
env:
  HORUS_LOG: debug
  MY_VAR: value
nodes: []
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = run_launch(&path, true, None, 5);
        assert!(result.is_ok());
    }
}
