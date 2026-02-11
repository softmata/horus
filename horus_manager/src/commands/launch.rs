//! Launch command - Multi-node launch from YAML files
//!
//! Launches multiple HORUS nodes from a configuration file.

use colored::*;
use horus_core::error::{HorusError, HorusResult};
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
pub fn run_launch(file: &Path, dry_run: bool, namespace: Option<String>) -> HorusResult<()> {
    // Check if file exists
    if !file.exists() {
        return Err(HorusError::Config(format!(
            "Launch file not found: {}",
            file.display()
        )));
    }

    // Read and parse the launch file
    let content = std::fs::read_to_string(file).map_err(HorusError::Io)?;

    let config: LaunchConfig = serde_yaml::from_str(&content)
        .map_err(|e| HorusError::Config(format!("Failed to parse launch file: {}", e)))?;

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
                return Err(HorusError::Config(format!(
                    "Dependency '{}' for node '{}' not found or not started",
                    dep, node.name
                )));
            }
        }

        // Apply start delay if specified
        if let Some(delay) = node.start_delay {
            if delay > 0.0 {
                println!("  {} Waiting {:.1}s for {}", "".dimmed(), delay, node.name);
                std::thread::sleep(std::time::Duration::from_secs_f64(delay));
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
                        println!(" {}", "stopped".green());
                    } else {
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
                    eprintln!("Error checking node '{}': {}", name, e);
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

        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // Clean up
    println!();
    println!("{}", "Shutting down nodes...".yellow().bold());
    for (name, mut proc) in processes {
        print!("  {} Stopping {}...", "".yellow(), name);
        // First try SIGTERM
        #[cfg(unix)]
        // SAFETY: proc.id() is a valid PID of a child process we spawned. SIGTERM requests graceful shutdown.
        unsafe {
            libc::kill(proc.id() as i32, libc::SIGTERM);
        }
        #[cfg(not(unix))]
        {
            let _ = proc.kill();
        }

        // Wait a bit for graceful shutdown
        std::thread::sleep(std::time::Duration::from_millis(500));

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
            return Err(HorusError::Config(format!(
                "Circular dependency detected involving node '{}'",
                node.name
            )));
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
            return Err(HorusError::Config(format!(
                "Empty command for node '{}'",
                node.name
            )));
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
        return Err(HorusError::Config(format!(
            "Node '{}' must specify either 'command' or 'package'",
            node.name
        )));
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
    cmd.spawn()
        .map_err(|e| HorusError::Config(format!("Failed to launch node '{}': {}", node.name, e)))
}

/// List nodes in a launch file
pub fn list_launch_nodes(file: &Path) -> HorusResult<()> {
    if !file.exists() {
        return Err(HorusError::Config(format!(
            "Launch file not found: {}",
            file.display()
        )));
    }

    let content = std::fs::read_to_string(file).map_err(HorusError::Io)?;

    let config: LaunchConfig = serde_yaml::from_str(&content)
        .map_err(|e| HorusError::Config(format!("Failed to parse launch file: {}", e)))?;

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
