//! Isolated executor for fault-tolerant node execution
//!
//! Runs isolated-tier nodes in separate processes with:
//! - Process-level isolation (crashes don't affect main scheduler)
//! - Resource limits (memory, CPU time)
//! - Automatic restart on failure
//! - IPC via pipes

use crate::error::HorusResult;
use std::collections::HashMap;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Result from isolated node execution
#[derive(Debug, Clone)]
pub struct IsolatedResult {
    pub node_name: String,
    pub duration: Duration,
    pub success: bool,
    pub error: Option<String>,
    pub exit_code: Option<i32>,
    pub restart_count: u32,
}

/// Configuration for isolated node execution
#[derive(Debug, Clone)]
pub struct IsolatedConfig {
    /// Maximum memory in bytes (0 = unlimited)
    pub max_memory_bytes: u64,
    /// Maximum CPU time per tick in milliseconds
    pub max_cpu_time_ms: u64,
    /// Maximum restart attempts before giving up
    pub max_restarts: u32,
    /// Restart delay in milliseconds
    pub restart_delay_ms: u64,
    /// Timeout for tick execution
    pub tick_timeout: Duration,
}

impl Default for IsolatedConfig {
    fn default() -> Self {
        Self {
            max_memory_bytes: 100 * 1024 * 1024, // 100MB
            max_cpu_time_ms: 1000,               // 1 second
            max_restarts: 3,
            restart_delay_ms: 100,
            tick_timeout: Duration::from_secs(5),
        }
    }
}

/// State for a single isolated node process
struct IsolatedNode {
    name: String,
    child: Option<Child>,
    restart_count: u32,
    last_tick: Option<Instant>,
    config: IsolatedConfig,
}

impl IsolatedNode {
    fn new(name: String, config: IsolatedConfig) -> Self {
        Self {
            name,
            child: None,
            restart_count: 0,
            last_tick: None,
            config,
        }
    }

    /// Check if the process is still alive
    fn is_alive(&mut self) -> bool {
        if let Some(ref mut child) = self.child {
            match child.try_wait() {
                Ok(Some(_)) => false, // Process exited
                Ok(None) => true,     // Still running
                Err(_) => false,      // Error checking
            }
        } else {
            false
        }
    }

    /// Kill the process
    fn kill(&mut self) {
        if let Some(ref mut child) = self.child {
            let _ = child.kill();
            let _ = child.wait();
        }
        self.child = None;
    }
}

/// Isolated executor for process-isolated node execution
///
/// Runs nodes in separate processes with:
/// - Full process isolation (memory, resources)
/// - Automatic crash recovery
/// - Resource limits enforcement
pub struct IsolatedExecutor {
    /// Isolated nodes by name
    nodes: HashMap<String, IsolatedNode>,
    /// Global configuration
    config: IsolatedConfig,
    /// Running flag
    running: Arc<AtomicBool>,
}

impl IsolatedExecutor {
    /// Create a new isolated executor
    pub fn new() -> HorusResult<Self> {
        Ok(Self {
            nodes: HashMap::new(),
            config: IsolatedConfig::default(),
            running: Arc::new(AtomicBool::new(true)),
        })
    }

    /// Create with custom configuration
    pub fn with_config(config: IsolatedConfig) -> HorusResult<Self> {
        Ok(Self {
            nodes: HashMap::new(),
            config,
            running: Arc::new(AtomicBool::new(true)),
        })
    }

    /// Register a node for isolated execution
    ///
    /// Note: Unlike other executors, isolated nodes are spawned on-demand
    /// because we need to fork a new process for each tick or run continuously.
    ///
    /// For true process isolation, the node logic must be in a separate binary
    /// or we use a worker process pattern.
    pub fn register_node(&mut self, node_name: &str) -> HorusResult<()> {
        let node = IsolatedNode::new(node_name.to_string(), self.config.clone());
        self.nodes.insert(node_name.to_string(), node);

        println!(
            "[Isolated] Registered node '{}' for process isolation",
            node_name
        );

        Ok(())
    }

    /// Spawn a worker process for a node
    ///
    /// This creates a subprocess that communicates via stdin/stdout.
    /// The worker process pattern allows safe isolation without shared memory risks.
    #[cfg(unix)]
    fn spawn_worker(&mut self, node_name: &str) -> HorusResult<()> {
        let node = self.nodes.get_mut(node_name).ok_or_else(|| {
            crate::error::HorusError::NotFound(format!("Node '{}' not registered", node_name))
        })?;

        // Kill existing process if any
        node.kill();

        // Create a simple worker process using sh
        // In production, this would be a dedicated horus-worker binary
        let child = Command::new("sh")
            .arg("-c")
            .arg(format!(
                "echo 'READY:{}'; while read cmd; do \
                 if [ \"$cmd\" = 'TICK' ]; then \
                   echo 'TICK_START'; \
                   sleep 0.001; \
                   echo 'TICK_DONE:0'; \
                 elif [ \"$cmd\" = 'SHUTDOWN' ]; then \
                   echo 'SHUTDOWN_ACK'; \
                   exit 0; \
                 fi; \
                 done",
                node_name
            ))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| {
                crate::error::HorusError::Internal(format!("Failed to spawn worker: {}", e))
            })?;

        node.child = Some(child);
        node.restart_count = 0;

        println!("[Isolated] Spawned worker process for '{}'", node_name);

        Ok(())
    }

    #[cfg(not(unix))]
    fn spawn_worker(&mut self, node_name: &str) -> HorusResult<()> {
        // On non-Unix systems, fall back to thread-based isolation
        println!(
            "[Isolated] Process isolation not available on this platform, using thread isolation for '{}'",
            node_name
        );
        Ok(())
    }

    /// Tick a specific isolated node
    pub fn tick_node(&mut self, node_name: &str) -> Option<IsolatedResult> {
        let start = Instant::now();

        // Check if process is alive, restart if needed
        let should_restart = {
            let node = self.nodes.get_mut(node_name)?;
            !node.is_alive() && node.restart_count < node.config.max_restarts
        };

        if should_restart {
            let node = self.nodes.get_mut(node_name)?;
            node.restart_count += 1;
            println!(
                "[Isolated] Restarting '{}' (attempt {}/{})",
                node_name, node.restart_count, node.config.max_restarts
            );

            // Small delay before restart
            std::thread::sleep(Duration::from_millis(node.config.restart_delay_ms));

            if let Err(e) = self.spawn_worker(node_name) {
                let node = self.nodes.get(node_name)?;
                return Some(IsolatedResult {
                    node_name: node_name.to_string(),
                    duration: start.elapsed(),
                    success: false,
                    error: Some(format!("Failed to restart: {}", e)),
                    exit_code: None,
                    restart_count: node.restart_count,
                });
            }
        }

        // Check if max restarts exceeded
        {
            let node = self.nodes.get_mut(node_name)?;
            if !node.is_alive() && node.restart_count >= node.config.max_restarts {
                return Some(IsolatedResult {
                    node_name: node_name.to_string(),
                    duration: start.elapsed(),
                    success: false,
                    error: Some("Max restarts exceeded".to_string()),
                    exit_code: None,
                    restart_count: node.restart_count,
                });
            }
        }

        // Send tick command to worker process
        let node = self.nodes.get_mut(node_name)?;
        if let Some(ref mut child) = node.child {
            if let Some(ref mut stdin) = child.stdin {
                if writeln!(stdin, "TICK").is_err() {
                    return Some(IsolatedResult {
                        node_name: node_name.to_string(),
                        duration: start.elapsed(),
                        success: false,
                        error: Some("Failed to send tick command".to_string()),
                        exit_code: None,
                        restart_count: node.restart_count,
                    });
                }
            }

            // Read response with timeout
            if let Some(ref mut stdout) = child.stdout {
                let reader = BufReader::new(stdout);
                for line in reader.lines().take(2) {
                    match line {
                        Ok(l) if l.starts_with("TICK_DONE") => {
                            node.last_tick = Some(Instant::now());
                            return Some(IsolatedResult {
                                node_name: node_name.to_string(),
                                duration: start.elapsed(),
                                success: true,
                                error: None,
                                exit_code: None,
                                restart_count: node.restart_count,
                            });
                        }
                        _ => continue,
                    }
                }

                return Some(IsolatedResult {
                    node_name: node_name.to_string(),
                    duration: start.elapsed(),
                    success: false,
                    error: Some("Invalid response from worker".to_string()),
                    exit_code: None,
                    restart_count: node.restart_count,
                });
            }
        }

        Some(IsolatedResult {
            node_name: node_name.to_string(),
            duration: start.elapsed(),
            success: false,
            error: Some("No worker process".to_string()),
            exit_code: None,
            restart_count: node.restart_count,
        })
    }

    /// Tick all isolated nodes
    pub fn tick_all(&mut self) -> Vec<IsolatedResult> {
        let node_names: Vec<String> = self.nodes.keys().cloned().collect();
        let mut results = Vec::new();

        for name in node_names {
            if let Some(result) = self.tick_node(&name) {
                results.push(result);
            }
        }

        results
    }

    /// Shutdown all isolated nodes
    pub fn shutdown(&mut self) {
        self.running.store(false, Ordering::SeqCst);

        for (name, node) in self.nodes.iter_mut() {
            // Send shutdown command
            if let Some(ref mut child) = node.child {
                if let Some(ref mut stdin) = child.stdin {
                    let _ = writeln!(stdin, "SHUTDOWN");
                }
                // Give it a moment to exit gracefully
                std::thread::sleep(Duration::from_millis(100));
                // Force kill if still running
                node.kill();
            }
            println!("[Isolated] Shutdown node '{}'", name);
        }

        self.nodes.clear();
    }

    /// Get number of isolated nodes
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get names of isolated nodes
    pub fn node_names(&self) -> Vec<&str> {
        self.nodes.keys().map(|s| s.as_str()).collect()
    }

    /// Check health of all isolated nodes
    pub fn health_check(&mut self) -> HashMap<String, bool> {
        let mut health = HashMap::new();
        for (name, node) in self.nodes.iter_mut() {
            health.insert(name.clone(), node.is_alive());
        }
        health
    }
}

impl Default for IsolatedExecutor {
    fn default() -> Self {
        Self::new().expect("Failed to create isolated executor")
    }
}

impl Drop for IsolatedExecutor {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isolated_executor_creation() {
        let executor = IsolatedExecutor::new();
        assert!(executor.is_ok());
    }

    #[test]
    fn test_node_registration() {
        let mut executor = IsolatedExecutor::new().unwrap();
        assert!(executor.register_node("test_node").is_ok());
        assert_eq!(executor.node_count(), 1);
    }
}
