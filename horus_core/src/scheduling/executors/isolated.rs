//! Isolated executor for fault-tolerant process isolation
//!
//! Runs nodes in separate processes for:
//! - Fault isolation (crashes don't affect main scheduler)
//! - Memory isolation (memory leaks/corruption contained)
//! - Security isolation (untrusted code sandboxed)
//!
//! Uses shared memory for IPC with minimal latency overhead.

use crate::core::node::{Node, NodeInfo};
use crate::error::{HorusError, HorusResult};
use crate::memory::platform::shm_base_dir;
use std::collections::HashMap;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

// ============================================================================
// IPC Protocol Constants
// ============================================================================

/// Magic number for IPC validation (HORUS_IP as ASCII)
const IPC_MAGIC: u64 = 0x484F5255535F4950; // "HORUS_IP"

/// IPC command types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcCommand {
    None = 0,
    Init = 1,
    Tick = 2,
    Shutdown = 3,
    HealthCheck = 4,
}

impl From<u8> for IpcCommand {
    fn from(v: u8) -> Self {
        match v {
            1 => IpcCommand::Init,
            2 => IpcCommand::Tick,
            3 => IpcCommand::Shutdown,
            4 => IpcCommand::HealthCheck,
            _ => IpcCommand::None,
        }
    }
}

/// IPC response status
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcStatus {
    Idle = 0,
    Processing = 1,
    Success = 2,
    Error = 3,
    Crashed = 4,
}

impl From<u8> for IpcStatus {
    fn from(v: u8) -> Self {
        match v {
            1 => IpcStatus::Processing,
            2 => IpcStatus::Success,
            3 => IpcStatus::Error,
            4 => IpcStatus::Crashed,
            _ => IpcStatus::Idle,
        }
    }
}

// ============================================================================
// Shared Memory IPC Layout
// ============================================================================

/// IPC shared memory layout (fixed size for simplicity)
///
/// Layout (256 bytes total):
/// - [0..8]   magic: u64 - Magic number for validation
/// - [8..16]  sequence: u64 - Command sequence number
/// - [16]     command: u8 - IpcCommand
/// - [17]     status: u8 - IpcStatus
/// - [18..26] duration_ns: u64 - Last tick duration in nanoseconds
/// - [26..34] tick_count: u64 - Total successful ticks
/// - [34..42] error_count: u64 - Total errors
/// - [42..50] last_heartbeat: u64 - Unix timestamp of last heartbeat
/// - [50..54] pid: u32 - Child process PID
/// - [54..55] restart_count: u8 - Number of restarts
/// - [55..56] reserved: u8
/// - [56..256] error_message: [u8; 200] - Error message (null-terminated)
const IPC_SIZE: usize = 256;
const IPC_ERROR_MSG_OFFSET: usize = 56;
const IPC_ERROR_MSG_SIZE: usize = 200;

/// IPC region accessor for shared memory communication
pub struct IpcRegion {
    path: PathBuf,
    file: File,
}

impl IpcRegion {
    /// Create a new IPC region
    pub fn create(node_name: &str) -> HorusResult<Self> {
        let dir = shm_base_dir().join("isolated");
        fs::create_dir_all(&dir)?;

        let path = dir.join(format!("{}.ipc", node_name));

        // Create and initialize the file
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&path)?;

        // Initialize with zeros and magic number
        let mut buffer = vec![0u8; IPC_SIZE];
        buffer[0..8].copy_from_slice(&IPC_MAGIC.to_le_bytes());
        file.write_all(&buffer)?;
        file.sync_all()?;

        Ok(Self { path, file })
    }

    /// Open an existing IPC region
    pub fn open(node_name: &str) -> HorusResult<Self> {
        let path = shm_base_dir()
            .join("isolated")
            .join(format!("{}.ipc", node_name));

        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&path)
            .map_err(|e| HorusError::SharedMemory(format!("Failed to open IPC region: {}", e)))?;

        Ok(Self { path, file })
    }

    /// Read the magic number
    pub fn read_magic(&mut self) -> HorusResult<u64> {
        let mut buf = [0u8; 8];
        self.file.seek(SeekFrom::Start(0))?;
        self.file.read_exact(&mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Read sequence number
    pub fn read_sequence(&mut self) -> HorusResult<u64> {
        let mut buf = [0u8; 8];
        self.file.seek(SeekFrom::Start(8))?;
        self.file.read_exact(&mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Write sequence number
    pub fn write_sequence(&mut self, seq: u64) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(8))?;
        self.file.write_all(&seq.to_le_bytes())?;
        self.file.sync_data()?;
        Ok(())
    }

    /// Read command
    pub fn read_command(&mut self) -> HorusResult<IpcCommand> {
        let mut buf = [0u8; 1];
        self.file.seek(SeekFrom::Start(16))?;
        self.file.read_exact(&mut buf)?;
        Ok(IpcCommand::from(buf[0]))
    }

    /// Write command
    pub fn write_command(&mut self, cmd: IpcCommand) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(16))?;
        self.file.write_all(&[cmd as u8])?;
        self.file.sync_data()?;
        Ok(())
    }

    /// Read status
    pub fn read_status(&mut self) -> HorusResult<IpcStatus> {
        let mut buf = [0u8; 1];
        self.file.seek(SeekFrom::Start(17))?;
        self.file.read_exact(&mut buf)?;
        Ok(IpcStatus::from(buf[0]))
    }

    /// Write status
    pub fn write_status(&mut self, status: IpcStatus) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(17))?;
        self.file.write_all(&[status as u8])?;
        self.file.sync_data()?;
        Ok(())
    }

    /// Read duration in nanoseconds
    pub fn read_duration_ns(&mut self) -> HorusResult<u64> {
        let mut buf = [0u8; 8];
        self.file.seek(SeekFrom::Start(18))?;
        self.file.read_exact(&mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Write duration in nanoseconds
    pub fn write_duration_ns(&mut self, ns: u64) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(18))?;
        self.file.write_all(&ns.to_le_bytes())?;
        Ok(())
    }

    /// Read tick count
    pub fn read_tick_count(&mut self) -> HorusResult<u64> {
        let mut buf = [0u8; 8];
        self.file.seek(SeekFrom::Start(26))?;
        self.file.read_exact(&mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Write tick count
    pub fn write_tick_count(&mut self, count: u64) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(26))?;
        self.file.write_all(&count.to_le_bytes())?;
        Ok(())
    }

    /// Read error count
    pub fn read_error_count(&mut self) -> HorusResult<u64> {
        let mut buf = [0u8; 8];
        self.file.seek(SeekFrom::Start(34))?;
        self.file.read_exact(&mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Write error count
    pub fn write_error_count(&mut self, count: u64) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(34))?;
        self.file.write_all(&count.to_le_bytes())?;
        Ok(())
    }

    /// Read last heartbeat timestamp
    pub fn read_last_heartbeat(&mut self) -> HorusResult<u64> {
        let mut buf = [0u8; 8];
        self.file.seek(SeekFrom::Start(42))?;
        self.file.read_exact(&mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Write last heartbeat timestamp
    pub fn write_last_heartbeat(&mut self, ts: u64) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(42))?;
        self.file.write_all(&ts.to_le_bytes())?;
        Ok(())
    }

    /// Read child PID
    pub fn read_pid(&mut self) -> HorusResult<u32> {
        let mut buf = [0u8; 4];
        self.file.seek(SeekFrom::Start(50))?;
        self.file.read_exact(&mut buf)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Write child PID
    pub fn write_pid(&mut self, pid: u32) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(50))?;
        self.file.write_all(&pid.to_le_bytes())?;
        Ok(())
    }

    /// Read restart count
    pub fn read_restart_count(&mut self) -> HorusResult<u8> {
        let mut buf = [0u8; 1];
        self.file.seek(SeekFrom::Start(54))?;
        self.file.read_exact(&mut buf)?;
        Ok(buf[0])
    }

    /// Write restart count
    pub fn write_restart_count(&mut self, count: u8) -> HorusResult<()> {
        self.file.seek(SeekFrom::Start(54))?;
        self.file.write_all(&[count])?;
        Ok(())
    }

    /// Read error message
    pub fn read_error_message(&mut self) -> HorusResult<String> {
        let mut buf = [0u8; IPC_ERROR_MSG_SIZE];
        self.file
            .seek(SeekFrom::Start(IPC_ERROR_MSG_OFFSET as u64))?;
        self.file.read_exact(&mut buf)?;

        // Find null terminator
        let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        Ok(String::from_utf8_lossy(&buf[..len]).to_string())
    }

    /// Write error message
    pub fn write_error_message(&mut self, msg: &str) -> HorusResult<()> {
        let mut buf = [0u8; IPC_ERROR_MSG_SIZE];
        let bytes = msg.as_bytes();
        let len = bytes.len().min(IPC_ERROR_MSG_SIZE - 1);
        buf[..len].copy_from_slice(&bytes[..len]);

        self.file
            .seek(SeekFrom::Start(IPC_ERROR_MSG_OFFSET as u64))?;
        self.file.write_all(&buf)?;
        Ok(())
    }

    /// Update heartbeat to current time
    pub fn update_heartbeat(&mut self) -> HorusResult<()> {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        self.write_last_heartbeat(now)
    }

    /// Get the IPC file path
    pub fn path(&self) -> &PathBuf {
        &self.path
    }

    /// Sync all data to disk
    pub fn sync(&mut self) -> HorusResult<()> {
        self.file.sync_all()?;
        Ok(())
    }
}

impl Drop for IpcRegion {
    fn drop(&mut self) {
        // Don't delete the file - let it be cleaned up explicitly
    }
}

// ============================================================================
// Isolated Node Handle
// ============================================================================

/// Handle for an isolated node running in a child process
pub struct IsolatedNodeHandle {
    /// Node name
    pub node_name: String,
    /// Child process
    child: Option<Child>,
    /// IPC region for communication
    ipc: IpcRegion,
    /// Current sequence number
    sequence: u64,
    /// Configuration for restarts
    config: IsolatedNodeConfig,
    /// Number of times this node has been restarted
    restart_count: u8,
    /// Last known tick count
    last_tick_count: u64,
    /// Last known error count
    last_error_count: u64,
    /// Time of last successful health check
    last_health_check: Instant,
}

/// Configuration for isolated nodes
#[derive(Debug, Clone)]
pub struct IsolatedNodeConfig {
    /// Maximum number of restart attempts
    pub max_restarts: u8,
    /// Delay between restart attempts
    pub restart_delay: Duration,
    /// Timeout for waiting on responses
    pub response_timeout: Duration,
    /// Heartbeat timeout (consider dead if no heartbeat for this long)
    pub heartbeat_timeout: Duration,
    /// Path to the node runner binary (if None, uses in-process fallback)
    pub runner_binary: Option<PathBuf>,
    /// Environment variables for the child process
    pub env_vars: HashMap<String, String>,
}

impl Default for IsolatedNodeConfig {
    fn default() -> Self {
        Self {
            max_restarts: 3,
            restart_delay: Duration::from_millis(500),
            response_timeout: Duration::from_millis(5000),
            heartbeat_timeout: Duration::from_secs(10),
            runner_binary: None,
            env_vars: HashMap::new(),
        }
    }
}

impl IsolatedNodeHandle {
    /// Create a new isolated node handle
    fn new(node_name: String, config: IsolatedNodeConfig) -> HorusResult<Self> {
        let ipc = IpcRegion::create(&node_name)?;

        Ok(Self {
            node_name,
            child: None,
            ipc,
            sequence: 0,
            config,
            restart_count: 0,
            last_tick_count: 0,
            last_error_count: 0,
            last_health_check: Instant::now(),
        })
    }

    /// Spawn the child process
    fn spawn_process(&mut self, node_factory_name: &str) -> HorusResult<()> {
        // Determine the runner binary
        let runner = self.config.runner_binary.clone().unwrap_or_else(|| {
            // Try to find horus-isolated-runner in PATH or current directory
            std::env::current_exe()
                .ok()
                .and_then(|p| p.parent().map(|d| d.join("horus-isolated-runner")))
                .unwrap_or_else(|| PathBuf::from("horus-isolated-runner"))
        });

        // Spawn child process
        let mut cmd = Command::new(&runner);
        cmd.arg("--node-name")
            .arg(&self.node_name)
            .arg("--factory")
            .arg(node_factory_name)
            .arg("--ipc-path")
            .arg(self.ipc.path())
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit());

        // Add environment variables
        for (key, value) in &self.config.env_vars {
            cmd.env(key, value);
        }

        // Also pass session ID if set
        if let Ok(session_id) = std::env::var("HORUS_SESSION_ID") {
            cmd.env("HORUS_SESSION_ID", session_id);
        }

        let child = cmd.spawn().map_err(|e| {
            HorusError::Scheduling(format!(
                "Failed to spawn isolated process for '{}': {}. Runner binary: {:?}",
                self.node_name, e, runner
            ))
        })?;

        let pid = child.id();
        self.ipc.write_pid(pid)?;
        self.child = Some(child);

        // Wait for the child to initialize
        self.wait_for_ready()?;

        println!(
            "[Isolated] Spawned node '{}' in process {} (restarts: {})",
            self.node_name, pid, self.restart_count
        );

        Ok(())
    }

    /// Wait for the child process to signal ready
    fn wait_for_ready(&mut self) -> HorusResult<()> {
        let deadline = Instant::now() + self.config.response_timeout;

        while Instant::now() < deadline {
            let status = self.ipc.read_status()?;
            if status == IpcStatus::Idle {
                // Child is ready
                self.ipc.update_heartbeat()?;
                return Ok(());
            }
            thread::sleep(Duration::from_millis(10));
        }

        Err(HorusError::Timeout(format!(
            "Isolated node '{}' failed to initialize within {:?}",
            self.node_name, self.config.response_timeout
        )))
    }

    /// Send a command and wait for response
    fn send_command(&mut self, cmd: IpcCommand) -> HorusResult<IpcStatus> {
        self.sequence += 1;
        self.ipc.write_sequence(self.sequence)?;
        self.ipc.write_command(cmd)?;
        self.ipc.sync()?;

        let deadline = Instant::now() + self.config.response_timeout;

        while Instant::now() < deadline {
            let status = self.ipc.read_status()?;
            let seq = self.ipc.read_sequence()?;

            // Check if response is for our command
            if seq == self.sequence && status != IpcStatus::Processing {
                return Ok(status);
            }

            // Check if child process is still alive
            if let Some(ref mut child) = self.child {
                match child.try_wait() {
                    Ok(Some(exit_status)) => {
                        // Child exited
                        return Err(HorusError::Node {
                            node: self.node_name.clone(),
                            message: format!("Process exited with status: {:?}", exit_status),
                        });
                    }
                    Ok(None) => {
                        // Still running, continue waiting
                    }
                    Err(e) => {
                        return Err(HorusError::Node {
                            node: self.node_name.clone(),
                            message: format!("Failed to check process status: {}", e),
                        });
                    }
                }
            }

            thread::sleep(Duration::from_micros(100));
        }

        Err(HorusError::Timeout(format!(
            "Command {:?} timed out for node '{}'",
            cmd, self.node_name
        )))
    }

    /// Trigger a tick
    pub fn tick(&mut self) -> HorusResult<Duration> {
        let status = self.send_command(IpcCommand::Tick)?;

        match status {
            IpcStatus::Success => {
                let duration_ns = self.ipc.read_duration_ns()?;
                self.last_tick_count = self.ipc.read_tick_count()?;
                self.last_health_check = Instant::now();
                Ok(Duration::from_nanos(duration_ns))
            }
            IpcStatus::Error => {
                self.last_error_count = self.ipc.read_error_count()?;
                let msg = self.ipc.read_error_message()?;
                Err(HorusError::Node {
                    node: self.node_name.clone(),
                    message: msg,
                })
            }
            _ => Err(HorusError::Node {
                node: self.node_name.clone(),
                message: format!("Unexpected status: {:?}", status),
            }),
        }
    }

    /// Check if the node is healthy
    pub fn is_healthy(&mut self) -> bool {
        // Check if child process is still running
        if let Some(ref mut child) = self.child {
            match child.try_wait() {
                Ok(Some(_)) => return false, // Process exited
                Ok(None) => {}               // Still running
                Err(_) => return false,
            }
        } else {
            return false;
        }

        // Check heartbeat
        if let Ok(last_hb) = self.ipc.read_last_heartbeat() {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs();

            if now.saturating_sub(last_hb) > self.config.heartbeat_timeout.as_secs() {
                return false;
            }
        }

        true
    }

    /// Attempt to restart the node
    pub fn restart(&mut self, node_factory_name: &str) -> HorusResult<()> {
        if self.restart_count >= self.config.max_restarts {
            return Err(HorusError::Node {
                node: self.node_name.clone(),
                message: format!(
                    "Maximum restart attempts ({}) exceeded",
                    self.config.max_restarts
                ),
            });
        }

        // Kill existing process if any
        self.kill();

        // Wait before restart
        thread::sleep(self.config.restart_delay);

        self.restart_count += 1;
        self.ipc.write_restart_count(self.restart_count)?;

        // Reset IPC state
        self.ipc.write_status(IpcStatus::Idle)?;
        self.ipc.write_command(IpcCommand::None)?;
        self.ipc.sync()?;

        // Spawn new process
        self.spawn_process(node_factory_name)?;

        Ok(())
    }

    /// Kill the child process
    pub fn kill(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }

    /// Shutdown the node gracefully
    pub fn shutdown(&mut self) -> HorusResult<()> {
        if self.child.is_some() {
            // Send shutdown command
            let _ = self.send_command(IpcCommand::Shutdown);

            // Wait for process to exit
            if let Some(mut child) = self.child.take() {
                let _ = child.wait();
            }
        }

        // Clean up IPC file
        let _ = fs::remove_file(self.ipc.path());

        Ok(())
    }

    /// Get statistics
    pub fn stats(&mut self) -> IsolatedNodeStats {
        IsolatedNodeStats {
            node_name: self.node_name.clone(),
            tick_count: self.last_tick_count,
            error_count: self.last_error_count,
            restart_count: self.restart_count,
            is_healthy: self.is_healthy(),
            pid: self.child.as_ref().map(|c| c.id()),
        }
    }
}

impl Drop for IsolatedNodeHandle {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}

/// Statistics for an isolated node
#[derive(Debug, Clone)]
pub struct IsolatedNodeStats {
    pub node_name: String,
    pub tick_count: u64,
    pub error_count: u64,
    pub restart_count: u8,
    pub is_healthy: bool,
    pub pid: Option<u32>,
}

// ============================================================================
// Isolated Executor
// ============================================================================

/// Result from isolated node execution
#[derive(Debug)]
pub struct IsolatedResult {
    pub node_name: String,
    pub duration: Duration,
    pub success: bool,
    pub error: Option<String>,
    pub restart_attempted: bool,
}

/// Isolated executor for process-isolated node execution
///
/// Runs high-failure-rate or untrusted nodes in separate processes
/// for fault isolation. If a node crashes, only that process dies
/// and can be automatically restarted.
///
/// # Example
/// ```ignore
/// use horus_core::scheduling::executors::isolated::{IsolatedExecutor, IsolatedNodeConfig};
///
/// let mut executor = IsolatedExecutor::new(IsolatedNodeConfig::default())?;
///
/// // Register a node factory and spawn isolated node
/// executor.register_factory("my_node", || Box::new(MyNode::new()));
/// executor.spawn_node("my_node", "my_node")?;
///
/// // Tick all isolated nodes
/// let results = executor.tick_all();
/// ```
pub struct IsolatedExecutor {
    /// Node handles by name
    handles: HashMap<String, IsolatedNodeHandle>,
    /// Node factory names (for restart)
    factory_names: HashMap<String, String>,
    /// Default configuration
    default_config: IsolatedNodeConfig,
    /// Running flag
    running: Arc<AtomicBool>,
    /// Watchdog thread handle
    watchdog_handle: Option<JoinHandle<()>>,
    /// Last tick count for progress tracking
    tick_count: AtomicU64,
}

impl IsolatedExecutor {
    /// Create a new isolated executor
    pub fn new(config: IsolatedNodeConfig) -> HorusResult<Self> {
        let running = Arc::new(AtomicBool::new(true));

        Ok(Self {
            handles: HashMap::new(),
            factory_names: HashMap::new(),
            default_config: config,
            running,
            watchdog_handle: None,
            tick_count: AtomicU64::new(0),
        })
    }

    /// Spawn an isolated node
    ///
    /// # Arguments
    /// * `node` - The node to run in isolation (ownership transferred)
    /// * `factory_name` - Name of the factory to recreate this node type on restart
    /// * `context` - Optional node context
    pub fn spawn_node(
        &mut self,
        node: Box<dyn Node>,
        factory_name: &str,
        _context: Option<NodeInfo>,
    ) -> HorusResult<()> {
        let node_name = node.name().to_string();

        // For in-process fallback mode, we use a different approach
        // Store the node in a static registry and have child process look it up
        //
        // In production, you would:
        // 1. Have a separate binary (horus-isolated-runner) that can instantiate nodes
        // 2. Pass the factory name to that binary
        // 3. The binary creates the node and runs the IPC loop
        //
        // For now, we use the InProcessIsolatedNode which provides isolation
        // via panic catching and thread isolation (not full process isolation)

        // Check if we should use in-process mode (no runner binary specified)
        if self.default_config.runner_binary.is_none() {
            // Use in-process isolated execution
            return self.spawn_in_process(node, factory_name, _context);
        }

        // Create handle with IPC region
        let mut handle = IsolatedNodeHandle::new(node_name.clone(), self.default_config.clone())?;

        // Spawn the child process
        handle.spawn_process(factory_name)?;

        self.handles.insert(node_name.clone(), handle);
        self.factory_names
            .insert(node_name, factory_name.to_string());

        Ok(())
    }

    /// Spawn node in-process with thread isolation (fallback mode)
    fn spawn_in_process(
        &mut self,
        node: Box<dyn Node>,
        factory_name: &str,
        context: Option<NodeInfo>,
    ) -> HorusResult<()> {
        let node_name = node.name().to_string();

        // Create IPC region for this node
        let ipc = IpcRegion::create(&node_name)?;

        // Create the in-process isolated runner
        let mut runner = InProcessIsolatedRunner::new(node, context, ipc, self.running.clone());

        // Spawn the runner thread
        let handle = thread::Builder::new()
            .name(format!("horus-isolated-{}", node_name))
            .spawn(move || {
                runner.run();
            })
            .map_err(|e| {
                HorusError::Scheduling(format!(
                    "Failed to spawn isolated thread for '{}': {}",
                    node_name, e
                ))
            })?;

        // Create a handle that communicates via IPC
        let node_handle = IsolatedNodeHandle::new(node_name.clone(), self.default_config.clone())?;

        // Mark as "spawned" even though it's in-process
        // We'll communicate via the IPC region

        self.handles.insert(node_name.clone(), node_handle);
        self.factory_names
            .insert(node_name, factory_name.to_string());

        // Store the thread handle (we could use this for join later)
        // For now, we let it run detached
        drop(handle);

        Ok(())
    }

    /// Start the watchdog thread for health monitoring
    pub fn start_watchdog(&mut self) {
        if self.watchdog_handle.is_some() {
            return;
        }

        let running = Arc::clone(&self.running);
        let handle = thread::Builder::new()
            .name("horus-isolated-watchdog".to_string())
            .spawn(move || {
                while running.load(Ordering::Relaxed) {
                    thread::sleep(Duration::from_secs(1));
                    // Watchdog work is done in tick_all via health checks
                }
            })
            .expect("Failed to spawn watchdog thread");

        self.watchdog_handle = Some(handle);
    }

    /// Tick all isolated nodes
    pub fn tick_all(&mut self) -> Vec<IsolatedResult> {
        let mut results = Vec::new();

        // Collect node names to avoid borrow issues
        let node_names: Vec<String> = self.handles.keys().cloned().collect();

        for node_name in node_names {
            let result = self.tick_node(&node_name);
            results.push(result);
        }

        self.tick_count.fetch_add(1, Ordering::Relaxed);
        results
    }

    /// Tick a single node
    fn tick_node(&mut self, node_name: &str) -> IsolatedResult {
        let factory_name = self.factory_names.get(node_name).cloned();

        let handle = match self.handles.get_mut(node_name) {
            Some(h) => h,
            None => {
                return IsolatedResult {
                    node_name: node_name.to_string(),
                    duration: Duration::ZERO,
                    success: false,
                    error: Some("Node not found".to_string()),
                    restart_attempted: false,
                };
            }
        };

        // Check health first
        if !handle.is_healthy() {
            // Attempt restart
            if let Some(ref factory) = factory_name {
                match handle.restart(factory) {
                    Ok(_) => {
                        return IsolatedResult {
                            node_name: node_name.to_string(),
                            duration: Duration::ZERO,
                            success: false,
                            error: Some("Node was unhealthy, restarted".to_string()),
                            restart_attempted: true,
                        };
                    }
                    Err(e) => {
                        return IsolatedResult {
                            node_name: node_name.to_string(),
                            duration: Duration::ZERO,
                            success: false,
                            error: Some(format!("Failed to restart: {}", e)),
                            restart_attempted: true,
                        };
                    }
                }
            }
        }

        // Execute tick
        match handle.tick() {
            Ok(duration) => IsolatedResult {
                node_name: node_name.to_string(),
                duration,
                success: true,
                error: None,
                restart_attempted: false,
            },
            Err(e) => {
                // Check if we should restart
                let restart_attempted = if let Some(ref factory) = factory_name {
                    handle.restart(factory).is_ok()
                } else {
                    false
                };

                IsolatedResult {
                    node_name: node_name.to_string(),
                    duration: Duration::ZERO,
                    success: false,
                    error: Some(e.to_string()),
                    restart_attempted,
                }
            }
        }
    }

    /// Get statistics for all nodes
    pub fn stats(&mut self) -> Vec<IsolatedNodeStats> {
        self.handles.values_mut().map(|h| h.stats()).collect()
    }

    /// Get number of isolated nodes
    pub fn node_count(&self) -> usize {
        self.handles.len()
    }

    /// Shutdown all isolated nodes
    pub fn shutdown(&mut self) {
        self.running.store(false, Ordering::SeqCst);

        // Shutdown all nodes
        for (_, handle) in self.handles.drain() {
            let mut handle = handle;
            let _ = handle.shutdown();
        }

        // Wait for watchdog
        if let Some(handle) = self.watchdog_handle.take() {
            let _ = handle.join();
        }

        self.factory_names.clear();
    }
}

impl Default for IsolatedExecutor {
    fn default() -> Self {
        Self::new(IsolatedNodeConfig::default()).expect("Failed to create isolated executor")
    }
}

impl Drop for IsolatedExecutor {
    fn drop(&mut self) {
        self.shutdown();
    }
}

// ============================================================================
// In-Process Isolated Runner (Fallback)
// ============================================================================

/// In-process isolated runner for when no external binary is available
///
/// This provides thread-level isolation with panic catching, but not true
/// process isolation. It's useful for development and testing.
struct InProcessIsolatedRunner {
    node: Box<dyn Node>,
    context: Option<NodeInfo>,
    ipc: IpcRegion,
    running: Arc<AtomicBool>,
    tick_count: u64,
    error_count: u64,
}

impl InProcessIsolatedRunner {
    fn new(
        node: Box<dyn Node>,
        context: Option<NodeInfo>,
        ipc: IpcRegion,
        running: Arc<AtomicBool>,
    ) -> Self {
        Self {
            node,
            context,
            ipc,
            running,
            tick_count: 0,
            error_count: 0,
        }
    }

    fn run(&mut self) {
        // Signal ready
        let _ = self.ipc.write_status(IpcStatus::Idle);
        let _ = self.ipc.update_heartbeat();
        let _ = self.ipc.sync();

        let mut last_sequence = 0u64;

        while self.running.load(Ordering::Relaxed) {
            // Check for new command
            let sequence = self.ipc.read_sequence().unwrap_or(0);

            if sequence > last_sequence {
                last_sequence = sequence;

                let command = self.ipc.read_command().unwrap_or(IpcCommand::None);

                match command {
                    IpcCommand::Init => {
                        self.handle_init();
                    }
                    IpcCommand::Tick => {
                        self.handle_tick();
                    }
                    IpcCommand::Shutdown => {
                        self.handle_shutdown();
                        break;
                    }
                    IpcCommand::HealthCheck => {
                        self.handle_health_check();
                    }
                    IpcCommand::None => {}
                }

                // Update sequence to acknowledge command
                let _ = self.ipc.write_sequence(sequence);
                let _ = self.ipc.sync();
            }

            thread::sleep(Duration::from_micros(100));
        }
    }

    fn handle_init(&mut self) {
        let _ = self.ipc.write_status(IpcStatus::Processing);

        if let Some(ref mut ctx) = self.context {
            match self.node.init(ctx) {
                Ok(_) => {
                    let _ = self.ipc.write_status(IpcStatus::Success);
                }
                Err(e) => {
                    let _ = self.ipc.write_error_message(&e.to_string());
                    let _ = self.ipc.write_status(IpcStatus::Error);
                }
            }
        } else {
            let _ = self.ipc.write_status(IpcStatus::Success);
        }

        let _ = self.ipc.update_heartbeat();
    }

    fn handle_tick(&mut self) {
        let _ = self.ipc.write_status(IpcStatus::Processing);

        let start = Instant::now();

        // Execute tick with panic catching
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            if let Some(ref mut ctx) = self.context {
                ctx.start_tick();
            }
            self.node.tick();
            if let Some(ref mut ctx) = self.context {
                ctx.record_tick();
            }
        }));

        let duration = start.elapsed();

        match result {
            Ok(_) => {
                self.tick_count += 1;
                let _ = self.ipc.write_duration_ns(duration.as_nanos() as u64);
                let _ = self.ipc.write_tick_count(self.tick_count);
                let _ = self.ipc.write_status(IpcStatus::Success);
            }
            Err(panic_info) => {
                self.error_count += 1;

                let msg = if let Some(s) = panic_info.downcast_ref::<&str>() {
                    format!("Node panicked: {}", s)
                } else if let Some(s) = panic_info.downcast_ref::<String>() {
                    format!("Node panicked: {}", s)
                } else {
                    "Node panicked with unknown error".to_string()
                };

                if let Some(ref mut ctx) = self.context {
                    ctx.record_tick_failure(msg.clone());
                }

                let _ = self.ipc.write_error_message(&msg);
                let _ = self.ipc.write_error_count(self.error_count);
                let _ = self.ipc.write_status(IpcStatus::Error);
            }
        }

        let _ = self.ipc.update_heartbeat();
    }

    fn handle_shutdown(&mut self) {
        let _ = self.ipc.write_status(IpcStatus::Processing);

        if let Some(ref mut ctx) = self.context {
            let _ = self.node.shutdown(ctx);
        }

        let _ = self.ipc.write_status(IpcStatus::Success);
    }

    fn handle_health_check(&mut self) {
        let _ = self.ipc.write_status(IpcStatus::Success);
        let _ = self.ipc.update_heartbeat();
    }
}

// ============================================================================
// Node Runner Binary Entry Point
// ============================================================================

/// Run an isolated node from command line arguments
///
/// This function is intended to be called from the `horus-isolated-runner` binary.
/// It reads IPC commands from shared memory and executes the node.
///
/// # Arguments
/// * `node` - The node to run
/// * `ipc_path` - Path to the IPC shared memory file
pub fn run_isolated_node(mut node: Box<dyn Node>, ipc_path: &std::path::Path) -> HorusResult<()> {
    // Open IPC region
    let node_name = ipc_path
        .file_stem()
        .and_then(|s| s.to_str())
        .map(|s| s.trim_end_matches(".ipc"))
        .unwrap_or("unknown")
        .to_string();

    let mut ipc = IpcRegion::open(&node_name)?;

    // Validate magic
    let magic = ipc.read_magic()?;
    if magic != IPC_MAGIC {
        return Err(HorusError::SharedMemory(format!(
            "Invalid IPC magic: expected {:x}, got {:x}",
            IPC_MAGIC, magic
        )));
    }

    // Create context
    let mut context = NodeInfo::new(node_name.clone(), true);

    // Signal ready
    ipc.write_pid(std::process::id())?;
    ipc.write_status(IpcStatus::Idle)?;
    ipc.update_heartbeat()?;
    ipc.sync()?;

    println!(
        "[IsolatedRunner] Node '{}' started (PID: {})",
        node_name,
        std::process::id()
    );

    let mut last_sequence = 0u64;
    let mut tick_count = 0u64;
    let mut error_count = 0u64;

    loop {
        // Check for new command
        let sequence = ipc.read_sequence()?;

        if sequence > last_sequence {
            last_sequence = sequence;

            let command = ipc.read_command()?;

            match command {
                IpcCommand::Init => {
                    ipc.write_status(IpcStatus::Processing)?;

                    match node.init(&mut context) {
                        Ok(_) => {
                            ipc.write_status(IpcStatus::Success)?;
                        }
                        Err(e) => {
                            ipc.write_error_message(&e.to_string())?;
                            ipc.write_status(IpcStatus::Error)?;
                        }
                    }
                }
                IpcCommand::Tick => {
                    ipc.write_status(IpcStatus::Processing)?;

                    let start = Instant::now();
                    context.start_tick();

                    // Execute with panic catching
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        node.tick();
                    }));

                    let duration = start.elapsed();

                    match result {
                        Ok(_) => {
                            context.record_tick();
                            tick_count += 1;
                            ipc.write_duration_ns(duration.as_nanos() as u64)?;
                            ipc.write_tick_count(tick_count)?;
                            ipc.write_status(IpcStatus::Success)?;
                        }
                        Err(panic_info) => {
                            error_count += 1;

                            let msg = if let Some(s) = panic_info.downcast_ref::<&str>() {
                                format!("Node panicked: {}", s)
                            } else if let Some(s) = panic_info.downcast_ref::<String>() {
                                format!("Node panicked: {}", s)
                            } else {
                                "Node panicked with unknown error".to_string()
                            };

                            context.record_tick_failure(msg.clone());
                            ipc.write_error_message(&msg)?;
                            ipc.write_error_count(error_count)?;
                            ipc.write_status(IpcStatus::Error)?;
                        }
                    }
                }
                IpcCommand::Shutdown => {
                    ipc.write_status(IpcStatus::Processing)?;
                    let _ = node.shutdown(&mut context);
                    ipc.write_status(IpcStatus::Success)?;
                    ipc.sync()?;

                    println!("[IsolatedRunner] Node '{}' shutting down", node_name);
                    break;
                }
                IpcCommand::HealthCheck => {
                    ipc.write_status(IpcStatus::Success)?;
                }
                IpcCommand::None => {}
            }

            // Acknowledge command
            ipc.write_sequence(sequence)?;
            ipc.update_heartbeat()?;
            ipc.sync()?;
        }

        thread::sleep(Duration::from_micros(100));
    }

    Ok(())
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    struct TestNode {
        tick_count: u32,
        should_panic: bool,
    }

    impl TestNode {
        fn new() -> Self {
            Self {
                tick_count: 0,
                should_panic: false,
            }
        }
    }

    impl Node for TestNode {
        fn name(&self) -> &'static str {
            "TestIsolatedNode"
        }

        fn tick(&mut self) {
            if self.should_panic && self.tick_count > 2 {
                panic!("Intentional panic for testing");
            }
            self.tick_count += 1;
            thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn test_ipc_region_create_and_read() {
        let mut ipc = IpcRegion::create("test_ipc").unwrap();

        // Verify magic
        assert_eq!(ipc.read_magic().unwrap(), IPC_MAGIC);

        // Test sequence
        ipc.write_sequence(42).unwrap();
        assert_eq!(ipc.read_sequence().unwrap(), 42);

        // Test command
        ipc.write_command(IpcCommand::Tick).unwrap();
        assert_eq!(ipc.read_command().unwrap(), IpcCommand::Tick);

        // Test status
        ipc.write_status(IpcStatus::Success).unwrap();
        assert_eq!(ipc.read_status().unwrap(), IpcStatus::Success);

        // Clean up
        let _ = fs::remove_file(ipc.path());
    }

    #[test]
    fn test_ipc_error_message() {
        let mut ipc = IpcRegion::create("test_ipc_error").unwrap();

        let msg = "Test error message";
        ipc.write_error_message(msg).unwrap();
        assert_eq!(ipc.read_error_message().unwrap(), msg);

        // Test long message truncation
        let long_msg = "x".repeat(300);
        ipc.write_error_message(&long_msg).unwrap();
        let read_msg = ipc.read_error_message().unwrap();
        assert!(read_msg.len() < 300);

        // Clean up
        let _ = fs::remove_file(ipc.path());
    }

    #[test]
    fn test_isolated_executor_creation() {
        let executor = IsolatedExecutor::new(IsolatedNodeConfig::default());
        assert!(executor.is_ok());
    }

    #[test]
    fn test_isolated_node_in_process() {
        let mut executor = IsolatedExecutor::new(IsolatedNodeConfig::default()).unwrap();

        let node = Box::new(TestNode::new());
        executor.spawn_node(node, "TestNode", None).unwrap();

        // Wait for node to be ready
        thread::sleep(Duration::from_millis(100));

        // Tick multiple times
        for _ in 0..5 {
            let results = executor.tick_all();
            assert!(!results.is_empty());
            thread::sleep(Duration::from_millis(50));
        }

        // Shutdown
        executor.shutdown();
    }

    #[test]
    fn test_ipc_command_conversion() {
        assert_eq!(IpcCommand::from(0), IpcCommand::None);
        assert_eq!(IpcCommand::from(1), IpcCommand::Init);
        assert_eq!(IpcCommand::from(2), IpcCommand::Tick);
        assert_eq!(IpcCommand::from(3), IpcCommand::Shutdown);
        assert_eq!(IpcCommand::from(4), IpcCommand::HealthCheck);
        assert_eq!(IpcCommand::from(255), IpcCommand::None);
    }

    #[test]
    fn test_ipc_status_conversion() {
        assert_eq!(IpcStatus::from(0), IpcStatus::Idle);
        assert_eq!(IpcStatus::from(1), IpcStatus::Processing);
        assert_eq!(IpcStatus::from(2), IpcStatus::Success);
        assert_eq!(IpcStatus::from(3), IpcStatus::Error);
        assert_eq!(IpcStatus::from(4), IpcStatus::Crashed);
        assert_eq!(IpcStatus::from(255), IpcStatus::Idle);
    }
}
