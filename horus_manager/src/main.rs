use clap::{CommandFactory, Parser, Subcommand};
use clap_complete::generate;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use horus_core::memory::{has_native_shm, shm_base_dir};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

// Use modules from the library instead of redeclaring them
use horus_manager::{commands, monitor, monitor_tui, registry, security, workspace};

/// Calculate the total size of a directory recursively
fn dir_size(path: &Path) -> std::io::Result<u64> {
    let mut size = 0;
    if path.is_dir() {
        for entry in fs::read_dir(path)? {
            let entry = entry?;
            let path = entry.path();
            if path.is_dir() {
                size += dir_size(&path)?;
            } else {
                size += entry.metadata()?.len();
            }
        }
    }
    Ok(size)
}

/// Format a size in bytes to human-readable format
fn format_size(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;

    if bytes >= GB {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.2} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.2} KB", bytes as f64 / KB as f64)
    } else {
        format!("{} B", bytes)
    }
}

#[derive(Parser)]
#[command(name = "horus")]
#[command(about = "HORUS - Hybrid Optimized Robotics Unified System")]
#[command(version = "0.1.8")]
#[command(propagate_version = true)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Initialize HORUS workspace in current directory
    Init {
        /// Workspace name (optional, defaults to directory name)
        #[arg(short = 'n', long = "name")]
        name: Option<String>,
    },

    /// Create a new HORUS project
    New {
        /// Project name
        name: String,
        /// Output directory (optional, defaults to current directory)
        #[arg(short = 'o', long = "output")]
        path: Option<PathBuf>,
        /// Use Python
        #[arg(short = 'p', long = "python", conflicts_with = "rust")]
        python: bool,
        /// Use Rust
        #[arg(short = 'r', long = "rust", conflicts_with = "python")]
        rust: bool,
        /// Use Rust with macros
        #[arg(short = 'm', long = "macro", conflicts_with = "python")]
        use_macro: bool,
    },

    /// Run a HORUS project or file(s)
    Run {
        /// File(s) to run (optional, auto-detects if not specified)
        /// Can specify multiple files: horus run file1.py file2.rs file3.py
        files: Vec<PathBuf>,

        /// Build in release mode
        #[arg(short = 'r', long = "release")]
        release: bool,

        /// Clean build (remove cache)
        #[arg(short = 'c', long = "clean")]
        clean: bool,

        /// Suppress progress indicators
        #[arg(short = 'q', long = "quiet")]
        quiet: bool,

        /// Override detected drivers (comma-separated list)
        /// Example: --drivers camera,lidar,imu
        #[arg(short = 'd', long = "drivers", value_delimiter = ',')]
        drivers: Option<Vec<String>>,

        /// Enable capabilities (comma-separated list)
        /// Example: --enable cuda,editor,python
        #[arg(short = 'e', long = "enable", value_delimiter = ',')]
        enable: Option<Vec<String>>,

        /// Enable recording for this session
        /// Use 'horus record list' to see recordings
        #[arg(long = "record")]
        record: Option<String>,

        /// Additional arguments to pass to the program (use -- to separate)
        #[arg(last = true)]
        args: Vec<String>,
    },

    /// Validate horus.yaml, source files, or entire workspace
    Check {
        /// Path to file, directory, or workspace (default: current directory)
        #[arg(value_name = "PATH")]
        path: Option<PathBuf>,

        /// Only show errors, suppress warnings
        #[arg(short = 'q', long = "quiet")]
        quiet: bool,
    },

    /// Run tests for the HORUS project
    Test {
        /// Test name filter (runs tests matching this string)
        #[arg(value_name = "FILTER")]
        filter: Option<String>,

        /// Run tests in release mode
        #[arg(short = 'r', long = "release")]
        release: bool,

        /// Show test output (--nocapture)
        #[arg(long = "nocapture")]
        nocapture: bool,

        /// Number of test threads (default: 1 for shared memory safety)
        #[arg(short = 'j', long = "test-threads")]
        test_threads: Option<usize>,

        /// Allow parallel test execution (overrides default single-threaded mode)
        #[arg(long = "parallel")]
        parallel: bool,

        /// Enable simulation mode (use simulation drivers, no hardware required)
        #[arg(long = "sim")]
        simulation: bool,

        /// Run integration tests (tests marked `#[ignore]`)
        #[arg(long = "integration")]
        integration: bool,

        /// Skip the build step (assume already built)
        #[arg(long = "no-build")]
        no_build: bool,

        /// Skip shared memory cleanup after tests
        #[arg(long = "no-cleanup")]
        no_cleanup: bool,

        /// Verbose output
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Override detected drivers (comma-separated list)
        /// Example: --drivers camera,lidar,imu
        #[arg(short = 'd', long = "drivers", value_delimiter = ',')]
        drivers: Option<Vec<String>>,

        /// Enable capabilities (comma-separated list)
        /// Example: --enable cuda,editor,python
        #[arg(short = 'e', long = "enable", value_delimiter = ',')]
        enable: Option<Vec<String>>,
    },

    /// Build the HORUS project without running
    Build {
        /// File(s) to build (optional, auto-detects if not specified)
        files: Vec<PathBuf>,

        /// Build in release mode
        #[arg(short = 'r', long = "release")]
        release: bool,

        /// Clean build (remove cache)
        #[arg(short = 'c', long = "clean")]
        clean: bool,

        /// Suppress progress indicators
        #[arg(short = 'q', long = "quiet")]
        quiet: bool,

        /// Override detected drivers (comma-separated list)
        /// Example: --drivers camera,lidar,imu
        #[arg(short = 'd', long = "drivers", value_delimiter = ',')]
        drivers: Option<Vec<String>>,

        /// Enable capabilities (comma-separated list)
        /// Example: --enable cuda,editor,python
        #[arg(short = 'e', long = "enable", value_delimiter = ',')]
        enable: Option<Vec<String>>,
    },

    /// Monitor running HORUS nodes, topics, and system health
    Monitor {
        /// Port for web interface (default: 3000)
        #[arg(value_name = "PORT", default_value = "3000")]
        port: u16,

        /// Use Terminal UI mode instead of web
        #[arg(short = 't', long = "tui")]
        tui: bool,

        /// Reset password before starting
        #[arg(short = 'r', long = "reset-password")]
        reset_password: bool,
    },

    /// Topic interaction (list, echo, publish)
    Topic {
        #[command(subcommand)]
        command: TopicCommands,
    },

    /// Zenoh network discovery and interaction (requires zenoh-transport feature)
    #[cfg(feature = "zenoh-transport")]
    Zenoh {
        #[command(subcommand)]
        command: ZenohCommands,
    },

    /// Node management (list, info, kill)
    Node {
        #[command(subcommand)]
        command: NodeCommands,
    },

    /// Parameter management (get, set, list, delete)
    Param {
        #[command(subcommand)]
        command: ParamCommands,
    },

    /// HFrame operations (list, echo, tree) - coordinate transform frames
    Hf {
        #[command(subcommand)]
        command: HfCommands,
    },

    /// System diagnostics and health check
    Doctor {
        /// Show detailed diagnostic information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },

    /// Discover HORUS nodes on the local network via mDNS
    #[cfg(feature = "mdns")]
    Discover {
        /// Scan duration in seconds (default: 2)
        #[arg(short = 't', long = "timeout", default_value = "2")]
        timeout: u64,

        /// Filter by topic name
        #[arg(long = "topic")]
        topic: Option<String>,

        /// Filter by node name (partial match)
        #[arg(long = "name")]
        name: Option<String>,

        /// Watch for nodes joining/leaving (continuous mode)
        #[arg(short = 'w', long = "watch")]
        watch: bool,

        /// Output format: table, json, or simple
        #[arg(short = 'f', long = "format", default_value = "table")]
        format: String,
    },

    /// Hardware discovery and platform detection
    Hardware {
        #[command(subcommand)]
        command: HardwareCommands,
    },

    /// Clean build artifacts and shared memory
    Clean {
        /// Only clean shared memory
        #[arg(long = "shm")]
        shm: bool,

        /// Clean everything (build cache + shared memory + horus cache)
        #[arg(short = 'a', long = "all")]
        all: bool,

        /// Show what would be cleaned without removing anything
        #[arg(short = 'n', long = "dry-run")]
        dry_run: bool,
    },

    /// Launch multiple nodes from a YAML file
    Launch {
        /// Path to launch file (YAML)
        file: std::path::PathBuf,

        /// Show what would launch without actually launching
        #[arg(short = 'n', long = "dry-run")]
        dry_run: bool,

        /// Namespace prefix for all nodes
        #[arg(long = "namespace")]
        namespace: Option<String>,

        /// List nodes in the launch file without launching
        #[arg(long = "list")]
        list: bool,
    },

    /// Message type introspection
    Msg {
        #[command(subcommand)]
        command: MsgCommands,
    },

    /// View and filter logs
    Log {
        /// Filter by node name
        #[arg(value_name = "NODE")]
        node: Option<String>,

        /// Filter by log level (trace, debug, info, warn, error)
        #[arg(short = 'l', long = "level")]
        level: Option<String>,

        /// Show logs from last duration (e.g., "5m", "1h", "30s")
        #[arg(short = 's', long = "since")]
        since: Option<String>,

        /// Follow log output in real-time
        #[arg(short = 'f', long = "follow")]
        follow: bool,

        /// Number of recent log entries to show
        #[arg(short = 'n', long = "count")]
        count: Option<usize>,

        /// Clear logs instead of viewing
        #[arg(long = "clear")]
        clear: bool,

        /// Clear all logs (including file-based logs)
        #[arg(long = "clear-all")]
        clear_all: bool,
    },

    /// Package management
    Pkg {
        #[command(subcommand)]
        command: PkgCommands,
    },

    /// Environment management (freeze/restore)
    Env {
        #[command(subcommand)]
        command: EnvCommands,
    },

    /// Authentication commands
    Auth {
        #[command(subcommand)]
        command: AuthCommands,
    },

    /// Driver management (list, info, search)
    Driver {
        #[command(subcommand)]
        command: DriverCommands,
    },

    /// Deploy project to a remote robot
    Deploy {
        /// Target host (user@host or configured target name)
        #[arg(required_unless_present = "list")]
        target: Option<String>,

        /// Remote directory to deploy to (default: ~/horus_deploy)
        #[arg(short = 'd', long = "dir")]
        remote_dir: Option<String>,

        /// Target architecture (aarch64, armv7, x86_64, native)
        #[arg(short = 'a', long = "arch")]
        arch: Option<String>,

        /// Run the project after deploying
        #[arg(short = 'r', long = "run")]
        run_after: bool,

        /// Build in debug mode instead of release
        #[arg(long = "debug")]
        debug: bool,

        /// SSH port (default: 22)
        #[arg(short = 'p', long = "port", default_value = "22")]
        port: u16,

        /// SSH identity file
        #[arg(short = 'i', long = "identity")]
        identity: Option<PathBuf>,

        /// Show what would be done without actually doing it
        #[arg(short = 'n', long = "dry-run")]
        dry_run: bool,

        /// List configured deployment targets
        #[arg(long = "list")]
        list: bool,
    },

    /// Add a package, driver, or plugin (smart auto-detection)
    Add {
        /// Package/driver/plugin name to add
        name: String,
        /// Specific version (optional)
        #[arg(short = 'v', long = "ver")]
        ver: Option<String>,
        /// Force install as driver
        #[arg(long = "driver", conflicts_with = "plugin")]
        driver: bool,
        /// Force install as plugin
        #[arg(long = "plugin", conflicts_with = "driver")]
        plugin: bool,
        /// Force local installation (default for drivers/packages)
        #[arg(long = "local", conflicts_with = "global")]
        local: bool,
        /// Force global installation (default for plugins)
        #[arg(short = 'g', long = "global", conflicts_with = "local")]
        global: bool,
        /// Skip installing system dependencies
        #[arg(long = "no-system")]
        no_system: bool,
    },

    /// Remove a package, driver, or plugin
    Remove {
        /// Package/driver/plugin name to remove
        name: String,
        /// Remove from global scope
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// Also remove unused dependencies
        #[arg(long = "purge")]
        purge: bool,
    },

    /// Plugin management (list, enable, disable)
    Plugin {
        #[command(subcommand)]
        command: PluginCommands,
    },

    /// Cache management (info, clean, purge)
    Cache {
        #[command(subcommand)]
        command: CacheCommands,
    },

    /// Record/replay management for debugging and testing
    Record {
        #[command(subcommand)]
        command: RecordCommands,
    },

    /// Network diagnostics (connectivity, latency, troubleshooting)
    Net {
        #[command(subcommand)]
        command: NetCommands,
    },

    /// Husarnet VPN integration (status, peers, connectivity)
    Husarnet {
        #[command(subcommand)]
        command: HusarnetCommands,
    },

    /// Generate shell completion scripts
    #[command(hide = true)]
    Completion {
        /// Shell to generate completions for
        #[arg(value_enum)]
        shell: clap_complete::Shell,
    },
}

#[derive(Subcommand)]
enum NetCommands {
    /// Full connectivity check (local, external, NAT)
    Check {
        /// Show detailed diagnostic output
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },

    /// Measure latency to an endpoint
    Ping {
        /// Target endpoint (hostname, IP, or hostname:port)
        endpoint: String,

        /// Number of pings to send
        #[arg(short = 'c', long = "count", default_value = "5")]
        count: u32,

        /// Interval between pings in milliseconds
        #[arg(short = 'i', long = "interval", default_value = "1000")]
        interval: u64,
    },

    /// Trace network path to endpoint
    Trace {
        /// Target endpoint
        endpoint: String,

        /// Maximum number of hops
        #[arg(short = 'm', long = "max-hops", default_value = "30")]
        max_hops: u32,
    },

    /// Network troubleshooting with fix suggestions
    Doctor,

    /// Start a WebSocket signaling server for P2P connection coordination
    #[cfg(feature = "signaling-server")]
    SignalServer {
        /// Port to listen on (default: 8765)
        #[arg(short = 'p', long = "port", default_value = "8765")]
        port: u16,

        /// Bind address (default: 0.0.0.0)
        #[arg(short = 'b', long = "bind", default_value = "0.0.0.0")]
        bind: String,

        /// Shared secret for authentication (optional)
        #[arg(short = 's', long = "secret")]
        secret: Option<String>,

        /// Maximum number of peers (default: 10000)
        #[arg(long = "max-peers", default_value = "10000")]
        max_peers: usize,

        /// Enable verbose logging
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },
}

#[derive(Subcommand)]
enum HusarnetCommands {
    /// Check Husarnet daemon status and configuration
    Status {
        /// Show detailed diagnostic output
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },

    /// List Husarnet network peers
    Peers {
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Test connectivity to Husarnet peers
    Test {
        /// Target hostname (optional, tests all peers if not specified)
        target: Option<String>,

        /// Number of test packets to send
        #[arg(short = 'c', long = "count", default_value = "5")]
        count: u32,

        /// Timeout per packet in milliseconds
        #[arg(short = 't', long = "timeout", default_value = "1000")]
        timeout: u64,
    },

    /// Diagnose Husarnet configuration and provide recommendations
    Doctor,
}

#[derive(Subcommand)]
enum PkgCommands {
    /// Install a package from registry
    Install {
        /// Package name to install
        package: String,
        /// Specific package version (optional)
        #[arg(short = 'v', long = "ver")]
        ver: Option<String>,
        /// Install to global cache (shared across projects)
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// Target workspace/project name (if not in workspace)
        #[arg(short = 't', long = "target")]
        target: Option<String>,
    },

    /// Remove an installed package
    Remove {
        /// Package name to remove
        package: String,
        /// Remove from global cache
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// Target workspace/project name
        #[arg(short = 't', long = "target")]
        target: Option<String>,
    },

    /// List installed packages or search registry
    List {
        /// Search query (optional)
        query: Option<String>,
        /// List global cache packages
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// List all (local + global)
        #[arg(short = 'a', long = "all")]
        all: bool,
    },

    /// Publish package to registry
    Publish {
        /// Also generate freeze file
        #[arg(long)]
        freeze: bool,
        /// Validate package without actually publishing
        #[arg(long = "dry-run")]
        dry_run: bool,
    },

    /// Update installed packages to latest versions
    Update {
        /// Specific package to update (updates all if omitted)
        package: Option<String>,
        /// Update global cache packages
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// Show what would be updated without making changes
        #[arg(long = "dry-run")]
        dry_run: bool,
    },

    /// Generate signing key pair for package signing
    KeyGen,

    /// Unpublish a package from the registry
    Unpublish {
        /// Package name to unpublish
        package: String,
        /// Package version to unpublish
        version: String,
        /// Skip confirmation prompt
        #[arg(short = 'y', long = "yes")]
        yes: bool,
    },
}

#[derive(Subcommand)]
enum PluginCommands {
    /// List installed plugins
    List {
        /// Show all plugins including disabled
        #[arg(short = 'a', long = "all")]
        all: bool,
    },

    /// Search for available plugins
    Search {
        /// Search query (e.g., "camera", "lidar", "motor")
        query: String,
        /// Filter by category
        #[arg(short = 'c', long = "category")]
        category: Option<String>,
    },

    /// Show all available plugins from registry
    Available {
        /// Filter by category (camera, lidar, motor, servo, bus, gps, simulation)
        #[arg(short = 'c', long = "category")]
        category: Option<String>,
        /// Include local development plugins
        #[arg(short = 'l', long = "local")]
        include_local: bool,
    },

    /// Show detailed info about a plugin
    Info {
        /// Plugin name (e.g., "horus-rplidar")
        name: String,
    },

    /// Enable a disabled plugin
    Enable {
        /// Plugin command name to enable
        command: String,
    },

    /// Disable a plugin (keep installed but don't execute)
    Disable {
        /// Plugin command name to disable
        command: String,
        /// Reason for disabling
        #[arg(short = 'r', long = "reason")]
        reason: Option<String>,
    },

    /// Verify integrity of installed plugins
    Verify {
        /// Specific plugin to verify (optional, verifies all if not specified)
        plugin: Option<String>,
    },
}

#[derive(Subcommand)]
enum CacheCommands {
    /// Show cache information (size, packages, disk usage)
    Info,

    /// Remove unused packages from cache
    Clean {
        /// Show what would be removed without actually removing
        #[arg(short = 'n', long = "dry-run")]
        dry_run: bool,
    },

    /// Remove ALL packages from cache (nuclear option)
    Purge {
        /// Skip confirmation prompt
        #[arg(short = 'y', long = "yes")]
        yes: bool,
    },

    /// List all cached packages
    List,
}

#[derive(Subcommand)]
enum EnvCommands {
    /// List all published environments
    List,

    /// Show details of an environment
    Show {
        /// Environment ID (e.g., horus-env-abc123)
        id: String,
    },

    /// Freeze current environment to a manifest file
    Freeze {
        /// Output file path (default: horus-freeze.yaml)
        #[arg(short = 'o', long = "output")]
        output: Option<PathBuf>,

        /// Publish environment to registry for sharing by ID
        #[arg(short = 'p', long = "publish")]
        publish: bool,
    },

    /// Restore environment from freeze file or registry ID
    Restore {
        /// Path to freeze file or environment ID
        source: String,
    },
}

#[derive(Subcommand)]
enum AuthCommands {
    /// Login to HORUS registry (requires GitHub)
    Login,
    /// Generate API key after GitHub login
    GenerateKey {
        /// Name for the API key
        #[arg(long)]
        name: Option<String>,
        /// Environment (e.g., 'laptop', 'ci-cd')
        #[arg(long)]
        environment: Option<String>,
    },
    /// Logout from HORUS registry
    Logout,
    /// Show current authenticated user
    Whoami,
    /// Show organization info (HORUS Cloud)
    Org,
    /// Show current billing usage (HORUS Cloud)
    Usage,
    /// Show current plan details (HORUS Cloud)
    Plan,
    /// Manage API keys
    Keys {
        #[command(subcommand)]
        command: AuthKeysCommands,
    },
}

#[derive(Subcommand)]
enum AuthKeysCommands {
    /// List all API keys
    List,
    /// Revoke an API key
    Revoke {
        /// Key ID to revoke (e.g., horus_key_abc123...)
        key_id: String,
    },
}

#[derive(Subcommand)]
enum DriverCommands {
    /// List all available drivers (local + registry + plugins)
    List {
        /// Filter by category (sensor, actuator, bus, input, simulation)
        #[arg(short = 'c', long = "category")]
        category: Option<String>,
        /// Show only registry drivers (not local built-ins)
        #[arg(short = 'r', long = "registry")]
        registry_only: bool,
        /// Show only loaded plugins
        #[arg(short = 'p', long = "plugins")]
        plugins_only: bool,
        /// Driver loading mode when using --plugins (static, dynamic, hybrid)
        #[arg(short = 'm', long = "mode", default_value = "hybrid")]
        mode: String,
    },

    /// Show detailed information about a driver
    Info {
        /// Driver ID (e.g., rplidar, mpu6050, bno055)
        driver: String,
    },

    /// Search for drivers (searches registry)
    Search {
        /// Search query
        query: String,
        /// Filter by bus type (usb, i2c, spi, serial)
        #[arg(short = 'b', long = "bus")]
        bus_type: Option<String>,
    },

    /// Probe for available hardware using plugins
    Probe {
        /// Specific plugin to probe (e.g., horus-imu)
        #[arg(short = 'p', long = "plugin")]
        plugin: Option<String>,
        /// Specific backend to probe (e.g., mpu6050)
        #[arg(short = 'b', long = "backend")]
        backend: Option<String>,
        /// Driver loading mode (static, dynamic, hybrid)
        #[arg(short = 'm', long = "mode", default_value = "hybrid")]
        mode: String,
    },

    /// List and manage loaded driver plugins
    Plugins {
        /// Reload all plugins
        #[arg(short = 'r', long = "reload")]
        reload: bool,
        /// Driver loading mode (static, dynamic, hybrid)
        #[arg(short = 'm', long = "mode", default_value = "hybrid")]
        mode: String,
    },
}

#[derive(Subcommand)]
enum RecordCommands {
    /// List all recording sessions
    List {
        /// Show detailed info (file sizes, tick counts)
        #[arg(short = 'l', long = "long")]
        long: bool,
    },

    /// Show details of a specific recording session
    Info {
        /// Session name
        session: String,
    },

    /// Delete a recording session
    Delete {
        /// Session name to delete
        session: String,
        /// Force delete without confirmation
        #[arg(short = 'f', long = "force")]
        force: bool,
    },

    /// Replay a recording
    Replay {
        /// Path to scheduler recording or session name
        recording: String,

        /// Start at specific tick (time travel)
        #[arg(long)]
        start_tick: Option<u64>,

        /// Stop at specific tick
        #[arg(long)]
        stop_tick: Option<u64>,

        /// Playback speed multiplier (e.g., 0.5 for half speed)
        #[arg(long, default_value = "1.0")]
        speed: f64,

        /// Override values (format: node.output=value)
        #[arg(long = "override", value_parser = parse_override)]
        overrides: Vec<(String, String, String)>,
    },

    /// Compare two recording sessions (diff)
    Diff {
        /// First session name or path
        session1: String,
        /// Second session name or path
        session2: String,
        /// Only show first N differences
        #[arg(short = 'n', long = "limit")]
        limit: Option<usize>,
    },

    /// Export a recording to different format
    Export {
        /// Session name
        session: String,
        /// Output file path
        #[arg(short = 'o', long = "output")]
        output: PathBuf,
        /// Export format (json, csv)
        #[arg(short = 'f', long = "format", default_value = "json")]
        format: String,
    },

    /// Inject recorded node(s) into a new scheduler with live code
    ///
    /// This allows mixing recorded data with live processing nodes.
    /// Useful for testing algorithms with recorded sensor data without
    /// needing the physical hardware connected.
    ///
    /// Example: horus record inject my_session --nodes camera_node --script process.rs
    Inject {
        /// Session name containing the recorded nodes
        session: String,

        /// Node names to inject (comma-separated, or use --all)
        #[arg(short = 'n', long = "nodes", value_delimiter = ',')]
        nodes: Vec<String>,

        /// Inject all nodes from the session
        #[arg(long = "all")]
        all: bool,

        /// Rust script file containing live nodes to run alongside
        #[arg(short = 's', long = "script")]
        script: Option<PathBuf>,

        /// Start at specific tick
        #[arg(long = "start-tick")]
        start_tick: Option<u64>,

        /// Stop at specific tick
        #[arg(long = "stop-tick")]
        stop_tick: Option<u64>,

        /// Playback speed multiplier
        #[arg(long = "speed", default_value = "1.0")]
        speed: f64,

        /// Loop the recording (restart when finished)
        #[arg(long = "loop")]
        loop_playback: bool,
    },
}

#[derive(Subcommand)]
enum TopicCommands {
    /// List all active topics
    List {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Echo messages from a topic
    Echo {
        /// Topic name
        name: String,

        /// Number of messages to echo (optional)
        #[arg(short = 'n', long = "count")]
        count: Option<usize>,

        /// Maximum rate in Hz (optional)
        #[arg(short = 'r', long = "rate")]
        rate: Option<f64>,
    },

    /// Show detailed info about a topic
    Info {
        /// Topic name
        name: String,
    },

    /// Measure topic publish rate
    Hz {
        /// Topic name
        name: String,

        /// Window size for averaging (default: 10)
        #[arg(short = 'w', long = "window")]
        window: Option<usize>,
    },

    /// Publish a message to a topic (for testing)
    Pub {
        /// Topic name
        name: String,

        /// Message content
        message: String,

        /// Publish rate in Hz (optional)
        #[arg(short = 'r', long = "rate")]
        rate: Option<f64>,

        /// Number of messages to publish (default: 1)
        #[arg(short = 'n', long = "count")]
        count: Option<usize>,
    },
}

#[derive(Subcommand)]
enum HfCommands {
    /// List all coordinate frames
    List {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Echo transform between two frames (like tf_echo)
    Echo {
        /// Source frame
        source: String,

        /// Target frame
        target: String,

        /// Update rate in Hz (default: 1.0)
        #[arg(short = 'r', long = "rate")]
        rate: Option<f64>,

        /// Number of transforms to echo (optional)
        #[arg(short = 'n', long = "count")]
        count: Option<usize>,
    },

    /// Show frame tree structure (like view_frames)
    Tree {
        /// Output to file (PDF/SVG)
        #[arg(short = 'o', long = "output")]
        output: Option<String>,
    },

    /// Show detailed info about a frame
    Info {
        /// Frame name
        name: String,
    },

    /// Check if transform is available between frames
    Can {
        /// Source frame
        source: String,

        /// Target frame
        target: String,
    },

    /// Monitor frame update rates
    Hz {
        /// Window size for averaging (default: 10)
        #[arg(short = 'w', long = "window")]
        window: Option<usize>,
    },
}

#[derive(Subcommand)]
enum HardwareCommands {
    /// Scan for connected hardware devices
    Scan {
        /// Scan USB devices
        #[arg(long = "usb")]
        usb: bool,

        /// Scan serial ports
        #[arg(long = "serial")]
        serial: bool,

        /// Scan I2C buses (Linux only)
        #[arg(long = "i2c")]
        i2c: bool,

        /// Probe I2C addresses to detect devices (requires root)
        #[arg(long = "probe-i2c")]
        probe_i2c: bool,

        /// Scan GPIO chips (Linux only)
        #[arg(long = "gpio")]
        gpio: bool,

        /// Scan cameras (Linux only)
        #[arg(long = "cameras")]
        cameras: bool,

        /// Scan all device types (default if no flags specified)
        #[arg(short = 'a', long = "all")]
        all: bool,

        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON for machine-readable results
        #[arg(long = "json")]
        json: bool,

        /// Filter by category (comma-separated: usb,serial,i2c,gpio,cameras,sensors,motors,all)
        #[arg(long = "filter", short = 'f')]
        filter: Option<String>,

        /// Timeout per device probe in milliseconds (default: 500)
        #[arg(long = "timeout", short = 't')]
        timeout_ms: Option<u64>,
    },

    /// Show platform information
    Platform {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },

    /// Suggest HORUS node configuration based on detected hardware
    Suggest {
        /// Show detailed configuration
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },

    /// Get detailed information about a specific device
    Info {
        /// Device path (e.g., /dev/ttyUSB0, /dev/video0)
        device: String,

        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },

    /// Export hardware configuration to a TOML file
    Export {
        /// Output file path (prints to stdout if not specified)
        #[arg(short = 'o', long = "output")]
        output: Option<String>,

        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,
    },

    /// Watch for hardware connect/disconnect events (hotplug monitoring)
    Watch {
        /// Watch USB devices
        #[arg(long = "usb")]
        usb: bool,

        /// Watch serial ports
        #[arg(long = "serial")]
        serial: bool,

        /// Watch I2C buses (Linux only)
        #[arg(long = "i2c")]
        i2c: bool,

        /// Watch GPIO chips (Linux only)
        #[arg(long = "gpio")]
        gpio: bool,

        /// Watch cameras (Linux only)
        #[arg(long = "cameras")]
        cameras: bool,

        /// Watch all device types (default if no flags specified)
        #[arg(short = 'a', long = "all")]
        all: bool,

        /// Timeout per device probe in milliseconds (default: 500)
        #[arg(long = "timeout", short = 't')]
        timeout_ms: Option<u64>,
    },
}

#[derive(Subcommand)]
enum NodeCommands {
    /// List all running nodes
    List {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,

        /// Filter by category (node, tool, cli)
        #[arg(short = 'c', long = "category")]
        category: Option<String>,
    },

    /// Show detailed info about a node
    Info {
        /// Node name
        name: String,
    },

    /// Kill a running node
    Kill {
        /// Node name
        name: String,

        /// Force kill (SIGKILL instead of SIGTERM)
        #[arg(short = 'f', long = "force")]
        force: bool,
    },

    /// Restart a node (re-initialize without killing scheduler)
    Restart {
        /// Node name
        name: String,
    },

    /// Pause a running node (temporarily stop ticking)
    Pause {
        /// Node name
        name: String,
    },

    /// Resume a paused node
    Resume {
        /// Node name
        name: String,
    },
}

#[derive(Subcommand)]
enum ParamCommands {
    /// List all parameters
    List {
        /// Show detailed information (description, unit, validation)
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Get a parameter value
    Get {
        /// Parameter key
        key: String,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Set a parameter value
    Set {
        /// Parameter key
        key: String,

        /// Parameter value (auto-detected type: bool, int, float, string, or JSON)
        value: String,
    },

    /// Delete a parameter
    Delete {
        /// Parameter key
        key: String,
    },

    /// Reset all parameters to defaults
    Reset {
        /// Force reset without confirmation
        #[arg(short = 'f', long = "force")]
        force: bool,
    },

    /// Load parameters from a YAML file
    Load {
        /// Path to YAML file
        file: std::path::PathBuf,
    },

    /// Save parameters to a YAML file
    Save {
        /// Path to YAML file (default: .horus/config/params.yaml)
        #[arg(short = 'o', long = "output")]
        file: Option<std::path::PathBuf>,
    },

    /// Dump all parameters as YAML to stdout
    Dump,
}

/// Zenoh network discovery and interaction commands
#[cfg(feature = "zenoh-transport")]
#[derive(Subcommand)]
enum ZenohCommands {
    /// List all discovered Zenoh topics
    List {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,

        /// Only show ROS2 topics (rt/*, rq/*, rr/*)
        #[arg(long = "ros2")]
        ros2_only: bool,
    },

    /// Show detailed info about a Zenoh topic
    Info {
        /// Topic key expression (e.g., "rt/scan" or "/scan" for ROS2)
        key_expr: String,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Echo messages from a Zenoh topic
    Echo {
        /// Topic key expression
        key_expr: String,

        /// Number of messages to echo (default: unlimited)
        #[arg(short = 'n', long = "count")]
        count: Option<usize>,

        /// Show raw hex dump instead of interpreting data
        #[arg(long = "raw")]
        raw: bool,
    },

    /// Publish a message to a Zenoh topic
    Pub {
        /// Topic key expression
        key_expr: String,

        /// Message content (JSON or plain string)
        message: String,

        /// Publishing rate in Hz (default: 1.0)
        #[arg(short = 'r', long = "rate")]
        rate: Option<f64>,

        /// Number of messages to publish (default: 1)
        #[arg(short = 'n', long = "count")]
        count: Option<usize>,
    },
}

#[derive(Subcommand)]
enum MsgCommands {
    /// List all message types
    List {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Filter by name or module
        #[arg(short = 'f', long = "filter")]
        filter: Option<String>,
    },

    /// Show message type definition
    Show {
        /// Message type name
        name: String,
    },

    /// Show message type MD5 hash
    Md5 {
        /// Message type name
        name: String,
    },
}

/// Parse override argument in format "node.output=value"
fn parse_override(s: &str) -> Result<(String, String, String), String> {
    let parts: Vec<&str> = s.splitn(2, '=').collect();
    if parts.len() != 2 {
        return Err("Override must be in format 'node.output=value'".to_string());
    }

    let key_parts: Vec<&str> = parts[0].splitn(2, '.').collect();
    if key_parts.len() != 2 {
        return Err("Override key must be in format 'node.output'".to_string());
    }

    Ok((
        key_parts[0].to_string(),
        key_parts[1].to_string(),
        parts[1].to_string(),
    ))
}

/// Parse a hex string (without 0x prefix) into bytes
fn parse_hex_string(s: &str) -> Result<Vec<u8>, String> {
    if !s.len().is_multiple_of(2) {
        return Err("Hex string must have even length".to_string());
    }

    (0..s.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&s[i..i + 2], 16)
                .map_err(|e| format!("Invalid hex at position {}: {}", i, e))
        })
        .collect()
}

// SimCommands enum removed - sim now uses flags directly with 3D as default

fn main() {
    // First, try to handle as a plugin command before clap parsing
    // This allows plugins to be invoked as: `horus <plugin-name> [args...]`
    let args: Vec<String> = std::env::args().collect();

    // If there's at least one argument (besides program name) and it's not a built-in command
    if args.len() >= 2 {
        let potential_command = &args[1];

        // Skip if it's a built-in command, help flag, or version flag
        let is_builtin = matches!(
            potential_command.as_str(),
            "init"
                | "new"
                | "run"
                | "check"
                | "monitor"
                | "topic"
                | "node"
                | "doctor"
                | "clean"
                | "launch"
                | "msg"
                | "log"
                | "pkg"
                | "env"
                | "auth"
                | "sim2d"
                | "sim3d"
                | "driver"
                | "deploy"
                | "record"
                | "completion"
                | "help"
                | "--help"
                | "-h"
                | "--version"
                | "-V"
        );

        if !is_builtin && !potential_command.starts_with('-') {
            // Try to execute as plugin
            if let Ok(executor) = horus_manager::plugins::PluginExecutor::new() {
                let plugin_args: Vec<String> = args.iter().skip(2).cloned().collect();
                match executor.try_execute(potential_command, &plugin_args) {
                    Ok(Some(exit_code)) => {
                        // Plugin was found and executed - exit with the same code
                        std::process::exit(exit_code);
                    }
                    Ok(None) => {
                        // Not a plugin, fall through to normal clap parsing
                    }
                    Err(e) => {
                        // Plugin found but execution failed
                        eprintln!("{} {}", "Error:".red().bold(), e);
                        std::process::exit(1);
                    }
                }
            }
        }
    }

    // Normal clap parsing
    let cli = Cli::parse();

    if let Err(e) = run_command(cli.command) {
        eprintln!("{} {}", "Error:".red().bold(), e);
        std::process::exit(1);
    }
}

fn run_command(command: Commands) -> HorusResult<()> {
    match command {
        Commands::Init { name } => {
            commands::init::run_init(name).map_err(|e| HorusError::Config(e.to_string()))
        }

        Commands::New {
            name,
            path,
            python,
            rust,
            use_macro,
        } => {
            let language = if python {
                "python"
            } else if rust || use_macro {
                "rust"
            } else {
                "" // Will use interactive prompt
            };

            commands::new::create_new_project(name, path, language.to_string(), use_macro)
                .map_err(|e| HorusError::Config(e.to_string()))
        }

        Commands::Run {
            files,
            release,
            clean,
            quiet,
            drivers,
            enable,
            args,
            record,
        } => {
            // Set quiet mode for progress indicators
            horus_manager::progress::set_quiet(quiet);

            // Store drivers override in environment variable for later use
            if let Some(ref driver_list) = drivers {
                std::env::set_var("HORUS_DRIVERS", driver_list.join(","));
            }

            // Store enable capabilities in environment variable for later use
            if let Some(ref enable_list) = enable {
                std::env::set_var("HORUS_ENABLE", enable_list.join(","));
            }

            // If recording enabled, set environment variable for nodes to pick up
            if let Some(ref session_name) = record {
                std::env::set_var("HORUS_RECORD_SESSION", session_name);
                println!(
                    "{} Recording enabled: session '{}'",
                    "[RECORD]".yellow().bold(),
                    session_name
                );
            }

            // Build and run
            commands::run::execute_run(files, args, release, clean)
                .map_err(|e| HorusError::Config(e.to_string()))
        }

        Commands::Build {
            files,
            release,
            clean,
            quiet,
            drivers,
            enable,
        } => {
            // Set quiet mode for progress indicators
            horus_manager::progress::set_quiet(quiet);

            // Store drivers override in environment variable for later use
            if let Some(ref driver_list) = drivers {
                std::env::set_var("HORUS_DRIVERS", driver_list.join(","));
            }

            // Store enable capabilities in environment variable for later use
            if let Some(ref enable_list) = enable {
                std::env::set_var("HORUS_ENABLE", enable_list.join(","));
            }

            // Build only - compile but don't execute
            commands::run::execute_build_only(files, release, clean)
                .map_err(|e| HorusError::Config(e.to_string()))
        }

        Commands::Check { path, quiet } => {
            use horus_manager::commands::run::parse_horus_yaml_dependencies_v2;
            use horus_manager::dependency_resolver::DependencySource;
            use std::collections::HashSet;
            use walkdir::WalkDir;

            let target_path = path.unwrap_or_else(|| PathBuf::from("."));

            if !target_path.exists() {
                println!(
                    "{} Path not found: {}",
                    "[FAIL]".red(),
                    target_path.display()
                );
                return Err(HorusError::Config("Path not found".to_string()));
            }

            // Check if it's a directory (workspace scan) or single file
            if target_path.is_dir() {
                println!(
                    "{} Scanning workspace: {}\n",
                    "".cyan().bold(),
                    target_path
                        .canonicalize()
                        .unwrap_or(target_path.clone())
                        .display()
                );

                let mut total_errors = 0;
                let mut total_warnings = 0;
                let mut files_checked = 0;
                let mut horus_yamls: Vec<PathBuf> = Vec::new();
                let mut rust_files: Vec<PathBuf> = Vec::new();
                let mut python_files: Vec<PathBuf> = Vec::new();

                // Collect all files to check
                for entry in WalkDir::new(&target_path)
                    .into_iter()
                    .filter_entry(|e| {
                        let name = e.file_name().to_string_lossy();
                        // Skip hidden dirs, target, node_modules, __pycache__, .horus
                        !name.starts_with('.')
                            && name != "target"
                            && name != "node_modules"
                            && name != "__pycache__"
                            && name != ".horus"
                    })
                    .filter_map(|e| e.ok())
                {
                    let path = entry.path();
                    if path.is_file() {
                        let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
                        let ext = path.extension().and_then(|e| e.to_str());

                        if filename == "horus.yaml" {
                            horus_yamls.push(path.to_path_buf());
                        } else if ext == Some("rs") && filename != "build.rs" {
                            rust_files.push(path.to_path_buf());
                        } else if ext == Some("py") {
                            python_files.push(path.to_path_buf());
                        }
                    }
                }

                println!("  Found {} horus.yaml file(s)", horus_yamls.len());
                println!("  Found {} Rust file(s)", rust_files.len());
                println!("  Found {} Python file(s)\n", python_files.len());

                // Find Cargo.toml directories for deep Rust checking
                let mut cargo_dirs: HashSet<PathBuf> = HashSet::new();
                for entry in WalkDir::new(&target_path)
                    .into_iter()
                    .filter_entry(|e| {
                        let name = e.file_name().to_string_lossy();
                        !name.starts_with('.') && name != "target" && name != "node_modules"
                    })
                    .filter_map(|e| e.ok())
                {
                    if entry.file_name() == "Cargo.toml" {
                        if let Some(parent) = entry.path().parent() {
                            cargo_dirs.insert(parent.to_path_buf());
                        }
                    }
                }

                // ═══════════════════════════════════════════════════════════
                // PHASE 1: Validate horus.yaml manifests
                // ═══════════════════════════════════════════════════════════
                if !horus_yamls.is_empty() {
                    println!("{}", "━".repeat(60).dimmed());
                    println!(
                        "{} Phase 1: Validating horus.yaml manifests...\n",
                        "".cyan().bold()
                    );

                    for yaml_path in &horus_yamls {
                        let rel_path = yaml_path.strip_prefix(&target_path).unwrap_or(yaml_path);
                        println!("  {} {}", "".cyan(), rel_path.display());

                        match fs::read_to_string(yaml_path) {
                            Ok(content) => {
                                match serde_yaml::from_str::<serde_yaml::Value>(&content) {
                                    Ok(yaml) => {
                                        let mut file_errors: Vec<String> = Vec::new();
                                        let base_dir = yaml_path.parent().unwrap_or(Path::new("."));

                                        // Required fields
                                        if yaml.get("name").is_none() {
                                            file_errors.push("missing 'name' field".to_string());
                                        }
                                        let language =
                                            yaml.get("language").and_then(|l| l.as_str());
                                        if language.is_none() {
                                            file_errors
                                                .push("missing 'language' field".to_string());
                                        }

                                        // Check main file exists
                                        if let Some(lang) = language {
                                            let main_exists = match lang {
                                                "rust" => {
                                                    base_dir.join("main.rs").exists()
                                                        || base_dir.join("src/main.rs").exists()
                                                        || base_dir.join("Cargo.toml").exists()
                                                }
                                                "python" => base_dir.join("main.py").exists(),
                                                _ => true,
                                            };
                                            if !main_exists {
                                                file_errors.push(format!(
                                                    "main file not found for '{}'",
                                                    lang
                                                ));
                                            }
                                        }

                                        // Validate path dependencies exist
                                        if let Ok(deps) = parse_horus_yaml_dependencies_v2(
                                            yaml_path.to_str().unwrap_or(""),
                                        ) {
                                            for dep in &deps {
                                                if let DependencySource::Path(path_str) =
                                                    &dep.source
                                                {
                                                    let dep_path =
                                                        if Path::new(path_str).is_absolute() {
                                                            PathBuf::from(path_str)
                                                        } else {
                                                            base_dir.join(path_str)
                                                        };
                                                    if !dep_path.exists() {
                                                        file_errors.push(format!(
                                                            "dependency '{}' path not found: {}",
                                                            dep.name,
                                                            dep_path.display()
                                                        ));
                                                    }
                                                }
                                            }
                                        }

                                        if file_errors.is_empty() {
                                            println!("      {} manifest valid", "".green());
                                        } else {
                                            for err in &file_errors {
                                                println!("      {} {}", "".red(), err);
                                            }
                                            total_errors += file_errors.len();
                                        }
                                    }
                                    Err(e) => {
                                        println!("      {} YAML parse error: {}", "".red(), e);
                                        total_errors += 1;
                                    }
                                }
                            }
                            Err(e) => {
                                println!("      {} Read error: {}", "".red(), e);
                                total_errors += 1;
                            }
                        }
                        files_checked += 1;
                    }
                }

                // ═══════════════════════════════════════════════════════════
                // PHASE 2: Deep Rust compilation check (cargo check)
                // ═══════════════════════════════════════════════════════════
                if !cargo_dirs.is_empty() {
                    println!("\n{}", "━".repeat(60).dimmed());
                    println!(
                        "{} Phase 2: Deep Rust check (cargo check)...\n",
                        "".cyan().bold()
                    );

                    for cargo_dir in &cargo_dirs {
                        let rel_path = cargo_dir.strip_prefix(&target_path).unwrap_or(cargo_dir);
                        let display_path = if rel_path.as_os_str().is_empty() {
                            "."
                        } else {
                            rel_path.to_str().unwrap_or(".")
                        };
                        print!("  {} {} ... ", "".cyan(), display_path);
                        std::io::Write::flush(&mut std::io::stdout()).ok();

                        let output = std::process::Command::new("cargo")
                            .arg("check")
                            .arg("--message-format=short")
                            .current_dir(cargo_dir)
                            .output();

                        match output {
                            Ok(result) if result.status.success() => {
                                println!("{}", "".green());
                            }
                            Ok(result) => {
                                println!("{}", "".red());
                                let stderr = String::from_utf8_lossy(&result.stderr);
                                for line in stderr.lines().take(5) {
                                    if line.contains("error") {
                                        println!("      {} {}", "".red(), line.trim());
                                    }
                                }
                                total_errors += 1;
                            }
                            Err(e) => {
                                println!("{} cargo error: {}", "".yellow(), e);
                                total_warnings += 1;
                            }
                        }
                        files_checked += 1;
                    }
                }

                // ═══════════════════════════════════════════════════════════
                // PHASE 3: Python validation (syntax + imports)
                // ═══════════════════════════════════════════════════════════
                if !python_files.is_empty() {
                    println!("\n{}", "━".repeat(60).dimmed());
                    println!(
                        "{} Phase 3: Python validation (syntax + imports)...\n",
                        "".cyan().bold()
                    );

                    for py_path in &python_files {
                        print!(
                            "  {} {} ",
                            "".cyan(),
                            py_path
                                .strip_prefix(&target_path)
                                .unwrap_or(py_path)
                                .display()
                        );

                        // Syntax check
                        let syntax_check = std::process::Command::new("python3")
                            .arg("-m")
                            .arg("py_compile")
                            .arg(py_path)
                            .output();

                        match syntax_check {
                            Ok(result) if result.status.success() => {
                                // Syntax OK - now check imports
                                let import_script = format!(
                                    r#"
import ast, sys
try:
    with open('{}') as f:
        tree = ast.parse(f.read())
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.add(alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module.split('.')[0])
    for imp in imports:
        if imp not in ('__future__',):
            __import__(imp)
except ModuleNotFoundError as e:
    print(f'ModuleNotFoundError: {{e.name}}', file=sys.stderr)
    sys.exit(1)
except ImportError as e:
    print(f'ImportError: {{e}}', file=sys.stderr)
    sys.exit(1)
"#,
                                    py_path.display()
                                );

                                let import_check = std::process::Command::new("python3")
                                    .arg("-c")
                                    .arg(&import_script)
                                    .output();

                                match import_check {
                                    Ok(r) if r.status.success() => {
                                        println!("{}", "".green());
                                    }
                                    Ok(r) => {
                                        println!("{}", "".yellow());
                                        let err = String::from_utf8_lossy(&r.stderr);
                                        if !err.is_empty() {
                                            println!(
                                                "      {} {}",
                                                "".yellow(),
                                                err.lines().next().unwrap_or("").trim()
                                            );
                                        }
                                        total_warnings += 1;
                                    }
                                    Err(_) => println!("{}", "".green()),
                                }
                            }
                            Ok(result) => {
                                println!("{}", "".red());
                                let error = String::from_utf8_lossy(&result.stderr);
                                if !error.is_empty() {
                                    println!(
                                        "      {} {}",
                                        "".red(),
                                        error.lines().next().unwrap_or("").trim()
                                    );
                                }
                                total_errors += 1;
                            }
                            Err(_) => {
                                println!("{}", "⊘".dimmed());
                                if !quiet {
                                    total_warnings += 1;
                                }
                            }
                        }
                        files_checked += 1;
                    }
                }

                // Summary
                println!("\n{}", "━".repeat(60).dimmed());
                println!("{} Workspace Check Summary\n", "".cyan().bold());
                println!("  Files checked: {}", files_checked);

                if total_errors == 0 && total_warnings == 0 {
                    println!("  Status: {} All checks passed!", "".green());
                } else {
                    if total_errors > 0 {
                        println!("  Errors: {} {}", "".red(), total_errors);
                    }
                    if total_warnings > 0 && !quiet {
                        println!("  Warnings: {} {}", "".yellow(), total_warnings);
                    }
                }
                println!();

                if total_errors > 0 {
                    return Err(HorusError::Config(format!(
                        "{} error(s) found",
                        total_errors
                    )));
                }
                return Ok(());
            }

            // Single file check (existing logic)
            let horus_yaml_path = target_path;

            // Check if it's a source file (.rs, .py) or horus.yaml
            let extension = horus_yaml_path.extension().and_then(|s| s.to_str());

            match extension {
                Some("rs") => {
                    // Check Rust file
                    println!(
                        "{} Checking Rust file: {}\n",
                        "".cyan(),
                        horus_yaml_path.display()
                    );

                    print!("  {} Parsing Rust syntax... ", "".cyan());
                    let content = fs::read_to_string(&horus_yaml_path)?;

                    // Use syn to parse Rust code
                    match syn::parse_file(&content) {
                        Ok(_) => {
                            println!("{}", "".green());
                            println!("\n{} Syntax check passed!", "".green().bold());
                        }
                        Err(e) => {
                            println!("{}", "".red());
                            println!("\n{} Syntax error:", "[FAIL]".red().bold());
                            println!("  {}", e);
                            return Err(HorusError::Config(format!("Rust syntax error: {}", e)));
                        }
                    }

                    // Check hardware requirements
                    use horus_manager::commands::run::check_hardware_requirements;
                    if let Err(e) = check_hardware_requirements(&horus_yaml_path, "rust") {
                        eprintln!("\n{} Hardware check error: {}", "[WARNING]".yellow(), e);
                    }

                    return Ok(());
                }
                Some("py") => {
                    // Check Python file
                    println!(
                        "{} Checking Python file: {}\n",
                        "".cyan(),
                        horus_yaml_path.display()
                    );

                    print!("  {} Parsing Python syntax... ", "".cyan());

                    // Use python3 to check syntax
                    let output = std::process::Command::new("python3")
                        .arg("-m")
                        .arg("py_compile")
                        .arg(&horus_yaml_path)
                        .output();

                    match output {
                        Ok(result) if result.status.success() => {
                            println!("{}", "".green());
                            println!("\n{} Syntax check passed!", "".green().bold());
                        }
                        Ok(result) => {
                            println!("{}", "".red());
                            let error = String::from_utf8_lossy(&result.stderr);
                            println!("\n{} Syntax error:", "[FAIL]".red().bold());
                            println!("  {}", error);
                            return Err(HorusError::Config(format!(
                                "Python syntax error: {}",
                                error
                            )));
                        }
                        Err(e) => {
                            println!("{}", "[WARNING]".yellow());
                            println!(
                                "\n{} Could not check Python syntax (python3 not found): {}",
                                "[WARNING]".yellow(),
                                e
                            );
                        }
                    }
                    return Ok(());
                }
                _ => {
                    // Assume it's horus.yaml or yaml file
                    println!("{} Checking {}...\n", "".cyan(), horus_yaml_path.display());
                }
            }

            let mut errors = Vec::new();
            let mut warn_msgs = Vec::new();
            let base_dir = horus_yaml_path.parent().unwrap_or(Path::new("."));

            // 1. YAML Syntax Validation
            print!("  {} Validating YAML syntax... ", "".cyan());
            let yaml_content = match fs::read_to_string(&horus_yaml_path) {
                Ok(content) => {
                    println!("{}", "".green());
                    content
                }
                Err(e) => {
                    println!("{}", "".red());
                    errors.push(format!("Cannot read file: {}", e));
                    String::new()
                }
            };

            let yaml_value: Option<serde_yaml::Value> = if !yaml_content.is_empty() {
                match serde_yaml::from_str(&yaml_content) {
                    Ok(val) => Some(val),
                    Err(e) => {
                        errors.push(format!("Invalid YAML syntax: {}", e));
                        None
                    }
                }
            } else {
                None
            };

            // 2. Required Fields Check
            if let Some(ref yaml) = yaml_value {
                print!("  {} Checking required fields... ", "".cyan());
                let mut missing_fields = Vec::new();

                if yaml.get("name").is_none() {
                    missing_fields.push("name");
                }
                if yaml.get("version").is_none() {
                    missing_fields.push("version");
                }

                if missing_fields.is_empty() {
                    println!("{}", "".green());
                } else {
                    println!("{}", "".red());
                    errors.push(format!(
                        "Missing required fields: {}",
                        missing_fields.join(", ")
                    ));
                }

                // Optional fields warning
                if !quiet {
                    if yaml.get("description").is_none() {
                        warn_msgs.push("Optional field missing: description".to_string());
                    }
                    if yaml.get("author").is_none() {
                        warn_msgs.push("Optional field missing: author".to_string());
                    }
                }

                // License warning (encourage projects to declare their license)
                print!("  {} Checking license field... ", "".cyan());
                let missing_license_warning = "No license specified. Consider adding a license field (e.g., Apache-2.0, BSD-3-Clause).";
                if let Some(license) = yaml.get("license").and_then(|l| l.as_str()) {
                    if license.trim().is_empty() {
                        println!("{}", "[WARNING]".yellow());
                        warn_msgs.push(missing_license_warning.to_string());
                    } else {
                        println!("{} ({})", "".green(), license.dimmed());
                    }
                } else {
                    println!("{}", "[WARNING]".yellow());
                    warn_msgs.push(missing_license_warning.to_string());
                }

                // Language validation
                print!("  {} Validating language field... ", "".cyan());
                if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
                    if language == "rust" || language == "python" {
                        println!("{}", "".green());
                    } else {
                        println!("{}", "".red());
                        errors.push(format!(
                            "Invalid language '{}' - must be: rust or python",
                            language
                        ));
                    }
                } else {
                    println!("{}", "".red());
                    errors.push(
                        "Missing or invalid 'language' field - must be: rust or python".to_string(),
                    );
                }

                // Version format validation
                print!("  {} Validating version format... ", "".cyan());
                if let Some(version_str) = yaml.get("version").and_then(|v| v.as_str()) {
                    use semver::Version;
                    match Version::parse(version_str) {
                        Ok(_) => println!("{}", "".green()),
                        Err(e) => {
                            println!("{}", "".red());
                            errors.push(format!(
                                "Invalid version format '{}': {} (must be valid semver like 0.1.0)",
                                version_str, e
                            ));
                        }
                    }
                } else if yaml.get("version").is_some() {
                    println!("{}", "".red());
                    errors.push("Version field must be a string".to_string());
                }

                // Project name validation
                print!("  {} Validating project name... ", "".cyan());
                if let Some(name) = yaml.get("name").and_then(|n| n.as_str()) {
                    let mut name_issues = Vec::new();

                    if name.is_empty() {
                        name_issues.push("name cannot be empty");
                    }
                    if name.contains(' ') {
                        name_issues.push("name cannot contain spaces");
                    }
                    if name
                        .chars()
                        .any(|c| !c.is_ascii_alphanumeric() && c != '_' && c != '-')
                    {
                        name_issues.push(
                            "name can only contain letters, numbers, hyphens, and underscores",
                        );
                    }

                    if name_issues.is_empty() {
                        println!("{}", "".green());
                        // Warn if uppercase
                        if !quiet && name.chars().any(|c| c.is_uppercase()) {
                            warn_msgs.push(format!(
                                "Project name '{}' contains uppercase - consider using lowercase",
                                name
                            ));
                        }
                    } else {
                        println!("{}", "".red());
                        for issue in name_issues {
                            errors.push(format!("Invalid project name: {}", issue));
                        }
                    }
                }

                // Main file existence check
                print!("  {} Checking for main file... ", "".cyan());
                if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
                    let main_files = match language {
                        "rust" => vec!["main.rs", "src/main.rs"],
                        "python" => vec!["main.py"],
                        _ => vec![],
                    };

                    let mut found = false;
                    for main_file in &main_files {
                        let path = base_dir.join(main_file);
                        if path.exists() {
                            println!("{}", "".green());
                            found = true;
                            break;
                        }
                    }

                    if !found && !main_files.is_empty() {
                        println!("{}", "".yellow());
                        if !quiet {
                            warn_msgs.push(format!(
                                "No main file found - expected one of: {}",
                                main_files.join(", ")
                            ));
                        }
                    }
                } else {
                    println!("{}", "⊘".dimmed());
                }
            }

            // 3. Parse Dependencies
            print!("  {} Parsing dependencies... ", "".cyan());
            let dep_specs =
                match parse_horus_yaml_dependencies_v2(horus_yaml_path.to_str().unwrap()) {
                    Ok(specs) => {
                        println!("{}", "".green());
                        specs
                    }
                    Err(e) => {
                        println!("{}", "".red());
                        errors.push(format!("Failed to parse dependencies: {}", e));
                        Vec::new()
                    }
                };

            // 4. Check for Duplicates
            if !dep_specs.is_empty() {
                print!("  {} Checking for duplicates... ", "".cyan());
                let mut seen = HashSet::new();
                let mut duplicates = Vec::new();

                for spec in &dep_specs {
                    if !seen.insert(&spec.name) {
                        duplicates.push(spec.name.clone());
                    }
                }

                if duplicates.is_empty() {
                    println!("{}", "".green());
                } else {
                    println!("{}", "".red());
                    errors.push(format!("Duplicate dependencies: {}", duplicates.join(", ")));
                }
            }

            // 5. Validate Path Dependencies
            println!("\n  {} Checking path dependencies...", "".cyan());
            let mut path_deps_found = false;

            for spec in &dep_specs {
                use horus_manager::dependency_resolver::DependencySource;
                if let DependencySource::Path(ref path) = spec.source {
                    path_deps_found = true;
                    let resolved_path = if path.is_absolute() {
                        path.clone()
                    } else {
                        base_dir.join(path)
                    };

                    if resolved_path.exists() {
                        if resolved_path.is_dir() {
                            println!("    {} {} ({})", "".green(), spec.name, path.display());
                        } else {
                            println!(
                                "    {} {} ({}) - Not a directory",
                                "[FAIL]".red(),
                                spec.name,
                                path.display()
                            );
                            errors.push(format!(
                                "Path dependency '{}' is not a directory: {}",
                                spec.name,
                                path.display()
                            ));
                        }
                    } else {
                        println!(
                            "    {} {} ({}) - Path not found",
                            "[FAIL]".red(),
                            spec.name,
                            path.display()
                        );
                        errors.push(format!(
                            "Path dependency '{}' not found: {}",
                            spec.name,
                            path.display()
                        ));
                    }
                }
            }

            if !path_deps_found {
                println!("    {} No path dependencies", "".dimmed());
            }

            // 6. Circular Dependency Detection (Simple Check)
            println!("\n  {} Checking for circular dependencies...", "".cyan());
            let mut circular_found = false;

            for spec in &dep_specs {
                use horus_manager::dependency_resolver::DependencySource;
                if let DependencySource::Path(ref path) = spec.source {
                    let resolved_path = if path.is_absolute() {
                        path.clone()
                    } else {
                        base_dir.join(path)
                    };

                    // Check if target has horus.yaml
                    let target_yaml = resolved_path.join("horus.yaml");
                    if target_yaml.exists() {
                        // Check if it references us back
                        if let Ok(target_deps) =
                            parse_horus_yaml_dependencies_v2(target_yaml.to_str().unwrap())
                        {
                            let our_name = yaml_value
                                .as_ref()
                                .and_then(|y| y.get("name"))
                                .and_then(|n| n.as_str())
                                .unwrap_or("");

                            for target_dep in target_deps {
                                if target_dep.name == our_name {
                                    if let DependencySource::Path(_) = target_dep.source {
                                        circular_found = true;
                                        errors.push(format!(
                                            "Circular dependency detected: {} -> {} -> {}",
                                            our_name, spec.name, our_name
                                        ));
                                        println!(
                                            "    {} Circular: {} <-> {}",
                                            "[FAIL]".red(),
                                            our_name,
                                            spec.name
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if !circular_found {
                println!("    {} No circular dependencies", "".green());
            }

            // 7. Version Constraint Validation
            print!("\n  {} Validating version constraints... ", "".cyan());

            for spec in &dep_specs {
                // Version requirement is already parsed in DependencySpec
                // If it parsed successfully, it's valid
                // But we can check for common mistakes
                if spec.requirement.to_string() == "*" && !quiet {
                    warn_msgs.push(format!("Dependency '{}' uses wildcard version (*) - consider pinning to a specific version", spec.name));
                }
            }

            println!("{}", "".green());

            // 8. Workspace Structure Check
            print!("\n  {} Checking workspace structure... ", "".cyan());
            let base_dir = horus_yaml_path.parent().unwrap_or_else(|| Path::new("."));
            let horus_dir = base_dir.join(".horus");

            if horus_dir.exists() && horus_dir.is_dir() {
                println!("{}", "".green());
            } else {
                println!("{}", "".yellow());
                if !quiet {
                    warn_msgs.push(
                        "No .horus/ workspace directory found - will be created on first run"
                            .to_string(),
                    );
                }
            }

            // 9. Dependency Installation Check
            print!("  {} Checking installed dependencies... ", "".cyan());
            if horus_dir.exists() {
                let packages_dir = horus_dir.join("packages");
                if packages_dir.exists() {
                    let mut missing_deps = Vec::new();

                    for spec in &dep_specs {
                        match &spec.source {
                            DependencySource::Registry => {
                                let package_dir = packages_dir.join(&spec.name);
                                if !package_dir.exists() {
                                    missing_deps.push(spec.name.clone());
                                }
                            }
                            DependencySource::Path(_) => {
                                // Path deps checked separately
                            }
                            DependencySource::Git { .. } => {
                                // Git deps are cloned by horus run, skip here
                            }
                            DependencySource::Pip { package_name } => {
                                // Pip deps are installed to venv, check later during run
                                // We could check if the package is in .horus/venv but that's complex
                                // Just skip for status check - horus run will handle it
                                let _ = package_name; // suppress unused warning
                            }
                        }
                    }

                    if missing_deps.is_empty() {
                        println!("{}", "".green());
                    } else {
                        println!("{}", "".yellow());
                        if !missing_deps.is_empty() && !quiet {
                            warn_msgs.push(format!(
                                "Missing dependencies: {} (run 'horus run' to install)",
                                missing_deps.join(", ")
                            ));
                        }
                    }
                } else {
                    println!("{}", "".yellow());
                    if !quiet {
                        warn_msgs.push(
                            "No packages directory - dependencies not installed yet".to_string(),
                        );
                    }
                }
            } else {
                println!("{}", "⊘".dimmed());
            }

            // 10. Toolchain Check
            print!("  {} Checking toolchain... ", "".cyan());
            if let Some(ref yaml) = yaml_value {
                if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
                    let toolchain_available = match language {
                        "rust" => std::process::Command::new("rustc")
                            .arg("--version")
                            .output()
                            .map(|o| o.status.success())
                            .unwrap_or(false),
                        "python" => std::process::Command::new("python3")
                            .arg("--version")
                            .output()
                            .map(|o| o.status.success())
                            .unwrap_or(false),
                        _ => false,
                    };

                    if toolchain_available {
                        println!("{}", "".green());
                    } else {
                        println!("{}", "".red());
                        errors.push(format!(
                            "Required toolchain for '{}' not found in PATH",
                            language
                        ));
                    }
                } else {
                    println!("{}", "⊘".dimmed());
                }
            } else {
                println!("{}", "⊘".dimmed());
            }

            // 11. Code Validation (optional, can be slow)
            print!("  {} Validating code syntax... ", "".cyan());
            if let Some(ref yaml) = yaml_value {
                if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
                    match language {
                        "rust" => {
                            // Check for Cargo.toml or main.rs
                            let has_cargo = base_dir.join("Cargo.toml").exists();
                            let has_main = base_dir.join("main.rs").exists()
                                || base_dir.join("src/main.rs").exists();

                            if has_cargo || has_main {
                                let check_result = std::process::Command::new("cargo")
                                    .arg("build")
                                    .arg("--quiet")
                                    .current_dir(base_dir)
                                    .output();

                                match check_result {
                                    Ok(output) if output.status.success() => {
                                        println!("{}", "".green());
                                    }
                                    Ok(_) => {
                                        println!("{}", "".red());
                                        errors.push("Rust code has compilation errors (run 'cargo build' for details)".to_string());
                                    }
                                    Err(_) => {
                                        println!("{}", "".yellow());
                                        if !quiet {
                                            warn_msgs.push("Could not run 'cargo build' - skipping code validation".to_string());
                                        }
                                    }
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }
                        }
                        "python" => {
                            // Check main.py syntax
                            let main_py = base_dir.join("main.py");
                            if main_py.exists() {
                                let check_result = std::process::Command::new("python3")
                                    .arg("-m")
                                    .arg("py_compile")
                                    .arg(&main_py)
                                    .output();

                                match check_result {
                                    Ok(output) if output.status.success() => {
                                        println!("{}", "".green());
                                    }
                                    Ok(_) => {
                                        println!("{}", "".red());
                                        errors.push("Python code has syntax errors".to_string());
                                    }
                                    Err(_) => {
                                        println!("{}", "".yellow());
                                        if !quiet {
                                            warn_msgs.push(
                                                "Could not validate Python syntax".to_string(),
                                            );
                                        }
                                    }
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }
                        }
                        _ => {
                            println!("{}", "⊘".dimmed());
                        }
                    }
                } else {
                    println!("{}", "⊘".dimmed());
                }
            } else {
                println!("{}", "⊘".dimmed());
            }

            // 12. HORUS System Check
            print!("\n  {} Checking HORUS installation... ", "".cyan());
            let horus_version = env!("CARGO_PKG_VERSION");
            println!("v{}", horus_version.dimmed());

            // 13. Registry Connectivity
            print!("  {} Checking registry connectivity... ", "".cyan());
            // Simple connectivity check - try to connect to registry
            let registry_available = std::process::Command::new("ping")
                .arg("-c")
                .arg("1")
                .arg("-W")
                .arg("1")
                .arg("registry.horus.rs")
                .output()
                .map(|o| o.status.success())
                .unwrap_or(false);

            if registry_available {
                println!("{}", "".green());
            } else {
                println!("{}", "⊘".dimmed());
                if !quiet {
                    warn_msgs
                        .push("Registry not reachable - package installation may fail".to_string());
                }
            }

            // 14. System Requirements Check
            print!("  {} Checking system requirements... ", "".cyan());
            let mut sys_issues = Vec::new();

            // Check shared memory directory availability (cross-platform)
            #[allow(unused_variables)] // Used in non-Linux cfg block
            let shm_path = shm_base_dir();
            // On Linux, check /dev/shm permissions; on other platforms, just check the base dir
            if has_native_shm() {
                #[cfg(target_os = "linux")]
                {
                    let dev_shm = std::path::Path::new("/dev/shm");
                    if !dev_shm.exists() {
                        sys_issues.push("/dev/shm not available");
                    } else if let Ok(metadata) = std::fs::metadata(dev_shm) {
                        use std::os::unix::fs::PermissionsExt;
                        let mode = metadata.permissions().mode();
                        if mode & 0o777 != 0o777 {
                            sys_issues.push("/dev/shm permissions restrictive");
                        }
                    }
                }
            }
            // For non-Linux platforms, ensure the shared memory directory can be created
            #[cfg(not(target_os = "linux"))]
            {
                if let Err(_) = std::fs::create_dir_all(&shm_path) {
                    sys_issues.push("Cannot create shared memory directory");
                }
            }

            if sys_issues.is_empty() {
                println!("{}", "".green());
            } else {
                println!("{}", "".yellow());
                for issue in sys_issues {
                    if !quiet {
                        warn_msgs.push(format!("System issue: {}", issue));
                    }
                }
            }

            // Disk Space Check
            print!("  {} Checking available disk space... ", "".cyan());
            #[cfg(target_os = "linux")]
            {
                use std::process::Command;

                // Check available space in current directory (where .horus will be created)
                if let Ok(output) = Command::new("df").arg("-BM").arg(base_dir).output() {
                    if output.status.success() {
                        let output_str = String::from_utf8_lossy(&output.stdout);
                        // Parse df output: Filesystem  1M-blocks  Used Available Use% Mounted
                        if let Some(line) = output_str.lines().nth(1) {
                            let parts: Vec<&str> = line.split_whitespace().collect();
                            if parts.len() >= 4 {
                                if let Some(available) = parts[3].strip_suffix('M') {
                                    if let Ok(available_mb) = available.parse::<u64>() {
                                        // Warn if less than 500MB available
                                        if available_mb < 500 {
                                            println!("{} ({}MB free)", "".yellow(), available_mb);
                                            if !quiet {
                                                warn_msgs.push(format!(
                                                    "Low disk space: only {}MB available (recommended: 500MB+)",
                                                    available_mb
                                                ));
                                            }
                                        } else if available_mb < 100 {
                                            println!("{} ({}MB free)", "".red(), available_mb);
                                            errors.push(format!(
                                                "Critically low disk space: only {}MB available",
                                                available_mb
                                            ));
                                        } else {
                                            println!("{} ({}MB free)", "".green(), available_mb);
                                        }
                                    } else {
                                        println!("{}", "⊘".dimmed());
                                    }
                                } else {
                                    println!("{}", "⊘".dimmed());
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }
                        } else {
                            println!("{}", "⊘".dimmed());
                        }
                    } else {
                        println!("{}", "⊘".dimmed());
                    }
                } else {
                    println!("{}", "⊘".dimmed());
                }
            }
            #[cfg(not(target_os = "linux"))]
            {
                println!("{}", "⊘".dimmed());
            }

            // 15. API Usage Check (basic pattern matching)
            print!("  {} Checking API usage... ", "".cyan());
            if let Some(ref yaml) = yaml_value {
                if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
                    match language {
                        "rust" => {
                            // Check if using HORUS dependencies
                            let uses_horus = dep_specs
                                .iter()
                                .any(|spec| spec.name == "horus" || spec.name == "horus_macros");

                            if uses_horus {
                                // Quick check for common patterns in main.rs or src/main.rs
                                let main_paths =
                                    vec![base_dir.join("main.rs"), base_dir.join("src/main.rs")];

                                let mut has_scheduler = false;
                                for main_path in main_paths {
                                    if main_path.exists() {
                                        if let Ok(content) = std::fs::read_to_string(&main_path) {
                                            has_scheduler = content.contains("Scheduler::new")
                                                || content.contains("scheduler.register");
                                            if has_scheduler {
                                                break;
                                            }
                                        }
                                    }
                                }

                                if has_scheduler {
                                    println!("{}", "".green());
                                } else {
                                    println!("{}", "".yellow());
                                    if !quiet {
                                        warn_msgs.push("HORUS dependency found but no Scheduler usage detected".to_string());
                                    }
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }
                        }
                        "python" => {
                            let uses_horus = dep_specs.iter().any(|spec| spec.name == "horus_py");

                            if uses_horus {
                                let main_py = base_dir.join("main.py");
                                if main_py.exists() {
                                    if let Ok(content) = std::fs::read_to_string(&main_py) {
                                        if content.contains("import horus")
                                            || content.contains("from horus")
                                        {
                                            println!("{}", "".green());
                                        } else {
                                            println!("{}", "".yellow());
                                            if !quiet {
                                                warn_msgs.push("horus_py dependency but no 'import horus' found".to_string());
                                            }
                                        }
                                    } else {
                                        println!("{}", "⊘".dimmed());
                                    }
                                } else {
                                    println!("{}", "⊘".dimmed());
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }

                            // Check external Python dependencies
                            print!("  {} Checking Python external dependencies... ", "".cyan());
                            let main_py = base_dir.join("main.py");
                            if main_py.exists() {
                                match parse_python_imports(&main_py) {
                                    Ok(imports) if !imports.is_empty() => {
                                        let mut missing_packages = Vec::new();
                                        for package in &imports {
                                            if !check_system_package_exists(package) {
                                                missing_packages.push(package.clone());
                                            }
                                        }

                                        if missing_packages.is_empty() {
                                            println!("{} ({})", "".green(), imports.len());
                                        } else {
                                            println!("{}", "".red());
                                            errors.push(format!(
                                                "Missing Python packages: {} (install with: pip install {})",
                                                missing_packages.join(", "),
                                                missing_packages.join(" ")
                                            ));
                                        }
                                    }
                                    Ok(_) => {
                                        println!("{}", "⊘".dimmed());
                                    }
                                    Err(e) => {
                                        println!("{}", "".yellow());
                                        if !quiet {
                                            warn_msgs.push(format!(
                                                "Could not parse Python imports: {}",
                                                e
                                            ));
                                        }
                                    }
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }
                        }
                        _ => {
                            println!("{}", "⊘".dimmed());
                        }
                    }
                } else {
                    println!("{}", "⊘".dimmed());
                }
            } else {
                println!("{}", "⊘".dimmed());
            }

            // Print Summary
            println!();
            if !quiet {
                if warn_msgs.is_empty() {
                    println!("{} No warnings detected.", "".green());
                } else {
                    println!("{} {} warning(s):", "[WARNING]".yellow(), warn_msgs.len());
                    for warn in &warn_msgs {
                        println!("  - {}", warn);
                    }
                }
                println!();
            }

            if errors.is_empty() {
                println!("{} All checks passed!", "".green().bold());
                Ok(())
            } else {
                println!(
                    "{} {} error(s) found:\n",
                    "[FAIL]".red().bold(),
                    errors.len()
                );
                for (i, err) in errors.iter().enumerate() {
                    println!("  {}. {}", i + 1, err);
                }
                println!();
                Err(HorusError::Config("Validation failed".to_string()))
            }
        }

        Commands::Test {
            filter,
            release,
            nocapture,
            test_threads,
            parallel,
            simulation,
            integration,
            no_build,
            no_cleanup,
            verbose,
            drivers,
            enable,
        } => {
            // Store drivers override in environment variable for later use
            if let Some(ref driver_list) = drivers {
                std::env::set_var("HORUS_DRIVERS", driver_list.join(","));
            }

            // Store enable capabilities in environment variable for later use
            if let Some(ref enable_list) = enable {
                std::env::set_var("HORUS_ENABLE", enable_list.join(","));
            }

            commands::test::run_tests(
                filter,
                release,
                nocapture,
                test_threads,
                parallel,
                simulation,
                integration,
                no_build,
                no_cleanup,
                verbose,
            )
            .map_err(|e| HorusError::Config(e.to_string()))?;
            Ok(())
        }

        Commands::Monitor {
            port,
            tui,
            reset_password,
        } => {
            // Reset password if requested
            if reset_password {
                security::auth::reset_password().map_err(|e| HorusError::Config(e.to_string()))?;
            }

            if tui {
                println!("{} Opening HORUS monitor (TUI)...", "".cyan());
                // Launch TUI monitor
                monitor_tui::TuiDashboard::run().map_err(|e| HorusError::Config(e.to_string()))
            } else {
                // Default: Launch web monitor and auto-open browser
                println!(
                    "{} Starting HORUS monitor on http://localhost:{}...",
                    "".cyan(),
                    port
                );
                println!("  {} Password-protected access", "".dimmed());
                println!("  {} Opening browser...", "".dimmed());
                println!(
                    "  {} Use 'horus monitor -t' for Terminal UI",
                    "Tip:".dimmed()
                );

                tokio::runtime::Runtime::new()
                    .unwrap()
                    .block_on(monitor::run(port))
                    .map_err(|e| {
                        let err_str = e.to_string();
                        if err_str.contains("Address already in use") || err_str.contains("os error 98") {
                            HorusError::Config(format!(
                                "Port {} is already in use.\n  {} Try a different port: horus monitor <PORT>\n  {} Example: horus monitor {}",
                                port,
                                "".cyan(),
                                "".cyan(),
                                port + 1
                            ))
                        } else {
                            HorusError::Config(err_str)
                        }
                    })
            }
        }

        Commands::Topic { command } => match command {
            TopicCommands::List { verbose, json } => commands::topic::list_topics(verbose, json),
            TopicCommands::Echo { name, count, rate } => {
                commands::topic::echo_topic(&name, count, rate)
            }
            TopicCommands::Info { name } => commands::topic::topic_info(&name),
            TopicCommands::Hz { name, window } => commands::topic::topic_hz(&name, window),
            TopicCommands::Pub {
                name,
                message,
                rate,
                count,
            } => commands::topic::publish_topic(&name, &message, rate, count),
        },

        #[cfg(feature = "zenoh-transport")]
        Commands::Zenoh { command } => match command {
            ZenohCommands::List {
                verbose,
                json,
                ros2_only,
            } => commands::zenoh::list_topics(verbose, json, ros2_only),
            ZenohCommands::Info { key_expr, json } => commands::zenoh::topic_info(&key_expr, json),
            ZenohCommands::Echo {
                key_expr,
                count,
                raw,
            } => commands::zenoh::echo_topic(&key_expr, count, raw),
            ZenohCommands::Pub {
                key_expr,
                message,
                rate,
                count,
            } => commands::zenoh::publish_topic(&key_expr, &message, rate, count),
        },

        Commands::Node { command } => match command {
            NodeCommands::List {
                verbose,
                json,
                category,
            } => commands::node::list_nodes(verbose, json, category),
            NodeCommands::Info { name } => commands::node::node_info(&name),
            NodeCommands::Kill { name, force } => commands::node::kill_node(&name, force),
            NodeCommands::Restart { name } => commands::node::restart_node(&name),
            NodeCommands::Pause { name } => commands::node::pause_node(&name),
            NodeCommands::Resume { name } => commands::node::resume_node(&name),
        },

        Commands::Param { command } => match command {
            ParamCommands::List { verbose, json } => commands::param::list_params(verbose, json),
            ParamCommands::Get { key, json } => commands::param::get_param(&key, json),
            ParamCommands::Set { key, value } => commands::param::set_param(&key, &value),
            ParamCommands::Delete { key } => commands::param::delete_param(&key),
            ParamCommands::Reset { force } => commands::param::reset_params(force),
            ParamCommands::Load { file } => commands::param::load_params(&file),
            ParamCommands::Save { file } => commands::param::save_params(file.as_deref()),
            ParamCommands::Dump => commands::param::dump_params(),
        },

        Commands::Hf { command } => match command {
            HfCommands::List { verbose, json } => commands::hf::list_frames(verbose, json),
            HfCommands::Echo {
                source,
                target,
                rate,
                count,
            } => commands::hf::echo_transform(&source, &target, rate, count),
            HfCommands::Tree { output } => commands::hf::view_frames(output.as_deref()),
            HfCommands::Info { name } => commands::hf::frame_info(&name),
            HfCommands::Can { source, target } => commands::hf::can_transform(&source, &target),
            HfCommands::Hz { window } => commands::hf::monitor_rates(window),
        },

        Commands::Hardware { command } => match command {
            HardwareCommands::Scan {
                usb,
                serial,
                i2c,
                probe_i2c,
                gpio,
                cameras,
                all,
                verbose,
                json,
                filter,
                timeout_ms,
            } => {
                // If no specific flags are set, default to scanning all
                let scan_all = all || (!usb && !serial && !i2c && !gpio && !cameras);
                let options = commands::hardware::HardwareScanOptions {
                    usb: scan_all || usb,
                    serial: scan_all || serial,
                    i2c: scan_all || i2c,
                    probe_i2c,
                    spi: scan_all,
                    can: scan_all,
                    gpio: scan_all || gpio,
                    pwm: scan_all,
                    cameras: scan_all || cameras,
                    bluetooth: scan_all,
                    network: scan_all,
                    audio: scan_all,
                    verbose,
                    json,
                    filter,
                    timeout_ms,
                    watch: false,
                };
                commands::hardware::run_scan(options)
            }
            HardwareCommands::Platform { verbose } => commands::hardware::run_platform(verbose),
            HardwareCommands::Suggest { verbose } => commands::hardware::run_suggest(verbose),
            HardwareCommands::Info { device, verbose } => {
                commands::hardware::run_device_info(&device, verbose)
            }
            HardwareCommands::Export { output, verbose } => {
                commands::hardware::run_export(output, verbose)
            }
            HardwareCommands::Watch {
                usb,
                serial,
                i2c,
                gpio,
                cameras,
                all,
                timeout_ms,
            } => {
                // If no specific flags are set, default to watching all
                let watch_all = all || (!usb && !serial && !i2c && !gpio && !cameras);
                let options = commands::hardware::HardwareScanOptions {
                    usb: watch_all || usb,
                    serial: watch_all || serial,
                    i2c: watch_all || i2c,
                    probe_i2c: false, // Don't probe I2C in watch mode
                    spi: watch_all,
                    can: watch_all,
                    gpio: watch_all || gpio,
                    pwm: watch_all,
                    cameras: watch_all || cameras,
                    bluetooth: watch_all,
                    network: watch_all,
                    audio: watch_all,
                    verbose: false,
                    json: false,
                    filter: None,
                    timeout_ms,
                    watch: true,
                };
                commands::hardware::run_watch(options)
            }
        },

        Commands::Doctor { verbose } => commands::doctor::run_doctor(verbose),

        #[cfg(feature = "mdns")]
        Commands::Discover {
            timeout,
            topic,
            name,
            watch,
            format,
        } => commands::discover::run_discover(timeout, topic, name, watch, &format),

        Commands::Clean { shm, all, dry_run } => commands::clean::run_clean(shm, all, dry_run),

        Commands::Launch {
            file,
            dry_run,
            namespace,
            list,
        } => {
            if list {
                commands::launch::list_launch_nodes(&file)
            } else {
                commands::launch::run_launch(&file, dry_run, namespace)
            }
        }

        Commands::Msg { command } => match command {
            MsgCommands::List { verbose, filter } => {
                commands::msg::list_messages(verbose, filter.as_deref())
            }
            MsgCommands::Show { name } => commands::msg::show_message(&name),
            MsgCommands::Md5 { name } => commands::msg::message_hash(&name),
        },

        Commands::Log {
            node,
            level,
            since,
            follow,
            count,
            clear,
            clear_all,
        } => {
            if clear || clear_all {
                commands::log::clear_logs(clear_all)
            } else {
                commands::log::view_logs(
                    node.as_deref(),
                    level.as_deref(),
                    since.as_deref(),
                    follow,
                    count,
                )
            }
        }

        Commands::Pkg { command } => {
            match command {
                PkgCommands::Install {
                    package,
                    ver,
                    global,
                    target,
                } => {
                    use horus_manager::yaml_utils::{
                        add_path_dependency_to_horus_yaml, is_path_like,
                        read_package_name_from_path,
                    };

                    // Check if package is actually a path
                    if is_path_like(&package) {
                        // Path dependency installation
                        if global {
                            return Err(HorusError::Config(
                                "Cannot install path dependencies globally. Path dependencies must be local.".to_string()
                            ));
                        }

                        println!(
                            "{} Installing path dependency: {}",
                            "".cyan(),
                            package.green()
                        );

                        // Resolve path
                        let path = PathBuf::from(&package);
                        let absolute_path = if path.is_absolute() {
                            path.clone()
                        } else {
                            std::env::current_dir()
                                .map_err(|e| HorusError::Config(e.to_string()))?
                                .join(&path)
                        };

                        // Verify path exists and is a directory
                        if !absolute_path.exists() {
                            return Err(HorusError::Config(format!(
                                "Path does not exist: {}",
                                absolute_path.display()
                            )));
                        }
                        if !absolute_path.is_dir() {
                            return Err(HorusError::Config(format!(
                                "Path is not a directory: {}",
                                absolute_path.display()
                            )));
                        }

                        // Read package name from the path
                        let package_name = read_package_name_from_path(&absolute_path)
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        println!(
                            "  {} Detected package name: {}",
                            "".cyan(),
                            package_name.cyan()
                        );

                        // Determine installation target
                        let install_target = if let Some(target_name) = target {
                            let registry = workspace::WorkspaceRegistry::load()
                                .map_err(|e| HorusError::Config(e.to_string()))?;
                            let ws = registry.find_by_name(&target_name).ok_or_else(|| {
                                HorusError::Config(format!("Workspace '{}' not found", target_name))
                            })?;
                            workspace::InstallTarget::Local(ws.path.clone())
                        } else {
                            workspace::detect_or_select_workspace(true)
                                .map_err(|e| HorusError::Config(e.to_string()))?
                        };

                        // Install using install_from_path
                        let client = registry::RegistryClient::new();
                        let workspace_path = match &install_target {
                            workspace::InstallTarget::Local(p) => p.clone(),
                            _ => unreachable!(), // Already blocked global above
                        };

                        // Pass None for base_dir - CLI paths are resolved relative to current_dir
                        client
                            .install_from_path(&package_name, &absolute_path, install_target, None)
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        // Update horus.yaml with path dependency
                        let horus_yaml_path = workspace_path.join("horus.yaml");
                        if horus_yaml_path.exists() {
                            if let Err(e) = add_path_dependency_to_horus_yaml(
                                &horus_yaml_path,
                                &package_name,
                                &package, // Use original path as provided by user
                            ) {
                                println!("  {} Failed to update horus.yaml: {}", "".yellow(), e);
                            } else {
                                println!("  {} Updated horus.yaml", "".green());
                            }
                        }

                        println!("{} Path dependency installed successfully!", "".green());
                        Ok(())
                    } else {
                        // Registry dependency installation (existing logic)
                        let install_target = if global {
                            workspace::InstallTarget::Global
                        } else if let Some(target_name) = target {
                            let registry = workspace::WorkspaceRegistry::load()
                                .map_err(|e| HorusError::Config(e.to_string()))?;
                            let ws = registry.find_by_name(&target_name).ok_or_else(|| {
                                HorusError::Config(format!("Workspace '{}' not found", target_name))
                            })?;
                            workspace::InstallTarget::Local(ws.path.clone())
                        } else {
                            workspace::detect_or_select_workspace(true)
                                .map_err(|e| HorusError::Config(e.to_string()))?
                        };

                        let client = registry::RegistryClient::new();
                        client
                            .install_to_target(&package, ver.as_deref(), install_target.clone())
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        // Update horus.yaml if installing locally
                        if let workspace::InstallTarget::Local(workspace_path) = install_target {
                            let horus_yaml_path = workspace_path.join("horus.yaml");
                            if horus_yaml_path.exists() {
                                let version = ver.as_deref().unwrap_or("latest");
                                if let Err(e) =
                                    horus_manager::yaml_utils::add_dependency_to_horus_yaml(
                                        &horus_yaml_path,
                                        &package,
                                        version,
                                    )
                                {
                                    println!(
                                        "  {} Failed to update horus.yaml: {}",
                                        "".yellow(),
                                        e
                                    );
                                }
                            }
                        }

                        Ok(())
                    }
                }

                PkgCommands::Remove {
                    package,
                    global,
                    target,
                } => {
                    println!("{} Removing {}...", "".cyan(), package.yellow());

                    // Track workspace path for horus.yaml update
                    let workspace_path = if global {
                        None
                    } else if let Some(target_name) = &target {
                        let registry = workspace::WorkspaceRegistry::load()
                            .map_err(|e| HorusError::Config(e.to_string()))?;
                        let ws = registry.find_by_name(target_name).ok_or_else(|| {
                            HorusError::Config(format!("Workspace '{}' not found", target_name))
                        })?;
                        Some(ws.path.clone())
                    } else {
                        workspace::find_workspace_root()
                    };

                    let remove_dir = if global {
                        // Remove from global cache
                        let home = dirs::home_dir().ok_or_else(|| {
                            HorusError::Config("Could not find home directory".to_string())
                        })?;
                        let global_cache = home.join(".horus/cache");

                        // Find versioned directory
                        let mut found = None;
                        if global_cache.exists() {
                            for entry in fs::read_dir(&global_cache)
                                .map_err(|e| HorusError::Config(e.to_string()))?
                            {
                                let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                                let name = entry.file_name().to_string_lossy().to_string();
                                if name == package || name.starts_with(&format!("{}@", package)) {
                                    found = Some(entry.path());
                                    break;
                                }
                            }
                        }
                        found.ok_or_else(|| {
                            HorusError::Config(format!(
                                "Package {} not found in global cache",
                                package
                            ))
                        })?
                    } else if let Some(target_name) = &target {
                        // Remove from specific workspace
                        let registry = workspace::WorkspaceRegistry::load()
                            .map_err(|e| HorusError::Config(e.to_string()))?;
                        let ws = registry.find_by_name(target_name).ok_or_else(|| {
                            HorusError::Config(format!("Workspace '{}' not found", target_name))
                        })?;
                        ws.path.join(".horus/packages").join(&package)
                    } else {
                        // Remove from current workspace
                        if let Some(root) = workspace::find_workspace_root() {
                            root.join(".horus/packages").join(&package)
                        } else {
                            PathBuf::from(".horus/packages").join(&package)
                        }
                    };

                    // Check for system package reference first
                    let packages_dir = if global {
                        let home = dirs::home_dir().ok_or_else(|| {
                            HorusError::Config("Could not find home directory".to_string())
                        })?;
                        home.join(".horus/cache")
                    } else if let Some(target_name) = &target {
                        let registry = workspace::WorkspaceRegistry::load()
                            .map_err(|e| HorusError::Config(e.to_string()))?;
                        let ws = registry.find_by_name(target_name).ok_or_else(|| {
                            HorusError::Config(format!("Workspace '{}' not found", target_name))
                        })?;
                        ws.path.join(".horus/packages")
                    } else if let Some(root) = workspace::find_workspace_root() {
                        root.join(".horus/packages")
                    } else {
                        PathBuf::from(".horus/packages")
                    };

                    let system_ref = packages_dir.join(format!("{}.system.json", package));
                    if system_ref.exists() {
                        // Read to determine package type
                        let content = fs::read_to_string(&system_ref).map_err(|e| {
                            HorusError::Config(format!("Failed to read system reference: {}", e))
                        })?;
                        let metadata: serde_json::Value =
                            serde_json::from_str(&content).map_err(|e| {
                                HorusError::Config(format!(
                                    "Failed to parse system reference: {}",
                                    e
                                ))
                            })?;

                        // Remove reference file
                        fs::remove_file(&system_ref).map_err(|e| {
                            HorusError::Config(format!("Failed to remove system reference: {}", e))
                        })?;

                        // If it's a cargo package, also remove bin symlink
                        if let Some(pkg_type) = metadata.get("package_type") {
                            if pkg_type == "CratesIO" {
                                let bin_dir = if let Some(root) = workspace::find_workspace_root() {
                                    root.join(".horus/bin")
                                } else {
                                    PathBuf::from(".horus/bin")
                                };
                                let bin_link = bin_dir.join(&package);
                                if bin_link.exists() || bin_link.read_link().is_ok() {
                                    fs::remove_file(&bin_link).map_err(|e| {
                                        HorusError::Config(format!(
                                            "Failed to remove binary link: {}",
                                            e
                                        ))
                                    })?;
                                    println!(" Removed binary link for {}", package);
                                }
                            }
                        }

                        println!(" Removed system package reference for {}", package);

                        // Update horus.yaml if removing from local workspace
                        if let Some(ws_path) = workspace_path {
                            let horus_yaml_path = ws_path.join("horus.yaml");
                            if horus_yaml_path.exists() {
                                let mut content = fs::read_to_string(&horus_yaml_path)
                                    .map_err(|e| HorusError::Config(e.to_string()))?;

                                // Remove package from dependencies list
                                let lines: Vec<&str> = content.lines().collect();
                                let mut new_lines = Vec::new();
                                let mut in_deps = false;

                                for line in lines {
                                    if line.trim() == "dependencies:" {
                                        in_deps = true;
                                        new_lines.push(line);
                                    } else if in_deps && line.starts_with("  -") {
                                        let dep = line.trim_start_matches("  -").trim();
                                        if dep != package
                                            && !dep.starts_with(&format!("{}@", package))
                                        {
                                            new_lines.push(line);
                                        }
                                    } else {
                                        if in_deps && !line.starts_with("  ") {
                                            in_deps = false;
                                        }
                                        new_lines.push(line);
                                    }
                                }

                                content = new_lines.join("\n") + "\n";
                                fs::write(&horus_yaml_path, content)
                                    .map_err(|e| HorusError::Config(e.to_string()))?;
                            }
                        }

                        return Ok(());
                    }

                    if !remove_dir.exists() {
                        println!(" Package {} is not installed", package);
                        return Ok(());
                    }

                    // Remove package directory
                    std::fs::remove_dir_all(&remove_dir).map_err(|e| {
                        HorusError::Config(format!("Failed to remove package: {}", e))
                    })?;

                    println!(" Removed {} from {}", package, remove_dir.display());

                    // Update horus.yaml if removing from local workspace
                    if let Some(ws_path) = workspace_path {
                        let horus_yaml_path = ws_path.join("horus.yaml");
                        if horus_yaml_path.exists() {
                            if let Err(e) =
                                horus_manager::yaml_utils::remove_dependency_from_horus_yaml(
                                    &horus_yaml_path,
                                    &package,
                                )
                            {
                                println!("  {} Failed to update horus.yaml: {}", "".yellow(), e);
                            }
                        }
                    }

                    Ok(())
                }

                PkgCommands::List { query, global, all } => {
                    let client = registry::RegistryClient::new();

                    if let Some(q) = query {
                        // Search registry marketplace
                        println!(
                            "{} Searching registry marketplace for '{}'...",
                            "".cyan(),
                            q
                        );
                        let results = client
                            .search(&q)
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        if results.is_empty() {
                            println!(" No packages found in marketplace matching '{}'", q);
                        } else {
                            println!(
                                "\n{} Found {} package(s) in marketplace:\n",
                                "".green(),
                                results.len()
                            );
                            for pkg in results {
                                println!(
                                    "  {} {} - {}",
                                    pkg.name.yellow().bold(),
                                    pkg.version.dimmed(),
                                    pkg.description.unwrap_or_default()
                                );
                            }
                        }
                    } else if all {
                        // List both local and global packages
                        let home = dirs::home_dir().ok_or_else(|| {
                            HorusError::Config("Could not find home directory".to_string())
                        })?;
                        let global_cache = home.join(".horus/cache");

                        // Show local packages
                        println!("{} Local packages:\n", "".cyan());
                        let packages_dir = if let Some(root) = workspace::find_workspace_root() {
                            root.join(".horus/packages")
                        } else {
                            PathBuf::from(".horus/packages")
                        };

                        if packages_dir.exists() {
                            let mut has_local = false;
                            for entry in fs::read_dir(&packages_dir)
                                .map_err(|e| HorusError::Config(e.to_string()))?
                            {
                                let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                                let entry_path = entry.path();

                                // Skip if it's a metadata file
                                if entry_path.extension().and_then(|s| s.to_str()) == Some("json") {
                                    continue;
                                }

                                if entry
                                    .file_type()
                                    .map_err(|e| HorusError::Config(e.to_string()))?
                                    .is_dir()
                                    || entry
                                        .file_type()
                                        .map_err(|e| HorusError::Config(e.to_string()))?
                                        .is_symlink()
                                {
                                    has_local = true;
                                    let name = entry.file_name().to_string_lossy().to_string();

                                    // Check for path dependency metadata
                                    let path_meta =
                                        packages_dir.join(format!("{}.path.json", name));
                                    if path_meta.exists() {
                                        if let Ok(content) = fs::read_to_string(&path_meta) {
                                            if let Ok(metadata) =
                                                serde_json::from_str::<serde_json::Value>(&content)
                                            {
                                                let version =
                                                    metadata["version"].as_str().unwrap_or("dev");
                                                let path = metadata["source_path"]
                                                    .as_str()
                                                    .unwrap_or("unknown");
                                                println!(
                                                    "   {} {} {} {}",
                                                    name.yellow(),
                                                    version.dimmed(),
                                                    "(path:".dimmed(),
                                                    format!("{})", path).dimmed()
                                                );
                                                continue;
                                            }
                                        }
                                    }

                                    // Check for system package metadata
                                    let system_meta =
                                        packages_dir.join(format!("{}.system.json", name));
                                    if system_meta.exists() {
                                        if let Ok(content) = fs::read_to_string(&system_meta) {
                                            if let Ok(metadata) =
                                                serde_json::from_str::<serde_json::Value>(&content)
                                            {
                                                let version = metadata["version"]
                                                    .as_str()
                                                    .unwrap_or("unknown");
                                                println!(
                                                    "   {} {} {}",
                                                    name.yellow(),
                                                    version.dimmed(),
                                                    "(system)".dimmed()
                                                );
                                                continue;
                                            }
                                        }
                                    }

                                    // Check for regular metadata.json
                                    let metadata_path = entry_path.join("metadata.json");
                                    if metadata_path.exists() {
                                        if let Ok(content) = fs::read_to_string(&metadata_path) {
                                            if let Ok(metadata) =
                                                serde_json::from_str::<serde_json::Value>(&content)
                                            {
                                                let version = metadata["version"]
                                                    .as_str()
                                                    .unwrap_or("unknown");
                                                println!(
                                                    "   {} {} {}",
                                                    name.yellow(),
                                                    version.dimmed(),
                                                    "(registry)".dimmed()
                                                );
                                                continue;
                                            }
                                        }
                                    }

                                    // Fallback: just show name
                                    println!("   {}", name.yellow());
                                }
                            }
                            if !has_local {
                                println!("  No local packages");
                            }
                        } else {
                            println!("  No local packages");
                        }

                        // Show global packages
                        println!("\n{} Global cache packages:\n", "".cyan());
                        if global_cache.exists() {
                            let mut has_global = false;
                            for entry in fs::read_dir(&global_cache)
                                .map_err(|e| HorusError::Config(e.to_string()))?
                            {
                                let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                                if entry
                                    .file_type()
                                    .map_err(|e| HorusError::Config(e.to_string()))?
                                    .is_dir()
                                {
                                    has_global = true;
                                    let name = entry.file_name().to_string_lossy().to_string();
                                    println!("   {}", name.yellow());
                                }
                            }
                            if !has_global {
                                println!("  No global packages");
                            }
                        } else {
                            println!("  No global packages");
                        }
                    } else if global {
                        // List global cache packages
                        println!("{} Global cache packages:\n", "".cyan());
                        let home = dirs::home_dir().ok_or_else(|| {
                            HorusError::Config("Could not find home directory".to_string())
                        })?;
                        let global_cache = home.join(".horus/cache");

                        if !global_cache.exists() {
                            println!("  No global packages yet");
                            return Ok(());
                        }

                        for entry in fs::read_dir(&global_cache)
                            .map_err(|e| HorusError::Config(e.to_string()))?
                        {
                            let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                            if entry
                                .file_type()
                                .map_err(|e| HorusError::Config(e.to_string()))?
                                .is_dir()
                            {
                                let name = entry.file_name().to_string_lossy().to_string();
                                println!("   {}", name.yellow());
                            }
                        }
                    } else {
                        // List local workspace packages (default)
                        let packages_dir = if let Some(root) = workspace::find_workspace_root() {
                            root.join(".horus/packages")
                        } else {
                            PathBuf::from(".horus/packages")
                        };

                        println!("{} Local packages:\n", "".cyan());

                        if !packages_dir.exists() {
                            println!("  No packages installed yet");
                            return Ok(());
                        }

                        for entry in fs::read_dir(&packages_dir)
                            .map_err(|e| HorusError::Config(e.to_string()))?
                        {
                            let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                            let entry_path = entry.path();

                            // Skip if it's a metadata file
                            if entry_path.extension().and_then(|s| s.to_str()) == Some("json") {
                                continue;
                            }

                            if entry
                                .file_type()
                                .map_err(|e| HorusError::Config(e.to_string()))?
                                .is_dir()
                                || entry
                                    .file_type()
                                    .map_err(|e| HorusError::Config(e.to_string()))?
                                    .is_symlink()
                            {
                                let name = entry.file_name().to_string_lossy().to_string();

                                // Check for path dependency metadata
                                let path_meta = packages_dir.join(format!("{}.path.json", name));
                                if path_meta.exists() {
                                    if let Ok(content) = fs::read_to_string(&path_meta) {
                                        if let Ok(metadata) =
                                            serde_json::from_str::<serde_json::Value>(&content)
                                        {
                                            let version =
                                                metadata["version"].as_str().unwrap_or("dev");
                                            let path = metadata["source_path"]
                                                .as_str()
                                                .unwrap_or("unknown");
                                            println!(
                                                "  {} {} {} {}",
                                                name.yellow(),
                                                version.dimmed(),
                                                "(path:".dimmed(),
                                                format!("{})", path).dimmed()
                                            );
                                            continue;
                                        }
                                    }
                                }

                                // Check for system package metadata
                                let system_meta =
                                    packages_dir.join(format!("{}.system.json", name));
                                if system_meta.exists() {
                                    if let Ok(content) = fs::read_to_string(&system_meta) {
                                        if let Ok(metadata) =
                                            serde_json::from_str::<serde_json::Value>(&content)
                                        {
                                            let version =
                                                metadata["version"].as_str().unwrap_or("unknown");
                                            println!(
                                                "  {} {} {}",
                                                name.yellow(),
                                                version.dimmed(),
                                                "(system)".dimmed()
                                            );
                                            continue;
                                        }
                                    }
                                }

                                // Try to read metadata.json (registry packages)
                                let metadata_path = entry_path.join("metadata.json");
                                if metadata_path.exists() {
                                    if let Ok(content) = fs::read_to_string(&metadata_path) {
                                        if let Ok(metadata) =
                                            serde_json::from_str::<serde_json::Value>(&content)
                                        {
                                            let version =
                                                metadata["version"].as_str().unwrap_or("unknown");
                                            println!(
                                                "  {} {} {}",
                                                name.yellow(),
                                                version.dimmed(),
                                                "(registry)".dimmed()
                                            );
                                            continue;
                                        }
                                    }
                                }

                                // Fallback: just show name
                                println!("  {}", name.yellow());
                            }
                        }
                    }

                    Ok(())
                }

                PkgCommands::Publish { freeze, dry_run } => {
                    let client = registry::RegistryClient::new();
                    client
                        .publish(None, dry_run, None)
                        .map_err(|e| HorusError::Config(e.to_string()))?;

                    // If --freeze flag is set, also generate freeze file
                    if freeze {
                        println!("\n{} Generating freeze file...", "".cyan());
                        let manifest = client
                            .freeze()
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        let freeze_file = "horus-freeze.yaml";
                        let yaml = serde_yaml::to_string(&manifest)
                            .map_err(|e| HorusError::Config(e.to_string()))?;
                        std::fs::write(freeze_file, yaml)
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        println!(" Environment also frozen to {}", freeze_file);
                    }

                    Ok(())
                }

                PkgCommands::Update {
                    package,
                    global,
                    dry_run,
                } => {
                    let client = registry::RegistryClient::new();
                    client
                        .update_packages(package.as_deref(), global, dry_run)
                        .map_err(|e| HorusError::Config(e.to_string()))?;
                    Ok(())
                }

                PkgCommands::KeyGen => {
                    registry::generate_signing_keypair()
                        .map_err(|e| HorusError::Config(e.to_string()))?;
                    Ok(())
                }

                PkgCommands::Unpublish {
                    package,
                    version,
                    yes,
                } => {
                    use std::io::{self, Write};

                    println!(
                        "{} Unpublishing {} v{}...",
                        "".cyan(),
                        package.yellow(),
                        version.yellow()
                    );

                    // Confirmation prompt (unless --yes flag is set)
                    if !yes {
                        println!(
                            "\n{} This action is {} and will:",
                            "Warning:".yellow().bold(),
                            "IRREVERSIBLE".red().bold()
                        );
                        println!("  • Delete {} v{} from the registry", package, version);
                        println!("  • Make this version unavailable for download");
                        println!("  • Cannot be undone");
                        println!(
                            "\n{} Consider using 'yank' instead for temporary removal",
                            "Tip:".dimmed()
                        );

                        print!("\nType the package name '{}' to confirm: ", package);
                        io::stdout().flush().unwrap();

                        let mut confirmation = String::new();
                        io::stdin().read_line(&mut confirmation).map_err(|e| {
                            HorusError::Config(format!("Failed to read input: {}", e))
                        })?;

                        if confirmation.trim() != package {
                            println!(" Package name mismatch. Unpublish cancelled.");
                            return Ok(());
                        }
                    }

                    // Call unpublish API
                    let client = registry::RegistryClient::new();
                    client
                        .unpublish(&package, &version)
                        .map_err(|e| HorusError::Config(e.to_string()))?;

                    println!(
                        "\n Successfully unpublished {} v{}",
                        package.green(),
                        version.green()
                    );
                    println!("   The package is no longer available on the registry");

                    Ok(())
                }
            }
        }

        Commands::Env { command } => {
            match command {
                EnvCommands::Freeze { output, publish } => {
                    println!("{} Freezing current environment...", "".cyan());

                    let client = registry::RegistryClient::new();
                    let manifest = client
                        .freeze()
                        .map_err(|e| HorusError::Config(e.to_string()))?;

                    // Save to local file
                    let freeze_file = output.unwrap_or_else(|| PathBuf::from("horus-freeze.yaml"));
                    let yaml = serde_yaml::to_string(&manifest)
                        .map_err(|e| HorusError::Config(e.to_string()))?;
                    std::fs::write(&freeze_file, yaml)
                        .map_err(|e| HorusError::Config(e.to_string()))?;

                    println!(" Environment frozen to {}", freeze_file.display());
                    println!("   ID: {}", manifest.horus_id);
                    println!("   Packages: {}", manifest.packages.len());

                    // Publish to registry if requested
                    if publish {
                        // Validate: check for path dependencies before publishing
                        let has_path_deps = manifest
                            .packages
                            .iter()
                            .any(|pkg| matches!(pkg.source, registry::PackageSource::Path { .. }));

                        if has_path_deps {
                            println!(
                                "\n{} Cannot publish environment with path dependencies!",
                                "Error:".red()
                            );
                            println!("\nPath dependencies found:");
                            for pkg in &manifest.packages {
                                if let registry::PackageSource::Path { ref path } = pkg.source {
                                    println!("  • {} -> {}", pkg.name, path);
                                }
                            }
                            println!("\n{}", "Path dependencies are not portable and cannot be published to the registry.".yellow());
                            println!(
                                "{}",
                                "You can still save locally with: horus env freeze".yellow()
                            );
                            return Err(HorusError::Config(
                                "Cannot publish environment with path dependencies".to_string(),
                            ));
                        }

                        println!();
                        client
                            .upload_environment(&manifest)
                            .map_err(|e| HorusError::Config(e.to_string()))?;
                    } else {
                        println!("\n{} To share this environment:", "Tip:".dimmed());
                        println!("   1. File: horus env restore {}", freeze_file.display());
                        println!("   2. Registry: horus env freeze --publish");
                    }

                    Ok(())
                }

                EnvCommands::Restore { source } => {
                    println!("{} Restoring environment from {}...", "".cyan(), source);

                    let client = registry::RegistryClient::new();

                    // Check if source is a file path or environment ID
                    if source.ends_with(".yaml")
                        || source.ends_with(".yml")
                        || PathBuf::from(&source).exists()
                    {
                        // It's a file path
                        let content = fs::read_to_string(&source).map_err(|e| {
                            HorusError::Config(format!("Failed to read freeze file: {}", e))
                        })?;

                        let manifest: registry::EnvironmentManifest =
                            serde_yaml::from_str(&content).map_err(|e| {
                                HorusError::Config(format!("Failed to parse freeze file: {}", e))
                            })?;

                        println!(" Found {} packages to restore", manifest.packages.len());

                        // Get workspace path for horus.yaml updates
                        let workspace_path = workspace::find_workspace_root();

                        // Install each package from the manifest
                        for pkg in &manifest.packages {
                            // Handle different package sources
                            match &pkg.source {
                                registry::PackageSource::System => {
                                    // Check if system package actually exists
                                    let exists = check_system_package_exists(&pkg.name);

                                    if exists {
                                        println!(
                                            "  {} {} v{} (system package - verified)",
                                            "".green(),
                                            pkg.name,
                                            pkg.version
                                        );
                                        continue;
                                    } else {
                                        println!(
                                            "\n  {} {} v{} (system package NOT found)",
                                            "[WARNING]".yellow(),
                                            pkg.name,
                                            pkg.version
                                        );

                                        // Prompt user for what to do
                                        match prompt_missing_system_package(&pkg.name)? {
                                            MissingSystemChoice::InstallGlobal => {
                                                println!(
                                                    "  {} Installing to HORUS global cache...",
                                                    "".cyan()
                                                );
                                                client
                                                    .install_to_target(
                                                        &pkg.name,
                                                        Some(&pkg.version),
                                                        workspace::InstallTarget::Global,
                                                    )
                                                    .map_err(|e| {
                                                        HorusError::Config(e.to_string())
                                                    })?;
                                            }
                                            MissingSystemChoice::InstallLocal => {
                                                println!(
                                                    "  {} Installing to HORUS local...",
                                                    "".cyan()
                                                );
                                                client
                                                    .install(&pkg.name, Some(&pkg.version))
                                                    .map_err(|e| {
                                                        HorusError::Config(e.to_string())
                                                    })?;
                                            }
                                            MissingSystemChoice::Skip => {
                                                println!("  {} Skipped {}", "⊘".yellow(), pkg.name);
                                                continue;
                                            }
                                        }
                                    }
                                }
                                registry::PackageSource::Path { path } => {
                                    println!(
                                        "  {} {} (path dependency)",
                                        "[WARNING]".yellow(),
                                        pkg.name
                                    );
                                    println!("    Path: {}", path);
                                    println!("    {} Path dependencies are not portable across machines.", "Note:".dimmed());
                                    println!("    {} Please update horus.yaml with the correct path if needed.", "Tip:".dimmed());
                                    // Don't try to install - user must fix path manually
                                    continue;
                                }
                                _ => {
                                    // Registry, PyPI, CratesIO - use standard install
                                    println!("  Installing {} v{}...", pkg.name, pkg.version);
                                    client
                                        .install(&pkg.name, Some(&pkg.version))
                                        .map_err(|e| HorusError::Config(e.to_string()))?;
                                }
                            }

                            // Update horus.yaml if in a workspace
                            if let Some(ref ws_path) = workspace_path {
                                let yaml_path = ws_path.join("horus.yaml");
                                if yaml_path.exists() {
                                    if let Err(e) =
                                        horus_manager::yaml_utils::add_dependency_to_horus_yaml(
                                            &yaml_path,
                                            &pkg.name,
                                            &pkg.version,
                                        )
                                    {
                                        eprintln!(
                                            "  {} Failed to update horus.yaml: {}",
                                            "".yellow(),
                                            e
                                        );
                                    }
                                }
                            }
                        }

                        println!(" Environment restored from {}", source);
                        println!("   ID: {}", manifest.horus_id);
                        println!("   Packages: {}", manifest.packages.len());
                    } else {
                        // It's an environment ID from registry
                        // Fetch manifest and install manually to update horus.yaml
                        println!(" Fetching environment {}...", source);

                        let url = format!("{}/api/environments/{}", client.base_url(), source);

                        let response = client
                            .http_client()
                            .get(&url)
                            .send()
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        if !response.status().is_success() {
                            return Err(HorusError::Config(format!(
                                "Environment not found: {}",
                                source
                            )));
                        }

                        let manifest: registry::EnvironmentManifest = response
                            .json()
                            .map_err(|e| HorusError::Config(e.to_string()))?;

                        println!(" Found {} packages to restore", manifest.packages.len());

                        // Get workspace path for horus.yaml updates
                        let workspace_path = workspace::find_workspace_root();

                        // Install each package
                        for pkg in &manifest.packages {
                            // Handle different package sources
                            match &pkg.source {
                                registry::PackageSource::System => {
                                    // Check if system package actually exists
                                    let exists = check_system_package_exists(&pkg.name);

                                    if exists {
                                        println!(
                                            "  {} {} v{} (system package - verified)",
                                            "".green(),
                                            pkg.name,
                                            pkg.version
                                        );
                                        continue;
                                    } else {
                                        println!(
                                            "\n  {} {} v{} (system package NOT found)",
                                            "[WARNING]".yellow(),
                                            pkg.name,
                                            pkg.version
                                        );

                                        // Prompt user for what to do
                                        match prompt_missing_system_package(&pkg.name)? {
                                            MissingSystemChoice::InstallGlobal => {
                                                println!(
                                                    "  {} Installing to HORUS global cache...",
                                                    "".cyan()
                                                );
                                                client
                                                    .install_to_target(
                                                        &pkg.name,
                                                        Some(&pkg.version),
                                                        workspace::InstallTarget::Global,
                                                    )
                                                    .map_err(|e| {
                                                        HorusError::Config(e.to_string())
                                                    })?;
                                            }
                                            MissingSystemChoice::InstallLocal => {
                                                println!(
                                                    "  {} Installing to HORUS local...",
                                                    "".cyan()
                                                );
                                                client
                                                    .install(&pkg.name, Some(&pkg.version))
                                                    .map_err(|e| {
                                                        HorusError::Config(e.to_string())
                                                    })?;
                                            }
                                            MissingSystemChoice::Skip => {
                                                println!("  {} Skipped {}", "⊘".yellow(), pkg.name);
                                                continue;
                                            }
                                        }
                                    }
                                }
                                registry::PackageSource::Path { path } => {
                                    println!(
                                        "  {} {} (path dependency)",
                                        "[WARNING]".yellow(),
                                        pkg.name
                                    );
                                    println!("    Path: {}", path);
                                    println!("    {} Path dependencies are not portable across machines.", "Note:".dimmed());
                                    println!("    {} Please update horus.yaml with the correct path if needed.", "Tip:".dimmed());
                                    // Don't try to install - user must fix path manually
                                    continue;
                                }
                                _ => {
                                    // Registry, PyPI, CratesIO - use standard install
                                    println!("  Installing {} v{}...", pkg.name, pkg.version);
                                    client
                                        .install(&pkg.name, Some(&pkg.version))
                                        .map_err(|e| HorusError::Config(e.to_string()))?;
                                }
                            }

                            // Update horus.yaml if in a workspace
                            if let Some(ref ws_path) = workspace_path {
                                let yaml_path = ws_path.join("horus.yaml");
                                if yaml_path.exists() {
                                    if let Err(e) =
                                        horus_manager::yaml_utils::add_dependency_to_horus_yaml(
                                            &yaml_path,
                                            &pkg.name,
                                            &pkg.version,
                                        )
                                    {
                                        eprintln!(
                                            "  {} Failed to update horus.yaml: {}",
                                            "".yellow(),
                                            e
                                        );
                                    }
                                }
                            }
                        }

                        println!(" Environment {} restored successfully!", source);
                    }

                    Ok(())
                }

                EnvCommands::List => {
                    println!("{} Fetching published environments...", "".cyan());

                    let client = registry::RegistryClient::new();
                    let environments = client
                        .list_environments()
                        .map_err(|e| HorusError::Config(e.to_string()))?;

                    if environments.is_empty() {
                        println!("\n{}", "No published environments found.".dimmed());
                        println!(
                            "{}",
                            "Publish one with: horus env freeze --publish".dimmed()
                        );
                    } else {
                        println!("\n{} Published Environments:\n", "".green());
                        for env in &environments {
                            let name = env.name.as_deref().unwrap_or("(unnamed)");
                            println!("  {} {}", "•".cyan(), env.horus_id.bold());
                            println!("    Name: {}", name);
                            if let Some(desc) = &env.description {
                                println!("    Description: {}", desc.dimmed());
                            }
                            println!(
                                "    Packages: {} | Created: {}",
                                env.manifest.packages.len(),
                                env.created_at
                            );
                            println!(
                                "    System: {} ({})",
                                env.manifest.system.os, env.manifest.system.arch
                            );
                            println!("    HORUS: v{}", env.manifest.horus_version);
                            println!();
                        }
                        println!("{} To restore: horus env restore <ID>", "Tip:".dimmed());
                        println!("{} For details: horus env show <ID>", "    ".dimmed());
                    }

                    Ok(())
                }

                EnvCommands::Show { id } => {
                    println!("{} Fetching environment {}...", "".cyan(), id);

                    let client = registry::RegistryClient::new();
                    let manifest = client
                        .get_environment(&id)
                        .map_err(|e| HorusError::Config(e.to_string()))?;

                    // Display environment details
                    println!("\n{} Environment Details\n", "".green());
                    println!("  ID:          {}", manifest.horus_id.bold());
                    println!(
                        "  Name:        {}",
                        manifest.name.as_deref().unwrap_or("(unnamed)")
                    );
                    if let Some(desc) = &manifest.description {
                        println!("  Description: {}", desc);
                    }
                    println!(
                        "  Created:     {}",
                        manifest.created_at.format("%Y-%m-%d %H:%M:%S UTC")
                    );
                    println!("  HORUS:       v{}", manifest.horus_version);

                    // System info
                    println!("\n{} System\n", "".cyan());
                    println!("  OS:          {}", manifest.system.os);
                    println!("  Arch:        {}", manifest.system.arch);
                    if let Some(py) = &manifest.system.python_version {
                        println!("  Python:      {}", py);
                    }
                    if let Some(rs) = &manifest.system.rust_version {
                        println!("  Rust:        {}", rs);
                    }
                    if let Some(gcc) = &manifest.system.gcc_version {
                        println!("  GCC:         {}", gcc);
                    }
                    if let Some(cuda) = &manifest.system.cuda_version {
                        println!("  CUDA:        {}", cuda);
                    }

                    // Packages
                    println!("\n{} Packages ({})\n", "".cyan(), manifest.packages.len());
                    for pkg in &manifest.packages {
                        let source_str = match &pkg.source {
                            registry::PackageSource::Registry => "registry".to_string(),
                            registry::PackageSource::PyPI => "pypi".to_string(),
                            registry::PackageSource::CratesIO => "crates.io".to_string(),
                            registry::PackageSource::System => "system".to_string(),
                            registry::PackageSource::Path { path } => format!("path:{}", path),
                        };
                        println!(
                            "  {} {} v{} ({})",
                            "•".cyan(),
                            pkg.name.bold(),
                            pkg.version,
                            source_str.dimmed()
                        );
                    }

                    println!(
                        "\n{} To restore: horus env restore {}",
                        "Tip:".dimmed(),
                        manifest.horus_id
                    );

                    Ok(())
                }
            }
        }

        Commands::Auth { command } => match command {
            AuthCommands::Login => commands::github_auth::login(),
            AuthCommands::GenerateKey { name, environment } => {
                commands::github_auth::generate_key(name, environment)
            }
            AuthCommands::Logout => commands::github_auth::logout(),
            AuthCommands::Whoami => commands::github_auth::whoami(),
            AuthCommands::Org => commands::github_auth::org(),
            AuthCommands::Usage => commands::github_auth::usage(),
            AuthCommands::Plan => commands::github_auth::plan(),
            AuthCommands::Keys { command: keys_cmd } => match keys_cmd {
                AuthKeysCommands::List => commands::github_auth::keys_list(),
                AuthKeysCommands::Revoke { key_id } => commands::github_auth::keys_revoke(&key_id),
            },
        },

        Commands::Deploy {
            target,
            remote_dir,
            arch,
            run_after,
            debug,
            port,
            identity,
            dry_run,
            list,
        } => {
            if list {
                commands::deploy::list_targets()
            } else if let Some(target) = target {
                commands::deploy::run_deploy(
                    &target, remote_dir, arch, run_after, !debug, // release = !debug
                    port, identity, dry_run,
                )
            } else {
                Err(HorusError::Config(
                    "Target is required for deploy".to_string(),
                ))
            }
        }

        Commands::Driver { command } => {
            // Driver alias resolver (local implementation)
            fn resolve_driver_alias_local(alias: &str) -> Option<Vec<&'static str>> {
                match alias {
                    "vision" => Some(vec!["camera", "depth-camera"]),
                    "navigation" => Some(vec!["lidar", "gps", "imu"]),
                    "manipulation" => Some(vec!["servo", "motor", "force-torque"]),
                    "locomotion" => Some(vec!["motor", "encoder", "imu"]),
                    "sensing" => Some(vec!["camera", "lidar", "ultrasonic", "imu"]),
                    _ => None,
                }
            }

            // Built-in driver definitions
            struct DriverInfo {
                id: &'static str,
                name: &'static str,
                category: &'static str,
                description: &'static str,
            }

            let built_in_drivers = vec![
                DriverInfo {
                    id: "camera",
                    name: "Camera Driver",
                    category: "Sensor",
                    description: "RGB camera support (OpenCV, V4L2)",
                },
                DriverInfo {
                    id: "depth-camera",
                    name: "Depth Camera Driver",
                    category: "Sensor",
                    description: "Depth sensing (RealSense, Kinect)",
                },
                DriverInfo {
                    id: "lidar",
                    name: "LiDAR Driver",
                    category: "Sensor",
                    description: "2D/3D LiDAR (RPLidar, Velodyne)",
                },
                DriverInfo {
                    id: "imu",
                    name: "IMU Driver",
                    category: "Sensor",
                    description: "Inertial measurement (MPU6050, BNO055)",
                },
                DriverInfo {
                    id: "gps",
                    name: "GPS Driver",
                    category: "Sensor",
                    description: "GPS/GNSS receivers",
                },
                DriverInfo {
                    id: "ultrasonic",
                    name: "Ultrasonic Driver",
                    category: "Sensor",
                    description: "Ultrasonic rangefinders",
                },
                DriverInfo {
                    id: "motor",
                    name: "Motor Driver",
                    category: "Actuator",
                    description: "DC/BLDC motors",
                },
                DriverInfo {
                    id: "servo",
                    name: "Servo Driver",
                    category: "Actuator",
                    description: "Servo motors (PWM, Dynamixel)",
                },
                DriverInfo {
                    id: "stepper",
                    name: "Stepper Driver",
                    category: "Actuator",
                    description: "Stepper motors",
                },
                DriverInfo {
                    id: "encoder",
                    name: "Encoder Driver",
                    category: "Sensor",
                    description: "Rotary/linear encoders",
                },
                DriverInfo {
                    id: "force-torque",
                    name: "Force/Torque Driver",
                    category: "Sensor",
                    description: "Force/torque sensors",
                },
                DriverInfo {
                    id: "serial",
                    name: "Serial Driver",
                    category: "Bus",
                    description: "UART/RS232/RS485",
                },
                DriverInfo {
                    id: "i2c",
                    name: "I2C Driver",
                    category: "Bus",
                    description: "I2C bus communication",
                },
                DriverInfo {
                    id: "spi",
                    name: "SPI Driver",
                    category: "Bus",
                    description: "SPI bus communication",
                },
                DriverInfo {
                    id: "can",
                    name: "CAN Driver",
                    category: "Bus",
                    description: "CAN bus communication",
                },
                DriverInfo {
                    id: "modbus",
                    name: "Modbus Driver",
                    category: "Bus",
                    description: "Modbus RTU/TCP",
                },
                DriverInfo {
                    id: "joystick",
                    name: "Joystick Driver",
                    category: "Input",
                    description: "Gamepad/joystick input",
                },
                DriverInfo {
                    id: "keyboard",
                    name: "Keyboard Driver",
                    category: "Input",
                    description: "Keyboard input",
                },
            ];

            match command {
                DriverCommands::List {
                    category,
                    registry_only,
                    plugins_only,
                    mode,
                } => {
                    // Show local built-in drivers unless --registry or --plugins flag
                    if !registry_only && !plugins_only {
                        let drivers: Vec<_> = if let Some(cat_str) = &category {
                            let cat_lower = cat_str.to_lowercase();
                            let cat_match = match cat_lower.as_str() {
                                "sensor" | "sensors" => "Sensor",
                                "actuator" | "actuators" => "Actuator",
                                "bus" | "buses" => "Bus",
                                "input" | "inputs" => "Input",
                                "simulation" | "sim" => "Simulation",
                                _ => {
                                    println!("{} Unknown category: {}", "[WARN]".yellow(), cat_str);
                                    println!(
                                        "       Valid categories: sensor, actuator, bus, input, simulation"
                                    );
                                    return Ok(());
                                }
                            };
                            built_in_drivers
                                .iter()
                                .filter(|d| d.category == cat_match)
                                .collect()
                        } else {
                            built_in_drivers.iter().collect()
                        };

                        if !drivers.is_empty() {
                            println!("{} Built-in Drivers:\n", "[BUILD]".cyan().bold());
                            for driver in &drivers {
                                println!(
                                    "  {} {} ({})",
                                    "-".green(),
                                    driver.id.yellow(),
                                    driver.category
                                );
                                if !driver.description.is_empty() {
                                    println!("     {}", driver.description.dimmed());
                                }
                            }
                            println!();
                        }
                    }

                    // Show loaded plugins
                    if !registry_only {
                        use horus_core::plugin::{DriverLoader, DriverLoaderConfig, DriverMode};

                        let driver_mode = match mode.to_lowercase().as_str() {
                            "static" => DriverMode::Static,
                            "dynamic" => DriverMode::Dynamic,
                            _ => DriverMode::Hybrid,
                        };

                        let loader_config = DriverLoaderConfig {
                            mode: driver_mode,
                            auto_discover: true,
                            ..Default::default()
                        };
                        let mut loader = DriverLoader::new(loader_config);

                        // Try to load plugins from standard locations
                        if let Err(e) = loader.initialize() {
                            if !plugins_only {
                                // Only warn if not plugins-only mode
                                println!("{} Could not load plugins: {}", "[WARN]".yellow(), e);
                            }
                        }

                        let loaded = loader.available_drivers();
                        if !loaded.is_empty() {
                            println!(
                                "{} Loaded Plugins ({}):\n",
                                "[PLUGIN]".magenta().bold(),
                                loaded.len()
                            );

                            for plugin_id in loaded {
                                if let Some(manifest) =
                                    loader.get_plugin(&plugin_id).ok().map(|p| p.manifest())
                                {
                                    // Filter by category if specified
                                    let category_match = if let Some(cat_str) = &category {
                                        let cat_lower = cat_str.to_lowercase();
                                        let manifest_cat =
                                            format!("{:?}", manifest.category).to_lowercase();
                                        manifest_cat.contains(&cat_lower)
                                    } else {
                                        true
                                    };

                                    if category_match {
                                        println!(
                                            "  {} {} v{} ({:?})",
                                            "◆".magenta(),
                                            manifest.name.yellow(),
                                            manifest.version,
                                            manifest.category
                                        );
                                        println!("     {}", manifest.description.dimmed());

                                        // Show backends provided by this plugin
                                        if !manifest.backends.is_empty() {
                                            let backend_ids: Vec<_> = manifest
                                                .backends
                                                .iter()
                                                .map(|b| b.id.as_str())
                                                .collect();
                                            println!(
                                                "     Backends: {}",
                                                backend_ids.join(", ").cyan()
                                            );
                                        }
                                    }
                                }
                            }
                            println!();
                        } else if plugins_only {
                            println!("{} No plugins loaded.", "[INFO]".cyan());
                            println!("  Place plugins in ~/.horus/drivers/ or use static linking.");
                        }
                    }

                    // Also fetch from registry (unless plugins_only)
                    if !plugins_only {
                        println!("{} Fetching drivers from registry...", "[NET]".cyan());
                        let client = registry::RegistryClient::new();
                        match client.list_drivers(category.as_deref()) {
                            Ok(registry_drivers) => {
                                if registry_drivers.is_empty() {
                                    println!("  No additional drivers in registry.");
                                } else {
                                    println!(
                                        "\n{} Registry Drivers ({} available):\n",
                                        "[PKG]".cyan().bold(),
                                        registry_drivers.len()
                                    );
                                    for d in registry_drivers {
                                        let bus = d.bus_type.as_deref().unwrap_or("unknown");
                                        let cat = d.category.as_deref().unwrap_or("driver");
                                        println!(
                                            "  {} {} ({}, {})",
                                            "-".green(),
                                            d.name.yellow(),
                                            cat,
                                            bus.dimmed()
                                        );
                                        if let Some(desc) = &d.description {
                                            println!("     {}", desc.dimmed());
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                println!("  {} Could not fetch registry: {}", "[WARN]".yellow(), e);
                            }
                        }
                    }
                    Ok(())
                }

                DriverCommands::Info { driver } => {
                    // Check if it's an alias first
                    if let Some(expanded) = resolve_driver_alias_local(&driver) {
                        println!(
                            "{} Driver Alias: {}\n",
                            "[LINK]".cyan().bold(),
                            driver.yellow()
                        );
                        println!("  Expands to: {}", expanded.join(", ").green());
                        return Ok(());
                    }

                    // Look up in local built-ins first
                    if let Some(info) = built_in_drivers.iter().find(|d| d.id == driver) {
                        println!(
                            "{} Driver: {} (built-in)\n",
                            "[BUILD]".cyan().bold(),
                            info.name.yellow()
                        );
                        println!("  ID:       {}", info.id);
                        println!("  Category: {}", info.category);
                        if !info.description.is_empty() {
                            println!("  Desc:     {}", info.description);
                        }
                        return Ok(());
                    }

                    // Try registry
                    println!("{} Looking up '{}' in registry...", "[NET]".cyan(), driver);
                    let client = registry::RegistryClient::new();
                    match client.fetch_driver_metadata(&driver) {
                        Ok(meta) => {
                            println!("\n{} Driver: {}\n", "[PKG]".cyan().bold(), driver.yellow());
                            if let Some(bus) = &meta.bus_type {
                                println!("  Bus Type:  {}", bus);
                            }
                            if let Some(cat) = &meta.driver_category {
                                println!("  Category:  {}", cat);
                            }

                            // Show dependencies
                            if let Some(features) = &meta.required_features {
                                if !features.is_empty() {
                                    println!("\n  {} Cargo Features:", "[BUILD]".cyan());
                                    for f in features {
                                        println!("    • {}", f.yellow());
                                    }
                                }
                            }
                            if let Some(cargo_deps) = &meta.cargo_dependencies {
                                if !cargo_deps.is_empty() {
                                    println!("\n  {} Cargo Dependencies:", "[PKG]".cyan());
                                    for d in cargo_deps {
                                        println!("    • {}", d);
                                    }
                                }
                            }
                            if let Some(py_deps) = &meta.python_dependencies {
                                if !py_deps.is_empty() {
                                    println!("\n  {} Python Dependencies:", "[PY]".cyan());
                                    for d in py_deps {
                                        println!("    • {}", d);
                                    }
                                }
                            }
                            if let Some(sys_deps) = &meta.system_dependencies {
                                if !sys_deps.is_empty() {
                                    println!("\n  {} System Packages:", "[SYS]".cyan());
                                    for d in sys_deps {
                                        println!("    • {}", d);
                                    }
                                }
                            }

                            println!(
                                "\n  {} To add: horus driver add {}",
                                "[TIP]".green(),
                                driver
                            );
                        }
                        Err(_) => {
                            println!("{} Driver '{}' not found.", "[WARN]".yellow(), driver);
                            println!("\nUse 'horus driver list' to see available drivers.");
                            println!("Use 'horus driver search <query>' to search.");
                        }
                    }
                    Ok(())
                }

                DriverCommands::Search { query, bus_type } => {
                    // Search local first
                    let query_lower = query.to_lowercase();
                    let local_matches: Vec<_> = built_in_drivers
                        .iter()
                        .filter(|d| {
                            d.id.to_lowercase().contains(&query_lower)
                                || d.name.to_lowercase().contains(&query_lower)
                                || d.description.to_lowercase().contains(&query_lower)
                        })
                        .collect();

                    if !local_matches.is_empty() {
                        println!("{} Built-in matches:\n", "[BUILD]".cyan().bold());
                        for driver in &local_matches {
                            println!("  {} {} - {}", "-".green(), driver.id.yellow(), driver.name);
                        }
                        println!();
                    }

                    // Search registry
                    println!("{} Searching registry for '{}'...", "[NET]".cyan(), query);
                    let client = registry::RegistryClient::new();
                    match client.search_drivers(&query, bus_type.as_deref()) {
                        Ok(results) => {
                            if results.is_empty() {
                                println!("  No matches in registry.");
                            } else {
                                println!(
                                    "\n{} Registry matches ({}):\n",
                                    "[PKG]".cyan().bold(),
                                    results.len()
                                );
                                for d in results {
                                    let bus = d.bus_type.as_deref().unwrap_or("?");
                                    println!(
                                        "  {} {} [{}]",
                                        "-".green(),
                                        d.name.yellow(),
                                        bus.dimmed()
                                    );
                                    if let Some(desc) = &d.description {
                                        println!("     {}", desc.dimmed());
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            println!("  {} Could not search registry: {}", "[WARN]".yellow(), e);
                        }
                    }
                    Ok(())
                }

                DriverCommands::Probe {
                    plugin,
                    backend,
                    mode,
                } => {
                    use horus_core::plugin::{DriverLoader, DriverLoaderConfig, DriverMode};

                    let driver_mode = match mode.to_lowercase().as_str() {
                        "static" => DriverMode::Static,
                        "dynamic" => DriverMode::Dynamic,
                        _ => DriverMode::Hybrid,
                    };

                    println!(
                        "{} Probing for hardware (mode: {})...\n",
                        "[PROBE]".cyan().bold(),
                        mode
                    );

                    let loader_config = DriverLoaderConfig {
                        mode: driver_mode,
                        auto_discover: true,
                        ..Default::default()
                    };
                    let mut loader = DriverLoader::new(loader_config);

                    if let Err(e) = loader.initialize() {
                        return Err(HorusError::Driver(format!(
                            "Failed to initialize plugin loader: {}",
                            e
                        )));
                    }

                    let plugins_to_probe: Vec<String> = if let Some(p) = plugin {
                        vec![p]
                    } else {
                        loader.available_drivers()
                    };

                    if plugins_to_probe.is_empty() {
                        println!("{} No plugins loaded to probe.", "[WARN]".yellow());
                        println!("  Place plugins in ~/.horus/drivers/ or use static linking.");
                        return Ok(());
                    }

                    let backend_filter = backend.as_deref();

                    for plugin_id in plugins_to_probe {
                        let results = match loader.probe(&plugin_id) {
                            Ok(r) => r,
                            Err(e) => {
                                println!(
                                    "  {} Plugin {}: {}",
                                    "[ERROR]".red(),
                                    plugin_id.yellow(),
                                    e
                                );
                                continue;
                            }
                        };

                        // Filter by backend if specified
                        let results: Vec<_> = if let Some(filter) = backend_filter {
                            results
                                .into_iter()
                                .filter(|r| r.backend_id.contains(filter))
                                .collect()
                        } else {
                            results
                        };

                        if results.is_empty() {
                            continue;
                        }

                        println!("  {} Plugin: {}", "◆".magenta(), plugin_id.yellow());

                        for result in results {
                            let status = if result.detected {
                                format!(
                                    "✓ Detected (confidence: {:.0}%)",
                                    result.confidence * 100.0
                                )
                                .green()
                            } else {
                                "✗ Not detected".red().to_string().into()
                            };

                            println!(
                                "    {} {} - {}",
                                "-".cyan(),
                                result.backend_id.yellow(),
                                status
                            );

                            if let Some(device_path) = &result.device_path {
                                println!("      Device: {}", device_path.dimmed());
                            }

                            if let Some(msg) = &result.message {
                                println!("      Info: {}", msg.dimmed());
                            }
                        }
                        println!();
                    }

                    Ok(())
                }

                DriverCommands::Plugins { reload, mode } => {
                    use horus_core::plugin::{DriverLoader, DriverLoaderConfig, DriverMode};

                    let driver_mode = match mode.to_lowercase().as_str() {
                        "static" => DriverMode::Static,
                        "dynamic" => DriverMode::Dynamic,
                        _ => DriverMode::Hybrid,
                    };

                    if reload {
                        println!(
                            "{} Reloading plugins (mode: {})...",
                            "[PLUGIN]".magenta().bold(),
                            mode
                        );
                    } else {
                        println!(
                            "{} Loaded Plugins (mode: {}):\n",
                            "[PLUGIN]".magenta().bold(),
                            mode
                        );
                    }

                    let loader_config = DriverLoaderConfig {
                        mode: driver_mode,
                        auto_discover: true,
                        ..Default::default()
                    };
                    let mut loader = DriverLoader::new(loader_config);

                    if let Err(e) = loader.initialize() {
                        return Err(HorusError::Driver(format!(
                            "Failed to initialize plugin loader: {}",
                            e
                        )));
                    }

                    let loaded = loader.available_drivers();

                    if loaded.is_empty() {
                        println!("{} No plugins loaded.", "[INFO]".cyan());
                        println!("\n  Plugin search paths:");
                        println!("    - ~/.horus/drivers/");
                        println!("    - ./drivers/");
                        println!("\n  To create a plugin:");
                        println!("    1. Build plugin as cdylib: crate-type = [\"cdylib\"]");
                        println!("    2. Export entry point: horus_driver_entry()");
                        println!("    3. Place .so file in search path");
                        return Ok(());
                    }

                    // Get plugin health status once
                    let health_map = loader.health();

                    for plugin_id in loaded {
                        if let Some(manifest) =
                            loader.get_plugin(&plugin_id).ok().map(|p| p.manifest())
                        {
                            println!(
                                "  {} {} v{} ({:?})",
                                "◆".magenta(),
                                manifest.name.yellow(),
                                manifest.version,
                                manifest.category
                            );
                            println!("     ID:          {}", manifest.id);
                            println!("     Description: {}", manifest.description.dimmed());

                            if let Some(author) = &manifest.author {
                                println!("     Author:      {}", author);
                            }

                            if !manifest.backends.is_empty() {
                                println!("     Backends:");
                                for backend in &manifest.backends {
                                    println!(
                                        "       - {} ({})",
                                        backend.id.cyan(),
                                        backend.description.dimmed()
                                    );
                                }
                            }

                            // Check plugin health
                            if let Some(health) = health_map.get(&plugin_id) {
                                let status = if health.healthy {
                                    "Healthy".green()
                                } else {
                                    "Unhealthy".red()
                                };
                                println!("     Status:      {}", status);
                            }
                            println!();
                        }
                    }

                    Ok(())
                }
            }
        }

        Commands::Add {
            name,
            ver,
            driver,
            plugin,
            local: _,
            global: _,
            no_system: _,
        } => {
            // Add dependency to horus.yaml (does NOT install - deferred to `horus run`)
            // Following cargo/uv model: `add` edits manifest, `run/build` installs

            // Find horus.yaml in current directory or workspace
            let workspace_path = workspace::find_workspace_root()
                .unwrap_or_else(|| std::env::current_dir().unwrap_or_default());
            let horus_yaml_path = workspace_path.join("horus.yaml");

            if !horus_yaml_path.exists() {
                return Err(HorusError::Config(
                    "No horus.yaml found. Run 'horus init' or 'horus new' first.".to_string(),
                ));
            }

            // Determine package type - either from flags or auto-detect from registry
            let pkg_type = if driver {
                "driver".to_string()
            } else if plugin {
                "plugin".to_string()
            } else {
                // Auto-detect from registry
                registry::fetch_package_type(&name).unwrap_or_else(|_| "node".to_string())
            };

            let version = ver.as_deref().unwrap_or("latest");

            // Format dependency string based on type
            let dep_string = match pkg_type.as_str() {
                "driver" => format!("driver:{}@{}", name, version),
                "plugin" => format!("plugin:{}@{}", name, version),
                _ => {
                    if version == "latest" {
                        name.clone()
                    } else {
                        format!("{}@{}", name, version)
                    }
                }
            };

            // Add to horus.yaml
            match horus_manager::yaml_utils::add_dependency_to_horus_yaml(
                &horus_yaml_path,
                &dep_string,
                version,
            ) {
                Ok(_) => {
                    println!("{} Added '{}' to horus.yaml", "✓".green(), name.cyan());
                    println!("  Type: {}", pkg_type.dimmed());
                    if version != "latest" {
                        println!("  Version: {}", version.dimmed());
                    }
                    println!();
                    println!("Run {} to install dependencies.", "horus run".cyan().bold());
                }
                Err(e) => {
                    return Err(HorusError::Config(format!(
                        "Failed to update horus.yaml: {}",
                        e
                    )));
                }
            }

            Ok(())
        }

        Commands::Remove {
            name,
            global: _,
            purge: _,
        } => {
            // Remove dependency from horus.yaml (does NOT delete from cache)
            // Following cargo/uv model: cache stays, only manifest changes
            // Use `horus cache clean` to remove unused packages

            // Find horus.yaml in current directory or workspace
            let workspace_path = workspace::find_workspace_root()
                .unwrap_or_else(|| std::env::current_dir().unwrap_or_default());
            let horus_yaml_path = workspace_path.join("horus.yaml");

            if !horus_yaml_path.exists() {
                return Err(HorusError::Config(
                    "No horus.yaml found in current directory or workspace.".to_string(),
                ));
            }

            // Remove from horus.yaml
            match horus_manager::yaml_utils::remove_dependency_from_horus_yaml(
                &horus_yaml_path,
                &name,
            ) {
                Ok(_) => {
                    println!("{} Removed '{}' from horus.yaml", "✓".green(), name.cyan());
                    println!();
                    println!("Note: Package remains in cache (~/.horus/cache/).");
                    println!(
                        "Run {} to clean unused packages.",
                        "horus cache clean".dimmed()
                    );
                }
                Err(e) => {
                    // Check if it's a "not found" error
                    let err_str = e.to_string();
                    if err_str.contains("not found") {
                        println!("{} '{}' is not in horus.yaml", "!".yellow(), name);
                    } else {
                        return Err(HorusError::Config(format!(
                            "Failed to update horus.yaml: {}",
                            e
                        )));
                    }
                }
            }

            Ok(())
        }

        Commands::Plugin { command } => match command {
            PluginCommands::List { all } => commands::pkg::list_plugins(true, all)
                .map_err(|e| HorusError::Config(e.to_string())),

            PluginCommands::Search { query, category: _ } => {
                use horus_manager::plugins::PluginDiscovery;

                let mut discovery = PluginDiscovery::new();

                // Add softmata workspace for local plugins
                if let Ok(cwd) = std::env::current_dir() {
                    // Check if we're in softmata workspace
                    if cwd.join("horus").exists() {
                        discovery.add_workspace_path(cwd);
                    } else if let Some(parent) = cwd.parent() {
                        if parent.join("horus").exists() {
                            discovery.add_workspace_path(parent.to_path_buf());
                        }
                    }
                }

                let results = discovery
                    .search(&query)
                    .map_err(|e| HorusError::Config(e.to_string()))?;

                if results.is_empty() {
                    println!(
                        "{}",
                        format!("No plugins found matching '{}'", query).yellow()
                    );
                } else {
                    println!(
                        "{}",
                        format!("Found {} plugins matching '{}':", results.len(), query)
                            .cyan()
                            .bold()
                    );
                    println!();
                    for plugin in results {
                        let source_badge = match plugin.source {
                            horus_manager::plugins::PluginSourceType::Local => "[LOCAL]".yellow(),
                            horus_manager::plugins::PluginSourceType::Registry => {
                                "[REGISTRY]".green()
                            }
                            horus_manager::plugins::PluginSourceType::CratesIo => {
                                "[CRATES.IO]".blue()
                            }
                            horus_manager::plugins::PluginSourceType::Git => "[GIT]".magenta(),
                        };
                        println!(
                            "  {} {} {} {}",
                            plugin.name.white().bold(),
                            format!("v{}", plugin.version).dimmed(),
                            source_badge,
                            if plugin.has_prebuilt {
                                "[prebuilt]".green()
                            } else {
                                "".normal()
                            }
                        );
                        println!("    {}", plugin.description.dimmed());
                        if !plugin.features.is_empty() {
                            println!("    Features: {}", plugin.features.join(", ").cyan());
                        }
                        println!();
                    }
                }
                Ok(())
            }

            PluginCommands::Available {
                category: _,
                include_local,
            } => {
                use horus_manager::plugins::PluginDiscovery;

                let mut discovery = PluginDiscovery::new();

                if include_local {
                    if let Ok(cwd) = std::env::current_dir() {
                        if cwd.join("horus").exists() {
                            discovery.add_workspace_path(cwd);
                        } else if let Some(parent) = cwd.parent() {
                            if parent.join("horus").exists() {
                                discovery.add_workspace_path(parent.to_path_buf());
                            }
                        }
                    }
                }

                let all_plugins = discovery
                    .discover_all()
                    .map_err(|e| HorusError::Config(e.to_string()))?;

                println!("{}", "Available HORUS Plugins".cyan().bold());
                println!("{}", "=".repeat(50).dimmed());
                println!();

                // Group by category
                let mut by_category: std::collections::HashMap<
                    String,
                    Vec<horus_manager::plugins::AvailablePlugin>,
                > = std::collections::HashMap::new();

                for plugin in all_plugins {
                    let cat_name = format!("{:?}", plugin.category);
                    by_category.entry(cat_name).or_default().push(plugin);
                }

                for (cat, plugins) in by_category.iter() {
                    println!(
                        "{}",
                        format!("  {} ({} plugins)", cat, plugins.len())
                            .yellow()
                            .bold()
                    );
                    for plugin in plugins {
                        let source = match plugin.source {
                            horus_manager::plugins::PluginSourceType::Local => "local".yellow(),
                            horus_manager::plugins::PluginSourceType::Registry => {
                                "registry".green()
                            }
                            horus_manager::plugins::PluginSourceType::CratesIo => {
                                "crates.io".blue()
                            }
                            horus_manager::plugins::PluginSourceType::Git => "git".magenta(),
                        };
                        println!(
                            "    {} {} ({})",
                            plugin.name.white(),
                            format!("v{}", plugin.version).dimmed(),
                            source
                        );
                    }
                    println!();
                }

                println!("{}", "Use 'horus plugins info <name>' for details".dimmed());
                Ok(())
            }

            PluginCommands::Info { name } => {
                use horus_manager::plugins::PluginDiscovery;

                let mut discovery = PluginDiscovery::new();

                // Add local workspace
                if let Ok(cwd) = std::env::current_dir() {
                    if cwd.join("horus").exists() {
                        discovery.add_workspace_path(cwd);
                    }
                }

                match discovery
                    .get_plugin(&name)
                    .map_err(|e| HorusError::Config(e.to_string()))?
                {
                    Some(plugin) => {
                        println!("{}", plugin.name.cyan().bold());
                        println!("{}", "=".repeat(plugin.name.len()).dimmed());
                        println!();
                        println!("  Version:     {}", plugin.version.white());
                        println!("  Category:    {:?}", plugin.category);
                        println!("  Source:      {:?}", plugin.source);
                        println!("  Description: {}", plugin.description);
                        println!();
                        println!("  {}", "Compatibility".yellow());
                        println!("    HORUS:     {}", plugin.horus_compat);
                        println!("    Platforms: {}", plugin.platforms.join(", "));
                        println!(
                            "    Prebuilt:  {}",
                            if plugin.has_prebuilt { "Yes" } else { "No" }
                        );
                        println!();
                        if !plugin.system_deps.is_empty() {
                            println!("  {}", "System Dependencies".yellow());
                            for dep in &plugin.system_deps {
                                println!("    - {}", dep);
                            }
                            println!();
                        }
                        if !plugin.features.is_empty() {
                            println!("  {}", "Features".yellow());
                            for feature in &plugin.features {
                                println!("    - {}", feature);
                            }
                            println!();
                        }
                        println!("  {}", "Installation".yellow());
                        println!(
                            "{}",
                            discovery
                                .get_install_instructions(&plugin)
                                .lines()
                                .map(|l| format!("    {}", l))
                                .collect::<Vec<_>>()
                                .join("\n")
                        );
                    }
                    None => {
                        println!("{}", format!("Plugin '{}' not found", name).red());
                        println!(
                            "{}",
                            "Use 'horus plugins search <query>' to find plugins".dimmed()
                        );
                    }
                }
                Ok(())
            }

            PluginCommands::Enable { command } => commands::pkg::enable_plugin(&command)
                .map_err(|e| HorusError::Config(e.to_string())),

            PluginCommands::Disable { command, reason } => {
                commands::pkg::disable_plugin(&command, reason.as_deref())
                    .map_err(|e| HorusError::Config(e.to_string()))
            }

            PluginCommands::Verify { plugin } => commands::pkg::verify_plugins(plugin.as_deref())
                .map_err(|e| HorusError::Config(e.to_string())),
        },

        Commands::Cache { command } => {
            let home = dirs::home_dir()
                .ok_or_else(|| HorusError::Config("Could not find home directory".to_string()))?;
            let cache_dir = home.join(".horus/cache");

            match command {
                CacheCommands::Info => {
                    println!("{}", "HORUS Cache Information".cyan().bold());
                    println!("{}", "═".repeat(40));

                    if !cache_dir.exists() {
                        println!("Cache directory: {} (not created yet)", cache_dir.display());
                        println!("Total size: 0 B");
                        println!("Packages: 0");
                        return Ok(());
                    }

                    println!("Cache directory: {}", cache_dir.display());

                    // Count packages and calculate size
                    let mut total_size: u64 = 0;
                    let mut package_count = 0;

                    if let Ok(entries) = fs::read_dir(&cache_dir) {
                        for entry in entries.flatten() {
                            if entry.path().is_dir() {
                                package_count += 1;
                                // Calculate directory size
                                if let Ok(size) = dir_size(&entry.path()) {
                                    total_size += size;
                                }
                            }
                        }
                    }

                    println!("Total size: {}", format_size(total_size));
                    println!("Packages: {}", package_count);

                    Ok(())
                }

                CacheCommands::List => {
                    println!("{}", "Cached Packages".cyan().bold());
                    println!("{}", "─".repeat(60));

                    if !cache_dir.exists() {
                        println!("  (cache is empty)");
                        return Ok(());
                    }

                    let mut packages: Vec<_> = fs::read_dir(&cache_dir)
                        .map_err(|e| HorusError::Config(e.to_string()))?
                        .filter_map(|e| e.ok())
                        .filter(|e| e.path().is_dir())
                        .collect();

                    packages.sort_by_key(|e| e.file_name());

                    if packages.is_empty() {
                        println!("  (cache is empty)");
                    } else {
                        for entry in packages {
                            let name = entry.file_name().to_string_lossy().to_string();
                            let size = dir_size(&entry.path()).unwrap_or(0);
                            println!("  {} {}", name.yellow(), format_size(size).dimmed());
                        }
                    }

                    Ok(())
                }

                CacheCommands::Clean { dry_run } => {
                    println!("{} Scanning for unused packages...", "[CACHE]".cyan());

                    if !cache_dir.exists() {
                        println!("Cache is empty, nothing to clean.");
                        return Ok(());
                    }

                    // Find all workspaces and their dependencies
                    let registry = workspace::WorkspaceRegistry::load()
                        .map_err(|e| HorusError::Config(e.to_string()))?;

                    let mut used_packages: std::collections::HashSet<String> =
                        std::collections::HashSet::new();

                    for ws in &registry.workspaces {
                        let yaml_path = ws.path.join("horus.yaml");
                        if yaml_path.exists() {
                            if let Ok(deps) =
                                horus_manager::commands::run::parse_horus_yaml_dependencies_v2(
                                    yaml_path.to_str().unwrap_or(""),
                                )
                            {
                                for dep in deps {
                                    // Extract package name from dependency spec
                                    used_packages.insert(dep.name.clone());
                                }
                            }
                        }
                    }

                    // Find cached packages not in use
                    let mut to_remove = Vec::new();
                    let mut freed_size: u64 = 0;

                    if let Ok(entries) = fs::read_dir(&cache_dir) {
                        for entry in entries.flatten() {
                            if entry.path().is_dir() {
                                let name = entry.file_name().to_string_lossy().to_string();
                                let pkg_base = name.split('@').next().unwrap_or(&name);

                                if !used_packages.contains(pkg_base) {
                                    let size = dir_size(&entry.path()).unwrap_or(0);
                                    freed_size += size;
                                    to_remove.push((entry.path(), name, size));
                                }
                            }
                        }
                    }

                    if to_remove.is_empty() {
                        println!("{} All cached packages are in use.", "✓".green());
                        return Ok(());
                    }

                    println!("\nUnused packages:");
                    for (_, name, size) in &to_remove {
                        println!("  {} {} ({})", "×".red(), name, format_size(*size));
                    }
                    println!("\nTotal to free: {}", format_size(freed_size).green());

                    if dry_run {
                        println!("\n{} Dry run - no files removed.", "[DRY]".yellow());
                    } else {
                        for (path, name, _) in &to_remove {
                            if let Err(e) = fs::remove_dir_all(path) {
                                println!("  {} Failed to remove {}: {}", "!".red(), name, e);
                            }
                        }
                        println!(
                            "\n{} Removed {} packages, freed {}.",
                            "✓".green(),
                            to_remove.len(),
                            format_size(freed_size)
                        );
                    }

                    Ok(())
                }

                CacheCommands::Purge { yes } => {
                    if !cache_dir.exists() {
                        println!("Cache is already empty.");
                        return Ok(());
                    }

                    let total_size = dir_size(&cache_dir).unwrap_or(0);
                    let count = fs::read_dir(&cache_dir).map(|e| e.count()).unwrap_or(0);

                    println!(
                        "{} This will remove ALL cached packages:",
                        "[WARN]".yellow().bold()
                    );
                    println!("  Packages: {}", count);
                    println!("  Size: {}", format_size(total_size));
                    println!();

                    if !yes {
                        print!("Continue? [y/N] ");
                        use std::io::Write;
                        std::io::stdout().flush().ok();

                        let mut input = String::new();
                        std::io::stdin().read_line(&mut input).ok();

                        if !input.trim().eq_ignore_ascii_case("y") {
                            println!("Aborted.");
                            return Ok(());
                        }
                    }

                    fs::remove_dir_all(&cache_dir)
                        .map_err(|e| HorusError::Config(format!("Failed to purge cache: {}", e)))?;

                    println!(
                        "{} Cache purged. Freed {}.",
                        "✓".green(),
                        format_size(total_size)
                    );
                    Ok(())
                }
            }
        }

        Commands::Record { command } => {
            use horus_core::scheduling::{diff_recordings, RecordingManager};

            let manager = RecordingManager::new();

            match command {
                RecordCommands::List { long } => {
                    let sessions = manager.list_sessions().map_err(|e| {
                        HorusError::Internal(format!("Failed to list recordings: {}", e))
                    })?;

                    if sessions.is_empty() {
                        println!("{} No recording sessions found.", "[INFO]".cyan());
                        println!("       Use 'horus run --record <session>' to create one.");
                    } else {
                        println!(
                            "{} Found {} recording session(s):\n",
                            "".green(),
                            sessions.len()
                        );

                        for session in sessions {
                            if long {
                                // Get detailed info
                                let recordings =
                                    manager.get_session_recordings(&session).unwrap_or_default();
                                let total_size: u64 = recordings
                                    .iter()
                                    .filter_map(|p| std::fs::metadata(p).ok())
                                    .map(|m| m.len())
                                    .sum();

                                println!(
                                    "  {} {} ({} files, {:.1} MB)",
                                    "".green(),
                                    session.yellow(),
                                    recordings.len(),
                                    total_size as f64 / 1_048_576.0
                                );
                            } else {
                                println!("  {} {}", "".green(), session.yellow());
                            }
                        }
                    }
                    Ok(())
                }

                RecordCommands::Info { session } => {
                    let recordings = manager.get_session_recordings(&session).map_err(|e| {
                        HorusError::Internal(format!("Failed to get session info: {}", e))
                    })?;

                    if recordings.is_empty() {
                        println!("{} Session '{}' not found.", "[WARN]".yellow(), session);
                        return Ok(());
                    }

                    println!("{} Session: {}\n", "".green(), session.yellow().bold());

                    for path in recordings {
                        let filename = path
                            .file_name()
                            .and_then(|n| n.to_str())
                            .unwrap_or("unknown");
                        let size = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);

                        // Try to load and get tick count
                        let tick_info = if let Ok(recording) =
                            horus_core::scheduling::NodeRecording::load(&path)
                        {
                            format!("ticks {}-{}", recording.first_tick, recording.last_tick)
                        } else {
                            "scheduler recording".to_string()
                        };

                        println!(
                            "  {} {} ({:.1} KB, {})",
                            "".cyan(),
                            filename,
                            size as f64 / 1024.0,
                            tick_info
                        );
                    }
                    Ok(())
                }

                RecordCommands::Delete { session, force } => {
                    if !force {
                        println!(
                            "{} Delete session '{}'? (y/N)",
                            "[CONFIRM]".yellow(),
                            session
                        );
                        let mut input = String::new();
                        std::io::stdin().read_line(&mut input).ok();
                        if !input.trim().eq_ignore_ascii_case("y") {
                            println!("Cancelled.");
                            return Ok(());
                        }
                    }

                    manager.delete_session(&session).map_err(|e| {
                        HorusError::Internal(format!("Failed to delete session: {}", e))
                    })?;

                    println!("{} Deleted session '{}'", "".green(), session);
                    Ok(())
                }

                RecordCommands::Replay {
                    recording,
                    start_tick,
                    stop_tick,
                    speed,
                    overrides,
                } => {
                    use horus_core::Scheduler;
                    use std::path::PathBuf;

                    // Resolve the recording path - could be a direct path or a session name
                    let scheduler_path = if PathBuf::from(&recording).exists() {
                        // Direct path to recording file
                        PathBuf::from(&recording)
                    } else {
                        // Treat as session name - find the scheduler recording
                        let recordings =
                            manager.get_session_recordings(&recording).map_err(|e| {
                                HorusError::Internal(format!(
                                    "Failed to get session '{}': {}",
                                    recording, e
                                ))
                            })?;

                        if recordings.is_empty() {
                            return Err(HorusError::Internal(format!(
                                "Session '{}' not found or has no recordings",
                                recording
                            )));
                        }

                        // Find the scheduler recording (file starting with "scheduler@")
                        recordings
                            .iter()
                            .find(|p| {
                                p.file_name()
                                    .and_then(|n| n.to_str())
                                    .map(|n| n.starts_with("scheduler@"))
                                    .unwrap_or(false)
                            })
                            .cloned()
                            .ok_or_else(|| {
                                HorusError::Internal(format!(
                                    "No scheduler recording found in session '{}'",
                                    recording
                                ))
                            })?
                    };

                    println!(
                        "{} Loading recording from: {}",
                        "[REPLAY]".cyan(),
                        scheduler_path.display()
                    );

                    // Load the scheduler from recording
                    let mut scheduler = Scheduler::replay_from(scheduler_path)?;

                    // Apply start tick if specified
                    if let Some(start) = start_tick {
                        scheduler = scheduler.start_at_tick(start);
                    }

                    // Apply stop tick if specified
                    if let Some(stop) = stop_tick {
                        scheduler = scheduler.stop_at_tick(stop);
                        println!("{} Will stop at tick {}", "[REPLAY]".cyan(), stop);
                    }

                    // Apply speed multiplier
                    if (speed - 1.0).abs() > f64::EPSILON {
                        scheduler = scheduler.with_replay_speed(speed);
                        println!("{} Playback speed: {}x", "[REPLAY]".cyan(), speed);
                    }

                    // Apply overrides
                    for (node, output, value_str) in &overrides {
                        // Parse the value string into bytes
                        // Support formats: hex (0x...), decimal numbers, or raw strings
                        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
                            // Hex format - parse manually without hex crate
                            parse_hex_string(hex_str)
                                .unwrap_or_else(|_| value_str.as_bytes().to_vec())
                        } else if let Ok(num) = value_str.parse::<f64>() {
                            // Float number
                            num.to_le_bytes().to_vec()
                        } else if let Ok(num) = value_str.parse::<i64>() {
                            // Integer number
                            num.to_le_bytes().to_vec()
                        } else {
                            // Raw string bytes
                            value_str.as_bytes().to_vec()
                        };
                        scheduler = scheduler.with_override(node, output, value_bytes);
                    }

                    println!("{} Starting replay...\n", "[REPLAY]".green());

                    // Run the replay
                    scheduler.run()?;

                    println!("\n{} Replay completed", "[DONE]".green());
                    Ok(())
                }

                RecordCommands::Diff {
                    session1,
                    session2,
                    limit,
                } => {
                    println!(
                        "{} Comparing '{}' vs '{}'...\n",
                        "[DIFF]".cyan(),
                        session1,
                        session2
                    );

                    // Get recordings from both sessions
                    let recordings1 = manager.get_session_recordings(&session1).map_err(|e| {
                        HorusError::Internal(format!(
                            "Failed to load session '{}': {}",
                            session1, e
                        ))
                    })?;

                    let recordings2 = manager.get_session_recordings(&session2).map_err(|e| {
                        HorusError::Internal(format!(
                            "Failed to load session '{}': {}",
                            session2, e
                        ))
                    })?;

                    // Find matching nodes
                    let mut total_diffs = 0;
                    let max_diffs = limit.unwrap_or(100);

                    for path1 in &recordings1 {
                        let name1 = path1
                            .file_stem()
                            .and_then(|n| n.to_str())
                            .unwrap_or("")
                            .split('@')
                            .next()
                            .unwrap_or("");

                        for path2 in &recordings2 {
                            let name2 = path2
                                .file_stem()
                                .and_then(|n| n.to_str())
                                .unwrap_or("")
                                .split('@')
                                .next()
                                .unwrap_or("");

                            if name1 == name2 && !name1.is_empty() && name1 != "scheduler" {
                                // Load and compare
                                if let (Ok(rec1), Ok(rec2)) = (
                                    horus_core::scheduling::NodeRecording::load(path1),
                                    horus_core::scheduling::NodeRecording::load(path2),
                                ) {
                                    let diffs = diff_recordings(&rec1, &rec2);
                                    if !diffs.is_empty() {
                                        println!(
                                            "  {} Node '{}': {} differences",
                                            "".yellow(),
                                            name1,
                                            diffs.len()
                                        );
                                        for diff in diffs.iter().take(max_diffs - total_diffs) {
                                            match diff {
                                                horus_core::scheduling::RecordingDiff::OutputDifference { tick, topic, .. } => {
                                                    println!("    Tick {}: output '{}' differs", tick, topic);
                                                }
                                                horus_core::scheduling::RecordingDiff::MissingTick { tick, in_recording } => {
                                                    println!("    Tick {} missing in recording {}", tick, in_recording);
                                                }
                                                horus_core::scheduling::RecordingDiff::MissingOutput { tick, topic, in_recording } => {
                                                    println!("    Tick {}: output '{}' missing in recording {}", tick, topic, in_recording);
                                                }
                                            }
                                            total_diffs += 1;
                                        }
                                    } else {
                                        println!("  {} Node '{}': identical", "".green(), name1);
                                    }
                                }
                            }
                        }
                    }

                    if total_diffs == 0 {
                        println!("\n{} No differences found!", "".green());
                    } else {
                        println!("\n{} Total: {} difference(s)", "".yellow(), total_diffs);
                    }
                    Ok(())
                }

                RecordCommands::Export {
                    session,
                    output,
                    format,
                } => {
                    println!(
                        "{} Exporting session '{}' to {:?} (format: {})",
                        "[EXPORT]".cyan(),
                        session,
                        output,
                        format
                    );

                    let recordings = manager.get_session_recordings(&session).map_err(|e| {
                        HorusError::Internal(format!("Failed to load session: {}", e))
                    })?;

                    if format == "json" {
                        use std::io::Write;
                        let mut file = std::fs::File::create(&output).map_err(|e| {
                            HorusError::Internal(format!("Failed to create output file: {}", e))
                        })?;

                        writeln!(file, "{{")?;
                        writeln!(file, "  \"session\": \"{}\",", session)?;
                        writeln!(file, "  \"recordings\": [")?;

                        for (i, path) in recordings.iter().enumerate() {
                            if let Ok(recording) = horus_core::scheduling::NodeRecording::load(path)
                            {
                                let comma = if i < recordings.len() - 1 { "," } else { "" };
                                writeln!(file, "    {{")?;
                                writeln!(
                                    file,
                                    "      \"node_name\": \"{}\",",
                                    recording.node_name
                                )?;
                                writeln!(file, "      \"node_id\": \"{}\",", recording.node_id)?;
                                writeln!(file, "      \"first_tick\": {},", recording.first_tick)?;
                                writeln!(file, "      \"last_tick\": {},", recording.last_tick)?;
                                writeln!(
                                    file,
                                    "      \"snapshot_count\": {}",
                                    recording.snapshots.len()
                                )?;
                                writeln!(file, "    }}{}", comma)?;
                            }
                        }

                        writeln!(file, "  ]")?;
                        writeln!(file, "}}")?;

                        println!("{} Exported to {:?}", "".green(), output);
                    } else if format == "csv" {
                        use std::io::Write;
                        let mut file = std::fs::File::create(&output).map_err(|e| {
                            HorusError::Internal(format!("Failed to create output file: {}", e))
                        })?;

                        // Write CSV header
                        writeln!(
                            file,
                            "node_name,node_id,tick,timestamp_us,input_count,output_count,outputs"
                        )?;

                        // Process each recording
                        for path in &recordings {
                            if let Ok(recording) = horus_core::scheduling::NodeRecording::load(path)
                            {
                                // Skip scheduler recordings (they have different structure)
                                if recording.node_name.starts_with("scheduler") {
                                    continue;
                                }

                                for snapshot in &recording.snapshots {
                                    // Serialize outputs as hex string for CSV
                                    let outputs_str: Vec<String> = snapshot
                                        .outputs
                                        .iter()
                                        .map(|(k, v)| {
                                            let hex: String =
                                                v.iter().map(|b| format!("{:02x}", b)).collect();
                                            format!("{}={}", k, hex)
                                        })
                                        .collect();

                                    writeln!(
                                        file,
                                        "{},{},{},{},{},{},\"{}\"",
                                        recording.node_name,
                                        recording.node_id,
                                        snapshot.tick,
                                        snapshot.timestamp_us,
                                        snapshot.inputs.len(),
                                        snapshot.outputs.len(),
                                        outputs_str.join(";")
                                    )?;
                                }
                            }
                        }

                        println!("{} Exported to {:?} (CSV format)", "".green(), output);
                    } else {
                        println!(
                            "{} Format '{}' not supported. Use 'json' or 'csv'.",
                            "[WARN]".yellow(),
                            format
                        );
                    }
                    Ok(())
                }

                RecordCommands::Inject {
                    session,
                    nodes,
                    all,
                    script,
                    start_tick,
                    stop_tick,
                    speed,
                    loop_playback,
                } => {
                    use horus_core::Scheduler;

                    println!(
                        "{} Injecting recorded nodes from session '{}'",
                        "[INJECT]".cyan(),
                        session
                    );

                    // Get all recordings from the session
                    let recordings = manager.get_session_recordings(&session).map_err(|e| {
                        HorusError::Internal(format!("Failed to load session '{}': {}", session, e))
                    })?;

                    if recordings.is_empty() {
                        return Err(HorusError::Internal(format!(
                            "Session '{}' not found or has no recordings",
                            session
                        )));
                    }

                    // Create a new scheduler for hybrid execution
                    let mut scheduler = Scheduler::new().with_name(&format!("Inject({})", session));

                    // Filter and add replay nodes
                    let mut injected_count = 0;
                    for path in &recordings {
                        let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("");

                        // Skip scheduler recordings
                        if filename.starts_with("scheduler@") {
                            continue;
                        }

                        // Extract node name from filename (format: node_name@id.horus)
                        let node_name = filename.split('@').next().unwrap_or("");

                        // Check if we should inject this node
                        let should_inject = all || nodes.iter().any(|n| n == node_name);

                        if should_inject {
                            match scheduler.add_replay(path.clone(), 0) {
                                Ok(_) => {
                                    println!("  {} Injected '{}'", "✓".green(), node_name);
                                    injected_count += 1;
                                }
                                Err(e) => {
                                    eprintln!(
                                        "  {} Failed to inject '{}': {}",
                                        "✗".red(),
                                        node_name,
                                        e
                                    );
                                }
                            }
                        }
                    }

                    if injected_count == 0 {
                        return Err(HorusError::Internal(
                            "No nodes were injected. Check node names or use --all".to_string(),
                        ));
                    }

                    // Apply start tick if specified
                    if let Some(start) = start_tick {
                        scheduler = scheduler.start_at_tick(start);
                    }

                    // Apply stop tick if specified
                    if let Some(stop) = stop_tick {
                        scheduler = scheduler.stop_at_tick(stop);
                    }

                    // Apply speed multiplier
                    if (speed - 1.0).abs() > f64::EPSILON {
                        scheduler = scheduler.with_replay_speed(speed);
                        println!("{} Playback speed: {}x", "[INJECT]".cyan(), speed);
                    }

                    // If a script is provided, compile and run it with the injected nodes
                    if let Some(script_path) = &script {
                        if !script_path.exists() {
                            return Err(HorusError::Internal(format!(
                                "Script file not found: {}",
                                script_path.display()
                            )));
                        }

                        println!(
                            "\n{} Compiling script: {}",
                            "[INJECT]".cyan(),
                            script_path.display()
                        );

                        // Use the existing run infrastructure to compile and execute
                        // We need to set up the environment for injection
                        std::env::set_var("HORUS_INJECT_SESSION", &session);
                        std::env::set_var(
                            "HORUS_INJECT_NODES",
                            if all {
                                "*".to_string()
                            } else {
                                nodes.join(",")
                            },
                        );

                        // For now, just inform user how to properly use injection with scripts
                        // Full integration would require modifying the `horus run` command
                        println!(
                            "\n{} To run a script with injected recordings, use:",
                            "[INFO]".cyan()
                        );
                        let inject_arg = if all {
                            "--inject-all".to_string()
                        } else {
                            format!("--inject-nodes {}", nodes.join(","))
                        };
                        println!(
                            "       horus run {} --inject {} {}",
                            script_path.display(),
                            session,
                            inject_arg
                        );
                        println!(
                            "\n{} Running injected nodes only for now...\n",
                            "[INJECT]".yellow()
                        );
                    } else {
                        println!(
                            "\n{} Running {} injected node(s)...\n",
                            "[INJECT]".green(),
                            injected_count
                        );
                    }

                    // Handle loop playback
                    if loop_playback {
                        println!(
                            "{} Loop mode: Recording will restart when finished",
                            "[INJECT]".cyan()
                        );

                        // Run in a loop until interrupted
                        loop {
                            // Reset tick counter for new iteration
                            scheduler = scheduler.start_at_tick(start_tick.unwrap_or(0));

                            match scheduler.run() {
                                Ok(()) => {
                                    println!(
                                        "\n{} Recording finished, restarting...\n",
                                        "[LOOP]".cyan()
                                    );

                                    // Recreate scheduler for next iteration
                                    scheduler =
                                        Scheduler::new().with_name(&format!("Inject({})", session));

                                    // Re-inject nodes
                                    for path in &recordings {
                                        let filename =
                                            path.file_name().and_then(|n| n.to_str()).unwrap_or("");
                                        if filename.starts_with("scheduler@") {
                                            continue;
                                        }
                                        let node_name = filename.split('@').next().unwrap_or("");
                                        let should_inject =
                                            all || nodes.iter().any(|n| n == node_name);
                                        if should_inject {
                                            let _ = scheduler.add_replay(path.clone(), 0);
                                        }
                                    }

                                    // Re-apply settings
                                    if let Some(start) = start_tick {
                                        scheduler = scheduler.start_at_tick(start);
                                    }
                                    if let Some(stop) = stop_tick {
                                        scheduler = scheduler.stop_at_tick(stop);
                                    }
                                    if (speed - 1.0).abs() > f64::EPSILON {
                                        scheduler = scheduler.with_replay_speed(speed);
                                    }
                                }
                                Err(e) => {
                                    // If interrupted (Ctrl+C), exit loop
                                    println!("\n{} Loop interrupted: {}", "[DONE]".yellow(), e);
                                    break;
                                }
                            }
                        }
                    } else {
                        // Single run
                        scheduler.run()?;
                        println!("\n{} Injection replay completed", "[DONE]".green());
                    }

                    Ok(())
                }
            }
        }

        Commands::Net { command } => match command {
            NetCommands::Check { verbose } => commands::net::run_check(verbose),
            NetCommands::Ping {
                endpoint,
                count,
                interval,
            } => commands::net::run_ping(&endpoint, count, interval),
            NetCommands::Trace { endpoint, max_hops } => {
                commands::net::run_trace(&endpoint, max_hops)
            }
            NetCommands::Doctor => commands::net::run_doctor(),
            #[cfg(feature = "signaling-server")]
            NetCommands::SignalServer {
                port,
                bind,
                secret,
                max_peers,
                verbose,
            } => commands::net::run_signal_server(&bind, port, secret, max_peers, verbose),
        },

        Commands::Husarnet { command } => match command {
            HusarnetCommands::Status { verbose } => commands::husarnet::run_status(verbose),
            HusarnetCommands::Peers { json } => commands::husarnet::run_peers(json),
            HusarnetCommands::Test {
                target,
                count,
                timeout,
            } => commands::husarnet::run_test(target, count, timeout),
            HusarnetCommands::Doctor => commands::husarnet::run_doctor(),
        },

        Commands::Completion { shell } => {
            // Hidden command used by install.sh for automatic completion setup
            let mut cmd = Cli::command();
            let bin_name = cmd.get_name().to_string();
            generate(shell, &mut cmd, bin_name, &mut io::stdout());
            Ok(())
        }
    }
}

// Helper functions for system package detection during restore

#[derive(Debug, Clone, PartialEq)]
enum MissingSystemChoice {
    InstallGlobal,
    InstallLocal,
    Skip,
}

fn check_system_package_exists(package_name: &str) -> bool {
    use std::process::Command;

    // Try Python package detection
    let py_check = Command::new("python3")
        .args(["-m", "pip", "show", package_name])
        .output();

    if let Ok(output) = py_check {
        if output.status.success() {
            return true;
        }
    }

    // Try Rust binary detection
    if let Some(home) = dirs::home_dir() {
        let cargo_bin = home.join(".cargo/bin").join(package_name);
        if cargo_bin.exists() {
            return true;
        }
    }

    false
}

/// Parse Python imports from a file
fn parse_python_imports(python_file: &Path) -> Result<Vec<String>, std::io::Error> {
    let content = std::fs::read_to_string(python_file)?;
    let mut imports = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();

        // Skip comments and empty lines
        if trimmed.starts_with('#') || trimmed.is_empty() {
            continue;
        }

        // Parse "import X" or "import X as Y"
        if let Some(rest) = trimmed.strip_prefix("import ") {
            let module = rest
                .split_whitespace()
                .next()
                .unwrap_or("")
                .split('.')
                .next()
                .unwrap_or("")
                .split(',')
                .next()
                .unwrap_or("")
                .trim();

            if !module.is_empty() && !imports.contains(&module.to_string()) {
                imports.push(module.to_string());
            }
        }

        // Parse "from X import Y"
        if let Some(rest) = trimmed.strip_prefix("from ") {
            if let Some(module) = rest.split_whitespace().next() {
                let base_module = module.split('.').next().unwrap_or("");
                if !base_module.is_empty() && !imports.contains(&base_module.to_string()) {
                    imports.push(base_module.to_string());
                }
            }
        }
    }

    // Filter out standard library and relative imports
    let stdlib_modules = vec![
        "os",
        "sys",
        "re",
        "json",
        "math",
        "time",
        "datetime",
        "collections",
        "itertools",
        "functools",
        "pathlib",
        "typing",
        "abc",
        "io",
        "logging",
        "argparse",
        "subprocess",
        "threading",
        "multiprocessing",
        "queue",
        "socket",
        "http",
        "urllib",
        "email",
        "xml",
        "html",
        "random",
        "string",
        "unittest",
        "pytest",
        "asyncio",
        "concurrent",
        "pickle",
        "copy",
        "enum",
        "dataclasses",
        "contextlib",
        "warnings",
        "traceback",
        "pdb",
        "timeit",
    ];

    imports.retain(|module| !stdlib_modules.contains(&module.as_str()) && module != "horus");

    Ok(imports)
}

fn prompt_missing_system_package(package_name: &str) -> Result<MissingSystemChoice, HorusError> {
    use std::io::{self, Write};

    println!(
        "\n  System package '{}' was expected but not found.",
        package_name
    );
    println!("  What would you like to do?");
    println!("    [1] Install to HORUS global cache (shared across projects)");
    println!("    [2] Install to HORUS local (this project only)");
    println!("    [3] Skip (you will install it manually later)");

    print!("\n  Choice [1-3]: ");
    io::stdout()
        .flush()
        .map_err(|e| HorusError::Config(e.to_string()))?;

    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .map_err(|e| HorusError::Config(e.to_string()))?;

    match input.trim() {
        "1" => Ok(MissingSystemChoice::InstallGlobal),
        "2" => Ok(MissingSystemChoice::InstallLocal),
        "3" => Ok(MissingSystemChoice::Skip),
        _ => {
            println!("  Invalid choice, defaulting to Skip");
            Ok(MissingSystemChoice::Skip)
        }
    }
}
