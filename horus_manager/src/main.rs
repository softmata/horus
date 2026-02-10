use clap::{CommandFactory, Parser, Subcommand};
use clap_complete::generate;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::io;
use std::path::PathBuf;

// Use modules from the library instead of redeclaring them
use horus_manager::{commands, monitor, monitor_tui, security};

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

        Commands::Check { path, quiet } => commands::check::run_check(path, quiet),

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

        Commands::Pkg { command } => match command {
            PkgCommands::Install {
                package,
                ver,
                global,
                target,
            } => commands::pkg::run_install(package, ver, global, target),
            PkgCommands::Remove {
                package,
                global,
                target,
            } => commands::pkg::run_remove(package, global, target),
            PkgCommands::List { query, global, all } => commands::pkg::run_list(query, global, all),
            PkgCommands::Publish { freeze, dry_run } => commands::pkg::run_publish(freeze, dry_run),
            PkgCommands::Update {
                package,
                global,
                dry_run,
            } => commands::pkg::run_update(package, global, dry_run),
            PkgCommands::KeyGen => commands::pkg::run_keygen(),
            PkgCommands::Unpublish {
                package,
                version,
                yes,
            } => commands::pkg::run_unpublish(package, version, yes),
        },

        Commands::Env { command } => match command {
            EnvCommands::Freeze { output, publish } => commands::env::run_freeze(output, publish),
            EnvCommands::Restore { source } => commands::env::run_restore(source),
            EnvCommands::List => commands::env::run_list(),
            EnvCommands::Show { id } => commands::env::run_show(id),
        },

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

        Commands::Driver { command } => match command {
            DriverCommands::List {
                category,
                registry_only,
                plugins_only,
                mode,
            } => commands::driver::run_list(category, registry_only, plugins_only, mode),
            DriverCommands::Info { driver } => commands::driver::run_info(driver),
            DriverCommands::Search { query, bus_type } => {
                commands::driver::run_search(query, bus_type)
            }
            DriverCommands::Probe {
                plugin,
                backend,
                mode,
            } => commands::driver::run_probe(plugin, backend, mode),
            DriverCommands::Plugins { reload, mode } => commands::driver::run_plugins(reload, mode),
        },

        Commands::Add {
            name,
            ver,
            driver,
            plugin,
            local: _,
            global: _,
            no_system: _,
        } => commands::pkg::run_add(name, ver, driver, plugin),

        Commands::Remove {
            name,
            global: _,
            purge: _,
        } => commands::pkg::run_remove_dep(name),

        Commands::Plugin { command } => match command {
            PluginCommands::List { all } => commands::pkg::list_plugins(true, all)
                .map_err(|e| HorusError::Config(e.to_string())),
            PluginCommands::Search { query, category: _ } => commands::plugin::run_search(query),
            PluginCommands::Available {
                category: _,
                include_local,
            } => commands::plugin::run_available(include_local),
            PluginCommands::Info { name } => commands::plugin::run_info(name),
            PluginCommands::Enable { command } => commands::pkg::enable_plugin(&command)
                .map_err(|e| HorusError::Config(e.to_string())),
            PluginCommands::Disable { command, reason } => {
                commands::pkg::disable_plugin(&command, reason.as_deref())
                    .map_err(|e| HorusError::Config(e.to_string()))
            }
            PluginCommands::Verify { plugin } => commands::pkg::verify_plugins(plugin.as_deref())
                .map_err(|e| HorusError::Config(e.to_string())),
        },

        Commands::Cache { command } => match command {
            CacheCommands::Info => commands::cache::run_info(),
            CacheCommands::List => commands::cache::run_list(),
            CacheCommands::Clean { dry_run } => commands::cache::run_clean(dry_run),
            CacheCommands::Purge { yes } => commands::cache::run_purge(yes),
        },

        Commands::Record { command } => match command {
            RecordCommands::List { long } => commands::record::run_list(long),
            RecordCommands::Info { session } => commands::record::run_info(session),
            RecordCommands::Delete { session, force } => {
                commands::record::run_delete(session, force)
            }
            RecordCommands::Replay {
                recording,
                start_tick,
                stop_tick,
                speed,
                overrides,
            } => commands::record::run_replay(recording, start_tick, stop_tick, speed, overrides),
            RecordCommands::Diff {
                session1,
                session2,
                limit,
            } => commands::record::run_diff(session1, session2, limit),
            RecordCommands::Export {
                session,
                output,
                format,
            } => commands::record::run_export(session, output, format),
            RecordCommands::Inject {
                session,
                nodes,
                all,
                script,
                start_tick,
                stop_tick,
                speed,
                loop_playback,
            } => commands::record::run_inject(
                session,
                nodes,
                all,
                script,
                start_tick,
                stop_tick,
                speed,
                loop_playback,
            ),
        },

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
        },

        Commands::Husarnet { command } => match command {
            HusarnetCommands::Status { verbose } => commands::husarnet::run_status(verbose),
            HusarnetCommands::Peers { json } => commands::husarnet::run_peers(json),
            HusarnetCommands::Test {
                target,
                count,
                timeout,
            } => commands::husarnet::run_test(target, count, timeout),
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
