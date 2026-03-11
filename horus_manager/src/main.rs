use clap::{CommandFactory, Parser, Subcommand};
use clap_complete::generate;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::io;
use std::path::PathBuf;

// Use modules from the library instead of redeclaring them
use horus_manager::{commands, monitor, monitor_tui, security};

#[derive(Parser)]
#[command(name = "horus")]
#[command(about = "Real-time robotics framework with zero-copy IPC and deterministic scheduling")]
#[command(version = "0.1.9")]
#[command(propagate_version = true)]
#[command(disable_help_subcommand = true)]
#[command(help_template = "\
{before-help}{name} {version} — {about}

{usage-heading} {usage}

Project:
  init              Initialize HORUS workspace in current directory
  new               Create a new HORUS project
  run               Run a HORUS project or file(s)
  build             Build the HORUS project without running
  test              Run tests for the HORUS project
  check             Validate horus.toml, source files, or workspace
  clean             Clean build artifacts and shared memory
  launch, l         Launch multiple nodes from a YAML file

Introspection:
  topic, t          Topic interaction (list, echo, publish, bw)
  node, n           Node management (list, info, kill)
  service, srv      Service interaction (list, call, info, find)
  action, a         Action interaction (list, info, send-goal, cancel-goal)
  param, p          Parameter management (get, set, list, delete)
  frame, frames     Coordinate frame operations (list, echo, tree)
  msg, m            Message type introspection
  discover          Discover HORUS nodes on the local network

Debugging:
  log               View and filter logs
  blackbox, bb      Inspect the BlackBox flight recorder
  monitor, mon      Monitor nodes, topics, and system health
  record, rec       Record/replay management
  cache             Cache management (info, clean, purge)

Packages:
  install, i        Install a package or plugin (name@version supported)
  remove            Remove a package, driver, or plugin
  search, s         Search available packages from registry
  list              List installed packages and plugins
  update            Update installed packages to latest versions
  info              Show detailed info about a package or plugin

Plugins:
  enable            Enable a disabled plugin
  disable           Disable a plugin (keep installed but don't execute)
  verify            Verify integrity of installed plugins

Publishing & Deploy:
  publish           Publish package to registry
  unpublish         Unpublish a package (name@version syntax)
  keygen            Generate signing key pair for package signing
  deploy            Deploy project to a remote robot
  env               Environment management (freeze/restore)
  auth              Authentication commands

{options}
{after-help}")]
#[command(after_help = "\
Quick Start:
  horus new my_robot -r           Create a new Rust project
  cd my_robot && horus run        Build and run it
  horus topic list                See active topics

More examples:
  horus init                      Initialize workspace in current directory
  horus install rplidar@1.2.0     Install a specific package version
  horus bb --anomalies            Show crash anomalies
  horus deploy robot@192.168.1.5  Deploy to a remote robot

Docs: https://docs.horus-registry.dev")]
struct Cli {
    /// Increase output verbosity (show debug messages)
    #[arg(short = 'v', long = "verbose", global = true)]
    verbose: bool,

    /// Suppress progress and informational output
    #[arg(short = 'q', long = "quiet", global = true)]
    quiet: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    // ── Project ──────────────────────────────────────────────────────────
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

        /// Override detected drivers (comma-separated list)
        /// Example: --drivers camera,lidar,imu
        #[arg(short = 'd', long = "drivers", value_delimiter = ',')]
        drivers: Option<Vec<String>>,

        /// Enable capabilities (comma-separated list)
        /// Example: --enable cuda,editor,python
        #[arg(short = 'e', long = "enable", value_delimiter = ',')]
        enable: Option<Vec<String>>,
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

        /// Show test stdout/stderr (verbose output)
        #[arg(long = "verbose")]
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

    /// Validate horus.toml, source files, or entire workspace
    Check {
        /// Path to file, directory, or workspace (default: current directory)
        #[arg(value_name = "PATH")]
        path: Option<PathBuf>,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
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

        /// Force clean even if HORUS processes are running
        #[arg(short = 'f', long = "force")]
        force: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Launch multiple nodes from a YAML file
    #[command(visible_alias = "l")]
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

        /// Seconds to wait for graceful shutdown before SIGKILL (default: 2)
        #[arg(long = "shutdown-timeout", default_value = "2")]
        shutdown_timeout: u64,
    },

    // ── Introspection ────────────────────────────────────────────────────
    /// Topic interaction (list, echo, publish)
    #[command(visible_alias = "t")]
    Topic {
        #[command(subcommand)]
        command: TopicCommands,
    },

    /// Node management (list, info, kill)
    #[command(visible_alias = "n")]
    Node {
        #[command(subcommand)]
        command: NodeCommands,
    },

    /// Parameter management (get, set, list, delete)
    #[command(visible_alias = "p")]
    Param {
        #[command(subcommand)]
        command: ParamCommands,
    },

    /// Coordinate frame operations (list, echo, tree)
    #[command(name = "frame", visible_alias = "frames", alias = "tf")]
    Frame {
        #[command(subcommand)]
        command: TfCommands,
    },

    /// Service interaction (list, call, info, find)
    #[command(visible_alias = "srv")]
    Service {
        #[command(subcommand)]
        command: ServiceCommands,
    },

    /// Action interaction (list, info, send-goal, cancel-goal)
    #[command(visible_alias = "a")]
    Action {
        #[command(subcommand)]
        command: ActionCommands,
    },

    /// Message type introspection
    #[command(visible_alias = "m")]
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
        #[arg(short = 'f', long = "follow", conflicts_with_all = ["clear", "clear_all"])]
        follow: bool,

        /// Number of recent log entries to show
        #[arg(short = 'n', long = "count")]
        count: Option<usize>,

        /// Clear logs instead of viewing
        #[arg(long = "clear", conflicts_with = "clear_all")]
        clear: bool,

        /// Clear all logs (including file-based logs)
        #[arg(long = "clear-all")]
        clear_all: bool,
    },

    /// Inspect the BlackBox flight recorder (post-mortem crash analysis)
    #[command(visible_alias = "bb")]
    Blackbox {
        /// Show only anomalies (errors, deadline misses, budget violations, e-stops)
        #[arg(short = 'a', long = "anomalies")]
        anomalies: bool,

        /// Follow mode — stream new events as they arrive (like tail -f)
        #[arg(short = 'f', long = "follow")]
        follow: bool,

        /// Filter by tick range (e.g. "4500-4510" or single tick "4500")
        #[arg(short = 't', long = "tick")]
        tick: Option<String>,

        /// Filter by node name (partial, case-insensitive match)
        #[arg(short = 'n', long = "node")]
        node: Option<String>,

        /// Filter by event type (e.g. "DeadlineMiss", "NodeError")
        #[arg(short = 'e', long = "event")]
        event: Option<String>,

        /// Output as machine-readable JSON
        #[arg(long = "json")]
        json: bool,

        /// Show only the last N events
        #[arg(short = 'l', long = "last")]
        last: Option<usize>,

        /// Custom blackbox directory (default: .horus/blackbox/)
        #[arg(short = 'p', long = "path")]
        path: Option<PathBuf>,

        /// Clear all blackbox data (with confirmation)
        #[arg(long = "clear")]
        clear: bool,
    },

    /// Monitor running HORUS nodes, topics, and system health
    #[command(visible_alias = "mon")]
    Monitor {
        /// Port for web interface (default: 3000)
        #[arg(value_name = "PORT", default_value = "3000")]
        port: u16,

        /// Use Terminal UI mode instead of web
        #[arg(short = 't', long = "tui")]
        tui: bool,

        /// Reset password before starting
        #[arg(long = "reset-password")]
        reset_password: bool,

        /// Disable authentication (no password required).
        ///
        /// WARNING: Anyone on your network will be able to access ALL monitoring
        /// APIs without a password.  Only use this in fully trusted environments
        /// such as isolated development machines or air-gapped robots.
        ///
        /// Without this flag, `horus monitor` refuses to start if no password
        /// has been configured.  Set a password with: horus monitor -r
        #[arg(long = "no-auth")]
        no_auth: bool,
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

    /// Discover HORUS nodes on the local network (requires 'mdns' feature)
    #[cfg(not(feature = "mdns"))]
    Discover,

    // ── Packages ─────────────────────────────────────────────────────────
    /// Install a package or plugin (use name@version for specific version)
    #[command(visible_alias = "i")]
    Install {
        /// Package name (supports name@version syntax, e.g. rplidar@1.2.0)
        name: String,
        /// Specific version (prefer name@version syntax instead)
        #[arg(long = "ver", hide = true)]
        ver: Option<String>,
        /// Install to global scope (~/.horus/cache/)
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// Target workspace/project name
        #[arg(short = 't', long = "target")]
        target: Option<String>,
        /// Install as driver (adds to horus.toml drivers section)
        #[arg(long = "driver", conflicts_with = "plugin")]
        driver: bool,
        /// Install as CLI plugin (defaults to global scope)
        #[arg(long = "plugin", conflicts_with = "driver")]
        plugin: bool,
    },

    /// Remove a package, driver, or plugin
    Remove {
        /// Package/driver/plugin name to remove
        name: String,
        /// Remove from global scope (~/.horus/cache/)
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// Also clean unused packages from cache after removal
        #[arg(long = "purge")]
        purge: bool,
    },

    /// Search available packages and plugins from registry
    #[command(visible_alias = "s")]
    Search {
        /// Search query (e.g., "camera", "lidar", "motor")
        query: String,
        /// Filter by category (camera, lidar, imu, motor, servo, bus, gps, simulation, cli)
        #[arg(short = 'c', long = "category")]
        category: Option<String>,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// List installed packages and plugins
    List {
        /// List global scope packages only
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// List all (local + global)
        #[arg(short = 'a', long = "all")]
        all: bool,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Update installed packages to latest versions
    Update {
        /// Specific package to update (updates all if omitted)
        package: Option<String>,
        /// Update global scope packages
        #[arg(short = 'g', long = "global")]
        global: bool,
        /// Show what would be updated without making changes
        #[arg(long = "dry-run")]
        dry_run: bool,
    },

    /// Show detailed info about a package or plugin
    Info {
        /// Package or plugin name
        name: String,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    // ── Plugins ──────────────────────────────────────────────────────────
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
        #[arg(long = "reason")]
        reason: Option<String>,
    },

    /// Verify integrity of installed plugins
    Verify {
        /// Specific plugin to verify (optional, verifies all if not specified)
        plugin: Option<String>,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    // ── Publishing & Deploy ──────────────────────────────────────────────
    /// Publish package to registry
    Publish {
        /// Also generate freeze file
        #[arg(long)]
        freeze: bool,
        /// Validate package without actually publishing
        #[arg(long = "dry-run")]
        dry_run: bool,
    },

    /// Unpublish a package from the registry (use name@version syntax)
    Unpublish {
        /// Package name (supports name@version syntax)
        package: String,
        /// Package version to unpublish (prefer name@version syntax instead)
        #[arg(value_name = "VERSION", hide = true)]
        ver: Option<String>,
        /// Skip confirmation prompt
        #[arg(short = 'y', long = "yes")]
        yes: bool,
    },

    /// Generate signing key pair for package signing
    #[command(name = "keygen")]
    KeyGen,

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
        #[arg(long = "run")]
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
        #[arg(long = "list", conflicts_with_all = ["run_after", "arch", "remote_dir", "dry_run"])]
        list: bool,
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

    /// Cache management (info, clean, purge)
    Cache {
        #[command(subcommand)]
        command: CacheCommands,
    },

    /// Record/replay management for debugging and testing
    #[command(visible_alias = "rec")]
    Record {
        #[command(subcommand)]
        command: RecordCommands,
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
enum CacheCommands {
    /// Show cache information (size, packages, disk usage)
    Info {
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

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
    List {
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },
}

#[derive(Subcommand)]
enum EnvCommands {
    /// List all published environments
    List {
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Show details of an environment
    Info {
        /// Environment ID (e.g., horus-env-abc123)
        id: String,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
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
enum RecordCommands {
    /// List all recording sessions
    List {
        /// Show detailed info (file sizes, tick counts)
        #[arg(short = 'l', long = "long")]
        long: bool,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Show details of a specific recording session
    Info {
        /// Session name
        session: String,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
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

    /// Measure topic bandwidth (bytes/sec)
    Bw {
        /// Topic name
        name: String,

        /// Window size for averaging (default: 100)
        #[arg(short = 'w', long = "window")]
        window: Option<usize>,
    },
}

#[derive(Subcommand)]
enum TfCommands {
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

        /// Print one result and exit
        #[arg(long = "once", conflicts_with = "count")]
        once: bool,

        /// Exit after N seconds
        #[arg(short = 't', long = "timeout")]
        timeout: Option<f64>,
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
        #[arg(value_name = "PATH")]
        file: Option<std::path::PathBuf>,
    },

    /// Dump all parameters as YAML to stdout
    Dump,
}

#[derive(Subcommand)]
enum ActionCommands {
    /// List all active actions
    List {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Show detailed info about an action
    Info {
        /// Action name
        name: String,
    },

    /// Send a goal to an action server
    #[command(name = "send-goal")]
    SendGoal {
        /// Action name
        name: String,

        /// Goal as JSON
        goal: String,

        /// Wait for and display the result
        #[arg(short = 'w', long = "wait")]
        wait: bool,

        /// Timeout in seconds when waiting for result (default: 30.0)
        #[arg(short = 't', long = "timeout", default_value = "30.0")]
        timeout: f64,
    },

    /// Cancel a goal on an action server
    #[command(name = "cancel-goal")]
    CancelGoal {
        /// Action name
        name: String,

        /// Goal ID to cancel (optional — cancels all if omitted)
        #[arg(short = 'i', long = "goal-id")]
        goal_id: Option<String>,
    },
}

#[derive(Subcommand)]
enum ServiceCommands {
    /// List all active services
    List {
        /// Show detailed information
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Call a service
    Call {
        /// Service name
        name: String,

        /// Request as JSON (e.g. '{"a": 3, "b": 4}')
        request: String,

        /// Timeout in seconds (default: 5.0)
        #[arg(short = 't', long = "timeout", default_value = "5.0")]
        timeout: f64,
    },

    /// Show type info for a service
    Info {
        /// Service name
        name: String,
    },

    /// Find services matching a name filter
    Find {
        /// Name filter (substring match)
        filter: String,
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

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Show message type definition
    Info {
        /// Message type name
        name: String,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Show message type definition hash
    Hash {
        /// Message type name
        name: String,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
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

fn main() {
    // First, try to handle as a plugin command before clap parsing
    // This allows plugins to be invoked as: `horus <plugin-name> [args...]`
    let args: Vec<String> = std::env::args().collect();

    // If there's at least one argument (besides program name) and it's not a built-in command
    if args.len() >= 2 {
        let potential_command = &args[1];

        // Skip if it's a built-in command, help flag, or version flag
        // Dynamically derived from clap so new commands are automatically included
        let cmd = Cli::command();
        let builtin_names: std::collections::HashSet<&str> = cmd
            .get_subcommands()
            .flat_map(|sc| {
                let mut names = vec![sc.get_name()];
                names.extend(sc.get_all_aliases());
                names
            })
            .collect();

        let is_builtin = builtin_names.contains(potential_command.as_str())
            || potential_command.starts_with('-');

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

    // Initialize the HORUS log bridge.
    //
    // This replaces env_logger and forwards all `log::` calls (from internal
    // subsystems like actions, mDNS, scheduler, blackbox) to GLOBAL_LOG_BUFFER
    // so they appear in `horus monitor --tui` and the web monitor, in addition
    // to being mirrored to stderr for console visibility.
    let log_level = if cli.verbose {
        "debug"
    } else if cli.quiet {
        "error"
    } else {
        "warn"
    };
    horus_core::core::log_bridge::try_init_log_bridge(log_level);

    // Propagate --quiet to the progress/cli_output quiet mode
    if cli.quiet {
        horus_manager::progress::set_quiet(true);
    }

    log::debug!("HORUS CLI v{}", env!("CARGO_PKG_VERSION"));

    // Silently clean up stale SHM namespace directories from crashed processes.
    // Cost: one read_dir + one kill() per stale dir (<1ms).
    let _ = horus_core::memory::cleanup_stale_namespaces();

    if let Err(e) = run_command(cli.command) {
        eprintln!("{} {}", "Error:".red().bold(), e);
        if let Some(hint) = e.help() {
            eprintln!("  {} {}", "hint:".yellow().bold(), hint);
        }
        // Propagate the child process exit code if available, otherwise default to 1.
        // Error messages from run_rust carry patterns like:
        //   "Process exited with code 42"
        //   "One or more processes failed (worst exit code: 42)"
        let exit_code = extract_exit_code(&e.to_string()).unwrap_or(1);
        std::process::exit(exit_code);
    }
}

/// Extract a process exit code from an error message.
///
/// Recognises two patterns produced by the run subsystem:
///   - "Process exited with code <N>"         (single-file mode)
///   - "worst exit code: <N>"                 (multi-file mode)
fn extract_exit_code(msg: &str) -> Option<i32> {
    // Single-file: "Process exited with code 42"
    if let Some(pos) = msg.find("Process exited with code ") {
        let after = &msg[pos + "Process exited with code ".len()..];
        if let Some(num_str) = after.split(|c: char| !c.is_ascii_digit() && c != '-').next() {
            if let Ok(code) = num_str.parse::<i32>() {
                return Some(code);
            }
        }
    }
    // Multi-file: "worst exit code: 42)"
    if let Some(pos) = msg.find("worst exit code: ") {
        let after = &msg[pos + "worst exit code: ".len()..];
        if let Some(num_str) = after.split(|c: char| !c.is_ascii_digit() && c != '-').next() {
            if let Ok(code) = num_str.parse::<i32>() {
                return Some(code);
            }
        }
    }
    None
}

fn run_command(command: Commands) -> HorusResult<()> {
    match command {
        Commands::Init { name } => commands::init::run_init(name).map_err(HorusError::from),

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
                .map_err(HorusError::from)
        }

        Commands::Run {
            files,
            release,
            clean,
            drivers,
            enable,
            args,
            record,
        } => {
            // SAFETY: These set_var calls run in single-threaded main() before
            // any child processes or threads are spawned. They configure env vars
            // that child processes inherit via process environment.
            if let Some(ref driver_list) = drivers {
                std::env::set_var("HORUS_DRIVERS", driver_list.join(","));
            }
            if let Some(ref enable_list) = enable {
                std::env::set_var("HORUS_ENABLE", enable_list.join(","));
            }
            if let Some(ref session_name) = record {
                std::env::set_var("HORUS_RECORD_SESSION", session_name);
                println!(
                    "{} Recording enabled: session '{}'",
                    horus_manager::cli_output::ICON_INFO.yellow().bold(),
                    session_name
                );
            }

            // Build and run
            commands::run::execute_run(files, args, release, clean).map_err(HorusError::from)
        }

        Commands::Build {
            files,
            release,
            clean,
            drivers,
            enable,
        } => {
            // SAFETY: Single-threaded main() context, before any child processes.
            if let Some(ref driver_list) = drivers {
                std::env::set_var("HORUS_DRIVERS", driver_list.join(","));
            }
            if let Some(ref enable_list) = enable {
                std::env::set_var("HORUS_ENABLE", enable_list.join(","));
            }

            // Build only - compile but don't execute
            commands::run::execute_build_only(files, release, clean).map_err(HorusError::from)
        }

        Commands::Check { path, json } => {
            commands::check::run_check(path, horus_manager::progress::is_quiet(), json)
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
            no_cleanup: _,
            verbose,
            drivers,
            enable,
        } => {
            // SAFETY: Single-threaded main() context, before any child processes.
            if let Some(ref driver_list) = drivers {
                std::env::set_var("HORUS_DRIVERS", driver_list.join(","));
            }
            if let Some(ref enable_list) = enable {
                std::env::set_var("HORUS_ENABLE", enable_list.join(","));
            }

            commands::test::run_tests(commands::test::TestConfig {
                filter,
                release,
                nocapture,
                test_threads,
                parallel,
                simulation,
                integration,
                no_build,
                verbose,
            })
            .map_err(HorusError::from)?;
            Ok(())
        }

        Commands::Monitor {
            port,
            tui,
            reset_password,
            no_auth,
        } => {
            // Reset password if requested
            if reset_password {
                security::auth::reset_password().map_err(HorusError::from)?;
            }

            if tui {
                println!("{} Opening HORUS monitor (TUI)...", "".cyan());
                // Launch TUI monitor
                monitor_tui::TuiDashboard::run().map_err(HorusError::from)
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
                    .map_err(|e| HorusError::Config(ConfigError::Other(format!("Failed to create async runtime: {}", e))))?
                    .block_on(monitor::run(port, no_auth))
                    .map_err(|e| {
                        let err_str = e.to_string();
                        if err_str.contains("Address already in use") || err_str.contains("os error 98") {
                            HorusError::Config(ConfigError::Other(format!(
                                "Port {} is already in use.\n  {} Try a different port: horus monitor <PORT>\n  {} Example: horus monitor {}",
                                port,
                                "".cyan(),
                                "".cyan(),
                                port + 1
                            )))
                        } else {
                            HorusError::Config(ConfigError::Other(err_str))
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
            TopicCommands::Bw { name, window } => commands::topic::topic_bw(&name, window),
        },

        Commands::Action { command } => match command {
            ActionCommands::List { verbose, json } => commands::action::list_actions(verbose, json),
            ActionCommands::Info { name } => commands::action::action_info(&name),
            ActionCommands::SendGoal {
                name,
                goal,
                wait,
                timeout,
            } => commands::action::send_goal(&name, &goal, wait, timeout),
            ActionCommands::CancelGoal { name, goal_id } => {
                commands::action::cancel_goal(&name, goal_id.as_deref())
            }
        },

        Commands::Service { command } => match command {
            ServiceCommands::List { verbose, json } => {
                commands::service::list_services(verbose, json)
            }
            ServiceCommands::Call {
                name,
                request,
                timeout,
            } => commands::service::call_service(&name, &request, timeout),
            ServiceCommands::Info { name } => commands::service::service_type(&name),
            ServiceCommands::Find { filter } => commands::service::find_services(&filter),
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

        Commands::Frame { command } => match command {
            TfCommands::List { verbose, json } => commands::tf::list_frames(verbose, json),
            TfCommands::Echo {
                source,
                target,
                rate,
                count,
                once,
                timeout,
            } => {
                let effective_count = if once { Some(1) } else { count };
                commands::tf::echo_transform(&source, &target, rate, effective_count, timeout)
            }
            TfCommands::Tree { output } => commands::tf::view_frames(output.as_deref()),
            TfCommands::Info { name } => commands::tf::frame_info(&name),
            TfCommands::Can { source, target } => commands::tf::can_transform(&source, &target),
            TfCommands::Hz { window } => commands::tf::monitor_rates(window),
        },

        #[cfg(feature = "mdns")]
        Commands::Discover {
            timeout,
            topic,
            name,
            watch,
            format,
        } => commands::discover::run_discover(timeout, topic, name, watch, &format),

        #[cfg(not(feature = "mdns"))]
        Commands::Discover => {
            eprintln!(
                "{} The 'discover' command requires the 'mdns' feature.",
                "Note:".yellow().bold()
            );
            eprintln!(
                "  Reinstall with: {}",
                "cargo install horus_manager --features mdns".cyan()
            );
            Err(HorusError::Config(horus_core::error::ConfigError::Other(
                "The 'discover' command requires the 'mdns' feature".to_string(),
            )))
        }

        Commands::Clean {
            shm,
            all,
            dry_run,
            force,
            json,
        } => commands::clean::run_clean(shm, all, dry_run, force, json),

        Commands::Launch {
            file,
            dry_run,
            namespace,
            list,
            shutdown_timeout,
        } => {
            if list {
                commands::launch::list_launch_nodes(&file)
            } else {
                commands::launch::run_launch(&file, dry_run, namespace, shutdown_timeout)
            }
        }

        Commands::Msg { command } => match command {
            MsgCommands::List {
                verbose,
                filter,
                json,
            } => commands::msg::list_messages(verbose, filter.as_deref(), json),
            MsgCommands::Info { name, json } => commands::msg::show_message(&name, json),
            MsgCommands::Hash { name, json } => commands::msg::message_hash(&name, json),
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

        // ── New top-level commands (replace horus pkg / horus plugin) ──
        Commands::Install {
            name,
            ver,
            global,
            target,
            driver,
            plugin,
        } => {
            // Parse name@version syntax (takes precedence over --ver)
            let (pkg_name, pkg_ver) = match name.find('@') {
                Some(idx) => (name[..idx].to_string(), Some(name[idx + 1..].to_string())),
                None => (name, ver),
            };

            if plugin {
                // Plugins default to global scope; --global is a no-op, only explicit --target overrides
                let local = target.is_some();
                commands::plugin::run_install(pkg_name, pkg_ver, local)
            } else if driver {
                commands::pkg::run_add(pkg_name, pkg_ver, true, false)
            } else {
                commands::pkg::run_install(pkg_name, pkg_ver, global, target)
            }
        }

        Commands::List { global, all, json } => commands::pkg::run_list(None, global, all, json),

        Commands::Search {
            query,
            category,
            json,
        } => commands::plugin::run_search_with_category(query, category, json),

        Commands::Update {
            package,
            global,
            dry_run,
        } => commands::pkg::run_update(package, global, dry_run),

        Commands::Publish { freeze, dry_run } => commands::pkg::run_publish(freeze, dry_run),

        Commands::Unpublish { package, ver, yes } => {
            // Parse name@version syntax (takes precedence over positional version)
            let (pkg_name, pkg_ver) = match package.find('@') {
                Some(idx) => {
                    let (name, rest) = package.split_at(idx);
                    (name.to_string(), rest[1..].to_string())
                }
                None => match ver {
                    Some(v) => (package, v),
                    None => {
                        return Err(HorusError::Config(ConfigError::Other(
                            "Version required. Use: horus unpublish name@version".to_string(),
                        )));
                    }
                },
            };
            commands::pkg::run_unpublish(pkg_name, pkg_ver, yes)
        }

        Commands::KeyGen => commands::pkg::run_keygen(),

        Commands::Info { name, json } => commands::plugin::run_info_unified(name, json),

        Commands::Enable { command } => {
            commands::pkg::enable_plugin(&command).map_err(HorusError::from)
        }

        Commands::Disable { command, reason } => {
            commands::pkg::disable_plugin(&command, reason.as_deref()).map_err(HorusError::from)
        }

        Commands::Verify { plugin, json } => {
            commands::pkg::verify_plugins(plugin.as_deref(), json).map_err(HorusError::from)
        }

        Commands::Env { command } => match command {
            EnvCommands::Freeze { output, publish } => commands::env::run_freeze(output, publish),
            EnvCommands::Restore { source } => commands::env::run_restore(source),
            EnvCommands::List { json } => commands::env::run_list(json),
            EnvCommands::Info { id, json } => commands::env::run_show(id, json),
        },

        Commands::Auth { command } => match command {
            AuthCommands::Login => commands::github_auth::login(),
            AuthCommands::GenerateKey { name, environment } => {
                commands::github_auth::generate_key(name, environment)
            }
            AuthCommands::Logout => commands::github_auth::logout(),
            AuthCommands::Whoami => commands::github_auth::whoami(),
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
                commands::deploy::run_deploy(commands::deploy::DeployArgs {
                    target,
                    remote_dir,
                    arch,
                    run_after,
                    release: !debug,
                    port,
                    identity,
                    dry_run,
                })
            } else {
                Err(HorusError::Config(ConfigError::Other(
                    "Target is required for deploy".to_string(),
                )))
            }
        }

        Commands::Remove {
            name,
            global,
            purge,
        } => {
            if global {
                // Remove from global scope (files + plugin unregistration)
                commands::plugin::run_remove(name.clone(), true)?;
                if purge {
                    commands::cache::run_clean(false)?;
                }
                Ok(())
            } else {
                // Remove from local horus.toml + optionally clean cache
                commands::pkg::run_remove_dep(name)?;
                if purge {
                    commands::cache::run_clean(false)?;
                }
                Ok(())
            }
        }

        Commands::Cache { command } => match command {
            CacheCommands::Info { json } => commands::cache::run_info(json),
            CacheCommands::List { json } => commands::cache::run_list(json),
            CacheCommands::Clean { dry_run } => commands::cache::run_clean(dry_run),
            CacheCommands::Purge { yes } => commands::cache::run_purge(yes),
        },

        Commands::Record { command } => match command {
            RecordCommands::List { long, json } => commands::record::run_list(long, json),
            RecordCommands::Info { session, json } => commands::record::run_info(session, json),
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
            } => commands::record::run_inject(commands::record::InjectArgs {
                session,
                nodes,
                all,
                script,
                start_tick,
                stop_tick,
                speed,
                loop_playback,
            }),
        },

        Commands::Blackbox {
            anomalies,
            follow,
            tick,
            node,
            event,
            json,
            last,
            path,
            clear,
        } => commands::blackbox::run_blackbox(commands::blackbox::BlackboxArgs {
            anomalies_only: anomalies,
            tail: follow,
            tick_range: tick,
            node_filter: node,
            event_filter: event,
            json_output: json,
            last_n: last,
            custom_path: path,
            clear,
        }),

        Commands::Completion { shell } => {
            // Hidden command used by install.sh for automatic completion setup
            let mut cmd = Cli::command();
            let bin_name = cmd.get_name().to_string();
            generate(shell, &mut cmd, bin_name, &mut io::stdout());
            Ok(())
        }
    }
}
