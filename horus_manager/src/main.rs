use clap::{CommandFactory, Parser, Subcommand};
use clap_complete::generate;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::io;
use std::path::PathBuf;

// Use modules from the library instead of redeclaring them
use horus_manager::commands;
#[cfg(feature = "monitor")]
use horus_manager::{monitor, monitor_tui, security};

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
  lock              Generate or verify horus.lock (pin dependency versions)
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


Debugging:
  log               View and filter logs
  blackbox, bb      Inspect the BlackBox flight recorder
  monitor, mon      Monitor nodes, topics, and system health
  record, rec       Record/replay management
  cache             Cache management (info, clean, purge)

Dependencies:
  add               Add a dependency to horus.toml (crates.io, PyPI, system, registry)
  remove            Remove a dependency from horus.toml

Packages:
  install, i        Install a standalone package or plugin from registry
  uninstall         Uninstall a standalone package or plugin
  search, s         Search available packages from registry
  list              List installed packages and plugins
  update            Update project dependencies to latest versions
  info              Show detailed info about a package or plugin

Plugins:
  enable            Enable a disabled plugin
  disable           Disable a plugin (keep installed but don't execute)
  verify            Verify integrity of installed plugins

Development:
  fmt               Format code (Rust + Python)
  lint              Lint code (clippy + ruff/pylint)
  doc               Generate documentation
  bench             Run benchmarks
  deps              Dependency insight (tree, why, outdated, audit)

Maintenance:
  doctor            Comprehensive ecosystem health check
  self update       Update the horus CLI to latest version
  config            View/edit horus.toml settings
  migrate           Migrate project to unified horus.toml format

Publishing & Deploy:
  publish           Publish package to registry
  unpublish         Unpublish a package (name@version syntax)
  yank              Yank a version (hide from new installs)
  unyank            Restore a yanked version
  deprecate         Mark a package as deprecated
  undeprecate       Remove deprecation from a package
  owner             Manage owners and transfer ownership
  deploy            Deploy project to a remote robot
  auth              Authentication (login, api-key, signing-key)

Native Tools:
  env               Set up shell integration (cargo/pip/cmake proxy)

{options}
{after-help}")]
#[command(after_help = "\
Quick Start:
  horus new my_robot -r           Create a new Rust project
  cd my_robot && horus run        Build and run it
  horus topic list                See active topics

More examples:
  horus init                      Initialize workspace in current directory
  horus add serde --source crates-io  Add a Rust dependency
  horus install rplidar@1.2.0    Install a standalone package
  horus bb --anomalies            Show crash anomalies
  horus deploy robot@192.168.1.5  Deploy to a remote robot

Docs: https://docs.horusrobotics.dev")]
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
        #[arg(short = 'p', long = "python", conflicts_with_all = ["rust", "cpp"])]
        python: bool,
        /// Use Rust
        #[arg(short = 'r', long = "rust", conflicts_with_all = ["python", "cpp"])]
        rust: bool,
        /// Use C++
        #[arg(short = 'c', long = "cpp", conflicts_with_all = ["python", "rust"])]
        cpp: bool,
        /// Use Rust with macros
        #[arg(short = 'm', long = "macro", conflicts_with_all = ["python", "cpp"])]
        use_macro: bool,
        /// Create as a workspace with multiple crates
        #[arg(short = 'w', long = "workspace")]
        workspace: bool,
        /// Create as a library crate (instead of binary)
        #[arg(short = 'l', long = "lib")]
        lib: bool,
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

        /// Output run results as JSON (for CI/AI tooling)
        #[arg(long = "json")]
        json: bool,

        /// Output build diagnostics as JSON lines for AI agents
        #[arg(long = "json-diagnostics")]
        json_diagnostics: bool,

        /// Enable recording for this session
        /// Use 'horus record list' to see recordings
        #[arg(long = "record")]
        record: Option<String>,

        /// Additional arguments to pass to the program (use -- to separate)
        #[arg(last = true)]
        args: Vec<String>,

        /// Skip [hooks] execution
        #[arg(long = "no-hooks")]
        no_hooks: bool,

        /// Run a specific workspace member by name
        #[arg(short = 'p', long = "package")]
        package: Option<String>,
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

        /// Output build results as JSON (for CI/AI tooling)
        #[arg(long = "json")]
        json: bool,

        /// Output build diagnostics as JSON lines for AI agents
        #[arg(long = "json-diagnostics")]
        json_diagnostics: bool,

        /// Skip [hooks] execution
        #[arg(long = "no-hooks")]
        no_hooks: bool,

        /// Build a specific workspace member by name
        #[arg(short = 'p', long = "package")]
        package: Option<String>,
    },

    /// Generate or verify horus.lock (pins all dependency versions)
    Lock {
        /// Only check if lockfile is up-to-date, don't regenerate
        #[arg(long = "check")]
        check: bool,
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

        /// Output test results as JSON (for CI/AI tooling)
        #[arg(long = "json")]
        json: bool,

        /// Skip [hooks] execution
        #[arg(long = "no-hooks")]
        no_hooks: bool,
    },

    /// Validate horus.toml, source files, or entire workspace
    Check {
        /// Path to file, directory, or workspace (default: current directory)
        #[arg(value_name = "PATH")]
        path: Option<PathBuf>,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,

        /// Run full validation (manifest + doctor + fmt + lint + deps)
        #[arg(long = "full")]
        full: bool,

        /// Run health check (alias for horus doctor)
        #[arg(long = "health")]
        health: bool,
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
    #[command(name = "frame", visible_alias = "tf")]
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
    #[cfg(feature = "monitor")]
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

    /// Monitor running HORUS nodes, topics, and system health (requires 'monitor' feature)
    #[cfg(not(feature = "monitor"))]
    #[command(visible_alias = "mon")]
    Monitor,

    // ── Dependencies ────────────────────────────────────────────────────
    /// Add a dependency to horus.toml (crates.io, PyPI, system, registry, git, path)
    Add {
        /// Dependency name (supports name@version syntax, e.g. serde@1.0)
        name: String,
        /// Version requirement (alternative to name@version syntax)
        #[arg(long = "ver", hide = true)]
        ver: Option<String>,
        /// Dependency source: crates-io, pypi, system, registry, git, path
        #[arg(short = 's', long = "source")]
        source: Option<String>,
        /// Features to enable (e.g. --features derive,serde)
        #[arg(short = 'F', long = "features", value_delimiter = ',')]
        features: Option<Vec<String>>,
        /// Add to [dev-dependencies] instead of [dependencies]
        #[arg(long = "dev")]
        dev: bool,
        /// Add as driver to [drivers] section
        #[arg(long = "driver")]
        driver: bool,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Remove a dependency from horus.toml
    Remove {
        /// Dependency name
        name: String,
        /// Also clean unused packages from cache
        #[arg(long = "purge")]
        purge: bool,
    },

    // ── Packages ─────────────────────────────────────────────────────────
    /// Install a standalone package or plugin from the registry
    #[command(visible_alias = "i")]
    Install {
        /// Package name (supports name@version syntax, e.g. rplidar@1.2.0)
        name: String,
        /// Specific version (alternative to name@version syntax)
        #[arg(long = "ver", hide = true)]
        ver: Option<String>,
        /// Install as CLI plugin
        #[arg(long = "plugin")]
        plugin: bool,
        /// Target workspace/project name
        #[arg(short = 't', long = "target")]
        target: Option<String>,
        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Uninstall a standalone package or plugin
    Uninstall {
        /// Package or plugin name
        name: String,
        /// Also purge cached files
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

    /// Update project dependencies to latest versions
    Update {
        /// Specific package to update (updates all deps if omitted)
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
    /// Plugin management (enable, disable, verify)
    #[command(visible_alias = "plugins")]
    Plugin {
        #[command(subcommand)]
        command: PluginCommands,
    },

    // ── Development ──────────────────────────────────────────────────────
    /// Format code (Rust + Python)
    Fmt {
        /// Check formatting without modifying files
        #[arg(long = "check")]
        check: bool,

        /// Additional arguments passed to underlying tools
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Lint code (clippy + ruff/pylint)
    Lint {
        /// Auto-fix lint issues where possible
        #[arg(long = "fix")]
        fix: bool,

        /// Also run Python type checker (mypy/pyright)
        #[arg(long = "types")]
        types: bool,

        /// Additional arguments passed to underlying tools
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Generate documentation
    Doc {
        /// Open documentation in browser after generating
        #[arg(long = "open")]
        open: bool,

        /// Extract machine-readable API documentation
        #[arg(long = "extract")]
        extract: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,

        /// Output as markdown (for LLM context)
        #[arg(long = "md")]
        md: bool,

        /// Output as self-contained HTML report
        #[arg(long = "html")]
        html: bool,

        /// Include doc comments in brief output
        #[arg(long = "full")]
        full: bool,

        /// Include private/crate-only symbols
        #[arg(long = "all")]
        all: bool,

        /// Filter by language (rust, cpp, python)
        #[arg(long = "lang")]
        lang: Option<String>,

        /// Show documentation coverage report
        #[arg(long = "coverage")]
        coverage: bool,

        /// Write output to file instead of stdout
        #[arg(short = 'o', long = "output")]
        output: Option<std::path::PathBuf>,

        /// Compare against a baseline JSON file
        #[arg(long = "diff", value_name = "BASELINE")]
        diff: Option<std::path::PathBuf>,

        /// Fail if doc coverage is below this percentage (for CI)
        #[arg(long = "fail-under", value_name = "PERCENT")]
        fail_under: Option<u32>,

        /// Watch for file changes and regenerate
        #[arg(long = "watch")]
        watch: bool,

        /// Additional arguments passed to underlying tools
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Run benchmarks
    Bench {
        /// Filter benchmarks by name
        #[arg(value_name = "FILTER")]
        filter: Option<String>,

        /// Additional arguments passed to underlying tools
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Dependency insight (tree, why, outdated, audit)
    Deps {
        #[command(subcommand)]
        command: DepsCommands,
    },

    // ── Maintenance ─────────────────────────────────────────────────────
    /// Comprehensive ecosystem health check
    Doctor {
        /// Show detailed output for each check
        #[arg(short = 'v', long = "verbose")]
        verbose: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,

        /// Install missing toolchains and system dependencies
        #[arg(long = "fix")]
        fix: bool,
    },

    /// Manage the horus CLI itself
    #[command(name = "self")]
    Self_ {
        #[command(subcommand)]
        command: SelfCommands,
    },

    /// DEPRECATED: Use `horus self update` instead
    #[command(name = "upgrade", hide = true)]
    DeprecatedUpgrade {
        /// Check for updates without installing
        #[arg(long = "check")]
        check_only: bool,
    },

    /// DEPRECATED: Use `horus doctor --fix` instead
    #[command(name = "sync", hide = true)]
    DeprecatedSync {
        /// Check only — don't install anything
        #[arg(long = "check")]
        check: bool,
    },

    /// View/edit horus.toml settings
    Config {
        #[command(subcommand)]
        command: ConfigCommands,
    },

    /// Migrate project to unified horus.toml format
    Migrate {
        /// Show what would change without modifying
        #[arg(short = 'n', long = "dry-run")]
        dry_run: bool,

        /// Skip confirmation prompts
        #[arg(short = 'f', long = "force")]
        force: bool,
    },

    /// Run a script defined in horus.toml [scripts]
    #[command(visible_alias = "script")]
    Scripts {
        /// Script name to run (omit to list available scripts)
        name: Option<String>,

        /// Arguments to pass to the script
        #[arg(last = true)]
        args: Vec<String>,
    },

    // ── Publishing & Deploy ──────────────────────────────────────────────
    /// Publish package to registry
    Publish {
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

    /// Yank a package version (hide from new installs, existing lockfiles still work)
    Yank {
        /// Package name@version (e.g., my-pkg@0.2.1)
        package: String,
        /// Reason for yanking
        #[arg(long)]
        reason: Option<String>,
    },

    /// Unyank a previously yanked package version
    Unyank {
        /// Package name@version (e.g., my-pkg@0.2.1)
        package: String,
    },

    /// Mark a package as deprecated
    Deprecate {
        /// Package name
        package: String,
        /// Deprecation message (e.g., "moved to @horus/lidar-unified")
        #[arg(long, short = 'm')]
        message: Option<String>,
    },

    /// Remove deprecation from a package
    Undeprecate {
        /// Package name
        package: String,
    },

    /// Manage package owners and ownership transfers
    Owner {
        #[command(subcommand)]
        command: OwnerCommands,
    },

    /// DEPRECATED: Use `horus auth signing-key` instead
    #[command(name = "keygen", hide = true)]
    DeprecatedKeyGen,

    /// Deploy project to remote robot(s)
    Deploy {
        /// Target(s) — named targets from deploy.yaml or direct user@host
        targets: Vec<String>,

        /// Deploy to ALL targets in deploy.yaml
        #[arg(long = "all")]
        all: bool,

        /// Deploy to multiple targets in parallel
        #[arg(long = "parallel")]
        parallel: bool,

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
        #[arg(long = "list")]
        list: bool,
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

    // ── Native Tool Proxies ─────────────────────────────────────────────
    /// Transparent cargo proxy — delegates to real cargo with horus.toml sync
    #[command(name = "cargo", hide = true)]
    Cargo {
        /// Arguments passed to cargo
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent pip proxy — delegates to real pip with horus.toml sync
    #[command(name = "pip", hide = true)]
    Pip {
        /// Arguments passed to pip
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent cmake proxy — delegates to real cmake with horus.toml sync
    #[command(name = "cmake", hide = true)]
    Cmake {
        /// Arguments passed to cmake
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent conan proxy — delegates to real conan with horus.toml sync
    #[command(name = "conan", hide = true)]
    Conan {
        /// Arguments passed to conan
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent vcpkg proxy — delegates to real vcpkg with horus.toml sync
    #[command(name = "vcpkg", hide = true)]
    Vcpkg {
        /// Arguments passed to vcpkg
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Manage shell integration for native tool support (cargo, pip, cmake)
    Env {
        /// Write shell integration files and add to shell RC
        #[arg(long)]
        init: bool,

        /// Remove shell integration from shell RC files
        #[arg(long)]
        uninstall: bool,
    },

    /// Internal: check if cwd is a horus project (exit code only)
    #[command(name = "_is-project", hide = true)]
    IsProject,
}

#[derive(Subcommand)]
enum PluginCommands {
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
enum AuthCommands {
    /// Login to HORUS registry (requires GitHub)
    Login,
    /// Generate API key for registry authentication
    #[command(name = "api-key")]
    ApiKey {
        /// Name for the API key
        #[arg(long)]
        name: Option<String>,
        /// Environment (e.g., 'laptop', 'ci-cd')
        #[arg(long)]
        environment: Option<String>,
    },
    /// Generate ed25519 signing key pair for package signing
    #[command(name = "signing-key")]
    SigningKey,
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
enum OwnerCommands {
    /// List owners of a package
    List {
        /// Package name
        package: String,
    },
    /// Add an owner to a package
    Add {
        /// Package name
        package: String,
        /// Username or user ID to add
        user: String,
    },
    /// Remove an owner from a package
    Remove {
        /// Package name
        package: String,
        /// Username or user ID to remove
        user: String,
    },
    /// Transfer package ownership to a user or organization
    Transfer {
        /// Package name
        package: String,
        /// Target username or org name
        target: String,
        /// Transfer to an organization instead of a user
        #[arg(long)]
        org: bool,
    },
    /// List pending incoming ownership transfers
    Pending,
    /// Accept a pending ownership transfer
    Accept {
        /// Transfer ID
        id: String,
    },
    /// Reject a pending ownership transfer
    Reject {
        /// Transfer ID
        id: String,
    },
}

#[derive(Subcommand)]
enum SelfCommands {
    /// Update the horus CLI to the latest version
    Update {
        /// Only check for updates, don't install
        #[arg(long = "check")]
        check_only: bool,
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

    /// Record transforms to a .tfr file
    Record {
        /// Output file path
        #[arg(short = 'o', long = "output")]
        output: String,

        /// Maximum recording duration in seconds
        #[arg(short = 'd', long = "duration")]
        duration: Option<f64>,
    },

    /// Replay a .tfr recording
    Play {
        /// Path to .tfr file
        path: String,

        /// Playback speed multiplier (default: 1.0)
        #[arg(short = 's', long = "speed", default_value = "1.0")]
        speed: f64,
    },

    /// Compare two .tfr recordings
    Diff {
        /// First recording file
        file1: String,

        /// Second recording file
        file2: String,

        /// Translation difference threshold in meters (default: 0.001)
        #[arg(long = "threshold-m", default_value = "0.001")]
        threshold_m: f64,

        /// Rotation difference threshold in degrees (default: 0.1)
        #[arg(long = "threshold-deg", default_value = "0.1")]
        threshold_deg: f64,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Interactively tune a static frame's offset
    Tune {
        /// Frame name to tune
        frame: String,

        /// Translation step size in meters (default: 0.001)
        #[arg(long = "step-m", default_value = "0.001")]
        step_m: f64,

        /// Rotation step size in degrees (default: 0.1)
        #[arg(long = "step-deg", default_value = "0.1")]
        step_deg: f64,
    },

    /// Compute sensor-to-base transform from point pairs (SVD registration)
    Calibrate {
        /// CSV file with point pairs (sensor_x,sensor_y,sensor_z,world_x,world_y,world_z)
        #[arg(long = "points-file")]
        points_file: String,
    },

    /// Solve hand-eye calibration (AX=XB) from pose pairs
    HandEye {
        /// CSV file with robot poses
        #[arg(long = "robot-poses")]
        robot_poses: String,

        /// CSV file with sensor poses
        #[arg(long = "sensor-poses")]
        sensor_poses: String,
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

#[derive(Subcommand)]
enum DepsCommands {
    /// Show dependency tree
    Tree {
        /// Additional arguments
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Explain why a dependency is included
    Why {
        /// Package name to trace
        package: String,

        /// Additional arguments
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Check for outdated dependencies
    Outdated {
        /// Additional arguments
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Security audit of dependencies
    Audit {
        /// Additional arguments
        #[arg(last = true)]
        extra_args: Vec<String>,
    },
}

#[derive(Subcommand)]
enum ConfigCommands {
    /// Get a config value
    Get {
        /// Config key (dot-notation, e.g. package.name)
        key: String,
    },

    /// Set a config value
    Set {
        /// Config key (dot-notation, e.g. package.name)
        key: String,

        /// Value to set
        value: String,
    },

    /// List all config values
    List,
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

/// Set up HORUS runtime environment variables before spawning child processes.
///
/// Called from both Run and Build commands to avoid duplicate env setup blocks.
fn setup_horus_env(
    drivers: &Option<Vec<String>>,
    enable: &Option<Vec<String>>,
    record: &Option<String>,
) {
    // SAFETY: These set_var calls run in single-threaded main() before
    // any child processes or threads are spawned.
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
}

/// Parse "name@version" syntax, returning (name, version). Errors if no version.
fn parse_name_version(input: &str, action: &str) -> Result<(String, String), HorusError> {
    match input.find('@') {
        Some(idx) => {
            let (name, rest) = input.split_at(idx);
            Ok((name.to_string(), rest[1..].to_string()))
        }
        None => Err(HorusError::Config(ConfigError::Other(format!(
            "Version required. Use: horus {} name@version",
            action
        )))),
    }
}

/// Split "name@version" syntax. If no `@`, returns (name, fallback).
fn split_name_version(input: String, fallback: Option<String>) -> (String, Option<String>) {
    match input.find('@') {
        Some(idx) => (input[..idx].to_string(), Some(input[idx + 1..].to_string())),
        None => (input, fallback),
    }
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
    // subsystems like actions, scheduler, blackbox) to GLOBAL_LOG_BUFFER
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
        if let Some(num_str) = after
            .split(|c: char| !c.is_ascii_digit() && c != '-')
            .next()
        {
            if let Ok(code) = num_str.parse::<i32>() {
                return Some(code);
            }
        }
    }
    // Multi-file: "worst exit code: 42)"
    if let Some(pos) = msg.find("worst exit code: ") {
        let after = &msg[pos + "worst exit code: ".len()..];
        if let Some(num_str) = after
            .split(|c: char| !c.is_ascii_digit() && c != '-')
            .next()
        {
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
            cpp,
            use_macro,
            workspace,
            lib,
        } => {
            let language = if python {
                "python"
            } else if cpp {
                "cpp"
            } else if rust || use_macro {
                "rust"
            } else if workspace {
                "rust" // Default workspace language
            } else {
                "" // Will use interactive prompt
            };

            commands::new::create_new_project(name, path, language.to_string(), use_macro, workspace, lib)
                .map_err(HorusError::from)
        }

        Commands::Run {
            files,
            release,
            clean,
            drivers,
            enable,
            json,
            json_diagnostics,
            args,
            record,
            no_hooks,
            package,
        } => {
            // Enable JSON diagnostics mode globally (read by error_wrapper::emit_diagnostic)
            if json_diagnostics {
                horus_manager::error_wrapper::set_json_diagnostics(true);
            }

            setup_horus_env(&drivers, &enable, &record);

            if !no_hooks {
                if let Ok(manifest) = horus_manager::manifest::HorusManifest::load_from(
                    std::path::Path::new("horus.toml"),
                ) {
                    if let Err(e) = commands::hooks::run_hooks("pre_run", &manifest) {
                        eprintln!("Hook failed: {}", e);
                        return Err(HorusError::from(e));
                    }
                }
            }

            // Build and run
            let result = commands::run::execute_run(files, args, release, clean, package);
            if json {
                match &result {
                    Ok(()) => {
                        println!("{}", serde_json::json!({"success": true, "command": "run"}))
                    }
                    Err(e) => println!(
                        "{}",
                        serde_json::json!({"success": false, "command": "run", "errors": [{"message": e.to_string()}]})
                    ),
                }
            }
            result.map_err(HorusError::from)
        }

        Commands::Build {
            files,
            release,
            clean,
            drivers,
            enable,
            json,
            json_diagnostics,
            no_hooks,
            package,
        } => {
            // Enable JSON diagnostics mode globally
            if json_diagnostics {
                horus_manager::error_wrapper::set_json_diagnostics(true);
            }

            setup_horus_env(&drivers, &enable, &None);

            if !no_hooks {
                if let Ok(manifest) = horus_manager::manifest::HorusManifest::load_from(
                    std::path::Path::new("horus.toml"),
                ) {
                    if let Err(e) = commands::hooks::run_hooks("pre_build", &manifest) {
                        eprintln!("Hook failed: {}", e);
                        return Err(HorusError::from(e));
                    }
                }
            }

            // Build only - compile but don't execute
            let result = commands::run::execute_build_only(files, release, clean, package);
            if json {
                match &result {
                    Ok(()) => {
                        println!(
                            "{}",
                            serde_json::json!({
                                "success": true,
                                "command": "build",
                            })
                        );
                    }
                    Err(e) => {
                        println!(
                            "{}",
                            serde_json::json!({
                                "success": false,
                                "command": "build",
                                "errors": [{
                                    "message": e.to_string(),
                                }]
                            })
                        );
                    }
                }
                result.map_err(HorusError::from)
            } else {
                result.map_err(HorusError::from)
            }
        }

        Commands::Lock { check } => {
            use horus_manager::lockfile::{HorusLockfile, HORUS_LOCK};
            let lock_path = std::path::Path::new(HORUS_LOCK);

            if check {
                // Verify lockfile exists and is parseable
                if !lock_path.exists() {
                    println!(
                        "{} No horus.lock found. Run `horus lock` to generate one.",
                        "[!]".yellow()
                    );
                    return Err(HorusError::Config(ConfigError::Other(
                        "No lockfile found".to_string(),
                    )));
                }
                match HorusLockfile::load_from(lock_path) {
                    Ok(lf) => {
                        println!(
                            "{} horus.lock v{} is valid ({} packages, {} system deps)",
                            "[✓]".green(),
                            lf.version,
                            lf.packages.len(),
                            lf.system_deps.len(),
                        );
                        // Run verification
                        let result = horus_manager::system_deps::verify_lockfile(&lf);
                        let report = horus_manager::system_deps::format_lockfile_report(&result);
                        if !report.is_empty() {
                            print!("{}", report);
                        }
                        Ok(())
                    }
                    Err(e) => {
                        println!("{} Failed to parse horus.lock: {}", "[✗]".red(), e);
                        Err(HorusError::Config(ConfigError::Other(e.to_string())))
                    }
                }
            } else {
                // Generate/regenerate lockfile
                let mut lockfile = if lock_path.exists() {
                    HorusLockfile::load_from(lock_path).unwrap_or_else(|_| HorusLockfile::new())
                } else {
                    HorusLockfile::new()
                };

                // Pin current toolchain versions
                lockfile.toolchain = Some(horus_manager::lockfile::ToolchainPins {
                    rust: horus_manager::registry::helpers::get_rust_version(),
                    python: horus_manager::registry::helpers::get_python_version(),
                    cmake: None,
                });

                lockfile
                    .save_to(lock_path)
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
                println!(
                    "{} Generated horus.lock v{} ({} packages)",
                    "[✓]".green(),
                    lockfile.version,
                    lockfile.packages.len(),
                );
                Ok(())
            }
        }

        Commands::Check {
            path,
            json,
            full,
            health,
        } => {
            if health {
                return commands::doctor::run_doctor(false, json, false).map_err(HorusError::from);
            }
            let has_manifest = path
                .as_ref()
                .map(|p| p.join("horus.toml").exists())
                .unwrap_or_else(|| std::path::Path::new("horus.toml").exists());
            if !has_manifest && path.is_none() {
                return commands::doctor::run_doctor(false, json, false).map_err(HorusError::from);
            }
            if full {
                commands::check::run_check_full(path, json)
            } else {
                commands::check::run_check(path, horus_manager::progress::is_quiet(), json)
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
            verbose,
            drivers,
            enable,
            json,
            no_hooks,
        } => {
            setup_horus_env(&drivers, &enable, &None);

            let hooks_manifest = if !no_hooks {
                horus_manager::manifest::HorusManifest::load_from(std::path::Path::new(
                    "horus.toml",
                ))
                .ok()
            } else {
                None
            };

            if let Some(ref manifest) = hooks_manifest {
                if let Err(e) = commands::hooks::run_hooks("pre_test", manifest) {
                    eprintln!("Hook failed: {}", e);
                    return Err(HorusError::from(e));
                }
            }

            let result = commands::test::run_tests(commands::test::TestConfig {
                filter,
                release,
                nocapture,
                test_threads,
                parallel,
                simulation,
                integration,
                no_build,
                verbose,
            });
            if json {
                match &result {
                    Ok(()) => println!(
                        "{}",
                        serde_json::json!({"success": true, "command": "test"})
                    ),
                    Err(e) => println!(
                        "{}",
                        serde_json::json!({"success": false, "command": "test", "errors": [{"message": e.to_string()}]})
                    ),
                }
            }
            result.map_err(HorusError::from)?;

            if let Some(ref manifest) = hooks_manifest {
                if let Err(e) = commands::hooks::run_hooks("post_test", manifest) {
                    eprintln!("Hook failed: {}", e);
                    return Err(HorusError::from(e));
                }
            }

            Ok(())
        }

        #[cfg(feature = "monitor")]
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

        #[cfg(not(feature = "monitor"))]
        Commands::Monitor => {
            eprintln!(
                "{} The 'monitor' command requires the 'monitor' feature (web dashboard + TUI).",
                "Note:".yellow().bold()
            );
            eprintln!(
                "  Reinstall with: {}",
                "cargo install horus_manager --features monitor".cyan()
            );
            eprintln!(
                "  Or build from source: {}",
                "cargo build --features monitor".cyan()
            );
            Err(HorusError::Config(ConfigError::Other(
                "Feature 'monitor' not enabled. Reinstall with: cargo install horus_manager --features monitor".into(),
            )))
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
            TfCommands::Record { output, duration } => {
                commands::tf::record_transforms(&output, duration)
            }
            TfCommands::Play { path, speed } => commands::tf::replay_transforms(&path, speed),
            TfCommands::Diff {
                file1,
                file2,
                threshold_m,
                threshold_deg,
                json,
            } => commands::tf::diff_transforms(&file1, &file2, threshold_m, threshold_deg, json),
            TfCommands::Tune {
                frame,
                step_m,
                step_deg,
            } => commands::tf::tune_static_frame(&frame, step_m, step_deg),
            TfCommands::Calibrate { points_file } => {
                commands::tf::calibrate_from_points(&points_file)
            }
            TfCommands::HandEye {
                robot_poses,
                sensor_poses,
            } => commands::tf::hand_eye_calibration(&robot_poses, &sensor_poses)
                .map_err(|e| horus_core::error::HorusError::Config(e)),
        },

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

        // ── Dependencies ──────────────────────────────────────────────────
        Commands::Add {
            name,
            ver,
            source,
            features,
            dev,
            driver,
            json,
        } => {
            let (pkg_name, pkg_ver) = split_name_version(name, ver);

            let result = if driver {
                commands::pkg::run_add(pkg_name.clone(), pkg_ver.clone(), true, false)
            } else {
                commands::pkg::run_add_dep(
                    pkg_name.clone(),
                    pkg_ver.clone(),
                    None,
                    source,
                    features,
                    dev,
                )
            };
            if json {
                match &result {
                    Ok(()) => println!(
                        "{}",
                        serde_json::json!({"success": true, "command": "add", "package": pkg_name, "version": pkg_ver})
                    ),
                    Err(e) => println!(
                        "{}",
                        serde_json::json!({"success": false, "command": "add", "package": pkg_name, "errors": [{"message": e.to_string()}]})
                    ),
                }
            }
            result
        }

        // ── Packages ────────────────────────────────────────────────────────
        Commands::Install {
            name,
            ver,
            plugin,
            target,
            json,
        } => {
            let (pkg_name, pkg_ver) = split_name_version(name, ver);

            let result = if plugin {
                let local = target.is_some();
                commands::plugin::run_install(pkg_name.clone(), pkg_ver.clone(), local)
            } else {
                commands::pkg::run_install_standalone(
                    &pkg_name,
                    pkg_ver.as_deref(),
                    target,
                )
            };
            if json {
                match &result {
                    Ok(()) => println!(
                        "{}",
                        serde_json::json!({"success": true, "command": "install", "package": pkg_name, "version": pkg_ver})
                    ),
                    Err(e) => println!(
                        "{}",
                        serde_json::json!({"success": false, "command": "install", "package": pkg_name, "errors": [{"message": e.to_string()}]})
                    ),
                }
            }
            result
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
        } => {
            commands::pkg::run_update(package, global, dry_run)?;
            // Check for CLI updates (non-blocking hint)
            if let Ok(Some(latest)) = commands::upgrade::check_latest_version() {
                let current = env!("CARGO_PKG_VERSION");
                if latest != current {
                    println!(
                        "\n  {} horus {} available (current: {}). Run `horus self update` to upgrade.",
                        "hint:".yellow(),
                        latest.green(),
                        current.dimmed()
                    );
                }
            }
            Ok(())
        }

        Commands::Publish { dry_run } => commands::pkg::run_publish(dry_run),

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

        Commands::Yank { package, reason } => {
            let (pkg_name, pkg_ver) = parse_name_version(&package, "yank")?;
            commands::pkg::run_yank(pkg_name, pkg_ver, reason)
        }

        Commands::Unyank { package } => {
            let (pkg_name, pkg_ver) = parse_name_version(&package, "unyank")?;
            commands::pkg::run_unyank(pkg_name, pkg_ver)
        }

        Commands::Deprecate { package, message } => commands::pkg::run_deprecate(package, message),

        Commands::Undeprecate { package } => commands::pkg::run_undeprecate(package),

        Commands::Owner { command } => match command {
            OwnerCommands::List { package } => commands::pkg::run_owner_list(package),
            OwnerCommands::Add { package, user } => commands::pkg::run_owner_add(package, user),
            OwnerCommands::Remove { package, user } => {
                commands::pkg::run_owner_remove(package, user)
            }
            OwnerCommands::Transfer {
                package,
                target,
                org,
            } => commands::pkg::run_owner_transfer(package, target, org),
            OwnerCommands::Pending => commands::pkg::run_owner_pending(),
            OwnerCommands::Accept { id } => commands::pkg::run_owner_accept(id),
            OwnerCommands::Reject { id } => commands::pkg::run_owner_reject(id),
        },

        Commands::DeprecatedKeyGen => {
            eprintln!(
                "{} `horus keygen` is deprecated. Use `horus auth signing-key` instead.",
                "WARNING:".yellow().bold()
            );
            commands::pkg::run_keygen()
        }

        Commands::Info { name, json } => commands::plugin::run_info_unified(name, json),

        Commands::Plugin { command } => match command {
            PluginCommands::Enable { command } => {
                commands::pkg::enable_plugin(&command).map_err(HorusError::from)
            }
            PluginCommands::Disable { command, reason } => {
                commands::pkg::disable_plugin(&command, reason.as_deref()).map_err(HorusError::from)
            }
            PluginCommands::Verify { plugin, json } => {
                commands::pkg::verify_plugins(plugin.as_deref(), json).map_err(HorusError::from)
            }
        },

        Commands::Auth { command } => match command {
            AuthCommands::Login => commands::github_auth::login(),
            AuthCommands::ApiKey { name, environment } => {
                commands::github_auth::generate_key(name, environment)
            }
            AuthCommands::SigningKey => commands::pkg::run_keygen(),
            AuthCommands::Logout => commands::github_auth::logout(),
            AuthCommands::Whoami => commands::github_auth::whoami(),
            AuthCommands::Keys { command: keys_cmd } => match keys_cmd {
                AuthKeysCommands::List => commands::github_auth::keys_list(),
                AuthKeysCommands::Revoke { key_id } => commands::github_auth::keys_revoke(&key_id),
            },
        },

        Commands::Deploy {
            targets,
            all,
            parallel,
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
            } else {
                commands::deploy::run_deploy_multi(commands::deploy::DeployMultiArgs {
                    targets,
                    all,
                    parallel,
                    remote_dir,
                    arch,
                    run_after,
                    release: !debug,
                    port,
                    identity,
                    dry_run,
                })
            }
        }

        Commands::Uninstall { name, purge } => {
            // Remove standalone package/plugin from global scope
            commands::plugin::run_remove(name.clone(), true)?;
            if purge {
                commands::cache::run_clean(false)?;
            }
            Ok(())
        }

        Commands::Remove { name, purge } => {
            {
                // Remove from local horus.toml
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

        // ── Development commands ────────────────────────────────────────
        Commands::Fmt { check, extra_args } => {
            commands::fmt::run_fmt(check, extra_args).map_err(HorusError::from)
        }

        Commands::Lint {
            fix,
            types,
            extra_args,
        } => commands::lint::run_lint(fix, types, extra_args).map_err(HorusError::from),

        Commands::Doc {
            open,
            extract,
            json,
            md,
            html,
            full,
            all,
            lang,
            coverage,
            output,
            diff,
            fail_under,
            watch,
            extra_args,
        } => {
            if extract || json || md || html || coverage || diff.is_some() {
                let config = commands::doc_extract::ExtractConfig {
                    json,
                    md,
                    html,
                    brief: false,
                    full,
                    all,
                    lang,
                    coverage,
                    output,
                    watch,
                    diff,
                    fail_under,
                };
                commands::doc_extract::run_extract(config).map_err(HorusError::from)
            } else {
                commands::doc::run_doc(open, extra_args).map_err(HorusError::from)
            }
        }

        Commands::Bench { filter, extra_args } => {
            commands::bench::run_bench(filter, extra_args).map_err(HorusError::from)
        }

        Commands::Deps { command } => match command {
            DepsCommands::Tree { extra_args } => {
                commands::deps::run_deps(commands::deps::DepsAction::Tree, extra_args)
                    .map_err(HorusError::from)
            }
            DepsCommands::Why {
                package,
                extra_args,
            } => commands::deps::run_deps(commands::deps::DepsAction::Why(package), extra_args)
                .map_err(HorusError::from),
            DepsCommands::Outdated { extra_args } => {
                commands::deps::run_deps(commands::deps::DepsAction::Outdated, extra_args)
                    .map_err(HorusError::from)
            }
            DepsCommands::Audit { extra_args } => {
                commands::deps::run_deps(commands::deps::DepsAction::Audit, extra_args)
                    .map_err(HorusError::from)
            }
        },

        // ── Maintenance commands ────────────────────────────────────────
        Commands::Doctor { verbose, json, fix } => {
            commands::doctor::run_doctor(verbose, json, fix).map_err(HorusError::from)
        }

        Commands::Self_ { command } => match command {
            SelfCommands::Update { check_only } => {
                commands::upgrade::run_upgrade(check_only).map_err(HorusError::from)?;
                Ok(())
            }
        },

        Commands::DeprecatedUpgrade { check_only } => {
            eprintln!(
                "{} `horus upgrade` is deprecated. Use `horus self update` instead.",
                "WARNING:".yellow().bold()
            );
            commands::upgrade::run_upgrade(check_only).map_err(HorusError::from)?;

            Ok(())
        }

        Commands::DeprecatedSync { check } => {
            eprintln!(
                "{} `horus sync` is deprecated. Use `horus doctor{}` instead.",
                "WARNING:".yellow().bold(),
                if check { "" } else { " --fix" }
            );
            commands::doctor::run_doctor(false, false, !check).map_err(HorusError::from)
        }

        Commands::Config { command } => match command {
            ConfigCommands::Get { key } => {
                commands::config::run_config(commands::config::ConfigAction::Get(key))
                    .map_err(HorusError::from)
            }
            ConfigCommands::Set { key, value } => {
                commands::config::run_config(commands::config::ConfigAction::Set(key, value))
                    .map_err(HorusError::from)
            }
            ConfigCommands::List => {
                commands::config::run_config(commands::config::ConfigAction::List)
                    .map_err(HorusError::from)
            }
        },

        Commands::Migrate { dry_run, force } => {
            commands::migrate::run_migrate(dry_run, force).map_err(HorusError::from)
        }

        Commands::Scripts { name, args } => commands::scripts::run_scripts(name, args),

        Commands::Completion { shell } => {
            // Hidden command used by install.sh for automatic completion setup
            let mut cmd = Cli::command();
            let bin_name = cmd.get_name().to_string();
            generate(shell, &mut cmd, bin_name, &mut io::stdout());
            Ok(())
        }

        // ── Native tool proxies ─────────────────────────────────────────
        Commands::Cargo { args } => {
            let code = commands::proxy::run_cargo_proxy(args).map_err(HorusError::from)?;
            std::process::exit(code);
        }
        Commands::Pip { args } => {
            let code = commands::proxy::run_pip_proxy(args).map_err(HorusError::from)?;
            std::process::exit(code);
        }
        Commands::Cmake { args } => {
            let code = commands::proxy::run_cmake_proxy(args).map_err(HorusError::from)?;
            std::process::exit(code);
        }
        Commands::Conan { args } => {
            let code = commands::proxy::run_conan_proxy(args).map_err(HorusError::from)?;
            std::process::exit(code);
        }
        Commands::Vcpkg { args } => {
            let code = commands::proxy::run_vcpkg_proxy(args).map_err(HorusError::from)?;
            std::process::exit(code);
        }
        Commands::Env { init, uninstall } => {
            if uninstall {
                commands::env::run_env_uninstall().map_err(HorusError::from)
            } else if init {
                commands::env::run_env_init().map_err(HorusError::from)
            } else {
                commands::env::run_env_print().map_err(HorusError::from)
            }
        }
        Commands::IsProject => {
            if commands::proxy::run_is_project() {
                Ok(())
            } else {
                std::process::exit(1);
            }
        }
    }
}
