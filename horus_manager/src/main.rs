use clap::{CommandFactory, Parser, Subcommand};
use clap_complete::generate;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::io;
use std::path::PathBuf;

// Use modules from the library instead of redeclaring them
use horus_manager::commands;

#[derive(Parser)]
#[command(name = "horus")]
#[command(about = "Real-time robotics framework with zero-copy IPC and deterministic scheduling")]
#[command(version = env!("CARGO_PKG_VERSION"))]
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
  scripts, script   Run a script defined in horus.toml [scripts]
  test              Run tests for the HORUS project
  check             Validate this project (manifest, sources, workspace)
  clean             Clean build artifacts and shared memory
  launch, l         Launch multiple nodes from a YAML file

Introspection:
  topic, t          Topic interaction (list, echo, info, hz, pub, bw)
  node, n           Node management (list, info, kill)
  service, srv      Service interaction (list, call, info, find)
  action, a         Action interaction (list, info, send-goal, cancel-goal)
  param, p          Parameter management (get, set, list, delete)
  frame, tf         Coordinate frame operations (list, echo, tree)
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
  plugin, plugins   Manage plugins (enable, disable, verify, trust, untrust, trusted)

Development:
  fmt               Format code (rustfmt + ruff/black + clang-format)
  lint              Lint code (clippy + ruff/pylint + clang-tidy)
  doc               Generate documentation
  bench             Run benchmarks
  deps              Dependency insight (tree, why, outdated, audit)

Maintenance:
  doctor            Check this machine (toolchains, RT, shared memory)
  self update       Update the horus CLI to latest version
  config            View/edit horus.toml settings
  migrate           Migrate project to unified horus.toml format
  schema            Print the horus.toml JSON Schema (for editor validation)
  setup-rt          Configure the real-time kernel and system settings

Publishing & Deploy:
  publish           Publish package to registry
  unpublish         Unpublish a package (name@version syntax)
  yank              Yank a version (hide from new installs)
  unyank            Restore a yanked version
  deprecate         Mark a package as deprecated
  undeprecate       Remove deprecation from a package
  owner             Manage owners and transfer ownership
  deploy            Deploy project to a remote robot
  auth              Authentication (login, api-key, signing-key, trust-publisher)

Native Tools:
  env               Shell integration: proxy cargo/pip/pip3/cmake/conan/vcpkg
  completion        Generate a shell completion script (bash/zsh/fish)
  man               Write the man page to stdout (horus man > horus.1)

Options:
{options}
{after-help}")]
#[command(after_help = "\
Quick Start:
  horus new my_robot --rust       Create a new Rust project
  cd my_robot && horus run        Build and run it
  horus topic list                See active topics

More examples:
  horus init                      Initialize workspace in current directory
  horus add serde --source crates.io   Add a Rust dependency
  horus install rplidar@1.2.0    Install a standalone package
  horus bb --anomalies            Show crash anomalies
  horus deploy robot@192.168.1.5  Deploy to a remote robot

Scripting:
  horus check --json              Most reporting commands accept --json
  horus topic list --json         Ask any command with `<command> --help`;
                                  a command without JSON output says so

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
        /// Use Python (`-p` is deprecated here: it means --package elsewhere)
        ///
        /// `-p`, `-r` and `-c` still work and are still accepted, but they are
        /// no longer advertised: on `horus build`, `horus run` and `horus test`
        /// — the commands typed minutes later in the same project — the same
        /// three letters mean `--package`, `--release` and `--clean`. Nothing
        /// errors when the habit crosses over, it just builds something else.
        /// They stop being accepted in HORUS 0.4.0; use the long form.
        #[arg(long = "python", short_alias = 'p', conflicts_with_all = ["rust", "cpp"])]
        python: bool,
        /// Use Rust (`-r` is deprecated here: it means --release elsewhere)
        #[arg(long = "rust", short_alias = 'r', conflicts_with_all = ["python", "cpp"])]
        rust: bool,
        /// Use C++ (`-c` is deprecated here: it means --clean elsewhere)
        #[arg(long = "cpp", short_alias = 'c', conflicts_with_all = ["python", "rust"])]
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

        /// Start from a shipped example instead of the blank template
        ///
        /// `horus new my_robot --from differential_drive` copies
        /// examples/differential_drive, renames the package to `my_robot` and
        /// leaves the build output behind. Pass a name that does not exist to
        /// see the list. The language comes from the example, so the language
        /// flags do not apply.
        #[arg(
            long = "from",
            value_name = "EXAMPLE",
            conflicts_with_all = ["python", "rust", "cpp", "use_macro", "workspace", "lib"]
        )]
        from: Option<String>,

        /// Accept defaults without prompting (for scripts, Dockerfiles and CI)
        ///
        /// The README's `horus new my_robot` opened two prompts that the README
        /// never mentions, so the documented one-liner could not be scripted.
        #[arg(short = 'y', long = "yes")]
        yes: bool,
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

        /// Run in simulation mode using horus-sim3d.
        ///
        /// Without arguments: simulates every [hardware] entry marked `sim = true`.
        /// With arguments: simulates only the named drivers (mixed mode) — the
        /// list narrows the simulated set, it does not create it.
        ///
        /// `[sim-drivers]` is the legacy spelling and is still read, but
        /// `sim = true` under [hardware] is what the manifest documents.
        ///
        /// Examples:
        ///   horus run --sim              # all drivers simulated
        ///   horus run --sim lidar        # only lidar simulated
        ///   horus run --sim lidar camera # lidar + camera simulated
        #[arg(long = "sim", num_args = 0..)]
        sim: Option<Vec<String>>,

        /// Enable LAN network replication (transparent cross-machine topics)
        ///
        /// When enabled, topics are automatically shared with other horus
        /// processes on the same LAN. Zero config — uses UDP multicast.
        /// Override with HORUS_NET_PEER=ip for WiFi/cross-subnet.
        #[arg(long = "net")]
        net: bool,
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

    /// Validate this project: horus.toml, source files, or the whole workspace
    ///
    /// Checks what is in the repository. `horus doctor` checks the machine it
    /// is on — toolchains, real-time capability, shared memory — and the two do
    /// not overlap: a project can be valid on a machine that cannot build it,
    /// and a healthy machine says nothing about whether the manifest parses.
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

        /// Check the machine instead of the project — same as `horus doctor`
        ///
        /// Exists so that `horus check --full` has a switch for the half it
        /// runs. On its own it is `horus doctor` and nothing more.
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

        /// Also remove live namespaces belonging to other processes
        #[arg(long = "all-namespaces")]
        all_namespaces: bool,

        /// Output as JSON
        #[arg(long = "json")]
        json: bool,
    },

    /// Launch multiple nodes from a YAML file
    #[command(visible_alias = "l")]
    Launch {
        /// Path to launch file (YAML). Omit when using --status.
        file: Option<std::path::PathBuf>,

        /// Show what would launch without actually launching
        #[arg(short = 'n', long = "dry-run")]
        dry_run: bool,

        /// Namespace prefix for all nodes
        #[arg(long = "namespace")]
        namespace: Option<String>,

        /// List nodes in the launch file without launching
        #[arg(long = "list")]
        list: bool,

        /// Show active launch sessions (no file needed)
        #[arg(long = "status")]
        status: bool,

        /// Stop a running launch session by name
        #[arg(long = "stop")]
        stop: Option<String>,

        /// Seconds to wait for graceful shutdown before SIGKILL (default: 2)
        #[arg(long = "shutdown-timeout", default_value = "2")]
        shutdown_timeout: u64,
    },

    // ── Introspection ────────────────────────────────────────────────────
    /// Topic interaction (list, echo, info, hz, pub, bw)
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

        /// Custom blackbox directory.
        ///
        /// Defaults to this project's .horus/blackbox/ when one exists with
        /// recordings in it, and otherwise to the machine-wide
        /// ~/.local/share/horus/blackbox — so outside a project, or before the
        /// first recording, this reads every project's events on the machine.
        #[arg(short = 'p', long = "path")]
        path: Option<PathBuf>,

        /// Clear all blackbox data (with confirmation)
        #[arg(long = "clear")]
        clear: bool,
    },

    /// Monitor running HORUS nodes, topics, and system health (plugin)
    #[command(visible_alias = "mon")]
    Monitor {
        /// Arguments passed to horus-monitor plugin
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    // ── Dependencies ────────────────────────────────────────────────────
    /// Add a dependency to horus.toml (crates.io, PyPI, system, registry, git, path)
    ///
    /// `add` edits the horus.toml you are standing in: the dependency belongs
    /// to this project and is resolved the next time it builds. `horus install`
    /// puts a standalone package or plugin on the machine and writes no
    /// manifest entry. Undo `add` with `horus remove`, `install` with
    /// `horus uninstall`.
    Add {
        /// Dependency name (supports name@version syntax, e.g. serde@1.0)
        name: String,
        /// Version requirement (alternative to name@version syntax)
        #[arg(long = "ver", hide = true)]
        ver: Option<String>,
        /// Dependency source: crates.io, pypi, system, registry, git, path
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
    ///
    /// The undo of `horus add` — it edits this project's manifest. To take an
    /// installed package or plugin off the machine, use `horus uninstall`.
    Remove {
        /// Dependency name
        name: String,
        /// Also clean unused packages from cache
        #[arg(long = "purge")]
        purge: bool,
    },

    // ── Packages ─────────────────────────────────────────────────────────
    /// Install a standalone package or plugin from the registry
    ///
    /// Installs onto the machine — global by default, `-t <workspace>` targets
    /// a registered workspace — and writes nothing to horus.toml. To declare a
    /// dependency of *this* project instead, use `horus add`. Undo with
    /// `horus uninstall`.
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
    ///
    /// The undo of `horus install`. It does not touch horus.toml — to drop a
    /// dependency this project declares, use `horus remove`.
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
    ///
    /// Lists what `horus install` has put on this machine. This is not the
    /// runtime `list`: `horus topic list`, `horus node list`, `horus param
    /// list`, `horus service list`, `horus action list`, `horus frame list`
    /// and `horus msg list` ask a running system what it has right now, and
    /// come back empty when nothing is running.
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
        #[arg(short = 'n', long = "dry-run")]
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
    /// Plugin management (enable, disable, verify, trust, untrust, trusted)
    #[command(visible_alias = "plugins")]
    Plugin {
        #[command(subcommand)]
        command: PluginCommands,
    },

    // ── Development ──────────────────────────────────────────────────────
    /// Format code (rustfmt + ruff/black + clang-format)
    Fmt {
        /// Check formatting without modifying files
        #[arg(long = "check")]
        check: bool,

        /// Additional arguments passed to underlying tools
        #[arg(last = true)]
        extra_args: Vec<String>,
    },

    /// Lint code (clippy + ruff/pylint + clang-tidy)
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

        /// Condensed one-line-per-symbol output
        #[arg(long = "brief")]
        brief: bool,

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
    /// Check this machine: toolchains, real-time capability, shared memory
    ///
    /// Checks the environment. `horus check` checks the repository, and neither
    /// substitutes for the other.
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

        /// Run RT readiness report: system audit + jitter benchmark + IPC benchmark
        #[arg(long = "rt")]
        rt: bool,
    },

    /// Configure real-time kernel and system settings
    #[command(name = "setup-rt")]
    SetupRt {
        /// Just check current RT status, don't install anything
        #[arg(long = "check")]
        check: bool,

        /// Undo RT setup (remove limits file)
        #[arg(long = "undo")]
        undo: bool,
    },

    /// Manage the horus CLI itself
    #[command(name = "self")]
    Self_ {
        #[command(subcommand)]
        command: SelfCommands,
    },

    /// View/edit horus.toml settings
    Config {
        #[command(subcommand)]
        command: ConfigCommands,
    },

    /// Print the JSON Schema for horus.toml (for editor validation)
    ///
    /// Point your editor at the output for autocomplete, hover docs and inline
    /// errors on horus.toml:
    ///
    ///   horus schema > horus-schema.json
    ///
    /// then in VS Code (Even Better TOML) or any taplo-based setup:
    ///
    ///   [[schema]]
    ///   path = "horus-schema.json"
    ///   include = ["**/horus.toml"]
    Schema {
        /// Write to a file instead of stdout
        #[arg(short = 'o', long = "output")]
        output: Option<PathBuf>,
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
    /// Publish this package to the registry
    ///
    /// Packages the current project and uploads it. Unrelated to the
    /// `publish()` call in the Rust, C++ and Python APIs, which sends one
    /// message on a topic — the CLI equivalent of that is `horus topic pub`.
    ///
    /// The name stays `publish` rather than moving to `horus registry publish`:
    /// `unpublish`, `yank`, `unyank`, `deprecate`, `undeprecate` and `owner`
    /// are all top-level registry verbs, and renaming one of seven would trade
    /// a documented ambiguity for an undocumented inconsistency.
    Publish {
        /// Validate package without actually publishing
        #[arg(short = 'n', long = "dry-run")]
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
    Completion {
        /// Shell to generate completions for
        #[arg(value_enum)]
        shell: clap_complete::Shell,
    },

    /// Write the man page to stdout (`horus man > horus.1`)
    ///
    /// Rendered from the same clap tree the CLI is built from, so it cannot
    /// describe a command the binary does not have. HORUS shipped no man page
    /// at all — `man horus` found nothing, on a tool whose install script
    /// already places a completion script.
    Man,

    // ── Native Tool Proxies ─────────────────────────────────────────────
    // A proxy must not answer for the tool it proxies. clap claimed --help/-h
    // and --version/-V before the arguments reached the real binary, so inside
    // a horus project (where the shell function delegates) `cargo --version`
    // printed "horus-cargo 0.2.2" instead of "cargo 1.97.1" — a lie that any
    // script parsing that output acts on.
    /// Transparent cargo proxy — delegates to real cargo with horus.toml sync
    #[command(
        name = "cargo",
        hide = true,
        disable_help_flag = true,
        disable_version_flag = true
    )]
    Cargo {
        /// Arguments passed to cargo
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent pip proxy — delegates to real pip with horus.toml sync
    #[command(
        name = "pip",
        hide = true,
        disable_help_flag = true,
        disable_version_flag = true
    )]
    Pip {
        /// Arguments passed to pip
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent cmake proxy — delegates to real cmake with horus.toml sync
    #[command(
        name = "cmake",
        hide = true,
        disable_help_flag = true,
        disable_version_flag = true
    )]
    Cmake {
        /// Arguments passed to cmake
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent conan proxy — delegates to real conan with horus.toml sync
    #[command(
        name = "conan",
        hide = true,
        disable_help_flag = true,
        disable_version_flag = true
    )]
    Conan {
        /// Arguments passed to conan
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Transparent vcpkg proxy — delegates to real vcpkg with horus.toml sync
    #[command(
        name = "vcpkg",
        hide = true,
        disable_help_flag = true,
        disable_version_flag = true
    )]
    Vcpkg {
        /// Arguments passed to vcpkg
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },

    /// Manage shell integration for native tools (cargo, pip, pip3, cmake, conan, vcpkg)
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
    /// Trust a project-local plugin so it may execute (records its content hash
    /// in the out-of-repo trust store). Required before an unsigned plugin that
    /// ships inside a checkout's .horus/ can run.
    Trust {
        /// Plugin command name to trust
        command: String,
    },
    /// Remove a plugin from the trust store (it will refuse to execute after).
    Untrust {
        /// Plugin command name to untrust
        command: String,
    },
    /// List plugins that have been trusted for execution.
    Trusted {
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
    /// Trust a publisher's public key for package signature verification
    #[command(name = "trust-publisher")]
    TrustPublisher {
        /// Publisher name — must match the package owner (or the package name)
        name: String,
        /// Path to the publisher's .pub file, or the 64-char hex key itself
        key: String,
    },
    /// List publisher keys trusted for signature verification
    #[command(name = "publishers")]
    Publishers,
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

    /// Delete old recording sessions
    Clean {
        /// Only delete sessions older than N days
        #[arg(long = "older-than")]
        older_than_days: Option<u64>,
        /// Show what would be deleted without deleting
        #[arg(short = 'n', long = "dry-run")]
        dry_run: bool,
        /// Force without confirmation
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
        ///
        /// Hyphen-led values are accepted: every offset, bias and joint limit in
        /// robotics can be negative, and `horus param set min_angle -1.57` used
        /// to fail with `unexpected argument '-1' found`.
        #[arg(allow_hyphen_values = true)]
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

    /// Generate Rust, C++ and Python types from msgs/*.hmsg
    Gen {
        /// Verify the generated files are up to date without rewriting them
        #[arg(long = "check")]
        check: bool,
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

/// The command tree a completion script should offer.
///
/// `clap_complete` walks `Command::get_subcommands()`, which yields hidden
/// subcommands as well as visible ones, so the generated script offered every
/// internal as a top-level candidate for `horus <TAB>`: the five native-tool
/// proxies (`cargo`, `pip`, `cmake`, `conan`, `vcpkg` — reachable only through
/// the shell functions `horus env --init` writes, never meant to be typed) and
/// `_is-project`, which exists purely as an exit code for those functions.
/// Installing completions therefore undid the `hide = true` that keeps them out
/// of `--help`.
///
/// clap has no API for removing a subcommand, so the visible half of the tree
/// is rebuilt onto a fresh `Command`. Only the pieces a completion script reads
/// are copied: the top-level arguments and the visible subcommands (each
/// cloned whole, so their own arguments and nested subcommands come with them).
fn completion_tree() -> clap::Command {
    let full = Cli::command();
    debug_assert_eq!(full.get_name(), "horus", "completion bin name drifted");
    // The root settings a completion script can observe: no `help`
    // subcommand, and `-V`/`--version` on every subcommand. `--help` and
    // `--version` themselves are left for clap to generate, exactly as it does
    // for the real tree — copying them from `full` would drop them, since the
    // derive only materialises them at build() time.
    let mut tree = clap::Command::new("horus")
        .version(env!("CARGO_PKG_VERSION"))
        .disable_help_subcommand(true)
        .propagate_version(true);
    for arg in full.get_arguments() {
        tree = tree.arg(arg.clone());
    }
    for sub in full.get_subcommands() {
        if !sub.is_hide_set() {
            tree = tree.subcommand(sub.clone());
        }
    }
    tree
}

/// Short flags on `horus new` that mean something else on the commands the
/// same user types next, as `(short, long form, what it means elsewhere)`.
///
/// `horus new x -r` selects Rust; `horus build -r` and `horus run -r` select
/// release. Nothing errors when the habit crosses over — the wrong thing is
/// built, quietly. Same for `-p` (Python here, `--package` there) and `-c`
/// (C++ here, `--clean` there).
///
/// They are `short_alias` rather than `short` on the `New` variant, so
/// `horus new --help` no longer teaches them, and this notice tells anyone
/// still typing them what to switch to and when they stop working. Removing
/// them outright would break every script, README and CI job that uses them,
/// which is what a deprecation cycle is for.
const DEPRECATED_NEW_SHORT_FLAGS: [(char, &str, &str); 3] = [
    (
        'r',
        "--rust",
        "`--release` on `horus build`, `horus run` and `horus test`",
    ),
    (
        'p',
        "--python",
        "`--package` on `horus build` and `horus run`",
    ),
    ('c', "--cpp", "`--clean` on `horus build` and `horus run`"),
];

/// The release in which the short forms above stop being accepted.
const NEW_SHORT_FLAG_REMOVAL: &str = "0.4.0";

/// Tell anyone still using `horus new -r` what it will mean tomorrow.
///
/// clap does not report which spelling matched an argument, so the raw
/// command line is re-read. Only the tokens of the `new` invocation itself are
/// considered, and only single-dash ones, so a project literally named `-r`
/// after a `--` is not mistaken for a flag.
fn warn_about_deprecated_new_short_flags() {
    let mut args = std::env::args().skip(1).skip_while(|a| a != "new");
    if args.next().is_none() {
        return; // not a `new` invocation
    }

    let mut seen: Vec<char> = Vec::new();
    for arg in args {
        if arg == "--" {
            break;
        }
        if !arg.starts_with('-') || arg.starts_with("--") {
            continue;
        }
        for (short, _, _) in DEPRECATED_NEW_SHORT_FLAGS {
            if arg[1..].contains(short) && !seen.contains(&short) {
                seen.push(short);
            }
        }
    }

    for short in seen {
        let (_, long, elsewhere) = DEPRECATED_NEW_SHORT_FLAGS
            .iter()
            .find(|(c, _, _)| *c == short)
            .expect("only chars from the table are collected");
        horus_manager::cli_output::warn(&format!(
            "`-{short}` on `horus new` means `{long}`, but `-{short}` means {elsewhere}.\n  Write `{long}`: `-{short}` here is deprecated and stops being accepted in HORUS {NEW_SHORT_FLAG_REMOVAL}."
        ));
    }
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

/// `horus run --sim` could not start its simulator, so refuse to run.
///
/// This used to be a `log::warn!` that fell through to `execute_run`, which
/// builds and runs the node against whatever is actually attached. The flag was
/// asked for explicitly, and the whole point of asking for it is that the code
/// about to run should not reach the hardware — on a bench with actuators
/// powered, the difference between a simulation and a motion command is the
/// difference this warning was quietly deciding. A warning scrolls past in a
/// build log; a non-zero exit does not.
///
/// The message carries the plugin's own error through unchanged: every variant
/// out of `spawn_background` already names the plugin and the install command,
/// and re-wrapping it printed the remediation twice.
fn sim_unavailable(simulator: &str, cause: &str) -> HorusError {
    HorusError::Config(ConfigError::Other(format!(
        "--sim was requested but the '{simulator}' simulator could not be started, \
         so nothing would be simulated: {cause}\n\n\
         Refusing to run: without the simulator the nodes would drive the real \
         hardware. Re-run without --sim if that is what you want."
    )))
}

/// The commands that can emit JSON, derived from the clap tree rather than
/// written down — a hand-kept list is exactly what drifted in the first place.
fn json_capable_commands() -> Vec<String> {
    fn walk(cmd: &clap::Command, path: &str, out: &mut Vec<String>) {
        let here = if path.is_empty() {
            cmd.get_name().to_string()
        } else {
            format!("{path} {}", cmd.get_name())
        };
        if cmd.get_arguments().any(|a| a.get_long() == Some("json")) {
            out.push(here.clone());
        }
        for sub in cmd.get_subcommands() {
            walk(sub, &here, out);
        }
    }
    let cmd = Cli::command();
    let mut out = Vec::new();
    for sub in cmd.get_subcommands() {
        walk(sub, "", &mut out);
    }
    out.sort();
    out
}

/// Answer `--json` on a command that has none, instead of letting clap suggest
/// passing it through to the user's program.
///
/// Returns `None` for every other parse error so clap keeps its own reporting —
/// this is a targeted replacement, not a general error handler.
fn json_unsupported_message(args: &[String], e: &clap::Error) -> Option<String> {
    if e.kind() != clap::error::ErrorKind::UnknownArgument {
        return None;
    }
    if !args.iter().any(|a| a == "--json") {
        return None;
    }
    let invoked: Vec<&str> = args
        .iter()
        .skip(1)
        .take_while(|a| !a.starts_with('-'))
        .map(|s| s.as_str())
        .collect();
    let name = invoked.join(" ");
    let capable = json_capable_commands();
    // A 31-line dump is not an error message. Name a few of the ones a reader
    // is most likely to have meant and say how to check the rest.
    let examples = ["check", "build", "topic list", "node list", "doctor"]
        .iter()
        .filter(|c| capable.iter().any(|k| k == *c))
        .map(|c| format!("`horus {c} --json`"))
        .collect::<Vec<_>>()
        .join(", ");
    Some(format!(
        "`horus {name}` has no JSON output, so --json is not accepted here.\n\n\
         {} other commands do — {examples} among them. `horus <command> --help` \
         says whether one accepts it.\n\n\
         To pass --json through to your own program, put it after `--`.",
        capable.len()
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

    // Normal clap parsing.
    //
    // `--json` is declared per-command rather than globally, so a command that
    // has no JSON output rejects it through clap's generic unknown-argument
    // path. That path ends in
    //
    //     tip: to pass '--json' as a value, use '-- --json'
    //
    // which is actively misleading: following it passes the literal string
    // `--json` to the user's program. Scripting the CLI is the one job the flag
    // exists for, so the failure is worth answering properly.
    let cli = match Cli::try_parse() {
        Ok(cli) => cli,
        Err(e) => {
            if let Some(msg) = json_unsupported_message(&args, &e) {
                eprintln!("{} {}", "Error:".red().bold(), msg);
                std::process::exit(2);
            }
            e.exit()
        }
    };

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
            yes,
            from,
        } => {
            warn_about_deprecated_new_short_flags();
            // `--yes` reuses the same signal the non-TTY path uses, so a
            // scripted run and an interactive `--yes` take identical defaults.
            if yes {
                std::env::set_var("HORUS_ASSUME_YES", "1");
            }

            // `--from <example>` is a different constructor, not a variant of
            // the template one: the language, the sources and the manifest all
            // come from the example, so nothing below applies. Clap rejects the
            // language/workspace/lib flags alongside it rather than silently
            // ignoring them.
            if let Some(example) = from {
                horus_manager::version::check_and_prompt_update().map_err(HorusError::from)?;
                return commands::new::create_project_from_example(name, path, example)
                    .map_err(HorusError::from);
            }

            let language = if python {
                "python"
            } else if cpp {
                "cpp"
            } else if rust || use_macro || workspace {
                "rust"
            } else {
                "" // Will use interactive prompt
            };

            // Version compatibility reads the global ~/.horus install state, so
            // it belongs here at the dispatch layer rather than inside
            // create_new_project, which the unit tests exercise directly.
            horus_manager::version::check_and_prompt_update().map_err(HorusError::from)?;

            commands::new::create_new_project(
                name,
                path,
                language.to_string(),
                use_macro,
                workspace,
                lib,
            )
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
            sim,
            net,
        } => {
            // Enable JSON diagnostics mode globally (read by error_wrapper::emit_diagnostic)
            if json_diagnostics {
                horus_manager::error_wrapper::set_json_diagnostics(true);
            }

            // `--net` used to set this variable and stop there. HORUS_NET is
            // read in exactly one place — `horus doctor` — so the flag never
            // reached the build: the `net` Cargo feature is not in `default`,
            // the generated manifest carried no `features = ["net"]`, and the
            // resulting binary contained no horus_net symbols at all.
            //
            // Push it through the capability list as well, which is what
            // actually turns into `cargo --features`.
            let mut enable = enable;
            if net {
                std::env::set_var("HORUS_NET", "1");
                let list = enable.get_or_insert_with(Vec::new);
                if !list.iter().any(|c| c.eq_ignore_ascii_case("net")) {
                    list.push("net".to_string());
                }
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

            // Handle --sim flag: auto-launch simulator plugin (mujoco, sim3d, etc.)
            let mut _sim_child: Option<std::process::Child> = None;
            if let Some(ref sim_targets) = sim {
                if sim_targets.is_empty() {
                    log::info!(
                        "Simulation mode: every [hardware] entry with `sim = true` will be simulated"
                    );
                } else {
                    log::info!(
                        "Simulation mode (selective): simulating [{}]",
                        sim_targets.join(", ")
                    );
                }
                std::env::set_var("HORUS_SIM_MODE", "1");
                if !sim_targets.is_empty() {
                    std::env::set_var("HORUS_SIM_TARGETS", sim_targets.join(","));
                }

                // Auto-launch simulator plugin in background
                let mut sim_args: Vec<String> = vec!["--driver-mode".to_string()];
                let mut simulator_name = "sim3d".to_string(); // default

                // Pass URDF from [robot].description if available
                if let Ok(manifest) = horus_manager::manifest::HorusManifest::load_from(
                    std::path::Path::new("horus.toml"),
                ) {
                    if let Some(ref robot) = manifest.robot {
                        if let Some(ref desc) = robot.description {
                            sim_args.push("--robot".to_string());
                            sim_args.push(desc.clone());
                        }
                        sim_args.push("--robot-name".to_string());
                        sim_args.push(robot.name.clone());
                        // Use configured simulator (default: "sim3d")
                        if let Some(ref sim) = robot.simulator {
                            simulator_name = sim.clone();
                        }
                    }
                }

                // Launch simulator plugin via PluginExecutor
                match horus_manager::plugins::PluginExecutor::new() {
                    Ok(executor) => {
                        match executor.spawn_background(&simulator_name, &sim_args, &[]) {
                            Ok(child) => {
                                log::info!(
                                    "{} launched (PID: {}), waiting for startup...",
                                    simulator_name,
                                    child.id()
                                );
                                std::thread::sleep(std::time::Duration::from_secs(2));
                                _sim_child = Some(child);
                            }
                            Err(e) => {
                                // The inner error already names the plugin and the
                                // install command — every variant out of
                                // spawn_background does. Wrapping it produced:
                                //
                                //   Failed to auto-launch 'sim3d': Plugin 'sim3d' not
                                //   found. Install it with: horus install horus-sim3d.
                                //   Install with: horus install horus-sim3d
                                //
                                // the remediation twice and the plugin name four times.
                                return Err(sim_unavailable(&simulator_name, &e.to_string()));
                            }
                        }
                    }
                    Err(e) => {
                        return Err(sim_unavailable(&simulator_name, &e.to_string()));
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
            // Clean up sim3d process on exit
            if let Some(mut child) = _sim_child {
                log::info!("Shutting down simulator (PID: {})...", child.id());
                let _ = child.kill();
                let _ = child.wait();
            }

            let result: Result<(), HorusError> = result.map_err(HorusError::from);
            if no_hooks {
                return result;
            }
            match horus_manager::manifest::HorusManifest::load_from(std::path::Path::new(
                "horus.toml",
            )) {
                Ok(manifest) => commands::hooks::run_teardown_hooks("post_run", &manifest, result),
                Err(_) => result,
            }
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
            }

            let result: Result<(), HorusError> = result.map_err(HorusError::from);
            if no_hooks {
                return result;
            }
            match horus_manager::manifest::HorusManifest::load_from(std::path::Path::new(
                "horus.toml",
            )) {
                Ok(manifest) => {
                    commands::hooks::run_teardown_hooks("post_build", &manifest, result)
                }
                Err(_) => result,
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
                            "[ok]".green(),
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
                        println!("{} Failed to parse horus.lock: {}", "[x]".red(), e);
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
                    "[ok]".green(),
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

        Commands::Monitor { args } => {
            // Delegate to horus-monitor plugin binary
            use std::process::Command;

            // Try to find horus-monitor binary
            let binary = which_monitor_binary();
            match binary {
                Some(path) => {
                    let status = Command::new(&path).args(&args).status().map_err(|e| {
                        HorusError::Config(ConfigError::Other(format!(
                            "Failed to run horus-monitor: {}",
                            e
                        )))
                    })?;
                    if status.success() {
                        Ok(())
                    } else {
                        Err(HorusError::Config(ConfigError::Other(format!(
                            "horus-monitor exited with {}",
                            status
                        ))))
                    }
                }
                None => {
                    eprintln!(
                        "{} The monitor plugin is not installed.",
                        "Note:".yellow().bold()
                    );
                    eprintln!("  Install with: {}", "horus install horus-monitor".cyan());
                    Err(HorusError::Config(ConfigError::Other(
                        "Monitor plugin not installed. Run: horus install horus-monitor".into(),
                    )))
                }
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
                .map_err(horus_core::error::HorusError::Config),
        },

        Commands::Clean {
            shm,
            all,
            dry_run,
            force,
            all_namespaces,
            json,
        } => commands::clean::run_clean(shm, all, dry_run, force, all_namespaces, json),

        Commands::Launch {
            file,
            dry_run,
            namespace,
            list,
            status,
            stop,
            shutdown_timeout,
        } => {
            if status {
                commands::launch::show_launch_status()
            } else if let Some(ref session_name) = stop {
                commands::launch::stop_launch_session(session_name)
            } else if let Some(ref file) = file {
                if list {
                    commands::launch::list_launch_nodes(file)
                } else {
                    commands::launch::run_launch(file, dry_run, namespace, shutdown_timeout)
                }
            } else {
                Err(horus_core::error::HorusError::Config(
                    horus_core::error::ConfigError::Other(
                        "Launch file required. Use --status to see active sessions.".into(),
                    ),
                ))
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
            MsgCommands::Gen { check, json } => commands::msg::generate_messages(check, json),
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
                commands::pkg::run_add(pkg_name.clone(), pkg_ver.clone(), true, false, source)
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
                commands::pkg::run_install_standalone(&pkg_name, pkg_ver.as_deref(), target)
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
            PluginCommands::Trust { command } => {
                commands::pkg::trust_plugin(&command).map_err(HorusError::from)
            }
            PluginCommands::Untrust { command } => {
                commands::pkg::untrust_plugin(&command).map_err(HorusError::from)
            }
            PluginCommands::Trusted { json } => {
                commands::pkg::list_trusted_plugins(json).map_err(HorusError::from)
            }
        },

        Commands::Auth { command } => match command {
            AuthCommands::Login => commands::github_auth::login(),
            AuthCommands::ApiKey { name, environment } => {
                commands::github_auth::generate_key(name, environment)
            }
            AuthCommands::SigningKey => commands::pkg::run_keygen(),
            AuthCommands::TrustPublisher { name, key } => {
                horus_manager::registry::trust_publisher_key(&name, &key).map_err(HorusError::from)
            }
            AuthCommands::Publishers => {
                horus_manager::registry::list_publisher_keys().map_err(HorusError::from)
            }
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
            RecordCommands::Clean {
                older_than_days,
                dry_run,
                force,
            } => commands::record::run_clean(older_than_days, dry_run, force),
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
            brief,
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
                    brief,
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
        Commands::Doctor {
            verbose,
            json,
            fix,
            rt,
        } => {
            if rt {
                commands::doctor::run_rt_report().map_err(HorusError::from)
            } else {
                commands::doctor::run_doctor(verbose, json, fix).map_err(HorusError::from)
            }
        }

        Commands::SetupRt { check, undo } => {
            commands::setup_rt::run_setup_rt(check, undo).map_err(HorusError::from)
        }

        Commands::Self_ { command } => match command {
            SelfCommands::Update { check_only } => {
                commands::upgrade::run_upgrade(check_only).map_err(HorusError::from)?;
                Ok(())
            }
        },

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

        // The generator lives behind the `schema` feature. That feature is on by
        // default and the shipped binary is meant to have it — but a build that
        // turns it off must still compile, and for a while it did not: adding
        // this arm broke `cargo build -p horus_manager --no-default-features`,
        // which is the exact command `install.sh` runs.
        #[cfg(feature = "schema")]
        Commands::Schema { output } => {
            let schema = horus_manager::manifest::generate_manifest_schema();
            match output {
                Some(path) => {
                    std::fs::write(&path, &schema).map_err(|e| {
                        HorusError::Config(ConfigError::Other(format!(
                            "Failed to write {}: {e}",
                            path.display()
                        )))
                    })?;
                    println!("Wrote horus.toml JSON Schema to {}", path.display());
                }
                None => println!("{schema}"),
            }
            Ok(())
        }

        #[cfg(not(feature = "schema"))]
        Commands::Schema { .. } => Err(HorusError::Config(ConfigError::Other(
            "This binary was built without the `schema` feature, so it cannot \
             emit the horus.toml JSON Schema. Rebuild with \
             `cargo build -p horus_manager` (the feature is on by default)."
                .to_string(),
        ))),

        Commands::Migrate { dry_run, force } => {
            commands::migrate::run_migrate(dry_run, force).map_err(HorusError::from)
        }

        Commands::Scripts { name, args } => commands::scripts::run_scripts(name, args),

        Commands::Man => {
            let cmd = Cli::command();
            clap_mangen::Man::new(cmd)
                .render(&mut io::stdout())
                .map_err(|e| HorusError::from(anyhow::anyhow!("rendering man page: {e}")))?;
            Ok(())
        }

        Commands::Completion { shell } => {
            // install.sh calls this to set completions up automatically.
            // `completion_tree()` rather than `Cli::command()`: the hidden
            // proxies and `_is-project` must not be offered by `horus <TAB>`.
            let mut cmd = completion_tree();
            generate(shell, &mut cmd, "horus", &mut io::stdout());
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

/// Find the horus-monitor binary on known paths.
fn which_monitor_binary() -> Option<std::path::PathBuf> {
    // 1. Check ~/.horus/bin/ (plugin install location)
    if let Some(home) = dirs::home_dir() {
        let plugin_path = home.join(".horus/bin/horus-monitor");
        if plugin_path.exists() {
            return Some(plugin_path);
        }
    }

    // 2. Project-local .horus/bin/ — OPT-IN ONLY.
    //
    // This is a CWD-relative path, and it used to be checked unconditionally,
    // BEFORE PATH. So `cd`ing into a cloned repository that ships
    // `.horus/bin/horus-monitor` and running `horus monitor` executed that
    // binary directly: no trust gate, no signature check, none of the sandbox
    // that `plugins::executor` applies — and it SHADOWED a properly installed
    // one. Cloning a repo should not be sufficient to run code.
    //
    // `.horus/` is generated build output (see CLAUDE.md — never hand-authored),
    // so a repo carrying a binary there is already anomalous. Kept behind an
    // explicit opt-in rather than deleted, for the workflow where a developer
    // genuinely builds the monitor into their own project tree.
    let local_opt_in = std::env::var("HORUS_ALLOW_LOCAL_PLUGINS")
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
        .unwrap_or(false);
    if local_opt_in {
        let local = std::path::PathBuf::from(".horus/bin/horus-monitor");
        if local.exists() {
            eprintln!(
                "{} Running horus-monitor from the project-local .horus/bin/ \
                 (HORUS_ALLOW_LOCAL_PLUGINS is set). This binary is NOT trust-checked.",
                "Warning:".yellow().bold()
            );
            return Some(local);
        }
    }

    // 3. Check PATH
    if let Ok(output) = std::process::Command::new("which")
        .arg("horus-monitor")
        .output()
    {
        if output.status.success() {
            let path = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if !path.is_empty() {
                return Some(std::path::PathBuf::from(path));
            }
        }
    }

    None
}
