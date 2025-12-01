use clap::{CommandFactory, Parser, Subcommand};
use clap_complete::generate;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use horus_core::memory::{has_native_shm, shm_base_dir};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

// Use modules from the library instead of redeclaring them
use horus_manager::{commands, dashboard, dashboard_tui, registry, security, workspace};

#[derive(Parser)]
#[command(name = "horus")]
#[command(about = "HORUS - Hybrid Optimized Robotics Unified System")]
#[command(version = "0.1.5")]
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

        /// Build only, don't run
        #[arg(short = 'b', long = "build-only")]
        build_only: bool,

        /// Build in release mode
        #[arg(short = 'r', long = "release")]
        release: bool,

        /// Clean build (remove cache)
        #[arg(short = 'c', long = "clean")]
        clean: bool,

        /// Suppress progress indicators
        #[arg(short = 'q', long = "quiet")]
        quiet: bool,

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

    /// Open the unified HORUS dashboard (web-based, auto-opens browser)
    Dashboard {
        /// Port for web dashboard (default: 3000)
        #[arg(value_name = "PORT", default_value = "3000")]
        port: u16,

        /// Use Terminal UI mode instead of web
        #[arg(short = 't', long = "tui")]
        tui: bool,

        /// Reset dashboard password before starting
        #[arg(short = 'r', long = "reset-password")]
        reset_password: bool,
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

    /// Run 2D simulator
    Sim2d {
        /// Headless mode (no rendering/GUI)
        #[arg(long)]
        headless: bool,

        /// World configuration file
        #[arg(long)]
        world: Option<PathBuf>,

        /// World image file (PNG, JPG, PGM) - occupancy grid
        #[arg(long)]
        world_image: Option<PathBuf>,

        /// Resolution in meters per pixel for world image
        #[arg(long)]
        resolution: Option<f32>,

        /// Obstacle threshold 0-255, darker = obstacle
        #[arg(long)]
        threshold: Option<u8>,

        /// Robot configuration file
        #[arg(long)]
        robot: Option<PathBuf>,

        /// HORUS topic prefix for robot topics (e.g. robot -> robot.cmd_vel, robot.odom)
        #[arg(long, default_value = "robot")]
        topic: String,

        /// Robot name for logging
        #[arg(long, default_value = "robot")]
        name: String,

        /// Articulated robot configuration file (YAML) for multi-link robots
        #[arg(long)]
        articulated: Option<PathBuf>,

        /// Use a preset articulated robot (arm_2dof, arm_6dof, humanoid)
        #[arg(long)]
        preset: Option<String>,

        /// Enable gravity (for side-view humanoid simulation)
        #[arg(long)]
        gravity: bool,
    },

    /// Run 3D simulator
    Sim3d {
        /// Headless mode (no rendering/GUI)
        #[arg(long)]
        headless: bool,

        /// Random seed for deterministic simulation
        #[arg(long)]
        seed: Option<u64>,

        /// Robot URDF file
        #[arg(long)]
        robot: Option<PathBuf>,

        /// World/scene file
        #[arg(long)]
        world: Option<PathBuf>,
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
    },

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

    /// List installed plugins
    Plugins {
        /// Show all plugins including disabled
        #[arg(short = 'a', long = "all")]
        all: bool,
    },
}

#[derive(Subcommand)]
enum EnvCommands {
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
                | "dashboard"
                | "pkg"
                | "env"
                | "auth"
                | "sim2d"
                | "sim3d"
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
            build_only,
            release,
            clean,
            quiet,
            args,
        } => {
            // Set quiet mode for progress indicators
            horus_manager::progress::set_quiet(quiet);

            if build_only {
                // Build-only mode - compile but don't execute
                commands::run::execute_build_only(files, release, clean)
                    .map_err(|e| HorusError::Config(e.to_string()))
            } else {
                // Normal run mode (build if needed, then run)
                commands::run::execute_run(files, args, release, clean)
                    .map_err(|e| HorusError::Config(e.to_string()))
            }
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

        Commands::Dashboard {
            port,
            tui,
            reset_password,
        } => {
            // Reset password if requested
            if reset_password {
                security::auth::reset_password().map_err(|e| HorusError::Config(e.to_string()))?;
            }

            if tui {
                println!("{} Opening HORUS Terminal UI dashboard...", "".cyan());
                // Launch TUI dashboard
                dashboard_tui::TuiDashboard::run().map_err(|e| HorusError::Config(e.to_string()))
            } else {
                // Default: Launch web dashboard and auto-open browser
                println!(
                    "{} Starting HORUS web dashboard on http://localhost:{}...",
                    "".cyan(),
                    port
                );
                println!("  {} Password-protected access", "".dimmed());
                println!("  {} Opening browser...", "".dimmed());
                println!(
                    "  {} Use 'horus dashboard -t' for Terminal UI",
                    "Tip:".dimmed()
                );

                tokio::runtime::Runtime::new()
                    .unwrap()
                    .block_on(dashboard::run(port))
                    .map_err(|e| {
                        let err_str = e.to_string();
                        if err_str.contains("Address already in use") || err_str.contains("os error 98") {
                            HorusError::Config(format!(
                                "Port {} is already in use.\n  {} Try a different port: horus dashboard <PORT>\n  {} Example: horus dashboard {}",
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

                PkgCommands::Publish { freeze } => {
                    let client = registry::RegistryClient::new();
                    client
                        .publish(None)
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

                PkgCommands::Enable { command } => commands::pkg::enable_plugin(&command)
                    .map_err(|e| HorusError::Config(e.to_string())),

                PkgCommands::Disable { command, reason } => {
                    commands::pkg::disable_plugin(&command, reason.as_deref())
                        .map_err(|e| HorusError::Config(e.to_string()))
                }

                PkgCommands::Verify { plugin } => commands::pkg::verify_plugins(plugin.as_deref())
                    .map_err(|e| HorusError::Config(e.to_string())),

                PkgCommands::Plugins { all: _ } => {
                    // Show both global and project plugins
                    commands::pkg::list_plugins(true, true)
                        .map_err(|e| HorusError::Config(e.to_string()))
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
            }
        }

        Commands::Auth { command } => match command {
            AuthCommands::Login => commands::github_auth::login(),
            AuthCommands::GenerateKey { name, environment } => {
                commands::github_auth::generate_key(name, environment)
            }
            AuthCommands::Logout => commands::github_auth::logout(),
            AuthCommands::Whoami => commands::github_auth::whoami(),
        },

        Commands::Sim2d {
            headless,
            world,
            world_image,
            resolution,
            threshold,
            robot,
            topic,
            name,
            articulated,
            preset,
            gravity,
        } => {
            use std::env;
            use std::process::Command;

            println!("{} Starting sim2d...", "".cyan());
            if headless {
                println!("  Mode: Headless (no GUI)");
            }
            println!("  Topic: {}", topic);
            println!("  Robot name: {}", name);
            if let Some(ref preset_name) = preset {
                println!("  Articulated preset: {}", preset_name);
            }
            if let Some(ref articulated_path) = articulated {
                println!("  Articulated config: {}", articulated_path.display());
            }
            if gravity {
                println!("  Gravity: enabled");
            }
            if let Some(ref world_path) = world {
                println!("  World: {}", world_path.display());
            }
            if let Some(ref robot_path) = robot {
                println!("  Robot config: {}", robot_path.display());
            }
            println!();

            // Find sim2d path relative to HORUS repo (cross-platform)
            let horus_source = env::var("HORUS_SOURCE")
                .ok()
                .or_else(|| {
                    dirs::home_dir()
                        .map(|h| h.join(".horus/cache/HORUS").to_string_lossy().to_string())
                })
                .unwrap_or_else(|| ".".to_string());

            let sim2d_path = format!("{}/horus_library/tools/sim2d", horus_source);

            // Try to run pre-built binary first (fast path) - cross-platform
            let sim2d_binary = dirs::home_dir()
                .map(|h| h.join(".cargo/bin/sim2d").to_string_lossy().to_string())
                .unwrap_or_else(|| "sim2d".to_string());

            let status = if std::path::Path::new(&sim2d_binary).exists() {
                // Run pre-built binary directly (instant launch!)
                println!("{} Launching sim2d...", "[RUN]".green());
                let mut binary_cmd = Command::new(&sim2d_binary);

                // Add arguments
                if let Some(ref w) = world {
                    binary_cmd.arg("--world").arg(w);
                }
                if let Some(ref w) = world_image {
                    binary_cmd.arg("--world_image").arg(w);
                    if let Some(res) = resolution {
                        binary_cmd.arg("--resolution").arg(res.to_string());
                    }
                    if let Some(thresh) = threshold {
                        binary_cmd.arg("--threshold").arg(thresh.to_string());
                    }
                }
                if let Some(ref r) = robot {
                    binary_cmd.arg("--robot").arg(r);
                }
                binary_cmd.arg("--topic").arg(&topic);
                binary_cmd.arg("--name").arg(&name);
                if headless {
                    binary_cmd.arg("--headless");
                }
                if let Some(ref a) = articulated {
                    binary_cmd.arg("--articulated").arg(a);
                }
                if let Some(ref p) = preset {
                    binary_cmd.arg("--preset").arg(p);
                }
                if gravity {
                    binary_cmd.arg("--gravity");
                }

                binary_cmd
                    .status()
                    .map_err(|e| HorusError::Config(format!("Failed to run sim2d: {}", e)))?
            } else {
                // Fallback: compile and run from source
                println!(
                    "{} Pre-built binary not found, compiling from source...",
                    "".yellow()
                );
                let mut cmd = Command::new("cargo");
                cmd.current_dir(&sim2d_path)
                    .arg("run")
                    .arg("--release")
                    .arg("--");

                // Add arguments
                if let Some(ref w) = world {
                    cmd.arg("--world").arg(w);
                }
                if let Some(ref w) = world_image {
                    cmd.arg("--world_image").arg(w);
                    if let Some(res) = resolution {
                        cmd.arg("--resolution").arg(res.to_string());
                    }
                    if let Some(thresh) = threshold {
                        cmd.arg("--threshold").arg(thresh.to_string());
                    }
                }
                if let Some(ref r) = robot {
                    cmd.arg("--robot").arg(r);
                }
                cmd.arg("--topic").arg(&topic);
                cmd.arg("--name").arg(&name);
                if headless {
                    cmd.arg("--headless");
                }
                if let Some(ref a) = articulated {
                    cmd.arg("--articulated").arg(a);
                }
                if let Some(ref p) = preset {
                    cmd.arg("--preset").arg(p);
                }
                if gravity {
                    cmd.arg("--gravity");
                }

                cmd.status()
                    .map_err(|e| HorusError::Config(format!("Failed to run sim2d: {}. Try running manually: cd {} && cargo run --release", e, sim2d_path)))?
            };

            if !status.success() {
                return Err(HorusError::Config(format!(
                    "sim2d exited with error code: {:?}",
                    status.code()
                )));
            }

            Ok(())
        }

        Commands::Sim3d {
            headless,
            seed,
            robot,
            world,
        } => {
            use std::env;
            use std::process::Command;

            println!("{} Starting sim3d...", "".cyan());
            if headless {
                println!("  Mode: Headless");
            } else {
                println!("  Mode: Visual (default)");
            }
            if let Some(s) = seed {
                println!("  Seed: {}", s);
            }
            if let Some(ref robot_path) = robot {
                println!("  Robot: {}", robot_path.display());
            }
            if let Some(ref world_path) = world {
                println!("  World: {}", world_path.display());
            }
            println!();

            // Convert relative paths to absolute paths before changing directory
            // This is critical for cargo run mode which changes CWD
            let robot = robot.map(|p| {
                std::fs::canonicalize(&p)
                    .unwrap_or_else(|_| std::env::current_dir().unwrap().join(p))
            });
            let world = world.map(|p| {
                std::fs::canonicalize(&p)
                    .unwrap_or_else(|_| std::env::current_dir().unwrap().join(p))
            });

            // Find sim3d path relative to HORUS repo (cross-platform)
            let horus_source = env::var("HORUS_SOURCE")
                .ok()
                .or_else(|| {
                    dirs::home_dir()
                        .map(|h| h.join(".horus/cache/HORUS").to_string_lossy().to_string())
                })
                .unwrap_or_else(|| ".".to_string());

            let sim3d_path = format!("{}/horus_library/tools/sim3d", horus_source);

            // Try to run pre-built binary first (cross-platform)
            let sim3d_binary = dirs::home_dir()
                .map(|h| h.join(".cargo/bin/sim3d").to_string_lossy().to_string())
                .unwrap_or_else(|| "sim3d".to_string());

            let status = if std::path::Path::new(&sim3d_binary).exists() {
                println!("{} Launching sim3d...", "[RUN]".green());
                let mut binary_cmd = Command::new(&sim3d_binary);

                // Only pass --mode headless if needed (visual is default)
                if headless {
                    binary_cmd.arg("--mode").arg("headless");
                }
                if let Some(s) = seed {
                    binary_cmd.arg("--seed").arg(s.to_string());
                }
                if let Some(ref r) = robot {
                    binary_cmd.arg("--robot").arg(r);
                }
                if let Some(ref w) = world {
                    binary_cmd.arg("--world").arg(w);
                }

                binary_cmd
                    .status()
                    .map_err(|e| HorusError::Config(format!("Failed to run sim3d: {}", e)))?
            } else {
                println!(
                    "{} Pre-built binary not found, compiling from source...",
                    "".yellow()
                );
                let mut cmd = Command::new("cargo");
                cmd.current_dir(&sim3d_path).arg("run").arg("--release");

                // Use features based on mode
                if headless {
                    cmd.arg("--no-default-features")
                        .arg("--features")
                        .arg("headless");
                }

                cmd.arg("--");

                // Only pass --mode headless if needed (visual is default)
                if headless {
                    cmd.arg("--mode").arg("headless");
                }
                if let Some(s) = seed {
                    cmd.arg("--seed").arg(s.to_string());
                }
                if let Some(ref r) = robot {
                    cmd.arg("--robot").arg(r);
                }
                if let Some(ref w) = world {
                    cmd.arg("--world").arg(w);
                }

                cmd.status()
                    .map_err(|e| HorusError::Config(format!("Failed to run sim3d: {}. Try running manually: cd {} && cargo run --release", e, sim3d_path)))?
            };

            if !status.success() {
                return Err(HorusError::Config(format!(
                    "sim3d exited with error code: {:?}",
                    status.code()
                )));
            }

            Ok(())
        }

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
