# HORUS Manager

The command-line interface and management tool for the HORUS robotics framework.

## Overview

HORUS Manager (`horus`) is the primary CLI tool for interacting with the HORUS robotics system. It provides a unified interface for building, running, monitoring, and managing robotics applications and components.

## Installation

Build and install from source:

```bash
cd horus_manager
cargo build --release
cargo install --path .
```

Now the `horus` command is available globally:

```bash
horus --version
```

## CLI Commands

HORUS Manager provides 8 main commands:

### 1. `horus new` - Project Creation

Create new HORUS projects with interactive prompts or flags.

```bash
# Interactive mode (prompts for language choice)
horus new my_robot

# Create Rust project
horus new my_robot -r
horus new my_robot --rust

# Create Python project
horus new my_robot -p
horus new my_robot --python

# Create Rust project with macros
horus new my_robot -m
horus new my_robot --macro

# Custom output directory
horus new my_robot -o /path/to/dir
horus new my_robot --output /path/to/dir
```

**Flags:**
- `-r, --rust` - Create Rust project
- `-p, --python` - Create Python project
- `-m, --macro` - Create Rust project with node! macro
- `-o, --output <PATH>` - Output directory

### 2. `horus run` - Build and Execute

Build and run HORUS projects with automatic language detection.

```bash
# Auto-detect and run current directory
horus run

# Run specific file
horus run main.rs
horus run main.py

# Build in release mode
horus run --release
horus run main.rs --release

# Build only (don't run)
horus run --build-only
horus run -b

# Clean build cache before building
horus run --clean
horus run -c

# Pass arguments to the program
horus run main.rs -- arg1 arg2
```

**Flags:**
- `-b, --build-only` - Build without running
- `-r, --release` - Build in release mode
- `-c, --clean` - Clean build cache before building
- Trailing args passed to program

### 3. `horus monitor` - System Monitor

Launch real-time monitoring interface (web or terminal UI).

```bash
# Web interface on port 3000 (auto-opens browser)
horus monitor

# Custom port
horus monitor 3001
horus monitor 8080

# Terminal UI mode (for SSH sessions)
horus monitor -t
horus monitor --tui
```

**Modes:**
- Default: Web interface (Axum on port 3000, auto-opens browser)
- `<PORT>`: Custom port for web interface
- `-t, --tui`: Terminal UI mode

**Monitor Features:**
- Real-time process monitoring
- Topic-based message flow visualization
- Performance metrics and latency tracking
- Interactive node graph
- Package management interface

### 4. `horus pkg` - Package Management

Manage packages with global cache support.

```bash
# Install package
horus pkg install my_package
horus pkg install my_package -v 1.0.0       # Specific version
horus pkg install my_package -g             # Install to global cache
horus pkg install my_package -t /path       # Target workspace

# Remove package
horus pkg remove my_package
horus pkg remove my_package -g              # Remove from global cache
horus pkg remove my_package -t /path        # Target workspace

# List packages
horus pkg list                              # List local packages
horus pkg list -g                           # List global cache
horus pkg list -a                           # List all (local + global)
horus pkg list vision                       # Search registry
```

**Subcommands:**
- `install <package>` - Install package
  - `-v, --ver <VERSION>` - Specific version
  - `-g, --global` - Install to global cache (`~/.horus/cache`)
  - `-t, --target <WORKSPACE>` - Target workspace
  - `-d, --dev` - Dev dependency (not yet supported)
- `remove <package>` - Remove package
  - `-g, --global` - Remove from global cache
  - `-t, --target <WORKSPACE>` - Target workspace
- `list [query]` - List or search packages
  - `-g, --global` - List global cache
  - `-a, --all` - List all (local + global)
- `publish` - Publish current package to registry
  - `--freeze` - Also generate freeze file
- `unpublish <name> <version>` - Remove package from registry

**Package Locations:**
- Global cache: `~/.horus/cache/`
- Local packages: `.horus/packages/` (per project)
- Metadata: `metadata.json` in package directory

### 5. `horus env` - Environment Management

Freeze and restore development environments for reproducibility.

```bash
# Freeze current environment
horus env freeze                            # Creates horus-freeze.yaml
horus env freeze -o custom.yaml             # Custom output file
horus env freeze --output freeze.yaml

# Publish frozen environment to registry
horus env freeze --publish                  # Returns environment ID

# Restore from file
horus env restore horus-freeze.yaml
horus env restore custom.yaml

# Restore from registry ID
horus env restore env_abc123
```

**Subcommands:**
- `freeze` - Freeze current environment
  - `-o, --output <FILE>` - Output file (default: horus-freeze.yaml)
  - `--publish` - Publish to registry
- `restore <source>` - Restore from file or registry ID

**Environment Files:**
- YAML manifest with all packages
- Includes checksums and versions
- Can be published to registry with unique ID

### 6. `horus auth` - Authentication

Manage authentication for the package registry.

```bash
# Login with GitHub OAuth
horus auth login

# Generate API key
horus auth generate-key
horus auth generate-key --name ci-server
horus auth generate-key --environment ci-cd

# Show current user
horus auth whoami

# Logout
horus auth logout
```

**Subcommands:**
- `login` - Login to registry
  - `--github` - GitHub OAuth
- `generate-key` - Generate API key
  - `--name <NAME>` - Key name
  - `--environment <ENV>` - Environment (e.g., ci-cd)
- `logout` - Logout
- `whoami` - Show current user

### 7. `horus sim` - Simulation Tools

Launch 2D or 3D simulators for testing robotics code. Runs 3D simulator by default.

```bash
# Note: Simulation features are under active development

# Run 3D simulator (default, planned)
horus sim
horus sim --headless
horus sim --seed 12345

# Run 2D simulator (use --2d flag)
horus sim --2d --world world.yaml --robot robot.yaml
horus sim --2d --headless
```

**Flags:**
- `--2d` - Run 2D simulator instead of 3D (default: false)
- `--headless` - Run without GUI/rendering (both 2D and 3D)
- `--seed <NUM>` - Random seed for deterministic simulation (3D only)
- `--world <FILE>` - World configuration file (2D only)
- `--robot <FILE>` - Robot configuration file (2D only)
- `--topic <NAME>` - HORUS topic for velocity commands (2D only, default: "cmd_vel")
- `--name <NAME>` - Robot name for logging (2D only, default: "robot")
- `--world_image <FILE>` - World image file (PNG, JPG, PGM) for occupancy grid (2D only)
- `--resolution <NUM>` - Resolution in meters per pixel for world image (2D only)
- `--threshold <NUM>` - Obstacle threshold 0-255, darker = obstacle (2D only)

### 8. `horus version` - Version Information

Display HORUS version information.

```bash
# Show version
horus version
horus --version
horus -V
```

## Project Structure

```
horus_manager/
├── src/
│   ├── main.rs              # CLI entry point
│   ├── commands/            # Command implementations
│   │   ├── mod.rs          # Commands module
│   │   ├── new.rs          # Project creation
│   │   ├── run.rs          # Build and execution
│   │   ├── github_auth.rs  # GitHub authentication
│   │   └── monitor.rs      # System monitoring
│   ├── monitor.rs           # Web monitor (Axum)
│   ├── monitor_tui.rs       # Terminal UI monitor
│   ├── registry.rs          # Package registry client
│   ├── workspace.rs         # Workspace detection
│   ├── graph.rs            # Dependency graph visualization
│   ├── dependency_resolver.rs # Package dependency resolution
│   ├── version.rs          # Version management
│   ├── yaml_utils.rs       # YAML parsing utilities
│   └── crates_io.rs        # Crates.io integration
└── Cargo.toml
```

## Configuration

### Environment Variables

- `HORUS_REGISTRY_URL` - Registry endpoint (default: https://horus-marketplace-api.onrender.com)
- `HORUS_API_KEY` - CLI authentication token

### Package Metadata

Example `metadata.json`:
```json
{
  "name": "test-package",
  "version": "latest",
  "checksum": "9d876483e578e299f45f0b5b0305b4f0b20d49e3274b675ac6e12c85bc7809cb"
}
```

## Development

### Building from Source

```bash
cd horus_manager
cargo build --release
```

### Running Tests

```bash
cargo test
```

### Installing Locally

```bash
cargo install --path .
```

## Examples

### Create and Run a Rust Project

```bash
horus new my_robot -r
cd my_robot
horus run --release
```

### Create Python Project

```bash
horus new sensor_node -p
cd sensor_node
horus run
```


### Package Management

```bash
# Install package globally
horus pkg install vision-toolkit -g

# List all packages
horus pkg list -a

# Remove package
horus pkg remove vision-toolkit
```

### Environment Management

```bash
# Freeze current environment
horus env freeze -o production.yaml

# Restore later
horus env restore production.yaml
```

### Monitor Running System

```bash
# Launch web monitor
horus monitor

# Terminal UI for SSH
horus monitor -t
```

## Implementation Details

### Command Pattern

All commands follow consistent patterns:
- Clap-based argument parsing
- Colored output for clarity
- Error handling with user-friendly messages
- Progress indicators for long operations

### Auto-Detection

`horus run` automatically detects:
- Rust projects (Cargo.toml)
- Python files (.py)
- Current working directory context

### Build System Integration

- Rust: Uses `cargo build` and `cargo run`
- Python: Direct execution with `python3`

## Troubleshooting

### Common Issues

1. **Command not found**: Ensure `cargo install --path .` was run
2. **Package not found**: Check registry URL configuration
3. **Build failures**: Check that required compilers are installed

### Getting Help

```bash
# Command-specific help
horus new --help
horus run --help
horus pkg --help
horus env --help
horus publish --help
horus auth --help
horus monitor --help

# General help
horus --help
```

## License

Part of the HORUS robotics framework. See main project for license details.
