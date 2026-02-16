# HORUS Manager

The command-line interface and management tool for the HORUS robotics framework.

## Overview

HORUS Manager (`horus`) is the primary CLI tool for interacting with the HORUS robotics system. It provides a unified interface for building, running, monitoring, and managing robotics applications and components.

## Installation

```bash
cd horus_manager
cargo build --release
cargo install --path .
```

## CLI Commands

HORUS Manager provides 25+ commands organized by function:

### Core Workflow

| Command | Description |
|---------|-------------|
| `horus init` | Initialize HORUS workspace in current directory |
| `horus new <name>` | Create new project (`-r` Rust, `-p` Python, `-m` macro) |
| `horus run [files]` | Build and run project or file(s) (`--release`, `--record`) |
| `horus build [files]` | Build without running (`--release`, `--clean`) |
| `horus test [filter]` | Run tests (`--sim`, `--integration`, `--parallel`) |
| `horus check [path]` | Validate workspace, horus.yaml, or source files |
| `horus clean` | Clean build artifacts (`--shm` for shared memory, `--all`) |

### Introspection & Monitoring

| Command | Description |
|---------|-------------|
| `horus monitor [port]` | Web UI or TUI (`-t`) system monitor |
| `horus topic list\|echo\|pub` | Topic interaction and debugging |
| `horus node list\|info\|kill` | Node management |
| `horus param get\|set\|list\|delete` | Runtime parameter management |
| `horus hf list\|echo\|tree` | HFrame coordinate transform inspection |
| `horus msg list\|show\|fields` | Message type introspection |
| `horus log [node]` | View and filter logs (`-f` follow, `-l` level) |

### Package Ecosystem

| Command | Description |
|---------|-------------|
| `horus pkg install\|remove\|list` | Package management with global cache |
| `horus add <name>` | Smart add (auto-detects package/driver/plugin) |
| `horus remove <name>` | Smart remove with optional `--purge` |
| `horus driver list\|info\|search` | Driver management |
| `horus plugin list\|enable\|disable` | Plugin management |
| `horus cache info\|clean\|purge` | Cache management |

### Deployment & Operations

| Command | Description |
|---------|-------------|
| `horus deploy <target>` | Deploy to remote robot (`--run`, `--arch`) |
| `horus launch <file>` | Launch multiple nodes from YAML |
| `horus env freeze\|restore\|show\|list` | Environment reproducibility |
| `horus record list\|play\|info` | Recording/replay management |
| `horus net check\|ping\|scan` | Network diagnostics |
| `horus sim [--2d]` | Launch 2D or 3D simulator |
| `horus auth login\|logout\|whoami` | Registry authentication |

### Examples

```bash
# Create and run a Rust project
horus new my_robot -r && cd my_robot && horus run --release

# Create Python project
horus new sensor_node -p && cd sensor_node && horus run

# Monitor running system
horus monitor          # Web UI
horus monitor -t       # Terminal UI

# Package management
horus add lidar-driver           # Auto-detect and install
horus pkg install vision-toolkit # Install from registry
horus pkg list -a                # List all packages

# Environment reproducibility
horus env freeze --publish       # Freeze and share
horus env restore a3f9c2b7      # Restore exact environment

# Deploy to robot
horus deploy pi@192.168.1.50 --run --arch aarch64

# Topic debugging
horus topic list                 # List active topics
horus topic echo sensor_data     # Echo topic messages
```

## Configuration

### Environment Variables

- `HORUS_REGISTRY_URL` - Registry endpoint (default: https://horus-marketplace-api.onrender.com)
- `HORUS_API_KEY` - CLI authentication token

### Auto-Detection

`horus run` automatically detects:
- Rust projects (Cargo.toml)
- Python files (.py)
- Multiple files with glob patterns (`horus run "nodes/*.py"`)

## Project Structure

```
horus_manager/
-- src/
   -- main.rs              # CLI entry point (clap-based)
   -- commands/            # Command implementations
      -- mod.rs
      -- new.rs           # Project creation
      -- run.rs           # Build and execution
      -- github_auth.rs   # Authentication
      -- monitor.rs       # System monitoring
   -- monitor.rs           # Web monitor (Axum)
   -- monitor_tui.rs       # Terminal UI monitor (ratatui)
   -- registry.rs          # Package registry client
   -- workspace.rs         # Workspace detection
   -- security.rs          # Security and auth utilities
-- Cargo.toml
```

## License

Part of the HORUS robotics framework. Apache License 2.0.
