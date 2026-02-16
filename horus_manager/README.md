# HORUS Manager

The command-line tool for the HORUS robotics framework.

## Installation

```bash
cd horus && ./install.sh
```

## Commands

### Project Workflow

```bash
horus new my_robot              # Create project (-r Rust, -p Python, -m macro)
horus run                       # Build and run (--release for optimized)
horus build                     # Build without running
horus test                      # Run tests (--sim, --integration)
horus check                     # Validate workspace or source files
horus clean                     # Clean build artifacts (--shm for shared memory)
```

### Monitoring & Debugging

```bash
horus monitor                   # Web UI system monitor (-t for terminal UI)
horus topic list|echo|pub       # Topic interaction
horus node list|info|kill       # Node management
horus param get|set|list        # Runtime parameters
horus hf list|echo|tree         # Coordinate transforms
horus msg list|show|fields      # Message type inspection
horus log [node]                # View logs (-f follow, -l level)
```

### Packages & Deployment

```bash
horus pkg install|remove|list   # Package management
horus add <name>                # Smart add (auto-detects type)
horus remove <name>             # Smart remove
horus deploy <target>           # Deploy to remote robot (--run, --arch)
horus launch <file>             # Launch nodes from YAML
horus sim                       # Launch simulator (--2d for 2D mode)
```

### Other

```bash
horus driver list|info|search   # Driver management
horus plugin list|enable|disable # Plugin management
horus env freeze|restore        # Environment reproducibility
horus record list|play|info     # Recording/replay
horus net check|ping|scan       # Network diagnostics
horus cache info|clean          # Cache management
horus auth login|logout|whoami  # Registry authentication
```

## Examples

```bash
# Create and run a Rust project
horus new my_robot -r && cd my_robot && horus run --release

# Create Python project
horus new sensor_node -p && cd sensor_node && horus run

# Monitor running system
horus monitor          # Web UI
horus monitor -t       # Terminal UI

# Install packages
horus add lidar-driver
horus pkg install vision-toolkit

# Deploy to robot
horus deploy pi@192.168.1.50 --run --arch aarch64
```

## Configuration

- `HORUS_REGISTRY_URL` - Registry endpoint (default: https://horus-marketplace-api.onrender.com)
- `HORUS_API_KEY` - CLI authentication token

`horus run` auto-detects Rust projects (Cargo.toml), Python files (.py), and glob patterns (`horus run "nodes/*.py"`).

## License

Part of the HORUS robotics framework. Apache License 2.0.
