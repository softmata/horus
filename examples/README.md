# Horus Examples

Five complete robotics applications, ordered from simple to advanced. Each example includes a controller (`main.rs`), a robot definition (URDF), and a simulation world (YAML).

| Example | What you'll learn |
|---------|-------------------|
| [differential_drive](differential_drive/) | Nodes, topics, messages, scheduler basics |
| [robot_arm](robot_arm/) | Services, frame transforms (TransformFrame), trajectory control |
| [sensor_navigation](sensor_navigation/) | Multi-rate scheduling, sensor pipelines, reactive control |
| [multi_robot](multi_robot/) | Namespaced topics, launch files, fleet coordination |
| [quadruped](quadruped/) | Real-time nodes, budget budgets, deadlines, IMU feedback |

## Prerequisites

- [Horus](https://github.com/softmata/horus) installed (`horus` CLI available)
- [sim3d](https://github.com/softmata/horus-sim3d) installed (for physics simulation)

## Quick start

Every example follows the same workflow:

```bash
# Terminal 1: start the simulator
sim3d --mode visual --robot robots/<robot>.urdf --world worlds/<world>.yaml --robot-name <name>

# Terminal 2: run the controller
horus run main.rs
```

While running, use the CLI to inspect the system:

```bash
horus topic list          # see active topics
horus topic echo <topic>  # watch live messages
horus node list           # see running nodes
horus monitor -t          # TUI dashboard
```

## File structure

Each example contains:

```
example_name/
  main.rs              # Controller application
  horus.toml           # Project manifest
  robots/*.urdf        # Robot description (links, joints, sensors)
  worlds/*.yaml        # Environment (ground, obstacles, lighting)
  launch/*.yaml        # Launch configuration (multi_robot only)
  README.md            # Walkthrough and expected behavior
```
