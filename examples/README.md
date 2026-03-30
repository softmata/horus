# Horus Examples

Ten complete robotics applications covering every workflow a ROS2 developer needs. Ordered from simple to advanced, with both Rust and Python examples.

## Learning Path

### Getting Started
| Example | Language | What you'll learn |
|---------|----------|-------------------|
| [differential_drive](differential_drive/) | Rust | Nodes, topics, messages, scheduler basics |
| [python_robot](python_robot/) | **Python** | Same as differential_drive — the Python starting point |

### Core Robotics
| Example | Language | What you'll learn |
|---------|----------|-------------------|
| [robot_arm](robot_arm/) | Rust | Services, frame transforms (TransformFrame), trajectory control |
| [sensor_navigation](sensor_navigation/) | Rust | Multi-rate scheduling, sensor pipelines, reactive control |
| [camera_perception](camera_perception/) | Rust | CV pipeline: camera → object detection → tracking (IoU, SORT) |

### Advanced
| Example | Language | What you'll learn |
|---------|----------|-------------------|
| [multi_robot](multi_robot/) | Rust | Namespaced topics, launch files, fleet coordination |
| [quadruped](quadruped/) | Rust | Real-time nodes, budgets, deadlines, IMU feedback |
| [pick_and_place](pick_and_place/) | Rust | **Actions**: goal lifecycle, feedback, cancellation |

### Production
| Example | Language | What you'll learn |
|---------|----------|-------------------|
| [driver_integration](driver_integration/) | Rust | Hardware drivers via `[drivers]` config, Terra HAL, custom drivers |
| [record_replay](record_replay/) | Rust | Session recording, blackbox forensics, deterministic replay |

## ROS2 Migration Guide

| ROS2 Workflow | Horus Example |
|---------------|---------------|
| Publisher/Subscriber tutorial | `differential_drive` or `python_robot` |
| tf2 and frame transforms | `robot_arm` |
| nav2 sensor pipeline | `sensor_navigation` |
| darknet_ros / YOLO detection | `camera_perception` |
| Multi-robot swarm | `multi_robot` |
| ros2_control hardware | `driver_integration` |
| MoveIt pick-and-place | `pick_and_place` |
| rosbag2 record/play | `record_replay` |
| Legged robot control | `quadruped` |

## Prerequisites

- [Horus](https://github.com/softmata/horus) installed (`horus` CLI available)
- [sim3d](https://github.com/softmata/horus-sim3d) installed (for physics simulation)
- Python 3.9+ (for `python_robot` example)

## Quick start

Every Rust example follows the same workflow:

```bash
# Terminal 1: start the simulator (optional — examples work without sim)
sim3d --mode visual --robot robots/<robot>.urdf --world worlds/<world>.yaml

# Terminal 2: run the controller
horus run main.rs
```

For Python:
```bash
horus run main.py
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
  main.rs / main.py      # Controller application
  horus.toml              # Project manifest (deps, drivers, scripts)
  robots/*.urdf           # Robot description (links, joints, sensors)
  worlds/*.yaml           # Environment (ground, obstacles, lighting)
  launch/*.yaml           # Launch configuration (multi_robot only)
  README.md               # Walkthrough and expected behavior
```
