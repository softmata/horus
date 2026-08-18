# Horus Examples

Ten complete robotics applications covering every workflow a ROS2 developer needs. Ordered from simple to advanced. Rust and Python examples live here; the C++ examples are under [`horus_cpp/examples/`](../horus_cpp/examples/) — see [C++](#c) below.

## Learning Path

### Getting Started
| Example | Language | What you'll learn |
|---------|----------|-------------------|
| [differential_drive](differential_drive/) | Rust | Nodes, topics, messages, scheduler basics |
| [python_robot](python_robot/) | **Python** | Same as differential_drive — the Python starting point |

### Core Robotics
| Example                                 | Language | What you'll learn                                               |
|-----------------------------------------|----------|-----------------------------------------------------------------|
| [robot_arm](robot_arm/)                 | Rust     | Services, frame transforms (TransformFrame), trajectory control |
| [sensor_navigation](sensor_navigation/) | Rust     | Multi-rate scheduling, sensor pipelines, reactive control       |
| [camera_perception](camera_perception/) | Rust     | CV pipeline: camera → object detection → tracking (IoU, SORT)   |

### Advanced
| Example                           | Language | What you'll learn                                   |
|-----------------------------------|----------|-----------------------------------------------------|
| [multi_robot](multi_robot/)       | Rust     | Namespaced topics, launch files, fleet coordination |
| [quadruped](quadruped/)           | Rust     | Real-time nodes, budgets, deadlines, IMU feedback   |
| [pick_and_place](pick_and_place/) | Rust     | **Actions**: goal lifecycle, feedback, cancellation |

### Production
| Example                                   | Language | What you'll learn                                                  |
|-------------------------------------------|----------|--------------------------------------------------------------------|
| [driver_integration](driver_integration/) | Rust     | Hardware drivers via `[drivers]` config, Terra HAL, custom drivers |
| [record_replay](record_replay/)           | Rust     | Session recording, blackbox forensics, deterministic replay        |

## C++

The C++ examples are in [`horus_cpp/examples/`](../horus_cpp/examples/) rather
than this directory, because they build with CMake against the C++ bindings
instead of through `horus build`.

| Example | What you'll learn |
|---------|-------------------|
| [pub_sub_demo](../horus_cpp/examples/pub_sub_demo.cpp) | Topics, the loan/publish pattern, send-by-copy |
| [multi_node](../horus_cpp/examples/multi_node.cpp) | Several nodes in one scheduler, rates, budgets, `on_miss` |
| [obstacle_avoidance](../horus_cpp/examples/obstacle_avoidance.cpp) | Reactive control from a LaserScan |
| [camera_publisher](../horus_cpp/examples/camera_publisher.cpp) | Publishing sensor data at a fixed rate |
| [transform_frames](../horus_cpp/examples/transform_frames.cpp) | Coordinate frames from C++ |
| [params_demo](../horus_cpp/examples/params_demo.cpp) | Runtime parameters |

Build them with:

```bash
cargo build -p horus_cpp                      # produces libhorus_cpp.a
cmake -S horus_cpp/examples -B build/cpp-examples
cmake --build build/cpp-examples
```

CMake finds the library in `target/` automatically from a checkout; pass
`-DHORUS_CPP_LIB=...` to point it somewhere else.

## ROS2 Migration Guide

| ROS2 Workflow                 | Horus Example                          |
|-------------------------------|----------------------------------------|
| Publisher/Subscriber tutorial | `differential_drive` or `python_robot` |
| tf2 and frame transforms      | `robot_arm`                            |
| nav2 sensor pipeline          | `sensor_navigation`                    |
| darknet_ros / YOLO detection  | `camera_perception`                    |
| Multi-robot swarm             | `multi_robot`                          |
| ros2_control hardware         | `driver_integration`                   |
| MoveIt pick-and-place         | `pick_and_place`                       |
| rosbag2 record/play           | `record_replay`                        |
| Legged robot control          | `quadruped`                            |

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
horus monitor             # TUI dashboard
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
