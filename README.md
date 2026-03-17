<p align="center">
  <h1 align="center">HORUS — Real-Time AI Robotics Framework for Rust & Python</h1>
  <p align="center"><strong>575x faster than ROS2. Zero-copy shared memory. Deterministic scheduling.<br>Run PyTorch and real-time motor control in one process with sub-microsecond IPC.</strong></p>
</p>

<p align="center">
  <a href="https://github.com/softmata/horus/actions/workflows/ci.yml"><img src="https://github.com/softmata/horus/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
  <a href="https://github.com/softmata/horus/releases"><img src="https://img.shields.io/badge/v0.1.9-blue.svg" alt="Version"></a>
  <a href="https://www.rust-lang.org/"><img src="https://img.shields.io/badge/rust-%3E%3D1.92-orange.svg?logo=rust" alt="Rust"></a>
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/python-%3E%3D3.9-blue.svg?logo=python&logoColor=white" alt="Python"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-green.svg" alt="License"></a>
  <a href="https://discord.gg/hEZC3ev2Nf"><img src="https://img.shields.io/badge/Discord-Join-7289da?logo=discord&logoColor=white" alt="Discord"></a>
</p>

<p align="center">
  <a href="https://docs.horusrobotics.dev"><strong>Docs</strong></a> &middot;
  <a href="https://docs.horusrobotics.dev/getting-started/installation"><strong>Install</strong></a> &middot;
  <a href="https://docs.horusrobotics.dev/performance/performance"><strong>Benchmarks</strong></a> &middot;
  <a href="https://discord.gg/hEZC3ev2Nf"><strong>Discord</strong></a>
</p>

---

## What is HORUS?

**HORUS** (**H**ybrid **O**ptimized **R**obotics **U**nified **S**ystem) is a **robotics middleware** — a modern alternative to ROS2 that replaces DDS with mmap-backed ring buffers and lock-free synchronization. Write motor controllers in Rust at 1kHz, run YOLO or LLM inference in Python, and connect them over shared memory with **zero serialization and zero copying**.

```bash
horus new my_robot && cd my_robot && horus run
```

### Why developers choose HORUS over ROS2

| | **HORUS** | **ROS2** |
|---|---|---|
| Cross-process latency | **198 ns** (shared memory) | 50–500 µs (DDS) |
| GPU tensor transfer | **DLPack zero-copy** (PyTorch/JAX native) | Serialize → deserialize |
| AI + RT in one process | Yes — AsyncIo for GPU, RT for motors | Separate processes, DDS overhead |
| Perception types | 8 built-in (Detection, Tracking, Segmentation) | Define your own messages |
| Scheduling | Deterministic, 5 execution classes | Best-effort |
| RT support | Built-in (budget, deadline, watchdog) | Requires external RTOS |
| Languages | Rust + Python (same shared memory) | C++ + Python (separate serialization) |
| Config | Single `horus.toml` | package.xml + CMakeLists.txt + launch files |
| Setup | `horus new && horus run` | colcon build + source install + launch |

> **Status:** Active development (v0.1.9). Core API stabilizing. [Join the Discord](https://discord.gg/hEZC3ev2Nf) for updates.

---

## Quick Start

```rust
use horus::prelude::*;

message! {
    SensorReading { position: f64, velocity: f64 }
}

message! {
    MotorCommand { voltage: f64 }
}

node! {
    SensorNode {
        pub { reading: SensorReading -> "sensor.data" }
        data { position: f64 = 0.0 }

        tick {
            self.position += 0.01;
            self.reading.send(SensorReading {
                position: self.position, velocity: 0.5
            }).ok();
        }
    }
}

node! {
    ControllerNode {
        sub { sensor: SensorReading -> "sensor.data" }
        pub { command: MotorCommand -> "motor.cmd" }
        data { target: f64 = 1.0 }

        tick {
            if let Some(reading) = self.sensor.recv() {
                let error = self.target - reading.position;
                self.command.send(MotorCommand { voltage: error * 0.5 }).ok();
            }
        }
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new()
        .name("motor_control")
        .tick_rate(1000.hz());

    scheduler.add(SensorNode::new()?).order(0).build();
    scheduler.add(ControllerNode::new()?).order(1).build();

    scheduler.run()
}
```

Or in **Python**:

```python
import horus

def controller_tick(node):
    if node.has_msg("sensor"):
        reading = node.get("sensor")
        error = 1.0 - reading.position
        node.send("motor", horus.MotorCommand(voltage=error * 0.5))

sensor = horus.Node(name="sensor", pubs=["sensor.data"], tick=sensor_tick, rate=1000)
controller = horus.Node(name="ctrl", subs=["sensor.data"], pubs=["motor.cmd"], tick=controller_tick, rate=1000)
horus.run(sensor, controller)
```

Both share the same topics over the same shared memory — zero overhead between languages.

---

## Performance

Measured on Intel i7-10750H, 100K iterations, RDTSC timing with Tukey IQR outlier removal and bootstrap 95% CIs. Full methodology in [`benchmarks/`](benchmarks/).

**Same-process IPC** (lock-free ring buffers):

| Topology | p50 | p99 |
|----------|-----|-----|
| 1 pub → N sub | **3 ns** | 10 ns |
| 1 pub → 1 sub | 10 ns | 19 ns |
| N pub → N sub | 100 ns | 643 ns |

**Cross-process IPC** (shared memory):

| Topology | p50 | p99 |
|----------|-----|-----|
| 1 pub → 2 sub | **198 ns** | 298 ns |
| 1 pub → 8 sub | 276 ns | 510 ns |
| 4 pub → 4 sub | 304 ns | 1.5 µs |

ROS2 DDS typically measures 50–500 µs for the same workloads.

```bash
cargo run --release -p horus_benchmarks --bin all_paths_latency
```

---

## Key Features

### Deterministic Real-Time Scheduling

Five execution classes — the scheduler auto-selects based on your configuration:

```rust
// RT — 1kHz motor control with deadline enforcement
scheduler.add(motor).order(0).rate(1000.hz()).on_miss(Miss::SafeMode).build();

// Compute — path planning on thread pool (doesn't block RT)
scheduler.add(planner).order(1).compute().build();

// Event — fires only when emergency_stop topic updates
scheduler.add(estop).order(0).on("emergency_stop").build();

// AsyncIo — GPU inference without blocking anything
scheduler.add(detector).order(2).async_io().build();

// BestEffort — logging, telemetry (default)
scheduler.add(logger).build();
```

RT is automatic — set `.rate()`, `.budget()`, or `.deadline()` and the scheduler derives timing constraints. No manual thread management.

### Zero-Copy for Images, Point Clouds & Tensors

Large data lives in shared memory pools. Only a 168-byte descriptor crosses IPC. DLPack enables direct GPU tensor handoff to PyTorch, JAX, or TensorFlow.

```rust
// Camera frame: 1080p RGB = 6.2 MB — but only 168 bytes cross IPC
let mut img = Image::new(1920, 1080, ImageEncoding::Rgb8)?;
topic.send(&img);  // descriptor only, pixels stay in shared memory

// Receiver gets a pointer to the same memory pages
if let Some(frame) = topic.recv() {
    let pixels = frame.data();  // zero-copy access
}
```

### AI & Perception Pipeline

Built for the era of AI-powered robots. Run camera → detection → tracking → motor control with **zero-copy from sensor to GPU to actuator**.

**The pipeline:**

```
Camera (30fps)  →  YOLO Detection (GPU)  →  Tracking (CPU)  →  Motor Control (1kHz)
   [AsyncIo]          [Compute]              [BestEffort]          [RT]
  zero-copy SHM      PyTorch/DLPack        TrackedObject Pod     200µs budget
```

**What makes this fast:**
- **4K camera frames** stay in shared memory — only a 168-byte descriptor crosses IPC
- **DLPack** hands GPU tensors directly to PyTorch/JAX/TensorFlow — no CPU round-trip
- **Detection results** are 72-byte Pod structs with built-in `.iou()` for NMS
- **Tracking** uses `TrackedObject` with Kalman state (velocity, age, hits, state machine)
- **Motor controller runs on a separate RT thread** — YOLO taking 200ms doesn't affect 1kHz control

**Python AI node — shares memory with Rust nodes:**

```python
import horus
import torch

def detector_tick(node):
    if node.has_msg("camera"):
        frame = node.get("camera")
        tensor = torch.from_dlpack(frame)     # zero-copy GPU transfer
        detections = model(tensor)             # YOLO inference
        for det in detections:
            node.send("detections", horus.Detection(
                x=det.x, y=det.y, width=det.w, height=det.h,
                confidence=det.conf, class_name=det.label
            ))

detector = horus.Node(name="yolo", subs=["camera"], pubs=["detections"],
                       tick=detector_tick, rate=30)
horus.run(detector)
```

**Built-in perception types** — no custom messages needed:

| Type | Size | Use Case |
|------|------|----------|
| `Detection` | 72B Pod | 2D bounding box + confidence + class (YOLO, SSD) |
| `Detection3D` | 104B Pod | 6-DOF bbox + velocity (3D object detection) |
| `TrackedObject` | 96B Pod | Track ID + Kalman state + age (DeepSORT, SORT) |
| `SegmentationMask` | 64B header | Semantic/instance/panoptic masks |
| `Landmark` | keypoint | COCO pose keypoints (17 body joints) |
| `Image` | 168B descriptor | Zero-copy camera frames (any resolution) |
| `PointCloud` | 272B descriptor | Zero-copy LiDAR/depth point clouds |
| `CameraInfo` | 232B Pod | Intrinsics, distortion, stereo calibration |

### Python Bindings

PyO3-based bindings share the same topics and shared memory as Rust nodes. Full API parity — `Node`, `Scheduler`, `Topic`, all 40+ message types, driver system.

```python
import horus

node = horus.Node(name="ctrl", subs=["sensor"], pubs=["motor"], tick=my_tick, rate=100)
horus.run(node)
```

### Safety Monitoring & Fault Tolerance

- **Graduated watchdog**: Ok → Warning → Expired → Critical
- **Per-node health tracking**: Healthy → Warning → Unhealthy → Isolated
- **Budget/deadline enforcement** with configurable miss policies (Warn, Skip, SafeMode, Stop)
- **Blackbox flight recorder** for post-mortem crash analysis
- **Failure policies**: Fatal, Restart(n, backoff), Skip(n, cooldown), Ignore

### 40+ Robotics Message Types

| Category | Types |
|----------|-------|
| Geometry | `Pose2D`, `Pose3D`, `Twist`, `Quaternion`, `TransformStamped` |
| Sensors | `Imu`, `LaserScan`, `BatteryState`, `NavSatFix`, `Odometry` |
| Vision | `Image`, `CameraInfo`, `Detection`, `BoundingBox2D`, `SegmentationMask` |
| Navigation | `Path`, `OccupancyGrid`, `NavGoal`, `CostMap` |
| Control | `CmdVel`, `MotorCommand`, `JointState`, `ServoCommand`, `TrajectoryPoint` |
| Diagnostics | `Heartbeat`, `SafetyStatus`, `EmergencyStop`, `ResourceUsage` |

All are fixed-size Pod types — zero serialization overhead.

### Lock-Free Coordinate Transforms

`TransformFrame` replaces ROS TF2 with lock-free seqlock reads (~50ns by ID), ring buffer history, and SLERP interpolation:

```rust
let tf = TransformFrame::new();
tf.add_frame("laser").parent("base_link")
    .static_transform(&Transform::translation(0.2, 0.0, 0.1))
    .build()?;
let transform = tf.query("base_link").to("laser").lookup()?;
```

### Services and Actions

Request-reply for synchronous calls, actions for long-running tasks with progress feedback and cancellation:

```rust
service! {
    AddTwoInts {
        request  { a: i64, b: i64 }
        response { sum: i64 }
    }
}

action! {
    Navigate {
        goal     { target_x: f64, target_y: f64 }
        feedback { distance_remaining: f64 }
        result   { success: bool }
    }
}
```

### Hardware Driver System

Declare hardware in `horus.toml`, access typed handles in code. Supports 30+ Terra HAL drivers (Dynamixel, RPLiDAR, RealSense, CAN, EtherCAT, and more):

```toml
[drivers.arm]
terra = "dynamixel"
port = "/dev/ttyUSB0"
baudrate = 1000000
```

```rust
let mut hw = drivers::load()?;
scheduler.add(ArmDriver::new(hw.dynamixel("arm")?))
    .rate(500.hz()).on_miss(Miss::SafeMode).build()?;
```

### CLI Tooling

```bash
horus new my_robot                    # scaffold Rust, Python, or C++ project
horus run                             # build and run
horus test                            # run tests (cargo test / pytest / ctest)
horus topic list                      # inspect live topics
horus topic echo camera.rgb           # watch messages in real-time
horus node list                       # running nodes and tick rates
horus monitor -t                      # TUI system dashboard
horus blackbox -a                     # timing anomalies
horus param set max_speed 0.5         # tune parameters at runtime
horus frame tree                      # coordinate frame hierarchy
horus doctor                          # ecosystem health check
horus deploy pi@192.168.1.50          # deploy to robot
```

### Single Manifest

`horus.toml` replaces Cargo.toml, package.xml, and CMakeLists.txt. Native build files are generated into `.horus/` automatically. One source of truth for Rust, Python, and C++ projects.

---

## Who Is HORUS For?

- **AI/ML engineers** deploying perception on robots — YOLO, LLMs, point cloud processing with zero-copy GPU transfer
- **Robotics engineers** who need 1kHz control loops with deterministic scheduling and safety monitoring
- **Teams migrating from ROS2** who want 575x faster IPC, simpler tooling, and Python that shares memory with Rust
- **Autonomous vehicle / drone teams** running camera → detection → planning → control in one process
- **Students and researchers** who want `horus new && horus run` instead of fighting colcon and CMake

---

## Examples

The [`examples/`](examples/) directory contains five working projects you can run immediately:

| Example | What it demonstrates |
|---------|---------------------|
| [differential_drive](examples/differential_drive/) | Nodes, topics, messages, multi-rate scheduling |
| [robot_arm](examples/robot_arm/) | Services, transform frames, 6-DOF trajectory control |
| [sensor_navigation](examples/sensor_navigation/) | Multi-rate pipelines, LaserScan processing, reactive control |
| [multi_robot](examples/multi_robot/) | Namespaced topics, launch files, fleet coordination |
| [quadruped](examples/quadruped/) | RT scheduling at 200Hz, 12-DOF gait generation, IMU feedback |

Quick-start tutorials in [`horus/examples/`](horus/examples/):

```bash
cargo run --example 01_hello_node
cargo run --example 02_pub_sub
cargo run --example 03_multi_rate
cargo run --example 04_services
cargo run --example 05_realtime
```

---

## Installation

Requires Linux and Rust 1.92+. Python 3.9+ optional for bindings.

```bash
sudo apt install build-essential pkg-config libudev-dev libssl-dev libasound2-dev

git clone https://github.com/softmata/horus.git
cd horus
./install.sh
horus --version
```

Python bindings:

```bash
pip install horus-robotics
```

---

## Architecture

```
horus/              Umbrella crate — prelude, macros, re-exports
horus_core/         Runtime — scheduler, nodes, topics, services, actions, safety monitor
horus_library/      Standard library — 40+ message types, TransformFrame, params
horus_manager/      CLI tool — build, run, test, deploy, monitor (46+ commands)
horus_py/           Python bindings (PyO3) — full API parity
horus_sys/          Platform HAL — Linux, macOS, Windows
benchmarks/         Performance suite — latency, throughput, jitter, DDS comparison
```

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). PRs target the `dev` branch. Join the [Discord](https://discord.gg/hEZC3ev2Nf) for discussion.

## License

Apache-2.0 — see [LICENSE](LICENSE).
