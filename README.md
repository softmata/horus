<p align="center">
  <h1 align="center">HORUS</h1>
  <p align="center"><strong>A deterministic real-time robotics framework in Rust.</strong></p>
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

HORUS is a robotics framework written in Rust — **575x faster than ROS2** for same-machine IPC. It replaces the DDS middleware layer with mmap-backed ring buffers and seqlock synchronization, achieving sub-microsecond cross-process messaging. Write control loops in Rust, run AI/ML inference in Python — they communicate over shared memory with zero serialization and zero copying.

Built for the era of AI-powered robots: run PyTorch, TensorFlow, or LLM inference nodes alongside hard real-time motor control at 1kHz, all in one process with deterministic scheduling.

> **Status:** HORUS is in active development (v0.1.9). The core API is stabilizing but may have breaking changes between minor versions.

```bash
horus new my_robot && cd my_robot && horus run
```

## How It Works

Nodes communicate through lock-free ring buffers. The runtime auto-selects from 10 backends based on topology — same-process or cross-process — without changing your code.

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

## Performance

Measured on Intel i7-10750H (6C/12T), 100K iterations, RDTSC timing with Tukey IQR outlier removal and bootstrap 95% CIs. Full methodology and reproduction steps in [`benchmarks/`](benchmarks/).

**Intra-process** (heap ring buffers):

| Topology | Backend | p50 | p99 |
|----------|---------|-----|-----|
| 1 pub, N sub | SpmcIntra | 3 ns | 10 ns |
| 1 pub, 1 sub | SpscIntra | 10 ns | 19 ns |
| N pub, N sub | MpmcIntra | 100 ns | 643 ns |

**Cross-process** (shared memory, RDTSC-in-payload one-way):

| Topology | Backend | p50 | p99 | Framework overhead |
|----------|---------|-----|-----|-------------------|
| 1 pub, 2 sub | SpmcShm | 198 ns | 298 ns | 31 ns |
| 1 pub, 8 sub | SpmcShm | 276 ns | 510 ns | 31 ns |
| 4 pub, 4 sub | PodShm | 304 ns | 1.5 µs | — |
| Hardware floor | Raw atomic | 167 ns | 319 ns | 0 ns |

For context, ROS2 DDS cross-process latency is typically 50–500 µs depending on the DDS vendor and message size.

```bash
# Reproduce
cargo run --release -p horus_benchmarks --bin all_paths_latency
```

## Features

### Deterministic Scheduling

The scheduler guarantees execution order across five classes:

| Class | Behavior | Use case |
|-------|----------|----------|
| **RT** | Dedicated thread, auto-detected from `.rate()` / `.budget()` / `.deadline()` | Motor control, safety |
| **Compute** | Parallel thread pool | Path planning, SLAM |
| **Event** | Triggers on topic data | Alert handlers |
| **AsyncIo** | Tokio blocking pool | GPU inference, network I/O |
| **BestEffort** | Sequential, main thread | Logging, telemetry |

RT is automatic — set a rate, budget, or deadline and the scheduler derives the rest:

```rust
scheduler.add(motor).order(0).rate(1000.hz()).on_miss(Miss::Skip).build();
scheduler.add(planner).order(1).compute().build();
scheduler.add(detector).order(2).async_io().build();
```

### Zero-Copy Data

Images, point clouds, and depth maps live in shared memory pools. Only a 168-byte tensor descriptor crosses IPC. DLPack enables direct handoff to PyTorch, JAX, or TensorFlow without copying.

```rust
let mut img = Image::new(640, 480, ImageEncoding::Rgb8)?;
topic.send(&img);  // sends 168 bytes, not 921,600

if let Some(frame) = topic.recv() {
    let pixels = frame.data();  // pointer into shared memory
}
```

### Python Bindings & AI Integration

PyO3-based bindings share the same topics and memory as Rust nodes. Run AI/ML models (PyTorch, TensorFlow, JAX) alongside real-time control without IPC overhead:

```python
from horus import Node, Topic, Scheduler

class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.camera = Topic("camera.rgb")
        self.detections = Topic("detections")

    def tick(self):
        if frame := self.camera.recv():
            result = model(frame)           # PyTorch inference
            self.detections.send(result)

scheduler = Scheduler()
scheduler.node(Detector()).rate(30.0).build()
scheduler.run()
```

### Safety Monitoring

Graduated watchdog with four severity levels (Ok → Warning → Expired → Critical), per-node health tracking (Healthy → Warning → Unhealthy → Isolated), budget/deadline enforcement, and a blackbox flight recorder for post-mortem analysis.

### Coordinate Transforms

Lock-free `TransformFrame` with f64 precision, time-travel queries via ring buffer history, and SLERP interpolation:

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

### 40+ Built-In Message Types

| Category | Types |
|----------|-------|
| Geometry | `Pose2D`, `Pose3D`, `Twist`, `Quaternion`, `TransformStamped` |
| Sensors | `Imu`, `LaserScan`, `BatteryState`, `NavSatFix`, `Odometry` |
| Vision | `Image`, `CameraInfo`, `Detection`, `BoundingBox2D`, `SegmentationMask` |
| Navigation | `Path`, `OccupancyGrid`, `NavGoal`, `CostMap` |
| Control | `CmdVel`, `MotorCommand`, `JointState`, `ServoCommand`, `TrajectoryPoint` |
| Diagnostics | `Heartbeat`, `SafetyStatus`, `EmergencyStop`, `ResourceUsage` |

### CLI

```bash
horus new my_robot                    # scaffold a project
horus run                             # build and run
horus topic list                      # inspect live topics
horus topic echo camera.rgb           # watch messages
horus node list                       # running nodes and rates
horus monitor -t                      # TUI system dashboard
horus blackbox -a                     # timing anomalies
horus param set max_speed 0.5         # tune at runtime
horus frame tree                      # coordinate frame hierarchy
horus deploy pi@192.168.1.50          # deploy to robot
```

### Single Manifest

`horus.toml` is the single source of truth — it replaces Cargo.toml, package.xml, and launch files. Native build files are generated into `.horus/` automatically.

## Examples

The [`examples/`](examples/) directory contains five working projects:

| Example | What it demonstrates |
|---------|---------------------|
| [differential_drive](examples/differential_drive/) | Nodes, topics, messages, multi-rate scheduling |
| [robot_arm](examples/robot_arm/) | Services, transform frames, 6-DOF trajectory control |
| [sensor_navigation](examples/sensor_navigation/) | Multi-rate pipelines, LaserScan processing, reactive control |
| [multi_robot](examples/multi_robot/) | Namespaced topics, launch files, fleet coordination |
| [quadruped](examples/quadruped/) | RT scheduling at 200Hz, 12-DOF gait generation, IMU feedback |

## Installation

Requires Linux and Rust 1.92+. Python 3.9+ optional for bindings.

```bash
sudo apt install build-essential pkg-config libudev-dev libssl-dev libasound2-dev

git clone https://github.com/softmata/horus.git
cd horus
./install.sh
horus --version
```

Python bindings: `pip install horus-robotics`

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). PRs target the `dev` branch.

## License

Apache-2.0 — see [LICENSE](LICENSE).
