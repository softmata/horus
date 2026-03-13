<p align="center">
  <h1 align="center">HORUS</h1>
  <p align="center"><strong>The robotics framework for the AI era.</strong></p>
</p>

<p align="center">
  <a href="https://github.com/softmata/horus/releases"><img src="https://img.shields.io/badge/v0.1.9-blue.svg" alt="Version"></a>
  <a href="https://www.rust-lang.org/"><img src="https://img.shields.io/badge/rust-%3E%3D1.92-orange.svg?logo=rust" alt="Rust"></a>
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/python-%3E%3D3.9-blue.svg?logo=python&logoColor=white" alt="Python"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-green.svg" alt="License"></a>
  <a href="https://docs.horusrobotics.dev/getting-started/installation"><img src="https://img.shields.io/endpoint?url=https://telemetry.horusrobotics.dev/count/badge" alt="Installations"></a>
  <a href="https://discord.gg/hEZC3ev2Nf"><img src="https://img.shields.io/badge/Discord-Join-7289da?logo=discord&logoColor=white" alt="Discord"></a>
</p>

<p align="center">
  <a href="https://docs.horusrobotics.dev"><strong>Docs</strong></a> &middot;
  <a href="https://docs.horusrobotics.dev/getting-started/installation"><strong>Install</strong></a> &middot;
  <a href="https://docs.horusrobotics.dev/performance/performance"><strong>Benchmarks</strong></a> &middot;
  <a href="https://discord.gg/hEZC3ev2Nf"><strong>Discord</strong></a>
</p>

---

HORUS is a robotics framework built in Rust. Sub-microsecond IPC, zero-copy tensor sharing, first-class Python bindings, and a deterministic scheduler that runs perception-to-action pipelines at 100kHz. Write your control loops in Rust, your ML in Python, and let them talk at nanosecond speed.

```bash
horus new my_robot && cd my_robot && horus run
```

## 3ns IPC. Not a typo.

HORUS doesn't use a middleware layer. Nodes on the same thread share data in **3 nanoseconds**. Cross-process hits **50-167ns**. For comparison, ROS2 DDS starts at 50-500 *micro*seconds.

| Scenario | HORUS | ROS2 |
|----------|-------|------|
| Same thread | ~3 ns | N/A |
| Same process | 18-36 ns | ~50 us |
| Cross process | 50-167 ns | 50-500 us |
| Large data (images) | Zero-copy (168-byte descriptor) | Full serialization |

The secret: mmap-backed ring buffers with seqlock synchronization. No serialization, no copies, no allocations on the hot path.

## Built for AI Robotics

Most robotics frameworks were designed before the deep learning era. HORUS was built knowing that modern robots run neural networks.

**Zero-copy tensors** — Images, point clouds, and depth maps live in shared memory pools. Only a 168-byte descriptor crosses IPC. Your camera node and your ML inference node see the exact same memory.

```rust
// Camera node: write image to shared memory
let mut img = Image::new(640, 480, ImageEncoding::Rgb8)?;
topic.send(&img);  // sends 168 bytes, not 921,600

// ML node: read it — zero copies
if let Some(frame) = topic.recv() {
    let pixels = frame.data();  // direct pointer to shared memory
}
```

**DLPack protocol** — Hand tensors directly to PyTorch, JAX, or TensorFlow without copying. The standard used by the entire ML ecosystem.

**GPU-aware scheduling** — The `AsyncIo` execution class runs inference on a separate thread pool. Your neural network never blocks your 1kHz control loop.

```rust
scheduler.add(camera).order(0).build();                      // capture
scheduler.add(detector).order(1).async_io().build();         // GPU inference (non-blocking)
scheduler.add(controller).order(2).budget(200.us()).build(); // real-time control
```

**ML message types** — `Detection`, `BoundingBox2D`, `SegmentationMask` are built-in. No custom serialization needed.

**Python for ML, Rust for control** — Run PyTorch inference in a Python node, feed results to a Rust control node at 1kHz. Same shared memory, same topics.

```python
from horus import Node, Topic, Scheduler

class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.camera = Topic("camera.rgb")
        self.detections = Topic("detections")

    def tick(self, info=None):
        if frame := self.camera.recv():
            result = model(frame)           # PyTorch inference
            self.detections.send(result)

scheduler = Scheduler()
scheduler.node(Detector()).rate(30.hz()).build()
scheduler.run()
```

## Deterministic by Default

The scheduler guarantees execution order. Set `.order(0)` on your sensor, `.order(1)` on your controller — sensors always run first. No race conditions, no surprise reorderings.

Five execution classes let you put each node where it belongs:

| Class | What it does | Use case |
|-------|-------------|----------|
| **RT** | Spin-wait, auto-detected from budget/deadline | Motor control, safety |
| **Compute** | Parallel via crossbeam | Path planning, SLAM |
| **Event** | Triggers on topic data | Alert handlers |
| **AsyncIo** | Separate thread pool | GPU inference, network |
| **BestEffort** | Sequential, main thread | Logging, telemetry |

RT is automatic — set a budget or deadline and the scheduler handles the rest:

```rust
scheduler.add(motor)
    .order(0)
    .budget(200.us())       // auto-RT
    .on_miss(Miss::Skip)    // skip tick on overrun
    .build();
```

## Complete Example

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
            self.reading.send(SensorReading { position: self.position, velocity: 0.5 }).ok();
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
        .with_name("motor_control")
        .tick_rate(1000.0.hz());

    scheduler.add(SensorNode::new()?).order(0).build();
    scheduler.add(ControllerNode::new()?).order(1).build();

    scheduler.run()
}
```

## What's Included

**Messages** — 30+ standard robotics types out of the box:

| Category | Types |
|----------|-------|
| Geometry | `Pose2D`, `Pose3D`, `Twist`, `Quaternion`, `TransformStamped` |
| Sensors | `Imu`, `LaserScan`, `BatteryState`, `NavSatFix`, `Odometry` |
| Vision | `Image`, `CameraInfo`, `Detection`, `BoundingBox2D` |
| Navigation | `Path`, `OccupancyGrid`, `Goal` |
| Control | `CmdVel`, `MotorCommand`, `JointState`, `ServoCommand` |
| AI/ML | `Detection`, `BoundingBox2D`, `SegmentationMask` |

**Transforms** — coordinate frame management via `TransformFrame`:
```rust
let tf = TransformFrame::new();
hf.add_frame("laser").parent("base_link")
    .static_transform(&Transform::translation(0.2, 0.0, 0.1))
    .build()?;
let tf = hf.query("base_link").to("laser").lookup()?;
```

**CLI** — project management, monitoring, deployment:
```bash
horus new my_robot          # scaffold a project
horus run                   # build and run
horus monitor               # web UI + TUI system monitor
horus topic list            # inspect live topics
horus deploy <target>       # deploy to robot
```

## Installation

```bash
# Prerequisites (Ubuntu/Debian)
sudo apt install build-essential pkg-config libudev-dev libssl-dev libasound2-dev

# Install HORUS
git clone https://github.com/softmata/horus.git
cd horus
./install.sh
horus --version
```

**Python bindings** (optional): `pip install horus-robotics`

Requires **Linux**, **Rust 1.92+**, and optionally **Python 3.9+**.

## Examples

The [`examples/`](examples/) directory contains full working projects:

- **[differential_drive](examples/differential_drive/)** — 2-wheel robot with odometry
- **[robot_arm](examples/robot_arm/)** — Joint control with inverse kinematics
- **[quadruped](examples/quadruped/)** — 4-legged walking robot
- **[sensor_navigation](examples/sensor_navigation/)** — LiDAR-based obstacle avoidance
- **[multi_robot](examples/multi_robot/)** — Fleet coordination

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). PRs target the `dev` branch.

## License

Apache-2.0 — see [LICENSE](LICENSE).
