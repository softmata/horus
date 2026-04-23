# HORUS

**Real-time distributed middleware for Rust, Python, and C++. 575x faster than ROS2.**

[![CI](https://github.com/softmata/horus/actions/workflows/ci.yml/badge.svg)](https://github.com/softmata/horus/actions)
[![Version](https://img.shields.io/badge/v0.2.0-blue.svg)](https://github.com/softmata/horus/releases)
[![Rust](https://img.shields.io/badge/rust-%3E%3D1.92-orange.svg?logo=rust)](https://www.rust-lang.org/)
[![Python](https://img.shields.io/badge/python-%3E%3D3.9-blue.svg?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE)
[![Discord](https://img.shields.io/badge/Discord-Join-7289da?logo=discord&logoColor=white)](https://discord.gg/hEZC3ev2Nf)

[**Docs**](https://docs.horusrobotics.dev) &middot;
[**Quick Start**](https://docs.horusrobotics.dev/getting-started/quick-start) &middot;
[**Benchmarks**](https://docs.horusrobotics.dev/performance/benchmarks) &middot;
[**Coming from ROS2?**](https://docs.horusrobotics.dev/learn/coming-from-ros2) &middot;
[**Discord**](https://discord.gg/hEZC3ev2Nf)

---

## Get Started

```bash
curl -fsSL https://github.com/softmata/horus/raw/release/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

Or install manually:

```bash
git clone https://github.com/softmata/horus.git && cd horus && ./install.sh
```

Python: `pip install horus-robotics` · C++: link against `libhorus_cpp` and `#include <horus/horus.hpp>`

---

## Why HORUS?

HORUS is a real-time distributed middleware that replaces DDS with shared-memory ring buffers and lock-free synchronization. Built for any system where latency, determinism, and safety matter — robotics, industrial automation, autonomous vehicles, trading systems, game engines, and more.

| | **HORUS** | **ROS2** |
|---|---|---|
| IPC latency | **11–196 ns** (shared memory) | 50–500 µs (DDS) |
| Scheduling | Deterministic, 5 execution classes | Best-effort callbacks |
| RT support | Built-in (budget, deadline, watchdog) | Manual DDS QoS |
| Safety | Graduated watchdog, auto safe-state, BlackBox | Application-level |
| AI + RT | Same process — AsyncIo for GPU, RT for motors | Separate processes |
| GPU tensors | DLPack zero-copy (PyTorch/JAX native) | Serialize → deserialize |
| Languages | **Rust + Python + C++** (same shared memory) | C++ + Python (DDS serialization) |
| Config | Single `horus.toml` | package.xml + CMakeLists.txt + launch files |
| Setup | `horus new && horus run` | colcon build + source install + launch |

---

## Quick Start

**Rust** — 1kHz motor controller in one file:

```rust
use horus::prelude::*;

message! {
    SensorReading { position: f64, velocity: f64 }
    MotorCommand  { voltage: f64 }
}

node! {
    Sensor {
        pub { reading: SensorReading -> "sensor.data" }
        data { pos: f64 = 0.0 }
        tick {
            self.pos += 0.01;
            self.reading.send(SensorReading { position: self.pos, velocity: 0.5 });
        }
    }
}

node! {
    Controller {
        sub { sensor: SensorReading -> "sensor.data" }
        pub { cmd: MotorCommand -> "motor.cmd" }
        data { target: f64 = 1.0 }
        tick {
            if let Some(s) = self.sensor.recv() {
                self.cmd.send(MotorCommand { voltage: (self.target - s.position) * 0.5 });
            }
        }
    }
}

fn main() -> Result<()> {
    let mut sched = Scheduler::new().tick_rate(1000.hz());
    sched.add(Sensor::new()?).order(0).build()?;
    sched.add(Controller::new()?).order(1).rate(1000.hz()).on_miss(Miss::SafeMode).build()?;
    sched.run()
}
```

**Python** — same robot, 8 lines:

```python
import horus

def sensor_tick(node):
    node.send("sensor.data", {"position": sensor_tick.pos, "velocity": 0.5})
    sensor_tick.pos += 0.01
sensor_tick.pos = 0.0

def controller_tick(node):
    s = node.recv("sensor.data")
    if s is not None:
        node.send("motor.cmd", {"voltage": (1.0 - s["position"]) * 0.5})

horus.run(
    horus.Node(name="sensor", pubs=["sensor.data"], tick=sensor_tick, rate=1000),
    horus.Node(name="ctrl", subs=["sensor.data"], pubs=["motor.cmd"], tick=controller_tick, rate=1000),
)
```

**C++** — same robot, idiomatic API:

```cpp
#include <horus/horus.hpp>
using namespace horus::literals;

// Struct-based node with built-in pub/sub (like Rust's impl Node)
class Controller : public horus::Node {
public:
    Controller() : Node("controller") {
        sensor_ = subscribe<horus::msg::CmdVel>("sensor.data");
        motor_  = advertise<horus::msg::CmdVel>("motor.cmd");
    }

    void tick() override {
        auto s = sensor_->recv();
        if (!s) return;
        horus::msg::CmdVel cmd{};
        cmd.linear = (1.0f - s->get()->linear) * 0.5f;
        motor_->send(cmd);
    }

    void enter_safe_state() override { /* stop motors */ }

private:
    horus::Subscriber<horus::msg::CmdVel>* sensor_;
    horus::Publisher<horus::msg::CmdVel>*  motor_;
};

int main() {
    horus::Scheduler sched;
    sched.tick_rate(1000_hz);

    horus::Publisher<horus::msg::CmdVel> sensor_pub("sensor.data");

    sched.add("sensor").order(0)
        .tick([&] {
            auto out = sensor_pub.loan();
            out->linear = 0.5f;
            sensor_pub.publish(std::move(out));
        }).build();

    Controller ctrl;
    sched.add(ctrl).order(1).on_miss(horus::Miss::SafeMode).build();

    sched.spin();
}
```

All three languages share the same topics over shared memory — zero overhead between Rust, Python, and C++.

---

## Features

### Deterministic Scheduling

Five execution classes — the scheduler auto-selects based on your configuration:

```rust
sched.add(motor).order(0).rate(1000.hz()).on_miss(Miss::SafeMode).build()?;  // RT
sched.add(planner).compute().build()?;                                        // Thread pool
sched.add(estop).on("emergency.stop").build()?;                               // Event-driven
sched.add(detector).async_io().build()?;                                      // GPU / network I/O
sched.add(logger).build()?;                                                   // Best-effort
```

Set `.rate()`, `.budget()`, or `.deadline()` and RT is automatic — no manual thread management. [Learn more →](https://docs.horusrobotics.dev/concepts/execution-classes)

### Safety

The scheduler monitors every node at runtime:

- **Graduated watchdog** — warn → skip → isolate → safe state
- **Deadline enforcement** — `.budget()` and `.deadline()` with miss policies (Warn, Skip, SafeMode, Stop)
- **`enter_safe_state()`** — you define what "safe" means per node (stop motors, close valves)
- **BlackBox flight recorder** — ring-buffer event log for post-mortem crash analysis
- **Fault tolerance** — per-node failure policies (restart with backoff, skip, fatal)

[Safety Monitor →](https://docs.horusrobotics.dev/advanced/safety-monitor) · [BlackBox →](https://docs.horusrobotics.dev/advanced/blackbox) · [Fault Tolerance →](https://docs.horusrobotics.dev/advanced/circuit-breaker)

### Zero-Copy AI Pipeline

Run camera → YOLO → tracking → motor control in one process. 4K frames stay in shared memory; DLPack hands GPU tensors directly to PyTorch.

```python
def detector_tick(node):
    frame = node.recv("camera")
    if frame is not None:
        tensor = torch.from_dlpack(frame)        # zero-copy GPU transfer
        for det in model(tensor):
            node.send("detections", horus.Detection(
                x=det.x, y=det.y, width=det.w, height=det.h,
                confidence=det.conf, class_name=det.label
            ))
```

8 built-in perception types: `Detection`, `Detection3D`, `TrackedObject`, `SegmentationMask`, `Landmark`, `Image`, `PointCloud`, `CameraInfo`. [Learn more →](https://docs.horusrobotics.dev/concepts/message-types)

### 40+ Message Types · Services · Actions · Transforms

Everything you need for robotics, built-in:

```rust
// 40+ message types — all zero-copy Pod structs
let imu: Topic<Imu> = Topic::new("imu")?;
let cmd: Topic<CmdVel> = Topic::new("cmd_vel")?;

// Lock-free coordinate transforms (10-33x faster than ROS TF2)
let tf = TransformFrame::new();
tf.add_frame("laser").parent("base_link")
    .static_transform(&Transform::from_translation([0.2, 0.0, 0.1]))
    .build()?;

// Services (request/response) and Actions (long-running with feedback)
service! { AddTwoInts { request { a: i64, b: i64 } response { sum: i64 } } }
action!  { Navigate { goal { x: f64, y: f64 } feedback { dist: f64 } result { ok: bool } } }
```

### Hardware Drivers

Declare hardware in `horus.toml`, access typed handles in code. 30+ [Terra HAL](https://github.com/softmata/terra) drivers — Dynamixel, RPLiDAR, RealSense, CAN, EtherCAT, and more.

```toml
[drivers.arm]
terra = "dynamixel"
port = "/dev/ttyUSB0"
baudrate = 1000000
```

### CLI

```bash
horus new my_robot              # scaffold project (Rust, Python, or C++)
horus new my_bot --lang cpp     # scaffold C++ project
horus run                       # build and run
horus topic list                # inspect live topics
horus topic echo camera.rgb     # watch messages (works across all languages)
horus monitor -t                # TUI system dashboard
horus deploy pi@192.168.1.50    # deploy to robot
horus doctor                    # ecosystem health check
```

40+ commands. [Full CLI reference →](https://docs.horusrobotics.dev/development/cli-reference)

---

## Performance

Measured with RDTSC cycle counting, Tukey IQR outlier filtering, bootstrap 95% CIs on Intel i9-14900K. [Full methodology →](benchmarks/)

| Topology | HORUS | ROS2 (DDS) |
|----------|-------|------------|
| Same-process pub/sub | **91 ns** | ~50 µs (**550x**) |
| Cross-process | **171 ns** | ~100 µs (**585x**) |
| 1 pub → 3 subs | **80 ns** | ~70 µs (**875x**) |

| vs iceoryx2 | HORUS | iceoryx2 | Speedup |
|-------------|-------|----------|---------|
| Same-thread | 11 ns | 69 ns | **6.3x** |
| Cross-process | 170 ns | 361 ns | **2.1x** |
| Throughput | 95 M msg/s | 22 M msg/s | **4.3x** |

Scales near-linearly to 100 nodes (14% degradation) and O(1) to 1,000 topics.

```bash
cargo run --release -p horus_benchmarks --bin all_paths_latency    # run it yourself
```

---

## Examples

10 working projects in [`examples/`](examples/) — from differential drive to quadruped gait generation:

```bash
cargo run --example 01_hello_node     # your first node
cargo run --example 02_pub_sub        # topics and messages
cargo run --example 03_multi_rate     # multi-rate scheduling
cargo run --example 04_services       # request/response
cargo run --example 05_realtime       # RT with deadline enforcement
```

[Full examples →](examples/) · [Tutorials →](https://docs.horusrobotics.dev/tutorials) · [Recipes →](https://docs.horusrobotics.dev/recipes)

---

## Coming Soon

- **Embedded HORUS** — `no_std` runtime for STM32, ESP32, and other microcontrollers
- **HORUS–Zenoh Bridge** — Distributed multi-machine deployments over Zenoh for seamless cloud-edge-robot communication
- **ROS2 Bridge** — Bidirectional topic bridging between HORUS and ROS2

---

## Architecture

```
horus/          Umbrella crate — prelude, universal types
horus_core/     Runtime — scheduler, nodes, topics, services, actions, safety monitor
horus_types/    Universal IPC types — math, diagnostics, time, generic
horus_cpp/      C++ bindings — extern "C" FFI, idiomatic C++17 headers (pool, params, TF, services, actions)
horus_py/       Python bindings (PyO3)
horus_manager/  CLI — build, run, test, deploy, monitor (40+ commands)
horus_sys/      Platform HAL — Linux, macOS
horus_net/      LAN replication — transparent cross-machine topics

# Separate packages (install via `horus install`):
# horus-tf         Coordinate frame transforms (lock-free, 10-33x faster than ROS2 TF2)
# horus-robotics   Standard robotics message types (CmdVel, Imu, LaserScan, 45+ types)
benchmarks/     Performance suite — latency, throughput, jitter, comparisons
```

---

## Running HORUS on Real Hardware?

We'd love to hear from you. HORUS is validated in simulation — if you're running it on a real robot, your experience helps us improve.

Tell us (via [GitHub Issues](https://github.com/softmata/horus/issues), [Discord](https://discord.gg/hEZC3ev2Nf), or [email](mailto:contact@softmata.dev)):
- **What robot** — platform, actuators, sensors
- **What control rate** you're achieving on real hardware
- **What worked** out of the box
- **What needed tuning** — PID gains, sensor dropout thresholds, timing budgets
- **What broke** — anything that works in sim but fails on hardware

We'll add validated hardware to the docs and credit contributors.

---

<p align="center">
  <a href="https://github.com/softmata/horus/issues">Report a Bug</a> &middot;
  <a href="CONTRIBUTING.md">Contributing</a> &middot;
  <a href="https://discord.gg/hEZC3ev2Nf">Discord</a> &middot;
  <a href="LICENSE">Apache-2.0</a>
</p>
