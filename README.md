# HORUS

<p align="center">
  <strong>English</strong> ·
  <a href="README.zh-CN.md">简体中文</a> ·
  <a href="README.pt-BR.md">Português (Brasil)</a> ·
  <a href="README.ja.md">日本語</a> ·
  <a href="README.es.md">Español</a> ·
  <a href="README.de.md">Deutsch</a>
</p>

**Real-time distributed middleware for Rust, Python, and C++. Sub-200ns IPC.**

[![CI](https://github.com/softmata/horus/actions/workflows/ci.yml/badge.svg)](https://github.com/softmata/horus/actions)
[![Version](https://img.shields.io/badge/v0.4.0-blue.svg)](https://github.com/softmata/horus/releases)
[![Rust](https://img.shields.io/badge/rust-%3E%3D1.90-orange.svg?logo=rust)](https://www.rust-lang.org/)
[![Python](https://img.shields.io/badge/python-%3E%3D3.9-blue.svg?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE)
[![Discord](https://img.shields.io/badge/Discord-Join-7289da?logo=discord&logoColor=white)](https://discord.gg/hEZC3ev2Nf)

[**Docs**](https://docs.horusrobotics.dev) &middot;
[**Quick Start**](https://docs.horusrobotics.dev/getting-started/quick-start) &middot;
[**Benchmarks**](https://docs.horusrobotics.dev/performance/benchmarks) &middot;
[**Coming from ROS2?**](https://docs.horusrobotics.dev/tutorials/migrating-from-ros2-rust) &middot;
[**Discord**](https://discord.gg/hEZC3ev2Nf)

---

## Get Started

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

The installer resolves the latest release tag and takes both halves from it: the
prebuilt CLI, verified against that release's `SHA256SUMS`, and the matching
source tree, cached at `~/.horus/cache/horus@<version>`. Both come from the same
tag on purpose — `horus run` compiles your project against that cached tree as a
path dependency, so a CLI built from one revision and libraries from another
gives you nodes that cannot read each other's shared memory.

Pin the tag when you want the same install twice. The variable goes on the right
of the pipe — that is the environment `bash` reads; on the left it lands in
curl's, where nothing reads it and you silently get the latest release anyway:

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.4.0 bash
```

Or install manually:

```bash
git clone https://github.com/softmata/horus.git && cd horus && ./install.sh
```

**Requires Rust 1.90 or newer** (`rustup update stable`). Python 3.9+ for the
Python API; CMake 3.20+ and a C++17 compiler for the C++ API.

**C++**: link against `libhorus_cpp` and `#include <horus/horus.hpp>`.

**Python**: `pip install "horus-robotics>=0.4.0"` — keep the floor. The newest
release on PyPI is 0.1.9, from March 2026, and it speaks an older shared-memory
format than a 0.4.x CLI: bare `pip install horus-robotics` gets you a package
that imports, runs, and sees none of your topics. The floor turns that into a
resolver error instead. Until 0.4.x reaches PyPI, build the bindings from the
tree the installer already cached: `pip install ~/.horus/cache/horus@0.4.0/horus_py`.

**Rust**: there is no crates.io channel. `cargo add horus` and `cargo install
horus` fetch an unrelated crate that happens to own the name; HORUS's own crates
carry git and path dependencies that `cargo package` rejects, so none of them are
published. `horus new` and `horus run` point Cargo at the cached source instead.

Docker, upgrading, uninstalling and the platform matrix are in
[Install, Upgrade, Uninstall](#install-upgrade-uninstall).

---

## Why HORUS?

HORUS is a real-time distributed middleware that replaces DDS with shared-memory ring buffers and lock-free synchronization. Built for any system where latency, determinism, and safety matter — robotics, industrial automation, autonomous vehicles, trading systems, game engines, and more.

|             | **HORUS**                                     | **ROS2**                                    |
|-------------|-----------------------------------------------|---------------------------------------------|
| IPC latency | **171 ns** one-way, cross-process (`cross_process_benchmark`) | ~5 µs (REP 2014)                            |
| Scheduling  | Deterministic, 5 execution classes            | Best-effort callbacks                       |
| RT support  | Built-in (budget, deadline, watchdog)         | Manual DDS QoS                              |
| Safety      | Graduated watchdog, safe-state hook, BlackBox | Application-level                           |
| AI + RT     | Same process — AsyncIo for GPU, RT for motors | Separate processes                          |
| Tensors     | Pool-backed, zero-copy to NumPy (host memory)  | Serialize → deserialize                     |
| Languages   | **Rust + Python + C++** (same shared memory)  | C++ + Python (DDS serialization)            |
| Config      | Single `horus.toml`                           | package.xml + CMakeLists.txt + launch files |
| Setup       | `horus new && horus run`                      | colcon build + source install + launch      |
    
---

## Quick Start

**Rust** — 1kHz motor controller in one file:

```rust
use horus::prelude::*;

message! {
    SensorReading { position: f64, velocity: f64 }
    MotorCommand  { voltage: f64 }
}

struct Sensor {
    reading: Topic<SensorReading>,
    pos: f64,
}

impl Sensor {
    fn new() -> Result<Self> {
        Ok(Self { reading: Topic::new("sensor.data")?, pos: 0.0 })
    }
}

impl Node for Sensor {
    fn name(&self) -> &str { "sensor" }
    fn tick(&mut self) {
        self.pos += 0.01;
        self.reading.send(SensorReading { position: self.pos, velocity: 0.5 });
    }
}

struct Controller {
    sensor: Topic<SensorReading>,
    cmd: Topic<MotorCommand>,
    target: f64,
}

impl Controller {
    fn new() -> Result<Self> {
        Ok(Self {
            sensor: Topic::new("sensor.data")?,
            cmd: Topic::new("motor.cmd")?,
            target: 1.0,
        })
    }
}

impl Node for Controller {
    fn name(&self) -> &str { "controller" }
    fn tick(&mut self) {
        if let Some(s) = self.sensor.recv() {
            self.cmd.send(MotorCommand { voltage: (self.target - s.position) * 0.5 });
        }
    }
}

fn main() -> Result<()> {
    let mut sched = Scheduler::new().tick_rate(1000_u64.hz());
    sched.add(Sensor::new()?).order(0).build()?;
    sched.add(Controller::new()?).order(1).rate(1000_u64.hz()).on_miss(Miss::SafeMode).build()?;
    sched.run()
}
```

**You need three concepts to read that**: a **node** (a struct with a `tick()`),
a **topic** (`Topic::new("sensor.data")`), and the **scheduler** that runs them.
Everything else in `main()` is timing policy and can wait: `tick_rate()` sets the
scheduler's clock (default 100 Hz), `.order()` sequences nodes within a tick
(default 0), `.rate()` moves a node onto its own real-time thread, and
`.on_miss()` says what to do when a node overruns its deadline. Delete all four
and the program still runs — every node ticks best-effort at 100 Hz. Add them
back when timing matters.
[Execution classes &rarr;](https://docs.horusrobotics.dev/advanced/scheduler-configuration#execution-classes)

Prefer less boilerplate? The `node!` macro writes the `struct` and `impl Node`
for you. `horus new --macro` scaffolds a starter in that style — a single
`Controller` publishing `Twist` on `motors.cmd_vel`, not the two-node example
above.

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

`node.recv(topic)` returns `None` when nothing is waiting. `node.has_msg(topic)`
asks the same question without consuming the message — the reading is held and
handed to the next `recv()`. `horus new --python` scaffolds a one-node starter
that uses both.

`pubs=` and `subs=` accept either a list of topics, as above, or a single bare
topic string: `pubs="motor.cmd"` and `pubs=["motor.cmd"]` build the same node.
Both spellings are in circulation across the docs, so a bare string in an
example is not a typo.

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

`horus::log::info(node_name, message)` writes to the HORUS log stream rather
than stdout, so `horus log` sees it. `horus new --cpp` scaffolds a one-node
starter that uses it.

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

Set `.rate()`, `.budget()`, or `.deadline()` and RT is automatic — no manual thread management. [Learn more →](https://docs.horusrobotics.dev/advanced/scheduler-configuration#execution-classes)

### Safety

The scheduler monitors every node at runtime:

- **Graduated watchdog** — warn → halve the rate → isolate → kill, at 3, 5, 10
  and 20 consecutive misses by default. Recovery walks back down: 100 clean
  ticks de-isolate, 100 more restore the original rate. A killed node stays
  stopped.
- **Deadline enforcement** — `.budget()` and `.deadline()` with miss policies (Warn, Skip, SafeMode, Stop)
- **`enter_safe_state()`** — you define what "safe" means per node (stop motors, close valves)
- **BlackBox flight recorder** — ring-buffer event log for post-mortem crash analysis
- **Fault tolerance** — per-node failure policies (restart with backoff, skip, fatal)

`Miss::SafeMode` is a hook, not a state machine. The scheduler calls
`enter_safe_state()` once, on the transition into safe mode, and the node keeps
ticking afterwards — it is not isolated, and `is_safe_state()` is never polled.
So `tick()` has to go on publishing the safe outputs itself; a zeroed velocity
command still has to be sent every cycle. The latch clears the first time the
node meets its deadline again, which is what lets a later degradation be caught,
so a flapping node is safed once per episode rather than once per miss.
Isolating or stopping a node is the graduated watchdog's job, above.

[Safety Monitor →](https://docs.horusrobotics.dev/advanced/safety-monitor) · [BlackBox →](https://docs.horusrobotics.dev/advanced/blackbox) · [Fault Tolerance →](https://docs.horusrobotics.dev/advanced/circuit-breaker)

### Zero-Copy AI Pipeline

Run camera → YOLO → tracking → motor control in one process. 4K frames stay in shared memory and reach NumPy without a copy. Tensor memory is host-side: the pool allocator is mmap, so moving a frame to a GPU is still an explicit host→device copy that you make.

```python
def detector_tick(node):
    frame = node.recv("camera")
    if frame is not None:
        array = frame.to_numpy()                 # zero-copy view of shared memory
        for det in model(array):
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
horus new my_bot --cpp          # scaffold C++ project
horus run                       # build and run
horus topic list                # inspect live topics
horus topic echo camera.rgb     # watch messages (works across all languages)
horus monitor                   # TUI system dashboard
horus deploy pi@192.168.1.50    # deploy to robot
horus doctor                    # ecosystem health check
horus self update               # upgrade the CLI and the cached source together
```

40+ commands. [Full CLI reference →](https://docs.horusrobotics.dev/development/cli-reference)

---

## Performance

Measured with RDTSC cycle counting, Tukey IQR outlier filtering, bootstrap 95% CIs on Intel i9-14900K. [Full methodology →](benchmarks/)

| Topology             | HORUS      | Measurement        |
|----------------------|------------|--------------------|
| Same-process pub/sub | **91 ns**  | producer-side `send()` |
| Cross-process        | **171 ns** | end-to-end, one-way    |
| 1 pub → 3 subs       | **80 ns**  | producer-side `send()` |

The link above is to the methodology — RDTSC timing with calibrated overhead
subtraction, Tukey IQR fences, bootstrap 95% CIs — which is shared. The numbers
are not: this table is an i9-14900K run whose raw percentiles are not published
here, while the tables in [`benchmarks/README.md`](benchmarks/README.md) were
taken on an Intel Core i7-10750H @ 2.60 GHz under the `powersave` governor. The
two sets are not comparable row to row, and the i7 figures are the ones this
repository can show its work for.

Reproduce with `cargo run --release -p horus_benchmarks --bin all_paths_latency`, which prints the
full percentile distribution, the backend selected for each topology, and the
measured hardware floor it subtracts.

**Against ROS 2.** The nearest published figure is ROS 2's REP 2014 reference for
default DDS, ~5 µs median for a 64-byte same-process message. Compared to HORUS's
end-to-end cross-process 171 ns — the harder case for HORUS, and therefore the
conservative comparison — that is roughly **30x**. HORUS does not measure ROS 2
itself, and no longer ships a benchmark that quotes it: the binary that did
built ROS 2, CycloneDDS and FastDDS distributions arithmetically from two
constants and wrote them beside real measurements, so it was deleted rather
than relabelled. The REP 2014 figure above is cited, not measured here. Any
number that matters to your decision is worth measuring on your own hardware
and message sizes.

|  vs iceoryx2              | HORUS med | iox2 med | median    | p99      |
|---------------------------|-----------|----------|-----------|----------|
| Same-thread (8 B)         |   55 ns   |  416 ns  | **7.6x**  | **5.8x** |
| Cross-thread RTT (8 B)    |  371 ns   | 1716 ns  | **4.6x**  | **3.9x** |
| Cross-process RTT (8 B)   |  413 ns   | 1622 ns  | **3.9x**  | **3.4x** |
| Same-thread (1 KB)        |  169 ns   |  448 ns  | **2.7x**  | **2.7x** |
| Same-thread (4 KB)        |  574 ns   |  550 ns  | 0.96x     | 0.93x    |
| MPMC 4P/4S recv cost      |   35 ns   |  216 ns  | **6.2x**  | **3.0x** |
| Throughput (same loop)    | 37.8 M/s  |  3.0 M/s | **12.8x** | —        |

Ratios above 1 favour HORUS. Both columns come from the same run, which is what
makes the ratio trustworthy; the absolute nanoseconds drift with machine load
and thermal state, so compare ratios across runs, not absolutes.

Unlike the ROS 2 row above, this one is measured on both sides — see the
reproduce command below.

4 KB is now roughly a tie (0.96x median). It used to be 0.46x — see the
zero-copy note below for why a 4 KB POD sent **by value** is the wrong shape for
bulk data in the first place.

**Bulk payloads do not go through `Topic<[u8; N]>`.** `Image`, `PointCloud`,
`DepthImage` and `Tensor` keep their buffers in a shared-memory pool and put
only a 224-byte descriptor on the topic, so publishing is zero-copy and the
latency is flat in frame size:

| frame (Mono8)   | pixels  | one-way p50 |
|-----------------|---------|-------------|
| 64x64           | 4 KB    | 306 ns      |
| 640x480         | 307 KB  | 297 ns      |
| 1920x1080       | 2 MB    | 284 ns      |

That is the row to compare against a loan/publish API. The 4 KB line in the
table above sends a POD **by value**, which copies 4 KB into the ring and 4 KB
back out; it is the wrong API for bulk data and is kept in the table because
removing a row where HORUS loses would be the more dishonest choice.

Reproduce with `cargo run --release -p horus_benchmarks --bin topic_probe --
--image 1920x1080`.

One caveat worth knowing before you build a camera pipeline: `Image::new`
zero-initialises the buffer, which for a 1920x1080 frame costs ~82 us. That is
a deliberate scrub — a pool slot can be reused by another process, so it is
cleared before it is handed out — but it means allocating a frame per capture
puts that cost on your critical path. Allocate once and clone; a clone is a
refcount bump and a descriptor copy, which is what the numbers above measure.

HORUS's advantage is largest at small payloads, which is where robotics control
traffic lives: a CmdVel is 16 bytes. If your messages are camera frames or point
clouds, use the tensor-backed types below rather than sending them by value, and
measure on your own hardware either way.

The round-trip rows are full round trips, not halved. Halving reports a
one-sided stall as half a stall on each leg, which understates exactly the tail
that matters. The same-thread rows are one thread writing and then reading its
own ring: a lower bound on IPC cost with no cross-core transfer in it, for
either library. Prefer the cross-process row when quoting a number.

This table replaces one claiming 6.3x / 2.1x / 4.3x. Those came from a harness
that gave the two libraries different topologies: the HORUS arm sent and
received on a single handle, which selects an inlined same-instance fast path
with no dispatch and no ring publication, while the iceoryx2 arm used a real
publisher and subscriber. Both arms now use separate handles.

Reproduce with `cargo run --release -p horus_benchmarks --bin iceoryx2_comparison
--features iceoryx2`, which links iceoryx2 and times it in the same harness.
Measured on an i7-10750H under the `powersave` governor; absolute values will
differ on your hardware, ratios less so.

Scales near-linearly to 100 nodes (14% degradation) and O(1) to 1,000 topics.

```bash
cargo run --release -p horus_benchmarks --bin all_paths_latency    # run it yourself
```

---

## Examples

12 working projects in [`examples/`](examples/) — from differential drive to quadruped gait generation. Every directory there is a `horus.toml` project: `horus build` and `horus run` drive all of them, in Rust, Python or C++.

The single-file API demos are examples of the `horus` crate. The workspace root is a virtual manifest, so select the package with `-p`:

```bash
cargo run -p horus --example 01_hello_node     # your first node
cargo run -p horus --example 02_pub_sub        # topics and messages
cargo run -p horus --example 03_multi_rate     # multi-rate scheduling
cargo run -p horus --example 04_services       # request/response
cargo run -p horus --example 05_realtime       # RT with deadline enforcement
```

[Full examples →](examples/) · [Tutorials →](https://docs.horusrobotics.dev/tutorials/01-sensor-node-rust) · [ROS 2 bridge recipe →](https://docs.horusrobotics.dev/recipes/ros2-bridge)

---

## Install, Upgrade, Uninstall

### Upgrade

```bash
horus self update           # move the CLI and the cached source to the latest release
horus self update --check   # report current vs latest, change nothing
```

`horus self update` resolves the latest release, verifies the download against
that release's `SHA256SUMS`, replaces the binary, and refreshes
`~/.horus/cache/horus@<version>` at the same tag. Both halves move together, for
the reason in [Get Started](#get-started). Re-running the installer does the
same job, and is how you move to a *particular* release — including back to an
older one:

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.4.0 bash
```

To see what you are running: `horus --version` reports the CLI, and
`~/.horus/install_manifest.toml` records what the last install put on the machine
— version, tag, commit and method. The CLI compares the two itself and warns when
they have drifted apart, naming the command that reconciles them; set
`HORUS_STRICT_VERSION=1` to make that a hard failure instead, which is what you
want in CI.

### Uninstall

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | bash
```

Or `./uninstall.sh` from a clone. It prints an inventory first — binaries,
`~/.horus` (cache, install manifest, credentials, config), shell completions, the
man page, the shell-profile lines and `/dev/shm/horus_*` — and asks before
deleting any of it, on your terminal even when the script itself arrived through
the pipe. `--dry-run` prints that inventory and removes nothing. `--yes` runs
unattended and keeps `~/.horus/config.toml`, your credentials and the Cargo
registry cache, so an automated run never removes more than a supervised one
would by default:

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | bash -s -- --yes
```

With no terminal and no `--yes` — a CI job, a provisioning script — it stops and
says so rather than guessing at a default.

### Docker

```bash
docker build --target dev -t horus:dev .
docker run --rm -it --shm-size=1g -v "$PWD:/work" -w /work horus:dev run
```

`--target dev` is the one you want. A bare `docker build .` builds the default
stage, the slim CLI image the [Dockerfile](Dockerfile) tags `horus:cli` — 284 MB,
no language toolchains. It can inspect a system, validate a manifest and read
logs, but it cannot build or run a project: `horus run` there stops at `H060
Rust toolchain not installed`. The dev image carries Rust, Python and CMake and
costs 3.98 GB for them.

Nodes talk through shared memory, so a container that runs them needs a real
`/dev/shm` — the 64 MB default is small for image or point-cloud topics, hence
`--shm-size`. Add `--ipc=host` for IPC between containers, and `--cap-add=SYS_NICE`
for real-time nodes: without it every RT thread drops to normal priority, and a
benchmark in that container measures the container. Each release tag publishes
both images to GitHub Container Registry — `ghcr.io/softmata/horus:dev` and
`:latest`, plus the immutable `:<version>` and `:<version>-dev` to pin a robot
to — so `docker build` is only needed for a fork or an unreleased commit. The
Dockerfile header covers the rest, including how to keep file ownership on bind
mounts.

### Platform Support

| Platform              | Prebuilt binary             | Notes                                     |
|-----------------------|-----------------------------|-------------------------------------------|
| Linux x86_64          | `horus-linux-amd64.tar.gz`  | glibc 2.28 or newer                       |
| Linux aarch64         | `horus-linux-arm64.tar.gz`  | glibc 2.28 or newer — Pi OS 64-bit, Jetson |
| Linux armv7 (32-bit)  | `horus-linux-armv7.tar.gz`  | glibc 2.28 or newer — Pi OS 32-bit         |
| macOS Apple Silicon   | `horus-macos-arm64.tar.gz`  |                                           |
| macOS Intel           | `horus-macos-amd64.tar.gz`  |                                           |
| Windows x86_64        | `horus-windows-amd64.zip`   | `install.ps1`, or `install.sh` under Git Bash / WSL |

On Windows, PowerShell takes the same install with the same knobs — set them as
their own statement, since PowerShell has no `VAR=value command` prefix:

```powershell
irm https://github.com/softmata/horus/raw/main/install.ps1 | iex
```

The Linux binaries are linked against glibc rather than static, and are
cross-compiled against a glibc 2.28 floor — Debian 10, Ubuntu 18.04, RHEL 8 —
which the release workflow reads back out of each ELF and enforces. Releases cut
before that floor existed, `v0.4.0` and earlier, were built natively on the CI
runner and need glibc 2.39, so they will not start on Pi OS, JetPack, Ubuntu
22.04, Debian 12 or RHEL 9. Below the floor, or on a musl distro such as Alpine,
install with `HORUS_BUILD_FROM_SOURCE=1` and compile; budget hours, not minutes,
on a single-board computer.

### Installer Options

| Variable                       | Effect                                                                          |
|--------------------------------|---------------------------------------------------------------------------------|
| `HORUS_VERSION=v0.4.0`         | install exactly that release — binary and source. Leading `v` optional          |
| `HORUS_BUILD_FROM_SOURCE=1`    | skip the prebuilt binary, compile the source at the resolved tag                 |
| `HORUS_LOCAL_SOURCE=/path`     | use a source tree already on disk — no clone, nothing fetched for it (air-gapped) |
| `HORUS_PREFIX=/opt/horus`      | install root override — `<prefix>/bin` for the binary, `<prefix>` in place of `~/.horus`. A root install with no `SUDO_USER` refuses without it, rather than installing for root alone |
| `HORUS_INSTALL_BRANCH=dev`     | developer escape hatch: build from a branch. Forces a source build — a branch is never paired with a release binary |
| `HORUS_NO_SHELL_INTEGRATION=1` | leave shell rc files alone (skips `horus env --init`, which shadows cargo/pip/cmake inside HORUS projects) |

[Installation guide →](https://docs.horusrobotics.dev/getting-started/installation)

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
