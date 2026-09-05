# Differential Drive (C++)

The [`differential_drive`](../differential_drive/) example, ported to C++. Same
robot, same topics, same two nodes — read the two side by side to see what the
language boundary changes and what it does not.

This is a `horus.toml` project, so it builds and runs with the same commands as
the Rust and Python examples:

```bash
horus build
horus run
```

(The programs under [`horus_cpp/examples/`](../../horus_cpp/examples/) are
standalone CMake translation units built with `cmake -S horus_cpp/examples`.
They show individual APIs; this one shows the project workflow the C++
tutorials teach.)

## What you'll learn

- Subclassing `horus::Node` — the C++ equivalent of Rust's `impl Node`
- `advertise<T>()` / `subscribe<T>()` in a node constructor
- Publishing and receiving `horus::msg::CmdVel` and `horus::msg::Odometry`
- Multi-rate scheduling (50 Hz controller + 10 Hz safety monitor)
- Logging with `horus::log::info` / `horus::log::warn`, which `horus log` reads

## Differences from the Rust example

| | Rust | C++ |
|---|---|---|
| Odometry message | declared in the example with `message!` | the built-in `horus::msg::Odometry` |
| Run duration | `scheduler.run_for(30_u64.secs())` | the driver counts ticks and calls `sched.stop()` — the C++ `Scheduler` has no `run_for` |
| Reading a subscription | `if let Some(odom) = sub.recv()` | `if (auto odom = sub->recv())`, then two arrows to a field |

Everything else — topic names, rates, the square pattern, the safety envelope —
is the same.

## Layout

```
differential_drive_cpp/
  horus.toml           # project manifest
  src/main.cpp         # the controller
  robots/diffbot.urdf  # robot description
  worlds/flat_ground.yaml
```

`src/main.cpp`, not `main.cpp`: the generated CMakeLists globs `src/`, so a C++
source at the project root is detected as the main file and then never compiled.

## Robot

`robots/diffbot.urdf` — 4 links (base + 2 wheels + caster), 3 joints
(2 continuous + 1 fixed). Primitive geometry only (box, cylinder, sphere).

## Running

**Terminal 1 — start the simulator (optional; the controller runs without it):**

> **sim3d is not public yet.** The simulator lives in a private repository,
> so this command will not work for you — see [../README.md](../README.md#prerequisites).
> The example below runs without it; the simulator only adds visualisation
> and simulated sensor input.

```bash
sim3d --mode visual --robot robots/diffbot.urdf --world worlds/flat_ground.yaml --robot-name diffbot
```

**Terminal 2 — run the controller:**

```bash
horus run
```

The robot drives forward for ~2 seconds, turns ~90 degrees, then repeats —
tracing a square over 30 seconds, after which the driver stops the scheduler and
the timing report prints:

```text
[INFO] [SquareDriver] 30 s elapsed — stopping
Node                  Avg(us)  P99(us)  Max(us)   Stddev Budget(us) Overruns    Ticks   Misses
SafetyNode                  9       42       50      5.7      80000        0      300       0
SquareDriver               29      155     1339     59.5      16000        0     1500       0
```

1500 driver ticks is 30 s at 50 Hz; 300 safety ticks is 30 s at 10 Hz.

## Inspecting the system

While the controller is running:

```bash
horus topic list
#   cmd_vel   18.6 KB   48.0 Hz   active
# `odom` appears here only while something publishes it — the controller
# subscribes to it, and a topic with no publisher has nothing to list. Start
# the simulator to see it.

horus topic echo cmd_vel
#   [16:04:45.971] #1: linear: 0.300, angular: 0.000
#   [16:04:49.175] #2: linear: 0.000, angular: 1.000

horus node list
#   SafetyNode      Running   Healthy   10   10 Hz    299 ticks
#   SquareDriver    Running   Healthy    0   50 Hz   1497 ticks

horus monitor             # TUI dashboard
```

## Making it your own

```bash
horus new my_robot --from differential_drive_cpp
cd my_robot
horus build
```

`--from` copies the example, renames the package, and leaves the build
artifacts behind.

## Prerequisites

`horus build` compiles this with cmake and links `libhorus_cpp.a`, building the
static library on first use. You need a C++17 compiler, cmake 3.20+, and either
make or ninja — `horus doctor` checks all three.
