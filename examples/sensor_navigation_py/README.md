# Sensor Navigation (Python)

The [`sensor_navigation`](../sensor_navigation/) example, ported to Python.
Same four nodes, same rates, same topics.

Where [`python_robot`](../python_robot/) is the Python "hello robot" — one
control loop and a safety monitor — this is the next step: a multi-rate sensor
pipeline, a reactive controller, and a frame tree.

```bash
horus run main.py
```

## What you'll learn

- Multi-rate scheduling from Python (100 Hz / 20 Hz / 2 Hz / 1 Hz in one process)
- Consuming a typed sensor message (`horus.LaserScan`) and its helpers
- Mixing typed and untyped topics — `cmd_vel` is a `CmdVel`, `obstacles` is a dict
- Building a static frame tree with `horus.TransformFrame`
- Node `init` callbacks, which is where Python puts what Rust puts in a constructor

## Architecture

```
LidarProcessor (100Hz) --> Navigator (20Hz) --> cmd_vel
       |                       ^
   obstacles                imu.data
                               |
SensorFrames (1Hz)     TelemetryLogger (2Hz)
```

## Differences from the Rust example

| | Rust | Python |
|---|---|---|
| Obstacle alert | a `message!` struct | a plain dict on an untyped topic |
| Closest reading | `ranges.iter().filter(..).fold(f32::MAX, f32::min)` | `scan.min_range()`, which already skips invalid returns |
| Periodic log | `hlog_every!(50, info, ...)` | the node counts its own ticks — Python has no such macro |
| Frame setup | `SensorFrames::new()` | the node's `init` callback |
| Frame API | `tf.add_frame("x").parent("y").build()` | `tf.register_frame("x", "y")` |
| Run duration | `scheduler.run_for(60_u64.secs())` | `horus.run(..., duration=60.0)` |

## Robot

`robots/sensor_bot.urdf` — differential drive with a LiDAR puck and an IMU.
6 links, 5 joints (2 continuous wheels + 3 fixed sensor mounts). The static
offsets in `build_frames()` are read from it.

## Running

**Terminal 1 — start the simulator (it is what publishes `lidar.scan` and `imu.data`):**

```bash
sim3d --mode visual --robot robots/sensor_bot.urdf --world worlds/obstacle_course.yaml --robot-name sensor_bot
```

**Terminal 2 — run the controller:**

```bash
horus run main.py
```

Without the simulator the pipeline still runs, and says so rather than
pretending:

```text
[INFO] [SensorFrames] frame tree ready (5 frames)
[INFO] [TelemetryLogger] [T1] lin=0.30 ang=0.00 | no imu
[INFO] [TelemetryLogger] [T2] lin=0.30 ang=0.00 | no imu
```

With something publishing scans, the navigator reacts:

```text
[WARN] [Navigator] Obstacle at 0.30m, angle=90.0deg - turning
```

## Inspecting the system

```bash
horus topic list          # cmd_vel, obstacles — plus lidar.scan and imu.data
                          # once the simulator is publishing them
horus topic echo cmd_vel  # linear: 0.300, angular: 0.000
horus node list           # LidarProcessor, Navigator, SensorFrames, TelemetryLogger
horus frame tree          # map -> odom -> base_link -> lidar_link / imu_link
horus monitor             # TUI dashboard
```

A topic appears in `horus topic list` only while something publishes it, so
`lidar.scan` and `imu.data` are absent until the simulator runs.

## Making it your own

```bash
horus new my_robot --from sensor_navigation_py
cd my_robot
horus run
```
