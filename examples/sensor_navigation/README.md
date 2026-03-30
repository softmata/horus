# Sensor Navigation

A mobile robot with LiDAR and IMU that navigates through an obstacle course. Demonstrates multi-rate scheduling and sensor pipelines.

## What you'll learn

- Multi-rate node scheduling (100Hz, 20Hz, 2Hz, 1Hz)
- Sensor data pipelines: LiDAR scan, obstacle detection, reactive navigation
- Static frame transforms for sensor mounts
- Periodic logging with `hlog_every!`
- Blackbox post-mortem analysis

## Architecture

Four nodes at different rates form a processing pipeline:

```
LidarProcessor (100Hz) --> Navigator (20Hz) --> cmd_vel
       |                       ^
   ObstacleAlert           Imu data
                               |
SensorFrames (1Hz)     TelemetryLogger (2Hz)
```

## Robot

`robots/sensor_bot.urdf` — differential-drive with LiDAR puck and IMU. 6 links, 5 joints (2 continuous wheels + 3 fixed sensor mounts).

## Running

**Terminal 1 — start the simulator:**

```bash
sim3d --mode visual \
  --robot robots/sensor_bot.urdf \
  --world worlds/obstacle_course.yaml \
  --robot-name sensor_bot
```

You should see a teal cylindrical robot in a corridor with three colored obstacles (red, green, blue) and a green goal marker.

**Terminal 2 — run the controller:**

```bash
horus run main.rs
```

The robot drives forward through the corridor. When it detects an obstacle within 50cm, it turns away for ~2 seconds, then resumes forward motion. Console shows obstacle alerts and telemetry data.

## Inspecting the system

```bash
# Verify multi-rate scheduling
horus node list
# LidarProcessor: 100Hz, Navigator: 20Hz, SensorFrames: 1Hz, TelemetryLogger: 2Hz

# Check navigation command rate
horus topic bw cmd_vel
# Expected: ~20 msg/sec (Navigator rate)

# Obstacles topic only publishes on detection
horus topic bw obstacles

# View static sensor frame tree
horus frame tree
# map
# +-- odom
#     +-- base_link
#         +-- lidar_link (static)
#         +-- imu_link (static)

# Verify static transform
horus frame echo lidar_link base_link
# translation=[0.05, 0.0, 0.10] (constant)

# Filter logs by component
horus log TelemetryLogger -f

# Filter by level
horus log -f -l warn
```

## Post-mortem analysis

After stopping the controller:

```bash
# Check for timing anomalies
horus blackbox -a

# View last 10 ticks with timing data
horus blackbox -t 10
```
