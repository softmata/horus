# Quadruped Walking Gait

A quadruped robot with a trot gait, real-time joint control, and IMU-based balance feedback. The most advanced example.

## What you'll learn

- High-frequency RT scheduling (200Hz)
- RT node configuration: `wcet_budget()`, `deadline()`, `deadline_miss_policy()`
- 12-DOF joint coordination (4 legs x 3 joints)
- IMU-based balance compensation
- Deadline monitoring with blackbox analysis
- Runtime parameter tuning

## Architecture

```
GaitGenerator (200Hz)  -->  JointController (200Hz, RT)  -->  joint_commands
                                    ^
                                    | body_state
                                    |
                            BalanceMonitor (50Hz)  <--  imu/data
```

`GaitGenerator` produces a trot gait (diagonal legs FL+RR move together, then FR+RL). `JointController` smooths the targets and runs at RT priority with a 100us WCET budget and 500us deadline. `BalanceMonitor` reads IMU data and publishes roll/pitch corrections.

## Robot

`robots/quadruped.urdf` — 13 links (body + 4 legs x 3 segments), 12 revolute joints with limits, damping, and friction. Dark grey body with orange hip joints.

## Running

**Terminal 1 — start the simulator:**

```bash
sim3d --mode visual --robot robots/quadruped.urdf --world worlds/terrain.yaml --robot-name quadruped
```

You should see the quadruped standing on a ground plane with stepping stones and a ramp nearby.

**Terminal 2 — run the controller:**

```bash
horus run main.rs
```

The robot performs a trot gait — diagonal leg pairs swing simultaneously. Body stays roughly level thanks to balance compensation. Joint state logs appear periodically.

## Inspecting the system

```bash
# Verify RT scheduling
horus node list
# GaitGenerator: 200Hz (BestEffort)
# JointController: 200Hz (RT)
# BalanceMonitor: 50Hz (BestEffort)

# Inspect RT node configuration
horus node info JointController
# WCET budget: 100us, Deadline: 500us, Priority: High, RT class: Firm

# Watch joint commands at 200Hz
horus topic bw joint_commands
# Expected: ~200 msg/sec

# Watch topics
horus topic echo joint_commands
horus topic echo body_state
```

## Runtime tuning

Adjust gait speed while the controller is running:

```bash
# Slow down
horus param set gait_speed 0.5

# Speed up
horus param set gait_speed 2.0

# List all parameters
horus param list
```

## Balance monitoring

If the robot encounters uneven terrain or a disturbance, the console shows:

```
BALANCE WARNING: roll=0.35 pitch=0.12
```

The balance monitor compensates by adjusting hip joint angles proportionally to measured tilt.

## RT analysis

After running for 30 seconds:

```bash
# Check for WCET violations and deadline misses
horus blackbox -a

# View timing data for last 20 ticks
horus blackbox -t 20
```

On an unloaded system, there should be 0 WCET violations and 0 deadline misses.

## Emergency stop

```bash
# Kill the RT joint controller
horus node kill JointController
```

The robot collapses to the ground (no active joint control). The gait generator continues publishing, but joints no longer track. Balance monitor reports extreme tilt.
