# Differential Drive

A mobile robot that drives in a square pattern. The simplest possible Horus application.

## What you'll learn

- Defining custom messages with `message!`
- Implementing the `Node` trait (`name()` + `tick()`)
- Publishing and subscribing to topics
- Multi-rate scheduling (50Hz controller + 10Hz safety monitor)
- Logging with `hlog!`

## Robot

`robots/diffbot.urdf` — 4 links (base + 2 wheels + caster), 3 joints (2 continuous + 1 fixed). Primitive geometry only (box, cylinder, sphere).

## Running

**Terminal 1 — start the simulator:**

```bash
sim3d --mode visual --robot robots/diffbot.urdf --world worlds/flat_ground.yaml --robot-name diffbot
```

You should see a blue rectangular chassis on a green ground plane, with two dark wheels and a small caster sphere.

**Terminal 2 — run the controller:**

```bash
horus run main.rs
```

The robot drives forward for ~2 seconds, turns ~90 degrees, then repeats — tracing a square path over 30 seconds. Console output shows position updates via `hlog!`.

## Inspecting the system

While the controller is running:

```bash
# List active topics
horus topic list
# Expected: cmd_vel, odom

# Watch velocity commands
horus topic echo cmd_vel
# lin=0.30, ang=0.00 (forward) alternating with lin=0.00, ang=1.00 (turning)

# Check message rate
horus topic bw cmd_vel
# Expected: ~50 msg/sec

# List running nodes
horus node list
# Expected: SquareDriver, SafetyNode

# Open the TUI dashboard
horus monitor -t
```

## Shutdown

Press `Ctrl+C` on the controller. The scheduler prints a shutdown summary.

```bash
# Verify no orphaned shared memory
horus clean --shm
```

## Headless mode

Run without a window for CI or testing:

```bash
sim3d --headless --robot robots/diffbot.urdf --world worlds/flat_ground.yaml --robot-name diffbot
```
