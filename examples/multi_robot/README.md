# Multi-Robot Formation

Three scout robots maintain a triangular formation while moving in a circle. Demonstrates multi-robot coordination, namespaced topics, and launch files.

## What you'll learn

- Multi-robot scenes (multiple URDFs in one world)
- Namespaced topics (`scout_1/cmd_vel`, `scout_2/cmd_vel`, etc.)
- Fleet-level shared topics (`fleet/poses`, `fleet/formation_cmd`)
- Launch file configuration
- Node discovery and web monitoring

## Architecture

```
FormationCoordinator (5Hz)
     |
     | fleet/formation_cmd
     v
ScoutController x3 (20Hz)  -->  scout_N/cmd_vel
     |
     | fleet/poses (shared)
     +------------------------->  FormationCoordinator
```

Each scout runs proportional control toward its formation offset from the coordinator's target center, which traces a slow circle.

## Robot

`robots/scout.urdf` — small circular differential-drive robot (~20cm diameter). 4 links, 3 joints.

## Running

**Terminal 1 — start the simulator:**

```bash
sim3d --mode visual --world worlds/arena.yaml
```

Three blue scout robots appear at different positions inside a walled arena. The world file (`worlds/arena.yaml`) defines all three robots with their starting positions and LiDAR sensors.

**Terminal 2 — run the controller:**

```bash
horus run main.rs
```

The three robots converge into a triangular formation, then the formation center moves in a slow circle. Console logs show scout positions.

## Using the launch file

Instead of `horus run main.rs`, you can use the launch file for parameterized deployment:

```bash
# Preview what will be launched
horus launch launch/formation.yaml --dry-run

# List all nodes and parameters
horus launch launch/formation.yaml --list

# Run via launch file
horus launch launch/formation.yaml
```

The launch file (`launch/formation.yaml`) configures 4 nodes with namespaces, rates, and parameters for each scout's formation offset.

## Inspecting the system

```bash
# List all topics (namespaced + fleet-level)
horus topic list
# scout_1/cmd_vel, scout_2/cmd_vel, scout_3/cmd_vel
# fleet/poses, fleet/formation_cmd

# Watch interleaved pose messages from all scouts
horus topic echo fleet/poses

# Watch a single scout's commands
horus topic echo scout_1/cmd_vel

# Web dashboard
horus monitor
# Open browser to http://localhost:3000

# Parameter inspection
horus param list
horus param get /scout_1/robot_id
```
