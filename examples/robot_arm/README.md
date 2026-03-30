# Robot Arm

A 6-DOF manipulator that sweeps through predefined waypoints. Demonstrates joint-space control, frame transforms, and services.

## What you'll learn

- Complex URDF loading (7 links, 6 revolute joints with limits)
- Joint-space trajectory interpolation
- Frame transform trees with `TransformFrame`
- Defining services with `service!`
- Querying transforms between frames

## Robot

`robots/arm6dof.urdf` — 7 links (base pedestal through end-effector), 6 revolute joints. Grey base, orange links, red end-effector.

## Running

**Terminal 1 — start the simulator:**

```bash
sim3d --mode visual --robot robots/arm6dof.urdf --world worlds/tabletop.yaml --robot-name arm6dof
```

You should see the arm standing upright on a workbench, with red/green cubes and a blue target zone on the table.

**Terminal 2 — run the controller:**

```bash
horus run main.rs
```

The arm moves through waypoints: home, ready, pick, ready, place, ready, home — then repeats. Each transition takes ~4 seconds with smooth interpolation. Console shows end-effector transform updates.

## Inspecting the system

```bash
# View the frame tree
horus frame tree
# world
# +-- base_link
#     +-- shoulder
#         +-- upper_arm
#             +-- forearm
#                 +-- wrist_1
#                     +-- wrist_2
#                         +-- ee

# Watch end-effector transform live
horus frame echo ee base_link

# Watch joint commands
horus topic echo joint_commands

# Watch end-effector pose
horus topic echo ee_pose

# List available services
horus service list
# Expected: MoveToNamed

# Call the service
horus service call MoveToNamed '{"position_name": 1}'
```

## Key concepts

**Frame transforms**: `FramePublisher` builds an 8-frame tree (`world` through `ee`) and updates transforms from joint angles each tick. Use `horus frame echo` to see live transform queries between any two frames.

**Trajectory interpolation**: `TrajectoryController` linearly interpolates between 7 waypoints (home/ready/pick/place positions). The simplified FK computes an approximate end-effector pose from joint 2 and 3 angles.
