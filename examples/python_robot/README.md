# Python Robot — Differential Drive in Python

The same differential-drive example as `differential_drive/`, but written entirely in Python. This is the starting point for Python robotics developers migrating from ROS2.

## What This Demonstrates

- `horus.Node()` — creating nodes with tick callbacks
- `horus.CmdVel` — typed message publishing
- `node.send()` / `node.recv()` — topic communication
- `horus.run()` — one-liner scheduler execution
- Multi-rate nodes (30Hz driver + 10Hz safety monitor)
- Simulated dead-reckoning odometry

## ROS2 Equivalent

Every ROS2 tutorial starts with a Python publisher/subscriber. This is the horus equivalent — same concepts, simpler API:

| ROS2 | Horus |
|------|-------|
| `rclpy.create_node()` | `horus.Node(name=..., tick=...)` |
| `node.create_publisher(CmdVel, 'cmd_vel', 10)` | `pubs=["cmd_vel"]` |
| `node.create_subscription(CmdVel, 'cmd_vel', cb, 10)` | `subs=["cmd_vel"]` |
| `rclpy.spin(node)` | `horus.run(node)` |
| `node.get_logger().info(...)` | `node.log_info(...)` |

## Run

```bash
horus run main.py
```

## Introspection (while running)

```bash
horus topic list          # See active topics
horus topic echo cmd_vel  # Watch velocity commands
horus node list           # See running nodes
```
