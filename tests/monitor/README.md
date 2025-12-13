# HORUS Monitor Test Applications

This directory contains two complete robotics applications designed to test the HORUS monitor and package management features.

## Applications

### 1. **robot_fleet_rust** - Fleet Management System (Rust)

A comprehensive mobile robot fleet management system with:

**Sensor Nodes:**
- `CameraNode` - Vision sensor (640x480 RGB)
- `LidarNode` - 360° laser scanner (1° resolution)
- `ImuNode` - Inertial measurement unit
- `GpsNode` - GPS position tracking

**Control Nodes:**
- `NavigationNode` - Waypoint navigation
- `ObstacleAvoidanceNode` - Real-time obstacle detection and avoidance

**Actuator Nodes:**
- `MotorControllerNode` - Differential drive control with odometry feedback

**Monitoring Nodes:**
- `BatteryMonitorNode` - Battery voltage monitoring
- `SystemHealthNode` - System health metrics

**Topics:**
```
sensors.camera           Image
sensors.lidar            LaserScan
sensors.imu              Imu
sensors.gps              GpsCoordinates
sensors.odometry         Odometry
control.cmd_vel          CmdVel
control.cmd_vel_override CmdVel
safety.obstacle_alert    ObstacleAlert
system.battery           BatteryStatus
system.health            SystemHealth
```

---

### 2. **robot_fleet_python** - Warehouse Robot System (Python)

An autonomous warehouse robot with AI vision and task management:

**Vision Nodes:**
- `QrScannerNode` - QR code scanner for inventory
- `ObjectDetectorNode` - Object detection (persons, forklifts, pallets, boxes)

**Localization Nodes:**
- `SlamNode` - SLAM with pose estimation
- `PositionEstimatorNode` - Multi-sensor fusion

**Task Management Nodes:**
- `TaskSchedulerNode` - Task queue management (pick/deliver/return)
- `PathExecutorNode` - Path planning and execution

**Safety Nodes:**
- `CollisionDetectorNode` - Real-time collision warning
- `EmergencyHandlerNode` - Emergency stop and safety overrides

**Performance Monitoring:**
- `PerformanceMonitorNode` - System metrics (CPU, memory, tick rate)

**Topics:**
```
vision.qr_codes          dict (code, confidence)
vision.objects           dict (objects, timestamp)
localization.map         dict (resolution, occupied_cells)
localization.pose        dict (x, y, theta, covariance)
localization.position_estimate  dict (x, y, theta, confidence)
tasks.current_task       dict (id, type, shelf.dock, item)
tasks.status             dict (queue_size, active_task, completed)
control.cmd_vel          dict (linear, angular)
control.cmd_vel_safe     dict (linear, angular)
safety.collision_alert   dict (type, object_class, distance, severity)
safety.status            dict (emergency_active, system_status)
system.performance       dict (uptime, tick_rate, cpu, memory)
```

---

## Running the Applications

### Rust Application

```bash
cd tests/monitor/robot_fleet_rust
horus run
```

Expected output:
```
 Starting Robot Fleet Management System (Rust)
 Monitor available at: http://localhost:8080
 Run 'horus monitor' in another terminal to view

 All nodes registered
 Starting scheduler...
```

### Python Application

```bash
cd tests/monitor/robot_fleet_python
horus run
```

Expected output:
```
🏭 Starting Warehouse Robot System (Python)
 Monitor available at: http://localhost:8080
 Run 'horus monitor' in another terminal to view

 All nodes registered:
   - 2 Vision nodes
   - 2 Localization nodes
   - 2 Task management nodes
   - 2 Safety nodes
   - 1 Performance monitoring node

 Starting scheduler...
```

---

## Testing the Monitor

### Start Monitor (in separate terminal)

```bash
horus monitor
```

or

```bash
horus monitor 8080
```

### What to Test

**1. Node Monitoring:**
- All nodes should appear in the monitor
- Node states: Running, tick counts
- Priority ordering visible
- Real-time tick rate

**2. Topic Inspector:**
- 10+ active topics (Rust) or 12+ topics (Python)
- Message throughput (messages/sec)
- Topic hierarchies (`sensors/`, `control/`, `safety/`, `system/`)

**3. Performance Metrics:**
- IPC latency < 1μs
- Throughput > 1M msg/s
- CPU usage per node
- Memory usage

**4. Log Streaming:**
- Real-time log messages
- Pub/Sub tracking
- Error detection

**5. Package Management:**
- Dependency tree
- Installed packages
- Environment snapshot

### Expected Behavior

**Rust Application:**
- 9 nodes running
- ~10 topics active
- Obstacle alerts trigger when LIDAR detects close objects
- GPS coordinates drift slowly
- Battery drains and recharges

**Python Application:**
- 9 nodes running
- ~12 topics active
- QR codes scanned periodically
- Tasks cycle through pick/deliver/return
- Collision alerts trigger emergency stops
- Performance metrics update every ~3 seconds

---

## Troubleshooting

### Rust app won't compile

```bash
# Check dependencies
cd robot_fleet_rust
cargo check

# Missing rand crate? It should be added automatically
```

### Python app can't import horus

```bash
# Install Python bindings
pip3 install horus

# Or use from cache
export PYTHONPATH=~/.horus/cache/horus@0.1.0/python:$PYTHONPATH
```

### Monitor not showing data

```bash
# Check /dev/shm/horus
ls -la /dev/shm/horus/topics/
ls -la /dev/shm/horus/heartbeats/

# Should see files for each topic and node
```

### Both apps running simultaneously

```bash
# Both apps share the same global topic namespace (ROS-like)
# Use unique topic names/prefixes to avoid conflicts:
horus run app1/main.rs   # Uses topics like "app1.cmd_vel"
horus run app2/main.py   # Uses topics like "app2.cmd_vel"

# All topics visible in /dev/shm/horus/topics/
```

---

## Architecture Comparison

| Feature | Rust App | Python App |
|---------|----------|------------|
| **Domain** | Mobile robot fleet | Warehouse automation |
| **Sensors** | Camera, LIDAR, IMU, GPS | Vision, QR scanner |
| **Control** | Navigation, obstacle avoidance | Task scheduling, path execution |
| **Safety** | Emergency stop | Collision detection, emergency handler |
| **Messages** | Strongly typed (Rust structs) | Dict-based (Python) |
| **Performance** | Lower latency, higher throughput | Easier prototyping |
| **Node Count** | 9 nodes | 9 nodes |
| **Topic Count** | ~10 topics | ~12 topics |

---

## Success Criteria

### Minimal (Must Work)
-  Both apps compile/run without errors
-  All nodes register with scheduler
-  Messages flow between nodes
-  Monitor shows active nodes

### Good (Should Work)
-  Monitor shows all topics
-  Log streaming works
-  Performance metrics visible
-  Can run both simultaneously with session IDs

### Excellent (Nice to Have)
-  Topic inspector shows message content
-  Node graph visualization
-  Real-time latency graphs
-  Package dependency tree

---

## Next Steps

After testing:
1. Document any bugs found
2. Note monitor features that don't work
3. Test on different platforms (Linux, macOS)
4. Measure IPC latency under load
5. Create demo video for YC application

---

**Note:** These are simulation applications. No real hardware is required. All sensor data is generated synthetically for testing purposes.
