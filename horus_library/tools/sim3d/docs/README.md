# Sim3D Documentation

Welcome to the Sim3D documentation. Sim3D is a production-grade 3D robotics simulator built with Bevy and Rapier3D, designed for both visual simulation and high-performance reinforcement learning training.

## Features

- **Dual-mode operation**: Visual 3D rendering + headless RL training
- **URDF support**: Load standard robot descriptions
- **16 sensor types**: LiDAR, cameras, IMU, GPS, force/torque, and more
- **Multi-robot**: Communication, coordination, and swarm behaviors
- **RL-first design**: Vectorized environments, domain randomization, Python bindings
- **HORUS-native**: Direct Hub integration for sim-to-real transfer
- **Performance**: 60 FPS visual / 100K+ steps/sec headless
- **Pure Rust**: Memory-safe, cross-platform, single binary

## Table of Contents

### Getting Started

- **[Getting Started Guide](getting_started.md)** - Installation, first simulation, key concepts, project structure

### Tutorials

Step-by-step guides to learn Sim3D:

1. **[Basic Simulation](tutorials/01_basic_simulation.md)**
   - Create an empty world
   - Add ground plane
   - Spawn rigid bodies (boxes, spheres, cylinders, capsules)
   - Run simulation
   - Complete runnable example

2. **[Robot Simulation](tutorials/02_robot_simulation.md)**
   - Load URDF robot models
   - Control joints (position, velocity)
   - Read joint state
   - Complete example with TurtleBot3

3. **[Sensors](tutorials/03_sensors.md)**
   - Add LiDAR to robot
   - Add Camera (RGB, Depth)
   - Add IMU
   - Add GPS
   - Read sensor data
   - Complete multi-sensor example

4. **[Reinforcement Learning](tutorials/04_reinforcement_learning.md)**
   - Set up RL environment
   - Define observations and actions
   - Implement reward function
   - Use domain randomization
   - Python bindings with Stable-Baselines3

### API Reference

Detailed documentation for all Sim3D APIs:

- **[Physics API](api/physics.md)**
  - PhysicsWorld resource
  - RigidBody configuration (dynamic, static, kinematic)
  - Collision shapes (box, sphere, capsule, cylinder, mesh)
  - Joint types (revolute, prismatic, fixed, spherical)
  - Joint motors and springs
  - Advanced physics (CCD, collision groups, contact events)

- **[Sensors API](api/sensors.md)**
  - All 16 sensor types documented:
    - LiDAR 2D and 3D
    - RGB Camera, Depth Camera, RGBD Camera
    - IMU (Inertial Measurement Unit)
    - GPS with velocity estimation
    - Force/Torque sensor
    - Contact sensor
    - Encoder
    - Magnetometer
    - Barometer
    - Radar
    - Sonar
    - Thermal Camera
    - Event Camera
  - Configuration options for each
  - Data formats and access methods

- **[Multi-Robot API](api/multi_robot.md)**
  - Robot management
  - Communication system (messages, queues, broadcast)
  - Network simulation (latency, packet loss, bandwidth)
  - Swarm coordination (Reynolds flocking)
  - Formation control (line, circle, grid, wedge)
  - Consensus algorithms
  - Task allocation

### Deployment

Production deployment guides:

- **[Cloud Deployment](deployment/cloud.md)**
  - Docker deployment and Dockerfile
  - Docker Compose configuration
  - Kubernetes setup (deployment, service, HPA)
  - AWS deployment (ECS, CloudFormation)
  - GCP deployment (Cloud Run, GKE)
  - Azure deployment (ACI, AKS)
  - Headless rendering (Xvfb, EGL)
  - Performance optimization
  - Monitoring and logging

## Quick Links

### Common Tasks

| Task | Documentation |
|------|---------------|
| Install Sim3D | [Getting Started](getting_started.md#installation) |
| Create a simulation | [Tutorial 1](tutorials/01_basic_simulation.md) |
| Load a URDF robot | [Tutorial 2](tutorials/02_robot_simulation.md) |
| Add sensors | [Tutorial 3](tutorials/03_sensors.md) |
| Train with RL | [Tutorial 4](tutorials/04_reinforcement_learning.md) |
| Deploy to cloud | [Cloud Deployment](deployment/cloud.md) |

### Sensor Quick Reference

| Sensor | Use Case | Output |
|--------|----------|--------|
| LiDAR2D | Obstacle detection, SLAM | Distance array |
| LiDAR3D | 3D mapping, perception | Point cloud |
| Camera | Visual navigation, object detection | RGB image |
| DepthCamera | Depth perception, 3D reconstruction | Depth map |
| IMU | Orientation, motion estimation | Quaternion + velocities |
| GPS | Global localization | Position + velocity |
| ForceTorque | Manipulation, contact | 6-DOF forces |
| Encoder | Odometry | Position + velocity |

### Physics Quick Reference

| Shape | When to Use |
|-------|-------------|
| Box | Rectangular objects, walls |
| Sphere | Balls, wheels (simplified) |
| Capsule | Characters, elongated objects |
| Cylinder | Wheels, cans, poles |
| Mesh | Complex geometry (expensive) |

### RL Environment Types

| Environment | Observation Dim | Action Dim | Description |
|-------------|-----------------|------------|-------------|
| reaching | 10 | 6 | Arm reaching tasks |
| balancing | 8 | 1 | Inverted pendulum |
| locomotion | 12 | 2 | Legged robot walking |
| navigation | 20 | 2 | Point-to-point navigation |
| manipulation | 15 | 7 | Pick and place |
| push | 18 | 2 | Object pushing |

## Architecture Overview

```
sim3d/
├── src/
│   ├── lib.rs              # Library entry point
│   ├── main.rs             # CLI application
│   ├── physics/            # Rapier3D physics integration
│   │   ├── world.rs        # PhysicsWorld resource
│   │   ├── collider.rs     # Collision shapes
│   │   └── joints.rs       # Joint constraints
│   ├── robot/              # Robot loading and control
│   │   ├── urdf_loader.rs  # URDF parser
│   │   └── robot.rs        # Robot component
│   ├── sensors/            # All 16 sensor implementations
│   │   ├── lidar2d.rs
│   │   ├── lidar3d.rs
│   │   ├── camera.rs
│   │   ├── imu.rs
│   │   ├── gps.rs
│   │   └── ...
│   ├── rl/                 # Reinforcement learning
│   │   ├── environment.rs  # RL env interface
│   │   └── domain_randomization.rs
│   ├── multi_robot/        # Multi-robot support
│   │   ├── communication.rs
│   │   ├── coordination.rs
│   │   └── network.rs
│   └── rendering/          # Visual rendering systems
├── python/                 # Python RL bindings
│   ├── sim3d_rl/
│   └── examples/
├── assets/
│   ├── robots/             # URDF robot models
│   ├── scenes/             # Predefined scenes
│   └── objects/            # Object definitions
├── k8s/                    # Kubernetes manifests
└── docs/                   # This documentation
```

## Getting Help

- **Issues**: Report bugs or request features on GitHub
- **Discussions**: Ask questions in GitHub Discussions
- **HORUS Hub**: See real-time integration examples

## Version History

- **0.1.6** - Current version
  - 16 sensor types
  - Multi-robot communication
  - Domain randomization
  - Python bindings for RL
  - Kubernetes deployment support

## License

Sim3D is part of the HORUS project. See the main repository for license information.
