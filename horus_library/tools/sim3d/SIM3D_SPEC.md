# sim3d Technical Specification

**Version:** 1.0
**Status:** Blueprint (Not Yet Implemented)
**Author:** HORUS Team
**Date:** 2025-10-29

---

## Executive Summary

sim3d is a production-grade 3D robotics simulator built with Bevy and Rapier3D, designed as the evolution of HORUS's existing sim2d tool. It provides:

- **Dual-mode operation:** Visual 3D rendering + headless RL training
- **URDF support:** Load standard robot descriptions
- **RL-first design:** Vectorized environments, domain randomization
- **HORUS-native:** Direct Hub integration for perfect sim-to-real transfer
- **Performance target:** 60 FPS visual / 100K+ steps/sec headless
- **Pure Rust:** Memory-safe, cross-platform, single binary

---

## Table of Contents

1. [Tech Stack](#tech-stack)
2. [Architecture](#architecture)
3. [File Structure](#file-structure)
4. [Core Systems](#core-systems)
5. [Feature Implementation](#feature-implementation)
6. [Build Configuration](#build-configuration)
7. [Common Pitfalls & Solutions](#common-pitfalls--solutions)
8. [Testing Strategy](#testing-strategy)
9. [Performance Targets](#performance-targets)
10. [Migration from sim2d](#migration-from-sim2d)

---

## Tech Stack

### Core Dependencies (Cargo.toml)

```toml
[package]
name = "sim3d"
version = "0.1.6"
edition = "2021"
description = "3D robotics simulator with RL support"

[[bin]]
name = "sim3d"
path = "src/main.rs"

[features]
default = ["visual"]
visual = ["bevy/default", "bevy_egui"]
headless = []
editor = ["bevy_editor_pls"]

[dependencies]
# HORUS Core
horus_core = { workspace = true }
horus_library = { workspace = true }

# 3D Physics (CRITICAL: Use exact version to avoid breaking changes)
rapier3d = { version = "0.22", features = ["serde-serialize", "simd-stable"] }
nalgebra = { version = "0.33", features = ["serde-serialize"] }
parry3d = "0.17"  # Collision detection (comes with rapier3d)

# 3D Rendering
bevy = { version = "0.15", default-features = false, features = [
    "bevy_asset",
    "bevy_scene",
    "bevy_winit",
    "bevy_core_pipeline",
    "bevy_pbr",
    "bevy_gltf",
    "bevy_render",
    "bevy_sprite",
    "bevy_text",
    "bevy_ui",
    "multi_threaded",
    "png",
    "hdr",
    "ktx2",
    "zstd",
    "x11",  # Linux
    "wayland",  # Linux alternative
    "tonemapping_luts",
    "default_font",
] }

# GUI & Editor (only with "visual" feature)
bevy_egui = { version = "0.31", optional = true }
bevy_editor_pls = { version = "0.9", optional = true }

# URDF Support
urdf-rs = "0.8"
roxmltree = "0.20"  # XML parsing

# Asset Loading
gltf = "1.4"
image = "0.25"

# Utilities
anyhow = { workspace = true }
thiserror = { workspace = true }
serde = { workspace = true, features = ["derive"] }
serde_yaml = "0.9"
serde_json = "1.0"
bytemuck = { version = "1.18", features = ["derive"] }

# CLI
clap = { workspace = true, features = ["derive"] }

# Parallel Processing (for RL)
rayon = "1.10"

# Python Bindings (for RL)
pyo3 = { version = "0.22", features = ["extension-module"], optional = true }
numpy = { version = "0.22", optional = true }

# Logging
tracing = { workspace = true }
tracing-subscriber = { workspace = true, features = ["env-filter"] }

# Math utilities
glam = "0.29"  # Bevy uses glam internally

[dev-dependencies]
criterion = "0.5"
approx = "0.5"

[build-dependencies]
# None required unless adding shader compilation

[profile.release]
opt-level = 3
lto = "thin"
codegen-units = 1

[profile.dev]
opt-level = 1  # Faster debug builds

[profile.dev.package."*"]
opt-level = 3  # Keep dependencies optimized
```

### System Dependencies

**Ubuntu/Debian:**
```bash
sudo apt install -y \
    pkg-config \
    libx11-dev \
    libxi-dev \
    libxcursor-dev \
    libxrandr-dev \
    libasound2-dev \
    libudev-dev \
    libwayland-dev \
    libxkbcommon-dev
```

**Fedora/RHEL:**
```bash
sudo dnf install -y \
    pkg-config \
    libX11-devel \
    libXi-devel \
    libXcursor-devel \
    libXrandr-devel \
    alsa-lib-devel \
    systemd-devel \
    wayland-devel \
    libxkbcommon-devel
```

**macOS:**
```bash
# Xcode command line tools required
xcode-select --install
```

**Environment Variables (Required):**
```bash
export PKG_CONFIG_ALLOW_SYSTEM_LIBS=1
export PKG_CONFIG_ALLOW_SYSTEM_CFLAGS=1
```

---

## Architecture

### High-Level System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     User Interface Layer                      │
├──────────────────────────────────────────────────────────────┤
│  CLI (clap)              │  GUI (egui)    │  Python (PyO3)   │
│  • sim3d --visual        │  • Debug panel │  • Gym interface │
│  • sim3d --headless      │  • View modes  │  • RL training   │
└────────────┬─────────────┴────────┬───────┴──────────────────┘
             │                      │
┌────────────▼──────────────────────▼───────────────────────────┐
│                   Simulation Core (Bevy ECS)                  │
├───────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────────────┐      ┌─────────────────────┐       │
│  │  Physics System     │      │  Sensor System      │       │
│  │  (Rapier3D)         │      │                     │       │
│  │  • Rigid bodies     │◄────►│  • Lidar3D          │       │
│  │  • Colliders        │      │  • Camera/Depth     │       │
│  │  • Joints           │      │  • IMU              │       │
│  │  • Forces           │      │  • GPS              │       │
│  └─────────┬───────────┘      │  • Force/Torque     │       │
│            │                  └──────────┬──────────┘       │
│            │                             │                  │
│  ┌─────────▼──────────────────────────────▼──────────┐      │
│  │           Transform (TF) System                   │      │
│  │  • Frame hierarchy                                │      │
│  │  • Coordinate transforms                          │      │
│  │  • URDF kinematic chains                          │      │
│  └─────────┬─────────────────────────────────────────┘      │
│            │                                                │
│  ┌─────────▼─────────────────────────────────────────┐      │
│  │           Robot Control System                    │      │
│  │  • Joint controllers (PID)                        │      │
│  │  • Differential drive                             │      │
│  │  • Velocity commands                              │      │
│  └─────────┬─────────────────────────────────────────┘      │
│            │                                                │
│  ┌─────────▼─────────────────────────────────────────┐      │
│  │           HORUS Hub Bridge                        │      │
│  │  • cmd_vel subscription                           │      │
│  │  • /tf publishing                                 │      │
│  │  • Sensor data publishing                         │      │
│  │  • Odometry publishing                            │      │
│  └───────────────────────────────────────────────────┘      │
│                                                               │
└───────────────────────────────────────────────────────────────┘
             │                      │
┌────────────▼──────────┐  ┌───────▼──────────────────────────┐
│  Rendering Pipeline   │  │  Headless Mode (RL)              │
│  (Bevy 3D)            │  │  • Vectorized environments       │
│  • PBR materials      │  │  • Domain randomization          │
│  • Shadows            │  │  • Batch observations            │
│  • Cameras            │  │  • Parallel physics              │
│  • Gizmos (TF, etc)   │  │  • Fast reset (<1ms)             │
└───────────────────────┘  └──────────────────────────────────┘
```

### Data Flow (Visual Mode)

```
User Input (Keyboard/Mouse)
    │
    ▼
┌──────────────┐
│ Bevy Events  │
└──────┬───────┘
       │
       ▼
┌──────────────────────┐     ┌─────────────────┐
│ HORUS Hub (cmd_vel)  │────►│ Robot Controller│
└──────────────────────┘     └────────┬────────┘
                                      │
                                      ▼
                            ┌──────────────────┐
                            │ Rapier3D Physics │
                            │ • Apply forces   │
                            │ • Step simulation│
                            │ • Collision det. │
                            └────────┬─────────┘
                                     │
                    ┌────────────────┼────────────────┐
                    │                │                │
                    ▼                ▼                ▼
            ┌──────────┐    ┌──────────┐    ┌──────────┐
            │ Update   │    │ Update   │    │ Sensor   │
            │ Visuals  │    │ TF Tree  │    │ Reading  │
            └──────────┘    └──────────┘    └─────┬────┘
                                                   │
                                                   ▼
                                        ┌──────────────────┐
                                        │ Publish to HORUS │
                                        │ (odom, /tf, etc) │
                                        └──────────────────┘
```

### Data Flow (Headless RL Mode)

```
Python RL Framework (SB3/RLlib)
    │
    ▼
┌──────────────────────┐
│ Gym Environment      │
│ env.step(action)     │
└──────┬───────────────┘
       │ (PyO3 bridge)
       ▼
┌──────────────────────────────┐
│ Vectorized Rust Backend      │
│ • 1024 parallel environments │
└──────┬───────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│ Batch Action Processing      │
│ (Rayon parallel iterator)    │
└──────┬───────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│ Physics Step (all envs)      │
│ • No rendering               │
│ • Pure computation           │
└──────┬───────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│ Collect Observations         │
│ • Lidar readings             │
│ • Robot state                │
│ • Goal distance              │
└──────┬───────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│ Compute Rewards & Dones      │
└──────┬───────────────────────┘
       │ (Return to Python)
       ▼
Python: obs, reward, done, info
```

---

## File Structure

```
horus_library/tools/sim3d/
├── Cargo.toml                    # Dependencies (see above)
├── SIM3D_SPEC.md                 # This document
├── README.md                     # User-facing documentation
│
├── src/
│   ├── main.rs                   # Entry point, app setup
│   │
│   ├── cli.rs                    # Command-line argument parsing
│   │
│   ├── config/
│   │   ├── mod.rs
│   │   ├── robot.rs              # RobotConfig struct
│   │   ├── world.rs              # WorldConfig struct
│   │   ├── sensors.rs            # SensorConfig struct
│   │   └── physics.rs            # PhysicsConfig struct
│   │
│   ├── physics/
│   │   ├── mod.rs
│   │   ├── world.rs              # PhysicsWorld resource
│   │   ├── rigid_body.rs         # Rigid body management
│   │   ├── collider.rs           # Collision shapes
│   │   ├── joints.rs             # Joint constraints
│   │   ├── controllers.rs        # PID controllers, etc.
│   │   └── diff_drive.rs         # Differential drive kinematics
│   │
│   ├── robot/
│   │   ├── mod.rs
│   │   ├── robot.rs              # Robot component & spawning
│   │   ├── urdf_loader.rs        # URDF parsing & loading
│   │   ├── articulated.rs        # Multi-DOF robot support
│   │   └── state.rs              # Robot state tracking
│   │
│   ├── sensors/
│   │   ├── mod.rs
│   │   ├── lidar3d.rs            # 3D LiDAR simulation
│   │   ├── camera.rs             # RGB camera
│   │   ├── depth.rs              # Depth camera
│   │   ├── rgbd.rs               # RGB-D camera
│   │   ├── imu.rs                # IMU sensor
│   │   ├── gps.rs                # GPS sensor
│   │   ├── encoder.rs            # Wheel encoders
│   │   ├── force_torque.rs       # Force/torque sensors
│   │   └── noise.rs              # Noise models
│   │
│   ├── tf/
│   │   ├── mod.rs
│   │   ├── tree.rs               # TF tree data structure
│   │   ├── visualizer.rs         # TF frame rendering
│   │   ├── publisher.rs          # Publish to HORUS /tf topic
│   │   └── urdf_parser.rs        # Extract TF from URDF
│   │
│   ├── scene/
│   │   ├── mod.rs
│   │   ├── loader.rs             # Scene loading from YAML/JSON
│   │   ├── serializer.rs         # Scene saving
│   │   ├── spawner.rs            # Runtime object spawning
│   │   └── sdf_importer.rs       # Import Gazebo SDF worlds
│   │
│   ├── rendering/
│   │   ├── mod.rs
│   │   ├── setup.rs              # Camera, lights setup
│   │   ├── materials.rs          # PBR materials
│   │   ├── gizmos.rs             # Debug visualization
│   │   └── camera_controller.rs  # Orbital/follow cameras
│   │
│   ├── ui/
│   │   ├── mod.rs
│   │   ├── debug_panel.rs        # Main egui debug window
│   │   ├── view_modes.rs         # View mode toggles
│   │   ├── tf_panel.rs           # TF tree display
│   │   ├── stats_panel.rs        # Performance stats
│   │   └── controls.rs           # User controls
│   │
│   ├── view_modes/
│   │   ├── mod.rs
│   │   ├── tf_mode.rs            # TF frame visualization
│   │   ├── collision_mode.rs     # Collision shape overlay
│   │   ├── sensor_mode.rs        # Sensor FOV visualization
│   │   └── physics_mode.rs       # Force/velocity vectors
│   │
│   ├── horus_bridge/
│   │   ├── mod.rs
│   │   ├── subscriber.rs         # Subscribe to HORUS topics
│   │   ├── publisher.rs          # Publish to HORUS topics
│   │   └── messages.rs           # Message conversions
│   │
│   ├── rl/
│   │   ├── mod.rs
│   │   ├── vectorized.rs         # Parallel environment runner
│   │   ├── gym_bindings.rs       # PyO3 Gymnasium interface
│   │   ├── domain_rand.rs        # Domain randomization
│   │   ├── rewards.rs            # Reward functions
│   │   ├── observations.rs       # Observation space
│   │   └── tasks/
│   │       ├── mod.rs
│   │       ├── navigation.rs     # Navigation task
│   │       ├── manipulation.rs   # Pick & place task
│   │       └── racing.rs         # Racing task
│   │
│   ├── systems/
│   │   ├── mod.rs
│   │   ├── physics_step.rs       # Physics update system
│   │   ├── sensor_update.rs      # Sensor data generation
│   │   ├── tf_update.rs          # TF tree updates
│   │   ├── sync_visual.rs        # Physics  visual sync
│   │   └── horus_sync.rs         # HORUS Hub sync
│   │
│   └── utils/
│       ├── mod.rs
│       ├── conversions.rs        # Type conversions (nalgebra ↔ glam)
│       ├── math.rs               # Math utilities
│       └── string_utils.rs       # String helpers
│
├── configs/
│   ├── robot3d.yaml              # Example robot config
│   ├── world3d.yaml              # Example world config
│   ├── sensors.yaml              # Sensor configurations
│   └── rl_tasks.yaml             # RL task definitions
│
├── assets/
│   ├── models/
│   │   ├── turtlebot3.urdf       # Example robot
│   │   ├── turtlebot3.gltf       # Visual mesh
│   │   └── meshes/               # Mesh files
│   ├── textures/
│   │   ├── ground.png
│   │   └── concrete.png
│   └── scenes/
│       ├── empty_world.yaml
│       └── warehouse.yaml
│
├── python/                        # Python RL interface
│   ├── horus_sim3d/
│   │   ├── __init__.py
│   │   ├── env.py                # Gymnasium environment
│   │   ├── wrappers.py           # Env wrappers
│   │   └── utils.py              # Utilities
│   ├── examples/
│   │   ├── train_ppo.py          # PPO training example
│   │   ├── train_sac.py          # SAC training example
│   │   └── evaluate.py           # Policy evaluation
│   └── setup.py                  # Python package setup
│
├── benches/
│   ├── physics_bench.rs          # Physics performance
│   ├── sensor_bench.rs           # Sensor generation
│   └── vectorized_bench.rs       # RL environment speed
│
├── tests/
│   ├── integration/
│   │   ├── urdf_loading.rs
│   │   ├── physics_accuracy.rs
│   │   └── horus_bridge.rs
│   └── unit/
│       ├── tf_tree.rs
│       ├── kinematics.rs
│       └── sensors.rs
│
└── docs/
    ├── ARCHITECTURE.md           # Architecture details
    ├── URDF_GUIDE.md             # URDF usage guide
    ├── RL_TRAINING.md            # RL training guide
    └── API.md                    # API documentation
```

---

## Core Systems

### 1. Physics System (Rapier3D Integration)

#### PhysicsWorld Resource

```rust
// src/physics/world.rs

use bevy::prelude::*;
use rapier3d::prelude::*;
use nalgebra::Vector3;

#[derive(Resource)]
pub struct PhysicsWorld {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub gravity: Vector3<f32>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 240.0;  // 240 Hz physics

        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity: vector![0.0, -9.81, 0.0],  // Standard gravity
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
        }
    }
}

impl PhysicsWorld {
    pub fn step(&mut self) {
        let physics_hooks = ();
        let event_handler = ();

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,  // Query pipeline (optional)
            &physics_hooks,
            &event_handler,
        );
    }

    pub fn fast_reset(&mut self) {
        // For RL: don't recreate, just reset positions
        for (_, rb) in self.rigid_body_set.iter_mut() {
            if let Some(initial_pos) = rb.user_data as Option<Isometry<f32>> {
                rb.set_position(initial_pos, true);
                rb.set_linvel(vector![0.0, 0.0, 0.0], true);
                rb.set_angvel(vector![0.0, 0.0, 0.0], true);
            }
        }
    }
}
```

#### Physics Step System

```rust
// src/systems/physics_step.rs

use bevy::prelude::*;

pub fn physics_step_system(
    mut physics_world: ResMut<PhysicsWorld>,
    time: Res<Time>,
) {
    // Fixed timestep: always 240 Hz regardless of frame rate
    let dt = 1.0 / 240.0;
    let elapsed = time.delta_seconds();

    // Accumulator pattern for fixed timestep
    static mut ACCUMULATOR: f32 = 0.0;
    unsafe {
        ACCUMULATOR += elapsed;
        while ACCUMULATOR >= dt {
            physics_world.step();
            ACCUMULATOR -= dt;
        }
    }
}
```

**CRITICAL: Avoid Common Rapier3D Pitfalls**

1. **Never share RigidBodyHandle across threads without Arc<Mutex<>>**
2. **Always call `step()` at fixed rate, not variable deltaTime**
3. **Use CCD for fast-moving objects: `rigid_body.enable_ccd(true)`**
4. **Set `user_data` to store Bevy Entity for reverse lookup**

```rust
// CORRECT: Store Bevy entity in rigid body
let rb_handle = physics_world.rigid_body_set.insert(rigid_body);
physics_world.rigid_body_set[rb_handle].user_data = entity.to_bits() as u128;

// INCORRECT: Storing handles in components (leads to desync)
// #[derive(Component)]
// struct PhysicsBody { handle: RigidBodyHandle }  // DON'T DO THIS
```

### 2. TF (Transform Frame) System

#### TF Tree Data Structure

```rust
// src/tf/tree.rs

use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct TransformFrame {
    pub name: String,
    pub parent: Option<String>,
    pub transform: Isometry3<f32>,
    pub children: Vec<String>,
}

#[derive(Resource, Default)]
pub struct TFTree {
    frames: HashMap<String, TransformFrame>,
    root: String,
}

impl TFTree {
    pub fn new(root: impl Into<String>) -> Self {
        let root = root.into();
        let mut frames = HashMap::new();

        frames.insert(root.clone(), TransformFrame {
            name: root.clone(),
            parent: None,
            transform: Isometry3::identity(),
            children: Vec::new(),
        });

        Self { frames, root }
    }

    pub fn add_frame(
        &mut self,
        name: impl Into<String>,
        parent: impl Into<String>,
        transform: Isometry3<f32>,
    ) -> Result<(), String> {
        let name = name.into();
        let parent = parent.into();

        if !self.frames.contains_key(&parent) {
            return Err(format!("Parent frame '{}' does not exist", parent));
        }

        // Add to parent's children list
        if let Some(parent_frame) = self.frames.get_mut(&parent) {
            parent_frame.children.push(name.clone());
        }

        // Insert new frame
        self.frames.insert(name.clone(), TransformFrame {
            name: name.clone(),
            parent: Some(parent),
            transform,
            children: Vec::new(),
        });

        Ok(())
    }

    pub fn update_frame(
        &mut self,
        name: &str,
        transform: Isometry3<f32>,
    ) -> Result<(), String> {
        self.frames
            .get_mut(name)
            .ok_or_else(|| format!("Frame '{}' not found", name))?
            .transform = transform;
        Ok(())
    }

    pub fn get_transform(
        &self,
        from: &str,
        to: &str,
    ) -> Result<Isometry3<f32>, String> {
        if from == to {
            return Ok(Isometry3::identity());
        }

        // Find path from 'from' to root
        let mut from_path = vec![from.to_string()];
        let mut current = from;
        while let Some(frame) = self.frames.get(current) {
            if let Some(parent) = &frame.parent {
                from_path.push(parent.clone());
                current = parent;
            } else {
                break;
            }
        }

        // Find path from 'to' to root
        let mut to_path = vec![to.to_string()];
        current = to;
        while let Some(frame) = self.frames.get(current) {
            if let Some(parent) = &frame.parent {
                to_path.push(parent.clone());
                current = parent;
            } else {
                break;
            }
        }

        // Find common ancestor
        let common_ancestor = from_path
            .iter()
            .find(|f| to_path.contains(f))
            .ok_or("No common ancestor found")?;

        // Compute transform up to common ancestor
        let mut transform = Isometry3::identity();

        // from  common_ancestor (inverse transforms)
        for frame_name in from_path.iter().take_while(|f| *f != common_ancestor) {
            let frame = &self.frames[frame_name];
            transform = frame.transform.inverse() * transform;
        }

        // common_ancestor  to (forward transforms)
        let to_ancestor_path: Vec<_> = to_path
            .iter()
            .take_while(|f| *f != common_ancestor)
            .collect();

        for frame_name in to_ancestor_path.iter().rev() {
            let frame = &self.frames[*frame_name];
            transform = frame.transform * transform;
        }

        Ok(transform)
    }

    pub fn from_urdf(urdf: &urdf_rs::Robot) -> Self {
        let mut tree = TFTree::new("world");

        // Add robot base link
        if let Some(base_link) = urdf.links.first() {
            tree.add_frame(&base_link.name, "world", Isometry3::identity())
                .unwrap();

            // Recursively add child links via joints
            for joint in &urdf.joints {
                if let Some(parent) = tree.frames.get(&joint.parent.link).cloned() {
                    let transform = urdf_origin_to_isometry(&joint.origin);
                    tree.add_frame(&joint.child.link, &parent.name, transform)
                        .unwrap();
                }
            }
        }

        tree
    }
}

fn urdf_origin_to_isometry(origin: &urdf_rs::Pose) -> Isometry3<f32> {
    let translation = Translation3::new(
        origin.xyz[0] as f32,
        origin.xyz[1] as f32,
        origin.xyz[2] as f32,
    );

    let rotation = UnitQuaternion::from_euler_angles(
        origin.rpy[0] as f32,
        origin.rpy[1] as f32,
        origin.rpy[2] as f32,
    );

    Isometry3::from_parts(translation, rotation)
}
```

#### TF Visualization System

```rust
// src/tf/visualizer.rs

use bevy::prelude::*;

#[derive(Resource)]
pub struct TFVisualizer {
    pub enabled: bool,
    pub axis_length: f32,
    pub show_labels: bool,
    pub filter: TFFilter,
}

impl Default for TFVisualizer {
    fn default() -> Self {
        Self {
            enabled: false,
            axis_length: 0.3,
            show_labels: true,
            filter: TFFilter::All,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum TFFilter {
    All,
    OnlyRobot,
    OnlySensors,
    Custom(Vec<String>),
}

pub fn render_tf_frames(
    mut gizmos: Gizmos,
    tf_tree: Res<TFTree>,
    config: Res<TFVisualizer>,
) {
    if !config.enabled {
        return;
    }

    for (name, frame) in tf_tree.frames.iter() {
        if !should_show_frame(name, &config.filter) {
            continue;
        }

        // Convert nalgebra Isometry3 to Bevy Transform
        let transform = isometry_to_bevy_transform(&frame.transform);
        let pos = transform.translation;

        let axis_len = config.axis_length;

        // X axis - RED
        gizmos.arrow(
            pos,
            pos + transform.rotation * Vec3::X * axis_len,
            Color::srgb(1.0, 0.0, 0.0),
        );

        // Y axis - GREEN
        gizmos.arrow(
            pos,
            pos + transform.rotation * Vec3::Y * axis_len,
            Color::srgb(0.0, 1.0, 0.0),
        );

        // Z axis - BLUE
        gizmos.arrow(
            pos,
            pos + transform.rotation * Vec3::Z * axis_len,
            Color::srgb(0.0, 0.0, 1.0),
        );
    }
}

fn isometry_to_bevy_transform(iso: &nalgebra::Isometry3<f32>) -> Transform {
    let translation = Vec3::new(
        iso.translation.x,
        iso.translation.y,
        iso.translation.z,
    );

    let rotation = Quat::from_xyzw(
        iso.rotation.i,
        iso.rotation.j,
        iso.rotation.k,
        iso.rotation.w,
    );

    Transform {
        translation,
        rotation,
        scale: Vec3::ONE,
    }
}

fn should_show_frame(name: &str, filter: &TFFilter) -> bool {
    match filter {
        TFFilter::All => true,
        TFFilter::OnlyRobot => !name.contains("sensor") && name != "world",
        TFFilter::OnlySensors => name.contains("sensor") || name.contains("camera"),
        TFFilter::Custom(names) => names.contains(&name.to_string()),
    }
}
```

### 3. URDF Loader

#### Robot Description Parsing

```rust
// src/robot/urdf_loader.rs

use urdf_rs::{Robot as URDFRobot, read_file};
use bevy::prelude::*;
use anyhow::{Result, Context};

#[derive(Debug)]
pub struct LoadedRobot {
    pub urdf: URDFRobot,
    pub meshes: HashMap<String, Handle<Mesh>>,
    pub materials: HashMap<String, Handle<StandardMaterial>>,
}

pub struct URDFLoader;

impl URDFLoader {
    pub fn load(
        path: &str,
        asset_server: &AssetServer,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<LoadedRobot> {
        // Parse URDF file
        let urdf = read_file(path)
            .with_context(|| format!("Failed to read URDF file: {}", path))?;

        info!("Loaded URDF: {}", urdf.name);
        info!("  Links: {}", urdf.links.len());
        info!("  Joints: {}", urdf.joints.len());

        // Load meshes
        let mut mesh_handles = HashMap::new();
        let mut material_handles = HashMap::new();

        for link in &urdf.links {
            // Load visual meshes
            for visual in &link.visual {
                if let urdf_rs::Geometry::Mesh { filename, scale } = &visual.geometry {
                    let mesh_path = resolve_mesh_path(filename)?;

                    // Load mesh (GLTF/OBJ/STL)
                    let mesh_handle = load_mesh_file(
                        &mesh_path,
                        scale.as_ref(),
                        asset_server,
                        meshes,
                    )?;

                    mesh_handles.insert(link.name.clone(), mesh_handle);
                }
            }

            // Load materials
            for visual in &link.visual {
                if let Some(material) = &visual.material {
                    let mat_handle = create_material(material, materials);
                    material_handles.insert(link.name.clone(), mat_handle);
                }
            }
        }

        Ok(LoadedRobot {
            urdf,
            meshes: mesh_handles,
            materials: material_handles,
        })
    }

    pub fn spawn_robot(
        loaded_robot: &LoadedRobot,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        tf_tree: &mut TFTree,
    ) -> Entity {
        // Create robot entity
        let robot_entity = commands.spawn((
            Robot {
                name: loaded_robot.urdf.name.clone(),
            },
            SpatialBundle::default(),
        )).id();

        // Spawn each link
        for link in &loaded_robot.urdf.links {
            spawn_link(
                link,
                robot_entity,
                loaded_robot,
                commands,
                physics_world,
                tf_tree,
            );
        }

        // Create joints
        for joint in &loaded_robot.urdf.joints {
            create_joint(
                joint,
                robot_entity,
                physics_world,
                tf_tree,
            );
        }

        robot_entity
    }
}

fn resolve_mesh_path(filename: &str) -> Result<String> {
    // Handle package:// URIs
    if filename.starts_with("package://") {
        let path = filename.replace("package://", "assets/");
        Ok(path)
    } else if filename.starts_with("file://") {
        Ok(filename.replace("file://", ""))
    } else {
        Ok(filename.to_string())
    }
}

fn load_mesh_file(
    path: &str,
    scale: Option<&urdf_rs::Vec3>,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
) -> Result<Handle<Mesh>> {
    let extension = std::path::Path::new(path)
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("");

    match extension {
        "gltf" | "glb" => {
            // Bevy loads GLTF automatically
            Ok(asset_server.load(path))
        }
        "stl" => {
            // Parse STL and create Bevy mesh
            load_stl_mesh(path, scale, meshes)
        }
        "obj" => {
            // Parse OBJ
            load_obj_mesh(path, scale, meshes)
        }
        _ => Err(anyhow::anyhow!("Unsupported mesh format: {}", extension)),
    }
}

fn create_material(
    urdf_material: &urdf_rs::Material,
    materials: &mut Assets<StandardMaterial>,
) -> Handle<StandardMaterial> {
    let color = if let Some(rgba) = &urdf_material.color {
        Color::srgba(
            rgba.rgba[0] as f32,
            rgba.rgba[1] as f32,
            rgba.rgba[2] as f32,
            rgba.rgba[3] as f32,
        )
    } else {
        Color::srgb(0.8, 0.8, 0.8)
    };

    materials.add(StandardMaterial {
        base_color: color,
        metallic: 0.1,
        perceptual_roughness: 0.8,
        ..default()
    })
}

// CRITICAL: Handle URDF coordinate system differences
// URDF uses different conventions than Bevy/Rapier
// X-forward, Y-left, Z-up (URDF) vs Y-up (Bevy)
fn urdf_to_bevy_rotation(rpy: &[f64; 3]) -> Quat {
    // Convert RPY (roll-pitch-yaw) to quaternion
    // Then apply coordinate system transformation
    let roll = rpy[0] as f32;
    let pitch = rpy[1] as f32;
    let yaw = rpy[2] as f32;

    Quat::from_euler(EulerRot::XYZ, roll, pitch, yaw)
}
```

**CRITICAL URDF Pitfalls:**

1. **Mesh path resolution:** URDF uses `package://` URIs - must resolve correctly
2. **Coordinate systems:** URDF is Z-up, Bevy is Y-up - requires rotation
3. **Unit scaling:** Check if URDF uses meters (some use mm/cm)
4. **Joint types:** Map URDF joint types to Rapier constraints correctly

### 4. Sensor Systems

#### Lidar3D Sensor

```rust
// src/sensors/lidar3d.rs

use bevy::prelude::*;
use rapier3d::prelude::*;

#[derive(Component)]
pub struct Lidar3D {
    pub horizontal_rays: usize,
    pub vertical_rays: usize,
    pub horizontal_fov: f32,  // radians
    pub vertical_fov: f32,
    pub max_range: f32,
    pub min_range: f32,
    pub rate_hz: f32,
    pub noise_std: f32,
    last_update: f32,
}

impl Default for Lidar3D {
    fn default() -> Self {
        Self {
            horizontal_rays: 720,
            vertical_rays: 16,
            horizontal_fov: std::f32::consts::PI * 2.0,  // 360°
            vertical_fov: std::f32::consts::PI / 6.0,    // 30°
            max_range: 20.0,
            min_range: 0.1,
            rate_hz: 10.0,
            noise_std: 0.01,
            last_update: 0.0,
        }
    }
}

#[derive(Clone)]
pub struct PointCloud {
    pub points: Vec<Vec3>,
    pub intensities: Vec<f32>,
}

pub fn lidar3d_update_system(
    mut query: Query<(&GlobalTransform, &mut Lidar3D)>,
    physics_world: Res<PhysicsWorld>,
    time: Res<Time>,
    mut lidar_pub: ResMut<Hub<PointCloud>>,
) {
    for (transform, mut lidar) in query.iter_mut() {
        let elapsed = time.elapsed_seconds();
        let dt = 1.0 / lidar.rate_hz;

        if elapsed - lidar.last_update < dt {
            continue;
        }

        lidar.last_update = elapsed;

        // Generate point cloud
        let point_cloud = generate_point_cloud(
            &lidar,
            transform,
            &physics_world,
        );

        // Publish to HORUS
        lidar_pub.send(point_cloud, None).ok();
    }
}

fn generate_point_cloud(
    lidar: &Lidar3D,
    transform: &GlobalTransform,
    physics_world: &PhysicsWorld,
) -> PointCloud {
    let mut points = Vec::new();
    let mut intensities = Vec::new();

    let origin = transform.translation();
    let rotation = transform.rotation();

    // Horizontal scan
    for h in 0..lidar.horizontal_rays {
        let h_angle = (h as f32 / lidar.horizontal_rays as f32) * lidar.horizontal_fov
            - lidar.horizontal_fov / 2.0;

        // Vertical scan
        for v in 0..lidar.vertical_rays {
            let v_angle = (v as f32 / lidar.vertical_rays as f32) * lidar.vertical_fov
                - lidar.vertical_fov / 2.0;

            // Ray direction in local frame
            let local_dir = Vec3::new(
                h_angle.cos() * v_angle.cos(),
                v_angle.sin(),
                h_angle.sin() * v_angle.cos(),
            );

            // Transform to world frame
            let world_dir = rotation * local_dir;

            // Cast ray
            if let Some(hit) = cast_ray(
                origin,
                world_dir,
                lidar.max_range,
                physics_world,
            ) {
                if hit.distance >= lidar.min_range {
                    // Add noise
                    let noisy_distance = hit.distance
                        + rand::thread_rng().gen::<f32>() * lidar.noise_std;

                    let point = origin + world_dir * noisy_distance;
                    points.push(point);
                    intensities.push(hit.intensity);
                }
            }
        }
    }

    PointCloud { points, intensities }
}

fn cast_ray(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    physics_world: &PhysicsWorld,
) -> Option<RayHit> {
    let ray = Ray::new(
        nalgebra::Point3::new(origin.x, origin.y, origin.z),
        nalgebra::Vector3::new(direction.x, direction.y, direction.z),
    );

    let max_toi = max_distance;
    let solid = true;
    let filter = QueryFilter::default();

    if let Some((handle, intersection)) = physics_world.query_pipeline.cast_ray(
        &physics_world.rigid_body_set,
        &physics_world.collider_set,
        &ray,
        max_toi,
        solid,
        filter,
    ) {
        Some(RayHit {
            distance: intersection,
            intensity: 1.0,  // Could compute based on material
        })
    } else {
        None
    }
}

struct RayHit {
    distance: f32,
    intensity: f32,
}
```

**CRITICAL Sensor Implementation Notes:**

1. **Ray casting performance:** Use Rapier's `QueryPipeline` for batched rays
2. **Coordinate frames:** Always cast in world frame, not local
3. **Rate limiting:** Use `last_update` to respect sensor frequency
4. **Noise models:** Add Gaussian noise after measurement, not before

### 5. Domain Randomization (RL)

```rust
// src/rl/domain_rand.rs

use bevy::prelude::*;
use rand::Rng;

#[derive(Resource)]
pub struct DomainRandomization {
    pub physics: PhysicsRandomization,
    pub visual: VisualRandomization,
    pub sensor: SensorRandomization,
}

pub struct PhysicsRandomization {
    pub mass_range: (f32, f32),
    pub friction_range: (f32, f32),
    pub restitution_range: (f32, f32),
    pub damping_range: (f32, f32),
}

pub struct VisualRandomization {
    pub lighting_intensity: (f32, f32),
    pub color_hsv_shift: (f32, f32, f32),
    pub texture_variants: Vec<Handle<Image>>,
}

pub struct SensorRandomization {
    pub noise_std_range: (f32, f32),
    pub dropout_probability: f32,
    pub latency_range: (f32, f32),  // milliseconds
}

impl DomainRandomization {
    pub fn randomize_episode(&self, world: &mut World) {
        self.randomize_physics(world);
        self.randomize_visual(world);
        self.randomize_sensors(world);
    }

    fn randomize_physics(&self, world: &mut World) {
        let mut rng = rand::thread_rng();
        let mut physics_world = world.resource_mut::<PhysicsWorld>();

        // Randomize rigid body properties
        for (_, rb) in physics_world.rigid_body_set.iter_mut() {
            // Mass
            let mass_scale = rng.gen_range(self.physics.mass_range.0..=self.physics.mass_range.1);
            rb.set_additional_mass(rb.mass() * (mass_scale - 1.0), true);

            // Damping
            let damping = rng.gen_range(self.physics.damping_range.0..=self.physics.damping_range.1);
            rb.set_linear_damping(damping);
        }

        // Randomize collider properties
        for (_, collider) in physics_world.collider_set.iter_mut() {
            let friction = rng.gen_range(self.physics.friction_range.0..=self.physics.friction_range.1);
            let restitution = rng.gen_range(self.physics.restitution_range.0..=self.physics.restitution_range.1);

            collider.set_friction(friction);
            collider.set_restitution(restitution);
        }
    }

    fn randomize_visual(&self, world: &mut World) {
        let mut rng = rand::thread_rng();

        // Randomize lighting
        let mut lights = world.query::<&mut PointLight>();
        for mut light in lights.iter_mut(world) {
            let intensity_scale = rng.gen_range(
                self.visual.lighting_intensity.0..=self.visual.lighting_intensity.1
            );
            light.intensity *= intensity_scale;
        }

        // Randomize materials (color shift)
        let mut materials = world.resource_mut::<Assets<StandardMaterial>>();
        for (_, material) in materials.iter_mut() {
            let hue_shift = rng.gen_range(
                -self.visual.color_hsv_shift.0..=self.visual.color_hsv_shift.0
            );

            // Apply HSV color shift
            material.base_color = shift_color_hue(material.base_color, hue_shift);
        }
    }

    fn randomize_sensors(&self, world: &mut World) {
        let mut rng = rand::thread_rng();

        // Randomize lidar noise
        let mut lidars = world.query::<&mut Lidar3D>();
        for mut lidar in lidars.iter_mut(world) {
            lidar.noise_std = rng.gen_range(
                self.sensor.noise_std_range.0..=self.sensor.noise_std_range.1
            );
        }
    }
}

fn shift_color_hue(color: Color, hue_shift: f32) -> Color {
    // Convert RGB to HSV, shift hue, convert back
    let [r, g, b, a] = color.to_srgba().to_f32_array();

    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let delta = max - min;

    let mut h = if delta == 0.0 {
        0.0
    } else if max == r {
        60.0 * (((g - b) / delta) % 6.0)
    } else if max == g {
        60.0 * (((b - r) / delta) + 2.0)
    } else {
        60.0 * (((r - g) / delta) + 4.0)
    };

    h = (h + hue_shift).rem_euclid(360.0);

    let s = if max == 0.0 { 0.0 } else { delta / max };
    let v = max;

    // HSV to RGB
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;

    let (r, g, b) = match h as i32 / 60 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };

    Color::srgba(r + m, g + m, b + m, a)
}
```

---

## Build Configuration

### Cargo Features

```toml
[features]
default = ["visual"]

# Visual mode - full 3D rendering
visual = [
    "bevy/default",
    "bevy_egui",
]

# Headless mode - no rendering
headless = []

# Editor tools
editor = [
    "visual",
    "bevy_editor_pls",
]

# Python bindings for RL
python = [
    "pyo3",
    "numpy",
]

# All features
full = [
    "visual",
    "editor",
    "python",
]
```

### Build Commands

```bash
# Visual mode (default)
cargo build --release

# Headless mode (RL training)
cargo build --release --no-default-features --features headless

# With editor
cargo build --release --features editor

# Python bindings
cargo build --release --features python
cd python && pip install -e .
```

### Cross-Platform Considerations

**Linux:**
```bash
# Required environment
export PKG_CONFIG_ALLOW_SYSTEM_LIBS=1
export PKG_CONFIG_ALLOW_SYSTEM_CFLAGS=1

# X11 (default)
cargo build --release

# Wayland
cargo build --release --no-default-features --features "visual,wayland"
```

**macOS:**
```bash
# May need to link against system frameworks
export RUSTFLAGS="-C link-arg=-fuse-ld=lld"
cargo build --release
```

**Windows:**
```powershell
# Visual Studio Build Tools required
cargo build --release
```

---

## Common Pitfalls & Solutions

### 1. Bevy-Rapier Coordinate System Mismatch

**Problem:** Bevy uses Y-up, some physics engines use Z-up

**Solution:**
```rust
// Always use consistent coordinate system
// HORUS standard: Y-up (same as Bevy)

fn spawn_robot(position: Vec3) -> RigidBody {
    // CORRECT: Y is up
    RigidBody::dynamic()
        .translation(vector![position.x, position.y, position.z])

    // WRONG: Don't swap axes
    // .translation(vector![position.x, position.z, position.y])
}
```

### 2. Entity-Handle Desynchronization

**Problem:** Bevy entities and Rapier handles can desync

**Solution:**
```rust
// Store Bevy Entity in Rapier user_data
rigid_body.user_data = entity.to_bits() as u128;

// Retrieve later
fn get_entity_from_handle(handle: RigidBodyHandle, physics: &PhysicsWorld) -> Entity {
    let user_data = physics.rigid_body_set[handle].user_data;
    Entity::from_bits(user_data as u64)
}

// DON'T store RigidBodyHandle in components (causes desync on removal)
```

### 3. Fixed Timestep Physics

**Problem:** Variable deltaTime causes physics instability

**Solution:**
```rust
// CORRECT: Fixed timestep accumulator
fn physics_system(mut physics: ResMut<PhysicsWorld>, time: Res<Time>) {
    const FIXED_DT: f32 = 1.0 / 240.0;
    static mut ACCUMULATOR: f32 = 0.0;

    unsafe {
        ACCUMULATOR += time.delta_seconds();
        while ACCUMULATOR >= FIXED_DT {
            physics.step();
            ACCUMULATOR -= FIXED_DT;
        }
    }
}

// WRONG: Variable timestep
// physics.integration_parameters.dt = time.delta_seconds();  // DON'T
```

### 4. URDF Mesh Loading

**Problem:** Mesh paths in URDF don't resolve

**Solution:**
```rust
fn resolve_urdf_mesh(path: &str) -> PathBuf {
    // Handle different URI schemes
    if path.starts_with("package://") {
        // ROS package path
        PathBuf::from(path.replace("package://", "assets/models/"))
    } else if path.starts_with("file://") {
        PathBuf::from(path.strip_prefix("file://").unwrap())
    } else {
        // Relative to URDF file
        PathBuf::from(path)
    }
}
```

### 5. Memory Leaks in Headless Mode

**Problem:** Creating/destroying environments without cleanup

**Solution:**
```rust
impl VectorizedEnv {
    pub fn reset_env(&mut self, env_idx: usize) {
        // CORRECT: Reuse existing physics world
        self.physics_worlds[env_idx].fast_reset();

        // WRONG: Creating new world every reset
        // self.physics_worlds[env_idx] = PhysicsWorld::new();  // LEAK!
    }
}
```

### 6. Bevy 0.15 Breaking Changes

**Critical changes from Bevy 0.14  0.15:**

1. **Color API changed:**
```rust
// OLD (Bevy 0.14)
Color::rgb(1.0, 0.0, 0.0)

// NEW (Bevy 0.15)
Color::srgb(1.0, 0.0, 0.0)
```

2. **Window setup changed:**
```rust
// OLD
.add_plugins(DefaultPlugins.set(WindowPlugin {
    window: WindowDescriptor {
        title: "sim3d".to_string(),
        ..default()
    },
    ..default()
}))

// NEW
.add_plugins(DefaultPlugins.set(WindowPlugin {
    primary_window: Some(Window {
        title: "sim3d".into(),
        ..default()
    }),
    ..default()
}))
```

---

## Testing Strategy

### Unit Tests

```rust
// tests/unit/tf_tree.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tf_transform_chain() {
        let mut tree = TFTree::new("world");

        tree.add_frame("base", "world",
            Isometry3::translation(1.0, 0.0, 0.0)).unwrap();
        tree.add_frame("sensor", "base",
            Isometry3::translation(0.0, 0.0, 0.5)).unwrap();

        let transform = tree.get_transform("world", "sensor").unwrap();

        assert_approx_eq!(transform.translation.x, 1.0, 0.001);
        assert_approx_eq!(transform.translation.z, 0.5, 0.001);
    }
}
```

### Integration Tests

```rust
// tests/integration/urdf_loading.rs
#[test]
fn test_load_turtlebot3_urdf() {
    let urdf_path = "assets/models/turtlebot3.urdf";
    let robot = URDFLoader::load(urdf_path).unwrap();

    assert_eq!(robot.urdf.name, "turtlebot3");
    assert!(robot.urdf.links.len() > 0);
    assert!(robot.urdf.joints.len() > 0);
}
```

### Benchmarks

```rust
// benches/physics_bench.rs
use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn bench_physics_step(c: &mut Criterion) {
    let mut physics = PhysicsWorld::default();

    // Setup test scenario
    spawn_test_robots(&mut physics, 10);

    c.bench_function("physics_step_10_robots", |b| {
        b.iter(|| {
            physics.step();
        });
    });
}

criterion_group!(benches, bench_physics_step);
criterion_main!(benches);
```

---

## Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| Physics rate | 240 Hz | Fixed timestep |
| Visual FPS | 60 FPS | Frame time < 16.6ms |
| Lidar3D generation | < 5ms | 720x16 rays |
| URDF loading | < 500ms | TurtleBot3 model |
| Headless step rate | 100K+ steps/sec | 1024 parallel envs |
| Memory (visual) | < 500MB | Single robot |
| Memory (headless) | < 50MB/env | Minimal overhead |
| Reset time (RL) | < 1ms | Fast reset optimization |

---

## Migration from sim2d

### Code Reuse

**Can be directly copied:**
- `src/ui/mod.rs`  egui panel structure
- `src/config/`  YAML loading
- `src/horus_bridge/`  Hub communication
- CLI argument parsing

**Needs modification:**
- `src/physics/`  2D  3D (Rapier2D  Rapier3D)
- `src/rendering/`  Sprites  3D meshes
- `src/sensors/`  2D lidar  3D sensors

**New components:**
- `src/tf/`  Transform frames (new)
- `src/robot/urdf_loader.rs`  URDF support (new)
- `src/rl/`  RL features (new)

### Migration Checklist

```bash
# 1. Copy sim2d to sim3d
cp -r horus_library/tools/sim2d horus_library/tools/sim3d

# 2. Update Cargo.toml
# - rapier2d  rapier3d
# - Add urdf-rs, pyo3, etc.

# 3. Find/replace namespaces
# - rapier2d  rapier3d
# - nalgebra::Vector2  nalgebra::Vector3
# - bevy::sprite  bevy::pbr

# 4. Update physics
# - Add Z-axis to all vector operations
# - Update collision shapes (2D  3D)

# 5. Update rendering
# - Remove sprite components
# - Add PBR materials, meshes
# - Setup 3D camera

# 6. Add new systems
# - TF tree
# - URDF loading
# - 3D sensors
# - Domain randomization

# 7. Test build
cargo build --release
```

---

## Appendix: Key Type Conversions

### nalgebra ↔ glam (Bevy)

```rust
// nalgebra Vector3  glam Vec3
fn nalgebra_to_glam(v: nalgebra::Vector3<f32>) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

// glam Vec3  nalgebra Vector3
fn glam_to_nalgebra(v: Vec3) -> nalgebra::Vector3<f32> {
    vector![v.x, v.y, v.z]
}

// nalgebra Isometry3  Bevy Transform
fn isometry_to_transform(iso: nalgebra::Isometry3<f32>) -> Transform {
    Transform {
        translation: Vec3::new(
            iso.translation.x,
            iso.translation.y,
            iso.translation.z,
        ),
        rotation: Quat::from_xyzw(
            iso.rotation.i,
            iso.rotation.j,
            iso.rotation.k,
            iso.rotation.w,
        ),
        scale: Vec3::ONE,
    }
}

// Bevy Transform  nalgebra Isometry3
fn transform_to_isometry(t: Transform) -> nalgebra::Isometry3<f32> {
    let translation = nalgebra::Translation3::new(
        t.translation.x,
        t.translation.y,
        t.translation.z,
    );
    let rotation = nalgebra::UnitQuaternion::from_quaternion(
        nalgebra::Quaternion::new(
            t.rotation.w,
            t.rotation.x,
            t.rotation.y,
            t.rotation.z,
        )
    );
    nalgebra::Isometry3::from_parts(translation, rotation)
}
```

---

## Summary

This specification provides a complete blueprint for implementing sim3d. Key takeaways:

1. **Build on sim2d:** Reuse UI, CLI, HORUS bridge code
2. **Use exact versions:** Bevy 0.15, Rapier3D 0.22 to avoid breaking changes
3. **Dual-mode architecture:** Visual (default) + headless (RL)
4. **TF frames:** Essential for debugging, built from URDF
5. **URDF support:** Core feature for robot ecosystem compatibility
6. **Domain randomization:** Built-in from day 1 for RL
7. **Performance first:** 240Hz physics, 60 FPS visual, 100K+ steps/sec headless
8. **Memory safety:** Pure Rust, no segfaults
9. **Testing:** Unit tests, integration tests, benchmarks
10. **Common pitfalls:** Documented and solved
---

**Document Version:** 1.0
**Last Updated:** 2025-10-29
**Status:** Ready for Implementation
