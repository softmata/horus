# HORUS Message Gaps Roadmap — ROS2 Parity

Fill core IPC message type gaps vs ROS2 standard messages.
All new types follow existing patterns: `#[repr(C)]`, Pod/zero-copy, `timestamp_ns: u64`, `frame_id: [u8; 32]`.

**Scope:** Only types that belong in the core transport layer (geometry, sensors, time).
Domain-specific types (trajectories, visualization, maps) belong in downstream crates.

### What moved out

| Type | Destination | Reason |
|------|-------------|--------|
| Inertia | KINETIC (MoveIt equivalent) | Dynamics/simulation domain |
| JointTrajectory, MultiDOFJointTrajectoryPoint | KINETIC | Motion planning domain |
| Marker, MarkerArray | Talos / `horus_viz` | Visualization is renderer-specific |
| MapMetaData, GridCells | PATHFINDER (Nav2 equivalent) | Navigation stack domain |

---

## Phase 1: Geometry — 3D Pose & Covariance Types

> File: `horus_library/messages/geometry.rs`
> All types are Pod/zero-copy. Reuse existing `Point3`, `Vector3`, `Quaternion`.

### Task 1.1: Add `Pose3D`
Full 6DOF pose (position + orientation).
```rust
#[repr(C)]
pub struct Pose3D {
    pub position: Point3,       // reuse existing
    pub orientation: Quaternion, // reuse existing
    pub timestamp_ns: u64,
}
```
Helpers: `new()`, `from_pose_2d(Pose2D)`, `identity()`, `is_valid()`, `distance_to()`.

### Task 1.2: Add `PoseStamped`
Pose3D with frame context.
```rust
#[repr(C)]
pub struct PoseStamped {
    pub pose: Pose3D,
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```
Helpers: `with_frame_id()`, `new()`.

### Task 1.3: Add `PoseWithCovariance`
For localization (AMCL, EKF output).
```rust
#[repr(C)]
pub struct PoseWithCovariance {
    pub pose: Pose3D,
    pub covariance: [f64; 36], // 6x6: x,y,z,roll,pitch,yaw
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```
Helpers: `with_frame_id()`, `position_variance()`, `orientation_variance()`.

### Task 1.4: Add `TwistWithCovariance`
For sensor fusion (EKF velocity input).
```rust
#[repr(C)]
pub struct TwistWithCovariance {
    pub twist: Twist,          // reuse existing
    pub covariance: [f64; 36], // 6x6: vx,vy,vz,wx,wy,wz
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```
Helpers: `with_frame_id()`, `linear_variance()`, `angular_variance()`.

### Task 1.5: Add `Accel` and `AccelStamped`
Standalone acceleration message.
```rust
#[repr(C)]
pub struct Accel {
    pub linear: [f64; 3],
    pub angular: [f64; 3],
    pub timestamp_ns: u64,
}

#[repr(C)]
pub struct AccelStamped {
    pub accel: Accel,
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```

### Task 1.6: Register all new geometry types as Pod
```rust
crate::messages::impl_pod_message!(
    Pose3D, PoseStamped, PoseWithCovariance, TwistWithCovariance,
    Accel, AccelStamped,
);
```

---

## Phase 2: Sensor — JointState & Environment

> File: `horus_library/messages/sensor.rs`
> Pod/zero-copy with fixed-size arrays.

### Task 2.1: Add `JointState`
Joint feedback message (matches ROS2 sensor_msgs/JointState).
```rust
#[repr(C)]
pub struct JointState {
    pub names: [[u8; 32]; 16],   // up to 16 joints, matches JointCommand
    pub joint_count: u8,
    pub positions: [f64; 16],
    pub velocities: [f64; 16],
    pub efforts: [f64; 16],      // torque or force
    pub timestamp_ns: u64,
}
```
Helpers: `add_joint()`, `joint_name(idx)`, `position(name)`, `velocity(name)`, `effort(name)`.
NOTE: Mirrors `JointCommand` layout for consistency — same 16-joint limit, same name encoding.

### Task 2.2: Add `MagneticField`
Magnetometer data.
```rust
#[repr(C)]
pub struct MagneticField {
    pub magnetic_field: [f64; 3],           // Tesla (x, y, z)
    pub magnetic_field_covariance: [f64; 9], // 3x3
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```

### Task 2.3: Add `Temperature`
Single temperature reading.
```rust
#[repr(C)]
pub struct Temperature {
    pub temperature: f64,    // Celsius
    pub variance: f64,
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```

### Task 2.4: Add `FluidPressure`
Barometer / pressure sensor.
```rust
#[repr(C)]
pub struct FluidPressure {
    pub fluid_pressure: f64, // Pascals
    pub variance: f64,
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```

### Task 2.5: Add `Illuminance`
Light sensor.
```rust
#[repr(C)]
pub struct Illuminance {
    pub illuminance: f64, // Lux
    pub variance: f64,
    pub frame_id: [u8; 32],
    pub timestamp_ns: u64,
}
```

### Task 2.6: Register new sensor types as Pod
```rust
crate::messages::impl_pod_message!(JointState, MagneticField, Temperature, FluidPressure, Illuminance);
```

---

## Phase 3: Clock & Time Synchronization

> New file: `horus_library/messages/clock.rs`

### Task 3.1: Create `clock.rs` module
Add `pub mod clock;` to `messages/mod.rs`.

### Task 3.2: Add `Clock`
Simulation/replay time broadcast.
```rust
#[repr(C)]
pub struct Clock {
    pub clock_ns: u64,       // Current simulation/replay time
    pub realtime_ns: u64,    // Wall clock time for comparison
    pub sim_speed: f64,      // 1.0 = real-time, 2.0 = 2x speed
    pub paused: u8,          // 0 = running, 1 = paused
    pub source: u8,          // SOURCE_WALL=0, SOURCE_SIM=1, SOURCE_REPLAY=2
    pub timestamp_ns: u64,
}
```
Constants: `SOURCE_WALL`, `SOURCE_SIM`, `SOURCE_REPLAY`.
Helpers: `wall_clock()`, `sim_time()`, `elapsed_since()`, `is_paused()`.

### Task 3.3: Add `TimeReference`
External time source sync (GPS time, NTP, PTP).
```rust
#[repr(C)]
pub struct TimeReference {
    pub time_ref_ns: u64,     // External reference time
    pub source: [u8; 32],     // e.g. "gps", "ntp", "ptp"
    pub offset_ns: i64,       // Local - reference offset
    pub timestamp_ns: u64,
}
```

### Task 3.4: Register clock types as Pod
```rust
crate::messages::impl_pod_message!(Clock, TimeReference);
```

---

## Phase 4: Integration & Wiring

### Task 4.1: Update `messages/mod.rs`
- Add `pub mod clock;`
- Add all new types to `impl_pod_message!` calls
- Re-export key types

### Task 4.2: Update horus_library prelude
```rust
// Geometry (new)
pub use crate::messages::geometry::{Pose3D, PoseStamped, PoseWithCovariance, TwistWithCovariance, Accel, AccelStamped};
// Sensor (new)
pub use crate::messages::sensor::{JointState, MagneticField, Temperature, FluidPressure, Illuminance};
// Clock (new)
pub use crate::messages::clock::{Clock, TimeReference};
```

### Task 4.3: Update horus facade prelude
Add key types to `horus/src/lib.rs` prelude re-exports.

### Task 4.4: Update API.md
Document all new message types in the user-facing API reference.

### Task 4.5: Update horus_py bindings
Add Python bindings for the most critical new types: `Pose3D`, `JointState`, `Clock`.

---

## Phase 5: Testing & Verification

### Task 5.1: Unit tests for geometry types
Test `Pose3D::from_pose_2d()`, covariance accessors, `is_valid()`, quaternion normalization.

### Task 5.2: Unit tests for JointState and sensor types
Test `add_joint()`, name lookup, environmental sensor construction.

### Task 5.3: Integration test — Pod round-trip
Verify ALL 14 new Pod types survive `Topic<T>::send()/recv()` with data intact.

### Task 5.4: Build verification and size audit
`cargo build --workspace` and `cargo clippy --workspace` pass with zero warnings.
Print sizeof for each new type.

---

## Summary

| Phase | Types Added | Zero-Copy | Count |
|-------|-----------|-----------|-------|
| 1. Geometry | Pose3D, PoseStamped, PoseWithCovariance, TwistWithCovariance, Accel, AccelStamped | Yes | 6 |
| 2. Sensor | JointState, MagneticField, Temperature, FluidPressure, Illuminance | Yes | 5 |
| 3. Clock | Clock, TimeReference | Yes | 2 |
| **Total** | | **All Pod/Zero-Copy** | **13** |

After completion: HORUS will have **~93+ message types** covering core robotics IPC.

Domain-specific types will live in their respective crates:
- **KINETIC** — JointTrajectory, MultiDOFJointTrajectoryPoint, Inertia
- **PATHFINDER** — MapMetaData, GridCells
- **Talos** — Marker, MarkerArray
