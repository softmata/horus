# HORUS API Reference

Complete user-facing API for the HORUS robotics framework.

## Crate Structure

```
horus_types    →  Pod descriptors, TensorDtype, Device, ImageEncoding
     ↑
horus_core     →  Topic, Node, Scheduler, Memory, Actions, Error, Params
     ↑
horus_library  →  Message types, HFrame transforms
     ↑
horus_macros   →  node! macro, LogSummary derive
     ↑
horus          →  Facade crate, prelude re-exports
```

Users depend on `horus` and import via `use horus::prelude::*`.

---

## IPC / Topics

### `Topic<T>`

Primary IPC primitive. Zero-copy shared memory ring buffer.

```rust
let topic: Topic<CmdVel> = Topic::new("cmd_vel")?;
topic.send(&CmdVel::new(1.0, 0.0));
let msg = topic.recv();
```

| Method | Description |
|--------|-------------|
| `Topic::new(name)` | Create or open a topic |
| `Topic::with_capacity(name, cap)` | Create with custom ring buffer size |
| `send(&self, msg)` | Send a message |
| `recv(&self) -> Option<T>` | Receive latest message |
| `has_message(&self) -> bool` | Check if data available |
| `pending_count(&self) -> usize` | Number of pending messages |
| `name(&self) -> &str` | Topic name |
| `metrics(&self) -> TopicMetrics` | Send/recv counters |
| `read_latest(&self) -> Option<T>` | Peek without consuming (requires `T: Copy`) |
| `with_logging(self) -> Self` | Enable debug logging |

Specialized for `Topic<Image>`, `Topic<PointCloud>`, `Topic<DepthImage>` — same `send`/`recv` API, but internally uses pool-backed zero-copy transport.

Specialized for `Topic<HorusTensor>`:

| Method | Description |
|--------|-------------|
| `pool(&self) -> Arc<TensorPool>` | Get the underlying tensor pool |
| `alloc_tensor(&self, ...)` | Allocate a tensor from the pool |
| `send_handle(&self, handle)` | Send a tensor handle |
| `recv_handle(&self) -> Option<TensorHandle>` | Receive a tensor handle |

### `trait TopicMessage`

Defines how types flow through the ring buffer. Blanket impl for all `T: Clone + Send + Sync + Serialize + DeserializeOwned`. Pool-backed types (Image, PointCloud, DepthImage) have specialized impls that transport descriptors instead.

### `unsafe trait PodMessage`

Marker for zero-serialization types. Auto-detected — if a type is `Copy` and has no `Drop`, it goes through the zero-copy path automatically.

| Method | Description |
|--------|-------------|
| `as_bytes(&self) -> &[u8]` | View as raw bytes |
| `from_bytes(bytes) -> &Self` | Reinterpret bytes as Self |
| `zeroed() -> Self` | Zero-initialized instance |
| `is_pod::<T>() -> bool` | Check if a type is Pod at runtime |

### `struct TopicMetrics`

Fields: `messages_sent`, `messages_received`, `send_failures`, `recv_failures`.

---

## Node System

### `trait Node`

Primary lifecycle trait for all HORUS nodes.

```rust
struct MyNode { /* ... */ }
impl Node for MyNode {
    fn name(&self) -> &'static str { "my_node" }
    fn tick(&mut self) { /* called each cycle */ }
}
```

| Method | Required | Description |
|--------|----------|-------------|
| `name() -> &'static str` | Yes | Node name |
| `tick(&mut self)` | Yes | Called each cycle |
| `init(&mut self)` | No | Called once before first tick |
| `shutdown(&mut self)` | No | Called on shutdown |
| `publishers() -> Vec<TopicMetadata>` | No | Declared published topics |
| `subscribers() -> Vec<TopicMetadata>` | No | Declared subscribed topics |
| `on_error(&mut self, error)` | No | Error handler |
| `rate_hz() -> Option<f64>` | No | Desired tick rate |
| `config() -> NodeConfig` | No | Node configuration |
| `is_healthy() -> bool` | No | Health check |
| `checkpoint_state() -> Option<Vec<u8>>` | No | Save state for restore |
| `restore_state(&mut self, data) -> bool` | No | Restore saved state |

### `trait RtNode: Node`

Extends Node for hard/soft real-time guarantees.

| Method | Description |
|--------|-------------|
| `wcet_budget() -> Duration` | Worst-case execution time budget |
| `deadline() -> Duration` | Absolute deadline |
| `rt_priority() -> RtPriority` | Scheduling priority |
| `rt_class() -> RtClass` | Hard / Firm / Soft |
| `deadline_miss_policy() -> DeadlineMissPolicy` | What to do on miss |
| `on_wcet_violation(&mut self, violation)` | WCET overrun handler |
| `on_deadline_miss(&mut self, deadline, actual)` | Deadline miss handler |
| `fallback_node() -> Option<Box<dyn Node>>` | Fallback if this node fails |
| `is_safe_state() -> bool` | Check safety |
| `enter_safe_state(&mut self)` | Enter safe state |

### `trait LogSummary`

One-line summary for debug logging: `fn log_summary(&self) -> String`.

### `enum NodeState`

`Uninitialized`, `Initializing`, `Running`, `Paused`, `Stopping`, `Stopped`, `Error(String)`, `Crashed(String)`

### `enum HealthStatus`

`Healthy`, `Warning`, `Error`, `Critical`, `Unknown`

### `struct NodeConfig`

| Field | Type | Description |
|-------|------|-------------|
| `auto_restart` | `bool` | Auto-restart on crash |
| `max_restarts` | `u32` | Max restart attempts |
| `restart_delay` | `Duration` | Delay between restarts |
| `health_check_interval` | `Duration` | Health check interval |
| `watchdog_timeout` | `Duration` | Watchdog timeout |
| `enable_circuit_breaker` | `bool` | Enable circuit breaker |

### Real-Time Enums

| Type | Variants |
|------|----------|
| `RtPriority` | `Critical`, `High`, `Medium`, `Low`, `Custom(u32)` |
| `RtClass` | `Hard`, `Firm`, `Soft` |
| `DeadlineMissPolicy` | `Warn`, `Skip`, `EmergencyStop`, `Degrade`, `Fallback` |

### `struct RtConfig`

Configure real-time thread settings.

| Method | Description |
|--------|-------------|
| `RtConfig::new()` | Default config |
| `RtConfig::hard_realtime()` | Preset for hard RT |
| `RtConfig::soft_realtime()` | Preset for soft RT |
| `RtConfig::normal()` | No RT settings |
| `apply(&self) -> RtApplyResult` | Apply RT settings to current thread |

### `struct RtConfigBuilder`

| Method | Description |
|--------|-------------|
| `memory_locked(self)` | Lock memory (prevent page faults) |
| `priority(self, p)` | Set thread priority |
| `scheduler(self, s)` | Set scheduler (`Fifo`, `RoundRobin`, `Deadline`) |
| `cpu_affinity(self, cores)` | Pin to specific CPU cores |
| `prefault_stack(self, size)` | Pre-fault stack memory |
| `build(self) -> RtConfig` | Build config |

### RT Free Functions

| Function | Description |
|----------|-------------|
| `pin_thread_to_core(core_id)` | Pin current thread to a CPU core |
| `detect_isolated_cpus() -> Vec<usize>` | Detect kernel-isolated CPUs |
| `get_rt_recommended_cpus() -> Vec<usize>` | Get CPUs recommended for RT |
| `prefault_stack(size)` | Pre-fault stack to avoid page faults |

---

## Scheduler

### `struct Scheduler`

Orchestrates node execution with monitoring, fault tolerance, recording, and replay.

```rust
let mut sched = Scheduler::new().tick_hz(100);
sched.add(MyNode::new()).rate_hz(50.0).done();
sched.run()?;
```

**Constructors:**

| Method | Description |
|--------|-------------|
| `Scheduler::new()` | Default scheduler |
| `Scheduler::deploy()` | Production deployment preset |
| `Scheduler::safety_critical()` | Safety-critical preset |
| `Scheduler::high_performance()` | Performance-optimized preset |
| `Scheduler::deterministic()` | Deterministic execution preset |
| `Scheduler::hard_realtime()` | Hard real-time preset |

**Configuration:**

| Method | Description |
|--------|-------------|
| `tick_hz(self, hz)` | Set tick rate |
| `with_config(self, config)` | Apply full config |
| `with_capacity(self, n)` | Pre-allocate node capacity |
| `with_name(self, name)` | Name the scheduler |

**Node Management:**

| Method | Description |
|--------|-------------|
| `add(node) -> NodeBuilder` | Add node with builder API |
| `set_node_rate(name, hz)` | Change a node's tick rate |

**Execution:**

| Method | Description |
|--------|-------------|
| `run(&mut self)` | Run until stopped |
| `run_for(&mut self, duration)` | Run for a duration |
| `tick(&mut self, names)` | Tick specific nodes once |
| `tick_for(&mut self, names, duration)` | Tick specific nodes for a duration |
| `stop(&self)` | Signal stop |
| `is_running(&self) -> bool` | Check if running |

**OS-Level RT:**

| Method | Description |
|--------|-------------|
| `set_os_priority(priority)` | Set OS thread priority |
| `pin_to_cpu(cpu_id)` | Pin scheduler to CPU |
| `lock_memory()` | Lock all memory (mlock) |
| `prefault_stack(size)` | Pre-fault stack |

**Monitoring:**

| Method | Description |
|--------|-------------|
| `status() -> String` | Human-readable status |
| `node_list() -> Vec<String>` | List node names |
| `node_info(name)` | Get node info |
| `metrics() -> Vec<SchedulerNodeMetrics>` | All node metrics |

**Safety:**

| Method | Description |
|--------|-------------|
| `safety_stats()` | Safety monitoring stats |
| `blackbox()` | Flight recorder access |
| `circuit_state(name)` | Circuit breaker state for a node |
| `circuit_summary()` | (open, half-open, closed) counts |

### `struct NodeBuilder`

Fluent API returned by `Scheduler::add()`.

| Method | Description |
|--------|-------------|
| `order(self, u32)` | Execution order |
| `rate_hz(self, f64)` | Tick rate |
| `rt(self)` | Enable real-time |
| `wcet_us(self, u64)` | WCET budget in microseconds |
| `wcet_ms(self, u64)` | WCET budget in milliseconds |
| `deadline_us(self, u64)` | Deadline in microseconds |
| `deadline_ms(self, u64)` | Deadline in milliseconds |
| `tier(self, NodeTier)` | Priority tier |
| `failure_policy(self, FailurePolicy)` | Failure handling policy |
| `done(self) -> &mut Scheduler` | Finalize registration |

---

## Memory / Tensor Types

### `struct Image`

Pool-backed image with RAII lifetime management.

```rust
let img = Image::new(480, 640, Rgb8);
img.copy_from(&pixel_data);
topic.send(&img);
```

| Method | Description |
|--------|-------------|
| `Image::new(height, width, encoding)` | Allocate from global pool |
| `pixel(row, col) -> &[u8]` | Get pixel data |
| `set_pixel(row, col, data)` | Set pixel data |
| `fill(value)` | Fill entire image |
| `roi(x, y, w, h) -> &[u8]` | Extract region of interest |
| `height() -> u32` | Image height |
| `width() -> u32` | Image width |
| `channels() -> u32` | Number of channels |
| `encoding() -> ImageEncoding` | Pixel encoding |
| `data() -> &[u8]` | Raw pixel data |
| `data_mut() -> &mut [u8]` | Mutable raw data |
| `copy_from(src) -> &mut Self` | Copy data into image |
| `timestamp_ns() / set_timestamp_ns(t)` | Timestamp access |
| `frame_id() / set_frame_id(s)` | Frame ID access |

### `struct PointCloud`

Pool-backed 3D point cloud.

```rust
let pc = PointCloud::new(1024, 3, TensorDtype::F32);
```

| Method | Description |
|--------|-------------|
| `PointCloud::new(num_points, fields_per_point, dtype)` | Allocate |
| `point_at(index) -> &[u8]` | Get raw point data |
| `extract_xyz(index) -> Option<[f32; 3]>` | Extract XYZ coordinates |
| `point_count() -> u64` | Number of points |
| `fields_per_point() -> u32` | Fields per point |
| `is_xyz() -> bool` | Check if XYZ-only format |
| `has_intensity() -> bool` | Check for intensity field |
| `has_color() -> bool` | Check for color fields |
| `data() / data_mut() / copy_from()` | Raw data access |

### `struct DepthImage`

Pool-backed depth map.

```rust
let depth = DepthImage::new(480, 640, TensorDtype::F32);
depth.set_depth(100, 200, 3.5);
```

| Method | Description |
|--------|-------------|
| `DepthImage::new(height, width, dtype)` | Allocate |
| `get_depth(row, col) -> Option<f32>` | Read depth at pixel |
| `set_depth(row, col, value)` | Write depth at pixel |
| `get_depth_u16(row, col) -> Option<u16>` | Read as u16 (millimeters) |
| `depth_statistics() -> (min, max, mean)` | Compute depth stats |
| `height() / width()` | Dimensions |
| `depth_scale() -> f32` | Depth units scale factor |

### `struct TensorPool`

Shared memory pool for tensor allocation.

| Method | Description |
|--------|-------------|
| `TensorPool::new(config)` | Create new pool |
| `TensorPool::open(pool_id)` | Open existing pool |
| `alloc(size, dtype, shape) -> Option<HorusTensor>` | Allocate tensor |
| `retain(&self, tensor)` | Increment refcount |
| `release(&self, tensor)` | Decrement refcount |
| `data_slice(&self, tensor) -> &[u8]` | Get tensor data |
| `data_slice_mut(&self, tensor) -> &mut [u8]` | Get mutable data |
| `stats(&self) -> TensorPoolStats` | Pool usage stats |

### `struct TensorHandle`

RAII wrapper — automatically retains/releases pool slot on clone/drop.

| Method | Description |
|--------|-------------|
| `TensorHandle::alloc(pool, size, dtype, shape)` | Allocate with RAII |
| `data_slice() -> &[u8]` | Raw data |
| `data_as::<T>() -> &[T]` | Typed data view |
| `data_as_mut::<T>() -> &mut [T]` | Mutable typed view |
| `shape() -> &[u64]` | Tensor shape |
| `strides() -> &[u64]` | Tensor strides |
| `dtype() -> TensorDtype` | Data type |
| `numel() -> u64` | Number of elements |
| `nbytes() -> u64` | Size in bytes |
| `slice_first_dim(start, end)` | Slice along first dimension |
| `view(shape)` | Reshape view |

### `struct HorusTensor`

Low-level 232-byte Pod tensor descriptor (used in ring buffer transport).

### `enum TensorDtype`

`F32`, `F64`, `F16`, `BF16`, `I8`, `I16`, `I32`, `I64`, `U8`, `U16`, `U32`, `U64`, `Bool`

| Method | Description |
|--------|-------------|
| `element_size() -> usize` | Bytes per element |
| `numpy_typestr() -> &str` | NumPy type string |
| `parse(s) -> Option<Self>` | Parse from string |
| `is_float() / is_signed_int() / is_unsigned_int()` | Type predicates |

### `enum ImageEncoding`

`Mono8`, `Mono16`, `Rgb8`, `Bgr8`, `Rgba8`, `Bgra8`, `Yuv422`, `Mono32F`, `Rgb32F`, `BayerRggb8`, `Depth16`

| Method | Description |
|--------|-------------|
| `bytes_per_pixel() -> u32` | Bytes per pixel |
| `channels() -> u32` | Number of channels |
| `is_color() -> bool` | Whether it's a color encoding |
| `tensor_dtype() -> TensorDtype` | Corresponding tensor dtype |

### `struct Device`

CPU or CUDA device identifier.

| Method | Description |
|--------|-------------|
| `Device::cpu()` | CPU device |
| `Device::cuda(index)` | CUDA device |
| `Device::CPU` / `Device::CUDA0` | Constants |
| `is_cpu() / is_cuda()` | Device predicates |
| `parse(s) -> Option<Self>` | Parse "cpu", "cuda:0", etc. |

---

## Actions System

Long-running tasks with goal/feedback/result pattern (like ROS 2 actions).

### Defining an Action

```rust
action! {
    NavigateToGoal {
        Goal: NavigateGoal,
        Feedback: NavigateFeedback,
        Result: NavigateResult,
    }
}
```

### `trait Action`

| Associated Type | Description |
|-----------------|-------------|
| `type Goal` | What the client requests |
| `type Feedback` | Progress updates sent to client |
| `type Result` | Final outcome |

### Client Side

```rust
let client = ActionClientBuilder::<NavigateToGoal>::new("navigate")
    .build()?;
let handle = client.send_goal(goal)?;
let result = handle.await_result(Duration::from_secs(30));
```

**`ClientGoalHandle<A>`:**

| Method | Description |
|--------|-------------|
| `goal_id() -> GoalId` | Unique goal identifier |
| `status() -> GoalStatus` | Current status |
| `is_active() / is_done() / is_success()` | Status predicates |
| `result() -> Option<A::Result>` | Get result (if done) |
| `last_feedback() -> Option<A::Feedback>` | Latest feedback |
| `await_result(timeout) -> Option<A::Result>` | Block until result |
| `elapsed() -> Duration` | Time since goal sent |

### Server Side

```rust
let server = ActionServerBuilder::<NavigateToGoal>::new("navigate")
    .on_goal(|goal| GoalResponse::Accept)
    .build()?;
```

**`ServerGoalHandle<A>`:**

| Method | Description |
|--------|-------------|
| `goal() -> &A::Goal` | The requested goal |
| `is_cancel_requested() -> bool` | Client requested cancel |
| `publish_feedback(feedback)` | Send progress update |
| `succeed(self, result) -> GoalOutcome` | Complete successfully |
| `abort(self, result) -> GoalOutcome` | Abort with result |
| `canceled(self, result) -> GoalOutcome` | Mark as canceled |

### Action Enums

| Type | Variants |
|------|----------|
| `GoalStatus` | `Pending`, `Active`, `Succeeded`, `Aborted`, `Canceled`, `Preempted`, `Rejected` |
| `GoalResponse` | `Accept`, `Reject { reason }` |
| `PreemptionPolicy` | `RejectNew`, `PreemptOld`, `Priority`, `Queue { max_size }` |
| `GoalPriority` | Constants: `HIGHEST`, `HIGH`, `NORMAL`, `LOW`, `LOWEST` |

---

## HFrame (Coordinate Transforms)

Lock-free coordinate frame tree with temporal interpolation.

```rust
let tf = HFrame::new();
tf.register_frame("base_link", "world")?;
tf.register_frame("camera", "base_link")?;
tf.update_transform("camera", camera_tf, timestamp_now());
let t = tf.tf("camera", "world")?;
```

### `struct HFrame`

| Method | Description |
|--------|-------------|
| `HFrame::new()` | Default config |
| `HFrame::with_config(config)` | Custom config |
| `HFrame::small() / medium() / large() / massive()` | Size presets |
| `register_frame(name, parent)` | Register dynamic frame |
| `register_static_frame(name, parent, transform)` | Register static frame |
| `unregister_frame(name)` | Remove a frame |
| `update_transform(name, transform, timestamp)` | Update frame transform |
| `set_static_transform(name, transform)` | Update static transform |
| `tf(src, dst) -> HorusResult<Transform>` | Look up transform (latest) |
| `tf_at(src, dst, timestamp)` | Look up transform at time |
| `can_transform(src, dst) -> bool` | Check if transform exists |
| `transform_point(src, dst, point)` | Transform a 3D point |
| `transform_vector(src, dst, vector)` | Transform a 3D vector |
| `parent(name) -> Option<String>` | Get parent frame |
| `children(name) -> Vec<String>` | Get child frames |
| `frame_chain(src, dst)` | Get chain of frames |
| `all_frames() -> Vec<String>` | List all frames |
| `stats() -> HFrameStats` | Usage statistics |

### `struct Transform`

3D rigid body transformation (translation + quaternion).

| Method | Description |
|--------|-------------|
| `Transform::identity()` | Identity transform |
| `Transform::from_translation(t)` | Translation only |
| `Transform::from_rotation(q)` | Rotation only |
| `Transform::from_euler(translation, rpy)` | From roll/pitch/yaw |
| `Transform::from_axis_angle(translation, axis, angle)` | From axis-angle |
| `Transform::from_matrix(4x4)` | From 4x4 matrix |
| `compose(&self, other) -> Transform` | Chain transforms |
| `inverse() -> Transform` | Invert |
| `transform_point(point) -> [f64; 3]` | Apply to point |
| `transform_vector(vector) -> [f64; 3]` | Apply to vector |
| `to_euler() -> [f64; 3]` | Extract roll/pitch/yaw |
| `to_matrix() -> [[f64; 4]; 4]` | Convert to matrix |
| `interpolate(&self, other, t) -> Transform` | SLERP interpolation |

---

## Runtime Parameters

```rust
let params = RuntimeParams::init("my_node");
params.set("max_speed", 1.5);
let speed: f64 = params.get_or("max_speed", 1.0);
```

### `struct RuntimeParams`

| Method | Description |
|--------|-------------|
| `RuntimeParams::init(name)` | Create named param set |
| `get::<T>(key) -> Option<T>` | Get parameter |
| `get_or::<T>(key, default) -> T` | Get with default |
| `set::<T>(key, value)` | Set parameter |
| `has(key) -> bool` | Check existence |
| `list_keys() -> Vec<String>` | List all keys |
| `remove(key)` | Delete a parameter |
| `reset()` | Clear all parameters |
| `save_to_disk(path)` | Persist to disk |
| `load_from_disk(path)` | Load from disk |
| `set_metadata(key, metadata)` | Attach validation rules |
| `set_with_version(key, value, version) -> bool` | Optimistic concurrency |

---

## Error Handling

### `enum HorusError`

| Variant | Description |
|---------|-------------|
| `Io(io::Error)` | I/O error |
| `Config(String)` | Configuration error |
| `Communication(String)` | IPC/topic error |
| `Node(String)` | Node lifecycle error |
| `Memory(String)` | Memory allocation error |
| `Serialization(String)` | Ser/de error |
| `NotFound(String)` | Resource not found |
| `InvalidInput(String)` | Invalid argument |
| `AlreadyExists(String)` | Duplicate resource |
| `Unsupported(String)` | Unsupported operation |
| `Internal(String)` | Internal error |

### `type HorusResult<T> = Result<T, HorusError>`

---

## Macros

### `node!` (proc macro)

Generates a full `Node` impl from a declarative DSL:

```rust
node! {
    struct DriverNode {
        pub { cmd: Topic<CmdVel> }          // publishers
        sub { sensor: Topic<LaserScan> }    // subscribers
        data { speed: f64 = 0.0 }           // internal state

        init { /* called once */ }
        tick { /* called each cycle */ }
        shutdown { /* cleanup */ }
    }
}
```

### `#[derive(LogSummary)]`

Auto-derives `LogSummary` using `Debug` formatting.

### `hlog!()`

Context-aware logging — automatically includes node name and tick number.

### `action!` / `simple_action!` / `standard_action!`

Define action types with Goal/Feedback/Result.

---

## Message Types

### Geometry

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `Twist` | Linear + angular velocity | `new()`, `new_2d()`, `stop()` |
| `Pose2D` | 2D position + heading | `new()`, `origin()` |
| `Point3` | 3D point | `new()`, `origin()` |
| `Vector3` | 3D vector | `new()`, `zero()`, `normalize()`, `dot()`, `cross()` |
| `Quaternion` | Rotation quaternion | `new()`, `identity()`, `from_euler()` |
| `TransformStamped` | Timestamped transform | `new()`, `identity()`, `from_pose_2d()` |

### Sensors

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `LaserScan` | 360-point lidar | `new()`, `angle_at()`, `valid_count()` |
| `Imu` | 9-DOF IMU | `new()`, `set_orientation_from_euler()` |
| `Odometry` | Pose + velocity | `new()`, `update()` |
| `Range` | Single distance measurement | `new()`, `ULTRASONIC`, `INFRARED` |
| `BatteryState` | Battery status | `new()`, `is_low()`, `is_critical()` |
| `NavSatFix` | GPS/GNSS position | `new()`, `from_coordinates()`, `distance_to()` |

### Control

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `CmdVel` | Velocity command | `new(linear, angular)`, `zero()` |
| `MotorCommand` | Direct motor control | `velocity()`, `position()`, `stop()` |
| `DifferentialDriveCommand` | 2-wheel drive | `new()`, `from_twist()` |
| `ServoCommand` | Position servo | `new()`, `from_degrees()` |
| `PwmCommand` | PWM motor | `forward()`, `reverse()`, `brake()` |
| `StepperCommand` | Stepper motor | `steps()`, `position()`, `velocity()`, `home()` |
| `PidConfig` | PID controller gains | `new()`, `proportional()`, `pi()`, `pd()` |
| `JointCommand` | Multi-DOF joint | `new()`, `add_position()`, `add_velocity()` |
| `TrajectoryPoint` | Path waypoint | `new_2d()`, `stationary()` |

### Diagnostics

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `Heartbeat` | Node alive signal | `new()`, `update()` |
| `Status` | General status | `ok()`, `warn()`, `error()`, `fatal()` |
| `EmergencyStop` | Safety e-stop | `engage()`, `release()` |
| `ResourceUsage` | CPU/memory/temp | `new()`, `is_cpu_high()` |
| `DiagnosticReport` | Multi-value report | `new()`, `add_string()`, `add_float()` |
| `SafetyStatus` | Safety system state | `new()`, `is_safe()`, `set_fault()` |

### Navigation

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `Goal` | Navigation target | `new()`, `is_reached()` |
| `Path` | 256-waypoint path | `new()`, `add_waypoint()` |
| `PathPlan` | Fixed-size path plan | `new()`, `add_waypoint()` |
| `OccupancyGrid` | Grid map (heap-allocated) | `new()`, `is_free()`, `is_occupied()` |
| `CostMap` | Navigation costmap (heap-allocated) | `from_occupancy_grid()`, `cost()` |
| `Waypoint` | Path waypoint | `new()`, `with_velocity()` |

### Vision

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `CompressedImage` | JPEG/PNG (heap-allocated) | `new()`, `format_str()` |
| `CameraInfo` | Camera calibration | `new()`, `focal_lengths()`, `principal_point()` |
| `RegionOfInterest` | Image ROI | `new()`, `contains()`, `area()` |

### Detection & Tracking

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `Detection` | 2D object detection | `new()`, `class_name()`, `is_confident()` |
| `Detection3D` | 3D object detection | `new()`, `with_velocity()` |
| `BoundingBox2D` | 2D bbox | `new()`, `center_x()`, `area()`, `iou()` |
| `BoundingBox3D` | 3D bbox | `new()`, `volume()` |
| `TrackedObject` | Tracked object | `new()`, `confirm()`, `update()`, `speed()` |
| `SegmentationMask` | Segmentation mask | `semantic()`, `instance()`, `panoptic()` |
| `LandmarkArray` | Pose landmarks | `coco_pose()`, `mediapipe_pose()`, `mediapipe_hand()` |

### Force/Tactile

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `WrenchStamped` | 6-DOF force/torque | `new()`, `force_only()`, `torque_only()` |
| `TactileArray` | 64-sensor tactile array | `new()`, `total_force()`, `center_of_pressure()` |
| `ImpedanceParameters` | Impedance control | `compliant()`, `stiff()` |
| `ForceCommand` | Force control command | `force_only()`, `hybrid()`, `surface_contact()` |
| `HapticFeedback` | Haptic output | `vibration()`, `force()`, `pulse()` |

### Machine Learning

| Type | Description |
|------|-------------|
| `TensorData` | Generic ML tensor (heap-allocated) |
| `Predictions` | Classification results |
| `InferenceMetrics` | Inference timing |
| `ModelInfo` | Model metadata |
| `FeatureVector` | Embedding vector |
| `Classification` | Top-K classification |
| `LLMRequest` / `LLMResponse` | LLM chat interface |
| `TrainingMetrics` | Training progress |
| `DeploymentConfig` | Model deployment config |

### Input

| Type | Description | Key Constructors |
|------|-------------|------------------|
| `KeyboardInput` | Keyboard event | `new()`, `is_pressed()`, `is_ctrl()`, `key_name()` |
| `JoystickInput` | Gamepad event | `new_button()`, `new_axis()`, `is_connected()` |

### Generic

| Type | Description |
|------|-------------|
| `GenericMessage` | 4KB fixed-size message for dynamic/unknown formats |

---

## Record & Replay

| Type | Description |
|------|-------------|
| `NodeRecorder` | Records node tick inputs/outputs |
| `NodeReplayer` | Replays recorded data |
| `RecordingManager` | Manages recording sessions |
| `ReplayDebugger` | Step-through replay debugger with breakpoints |
| `diff_recordings(a, b)` | Compare two recordings |

---

## Prelude

`use horus::prelude::*` imports everything listed above plus `Arc`, `Mutex`, `Duration`, `Instant`, `Serialize`/`Deserialize`.
