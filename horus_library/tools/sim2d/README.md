# sim2d - Simple 2D Robotics Simulator for HORUS

**One command. Physics + Visualization. Simple control.**

A lightweight 2D robotics simulator built with Bevy and Rapier2D, designed for the HORUS ecosystem. Perfect for testing navigation algorithms, multi-robot coordination, and rapid prototyping.

---

## What is sim2d?

sim2d is a **2D top-down simulator** for testing robot control algorithms:
- **Simple**: One binary, no complex setup
- **Fast**: Rapier2D physics engine (1000+ Hz headless)
- **Visual**: Bevy-based real-time rendering
- **HORUS-native**: Direct Hub topic integration (85-167ns latency)

**Use it for:**
- Navigation algorithm testing
- Path planning development
- Sensor simulation (LiDAR, odometry, IMU) ✅ INTEGRATED
- CI/CD automated testing (headless mode) ✅ IMPLEMENTED
- Dynamic environment testing (runtime obstacle spawning) ✅ NEW

**NOT for:**
- Realistic 3D visualization (use Gazebo/Webots instead)
- Detailed robot modeling
- Camera simulation

---

## Quick Start

### Install System Dependencies
```bash
# Ubuntu/Debian
sudo apt install -y pkg-config libx11-dev libasound2-dev

# Set environment variables
export PKG_CONFIG_ALLOW_SYSTEM_LIBS=1
export PKG_CONFIG_ALLOW_SYSTEM_CFLAGS=1
```

### Run the Simulator

**Terminal 1 - Start sim2d:**
```bash
# Via HORUS CLI (recommended)
horus sim --2d

# Or directly via cargo
cd horus_library/tools/sim2d
cargo run --release

# With custom world image (NEW!)
horus sim --2d --world-image floor_plan.png

# Headless mode for CI/CD (NEW!)
horus sim --2d --headless

# Full configuration
horus sim --2d \
  --world-image map.png \
  --resolution 0.05 \
  --robot configs/robot.yaml \
  --topic my_robot.cmd_vel \
  --headless
```

**Terminal 2 - Control the robot:**
```bash
# Create a simple controller
cat > circle_driver.rs << 'EOF'
use horus::prelude::*;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd_pub: Hub<CmdVel> = Hub::new("cmd_vel")?;
    eprintln!(" Driving in circles...");

    loop {
        cmd_pub.send(CmdVel::new(1.0, 0.5), None)?;
        std::thread::sleep(Duration::from_millis(100));
    }
}
EOF

# Run with horus CLI
horus run circle_driver.rs
```

**What you'll see:**
- Window with 2D top-down view
- Green robot (rectangle) in center
- Gray boundary walls
- Brown obstacles
- Robot moves when you send velocity commands

---

## 📐 How It Works

### Architecture
```
────────────────────────────────────────
         sim2d (Single Binary)          
────────────────────────────────────────
                                        
  ──────────────  ───────────────  
     Rapier2D          Bevy        
     Physics        Rendering      
  ─────────────  ──────────────  
                                      
         ──────────────────          
                                       
         ───────▼───────              
           HORUS Hub                  
           cmd_vel                   
           odom                      
         ───────────────              
────────────────────────────────────────
```

### What Gets Simulated

**Physics (Rapier2D):**
- Differential drive robot kinematics
- Collision detection with walls/obstacles
- Realistic damping and friction
- Deterministic simulation

**Visualization (Bevy):**
- 2D top-down view
- Simple geometric shapes (rectangles, circles)
- Real-time transform updates
- Window resolution: 1200×900

**HORUS Integration:**
- Subscribes to: `cmd_vel` (velocity commands), `sim2d.obstacle_cmd` (dynamic obstacles)
- Publishes: `robot.odom` (odometry), `robot.imu` (IMU), `robot.scan` (LiDAR)
- Direct shared memory communication (85-167ns latency)
- No ROS dependency

**Sensors:**
- **LiDAR**: 720-beam laser scanner with configurable FOV and range
- **IMU**: Linear and angular acceleration with noise simulation
- **Odometry**: Ground-truth pose from physics engine

---

##  Configuration

### Robot Configuration (`configs/robot.yaml`)

Defines robot physical properties:

```yaml
# Physical dimensions
robot:
  width: 0.5          # Robot width (meters)
  length: 0.8         # Robot length (meters)
  max_speed: 2.0      # Maximum velocity (m/s)
  color: [0.2, 0.8, 0.2]  # RGB color [0.0-1.0]

# Sensor configuration (future)
sensors:
  lidar:
    beams: 720          # Number of beams
    fov_deg: 270.0      # Field of view (degrees)
    max_range: 20.0     # Maximum range (meters)
    rate_hz: 10.0       # Update rate (Hz)
```

### World Configuration (`configs/world.yaml`)

Defines world boundaries and obstacles:

```yaml
world:
  width: 20.0         # World width (meters)
  height: 15.0        # World height (meters)

# Obstacles (rectangles and circles)
obstacles:
  # Rectangular obstacle (default shape)
  - pos: [5.0, 5.0]     # Center position [x, y]
    size: [2.0, 1.0]    # Dimensions [width, height]

  # Rectangular obstacle with custom color
  - pos: [-3.0, -2.0]
    size: [1.5, 1.5]
    color: [0.8, 0.2, 0.2]  # RGB (0.0-1.0), red obstacle

  # Circular obstacle
  - pos: [-6.0, 4.0]
    shape: circle       # "rectangle" (default) or "circle"
    size: [1.5, 1.5]    # For circles, first value is radius
    color: [0.9, 0.7, 0.1]  # Optional custom color (yellow)

  - pos: [0.0, 7.0]
    size: [3.0, 0.5]
```

---

## 🆕 Dynamic Obstacle Spawning

Add and remove obstacles at runtime - perfect for dynamic environment testing!

### Quick Start

```bash
# Terminal 1: Run sim2d
cargo run --release

# Terminal 2: Run example script
cd examples
python3 dynamic_obstacles.py simple      # Add 3 colored obstacles
python3 dynamic_obstacles.py grid        # Create 3x3 grid
python3 dynamic_obstacles.py animation   # Animated wave
python3 dynamic_obstacles.py interactive # Manual control
```

### API Usage

```python
from horus import Hub

# Connect to obstacle command topic
hub = Hub("sim2d.obstacle_cmd")

# Add rectangular obstacle
hub.send({
    "action": "add",
    "obstacle": {
        "pos": [3.0, 2.0],        # Position [x, y] in meters
        "shape": "rectangle",     # "rectangle" or "circle"
        "size": [1.5, 1.0],       # [width, height] for rectangles
        "color": [0.8, 0.2, 0.2]  # Optional RGB (0.0-1.0)
    }
})

# Add circular obstacle
hub.send({
    "action": "add",
    "obstacle": {
        "pos": [-2.0, 4.0],
        "shape": "circle",
        "size": [0.8, 0.8],       # [radius, _] for circles
        "color": [0.2, 0.8, 0.2]  # Green
    }
})

# Remove obstacle (10cm position tolerance)
hub.send({
    "action": "remove",
    "obstacle": {
        "pos": [3.0, 2.0],        # Position of obstacle to remove
        "shape": "rectangle",
        "size": [0.0, 0.0]
    }
})
```

See `examples/README.md` for complete documentation and troubleshooting.

---

## 📡 Sensor Integration

sim2d publishes sensor data on HORUS topics - ready for your navigation algorithms!

### Available Sensors

**LiDAR (`robot.scan`)**
```rust
// Subscribe to laser scan data
use horus_library::messages::laser_scan::LaserScan;

let mut scan_sub: Hub<LaserScan> = Hub::new("robot.scan")?;
if let Some(scan) = scan_sub.recv(None)? {
    println!("Got {} ranges, min angle: {}, max angle: {}",
        scan.ranges.len(), scan.angle_min, scan.angle_max);
}
```

**IMU (`robot.imu`)**
```rust
// Subscribe to IMU data
use horus_library::messages::imu::Imu;

let mut imu_sub: Hub<Imu> = Hub::new("robot.imu")?;
if let Some(imu) = imu_sub.recv(None)? {
    println!("Linear accel: {:?}", imu.linear_acceleration);
    println!("Angular vel: {:?}", imu.angular_velocity);
}
```

**Odometry (`robot.odom`)**
```rust
// Subscribe to odometry (ground truth)
use horus_library::messages::odometry::Odometry;

let mut odom_sub: Hub<Odometry> = Hub::new("robot.odom")?;
if let Some(odom) = odom_sub.recv(None)? {
    println!("Position: ({}, {})", odom.pose.position.x, odom.pose.position.y);
    println!("Orientation: {}", odom.pose.orientation.z);
}
```

All sensors work in both GUI and headless modes!

---

## Usage Examples

### Basic Navigation Test
```bash
# Terminal 1: Start simulator
cargo run --release

# Terminal 2: Create a simple test driver
cat > test_driver.rs << 'EOF'
use horus::prelude::*;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd: Hub<CmdVel> = Hub::new("cmd_vel")?;

    loop {
        // Forward 2 seconds
        for _ in 0..20 {
            cmd.send(CmdVel::new(1.0, 0.0), None)?;
            std::thread::sleep(Duration::from_millis(100));
        }
        // Turn 1 second
        for _ in 0..10 {
            cmd.send(CmdVel::new(0.0, 1.5), None)?;
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}
EOF

horus run test_driver.rs
```

### Circle Pattern
```bash
# Terminal 1: Start sim2d
cargo run --release

# Terminal 2: Create and run a simple driver
cat > circle.rs << 'EOF'
use horus::prelude::*;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd: Hub<CmdVel> = Hub::new("cmd_vel")?;
    loop {
        cmd.send(CmdVel::new(1.0, 0.5), None)?;
        std::thread::sleep(Duration::from_millis(100));
    }
}
EOF

horus run circle.rs
```

### Custom World
```bash
# Create custom world configuration
cat > my_world.yaml << 'EOF'
world:
  width: 30.0
  height: 30.0

obstacles:
  - pos: [0.0, 0.0]
    size: [4.0, 4.0]
EOF

# Run with custom world
cargo run --release -- --world my_world.yaml
```

---

## Command-Line Options

```bash
sim2d [OPTIONS]

Options:
  --robot <FILE>          Robot configuration (YAML/TOML)
                          Default: Built-in defaults

  --world <FILE>          World configuration (YAML/TOML)
                          Default: Built-in defaults

  --world-image <FILE>    World image (PNG/JPG/PGM) - NEW!
                          Takes priority over --world
                          Perfect for floor plans and ROS maps

  --resolution <FLOAT>    Image resolution in meters/pixel
                          Default: 0.05 (ROS standard)

  --threshold <0-255>     Obstacle threshold (darker = obstacle)
                          Default: 128

  --topic <NAME>          HORUS topic for velocity commands
                          Default: robot.cmd_vel

  --name <NAME>           Robot name for logging
                          Default: robot

  --headless              Run without GUI (for CI/CD, servers) - NEW!

  -h, --help              Print help
```

## NEW: Image-Based World Loading

Load worlds directly from images - perfect for floor plans, ROS maps, or quick sketches!

### Supported Formats
- **PNG** - Recommended for editing
- **JPEG/JPG** - Photos of floor plans
- **PGM** - ROS occupancy grids (direct compatibility!)

### Quick Example

```bash
# Create test map with Python
python3 << 'EOF'
from PIL import Image, ImageDraw
img = Image.new('L', (400, 400), 255)  # White background
draw = ImageDraw.Draw(img)
draw.rectangle([100, 100, 150, 300], fill=0)  # Black wall
draw.rectangle([250, 50, 350, 100], fill=0)   # Another wall
img.save('test_map.png')
EOF

# Load it in sim2d
horus sim --2d --world-image test_map.png --resolution 0.05
```

### How It Works
1. Image converted to grayscale
2. Pixels **darker than threshold**  obstacles
3. Each obstacle pixel  small square collider
4. World size = `image_size × resolution`

### Parameters

**Resolution** (meters per pixel):
- `0.01` - High detail (10mm/pixel)
- `0.05` - Standard (50mm/pixel)  **ROS default**
- `0.1` - Low detail (100mm/pixel)

**Threshold** (0-255):
- `200` - Strict (light gray = obstacle)
- `128` - Standard  **Default**
- `50` - Permissive (only black = obstacle)

### ROS Map Compatibility

```bash
# Use ROS PGM maps directly!
horus sim --2d \
  --world-image ~/ros_ws/maps/office.pgm \
  --resolution 0.05 \
  --threshold 254
```

## NEW: Headless Mode

Run sim2d without GUI - perfect for CI/CD, servers, and SSH!

```bash
# Headless - no window, physics only
horus sim --2d --headless

# With image
horus sim --2d --headless --world-image map.png

# CI/CD example
horus sim --2d --headless --world-image test_env.png &
SIM_PID=$!
timeout 60s horus run navigation_test.rs
kill $SIM_PID
```

**Performance:**
- GUI Mode: 60 Hz physics, ~150 MB memory, full sensor publishing
- Headless Mode: 1000+ Hz physics, ~30 MB memory, full sensor publishing ✅ IMPLEMENTED

---

## Performance

**Typical performance on modern desktop:**

| Mode | Physics Rate | Memory | Latency | Sensors |
|------|--------------|--------|---------|---------|
| Visual | 60 Hz (limited by render) | ~150 MB | < 1ms | LiDAR, IMU, Odom |
| Headless | 1000+ Hz | ~30 MB | < 200ns | LiDAR, IMU, Odom |

Note: Headless mode provides full sensor publishing for CI/CD testing

---

## Development

### Project Structure
```
sim2d/
── Cargo.toml          # Dependencies and build config
── README.md           # This file
── src/
   ── main.rs         # Single-file simulator (Bevy + Rapier2D + HORUS)
── configs/            # Example configurations
    ── robot.yaml      # Robot physical parameters
    ── world.yaml      # World layout and obstacles
```

### Key Dependencies
- `bevy` - Game engine for rendering and ECS
- `rapier2d` - Physics simulation
- `horus_core` - HORUS Hub communication
- `nalgebra` - Linear algebra for physics

### Building from Source
```bash
# Debug build (faster compile, slower runtime)
cargo build

# Release build (slower compile, faster runtime)
cargo build --release

# Run with logs
RUST_LOG=info cargo run --release
```

---

## How Visualization Works

sim2d renders **simple 2D shapes**, not detailed 3D models:

**What you see:**
- Robot: Green rectangle (configurable size and color)
- Walls: Gray rectangles (world boundaries)
- Obstacles: Brown rectangles (configurable position and size)
- Grid: Optional coordinate grid overlay

**What you DON'T see:**
- 3D meshes or models
- Textures or materials
- Detailed robot parts (wheels, sensors, etc.)
- Realistic lighting/shadows

**Scaling:** Coordinates are scaled 50× for visibility (0.5m robot  25 pixels)

---

## ✨ Features & Capabilities

**Fully Implemented:**
- ✅ Single robot simulation with differential drive kinematics
- ✅ Physics simulation with collisions (Rapier2D)
- ✅ Rectangular and circular obstacles
- ✅ Custom RGB colors per obstacle
- ✅ HORUS topic integration (cmd_vel, odom, imu, scan)
- ✅ Real-time visualization with Bevy
- ✅ **LiDAR sensor** - 720-beam laser scanner with configurable FOV
- ✅ **IMU sensor** - Linear/angular acceleration with noise
- ✅ **Odometry publishing** - Ground-truth pose on `robot.odom`
- ✅ **Headless mode** - For CI/CD and server deployments (1000+ Hz)
- ✅ **Dynamic obstacle spawning** - Add/remove obstacles at runtime via `sim2d.obstacle_cmd`
- ✅ Image-based world loading (PNG/JPG/PGM, perfect for ROS maps)

**In Progress:**
- ⏳ Multi-robot support (planned)
- ⏳ More sensor types (camera, GPS, etc.)

**Not Planned:**
- ❌ 3D visualization (use Gazebo/Webots instead)
- ❌ Detailed robot mesh rendering
- ❌ Realistic lighting/shadows

---

## Integration with HORUS Ecosystem

### As a Development Tool
```bash
# In your robot project
[dependencies]
horus = "0.1.0"

# Terminal 1: Start sim2d
cd ../sim2d && cargo run --release

# Terminal 2: Run your controller
horus run my_controller.rs
```

### For Testing
```rust
// Your navigation algorithm
node! {
    NavigationController {
        pub { cmd_vel: CmdVel -> "cmd_vel" }
        sub { goal: Goal -> "navigation/goal" }

        tick(ctx) {
            let velocity = self.compute_velocity();
            self.cmd_vel.send(velocity, ctx).ok();
        }
    }
}

// Test in sim2d before deploying to real robot!
```

---

## When to Use What

**Use sim2d when:**
- Testing navigation algorithms (A*, DWA, TEB, etc.)
- Developing path planning
- Rapid prototyping of control logic
- CI/CD automated testing
- Learning HORUS framework
- 2D mobile robot simulations

**Use Gazebo/Webots when:**
- Need realistic 3D visualization
- Simulating manipulators/arms
- Camera sensor simulation required
- Human-robot interaction demos
- Marketing/presentation videos
- Complex multi-DOF robots

**Use HORUS + Webots bridge when:**
- Want beautiful visuals + fast HORUS control
- Best of both worlds approach
- See horus-webots package (future)

---

## Additional Resources

**HORUS Documentation:**
- Main HORUS README: `../../README.md`
- HORUS Core API: `../../horus_core/`
- Message Types: `../../horus_library/messages/`

**Learning Path:**
1. Start sim2d: `cargo run --release`
2. Create simple controller publishing to `cmd_vel`
3. Experiment with different velocities
4. Add obstacles in `configs/world.yaml`
5. Test navigation around obstacles
6. Deploy algorithm to real robot!

---

## Troubleshooting

**Build fails with "cannot find -lasound":**
```bash
sudo apt install libasound2-dev
```

**Build fails with X11 errors:**
```bash
sudo apt install libx11-dev libxrandr-dev libxi-dev
```

**"Failed to connect to HORUS" warning:**
- This is normal if running standalone
- sim2d still works, just can't receive external commands
- Make sure HORUS topics are available if you want external control

**Robot doesn't move:**
- Check topic name matches (default: `cmd_vel`)
- Verify CmdVel message format: `{"linear": f32, "angular": f32}`
- Check HORUS Hub connection in logs

**Window too small/large:**
- Edit `src/main.rs`, line ~445: `resolution: (1200.0, 900.0)`
- Rebuild: `cargo build --release`

---

## License

Licensed under Apache License 2.0, same as HORUS framework.

---

## Summary

**sim2d in one sentence:**
A simple, fast 2D robot simulator with full sensor integration (LiDAR, IMU, odometry), dynamic obstacle spawning, and headless CI/CD support - perfect for testing HORUS control algorithms before deploying to real robots.

**Key Features:**
- ✅ Complete sensor suite (LiDAR, IMU, Odometry)
- ✅ Dynamic obstacle spawning via HORUS topics
- ✅ Headless mode for CI/CD (1000+ Hz)
- ✅ Circular and rectangular obstacles with custom colors
- ✅ Image-based world loading (PNG/JPG/PGM)
- ✅ Zero-copy HORUS communication (85-167ns latency)

**Quick commands:**
```bash
# Install dependencies
sudo apt install pkg-config libx11-dev libasound2-dev

# Terminal 1: Run simulator
cd horus_library/tools/sim2d
cargo run --release

# Terminal 2: Create a driver and run it
cat > driver.rs << 'EOF'
use horus::prelude::*;
use horus_library::messages::cmd_vel::CmdVel;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd: Hub<CmdVel> = Hub::new("cmd_vel")?;
    loop {
        cmd.send(CmdVel::new(1.0, 0.5), None)?;
        std::thread::sleep(Duration::from_millis(100));
    }
}
EOF

horus run driver.rs
```

**That's it! Simple, clean, functional.** 
