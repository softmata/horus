# Keyboard Input Node

Keyboard input capture with customizable key mapping for teleoperation and control applications.

## Overview

The Keyboard Input Node provides a standardized interface for capturing keyboard events and publishing them to the HORUS system. It supports two modes of operation: terminal mode (using the `crossterm` feature) for real keyboard input capture, and demo mode for testing. The node features customizable key mappings, allowing users to override default bindings for specific applications like teleoperation, gaming controls, or custom command interfaces.

**Key Features**:
- Real-time keyboard event capture (with `crossterm` feature)
- Customizable key mappings
- Support for modifier keys (Ctrl, Alt, Shift, etc.)
- Arrow keys, WASD, function keys, and full keyboard support
- Terminal and GUI mode compatibility
- Built-in teleoperation key bindings

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `keyboard_input` | `KeyboardInput` | Keyboard events with key name, code, modifiers, and timestamp |

## Configuration Parameters

The KeyboardInputNode can be configured programmatically through its public API. There are no runtime configuration topics.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `topic` | `&str` | `"keyboard_input"` | Topic name for publishing keyboard events |
| `terminal_enabled` | `bool` | `true` (with crossterm) | Whether terminal raw mode is enabled for key capture |
| `custom_mapping` | `HashMap` | Default mappings | Custom key mapping overrides |

## Message Types

### KeyboardInput

The keyboard input message structure used for all published events:

```rust
pub struct KeyboardInput {
    pub key_name: [u8; 32],      // Fixed-size key name buffer (null-terminated)
    pub code: u32,               // Raw key code
    pub modifier_flags: u32,     // Bit flags for modifiers
    pub pressed: bool,           // True for press, false for release
    pub timestamp: u64,          // Unix timestamp in milliseconds
    pub _padding: [u8; 16],      // Padding for memory alignment
}

// Modifier bit flags
pub const MODIFIER_CTRL: u32 = 1 << 0;
pub const MODIFIER_ALT: u32 = 1 << 1;
pub const MODIFIER_SHIFT: u32 = 1 << 2;
pub const MODIFIER_SUPER: u32 = 1 << 3;
pub const MODIFIER_HYPER: u32 = 1 << 4;
pub const MODIFIER_META: u32 = 1 << 5;
```

### KeyboardInput Methods

```rust
impl KeyboardInput {
    // Create a new keyboard input event
    pub fn new(key: String, code: u32, modifiers: Vec<String>, pressed: bool) -> Self;

    // Check if a specific modifier is pressed
    pub fn has_modifier(&self, modifier: &str) -> bool;

    // Get the key name as a String
    pub fn get_key_name(&self) -> String;

    // Get modifier names as Vec<String>
    pub fn get_modifiers(&self) -> Vec<String>;

    // Convenience methods for common modifiers
    pub fn is_ctrl(&self) -> bool;
    pub fn is_shift(&self) -> bool;
    pub fn is_alt(&self) -> bool;
}
```

## Public API

### Construction

```rust
use horus_library::nodes::KeyboardInputNode;

// Create with default topic "keyboard_input"
let mut keyboard = KeyboardInputNode::new()?;

// Create with custom topic
let mut keyboard = KeyboardInputNode::new_with_topic("teleop_keys")?;
```

### Key Mapping Methods

```rust
// Override a single key mapping
keyboard.set_key_mapping(
    "w".to_string(),           // Input key
    "Forward".to_string(),     // Key name
    87                         // Keycode
);

// Add multiple key mappings at once
use std::collections::HashMap;
let mut mappings = HashMap::new();
mappings.insert("i".to_string(), ("Forward".to_string(), 73));
mappings.insert("k".to_string(), ("Backward".to_string(), 75));
mappings.insert("j".to_string(), ("TurnLeft".to_string(), 74));
mappings.insert("l".to_string(), ("TurnRight".to_string(), 76));
keyboard.set_key_mappings(mappings);

// Get current mapping for a key
if let Some((name, code)) = keyboard.get_key_mapping("w") {
    eprintln!("Key 'w' maps to {} (code: {})", name, code);
}

// Reset all mappings to defaults
keyboard.reset_mappings();
```

### Keycodes Module

The node provides a comprehensive set of keycode constants:

```rust
use horus_library::nodes::keyboard_input_node::keycodes;

// Letters (A-Z)
keycodes::KEY_A through keycodes::KEY_Z  // 65-90

// Numbers (0-9)
keycodes::KEY_0 through keycodes::KEY_9  // 48-57

// Function keys (F1-F12)
keycodes::KEY_F1 through keycodes::KEY_F12  // 112-123

// Arrow keys
keycodes::KEY_ARROW_UP       // 38
keycodes::KEY_ARROW_DOWN     // 40
keycodes::KEY_ARROW_LEFT     // 37
keycodes::KEY_ARROW_RIGHT    // 39

// Special keys
keycodes::KEY_ENTER          // 13
keycodes::KEY_ESCAPE         // 27
keycodes::KEY_SPACE          // 32
keycodes::KEY_TAB            // 9
keycodes::KEY_BACKSPACE      // 8
keycodes::KEY_DELETE         // 46
keycodes::KEY_HOME           // 36
keycodes::KEY_END            // 35
keycodes::KEY_PAGEUP         // 33
keycodes::KEY_PAGEDOWN       // 34

// Modifiers
keycodes::KEY_SHIFT          // 16
keycodes::KEY_CONTROL        // 17
keycodes::KEY_ALT            // 18

// Numpad
keycodes::KEY_NUMPAD_0 through keycodes::KEY_NUMPAD_9  // 96-105
keycodes::KEY_NUMPAD_ADD         // 107
keycodes::KEY_NUMPAD_SUBTRACT    // 109
keycodes::KEY_NUMPAD_MULTIPLY    // 106
keycodes::KEY_NUMPAD_DIVIDE      // 111
```

## Usage Examples

### Basic Teleoperation

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::KeyboardInput;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create keyboard input node
    let keyboard = KeyboardInputNode::new()?;

    // Create a subscriber to receive keyboard events
    let mut subscriber = Hub::<KeyboardInput>::subscriber("keyboard_input")?;

    runtime.add_node(keyboard);

    // In your control loop or separate node
    while let Ok(key_event) = subscriber.recv() {
        match key_event.get_key_name().as_str() {
            "ArrowUp" | "W" => eprintln!("Move forward"),
            "ArrowDown" | "S" => eprintln!("Move backward"),
            "ArrowLeft" | "A" => eprintln!("Turn left"),
            "ArrowRight" | "D" => eprintln!("Turn right"),
            "Space" => eprintln!("Stop"),
            "Escape" => break,
            _ => {}
        }
    }

    Ok(())
}
```

### Robot Teleoperation with Velocity Control

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::KeyboardInput;
use horus_core::{Node, Runtime, Hub, NodeInfo};

struct TeleopNode {
    keyboard_sub: Hub<KeyboardInput>,
    velocity_pub: Hub<(f32, f32)>,
    linear_vel: f32,
    angular_vel: f32,
}

impl TeleopNode {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            keyboard_sub: Hub::<KeyboardInput>::subscriber("keyboard_input")?,
            velocity_pub: Hub::new("velocity_command")?,
            linear_vel: 0.0,
            angular_vel: 0.0,
        })
    }
}

impl Node for TeleopNode {
    fn name(&self) -> &'static str {
        "TeleopNode"
    }

    fn tick(&mut self, ctx: Option<&mut NodeInfo>) {
        // Process all keyboard events
        while let Ok(key_event) = self.keyboard_sub.try_recv() {
            if !key_event.pressed {
                continue;  // Only handle key press events
            }

            match key_event.get_key_name().as_str() {
                "ArrowUp" | "W" => {
                    self.linear_vel = if key_event.is_shift() { 2.0 } else { 1.0 };
                }
                "ArrowDown" | "S" => {
                    self.linear_vel = if key_event.is_shift() { -2.0 } else { -1.0 };
                }
                "ArrowLeft" | "A" => {
                    self.angular_vel = 1.0;
                }
                "ArrowRight" | "D" => {
                    self.angular_vel = -1.0;
                }
                "Space" => {
                    self.linear_vel = 0.0;
                    self.angular_vel = 0.0;
                }
                _ => {}
            }
        }

        // Publish velocity command
        let _ = self.velocity_pub.send((self.linear_vel, self.angular_vel), ctx);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    runtime.add_node(KeyboardInputNode::new()?);
    runtime.add_node(TeleopNode::new()?);

    runtime.run()?;
    Ok(())
}
```

### Custom Key Bindings for Gaming

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::nodes::keyboard_input_node::keycodes;
use std::collections::HashMap;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut keyboard = KeyboardInputNode::new()?;

    // Custom gaming controls: IJKL instead of WASD
    let mut custom_bindings = HashMap::new();
    custom_bindings.insert("i".to_string(), ("MoveUp".to_string(), keycodes::KEY_I));
    custom_bindings.insert("k".to_string(), ("MoveDown".to_string(), keycodes::KEY_K));
    custom_bindings.insert("j".to_string(), ("MoveLeft".to_string(), keycodes::KEY_J));
    custom_bindings.insert("l".to_string(), ("MoveRight".to_string(), keycodes::KEY_L));

    // Action keys
    custom_bindings.insert("f".to_string(), ("Fire".to_string(), keycodes::KEY_F));
    custom_bindings.insert("r".to_string(), ("Reload".to_string(), keycodes::KEY_R));
    custom_bindings.insert("e".to_string(), ("Interact".to_string(), keycodes::KEY_E));

    keyboard.set_key_mappings(custom_bindings);

    // Now use the keyboard node with custom bindings
    // ...

    Ok(())
}
```

### Drone Teleoperation with Multi-Key Support

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::KeyboardInput;
use horus_core::{Node, Runtime, Hub, NodeInfo};

struct DroneControlNode {
    keyboard_sub: Hub<KeyboardInput>,
    control_pub: Hub<(f32, f32, f32, f32)>,  // (pitch, roll, yaw, throttle)
    pitch: f32,
    roll: f32,
    yaw: f32,
    throttle: f32,
}

impl Node for DroneControlNode {
    fn name(&self) -> &'static str {
        "DroneControlNode"
    }

    fn tick(&mut self, ctx: Option<&mut NodeInfo>) {
        while let Ok(key_event) = self.keyboard_sub.try_recv() {
            if !key_event.pressed {
                continue;
            }

            match key_event.get_key_name().as_str() {
                // Pitch (forward/backward)
                "W" | "ArrowUp" => self.pitch = 1.0,
                "S" | "ArrowDown" => self.pitch = -1.0,

                // Roll (left/right)
                "A" | "ArrowLeft" => self.roll = -1.0,
                "D" | "ArrowRight" => self.roll = 1.0,

                // Yaw (rotate)
                "Q" => self.yaw = -1.0,
                "E" => self.yaw = 1.0,

                // Throttle (altitude)
                "Space" => self.throttle = 1.0,
                "C" => self.throttle = -1.0,

                // Emergency stop
                "X" => {
                    self.pitch = 0.0;
                    self.roll = 0.0;
                    self.yaw = 0.0;
                    self.throttle = 0.0;
                }

                _ => {}
            }
        }

        // Apply decay for smooth control
        self.pitch *= 0.9;
        self.roll *= 0.9;
        self.yaw *= 0.9;
        self.throttle *= 0.95;

        // Publish control command
        let _ = self.control_pub.send(
            (self.pitch, self.roll, self.yaw, self.throttle),
            ctx
        );
    }
}
```

### Function Key Commands

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::KeyboardInput;
use horus_core::Hub;

fn setup_command_handler() -> Result<(), Box<dyn std::error::Error>> {
    let keyboard = KeyboardInputNode::new()?;
    let mut subscriber = Hub::<KeyboardInput>::subscriber("keyboard_input")?;

    while let Ok(key_event) = subscriber.recv() {
        if !key_event.pressed {
            continue;
        }

        // Function key commands
        match key_event.get_key_name().as_str() {
            "F1" => eprintln!("Help menu"),
            "F2" => eprintln!("Save state"),
            "F3" => eprintln!("Load state"),
            "F4" => eprintln!("Settings"),
            "F5" => eprintln!("Refresh"),
            "F6" => eprintln!("Toggle mode"),
            "F7" => eprintln!("Debug info"),
            "F8" => eprintln!("Performance stats"),
            "F9" => eprintln!("Screenshot"),
            "F10" => eprintln!("Toggle fullscreen"),
            "F11" => eprintln!("Console"),
            "F12" => eprintln!("Developer tools"),
            _ => {}
        }

        // Ctrl+Key combinations
        if key_event.is_ctrl() {
            match key_event.get_key_name().as_str() {
                "S" => eprintln!("Save (Ctrl+S)"),
                "O" => eprintln!("Open (Ctrl+O)"),
                "Z" => eprintln!("Undo (Ctrl+Z)"),
                "Y" => eprintln!("Redo (Ctrl+Y)"),
                "C" => eprintln!("Copy (Ctrl+C)"),
                "V" => eprintln!("Paste (Ctrl+V)"),
                _ => {}
            }
        }
    }

    Ok(())
}
```

## Key Bindings

### Default Key Bindings

The node comes with pre-configured default mappings for common use cases:

| Input Key(s) | Key Name | Keycode | Description |
|-------------|----------|---------|-------------|
| `up`, `arrowup` | ArrowUp | 38 | Up arrow key |
| `down`, `arrowdown` | ArrowDown | 40 | Down arrow key |
| `left`, `arrowleft` | ArrowLeft | 37 | Left arrow key |
| `right`, `arrowright` | ArrowRight | 39 | Right arrow key |
| `w` | W | 87 | WASD forward |
| `a` | A | 65 | WASD left |
| `s` | S | 83 | WASD backward |
| `d` | D | 68 | WASD right |
| `space`, ` ` | Space | 32 | Space bar |
| `enter`, `return` | Enter | 13 | Enter key |
| `escape`, `esc` | Escape | 27 | Escape key |
| `tab` | Tab | 9 | Tab key |
| `a`-`z` | A-Z | 65-90 | Letter keys |
| `0`-`9` | 0-9 | 48-57 | Number keys |
| `f1`-`f12` | F1-F12 | 112-123 | Function keys |

### Custom Key Bindings

You can override any default binding or create new ones:

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::nodes::keyboard_input_node::keycodes;

let mut keyboard = KeyboardInputNode::new()?;

// Override 'w' to map to "Throttle" instead of "W"
keyboard.set_key_mapping(
    "w".to_string(),
    "Throttle".to_string(),
    keycodes::KEY_W
);

// Add custom binding for a symbol key
keyboard.set_key_mapping(
    "/".to_string(),
    "Command".to_string(),
    191  // KEY_SLASH
);
```

### Teleoperation Key Binding Recommendations

| Operation | Recommended Keys | Alternative |
|-----------|-----------------|-------------|
| Forward | W, Up Arrow | I, Numpad 8 |
| Backward | S, Down Arrow | K, Numpad 2 |
| Left | A, Left Arrow | J, Numpad 4 |
| Right | D, Right Arrow | L, Numpad 6 |
| Rotate CCW | Q | Numpad 7 |
| Rotate CW | E | Numpad 9 |
| Stop/Brake | Space, X | Numpad 5 |
| Turbo/Fast | Shift (modifier) | Ctrl (modifier) |
| Precision/Slow | Alt (modifier) | - |
| Emergency Stop | Escape | Backspace |

## Terminal vs GUI Mode

### Terminal Mode (with `crossterm` feature)

When the `crossterm` feature is enabled, the node operates in terminal mode:

**Characteristics**:
- Captures real keyboard input from the terminal
- Requires raw terminal mode (automatically enabled)
- Non-blocking keyboard event polling
- Supports all standard keyboard keys and modifiers
- Automatically handles terminal cleanup on exit
- Press ESC or Ctrl+C to disable raw mode and exit

**Enabling crossterm**:
```toml
# Cargo.toml
[dependencies]
horus_library = { version = "0.1", features = ["crossterm"] }
```

**Behavior**:
- Raw mode is enabled on node creation
- Terminal displays message: "Terminal keyboard input enabled..."
- ESC key disables raw mode and returns to normal terminal
- Ctrl+C triggers graceful shutdown
- Raw mode automatically disabled when node is dropped

### Demo Mode (without `crossterm`)

When `crossterm` is not enabled, the node runs in demo/simulation mode:

**Characteristics**:
- Simulates keyboard input for testing
- Cycles through arrow keys every 2 seconds
- Useful for automated testing and demos
- No real keyboard input required

**Use Cases**:
- Automated testing
- CI/CD pipelines
- Demonstrations without user interaction
- Integration tests

### Mode Comparison

| Feature | Terminal Mode | Demo Mode |
|---------|--------------|-----------|
| Real keyboard input | Yes | No (simulated) |
| User interaction | Required | Automatic |
| Terminal setup | Raw mode | Normal |
| Testing | Interactive | Automated |
| Dependencies | crossterm | None |
| Key support | Full keyboard | Arrow keys only |
| Modifiers | Yes | No |

## Troubleshooting

### Issue: No keyboard events received

**Cause**: Terminal raw mode not enabled or crossterm feature not compiled

**Solution**:
```bash
# Check if crossterm feature is enabled
cargo build --features crossterm

# Or add to Cargo.toml
[dependencies]
horus_library = { version = "0.1", features = ["crossterm"] }
```

**Verification**:
```rust
// Look for this message on startup:
// " Terminal keyboard input enabled. Press arrow keys to control, ESC or Ctrl+C to quit."
```

### Issue: Terminal behaves strangely after program exit

**Cause**: Raw mode not properly disabled due to program crash or forced termination

**Solution**:
```bash
# Reset terminal manually
reset

# Or use stty
stty sane
```

**Prevention**:
```rust
// The node automatically disables raw mode on Drop
// Ensure proper cleanup by avoiding forced kills (SIGKILL)
// Use Ctrl+C (SIGINT) or ESC for graceful shutdown
```

### Issue: Some keys not working

**Cause**: Terminal or platform limitations, key already bound by OS

**Solution**:
```rust
// Use alternative key bindings
keyboard.set_key_mapping(
    "h".to_string(),  // Use different key
    "Help".to_string(),
    keycodes::KEY_H
);

// Or check if modifiers are interfering
if key_event.get_modifiers().is_empty() {
    // Process only unmodified keys
}
```

### Issue: Custom mappings not working

**Cause**: Input key case sensitivity or mapping after node creation

**Solution**:
```rust
// Input keys are automatically lowercased
keyboard.set_key_mapping(
    "w".to_string(),  // Use lowercase
    "Forward".to_string(),
    87
);

// Set mappings BEFORE adding node to runtime
let mut keyboard = KeyboardInputNode::new()?;
keyboard.set_key_mapping(...);  // Configure first
runtime.add_node(keyboard);      // Then add to runtime
```

### Issue: Ctrl+C doesn't exit

**Cause**: Ctrl+C is intercepted by the node for custom handling

**Solution**:
```rust
// The node handles Ctrl+C by default and exits
// If you need custom Ctrl+C handling:
while let Ok(key_event) = subscriber.recv() {
    if key_event.has_modifier("Ctrl") &&
       key_event.get_key_name() == "C" {
        eprintln!("Custom cleanup...");
        std::process::exit(0);
    }
}

// Or use ESC as alternative exit key
```

### Issue: High CPU usage

**Cause**: Tight polling loop in tick method

**Solution**:
```rust
// The node uses non-blocking polling (Duration::from_millis(0))
// CPU usage should be minimal
// If experiencing high CPU, check your subscriber loop:

// Bad - tight loop
while let Ok(key_event) = subscriber.recv() {
    // Process
}

// Good - with timeout or tick-based
while let Ok(key_event) = subscriber.recv_timeout(Duration::from_millis(10)) {
    // Process
}
```

### Issue: Modifiers not detected

**Cause**: Modifier state not captured at key press time

**Solution**:
```rust
// Check modifiers immediately when processing event
if key_event.pressed {
    if key_event.has_modifier("Ctrl") {
        eprintln!("Ctrl is pressed");
    }
    if key_event.has_modifier("Shift") {
        eprintln!("Shift is pressed");
    }
    if key_event.has_modifier("Alt") {
        eprintln!("Alt is pressed");
    }
}

// Use convenience methods
if key_event.is_ctrl() && key_event.get_key_name() == "S" {
    // Handle Ctrl+S
}
```

## Integration Examples

### Integration with Motor Control

```rust
use horus_library::nodes::{KeyboardInputNode, MotorControllerNode};
use horus_library::KeyboardInput;
use horus_core::{Node, Runtime, Hub, NodeInfo};

struct KeyboardMotorBridge {
    keyboard_sub: Hub<KeyboardInput>,
    motor_pub: Hub<(u8, f32)>,  // (motor_id, speed)
}

impl Node for KeyboardMotorBridge {
    fn name(&self) -> &'static str {
        "KeyboardMotorBridge"
    }

    fn tick(&mut self, ctx: Option<&mut NodeInfo>) {
        while let Ok(key_event) = self.keyboard_sub.try_recv() {
            if !key_event.pressed {
                continue;
            }

            let speed = if key_event.is_shift() { 100.0 } else { 50.0 };

            match key_event.get_key_name().as_str() {
                "ArrowUp" => {
                    let _ = self.motor_pub.send((0, speed), ctx);  // Motor 0 forward
                }
                "ArrowDown" => {
                    let _ = self.motor_pub.send((0, -speed), ctx);  // Motor 0 backward
                }
                "Space" => {
                    let _ = self.motor_pub.send((0, 0.0), ctx);  // Stop
                }
                _ => {}
            }
        }
    }
}
```

### Integration with State Machine

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::KeyboardInput;
use horus_core::{Node, Hub, NodeInfo};

#[derive(Debug, Clone, Copy, PartialEq)]
enum RobotState {
    Idle,
    Manual,
    Autonomous,
    Emergency,
}

struct StateMachineNode {
    keyboard_sub: Hub<KeyboardInput>,
    state: RobotState,
}

impl Node for StateMachineNode {
    fn name(&self) -> &'static str {
        "StateMachineNode"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        while let Ok(key_event) = self.keyboard_sub.try_recv() {
            if !key_event.pressed {
                continue;
            }

            match key_event.get_key_name().as_str() {
                "F1" => {
                    self.state = RobotState::Idle;
                    eprintln!("State: IDLE");
                }
                "F2" => {
                    self.state = RobotState::Manual;
                    eprintln!("State: MANUAL CONTROL");
                }
                "F3" => {
                    self.state = RobotState::Autonomous;
                    eprintln!("State: AUTONOMOUS");
                }
                "Escape" | "F12" => {
                    self.state = RobotState::Emergency;
                    eprintln!("State: EMERGENCY STOP");
                }
                _ => {}
            }
        }
    }
}
```

### Integration with Differential Drive Robot

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::KeyboardInput;
use horus_core::{Node, Hub, NodeInfo};

struct DiffDriveKeyboardNode {
    keyboard_sub: Hub<KeyboardInput>,
    left_motor_pub: Hub<f32>,
    right_motor_pub: Hub<f32>,
    max_speed: f32,
    turn_ratio: f32,
}

impl DiffDriveKeyboardNode {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            keyboard_sub: Hub::<KeyboardInput>::subscriber("keyboard_input")?,
            left_motor_pub: Hub::new("left_motor_speed")?,
            right_motor_pub: Hub::new("right_motor_speed")?,
            max_speed: 100.0,
            turn_ratio: 0.5,
        })
    }
}

impl Node for DiffDriveKeyboardNode {
    fn name(&self) -> &'static str {
        "DiffDriveKeyboardNode"
    }

    fn tick(&mut self, ctx: Option<&mut NodeInfo>) {
        while let Ok(key_event) = self.keyboard_sub.try_recv() {
            if !key_event.pressed {
                continue;
            }

            let (left_speed, right_speed) = match key_event.get_key_name().as_str() {
                "ArrowUp" | "W" => {
                    // Forward
                    (self.max_speed, self.max_speed)
                }
                "ArrowDown" | "S" => {
                    // Backward
                    (-self.max_speed, -self.max_speed)
                }
                "ArrowLeft" | "A" => {
                    // Turn left (differential)
                    (-self.max_speed * self.turn_ratio, self.max_speed * self.turn_ratio)
                }
                "ArrowRight" | "D" => {
                    // Turn right (differential)
                    (self.max_speed * self.turn_ratio, -self.max_speed * self.turn_ratio)
                }
                "Space" => {
                    // Stop
                    (0.0, 0.0)
                }
                _ => continue,
            };

            let _ = self.left_motor_pub.send(left_speed, ctx);
            let _ = self.right_motor_pub.send(right_speed, ctx);
        }
    }
}
```

### Integration with Logging and Debugging

```rust
use horus_library::nodes::KeyboardInputNode;
use horus_library::KeyboardInput;
use horus_core::{Node, Hub, NodeInfo};

struct DebugNode {
    keyboard_sub: Hub<KeyboardInput>,
    verbose: bool,
    log_level: u8,
}

impl Node for DebugNode {
    fn name(&self) -> &'static str {
        "DebugNode"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        while let Ok(key_event) = self.keyboard_sub.try_recv() {
            // Log all key events in verbose mode
            if self.verbose {
                eprintln!(
                    "[DEBUG] Key: {}, Code: {}, Modifiers: {:?}, Pressed: {}, Time: {}",
                    key_event.get_key_name(),
                    key_event.code,
                    key_event.get_modifiers(),
                    key_event.pressed,
                    key_event.timestamp
                );
            }

            if !key_event.pressed {
                continue;
            }

            // Toggle verbose mode with 'V'
            if key_event.get_key_name() == "V" && key_event.is_ctrl() {
                self.verbose = !self.verbose;
                eprintln!("Verbose mode: {}", self.verbose);
            }

            // Adjust log level with F9-F12
            match key_event.get_key_name().as_str() {
                "F9" => self.log_level = 0,   // Error
                "F10" => self.log_level = 1,  // Warning
                "F11" => self.log_level = 2,  // Info
                "F12" => self.log_level = 3,  // Debug
                _ => {}
            }
        }
    }
}
```

## Performance Considerations

### Update Rate

The keyboard input node processes events on every tick:

- **Terminal Mode**: Non-blocking polling with 0ms timeout
- **Demo Mode**: Event generation every 2000ms
- **Typical tick rate**: 10-100 Hz (10-100ms)
- **Event processing**: Immediate, no buffering

### CPU Usage

- Minimal CPU usage in terminal mode (non-blocking poll)
- Event-driven processing (only active when keys pressed)
- No background threads
- HashMap lookup for key mapping: O(1) average case

### Memory Usage

- Fixed memory footprint: ~500 bytes
- KeyboardInput message size: 72 bytes (fixed)
- HashMap capacity scales with number of custom mappings
- No dynamic allocation during event processing

### Latency

- Input-to-publish latency: < 1ms (terminal mode)
- Network/IPC latency: Depends on Hub configuration
- End-to-end latency: Typically 5-20ms from keypress to action

## Best Practices

### 1. Configure Before Runtime

```rust
// Good - configure before adding to runtime
let mut keyboard = KeyboardInputNode::new()?;
keyboard.set_key_mapping(...);
runtime.add_node(keyboard);

// Bad - cannot configure after adding to runtime
runtime.add_node(KeyboardInputNode::new()?);
// keyboard.set_key_mapping(...);  // Cannot access anymore
```

### 2. Use Try_recv for Non-Blocking

```rust
// Good - non-blocking, processes all available events
while let Ok(key_event) = subscriber.try_recv() {
    // Process event
}

// Bad - blocking, may miss events
let key_event = subscriber.recv()?;  // Blocks until event
```

### 3. Check Key Press State

```rust
// Good - only handle key press events
if key_event.pressed {
    match key_event.get_key_name().as_str() {
        "Space" => handle_action(),
        _ => {}
    }
}

// Bad - handles both press and release
match key_event.get_key_name().as_str() {
    "Space" => handle_action(),  // Called twice!
    _ => {}
}
```

### 4. Handle Cleanup Gracefully

```rust
// Good - let Drop handle cleanup
{
    let keyboard = KeyboardInputNode::new()?;
    runtime.add_node(keyboard);
    runtime.run()?;
}  // Raw mode automatically disabled

// Good - use ESC or Ctrl+C for exit
if key_event.code == keycodes::KEY_ESCAPE {
    // Node will clean up automatically
    return;
}
```

### 5. Use Keycodes Module

```rust
use horus_library::nodes::keyboard_input_node::keycodes;

// Good - use named constants
if key_event.code == keycodes::KEY_ESCAPE {
    exit();
}

// Bad - magic numbers
if key_event.code == 27 {  // What is 27?
    exit();
}
```

## Related Nodes

- **DifferentialDriveNode**: Uses keyboard input for robot teleoperation
- **MotorControllerNode**: Receives commands from keyboard input
- **GamepadInputNode**: Alternative input method for control
- **JoystickInputNode**: Analog control alternative

## See Also

- [Crossterm Documentation](https://docs.rs/crossterm/)
- [Terminal Raw Mode](https://en.wikipedia.org/wiki/Terminal_mode)
- [Keyboard Scancode Reference](https://www.win.tue.nl/~aeb/linux/kbd/scancodes.html)
