# Emergency Stop Node

Safety-critical emergency stop handler with hardware button support and watchdog timer functionality.

## Overview

The Emergency Stop Node provides industrial-grade safety control for robotic systems. It monitors hardware emergency stop buttons (via GPIO), software triggers, and system conditions to immediately halt dangerous operations. The node maintains fail-safe behavior by publishing emergency stop signals and safety status messages that other nodes can subscribe to for coordinated shutdown.

This node is designed for safety-critical applications where immediate system shutdown is required. It supports both hardware button inputs (Raspberry Pi GPIO) and software-triggered emergency stops, with configurable auto-reset behavior and timeout management.

## Topics

### Subscribers

This node does not subscribe to any topics. Emergency stops are triggered through:
- Hardware GPIO pins (physical emergency stop button)
- Software API calls (`trigger_stop()`)

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `emergency_stop` | `EmergencyStop` | Emergency stop state and reason |
| `emergency_stop_safety` | `SafetyStatus` | Detailed safety system status |

**Note**: The safety topic is automatically named by appending `_safety` to the main topic name. For example, if the main topic is `robot_estop`, the safety topic will be `robot_estop_safety`.

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `gpio_pin` | `Option<u8>` | `None` | GPIO pin number for hardware emergency stop button (Raspberry Pi only) |
| `auto_reset` | `bool` | `false` | Enable automatic reset after timeout (use with caution in safety applications) |
| `stop_timeout_ms` | `u64` | `5000` | Timeout in milliseconds for automatic reset (5 seconds) |

**Safety Note**: Auto-reset functionality should only be enabled in non-critical applications or testing environments. Industrial safety standards typically require manual reset after emergency stop conditions.

## Message Types

### EmergencyStop

Emergency stop signal with engagement state and reason:

```rust
pub struct EmergencyStop {
    /// Emergency stop is active
    pub engaged: bool,
    /// Reason for emergency stop (up to 63 bytes, null-terminated)
    pub reason: [u8; 64],
    /// Source that triggered the stop (up to 31 bytes, null-terminated)
    pub source: [u8; 32],
    /// Auto-reset allowed after clearing
    pub auto_reset: bool,
    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}
```

**Helper methods**:
```rust
// Create an engaged emergency stop with reason
let estop = EmergencyStop::engage("Hardware button pressed");

// Create a release signal (disengaged)
let release = EmergencyStop::release();
```

### SafetyStatus

Comprehensive safety system status:

```rust
pub struct SafetyStatus {
    /// Safety system is active
    pub enabled: bool,
    /// Emergency stop is engaged
    pub estop_engaged: bool,
    /// Watchdog timer is OK
    pub watchdog_ok: bool,
    /// All limits are within bounds
    pub limits_ok: bool,
    /// Communication is healthy
    pub comms_ok: bool,
    /// Safety mode (0=normal, 1=reduced, 2=safe_stop)
    pub mode: u8,
    /// Fault code if any (1 = emergency stop fault)
    pub fault_code: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}
```

**Safety modes**:
- `MODE_NORMAL (0)`: System operating normally
- `MODE_REDUCED (1)`: Operating with reduced capabilities
- `MODE_SAFE_STOP (2)`: Emergency stop engaged, system halted

## Public API

### Construction

```rust
use horus_library::nodes::EmergencyStopNode;

// Create with default topic "emergency_stop"
let mut estop = EmergencyStopNode::new()?;

// Create with custom topic name
let mut estop = EmergencyStopNode::new_with_topic("robot_estop")?;
// This will publish to "robot_estop" and "robot_estop_safety"
```

### Configuration Methods

```rust
// Configure hardware emergency stop button (GPIO pin 17 on Raspberry Pi)
estop.set_gpio_pin(17);

// Enable automatic reset after timeout (use with caution!)
estop.set_auto_reset(true);

// Set auto-reset timeout to 10 seconds
estop.set_reset_timeout(10000);

// Disable auto-reset (recommended for safety-critical applications)
estop.set_auto_reset(false);
```

### Triggering and Resetting

```rust
// Trigger emergency stop with a reason
estop.trigger_stop("Collision detected");

// Check if emergency stop is currently active
if estop.is_emergency_stopped() {
    eprintln!("System is in emergency stop state");
}

// Get the reason for the current emergency stop
let reason = estop.get_stop_reason();
eprintln!("Stop reason: {}", reason);

// Manually reset emergency stop (only if safe to do so)
estop.reset();
```

### Thread-Safe Status Checking

The emergency stop state is stored in an `Arc<AtomicBool>`, making it safe to check from multiple threads:

```rust
// The is_stopped flag can be safely read from any thread
let is_stopped = estop.is_emergency_stopped();
```

## Usage Examples

### Basic Hardware Emergency Stop

```rust
use horus_library::nodes::EmergencyStopNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create emergency stop node with hardware button
    let mut estop = EmergencyStopNode::new()?;

    // Configure GPIO pin 17 for emergency stop button
    // Button should be wired active-low (pulled high, grounds when pressed)
    estop.set_gpio_pin(17);

    // No auto-reset - requires manual intervention
    estop.set_auto_reset(false);

    runtime.add_node(estop);
    runtime.run()?;

    Ok(())
}
```

### Software-Triggered Emergency Stop with Monitoring

```rust
use horus_library::nodes::EmergencyStopNode;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create emergency stop node
    let mut estop = EmergencyStopNode::new_with_topic("system_estop")?;
    estop.set_auto_reset(false);

    // Subscribe to emergency stop messages for monitoring
    let estop_sub = Hub::<EmergencyStop>::subscribe("system_estop")?;

    runtime.add_node(estop);

    // Spawn monitoring thread
    std::thread::spawn(move || {
        loop {
            if let Ok(msg) = estop_sub.receive(1000) {
                if msg.engaged {
                    let reason = std::str::from_utf8(&msg.reason)
                        .unwrap_or("Unknown")
                        .trim_end_matches('\0');
                    eeprintln!("EMERGENCY STOP: {}", reason);
                    // Trigger system shutdown, log event, etc.
                } else {
                    eprintln!("Emergency stop released");
                }
            }
        }
    });

    runtime.run()?;
    Ok(())
}
```

### Multi-Layer Safety System with Auto-Reset for Testing

```rust
use horus_library::nodes::EmergencyStopNode;
use horus_library::{EmergencyStop, SafetyStatus};
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create emergency stop node with auto-reset for testing
    let mut estop = EmergencyStopNode::new()?;
    estop.set_gpio_pin(17);
    estop.set_auto_reset(true);  // ONLY for testing/simulation
    estop.set_reset_timeout(3000);  // 3 second timeout

    // Subscribe to safety status for detailed monitoring
    let safety_sub = Hub::<SafetyStatus>::subscribe("emergency_stop_safety")?;

    runtime.add_node(estop);

    // Monitor safety status in separate thread
    std::thread::spawn(move || {
        loop {
            if let Ok(status) = safety_sub.receive(100) {
                match status.mode {
                    SafetyStatus::MODE_NORMAL => {
                        eprintln!("Safety: NORMAL");
                    }
                    SafetyStatus::MODE_REDUCED => {
                        eprintln!("Safety: REDUCED CAPABILITY");
                    }
                    SafetyStatus::MODE_SAFE_STOP => {
                        eprintln!("Safety: EMERGENCY STOP (fault code: {})", status.fault_code);
                        // Halt all motion, disable actuators, etc.
                    }
                    _ => {
                        eprintln!("Safety: UNKNOWN MODE");
                    }
                }
            }
        }
    });

    runtime.run()?;
    Ok(())
}
```

### Integration with Motor Controller

```rust
use horus_library::nodes::EmergencyStopNode;
use horus_library::EmergencyStop;
use horus_core::{Node, Runtime, Hub};

// Motor controller node that respects emergency stops
struct SafeMotorController {
    estop_sub: Hub<EmergencyStop>,
    is_stopped: bool,
}

impl SafeMotorController {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            estop_sub: Hub::subscribe("emergency_stop")?,
            is_stopped: false,
        })
    }
}

impl Node for SafeMotorController {
    fn name(&self) -> &'static str {
        "SafeMotorController"
    }

    fn tick(&mut self, _ctx: Option<&mut horus_core::NodeInfo>) {
        // Check for emergency stop messages
        if let Ok(estop) = self.estop_sub.receive(0) {
            self.is_stopped = estop.engaged;

            if estop.engaged {
                // Immediately halt all motors
                eprintln!("MOTOR EMERGENCY STOP");
                // Set all motor commands to zero
                // Engage mechanical brakes if available
            } else {
                eprintln!("Motors released from emergency stop");
            }
        }

        // Only allow motor commands if not stopped
        if !self.is_stopped {
            // Normal motor control logic here
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create emergency stop node
    let mut estop = EmergencyStopNode::new()?;
    estop.set_gpio_pin(17);

    // Create motor controller that subscribes to emergency stop
    let motor_controller = SafeMotorController::new()?;

    runtime.add_node(estop);
    runtime.add_node(motor_controller);
    runtime.run()?;

    Ok(())
}
```

### Watchdog Timer Pattern

```rust
use horus_library::nodes::EmergencyStopNode;
use horus_library::SafetyStatus;
use horus_core::{Node, Runtime, Hub};
use std::time::{Duration, Instant};

// Watchdog node that triggers emergency stop on timeout
struct WatchdogNode {
    estop_trigger: Arc<Mutex<EmergencyStopNode>>,
    heartbeat_sub: Hub<Heartbeat>,
    last_heartbeat: Instant,
    timeout: Duration,
}

impl WatchdogNode {
    fn new(
        estop: Arc<Mutex<EmergencyStopNode>>,
        timeout_ms: u64
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            estop_trigger: estop,
            heartbeat_sub: Hub::subscribe("system_heartbeat")?,
            last_heartbeat: Instant::now(),
            timeout: Duration::from_millis(timeout_ms),
        })
    }
}

impl Node for WatchdogNode {
    fn name(&self) -> &'static str {
        "WatchdogNode"
    }

    fn tick(&mut self, _ctx: Option<&mut horus_core::NodeInfo>) {
        // Check for heartbeat messages
        if let Ok(_heartbeat) = self.heartbeat_sub.receive(0) {
            self.last_heartbeat = Instant::now();
        }

        // Check for timeout
        if self.last_heartbeat.elapsed() > self.timeout {
            if let Ok(mut estop) = self.estop_trigger.lock() {
                if !estop.is_emergency_stopped() {
                    estop.trigger_stop("Watchdog timeout - system unresponsive");
                }
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create emergency stop node
    let estop = Arc::new(Mutex::new(EmergencyStopNode::new()?));

    // Create watchdog that monitors heartbeat
    let watchdog = WatchdogNode::new(estop.clone(), 1000)?; // 1 second timeout

    runtime.add_node(watchdog);
    runtime.run()?;

    Ok(())
}
```

## Safety-Critical Considerations

### Fail-Safe Design Principles

The Emergency Stop Node follows critical safety principles:

1. **Fail-Safe Default**: System initializes in a safe state (not stopped, but ready to stop)
2. **Active Monitoring**: Continuously monitors GPIO input every tick cycle
3. **Immediate Response**: Emergency stop triggers immediately without delay
4. **Persistent State**: Stop state is maintained until explicitly reset
5. **Redundant Communication**: Publishes both `EmergencyStop` and `SafetyStatus` messages

### Hardware Button Wiring

For maximum safety, wire emergency stop buttons using this configuration:

```
Raspberry Pi GPIO Pin (configured with internal pull-up)
    |
    +---- Emergency Stop Button ----+ GND

Normal state: HIGH (button not pressed)
Emergency state: LOW (button pressed, grounds the pin)
```

This configuration ensures that:
- Wire breakage triggers emergency stop (fail-safe)
- Button press triggers emergency stop (active-low)
- Internal pull-up resistor provides default HIGH state

### GPIO Pin Selection

Recommended GPIO pins for Raspberry Pi emergency stop:
- **GPIO 17** (Physical pin 11): Commonly used, no special functions
- **GPIO 27** (Physical pin 13): Alternative, no special functions
- **GPIO 22** (Physical pin 15): Another safe choice

**Avoid these pins**:
- GPIO 2, 3 (I2C - may interfere with other devices)
- GPIO 14, 15 (UART - used for serial communication)
- GPIO 9, 10, 11 (SPI - may interfere with SPI devices)

### Reset Procedures

#### Manual Reset (Recommended for Safety-Critical Systems)

```rust
// Only reset when it is VERIFIED SAFE to do so
if operator_confirms_safe() && all_hazards_cleared() {
    estop.reset();
    eprintln!("System reset - operations may resume");
}
```

#### Automatic Reset (Use with Extreme Caution)

Auto-reset should ONLY be used in:
- Testing and simulation environments
- Non-safety-critical applications
- Supervised operation with operator override capability

```rust
// Configure auto-reset with appropriate timeout
estop.set_auto_reset(true);
estop.set_reset_timeout(5000); // 5 seconds

// ALWAYS log auto-reset events
if estop.is_emergency_stopped() {
    log::warn!("Auto-reset enabled - system will reset in {} ms", 5000);
}
```

### Integration with Other Safety Systems

The Emergency Stop Node should be integrated with:

1. **Motor Controllers**: All motor nodes must subscribe to emergency stop
2. **Actuators**: Pneumatic, hydraulic, and servo systems must halt on emergency stop
3. **High-Power Devices**: Heaters, lasers, and other high-power devices must disable
4. **Safety PLCs**: Bridge to industrial safety PLCs for redundant safety
5. **Logging Systems**: Record all emergency stop events with timestamps and reasons

## Troubleshooting

### Issue: GPIO button not triggering emergency stop

**Cause**: Incorrect GPIO pin configuration, wiring issue, or missing Raspberry Pi feature flag

**Solution**:
```rust
// Verify GPIO pin number matches physical wiring
estop.set_gpio_pin(17); // Double-check this matches your button

// Ensure raspberry-pi feature is enabled in Cargo.toml:
// horus_library = { version = "0.1", features = ["raspberry-pi"] }

// Test GPIO pin manually:
// gpio readall  # Check pin status on Raspberry Pi
```

**Hardware checks**:
- Verify button is wired to correct pin and ground
- Test button continuity with multimeter
- Check for loose connections or corroded contacts
- Ensure GPIO pin is not used by another process

### Issue: Emergency stop not resetting

**Cause**: Auto-reset disabled or timeout not reached

**Solution**:
```rust
// Check if auto-reset is enabled
if !estop.is_emergency_stopped() {
    eprintln!("Not in emergency stop state");
} else {
    // Manually reset
    estop.reset();

    // Or enable auto-reset
    estop.set_auto_reset(true);
    estop.set_reset_timeout(5000);
}
```

### Issue: Spurious emergency stop triggers

**Cause**: Electrical noise on GPIO pin, switch bouncing, or loose connections

**Solution**:
```rust
// Hardware solutions:
// - Add hardware debounce capacitor (0.1uF across button)
// - Use shielded cable for button wiring
// - Keep emergency stop wiring away from high-power lines
// - Ensure good ground connection

// Software solution (requires code modification):
// - Implement debounce logic (wait for stable LOW state)
// - Require button press for minimum duration (e.g., 50ms)
```

### Issue: Other nodes not responding to emergency stop

**Cause**: Nodes not subscribing to emergency stop topic, or incorrect topic name

**Solution**:
```rust
// Ensure all safety-critical nodes subscribe to the correct topic
let estop_sub = Hub::<EmergencyStop>::subscribe("emergency_stop")?;

// In motor controller or other critical nodes:
impl Node for YourNode {
    fn tick(&mut self, _ctx: Option<&mut horus_core::NodeInfo>) {
        // Check emergency stop FIRST, before any other logic
        if let Ok(estop) = self.estop_sub.receive(0) {
            if estop.engaged {
                self.shutdown_all_actuators();
                return; // Exit immediately
            }
        }

        // Normal operation only if not in emergency stop
        // ...
    }
}
```

### Issue: Emergency stop state lost on restart

**Cause**: Emergency stop state is not persisted across process restarts

**Solution**:
```rust
// If persistence is required, implement state saving:
use std::fs;

// Before shutdown, save state
let state = serde_json::json!({
    "emergency_stop": estop.is_emergency_stopped(),
    "reason": estop.get_stop_reason(),
});
fs::write("/var/run/estop_state.json", state.to_string())?;

// On startup, restore state
if let Ok(state) = fs::read_to_string("/var/run/estop_state.json") {
    let state: serde_json::Value = serde_json::from_str(&state)?;
    if state["emergency_stop"].as_bool().unwrap_or(false) {
        estop.trigger_stop(&state["reason"].as_str().unwrap_or("Previous stop"));
    }
}
```

### Issue: Cannot compile with GPIO support

**Cause**: Missing `raspberry-pi` feature flag or building for wrong architecture

**Solution**:
```toml
# In Cargo.toml:
[dependencies]
horus_library = { version = "0.1", features = ["raspberry-pi"] }

# Build for Raspberry Pi (if cross-compiling):
# cargo build --target armv7-unknown-linux-gnueabihf --features raspberry-pi
```

For simulation/testing without GPIO:
```rust
// GPIO support is automatically disabled without raspberry-pi feature
// The node will still work, but check_gpio_pin() will always return false
let mut estop = EmergencyStopNode::new()?;
// Don't call set_gpio_pin() in simulation mode
// Use trigger_stop() for software-triggered stops instead
```

## Performance Considerations

### Update Rate

The Emergency Stop Node should run at high frequency for safety:

- **Safety-critical systems**: 100-500 Hz (2-10ms tick rate)
- **Industrial robotics**: 50-100 Hz (10-20ms tick rate)
- **General automation**: 20-50 Hz (20-50ms tick rate)

**Response time** = Tick period + message propagation time (typically < 1ms)

### CPU Usage

Minimal CPU usage:
- Without GPIO: ~0.1% CPU (only checks timers and publishes status)
- With GPIO: ~0.5% CPU (adds GPIO pin read via rppal library)

### Memory Usage

Small fixed memory footprint:
- Node structure: ~200 bytes
- Publishers: ~100 bytes each
- Total: ~400 bytes per instance

### Thread Safety

The `is_stopped` flag uses `Arc<AtomicBool>` for lock-free, thread-safe access:
```rust
// Safe to call from any thread
let stopped = estop.is_emergency_stopped();
```

## Integration with Other Nodes

### Motor Control Nodes

All motor control nodes should subscribe to emergency stop:

```rust
// In any motor controller
let estop_sub = Hub::<EmergencyStop>::subscribe("emergency_stop")?;

// First action in tick():
if let Ok(msg) = self.estop_sub.receive(0) {
    if msg.engaged {
        self.halt_immediately();
        return;
    }
}
```

### Differential Drive Node

```rust
use horus_library::nodes::DifferentialDriveNode;

// Differential drive automatically respects emergency stops
// if it subscribes to the same topic
let mut drive = DifferentialDriveNode::new()?;
// Configure drive to monitor emergency stop
```

### Safety Monitor Node

```rust
// Create a comprehensive safety monitoring system
let safety_sub = Hub::<SafetyStatus>::subscribe("emergency_stop_safety")?;

// Monitor all safety parameters
if status.estop_engaged {
    // Emergency stop active
}
if !status.watchdog_ok {
    // Watchdog timer expired
}
if !status.limits_ok {
    // Position/velocity limits exceeded
}
if !status.comms_ok {
    // Communication failure detected
}
```

### PID Controller Integration

PID controllers should zero their output during emergency stop:

```rust
// In PID controller node
if let Ok(estop) = self.estop_sub.receive(0) {
    if estop.engaged {
        self.reset(); // Clear integral and derivative terms
        self.output = 0.0; // Zero output
        return;
    }
}
```

## Related Nodes

- **MotorControllerNode**: Must subscribe to emergency stop for immediate motor shutdown
- **DifferentialDriveNode**: Integrates emergency stop for mobile robot safety
- **ServoControllerNode**: Should halt all servo motion on emergency stop
- **SafetyMonitorNode**: Monitors multiple safety conditions and can trigger emergency stop
- **WatchdogNode**: Monitors system heartbeat and triggers emergency stop on timeout

## Compliance and Standards

This node is designed to support compliance with:

- **ISO 13849-1**: Safety of machinery - Safety-related parts of control systems
- **IEC 61508**: Functional safety of electrical/electronic/programmable electronic safety-related systems
- **ISO 10218**: Robots and robotic devices - Safety requirements for industrial robots

**Note**: Full compliance requires:
- Redundant emergency stop systems (hardware + software)
- Safety-rated relays for high-power circuits
- Regular testing and validation
- Documentation of safety analysis (FMEA, risk assessment)
- Professional safety engineering review

## Best Practices

1. **Always disable auto-reset in production**: Manual reset ensures operator verification
2. **Use hardware buttons for critical safety**: Don't rely solely on software triggers
3. **Subscribe all motor/actuator nodes**: Ensure coordinated emergency shutdown
4. **Log all emergency stop events**: Maintain audit trail for safety analysis
5. **Test emergency stop regularly**: Verify functionality as part of maintenance procedures
6. **Implement redundant safety**: Use multiple safety layers (hardware + software)
7. **Clear indication to operators**: Visual/audible alarms when emergency stop is active
8. **Document reset procedures**: Ensure operators know when it's safe to reset

## See Also

- [Functional Safety - ISO 13849](https://www.iso.org/standard/69883.html)
- [Emergency Stop Devices - ISO 13850](https://www.iso.org/standard/59970.html)
- [Raspberry Pi GPIO Documentation](https://www.raspberrypi.org/documentation/hardware/gpio/)
- [Safety PLC Integration Guide](../integration/safety_plc.md)
