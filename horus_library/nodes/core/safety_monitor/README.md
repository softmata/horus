# Safety Monitor Node

Comprehensive safety monitoring system for critical parameters and protective actions.

## Overview

The Safety Monitor Node implements a multi-layered safety monitoring system that watches critical parameters including system resources (CPU, memory, disk), communication health, battery levels, emergency stops, and custom safety checks. It aggregates these inputs to determine the overall system safety level and publishes appropriate safety status messages that can trigger protective actions throughout the system.

The node operates on a continuous monitoring cycle, evaluating all safety checks on each tick and escalating the safety level when thresholds are exceeded. It supports both built-in system monitoring (via the `sysinfo` feature) and custom application-specific safety checks.

## Architecture

**This node is a thin wrapper** around the pure algorithm in `horus_library/algorithms/`:

- **`algorithms::safety_layer::SafetyLayer`** - Multi-layered safety checking (velocity, obstacle distance, battery, temperature)

The node handles:
- Topic subscription/publishing (Hub I/O)
- System resource monitoring (CPU, memory, disk)
- Communication health checking
- Custom safety check management
- Safety status publishing
- Battery and temperature tracking

The algorithm handles:
- Velocity safety checks
- Obstacle distance validation
- Battery level monitoring
- Temperature threshold checking
- Comprehensive safety status (Safe/Warning/Critical)
- Pure safety logic computations

This separation enables safety layer algorithm reuse across different monitoring contexts.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `emergency_stop` | `EmergencyStop` | Emergency stop signals from hardware or software |
| `battery_state` | `BatteryState` | Battery voltage, current, and charge level |
| `resource_usage` | `ResourceUsage` | System resource utilization metrics |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `safety_status` | `SafetyStatus` | Aggregated safety system status |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cpu_threshold` | `f32` | `90.0` | Maximum CPU usage percentage (0-100%) |
| `memory_threshold` | `f32` | `85.0` | Maximum memory usage percentage (0-100%) |
| `disk_threshold` | `f32` | `95.0` | Maximum disk usage percentage (0-100%) |
| `temperature_threshold` | `f32` | `80.0` | Maximum system temperature (°C) |
| `battery_threshold` | `f32` | `15.0` | Minimum battery percentage (0-100%) |
| `communication_timeout_ms` | `u64` | `5000` | Communication timeout in milliseconds |

## Message Types

### EmergencyStop

Emergency stop control message:

```rust
pub struct EmergencyStop {
    /// Emergency stop is active
    pub engaged: bool,
    /// Reason for emergency stop
    pub reason: [u8; 64],
    /// Source that triggered the stop
    pub source: [u8; 32],
    /// Auto-reset allowed after clearing
    pub auto_reset: bool,
    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}
```

### BatteryState

Battery status and health information:

```rust
pub struct BatteryState {
    /// Voltage in volts
    pub voltage: f32,
    /// Current in amperes (negative = discharging)
    pub current: f32,
    /// Charge in amp-hours
    pub charge: f32,
    /// Capacity in amp-hours
    pub capacity: f32,
    /// Percentage charge (0-100)
    pub percentage: f32,
    /// Power supply status (0=unknown, 1=charging, 2=discharging, 3=full)
    pub power_supply_status: u8,
    /// Temperature in celsius
    pub temperature: f32,
    /// Cell voltages if available
    pub cell_voltages: [f32; 16],
    /// Number of valid cell voltage readings
    pub cell_count: u8,
    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}
```

### ResourceUsage

System resource utilization metrics:

```rust
pub struct ResourceUsage {
    /// CPU usage percentage (0-100)
    pub cpu_percent: f32,
    /// Memory usage in bytes
    pub memory_bytes: u64,
    /// Memory usage percentage (0-100)
    pub memory_percent: f32,
    /// Disk usage in bytes
    pub disk_bytes: u64,
    /// Disk usage percentage (0-100)
    pub disk_percent: f32,
    /// Network bytes sent
    pub network_tx_bytes: u64,
    /// Network bytes received
    pub network_rx_bytes: u64,
    /// System temperature in celsius
    pub temperature: f32,
    /// Number of active threads
    pub thread_count: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}
```

### SafetyStatus

Aggregated safety system status output:

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
    /// Fault code if any
    pub fault_code: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}

// Safety modes
const MODE_NORMAL: u8 = 0;      // Normal operation
const MODE_REDUCED: u8 = 1;     // Reduced performance mode
const MODE_SAFE_STOP: u8 = 2;   // Safe stop required
```

### StatusLevel

Safety severity levels (internally used for escalation):

```rust
pub enum StatusLevel {
    /// Everything is OK
    Ok = 0,
    /// Warning condition
    Warn = 1,
    /// Error condition (recoverable)
    Error = 2,
    /// Fatal error (system should stop)
    Fatal = 3,
}
```

## Public API

### Construction

```rust
use horus_library::nodes::SafetyMonitorNode;

// Create with default topic "safety_status"
let mut monitor = SafetyMonitorNode::new()?;

// Create with custom output topic
let mut monitor = SafetyMonitorNode::new_with_topic("robot_safety")?;
```

### Configuration Methods

```rust
// Set resource thresholds
monitor.set_cpu_threshold(85.0);           // 85% CPU
monitor.set_memory_threshold(80.0);        // 80% memory
monitor.set_disk_threshold(90.0);          // 90% disk
monitor.set_temperature_threshold(75.0);   // 75°C

// Set battery threshold
monitor.set_battery_threshold(20.0);       // 20% minimum battery

// Set communication timeout
monitor.set_communication_timeout(3000);   // 3 seconds

// Add custom safety check
monitor.add_safety_check("lidar_check", 1000);  // 1 second timeout

// Update custom safety check
monitor.update_safety_check(
    "lidar_check",
    StatusLevel::Ok,
    "Lidar operating normally"
);
```

## Usage Examples

### Basic Safety Monitoring

```rust
use horus_library::nodes::SafetyMonitorNode;
use horus_core::{Node, Scheduler};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create safety monitor with default settings
    let mut monitor = SafetyMonitorNode::new()?;

    // Configure thresholds for your application
    monitor.set_cpu_threshold(90.0);
    monitor.set_memory_threshold(85.0);
    monitor.set_battery_threshold(15.0);
    monitor.set_communication_timeout(5000);

    scheduler.add(Box::new(monitor), 50, Some(true));
    scheduler.run()?;

    Ok(())
}
```

### Velocity Monitoring

```rust
use horus_library::nodes::SafetyMonitorNode;
use horus_library::messages::StatusLevel;
use horus_core::{Node, Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create safety monitor
    let mut monitor = SafetyMonitorNode::new()?;

    // Add custom velocity safety check
    monitor.add_safety_check("velocity_limit", 500);  // 500ms timeout

    scheduler.add(Box::new(monitor), 50, Some(true));

    // In your velocity monitoring code:
    let velocity_topic = Hub::<f32>::new("current_velocity")?;

    loop {
        if let Some(velocity) = velocity_topic.recv(None) {
            if velocity > 2.0 {  // Max 2.0 m/s
                monitor.update_safety_check(
                    "velocity_limit",
                    StatusLevel::Error,
                    "Velocity exceeded 2.0 m/s"
                );
            } else {
                monitor.update_safety_check(
                    "velocity_limit",
                    StatusLevel::Ok,
                    "Velocity within limits"
                );
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}
```

### Obstacle Detection Integration

```rust
use horus_library::nodes::SafetyMonitorNode;
use horus_library::messages::{StatusLevel, LaserScan};
use horus_core::{Node, Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create safety monitor
    let mut monitor = SafetyMonitorNode::new()?;

    // Add obstacle detection safety check
    monitor.add_safety_check("obstacle_detection", 1000);

    scheduler.add(Box::new(monitor), 50, Some(true));

    // Monitor laser scan for obstacles
    let laser_topic = Hub::<LaserScan>::new("scan")?;
    let min_safe_distance = 0.5;  // 0.5 meters

    loop {
        if let Some(scan) = laser_topic.recv(None) {
            let min_range = scan.ranges.iter()
                .filter(|r| r.is_finite() && **r > 0.0)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(&f32::MAX);

            if *min_range < min_safe_distance {
                monitor.update_safety_check(
                    "obstacle_detection",
                    StatusLevel::Fatal,
                    &format!("Obstacle at {:.2}m", min_range)
                );
            } else if *min_range < min_safe_distance * 1.5 {
                monitor.update_safety_check(
                    "obstacle_detection",
                    StatusLevel::Warn,
                    "Obstacle approaching"
                );
            } else {
                monitor.update_safety_check(
                    "obstacle_detection",
                    StatusLevel::Ok,
                    "No obstacles detected"
                );
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}
```

### Battery Monitoring with Multi-Level Warnings

```rust
use horus_library::nodes::SafetyMonitorNode;
use horus_library::messages::{BatteryState, SafetyStatus};
use horus_core::{Node, Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create safety monitor with battery focus
    let mut monitor = SafetyMonitorNode::new()?;

    // Set aggressive battery threshold for critical operations
    monitor.set_battery_threshold(25.0);  // 25% critical level

    // Add custom battery health check
    monitor.add_safety_check("battery_health", 2000);

    scheduler.add(Box::new(monitor), 50, Some(true));

    // Monitor battery state
    let battery_topic = Hub::<BatteryState>::new("battery_state")?;
    let safety_topic = Hub::<SafetyStatus>::new("safety_status")?;

    loop {
        if let Some(battery) = battery_topic.recv(None) {
            // Check battery temperature
            if battery.temperature > 50.0 {
                monitor.update_safety_check(
                    "battery_health",
                    StatusLevel::Error,
                    "Battery temperature high"
                );
            }
            // Check for critical voltage
            else if battery.voltage < 10.5 {
                monitor.update_safety_check(
                    "battery_health",
                    StatusLevel::Fatal,
                    "Battery voltage critical"
                );
            }
            // Check percentage thresholds
            else if battery.percentage < 10.0 {
                monitor.update_safety_check(
                    "battery_health",
                    StatusLevel::Error,
                    "Battery critically low"
                );
            } else if battery.percentage < 25.0 {
                monitor.update_safety_check(
                    "battery_health",
                    StatusLevel::Warn,
                    "Battery low"
                );
            } else {
                monitor.update_safety_check(
                    "battery_health",
                    StatusLevel::Ok,
                    "Battery healthy"
                );
            }
        }

        // Check safety status and take action
        if let Some(status) = safety_topic.recv(None) {
            if status.mode == SafetyStatus::MODE_SAFE_STOP {
                eprintln!("SAFETY STOP TRIGGERED: fault code {}", status.fault_code);
                // Trigger protective actions here
            } else if status.mode == SafetyStatus::MODE_REDUCED {
                eprintln!("Reduced performance mode active");
                // Reduce operation speed/power
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    Ok(())
}
```

### Complete Multi-Sensor Safety System

```rust
use horus_library::nodes::SafetyMonitorNode;
use horus_library::messages::{StatusLevel, SafetyStatus};
use horus_core::{Node, Scheduler};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create comprehensive safety monitor
    let mut monitor = SafetyMonitorNode::new()?;

    // Configure system resource thresholds
    monitor.set_cpu_threshold(85.0);
    monitor.set_memory_threshold(80.0);
    monitor.set_temperature_threshold(75.0);

    // Configure battery safety
    monitor.set_battery_threshold(20.0);

    // Configure communication health
    monitor.set_communication_timeout(3000);

    // Add application-specific safety checks
    monitor.add_safety_check("motor_feedback", 500);
    monitor.add_safety_check("sensor_health", 1000);
    monitor.add_safety_check("navigation_state", 2000);
    monitor.add_safety_check("user_input", 5000);

    scheduler.add(Box::new(monitor), 50, Some(true));
    scheduler.run()?;

    Ok(())
}
```

## Safety Checks Explanation

### Built-in Safety Checks

#### System Resource Monitoring

The safety monitor automatically checks system resources when the `sysinfo` feature is enabled:

1. **CPU Usage**: Compares global CPU usage against `cpu_threshold`
   - Above threshold: `StatusLevel::Fatal`
   - Above 80% of threshold: `StatusLevel::Error`
   - Otherwise: `StatusLevel::Ok`

2. **Memory Usage**: Monitors RAM utilization percentage
   - Above threshold: `StatusLevel::Fatal`
   - Above 80% of threshold: `StatusLevel::Error`
   - Otherwise: `StatusLevel::Ok`

3. **Temperature**: Checks system temperature (if available)
   - Above threshold: Contributes to safety level degradation

#### Communication Health

Monitors the freshness of incoming messages:

1. **Emergency Stop**: Checks if emergency stop messages are being received within `communication_timeout_ms`
   - No recent message: `StatusLevel::Error`
   - Otherwise: `StatusLevel::Ok`

2. **Battery State**: Verifies battery telemetry is current
   - No recent message: `StatusLevel::Warn`
   - Otherwise: `StatusLevel::Ok`

#### Battery Level

Directly monitors battery percentage:

- Below `battery_threshold`: `StatusLevel::Error`
- Otherwise: `StatusLevel::Ok`

### Custom Safety Checks

Custom safety checks allow you to integrate application-specific monitoring:

```rust
// Add a check with timeout
monitor.add_safety_check("custom_check", timeout_ms);

// Update the check status
monitor.update_safety_check(
    "custom_check",
    StatusLevel::Warn,  // or Ok, Error, Fatal
    "Status description"
);
```

Each custom check:
- Has a timeout period (in milliseconds)
- Must be periodically updated via `update_safety_check()`
- Automatically escalates to `StatusLevel::Error` if not updated within timeout
- Contributes to the overall safety level calculation

### Safety Level Aggregation

The node determines the overall safety level by taking the **worst** (highest severity) status from all checks:

```
overall_safety_level = max(
    system_resource_status,
    communication_health_status,
    battery_level_status,
    custom_checks_status
)
```

This ensures that any single critical failure triggers appropriate protective actions.

## Warning/Alarm Thresholds

### Threshold Hierarchy

The safety monitor implements a multi-level threshold system:

| Level | Percentage of Threshold | Status | Safety Mode | Action |
|-------|------------------------|--------|-------------|---------|
| Normal | < 80% | `Ok` | `MODE_NORMAL` | Normal operation |
| Warning | 80% - 100% | `Warn` or `Error` | `MODE_REDUCED` | Reduced performance |
| Critical | > 100% | `Error` or `Fatal` | `MODE_SAFE_STOP` | Stop all motion |

### Resource-Specific Thresholds

#### CPU Usage
- **Warning** (80% of threshold): System approaching CPU limits
- **Critical** (100% of threshold): System overloaded, safety compromised
- **Default**: 90% (warning at 72%, critical at 90%)

#### Memory Usage
- **Warning** (80% of threshold): Memory pressure building
- **Critical** (100% of threshold): Risk of out-of-memory crash
- **Default**: 85% (warning at 68%, critical at 85%)

#### Disk Usage
- **Warning** (80% of threshold): Disk space running low
- **Critical** (100% of threshold): Risk of logging/data loss
- **Default**: 95% (warning at 76%, critical at 95%)

#### Temperature
- **Critical** (100% of threshold): Thermal throttling or damage risk
- **Default**: 80°C

#### Battery Level
- **Critical** (below threshold): Insufficient power for safe operation
- **Default**: 15%

### Fault Codes

The safety monitor publishes fault codes in the `SafetyStatus` message:

| Fault Code | Meaning | Safety Level |
|------------|---------|--------------|
| `0` | No fault | `Ok` |
| `1` | Warning condition | `Warn` |
| `2` | Error condition | `Error` |
| `999` | Critical/Fatal condition | `Fatal` |

Custom fault codes can be implemented by extending the safety logic.

## Troubleshooting

### Issue: Constant warning about communication timeout

**Cause**: Emergency stop or battery messages not being published

**Solution**:
```rust
// Ensure emergency stop node is running
let estop_node = EmergencyStopNode::new()?;
scheduler.add(Box::new(estop_node), 50, Some(true));

// Or publish periodic heartbeats
let estop_topic = Hub::<EmergencyStop>::new("emergency_stop")?;
loop {
    estop_topic.send(EmergencyStop::release(), &mut None)?;
    std::thread::sleep(std::time::Duration::from_millis(1000));
}
```

### Issue: False high CPU warnings

**Cause**: CPU threshold too low for your application

**Solution**:
```rust
// Increase threshold for CPU-intensive applications
monitor.set_cpu_threshold(95.0);

// Or disable system monitoring by building without sysinfo feature
```

### Issue: Custom safety check always timing out

**Cause**: Not updating the check frequently enough

**Solution**:
```rust
// Update checks more frequently than timeout
let check_timeout = 1000;  // 1 second
monitor.add_safety_check("my_check", check_timeout);

// Update every 500ms (well before timeout)
loop {
    monitor.update_safety_check("my_check", StatusLevel::Ok, "Updated");
    std::thread::sleep(std::time::Duration::from_millis(500));
}
```

### Issue: Safety status not triggering emergency stop

**Cause**: Other nodes not subscribing to safety status

**Solution**:
```rust
// Subscribe to safety status in critical nodes
let safety_topic = Hub::<SafetyStatus>::new("safety_status")?;

loop {
    if let Some(status) = safety_topic.recv(None) {
        if status.estop_engaged || !status.is_safe() {
            // Stop all motion immediately
            stop_all_motors();
        }
    }
}
```

### Issue: Memory threshold constantly exceeded

**Cause**: Memory leak or insufficient RAM

**Solution**:
```rust
// Investigate memory usage
// Check for memory leaks in custom nodes
// Increase threshold temporarily while debugging
monitor.set_memory_threshold(90.0);

// Or add more RAM to the system
```

### Issue: Battery warnings despite full charge

**Cause**: Battery percentage not being published correctly

**Solution**:
```rust
// Verify battery state message
let battery_topic = Hub::<BatteryState>::new("battery_state")?;
if let Some(battery) = battery_topic.recv(None) {
    eprintln!("Battery: {}%", battery.percentage);
}

// Adjust threshold if needed
monitor.set_battery_threshold(10.0);  // Lower threshold
```

## Integration with Emergency Stop

The Safety Monitor Node works in close coordination with the Emergency Stop system:

### Basic Integration

```rust
use horus_library::nodes::{SafetyMonitorNode, EmergencyStopNode};
use horus_core::{Node, Scheduler};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create emergency stop node
    let estop = EmergencyStopNode::new()?;

    // Create safety monitor
    let mut monitor = SafetyMonitorNode::new()?;
    monitor.set_battery_threshold(20.0);
    monitor.set_cpu_threshold(90.0);

    scheduler.add(Box::new(estop), 50, Some(true));
    scheduler.add(Box::new(monitor), 50, Some(true));
    scheduler.run()?;

    Ok(())
}
```

### Safety-Triggered Emergency Stop

```rust
use horus_library::nodes::SafetyMonitorNode;
use horus_library::messages::{SafetyStatus, EmergencyStop};
use horus_core::{Node, Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create safety monitor
    let monitor = SafetyMonitorNode::new()?;
    scheduler.add(Box::new(monitor), 50, Some(true));

    // Monitor safety status and trigger emergency stop
    let safety_topic = Hub::<SafetyStatus>::new("safety_status")?;
    let estop_topic = Hub::<EmergencyStop>::new("emergency_stop")?;

    loop {
        if let Some(status) = safety_topic.recv(None) {
            // Trigger emergency stop on fatal safety condition
            if status.mode == SafetyStatus::MODE_SAFE_STOP {
                let estop = EmergencyStop::engage("Safety system triggered")
                    .with_source("safety_monitor");
                estop_topic.send(estop, &mut None)?;
            }

            // Release emergency stop when safe
            if status.is_safe() {
                estop_topic.send(EmergencyStop::release(), &mut None)?;
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}
```

### Motor Control with Safety Integration

```rust
use horus_library::nodes::SafetyMonitorNode;
use horus_library::messages::{SafetyStatus, MotorCommand};
use horus_core::{Node, Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create safety monitor
    let monitor = SafetyMonitorNode::new()?;
    scheduler.add(Box::new(monitor), 50, Some(true));

    // Motor control with safety override
    let safety_topic = Hub::<SafetyStatus>::new("safety_status")?;
    let motor_topic = Hub::<MotorCommand>::new("motor.command")?;

    let mut allowed_to_move = true;

    loop {
        // Check safety status
        if let Some(status) = safety_topic.recv(None) {
            allowed_to_move = status.is_safe();

            if status.mode == SafetyStatus::MODE_SAFE_STOP {
                // Immediate stop
                motor_topic.send(
                    MotorCommand::Velocity { motor_id: 0, value: 0.0 },
                    None
                )?;
                eprintln!("SAFETY STOP: {}", status.fault_code);
            } else if status.mode == SafetyStatus::MODE_REDUCED {
                // Reduced performance mode
                eprintln!("Reduced performance mode active");
            }
        }

        // Only send motor commands if safe
        if allowed_to_move {
            // Normal motor control logic here
            motor_topic.send(
                MotorCommand::Velocity { motor_id: 0, value: 1.0 },
                None
            )?;
        }

        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    Ok(())
}
```

### Multi-Layer Safety Architecture

```rust
use horus_library::nodes::{SafetyMonitorNode, EmergencyStopNode};
use horus_library::messages::{SafetyStatus, EmergencyStop, StatusLevel};
use horus_core::{Node, Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Layer 1: Emergency Stop Node (hardware integration)
    let estop = EmergencyStopNode::new()?;

    // Layer 2: Safety Monitor Node (system monitoring)
    let mut monitor = SafetyMonitorNode::new()?;
    monitor.set_cpu_threshold(85.0);
    monitor.set_battery_threshold(20.0);
    monitor.add_safety_check("hardware_watchdog", 1000);
    monitor.add_safety_check("navigation_health", 2000);

    scheduler.add(Box::new(estop), 50, Some(true));
    scheduler.add(Box::new(monitor), 50, Some(true));

    // Layer 3: Application-level safety coordinator
    let safety_topic = Hub::<SafetyStatus>::new("safety_status")?;
    let estop_topic = Hub::<EmergencyStop>::new("emergency_stop")?;

    let mut degraded_mode = false;

    loop {
        if let Some(status) = safety_topic.recv(None) {
            match status.mode {
                SafetyStatus::MODE_NORMAL => {
                    degraded_mode = false;
                    // Full performance
                }
                SafetyStatus::MODE_REDUCED => {
                    degraded_mode = true;
                    eprintln!("WARNING: Operating in reduced performance mode");
                    // Reduce speeds, increase safety margins
                }
                SafetyStatus::MODE_SAFE_STOP => {
                    eprintln!("CRITICAL: Safety stop triggered!");
                    // Emergency stop all systems
                    estop_topic.send(
                        EmergencyStop::engage("Safety monitor triggered")
                            .with_source("safety_coordinator"),
                        None
                    )?;
                }
                _ => {}
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}
```

## Performance Considerations

### Update Rate

The Safety Monitor Node should tick at a rate appropriate for your safety requirements:

- **Critical safety systems**: 50-100 Hz (10-20ms tick rate)
- **Standard monitoring**: 10-20 Hz (50-100ms tick rate)
- **Low-frequency checks**: 1-5 Hz (200-1000ms tick rate)

### CPU Usage

Minimal CPU usage with `sysinfo` feature enabled:
- System resource checks: ~0.5-2% CPU per tick
- Custom safety checks: Negligible overhead
- Without `sysinfo`: Nearly zero CPU overhead

### Memory Usage

- Base node: ~500 bytes
- Per custom safety check: ~128 bytes
- System monitoring (with `sysinfo`): ~1-2 KB

### Latency Considerations

- **Resource checks**: <1ms (with `sysinfo`)
- **Communication checks**: <0.1ms
- **Custom checks**: <0.1ms per check
- **Overall safety determination**: <2ms total

## Safety Best Practices

### 1. Always Monitor Critical Parameters

```rust
// Minimum recommended checks
monitor.set_battery_threshold(20.0);
monitor.set_communication_timeout(5000);
monitor.add_safety_check("watchdog", 1000);
```

### 2. Use Appropriate Thresholds

```rust
// Set thresholds based on your system capabilities
// Leave headroom for spikes
monitor.set_cpu_threshold(80.0);     // Not 100%
monitor.set_memory_threshold(75.0);  // Leave room for buffers
```

### 3. Implement Graceful Degradation

```rust
// React to warning levels before critical
if status.mode == SafetyStatus::MODE_REDUCED {
    reduce_performance();  // Slow down before stopping
}
```

### 4. Test Safety Triggers

```rust
// Regularly test safety system responses
#[cfg(test)]
fn test_safety_trigger() {
    monitor.update_safety_check("test", StatusLevel::Fatal, "Test trigger");
    // Verify system responds correctly
}
```

### 5. Log Safety Events

```rust
if status.fault_code != 0 {
    log::error!("Safety fault {}: {:?}", status.fault_code, status);
}
```

## Related Nodes

- **EmergencyStopNode**: Hardware emergency stop integration
- **BatteryMonitorNode**: Detailed battery management
- **DiagnosticNode**: System diagnostics and reporting
- **WatchdogNode**: Process supervision and recovery

## See Also

- [Emergency Stop Node Documentation](emergency_stop_node.md)
- [Functional Safety Standards (IEC 61508)](https://en.wikipedia.org/wiki/IEC_61508)
- [Robot Safety Standards (ISO 10218)](https://en.wikipedia.org/wiki/ISO_10218)
- [Safety Integrity Levels (SIL)](https://en.wikipedia.org/wiki/Safety_integrity_level)
