# Battery Monitor Node

Battery voltage, current, charge state, and cell health monitoring for robotics power management.

## Overview

The Battery Monitor Node provides comprehensive battery monitoring for robotic systems. It tracks voltage, current draw, state of charge (SOC), remaining capacity, temperature, and individual cell voltages. The node supports various battery chemistries and monitoring interfaces, with automatic low battery warnings and critical alerts for safe system operation.

### Supported Hardware

- **I2C/SMBus Fuel Gauges**: BQ27441, MAX17043, LC709203F
- **Analog Voltage/Current Sensors**: INA219, INA226, ACS712
- **Power Modules**: PM07, PM06, Matek systems with telemetry
- **Battery Management Systems**: BMS via UART/CAN
- **Direct ADC Sampling**: Voltage dividers and current shunts

### Battery Types

- **LiPo/LiFePO4**: 1S-6S+ configurations
- **Li-ion**: 18650, 21700, custom packs
- **NiMH**: 6-12 cell packs
- **Lead-acid**: 6V, 12V, 24V systems
- **Custom battery packs**: Configurable chemistry

### Key Features

- Voltage monitoring with cell-level detection (up to 16 cells)
- Current sensing (charge/discharge)
- State of charge (SOC) estimation with Coulomb counting
- Remaining capacity and runtime calculation
- Temperature monitoring with over-temperature warnings
- Low battery warnings and critical alerts
- Charge cycle counting
- Moving average voltage filtering
- I2C hardware support (INA219/INA226)
- Simulation mode for testing without hardware

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `battery` | `BatteryState` | Battery status including voltage, current, charge, temperature |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cell_count` | `u8` | `3` | Number of cells in series (1-16) |
| `nominal_voltage_per_cell` | `f32` | `3.7` | Nominal voltage per cell (V) |
| `capacity_mah` | `f32` | `5000.0` | Battery capacity in milliamp-hours |
| `chemistry` | `BatteryChemistry` | `LiPo` | Battery chemistry type |
| `monitor_interface` | `MonitorInterface` | `Simulated` | Monitoring hardware interface |
| `full_voltage` | `f32` | `12.6` | Fully charged voltage (V) |
| `nominal_voltage` | `f32` | `11.1` | Nominal/average voltage (V) |
| `low_voltage` | `f32` | `10.5` | Low battery warning threshold (V) |
| `critical_voltage` | `f32` | `9.9` | Critical shutdown threshold (V) |
| `max_current` | `f32` | `100.0` | Maximum current threshold (A) |
| `max_temperature` | `f32` | `60.0` | Maximum temperature threshold (C) |
| `sampling_rate` | `f32` | `1.0` | Measurement frequency in Hz (0.1-100) |
| `enable_cell_monitoring` | `bool` | `false` | Enable individual cell voltage monitoring |
| `i2c_bus` | `u8` | `1` | I2C bus number for INA219/INA226 |
| `i2c_address` | `u16` | `0x40` | I2C device address |
| `shunt_resistance_mohm` | `f32` | `100.0` | Shunt resistor value in milliohms |

## Message Types

### BatteryState

Battery status message with comprehensive monitoring data:

```rust
pub struct BatteryState {
    pub voltage: f32,              // Voltage in volts
    pub current: f32,              // Current in amperes (negative = discharging)
    pub charge: f32,               // Charge in amp-hours (NaN if unknown)
    pub capacity: f32,             // Capacity in amp-hours (NaN if unknown)
    pub percentage: f32,           // Percentage charge (0-100)
    pub power_supply_status: u8,   // Power supply status
    pub temperature: f32,          // Temperature in celsius
    pub cell_voltages: [f32; 16],  // Individual cell voltages
    pub cell_count: u8,            // Number of valid cell voltage readings
    pub timestamp: u64,            // Timestamp in nanoseconds since epoch
}
```

**Power Supply Status Constants:**
- `BatteryState::STATUS_UNKNOWN = 0`
- `BatteryState::STATUS_CHARGING = 1`
- `BatteryState::STATUS_DISCHARGING = 2`
- `BatteryState::STATUS_FULL = 3`

**Helper Methods:**
```rust
impl BatteryState {
    pub fn new(voltage: f32, percentage: f32) -> Self;
    pub fn is_low(&self, threshold: f32) -> bool;
    pub fn is_critical(&self) -> bool;
    pub fn time_remaining(&self) -> Option<f32>; // Seconds remaining
}
```

### Battery Chemistry Types

```rust
pub enum BatteryChemistry {
    LiPo,      // Lithium Polymer (3.7V nominal, 4.2V max, 3.3V min)
    LiFePO4,   // Lithium Iron Phosphate (3.2V nominal, 3.65V max, 2.5V min)
    LiIon,     // Lithium Ion (3.6V nominal, 4.2V max, 3.0V min)
    NiMH,      // Nickel Metal Hydride (1.2V nominal, 1.4V max, 0.9V min)
    LeadAcid,  // Lead Acid (2.0V nominal, 2.15V max, 1.75V min)
    Custom,    // Custom chemistry with manual voltage thresholds
}
```

### Monitor Interface Types

```rust
pub enum MonitorInterface {
    I2C,       // I2C fuel gauge (INA219, INA226)
    Analog,    // ADC sampling
    UART,      // Serial BMS
    CAN,       // CAN bus BMS
    PWM,       // PWM power module
    Simulated, // Software simulation
}
```

## Public API

### Construction

```rust
use horus_library::nodes::BatteryMonitorNode;

// Create with default topic "battery"
let mut battery = BatteryMonitorNode::new()?;

// Create with custom topic
let mut battery = BatteryMonitorNode::new_with_topic("power.battery")?;
```

### Configuration Methods

```rust
// Set number of cells in series (1-16)
battery.set_cell_count(3);  // 3S battery

// Set battery capacity in mAh
battery.set_capacity(5000.0);  // 5000 mAh

// Set battery chemistry (auto-configures voltage thresholds)
battery.set_chemistry(BatteryChemistry::LiPo);
battery.set_chemistry(BatteryChemistry::LiFePO4);
battery.set_chemistry(BatteryChemistry::LiIon);

// Set monitoring interface
battery.set_monitor_interface(MonitorInterface::I2C);

// Set custom voltage thresholds (volts)
battery.set_voltage_thresholds(
    12.6,  // full
    11.1,  // nominal
    10.5,  // low warning
    9.9    // critical
);

// Set individual thresholds
battery.set_low_voltage_threshold(10.5);
battery.set_critical_voltage_threshold(9.9);

// Set protection limits
battery.set_max_current(100.0);      // 100A max
battery.set_max_temperature(60.0);   // 60C max

// Set sampling rate in Hz
battery.set_sampling_rate(1.0);  // 1 Hz

// Enable individual cell monitoring
battery.enable_cell_monitoring(true);

// Configure I2C hardware (for INA219/INA226)
battery.set_i2c_config(
    1,      // I2C bus number
    0x40,   // I2C address
    100.0   // Shunt resistance in milliohms
);
```

### Query Methods

```rust
// Get current battery state
let state = battery.get_state();

// Check if battery is healthy
if battery.is_healthy() {
    println!("Battery OK");
}

// Get estimated remaining time in seconds
if let Some(time) = battery.time_remaining() {
    println!("Time remaining: {:.0} minutes", time / 60.0);
}
```

### Preset Configurations

```rust
// Configure for 3S LiPo battery
battery.configure_3s_lipo(5000.0);  // 5000 mAh

// Configure for 4S LiPo battery
battery.configure_4s_lipo(10000.0);  // 10000 mAh

// Configure for 6S LiPo battery
battery.configure_6s_lipo(8000.0);  // 8000 mAh

// Configure for 12V lead-acid battery
battery.configure_12v_lead_acid(35000.0);  // 35Ah = 35000 mAh

// Configure for LiFePO4 battery
battery.configure_lifepo4(4, 6000.0);  // 4S, 6000 mAh
```

## Usage Examples

### Basic Battery Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create and configure battery monitor
    let mut battery = BatteryMonitorNode::new()?;
    battery.configure_3s_lipo(5000.0);  // 3S 5000mAh LiPo
    battery.set_sampling_rate(1.0);     // 1 Hz updates

    scheduler.add(Box::new(battery), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Battery with Low Voltage Alert

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Battery monitor
    let mut battery = BatteryMonitorNode::new()?;
    battery.configure_4s_lipo(10000.0);
    battery.set_low_voltage_threshold(14.0);      // 3.5V per cell
    battery.set_critical_voltage_threshold(13.2); // 3.3V per cell
    scheduler.add(Box::new(battery), 1, Some(true));

    // Alert node
    let alert_node = node! {
        name: "battery_alert",
        tick: |ctx| {
            let hub = Hub::<BatteryState>::new("battery")?;

            while let Some(state) = hub.recv(None) {
                if state.percentage < 20.0 {
                    ctx.log_warning(&format!(
                        "Low battery: {:.1}% ({:.2}V)",
                        state.percentage, state.voltage
                    ));
                }

                if state.is_critical() {
                    ctx.log_error("CRITICAL BATTERY - LAND IMMEDIATELY!");
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(alert_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### I2C Hardware Monitoring (INA219)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut battery = BatteryMonitorNode::new()?;

    // Configure for I2C monitoring with INA219
    battery.set_monitor_interface(MonitorInterface::I2C);
    battery.set_i2c_config(
        1,      // I2C bus /dev/i2c-1
        0x40,   // Default INA219 address
        100.0   // 100 milliohm shunt resistor
    );

    // Configure battery specs
    battery.set_cell_count(3);
    battery.set_capacity(5000.0);
    battery.set_chemistry(BatteryChemistry::LiPo);
    battery.set_sampling_rate(2.0);  // 2 Hz

    scheduler.add(Box::new(battery), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Cell-Level Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Battery with cell monitoring
    let mut battery = BatteryMonitorNode::new()?;
    battery.configure_4s_lipo(8000.0);
    battery.enable_cell_monitoring(true);
    scheduler.add(Box::new(battery), 1, Some(true));

    // Cell balance checker
    let checker = node! {
        name: "cell_balance_checker",
        tick: |ctx| {
            let hub = Hub::<BatteryState>::new("battery")?;

            while let Some(state) = hub.recv(None) {
                if state.cell_count > 0 {
                    let cells: Vec<f32> = state.cell_voltages[..state.cell_count as usize].to_vec();
                    let min_cell = cells.iter().cloned().fold(f32::INFINITY, f32::min);
                    let max_cell = cells.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                    let imbalance = max_cell - min_cell;

                    if imbalance > 0.1 {
                        ctx.log_warning(&format!(
                            "Cell imbalance: {:.3}V (min={:.3}V, max={:.3}V)",
                            imbalance, min_cell, max_cell
                        ));
                    }

                    ctx.log_debug(&format!("Cells: {:?}", cells));
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(checker), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Runtime Calculator

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut battery = BatteryMonitorNode::new()?;
    battery.configure_3s_lipo(5000.0);
    scheduler.add(Box::new(battery), 1, Some(true));

    let runtime_calc = node! {
        name: "runtime_calculator",
        tick: |ctx| {
            let hub = Hub::<BatteryState>::new("battery")?;

            while let Some(state) = hub.recv(None) {
                // Calculate power consumption
                let power_watts = state.voltage * state.current.abs();

                // Calculate efficiency
                let nominal_voltage = 11.1;  // 3S nominal
                let efficiency = if state.current < 0.0 {
                    state.voltage / nominal_voltage
                } else {
                    1.0
                };

                // Display status
                if state.current < -0.1 {  // Discharging
                    if let Some(time_remaining) = state.time_remaining() {
                        ctx.log_info(&format!(
                            "Battery: {:.1}% | {:.2}V | {:.1}A | {:.1}W | {:.0}min remaining",
                            state.percentage,
                            state.voltage,
                            state.current.abs(),
                            power_watts,
                            time_remaining / 60.0
                        ));
                    }
                } else if state.current > 0.1 {  // Charging
                    ctx.log_info(&format!(
                        "Charging: {:.1}% | {:.2}V | +{:.1}A",
                        state.percentage, state.voltage, state.current
                    ));
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(runtime_calc), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### INA219/INA226 Wiring

The INA219 and INA226 are I2C current/voltage sensors commonly used for battery monitoring:

```
INA219/INA226          Raspberry Pi
VCC               ---> 3.3V
GND               ---> GND
SDA               ---> GPIO 2 (SDA)
SCL               ---> GPIO 3 (SCL)

Battery Connection:
VIN+  <--- Battery Positive
VIN-  <--- Load/ESC Positive

       Battery Negative ----> Load/ESC Negative
```

### System Requirements

```bash
# Install I2C tools
sudo apt install i2c-tools libi2c-dev

# Enable I2C interface
sudo raspi-config
# Select: Interface Options -> I2C -> Enable

# Verify INA219 detection (address 0x40)
i2cdetect -y 1

# Expected output shows "40" in the grid
```

### Enable in Project

Add to `Cargo.toml`:
```toml
[dependencies]
horus_library = { version = "0.1", features = ["i2c-hardware"] }
```

Build with I2C support:
```bash
cargo build --features="i2c-hardware"
```

## Battery Chemistry Voltage Characteristics

### LiPo (Lithium Polymer)

| State | Voltage/Cell | 3S (V) | 4S (V) | 6S (V) |
|-------|--------------|--------|--------|--------|
| Fully Charged | 4.20V | 12.6 | 16.8 | 25.2 |
| Nominal | 3.70V | 11.1 | 14.8 | 22.2 |
| Low Warning | 3.50V | 10.5 | 14.0 | 21.0 |
| Critical | 3.30V | 9.9 | 13.2 | 19.8 |
| Damage Threshold | 3.00V | 9.0 | 12.0 | 18.0 |

### LiFePO4 (Lithium Iron Phosphate)

| State | Voltage/Cell | 4S (V) |
|-------|--------------|--------|
| Fully Charged | 3.65V | 14.6 |
| Nominal | 3.20V | 12.8 |
| Low Warning | 3.00V | 12.0 |
| Critical | 2.50V | 10.0 |

### Lead-Acid (12V system)

| State | Total Voltage |
|-------|---------------|
| Fully Charged | 12.9V |
| Nominal | 12.0V |
| Low Warning | 11.1V |
| Critical | 10.5V |

## Best Practices

1. **Set appropriate chemistry:**
   ```rust
   battery.set_chemistry(BatteryChemistry::LiPo);  // Match your battery
   ```

2. **Configure capacity accurately:**
   ```rust
   battery.set_capacity(5000.0);  // Actual battery capacity in mAh
   ```

3. **Use preset configurations when possible:**
   ```rust
   battery.configure_3s_lipo(5000.0);  // Automatic threshold setup
   ```

4. **Monitor cell balance regularly:**
   ```rust
   battery.enable_cell_monitoring(true);
   // Check cell voltages for imbalance > 0.1V
   ```

5. **Set conservative voltage thresholds:**
   ```rust
   // Leave safety margin above damage threshold
   battery.set_critical_voltage_threshold(9.9);  // 3.3V/cell for 3S
   ```

6. **Implement emergency shutdown:**
   ```rust
   if state.voltage <= critical_voltage {
       emergency_land();
       shutdown_motors();
   }
   ```

7. **Never over-discharge LiPo batteries:**
   - LiPo damaged below 3.0V per cell
   - Set critical threshold at 3.3V per cell minimum
   - Land/stop before critical threshold

## Troubleshooting

### Issue: "Hardware unavailable - using SIMULATION mode"

```
[WARN] BatteryMonitorNode: Hardware unavailable - using SIMULATION mode
[WARN]   Tried: /dev/i2c-1 address 0x40
[WARN]   Error: Permission denied
```

**Solutions:**
1. Install I2C tools: `sudo apt install i2c-tools`
2. Enable I2C: `sudo raspi-config` -> Interface Options -> I2C
3. Verify wiring: Check INA219/INA226 connections
4. Test I2C: `i2cdetect -y 1` (should show device at 0x40)
5. Check permissions: `sudo usermod -a -G i2c $USER`
6. Rebuild: `cargo build --features="i2c-hardware"`

### Issue: Inaccurate SOC readings

**Possible Causes:**
1. Incorrect capacity setting
2. Not calibrated to full charge
3. Battery aging/degradation

**Solutions:**
```rust
// Set accurate capacity
battery.set_capacity(4800.0);  // Measured capacity, not rated

// Reset SOC after full charge
battery.set_capacity(5000.0);  // Resets to 100%
```

### Issue: Voltage drops suddenly under load

This is normal battery behavior (internal resistance). The node includes voltage filtering:

```rust
// Increase filtering window (already enabled by default)
// The node uses 10-sample moving average automatically
```

### Issue: Cell voltages all showing 0.0V

**Cause:** Cell monitoring not enabled or hardware doesn't support it

**Solution:**
```rust
battery.enable_cell_monitoring(true);
```

Note: Cell-level monitoring requires BMS hardware support. Not all monitoring interfaces provide individual cell voltages.

## Simulation Mode

When I2C hardware is unavailable, the node automatically operates in simulation mode:

```
[INFO] Battery: 11.12V (85%) -2.5A 25.3C DISCHARGING | 102min remaining
```

Simulated behavior:
- Voltage decreases with SOC using realistic discharge curve
- Current draw simulated based on typical loads
- Temperature increases with current draw
- Useful for algorithm testing without hardware

## See Also

- [MotorNode](../dc_motor/) - Motor control (current consumers)
- [SafetyMonitorNode](../safety_monitor/) - Safety system integration
- [PowerManagementNode](../power_management/) - Power distribution
- [TelemetryNode](../telemetry/) - Data logging and monitoring
