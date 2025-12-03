# CAN Bus Communication Node

Controller Area Network (CAN) communication for automotive, industrial, and robotics applications with SocketCAN support.

## Overview

The CAN Bus Node provides robust CAN 2.0A/B and CAN-FD communication for connecting to motor controllers, sensors, PLCs, and other CAN-enabled devices. It supports SocketCAN interfaces on Linux, standard and extended identifiers, filtering, error handling, and automatic hardware/simulation fallback.

Supports J1939 (heavy vehicle), CANopen (industrial automation), DeviceNet, and custom CAN protocols.

Key features:
- SocketCAN interface support (can0, vcan0, etc.)
- Standard (11-bit) and Extended (29-bit) identifiers
- CAN-FD with bit rate switching (up to 2 Mbit/s data phase)
- Configurable bitrates (125k, 250k, 500k, 1M)
- ID filtering (whitelist/blacklist)
- Listen-only and loopback modes
- Bus-off auto-recovery
- Error monitoring and statistics
- Hardware fallback to simulation

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `can/{interface}/tx` | `CanFrame` | CAN frames to transmit on the bus |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `can/{interface}/rx` | `CanFrame` | CAN frames received from the bus |
| `can/{interface}/error` | `CanFrame` | Error frames detected on the bus |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `interface_name` | `String` | `"can0"` | CAN interface name (can0, vcan0, etc.) |
| `bitrate` | `u32` | `500000` | Nominal bitrate in bits/second (125k, 250k, 500k, 1M) |
| `fd_bitrate` | `u32` | `2000000` | CAN-FD data phase bitrate in bits/second |
| `enable_fd` | `bool` | `false` | Enable CAN-FD mode |
| `enable_loopback` | `bool` | `false` | Enable loopback mode for testing |
| `enable_listen_only` | `bool` | `false` | Enable listen-only mode (no ACK) |
| `restart_on_bus_off` | `bool` | `true` | Automatically restart after bus-off condition |
| `filter_mode` | `FilterMode` | `Accept` | Filter operation mode (Accept, Whitelist, Blacklist) |

## Hardware Setup

### System Requirements

```bash
# Install CAN utilities
sudo apt install can-utils

# Load SocketCAN kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan  # Virtual CAN for testing
```

### Setup Virtual CAN (Testing)

```bash
# Create virtual CAN interface
sudo ip link add dev can0 type vcan
sudo ip link set up can0

# Verify interface is up
ip link show can0

# Monitor CAN traffic
candump can0
```

### Setup Real CAN Hardware

For physical CAN interfaces (e.g., MCP2515, PCAN-USB, Kvaser):

```bash
# Configure bitrate and bring up interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Verify status
ip -details -statistics link show can0

# Test transmission
cansend can0 123#DEADBEEF
```

### Common CAN Hardware

- **USB-CAN adapters**: PCAN-USB, CANable, Kvaser Leaf, Peak CAN
- **Raspberry Pi**: MCP2515 SPI CAN module with waveshare/seeed drivers
- **Embedded**: STM32/ESP32 with built-in CAN controllers
- **Industrial**: CAN-to-Ethernet gateways (HMS Anybus, Ixxat)

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["can-hardware"] }
```

```bash
cargo build --features="can-hardware"
```

## Message Types

### CanFrame

CAN 2.0A/B and CAN-FD frame format:

```rust
pub struct CanFrame {
    pub id: u32,              // CAN identifier (11-bit or 29-bit)
    pub is_extended: bool,    // Extended frame format (29-bit ID)
    pub is_rtr: bool,         // Remote transmission request
    pub is_error: bool,       // Error frame
    pub data: [u8; 64],       // Data payload (8 bytes for CAN 2.0, 64 for CAN-FD)
    pub dlc: u8,              // Data length code (0-8 for CAN 2.0, 0-64 for CAN-FD)
    pub is_fd: bool,          // CAN-FD frame format
    pub is_brs: bool,         // Bit rate switch (CAN-FD)
    pub is_esi: bool,         // Error state indicator (CAN-FD)
    pub interface: [u8; 16],  // CAN interface name
    pub timestamp: u64,       // Reception/transmission time (ns since epoch)
}
```

**Constants**:
- `CanFrame::MAX_DLC = 8` - CAN 2.0 maximum data length
- `CanFrame::MAX_FD_DLC = 64` - CAN-FD maximum data length
- `CanFrame::MAX_STANDARD_ID = 0x7FF` - 11-bit identifier limit
- `CanFrame::MAX_EXTENDED_ID = 0x1FFFFFFF` - 29-bit identifier limit

**Helper Methods**:
```rust
// Create frames
CanFrame::new(id, data)              // Standard frame
CanFrame::new_extended(id, data)     // Extended frame
CanFrame::new_rtr(id, dlc)          // Remote transmission request
CanFrame::new_fd(id, data, brs)     // CAN-FD frame

// Pack/unpack data
frame.pack_u8(offset, value)         // Pack 8-bit value
frame.pack_u16(offset, value)        // Pack 16-bit value (little-endian)
frame.pack_u32(offset, value)        // Pack 32-bit value (little-endian)
frame.unpack_u8(offset)              // Unpack 8-bit value
frame.unpack_u16(offset)             // Unpack 16-bit value
frame.unpack_u32(offset)             // Unpack 32-bit value

// Interface management
frame.set_interface("can0")          // Set interface name
frame.get_interface()                // Get interface name as string
frame.data_slice()                   // Get data as slice [0..dlc]
frame.is_valid()                     // Check if frame is valid
```

## Public API

### Construction

```rust
use horus_library::nodes::CanBusNode;

// Create CAN node for interface "can0"
let mut can = CanBusNode::new("can0")?;

// Interface name determines topic namespace:
// - Subscribes to: "can.can0.tx"
// - Publishes to: "can.can0.rx" and "can.can0.error"
```

### Configuration Methods

```rust
// Set nominal bitrate (CAN 2.0)
can.set_bitrate(500_000);  // 500 kbit/s

// Set CAN-FD data phase bitrate
can.set_fd_bitrate(2_000_000);  // 2 Mbit/s

// Enable CAN-FD mode
can.enable_can_fd(true);

// Enable loopback mode (for testing without hardware)
can.enable_loopback(true);

// Enable listen-only mode (passive monitoring, no ACK)
can.enable_listen_only(true);

// Enable automatic restart after bus-off
can.enable_auto_restart(true);

// Add CAN ID filter
can.add_filter(0x123, 0x7FF);  // ID, mask (0x7FF = exact match)

// Add CAN ID range filter
can.add_range_filter(0x100, 0x1FF);  // Accept IDs 0x100-0x1FF

// Clear all filters
can.clear_filters();

// Set filter mode
can.set_whitelist_mode();  // Accept only filtered IDs
can.set_blacklist_mode();  // Reject filtered IDs
```

### Preset Configurations

```rust
// Standard bitrates
can.configure_125k();   // 125 kbit/s (CANopen default)
can.configure_250k();   // 250 kbit/s
can.configure_500k();   // 500 kbit/s (common automotive)
can.configure_1m();     // 1 Mbit/s (maximum for CAN 2.0)

// Protocol-specific configurations
can.configure_j1939();              // J1939 heavy vehicle (250k, extended IDs)
can.configure_canopen(1_000_000);   // CANopen (1M, standard IDs, COB-ID filtering)
can.configure_devicenet(125_000);   // DeviceNet (125k/250k/500k)
```

### Query Methods

```rust
// Get bus state
let state = can.get_bus_state();  // ErrorActive, ErrorPassive, BusOff, Stopped

// Get error counters (TX, RX)
let (tx_errors, rx_errors) = can.get_error_counters();

// Get statistics (TX count, RX count, errors, FPS, bus load %)
let (tx_count, rx_count, error_count, fps, bus_load) = can.get_statistics();

// Reset statistics counters
can.reset_statistics();

// Start/stop interface manually (auto-starts on first tick)
can.start(Some(&mut ctx));
can.stop(Some(&mut ctx));
```

## Usage Examples

### Basic CAN Transmission

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create CAN node
    let mut can = CanBusNode::new("can0")?;
    can.set_bitrate(500_000);
    scheduler.add(Box::new(can), 1, Some(true));

    // Create transmitter node
    let tx_node = node! {
        name: "can_transmitter",
        tick: |ctx| {
            // Create a CAN frame
            let frame = CanFrame::new(0x123, &[0x01, 0x02, 0x03, 0x04]);

            // Send to CAN bus
            let hub = Hub::<CanFrame>::new("can.can0.tx")?;
            hub.send(frame, &mut None)?;

            ctx.log_info("Sent CAN frame ID 0x123");
            Ok(())
        }
    };
    scheduler.add(Box::new(tx_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### CAN Frame Reception

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut can = CanBusNode::new("can0")?;
    can.set_bitrate(500_000);
    scheduler.add(Box::new(can), 1, Some(true));

    // Create receiver node
    let rx_node = node! {
        name: "can_receiver",
        tick: |ctx| {
            let hub = Hub::<CanFrame>::new("can.can0.rx")?;

            // Process all received frames
            while let Some(frame) = hub.recv(None) {
                ctx.log_info(&format!(
                    "RX: ID=0x{:03X}{} DLC={} Data={:02X?}",
                    frame.id,
                    if frame.is_extended { " EXT" } else { "" },
                    frame.dlc,
                    frame.data_slice()
                ));

                // Parse specific ID
                if frame.id == 0x200 {
                    if let Some(value) = frame.unpack_u16(0) {
                        ctx.log_info(&format!("Motor speed: {}", value));
                    }
                }
            }
            Ok(())
        }
    };
    scheduler.add(Box::new(rx_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### CAN-FD Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure CAN-FD
    let mut can = CanBusNode::new("can0")?;
    can.enable_can_fd(true);
    can.set_bitrate(500_000);        // Nominal phase: 500 kbit/s
    can.set_fd_bitrate(2_000_000);   // Data phase: 2 Mbit/s

    scheduler.add(Box::new(can), 1, Some(true));

    // Transmit CAN-FD frame
    let tx_node = node! {
        name: "can_fd_tx",
        tick: |ctx| {
            // Create 16-byte CAN-FD frame with BRS
            let data = [0xAA; 16];
            let frame = CanFrame::new_fd(0x456, &data, true);  // ID, data, BRS

            let hub = Hub::<CanFrame>::new("can.can0.tx")?;
            hub.send(frame, &mut None)?;

            ctx.log_info("Sent CAN-FD frame with 16 bytes");
            Ok(())
        }
    };
    scheduler.add(Box::new(tx_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### CAN ID Filtering

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut can = CanBusNode::new("can0")?;
    can.set_bitrate(500_000);

    // Accept only specific ID ranges
    can.add_range_filter(0x100, 0x1FF);  // Accept 0x100-0x1FF
    can.add_range_filter(0x200, 0x2FF);  // Accept 0x200-0x2FF
    can.set_whitelist_mode();            // Enable whitelist filtering

    scheduler.add(Box::new(can), 1, Some(true));

    let rx_node = node! {
        name: "filtered_receiver",
        tick: |ctx| {
            let hub = Hub::<CanFrame>::new("can.can0.rx")?;

            // Only receive frames matching filters
            while let Some(frame) = hub.recv(None) {
                ctx.log_info(&format!("Filtered frame: 0x{:03X}", frame.id));
            }
            Ok(())
        }
    };
    scheduler.add(Box::new(rx_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### J1939 Heavy Vehicle Protocol

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure for J1939 (250 kbit/s, extended IDs)
    let mut can = CanBusNode::new("can0")?;
    can.configure_j1939();

    scheduler.add(Box::new(can), 1, Some(true));

    let j1939_node = node! {
        name: "j1939_engine",
        tick: |ctx| {
            // J1939 PGN 61444 (0xF004): Electronic Engine Controller 1
            // Priority 3, PGN 61444 (0xF004), Source Address 0 (engine)
            let can_id = (3 << 26) | (0xF004 << 8) | 0x00;  // 0x18F00400

            let mut frame = CanFrame::new_extended(can_id, &[0; 8]);
            frame.dlc = 8;

            // Engine speed (0.125 rpm/bit, offset 0)
            let engine_rpm = 1500u16;  // 1500 RPM
            let rpm_raw = (engine_rpm as f32 / 0.125) as u16;
            frame.pack_u16(3, rpm_raw);

            let hub = Hub::<CanFrame>::new("can.can0.tx")?;
            hub.send(frame, &mut None)?;

            ctx.log_info(&format!("Sent J1939 engine RPM: {}", engine_rpm));
            Ok(())
        }
    };
    scheduler.add(Box::new(j1939_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### CANopen Motion Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure for CANopen (1 Mbit/s, standard IDs)
    let mut can = CanBusNode::new("can0")?;
    can.configure_canopen(1_000_000);

    scheduler.add(Box::new(can), 1, Some(true));

    let canopen_node = node! {
        name: "canopen_motor",
        tick: |ctx| {
            let node_id = 5u8;  // Motor controller node ID

            // Send NMT Start command
            let mut nmt_frame = CanFrame::new(0x000, &[0x01, node_id]);
            nmt_frame.dlc = 2;

            // Send PDO with velocity setpoint (COB-ID 0x200 + node_id)
            let velocity_setpoint = 1000i32;  // RPM or counts/sec
            let mut pdo_frame = CanFrame::new(0x200 + node_id as u32, &[0; 4]);
            pdo_frame.pack_u32(0, velocity_setpoint as u32);
            pdo_frame.dlc = 4;

            let hub = Hub::<CanFrame>::new("can.can0.tx")?;
            hub.send(nmt_frame, &mut None)?;
            hub.send(pdo_frame, &mut None)?;

            ctx.log_info("Sent CANopen velocity command");
            Ok(())
        }
    };
    scheduler.add(Box::new(canopen_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Error Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut can = CanBusNode::new("can0")?;
    can.set_bitrate(500_000);
    can.enable_auto_restart(true);

    scheduler.add(Box::new(can), 1, Some(true));

    let monitor_node = node! {
        name: "can_monitor",
        tick: |ctx| {
            // Monitor error frames
            let error_hub = Hub::<CanFrame>::new("can.can0.error")?;
            while let Some(error_frame) = error_hub.recv(None) {
                ctx.log_error(&format!(
                    "CAN error on {}: {:?}",
                    error_frame.get_interface(),
                    error_frame
                ));
            }

            // Periodically check bus health
            // Note: In real implementation, you'd query the CanBusNode directly
            // This is a simplified example
            Ok(())
        }
    };
    scheduler.add(Box::new(monitor_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Multiple CAN Interfaces

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // CAN0: Vehicle bus (500 kbit/s)
    let mut can0 = CanBusNode::new("can0")?;
    can0.set_bitrate(500_000);
    scheduler.add(Box::new(can0), 1, Some(true));

    // CAN1: Sensor bus (250 kbit/s)
    let mut can1 = CanBusNode::new("can1")?;
    can1.set_bitrate(250_000);
    scheduler.add(Box::new(can1), 1, Some(true));

    // Gateway node: forward frames between buses
    let gateway_node = node! {
        name: "can_gateway",
        tick: |ctx| {
            // Forward CAN0 -> CAN1
            let can0_rx = Hub::<CanFrame>::new("can.can0.rx")?;
            let can1_tx = Hub::<CanFrame>::new("can.can1.tx")?;

            while let Some(frame) = can0_rx.recv(None) {
                if frame.id >= 0x200 && frame.id <= 0x2FF {
                    can1_tx.send(frame, &mut None)?;
                    ctx.log_debug(&format!("Forwarded 0x{:03X} to CAN1", frame.id));
                }
            }

            // Forward CAN1 -> CAN0
            let can1_rx = Hub::<CanFrame>::new("can.can1.rx")?;
            let can0_tx = Hub::<CanFrame>::new("can.can0.tx")?;

            while let Some(frame) = can1_rx.recv(None) {
                if frame.id >= 0x300 && frame.id <= 0x3FF {
                    can0_tx.send(frame, &mut None)?;
                    ctx.log_debug(&format!("Forwarded 0x{:03X} to CAN0", frame.id));
                }
            }
            Ok(())
        }
    };
    scheduler.add(Box::new(gateway_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Listen-Only Mode (Bus Monitoring)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure listen-only mode (passive monitoring)
    let mut can = CanBusNode::new("can0")?;
    can.set_bitrate(500_000);
    can.enable_listen_only(true);  // No ACK, silent monitoring

    scheduler.add(Box::new(can), 1, Some(true));

    let logger_node = node! {
        name: "can_logger",
        tick: |ctx| {
            let hub = Hub::<CanFrame>::new("can.can0.rx")?;

            while let Some(frame) = hub.recv(None) {
                // Log all CAN traffic without affecting the bus
                println!("[{}] 0x{:03X}  [{}]  {:02X?}",
                    frame.timestamp,
                    frame.id,
                    frame.dlc,
                    frame.data_slice()
                );
            }
            Ok(())
        }
    };
    scheduler.add(Box::new(logger_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Testing with can-utils

### Monitor CAN Traffic

```bash
# Monitor all frames on can0
candump can0

# Monitor with timestamps
candump -t a can0

# Monitor with delta timestamps
candump -t d can0

# Monitor specific ID
candump can0,123:7FF

# Save to log file
candump -l can0
```

### Send Test Frames

```bash
# Send standard frame
cansend can0 123#DEADBEEF

# Send extended frame
cansend can0 12345678#1122334455667788

# Send RTR frame
cansend can0 123#R

# Send with DLC but no data
cansend can0 123#R8
```

### Generate Traffic

```bash
# Generate random CAN traffic
cangen can0 -v

# Generate specific ID range
cangen can0 -I 100 -L 8 -D r -v

# Generate at specific rate
cangen can0 -g 10 -v  # 10ms gap between frames
```

### Bus Statistics

```bash
# Show interface statistics
ip -s -s link show can0

# Monitor error frames
candump -e can0

# Display timing
cansequence -r can0
```

### Replay Captured Traffic

```bash
# Capture traffic to file
candump -l can0

# Replay from file
canplayer -I candump-2024-01-01.log

# Replay with original timing
canplayer -v -I candump.log
```

## Timing and Performance

### Bitrate Selection

| Bitrate | Max Cable Length | Typical Use Case |
|---------|------------------|------------------|
| 1 Mbit/s | 40 meters | High-speed control, short cables |
| 500 kbit/s | 100 meters | Automotive ECU communication |
| 250 kbit/s | 250 meters | J1939 heavy vehicle, industrial |
| 125 kbit/s | 500 meters | CANopen, building automation |

### Bus Arbitration

CAN uses CSMA/CR (Carrier Sense Multiple Access with Collision Resolution):
- Lower CAN IDs have higher priority
- ID 0x000 has highest priority
- ID 0x7FF (standard) or 0x1FFFFFFF (extended) has lowest priority

### Message Timing

```
Standard CAN frame (8 bytes data, 500 kbit/s):
- Frame size: ~130 bits (including stuff bits)
- Transmission time: ~260 microseconds
- Maximum throughput: ~3800 frames/second

CAN-FD frame (64 bytes data, 500k nominal / 2M data):
- Frame size: ~600 bits
- Transmission time: ~400 microseconds
- Maximum throughput: ~2500 frames/second
```

### Bus Load Estimation

```rust
// Calculated automatically by CanBusNode
let (_, _, _, fps, bus_load) = can.get_statistics();
println!("Bus load: {:.1}%", bus_load);

// Warning: Bus load > 70% may cause delays
// Critical: Bus load > 90% may cause frame drops
```

## Best Practices

1. **Use appropriate bitrates**:
   ```rust
   // Match the network bitrate
   can.set_bitrate(500_000);  // Common automotive standard
   ```

2. **Enable ID filtering for performance**:
   ```rust
   // Only receive relevant messages
   can.add_range_filter(0x100, 0x1FF);
   can.set_whitelist_mode();
   ```

3. **Handle bus-off conditions**:
   ```rust
   can.enable_auto_restart(true);  // Automatically recover from bus-off
   ```

4. **Monitor error counters**:
   ```rust
   let (tx_err, rx_err) = can.get_error_counters();
   if tx_err > 100 || rx_err > 100 {
       ctx.log_warning("High CAN error count - check wiring");
   }
   ```

5. **Use proper termination**:
   - CAN bus requires 120 Ohm termination resistors at both ends
   - Measure resistance between CANH and CANL (should be ~60 Ohms)

6. **Validate frames before sending**:
   ```rust
   if frame.is_valid() {
       hub.send(frame, &mut None)?;
   } else {
       ctx.log_error("Invalid CAN frame");
   }
   ```

7. **Test with virtual CAN first**:
   ```bash
   sudo ip link add dev vcan0 type vcan
   sudo ip link set up vcan0
   ```

8. **Monitor bus load**:
   ```rust
   let (_, _, _, _, load) = can.get_statistics();
   if load > 80.0 {
       ctx.log_warning(&format!("High bus load: {:.1}%", load));
   }
   ```

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] CanBusNode: Hardware unavailable - using SIMULATION mode
[WARN]   Tried: can0
[WARN]   Error: No such device
```

**Solutions:**
1. Check if interface exists:
   ```bash
   ip link show can0
   ```

2. Create virtual CAN for testing:
   ```bash
   sudo modprobe vcan
   sudo ip link add dev can0 type vcan
   sudo ip link set up can0
   ```

3. For physical hardware, configure bitrate:
   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set up can0
   ```

4. Load required kernel modules:
   ```bash
   sudo modprobe can
   sudo modprobe can_raw
   ```

5. Install CAN utilities:
   ```bash
   sudo apt install can-utils
   ```

6. Rebuild with hardware support:
   ```bash
   cargo build --features="can-hardware"
   ```

### No frames received

**Solutions:**
1. Verify interface is up:
   ```bash
   ip link show can0
   ```

2. Check for traffic with candump:
   ```bash
   candump can0
   ```

3. Verify bitrate matches network:
   ```bash
   ip -details link show can0
   ```

4. Check CAN termination (should measure ~60 Ohms between CANH/CANL)

5. Verify wiring connections (CANH, CANL, GND)

6. Disable filtering temporarily:
   ```rust
   can.clear_filters();
   ```

### Bus-off errors

```
[ERROR] CAN bus-off on can0: TxErr=256, RxErr=128
```

**Solutions:**
1. Check for short circuit or missing termination
2. Verify bitrate matches network
3. Reduce transmission rate
4. Check for defective transceiver
5. Ensure proper grounding

### High error count

**Solutions:**
1. Check cable quality and length
2. Verify termination resistors (120 Ohms at both ends)
3. Reduce bitrate for longer cables
4. Check for electromagnetic interference
5. Verify transceiver voltage levels

### Permission denied errors

```
Error: Permission denied (os error 13)
```

**Solutions:**
1. Add user to dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. Set CAN interface permissions:
   ```bash
   sudo chmod 666 /dev/can*
   ```

3. Run with sudo (not recommended for production):
   ```bash
   sudo ./target/release/my_can_app
   ```

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] Starting CAN interface can0 at 500 kbit/s
[WARN] CanBusNode: Hardware unavailable - using SIMULATION mode
[DEBUG] TX CAN (SIM): id=0x123 dlc=4 data=[01, 02, 03, 04]
[DEBUG] RX CAN: id=0x100 dlc=2 data=[00, 00]
```

Simulated behavior:
- Transmissions are logged but not sent to hardware
- Receives periodic test frames (IDs 0x100-0x107)
- Includes realistic timing and statistics
- Error conditions can be simulated
- Useful for logic testing without hardware

## See Also

- [I2cBusNode](../i2c_bus/) - I2C serial communication
- [SpiBusNode](../spi_bus/) - SPI serial communication
- [ModbusNode](../modbus/) - Modbus RTU/TCP protocol
- [SerialNode](../serial/) - UART/RS232/RS485 communication
- [BldcMotorNode](../bldc_motor/) - BLDC motor control (can use CAN)
- [OdriveNode](../odrive/) - ODrive motor controller (CAN interface)
