# Soft Real-Time Scheduling

## Overview

HORUS Python API provides soft real-time scheduling capabilities, allowing you to set millisecond-level deadlines for individual nodes and monitor deadline violations. This feature is essential for:
- Time-critical control loops
- Sensor data processing with latency requirements
- Performance monitoring and optimization
- Identifying computational bottlenecks

## Key Features

### 1. Per-Node Deadlines

Set individual execution deadlines for each node:

```python
import horus

config = horus.SchedulerConfig.standard()
config.deadline_monitoring = True  # Enable deadline warnings

scheduler = horus.Scheduler.from_config(config)
scheduler.add(my_node, priority=1)

# Get node name
names = scheduler.get_node_names()

# Set 10ms deadline for this node
scheduler.set_node_deadline(names[0], 10.0)
```

### 2. Automatic Deadline Monitoring

When enabled, the scheduler automatically tracks execution time and logs warnings for violations:

```python
# Deadline miss warnings are automatically logged to stderr:
# "Deadline miss: node 'sensor_node' took 25.15ms (deadline: 10ms, miss #1)"
```

### 3. Deadline Statistics

Query deadline violation statistics via node introspection:

```python
nodes = scheduler.get_all_nodes()
for node in nodes:
    print(f"Node: {node['name']}")
    print(f"  Deadline: {node.get('deadline_ms', 'N/A')}ms")
    print(f"  Total misses: {node.get('deadline_misses', 0)}")
    print(f"  Last tick duration: {node.get('last_tick_duration_ms', 0):.3f}ms")
    print(f"  Avg tick duration: {node.get('avg_tick_duration_ms', 0):.3f}ms")
```

## API Reference

### `SchedulerConfig.deadline_monitoring`

Enable/disable deadline monitoring globally.

**Type:** `bool`
**Default:** `False`

**Example:**
```python
config = horus.SchedulerConfig.standard()
config.deadline_monitoring = True
scheduler = horus.Scheduler.from_config(config)
```

**Note:** When `False`, deadline violations are still tracked but not logged.

### `Scheduler.set_node_deadline(node_name: str, deadline_ms: Optional[float])`

Set or clear a deadline for a specific node.

**Parameters:**
- `node_name` (str): Name of the node to configure
- `deadline_ms` (Optional[float]): Deadline in milliseconds (0-10000), or `None` to disable

**Raises:**
- `RuntimeError`: If node not found or deadline out of range

**Example:**
```python
# Set deadline
scheduler.set_node_deadline("sensor_node", 10.0)

# Clear deadline
scheduler.set_node_deadline("sensor_node", None)
```

### Deadline Metrics

Available via `get_node_stats()` and `get_all_nodes()`:

| Field | Type | Description |
|-------|------|-------------|
| `deadline_ms` | float\|None | Configured deadline in milliseconds |
| `deadline_misses` | int | Total number of deadline violations |
| `last_tick_duration_ms` | float | Duration of most recent tick |
| `avg_tick_duration_ms` | float | Average tick duration |
| `min_tick_duration_ms` | float | Minimum tick duration |
| `max_tick_duration_ms` | float | Maximum tick duration |

## Use Cases

### 1. Control Loop Latency Monitoring

Ensure control loops execute within acceptable time bounds:

```python
class ControllerNode(horus.Node):
    def tick(self):
        # Read sensors
        sensor_data = self.get("sensor_input")

        # Compute control output
        output = self.compute_control(sensor_data)

        # Send to actuators
        self.send("actuator_cmd", output)

config = horus.SchedulerConfig.standard()
config.tick_rate = 100.0  # 100 Hz control loop
config.deadline_monitoring = True

scheduler = horus.Scheduler.from_config(config)
controller = ControllerNode()
scheduler.add(controller, priority=1)

# Set 5ms deadline (well below 10ms period at 100Hz)
names = scheduler.get_node_names()
scheduler.set_node_deadline(names[0], 5.0)

scheduler.run()
```

### 2. Performance Budgeting

Allocate time budgets to different components:

```python
# Sensor node: 2ms budget
scheduler.set_node_deadline(sensor_name, 2.0)

# Processing node: 10ms budget
scheduler.set_node_deadline(processing_name, 10.0)

# Control node: 5ms budget
scheduler.set_node_deadline(control_name, 5.0)
```

### 3. Identifying Bottlenecks

Find nodes that consistently miss deadlines:

```python
def analyze_deadline_performance(scheduler):
    nodes = scheduler.get_all_nodes()

    for node in nodes:
        total_ticks = node.get('total_ticks', 0)
        misses = node.get('deadline_misses', 0)

        if total_ticks > 0:
            miss_rate = (misses / total_ticks) * 100

            if miss_rate > 10:
                print(f"[WARNING]️  {node['name']}: {miss_rate:.1f}% miss rate")
                print(f"   Avg: {node.get('avg_tick_duration_ms', 0):.3f}ms")
                print(f"   Max: {node.get('max_tick_duration_ms', 0):.3f}ms")
                print(f"   Deadline: {node.get('deadline_ms')}ms")
```

### 4. Real-Time Monitor

Create a live monitoring display:

```python
import time

def monitor_deadlines(scheduler, interval=1.0):
    while True:
        time.sleep(interval)

        nodes = scheduler.get_all_nodes()
        print("\n" + "=" * 70)
        print("Deadline Monitor")
        print("=" * 70)

        for node in nodes:
            name = node['name']
            deadline = node.get('deadline_ms', 'N/A')
            misses = node.get('deadline_misses', 0)
            last_dur = node.get('last_tick_duration_ms', 0)

            status = "[OK]" if last_dur < deadline else "[FAIL]"
            print(f"{status} {name}: {last_dur:.3f}ms / {deadline}ms (misses: {misses})")
```

### 5. Adaptive Rate Control

Dynamically adjust node rates based on deadline performance:

```python
def adaptive_rate_control(scheduler):
    nodes = scheduler.get_all_nodes()

    for node in nodes:
        total_ticks = node.get('total_ticks', 0)
        misses = node.get('deadline_misses', 0)
        avg_duration = node.get('avg_tick_duration_ms', 0)
        deadline = node.get('deadline_ms')

        if total_ticks < 10:
            continue  # Wait for sufficient data

        miss_rate = misses / total_ticks

        if miss_rate > 0.2:  # > 20% misses
            # Reduce rate or increase deadline
            new_deadline = avg_duration * 1.5
            scheduler.set_node_deadline(node['name'], new_deadline)
            print(f"Adjusted {node['name']} deadline to {new_deadline:.1f}ms")
```

## Demo

Run the included demo to see soft real-time scheduling in action:

```bash
python3 demo_soft_realtime.py
```

The demo shows:
1. Configuring deadline monitoring
2. Setting per-node deadlines
3. Real-time deadline violation warnings
4. Post-run performance analysis
5. Miss rate calculations

## Best Practices

### 1. Setting Realistic Deadlines

- **Too tight**: Frequent violations, noise in logs
- **Too loose**: Won't catch real issues
- **Rule of thumb**: Set deadline at 80% of period (e.g., 8ms for 100Hz node)

```python
# For 100 Hz node (10ms period):
scheduler.set_node_deadline(node_name, 8.0)  # 8ms = 80% of period
```

### 2. Enable Monitoring During Development

```python
# Development: verbose monitoring
config.deadline_monitoring = True

# Production: silent tracking
config.deadline_monitoring = False
```

### 3. Monitor Miss Rates, Not Individual Misses

A few misses are normal due to scheduling jitter. Focus on trends:

```python
miss_rate = deadline_misses / total_ticks
if miss_rate > 0.1:  # > 10% is problematic
    print(f"Warning: High miss rate ({miss_rate*100:.1f}%)")
```

### 4. Different Deadlines for Different Priorities

```python
# High-priority fast nodes: tight deadlines
scheduler.set_node_deadline(sensor_name, 2.0)

# Lower-priority processing: looser deadlines
scheduler.set_node_deadline(analytics_name, 50.0)
```

### 5. Combine with Fault Tolerance

```python
config = horus.SchedulerConfig.standard()
config.deadline_monitoring = True  # Track performance
config.circuit_breaker = True      # Handle failures
config.max_failures = 5

scheduler = horus.Scheduler.from_config(config)
```

## Soft vs Hard Real-Time

HORUS provides **soft real-time** guarantees:

| Aspect | Soft Real-Time (HORUS) | Hard Real-Time |
|--------|------------------------|----------------|
| Deadline miss | Logged, tracked | System failure |
| Guarantee | Best effort | Guaranteed |
| Overhead | Low | High |
| Use case | Most robotics, monitoring | Safety-critical |
| Jitter tolerance | Yes | No |

**When to use soft real-time:**
- Sensor data processing
- Control loops with graceful degradation
- Monitoring and logging
- Non-safety-critical automation

**When you need hard real-time:**
- Safety systems (airbags, brakes)
- Critical infrastructure control
- Medical devices
- Guaranteed response requirements

## Performance Impact

Deadline monitoring has minimal overhead:
- **Enabled**: ~1-2% CPU overhead for timing measurement
- **Disabled**: ~0.1% overhead (tracking only, no logging)

Measurements use high-resolution timers (`Instant::now()`) with nanosecond precision.

## Limitations

1. **Soft guarantees**: Violations are logged but don't stop execution
2. **Python GIL**: Python's Global Interpreter Lock can cause jitter
3. **Scheduling**: OS scheduling affects actual execution timing
4. **Range**: Deadlines limited to 0-10000ms (10 seconds)
5. **Per-node only**: Cannot set deadlines for groups of nodes

## Troubleshooting

### High Miss Rates

**Problem**: Node consistently misses deadlines

**Solutions:**
1. Profile the tick() method to find bottlenecks
2. Offload work to separate threads/processes
3. Reduce node execution rate
4. Increase deadline to realistic value

### Inconsistent Performance

**Problem**: Miss rate varies widely

**Solutions:**
1. Check system load (CPU, I/O)
2. Look for periodic background tasks
3. Increase process priority
4. Pin to dedicated CPU cores (advanced)

### No Violations Logged

**Problem**: Expecting violations but none appear

**Check:**
1. `config.deadline_monitoring = True` is set
2. Deadline is actually set: `scheduler.set_node_deadline(...)`
3. Node is actually executing: Check `total_ticks > 0`

## Integration with Other Features

### With Node Introspection

```python
# Get comprehensive node information including deadlines
nodes = scheduler.get_all_nodes()
for node in nodes:
    print(f"{node['name']}:")
    print(f"  Priority: {node['priority']}")
    print(f"  Rate: {node['rate_hz']} Hz")
    print(f"  Deadline: {node.get('deadline_ms', 'N/A')}ms")
    print(f"  Misses: {node.get('deadline_misses', 0)}")
```

### With Fault Tolerance

```python
# Combine deadline monitoring with circuit breaker
config = horus.SchedulerConfig.standard()
config.deadline_monitoring = True
config.circuit_breaker = True
config.max_failures = 3

# Nodes that consistently miss deadlines won't trigger circuit breaker
# (deadline violations are warnings, not failures)
```

### With Configuration Presets

```python
# Use RobotPreset with deadline monitoring
config = horus.SchedulerConfig.from_preset(horus.RobotPreset.HIGH_PERFORMANCE)
config.deadline_monitoring = True  # Override specific setting
scheduler = horus.Scheduler.from_config(config)
```

## Future Enhancements

Planned improvements:
- Deadline violation callbacks
- Automatic rate adjustment
- Deadline groups (set for multiple nodes)
- Historical miss rate tracking
- Integration with profiling tools
- Export to monitoring systems

## Related Features

- **Node Introspection**: See `NODE_INTROSPECTION.md`
- **Fault Tolerance**: See `FAULT_TOLERANCE.md`
- **Configuration System**: See `horus_py/src/config.rs`
