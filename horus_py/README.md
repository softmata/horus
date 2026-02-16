# HORUS Robotics - Python Bindings

[![PyPI](https://img.shields.io/pypi/v/horus-robotics)](https://pypi.org/project/horus-robotics/)
[![Python](https://img.shields.io/pypi/pyversions/horus-robotics)](https://pypi.org/project/horus-robotics/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](https://github.com/softmata/horus/blob/main/LICENSE)

**Ultra-low latency robotics framework with Python bindings** - 100x faster than ROS2.

HORUS is a production-grade robotics framework built in Rust, delivering **3ns - 167ns IPC latency** across 10 auto-selected backends and **12M+ messages/second** throughput. This package provides native Python bindings via PyO3.

## Installation

```bash
pip install horus-robotics
```

## Quick Start

```python
import horus

# Create a topic for communication
topic = horus.Topic("sensor_data")

# Send data
topic.send({"temperature": 25.5, "humidity": 60})

# Receive data
data = topic.recv()
print(data)
```

## Features

- **Ultra-Low Latency**: 3ns - 167ns IPC (10 auto-selected backends)
- **High Throughput**: 12M+ messages/second
- **Zero-Copy**: Shared memory transport for maximum performance
- **Type-Safe**: Strong typing with standard robotics messages
- **Cross-Language**: Seamless interop with Rust nodes
- **Real-Time Ready**: Priority-based scheduling with opt-in RT features

## Core Components

### Topic (Unified Communication)

```python
import horus

topic = horus.Topic("robot_commands")
topic.send({"velocity": [1.0, 0.0, 0.0]})
msg = topic.recv()
```

### Typed Messages

```python
from horus import Topic, CmdVel, Pose2D

topic = Topic("cmd_vel", CmdVel)
topic.send(CmdVel(linear=1.5, angular=0.3))
msg = topic.recv()  # Returns CmdVel instance
```

### Nodes

```python
import horus

class SensorNode(horus.Node):
    def __init__(self):
        super().__init__("sensor_node")
        self.topic = horus.Topic("sensor_data")

    def tick(self):
        reading = self.read_sensor()
        self.topic.send(reading)

node = SensorNode()
horus.run(node, rate_hz=100)
```

### Scheduler with Presets

```python
import horus

# Lightweight scheduler (default)
scheduler = horus.Scheduler()

# Or use presets
scheduler = horus.Scheduler.deploy()            # Production: RT + BlackBox
scheduler = horus.Scheduler.safety_critical()    # WCET + watchdog
scheduler = horus.Scheduler.deterministic()      # Reproducible execution

# Builder pattern
scheduler = horus.Scheduler()
scheduler.realtime()         # Opt-in RT features
scheduler.tick_hz(1000.0)    # 1kHz tick rate
scheduler.with_blackbox(16)  # 16MB flight recorder
```

## Performance Comparison

| Framework | IPC Latency | Throughput |
|-----------|-------------|------------|
| **HORUS** | **3ns - 167ns** | **12M+ msg/s** |
| ROS2 (FastDDS) | 50-100us | ~100K msg/s |
| ZeroMQ | 10-30us | ~1M msg/s |

## Documentation

- **Full Documentation**: [docs.horus-registry.dev](https://docs.horus-registry.dev)
- **Getting Started**: [Installation Guide](https://docs.horus-registry.dev/getting-started/installation)
- **API Reference**: [Python API](https://docs.horus-registry.dev/python-api)

## Requirements

- Python 3.9+
- Linux (x86_64)
- glibc 2.39+

## License

Apache-2.0 - see [LICENSE](https://github.com/softmata/horus/blob/main/LICENSE)

## Links

- **GitHub**: [github.com/softmata/horus](https://github.com/softmata/horus)
- **Discord**: [Join Community](https://discord.gg/hEZC3ev2Nf)
- **Documentation**: [docs.horus-registry.dev](https://docs.horus-registry.dev)
