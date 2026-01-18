# HORUS Robotics - Python Bindings

[![PyPI](https://img.shields.io/pypi/v/horus-robotics)](https://pypi.org/project/horus-robotics/)
[![Python](https://img.shields.io/pypi/pyversions/horus-robotics)](https://pypi.org/project/horus-robotics/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](https://github.com/softmata/horus/blob/main/LICENSE)

**Ultra-low latency robotics framework with Python bindings** - 100x faster than ROS2.

HORUS is a production-grade robotics framework built in Rust, delivering **87ns IPC latency** and **12M+ messages/second** throughput. This package provides native Python bindings via PyO3.

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

- **Ultra-Low Latency**: 87ns IPC latency (100x faster than ROS2)
- **High Throughput**: 12M+ messages/second
- **Zero-Copy**: Shared memory transport for maximum performance
- **Type-Safe**: Strong typing with standard robotics messages
- **Cross-Language**: Seamless interop with Rust nodes
- **Real-Time Ready**: Priority-based scheduling with deadline guarantees

## Core Components

### Topic (Unified Communication)

The `Topic` class provides pub/sub communication with automatic backend selection:

```python
import horus

# Create a topic
topic = horus.Topic("robot_commands")

# Send messages
topic.send({"velocity": [1.0, 0.0, 0.0]})

# Receive messages
msg = topic.recv()
```

### Typed Messages

Use typed messages for zero-copy performance:

```python
from horus import Topic, CmdVel, Pose2D

# Create typed topic
topic = Topic("cmd_vel", CmdVel)

# Send typed message
topic.send(CmdVel(linear=1.5, angular=0.3))

# Receive typed message
msg = topic.recv()  # Returns CmdVel instance
print(f"Linear: {msg.linear}, Angular: {msg.angular}")
```

### Nodes

Create custom processing nodes:

```python
import horus

class SensorNode(horus.Node):
    def __init__(self):
        super().__init__("sensor_node")
        self.topic = horus.Topic("sensor_data")

    def tick(self):
        reading = self.read_sensor()
        self.topic.send(reading)

# Run the node
node = SensorNode()
horus.run(node, rate_hz=100)
```

### Async Nodes

For I/O-bound operations, use async nodes:

```python
import horus

class AsyncSensorNode(horus.AsyncNode):
    async def setup(self):
        self.topic = horus.AsyncTopic("sensor_data")

    async def tick(self):
        data = await self.fetch_data()
        await self.topic.send(data)

    async def fetch_data(self):
        await horus.sleep(0.01)
        return {"value": 42}
```

## Message Types

HORUS provides standard robotics message types:

```python
from horus import (
    CmdVel,      # Velocity commands (linear, angular)
    Pose2D,      # 2D position (x, y, theta)
    Imu,         # IMU readings
    Odometry,    # Odometry data
    LaserScan,   # LIDAR data
)
```

## Performance Comparison

| Framework | IPC Latency | Throughput |
|-----------|-------------|------------|
| **HORUS** | **87ns** | **12M+ msg/s** |
| ROS2 (FastDDS) | 50-100us | ~100K msg/s |
| ZeroMQ | 10-30us | ~1M msg/s |

## Documentation

- **Full Documentation**: [docs.horus-registry.dev](https://docs.horus-registry.dev)
- **Getting Started**: [Installation Guide](https://docs.horus-registry.dev/installation)
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
