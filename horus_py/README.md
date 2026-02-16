# HORUS Robotics - Python Bindings

[![PyPI](https://img.shields.io/pypi/v/horus-robotics)](https://pypi.org/project/horus-robotics/)
[![Python](https://img.shields.io/pypi/pyversions/horus-robotics)](https://pypi.org/project/horus-robotics/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](https://github.com/softmata/horus/blob/main/LICENSE)

Python bindings for the HORUS robotics framework. Sub-microsecond messaging with a simple Python API.

## Installation

```bash
pip install horus-robotics
```

## Quick Start

```python
import horus

# Create a topic for communication
topic = horus.Topic("sensor_data")

# Send and receive data
topic.send({"temperature": 25.5, "humidity": 60})
data = topic.recv()
```

## Nodes

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

## Typed Messages

```python
from horus import Topic, CmdVel, Pose2D

topic = Topic("cmd_vel", CmdVel)
topic.send(CmdVel(linear=1.5, angular=0.3))
msg = topic.recv()  # Returns CmdVel instance
```

## Scheduler Presets

```python
import horus

scheduler = horus.Scheduler()                    # Lightweight default
scheduler = horus.Scheduler.deploy()             # Production
scheduler = horus.Scheduler.safety_critical()    # Safety systems
scheduler = horus.Scheduler.deterministic()      # Reproducible execution
```

## Cross-Language

Rust and Python nodes communicate seamlessly through shared topics. Use Rust for real-time control and Python for AI/ML and high-level logic.

## Documentation

- [Full Docs](https://docs.horus-registry.dev)
- [Getting Started](https://docs.horus-registry.dev/getting-started/installation)
- [Python API](https://docs.horus-registry.dev/python-api)

## Requirements

- Python 3.9+
- Linux (x86_64)

## License

Apache-2.0 - see [LICENSE](https://github.com/softmata/horus/blob/main/LICENSE)

## Links

- [GitHub](https://github.com/softmata/horus)
- [Discord](https://discord.gg/hEZC3ev2Nf)
- [Documentation](https://docs.horus-registry.dev)
