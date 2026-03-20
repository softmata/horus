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

# Create a node with callbacks
def my_tick(node):
    node.send("output", 42.0)
    msg = node.get("input")
    if msg:
        print(f"Got: {msg}")

node = horus.Node(
    name="my_node",
    pubs=["output"],
    subs=["input"],
    tick=my_tick,
    rate=30,
)

horus.run(node, duration=10)
```

## Typed Messages

```python
from horus import Topic, CmdVel, Pose2D

topic = Topic(CmdVel)                        # Typed topic (auto-named "cmd_vel")
topic.send(CmdVel(linear=1.5, angular=0.3))
msg = topic.recv()                            # Returns CmdVel instance
```

## Scheduler API

```python
import horus
from horus import Node, Scheduler

sched = Scheduler(tick_rate=1000)
sched.add(Node(tick=motor_fn, rate=1000, order=0))
sched.add(Node(tick=planner_fn, rate=100, order=5, compute=True))
sched.add(Node(tick=telemetry_fn, rate=1, order=10, async_io=True))
sched.run()
```

## Multiple Nodes

```python
import horus

def sensor_tick(node):
    node.send("sensor_data", 42.0)

def control_tick(node):
    data = node.get("sensor_data")
    if data:
        node.send("cmd_vel", data * 0.5)

sensor = horus.Node(name="sensor", pubs=["sensor_data"], tick=sensor_tick, rate=100)
controller = horus.Node(name="controller", subs=["sensor_data"], pubs=["cmd_vel"], tick=control_tick, rate=100)

horus.run(sensor, controller, duration=30)
```

## Cross-Language

Rust and Python nodes communicate seamlessly through shared topics. Use Rust for real-time control and Python for AI/ML and high-level logic.

## Documentation

- [Full Docs](https://docs.horusrobotics.dev)
- [Getting Started](https://docs.horusrobotics.dev/getting-started/installation)
- [Python API](https://docs.horusrobotics.dev/python-api)

## Requirements

- Python 3.9+
- Linux (x86_64)

## License

Apache-2.0 - see [LICENSE](https://github.com/softmata/horus/blob/main/LICENSE)

## Links

- [GitHub](https://github.com/softmata/horus)
- [Discord](https://discord.gg/hEZC3ev2Nf)
- [Documentation](https://docs.horusrobotics.dev)
