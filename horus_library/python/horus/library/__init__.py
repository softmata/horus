"""
HORUS Library - Standard robotics messages, nodes, and algorithms

The official standard library for the HORUS robotics framework (Python bindings).

## Structure

```
horus_library/
├── messages/      # Shared memory-safe messages
├── nodes/         # Reusable nodes (future)
├── algorithms/    # Common algorithms (future)
├── sim2d/         # 2D robotics simulator
└── sim3d/         # 3D robotics simulator with RL support
```

## Usage

```python
# Option 1: Import from submodules (organized)
from horus.library.messages import Pose2D, CmdVel, LaserScan
from horus.library.sim2d import Sim2D
from horus.library.sim3d import make_env, Sim3DEnv

# Option 2: Import from root (convenient)
from horus.library import Pose2D, CmdVel, LaserScan

# Option 3: Via main horus package (recommended)
from horus import Pose2D, CmdVel, LaserScan
```

## Simulation Examples

```python
# 2D Simulation
from horus.library.sim2d import Sim2D
sim = Sim2D(robot_name="robot", headless=False)
sim.add_obstacle(pos=(5.0, 5.0), size=(1.0, 1.0))
sim.run(duration=10.0)

# 3D RL Environment (requires sim3d feature)
from horus.library.sim3d import make_env
env = make_env("locomotion")
obs = env.reset()
obs, reward, done, truncated, info = env.step([0.1] * 12)
```

## Message Examples

```python
>>> from horus import Pose2D, Twist, CmdVel, LaserScan
>>> pose = Pose2D(x=1.0, y=2.0, theta=0.5)
>>> cmd = Twist.new_2d(linear_x=0.5, angular_z=0.1)
>>> vel = CmdVel(linear=1.0, angular=0.5)
>>> scan = LaserScan()
>>> imu = Imu()
>>> battery = BatteryState(voltage=12.6, percentage=85.0)
>>> gps = NavSatFix(latitude=37.7749, longitude=-122.4194, altitude=10.0)
```
"""

__version__ = "0.1.6"

# Submodules
from . import messages
from . import nodes
from . import algorithms
from . import sim2d
from . import sim3d

# Re-export all messages at the root for convenience (matches Rust API)
from .messages import *

__all__ = [
    # Submodules
    "messages",
    "nodes",
    "algorithms",
    "sim2d",
    "sim3d",
]

# Add all message types to __all__
__all__.extend(messages.__all__)

# Add sim2d types to __all__ if available
if hasattr(sim2d, '__all__'):
    __all__.extend(sim2d.__all__)

# Add sim3d types to __all__ if available
if hasattr(sim3d, '__all__'):
    __all__.extend(sim3d.__all__)
