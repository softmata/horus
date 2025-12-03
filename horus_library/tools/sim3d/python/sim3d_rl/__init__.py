"""
Sim3D RL - Python bindings for 3D robotics simulation

Provides Gymnasium-compatible RL environments for robot learning tasks.
"""

__version__ = "0.1.6"

# Import Rust module
from .sim3d_rl import (
    Sim3DEnv,
    VecSim3DEnv,
    make_env,
    make_vec_env,
)

__all__ = [
    "Sim3DEnv",
    "VecSim3DEnv",
    "make_env",
    "make_vec_env",
]
