# sim3d Python RL Training Examples

This directory contains Python examples for training RL agents using sim3d.

## Installation

### 1. Build sim3d Python bindings

```bash
# Install maturin (if not already installed)
cargo install maturin

# Build Python bindings
cd horus_library/tools/sim3d
maturin develop --release --features python
```

### 2. Install dependencies

```bash
pip install stable-baselines3 gymnasium numpy torch
```

## Examples

### Basic Environment Test

Test that the environment works correctly:

```bash
python examples/train_ppo.py --task navigation --test-only
```

### Train PPO Agent

Train a PPO agent on the navigation task:

```bash
python examples/train_ppo.py --task navigation --timesteps 1000000 --n-envs 4
```

Available tasks:
- `reaching` - Reach a target position with robot arm
- `balancing` - Balance an inverted pendulum
- `locomotion` - Learn to walk forward
- `navigation` - Navigate to goal while avoiding obstacles
- `manipulation` - Pick and place objects
- `push` - Push object to target location

### Benchmark Performance

Test environment throughput:

```bash
python examples/benchmark_env.py --task navigation --steps 10000
```

## Expected Performance

**Target performance (headless mode):**
- Single environment: ~2,500 steps/sec
- 16 parallel envs: ~125,000 steps/sec  
- Reset time: <1 ms

## Monitoring Training

View training progress with TensorBoard:

```bash
tensorboard --logdir ./logs/ppo_navigation/tensorboard/
```

## Trained Models

Models are saved to:
- Best model: `./logs/ppo_{task}/best/best_model.zip`
- Final model: `./logs/ppo_{task}/final_model.zip`
- Checkpoints: `./logs/ppo_{task}/checkpoints/`

## Loading and Evaluating

```python
from stable_baselines3 import PPO
import sim3d_rl

# Load trained model
model = PPO.load("./logs/ppo_navigation/best/best_model")

# Create environment
env = sim3d_rl.make_env("navigation", obs_dim=20, action_dim=2)

# Evaluate
obs = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    if done or truncated:
        print(f"Episode finished! Success: {info['success']}")
        obs = env.reset()
```

## Troubleshooting

### "No module named 'sim3d_rl'"

Make sure you built the Python bindings:
```bash
cd horus_library/tools/sim3d
maturin develop --release --features python
```

### Slow performance

1. Make sure you built in release mode (`--release`)
2. Check that headless mode is being used (no window should appear)
3. Run benchmarks to verify performance

### Import errors

Ensure all dependencies are installed:
```bash
pip install stable-baselines3 gymnasium numpy torch
```
