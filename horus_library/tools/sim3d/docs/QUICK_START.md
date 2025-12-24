# sim3d Quick Start Guide

Get up and running with sim3d in minutes!

## Installation

### 1. Build sim3d

```bash
cd horus_library/tools/sim3d

# Visual mode (with rendering)
cargo build --release

# Headless mode (for RL training, no rendering)
cargo build --release --no-default-features --features headless
```

### 2. (Optional) Build Python bindings

For RL training with Python:

```bash
# Install maturin via Cargo (recommended for Ubuntu 24.04+)
cargo install maturin
pip install numpy
maturin develop --release --features python
```

Install RL dependencies:

```bash
pip install stable-baselines3 gymnasium torch
```

---

## Running sim3d

### Visual Mode (Interactive)

Launch with default scene:

```bash
./target/release/sim3d --mode visual
```

With a robot:

```bash
./target/release/sim3d --mode visual --robot assets/models/turtlebot3/turtlebot3.urdf
```

**Controls:**
- **Mouse**: Orbit camera (left drag)
- **Scroll**: Zoom in/out
- **WASD**: Pan camera
- **F1**: Toggle debug panel
- **ESC**: Exit

---

### Headless Mode (RL Training)

For maximum performance (no rendering):

```bash
./target/release/sim3d --mode headless
```

---

## Python RL Training

### Test Environment

```bash
python python/examples/train_ppo.py --task navigation --test-only
```

Expected output:
```
Testing navigation environment...
Episode 1/5
Initial observation shape: (20,)
  Steps: 342
  Total reward: -45.23
  Success: False
  Termination: MaxSteps
...
```

### Train an Agent

Train PPO on navigation task:

```bash
python python/examples/train_ppo.py \
    --task navigation \
    --timesteps 1000000 \
    --n-envs 4
```

**Training will:**
- Create 4 parallel environments
- Train for 1M total steps (~250K per env)
- Save checkpoints every 50K steps
- Evaluate every 10K steps
- Log to TensorBoard

**Monitor training:**
```bash
tensorboard --logdir ./logs/ppo_navigation/tensorboard/
```

Visit: http://localhost:6006

---

### Benchmark Performance

Test environment speed:

```bash
python python/examples/benchmark_env.py --task navigation --steps 10000
```

Expected (headless mode):
- Single env: **~2,500 steps/sec**
- 16 parallel: **~125,000 steps/sec**

---

## Available RL Tasks

| Task | Description | Obs Dim | Action Dim | Difficulty |
|------|-------------|---------|------------|------------|
| **reaching** | Robot arm reaches target position | 10 | 6 | Easy |
| **balancing** | Balance inverted pendulum | 8 | 1 | Medium |
| **locomotion** | Bipedal walking | 12 | 2 | Hard |
| **navigation** | Navigate to goal, avoid obstacles | 20 | 2 | Medium |
| **manipulation** | Pick and place objects | 15 | 7 | Hard |
| **push** | Push object to target | 18 | 2 | Medium |

---

## Quick Examples

### 1. Train a Reaching Agent (5 minutes)

```bash
python python/examples/train_ppo.py \
    --task reaching \
    --timesteps 100000 \
    --n-envs 8
```

This task trains quickly and demonstrates the full pipeline.

### 2. Evaluate Trained Model

```python
from stable_baselines3 import PPO
import sim3d_rl

# Load model
model = PPO.load("./logs/ppo_reaching/best/best_model")

# Create environment
env = sim3d_rl.make_env("reaching", obs_dim=10, action_dim=6)

# Run evaluation
obs = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    if done or truncated:
        print(f"Success: {info['success']}, Reward: {info['total_reward']}")
        obs = env.reset()
```

### 3. Custom Training Loop

```python
import sim3d_rl
import numpy as np

env = sim3d_rl.make_env("navigation", obs_dim=20, action_dim=2)

for episode in range(100):
    obs = env.reset()
    total_reward = 0
    
    for step in range(1000):
        # Your policy here (e.g., random)
        action = np.random.randn(2).astype(np.float32)
        
        obs, reward, done, truncated, info = env.step(action)
        total_reward += reward
        
        if done or truncated:
            print(f"Episode {episode}: {total_reward:.1f}")
            break
```

---

## Troubleshooting

### Python import error: "No module named 'sim3d_rl'"

**Solution:** Build Python bindings:
```bash
cd horus_library/tools/sim3d
maturin develop --release --features python
```

### Slow performance

**Solutions:**
1. Use headless mode: `--mode headless`
2. Build in release: `cargo build --release`
3. Use vectorized environments (increase `--n-envs`)

### Compilation errors

**Common fixes:**
```bash
# Update dependencies
cargo update

# Clean build
cargo clean
cargo build --release
```

---

## Next Steps

1. **Try all tasks:** Test each RL task to see which fits your needs
2. **Tune hyperparameters:** Adjust PPO settings for better performance
3. **Add custom robots:** Load your own URDF files
4. **Create custom tasks:** Implement new RL tasks in Rust

---

## Resources

- **Examples:** `python/examples/`
- **Documentation:** `docs/`
- **RL Tasks:** `src/rl/tasks/*.rs`
- **Sensors:** `src/sensors/*.rs`

## Getting Help

- Check `docs/PROGRESS_REPORT.md` for implementation status
- Read Python examples for working code
- Examine RL task source code for implementation details

---

**Happy training!** 🚀
