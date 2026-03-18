# Recording & Replay — Crash Forensics and Reproducibility

Horus's replacement for ROS2's rosbag2. Record everything during a run, analyze anomalies post-mortem, and replay deterministically to reproduce bugs.

## The Debugging Workflow

```
1. RECORD on robot    →  2. TRANSFER to laptop  →  3. REPLAY to reproduce
   horus run --record        scp recording.horus       horus record replay
```

## What This Demonstrates

- **`.with_recording()`**: Enable automatic session recording
- **`.blackbox(n)`**: Flight recorder for crash forensics (survives crashes)
- **`.deterministic(true)`**: SimClock + sequential execution for reproducible replay
- **Blackbox CLI**: `horus blackbox -a` for anomaly detection
- **Record CLI**: `horus record list/info/replay/export/diff`

## ROS2 Equivalent

| ROS2 | Horus |
|------|-------|
| `ros2 bag record -a` | `horus run --record` or `.with_recording()` |
| `ros2 bag play recording.db3` | `horus record replay <session>` |
| `ros2 bag info recording.db3` | `horus record info <session>` |
| Manual crash debugging | `horus blackbox -a` (automatic anomaly detection) |
| No deterministic replay | `.deterministic(true)` guarantees identical outputs |

## Key Advantage Over rosbag2

rosbag2 records topic messages. Horus records **node execution** — which node ticked, how long it took, what it read from topics, what it wrote. This means you can:

1. **Detect budget violations**: Which node exceeded its timing budget?
2. **Trace data flow**: What input caused this specific output?
3. **Replay exactly**: Same SimClock time, same execution order, same RNG seeds

## Run

```bash
# Normal run with recording
horus run --record main.rs

# Or just run the example (recording enabled in code)
horus run main.rs

# After the run, analyze
horus blackbox -a                    # Show anomalies
horus record list                    # List recorded sessions
horus record info <session>          # Session details
horus record replay <session>        # Replay
horus record diff <session1> <session2>  # Compare two runs
```
