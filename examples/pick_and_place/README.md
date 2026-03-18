# Pick and Place — Action-Based Manipulation

Demonstrates horus Actions — the replacement for ROS2 actionlib. Actions are for long-running tasks with progress feedback: send a goal, receive feedback during execution, get a result (or cancel mid-execution).

## Architecture

```
TaskCommander                    PickPlaceServer
     │                                │
     ├─── send_goal(obj=1, x=2, y=1) ──►│
     │                                │── Phase 0: approach
     │◄── feedback(phase=0, prog=0.5) ──│
     │◄── feedback(phase=0, prog=1.0) ──│
     │                                │── Phase 1: grasp
     │◄── feedback(phase=1, prog=1.0) ──│
     │                                │── Phase 2: lift
     │                                │── Phase 3: move
     │                                │── Phase 4: place
     │◄── result(success=true)  ────────│
```

## What This Demonstrates

- **`action!` macro**: Define goal/feedback/result types in one block
- **`ActionServerBuilder`**: Build a server with `on_goal`, `on_cancel`, `on_execute` callbacks
- **`SyncActionClient`**: Send goals and block for results
- **Feedback streaming**: Server publishes progress during execution
- **Cancellation**: Server checks `is_cancel_requested()` between phases
- **5-phase manipulation**: approach → grasp → lift → move → place

## ROS2 Equivalent

| ROS2 | Horus |
|------|-------|
| `action_msgs/GoalStatus` | `GoalStatus` enum |
| `ActionServer` (rclpy/rclcpp) | `ActionServerBuilder::new().on_execute(...)` |
| `ActionClient.send_goal_async()` | `SyncActionClient::send_goal_and_wait()` |
| `goal_handle.publish_feedback()` | `handle.publish_feedback(...)` |
| `goal_handle.succeed()` | `handle.succeed(result)` |
| `goal_handle.abort()` | `handle.abort(result)` |
| `goal_handle.canceled()` | `handle.cancel(result)` |

## Key Patterns

### Define an Action
```rust
action! {
    PickPlace {
        goal { object_id: u32, place_x: f64, place_y: f64 }
        feedback { phase: u32, progress: f32, gripper_force: f32 }
        result { success: u8, final_x: f64, final_y: f64 }
    }
}
```

### Server with Cancellation Support
```rust
fn execute(handle: ServerGoalHandle<PickPlace>) -> GoalOutcome<PickPlace> {
    for phase in 0..5 {
        if handle.is_cancel_requested() {
            return handle.cancel(result);
        }
        handle.publish_feedback(feedback);
        // ... do work ...
    }
    handle.succeed(result)
}
```

### Client with Feedback
```rust
let result = client.send_goal_and_wait_with_feedback(
    goal,
    timeout,
    |feedback| println!("Progress: {}%", feedback.progress * 100.0),
);
```

## Run

```bash
horus run main.rs
```
