/// Action-based pick-and-place manipulation.
///
/// Demonstrates the horus Action system — the replacement for ROS2's
/// actionlib. Actions are for long-running tasks with feedback:
///   send goal → receive progress feedback → get result (or cancel)
///
/// Nodes:
///   - PickPlaceServer: ActionServer that executes pick-and-place sequences
///   - TaskCommander: Sends pick-and-place goals and monitors progress
///
/// Actions:
///   - PickPlace: goal(object_id, place_x, place_y) → feedback(phase, progress) → result(success)
///
/// ROS2 equivalent: MoveIt pick-and-place demo, actionlib tutorials

use horus::prelude::*;
use horus::DurationExt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

// ============================================================================
// Action Definition — the pick-and-place task
// ============================================================================

action! {
    /// Pick an object and place it at a target location.
    PickPlace {
        goal {
            /// Object ID to pick
            object_id: u32,
            /// Target placement X coordinate
            place_x: f64,
            /// Target placement Y coordinate
            place_y: f64,
        }
        feedback {
            /// Current phase: 0=approaching, 1=grasping, 2=lifting, 3=moving, 4=placing
            phase: u32,
            /// Progress within current phase (0.0 to 1.0)
            progress: f32,
            /// Human-readable status
            gripper_force: f32,
        }
        result {
            /// Whether the pick-and-place succeeded
            success: u8,
            /// Final X position of the object
            final_x: f64,
            /// Final Y position of the object
            final_y: f64,
        }
    }
}

// ============================================================================
// Pick-and-Place Server — executes the manipulation sequence
// ============================================================================

/// Simulates a 5-phase pick-and-place sequence:
///   Phase 0: Approach object (move arm to pre-grasp pose)
///   Phase 1: Grasp (close gripper)
///   Phase 2: Lift (raise arm with object)
///   Phase 3: Move to target (move arm to place position)
///   Phase 4: Place (open gripper, retract)
fn execute_pick_place(handle: ServerGoalHandle<PickPlace>) -> GoalOutcome<PickPlace> {
    let goal = handle.goal();
    let phase_names = ["approaching", "grasping", "lifting", "moving", "placing"];

    hlog!(info, "PickPlace server: starting goal for object {} → ({:.1}, {:.1})",
        goal.object_id, goal.place_x, goal.place_y);

    for phase in 0..5u32 {
        // Check for cancellation before each phase
        if handle.is_cancel_requested() {
            hlog!(info, "PickPlace server: cancelled during phase '{}'", phase_names[phase as usize]);
            return handle.cancel(PickPlaceResult {
                success: 0,
                final_x: 0.0,
                final_y: 0.0,
            });
        }

        // Simulate phase execution with progress updates
        let steps = 10;
        for step in 0..steps {
            // Simulate work
            std::thread::sleep(Duration::from_millis(50));

            // Publish feedback
            let gripper_force = if phase == 1 || phase == 2 || phase == 3 {
                5.0 + (step as f32) * 0.5 // Gripping force during hold phases
            } else {
                0.0
            };

            handle.publish_feedback(PickPlaceFeedback {
                phase,
                progress: (step + 1) as f32 / steps as f32,
                gripper_force,
            });
        }

        hlog!(info, "PickPlace server: completed phase '{}' ({}/5)",
            phase_names[phase as usize], phase + 1);
    }

    // Success!
    hlog!(info, "PickPlace server: object {} placed at ({:.1}, {:.1})",
        goal.object_id, goal.place_x, goal.place_y);

    handle.succeed(PickPlaceResult {
        success: 1,
        final_x: goal.place_x,
        final_y: goal.place_y,
    })
}

// ============================================================================
// Task Commander — sends goals and monitors progress
// ============================================================================

fn run_task_commander() {
    // Wait for server to start
    std::thread::sleep(Duration::from_millis(200));

    let client = SyncActionClient::<PickPlace>::new()
        .expect("Failed to create PickPlace client");

    // === Task 1: Pick object 1 and place at (2.0, 1.0) ===
    hlog!(info, "Commander: sending pick-and-place goal for object 1");

    let feedback_count = Arc::new(AtomicU64::new(0));
    let fb_count = feedback_count.clone();

    let result = client.send_goal_and_wait_with_feedback(
        PickPlaceGoal {
            object_id: 1,
            place_x: 2.0,
            place_y: 1.0,
        },
        Duration::from_secs(30),
        move |fb| {
            fb_count.fetch_add(1, Ordering::Relaxed);
            let phase_names = ["approaching", "grasping", "lifting", "moving", "placing"];
            let phase_name = phase_names.get(fb.phase as usize).unwrap_or(&"unknown");
            if fb.progress >= 1.0 {
                hlog!(info, "Commander: phase '{}' complete (force={:.1}N)",
                    phase_name, fb.gripper_force);
            }
        },
    );

    match result {
        Ok(r) => {
            let fb_total = feedback_count.load(Ordering::Relaxed);
            if r.success == 1 {
                hlog!(info, "Commander: SUCCESS! Object placed at ({:.1}, {:.1}). {} feedback messages received.",
                    r.final_x, r.final_y, fb_total);
            } else {
                hlog!(info, "Commander: FAILED. {} feedback messages received.", fb_total);
            }
        }
        Err(e) => hlog!(error, "Commander: goal failed: {:?}", e),
    }

    // === Task 2: Pick object 2 and place at (3.0, 2.0) ===
    hlog!(info, "Commander: sending second goal for object 2");

    let result = client.send_goal_and_wait(
        PickPlaceGoal {
            object_id: 2,
            place_x: 3.0,
            place_y: 2.0,
        },
        Duration::from_secs(30),
    );

    match result {
        Ok(r) if r.success == 1 => hlog!(info, "Commander: object 2 placed at ({:.1}, {:.1})", r.final_x, r.final_y),
        Ok(_) => hlog!(info, "Commander: object 2 placement failed"),
        Err(e) => hlog!(error, "Commander: goal 2 failed: {:?}", e),
    }

    hlog!(info, "Commander: all tasks complete");
}

// ============================================================================
// Main
// ============================================================================

fn main() -> Result<()> {
    hlog!(info, "Pick-and-Place Example");
    hlog!(info, "  Demonstrates: Action goal lifecycle, feedback streaming, cancellation");
    hlog!(info, "  Server phases: approach → grasp → lift → move → place");
    hlog!(info, "");

    // Build the action server node
    let server = ActionServerBuilder::<PickPlace>::new()
        .on_goal(|goal| {
            hlog!(info, "Server: received goal for object {}", goal.object_id);
            GoalResponse::Accept
        })
        .on_cancel(|_id| {
            hlog!(info, "Server: cancel request accepted");
            CancelResponse::Accept
        })
        .on_execute(execute_pick_place)
        .build();

    // Run server in scheduler on background thread
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();

    let server_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(server).order(0).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    // Run commander on main thread
    run_task_commander();

    // Shutdown
    running.store(false, Ordering::Relaxed);
    server_thread.join().unwrap();

    Ok(())
}
