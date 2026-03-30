/// QA Test: Action server — pick_object action with feedback.
///
/// Terminal 1: cargo run --no-default-features --example qa_action_server
/// Terminal 2: horus action list, horus action send-goal pick_object '{"object_id":"box1"}'
use horus::prelude::*;

action! {
    /// Pick up an object from the workspace
    PickObject {
        goal {
            object_id: String,
        }
        feedback {
            progress: f32,
            status: String,
        }
        result {
            success: bool,
            message: String,
        }
    }
}

struct KeepAlive;
impl Node for KeepAlive {
    fn name(&self) -> &str {
        "keep_alive"
    }
    fn tick(&mut self) {}
}

fn main() -> Result<()> {
    println!("=== QA Action Server (Ctrl+C to stop) ===");

    let server = ActionServerNode::<PickObject>::builder()
        .on_goal(|goal| {
            println!("  [action] Goal received: pick '{}'", goal.object_id);
            GoalResponse::Accept
        })
        .on_execute(|handle| {
            let object_id = handle.goal().object_id.clone();

            // Simulate picking in steps
            for step in 0..5 {
                std::thread::sleep(std::time::Duration::from_millis(500));
                let progress = (step + 1) as f32 / 5.0;
                let status = format!("step {}/5", step + 1);
                println!("  [action] feedback: {:.0}% — {}", progress * 100.0, status);
                handle.publish_feedback(PickObjectFeedback { progress, status });

                if handle.is_cancel_requested() {
                    return GoalOutcome::Canceled(PickObjectResult {
                        success: false,
                        message: "Cancelled".to_string(),
                    });
                }
            }

            GoalOutcome::Succeeded(PickObjectResult {
                success: true,
                message: format!("Picked '{}'", object_id),
            })
        })
        .build();

    println!("Action 'pick_object' ready");

    let mut scheduler = Scheduler::new().name("qa_action").tick_rate(50_u64.hz());

    scheduler.add(server).rate(50_u64.hz()).order(0).build()?;
    scheduler.add(KeepAlive).order(99).build()?;

    scheduler.run()?;
    Ok(())
}
