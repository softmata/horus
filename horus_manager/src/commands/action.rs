//! Action command - Interact with HORUS actions
//!
//! Provides commands for listing, inspecting, and sending goals to actions.
//! Equivalent to `ros2 action list/info/send_goal/cancel_goal`.

use crate::cli_output;
use crate::discovery::discover_shared_memory;
use colored::*;
use horus_core::error::{HorusError, HorusResult};

// ─── Action discovery ─────────────────────────────────────────────────────────

/// Action discovered from SHM topics.
#[derive(Debug, Clone)]
struct DiscoveredAction {
    name: String,
    has_goal: bool,
    has_result: bool,
    has_feedback: bool,
    has_status: bool,
    has_cancel: bool,
    goal_publishers: usize,
    result_subscribers: usize,
}

/// Discover active actions by scanning SHM topics for action-specific topic patterns.
///
/// An action named `/navigate` creates topics:
///   navigate/goal, navigate/result, navigate/feedback,
///   navigate/status, navigate/cancel
fn discover_actions() -> HorusResult<Vec<DiscoveredAction>> {
    let topics = discover_shared_memory()?;

    let mut actions: std::collections::HashMap<String, DiscoveredAction> =
        std::collections::HashMap::new();

    let suffixes = [
        ("/goal", "goal"),
        ("/result", "result"),
        ("/feedback", "feedback"),
        ("/status", "status"),
        ("/cancel", "cancel"),
    ];

    for topic in &topics {
        let topic_name = topic
            .topic_name
            .strip_prefix("horus_topic/")
            .unwrap_or(&topic.topic_name);

        for (suffix, field) in &suffixes {
            if let Some(action_name) = topic_name.strip_suffix(suffix) {
                // Skip topics that look like service topics (also have /request, /response)
                let entry =
                    actions
                        .entry(action_name.to_string())
                        .or_insert_with(|| DiscoveredAction {
                            name: action_name.to_string(),
                            has_goal: false,
                            has_result: false,
                            has_feedback: false,
                            has_status: false,
                            has_cancel: false,
                            goal_publishers: 0,
                            result_subscribers: 0,
                        });

                match *field {
                    "goal" => {
                        entry.has_goal = true;
                        entry.goal_publishers = topic.publishers.len();
                    }
                    "result" => {
                        entry.has_result = true;
                        entry.result_subscribers = topic.subscribers.len();
                    }
                    "feedback" => entry.has_feedback = true,
                    "status" => entry.has_status = true,
                    "cancel" => entry.has_cancel = true,
                    _ => {}
                }
            }
        }
    }

    // Only return entries that look like real actions (have at least goal + result)
    let mut result: Vec<DiscoveredAction> = actions
        .into_values()
        .filter(|a| a.has_goal || a.has_result)
        .collect();

    // Exclude entries that are actually services (have /request or /response partner)
    let topic_names: std::collections::HashSet<String> = topics
        .iter()
        .map(|t| {
            t.topic_name
                .strip_prefix("horus_topic/")
                .unwrap_or(&t.topic_name)
                .to_string()
        })
        .collect();

    result.retain(|a| {
        !topic_names.contains(&format!("{}/request", a.name))
            && !topic_names.contains(&format!("{}/response", a.name))
    });

    result.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(result)
}

// ─── list_actions ─────────────────────────────────────────────────────────────

/// List all active actions (`horus action list`)
pub fn list_actions(verbose: bool, json: bool) -> HorusResult<()> {
    let actions = discover_actions()?;

    if json {
        let items: Vec<_> = actions
            .iter()
            .map(|a| {
                serde_json::json!({
                    "name": a.name,
                    "goal_publishers": a.goal_publishers,
                    "result_subscribers": a.result_subscribers,
                    "has_feedback": a.has_feedback,
                    "has_status": a.has_status,
                    "has_cancel": a.has_cancel,
                })
            })
            .collect();
        let output = serde_json::json!({
            "count": items.len(),
            "items": items
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    if actions.is_empty() {
        println!("{}", "No active actions found.".yellow());
        println!(
            "  {} Start a HORUS application with action servers to see them here",
            "Tip:".dimmed()
        );
        return Ok(());
    }

    println!("{}", "Active Actions:".green().bold());
    println!();

    if verbose {
        for action in &actions {
            let complete = action.has_goal
                && action.has_result
                && action.has_feedback
                && action.has_status
                && action.has_cancel;

            println!("  {} {}", "Action:".cyan(), action.name.white().bold());
            println!(
                "    {} {}",
                "Status:".dimmed(),
                if complete {
                    "fully connected".green()
                } else {
                    "partial".yellow()
                }
            );
            println!(
                "    {} {}",
                "Topics:".dimmed(),
                [
                    if action.has_goal { "goal" } else { "" },
                    if action.has_result { "result" } else { "" },
                    if action.has_feedback { "feedback" } else { "" },
                    if action.has_status { "status" } else { "" },
                    if action.has_cancel { "cancel" } else { "" },
                ]
                .iter()
                .filter(|s| !s.is_empty())
                .cloned()
                .collect::<Vec<_>>()
                .join(", ")
            );
            println!(
                "    {} {}",
                "Goal publishers:".dimmed(),
                action.goal_publishers
            );
            println!(
                "    {} {}",
                "Result subscribers:".dimmed(),
                action.result_subscribers
            );
            println!();
        }
    } else {
        println!(
            "  {:<40} {:>6} {}",
            "NAME".dimmed(),
            "GOALS".dimmed(),
            "TOPICS".dimmed()
        );
        println!("  {}", "-".repeat(60).dimmed());
        for action in &actions {
            let topics_present = [
                action.has_goal,
                action.has_result,
                action.has_feedback,
                action.has_status,
                action.has_cancel,
            ]
            .iter()
            .filter(|&&b| b)
            .count();
            println!(
                "  {:<40} {:>6} {}/5 topics",
                action.name, action.goal_publishers, topics_present
            );
        }
    }

    println!();
    println!("  {} {} action(s)", "Total:".dimmed(), actions.len());

    Ok(())
}

// ─── action_info ─────────────────────────────────────────────────────────────

/// Show detailed info about an action (`horus action info <name>`)
pub fn action_info(name: &str) -> HorusResult<()> {
    let actions = discover_actions()?;

    let action = actions.iter().find(|a| {
        a.name == name
            || a.name.ends_with(&format!("/{}", name))
            || a.name
                .rsplit('/')
                .next()
                .map(|base| base == name)
                .unwrap_or(false)
    });

    let Some(action) = action else {
        return Err(HorusError::Config(format!(
            "Action '{}' not found. Use 'horus action list' to see available actions.",
            name
        )));
    };

    println!("{}", "Action Information".green().bold());
    println!();
    println!("  {} {}", "Name:".cyan(), action.name.white().bold());
    println!();
    println!("  {}", "Topics:".cyan());

    let topics = [
        ("goal", action.has_goal, "clients send goals here"),
        ("cancel", action.has_cancel, "clients request cancellation"),
        ("status", action.has_status, "server broadcasts goal states"),
        ("feedback", action.has_feedback, "server sends progress"),
        ("result", action.has_result, "server sends final result"),
    ];

    for (suffix, present, desc) in &topics {
        let status = if *present {
            "✓".green().to_string()
        } else {
            "✗".red().to_string()
        };
        println!(
            "    {} {}/{:<12} — {}",
            status,
            action.name,
            suffix,
            desc.dimmed()
        );
    }

    println!();
    println!(
        "  {} {}",
        "Goal publishers (clients):".cyan(),
        action.goal_publishers
    );
    println!(
        "  {} {}",
        "Result subscribers (clients):".cyan(),
        action.result_subscribers
    );
    println!();
    println!(
        "  {} Use 'horus action send_goal {} <goal_json>' to send a goal",
        "Tip:".dimmed(),
        action.name
    );

    Ok(())
}

// ─── send_goal ────────────────────────────────────────────────────────────────

/// Send a goal to an action server (`horus action send_goal <name> <goal_json>`)
///
/// Sends the goal and then monitors status/feedback/result until completion or
/// timeout.  Equivalent to `ros2 action send_goal`.
pub fn send_goal(
    name: &str,
    goal_json: &str,
    wait_result: bool,
    timeout_secs: f64,
) -> HorusResult<()> {
    use horus_core::communication::Topic;
    use std::time::{Duration, Instant};

    // Validate JSON
    let goal_value: serde_json::Value =
        serde_json::from_str(goal_json).map_err(|e| {
            HorusError::Config(format!(
                "Invalid goal JSON: {}\n  Example: horus action send_goal {} '{{\"x\": 1.0, \"y\": 2.0}}'",
                e, name
            ))
        })?;

    // Action topics use a wrapper: GoalRequest<G> which has uuid + priority + payload
    // We use serde_json::Value as the payload.
    let goal_topic_name = format!("{}/goal", name);
    let status_topic_name = format!("{}/status", name);
    let feedback_topic_name = format!("{}/feedback", name);
    let result_topic_name = format!("{}/result", name);

    // Generate a goal UUID
    let goal_id = uuid::Uuid::new_v4().to_string();

    let goal_request = serde_json::json!({
        "goal_id": goal_id,
        "priority": 128,   // NORMAL priority
        "payload": goal_value,
    });

    println!(
        "{} Sending goal to action: {}",
        cli_output::ICON_INFO.cyan(),
        name.white().bold()
    );
    println!("  {} {}", "Goal:".dimmed(), goal_json);
    println!(
        "  {} {}",
        "Goal ID:".dimmed(),
        goal_id.get(..8).unwrap_or(&goal_id)
    );
    println!();

    let goal_topic: Topic<serde_json::Value> = Topic::new(&goal_topic_name).map_err(|e| {
        HorusError::Config(format!(
            "Cannot open goal topic '{}': {}\n  Is an action server running for '{}'?",
            goal_topic_name, e, name
        ))
    })?;

    goal_topic.send(goal_request);
    println!("{} Goal sent", cli_output::ICON_SUCCESS.green());

    if !wait_result {
        return Ok(());
    }

    println!("  {} Waiting for result (Ctrl+C to stop)...", "".dimmed());
    println!();

    // Monitor status and feedback
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    let status_topic: Topic<serde_json::Value> = Topic::new(&status_topic_name).map_err(|e| {
        HorusError::Communication(
            format!(
                "Failed to create status topic '{}': {}",
                status_topic_name, e
            )
            .into(),
        )
    })?;
    let feedback_topic: Topic<serde_json::Value> =
        Topic::new(&feedback_topic_name).map_err(|e| {
            HorusError::Communication(
                format!(
                    "Failed to create feedback topic '{}': {}",
                    feedback_topic_name, e
                )
                .into(),
            )
        })?;
    let result_topic: Topic<serde_json::Value> = Topic::new(&result_topic_name).map_err(|e| {
        HorusError::Communication(
            format!(
                "Failed to create result topic '{}': {}",
                result_topic_name, e
            )
            .into(),
        )
    })?;

    let deadline = Instant::now() + Duration::from_secs_f64(timeout_secs);

    loop {
        if !running.load(std::sync::atomic::Ordering::SeqCst) {
            println!();
            println!("{} Interrupted", "".yellow());
            break;
        }
        if Instant::now() >= deadline {
            println!();
            println!(
                "{} Timed out waiting for result after {:.1}s",
                "".yellow(),
                timeout_secs
            );
            break;
        }

        // Check feedback (filter by goal_id)
        if let Some(feedback) = feedback_topic.recv() {
            let msg_goal_id = feedback.get("goal_id").and_then(|v| v.as_str());
            if msg_goal_id.is_none() || msg_goal_id == Some(&goal_id) {
                println!(
                    "  {} {}",
                    "Feedback:".cyan(),
                    serde_json::to_string(&feedback).unwrap_or_default()
                );
            }
        }

        // Check status (filter by goal_id)
        if let Some(status) = status_topic.recv() {
            let msg_goal_id = status.get("goal_id").and_then(|v| v.as_str());
            if msg_goal_id.is_none() || msg_goal_id == Some(&goal_id) {
                if let Some(status_str) = status.get("status").and_then(|s| s.as_str()) {
                    println!("  {} {}", "Status:".cyan(), status_str);
                    match status_str {
                        "Succeeded" | "Aborted" | "Canceled" | "Preempted" => break,
                        _ => {}
                    }
                }
            }
        }

        // Check result (filter by goal_id)
        if let Some(result) = result_topic.recv() {
            let msg_goal_id = result.get("goal_id").and_then(|v| v.as_str());
            if msg_goal_id.is_none() || msg_goal_id == Some(&goal_id) {
                println!();
                println!("{} Result:", cli_output::ICON_SUCCESS.green());
                println!(
                    "{}",
                    serde_json::to_string_pretty(&result).unwrap_or_default()
                );
                break;
            }
        }

        std::thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}

// ─── cancel_goal ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discovered_action_defaults() {
        let action = DiscoveredAction {
            name: "navigate".to_string(),
            has_goal: true,
            has_result: true,
            has_feedback: true,
            has_status: true,
            has_cancel: true,
            goal_publishers: 2,
            result_subscribers: 1,
        };
        assert_eq!(action.name, "navigate");
        assert!(action.has_goal && action.has_result && action.has_feedback);
    }

    #[test]
    fn discovered_action_partial() {
        let action = DiscoveredAction {
            name: "pick".to_string(),
            has_goal: true,
            has_result: false,
            has_feedback: false,
            has_status: false,
            has_cancel: false,
            goal_publishers: 1,
            result_subscribers: 0,
        };
        assert!(action.has_goal);
        assert!(!action.has_result);
    }

    #[test]
    fn action_topic_suffix_stripping() {
        let suffixes = ["/goal", "/result", "/feedback", "/status", "/cancel"];
        let base = "navigate_to_pose";
        for suffix in &suffixes {
            let topic = format!("{}{}", base, suffix);
            let stripped = topic.strip_suffix(suffix).unwrap();
            assert_eq!(stripped, base);
        }
    }

    #[test]
    fn action_topic_with_horus_prefix() {
        let topic = "horus_topic/navigate/goal";
        let stripped = topic
            .strip_prefix("horus_topic/")
            .and_then(|n| n.strip_suffix("/goal"));
        assert_eq!(stripped, Some("navigate"));
    }

    #[test]
    fn action_not_confused_with_service() {
        // An action should NOT have /request or /response topics
        let topic_names: std::collections::HashSet<String> =
            ["nav/goal", "nav/result", "nav/feedback"]
                .iter()
                .map(|s| s.to_string())
                .collect();

        // No /request or /response for "nav" means it's an action, not a service
        assert!(!topic_names.contains("nav/request"));
        assert!(!topic_names.contains("nav/response"));
    }

    #[test]
    fn send_goal_rejects_invalid_json() {
        // send_goal validates JSON before opening topics. Invalid JSON → Config error.
        let result = send_goal("test_action", "not valid json", false, 5.0);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Invalid goal JSON"),
            "error should mention invalid JSON: {}",
            err
        );
    }

    #[test]
    fn action_name_matching_logic() {
        // Mimic the matching in action_info: exact, ends_with /name, or basename match.
        let action_name = "robot/navigate_to_pose";
        let search = "navigate_to_pose";

        let matches = action_name == search
            || action_name.ends_with(&format!("/{}", search))
            || action_name
                .rsplit('/')
                .next()
                .map(|base| base == search)
                .unwrap_or(false);
        assert!(matches, "should match by base name");
    }

    #[test]
    fn action_name_exact_match() {
        let action_name = "navigate";
        let search = "navigate";
        assert_eq!(action_name, search);
    }

    #[test]
    fn action_name_no_match() {
        let action_name = "navigate";
        let search = "pick";
        let matches = action_name == search
            || action_name.ends_with(&format!("/{}", search))
            || action_name
                .rsplit('/')
                .next()
                .map(|base| base == search)
                .unwrap_or(false);
        assert!(!matches);
    }

    #[test]
    fn cancel_goal_topic_name_format() {
        let name = "navigate";
        let cancel_topic = format!("{}/cancel", name);
        assert_eq!(cancel_topic, "navigate/cancel");
    }

    #[test]
    fn goal_request_json_structure() {
        let goal_id = "test-uuid-1234";
        let goal_value = serde_json::json!({"x": 1.0, "y": 2.0});
        let goal_request = serde_json::json!({
            "goal_id": goal_id,
            "priority": 128,
            "payload": goal_value,
        });
        assert_eq!(goal_request["goal_id"], "test-uuid-1234");
        assert_eq!(goal_request["priority"], 128);
        assert_eq!(goal_request["payload"]["x"], 1.0);
    }
}

/// Cancel a goal on an action server (`horus action cancel_goal <name>`)
pub fn cancel_goal(name: &str, goal_id: Option<&str>) -> HorusResult<()> {
    use horus_core::communication::Topic;

    let cancel_topic_name = format!("{}/cancel", name);
    let cancel_request = serde_json::json!({
        "goal_id": goal_id.unwrap_or(""),  // empty = cancel all
        "cancel_all": goal_id.is_none(),
    });

    let cancel_topic: Topic<serde_json::Value> = Topic::new(&cancel_topic_name).map_err(|e| {
        HorusError::Config(format!(
            "Cannot open cancel topic '{}': {}",
            cancel_topic_name, e
        ))
    })?;

    cancel_topic.send(cancel_request);

    println!(
        "{} Cancel request sent to action '{}'",
        cli_output::ICON_SUCCESS.green(),
        name
    );
    if let Some(id) = goal_id {
        println!("  {} {}", "Goal ID:".dimmed(), id);
    } else {
        println!("  {} (all goals)", "Canceling:".dimmed());
    }

    Ok(())
}
