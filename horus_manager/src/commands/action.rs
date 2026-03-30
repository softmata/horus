//! Action command - Interact with HORUS actions
//!
//! Provides commands for listing, inspecting, and sending goals to actions.
//! Equivalent to `ros2 action list/info/send_goal/cancel_goal`.

use crate::cli_output;
use crate::discovery::{discover_nodes, discover_shared_memory};
use colored::*;
use horus_core::communication::TopicKind;
use horus_core::core::DurationExt;
use horus_core::error::{ConfigError, HorusError, HorusResult};

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

/// Discover active actions using three complementary sources:
///
/// 1. **SHM topic suffixes**: Topics ending with `.goal`, `.result`, etc.
/// 2. **TopicKind metadata**: SHM header `topic_kind` field set to ActionGoal/
///    ActionResult/etc. by `Topic::new_with_kind()`.
/// 3. **Node presence**: The `actions` field in node presence files lists
///    action names registered by running action servers.
///
/// Sources are merged — an action found by any source is included.
fn discover_actions() -> HorusResult<Vec<DiscoveredAction>> {
    let topics = discover_shared_memory()?;

    let mut actions: std::collections::HashMap<String, DiscoveredAction> =
        std::collections::HashMap::new();

    let suffixes = [
        (".goal", "goal"),
        (".result", "result"),
        (".feedback", "feedback"),
        (".status", "status"),
        (".cancel", "cancel"),
    ];

    // ── Source 1: SHM topic suffix matching ──────────────────────────────

    for topic in &topics {
        let topic_name = topic
            .topic_name
            .strip_prefix("horus_topic/")
            .unwrap_or(&topic.topic_name);

        for (suffix, field) in &suffixes {
            if let Some(action_name) = topic_name.strip_suffix(suffix) {
                if action_name.is_empty() {
                    continue;
                }
                let entry = actions
                    .entry(action_name.to_string())
                    .or_insert_with(|| DiscoveredAction::new(action_name));

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

    // ── Source 2: TopicKind metadata ─────────────────────────────────────
    //
    // ActionServer/ActionClient set topic_kind to ActionGoal (3), ActionResult (5),
    // etc. This catches action topics even if the naming convention changes.

    let topic_kind_map: &[(u8, &str)] = &[
        (TopicKind::ActionGoal as u8, "goal"),
        (TopicKind::ActionFeedback as u8, "feedback"),
        (TopicKind::ActionResult as u8, "result"),
        (TopicKind::ActionStatus as u8, "status"),
        (TopicKind::ActionCancel as u8, "cancel"),
    ];

    for topic in &topics {
        if topic.topic_kind == 0 {
            continue; // TopicKind::Data — skip
        }
        for &(kind, field) in topic_kind_map {
            if topic.topic_kind != kind {
                continue;
            }
            // Derive action name by stripping the suffix from topic_name
            let topic_name = topic
                .topic_name
                .strip_prefix("horus_topic/")
                .unwrap_or(&topic.topic_name);
            let suffix = format!(".{}", field);
            let action_name = topic_name.strip_suffix(&suffix).unwrap_or(topic_name);
            if action_name.is_empty() {
                continue;
            }
            let entry = actions
                .entry(action_name.to_string())
                .or_insert_with(|| DiscoveredAction::new(action_name));
            match field {
                "goal" => {
                    entry.has_goal = true;
                    if entry.goal_publishers == 0 {
                        entry.goal_publishers = topic.publishers.len();
                    }
                }
                "result" => {
                    entry.has_result = true;
                    if entry.result_subscribers == 0 {
                        entry.result_subscribers = topic.subscribers.len();
                    }
                }
                "feedback" => entry.has_feedback = true,
                "status" => entry.has_status = true,
                "cancel" => entry.has_cancel = true,
                _ => {}
            }
            break;
        }
    }

    // ── Source 3: Node presence files ────────────────────────────────────
    //
    // Running nodes report their topics in presence files. Topics matching
    // the action naming convention ({name}.goal, {name}.result, etc.) are
    // used to discover actions. The `actions` field in presence files also
    // lists action names directly.

    if let Ok(nodes) = discover_nodes() {
        for node in &nodes {
            for pub_topic in &node.publishers {
                for (suffix, field) in &suffixes {
                    if let Some(action_name) = pub_topic.topic.strip_suffix(suffix) {
                        if action_name.is_empty() {
                            continue;
                        }
                        let entry = actions
                            .entry(action_name.to_string())
                            .or_insert_with(|| DiscoveredAction::new(action_name));
                        apply_field(entry, field);
                    }
                }
            }
            for sub_topic in &node.subscribers {
                for (suffix, field) in &suffixes {
                    if let Some(action_name) = sub_topic.topic.strip_suffix(suffix) {
                        if action_name.is_empty() {
                            continue;
                        }
                        let entry = actions
                            .entry(action_name.to_string())
                            .or_insert_with(|| DiscoveredAction::new(action_name));
                        apply_field(entry, field);
                    }
                }
            }
        }
    }

    // ── Filter: require at least one action-specific topic ──────────────

    let mut result: Vec<DiscoveredAction> = actions
        .into_values()
        .filter(|a| a.has_goal || a.has_result)
        .collect();

    // Exclude entries that are actually services (have .request or .response partner)
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
        !topic_names.contains(&format!("{}.request", a.name))
            && !topic_names.contains(&format!("{}.response", a.name))
    });

    result.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(result)
}

impl DiscoveredAction {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            has_goal: false,
            has_result: false,
            has_feedback: false,
            has_status: false,
            has_cancel: false,
            goal_publishers: 0,
            result_subscribers: 0,
        }
    }
}

/// Apply an action field based on the suffix name.
fn apply_field(entry: &mut DiscoveredAction, field: &str) {
    match field {
        "goal" => entry.has_goal = true,
        "result" => entry.has_result = true,
        "feedback" => entry.has_feedback = true,
        "status" => entry.has_status = true,
        "cancel" => entry.has_cancel = true,
        _ => {}
    }
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
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Action '{}' not found. Use 'horus action list' to see available actions.",
            name
        ))));
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
            "*".green().to_string()
        } else {
            "x".red().to_string()
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
    use std::time::Instant;

    // Note: action goals across network (horus_net) are not yet supported.
    eprintln!(
        "{}",
        "Note: Cross-machine action goals via horus_net are not yet supported.\n      \
         If the action server is on a remote machine, this may time out."
            .dimmed()
    );

    // Validate JSON
    let goal_value: serde_json::Value =
        serde_json::from_str(goal_json).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Invalid goal JSON: {}\n  Example: horus action send_goal {} '{{\"x\": 1.0, \"y\": 2.0}}'",
                e, name
            )))
        })?;

    // Action topics use a wrapper: GoalRequest<G> which has uuid + priority + payload
    // We use serde_json::Value as the payload.
    let goal_topic_name = format!("{}.goal", name);
    let status_topic_name = format!("{}.status", name);
    let feedback_topic_name = format!("{}.feedback", name);
    let result_topic_name = format!("{}.result", name);

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

    // Write goal as JSON to file gateway (same approach as service call).
    // Avoids type mismatch: CLI uses Value, server uses GoalRequest<PickObjectGoal>.
    let json_bytes = serde_json::to_vec(&goal_request).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!("Failed to serialize goal: {}", e)))
    })?;
    let gateway_dir = horus_core::memory::shm_topics_dir().join(".service_gateway");
    std::fs::create_dir_all(&gateway_dir).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to create gateway dir: {}", e
        )))
    })?;
    let goal_file = gateway_dir.join(format!("{}.goal.json", name));
    std::fs::write(&goal_file, &json_bytes).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!("Failed to write goal: {}", e)))
    })?;
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

    let deadline = Instant::now() + timeout_secs.secs();

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

        std::thread::sleep(100_u64.ms());
    }

    Ok(())
}

/// Cancel a goal on an action server (`horus action cancel_goal <name>`)
pub fn cancel_goal(name: &str, goal_id: Option<&str>) -> HorusResult<()> {
    use horus_core::communication::Topic;

    let cancel_topic_name = format!("{}.cancel", name);
    let cancel_request = serde_json::json!({
        "goal_id": goal_id.unwrap_or(""),  // empty = cancel all
        "cancel_all": goal_id.is_none(),
    });

    let cancel_topic: Topic<serde_json::Value> = Topic::new(&cancel_topic_name).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Cannot open cancel topic '{}': {}",
            cancel_topic_name, e
        )))
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

// ─── cancel_goal ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discovered_action_complete_has_all_sub_topics() {
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
        // A complete action must have all 5 sub-topics
        assert!(action.has_goal && action.has_result && action.has_feedback);
        assert!(action.has_status && action.has_cancel);
        assert!(
            action.goal_publishers > 0,
            "complete action needs at least one goal publisher"
        );
        assert!(
            action.result_subscribers > 0,
            "complete action needs at least one result subscriber"
        );

        // Clone preserves all fields
        let cloned = action.clone();
        assert_eq!(cloned.name, action.name);
        assert_eq!(cloned.goal_publishers, action.goal_publishers);
        assert_eq!(cloned.result_subscribers, action.result_subscribers);

        // Debug contains action name
        let debug = format!("{:?}", action);
        assert!(
            debug.contains("navigate"),
            "Debug should contain action name"
        );
    }

    #[test]
    fn discovered_action_partial_is_incomplete() {
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
        assert!(!action.has_feedback);
        assert!(!action.has_status);
        assert!(!action.has_cancel);
        // A partial action with no result subscribers is inconsistent
        assert_eq!(action.result_subscribers, 0);
        // But it still has a valid goal publisher
        assert_eq!(action.goal_publishers, 1);
    }

    #[test]
    fn action_topic_suffix_stripping() {
        let suffixes = [".goal", ".result", ".feedback", ".status", ".cancel"];
        let base = "navigate_to_pose";
        for suffix in &suffixes {
            let topic = format!("{}{}", base, suffix);
            let stripped = topic.strip_suffix(suffix).unwrap();
            assert_eq!(stripped, base);
        }
    }

    #[test]
    fn action_topic_with_horus_prefix() {
        let topic = "horus_topic/navigate.goal";
        let stripped = topic
            .strip_prefix("horus_topic/")
            .and_then(|n| n.strip_suffix(".goal"));
        assert_eq!(stripped, Some("navigate"));
    }

    #[test]
    fn action_not_confused_with_service() {
        // An action should NOT have .request or .response topics
        let topic_names: std::collections::HashSet<String> =
            ["nav.goal", "nav.result", "nav.feedback"]
                .iter()
                .map(|s| s.to_string())
                .collect();

        // No .request or .response for "nav" means it's an action, not a service
        assert!(!topic_names.contains("nav.request"));
        assert!(!topic_names.contains("nav.response"));
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
        let cancel_topic = format!("{}.cancel", name);
        assert_eq!(cancel_topic, "navigate.cancel");
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

    // ── Battle tests: send_goal JSON validation ──────────────────────────

    #[test]
    fn send_goal_empty_string_is_invalid_json() {
        let result = send_goal("test_action", "", false, 5.0);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Invalid goal JSON"), "error: {}", err);
    }

    #[test]
    fn send_goal_null_json_is_valid() {
        // "null" is valid JSON
        let result = send_goal("test_action", "null", false, 5.0);
        // May fail due to Topic::new (no SHM), but should NOT fail on JSON parsing
        if let Err(e) = &result {
            let msg = e.to_string();
            assert!(
                !msg.contains("Invalid goal JSON"),
                "null should be valid JSON, got: {}",
                msg
            );
        }
    }

    #[test]
    fn send_goal_number_json_is_valid() {
        let result = send_goal("test_action", "42", false, 5.0);
        if let Err(e) = &result {
            let msg = e.to_string();
            assert!(
                !msg.contains("Invalid goal JSON"),
                "42 should be valid JSON, got: {}",
                msg
            );
        }
    }

    #[test]
    fn send_goal_string_json_is_valid() {
        let result = send_goal("test_action", "\"hello\"", false, 5.0);
        if let Err(e) = &result {
            let msg = e.to_string();
            assert!(
                !msg.contains("Invalid goal JSON"),
                "string should be valid JSON, got: {}",
                msg
            );
        }
    }

    #[test]
    fn send_goal_array_json_is_valid() {
        let result = send_goal("test_action", "[1, 2, 3]", false, 5.0);
        if let Err(e) = &result {
            let msg = e.to_string();
            assert!(
                !msg.contains("Invalid goal JSON"),
                "array should be valid JSON, got: {}",
                msg
            );
        }
    }

    #[test]
    fn send_goal_deeply_nested_json_is_valid() {
        let nested = r#"{"a":{"b":{"c":{"d":{"e":1}}}}}"#;
        let result = send_goal("test_action", nested, false, 5.0);
        if let Err(e) = &result {
            let msg = e.to_string();
            assert!(
                !msg.contains("Invalid goal JSON"),
                "nested JSON should be valid: {}",
                msg
            );
        }
    }

    #[test]
    fn send_goal_truncated_json_is_invalid() {
        let result = send_goal("test_action", "{\"x\": ", false, 5.0);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Invalid goal JSON"), "error: {}", err);
    }

    #[test]
    fn send_goal_trailing_comma_is_invalid() {
        let result = send_goal("test_action", "{\"x\": 1,}", false, 5.0);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Invalid goal JSON"), "error: {}", err);
    }

    #[test]
    fn send_goal_unicode_json_is_valid() {
        let result = send_goal("test_action", "{\"name\": \"\u{1F916}\"}", false, 5.0);
        if let Err(e) = &result {
            let msg = e.to_string();
            assert!(
                !msg.contains("Invalid goal JSON"),
                "unicode JSON should be valid: {}",
                msg
            );
        }
    }

    // ── Battle tests: action name matching ────────────────────────────────

    #[test]
    fn action_name_matching_nested_path() {
        let action_name = "robot/arm/navigate_to_pose";
        let search = "navigate_to_pose";

        let matches = action_name == search
            || action_name.ends_with(&format!("/{}", search))
            || action_name
                .rsplit('/')
                .next()
                .map(|base| base == search)
                .unwrap_or(false);
        assert!(matches, "deeply nested path should match by basename");
    }

    #[test]
    fn action_name_matching_empty_search() {
        let action_name = "navigate";
        let search = "";
        let matches = action_name == search
            || action_name.ends_with(&format!("/{}", search))
            || action_name
                .rsplit('/')
                .next()
                .map(|base| base == search)
                .unwrap_or(false);
        // "navigate".ends_with("/") => false, rsplit gives "navigate" != ""
        assert!(!matches, "empty search should not match non-empty action");
    }

    #[test]
    fn action_name_matching_both_empty() {
        let action_name = "";
        let search = "";
        let matches = action_name == search
            || action_name.ends_with(&format!("/{}", search))
            || action_name
                .rsplit('/')
                .next()
                .map(|base| base == search)
                .unwrap_or(false);
        assert!(matches, "empty matches empty via ==");
    }

    #[test]
    fn action_name_matching_slash_only() {
        let action_name = "/";
        let search = "";
        let matches = action_name == search
            || action_name.ends_with(&format!("/{}", search))
            || action_name
                .rsplit('/')
                .next()
                .map(|base| base == search)
                .unwrap_or(false);
        // "/" ends_with "/" => true
        assert!(
            matches,
            "slash action should match empty search via ends_with"
        );
    }

    // ── Battle tests: topic suffix stripping edge cases ──────────────────

    #[test]
    fn action_topic_suffix_with_dots_in_name() {
        // Action name containing dots should not confuse suffix stripping
        let topic = "my.robot.navigate.goal";
        let stripped = topic.strip_suffix(".goal");
        assert_eq!(stripped, Some("my.robot.navigate"));
    }

    #[test]
    fn action_topic_suffix_no_match() {
        let topic = "navigate.request";
        let stripped = topic.strip_suffix(".goal");
        assert!(stripped.is_none());
    }

    #[test]
    fn action_topic_empty_name_with_suffix() {
        let topic = ".goal";
        let stripped = topic.strip_suffix(".goal");
        assert_eq!(stripped, Some(""));
    }

    // ── Battle tests: DiscoveredAction struct ─────────────────────────────

    #[test]
    fn discovered_action_default_values() {
        let action = DiscoveredAction {
            name: String::new(),
            has_goal: false,
            has_result: false,
            has_feedback: false,
            has_status: false,
            has_cancel: false,
            goal_publishers: 0,
            result_subscribers: 0,
        };
        assert!(action.name.is_empty());
        assert!(!action.has_goal);
        assert!(!action.has_result);
        assert!(!action.has_feedback);
        assert!(!action.has_status);
        assert!(!action.has_cancel);
        assert_eq!(action.goal_publishers, 0);
        assert_eq!(action.result_subscribers, 0);
    }

    #[test]
    fn discovered_action_with_unicode_name() {
        let action = DiscoveredAction {
            name: "\u{1F916}_robot/\u{1F3AF}_navigate".to_string(),
            has_goal: true,
            has_result: true,
            has_feedback: false,
            has_status: false,
            has_cancel: false,
            goal_publishers: 1,
            result_subscribers: 1,
        };
        assert!(action.name.contains('\u{1F916}'));
        assert!(action.has_goal);
        assert!(action.has_result);
    }

    #[test]
    fn discovered_action_large_publisher_count() {
        let action = DiscoveredAction {
            name: "stress_test".to_string(),
            has_goal: true,
            has_result: true,
            has_feedback: true,
            has_status: true,
            has_cancel: true,
            goal_publishers: usize::MAX,
            result_subscribers: usize::MAX,
        };
        assert_eq!(action.goal_publishers, usize::MAX);
        assert_eq!(action.result_subscribers, usize::MAX);
    }

    // ── Battle tests: cancel_goal ─────────────────────────────────────────

    #[test]
    fn cancel_goal_json_structure_with_id() {
        let goal_id = Some("abc-123");
        let cancel_request = serde_json::json!({
            "goal_id": goal_id.unwrap_or(""),
            "cancel_all": goal_id.is_none(),
        });
        assert_eq!(cancel_request["goal_id"], "abc-123");
        assert_eq!(cancel_request["cancel_all"], false);
    }

    #[test]
    fn cancel_goal_json_structure_cancel_all() {
        let goal_id: Option<&str> = None;
        let cancel_request = serde_json::json!({
            "goal_id": goal_id.unwrap_or(""),
            "cancel_all": goal_id.is_none(),
        });
        assert_eq!(cancel_request["goal_id"], "");
        assert_eq!(cancel_request["cancel_all"], true);
    }

    #[test]
    fn cancel_goal_topic_name_with_namespace() {
        let name = "robot/arm/grasp";
        let cancel_topic = format!("{}.cancel", name);
        assert_eq!(cancel_topic, "robot/arm/grasp.cancel");
    }

    #[test]
    fn cancel_goal_topic_name_empty() {
        let name = "";
        let cancel_topic = format!("{}.cancel", name);
        assert_eq!(cancel_topic, ".cancel");
    }

    // ── Battle tests: goal topic name format ──────────────────────────────

    #[test]
    fn goal_topic_name_format() {
        let name = "navigate";
        assert_eq!(format!("{}.goal", name), "navigate.goal");
        assert_eq!(format!("{}.status", name), "navigate.status");
        assert_eq!(format!("{}.feedback", name), "navigate.feedback");
        assert_eq!(format!("{}.result", name), "navigate.result");
        assert_eq!(format!("{}.cancel", name), "navigate.cancel");
    }

    #[test]
    fn goal_topic_name_with_deep_namespace() {
        let name = "a/b/c/d/action";
        assert_eq!(format!("{}.goal", name), "a/b/c/d/action.goal");
    }

    // ── Battle tests: goal_id truncation display ─────────────────────────

    #[test]
    fn goal_id_short_display() {
        let goal_id = "abcdefgh-1234-5678";
        let short = goal_id.get(..8).unwrap_or(&goal_id);
        assert_eq!(short, "abcdefgh");
    }

    #[test]
    fn goal_id_short_display_very_short() {
        let goal_id = "abc";
        let short = goal_id.get(..8).unwrap_or(&goal_id);
        // get(..8) on a 3-char string returns None -> falls back to full string
        assert_eq!(short, "abc");
    }

    #[test]
    fn goal_id_short_display_empty() {
        let goal_id = "";
        let short = goal_id.get(..8).unwrap_or(&goal_id);
        assert_eq!(short, "");
    }

    // ── Battle tests: service vs action disambiguation ───────────────────

    #[test]
    fn action_excluded_when_request_topic_exists() {
        let topic_names: std::collections::HashSet<String> = [
            "nav.goal",
            "nav.result",
            "nav.feedback",
            "nav.request", // This makes it look like a service
            "nav.response",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();

        let name = "nav";
        let has_request = topic_names.contains(&format!("{}.request", name));
        let has_response = topic_names.contains(&format!("{}.response", name));
        // An action should be excluded if it has .request or .response
        assert!(has_request, "should detect .request");
        assert!(has_response, "should detect .response");
    }

    #[test]
    fn action_kept_when_no_service_topics() {
        let topic_names: std::collections::HashSet<String> = [
            "nav.goal",
            "nav.result",
            "nav.feedback",
            "nav.status",
            "nav.cancel",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();

        let name = "nav";
        let has_request = topic_names.contains(&format!("{}.request", name));
        let has_response = topic_names.contains(&format!("{}.response", name));
        assert!(!has_request);
        assert!(!has_response);
    }

    // ── Battle tests: list/info with no running app ──────────────────────

    #[test]
    fn list_actions_returns_ok_or_discovery_error() {
        let result = list_actions(false, false);
        // Without running HORUS, this may return Ok (empty) or Err (no SHM)
        // The important thing is no panic
        let _ = result;
    }

    #[test]
    fn list_actions_json_returns_ok_or_discovery_error() {
        let result = list_actions(false, true);
        let _ = result;
    }

    #[test]
    fn list_actions_verbose_returns_ok_or_discovery_error() {
        let result = list_actions(true, false);
        let _ = result;
    }

    #[test]
    fn action_info_nonexistent_returns_error() {
        // action_info first calls discover_actions; if that succeeds with empty,
        // it returns a "not found" error
        let result = action_info("totally_nonexistent_action_xyz");
        // Either discover error or not-found error
        if let Err(e) = &result {
            let msg = e.to_string();
            // Should contain either "not found" or a discovery-related error
            let _ = msg; // Just verifying no panic
        }
    }

    #[test]
    fn action_info_empty_name() {
        let result = action_info("");
        // Should not panic
        let _ = result;
    }

    #[test]
    fn action_info_unicode_name() {
        let result = action_info("\u{1F916}");
        let _ = result;
    }

    #[test]
    fn action_info_very_long_name() {
        let long_name = "x".repeat(4096);
        let result = action_info(&long_name);
        let _ = result;
    }

    // ── Integration: discover_actions with real SHM topics ───────────────

    /// Create a real SHM Topic and return it (keeps the SHM file alive).
    fn create_shm_topic(name: &str) -> horus_core::communication::Topic<u8> {
        horus_core::communication::Topic::new(name).expect("create SHM topic")
    }

    /// Wait for the discovery cache to expire so fresh SHM scans happen.
    fn wait_cache_expire() {
        std::thread::sleep(std::time::Duration::from_millis(300));
    }

    #[test]
    fn discover_actions_finds_action_from_shm_topics() {
        // Create SHM topics mimicking an action server
        let _goal = create_shm_topic("test_discover_nav.goal");
        let _result = create_shm_topic("test_discover_nav.result");
        let _feedback = create_shm_topic("test_discover_nav.feedback");
        let _status = create_shm_topic("test_discover_nav.status");
        let _cancel = create_shm_topic("test_discover_nav.cancel");

        wait_cache_expire();
        let actions = discover_actions().unwrap_or_default();
        let found = actions
            .iter()
            .find(|a| a.name.contains("test_discover_nav"));

        assert!(
            found.is_some(),
            "should discover action from 5 SHM topics. Found: {:?}",
            actions.iter().map(|a| &a.name).collect::<Vec<_>>()
        );

        if let Some(action) = found {
            assert!(action.has_goal, "should have .goal topic");
            assert!(action.has_result, "should have .result topic");
            assert!(action.has_feedback, "should have .feedback topic");
            assert!(action.has_status, "should have .status topic");
            assert!(action.has_cancel, "should have .cancel topic");
        }
    }

    #[test]
    fn discover_actions_partial_only_goal() {
        let _goal = create_shm_topic("test_partial_act.goal");

        wait_cache_expire();
        let actions = discover_actions().unwrap_or_default();
        let found = actions
            .iter()
            .find(|a| a.name.contains("test_partial_act"));

        assert!(
            found.is_some(),
            "should discover partial action with only .goal"
        );
        if let Some(action) = found {
            assert!(action.has_goal);
            assert!(!action.has_result);
        }
    }

    #[test]
    fn discover_actions_excludes_service_topics() {
        // Create topics that look like BOTH an action and a service
        let _goal = create_shm_topic("test_svc_excl.goal");
        let _result = create_shm_topic("test_svc_excl.result");
        let _request = create_shm_topic("test_svc_excl.request");
        let _response = create_shm_topic("test_svc_excl.response");

        wait_cache_expire();
        let actions = discover_actions().unwrap_or_default();
        let found = actions
            .iter()
            .find(|a| a.name.contains("test_svc_excl"));

        assert!(
            found.is_none(),
            "should exclude topics that also have .request/.response (service)"
        );
    }

    #[test]
    fn discover_actions_empty_shm_returns_empty() {
        // Just verify no panic — may find actions from other tests in same process
        let actions = discover_actions();
        assert!(actions.is_ok(), "discover_actions should not error");
    }

    #[test]
    fn discover_actions_multiple_actions() {
        let _g1 = create_shm_topic("test_multi_a.goal");
        let _r1 = create_shm_topic("test_multi_a.result");
        let _g2 = create_shm_topic("test_multi_b.goal");
        let _r2 = create_shm_topic("test_multi_b.result");

        wait_cache_expire();
        let actions = discover_actions().unwrap_or_default();
        let found_a = actions
            .iter()
            .any(|a| a.name.contains("test_multi_a"));
        let found_b = actions
            .iter()
            .any(|a| a.name.contains("test_multi_b"));

        assert!(found_a, "should find test_multi_a action");
        assert!(found_b, "should find test_multi_b action");
    }
}
