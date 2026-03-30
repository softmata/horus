//! Service command - Interact with HORUS services
//!
//! Provides commands for listing, calling, and inspecting services.
//! Equivalent to `ros2 service list/call/type/find`.

use crate::cli_output;
use crate::discovery::discover_shared_memory;
use colored::*;
use horus_core::core::DurationExt;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::memory::shm_topics_dir;

// ─── Service discovery helpers ────────────────────────────────────────────────

/// Detected service based on topic pairs in shared memory.
#[derive(Debug, Clone)]
struct DiscoveredService {
    name: String,
    has_request: bool,
    has_response: bool,
    request_publishers: usize,
    request_subscribers: usize,
    response_publishers: usize,
    response_subscribers: usize,
}

/// Discover active services by scanning SHM topics for `.request` + `.response` pairs.
fn discover_services() -> HorusResult<Vec<DiscoveredService>> {
    let topics = discover_shared_memory()?;

    let mut services: std::collections::HashMap<String, DiscoveredService> =
        std::collections::HashMap::new();

    for topic in &topics {
        // Match topics that end with .request or .response
        if let Some(svc_name) = topic.topic_name.strip_suffix(".request").or_else(|| {
            // Also handle horus_topic/ prefix
            topic
                .topic_name
                .strip_prefix("horus_topic/")
                .and_then(|n| n.strip_suffix(".request"))
        }) {
            let entry = services
                .entry(svc_name.to_string())
                .or_insert_with(|| DiscoveredService {
                    name: svc_name.to_string(),
                    has_request: false,
                    has_response: false,
                    request_publishers: 0,
                    request_subscribers: 0,
                    response_publishers: 0,
                    response_subscribers: 0,
                });
            entry.has_request = true;
            entry.request_publishers = topic.publishers.len();
            entry.request_subscribers = topic.subscribers.len();
        } else if let Some(svc_name) = topic.topic_name.strip_suffix(".response").or_else(|| {
            topic
                .topic_name
                .strip_prefix("horus_topic/")
                .and_then(|n| n.strip_suffix(".response"))
        }) {
            let entry = services
                .entry(svc_name.to_string())
                .or_insert_with(|| DiscoveredService {
                    name: svc_name.to_string(),
                    has_request: false,
                    has_response: false,
                    request_publishers: 0,
                    request_subscribers: 0,
                    response_publishers: 0,
                    response_subscribers: 0,
                });
            entry.has_response = true;
            entry.response_publishers = topic.publishers.len();
            entry.response_subscribers = topic.subscribers.len();
        }
    }

    // Return services that have at least a request topic.
    let mut result: Vec<DiscoveredService> = services.into_values().collect();
    result.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(result)
}

// ─── list_services ────────────────────────────────────────────────────────────

/// List all active services (`horus service list`)
pub fn list_services(verbose: bool, json: bool) -> HorusResult<()> {
    let services = discover_services()?;

    if json {
        let items: Vec<_> = services
            .iter()
            .map(|s| {
                serde_json::json!({
                    "name": s.name,
                    "active": s.has_request && s.has_response,
                    "servers": s.response_publishers,
                    "clients": s.request_publishers,
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

    if services.is_empty() {
        println!("{}", "No active services found.".yellow());
        println!(
            "  {} Start a HORUS application with services to see them here",
            "Tip:".dimmed()
        );
        return Ok(());
    }

    println!("{}", "Active Services:".green().bold());
    println!();

    if verbose {
        for svc in &services {
            let status = if svc.has_request && svc.has_response {
                "active".green()
            } else if svc.has_request {
                "waiting for server".yellow()
            } else {
                "response only (unusual)".red()
            };
            println!("  {} {}", "Service:".cyan(), svc.name.white().bold());
            println!("    {} {}", "Status:".dimmed(), status);
            println!("    {} {}", "Servers:".dimmed(), svc.response_publishers);
            println!("    {} {}", "Clients:".dimmed(), svc.request_publishers);
            println!("    {} {}/request", "Request topic:".dimmed(), svc.name);
            println!("    {} {}/response", "Response topic:".dimmed(), svc.name);
            println!();
        }
    } else {
        println!(
            "  {:<35} {:>8} {:>8} {}",
            "NAME".dimmed(),
            "SERVERS".dimmed(),
            "CLIENTS".dimmed(),
            "STATUS".dimmed()
        );
        println!("  {}", "-".repeat(64).dimmed());
        for svc in &services {
            let status = if svc.has_request && svc.has_response {
                "active".green()
            } else {
                "waiting".yellow()
            };
            println!(
                "  {:<35} {:>8} {:>8} {}",
                svc.name, svc.response_publishers, svc.request_publishers, status
            );
        }
    }

    println!();
    println!("  {} {} service(s)", "Total:".dimmed(), services.len());

    Ok(())
}

// ─── service_type ─────────────────────────────────────────────────────────────

/// Show type info for a service (`horus service type <name>`)
pub fn service_type(name: &str) -> HorusResult<()> {
    let services = discover_services()?;

    let svc = services.iter().find(|s| {
        s.name == name
            || s.name.ends_with(&format!("/{}", name))
            || s.name
                .rsplit('/')
                .next()
                .map(|base| base == name)
                .unwrap_or(false)
    });

    let Some(svc) = svc else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Service '{}' not found. Use 'horus service list' to see available services.",
            name
        ))));
    };

    println!("{}", "Service Type Information".green().bold());
    println!();
    println!("  {} {}", "Name:".cyan(), svc.name.white().bold());
    println!(
        "  {} {}/request  (ServiceRequest<Req>)",
        "Request topic:".cyan(),
        svc.name
    );
    println!(
        "  {} {}/response (ServiceResponse<Res>)",
        "Response topic:".cyan(),
        svc.name
    );
    println!(
        "  {} {}",
        "Status:".cyan(),
        if svc.has_request && svc.has_response {
            "active".green()
        } else {
            "incomplete".yellow()
        }
    );
    println!("  {} {}", "Servers:".cyan(), svc.response_publishers);
    println!("  {} {}", "Clients:".cyan(), svc.request_publishers);
    println!();
    println!(
        "  {} Use 'horus service call {} <request_json>' to call this service",
        "Tip:".dimmed(),
        svc.name
    );

    Ok(())
}

// ─── Service name resolution ─────────────────────────────────────────────────

/// Resolve a user-provided service name to the actual name used in the SHM gateway.
///
/// The `service!` macro converts CamelCase type names to snake_case for the
/// service's wire name (e.g., `GetStatus` becomes `get_status`). Users may
/// pass either form on the CLI.
///
/// Resolution order:
/// 1. Try to discover the service from SHM topics (exact match, then suffix match)
/// 2. If not found via discovery, convert CamelCase to snake_case as a fallback
/// 3. If the name is already snake_case, return it unchanged
fn resolve_service_name(user_name: &str) -> String {
    // Try discovery first — this gives us the exact name from the running server.
    if let Ok(services) = discover_services() {
        // Exact match
        if let Some(svc) = services.iter().find(|s| s.name == user_name) {
            return svc.name.clone();
        }
        // Match by suffix (e.g., user passes "move" for "robot/arm/move")
        if let Some(svc) = services.iter().find(|s| {
            s.name.ends_with(&format!("/{}", user_name))
                || s.name
                    .rsplit('/')
                    .next()
                    .map(|base| base == user_name)
                    .unwrap_or(false)
        }) {
            return svc.name.clone();
        }
        // Match after converting user input to snake_case
        let snake = camel_to_snake(user_name);
        if let Some(svc) = services.iter().find(|s| s.name == snake) {
            return svc.name.clone();
        }
        // Also try suffix match with snake_case version
        if let Some(svc) = services.iter().find(|s| {
            s.name.ends_with(&format!("/{}", snake))
                || s.name
                    .rsplit('/')
                    .next()
                    .map(|base| base == snake)
                    .unwrap_or(false)
        }) {
            return svc.name.clone();
        }
    }

    // Fallback: if the name looks like CamelCase, convert to snake_case.
    // This handles the case where the server is running but discovery failed,
    // or the SHM topics haven't been created yet.
    let snake = camel_to_snake(user_name);
    if snake != user_name {
        return snake;
    }

    // Already snake_case or doesn't match any pattern — use as-is.
    user_name.to_string()
}

/// Convert a CamelCase identifier to snake_case.
///
/// Mirrors the conversion in the `service!` macro (`macros.rs`):
/// - `GetStatus` → `get_status`
/// - `AddTwoInts` → `add_two_ints`
/// - `already_snake` → `already_snake` (no change)
fn camel_to_snake(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 4);
    for (i, c) in s.chars().enumerate() {
        if c.is_uppercase() {
            if i > 0 {
                out.push('_');
            }
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

// ─── service_call ─────────────────────────────────────────────────────────────

/// Call a service from the CLI (`horus service call <name> <request_json>`)
///
/// Since the CLI doesn't know the concrete Rust types at compile time, this
/// sends the request as a JSON-encoded string and displays the raw JSON
/// response.  This matches `ros2 service call` behaviour.
///
/// The server's JSON gateway uses the snake_case service name from `Service::name()`
/// for file paths (e.g., `get_status.request.json`). The CLI resolves the
/// user-provided name (which may be CamelCase like `GetStatus`) to the actual
/// service name via SHM topic discovery, falling back to a CamelCase-to-snake_case
/// conversion.
pub fn call_service(name: &str, request_json: &str, timeout_secs: f64) -> HorusResult<()> {
    use horus_core::services::types::{ServiceRequest, ServiceResponse};
    use std::time::Instant;

    // Note: service calls across network (horus_net) are not yet supported.
    // The request/response correlation requires both directions to work
    // reliably with proper timeouts. If the service is on a remote peer,
    // this call will likely time out.
    eprintln!(
        "{}",
        "Note: Cross-machine service calls via horus_net are not yet supported.\n      \
         If the service is on a remote machine, this may time out."
            .dimmed()
    );

    // Parse the request JSON to validate it's valid JSON.
    let request_value: serde_json::Value = serde_json::from_str(request_json).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Invalid request JSON: {}\n  Example: horus service call {} '{{\"a\": 3, \"b\": 4}}'",
            e, name
        )))
    })?;

    // Resolve the user-provided name to the actual service name used in SHM.
    // The service! macro converts CamelCase to snake_case (e.g., GetStatus → get_status),
    // and the server's JSON gateway uses that snake_case name for file paths.
    let resolved_name = resolve_service_name(name);

    // Use a unique request_id for correlation.
    let request_id: u64 = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .subsec_nanos() as u64
        | (std::process::id() as u64) << 32;

    println!(
        "{} Calling service: {}",
        cli_output::ICON_INFO.cyan(),
        name.white().bold()
    );
    if resolved_name != name {
        println!("  {} resolved to '{}'", "".dimmed(), resolved_name);
    }
    println!("  {} {}", "Request:".dimmed(), request_json);
    println!(
        "  {} Waiting up to {:.1}s for response...",
        "".dimmed(),
        timeout_secs
    );
    println!();

    // Write JSON request to a file-based gateway (simple, no ring buffer complexity).
    // The server polls this file for requests.
    let req_msg = ServiceRequest {
        request_id,
        payload: request_value,
        response_topic: None,
    };
    let json_bytes = serde_json::to_vec(&req_msg).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to serialize request: {}",
            e
        )))
    })?;
    let gateway_dir = shm_topics_dir().join(".service_gateway");
    std::fs::create_dir_all(&gateway_dir).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to create service gateway dir '{}': {}\n  Is a HORUS server running?",
            gateway_dir.display(),
            e
        )))
    })?;
    let req_file = gateway_dir.join(format!("{}.request.json", resolved_name));
    let res_file = gateway_dir.join(format!("{}.response.{}.json", resolved_name, request_id));
    std::fs::write(&req_file, &json_bytes).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to write request file '{}': {}",
            req_file.display(),
            e
        )))
    })?;

    // Poll for response using raw SHM byte reading (avoids cross-process recv issues)
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    let timeout = timeout_secs.secs();
    let deadline = Instant::now() + timeout;
    let poll_interval = 10_u64.ms();
    loop {
        if !running.load(std::sync::atomic::Ordering::SeqCst) {
            println!();
            println!("{} Service call cancelled.", cli_output::ICON_WARN.yellow());
            let _ = std::fs::remove_file(&req_file);
            return Ok(());
        }

        if Instant::now() >= deadline {
            let _ = std::fs::remove_file(&req_file);
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Service call timed out after {:.1}s.\n  Is a server registered for '{}'?",
                timeout_secs, name
            ))));
        }

        // Poll for the JSON response file
        if let Ok(data) = std::fs::read(&res_file) {
            let _ = std::fs::remove_file(&res_file);
            let _ = std::fs::remove_file(&req_file);
            if let Ok(resp) = serde_json::from_slice::<ServiceResponse<serde_json::Value>>(&data) {
                if resp.ok {
                    let payload = resp.payload.unwrap_or(serde_json::Value::Null);
                    println!("{} Response received:", cli_output::ICON_SUCCESS.green());
                    println!(
                        "{}",
                        serde_json::to_string_pretty(&payload).unwrap_or_default()
                    );
                } else {
                    let err = resp.error.as_deref().unwrap_or("unknown error");
                    println!(
                        "{} Service returned error: {}",
                        cli_output::ICON_ERROR.red(),
                        err
                    );
                }
                return Ok(());
            }
        }

        std::thread::sleep(poll_interval);
    }
}

/// Find services by type name filter (`horus service find <type_filter>`)
pub fn find_services(type_filter: &str) -> HorusResult<()> {
    let services = discover_services()?;

    // Since we don't store type metadata in the SHM topics (the type names are
    // only known to the Rust types at compile time), we fall back to matching
    // service names that contain the filter string.
    let matched: Vec<&DiscoveredService> = services
        .iter()
        .filter(|s| s.name.to_lowercase().contains(&type_filter.to_lowercase()))
        .collect();

    if matched.is_empty() {
        println!(
            "{}",
            format!("No services matching '{}' found.", type_filter).yellow()
        );
        return Ok(());
    }

    for svc in &matched {
        println!("{}", svc.name);
    }

    Ok(())
}

// ─── service_find ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discovered_service_complete_has_both_topics() {
        let svc = DiscoveredService {
            name: "add_two_ints".to_string(),
            has_request: true,
            has_response: true,
            request_publishers: 1,
            request_subscribers: 1,
            response_publishers: 1,
            response_subscribers: 1,
        };
        assert_eq!(svc.name, "add_two_ints");
        // A complete service must have both request and response
        assert!(svc.has_request && svc.has_response);
        // With participants on both sides
        assert!(
            svc.request_publishers > 0,
            "service needs request publishers"
        );
        assert!(
            svc.request_subscribers > 0,
            "service needs request subscribers"
        );
        assert!(
            svc.response_publishers > 0,
            "service needs response publishers"
        );
        assert!(
            svc.response_subscribers > 0,
            "service needs response subscribers"
        );

        // Clone preserves all fields
        let cloned = svc.clone();
        assert_eq!(cloned.name, svc.name);
        assert_eq!(cloned.request_publishers, svc.request_publishers);
        assert_eq!(cloned.response_subscribers, svc.response_subscribers);

        // Debug contains service name
        let debug = format!("{:?}", svc);
        assert!(
            debug.contains("add_two_ints"),
            "Debug should contain service name"
        );
    }

    #[test]
    fn discovered_service_partial_lacks_response() {
        let svc = DiscoveredService {
            name: "broken_svc".to_string(),
            has_request: true,
            has_response: false,
            request_publishers: 1,
            request_subscribers: 0,
            response_publishers: 0,
            response_subscribers: 0,
        };
        assert!(svc.has_request);
        assert!(!svc.has_response);
        // Partial service: has request topic but no response topic
        assert_eq!(
            svc.response_publishers, 0,
            "no response means no response publishers"
        );
        assert_eq!(
            svc.response_subscribers, 0,
            "no response means no response subscribers"
        );
        // Even request_subscribers is 0 -- nobody listening
        assert_eq!(svc.request_subscribers, 0);
    }

    #[test]
    fn topic_suffix_stripping_request() {
        let topic = "my_service.request";
        let svc_name = topic.strip_suffix(".request");
        assert_eq!(svc_name, Some("my_service"));
    }

    #[test]
    fn topic_suffix_stripping_response() {
        let topic = "my_service.response";
        let svc_name = topic.strip_suffix(".response");
        assert_eq!(svc_name, Some("my_service"));
    }

    #[test]
    fn topic_suffix_stripping_with_prefix() {
        let topic = "horus_topic/my_service.request";
        let svc_name = topic
            .strip_prefix("horus_topic/")
            .and_then(|n| n.strip_suffix(".request"));
        assert_eq!(svc_name, Some("my_service"));
    }

    #[test]
    fn topic_suffix_stripping_no_match() {
        let topic = "my_topic.data";
        assert!(topic.strip_suffix(".request").is_none());
        assert!(topic.strip_suffix(".response").is_none());
    }

    // ── Battle tests: DiscoveredService struct edge cases ──────────────────

    #[test]
    fn discovered_service_no_topics_at_all() {
        let svc = DiscoveredService {
            name: "ghost_service".to_string(),
            has_request: false,
            has_response: false,
            request_publishers: 0,
            request_subscribers: 0,
            response_publishers: 0,
            response_subscribers: 0,
        };
        assert!(!svc.has_request);
        assert!(!svc.has_response);
        assert_eq!(svc.request_publishers + svc.response_publishers, 0);
    }

    #[test]
    fn discovered_service_response_only_unusual() {
        let svc = DiscoveredService {
            name: "odd_svc".to_string(),
            has_request: false,
            has_response: true,
            request_publishers: 0,
            request_subscribers: 0,
            response_publishers: 2,
            response_subscribers: 1,
        };
        assert!(!svc.has_request, "Should lack request topic");
        assert!(svc.has_response, "Should have response topic");
        assert_eq!(svc.response_publishers, 2);
    }

    #[test]
    fn discovered_service_multiple_servers_and_clients() {
        let svc = DiscoveredService {
            name: "load_balanced".to_string(),
            has_request: true,
            has_response: true,
            request_publishers: 5,
            request_subscribers: 3,
            response_publishers: 3,
            response_subscribers: 5,
        };
        assert!(svc.has_request && svc.has_response, "Should be active");
        assert_eq!(svc.request_publishers, 5, "5 clients publishing requests");
        assert_eq!(svc.response_publishers, 3, "3 servers publishing responses");
    }

    // ── Battle tests: topic suffix parsing edge cases ─────────────────────

    #[test]
    fn suffix_stripping_nested_service_name_request() {
        let topic = "robot/arm/move.request";
        let svc = topic.strip_suffix(".request");
        assert_eq!(svc, Some("robot/arm/move"));
    }

    #[test]
    fn suffix_stripping_nested_service_name_response() {
        let topic = "robot/arm/move.response";
        let svc = topic.strip_suffix(".response");
        assert_eq!(svc, Some("robot/arm/move"));
    }

    #[test]
    fn suffix_stripping_double_suffix_takes_last() {
        // Edge case: service name itself contains ".request"
        let topic = "debug.request.request";
        let svc = topic.strip_suffix(".request");
        assert_eq!(svc, Some("debug.request"));
    }

    #[test]
    fn suffix_stripping_with_horus_prefix_response() {
        let topic = "horus_topic/my_service.response";
        let svc = topic
            .strip_prefix("horus_topic/")
            .and_then(|n| n.strip_suffix(".response"));
        assert_eq!(svc, Some("my_service"));
    }

    #[test]
    fn suffix_stripping_empty_service_name() {
        let topic = ".request";
        let svc = topic.strip_suffix(".request");
        assert_eq!(svc, Some(""), "Empty service name is valid for stripping");
    }

    #[test]
    fn suffix_stripping_just_request_word_no_dot() {
        let topic = "request";
        let svc = topic.strip_suffix(".request");
        assert_eq!(svc, None, "'request' without dot should not strip");
    }

    // ── Battle tests: call_service JSON validation ────────────────────────

    #[test]
    fn call_service_rejects_invalid_json() {
        let result = call_service("test_svc", "not valid json", 1.0);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("Invalid request JSON"),
            "Should mention invalid JSON, got: {}",
            err
        );
    }

    #[test]
    fn call_service_rejects_trailing_comma_json() {
        let result = call_service("test_svc", "{\"a\": 1,}", 1.0);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("Invalid request JSON"),
            "Should reject trailing comma, got: {}",
            err
        );
    }

    #[test]
    fn call_service_rejects_empty_string() {
        let result = call_service("test_svc", "", 1.0);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("Invalid request JSON"),
            "Should reject empty string, got: {}",
            err
        );
    }

    // ── Battle tests: service name matching logic ─────────────────────────

    #[test]
    fn service_name_matching_exact() {
        let services = vec![
            make_test_service("add_two_ints"),
            make_test_service("multiply"),
        ];
        let name = "add_two_ints";
        let found = services.iter().find(|s| {
            s.name == name
                || s.name.ends_with(&format!("/{}", name))
                || s.name
                    .rsplit('/')
                    .next()
                    .map(|base| base == name)
                    .unwrap_or(false)
        });
        assert!(found.is_some());
        assert_eq!(found.unwrap().name, "add_two_ints");
    }

    #[test]
    fn service_name_matching_by_suffix() {
        let services = vec![make_test_service("robot/arm/move")];
        let name = "move";
        let found = services.iter().find(|s| {
            s.name == name
                || s.name.ends_with(&format!("/{}", name))
                || s.name
                    .rsplit('/')
                    .next()
                    .map(|base| base == name)
                    .unwrap_or(false)
        });
        assert!(found.is_some());
        assert_eq!(found.unwrap().name, "robot/arm/move");
    }

    #[test]
    fn service_name_matching_no_match() {
        let services = vec![make_test_service("robot/arm/move")];
        let name = "nonexistent_service";
        let found = services.iter().find(|s| {
            s.name == name
                || s.name.ends_with(&format!("/{}", name))
                || s.name
                    .rsplit('/')
                    .next()
                    .map(|base| base == name)
                    .unwrap_or(false)
        });
        assert!(found.is_none());
    }

    // ── Battle tests: find_services filter logic (in-process) ─────────────

    #[test]
    fn find_filter_case_insensitive() {
        let services = vec![
            make_test_service("RobotArm"),
            make_test_service("sensor_hub"),
        ];
        let filter = "robotarm";
        let matched: Vec<_> = services
            .iter()
            .filter(|s| s.name.to_lowercase().contains(&filter.to_lowercase()))
            .collect();
        assert_eq!(matched.len(), 1);
        assert_eq!(matched[0].name, "RobotArm");
    }

    #[test]
    fn find_filter_partial_match() {
        let services = vec![
            make_test_service("robot/arm/move"),
            make_test_service("robot/arm/grip"),
            make_test_service("sensor/lidar"),
        ];
        let filter = "arm";
        let matched: Vec<_> = services
            .iter()
            .filter(|s| s.name.to_lowercase().contains(&filter.to_lowercase()))
            .collect();
        assert_eq!(matched.len(), 2);
    }

    #[test]
    fn find_filter_empty_matches_all() {
        let services = vec![make_test_service("svc_a"), make_test_service("svc_b")];
        let filter = "";
        let matched: Vec<_> = services
            .iter()
            .filter(|s| s.name.to_lowercase().contains(&filter.to_lowercase()))
            .collect();
        assert_eq!(matched.len(), 2);
    }

    #[test]
    fn find_filter_no_match_returns_empty() {
        let services = vec![make_test_service("svc_a")];
        let filter = "zzz_nonexistent";
        let matched: Vec<_> = services
            .iter()
            .filter(|s| s.name.to_lowercase().contains(&filter.to_lowercase()))
            .collect();
        assert!(matched.is_empty());
    }

    /// Helper to create a DiscoveredService for testing
    fn make_test_service(name: &str) -> DiscoveredService {
        DiscoveredService {
            name: name.to_string(),
            has_request: true,
            has_response: true,
            request_publishers: 1,
            request_subscribers: 1,
            response_publishers: 1,
            response_subscribers: 1,
        }
    }

    // ── camel_to_snake tests ─────────────────────────────────────────────

    #[test]
    fn camel_to_snake_basic() {
        assert_eq!(camel_to_snake("GetStatus"), "get_status");
    }

    #[test]
    fn camel_to_snake_multiple_words() {
        assert_eq!(camel_to_snake("AddTwoInts"), "add_two_ints");
    }

    #[test]
    fn camel_to_snake_single_word() {
        assert_eq!(camel_to_snake("Status"), "status");
    }

    #[test]
    fn camel_to_snake_already_snake() {
        assert_eq!(camel_to_snake("get_status"), "get_status");
    }

    #[test]
    fn camel_to_snake_all_lowercase() {
        assert_eq!(camel_to_snake("myservice"), "myservice");
    }

    #[test]
    fn camel_to_snake_empty() {
        assert_eq!(camel_to_snake(""), "");
    }

    #[test]
    fn camel_to_snake_single_char_upper() {
        assert_eq!(camel_to_snake("A"), "a");
    }

    #[test]
    fn camel_to_snake_consecutive_uppers() {
        // Matches the service! macro behavior: each uppercase gets an underscore
        assert_eq!(camel_to_snake("HTTPServer"), "h_t_t_p_server");
    }

    // ── resolve_service_name tests ───────────────────────────────────────

    #[test]
    fn resolve_service_name_converts_camel_case() {
        // Without any running services, falls back to CamelCase → snake_case.
        let resolved = resolve_service_name("GetStatus");
        assert_eq!(resolved, "get_status");
    }

    #[test]
    fn resolve_service_name_preserves_snake_case() {
        let resolved = resolve_service_name("get_status");
        assert_eq!(resolved, "get_status");
    }

    #[test]
    fn resolve_service_name_converts_multi_word_camel() {
        let resolved = resolve_service_name("AddTwoInts");
        assert_eq!(resolved, "add_two_ints");
    }
}
