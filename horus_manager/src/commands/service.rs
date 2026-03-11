//! Service command - Interact with HORUS services
//!
//! Provides commands for listing, calling, and inspecting services.
//! Equivalent to `ros2 service list/call/type/find`.

use crate::cli_output;
use crate::discovery::discover_shared_memory;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::core::DurationExt;

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

// ─── service_call ─────────────────────────────────────────────────────────────

/// Call a service from the CLI (`horus service call <name> <request_json>`)
///
/// Since the CLI doesn't know the concrete Rust types at compile time, this
/// sends the request as a JSON-encoded string and displays the raw JSON
/// response.  This matches `ros2 service call` behaviour.
pub fn call_service(name: &str, request_json: &str, timeout_secs: f64) -> HorusResult<()> {
    use horus_core::communication::Topic;
    use horus_core::services::types::{ServiceRequest, ServiceResponse};
    use std::time::Instant;

    // Parse the request JSON to validate it's valid JSON.
    let request_value: serde_json::Value = serde_json::from_str(request_json).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Invalid request JSON: {}\n  Example: horus service call {} '{{\"a\": 3, \"b\": 4}}'",
            e, name
        )))
    })?;

    // We use serde_json::Value as the payload type — the topics are JSON-encoded.
    let req_topic_name = format!("{}.request", name);
    let res_topic_name = format!("{}.response", name);

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
    println!("  {} {}", "Request:".dimmed(), request_json);
    println!(
        "  {} Waiting up to {:.1}s for response...",
        "".dimmed(),
        timeout_secs
    );
    println!();

    let req_topic: Topic<ServiceRequest<serde_json::Value>> =
        Topic::new(&req_topic_name).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Cannot open request topic '{}': {}\n  Is a HORUS server running for service '{}'?",
                req_topic_name, e, name
            )))
        })?;

    let res_topic: Topic<ServiceResponse<serde_json::Value>> = Topic::new(&res_topic_name)
        .map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Cannot open response topic '{}': {}",
                res_topic_name, e
            )))
        })?;

    // Send the request.
    req_topic.send(ServiceRequest {
        request_id,
        payload: request_value,
    });

    // Poll for the matching response with Ctrl+C support.
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
            return Ok(());
        }

        if Instant::now() >= deadline {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Service call timed out after {:.1}s.\n  Is a server registered for '{}'?",
                timeout_secs, name
            ))));
        }

        if let Some(resp) = res_topic.recv() {
            if resp.request_id == request_id {
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
    fn discovered_service_defaults() {
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
        assert!(svc.has_request && svc.has_response);
    }

    #[test]
    fn discovered_service_partial() {
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
}
