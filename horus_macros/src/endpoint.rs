//! Compile-time endpoint validation macro.
//!
//! This module provides the `endpoint!` macro for validating endpoint strings
//! at compile time, catching syntax errors before runtime.

use proc_macro2::TokenStream;
use quote::quote;
use syn::{parse::Parse, parse::ParseStream, LitStr, Result};

/// Input for the endpoint! macro - just a string literal
pub struct EndpointInput {
    pub endpoint_str: LitStr,
}

impl Parse for EndpointInput {
    fn parse(input: ParseStream) -> Result<Self> {
        let endpoint_str: LitStr = input.parse()?;
        Ok(EndpointInput { endpoint_str })
    }
}

/// Validate an endpoint string at compile time.
///
/// Returns a validation error message if invalid, None if valid.
fn validate_endpoint(input: &str) -> Option<String> {
    // Check for empty input
    if input.is_empty() {
        return Some("Endpoint cannot be empty".to_string());
    }

    // Local endpoint (no @)
    if !input.contains('@') {
        // Just validate topic name
        return validate_topic_name(input);
    }

    // Split on @ - should have exactly 2 parts
    let parts: Vec<&str> = input.split('@').collect();
    if parts.len() != 2 {
        return Some(format!(
            "Invalid endpoint format: expected 'topic@location', got multiple '@' symbols"
        ));
    }

    let topic = parts[0];
    let location = parts[1];

    // Validate topic
    if let Some(err) = validate_topic_name(topic) {
        return Some(err);
    }

    // Validate location
    validate_location(location)
}

/// Validate a topic name
fn validate_topic_name(topic: &str) -> Option<String> {
    if topic.is_empty() {
        return Some("Topic name cannot be empty".to_string());
    }

    // Topic can contain: letters, numbers, underscores, slashes (for namespacing)
    for c in topic.chars() {
        if !c.is_ascii_alphanumeric() && c != '_' && c != '/' && c != '-' {
            return Some(format!(
                "Invalid character '{}' in topic name. Allowed: letters, numbers, underscore, hyphen, slash",
                c
            ));
        }
    }

    // Cannot start or end with slash
    if topic.starts_with('/') && !is_ros2_topic(topic) {
        // ROS2 topics can start with /
    }
    if topic.ends_with('/') {
        return Some("Topic name cannot end with '/'".to_string());
    }

    None
}

/// Check if this looks like a ROS2-style topic (starts with /)
fn is_ros2_topic(topic: &str) -> bool {
    topic.starts_with('/')
}

/// Validate the location part of an endpoint
fn validate_location(location: &str) -> Option<String> {
    if location.is_empty() {
        return Some("Location cannot be empty after '@'".to_string());
    }

    // Check known patterns
    match location {
        // Simple patterns
        "*" | "multicast" | "localhost" | "router" | "zenoh" | "zenoh/ros2" => return None,

        // More complex patterns need further validation
        _ => {}
    }

    // Check for specific prefixes
    if location.starts_with("router:") {
        return validate_router_location(&location[7..]);
    }

    if location.starts_with("zenoh:cloud") {
        return validate_zenoh_cloud_location(location);
    }

    if location.starts_with("zenoh:") || location.starts_with("zenoh/ros2:") {
        // Zenoh with connect endpoint
        return None; // Accept any connect string
    }

    if location.starts_with("p2p:") {
        return validate_p2p_location(&location[4..]);
    }

    if location.starts_with("cloud:")
        || location.starts_with("relay:")
        || location.starts_with("relay-tls:")
        || location.starts_with("vpn:")
        || location.starts_with("vpn/")
    {
        return validate_cloud_location(location);
    }

    // Check for mDNS (.local suffix)
    if location.ends_with(".local") || location.contains(".local:") {
        return validate_mdns_location(location);
    }

    // Check for localhost variants
    if location == "127.0.0.1" || location == "::1" {
        return None;
    }

    // Should be an IP address (with optional port)
    validate_ip_location(location)
}

/// Validate router:port syntax
fn validate_router_location(port_str: &str) -> Option<String> {
    if port_str.is_empty() {
        return Some("Router port cannot be empty".to_string());
    }

    match port_str.parse::<u16>() {
        Ok(_) => None,
        Err(_) => Some(format!("Invalid router port '{}': must be 0-65535", port_str)),
    }
}

/// Validate zenoh:cloud variants
fn validate_zenoh_cloud_location(location: &str) -> Option<String> {
    // Valid patterns:
    // zenoh:cloud
    // zenoh:cloud/auto
    // zenoh:cloud/horus
    // zenoh:cloud:tcp/host:port
    // zenoh:cloud:tcp/host1:port,tcp/host2:port

    if location == "zenoh:cloud" || location == "zenoh:cloud/auto" || location == "zenoh:cloud/horus"
    {
        return None;
    }

    if location.starts_with("zenoh:cloud:") {
        let routers = &location[12..];
        if routers.is_empty() {
            return Some("Zenoh cloud router specification cannot be empty".to_string());
        }
        // Accept any router specification (will be validated at runtime)
        return None;
    }

    Some(format!("Invalid zenoh:cloud format: '{}'", location))
}

/// Validate p2p:peer-id syntax
fn validate_p2p_location(rest: &str) -> Option<String> {
    if rest.is_empty() {
        return Some("P2P peer ID cannot be empty".to_string());
    }

    // Check for strategy suffix
    let (peer_part, strategy) = if let Some(slash_pos) = rest.rfind('/') {
        (&rest[..slash_pos], Some(&rest[slash_pos + 1..]))
    } else {
        (rest, None)
    };

    // Validate peer ID format: xxxx-xxxx-xxxx
    let segments: Vec<&str> = peer_part.split('-').collect();
    if segments.len() != 3 {
        return Some(format!(
            "Invalid peer ID '{}': expected format 'xxxx-yyyy-zzzz'",
            peer_part
        ));
    }

    for seg in &segments {
        if seg.len() != 4 {
            return Some(format!(
                "Invalid peer ID segment '{}': each segment must be 4 characters",
                seg
            ));
        }
        if !seg.chars().all(|c| c.is_ascii_alphanumeric()) {
            return Some(format!(
                "Invalid peer ID segment '{}': must be alphanumeric",
                seg
            ));
        }
    }

    // Validate strategy if present
    if let Some(strat) = strategy {
        match strat {
            "auto" | "direct" | "stun" | "turn" => {}
            _ => {
                return Some(format!(
                    "Unknown P2P strategy '{}': expected 'auto', 'direct', 'stun', or 'turn'",
                    strat
                ))
            }
        }
    }

    None
}

/// Validate cloud/relay/vpn location
fn validate_cloud_location(location: &str) -> Option<String> {
    // cloud:room-name
    // relay:host[:port]
    // relay-tls:host[:port]
    // vpn:peer
    // vpn/tailscale:peer

    if location.starts_with("cloud:") {
        let room = &location[6..];
        if room.is_empty() {
            return Some("Cloud room name cannot be empty".to_string());
        }
        // Room name validation: alphanumeric, hyphen, underscore
        for c in room.chars() {
            if !c.is_ascii_alphanumeric() && c != '-' && c != '_' && c != '/' {
                return Some(format!("Invalid character '{}' in room name", c));
            }
        }
        return None;
    }

    if location.starts_with("relay:") || location.starts_with("relay-tls:") {
        // Accept any host/port
        return None;
    }

    if location.starts_with("vpn:") || location.starts_with("vpn/") {
        // Accept any VPN peer specification
        return None;
    }

    Some(format!("Unrecognized cloud location format: '{}'", location))
}

/// Validate mDNS hostname
fn validate_mdns_location(location: &str) -> Option<String> {
    // Extract hostname and optional port
    let (hostname_with_local, port_str) = if let Some(colon_pos) = location.rfind(':') {
        let after = &location[colon_pos + 1..];
        if after.parse::<u16>().is_ok() {
            (&location[..colon_pos], Some(after))
        } else {
            (location, None)
        }
    } else {
        (location, None)
    };

    // Extract hostname without .local
    let hostname = hostname_with_local
        .strip_suffix(".local")
        .unwrap_or(hostname_with_local);

    if hostname.is_empty() {
        return Some("mDNS hostname cannot be empty".to_string());
    }

    // Validate hostname: alphanumeric and hyphens only
    if hostname.starts_with('-') || hostname.ends_with('-') {
        return Some("mDNS hostname cannot start or end with hyphen".to_string());
    }

    for c in hostname.chars() {
        if !c.is_ascii_alphanumeric() && c != '-' {
            return Some(format!(
                "Invalid character '{}' in mDNS hostname: only letters, numbers, and hyphens allowed",
                c
            ));
        }
    }

    // Validate port if present
    if let Some(port) = port_str {
        if port.parse::<u16>().is_err() {
            return Some(format!("Invalid port '{}': must be 0-65535", port));
        }
    }

    None
}

/// Validate IP address location
fn validate_ip_location(location: &str) -> Option<String> {
    // IPv6 with brackets: [2001:db8::1]:9000
    if location.starts_with('[') {
        if let Some(bracket_end) = location.find(']') {
            let ipv6_str = &location[1..bracket_end];
            // Basic IPv6 validation - check for colons
            if !ipv6_str.contains(':') {
                return Some(format!("Invalid IPv6 address: '{}'", ipv6_str));
            }

            // Check for port after bracket
            if location.len() > bracket_end + 1 {
                if location.chars().nth(bracket_end + 1) != Some(':') {
                    return Some(format!("Invalid format after IPv6 address: '{}'", location));
                }
                let port_str = &location[bracket_end + 2..];
                if port_str.parse::<u16>().is_err() {
                    return Some(format!("Invalid port '{}': must be 0-65535", port_str));
                }
            }
            return None;
        } else {
            return Some("Missing closing bracket in IPv6 address".to_string());
        }
    }

    // Could be IPv4, IPv6 without brackets, or IPv4:port
    // Check for last colon (could be port separator or IPv6 address part)
    if let Some(colon_pos) = location.rfind(':') {
        let potential_ip = &location[..colon_pos];
        let potential_port = &location[colon_pos + 1..];

        // If the potential port parses as u16 and potential_ip looks like IPv4
        if potential_port.parse::<u16>().is_ok() && looks_like_ipv4(potential_ip) {
            // It's IP:port
            return validate_ipv4(potential_ip);
        }
    }

    // Could be IPv4 or IPv6 without port
    if location.contains(':') {
        // Likely IPv6 - basic validation
        if location.chars().filter(|c| *c == ':').count() < 2 {
            return Some(format!("Invalid address format: '{}'", location));
        }
        // Accept as IPv6 - will be validated at runtime
        None
    } else {
        // Should be IPv4
        validate_ipv4(location)
    }
}

fn looks_like_ipv4(s: &str) -> bool {
    let parts: Vec<&str> = s.split('.').collect();
    parts.len() == 4 && parts.iter().all(|p| p.parse::<u8>().is_ok())
}

fn validate_ipv4(s: &str) -> Option<String> {
    let parts: Vec<&str> = s.split('.').collect();
    if parts.len() != 4 {
        return Some(format!(
            "Invalid IPv4 address '{}': expected format 'x.x.x.x'",
            s
        ));
    }

    for part in parts {
        if part.parse::<u8>().is_err() {
            return Some(format!(
                "Invalid IPv4 address '{}': each octet must be 0-255",
                s
            ));
        }
    }

    None
}

/// Generate the endpoint! macro output
pub fn generate_endpoint_macro(input: EndpointInput) -> TokenStream {
    let endpoint_str = input.endpoint_str;
    let endpoint_value = endpoint_str.value();

    // Validate at compile time
    if let Some(error) = validate_endpoint(&endpoint_value) {
        return syn::Error::new(endpoint_str.span(), error)
            .to_compile_error()
            .into();
    }

    // Generate code that calls parse_endpoint at runtime (but we know it's valid)
    quote! {
        {
            // Validated at compile time by endpoint! macro
            const _ENDPOINT_STR: &str = #endpoint_str;
            ::horus_core::communication::network::parse_endpoint(_ENDPOINT_STR)
                .expect("endpoint! macro should have validated this")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_local() {
        assert!(validate_endpoint("mytopic").is_none());
        assert!(validate_endpoint("my_topic").is_none());
        assert!(validate_endpoint("robot/sensor").is_none());
        assert!(validate_endpoint("/ros2/topic").is_none());
    }

    #[test]
    fn test_validate_direct() {
        assert!(validate_endpoint("topic@192.168.1.1").is_none());
        assert!(validate_endpoint("topic@192.168.1.1:9000").is_none());
        assert!(validate_endpoint("topic@10.0.0.1:8080").is_none());
    }

    #[test]
    fn test_validate_special() {
        assert!(validate_endpoint("topic@*").is_none());
        assert!(validate_endpoint("topic@localhost").is_none());
        assert!(validate_endpoint("topic@router").is_none());
        assert!(validate_endpoint("topic@zenoh").is_none());
        assert!(validate_endpoint("topic@zenoh/ros2").is_none());
    }

    #[test]
    fn test_validate_mdns() {
        assert!(validate_endpoint("topic@robot.local").is_none());
        assert!(validate_endpoint("topic@my-robot.local").is_none());
        assert!(validate_endpoint("topic@robot01.local:9000").is_none());
    }

    #[test]
    fn test_validate_p2p() {
        assert!(validate_endpoint("topic@p2p:a3f7-k2m9-p4n8").is_none());
        assert!(validate_endpoint("topic@p2p:a3f7-k2m9-p4n8/direct").is_none());
        assert!(validate_endpoint("topic@p2p:a3f7-k2m9-p4n8/stun").is_none());
    }

    #[test]
    fn test_validate_cloud() {
        assert!(validate_endpoint("topic@cloud:my-room").is_none());
        assert!(validate_endpoint("topic@zenoh:cloud").is_none());
        assert!(validate_endpoint("topic@zenoh:cloud/auto").is_none());
    }

    #[test]
    fn test_validate_errors() {
        // Empty
        assert!(validate_endpoint("").is_some());

        // Empty topic
        assert!(validate_endpoint("@192.168.1.1").is_some());

        // Invalid characters
        assert!(validate_endpoint("topic with space").is_some());

        // Invalid P2P format
        assert!(validate_endpoint("topic@p2p:invalid").is_some());

        // Invalid IP
        assert!(validate_endpoint("topic@999.999.999.999").is_some());
    }
}
