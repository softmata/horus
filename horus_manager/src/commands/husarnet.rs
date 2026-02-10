//! Husarnet VPN integration commands for HORUS
//!
//! Provides CLI commands for:
//! - Checking Husarnet daemon status
//! - Listing Husarnet network peers
//! - Testing connectivity to peers
//! - Diagnosing Husarnet configuration issues

use colored::*;
use horus_core::error::{HorusError, HorusResult};
use horus_core::horus_internal;
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

/// Check if Husarnet daemon is running and accessible
pub fn run_status(verbose: bool) -> HorusResult<()> {
    println!("{}", "Husarnet VPN Status".cyan().bold());
    println!("{}", "═".repeat(50));

    // Check 1: Is husarnet daemon running?
    println!("\n{} Checking Husarnet daemon...", "●".cyan());

    let daemon_running = check_daemon_running();
    if daemon_running {
        println!("  {} Husarnet daemon is running", "✓".green());
    } else {
        println!("  {} Husarnet daemon is NOT running", "✗".red());
        println!("  {}", "Run: sudo systemctl start husarnet".yellow());
        return Ok(());
    }

    // Check 2: Query the API for status
    println!("\n{} Checking daemon API...", "●".cyan());

    match query_husarnet_api("/api/status") {
        Ok(status) => {
            println!("  {} API accessible", "✓".green());

            // Parse JSON once and reuse
            let json: Option<serde_json::Value> = serde_json::from_str(&status).ok();

            if verbose {
                println!("\n{}", "API Response:".cyan());
                // Pretty print the JSON
                if let Some(ref parsed) = json {
                    println!(
                        "{}",
                        serde_json::to_string_pretty(parsed).unwrap_or_else(|_| status.clone())
                    );
                } else {
                    println!("{}", status);
                }
            }

            // Extract key info
            if let Some(ref json) = json {
                if let Some(websetup_address) =
                    json.get("websetup_address").and_then(|v| v.as_str())
                {
                    println!("  Websetup address: {}", websetup_address.cyan());
                }
                if let Some(is_joined) = json.get("is_joined").and_then(|v| v.as_bool()) {
                    if is_joined {
                        println!("  {} Joined to a Husarnet network", "✓".green());
                    } else {
                        println!("  {} Not joined to any network", "⚠".yellow());
                        println!(
                            "  {}",
                            "Use 'husarnet join <joincode>' to join a network".yellow()
                        );
                    }
                }
            }
        }
        Err(e) => {
            println!("  {} API not accessible: {}", "✗".red(), e);
            println!(
                "  {}",
                "The daemon may be starting up, try again in a few seconds".yellow()
            );
        }
    }

    // Check 3: Check hnet0 interface
    println!("\n{} Checking hnet0 interface...", "●".cyan());

    match get_hnet0_address() {
        Some(addr) => {
            println!(
                "  {} hnet0 interface found: {}",
                "✓".green(),
                addr.to_string().cyan()
            );
        }
        None => {
            println!("  {} hnet0 interface not found", "⚠".yellow());
            println!(
                "  {}",
                "This is normal if not joined to a network yet".dimmed()
            );
        }
    }

    println!("\n{}", "═".repeat(50));
    Ok(())
}

/// List all Husarnet peers
pub fn run_peers(json: bool) -> HorusResult<()> {
    if !check_daemon_running() {
        return Err(HorusError::communication(
            "Husarnet daemon is not running".to_string(),
        ));
    }

    let peers = get_husarnet_peers()?;

    if json {
        let json_output = serde_json::to_string_pretty(&peers)
            .map_err(|e| horus_internal!("JSON serialization failed: {}", e))?;
        println!("{}", json_output);
        return Ok(());
    }

    println!("{}", "Husarnet Peers".cyan().bold());
    println!("{}", "═".repeat(60));

    if peers.is_empty() {
        println!("\n{}", "No peers found.".yellow());
        println!(
            "{}",
            "Peers appear when other devices join your Husarnet network.".dimmed()
        );
        return Ok(());
    }

    println!(
        "\n{:<20} {:<40} {}",
        "Hostname".bold(),
        "IPv6 Address".bold(),
        "Status".bold()
    );
    println!("{}", "─".repeat(60));

    for peer in &peers {
        let status_icon = if peer.is_active {
            "●".green()
        } else {
            "○".dimmed()
        };
        let status_text = if peer.is_active { "active" } else { "inactive" };

        println!(
            "{:<20} {:<40} {} {}",
            peer.hostname.cyan(),
            peer.ipv6.to_string(),
            status_icon,
            status_text
        );
    }

    println!("\n{} Total: {} peer(s)", "".cyan(), peers.len());
    Ok(())
}

/// Test connectivity to a specific peer or all peers
pub fn run_test(target: Option<String>, count: u32, timeout_ms: u64) -> HorusResult<()> {
    if !check_daemon_running() {
        return Err(HorusError::communication(
            "Husarnet daemon is not running".to_string(),
        ));
    }

    let peers = get_husarnet_peers()?;

    if peers.is_empty() {
        println!("{}", "No peers to test connectivity with.".yellow());
        return Ok(());
    }

    // If a specific target is given, filter to just that peer
    let test_peers: Vec<_> = match &target {
        Some(hostname) => {
            let matching: Vec<_> = peers
                .iter()
                .filter(|p| p.hostname.contains(hostname))
                .cloned()
                .collect();
            if matching.is_empty() {
                return Err(HorusError::communication(format!(
                    "No peer matching '{}' found",
                    hostname
                )));
            }
            matching
        }
        None => peers,
    };

    println!("{}", "Husarnet Connectivity Test".cyan().bold());
    println!("{}", "═".repeat(60));
    println!(
        "Testing {} peer(s) with {} packets each...\n",
        test_peers.len(),
        count
    );

    let timeout = Duration::from_millis(timeout_ms);

    for peer in &test_peers {
        print!("{:<20} ", peer.hostname.cyan());

        let mut successes = 0;
        let mut total_rtt = Duration::ZERO;
        let mut min_rtt = Duration::MAX;
        let mut max_rtt = Duration::ZERO;

        // Use HORUS default port for testing
        let target_addr = SocketAddr::new(IpAddr::V6(peer.ipv6), 9847);

        for _ in 0..count {
            if let Ok(rtt) = test_udp_connectivity(target_addr, timeout) {
                successes += 1;
                total_rtt += rtt;
                min_rtt = min_rtt.min(rtt);
                max_rtt = max_rtt.max(rtt);
            }
        }

        if successes > 0 {
            let avg_rtt = total_rtt / successes;
            println!(
                "{} {}/{} packets | avg={:.2}ms min={:.2}ms max={:.2}ms",
                "✓".green(),
                successes,
                count,
                avg_rtt.as_secs_f64() * 1000.0,
                min_rtt.as_secs_f64() * 1000.0,
                max_rtt.as_secs_f64() * 1000.0
            );
        } else {
            println!(
                "{} {}/{} packets | {}",
                "✗".red(),
                successes,
                count,
                "No response (peer may be offline or firewall blocking)".dimmed()
            );
        }
    }

    Ok(())
}

// ============================================================================
// Helper functions
// ============================================================================

/// Check if husarnet daemon is running by checking the API port
fn check_daemon_running() -> bool {
    // Try to connect to the Husarnet daemon API
    use std::net::TcpStream;
    TcpStream::connect_timeout(
        &"127.0.0.1:16216".parse().unwrap(),
        Duration::from_millis(500),
    )
    .is_ok()
}

/// Query the Husarnet daemon API
fn query_husarnet_api(endpoint: &str) -> HorusResult<String> {
    let url = format!("http://127.0.0.1:16216{}", endpoint);

    // Use reqwest blocking client for HTTP GET request
    let client = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(5))
        .build()
        .map_err(|e| HorusError::communication(format!("Failed to create HTTP client: {}", e)))?;

    let response = client
        .get(&url)
        .send()
        .map_err(|e| HorusError::communication(format!("API request failed: {}", e)))?;

    response
        .text()
        .map_err(|e| HorusError::communication(format!("Failed to read API response: {}", e)))
}

/// Get the hnet0 interface IPv6 address
fn get_hnet0_address() -> Option<std::net::Ipv6Addr> {
    #[cfg(unix)]
    {
        use std::process::Command;
        let output = Command::new("ip")
            .args(["-6", "addr", "show", "hnet0"])
            .output()
            .ok()?;

        let stdout = String::from_utf8_lossy(&output.stdout);
        for line in stdout.lines() {
            if line.contains("inet6") && line.contains("fc94:") {
                // Parse: "    inet6 fc94:xxxx::xxxx/128 scope global"
                let parts: Vec<&str> = line.split_whitespace().collect();
                if parts.len() >= 2 {
                    let addr_with_prefix = parts[1];
                    let addr = addr_with_prefix.split('/').next()?;
                    return addr.parse().ok();
                }
            }
        }
    }
    None
}

/// Peer info structure for JSON output
#[derive(Clone, serde::Serialize)]
pub struct PeerInfo {
    pub hostname: String,
    pub ipv6: std::net::Ipv6Addr,
    pub is_active: bool,
}

/// Get list of Husarnet peers from the daemon
fn get_husarnet_peers() -> HorusResult<Vec<PeerInfo>> {
    let status = query_husarnet_api("/api/status")?;
    let json: serde_json::Value = serde_json::from_str(&status)
        .map_err(|e| horus_internal!("Failed to parse API response: {}", e))?;

    let mut peers = Vec::new();

    // The peers are usually in json["peers"] as an array of objects
    // Structure may vary by Husarnet version, so we handle multiple formats
    if let Some(peer_list) = json.get("peers").and_then(|v| v.as_array()) {
        for peer in peer_list {
            if let (Some(hostname), Some(ipv6_str)) = (
                peer.get("hostname").and_then(|v| v.as_str()),
                peer.get("ipv6").and_then(|v| v.as_str()),
            ) {
                if let Ok(ipv6) = ipv6_str.parse() {
                    let is_active = peer
                        .get("is_active")
                        .and_then(|v| v.as_bool())
                        .unwrap_or(false);
                    peers.push(PeerInfo {
                        hostname: hostname.to_string(),
                        ipv6,
                        is_active,
                    });
                }
            }
        }
    }

    // Also check "host_table" format used by some versions
    if peers.is_empty() {
        if let Some(host_table) = json.get("host_table").and_then(|v| v.as_object()) {
            for (hostname, addr_value) in host_table {
                if let Some(ipv6_str) = addr_value.as_str() {
                    if let Ok(ipv6) = ipv6_str.parse() {
                        peers.push(PeerInfo {
                            hostname: hostname.clone(),
                            ipv6,
                            is_active: true, // Assume active if in host_table
                        });
                    }
                }
            }
        }
    }

    Ok(peers)
}

/// Test UDP connectivity to a peer
fn test_udp_connectivity(target: SocketAddr, timeout: Duration) -> HorusResult<Duration> {
    let socket = UdpSocket::bind("[::]:0")
        .map_err(|e| HorusError::communication(format!("Failed to bind socket: {}", e)))?;
    socket
        .set_read_timeout(Some(timeout))
        .map_err(|e| HorusError::communication(format!("Failed to set timeout: {}", e)))?;

    // Send a simple HORUS discovery ping
    let ping_data = b"HORUS_PING";
    let start = Instant::now();

    socket
        .send_to(ping_data, target)
        .map_err(|e| HorusError::communication(format!("Send failed: {}", e)))?;

    // Wait for response (may not always get one if peer doesn't respond to pings)
    let mut buf = [0u8; 64];
    match socket.recv_from(&mut buf) {
        Ok(_) => Ok(start.elapsed()),
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
            Err(HorusError::communication("Timeout".to_string()))
        }
        Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
            Err(HorusError::communication("Timeout".to_string()))
        }
        Err(e) => Err(HorusError::communication(format!("Recv failed: {}", e))),
    }
}
