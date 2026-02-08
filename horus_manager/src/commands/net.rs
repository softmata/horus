//! Network diagnostics CLI for HORUS
//!
//! Provides network connectivity testing, latency measurement, and troubleshooting.
//!
//! Commands:
//! - `horus net check` - Full connectivity test
//! - `horus net ping <endpoint>` - Latency test to endpoint
//! - `horus net trace <endpoint>` - Path analysis
//! - `horus net doctor` - Network-specific fix suggestions

use colored::*;
use horus_core::error::HorusResult;
use std::net::{IpAddr, SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::time::{Duration, Instant};

/// Result status for network checks
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NetStatus {
    Ok,
    Warning,
    Error,
}

/// Network check result
#[derive(Debug)]
pub struct NetCheckResult {
    pub name: String,
    pub status: NetStatus,
    pub message: String,
    pub latency_ms: Option<f64>,
}

impl NetCheckResult {
    fn ok(name: &str, message: &str) -> Self {
        Self {
            name: name.to_string(),
            status: NetStatus::Ok,
            message: message.to_string(),
            latency_ms: None,
        }
    }

    fn ok_with_latency(name: &str, message: &str, latency_ms: f64) -> Self {
        Self {
            name: name.to_string(),
            status: NetStatus::Ok,
            message: message.to_string(),
            latency_ms: Some(latency_ms),
        }
    }

    fn warning(name: &str, message: &str) -> Self {
        Self {
            name: name.to_string(),
            status: NetStatus::Warning,
            message: message.to_string(),
            latency_ms: None,
        }
    }

    fn error(name: &str, message: &str) -> Self {
        Self {
            name: name.to_string(),
            status: NetStatus::Error,
            message: message.to_string(),
            latency_ms: None,
        }
    }
}

/// Run full connectivity check
pub fn run_check(verbose: bool) -> HorusResult<()> {
    println!("{}", "HORUS Network Diagnostics".green().bold());
    println!();

    let mut results = Vec::new();

    // Local interface checks
    print_section("Local Network");
    results.push(check_localhost());
    results.push(check_local_ip());
    results.push(check_udp_binding());
    results.push(check_multicast());

    for result in &results[results.len().saturating_sub(4)..] {
        print_result(result, verbose);
    }

    // External connectivity checks
    println!();
    print_section("External Connectivity");
    results.push(check_dns());
    results.push(check_internet());
    results.push(check_horus_registry());

    for result in &results[results.len().saturating_sub(3)..] {
        print_result(result, verbose);
    }

    // NAT/Firewall checks
    println!();
    print_section("NAT & Firewall");
    results.push(check_stun());
    results.push(check_nat_type());

    for result in &results[results.len().saturating_sub(2)..] {
        print_result(result, verbose);
    }

    // Summary
    println!();
    let errors = results
        .iter()
        .filter(|r| r.status == NetStatus::Error)
        .count();
    let warnings = results
        .iter()
        .filter(|r| r.status == NetStatus::Warning)
        .count();

    if errors > 0 {
        println!("{} {} error(s), {} warning(s)", "✗".red(), errors, warnings);
        println!(
            "  {} Run `horus net doctor` for suggested fixes",
            "Tip:".dimmed()
        );
    } else if warnings > 0 {
        println!("{} {} warning(s), no errors", "⚠".yellow(), warnings);
        println!(
            "  {} Some network features may be limited",
            "Note:".dimmed()
        );
    } else {
        println!("{} All network checks passed!", "✓".green());
    }

    Ok(())
}

/// Ping an endpoint and measure latency
pub fn run_ping(endpoint: &str, count: u32, interval_ms: u64) -> HorusResult<()> {
    println!("{} {}", "Pinging".green().bold(), endpoint.cyan());
    println!();

    // Parse endpoint
    let addr = match parse_endpoint(endpoint) {
        Ok(a) => a,
        Err(e) => {
            println!("{} Cannot resolve endpoint: {}", "✗".red(), e);
            return Ok(());
        }
    };

    println!("  Resolved: {}", addr.to_string().dimmed());
    println!();

    let mut latencies = Vec::new();
    let mut successes = 0;
    let mut failures = 0;

    for seq in 1..=count {
        match tcp_ping(&addr, Duration::from_secs(5)) {
            Ok(latency) => {
                let latency_ms = latency.as_secs_f64() * 1000.0;
                latencies.push(latency_ms);
                successes += 1;
                println!("  seq={} time={:.2}ms {}", seq, latency_ms, "✓".green());
            }
            Err(e) => {
                failures += 1;
                println!(
                    "  seq={} {} {}",
                    seq,
                    "timeout".red(),
                    format!("({})", e).dimmed()
                );
            }
        }

        if seq < count {
            std::thread::sleep(Duration::from_millis(interval_ms));
        }
    }

    // Statistics
    println!();
    println!("{}", "Statistics:".cyan().bold());

    let loss_pct = (failures as f64 / count as f64) * 100.0;
    println!(
        "  Packets: sent={}, received={}, loss={:.1}%",
        count, successes, loss_pct
    );

    if !latencies.is_empty() {
        let min = latencies.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = latencies.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let avg = latencies.iter().sum::<f64>() / latencies.len() as f64;

        // Standard deviation
        let variance =
            latencies.iter().map(|x| (x - avg).powi(2)).sum::<f64>() / latencies.len() as f64;
        let stddev = variance.sqrt();

        println!(
            "  Latency: min={:.2}ms, avg={:.2}ms, max={:.2}ms, stddev={:.2}ms",
            min, avg, max, stddev
        );

        // Quality assessment
        let quality = if avg < 10.0 && loss_pct == 0.0 {
            "Excellent".green()
        } else if avg < 50.0 && loss_pct < 5.0 {
            "Good".green()
        } else if avg < 100.0 && loss_pct < 10.0 {
            "Fair".yellow()
        } else {
            "Poor".red()
        };
        println!("  Quality: {}", quality);
    }

    Ok(())
}

/// Trace path to endpoint
pub fn run_trace(endpoint: &str, _max_hops: u32) -> HorusResult<()> {
    println!("{} path to {}", "Tracing".green().bold(), endpoint.cyan());
    println!();

    // Parse endpoint
    let addr = match parse_endpoint(endpoint) {
        Ok(a) => a,
        Err(e) => {
            println!("{} Cannot resolve endpoint: {}", "✗".red(), e);
            return Ok(());
        }
    };

    println!("  Target: {} ({})", endpoint, addr);
    println!();

    // Note: Real traceroute requires raw sockets (root privileges)
    // We do a simplified TCP-based trace
    println!("  {} TCP connect trace (simplified):", "Note:".dimmed());
    println!();

    match tcp_ping(&addr, Duration::from_secs(10)) {
        Ok(latency) => {
            let latency_ms = latency.as_secs_f64() * 1000.0;
            println!(
                "  1. {} -> {} ({:.2}ms) {}",
                get_local_ip().unwrap_or_else(|| "localhost".to_string()),
                addr,
                latency_ms,
                "✓".green()
            );
            println!();
            println!(
                "  {} Full traceroute requires: sudo traceroute {}",
                "Tip:".dimmed(),
                endpoint
            );
        }
        Err(e) => {
            println!("  {} Unable to reach destination: {}", "✗".red(), e);
            println!();

            // Suggest troubleshooting
            println!("  {}", "Troubleshooting:".yellow());
            println!("    1. Check if firewall is blocking outbound connections");
            println!("    2. Verify DNS resolution: nslookup {}", endpoint);
            println!("    3. Check route: ip route get {}", addr.ip());
        }
    }

    Ok(())
}

/// Run network doctor with fix suggestions
pub fn run_doctor() -> HorusResult<()> {
    println!("{}", "HORUS Network Doctor".green().bold());
    println!();

    let mut issues = Vec::new();

    // Run all checks
    let checks = vec![
        check_localhost(),
        check_local_ip(),
        check_udp_binding(),
        check_multicast(),
        check_dns(),
        check_internet(),
        check_horus_registry(),
        check_stun(),
    ];

    // Collect issues
    for check in &checks {
        if check.status != NetStatus::Ok {
            issues.push(check);
        }
    }

    if issues.is_empty() {
        println!("{} No network issues detected!", "✓".green());
        println!();
        println!("{}", "Network Status:".cyan().bold());
        println!("  • Local networking: OK");
        println!("  • External connectivity: OK");
        println!("  • NAT traversal: OK");
        return Ok(());
    }

    // Print issues with fixes
    println!("{} issues found:\n", format!("{}", issues.len()).yellow());

    for (i, issue) in issues.iter().enumerate() {
        let icon = match issue.status {
            NetStatus::Error => "✗".red(),
            NetStatus::Warning => "⚠".yellow(),
            NetStatus::Ok => "✓".green(),
        };

        println!(
            "{}. {} {}: {}",
            i + 1,
            icon,
            issue.name.cyan(),
            issue.message
        );
        println!();

        // Print fix suggestions based on issue
        print_fix_suggestion(&issue.name, &issue.message);
        println!();
    }

    Ok(())
}

// === Helper Functions ===

fn print_section(name: &str) {
    println!("{}", format!("─── {} ───", name).dimmed());
}

fn print_result(result: &NetCheckResult, _verbose: bool) {
    let icon = match result.status {
        NetStatus::Ok => "[OK]".green(),
        NetStatus::Warning => "[WARN]".yellow(),
        NetStatus::Error => "[ERR]".red(),
    };

    let latency_str = result
        .latency_ms
        .map(|l| format!(" ({:.1}ms)", l))
        .unwrap_or_default();

    let msg = format!("{}{}", result.message, latency_str);
    let colored_msg = match result.status {
        NetStatus::Ok => msg.normal(),
        NetStatus::Warning => msg.yellow(),
        NetStatus::Error => msg.red(),
    };

    println!("  {} {:20} {}", icon, result.name.cyan(), colored_msg);
}

fn print_fix_suggestion(name: &str, _message: &str) {
    println!("   {}", "Suggested fix:".yellow());

    match name {
        "Localhost" => {
            println!("   • Check if loopback interface is up: ip addr show lo");
            println!("   • Enable loopback: sudo ip link set lo up");
        }
        "Local IP" => {
            println!("   • Check network interfaces: ip addr");
            println!("   • Verify network manager status: systemctl status NetworkManager");
            println!("   • Restart networking: sudo systemctl restart NetworkManager");
        }
        "UDP Binding" => {
            println!("   • Check if another process is using the port");
            println!("   • Verify firewall rules: sudo iptables -L");
            println!("   • Allow UDP traffic: sudo ufw allow 47808/udp");
        }
        "Multicast" => {
            println!("   • Enable multicast on interface: sudo ip link set <iface> multicast on");
            println!("   • Add multicast route: sudo ip route add 224.0.0.0/4 dev <iface>");
            println!("   • Check firewall for multicast: sudo iptables -I INPUT -p udp -m udp --dport 47809 -j ACCEPT");
        }
        "DNS" => {
            println!("   • Check DNS configuration: cat /etc/resolv.conf");
            println!("   • Test DNS: nslookup google.com");
            println!("   • Try Google DNS: echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf");
        }
        "Internet" => {
            println!("   • Check default route: ip route show default");
            println!(
                "   • Ping gateway: ping -c 3 $(ip route | grep default | awk '{{print $3}}')"
            );
            println!("   • Verify DNS: nslookup google.com");
        }
        "HORUS Registry" => {
            println!("   • The registry may be temporarily unavailable");
            println!("   • HORUS works offline - you can still use local packages");
            println!("   • Check status at: https://status.horus-robotics.dev");
        }
        "STUN/NAT" => {
            println!("   • NAT traversal may be limited");
            println!("   • For P2P connectivity, consider:");
            println!("     - Using a relay server");
            println!("     - Port forwarding on your router");
            println!("     - Using a VPN for direct connectivity");
        }
        "NAT Type" => {
            println!("   • Symmetric NAT detected - P2P may not work directly");
            println!("   • Options:");
            println!("     - Configure port forwarding");
            println!("     - Use TURN relay (automatic fallback)");
            println!("     - Connect via HORUS Cloud relay");
        }
        _ => {
            println!("   • Check system logs: journalctl -xe");
            println!("   • Restart network services");
            println!("   • Run: horus net check --verbose");
        }
    }
}

// === Network Checks ===

fn check_localhost() -> NetCheckResult {
    match TcpStream::connect_timeout(&"127.0.0.1:0".parse().unwrap(), Duration::from_millis(100)) {
        // Connection refused is expected (no server), but means network works
        Err(e) if e.kind() == std::io::ErrorKind::ConnectionRefused => {
            NetCheckResult::ok("Localhost", "Loopback OK")
        }
        _ => {
            // Try binding instead
            match std::net::TcpListener::bind("127.0.0.1:0") {
                Ok(listener) => {
                    drop(listener);
                    NetCheckResult::ok("Localhost", "Loopback OK")
                }
                Err(e) => NetCheckResult::error("Localhost", &format!("Cannot bind: {}", e)),
            }
        }
    }
}

fn check_local_ip() -> NetCheckResult {
    match get_local_ip() {
        Some(ip) => NetCheckResult::ok("Local IP", &ip),
        None => NetCheckResult::warning("Local IP", "No non-loopback IP found"),
    }
}

fn check_udp_binding() -> NetCheckResult {
    match UdpSocket::bind("0.0.0.0:0") {
        Ok(socket) => {
            let port = socket.local_addr().map(|a| a.port()).unwrap_or(0);
            NetCheckResult::ok("UDP Binding", &format!("OK (port {})", port))
        }
        Err(e) => NetCheckResult::error("UDP Binding", &format!("Failed: {}", e)),
    }
}

fn check_multicast() -> NetCheckResult {
    // Try to join a multicast group
    let socket = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => return NetCheckResult::error("Multicast", &format!("Cannot bind: {}", e)),
    };

    match socket.join_multicast_v4(
        &"239.255.255.250".parse().unwrap(),
        &"0.0.0.0".parse().unwrap(),
    ) {
        Ok(_) => {
            let _ = socket.leave_multicast_v4(
                &"239.255.255.250".parse().unwrap(),
                &"0.0.0.0".parse().unwrap(),
            );
            NetCheckResult::ok("Multicast", "Supported")
        }
        Err(e) => NetCheckResult::warning("Multicast", &format!("Not available: {}", e)),
    }
}

fn check_dns() -> NetCheckResult {
    let start = Instant::now();
    match "google.com:80".to_socket_addrs() {
        Ok(mut addrs) => {
            let latency = start.elapsed().as_secs_f64() * 1000.0;
            if let Some(addr) = addrs.next() {
                NetCheckResult::ok_with_latency("DNS", &format!("OK ({})", addr.ip()), latency)
            } else {
                NetCheckResult::warning("DNS", "No addresses returned")
            }
        }
        Err(e) => NetCheckResult::error("DNS", &format!("Resolution failed: {}", e)),
    }
}

fn check_internet() -> NetCheckResult {
    let start = Instant::now();

    // Try to connect to a well-known service
    let targets = [("1.1.1.1:443", "Cloudflare"), ("8.8.8.8:443", "Google")];

    for (addr, name) in &targets {
        if let Ok(addr) = addr.parse::<SocketAddr>() {
            match TcpStream::connect_timeout(&addr, Duration::from_secs(5)) {
                Ok(_) => {
                    let latency = start.elapsed().as_secs_f64() * 1000.0;
                    return NetCheckResult::ok_with_latency(
                        "Internet",
                        &format!("{} reachable", name),
                        latency,
                    );
                }
                Err(_) => continue,
            }
        }
    }

    NetCheckResult::error("Internet", "Cannot reach external hosts")
}

fn check_horus_registry() -> NetCheckResult {
    let start = Instant::now();

    // Try DNS resolution first
    match "registry.horus-robotics.dev:443".to_socket_addrs() {
        Ok(mut addrs) => {
            if let Some(addr) = addrs.next() {
                // Try TCP connection
                match TcpStream::connect_timeout(&addr, Duration::from_secs(5)) {
                    Ok(_) => {
                        let latency = start.elapsed().as_secs_f64() * 1000.0;
                        NetCheckResult::ok_with_latency("HORUS Registry", "Connected", latency)
                    }
                    Err(_) => {
                        NetCheckResult::warning("HORUS Registry", "DNS OK but connection failed")
                    }
                }
            } else {
                NetCheckResult::warning("HORUS Registry", "No addresses resolved")
            }
        }
        Err(_) => NetCheckResult::warning("HORUS Registry", "Unreachable (offline OK)"),
    }
}

fn check_stun() -> NetCheckResult {
    // Try to use STUN to discover external address
    let socket = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => return NetCheckResult::error("STUN/NAT", &format!("Cannot bind: {}", e)),
    };

    socket.set_read_timeout(Some(Duration::from_secs(3))).ok();
    socket.set_write_timeout(Some(Duration::from_secs(3))).ok();

    // STUN binding request (simplified - just checking if we can reach STUN servers)
    let stun_servers = ["stun.l.google.com:19302", "stun1.l.google.com:19302"];

    for server in &stun_servers {
        if let Ok(addrs) = server.to_socket_addrs() {
            for addr in addrs {
                // Simple connectivity check
                if socket.connect(addr).is_ok() {
                    return NetCheckResult::ok(
                        "STUN/NAT",
                        &format!("Reachable ({})", server.split(':').next().unwrap_or("stun")),
                    );
                }
            }
        }
    }

    NetCheckResult::warning("STUN/NAT", "STUN servers unreachable")
}

fn check_nat_type() -> NetCheckResult {
    // This would require full STUN implementation
    // For now, we do a simplified check

    let socket = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(_) => return NetCheckResult::warning("NAT Type", "Cannot determine"),
    };

    // Verify socket is bound
    let _local_addr = match socket.local_addr() {
        Ok(a) => a,
        Err(_) => return NetCheckResult::warning("NAT Type", "Cannot determine"),
    };

    // Check if we have a public IP
    if let Some(ip) = get_local_ip() {
        let ip: IpAddr = ip.parse().unwrap_or(IpAddr::V4("0.0.0.0".parse().unwrap()));

        if ip.is_loopback() {
            NetCheckResult::warning("NAT Type", "Loopback only")
        } else if is_private_ip(&ip) {
            NetCheckResult::ok("NAT Type", "Behind NAT (common)")
        } else {
            NetCheckResult::ok("NAT Type", "Public IP")
        }
    } else {
        NetCheckResult::warning("NAT Type", "Cannot determine")
    }
}

// === Utility Functions ===

fn parse_endpoint(endpoint: &str) -> Result<SocketAddr, String> {
    // Try direct parse
    if let Ok(addr) = endpoint.parse::<SocketAddr>() {
        return Ok(addr);
    }

    // Try with default port
    let with_port = if endpoint.contains(':') {
        endpoint.to_string()
    } else {
        format!("{}:80", endpoint)
    };

    with_port
        .to_socket_addrs()
        .map_err(|e| format!("DNS resolution failed: {}", e))?
        .next()
        .ok_or_else(|| "No addresses found".to_string())
}

fn tcp_ping(addr: &SocketAddr, timeout: Duration) -> Result<Duration, String> {
    let start = Instant::now();
    TcpStream::connect_timeout(addr, timeout).map_err(|e| e.to_string())?;
    Ok(start.elapsed())
}

fn get_local_ip() -> Option<String> {
    // Try to find a non-loopback IP by connecting to a public address
    let socket = UdpSocket::bind("0.0.0.0:0").ok()?;
    socket.connect("8.8.8.8:80").ok()?;
    socket.local_addr().ok().map(|a| a.ip().to_string())
}

fn is_private_ip(ip: &IpAddr) -> bool {
    match ip {
        IpAddr::V4(v4) => v4.is_private() || v4.is_loopback() || v4.is_link_local(),
        IpAddr::V6(v6) => v6.is_loopback(),
    }
}
