//! Developer-friendly network error handling for HORUS
//!
//! This module provides comprehensive error types for network operations with:
//! - Unique error codes for programmatic handling (H-NET-xxx)
//! - Suggested fixes for each error type
//! - Links to documentation
//! - Context preservation through error chains
//!
//! # Example
//! ```rust,ignore
//! use horus::communication::network::{NetworkError, NetworkErrorCode};
//!
//! // Errors include helpful suggestions
//! let err = NetworkError::connection_failed(
//!     "192.168.1.100:7447",
//!     "Connection refused",
//! );
//! println!("{}", err.help_text());
//! // Output: "Connection refused to 192.168.1.100:7447
//! //         Suggested fix: Check that the remote endpoint is running and accepting connections.
//! //         Try: horus net ping 192.168.1.100:7447"
//! ```

use std::fmt;
use std::time::Duration;

/// Unique error codes for HORUS network operations
///
/// Error codes follow the format: H-NET-XXX
/// - H: HORUS prefix
/// - NET: Network subsystem
/// - XXX: Specific error number
///
/// Error ranges:
/// - 001-099: Connection errors
/// - 100-199: DNS/Discovery errors
/// - 200-299: Transport errors
/// - 300-399: Protocol errors
/// - 400-499: NAT/Firewall errors
/// - 500-599: Configuration errors
/// - 600-699: Resource errors
/// - 700-799: Security errors
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NetworkErrorCode {
    // Connection errors (001-099)
    /// Connection to remote endpoint failed
    ConnectionFailed = 1,
    /// Connection timed out
    ConnectionTimeout = 2,
    /// Connection was refused by remote
    ConnectionRefused = 3,
    /// Connection was reset by peer
    ConnectionReset = 4,
    /// Unable to reach the network/host
    NetworkUnreachable = 5,
    /// Host is unreachable
    HostUnreachable = 6,
    /// Maximum retry attempts exceeded
    MaxRetriesExceeded = 7,
    /// Handshake failed during connection
    HandshakeFailed = 8,

    // DNS/Discovery errors (100-199)
    /// DNS resolution failed
    DnsResolutionFailed = 100,
    /// mDNS discovery failed
    MdnsDiscoveryFailed = 101,
    /// No peers found in discovery
    NoPeersFound = 102,
    /// Service not found in registry
    ServiceNotFound = 103,
    /// Invalid hostname format
    InvalidHostname = 104,

    // Transport errors (200-299)
    /// Failed to bind to local port
    PortBindFailed = 200,
    /// Port is already in use
    PortInUse = 201,
    /// Socket operation failed
    SocketError = 202,
    /// UDP multicast join failed
    MulticastJoinFailed = 203,
    /// Multicast not available on interface
    MulticastNotAvailable = 204,
    /// Message too large for transport
    MessageTooLarge = 205,
    /// Packet fragmentation failed
    FragmentationFailed = 206,
    /// Failed to reassemble fragments
    ReassemblyFailed = 207,
    /// Invalid transport protocol
    InvalidProtocol = 208,

    // Protocol errors (300-399)
    /// Invalid packet format
    InvalidPacket = 300,
    /// Protocol version mismatch
    ProtocolVersionMismatch = 301,
    /// Serialization failed
    SerializationFailed = 302,
    /// Deserialization failed
    DeserializationFailed = 303,
    /// Checksum/CRC mismatch
    ChecksumMismatch = 304,
    /// Unexpected message type
    UnexpectedMessage = 305,
    /// Message sequence error
    SequenceError = 306,

    // NAT/Firewall errors (400-499)
    /// NAT traversal failed
    NatTraversalFailed = 400,
    /// STUN binding failed
    StunBindingFailed = 401,
    /// TURN allocation failed
    TurnAllocationFailed = 402,
    /// Symmetric NAT detected (hard to traverse)
    SymmetricNatDetected = 403,
    /// ICE connectivity check failed
    IceCheckFailed = 404,
    /// Firewall blocking connection
    FirewallBlocked = 405,
    /// UPnP port mapping failed
    UpnpMappingFailed = 406,

    // Configuration errors (500-599)
    /// Invalid endpoint configuration
    InvalidEndpoint = 500,
    /// Invalid network configuration
    InvalidConfig = 501,
    /// Missing required configuration
    MissingConfig = 502,
    /// Conflicting configuration options
    ConflictingConfig = 503,
    /// Invalid QoS settings
    InvalidQos = 504,

    // Resource errors (600-699)
    /// System resource exhausted
    ResourceExhausted = 600,
    /// Too many open connections
    TooManyConnections = 601,
    /// Buffer overflow
    BufferOverflow = 602,
    /// Memory allocation failed
    AllocationFailed = 603,
    /// File descriptor limit reached
    FdLimitReached = 604,
    /// Shared memory error
    SharedMemoryError = 605,

    // Security errors (700-799)
    /// TLS/SSL handshake failed
    TlsHandshakeFailed = 700,
    /// Certificate verification failed
    CertificateInvalid = 701,
    /// Certificate expired
    CertificateExpired = 702,
    /// Authentication failed
    AuthenticationFailed = 703,
    /// Authorization denied
    AuthorizationDenied = 704,
    /// Encryption error
    EncryptionError = 705,

    // Unknown/Other (900+)
    /// Unknown error occurred
    Unknown = 999,
}

impl NetworkErrorCode {
    /// Get the full error code string (e.g., "H-NET-001")
    pub fn code(&self) -> String {
        format!("H-NET-{:03}", *self as u16)
    }

    /// Get the documentation URL for this error
    pub fn doc_url(&self) -> String {
        format!(
            "https://docs.horus.dev/errors/network/{}",
            self.code().to_lowercase()
        )
    }

    /// Get a brief description of the error category
    pub fn category(&self) -> &'static str {
        let code = *self as u16;
        match code {
            1..=99 => "Connection",
            100..=199 => "Discovery",
            200..=299 => "Transport",
            300..=399 => "Protocol",
            400..=499 => "NAT/Firewall",
            500..=599 => "Configuration",
            600..=699 => "Resource",
            700..=799 => "Security",
            _ => "Unknown",
        }
    }
}

impl fmt::Display for NetworkErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.code())
    }
}

/// A network error with rich context for debugging
///
/// NetworkError provides:
/// - A unique error code for programmatic handling
/// - A human-readable message
/// - Optional cause/source error
/// - Suggested fixes and CLI commands
/// - Documentation links
#[derive(Debug)]
pub struct NetworkError {
    /// The error code for this error
    pub code: NetworkErrorCode,
    /// Human-readable error message
    pub message: String,
    /// The endpoint involved (if any)
    pub endpoint: Option<String>,
    /// The underlying cause
    pub cause: Option<Box<dyn std::error::Error + Send + Sync>>,
    /// Suggested fix for this error
    pub suggestion: String,
    /// CLI command to help diagnose/fix
    pub cli_hint: Option<String>,
    /// Additional context fields
    pub context: Vec<(String, String)>,
}

impl NetworkError {
    /// Create a new network error with the given code and message
    pub fn new(code: NetworkErrorCode, message: impl Into<String>) -> Self {
        let msg = message.into();
        let suggestion = Self::default_suggestion(code);
        Self {
            code,
            message: msg,
            endpoint: None,
            cause: None,
            suggestion,
            cli_hint: Self::default_cli_hint(code),
            context: Vec::new(),
        }
    }

    /// Create a connection failed error
    pub fn connection_failed(endpoint: impl Into<String>, cause: impl Into<String>) -> Self {
        let ep = endpoint.into();
        Self {
            code: NetworkErrorCode::ConnectionFailed,
            message: format!("Failed to connect to {}", ep),
            endpoint: Some(ep.clone()),
            cause: None,
            suggestion: format!(
                "Check that the remote endpoint '{}' is running and accepting connections.",
                ep
            ),
            cli_hint: Some(format!("horus net ping {}", ep)),
            context: vec![("cause".to_string(), cause.into())],
        }
    }

    /// Create a connection timeout error
    pub fn connection_timeout(endpoint: impl Into<String>, duration: Duration) -> Self {
        let ep = endpoint.into();
        Self {
            code: NetworkErrorCode::ConnectionTimeout,
            message: format!("Connection to {} timed out after {:?}", ep, duration),
            endpoint: Some(ep.clone()),
            cause: None,
            suggestion: format!(
                "The endpoint '{}' may be overloaded or unreachable. Check network connectivity.",
                ep
            ),
            cli_hint: Some(format!("horus net check {}", ep)),
            context: vec![("timeout".to_string(), format!("{:?}", duration))],
        }
    }

    /// Create a DNS resolution failed error
    pub fn dns_failed(hostname: impl Into<String>, cause: impl Into<String>) -> Self {
        let host = hostname.into();
        Self {
            code: NetworkErrorCode::DnsResolutionFailed,
            message: format!("DNS resolution failed for '{}'", host),
            endpoint: Some(host.clone()),
            cause: None,
            suggestion: format!(
                "Check that '{}' is a valid hostname. Verify DNS settings or try using an IP address.",
                host
            ),
            cli_hint: Some(format!("nslookup {} && horus net doctor", host)),
            context: vec![("cause".to_string(), cause.into())],
        }
    }

    /// Create a port binding failed error
    pub fn port_bind_failed(port: u16, cause: impl Into<String>) -> Self {
        Self {
            code: NetworkErrorCode::PortBindFailed,
            message: format!("Failed to bind to port {}", port),
            endpoint: Some(format!("0.0.0.0:{}", port)),
            cause: None,
            suggestion: format!(
                "Port {} may be in use by another process or require elevated privileges.",
                port
            ),
            cli_hint: Some(format!(
                "lsof -i :{} && horus net check --local-ports",
                port
            )),
            context: vec![
                ("port".to_string(), port.to_string()),
                ("cause".to_string(), cause.into()),
            ],
        }
    }

    /// Create a multicast not available error
    pub fn multicast_not_available(interface: impl Into<String>) -> Self {
        let iface = interface.into();
        Self {
            code: NetworkErrorCode::MulticastNotAvailable,
            message: format!(
                "Multicast is not available on interface '{}'",
                iface
            ),
            endpoint: None,
            cause: None,
            suggestion: format!(
                "Interface '{}' does not support multicast. Try a different network interface or use direct connections.",
                iface
            ),
            cli_hint: Some("horus net doctor --check-multicast".to_string()),
            context: vec![("interface".to_string(), iface)],
        }
    }

    /// Create a NAT traversal failed error
    pub fn nat_traversal_failed(nat_type: impl Into<String>) -> Self {
        let nt = nat_type.into();
        Self {
            code: NetworkErrorCode::NatTraversalFailed,
            message: format!(
                "NAT traversal failed. Detected NAT type: {}",
                nt
            ),
            endpoint: None,
            cause: None,
            suggestion: format!(
                "Your network uses {} NAT which may block peer-to-peer connections. Consider using a TURN relay or cloud bridge.",
                nt
            ),
            cli_hint: Some("horus net doctor --check-nat".to_string()),
            context: vec![("nat_type".to_string(), nt)],
        }
    }

    /// Create a TLS handshake failed error
    pub fn tls_handshake_failed(endpoint: impl Into<String>, cause: impl Into<String>) -> Self {
        let ep = endpoint.into();
        Self {
            code: NetworkErrorCode::TlsHandshakeFailed,
            message: format!("TLS handshake failed with {}", ep),
            endpoint: Some(ep.clone()),
            cause: None,
            suggestion:
                "Check that certificates are valid and both sides support compatible TLS versions."
                    .to_string(),
            cli_hint: Some(format!("horus net check {} --tls", ep)),
            context: vec![("cause".to_string(), cause.into())],
        }
    }

    /// Create a protocol version mismatch error
    pub fn protocol_mismatch(
        local_version: impl Into<String>,
        remote_version: impl Into<String>,
    ) -> Self {
        let local = local_version.into();
        let remote = remote_version.into();
        Self {
            code: NetworkErrorCode::ProtocolVersionMismatch,
            message: format!(
                "Protocol version mismatch: local={}, remote={}",
                local, remote
            ),
            endpoint: None,
            cause: None,
            suggestion: format!(
                "Update HORUS to match the remote version. Local: {}, Remote: {}",
                local, remote
            ),
            cli_hint: Some("horus --version && horus net doctor".to_string()),
            context: vec![
                ("local_version".to_string(), local),
                ("remote_version".to_string(), remote),
            ],
        }
    }

    /// Create an invalid endpoint error
    pub fn invalid_endpoint(endpoint: impl Into<String>, reason: impl Into<String>) -> Self {
        let ep = endpoint.into();
        Self {
            code: NetworkErrorCode::InvalidEndpoint,
            message: format!("Invalid endpoint: {}", ep),
            endpoint: Some(ep.clone()),
            cause: None,
            suggestion: format!(
                "Endpoint '{}' is not valid. Use format: host:port, tcp://host:port, or unix:///path",
                ep
            ),
            cli_hint: Some("horus net check --help".to_string()),
            context: vec![("reason".to_string(), reason.into())],
        }
    }

    /// Create a resource exhausted error
    pub fn resource_exhausted(resource: impl Into<String>, limit: impl Into<String>) -> Self {
        let res = resource.into();
        let lim = limit.into();
        Self {
            code: NetworkErrorCode::ResourceExhausted,
            message: format!("{} limit reached: {}", res, lim),
            endpoint: None,
            cause: None,
            suggestion: format!(
                "System {} limit reached ({}). Consider reducing connections or increasing limits.",
                res, lim
            ),
            cli_hint: Some("ulimit -a && horus net doctor".to_string()),
            context: vec![("resource".to_string(), res), ("limit".to_string(), lim)],
        }
    }

    // Builder methods

    /// Add the underlying cause
    pub fn with_cause<E: std::error::Error + Send + Sync + 'static>(mut self, cause: E) -> Self {
        self.cause = Some(Box::new(cause));
        self
    }

    /// Add an endpoint context
    pub fn with_endpoint(mut self, endpoint: impl Into<String>) -> Self {
        self.endpoint = Some(endpoint.into());
        self
    }

    /// Add a custom suggestion
    pub fn with_suggestion(mut self, suggestion: impl Into<String>) -> Self {
        self.suggestion = suggestion.into();
        self
    }

    /// Add a CLI hint
    pub fn with_cli_hint(mut self, hint: impl Into<String>) -> Self {
        self.cli_hint = Some(hint.into());
        self
    }

    /// Add context field
    pub fn with_context(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.context.push((key.into(), value.into()));
        self
    }

    // Helper methods

    /// Get a formatted help text with suggestions and CLI hints
    pub fn help_text(&self) -> String {
        let mut text = format!("{}\n\n", self.message);
        text.push_str(&format!("Error code: {}\n", self.code));
        text.push_str(&format!("Suggested fix: {}\n", self.suggestion));

        if let Some(ref hint) = self.cli_hint {
            text.push_str(&format!("Try: {}\n", hint));
        }

        text.push_str(&format!("Documentation: {}\n", self.code.doc_url()));

        text
    }

    /// Get default suggestion for an error code
    fn default_suggestion(code: NetworkErrorCode) -> String {
        match code {
            NetworkErrorCode::ConnectionFailed => {
                "Check that the remote endpoint is running and accepting connections.".to_string()
            }
            NetworkErrorCode::ConnectionTimeout => {
                "The endpoint may be overloaded. Check network connectivity.".to_string()
            }
            NetworkErrorCode::ConnectionRefused => {
                "The remote service is not listening on this port.".to_string()
            }
            NetworkErrorCode::NetworkUnreachable => {
                "Check your network connection and routing table.".to_string()
            }
            NetworkErrorCode::DnsResolutionFailed => {
                "Verify the hostname is correct and DNS is working.".to_string()
            }
            NetworkErrorCode::PortBindFailed => {
                "The port may be in use. Try a different port or stop the conflicting process."
                    .to_string()
            }
            NetworkErrorCode::PortInUse => {
                "Another process is using this port. Use 'lsof -i :PORT' to find it.".to_string()
            }
            NetworkErrorCode::MulticastJoinFailed => {
                "Multicast may be disabled on this interface. Check network settings.".to_string()
            }
            NetworkErrorCode::NatTraversalFailed => {
                "Your NAT type may be too restrictive. Consider using a relay server.".to_string()
            }
            NetworkErrorCode::TlsHandshakeFailed => {
                "Check certificates and TLS configuration.".to_string()
            }
            NetworkErrorCode::CertificateInvalid => {
                "The certificate is invalid. Generate new certificates with 'horus net cert'."
                    .to_string()
            }
            NetworkErrorCode::ResourceExhausted => {
                "System resources are exhausted. Reduce connections or increase limits.".to_string()
            }
            _ => "Check the error message and documentation for more details.".to_string(),
        }
    }

    /// Get default CLI hint for an error code
    fn default_cli_hint(code: NetworkErrorCode) -> Option<String> {
        match code {
            NetworkErrorCode::ConnectionFailed
            | NetworkErrorCode::ConnectionTimeout
            | NetworkErrorCode::ConnectionRefused => Some("horus net check <endpoint>".to_string()),
            NetworkErrorCode::NetworkUnreachable | NetworkErrorCode::HostUnreachable => {
                Some("horus net trace <endpoint>".to_string())
            }
            NetworkErrorCode::DnsResolutionFailed => {
                Some("horus net doctor --check-dns".to_string())
            }
            NetworkErrorCode::PortBindFailed | NetworkErrorCode::PortInUse => {
                Some("horus net check --local-ports".to_string())
            }
            NetworkErrorCode::MulticastJoinFailed | NetworkErrorCode::MulticastNotAvailable => {
                Some("horus net doctor --check-multicast".to_string())
            }
            NetworkErrorCode::NatTraversalFailed
            | NetworkErrorCode::SymmetricNatDetected
            | NetworkErrorCode::StunBindingFailed => {
                Some("horus net doctor --check-nat".to_string())
            }
            NetworkErrorCode::TlsHandshakeFailed
            | NetworkErrorCode::CertificateInvalid
            | NetworkErrorCode::CertificateExpired => {
                Some("horus net doctor --check-tls".to_string())
            }
            _ => Some("horus net doctor".to_string()),
        }
    }
}

impl std::fmt::Display for NetworkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}] {}", self.code, self.message)?;
        if let Some(ref ep) = self.endpoint {
            write!(f, " (endpoint: {})", ep)?;
        }
        Ok(())
    }
}

impl std::error::Error for NetworkError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        self.cause
            .as_ref()
            .map(|e| e.as_ref() as &(dyn std::error::Error + 'static))
    }
}

// Conversion to HorusError
impl From<NetworkError> for crate::error::HorusError {
    fn from(err: NetworkError) -> Self {
        crate::error::HorusError::Communication(format!("{}", err))
    }
}

/// Extension trait for adding network error context to any error type
#[allow(clippy::result_large_err)]
pub trait NetworkErrorContext<T> {
    /// Wrap an error with network context
    fn network_context(
        self,
        code: NetworkErrorCode,
        message: impl Into<String>,
    ) -> Result<T, NetworkError>;

    /// Wrap with connection failed context
    fn connection_context(self, endpoint: impl Into<String>) -> Result<T, NetworkError>;
}

impl<T, E: std::error::Error + Send + Sync + 'static> NetworkErrorContext<T> for Result<T, E> {
    fn network_context(
        self,
        code: NetworkErrorCode,
        message: impl Into<String>,
    ) -> Result<T, NetworkError> {
        self.map_err(|e| NetworkError::new(code, message).with_cause(e))
    }

    fn connection_context(self, endpoint: impl Into<String>) -> Result<T, NetworkError> {
        self.map_err(|e| {
            let ep = endpoint.into();
            NetworkError::connection_failed(ep.clone(), e.to_string()).with_cause(e)
        })
    }
}

/// ANSI color codes for terminal output
pub mod colors {
    pub const RESET: &str = "\x1b[0m";
    pub const BOLD: &str = "\x1b[1m";
    pub const RED: &str = "\x1b[31m";
    pub const GREEN: &str = "\x1b[32m";
    pub const YELLOW: &str = "\x1b[33m";
    pub const BLUE: &str = "\x1b[34m";
    pub const CYAN: &str = "\x1b[36m";
    pub const DIM: &str = "\x1b[2m";
}

/// Create a formatted error report for display (plain text)
pub fn format_error_report(error: &NetworkError) -> String {
    format_error_report_impl(error, false)
}

/// Create a formatted error report with ANSI colors for terminal display
pub fn format_error_report_colored(error: &NetworkError) -> String {
    format_error_report_impl(error, true)
}

fn format_error_report_impl(error: &NetworkError, colored: bool) -> String {
    let mut report = String::new();

    // Color helpers
    let (red, yellow, green, cyan, blue, bold, dim, reset) = if colored {
        (
            colors::RED,
            colors::YELLOW,
            colors::GREEN,
            colors::CYAN,
            colors::BLUE,
            colors::BOLD,
            colors::DIM,
            colors::RESET,
        )
    } else {
        ("", "", "", "", "", "", "", "")
    };

    // Header with error code
    report.push_str(&format!(
        "{}╭─────────────────────────────────────────────────────────────────╮{}\n",
        red, reset
    ));
    report.push_str(&format!(
        "{}│{} {}HORUS Network Error:{} {}{:41}{} {}│{}\n",
        red,
        reset,
        bold,
        reset,
        red,
        error.code.code(),
        reset,
        red,
        reset
    ));
    report.push_str(&format!(
        "{}├─────────────────────────────────────────────────────────────────┤{}\n",
        red, reset
    ));

    // Error message - wrap long messages
    let msg_lines = wrap_text(&error.message, 63);
    for line in msg_lines {
        report.push_str(&format!(
            "{}│{} {:63} {}│{}\n",
            red, reset, line, red, reset
        ));
    }

    // Endpoint if present
    if let Some(ref ep) = error.endpoint {
        report.push_str(&format!(
            "{}│{} {}Endpoint:{} {:54} {}│{}\n",
            red, reset, dim, reset, ep, red, reset
        ));
    }

    // Context fields
    if !error.context.is_empty() {
        report.push_str(&format!(
            "{}├─────────────────────────────────────────────────────────────────┤{}\n",
            dim, reset
        ));
        for (key, value) in &error.context {
            let line = format!("{}: {}", key, value);
            report.push_str(&format!(
                "{}│{} {:63} {}│{}\n",
                dim, reset, line, dim, reset
            ));
        }
    }

    // Suggestion section
    report.push_str(&format!(
        "{}├─────────────────────────────────────────────────────────────────┤{}\n",
        yellow, reset
    ));
    let suggestion_lines = wrap_text(&error.suggestion, 60);
    for (i, line) in suggestion_lines.iter().enumerate() {
        let prefix = if i == 0 { "💡" } else { "  " };
        report.push_str(&format!(
            "{}│{} {} {}{:60}{} {}│{}\n",
            yellow, reset, prefix, green, line, reset, yellow, reset
        ));
    }

    // CLI hint
    if let Some(ref hint) = error.cli_hint {
        report.push_str(&format!(
            "{}│{} 🔧 Try: {}{:55}{} {}│{}\n",
            yellow, reset, cyan, hint, reset, yellow, reset
        ));
    }

    // Documentation link
    report.push_str(&format!(
        "{}├─────────────────────────────────────────────────────────────────┤{}\n",
        blue, reset
    ));
    report.push_str(&format!(
        "{}│{} 📖 {}{:60}{} {}│{}\n",
        blue,
        reset,
        blue,
        error.code.doc_url(),
        reset,
        blue,
        reset
    ));
    report.push_str(&format!(
        "{}╰─────────────────────────────────────────────────────────────────╯{}\n",
        blue, reset
    ));

    report
}

/// Wrap text to fit within a given width
fn wrap_text(text: &str, max_width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut current_line = String::new();

    for word in text.split_whitespace() {
        if current_line.is_empty() {
            current_line = word.to_string();
        } else if current_line.len() + 1 + word.len() <= max_width {
            current_line.push(' ');
            current_line.push_str(word);
        } else {
            lines.push(current_line);
            current_line = word.to_string();
        }
    }

    if !current_line.is_empty() {
        lines.push(current_line);
    }

    if lines.is_empty() {
        lines.push(String::new());
    }

    lines
}

/// Quick constructors for common errors
pub mod errors {
    use super::*;

    /// Connection was refused by the remote endpoint
    pub fn connection_refused(endpoint: impl Into<String>) -> NetworkError {
        let ep = endpoint.into();
        NetworkError::new(
            NetworkErrorCode::ConnectionRefused,
            format!("Connection refused by {}", ep),
        )
        .with_endpoint(ep.clone())
        .with_suggestion(format!(
            "The service at '{}' is not accepting connections. Ensure the service is running.",
            ep
        ))
        .with_cli_hint(format!("horus net ping {}", ep))
    }

    /// Network is unreachable
    pub fn network_unreachable(destination: impl Into<String>) -> NetworkError {
        let dest = destination.into();
        NetworkError::new(
            NetworkErrorCode::NetworkUnreachable,
            format!("Network unreachable: {}", dest),
        )
        .with_endpoint(dest.clone())
        .with_suggestion("Check your network connection and routing configuration.")
        .with_cli_hint(format!("horus net trace {}", dest))
    }

    /// Port already in use
    pub fn port_in_use(port: u16) -> NetworkError {
        NetworkError::new(
            NetworkErrorCode::PortInUse,
            format!("Port {} is already in use", port),
        )
        .with_context("port", port.to_string())
        .with_suggestion(format!(
            "Port {} is being used by another process. Use 'lsof -i :{}' to identify it.",
            port, port
        ))
        .with_cli_hint(format!(
            "lsof -i :{} && horus net check --local-ports",
            port
        ))
    }

    /// Invalid configuration
    pub fn invalid_config(field: impl Into<String>, reason: impl Into<String>) -> NetworkError {
        let f = field.into();
        let r = reason.into();
        NetworkError::new(
            NetworkErrorCode::InvalidConfig,
            format!("Invalid configuration for '{}': {}", f, r),
        )
        .with_context("field", f.clone())
        .with_context("reason", r)
        .with_suggestion(format!(
            "Check the configuration for '{}' in your horus.toml file.",
            f
        ))
    }

    /// Symmetric NAT detected
    pub fn symmetric_nat() -> NetworkError {
        NetworkError::new(
            NetworkErrorCode::SymmetricNatDetected,
            "Symmetric NAT detected - peer-to-peer may not work",
        )
        .with_suggestion(
            "Symmetric NAT makes direct P2P difficult. Use 'cloud://' or a TURN relay instead.",
        )
        .with_cli_hint("horus net doctor --check-nat")
    }

    /// Too many connections
    pub fn too_many_connections(current: usize, max: usize) -> NetworkError {
        NetworkError::new(
            NetworkErrorCode::TooManyConnections,
            format!("Too many connections: {} (max: {})", current, max),
        )
        .with_context("current", current.to_string())
        .with_context("max", max.to_string())
        .with_suggestion("Close unused connections or increase the connection limit.")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_code_format() {
        assert_eq!(NetworkErrorCode::ConnectionFailed.code(), "H-NET-001");
        assert_eq!(NetworkErrorCode::DnsResolutionFailed.code(), "H-NET-100");
        assert_eq!(NetworkErrorCode::PortBindFailed.code(), "H-NET-200");
        assert_eq!(NetworkErrorCode::InvalidPacket.code(), "H-NET-300");
        assert_eq!(NetworkErrorCode::NatTraversalFailed.code(), "H-NET-400");
        assert_eq!(NetworkErrorCode::InvalidEndpoint.code(), "H-NET-500");
        assert_eq!(NetworkErrorCode::ResourceExhausted.code(), "H-NET-600");
        assert_eq!(NetworkErrorCode::TlsHandshakeFailed.code(), "H-NET-700");
    }

    #[test]
    fn test_error_categories() {
        assert_eq!(NetworkErrorCode::ConnectionFailed.category(), "Connection");
        assert_eq!(
            NetworkErrorCode::DnsResolutionFailed.category(),
            "Discovery"
        );
        assert_eq!(NetworkErrorCode::PortBindFailed.category(), "Transport");
        assert_eq!(NetworkErrorCode::InvalidPacket.category(), "Protocol");
        assert_eq!(
            NetworkErrorCode::NatTraversalFailed.category(),
            "NAT/Firewall"
        );
        assert_eq!(
            NetworkErrorCode::InvalidEndpoint.category(),
            "Configuration"
        );
        assert_eq!(NetworkErrorCode::ResourceExhausted.category(), "Resource");
        assert_eq!(NetworkErrorCode::TlsHandshakeFailed.category(), "Security");
    }

    #[test]
    fn test_connection_failed_error() {
        let err = NetworkError::connection_failed("192.168.1.1:7447", "Connection refused");
        assert_eq!(err.code, NetworkErrorCode::ConnectionFailed);
        assert!(err.message.contains("192.168.1.1:7447"));
        assert!(err.suggestion.contains("running"));
        assert!(err.cli_hint.is_some());
    }

    #[test]
    fn test_error_display() {
        let err = NetworkError::new(NetworkErrorCode::ConnectionTimeout, "Test timeout");
        let display = format!("{}", err);
        assert!(display.contains("H-NET-002"));
        assert!(display.contains("Test timeout"));
    }

    #[test]
    fn test_help_text() {
        let err = NetworkError::connection_failed("localhost:8080", "refused");
        let help = err.help_text();
        assert!(help.contains("H-NET-001"));
        assert!(help.contains("Suggested fix"));
        assert!(help.contains("Try:"));
        assert!(help.contains("Documentation"));
    }

    #[test]
    fn test_error_with_context() {
        let err = NetworkError::new(NetworkErrorCode::SocketError, "Failed")
            .with_endpoint("localhost:1234")
            .with_context("socket_type", "TCP")
            .with_context("errno", "98");

        assert!(err.endpoint.is_some());
        assert_eq!(err.context.len(), 2);
    }

    #[test]
    fn test_format_error_report() {
        let err = NetworkError::connection_failed("10.0.0.1:7447", "Connection timed out");
        let report = format_error_report(&err);
        assert!(report.contains("H-NET-001"));
        assert!(report.contains("10.0.0.1:7447"));
        assert!(report.contains("💡"));
    }

    #[test]
    fn test_quick_constructors() {
        let err = errors::connection_refused("localhost:8080");
        assert_eq!(err.code, NetworkErrorCode::ConnectionRefused);

        let err = errors::port_in_use(8080);
        assert_eq!(err.code, NetworkErrorCode::PortInUse);

        let err = errors::symmetric_nat();
        assert_eq!(err.code, NetworkErrorCode::SymmetricNatDetected);
    }

    #[test]
    fn test_colored_error_report() {
        let err = NetworkError::connection_failed("10.0.0.1:7447", "Connection timed out");
        let colored_report = format_error_report_colored(&err);

        // Should contain ANSI color codes
        assert!(colored_report.contains("\x1b["));
        // Should still contain the content
        assert!(colored_report.contains("H-NET-001"));
        assert!(colored_report.contains("10.0.0.1:7447"));
    }

    #[test]
    fn test_wrap_text() {
        let long_text = "This is a very long message that should be wrapped across multiple lines for better readability in the terminal output";
        let lines = wrap_text(long_text, 30);
        assert!(lines.len() > 1);
        for line in &lines {
            assert!(line.len() <= 30 || !line.contains(' '));
        }
    }

    #[test]
    fn test_error_conversion_to_horus_error() {
        let net_err = NetworkError::connection_failed("localhost:8080", "refused");
        let horus_err: crate::error::HorusError = net_err.into();

        match horus_err {
            crate::error::HorusError::Communication(msg) => {
                assert!(msg.contains("H-NET-001"));
            }
            _ => panic!("Expected Communication error"),
        }
    }
}
