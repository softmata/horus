/// Network communication utilities for HORUS
///
/// This module provides network-related utilities:
/// - Developer-friendly network error handling
/// - mDNS/DNS-SD node discovery (behind `mdns` feature)

// Developer-friendly network error handling
pub mod network_error;

// mDNS/DNS-SD for zero-config networking
#[cfg(feature = "mdns")]
pub mod mdns;

// Network error re-exports
pub use network_error::{NetworkError, NetworkErrorCode, NetworkErrorContext};

// mDNS re-exports
#[cfg(feature = "mdns")]
pub use mdns::{
    // Discovery API
    discover,
    discover_full,
    discover_full_with_options,
    discover_with_options,
    find_node,
    find_nodes_with_topic,
    nodes_to_json,
    // Core mDNS
    resolve_mdns_hostname,
    resolve_mdns_hostname_with_timeout,
    // Registration API
    to_json,
    watch,
    watch_with_interval,
    DiscoveredNode,
    DiscoveryEvent,
    DiscoveryOptions,
    DiscoveryResult,
    DiscoveryWatcher,
    GlobalMdnsManager,
    HorusMdns,
    MdnsCacheStats,
    MdnsNodeRegistration,
    MdnsRegistrationBuilder,
    MdnsRegistrationConfig,
    ServiceInfo as MdnsServiceInfo,
    BROWSE_TIMEOUT,
    DEFAULT_HORUS_PORT,
    HORUS_SERVICE_TYPE,
    MDNS_TIMEOUT,
};
