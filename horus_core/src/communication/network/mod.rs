/// Network communication utilities for HORUS
///
/// This module provides network-related utilities:
/// - Developer-friendly network error handling
/// - Zenoh transport configuration (behind `zenoh-transport` feature)
/// - mDNS/DNS-SD node discovery (behind `mdns` feature)

// Developer-friendly network error handling
pub mod network_error;

// Zenoh configuration (used by communication::config for endpoint config parsing)
pub mod zenoh_config;

// mDNS/DNS-SD for zero-config networking
#[cfg(feature = "mdns")]
pub mod mdns;

// Network error re-exports
pub use network_error::{NetworkError, NetworkErrorCode, NetworkErrorContext};

// Zenoh config re-exports
pub use zenoh_config::{
    CongestionControl, ConnectionQualityState, Durability, HistoryPolicy, Liveliness, Reliability,
    SerializationFormat, ZenohCloudConfig, ZenohConfig, ZenohConnectionQuality, ZenohMode,
    ZenohQos, HORUS_CLOUD_ROUTER, HORUS_CLOUD_ROUTERS,
};

// Zenoh router discovery via mDNS (requires mdns feature)
#[cfg(feature = "mdns")]
pub use zenoh_config::discover_zenoh_routers_mdns;

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
