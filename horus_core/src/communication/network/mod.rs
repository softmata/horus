/// Network communication backends for HORUS
///
/// This module provides network-based communication in addition to the local
/// shared memory backend. It includes:
/// - Endpoint parsing for network addresses
/// - Binary protocol for efficient serialization
/// - UDP direct connections (no discovery)
/// - Unix domain sockets (localhost optimization)
/// - Multicast discovery
/// - Message batching for efficiency
/// - Congestion control with drop policies
/// - Compression (LZ4/Zstd)
/// - Query/Response patterns
/// - Topic caching with TTL
///
/// Network v2 high-performance backends:
/// - Batch UDP with sendmmsg/recvmmsg (200K+ packets/sec)
/// - Real io_uring zero-copy (3-5µs latency)
/// - QUIC transport (0-RTT, reliable)
/// - Smart transport selection (auto-picks best backend)
// Developer-friendly network error handling
pub mod network_error;

pub mod backend;
pub mod batching;
pub mod caching;
pub mod compression;
pub mod congestion;
pub mod direct;
pub mod discovery;
pub mod endpoint;
pub mod fragmentation;
pub mod protocol;
pub mod queryable;
pub mod reconnect;
pub mod router;
pub mod udp_multicast;

// Network v2 high-performance modules
pub mod batch_udp;
pub mod smart_copy;
pub mod smart_transport;

// Unix domain sockets are only available on Unix-like systems
#[cfg(unix)]
pub mod unix_socket;

#[cfg(feature = "tls")]
pub mod tls;

// Certificate management for auto-generation and persistence
#[cfg(feature = "tls")]
pub mod certificate_manager;

// QUIC transport (requires quic feature)
#[cfg(feature = "quic")]
pub mod quic;

// Husarnet VPN integration for WAN connectivity
#[cfg(feature = "husarnet")]
pub mod husarnet;

// Hybrid discovery (LAN multicast + Husarnet unicast)
pub mod hybrid_discovery;

// Connection health monitoring
pub mod health_monitor;

// Cloud connectivity patterns
pub mod cloud;

// io_uring backend (real implementation using io-uring crate, Linux only)
// Requires the `io-uring-net` feature flag
#[cfg(target_os = "linux")]
pub mod io_uring;

// mDNS/DNS-SD for zero-config networking
// Requires the `mdns` feature flag
#[cfg(feature = "mdns")]
pub mod mdns;

// Zenoh transport for multi-robot mesh, cloud connectivity, and ROS2 interop
// Requires the `zenoh-transport` feature flag
#[cfg(feature = "zenoh-transport")]
pub mod zenoh_backend;
pub mod zenoh_config;

// Hybrid backend - zero-copy local bypass combining SHM + Zenoh
pub mod hybrid_backend;

// Locality detection for Zenoh peers (same-process, same-machine, remote)
pub mod locality;

// Type introspection for Zenoh topics
pub mod zenoh_discovery;

// Testing utilities for ROS2 interoperability (mock nodes without ROS2 installation)
pub mod testing;

// Zenoh integration test harness (namespace isolation, parallel-safe)
pub mod test_harness;

// ROS2 protocol implementations moved to separate horus_ros2_bridge crate
// To use ROS2 services/actions/parameters, add horus_ros2_bridge to your Cargo.toml:
//
// [dependencies]
// horus_core = { version = "0.1.7", features = ["zenoh-transport"] }
// horus_ros2_bridge = "0.1.7"

// Re-export commonly used types
pub use backend::NetworkBackend;
pub use direct::{DirectBackend, DirectRole};
pub use discovery::{DiscoveryService, PeerInfo};
pub use endpoint::{
    parse_endpoint, Endpoint, EndpointBuilder, DEFAULT_PORT, MULTICAST_ADDR, MULTICAST_PORT,
};
pub use fragmentation::{Fragment, FragmentManager};
pub use protocol::{HorusPacket, MessageType};
pub use reconnect::{
    retry, retry_with_logging, CancellationToken, ConnectionHealth, ConnectionQuality,
    ConnectionState, ConnectionStateMachine, FailureReason, ReconnectContext, ReconnectStrategy,
    RetryCallback, RetryEvent, RetryExecutor, RetryResult, StateChangeCallback, StateTransition,
};
pub use router::RouterBackend;
pub use udp_multicast::UdpMulticastBackend;

// Re-export new modules
pub use batching::{
    AdaptiveBatcher, AdaptiveBatcherConfig, AdaptiveMode, BatchConfig, BatchReceiver,
    FrequencyTracker, FrequencyTrackerConfig, MessageBatch, MessageBatcher, SharedBatcher,
};
pub use caching::{CacheConfig, CacheStats, SharedCache, TopicCache};
pub use compression::{
    CompressedData, CompressedPacket, CompressionAlgo, CompressionConfig, Compressor,
};
pub use congestion::{
    CongestionConfig, CongestionController, CongestionResult, DropPolicy,
    SharedCongestionController,
};
pub use queryable::{
    QueryClient, QueryConfig, QueryError, QueryHandler, QueryRequest, QueryResponse, QueryServer,
    ResponseStatus,
};

#[cfg(unix)]
pub use unix_socket::UnixSocketBackend;

#[cfg(feature = "tls")]
pub use tls::{TlsCertConfig, TlsStream};

#[cfg(feature = "tls")]
pub use certificate_manager::{
    CertificateConfig, CertificateInfo, CertificateManager, CertificateManagerBuilder,
    CertificateStatus, DEFAULT_CERT_DIR, DEFAULT_CERT_FILE, DEFAULT_KEY_FILE,
};

// Network v2 re-exports
pub use batch_udp::{
    BatchUdpConfig, BatchUdpReceiver, BatchUdpSender, BatchUdpStats, ReceivedPacket,
    ScalableUdpBackend,
};
pub use smart_copy::{
    BufferPool, CopyStrategy, RegisteredBuffer, SmartCopyConfig, SmartCopySender, SmartCopyStats,
};
pub use smart_transport::{
    NetworkLocation, TransportBuilder, TransportPreferences, TransportSelector,
    TransportSelectorStats, TransportType,
};

// io_uring re-exports (real implementation)
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
pub use io_uring::{CompletionResult, RealIoUringBackend, RealIoUringConfig, RealIoUringStats};

#[cfg(all(feature = "quic", feature = "tls"))]
pub use quic::{get_or_create_cert, get_or_create_cert_with_config};
#[cfg(feature = "quic")]
pub use quic::{
    QuicBackend, QuicConfig, QuicCongestionControl, QuicQosProfile, QuicStats, QuicStreamPriority,
    QuicTransport,
};

// Hybrid discovery re-exports (LAN multicast + Husarnet unicast)
pub use hybrid_discovery::{
    DiscoverySource, DiscoverySummary, HybridDiscovery, HybridDiscoveryConfig, HybridPeerInfo,
};

// Health monitoring re-exports
pub use health_monitor::{
    AlertCallback, AlertType, HealthAlert, HealthLevel, HealthMonitor, HealthMonitorConfig,
    HealthStatus, HeartbeatRequest, HeartbeatResponse,
};

// Cloud re-exports
pub use cloud::{
    extract_original_topic, extract_room_name, is_room_scoped, parse_cloud_location,
    scope_topic_to_room, CloudConfig, CloudMode, CloudRoom, CloudStats, RoomError, RoomHandle,
    RoomRegistry, VpnType,
};

// mDNS re-exports (all from consolidated mdns module)
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

// Husarnet re-exports
#[cfg(feature = "husarnet")]
pub use husarnet::{
    get_hnet0_address, is_husarnet_address, HusarnetConfig, HusarnetDiscovery,
    HusarnetDiscoveryBuilder, HusarnetError, HusarnetPeer, HusarnetStatus, DEFAULT_HUSARNET_PORT,
    HUSARNET_API_HOST, HUSARNET_API_PORT, HUSARNET_PREFIX, PEER_REFRESH_INTERVAL,
};

// Zenoh re-exports
#[cfg(feature = "zenoh-transport")]
pub use zenoh_backend::{ZenohBackend, ZenohSessionInfo};
pub use zenoh_config::{
    CongestionControl, ConnectionQualityState, Durability, HistoryPolicy, Liveliness, Reliability,
    SerializationFormat, ZenohCloudConfig, ZenohConfig, ZenohConnectionQuality, ZenohMode,
    ZenohQos, HORUS_CLOUD_ROUTER, HORUS_CLOUD_ROUTERS,
};

// Zenoh router discovery via mDNS (requires mdns feature)
#[cfg(feature = "mdns")]
pub use zenoh_config::discover_zenoh_routers_mdns;

// Hybrid backend re-exports (SHM + Zenoh zero-copy local bypass)
#[cfg(feature = "zenoh-transport")]
pub use hybrid_backend::HybridBackend;
pub use hybrid_backend::{HybridConfig, HybridMode, HybridStats};

// Locality detection re-exports (for detecting same-process/machine peers)
pub use locality::{
    get_hostname, is_local_address, is_private_address, LocalityDetector, LocalityLevel,
    LocalityPeerInfo, LocalityStats,
};

// ROS2 services, actions, and parameters are now in the separate horus_ros2_bridge crate
// No re-exports here to avoid circular dependencies

// Network error re-exports - developer-friendly error handling
pub use network_error::{NetworkError, NetworkErrorCode, NetworkErrorContext};

// Testing utilities re-exports (mock ROS2 nodes without ROS2 installation)
pub use testing::{MockPublisher, MockRos2Node, MockScenarioBuilder, MockSubscriber, TopicStats};

// Zenoh test harness re-exports (namespace isolation, parallel-safe)
pub use test_harness::{
    zenoh_test_sync, zenoh_test_sync_with_config, ZenohTestConfig, ZenohTestContext, ZenohTestGuard,
};
