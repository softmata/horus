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
pub mod udp_direct;
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

// WebRTC data channels (requires webrtc-transport feature)
#[cfg(feature = "webrtc-transport")]
pub mod webrtc;

// Built-in signaling server for P2P connection coordination
#[cfg(feature = "signaling-server")]
pub mod signaling_server;

// Signaling server re-exports
#[cfg(feature = "signaling-server")]
pub use signaling_server::{
    SignalingServer, SignalingServerBuilder, SignalingServerConfig, SignalingServerStats,
    SignalingStatsSnapshot, WireIceCandidate, WireMessage, DEFAULT_SIGNAL_PORT,
};

// Husarnet VPN integration for WAN connectivity
#[cfg(feature = "husarnet")]
pub mod husarnet;

// P2P connectivity with NAT traversal
pub mod p2p;

// Hybrid discovery (LAN multicast + Husarnet unicast)
pub mod hybrid_discovery;

// STUN client for NAT traversal
pub mod stun;

// ICE-lite for NAT hole punching
pub mod ice;

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
#[cfg(feature = "mdns")]
pub mod mdns_discovery;
#[cfg(feature = "mdns")]
pub mod mdns_registration;

// Zenoh transport for multi-robot mesh, cloud connectivity, and ROS2 interop
// Requires the `zenoh-transport` feature flag
#[cfg(feature = "zenoh-transport")]
pub mod zenoh_backend;
pub mod zenoh_config;

// ROS2 service protocol for Zenoh (request/response over rq/rs topics)
// Requires the `zenoh-transport` feature flag
#[cfg(feature = "zenoh-transport")]
pub mod zenoh_ros2_services;

// ROS2 action protocol for Zenoh (goal/cancel/result/feedback/status topics)
// Requires the `zenoh-transport` feature flag
#[cfg(feature = "zenoh-transport")]
pub mod zenoh_ros2_actions;

// ROS2 parameter server protocol for Zenoh (list/get/set/describe parameters)
// Requires the `zenoh-transport` feature flag
#[cfg(feature = "zenoh-transport")]
pub mod zenoh_ros2_params;

// Re-export commonly used types
pub use backend::NetworkBackend;
pub use direct::{DirectBackend, DirectRole};
pub use discovery::{DiscoveryService, PeerInfo};
pub use endpoint::{parse_endpoint, Endpoint, DEFAULT_PORT, MULTICAST_ADDR, MULTICAST_PORT};
pub use fragmentation::{Fragment, FragmentManager};
pub use protocol::{HorusPacket, MessageType};
pub use reconnect::{
    retry, retry_with_logging, CancellationToken, ConnectionHealth, ConnectionQuality,
    ConnectionState, ConnectionStateMachine, FailureReason, ReconnectContext, ReconnectStrategy,
    RetryCallback, RetryEvent, RetryExecutor, RetryResult, StateChangeCallback, StateTransition,
};
pub use router::RouterBackend;
pub use udp_direct::UdpDirectBackend;
pub use udp_multicast::UdpMulticastBackend;

// Re-export new modules
pub use batching::{BatchConfig, BatchReceiver, MessageBatch, MessageBatcher, SharedBatcher};
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
pub use io_uring::{
    is_real_io_uring_available, is_sqpoll_available, CompletionResult, RealIoUringBackend,
    RealIoUringConfig, RealIoUringStats,
};

#[cfg(feature = "quic")]
pub use quic::{
    generate_self_signed_cert, QuicBackend, QuicConfig, QuicCongestionControl, QuicQosProfile,
    QuicStats, QuicStreamPriority, QuicTransport,
};


#[cfg(all(feature = "quic", feature = "tls"))]
pub use quic::{get_or_create_cert, get_or_create_cert_with_config};

// WebRTC re-exports
#[cfg(feature = "webrtc-transport")]
pub use webrtc::{
    parse_webrtc_location, DataChannelHandle, DataChannelMode, IceServerConfig,
    WebRtcConfig, WebRtcSignal, WebRtcStats, WebRtcTransport,
};

// P2P re-exports
pub use p2p::{
    parse_p2p_location, IceCandidate, IceCandidateType, IceProtocol, P2pConfig,
    P2pConnectionResult, P2pConnectionState, P2pStats, P2pStrategy, PeerId,
    SignalingClientConfig, SignalingErrorCode, SignalingMessage, TurnServer,
};

// Hybrid discovery re-exports (LAN multicast + Husarnet unicast)
pub use hybrid_discovery::{
    DiscoverySource, DiscoverySummary, HybridDiscovery, HybridDiscoveryConfig, HybridPeerInfo,
};

// STUN re-exports
pub use stun::{
    discover_external_address, full_stun_discovery, NatType, StunClient, StunConfig,
    StunDiscoveryResult,
};

// ICE re-exports
pub use ice::{
    CandidatePair, CandidatePairState, IceAgent, IceCandidateInfo, IceConfig,
    IceConnectionState, IceGatheringState, IceRole, IceStats, ICE_CONNECTIVITY_CHECK_INTERVAL,
    ICE_CONSENT_INTERVAL, ICE_DEFAULT_TIMEOUT, ICE_KEEPALIVE_INTERVAL,
};

// Cloud re-exports
pub use cloud::{
    parse_cloud_location, CloudConfig, CloudMode, CloudRoom, CloudStats, VpnType,
};

// mDNS re-exports
#[cfg(feature = "mdns")]
pub use mdns::{
    resolve_mdns_hostname, resolve_mdns_hostname_with_timeout, HorusMdns, MdnsCacheStats,
    ServiceInfo as MdnsServiceInfo, BROWSE_TIMEOUT, HORUS_SERVICE_TYPE, MDNS_TIMEOUT,
};
#[cfg(feature = "mdns")]
pub use mdns_discovery::{
    discover, discover_full, discover_full_with_options, discover_with_options, find_node,
    find_nodes_with_topic, nodes_to_json, to_json, watch, watch_with_interval, DiscoveredNode,
    DiscoveryEvent, DiscoveryOptions, DiscoveryResult, DiscoveryWatcher,
};
#[cfg(feature = "mdns")]
pub use mdns_registration::{
    sanitize_hostname, GlobalMdnsManager, MdnsNodeRegistration, MdnsRegistrationBuilder,
    MdnsRegistrationConfig, DEFAULT_HORUS_PORT,
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

// ROS2 services re-exports
#[cfg(feature = "zenoh-transport")]
pub use zenoh_ros2_services::{
    parse_service_topic, service_to_request_topic, service_to_response_topic, EmptyRequest,
    EmptyResponse, RequestTracker, Ros2RequestHeader, Ros2ServiceConfig, Ros2ServiceError,
    Ros2ServiceRequest, Ros2ServiceResponse, ServiceRegistry, ServiceStats, SetBoolRequest,
    SetBoolResponse, TriggerRequest, TriggerResponse,
};

// ROS2 actions re-exports
#[cfg(feature = "zenoh-transport")]
pub use zenoh_ros2_actions::{
    action_to_cancel_goal_topic,
    action_to_feedback_topic,
    action_to_get_result_topic,
    // Topic naming
    action_to_send_goal_topic,
    action_to_status_topic,
    apply_namespace_to_action,
    parse_action_topic,
    ActionClientTopics,
    ActionServerTopics,
    ActionStats,
    ActionTopicType,
    // Trait
    ActionType,
    CancelCallback,
    CancelGoalErrorCode,
    CancelGoalRequest,
    CancelGoalResponse,
    // Goal handles
    ClientGoalHandle,
    EmptyFeedback,
    // Common action types
    EmptyGoal,
    EmptyResult,
    ExecuteCallback,
    FeedbackMessage,
    GetResultRequest,
    GetResultResponse,
    GoalCallback,
    // Core types
    GoalId,
    GoalInfo,
    GoalStatus,
    GoalStatusArray,
    GoalStatusInfo,
    ProgressFeedback,
    // Client/Server
    Ros2ActionClient,
    Ros2ActionConfig,
    Ros2ActionError,
    Ros2ActionServer,
    Ros2Time,
    // Request/Response types
    SendGoalRequest,
    SendGoalResponse,
    ServerGoalHandle,
};

// Network error re-exports - developer-friendly error handling
pub use network_error::{
    colors as net_colors, errors as net_errors, format_error_report, format_error_report_colored,
    NetworkError, NetworkErrorCode, NetworkErrorContext,
};

// ROS2 parameters re-exports
#[cfg(feature = "zenoh-transport")]
pub use zenoh_ros2_params::{
    param_to_describe_parameters_topic,
    param_to_events_topic,
    param_to_get_parameter_types_topic,
    param_to_get_parameters_topic,
    // Topic naming
    param_to_list_parameters_topic,
    param_to_set_parameters_atomically_topic,
    param_to_set_parameters_topic,
    DescribeParametersRequest,
    DescribeParametersResponse,
    FloatingPointRange,
    GetParameterTypesRequest,
    GetParameterTypesResponse,
    GetParametersRequest,
    GetParametersResponse,
    IntegerRange,
    // Request/Response types
    ListParametersRequest,
    ListParametersResponse,
    ListParametersResult,
    // Store and Client/Server
    LocalParameterStore,
    Parameter,
    ParameterClientTopics,
    ParameterDescriptor,
    ParameterEvent,
    ParameterEventType,
    ParameterQos,
    ParameterServerTopics,
    ParameterStats,
    ParameterTopicType,
    // Core types
    ParameterType,
    ParameterValue,
    Ros2ParamTime,
    Ros2ParameterClient,
    Ros2ParameterConfig,
    Ros2ParameterError,
    Ros2ParameterServer,
    SetParameterResult,
    SetParametersAtomicallyRequest,
    SetParametersAtomicallyResponse,
    SetParametersRequest,
    SetParametersResponse,
};
