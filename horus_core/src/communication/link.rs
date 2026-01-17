use crate::communication::network::smart_transport::NetworkLocation;
use crate::core::node::NodeInfo;
use crate::error::HorusResult;
use crate::memory::shm_region::ShmRegion;
use crate::memory::simd::{prefetch, PrefetchHint};
use std::marker::PhantomData;
use std::mem;
use std::net::{SocketAddr, UdpSocket};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Instant;

#[cfg(unix)]
use std::os::unix::net::UnixDatagram;

/// Branch prediction hint: this condition is unlikely
/// Helps CPU predict the common path (not full, has data)
#[inline(always)]
fn unlikely(b: bool) -> bool {
    // Use core::intrinsics::unlikely when stable, for now use cold hint
    #[cold]
    #[inline(never)]
    fn cold_path() {}

    if b {
        cold_path();
    }
    b
}

/// Link role - determines whether this end can send or receive
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkRole {
    Producer,
    Consumer,
}

/// Connection state for Link connections
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Failed,
}

// Helper functions for state conversion
impl ConnectionState {
    fn into_u8(self) -> u8 {
        match self {
            ConnectionState::Disconnected => 0,
            ConnectionState::Connecting => 1,
            ConnectionState::Connected => 2,
            ConnectionState::Reconnecting => 3,
            ConnectionState::Failed => 4,
        }
    }

    fn from_u8(value: u8) -> Self {
        match value {
            0 => ConnectionState::Disconnected,
            1 => ConnectionState::Connecting,
            2 => ConnectionState::Connected,
            3 => ConnectionState::Reconnecting,
            _ => ConnectionState::Failed,
        }
    }
}

/// Metrics for Link monitoring
#[derive(Debug, Clone, Default)]
pub struct LinkMetrics {
    pub messages_sent: u64,
    pub messages_received: u64,
    pub send_failures: u64,
    pub recv_failures: u64,
}

/// Lock-free atomic metrics for Link monitoring (stored in local memory)
#[derive(Debug)]
#[repr(align(64))] // Cache-line aligned to prevent false sharing
struct AtomicLinkMetrics {
    messages_sent: std::sync::atomic::AtomicU64,
    messages_received: std::sync::atomic::AtomicU64,
    send_failures: std::sync::atomic::AtomicU64,
    recv_failures: std::sync::atomic::AtomicU64,
    _padding: [u8; 32], // Pad to cache line boundary (4 * 8 bytes + 32 = 64)
}

/// Header for Link shared memory - single-slot design
/// Just a sequence counter to signal new data availability
/// This is the simplest possible 1P1C design - producer overwrites, consumer tracks what it's seen
#[repr(C, align(64))]
struct LinkHeader {
    sequence: AtomicU64,       // Version counter - incremented on each write
    element_size: AtomicUsize, // For validation
    _padding: [u8; 48],        // Pad to full cache line (8 + 8 + 48 = 64)
}

// =============================================================================
// 1P1C OPTIMIZED NETWORK BACKEND FOR LINK
// =============================================================================
//
// Link uses a specialized 1P1C (One Producer, One Consumer) design that is
// fundamentally different from Hub's pub/sub MPMC pattern. This allows for
// significant performance optimizations:
//
// 1. **Connected UDP** - Uses connect() to establish a "virtual connection"
//    - Producer: connect() to consumer, then send() (not sendto())
//    - Consumer: bind(), then recv() (not recvfrom())
//    - ~30% faster than unconnected UDP due to kernel route caching
//
// 2. **No routing overhead** - Direct point-to-point, no topic matching
//
// 3. **Lock-free** - No contention between producer and consumer
//
// 4. **Pre-allocated buffers** - Reuse serialization buffers
//
// Performance hierarchy (fastest to slowest):
// - Local shared memory: ~250ns
// - Unix domain socket (localhost): ~1-2µs
// - Connected UDP (LAN): ~3-5µs
// - Standard UDP (LAN): ~5-8µs
// =============================================================================

/// Network backend type for Link's 1P1C (one producer, one consumer) pattern
///
/// Unlike Hub's NetworkBackend which handles pub/sub routing, Link's backend
/// is optimized for direct point-to-point communication with a single peer.
pub enum LinkNetworkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// Connected UDP - optimized for 1P1C with connect() syscall
    /// ~30% faster than unconnected UDP due to kernel route caching
    ConnectedUdp(ConnectedUdpBackend<T>),

    /// Unix domain socket (localhost only, very fast)
    #[cfg(unix)]
    UnixSocket(UnixSocketLinkBackend<T>),

    /// Zenoh transport for multi-robot mesh, cloud connectivity, and ROS2 interop
    #[cfg(feature = "zenoh-transport")]
    Zenoh(ZenohLinkBackend<T>),
}

/// Connected UDP backend - the fastest UDP option for 1P1C
///
/// Uses connect() to establish a "virtual connection" which allows:
/// - send()/recv() instead of sendto()/recvfrom() (fewer syscall args)
/// - Kernel caches route lookup (no per-packet route decision)
/// - ICMP errors are delivered to the socket
/// - ~30% faster than unconnected UDP
pub struct ConnectedUdpBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    socket: UdpSocket,
    role: LinkRole,
    /// Pre-allocated send buffer to avoid allocation on hot path
    send_buffer: std::cell::UnsafeCell<Vec<u8>>,
    /// Pre-allocated receive buffer
    recv_buffer: std::cell::UnsafeCell<Vec<u8>>,
    _phantom: std::marker::PhantomData<T>,
}

// Safety: ConnectedUdpBackend is designed for 1P1C where only one thread
// accesses send_buffer (producer) and one thread accesses recv_buffer (consumer)
unsafe impl<T> Send for ConnectedUdpBackend<T> where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static
{
}
unsafe impl<T> Sync for ConnectedUdpBackend<T> where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static
{
}

/// Unix socket backend for Link - optimized for localhost 1P1C
#[cfg(unix)]
pub struct UnixSocketLinkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    socket: UnixDatagram,
    socket_path: String,
    role: LinkRole,
    /// Pre-allocated send buffer
    send_buffer: std::cell::UnsafeCell<Vec<u8>>,
    /// Pre-allocated receive buffer
    recv_buffer: std::cell::UnsafeCell<Vec<u8>>,
    _phantom: std::marker::PhantomData<T>,
}

#[cfg(unix)]
unsafe impl<T> Send for UnixSocketLinkBackend<T> where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static
{
}
#[cfg(unix)]
unsafe impl<T> Sync for UnixSocketLinkBackend<T> where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static
{
}

impl<T> std::fmt::Debug for LinkNetworkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LinkNetworkBackend::ConnectedUdp(u) => f
                .debug_struct("ConnectedUdp")
                .field("role", &u.role)
                .finish(),
            #[cfg(unix)]
            LinkNetworkBackend::UnixSocket(u) => f
                .debug_struct("UnixSocket")
                .field("role", &u.role)
                .field("path", &u.socket_path)
                .finish(),
            #[cfg(feature = "zenoh-transport")]
            LinkNetworkBackend::Zenoh(z) => f.debug_struct("Zenoh").field("role", &z.role).finish(),
        }
    }
}

// Buffer size for network messages (64KB - fits most robotics data)
const LINK_BUFFER_SIZE: usize = 65536;

impl<T> ConnectedUdpBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// Create a Connected UDP producer
    ///
    /// The producer connect()s to the consumer's address, enabling fast send() calls.
    /// This is ~30% faster than using sendto() on each message.
    pub fn new_producer(consumer_addr: SocketAddr) -> HorusResult<Self> {
        let bind_addr: SocketAddr = if consumer_addr.is_ipv4() {
            "0.0.0.0:0".parse().unwrap()
        } else {
            "[::]:0".parse().unwrap()
        };

        let socket =
            UdpSocket::bind(bind_addr).map_err(|e| format!("Failed to bind UDP socket: {}", e))?;

        // KEY OPTIMIZATION: connect() to peer for faster send()
        socket
            .connect(consumer_addr)
            .map_err(|e| format!("Failed to connect UDP socket to {}: {}", consumer_addr, e))?;

        socket
            .set_nonblocking(true)
            .map_err(|e| format!("Failed to set non-blocking: {}", e))?;

        Ok(Self {
            socket,
            role: LinkRole::Producer,
            send_buffer: std::cell::UnsafeCell::new(Vec::with_capacity(LINK_BUFFER_SIZE)),
            recv_buffer: std::cell::UnsafeCell::new(Vec::new()), // Producer doesn't recv
            _phantom: std::marker::PhantomData,
        })
    }

    /// Create a Connected UDP consumer
    ///
    /// The consumer bind()s to a port and waits for the producer to send data.
    /// Once first packet arrives, we know the producer's address.
    pub fn new_consumer(listen_addr: SocketAddr) -> HorusResult<Self> {
        let socket = UdpSocket::bind(listen_addr)
            .map_err(|e| format!("Failed to bind UDP socket to {}: {}", listen_addr, e))?;

        socket
            .set_nonblocking(true)
            .map_err(|e| format!("Failed to set non-blocking: {}", e))?;

        Ok(Self {
            socket,
            role: LinkRole::Consumer,
            send_buffer: std::cell::UnsafeCell::new(Vec::new()), // Consumer doesn't send
            recv_buffer: std::cell::UnsafeCell::new(vec![0u8; LINK_BUFFER_SIZE]),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Ultra-fast send using connected UDP
    ///
    /// Uses send() instead of sendto() - kernel already knows the destination
    /// from the connect() call, so this is ~30% faster.
    #[inline(always)]
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        // Safety: In 1P1C design, only producer calls send()
        let buffer = unsafe { &mut *self.send_buffer.get() };

        // Serialize into pre-allocated buffer
        buffer.clear();
        bincode::serialize_into(&mut *buffer, msg)
            .map_err(|e| format!("Serialization error: {}", e))?;

        // Connected UDP: just send() - no address needed!
        self.socket
            .send(buffer)
            .map_err(|e| format!("UDP send error: {}", e))?;

        Ok(())
    }

    /// Ultra-fast receive using pre-allocated buffer
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        // Safety: In 1P1C design, only consumer calls recv()
        let buffer = unsafe { &mut *self.recv_buffer.get() };

        // recv() is faster than recv_from() for connected sockets
        match self.socket.recv(buffer) {
            Ok(len) => bincode::deserialize(&buffer[..len]).ok(),
            Err(_) => None,
        }
    }

    /// Get the local address this socket is bound to
    pub fn local_addr(&self) -> Option<SocketAddr> {
        self.socket.local_addr().ok()
    }
}

#[cfg(unix)]
impl<T> UnixSocketLinkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// Create a Unix socket producer with connect() for fast send()
    pub fn new_producer(topic: &str, consumer_path: &str) -> HorusResult<Self> {
        let socket_path = format!("/tmp/horus_link_{}_{}.sock", topic, std::process::id());

        // Remove if exists
        let _ = std::fs::remove_file(&socket_path);

        let socket = UnixDatagram::bind(&socket_path)
            .map_err(|e| format!("Failed to bind Unix socket: {}", e))?;

        // KEY OPTIMIZATION: connect() to consumer for fast send()
        socket
            .connect(consumer_path)
            .map_err(|e| format!("Failed to connect Unix socket to {}: {}", consumer_path, e))?;

        socket
            .set_nonblocking(true)
            .map_err(|e| format!("Failed to set non-blocking: {}", e))?;

        Ok(Self {
            socket,
            socket_path,
            role: LinkRole::Producer,
            send_buffer: std::cell::UnsafeCell::new(Vec::with_capacity(LINK_BUFFER_SIZE)),
            recv_buffer: std::cell::UnsafeCell::new(Vec::new()),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Create a Unix socket consumer
    pub fn new_consumer(topic: &str) -> HorusResult<Self> {
        let socket_path = format!("/tmp/horus_link_{}_consumer.sock", topic);

        // Remove if exists
        let _ = std::fs::remove_file(&socket_path);

        let socket = UnixDatagram::bind(&socket_path)
            .map_err(|e| format!("Failed to bind Unix socket: {}", e))?;
        socket
            .set_nonblocking(true)
            .map_err(|e| format!("Failed to set non-blocking: {}", e))?;

        Ok(Self {
            socket,
            socket_path,
            role: LinkRole::Consumer,
            send_buffer: std::cell::UnsafeCell::new(Vec::new()),
            recv_buffer: std::cell::UnsafeCell::new(vec![0u8; LINK_BUFFER_SIZE]),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Ultra-fast send using connected Unix socket
    #[inline(always)]
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        let buffer = unsafe { &mut *self.send_buffer.get() };

        buffer.clear();
        bincode::serialize_into(&mut *buffer, msg)
            .map_err(|e| format!("Serialization error: {}", e))?;

        // Connected Unix socket: just send()!
        self.socket
            .send(buffer)
            .map_err(|e| format!("Unix socket send error: {}", e))?;

        Ok(())
    }

    /// Ultra-fast receive using pre-allocated buffer
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        let buffer = unsafe { &mut *self.recv_buffer.get() };

        match self.socket.recv(buffer) {
            Ok(len) => bincode::deserialize(&buffer[..len]).ok(),
            Err(_) => None,
        }
    }

    pub fn get_path(&self) -> &str {
        &self.socket_path
    }
}

#[cfg(unix)]
impl<T> Drop for UnixSocketLinkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.socket_path);
    }
}

/// Zenoh backend for Link - enables multi-robot mesh, cloud, and ROS2 connectivity
///
/// While Link is designed for 1P1C (one producer, one consumer), Zenoh
/// provides pub/sub semantics. This backend adapts Zenoh for Link's API,
/// providing network-transparent point-to-point communication with:
/// - Multi-robot mesh networking
/// - Cloud connectivity
/// - ROS2 interoperability (with ros2_mode)
#[cfg(feature = "zenoh-transport")]
pub struct ZenohLinkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// The underlying Zenoh backend from horus_core's network module
    backend: crate::communication::network::ZenohBackend<T>,
    role: LinkRole,
}

#[cfg(feature = "zenoh-transport")]
impl<T> ZenohLinkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// Create a Zenoh backend for Link producer
    pub fn new_producer(
        topic: &str,
        config: crate::communication::network::ZenohConfig,
    ) -> crate::error::HorusResult<Self> {
        let mut backend = crate::communication::network::ZenohBackend::new_blocking(topic, config)?;
        // Initialize publisher for producer role
        // Use tokio runtime to run async init
        let rt = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))
            .map_err(|e| format!("Failed to get tokio runtime: {}", e))?;
        rt.block_on(backend.init_publisher())?;

        Ok(Self {
            backend,
            role: LinkRole::Producer,
        })
    }

    /// Create a Zenoh backend for Link consumer
    pub fn new_consumer(
        topic: &str,
        config: crate::communication::network::ZenohConfig,
    ) -> crate::error::HorusResult<Self> {
        let mut backend = crate::communication::network::ZenohBackend::new_blocking(topic, config)?;
        // Initialize subscriber for consumer role
        let rt = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))
            .map_err(|e| format!("Failed to get tokio runtime: {}", e))?;
        rt.block_on(backend.init_subscriber())?;

        Ok(Self {
            backend,
            role: LinkRole::Consumer,
        })
    }

    /// Send a message (producer only)
    #[inline(always)]
    pub fn send(&self, msg: &T) -> crate::error::HorusResult<()> {
        self.backend.send(msg)
    }

    /// Receive a message (consumer only)
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        self.backend.recv()
    }

    /// Get the role of this backend
    pub fn role(&self) -> LinkRole {
        self.role
    }
}

/// SPSC (Single Producer Single Consumer) direct link with shared memory IPC or network
/// Single-slot design: always returns the LATEST value, perfect for sensors/control
/// Producer overwrites old data, consumer tracks what it's already read via sequence number
///
/// Supports both local shared memory and network endpoints:
/// - `"topic"` → Local shared memory (248ns latency)
/// - `"topic@192.168.1.5:9000"` → Direct network connection (3-5µs with batch UDP)
/// - `"topic@localhost"` → Unix socket (optimized for localhost)
///
/// Network v2: Smart transport selection automatically picks the best backend:
/// - Batch UDP with sendmmsg/recvmmsg for high throughput (Linux)
/// - Standard UDP for cross-platform compatibility
/// - Unix domain sockets for localhost optimization
/// - TCP fallback for reliability when needed
///
/// # Deprecation Notice
///
/// `Link<T>` is deprecated in favor of `Topic<T>`. The Topic API provides a unified
/// interface with automatic backend selection based on your communication pattern.
///
/// ## Migration Guide
///
/// ```rust,ignore
/// // Before (deprecated):
/// let (producer, consumer) = Link::channel("sensor");
/// producer.send(&msg)?;
/// let received = consumer.recv();
///
/// // After (recommended):
/// let producer: Topic<MyMessage> = Topic::spsc("sensor", true)?;
/// let consumer: Topic<MyMessage> = Topic::spsc("sensor", false)?;
/// producer.send(msg, &mut None)?;
/// let received = consumer.recv(&mut None);
/// ```
///
/// For SPSC cross-process communication, Topic::spsc() provides equivalent functionality.
#[deprecated(
    since = "0.2.0",
    note = "Use Topic<T>::spsc() instead. See Link documentation for migration guide."
)]
#[repr(align(64))]
pub struct Link<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    shm_region: Option<Arc<ShmRegion>>, // Local shared memory (if local)
    network: Option<LinkNetworkBackend<T>>, // Network backend (if network)
    is_network: bool,                   // Fast dispatch flag
    transport_type: &'static str,       // For diagnostics
    topic_name: String,
    producer_node: String,
    consumer_node: String,
    role: LinkRole,
    header: Option<NonNull<LinkHeader>>, // Only for local
    data_ptr: Option<NonNull<u8>>,       // Only for local
    last_seen_sequence: AtomicU64,       // Consumer tracks what it's read (local memory)
    metrics: Arc<AtomicLinkMetrics>,
    state: std::sync::atomic::AtomicU8, // Lock-free state using atomic u8
    _phantom: PhantomData<T>,
}

// Manual Debug implementation since DirectBackend doesn't implement Debug for all T
impl<T> std::fmt::Debug for Link<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Link")
            .field("topic_name", &self.topic_name)
            .field("role", &self.role)
            .field("is_network", &self.is_network)
            .field("transport_type", &self.transport_type)
            .field(
                "state",
                &ConnectionState::from_u8(self.state.load(std::sync::atomic::Ordering::Relaxed)),
            )
            .finish_non_exhaustive()
    }
}

impl<T> Link<T>
where
    T: crate::core::LogSummary
        + serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    // ====== PRIMARY API (recommended) ======

    /// Create a Link as a producer (sender)
    ///
    /// The producer can send messages but cannot receive.
    /// Single-slot design: always overwrites with latest value.
    ///
    /// Supports both local shared memory and network endpoints:
    /// - `"sensor_data"` → Local shared memory (248ns latency)
    /// - `"sensor_data@192.168.1.5:9000"` → Direct network connection (5-15µs latency)
    ///
    /// # Example
    /// ```rust,ignore
    /// // Local
    /// let output: Link<f32> = Link::producer("sensor_data")?;
    ///
    /// // Network
    /// let output: Link<f32> = Link::producer("sensor_data@192.168.1.5:9000")?;
    /// output.send(42.0, None)?;
    /// ```
    pub fn producer(topic: &str) -> HorusResult<Self> {
        Self::with_role(topic, LinkRole::Producer)
    }

    /// Create a Link as a consumer (receiver)
    ///
    /// The consumer can receive messages but cannot send.
    /// Single-slot design: always reads latest value, skips if already seen.
    ///
    /// Supports both local shared memory and network endpoints:
    /// - `"sensor_data"` → Local shared memory (248ns latency)
    /// - `"sensor_data@0.0.0.0:9000"` → Listen for network connections (5-15µs latency)
    ///
    /// # Example
    /// ```rust,ignore
    /// // Local
    /// let input: Link<f32> = Link::consumer("sensor_data")?;
    ///
    /// // Network (listen for producer)
    /// let input: Link<f32> = Link::consumer("sensor_data@0.0.0.0:9000")?;
    /// if let Some(value) = input.recv(None) {
    ///     println!("Received: {}", value);
    /// }
    /// ```
    pub fn consumer(topic: &str) -> HorusResult<Self> {
        Self::with_role(topic, LinkRole::Consumer)
    }

    /// Create a Link as a producer (alias for `producer`)
    ///
    /// **Note:** With flat namespace, all Links are system-wide accessible.
    /// This method is equivalent to `producer()` and kept for backwards compatibility.
    ///
    /// # Example
    /// ```rust,ignore
    /// let output: Link<f32> = Link::producer_global("sensor")?;
    /// output.send(42.0, None)?;
    /// ```
    #[deprecated(
        since = "0.1.7",
        note = "Use producer() instead - all topics are now global"
    )]
    pub fn producer_global(topic: &str) -> HorusResult<Self> {
        Self::producer(topic)
    }

    /// Create a Link as a consumer (alias for `consumer`)
    ///
    /// **Note:** With flat namespace, all Links are system-wide accessible.
    /// This method is equivalent to `consumer()` and kept for backwards compatibility.
    ///
    /// # Example
    /// ```rust,ignore
    /// let input: Link<f32> = Link::consumer_global("sensor")?;
    /// if let Some(value) = input.recv(None) {
    ///     println!("Received: {}", value);
    /// }
    /// ```
    #[deprecated(
        since = "0.1.7",
        note = "Use consumer() instead - all topics are now global"
    )]
    pub fn consumer_global(topic: &str) -> HorusResult<Self> {
        Self::consumer(topic)
    }

    /// Create a Link producer from configuration file
    ///
    /// Loads link configuration from TOML/YAML file and creates a producer.
    ///
    /// # Arguments
    /// * `link_name` - Name of the link to look up in the config file
    ///
    /// # Config File Format
    ///
    /// TOML example:
    /// ```toml
    /// [hubs.video_link]
    /// name = "video"
    /// endpoint = "video@192.168.1.50:9000"  # Producer connects to this
    /// ```
    ///
    /// YAML example:
    /// ```yaml
    /// hubs:
    ///   video_link:
    ///     name: video
    ///     endpoint: video@192.168.1.50:9000
    /// ```
    ///
    /// # Config File Search Paths
    /// 1. `./horus.toml` or `./horus.yaml`
    /// 2. `~/.horus/config.toml` or `~/.horus/config.yaml`
    /// 3. `/etc/horus/config.toml` or `/etc/horus/config.yaml`
    ///
    /// # Example
    /// ```rust,ignore
    /// // Load from config and create producer
    /// let output: Link<VideoFrame> = Link::producer_from_config("video_link")?;
    /// output.send(frame, None)?;
    /// ```
    pub fn producer_from_config(link_name: &str) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from standard search paths
        let config = HorusConfig::find_and_load()?;

        // Get link config
        let link_config = config.get_hub(link_name)?;

        // Get endpoint string
        let endpoint_str = link_config.get_endpoint();

        // Create producer with the endpoint
        Self::producer(&endpoint_str)
    }

    /// Create a Link producer from a specific config file path
    ///
    /// # Arguments
    /// * `config_path` - Path to the configuration file (TOML or YAML)
    /// * `link_name` - Name of the link to look up in the config file
    ///
    /// # Example
    /// ```rust,ignore
    /// let output: Link<f32> = Link::producer_from_config_file("my_config.toml", "sensor_link")?;
    /// ```
    pub fn producer_from_config_file<P: AsRef<std::path::Path>>(
        config_path: P,
        link_name: &str,
    ) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from specific file
        let config = HorusConfig::from_file(config_path)?;

        // Get link config
        let link_config = config.get_hub(link_name)?;

        // Get endpoint string
        let endpoint_str = link_config.get_endpoint();

        // Create producer with the endpoint
        Self::producer(&endpoint_str)
    }

    /// Create a Link consumer from configuration file
    ///
    /// Loads link configuration from TOML/YAML file and creates a consumer.
    ///
    /// # Arguments
    /// * `link_name` - Name of the link to look up in the config file
    ///
    /// # Config File Format
    ///
    /// TOML example:
    /// ```toml
    /// [hubs.video_link]
    /// name = "video"
    /// endpoint = "video@0.0.0.0:9000"  # Consumer listens on this port
    /// ```
    ///
    /// # Example
    /// ```rust,ignore
    /// // Load from config and create consumer
    /// let input: Link<VideoFrame> = Link::consumer_from_config("video_link")?;
    /// if let Some(frame) = input.recv(None) {
    ///     process(frame);
    /// }
    /// ```
    pub fn consumer_from_config(link_name: &str) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from standard search paths
        let config = HorusConfig::find_and_load()?;

        // Get link config
        let link_config = config.get_hub(link_name)?;

        // Get endpoint string
        let endpoint_str = link_config.get_endpoint();

        // Create consumer with the endpoint
        Self::consumer(&endpoint_str)
    }

    /// Create a Link consumer from a specific config file path
    ///
    /// # Arguments
    /// * `config_path` - Path to the configuration file (TOML or YAML)
    /// * `link_name` - Name of the link to look up in the config file
    ///
    /// # Example
    /// ```rust,ignore
    /// let input: Link<f32> = Link::consumer_from_config_file("my_config.toml", "sensor_link")?;
    /// ```
    pub fn consumer_from_config_file<P: AsRef<std::path::Path>>(
        config_path: P,
        link_name: &str,
    ) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from specific file
        let config = HorusConfig::from_file(config_path)?;

        // Get link config
        let link_config = config.get_hub(link_name)?;

        // Get endpoint string
        let endpoint_str = link_config.get_endpoint();

        // Create consumer with the endpoint
        Self::consumer(&endpoint_str)
    }

    // ====== INTERNAL IMPLEMENTATION ======

    /// Internal method to create Link with explicit role
    fn with_role(topic: &str, role: LinkRole) -> HorusResult<Self> {
        let element_size = mem::size_of::<T>();

        if element_size == 0 {
            return Err("Cannot create Link for zero-sized types".into());
        }

        // Parse endpoint: check if it's network (contains '@')
        if topic.contains('@') {
            // Network endpoint
            return Self::create_network_link(topic, role);
        }

        // Local shared memory
        Self::create_local_link(topic, role)
    }

    /// Create a network-based Link with smart transport selection
    ///
    /// Network v2: Automatically selects the best transport based on target:
    /// - localhost: Unix sockets (lowest latency)
    /// - LAN: Batch UDP with sendmmsg/recvmmsg (Linux) or standard UDP
    /// - WAN: TCP with congestion control (fallback)
    fn create_network_link(endpoint: &str, role: LinkRole) -> HorusResult<Self> {
        // Parse endpoint: "topic@host:port" or "topic@localhost"
        let parts: Vec<&str> = endpoint.split('@').collect();
        if parts.len() != 2 {
            return Err(format!("Invalid network endpoint: {}", endpoint).into());
        }

        let topic_name = parts[0];
        let addr_str = parts[1];

        // Use smart transport selection based on network location
        let (network, transport_type) = Self::select_transport(topic_name, addr_str, role)?;

        log::info!(
            "Link '{}': Created as {:?} ({} to {})",
            topic_name,
            role,
            transport_type,
            addr_str
        );

        let metrics = Arc::new(AtomicLinkMetrics {
            messages_sent: AtomicU64::new(0),
            messages_received: AtomicU64::new(0),
            send_failures: AtomicU64::new(0),
            recv_failures: AtomicU64::new(0),
            _padding: [0; 32],
        });

        Ok(Link {
            shm_region: None,
            network: Some(network),
            is_network: true,
            transport_type,
            topic_name: topic_name.to_string(),
            producer_node: "producer".to_string(),
            consumer_node: "consumer".to_string(),
            role,
            header: None,
            data_ptr: None,
            last_seen_sequence: AtomicU64::new(0),
            metrics,
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _phantom: PhantomData,
        })
    }

    /// Select the best transport for the given address
    ///
    /// Link uses a simplified transport selection optimized for 1P1C:
    /// - zenoh: Zenoh transport for multi-robot mesh, cloud, ROS2
    /// - localhost: Unix sockets (fastest for local IPC)
    /// - LAN/WAN: Connected UDP (optimized for point-to-point)
    fn select_transport(
        topic: &str,
        addr_str: &str,
        role: LinkRole,
    ) -> HorusResult<(LinkNetworkBackend<T>, &'static str)> {
        // Check for Zenoh transport
        #[cfg(feature = "zenoh-transport")]
        {
            if addr_str == "zenoh"
                || addr_str.starts_with("zenoh/")
                || addr_str.starts_with("zenoh:")
            {
                // Parse Zenoh mode and connect endpoint
                let (ros2_mode, connect) = if addr_str == "zenoh" {
                    (false, None)
                } else if addr_str == "zenoh/ros2" {
                    (true, None)
                } else if let Some(rest) = addr_str.strip_prefix("zenoh:") {
                    (false, Some(rest.to_string()))
                } else if let Some(rest) = addr_str.strip_prefix("zenoh/ros2:") {
                    (true, Some(rest.to_string()))
                } else {
                    return Err(format!("Invalid zenoh endpoint format: {}", addr_str).into());
                };

                let mut config = if ros2_mode {
                    crate::communication::network::ZenohConfig::ros2(0)
                } else {
                    crate::communication::network::ZenohConfig::default()
                };
                if let Some(endpoint) = connect {
                    config = config.connect_to(&endpoint);
                }

                let backend = match role {
                    LinkRole::Producer => ZenohLinkBackend::new_producer(topic, config)?,
                    LinkRole::Consumer => ZenohLinkBackend::new_consumer(topic, config)?,
                };

                log::info!(
                    "Link '{}': Using Zenoh transport ({}) as {:?}",
                    topic,
                    if ros2_mode { "ROS2 mode" } else { "HORUS mode" },
                    role
                );

                return Ok((LinkNetworkBackend::Zenoh(backend), "zenoh"));
            }
        }

        #[cfg(not(feature = "zenoh-transport"))]
        {
            if addr_str == "zenoh"
                || addr_str.starts_with("zenoh/")
                || addr_str.starts_with("zenoh:")
            {
                return Err(
                    "Zenoh transport not enabled. Compile with --features zenoh-transport".into(),
                );
            }
        }

        // Check for localhost - use Unix socket (fastest for local)
        if addr_str == "localhost"
            || addr_str.starts_with("127.0.0.1")
            || addr_str.starts_with("::1")
        {
            #[cfg(unix)]
            {
                // For localhost, prefer Unix sockets - ~1-2µs latency
                let consumer_path = format!("/tmp/horus_link_{}_consumer.sock", topic);
                let backend = match role {
                    LinkRole::Producer => {
                        UnixSocketLinkBackend::new_producer(topic, &consumer_path)?
                    }
                    LinkRole::Consumer => UnixSocketLinkBackend::new_consumer(topic)?,
                };
                return Ok((LinkNetworkBackend::UnixSocket(backend), "unix_socket"));
            }
            #[cfg(not(unix))]
            {
                // Fall through to Connected UDP on non-Unix
            }
        }

        // Parse address for non-localhost
        let addr: SocketAddr = addr_str
            .parse()
            .map_err(|e| format!("Invalid address '{}': {}", addr_str, e))?;

        // Log transport selection
        let location = NetworkLocation::from_addr(&addr);
        log::debug!(
            "Link '{}': Using Connected UDP for {:?} (1P1C optimized)",
            topic,
            location
        );

        // For all network scenarios, use Connected UDP
        // This is optimal for 1P1C because:
        // - connect() caches route in kernel (~30% faster than sendto)
        // - No batching overhead (single producer/consumer)
        // - Pre-allocated buffers avoid allocation on hot path
        let backend = match role {
            LinkRole::Producer => ConnectedUdpBackend::new_producer(addr)?,
            LinkRole::Consumer => ConnectedUdpBackend::new_consumer(addr)?,
        };
        Ok((LinkNetworkBackend::ConnectedUdp(backend), "connected_udp"))
    }

    /// Create a local shared memory Link
    fn create_local_link(topic: &str, role: LinkRole) -> HorusResult<Self> {
        let element_size = mem::size_of::<T>();
        let element_align = mem::align_of::<T>();
        let header_size = mem::size_of::<LinkHeader>();

        // Single-slot design: header + one element
        let aligned_header_size = header_size.div_ceil(element_align) * element_align;
        let total_size = aligned_header_size + element_size;

        let link_name = format!("links/{}", topic);
        let shm_region = Arc::new(ShmRegion::new(&link_name, total_size)?);

        // Use role names for logging
        let (producer_node, consumer_node) = match role {
            LinkRole::Producer => ("producer", "consumer"),
            LinkRole::Consumer => ("consumer", "producer"),
        };

        Self::create_link(topic, producer_node, consumer_node, role, shm_region)
    }

    /// Common link creation logic
    fn create_link(
        topic_name: &str,
        producer_node: &str,
        consumer_node: &str,
        role: LinkRole,
        shm_region: Arc<ShmRegion>,
    ) -> HorusResult<Self> {
        let element_size = mem::size_of::<T>();
        let element_align = mem::align_of::<T>();
        let header_size = mem::size_of::<LinkHeader>();
        let aligned_header_size = header_size.div_ceil(element_align) * element_align;

        // Initialize header
        let header_ptr = shm_region.as_ptr() as *mut LinkHeader;
        if header_ptr.is_null() {
            return Err("Null pointer for Link header".into());
        }

        let header = unsafe { NonNull::new_unchecked(header_ptr) };

        if shm_region.is_owner() {
            // Initialize header for first time - single-slot design
            unsafe {
                (*header.as_ptr()).sequence.store(0, Ordering::Relaxed);
                (*header.as_ptr())
                    .element_size
                    .store(element_size, Ordering::Relaxed);
                (*header.as_ptr())._padding = [0; 48];
            }
        } else {
            // Validate existing header
            let stored_element_size =
                unsafe { (*header.as_ptr()).element_size.load(Ordering::Relaxed) };

            if stored_element_size != element_size {
                return Err(format!(
                    "Element size mismatch: expected {}, got {}",
                    element_size, stored_element_size
                )
                .into());
            }
        }

        // Data pointer
        let data_ptr = unsafe {
            let raw_ptr = (shm_region.as_ptr() as *mut u8).add(aligned_header_size);
            if raw_ptr.is_null() {
                return Err("Null pointer for Link data".into());
            }
            NonNull::new_unchecked(raw_ptr)
        };

        log::info!(
            "Link '{}': Created as {:?} ({} -> {})",
            topic_name,
            role,
            producer_node,
            consumer_node
        );

        // Initialize metrics in local memory (Arc for cheap cloning)
        let metrics = Arc::new(AtomicLinkMetrics {
            messages_sent: std::sync::atomic::AtomicU64::new(0),
            messages_received: std::sync::atomic::AtomicU64::new(0),
            send_failures: std::sync::atomic::AtomicU64::new(0),
            recv_failures: std::sync::atomic::AtomicU64::new(0),
            _padding: [0; 32],
        });

        Ok(Link {
            shm_region: Some(shm_region),
            network: None,
            is_network: false,
            transport_type: "shm",
            topic_name: topic_name.to_string(),
            producer_node: producer_node.to_string(),
            consumer_node: consumer_node.to_string(),
            role,
            header: Some(header),
            data_ptr: Some(data_ptr),
            last_seen_sequence: AtomicU64::new(0),
            metrics,
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _phantom: PhantomData,
        })
    }

    /// Ultra-fast send with inline zero-copy - optimized for minimum latency
    /// Single-slot design: always overwrites with latest value
    /// Automatically logs if context is provided
    ///
    /// Supports both local shared memory and network transparently
    ///
    /// Optimizations applied:
    /// - Single atomic operation (sequence increment) for local
    /// - Connected UDP with pre-allocated buffers for network
    /// - Relaxed atomics for metrics
    #[inline(always)]
    pub fn send(&self, msg: T, ctx: &mut Option<&mut NodeInfo>) -> Result<(), T>
    where
        T: std::fmt::Debug + Clone + serde::Serialize,
    {
        // Network path - optimized for 1P1C
        if self.is_network {
            if let Some(ref network) = self.network {
                let send_result = match network {
                    LinkNetworkBackend::ConnectedUdp(udp) => udp.send(&msg),
                    #[cfg(unix)]
                    LinkNetworkBackend::UnixSocket(unix) => unix.send(&msg),
                    #[cfg(feature = "zenoh-transport")]
                    LinkNetworkBackend::Zenoh(zenoh) => zenoh.send(&msg),
                };

                match send_result {
                    Ok(_) => {
                        self.metrics.messages_sent.fetch_add(1, Ordering::Relaxed);
                        self.state.store(
                            ConnectionState::Connected.into_u8(),
                            std::sync::atomic::Ordering::Relaxed,
                        );

                        if unlikely(ctx.is_some()) {
                            if let Some(ref mut ctx) = ctx {
                                ctx.register_publisher(
                                    &self.topic_name,
                                    std::any::type_name::<T>(),
                                );
                                ctx.log_pub(&self.topic_name, &msg, 0);
                            }
                        }

                        return Ok(());
                    }
                    Err(_) => {
                        self.metrics.send_failures.fetch_add(1, Ordering::Relaxed);
                        self.state.store(
                            ConnectionState::Failed.into_u8(),
                            std::sync::atomic::Ordering::Relaxed,
                        );
                        return Err(msg);
                    }
                }
            }
            return Err(msg); // Shouldn't happen
        }

        // Local shared memory path (optimized with IPC timing)
        let header = unsafe { self.header.as_ref().unwrap().as_ref() };
        let data_ptr = self.data_ptr.unwrap();

        // Fast path: when ctx is None (benchmarks), bypass timing and logging completely
        if ctx.is_none() {
            // TIME ONLY THE ACTUAL IPC OPERATION
            let ipc_start = Instant::now();

            // Write message to the single slot
            unsafe {
                let slot = data_ptr.as_ptr() as *mut T;
                std::ptr::write(slot, msg);
            }

            // Increment sequence with Release to publish (this is the only sync point!)
            header.sequence.fetch_add(1, Ordering::Release);

            let _ipc_ns = ipc_start.elapsed().as_nanos() as u64;
            // END TIMING - no logging path

            // Update local metrics and state
            self.metrics.messages_sent.fetch_add(1, Ordering::Relaxed);
            self.state.store(
                ConnectionState::Connected.into_u8(),
                std::sync::atomic::Ordering::Relaxed,
            );

            return Ok(());
        }

        // Logging enabled path: time IPC and log with accurate timing
        // TIME ONLY THE ACTUAL IPC OPERATION
        let ipc_start = Instant::now();

        // Write message to the single slot
        unsafe {
            let slot = data_ptr.as_ptr() as *mut T;
            std::ptr::write(slot, msg);
        }

        // Increment sequence with Release to publish (this is the only sync point!)
        header.sequence.fetch_add(1, Ordering::Release);

        let ipc_ns = ipc_start.elapsed().as_nanos() as u64;
        // END TIMING - everything after this is logging overhead

        // Update local metrics and state
        self.metrics.messages_sent.fetch_add(1, Ordering::Relaxed);
        self.state.store(
            ConnectionState::Connected.into_u8(),
            std::sync::atomic::Ordering::Relaxed,
        );

        // Log with accurate IPC timing
        if let Some(ref mut ctx) = ctx {
            ctx.register_publisher(&self.topic_name, std::any::type_name::<T>());
            let slot = unsafe { &*(data_ptr.as_ptr() as *const T) };
            ctx.log_pub(&self.topic_name, slot, ipc_ns);
        }

        Ok(())
    }

    /// Ultra-fast receive with inline - optimized for minimum latency
    /// Single-slot design: reads latest value if new, returns None if already seen
    /// Automatically logs if context is provided
    ///
    /// Supports both local shared memory and network transparently
    ///
    /// Optimizations applied:
    /// - Single atomic load with Acquire (syncs with producer's Release) for local
    /// - Connected UDP with pre-allocated buffers for network
    /// - Local sequence tracking (no atomic stores to shared memory)
    /// - Relaxed atomics for metrics
    #[inline(always)]
    pub fn recv(&self, ctx: &mut Option<&mut NodeInfo>) -> Option<T>
    where
        T: std::fmt::Debug + Clone + serde::de::DeserializeOwned,
    {
        // Network path - optimized for 1P1C
        if self.is_network {
            if let Some(ref network) = self.network {
                let recv_result = match network {
                    LinkNetworkBackend::ConnectedUdp(udp) => udp.recv(),
                    #[cfg(unix)]
                    LinkNetworkBackend::UnixSocket(unix) => unix.recv(),
                    #[cfg(feature = "zenoh-transport")]
                    LinkNetworkBackend::Zenoh(zenoh) => zenoh.recv(),
                };

                if let Some(msg) = recv_result {
                    self.metrics
                        .messages_received
                        .fetch_add(1, Ordering::Relaxed);
                    self.state.store(
                        ConnectionState::Connected.into_u8(),
                        std::sync::atomic::Ordering::Relaxed,
                    );

                    if unlikely(ctx.is_some()) {
                        if let Some(ref mut ctx) = ctx {
                            ctx.register_subscriber(&self.topic_name, std::any::type_name::<T>());
                            ctx.log_sub(&self.topic_name, &msg, 0);
                        }
                    }

                    return Some(msg);
                }
                // Network recv returned None - this is normal for non-blocking
                // Don't count as failure since it could just mean no data yet
            }
            return None;
        }

        // Local shared memory path (optimized with IPC timing)
        let header = unsafe { self.header.as_ref().unwrap().as_ref() };
        let data_ptr = self.data_ptr.unwrap();

        // PREFETCH OPTIMIZATION: Start loading data into L1 cache BEFORE we check
        // if there's new data. This hides memory latency behind the atomic sequence check.
        // By the time we know there's data and read it, it should already be in cache.
        prefetch(data_ptr.as_ptr() as *const u8, PrefetchHint::T0);

        // TIME ONLY THE ACTUAL IPC OPERATION
        let ipc_start = Instant::now();

        // Read sequence with Acquire to synchronize with producer's Release
        let current_seq = header.sequence.load(Ordering::Acquire);
        let last_seen = self.last_seen_sequence.load(Ordering::Relaxed);

        // If we've already seen this sequence, return None (no new data)
        if current_seq <= last_seen {
            return None;
        }

        // Read the message (data should already be in L1 cache from prefetch above)
        let msg = unsafe {
            let slot = data_ptr.as_ptr() as *const T;
            std::ptr::read(slot)
        };

        let ipc_ns = ipc_start.elapsed().as_nanos() as u64;
        // END TIMING - everything after this is post-IPC operations

        // Update what we've seen (local memory, Relaxed is fine)
        self.last_seen_sequence
            .store(current_seq, Ordering::Relaxed);

        // Update local metrics and state
        self.metrics
            .messages_received
            .fetch_add(1, Ordering::Relaxed);
        self.state.store(
            ConnectionState::Connected.into_u8(),
            std::sync::atomic::Ordering::Relaxed,
        );

        // Log with accurate IPC timing
        if unlikely(ctx.is_some()) {
            if let Some(ref mut ctx) = ctx {
                ctx.register_subscriber(&self.topic_name, std::any::type_name::<T>());
                ctx.log_sub(&self.topic_name, &msg, ipc_ns);
            }
        }

        Some(msg)
    }

    /// Check if link has messages available (new data since last read)
    ///
    /// For local shared memory Links, this checks if the sequence number has incremented
    /// (indicating new data).
    ///
    /// For network Links using UDP, this always returns true since UDP is non-blocking
    /// and we can't peek without consuming. Use recv() to check for actual data.
    ///
    /// # Returns
    ///
    /// - `true` if new data is available (or might be available for network)
    /// - `false` if no new data (already seen all data)
    pub fn has_messages(&self) -> bool {
        if self.is_network {
            // For 1P1C network Links, we can't peek UDP without consuming
            // Return true to indicate caller should try recv()
            // This is semantically correct for the single-slot "latest value" design
            true
        } else {
            // Local shared memory: check sequence number
            let header = unsafe { self.header.as_ref().unwrap().as_ref() };
            let current_seq = header.sequence.load(Ordering::Acquire);
            let last_seen = self.last_seen_sequence.load(Ordering::Relaxed);
            current_seq > last_seen
        }
    }

    /// Get the role of this Link end
    pub fn role(&self) -> LinkRole {
        self.role
    }

    /// Check if this Link end is a producer
    pub fn is_producer(&self) -> bool {
        matches!(self.role, LinkRole::Producer)
    }

    /// Check if this Link end is a consumer
    pub fn is_consumer(&self) -> bool {
        matches!(self.role, LinkRole::Consumer)
    }

    /// Get the topic name
    pub fn get_topic_name(&self) -> &str {
        &self.topic_name
    }

    /// Get current connection state (lock-free)
    ///
    /// Returns the current connection state of the Link.
    /// For local shared memory Links, this will typically always be Connected.
    /// For network Links, this tracks whether the connection is healthy or has failures.
    pub fn get_connection_state(&self) -> ConnectionState {
        let state_u8 = self.state.load(std::sync::atomic::Ordering::Relaxed);
        ConnectionState::from_u8(state_u8)
    }

    /// Get performance metrics snapshot (lock-free)
    ///
    /// Returns current counts of messages sent, received, send failures, and recv failures.
    /// These metrics are stored in local memory for zero-overhead tracking.
    pub fn get_metrics(&self) -> LinkMetrics {
        LinkMetrics {
            messages_sent: self.metrics.messages_sent.load(Ordering::Relaxed),
            messages_received: self.metrics.messages_received.load(Ordering::Relaxed),
            send_failures: self.metrics.send_failures.load(Ordering::Relaxed),
            recv_failures: self.metrics.recv_failures.load(Ordering::Relaxed),
        }
    }
}

// Clone implementation for local shared memory Links only
impl<T> Clone for Link<T>
where
    T: crate::core::LogSummary
        + serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// Clone this Link
    ///
    /// # Panics
    ///
    /// Panics if called on a network Link, as network backends contain
    /// non-cloneable resources (TCP streams, sockets, etc.).
    ///
    /// Only local shared memory Links can be cloned safely.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let producer: Link<f64> = Link::producer("sensor")?;
    /// let producer_clone = producer.clone();  // [OK] Works for local Links
    ///
    /// // Both can send independently
    /// producer.send(1.0, None)?;
    /// producer_clone.send(2.0, None)?;
    /// ```
    fn clone(&self) -> Self {
        if self.is_network {
            panic!(
                "Cannot clone network Link '{}': network backends contain non-cloneable resources. \
                Create separate Link instances for each endpoint instead.",
                self.topic_name
            );
        }

        Self {
            shm_region: self.shm_region.clone(), // Arc - cheap clone
            network: None, // Network backend dropped (only local Links can be cloned)
            is_network: false,
            transport_type: self.transport_type,
            topic_name: self.topic_name.clone(),
            producer_node: self.producer_node.clone(),
            consumer_node: self.consumer_node.clone(),
            role: self.role,
            header: self.header,     // NonNull - just copy the pointer
            data_ptr: self.data_ptr, // NonNull - just copy the pointer
            last_seen_sequence: AtomicU64::new(self.last_seen_sequence.load(Ordering::Relaxed)),
            metrics: self.metrics.clone(), // Arc - cheap clone
            state: std::sync::atomic::AtomicU8::new(self.state.load(Ordering::Relaxed)),
            _phantom: PhantomData,
        }
    }
}

unsafe impl<T> Send for Link<T> where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static
{
}
unsafe impl<T> Sync for Link<T> where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static
{
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    /// Test message type implementing all required traits
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    struct TestMessage {
        id: u32,
        value: f64,
        label: String,
    }

    impl crate::core::LogSummary for TestMessage {
        fn log_summary(&self) -> String {
            format!("TestMsg(id={}, val={:.2})", self.id, self.value)
        }
    }

    /// Simple test message for basic tests
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    struct SimpleValue(f64);

    impl crate::core::LogSummary for SimpleValue {
        fn log_summary(&self) -> String {
            format!("{:.2}", self.0)
        }
    }

    // =========================================================================
    // Link Creation Tests
    // =========================================================================

    #[test]
    fn test_link_producer_creation() {
        let producer: Link<SimpleValue> = Link::producer("test_link_producer").unwrap();
        assert_eq!(producer.get_topic_name(), "test_link_producer");
        assert!(producer.is_producer());
        assert!(!producer.is_consumer());
        assert_eq!(producer.get_connection_state(), ConnectionState::Connected);
    }

    #[test]
    fn test_link_consumer_creation() {
        let consumer: Link<SimpleValue> = Link::consumer("test_link_consumer").unwrap();
        assert_eq!(consumer.get_topic_name(), "test_link_consumer");
        assert!(consumer.is_consumer());
        assert!(!consumer.is_producer());
        assert_eq!(consumer.get_connection_state(), ConnectionState::Connected);
    }

    #[test]
    fn test_link_debug() {
        let link: Link<SimpleValue> = Link::producer("test_link_debug").unwrap();
        let debug_str = format!("{:?}", link);
        assert!(debug_str.contains("Link"));
        assert!(debug_str.contains("test_link_debug"));
    }

    // =========================================================================
    // Connection State Tests
    // =========================================================================

    #[test]
    fn test_link_connection_state_conversion() {
        assert_eq!(ConnectionState::Disconnected.into_u8(), 0);
        assert_eq!(ConnectionState::Connecting.into_u8(), 1);
        assert_eq!(ConnectionState::Connected.into_u8(), 2);
        assert_eq!(ConnectionState::Reconnecting.into_u8(), 3);
        assert_eq!(ConnectionState::Failed.into_u8(), 4);

        assert_eq!(ConnectionState::from_u8(0), ConnectionState::Disconnected);
        assert_eq!(ConnectionState::from_u8(1), ConnectionState::Connecting);
        assert_eq!(ConnectionState::from_u8(2), ConnectionState::Connected);
        assert_eq!(ConnectionState::from_u8(3), ConnectionState::Reconnecting);
        assert_eq!(ConnectionState::from_u8(4), ConnectionState::Failed);
        assert_eq!(ConnectionState::from_u8(255), ConnectionState::Failed);
    }

    // =========================================================================
    // Basic Send/Receive Tests
    // =========================================================================

    #[test]
    fn test_link_send_recv_simple() {
        let producer: Link<SimpleValue> = Link::producer("test_link_sr_simple").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_sr_simple").unwrap();

        // Producer sends
        let result = producer.send(SimpleValue(42.0), &mut None);
        assert!(result.is_ok());

        // Consumer receives
        let received = consumer.recv(&mut None);
        assert!(received.is_some());
        assert_eq!(received.unwrap(), SimpleValue(42.0));
    }

    #[test]
    fn test_link_send_recv_complex_message() {
        let producer: Link<TestMessage> = Link::producer("test_link_sr_complex").unwrap();
        let consumer: Link<TestMessage> = Link::consumer("test_link_sr_complex").unwrap();

        let msg = TestMessage {
            id: 123,
            value: 1.234, // Arbitrary test value
            label: "test".to_string(),
        };

        producer.send(msg.clone(), &mut None).unwrap();
        let received = consumer.recv(&mut None).unwrap();
        assert_eq!(received, msg);
    }

    #[test]
    fn test_link_recv_empty() {
        let consumer: Link<SimpleValue> = Link::consumer("test_link_recv_empty").unwrap();
        // No message sent, should return None
        let received = consumer.recv(&mut None);
        assert!(received.is_none());
    }

    #[test]
    fn test_link_overwrite_semantics() {
        // Link uses single-slot semantics - new message overwrites old
        let producer: Link<SimpleValue> = Link::producer("test_link_overwrite").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_overwrite").unwrap();

        producer.send(SimpleValue(1.0), &mut None).unwrap();
        producer.send(SimpleValue(2.0), &mut None).unwrap();
        producer.send(SimpleValue(3.0), &mut None).unwrap();

        // Consumer should get the latest value
        let received = consumer.recv(&mut None);
        assert!(received.is_some());
        assert_eq!(received.unwrap(), SimpleValue(3.0));
    }

    #[test]
    fn test_link_sequence_tracking() {
        // Consumer tracks sequence to avoid re-reading same message
        let producer: Link<SimpleValue> = Link::producer("test_link_sequence").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_sequence").unwrap();

        // Send one message
        producer.send(SimpleValue(1.0), &mut None).unwrap();

        // First recv gets it
        let first = consumer.recv(&mut None);
        assert!(first.is_some());
        assert_eq!(first.unwrap(), SimpleValue(1.0));

        // Second recv without new send returns None (already seen)
        let second = consumer.recv(&mut None);
        assert!(second.is_none());

        // Send new message
        producer.send(SimpleValue(2.0), &mut None).unwrap();

        // Now recv gets the new one
        let third = consumer.recv(&mut None);
        assert!(third.is_some());
        assert_eq!(third.unwrap(), SimpleValue(2.0));
    }

    // =========================================================================
    // Metrics Tests
    // =========================================================================

    #[test]
    fn test_link_metrics_initial() {
        let link: Link<SimpleValue> = Link::producer("test_link_metrics_init").unwrap();
        let metrics = link.get_metrics();

        assert_eq!(metrics.messages_sent, 0);
        assert_eq!(metrics.messages_received, 0);
        assert_eq!(metrics.send_failures, 0);
        assert_eq!(metrics.recv_failures, 0);
    }

    #[test]
    fn test_link_metrics_after_send() {
        let producer: Link<SimpleValue> = Link::producer("test_link_metrics_send").unwrap();

        producer.send(SimpleValue(1.0), &mut None).unwrap();
        producer.send(SimpleValue(2.0), &mut None).unwrap();
        producer.send(SimpleValue(3.0), &mut None).unwrap();

        let metrics = producer.get_metrics();
        assert_eq!(metrics.messages_sent, 3);
        assert_eq!(metrics.send_failures, 0);
    }

    #[test]
    fn test_link_metrics_after_recv() {
        let producer: Link<SimpleValue> = Link::producer("test_link_metrics_recv").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_metrics_recv").unwrap();

        producer.send(SimpleValue(1.0), &mut None).unwrap();
        consumer.recv(&mut None);

        let metrics = consumer.get_metrics();
        assert_eq!(metrics.messages_received, 1);
    }

    // =========================================================================
    // Clone Tests
    // =========================================================================

    #[test]
    fn test_link_clone_local() {
        let producer1: Link<SimpleValue> = Link::producer("test_link_clone_local").unwrap();
        let producer2 = producer1.clone();

        // Both should work independently
        producer1.send(SimpleValue(1.0), &mut None).unwrap();
        producer2.send(SimpleValue(2.0), &mut None).unwrap();

        // Metrics are shared
        assert_eq!(producer1.get_metrics().messages_sent, 2);
        assert_eq!(producer2.get_metrics().messages_sent, 2);
    }

    // =========================================================================
    // Edge Cases
    // =========================================================================

    #[test]
    fn test_link_with_large_message() {
        #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
        struct LargeMessage {
            data: Vec<u8>,
        }

        impl crate::core::LogSummary for LargeMessage {
            fn log_summary(&self) -> String {
                format!("LargeMsg({}B)", self.data.len())
            }
        }

        let producer: Link<LargeMessage> = Link::producer("test_link_large_msg").unwrap();
        let consumer: Link<LargeMessage> = Link::consumer("test_link_large_msg").unwrap();

        let large_data = LargeMessage {
            data: vec![42u8; 10000], // 10KB message
        };

        producer.send(large_data.clone(), &mut None).unwrap();
        let received = consumer.recv(&mut None).unwrap();
        assert_eq!(received.data.len(), 10000);
        assert!(received.data.iter().all(|&b| b == 42));
    }

    #[test]
    fn test_link_rapid_send_recv() {
        let producer: Link<SimpleValue> = Link::producer("test_link_rapid").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_rapid").unwrap();

        for i in 0..1000 {
            producer.send(SimpleValue(i as f64), &mut None).unwrap();
            let val = consumer.recv(&mut None);
            assert!(val.is_some());
            assert_eq!(val.unwrap(), SimpleValue(i as f64));
        }

        assert_eq!(producer.get_metrics().messages_sent, 1000);
        assert_eq!(consumer.get_metrics().messages_received, 1000);
    }

    #[test]
    fn test_link_multiple_send_single_recv() {
        let producer: Link<SimpleValue> = Link::producer("test_link_multi_send").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_multi_send").unwrap();

        // Send many messages rapidly
        for i in 0..100 {
            producer.send(SimpleValue(i as f64), &mut None).unwrap();
        }

        // Consumer only reads once - should get the latest
        let received = consumer.recv(&mut None);
        assert!(received.is_some());
        // Value should be 99.0 (last sent)
        assert_eq!(received.unwrap(), SimpleValue(99.0));
    }

    // =========================================================================
    // Role Tests
    // =========================================================================

    #[test]
    fn test_link_role_enum() {
        assert_eq!(LinkRole::Producer, LinkRole::Producer);
        assert_eq!(LinkRole::Consumer, LinkRole::Consumer);
        assert_ne!(LinkRole::Producer, LinkRole::Consumer);
    }

    #[test]
    fn test_link_role_checking() {
        let producer: Link<SimpleValue> = Link::producer("test_link_role_check").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_role_check").unwrap();

        assert!(producer.is_producer());
        assert!(!producer.is_consumer());

        assert!(consumer.is_consumer());
        assert!(!consumer.is_producer());
    }

    // =========================================================================
    // Thread Safety Tests
    // =========================================================================

    #[test]
    fn test_link_send_from_different_thread() {
        use std::thread;

        let producer: Link<SimpleValue> = Link::producer("test_link_threaded").unwrap();
        let consumer: Link<SimpleValue> = Link::consumer("test_link_threaded").unwrap();

        // Send from another thread
        let handle = thread::spawn(move || {
            for i in 0..10 {
                producer.send(SimpleValue(i as f64), &mut None).unwrap();
                thread::sleep(std::time::Duration::from_micros(100));
            }
            producer.get_metrics().messages_sent
        });

        // Let producer get ahead
        thread::sleep(std::time::Duration::from_millis(5));

        // Receive should work
        let mut received_count = 0;
        for _ in 0..20 {
            if consumer.recv(&mut None).is_some() {
                received_count += 1;
            }
            thread::sleep(std::time::Duration::from_micros(100));
        }

        let sent_count = handle.join().unwrap();
        assert_eq!(sent_count, 10);
        // Should have received at least some messages
        assert!(received_count >= 1);
    }

    // =========================================================================
    // LinkMetrics Tests
    // =========================================================================

    #[test]
    fn test_link_metrics_default() {
        let metrics = LinkMetrics::default();
        assert_eq!(metrics.messages_sent, 0);
        assert_eq!(metrics.messages_received, 0);
        assert_eq!(metrics.send_failures, 0);
        assert_eq!(metrics.recv_failures, 0);
    }

    #[test]
    fn test_link_metrics_clone() {
        let metrics = LinkMetrics {
            messages_sent: 10,
            messages_received: 5,
            send_failures: 1,
            recv_failures: 2,
        };
        let cloned = metrics.clone();
        assert_eq!(cloned.messages_sent, 10);
        assert_eq!(cloned.messages_received, 5);
    }
}
