use super::endpoint::Endpoint;
use super::router::RouterBackend;
use super::smart_copy::{CopyStrategy, SmartCopyConfig, SmartCopySender};
use super::smart_transport::{NetworkLocation, TransportSelector, TransportType};
use super::udp_multicast::UdpMulticastBackend;
#[cfg(unix)]
use super::unix_socket::UnixSocketBackend;

use super::batch_udp::{BatchUdpConfig, BatchUdpReceiver, BatchUdpSender};

#[cfg(feature = "zenoh-transport")]
use super::zenoh_backend::ZenohBackend;
#[cfg(feature = "zenoh-transport")]
use super::zenoh_config::ZenohConfig;

#[cfg(feature = "quic")]
use super::quic::{QuicBackend, QuicConfig};

use crate::error::HorusResult;
use std::net::SocketAddr;
use std::sync::Arc;

/// Network backend for Topic communication
///
/// Provides actual network implementations with automatic selection:
/// - Shared memory (local, fastest)
/// - Unix domain sockets (localhost, Unix only)
/// - Batch UDP with sendmmsg/recvmmsg (Linux, high throughput)
/// - Standard UDP (fallback)
/// - Router (central message broker)
/// - Multicast discovery
///
/// Network backend enum wrapping different transport types
#[allow(clippy::large_enum_variant)]
pub enum NetworkBackend<T> {
    /// Unix domain socket (localhost, Unix only)
    #[cfg(unix)]
    UnixSocket(UnixSocketBackend<T>),

    /// Batch UDP with sendmmsg/recvmmsg (Linux) or regular UDP (fallback)
    BatchUdp(BatchUdpBackendWrapper<T>),

    /// Multicast discovery
    Multicast(UdpMulticastBackend<T>),

    /// Router (central message broker)
    Router(RouterBackend<T>),

    /// Zenoh transport (multi-robot mesh, cloud, ROS2 interop)
    #[cfg(feature = "zenoh-transport")]
    Zenoh(ZenohBackend<T>),

    /// QUIC transport (reliable, encrypted, low-latency)
    #[cfg(feature = "quic")]
    Quic(QuicBackend<T>),
}

/// Wrapper for batch UDP sender/receiver pair with smart copy support
#[cfg(target_os = "linux")]
pub struct BatchUdpBackendWrapper<T> {
    sender: std::sync::Mutex<BatchUdpSender>,
    receiver: std::sync::Mutex<BatchUdpReceiver>,
    smart_copy: Arc<SmartCopySender>,
    topic: String,
    remote_addr: SocketAddr,
    _phantom: std::marker::PhantomData<T>,
}

// Safety: The Mutex provides interior mutability with thread-safety
#[cfg(target_os = "linux")]
unsafe impl<T: Send> Send for BatchUdpBackendWrapper<T> {}
#[cfg(target_os = "linux")]
unsafe impl<T: Send> Sync for BatchUdpBackendWrapper<T> {}

#[cfg(target_os = "linux")]
impl<T> std::fmt::Debug for BatchUdpBackendWrapper<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BatchUdpBackendWrapper")
            .field("topic", &self.topic)
            .field("remote_addr", &self.remote_addr)
            .field("smart_copy", &self.smart_copy)
            .finish()
    }
}

impl<T> NetworkBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// Create a new network backend from an endpoint
    ///
    /// Uses smart transport selection to automatically choose the best backend
    /// based on network location and available system features.
    pub fn new(endpoint: Endpoint) -> HorusResult<Self> {
        match endpoint {
            Endpoint::Local { .. } => Err(crate::error::HorusError::Communication(
                "Local endpoint should use shared memory, not network backend".to_string(),
            )),

            Endpoint::Localhost { topic, .. } => {
                // For localhost, use smart transport selection
                let localhost_addr: SocketAddr = "127.0.0.1:0".parse().unwrap();
                Self::create_for_address(&topic, localhost_addr, true)
            }

            Endpoint::Direct { topic, host, port } => {
                // Create with smart transport selection based on target address
                let addr = SocketAddr::new(host, port);
                Self::create_for_address(&topic, addr, false)
            }

            Endpoint::Multicast { topic } => {
                // Multicast doesn't use smart selection - it's a specific use case
                let multicast_backend = UdpMulticastBackend::new(&topic)?;
                Ok(NetworkBackend::Multicast(multicast_backend))
            }

            Endpoint::Router { topic, host, port } => {
                // Router backend is also a specific use case
                let router_backend = if let Some(h) = host {
                    let p = port.unwrap_or(7777);
                    RouterBackend::new_with_addr(&topic, h, p)?
                } else {
                    let h = "127.0.0.1".parse().unwrap();
                    let p = port.unwrap_or(7777);
                    RouterBackend::new_with_addr(&topic, h, p)?
                };
                Ok(NetworkBackend::Router(router_backend))
            }

            Endpoint::Zenoh {
                topic,
                ros2_mode,
                connect,
            } => {
                #[cfg(feature = "zenoh-transport")]
                {
                    let mut config = if ros2_mode {
                        ZenohConfig::ros2(0)
                    } else {
                        ZenohConfig::default()
                    };

                    // Add connect endpoint if specified
                    if let Some(endpoint) = connect {
                        config = config.connect_to(&endpoint);
                    }

                    let backend = ZenohBackend::new_blocking(&topic, config)?;
                    Ok(NetworkBackend::Zenoh(backend))
                }
                #[cfg(not(feature = "zenoh-transport"))]
                {
                    let _ = (topic, ros2_mode, connect);
                    Err(crate::error::HorusError::Communication(
                        "Zenoh transport not enabled. Compile with --features zenoh-transport"
                            .to_string(),
                    ))
                }
            }

            Endpoint::Mdns {
                topic,
                hostname,
                port,
            } => {
                // mDNS resolution requires the mdns-sd feature
                #[cfg(feature = "mdns")]
                {
                    // Resolve hostname via mDNS and create direct connection
                    use crate::communication::network::mdns::resolve_mdns_hostname;
                    let resolved_ip = resolve_mdns_hostname(&hostname)?;
                    let resolved_port = port.unwrap_or(super::endpoint::DEFAULT_PORT);
                    let addr = SocketAddr::new(resolved_ip, resolved_port);
                    Self::create_for_address(&topic, addr, false)
                }
                #[cfg(not(feature = "mdns"))]
                {
                    let _ = (topic, hostname, port);
                    Err(crate::error::HorusError::Communication(
                        "mDNS resolution not enabled. Compile with --features mdns or use \
                         direct IP address instead (e.g., 'topic@192.168.1.5')"
                            .to_string(),
                    ))
                }
            }


            Endpoint::Cloud { topic, mode } => {
                // Cloud transport requires WebSocket connection to cloud/relay server
                // or VPN address resolution
                let _ = (topic, mode);
                Err(crate::error::HorusError::Communication(
                    "Cloud transport not yet implemented. Use Zenoh (topic@zenoh) for \
                     cloud connectivity, or direct IP addresses for VPN peers."
                        .to_string(),
                ))
            }

            Endpoint::ZenohCloud { topic, config } => {
                // Zenoh cloud mode with router configuration
                #[cfg(feature = "zenoh-transport")]
                {
                    // Build Zenoh config from ZenohCloudConfig
                    let mut zenoh_config = ZenohConfig::default();

                    // Add primary and backup routers as connect endpoints
                    zenoh_config = zenoh_config.connect_to(&config.primary_router);
                    for backup in &config.backup_routers {
                        zenoh_config = zenoh_config.connect_to(backup);
                    }

                    let backend = ZenohBackend::new_blocking(&topic, zenoh_config)?;
                    Ok(NetworkBackend::Zenoh(backend))
                }
                #[cfg(not(feature = "zenoh-transport"))]
                {
                    let _ = (topic, config);
                    Err(crate::error::HorusError::Communication(
                        "Zenoh transport not enabled. Compile with --features zenoh-transport"
                            .to_string(),
                    ))
                }
            }
        }
    }

    /// Create backend for a specific address using smart transport selection
    fn create_for_address(topic: &str, addr: SocketAddr, is_localhost: bool) -> HorusResult<Self> {
        let selector = TransportSelector::new();
        let transport_type = selector.select(&addr);

        log::debug!(
            "Smart transport selected {:?} for {} (location: {:?})",
            transport_type,
            addr,
            NetworkLocation::from_addr(&addr)
        );

        // Try the selected transport, fall back if it fails
        match Self::try_create_transport(topic, addr, transport_type, is_localhost) {
            Ok(backend) => Ok(backend),
            Err(e) => {
                // Try fallback
                if let Some(fallback) = selector.get_fallback(transport_type) {
                    log::warn!(
                        "Primary transport {:?} failed ({}), trying fallback {:?}",
                        transport_type,
                        e,
                        fallback
                    );
                    Self::try_create_transport(topic, addr, fallback, is_localhost)
                } else {
                    Err(e)
                }
            }
        }
    }

    /// Try to create a specific transport type
    fn try_create_transport(
        topic: &str,
        addr: SocketAddr,
        transport: TransportType,
        is_localhost: bool,
    ) -> HorusResult<Self> {
        match transport {
            TransportType::SharedMemory => {
                // Shared memory is handled at a higher level (Topic), not here
                // Fall through to Unix socket or UDP
                if is_localhost {
                    #[cfg(unix)]
                    {
                        let unix_backend = UnixSocketBackend::new_subscriber(topic)?;
                        return Ok(NetworkBackend::UnixSocket(unix_backend));
                    }
                }
                // Fall through to UDP
                Self::create_batch_udp(topic, addr)
            }

            TransportType::UnixSocket => {
                #[cfg(unix)]
                {
                    let unix_backend = UnixSocketBackend::new_subscriber(topic)?;
                    Ok(NetworkBackend::UnixSocket(unix_backend))
                }
                #[cfg(not(unix))]
                {
                    // Fall back to UDP on non-Unix
                    Self::create_batch_udp(topic, addr)
                }
            }

            TransportType::BatchUdp => Self::create_batch_udp(topic, addr),

            TransportType::IoUring => {
                // io_uring requires special setup and is best used via BatchUdp wrapper
                // For now, fall back to BatchUdp or standard UDP
                #[cfg(target_os = "linux")]
                {
                    Self::create_batch_udp(topic, addr)
                }
                #[cfg(not(target_os = "linux"))]
                {
                    Self::create_batch_udp(topic, addr)
                }
            }

            TransportType::Udp => {
                Self::create_batch_udp(topic, addr)
            }

            TransportType::Tcp => {
                // TCP is handled via Router backend
                // For direct TCP, fall back to UDP for now
                Self::create_batch_udp(topic, addr)
            }

            TransportType::Quic => {
                #[cfg(feature = "quic")]
                {
                    let config = QuicConfig::default();
                    let backend = QuicBackend::new_blocking(topic, addr, config).map_err(|e| {
                        crate::error::HorusError::Communication(format!(
                            "Failed to create QUIC backend: {}",
                            e
                        ))
                    })?;
                    Ok(NetworkBackend::Quic(backend))
                }
                #[cfg(not(feature = "quic"))]
                {
                    log::warn!(
                        "QUIC transport requested but feature not enabled; falling back to UDP"
                    );
                    Self::create_batch_udp(topic, addr)
                }
            }

            TransportType::Zenoh => {
                #[cfg(feature = "zenoh-transport")]
                {
                    let config = ZenohConfig::default();
                    let backend = ZenohBackend::new_blocking(topic, config)?;
                    Ok(NetworkBackend::Zenoh(backend))
                }
                #[cfg(not(feature = "zenoh-transport"))]
                {
                    log::warn!(
                        "Zenoh transport requested but feature not enabled; falling back to UDP"
                    );
                    Self::create_batch_udp(topic, addr)
                }
            }

            TransportType::ZenohRos2 => {
                #[cfg(feature = "zenoh-transport")]
                {
                    // Use ROS2-compatible configuration
                    let config = ZenohConfig::ros2(0); // Domain ID 0 by default
                    let backend = ZenohBackend::new_blocking(topic, config)?;
                    Ok(NetworkBackend::Zenoh(backend))
                }
                #[cfg(not(feature = "zenoh-transport"))]
                {
                    log::warn!("Zenoh ROS2 transport requested but feature not enabled; falling back to UDP");
                    Self::create_batch_udp(topic, addr)
                }
            }
        }
    }

    /// Create batch UDP backend (Linux only) with smart copy support
    #[cfg(target_os = "linux")]
    fn create_batch_udp(topic: &str, addr: SocketAddr) -> HorusResult<Self> {
        let config = BatchUdpConfig::default();

        // Bind to any available port for sending
        let bind_addr: SocketAddr = if addr.is_ipv4() {
            "0.0.0.0:0".parse().unwrap()
        } else {
            "[::]:0".parse().unwrap()
        };

        let sender = BatchUdpSender::new(bind_addr, config.clone()).map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to create batch UDP sender: {}",
                e
            ))
        })?;

        // Receiver binds to a specific port (use default HORUS port or dynamic)
        let recv_addr: SocketAddr = if addr.is_ipv4() {
            "0.0.0.0:0".parse().unwrap()
        } else {
            "[::]:0".parse().unwrap()
        };

        let receiver = BatchUdpReceiver::new(recv_addr, config).map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to create batch UDP receiver: {}",
                e
            ))
        })?;

        // Create smart copy sender for automatic zero-copy on large messages
        let smart_copy = Arc::new(SmartCopySender::new(SmartCopyConfig::default()));

        Ok(NetworkBackend::BatchUdp(BatchUdpBackendWrapper {
            sender: std::sync::Mutex::new(sender),
            receiver: std::sync::Mutex::new(receiver),
            smart_copy,
            topic: topic.to_string(),
            remote_addr: addr,
            _phantom: std::marker::PhantomData,
        }))
    }

    /// Send a message over the network
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        match self {
            #[cfg(unix)]
            NetworkBackend::UnixSocket(backend) => backend.send(msg),            #[cfg(target_os = "linux")]
            NetworkBackend::BatchUdp(backend) => {
                let data = bincode::serialize(msg).map_err(|e| {
                    crate::error::HorusError::Communication(format!("Serialization error: {}", e))
                })?;

                // Use smart copy - automatically selects zero-copy for large messages
                let (strategy, buffer) = backend.smart_copy.prepare_send(&data);

                let result = {
                    let mut sender = backend.sender.lock().map_err(|e| {
                        crate::error::HorusError::Communication(format!("Sender lock error: {}", e))
                    })?;

                    // Use the buffer if we got one (zero-copy path), otherwise use data directly
                    let send_data = match &buffer {
                        Some(buf) => buf.as_slice(),
                        None => &data,
                    };

                    sender.send(send_data, backend.remote_addr).map_err(|e| {
                        crate::error::HorusError::Communication(format!(
                            "Batch UDP send error: {}",
                            e
                        ))
                    })
                };

                // Release buffer back to pool after send completes
                backend.smart_copy.complete_send(buffer);

                // Log strategy for debugging (only in debug mode)
                log::trace!("BatchUdp send: {} bytes via {:?}", data.len(), strategy);

                result
            }
            NetworkBackend::Multicast(backend) => backend.send(msg),
            NetworkBackend::Router(backend) => backend.send(msg),
            #[cfg(feature = "zenoh-transport")]
            NetworkBackend::Zenoh(backend) => backend.send(msg),
            #[cfg(feature = "quic")]
            NetworkBackend::Quic(backend) => backend.send(msg).map_err(|e| {
                crate::error::HorusError::Communication(format!("QUIC send error: {}", e))
            }),
        }
    }

    /// Receive a message from the network
    pub fn recv(&mut self) -> Option<T> {
        match self {
            #[cfg(unix)]
            NetworkBackend::UnixSocket(backend) => backend.recv(),            #[cfg(target_os = "linux")]
            NetworkBackend::BatchUdp(backend) => {
                let mut receiver = match backend.receiver.lock() {
                    Ok(r) => r,
                    Err(_) => return None,
                };
                match receiver.recv_batch(1) {
                    Ok(packets) => {
                        if let Some(packet) = packets.into_iter().next() {
                            bincode::deserialize(&packet.data).ok()
                        } else {
                            None
                        }
                    }
                    Err(_) => None,
                }
            }
            NetworkBackend::Multicast(backend) => backend.recv(),
            NetworkBackend::Router(backend) => backend.recv(),
            #[cfg(feature = "zenoh-transport")]
            NetworkBackend::Zenoh(backend) => backend.recv(),
            #[cfg(feature = "quic")]
            NetworkBackend::Quic(backend) => backend.recv(),
        }
    }

    /// Get the selected transport type for diagnostics
    pub fn transport_type(&self) -> &'static str {
        match self {
            #[cfg(unix)]
            NetworkBackend::UnixSocket(_) => "unix_socket",            #[cfg(target_os = "linux")]
            NetworkBackend::BatchUdp(_) => "batch_udp",
            NetworkBackend::Multicast(_) => "multicast",
            NetworkBackend::Router(_) => "router",
            #[cfg(feature = "zenoh-transport")]
            NetworkBackend::Zenoh(_) => "zenoh",
            #[cfg(feature = "quic")]
            NetworkBackend::Quic(_) => "quic",
        }
    }

    /// Get smart copy statistics (BatchUdp only)
    ///
    /// Returns None for non-BatchUdp backends
    #[cfg(target_os = "linux")]
    pub fn smart_copy_stats(&self) -> Option<&super::smart_copy::SmartCopyStats> {
        match self {
            NetworkBackend::BatchUdp(backend) => Some(backend.smart_copy.stats()),
            _ => None,
        }
    }

    /// Get copy strategy that would be used for a message of given size
    #[cfg(target_os = "linux")]
    pub fn copy_strategy_for_size(&self, size: usize) -> Option<CopyStrategy> {
        match self {
            NetworkBackend::BatchUdp(backend) => Some(backend.smart_copy.select_strategy(size)),
            _ => None,
        }
    }
}

impl<T> std::fmt::Debug for NetworkBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            #[cfg(unix)]
            NetworkBackend::UnixSocket(backend) => f
                .debug_struct("NetworkBackend::UnixSocket")
                .field("backend", backend)
                .finish(),
            #[cfg(target_os = "linux")]
            NetworkBackend::BatchUdp(backend) => f
                .debug_struct("NetworkBackend::BatchUdp")
                .field("backend", backend)
                .finish(),
            NetworkBackend::Multicast(backend) => f
                .debug_struct("NetworkBackend::Multicast")
                .field("backend", backend)
                .finish(),
            NetworkBackend::Router(backend) => f
                .debug_struct("NetworkBackend::Router")
                .field("backend", backend)
                .finish(),
            #[cfg(feature = "zenoh-transport")]
            NetworkBackend::Zenoh(backend) => f
                .debug_struct("NetworkBackend::Zenoh")
                .field("backend", backend)
                .finish(),
            #[cfg(feature = "quic")]
            NetworkBackend::Quic(backend) => f
                .debug_struct("NetworkBackend::Quic")
                .field("backend", backend)
                .finish(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::network::endpoint::parse_endpoint;

    #[test]
    fn test_transport_type_method() {
        // Test that we can get transport type string
        let endpoint = parse_endpoint("test@192.168.1.100:9870").unwrap();
        if let Ok(backend) = NetworkBackend::<Vec<u8>>::new(endpoint) {
            let transport = backend.transport_type();
            assert!(!transport.is_empty());
            println!("Selected transport: {}", transport);
        }
    }

    #[test]
    fn test_localhost_selection() {
        let endpoint = parse_endpoint("test@localhost").unwrap();
        if let Ok(backend) = NetworkBackend::<Vec<u8>>::new(endpoint) {
            let transport = backend.transport_type();
            // Should be unix_socket on Unix, or udp_direct on Windows
            #[cfg(unix)]
            assert!(transport == "unix_socket" || transport == "batch_udp");
            #[cfg(not(unix))]
            assert_eq!(transport, "udp_direct");
        }
    }
}
