//! WebRTC Data Channel transport for HORUS
//!
//! This module provides WebRTC-based communication enabling:
//! - Browser-to-robot connections (web dashboards, remote control)
//! - Robot-to-robot connections with NAT traversal
//! - Low-latency unreliable/reliable data channels
//!
//! # Features
//!
//! - **DTLS encryption** - All connections are encrypted by default
//! - **ICE integration** - Uses existing HORUS P2P/ICE infrastructure
//! - **Data channels** - Supports both reliable (ordered) and unreliable (unordered) modes
//! - **Browser compatibility** - Works with standard WebRTC browser APIs
//!
//! # Endpoint Format
//!
//! ```text
//! topic@webrtc:peer-id              - Connect to peer via WebRTC
//! topic@webrtc:peer-id/reliable     - Force reliable data channel
//! topic@webrtc:peer-id/unreliable   - Force unreliable data channel
//! ```
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::network::webrtc::*;
//!
//! // Create WebRTC transport
//! let config = WebRtcConfig::default();
//! let transport = WebRtcTransport::new(config).await?;
//!
//! // Connect to a peer (via signaling server)
//! let peer_id = PeerId::from_short_id("a3f7-k2m9-p4n8");
//! let channel = transport.connect(&peer_id).await?;
//!
//! // Send data
//! channel.send(b"hello robot").await?;
//!
//! // Receive data
//! let data = channel.recv().await?;
//! ```
//!
//! Requires the `webrtc-transport` feature to be enabled.

use crate::communication::network::p2p::{PeerId, TurnServer};

use std::collections::HashMap;
use std::io;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use parking_lot::RwLock;
use tokio::sync::{mpsc, oneshot};

#[cfg(feature = "webrtc-transport")]
use bytes::Bytes;
#[cfg(feature = "webrtc-transport")]
use webrtc::api::interceptor_registry::register_default_interceptors;
#[cfg(feature = "webrtc-transport")]
use webrtc::api::media_engine::MediaEngine;
#[cfg(feature = "webrtc-transport")]
use webrtc::api::APIBuilder;
#[cfg(feature = "webrtc-transport")]
use webrtc::data_channel::data_channel_init::RTCDataChannelInit;
#[cfg(feature = "webrtc-transport")]
use webrtc::data_channel::data_channel_message::DataChannelMessage;
#[cfg(feature = "webrtc-transport")]
use webrtc::data_channel::RTCDataChannel;
#[cfg(feature = "webrtc-transport")]
use webrtc::ice_transport::ice_candidate::RTCIceCandidateInit;
#[cfg(feature = "webrtc-transport")]
use webrtc::ice_transport::ice_server::RTCIceServer;
#[cfg(feature = "webrtc-transport")]
use webrtc::interceptor::registry::Registry;
#[cfg(feature = "webrtc-transport")]
use webrtc::peer_connection::configuration::RTCConfiguration;
#[cfg(feature = "webrtc-transport")]
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;
#[cfg(feature = "webrtc-transport")]
use webrtc::peer_connection::RTCPeerConnection;

/// WebRTC data channel mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum DataChannelMode {
    /// Reliable ordered delivery (like TCP)
    #[default]
    Reliable,
    /// Unreliable unordered delivery (like UDP)
    Unreliable,
    /// Reliable but unordered
    ReliableUnordered,
    /// Unreliable with limited retransmissions
    PartialReliable {
        /// Maximum number of retransmissions
        max_retransmits: u16,
    },
    /// Unreliable with time-limited retransmissions
    TimeLimited {
        /// Maximum milliseconds to retransmit
        max_packet_lifetime: u16,
    },
}

impl DataChannelMode {
    /// Check if this mode provides reliable delivery
    pub fn is_reliable(&self) -> bool {
        matches!(self, Self::Reliable | Self::ReliableUnordered)
    }

    /// Check if this mode maintains ordering
    pub fn is_ordered(&self) -> bool {
        matches!(self, Self::Reliable)
    }

    /// Get SCTP parameters for this mode
    pub fn sctp_params(&self) -> (bool, Option<u16>, Option<u16>) {
        match self {
            Self::Reliable => (true, None, None),
            Self::Unreliable => (false, Some(0), None),
            Self::ReliableUnordered => (false, None, None),
            Self::PartialReliable { max_retransmits } => (false, Some(*max_retransmits), None),
            Self::TimeLimited { max_packet_lifetime } => (false, None, Some(*max_packet_lifetime)),
        }
    }
}

/// WebRTC transport configuration
#[derive(Debug, Clone)]
pub struct WebRtcConfig {
    /// ICE servers (STUN/TURN)
    pub ice_servers: Vec<IceServerConfig>,
    /// Default data channel mode
    pub default_mode: DataChannelMode,
    /// Connection timeout
    pub connection_timeout: Duration,
    /// ICE gathering timeout
    pub ice_gathering_timeout: Duration,
    /// Data channel buffer size
    pub buffer_size: usize,
    /// Enable trickle ICE (send candidates as they're discovered)
    pub trickle_ice: bool,
    /// Maximum message size for data channels
    pub max_message_size: usize,
    /// Keep-alive interval for data channels
    pub keepalive_interval: Duration,
}

impl Default for WebRtcConfig {
    fn default() -> Self {
        Self {
            ice_servers: vec![
                IceServerConfig {
                    urls: vec!["stun:stun.l.google.com:19302".to_string()],
                    username: None,
                    credential: None,
                },
            ],
            default_mode: DataChannelMode::Reliable,
            connection_timeout: Duration::from_secs(30),
            ice_gathering_timeout: Duration::from_secs(10),
            buffer_size: 64 * 1024, // 64KB
            trickle_ice: true,
            max_message_size: 256 * 1024, // 256KB
            keepalive_interval: Duration::from_secs(5),
        }
    }
}

impl WebRtcConfig {
    /// Create config optimized for sensor data (low latency, some loss acceptable)
    pub fn sensor_streaming() -> Self {
        Self {
            default_mode: DataChannelMode::Unreliable,
            buffer_size: 16 * 1024,
            max_message_size: 64 * 1024,
            ..Default::default()
        }
    }

    /// Create config for reliable command/control
    pub fn control_plane() -> Self {
        Self {
            default_mode: DataChannelMode::Reliable,
            buffer_size: 8 * 1024,
            max_message_size: 4 * 1024,
            ..Default::default()
        }
    }

    /// Create config for browser connections
    pub fn browser_compatible() -> Self {
        Self {
            // Most browsers have good STUN support
            ice_servers: vec![
                IceServerConfig {
                    urls: vec![
                        "stun:stun.l.google.com:19302".to_string(),
                        "stun:stun1.l.google.com:19302".to_string(),
                    ],
                    username: None,
                    credential: None,
                },
            ],
            default_mode: DataChannelMode::Reliable,
            trickle_ice: true,
            max_message_size: 64 * 1024, // Browser-safe limit
            ..Default::default()
        }
    }

    /// Add a TURN server for symmetric NAT traversal
    pub fn with_turn_server(mut self, url: &str, username: &str, credential: &str) -> Self {
        self.ice_servers.push(IceServerConfig {
            urls: vec![url.to_string()],
            username: Some(username.to_string()),
            credential: Some(credential.to_string()),
        });
        self
    }
}

/// ICE server configuration
#[derive(Debug, Clone)]
pub struct IceServerConfig {
    /// Server URLs (stun:host:port or turn:host:port)
    pub urls: Vec<String>,
    /// Username for TURN authentication
    pub username: Option<String>,
    /// Credential for TURN authentication
    pub credential: Option<String>,
}

impl From<&TurnServer> for IceServerConfig {
    fn from(turn: &TurnServer) -> Self {
        Self {
            urls: vec![format!("turn:{}", turn.address)],
            username: Some(turn.username.clone()),
            credential: Some(turn.credential.clone()),
        }
    }
}

/// WebRTC transport statistics
#[derive(Debug, Default)]
pub struct WebRtcStats {
    /// Total connections established
    pub connections_established: AtomicU64,
    /// Total connections failed
    pub connections_failed: AtomicU64,
    /// Active connections
    pub active_connections: AtomicU64,
    /// Total messages sent
    pub messages_sent: AtomicU64,
    /// Total messages received
    pub messages_received: AtomicU64,
    /// Total bytes sent
    pub bytes_sent: AtomicU64,
    /// Total bytes received
    pub bytes_received: AtomicU64,
    /// Data channels opened
    pub channels_opened: AtomicU64,
    /// Data channels closed
    pub channels_closed: AtomicU64,
    /// ICE connection state changes
    pub ice_state_changes: AtomicU64,
    /// ICE candidates gathered
    pub ice_candidates_gathered: AtomicU64,
}

impl WebRtcStats {
    /// Get active connection count
    pub fn active_count(&self) -> u64 {
        self.active_connections.load(Ordering::Relaxed)
    }

    /// Get total messages
    pub fn total_messages(&self) -> u64 {
        self.messages_sent.load(Ordering::Relaxed) + self.messages_received.load(Ordering::Relaxed)
    }

    /// Get total bytes
    pub fn total_bytes(&self) -> u64 {
        self.bytes_sent.load(Ordering::Relaxed) + self.bytes_received.load(Ordering::Relaxed)
    }
}

/// WebRTC data channel handle
#[derive(Debug)]
pub struct DataChannelHandle {
    /// Channel label (identifier)
    pub label: String,
    /// Channel ID
    pub id: u16,
    /// Channel mode
    pub mode: DataChannelMode,
    /// Connected peer ID
    pub peer_id: PeerId,
    /// Channel is open
    open: Arc<AtomicBool>,
    /// Send queue
    #[cfg(feature = "webrtc-transport")]
    send_tx: mpsc::Sender<Vec<u8>>,
    #[cfg(not(feature = "webrtc-transport"))]
    send_tx: mpsc::Sender<Vec<u8>>,
    /// Receive queue
    recv_rx: mpsc::Receiver<Vec<u8>>,
}

impl DataChannelHandle {
    /// Check if channel is open
    pub fn is_open(&self) -> bool {
        self.open.load(Ordering::Relaxed)
    }

    /// Send data on the channel
    pub async fn send(&self, data: &[u8]) -> io::Result<()> {
        if !self.is_open() {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "Data channel is closed",
            ));
        }

        self.send_tx
            .send(data.to_vec())
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::BrokenPipe, e))
    }

    /// Receive data from the channel
    pub async fn recv(&mut self) -> io::Result<Vec<u8>> {
        self.recv_rx
            .recv()
            .await
            .ok_or_else(|| io::Error::new(io::ErrorKind::BrokenPipe, "Channel closed"))
    }

    /// Try to receive data without blocking
    pub fn try_recv(&mut self) -> Option<Vec<u8>> {
        self.recv_rx.try_recv().ok()
    }

    /// Close the channel
    pub fn close(&self) {
        self.open.store(false, Ordering::Relaxed);
    }
}

/// Signaling data for WebRTC connection establishment
#[derive(Debug, Clone)]
pub enum WebRtcSignal {
    /// SDP offer
    Offer {
        peer_id: PeerId,
        sdp: String,
    },
    /// SDP answer
    Answer {
        peer_id: PeerId,
        sdp: String,
    },
    /// ICE candidate
    IceCandidate {
        peer_id: PeerId,
        candidate: String,
        sdp_mid: Option<String>,
        sdp_mline_index: Option<u16>,
    },
    /// ICE gathering complete
    IceGatheringComplete {
        peer_id: PeerId,
    },
}

/// Connection request for establishing WebRTC peer connection
#[derive(Debug)]
pub struct ConnectionRequest {
    /// Target peer ID
    pub peer_id: PeerId,
    /// Data channel label
    pub channel_label: String,
    /// Data channel mode
    pub mode: DataChannelMode,
    /// Response channel
    pub response_tx: oneshot::Sender<io::Result<DataChannelHandle>>,
}

/// WebRTC transport manager
///
/// Manages WebRTC peer connections and data channels for HORUS nodes.
#[cfg(feature = "webrtc-transport")]
pub struct WebRtcTransport {
    /// Configuration
    config: WebRtcConfig,
    /// Our peer ID
    local_peer_id: PeerId,
    /// Statistics
    stats: Arc<WebRtcStats>,
    /// Active peer connections
    connections: Arc<RwLock<HashMap<String, Arc<RTCPeerConnection>>>>,
    /// Active data channels by peer
    channels: Arc<RwLock<HashMap<String, Vec<Arc<RTCDataChannel>>>>>,
    /// Shutdown flag
    shutdown: Arc<AtomicBool>,
    /// Signal sender for outgoing signaling messages
    signal_tx: mpsc::Sender<WebRtcSignal>,
    /// Signal receiver for incoming signaling messages
    signal_rx: Arc<RwLock<Option<mpsc::Receiver<WebRtcSignal>>>>,
}

#[cfg(feature = "webrtc-transport")]
impl WebRtcTransport {
    /// Create a new WebRTC transport
    pub async fn new(config: WebRtcConfig, local_peer_id: PeerId) -> io::Result<Self> {
        let (signal_tx, signal_rx) = mpsc::channel(100);

        Ok(Self {
            config,
            local_peer_id,
            stats: Arc::new(WebRtcStats::default()),
            connections: Arc::new(RwLock::new(HashMap::new())),
            channels: Arc::new(RwLock::new(HashMap::new())),
            shutdown: Arc::new(AtomicBool::new(false)),
            signal_tx,
            signal_rx: Arc::new(RwLock::new(Some(signal_rx))),
        })
    }

    /// Get the local peer ID
    pub fn local_peer_id(&self) -> &PeerId {
        &self.local_peer_id
    }

    /// Get statistics
    pub fn stats(&self) -> &WebRtcStats {
        &self.stats
    }

    /// Get signal sender for signaling integration
    pub fn signal_sender(&self) -> mpsc::Sender<WebRtcSignal> {
        self.signal_tx.clone()
    }

    /// Take the signal receiver (can only be called once)
    pub fn take_signal_receiver(&self) -> Option<mpsc::Receiver<WebRtcSignal>> {
        self.signal_rx.write().take()
    }

    /// Create a WebRTC API instance
    async fn create_api(&self) -> io::Result<webrtc::api::API> {
        let mut media_engine = MediaEngine::default();

        let mut registry = Registry::new();
        registry = register_default_interceptors(registry, &mut media_engine)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        let api = APIBuilder::new()
            .with_media_engine(media_engine)
            .with_interceptor_registry(registry)
            .build();

        Ok(api)
    }

    /// Create RTCConfiguration from our config
    fn create_rtc_config(&self) -> RTCConfiguration {
        RTCConfiguration {
            ice_servers: self
                .config
                .ice_servers
                .iter()
                .map(|s| RTCIceServer {
                    urls: s.urls.clone(),
                    username: s.username.clone().unwrap_or_default(),
                    credential: s.credential.clone().unwrap_or_default(),
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        }
    }

    /// Create a new peer connection and initiate connection (as offerer)
    pub async fn connect(
        &self,
        peer_id: &PeerId,
        channel_label: &str,
        mode: DataChannelMode,
    ) -> io::Result<DataChannelHandle> {
        let api = self.create_api().await?;
        let config = self.create_rtc_config();

        let peer_connection = Arc::new(
            api.new_peer_connection(config)
                .await
                .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?,
        );

        // Set up ICE candidate handler
        let signal_tx = self.signal_tx.clone();
        let peer_id_clone = peer_id.clone();
        let stats = self.stats.clone();

        peer_connection.on_ice_candidate(Box::new(move |candidate| {
            if let Some(c) = candidate {
                stats.ice_candidates_gathered.fetch_add(1, Ordering::Relaxed);
                let signal_tx = signal_tx.clone();
                let peer_id = peer_id_clone.clone();
                Box::pin(async move {
                    let candidate_str = c.to_json()
                        .map(|j| j.candidate)
                        .unwrap_or_default();
                    let _ = signal_tx
                        .send(WebRtcSignal::IceCandidate {
                            peer_id,
                            candidate: candidate_str,
                            sdp_mid: c.to_json().ok().and_then(|j| j.sdp_mid),
                            sdp_mline_index: c.to_json().ok().and_then(|j| j.sdp_mline_index),
                        })
                        .await;
                })
            } else {
                // ICE gathering complete
                let signal_tx = signal_tx.clone();
                let peer_id = peer_id_clone.clone();
                Box::pin(async move {
                    let _ = signal_tx
                        .send(WebRtcSignal::IceGatheringComplete { peer_id })
                        .await;
                })
            }
        }));

        // Set up ICE connection state handler
        let stats_clone = self.stats.clone();
        peer_connection.on_ice_connection_state_change(Box::new(move |state| {
            stats_clone.ice_state_changes.fetch_add(1, Ordering::Relaxed);
            log::debug!("ICE connection state changed to: {:?}", state);
            Box::pin(async {})
        }));

        // Create data channel
        let (ordered, max_retransmits, max_packet_lifetime) = mode.sctp_params();
        let dc_init = RTCDataChannelInit {
            ordered: Some(ordered),
            max_retransmits,
            max_packet_life_time: max_packet_lifetime,
            ..Default::default()
        };

        let data_channel = peer_connection
            .create_data_channel(channel_label, Some(dc_init))
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        self.stats.channels_opened.fetch_add(1, Ordering::Relaxed);

        // Set up channel message handlers
        let (send_tx, mut send_rx) = mpsc::channel::<Vec<u8>>(self.config.buffer_size / 1024);
        let (recv_tx, recv_rx) = mpsc::channel::<Vec<u8>>(self.config.buffer_size / 1024);
        let open = Arc::new(AtomicBool::new(false));
        let open_clone = open.clone();

        // Handle channel open
        data_channel.on_open(Box::new(move || {
            open_clone.store(true, Ordering::Relaxed);
            log::debug!("Data channel opened");
            Box::pin(async {})
        }));

        // Handle incoming messages
        let recv_tx_clone = recv_tx.clone();
        let stats_recv = self.stats.clone();
        data_channel.on_message(Box::new(move |msg: DataChannelMessage| {
            stats_recv.messages_received.fetch_add(1, Ordering::Relaxed);
            stats_recv.bytes_received.fetch_add(msg.data.len() as u64, Ordering::Relaxed);
            let recv_tx = recv_tx_clone.clone();
            Box::pin(async move {
                let _ = recv_tx.send(msg.data.to_vec()).await;
            })
        }));

        // Handle channel close
        let open_close = open.clone();
        let stats_close = self.stats.clone();
        data_channel.on_close(Box::new(move || {
            open_close.store(false, Ordering::Relaxed);
            stats_close.channels_closed.fetch_add(1, Ordering::Relaxed);
            log::debug!("Data channel closed");
            Box::pin(async {})
        }));

        // Spawn send task
        let dc_clone = data_channel.clone();
        let stats_send = self.stats.clone();
        tokio::spawn(async move {
            while let Some(data) = send_rx.recv().await {
                stats_send.messages_sent.fetch_add(1, Ordering::Relaxed);
                stats_send.bytes_sent.fetch_add(data.len() as u64, Ordering::Relaxed);
                if dc_clone.send(&Bytes::from(data)).await.is_err() {
                    break;
                }
            }
        });

        // Create and send offer
        let offer = peer_connection
            .create_offer(None)
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        peer_connection
            .set_local_description(offer.clone())
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        // Send offer via signaling
        self.signal_tx
            .send(WebRtcSignal::Offer {
                peer_id: peer_id.clone(),
                sdp: offer.sdp,
            })
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        // Store connection
        self.connections
            .write()
            .insert(peer_id.short_id.clone(), peer_connection);
        self.channels
            .write()
            .entry(peer_id.short_id.clone())
            .or_default()
            .push(data_channel);

        self.stats.connections_established.fetch_add(1, Ordering::Relaxed);
        self.stats.active_connections.fetch_add(1, Ordering::Relaxed);

        Ok(DataChannelHandle {
            label: channel_label.to_string(),
            id: 0, // Will be assigned by SCTP
            mode,
            peer_id: peer_id.clone(),
            open,
            send_tx,
            recv_rx,
        })
    }

    /// Handle incoming offer and create answer (as answerer)
    pub async fn handle_offer(
        &self,
        peer_id: &PeerId,
        sdp: &str,
        channel_label: &str,
        mode: DataChannelMode,
    ) -> io::Result<DataChannelHandle> {
        let api = self.create_api().await?;
        let config = self.create_rtc_config();

        let peer_connection = Arc::new(
            api.new_peer_connection(config)
                .await
                .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?,
        );

        // Set up ICE candidate handler
        let signal_tx = self.signal_tx.clone();
        let peer_id_clone = peer_id.clone();
        let stats = self.stats.clone();

        peer_connection.on_ice_candidate(Box::new(move |candidate| {
            if let Some(c) = candidate {
                stats.ice_candidates_gathered.fetch_add(1, Ordering::Relaxed);
                let signal_tx = signal_tx.clone();
                let peer_id = peer_id_clone.clone();
                Box::pin(async move {
                    let candidate_str = c.to_json()
                        .map(|j| j.candidate)
                        .unwrap_or_default();
                    let _ = signal_tx
                        .send(WebRtcSignal::IceCandidate {
                            peer_id,
                            candidate: candidate_str,
                            sdp_mid: c.to_json().ok().and_then(|j| j.sdp_mid),
                            sdp_mline_index: c.to_json().ok().and_then(|j| j.sdp_mline_index),
                        })
                        .await;
                })
            } else {
                let signal_tx = signal_tx.clone();
                let peer_id = peer_id_clone.clone();
                Box::pin(async move {
                    let _ = signal_tx
                        .send(WebRtcSignal::IceGatheringComplete { peer_id })
                        .await;
                })
            }
        }));

        // Prepare channel handle components
        // Note: send_rx is unused here because as answerer we receive the channel from the remote peer
        let (send_tx, _send_rx) = mpsc::channel::<Vec<u8>>(self.config.buffer_size / 1024);
        let (recv_tx, recv_rx) = mpsc::channel::<Vec<u8>>(self.config.buffer_size / 1024);
        let open = Arc::new(AtomicBool::new(false));
        // These are kept for potential future use in channel setup
        let _channel_label_clone = channel_label.to_string();
        let _mode_clone = mode;
        let _stats_dc = self.stats.clone();

        // Handle incoming data channel (as answerer)
        let recv_tx_clone = recv_tx.clone();
        let open_clone = open.clone();
        let stats_open = self.stats.clone();

        peer_connection.on_data_channel(Box::new(move |dc: Arc<RTCDataChannel>| {
            let recv_tx = recv_tx_clone.clone();
            let open = open_clone.clone();
            let stats = stats_open.clone();

            stats.channels_opened.fetch_add(1, Ordering::Relaxed);

            dc.on_open(Box::new({
                let open = open.clone();
                move || {
                    open.store(true, Ordering::Relaxed);
                    log::debug!("Data channel opened (answerer)");
                    Box::pin(async {})
                }
            }));

            dc.on_message(Box::new({
                let recv_tx = recv_tx.clone();
                let stats = stats.clone();
                move |msg: DataChannelMessage| {
                    stats.messages_received.fetch_add(1, Ordering::Relaxed);
                    stats.bytes_received.fetch_add(msg.data.len() as u64, Ordering::Relaxed);
                    let recv_tx = recv_tx.clone();
                    Box::pin(async move {
                        let _ = recv_tx.send(msg.data.to_vec()).await;
                    })
                }
            }));

            dc.on_close(Box::new({
                let open = open.clone();
                let stats = stats.clone();
                move || {
                    open.store(false, Ordering::Relaxed);
                    stats.channels_closed.fetch_add(1, Ordering::Relaxed);
                    log::debug!("Data channel closed (answerer)");
                    Box::pin(async {})
                }
            }));

            Box::pin(async {})
        }));

        // Set remote description (offer)
        let offer = RTCSessionDescription::offer(sdp.to_string())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        peer_connection
            .set_remote_description(offer)
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        // Create and send answer
        let answer = peer_connection
            .create_answer(None)
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        peer_connection
            .set_local_description(answer.clone())
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        // Send answer via signaling
        self.signal_tx
            .send(WebRtcSignal::Answer {
                peer_id: peer_id.clone(),
                sdp: answer.sdp,
            })
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        // Store connection
        self.connections
            .write()
            .insert(peer_id.short_id.clone(), peer_connection);

        self.stats.connections_established.fetch_add(1, Ordering::Relaxed);
        self.stats.active_connections.fetch_add(1, Ordering::Relaxed);

        Ok(DataChannelHandle {
            label: channel_label.to_string(),
            id: 0,
            mode,
            peer_id: peer_id.clone(),
            open,
            send_tx,
            recv_rx,
        })
    }

    /// Handle incoming answer
    pub async fn handle_answer(&self, peer_id: &PeerId, sdp: &str) -> io::Result<()> {
        let connections = self.connections.read();
        let pc = connections
            .get(&peer_id.short_id)
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Peer connection not found"))?;

        let answer = RTCSessionDescription::answer(sdp.to_string())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        pc.set_remote_description(answer)
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))
    }

    /// Add ICE candidate from remote peer
    pub async fn add_ice_candidate(
        &self,
        peer_id: &PeerId,
        candidate: &str,
        sdp_mid: Option<&str>,
        sdp_mline_index: Option<u16>,
    ) -> io::Result<()> {
        let connections = self.connections.read();
        let pc = connections
            .get(&peer_id.short_id)
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Peer connection not found"))?;

        let ice_candidate = RTCIceCandidateInit {
            candidate: candidate.to_string(),
            sdp_mid: sdp_mid.map(|s| s.to_string()),
            sdp_mline_index,
            ..Default::default()
        };

        pc.add_ice_candidate(ice_candidate)
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))
    }

    /// Close connection to a peer
    pub async fn close_connection(&self, peer_id: &PeerId) -> io::Result<()> {
        let mut connections = self.connections.write();
        if let Some(pc) = connections.remove(&peer_id.short_id) {
            pc.close()
                .await
                .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
            self.stats.active_connections.fetch_sub(1, Ordering::Relaxed);
        }

        self.channels.write().remove(&peer_id.short_id);
        Ok(())
    }

    /// Check if connected to a peer
    pub fn is_connected(&self, peer_id: &PeerId) -> bool {
        self.connections.read().contains_key(&peer_id.short_id)
    }

    /// Get list of connected peers
    pub fn connected_peers(&self) -> Vec<String> {
        self.connections.read().keys().cloned().collect()
    }

    /// Shutdown the transport
    pub async fn shutdown(&self) {
        self.shutdown.store(true, Ordering::Relaxed);

        let peer_ids: Vec<_> = self.connections.read().keys().cloned().collect();
        for peer_id in peer_ids {
            let _ = self.close_connection(&PeerId::from_short_id(&peer_id)).await;
        }
    }
}

/// Parse WebRTC endpoint location string
pub fn parse_webrtc_location(location: &str) -> Option<(PeerId, DataChannelMode)> {
    // Format: peer-id or peer-id/mode
    let parts: Vec<&str> = location.split('/').collect();

    if parts.is_empty() {
        return None;
    }

    let peer_id = PeerId::from_short_id(parts[0]);

    let mode = if parts.len() > 1 {
        match parts[1].to_lowercase().as_str() {
            "reliable" => DataChannelMode::Reliable,
            "unreliable" => DataChannelMode::Unreliable,
            "unordered" => DataChannelMode::ReliableUnordered,
            _ => DataChannelMode::default(),
        }
    } else {
        DataChannelMode::default()
    };

    Some((peer_id, mode))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_data_channel_mode_defaults() {
        assert!(DataChannelMode::default().is_reliable());
        assert!(DataChannelMode::default().is_ordered());
    }

    #[test]
    fn test_data_channel_mode_properties() {
        assert!(DataChannelMode::Reliable.is_reliable());
        assert!(DataChannelMode::Reliable.is_ordered());

        assert!(!DataChannelMode::Unreliable.is_reliable());
        assert!(!DataChannelMode::Unreliable.is_ordered());

        assert!(DataChannelMode::ReliableUnordered.is_reliable());
        assert!(!DataChannelMode::ReliableUnordered.is_ordered());
    }

    #[test]
    fn test_data_channel_mode_sctp_params() {
        let (ordered, max_retransmits, max_lifetime) = DataChannelMode::Reliable.sctp_params();
        assert!(ordered);
        assert!(max_retransmits.is_none());
        assert!(max_lifetime.is_none());

        let (ordered, max_retransmits, max_lifetime) = DataChannelMode::Unreliable.sctp_params();
        assert!(!ordered);
        assert_eq!(max_retransmits, Some(0));
        assert!(max_lifetime.is_none());

        let (ordered, max_retransmits, max_lifetime) =
            DataChannelMode::PartialReliable { max_retransmits: 5 }.sctp_params();
        assert!(!ordered);
        assert_eq!(max_retransmits, Some(5));
        assert!(max_lifetime.is_none());
    }

    #[test]
    fn test_webrtc_config_defaults() {
        let config = WebRtcConfig::default();
        assert!(!config.ice_servers.is_empty());
        assert!(config.trickle_ice);
        assert_eq!(config.default_mode, DataChannelMode::Reliable);
    }

    #[test]
    fn test_webrtc_config_presets() {
        let sensor = WebRtcConfig::sensor_streaming();
        assert_eq!(sensor.default_mode, DataChannelMode::Unreliable);

        let control = WebRtcConfig::control_plane();
        assert_eq!(control.default_mode, DataChannelMode::Reliable);

        let browser = WebRtcConfig::browser_compatible();
        assert!(browser.trickle_ice);
    }

    #[test]
    fn test_parse_webrtc_location() {
        let (peer_id, mode) = parse_webrtc_location("a3f7-k2m9-p4n8").unwrap();
        assert_eq!(peer_id.short_id, "a3f7-k2m9-p4n8");
        assert_eq!(mode, DataChannelMode::Reliable);

        let (peer_id, mode) = parse_webrtc_location("a3f7-k2m9-p4n8/unreliable").unwrap();
        assert_eq!(peer_id.short_id, "a3f7-k2m9-p4n8");
        assert_eq!(mode, DataChannelMode::Unreliable);

        let (peer_id, mode) = parse_webrtc_location("a3f7-k2m9-p4n8/unordered").unwrap();
        assert_eq!(peer_id.short_id, "a3f7-k2m9-p4n8");
        assert_eq!(mode, DataChannelMode::ReliableUnordered);
    }

    #[test]
    fn test_ice_server_config_from_turn() {
        let turn = TurnServer {
            address: "turn.example.com:3478".to_string(),
            username: "user".to_string(),
            credential: "pass".to_string(),
        };

        let ice_config = IceServerConfig::from(&turn);
        assert_eq!(ice_config.urls, vec!["turn:turn.example.com:3478"]);
        assert_eq!(ice_config.username, Some("user".to_string()));
        assert_eq!(ice_config.credential, Some("pass".to_string()));
    }

    #[test]
    fn test_webrtc_stats_default() {
        let stats = WebRtcStats::default();
        assert_eq!(stats.active_count(), 0);
        assert_eq!(stats.total_messages(), 0);
        assert_eq!(stats.total_bytes(), 0);
    }
}
