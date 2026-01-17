//! Built-in WebSocket signaling server for HORUS P2P connections
//!
//! This module provides a lightweight signaling server that enables:
//! - P2P connection coordination via WebSocket
//! - Room-based peer matching for multi-robot scenarios
//! - Optional shared secret authentication
//! - Embedded mode (run in existing process) or standalone mode
//!
//! # Features
//!
//! - **WebSocket transport** - Standard WebSocket for browser compatibility
//! - **Room support** - Peers in the same room can discover each other
//! - **Authentication** - Optional shared secret for private networks
//! - **Scalable** - Supports 1000+ concurrent peers using async I/O
//! - **Embeddable** - Can run as part of horus_manager or standalone
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::network::signaling_server::*;
//!
//! // Create and start signaling server
//! let config = SignalingServerConfig {
//!     bind_addr: "0.0.0.0:8765".parse().unwrap(),
//!     ..Default::default()
//! };
//!
//! let server = SignalingServer::new(config);
//! server.start().await?;
//! ```
//!
//! Requires the `signaling-server` feature to be enabled.

use std::io;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;

#[cfg(feature = "signaling-server")]
use dashmap::DashMap;
#[cfg(feature = "signaling-server")]
use futures::{SinkExt, StreamExt};
#[cfg(feature = "signaling-server")]
use tokio_tungstenite::tungstenite::Message;
#[cfg(feature = "signaling-server")]
use tokio_tungstenite::accept_async;

use crate::communication::network::p2p::{IceCandidate, IceCandidateType, IceProtocol};

/// Default signaling server port
pub const DEFAULT_SIGNAL_PORT: u16 = 8765;

/// Signaling server configuration
#[derive(Debug, Clone)]
pub struct SignalingServerConfig {
    /// Address to bind the server to
    pub bind_addr: SocketAddr,
    /// Optional shared secret for authentication
    pub shared_secret: Option<String>,
    /// Maximum number of peers allowed
    pub max_peers: usize,
    /// Maximum number of rooms
    pub max_rooms: usize,
    /// Peer timeout (disconnect if no ping)
    pub peer_timeout: Duration,
    /// Ping interval for keep-alive
    pub ping_interval: Duration,
    /// Enable verbose logging
    pub verbose: bool,
}

impl Default for SignalingServerConfig {
    fn default() -> Self {
        Self {
            bind_addr: SocketAddr::from(([0, 0, 0, 0], DEFAULT_SIGNAL_PORT)),
            shared_secret: None,
            max_peers: 10000,
            max_rooms: 1000,
            peer_timeout: Duration::from_secs(60),
            ping_interval: Duration::from_secs(30),
            verbose: false,
        }
    }
}

impl SignalingServerConfig {
    /// Create config for local development (no auth, localhost only)
    pub fn local_dev() -> Self {
        Self {
            bind_addr: SocketAddr::from(([127, 0, 0, 1], DEFAULT_SIGNAL_PORT)),
            shared_secret: None,
            ..Default::default()
        }
    }

    /// Create config with authentication
    pub fn with_auth(bind_addr: SocketAddr, secret: &str) -> Self {
        Self {
            bind_addr,
            shared_secret: Some(secret.to_string()),
            ..Default::default()
        }
    }
}

/// Wire protocol messages (JSON serialized)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum WireMessage {
    /// Register with the signaling server
    Register {
        peer_id: String,
        room: Option<String>,
        auth_token: Option<String>,
    },
    /// Registration acknowledged
    Registered {
        peer_id: String,
        room: String,
    },
    /// Join a room
    JoinRoom {
        room: String,
    },
    /// Room joined confirmation
    RoomJoined {
        room: String,
        peers: Vec<String>,
    },
    /// Leave a room
    LeaveRoom {
        room: String,
    },
    /// List peers in current room
    ListPeers,
    /// Peer list response
    PeerList {
        room: String,
        peers: Vec<String>,
    },
    /// Request connection to peer
    Connect {
        target_peer_id: String,
        topics: Vec<String>,
    },
    /// Incoming call notification
    IncomingCall {
        from_peer_id: String,
        topics: Vec<String>,
    },
    /// Accept connection
    Accept {
        peer_id: String,
    },
    /// Reject connection
    Reject {
        peer_id: String,
        reason: Option<String>,
    },
    /// Exchange peer info
    PeerInfo {
        peer_id: String,
        local_addrs: Vec<String>,
        public_addr: Option<String>,
    },
    /// Exchange ICE candidates
    IceCandidates {
        peer_id: String,
        candidates: Vec<WireIceCandidate>,
    },
    /// SDP offer for WebRTC
    SdpOffer {
        peer_id: String,
        sdp: String,
    },
    /// SDP answer for WebRTC
    SdpAnswer {
        peer_id: String,
        sdp: String,
    },
    /// Connection established
    Connected {
        peer_id: String,
    },
    /// Peer disconnected
    Disconnected {
        peer_id: String,
    },
    /// Peer joined room
    PeerJoined {
        peer_id: String,
        room: String,
    },
    /// Peer left room
    PeerLeft {
        peer_id: String,
        room: String,
    },
    /// Error message
    Error {
        code: String,
        message: String,
    },
    /// Keep-alive ping
    Ping,
    /// Keep-alive pong
    Pong,
}

/// Wire format for ICE candidate
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WireIceCandidate {
    pub candidate_type: String,
    pub protocol: String,
    pub address: String,
    pub priority: u32,
    pub foundation: String,
    pub component: u8,
}

impl From<&IceCandidate> for WireIceCandidate {
    fn from(c: &IceCandidate) -> Self {
        Self {
            candidate_type: match c.candidate_type {
                IceCandidateType::Host => "host",
                IceCandidateType::ServerReflexive => "srflx",
                IceCandidateType::PeerReflexive => "prflx",
                IceCandidateType::Relay => "relay",
            }
            .to_string(),
            protocol: match c.protocol {
                IceProtocol::Udp => "udp",
                IceProtocol::Tcp => "tcp",
            }
            .to_string(),
            address: c.address.to_string(),
            priority: c.priority,
            foundation: c.foundation.clone(),
            component: c.component,
        }
    }
}

/// Signaling server statistics
#[derive(Debug, Default)]
pub struct SignalingServerStats {
    /// Total connections received
    pub total_connections: AtomicU64,
    /// Current active peers
    pub active_peers: AtomicU64,
    /// Current active rooms
    pub active_rooms: AtomicU64,
    /// Total messages relayed
    pub messages_relayed: AtomicU64,
    /// Total bytes relayed
    pub bytes_relayed: AtomicU64,
    /// Authentication failures
    pub auth_failures: AtomicU64,
    /// Peers rejected (max limit)
    pub peers_rejected: AtomicU64,
}

impl SignalingServerStats {
    /// Get a snapshot of current stats
    pub fn snapshot(&self) -> SignalingStatsSnapshot {
        SignalingStatsSnapshot {
            total_connections: self.total_connections.load(Ordering::Relaxed),
            active_peers: self.active_peers.load(Ordering::Relaxed),
            active_rooms: self.active_rooms.load(Ordering::Relaxed),
            messages_relayed: self.messages_relayed.load(Ordering::Relaxed),
            bytes_relayed: self.bytes_relayed.load(Ordering::Relaxed),
            auth_failures: self.auth_failures.load(Ordering::Relaxed),
            peers_rejected: self.peers_rejected.load(Ordering::Relaxed),
        }
    }
}

/// Snapshot of signaling server stats
#[derive(Debug, Clone)]
pub struct SignalingStatsSnapshot {
    pub total_connections: u64,
    pub active_peers: u64,
    pub active_rooms: u64,
    pub messages_relayed: u64,
    pub bytes_relayed: u64,
    pub auth_failures: u64,
    pub peers_rejected: u64,
}

/// Connected peer state
#[cfg(feature = "signaling-server")]
#[allow(dead_code)] // remote_addr kept for debugging/logging
struct PeerState {
    /// Peer ID
    peer_id: String,
    /// Current room
    room: String,
    /// Sender channel to peer's WebSocket
    sender: mpsc::Sender<WireMessage>,
    /// Last activity time
    last_activity: Instant,
    /// Remote address
    remote_addr: SocketAddr,
}

/// Room state
#[cfg(feature = "signaling-server")]
#[allow(dead_code)] // created_at kept for debugging/metrics
struct RoomState {
    /// Peer IDs in this room
    peers: Vec<String>,
    /// Room creation time
    created_at: Instant,
}

/// Signaling server
#[cfg(feature = "signaling-server")]
pub struct SignalingServer {
    /// Configuration
    config: SignalingServerConfig,
    /// Connected peers (peer_id -> state)
    peers: Arc<DashMap<String, PeerState>>,
    /// Rooms (room_name -> state)
    rooms: Arc<DashMap<String, RoomState>>,
    /// Server statistics
    stats: Arc<SignalingServerStats>,
    /// Shutdown flag
    shutdown: Arc<AtomicBool>,
}

#[cfg(feature = "signaling-server")]
impl SignalingServer {
    /// Create a new signaling server
    pub fn new(config: SignalingServerConfig) -> Self {
        Self {
            config,
            peers: Arc::new(DashMap::new()),
            rooms: Arc::new(DashMap::new()),
            stats: Arc::new(SignalingServerStats::default()),
            shutdown: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Start the signaling server
    pub async fn start(&self) -> io::Result<()> {
        let listener = TcpListener::bind(&self.config.bind_addr).await?;
        log::info!(
            "Signaling server listening on ws://{}",
            self.config.bind_addr
        );

        // Spawn keep-alive task
        let peers_clone = self.peers.clone();
        let timeout = self.config.peer_timeout;
        let shutdown_clone = self.shutdown.clone();
        tokio::spawn(async move {
            Self::keepalive_loop(peers_clone, timeout, shutdown_clone).await;
        });

        // Accept connections
        while !self.shutdown.load(Ordering::Relaxed) {
            match listener.accept().await {
                Ok((stream, addr)) => {
                    self.stats.total_connections.fetch_add(1, Ordering::Relaxed);

                    // Check peer limit
                    if self.peers.len() >= self.config.max_peers {
                        log::warn!("Rejecting connection from {}: max peers reached", addr);
                        self.stats.peers_rejected.fetch_add(1, Ordering::Relaxed);
                        continue;
                    }

                    let peers = self.peers.clone();
                    let rooms = self.rooms.clone();
                    let stats = self.stats.clone();
                    let config = self.config.clone();
                    let shutdown = self.shutdown.clone();

                    tokio::spawn(async move {
                        if let Err(e) =
                            Self::handle_connection(stream, addr, peers, rooms, stats, config, shutdown)
                                .await
                        {
                            log::debug!("Connection error from {}: {}", addr, e);
                        }
                    });
                }
                Err(e) => {
                    log::error!("Failed to accept connection: {}", e);
                }
            }
        }

        Ok(())
    }

    /// Handle a single WebSocket connection
    async fn handle_connection(
        stream: TcpStream,
        addr: SocketAddr,
        peers: Arc<DashMap<String, PeerState>>,
        rooms: Arc<DashMap<String, RoomState>>,
        stats: Arc<SignalingServerStats>,
        config: SignalingServerConfig,
        shutdown: Arc<AtomicBool>,
    ) -> io::Result<()> {
        // Upgrade to WebSocket
        let ws_stream = accept_async(stream)
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        let (mut ws_sender, mut ws_receiver) = ws_stream.split();

        // Create message channel for this peer
        let (msg_tx, mut msg_rx) = mpsc::channel::<WireMessage>(256);

        // Spawn sender task
        let sender_task = tokio::spawn(async move {
            while let Some(msg) = msg_rx.recv().await {
                let json = match serde_json::to_string(&msg) {
                    Ok(j) => j,
                    Err(e) => {
                        log::error!("Failed to serialize message: {}", e);
                        continue;
                    }
                };
                if ws_sender.send(Message::Text(json)).await.is_err() {
                    break;
                }
            }
        });

        let mut peer_id: Option<String> = None;
        let mut current_room: Option<String> = None;

        // Receive and process messages
        while !shutdown.load(Ordering::Relaxed) {
            let msg = match ws_receiver.next().await {
                Some(Ok(msg)) => msg,
                Some(Err(e)) => {
                    log::debug!("WebSocket error from {}: {}", addr, e);
                    break;
                }
                None => break,
            };

            let text = match msg {
                Message::Text(t) => t,
                Message::Ping(_) => {
                    let _ = msg_tx.send(WireMessage::Pong).await;
                    continue;
                }
                Message::Pong(_) => continue,
                Message::Close(_) => break,
                _ => continue,
            };

            let wire_msg: WireMessage = match serde_json::from_str(&text) {
                Ok(m) => m,
                Err(e) => {
                    log::debug!("Invalid message from {}: {}", addr, e);
                    let _ = msg_tx
                        .send(WireMessage::Error {
                            code: "invalid_message".to_string(),
                            message: e.to_string(),
                        })
                        .await;
                    continue;
                }
            };

            stats.messages_relayed.fetch_add(1, Ordering::Relaxed);
            stats
                .bytes_relayed
                .fetch_add(text.len() as u64, Ordering::Relaxed);

            match wire_msg {
                WireMessage::Register {
                    peer_id: pid,
                    room,
                    auth_token,
                } => {
                    // Authenticate if required
                    if let Some(ref secret) = config.shared_secret {
                        if auth_token.as_deref() != Some(secret.as_str()) {
                            log::warn!("Auth failure from {} (peer_id: {})", addr, pid);
                            stats.auth_failures.fetch_add(1, Ordering::Relaxed);
                            let _ = msg_tx
                                .send(WireMessage::Error {
                                    code: "auth_failed".to_string(),
                                    message: "Invalid authentication token".to_string(),
                                })
                                .await;
                            break;
                        }
                    }

                    // Check if peer ID is already taken
                    if peers.contains_key(&pid) {
                        let _ = msg_tx
                            .send(WireMessage::Error {
                                code: "peer_exists".to_string(),
                                message: "Peer ID already registered".to_string(),
                            })
                            .await;
                        continue;
                    }

                    let room_name = room.unwrap_or_else(|| "default".to_string());

                    // Register peer
                    peers.insert(
                        pid.clone(),
                        PeerState {
                            peer_id: pid.clone(),
                            room: room_name.clone(),
                            sender: msg_tx.clone(),
                            last_activity: Instant::now(),
                            remote_addr: addr,
                        },
                    );

                    // Add to room
                    rooms
                        .entry(room_name.clone())
                        .or_insert_with(|| RoomState {
                            peers: Vec::new(),
                            created_at: Instant::now(),
                        })
                        .peers
                        .push(pid.clone());

                    stats.active_peers.fetch_add(1, Ordering::Relaxed);
                    stats
                        .active_rooms
                        .store(rooms.len() as u64, Ordering::Relaxed);

                    peer_id = Some(pid.clone());
                    current_room = Some(room_name.clone());

                    if config.verbose {
                        log::info!("Peer {} registered in room {} from {}", pid, room_name, addr);
                    }

                    // Send confirmation
                    let _ = msg_tx
                        .send(WireMessage::Registered {
                            peer_id: pid.clone(),
                            room: room_name.clone(),
                        })
                        .await;

                    // Notify other peers in room
                    if let Some(room_state) = rooms.get(&room_name) {
                        for other_pid in &room_state.peers {
                            if other_pid != &pid {
                                if let Some(other_peer) = peers.get(other_pid) {
                                    let _ = other_peer
                                        .sender
                                        .send(WireMessage::PeerJoined {
                                            peer_id: pid.clone(),
                                            room: room_name.clone(),
                                        })
                                        .await;
                                }
                            }
                        }
                    }
                }

                WireMessage::JoinRoom { room } => {
                    if let Some(ref pid) = peer_id {
                        // Leave current room
                        if let Some(ref old_room) = current_room {
                            Self::leave_room(&peers, &rooms, pid, old_room);
                        }

                        // Join new room
                        rooms
                            .entry(room.clone())
                            .or_insert_with(|| RoomState {
                                peers: Vec::new(),
                                created_at: Instant::now(),
                            })
                            .peers
                            .push(pid.clone());

                        // Update peer's room
                        if let Some(mut peer) = peers.get_mut(pid) {
                            peer.room = room.clone();
                        }

                        current_room = Some(room.clone());

                        // Get peer list
                        let peer_list: Vec<String> = rooms
                            .get(&room)
                            .map(|r| r.peers.clone())
                            .unwrap_or_default();

                        let _ = msg_tx
                            .send(WireMessage::RoomJoined {
                                room: room.clone(),
                                peers: peer_list,
                            })
                            .await;

                        // Notify others
                        if let Some(room_state) = rooms.get(&room) {
                            for other_pid in &room_state.peers {
                                if other_pid != pid {
                                    if let Some(other_peer) = peers.get(other_pid) {
                                        let _ = other_peer
                                            .sender
                                            .send(WireMessage::PeerJoined {
                                                peer_id: pid.clone(),
                                                room: room.clone(),
                                            })
                                            .await;
                                    }
                                }
                            }
                        }
                    }
                }

                WireMessage::ListPeers => {
                    if let Some(ref room) = current_room {
                        let peer_list: Vec<String> = rooms
                            .get(room)
                            .map(|r| r.peers.clone())
                            .unwrap_or_default();

                        let _ = msg_tx
                            .send(WireMessage::PeerList {
                                room: room.clone(),
                                peers: peer_list,
                            })
                            .await;
                    }
                }

                WireMessage::Connect {
                    target_peer_id,
                    topics,
                } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target) = peers.get(&target_peer_id) {
                            // Forward connection request
                            let _ = target
                                .sender
                                .send(WireMessage::IncomingCall {
                                    from_peer_id: pid.clone(),
                                    topics,
                                })
                                .await;
                        } else {
                            let _ = msg_tx
                                .send(WireMessage::Error {
                                    code: "peer_not_found".to_string(),
                                    message: format!("Peer {} not found", target_peer_id),
                                })
                                .await;
                        }
                    }
                }

                WireMessage::Accept { peer_id: target } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target_peer) = peers.get(&target) {
                            let _ = target_peer
                                .sender
                                .send(WireMessage::Accept {
                                    peer_id: pid.clone(),
                                })
                                .await;
                        }
                    }
                }

                WireMessage::Reject { peer_id: target, reason } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target_peer) = peers.get(&target) {
                            let _ = target_peer
                                .sender
                                .send(WireMessage::Reject {
                                    peer_id: pid.clone(),
                                    reason,
                                })
                                .await;
                        }
                    }
                }

                WireMessage::PeerInfo {
                    peer_id: target,
                    local_addrs,
                    public_addr,
                } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target_peer) = peers.get(&target) {
                            let _ = target_peer
                                .sender
                                .send(WireMessage::PeerInfo {
                                    peer_id: pid.clone(),
                                    local_addrs,
                                    public_addr,
                                })
                                .await;
                        }
                    }
                }

                WireMessage::IceCandidates {
                    peer_id: target,
                    candidates,
                } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target_peer) = peers.get(&target) {
                            let _ = target_peer
                                .sender
                                .send(WireMessage::IceCandidates {
                                    peer_id: pid.clone(),
                                    candidates,
                                })
                                .await;
                        }
                    }
                }

                WireMessage::SdpOffer { peer_id: target, sdp } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target_peer) = peers.get(&target) {
                            let _ = target_peer
                                .sender
                                .send(WireMessage::SdpOffer {
                                    peer_id: pid.clone(),
                                    sdp,
                                })
                                .await;
                        }
                    }
                }

                WireMessage::SdpAnswer { peer_id: target, sdp } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target_peer) = peers.get(&target) {
                            let _ = target_peer
                                .sender
                                .send(WireMessage::SdpAnswer {
                                    peer_id: pid.clone(),
                                    sdp,
                                })
                                .await;
                        }
                    }
                }

                WireMessage::Connected { peer_id: target } => {
                    if let Some(ref pid) = peer_id {
                        if let Some(target_peer) = peers.get(&target) {
                            let _ = target_peer
                                .sender
                                .send(WireMessage::Connected {
                                    peer_id: pid.clone(),
                                })
                                .await;
                        }
                    }
                }

                WireMessage::Ping => {
                    let _ = msg_tx.send(WireMessage::Pong).await;
                    if let Some(ref pid) = peer_id {
                        if let Some(mut peer) = peers.get_mut(pid) {
                            peer.last_activity = Instant::now();
                        }
                    }
                }

                WireMessage::Pong => {
                    if let Some(ref pid) = peer_id {
                        if let Some(mut peer) = peers.get_mut(pid) {
                            peer.last_activity = Instant::now();
                        }
                    }
                }

                _ => {}
            }
        }

        // Cleanup on disconnect
        if let Some(ref pid) = peer_id {
            if let Some(ref room) = current_room {
                Self::leave_room(&peers, &rooms, pid, room);
            }
            peers.remove(pid);
            stats.active_peers.fetch_sub(1, Ordering::Relaxed);

            if config.verbose {
                log::info!("Peer {} disconnected", pid);
            }
        }

        sender_task.abort();
        Ok(())
    }

    /// Remove peer from room and notify others
    fn leave_room(
        peers: &Arc<DashMap<String, PeerState>>,
        rooms: &Arc<DashMap<String, RoomState>>,
        peer_id: &str,
        room: &str,
    ) {
        let mut peers_in_room = Vec::new();

        if let Some(mut room_state) = rooms.get_mut(room) {
            room_state.peers.retain(|p| p != peer_id);
            peers_in_room = room_state.peers.clone();

            // Remove empty rooms
            if room_state.peers.is_empty() {
                drop(room_state);
                rooms.remove(room);
            }
        }

        // Notify remaining peers
        for other_pid in peers_in_room {
            if let Some(other_peer) = peers.get(&other_pid) {
                let _ = other_peer.sender.try_send(WireMessage::PeerLeft {
                    peer_id: peer_id.to_string(),
                    room: room.to_string(),
                });
            }
        }
    }

    /// Keep-alive loop to clean up stale connections
    async fn keepalive_loop(
        peers: Arc<DashMap<String, PeerState>>,
        timeout: Duration,
        shutdown: Arc<AtomicBool>,
    ) {
        let mut interval = tokio::time::interval(Duration::from_secs(10));

        while !shutdown.load(Ordering::Relaxed) {
            interval.tick().await;

            let now = Instant::now();
            let stale_peers: Vec<String> = peers
                .iter()
                .filter(|entry| now.duration_since(entry.last_activity) > timeout)
                .map(|entry| entry.peer_id.clone())
                .collect();

            for pid in stale_peers {
                log::debug!("Removing stale peer: {}", pid);
                if let Some((_, peer)) = peers.remove(&pid) {
                    let _ = peer.sender.try_send(WireMessage::Error {
                        code: "timeout".to_string(),
                        message: "Connection timed out".to_string(),
                    });
                }
            }
        }
    }

    /// Shutdown the server
    pub fn shutdown(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
    }

    /// Get the shutdown signal for external control (e.g., Ctrl+C handler)
    pub fn shutdown_signal(&self) -> Arc<AtomicBool> {
        self.shutdown.clone()
    }

    /// Get server statistics
    pub fn stats(&self) -> &SignalingServerStats {
        &self.stats
    }

    /// Get number of connected peers
    pub fn peer_count(&self) -> usize {
        self.peers.len()
    }

    /// Get number of active rooms
    pub fn room_count(&self) -> usize {
        self.rooms.len()
    }

    /// List peers in a room
    pub fn list_peers_in_room(&self, room: &str) -> Vec<String> {
        self.rooms
            .get(room)
            .map(|r| r.peers.clone())
            .unwrap_or_default()
    }

    /// Broadcast message to all peers in a room
    pub async fn broadcast_to_room(&self, room: &str, message: WireMessage) {
        if let Some(room_state) = self.rooms.get(room) {
            for pid in &room_state.peers {
                if let Some(peer) = self.peers.get(pid) {
                    let _ = peer.sender.send(message.clone()).await;
                }
            }
        }
    }
}

/// Builder for signaling server
#[derive(Default)]
pub struct SignalingServerBuilder {
    config: SignalingServerConfig,
}

impl SignalingServerBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn bind_addr(mut self, addr: SocketAddr) -> Self {
        self.config.bind_addr = addr;
        self
    }

    pub fn port(mut self, port: u16) -> Self {
        self.config.bind_addr.set_port(port);
        self
    }

    pub fn shared_secret(mut self, secret: impl Into<String>) -> Self {
        self.config.shared_secret = Some(secret.into());
        self
    }

    pub fn max_peers(mut self, max: usize) -> Self {
        self.config.max_peers = max;
        self
    }

    pub fn max_rooms(mut self, max: usize) -> Self {
        self.config.max_rooms = max;
        self
    }

    pub fn peer_timeout(mut self, timeout: Duration) -> Self {
        self.config.peer_timeout = timeout;
        self
    }

    pub fn verbose(mut self, verbose: bool) -> Self {
        self.config.verbose = verbose;
        self
    }

    #[cfg(feature = "signaling-server")]
    pub fn build(self) -> SignalingServer {
        SignalingServer::new(self.config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signaling_config_defaults() {
        let config = SignalingServerConfig::default();
        assert_eq!(config.bind_addr.port(), DEFAULT_SIGNAL_PORT);
        assert!(config.shared_secret.is_none());
        assert_eq!(config.max_peers, 10000);
    }

    #[test]
    fn test_signaling_config_local_dev() {
        let config = SignalingServerConfig::local_dev();
        assert!(config.bind_addr.ip().is_loopback());
        assert!(config.shared_secret.is_none());
    }

    #[test]
    fn test_signaling_config_with_auth() {
        let addr: SocketAddr = "0.0.0.0:9000".parse().unwrap();
        let config = SignalingServerConfig::with_auth(addr, "secret123");
        assert_eq!(config.bind_addr.port(), 9000);
        assert_eq!(config.shared_secret, Some("secret123".to_string()));
    }

    #[test]
    fn test_wire_message_serialize() {
        let msg = WireMessage::Register {
            peer_id: "a3f7-k2m9-p4n8".to_string(),
            room: Some("robots".to_string()),
            auth_token: None,
        };
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("register"));
        assert!(json.contains("a3f7-k2m9-p4n8"));
    }

    #[test]
    fn test_wire_message_deserialize() {
        let json = r#"{"type":"register","peer_id":"test-id","room":null,"auth_token":null}"#;
        let msg: WireMessage = serde_json::from_str(json).unwrap();
        match msg {
            WireMessage::Register { peer_id, .. } => {
                assert_eq!(peer_id, "test-id");
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_signaling_stats_default() {
        let stats = SignalingServerStats::default();
        assert_eq!(stats.total_connections.load(Ordering::Relaxed), 0);
        assert_eq!(stats.active_peers.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_builder() {
        let builder = SignalingServerBuilder::new()
            .port(9999)
            .shared_secret("test")
            .max_peers(100)
            .verbose(true);

        assert_eq!(builder.config.bind_addr.port(), 9999);
        assert_eq!(builder.config.shared_secret, Some("test".to_string()));
        assert_eq!(builder.config.max_peers, 100);
        assert!(builder.config.verbose);
    }

    #[test]
    fn test_wire_ice_candidate_from() {
        let ice = IceCandidate {
            candidate_type: IceCandidateType::Host,
            protocol: IceProtocol::Udp,
            address: "192.168.1.1:5000".parse().unwrap(),
            priority: 1000,
            foundation: "1".to_string(),
            component: 1,
        };
        let wire = WireIceCandidate::from(&ice);
        assert_eq!(wire.candidate_type, "host");
        assert_eq!(wire.protocol, "udp");
        assert_eq!(wire.address, "192.168.1.1:5000");
    }
}
