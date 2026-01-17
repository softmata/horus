//! ICE-lite implementation for NAT traversal and hole punching
//!
//! This module provides Interactive Connectivity Establishment (ICE) functionality
//! for establishing direct P2P connections through NATs.
//!
//! # ICE Candidate Types
//!
//! - **Host**: Local network addresses (highest priority)
//! - **Server Reflexive (srflx)**: Public addresses discovered via STUN
//! - **Peer Reflexive (prflx)**: Addresses discovered during connectivity checks
//! - **Relay**: TURN server relayed addresses (lowest priority, always works)
//!
//! # Connection Flow
//!
//! ```text
//! 1. Gather local (host) candidates
//! 2. Query STUN servers for server-reflexive candidates
//! 3. Exchange candidates via signaling channel
//! 4. Perform connectivity checks on candidate pairs
//! 5. Select best working pair and start data transfer
//! ```
//!
//! # Hole Punching
//!
//! For NAT traversal, both peers send simultaneous UDP packets:
//! - Creates outbound mapping in local NAT
//! - Peer's inbound packet uses this mapping
//! - Works with cone NATs, falls back to relay for symmetric NAT

use std::collections::HashMap;
use std::io;
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

use super::p2p::{IceCandidate, IceCandidateType, IceProtocol};
use super::stun::{NatType, StunClient, StunConfig};

// =============================================================================
// ICE Constants (RFC 8445)
// =============================================================================

/// ICE candidate foundation counter
static FOUNDATION_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Default ICE configuration values
pub const ICE_DEFAULT_TIMEOUT: Duration = Duration::from_secs(30);
pub const ICE_CONNECTIVITY_CHECK_INTERVAL: Duration = Duration::from_millis(50);
pub const ICE_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(15);
pub const ICE_CONSENT_INTERVAL: Duration = Duration::from_secs(5);

/// Priority calculation type preferences (RFC 8445 Section 5.1.2.1)
const TYPE_PREFERENCE_HOST: u32 = 126;
const TYPE_PREFERENCE_SRFLX: u32 = 100;
const TYPE_PREFERENCE_PRFLX: u32 = 110;
const TYPE_PREFERENCE_RELAY: u32 = 0;

/// Component IDs
const COMPONENT_RTP: u8 = 1;

// STUN constants for connectivity checks
const STUN_MAGIC_COOKIE: u32 = 0x2112A442;
const STUN_BINDING_REQUEST: u16 = 0x0001;
const STUN_BINDING_RESPONSE: u16 = 0x0101;
const STUN_HEADER_SIZE: usize = 20;

// STUN attribute types for ICE
const ATTR_XOR_MAPPED_ADDRESS: u16 = 0x0020;
const ATTR_PRIORITY: u16 = 0x0024;
const ATTR_USE_CANDIDATE: u16 = 0x0025;
const ATTR_ICE_CONTROLLING: u16 = 0x802A;
const ATTR_ICE_CONTROLLED: u16 = 0x8029;

// =============================================================================
// ICE Types
// =============================================================================

/// ICE agent role in the connection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IceRole {
    /// Controlling agent (initiator, makes final candidate selection)
    Controlling,
    /// Controlled agent (responder, follows controller's selection)
    Controlled,
}

impl IceRole {
    /// Determine role based on tie-breaker values
    pub fn resolve_conflict(our_tiebreaker: u64, their_tiebreaker: u64) -> Self {
        if our_tiebreaker > their_tiebreaker {
            IceRole::Controlling
        } else {
            IceRole::Controlled
        }
    }
}

/// ICE gathering state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IceGatheringState {
    /// Not started gathering
    New,
    /// Gathering candidates in progress
    Gathering,
    /// All candidates gathered
    Complete,
}

/// ICE connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IceConnectionState {
    /// Initial state
    New,
    /// Checking connectivity
    Checking,
    /// At least one pair working, still checking others
    Connected,
    /// Connectivity checks complete
    Completed,
    /// All checks failed
    Failed,
    /// Connection closed
    Closed,
}

/// State of a candidate pair
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CandidatePairState {
    /// Pair created but not checked
    Frozen,
    /// Waiting to be checked
    Waiting,
    /// Check in progress
    InProgress,
    /// Check succeeded
    Succeeded,
    /// Check failed
    Failed,
}

/// A local or remote ICE candidate
#[derive(Debug, Clone)]
pub struct IceCandidateInfo {
    /// Candidate type
    pub candidate_type: IceCandidateType,
    /// Transport protocol (UDP/TCP)
    pub protocol: IceProtocol,
    /// Network address
    pub address: SocketAddr,
    /// Candidate priority
    pub priority: u32,
    /// Foundation string (for pairing)
    pub foundation: String,
    /// Component ID (1 for RTP)
    pub component: u8,
    /// Base address (for srflx/prflx, the local socket address)
    pub base_addr: Option<SocketAddr>,
    /// Related address (STUN server for srflx)
    pub related_addr: Option<SocketAddr>,
}

impl IceCandidateInfo {
    /// Convert to IceCandidate for signaling
    pub fn to_signaling_candidate(&self) -> IceCandidate {
        IceCandidate {
            candidate_type: self.candidate_type,
            protocol: self.protocol,
            address: self.address,
            priority: self.priority,
            foundation: self.foundation.clone(),
            component: self.component,
        }
    }
}

/// A candidate pair for connectivity checking
#[derive(Debug, Clone)]
pub struct CandidatePair {
    /// Local candidate
    pub local: IceCandidateInfo,
    /// Remote candidate
    pub remote: IceCandidateInfo,
    /// Pair priority
    pub priority: u64,
    /// Current state
    pub state: CandidatePairState,
    /// Number of check attempts
    pub check_count: u32,
    /// Last check time
    pub last_check: Option<Instant>,
    /// Round-trip time (if succeeded)
    pub rtt: Option<Duration>,
    /// Is this the nominated pair?
    pub nominated: bool,
}

impl CandidatePair {
    /// Calculate pair priority (RFC 8445 Section 6.1.2.3)
    pub fn calculate_priority(role: IceRole, local_prio: u32, remote_prio: u32) -> u64 {
        let (g, d) = match role {
            IceRole::Controlling => (local_prio as u64, remote_prio as u64),
            IceRole::Controlled => (remote_prio as u64, local_prio as u64),
        };

        // priority = 2^32*MIN(G,D) + 2*MAX(G,D) + (G>D?1:0)
        let min = g.min(d);
        let max = g.max(d);
        (1u64 << 32) * min + 2 * max + if g > d { 1 } else { 0 }
    }

    /// Create a new candidate pair
    pub fn new(local: IceCandidateInfo, remote: IceCandidateInfo, role: IceRole) -> Self {
        let priority = Self::calculate_priority(role, local.priority, remote.priority);
        Self {
            local,
            remote,
            priority,
            state: CandidatePairState::Frozen,
            check_count: 0,
            last_check: None,
            rtt: None,
            nominated: false,
        }
    }
}

/// ICE agent configuration
#[derive(Debug, Clone)]
pub struct IceConfig {
    /// STUN servers for candidate gathering
    pub stun_servers: Vec<String>,
    /// Overall ICE timeout
    pub timeout: Duration,
    /// Connectivity check interval
    pub check_interval: Duration,
    /// Maximum connectivity check retries per pair
    pub max_check_retries: u32,
    /// Whether to gather host candidates
    pub gather_host: bool,
    /// Whether to gather server-reflexive candidates
    pub gather_srflx: bool,
    /// Whether to use aggressive nomination
    pub aggressive_nomination: bool,
    /// Interface filter (None = all interfaces)
    pub interface_filter: Option<Vec<String>>,
}

impl Default for IceConfig {
    fn default() -> Self {
        Self {
            stun_servers: vec![
                "stun.l.google.com:19302".to_string(),
                "stun1.l.google.com:19302".to_string(),
            ],
            timeout: ICE_DEFAULT_TIMEOUT,
            check_interval: ICE_CONNECTIVITY_CHECK_INTERVAL,
            max_check_retries: 7,
            gather_host: true,
            gather_srflx: true,
            aggressive_nomination: true,
            interface_filter: None,
        }
    }
}

/// ICE agent statistics
#[derive(Debug, Default)]
pub struct IceStats {
    /// Candidates gathered
    pub candidates_gathered: AtomicU32,
    /// Connectivity checks sent
    pub checks_sent: AtomicU64,
    /// Connectivity check responses received
    pub checks_received: AtomicU64,
    /// Successful pairs
    pub successful_pairs: AtomicU32,
    /// Failed pairs
    pub failed_pairs: AtomicU32,
    /// Selected pair RTT (microseconds)
    pub selected_rtt_us: AtomicU64,
}

// =============================================================================
// ICE Agent
// =============================================================================

/// ICE agent for managing NAT traversal and connectivity
pub struct IceAgent {
    /// Configuration
    config: IceConfig,
    /// Agent role
    role: RwLock<IceRole>,
    /// Tie-breaker for role conflicts
    tiebreaker: u64,
    /// Gathering state
    gathering_state: RwLock<IceGatheringState>,
    /// Connection state
    connection_state: RwLock<IceConnectionState>,
    /// Local candidates
    local_candidates: RwLock<Vec<IceCandidateInfo>>,
    /// Remote candidates
    remote_candidates: RwLock<Vec<IceCandidateInfo>>,
    /// Candidate pairs (sorted by priority)
    candidate_pairs: RwLock<Vec<CandidatePair>>,
    /// Selected (nominated) pair index
    selected_pair: RwLock<Option<usize>>,
    /// UDP socket for connectivity checks
    socket: Option<UdpSocket>,
    /// Detected NAT type
    nat_type: RwLock<Option<NatType>>,
    /// Statistics
    stats: Arc<IceStats>,
    /// Transaction ID -> pair index mapping for check responses
    pending_checks: Mutex<HashMap<[u8; 12], usize>>,
}

impl IceAgent {
    /// Create a new ICE agent
    pub fn new(config: IceConfig) -> io::Result<Self> {
        // Generate random tiebreaker
        let tiebreaker = generate_tiebreaker();

        // Create UDP socket for connectivity checks
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.set_nonblocking(true)?;

        Ok(Self {
            config,
            role: RwLock::new(IceRole::Controlling),
            tiebreaker,
            gathering_state: RwLock::new(IceGatheringState::New),
            connection_state: RwLock::new(IceConnectionState::New),
            local_candidates: RwLock::new(Vec::new()),
            remote_candidates: RwLock::new(Vec::new()),
            candidate_pairs: RwLock::new(Vec::new()),
            selected_pair: RwLock::new(None),
            socket: Some(socket),
            nat_type: RwLock::new(None),
            stats: Arc::new(IceStats::default()),
            pending_checks: Mutex::new(HashMap::new()),
        })
    }

    /// Create with default configuration
    pub fn with_defaults() -> io::Result<Self> {
        Self::new(IceConfig::default())
    }

    /// Set the ICE role
    pub fn set_role(&self, role: IceRole) {
        *self.role.write().unwrap() = role;
    }

    /// Get the current role
    pub fn get_role(&self) -> IceRole {
        *self.role.read().unwrap()
    }

    /// Get the tiebreaker value
    pub fn get_tiebreaker(&self) -> u64 {
        self.tiebreaker
    }

    /// Get gathering state
    pub fn gathering_state(&self) -> IceGatheringState {
        *self.gathering_state.read().unwrap()
    }

    /// Get connection state
    pub fn connection_state(&self) -> IceConnectionState {
        *self.connection_state.read().unwrap()
    }

    /// Get local candidates
    pub fn local_candidates(&self) -> Vec<IceCandidateInfo> {
        self.local_candidates.read().unwrap().clone()
    }

    /// Get local candidates for signaling
    pub fn get_signaling_candidates(&self) -> Vec<IceCandidate> {
        self.local_candidates
            .read()
            .unwrap()
            .iter()
            .map(|c| c.to_signaling_candidate())
            .collect()
    }

    /// Get detected NAT type
    pub fn nat_type(&self) -> Option<NatType> {
        *self.nat_type.read().unwrap()
    }

    /// Get statistics
    pub fn stats(&self) -> &IceStats {
        &self.stats
    }

    /// Get selected pair info
    pub fn selected_pair(&self) -> Option<(SocketAddr, SocketAddr)> {
        let selected = self.selected_pair.read().unwrap();
        let pairs = self.candidate_pairs.read().unwrap();

        selected.and_then(|idx| pairs.get(idx)).map(|pair| {
            (pair.local.address, pair.remote.address)
        })
    }

    /// Get the local address for sending data
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "Socket not initialized"))?
            .local_addr()
    }

    // =========================================================================
    // Candidate Gathering
    // =========================================================================

    /// Gather all ICE candidates
    ///
    /// This collects:
    /// 1. Host candidates (local addresses)
    /// 2. Server-reflexive candidates (via STUN)
    pub fn gather_candidates(&self) -> io::Result<()> {
        *self.gathering_state.write().unwrap() = IceGatheringState::Gathering;

        // Gather host candidates
        if self.config.gather_host {
            self.gather_host_candidates()?;
        }

        // Gather server-reflexive candidates via STUN
        if self.config.gather_srflx && !self.config.stun_servers.is_empty() {
            self.gather_srflx_candidates()?;
        }

        *self.gathering_state.write().unwrap() = IceGatheringState::Complete;

        log::info!(
            "ICE gathering complete: {} candidates",
            self.local_candidates.read().unwrap().len()
        );

        Ok(())
    }

    /// Gather host (local) candidates
    fn gather_host_candidates(&self) -> io::Result<()> {
        let local_addrs = get_local_addresses()?;

        let socket_port = self
            .socket
            .as_ref()
            .map(|s| s.local_addr().ok())
            .flatten()
            .map(|a| a.port())
            .unwrap_or(0);

        let mut candidates = self.local_candidates.write().unwrap();

        for ip in local_addrs {
            // Apply interface filter
            if let Some(ref filter) = self.config.interface_filter {
                let ip_str = ip.to_string();
                if !filter.iter().any(|f| ip_str.contains(f)) {
                    continue;
                }
            }

            // Skip loopback for external connectivity
            if ip.is_loopback() {
                continue;
            }

            let addr = SocketAddr::new(ip, socket_port);
            let priority = calculate_priority(IceCandidateType::Host, addr);
            let foundation = generate_foundation(IceCandidateType::Host, &addr, None);

            let candidate = IceCandidateInfo {
                candidate_type: IceCandidateType::Host,
                protocol: IceProtocol::Udp,
                address: addr,
                priority,
                foundation,
                component: COMPONENT_RTP,
                base_addr: Some(addr),
                related_addr: None,
            };

            candidates.push(candidate);
            self.stats.candidates_gathered.fetch_add(1, Ordering::Relaxed);
        }

        Ok(())
    }

    /// Gather server-reflexive candidates via STUN
    fn gather_srflx_candidates(&self) -> io::Result<()> {
        let stun_config = StunConfig {
            servers: self.config.stun_servers.clone(),
            timeout: Duration::from_secs(3),
            retries: 2,
            cache_ttl: Duration::from_secs(300),
            local_addr: self.socket.as_ref().and_then(|s| s.local_addr().ok()),
        };

        let stun_client = StunClient::new(stun_config)?;

        // Get external address
        let external_addr = stun_client.get_external_address()?;

        // Detect NAT type
        let nat_type = stun_client.detect_nat_type().unwrap_or(NatType::Unknown);
        *self.nat_type.write().unwrap() = Some(nat_type);

        log::info!("NAT type detected: {:?}", nat_type);

        // Get local base address
        let base_addr = self.socket.as_ref().and_then(|s| s.local_addr().ok());

        let priority = calculate_priority(IceCandidateType::ServerReflexive, external_addr);
        let foundation = generate_foundation(
            IceCandidateType::ServerReflexive,
            &external_addr,
            base_addr.as_ref(),
        );

        let candidate = IceCandidateInfo {
            candidate_type: IceCandidateType::ServerReflexive,
            protocol: IceProtocol::Udp,
            address: external_addr,
            priority,
            foundation,
            component: COMPONENT_RTP,
            base_addr,
            related_addr: Some(external_addr),
        };

        let mut candidates = self.local_candidates.write().unwrap();
        candidates.push(candidate);
        self.stats.candidates_gathered.fetch_add(1, Ordering::Relaxed);

        Ok(())
    }

    // =========================================================================
    // Remote Candidate Handling
    // =========================================================================

    /// Add remote candidates from signaling
    pub fn add_remote_candidates(&self, candidates: &[IceCandidate]) {
        let mut remote = self.remote_candidates.write().unwrap();

        for c in candidates {
            let info = IceCandidateInfo {
                candidate_type: c.candidate_type,
                protocol: c.protocol,
                address: c.address,
                priority: c.priority,
                foundation: c.foundation.clone(),
                component: c.component,
                base_addr: None,
                related_addr: None,
            };

            remote.push(info);
        }

        // Re-compute candidate pairs
        drop(remote);
        self.compute_candidate_pairs();
    }

    /// Compute all candidate pairs from local and remote candidates
    fn compute_candidate_pairs(&self) {
        let local = self.local_candidates.read().unwrap();
        let remote = self.remote_candidates.read().unwrap();
        let role = *self.role.read().unwrap();

        let mut pairs = Vec::new();

        for l in local.iter() {
            for r in remote.iter() {
                // Only pair same component and protocol
                if l.component != r.component || l.protocol != r.protocol {
                    continue;
                }

                // Only pair same address family
                let l_is_ipv6 = l.address.is_ipv6();
                let r_is_ipv6 = r.address.is_ipv6();
                if l_is_ipv6 != r_is_ipv6 {
                    continue;
                }

                let pair = CandidatePair::new(l.clone(), r.clone(), role);
                pairs.push(pair);
            }
        }

        // Sort by priority (highest first)
        pairs.sort_by(|a, b| b.priority.cmp(&a.priority));

        // Unfreeze first pair of each foundation
        let mut seen_foundations = std::collections::HashSet::new();
        for pair in &mut pairs {
            let foundation_key = format!("{}:{}", pair.local.foundation, pair.remote.foundation);
            if !seen_foundations.contains(&foundation_key) {
                pair.state = CandidatePairState::Waiting;
                seen_foundations.insert(foundation_key);
            }
        }

        *self.candidate_pairs.write().unwrap() = pairs;

        log::debug!(
            "Computed {} candidate pairs",
            self.candidate_pairs.read().unwrap().len()
        );
    }

    // =========================================================================
    // Connectivity Checks
    // =========================================================================

    /// Run connectivity checks on all candidate pairs
    ///
    /// Returns Ok(true) if at least one pair succeeded, Ok(false) if still checking,
    /// or Err if all pairs failed or timeout.
    pub fn run_connectivity_checks(&self) -> io::Result<bool> {
        *self.connection_state.write().unwrap() = IceConnectionState::Checking;

        let start = Instant::now();
        let timeout = self.config.timeout;

        loop {
            // Check timeout
            if start.elapsed() > timeout {
                *self.connection_state.write().unwrap() = IceConnectionState::Failed;
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "ICE connectivity check timeout",
                ));
            }

            // Process any incoming responses
            self.process_incoming()?;

            // Find next pair to check
            if let Some(pair_idx) = self.find_next_pair_to_check() {
                self.send_connectivity_check(pair_idx)?;
            }

            // Check if we have a successful pair
            {
                let pairs = self.candidate_pairs.read().unwrap();
                let succeeded = pairs
                    .iter()
                    .any(|p| p.state == CandidatePairState::Succeeded);

                if succeeded {
                    // Select best succeeded pair
                    drop(pairs);
                    self.select_best_pair();

                    let state = if self.all_checks_complete() {
                        IceConnectionState::Completed
                    } else {
                        IceConnectionState::Connected
                    };
                    *self.connection_state.write().unwrap() = state;

                    return Ok(true);
                }

                // Check if all pairs failed
                let all_failed = pairs
                    .iter()
                    .all(|p| p.state == CandidatePairState::Failed);

                if all_failed && !pairs.is_empty() {
                    *self.connection_state.write().unwrap() = IceConnectionState::Failed;
                    return Err(io::Error::new(
                        io::ErrorKind::ConnectionRefused,
                        "All connectivity checks failed",
                    ));
                }
            }

            // Brief sleep to avoid busy loop
            std::thread::sleep(self.config.check_interval);
        }
    }

    /// Find the next candidate pair to check
    fn find_next_pair_to_check(&self) -> Option<usize> {
        let mut pairs = self.candidate_pairs.write().unwrap();

        for (idx, pair) in pairs.iter_mut().enumerate() {
            match pair.state {
                CandidatePairState::Waiting => {
                    return Some(idx);
                }
                CandidatePairState::InProgress => {
                    // Check for timeout/retry
                    if let Some(last) = pair.last_check {
                        if last.elapsed() > Duration::from_millis(500) {
                            if pair.check_count < self.config.max_check_retries {
                                return Some(idx);
                            } else {
                                pair.state = CandidatePairState::Failed;
                                self.stats.failed_pairs.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                    }
                }
                CandidatePairState::Frozen => {
                    // Unfreeze pairs as needed
                    pair.state = CandidatePairState::Waiting;
                    return Some(idx);
                }
                _ => {}
            }
        }

        None
    }

    /// Send a STUN connectivity check
    fn send_connectivity_check(&self, pair_idx: usize) -> io::Result<()> {
        let socket = self
            .socket
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No socket"))?;

        let role = *self.role.read().unwrap();

        let (remote_addr, txn_id, priority) = {
            let mut pairs = self.candidate_pairs.write().unwrap();
            let pair = pairs
                .get_mut(pair_idx)
                .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "Invalid pair index"))?;

            pair.state = CandidatePairState::InProgress;
            pair.check_count += 1;
            pair.last_check = Some(Instant::now());

            let txn_id = generate_transaction_id();

            // Store pending check
            self.pending_checks
                .lock()
                .unwrap()
                .insert(txn_id, pair_idx);

            (pair.remote.address, txn_id, pair.local.priority)
        };

        // Build STUN binding request with ICE attributes
        let request = build_ice_binding_request(
            txn_id,
            priority,
            self.tiebreaker,
            role,
            self.config.aggressive_nomination,
        );

        socket.send_to(&request, remote_addr)?;
        self.stats.checks_sent.fetch_add(1, Ordering::Relaxed);

        log::trace!(
            "Sent connectivity check to {} (pair {})",
            remote_addr,
            pair_idx
        );

        Ok(())
    }

    /// Process incoming STUN messages
    fn process_incoming(&self) -> io::Result<()> {
        let socket = match &self.socket {
            Some(s) => s,
            None => return Ok(()),
        };

        let mut buf = [0u8; 1500];

        loop {
            match socket.recv_from(&mut buf) {
                Ok((len, from)) => {
                    if len >= STUN_HEADER_SIZE {
                        self.process_stun_message(&buf[..len], from)?;
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }

        Ok(())
    }

    /// Process a received STUN message
    fn process_stun_message(&self, data: &[u8], from: SocketAddr) -> io::Result<()> {
        if data.len() < STUN_HEADER_SIZE {
            return Ok(());
        }

        let msg_type = u16::from_be_bytes([data[0], data[1]]);
        let magic = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);

        if magic != STUN_MAGIC_COOKIE {
            return Ok(());
        }

        let txn_id: [u8; 12] = data[8..20].try_into().unwrap();

        match msg_type {
            STUN_BINDING_REQUEST => {
                // Incoming connectivity check - respond
                self.handle_binding_request(data, from, txn_id)?;
            }
            STUN_BINDING_RESPONSE => {
                // Response to our check
                self.handle_binding_response(data, from, txn_id)?;
            }
            _ => {}
        }

        Ok(())
    }

    /// Handle incoming STUN binding request (remote peer checking us)
    fn handle_binding_request(
        &self,
        _data: &[u8],
        from: SocketAddr,
        txn_id: [u8; 12],
    ) -> io::Result<()> {
        let socket = self
            .socket
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No socket"))?;

        // Build success response with XOR-MAPPED-ADDRESS
        let response = build_ice_binding_response(txn_id, from);

        socket.send_to(&response, from)?;
        self.stats.checks_received.fetch_add(1, Ordering::Relaxed);

        log::trace!("Responded to connectivity check from {}", from);

        Ok(())
    }

    /// Handle STUN binding response
    fn handle_binding_response(
        &self,
        _data: &[u8],
        from: SocketAddr,
        txn_id: [u8; 12],
    ) -> io::Result<()> {
        // Find the pair this response belongs to
        let pair_idx = {
            let mut pending = self.pending_checks.lock().unwrap();
            pending.remove(&txn_id)
        };

        let Some(pair_idx) = pair_idx else {
            log::trace!("Received response for unknown transaction");
            return Ok(());
        };

        // Update pair state
        let mut pairs = self.candidate_pairs.write().unwrap();
        if let Some(pair) = pairs.get_mut(pair_idx) {
            if pair.state == CandidatePairState::InProgress {
                let rtt = pair.last_check.map(|t| t.elapsed());
                pair.state = CandidatePairState::Succeeded;
                pair.rtt = rtt;

                self.stats.successful_pairs.fetch_add(1, Ordering::Relaxed);

                log::info!(
                    "Connectivity check succeeded: {} <-> {} (RTT: {:?})",
                    pair.local.address,
                    pair.remote.address,
                    rtt
                );
            }
        }

        let _ = from; // Used for peer reflexive candidate discovery in full ICE

        Ok(())
    }

    /// Select the best succeeded pair
    fn select_best_pair(&self) {
        let pairs = self.candidate_pairs.read().unwrap();

        let best = pairs
            .iter()
            .enumerate()
            .filter(|(_, p)| p.state == CandidatePairState::Succeeded)
            .max_by_key(|(_, p)| p.priority);

        if let Some((idx, pair)) = best {
            *self.selected_pair.write().unwrap() = Some(idx);

            if let Some(rtt) = pair.rtt {
                self.stats
                    .selected_rtt_us
                    .store(rtt.as_micros() as u64, Ordering::Relaxed);
            }

            log::info!(
                "Selected pair: {} <-> {} (priority: {}, RTT: {:?})",
                pair.local.address,
                pair.remote.address,
                pair.priority,
                pair.rtt
            );
        }
    }

    /// Check if all connectivity checks are complete
    fn all_checks_complete(&self) -> bool {
        let pairs = self.candidate_pairs.read().unwrap();
        pairs.iter().all(|p| {
            matches!(
                p.state,
                CandidatePairState::Succeeded | CandidatePairState::Failed
            )
        })
    }

    // =========================================================================
    // Hole Punching
    // =========================================================================

    /// Perform synchronized hole punching
    ///
    /// Both peers should call this simultaneously (within ~100ms) for best results.
    /// The coordination should be done via signaling before calling this method.
    pub fn punch_hole(&self, remote_addr: SocketAddr) -> io::Result<()> {
        let socket = self
            .socket
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No socket"))?;

        // Send several packets rapidly to open NAT mapping
        for i in 0..5 {
            let txn_id = generate_transaction_id();
            let request = build_simple_binding_request(txn_id);

            socket.send_to(&request, remote_addr)?;

            // Small delay between packets
            if i < 4 {
                std::thread::sleep(Duration::from_millis(20));
            }
        }

        log::debug!("Hole punch packets sent to {}", remote_addr);

        Ok(())
    }

    /// Perform hole punching with timing coordination
    ///
    /// Waits until `start_time` to send, ensuring both peers punch simultaneously.
    pub fn punch_hole_at(
        &self,
        remote_addr: SocketAddr,
        start_time: Instant,
    ) -> io::Result<()> {
        // Wait until start time
        let now = Instant::now();
        if start_time > now {
            std::thread::sleep(start_time - now);
        }

        self.punch_hole(remote_addr)
    }
}

impl std::fmt::Debug for IceAgent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("IceAgent")
            .field("role", &self.role.read().unwrap())
            .field("gathering_state", &self.gathering_state.read().unwrap())
            .field("connection_state", &self.connection_state.read().unwrap())
            .field(
                "local_candidates",
                &self.local_candidates.read().unwrap().len(),
            )
            .field(
                "remote_candidates",
                &self.remote_candidates.read().unwrap().len(),
            )
            .field("nat_type", &self.nat_type.read().unwrap())
            .finish()
    }
}

// =============================================================================
// Helper Functions
// =============================================================================

/// Get all local IP addresses
fn get_local_addresses() -> io::Result<Vec<IpAddr>> {
    let mut addrs = Vec::new();

    // Get addresses from all interfaces
    // This is platform-specific, so we use a simple approach

    // Try to get local address by connecting to a public IP (doesn't actually connect)
    if let Ok(socket) = UdpSocket::bind("0.0.0.0:0") {
        // Connect to Google DNS to determine our default route IP
        if socket.connect("8.8.8.8:53").is_ok() {
            if let Ok(local) = socket.local_addr() {
                addrs.push(local.ip());
            }
        }
    }

    // Also try IPv6
    if let Ok(socket) = UdpSocket::bind("[::]:0") {
        if socket.connect("[2001:4860:4860::8888]:53").is_ok() {
            if let Ok(local) = socket.local_addr() {
                addrs.push(local.ip());
            }
        }
    }

    // Add loopback as fallback
    if addrs.is_empty() {
        addrs.push(IpAddr::V4(std::net::Ipv4Addr::LOCALHOST));
    }

    Ok(addrs)
}

/// Calculate candidate priority (RFC 8445)
fn calculate_priority(candidate_type: IceCandidateType, addr: SocketAddr) -> u32 {
    let type_pref = match candidate_type {
        IceCandidateType::Host => TYPE_PREFERENCE_HOST,
        IceCandidateType::ServerReflexive => TYPE_PREFERENCE_SRFLX,
        IceCandidateType::PeerReflexive => TYPE_PREFERENCE_PRFLX,
        IceCandidateType::Relay => TYPE_PREFERENCE_RELAY,
    };

    // Local preference (prefer IPv4 for now)
    let local_pref: u32 = if addr.is_ipv6() { 65534 } else { 65535 };

    // Component ID (always 1 for our use case)
    let component_id = COMPONENT_RTP as u32;

    // priority = (2^24)*(type preference) + (2^8)*(local preference) + (256 - component ID)
    (type_pref << 24) + (local_pref << 8) + (256 - component_id)
}

/// Generate foundation string for candidate
fn generate_foundation(
    candidate_type: IceCandidateType,
    addr: &SocketAddr,
    base: Option<&SocketAddr>,
) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    // Hash the candidate type as its discriminant value
    let type_id: u8 = match candidate_type {
        IceCandidateType::Host => 0,
        IceCandidateType::ServerReflexive => 1,
        IceCandidateType::PeerReflexive => 2,
        IceCandidateType::Relay => 3,
    };
    type_id.hash(&mut hasher);
    addr.ip().hash(&mut hasher);
    if let Some(b) = base {
        b.hash(&mut hasher);
    }

    let hash = hasher.finish();
    format!("{}", hash & 0xFFFFFFFF)
}

/// Generate random tiebreaker value
fn generate_tiebreaker() -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::time::SystemTime;

    let mut hasher = DefaultHasher::new();
    SystemTime::now().hash(&mut hasher);
    std::process::id().hash(&mut hasher);
    FOUNDATION_COUNTER.fetch_add(1, Ordering::Relaxed).hash(&mut hasher);
    hasher.finish()
}

/// Generate random transaction ID
fn generate_transaction_id() -> [u8; 12] {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::time::SystemTime;

    let mut hasher = DefaultHasher::new();
    SystemTime::now().hash(&mut hasher);
    std::process::id().hash(&mut hasher);
    FOUNDATION_COUNTER.fetch_add(1, Ordering::Relaxed).hash(&mut hasher);
    std::thread::current().id().hash(&mut hasher);
    let h1 = hasher.finish();

    let mut hasher2 = DefaultHasher::new();
    h1.hash(&mut hasher2);
    let h2 = hasher2.finish();

    let mut result = [0u8; 12];
    result[0..8].copy_from_slice(&h1.to_le_bytes());
    result[8..12].copy_from_slice(&h2.to_le_bytes()[0..4]);
    result
}

/// Build a STUN binding request for ICE connectivity checks
fn build_ice_binding_request(
    txn_id: [u8; 12],
    priority: u32,
    tiebreaker: u64,
    role: IceRole,
    use_candidate: bool,
) -> Vec<u8> {
    let mut msg = Vec::with_capacity(64);

    // Header
    msg.extend_from_slice(&STUN_BINDING_REQUEST.to_be_bytes());
    let len_pos = msg.len();
    msg.extend_from_slice(&0u16.to_be_bytes()); // Length placeholder
    msg.extend_from_slice(&STUN_MAGIC_COOKIE.to_be_bytes());
    msg.extend_from_slice(&txn_id);

    // PRIORITY attribute
    msg.extend_from_slice(&ATTR_PRIORITY.to_be_bytes());
    msg.extend_from_slice(&4u16.to_be_bytes());
    msg.extend_from_slice(&priority.to_be_bytes());

    // ICE-CONTROLLING or ICE-CONTROLLED
    let role_attr = match role {
        IceRole::Controlling => ATTR_ICE_CONTROLLING,
        IceRole::Controlled => ATTR_ICE_CONTROLLED,
    };
    msg.extend_from_slice(&role_attr.to_be_bytes());
    msg.extend_from_slice(&8u16.to_be_bytes());
    msg.extend_from_slice(&tiebreaker.to_be_bytes());

    // USE-CANDIDATE (for aggressive nomination)
    if use_candidate && role == IceRole::Controlling {
        msg.extend_from_slice(&ATTR_USE_CANDIDATE.to_be_bytes());
        msg.extend_from_slice(&0u16.to_be_bytes()); // Zero length
    }

    // Update length
    let attr_len = (msg.len() - STUN_HEADER_SIZE) as u16;
    msg[len_pos..len_pos + 2].copy_from_slice(&attr_len.to_be_bytes());

    msg
}

/// Build a simple STUN binding request (for hole punching)
fn build_simple_binding_request(txn_id: [u8; 12]) -> Vec<u8> {
    let mut msg = Vec::with_capacity(32);

    msg.extend_from_slice(&STUN_BINDING_REQUEST.to_be_bytes());
    msg.extend_from_slice(&0u16.to_be_bytes()); // No attributes
    msg.extend_from_slice(&STUN_MAGIC_COOKIE.to_be_bytes());
    msg.extend_from_slice(&txn_id);

    msg
}

/// Build a STUN binding success response with XOR-MAPPED-ADDRESS
fn build_ice_binding_response(txn_id: [u8; 12], from: SocketAddr) -> Vec<u8> {
    let mut msg = Vec::with_capacity(48);

    // Header
    msg.extend_from_slice(&STUN_BINDING_RESPONSE.to_be_bytes());
    let len_pos = msg.len();
    msg.extend_from_slice(&0u16.to_be_bytes()); // Length placeholder
    msg.extend_from_slice(&STUN_MAGIC_COOKIE.to_be_bytes());
    msg.extend_from_slice(&txn_id);

    // XOR-MAPPED-ADDRESS
    msg.extend_from_slice(&ATTR_XOR_MAPPED_ADDRESS.to_be_bytes());

    match from {
        SocketAddr::V4(v4) => {
            msg.extend_from_slice(&8u16.to_be_bytes()); // Attr length
            msg.push(0); // Reserved
            msg.push(0x01); // IPv4 family

            let xor_port = from.port() ^ ((STUN_MAGIC_COOKIE >> 16) as u16);
            msg.extend_from_slice(&xor_port.to_be_bytes());

            let ip_bytes: u32 = (*v4.ip()).into();
            let xor_ip = ip_bytes ^ STUN_MAGIC_COOKIE;
            msg.extend_from_slice(&xor_ip.to_be_bytes());
        }
        SocketAddr::V6(v6) => {
            msg.extend_from_slice(&20u16.to_be_bytes()); // Attr length
            msg.push(0); // Reserved
            msg.push(0x02); // IPv6 family

            let xor_port = from.port() ^ ((STUN_MAGIC_COOKIE >> 16) as u16);
            msg.extend_from_slice(&xor_port.to_be_bytes());

            let ip_bytes = v6.ip().octets();
            let xor_bytes: Vec<u8> = STUN_MAGIC_COOKIE
                .to_be_bytes()
                .iter()
                .chain(txn_id.iter())
                .copied()
                .collect();

            for i in 0..16 {
                msg.push(ip_bytes[i] ^ xor_bytes[i]);
            }
        }
    }

    // Update length
    let attr_len = (msg.len() - STUN_HEADER_SIZE) as u16;
    msg[len_pos..len_pos + 2].copy_from_slice(&attr_len.to_be_bytes());

    msg
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ice_agent_creation() {
        let agent = IceAgent::with_defaults().unwrap();
        assert_eq!(agent.gathering_state(), IceGatheringState::New);
        assert_eq!(agent.connection_state(), IceConnectionState::New);
    }

    #[test]
    fn test_ice_role() {
        let agent = IceAgent::with_defaults().unwrap();
        assert_eq!(agent.get_role(), IceRole::Controlling);

        agent.set_role(IceRole::Controlled);
        assert_eq!(agent.get_role(), IceRole::Controlled);
    }

    #[test]
    fn test_candidate_priority() {
        let host_prio = calculate_priority(
            IceCandidateType::Host,
            "192.168.1.1:8080".parse().unwrap(),
        );
        let srflx_prio = calculate_priority(
            IceCandidateType::ServerReflexive,
            "1.2.3.4:8080".parse().unwrap(),
        );

        // Host should have higher priority
        assert!(host_prio > srflx_prio);
    }

    #[test]
    fn test_pair_priority() {
        // Use Host vs ServerReflexive to get different priorities
        let local_prio = calculate_priority(
            IceCandidateType::Host,
            "192.168.1.1:8080".parse().unwrap(),
        );
        let remote_prio = calculate_priority(
            IceCandidateType::ServerReflexive,
            "203.0.113.1:8080".parse().unwrap(),
        );

        // Verify we have different priorities (Host > ServerReflexive)
        assert_ne!(local_prio, remote_prio);
        assert!(local_prio > remote_prio, "Host should have higher priority than srflx");

        let ctrl_pair = CandidatePair::calculate_priority(IceRole::Controlling, local_prio, remote_prio);
        let ctld_pair = CandidatePair::calculate_priority(IceRole::Controlled, local_prio, remote_prio);

        // When local and remote priorities differ, role affects which is G vs D
        // Controlling: G=local, D=remote; Controlled: G=remote, D=local
        // This changes the (G>D?1:0) term and potentially the min/max arrangement
        assert_ne!(ctrl_pair, ctld_pair);

        // Test edge case: when priorities are equal, pair priority is equal regardless of role
        let same_prio = calculate_priority(
            IceCandidateType::Host,
            "192.168.1.1:8080".parse().unwrap(),
        );
        let ctrl_same = CandidatePair::calculate_priority(IceRole::Controlling, same_prio, same_prio);
        let ctld_same = CandidatePair::calculate_priority(IceRole::Controlled, same_prio, same_prio);
        assert_eq!(ctrl_same, ctld_same, "Equal priorities should give equal pair priority");
    }

    #[test]
    fn test_foundation_generation() {
        let addr: SocketAddr = "192.168.1.1:8080".parse().unwrap();

        let f1 = generate_foundation(IceCandidateType::Host, &addr, None);
        let f2 = generate_foundation(IceCandidateType::Host, &addr, None);
        let f3 = generate_foundation(IceCandidateType::ServerReflexive, &addr, None);

        // Same type + addr should give same foundation
        assert_eq!(f1, f2);
        // Different type should give different foundation
        assert_ne!(f1, f3);
    }

    #[test]
    fn test_transaction_id_uniqueness() {
        let id1 = generate_transaction_id();
        let id2 = generate_transaction_id();
        assert_ne!(id1, id2);
    }

    #[test]
    fn test_tiebreaker_uniqueness() {
        let tb1 = generate_tiebreaker();
        let tb2 = generate_tiebreaker();
        // Should be different (with very high probability)
        assert_ne!(tb1, tb2);
    }

    #[test]
    fn test_ice_binding_request_format() {
        let txn_id = generate_transaction_id();
        let request = build_ice_binding_request(
            txn_id,
            12345,
            0xDEADBEEF,
            IceRole::Controlling,
            false,
        );

        // Verify header
        assert!(request.len() >= STUN_HEADER_SIZE);
        let msg_type = u16::from_be_bytes([request[0], request[1]]);
        assert_eq!(msg_type, STUN_BINDING_REQUEST);

        let magic = u32::from_be_bytes([request[4], request[5], request[6], request[7]]);
        assert_eq!(magic, STUN_MAGIC_COOKIE);
    }

    #[test]
    fn test_ice_binding_response_format() {
        let txn_id = generate_transaction_id();
        let from: SocketAddr = "1.2.3.4:5678".parse().unwrap();
        let response = build_ice_binding_response(txn_id, from);

        // Verify header
        assert!(response.len() >= STUN_HEADER_SIZE);
        let msg_type = u16::from_be_bytes([response[0], response[1]]);
        assert_eq!(msg_type, STUN_BINDING_RESPONSE);

        // Should contain XOR-MAPPED-ADDRESS
        assert!(response.len() > STUN_HEADER_SIZE);
    }

    #[test]
    fn test_nat_type_hole_punching_support() {
        assert!(NatType::FullCone.supports_hole_punching());
        assert!(NatType::RestrictedCone.supports_hole_punching());
        assert!(NatType::PortRestrictedCone.supports_hole_punching());
        assert!(!NatType::Symmetric.supports_hole_punching());
    }

    #[test]
    fn test_ice_role_conflict_resolution() {
        let our_tb = 100;
        let their_tb = 50;
        assert_eq!(
            IceRole::resolve_conflict(our_tb, their_tb),
            IceRole::Controlling
        );

        let our_tb = 50;
        let their_tb = 100;
        assert_eq!(
            IceRole::resolve_conflict(our_tb, their_tb),
            IceRole::Controlled
        );
    }

    #[test]
    fn test_candidate_info_to_signaling() {
        let candidate = IceCandidateInfo {
            candidate_type: IceCandidateType::Host,
            protocol: IceProtocol::Udp,
            address: "192.168.1.1:8080".parse().unwrap(),
            priority: 12345,
            foundation: "abc123".to_string(),
            component: 1,
            base_addr: None,
            related_addr: None,
        };

        let signaling = candidate.to_signaling_candidate();
        assert_eq!(signaling.candidate_type, IceCandidateType::Host);
        assert_eq!(signaling.priority, 12345);
        assert_eq!(signaling.foundation, "abc123");
    }

    #[test]
    fn test_candidate_pair_creation() {
        let local = IceCandidateInfo {
            candidate_type: IceCandidateType::Host,
            protocol: IceProtocol::Udp,
            address: "192.168.1.1:8080".parse().unwrap(),
            priority: 2130706431, // Typical host priority
            foundation: "1".to_string(),
            component: 1,
            base_addr: None,
            related_addr: None,
        };

        let remote = IceCandidateInfo {
            candidate_type: IceCandidateType::Host,
            protocol: IceProtocol::Udp,
            address: "192.168.2.1:8080".parse().unwrap(),
            priority: 2130706431,
            foundation: "2".to_string(),
            component: 1,
            base_addr: None,
            related_addr: None,
        };

        let pair = CandidatePair::new(local, remote, IceRole::Controlling);
        assert_eq!(pair.state, CandidatePairState::Frozen);
        assert!(pair.priority > 0);
        assert_eq!(pair.check_count, 0);
    }

    #[test]
    fn test_ice_config_default() {
        let config = IceConfig::default();
        assert!(!config.stun_servers.is_empty());
        assert!(config.gather_host);
        assert!(config.gather_srflx);
        assert!(config.timeout > Duration::ZERO);
    }

    #[test]
    fn test_gather_candidates() {
        let mut config = IceConfig::default();
        // Disable STUN for this test (requires network)
        config.gather_srflx = false;

        let agent = IceAgent::new(config).unwrap();
        agent.gather_candidates().unwrap();

        // Should have at least one host candidate
        assert!(!agent.local_candidates().is_empty());
        assert_eq!(agent.gathering_state(), IceGatheringState::Complete);
    }

    #[test]
    fn test_add_remote_candidates() {
        let agent = IceAgent::with_defaults().unwrap();

        let remote_candidates = vec![
            IceCandidate {
                candidate_type: IceCandidateType::Host,
                protocol: IceProtocol::Udp,
                address: "10.0.0.1:8080".parse().unwrap(),
                priority: 2130706431,
                foundation: "1".to_string(),
                component: 1,
            },
        ];

        agent.add_remote_candidates(&remote_candidates);

        let remote = agent.remote_candidates.read().unwrap();
        assert_eq!(remote.len(), 1);
    }
}
