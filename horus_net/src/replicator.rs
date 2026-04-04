//! The Replicator — central event-driven loop tying everything together.
//!
//! Three event sources:
//! - **UDP socket**: incoming network packets (import path)
//! - **SHM eventfd**: new local data to export (export path)
//! - **Timer**: periodic discovery announcements + heartbeat + liveness checks
//!
//! Demand-driven: only replicates topics with remote interest.
//! See blueprint section 18.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::config::{check_no_peers_diagnostic, DiscoveryMode, NetConfig};
use crate::discovery::{
    decode_announcement, encode_announcement, find_matches, generate_peer_id, peer_id_hash,
    ReplicationMatch,
};
use crate::encoding;
use crate::event_loop::{EventSource, PlatformEventLoop};
use crate::flow_control::FlowController;
use crate::fragment::{Fragmenter, Reassembler};
use crate::guard::{ExportMode, ImportExportGuard, ImportMode};
use crate::heartbeat::{LinkLostAction, SafetyHeartbeat};
use crate::metrics::NetMetrics;
use crate::optimize::OptimizerChain;
use crate::peer::PeerTable;
use crate::priority::{Encoding, Priority, Reliability};
use crate::registry::TopicRegistry;
use crate::reliability::ReliabilityLayer;
use crate::shm_reader::ShmRingReader;
use crate::shm_writer::ShmRingWriter;
use crate::transport::udp::UdpTransport;
use crate::transport::Transport;
use crate::wire::{self, PacketFlags, PacketHeader};

/// Discovery announcement interval.
const DISCOVERY_INTERVAL: Duration = Duration::from_secs(1);

/// Timer tick interval (heartbeat rate — 50ms).
const TIMER_INTERVAL: Duration = Duration::from_millis(50);

/// No-peers diagnostic timeout.
const NO_PEERS_TIMEOUT: Duration = Duration::from_secs(5);

/// The Replicator — one per process, started by the Scheduler.
pub struct Replicator {
    // ── Core ──
    registry: Arc<TopicRegistry>,
    config: NetConfig,
    peer_id: [u8; 16],
    peer_id_hash: u16,
    secret_hash: [u8; 4],
    peers: PeerTable,
    transport: UdpTransport,
    readers: HashMap<String, ShmRingReader>,
    writers: HashMap<String, ShmRingWriter>,
    matches: Vec<ReplicationMatch>,
    packet_seq: u32,
    start_time: Instant,
    last_discovery: Instant,
    running: Arc<AtomicBool>,
    // ── Safety & Reliability (Phase 2 — NOW WIRED) ──
    guard: ImportExportGuard,
    heartbeat: SafetyHeartbeat,
    reliability: ReliabilityLayer,
    fragmenter: Fragmenter,
    reassembler: Reassembler,
    flow_control: FlowController,
    metrics: NetMetrics,
    optimizers: OptimizerChain,
    // ── Observability (system topics) ──
    presence_receiver: crate::presence::PresenceReceiver,
    log_drain: crate::log_replication::LogDrain,
    estop_broadcaster: crate::estop::EstopBroadcaster,
    last_presence_cleanup: Instant,
}

impl Replicator {
    /// Create a new Replicator.
    pub fn new(registry: Arc<TopicRegistry>, config: NetConfig) -> std::io::Result<Self> {
        let transport = UdpTransport::bind(config.port)?;

        if let DiscoveryMode::Multicast { ref group } = config.discovery_mode() {
            if let Err(e) = transport.join_multicast(group) {
                eprintln!("[horus_net] Failed to join multicast group: {e}");
            }
        }

        let peer_id = generate_peer_id();
        let id_hash = peer_id_hash(&peer_id);
        let secret_hash = config.secret_hash();

        // Build import guard from config
        let import_mode = match &config.import {
            crate::config::ImportConfig::Deny => ImportMode::Deny,
            crate::config::ImportConfig::Auto => ImportMode::Auto,
            crate::config::ImportConfig::AllowList(list) => ImportMode::AllowList(list.clone()),
        };
        let export_mode = if config.deny_export.is_empty() {
            ExportMode::All
        } else {
            ExportMode::DenyList(config.deny_export.clone())
        };
        let guard = ImportExportGuard::new(import_mode, export_mode, Some(registry.clone()));

        // Build heartbeat from config
        let hb_interval = Duration::from_millis(config.safety.heartbeat_ms);
        let hb_action = LinkLostAction::from_str(&config.safety.on_link_lost);
        if crate::config::is_wsl2() {
            eprintln!(
                "[horus_net] WSL2 detected — using relaxed timeouts (heartbeat {}ms, {} missed threshold)",
                config.safety.heartbeat_ms, config.safety.missed_threshold
            );
        }
        let heartbeat = SafetyHeartbeat::with_config(
            peer_id,
            hb_interval,
            config.safety.missed_threshold,
            hb_action,
        );

        // Build optimizer chain from config
        let optimizers = OptimizerChain::from_config(&config.optimizers);

        Ok(Self {
            registry,
            config,
            peer_id,
            peer_id_hash: id_hash,
            secret_hash,
            peers: PeerTable::new(),
            transport,
            readers: HashMap::new(),
            writers: HashMap::new(),
            matches: Vec::new(),
            packet_seq: 0,
            start_time: Instant::now(),
            last_discovery: Instant::now() - DISCOVERY_INTERVAL,
            running: Arc::new(AtomicBool::new(true)),
            guard,
            heartbeat,
            reliability: ReliabilityLayer::new(),
            fragmenter: Fragmenter::new(),
            reassembler: Reassembler::new(),
            flow_control: FlowController::new(),
            metrics: NetMetrics::new(),
            optimizers,
            presence_receiver: crate::presence::PresenceReceiver::new(),
            log_drain: crate::log_replication::LogDrain::new(id_hash),
            estop_broadcaster: crate::estop::EstopBroadcaster::new(),
            last_presence_cleanup: Instant::now(),
        })
    }

    /// Get a handle to signal shutdown.
    pub fn running_flag(&self) -> Arc<AtomicBool> {
        self.running.clone()
    }

    /// Run the main event loop. Blocks until shutdown is signaled.
    pub fn run(&mut self) {
        let mut event_loop = match PlatformEventLoop::new(TIMER_INTERVAL) {
            Ok(el) => el,
            Err(e) => {
                eprintln!("[horus_net] Failed to create event loop: {e}");
                return;
            }
        };

        #[cfg(unix)]
        if let Err(e) = event_loop.register_udp(self.transport.raw_fd()) {
            eprintln!("[horus_net] Failed to register UDP socket: {e}");
            return;
        }

        let shm_notify_fd = event_loop.shm_notify_fd();
        self.registry.set_on_change(move || {
            let val: u64 = 1;
            unsafe {
                libc::write(shm_notify_fd, &val as *const u64 as *const libc::c_void, 8);
            }
        });

        while self.running.load(Ordering::Relaxed) {
            let events = match event_loop.wait(1000) {
                Ok(events) => events,
                Err(e) => {
                    if e.kind() != std::io::ErrorKind::Interrupted {
                        eprintln!("[horus_net] Event loop error: {e}");
                    }
                    continue;
                }
            };

            for event in &events {
                match event.source {
                    EventSource::UdpSocket => self.handle_incoming(),
                    EventSource::ShmNotify => self.handle_export(),
                    EventSource::Timer => self.handle_timer(),
                }
            }

            if events.is_empty() {
                self.handle_timer();
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // IMPORT PATH: Network → Local SHM
    // ═══════════════════════════════════════════════════════════════════════

    fn handle_incoming(&mut self) {
        let mut buf = [0u8; 65536];
        loop {
            match self.transport.recv_from(&mut buf) {
                Ok((n, from)) => {
                    self.metrics.record_recv(self.peer_id_hash, n);
                    self.process_packet(&buf[..n], from);
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
    }

    fn process_packet(&mut self, buf: &[u8], from: SocketAddr) {
        // Discovery announcement
        if let Some(ann) = decode_announcement(buf, from) {
            if ann.peer_id == self.peer_id {
                return;
            }
            if self.secret_hash != [0u8; 4] && ann.has_secret && ann.secret_hash != self.secret_hash
            {
                return;
            }
            self.peers.update_peer(&ann);
            self.heartbeat.add_peer(ann.peer_id, ann.source_addr);
            // Record peer discovery to blackbox
            horus_core::scheduling::record_external_event(
                horus_core::scheduling::BlackBoxEvent::NetPeerDiscovered {
                    peer_addr: from.to_string(),
                    topic_count: ann.topics.len(),
                },
            );
            self.update_matches();
            return;
        }

        // Heartbeat packet
        let header = match wire::PacketHeader::decode(buf) {
            Some(h) => h,
            None => return,
        };

        if header.flags.heartbeat() {
            if let Some(hb) = wire::decode_heartbeat(buf) {
                self.heartbeat.on_received(&hb.peer_id);
            }
            return;
        }

        // ACK packet
        if header.flags.ack() {
            if let Some(ack) = wire::decode_ack(buf) {
                self.reliability.on_ack(&ack);
            }
            return;
        }

        // Data packet — handle fragments and regular messages
        if header.flags.fragment() {
            // Parse the message header (contains topic_hash, sequence, etc.)
            // then the fragment header, then the fragment payload
            let mh_start = wire::PacketHeader::SIZE;
            if let Some(mh) = wire::MessageHeader::decode(&buf[mh_start..]) {
                let fh_start = mh_start + wire::MessageHeader::SIZE;
                if let Some(fh) = wire::FragmentHeader::decode(&buf[fh_start..]) {
                    let payload_start = fh_start + wire::FragmentHeader::SIZE;
                    let payload = buf[payload_start..].to_vec();

                    let frag = crate::fragment::Fragment {
                        topic_hash: mh.topic_hash,
                        sequence: mh.sequence,
                        timestamp_ns: mh.timestamp_ns,
                        priority: mh.priority,
                        reliability: mh.reliability,
                        encoding: mh.encoding,
                        fragment_id: fh.fragment_id,
                        fragment_index: fh.fragment_index,
                        fragment_count: fh.fragment_count,
                        total_payload_len: fh.total_payload_len,
                        payload,
                    };

                    if let Some(reassembled) = self.reassembler.feed(frag) {
                        // Reassembled — process as a normal incoming message
                        let msg = wire::InMessage {
                            topic_hash: reassembled.topic_hash,
                            payload: reassembled.payload,
                            timestamp_ns: reassembled.timestamp_ns,
                            sequence: reassembled.sequence,
                            priority: reassembled.priority,
                            reliability: reassembled.reliability,
                            encoding: reassembled.encoding,
                        };
                        self.process_incoming_message(&header, msg, from);
                    }
                }
            }
            return;
        }

        let (_, mut messages) = match wire::decode_packet(buf) {
            Some(r) => r,
            None => return,
        };

        // Dedup redundant copies
        self.reliability
            .dedup_messages(header.sender_id_hash, &mut messages);

        // Run optimizer incoming chain (reverse of outgoing)
        self.optimizers.process_incoming(&mut messages);

        for msg in messages {
            self.process_incoming_message(&header, msg, from);
        }
    }

    /// Process a single incoming data message (shared by regular and reassembled paths).
    fn process_incoming_message(
        &mut self,
        header: &PacketHeader,
        msg: wire::InMessage,
        from: SocketAddr,
    ) {
        // Track sequence gaps for flow control
        self.flow_control
            .on_received(header.sender_id_hash, msg.topic_hash, msg.sequence);

        // System topic dispatch — bypass normal import guard
        let presence_hash = wire::topic_hash(crate::registry::SYSTEM_TOPIC_PRESENCE);
        let logs_hash = wire::topic_hash(crate::registry::SYSTEM_TOPIC_LOGS);
        let estop_hash = wire::topic_hash(crate::registry::SYSTEM_TOPIC_ESTOP);

        if msg.topic_hash == presence_hash {
            self.presence_receiver.handle_broadcast(&msg.payload);
            return;
        }
        if msg.topic_hash == logs_hash {
            crate::log_replication::handle_remote_logs(&msg.payload);
            return;
        }
        if msg.topic_hash == estop_hash {
            crate::estop::handle_remote_estop(&msg.payload);
            return;
        }

        // Import guard: reject unauthorized topics
        let topic_name = self.topic_name_by_hash(msg.topic_hash);
        if let Some(ref name) = topic_name {
            if !self.guard.allow_import(name) {
                self.metrics.record_topic_drop(msg.topic_hash);
                horus_core::scheduling::record_external_event(
                    horus_core::scheduling::BlackBoxEvent::NetImportRejected {
                        topic: name.clone(),
                        peer_addr: from.to_string(),
                    },
                );
                return;
            }
        }

        // Type hash validation: reject messages with mismatched type hash.
        // Prevents silent data corruption when two machines publish different
        // types on the same topic name.
        if let Some(ref name) = topic_name {
            if let Some(local_entry) = self.registry.get_entry(name) {
                if local_entry.type_hash != 0
                    && msg.topic_hash != 0
                    && local_entry.type_hash != msg.topic_hash
                {
                    self.metrics.record_type_mismatch();
                    eprintln!(
                        "[horus_net] Type mismatch on topic '{}': local hash={:#x}, remote hash={:#x}. Rejecting import.",
                        name, local_entry.type_hash, msg.topic_hash
                    );
                    return;
                }
            }
        }

        // Encoding: handle cross-endian if needed
        let mut payload = msg.payload;
        let payload_len = payload.len();
        encoding::process_incoming_payload(&mut payload, msg.encoding, payload_len);

        // Write to local SHM
        if let Some(writer) = self.find_writer_by_hash(msg.topic_hash) {
            writer.write(&payload, msg.encoding);
            self.metrics
                .record_topic_recv(msg.topic_hash, payload.len());
        }

        // Send ACK for latched messages
        if msg.reliability == Reliability::Latched {
            let ack = wire::AckPayload {
                acked_topic_hash: msg.topic_hash,
                acked_sequence: msg.sequence,
            };
            let mut ack_buf = [0u8; 64];
            let ack_len = wire::encode_ack(
                self.peer_id_hash,
                self.next_packet_seq(),
                &ack,
                &mut ack_buf,
            );
            let _ = self.transport.send_to(&ack_buf[..ack_len], from);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // EXPORT PATH: Local SHM → Network
    // ═══════════════════════════════════════════════════════════════════════

    fn handle_export(&mut self) {
        let topic_names: Vec<String> = self
            .readers
            .keys()
            .filter(|name| self.peers.has_remote_subscribers(name))
            .filter(|name| self.guard.allow_export(name))
            .cloned()
            .collect();

        let mut outbox: Vec<wire::OutMessage> = Vec::new();

        for topic_name in topic_names {
            let reader = match self.readers.get_mut(&topic_name) {
                Some(r) => r,
                None => continue,
            };

            if let Some(raw) = reader.try_read_latest() {
                let topic_hash = wire::topic_hash(&topic_name);
                let priority = Priority::auto_infer(&topic_name, false, raw.data.len());
                let reliability = Reliability::default_for(priority);

                outbox.push(wire::OutMessage {
                    topic_name: topic_name.clone(),
                    topic_hash,
                    payload: raw.data,
                    timestamp_ns: 0,
                    sequence: raw.write_idx as u32,
                    priority,
                    reliability,
                    encoding: raw.encoding,
                });
            }
        }

        if outbox.is_empty() {
            return;
        }

        // Run optimizer chain (Immediate messages bypass automatically)
        self.optimizers.process_outgoing(&mut outbox);

        for msg in &outbox {
            self.metrics
                .record_topic_send(msg.topic_hash, msg.payload.len());

            // Fragment if needed
            let fragments = self.fragmenter.fragment(msg);

            let copies = ReliabilityLayer::copies_for(msg.reliability);
            let sub_addrs: Vec<SocketAddr> = self
                .peers
                .subscribers_of(&msg.topic_name)
                .iter()
                .map(|p| p.data_addr())
                .collect();

            for frag in &fragments {
                let header = PacketHeader::new(
                    if frag.fragment_count > 1 {
                        PacketFlags::empty().with(PacketFlags::FRAGMENT)
                    } else {
                        PacketFlags::empty()
                    },
                    self.peer_id_hash,
                    self.next_packet_seq(),
                );

                // Encode single fragment as packet
                let frag_msg = wire::OutMessage {
                    topic_name: msg.topic_name.clone(),
                    topic_hash: frag.topic_hash,
                    payload: frag.payload.clone(),
                    timestamp_ns: frag.timestamp_ns,
                    sequence: frag.sequence,
                    priority: frag.priority,
                    reliability: frag.reliability,
                    encoding: frag.encoding,
                };

                let mut send_buf = [0u8; 65536];
                let len = wire::encode_single(&header, &frag_msg, &mut send_buf);

                for _ in 0..copies {
                    for addr in &sub_addrs {
                        if msg.priority == Priority::Immediate
                            || msg.priority == Priority::RealTime
                            || self
                                .flow_control
                                .should_send(msg.topic_hash, self.peer_id_hash)
                        {
                            let _ = self.transport.send_to(&send_buf[..len], *addr);
                            self.metrics.record_send(self.peer_id_hash, len);
                        }
                    }
                }
            }

            // Start latch tracking for latched messages
            if msg.reliability == Reliability::Latched {
                self.reliability
                    .start_latch(msg.topic_hash, msg.sequence, msg.payload.clone());
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TIMER PATH: Discovery + Heartbeat + Liveness + Resends
    // ═══════════════════════════════════════════════════════════════════════

    fn handle_timer(&mut self) {
        let now = Instant::now();

        // Safety heartbeat: send to matched peers + check for link loss
        let link_lost =
            self.heartbeat
                .tick(&self.transport, self.peer_id_hash, &mut self.packet_seq);
        for (peer_id, action) in link_lost {
            match action {
                LinkLostAction::Warn => {
                    // Already logged by heartbeat.tick()
                }
                LinkLostAction::SafeState | LinkLostAction::Stop => {
                    horus_core::scheduling::trigger_external_emergency_stop(format!(
                        "Network peer {:02X?}... lost — {:?} action triggered",
                        &peer_id[..4],
                        action
                    ));
                }
            }
        }

        // Latched message resends
        let resends = self.reliability.tick_resends();
        for (topic_hash, sequence, payload) in resends {
            let header = PacketHeader::new(
                PacketFlags::empty(),
                self.peer_id_hash,
                self.next_packet_seq(),
            );
            let msg = wire::OutMessage {
                topic_name: String::new(), // Not needed for encoding
                topic_hash,
                payload,
                timestamp_ns: 0,
                sequence,
                priority: Priority::Immediate,
                reliability: Reliability::Latched,
                encoding: Encoding::PodLe,
            };
            let mut buf = [0u8; 65536];
            let len = wire::encode_single(&header, &msg, &mut buf);
            // Resend to all known peers (latched = broadcast)
            for peer in self.peers.alive_peers() {
                let _ = self.transport.send_to(&buf[..len], peer.data_addr());
            }
        }

        // Fragment reassembly GC
        self.reassembler.gc_stale();

        // Discovery (every 1s)
        if now.duration_since(self.last_discovery) >= DISCOVERY_INTERVAL {
            self.send_discovery_announcement();
            self.last_discovery = now;

            let dead = self.peers.check_liveness();
            if !dead.is_empty() {
                for dead_id in &dead {
                    eprintln!(
                        "[horus_net] Peer {:02X?}... lost (discovery timeout)",
                        &dead_id[..4]
                    );
                    self.heartbeat.remove_peer(dead_id);
                    horus_core::scheduling::record_external_event(
                        horus_core::scheduling::BlackBoxEvent::NetPeerLost {
                            peer_addr: format!("{:02X?}", &dead_id[..4]),
                            reason: "discovery timeout".into(),
                        },
                    );
                }
                self.peers.remove_dead_peers();
                self.update_matches();
            }

            if let Some(msg) = check_no_peers_diagnostic(
                self.peers.alive_count(),
                self.start_time,
                NO_PEERS_TIMEOUT,
            ) {
                eprintln!("{msg}");
            }

            // Presence broadcast (same 1s interval as discovery)
            if let Some(payload) = crate::presence::build_local_presence(self.peer_id_hash) {
                self.send_system_topic(crate::registry::SYSTEM_TOPIC_PRESENCE, &payload);
            }
        }

        // Log drain (every timer tick — batching handled internally)
        if let Some(payload) = self.log_drain.poll() {
            self.send_system_topic(crate::registry::SYSTEM_TOPIC_LOGS, &payload);
        }

        // E-stop retry processing
        let peer_addrs: Vec<SocketAddr> = self
            .peers
            .alive_peers()
            .iter()
            .map(|p| p.data_addr())
            .collect();
        let mcast = self.multicast_addr();
        self.estop_broadcaster
            .tick(&self.transport, mcast, &peer_addrs);

        // Presence cleanup (every 5s)
        if now.duration_since(self.last_presence_cleanup) >= Duration::from_secs(5) {
            self.presence_receiver.cleanup_stale();
            self.last_presence_cleanup = now;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════

    /// Send a system topic payload to all known peers + multicast.
    fn send_system_topic(&self, topic_name: &str, payload: &[u8]) {
        let topic_hash = wire::topic_hash(topic_name);
        let priority = Priority::for_system_topic(topic_name);
        let reliability = Reliability::for_system_topic(topic_name);

        let header = PacketHeader::new(PacketFlags::empty(), self.peer_id_hash, self.packet_seq);
        let msg = wire::MessageHeader {
            topic_hash,
            payload_len: payload.len() as u32,
            timestamp_ns: 0,
            sequence: 0,
            priority,
            reliability,
            encoding: Encoding::Bincode, // system topics use their own encoding
            source_host: (self.peer_id_hash & 0xFF) as u8,
        };

        // Encode: packet header + message header + payload
        let total = PacketHeader::SIZE + wire::MessageHeader::SIZE + payload.len();
        if total > wire::MAX_UDP_PAYLOAD {
            return; // Too large for single packet — skip (presence should be small)
        }
        let mut buf = vec![0u8; total];
        header.encode(&mut buf[..PacketHeader::SIZE]);
        msg.encode(&mut buf[PacketHeader::SIZE..PacketHeader::SIZE + wire::MessageHeader::SIZE]);
        buf[PacketHeader::SIZE + wire::MessageHeader::SIZE..].copy_from_slice(payload);

        // Send to multicast
        if let Some(mcast) = self.multicast_addr() {
            let _ = self.transport.send_to(&buf, mcast);
        }
        // Send to all known peers
        for peer in self.peers.alive_peers() {
            let _ = self.transport.send_to(&buf, peer.data_addr());
        }
    }

    /// Get the multicast address if configured.
    fn multicast_addr(&self) -> Option<SocketAddr> {
        match self.config.discovery_mode() {
            DiscoveryMode::Multicast { ref group } => {
                format!("{group}:{}", self.config.port).parse().ok()
            }
            _ => None,
        }
    }

    fn send_discovery_announcement(&self) {
        let entries = self.registry.entries();
        let mut buf = [0u8; 65536];
        let len = encode_announcement(
            &self.peer_id,
            self.config.port,
            &self.secret_hash,
            &entries,
            &mut buf,
        );

        match self.config.discovery_mode() {
            DiscoveryMode::Multicast { ref group } => {
                let addr: SocketAddr = format!("{group}:9100")
                    .parse()
                    .unwrap_or_else(|_| "224.0.69.72:9100".parse().unwrap());
                let _ = self.transport.send_to(&buf[..len], addr);
            }
            DiscoveryMode::Unicast { ref peers } => {
                for peer_addr in peers {
                    let _ = self.transport.send_to(&buf[..len], *peer_addr);
                }
            }
        }
    }

    fn update_matches(&mut self) {
        let local_entries = self.registry.entries();
        let mut all_matches = Vec::new();

        for peer in self.peers.alive_peers() {
            let (peer_matches, warnings) = find_matches(&local_entries, &peer.topics);
            for w in warnings {
                eprintln!("[horus_net] {w}");
            }
            all_matches.extend(peer_matches);
        }

        let mut seen = std::collections::HashSet::new();
        all_matches.retain(|m| seen.insert(m.topic.clone()));

        for m in &all_matches {
            if m.export && !self.readers.contains_key(&m.topic) {
                self.readers
                    .insert(m.topic.clone(), ShmRingReader::new(&m.topic));
            }
            if m.import && !self.writers.contains_key(&m.topic) {
                if let Some(writer) = ShmRingWriter::open(&m.topic) {
                    self.writers.insert(m.topic.clone(), writer);
                }
            }
        }

        let matched_topics: std::collections::HashSet<_> =
            all_matches.iter().map(|m| m.topic.clone()).collect();
        self.readers.retain(|name, _| matched_topics.contains(name));
        self.writers.retain(|name, _| matched_topics.contains(name));

        self.matches = all_matches;
    }

    /// Reverse lookup: topic hash → topic name (from writers map).
    fn topic_name_by_hash(&self, hash: u32) -> Option<String> {
        self.writers
            .keys()
            .find(|name| wire::topic_hash(name) == hash)
            .cloned()
    }

    fn find_writer_by_hash(&mut self, topic_hash: u32) -> Option<&mut ShmRingWriter> {
        self.writers
            .iter_mut()
            .find(|(name, _)| wire::topic_hash(name) == topic_hash)
            .map(|(_, writer)| writer)
    }

    fn next_packet_seq(&mut self) -> u32 {
        let seq = self.packet_seq;
        self.packet_seq = self.packet_seq.wrapping_add(1);
        seq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replicator_creation() {
        let registry = Arc::new(TopicRegistry::new());
        let config = NetConfig::test_config(0);
        let replicator = Replicator::new(registry, config).unwrap();
        assert!(replicator.running.load(Ordering::Relaxed));
        assert!(replicator.readers.is_empty());
        assert!(replicator.writers.is_empty());
        assert!(replicator.matches.is_empty());
    }

    #[test]
    fn packet_seq_increments() {
        let registry = Arc::new(TopicRegistry::new());
        let config = NetConfig::test_config(0);
        let mut replicator = Replicator::new(registry, config).unwrap();
        assert_eq!(replicator.next_packet_seq(), 0);
        assert_eq!(replicator.next_packet_seq(), 1);
        assert_eq!(replicator.next_packet_seq(), 2);
    }

    #[test]
    fn replicator_has_all_modules() {
        let registry = Arc::new(TopicRegistry::new());
        let config = NetConfig::test_config(0);
        let rep = Replicator::new(registry, config).unwrap();
        // Verify all Phase 2 modules are initialized
        assert_eq!(rep.heartbeat.peer_count(), 0);
        assert_eq!(rep.reliability.pending_latches(), 0);
        assert_eq!(rep.reassembler.pending_count(), 0);
        assert_eq!(rep.flow_control.tracked_count(), 0);
        assert!(rep.optimizers.is_empty()); // No optimizers by default
    }
}
