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

/// Most messages one topic contributes to a single export tick.
///
/// Only a streaming topic (`ExportSampling::AllSlots`) can reach it; a sampled
/// one contributes one. It bounds the work and the burst of a stream topic without
/// losing anything: `ShmRingReader` resumes from its own cursor, so the
/// remainder is exported on the next tick rather than dropped. At the 50 ms
/// timer that is 1280 messages/second/topic sustained, which covers a 1 kHz
/// joint-state stream with room to catch up after a stall.
const MAX_EXPORT_BATCH: usize = 64;

/// The Replicator — one per process, started by the Scheduler.
/// Size of the packet scratch buffers.
///
/// A UDP payload maxes at [`wire::MAX_UDP_PAYLOAD`] (65507); rounded up to a
/// power of two so the encoders never have to bounds-check against an odd
/// limit.
const MAX_PACKET_BYTES: usize = 65536;

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
    // ── Inbound admission control (GHSA-3frr-c2j9-hhr7) ──
    /// Source-address allowlist, applied to every datagram before any parsing.
    peer_filter: crate::netfilter::PeerFilter,
    /// Per-system-topic token buckets, so halting the fleet is not free.
    system_limits: crate::netfilter::SystemTopicLimits,
    /// Count of datagrams dropped by `peer_filter`, logged at most once.
    filtered_count: u64,
    /// Scratch buffer for encoding one outgoing packet.
    ///
    /// This was `let mut send_buf = [0u8; 65536]` inside the per-fragment loop.
    /// A zero-initialised 64 KB stack array is not free and LLVM does not elide
    /// it — measured at 1145 ns per construction against 2.8 ns for a reused
    /// buffer, so every fragment this node exported paid a microsecond to zero
    /// memory it was about to overwrite.
    ///
    /// Boxed rather than inline so `Replicator` itself stays small enough to
    /// move cheaply.
    send_buf: Box<[u8; MAX_PACKET_BYTES]>,
    /// Scratch buffer for one inbound datagram. Same reasoning as
    /// [`Self::send_buf`]: this was a fresh `[0u8; 65536]` on entry to
    /// `handle_incoming`, i.e. ~1.1 us of zeroing per event-loop iteration
    /// before a single packet was read.
    recv_buf: Option<Box<[u8; MAX_PACKET_BYTES]>>,
    filtered_logged: bool,
    /// Monotonic sequence stamped on framed system-topic messages.
    ///
    /// Shared across the three system topics; dedup is keyed per topic, so a
    /// shared counter still yields a strictly increasing sequence for each.
    system_seq: u32,
}

impl Replicator {
    /// Create a new Replicator.
    pub fn new(registry: Arc<TopicRegistry>, config: NetConfig) -> std::io::Result<Self> {
        let transport = UdpTransport::bind(config.port)?;

        if let DiscoveryMode::Multicast { ref group } = config.discovery_mode() {
            if let Err(e) = transport.join_multicast(group) {
                horus_core::terminal::eprint_line(&format!(
                    "[horus_net] Failed to join multicast group: {e}"
                ));
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
            horus_core::terminal::eprint_line(&format!(
                "[horus_net] WSL2 detected — using relaxed timeouts (heartbeat {}ms, {} missed threshold)",
                config.safety.heartbeat_ms, config.safety.missed_threshold
            ));
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
            peer_filter: crate::netfilter::PeerFilter::from_env(),
            system_limits: crate::netfilter::SystemTopicLimits::default(),
            filtered_count: 0,
            send_buf: Box::new([0u8; MAX_PACKET_BYTES]),
            recv_buf: Some(Box::new([0u8; MAX_PACKET_BYTES])),
            filtered_logged: false,
            system_seq: 0,
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
                horus_core::terminal::eprint_line(&format!(
                    "[horus_net] Failed to create event loop: {e}"
                ));
                return;
            }
        };

        #[cfg(unix)]
        if let Err(e) = event_loop.register_udp(self.transport.raw_fd()) {
            horus_core::terminal::eprint_line(&format!(
                "[horus_net] Failed to register UDP socket: {e}"
            ));
            return;
        }

        // Never hand a raw fd across an ownership boundary. `shm_notify_fd` is
        // owned by `event_loop`, which lives on this stack frame, but the
        // callback is installed into the process-lifetime registry singleton and
        // outlives it: every `Topic<T>` dropped during scheduler shutdown — which
        // happens AFTER this thread has joined and `EpollLoop::drop` has closed
        // the descriptor — still wrote 8 bytes to that descriptor *number*. By
        // then the number may have been reissued to an unrelated file the
        // shutdown path opened (blackbox WAL flush, crash report), and the
        // discarded return value made the corruption silent.
        //
        // The closure now owns a dup of the eventfd description instead. Writing
        // into it after shutdown is harmless — nothing reads it — and it is
        // released when `clear_on_change` below drops the callback.
        let notify_fd = unsafe { libc::dup(event_loop.shm_notify_fd()) };
        if notify_fd < 0 {
            // Not fatal: `handle_timer` drives the export path every tick, so
            // losing the wakeup only delays a newly registered topic's first
            // export by up to TIMER_INTERVAL.
            horus_core::terminal::eprint_line(&format!(
                "[horus_net] Could not duplicate the export notify fd ({}); topic \
                 registration will not wake the event loop early",
                std::io::Error::last_os_error()
            ));
        } else {
            let notify_fd =
                unsafe { <std::os::fd::OwnedFd as std::os::fd::FromRawFd>::from_raw_fd(notify_fd) };
            self.registry.set_on_change(move || {
                let val: u64 = 1;
                unsafe {
                    libc::write(
                        std::os::fd::AsRawFd::as_raw_fd(&notify_fd),
                        &val as *const u64 as *const libc::c_void,
                        8,
                    );
                }
            });
        }

        while self.running.load(Ordering::Relaxed) {
            let events = match event_loop.wait(1000) {
                Ok(events) => events,
                Err(e) => {
                    if e.kind() != std::io::ErrorKind::Interrupted {
                        horus_core::terminal::eprint_line(&format!(
                            "[horus_net] Event loop error: {e}"
                        ));
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

        // Drop the callback before `event_loop` goes out of scope. The registry
        // is a process-lifetime singleton, so a callback left installed here
        // survives this thread and keeps firing on every topic unregistered
        // during scheduler shutdown.
        self.registry.clear_on_change();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // IMPORT PATH: Network → Local SHM
    // ═══════════════════════════════════════════════════════════════════════

    fn handle_incoming(&mut self) {
        // Bounded drain. This used to loop until the socket was empty, which
        // never happens under a sustained inbound rate — and because `run()`
        // dispatches the Timer as a SIBLING match arm, reached only after this
        // returns, a flood starved every periodic safety action: outbound
        // heartbeats, peer liveness, reassembly GC, and the e-stop broadcast
        // drain. (The local halt is latched in `safety_monitor`, so the fleet
        // notification was delayed rather than lost — which is what keeps this
        // out of critical — but delay is not acceptable for an e-stop.)
        //
        // The budget is sized for legitimate throughput, not just for attack
        // mitigation: at the 50ms timer interval this still admits ~5k
        // datagrams/sec of real traffic before the cap binds, well above what a
        // LAN robot fleet produces. Anything left in the socket is read on the
        // next wakeup; the event loop is edge-triggered but the timer fires
        // every TIMER_INTERVAL regardless, so nothing is stranded.
        const MAX_DATAGRAMS_PER_WAKEUP: usize = 256;
        // Move the scratch buffer out of `self` for the duration of the drain.
        // `process_packet` takes `&mut self`, so a buffer borrowed FROM self
        // cannot be passed to it; taking ownership splits the borrow without
        // allocating. Restored below, and re-created if a panic unwound past
        // the restore, so a single failure cannot leave the replicator without
        // a receive buffer.
        let mut buf = self
            .recv_buf
            .take()
            .unwrap_or_else(|| Box::new([0u8; MAX_PACKET_BYTES]));
        for _ in 0..MAX_DATAGRAMS_PER_WAKEUP {
            match self.transport.recv_from(buf.as_mut_slice()) {
                Ok((n, from)) => {
                    self.metrics.record_recv(self.peer_id_hash, n);
                    self.process_packet(&buf[..n], from);
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        self.recv_buf = Some(buf);
    }

    /// Route one received datagram to its handler.
    ///
    /// Dispatch contract (relies on the §1 bit-7 discriminator so the branches are
    /// mutually exclusive — before it, `decode_announcement` swallowed everything):
    ///   1. Announcement (flags bit 7 set)  → peer table + discovery/matching
    ///   2. Heartbeat    (`PacketFlags::HEARTBEAT`) → `heartbeat.on_received`
    ///   3. Ack          (`PacketFlags::ACK`)       → `reliability.on_ack`
    ///   4. Fragment     (`PacketFlags::FRAGMENT`)  → reassembly → message dispatch
    ///   5. Data         (no special flag)          → `decode_packet` → message dispatch
    ///
    /// Fragment and data both funnel into `process_incoming_message`, which
    /// dispatches system topics (presence / logs / estop) and normal imports.
    fn process_packet(&mut self, buf: &[u8], from: SocketAddr) {
        // 0. Source admission control (GHSA-3frr-c2j9-hhr7).
        //
        //    This must be the FIRST thing every datagram meets. The socket binds
        //    0.0.0.0, and the system-topic dispatch further down returns early —
        //    before the import guard — so `_horus.estop`, `_horus.logs` and
        //    `_horus.presence` were reachable by any routable host. Reordering the
        //    guard would not have helped: `ImportExportGuard::allow_import` returns
        //    `true` unconditionally for system topics. Filtering here covers all
        //    five packet kinds and all three system topics with one check.
        //
        //    Default posture is private/loopback/link-local only; set
        //    HORUS_NET_ALLOW_PEERS to widen or narrow it. This limits *reach*, not
        //    identity — HORUS_ESTOP_KEY (crate::mac) is what authenticates the
        //    safety channel.
        if !self.peer_filter.admits(from) {
            self.filtered_count += 1;
            if !self.filtered_logged {
                self.filtered_logged = true;
                horus_core::terminal::eprint_line(&format!(
                    "[horus_net] Dropped a datagram from {from} — outside the allowed \
                     peer range. Set HORUS_NET_ALLOW_PEERS to admit it (further drops \
                     are counted, not logged)."
                ));
            }
            return;
        }

        // 1. Discovery announcement — self-filters on the bit-7 discriminator, so a
        //    real heartbeat/ack/fragment/data packet no longer matches here.
        if let Some(ann) = decode_announcement(buf, from) {
            if ann.peer_id == self.peer_id {
                return;
            }
            // Fails CLOSED. This used to also require `ann.has_secret`, so a
            // peer that simply cleared the has_secret flag bit skipped the check
            // entirely and HORUS_NET_SECRET filtered nobody. When we have a
            // secret configured, an announcement must carry a matching one.
            //
            // Still only a filter, not authentication: the hash is a
            // non-cryptographic FNV-1a value broadcast in cleartext, so anyone
            // who can see the traffic can replay it. HORUS_ESTOP_KEY is the
            // authenticated channel; this just stops accidental cross-fleet
            // mixing from silently succeeding.
            if self.secret_hash != [0u8; 4]
                && (!ann.has_secret || ann.secret_hash != self.secret_hash)
            {
                return;
            }
            // Only register a heartbeat emitter for a peer the table actually
            // accepted. This used to be unconditional, so every announcement
            // the (capped) peer table refused still created an uncapped
            // heartbeat entry keyed on the attacker-chosen peer_id — one that
            // nothing ever removed and that `tick` used to transmit to the
            // announcement's spoofable source address every 50 ms, forever.
            // One forged datagram bought permanent outbound traffic aimed
            // wherever the sender liked.
            if self.peers.update_peer(&ann) {
                self.heartbeat.add_peer(ann.peer_id, ann.source_addr);
            }
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

        // Not an announcement: decode the packet header and route by flags.
        let header = match wire::PacketHeader::decode(buf) {
            Some(h) => h,
            None => return,
        };

        // 2. Heartbeat packet
        if header.flags.heartbeat() {
            if let Some(hb) = wire::decode_heartbeat(buf) {
                self.heartbeat.on_received(&hb.peer_id);
            }
            return;
        }

        // 3. ACK packet
        if header.flags.ack() {
            if let Some(ack) = wire::decode_ack(buf) {
                self.reliability.on_ack(&ack);
            }
            return;
        }

        // 4. Fragmented data packet — reassemble, then dispatch as a message
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
                        // The reassembly key includes the sender: `fragment_id`
                        // is a per-process counter every peer starts at 0, so
                        // without this two peers publishing the same topic
                        // reassemble into a single buffer and one message is
                        // spliced together from both of their fragments.
                        sender_id_hash: header.sender_id_hash,
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

        // 5. Regular (unfragmented) data packet
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
    ///
    /// **Nothing on this path authenticates the sender.** `peer_filter` limits
    /// *reach*, the import guard limits *which topics*, and the type-hash check
    /// compares against a hash the sender itself announced — so a host inside the
    /// allowed peer range can write arbitrary bytes into any importable local SHM
    /// topic, actuation commands included, and can replay anything it captured.
    /// Only `_horus.estop` is authenticated (HMAC, `HORUS_ESTOP_KEY`). See the
    /// crate-level trust model; closing this needs a per-datagram MAC and a
    /// replay window, which is a wire-format change.
    fn process_incoming_message(
        &mut self,
        header: &PacketHeader,
        msg: wire::InMessage,
        from: SocketAddr,
    ) {
        // Track sequence gaps for flow control.
        //
        // Only for a datagram we can attribute to a peer we actually know at this
        // source address. This is the one place where inbound, unauthenticated
        // wire fields feed a decision on the OUTBOUND side: `handle_export` gates
        // every non-Immediate send on `flow_control.should_send(peer_id_hash,
        // topic_hash)`. Recording unconditionally meant any host the source
        // filter admits could file a fabricated loss statistic under a legitimate
        // peer's id hash — a 16-bit value it can simply read out of that peer's
        // cleartext announcements — for any topic hash it liked, and thereby
        // throttle this robot's replication of a topic it cannot write.
        //
        // That is an effect on a topic the import guard exists to protect: a
        // topic we PUBLISH is denied to remote writers precisely so a peer cannot
        // fight the commands produced here, and suppressing our sends reaches the
        // same subscriber with the same staleness. `sender_id_hash` names nobody
        // on its own, so it must not be a key we act on by itself.
        if self
            .peers
            .id_hash_announced_from(from, header.sender_id_hash)
        {
            self.flow_control
                .on_received(header.sender_id_hash, msg.topic_hash, msg.sequence);
        }

        // System topic dispatch — bypass normal import guard
        let presence_hash = wire::topic_hash(crate::registry::SYSTEM_TOPIC_PRESENCE);
        let logs_hash = wire::topic_hash(crate::registry::SYSTEM_TOPIC_LOGS);
        let estop_hash = wire::topic_hash(crate::registry::SYSTEM_TOPIC_ESTOP);

        // Each system topic is metered by its own token bucket. The source has
        // already passed `peer_filter`, but a permitted-but-hostile LAN host could
        // otherwise halt-and-log at line rate; on an RT node that is a jitter and
        // disk-fill primitive as much as a safety one. E-stop's bucket is sized to
        // pass a genuine triple-redundant burst (see `SystemTopicLimits`).
        if msg.topic_hash == presence_hash {
            if self.system_limits.presence.take() {
                self.presence_receiver.handle_broadcast(&msg.payload);
            } else {
                self.metrics.record_topic_drop(msg.topic_hash);
            }
            return;
        }
        if msg.topic_hash == logs_hash {
            if self.system_limits.logs.take() {
                crate::log_replication::handle_remote_logs(&msg.payload);
            } else {
                self.metrics.record_topic_drop(msg.topic_hash);
            }
            return;
        }
        if msg.topic_hash == estop_hash {
            // Authenticate BEFORE charging the rate limit.
            //
            // Charging first meant a forged flood — packets that fail the MAC and
            // do nothing — drained the bucket, so the very next GENUINE halt was
            // dropped for want of a token. The limiter existed to stop an
            // attacker halting the fleet at line rate; ordering it ahead of
            // authentication let the attacker use it to SUPPRESS a halt instead,
            // which is the strictly worse direction.
            //
            // A forged packet (or any packet when no key is provisioned) is
            // rejected without spending anything, and only packets that could
            // actually halt the robot consume budget. Verification is an HMAC over at most a few
            // hundred bytes and the source has already passed `peer_filter`, so
            // the CPU an unauthenticated flood can buy is bounded.
            //
            if !crate::estop::estop_packet_is_authentic(&msg.payload) {
                self.metrics.record_topic_drop(msg.topic_hash);
                return;
            }
            if self.system_limits.estop.take() {
                crate::estop::handle_remote_estop(&msg.payload, self.config.estop_remote);
            } else {
                self.metrics.record_topic_drop(msg.topic_hash);
            }
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
                // Compare like with like. This used to be
                // `local_entry.type_hash != msg.topic_hash` — a hash of the TYPE
                // name against a hash of the TOPIC name. Those are hashes of
                // different strings, so the condition was true for every message
                // on any topic with a registered type, and every legitimate
                // import was rejected. `InMessage` has no type_hash field at all
                // (the 24-byte MessageHeader has no room for one), so the peer's
                // discovery announcement is the only source of its type identity.
                //
                // A peer chooses what it announces, so this catches accidental
                // type mismatch between machines — its stated purpose — and is
                // not a defence against a hostile peer. Payload size is checked
                // unconditionally in ShmRingWriter::write_pod, which is what
                // actually stops a wrong-sized write.
                let remote_type_hash = self.peers.announced_type_hash(from, name).unwrap_or(0);
                if local_entry.type_hash != 0
                    && remote_type_hash != 0
                    && local_entry.type_hash != remote_type_hash
                {
                    self.metrics.record_type_mismatch();
                    horus_core::terminal::eprint_line(&format!(
                        "[horus_net] Type mismatch on topic '{}': local type hash={:#x}, peer-announced type hash={:#x}. Rejecting import.",
                        name, local_entry.type_hash, remote_type_hash
                    ));
                    return;
                }
            }
        }

        // Encoding: handle cross-endian if needed
        let mut payload = msg.payload;
        let payload_len = payload.len();
        encoding::process_incoming_payload(&mut payload, msg.encoding, payload_len);

        // Write to local SHM.
        //
        // `write` returns false when the payload does not match the topic's
        // layout (a POD topic whose type_size differs, most often). The result
        // used to be discarded and `record_topic_recv` bumped unconditionally,
        // so a subscriber that never saw another sample still looked, in the
        // metrics, like it was receiving everything.
        let mut accepted = false;
        if let Some(writer) = self.find_writer_by_hash(msg.topic_hash) {
            if writer.write(&payload, msg.encoding) {
                accepted = true;
                self.metrics
                    .record_topic_recv(msg.topic_hash, payload.len());
            } else {
                self.metrics.record_topic_drop(msg.topic_hash);
            }
        }

        // Send ACK for latched messages — but only for a message we actually
        // accepted, and only to a peer we know at this source address.
        //
        // This used to fire on the reliability BYTE alone, whatever became of the
        // message. Two consequences, and the first is the worse one:
        //
        // 1. It ACKed messages this node DISCARDED. `Reliability::Latched` is the
        //    tier that exists to guarantee delivery of safety-critical state
        //    changes: the sender resends every 10ms until acknowledged. But an
        //    ACK went out even when `find_writer_by_hash` found nothing, or when
        //    `ShmRingWriter::write` refused the payload (a POD topic whose
        //    type_size does not match — the check that stops a wrong-sized write).
        //    The sender then cancelled its latch, satisfied. Silent loss of a
        //    safety message, carrying a delivery confirmation, on the one tier
        //    built to make that impossible.
        //
        // 2. It was a reflector. `decode_packet` accepts a batch and a message
        //    header with `payload_len = 0` is 24 bytes, so one 65507-byte datagram
        //    carries ~2700 of them — each drawing a separate 20-byte ACK datagram
        //    to the packet's spoofable source address. ~2x in bytes but ~2700x in
        //    packet COUNT, aimable at any host the peer filter admits, including
        //    the robot's own control station.
        //
        // Both conditions are free for legitimate traffic: a latched import only
        // reaches here at all once discovery has matched the topic, which means
        // the sender is already in the peer table. An ACK withheld meanwhile is
        // exactly what latching is for — the sender resends.
        if accepted && self.peers.has_peer_at(from) && msg.reliability == Reliability::Latched {
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
        // Collected while the readers are borrowed, applied once the loop ends.
        let mut skipped: Vec<(u32, u64)> = Vec::new();
        let mut reports: Vec<String> = Vec::new();
        let now = Instant::now();

        for topic_name in topic_names {
            let reader = match self.readers.get_mut(&topic_name) {
                Some(r) => r,
                None => continue,
            };

            // How much of the ring this topic's mode takes. A stream topic
            // hands back everything published since the previous tick, bounded
            // by MAX_EXPORT_BATCH; a sampled topic hands back the newest slot
            // and counts the rest as skipped.
            let before_skipped = reader.skipped_total();
            let batch = reader.read_pending(MAX_EXPORT_BATCH);
            let topic_hash = wire::topic_hash(&topic_name);
            skipped.push((
                topic_hash,
                reader.skipped_total().saturating_sub(before_skipped),
            ));

            // Say it out loud, on the topic's own terms and at most once every
            // SKIP_REPORT_INTERVAL. A replicated topic that is being decimated
            // has to announce it: from the far end, odometry arriving at 20 Hz
            // because the exporter sampled it is indistinguishable from odometry
            // that is published at 20 Hz, and the fleet operator debugging a
            // drifting position estimate has no thread to pull on.
            if let Some(report) = reader.due_report(now) {
                reports.push(report.message());
            }

            for raw in batch {
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

        for (topic_hash, count) in skipped {
            self.metrics.record_topic_skipped(topic_hash, count);
        }
        for line in reports {
            horus_core::terminal::eprint_line(&line);
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
            // Carry each subscriber's own id hash alongside its address: the
            // send throttle below is per-destination, and flow-control state
            // is keyed by the hash that peer stamps into ITS packets.
            let sub_addrs: Vec<(u16, SocketAddr)> = self
                .peers
                .subscribers_of(&msg.topic_name)
                .iter()
                .map(|p| (p.id_hash(), p.data_addr()))
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

                // A fragmented message MUST carry a FragmentHeader. This used
                // to call `encode_single` for every fragment, which emits none,
                // while the receive path reads one straight after the
                // MessageHeader — so it consumed the first 12 bytes of the
                // chunk as fragment metadata and reassembly of anything over
                // MAX_FRAGMENT_PAYLOAD never produced the original message.
                let len = if frag.fragment_count > 1 {
                    let fh = wire::FragmentHeader {
                        fragment_id: frag.fragment_id,
                        fragment_index: frag.fragment_index,
                        fragment_count: frag.fragment_count,
                        total_payload_len: frag.total_payload_len,
                    };
                    wire::encode_fragment(&header, &frag_msg, &fh, self.send_buf.as_mut_slice())
                } else {
                    wire::encode_single(&header, &frag_msg, self.send_buf.as_mut_slice())
                };
                // 0 = did not fit; skip rather than send a truncated packet.
                if len == 0 {
                    continue;
                }

                for _ in 0..copies {
                    for (sub_id_hash, addr) in &sub_addrs {
                        if msg.priority == Priority::Immediate
                            || msg.priority == Priority::RealTime
                            || self.flow_control.should_send(*sub_id_hash, msg.topic_hash)
                        {
                            let _ = self.transport.send_to(&self.send_buf[..len], *addr);
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

        // Export whatever publishers have written since the last tick.
        //
        // `handle_export` polls the SHM readers, but until now the only thing
        // that reached it was `EventSource::ShmNotify`, and the only writer of
        // that eventfd is `TopicRegistry::notify_change` — which fires on
        // register and unregister and nothing else. So a topic was exported
        // when it was *created* and then never again, however much data flowed
        // through it. Outbound replication stopped as soon as the topology
        // settled, which is the moment a robot starts doing useful work.
        //
        // Driving it from the timer makes export data-driven at the tick rate
        // rather than topology-driven — but the tick rate then becomes the
        // export rate. `try_read_latest` takes the newest sample and leaves the
        // rest, so EVERY replicated topic was pinned at ~20 Hz however fast its
        // publisher ran, and nothing counted or said so: a 500 Hz odometry
        // stream crossed the LAN at 20 Hz and looked, at the far end, exactly
        // like a 20 Hz odometry stream. Sampling is right for state and wrong
        // for a measurement stream, so it is now a per-topic choice
        // (`export_stream` / `HORUS_NET_EXPORT_STREAM`), and whichever way a
        // topic is set, what it drops is counted and reported.
        //
        // The eventfd path stays: a newly registered topic is still exported
        // immediately rather than waiting up to TIMER_INTERVAL.
        self.handle_export();

        // Safety heartbeat: send to matched peers + check for link loss
        let link_lost =
            self.heartbeat
                .tick(&self.transport, self.peer_id_hash, &mut self.packet_seq);
        for (peer_id, action) in link_lost {
            match action {
                LinkLostAction::Warn => {
                    // Already logged by heartbeat.tick()
                }
                LinkLostAction::SafeState => {
                    // Distinct from Stop. An operator who configured
                    // `safety.on_link_lost = "safe_state"` chose per-node safing
                    // over halting the robot; routing both here to
                    // `trigger_external_emergency_stop` gave them the full halt
                    // and preserved their choice only in the log line.
                    horus_core::scheduling::trigger_external_safe_state(format!(
                        "Network peer {:02X?}... lost — entering safe state",
                        &peer_id[..4]
                    ));
                }
                LinkLostAction::Stop => {
                    horus_core::scheduling::trigger_external_emergency_stop(format!(
                        "Network peer {:02X?}... lost — emergency stop",
                        &peer_id[..4]
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
            let len = wire::encode_single(&header, &msg, self.send_buf.as_mut_slice());
            if len == 0 {
                continue; // did not fit; skip rather than send an empty datagram
            }
            // Resend to all known peers (latched = broadcast)
            for peer in self.peers.alive_peers() {
                let _ = self
                    .transport
                    .send_to(&self.send_buf[..len], peer.data_addr());
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
                    horus_core::terminal::eprint_line(&format!(
                        "[horus_net] Peer {:02X?}... lost (discovery timeout)",
                        &dead_id[..4]
                    ));
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

            // Re-sync heartbeat membership against the peer table every
            // discovery interval. `remove_peer` above only fires for peers the
            // peer table reported as newly dead; a peer evicted to make room
            // for another (peer.rs cap path) is never reported that way, so its
            // heartbeat entry would otherwise live — and transmit — forever.
            let known = &self.peers;
            self.heartbeat.retain_peers(|id| known.get(id).is_some());

            if let Some(msg) = check_no_peers_diagnostic(
                self.peers.alive_count(),
                self.start_time,
                NO_PEERS_TIMEOUT,
            ) {
                horus_core::terminal::eprint_line(&msg);
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
        let retry_delivery = self
            .estop_broadcaster
            .tick(&self.transport, mcast, &peer_addrs);
        if self.estop_broadcaster.should_warn(&retry_delivery) {
            horus_core::terminal::eprint_line(
                "[horus_net] CRITICAL: every e-stop retry failed to send; peers have \
                 NOT been notified. Check the network interface and multicast route.",
            );
        }

        // SEND path (the other half of NET-F1): announce this robot's OWN e-stop to
        // the fleet. Drained here on the timer tick (worst-case ~TIMER_INTERVAL=50ms
        // latency — a CONSCIOUS tradeoff vs the current path which never sends at all;
        // an immediate wake via the event-loop eventfd is a possible follow-up if a
        // <5ms bound is required). take_pending_local_estop() returns Some ONLY for
        // local-origin rising-edge e-stops (see safety_monitor.rs) — received e-stops
        // are never re-broadcast (anti-storm invariant).
        //
        // Known limitation: a node joining an already-e-stopped fleet won't learn it
        // (no join-time state sync). One broadcast per episode — EstopBroadcaster's
        // triple-redundant multicast+unicast + 3× retry handles delivery reliability.
        //
        // RESTS-ON-DESIGN: real cross-process delivery is not exercised here (the
        // sandbox's cross-process UDP/SHM is env-broken, same as NET-F1's gate test);
        // the send seam is unit-tested at EstopBroadcaster::broadcast in estop.rs.
        if let Some(reason) = horus_core::scheduling::take_pending_local_estop() {
            let payload = crate::estop::encode_estop(self.peer_id_hash, &reason);
            // MUST be framed. EstopBroadcaster used to be handed the bare e-stop
            // body, which every receiver then dropped at the magic check in
            // PacketHeader::decode — so a genuine cross-machine e-stop never
            // propagated, while a forged one (framed correctly by the attacker)
            // did. Framing here rather than inside the broadcaster keeps the
            // retry copies byte-identical to the first send.
            match self.frame_system_topic(crate::registry::SYSTEM_TOPIC_ESTOP, &payload) {
                Some(framed) => {
                    // The sends are best-effort — a safety path must not block on
                    // a socket — but best-effort was written as `let _ =`, so a
                    // broadcast that reached nobody looked exactly like one that
                    // reached everybody. The framing failure below was already
                    // reported on the principle "never drop an e-stop silently";
                    // a dead interface is far likelier than an over-long reason
                    // string and was the one case that stayed quiet.
                    let delivery = self.estop_broadcaster.broadcast(
                        &self.transport,
                        framed,
                        mcast,
                        &peer_addrs,
                    );
                    if self.estop_broadcaster.should_warn(&delivery) {
                        horus_core::terminal::eprint_line(&format!(
                            "[horus_net] CRITICAL: local e-stop reached none of its {} \
                             destinations; peers have NOT been notified. Check the \
                             network interface and multicast route.",
                            delivery.attempted
                        ));
                    }
                }
                None => {
                    // Only reachable if the reason string somehow exceeded a
                    // datagram; encode_estop caps it at 200 bytes. Never drop an
                    // e-stop silently.
                    horus_core::terminal::eprint_line(
                        "[horus_net] CRITICAL: local e-stop could not be framed for \
                         broadcast (reason too long); peers will NOT be notified",
                    );
                }
            }
        }

        // Presence cleanup (every 5s)
        if now.duration_since(self.last_presence_cleanup) >= Duration::from_secs(5) {
            self.presence_receiver.cleanup_stale();
            self.last_presence_cleanup = now;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════

    /// Wrap a system-topic payload in the wire framing every receiver expects.
    ///
    /// `process_packet` dispatches on `PacketHeader::decode`, which rejects
    /// anything whose first four bytes are not the HORS magic. A payload sent
    /// without this framing is therefore silently dropped by every peer — which
    /// is exactly what was happening to networked e-stop (see the call site in
    /// `handle_timer`). Returns `None` if the framed packet would exceed one
    /// datagram.
    fn frame_system_topic(&mut self, topic_name: &str, payload: &[u8]) -> Option<Vec<u8>> {
        let topic_hash = wire::topic_hash(topic_name);
        let priority = Priority::for_system_topic(topic_name);
        let reliability = Reliability::for_system_topic(topic_name);

        // Sequence MUST advance per episode. Framed system messages take the
        // normal data path on the receiver, which runs
        // `reliability.dedup_messages` BEFORE the system-topic dispatch, and
        // `is_new_message` drops anything whose sequence is <= the last seen for
        // that (sender, topic). A fixed 0 would therefore deliver the FIRST
        // e-stop episode and silently drop every later one, forever — which is
        // exactly the failure mode framing was added to fix.
        //
        // Retries deliberately reuse the already-framed bytes, so they keep the
        // same sequence and are still deduped. That is the point of the
        // triple-redundant send.
        self.system_seq = self.system_seq.wrapping_add(1);
        let header = PacketHeader::new(PacketFlags::empty(), self.peer_id_hash, self.packet_seq);
        let msg = wire::MessageHeader {
            topic_hash,
            payload_len: payload.len() as u32,
            timestamp_ns: 0,
            sequence: self.system_seq,
            priority,
            reliability,
            encoding: Encoding::Bincode,
            source_host: (self.peer_id_hash & 0xFF) as u8,
        };

        let total = PacketHeader::SIZE + wire::MessageHeader::SIZE + payload.len();
        if total > wire::MAX_UDP_PAYLOAD {
            return None;
        }
        let mut buf = vec![0u8; total];
        header.encode(&mut buf[..PacketHeader::SIZE]);
        msg.encode(&mut buf[PacketHeader::SIZE..PacketHeader::SIZE + wire::MessageHeader::SIZE]);
        buf[PacketHeader::SIZE + wire::MessageHeader::SIZE..].copy_from_slice(payload);
        Some(buf)
    }

    /// Send a system topic payload to all known peers + multicast.
    fn send_system_topic(&mut self, topic_name: &str, payload: &[u8]) {
        // Share the framing (and therefore the monotonic sequence) with the
        // e-stop path. This used to build its own header with a hard-coded
        // `sequence: 0`, which the receiver's dedup — which runs ahead of the
        // system-topic dispatch — reads as "already seen" for every message
        // after the first. Presence and log replication were being deduped away
        // after their first packet for exactly the same reason e-stop was.
        let Some(buf) = self.frame_system_topic(topic_name, payload) else {
            return; // too large for a single datagram; presence should be small
        };

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
                horus_core::terminal::eprint_line(&format!("[horus_net] {w}"));
            }
            all_matches.extend(peer_matches);
        }

        let mut seen = std::collections::HashSet::new();
        all_matches.retain(|m| seen.insert(m.topic.clone()));

        for m in &all_matches {
            if m.export && !self.readers.contains_key(&m.topic) {
                let sampling = self.config.export_sampling(&m.topic);
                self.readers.insert(
                    m.topic.clone(),
                    ShmRingReader::new(&m.topic).with_sampling(sampling),
                );
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

    // ─── §2/§6 process_packet dispatch (gate) ───────────────────────────────
    // These drive the private receive path directly. Run with --test-threads=1:
    // the data-packet test installs the global external emergency-stop hook.

    fn test_replicator() -> Replicator {
        let registry = Arc::new(TopicRegistry::new());
        Replicator::new(registry, NetConfig::test_config(0)).unwrap()
    }

    #[test]
    fn process_packet_does_not_swallow_heartbeat_as_announcement() {
        // RED→GREEN: before the §1 bit-7 discriminator, a genuine heartbeat (≥30B,
        // shares MAGIC+VERSION) false-matched decode_announcement and was swallowed,
        // polluting the peer table with a bogus peer (alive_count would be 1). After
        // the fix it routes to the heartbeat path and the peer table stays clean.
        let mut rep = test_replicator();
        let payload = wire::HeartbeatPayload {
            peer_id: [0x55; 16],
            heartbeat_sequence: 1,
        };
        let mut buf = [0u8; 64];
        let len = wire::encode_heartbeat(0x1234, 1, &payload, &mut buf);
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();
        rep.process_packet(&buf[..len], from);
        assert_eq!(
            rep.peers.alive_count(),
            0,
            "heartbeat must not be swallowed as a bogus announcement/peer"
        );
    }

    #[test]
    fn process_packet_routes_real_announcement() {
        // Positive control: a genuine announcement (bit 7 set) registers a peer.
        let mut rep = test_replicator();
        let peer_id = [0x77; 16];
        let mut buf = [0u8; 4096];
        let len = crate::discovery::encode_announcement(&peer_id, 9100, &[0u8; 4], &[], &mut buf);
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();
        rep.process_packet(&buf[..len], from);
        assert_eq!(
            rep.peers.alive_count(),
            1,
            "real announcement registers a peer"
        );
    }

    #[test]
    fn forged_announcement_flood_cannot_grow_the_heartbeat_table() {
        // Regression for the UDP-reflector leak: the heartbeat table had no cap
        // of any kind and was populated unconditionally from every announcement,
        // including the ones the (capped) peer table refused. A flood of forged
        // peer ids grew it without bound and made every entry transmit to the
        // announcement's source address every 50ms, forever.
        let mut rep = test_replicator();
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();
        for i in 0..(crate::netfilter::MAX_PEERS * 4) {
            let mut peer_id = [0u8; 16];
            peer_id[..8].copy_from_slice(&(i as u64).to_le_bytes());
            let mut buf = [0u8; 4096];
            let len =
                crate::discovery::encode_announcement(&peer_id, 9100, &[0u8; 4], &[], &mut buf);
            rep.process_packet(&buf[..len], from);
            assert!(
                rep.heartbeat.peer_count() <= crate::netfilter::MAX_PEERS,
                "heartbeat table exceeded the peer cap at iteration {i}: {}",
                rep.heartbeat.peer_count()
            );
        }
        assert!(rep.peers.total_count() <= crate::netfilter::MAX_PEERS);
    }

    /// Build one data packet carrying a single message with the given fields.
    fn data_packet(
        sender_id_hash: u16,
        topic_hash: u32,
        sequence: u32,
        reliability: Reliability,
    ) -> Vec<u8> {
        let header = PacketHeader::new(PacketFlags::empty(), sender_id_hash, 1);
        let msg = wire::OutMessage {
            topic_name: String::new(),
            topic_hash,
            payload: vec![0u8; 8],
            timestamp_ns: 0,
            sequence,
            priority: Priority::Normal,
            reliability,
            encoding: Encoding::PodLe,
        };
        let mut buf = [0u8; 256];
        let len = wire::encode_single(&header, &msg, &mut buf);
        buf[..len].to_vec()
    }

    /// THE REGRESSION. `flow_control.on_received` was fed from every datagram
    /// that cleared the source filter, keyed on the packet's 16-bit
    /// `sender_id_hash` and its topic hash — both bare wire fields. `handle_export`
    /// then gates every non-Immediate send on
    /// `flow_control.should_send(subscriber_id_hash, topic_hash)`.
    ///
    /// So an unauthenticated host could file a fabricated loss statistic under a
    /// legitimate peer's identity — a value it reads straight out of that peer's
    /// cleartext announcements — and throttle this robot's export of a topic it
    /// has no other way to touch. Two datagrams were enough, and the state was
    /// only ever mutated from `on_received`, so it never recovered.
    ///
    /// A datagram that names no peer we know must leave the throttle untouched.
    #[test]
    fn an_unattributable_datagram_cannot_seed_flow_control_state() {
        let mut rep = test_replicator();
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();
        let topic = wire::topic_hash("cmd_vel");
        const VICTIM_HASH: u16 = 0xBEEF;

        // The two-packet poisoning burst, from a host in no peer table.
        rep.process_packet(&data_packet(VICTIM_HASH, topic, 1, Reliability::None), from);
        rep.process_packet(
            &data_packet(VICTIM_HASH, topic, 1_000_000, Reliability::None),
            from,
        );

        assert_eq!(
            rep.flow_control.tracked_count(),
            0,
            "an unknown source must not create per-peer send-throttle state"
        );
        assert!(
            rep.flow_control.should_send(VICTIM_HASH, topic),
            "export to a legitimate peer must not be throttled by a stranger's packets"
        );
    }

    /// Positive control: a datagram that IS attributable still feeds the
    /// throttle, so the fix above is a binding, not a disablement.
    #[test]
    fn an_attributable_datagram_still_feeds_flow_control() {
        let mut rep = test_replicator();
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();
        // A peer id whose truncated hash is NOT zero, so this test would fail if
        // the attribution check degenerated to "always true" or to comparing
        // against a default.
        let peer_id = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let real_hash = crate::discovery::peer_id_hash(&peer_id);
        assert_ne!(real_hash, 0, "fixture must exercise a distinguishing hash");

        let mut ann_buf = [0u8; 4096];
        let ann_len =
            crate::discovery::encode_announcement(&peer_id, 9100, &[0u8; 4], &[], &mut ann_buf);
        rep.process_packet(&ann_buf[..ann_len], from);
        assert_eq!(rep.peers.alive_count(), 1);

        let topic = wire::topic_hash("imu");

        // Same source address, but a sender hash this peer does not stamp: the
        // attacker's case, from inside an address the peer table knows.
        rep.process_packet(
            &data_packet(real_hash ^ 0x5A5A, topic, 1, Reliability::None),
            from,
        );
        assert_eq!(
            rep.flow_control.tracked_count(),
            0,
            "a hash no peer at this address announced must not be measured"
        );

        // The peer's own packet does get measured.
        rep.process_packet(&data_packet(real_hash, topic, 1, Reliability::None), from);
        assert_eq!(
            rep.flow_control.tracked_count(),
            1,
            "a known peer's own packets must still be measured"
        );
    }

    /// THE REGRESSION, in both of its forms. The ACK fired on the reliability
    /// byte alone, whatever became of the message.
    ///
    /// Correctness: `Reliability::Latched` is the tier that resends every 10ms
    /// until acknowledged, for safety-critical state changes. ACKing a message
    /// this node found no writer for — or that `ShmRingWriter::write` refused —
    /// told the sender its safety message had landed when it had been thrown
    /// away. This test drives the `no writer` half of that path; the refused-write
    /// half shares the same `accepted` flag.
    ///
    /// Amplification: `decode_packet` accepts a batch and a zero-payload message
    /// header is 24 bytes, so one 65507-byte datagram carries ~2700 of them — and
    /// every one drew a separate 20-byte ACK datagram to the packet's (spoofable)
    /// source address. ~2700x packet-count amplification, aimable at any host the
    /// peer filter admits.
    #[test]
    fn a_batch_of_latched_messages_for_unknown_topics_emits_no_acks() {
        let mut rep = test_replicator();
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();

        // A batch of zero-payload latched message headers, all naming topics
        // this node has no writer for.
        let header = PacketHeader::new(PacketFlags::empty().with(PacketFlags::BATCH), 0x1234, 1);
        let msgs: Vec<wire::OutMessage> = (0..64u32)
            .map(|i| wire::OutMessage {
                topic_name: String::new(),
                topic_hash: 0xF000_0000 + i,
                payload: Vec::new(),
                timestamp_ns: 0,
                sequence: i,
                priority: Priority::Normal,
                reliability: Reliability::Latched,
                encoding: Encoding::PodLe,
            })
            .collect();
        let mut buf = [0u8; 4096];
        let len = wire::encode_batch(&header, &msgs, &mut buf);

        let before = rep.packet_seq;
        rep.process_packet(&buf[..len], from);

        // `next_packet_seq` is called once per ACK sent, so the counter is an
        // exact record of how many this datagram elicited.
        assert_eq!(
            rep.packet_seq, before,
            "a datagram naming topics we do not import must draw no reply at all — \
             it used to draw one ACK per message header"
        );
        assert!(
            rep.writers.is_empty(),
            "fixture sanity: nothing was importable here"
        );
    }

    #[test]
    fn process_packet_rejects_unkeyed_estop_data_packet() {
        // Regression for GHSA-3frr-c2j9-hhr7: a fresh, well-formed network packet
        // must not reach the hook when no authentication key is provisioned.
        let fired = Arc::new(AtomicBool::new(false));
        let fired2 = fired.clone();
        horus_core::scheduling::set_emergency_stop_hook(move |_reason| {
            fired2.store(true, Ordering::SeqCst);
        });

        let mut rep = test_replicator(); // estop_remote defaults to Warn
        let estop_payload = crate::estop::encode_estop(0xABCD, "gate test"); // fresh ts
        let header = PacketHeader::new(PacketFlags::empty(), 0x1234, 1);
        let msg = wire::OutMessage {
            topic_name: crate::registry::SYSTEM_TOPIC_ESTOP.into(),
            topic_hash: wire::topic_hash(crate::registry::SYSTEM_TOPIC_ESTOP),
            payload: estop_payload,
            timestamp_ns: 0,
            sequence: 0,
            priority: Priority::Immediate,
            reliability: Reliability::None,
            encoding: Encoding::Bincode,
        };
        let mut buf = [0u8; 1024];
        let len = wire::encode_single(&header, &msg, &mut buf);
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();
        rep.process_packet(&buf[..len], from);

        assert!(
            !fired.load(Ordering::SeqCst),
            "unkeyed estop data packet must not dispatch to the estop hook"
        );
    }

    #[test]
    fn unkeyed_estop_episodes_never_reach_the_hook() {
        // Guards two properties at once: a LATER episode must halt (the original
        // sequence-0 regression), and no unauthenticated cache upstream of the
        // MAC may drop a system-topic message.
        // Regression guard for a trap introduced BY the framing fix. Framed
        // system messages take the normal data path, which runs
        // reliability.dedup_messages BEFORE the system-topic dispatch.
        // is_new_message drops anything whose sequence is <= the last seen for
        // that (sender, topic) — so a fixed sequence of 0 would deliver the FIRST
        // e-stop episode and silently drop every later one, forever.
        //
        // Retries reuse the already-framed bytes and SHOULD still dedup; a new
        // episode carries a new sequence and must get through.
        let fired = Arc::new(AtomicBool::new(false));
        let fired2 = fired.clone();
        horus_core::scheduling::set_emergency_stop_hook(move |_reason| {
            fired2.store(true, Ordering::SeqCst);
        });

        let mut rep = test_replicator();
        let from: SocketAddr = "127.0.0.1:9100".parse().unwrap();

        // Frame two independent episodes exactly as handle_timer does.
        let ep1 = rep
            .frame_system_topic(
                crate::registry::SYSTEM_TOPIC_ESTOP,
                &crate::estop::encode_estop(0xABCD, "episode one"),
            )
            .expect("episode 1 must frame");
        let ep2 = rep
            .frame_system_topic(
                crate::registry::SYSTEM_TOPIC_ESTOP,
                &crate::estop::encode_estop(0xABCD, "episode two"),
            )
            .expect("episode 2 must frame");

        // The two episodes must not carry the same wire sequence.
        let seq1 = wire::MessageHeader::decode(&ep1[PacketHeader::SIZE..])
            .unwrap()
            .sequence;
        let seq2 = wire::MessageHeader::decode(&ep2[PacketHeader::SIZE..])
            .unwrap()
            .sequence;
        assert_ne!(
            seq1, seq2,
            "each e-stop episode needs its own sequence or dedup eats it"
        );

        rep.process_packet(&ep1, from);
        assert!(
            !fired.load(Ordering::SeqCst),
            "unkeyed episode must not halt"
        );

        // A byte-identical retry must remain inert without key provisioning.
        fired.store(false, Ordering::SeqCst);
        rep.process_packet(&ep1, from);
        assert!(
            !fired.load(Ordering::SeqCst),
            "an unkeyed retry must remain inert"
        );

        // A genuinely new but unauthenticated episode must also remain inert.
        fired.store(false, Ordering::SeqCst);
        rep.process_packet(&ep2, from);
        assert!(
            !fired.load(Ordering::SeqCst),
            "a later unkeyed e-stop episode must not halt"
        );
    }

    // ─── Export sampling: what leaves this machine, and what is admitted lost ──
    //
    // These drive the private export path against a real SHM ring and a real
    // UDP socket standing in for the remote subscriber, because the bug they
    // pin was invisible at every layer above: the exporter took one slot per
    // 50 ms tick and dropped the rest with no counter, so a 500 Hz odometry
    // stream reached the fleet at 20 Hz and looked from there exactly like a
    // 20 Hz odometry stream.

    /// A replicator that exports `topic` to a peer listening on `sub_addr`.
    fn exporting_replicator(topic: &str, sub_addr: SocketAddr, stream: bool) -> Replicator {
        let type_hash = wire::topic_hash(topic);
        let registry = Arc::new(TopicRegistry::new());
        registry.register(
            topic,
            type_hash,
            std::mem::size_of::<horus_robotics::CmdVel>() as u32,
            crate::registry::TopicRole::Publisher,
            true,
        );

        let mut config = NetConfig::test_config(0);
        if stream {
            config.export_stream = vec![topic.to_string()];
        }
        let mut rep = Replicator::new(registry, config).unwrap();

        // A peer that subscribes to the topic, announced from the socket the
        // test is listening on.
        let announcement = crate::discovery::PeerAnnouncement {
            peer_id: [0x5A; 16],
            data_port: sub_addr.port(),
            secret_hash: [0u8; 4],
            has_secret: false,
            topics: vec![crate::discovery::WireTopicEntry {
                name: topic.to_string(),
                type_hash,
                type_size: std::mem::size_of::<horus_robotics::CmdVel>() as u32,
                role: 2, // subscriber
                is_pod: 1,
                priority: 0,
            }],
            source_addr: sub_addr,
        };
        rep.peers.update_peer(&announcement);
        rep.update_matches();
        assert!(
            rep.readers.contains_key(topic),
            "the topic must be matched for export before the test means anything"
        );
        rep
    }

    fn publish_shm(path: &std::path::Path, linear: f32) {
        let cmd = horus_robotics::CmdVel::new(linear, 0.0);
        // SAFETY: CmdVel is POD; this is the byte form the ring holds.
        let bytes: &[u8] = unsafe {
            std::slice::from_raw_parts(
                &cmd as *const horus_robotics::CmdVel as *const u8,
                std::mem::size_of::<horus_robotics::CmdVel>(),
            )
        };
        assert!(horus_core::communication::write_topic_slot_bytes(
            path, bytes
        ));
    }

    /// Drain every datagram already queued on `sock` and return the CmdVel
    /// `linear` field of each message they carry.
    fn drain_exported(sock: &std::net::UdpSocket) -> Vec<f32> {
        let mut out = Vec::new();
        let mut buf = [0u8; 65536];
        while let Ok((n, _)) = sock.recv_from(&mut buf) {
            let (_, messages) = match wire::decode_packet(&buf[..n]) {
                Some(p) => p,
                None => continue,
            };
            for m in messages {
                assert_eq!(
                    m.payload.len(),
                    std::mem::size_of::<horus_robotics::CmdVel>()
                );
                // SAFETY: the payload is the CmdVel bytes written into SHM.
                let cmd: horus_robotics::CmdVel =
                    unsafe { std::ptr::read_unaligned(m.payload.as_ptr() as *const _) };
                out.push(cmd.linear);
            }
        }
        out
    }

    fn export_test_topic(base: &str) -> String {
        use std::sync::atomic::AtomicU32;
        static COUNTER: AtomicU32 = AtomicU32::new(0);
        let id = COUNTER.fetch_add(1, Ordering::Relaxed);
        format!("exp_{base}_{id}_{}", std::process::id())
    }

    #[test]
    fn a_burst_reaches_the_subscriber_intact_when_the_topic_is_a_stream() {
        let sub = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        sub.set_read_timeout(Some(Duration::from_millis(150)))
            .unwrap();
        let sub_addr = sub.local_addr().unwrap();

        let name = export_test_topic("burst");
        let _topic: horus_core::communication::Topic<horus_robotics::CmdVel> =
            horus_core::communication::Topic::new(&name).expect("create topic");
        let path = horus_sys::shm::topic_shm_path(&name);

        let mut rep = exporting_replicator(&name, sub_addr, true);

        // First tick starts the reader at "now".
        publish_shm(&path, 0.0);
        rep.handle_export();
        assert_eq!(drain_exported(&sub), vec![0.0]);

        // 50 messages published between two export ticks — a publisher running
        // 25x faster than the timer.
        const BURST: usize = 50;
        for i in 1..=BURST {
            publish_shm(&path, i as f32);
        }
        rep.handle_export();

        let crossed = drain_exported(&sub);
        let expected: Vec<f32> = (1..=BURST).map(|i| i as f32).collect();
        assert_eq!(
            crossed, expected,
            "every message published between two ticks must reach the peer, in order"
        );

        let snap = rep.metrics.snapshot();
        let topic = snap
            .topics
            .iter()
            .find(|t| t.topic_hash == wire::topic_hash(&name))
            .expect("the topic must appear in the metrics");
        assert_eq!(topic.messages_sent, BURST as u64 + 1);
        assert_eq!(topic.messages_skipped, 0);
    }

    #[test]
    fn the_default_still_samples_but_no_longer_hides_what_it_dropped() {
        let sub = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        sub.set_read_timeout(Some(Duration::from_millis(150)))
            .unwrap();
        let sub_addr = sub.local_addr().unwrap();

        let name = export_test_topic("sampled");
        let _topic: horus_core::communication::Topic<horus_robotics::CmdVel> =
            horus_core::communication::Topic::new(&name).expect("create topic");
        let path = horus_sys::shm::topic_shm_path(&name);

        let mut rep = exporting_replicator(&name, sub_addr, false);

        publish_shm(&path, 0.0);
        rep.handle_export();
        assert_eq!(drain_exported(&sub), vec![0.0]);

        // One tick of a 500 Hz publisher.
        for i in 1..=25 {
            publish_shm(&path, i as f32);
        }
        rep.handle_export();

        assert_eq!(
            drain_exported(&sub),
            vec![25.0],
            "the default is unchanged: the freshest sample, once per tick"
        );

        let snap = rep.metrics.snapshot();
        let topic = snap
            .topics
            .iter()
            .find(|t| t.topic_hash == wire::topic_hash(&name))
            .expect("the topic must appear in the metrics");
        assert_eq!(topic.messages_sent, 2);
        assert_eq!(
            topic.messages_skipped, 24,
            "the 24 samples that never left this machine have to be visible \
             somewhere, or a 25x downsample is indistinguishable from a slow \
             publisher"
        );
    }
}
