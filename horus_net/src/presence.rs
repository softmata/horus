//! Presence replication — broadcast local node presence to remote peers.
//!
//! Every 1 second, the replicator serializes local presence data and broadcasts
//! it on the `_horus.presence` system topic. Remote machines receive this and
//! write synthetic presence files so `horus node list` shows remote nodes.
//!
//! Liveness: heartbeat-based (last_seen_ns timestamp), NOT PID-based.
//! Graduated staleness: reachable (0-3s) → unreachable (3-30s) → dead (>30s, removed).
//!
//! Encoding: simple length-prefixed strings (no serde dependency).

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Staleness thresholds.
const DEAD_TIMEOUT: Duration = Duration::from_secs(30);

/// Maximum node entries accepted from one presence broadcast.
///
/// `node_count` is a u16 off the wire and each entry costs only 11 bytes there
/// (2 length + 1 name byte + 8 rate), but expands to ~27 bytes of JSON — so the
/// decoder is a ~2.5x amplifier writing into `shm_nodes_dir()`, which on Linux
/// is **`/dev/shm`: the same tmpfs the topic rings live in**. A single 65507-byte
/// datagram yields ~5900 entries (~160 KB written); a *fragmented* one reaches
/// `fragment::MAX_REASSEMBLY_SIZE` (1 MB) and yields ~90 000 entries, ~2.4 MB per
/// file. At the presence token bucket's sustained 32/s, held for the 30s
/// `DEAD_TIMEOUT`, that is gigabytes of RAM-backed files — and exhausting that
/// tmpfs is not a disk-space problem, it is every subsequent `Topic::new` failing
/// and lazily-allocated tmpfs pages faulting for processes already mapped in.
///
/// No real host runs anywhere near this many nodes; a fleet member that does is
/// truncated in its presence listing rather than allowed to fill memory.
const MAX_NODES_PER_BROADCAST: usize = 256;

/// Maximum distinct remote hosts whose presence files this receiver maintains.
///
/// `host_id` is chosen by the sender and every distinct value creates a file.
/// Matches `netfilter::MAX_PEERS`, the crate's existing ceiling on distinct
/// remote hosts. Like `PeerTable`, this refuses NEW hosts at the cap rather than
/// evicting, so a flood of forged ids cannot displace the real fleet's entries.
const MAX_REMOTE_HOSTS: usize = crate::netfilter::MAX_PEERS;

/// Reject a wire-supplied name that could escape the presence directory when used
/// to build a filesystem path (`remote_{host_id}.json`).
///
/// Fail closed: accept only a single "normal" path component — no path separators,
/// no `..`, no absolute/root markers, no NUL. `host_id` arrives from an untrusted
/// UDP peer, so a value like `../../etc/cron.d/x` must never reach `Path::join`.
fn is_safe_path_component(name: &str) -> bool {
    use std::path::{Component, Path};
    if name.is_empty() || name.len() > 128 {
        return false;
    }
    if name.contains('/') || name.contains('\\') || name.contains('\0') || name.contains("..") {
        return false;
    }
    // `host_id` is interpolated as a JSON *value* as well as used as a filename,
    // and this check used to look only at path safety — a quote character passed
    // happily and let a remote peer author JSON structure in the presence file.
    if name.contains('"') || name.chars().any(|c| (c as u32) < 0x20) {
        return false;
    }
    let mut comps = Path::new(name).components();
    matches!(comps.next(), Some(Component::Normal(_))) && comps.next().is_none()
}

/// Escape a wire-supplied string for interpolation into the presence JSON.
///
/// `host_id` and node names arrive from an unauthenticated UDP peer and are
/// pasted into a JSON document with `format!`. Without escaping, a node named
/// `x","rate_hz":0},{"name":"fake` authors JSON *structure*, so a remote party
/// decides what `horus node list` reports about the fleet — and an unbalanced
/// quote makes the document unparseable, taking the host's presence out
/// entirely. The module encodes JSON by hand on purpose (no serde here), so the
/// escaper is the fix that fits: quote, backslash and every control character.
fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 8);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

/// Whether a wire-supplied node name is a plain identifier.
///
/// Node names had no validation at all: any length, any bytes. Bounding them
/// and restricting them to identifier characters keeps a hostile broadcast from
/// filling the presence file with megabytes of arbitrary text (and is a second
/// line of defence behind `json_escape`).
fn is_plain_node_name(name: &str) -> bool {
    !name.is_empty()
        // The bound is horus_core's, not a second copy of the number. It used
        // to be a bare 128 here against a bare 255 there, so a name in
        // 129..=255 was minted locally and dropped here without a word.
        && name.len() <= horus_core::core::presence::MAX_REPLICATED_NODE_NAME_LEN
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-' || c == '.')
}

/// Manages incoming remote presence data — writes presence files to disk.
pub struct PresenceReceiver {
    /// Active remote host files: host_id → (file_path, last_seen)
    active_hosts: HashMap<String, (PathBuf, Instant)>,
    /// Local namespace for filtering
    local_namespace: String,
    /// Whether the host-cap warning has already been emitted.
    host_cap_warned: bool,
}

impl Default for PresenceReceiver {
    fn default() -> Self {
        Self::new()
    }
}

impl PresenceReceiver {
    pub fn new() -> Self {
        Self {
            active_hosts: HashMap::new(),
            // The one accessor, not a second reading of the environment.
            //
            // `shm_namespace()` is what every other part of the process uses to
            // decide which `/dev/shm/horus_*` tree it belongs to, and it does
            // more than read `HORUS_NAMESPACE`: it sanitises the value and
            // derives one when the variable is unset. Reading the variable here
            // instead meant this filter could disagree with the directory the
            // very next line writes into — accepting a broadcast for a
            // namespace this process is not in, or rejecting one for the
            // namespace it is.
            local_namespace: horus_sys::shm::shm_namespace(),
            host_cap_warned: false,
        }
    }

    /// Process an incoming presence broadcast payload.
    /// Writes/updates a remote presence file in shm_nodes_dir().
    pub fn handle_broadcast(&mut self, data: &[u8]) {
        // Decode: [namespace_len:u16][namespace][host_id_len:u16][host_id][node_count:u16][nodes...]
        let mut pos = 0;
        if data.len() < 6 {
            return;
        }

        // Namespace
        let ns_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;
        if pos + ns_len > data.len() {
            return;
        }
        let namespace = String::from_utf8_lossy(&data[pos..pos + ns_len]).to_string();
        pos += ns_len;

        // Namespace isolation
        if namespace != self.local_namespace {
            return;
        }

        // Host ID
        if pos + 2 > data.len() {
            return;
        }
        let hid_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;
        if pos + hid_len > data.len() {
            return;
        }
        let host_id = String::from_utf8_lossy(&data[pos..pos + hid_len]).to_string();
        pos += hid_len;

        // Path-traversal guard: host_id becomes a filename (remote_{host_id}.json)
        // under shm_nodes_dir(). Reject separators / `..` / absolute components so an
        // untrusted peer cannot escape the directory. Fail closed — skip this entry.
        if !is_safe_path_component(&host_id) {
            horus_core::terminal::eprint_line(
                "[horus_net] Rejecting presence broadcast: unsafe host_id (path traversal)",
            );
            return;
        }

        // Timestamp
        if pos + 8 > data.len() {
            return;
        }
        let timestamp_ns = u64::from_le_bytes([
            data[pos],
            data[pos + 1],
            data[pos + 2],
            data[pos + 3],
            data[pos + 4],
            data[pos + 5],
            data[pos + 6],
            data[pos + 7],
        ]);
        pos += 8;

        // Refuse a NEW host once we are already tracking a full fleet's worth.
        //
        // `host_id` is sender-chosen and every distinct value creates a file
        // under `/dev/shm`. Refusing (rather than evicting) keeps a flood of
        // forged ids from displacing the real fleet, exactly as `PeerTable`
        // does. Known hosts keep updating, so a fleet at the cap still works.
        if !self.active_hosts.contains_key(&host_id) && self.active_hosts.len() >= MAX_REMOTE_HOSTS
        {
            if !self.host_cap_warned {
                self.host_cap_warned = true;
                horus_core::terminal::eprint_line(&format!(
                    "[horus_net] Tracking presence for {MAX_REMOTE_HOSTS} remote hosts; \
                     ignoring further ones. If this is not a real fleet of that size, a \
                     host is forging presence broadcasts — check HORUS_NET_ALLOW_PEERS. \
                     (Fires once.)"
                ));
            }
            return;
        }

        // Node count
        if pos + 2 > data.len() {
            return;
        }
        // Bounded before it drives the decode loop: see MAX_NODES_PER_BROADCAST.
        // The wire count is a u16 and each entry is as little as 11 bytes, so an
        // unbounded loop turns one reassembled 1 MB datagram into ~2.4 MB of JSON
        // in the tmpfs that backs every topic ring on this host.
        let node_count =
            (u16::from_le_bytes([data[pos], data[pos + 1]]) as usize).min(MAX_NODES_PER_BROADCAST);
        pos += 2;

        // Build JSON presence file content (discovery reads JSON)
        let mut nodes_json = Vec::with_capacity(node_count);
        for _ in 0..node_count {
            if pos + 2 > data.len() {
                break;
            }
            let name_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
            pos += 2;
            if pos + name_len > data.len() {
                break;
            }
            let name = String::from_utf8_lossy(&data[pos..pos + name_len]).to_string();
            pos += name_len;

            // Skip just this node, not the whole broadcast: one malformed entry
            // should not cost us the rest of a peer's presence.
            let name_ok = is_plain_node_name(&name);

            // Rate Hz (f64)
            if pos + 8 > data.len() {
                break;
            }
            let rate_bits = u64::from_le_bytes([
                data[pos],
                data[pos + 1],
                data[pos + 2],
                data[pos + 3],
                data[pos + 4],
                data[pos + 5],
                data[pos + 6],
                data[pos + 7],
            ]);
            let rate_hz = f64::from_bits(rate_bits);
            pos += 8;

            if !name_ok {
                continue;
            }

            nodes_json.push(format!(
                r#"{{"name":"{}","rate_hz":{}}}"#,
                json_escape(&name),
                if rate_hz.is_finite() {
                    format!("{:.1}", rate_hz)
                } else {
                    "null".to_string()
                }
            ));
        }

        let nodes_dir = horus_sys::shm::shm_nodes_dir();
        // Bare create_dir_all here left the nodes directory world-readable and
        // skipped the ownership gate — and this runs on the RECEIVE path, so a
        // remote peer's presence announcement was enough to create the tree
        // with the wrong mode before any local process hardened it.
        let _ = horus_sys::shm::create_shm_dir_all(&nodes_dir);

        let file_path = nodes_dir.join(format!("remote_{}.json", host_id));

        // Write JSON presence file
        let json = format!(
            r#"{{"host_id":"{}","is_remote":true,"last_seen_ns":{},"namespace":"{}","nodes":[{}]}}"#,
            json_escape(&host_id),
            timestamp_ns,
            json_escape(&namespace),
            nodes_json.join(",")
        );

        // Atomic write
        let tmp_path = file_path.with_extension("json.tmp");
        if std::fs::write(&tmp_path, json.as_bytes()).is_ok() {
            let _ = std::fs::rename(&tmp_path, &file_path);
        }

        self.active_hosts
            .insert(host_id, (file_path, Instant::now()));
    }

    /// Clean up stale remote presence files (>30s since last broadcast).
    pub fn cleanup_stale(&mut self) {
        let now = Instant::now();
        let mut to_remove = Vec::new();

        for (host_id, (path, last_seen)) in &self.active_hosts {
            if now.duration_since(*last_seen) > DEAD_TIMEOUT {
                let _ = std::fs::remove_file(path);
                to_remove.push(host_id.clone());
            }
        }

        for host_id in to_remove {
            self.active_hosts.remove(&host_id);
        }
    }

    /// Remove all remote presence files (on shutdown).
    pub fn cleanup_all(&mut self) {
        for (_host_id, (path, _)) in self.active_hosts.drain() {
            let _ = std::fs::remove_file(&path);
        }
    }
}

/// Build a presence broadcast payload from local node presence.
///
/// Wire format: `[namespace_len:u16][namespace][host_id_len:u16][host_id]`
///              `[timestamp_ns:u64][node_count:u16]`
///              `[name_len:u16][name][rate_hz:f64]` × node_count
///
/// Returns None if no local nodes exist.
pub fn build_local_presence(peer_id_hash: u16) -> Option<Vec<u8>> {
    // The same accessor the receiver filters on, and the same one that decides
    // which `/dev/shm/horus_*` tree this process belongs to. Reading the
    // environment directly is a second, subtly different answer — it skips the
    // sanitising and the derivation for an unset variable — so a broadcast
    // could announce a namespace this process is not actually in.
    let namespace = horus_sys::shm::shm_namespace();

    let presences = horus_core::NodePresence::read_all();
    if presences.is_empty() {
        return None;
    }

    let host_id = format!("{:04x}", peer_id_hash);

    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;

    let mut buf = Vec::with_capacity(512);

    // Namespace
    let ns_bytes = namespace.as_bytes();
    buf.extend_from_slice(&(ns_bytes.len() as u16).to_le_bytes());
    buf.extend_from_slice(ns_bytes);

    // Host ID
    let hid_bytes = host_id.as_bytes();
    buf.extend_from_slice(&(hid_bytes.len() as u16).to_le_bytes());
    buf.extend_from_slice(hid_bytes);

    // Timestamp
    buf.extend_from_slice(&now_ns.to_le_bytes());

    // Node count
    buf.extend_from_slice(&(presences.len() as u16).to_le_bytes());

    // Nodes
    for p in &presences {
        let name = p.name();
        let name_bytes = name.as_bytes();
        buf.extend_from_slice(&(name_bytes.len() as u16).to_le_bytes());
        buf.extend_from_slice(name_bytes);

        let rate_hz = p.rate_hz().unwrap_or(0.0);
        buf.extend_from_slice(&rate_hz.to_bits().to_le_bytes());
    }

    Some(buf)
}

#[cfg(test)]
mod namespace_agreement_tests {
    use super::*;

    /// The sender and the receiver must decide "which namespace am I in" the
    /// same way, and the same way as the directory they both use.
    ///
    /// Both read `HORUS_NAMESPACE` directly while the directory comes from
    /// `shm_namespace()`. Those are not the same answer: `shm_namespace`
    /// sanitises the variable and derives one when it is unset, which is the
    /// case for every test binary. A node could announce a namespace it was not
    /// in, or reject a peer that was in its own.
    #[test]
    fn both_sides_use_the_shared_accessor() {
        let expected = horus_sys::shm::shm_namespace();
        assert_eq!(
            PresenceReceiver::new().local_namespace,
            expected,
            "the receiver filters on a namespace other than this process's"
        );

        // The sender stamps the same value into the payload. Decode the
        // namespace field back out of a payload it built.
        if let Some(payload) = build_local_presence(0x1234) {
            let ns_len = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let announced = String::from_utf8_lossy(&payload[2..2 + ns_len]).to_string();
            assert_eq!(
                announced, expected,
                "the broadcast announces a namespace other than this process's"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn safe_component_accepts_normal_host_ids() {
        assert!(is_safe_path_component("a1b2"));
        assert!(is_safe_path_component("host_01"));
        assert!(is_safe_path_component("robot-42"));
    }

    #[test]
    fn safe_component_rejects_traversal_and_separators() {
        assert!(!is_safe_path_component(""));
        assert!(!is_safe_path_component(".."));
        assert!(!is_safe_path_component("../../etc/passwd"));
        assert!(!is_safe_path_component("a/b"));
        assert!(!is_safe_path_component("x/../../y"));
        assert!(!is_safe_path_component("/etc/passwd"));
        assert!(!is_safe_path_component("a\\b"));
        assert!(!is_safe_path_component("a\0b"));
        assert!(!is_safe_path_component("."));
    }

    /// Build a minimal presence wire payload with a chosen host_id and nodes.
    fn presence_payload_with_nodes(namespace: &str, host_id: &str, nodes: &[&str]) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&(namespace.len() as u16).to_le_bytes());
        buf.extend_from_slice(namespace.as_bytes());
        buf.extend_from_slice(&(host_id.len() as u16).to_le_bytes());
        buf.extend_from_slice(host_id.as_bytes());
        buf.extend_from_slice(&0u64.to_le_bytes()); // timestamp
        buf.extend_from_slice(&(nodes.len() as u16).to_le_bytes());
        for name in nodes {
            buf.extend_from_slice(&(name.len() as u16).to_le_bytes());
            buf.extend_from_slice(name.as_bytes());
            buf.extend_from_slice(&0.0f64.to_bits().to_le_bytes());
        }
        buf
    }

    /// Build a minimal presence wire payload with a chosen host_id and no nodes.
    fn presence_payload(namespace: &str, host_id: &str) -> Vec<u8> {
        presence_payload_with_nodes(namespace, host_id, &[])
    }

    #[test]
    fn json_escape_neutralises_quotes_and_controls() {
        assert_eq!(json_escape(r#"a"b"#), r#"a\"b"#);
        assert_eq!(json_escape("a\\b"), "a\\\\b");
        assert_eq!(json_escape("a\nb"), "a\\nb");
        assert_eq!(json_escape("a\u{1}b"), "a\\u0001b");
        assert_eq!(json_escape("plain-name.1"), "plain-name.1");
    }

    #[test]
    fn safe_component_rejects_json_metacharacters() {
        // host_id is interpolated as a JSON value, not just used as a filename.
        assert!(!is_safe_path_component("a\"b"));
        assert!(!is_safe_path_component("a\nb"));
    }

    #[test]
    fn wire_node_names_cannot_inject_json_structure() {
        // A remote peer used to author JSON *structure* through a node name:
        // the name below closed the object and started a second, fabricated
        // node that `horus node list` reported as real.
        let mut recv = PresenceReceiver::new();
        let ns = recv.local_namespace.clone();
        let hostile = r#"x","is_bridged":true,"name":"fake"#;
        recv.handle_broadcast(&presence_payload_with_nodes(
            &ns,
            "injtest",
            &[hostile, "real_node"],
        ));

        let path = horus_sys::shm::shm_nodes_dir().join("remote_injtest.json");
        let content = std::fs::read_to_string(&path).expect("presence file must be written");
        let _ = std::fs::remove_file(&path);

        assert!(
            !content.contains("is_bridged"),
            "wire node name authored JSON structure: {content}"
        );
        assert!(
            content.contains("real_node"),
            "a well-formed node in the same broadcast must survive: {content}"
        );
        // The document must still be one flat object with one nodes array.
        assert_eq!(content.matches("\"nodes\"").count(), 1);
    }

    #[test]
    fn a_broadcast_cannot_write_an_unbounded_presence_file() {
        // `node_count` is a u16 and an entry costs 11 bytes on the wire but ~27
        // in JSON, and the file lands in /dev/shm — the tmpfs every topic ring
        // on this host is allocated from. Unbounded, one reassembled datagram
        // wrote megabytes of RAM-backed JSON, repeatable at the presence token
        // bucket's sustained rate.
        let mut recv = PresenceReceiver::new();
        let ns = recv.local_namespace.clone();
        let names: Vec<String> = (0..(MAX_NODES_PER_BROADCAST * 3))
            .map(|i| format!("n{i}"))
            .collect();
        let refs: Vec<&str> = names.iter().map(|s| s.as_str()).collect();
        recv.handle_broadcast(&presence_payload_with_nodes(&ns, "floodtest", &refs));

        let path = horus_sys::shm::shm_nodes_dir().join("remote_floodtest.json");
        let content = std::fs::read_to_string(&path).expect("presence file must be written");
        let _ = std::fs::remove_file(&path);

        let written = content.matches("\"rate_hz\"").count();
        assert_eq!(
            written,
            MAX_NODES_PER_BROADCAST,
            "a broadcast claiming {} nodes must be truncated to the cap, not written whole",
            refs.len()
        );
    }

    #[test]
    fn a_forged_host_id_flood_cannot_grow_the_presence_file_set() {
        // Every distinct sender-chosen `host_id` creates a file in /dev/shm.
        let mut recv = PresenceReceiver::new();
        let ns = recv.local_namespace.clone();
        for i in 0..(MAX_REMOTE_HOSTS * 2) {
            let host = format!("flood{i}");
            recv.handle_broadcast(&presence_payload(&ns, &host));
            assert!(
                recv.active_hosts.len() <= MAX_REMOTE_HOSTS,
                "presence host set exceeded its cap at iteration {i}: {}",
                recv.active_hosts.len()
            );
        }
        assert_eq!(recv.active_hosts.len(), MAX_REMOTE_HOSTS);

        // A host already tracked must still be able to refresh at the cap —
        // otherwise a full table would freeze the real fleet's presence.
        let known = "flood0".to_string();
        assert!(recv.active_hosts.contains_key(&known));
        recv.handle_broadcast(&presence_payload(&ns, &known));
        assert!(recv.active_hosts.contains_key(&known));

        recv.cleanup_all();
    }

    #[test]
    fn handle_broadcast_skips_traversal_host_id() {
        let mut recv = PresenceReceiver::new();
        let ns = recv.local_namespace.clone();
        // A traversal host_id must be rejected before any path is built.
        recv.handle_broadcast(&presence_payload(&ns, "../pwned"));
        assert!(
            recv.active_hosts.is_empty(),
            "traversal host_id must not be tracked or written"
        );
    }
}
