//! Inbound source-address filtering and rate limiting.
//!
//! ## Why this exists
//!
//! `UdpTransport::bind` binds `0.0.0.0` (see `transport/udp.rs`), so the
//! replicator accepts datagrams from *any* host that can route to its port —
//! not merely a link-local neighbour, which is what the e-stop warning used to
//! claim. Combined with the system-topic dispatch in
//! `Replicator::process_incoming_message`, which fires **before** the import
//! guard, that made `_horus.estop`, `_horus.logs` and `_horus.presence`
//! reachable by an unauthenticated remote party (GHSA-3frr-c2j9-hhr7).
//!
//! Rather than reorder the guard — which would not help, because
//! `ImportExportGuard::allow_import` unconditionally returns `true` for system
//! topics — this module filters at the single choke point every datagram passes
//! through, so all five packet kinds and all three system topics are covered by
//! one check.
//!
//! ## Default posture
//!
//! With `HORUS_NET_ALLOW_PEERS` unset the filter admits **loopback, RFC 1918
//! private space, RFC 3927 link-local and RFC 6598 CGNAT** sources only. That
//! preserves the documented use case — zero-config replication between robots
//! on a LAN — while removing reachability from the public internet, which was
//! never intended and which the advisory's CVSS `AV:N` reflects.
//!
//! This is a reduction in reach, not authentication: a hostile host *on the same
//! LAN* still passes the filter. Authentication of the safety-critical channel
//! is `HORUS_ESTOP_KEY` (see [`crate::mac`]). The two are complementary and both
//! are needed.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

// ─── CIDR matching ──────────────────────────────────────────────────────────

/// An IPv4 CIDR block. `prefix_len` is 0..=32.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cidr {
    base: u32,
    prefix_len: u8,
}

impl Cidr {
    /// Build a CIDR, masking off host bits so `10.1.2.3/8` behaves as `10.0.0.0/8`.
    pub fn new(addr: Ipv4Addr, prefix_len: u8) -> Option<Self> {
        if prefix_len > 32 {
            return None;
        }
        let mask = Self::mask(prefix_len);
        Some(Self {
            base: u32::from(addr) & mask,
            prefix_len,
        })
    }

    /// Netmask for a prefix length. `/0` must not shift by 32 (UB-adjacent in C,
    /// a panic in debug Rust and a no-op shift in release), hence the branch.
    fn mask(prefix_len: u8) -> u32 {
        if prefix_len == 0 {
            0
        } else {
            u32::MAX << (32 - prefix_len)
        }
    }

    /// Parse `a.b.c.d` (implicit /32) or `a.b.c.d/len`.
    pub fn parse(s: &str) -> Option<Self> {
        let s = s.trim();
        let (addr_part, prefix_part) = match s.split_once('/') {
            Some((a, p)) => (a, Some(p)),
            None => (s, None),
        };
        let addr: Ipv4Addr = addr_part.trim().parse().ok()?;
        let prefix_len = match prefix_part {
            Some(p) => p.trim().parse::<u8>().ok()?,
            None => 32,
        };
        Self::new(addr, prefix_len)
    }

    /// True if `addr` falls inside this block.
    pub fn contains(&self, addr: Ipv4Addr) -> bool {
        let mask = Self::mask(self.prefix_len);
        (u32::from(addr) & mask) == self.base
    }
}

/// The default-allow set: loopback, private, link-local, CGNAT.
fn default_allow() -> Vec<Cidr> {
    [
        "127.0.0.0/8",    // loopback
        "10.0.0.0/8",     // RFC 1918
        "172.16.0.0/12",  // RFC 1918
        "192.168.0.0/16", // RFC 1918
        "169.254.0.0/16", // RFC 3927 link-local
        "100.64.0.0/10",  // RFC 6598 CGNAT / typical mesh-VPN space
    ]
    .iter()
    .filter_map(|s| Cidr::parse(s))
    .collect()
}

// ─── Peer filter ────────────────────────────────────────────────────────────

/// Source-address admission control for inbound datagrams.
#[derive(Debug, Clone)]
pub enum PeerFilter {
    /// Accept every source. Only reachable by explicitly setting
    /// `HORUS_NET_ALLOW_PEERS=any` — never the default.
    Any,
    /// Accept only sources inside one of these blocks.
    Allow(Vec<Cidr>),
}

impl Default for PeerFilter {
    fn default() -> Self {
        Self::Allow(default_allow())
    }
}

impl PeerFilter {
    /// Build from `HORUS_NET_ALLOW_PEERS`.
    ///
    /// * unset / empty → [`PeerFilter::default`] (private space only)
    /// * `any` / `*` / `0.0.0.0/0` → [`PeerFilter::Any`]
    /// * otherwise a comma-separated list of addresses and CIDR blocks
    ///
    /// Unparseable entries are reported and skipped rather than silently
    /// dropped. If *every* entry is unparseable the default is used, because
    /// falling back to `Any` on a typo would quietly undo the hardening.
    pub fn from_env() -> Self {
        let Ok(raw) = std::env::var("HORUS_NET_ALLOW_PEERS") else {
            return Self::default();
        };
        Self::parse(&raw)
    }

    /// Parse a filter spec. Exposed for tests and for `horus.toml` wiring.
    pub fn parse(raw: &str) -> Self {
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            return Self::default();
        }
        if matches!(
            trimmed.to_lowercase().as_str(),
            "any" | "*" | "all" | "0.0.0.0/0"
        ) {
            eprintln!(
                "[horus_net] WARNING: HORUS_NET_ALLOW_PEERS=any — the replicator will \
                 accept UDP from ANY routable host, including the public internet. \
                 Set HORUS_ESTOP_KEY so the safety channel is still authenticated."
            );
            return Self::Any;
        }

        let mut cidrs = Vec::new();
        for entry in trimmed.split(',') {
            let entry = entry.trim();
            if entry.is_empty() {
                continue;
            }
            match Cidr::parse(entry) {
                Some(c) => cidrs.push(c),
                None => eprintln!(
                    "[horus_net] WARNING: ignoring unparseable HORUS_NET_ALLOW_PEERS \
                     entry {entry:?} (expected `a.b.c.d` or `a.b.c.d/len`)"
                ),
            }
        }
        if cidrs.is_empty() {
            eprintln!(
                "[horus_net] WARNING: HORUS_NET_ALLOW_PEERS had no usable entries; \
                 falling back to the default private-space-only filter."
            );
            return Self::default();
        }
        Self::Allow(cidrs)
    }

    /// True if a datagram from `addr` should be processed at all.
    pub fn admits(&self, addr: SocketAddr) -> bool {
        match self {
            Self::Any => true,
            Self::Allow(cidrs) => match addr.ip() {
                IpAddr::V4(v4) => cidrs.iter().any(|c| c.contains(v4)),
                // The socket binds 0.0.0.0, so a V6 source should be unreachable.
                // If one arrives it is either an IPv4-mapped address (unwrap and
                // match normally) or something unexpected, which we deny.
                IpAddr::V6(v6) => match v6.to_ipv4_mapped() {
                    Some(v4) => cidrs.iter().any(|c| c.contains(v4)),
                    None => false,
                },
            },
        }
    }
}

// ─── Token-bucket rate limiting ─────────────────────────────────────────────

/// A simple token bucket: `capacity` tokens, refilled at `refill_per_sec`.
///
/// Used to bound how much work an unauthenticated remote party can force on the
/// system-topic handlers. Without it, halting the fleet — and the logging each
/// halt emits — is free and repeatable at line rate.
#[derive(Debug)]
pub struct TokenBucket {
    capacity: f64,
    refill_per_sec: f64,
    tokens: f64,
    last: Instant,
}

impl TokenBucket {
    pub fn new(capacity: u32, refill_per_sec: u32) -> Self {
        Self::new_at(capacity, refill_per_sec, Instant::now())
    }

    /// Construct with an explicit start instant. Test seam: it lets a test pin
    /// `last` exactly, so refill assertions are not fighting the nanoseconds
    /// between reading the clock and constructing the bucket.
    pub fn new_at(capacity: u32, refill_per_sec: u32, now: Instant) -> Self {
        Self {
            capacity: capacity as f64,
            refill_per_sec: refill_per_sec as f64,
            tokens: capacity as f64,
            last: now,
        }
    }

    /// Take one token. Returns `false` when the bucket is empty (caller drops).
    pub fn take(&mut self) -> bool {
        self.refill(Instant::now());
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }

    /// Take one token using an explicit clock. Test seam.
    pub fn take_at(&mut self, now: Instant) -> bool {
        self.refill(now);
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }

    fn refill(&mut self, now: Instant) {
        let elapsed = now.saturating_duration_since(self.last);
        if elapsed == Duration::ZERO {
            return;
        }
        self.last = now;
        self.tokens =
            (self.tokens + elapsed.as_secs_f64() * self.refill_per_sec).min(self.capacity);
    }
}

/// Per-system-topic rate limits.
///
/// E-stop gets the most generous budget on purpose. A genuine e-stop is sent
/// triple-redundantly (multicast + unicast per peer + 3 retries), so a real
/// halt can legitimately arrive as a burst of tens of packets, and dropping a
/// real halt is worse than processing a forged one. Logs and presence are
/// purely informational and are held much tighter.
#[derive(Debug)]
pub struct SystemTopicLimits {
    pub estop: TokenBucket,
    pub logs: TokenBucket,
    pub presence: TokenBucket,
}

impl Default for SystemTopicLimits {
    fn default() -> Self {
        Self {
            estop: TokenBucket::new(128, 64),
            logs: TokenBucket::new(64, 32),
            presence: TokenBucket::new(64, 32),
        }
    }
}

// ─── Bounded attacker-keyed maps ────────────────────────────────────────────

/// Maximum entries in any map keyed on values an unauthenticated peer chooses.
///
/// `FlowController::state` and `ReliabilityLayer::dedup` are both keyed on
/// `(sender_id_hash, topic_hash)` — 16 bits of sender and 32 bits of topic, all
/// taken straight off the wire before any guard runs. Neither had a cap or an
/// eviction policy, so a peer that varies either field grows them without limit
/// until the process is OOM-killed.
///
/// A real deployment has tens of live (peer, topic) pairs; four thousand means
/// something is wrong.
pub const MAX_TRACKED_KEYS: usize = 4096;

/// Maximum peers retained in the peer table.
///
/// Announcements are unauthenticated and the peer id is attacker-chosen, so
/// without a cap one host can insert unboundedly many peers — each of which also
/// multiplies the O(peers x topics) rematch performed on every announcement.
pub const MAX_PEERS: usize = 256;

/// Enforce [`MAX_TRACKED_KEYS`] on a wire-keyed map, returning `true` if the
/// caller may insert a new key.
///
/// At the cap the map is cleared wholesale rather than evicted one entry at a
/// time. That is deliberate: tracking an LRU or insertion order costs memory and
/// time on the receive hot path, and the state being dropped is soft — flow
/// control loses its loss estimate and dedup loses its high-water sequence, both
/// of which rebuild within a few messages. Bounding the memory absolutely
/// matters more than preserving statistics under what is, by construction, an
/// attack or a gross misconfiguration.
pub fn enforce_key_cap<K, V>(map: &mut std::collections::HashMap<K, V>, warned: &mut bool) -> bool
where
    K: std::hash::Hash + Eq,
{
    if map.len() < MAX_TRACKED_KEYS {
        return true;
    }
    map.clear();
    if !*warned {
        *warned = true;
        eprintln!(
            "[horus_net] Per-peer tracking table hit its {MAX_TRACKED_KEYS}-entry cap and was \
             reset. This normally means a peer is varying its sender or topic hash — check \
             HORUS_NET_ALLOW_PEERS. (Fires once.)"
        );
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::SocketAddrV4;

    fn sa(s: &str) -> SocketAddr {
        SocketAddr::V4(SocketAddrV4::new(s.parse().unwrap(), 9100))
    }

    #[test]
    fn cidr_parses_bare_address_as_slash_32() {
        let c = Cidr::parse("192.168.1.5").unwrap();
        assert!(c.contains("192.168.1.5".parse().unwrap()));
        assert!(!c.contains("192.168.1.6".parse().unwrap()));
    }

    #[test]
    fn cidr_masks_host_bits() {
        // 10.1.2.3/8 must behave exactly as 10.0.0.0/8.
        let c = Cidr::parse("10.1.2.3/8").unwrap();
        assert_eq!(c, Cidr::parse("10.0.0.0/8").unwrap());
        assert!(c.contains("10.255.255.255".parse().unwrap()));
        assert!(!c.contains("11.0.0.1".parse().unwrap()));
    }

    #[test]
    fn cidr_slash_zero_matches_everything_without_shift_overflow() {
        let c = Cidr::parse("0.0.0.0/0").unwrap();
        assert!(c.contains("8.8.8.8".parse().unwrap()));
        assert!(c.contains("127.0.0.1".parse().unwrap()));
    }

    #[test]
    fn cidr_rejects_garbage() {
        assert!(Cidr::parse("not-an-ip").is_none());
        assert!(Cidr::parse("10.0.0.0/33").is_none());
        assert!(Cidr::parse("10.0.0.0/abc").is_none());
        assert!(Cidr::parse("").is_none());
    }

    #[test]
    fn default_filter_admits_lan_and_denies_public() {
        let f = PeerFilter::default();
        assert!(f.admits(sa("127.0.0.1")));
        assert!(f.admits(sa("10.4.5.6")));
        assert!(f.admits(sa("172.16.0.1")));
        assert!(f.admits(sa("172.31.255.254")));
        assert!(f.admits(sa("192.168.1.100")));
        assert!(f.admits(sa("169.254.10.10")));
        assert!(f.admits(sa("100.64.0.1")));

        // The whole point: a routable public source is now dropped.
        assert!(!f.admits(sa("8.8.8.8")));
        assert!(!f.admits(sa("1.1.1.1")));
        assert!(!f.admits(sa("203.0.113.7")));
        // 172.32/12 is NOT private — the classic off-by-one on the RFC 1918 range.
        assert!(!f.admits(sa("172.32.0.1")));
    }

    #[test]
    fn explicit_any_admits_public() {
        let f = PeerFilter::parse("any");
        assert!(f.admits(sa("8.8.8.8")));
    }

    #[test]
    fn explicit_list_is_exclusive() {
        let f = PeerFilter::parse("192.168.5.0/24, 10.0.0.7");
        assert!(f.admits(sa("192.168.5.99")));
        assert!(f.admits(sa("10.0.0.7")));
        assert!(!f.admits(sa("10.0.0.8")));
        // An explicit list REPLACES the default set — loopback is not implied.
        assert!(!f.admits(sa("127.0.0.1")));
    }

    #[test]
    fn all_garbage_falls_back_to_default_not_to_any() {
        // A typo must never silently open the socket to the internet.
        let f = PeerFilter::parse("nonsense, alsononsense");
        assert!(!f.admits(sa("8.8.8.8")));
        assert!(f.admits(sa("192.168.0.1")));
    }

    #[test]
    fn empty_spec_falls_back_to_default() {
        let f = PeerFilter::parse("   ");
        assert!(!f.admits(sa("8.8.8.8")));
        assert!(f.admits(sa("10.0.0.1")));
    }

    #[test]
    fn ipv4_mapped_v6_source_is_matched_as_v4() {
        let f = PeerFilter::default();
        let mapped: IpAddr = "::ffff:10.0.0.1".parse().unwrap();
        assert!(f.admits(SocketAddr::new(mapped, 9100)));
        let mapped_public: IpAddr = "::ffff:8.8.8.8".parse().unwrap();
        assert!(!f.admits(SocketAddr::new(mapped_public, 9100)));
    }

    #[test]
    fn bucket_allows_burst_then_throttles() {
        let mut b = TokenBucket::new(4, 4);
        for _ in 0..4 {
            assert!(b.take());
        }
        assert!(!b.take(), "5th take must fail — bucket is empty");
    }

    #[test]
    fn bucket_refills_over_time() {
        let start = Instant::now();
        let mut b = TokenBucket::new_at(2, 10, start); // 10 tokens/sec
        assert!(b.take_at(start));
        assert!(b.take_at(start));
        assert!(!b.take_at(start));
        // 200ms later: 2 tokens accrued.
        let later = start + Duration::from_millis(200);
        assert!(b.take_at(later));
        assert!(b.take_at(later));
        assert!(!b.take_at(later));
    }

    #[test]
    fn bucket_does_not_accrue_beyond_capacity() {
        let start = Instant::now();
        let mut b = TokenBucket::new_at(3, 100, start);
        // An hour of idling must not bank an hour of tokens.
        let much_later = start + Duration::from_secs(3600);
        for _ in 0..3 {
            assert!(b.take_at(much_later));
        }
        assert!(!b.take_at(much_later));
    }

    #[test]
    fn estop_budget_is_more_generous_than_logs() {
        let l = SystemTopicLimits::default();
        assert!(l.estop.capacity > l.logs.capacity);
    }

    #[test]
    fn key_cap_bounds_a_wire_keyed_map() {
        use std::collections::HashMap;
        let mut m: HashMap<(u16, u32), u32> = HashMap::new();
        let mut warned = false;
        // Simulate a peer varying its topic hash on every packet.
        for i in 0..(MAX_TRACKED_KEYS as u32 * 3) {
            if enforce_key_cap(&mut m, &mut warned) {
                m.insert((0, i), i);
            }
            assert!(
                m.len() <= MAX_TRACKED_KEYS,
                "map must never exceed the cap, was {}",
                m.len()
            );
        }
        assert!(warned, "hitting the cap must warn at least once");
    }

    #[test]
    fn key_cap_is_transparent_below_the_limit() {
        use std::collections::HashMap;
        let mut m: HashMap<(u16, u32), u32> = HashMap::new();
        let mut warned = false;
        for i in 0..100 {
            assert!(enforce_key_cap(&mut m, &mut warned));
            m.insert((0, i), i);
        }
        assert_eq!(m.len(), 100, "normal traffic must not be disturbed");
        assert!(!warned, "no warning below the cap");
    }
}
