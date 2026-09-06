//! # horus_net — Transparent LAN Replication for HORUS
//!
//! Replicates SHM topic data between horus processes on different machines over UDP.
//! Same `Topic<T>` API. Zero config.
//!
//! ## Design Principle
//!
//! The user writes `Topic::<Imu>::new("imu")` and it works locally at 50ns.
//! It works across the LAN at ~50µs. Same code. Same API.
//!
//! The network layer activates automatically when remote horus peers exist on the LAN,
//! and costs nothing when they don't.
//!
//! ## Trust model — READ BEFORE DEPLOYING
//!
//! **Topic replication is unauthenticated.** There is no MAC, no handshake and no
//! source binding on the data path: a datagram that reaches the port and passes
//! the source-address filter is treated as coming from a legitimate peer. Any
//! host that can send UDP to this process — anything inside the allowed peer
//! ranges, which default to private/loopback/link-local — can therefore:
//!
//! - **Write arbitrary bytes into local SHM topics.** One forged discovery
//!   announcement claiming to publish `cmd_vel`, then one forged data packet, and
//!   the local subscriber reads the attacker's value as if a local publisher had
//!   sent it. This includes actuation commands.
//! - **Impersonate a peer.** `HORUS_NET_SECRET` is a cleartext FNV-1a value in
//!   every announcement, so it stops accidental cross-fleet mixing, not an
//!   attacker; `sender_id_hash` is 16 bits and attacker-chosen; the type-hash
//!   check compares against a hash the *sender* announced.
//! - **Replay any packet it captured**, including a captured `cmd_vel`.
//! - **Consume bounded resources**: peer table, heartbeat table, reassembly
//!   buffers and system-topic rates are all capped, so a flood degrades
//!   throughput rather than exhausting memory — but it is not kept out.
//!
//! What *is* authenticated: the e-stop channel only. `_horus.estop` packets carry
//! an HMAC keyed by `HORUS_ESTOP_KEY` (see [`mac`]) and are rejected outright
//! when no key is provisioned.
//!
//! Consequently: **run `horus_net` only on a network you trust as much as you
//! trust the robot.** On a shared or reachable network, disable it
//! (`HORUS_NO_NETWORK=1`), or set `HORUS_NET_IMPORT=deny` to refuse remote writes
//! into local SHM, and narrow `HORUS_NET_ALLOW_PEERS` to the exact peer
//! addresses. Authenticating the data plane (per-datagram MAC + replay window)
//! is a wire-format change and is not implemented.

pub mod config;
pub mod discovery;
pub mod encoding;
pub mod estop;
pub mod event_loop;
pub mod flow_control;
pub mod fragment;
pub mod guard;
pub mod heartbeat;
pub mod log_replication;
pub mod mac;
pub mod metrics;
pub mod netfilter;
pub mod optimize;
pub mod peer;
pub mod presence;
pub mod priority;
pub mod registry;
pub mod reliability;
pub mod replicator;
pub mod shm_reader;
pub mod shm_writer;
pub mod transport;
pub mod wire;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use config::NetConfig;
use registry::global_registry;
use replicator::Replicator;

/// Start the replicator thread. Called automatically by the Scheduler.
///
/// Returns `None` if networking is disabled (`HORUS_NO_NETWORK=1`).
/// The returned handle stops the replicator on drop (with 3s timeout).
pub fn start_replicator(config: NetConfig) -> Option<ReplicatorHandle> {
    if !config.enabled {
        return None;
    }

    let registry = global_registry();

    // Wire horus_core Topic lifecycle → horus_net TopicRegistry
    let reg_clone = registry.clone();
    horus_core::communication::set_topic_lifecycle_hook(move |event| {
        use horus_core::communication::TopicLifecycleEvent;
        match event {
            TopicLifecycleEvent::Created {
                name,
                type_name_hash,
                type_size,
                is_pod,
            } => {
                // Presence only. A `Topic<T>` handle can send *and* receive, so
                // at construction its direction is genuinely unknown; recording
                // `Both` here was what collapsed the import guard's
                // publisher/subscriber distinction. `Subscriber` is the enum's
                // own default and the safe reading of "open but unused": it
                // admits remote data for a topic nobody here writes, and the
                // first local send upgrades the entry to `Both`, which denies
                // it. The window between the two is one node tick, inside the
                // peer's one-second announce cycle.
                reg_clone.register(
                    &name,
                    type_name_hash,
                    type_size,
                    crate::registry::TopicRole::Subscriber,
                    is_pod,
                );
            }
            TopicLifecycleEvent::RoleObserved {
                name,
                publisher,
                type_name_hash,
                type_size,
                is_pod,
            } => {
                // First send or first recv on a handle — the earliest point the
                // direction is knowable. `register` merges, so a topic used both
                // ways ends up `Both` and stops importing, which is the
                // documented behaviour: a remote peer must not overwrite the
                // commands this robot produces itself.
                let role = if publisher {
                    crate::registry::TopicRole::Publisher
                } else {
                    crate::registry::TopicRole::Subscriber
                };
                reg_clone.register(&name, type_name_hash, type_size, role, is_pod);
            }
            TopicLifecycleEvent::Dropped { name } => {
                reg_clone.unregister(&name, crate::registry::TopicRole::Both);
            }
        }
    });

    let mut rep = match Replicator::new(registry, config) {
        Ok(r) => r,
        Err(e) => {
            horus_core::terminal::eprint_line(&format!(
                "[horus_net] Failed to start replicator: {e}"
            ));
            return None;
        }
    };

    let running = rep.running_flag();

    let handle = horus_core::scheduling::spawn_best_effort("horus-net", 0, move || {
        rep.run();
    })
    .ok()?;

    Some(ReplicatorHandle {
        running,
        thread: Some(handle),
    })
}

/// Start the replicator with default configuration (reads env vars).
///
/// Convenience for `start_replicator(NetConfig::default())`.
pub fn start_replicator_default() -> Option<ReplicatorHandle> {
    start_replicator(NetConfig::default())
}

/// Handle to the running replicator thread. Stops the replicator on drop.
pub struct ReplicatorHandle {
    running: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl ReplicatorHandle {
    /// Signal the replicator to stop (non-blocking).
    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }

    /// Check if the replicator is still running.
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }
}

impl Drop for ReplicatorHandle {
    fn drop(&mut self) {
        // Signal stop
        self.running.store(false, Ordering::Relaxed);

        // Join with 3s timeout
        if let Some(handle) = self.thread.take() {
            let deadline = std::time::Instant::now() + Duration::from_secs(3);
            loop {
                if handle.is_finished() {
                    let _ = handle.join();
                    break;
                }
                if std::time::Instant::now() >= deadline {
                    // Timeout — detach the thread (it will exit on next event loop cycle)
                    horus_core::terminal::eprint_line(
                        "[horus_net] Replicator thread did not stop within 3s, detaching",
                    );
                    break;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disabled_returns_none() {
        let mut config = NetConfig::test_config(0);
        config.enabled = false;
        assert!(start_replicator(config).is_none());
    }

    #[test]
    fn start_and_stop() {
        let config = NetConfig::test_config(0);
        let handle = start_replicator(config).unwrap();
        assert!(handle.is_running());

        handle.stop();
        std::thread::sleep(Duration::from_millis(100));
    }

    #[test]
    fn drop_stops_thread() {
        let config = NetConfig::test_config(0);
        {
            let _handle = start_replicator(config).unwrap();
        }
    }
}
