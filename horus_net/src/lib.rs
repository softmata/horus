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
pub mod metrics;
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
                reg_clone.register(
                    &name,
                    type_name_hash,
                    type_size,
                    crate::registry::TopicRole::Both, // Topic can be pub or sub
                    is_pod,
                );
            }
            TopicLifecycleEvent::Dropped { name } => {
                reg_clone.unregister(&name, crate::registry::TopicRole::Both);
            }
        }
    });

    let mut rep = match Replicator::new(registry, config) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("[horus_net] Failed to start replicator: {e}");
            return None;
        }
    };

    let running = rep.running_flag();

    let handle = std::thread::Builder::new()
        .name("horus-net".into())
        .spawn(move || {
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
                    eprintln!("[horus_net] Replicator thread did not stop within 3s, detaching");
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
