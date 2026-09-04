//! Transparent LAN replication — zero config, same `Topic<T>` API.
//!
//! Network replication starts **automatically** when `scheduler.run()` is called.
//! No manual wiring needed:
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! let mut scheduler = Scheduler::new()
//!     .tick_rate(100_u64.hz());
//! // ... add nodes ...
//! scheduler.run()?;
//! // Network replication starts automatically and stops on shutdown
//! ```
//!
//! ## Opting out
//!
//! Disable networking via `.network(false)` or `HORUS_NET_ENABLED=false` env var:
//!
//! ```rust,ignore
//! let mut scheduler = Scheduler::new()
//!     .network(false)              // <-- disables LAN replication
//!     .tick_rate(100_u64.hz());
//! ```
//!
//! ## Custom config
//!
//! For advanced networking configuration, use [`wire_with_config()`] to override
//! the automatic default:
//!
//! ```rust,ignore
//! use horus::net::NetConfig;
//!
//! let config = NetConfig { port: 9200, ..NetConfig::default() };
//! horus::net::wire_with_config(&mut scheduler, config);
//! scheduler.run()?;
//! ```
//!
//! ## Manual control
//!
//! Call [`enable()`] before `scheduler.run()` and hold the returned handle:
//!
//! ```rust,ignore
//! let _net = horus::net::enable();
//! scheduler.run()?;
//! ```

/// Enable network replication with default settings.
///
/// Returns a handle that stops the replicator on drop.
/// Returns `None` if networking is disabled (`HORUS_NO_NETWORK=1`).
pub fn enable() -> Option<horus_net::ReplicatorHandle> {
    horus_net::start_replicator_default()
}

/// Enable network replication with custom config.
pub fn enable_with_config(
    config: horus_net::config::NetConfig,
) -> Option<horus_net::ReplicatorHandle> {
    horus_net::start_replicator(config)
}

/// Wire `horus_net` into the scheduler's lifecycle (manual override).
///
/// **Note**: As of HORUS 0.2, networking is wired automatically when
/// `scheduler.run()` is called — you do NOT need to call this function
/// for default configuration. It remains available for backward
/// compatibility and as an explicit opt-in if you want to be explicit.
///
/// If networking was explicitly disabled (`.network(false)` or
/// `HORUS_NET_ENABLED=false`), this is a no-op.
///
/// # Example
/// ```rust,ignore
/// use horus::prelude::*;
///
/// let mut scheduler = Scheduler::new()
///     .tick_rate(100_u64.hz());
/// scheduler.add(my_node).build()?;
///
/// // Optional: explicit wire (happens automatically if omitted)
/// horus::net::wire(&mut scheduler);
/// scheduler.run()?;
/// ```
pub fn wire(scheduler: &mut horus_core::Scheduler) {
    if !scheduler.network_enabled() {
        return;
    }

    scheduler.on_start(|| match horus_net::start_replicator_default() {
        Some(handle) => {
            eprintln!("[horus_net] Network replication started");
            Some(Box::new(handle))
        }
        None => None,
    });
}

/// Wire `horus_net` with custom configuration into the scheduler's lifecycle.
///
/// Same as [`wire()`] but allows passing a custom [`NetConfig`].
/// When called, the custom config takes precedence over the automatic
/// default wiring.
pub fn wire_with_config(
    scheduler: &mut horus_core::Scheduler,
    config: horus_net::config::NetConfig,
) {
    if !scheduler.network_enabled() {
        return;
    }

    scheduler.on_start(move || match horus_net::start_replicator(config) {
        Some(handle) => {
            eprintln!("[horus_net] Network replication started (custom config)");
            Some(Box::new(handle))
        }
        None => None,
    });
}

pub use horus_net::config::NetConfig;
