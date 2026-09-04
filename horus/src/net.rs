//! Transparent LAN replication — zero config, same `Topic<T>` API.
//!
//! Network replication starts **automatically** when `scheduler.run()` is called,
//! provided you have not registered a startup hook of your own. No manual wiring
//! needed:
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
//! ## When auto-start does NOT happen
//!
//! The auto-wire runs only when the scheduler has **no** lifecycle start hooks
//! registered (`scheduler/mod.rs`: `network_enabled && lifecycle_start_hooks
//! .is_empty()`). `Scheduler::on_start()` pushes into that same list, so *any*
//! startup hook — including one that has nothing to do with networking —
//! suppresses it:
//!
//! ```rust,ignore
//! scheduler.on_start(|| { start_my_logger(); None });
//! scheduler.run()?;   // <-- no network replication, and no warning
//! ```
//!
//! That is deliberate for [`wire_with_config()`], which registers a hook
//! precisely so it can replace the default. It is a surprise for any other
//! hook. If you register one and still want networking, wire it explicitly:
//!
//! ```rust,ignore
//! scheduler.on_start(|| { start_my_logger(); None });
//! horus::net::wire(&mut scheduler);   // <-- put the default back
//! scheduler.run()?;
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
//! the automatic default. It registers a lifecycle hook, which is what
//! suppresses the auto-wire described above — replacing the default rather than
//! adding to it:
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
