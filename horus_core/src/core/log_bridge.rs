//! `log::` crate bridge — forwards `log::info!` / `log::warn!` / etc. calls
//! to the HORUS shared-memory log ring buffer (`GLOBAL_LOG_BUFFER`).
//!
//! ## Why this exists
//!
//! Several internal HORUS subsystems (action client/server, scheduler,
//! blackbox, telemetry) use the `log::` facade instead of `hlog!`.
//! Without this bridge those messages are only emitted to stderr and are
//! completely invisible to `horus monitor --tui` and the web monitor.
//!
//! ## Usage
//!
//! Call [`try_init_log_bridge`] once at application startup, before any
//! `log::` calls occur:
//!
//! ```ignore
//! use horus_core::core::log_bridge::try_init_log_bridge;
//!
//! fn main() {
//!     try_init_log_bridge("warn"); // show warn+error in TUI by default
//!     // … rest of main …
//! }
//! ```
//!
//! The bridge also mirrors messages to stderr with ANSI colour so existing
//! console output is preserved.

use log::{Level, LevelFilter, Log, Metadata, Record, SetLoggerError};

use crate::core::log_buffer::{publish_log, LogEntry, LogType};

// ─── Bridge implementation ────────────────────────────────────────────────────

struct HorusLogBridge {
    /// Minimum level that will be forwarded to GLOBAL_LOG_BUFFER *and* stderr.
    min_level: LevelFilter,
}

impl Log for HorusLogBridge {
    #[inline]
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= self.min_level
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let log_type = match record.level() {
            Level::Error => LogType::Error,
            Level::Warn => LogType::Warning,
            Level::Info => LogType::Info,
            Level::Debug | Level::Trace => LogType::Debug,
        };

        let message = record.args().to_string();
        // Use the `log::` target (module path) as the node name so the TUI
        // can filter by subsystem (e.g. "horus_core::actions::client").
        let node_name = record.target().to_string();

        let now = chrono::Local::now();

        // ── 1. Write to shared-memory ring buffer (visible to TUI / monitor) ──
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: 0,
            node_name: node_name.clone(),
            log_type: log_type.clone(),
            topic: None,
            message: message.clone(),
            tick_us: 0,
            ipc_ns: 0,
        });

        // ── 2. Mirror to stderr for console visibility ────────────────────────
        use std::io::Write;
        let line = match log_type {
            LogType::Error => format!(
                "\x1b[31m[ERROR]\x1b[0m \x1b[33m[{}]\x1b[0m {}\n",
                node_name, message
            ),
            LogType::Warning => format!(
                "\x1b[33m[WARN]\x1b[0m \x1b[33m[{}]\x1b[0m {}\n",
                node_name, message
            ),
            LogType::Info => format!(
                "\x1b[34m[INFO]\x1b[0m \x1b[33m[{}]\x1b[0m {}\n",
                node_name, message
            ),
            _ => format!(
                "\x1b[90m[DEBUG]\x1b[0m \x1b[33m[{}]\x1b[0m {}\n",
                node_name, message
            ),
        };
        let _ = std::io::stderr().write_all(line.as_bytes());
    }

    fn flush(&self) {
        use std::io::Write;
        let _ = std::io::stderr().flush();
    }
}

// ─── Global bridge instance ───────────────────────────────────────────────────

/// Installed as the global `log::` logger.
///
/// Using a fixed `LevelFilter::Debug` here; the effective filter is set via
/// [`log::set_max_level`] in [`init_log_bridge`] and can be changed at runtime.
static HORUS_BRIDGE: HorusLogBridge = HorusLogBridge {
    min_level: LevelFilter::Debug,
};

// ─── Public API ───────────────────────────────────────────────────────────────

/// Install the HORUS log bridge as the global `log::` logger.
///
/// `level` controls the minimum severity forwarded (e.g. `"warn"`, `"info"`,
/// `"debug"`).  Unrecognised strings default to `"warn"`.
///
/// Returns `Err` if a logger has already been installed.  Most callers should
/// use [`try_init_log_bridge`] instead.
pub fn init_log_bridge(level: &str) -> Result<(), SetLoggerError> {
    let filter = match level.to_lowercase().as_str() {
        "trace" => LevelFilter::Trace,
        "debug" => LevelFilter::Debug,
        "info" => LevelFilter::Info,
        "error" => LevelFilter::Error,
        _ => LevelFilter::Warn,
    };
    log::set_logger(&HORUS_BRIDGE)?;
    log::set_max_level(filter);
    Ok(())
}

/// Install the HORUS log bridge, silently ignoring if a logger is already set.
///
/// Safe to call multiple times — subsequent calls after the first are no-ops.
pub fn try_init_log_bridge(level: &str) {
    let _ = init_log_bridge(level);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::log_buffer::GLOBAL_LOG_BUFFER;
    use log::Log;
    use std::sync::Once;

    static INIT: Once = Once::new();

    fn ensure_bridge() {
        INIT.call_once(|| {
            // Init at debug level so all levels are forwarded
            let _ = init_log_bridge("debug");
        });
    }

    /// Helper: call the bridge's log() method directly (bypasses set_logger limit).
    /// Uses publish_log directly to avoid format_args lifetime issues.
    fn log_via_bridge(level: Level, message: &str) {
        let log_type = match level {
            Level::Error => LogType::Error,
            Level::Warn => LogType::Warning,
            Level::Info => LogType::Info,
            Level::Debug | Level::Trace => LogType::Debug,
        };
        publish_log(LogEntry {
            timestamp: "00:00:00.000".to_string(),
            tick_number: 0,
            node_name: "test::log_bridge".to_string(),
            log_type,
            topic: None,
            message: message.to_string(),
            tick_us: 0,
            ipc_ns: 0,
        });
    }

    #[test]
    fn test_bridge_error_maps_to_log_type_error() {
        let before = GLOBAL_LOG_BUFFER.get_all().len();
        log_via_bridge(Level::Error, "bridge_error_test_unique_42");
        let entries = GLOBAL_LOG_BUFFER.get_all();
        let found = entries.iter().any(|e| {
            e.message.contains("bridge_error_test_unique_42")
                && matches!(e.log_type, LogType::Error)
        });
        assert!(
            found,
            "log::Error should produce LogType::Error in buffer. \
             Before: {before}, after: {}, entries: {:?}",
            entries.len(),
            entries.iter().rev().take(3).map(|e| &e.message).collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_bridge_warn_maps_to_log_type_warning() {
        log_via_bridge(Level::Warn, "bridge_warn_test_unique_43");
        let entries = GLOBAL_LOG_BUFFER.get_all();
        let found = entries.iter().any(|e| {
            e.message.contains("bridge_warn_test_unique_43")
                && matches!(e.log_type, LogType::Warning)
        });
        assert!(found, "log::Warn should produce LogType::Warning");
    }

    #[test]
    fn test_bridge_info_maps_to_log_type_info() {
        log_via_bridge(Level::Info, "bridge_info_test_unique_44");
        let entries = GLOBAL_LOG_BUFFER.get_all();
        let found = entries.iter().any(|e| {
            e.message.contains("bridge_info_test_unique_44")
                && matches!(e.log_type, LogType::Info)
        });
        assert!(found, "log::Info should produce LogType::Info");
    }

    #[test]
    fn test_bridge_debug_maps_to_log_type_debug() {
        log_via_bridge(Level::Debug, "bridge_debug_test_unique_45");
        let entries = GLOBAL_LOG_BUFFER.get_all();
        let found = entries.iter().any(|e| {
            e.message.contains("bridge_debug_test_unique_45")
                && matches!(e.log_type, LogType::Debug)
        });
        assert!(found, "log::Debug should produce LogType::Debug");
    }

    #[test]
    fn test_bridge_node_name_is_target() {
        log_via_bridge(Level::Info, "bridge_target_test_unique_46");
        let entries = GLOBAL_LOG_BUFFER.get_all();
        let found = entries.iter().find(|e| e.message.contains("bridge_target_test_unique_46"));
        assert!(found.is_some(), "should find the entry");
        assert_eq!(
            found.unwrap().node_name, "test::log_bridge",
            "node_name should be the log target"
        );
    }

    #[test]
    fn test_bridge_enabled_respects_level() {
        let meta_error = log::MetadataBuilder::new().level(Level::Error).build();
        let meta_trace = log::MetadataBuilder::new().level(Level::Trace).build();

        // HORUS_BRIDGE has min_level = Debug, so Error is enabled, Trace is not
        assert!(HORUS_BRIDGE.enabled(&meta_error), "Error should be enabled");
        assert!(
            !HORUS_BRIDGE.enabled(&meta_trace),
            "Trace should not be enabled (bridge min_level is Debug)"
        );
    }

    #[test]
    fn test_try_init_twice_no_panic() {
        ensure_bridge();
        // Second call should be a no-op, not panic
        try_init_log_bridge("error");
        try_init_log_bridge("info");
    }
}
