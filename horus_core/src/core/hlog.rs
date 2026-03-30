//! Thread-local node logging context for HORUS nodes.
//!
//! This module provides the `hlog!()` macro which allows nodes to log
//! without needing to pass around a `NodeInfo` context. The scheduler
//! sets the current node context before each lifecycle call (init, tick, shutdown).
//!
//! # Example
//!
//! ```ignore
//! use horus::hlog;
//!
//! fn tick(&mut self) {
//!     hlog!(info, "Processing sensor data");
//!     if let Err(e) = self.process() {
//!         hlog!(error, "Processing failed: {}", e);
//!     }
//! }
//! ```

use std::cell::RefCell;
use std::time::Instant;

use crate::core::log_buffer::{publish_log, LogEntry, LogType};
use crate::terminal::is_raw_mode;

thread_local! {
    static CURRENT_NODE: RefCell<Option<NodeLogContext>> = const { RefCell::new(None) };
}

/// Thread-local context for node logging.
pub struct NodeLogContext {
    /// The node's name for log attribution.
    pub name: String,
    /// When the current tick started (for timing info).
    pub tick_start: Option<Instant>,
    /// Current tick number.
    pub tick_number: u64,
}

/// Set the current node context for this thread.
/// Called by the scheduler before invoking node lifecycle methods.
///
/// Reuses the existing allocation when possible — zero alloc in steady state.
pub fn set_node_context(name: &str, tick_number: u64) {
    CURRENT_NODE.with(|ctx| {
        let mut slot = ctx.borrow_mut();
        if let Some(ref mut existing) = *slot {
            // Reuse allocation: clear + push_str avoids realloc if capacity suffices
            existing.name.clear();
            existing.name.push_str(name);
            existing.tick_start = Some(Instant::now());
            existing.tick_number = tick_number;
        } else {
            *slot = Some(NodeLogContext {
                name: name.to_owned(),
                tick_start: Some(Instant::now()),
                tick_number,
            });
        }
    });
}

/// Clear the current node context for this thread.
/// Called by the scheduler after node lifecycle methods complete.
///
/// Keeps the allocation alive — next `set_node_context` reuses it.
pub fn clear_node_context() {
    CURRENT_NODE.with(|ctx| {
        if let Some(ref mut existing) = *ctx.borrow_mut() {
            existing.tick_start = None;
        }
    });
}

/// Get the current node name if set, otherwise "unknown".
pub fn current_node_name() -> String {
    CURRENT_NODE.with(|ctx| {
        ctx.borrow()
            .as_ref()
            .filter(|c| c.tick_start.is_some())
            .map(|c| c.name.clone())
            .unwrap_or_else(|| "unknown".to_string())
    })
}

/// Get the current tick number if set, otherwise 0.
pub fn current_tick_number() -> u64 {
    CURRENT_NODE.with(|ctx| ctx.borrow().as_ref().map(|c| c.tick_number).unwrap_or(0))
}

/// Internal function used by the hlog!() macro.
/// Logs a message with the current node context.
pub fn log_with_context(level: LogType, message: String) {
    let now = chrono::Local::now();

    let (node_name, tick_us, tick_number) = CURRENT_NODE.with(|ctx| {
        if let Some(ref c) = *ctx.borrow() {
            if let Some(tick_start) = c.tick_start {
                (
                    c.name.clone(),
                    tick_start.elapsed().as_micros() as u64,
                    c.tick_number,
                )
            } else {
                ("unknown".to_string(), 0, 0)
            }
        } else {
            ("unknown".to_string(), 0, 0)
        }
    });

    // Write to shared memory log buffer for monitor
    publish_log(LogEntry {
        timestamp: now.format("%H:%M:%S%.3f").to_string(),
        tick_number,
        node_name: node_name.clone(),
        log_type: level.clone(),
        topic: None,
        message: message.clone(),
        tick_us,
        ipc_ns: 0,
    });

    // Also emit to stderr/stdout for console visibility
    let line_ending = if is_raw_mode() { "\r\n" } else { "\n" };

    use std::io::{self, Write};

    match level {
        LogType::Info => {
            let msg = format!(
                "\x1b[34m[INFO]\x1b[0m \x1b[33m[{}]\x1b[0m {}{}",
                node_name, message, line_ending
            );
            let _ = io::stderr().write_all(msg.as_bytes());
            let _ = io::stderr().flush();
        }
        LogType::Warning => {
            let msg = format!(
                "\x1b[33m[WARN]\x1b[0m \x1b[33m[{}]\x1b[0m {}{}",
                node_name, message, line_ending
            );
            let _ = io::stderr().write_all(msg.as_bytes());
            let _ = io::stderr().flush();
        }
        LogType::Error => {
            let msg = format!(
                "\x1b[31m[ERROR]\x1b[0m \x1b[33m[{}]\x1b[0m {}{}",
                node_name, message, line_ending
            );
            let _ = io::stderr().write_all(msg.as_bytes());
            let _ = io::stderr().flush();
        }
        LogType::Debug => {
            let msg = format!(
                "\x1b[90m[DEBUG]\x1b[0m \x1b[33m[{}]\x1b[0m {}{}",
                node_name, message, line_ending
            );
            let _ = io::stderr().write_all(msg.as_bytes());
            let _ = io::stderr().flush();
        }
        _ => {
            // Other log types (Publish, Subscribe, etc.) - just to shared memory
        }
    }
}

/// Log a message from within a HORUS node.
///
/// This macro automatically captures the current node context (set by the scheduler)
/// and publishes logs to the shared memory buffer for the monitor to see.
///
/// # Syntax
///
/// ```ignore
/// hlog!(info, "Simple message");
/// hlog!(warn, "Warning with value: {}", value);
/// hlog!(error, "Error: {}", err);
/// hlog!(debug, "Debug info: {:?}", data);
/// ```
///
/// # Log Levels
///
/// - `info` - General informational messages
/// - `warn` - Warning conditions that should be noted
/// - `error` - Error conditions that need attention
/// - `debug` - Detailed debug information
///
/// # Example
///
/// ```ignore
/// use horus::hlog;
///
/// impl Node for MyNode {
///     fn name(&self) -> &str { "my_node" }
///
///     fn init(&mut self) -> HorusResult<()> {
///         hlog!(info, "Initializing with config: {:?}", self.config);
///         Ok(())
///     }
///
///     fn tick(&mut self) {
///         hlog!(debug, "Tick start");
///         if let Err(e) = self.process() {
///             hlog!(error, "Processing failed: {}", e);
///         }
///     }
///
///     fn shutdown(&mut self) -> HorusResult<()> {
///         hlog!(info, "Shutting down");
///         Ok(())
///     }
/// }
/// ```
#[macro_export]
macro_rules! hlog {
    (info, $($arg:tt)*) => {
        $crate::core::hlog::log_with_context($crate::core::LogType::Info, format!($($arg)*))
    };
    (warn, $($arg:tt)*) => {
        $crate::core::hlog::log_with_context($crate::core::LogType::Warning, format!($($arg)*))
    };
    (error, $($arg:tt)*) => {
        $crate::core::hlog::log_with_context($crate::core::LogType::Error, format!($($arg)*))
    };
    (debug, $($arg:tt)*) => {
        $crate::core::hlog::log_with_context($crate::core::LogType::Debug, format!($($arg)*))
    };
}

/// Log a [`HorusError`] with its remediation hint (if any).
///
/// Logs the error at ERROR level, then appends the `help()` hint on a
/// second line if one exists. Use this instead of `hlog!(error, "{}", e)`
/// when you want the user to see actionable remediation.
///
/// ```ignore
/// if let Err(e) = topic.send(&msg) {
///     horus_core::core::hlog::log_horus_error(&e);
/// }
/// ```
pub fn log_horus_error(err: &crate::error::HorusError) {
    let msg = if let Some(hint) = err.help() {
        format!("{}\n  hint: {}", err, hint)
    } else {
        format!("{}", err)
    };
    log_with_context(LogType::Error, msg);
}

/// Log a message at most once per `$interval_ms` milliseconds (throttled).
///
/// Equivalent to ROS2's `RCLCPP_INFO_THROTTLE` / `RCLCPP_WARN_THROTTLE`.
/// Uses a per-callsite atomic counter — zero overhead when the interval has
/// not elapsed.
///
/// # Syntax
///
/// ```ignore
/// hlog_every!(1000, warn, "Battery low: {}%", battery_pct);
/// //              ^^^^  ^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
/// //       interval_ms  level  format string
/// ```
///
/// # Example
///
/// ```ignore
/// fn tick(&mut self) {
///     // Log at most once per 5 seconds
///     hlog_every!(5000, info, "State: {:?}", self.state);
/// }
/// ```
#[macro_export]
macro_rules! hlog_every {
    ($interval_ms:expr, $level:ident, $($arg:tt)*) => {{
        use std::sync::atomic::{AtomicU64, Ordering};
        static LAST_LOG_MS: AtomicU64 = AtomicU64::new(0);
        let now_ms = ::std::time::SystemTime::now()
            .duration_since(::std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        let last = LAST_LOG_MS.load(Ordering::Relaxed);
        if now_ms.saturating_sub(last) >= ($interval_ms as u64) {
            LAST_LOG_MS.store(now_ms, Ordering::Relaxed);
            $crate::hlog!($level, $($arg)*);
        }
    }};
}

/// Log a message exactly once per program run (at the callsite).
///
/// Equivalent to ROS2's `RCLCPP_INFO_ONCE` / `RCLCPP_WARN_ONCE`.
/// Subsequent calls from the same source location are silently ignored.
///
/// # Syntax
///
/// ```ignore
/// hlog_once!(info, "Sensor calibration complete (model: {})", self.model);
/// ```
#[macro_export]
macro_rules! hlog_once {
    ($level:ident, $($arg:tt)*) => {{
        use std::sync::atomic::{AtomicBool, Ordering};
        static LOGGED: AtomicBool = AtomicBool::new(false);
        if !LOGGED.swap(true, Ordering::Relaxed) {
            $crate::hlog!($level, $($arg)*);
        }
    }};
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_and_clear_context() {
        // Initially no context
        assert_eq!(current_node_name(), "unknown");

        // Set context
        set_node_context("test_node", 42);
        assert_eq!(current_node_name(), "test_node");

        // Clear context
        clear_node_context();
        assert_eq!(current_node_name(), "unknown");
    }

    #[test]
    fn test_context_thread_isolation() {
        set_node_context("main_thread_node", 1);

        let handle = std::thread::spawn(|| {
            // Different thread should not see main thread's context
            assert_eq!(current_node_name(), "unknown");

            // Set its own context
            set_node_context("spawned_thread_node", 2);
            assert_eq!(current_node_name(), "spawned_thread_node");
        });

        handle.join().unwrap();

        // Main thread context should be unchanged
        assert_eq!(current_node_name(), "main_thread_node");

        clear_node_context();
    }

    // ── Throttling macro tests ──────────────────────────────────────────

    #[test]
    fn test_hlog_once_fires_exactly_once() {
        use crate::core::log_buffer::GLOBAL_LOG_BUFFER;

        let before = GLOBAL_LOG_BUFFER
            .get_all()
            .iter()
            .filter(|e| e.message.contains("once_unique_9901"))
            .count();

        for _ in 0..100 {
            crate::hlog_once!(info, "once_unique_9901");
        }

        let after = GLOBAL_LOG_BUFFER
            .get_all()
            .iter()
            .filter(|e| e.message.contains("once_unique_9901"))
            .count();

        assert_eq!(
            after - before,
            1,
            "hlog_once! should fire exactly 1 time in 100 calls. before={before}, after={after}"
        );
    }

    #[test]
    fn test_hlog_once_different_callsites_both_fire() {
        use crate::core::log_buffer::GLOBAL_LOG_BUFFER;

        crate::hlog_once!(info, "once_site_a_9902");
        crate::hlog_once!(info, "once_site_b_9903");

        let entries = GLOBAL_LOG_BUFFER.get_all();
        let has_a = entries
            .iter()
            .any(|e| e.message.contains("once_site_a_9902"));
        let has_b = entries
            .iter()
            .any(|e| e.message.contains("once_site_b_9903"));

        assert!(has_a, "first callsite should fire");
        assert!(has_b, "second callsite should fire independently");
    }

    #[test]
    fn test_hlog_every_first_call_fires_immediately() {
        use crate::core::log_buffer::GLOBAL_LOG_BUFFER;

        let before = GLOBAL_LOG_BUFFER
            .get_all()
            .iter()
            .filter(|e| e.message.contains("every_first_9904"))
            .count();

        // Interval of 60 seconds — but first call should fire immediately
        crate::hlog_every!(60000, info, "every_first_9904");

        let after = GLOBAL_LOG_BUFFER
            .get_all()
            .iter()
            .filter(|e| e.message.contains("every_first_9904"))
            .count();

        assert_eq!(
            after - before,
            1,
            "hlog_every! first call should fire immediately even with long interval"
        );
    }

    #[test]
    fn test_hlog_every_throttles_by_interval() {
        use crate::core::log_buffer::GLOBAL_LOG_BUFFER;

        let before = GLOBAL_LOG_BUFFER
            .get_all()
            .iter()
            .filter(|e| e.message.contains("every_throttle_9905"))
            .count();

        // Call in a tight loop for 500ms with 200ms interval
        let start = std::time::Instant::now();
        while start.elapsed() < std::time::Duration::from_millis(500) {
            crate::hlog_every!(200, info, "every_throttle_9905");
            std::thread::sleep(std::time::Duration::from_millis(5));
        }

        let after = GLOBAL_LOG_BUFFER
            .get_all()
            .iter()
            .filter(|e| e.message.contains("every_throttle_9905"))
            .count();

        let count = after - before;
        // At 200ms interval over 500ms: expect 2-4 entries (not 100+)
        assert!(
            count >= 2 && count <= 6,
            "hlog_every!(200ms) over 500ms should produce 2-6 entries, got {count}"
        );
    }
}
