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
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Instant;

use crate::core::log_buffer::{publish_log, LogEntry, LogType};
use crate::terminal::is_raw_mode;

// ════════════════════════════════════════════════════════════════════════════
// Level filter
// ════════════════════════════════════════════════════════════════════════════
//
// `hlog!` had no level filter at all. Every arm evaluated `format!`
// unconditionally and then paid, in order: `chrono::Local::now()` and a
// formatted timestamp `String`, two more owned `String`s, a `bincode`
// serialisation behind the shared-memory log buffer's PROCESS-GLOBAL mutex, and
// an unbuffered `write(2)` + `flush` to stderr. There was no such thing as a
// disabled log line — a `hlog!(debug, ...)` in a 1 kHz `tick()` cost the full
// several microseconds whether or not anybody would ever read it.
//
// The filter has to short-circuit BEFORE argument evaluation, which is why it
// lives in the macro rather than at the top of `log_with_context`. A level check
// after `format!` has already run buys nothing: the allocation, the `Display`
// impls of every argument, and any side effect they have are the expensive part.
// Below the active level a call is now a relaxed atomic load, a compare and a
// not-taken branch — a couple of nanoseconds, and it never touches the log
// buffer's mutex at all.
//
// The default is [`DEFAULT_LEVEL`], and it is `debug` — nothing is suppressed
// until an operator sets `HORUS_LOG_LEVEL` or calls [`set_max_level`]. See that
// constant for why the conventional `info` default is not used here and what it
// would take to change it. The consequence, stated rather than implied: an
// unconfigured deployment gets the mechanism but not the saving.

/// Nothing is logged.
pub const LEVEL_OFF: u8 = 0;
/// Errors only.
pub const LEVEL_ERROR: u8 = 1;
/// Errors and warnings.
pub const LEVEL_WARN: u8 = 2;
/// Errors, warnings and informational messages. The level a production image
/// should set; see [`DEFAULT_LEVEL`] for why it is not the default.
pub const LEVEL_INFO: u8 = 3;
/// Everything, including debug.
pub const LEVEL_DEBUG: u8 = 4;

/// The level in force when nothing has set one.
///
/// **`Debug` — nothing is suppressed unless it is configured. Say plainly what
/// that costs:** the saving this filter exists for is only realised once a
/// level is actually set, so a deployment that never sets one still pays the
/// full per-line cost for `hlog!(debug, ...)` inside a tick. Production images
/// should set `HORUS_LOG_LEVEL=info` (or `warn`).
///
/// The conventional default would be `Info`, i.e. debug off. It is not the
/// default here because this crate has an existing, deliberate, *tested*
/// invariant that says otherwise — `hlog_ignores_log_bridge_level` in
/// `tests/log_system_tests.rs` asserts that a `Debug` entry reaches the shared
/// buffer regardless of any level, and documents the reason: "hlog!() is the
/// node-level logger and should never be silently dropped". Two more tests
/// (`python_hlog_all_levels_roundtrip`, `log_without_context_uses_unknown_node_name`)
/// depend on the same thing. Changing this constant to [`LEVEL_INFO`] is a
/// one-token change; it requires those assertions to be renegotiated first, and
/// that is a decision about the product's logging contract, not about latency.
pub const DEFAULT_LEVEL: u8 = LEVEL_DEBUG;

/// Environment override: `off`, `error`, `warn`, `info`, `debug`.
pub const LEVEL_ENV: &str = "HORUS_LOG_LEVEL";

/// Sentinel meaning "the environment has not been consulted yet".
const LEVEL_UNSET: u8 = u8::MAX;

// The filter is a single `level <= max_level` comparison, so the constants must
// be ordered least-verbose to most-verbose. Enforced at compile time rather than
// in a test: reordering them would silently invert the filter, and a wrong
// answer here means a log an operator is relying on stops appearing.
const _: () = assert!(LEVEL_OFF < LEVEL_ERROR);
const _: () = assert!(LEVEL_ERROR < LEVEL_WARN);
const _: () = assert!(LEVEL_WARN < LEVEL_INFO);
const _: () = assert!(LEVEL_INFO < LEVEL_DEBUG);
// `set_max_level` clamps to LEVEL_DEBUG, so a caller can never store the
// sentinel and turn every subsequent check into a re-read of the environment.
const _: () = assert!(LEVEL_DEBUG < LEVEL_UNSET);

static MAX_LEVEL: AtomicU8 = AtomicU8::new(LEVEL_UNSET);

/// Parse a level name. Returns `None` for anything unrecognised, so a typo
/// falls back to the default rather than silently disabling logging.
pub fn parse_level(name: &str) -> Option<u8> {
    match name.trim().to_ascii_lowercase().as_str() {
        "off" | "none" | "silent" => Some(LEVEL_OFF),
        "error" | "err" => Some(LEVEL_ERROR),
        "warn" | "warning" => Some(LEVEL_WARN),
        "info" => Some(LEVEL_INFO),
        "debug" | "trace" | "all" => Some(LEVEL_DEBUG),
        _ => None,
    }
}

/// Read `HORUS_LOG_LEVEL` and latch the result. Cold: runs at most once per
/// process on the first `hlog!`, and never again.
#[cold]
#[inline(never)]
fn init_level() -> u8 {
    let level = match std::env::var(LEVEL_ENV) {
        Ok(ref v) => parse_level(v).unwrap_or_else(|| {
            // An unrecognised value must not silently mean "default" — that is
            // how an operator ends up believing debug logging is on.
            crate::terminal::eprint_line(&format!(
                "[hlog] {LEVEL_ENV}='{v}' not recognised (expected off/error/warn/info/debug) \
                 — using the default"
            ));
            DEFAULT_LEVEL
        }),
        Err(_) => DEFAULT_LEVEL,
    };
    // Racing callers all compute the same value, so a plain store is fine.
    MAX_LEVEL.store(level, Ordering::Relaxed);
    level
}

/// The highest level currently being logged.
pub fn max_level() -> u8 {
    let current = MAX_LEVEL.load(Ordering::Relaxed);
    if current == LEVEL_UNSET {
        init_level()
    } else {
        current
    }
}

/// Set the highest level to log, overriding `HORUS_LOG_LEVEL`.
///
/// Takes effect immediately for every thread.
pub fn set_max_level(level: u8) {
    MAX_LEVEL.store(level.min(LEVEL_DEBUG), Ordering::Relaxed);
}

/// Whether `level` would be logged.
///
/// The guard the `hlog!` macros expand to, so it must stay cheap enough to sit
/// in a 1 kHz tick: in steady state one relaxed load, one compare, one branch.
#[inline]
pub fn level_enabled(level: u8) -> bool {
    let current = MAX_LEVEL.load(Ordering::Relaxed);
    if current != LEVEL_UNSET {
        return level <= current;
    }
    level <= init_level()
}

/// Maps the bare level word the macros take to its numeric constant.
///
/// Exported because `hlog_every!` and `hlog_once!` take the level as an
/// `ident`, and they expand in downstream crates. Not part of the public API.
#[doc(hidden)]
#[macro_export]
macro_rules! __hlog_level_const {
    (info) => {
        $crate::core::hlog::LEVEL_INFO
    };
    (warn) => {
        $crate::core::hlog::LEVEL_WARN
    };
    (error) => {
        $crate::core::hlog::LEVEL_ERROR
    };
    (debug) => {
        $crate::core::hlog::LEVEL_DEBUG
    };
    // Without this arm a misspelled level fails inside this helper with
    // "no rules expected `verbose`", pointing at a macro the author never
    // wrote — the same class of unhelpful error the `hlog!` failure arm below
    // exists to prevent.
    ($other:tt) => {
        compile_error!(concat!(
            "unknown hlog level `",
            stringify!($other),
            "` — the levels are: info, warn, error, debug"
        ))
    };
}

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

/// Whether this thread is currently inside a scheduler-managed tick.
///
/// Cheaper than [`current_node_name`], which allocates a `String` on every
/// call. `Topic::send` is on the real-time path and consults this on each send
/// until an owner is resolved, so the allocation would be a per-send cost on a
/// topic that has none.
#[inline]
pub fn in_node_context() -> bool {
    CURRENT_NODE.with(|ctx| {
        ctx.borrow()
            .as_ref()
            .is_some_and(|c| c.tick_start.is_some())
    })
}

/// Get the current tick number if set, otherwise 0.
pub fn current_tick_number() -> u64 {
    CURRENT_NODE.with(|ctx| ctx.borrow().as_ref().map(|c| c.tick_number).unwrap_or(0))
}

/// The filter level a [`LogType`] is subject to, or `None` for the transport
/// entries.
///
/// `Publish`/`Subscribe` exist for the monitor, never reach the console, and are
/// not something an operator sets a *log level* to control — so the user-facing
/// level filter deliberately does not gate them.
fn filter_level(log_type: &LogType) -> Option<u8> {
    match log_type {
        LogType::Error => Some(LEVEL_ERROR),
        LogType::Warning => Some(LEVEL_WARN),
        LogType::Info => Some(LEVEL_INFO),
        LogType::Debug => Some(LEVEL_DEBUG),
        LogType::Publish | LogType::Subscribe => None,
    }
}

/// Whether a [`LogType`] passes the active level filter.
#[inline]
pub fn log_type_enabled(log_type: &LogType) -> bool {
    filter_level(log_type).is_none_or(level_enabled)
}

/// Internal function used by the hlog!() macro.
/// Logs a message with the current node context.
///
/// The `hlog!` macros check the level before they build `message`, which is
/// where the saving is. The check is repeated here for the direct callers (the
/// C FFI, `log_horus_error`, anything holding a `String` already): they still
/// pay for the message they built, but not for the timestamp, the buffer mutex
/// or the console write.
/// The `hlog!` entry point.
///
/// Carries the message as `core::fmt::Arguments` rather than a `String`, which
/// is what lets the real-time path format it into a fixed-size ring slot
/// without ever reaching the allocator. The macro used to commit to a `String`
/// before anything could intervene: by the time RT-aware code could run, the
/// allocation had already happened — and on a `.no_alloc()` node that
/// allocation is itself the violation.
#[inline]
pub fn log_fmt(level: LogType, args: std::fmt::Arguments<'_>) {
    if !log_type_enabled(&level) {
        return;
    }
    if crate::core::hlog_rt::in_rt_thread() {
        // Reads the thread-local WITHOUT cloning the name: `queue` copies the
        // bytes straight out of the borrow into the slot.
        CURRENT_NODE.with(|ctx| {
            // The borrow is held across `queue` on purpose: that is what lets
            // the node name reach the ring as a `&str` the slot copies from,
            // instead of the `String` clone the old path made per line.
            let guard = ctx.borrow();
            let (name, tick_us, tick_number) = match *guard {
                Some(ref c) => (
                    c.name.as_str(),
                    c.tick_start
                        .map(|t| t.elapsed().as_micros() as u64)
                        .unwrap_or(0),
                    c.tick_number,
                ),
                None => ("unknown", 0, 0),
            };
            crate::core::hlog_rt::queue(level.clone(), name, tick_number, tick_us, args);
        });
        return;
    }
    log_with_context(level, fmt_to_string(args));
}

/// Render `Arguments` with one allocation.
///
/// `as_str()` returns `Some(&'static str)` for a literal with no interpolation,
/// which is the common `hlog!(info, "started")` shape — one `to_owned` instead
/// of running the whole formatting machine.
#[inline]
fn fmt_to_string(args: std::fmt::Arguments<'_>) -> String {
    match args.as_str() {
        Some(s) => s.to_owned(),
        None => args.to_string(),
    }
}

/// Emit an entry that was queued on an RT thread and drained off it.
///
/// Takes the node name and timing explicitly because the drain thread's own
/// `CURRENT_NODE` is empty — it never ticks a node — and attributing every
/// RT-queued line to "unknown" would defeat `horus log --node <name>`, which
/// filters on exactly that field.
pub(crate) fn emit_recorded(
    level: &LogType,
    node_name: &str,
    message: &str,
    tick_number: u64,
    tick_us: u64,
) {
    let now = chrono::Local::now();
    let console = format_console_line(level, node_name, message, cached_raw_mode());

    publish_log(LogEntry {
        timestamp: now.format("%H:%M:%S%.3f").to_string(),
        tick_number,
        node_name: node_name.to_string(),
        log_type: level.clone(),
        topic: None,
        message: message.to_string(),
        tick_us,
        ipc_ns: 0,
    });

    if let Some(line) = console {
        write_console_line(&line);
    }
}

/// Cached answer to "is the terminal in raw mode".
///
/// `is_raw_mode()` is an `isatty` plus a `tcgetattr`, and it ran on every
/// single log line. `u8::MAX` means not yet sampled.
static RAW_MODE: AtomicU8 = AtomicU8::new(u8::MAX);

/// Whether the terminal is in raw mode, from the cache.
#[inline]
pub(crate) fn cached_raw_mode() -> bool {
    match RAW_MODE.load(Ordering::Relaxed) {
        u8::MAX => {
            let raw = is_raw_mode();
            RAW_MODE.store(raw as u8, Ordering::Relaxed);
            raw
        }
        v => v != 0,
    }
}

/// Re-sample raw mode. Called once per drain poll, so a TUI entering raw mode
/// is picked up within one poll instead of costing two syscalls per line.
pub(crate) fn refresh_raw_mode() {
    RAW_MODE.store(is_raw_mode() as u8, Ordering::Relaxed);
}

pub fn log_with_context(level: LogType, message: String) {
    if !log_type_enabled(&level) {
        return;
    }

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

    // Render the console line BEFORE the entry takes ownership. This used to
    // `.clone()` both `node_name` and `message` — two heap allocations and two
    // copies on every log line, on a path that runs inside `tick()`.
    let console = format_console_line(&level, &node_name, &message, cached_raw_mode());

    // Write to shared memory log buffer for monitor
    publish_log(LogEntry {
        timestamp: now.format("%H:%M:%S%.3f").to_string(),
        tick_number,
        node_name,
        log_type: level,
        topic: None,
        message,
        tick_us,
        ipc_ns: 0,
    });

    // Console last: the shared-memory buffer is what survives a crash, and the
    // stderr write is the one that can block on a slow reader.
    if let Some(line) = console {
        write_console_line(&line);
    }
}

/// Log a message attributed to a named node, bypassing the thread-local.
///
/// `log_with_context` reads `CURRENT_NODE`, which is empty on an executor
/// thread and is also treated as empty once the tick's `tick_start` has been
/// cleared. Anything logged from those points came out as node "unknown" — and
/// `horus log --node <name>` filters on that field, so the entries most worth
/// filtering for (a node's own failures) were the ones it could not find.
///
/// Use this wherever the node is known at the call site.
pub fn log_as_node(level: LogType, node_name: &str, message: &str) {
    if !log_type_enabled(&level) {
        return;
    }

    let now = chrono::Local::now();
    let (tick_us, tick_number) = CURRENT_NODE.with(|ctx| match *ctx.borrow() {
        Some(ref c) => (
            c.tick_start
                .map(|t| t.elapsed().as_micros() as u64)
                .unwrap_or(0),
            c.tick_number,
        ),
        None => (0, 0),
    });

    let console = format_console_line(&level, node_name, message, cached_raw_mode());

    publish_log(LogEntry {
        timestamp: now.format("%H:%M:%S%.3f").to_string(),
        tick_number,
        node_name: node_name.to_string(),
        log_type: level,
        topic: None,
        message: message.to_string(),
        tick_us,
        ipc_ns: 0,
    });

    if let Some(line) = console {
        write_console_line(&line);
    }
}

/// Write one log line to the terminal.
///
/// Public, and takes the node name explicitly, because there are three callers
/// and only one of them can use the thread-local: the C FFI (`horus_log`) is
/// given a name by the C++ caller, and is routinely called outside a tick —
/// from a constructor, from `main`, from a helper thread — where the
/// thread-local is empty and the name would come out as "unknown".
///
/// It exists at all because that FFI wrote to the shared-memory buffer and
/// nowhere else, so a C++ node's own log statements were invisible in the
/// terminal they were running in. They were not being swallowed; nothing was
/// ever sending them. `horus log` showed them all along.
///
/// `Publish` and `Subscribe` entries are deliberately not printed — they exist
/// for the monitor and would drown a console at tick rate.
pub fn emit_console(level: &LogType, node_name: &str, message: &str) {
    if !log_type_enabled(level) {
        return;
    }
    // A TUI in raw mode has no implicit carriage return.
    let Some(line) = format_console_line(level, node_name, message, cached_raw_mode()) else {
        return;
    };
    write_console_line(&line);
}

/// Write one already-formatted line to stderr.
///
/// Split out from [`emit_console`] so the callers that have to build the line
/// early — to hand its inputs to the log buffer by value rather than by clone —
/// can still perform the write last.
///
/// NOTE, since this is a real cost and not a hypothetical: this write is
/// unbuffered and blocking. On a slow or stopped stderr reader it blocks for as
/// long as the reader takes, and a `hlog!` from inside a `tick()` inherits that
/// unbounded wait. The level filter above is what keeps the common case off
/// this path entirely; RT-thread diagnostics take a different route
/// (`scheduling::rt_executor::rt_diag`, a preallocated ring drained off-thread)
/// precisely because they cannot afford it.
fn write_console_line(line: &str) {
    use std::io::{self, Write};
    let mut err = io::stderr();
    let _ = err.write_all(line.as_bytes());
    let _ = err.flush();
}

/// The exact bytes [`emit_console`] writes, or `None` for a level that is not
/// printed.
///
/// Split out from the write so the format is testable without capturing a
/// process's stderr.
pub fn format_console_line(
    level: &LogType,
    node_name: &str,
    message: &str,
    raw_mode: bool,
) -> Option<String> {
    let tag = match level {
        LogType::Info => "\x1b[34m[INFO]\x1b[0m",
        LogType::Warning => "\x1b[33m[WARN]\x1b[0m",
        LogType::Error => "\x1b[31m[ERROR]\x1b[0m",
        LogType::Debug => "\x1b[90m[DEBUG]\x1b[0m",
        // Publish/Subscribe exist for the monitor and would drown a console at
        // tick rate.
        _ => return None,
    };
    let line_ending = if raw_mode { "\r\n" } else { "\n" };
    Some(format!(
        "{tag} \x1b[33m[{node_name}]\x1b[0m {message}{line_ending}"
    ))
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
/// # Filtering
///
/// A call below the active level costs one relaxed atomic load and a
/// not-taken branch: the level is checked **before** the format arguments are
/// evaluated, so a suppressed line pays for neither `format!` nor its
/// arguments' `Display` impls. The default level is
/// [`DEFAULT_LEVEL`](crate::core::hlog::DEFAULT_LEVEL) (`info`, i.e. `debug`
/// is off); change it with `HORUS_LOG_LEVEL=debug` or
/// [`set_max_level`](crate::core::hlog::set_max_level).
///
/// The corollary is that arguments with side effects are not evaluated when
/// the line is suppressed — which is the intended semantics of a level, and
/// the reason the check cannot live inside the logging function.
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
    // The level check is OUTSIDE `format!` in every arm. That placement is the
    // whole point: a check after the arguments have been formatted has already
    // paid for everything the filter exists to avoid.
    (info, $($arg:tt)*) => {
        if $crate::core::hlog::level_enabled($crate::core::hlog::LEVEL_INFO) {
            $crate::core::hlog::log_fmt($crate::core::LogType::Info, format_args!($($arg)*))
        }
    };
    (warn, $($arg:tt)*) => {
        if $crate::core::hlog::level_enabled($crate::core::hlog::LEVEL_WARN) {
            $crate::core::hlog::log_fmt($crate::core::LogType::Warning, format_args!($($arg)*))
        }
    };
    (error, $($arg:tt)*) => {
        if $crate::core::hlog::level_enabled($crate::core::hlog::LEVEL_ERROR) {
            $crate::core::hlog::log_fmt($crate::core::LogType::Error, format_args!($($arg)*))
        }
    };
    (debug, $($arg:tt)*) => {
        if $crate::core::hlog::level_enabled($crate::core::hlog::LEVEL_DEBUG) {
            $crate::core::hlog::log_fmt($crate::core::LogType::Debug, format_args!($($arg)*))
        }
    };
    // Failure arm. Without it, forgetting the level produced:
    //
    //     error: no rules expected `"pos=({:.2}, {:.2})"`
    //     note: while trying to match `info`
    //
    // which points at the format string and blames a token the author never
    // wrote. Nine of the ten shipped examples called `hlog!` this way, so the
    // first thing a reader compiled failed with an error that did not name the
    // mistake.
    ($($rest:tt)*) => {
        compile_error!(concat!(
            "hlog! needs a level as its first argument.\n",
            "Expected:  hlog!(info, \"message {}\", value)\n",
            "  - levels are: info, warn, error, debug\n",
            "  - the level is a bare word, not a string, and is followed by a comma"
        ));
    };
}

/// Log an [`Error`](crate::error::Error) with its remediation hint (if any).
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
    // Ahead of the `Display` impls and the `format!`: an error's `Display` can
    // be expensive (it walks a chain of sources) and its `help()` allocates.
    if !level_enabled(LEVEL_ERROR) {
        return;
    }
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
        // Level first, ahead of the clock read: `SystemTime::now()` is a vDSO
        // call, and paying it to decide whether to skip a line that is disabled
        // anyway is the same mistake as formatting first.
        if $crate::core::hlog::level_enabled($crate::__hlog_level_const!($level)) {
            let now_ms = ::std::time::SystemTime::now()
                .duration_since(::std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64;
            let last = LAST_LOG_MS.load(Ordering::Relaxed);
            if now_ms.saturating_sub(last) >= ($interval_ms as u64) {
                LAST_LOG_MS.store(now_ms, Ordering::Relaxed);
                $crate::hlog!($level, $($arg)*);
            }
        }
    }};
    // Same failure arm as `hlog!`, for the same reason: without it, omitting
    // the level pointed at the format string and blamed a token the author
    // never wrote. Two of the shipped examples called it this way.
    ($($rest:tt)*) => {
        compile_error!(concat!(
            "hlog_every! needs an interval and a level.\n",
            "Expected:  hlog_every!(500, info, \"message {}\", value)\n",
            "  - the first argument is the interval in milliseconds\n",
            "  - the second is the level: info, warn, error, debug\n",
            "  - the level is a bare word, not a string"
        ));
    };
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
        // Short-circuit `&&`: when the level is disabled the swap does not
        // happen, so raising the level later still lets the line through once.
        // Consuming the token while suppressed would silently spend it.
        if $crate::core::hlog::level_enabled($crate::__hlog_level_const!($level))
            && !LOGGED.swap(true, Ordering::Relaxed)
        {
            $crate::hlog!($level, $($arg)*);
        }
    }};
}

#[cfg(test)]
mod tests {
    // ── Console emission ────────────────────────────────────────────────
    //
    // A C++ node's `horus::log::info(...)` went to the shared-memory buffer and
    // nowhere else, so the first log statement a C++ user writes produced no
    // output in the terminal they were watching. `horus log` had it all along;
    // nothing pointed there. Rust and Python nodes have always printed.

    #[test]
    fn console_line_carries_the_level_and_the_node() {
        let line = super::format_console_line(
            &crate::core::log_buffer::LogType::Info,
            "controller",
            "Published cmd_vel",
            false,
        )
        .expect("Info must be printed");
        assert!(line.contains("[INFO]"), "{line:?}");
        assert!(line.contains("controller"), "{line:?}");
        assert!(line.contains("Published cmd_vel"), "{line:?}");
        assert!(line.ends_with('\n'), "{line:?}");
    }

    /// A TUI in raw mode has no implicit carriage return, so a bare `\n`
    /// staircases the output.
    #[test]
    fn raw_mode_uses_crlf() {
        let line =
            super::format_console_line(&crate::core::log_buffer::LogType::Warning, "n", "m", true)
                .expect("Warning must be printed");
        assert!(line.ends_with("\r\n"), "{line:?}");
    }

    /// Publish/Subscribe entries exist for the monitor. Printing them would
    /// drown a console at tick rate.
    #[test]
    fn transport_entries_are_not_printed() {
        assert!(super::format_console_line(
            &crate::core::log_buffer::LogType::Publish,
            "n",
            "m",
            false
        )
        .is_none());
    }

    /// Every level a user can produce must reach the terminal.
    #[test]
    fn every_user_level_is_printed() {
        use crate::core::log_buffer::LogType;
        for (level, tag) in [
            (LogType::Info, "[INFO]"),
            (LogType::Warning, "[WARN]"),
            (LogType::Error, "[ERROR]"),
            (LogType::Debug, "[DEBUG]"),
        ] {
            let line = super::format_console_line(&level, "n", "m", false)
                .unwrap_or_else(|| panic!("{level:?} must be printed"));
            assert!(line.contains(tag), "{level:?} -> {line:?}");
        }
    }

    use super::*;

    // ── Level filter ────────────────────────────────────────────────────
    //
    // `hlog!` had no filter at all: every arm ran `format!` unconditionally and
    // then paid a timestamp, three owned `String`s, a `bincode` write behind
    // the log buffer's process-global mutex, and an unbuffered blocking
    // `write(2)` to stderr. A `hlog!(debug, ...)` in a 1 kHz tick cost the full
    // several microseconds whether or not anyone would read it.

    /// Serialises the tests that move the process-global level.
    ///
    /// They only ever narrow as far as `LEVEL_INFO`, the default, so they
    /// cannot suppress the `info` lines the buffer tests above depend on.
    static LEVEL_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// The ordering of the level constants is asserted at compile time next to
    /// their definitions; this pins the *default*, which is a product decision.
    #[test]
    fn nothing_is_suppressed_by_default() {
        // Pinned deliberately. `hlog_ignores_log_bridge_level` and two other
        // integration tests assert that a Debug entry always reaches the shared
        // buffer, and document "should never be silently dropped" as the
        // intent. Suppression is therefore opt-in; see `DEFAULT_LEVEL`.
        assert_eq!(
            DEFAULT_LEVEL, LEVEL_DEBUG,
            "the default must not drop a level the logging contract promises to keep"
        );
    }

    #[test]
    fn unrecognised_level_names_do_not_silently_disable_logging() {
        assert_eq!(parse_level("warn"), Some(LEVEL_WARN));
        assert_eq!(parse_level("  DEBUG "), Some(LEVEL_DEBUG));
        assert_eq!(parse_level("off"), Some(LEVEL_OFF));
        // A typo must fall back to the default, never to OFF: silently logging
        // nothing is the worst possible response to a misspelled level.
        assert_eq!(parse_level("verbose"), None);
        assert_eq!(parse_level(""), None);
    }

    /// Transport entries are the monitor's, not an operator's log level, so the
    /// filter must not gate them.
    #[test]
    fn transport_entries_bypass_the_level_filter() {
        let _guard = LEVEL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let previous = max_level();
        set_max_level(LEVEL_INFO);
        assert!(log_type_enabled(&LogType::Publish));
        assert!(log_type_enabled(&LogType::Subscribe));
        assert!(!log_type_enabled(&LogType::Debug));
        assert!(log_type_enabled(&LogType::Error));
        set_max_level(previous);
    }

    /// **The** property of the filter: a suppressed line does not evaluate its
    /// arguments.
    ///
    /// A level check placed after `format!` has already run buys nothing — the
    /// allocation and every argument's `Display` impl are the expensive part.
    /// This asserts the check is where it has to be, in the macro.
    #[test]
    fn a_suppressed_line_does_not_evaluate_its_arguments() {
        let _guard = LEVEL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let previous = max_level();
        set_max_level(LEVEL_INFO);

        let evaluated = std::cell::Cell::new(0u32);
        let expensive = || {
            evaluated.set(evaluated.get() + 1);
            "payload"
        };

        crate::hlog!(debug, "suppressed {}", expensive());
        assert_eq!(
            evaluated.get(),
            0,
            "a debug line below the active level must not evaluate its arguments"
        );

        crate::hlog!(info, "emitted {}", expensive());
        assert_eq!(
            evaluated.get(),
            1,
            "a line at or above the active level must still be logged"
        );

        set_max_level(previous);
    }

    /// Raising the level makes suppressed lines appear — the filter hides
    /// output, it does not disable the call site.
    #[test]
    fn raising_the_level_lets_debug_through() {
        use crate::core::log_buffer::GLOBAL_LOG_BUFFER;
        let _guard = LEVEL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let previous = max_level();

        let count = || {
            GLOBAL_LOG_BUFFER
                .get_all()
                .iter()
                .filter(|e| e.message.contains("level_gate_9920"))
                .count()
        };
        let before = count();

        set_max_level(LEVEL_INFO);
        crate::hlog!(debug, "level_gate_9920 suppressed");
        assert_eq!(count(), before, "debug must not reach the buffer at info");

        set_max_level(LEVEL_DEBUG);
        crate::hlog!(debug, "level_gate_9920 emitted");
        assert_eq!(count(), before + 1, "debug must reach the buffer at debug");

        set_max_level(previous);
    }

    /// `hlog_once!` must not spend its one shot on a suppressed call.
    ///
    /// Otherwise raising the level later would never show the line — the token
    /// was consumed by a call that produced nothing.
    #[test]
    fn hlog_once_does_not_spend_its_token_while_suppressed() {
        use crate::core::log_buffer::GLOBAL_LOG_BUFFER;
        let _guard = LEVEL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let previous = max_level();

        // One closure, therefore one expansion, therefore one `static` — the
        // same call site on every invocation.
        let emit = || crate::hlog_once!(debug, "once_gate_9921");
        let count = || {
            GLOBAL_LOG_BUFFER
                .get_all()
                .iter()
                .filter(|e| e.message.contains("once_gate_9921"))
                .count()
        };
        let before = count();

        set_max_level(LEVEL_INFO);
        for _ in 0..5 {
            emit();
        }
        assert_eq!(count(), before, "suppressed calls must produce nothing");

        set_max_level(LEVEL_DEBUG);
        emit();
        assert_eq!(
            count(),
            before + 1,
            "the once-token must survive the suppressed calls"
        );

        set_max_level(previous);
    }

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

        const INTERVAL_MS: u128 = 200;
        const BURST: u32 = 64;

        // One closure means one macro expansion, so every call below shares the
        // single per-callsite `static` the throttle keys on.
        let emit = || crate::hlog_every!(200, info, "every_throttle_9905");
        // `saturating_sub`: the log buffer is a fixed-size ring shared across the
        // whole suite, so marker entries can be evicted between two reads and a
        // plain subtraction would panic with an unsigned overflow.
        let count_now = || {
            GLOBAL_LOG_BUFFER
                .get_all()
                .iter()
                .filter(|e| e.message.contains("every_throttle_9905"))
                .count()
        };

        // Phase 1 — it suppresses. A tight burst with NO sleep: consecutive calls
        // are microseconds apart on any machine.
        //
        // This used to be a 500ms loop paced by `sleep(5)`, asserting
        // `count < calls`. That silently assumed the loop calls faster than the
        // throttle throttles. Under load the sleep stretched past 200ms, the loop
        // managed 2 iterations, and both were legitimately due — a CORRECT
        // throttle reported as "it is not throttling". A burst cannot degenerate
        // that way, and the guard below refuses to pass vacuously if it somehow does.
        let before = count_now();
        let t0 = std::time::Instant::now();
        for _ in 0..BURST {
            emit();
        }
        let burst_ms = t0.elapsed().as_millis();
        let count = count_now().saturating_sub(before) as u128;

        let most = burst_ms / INTERVAL_MS + 1;
        assert!(
            most < u128::from(BURST),
            "the burst took {burst_ms}ms, so {most} emissions would be legal out of \
             {BURST} calls — this machine is too starved for the measurement to mean \
             anything, rather than the throttle being broken"
        );
        assert!(
            count >= 1,
            "hlog_every! emitted nothing for the first call, which must always fire"
        );
        assert!(
            count <= most,
            "hlog_every!({INTERVAL_MS}ms) emitted {count} entries for {BURST} calls in \
             {burst_ms}ms — at most {most} are possible if it is throttling"
        );

        // Phase 2 — it resumes. Oversleeping only widens the gap, so load can only
        // make this more true; the gap is measured rather than assumed.
        let mark = std::time::Instant::now();
        std::thread::sleep(std::time::Duration::from_millis(INTERVAL_MS as u64 + 50));
        assert!(mark.elapsed().as_millis() >= INTERVAL_MS);
        let before2 = count_now();
        emit();
        assert_eq!(
            count_now().saturating_sub(before2),
            1,
            "a call a full interval after the last emission must fire again — the \
             throttle suppresses, it must not latch off"
        );
    }
}
