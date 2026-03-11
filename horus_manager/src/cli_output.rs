//! Standardized CLI output helpers for consistent HORUS CLI experience.
//!
//! All user-facing output should use these helpers instead of raw `println!`.
//! Informational helpers (`info`, `success`, `hint`, `header`) respect quiet
//! mode set via [`crate::progress::set_quiet`]. Errors and warnings always print.

use colored::*;

pub const ICON_SUCCESS: &str = "\u{2713}"; // ✓
pub const ICON_ERROR: &str = "\u{2717}"; // ✗
pub const ICON_WARN: &str = "\u{26a0}"; // ⚠
pub const ICON_INFO: &str = "\u{25b6}"; // ▶
pub const ICON_HINT: &str = "\u{00b7}"; // ·

/// Print a success message: ✓ message (suppressed in quiet mode)
pub fn success(msg: &str) {
    if !crate::progress::is_quiet() {
        println!("{} {}", ICON_SUCCESS.green(), msg);
    }
}

/// Print an error message to stderr: ✗ message (always prints)
pub fn error(msg: &str) {
    eprintln!("{} {}", ICON_ERROR.red(), msg);
}

/// Print a warning message: ⚠ message (always prints)
pub fn warn(msg: &str) {
    eprintln!("{} {}", ICON_WARN.yellow(), msg);
}

/// Print an info/action message: ▶ message (suppressed in quiet mode)
pub fn info(msg: &str) {
    if !crate::progress::is_quiet() {
        eprintln!("{} {}", ICON_INFO.cyan(), msg);
    }
}

/// Print a dimmed hint: · message (suppressed in quiet mode)
pub fn hint(msg: &str) {
    if !crate::progress::is_quiet() {
        println!("  {} {}", ICON_HINT.dimmed(), msg.dimmed());
    }
}

/// Print a bold cyan header (suppressed in quiet mode)
pub fn header(msg: &str) {
    if !crate::progress::is_quiet() {
        println!("{}", msg.cyan().bold());
    }
}

/// Print an empty-state message with an optional tip
pub fn empty(msg: &str, tip: Option<&str>) {
    println!("{}", msg.yellow());
    if let Some(t) = tip {
        hint(t);
    }
}

/// Truncate a string to at most `max_bytes` bytes, respecting UTF-8 char boundaries.
/// Appends "..." if truncated.
pub fn safe_truncate(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        return s.to_string();
    }
    let suffix = "...";
    let target = max_len.saturating_sub(suffix.len());
    // Walk back to nearest char boundary
    let mut end = target;
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    format!("{}{}", &s[..end], suffix)
}
