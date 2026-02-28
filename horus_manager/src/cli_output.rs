//! Standardized CLI output helpers for consistent HORUS CLI experience.
//!
//! All user-facing output should use these helpers instead of raw `println!`.

use colored::*;

pub const ICON_SUCCESS: &str = "\u{2713}"; // ✓
pub const ICON_ERROR: &str = "\u{2717}";   // ✗
pub const ICON_WARN: &str = "\u{26a0}";    // ⚠
pub const ICON_INFO: &str = "\u{25b6}";    // ▶
pub const ICON_HINT: &str = "\u{00b7}";    // ·

/// Print a success message: ✓ message
pub fn success(msg: &str) {
    println!("{} {}", ICON_SUCCESS.green(), msg);
}

/// Print an error message to stderr: ✗ message
pub fn error(msg: &str) {
    eprintln!("{} {}", ICON_ERROR.red(), msg);
}

/// Print a warning message: ⚠ message
pub fn warn(msg: &str) {
    println!("{} {}", ICON_WARN.yellow(), msg);
}

/// Print an info/action message: ▶ message
pub fn info(msg: &str) {
    println!("{} {}", ICON_INFO.cyan(), msg);
}

/// Print a dimmed hint: · message
pub fn hint(msg: &str) {
    println!("  {} {}", ICON_HINT.dimmed(), msg.dimmed());
}

/// Print a bold cyan header
pub fn header(msg: &str) {
    println!("{}", msg.cyan().bold());
}

/// Print an empty-state message with an optional tip
pub fn empty(msg: &str, tip: Option<&str>) {
    println!("{}", msg.yellow());
    if let Some(t) = tip {
        hint(t);
    }
}
