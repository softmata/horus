//! Terminal utilities for HORUS.
//!
//! Output that is safe to emit from a robot. When a terminal is in raw mode
//! (e.g. for keyboard input) a bare `\n` does not carry a carriage return and
//! output staircases; more importantly, `println!`/`eprintln!` **panic** when
//! the write fails, and a robot's console fails routinely — the supervisor that
//! launched it exits, an operator pipes `horus run` into `head`, a disk fills.
//!
//! These helpers handle both: raw-mode CRLF, and swallowing the write error
//! instead of unwinding.
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_core::terminal::{eprint_line, print_line};
//!
//! print_line("This displays correctly in raw mode and survives a closed pipe");
//! eprint_line("Diagnostics go to stderr, same guarantees");
//! ```

/// Check if terminal raw mode is currently enabled.
///
/// Delegates to [`horus_sys::terminal::is_raw_mode()`].
pub use horus_sys::terminal::is_raw_mode;

/// Print a line to stdout, using `\r\n` if in raw terminal mode.
///
/// Use instead of `println!`: it does not panic when the write fails. The
/// scheduler and RT emergency-stop paths log through this **before** latching
/// the stop flag, so a panic here would downgrade a system e-stop to a single
/// node stopping.
pub use horus_sys::terminal::print_line;

/// Print a line to stderr, using `\r\n` if in raw terminal mode.
///
/// Use instead of `eprintln!`: on a background thread (the network replicator,
/// a node's worker) a print panic unwinds that thread alone and leaves the
/// process running with the subsystem silently dead.
pub use horus_sys::terminal::eprint_line;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raw_mode_detection() {
        assert!(!is_raw_mode());
        assert_eq!(is_raw_mode(), is_raw_mode());
    }

    #[test]
    fn test_print_line_does_not_panic() {
        print_line("");
        print_line("hello from test");
        print_line("line with special chars: \t\x1b[0m");
    }

    #[test]
    fn test_eprint_line_does_not_panic() {
        eprint_line("");
        eprint_line("hello from test");
    }
}
