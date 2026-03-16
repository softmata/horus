//! Terminal utilities for HORUS.
//!
//! This module provides utilities for terminal output that correctly handles
//! raw terminal mode. When a terminal is in raw mode (e.g., for keyboard input),
//! newlines (`\n`) don't automatically include carriage returns (`\r`), causing
//! a "staircase effect" in output.
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_core::terminal::print_line;
//!
//! // Use print_line instead of println! for correct output in raw mode
//! print_line("This will display correctly in raw mode");
//! ```

/// Check if terminal raw mode is currently enabled.
///
/// Delegates to [`horus_sys::terminal::is_raw_mode()`].
pub fn is_raw_mode() -> bool {
    horus_sys::terminal::is_raw_mode()
}

/// Print a line to stdout, using `\r\n` if in raw terminal mode.
///
/// This function should be used instead of `println!` when output might
/// occur while the terminal is in raw mode.
#[inline]
pub fn print_line(msg: &str) {
    if is_raw_mode() {
        print!("{}\r\n", msg);
    } else {
        println!("{}", msg);
    }
    use std::io::Write;
    let _ = std::io::stdout().flush();
}

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
}
