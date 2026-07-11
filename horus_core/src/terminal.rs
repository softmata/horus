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
    use std::io::Write;
    // Non-fatal writes: print_line is called from safety-critical paths (e.g. the
    // scheduler/RT emergency-stop arms log via print_line BEFORE latching the stop
    // flag). print!/println! panic on a stdout write error (EPIPE from a closed
    // pipe, ENOSPC on a full disk), which — even when caught by the RT loop's
    // catch_unwind — would skip the stop action and downgrade a system e-stop to a
    // single-node stop. write!/writeln! return the error instead, which we swallow.
    let mut out = std::io::stdout();
    let _ = write_line(&mut out, msg, is_raw_mode());
    let _ = out.flush();
}

/// Write a single line to `out`, honouring raw-mode CRLF. Returns the write error
/// (rather than panicking like `println!`) so callers on safety paths can swallow
/// a broken stdout instead of unwinding.
#[inline]
fn write_line<W: std::io::Write>(out: &mut W, msg: &str, raw_mode: bool) -> std::io::Result<()> {
    if raw_mode {
        write!(out, "{}\r\n", msg)
    } else {
        writeln!(out, "{}", msg)
    }
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

    /// Regression: the write path must RETURN a broken-stdout error, never panic
    /// like println! does. print_line is called from emergency-stop logging, so a
    /// write failure (EPIPE/ENOSPC) must be swallowed, not turned into a panic that
    /// skips the stop action.
    #[test]
    fn write_line_returns_error_instead_of_panicking() {
        struct FailWriter;
        impl std::io::Write for FailWriter {
            fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
                Err(std::io::Error::from(std::io::ErrorKind::BrokenPipe))
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Err(std::io::Error::from(std::io::ErrorKind::BrokenPipe))
            }
        }
        let mut w = FailWriter;
        assert!(write_line(&mut w, "e-stop message", false).is_err());
        assert!(write_line(&mut w, "raw-mode message", true).is_err());
    }
}
