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
//! use horus_core::terminal::{set_raw_mode, terminal_println};
//!
//! // When enabling raw terminal mode
//! set_raw_mode(true);
//!
//! // Use terminal_println! instead of println! for correct output
//! terminal_println!("This will display correctly in raw mode");
//!
//! // When disabling raw terminal mode
//! set_raw_mode(false);
//! ```

use std::sync::atomic::{AtomicBool, Ordering};

/// Global flag indicating if the terminal is in raw mode.
/// When true, output functions should use `\r\n` instead of just `\n`.
static TERMINAL_RAW_MODE: AtomicBool = AtomicBool::new(false);

/// Set the terminal raw mode flag.
///
/// When set to `true`, the `terminal_print!` and `terminal_println!` macros
/// will output with `\r\n` line endings to prevent the staircase effect.
///
/// # Arguments
///
/// * `enabled` - Whether raw terminal mode is enabled
pub fn set_raw_mode(enabled: bool) {
    TERMINAL_RAW_MODE.store(enabled, Ordering::SeqCst);
}

/// Check if terminal raw mode is currently enabled.
///
/// This function first checks the process-local flag, then falls back to
/// actually probing the terminal state. This handles the case where another
/// process has enabled raw mode on the shared terminal.
pub fn is_raw_mode() -> bool {
    // First check the local flag (fast path)
    if TERMINAL_RAW_MODE.load(Ordering::SeqCst) {
        return true;
    }

    // Fall back to checking actual terminal state (handles cross-process raw mode)
    // This detects when another process has put the terminal in raw mode
    is_terminal_in_raw_mode()
}

/// Directly probe the terminal to check if it's in raw mode.
///
/// On Unix, this checks if ICRNL (map CR to NL) and OPOST (output processing)
/// are disabled, which indicates raw mode.
#[cfg(unix)]
fn is_terminal_in_raw_mode() -> bool {
    use std::os::unix::io::AsRawFd;

    // Only check if stdout is a tty
    if !std::io::IsTerminal::is_terminal(&std::io::stdout()) {
        return false;
    }

    // Use libc to get terminal attributes
    // SAFETY: stdout fd is valid; termios is zeroed C struct; tcgetattr reads into it
    unsafe {
        let fd = std::io::stdout().as_raw_fd();
        let mut termios: libc::termios = std::mem::zeroed();

        if libc::tcgetattr(fd, &mut termios) != 0 {
            return false; // Can't get attributes, assume not raw
        }

        // Raw mode typically has these flags disabled:
        // - ICANON: canonical mode (line editing)
        // - ECHO: echo input characters
        // - OPOST: output post-processing (converts \n to \r\n)
        // If OPOST is disabled, we need to use \r\n explicitly
        let opost_disabled = (termios.c_oflag & libc::OPOST) == 0;
        let icanon_disabled = (termios.c_lflag & libc::ICANON) == 0;

        // If either output processing or canonical mode is disabled, treat as raw
        opost_disabled || icanon_disabled
    }
}

/// On non-Unix platforms, fall back to the flag only.
#[cfg(not(unix))]
fn is_terminal_in_raw_mode() -> bool {
    false
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
    // Flush to ensure immediate output
    use std::io::Write;
    let _ = std::io::stdout().flush();
}

/// Print to stdout, using `\r\n` for newlines if in raw terminal mode.
#[inline]
pub fn print_str(msg: &str) {
    if is_raw_mode() {
        // Replace \n with \r\n for raw mode
        let fixed = msg.replace('\n', "\r\n");
        print!("{}", fixed);
    } else {
        print!("{}", msg);
    }
    use std::io::Write;
    let _ = std::io::stdout().flush();
}

/// A macro similar to `println!` that handles raw terminal mode correctly.
///
/// When the terminal is in raw mode, this macro will use `\r\n` line endings
/// instead of just `\n` to prevent the "staircase effect".
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::terminal_println;
///
/// terminal_println!("Hello, world!");
/// terminal_println!("Value: {}", 42);
/// ```
#[macro_export]
macro_rules! terminal_println {
    () => {
        $crate::terminal::print_line("")
    };
    ($($arg:tt)*) => {
        $crate::terminal::print_line(&format!($($arg)*))
    };
}

/// A macro similar to `print!` that handles raw terminal mode correctly.
///
/// When the terminal is in raw mode, this macro will replace `\n` with `\r\n`
/// to prevent the "staircase effect".
#[macro_export]
macro_rules! terminal_print {
    ($($arg:tt)*) => {
        $crate::terminal::print_str(&format!($($arg)*))
    };
}

/// Print a line to stderr, using `\r\n` if in raw terminal mode.
///
/// This function should be used instead of `eprintln!` when output might
/// occur while the terminal is in raw mode.
#[inline]
pub fn eprint_line(msg: &str) {
    use std::io::Write;
    if is_raw_mode() {
        let _ = write!(std::io::stderr(), "{}\r\n", msg);
    } else {
        eprintln!("{}", msg);
    }
    let _ = std::io::stderr().flush();
}

/// A macro similar to `eprintln!` that handles raw terminal mode correctly.
///
/// When the terminal is in raw mode, this macro will use `\r\n` line endings
/// instead of just `\n` to prevent the "staircase effect".
#[macro_export]
macro_rules! terminal_eprintln {
    () => {
        $crate::terminal::eprint_line("")
    };
    ($($arg:tt)*) => {
        $crate::terminal::eprint_line(&format!($($arg)*))
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raw_mode_flag() {
        // Default should be false
        assert!(!is_raw_mode());

        // Set to true
        set_raw_mode(true);
        assert!(is_raw_mode());

        // Set back to false
        set_raw_mode(false);
        assert!(!is_raw_mode());
    }
}
