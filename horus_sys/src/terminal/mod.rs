//! Terminal control — raw mode detection, ANSI color support.
//!
//! - **Unix**: `tcgetattr()` for raw mode, ANSI natively supported
//! - **Windows**: `GetConsoleMode()` + `SetConsoleMode(ENABLE_VIRTUAL_TERMINAL_PROCESSING)`

/// Check if terminal raw mode is currently enabled.
///
/// - Unix: checks if OPOST or ICANON flags are disabled via `tcgetattr()`
/// - Windows: checks if ENABLE_PROCESSED_INPUT is disabled via `GetConsoleMode()`
pub fn is_raw_mode() -> bool {
    #[cfg(unix)]
    {
        unix_is_raw_mode()
    }
    #[cfg(windows)]
    {
        windows_is_raw_mode()
    }
    #[cfg(not(any(unix, windows)))]
    {
        false
    }
}

/// Print a line to stdout, using `\r\n` if in raw terminal mode.
///
/// Use this instead of `println!` anywhere output can happen from a robot.
/// `println!` panics when the write fails, and a robot's stdout fails
/// routinely: the supervisor that launched it exits, or an operator pipes
/// `horus run` into `head`. On a safety path that panic can skip the action it
/// was announcing; on a background thread it unwinds that thread alone and
/// leaves the process running with a subsystem silently dead. Losing the
/// message is always the better failure.
#[inline]
pub fn print_line(msg: &str) {
    use std::io::Write;
    let mut out = std::io::stdout();
    let _ = write_line(&mut out, msg, is_raw_mode());
    let _ = out.flush();
}

/// Print a line to stderr, using `\r\n` if in raw terminal mode.
///
/// The stderr twin of [`print_line`], for the same reasons.
#[inline]
pub fn eprint_line(msg: &str) {
    use std::io::Write;
    let mut err = std::io::stderr();
    let _ = write_line(&mut err, msg, is_raw_mode());
    let _ = err.flush();
}

/// Write a single line to `out`, honouring raw-mode CRLF. Returns the write
/// error (rather than panicking like `println!`) so callers on safety paths can
/// swallow a broken stream instead of unwinding.
#[inline]
fn write_line<W: std::io::Write>(out: &mut W, msg: &str, raw_mode: bool) -> std::io::Result<()> {
    if raw_mode {
        write!(out, "{}\r\n", msg)
    } else {
        writeln!(out, "{}", msg)
    }
}

/// Check if the terminal supports ANSI color sequences.
///
/// - Unix: always true (ANSI supported natively)
/// - Windows: true if virtual terminal processing is enabled
pub fn supports_color() -> bool {
    #[cfg(unix)]
    {
        // Check if stdout is a terminal
        std::io::IsTerminal::is_terminal(&std::io::stdout())
    }
    #[cfg(windows)]
    {
        // Windows 10+ supports ANSI via VT processing
        std::io::IsTerminal::is_terminal(&std::io::stdout())
    }
    #[cfg(not(any(unix, windows)))]
    {
        false
    }
}

#[cfg(unix)]
fn unix_is_raw_mode() -> bool {
    use std::os::unix::io::AsRawFd;

    if !std::io::IsTerminal::is_terminal(&std::io::stdout()) {
        return false;
    }

    // SAFETY: stdout fd is valid; termios is zeroed C struct; tcgetattr reads into it
    unsafe {
        let fd = std::io::stdout().as_raw_fd();
        let mut termios: libc::termios = std::mem::zeroed();

        if libc::tcgetattr(fd, &mut termios) != 0 {
            return false;
        }

        let opost_disabled = (termios.c_oflag & libc::OPOST) == 0;
        let icanon_disabled = (termios.c_lflag & libc::ICANON) == 0;

        opost_disabled || icanon_disabled
    }
}

#[cfg(windows)]
fn windows_is_raw_mode() -> bool {
    use windows_sys::Win32::System::Console::{
        GetConsoleMode, GetStdHandle, ENABLE_PROCESSED_INPUT, STD_INPUT_HANDLE,
    };

    // SAFETY: GetStdHandle and GetConsoleMode are always safe to call
    unsafe {
        let handle = GetStdHandle(STD_INPUT_HANDLE);
        let mut mode = 0u32;
        if GetConsoleMode(handle, &mut mode) == 0 {
            return false;
        }
        // If ENABLE_PROCESSED_INPUT is disabled, we're in raw mode
        (mode & ENABLE_PROCESSED_INPUT) == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_raw_mode_returns_bool() {
        // In test environment (no tty), should return false
        let result = is_raw_mode();
        assert!(!result, "Should not be in raw mode during tests");
    }

    #[test]
    fn is_raw_mode_is_consistent() {
        assert_eq!(is_raw_mode(), is_raw_mode());
    }

    /// Regression: the write path must RETURN a broken-stream error, never panic
    /// like `println!` does. These are called from emergency-stop logging, so a
    /// write failure (EPIPE/ENOSPC) must be swallowed rather than turned into a
    /// panic that skips the stop action.
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

    #[test]
    fn print_helpers_do_not_panic() {
        print_line("");
        print_line("line with special chars: \t\x1b[0m");
        eprint_line("stderr line");
    }

    #[test]
    fn supports_color_returns_bool() {
        let _ = supports_color(); // smoke test
    }
}
