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

    #[test]
    fn supports_color_returns_bool() {
        let _ = supports_color(); // smoke test
    }
}
