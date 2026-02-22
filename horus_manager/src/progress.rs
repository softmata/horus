//! Progress indicators for HORUS CLI
//!
//! Provides spinners and progress bars for long-running operations.

use console::style;
use indicatif::{ProgressBar, ProgressStyle};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

/// Global quiet mode flag
static QUIET_MODE: AtomicBool = AtomicBool::new(false);

/// Set global quiet mode
pub fn set_quiet(quiet: bool) {
    QUIET_MODE.store(quiet, Ordering::SeqCst);
}

/// Check if quiet mode is enabled
pub fn is_quiet() -> bool {
    QUIET_MODE.load(Ordering::SeqCst)
}

/// Alternative braille spinner
pub const BRAILLE_SPINNER: &[&str] = &["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"];

/// Success indicator
pub const STATUS_SUCCESS: &str = "[+]";

/// Warning indicator
pub const STATUS_WARNING: &str = "[!]";

/// Error indicator
pub const STATUS_ERROR: &str = "[-]";

/// Create a spinner for indefinite operations
pub fn spinner(message: &str) -> ProgressBar {
    if is_quiet() {
        return ProgressBar::hidden();
    }
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::default_spinner()
            .tick_strings(BRAILLE_SPINNER)
            .template("{spinner} {msg}")
            .expect("valid template"),
    );
    pb.set_message(message.to_string());
    pb.enable_steady_tick(Duration::from_millis(80));
    pb
}

/// Create a build spinner
pub fn build_spinner(message: &str) -> ProgressBar {
    spinner(message)
}

/// Create a download/install spinner
pub fn download_spinner(message: &str) -> ProgressBar {
    spinner(message)
}

/// Finish a spinner with success
pub fn finish_success(pb: &ProgressBar, message: &str) {
    pb.set_style(ProgressStyle::default_spinner().template("{msg}").expect("valid template"));
    pb.finish_with_message(format!("{} {}", STATUS_SUCCESS, style(message).green()));
}

/// Finish a spinner with error
pub fn finish_error(pb: &ProgressBar, message: &str) {
    pb.set_style(ProgressStyle::default_spinner().template("{msg}").expect("valid template"));
    pb.finish_with_message(format!("{} {}", STATUS_ERROR, style(message).red()));
}

/// Create a build spinner (legacy alias)
pub fn robot_build_spinner(message: &str) -> ProgressBar {
    build_spinner(message)
}

/// Create a download spinner (legacy alias)
pub fn robot_download_spinner(message: &str) -> ProgressBar {
    download_spinner(message)
}

/// Format bytes in a human-readable way
pub fn format_bytes(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;

    if bytes < KB {
        format!("{} B", bytes)
    } else if bytes < MB {
        format!("{:.1} KB", bytes as f64 / KB as f64)
    } else if bytes < GB {
        format!("{:.1} MB", bytes as f64 / MB as f64)
    } else {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_spinner_creation() {
        let pb = spinner("Testing...");
        thread::sleep(Duration::from_millis(500));
        finish_success(&pb, "Test complete!");
    }

    #[test]
    fn test_quiet_mode() {
        set_quiet(true);
        let pb = spinner("Should be hidden");
        assert!(pb.is_hidden());
        set_quiet(false);
    }

    #[test]
    fn test_format_bytes() {
        assert_eq!(format_bytes(500), "500 B");
        assert_eq!(format_bytes(1024), "1.0 KB");
        assert_eq!(format_bytes(1536), "1.5 KB");
        assert_eq!(format_bytes(1048576), "1.0 MB");
        assert_eq!(format_bytes(1073741824), "1.00 GB");
    }
}
